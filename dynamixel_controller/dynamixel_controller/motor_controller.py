#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from dynamixel_handler_msgs.msg import DynamixelControlXPosition, DynamixelControlXCurrent, DynamixelPresent
from happymimi_manipulation2_msgs.srv import ArmPose, SetArmPose
import yaml

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        yaml_file_path = '/home/daniil/main_ws/src/happy_manipulation/dynamixel_controller/config/mimi_specification.yaml'
        self.get_logger().info(f"Using fixed yaml path: {yaml_file_path}")

        try:
            with open(yaml_file_path, 'r') as file:
                config = yaml.safe_load(file)

            self.origin_angle_from_yaml = config['Origin_Angle']
            self.gear_ratio_from_yaml = config['Gear_Ratio']
            self.start_pose_from_yaml = config['Start_Pose']

            self.poses_from_yaml = {k: config.get(k, [0, 0, 0]) for k in [
                "Origin_Pose", "Carry_Pose", "Recieve_Pose", "Give_Pose", "Point_Pose",
                "Look_Floor_Pose"
            ]}
            self.get_logger().info("YAML file loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML file: {str(e)}")
            self.origin_angle_from_yaml = [2048] * 5
            self.gear_ratio_from_yaml = 1.0
            self.start_pose_from_yaml = 'Look_Floor_Pose'
            self.poses_from_yaml = {}

        self.origin_angle = self.origin_angle_from_yaml
        self.gear_ratio = self.gear_ratio_from_yaml

        self.base_velocity = 80.0
        self.base_acceleration = 25.0

        self.current_joint_angles = {
            i: math.degrees(self.step_to_rad(self.origin_angle[i])) if i < len(self.origin_angle) else 0.0
            for i in range(5)
        }

        self.ee_actual_deg = None  # End effector actual degree from state

        self.position_pub = self.create_publisher(DynamixelControlXPosition, '/dynamixel/command/x/position_control', 10)
        self.current_pub = self.create_publisher(DynamixelControlXCurrent, '/dynamixel/command/x/current_control', 10)
        self.grasp_state_pub = self.create_publisher(Bool, '/servo/object_grasped', 10)

        self.create_service(ArmPose, '/servo/arm', self.handle_preset_pose)
        self.create_service(SetArmPose, '/servo/set_arm', self.handle_custom_pose)

        self.create_subscription(Float64, '/servo/shoulder', self.cb_shoulder_topic, 10)
        self.create_subscription(Float64, '/servo/elbow', self.cb_elbow_topic, 10)
        self.create_subscription(Float64, '/servo/wrist', self.cb_wrist_topic, 10)
        self.create_subscription(Bool, '/servo/endeffector', self.cb_endeffector_topic, 10)
        self.create_subscription(Float64, '/servo/head', self.cb_head_topic, 10)
        self.create_subscription(DynamixelPresent, '/dynamixel/state/present', self.cb_present_state, 10)

        self.move_to_start_pose()

    def cb_present_state(self, msg):
        try:
            if 4 in msg.id_list:
                idx = msg.id_list.index(4)
                self.ee_actual_deg = msg.position_deg[idx]
        except Exception as e:
            self.get_logger().warn(f"Failed to extract end effector angle: {e}")

    def deg_to_rad(self, deg): return math.radians(deg)
    def step_to_rad(self, step): return (step - 2048) / 2048.0 * math.pi
    def rad_to_step(self, rad): return int(rad / math.pi * 2048 + 2048)

    def calculate_movement_time(self, distance, velocity, acceleration):
        if distance == 0: return 0
        max_reachable_velocity = math.sqrt(distance * acceleration)
        if max_reachable_velocity <= velocity:
            return 2 * math.sqrt(distance / acceleration)
        accel_time = velocity / acceleration
        constant_velocity_distance = distance - acceleration * accel_time ** 2
        constant_velocity_time = constant_velocity_distance / velocity
        return 2 * accel_time + constant_velocity_time

    def calculate_velocity_for_distance_and_time(self, distance, time):
        if time <= 0 or distance == 0:
            return (self.base_velocity, self.base_acceleration)
        a = 4 * distance / (time * time)
        v = a * time / 2
        return (max(v, 5.0), max(a, 50.0))

    def calculate_synchronized_parameters(self, target_angles, current_angles, joint_ids):
        distances = [abs(t - c) for t, c in zip(target_angles, current_angles)]
        movement_times = []
        for i, d in enumerate(distances):
            joint_id = joint_ids[i]
            t = self.calculate_movement_time(d, self.base_velocity, self.base_acceleration)
            if joint_id in [0, 1]:
                t *= 0.5
            movement_times.append(t)
        max_time = max(movement_times)
        return [self.calculate_velocity_for_distance_and_time(d, max_time)[0] for d in distances], \
               [self.calculate_velocity_for_distance_and_time(d, max_time)[1] for d in distances]

    def send_position_command(self, ids, target_degrees):
        current_degrees = [self.current_joint_angles.get(jid, 0.0) for jid in ids]
        velocities, accelerations = self.calculate_synchronized_parameters(target_degrees, current_degrees, ids)
        msg = DynamixelControlXPosition()
        msg.id_list = ids
        msg.position_deg = list(target_degrees)
        msg.profile_vel_deg_s = list(velocities)
        msg.profile_acc_deg_ss = list(accelerations)
        for i, jid in enumerate(ids):
            self.current_joint_angles[jid] = target_degrees[i]
        self.position_pub.publish(msg)

    def cb_shoulder_topic(self, msg): self.control_shoulder(msg.data)
    def cb_elbow_topic(self, msg): self.control_elbow(msg.data)
    def cb_wrist_topic(self, msg): self.control_wrist(msg.data)
    def cb_endeffector_topic(self, msg): self.control_endeffector(msg.data)
    def cb_head_topic(self, msg): self.control_head(msg.data)

    def control_shoulder(self, angle_deg):
        deg = angle_deg * 2
        l = -deg
        r = deg
        m0 = math.degrees(self.step_to_rad(self.origin_angle[0]) + math.radians(l))
        m1 = math.degrees(self.step_to_rad(self.origin_angle[1]) + math.radians(r))
        self.send_position_command([0, 1], [m0, m1])

    def control_elbow(self, angle_deg):
        rad = self.step_to_rad(self.origin_angle[2]) + math.radians(angle_deg)
        self.send_position_command([2], [math.degrees(rad)])

    def control_wrist(self, angle_deg):
        rad = self.step_to_rad(self.origin_angle[3]) + math.radians(angle_deg)
        self.send_position_command([3], [math.degrees(rad)])

    def control_endeffector(self, close):
        if close:
            current_msg = DynamixelControlXCurrent()
            current_msg.id_list = [4]
            current_msg.current_ma = [240.0]
            self.current_pub.publish(current_msg)

            actual_deg = self.ee_actual_deg
            if actual_deg is not None:
                grasp_msg = Bool()
                print(f"actual_deg{actual_deg}")
                grasp_msg.data = actual_deg < -87.0  # -85 = fully closed, -148 = fully open
                self.grasp_state_pub.publish(grasp_msg)
            else:
                self.get_logger().warn(f"Failed: {e}")
        else:
            offset = -30.0
            target_rad = self.step_to_rad(self.origin_angle[4]) + math.radians(offset)
            target_deg = math.degrees(target_rad)

            msg = DynamixelControlXPosition()
            msg.id_list = [4]
            msg.position_deg = [target_deg]
            msg.profile_vel_deg_s = [20.0]
            msg.profile_acc_deg_ss = [100.0]
            self.position_pub.publish(msg)

            self.current_joint_angles[4] = target_deg

    def control_head(self, angle_deg):
        try:
            target_rad = self.step_to_rad(self.origin_angle[5]) + math.radians(angle_deg)
            target_deg = math.degrees(target_rad)

            msg = DynamixelControlXPosition()
            msg.id_list = [5]
            msg.position_deg = [target_deg]
            msg.profile_vel_deg_s = [40.0]
            msg.profile_acc_deg_ss = [100.0]
            self.position_pub.publish(msg)

            self.current_joint_angles[5] = target_deg
        except IndexError:
            self.get_logger().warn(f"Failed: {e}")

    def send_pose(self, pose_deg_list):
        o = [self.step_to_rad(self.origin_angle[i]) for i in range(4)]
        d0 = math.degrees(o[0] + math.radians(-pose_deg_list[0] * 2))
        d1 = math.degrees(o[1] + math.radians(pose_deg_list[0] * 2))
        d2 = math.degrees(o[2] + math.radians(pose_deg_list[1]))
        d3 = math.degrees(o[3] + math.radians(pose_deg_list[2]))
        self.send_position_command([0, 1, 2, 3], [d0, d1, d2, d3])

    def handle_preset_pose(self, request, response):
        name = request.name.strip()
        if name in self.poses_from_yaml:
            pose = self.poses_from_yaml[name]

            if len(pose) == 4:  # Give または Recieve
                self.send_pose(pose[:3])  # 肩・肘・手首を動かす
                self.control_endeffector(pose[3])  # endeffector を閉じる or 開く

                response.success = True
                response.message = f"Pose '{name}' executed with delayed transition."

                time.sleep(6.0)
                self.control_endeffector(not pose[3])
                time.sleep(2.0)
                if 'Carry_Pose' in self.poses_from_yaml:
                    carry_pose = self.poses_from_yaml['Carry_Pose']
                    self.send_pose(carry_pose[:3])
                else:
                    self.get_logger().warn(f"Failed: {e}")

            elif len(pose) == 3:
                self.send_pose(pose)
                response.success = True
                response.message = f"Pose '{name}' executed."
            else:
                response.success = False
                response.message = f"Pose '{name}' is incomplete."
        else:
            response.success = False
            response.message = f"Pose '{name}' not found."
        return response

    def handle_custom_pose(self, request, response):
        try:
            self.control_shoulder(request.shoulder)
            self.control_elbow(request.elbow)
            self.control_wrist(request.wrist)
            self.control_endeffector(request.endeffector)
            response.success = True
            response.message = "Custom arm pose applied."
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def move_to_start_pose(self):
        name = self.start_pose_from_yaml
        if name in self.poses_from_yaml:
            pose = self.poses_from_yaml[name]
            if len(pose) == 3:
                time.sleep(0.5)
                self.send_pose(pose)

        time.sleep(1.0)  # ← 1秒待機（状況に応じて0.5〜2.0秒で調整）
        self.control_head(0.0)


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()