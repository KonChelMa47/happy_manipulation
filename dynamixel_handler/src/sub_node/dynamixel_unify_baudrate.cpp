#include "dynamixel_communicator.h"
#include <vector>
using std::vector;
#include <string>
using std::string;

#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
	// *** init node
	std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("dynamixel_unify_baudrate_node");


    int id_max, id_min;
    int baudrate_max, baudrate_min, baudrate_target; 
    int latency_timer;
	string device_name;
    nh->declare_parameter("min_id", 0);
    nh->declare_parameter("max_id", 50);
    nh->declare_parameter("device_name", "/dev/ttyUSB0");
    nh->declare_parameter("min_search_baudrate", 57600);
    nh->declare_parameter("max_search_baudrate", 4000000);
    nh->declare_parameter("target_baudrate", 1000000);
    nh->declare_parameter("latency_timer", 16);

    id_min = nh->get_parameter("min_id").as_int();
    id_max = nh->get_parameter("max_id").as_int();
    device_name = nh->get_parameter("device_name").as_string();
    baudrate_min = nh->get_parameter("min_search_baudrate").as_int();
    baudrate_max = nh->get_parameter("max_search_baudrate").as_int();
    baudrate_target = nh->get_parameter("target_baudrate").as_int();
    latency_timer = nh->get_parameter("latency_timer").as_int();

    auto dyn_comm = DynamixelCommunicator();
    dyn_comm.GetPortHandler(device_name.c_str());
    dyn_comm.set_retry_config(2, 5); 

    uint64_t dyn_baudrate;
    switch (baudrate_target) {
        case 9600:    dyn_baudrate = BAUDRATE_INDEX_9600;   break;
        case 57600:   dyn_baudrate = BAUDRATE_INDEX_57600;  break;
        case 115200:  dyn_baudrate = BAUDRATE_INDEX_115200; break;
        case 1000000: dyn_baudrate = BAUDRATE_INDEX_1M;     break;
        case 2000000: dyn_baudrate = BAUDRATE_INDEX_2M;     break;
        case 3000000: dyn_baudrate = BAUDRATE_INDEX_3M;     break;
        case 4000000: dyn_baudrate = BAUDRATE_INDEX_4M;     break;
        case 4500000: dyn_baudrate = BAUDRATE_INDEX_4M5;    break;
        case 6000000: dyn_baudrate = BAUDRATE_INDEX_6M;     break;
        case 10500000:dyn_baudrate = BAUDRATE_INDEX_10M5;   break;
        default: RCLCPP_ERROR(nh->get_logger(), "Invalid baudrate %d", baudrate_target); 
            rclcpp::shutdown(); std::exit(EXIT_SUCCESS);
    }

    vector<int> baudrate_list = {
        9600,
        57600,
        115200,
        1000000,
        2000000,
        3000000,
        4000000,
        4500000,
        6000000,
        10500000
    };

    vector<int> found_ids;
    for ( auto br : baudrate_list ) {
        if ( br < baudrate_min || br > baudrate_max) continue;
        RCLCPP_INFO(nh->get_logger(), "                                              ");
        RCLCPP_INFO(nh->get_logger(), "=== Searching, baudrate:'%d', id:[%d]~[%d] ===", br, id_min, id_max);

        dyn_comm.set_baudrate(br);
        dyn_comm.set_latency_timer( (br<=57600) ? 16 : latency_timer);
        if ( !dyn_comm.OpenPort() ) { fflush(stdout);
            RCLCPP_ERROR(nh->get_logger(), "Failed to open"); 
            continue; 
        } fflush(stdout);

        for (int i=id_min; i<=id_max; i++) { 
            if ( !rclcpp::ok() ) {rclcpp::shutdown(); std::exit(EXIT_SUCCESS);}

            RCLCPP_INFO(nh->get_logger(),"  Scanning ID: %d\x1b[999D\x1b[1A", i);
            if ( std::find(found_ids.begin(), found_ids.end(), i) != found_ids.end() ) continue;
            if ( !dyn_comm.tryPing(i) ) continue;
            RCLCPP_INFO(nh->get_logger(), "  ID [%d] is found, try change baudrate %d", i, baudrate_target);
            auto dyn_model = dyn_comm.tryRead(AddrCommon::model_number, i);
            switch ( dynamixel_series(dyn_model) ) {
                case SERIES_X: 
                    dyn_comm.tryWrite(AddrX::torque_enable, i, TORQUE_DISABLE);
                    dyn_comm.tryWrite(AddrX::baudrate, i, dyn_baudrate);
                    break;
                case SERIES_P:
                    dyn_comm.tryWrite(AddrP::torque_enable, i, TORQUE_DISABLE);
                    dyn_comm.tryWrite(AddrP::baudrate, i, dyn_baudrate);
                    break;
                default:
                    RCLCPP_ERROR(nh->get_logger(), "  Unknown series %ld", dyn_model);
                    continue;
            } 
            found_ids.push_back(i);
        }
        dyn_comm.ClosePort(); fflush(stdout);
    }
    RCLCPP_INFO(nh->get_logger(), "                                              ");
    RCLCPP_INFO(nh->get_logger(), "=== ============ Finish scanning ============ ===\n");

    RCLCPP_INFO(nh->get_logger(), "=== Checking,  baudrate:'%d', id:[%d]~[%d] ===", baudrate_target, id_min, id_max);
    dyn_comm.set_baudrate(baudrate_target);
    dyn_comm.set_latency_timer( (baudrate_target<=57600) ? 16 : latency_timer);
    if ( !dyn_comm.OpenPort() ) { fflush(stdout);
        RCLCPP_ERROR(nh->get_logger(), "Failed to open USB device [%s]", dyn_comm.port_name().c_str()); 
        rclcpp::shutdown(); std::exit(EXIT_SUCCESS);
    } fflush(stdout);
    for ( auto i : found_ids ) {
        if ( dyn_comm.tryPing(i) )
            RCLCPP_INFO(nh->get_logger(), "  ID [%d] is succeded to change baudrate %d", i, baudrate_target);
        else
            RCLCPP_ERROR(nh->get_logger(), "  ID [%d] is failed to change baudrate", i);
    }
    dyn_comm.ClosePort(); fflush(stdout);
    RCLCPP_INFO(nh->get_logger(), "                                              ");
    RCLCPP_INFO(nh->get_logger(), "=== ============ Finish checking ============ ===\n");

    rclcpp::shutdown();
    return 0;
}