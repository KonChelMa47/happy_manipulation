# lib_dynamixel

Robotis社の[Dynamixel](https://e-shop.robotis.co.jp/list.php?c_id=89)をROSから制御するためのライブラリ.  
基本的にROSで使うことを想定しているので，単体でのビルドは試してない．
ROS1に組み込んだものが[こちら](https://github.com/SHINOBI-organization/DynamixelHandler-ros1/tree/main)

## 初期設定と注意事項

`src\download\port_handler_linux.cpp` 内の `LATENCY_TIMER` と使用するUBSデバイスのlatency timerを一致させる必要がある．

このライブラリのデフォルトは`LATENCY_TIMER=16`で，USBデバイス側のデフォルトと等しいので通常は問題にならない.

`src\download\port_handler_linux.cpp` 内の `LATENCY_TIMER`は，該当部分を書き換えてコンパイルでOK

例
```cpp
#define LATENCY_TIMER 4
```

使用するUSBデバイスのlatency timerは次のようにして変更する．
```
$ echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"4\" > 99-dynamixelsdk-usb.rules
$ sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger --action=add
$ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

一時的であれば以下のようにしてもよい．
```
$ echo 4 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
$ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

### コピペ用

```
echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"4\" > 99-dynamixelsdk-usb.rules
sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

```
echo 4 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

### メモ
本体 2, このライブラリ 4くらいがちょうどいいかもしれない．