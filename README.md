# can_serial

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

<img src="./doc/team_logo.png" alt="MosasAurus Logo" title="MOSAS AURUS" style="width:100px"/>  

东北大学秦皇岛分校 Robomaster征龙凌沧战队2025赛季上位机can通信代码 **ROS1 版本**  

波特率1000000  
字节存储使用小端模式  

已在 **Jetson orin nx** 和 **OrangePi 5 Pro** 上测试通过，理论上来讲其他拥有can通信硬件条件的设备也可使用（未测试）。

## include
包含can和ros1头文件声明和can定义  
`CanSerialCore.hpp` can头文件声明  
`CanSerialNode.hpp` ros1 node头文件声明

## src
`CanSerialCore.cpp` can函数定义  
`CanSerialNode.cpp` ros1 node代码定义

can通信的接收处理函数绑定必须在ros1创建完发布者之后，不然会出现 Segmentation Fault

## 使用说明
先运行scripts目录下的脚本`setup_can_autostart.sh`，将 jetson orin 设备的 can0 设备设置为开机自启。然后编译运行即可

## 本地回环测试can功能
(该测试与本代码无关，只是对设备的can接口进行测试)
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 loopback on
sudo ip link set can0 up

# 终端1
candump can0
# 终端2
cansend can0 123#DEADBEEF
```
### 实际使用需关闭本地回环模式
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 loopback off
sudo ip link set can0 up
```