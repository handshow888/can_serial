# can_serial

can通信实现  
波特率1000000  
字节存储使用小端模式

## include
包含can和ros2头文件声明和can定义  
`CanSerialCore.hpp` can头文件声明  
`CanSerialNode.hpp` ros2 node头文件声明

## src
`CanSerialCore.cpp` can函数定义  
`CanSerialNode.cpp` ros2 node代码定义

can通信的接收处理函数绑定必须在ros2创建完发布者之后，不然会出现 Segmentation Fault

## 使用说明
先运行scripts目录下的脚本`setup_can_autostart.sh`，将 jetson orin 设备的 can0 设备设置为开机自启。然后编译运行即可

## 示例通信协议
### 下位机->上位机
|  字节                  | 数据                                     |
|------------------------|----------------------------------------|
| byte0（0-2bit）        | Enum_MiniPC_Game_Stage（比赛阶段）     |
| byte0（3-3bit）        | Enum_MiniPC_Type（自瞄目标）           |
| byte0（4-4bit）        | Enum_Windmill_Type（风车类型）         |
| byte1-2                | int16_t roll（roll为float原始值*100）  |
| byte3-4                | int16_t pitch（pitch为float原始值*100）|
| byte5-6                | int16_t yaw（yaw为float原始值*100）    |
### 上位机->下位机
| 字节         | 数据                             |
|--------------|----------------------------------|
| byte0-1      | int16_t target_x <br> target_x为float原始值*1000  |
| byte2-3      | int16_t target_y <br> target_y为float原始值*1000  |
| byte4-5      | int16_t target_z <br> target_z为float原始值*1000  |