#include "can_serial/CanSerialNode.hpp"

CanSerialNode::CanSerialNode(ros::NodeHandle &nh)
{

  // ROS

  // 初始化CAN驱动
  initialCanCore();
  ros::Rate loop_rate(1);
  while(ros::ok()){
    testCanSend();
    loop_rate.sleep();
  }
}

/*
 * @brief can 接收处理函数
 */
void CanSerialNode::handleCanFrame(const can_frame &frame)
{
  ROS_INFO("收到CAN帧 - ID: 0x%x, 长度: %d", frame.can_id, frame.can_dlc);
  return;
  // 仅处理ID为0xA0的帧
  if (frame.can_id == 0xA0 && frame.can_dlc >= 7)
  {
    std::bitset<8> byte0(frame.data[0]);
    uint8_t game_stage = (frame.data[0] >> 0) & 0x07;    // 0-2bit
    uint8_t pc_type = (frame.data[0] >> 3) & 0x01;       // 3bit
    uint8_t windmill_type = (frame.data[0] >> 4) & 0x01; // 4bit
    int16_t int16_roll = (frame.data[2] << 8) | frame.data[1];
    int16_t int16_pitch = (frame.data[4] << 8) | frame.data[3];
    int16_t int16_yaw = (frame.data[6] << 8) | frame.data[5];
    // 欧拉角int16转float，除以100，防止溢出
    float roll = int16_roll / 100.0f;
    float pitch = int16_pitch / 100.0f;
    float yaw = int16_yaw / 100.0f;

    if (DEBUG)
    {
      auto msg = std_msgs::String();
      msg.data =
          "Stage: " + std::to_string(game_stage) +
          " PCType: " + std::to_string(pc_type) +
          " Windmill: " + std::to_string(windmill_type) +
          " Roll: " + std::to_string(roll) +
          " Pitch: " + std::to_string(pitch) +
          " Yaw: " + std::to_string(yaw);

      ROS_INFO("---");
      ROS_INFO("收到数据: %s", msg.data.c_str());
      // ROS_INFO("收到数据: %x %x %x %x %x %x %x",
      //             frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6]);
      // ROS_INFO("收到数据: %s", toBinaryString(frame.data[0]).c_str());
    }
  }
}

void CanSerialNode::testCanSend()
{
  // 构造发送帧（ID 0xA1）
  can_frame frame_to_send;
  frame_to_send.can_id = 0xA1;
  frame_to_send.can_dlc = 8;
  // 构造从零开始自增一的发送代码
  static uint8_t current_value = 0;

  for (int i = 0; i < 8; ++i)
  {
    frame_to_send.data[i] = current_value++;
  }
  can_core_->send_frame(frame_to_send);

  ROS_INFO("---");
  ROS_INFO("发送数据: 帧头%x, %x %x %x %x %x %x %x %x", frame_to_send.can_id
                                                    , frame_to_send.data[0], frame_to_send.data[1]
                                                    , frame_to_send.data[2], frame_to_send.data[3]
                                                    , frame_to_send.data[4], frame_to_send.data[5]
                                                    , frame_to_send.data[6], frame_to_send.data[7]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "can_serial_node");
  ros::NodeHandle nh;
  // 设置本地编码为UTF-8
  setlocale(LC_CTYPE, "zh_CN.utf8");
  CanSerialNode canNode(nh);
  ros::spin();
  return 0;
}