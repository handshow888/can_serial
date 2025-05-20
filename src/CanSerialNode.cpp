#include "can_serial/CanSerialNode.hpp"

enum Enum_Windmill_Type : uint8_t // 风车类型
{
  Windmill_Type_Small = 0,
  Windmill_Type_Big,
};

enum Enum_MiniPC_Type : uint8_t // 迷你主机控制类型
{
  MiniPC_Type_Nomal = 0, // 装甲板
  MiniPC_Type_Windmill,  // 风车
};

enum Enum_MiniPC_Game_Stage : uint8_t // 比赛阶段
{
  MiniPC_Game_Stage_NOT_STARTED = 0,
  MiniPC_Game_Stage_READY,
  MiniPC_Game_Stage_SELF_TESTING,
  MiniPC_Game_Stage_5S_COUNTDOWN,
  MiniPC_Game_Stage_BATTLE,
  MiniPC_Game_Stage_SETTLEMENT,
};

CanSerialNode::CanSerialNode(const rclcpp::NodeOptions &options)
    : Node("can_serial_node", options)
{

  // ROS
  this->declare_parameter("timestamp_offset", 0.006);
  this->declare_parameter("cam_x", 0.135);  // 相机x轴相对于云台旋转中心的偏移量(正数)，单位m
  this->declare_parameter("cam_z", -0.043); // 相机z轴相对于云台旋转中心的偏移量(正数)，单位m
  DEBUG = this->declare_parameter("DEBUG", false);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  game_progress_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/game_progress", rclcpp::SensorDataQoS());
  points_num_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/points_num", rclcpp::SensorDataQoS());

  target_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/tracker/target", rclcpp::SensorDataQoS(),
      std::bind(&CanSerialNode::target_callback, this,
                std::placeholders::_1));

  // 初始化CAN驱动
  can_core_ = std::make_unique<CanSerial>("can0");
  try
  {
    can_core_->init();
    can_core_->set_frame_callback(
        std::bind(&CanSerialNode::handle_can_frame, this, std::placeholders::_1));
    can_core_->async_read();
    // 启动Boost.Asio事件循环线程
    can_core_->start_io_service();
    std::cout << "Boost.Asio线程已启动" << std::endl;
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(get_logger(), "CAN初始化失败: %s", e.what());
    throw;
  }
}

/*
 * @brief can 接收处理函数
 */
void CanSerialNode::handle_can_frame(const can_frame &frame)
{
  // RCLCPP_INFO(get_logger(), "收到CAN帧 - ID: 0x%x, 长度: %d", recv_frame_.can_id, recv_frame_.can_dlc);
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

    // 发布TF
    geometry_msgs::msg::TransformStamped transform_stamped, transform_gimbal_cam;
    timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
    cam_x = this->get_parameter("cam_x").as_double();
    cam_z = this->get_parameter("cam_z").as_double();
    transform_stamped.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "gimbal";
    transform_stamped.transform.translation.x = 0;
    transform_stamped.transform.translation.y = 0;
    transform_stamped.transform.translation.z = 0;

    tf2::Quaternion q;
    q.setRPY(roll / 180.0 * M_PI, pitch / 180.0 * M_PI,
             yaw / 180.0 * M_PI);
    transform_stamped.transform.rotation = tf2::toMsg(q);

    // 云台到相机的变换
    transform_gimbal_cam.header.stamp = transform_stamped.header.stamp;
    transform_gimbal_cam.header.frame_id = "gimbal";
    transform_gimbal_cam.child_frame_id = "camera";
    transform_gimbal_cam.transform.translation.x = cam_x;
    transform_gimbal_cam.transform.translation.z = cam_z;

    tf_broadcaster_->sendTransform(transform_stamped);
    // tf_broadcaster_->sendTransform(transform_gimbal_cam);
    static_broadcaster_->sendTransform(transform_gimbal_cam);

    // 发布当前比赛阶段 and points_num
    auto progress_ = std_msgs::msg::UInt8();
    progress_.data = game_stage;
    game_progress_pub_->publish(progress_);

    //points_num
    if (pc_type==0)
    progress_.data = 4.0f;
    else
    progress_.data = 5.0f;

    points_num_pub_->publish(progress_);

    if (DEBUG)
    {
      auto msg = std_msgs::msg::String();
      msg.data =
          "Stage: " + std::to_string(game_stage) +
          " PCType: " + std::to_string(pc_type) +
          " Windmill: " + std::to_string(windmill_type) +
          " Roll: " + std::to_string(roll) +
          " Pitch: " + std::to_string(pitch) +
          " Yaw: " + std::to_string(yaw);

      RCLCPP_INFO(this->get_logger(), "---");
      RCLCPP_INFO(this->get_logger(), "收到数据: %s", msg.data.c_str());
      // RCLCPP_INFO(this->get_logger(), "收到数据: %x %x %x %x %x %x %x",
      //             frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6]);
      // RCLCPP_INFO(this->get_logger(), "收到数据: %s", to_binary_string(frame.data[0]).c_str());
    }
  }
}

void CanSerialNode::target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  // 构造发送帧（ID 0xA1）
  can_frame frame_to_send;
  frame_to_send.can_id = 0xA1;
  frame_to_send.can_dlc = 6;

  // 转换浮点到int16_t（放大1000倍）
  // int16 乘1000 最大可表示32.767米距离，所以坐标值可用1000
  int16_t x = static_cast<int16_t>(msg->point.x * 1000);
  int16_t y = static_cast<int16_t>(msg->point.y * 1000);
  int16_t z = static_cast<int16_t>(msg->point.z * 1000);

  // 填充数据（小端序）
  frame_to_send.data[0] = x & 0xFF;
  frame_to_send.data[1] = (x >> 8) & 0xFF;
  frame_to_send.data[2] = y & 0xFF;
  frame_to_send.data[3] = (y >> 8) & 0xFF;
  frame_to_send.data[4] = z & 0xFF;
  frame_to_send.data[5] = (z >> 8) & 0xFF;

  // 发送数据
  can_core_->send_frame(frame_to_send);
  RCLCPP_INFO(this->get_logger(), "---");
  RCLCPP_INFO(this->get_logger(), "发送数据: x=%.3f, y=%.3f, z=%.3f", msg->point.x, msg->point.y, msg->point.z);
  // RCLCPP_INFO(get_logger(), "发送数据: %x %x %x %x %x %x",
  //             frame_to_send.data[0], frame_to_send.data[1], frame_to_send.data[2], frame_to_send.data[3], frame_to_send.data[4], frame_to_send.data[5]);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CanSerialNode)