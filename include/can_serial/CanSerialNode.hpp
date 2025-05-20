#include "CanSerialCore.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class CanSerialNode : public rclcpp::Node
{
public:
    explicit CanSerialNode(const rclcpp::NodeOptions &options);

private:
    void handle_can_frame(const can_frame &frame); // 原handle_received_frame
    void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    std::string to_binary_string(uint8_t value);

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr game_progress_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr points_num_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    bool DEBUG;
    double timestamp_offset_, cam_x, cam_z;

    std::unique_ptr<CanSerial> can_core_;
};

// 辅助函数：将 uint8_t 转换为二进制字符串
std::string CanSerialNode::to_binary_string(uint8_t value)
{
    std::ostringstream oss;
    for (int i = 7; i >= 0; --i) // 从最高位到最低位
    {
        oss << ((value >> i) & 1); // 提取第 i 位
    }
    return oss.str();
}