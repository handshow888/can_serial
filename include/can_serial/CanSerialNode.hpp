#include "CanSerialCore.hpp"
#include <ros/ros.h>
#include <std_msgs/String.h>

class CanSerialNode
{
public:
    explicit CanSerialNode(ros::NodeHandle &nh);

private:
    void initialCanCore();
    void handleCanFrame(const can_frame &frame); // can 接收处理函数
    void testCanSend();
    std::string toBinaryString(uint8_t value);

    bool DEBUG;

    std::unique_ptr<CanSerial> can_core_;
};

void CanSerialNode::initialCanCore()
{
    can_core_ = std::make_unique<CanSerial>("can0");
    try
    {
        can_core_->init();
        can_core_->set_frame_callback(
            std::bind(&CanSerialNode::handleCanFrame, this, std::placeholders::_1));
        can_core_->async_read();
        // 启动Boost.Asio事件循环线程
        can_core_->start_io_service();
        std::cout << "Boost.Asio线程已启动" << std::endl;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("CAN初始化失败: %s", e.what());
        throw;
    }
}

// 辅助函数：将 uint8_t 转换为二进制字符串
std::string CanSerialNode::toBinaryString(uint8_t value)
{
    std::ostringstream oss;
    for (int i = 7; i >= 0; --i) // 从最高位到最低位
    {
        oss << ((value >> i) & 1); // 提取第 i 位
    }
    return oss.str();
}