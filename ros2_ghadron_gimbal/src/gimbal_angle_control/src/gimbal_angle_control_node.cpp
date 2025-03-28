#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "payloadSdkInterface.h"
#include <csignal>

// 定义连接参数
T_ConnInfo s_conn = {
    CONTROL_UDP,
    udp_ip_target,
    udp_port_target
};

class GimbalAngleControlNode : public rclcpp::Node
{
public:
    GimbalAngleControlNode() : Node("gimbal_angle_control_node")
    {
        RCLCPP_INFO(this->get_logger(), "Gimbal angle control node started");
        
        // Create subscriber for angle commands
        angle_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "gimbal_angles", 10,
            std::bind(&GimbalAngleControlNode::angle_callback, this, std::placeholders::_1));

        // 初始化 SDK
        try {
            my_payload_ = new PayloadSdkInterface(s_conn);
            my_payload_->sdkInitConnection();
            my_payload_->checkPayloadConnection();
            
            // 设置云台为FOLLOW模式（而不是LOCK模式）
            my_payload_->setPayloadCameraParam(PAYLOAD_CAMERA_GIMBAL_MODE, 
                PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW, PARAM_TYPE_UINT32);
            usleep(1000000);  // 等待模式切换完成
                
            RCLCPP_INFO(this->get_logger(), "SDK initialized successfully");
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize SDK");
            if (my_payload_ != nullptr) {
                delete my_payload_;
                my_payload_ = nullptr;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Node is ready to receive angle commands");
        RCLCPP_INFO(this->get_logger(), "Using topic: /gimbal_angles");

        // Create publisher for current angles
        current_angles_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("current_gimbal_angles", 10);
    }

    ~GimbalAngleControlNode()
    {
        if (my_payload_ != nullptr) {
            delete my_payload_;
        }
    }

    void read_gimbal_angles()
    {
        if (my_payload_ == nullptr) {
            return;
        }

        try {
            mavlink_message_t msg;
            uint8_t msg_cnt = my_payload_->getNewMewssage(msg);
            if (msg_cnt && msg.msgid == MAVLINK_MSG_ID_MOUNT_ORIENTATION) {
                mavlink_mount_orientation_t mount_orientation;
                mavlink_msg_mount_orientation_decode(&msg, &mount_orientation);

                // 打印到日志
                RCLCPP_INFO(this->get_logger(), 
                    "\n当前云台角度:\n  Roll: %.2f\n  Pitch: %.2f\n  Yaw: %.2f", 
                    mount_orientation.roll,
                    mount_orientation.pitch,
                    mount_orientation.yaw);

                // 发布到话题
                auto msg = geometry_msgs::msg::Vector3();
                msg.x = mount_orientation.roll;
                msg.y = mount_orientation.pitch;
                msg.z = mount_orientation.yaw;
                current_angles_publisher_->publish(msg);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "读取云台角度时发生错误: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "读取云台角度时发生未知错误");
        }
    }

private:
    void angle_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "进入回调函数");
        
        RCLCPP_INFO(this->get_logger(), 
            "\n收到角度命令:\n  Roll: %.2f\n  Pitch: %.2f\n  Yaw: %.2f", 
            msg->x, msg->y, msg->z);

        if (my_payload_ == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "云台未连接 (my_payload_ 为空)");
            return;
        }

        try {
            RCLCPP_DEBUG(this->get_logger(), "尝试设置云台角度...");
            // 直接使用INPUT_ANGLE模式，不需要预先设置LOCK模式
            my_payload_->setGimbalSpeed(msg->x, msg->y, msg->z, Gimbal_Protocol::INPUT_ANGLE);
            usleep(500000);  // 给云台一些响应时间
            RCLCPP_INFO(this->get_logger(), "云台角度设置成功");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "设置云台角度失败: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "设置云台角度时发生未知错误");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angle_subscriber_;
    PayloadSdkInterface* my_payload_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr current_angles_publisher_;
};

// Signal handler function
void signal_handler(int signum) {
    (void)signum;  // 显式标记参数未使用
    rclcpp::shutdown();
}

int main(int argc, char** argv)
{
    signal(SIGINT, signal_handler);
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GimbalAngleControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}