#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class RepublishNode : public rclcpp::Node
{
public:
    RepublishNode() : Node("republish_node")
    {
        subscription_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_state_broadcaster/joint_states",
            1,
            std::bind(&RepublishNode::jointStateCallback, this, std::placeholders::_1)
        );

        publisher_ = create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states",
            1
        );

        timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000/30)),  // 1/30 seconds
            std::bind(&RepublishNode::publishRepublishedTopic, this)
        );
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        joint_state_data_ = msg;
    }

    void publishRepublishedTopic()
    {
        if (joint_state_data_)
        {
            publisher_->publish(*joint_state_data_);
            joint_state_data_ = nullptr;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState::SharedPtr joint_state_data_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto republish_node = std::make_shared<RepublishNode>();
    rclcpp::spin(republish_node);
    rclcpp::shutdown();
    return 0;
}
