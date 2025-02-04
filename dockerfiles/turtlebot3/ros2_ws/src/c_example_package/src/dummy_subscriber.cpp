#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DummySubscriber : public rclcpp::Node
{
public:
    DummySubscriber()
    : Node("dummy_subscriber")
    {
        // Declare parameter
        this->declare_parameter<std::string>("topic_name", "chatter");

        // Get the topic name from the parameter
        topic_name_ = this->get_parameter("topic_name").as_string();

        // Create subscription
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            topic_name_, 10,
            std::bind(&DummySubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }

    std::string topic_name_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DummySubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
