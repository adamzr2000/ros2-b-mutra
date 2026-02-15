#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DummyPublisher : public rclcpp::Node
{
public:
    DummyPublisher()
    : Node("dummy_publisher")
    {
        // Declare parameters
        this->declare_parameter<std::string>("topic_name", "chatter");
        this->declare_parameter<double>("publish_rate", 1.0);
        this->declare_parameter<std::string>("message", "Hello from dummy_publisher");

        // Get parameters
        topic_name_ = this->get_parameter("topic_name").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        message_ = this->get_parameter("message").as_string();

        // Create publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name_, 10);

        // Create timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / publish_rate_)),
            std::bind(&DummyPublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        auto message = std_msgs::msg::String();
        message.data = message_;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    std::string topic_name_;
    double publish_rate_;
    std::string message_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DummyPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
