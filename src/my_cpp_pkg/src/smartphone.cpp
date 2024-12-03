#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace example_interfaces::msg;
using namespace std;
using namespace std::chrono;

class SmartphoneNode : public rclcpp::Node
{
public:
    SmartphoneNode() : Node("smartphone")
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::String>(
            "robot_news", 10,
            bind(&SmartphoneNode::callbackRobotNews, this, placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Smartphone has been started.");
    }

private:
    rclcpp::Subscription<String>::SharedPtr subscriber_;
    
    void callbackRobotNews(const String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmartphoneNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
