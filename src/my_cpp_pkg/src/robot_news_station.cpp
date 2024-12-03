#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::chrono;
using namespace example_interfaces::msg;
class RobotNewsStationNode : public rclcpp::Node
{
public:
    RobotNewsStationNode() : Node("robot_news_station"), robot_name_("R2D2")
    {
        this->declare_parameter("robot_name", "R2D2");
        robot_name_ = this->get_parameter("robot_name").as_string();
        
        publisher = this->create_publisher<String>("robot_news", 10);
        timer_ = this->create_wall_timer(milliseconds(500), bind(&RobotNewsStationNode::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "RobotNewsStationNode is started !");
    }

private:
    rclcpp::Publisher<String>::SharedPtr publisher;
    std::string robot_name_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publishNews()
    {
        auto msg = String();
        msg.data = std::string("Hallo here is ") + robot_name_ + std::string(" from the Robot News Station");
        publisher->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
