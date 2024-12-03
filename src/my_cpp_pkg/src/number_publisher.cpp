#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono;
using namespace example_interfaces::msg;
using namespace std::chrono;
using namespace example_interfaces::msg;

class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {
        this->declare_parameter("number", 2);
        number_ = this->get_parameter("number").as_int();

        this->declare_parameter("frequency", 500);
        frequency = this->get_parameter("frequency").as_int();

        publisher = this->create_publisher<Int64>("number", 10);
        timer_ = this->create_wall_timer(milliseconds(frequency), bind(&NumberPublisherNode::publishNumbers, this));
        RCLCPP_INFO(this->get_logger(), "NumberPublisherNode is started !");

    }

private:
    rclcpp::Publisher<Int64>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer_;
    int number_;
    int frequency;

    void publishNumbers()
    {
        auto msg = Int64();
        msg.data = number_;
        publisher->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
