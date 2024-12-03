#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

using namespace my_robot_interfaces::msg;
using namespace std::chrono;


class HardwareStatusPublisher : public rclcpp::Node 
{
public:
    HardwareStatusPublisher() : Node("hardware_status_publisher") 
    {
        publisher = this->create_publisher<HardwareStatus>("set_led", 10);
        timer_ = this->create_wall_timer(milliseconds(500), bind(&HardwareStatusPublisher::publishHardwareStatus, this));
        RCLCPP_INFO(this->get_logger(), "HardwareStatusPublisher is started !");
    }

private:
    rclcpp::Publisher<HardwareStatus>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer_;

    void publishHardwareStatus()
    {
        auto msg = HardwareStatus();
        msg.temperature = 57;
        msg.are_motors_ready = false;
        msg.debug_message = "Motors are too hot!";
        publisher->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisher>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
