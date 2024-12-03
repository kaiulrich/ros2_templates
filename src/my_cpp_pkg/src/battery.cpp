#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_state_array.hpp"

using namespace my_robot_interfaces::srv;
using namespace my_robot_interfaces::msg;
using namespace std;
using namespace std::chrono;

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery")
    {
        last_time_battery_state_changed_ = this->get_clock()->now().seconds();
        timer_ = this->create_wall_timer(milliseconds(500), bind(&BatteryNode::checkBateryState, this));
        RCLCPP_INFO(this->get_logger(), "BatteryNode has been started.");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::thread> threads_;

    std::string battery_state_;
    double last_time_battery_state_changed_;

    void checkBateryState()
    {
        threads_.push_back(thread(bind(&BatteryNode::callSetLedService, this)));
    }

    void callSetLedService()
    {

        double time_now = this->get_clock()->now().seconds();

        if (battery_state_ == "full")
        {
            if (time_now - last_time_battery_state_changed_ > 4.0)
            {
                RCLCPP_INFO(this->get_logger(), "Battery is empty! Charging battery...");
                battery_state_ = "empty";
                last_time_battery_state_changed_ = time_now;
                setLed(3, 1);
            }
        }
        else
        {
            if (time_now - last_time_battery_state_changed_ > 6.0)
            {
                RCLCPP_INFO(this->get_logger(), "Battery is now full again.");
                battery_state_ = "full";
                last_time_battery_state_changed_ = time_now;
                setLed(3, 0);
            }
        }
    }

    void setLed(int led_number, int state)
    {
        auto client = this->create_client<SetLed>("set_led");
        while (!client->wait_for_service(seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        auto request = std::make_shared<SetLed::Request>();
        request->led_number = led_number;
        request->state = state;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Baterie set led_number %d, state %d, -> result: %d", (int)request->led_number, (int)request->state, (int)response->success);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
