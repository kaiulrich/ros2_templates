#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_state_array.hpp"

using namespace my_robot_interfaces::srv;
using namespace my_robot_interfaces::msg;
using namespace std;
using namespace std::chrono;

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel")
    {

        this->declare_parameter("led_states", std::vector<int>({0, 0, 0}));
        led_states_ = this->get_parameter("led_states").as_integer_array();
        
        set_led_service_ = this->create_service<SetLed>("set_led", bind(&LedPanelNode::callbackSetLed, this, placeholders::_1, placeholders::_2));
        led_states_publisher_ = this->create_publisher<LedStateArray>("led_panel_state", 10);
      
        RCLCPP_INFO(this->get_logger(), "Service server has been started.");
    }

private:
    rclcpp::Service<SetLed>::SharedPtr set_led_service_;
    rclcpp::Publisher<LedStateArray>::SharedPtr led_states_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int64_t> led_states_;

    void publishLedStates()
    {
        auto msg = LedStateArray();
        msg.led_states = led_states_;
        led_states_publisher_->publish(msg);
    }

    void callbackSetLed(const SetLed_Request::SharedPtr request, const SetLed_Response::SharedPtr response)
    {
        int64_t led_number = request->led_number;
        int64_t state = request->state;

        

        if (led_number > (int64_t)led_states_.size() || led_number <= 0)
        {
            response->success = false;
            return;
        }

        if (state != 0 && state != 1)
        {
            response->success = false;
            return;
        }

        led_states_.at(led_number - 1) = state;
        response->success = true;
        publishLedStates();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
