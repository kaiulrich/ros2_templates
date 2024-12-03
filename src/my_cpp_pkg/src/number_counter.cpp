#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std;
using namespace std::chrono;
using namespace example_interfaces::msg;
using namespace example_interfaces::srv;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter")
    {
        publisher_ = this->create_publisher<Int64>("number_count", 10);
        
        subscriber_ = this->create_subscription<Int64>(
            "number", 10,
            bind(&NumberCounterNode::callbackNuberCount, this, placeholders::_1));
        
        reset_counter_server_ = this->create_service<SetBool>("reset_counter", bind(&NumberCounterNode::callbackResetCounter, this, placeholders::_1, placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "NumberCounterNode is started !");
    }

private:
    rclcpp::Publisher<Int64>::SharedPtr publisher_;
    rclcpp::Subscription<Int64>::SharedPtr subscriber_;
    rclcpp::Service<SetBool>::SharedPtr reset_counter_server_;

    int counter_ = 0;

    void callbackResetCounter(const SetBool_Request::SharedPtr request, const SetBool_Response::SharedPtr response)
    {
        if (request->data)
        {
            counter_ = 0;
            response->success = true;
            RCLCPP_INFO(this->get_logger(), "Reset counter sucsessfull counter=%d", counter_);
        }
        else
        {
            response->success = false;
            response->message ="Somthing went wrong!";
            RCLCPP_INFO(this->get_logger(), "Reset counter NOT sucsessfull counter=%d", counter_);
        }
    }

    void callbackNuberCount(const Int64::SharedPtr msg)
    {
        counter_ += msg->data;
        auto newMsg = Int64();
        newMsg.data = counter_;
        publisher_->publish(newMsg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
