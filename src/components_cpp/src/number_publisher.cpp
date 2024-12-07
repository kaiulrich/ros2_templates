#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

#include "components_cpp/NumberPublisher.hpp"

using namespace std::chrono_literals;

namespace my_namespace
{

    NumberPublisher::NumberPublisher(const rclcpp::NodeOptions &options) : Node("number_publisher", options)
    {
        number_ = 2;

        number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        number_timer_ = this->create_wall_timer(1000ms,
                                                std::bind(&NumberPublisher::publishNumber, this));
        RCLCPP_INFO(this->get_logger(), "Number publisher has been started.");
    }

    void NumberPublisher::publishNumber()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_;
        number_publisher_->publish(msg);
    }

} // namespace mynamespace

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_namespace::NumberPublisher)