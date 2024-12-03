#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

class LifecycleNodeManager : public rclcpp::Node
{
public:
    LifecycleNodeManager() : Node("lifecycle_manager")
    {
        this->declare_parameter("managed_node_name", "number_publisher");
        std::string node_name = this->get_parameter("managed_node_name").as_string();
        service_change_state_name = "/" + node_name + "/change_state";

        threads_.push_back(std::thread(std::bind(&LifecycleNodeManager::initialization_sequence, this)));
        RCLCPP_INFO(this->get_logger(), "LifecycleNodeManager has been started.");

    }

private:
    std::string service_change_state_name;
    std::vector<std::thread> threads_;

    void initialization_sequence()
    {
      
        RCLCPP_INFO(this->get_logger(), "Trying to switch to configuring");
        change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "configure");
        RCLCPP_INFO(this->get_logger(), "Configuring OK, now inactive");
        
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        RCLCPP_INFO(this->get_logger(), "Trying to switch to activating");
        change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "activate");
        RCLCPP_INFO(this->get_logger(), "Activating OK, now active");
    }

    void change_state(int transition_id, std::string transition_label)
    {

        auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(service_change_state_name);
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        auto transition = lifecycle_msgs::msg::Transition();
        transition.id = transition_id;
        transition.label = transition_label;
        request->transition = transition;

        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Set ChangeState id=%d label=%s -> success: %d", (int)transition_id, transition_label.c_str(), (int)response->success);
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
    auto node = std::make_shared<LifecycleNodeManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
