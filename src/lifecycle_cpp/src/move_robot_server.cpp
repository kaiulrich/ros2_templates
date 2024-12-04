#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_robot.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using MoveRobot = my_robot_interfaces::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ServerGoalHandle<MoveRobot>;
using LifecycleCallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::placeholders;

class MoveRobotServerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    MoveRobotServerNode() : LifecycleNode("move_robot_server")
    {
        robot_position_ = 50;
        server_activated_ = false;
        robot_name_ = "";
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);
    }

    // Create ROS2 Communication, connect to hardware
    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "IN on_configure");
        
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.dynamic_typing = true;
        this->declare_parameter("robot_name", rclcpp::ParameterValue{}, descriptor);
        robot_name_ = this->get_parameter("robot_name").as_string();
        
        move_robot_server_ = rclcpp_action::create_server<MoveRobot>(
            this,
            "move_robot_" + robot_name_,
            std::bind(&MoveRobotServerNode::goal_callback, this, _1, _2),
            std::bind(&MoveRobotServerNode::cancel_callback, this, _1),
            std::bind(&MoveRobotServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_);
        RCLCPP_INFO(this->get_logger(), "Action server has been started");
        return LifecycleCallbackReturn::SUCCESS;
    }

    // Destroy ROS2 Communication, disconnect to hardware
    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "IN on_cleanup");
        (void)previous_state;
        robot_name_ = "";
        this->undeclare_parameter("robot_name");
        move_robot_server_.reset();
        return LifecycleCallbackReturn::SUCCESS;
    }

    // activate/enable HW
    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "IN on_activate");
        server_activated_ = true;
        rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    // deactivate/disable HW
    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "IN on_deactivate");
        server_activated_ = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle_)
            {
                if (goal_handle_->is_active())
                {
                    preempted_goal_id_ = goal_handle_->get_goal_id();
                }
            }
        }
        rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    // cleanup everything
    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        robot_name_ = "";
        this->undeclare_parameter("robot_name");
        move_robot_server_.reset();
        return LifecycleCallbackReturn::SUCCESS;
    }
    // prosess errors + cleanup
    LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "IN on_error");
        return LifecycleCallbackReturn::SUCCESS;
    }

private:
    rclcpp_action::Server<MoveRobot>::SharedPtr count_until_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::shared_ptr<MoveRobotGoalHandle> goal_handle_;
    std::mutex mutex_;
    rclcpp_action::GoalUUID preempted_goal_id_;

    rclcpp_action::Server<MoveRobot>::SharedPtr move_robot_server_;

    int robot_position_;
    bool server_activated_;
    std::string robot_name_;

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveRobot::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(this->get_logger(), "Received a goal");
        // refuse goal if one goal is active
        // Check if node is "activated"
        if (!server_activated_)
        {
            RCLCPP_WARN(this->get_logger(), "Node not activated yet");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Validate new goal
        if ((goal->position < 0) || (goal->position > 100) || (goal->velocity <= 0))
        {
            RCLCPP_INFO(this->get_logger(), "Invalid position/velocity, reject goal");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Policy: preempt existing goal when receiving a new valid goal
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle_)
            {
                if (goal_handle_->is_active())
                {
                    RCLCPP_INFO(this->get_logger(), "Abort current goal and accept new goal");
                    preempted_goal_id_ = goal_handle_->get_goal_id();
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Accepting the goal");

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<MoveRobotGoalHandle> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(
        const std::shared_ptr<MoveRobotGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing the goal");
        execute_goal(goal_handle);
    }

    void execute_goal(const std::shared_ptr<MoveRobotGoalHandle> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }

        this->goal_handle_ = goal_handle;

        // Get request from goal
        int goal_position = goal_handle->get_goal()->position;
        double velocity = goal_handle->get_goal()->velocity;

        auto result = std::make_shared<MoveRobot::Result>();
        auto feedback = std::make_shared<MoveRobot::Feedback>();
        rclcpp::Rate loop_rate(1.0);

        RCLCPP_INFO(this->get_logger(), "Execute goal");
        while (rclcpp::ok())
        {
            {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (goal_handle->get_goal_id() == preempted_goal_id_)
                    {
                        result->position = robot_position_;
                        result->message = "Preempted by another goal, or deactivated";
                        goal_handle->abort(result);
                        return;
                    }
                }

                // Check if cancel request
                if (goal_handle->is_canceling())
                {
                    result->position = robot_position_;
                    if (goal_position == robot_position_)
                    {
                        result->message = "Success";
                        goal_handle->succeed(result);
                    }
                    else
                    {
                        result->message = "Canceled";
                        goal_handle->canceled(result);
                    }
                    return;
                }

                int diff = goal_position - robot_position_;

                if (diff == 0)
                {
                    result->position = robot_position_;
                    result->message = "Success";
                    goal_handle->succeed(result);
                    return;
                }
                else if (diff > 0)
                {
                    if (diff < velocity)
                    {
                        robot_position_ += diff;
                    }
                    else
                    {
                        robot_position_ += velocity;
                    }
                }
                else if (diff < 0)
                {
                    if (abs(diff) < velocity)
                    {
                        robot_position_ -= abs(diff);
                    }
                    else
                    {
                        robot_position_ -= velocity;
                    }
                }
            }

            RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);
            feedback->current_position = robot_position_;
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveRobotServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
