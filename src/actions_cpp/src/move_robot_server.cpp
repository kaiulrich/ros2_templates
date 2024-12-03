#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/move_robot.hpp"
using MoveRobot = my_robot_interfaces::action::MoveRobot;
using MoveRobotGoalHandle = rclcpp_action::ServerGoalHandle<MoveRobot>;
using namespace std::placeholders;

class MoveRobotServerNode : public rclcpp::Node
{
public:
    MoveRobotServerNode() : Node("move_robot_server")
    {
        robot_position_ = 50;
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        count_until_server_ = rclcpp_action::create_server<MoveRobot>(
            this,
            "move_robot",
            std::bind(&MoveRobotServerNode::goal_callback, this, _1, _2),
            std::bind(&MoveRobotServerNode::cancel_callback, this, _1),
            std::bind(&MoveRobotServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_);
        RCLCPP_INFO(this->get_logger(), "Action server has been started");
        RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);
    }

private:
    rclcpp_action::Server<MoveRobot>::SharedPtr count_until_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    std::shared_ptr<MoveRobotGoalHandle> goal_handle_;
    std::mutex mutex_;
    rclcpp_action::GoalUUID preempted_goal_id_;

    int robot_position_;

    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const MoveRobot::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(this->get_logger(), "Received a goal");
        // refuse goal if one goal is active

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
                        result->message = "Preempted by another goal";
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
    executor.add_node(node);
    executor.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
