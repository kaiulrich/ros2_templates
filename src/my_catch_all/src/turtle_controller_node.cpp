#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/kill_turtle.hpp"


using namespace std;
using namespace std::chrono;
using namespace turtlesim::msg;
using namespace geometry_msgs::msg;
using namespace my_robot_interfaces::msg;
using namespace my_robot_interfaces::srv;

class TurtleControllerNode : public rclcpp::Node
{
public:
    TurtleControllerNode() : Node("turtle_controller"), name_("turtle1"), turtlesim_up_(false) , catch_closest_turtle_first(true)
    {
        
        this->declare_parameter("catch_closest_turtle_first", false);
        catch_closest_turtle_first = this->get_parameter("catch_closest_turtle_first").as_bool();

        subscriber_ = this->create_subscription<Pose>(
            name_ + "/pose", 10,
            bind(&TurtleControllerNode::callbackPose, this, placeholders::_1));

        turtles_alive_subscriber_ = this->create_subscription<TurtleArray>(
            "/alive_turtles", 10,
            bind(&TurtleControllerNode::callbackTurtlesAlive, this, placeholders::_1));

        publisher = this->create_publisher<Twist>(name_ + "/cmd_vel", 10);

        timer_ = this->create_wall_timer(milliseconds(10), bind(&TurtleControllerNode::controllLoopCallback, this));

        RCLCPP_INFO(this->get_logger(), "TurtleControllerNode started");
    };

private:
    string name_;
    rclcpp::Subscription<Pose>::SharedPtr subscriber_;
    rclcpp::Subscription<TurtleArray>::SharedPtr turtles_alive_subscriber_;
    rclcpp::Publisher<Twist>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::shared_ptr<std::thread>> kill_turtle_threads_;

   bool turtlesim_up_; 
   
   bool catch_closest_turtle_first; 

    Turtle turtle_to_catch_;

    Pose pose_;

    void callKillTurtleService(string name)
    {
        auto client = this->create_client<KillTurtle>("kill_turtle");
        while (!client->wait_for_service(seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        auto request = std::make_shared<KillTurtle::Request>();
        request->name = name;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Kill Turtle %s -> result: %d", request->name.c_str(), (int)response->success);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    void callbackTurtlesAlive(TurtleArray::SharedPtr msg)
    {
        if (!msg->turtles.empty() || turtle_to_catch_.name == "")
        {
            if (catch_closest_turtle_first)
            {
                Turtle closest_turtle = msg->turtles.at(0);
                double closest_turtle_distance = getDistanceFromCurrentPose(closest_turtle);

                for (int i = 1; i < (int)msg->turtles.size(); i++)
                {
                    double distance = getDistanceFromCurrentPose(msg->turtles.at(i));
                    if (distance < closest_turtle_distance)
                    {
                        closest_turtle = msg->turtles.at(i);
                        closest_turtle_distance = distance;
                    }
                }

                turtle_to_catch_ = closest_turtle;
            }
            else
            {
                turtle_to_catch_ = msg->turtles.at(0);
            }
        }
    }

    double getDistanceFromCurrentPose(Turtle turtle)
    {
        double dist_x = turtle.x - pose_.x;
        double dist_y = turtle.y - pose_.y;
        return std::sqrt(dist_x * dist_x + dist_y * dist_y);
    }

    void controllLoopCallback()
    {
        if (!turtlesim_up_ || turtle_to_catch_.name == "")
        {
            return;
        }

        double dist_x = turtle_to_catch_.x - pose_.x;
        double dist_y = turtle_to_catch_.y - pose_.y;
        double distance = std::sqrt(dist_x * dist_x + dist_y * dist_y);

        auto msg = geometry_msgs::msg::Twist();

        if (distance > 0.5)
        {
            // position
            msg.linear.x = 2 * distance;

            // orientation
            double steering_angle = std::atan2(dist_y, dist_x);
            double angle_diff = steering_angle - pose_.theta;
            if (angle_diff > M_PI)
            {
                angle_diff -= 2 * M_PI;
            }
            else if (angle_diff < -M_PI)
            {
                angle_diff += 2 * M_PI;
            }
            msg.angular.z = 6 * angle_diff;
        }
        else
        {
            // target reached!
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            kill_turtle_threads_.push_back(
                std::make_shared<std::thread>(
                    std::bind(&TurtleControllerNode::callKillTurtleService, this, turtle_to_catch_.name)));
            turtle_to_catch_.name = "";
        
        }
        

        publisher->publish(msg);
    };

    void callbackPose(Pose::SharedPtr pose)
    {
        turtlesim_up_ = true;
        pose_ = *pose.get();
    };
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}