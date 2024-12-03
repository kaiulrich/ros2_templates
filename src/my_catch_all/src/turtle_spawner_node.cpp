#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"

#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/kill_turtle.hpp"

using namespace std;
using namespace std::chrono;
using namespace turtlesim::srv;
using namespace my_robot_interfaces::msg;
using namespace my_robot_interfaces::srv;


class TurtleSpawerNode : public rclcpp::Node 
{
public:
    TurtleSpawerNode() : Node("turtle_spawer"), turtle_counter_(0)
    {
        this->declare_parameter("spawn_time", 2000);
        spawn_time_ = this->get_parameter("spawn_time").as_int();

        timer_ = this->create_wall_timer(milliseconds(spawn_time_), bind(&TurtleSpawerNode::spawnNewTurtle, this));

        alive_turtles_publisher = this->create_publisher<TurtleArray>("alive_turtles", 10);
        
        kill_turtle_service_ = this->create_service<KillTurtle>("kill_turtle", bind(&TurtleSpawerNode::callbackKillTurtle, this, placeholders::_1, placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "TurtleSpawerNode started");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<TurtleArray>::SharedPtr alive_turtles_publisher;
    rclcpp::Service<KillTurtle>::SharedPtr kill_turtle_service_;
    
    int turtle_counter_;
    int spawn_time_;
    std::string turtle_name_prefix_;
    vector<Turtle> alive_turtles;


    std::vector<std::shared_ptr<std::thread>> spawn_turtle_threads_;
    std::vector<std::shared_ptr<std::thread>> kill_turtle_threads_;


    void callbackKillTurtle(const KillTurtle_Request::SharedPtr request, const KillTurtle_Response::SharedPtr response)
    {
        kill_turtle_threads_.push_back(
            std::make_shared<std::thread>(
                bind(&TurtleSpawerNode::callKillTurtleService, this, request->name)));
        response->success = true;
    }

    void callKillTurtleService(string turtle_name){
        auto client = this->create_client<turtlesim::srv::Kill>("kill");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = turtle_name;

        auto future = client->async_send_request(request);

        try
        {
            future.get();
            for (int i = 0; i < (int)alive_turtles.size(); i++)
            {
                if (alive_turtles.at(i).name == turtle_name)
                {
                    alive_turtles.erase(alive_turtles.begin() + i);
                    publishTurtleArray();
                    break;
                }
            }

        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    void publishTurtleArray(){
        auto msg = TurtleArray();
        msg.turtles = alive_turtles;
        alive_turtles_publisher->publish(msg);
    }

    double randomDouble()
    {
        return double(std::rand()) / (double(RAND_MAX) + 1.0);
    }

    void spawnNewTurtle()
    {
        turtle_counter_++;
        auto name = turtle_name_prefix_ + std::to_string(turtle_counter_);
        double x = randomDouble() * 10.0;
        double y = randomDouble() * 10.0;
        double theta = randomDouble() * 2 * M_PI;

         spawn_turtle_threads_.push_back(
            std::make_shared<std::thread>(
                std::bind(&TurtleSpawerNode::callSpawnTurtleService, this, x, y, theta)));
    }

    void callSpawnTurtleService(double x, double y, double theta)
    {
        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            if (response->name != "")
            {
                Turtle msg = Turtle();
                msg.name = response->name;
                msg.x = x;
                msg.y = y;
                msg.theta = theta;
                alive_turtles.push_back(msg);
                publishTurtleArray();
                RCLCPP_INFO(this->get_logger(), "Turtle %s is now alive.", response->name.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
