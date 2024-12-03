#include "rclcpp/rclcpp.hpp"
using namespace std;
using namespace std::chrono;

class MyNode: public rclcpp::Node{
    public:
       MyNode() : Node("cpp_test"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello Cpp Node");

        timer_ = this->create_wall_timer(seconds(1), bind(&MyNode::timerCallback, this));
    };

    private:


        rclcpp::TimerBase::SharedPtr timer_;
        int counter_; 

        void timerCallback(){
            counter_++;
            RCLCPP_INFO(this->get_logger(), "Hallo %d",  counter_);
        };


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node =make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
        
    return 0;
}