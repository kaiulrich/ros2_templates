#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace example_interfaces::srv;
using namespace std;

class AddTwoIntsServerNode : public rclcpp::Node
{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server")
    {
        server_ = this->create_service<AddTwoInts>("add_two_ints", bind(&AddTwoIntsServerNode::callbackAddTwoInts, this,placeholders::_1, placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Service server has been started.");
    }

private: 
    rclcpp::Service<AddTwoInts>::SharedPtr server_;

    void callbackAddTwoInts(const AddTwoInts_Request::SharedPtr request, const AddTwoInts_Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "%d + %d = %d", (int)request->a, (int)request->b, (int)response->sum);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
