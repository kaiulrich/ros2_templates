from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remap_number_topic = ( ("number", "my_number"));


    number_publisher_node = Node(
        package = "my_cpp_pkg",
        executable ="number_publisher",
        name = "my_number_publisher",
        remappings = [
           remap_number_topic
        ],
        parameters=[
            {"number": 4},
            {"frequency": 1000}
        ]
    )

    number_counter_node = Node(
        package = "my_cpp_pkg",
        executable ="number_counter",
        name = "my_number_counter",
        remappings = [
           remap_number_topic
        ]
    )

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)

    return ld