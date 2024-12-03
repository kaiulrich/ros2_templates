from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

   
    robots = ["glskart", "bb8", "daneal", "lander", "c3po"];

    for i in robots:
        robot_news_station_node = Node(
            package = "my_cpp_pkg",
            executable ="robot_news_station",
            name = "robot_news_station_" + i,
           
            parameters=[
                {"robot_name": i}
            ]
        )
    
        ld.add_action(robot_news_station_node)
    

    smartphone_node = Node(
        package = "my_cpp_pkg",
        executable ="smartphone",
        name = "smartphone"
        
    )
    ld.add_action(smartphone_node)

    return ld