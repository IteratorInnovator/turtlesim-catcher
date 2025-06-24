from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    params_file = os.path.join(get_package_share_directory("turtle_buildup"), "config", "params.yaml")

    master_turtle_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='master_turtle',
        remappings=[
            ('/turtle1/cmd_vel', '/master_turtle/cmd_vel'),
            ('/turtle1/pose', '/master_turtle/pose'),
        ],
        parameters=[params_file]
    )

    turtle_controller_node = Node(
        package='turtlesim_catcher',
        executable='turtle_controller',
        name="turtle_controller_node"
    )

    turtle_spawner_node = Node(
        package='turtlesim_catcher',
        executable='turtle_spawner',
        name="turtle_spawner_node",
        parameters=[params_file]
    )

    return LaunchDescription([
        master_turtle_node,
        turtle_controller_node,
        turtle_spawner_node
    ])
