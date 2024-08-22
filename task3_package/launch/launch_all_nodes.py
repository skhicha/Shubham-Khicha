from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        # Node to publish the radius of movement
        Node(
            package='task3_package',         # Replace with the name of your package
            executable='radius_publisher',  # Node that publishes to the /radius topic
            name='radius_publisher',
            output='screen',
        ),

        # Node to provide the compute_ang_vel service
        Node(
            package='task3_package',           # Replace with the name of your package
            executable='compute_ang_vel_server',  # Service server for angular velocity computation
            name='compute_ang_vel_server',
            output='screen',
        ),

        # Node that acts as a client to the compute_ang_vel service and publishes to cmd_vel
        Node(
            package='task3_package',        # Replace with the name of your package
            executable='turtlebot_node', # Client node that subscribes to /radius and publishes to /cmd_vel
            name='turtlebot_node',
            output='screen',
        ),

        # Node to move the TurtleBot in an infinity shape
        Node(
            package='task3_package',        # Replace with the name of your package
            executable='infinity_publisher',
            name='infinity_publisher',
            output='screen',
        ),
    ])
