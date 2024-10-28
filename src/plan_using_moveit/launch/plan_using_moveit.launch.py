from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Initialize MoveIt configuration for your robot
    moveit_config = MoveItConfigsBuilder("roarm").to_dict()  # Replace "your_robot_name" with your actual robot name

    # Define the node with parameters
    plan_using_moveit_node = Node(
        package="plan_using_moveit",  # Replace with your package name
        executable="plan_using_moveit",  # Replace with your node/executable name
        output="screen",
        parameters=[
            moveit_config["robot_description_kinematics"],
        ],
    )

    return LaunchDescription([
        plan_using_moveit_node
    ])
