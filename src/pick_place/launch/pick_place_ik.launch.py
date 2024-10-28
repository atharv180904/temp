import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        
        # Launch MoveIt! demo, but suppress its output
        ExecuteProcess(
            cmd=['ros2', 'launch', 'roarm_moveit_config', 'demo.launch.py'],
            output='screen',
            log_cmd=False,
            shell=True
        ),

        # Run driver_for_ik.py (output should be shown)
        ExecuteProcess(
            cmd=['ros2', 'run', 'pick_place', 'driver_for_ik.py'],
            output='screen',
            log_cmd=False,
            shell=True
        ),

        # Run pick_place_ik (output should be shown)
        ExecuteProcess(
            cmd=['ros2', 'run', 'pick_place', 'pick_place_ik'],
            output='screen',
            shell=True
        )
    ])
