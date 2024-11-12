import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_filepath = os.path.join(
        get_package_share_directory('mrvn2_teleop'), 'config', 'thrustmaster.config.yml')

    # Error handling for missing configuration files
    if not os.path.exists(config_filepath):
        raise FileNotFoundError(f"Configuration file not found: {config_filepath}")

    # Error handling for invalid configuration values
    if not config_filepath:
        raise ValueError("Invalid configuration values provided")

    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(turtlesim)

    return ld
