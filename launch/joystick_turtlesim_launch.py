import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


def generate_launch_description():
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    # Error handling for missing configuration files
    if not os.path.exists(config_filepath):
        raise FileNotFoundError(f"Configuration file not found: {config_filepath}")

    # Error handling for invalid configuration values
    if not joy_config or not joy_dev:
        raise ValueError("Invalid configuration values provided")

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_config', default_value='thrustmaster'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('mrvn2_teleop'), 'config')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),

        launch_ros.actions.Node(
            package='turtlesim', executable='turtlesim_node',
            name='turtlesim_node'),

        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),
        launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[config_filepath])

    ])
