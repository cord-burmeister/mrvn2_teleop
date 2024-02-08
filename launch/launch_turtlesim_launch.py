
from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():
   turtlesim = Node(
      package = 'turtlesim',
      executable='turtlesim_node',
      output='screen'
   )

   # teleop_key = Node(
   #    package = 'm2_teleop',
   #    executable='m2_teleop_key_node',
   #    output='screen',
   #    emulate_tty=True,
   # )

   ld = LaunchDescription()
   ld.add_action (turtlesim)
   # ld.add_action (teleop_key)

   return ld
