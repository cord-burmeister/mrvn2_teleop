# Marvin Tele operations 

The *mrvn2_teleop*  repository contains some tele operation tools.


## mrvn2_teleop_key_node

The node allows the user to use the keyboard also on a terminal session to steer remotely a roboter base. This node allows also the movements from an omnidirectional roboter base. 

The arrow key allow a movement in all directions and rotation. See the following diagram.

![Keybaord navigation](doc/keyboard.png)


### Using with turtle Simulation

Starting the turtle simulation 

``` bash
ros2 run turtlesim turtlesim_node
```
Running the keyboard tele operations for the turtle simulation 

``` bash
ros2 run mrvn2_teleop mrvn2_teleop_key_node --ros-args --remap /cmd_vel:=turtle1/cmd_vel
```

