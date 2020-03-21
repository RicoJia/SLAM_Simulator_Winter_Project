# Open Loop Noiseless Control For Robot (C++)

In this part, a turtle on turtlesim will follow a customly defined path using feedforward control. 
To define the path, please go to [here](../real_world/config/params.yaml).

- noiseless_control is an example of simulating differential drive robot control using rigid2d librarie's waypoint_generator. The waypoint_generator generates 
a body twist for the robot based on a given set of waypoints. See [rigid2d library](../rigid2d/include/rigid2d/waypoints.hpp) for more information.

### System setup

- ROS Melodic 
- Linux Ubuntu 18.04
- C++ 17 compiler
- python 2.7 (python 3 above will have trouble launching rqt_plot)

### File List

- launch/slam_simulator.launch - launch file for the project
- src/noiseless_control.cpp - node for turtle's trajectory control 

### The node in the node structure

![Screenshot from 2020-03-20 22-15-01](https://user-images.githubusercontent.com/39393023/77218340-46c57200-6af8-11ea-94e0-da973448e66b.png)



