# Real_World

This project is a simulated environment for SLAM algorithms. At the current stage of development, there is a kinematics model for
differential drive robot. This package simulates the following items of a differential drive robot: 
- Pose in "real world", including orientation and (x,y) position in the world frame
- Angular positions and velocities of left and right wheels
- Circular and Rectangular obstacles
- 2D 360 degree Laser Scan data given the positions of obstacles and the robot  

Key assumptions being made here are 
- The robot is subject to slippage, which is modeled as Gaussian noise
- The odometeter is subject to both Gaussian noise and a steady drift. 

### Key Files and Functionalities
- config/params.yaml - parameters of the robot and configurations for visualization on Rviz  
- src/obstacles - publishing obstacles to Rviz using visualization_msgs/MarkerArray
- src/real_world - publishing a differential drive robot's "real world" pose, simulated Lidar scans, and associated transforms 

### System setup

- ROS Melodic 
- Linux Ubuntu 18.04
- C++ 17 compiler

### Usage
- Change all parameters, including differential drive robot, the world, laser scan, the filter in [here](../real_world/config/params.yaml).

### The node in the overall code structure: 
![Screenshot from 2020-03-20 22-13-51](https://user-images.githubusercontent.com/39393023/77218324-23022c00-6af8-11ea-8dca-7ef9163b7178.png)
