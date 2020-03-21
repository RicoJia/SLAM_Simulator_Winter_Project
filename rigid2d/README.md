# Rigid2d Library (C++)

This project is a simulated environment for SLAM algorithms. At the current stage of development, there is a kinematics model for
differential drive robot. This library was developed based on the screw theory and Lie group algebra. 

### Key Files and Functionalities
- config/params.yaml - parameters of the robot and configurations for visualization on Rviz  
- include/rigid2d/diff_drive.hpp    - diff drive kinematics model 
- include/rigid2d/fake_diff_encoders.hpp  - Publishes joint states for visualizing on rviz
- include/rigid2d/odometer.hpp  -  Updates odometry for Rviz Visualization
- include/rigid2d/rigid2d.hpp   - Basic mathematical functions of the library
- include/rigid2d/waypoints.hpp - generates body twists to follow along a given trajectory    

### System setup

- ROS Melodic 
- Linux Ubuntu 18.04
- python 2.7 (python 3 above will have trouble launching rqt_plot)
- If you'd like to use the waypoint generator, see the [tsim/noiseless_control.cpp](../tsim/src/noiseless_control.cpp) example 

### Usage:
In your c++ code, do #include "rigid2d/somefile.hpp" to use part of this library. 

### Rigid2D Nodes In Code Structure
![Screenshot from 2020-03-20 22-16-56](https://user-images.githubusercontent.com/39393023/77218359-88eeb380-6af8-11ea-957e-c9daba1a7056.png)
