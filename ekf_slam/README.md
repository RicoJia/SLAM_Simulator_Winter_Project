# Sample Filter Package

If have a landmark-based SLAM algorithm (EKF, UKF, etc) and you'd like to test the correctness of your algorithm,
test it here! This package is a template package to refer to. There are two parts: ekf is the pure filter algorithm, while ekf_node and ekf. ekf_node is 
the interface between the rest of the system and ekf. 

At this stage, we have a python implementation of the EKF filter, because of ease of prototyping with Python.
Since the laser scan topic is available,  you can implement your own feature abstraction functionality. 

Also, feel free to implement your filter in C++, and the basic structure of the code should remain similar.  

- Key assumptions being made here are 
    - Known correspondences with circular landmarks. 
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
- Change all filter parameters in [here](../real_world/config/params.yaml).

- We recommend not to change the frequency parameters and follow as strictly with the template as possible. The reason 
is that despite ease of prototyping, Python implementation of filters might suffer relatively long processing times. This 
can cause asynchronization issues. 

### The node in the overall code structure: 
![Screenshot from 2020-03-20 21-48-27](https://user-images.githubusercontent.com/39393023/77217969-8ee29580-6af4-11ea-944e-6bc206f8f3e4.png)
