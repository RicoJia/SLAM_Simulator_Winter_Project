# Simulator for SLAM Algorithms

### Author: Rico Ruotong Jia

This project serves as a light-weight differential drive robot simulator for various SLAM algorithms. 

- What is included:
    - Kinematics model of the robot (pose calculation using [screw theory](https://en.wikipedia.org/wiki/Screw_theory))
    - Odometer of the robot, with visualization on Turtlesim and configurable Gaussian noise 
    - 360-degree Lidar Measurement 
    - Closed loop trajectory following of the robot with turn and go strategy
    
- What has not yet been included: 
    - The slam algorithm part is left for interested users to implement
    - Collision Detection
    - Dynamic model of the robot
    - Noise on Lidar scan measurement 

![Screenshot from 2020-02-10 23-55-39](https://user-images.githubusercontent.com/39393023/74294928-16bece00-4d05-11ea-9538-5effafc6b0f5.png)
### Usage of the project 

To successfully run this project, first create the workspace
``` 
cd ~ 
mkdir -p SLAM_Simulator/src  
```

then copy the src directory to ```SLAM_Simulator/src```

Build the workspace and run the project 
```
$ cd ~/SLAM_Simulator 
$ catkin_make
$ source devel/setup.bash
$ roslaunch tsim odom turtle_odom.launch 
```

### Packages and Key Files
This project consists of the following four packages: 

- nuturtle_description
    - contains the URDF and robot's configuration files (files in nuturtle_description/config)
- real_world 
    - Calculates the robot's real world position based on the robot's intended body twist (real_world.cpp). 
    - Calculates the robot's transform from the world frame to the odometer frame, based on configurable wheel slippage,  odometer noise and odometer drift (real_world.cpp)
    - Publishes circular and rectangular obstacles (obstacles.cpp) 
    - Publishes laser scan messages (real_world.cpp)
    
- rigid2d
    - Library of 2D screw theory functions, including twist, frame transformation, etc. (see rigid2d.hpp)
    - Kinematics model of a differential drive (diff_drive.cpp)
    - Simulated odometer for Rviz Visualization (odometer.cpp)
    - Simulated wheel encoder (fake_encoder.cpp)

 
- tsim
    - Motion control node for travelling in a pentagon trajectory using velocity commands (turtle_way.cpp)