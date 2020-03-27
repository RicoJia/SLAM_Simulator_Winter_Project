# Simulator for SLAM Algorithms

### Author: Rico Ruotong Jia

Do you have a landmark-based SLAM algorithm (UKF, EKF, landmark-based FAST, etc.) and not sure if it really works? In real physical world there are always
unmodelled factors that make testing difficult. You might also think physics engine, such as Gazebo, are good choices. However, physics engines always come
with redundant features for SLAM testing, such as many dynamics features. If you're looking to test your SLAM algorithm, 
come try out this landmark-based SLAM simulator. This project serves as a light-weight differential drive robot simulator that provides all the essentials you need!
- What is included:
    - Kinematics model of the robot (pose calculation using [screw theory](https://en.wikipedia.org/wiki/Screw_theory))
    - Odometer of the robot, with visualization on Turtlesim and configurable Gaussian noise 
    - 360-degree Lidar Measurement 
    - Customized waypoint-following control with turn and go strategy
    
- What has not yet been included: 
    - Collision and Inertial properties of the robot

Here is a live demo
[<img src=https://user-images.githubusercontent.com/39393023/77220187-a5481b80-6b0b-11ea-949b-41cf7ebfe5e9.png width="600">](https://youtu.be/tB5PEmHVFVU)


### Usage of the project 

a. To successfully run this project, first create the workspace
``` 
$cd ~ 
$mkdir -p SLAM_Simulator/src  
```
b.  copy the src directory to ```SLAM_Simulator/src```

c. Build the workspace and run the project 
```
$ cd ~/SLAM_Simulator 
$ catkin_make
$ source devel/setup.bash
$ roslaunch tsim slam_simulator.launch
```
d. In the ekf_slam package, substitute the template filter node - ekf.py with your python filter algorithm. If you were to use ekf_node as the interface between the algorithm and the rest of the system, make sure the you follow the coding guide at the 
beginning of [ekf_node](ekf_slam/scripts/ekf_node). 

e. **Change all parameters, including differential drive robot, the world, the laser scanner, the filter in [here](../real_world/config/params.yaml).**

f. Check your results. The frames we are interested in are: 
- /world - The world frame
- /actual_robot - The actual robot pose with robot visualization 
- /map - world frame for EKF. Without external drift correction devices such as GPS, this frame coincides with /world
- /odom - world frame for visualizing odometry of the robot. When there is no nose, this frame coincides with /map. When there is
noise, the filter should pose the /map to /odom transform so the estimated robot pose could be shown properly. 
- /estimation - the estimated robot position.

g. Validation: when noise is 0, the estimated robot frame should be almost aligned with the actual robot. When the noise is not 0, the estimated robot frame should always "catch up" with the actual robot pose. 
Due to the low frequency of observation update, the estimated robot frame might "jump". This could be a future improvement as the current 
python implementation only supports 1 hz observation updates.   

### Tranfomation Tree and Code Structure

##### Transformation Tree

![Screenshot from 2020-03-20 23-20-50](https://user-images.githubusercontent.com/39393023/77219207-82b10500-6b01-11ea-96ce-be3a7cd258ae.png)

##### Code Structure

![Screenshot from 2020-03-26 17-59-52](https://user-images.githubusercontent.com/39393023/77704720-b8377180-6f8b-11ea-9a9b-a3b600ce5e84.png)

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
    - Motion control node for travelling in a pentagon trajectory using velocity commands (noiseless_control.cpp)

- ekf_slam
    - Template EKF SLAM pacakge with known landmark correspondences 

### References
The template EKF filter is based on S.Thrun et al's [Probablistic Robotics](http://www.probabilistic-robotics.org/).  