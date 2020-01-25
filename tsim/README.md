# ME495 - Turtlesim Trajectory Control in Roscpp 

This project contains two parts: turtle_rect and turtle_way. 

- turtle_rect is a project where I became familiar with essential Roscpp building blocks, including:
    - Publisher and subscriber
    - Service and service client
    - Launch file
    - Parameter Server
    - Turtlesim control 
    - Rosmsg

In this part, a turtle on turtlesim will follow a rectangular path using feedforward control. 

- turtle_way is an example of simulating differential drive robot control using rigid2d librarie's waypoint_generator. The waypoint_generator generates 
a body twist for the robot based on a given set of waypoints. See [rigid2d library](../rigid2d/include/rigid2d/waypoints.hpp) for more information.

## Part 1: turtle_rect 

In this project, turtlesim is launched and a turtle travels along a rectangular trajectory. The turtle will first be teleported to the lower left corner 
of the rectangle, then start travelling along the rectangle's sides. The velocity Its position can also be reset to the lower left corner of the box, (its trajectory is not reset).

![Screenshot from 2020-01-11 16-54-35](https://user-images.githubusercontent.com/39393023/72211777-9ce1bd80-3496-11ea-8479-eff77049fdb7.png)

Rqt_plot can be turned on and off for visualizing pose errors

<img src=https://user-images.githubusercontent.com/39393023/72211778-9ce1bd80-3496-11ea-9e7a-d642f5dab618.png width="600">

For live demo, checkout my Youtube Channel:

[![Screenshot from 2020-01-11 20-44-54](https://user-images.githubusercontent.com/39393023/72213356-53ec3200-34b3-11ea-8b13-6b6d63b2d0af.png)](https://www.youtube.com/embed/YcYuIzouRaE)

### System setup

python 2.7 (python 3 above will have trouble launching rqt_plot)

### Usage
To see the turtle travelling in a rectangular trajectory and see visualized pose errors on rqt_plot, do 
```
$ roslaunch tsim trect.launch
```

To turn off rqt_plot, do

```
$ roslaunch tsim trect.launch rqt_plot_on:=false
```
### File List

- config/params.yaml - Parameters for turtlesim control
- launch/trect.launch - launch file for the project
- msg/PoseError.msg - Msg file for publishing topic /pose_error  
- src/turtle_rect.cpp - node for turtle's trajectory control 
- src/turtle_rect.h - Header file for turtle_rect.cpp
- srv/traj_reset.srv - Service for resetting turtle's position


### Design 

##### Service
- ```traj_reset (tsim/traj_set)``` Reset the turtle's position to the lower left corner of the rectangle.However, this service does not restart the trajectory, i.e, the turtle's will finish the rest of the old trajectory. 
To call the service, do
```$ rosservice call /traj_reset```

##### Publishers
- ``` /pose_error (tsim/PoseError)``` Error between pose estimation from dead-reckoning and the real pose.
To echo the publisher, do ```$ rostopic echo /pose_error ```

- ``` /turtle1/cmd_vel (geometry_msgs/Twist)``` Velocity command to turtlesim. Velocity control is achieved through open loop feedforward control, i.e, we assume turtlebot will move exactly as commanded. 

##### rqt_plot
In this section, you will see constant x and y pose errors between the x and y dead reckoning estimation and 
the real x and y pose (dead-reckoning estimations come solely from velocity commands). The theta pose error was caclculated with 
angle [wrapping mechanism](https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code), so the angle is always within [-pi, pi].


## Part 2: turtle_way

In this project, turtlesim is launched and a turtle travels along a pentagon-shaped trajectory. The turtle will first be teleported to a specified location on turtlesim (parameters are set [here](./config/params.yaml)),
then the turtle starts travelling along the pentagon's sides. To see Turtlesim and the effect of wheel velocity control, 
run ```$ roslaunch tsim turtle_pent.launch ```

As a demonstration of simulating differential drive robot control, turtle_way can also be used to visualize wheel velocity control on a differential 
drive robot in Rviz. Also, the accurateness of the robot's odometry is calculated based on the pose information from turtlesim topic: /turtle1/pose.  

To see the visualization, as well as a live illustration  of the robot's odometry accurateness, run 
``` $ roslaunch tsim turtle_odom.launch ```  

[<img src=https://user-images.githubusercontent.com/39393023/73126242-dece3180-3f75-11ea-8151-d04e27988b68.png width="600">](https://www.youtube.com/embed/HyPuW4h9Eag)

- Robot's Odometry Accurateness Illustration 
<img src=https://user-images.githubusercontent.com/39393023/73126069-c3fabd80-3f73-11ea-9b0a-c96f9d0c51a2.png width="600">
