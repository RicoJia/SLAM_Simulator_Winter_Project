# ME495 - Turtlesim Trajectory Control in Roscpp 

This project is an educational project where I became familiar with essential Roscpp building blocks, including:
- Publisher and subscriber
- Service and service client
- Launch file
- Parameter Server
- Turtlesim control 
- Rosmsg

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


