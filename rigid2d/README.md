# ME495 - rigid2d library

differential drive robot 
This project is a simulated environment for SLAM algorithms. At the current stage of development, there is a kinematics model for
differential drive robot. 

### Key Files and Functionalities
- config/params.yaml - parameters of the robot and configurations for visualization on Rviz  
- include/rigid2d/diff_drive.hpp    - diff drive kinematics model 
- include/rigid2d/fake_diff_encoders.hpp  - Publishes joint states for visualizing on rviz
- include/rigid2d/odometer.hpp  -  Updates odometry for Rviz Visualization
- include/rigid2d/rigid2d.hpp   - Basic mathematical functions of the library
- include/rigid2d/waypoints.hpp - generates body twists to follow along a given trajectory    

### System setup

python 2.7 (python 3 above will have trouble launching rqt_plot)

### Usage
- If you have a differential drive robot and your control package publishes a turtlesim/Twist message, you can use fake_diff_encoders and odometer to visualize your result on rviz  
1. Adjust robot and visualization parameters in config/params.yaml
2. Write a launchfile called visualize_diff_drive.launch, add your node 
```
<launch>

<!-- add your node here  -->

<include file="$(find nuturtle_description)/launch/view_diff_drive.launch">
    <arg name="use_jsp_gui" value="false" doc="False for not using joint state publisher"/>
</include>

<node name = "Odometer" pkg = "rigid2d"  type = "rigid2d_Odometer_node" output="screen" >
  <rosparam file="$(find rigid2d)/config/params.yaml" />
  <remap from="body_frame_id" to="~body_frame_id"/>
  <remap from="right_wheel_joint" to="~right_wheel_joint"/>
  <remap from="left_wheel_joint" to="~left_wheel_joint"/>
  <remap from="odom_frame_id" to="~odom_frame_id"/>
</node>

<node name = "Fake_Diff_Encoders" pkg = "rigid2d"  type = "rigid2d_Fake_Diff_Encoders_node" output="screen" >
  <rosparam file="$(find rigid2d)/config/params.yaml" />
  <remap from="body_frame_id" to="~body_frame_id"/>
  <remap from="right_wheel_joint" to="~right_wheel_joint"/>
  <remap from="left_wheel_joint" to="~left_wheel_joint"/>
  <remap from="odom_frame_id" to="~odom_frame_id"/>
  <remap from="/cmd_vel" to="/turtle1/cmd_vel"/>
</node>
</launch>
```
     
3. To run the simulation, run
```
$ roslaunch <your_package_name> visualize_diff_drive.launch
```

- If you'd like to use the waypoint generator, see the [turtle_way](../tsim/src/turtle_way.cpp) example 

