# ME495 - Turtlesim Trajectory Control in Roscpp 

This project is an educational project where I became familiar with visualization using rviz.
At the current stage of development, an Rviz Xacro model is implemented for future robot control projects. 


![Screenshot from 2020-01-12 09-57-31](https://user-images.githubusercontent.com/39393023/72221618-21c5e900-3522-11ea-9704-f10a760f2507.png)

### System setup
- Ros Melodic
- Linux Ubuntu 18.04 
- python 2.7 (python 3 above will have trouble launching rqt_plot)

### Usage

1. Set up the project
    ```
    $ mkdir -p /Navigation_Projects/src
    $ cd /Navigation_Projects/src
    $ git clone git@github.com:ME495-Navigation/main-assignment-RicoJia.git
    $ cd ..
    $ source devel/setup.bash
    ```
2. To see the robot model in Rviz without joint_state_publisher gui, run
    ```
    $roslaunch nuturtle_description view_diff_drive.launch use_jsp_gui:=false
    ```
   Or if you'd like to see the project with joint_state_publisher gui, run
    ```
    $roslaunch nuturtle_description view_diff_drive.launch
    ``` 
   
### File List

1. config/view_robot.rviz - rviz view settings file 
2. launch/view_diff_drive.launch - launch file for launching rviz, joint_state_publisher, and the model
3. urdf/diff_drive.urdf.xacro - Xacro model 


