rosservice call /traj_reset "{}" 
System setup: python 2.7 (python 3 above will have trouble launching rqt_plot)

To see the turtle travelling in a rectangular trajectory and see visualized pose errors on rqt_plot, do 
```
$roslaunch tsim trect.launch
```

To turn off rqt_plot, do

```
 roslaunch tsim trect.launch rqt_plot_on:=false
```

rqt_plot:
In this section, you will see constant x and y pose errors between the x and y dead reckoning estimation and 
the real x and y pose (dead-reckoning estimations come solely from velocity commands). The theta pose error was caclculated with 
angle [wrapping mechanism](https://stackoverflow.com/questions/11498169/dealing-with-angle-wrap-in-c-code), so the angle is always within [-pi, pi]


