//
// Created by ricojia on 1/8/20.
//

#ifndef TSIM_TURTLE_RECT_H
#define TSIM_TURTLE_RECT_H

void drop_pen(ros::NodeHandle nh, bool);
void teleport_2_bottom(ros::NodeHandle nh, double x, double y);
enum class State{
    go_forward,
    turn
};

bool traj_reset_callback();
#endif //TSIM_TURTLE_RECT_H
