#!/bin/bash
#ROS exportations 
#ROS source 
rosrun two_scara_collaboration scara_left_motion_planner&
sleep 1
rosrun two_scara_collaboration scara_right_motion_planner&
