#!/usr/bin/env bash

# Starts gazebo simulator with a jaco robot configured, starts a server that read joints
# input and a python script that controls the robot
roslaunch jaco_on_table jaco_on_table_gazebo_controlled.launch \
| rosrun joint_states_listener joint_states_listener_real_jaco.py \
| python gazebo_realworld_linker.py real_world