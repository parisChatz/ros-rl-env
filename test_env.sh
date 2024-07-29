#!/bin/bash
killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient python python3 rviz
# Set ROS environment variables
echo "Setting ROS environment variables..."
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export ROS_PORT_SIM=11311
export GAZEBO_RESOURCE_PATH=~/cranfield-navigation-gym/ros-rl-env/catkin_ws/src/multi_robot_scenario/launch

# Source the .bashrc file
echo "Sourcing ~/.bashrc..."
source ~/.bashrc

# Source the workspace setup.bash file
echo "Sourcing devel_isolated/setup.bash..."
source ~/cranfield-navigation-gym/ros-rl-env/catkin_ws/devel_isolated/setup.bash

# Add the root directory to the Python path
export PYTHONPATH=~/cranfield-navigation-gym:$PYTHONPATH

roslaunch multi_robot_scenario multi_robot_scenario.launch