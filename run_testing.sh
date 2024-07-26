#!/bin/bash
killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient python python3 rviz
# Set ROS environment variables
echo "Setting ROS environment variables..."
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export ROS_PORT_SIM=11311
export GAZEBO_RESOURCE_PATH=~/DRL-robot-nav/catkin_ws/src/multi_robot_scenario/launch

# Source the .bashrc file
echo "Sourcing ~/.bashrc..."
source ~/.bashrc

# Source the workspace setup.bash file
echo "Sourcing devel_isolated/setup.bash..."
source ~/DRL-robot-nav/catkin_ws/devel_isolated/setup.bash

# Change directory to the TD3 folder
echo "Changing directory to the TD3 folder..."
cd ~/DRL-robot-nav/TD3

# Add the root directory to the Python path
export PYTHONPATH=~/DRL-robot-nav:$PYTHONPATH

# Run the Python script for training
echo "Running Python script for training (train_velodyne_td3.py)..."
# python3 test_velodyne_td3.py
python3 test.py
