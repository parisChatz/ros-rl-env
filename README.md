# DRL-robot-navigation

Deep Reinforcement Learning for mobile robot navigation in ROS Gazebo simulator. This is the workspace for the robot and the simulation environment on ROS. The robot learns to navigate to a random goal point in a simulated environment while avoiding obstacles. The robot is a pioneer3dx and it uses an rgb camera and a lidar.

Tested with ROS Noetic on Ubuntu 20.04 with python 3.8.10 and pytorch 1.10.

**Installation and code overview tutorial available** [here](https://medium.com/@reinis_86651/deep-reinforcement-learning-in-mobile-robot-navigation-tutorial-part1-installation-d62715722303)

Training example:
<p align="center">
    <img width=100% src="https://github.com/reiniscimurs/DRL-robot-navigation/blob/main/training.gif">
</p>

## Installation
Main dependencies: 

* [ROS Noetic](http://wiki.ros.org/noetic/Installation)

Clone the repository:
```shell
cd ~
### Clone this repo
git clone 
git checkout -b feature/ros-catkin-env
```
The network can be run with a standard 2D laser, but this implementation uses a simulated [3D Velodyne sensor](https://github.com/lmark1/velodyne_simulator)

Compile the workspace:
```shell
cd ~/DRL-robot-nav/catkin_ws
### Compile
catkin_make_isolated
```
## If you want to use this workspace be sure to:

Open a terminal and set up sources:
```shell
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
export ROS_PORT_SIM=11311
export GAZEBO_RESOURCE_PATH=~/DRL-robot-nav/catkin_ws/src/multi_robot_scenario/launch
source ~/.bashrc
cd ~/DRL-robot-nav/catkin_ws
source devel_isolated/setup.bash
```

## Testing
In the same terminal do: 
```
roslaunch multi_robot_scenario multi_robot_scenario.launch
```

Gazebo environment:
<p align="center">
    <img width=80% src="https://github.com/reiniscimurs/DRL-robot-navigation/blob/main/env1.png">
</p>

Rviz:
<p align="center">
    <img width=80% src="https://github.com/reiniscimurs/DRL-robot-navigation/blob/main/velodyne.png">
</p>

