# Pioneer 3DX ROS Environment for Reinforcement Learning

This repository contains the ROS environment necessary for launching and controlling the **Pioneer 3DX** robot in a Gazebo simulation. It is specifically designed to work with a custom [Gymnasium-ROS wrapper](https://github.com/mazqtpopx/cranfield-navigation-gym.git) that integrates ROS and reinforcement learning (RL) for training and controlling the robot.

## Table of Contents
- [Overview](#overview)
- [Installation](#installation)
- [Usage](#usage)
  - [Launching the Environment](#launching-the-environment)
  - [Controlling the Pioneer 3DX](#controlling-the-pioneer-3dx)
- [Packages Included](#packages-included)
- [License](#license)

## Overview

This environment, including the ROS map and robot configuration, is based on the [DRL robot navigation](https://github.com/reiniscimurs/DRL-robot-navigation) repository and the work described in the publication [Goal-Driven Autonomous Exploration Through Deep Reinforcement Learning](https://ieeexplore.ieee.org/document/9645287?source=authoralert).

The environment supports launching the **Pioneer 3DX** robot in Gazebo. It includes all necessary configurations and packages to bring up, describe, and control the robot.

## Installation

### Prerequisites

- **[ROS Noetic](http://wiki.ros.org/noetic/Installation)**: Ensure that ROS Noetic is installed and sourced.
- **Gazebo 11**: Required for simulation.
- **Python 3.8+**: For using Gymnasium-based reinforcement learning environments.

### Setting Up the Workspace

1. Clone this repository:
    ```bash
    git clone https://github.com/parisChatz/ros-rl-env.git
    cd ~/ros-rl-env/catkin_ws
    ```

2. Build the Catkin workspace:
    ```bash
    catkin_make_isolated
    ```

3. Source the workspace:
    ```bash
    source devel_isolated/setup.bash
    ```

## Usage

### Launching the Environment

To launch the simulation environment for the Pioneer 3DX robot:
```bash
source catkin_ws/devel_isolated/setup.bash
roslaunch multi_robot_scenario multi_robot_scenario.launch
```
