# ROS-Based Autonomous Navigation Using the Bug2 Algorithm

## Project Description

This project presents a simulation-based autonomous mobile robot navigation system using the Bug2 algorithm in ROS 1 Noetic and Gazebo.

A TurtleBot3 Burger is used as the simulated mobile robot. The robot navigates through a custom Gazebo environment containing obstacles, a starting position and a goal position.

## Technologies Used

- ROS 1 Noetic
- Gazebo
- Python
- TurtleBot3 Burger
- LiDAR / LaserScan
- Odometry
- Bug2 Navigation Algorithm

## ROS Topics

- `/scan` - LaserScan sensor data
- `/odom` - Robot odometry
- `/cmd_vel` - Robot velocity commands

## Package Structure

```text
bug2_navigation/
├── CMakeLists.txt
├── package.xml
├── scripts/
│   └── bug2_node.py
├── launch/
│   └── bug2.launch
├── worlds/
│   └── bug2_world.world
└── urdf/
