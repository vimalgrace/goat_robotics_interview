# GUI Based Food Delivery Robot

[![Watch the Video](https://img.youtube.com/vi/GnMJh-S3pOw/0.jpg)](https://www.youtube.com/watch?v=GnMJh-S3pOw)

## Introduction
This project demonstrates a GUI-based food delivery robot system. The robot is capable of navigating in an environment, picking up food orders, and delivering them to specified destinations.

## Requirements
- ROS 2
- Turtlebot3
- Gazebo

## Installation
1. Clone the repository:

   git clone https://github.com/your_username/your_repository.git

2. Source the workspace in three terminals:

   source goat_ws/install/setup.bash
   
## Usage
### Launch Gazebo with Turtlebot3:

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

### Launch the navigation stack:

ros2 launch goat_nav2 goat_bringup_launch.py

### Run the final task:

ros2 run goat_nav2 final_task.py
