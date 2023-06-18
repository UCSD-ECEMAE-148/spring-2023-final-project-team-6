# ECE/MAE 148 - Team 6 - Spring 2023 Final Project
# Autonomous obstacle detection and lane switching with ROS2
## UC San Diego | Jacobs School of Engineering

## :wave: Team 6: Chat GPT

Bora Gursel - MAE - Mechanical Engineering <br>
Karthik Vetrivelan - ECE - Computer Engineering <br>
Sidath Wijesinghe - ECE - Electrical Engineering

## :blue_book: Final Project Overview

The overall objective of this project is to supplement the baseline lane guidance programs in ROS2  by implementing autonomous lane switching in response to obstacles detected on the race track and automatically stopping at a critical distance threshold to prevent collisions. The reach goal for this project was to implement an adaptive cruise control scheme that could match the speed of a moving object ahead of the robot, e.g. another robot in motion. A LiDAR sensor is integrated with an OAK-D camera using the ROS2 framework to trigger autonomous lane switching on the race track.

### Robot

[Image of Robot]

### Schematic

[Image of Schematic]

### Deliverables
**Must Have:**
- OAK-D Camera detects obstacles on track using color-shift detection 
- ROS2 detection program implements lane switching 
- LIDAR detect obstacles to stop when it is too close
- LIDAR input stops robot for obstacles obstructing course track if lane switch fails

**Nice to Have:**
- Use LIDAR to detect obstacles in motion (i.e., other autonomous vehicles) and adjust robot speed to avoid collision 

### Key Hardware Items


We use the OAK-D Camera to provide vision capabilities alongside OpenCV image processing running on the Jetson Nano computer. 




