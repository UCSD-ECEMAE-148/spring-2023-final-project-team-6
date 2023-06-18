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
- Jetson Nano 
- Oak-D Camera
- LD06 Lidar Module
- VESC Motor Driver

We use the OAK-D Camera to provide vision capabilities alongside OpenCV image processing running on the Jetson Nano computer. 

### Lane Switching Implementation 
In order to detect the color of the traffic cone obstacle with OpenCV, we used Color Segmentation by masking each frame. Within this mask, the HSV color identification of each pixel on the image is mapped and compared to a threshold range of values corresponding to the acceptable value for the color orange. If a pixel falls within the specified color value range, that pixel will be counted. We set a threshold of whether the camera’s frame is approximately 7.5% orange. If the number of orange pixels breaks that threshold, the lane switching algorithm is activated. Additionally, we used the guidance package values to switch to the adjacent lane and also implemented a non-blocking delay of 4.5 seconds. This delay allows the robot to have a cooldown and not continuously switch lanes when the robot is in the process of switching lanes and it still sees the cone (i.e orange color). This concept can be shown in the figure below.

[LANE SWITCHING IMAGE]

### LiDAR Emergency Stop Implementation 
The LD06 LiDAR module continuously scans 360° at a 10 Hz sweep frequency with a resolution of just under one degree. The LiDAR outputs the distance measured to the nearest object up to a maximum range of approximately ~15m. The intended purpose of the LiDAR is to detect obstacles directly in front of the robot and stop the robot if the object distance is below a threshold of 40 cm. This will prevent collisions between the robot and obstacles that are not avoided in time with the lane switch algorithm. 

The output data is truncated to a 20° arc directly in front of the robot to remove undesired data collected from areas to the left, right, and rear of the robot. In the event that an obstruction is detected at a distance of 10 cm to 40 cm in front of the robot, the throttle will be set to zero. The minimum threshold value of 10 cm was specified to remove erroneous “zero” readings from the LiDAR while the 40cm threshold is specified as the approximate reaction distance of the robot to an obstacle.

[LIDAR IMAGE]

### Final Presentation
https://docs.google.com/presentation/d/1spB5_34QojcOzi590rJC94e3E9Y_qF3r1F_9n-xsfQw/edit#slide=id.p1

### Future Additions & Recommendations
Given our original goal of an adaptive cruise control system, having a LIDAR-driven throttle control would be able to achieve this. To build on our current implementation we would need to modify the way an obstacle is detected by the lidar from a boolean value to a float containing the distance to the object the car is set to follow. Then calculate the distance between the car and the object to be followed. You may need to account for the size of the car, the size of the object, and other factors.

After that determine the throttle response based on the distance to the object. A simple implementation might have a minimal throttle when the object is close, a maximal throttle when the object is far away, and a linearly increasing throttle for distances in between. Assuming there is a minimum and maximum distance between where the car should stay an example of a throttle scheduler could be.

Implementing YOLO object recognition would provide a more reliable method of recognizing cones. We could also use YOLO to expand the variety of objects detected, with the library’s wide catalog. This could be used to navigate through signs or to take a different path around certain objects. After implementing this style of object recognition we can manipulate the throttle and steering scheduler in a similar manner as we are currently using.


## Acknowledgements
Team 6 gives our heartfelt thanks and appreciation to Dr. Jack Silberman, Kishore Nukala, Moises Lopez and our peers in ECE/MAE-148!




