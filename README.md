# Linorobot-Melodic
Linorobot for Melodic with implementation of LiDAR, XBOX Kinect V1 and ultrasonic sensors.
This code is modified from the original Linorobot:
https://github.com/linorobot/linorobot

## Author:

1. Wilvan Gunawan Wirjono (@wilvn10)
2. Dr. Rusman Rusyadi (@rusmanr)

## Objectives:
1. Build an Autonomous Mobile Robot that able to navigate itself in a map using Linorobot.
2. Make the Linorobot compatible with ROS Melodic 18.04
3. Integrate XBOX Kinect V1 (for detecting hovering obstacles) and LiDAR for obstacle detection.
4. Implementation of ultrasonic sensor in the lower part of the robot to prevent the robot collide into small obstacles.

## How to Run the Code:

Install all perquisite packages to the system as shown in Chapter 3, section 3.9 ROS Package. Download this thesis source file.

> Install python package for Teensy uDev rule.

o'$ sudo apt-get install python-gtk2'

o $ sudo apt-get install python-gobject

 Make Teensy uDev 49-Rules from:

o https://www.pjrc.com/teensy/49-teensy.rules

 Create Teensy uDdev rules as shown in Chapter 4, section 4.1 Teensy 4.0 uDev Rules using the following command:

o $ rosrun lino_udev lino_udev.py – follow the instruction on the screen.

o $ sudo cp 58-lino.rules /etc/udev/rules.d/58-lino.rules

o $ sudo service udev reload

o $ sudo service udev restart

 Compile the code to the Teensy 4.0 using the following command:

o $ roscd linorobot/teensy/firmware

o $ platformio run – target upload

 To create a map, launch LiDAR and linobase node with SLAM Gmapping node using the following command:

o $ roslaunch linorobot bringup.launch

o $ roslaunch linorobot slam.launch

o $ roslaunch teleop_twist_joy joy.launch

o $ roscd lino_visualize/rviz

o $ rviz –d slam.rviz

o $ rosrun map_server map_saver –f ~/($your_directory)/src/linorobot/maps/map – saving the map

 To do the navigation, launch the linobase, Kinect, LiDAR node and navigation stack using the following command:

o $ roslaunch linorobot bringup_kinect.launch – wait until IMU received the data

o $ roslaunch ira_laser_tools laserscan_multi_merger.launch – launch laserscan merger for integration of Kinect and LiDAR.

o $ roslaunch linorobot navigate.launch

o $ roscd lino_visualize/rviz

o $ rviz –d navigate.rviz

o Localize the robot on the map using “2D Post Estimate” button in the RViZ.

o To set a goal in the RViZ, use “2D Nav Goal” button and click any known space on the map.

o To send a pre-determined goal coordinate, use the following command: $ roslaunch send_navigation_goal send_navigation_goal.launch – set the x, y and orientation goal in the x_cor, y_cor and w_orient parameter inside send_navigation_goal.launch file.

 To do the AR-Tag follower, launch the following code:

o $ roslaunch linorobot bringup_kinect.launch

o $ roslaunch ar_tag_toolbox ar_follower.launch

