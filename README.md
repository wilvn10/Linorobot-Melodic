# Linorobot-Melodic
Linorobot for Melodic with implementation of LiDAR, XBOX Kinect V1 and ultrasonic sensors.
This code is modified from the original Linorobot:
https://github.com/linorobot/linorobot

## Author:

1. Wilvan Gunawan Wirjono (@wilvn10)
2. Dr. Rusman Rusyadi (@rusmanr)

## Objectives:
1. Build an Autonomous Mobile Robot that able to navigate itself in a map using Linorobot software to deliver goods.
2. Make the Linorobot compatible with ROS Melodic 18.04
3. Integrate XBOX Kinect V1 (for detecting hovering obstacles) and LiDAR for obstacle detection.
4. Implementation of ultrasonic sensor in the lower part of the robot to prevent the robot collide into small obstacles.

## Components:
1. Teensy 4.0
2. RPLiDAR A2
3. XBOX Kinect V1
4. IMU (MPU-9250)
5. Rotary Encoder
5. Ultrasonic Sensor (HC-SR04)
6. Motor Driver (IBT-2)
7. DC Motor 24V
8. Accumulator Maintenance Free Battery 12V

## How to Run the Code:

Install all perquisite packages to the system as shown in Chapter 3, section 3.9 ROS Package. Download this thesis source file.

1. Install python package for Teensy uDev rule.

> $ sudo apt-get install python-gtk2'

> $ sudo apt-get install python-gobject

2. Make Teensy uDev 49-Rules from:

 https://www.pjrc.com/teensy/49-teensy.rules

3. Create Teensy uDdev rules as shown in Chapter 4, section 4.1 Teensy 4.0 uDev Rules using the following command:

> $ rosrun lino_udev lino_udev.py 
follow the instruction on the screen.

> $ sudo cp 58-lino.rules /etc/udev/rules.d/58-lino.rules

> $ sudo service udev reload

> $ sudo service udev restart

4. Compile the code to the Teensy 4.0 using the following command:

> $ roscd linorobot/teensy/firmware

> $ platformio run – target upload

5. To create a map, launch LiDAR and linobase node with SLAM Gmapping node using the following command:

> $ roslaunch linorobot bringup.launch

> $ roslaunch linorobot slam.launch

> $ roslaunch teleop_twist_joy joy.launch

> $ roscd lino_visualize/rviz

> $ rviz –d slam.rviz

> $ rosrun map_server map_saver –f ~/($your_directory)/src/linorobot/maps/map
saving the map

6. To do the navigation, launch the linobase, Kinect, LiDAR node and navigation stack using the following command:

> $ roslaunch linorobot bringup_kinect.launch 
wait until IMU received the data

> $ roslaunch ira_laser_tools laserscan_multi_merger.launch 
launch laserscan merger for integration of Kinect and LiDAR.

> $ roslaunch linorobot navigate.launch

> $ roscd lino_visualize/rviz

> $ rviz –d navigate.rviz

 Localize the robot on the map using “2D Post Estimate” button in the RViZ.

 To set a goal in the RViZ, use “2D Nav Goal” button and click any known space on the map.

 To send a pre-determined goal coordinate, use the following command: 
 >$ roslaunch send_navigation_goal send_navigation_goal.launch 
 set the x, y and orientation goal in the x_cor, y_cor and w_orient parameter inside send_navigation_goal.launch file.

7. To do the AR-Tag follower, launch the following code:

> $ roslaunch linorobot bringup_kinect.launch

> $ roslaunch ar_tag_toolbox ar_follower.launch

![AMR Design](https://github.com/wilvn10/Linorobot-Melodic/tree/master/image/AMR.png)


