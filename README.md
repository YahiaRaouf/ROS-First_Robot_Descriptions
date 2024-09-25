# ROS-Robot-description-pkg

## Overview
This package contains the URDF and Xacro files for three different robot models: a Pan-Tilt robot, a 7-DOF robotic arm, and a differential wheeled robot. Each model is described using either URDF or Xacro formats, allowing for efficient, modular descriptions of the robots.

## what is a URDF?
URDF (Unified Robot Description Format) is an XML-based file format used in ROS to describe a robot's physical configuration. A URDF file specifies the robot’s links (rigid parts), joints (connections between links), sensors, actuators, and other components that define the structure and kinematics of the robot.

Using URDF files, you can visualize robots in tools like Rviz, simulate them in Gazebo, and control them in ROS.

## Xacro: A More Efficient Way to Define Robots
Xacro (XML Macros) is an extension of URDF that provides a more flexible and efficient way to define robots. Xacro files allow you to use macros, parameters, and reusability to avoid duplication and make the robot description more maintainable.

- With Xacro, you can create modular files, reuse components across different robots, and make your robot descriptions more readable and easier to maintain.
- You can define parameters and use them throughout the robot description to easily modify the robot's structure (e.g., joint limits, dimensions) without duplicating code.

### why use Xacro?
Xacro is especially useful when you have complex robots or multiple configurations, as it allows you to simplify the URDF descriptions. By converting repetitive or parameterized parts into macros, Xacro reduces redundancy and makes it easier to update or extend robot models.

## Robots in this Package

### 1. Pan-Tilt Robot
![25 09 2024_19 58 15_REC](https://github.com/user-attachments/assets/94e586df-e8e5-4029-9166-a3317b3f45a1)

This package contains my first URDF robot description, a Pan-Tilt robot. The Pan-Tilt robot is defined in both URDF and Xacro formats, allowing you to compare the two approaches.
- pan_tilt.urdf
- pan_tilt.xacro
you can visualize the robot using its launch file "view_pan_tilt.launch"
```bash
roslaunch robot_discription_pkg view_pan_tilt.launch
```
this launch file launches three nodes Robot_state_publisher , joint_state_publisher and rviz to visualize and control the robot's joints

### 2.7-DOF Robot Arm
![25 09 2024_20 00 02_REC](https://github.com/user-attachments/assets/94d13e4f-a018-4c2d-86f2-302ef7438aa3)

The next robot described in this package is a 7-DOF (Degrees of Freedom) robotic arm. This robot is defined using a Xacro file, which demonstrates the power of Xacro in creating complex robots with multiple joints and links.

seven_DOF_arm.xacro file is what contains the full description of the robotic arm, with 7 joints that allow for versatile manipulation tasks. The modularity of Xacro makes it easy to modify joint parameters and reuse components across different robots.

You can visualize the 7-DOF robot arm using:
```bash
roslaunch robot_discription_pkg view_arm.launch
```
### 3. Differential Wheeled Robot
![25 09 2024_20 10 41_REC](https://github.com/user-attachments/assets/07577e7e-44e9-439e-8c19-750f5ccbd62c)

For the third robot, a differential wheeled robot, I used modular Xacro files to describe its components, specifically focusing on the wheels and the base.

- wheel.urdf.xacro: This file describes the wheel of the differential wheeled robot, including its dimensions, joints, and link properties.

- diff_wheeled_robot.xacro: The full robot description, which includes the wheel.urdf.xacro file using the <xacro:include> tag. This modular approach makes it easier to manage the wheel description separately and include it in other robots if needed.

You can visualize the differential-wheeled-robot using:
```bash
roslaunch robot_discription_pkg view_mobile_robot.launch
```
