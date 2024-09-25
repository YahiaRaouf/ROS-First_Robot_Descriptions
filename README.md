# ROS-Robot-description-pkg

## Overview
This package contains the URDF and Xacro files for three different robot models: a Pan-Tilt robot, a 7-DOF robotic arm, and a differential wheeled robot. Each model is described using either URDF or Xacro formats, allowing for efficient, modular descriptions of the robots.

## what is a URDF?
URDF (Unified Robot Description Format) is an XML-based file format used in ROS to describe a robot's physical configuration. A URDF file specifies the robot’s links (rigid parts), joints (connections between links), sensors, actuators, and other components that define the structure and kinematics of the robot.

Using URDF files, you can visualize robots in tools like Rviz, simulate them in Gazebo, and control them in ROS.

## Xacro: A More Efficient Way to Define Robots
Xacro (XML Macros) is an extension of URDF that provides a more flexible and efficient way to define robots. Xacro files allow you to use macros, parameters, and reusability to avoid duplication and make the robot description more maintainable.

.With Xacro, you can create modular files, reuse components across different robots, and make your robot descriptions more readable and easier to maintain.
.You can define parameters and use them throughout the robot description to easily modify the robot's structure (e.g., joint limits, dimensions) without duplicating code.
