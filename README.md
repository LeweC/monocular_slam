# Monocular SLAM using MIDAS depth estimation on ROS2


Welcome to the Monocular SLAM with MIDAS Depth Estimation on ROS2 repository! This project combines state-of-the-art computer vision techniques with the power of the Robot Operating System 2 (ROS 2) framework to enable monocular Simultaneous Localization and Mapping (SLAM) on a compact driving hardware platform.

## Overview

Monocular SLAM is a fundamental technology in robotics and autonomous navigation. It allows a robot to understand its environment and estimate its own position and orientation within an environment using only a single camera. This project uses MIDAS, a powerful deep learning-based depth estimation model, to generate dense depth maps from the camera's RGB images. These depth maps are then integrated into ROS2 to enable real-time SLAM, allowing our hardware platform to navigate and interact with its environment.

## Key features

**MIDAS Depth Estimation**: 
We use the MIDAS model, a deep neural network designed for monocular depth estimation, to produce accurate depth maps from a single camera feed.

**ROS 2 Integration: 
The project is based on the ROS 2 framework, which provides a flexible and modular architecture for robotics applications. We are using ROS 2 primarily to provide seamless communication between the different components of the system, making it easier to develop, test and deploy.
The ROS 2 framework is great for the project because there are a few things we do not have to worry about when using ROS:
- Networking and communication between the robot and a computer: With ROS2 they just need to be on the same network and we can send ROS messages between them.
- Multithreading and modularity: In ROS each process is represented by a node. ROS nodes are autonomous processes that perform specific tasks or functions and communicate with each other using the ROS communication infrastructure. Each node can be started and stopped individually. 
  
There are a few packages here that we have not written ourselves. These are open-source packages for reading our USB camera and, more importantly, sending it as a ROS message type, so that we can access it from a different node and don't have to worry about threading or timing. All four are running on the robot.
- image_common: https://github.com/ros-perception/image_common
- image_pipeline: https://github.com/ros-perception/image_pipeline
- opencv-cam: https://github.com/clydemcqueen/opencv_cam
- ros2_shared: https://github.com/ptrmu/ros2_shared
  
The robot_control package runs on the robot's Rasberry Pi and is used to control the motors. It receives control commands from the server package, also via the ROS messaging system.
The server package runs on a computer on the same network as the Raspberry Pi. We use this to do some of the heavy lifting, so the robot does not need to load an NN or visualize and save the map. 
For visualization, we use [RViz](http://wiki.ros.org/rviz). More specifically, we use a point cloud that we send via a message for RViz to display. 

**Monocular SLAM**: 
By combining depth estimation with visual odometry, our system performs monocular SLAM, enabling the hardware platform to create maps of its environment while simultaneously localizing itself within those maps.

## Proof of Work
Before we started working on the live monocular SLAM, we did a proof of concept to make sure that our idea would work.
We took four images manually over an 80-degree span and used the same script to estimate the depth. We saved the resulting depth values from this calculation in a file.\
<img src="docs/001.PNG" width="500"> <img src="docs/002.PNG" width="500"> <img src="docs/003.PNG" width="500"> <img src="docs/004.PNG" width="500">\
Later we read this file into the visualisation script. There we hard-coded the angle that each image was taken from, did some triangulation, and sent the resulting point cloud to RViz.\
<img src="docs/DepthMap.PNG">\


