# EKF for Pose Estimation

This ROS package implements an Extended Kalman Filter (EKF) for estimating the pose of a robot using odometry and IMU data. It is designed to run on a Raspberry Pi with ROS and provides a robust method for fusing sensor data to improve pose estimation accuracy.

## Overview

The ekf_pose_estimation package includes:

- *KalmanClass*: Implements the Extended Kalman Filter for state estimation.
- *Caller*: Manages sensor data subscriptions, performs EKF updates, and publishes the estimated pose.

## Features

- *State Prediction*: Predicts robot's state using odometry and IMU data.
- *State Update*: Updates the state estimate with incoming sensor measurements.
- *Message Publishing*: Publishes the estimated pose to the /pose_combined topic.

## Installation

1. *Install Dependencies*

   Install the required Python packages:

   ```
   pip3 install numpy scipy rospy```
