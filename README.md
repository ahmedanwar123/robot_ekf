# EKF for Pose Estimation

This ROS package implements an Extended Kalman Filter (EKF) to estimate the pose of a robot using odometry and IMU data. It is designed to fuse sensor data from IMU and Encoders to improve pose estimation accuracy.

## Overview

The `robot_ekf` package includes:

- **KalmanClass**: Implements the Extended Kalman Filter for state estimation.
- **Caller**: Manages sensor data subscriptions, performs EKF updates, and publishes the estimated pose.

## Features

- **State Prediction**: Predicts the robot's state using odometry and IMU data.
- **State Update**: Updates the state estimate with incoming sensor measurements.
- **Message Publishing**: Publishes the estimated pose to the `/pose_combined` topic.

## Installation

1. **Install Dependencies**

   Install the required Python packages:
   ```bash
   pip3 install numpy scipy
   ```
   or
   ```bash
   sudo apt install numpy scipy
   ```

3. **Clone the Repository**

   Clone this repository to your ROS workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/ahmedanwar123/robot_ekf.git
   ```

4. **Build the Package**

   Build the package in your catkin workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

5. **Source the Workspace**

   Source the setup file to make the package available:
   ```bash
   source devel/setup.bash
   ```

## Configuration

Set the following parameters for EKF in the code:

- `~odom_covariance`: Covariance matrix for odometry measurements.
- `~imu_covariance`: Covariance matrix for IMU measurements.

**Example Configuration**:
```python
~odom_covariance =[0.1, 0.1, 0.1, 0.1]
~imu_covariance =[0.1, 0.1]
```

## Running the Node

To run the EKF node:
```bash
roslaunch robot_ekf main.py
```

To run the GUI for tuning covariance values:
```bash
python3 gui.py
```

**Node Description**

- `ekf_node/kalman_dyn.py`: Main script for running the EKF. It subscribes to `/odom` and `/imu_data`, performs EKF updates, and publishes the pose to `/pose_combined`. Note that `dyn` refers to dynamic, as covariances can be tuned while the system is running, which is very helpful.

## Message Types

**Pose2D** (from `geometry_msgs`):
```plaintext
float64 x       # X position in meters
float64 y       # Y position in meters
float64 theta   # Orientation in radians
```

**Imu** (from `sensor_msgs`):
```plaintext
Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
```

**Odometry** (from `nav_msgs`):
```plaintext
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

### Notes

- **Pose2D:** Used for publishing the combined pose estimated by the Kalman filter.
- **Imu:** Subscribed to get IMU data (orientation and angular velocity).
- **Odometry:** Subscribed to get odometry data (position, orientation, and velocity).
