# sensor_data.py

import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
from typing import Optional


class SensorData:
    def __init__(self) -> None:
        """Initialize the ROS node and setup subscribers."""
        rospy.init_node("ekf_dyn")

        self.odom_covariance: np.ndarray = np.diag(
            rospy.get_param("~odom_covariance", [0.0, 0.0, 0.0, 0.0])
        )
        self.imu_covariance: np.ndarray = np.diag(
            rospy.get_param("~imu_covariance", [0.0, 0.0])
        )

        self.odom_x: float = 0.0
        self.odom_y: float = 0.0
        self.odom_theta: float = 0.0
        self.odom_v: float = 0.0
        self.imu_theta: float = 0.0
        self.imu_omega: float = 0.0
        self.I_see_something: bool = False
        self.time_stamp: Optional[rospy.Time] = None

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.callback_imu)

    def callback_odom(self, msg: Odometry) -> None:
        """Callback for Odometry messages to update the current odometry state."""
        self.time_stamp = msg.header.stamp
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_theta = euler_from_quaternion(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            ]
        )[2]
        self.odom_v = msg.twist.twist.linear.x
        self.I_see_something = True

    def callback_imu(self, msg: Imu) -> None:
        """Callback for IMU messages to update the current IMU state."""
        if self.time_stamp is None or msg.header.stamp < self.time_stamp:
            return

        self.imu_theta = euler_from_quaternion(
            [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w,
            ]
        )[2]
        self.imu_omega = msg.angular_velocity.z
        self.I_see_something = True
