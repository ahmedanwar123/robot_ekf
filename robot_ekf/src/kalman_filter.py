# kalman_filter.py

import numpy as np
import rospy
from scipy.linalg import block_diag
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose2D
from typing import Optional, Type
from sensor_data import SensorData  # Import the SensorData class


def normalize_angle(angle: float) -> float:
    """Normalize angle to the range [-pi, pi]."""
    y = angle % (2 * np.pi)
    if y > np.pi:
        y -= 2 * np.pi
    return y


class KalmanFilter:
    def __init__(self, x: list[float], P: list[float]) -> None:
        """Initialize the Kalman filter."""
        self.not_first_time: bool = False
        self.previous_x: np.matrix = np.matrix(x).reshape(-1, 1)
        self.previous_P: np.ndarray = np.diag(P)
        self.estimated_x: np.matrix = self.previous_x
        self.estimated_P: np.ndarray = self.previous_P
        self.C_redu: np.matrix = np.matrix(
            [
                [1, 0, 0, 0, 0],
                [0, 1, 0, 0, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 1, 0],
                [0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1],
            ]
        )

    def predict(self, T: float, sigma_v: float, sigma_omega: float) -> None:
        """Predict the next state and covariance matrix."""
        self.predicted_x: np.matrix = np.copy(self.estimated_x)
        self.predicted_x[2, 0] = normalize_angle(
            self.estimated_x[2, 0] + self.estimated_x[4, 0] * T
        )
        self.predicted_x[0, 0] = self.estimated_x[0, 0] + self.estimated_x[
            3, 0
        ] * T * np.cos(self.predicted_x[2, 0])
        self.predicted_x[1, 0] = self.estimated_x[1, 0] + self.estimated_x[
            3, 0
        ] * T * np.sin(self.predicted_x[2, 0])

        ang = normalize_angle(self.estimated_x[2, 0] + T * self.estimated_x[4, 0])
        d_f: np.matrix = np.matrix(
            [
                [T * np.cos(ang), -T * self.estimated_x[3, 0] * np.sin(ang)],
                [T * np.sin(ang), T * self.estimated_x[3, 0] * np.cos(ang)],
                [0, T],
                [1, 0],
                [0, 1],
            ]
        )
        d_f_prime: np.matrix = np.matrix(
            [
                [
                    1,
                    0,
                    -T * self.estimated_x[3, 0] * np.sin(ang),
                    T * np.cos(ang),
                    -T * self.estimated_x[3, 0] * np.sin(ang),
                ],
                [
                    0,
                    1,
                    T * self.estimated_x[3, 0] * np.cos(ang),
                    T * np.sin(ang),
                    T * self.estimated_x[3, 0] * np.cos(ang),
                ],
                [0, 0, 1, 0, T],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1],
            ]
        )

        var_Q: np.matrix = np.matrix([[sigma_v**2, 0], [0, sigma_omega**2]])
        Q: np.matrix = d_f @ var_Q @ d_f.T
        self.predicted_P: np.ndarray = d_f_prime @ self.estimated_P @ d_f_prime.T + Q

        self.previous_x = self.predicted_x
        self.previous_P = self.predicted_P

    def estimate(self, measure: "SensorData") -> Optional[np.matrix]:
        """Estimate the state based on the given measurements."""
        if measure.I_see_something:
            z: np.matrix = np.matrix(
                [
                    [measure.odom_x],
                    [measure.odom_y],
                    [measure.odom_theta],
                    [measure.odom_v],
                    [measure.imu_theta],
                    [measure.imu_omega],
                ]
            )
            C: np.matrix = self.C_redu
            R: np.matrix = np.matrix(
                block_diag(measure.odom_covariance, measure.imu_covariance)
            )
        else:
            return None

        S: np.matrix = C @ self.previous_P @ C.T + R
        S += np.eye(S.shape[0]) * 1e-6

        try:
            self.K: np.matrix = self.previous_P @ C.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError as e:
            rospy.logwarn(f"Matrix inversion failed: {e}")
            return None

        error: np.matrix = z - (C @ self.previous_x)
        error[2, 0] = normalize_angle(error[2, 0])

        if self.not_first_time:
            self.estimated_x = self.previous_x + self.K @ error
            self.estimated_x[2, 0] = normalize_angle(self.estimated_x[2, 0])
            mat: np.matrix = np.eye(5) - self.K @ C
            self.estimated_P = mat @ self.previous_P
        else:
            self.not_first_time = True
        rospy.loginfo(
            "Estimated Pose: x=%f, y=%f, theta=%f",
            self.estimated_x[0, 0],
            self.estimated_x[1, 0],
            self.estimated_x[2, 0],
        )

        return error

    def publish_message(self) -> None:
        """Publish the estimated pose as a Pose2D message."""
        pub = rospy.Publisher("/pose_combined", Pose2D, queue_size=10)
        msg_pose = Pose2D()
        x: float = self.estimated_x[0, 0]
        y: float = self.estimated_x[1, 0]
        theta: float = self.estimated_x[2, 0]
        msg_pose.x = x
        msg_pose.y = y
        msg_pose.theta = theta * (180 / np.pi)
        pub.publish(msg_pose)
        rospy.loginfo("Publishing Pose: x=%f, y=%f, theta=%f", x, y, theta)
