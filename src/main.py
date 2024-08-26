import rospy
from sensor_data import SensorData
from kalman_filter import KalmanFilter
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np


def main() -> None:
    """Main function to run the Kalman filter node."""
    rospy.loginfo("Starting Kalman Filter Node")
    ekf = KalmanFilter(
        x=[0.0, 0.0, 0.0, 0.0, 0.0],  # Initial state
        P=[0.1, 0.1, 0.1, 0.1, 0.1],  # Initial covariance
    )

    sensor_data = SensorData()
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        sensor_data.I_see_something = False
        sensor_data.callback_odom(Odometry())  # Simulate Odometry callback
        sensor_data.callback_imu(Imu())  # Simulate IMU callback
        ekf.predict(T=1.0, sigma_v=0.5, sigma_omega=0.1)
        error = ekf.estimate(measure=sensor_data)
        if error is not None:
            ekf.publish_message()
        rate.sleep()


if __name__ == "__main__":
    raise SystemExit(main())
