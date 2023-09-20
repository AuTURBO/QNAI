import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

from sensor_msgs.msg import PointField
from sensor_msgs.msg import PointCloud2
from rclpy.time import Time

import math
import time

import numpy as np


class IMUPublisher(Node):

    def __init__(
        self,
        imu_topic_name: str = 'imu_data',
    ) -> None:
        rclpy.init()
        super().__init__('imu_publisher')
        self.imu_publisher = self.create_publisher(Imu, imu_topic_name, 10)

        self.ori = np.array([1.0, 0.0, 0.0, 0.0])
        self.lin_acc = np.array([0.0, 0.0, 0.0])
        self.ang_acc = np.array([0.0, 0.0, 0.0])

    def publish(self) -> None:

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_frame'

        # Simulated IMU data (you should replace this with your sensor data)
        imu_msg.orientation = Quaternion(
            w=self.ori[0], x=self.ori[1], y=self.ori[2],
            z=self.ori[3])  # Replace with actual quaternion data

        imu_msg.linear_acceleration = Vector3(
            x=float(self.lin_acc[0]),
            y=float(self.lin_acc[1]),
            z=float(self.lin_acc[2]
                   ))  # Replace with actual linear acceleration data

        imu_msg.angular_velocity = Vector3(
            x=float(self.ang_acc[0]),
            y=float(self.ang_acc[1]),
            z=float(
                self.ang_acc[2]))  # Replace with actual angular velocity data

        self.imu_publisher.publish(imu_msg)

    def update(
        self,
        lin: np.ndarray,
        ang: np.ndarray,
        ori: np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])) -> None:

        assert lin.shape == (3,)
        assert ang.shape == (3,)
        assert ori.shape == (4,)

        self.lin_acc = lin
        self.ang_acc = ang
        self.ori = ori


class LidarPublisher(Node):

    def __init__(self):
        super().__init__('lidar_publisher')
        self.publisher = self.create_publisher(PointCloud2, 'lidar_data', 10)
        self.points = []

    def publish(self) -> None:

        # PointCloud2 메시지 생성
        msg = PointCloud2()
        msg.header.frame_id = 'lidar_frame'
        msg.height = 1
        msg.width = len(self.points)
        msg.fields.append(
            PointField(name='x', offset=0, datatype=PointField.FLOAT32,
                       count=1))
        msg.fields.append(
            PointField(name='y', offset=4, datatype=PointField.FLOAT32,
                       count=1))
        msg.fields.append(
            PointField(name='z', offset=8, datatype=PointField.FLOAT32,
                       count=1))
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * len(self.points)
        msg.is_dense = False
        msg.data = self.points.tobytes()

        self.publisher.publish(msg)
        self.get_logger().info('Published 3D LiDAR data')

    def update(self, points: np.ndarray) -> None:
        self.points = points