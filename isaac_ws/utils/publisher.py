import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
import math
import time

import numpy as np

class IMUPublisher(Node):
    def __init__(self, 
                 imu_topic_name: str = 'imu_data',
                 hz: float = 100) -> None:
        rclpy.init()
        super().__init__('imu_publisher')
        self.imu_publisher = self.create_publisher(Imu, imu_topic_name, 10)
        self.timer = self.create_timer(1.0 / hz, self.publish_imu_data)
        self.seq = 0

        self.ori = np.array([1.0, 0.0, 0.0, 0.0])
        self.lin_acc = np.array([0.0, 0.0, 0.0])
        self.ang_acc = np.array([0.0, 0.0, 0.0])

    def publish_imu_data(self) -> None:

        imu_msg = Imu()
        imu_msg.header.stamp = 0.0
        imu_msg.header.frame_id = 'imu_frame'
        imu_msg.header.seq = self.seq

        # Simulated IMU data (you should replace this with your sensor data)
        imu_msg.orientation = Quaternion(w=self.ori[0], 
                                         x=self.ori[1], 
                                         y=self.ori[2], 
                                         z=self.ori[3])  # Replace with actual quaternion data
        
        imu_msg.linear_acceleration = Vector3(x=self.lin_acc[0], 
                                              y=self.lin_acc[1], 
                                              z=self.lin_acc[2])  # Replace with actual linear acceleration data
        
        imu_msg.angular_velocity = Vector3(x=self.ang_acc[0], 
                                           y=self.ang_acc[1], 
                                           z=self.ang_acc[2])  # Replace with actual angular velocity data


        self.imu_publisher.publish(imu_msg)
        self.seq += 1

    def update(self, 
                lin: np.ndarray,
                ang: np.ndarray,
                ori: np.ndarray= np.array([1.0, 0.0, 0.0, 0.0]) 
                ) -> None:
        
        assert lin.shape == (3, )
        assert ang.shape == (3, )
        assert ori.shape == (4, )

        self.ori = ori
        self.lin_acc = lin
        self.ang_acc = ang

    def initialize(self):
        self.seq = 0