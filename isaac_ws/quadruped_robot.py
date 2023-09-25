"""
This script loads the hospital environment and the quadruped robot.
Author: Jinwon Kim
Date: 2023-09-07
# TODO: deprecated, will be removed in the future
"""

import sys
import os
import argparse
import numpy as np

import carb
import omni

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils import stage
from omni.isaac.core import World
<<<<<<< HEAD
from omni.isaac.core.prims import GeometryPrim, XFormPrim
from omni.isaac.core.robots import Robot
from omni.isaac.range_sensor import _range_sensor
=======
from omni.isaac.core.robots import Robot
from omni.isaac.range_sensor import _range_sensor
from omni.isaac.core.utils.stage import is_stage_loading
>>>>>>> tmp_merge_branch

parser = argparse.ArgumentParser(description="Ros2 Bridge Sample")
parser.add_argument(
    "--ros2_bridge",
    default="omni.isaac.ros2_bridge-humble",
    nargs="?",
    choices=["omni.isaac.ros2_bridge", "omni.isaac.ros2_bridge-humble"],
)
args, unknown = parser.parse_known_args()

# enable ROS2 bridge extension
enable_extension(args.ros2_bridge)

# ROS2 packages
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class PointCloudPublisher(Node):
    """
    PointCloud2 publisher 
    """

    def __init__(self):
        super().__init__("point_cloud_publisher")
        self.publisher = self.create_publisher(PointCloud2,
                                               "point_cloud_topic", 10)
        timer_period = 1.0  # Publish at 1 Hz
        self.timer = self.create_timer(timer_period, self.publish_point_cloud)

    def publish_point_cloud(self, data):
        point_cloud_msg = PointCloud2()
        point_cloud_msg.header.stamp = self.get_clock().now().to_msg()
        point_cloud_msg.header.frame_id = "frame"  # Set the frame ID

        # point cloud data
        point_cloud_data = data.astype(np.float32)

        # Populate the PointCloud2 message fields
        # point_cloud_msg.height = 1
        # point_cloud_msg.width = 1
        point_cloud_msg.fields = [
            PointField(name="x",
                       offset=0,
                       datatype=PointField.FLOAT32,
                       count=1),
            PointField(name="y",
                       offset=4,
                       datatype=PointField.FLOAT32,
                       count=1),
            PointField(name="z",
                       offset=8,
                       datatype=PointField.FLOAT32,
                       count=1),
        ]
        point_cloud_msg.is_bigendian = False
        point_cloud_msg.point_step = 12  # 3 * 4 bytes (float32)
        point_cloud_msg.row_step = point_cloud_msg.point_step * data.size
        point_cloud_msg.is_dense = True
        point_cloud_msg.data = point_cloud_data.tobytes()

        self.publisher.publish(point_cloud_msg)
        # self.get_logger().info("Published a 3D point cloud.")


<<<<<<< HEAD
from utils.quad_util import QuadrupedRobot

PHYSICS_DOWNTIME = 1 / 400
RENDER_DOWNTIME = PHYSICS_DOWNTIME * 8

# Locate Isaac Sim assets folder to load environment and robot stages
=======
PHYSICS_DOWNTIME = 1 / 4000.0  # 400
RENDER_DOWNTIME = PHYSICS_DOWNTIME * 8

simulation_app.update()

world = World(stage_units_in_meters=1.0,
              physics_dt=PHYSICS_DOWNTIME,
              rendering_dt=RENDER_DOWNTIME)

# Locate Isaac Sim assets folder to load environment and robot stages

>>>>>>> tmp_merge_branch
# Get the directory of the currently executing Python script
current_script_directory = os.path.dirname(os.path.abspath(__file__))

assets_root_path = current_script_directory
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

print("asset_path: ", assets_root_path)

<<<<<<< HEAD
# Stage = omni.usd.get_context().get_stage()

# simulation_app.update()
# Loading the hospital environment
# env_usd_path = os.path.join(assets_root_path, "Assets/Envs/hospital4.usd")
# stage.open_stage(env_usd_path)

# print("Loading hospital stage...")
# while stage.is_stage_loading():
#     simulation_app.update()
# print("Loading Complete")

# stage.add_reference_to_stage(env_usd_path, prim_path="/Environment/hospital")
# hospital = world.scene.add(
# XFormPrim(prim_path="/Environment/hospital", name="hospital"))

# Loading the world
world = World(stage_units_in_meters=1.0,
              physics_dt=PHYSICS_DOWNTIME,
              rendering_dt=RENDER_DOWNTIME)

world.scene.add_default_ground_plane()

# Loading the robot
robot_usd_path = os.path.join(assets_root_path, "Assets/Robots/go1.usd")
# stage.add_reference_to_stage(usd_path=robot_usd_path, prim_path="/World/go1")
# go1_robot = world.scene.add(Robot(prim_path="/World/go1", name="go1"))

go1_robot = world.scene.add(
    QuadrupedRobot(
        prim_path="/World/go1",
        name="go1",
        usd_path=robot_usd_path,
        position=[0.0, 0.0, 5.0],
        orientation=[0.0, 0.0, 0.0, 1.0],
    ))

print("Loading robot stage...")
while stage.is_stage_loading():
    simulation_app.update()
print("Loading Complete")

# lidar_path = "trunk/lidar_joint/lidar_sensor"

# Used to interact with the LIDAR
# lidarInterface = _range_sensor.acquire_lidar_sensor_interface()
=======
simulation_app.update()
# Loading the hospital environment
env_usd_path = os.path.join(assets_root_path, "Assets/Envs/hospital2.usd")
# stage.add_reference_to_stage(env_usd_path, "/World/hospital")
omni.usd.get_context().open_stage(env_usd_path, None)
# Wait two frames so that stage starts loading
simulation_app.update()
simulation_app.update()

print("Loading stage...")

while is_stage_loading():
    simulation_app.update()
print("Loading Complete")

# Loading the robot
robot_usd_path = os.path.join(assets_root_path, "Assets/Robots/go1.usd")
stage.add_reference_to_stage(usd_path=robot_usd_path, prim_path="/World/go1")
go1_robot = world.scene.add(Robot(prim_path="/World/go1", name="go1"))
lidar_path = "trunk/Lidar_shape/Lidar"

# Used to interact with the LIDAR
lidarInterface = _range_sensor.acquire_lidar_sensor_interface()

go1_position = np.array([0.0, 0.0, 0.5])
go1_orientation = np.array([0.0, 0.0, 0.0, 1.0])

go1_robot.set_world_pose(position=go1_position, orientation=go1_orientation)
>>>>>>> tmp_merge_branch

simulation_app.update()
simulation_app.update()

# ROS2 node
rclpy.init(args=unknown)
node = rclpy.create_node("lidar_publisher")

# ROS2 publisher
publisher = node.create_publisher(PointCloud2, "lidar", 10)
PointCloud2_msg = PointCloud2()

<<<<<<< HEAD
lidar_pub = PointCloudPublisher()

# timeline = omni.timeline.get_timeline_interface()

go1_robot.initialize()

while simulation_app.is_running():
    # timeline.play()
    simulation_app.update()

    # lidar_pub.publish_point_cloud(points_data)

    # print("Point Cloud", point_cloud.shape)
    world.step()
    go1_robot.update()
    go1_robot.advance(
        dt=PHYSICS_DOWNTIME,
        goal=[0.0, 0.0, 0.0, 0.0],
    )
=======
timeline = omni.timeline.get_timeline_interface()

lidar_pub = PointCloudPublisher()

while simulation_app.is_running():
    timeline.play()
    simulation_app.update()

    points_data = lidarInterface.get_point_cloud_data(
        os.path.join(go1_robot.prim_path, lidar_path))

    lidar_pub.publish_point_cloud(points_data)

    # print("Point Cloud", point_cloud.shape)
    world.step()
>>>>>>> tmp_merge_branch

simulation_app.close()
