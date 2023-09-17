"""
This script loads the hospital environment and the mobile robot.
Author: Jinwon Kim
Date: 2023-09-07
"""

import sys
import os
import argparse

import carb
import omni

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils import stage
from omni.isaac.core import World
from omni.isaac.core.robots import Robot

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

PHYSICS_DOWNTIME = 1 / 400.0
RENDER_DOWNTIME = PHYSICS_DOWNTIME * 8

simulation_app.update()

world = World(stage_units_in_meters=1.0,
              physics_dt=PHYSICS_DOWNTIME,
              rendering_dt=RENDER_DOWNTIME)

# Locate Isaac Sim assets folder to load environment and robot stages

# Get the directory of the currently executing Python script
current_script_directory = os.path.dirname(os.path.abspath(__file__))

assets_root_path = current_script_directory
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

print("asset_path: ", assets_root_path)

simulation_app.update()
# Loading the hospital environment
env_usd_path = os.path.join(assets_root_path,
                            "Assets/Envs/Hospital/hospital.usd")
stage.add_reference_to_stage(env_usd_path, "/World/hospital")

simulation_app.update()

# Loading the robot
robot_usd_path = os.path.join(assets_root_path,
                              "Assets/Robots/Carter/carter_v2.usd")
stage.add_reference_to_stage(usd_path=robot_usd_path,
                             prim_path="/World/carter")
carter_robot = world.scene.add(Robot(prim_path="/World/carter", name="carter"))

simulation_app.update()
simulation_app.update()

timeline = omni.timeline.get_timeline_interface()

while simulation_app.is_running():
    timeline.play()
    simulation_app.update()
    world.step()

simulation_app.close()
