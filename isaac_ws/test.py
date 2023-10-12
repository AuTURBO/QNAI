"""
This script is test file
Author: Jinwon Kim
Date: 2023-09-17
"""

import os
import sys

import carb

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni
from omni.isaac.core.utils import stage
from omni.isaac.core import World

PHYSICS_DOWNTIME = 1 / 4000.0  # 400
RENDER_DOWNTIME = PHYSICS_DOWNTIME * 8

# world.scene.add_default_ground_plane()

# Get the directory of the currently executing Python script
current_script_directory = os.path.dirname(os.path.abspath(__file__))

assets_root_path = current_script_directory
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Loading the hospital environment
stage.set_stage_up_axis()

env_usd_path = os.path.join(assets_root_path, "Assets/Envs/hospital4.usd")
stage.open_stage(env_usd_path)

print("Loading stage...")

while stage.is_stage_loading():
    simulation_app.update()
print("Loading Complete")

world = World(stage_units_in_meters=1.0,
              physics_dt=PHYSICS_DOWNTIME,
              rendering_dt=RENDER_DOWNTIME)

timeline = omni.timeline.get_timeline_interface()
world.reset()
while simulation_app.is_running():
    timeline.play()
    simulation_app.update()

    world.step()
