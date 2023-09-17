from omni.isaac.core.utils.stage import is_stage_loading
from omni.isaac.core import World
import os
import sys

import carb
import omni

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

PHYSICS_DOWNTIME = 1 / 4000.0  # 400
RENDER_DOWNTIME = PHYSICS_DOWNTIME * 8

test = 1

world = World(stage_units_in_meters=1.0,
              physics_dt=PHYSICS_DOWNTIME,
              rendering_dt=RENDER_DOWNTIME)

# Get the directory of the currently executing Python script
current_script_directory = os.path.dirname(os.path.abspath(__file__))

assets_root_path = current_script_directory
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Loading the hospital environment
env_usd_path = os.path.join(assets_root_path, "Assets/Envs/hospital3.usd")

print("env_usd_path: ", env_usd_path)

omni.usd.get_context().open_stage(env_usd_path, None)

# Wait two frames so that stage starts loading
simulation_app.update()
simulation_app.update()

print("Loading stage...")

while is_stage_loading():
    simulation_app.update()
print("Loading Complete")

timeline = omni.timeline.get_timeline_interface()
# timeline.play()

while simulation_app.is_running():
    timeline.play()
    simulation_app.update()

    # # print("Point Cloud", point_cloud.shape)
    world.step()
