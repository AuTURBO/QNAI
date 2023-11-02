"""
This file is anymal standalone python file.
"""

# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.appwindow  # Contains handle to keyboard

from omni.isaac.core import World

from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.core.utils.nucleus import get_assets_root_path

from utils.anymal import Anymal

import numpy as np
import carb

import sys
import os


class AnymalRunner(object):
    """[summary]
    Main class to run the simulation
    """

    def __init__(self, physics_dt, render_dt) -> None:
        """
        Summary

        creates the simulation world with preset physics_dt and render_dt and creates an anymal robot inside the warehouse

        Argument:
        physics_dt {float} -- Physics downtime of the scene.
        render_dt {float} -- Render downtime of the scene.

        """

        self._world = World(stage_units_in_meters=1.0,
                            physics_dt=physics_dt,
                            rendering_dt=render_dt)

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")

        current_script_directory = os.path.dirname(os.path.abspath(__file__))

        assets_root_path = current_script_directory
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            simulation_app.close()
            sys.exit()

        print("asset_path: ", assets_root_path)

        # spawn scene
        prim = get_prim_at_path("/World/hospital")
        if not prim.IsValid():
            prim = define_prim("/World/hospital", "Xform")
            env_asset_path = os.path.join(assets_root_path,
                                          "Assets/Envs/hospital.usd")
            print(env_asset_path)
            prim.GetReferences().AddReference(env_asset_path)

        robot_usd_path = os.path.join(assets_root_path,
                                      "Assets/Robots/anymal_c.usd")

        self._anymal = self._world.scene.add(
            Anymal(prim_path="/World/Anymal",
                   name="Anymal",
                   usd_path=robot_usd_path,
                   position=np.array([0, 0, 0.70]),
                   ros_version="humble",
                   ros_domain_id="10"))

        self._world.reset()
        self._enter_toggled = 0
        self._base_command = np.zeros(3)

        # bindings for keyboard to command
        self._input_keyboard_mapping = {
            # forward command
            "NUMPAD_8": [1.0, 0.0, 0.0],
            "UP": [1.0, 0.0, 0.0],
            # back command
            "NUMPAD_2": [-1.0, 0.0, 0.0],
            "DOWN": [-1.0, 0.0, 0.0],
            # left command
            "NUMPAD_6": [0.0, -1.0, 0.0],
            "RIGHT": [0.0, -1.0, 0.0],
            # right command
            "NUMPAD_4": [0.0, 1.0, 0.0],
            "LEFT": [0.0, 1.0, 0.0],
            # yaw command (positive)
            "NUMPAD_7": [0.0, 0.0, 1.0],
            "N": [0.0, 0.0, 1.0],
            # yaw command (negative)
            "NUMPAD_9": [0.0, 0.0, -1.0],
            "M": [0.0, 0.0, -1.0],
        }
        self.needs_reset = False

    def setup(self) -> None:
        """
        [Summary]

        Set up keyboard listener and add physics callback

        """
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._sub_keyboard_event)
        self._world.add_physics_callback("anymal_advance",
                                         callback_fn=self.on_physics_step)

    def on_physics_step(self, step_size) -> None:
        """
        [Summary]

        Physics call back, switch robot mode and call robot advance function to compute and apply joint torque

        """
        if self.needs_reset:
            self._world.reset(True)
            self.needs_reset = False
        self._anymal.advance(step_size, self._base_command)

    def run(self) -> None:
        """
        [Summary]

        Step simulation based on rendering downtime

        """
        # change to sim running
        while simulation_app.is_running():
            self._world.step(render=True)
            if not self._world.is_simulating():
                self.needs_reset = True

    def _sub_keyboard_event(self, event) -> bool:
        """
        [Summary]

        Keyboard subscriber callback to when kit is updated.

        """
        # reset event
        self._event_flag = False
        # when a key is pressed for released  the command is adjusted w.r.t the key-mapping
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # on pressing, the command is incremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command[0:3] += np.array(
                    self._input_keyboard_mapping[event.input.name])

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # on release, the command is decremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command[0:3] -= np.array(
                    self._input_keyboard_mapping[event.input.name])
        return True


def main():
    """
    [Summary]

    Parse arguments and instantiate the ANYmal runner

    """
    physics_dt = 1 / 200.0
    render_dt = 1 / 60.0

    runner = AnymalRunner(physics_dt=physics_dt, render_dt=render_dt)
    simulation_app.update()
    runner.setup()

    # an extra reset is needed to register
    runner._world.reset()  # pylint: disable=protected-access
    runner._world.reset()  # pylint: disable=protected-access
    runner.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
