# Copyright (c) 2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# TODO: deprecated, will be removed in the future

import omni
import omni.kit.commands
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import get_prim_at_path, define_prim
from omni.isaac.sensor import _sensor

from omni.isaac.core.utils.stage import get_current_stage, get_stage_units
from omni.isaac.core.articulations import Articulation
from omni.isaac.quadruped.utils.a1_classes import A1State, A1Measurement, A1Command
from omni.isaac.quadruped.controllers import A1QPController
from omni.isaac.sensor import IMUSensor
from omni.isaac.range_sensor import _range_sensor
from typing import Optional, List
from collections import deque
import numpy as np
import carb

import os


class QuadrupedRobot(Articulation):

    def __init__(self,
                 prim_path: str,
                 name: str = "go1",
                 physics_dt: Optional[float] = 1 / 400.0,
                 usd_path: Optional[str] = None,
                 position: Optional[np.ndarray] = None,
                 orientation: Optional[np.ndarray] = None,
                 model: Optional[str] = "Go1") -> None:
        """
        [Summary]
        
        initialize robot, set up sensors and controller
        
        Args:
            prim_path {str} -- prim path of the robot on the stage
            name {str} -- name of the quadruped
            physics_dt {float} -- physics downtime of the controller
            usd_path {str} -- robot usd filepath in the directory
            position {np.ndarray} -- position of the robot
            orientation {np.ndarray} -- orientation of the robot
            model {str} -- robot model (can be either A1 or Go1)
        
        """
        self._stage = get_current_stage()
        self._prim_path = prim_path
        prim = get_prim_at_path(self._prim_path)

        if not prim.IsValid():
            prim = define_prim(self._prim_path, "Xform")
            if usd_path:
                prim.GetReferences().AddReference(usd_path)
            else:
                carb.log_error("Please specify the robot usd file path")

        self._measurement = A1Measurement()
        self._command = A1Command()
        self._state = A1State()
        self._default_a1_state = A1State()

        # lidar sensor setup
        self._lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
        self._lidar_path = "trunk/lidar_joint/lidar_sensor"
        self._lidar_points = None

        if position is not None:
            self._default_a1_state.base_frame.pos = np.asarray(position)
        else:
            self._default_a1_state.base_frame.pos = np.array([0.0, 0.0, 0.0])

        self._default_a1_state.base_frame.quat = np.array([0.0, 0.0, 0.0, 1.0])
        self._default_a1_state.base_frame.ang_vel = np.array([0.0, 0.0, 0.0])
        self._default_a1_state.base_frame.lin_vel = np.array([0.0, 0.0, 0.0])
        self._default_a1_state.joint_pos = np.array(
            [0.0, 1.2, -1.8, 0, 1.2, -1.8, 0.0, 1.2, -1.8, 0, 1.2, -1.8])
        self._default_a1_state.joint_vel = np.zeros(12)

        self._goal = np.zeros(3)
        self.meters_per_unit = get_stage_units()

        super().__init__(prim_path=self._prim_path,
                         name=name,
                         position=position,
                         orientation=orientation)

        # contact sensor setup
        self.feet_order = ["FL", "FR", "RL", "RR"]
        self.feet_path = [
            self._prim_path + "/FL_foot",
            self._prim_path + "/FR_foot",
            self._prim_path + "/RL_foot",
            self._prim_path + "/RR_foot",
        ]

        self.color = [(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1), (1, 1, 0, 1)]

        # imu sensor setup
        self.imu_path = self._prim_path + "/imu_link/imu_sensor"
        self._imu_sensor = IMUSensor(
            prim_path=self.imu_path,
            name="imu",
            dt=physics_dt,
            translation=np.array([0, 0, 0]),
            orientation=np.array([1, 0, 0, 0]),
        )
        self.base_lin = np.zeros(3)
        self.ang_vel = np.zeros(3)

        # Controller
        self.physics_dt = physics_dt
        self._qp_controller = A1QPController(model, self.physics_dt)
        self._qp_controller.setup()
        self._dof_control_modes: List[int] = list()

        return

    def set_state(self, state: A1State) -> None:
        """[Summary]
        
        Set the kinematic state of the robot.

        Args:
            state {A1State} -- The state of the robot to set.

        Raises:
            RuntimeError: When the DC Toolbox interface has not been configured.
        """
        self.set_world_pose(position=state.base_frame.pos,
                            orientation=state.base_frame.quat[[3, 0, 1, 2]])
        self.set_linear_velocity(state.base_frame.lin_vel)
        self.set_angular_velocity(state.base_frame.ang_vel)
        # joint_state from the DC interface now has the order of
        # 'FL_hip_joint',   'FR_hip_joint',   'RL_hip_joint',   'RR_hip_joint',
        # 'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
        # 'FL_calf_joint',  'FR_calf_joint',  'RL_calf_joint',  'RR_calf_joint'

        # while the QP controller uses the order of
        # FL_hip_joint FL_thigh_joint FL_calf_joint
        # FR_hip_joint FR_thigh_joint FR_calf_joint
        # RL_hip_joint RL_thigh_joint RL_calf_joint
        # RR_hip_joint RR_thigh_joint RR_calf_joint
        # we convert controller order to DC order for setting state
        self.set_joint_positions(positions=np.asarray(np.array(
            state.joint_pos.reshape([4, 3]).T.flat),
                                                      dtype=np.float32))
        self.set_joint_velocities(velocities=np.asarray(np.array(
            state.joint_vel.reshape([4, 3]).T.flat),
                                                        dtype=np.float32))
        self.set_joint_efforts(np.zeros_like(state.joint_pos))
        return

    def update_imu_sensor_data(self) -> None:
        """[summary]
        
        Updates processed imu sensor data from the robot body, store them in member variable base_lin and ang_vel
        """
        frame = self._imu_sensor.get_current_frame()
        self.base_lin = frame["lin_acc"]
        self.ang_vel = frame["ang_vel"]
        return

    def update_lidar_sensor_data(self) -> None:
        """[summary]
        
        Updates processed lidar sensor data from the robot body, store them in member variable base_lin and ang_vel
        """
        self._lidar_points = self._lidar_interface.get_point_cloud_data(
            os.path.join(self._prim_path, self._lidar_path))

        return

    def update(self) -> None:
        """[summary]
        
        update robot sensor variables, state variables in A1Measurement
        """

        self.update_imu_sensor_data()
        self.update_lidar_sensor_data()

        # joint pos and vel from the DC interface
        self.joint_state = super().get_joints_state()

        # joint_state from the DC interface now has the order of
        # 'FL_hip_joint',   'FR_hip_joint',   'RL_hip_joint',   'RR_hip_joint',
        # 'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
        # 'FL_calf_joint',  'FR_calf_joint',  'RL_calf_joint',  'RR_calf_joint'

        # while the QP controller uses the order of
        # FL_hip_joint FL_thigh_joint FL_calf_joint
        # FR_hip_joint FR_thigh_joint FR_calf_joint
        # RL_hip_joint RL_thigh_joint RL_calf_joint
        # RR_hip_joint RR_thigh_joint RR_calf_joint
        # we convert DC order to controller order for joint info
        self._state.joint_pos = np.array(
            self.joint_state.positions.reshape([3, 4]).T.flat)
        self._state.joint_vel = np.array(
            self.joint_state.velocities.reshape([3, 4]).T.flat)

        # base frame
        base_pose = self.get_world_pose()
        self._state.base_frame.pos = base_pose[0]
        self._state.base_frame.quat = base_pose[1][[1, 2, 3, 0]]
        self._state.base_frame.lin_vel = self.get_linear_velocity()
        self._state.base_frame.ang_vel = self.get_angular_velocity()

        # assign to _measurement obj
        self._measurement.state = self._state
        self._measurement.base_ang_vel = np.asarray(self.ang_vel)
        self._measurement.base_lin_acc = np.asarray(self.base_lin)
        return

    def advance(self,
                dt,
                goal,
                path_follow=False,
                auto_start=True) -> np.ndarray:
        """[summary]
        
        compute desired torque and set articulation effort to robot joints
        
        Argument:
        dt {float} -- Timestep update in the world.
        goal {List[int]} -- x velocity, y velocity, angular velocity, state switch
        path_follow {bool} -- true for following coordinates, false for keyboard control
        auto_start {bool} -- true for start trotting after 1 sec, false for start trotting after switch mode function is called

        Returns:
        np.ndarray -- The desired joint torques for the robot.
        """
        if goal is None:
            goal = self._goal
        else:
            self._goal = goal
        self.update()
        self._qp_controller.set_target_command(goal)

        self._command.desired_joint_torque = self._qp_controller.advance(
            dt, self._measurement, path_follow, auto_start)

        # joint_state from the DC interface now has the order of
        # 'FL_hip_joint',   'FR_hip_joint',   'RL_hip_joint',   'RR_hip_joint',
        # 'FL_thigh_joint', 'FR_thigh_joint', 'RL_thigh_joint', 'RR_thigh_joint',
        # 'FL_calf_joint',  'FR_calf_joint',  'RL_calf_joint',  'RR_calf_joint'

        # while the QP controller uses the order of
        # FL_hip_joint FL_thigh_joint FL_calf_joint
        # FR_hip_joint FR_thigh_joint FR_calf_joint
        # RL_hip_joint RL_thigh_joint RL_calf_joint
        # RR_hip_joint RR_thigh_joint RR_calf_joint
        # we convert controller order to DC order for command torque
        torque_reorder = np.array(
            self._command.desired_joint_torque.reshape([4, 3]).T.flat)
        self.set_joint_efforts(np.asarray(torque_reorder, dtype=np.float32))
        return self._command

    def initialize(self, physics_sim_view=None) -> None:
        """[summary]

        initialize dc interface, set up drive mode and initial robot state
        """
        super().initialize(physics_sim_view=physics_sim_view)
        self.get_articulation_controller().set_effort_modes("force")
        self.get_articulation_controller().switch_control_mode("effort")
        self.set_state(self._default_a1_state)
        return

    def post_reset(self) -> None:
        """[summary]

        post reset articulation and qp_controller
        """
        super().post_reset()

        self._qp_controller.reset()
        self.set_state(self._default_a1_state)
        return