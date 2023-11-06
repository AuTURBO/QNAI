"""
This file is sensor utils python file.
"""

import omni.kit.commands
from pxr import Gf


def create_imu_sensor(name,
                      parent,
                      sensor_period,
                      translation=Gf.Vec3d(0, 0, 0),
                      orientation=Gf.Quatd(1, 0, 0, 0)):
    success, _ = omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path=name,
        parent=parent,
        sensor_period=sensor_period,
        translation=translation,
        orientation=orientation,
    )
    return success
