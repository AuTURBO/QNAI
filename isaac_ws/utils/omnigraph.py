"""
This file is for generating omnigraph.
creator: Jinwon Kim
"""

import omni.graph.core as og

class omnigraph_helper():
    """
    This class is for generating omnigraph.
    """

    def __init__(self, is_ros2=True):
        self._keys = og.Controller.Keys
        self._is_ros2 = is_ros2

        self.ros_vp_offset = 1
        if self._is_ros2:
            self._ros_version = "ROS2"
            self._ros_bridge_version = "ros2_bridge."
        else:
            self._ros_version = "ROS1"
            self._ros_bridge_version = "ros_bridge."
        pass
    
    def ros_clock(self):
        try:
            (self._clock_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": "/ROS_Clock",
                    "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
                },
                {
                    self._keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("readSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                        ("publishClock", "omni.isaac." + self._ros_bridge_version + self._ros_version + "PublishClock"),
                    ],
                    self._keys.CONNECT: [
                        ("OnTick.outputs:tick", "publishClock.inputs:execIn"),
                        ("readSimTime.outputs:simulationTime", "publishClock.inputs:timeStamp"),
                    ],
                },
            )
            return True
        
        except Exception as e:
            print(e)

            return False

    def ros_imu(self, prim_path):
        try:
            (self._on_tick, _, _, _) = og.Controller.edit(
                {
                    "graph_path": "/ROS_IMU",
                    "evaluator_name": "execution",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
                },
                {
                    self._keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        # ("IsaacReadIMUNode")
                    ]
                },
            )
            return True
        
        except Exception as e:
            print(e)

            return False