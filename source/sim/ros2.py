"""Isaac Sim OmniGraph wiring for Indy7 ROS 2 communication."""

from __future__ import annotations


ROS2_GRAPH_PATH = "/World/ROS2ActionGraph"
JOINT_STATES_TOPIC = "/indy7/joint_states"
JOINT_COMMAND_TOPIC = "/indy7/joint_command"
CLOCK_TOPIC = "/clock"
TF_TOPIC = "/tf"


def create_ros2_action_graph(articulation_prim_path: str) -> str:
    """Create the in-stage ROS 2 Action Graph for an Indy7 articulation.

    The graph publishes joint states, simulation clock, and the articulation
    transform tree. It also subscribes to JointState commands and forwards
    them to Isaac Sim's articulation controller.
    """
    import omni.graph.core as og
    import usdrt.Sdf

    prim = [usdrt.Sdf.Path(articulation_prim_path)]
    keys = og.Controller.Keys

    og.Controller.edit(
        {"graph_path": ROS2_GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", True),
                ("PublishJointState.inputs:targetPrim", prim),
                ("PublishJointState.inputs:topicName", JOINT_STATES_TOPIC),
                ("SubscribeJointState.inputs:topicName", JOINT_COMMAND_TOPIC),
                ("ArticulationController.inputs:targetPrim", prim),
                ("PublishClock.inputs:topicName", CLOCK_TOPIC),
                ("PublishTF.inputs:targetPrims", prim),
                ("PublishTF.inputs:topicName", TF_TOPIC),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                ("ROS2Context.outputs:context", "PublishJointState.inputs:context"),
                ("ROS2Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("ROS2Context.outputs:context", "PublishClock.inputs:context"),
                ("ROS2Context.outputs:context", "PublishTF.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                (
                    "SubscribeJointState.outputs:positionCommand",
                    "ArticulationController.inputs:positionCommand",
                ),
                (
                    "SubscribeJointState.outputs:velocityCommand",
                    "ArticulationController.inputs:velocityCommand",
                ),
                (
                    "SubscribeJointState.outputs:effortCommand",
                    "ArticulationController.inputs:effortCommand",
                ),
            ],
        },
    )

    return ROS2_GRAPH_PATH
