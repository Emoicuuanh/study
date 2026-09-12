"""TANG 0, Buoc D - Cau noi ROS2 DAY DU: point cloud + odometry + TF + clock.

KIEN TRUC (Isaac Sim py3.11 va ROS2 he thong py3.12 KHONG import lan nhau,
noi chuyen qua DDS):

  [Isaac Sim]  --DDS-->  [ROS2 Jazzy he thong]
   /clock                  slam_toolbox / FAST-LIO
   /points  (PointCloud2, ~27k diem @10Hz)
   /odom    (nav_msgs/Odometry)
   /tf      odom -> pelvis -> lidar

CAY TF - ai phat gi:
   map   -> odom     : SLAM phat (buoc E, chua co)
   odom  -> pelvis   : Isaac Sim phat (IsaacComputeOdometry + RawTF)
   pelvis-> lidar    : Isaac Sim phat (PublishTransformTree)

LUU Y: fullScan=True de phat NGUYEN vong quet (~27k diem @10Hz) thay vi
tung manh (~5k diem @90Hz) - chuan hon cho SLAM 3D.
"""
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path
enable_extension("isaacsim.ros2.bridge")

from lidar_utils import create_lidar
import omni.graph.core as og
import omni.replicator.core as rep

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

assets_root = get_assets_root_path()
add_reference_to_stage(assets_root + "/Isaac/Robots/Unitree/G1/g1.usd", "/World/G1")
for px, py in [(3.0, 0.0), (0.0, 3.0), (-3.0, 1.0), (2.0, -2.5), (-2.0, -3.0)]:
    rep.create.cube(position=(px, py, 0.5), scale=(0.6, 0.6, 1.0))

robot = Articulation(prim_paths_expr="/World/G1", name="g1")
world.scene.add(robot)
lidar = create_lidar("/World/G1/pelvis/lidar", "os0", [0.0, 0.0, 0.45])

world.reset()
lidar.initialize()
rp_path = lidar.get_render_product_path()

keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/ROS2Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick",   "omni.graph.action.OnPlaybackTick"),
            ("Context",  "isaacsim.ros2.bridge.ROS2Context"),
            ("SimTime",  "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("LidarPub", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ("PubTF",    "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ("Odom",     "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PubOdom",  "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("RawTF",    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "PubClock.inputs:execIn"),
            ("OnTick.outputs:tick", "LidarPub.inputs:execIn"),
            ("OnTick.outputs:tick", "PubTF.inputs:execIn"),
            ("OnTick.outputs:tick", "Odom.inputs:execIn"),
            ("Odom.outputs:execOut", "PubOdom.inputs:execIn"),
            ("Odom.outputs:execOut", "RawTF.inputs:execIn"),
            # du lieu odometry -> ca topic /odom lan TF odom->pelvis
            ("Odom.outputs:position",        "PubOdom.inputs:position"),
            ("Odom.outputs:orientation",     "PubOdom.inputs:orientation"),
            ("Odom.outputs:linearVelocity",  "PubOdom.inputs:linearVelocity"),
            ("Odom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
            ("Odom.outputs:position",        "RawTF.inputs:translation"),
            ("Odom.outputs:orientation",     "RawTF.inputs:rotation"),
            # context + thoi gian
            ("Context.outputs:context", "PubClock.inputs:context"),
            ("Context.outputs:context", "LidarPub.inputs:context"),
            ("Context.outputs:context", "PubTF.inputs:context"),
            ("Context.outputs:context", "PubOdom.inputs:context"),
            ("Context.outputs:context", "RawTF.inputs:context"),
            ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime", "PubTF.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
            ("SimTime.outputs:simulationTime", "RawTF.inputs:timeStamp"),
        ],
        keys.SET_VALUES: [
            ("PubClock.inputs:topicName", "/clock"),
            ("LidarPub.inputs:topicName", "/points"),
            ("LidarPub.inputs:frameId",   "lidar"),
            ("LidarPub.inputs:type",      "point_cloud"),
            ("LidarPub.inputs:fullScan",  True),
            ("LidarPub.inputs:renderProductPath", rp_path),
            ("PubTF.inputs:topicName",   "/tf"),
            ("PubTF.inputs:parentPrim",  [("/World/G1/pelvis")]),
            ("PubTF.inputs:targetPrims", [("/World/G1/pelvis/lidar")]),
            ("Odom.inputs:chassisPrim",  [("/World/G1/pelvis")]),   # articulation root that (buoc B)
            ("PubOdom.inputs:topicName",      "/odom"),
            ("PubOdom.inputs:odomFrameId",    "odom"),
            ("PubOdom.inputs:chassisFrameId", "pelvis"),
            ("RawTF.inputs:topicName",     "/tf"),
            ("RawTF.inputs:parentFrameId", "odom"),
            ("RawTF.inputs:childFrameId",  "pelvis"),
        ],
    },
)
print(">>> DO THI ROS2 (clock + points + odom + tf) DA TAO", flush=True)

n = robot.num_dof
robot.set_gains(kps=np.full((1, n), 400.0), kds=np.full((1, n), 40.0))
robot.set_joint_position_targets(np.zeros((1, n)))

DT, H = 1.0 / 60.0, 0.80
x, y, yaw = 0.0, 0.0, 0.0
print(">>> Dang phat ROS2 topic. Terminal khac: ros2 topic list / rviz2", flush=True)
for i in range(18000):        # 300 giay
    yaw += 0.3 * DT
    x += 0.4 * np.cos(yaw) * DT
    y += 0.4 * np.sin(yaw) * DT
    robot.set_world_poses(
        positions=np.array([[x, y, H]]),
        orientations=np.array([[np.cos(yaw/2), 0.0, 0.0, np.sin(yaw/2)]]),
    )
    robot.set_velocities(np.zeros((1, 6)))
    world.step(render=True)
    if i % 600 == 0:
        print(f">>> t={i*DT:.0f}s", flush=True)

app.close()
