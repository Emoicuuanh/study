"""CHANG A - SLAM 2D: ep dam may 3D thanh /scan roi chay slam_toolbox.

    ros2 launch g1_perception slam_2d.launch.py

Duong ong:
    /points (PointCloud2, frame lidar)
        |
        | pointcloud_to_laserscan  - lay mot DAI CHIEU CAO, bo phan con lai
        v
    /scan (LaserScan, frame pelvis)
        |
        | slam_toolbox  - scan matching + pose graph + loop closure
        v
    /map (OccupancyGrid)  +  TF  map -> odom

VI SAO LAM 2D TRUOC khi lam 3D:
  1. Kiem chung ca duong ong (TF, dong bo thoi gian, he toa do) voi chi phi
     thap. Loi o day tim ra RE hon nhieu so voi khi da co FAST-LIO chen vao.
  2. Nav2 (tang 4 cua lo trinh) THUC SU can costmap 2D - buoc nay khong bi
     bo di sau nay.
  3. Thay duoc loop closure hoat dong bang MAT: ban do dang giat mot cai roi
     thang lai.

=== HAI CAI BAY DA GAP KHI DUNG FILE NAY ===

BUG A: slam_toolbox la LIFECYCLE NODE, khong tu chay.
  Dung Node() thong thuong thi tien trinh len binh thuong, in ra
      "Node using stack size 40000000"
  ... roi NAM IM. `ros2 lifecycle get /slam_toolbox` tra ve "unconfigured".
  No KHONG subscribe /scan, KHONG phat /map, va khong bao loi gi.
  Phai phat event CONFIGURE, roi khi vao trang thai "inactive" thi phat
  tiep ACTIVATE. Do la viec cua configure_event/activate_event duoi day.

BUG B: pointcloud_to_laserscan la LAZY PUBLISHER.
  No chi subscribe /points KHI NAO co nguoi subscribe /scan:
      "Got a subscriber to laserscan, starting pointcloud subscriber"
      "No subscribers to laserscan, shutting down pointcloud subscriber"
  Ket hop voi BUG A thanh mot chuoi im lang hoan toan: slam_toolbox khong
  subscribe -> pointcloud_to_laserscan khong lam gi -> /scan trong -> de
  ket luan sai la "LiDAR hong" hoac "TF sai".
  BAI HOC: khi mot topic trong, kiem tra CA HAI DAU - ai phat va ai nghe.
  `ros2 topic info <topic> -v` cho biet so publisher va subscriber.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

# thu muc config nam canh thu muc launch nay
PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(PKG_DIR, "config")


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Phat odom -> pelvis_stab (pelvis da bo roll/pitch). PHAI co truoc
    # pointcloud_to_laserscan, neu khong thi moi quet bi drop vi thieu TF.
    stab = Node(
        package="g1_perception",
        executable="stabilize_frame.py",
        name="stabilize_frame",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    p2l = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[os.path.join(CONFIG, "pointcloud_to_laserscan.yaml"),
                    {"use_sim_time": use_sim_time}],
        remappings=[("cloud_in", "/points"), ("scan", "/scan")],
        output="screen",
    )

    slam = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        parameters=[os.path.join(CONFIG, "slam_toolbox_2d.yaml"),
                    {"use_sim_time": use_sim_time,
                     "use_lifecycle_manager": False}],
        output="screen",
    )

    # unconfigured -> inactive
    configure_event = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))

    # inactive -> active (chi khi da configure xong)
    activate_event = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam,
        start_state="configuring", goal_state="inactive",
        entities=[
            LogInfo(msg="[lifecycle] slam_toolbox: configure xong -> activate"),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam),
                transition_id=Transition.TRANSITION_ACTIVATE,
            )),
        ],
    ))

    return LaunchDescription([
        # use_sim_time PHAI la true: Isaac Sim co dong ho rieng phat ra /clock.
        # Neu node dung dong ho he thong thi moi quet den deu bi coi la "qua cu".
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        stab,
        p2l,
        slam,
        configure_event,
        activate_event,
    ])
