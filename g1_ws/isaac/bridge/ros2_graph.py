"""Cau noi Isaac Sim -> ROS2 bang OmniGraph.

KIEN TRUC - vi sao khong dung rclpy truc tiep:
  Isaac Sim chay Python 3.11 (conda), ROS2 Jazzy chay Python 3.12 (he
  thong). Hai ben KHONG import lan nhau duoc (rclpy bien dich cho 3.12).
  Giai phap: Isaac Sim phat topic bang thu vien ROS2 C++ NOI BO cua no
  (thu muc jazzy/ trong extension), con Nav2/SLAM chay tien trinh rieng.
  Hai ben gap nhau o tang DDS.

CAY TF - ai phat cai gi:
  map    -> odom     SLAM phat        (chua co o tang 0 nay)
  odom   -> pelvis   Isaac Sim phat   (IsaacComputeOdometry + RawTF)
  pelvis -> lidar    Isaac Sim phat   (PublishTransformTree)
  pelvis -> camera   Isaac Sim phat   (cung node tren)

HAI BUG DA GAP VA SUA O DAY:

  1. fullScan mac dinh = False
     -> LiDAR phat TUNG MANH quet (~5k diem @ 91Hz) thay vi nguyen vong
        (~27k diem @ 10Hz). SLAM 3D chuan mong doi vong quet hoan chinh.
     -> Fix: dat fullScan = True.

  2. chassisPrim tro sai cho
     -> Dat "/World/G1" thi node bao "prim is not a valid rigid body or
        articulation root", /odom KHONG duoc phat, va node chet LANG LE
        (chi thay khi doc log OmniGraph, khong co exception).
     -> Articulation root that nam o "/World/G1/pelvis".
     -> Bai hoc: OmniGraph nuot loi, phai chu dong doc log
        "omni.graph.core.plugin" khi topic khong xuat hien.

  3. frame_id cua camera KHONG KHOP giua topic va TF
     -> ROS2PublishTransformTree lay TEN PRIM lam frame_id, tuc "Camera"
        (viet hoa, do rep.create.camera dat ten). Nhung ROS2CameraHelper
        lai dat frameId = "camera" (viet thuong) neu ta go tay.
     -> Hai ten khac nhau -> RViz bao "frame camera does not exist" va
        KHONG ve duoc point cloud cua camera, du topic van co du lieu.
     -> Fix: lay frame_id cua camera TU TEN PRIM thay vi go tay.
     -> Bai hoc chung: frame_id trong message va frame trong TF phai
        khop TUNG KY TU, ke ca hoa/thuong.
"""
import omni.graph.core as og


def build_ros2_graph(lidar_rp_path, cam_rp_path=None, cam_prim_path=None,
                     chassis_prim="/World/G1/pelvis",
                     base_frame="pelvis", odom_frame="odom",
                     use_optical_frame=True,
                     optical_frame="camera_optical",
                     optical_rot_xyzw=(0.5, -0.5, 0.5, -0.5),
                     cam_offset=(0.08, 0.0, 0.62),
                     imu_prim_path=None,
                     graph_path="/ROS2Graph"):
    """Dung do thi phat: /clock, /points, /odom, /tf (+ camera neu co)."""
    keys = og.Controller.Keys

    nodes = [
        ("OnTick",   "omni.graph.action.OnPlaybackTick"),
        ("Context",  "isaacsim.ros2.bridge.ROS2Context"),
        ("SimTime",  "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PubClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("LidarPub", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
        ("PubTF",    "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ("Odom",     "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PubOdom",  "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("RawTF",    "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        # NHAN lenh van toc tu ngoai vao (teleop ban phim, hoac Nav2 sau nay).
        # Day dung kien truc cua robot that: Nav2 -> /cmd_vel -> SDK Unitree.
        ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
    ]
    connect = [
        ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
        ("Context.outputs:context", "SubTwist.inputs:context"),
        ("OnTick.outputs:tick", "PubClock.inputs:execIn"),
        ("OnTick.outputs:tick", "LidarPub.inputs:execIn"),
        ("OnTick.outputs:tick", "PubTF.inputs:execIn"),
        ("OnTick.outputs:tick", "Odom.inputs:execIn"),
        ("Odom.outputs:execOut", "PubOdom.inputs:execIn"),
        ("Odom.outputs:execOut", "RawTF.inputs:execIn"),
        ("Odom.outputs:position",        "PubOdom.inputs:position"),
        ("Odom.outputs:orientation",     "PubOdom.inputs:orientation"),
        ("Odom.outputs:linearVelocity",  "PubOdom.inputs:linearVelocity"),
        ("Odom.outputs:angularVelocity", "PubOdom.inputs:angularVelocity"),
        ("Odom.outputs:position",        "RawTF.inputs:translation"),
        ("Odom.outputs:orientation",     "RawTF.inputs:rotation"),
        ("Context.outputs:context", "PubClock.inputs:context"),
        ("Context.outputs:context", "LidarPub.inputs:context"),
        ("Context.outputs:context", "PubTF.inputs:context"),
        ("Context.outputs:context", "PubOdom.inputs:context"),
        ("Context.outputs:context", "RawTF.inputs:context"),
        ("SimTime.outputs:simulationTime", "PubClock.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "PubTF.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "PubOdom.inputs:timeStamp"),
        ("SimTime.outputs:simulationTime", "RawTF.inputs:timeStamp"),
    ]

    tf_targets = [(f"{chassis_prim}/lidar")]
    if cam_prim_path:
        tf_targets.append((cam_prim_path))
    # frame imu PHAI co trong TF, neu khong FAST-LIO khong noi duoc /imu voi
    # /points (va RViz bao "frame imu does not exist")
    if imu_prim_path:
        tf_targets.append((imu_prim_path))

    values = [
        ("PubClock.inputs:topicName", "/clock"),
        ("LidarPub.inputs:topicName", "/points"),
        ("LidarPub.inputs:frameId",   "lidar"),
        ("LidarPub.inputs:type",      "point_cloud"),
        ("LidarPub.inputs:fullScan",  True),          # BUG 1 - xem docstring
        ("LidarPub.inputs:renderProductPath", lidar_rp_path),
        ("PubTF.inputs:topicName",   "/tf"),
        ("PubTF.inputs:parentPrim",  [(chassis_prim)]),
        ("PubTF.inputs:targetPrims", tf_targets),
        ("Odom.inputs:chassisPrim",  [(chassis_prim)]),   # BUG 2 - xem docstring
        ("PubOdom.inputs:topicName",      "/odom"),
        ("PubOdom.inputs:odomFrameId",    odom_frame),
        ("PubOdom.inputs:chassisFrameId", base_frame),
        ("RawTF.inputs:topicName",     "/tf"),
        ("RawTF.inputs:parentFrameId", odom_frame),
        ("RawTF.inputs:childFrameId",  base_frame),
        ("SubTwist.inputs:topicName",  "/cmd_vel"),
    ]

    if cam_rp_path:
        # BUG 3: frame_id phai lay TU TEN PRIM (vd "Camera"), vi node phat
        # TF dung ten prim. Go tay "camera" se lech hoa/thuong -> RViz hong.
        cam_frame = cam_prim_path.rsplit("/", 1)[-1] if cam_prim_path else "camera"

        # BUG 4: point cloud do sau KHONG theo huong cua prim camera.
        #   Do thuc nghiem: xoay prim tu (90,0,-90) sang (90,0,0) roi nguoc
        #   lai, gia tri point cloud KHONG DOI mot chut nao:
        #       x[-1.44,-0.42]  y[-8.61,+8.58]  z[+2.44,+19.73]
        #   => ROS2CameraHelper luon xuat theo MOT quy uoc quang hoc co dinh.
        #
        #   Giai ma quy uoc do tu chinh so lieu (camera cao 1.42m so voi san):
        #       x: hep, am, bien do ~1.44 = do cao camera  -> x huong LEN
        #       y: doi xung rong                            -> y huong PHAI
        #       z: 2.4 den 19.7 = khoang cach               -> z huong TRUOC
        #
        #   Nen phat mot frame rieng 'camera_optical' gan THANG vao pelvis
        #   voi dung huong do (khong qua prim camera, de khong phu thuoc
        #   goc xoay prim):
        #       x_frame -> pelvis +Z (len), y_frame -> pelvis -Y (phai),
        #       z_frame -> pelvis +X (truoc)
        #   Quaternion tuong ung: w=0, x=0.7071, y=0, z=0.7071.
        #
        #   BUG 5 - THU TU QUATERNION: node RawTransformTree cua Isaac nhan
        #   theo thu tu (x, y, z, w), KHONG phai (w, x, y, z) nhu quy uoc
        #   USD. Truyen nham thu tu -> ma tran quay ra khac han:
        #       truyen [0, .707, 0, .707] (tuong nham la wxyz)
        #         -> TF thuc te xyzw = (0, .707, 0, .707)  -> R sai
        #       truyen [.707, 0, .707, 0] (dung xyzw)
        #         -> R = [[0,0,1],[0,-1,0],[1,0,0]]        -> DUNG
        #   Cach phat hien: doc nguoc TF bang tf2 va so ma tran quay thuc te
        #   voi ma tran mong doi - dung doan.
        #
        #   Phan cong: anh RGB dung frame prim (Camera), point cloud dung
        #   frame quang hoc (camera_optical).
        nodes += [
            ("CamRGB",    "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("CamPCL",    "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ]
        if use_optical_frame:
            nodes += [("OpticalTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree")]
        connect += [
            ("OnTick.outputs:tick", "CamRGB.inputs:execIn"),
            ("OnTick.outputs:tick", "CamPCL.inputs:execIn"),
            ("Context.outputs:context", "CamRGB.inputs:context"),
            ("Context.outputs:context", "CamPCL.inputs:context"),
        ]
        if use_optical_frame:
            connect += [
                ("OnTick.outputs:tick", "OpticalTF.inputs:execIn"),
                ("Context.outputs:context", "OpticalTF.inputs:context"),
                ("SimTime.outputs:simulationTime", "OpticalTF.inputs:timeStamp"),
            ]
        values += [
            ("CamRGB.inputs:topicName", "/camera/rgb"),
            ("CamRGB.inputs:frameId",   cam_frame),
            ("CamRGB.inputs:type",      "rgb"),
            ("CamRGB.inputs:renderProductPath", cam_rp_path),
            ("CamPCL.inputs:topicName", "/camera/depth_pcl"),
            ("CamPCL.inputs:frameId",   optical_frame if use_optical_frame else cam_frame),
            ("CamPCL.inputs:type",      "depth_pcl"),
            ("CamPCL.inputs:renderProductPath", cam_rp_path),
        ]
        if use_optical_frame:
            values += [
                ("OpticalTF.inputs:topicName",     "/tf"),
                ("OpticalTF.inputs:parentFrameId", base_frame),
                ("OpticalTF.inputs:childFrameId",  optical_frame),
                ("OpticalTF.inputs:translation",   list(cam_offset)),
                ("OpticalTF.inputs:rotation",      list(optical_rot_xyzw)),
            ]

    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {keys.CREATE_NODES: nodes, keys.CONNECT: connect, keys.SET_VALUES: values},
    )
    return f"{graph_path}/SubTwist"


def build_imu_graph(imu_prim_path, imu_frame="imu", topic="/imu",
                    graph_path="/IMUGraph"):
    """Do thi RIENG cho IMU, chay o tan so VAT LY (500Hz).

    === VI SAO PHAI LA MOT DO THI RIENG ===

    Da thu dat 3 node IMU vao chung /ROS2Graph va THAT BAI. Isaac Sim bao
    loi RO RANG trong log (nhung khong nem exception, nen script van chay
    binh thuong va /imu chi... khong bao gio xuat hien):

        [Error] [isaacsim.core.nodes] Physics OnSimulationStep node detected
        in a non on-demand Graph. Node will only trigger events if the parent
        Graph is set to compute on-demand. (/ROS2Graph/OnPhysics)

    Nguyen nhan: /ROS2Graph duoc tao voi pipeline stage mac dinh, chay theo
    NHIP RENDER. Node OnPhysicsStep chi hoat dong trong do thi ON-DEMAND -
    loai duoc goi tu trinh mo phong vat ly.

    === VI SAO KHONG DUNG OnTick CHO GON ===
    OnTick nham vao world.render(), o che do rl la 50 Hz. FAST-LIO can IMU
    >= 100 Hz (thuc te 200-500 Hz) de noi suy tu the trong mot vong quet
    100 ms. 50 Hz chi cho 5 mau moi vong quet - khong du de go meo.
    Do thi on-demand cho 500 Hz, tuc 50 mau moi vong quet.

    === BAI HOC CHUNG (lap lai lan thu 3 voi OmniGraph) ===
    OmniGraph KHONG nem exception khi cau hinh sai. Lan 1: chassisPrim sai
    -> /odom mat. Lan 2: frame_id lech hoa/thuong -> RViz khong ve. Lan 3:
    day. Moi lan trieu chung deu la "topic khong xuat hien". Phai chu dong
    grep log Isaac Sim, khong duoc doi exception.
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            # DONG QUYET DINH: on-demand -> OnPhysicsStep moi chay
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: [
                ("OnPhysics", "isaacsim.core.nodes.OnPhysicsStep"),
                ("Context",   "isaacsim.ros2.bridge.ROS2Context"),
                ("SimTime",   "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadIMU",   "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PubIMU",    "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            keys.CONNECT: [
                ("OnPhysics.outputs:step",  "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "PubIMU.inputs:execIn"),
                ("ReadIMU.outputs:linAcc",      "PubIMU.inputs:linearAcceleration"),
                ("ReadIMU.outputs:angVel",      "PubIMU.inputs:angularVelocity"),
                ("ReadIMU.outputs:orientation", "PubIMU.inputs:orientation"),
                ("Context.outputs:context",         "PubIMU.inputs:context"),
                ("SimTime.outputs:simulationTime",  "PubIMU.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("ReadIMU.inputs:imuPrim", [(imu_prim_path)]),
                # readGravity = True: IMU that DO duoc trong luc (dung yen
                # thi doc ra ~9.81 m/s2). FAST-LIO dua vao dieu nay de uoc
                # luong huong trong luc; dat False thi no khong hoi tu.
                ("ReadIMU.inputs:readGravity", True),
                ("PubIMU.inputs:topicName", topic),
                ("PubIMU.inputs:frameId",   imu_frame),
                ("PubIMU.inputs:publishLinearAcceleration", True),
                ("PubIMU.inputs:publishAngularVelocity",    True),
                ("PubIMU.inputs:publishOrientation",        True),
            ],
        },
    )
    return graph_path


def read_cmd_vel(twist_node_path, default=(0.0, 0.0, 0.0)):
    """Doc lenh van toc moi nhat nhan duoc tu /cmd_vel.

    Tra ve (vx, vy, omega). Neu chua co lenh nao thi tra ve `default`.
    Doc truc tiep thuoc tinh cua node OmniGraph - khong dung rclpy duoc
    vi Isaac Sim la Python 3.11 con ROS2 Jazzy la 3.12.
    """
    try:
        lin = og.Controller.get(og.Controller.attribute(f"{twist_node_path}.outputs:linearVelocity"))
        ang = og.Controller.get(og.Controller.attribute(f"{twist_node_path}.outputs:angularVelocity"))
        return float(lin[0]), float(lin[1]), float(ang[2])
    except Exception:
        return default
