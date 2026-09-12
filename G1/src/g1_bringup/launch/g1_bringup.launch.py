import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Load configuration file
    bringup_dir = get_package_share_directory('g1_bringup')
    config_path = os.path.join(bringup_dir, 'config', 'g1_bringup.yaml')
    
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Configuration file not found: {config_path}")
        
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    tfs = config.get('static_transforms', {})

    # 1. Evaluate launch configurations as python strings
    developer_mode = context.perform_substitution(LaunchConfiguration('developer_mode'))
    model = context.perform_substitution(LaunchConfiguration('model'))
    lowstate_topic = context.perform_substitution(LaunchConfiguration('lowstate_topic'))
    sync_lidar_time = context.perform_substitution(LaunchConfiguration('sync_lidar_time'))
    use_state_estimator = context.perform_substitution(LaunchConfiguration('use_state_estimator'))
    
    # 2. Locate URDF in g1_description package
    g1_description_dir = get_package_share_directory('g1_description')
    urdf_path = os.path.join(g1_description_dir, 'urdf', f'g1_{model}.urdf')
    
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")
        
    print(f"[g1_bringup] Loading G1 URDF file from: {urdf_path}")
    
    # 3. Read URDF file contents
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()

    # 4. Define Robot State Publisher node
    # It parses URDF and publishes fixed transforms, and computes dynamic joint transforms using /joint_states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output={'both': 'log'},
        parameters=[{'robot_description': robot_description_content}]
    )

    # 5. Define our LowState to JointState Bridge node
    # It reads motor positions from /lowstate and maps them dynamically to /joint_states using URDF active joints
    bridge_node = Node(
        package='g1_bringup',
        executable='lowstate_jointstate_bridge',
        name='lowstate_jointstate_bridge',
        output={'stderr': 'log', 'stdout': 'screen'},
        parameters=[{
            'robot_description': robot_description_content,
            'lowstate_topic': lowstate_topic,
            'joint_states_topic': 'joint_states'
        }]
    )

    # Helper function to create Static TF Publisher node from config
    def create_static_tf_node(config_name, node_name):
        tf_cfg = tfs.get(config_name, {})
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=node_name,
            arguments=[
                str(tf_cfg.get('x', 0.0)),
                str(tf_cfg.get('y', 0.0)),
                str(tf_cfg.get('z', 0.0)),
                str(tf_cfg.get('yaw', 0.0)),
                str(tf_cfg.get('pitch', 0.0)),
                str(tf_cfg.get('roll', 0.0)),
                tf_cfg.get('parent_frame', ''),
                tf_cfg.get('child_frame', '')
            ],
            output={'both': 'log'}
        )

    # 6. Define Static TF Publisher for base_link -> pelvis
    # Since URDF's root link is 'pelvis', this bridges our standard ROS 'base_link' to the 'pelvis' frame.
    static_tf_node = create_static_tf_node('base_link_to_pelvis', 'base_link_to_pelvis_publisher')

    # 7. Define Static TF Publisher for mid360_link -> livox_frame
    # This bridges the physical sensor frame in URDF to the frame published by the livox driver.
    # Since the physical LiDAR is mounted upside-down, we rotate it 180 degrees (3.14159265 rad) around the Roll axis.
    lidar_static_tf_node = create_static_tf_node('mid360_to_livox', 'mid360_to_livox_publisher')

    # 11. Define Static TF Publisher for pelvis -> dog_imu_link
    pelvis_to_imu_tf_node = create_static_tf_node('pelvis_to_dog_imu', 'pelvis_to_dog_imu_publisher')

    nodes_to_start = [
        static_tf_node,
        lidar_static_tf_node,
        pelvis_to_imu_tf_node
    ]

    if use_state_estimator.lower() == 'true':
        # EKF State Estimator mode: base_link -> body, camera_init -> odom
        base_link_to_body_tf_node = create_static_tf_node('base_link_to_body', 'base_link_to_body_publisher')
        odom_to_camera_init_tf_node = create_static_tf_node('odom_to_camera_init', 'odom_to_camera_init_publisher')
        nodes_to_start.extend([base_link_to_body_tf_node, odom_to_camera_init_tf_node])
    else:
        # LIO-only mode: odom -> odom_livox -> camera_init -> body -> base_link
        body_to_base_tf_node = create_static_tf_node('body_to_base', 'body_to_base_publisher')
        odom_to_odom_livox_tf_node = create_static_tf_node('odom_to_odom_livox', 'odom_to_odom_livox_publisher')
        odom_livox_to_camera_tf_node = create_static_tf_node('odom_livox_to_camera_init', 'odom_livox_to_camera_init_publisher')
        nodes_to_start.extend([body_to_base_tf_node, odom_to_odom_livox_tf_node, odom_livox_to_camera_tf_node])

    if developer_mode.lower() == 'true':
        nodes_to_start.append(robot_state_publisher_node)
        nodes_to_start.append(bridge_node)

    if sync_lidar_time.lower() == 'true':
        corrector_node = Node(
            package='g1_state_estimator',
            executable='timestamp_corrector_node',
            name='timestamp_corrector',
            output={'stderr': 'log', 'stdout': 'screen'}
        )
        nodes_to_start.append(corrector_node)

    return nodes_to_start

def generate_launch_description():
    bringup_dir = get_package_share_directory('g1_bringup')
    config_path = os.path.join(bringup_dir, 'config', 'g1_bringup.yaml')
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
        
    return LaunchDescription([
        DeclareLaunchArgument(
            'developer_mode',
            default_value=str(config.get('developer_mode', 'true')).lower(),
            choices=['true', 'false'],
            description='Set to true for low-level control (publishes joint states & robot description), false for regular high-level control'
        ),
        DeclareLaunchArgument(
            'model',
            default_value=str(config.get('model', '29dof')),
            choices=['23dof', '29dof'],
            description='G1 model variant: 23dof or 29dof'
        ),
        DeclareLaunchArgument(
            'lowstate_topic',
            default_value=str(config.get('lowstate_topic', 'lowstate')),
            description='Topic to subscribe to for LowState messages (e.g. lowstate or lf/lowstate)'
        ),
        DeclareLaunchArgument(
            'sync_lidar_time',
            default_value=str(config.get('sync_lidar_time', 'true')).lower(),
            choices=['true', 'false'],
            description='Whether to sync LiDAR pointcloud timestamps with host PC time'
        ),
        DeclareLaunchArgument(
            'use_state_estimator',
            default_value=str(config.get('use_state_estimator', 'true')).lower(),
            choices=['true', 'false'],
            description='Set to true to configure TFs for State Estimator (EKF), false for LIO-only mode'
        ),
        OpaqueFunction(function=launch_setup)
    ])
