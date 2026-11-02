import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    SLAM & Glass Detection System Launch.

    Based on system_slam.launch.py, extended with:
    1. RealSense Glass Obstacle Detection node (publishing /camera/glass_obstacles)
    2. Nav2 Costmaps with dedicated config (clearing=false for glass, clearing=true for LiDAR)
    3. Foxglove Bridge for remote WebSocket visualization (port 8765, from system_survey)
    4. Livox MID-360 LiDAR + FAST-LIO2 + PGO + PointCloud-to-LaserScan + SLAM Toolbox
    """

    bringup_share = get_package_share_directory("bringup")
    master_params_file = os.path.join(bringup_share, "config", "master_params.yaml")
    default_nav2_params = os.path.join(bringup_share, "config", "nav2_glass_costmap.yaml")
    slam_params_file = os.path.join(bringup_share, "config", "slam_toolbox_mapping.yaml")

    # Bag Recording setup (inherited from system_slam.launch.py)
    bag_profile = os.environ.get("FYP_SLAM_BAG_PROFILE", "lean").strip().lower()
    bag_topics = [
        "/livox/imu",
        "/fastlio2/lio_odom",
        "/fastlio2/degeneracy",
        "/odom_CBoar",
        "/scan",
        "/map",
        "/tf",
        "/tf_static",
        "/camera/glass_obstacles",
        "/camera/glass_detection/status",
        "/local_costmap/costmap",
        "/global_costmap/costmap",
    ]
    if bag_profile in {"debug", "full", "raw"}:
        bag_topics.extend(
            [
                "/livox/lidar",
                "/fastlio2/body_cloud",
            ]
        )
    bag_session_dir = os.environ.get("FYP_LOG_SESSION_DIR")
    bag_output = (
        os.path.join(bag_session_dir, "slam_glass_bag")
        if bag_session_dir
        else os.path.join("/tmp", f"slam_glass_bag_{os.getpid()}")
    )

    # Launch Configurations
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    use_lidar = LaunchConfiguration("use_lidar")
    use_fastlio = LaunchConfiguration("use_fastlio")
    use_serial = LaunchConfiguration("use_serial")
    use_pgo = LaunchConfiguration("use_pgo")
    use_slam = LaunchConfiguration("use_slam")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_realsense = LaunchConfiguration("use_realsense")
    launch_camera_driver = LaunchConfiguration("launch_camera_driver")
    use_foxglove = LaunchConfiguration("use_foxglove")
    foxglove_port = LaunchConfiguration("foxglove_port")
    use_rviz = LaunchConfiguration("use_rviz")
    record_bag = LaunchConfiguration("record_bag")
    camera_height = LaunchConfiguration("camera_height")
    camera_pitch_deg = LaunchConfiguration("camera_pitch_deg")
    depth_method = LaunchConfiguration("depth_method")

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("livox_ros_driver2"),
                        "launch_ROS2",
                        "msg_MID360_launch.py",
                    ]
                )
            ]
        ),
        # condition=IfCondition(use_lidar),
    )

    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("fastlio2"), "launch", "lio_no_rviz.py"]
                )
            ]
        ),
        launch_arguments={"params_file": master_params_file}.items(),
        # condition=IfCondition(use_fastlio),
    )

    serial_node = launch_ros.actions.Node(
        package="serial_twistctl",
        executable="serial_twistctl_node",
        name="serial_twistctl_node",
        output="screen",
        parameters=[master_params_file],
        # condition=IfCondition(use_serial),
    )

    serial_reader_node = launch_ros.actions.Node(
        package="serial_reader",
        executable="serial_reader_node",
        name="serial_reader_node",
        output="screen",
        parameters=[master_params_file],
        # condition=IfCondition(use_serial),
    )

    pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            master_params_file,
            os.path.join(
                bringup_share, "config", "pointcloud_to_laserscan_mapping.yaml"
            ),
        ],
        remappings=[
            ("cloud_in", "/fastlio2/body_cloud"),
            ("scan", "/scan"),
        ],
        # condition=IfCondition(use_fastlio),
    )

    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("pgo"), "config", "pgo_slam.yaml"]
    )
    pgo_node = launch_ros.actions.Node(
        package="pgo",
        executable="pgo_node",
        name="pgo_node",
        output="screen",
        parameters=[
            {"config_path": pgo_config_path.perform(launch.LaunchContext())}
        ],
        # condition=IfCondition(use_pgo),
    )

    slam_toolbox_node = launch_ros.actions.Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file],
        # condition=IfCondition(use_slam),
    )

    map_saver_server = launch_ros.actions.Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[{"save_map_timeout": 5000.0}],
        # condition=IfCondition(use_slam),
    )

    lifecycle_manager_mapping = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_mapping",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["map_saver_server"]}],
        # condition=IfCondition(use_slam),
    )

    delayed_slam = TimerAction(
        period=5.0,
        actions=[slam_toolbox_node, map_saver_server, lifecycle_manager_mapping],
    )

    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "robot_description.launch.py")
        )
    )

    static_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_to_base_tf",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
        condition=UnlessCondition(use_fastlio),
    )

    static_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        condition=UnlessCondition(use_slam),
    )

    # RealSense Glass Obstacle Detection Node
    glass_detection_node = Node(
        package="realsense_camera",
        executable="glass_detection_node",
        name="glass_detection_node",
        output="screen",
        parameters=[
            {
                "camera_height": camera_height,
                "camera_pitch_deg": camera_pitch_deg,
                "depth_method": depth_method,
                "wall_height": 1.5,
                "point_spacing": 0.05,
                "vertical_spacing": 0.05,
                "conf_threshold": 0.25,
                "color_topic": "/camera/camera/color/image_raw",
                "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
                "camera_info_topic": "/camera/camera/color/camera_info",
                "obstacle_cloud_topic": "/camera/glass_obstacles",
                "target_frame": "base_link",
            }
        ],
        # condition=IfCondition(use_realsense),
    )

    # Optional Native RealSense Driver (if official realsense2_camera wrapper not launched externally)
    realsense_driver_node = Node(
        package="realsense_camera",
        executable="realsense_driver_node",
        name="realsense_driver_node",
        output="screen",
        # condition=IfCondition(launch_camera_driver),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": nav2_params_file,
            "autostart": "true",
        }.items(),
        # condition=IfCondition(use_nav2),
    )

    delayed_nav2 = TimerAction(period=6.0, actions=[nav2_launch])

    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{"port": foxglove_port}],
        # condition=IfCondition(use_foxglove),
    )

    bag_recorder = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", bag_output, *bag_topics],
        output="screen",
        # condition=IfCondition(record_bag),
    )

    slam_rviz_config = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "config", "slam_toolbox_default.rviz"]
    )
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="slam_rviz",
        arguments=["-d", slam_rviz_config],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_nav2_params,
                description="Nav2 parameter file with glass obstacle layer configuration",
            ),
            DeclareLaunchArgument(
                "use_lidar",
                default_value="true",
                description="Start Livox MID-360 LiDAR driver",
            ),
            DeclareLaunchArgument(
                "use_fastlio",
                default_value="true",
                description="Start FAST-LIO2 front-end odometry",
            ),
            DeclareLaunchArgument(
                "use_serial",
                default_value="true",
                description="Start serial_twistctl and serial_reader nodes for STM32",
            ),
            DeclareLaunchArgument(
                "use_pgo",
                default_value="true",
                description="Start PGO loop closure node",
            ),
            DeclareLaunchArgument(
                "use_slam",
                default_value="true",
                description="Start SLAM Toolbox mapping and map_saver",
            ),
            DeclareLaunchArgument(
                "use_nav2",
                default_value="true",
                description="Start Nav2 costmaps and navigation stack",
            ),
            DeclareLaunchArgument(
                "use_realsense",
                default_value="true",
                description="Start RealSense glass detection node",
            ),
            DeclareLaunchArgument(
                "launch_camera_driver",
                default_value="false",
                description="Start built-in pyrealsense2 driver (set true if realsense2_camera wrapper is not running)",
            ),
            DeclareLaunchArgument(
                "use_foxglove",
                default_value="true",
                description="Start Foxglove WebSocket bridge for remote visualization",
            ),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value="8765",
                description="Foxglove WebSocket port",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Start RViz2 locally",
            ),
            DeclareLaunchArgument(
                "record_bag",
                default_value="false",
                description="Record ROS 2 bag evidence for this session",
            ),
            DeclareLaunchArgument(
                "camera_height",
                default_value="0.30",
                description="Camera optical center height above ground in meters",
            ),
            DeclareLaunchArgument(
                "camera_pitch_deg",
                default_value="0.0",
                description="Camera pitch down angle in degrees (0 = horizontal)",
            ),
            DeclareLaunchArgument(
                "depth_method",
                default_value="hybrid",
                description="Depth estimation method: hybrid | ground_plane | frame_depth",
            ),

            # Node executions
            livox_launch,
            fastlio_launch,
            serial_node,
            serial_reader_node,
            pointcloud_to_laserscan_node,
            pgo_node,
            urdf_launch,
            static_odom_tf,
            static_map_tf,
            glass_detection_node,
            realsense_driver_node,
            delayed_slam,
            delayed_nav2,
            foxglove_bridge_node,
            bag_recorder,
            rviz_node,
        ]
    )

