import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    runtime_root = os.environ.get("FYP_RUNTIME_ROOT", os.path.expanduser("~/frogbot2026/runtime-data"))
    log_file_path = os.path.join(runtime_root, "logs", "imu_trajectory", "imu_trajectory_log.txt")
    # 定义 IMU 轨迹计算节点
    imu_trajectory_node = Node(
        package='wit_imu_traj',  # 功能包名称
        executable='imu_trajectory_node',  # 可执行文件名
        name='imu_trajectory_node',  # 节点名称
        output='screen',  # 输出到终端
        parameters=[
            {"log_file_path": log_file_path}
        ]
    )

    # 返回 LaunchDescription
    return LaunchDescription([
        imu_trajectory_node
    ])