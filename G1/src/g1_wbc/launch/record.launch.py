"""Ghi rt/lowstate tho. CHI DOC - khong gui lenh nao den robot.

    ros2 launch g1_wbc record.launch.py label:="day vai + di vai buoc" secs:=40
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg = lambda n, d: DeclareLaunchArgument(n, default_value=d)
    cfg = LaunchConfiguration
    return LaunchDescription([
        arg("iface", "wlp0s20f3"),
        arg("label", ""),
        arg("name", ""),
        arg("secs", "0.0"),        # 0 = ghi den khi Ctrl-C
        arg("max_secs", "120.0"),
        Node(package="g1_wbc", executable="lowstate_recorder_node",
             name="lowstate_recorder_node", output="screen",
             emulate_tty=True,
             parameters=[{"network_interface": cfg("iface"),
                          "label": cfg("label"),
                          "name": cfg("name"),
                          "stop_after_secs": cfg("secs"),
                          "max_secs": cfg("max_secs")}]),
    ])
