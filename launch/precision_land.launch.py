"""
一鍵起「AprilTag 偵測 + 精準降落」。

用法:
    ros2 launch drone_apriltag_landing precision_land.launch.py
    ros2 launch drone_apriltag_landing precision_land.launch.py namespace:=MAV2
    ros2 launch drone_apriltag_landing precision_land.launch.py control_mode:=advisor

前提: 相機影像和 camera_info 已經有人發布了。模擬時是
      drone_nav2_apriltag 的 cameras.launch.py，實機時是 camera_ros。
      本套件不負責開相機 —— 那樣就會綁死在特定機體上。

觸發降落（本 launch 只是把節點準備好，不會自己開始降）:
    ros2 action send_goal /MAV1/precision_land \
        drone_apriltag_landing/action/PrecisionLand "{}"
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "drone_apriltag_landing"


def _setup(context, *args, **kwargs):
    ns = LaunchConfiguration("namespace").perform(context)
    cam = LaunchConfiguration("camera").perform(context)
    mode = LaunchConfiguration("control_mode").perform(context)

    share = get_package_share_directory(PKG)
    tag_cfg = os.path.join(share, "config", "tag_36h11.yaml")
    land_cfg = os.path.join(share, "config", "landing.yaml")

    # 相機發出來的兩個 topic。cameras.launch.py 的命名規則就是這樣。
    image_topic = f"/{ns}/{cam}/image_raw"
    info_topic = f"/{ns}/{cam}/camera_info"

    return [
        # ---- 現成的偵測器，我們一行都沒寫 ----
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag_node",
            namespace=ns,
            parameters=[tag_cfg],
            # apriltag_node 訂的是 image_rect（去過畸變的影像）。
            # Gazebo 的相機沒有鏡頭畸變，raw 直接當 rect 用沒問題；
            # 但實機接真相機時中間必須插一個 image_proc 的 rectify，
            # 否則邊緣的位姿會有系統性偏差。
            remappings=[
                ("image_rect", image_topic),
                ("camera_info", info_topic),
            ],
            output="screen",
        ),

        # ---- 我們的節點 ----
        Node(
            package=PKG,
            executable="precision_land_node",
            name="precision_land_node",
            namespace=ns,
            parameters=[
                land_cfg,
                # 命令列給的值要蓋過 yaml，所以放在後面
                {"namespace": ns, "control_mode": mode},
            ],
            output="screen",
            emulate_tty=True,   # 讓 RCLCPP_INFO 的顏色和即時輸出正常
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="MAV1",
                              description="PX4 namespace，多機時改這個"),
        DeclareLaunchArgument("camera", default_value="camera_down",
                              description="要用哪顆相機（對應 topic 前綴）"),
        DeclareLaunchArgument("control_mode", default_value="controller",
                              description="controller=接管飛機 / advisor=只回報誤差"),
        OpaqueFunction(function=_setup),
    ])
