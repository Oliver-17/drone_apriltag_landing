"""
沿拓樸圖飛到降落點，然後交給 AprilTag 精準降落。

    路線規劃 + 飛行  ← drone_nav2_apriltag（一行都沒改，只多帶一個參數）
    精準降落        ← 本套件

用法:
    # 前提：SITL 已經在跑（./start_arena_sitl.sh，且 MicroXRCEAgent 已開）
    ros2 launch drone_apriltag_landing nav2_then_land.launch.py

    ros2 launch drone_apriltag_landing nav2_then_land.launch.py namespace:=MAV2 drone_id:=1
    ros2 launch drone_apriltag_landing nav2_then_land.launch.py goal_node:=10

時序:
    route_server 規劃 -> 依序飛節點 -> 到終點上空
        │
        │  land_at_goal:=false -> 不降落，fly_nodes 退出
        ▼
    setpoint 停止 -> PX4 掉進 HOLD（懸停）
        │
        │  OnProcessExit 觸發
        ▼
    送 PrecisionLand goal -> SEARCHING -> ALIGNING -> DESCENDING -> HANDOFF -> 落地

為什麼用 ExecuteProcess 跑 `ros2 launch` 而不是 IncludeLaunchDescription:
    drone_nav2_apriltag 的 fly_nodes.launch.py 內部綁了
    OnProcessExit(fly_nodes) -> Shutdown（見該檔第 117 行左右），
    用 Include 的話那個 Shutdown 會把「整個 launch」關掉 ——
    包含本套件的降落節點，而且是在它還來不及動作之前。
    包成子程序之後，那個 Shutdown 只作用在子程序自己身上，
    我們則透過它的結束事件得知「飛到了」。

    代價是子程序的輸出會混在一起（兩邊都 output=screen），
    但 fly_nodes 和降落節點的執行期幾乎不重疊，實際上不太會互相干擾。
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

PKG = "drone_apriltag_landing"
ARENA = "drone_nav2_apriltag"


def _setup(context, *args, **kwargs):
    ns = LaunchConfiguration("namespace").perform(context)
    drone_id = LaunchConfiguration("drone_id").perform(context)
    cam = LaunchConfiguration("camera").perform(context)
    goal_node = LaunchConfiguration("goal_node").perform(context)
    altitude = LaunchConfiguration("flight_altitude").perform(context)
    target_system = LaunchConfiguration("target_system").perform(context)
    view = LaunchConfiguration("view").perform(context)

    share = get_package_share_directory(PKG)
    arena_share = get_package_share_directory(ARENA)
    tag_cfg = os.path.join(share, "config", "tag_36h11.yaml")
    land_cfg = os.path.join(share, "config", "landing.yaml")

    image_topic = f"/{ns}/{cam}/image_raw"
    info_topic = f"/{ns}/{cam}/camera_info"

    # ---- 相機橋接（用地圖包的，不重造）----
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arena_share, "launch", "cameras.launch.py")),
        launch_arguments={
            "namespace": ns,
            "drone_id": drone_id,
            # 預設不開影像視窗：降落靠的是 apriltag 算出來的數字，不是人眼。
            # 開視窗會多吃 GPU，而 GPU 被吃掉會拖慢 Gazebo 的物理步進，
            # lockstep 下 PX4 就收不到 IMU —— 想看畫面時再打開就好。
            "view": view,
            "lidar": "false",
        }.items(),
    )

    # ---- 偵測器（現成的）----
    apriltag = Node(
        package="apriltag_ros", executable="apriltag_node", name="apriltag_node",
        namespace=ns, parameters=[tag_cfg],
        # apriltag_node 訂的是 image_rect。Gazebo 相機沒有鏡頭畸變，
        # raw 直接當 rect 用沒問題；實機接真相機時中間必須插 image_proc 的 rectify。
        remappings=[("image_rect", image_topic), ("camera_info", info_topic)],
        output="screen",
    )

    # ---- 降落節點（待命，等 goal）----
    lander = Node(
        package=PKG, executable="precision_land_node", name="precision_land_node",
        namespace=ns,
        parameters=[land_cfg, {"namespace": ns,
                               "target_system": int(target_system)}],
        output="screen", emulate_tty=True,
    )

    # ---- 導航（子程序，這樣它內部的 Shutdown 不會波及我們）----
    nav = ExecuteProcess(
        cmd=["ros2", "launch", ARENA, "fly_nodes.launch.py",
             # ⚠️ 關鍵：到終點不要自己降落，停在原地把控制權交出來
             "land_at_goal:=false",
             f"px4_namespace:=/{ns}",
             f"target_system:={target_system}",
             f"goal_node:={goal_node}",
             f"flight_altitude:={altitude}"],
        output="screen",
    )

    # ---- 到了之後才送 goal ----
    # 為什麼不用「偵測位置接近終點」來判斷：那樣會在 fly_nodes 還在發 setpoint
    # 的時候就送 goal，而降落節點看到 trajectory_setpoint 上還有別人發布時
    # 會拒絕接手（交接協議）。等它整個程序結束是最明確的信號。
    send_goal = ExecuteProcess(
        cmd=["ros2", "action", "send_goal", "-f",
             f"/{ns}/precision_land",
             "drone_apriltag_landing/action/PrecisionLand",
             "{tag_id: -1, approach_altitude: 0.0, align_yaw: true}"],
        output="screen",
    )

    on_arrived = RegisterEventHandler(OnProcessExit(
        target_action=nav,
        on_exit=[
            LogInfo(msg="=== 路線飛完，交給 AprilTag 精準降落 ==="),
            send_goal,
        ]))

    # 降落結束（不論成敗）就收掉整組，不要讓 launch 一直掛著
    on_landed = RegisterEventHandler(OnProcessExit(
        target_action=send_goal,
        on_exit=[Shutdown(reason="降落流程結束")]))

    return [cameras, apriltag, lander, nav, on_arrived, on_landed]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="MAV1",
                              description="PX4 namespace，多機時改這個"),
        DeclareLaunchArgument("drone_id", default_value="0",
                              description="PX4 instance 編號，決定 Gazebo 模型名字"),
        DeclareLaunchArgument("target_system", default_value="1",
                              description="MAVLink target_system，等於 instance+1"),
        DeclareLaunchArgument("camera", default_value="camera_down"),
        DeclareLaunchArgument("goal_node", default_value="-1",
                              description="終點節點 id，-1 表示用最大的（降落點）"),
        DeclareLaunchArgument("flight_altitude", default_value="3.0"),
        DeclareLaunchArgument("view", default_value="false",
                              description="true 會開 rqt_image_view 顯示兩顆相機的畫面"),
        OpaqueFunction(function=_setup),
    ])
