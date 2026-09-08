#!/usr/bin/env python3
# =============================================================================
#  t3_landing_flight.py — 完整降落飛行（T3）
#
#  用法：
#      python3 src/drone_apriltag_landing/test/t3_landing_flight.py
#      python3 .../t3_landing_flight.py --gui        # 開視窗看飛
#      python3 .../t3_landing_flight.py --offset 2.0 1.5   # 改初始偏移
#
#  回傳值：降落成功且落點在容許範圍內 0，否則 1。
#
#  ⚠️ 這支會真的讓飛機飛起來。跑之前確認沒有別的 SITL 在跑。
#
#  為什麼要有這層：
#      T1 驗參數、T2 驗座標轉換，兩者都是「靜態」的 —— 它們證明不了
#      控制迴路會收斂、狀態機在真實時序下不會卡住。這些只有真的飛才知道。
#
#  怎麼判斷降得準不準（這支最關鍵的設計）：
#      把飛機 spawn 在 tag 的正上方 (27, 16)。PX4 的 local NED 原點就是
#      EKF 初始化的位置，也就是 tag 的位置 —— 於是「最後的 NED 座標」
#      直接就是「離 tag 多遠」，不需要另外量真值、不需要問 Gazebo。
#
#      流程：起飛 -> 故意飛開一段距離 -> 交接給降落節點 -> 看它把飛機
#      帶回原點多準。
#
#  它抓不到什麼（誠實說明）：
#      - 真實光線、動態模糊、相機曝光 —— 模擬影像一律完美
#      - 風擾與地效 —— SITL 的模型有限
#      - Nav2 導航本身 —— 這支刻意跳過導航，直接從 tag 附近起飛
# =============================================================================

import argparse
import math
import os
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

PKG_NAME = "drone_apriltag_landing"
NAMESPACE = "MAV1"
INSTANCE = 0
MODEL = "x500_nav2"
WORLD = "nav2_arena"

# 落地後離 tag 多遠算成功。
# 0.30 m 的依據：交接高度 1.0 m，PX4 自己降那一段沒有視覺修正，
# 在無風的 SITL 裡水平漂移約十幾公分，留一倍餘裕。
LANDING_TOL = 0.30

# 擺渡高度。T1 實測 tag 在 0.5~8 m 都偵測得到，3 m 有充足餘裕。
FERRY_ALT = 3.0


class Report:
    def __init__(self):
        self.failed = 0
        self.warned = 0

    def section(self, title):
        print(f"\n── {title} " + "─" * max(0, 58 - len(title)))

    def ok(self, msg):
        print(f"   ✓ {msg}")

    def warn(self, msg):
        print(f"   ! {msg}")
        self.warned += 1

    def fail(self, msg):
        print(f"   ✗ {msg}")
        self.failed += 1


# =============================================================================
#  子程序管理
# =============================================================================

class Procs:
    """統一管理子程序，確保不管怎麼結束都關得乾淨。

    絕對不要用 pkill -f "gz sim" 這種寬鬆比對 —— 它會連「命令列裡剛好出現
    這幾個字」的無關程序一起殺，包括呼叫這支腳本的 shell 自己。
    這裡一律用 process group，最後再用精準特徵補刀。
    """

    def __init__(self):
        self.items = []

    def start(self, name, cmd, env=None, log=None, cwd=None):
        f = open(log, "wb") if log else subprocess.DEVNULL
        p = subprocess.Popen(cmd, env=env, cwd=cwd, stdout=f,
                             stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        self.items.append((name, p, f))
        return p

    def stop_all(self):
        for _, p, _ in reversed(self.items):
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGTERM)
                except Exception:
                    pass
        deadline = time.time() + 6.0
        while time.time() < deadline and any(p.poll() is None
                                             for _, p, _ in self.items):
            time.sleep(0.2)
        for _, p, f in self.items:
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except Exception:
                    pass
            if hasattr(f, "close"):
                try:
                    f.close()
                except Exception:
                    pass
        # PX4 和 gz 有機會脫離 process group，用精準的執行檔名補刀
        for comm in ("px4", "ruby"):
            subprocess.run(["pkill", "-x", comm], capture_output=True)
        time.sleep(1.0)
        left = []
        for comm in ("px4", "ruby", "apriltag_node", "precision_land_"):
            r = subprocess.run(["pgrep", "-x", comm],
                               capture_output=True, text=True).stdout.split()
            left += [f"{comm}:{p}" for p in r]
        print(f"\n   收工：殘留 = {'有！' + ' '.join(left) if left else '無'}")
        return not left


# =============================================================================
#  環境（照 drone_nav2_apriltag/scripts/start_arena_sitl.sh 的設定）
# =============================================================================

def locate_arena():
    roots = []
    try:
        from ament_index_python.packages import get_package_share_directory
        roots.append(get_package_share_directory("drone_nav2_apriltag"))
    except Exception:
        pass
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        roots.append(os.path.join(d, "src", "drone_nav2_apriltag"))
        d = os.path.dirname(d)
    for r in roots:
        if os.path.isfile(os.path.join(r, "gz", "worlds", f"{WORLD}.sdf")):
            return r
    return None


def tag_position(arena_root):
    """從 world 讀 tag 的 ENU 座標。飛機就 spawn 在這個點正上方。"""
    root = ET.parse(os.path.join(arena_root, "gz", "worlds",
                                 f"{WORLD}.sdf")).getroot()
    for inc in root.iter("include"):
        if "apriltag" in (inc.findtext("uri") or ""):
            v = [float(x) for x in (inc.findtext("pose") or "0 0 0").split()]
            return v[0], v[1]
    return None


def build_env(arena_root, px4_dir, headless):
    env = os.environ.copy()
    # gz-transport 綁回環位址。不設的話 IMU 傳遞會抖動 -> Accel TIMEOUT
    # -> EKF 劣化 -> 起飛後失效保護。（start_arena_sitl.sh:46 的實測結論）
    env["GZ_IP"] = "127.0.0.1"

    # 先 source PX4 的 gz_env.sh 拿到外掛路徑與 server config，再把世界改指到本場地。
    # 順序不能反 —— gz_env.sh 是無條件覆寫 PX4_GZ_WORLDS 的。
    envsh = os.path.join(px4_dir, "build", "px4_sitl_default", "rootfs", "gz_env.sh")
    out = subprocess.run(["bash", "-c", f"source {envsh} && env"],
                         capture_output=True, text=True).stdout
    for line in out.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            if k.startswith(("PX4_GZ_", "GZ_SIM_")):
                env[k] = v

    env["PX4_GZ_WORLDS"] = os.path.join(arena_root, "gz", "worlds")
    env["PX4_GZ_WORLD"] = WORLD
    env["GZ_SIM_RESOURCE_PATH"] = (
        os.path.join(arena_root, "gz", "models") + ":"
        + env.get("GZ_SIM_RESOURCE_PATH", ""))
    # 機體模型走 file://${PX4_GZ_MODELS}/<name>/model.sdf，寫死單一目錄，
    # 不吃 GZ_SIM_RESOURCE_PATH，所以自製機體必須把這個變數整個指過來。
    env["PX4_GZ_MODELS"] = os.path.join(arena_root, "gz", "models")
    if headless:
        env["HEADLESS"] = "1"
    return env


# =============================================================================
#  擺渡飛行：扮演「上層呼叫方」（Nav2 / 編隊節點之後會做的事）
# =============================================================================

class Ferry:
    """把飛機解鎖、起飛、飛到一個刻意偏開 tag 的位置，然後交出控制權。

    這一段刻意寫成「另一個會發 setpoint 的節點」，因為它要順便驗交接協議：
    降落節點在 trajectory_setpoint 上還有別人發布時必須拒絕接手，
    而呼叫方停止發布之後 PX4 會掉出 offboard —— 降落節點要能自己切回來。
    這個空窗期是真實系統一定會遇到的，不模擬就等於沒測。
    """

    def __init__(self, node, ns, target_system):
        from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                                  VehicleCommand, VehicleLocalPosition,
                                  VehicleStatus)
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

        self.node = node
        self.ts = target_system
        self.VehicleCommand = VehicleCommand
        self.VehicleStatus = VehicleStatus

        sub_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.VOLATILE)
        self.pos = None
        self.status = None
        node.create_subscription(
            VehicleLocalPosition, f"/{ns}/fmu/out/vehicle_local_position_v1",
            self._on_pos, sub_qos)
        node.create_subscription(
            VehicleStatus, f"/{ns}/fmu/out/vehicle_status_v1",
            self._on_status, sub_qos)

        self.mode_pub = node.create_publisher(
            OffboardControlMode, f"/{ns}/fmu/in/offboard_control_mode", 10)
        self.sp_pub = node.create_publisher(
            TrajectorySetpoint, f"/{ns}/fmu/in/trajectory_setpoint", 10)
        self.cmd_pub = node.create_publisher(
            VehicleCommand, f"/{ns}/fmu/in/vehicle_command", 10)
        self.OffboardControlMode = OffboardControlMode
        self.TrajectorySetpoint = TrajectorySetpoint

    def _on_pos(self, m):
        self.pos = m

    def _on_status(self, m):
        self.status = m

    def _now_us(self):
        return int(self.node.get_clock().now().nanoseconds / 1000)

    def _cmd(self, command, p1=0.0, p2=0.0):
        m = self.VehicleCommand()
        m.command = command
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = self.ts
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True   # 少了這個 PX4 會當成內部指令直接拒絕
        m.timestamp = self._now_us()
        self.cmd_pub.publish(m)

    def _send_setpoint(self, n, e, d):
        m = self.OffboardControlMode()
        m.position = True        # 六個控制層級互斥，擺渡用位置控制
        m.timestamp = self._now_us()
        self.mode_pub.publish(m)

        sp = self.TrajectorySetpoint()
        sp.position = [float(n), float(e), float(d)]
        sp.yaw = 0.0             # 機頭朝北，讓航向修正有東西可修
        sp.timestamp = self._now_us()
        self.sp_pub.publish(sp)

    def spin(self, seconds, target=None, rate_hz=20.0):
        """跑 ROS 迴圈並以固定頻率發 setpoint。

        ⚠️ 一定要限速。spin_once 在有訊息可處理時幾乎立刻返回，
        位置與狀態是 30+ Hz 進來的，所以「迴圈裡直接發」等於用數千 Hz 灌，
        PX4 端的佇列會塞爆。PX4 只要求 > 2 Hz，20 Hz 已經十倍餘裕。
        """
        import rclpy
        end = time.time() + seconds
        period = 1.0 / rate_hz
        next_tx = 0.0
        while time.time() < end:
            now = time.time()
            if target is not None and now >= next_tx:
                self._send_setpoint(*target)
                next_tx = now + period
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_position(self, timeout=90.0):
        import rclpy
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.pos is not None and self.pos.xy_valid and self.pos.z_valid:
                return True
        return False

    def takeoff_and_move(self, north, east, alt, log):
        """解鎖 -> 切 offboard -> 起飛 -> 飛到指定偏移。回傳 (成功, 說明)。"""
        import rclpy
        target = (0.0, 0.0, -alt)

        # PX4 要求「先有 setpoint 串流，才准切 offboard」。
        # 順序反過來的話切模式指令會被拒絕，而且訊息很不明顯。
        self.spin(1.5, target)

        self._cmd(self.VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.spin(0.5, target)
        self._cmd(self.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.spin(1.0, target)

        # 沒切成或沒解鎖就重試幾次，SITL 剛起來時第一次常常吃不到
        for _ in range(10):
            armed = (self.status is not None and self.status.arming_state ==
                     self.VehicleStatus.ARMING_STATE_ARMED)
            offb = (self.status is not None and self.status.nav_state ==
                    self.VehicleStatus.NAVIGATION_STATE_OFFBOARD)
            if armed and offb:
                break
            if not offb:
                self._cmd(self.VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if not armed:
                self._cmd(self.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.spin(1.0, target)

        if self.status is None:
            return False, "收不到 vehicle_status"
        if self.status.arming_state != self.VehicleStatus.ARMING_STATE_ARMED:
            return False, (f"解鎖失敗（arming_state={self.status.arming_state}）。"
                           "預檢沒過的話看 PX4 的 out.log")
        if self.status.nav_state != self.VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return False, f"切 offboard 失敗（nav_state={self.status.nav_state}）"
        log("已解鎖並進入 offboard，開始爬升")

        # 爬到高度
        end = time.time() + 40.0
        while time.time() < end:
            self.spin(0.2, target)
            if self.pos and abs(-self.pos.z - alt) < 0.3:
                break
        else:
            return False, f"爬升逾時（目前高度 {-self.pos.z:.2f} m）"
        log(f"到達 {alt:.1f} m，飛往偏移點 北{north:+.1f} 東{east:+.1f}")

        # 飛到刻意偏開的位置
        target = (north, east, -alt)
        end = time.time() + 40.0
        while time.time() < end:
            self.spin(0.2, target)
            if self.pos and math.hypot(self.pos.x - north,
                                       self.pos.y - east) < 0.25:
                break
        else:
            return False, "飛往偏移點逾時"
        # 停穩，避免帶著速度交接
        self.spin(3.0, target)
        return True, (f"就位：北 {self.pos.x:+.2f} 東 {self.pos.y:+.2f} "
                      f"高 {-self.pos.z:.2f} m")

    def release(self):
        """停止發布 setpoint 並銷毀 publisher，把控制權交出去。

        一定要真的 destroy_publisher —— 降落節點會用 count_publishers 檢查
        還有沒有別人在發，只是「不再發訊息」的話那個檢查仍然會看到我們。
        """
        self.node.destroy_publisher(self.sp_pub)
        self.node.destroy_publisher(self.mode_pub)


# =============================================================================
#  主流程
# =============================================================================

def main():
    ap = argparse.ArgumentParser(
        description="完整降落飛行驗證（會真的讓飛機飛起來）")
    ap.add_argument("--gui", action="store_true", help="開 Gazebo 視窗")
    ap.add_argument("--offset", nargs=2, type=float, default=[1.0, 1.5],
                    metavar=("北", "東"),
                    help="交接前刻意偏開 tag 多少（公尺），預設 1.0 1.5")
    ap.add_argument("--px4-dir", default=os.path.expanduser("~/PX4-Autopilot"))
    ap.add_argument("--keep", action="store_true",
                    help="跑完不關閉（除錯用）")
    args = ap.parse_args()

    rep = Report()
    procs = Procs()
    scratch = f"/tmp/t3_{os.getpid()}"
    os.makedirs(scratch, exist_ok=True)
    node = None

    try:
        rep.section("C0 環境")
        arena = locate_arena()
        if not arena:
            rep.fail("找不到 drone_nav2_apriltag 的場地")
            return 1
        rep.ok(f"場地：{arena}")

        tag = tag_position(arena)
        if not tag:
            rep.fail("world 裡找不到 AprilTag")
            return 1
        rep.ok(f"tag 位置（ENU）：({tag[0]}, {tag[1]})")

        build_dir = os.path.join(args.px4_dir, "build", "px4_sitl_default")
        px4_bin = os.path.join(build_dir, "bin", "px4")
        if not os.path.isfile(px4_bin):
            rep.fail(f"找不到 {px4_bin}，先 make px4_sitl_default")
            return 1
        rep.ok(f"PX4：{px4_bin}")

        # ---- 清掉舊程序 ----
        for comm in ("px4", "ruby", "apriltag_node", "precision_land_"):
            subprocess.run(["pkill", "-x", comm], capture_output=True)
        time.sleep(2)

        env = build_env(arena, args.px4_dir, headless=not args.gui)

        rep.section("C1 啟動模擬")
        # XRCE Agent。沒有它 PX4 跑得好好的，但 ROS 2 一個 topic 都收不到。
        if subprocess.run(["pgrep", "-x", "MicroXRCEAgent"],
                          capture_output=True).returncode != 0:
            procs.start("agent", ["MicroXRCEAgent", "udp4", "-p", "8888"],
                        env=env, log=f"{scratch}/agent.log")
            time.sleep(2)
            rep.ok("已啟動 MicroXRCEAgent")
        else:
            rep.ok("MicroXRCEAgent 已在執行（沿用）")

        # ⚠️ 關鍵：spawn 在 tag 正上方。
        # PX4 的 local NED 原點 = EKF 初始化的位置 = 這裡，
        # 所以之後「最後的 NED 座標」直接就是「離 tag 多遠」。
        work = os.path.join(build_dir, f"instance_{INSTANCE}")
        os.makedirs(work, exist_ok=True)
        px4_env = dict(env)
        px4_env.update({
            "PX4_UXRCE_DDS_NS": NAMESPACE,
            "PX4_SYS_AUTOSTART": "4001",
            "PX4_SIM_MODEL": MODEL,
            "PX4_GZ_MODEL_POSE": f"{tag[0]},{tag[1]},0,0,0,0",
        })
        procs.start("px4", [px4_bin, "-i", str(INSTANCE), "-d",
                            os.path.join(build_dir, "etc")],
                    env=px4_env, log=f"{scratch}/px4.log", cwd=work)

        ok = False
        for _ in range(90):
            time.sleep(1)
            out = subprocess.run(["gz", "topic", "-l"], capture_output=True,
                                 text=True, env=env).stdout
            if f"/world/{WORLD}/clock" in out:
                ok = True
                break
        if not ok:
            rep.fail("Gazebo 世界 90 秒內沒建立起來")
            return 1
        rep.ok(f"Gazebo 世界 {WORLD} 已建立，飛機 spawn 在 tag 上方")

        # 等 PX4 把 topic 註冊出去
        ok = False
        for _ in range(60):
            time.sleep(1)
            try:
                with open(f"{scratch}/px4.log", "r", errors="ignore") as f:
                    if "vehicle_local_position" in f.read():
                        ok = True
                        break
            except OSError:
                pass
        if not ok:
            rep.warn("PX4 log 裡沒看到 vehicle_local_position 註冊，繼續試")
        else:
            rep.ok("PX4 已連上 XRCE Agent")

        # 機型檔 4001 會 set-default NAV_DLL_ACT 2，沒開 QGC 就 ARM 不起來
        param = os.path.join(build_dir, "bin", "px4-param")
        for k, v in (("NAV_DLL_ACT", "0"), ("CBRK_SUPPLY_CHK", "894281")):
            subprocess.run([param, "--instance", str(INSTANCE), "set", k, v],
                           capture_output=True, env=env)
        subprocess.run([param, "--instance", str(INSTANCE), "save"],
                       capture_output=True, env=env)
        rep.ok("預檢參數已設定（NAV_DLL_ACT=0, CBRK_SUPPLY_CHK）")

        # ---- 影像橋接 + 偵測器 + 降落節點 ----
        model_inst = f"{MODEL}_{INSTANCE}"
        gz_img = (f"/world/{WORLD}/model/{model_inst}"
                  f"/link/camera_down_link/sensor/imager_down/image")
        gz_info = gz_img.rsplit("/", 1)[0] + "/camera_info"

        here = os.path.dirname(os.path.abspath(__file__))
        cfg_dir = os.path.join(os.path.dirname(here), "config")
        if not os.path.isdir(cfg_dir):
            from ament_index_python.packages import get_package_share_directory
            cfg_dir = os.path.join(get_package_share_directory(PKG_NAME), "config")

        procs.start("img", ["ros2", "run", "ros_gz_image", "image_bridge", gz_img],
                    env=env, log=f"{scratch}/img.log")
        procs.start("info", ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                             f"{gz_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
                    env=env, log=f"{scratch}/info.log")
        procs.start("apriltag", ["ros2", "run", "apriltag_ros", "apriltag_node",
                                 "--ros-args", "--params-file",
                                 os.path.join(cfg_dir, "tag_36h11.yaml"),
                                 "-r", f"image_rect:={gz_img}",
                                 "-r", f"camera_info:={gz_info}",
                                 "-r", f"__ns:=/{NAMESPACE}"],
                    env=env, log=f"{scratch}/apriltag.log")
        procs.start("land", ["ros2", "run", PKG_NAME, "precision_land_node",
                             "--ros-args", "--params-file",
                             os.path.join(cfg_dir, "landing.yaml"),
                             "-r", f"__ns:=/{NAMESPACE}"],
                    env=env, log=f"{scratch}/land.log")
        rep.ok("影像橋接、apriltag_node、precision_land_node 都已啟動")
        time.sleep(3)

        # ---- ROS 節點 ----
        import rclpy
        from rclpy.action import ActionClient
        from drone_apriltag_landing.action import PrecisionLand

        rclpy.init()
        node = rclpy.create_node("t3_landing_flight")
        ferry = Ferry(node, NAMESPACE, INSTANCE + 1)

        rep.section("C2 擺渡（扮演上層呼叫方）")
        if not ferry.wait_position():
            rep.fail("收不到 vehicle_local_position。"
                     "確認 MicroXRCEAgent 在跑、namespace 是 " + NAMESPACE)
            return 1
        rep.ok("已收到位置估計")

        north, east = args.offset
        good, msg = ferry.takeoff_and_move(north, east, FERRY_ALT, rep.ok)
        if not good:
            rep.fail(f"擺渡失敗：{msg}")
            return 1
        rep.ok(msg)
        start_err = math.hypot(ferry.pos.x, ferry.pos.y)
        rep.ok(f"交接前離 tag {start_err:.2f} m —— 這就是降落節點要修掉的量")

        # ---- 交接 ----
        rep.section("C3 交接與降落")
        ferry.release()
        # 等 DDS 的 discovery 傳播，降落節點才看得到「沒有別人在發 setpoint 了」。
        # 這段空窗期 PX4 會掉出 offboard 進 HOLD，是真實系統一定會有的情況，
        # 降落節點必須自己把模式切回來。
        ferry.spin(2.0)
        rep.ok("呼叫方已停止發布 setpoint，控制權釋出")

        client = ActionClient(node, PrecisionLand, f"/{NAMESPACE}/precision_land")
        if not client.wait_for_server(timeout_sec=10.0):
            rep.fail("找不到 precision_land action server")
            return 1

        feedback_log = []

        def on_fb(fb):
            f = fb.feedback
            names = {0: "SEARCHING", 1: "ALIGNING", 2: "DESCENDING", 3: "HANDOFF"}
            row = (names.get(f.state, "?"), f.altitude, f.xy_error,
                   math.degrees(f.yaw_error), f.tag_visible)
            if not feedback_log or feedback_log[-1][0] != row[0]:
                print(f"     [{row[0]:<10}] 高度 {row[1]:5.2f} m  "
                      f"水平誤差 {row[2]:5.2f} m  yaw {row[3]:+6.1f}°  "
                      f"看得到 tag={row[4]}")
            feedback_log.append(row)

        goal = PrecisionLand.Goal()
        goal.tag_id = -1
        goal.approach_altitude = 0.0
        goal.align_yaw = True
        send = client.send_goal_async(goal, feedback_callback=on_fb)
        rclpy.spin_until_future_complete(node, send, timeout_sec=15.0)
        gh = send.result()
        if gh is None or not gh.accepted:
            rep.fail("goal 被拒絕。看 land.log 的 ERROR 行找原因")
            return 1
        rep.ok("goal 已被接受，降落節點接管")

        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(node, res_fut, timeout_sec=180.0)
        if not res_fut.done():
            rep.fail("等結果逾時（180 秒）")
            return 1
        result = res_fut.result().result

        rep.section("C4 驗收")
        codes = {0: "成功", 1: "逾時", 2: "被取消", 3: "沒看到 tag",
                 4: "位置估計失效", 5: "被拒絕"}
        print(f"     result_code : {result.result_code}"
              f"（{codes.get(result.result_code, '?')}）")
        print(f"     交接時殘餘   : 北 {result.final_x_error:+.3f}  "
              f"東 {result.final_y_error:+.3f}  "
              f"yaw {math.degrees(result.final_yaw_error):+.1f}°")

        states = []
        for r in feedback_log:
            if not states or states[-1] != r[0]:
                states.append(r[0])
        print(f"     走過的狀態   : {' -> '.join(states)}")

        if not result.success:
            rep.fail(f"降落回報失敗：{codes.get(result.result_code, '?')}")
        else:
            rep.ok("降落節點回報成功")

        # 最終落點。spawn 在 tag 上方，所以 NED 座標就是離 tag 的距離。
        ferry.spin(1.0)
        final = math.hypot(ferry.pos.x, ferry.pos.y)
        print(f"     最終位置     : 北 {ferry.pos.x:+.3f}  東 {ferry.pos.y:+.3f}  "
              f"高 {-ferry.pos.z:.3f} m")
        print(f"     離 tag       : {final:.3f} m（起始 {start_err:.2f} m）")
        if final > LANDING_TOL:
            rep.fail(f"落點離 tag {final:.3f} m，超過容許值 {LANDING_TOL} m")
        else:
            rep.ok(f"落點離 tag {final:.3f} m ≤ {LANDING_TOL} m")

        if -ferry.pos.z > 0.5:
            rep.fail(f"最終高度 {-ferry.pos.z:.2f} m，看起來沒有真的落地")
        else:
            rep.ok(f"最終高度 {-ferry.pos.z:.2f} m，已落地")

    except KeyboardInterrupt:
        print("\n(中斷)")
        return 130
    finally:
        if node is not None:
            try:
                node.destroy_node()
                import rclpy
                rclpy.shutdown()
            except Exception:
                pass
        if args.keep:
            print(f"\n   --keep：程序保留中，log 在 {scratch}")
        else:
            clean = procs.stop_all()
            if not clean:
                print("   ⚠️ 有殘留，請自己確認後關閉")
        print(f"   log：{scratch}")

    print()
    if rep.failed:
        print(f"結果：失敗 —— {rep.failed} 項不通過"
              + (f"，{rep.warned} 項警告" if rep.warned else ""))
        return 1
    print("結果：全部通過" + (f"（{rep.warned} 項警告）" if rep.warned else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
