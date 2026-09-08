#!/usr/bin/env python3
# =============================================================================
#  t2_pose_check.py — 座標系與位姿轉換驗證（T2）
#
#  用法：
#      python3 src/drone_apriltag_landing/test/t2_pose_check.py
#      python3 .../t2_pose_check.py --keep-gz     # 跑完不關 Gazebo（除錯用）
#      python3 .../t2_pose_check.py --gui         # 開視窗看探針擺在哪
#
#  回傳值：全部通過 0，有任何一項失敗 1。
#
#  為什麼需要這支：
#      T1 只比對檔案和參數，它看不見「方向」。而降落節點最可能出錯的地方
#      正是方向 —— tag 的位姿是在「相機光學座標系」(x 右 / y 下 / z 沿光軸)，
#      要變成 PX4 吃的 NED (x 北 / y 東 / z 下) 中間要串三個旋轉，
#      而下視相機本身還有 pitch +1.5707。
#
#      任何一個符號或軸搞錯，飛機都會往反方向飛 —— 而且看起來很像正常在動：
#      它確實在移動、確實在收斂（收斂到錯的地方），從 log 完全看不出來。
#      這種錯只能靠「放在已知位置，檢查算出來的答案對不對」抓。
#
#  怎麼做到「不用飛」：
#      完全不啟動 PX4。只開 Gazebo server，然後用 /world/<w>/create 生一個
#      <static>true</static> 的「相機探針」放在指定位置 —— 靜態模型不會掉下去，
#      位置由我們指定所以真值是已知的，不需要 EKF、不需要起飛、不需要解鎖。
#
#      探針的相機 link 是「執行時從 x500_nav2/model.sdf 抓出來」的，不是複製一份。
#      這樣改了機體的相機安裝角度，這支測試會跟著驗新的角度，不會悄悄過期。
#
#  它抓不到什麼（誠實說明）：
#      - 控制迴路會不會收斂、狀態機對不對 —— 要靠 T3
#      - 真實光線、動態模糊 —— 模擬影像一律完美
#      - PX4 的 heading 對不對 —— 這支不啟動 PX4，機頭朝向是我們自己指定的
# =============================================================================

import argparse
import math
import os
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

PKG_NAME = "drone_apriltag_landing"

# 相機探針在 Gazebo 裡的模型名稱。每個測試位置都用同一個名字：
# 先 remove 再 create，這樣 gz 的感測器 topic 名稱不會變，橋接只要起一次。
PROBE_NAME = "t2_camera_probe"

# 位置誤差容許值（公尺）。合成影像沒有雜訊，正確的實作應該遠小於這個數字；
# 放寬到 5 cm 是為了容忍 tag 貼圖的抗鋸齒與 PnP 的數值誤差。
# 這個門檻抓的是「方向錯」（誤差會是公尺級），不是「精度不足」。
POS_TOL = 0.05
YAW_TOL_DEG = 3.0


# =============================================================================
#  報告（格式和 check_graph.py / t1 一致）
# =============================================================================

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
#  幾何
# =============================================================================

def rot_rpy(roll, pitch, yaw):
    """SDF 的 <pose> 尾三個數字是 roll-pitch-yaw（繞固定軸 X-Y-Z，外旋）。

    回傳的矩陣把「子座標系裡的向量」轉成「父座標系裡的向量」。
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def mat_vec(m, v):
    return [sum(m[i][j] * v[j] for j in range(3)) for i in range(3)]


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


# 相機「光學座標系」→「link 座標系」的固定旋轉。
#
#   光學（ROS 慣例）：x 右、y 下、z 沿光軸向前
#   link（Gazebo 慣例）：x 沿光軸向前、y 左、z 上
#
# 所以 link_x = optical_z、link_y = -optical_x、link_z = -optical_y。
# apriltag_ros 算出來的位姿是光學系的，但 TF 上掛的 frame_id 是 link 的名字
# （camera_info 的 frame_id 來自 gz_frame_id），這個落差是最常見的錯誤來源之一。
R_OPTICAL_TO_LINK = [
    [0.0,  0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
]


def quat_to_yaw(x, y, z, w):
    """只取繞 z 軸的偏航角。平面 tag 的 PnP 解裡，這個分量是最穩的。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# =============================================================================
#  讀場景
# =============================================================================

def find_pkg_share(name):
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(name)
    except Exception:
        return None


def find_config_file(name):
    """找本套件的設定檔，原始碼樹與 install 樹都要找得到。"""
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [
        os.path.join(os.path.dirname(here), "config", name),
        os.path.join(os.path.dirname(os.path.dirname(here)),
                     "share", PKG_NAME, "config", name),
    ]
    share = find_pkg_share(PKG_NAME)
    if share:
        cands.append(os.path.join(share, "config", name))
    for c in cands:
        if os.path.isfile(c):
            return c
    return None


def load_ros_params(path):
    """讀 ROS 2 參數檔。最外層可能是 /** 或節點名稱，一律取第一個。"""
    import yaml
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict) or not raw:
        return {}
    top = next(iter(raw.values()))
    if isinstance(top, dict) and "ros__parameters" in top:
        return top["ros__parameters"]
    return top if isinstance(top, dict) else {}


def locate_arena():
    """找到場地 world 與 x500_nav2 的 model.sdf。

    刻意用「找得到才跑」而不是宣告相依：本套件不能相依 drone_nav2_apriltag，
    否則別人拿走時要連整個場地一起搬。
    """
    roots = []
    share = find_pkg_share("drone_nav2_apriltag")
    if share:
        roots.append(share)
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        roots.append(os.path.join(d, "src", "drone_nav2_apriltag"))
        d = os.path.dirname(d)
    roots.append(os.path.expanduser("~/ros2_ws/src/drone_nav2_apriltag"))

    for r in roots:
        w = os.path.join(r, "gz", "worlds", "nav2_arena.sdf")
        m = os.path.join(r, "gz", "models", "x500_nav2", "model.sdf")
        if os.path.isfile(w) and os.path.isfile(m):
            return r, w, m
    return None, None, None


def parse_tag_pose(world_sdf):
    """從 world 撈 AprilTag 的世界座標（ENU）。

    讀檔案而不是寫死 (27,16)：他把 tag 搬走的時候，這支測試要跟著搬，
    不然會拿舊座標當真值，測出一堆假的失敗。
    """
    root = ET.parse(world_sdf).getroot()
    for inc in root.iter("include"):
        uri = (inc.findtext("uri") or "")
        if "apriltag" not in uri:
            continue
        pose = (inc.findtext("pose") or "0 0 0 0 0 0").split()
        vals = [float(v) for v in pose] + [0.0] * 6
        return vals[:6]
    return None


def extract_camera_link(model_sdf, link_name):
    """把機體上的相機 link 整段抓出來，原封不動塞進探針模型。

    不複製一份貼在這裡的理由：安裝位置和角度（pose 裡的 pitch 1.5707）
    正是這支測試要驗的東西。複製的話改了機體，測試還在驗舊的角度，
    會出現「測試全過但實機是錯的」—— 比沒有測試更糟。
    """
    root = ET.parse(model_sdf).getroot()
    for link in root.iter("link"):
        if link.get("name") == link_name:
            return link
    return None


def link_pose_of(link):
    p = (link.findtext("pose") or "0 0 0 0 0 0").split()
    vals = [float(v) for v in p] + [0.0] * 6
    return vals[:6]


def build_probe_sdf(cam_link, pose):
    """組出探針模型：一個靜態模型，只有相機那個 link，放在指定位置。

    <static>true</static> 是關鍵 —— 不加的話模型會直接掉到地上，
    根本來不及拍。靜態模型連物理都不用跑，位置就是我們指定的那個，真值零誤差。

    位置寫進 <model><pose>，而不是用 EntityFactory 的 pose 欄位：
    這是 PX4 的 px4-rc.gzsim 用的做法（見該檔 sdf_pose_str），已知可用。
    """
    model = ET.Element("model", {"name": PROBE_NAME})
    ET.SubElement(model, "pose").text = " ".join(f"{v:.6f}" for v in pose)
    ET.SubElement(model, "static").text = "true"
    # 整段 link 原封不動塞進去，包含 <pose>、<sensor>、<visual>
    model.append(cam_link)
    sdf = ET.Element("sdf", {"version": "1.9"})
    sdf.append(model)
    raw = ET.tostring(sdf, encoding="unicode")
    # ⚠️ 壓成單行：gz service 的 --req 是 protobuf 文字格式，
    #    字串字面值裡不能有未跳脫的換行，有的話服務會直接拒絕而且不說原因。
    return " ".join(raw.split())


# =============================================================================
#  Gazebo 操作
# =============================================================================

def gz_env(arena_root):
    env = os.environ.copy()
    # PX4 的 gz_env.sh 會設 GZ_SIM_SERVER_CONFIG_PATH，那份 server.config
    # 才有 gz-sim-sensors-system。少了它相機不會算影像，而且完全不會報錯。
    px4 = env.get("PX4_DIR", os.path.expanduser("~/PX4-Autopilot"))
    cfg = os.path.join(px4, "src", "modules", "simulation",
                       "gz_bridge", "server.config")
    if os.path.isfile(cfg):
        env["GZ_SIM_SERVER_CONFIG_PATH"] = cfg
    res = [os.path.join(arena_root, "gz", "models")]
    if env.get("GZ_SIM_RESOURCE_PATH"):
        res.append(env["GZ_SIM_RESOURCE_PATH"])
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(res)
    return env


def gz_call(service, reqtype, req, env, timeout=5000):
    cmd = ["gz", "service", "-s", service,
           "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
           "--timeout", str(timeout), "--req", req]
    r = subprocess.run(cmd, capture_output=True, text=True, env=env)
    ok = "data: true" in r.stdout
    detail = (r.stdout + r.stderr).strip()
    return ok, detail


def spawn_probe(world, sdf_str, env):
    esc = sdf_str.replace("\\", "\\\\").replace('"', '\\"')
    req = f'sdf: "{esc}" name: "{PROBE_NAME}" allow_renaming: false'
    return gz_call(f"/world/{world}/create", "gz.msgs.EntityFactory", req, env)


def remove_probe(world, env):
    req = f'name: "{PROBE_NAME}" type: MODEL'
    return gz_call(f"/world/{world}/remove", "gz.msgs.Entity", req, env)


# =============================================================================
#  ROS 端：訂 /tf，把 tag 位姿轉成世界座標
# =============================================================================

class TagObserver:
    """訂閱 apriltag 的 /tf 與 detections，把觀測結果換算成 tag 的世界座標。

    整條換算鏈就是降落節點之後要實作的那條 —— 這支先把它驗過，
    C++ 節點照抄同一套公式，不用再賭一次。
    """

    def __init__(self, node, cam_link_pose):
        import tf2_ros
        from apriltag_msgs.msg import AprilTagDetectionArray
        from rclpy.qos import qos_profile_sensor_data

        self.node = node
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, node)
        self.last_detection = None
        node.create_subscription(
            AprilTagDetectionArray, "/detections",
            self._on_det, qos_profile_sensor_data)

        # 相機 link 在探針本體座標系裡的位置與姿態（來自 model.sdf）
        lx, ly, lz, lr, lp, lyaw = cam_link_pose
        self.link_offset = [lx, ly, lz]
        self.R_link_to_body = rot_rpy(lr, lp, lyaw)
        # 光學系 → 本體系 = (link→body) x (optical→link)
        self.R_optical_to_body = mat_mul(self.R_link_to_body, R_OPTICAL_TO_LINK)

    def _on_det(self, msg):
        if msg.detections:
            self.last_detection = msg

    def tag_frames(self):
        """列出 TF 樹上所有 frame，用來找出 apriltag 幫 tag 取的名字。

        不寫死名字：apriltag_ros 的 frame 命名會隨 tag.frames 參數改變，
        寫死的話換個設定就查不到，而且錯誤訊息會是「查不到 transform」，
        很難聯想到是名字問題。
        """
        import yaml
        try:
            data = yaml.safe_load(self.buffer.all_frames_as_yaml())
        except Exception:
            return {}
        return data if isinstance(data, dict) else {}

    def tag_in_world(self, cam_frame, tag_frame, probe_pose):
        """把 tag 的觀測位姿換算成世界（ENU）座標。

        鏈路：光學系 -> link -> 探針本體 -> 世界 ENU
        """
        import rclpy
        from rclpy.duration import Duration
        try:
            tf = self.buffer.lookup_transform(
                cam_frame, tag_frame, rclpy.time.Time(),
                timeout=Duration(seconds=1.0))
        except Exception as e:
            return None, None, str(e)

        t = tf.transform.translation
        q = tf.transform.rotation

        # 1) 光學系 -> 探針本體系
        v_body = mat_vec(self.R_optical_to_body, [t.x, t.y, t.z])
        # 相機 link 本身在本體上的平移
        v_body = [v_body[i] + self.link_offset[i] for i in range(3)]

        # 2) 本體系 -> 世界 ENU（探針只有 yaw，roll/pitch 皆 0）
        px, py, pz, _, _, pyaw = probe_pose
        c, s = math.cos(pyaw), math.sin(pyaw)
        wx = px + v_body[0] * c - v_body[1] * s
        wy = py + v_body[0] * s + v_body[1] * c
        wz = pz + v_body[2]

        # 3) tag 的 yaw：光學系裡繞光軸轉多少，就是世界系裡相對機頭轉多少
        tag_yaw_cam = quat_to_yaw(q.x, q.y, q.z, q.w)
        return [wx, wy, wz], tag_yaw_cam, None


# =============================================================================
#  測試位置
# =============================================================================

def build_cases(tag_xyz):
    """每個位置都針對一種可能的錯誤。

    只放一個「正上方」是不夠的：正上方時 x/y 誤差都是 0，
    x 和 y 交換、正負相反通通看不出來。一定要有偏移的位置。
    """
    tx, ty = tag_xyz[0], tag_xyz[1]
    return [
        # (說明, 探針 ENU pose x,y,z,roll,pitch,yaw, 這個位置在驗什麼)
        ("正上方 3 m",
         [tx, ty, 3.0, 0, 0, 0],
         "基本能見度與距離刻度"),
        ("東偏 1 m（ENU +x）",
         [tx + 1.0, ty, 3.0, 0, 0, 0],
         "東西軸的方向與正負"),
        ("北偏 1.5 m（ENU +y）",
         [tx, ty + 1.5, 3.0, 0, 0, 0],
         "南北軸的方向與正負，以及有沒有和東西軸交換"),
        ("東偏 1 m + 北偏 1.5 m",
         [tx + 1.0, ty + 1.5, 3.0, 0, 0, 0],
         "兩軸同時偏移，交換或轉置在這裡藏不住"),
        ("正上方 1.5 m（低空）",
         [tx, ty, 1.5, 0, 0, 0],
         "接近交接高度時仍然看得到、距離仍然正確"),
        ("正上方 3 m，機頭轉 +40 度",
         [tx, ty, 3.0, 0, 0, math.radians(40.0)],
         "機頭轉動時位置不受影響（位置與航向沒有互相污染）"),
        ("正上方 3 m，機頭轉 -25 度",
         [tx, ty, 3.0, 0, 0, math.radians(-25.0)],
         "反向轉動，用來確認航向的正負與線性"),
    ]


# =============================================================================
#  子程序管理
# =============================================================================

class Procs:
    """統一管理起出來的子程序，確保不管怎麼結束都會關乾淨。

    刻意用 process group + SIGTERM -> SIGKILL 兩段式：
    gz 是 Ruby 包裝腳本，直接 kill 父程序會留下真正的 server 在背景跑，
    而殘留的 Gazebo 會佔著 GPU 和 gz topic，害下一次執行行為詭異。
    """

    def __init__(self, marker=None):
        self.items = []
        # 用來精準辨認「這次測試起的程序」的字串，通常是 world 檔的完整路徑
        self.marker = marker

    def start(self, name, cmd, env=None, log=None):
        f = open(log, "wb") if log else subprocess.DEVNULL
        p = subprocess.Popen(cmd, env=env, stdout=f, stderr=subprocess.STDOUT,
                             preexec_fn=os.setsid)
        self.items.append((name, p, f))
        return p

    def stop_all(self, verbose=True):
        for name, p, f in reversed(self.items):
            if p.poll() is not None:
                continue
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                pass
        deadline = time.time() + 5.0
        while time.time() < deadline and any(p.poll() is None
                                             for _, p, _ in self.items):
            time.sleep(0.2)
        for name, p, f in self.items:
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
        # 補刀：只針對「命令列含有我們這次用的 world 檔路徑」的程序。
        #
        # 絕對不要用 pkill -f "gz sim" —— 那會連「命令列裡剛好出現這三個字」
        # 的無關程序一起殺掉，包括呼叫這支腳本的那個 shell 自己。
        # 這個坑我踩過：整個測試在收工那一步把自己的終端機幹掉。
        if self.marker:
            for pid in self._leftovers():
                try:
                    os.kill(int(pid), signal.SIGKILL)
                except Exception:
                    pass
        if verbose:
            left = self._leftovers()
            print(f"\n   收工：Gazebo 殘留 = "
                  f"{'有！pid ' + ' '.join(left) if left else '無'}")

    def _leftovers(self):
        """找出還活著、且確實屬於這次測試的程序。

        用 world 檔的完整路徑當標記，比對到之後還要再排除自己和 shell ——
        腳本自己的命令列也含有那個路徑，不排除的話會自殺。
        """
        if not self.marker:
            return []
        out = subprocess.run(["pgrep", "-f", self.marker],
                             capture_output=True, text=True).stdout.split()
        skip = {os.getpid(), os.getppid()}
        alive = []
        for pid in out:
            try:
                n = int(pid)
            except ValueError:
                continue
            if n in skip:
                continue
            try:
                with open(f"/proc/{n}/comm") as f:
                    comm = f.read().strip()
            except OSError:
                continue
            # 自己的工具鏈不算殘留
            if comm in ("bash", "sh", "python3", "pgrep", "grep", "timeout"):
                continue
            alive.append(pid)
        return alive


# =============================================================================
#  主流程
# =============================================================================

def main():
    ap = argparse.ArgumentParser(
        description="AprilTag 位姿與座標系轉換驗證（開 Gazebo，但不飛）")
    ap.add_argument("--gui", action="store_true", help="開 Gazebo 視窗")
    ap.add_argument("--keep-gz", action="store_true",
                    help="跑完不關 Gazebo（除錯用，記得自己 pkill -f 'gz sim'）")
    ap.add_argument("--settle", type=float, default=2.5,
                    help="每個位置生成後等幾秒讓影像與偵測穩定，預設 2.5")
    args = ap.parse_args()

    rep = Report()
    procs = Procs()   # marker 等找到 world 檔後再設
    scratch = os.path.join("/tmp", f"t2_{os.getpid()}")
    os.makedirs(scratch, exist_ok=True)

    try:
        # ---- C0 前置 ----
        rep.section("C0 環境與場景")
        arena_root, world_sdf, model_sdf = locate_arena()
        if not world_sdf:
            rep.fail("找不到 drone_nav2_apriltag 的場地檔，"
                     "T2 需要它提供世界與機體模型")
            return 1
        rep.ok(f"場地：{world_sdf}")
        procs.marker = world_sdf

        tag_pose = parse_tag_pose(world_sdf)
        if not tag_pose:
            rep.fail("world 裡找不到 AprilTag")
            return 1
        rep.ok(f"tag 世界座標（ENU）：({tag_pose[0]}, {tag_pose[1]}, {tag_pose[2]})")

        cam_link = extract_camera_link(model_sdf, "camera_down_link")
        if cam_link is None:
            rep.fail("x500_nav2 裡找不到 camera_down_link")
            return 1
        sdf_pose = link_pose_of(cam_link)
        rep.ok(f"相機 link 安裝位姿（讀自 model.sdf）：{sdf_pose}")

        # 換算時用「降落節點實際會用的參數」，而不是 model.sdf 的值。
        # 探針的幾何來自 SDF、數學來自 yaml —— 兩者不一致時位置就會對不上，
        # 等於順便驗了「參數有沒有抄錯」。這比只驗數學本身有價值得多。
        land_cfg = find_config_file("landing.yaml")
        if not land_cfg:
            rep.fail("找不到 landing.yaml")
            return 1
        land = load_ros_params(land_cfg)
        off = list(land.get("camera_offset", [0.0, 0.0, 0.0]))
        rpy = list(land.get("camera_rpy", [0.0, 0.0, 0.0]))
        cam_link_pose = (off + [0.0] * 3)[:3] + (rpy + [0.0] * 3)[:3]
        rep.ok(f"相機安裝參數（讀自 landing.yaml）：{cam_link_pose}")

        diff = max(abs(a - b) for a, b in zip(cam_link_pose, sdf_pose))
        if diff > 1e-3:
            rep.fail(f"landing.yaml 的相機安裝參數和 model.sdf 對不上"
                     f"（最大差 {diff:.4f}）。節點會用錯的安裝角度換算，"
                     "飛機會往偏掉的地方降 —— 下面的位置檢查也會跟著失敗")
        else:
            rep.ok("yaml 的相機參數和 model.sdf 一致")

        env = gz_env(arena_root)
        if "GZ_SIM_SERVER_CONFIG_PATH" not in env:
            rep.warn("找不到 PX4 的 server.config，相機可能不會算影像。"
                     "用 PX4_DIR 指定 PX4-Autopilot 位置")
        else:
            rep.ok("server.config 已設定（含 gz-sim-sensors-system）")

        world_name = ET.parse(world_sdf).getroot().find("world").get("name")

        # ---- 起 Gazebo ----
        rep.section("C1 啟動 Gazebo（不啟動 PX4，不起飛）")
        gz_cmd = ["gz", "sim", "-s", "-r", world_sdf]
        procs.start("gz", gz_cmd, env=env, log=os.path.join(scratch, "gz.log"))
        if args.gui:
            procs.start("gzgui", ["gz", "sim", "-g"], env=env,
                        log=os.path.join(scratch, "gui.log"))

        ok = False
        for _ in range(30):
            time.sleep(1.0)
            out = subprocess.run(["gz", "topic", "-l"], capture_output=True,
                                 text=True, env=env).stdout
            if f"/world/{world_name}/clock" in out:
                ok = True
                break
        if not ok:
            rep.fail("Gazebo 起不來（30 秒內沒看到 clock topic）")
            return 1
        rep.ok(f"Gazebo 已啟動，world = {world_name}")

        # ---- 起橋接與偵測器 ----
        gz_img = (f"/world/{world_name}/model/{PROBE_NAME}"
                  f"/link/camera_down_link/sensor/imager_down/image")
        gz_info = gz_img.rsplit("/", 1)[0] + "/camera_info"

        tag_cfg = find_config_file("tag_36h11.yaml")
        if not tag_cfg:
            rep.fail("找不到 tag_36h11.yaml")
            return 1

        procs.start("img_bridge",
                    ["ros2", "run", "ros_gz_image", "image_bridge", gz_img],
                    env=env, log=os.path.join(scratch, "img.log"))
        procs.start("info_bridge",
                    ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                     f"{gz_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
                    env=env, log=os.path.join(scratch, "info.log"))
        procs.start("apriltag",
                    ["ros2", "run", "apriltag_ros", "apriltag_node",
                     "--ros-args", "--params-file", tag_cfg,
                     "-r", f"image_rect:={gz_img}",
                     "-r", f"camera_info:={gz_info}"],
                    env=env, log=os.path.join(scratch, "apriltag.log"))
        rep.ok("影像橋接與 apriltag_node 已啟動")

        # ---- ROS 節點 ----
        import rclpy
        rclpy.init()
        node = rclpy.create_node("t2_pose_check")
        obs = TagObserver(node, cam_link_pose)

        def spin(seconds):
            end = time.time() + seconds
            while time.time() < end:
                rclpy.spin_once(node, timeout_sec=0.05)

        # ---- 逐個位置檢查 ----
        rep.section("C2 位姿換算（每個位置真值都是已知的）")
        cases = build_cases(tag_pose)
        cam_frame, tag_frame = None, None
        yaw_samples = []   # (探針 yaw, 量到的 tag yaw)

        for label, pose, purpose in cases:
            remove_probe(world_name, env)
            time.sleep(0.5)
            ok, detail = spawn_probe(
                world_name, build_probe_sdf(cam_link, pose), env)
            if not ok:
                rep.fail(f"{label}：探針生成失敗 —— {detail[:200]}")
                continue
            obs.last_detection = None
            spin(args.settle)

            # 第一次要先找出 apriltag 用的 frame 名稱
            if tag_frame is None:
                frames = obs.tag_frames()
                for f, info in frames.items():
                    parent = info.get("parent") if isinstance(info, dict) else None
                    if parent:
                        cam_frame, tag_frame = parent, f
                        break
                if tag_frame is None:
                    rep.fail(f"{label}：TF 上沒有任何 tag frame（偵測不到）")
                    continue
                print(f"   TF：{cam_frame} -> {tag_frame}")

            got, tag_yaw_cam, err = obs.tag_in_world(cam_frame, tag_frame, pose)
            if got is None:
                rep.fail(f"{label}：查不到 TF（{err}）")
                continue

            dx = got[0] - tag_pose[0]
            dy = got[1] - tag_pose[1]
            dz = got[2] - tag_pose[2]
            dist = math.sqrt(dx * dx + dy * dy)

            print(f"\n   [{label}]  {purpose}")
            print(f"     探針 ENU  : ({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}) "
                  f"yaw {math.degrees(pose[5]):.0f}°")
            print(f"     tag 真值   : ({tag_pose[0]:.2f}, {tag_pose[1]:.2f})")
            print(f"     算出來的   : ({got[0]:.3f}, {got[1]:.3f}, {got[2]:.3f})")
            print(f"     水平差     : Δx(東) {dx:+.3f}  Δy(北) {dy:+.3f}  "
                  f"合成 {dist:.3f} m")

            if dist > POS_TOL:
                # 診斷：把誤差的形狀跟幾種典型錯誤比對。
                #
                # 推導：探針在 tag 東邊 sx 處，正確算出來的位移是 -sx（往西看到 tag）。
                # 若東西軸符號寫反，會算成 +sx，於是 tag 被放到 tag_x + 2*sx，
                # 也就是誤差 dx = +2*sx。南北同理。
                #
                # ⚠️ 每個提示都必須先確認「該軸真的有偏移」才成立 ——
                #    偏移是 0 的軸，任何符號錯誤都不會顯現，這時比對條件會
                #    退化成 0 ≈ 0 而恆真，就會指錯軸（這個錯我犯過）。
                hints = []
                sx = pose[0] - tag_pose[0]
                sy = pose[1] - tag_pose[1]
                tol = POS_TOL * 4
                if abs(sx) > 0.2 and abs(dx - 2 * sx) < tol:
                    hints.append("東西軸正負相反")
                if abs(sy) > 0.2 and abs(dy - 2 * sy) < tol:
                    hints.append("南北軸正負相反")
                # 交換：算出來的位移變成 (-sy, -sx)，於是誤差 = (sx - sy, sy - sx)
                if (abs(sx - sy) > 0.2
                        and abs(dx - (sx - sy)) < tol
                        and abs(dy - (sy - sx)) < tol):
                    hints.append("東西與南北軸交換")
                rep.fail(f"{label}：算出來的 tag 位置差了 {dist:.3f} m"
                         + (f" —— 疑似 {'、'.join(hints)}" if hints
                            else "（誤差形狀不符合單純的軸交換或符號錯誤，"
                                 "可能是旋轉角度或 tag_size）"))
            else:
                rep.ok(f"{label}：水平誤差 {dist * 100:.1f} cm（≤ "
                       f"{POS_TOL * 100:.0f} cm）")

            if tag_yaw_cam is not None:
                yaw_samples.append((pose[5], tag_yaw_cam))

            if abs(dz) > 0.15:
                rep.fail(f"{label}：高度差 {dz:+.3f} m。"
                         "垂直方向錯了，多半是 tag_size 或相機 link 的 z 偏移")

        # ---- 航向 ----
        rep.section("C3 航向的正負與刻度")
        # 做法：把探針轉到不同角度，看量到的 tag 角度跟著怎麼變。
        # 不用絕對值比對（那要先知道 tag 在世界裡的朝向與相機的零位），
        # 改用「相對變化量」—— 這樣不必假設任何零點，卻足以釘死正負與刻度，
        # 而那兩件事正是降落節點轉機頭時會轉錯方向的原因。
        if len(yaw_samples) < 3:
            rep.fail(f"只取得 {len(yaw_samples)} 組航向樣本，不足以判斷")
        else:
            base_p, base_m = yaw_samples[0]
            signs, rows = [], []
            for pj, mj in yaw_samples[1:]:
                dp = wrap_pi(pj - base_p)
                dm = wrap_pi(mj - base_m)
                if abs(dp) < math.radians(1.0):
                    continue
                ratio = dm / dp
                rows.append((math.degrees(dp), math.degrees(dm), ratio))
                signs.append(ratio)
            for dp, dm, r in rows:
                print(f"     探針轉 {dp:+.1f}°  ->  量到 tag 轉 {dm:+.1f}°  "
                      f"(比值 {r:+.3f})")
            if not signs:
                rep.fail("沒有任何有效的轉動樣本")
            else:
                bad = [r for r in signs if abs(abs(r) - 1.0) > 0.08]
                if bad:
                    rep.fail(f"航向刻度不對：比值應該是 ±1.000，"
                             f"實際 {['%+.3f' % r for r in signs]}。"
                             "不是 1 代表中間有多餘或缺少的旋轉")
                elif len({r > 0 for r in signs}) != 1:
                    rep.fail(f"航向正負不一致：{['%+.3f' % r for r in signs]}。"
                             "不同轉向給出不同符號，代表換算式有問題")
                else:
                    pos = signs[0] > 0
                    sgn = "+1（同向）" if pos else "-1（反向）"
                    rep.ok(f"航向刻度 1:1，相對 ENU yaw 的正負 = {sgn}")
                    # ⚠️ 這裡量到的是「相對探針 ENU yaw」的關係，而降落節點用的是
                    #    PX4 的 NED heading。ENU yaw（從東起算、逆時針為正）和
                    #    NED heading（從北起算、順時針為正）旋轉方向相反，
                    #    換算過去符號要翻一次。
                    #    這個轉換漏掉過一次：節點的 yaw 穩定在差 180 度的地方，
                    #    水平位置卻收斂得很漂亮，看起來完全不像符號問題。
                    print(f"     ⚠️ 以上是相對「ENU yaw」。ENU 與 PX4 的 NED "
                          f"heading 旋轉方向相反，換算後符號要翻轉：")
                    print(f"     => 降落節點應該用："
                          f"yaw_error = {'+' if pos else '-'}tag_yaw_in_cam"
                          f"（目標 heading = 現在 heading + yaw_error）")

        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print("\n(中斷)")
        return 130
    finally:
        try:
            remove_probe(world_name, env)  # noqa
        except Exception:
            pass
        if not args.keep_gz:
            procs.stop_all()
        else:
            print("\n   --keep-gz：Gazebo 保留中，"
                  "記得自己關：pkill -f 'gz sim'")
        shutil.rmtree(scratch, ignore_errors=True)

    print()
    if rep.failed:
        print(f"結果：失敗 —— {rep.failed} 項不通過"
              + (f"，{rep.warned} 項警告" if rep.warned else ""))
        print("⚠️ T2 沒過之前不要飛（T3）。方向錯的飛機看起來很像正常在動。")
        return 1
    print("結果：全部通過"
          + (f"（{rep.warned} 項警告）" if rep.warned else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
