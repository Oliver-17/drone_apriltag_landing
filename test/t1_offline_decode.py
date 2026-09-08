#!/usr/bin/env python3
# =============================================================================
#  t1_offline_decode.py — 偵測參數與可視範圍的離線檢查（T1）
#
#  用法：
#      python3 src/drone_apriltag_landing/test/t1_offline_decode.py
#      python3 .../t1_offline_decode.py --tag-image /path/to/tag.png
#      python3 .../t1_offline_decode.py --hfov 1.2 --width 640 --height 480
#
#  回傳值：全部通過 0，有任何一項失敗 1。可以直接串進 CI 或 pre-commit。
#
#  為什麼需要這支：
#      這一層抓的是「不會報錯的錯」。降落失敗的原因裡，最難查的三種都在這裡：
#
#        1. tag_id 填錯      → 永遠偵測不到，畫面正常、節點正常、就是沒反應
#        2. tag_size 填錯    → ID 照樣讀得到，但距離整個縮放，飛機以為自己在別的高度
#        3. handoff 高度太低 → 對準過程中 tag 出框，狀態機在「下降/丟失」之間無限來回
#
#      三種都不會產生任何錯誤訊息。開了 Gazebo 再查要花好幾個小時，
#      在這裡花 30 秒就抓得到。
#
#      而且這支完全不需要 ROS、不需要 Gazebo、不需要飛機，改完參數立刻能重跑。
#
#  它抓不到什麼（誠實說明）：
#      - 座標系轉換方向錯誤（x/y 交換、符號相反）—— 要靠 T2，那要開 Gazebo
#      - 真實光線、動態模糊、相機曝光 —— 模擬合成圖一律完美，只有實機能驗
#      - 狀態機邏輯、控制收斂 —— 要靠 T3
# =============================================================================

import argparse
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile

PKG_NAME = "drone_apriltag_landing"
PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 現成的離線偵測器，apt 裝 ros-humble-apriltag 就會有。
# 用它而不是自己接 apriltag_node：這一層要的就是「不碰 ROS」。
APRILTAG_DEMO = "/opt/ros/humble/bin/apriltag_demo"

# 預設的相機規格，對應 drone_nav2_apriltag 的 x500_nav2 下視相機。
# 別人拿這包去用別的相機時，用 --hfov / --width / --height 覆蓋。
DEFAULT_HFOV = 1.74
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 960

# 貼圖裡「黑框外緣」佔整張圖的比例。
# 不寫死常數而是每次從圖片量出來 —— 換一張貼圖版面就可能不同，
# 寫死的話換圖後所有距離都會靜默地錯掉。
#
# 已用 apriltag_node 實測確認：它回報的 corners 就落在黑框外緣，
# 所以 tag_size 這個參數指的是黑框外緣的邊長，不是整張貼圖。

# 地磚（含白色靜區）在世界裡的實際邊長，公尺。
# 只在有提供貼圖時用來反推 tag_size 應該是多少，可用 --plane-size 覆蓋。
DEFAULT_PLANE_SIZE = 1.4

# 交接高度至少要比「偵測得到的最低高度」高這個倍率。
# 1.3 的理由：對準誤差容許 0.15 m，加上下降過程的超調與風擾，
# 剛好卡在邊界的話一晃就丟失。
HANDOFF_MARGIN = 1.3


# =============================================================================
#  報告（格式和 drone_nav2_apriltag/scripts/check_graph.py 一致）
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
#  讀檔
# =============================================================================

def find_config(name):
    """先找原始碼樹，再找 install 樹。

    理由：這支腳本會在兩個完全不同的路徑下被執行 ——
        原始碼樹  <pkg>/test/t1_offline_decode.py
        install 樹 <prefix>/lib/<pkg>/t1_offline_decode.py
    兩邊的相對位置不一樣，只認一邊的話另一邊會莫名其妙找不到檔案。
    這個坑很容易在「開發時好好的、colcon build 完就壞掉」的形式出現。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        # 原始碼樹：test/ 的上一層就是套件根目錄
        os.path.join(os.path.dirname(here), "config", name),
        # install 樹：lib/<pkg>/ 往上兩層是 prefix，再進 share/<pkg>/
        os.path.join(os.path.dirname(os.path.dirname(here)),
                     "share", PKG_NAME, "config", name),
    ]
    # 最後才問 ament（要 source 過才有，所以不能當唯一手段）
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(
            os.path.join(get_package_share_directory(PKG_NAME), "config", name))
    except Exception:
        pass

    for c in candidates:
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


def find_tag_image(explicit):
    """找一張 tag 貼圖來驗。

    刻意「找得到才驗、找不到就跳過」而不是硬性相依：
    這包不能依賴 drone_nav2_apriltag，否則別人拿走時要連場地一起拿。
    """
    if explicit:
        return explicit if os.path.isfile(explicit) else None
    rel = os.path.join("drone_nav2_apriltag", "gz", "models",
                       "apriltag_36h11", "apriltag_36h11.png")
    # 從這支腳本往上逐層找 src/，install 樹和原始碼樹都能命中
    guesses = []
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        guesses.append(os.path.join(d, "src", rel))
        d = os.path.dirname(d)
    guesses.append(os.path.expanduser("~/ros2_ws/src/" + rel))
    for g in guesses:
        if os.path.isfile(g):
            return g
    return None


# =============================================================================
#  偵測
# =============================================================================

DETECT_RE = re.compile(
    r"id \((\d+)x(\d+)\)-(\d+)\s*,\s*hamming (\d+),\s*margin\s+([\d.]+)")


def run_detector(paths):
    """一次餵多張圖給 apriltag_demo，回傳 {檔名: [(id, hamming, margin), ...]}。

    一次跑多張而不是每張跑一次：偵測器初始化要花時間，
    分開跑 12 張會多花十幾秒，這支腳本的價值就在於「改完馬上能重跑」。
    """
    cmd = [APRILTAG_DEMO, "-a", "0"] + paths
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=180).stdout

    results = {p: [] for p in paths}
    current = None
    for line in out.splitlines():
        # apriltag_demo 每處理一張圖會先印 "loading <路徑>"，
        # 後面的 detection 行不含檔名，所以要靠這行判斷現在讀到哪一張。
        for p in paths:
            if line.startswith("loading " + p) or line.startswith("image: " + p):
                current = p
                break
        m = DETECT_RE.search(line)
        if m and current is not None:
            results[current].append(
                (int(m.group(3)), int(m.group(4)), float(m.group(5))))
    return results


def measure_body_ratio(tag_png):
    """量出黑框外緣佔整張貼圖的比例。

    做法是掃描哪些列/行含有暗像素 —— 白色靜區不會觸發，黑框會。
    這個比例是把「貼圖像素」換算成「公尺」的橋樑，量錯的話
    後面所有高度計算都會跟著錯，所以寧可每次重量也不寫死。
    """
    from PIL import Image
    import numpy as np
    arr = np.array(Image.open(tag_png).convert("L"))
    dark = arr < 100
    rows = np.where(dark.any(axis=1))[0]
    cols = np.where(dark.any(axis=0))[0]
    if rows.size == 0 or cols.size == 0:
        return None
    span = max(rows.max() - rows.min() + 1, cols.max() - cols.min() + 1)
    return span / float(max(arr.shape))


def render_at_altitude(tag_png, altitude, tag_size, hfov, width, height,
                       body_ratio, out_path):
    """合成「無人機在 altitude 公尺高、正下方有這個 tag」時相機會拍到的畫面。

    做法是純幾何：針孔模型下焦距 f = (width/2) / tan(hfov/2)，
    邊長 S 的物體在距離 h 處成像 S*f/h 像素。
    背景刻意用中灰而不是白色 —— 全白的話 tag 四周的白色靜區跟背景連成一片，
    偵測器反而更好找，測出來的結果會比實際樂觀。
    """
    from PIL import Image

    focal_px = (width / 2.0) / math.tan(hfov / 2.0)
    body_px = tag_size * focal_px / altitude
    full_px = int(round(body_px / body_ratio))
    if full_px < 8:
        return None, body_px

    tag = Image.open(tag_png).convert("L").resize(
        (full_px, full_px), Image.LANCZOS)
    canvas = Image.new("L", (width, height), 128)
    canvas.paste(tag, ((width - full_px) // 2, (height - full_px) // 2))
    canvas.save(out_path)
    return out_path, body_px


# =============================================================================
#  各項檢查
# =============================================================================

def check_configs(rep, land, tag_cfg):
    rep.section("C1 參數一致性")

    size_land = land.get("tag_size")
    size_tag = tag_cfg.get("size")
    if size_land is None or size_tag is None:
        rep.fail("兩個設定檔裡缺少 tag_size / size")
    elif abs(float(size_land) - float(size_tag)) > 1e-6:
        rep.fail(f"tag 尺寸不同步：landing.yaml={size_land} 但 "
                 f"tag_36h11.yaml={size_tag}。"
                 "apriltag 用後者算位姿、降落節點用前者判高度，兩邊會打架")
    else:
        rep.ok(f"tag 尺寸兩邊一致：{size_land} m")

    fam = str(tag_cfg.get("family", ""))
    if fam != "36h11":
        rep.fail(f"family 是 {fam!r}，不是 36h11。"
                 "餵錯字典不會報錯，只會永遠偵測不到")
    else:
        rep.ok("family = 36h11")

    mh = tag_cfg.get("max_hamming")
    if mh != 0:
        rep.warn(f"max_hamming = {mh}（建議 0）。"
                 "允許修正位元會提高誤判成別的 ID 的機率")
    else:
        rep.ok("max_hamming = 0（不接受任何位元修正）")

    pem = str(tag_cfg.get("pose_estimation_method", ""))
    if pem != "pnp":
        rep.warn(f"pose_estimation_method = {pem!r}，"
                 "homography 比較快但比較不準，降落建議用 pnp")
    else:
        rep.ok("pose_estimation_method = pnp")

    kp = float(land.get("kp_xy", 0.0))
    if kp <= 0 or kp >= 1.0:
        rep.fail(f"kp_xy = {kp}，應該在 0~1 之間。"
                 "大於等於 1 會一次修過頭，然後來回震盪停不下來")
    else:
        rep.ok(f"kp_xy = {kp}（在合理範圍）")

    for key in ("handoff_altitude", "search_timeout_s", "total_timeout_s"):
        v = land.get(key)
        if v is None or float(v) <= 0:
            rep.fail(f"{key} = {v}，必須是正數")
    if all(land.get(k, 0) and float(land[k]) > 0
           for k in ("handoff_altitude", "search_timeout_s", "total_timeout_s")):
        rep.ok("逾時與交接高度都是正值")

    st, tt = float(land.get("search_timeout_s", 0)), float(land.get("total_timeout_s", 0))
    if st >= tt:
        rep.fail(f"search_timeout_s({st}) >= total_timeout_s({tt})，"
                 "搜尋還沒逾時整體就先逾時了，搜尋逾時等於沒作用")
    else:
        rep.ok(f"逾時層級正確：搜尋 {st}s < 總計 {tt}s")


def check_tag_size(rep, land, tag_png, plane_size, body_ratio):
    """從貼圖反推 tag_size 應該填多少，跟設定檔對照。

    這是 T1 最有價值的一項：tag_size 填錯不會有任何錯誤訊息，
    ID 照樣讀得到，只有距離會整個等比縮放 —— 飛機會以為自己在
    別的高度，然後照著錯的高度下降。
    """
    rep.section("C2 tag 尺寸推算")
    if body_ratio is None:
        rep.fail("貼圖裡找不到黑色區域，無法量測")
        return
    expect = plane_size * body_ratio
    got = float(land.get("tag_size", 0.0))
    print(f"   貼圖黑框佔比 : {body_ratio:.4f}（實測）")
    print(f"   地磚邊長     : {plane_size} m（--plane-size 可改）")
    print(f"   推算 tag_size: {plane_size} x {body_ratio:.4f} = {expect:.4f} m")
    if abs(expect - got) > 0.02:
        rep.fail(f"設定檔寫 {got} m，但依貼圖推算應該是 {expect:.3f} m"
                 f"（差 {abs(expect-got)/max(expect,1e-9)*100:.0f}%）。"
                 "tag_size 錯了 ID 照樣讀得到，但所有距離會等比縮放，"
                 "飛機會照著錯的高度下降")
    else:
        rep.ok(f"tag_size = {got} m，和貼圖推算的 {expect:.3f} m 相符")


def check_decode(rep, land, tag_png):
    """驗貼圖真的解得出來，而且 ID 就是設定檔裡寫的那個。"""
    rep.section("C3 貼圖解碼")

    tmpdir = tempfile.mkdtemp(prefix="t1_decode_")
    try:
        from PIL import Image
        im = Image.open(tag_png).convert("L")
        # 四周補灰底：直接餵原圖的話 tag 邊緣貼著影像邊界，
        # 偵測器找不到完整的四邊形。實際拍攝一定有背景，補上才符合真實情況。
        pad = max(im.width // 4, 40)
        canvas = Image.new("L", (im.width + 2 * pad, im.height + 2 * pad), 128)
        canvas.paste(im, (pad, pad))
        p = os.path.join(tmpdir, "plain.pgm")
        canvas.save(p)

        res = run_detector([p]).get(p, [])
        if not res:
            rep.fail(f"這張貼圖完全偵測不到 tag：{tag_png}")
            return None

        tid, ham, margin = res[0]
        rep.ok(f"偵測到 tag，ID = {tid}，hamming = {ham}，margin = {margin:.1f}")

        want = int(land.get("tag_id", -1))
        if tid != want:
            rep.fail(f"設定檔的 tag_id = {want}，但這張貼圖實際是 {tid}。"
                     "填錯不會報錯，只會永遠鎖不上目標")
        else:
            rep.ok(f"tag_id 對得上（{want}）")

        if ham != 0:
            rep.fail(f"hamming = {ham}，理想貼圖不該需要位元修正")

        thr = float(land.get("min_decision_margin", 0))
        if margin < thr:
            rep.fail(f"margin {margin:.1f} 低於門檻 {thr}。"
                     "連完美的合成圖都過不了，實拍一定更差")
        else:
            rep.ok(f"margin {margin:.1f} ≥ 門檻 {thr}")
        return tid
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)


def check_rotation(rep, tag_png, expect_id):
    """驗 yaw 任意角度都讀得到同一個 ID。

    為什麼要驗：無人機接近時機頭朝向是任意的，如果 ID 會隨角度改變，
    整個「用 tag 對齊航向」的設計就不成立。
    """
    rep.section("C4 旋轉不變性")
    tmpdir = tempfile.mkdtemp(prefix="t1_rot_")
    try:
        from PIL import Image
        im = Image.open(tag_png).convert("L")
        pad = max(im.width // 4, 40)
        paths, angles = [], (0, 45, 90, 135, 180, 270)
        for deg in angles:
            r = im.rotate(deg, expand=True, fillcolor=128)
            c = Image.new("L", (r.width + 2 * pad, r.height + 2 * pad), 128)
            c.paste(r, (pad, pad))
            p = os.path.join(tmpdir, f"rot{deg}.pgm")
            c.save(p)
            paths.append(p)

        res = run_detector(paths)
        ids = {}
        for deg, p in zip(angles, paths):
            d = res.get(p, [])
            ids[deg] = d[0][0] if d else None

        bad = [f"{d}°→{v}" for d, v in ids.items() if v != expect_id]
        if bad:
            rep.fail(f"旋轉後 ID 不一致：{', '.join(bad)}（期望全部 {expect_id}）")
        else:
            rep.ok(f"0/45/90/135/180/270 度都解出 ID {expect_id}")
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)


def check_visibility(rep, land, tag_png, hfov, width, height, body_ratio):
    """實測「降到多低就看不見 tag」，再檢查交接高度有沒有留夠餘裕。

    這是這支腳本最重要的一項。理論算式只能算出 tag 幾何上還在不在畫面內，
    但偵測器實際能不能解碼是另一回事（邊緣、靜區、解析度都有影響）。
    所以這裡用真的偵測器掃一遍高度，量出經驗值。
    """
    rep.section("C5 可視高度與交接點")

    tag_size = float(land.get("tag_size", 1.0))
    handoff = float(land.get("handoff_altitude", 1.0))
    focal_px = (width / 2.0) / math.tan(hfov / 2.0)

    # 理論值：黑框剛好塞滿畫面短邊的高度
    h_geom = tag_size * focal_px / min(width, height)
    print(f"   相機：hfov {hfov} rad / {width}x{height} / 焦距 {focal_px:.1f} px")
    print(f"   tag 邊長 {tag_size} m")
    print(f"   理論出框高度：{h_geom:.3f} m（低於此高度標記本體塞不進畫面）")

    tmpdir = tempfile.mkdtemp(prefix="t1_vis_")
    try:
        # 由高往低掃。步進 0.1 m 足以定位邊界，再細下去只是多花時間。
        # 低空 0.1 m 一階（邊界在這裡，要細）；高空 0.5 m 一階就夠。
        # 上限拉到 8 m 而不是停在 3 m —— 停在 3 m 的話「最高可偵測 3 m」
        # 只是掃描上限，那個檢查等於沒作用。
        alts = ([round(x * 0.1, 2) for x in range(30, 2, -1)]
                + [round(3.0 + x * 0.5, 2) for x in range(1, 11)])
        alts = sorted(set(alts), reverse=True)
        paths, alt_of = [], {}
        for a in alts:
            p = os.path.join(tmpdir, f"h{int(a * 100):04d}.pgm")
            made, _ = render_at_altitude(tag_png, a, tag_size, hfov,
                                         width, height, body_ratio, p)
            if made:
                paths.append(p)
                alt_of[p] = a

        res = run_detector(paths)
        detected = sorted(a for p, a in alt_of.items() if res.get(p))
        if not detected:
            rep.fail("所有高度都偵測不到，合成或參數有問題")
            return

        h_min, h_max = detected[0], detected[-1]
        # 連續性檢查：中間破洞代表偵測不穩，而不是單純的邊界
        expected = [a for a in sorted(alt_of.values()) if h_min <= a <= h_max]
        holes = [a for a in expected if a not in detected]

        rep.ok(f"實測可偵測範圍：{h_min:.2f} m ~ {h_max:.2f} m")
        if holes:
            rep.warn(f"範圍內有 {len(holes)} 個高度偵測不到："
                     f"{', '.join(f'{h:.2f}' for h in holes[:6])}"
                     f"{' ...' if len(holes) > 6 else ''}")

        need = h_min * HANDOFF_MARGIN
        if handoff < h_min:
            rep.fail(f"handoff_altitude = {handoff:.2f} m 低於實測下限 "
                     f"{h_min:.2f} m。飛機降到那裡時 tag 已經消失，"
                     "狀態機會在「下降」和「丟失」之間無限來回，永遠落不了地")
        elif handoff < need:
            rep.fail(f"handoff_altitude = {handoff:.2f} m 太貼近下限 "
                     f"{h_min:.2f} m（建議至少 {need:.2f} m）。"
                     "對準誤差加上下降超調，一晃就丟失")
        else:
            rep.ok(f"handoff_altitude = {handoff:.2f} m ≥ 建議值 "
                   f"{need:.2f} m（下限 {h_min:.2f} m × {HANDOFF_MARGIN}）")

        sweep_top = max(alt_of.values())
        if h_max < 4.0:
            rep.warn(f"高於 {h_max:.2f} m 就偵測不到了。"
                     "上層導航必須把飛機帶到這個高度以下才呼叫降落，"
                     "否則一開始就進不了 ALIGNING")
        elif h_max >= sweep_top:
            rep.ok(f"掃到上限 {sweep_top:.1f} m 都還偵測得到"
                   "（真正的上限更高，這裡不再往上掃）")
        else:
            rep.ok(f"最高可偵測 {h_max:.2f} m，一般巡航高度進得來")
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)


# =============================================================================
#  主程式
# =============================================================================

def main():
    ap = argparse.ArgumentParser(
        description="AprilTag 偵測參數與可視範圍的離線檢查（不需要 ROS/Gazebo）")
    ap.add_argument("--landing-config", default=None)
    ap.add_argument("--tag-config", default=None)
    ap.add_argument("--tag-image", default=None,
                    help="要驗的貼圖，預設自動找 drone_nav2_apriltag 那張")
    ap.add_argument("--plane-size", type=float, default=DEFAULT_PLANE_SIZE,
                    help="含白色靜區的地磚邊長（公尺），用來反推 tag_size，"
                         f"預設 {DEFAULT_PLANE_SIZE}")
    ap.add_argument("--hfov", type=float, default=DEFAULT_HFOV,
                    help=f"相機水平視角（弧度），預設 {DEFAULT_HFOV}")
    ap.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    ap.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    args = ap.parse_args()

    rep = Report()

    # ---- 前置：工具與檔案 ----
    rep.section("C0 環境與讀檔")
    if not os.path.isfile(APRILTAG_DEMO):
        rep.fail(f"找不到 {APRILTAG_DEMO}，請先 apt install ros-humble-apriltag")
        print("\n結果：失敗")
        return 1
    rep.ok(f"偵測器：{APRILTAG_DEMO}")

    try:
        import yaml  # noqa: F401
        from PIL import Image  # noqa: F401
    except ImportError as e:
        rep.fail(f"缺少 Python 套件：{e}（需要 python3-yaml 與 python3-pil）")
        print("\n結果：失敗")
        return 1

    land_path = args.landing_config or find_config("landing.yaml")
    tag_path = args.tag_config or find_config("tag_36h11.yaml")
    if not land_path or not tag_path:
        rep.fail("找不到設定檔，用 --landing-config / --tag-config 指定")
        print("\n結果：失敗")
        return 1
    rep.ok(f"landing.yaml  : {land_path}")
    rep.ok(f"tag_36h11.yaml: {tag_path}")

    land = load_ros_params(land_path)
    tag_cfg = load_ros_params(tag_path)

    tag_png = find_tag_image(args.tag_image)
    if tag_png:
        rep.ok(f"貼圖：{tag_png}")
    else:
        rep.warn("找不到 tag 貼圖，C2~C5 會跳過。"
                 "用 --tag-image 指定一張就能完整檢查"
                 "（本套件刻意不相依 drone_nav2_apriltag，所以找不到不算失敗）")

    # ---- 各項檢查 ----
    check_configs(rep, land, tag_cfg)

    if tag_png:
        ratio = measure_body_ratio(tag_png)
        check_tag_size(rep, land, tag_png, args.plane_size, ratio)
        tid = check_decode(rep, land, tag_png)
        if tid is not None:
            check_rotation(rep, tag_png, tid)
        if ratio:
            check_visibility(rep, land, tag_png, args.hfov,
                             args.width, args.height, ratio)

    print()
    if rep.failed:
        print(f"結果：失敗 —— {rep.failed} 項不通過"
              + (f"，{rep.warned} 項警告" if rep.warned else ""))
        return 1
    print("結果：全部通過"
          + (f"（{rep.warned} 項警告）" if rep.warned else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
