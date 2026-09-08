# drone_apriltag_landing — AprilTag 視覺精準降落

> 把無人機從「大概在降落點上方」帶到「準確停在 AprilTag 上」。

ROS 2 Humble + PX4 的**精準降落套件**，附三層可獨立執行的驗證工具。
**不綁定任何場地或機體** —— 相機 topic、tag ID、尺寸、安裝角度、控制增益
全部是參數，換環境只改 `config/landing.yaml`。

<table width="100%">
<thead><tr>
<th width="10%" align="center">驗證</th>
<th width="45%" align="left">內容</th>
<th width="15%" align="center">耗時</th>
<th width="30%" align="left">結果</th>
</tr></thead>
<tbody>
<tr><td align="center"><b>T1</b></td><td align="left">參數一致性、貼圖解碼、可視高度</td><td align="center">0.5 秒</td><td align="left">✅ 全過</td></tr>
<tr><td align="center"><b>T2</b></td><td align="left">座標系與位姿轉換（7 個已知位置）</td><td align="center">90 秒</td><td align="left">✅ 誤差 ≤ 0.4 cm</td></tr>
<tr><td align="center"><b>T3</b></td><td align="left">完整降落飛行</td><td align="center">3 分鐘</td><td align="left">✅ 落點 1.8 / 3.8 cm</td></tr>
<tr><td align="center"><b>整合</b></td><td align="left">接上拓樸地圖，端到端</td><td align="center">5 分鐘</td><td align="left">✅ 落點 9.8 cm</td></tr>
</tbody>
</table>

> 這個 repo **本身就是一個 ROS 2 package**，不是 workspace。
> clone 進你既有 workspace 的 `src/` 底下即可。

---

## 快速導覽

<table width="100%">
<thead><tr>
<th width="50%" align="left">你想知道</th>
<th width="50%" align="left">看哪一節</th>
</tr></thead>
<tbody>
<tr><td align="left"><b>接進我的系統前必讀</b></td><td align="left"><a href="#0-交接協議">0 交接協議</a></td></tr>
<tr><td align="left">這包在做什麼、跟誰的界線在哪</td><td align="left"><a href="#1-這包在做什麼">1 這包在做什麼</a></td></tr>
<tr><td align="left">AprilTag 怎麼算出位置和角度</td><td align="left"><a href="#2-原理">2 原理</a></td></tr>
<tr><td align="left">何時下降、何時交給 PX4</td><td align="left"><a href="#3-狀態機">3 狀態機</a></td></tr>
<tr><td align="left">怎麼證明這包是對的</td><td align="left"><a href="#4-三層驗證">4 三層驗證</a></td></tr>
<tr><td align="left"><b>我要在自己電腦上跑一次</b></td><td align="left"><a href="#5-在自己電腦上驗證">5 在自己電腦上驗證</a></td></tr>
<tr><td align="left"><b>接上拓樸地圖，飛完整趟</b></td><td align="left"><a href="#6-接上拓樸地圖">6 接上拓樸地圖</a></td></tr>
<tr><td align="left">飛行途中要看到相機畫面</td><td align="left"><a href="#看畫面">看畫面</a></td></tr>
<tr><td align="left">有哪些參數、換相機要改什麼</td><td align="left"><a href="#7-參數">7 參數</a></td></tr>
<tr><td align="left">檔案在哪</td><td align="left"><a href="#8-檔案結構">8 檔案結構</a></td></tr>
<tr><td align="left">有什麼坑</td><td align="left"><a href="#9-踩過的雷">9 踩過的雷</a></td></tr>
</tbody>
</table>

---

## 0 交接協議

**接進你的系統之前，只有這一段是非讀不可的。**

> **呼叫本 action 前，呼叫方必須停止發布 `trajectory_setpoint` 與
> `offboard_control_mode`。**

兩邊同時發 → PX4 收到交錯的矛盾指令 → 飛機抽搐。
這個現象在 log 裡看起來很像控制器沒調好，幾乎不可能聯想到是兩個節點在搶。

所以節點會**主動檢查**還有沒有別的發布者，有的話直接拒絕 goal。

停止發布之後 PX4 會在約 0.5 秒內掉出 offboard 進入 HOLD（懸停）。
這個空窗期是正常的，降落節點會自己把模式切回來。

**不想交出控制權**：把 `control_mode` 設成 `advisor`。
節點只把誤差發到 `~/landing_error`，完全不碰飛機，退化成純視覺定位感測器。

---

## 1 這包在做什麼

**只負責最後那幾十秒 —— 從看到 tag 到停在 tag 上。**

<table width="100%">
<thead><tr>
<th width="50%" align="left">✅ 做</th>
<th width="50%" align="left">❌ 不做</th>
</tr></thead>
<tbody>
<tr><td align="left">認出地上的 tag，算出相對位置與航向</td><td align="left">飛到 tag 附近（Nav2 / 編隊的事）</td></tr>
<tr><td align="left">對準 → 下降 → 交接給 PX4 落地</td><td align="left">定義場地、提供 tag 貼圖</td></tr>
<tr><td align="left">找不到就等，逾時回報失敗</td><td align="left">起飛、跑航線</td></tr>
<tr><td align="left">—</td><td align="left">開相機（模擬 <code>ros_gz_image</code>／實機 <code>camera_ros</code>）</td></tr>
</tbody>
</table>

相依只有 `rclcpp` / `rclcpp_action` / `px4_msgs` / `apriltag_msgs` / `tf2`。
**不相依 `drone_nav2_apriltag`** —— 別人拿去不用連帶搬走整個場地。

### 對外介面

```
action   <ns>/precision_land     drone_apriltag_landing/action/PrecisionLand
topic    <ns>/landing_error      geometry_msgs/Vector3Stamped （北誤差, 東誤差, yaw 誤差）
```

### 環境版本

<table width="100%">
<thead><tr>
<th width="50%" align="left">項目</th>
<th width="50%" align="left">版本</th>
</tr></thead>
<tbody>
<tr><td align="left">Ubuntu</td><td align="left">22.04</td></tr>
<tr><td align="left">ROS 2</td><td align="left">Humble</td></tr>
<tr><td align="left">PX4-Autopilot（SITL）</td><td align="left">v1.17.0</td></tr>
<tr><td align="left">Gazebo</td><td align="left">Harmonic 8.15.0</td></tr>
<tr><td align="left"><code>apriltag_ros</code></td><td align="left">3.4.0</td></tr>
<tr><td align="left"><code>apriltag</code>（C 演算法）</td><td align="left">3.4.5</td></tr>
</tbody>
</table>

---

## 2 原理

**AprilTag 不是在「比對圖片」，是在讀條碼。**

```
1. 找邊      影像轉灰階 → 找亮暗交界
2. 找四邊形   把邊界串成封閉的四邊形（候選）
3. 解碼      把內部拉正、切格子、量亮度 → 一串二進位
4. 查表      這串數字在 36h11 字典裡是第幾號 → tag ID
```

解碼時會把格子**依四個方向各讀一次**，哪個方向對上字典，就同時得到
「是幾號」和「轉了幾度」。

> **旋轉不是要克服的問題，旋轉就是答案的來源。**

`36h11` = 36 個資料位元、任兩個合法碼至少差 11 個 bit
（`/opt/ros/humble/include/apriltag/apriltag.h:78-79`）。
偵測器最多只肯修 2 個 bit，本套件設 `max_hamming: 0` ——
中間那 9~11 bit 是無人區，**寧可什麼都不回報，也不回報錯的 ID**。

### 位置與角度：PnP

有這三樣就能反推唯一的 6 自由度位姿：

<table width="100%">
<thead><tr>
<th width="50%" align="left">已知</th>
<th width="50%" align="left">從哪來</th>
</tr></thead>
<tbody>
<tr><td align="left">四個角在畫面上的像素座標</td><td align="left">偵測結果 <code>corners</code></td></tr>
<tr><td align="left">tag 在現實中邊長幾公尺</td><td align="left">參數 <code>tag_size</code></td></tr>
<tr><td align="left">相機焦距與光心</td><td align="left"><code>camera_info</code> topic</td></tr>
</tbody>
</table>

直覺：正對著看四角是矩形，斜著看變梯形。
**形變本身就是資訊** —— 越扁 = 越斜，越小 = 越遠。

### ⚠️ 位姿在 /tf，不在 detections

`apriltag_msgs/AprilTagDetection` **沒有 pose 欄位**，只有 ID、四個角、
hamming、decision_margin。位姿是用 `TransformBroadcaster` 發到 `/tf` 的。

所以本節點**兩邊都訂**：`detections` 過品質關卡，再拿它的時間戳去查 `/tf`。

### 座標轉換鏈

```
光學系 (x 右 / y 下 / z 沿光軸)
   ↓  固定旋轉（ROS 與 Gazebo 的慣例差異）
camera_down_link
   ↓  camera_offset / camera_rpy（參數，抄自機體 model.sdf）
機體 FLU (x 前 / y 左 / z 上)
   ↓  後兩軸反向
PX4 FRD (x 前 / y 右 / z 下)
   ↓  繞「下」軸轉 heading
NED
```

這條鏈由 [T2](#4-三層驗證) 在 Gazebo 用 7 個已知位置實測過，水平誤差 ≤ 0.4 cm。

---

## 3 狀態機

```
IDLE ──goal──> SEARCHING ──連續 5 幀鎖定──> ALIGNING
                   ▲                          │ 水平 < 0.15 m 且 yaw < 5°
                   │ 連續 15 幀丟失            ▼
                   └──────────────────── DESCENDING
                                              │ 高度 ≤ handoff_altitude
                                              ▼
                        回報結果 ◄── 落地 ◄── HANDOFF（送 NAV_LAND 給 PX4）
```

每個狀態轉換都會用 `RCLCPP_INFO` 印出當下的高度與水平誤差。

**SEARCHING 是「原地停住等」。**
上層導航應該已經把飛機帶到降落點上方，看不到多半是還沒停穩。
自作主張繞圈或升高比較危險 —— 逾時就回報失敗，**讓上層決定**要不要重試。

**DESCENDING 有防線。**
水平誤差擴大到容許值的 2.5 倍就退回 ALIGNING，不會歪著往下衝。

### ⚠️ HANDOFF：為什麼不自己降到底

下視相機 FOV 有限，tag 在某個高度以下會**整個出框**
（本套件的相機 + 0.8 m 的 tag，[T1](#4-三層驗證) 實測是 **0.5 m**）。

> 硬要用視覺降到地面，狀態機會在「下降」和「丟失」之間無限來回，
> 永遠落不了地。

所以降到 `handoff_altitude`（預設 1.0 m）就送 `VEHICLE_CMD_NAV_LAND`，
最後一段交給 PX4 —— 它有觸地偵測和自動 disarm。

換相機或換 tag 尺寸時要重算這個高度，**T1 會自動幫你檢查**。

---

## 4 三層驗證

<table width="100%">
<thead><tr>
<th width="10%" align="center"></th>
<th width="46%" align="left">內容</th>
<th width="14%" align="center">要 Gazebo</th>
<th width="12%" align="center">要飛</th>
<th width="18%" align="center">耗時</th>
</tr></thead>
<tbody>
<tr><td align="center"><b>T1</b></td><td align="left">參數一致性、貼圖解碼、可視高度</td><td align="center">❌</td><td align="center">❌</td><td align="center">0.5 秒</td></tr>
<tr><td align="center"><b>T2</b></td><td align="left"><b>座標系與位姿轉換</b></td><td align="center">✅</td><td align="center">❌</td><td align="center">90 秒</td></tr>
<tr><td align="center"><b>T3</b></td><td align="left">完整降落飛行</td><td align="center">✅</td><td align="center">✅</td><td align="center">3 分鐘</td></tr>
</tbody>
</table>

分層的意義在於**每一層抓到的東西不一樣**：

<table width="100%">
<thead><tr>
<th width="10%" align="center"></th>
<th width="90%" align="left">實際抓到過的問題</th>
</tr></thead>
<tbody>
<tr><td align="center"><b>T1</b></td><td align="left"><code>tag_size</code> 應該是 0.8 不是 1.0（黑框只佔 14 格裡的 8 格）</td></tr>
<tr><td align="center"><b>T2</b></td><td align="left">位置鏈路正確；<b>但它看不到 ENU/NED 的符號差</b>，因為不啟動 PX4</td></tr>
<tr><td align="center"><b>T3</b></td><td align="left">setpoint 灌爆 PX4、yaw 符號翻轉、HANDOFF 時模式打架</td></tr>
</tbody>
</table>

> **T2 過了不代表 T3 會過。** 上面那三個坑全部是 T3 才現形的。

### T2 怎麼做到「不用飛」

完全不啟動 PX4。只開 Gazebo server，用 `/world/<w>/create` 生一個
`<static>true</static>` 的**相機探針**放在指定位置 —— 靜態模型不會掉下去，
位置由我們指定，所以真值零誤差。

探針的相機 link 是**執行時從機體 `model.sdf` 抓出來**的，不是複製一份。
改了相機安裝角度，這支測試會跟著驗新的角度，不會悄悄過期。

而換算用的是 `landing.yaml` 的參數 —— **幾何來自 SDF、數學來自 yaml**，
兩邊不同步位置就會對不上，等於順便驗了「參數有沒有抄錯」。

### T3 怎麼判斷降得準不準

把飛機 spawn 在 tag 的**正上方**。PX4 的 local NED 原點就是 EKF 初始化的
位置，也就是 tag 的位置 —— 於是「最後的 NED 座標」直接就是「離 tag 多遠」，
不需要另外量真值。

流程：起飛 → 故意飛開一段 → 交接 → 看它把飛機帶回原點多準。

<table width="100%">
<thead><tr>
<th width="50%" align="center">起始偏移</th>
<th width="50%" align="center">落點誤差</th>
</tr></thead>
<tbody>
<tr><td align="center">1.84 m</td><td align="center"><b>1.8 cm</b></td></tr>
<tr><td align="center">2.21 m</td><td align="center"><b>3.8 cm</b></td></tr>
</tbody>
</table>

---

## 5 在自己電腦上驗證

**三支都會自己收工**，跑完不會留下 Gazebo 或 PX4 在背景。
全過回傳 `0`、任一項失敗回傳 `1`，可以直接串進 CI 或 pre-commit。

```bash
cd ~/ros2_ws
colcon build --packages-select drone_apriltag_landing
source install/setup.bash
```

### T1 — 0.5 秒，不開任何東西

```bash
python3 src/drone_apriltag_landing/test/t1_offline_decode.py
```

<details>
<summary>換相機規格或換貼圖</summary>

```bash
python3 src/drone_apriltag_landing/test/t1_offline_decode.py \
    --hfov 1.2 --width 640 --height 480

python3 src/drone_apriltag_landing/test/t1_offline_decode.py \
    --tag-image /path/to/tag.png --plane-size 0.5
```
</details>

### T2 — 約 90 秒，會自己開關 Gazebo

```bash
python3 src/drone_apriltag_landing/test/t2_pose_check.py
python3 src/drone_apriltag_landing/test/t2_pose_check.py --gui   # 看探針擺在哪
```

中間會沒反應約 20 秒（等 Gazebo 起來），是正常的。

### T3 — 約 3 分鐘，**會真的讓飛機飛起來**

```bash
# 跑之前確認沒有別的 SITL 在跑
pgrep -a -f "gz sim|px4" | grep -v pgrep     # 應該沒輸出

python3 src/drone_apriltag_landing/test/t3_landing_flight.py
python3 src/drone_apriltag_landing/test/t3_landing_flight.py --gui
python3 src/drone_apriltag_landing/test/t3_landing_flight.py --offset -2.0 1.0
```

> **T2 沒過之前不要跑 T3。** 方向錯的飛機看起來很像正常在動 ——
> 它確實在移動、確實在收斂，只是收斂到錯的地方。

---

## 6 接上拓樸地圖

沿 `drone_nav2_apriltag` 的拓樸圖飛到降落點，然後交給精準降落。

```bash
# 終端 A
MicroXRCEAgent udp4 -p 8888

# 終端 B
cd ~/ros2_ws
DRONES=1 ./src/drone_nav2_apriltag/scripts/start_arena_sitl.sh

# 終端 C
source install/setup.bash
ros2 launch drone_apriltag_landing nav2_then_land.launch.py
```

### 時序

```
route_server 規劃 → 依序飛節點 → 到終點上空
    │
    │  land_at_goal:=false → 不降落，fly_nodes 退出
    ▼
setpoint 停止 → PX4 掉進 HOLD（懸停）
    │
    │  OnProcessExit 觸發
    ▼
送 PrecisionLand goal → SEARCHING → ALIGNING → DESCENDING → HANDOFF → 落地
```

實測：`fly_nodes` 把飛機帶到離 tag **0.91 m**，精準降落收到 **9.8 cm**。

**對 `drone_nav2_apriltag` 的唯一要求**：它的 `fly_nodes.launch.py` 要有
`land_at_goal` 這個 launch 參數（預設 `true` = 原本行為不變）。

<details>
<summary>為什麼用 ExecuteProcess 而不是 IncludeLaunchDescription</summary>

`fly_nodes.launch.py` 內部綁了 `OnProcessExit(fly_nodes) → Shutdown`。
用 Include 的話那個 Shutdown 會把**整個 launch** 關掉，包含本套件的降落節點，
而且是在它還來不及動作之前。

包成子程序後，那個 Shutdown 只作用在子程序自己身上 ——
我們反而可以用它的結束事件當作「飛到了」的信號。
</details>

### 單獨啟動（不含導航）

```bash
ros2 launch drone_apriltag_landing precision_land.launch.py

ros2 action send_goal /MAV1/precision_land \
    drone_apriltag_landing/action/PrecisionLand "{}" --feedback
```

### 看畫面

模擬預設是無頭跑的（快、省 GPU）。想看畫面的話：

<table width="100%">
<thead><tr>
<th width="40%" align="left">想看什麼</th>
<th width="60%" align="left">怎麼開</th>
</tr></thead>
<tbody>
<tr><td align="left">飛行軌跡、降落過程（3D 場景）</td><td align="left">啟動 SITL 時<b>不要</b>加 <code>HEADLESS=1</code></td></tr>
<tr><td align="left">apriltag 實際看到的畫面</td><td align="left"><code>nav2_then_land.launch.py view:=true</code></td></tr>
<tr><td align="left">拓樸圖節點/邊 + 無人機位置</td><td align="left">下面的 RViz</td></tr>
</tbody>
</table>

#### RViz：看拓樸圖和無人機在圖上跑

另開一個終端（`drone_nav2_apriltag` 提供的，本套件不重造）：

```bash
ros2 launch drone_nav2_apriltag view_graph.launch.py \
    px4_namespace:=/MAV1 track_vehicle:=true
```

同一個視窗會有：**牆的輪廓 + 拓樸圖的節點與邊 + 兩顆相機的畫面 + 無人機當下位置**。

<table width="100%">
<thead><tr>
<th width="35%" align="left">參數</th>
<th width="65%" align="left">說明</th>
</tr></thead>
<tbody>
<tr><td align="left"><code>px4_namespace:=/MAV1</code></td><td align="left">要跟 SITL 的 namespace 一致，不填就看不到無人機</td></tr>
<tr><td align="left"><code>track_vehicle:=true</code></td><td align="left">畫出無人機當下位置</td></tr>
<tr><td align="left"><code>rviz:=false</code></td><td align="left">只發 MarkerArray 不開視窗</td></tr>
</tbody>
</table>

完整的四個終端：

```bash
# A  XRCE Agent
MicroXRCEAgent udp4 -p 8888

# B  SITL（拿掉 HEADLESS=1 就有 Gazebo 視窗）
cd ~/ros2_ws && DRONES=1 ./src/drone_nav2_apriltag/scripts/start_arena_sitl.sh

# C  RViz：拓樸圖 + 無人機
ros2 launch drone_nav2_apriltag view_graph.launch.py \
    px4_namespace:=/MAV1 track_vehicle:=true

# D  導航 + 精準降落
ros2 launch drone_apriltag_landing nav2_then_land.launch.py
```

> 全部開起來會吃不少 GPU。GPU 被吃掉會拖慢 Gazebo 的物理步進，
> lockstep 下 PX4 就收不到 IMU。出現 `Accel TIMEOUT` 就關掉幾個視窗。

---

## 7 參數

全部在 `config/landing.yaml`，**換環境只改這裡**。
⚠️ 標記的三個填錯不會報錯，只會安靜地失敗。

<table width="100%">
<thead><tr>
<th width="28%" align="left">參數</th>
<th width="22%" align="left">預設</th>
<th width="50%" align="left">說明</th>
</tr></thead>
<tbody>
<tr><td align="left"><code>namespace</code></td><td align="left"><code>MAV1</code></td><td align="left">PX4 namespace，多機時改這個</td></tr>
<tr><td align="left"><code>target_system</code></td><td align="left"><code>1</code></td><td align="left">MAVLink target_system，等於 instance+1</td></tr>
<tr><td align="left"><code>camera_frame</code></td><td align="left"><code>camera_down_link</code></td><td align="left">要和 <code>camera_info</code> 的 <code>frame_id</code> 一致</td></tr>
<tr><td align="left">⚠️ <code>tag_id</code></td><td align="left"><code>0</code></td><td align="left">填錯會<b>永遠偵測不到</b>且不報錯</td></tr>
<tr><td align="left">⚠️ <code>tag_size</code></td><td align="left"><code>0.8</code></td><td align="left"><b>黑框外緣</b>的邊長，不是整張貼圖</td></tr>
<tr><td align="left"><code>camera_offset</code></td><td align="left"><code>[0, 0, 0.10]</code></td><td align="left">相機在機體上的位置（x 前／y 左／z 上）</td></tr>
<tr><td align="left"><code>camera_rpy</code></td><td align="left"><code>[0, 1.5707, 0]</code></td><td align="left">安裝角度（弧度），抄自機體 <code>model.sdf</code></td></tr>
<tr><td align="left"><code>min_decision_margin</code></td><td align="left"><code>30.0</code></td><td align="left">黑白對比門檻，濾掉逆光那幾幀</td></tr>
<tr><td align="left"><code>lock_frames</code></td><td align="left"><code>5</code></td><td align="left">連續幾幀才算鎖定</td></tr>
<tr><td align="left"><code>lost_frames</code></td><td align="left"><code>15</code></td><td align="left">連續幾幀才算丟失</td></tr>
<tr><td align="left"><code>xy_tolerance</code></td><td align="left"><code>0.15</code></td><td align="left">水平誤差小於這個才准開始下降（m）</td></tr>
<tr><td align="left"><code>yaw_tolerance_deg</code></td><td align="left"><code>5.0</code></td><td align="left">航向誤差門檻（度）</td></tr>
<tr><td align="left"><code>kp_xy</code> <code>kp_z</code> <code>kp_yaw</code></td><td align="left"><code>0.5</code> <code>0.6</code> <code>0.8</code></td><td align="left">控制增益，不要設 1.0（會過衝震盪）</td></tr>
<tr><td align="left"><code>max_speed_xy</code></td><td align="left"><code>0.8</code></td><td align="left">水平速度上限（m/s）</td></tr>
<tr><td align="left"><code>descend_speed</code></td><td align="left"><code>0.3</code></td><td align="left">下降速度（m/s）</td></tr>
<tr><td align="left">⚠️ <code>handoff_altitude</code></td><td align="left"><code>1.0</code></td><td align="left">交接給 PX4 的高度，換相機要重算</td></tr>
<tr><td align="left"><code>search_timeout_s</code></td><td align="left"><code>30.0</code></td><td align="left">找不到 tag 多久放棄</td></tr>
<tr><td align="left"><code>total_timeout_s</code></td><td align="left"><code>120.0</code></td><td align="left">整體逾時</td></tr>
<tr><td align="left"><code>control_mode</code></td><td align="left"><code>controller</code></td><td align="left">或 <code>advisor</code>（只回報誤差，不碰飛機）</td></tr>
</tbody>
</table>

`config/tag_36h11.yaml` 是給現成的 `apriltag_node` 用的。
**自己帶一份而不是改 `/opt/ros/humble/...` 的系統檔** —— 改系統檔要 sudo、
下次 `apt upgrade` 會被蓋掉，而且會影響這台機器上所有專案。

> `landing.yaml` 的 `tag_size` 和 `tag_36h11.yaml` 的 `size` 必須一致，T1 會檢查。

---

## 8 檔案結構

```
drone_apriltag_landing/
├── action/
│   └── PrecisionLand.action        對外唯一的契約
├── src/
│   └── precision_land_node.cpp     唯一的節點（apriltag_node 是現成的）
├── config/
│   ├── landing.yaml                降落節點的參數  ← 換環境改這裡
│   └── tag_36h11.yaml              給 apriltag_node 的參數
├── launch/
│   ├── precision_land.launch.py    只起偵測器 + 降落節點
│   └── nav2_then_land.launch.py    接上拓樸地圖，飛完整趟
├── test/
│   ├── t1_offline_decode.py        不用 Gazebo
│   ├── t2_pose_check.py            要 Gazebo，不飛
│   └── t3_landing_flight.py        要飛
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## 9 踩過的雷

**共通點：全部都不會報錯。** 症狀和原因看起來毫無關聯，這才是難查的地方。

### `tag_size` 指的是黑框外緣，不是整張貼圖

用 `apriltag_node` 實測過：它回報的四個角落在**黑框外緣**上。
標準 tag36h11 圖檔是 10×10 格，黑框只佔中間 8 格
（1 格黑框 + 6×6 資料 + 1 格黑框，`6×6=36` 就是名字裡的「36」）。

`drone_nav2_apriltag` 那張貼圖是 14×14 格、地磚 1.4 m，
所以黑框外緣是 `1.4 × 8/14 = 0.8 m`，**不是 1.0 m**。

- **症狀**：ID 照樣讀得到，只有距離會等比縮放 25%
- **防線**：T1 會從貼圖反推正確值並比對設定檔

### tag ID 和拓樸圖節點編號是無關的兩套系統

貼圖實測是 **tag ID 0**，而它擺在拓樸圖的 **node 10**。

- **症狀**：`tag_id` 填 10 會永遠偵測不到，完全不報錯

### setpoint 沒限速會灌爆 PX4

`rclpy.spin_once()` 在有訊息可處理時幾乎立刻返回。在迴圈裡直接發 setpoint
等於用**數千 Hz** 在灌，PX4 端的佇列會塞爆。

- **症狀**：解鎖了、也進 offboard 了，就是不爬升
- **修法**：PX4 只要求 > 2 Hz，20 Hz 已經十倍餘裕

### ⚠️ ENU yaw 和 NED heading 旋轉方向相反

T2 量的是探針的 **ENU yaw**（從東起算、逆時針為正），
節點用的是 PX4 的 **NED heading**（從北起算、順時針為正）。
同一個實體轉動，**兩者符號相反**。

- **症狀**：水平位置收斂得很漂亮（1.85 m → 0.02 m），
  但 yaw 穩定在差 180 度的地方，永遠進不了 DESCENDING
- **為什麼難查**：看起來完全像「對不準」而不像「符號錯」
- **防線**：T2 的輸出現在會直接印出換算後該用的符號

### HANDOFF 之後要連模式指令一起停

只停 `trajectory_setpoint` 是不夠的。`ensureOffboard()` 會每 0.5 秒把飛機
拉回 offboard，跟 PX4 的 LAND 模式互相拉扯。

- **症狀**：飛機確實降下去了，卻回報「30 秒未偵測到落地」
- **為什麼難查**：第一眼會以為是收不到 `vehicle_land_detected`

### `pkill -f "gz sim"` 會誤殺

它會比對到「命令列裡剛好出現這幾個字」的無關程序，
**包括呼叫腳本的那個 shell 自己**。

- **修法**：測試腳本一律用 process group + 精準特徵比對

### 實機的相機有畸變

`apriltag_node` 訂的是 `image_rect`。Gazebo 相機沒有鏡頭畸變，
`image_raw` 直接當 `image_rect` 用沒問題。

- **實機**：中間必須插一個 `image_proc` 的 rectify，
  否則畫面邊緣的位姿會有系統性偏差
