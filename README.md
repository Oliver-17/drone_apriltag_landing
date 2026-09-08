# drone_apriltag_landing — AprilTag 視覺精準降落

ROS 2 Humble + PX4 的**精準降落套件**。
把無人機從「大概在降落點上方」帶到「準確停在 AprilTag 上」，
並提供三層可以獨立執行的驗證工具（離線參數檢查 / 座標轉換驗證 / 實飛）。

> 這個 repo **本身就是一個 ROS 2 package**，不是 workspace。
> clone 進你既有 workspace 的 `src/` 底下即可。

**不綁定任何場地或機體** —— 相機 topic、tag ID、tag 尺寸、安裝角度、
控制增益全部是參數，換環境只改 `config/landing.yaml`。

---

## 快速導覽

| 你想知道 | 看哪一節 |
|---|---|
| **接進我的系統前必讀** | [0 交接協議](#0-交接協議) |
| 這包在做什麼、跟誰的界線在哪 | [1 這包在做什麼](#1-這包在做什麼) |
| AprilTag 到底怎麼算出位置和角度 | [2 原理](#2-原理) |
| 它怎麼決定何時下降、何時交給 PX4 | [3 狀態機](#3-狀態機) |
| 怎麼證明這包是對的 | [4 三層驗證 T1 T2 T3](#4-三層驗證-t1-t2-t3) |
| **我要在自己電腦上跑一次** | [5 在自己電腦上驗證](#5-在自己電腦上驗證) |
| **接上拓樸地圖，飛完整趟** | [6 接上 drone_nav2_apriltag](#6-接上-drone_nav2_apriltag) |
| 飛行途中要看到相機畫面 | [看畫面](#看畫面) |
| 有哪些參數、換相機要改什麼 | [7 參數](#7-參數) |
| 檔案在哪 | [8 檔案結構](#8-檔案結構) |
| 有什麼坑 | [9 踩過的雷](#9-踩過的雷) |

---

## 0 交接協議

> **呼叫本 action 之前，呼叫方必須停止發布 `trajectory_setpoint` 與
> `offboard_control_mode`。** 本節點在 goal 被接受後接管控制權，
> 直到回報 result 才交還。

兩邊同時發的後果：PX4 收到交錯的矛盾指令，飛機抽搐。這個現象在 log 裡
看起來很像控制器沒調好，幾乎不可能聯想到是兩個節點在搶 —— 所以節點會
主動檢查 `trajectory_setpoint` 上還有沒有別的發布者，有的話**直接拒絕 goal**。

停止發布之後 PX4 會在約 0.5 秒內掉出 offboard 進入 HOLD（懸停）。
這個空窗期是正常的，降落節點會自己把模式切回來。

不想交出控制權的話，把 `control_mode` 設成 `advisor`：節點只把誤差發到
`~/landing_error`，完全不碰飛機，退化成一個純視覺定位感測器。

---

## 1 這包在做什麼

| ✅ 做 | ❌ 不做 |
|---|---|
| 認出地上的 tag，算出相對位置與航向 | 飛到 tag 附近（那是 Nav2 / 編隊的事） |
| 對準 → 下降 → 交接給 PX4 落地 | 定義場地、提供 tag 貼圖 |
| 找不到就等，逾時回報失敗 | 起飛、跑航線 |
| | 開相機（模擬靠 `ros_gz_image`，實機靠 `camera_ros`） |

相依只有 `rclcpp` / `rclcpp_action` / `px4_msgs` / `apriltag_msgs` / `tf2`。
**不相依 `drone_nav2_apriltag`** —— 別人拿去不用連帶搬走整個場地。

### 對外介面

```
action   <ns>/precision_land        drone_apriltag_landing/action/PrecisionLand
topic    <ns>/landing_error         geometry_msgs/Vector3Stamped （北誤差, 東誤差, yaw 誤差）
```

### 環境版本（已驗證可用的組合）

| 項目 | 版本 |
|---|---|
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| PX4-Autopilot（SITL） | v1.17.0 |
| Gazebo | Harmonic 8.15.0 |
| `apriltag_ros` | 3.4.0 |
| `apriltag`（C 演算法） | 3.4.5 |

---

## 2 原理

### AprilTag 不是在「比對圖片」，是在讀條碼

```
1. 找邊     影像轉灰階 → 找亮暗交界
2. 找四邊形  把邊界串成封閉的四邊形（候選）
3. 解碼     把四邊形內部拉正，切成格子，量亮度 → 一串二進位
4. 查表     這串數字在 36h11 字典裡是第幾號 → tag ID
```

解碼時會把格子**依四個方向各讀一次**，哪個方向對上字典就同時得到
「是幾號」和「轉了幾度」。**旋轉不是要克服的問題，旋轉就是答案的來源。**

`36h11` 的意思是 36 個資料位元、任兩個合法碼至少差 11 個 bit
（見 `/opt/ros/humble/include/apriltag/apriltag.h:78-79`）。
偵測器最多只肯修 2 個 bit，而本套件設 `max_hamming: 0` ——
中間那 9~11 bit 是無人區，**寧可什麼都不回報，也不回報錯的 ID**。

### 位置與角度：PnP

有三樣東西就能反推唯一的 6 自由度位姿：

| 已知 | 從哪來 |
|---|---|
| 四個角在畫面上的像素座標 | 偵測結果 `corners` |
| tag 在現實中邊長幾公尺 | 參數 `tag_size` |
| 相機焦距與光心 | `camera_info` topic |

直覺：正對著看四角是矩形，斜著看變梯形。**形變本身就是資訊** ——
越扁 = 越斜，越小 = 越遠。

### ⚠️ 位姿在 /tf，不在 detections

`apriltag_msgs/AprilTagDetection` **沒有 pose 欄位**，只有 ID、四個角、
hamming、decision_margin。位姿是用 `tf2_ros::TransformBroadcaster` 發到 `/tf` 的。

所以本節點**兩邊都訂**：`detections` 過品質關卡，再拿它的時間戳去查 `/tf`。

### 座標轉換鏈

```
光學系 (x 右 / y 下 / z 沿光軸)
   ↓ 固定旋轉（ROS 與 Gazebo 的慣例差異）
camera_down_link
   ↓ camera_offset / camera_rpy（參數，抄自機體 model.sdf）
機體 FLU (x 前 / y 左 / z 上)
   ↓ 後兩軸反向
PX4 FRD (x 前 / y 右 / z 下)
   ↓ 繞「下」軸轉 heading
NED
```

這條鏈由 [T2](#4-三層驗證-t1-t2-t3) 在 Gazebo 裡用 7 個已知位置實測驗證過，
水平誤差 ≤ 0.4 cm。

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

### SEARCHING 為什麼是「原地停住等」

上層導航應該已經把飛機帶到降落點上方，看不到多半是還沒停穩。
自作主張繞圈或升高比較危險，逾時就回報失敗**讓上層決定**要不要重試。

### DESCENDING 的防線

水平誤差擴大到容許值的 2.5 倍就退回 ALIGNING，不會歪著往下衝。

### ⚠️ HANDOFF：為什麼不自己降到底

下視相機 FOV 有限，tag 在某個高度以下會整個出框
（本套件的相機 + 0.8 m 的 tag，[T1](#4-三層驗證-t1-t2-t3) 實測是 **0.5 m**）。
硬要用視覺降到地面，狀態機會在「下降」和「丟失」之間無限來回，永遠落不了地。

所以降到 `handoff_altitude`（預設 1.0 m）就送 `VEHICLE_CMD_NAV_LAND`，
最後一段交給 PX4 —— 它有觸地偵測和自動 disarm。

**換相機或換 tag 尺寸時要重算這個高度。** T1 會自動幫你檢查。

---

## 4 三層驗證 T1 T2 T3

| | 內容 | 要 Gazebo | 要飛 | 耗時 |
|---|---|---|---|---|
| **T1** | 參數一致性、貼圖解碼、可視高度 | ❌ | ❌ | 0.5 秒 |
| **T2** | **座標系與位姿轉換** | ✅ | ❌ | 90 秒 |
| **T3** | 完整降落飛行 | ✅ | ✅ | 3 分鐘 |

分層的意義在於**每一層抓到的東西不一樣**：

| | 實際抓到過的問題 |
|---|---|
| T1 | `tag_size` 應該是 0.8 不是 1.0（貼圖黑框只佔 14 格裡的 8 格） |
| T2 | 位置轉換鏈正確；**但它看不到 ENU/NED 的符號差**，因為它不啟動 PX4 |
| T3 | setpoint 沒限速灌爆 PX4、yaw 符號翻轉、HANDOFF 時模式打架 |

**T2 過了不代表 T3 會過。** 上面那三個坑全部是 T3 才現形的。

### T2 怎麼做到「不用飛」

完全不啟動 PX4。只開 Gazebo server，用 `/world/<w>/create` 生一個
`<static>true</static>` 的**相機探針**放在指定位置 —— 靜態模型不會掉下去，
位置由我們指定所以真值是已知的。

探針的相機 link 是**執行時從機體的 `model.sdf` 抓出來**的，不是複製一份。
改了相機安裝角度，這支測試會跟著驗新的角度，不會悄悄過期。

而換算用的是 `landing.yaml` 裡的參數 —— **幾何來自 SDF、數學來自 yaml**，
兩邊不同步位置就會對不上，等於順便驗了「參數有沒有抄錯」。

### T3 怎麼判斷降得準不準

把飛機 spawn 在 tag 的**正上方**。PX4 的 local NED 原點就是 EKF 初始化的
位置，也就是 tag 的位置 —— 於是「最後的 NED 座標」直接就是「離 tag 多遠」，
不需要另外量真值。

流程：起飛 → 故意飛開一段 → 交接 → 看它把飛機帶回原點多準。

實測結果：

| 起始偏移 | 落點誤差 |
|---|---|
| 1.84 m | **1.8 cm** |
| 2.21 m | **3.8 cm** |

---

## 5 在自己電腦上驗證

三支都會自己收工，跑完不會留下 Gazebo 或 PX4 在背景。

```bash
cd ~/ros2_ws
colcon build --packages-select drone_apriltag_landing
source install/setup.bash
```

### T1 — 0.5 秒，不開任何東西

```bash
python3 src/drone_apriltag_landing/test/t1_offline_decode.py
```

換相機規格或換貼圖：

```bash
python3 src/drone_apriltag_landing/test/t1_offline_decode.py \
    --hfov 1.2 --width 640 --height 480
python3 src/drone_apriltag_landing/test/t1_offline_decode.py \
    --tag-image /path/to/tag.png --plane-size 0.5
```

### T2 — 約 90 秒，會自己開關 Gazebo

```bash
python3 src/drone_apriltag_landing/test/t2_pose_check.py
python3 src/drone_apriltag_landing/test/t2_pose_check.py --gui   # 看探針擺在哪
```

中間會沒反應約 20 秒（等 Gazebo 起來），是正常的。

### T3 — 約 3 分鐘，**會真的讓飛機飛起來**

```bash
python3 src/drone_apriltag_landing/test/t3_landing_flight.py
python3 src/drone_apriltag_landing/test/t3_landing_flight.py --gui
python3 src/drone_apriltag_landing/test/t3_landing_flight.py --offset -2.0 1.0
```

跑之前確認沒有別的 SITL 在跑：

```bash
pgrep -a -f "gz sim|px4" | grep -v pgrep     # 應該沒輸出
```

三支都是全過回傳 `0`、任一項失敗回傳 `1`，可以串進 CI 或 pre-commit。

> **T2 沒過之前不要跑 T3。** 方向錯的飛機看起來很像正常在動 ——
> 它確實在移動、確實在收斂，只是收斂到錯的地方。

---

## 6 接上 drone_nav2_apriltag

沿拓樸圖飛到降落點，然後交給精準降落。**一行指令跑完整趟。**

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

實測：`fly_nodes` 把飛機帶到離 tag 0.91 m，精準降落收到 **9.8 cm**。

### 對 drone_nav2_apriltag 的唯一要求

它的 `fly_nodes.launch.py` 要有 `land_at_goal` 這個 launch 參數
（預設 `true` = 原本行為不變）。

### 為什麼用 ExecuteProcess 而不是 IncludeLaunchDescription

`fly_nodes.launch.py` 內部綁了 `OnProcessExit(fly_nodes) → Shutdown`。
用 Include 的話那個 Shutdown 會把**整個 launch** 關掉，包含本套件的降落節點，
而且是在它還來不及動作之前。包成子程序後，那個 Shutdown 只作用在子程序自己
身上，我們反而可以用它的結束事件當作「飛到了」的信號。

### 單獨啟動（不含導航）

```bash
ros2 launch drone_apriltag_landing precision_land.launch.py

# 手動觸發
ros2 action send_goal /MAV1/precision_land \
    drone_apriltag_landing/action/PrecisionLand "{}" --feedback
```

### 看畫面

| 想看什麼 | 怎麼開 |
|---|---|
| 飛行軌跡、降落過程 | 啟動 SITL 時**不要**加 `HEADLESS=1` |
| apriltag 實際看到的畫面 | `nav2_then_land.launch.py view:=true` |
| 拓樸圖 + 兩顆相機（一個視窗） | `ros2 launch drone_nav2_apriltag view_graph.launch.py` |

> 全部開起來會吃不少 GPU。GPU 被吃掉會拖慢 Gazebo 的物理步進，
> lockstep 下 PX4 就收不到 IMU。出現 `Accel TIMEOUT` 就關掉幾個視窗。

---

## 7 參數

全部在 `config/landing.yaml`，換環境只改這裡。

| 參數 | 預設 | 說明 |
|---|---|---|
| `namespace` | `MAV1` | PX4 namespace，多機時改這個 |
| `target_system` | `1` | MAVLink target_system，等於 instance+1 |
| `camera_frame` | `camera_down_link` | 要和 `camera_info` 的 `frame_id` 一致 |
| `tag_id` | `0` | ⚠️ 填錯會**永遠偵測不到**且不報錯 |
| `tag_size` | `0.8` | ⚠️ **黑框外緣**的邊長，不是整張貼圖 |
| `camera_offset` | `[0, 0, 0.10]` | 相機在機體上的位置（x 前 / y 左 / z 上） |
| `camera_rpy` | `[0, 1.5707, 0]` | 安裝角度（弧度），抄自機體 `model.sdf` |
| `min_decision_margin` | `30.0` | 黑白對比門檻，濾掉逆光那幾幀 |
| `lock_frames` / `lost_frames` | `5` / `15` | 連續幾幀才算鎖定 / 丟失 |
| `xy_tolerance` | `0.15` | 水平誤差小於這個才准開始下降（m） |
| `yaw_tolerance_deg` | `5.0` | 航向誤差門檻（度） |
| `kp_xy` / `kp_z` / `kp_yaw` | `0.5` / `0.6` / `0.8` | 控制增益，不要設 1.0（會過衝震盪） |
| `max_speed_xy` | `0.8` | 水平速度上限（m/s） |
| `descend_speed` | `0.3` | 下降速度（m/s） |
| `handoff_altitude` | `1.0` | ⚠️ 交接給 PX4 的高度，換相機要重算 |
| `search_timeout_s` | `30.0` | 找不到 tag 多久放棄 |
| `total_timeout_s` | `120.0` | 整體逾時 |
| `control_mode` | `controller` | 或 `advisor`（只回報誤差，不碰飛機） |

`config/tag_36h11.yaml` 是給現成的 `apriltag_node` 用的。
**自己帶一份而不是改 `/opt/ros/humble/...` 的系統檔** —— 改系統檔要 sudo、
下次 `apt upgrade` 會被蓋掉，而且會影響這台機器上所有專案。

> `tag_size` 和 `size` 兩邊必須一致，T1 會檢查。

---

## 8 檔案結構

```
drone_apriltag_landing/
├── action/
│   └── PrecisionLand.action        對外唯一的契約
├── src/
│   └── precision_land_node.cpp     唯一的節點（apriltag_node 是現成的）
├── config/
│   ├── landing.yaml                降落節點的參數 ← 換環境改這裡
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

### `tag_size` 指的是黑框外緣，不是整張貼圖

用 `apriltag_node` 實測過：它回報的四個角落在**黑框外緣**上。
而標準的 tag36h11 圖檔是 10×10 格，其中黑框只佔中間 8 格
（1 格黑框 + 6×6 資料 + 1 格黑框，6×6=36 就是名字裡的「36」）。

`drone_nav2_apriltag` 那張貼圖是 14×14 格、地磚 1.4 m，
所以黑框外緣是 `1.4 × 8/14 = 0.8 m`，**不是 1.0 m**。

填錯不會報錯，ID 照樣讀得到，只有距離會等比縮放 25%。
T1 會從貼圖反推正確值並比對設定檔。

### tag ID 和拓樸圖節點編號是無關的兩套系統

貼圖實測是 **tag ID 0**，而它擺在拓樸圖的 **node 10**。
`tag_id` 要填 `0`，填 10 會永遠偵測不到而且完全不報錯。

### setpoint 沒限速會灌爆 PX4

`rclpy.spin_once()` 在有訊息可處理時幾乎立刻返回。在迴圈裡直接發 setpoint
等於用**數千 Hz** 在灌，PX4 端的佇列會塞爆 —— 症狀是「解鎖了、也進 offboard 了，
就是不爬升」，而且沒有任何錯誤訊息。PX4 只要求 > 2 Hz，20 Hz 已經十倍餘裕。

### ⚠️ ENU yaw 和 NED heading 旋轉方向相反

T2 量的是探針的 **ENU yaw**（從東起算、逆時針為正），
而節點用的是 PX4 的 **NED heading**（從北起算、順時針為正）。
同一個實體轉動，**兩者符號相反**。

漏掉這個轉換的話：水平位置收斂得很漂亮（1.85 m → 0.02 m），
但 yaw 穩定在差 180 度的地方，永遠進不了 DESCENDING。
看起來完全像「對不準」而不像「符號錯」。

T2 的輸出現在會直接印出換算後該用的符號。

### HANDOFF 之後要連模式指令一起停

只停 `trajectory_setpoint` 是不夠的。`ensureOffboard()` 會每 0.5 秒把飛機
拉回 offboard，跟 PX4 的 LAND 模式互相拉扯 —— 飛機看起來確實降下去了，
但落地偵測永遠穩不下來，最後以「30 秒未偵測到落地」逾時收場。
第一眼會以為是收不到 `vehicle_land_detected`。

### `pkill -f "gz sim"` 會誤殺

它會比對到「命令列裡剛好出現這幾個字」的無關程序，
包括呼叫腳本的那個 shell 自己。測試腳本一律用 process group + 精準特徵。

### 實機的相機有畸變

`apriltag_node` 訂的是 `image_rect`。Gazebo 相機沒有鏡頭畸變，
`image_raw` 直接當 `image_rect` 用沒問題；**實機接真相機時中間必須插一個
`image_proc` 的 rectify**，否則畫面邊緣的位姿會有系統性偏差。
