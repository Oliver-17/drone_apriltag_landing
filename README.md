# drone_apriltag_landing

以 AprilTag 做視覺精準降落的 PX4 套件。**不綁定任何場地或機體** —— 相機 topic、
tag ID、tag 尺寸、控制增益全部是參數,換環境只改 `config/landing.yaml`。

---

## ⚠️ 交接協議（接進你的系統前必讀）

> **呼叫本 action 之前,呼叫方必須停止發布 `trajectory_setpoint` 與
> `offboard_control_mode`。** 本節點在 goal 被接受後接管控制權,直到回報 result
> 才交還。
>
> 兩邊同時發的後果:PX4 會收到交錯的矛盾指令,飛機抽搐。這個現象在 log 裡看起來
> 很像控制器沒調好,幾乎不可能聯想到是兩個節點在搶,所以務必先確認這一點。

不想交出控制權的話,把 `control_mode` 設成 `advisor`:節點只把誤差發到
`~/landing_error`,完全不碰飛機,退化成一個純視覺定位感測器。

---

## 這包做什麼 / 不做什麼

| ✅ 做 | ❌ 不做 |
|---|---|
| 認出地上的 tag,算出相對位置與航向 | 飛到 tag 附近（那是 Nav2 / 編隊的事） |
| 對準 → 下降 → 交接給 PX4 落地 | 定義場地、提供 tag 貼圖 |
| 找不到就等,逾時回報失敗 | 起飛、跑航線 |
| | 開相機（模擬靠 ros_gz_image,實機靠 camera_ros） |

---

## 用法

```bash
# 1. 先確保相機影像和 camera_info 已經在發
#    模擬: ros2 launch drone_nav2_apriltag cameras.launch.py
# 2. 起偵測器 + 降落節點
ros2 launch drone_apriltag_landing precision_land.launch.py namespace:=MAV1

# 3. 觸發降落
ros2 action send_goal /MAV1/precision_land \
    drone_apriltag_landing/action/PrecisionLand "{}" --feedback
```

---

## 狀態機

```
IDLE ──goal──> SEARCHING ──鎖定──> ALIGNING ──誤差夠小──> DESCENDING
                   ▲                                          │
                   └────────── 丟失 tag ──────────────────────┘
                                                              │ 高度 ≤ handoff_altitude
                                                              ▼
                                     回報結果 <── 落地 <── HANDOFF（送 NAV_LAND 給 PX4）
```

`SEARCHING` 的行為是**原地停住等**(不繞圈、不升高)。理由:上層導航應該已經把
飛機帶到降落點上方,看不到多半是還沒停穩,等幾秒就好。逾時就回報失敗讓上層決定
要不要重試 —— 自作主張到處亂飛比較危險。

---

## ⚠️ 兩個一定要看的設計限制

### 1. 低於某個高度就看不見 tag，這是幾何必然

下視相機 FOV `1.74 rad`、tag 邊長 `0.8 m` 的組合下,**實測低於 0.5 m 就偵測不到**。所以降到 `handoff_altitude`(預設 1.0 m)就停止用視覺,改送
`VEHICLE_CMD_NAV_LAND`(=21)交給 PX4,由它的觸地偵測完成最後一段。

**換相機或換 tag 尺寸時要重算這個數字。**

### 2. tag_size 和 tag_id 填錯是「靜默失敗」

* `tag_id` 錯 → 永遠偵測不到,不會報錯
* `tag_size` 錯 → ID 照樣讀得到,但距離整個縮放,飛機會以為自己在別的高度

`apriltag_ros` 內建預設是 `0.173`。`drone_nav2_apriltag` 那張貼圖的正確值是
**0.8 m** —— `tag_size` 指的是**黑框外緣**,而黑框只佔貼圖的 8/14
(地磚 1.4 m x 8/14 = 0.8 m)。已用 `apriltag_node` 實測確認它回報的四個角
就落在黑框外緣上。

實測解出來的 ID 是 **0**(不是拓樸圖的 node 10,那是無關的編號系統)。

跑 `test/t1_offline_decode.py` 會自動從貼圖反推 tag_size 並比對設定檔,
這兩個坑都會被擋下來。

---

## 開發進度

| | 內容 | 狀態 |
|---|---|---|
| 骨架 | package / action / config / launch / 節點介面 | ✅ 編譯通過、action 已掛上 |
| T1 | 離線偵測驗證（不用 Gazebo） | ✅ 5 項全過，0.5 秒 |
| T2 | **座標系驗證**（要 Gazebo,不飛） | ✅ 7 個位置 + 航向全過 |
| 節點本體 | 狀態機、控制迴路 | ✅ 編譯過、拒絕條件已驗 |
| T3 | 完整降落飛行 | ✅ 兩次飛行，落點誤差 1.8 / 3.8 cm |

### 三個實測抓到的坑（都不會報錯）

| 現象 | 真正的原因 |
|---|---|
| 解鎖了、進 offboard 了，就是不爬升 | setpoint 沒限速，用數千 Hz 灌爆 PX4 的佇列 |
| 水平收斂得很漂亮，yaw 卡在 180 度 | T2 量的是 ENU yaw，PX4 用 NED heading，**兩者旋轉方向相反**，符號要翻 |
| 確實降下去了，但回報「30 秒未偵測到落地」 | HANDOFF 後只停了 setpoint，`ensureOffboard()` 還在把飛機拉回 offboard，和 LAND 模式打架 |

> **T2 沒過之前不要飛。** tag 位姿在相機光學座標系(x 右 / y 下 / z 沿光軸),
> 要變成 PX4 的 NED(x 北 / y 東 / z 下)中間串三個轉換,下視相機在 model.sdf
> 裡還有 `pitch +1.5707`。搞錯一個符號飛機會往反方向飛,而且看起來很像正常在動。
