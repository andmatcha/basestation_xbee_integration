# Communication Data Formats (Current Only)

このドキュメントは `arm9_ik` の **現行運用で実際に使っている**通信（Mac host / Ubuntu guest / miniPC / STM32 / frontend）だけを、
ポートとデータ形式（binary/JSON）まで含めてまとめたものです。

前提:

| Item | Detail |
|---|---|
| Host/guest 経路 | **現行デフォルト運用では** Ubuntu(Parallels guest) は miniPC/STM32 と直接通信せず、Mac(host) が中継する。miniPC command は設定により ROS 直結へ切替可能だが、本書は current default を記載する。 |
| Endian | binary はすべて **Little Endian**（Python `struct` 表記で記載）。 |
| UDP 単位 | UDP は 1 datagram = 1 packet（分割しない前提）。 |

Source of truth（実装）:

| Scope | Path |
|---|---|
| Mac WS/telemetry/video | `tools/dashboard_bridge.py` |
| Mac manual-only（optional, ROSなし） | `tools/gamepad_mode_sound_only_mac.py` |
| Mac miniPC relay | `tools/mac_minipc_udp_bridge.py` |
| Ubuntu uplink RX | `src/arm_ik_control/arm_ik_control/udp_joy_bridge.py` |
| Ubuntu unified AC TX | `src/arm_ik_control/arm_ik_control/udp_ac_tx.py` |
| Ubuntu JF RX | `src/arm_ik_control/arm_ik_control/udp_joint_state_rx.py` |
| Ubuntu JPEG RX | `src/arm_ik_control/arm_ik_control/udp_realsense_jpeg_rx.py` |
| Ubuntu keyboard detection RX | `src/arm_ik_control/arm_ik_control/udp_keyboard_detection_rx.py` |
| Ubuntu UI UDP RX | `src/arm_ik_control/arm_ik_control/udp_*_rx.py` |
| miniPC detection JSON schema | `minipc_ws/src/arm9_minipc/arm9_minipc/common.py` |
| miniPC command + ACK | `minipc_ws/src/arm9_minipc/arm9_minipc/cmd_protocol.py`, `minipc_ws/src/arm9_minipc/arm9_minipc/minipc_stack.py` |

## 1. Runtime Topology（current default: Mac relay）

```
miniPC  -- UDP6001(JSON detection) -->  Mac  -- UDP6001(JSON relay) --> Ubuntu(ROS2)
Ubuntu -- UDP6002(JSON cmd)        -->  Mac  -- UDP6002(JSON relay) --> miniPC

Ubuntu -- UDP7000(AC v6 binary) --> Mac -- Serial/XBee --> STM32
STM32  -- Serial/XBee --> Mac -- UDP5010(JF binary) --> Ubuntu

miniPC -- UDP4101(JPEG bytes) --> Mac(dashboard_bridge) -- WS4100 --> browser(frontend)
Mac(dashboard_bridge) -- UDP6201(JPEG bytes forward) --> Ubuntu(udp_realsense_jpeg_rx)
Ubuntu/Mac -- UDP4102(AC/JF/AI/MC raw bytes or kb_status/joint_state_pose/relay JSON) --> Mac(dashboard_bridge) -- WS4100 --> browser(frontend)
```

補足（例外）:
- 実機の簡易確認用に、**Ubuntu/ROS無し**で `tools/gamepad_mode_sound_only_mac.py` が `AC v6` を **XBee(TX)シリアルへ直接送信**する経路もあります。

## 2. Port / Transport Matrix（canonical defaults）

| Port | Transport | Direction | Sender -> Receiver | Payload |
|---:|---|---|---|---|
| 5005 | UDP | Mac -> Ubuntu | dashboard_bridge uplink（UI） -> `udp_joy_bridge` | `AI` / `MC` (binary) |
| 5010 | UDP | Mac -> Ubuntu | `dashboard_bridge` -> `udp_joint_state_rx` | `JF` (binary) |
| 6001 | UDP | miniPC -> Mac -> Ubuntu | miniPC -> `mac_minipc_udp_bridge` -> `udp_keyboard_detection_rx` | detection JSON + standalone `cmd_ack` |
| 6002 | UDP | Ubuntu -> Mac -> miniPC | `keyboard_auto_controller` -> `mac_minipc_udp_bridge` -> miniPC | command JSON |
| 7000 | UDP | Ubuntu -> Mac | `udp_ac_tx` -> `dashboard_bridge` | `AC v6` (binary) |
| 4100 | WS | browser <-> Mac | frontend <-> `dashboard_bridge` | WS JSON API |
| 4101 | UDP | miniPC -> Mac | miniPC -> `dashboard_bridge` | JPEG bytes |
| 6201 | UDP | Mac -> Ubuntu | `dashboard_bridge` -> `udp_realsense_jpeg_rx` | JPEG bytes（forward） |
| 4102 | UDP | Ubuntu/Mac -> Mac | `udp_keyboard_auto_status_tx` / `udp_joint_state_pose_tx` / `dashboard_bridge` tee / `mac_minipc_udp_bridge` -> `dashboard_bridge` | raw bytes (AC/JF/AI/MC) or `kb_status` / `joint_state_pose` / relay/runtime JSON |
| 6100 | UDP | Mac -> Ubuntu | `dashboard_bridge` -> `udp_keyboard_auto_request_rx` | keyboard-auto request JSON |
| 6101 | UDP | Mac -> Ubuntu | `dashboard_bridge` -> `udp_control_byte_override_rx` | AC control-byte override JSON |
| 6102 | UDP | Mac -> Ubuntu | `dashboard_bridge` -> `udp_enable_override_rx` | enable override JSON |
| 6103 | UDP | Mac -> Ubuntu | `dashboard_bridge` -> `udp_control_mode_override_rx` | control_mode override JSON |
| 6104 | UDP | Mac -> Ubuntu | `dashboard_bridge` -> `udp_ik_ready_request_rx` | IK ready request JSON |

## 3. Binary Packets

### 3.1 Mac -> Ubuntu uplink: `AI` (IK teleop)

受信: Ubuntu `udp_joy_bridge`（`0.0.0.0:5005`）

| Item | Value |
|---|---|
| Header | `b"AI"` |
| `struct` | `<2sBBfffffB` |
| Size | 25 bytes |

| Field | Type | Meaning |
|---|---|---|
| header | `2s` | `b"AI"` |
| seq | `uint8` | 送信側の連番（wrap可） |
| flags | `uint8` | 下表 |
| vy | `float32` | -1..1（Y方向の正規化速度） |
| vz | `float32` | -1..1（Z方向の正規化速度） |
| omega_base | `float32` | -1..1（ベースyaw入力） |
| omega_wrist_pitch | `float32` | -1..1（手首pitch入力） |
| omega_wrist_roll | `float32` | -1..1（手首roll入力） |
| control_byte | `uint8` | 共有control byte（AI byte24 / MC byte18 / AC byte30） |

`flags` bit layout（`udp_joy_bridge` の解釈）:

| Bit | Meaning |
|---:|---|
| 0 | enable/deadman（1=ON） |
| 1 | gripper（デジタル） |
| 2 | IK submode A request |
| 3 | IK submode B request |
| 4-5 | control mode（`0=IK, 1=MANUAL, 2=KEYBOARD_AUTO`） |

### 3.2 Mac -> Ubuntu uplink: `MC` (manual currents)

受信: Ubuntu `udp_joy_bridge`（`0.0.0.0:5005`）

| Item | Value |
|---|---|
| Header | `b"MC"` |
| `struct` | `<2sBB7HB` |
| Size | 19 bytes |

| Field | Type | Meaning |
|---|---|---|
| header | `2s` | `b"MC"` |
| seq | `uint8` | 送信側の連番（wrap可） |
| flags | `uint8` | `AI` と同じ（3.1） |
| current[0..6] | `uint16 x7` | 0..511（neutral=255） |
| control_byte | `uint8` | 共有control byte（AI byte24 / MC byte18 / AC byte30） |

`current[0..6]` の順序（UI/運用の呼称）:

| Index | Name |
|---:|---|
| 0 | M0 BaseHorizon |
| 1 | M1 BaseRoll |
| 2 | M2 Joint1 |
| 3 | M3 Joint2 |
| 4 | M4 Joint3 |
| 5 | M5 GripperRoll |
| 6 | M6 Gripper |

### 3.3 Mac -> Ubuntu downlink: `JF` (encoder feedback + STM-side keyboard-auto ready/done flags)

受信: Ubuntu `udp_joint_state_rx`（`0.0.0.0:5010`）

| Item | Value |
|---|---|
| Header | `b"JF"` |
| `struct` | `<2sBB5HH` |
| Size | 16 bytes |

| Field | Type | Meaning |
|---|---|---|
| header | `2s` | `b"JF"` |
| seq | `uint8` | 受信側で生存確認に使用 |
| flags | `uint8` | STM32 定義（Keyboard Auto ready + phase-specific done ACK） |
| encoders[0..4] | `uint16 x5` | `J0` は signed-mm linear feedback、`J1..J4` は encoder14 angle/count 系 |
| crc16 | `uint16` | CRC-16/CCITT-FALSE（Little Endian, 先頭14Bに対して計算） |

Keyboard Auto が参照する `flags` bit（デフォルト設定）:

| Bit | Meaning |
|---:|---|
| 0 | kbd_yaman_ready（STM 側の wrist-90 handshake 完了） |
| 1 | stm_ready（keyboard auto 開始準備完了） |
| 2 | x_align_done（global X完了） |
| 3 | global_yz_done（global YZ完了） |
| 4 | local_yz_done（local YZ完了） |
| 5 | keyboard_home_yz_done（keyboard_home YZ復帰完了） |
| 6 | keyboard_home_x_done（keyboard_home X復帰完了） |

補足（現行運用）:

| Item | Detail |
|---|---|
| mission start | Keyboard Auto 開始時は mission 開始直後に miniPC へ `{"cmd":"start","text":"...","nonce":"..."}` を送ります。 |
| 1文字目へ進む条件 | `miniPC start ACK` と STM ready（`JF.flags.bit1`）の **両方成立**です。 |

### 3.4 Ubuntu -> Mac unified command: `AC v6`

送信: Ubuntu `udp_ac_tx`（UDP 7000）

| Item | Value |
|---|---|
| Header | `b"AC"` |
| `struct` | `<2sBB7H3H3hBhHHH`（payload(37B) + `crc16`） |
| Size | 39 bytes |

| Field | Type | Meaning |
|---|---|---|
| header | `2s` | `b"AC"` |
| seq | `uint8` | 送信側の連番（wrap可） |
| flags | `uint8` | 下表 |
| current[0..6] | `uint16 x7` | 0..511（neutral=255） |
| angle[0..2] | `uint16 x3` | encoder14 counts（M2..M4） |
| vel[0..2] | `int16 x3` | rpm（M2..M4）。`KEYBOARD_AUTO` では 0 固定で未使用 |
| control_byte | `uint8` | packet byte30 の共有control byte（下表参照） |
| base_target_mm_j0 | `int16` | keyboard auto のJ0絶対目標位置（mm） |
| auto_flags | `uint16` | keyboard auto 状態フラグ（6章参照） |
| fault_code | `uint16` | keyboard auto fault（7章参照） |
| crc16 | `uint16` | CRC-16/CCITT-FALSE（Little Endian, 先頭37Bに対して計算） |

`flags` bit layout（`udp_ac_tx` の生成）:

| Bit | Meaning |
|---:|---|
| 0 | enable |
| 1 | gripper（デジタル） |
| 2 | mission_panel（IK submode。1=ミッション） |
| 4-5 | control mode（`0=IK, 1=MANUAL, 2=KEYBOARD_AUTO`） |

補足:

| Item | Detail |
|---|---|
| 通常経路 | 通常は Ubuntu の `udp_ac_tx` が `AC v6` を生成します。 |
| 簡易確認用経路 | 実機の簡易確認用に `tools/gamepad_mode_sound_only_mac.py` が **同じ `AC v6` バイト列**を生成し、XBee(TX)へ直接シリアル送信することがあります（MANUAL current のみ）。 |
| bit1 の旧名 | Mac の manual-only script では byte30 bit1 を旧名 `USB_READ_DATA` として定義しているが、bit 位置は current の `KBD_EN` と同じです。 |

共有 control byte（`AI` byte24 / `MC` byte18 / `AC v6` byte30）の bit 定義:

運用上、未使用bitは **0固定** とします（将来拡張用に予約）。

| Bit | Name | Meaning |
|---:|---|---|
| 0 | `KBD_PP` | keyboard push/pull pulse |
| 1 | `KBD_EN` | keyboard auto の押下許可 / arm 済み |
| 2 | `KBD_YAMAN` | pre-READY wrist-90 handshake request |
| 3 | `NYOKKI_PUSH` | manual 時の nyokki push パルス |
| 4 | `NYOKKI_PULL` | manual 時の nyokki pull パルス |
| 5 | `INIT` | initialize one-shot |
| 6 | `HOME` | home pose one-shot |
| 7 | `KBD_START` | keyboard auto start / character-cycle active |

## 4. miniPC <-> Ubuntu（Mac relay）: JSON

### 4.1 miniPC -> Ubuntu: detection JSON（UDP 6001）

送信: miniPC `arm9_minipc`（UDP 6001 -> Mac -> Ubuntu）

必須フィールド（miniPC実装が送るもの）:
```json
{
  "timestamp": 0.0,
  "t_capture": 0.0,
  "t_infer": 0.0,
  "seq": 0,
  "confidence": 0.0,
  "model_type": "global",
  "home_arm": [0.0, 0.0, 0.0],
  "keys": {
    "A": {"Xb_mm": 0.0, "Yb_mm": 0.0, "Zb_mm": 0.0}
  },
  "flags": 0,
  "frame_id": "dodai_1",
  "source": "minipc"
}
```

追加フィールド（任意・トップレベル）:

| Field | Meaning |
|---|---|
| `cmd_ack` | miniPC が start/cancel/phase を受け取ったACK（4.3参照） |
| `mission_active` | miniPC 側で mission active か |
| `mission_nonce` | miniPC 側で現在 active な mission nonce |
| `mission_phase` | miniPC 側の現在 phase（`global` / `local`） |
| `axes_authority` | その packet がどの軸責務で使われる想定か |
| `home_arm_source` | `home_arm` が `tracked_global` / `frame_keyboard` / `config_default` のどれ由来か |
| `home_arm_age_sec` | tracked keyboard home の更新からの経過秒 |
| `keyboard_home_valid` | tracked keyboard home が有効か |
| `home_arm_candidate` | 今フレームで `Keyboard` bbox から計算した候補値（未検出なら `null`） |
| `global_enabled` | runtime gate 的に global model が有効か |
| `local_enabled` | runtime gate 的に local model が有効か |
| `arm_pos` | miniPC 側が保持する現在の arm pose |
| `camera_ok` | 現フレームの camera health |
| `packet_version` | detection JSON の version |
| `health` | supervisor snapshot（有効時のみ） |
| `target_label` | active mission 中に要求されている target label |
| `target_index` | active mission 中の文字 index |
| `target_instance_count` | その frame で target label が何個見えているか |
| `target_unique` | `target_instance_count == 1` |
| `target_selected` | 今回の packet でその target を `keys` に採用したか |
| `target_reject_reason` | target を `keys` に採用しなかった理由 |

注:

| Item | Detail |
|---|---|
| `keys` label 正規化 | `keys` のキー（ラベル）は YOLO の class 名を **upper-case 正規化**したものです。例: `SPACE`, `''`, `F1..F12`, `0..9`, `, - . / ; = [ \\ ]`, `A..Z` |
| `KEYBOARD` の扱い | `KEYBOARD` は `keys` には入りません。 |
| `home_arm` | miniPC が保持する **tracked keyboard home** です。`global` model の `Keyboard` bbox center を camera->base 変換した値を runtime state に保存し、`local` phase や一時未検出でもその tracked 値を使い続けます。今フレームで計算した候補値は `home_arm_candidate` に残ります。 |
| `keys[*].Xb_mm/Yb_mm/Zb_mm` | 現行設定では raw key 座標ではなく、**nyokki 先端がキーの 40mm 手前で待機するための center target** です。 |
| active mission 中の `keys` | `target_label` で指定された **今の文字だけ**を `keys` に残します。 |

`target_reject_reason` の主な値:

| Value | Meaning |
|---|---|
| `missing_target_label` | target label が指定されていない |
| `target_not_detected` | target 自体が見えていない |
| `duplicate_target_label` | 同じ label が複数見えている |
| `target_not_stable` | stable gate をまだ満たしていない |

### 4.2 Ubuntu -> miniPC: command JSON（UDP 6002）

送信: Ubuntu `keyboard_auto_controller`（UDP 6002 -> Mac -> miniPC）

```json
{"cmd":"start","text":"ARES","nonce":"<uuid-hex>"}
{"cmd":"cancel","nonce":"<uuid-hex>"}
{"cmd":"phase","phase":"global|local","nonce":"<uuid-hex>","text":"ARES","label":"A","index":0}
```

### 4.3 miniPC -> Ubuntu: `cmd_ack`

miniPC は `cmd_ack` を 2 経路で返します。

| Route | Detail |
|---|---|
| standalone ACK | standalone UDP JSON (`{"source":"minipc_ack","cmd_ack":...}`) を即時返送 |
| detection 同梱 | detection JSON のトップレベルにも `cmd_ack` を同梱 |

```json
{
  "cmd_ack": {
    "nonce": "<uuid-hex>",
    "cmd": "start|cancel|phase",
    "text": "ARES",
    "phase": "global|local",
    "label": "A",
    "index": 0,
    "status": "accepted|rejected|cancelled",
    "reason": "..."
  }
}
```

active mission 中の detection packet は `label/index` で指定された **今の target** に正規化されます。

| Rule | Detail |
|---|---|
| target 正規化 | `target_label` / `target_index` で指定された target だけを `keys` に残します |
| duplicate reject | 同一 label が複数検出された場合は、その packet では target を reject します |

## 5. Frontend -> Ubuntu（dashboard_bridge 経由）: JSON (UDP)

### 5.1 Keyboard Auto request（UDP 6100）

`udp_keyboard_auto_request_rx` が受理:
```json
{"cmd":"start","text":"ARES","return_mode":"manual"}
{"cmd":"cancel"}
{"cmd":"release"}
```

`text` の仕様（Keyboard_eachkey の `model.names` と整合）:

| Item | Detail |
|---|---|
| 許可文字 | `A-Z` / `0-9` / `, - . / ; = [ \\ ]` / `SPACE`（空白 `" "`）/ `'`（apostrophe） |
| マルチ文字キー | token を使用: `<F1>.. <F12>`, `<SPACE>` |
| 例 | `ARES<SPACE><F10>` |
| 注意 | `F10` と書くと `F` と `1` と `0` の3キーになるので、Fキーは token 推奨 |

### 5.2 AC control-byte override（UDP 6101）

`udp_control_byte_override_rx` が受理:
```json
{"cmd":"home_pose"}        // AC packet control byte bit6
{"cmd":"initialize"}       // AC packet control byte bit5
{"control_byte":64}        // 任意の control byte 値
```

### 5.3 enable override（UDP 6102）

`udp_enable_override_rx` が受理:
```json
{"cmd":"estop_on"}         // publish 0
{"cmd":"estop_release"}    // publish 255
```

### 5.4 control_mode override（UDP 6103）

`udp_control_mode_override_rx` が受理:
```json
{"mode":"ik"}              // publish 0
{"mode":"manual"}          // publish 1
{"mode":"keyboard_auto"}   // publish 2
{"cmd":"clear"}            // publish 255
```

### 5.5 IK ready request（UDP 6104）

`udp_ik_ready_request_rx` が受理:
```json
{"cmd":"ready"}            // 現在のYAMAN姿勢を keyboard_home として capture
```

注:

| Item | Detail |
|---|---|
| `KBD_YAMAN` | `AC control_byte bit2` を pulse して STM 側の wrist-90 handshake を要求します。 |
| UI の `READY / RELEASE` | **keyboard_home capture / full reset** のことです。 |
| `RELEASE` | READY capture を破棄し、keyboard auto を止めて戻り先 mode へ戻します。 |
| `JF.flags` 対応 | `bit0` は `KBD_YAMAN ready`、`bit1` は **STM READY (downlink flag)** です。 |

## 6. Keyboard Auto: `auto_flags` bit layout（u16）

送信: Ubuntu `keyboard_auto_controller` → `udp_ac_tx`（`AC v6.auto_flags`）

| Bit | Meaning |
|---:|---|
| 0 | AUTO active |
| 1 | global stage active（`WAIT_GLOBAL` / `WAIT_X_ALIGN` / global YZ） |
| 2 | local stage active（`WAIT_LOCAL` / local YZ） |
| 3 | global detection valid |
| 4 | local detection valid |
| 5 | X command active（X align / keyboard_home X） |
| 6 | YZ command active（global/local/home YZ move） |
| 7 | IK valid |
| 8 | some JF done bit is currently high |
| 9 | global timeout |
| 10 | local timeout |
| 11 | JF timeout |
| 12 | fallback manual |

## 7. Keyboard Auto: `fault_code`（u16）

送信: Ubuntu `keyboard_auto_controller` → `udp_ac_tx`（`AC v6.fault_code`）

| Value | Meaning |
|---:|---|
| 0 | NONE |
| 1 | GLOBAL_TIMEOUT |
| 2 | LOCAL_TIMEOUT |
| 3 | JF_TIMEOUT |
| 4 | IK_FAILED |
| 5 | X_PHASE_FAILED |
| 6 | YZ_PHASE_FAILED |
| 7 | DEADMAN_OFF |
| 8 | PACKET_INVALID |
| 9 | MINIPC_ACK_TIMEOUT |
| 10 | MINIPC_ACK_REJECTED |
| 11 | STM_READY_TIMEOUT |