# AC パケット XBee 送信用縮小仕様（PacketMv2 / PacketIv2 / PacketBv2）

## 目的

`AC v6` パケットを XBee へ送信する前に、制御モードに応じて `PacketMv2`、`PacketIv2`、`PacketBv2` のいずれかへ縮小する。XBee へ実際に送信するのは、縮小後の v2 パケットとする。

v2 では、Enable / Disable を受信側へ伝えるため、従来パケットの `seq` 直後へ `flags` を 1 byte 追加する。header は従来と同じ ASCII `M` / `I` / `B` を維持するため、パケット長で v1 と v2 を区別する。

本仕様は `arm9_communication_data_formats.md` の `AC v6` 定義を元に、どのバイト位置がどの情報を表し、縮小時に何を保持または削除するかを整理したもの。

## 表記ルール

- バイト位置とビット位置はすべて 0 始まりとする。
- `byte N` は、変換前 `AC v6` パケットの 0 始まりバイト位置 `N` を指す。
- バイト範囲 `a~b` は `a` と `b` を含む。
- 複数バイト整数は Little Endian とする。
- ヘッダ変換とバイト削除後、残したバイトは順序を保ったまま前へ詰める。

## AC v6 元パケット構成

`AC v6` は 39 バイト固定長で、Python struct 表記では `<2sBB7H3H3hBhHHH`。

| byte 位置 | フィールド | 型 | 意味 |
| --- | --- | --- | --- |
| `0~1` | `header` | `2s` | ASCII `AC` |
| `2` | `seq` | `uint8` | 送信側の連番 |
| `3` | `flags` | `uint8` | enable / gripper / mission_panel / control mode |
| `4~5` | `current[0]` | `uint16` | M0 BaseHorizon current |
| `6~7` | `current[1]` | `uint16` | M1 BaseRoll current |
| `8~9` | `current[2]` | `uint16` | M2 Joint1 current |
| `10~11` | `current[3]` | `uint16` | M3 Joint2 current |
| `12~13` | `current[4]` | `uint16` | M4 Joint3 current |
| `14~15` | `current[5]` | `uint16` | M5 GripperRoll current |
| `16~17` | `current[6]` | `uint16` | M6 Gripper current |
| `18~19` | `angle[0]` | `uint16` | M2 encoder14 count |
| `20~21` | `angle[1]` | `uint16` | M3 encoder14 count |
| `22~23` | `angle[2]` | `uint16` | M4 encoder14 count |
| `24~25` | `vel[0]` | `int16` | M2 rpm。`KEYBOARD_AUTO` では未使用 |
| `26~27` | `vel[1]` | `int16` | M3 rpm。`KEYBOARD_AUTO` では未使用 |
| `28~29` | `vel[2]` | `int16` | M4 rpm。`KEYBOARD_AUTO` では未使用 |
| `30` | `control_byte` | `uint8` | keyboard auto / initialize / home などの共有制御ビット |
| `31~32` | `base_target_mm_j0` | `int16` | keyboard auto の J0 絶対目標位置 mm |
| `33~34` | `auto_flags` | `uint16` | keyboard auto 状態フラグ |
| `35~36` | `fault_code` | `uint16` | keyboard auto fault |
| `37~38` | `crc16` | `uint16` | 元 `AC v6` の CRC-16/CCITT-FALSE |

### flags

`flags` は AC v6 byte `3` の値を変更せず、そのまま各v2パケットのbyte `2`へ格納する。

| bit | 内容 |
| --- | --- |
| `0` | Enable。`1`: Enable、`0`: Disable |
| `1` | gripper |
| `2` | mission_panel |
| `4~5` | control mode。`0`: IK、`1`: MANUAL、`2`: KEYBOARD_AUTO |

未記載のbitも含め、変換処理では `flags` の全bitを保持する。

## パケット種別の判定

制御モードは、変換前 `AC v6` の `flags`、つまり byte `3` の bit `4~5` で判定する。

| 値 | 制御モード | 変換後パケット |
| --- | --- | --- |
| `0` | `IK` | `PacketIv2` |
| `1` | `MANUAL` | `PacketMv2` |
| `2` | `KEYBOARD_AUTO` | `PacketBv2` |

`3` は本仕様では未定義とする。

## 共通変換ルール

1. 変換前の `AC v6` から制御モードを判定する。
2. 先頭ヘッダ `AC` を削除し、変換後パケット種別を表す 1 バイトの ASCII 文字へ置き換える。
3. `seq` の直後へ、AC v6 byte `3` の `flags` を全bitそのまま保持する。
4. パケット種別ごとの削除対象フィールドを削除する。
5. 削除対象外のバイトを元の順序で詰め、変換後パケットとして XBee へ送信する。
6. 末尾 2 バイトの `crc16` は、変換後パケットの先頭から CRC 直前までのバイト列に対して CRC-16/CCITT-FALSE を再計算し、Little Endian で格納する。

## 削減方針まとめ

| AC フィールド | byte 位置 | PacketMv2 | PacketIv2 | PacketBv2 |
| --- | --- | --- | --- | --- |
| `header` | `0~1` | `M` へ置換 | `I` へ置換 | `B` へ置換 |
| `seq` | `2` | 保持 | 保持 | 保持 |
| `flags` | `3` | byte `2`へ保持 | byte `2`へ保持 | byte `2`へ保持 |
| `current[0]` M0 BaseHorizon | `4~5` | 保持 | 保持 | 削除 |
| `current[1]` M1 BaseRoll | `6~7` | 保持 | 保持 | 削除 |
| `current[2]` M2 Joint1 | `8~9` | 保持 | 削除 | 削除 |
| `current[3]` M3 Joint2 | `10~11` | 保持 | 削除 | 削除 |
| `current[4]` M4 Joint3 | `12~13` | 保持 | 削除 | 削除 |
| `current[5]` M5 GripperRoll | `14~15` | 保持 | 保持 | 削除 |
| `current[6]` M6 Gripper | `16~17` | 保持 | 保持 | 削除 |
| `angle[0]` M2 encoder14 | `18~19` | 削除 | 保持 | 保持 |
| `angle[1]` M3 encoder14 | `20~21` | 削除 | 保持 | 保持 |
| `angle[2]` M4 encoder14 | `22~23` | 削除 | 保持 | 保持 |
| `vel[0]` M2 rpm | `24~25` | 削除 | 削除 | 削除 |
| `vel[1]` M3 rpm | `26~27` | 削除 | 削除 | 削除 |
| `vel[2]` M4 rpm | `28~29` | 削除 | 削除 | 削除 |
| `control_byte` | `30` | 保持 | 保持 | 保持 |
| `base_target_mm_j0` | `31~32` | 削除 | 削除 | 保持 |
| `auto_flags` | `33~34` | 削除 | 削除 | 保持 |
| `fault_code` | `35~36` | 削除 | 削除 | 削除 |
| `crc16` | `37~38` | 再計算値へ置換 | 再計算値へ置換 | 再計算値へ置換 |

## PacketMv2

`MANUAL` の `AC v6` は `PacketMv2` へ変換する。

### 保持する情報

Manual 制御で必要な 7 軸 current、共有 `control_byte`、連番 `seq`、`flags` を保持する。
末尾の `crc16` は変換後 byte `0~17` に対して再計算した値を格納する。

### 削除する情報

- `angle[0..2]`: Manual では current 指令を使うため削除。
- `vel[0..2]`: この縮小仕様では全モードで削除。
- `base_target_mm_j0` / `auto_flags` / `fault_code`: Keyboard Auto 用情報のため削除。

### 変換後の構成

| 変換後 byte 位置 | 元 AC byte 位置 | フィールド | 意味 |
| --- | --- | --- | --- |
| `0` | 生成 | header | ASCII `M` |
| `1` | `2` | `seq` | 送信側の連番 |
| `2` | `3` | `flags` | Enable / control modeなど。全bitを保持 |
| `3~4` | `4~5` | `current[0]` | M0 BaseHorizon current |
| `5~6` | `6~7` | `current[1]` | M1 BaseRoll current |
| `7~8` | `8~9` | `current[2]` | M2 Joint1 current |
| `9~10` | `10~11` | `current[3]` | M3 Joint2 current |
| `11~12` | `12~13` | `current[4]` | M4 Joint3 current |
| `13~14` | `14~15` | `current[5]` | M5 GripperRoll current |
| `15~16` | `16~17` | `current[6]` | M6 Gripper current |
| `17` | `30` | `control_byte` | 共有制御ビット |
| `18~19` | 生成 | `crc16` | 変換後 byte `0~17` の CRC |

変換後サイズは 20 バイト。

## PacketIv2

`IK` の `AC v6` は `PacketIv2` へ変換する。

### 保持する情報

IK 制御で使う M0/M1/M5/M6 の current、M2/M3/M4 の encoder14 angle、共有 `control_byte`、連番 `seq`、`flags` を保持する。
末尾の `crc16` は変換後 byte `0~17` に対して再計算した値を格納する。

### 削除する情報

- `current[2..4]`: IK では M2/M3/M4 を angle 側で送るため削除。
- `vel[0..2]`: この縮小仕様では全モードで削除。
- `base_target_mm_j0` / `auto_flags` / `fault_code`: Keyboard Auto 用情報のため削除。

### 変換後の構成

| 変換後 byte 位置 | 元 AC byte 位置 | フィールド | 意味 |
| --- | --- | --- | --- |
| `0` | 生成 | header | ASCII `I` |
| `1` | `2` | `seq` | 送信側の連番 |
| `2` | `3` | `flags` | Enable / control modeなど。全bitを保持 |
| `3~4` | `4~5` | `current[0]` | M0 BaseHorizon current |
| `5~6` | `6~7` | `current[1]` | M1 BaseRoll current |
| `7~8` | `14~15` | `current[5]` | M5 GripperRoll current |
| `9~10` | `16~17` | `current[6]` | M6 Gripper current |
| `11~12` | `18~19` | `angle[0]` | M2 encoder14 count |
| `13~14` | `20~21` | `angle[1]` | M3 encoder14 count |
| `15~16` | `22~23` | `angle[2]` | M4 encoder14 count |
| `17` | `30` | `control_byte` | 共有制御ビット |
| `18~19` | 生成 | `crc16` | 変換後 byte `0~17` の CRC |

変換後サイズは 20 バイト。

## PacketBv2

`KEYBOARD_AUTO` の `AC v6` は `PacketBv2` へ変換する。

### 保持する情報

Keyboard Auto で使う M2/M3/M4 の encoder14 angle、共有 `control_byte`、J0 絶対目標位置 `base_target_mm_j0`、状態フラグ `auto_flags`、連番 `seq`、`flags` を保持する。
末尾の `crc16` は変換後 byte `0~13` に対して再計算した値を格納する。

### 削除する情報

- `current[0..6]`: Keyboard Auto では current 指令を送らないため削除。
- `vel[0..2]`: `KEYBOARD_AUTO` では 0 固定で未使用のため削除。
- `fault_code`: XBee 送信用縮小パケットでは送らない。

### 変換後の構成

| 変換後 byte 位置 | 元 AC byte 位置 | フィールド | 意味 |
| --- | --- | --- | --- |
| `0` | 生成 | header | ASCII `B` |
| `1` | `2` | `seq` | 送信側の連番 |
| `2` | `3` | `flags` | Enable / control modeなど。全bitを保持 |
| `3~4` | `18~19` | `angle[0]` | M2 encoder14 count |
| `5~6` | `20~21` | `angle[1]` | M3 encoder14 count |
| `7~8` | `22~23` | `angle[2]` | M4 encoder14 count |
| `9` | `30` | `control_byte` | 共有制御ビット |
| `10~11` | `31~32` | `base_target_mm_j0` | keyboard auto の J0 絶対目標位置 mm |
| `12~13` | `33~34` | `auto_flags` | keyboard auto 状態フラグ |
| `14~15` | 生成 | `crc16` | 変換後 byte `0~13` の CRC |

変換後サイズは 16 バイト。

## 実装時の注意

- 制御モード判定は、バイト削除やヘッダ置換を行う前の `AC v6` に対して実施する。
- 削除範囲は変換前 `AC v6` の byte 位置で判定する。
- 実装では元の `AC v6` を読みながら保持対象だけを出力バッファへコピーすると、インデックスずれを避けやすい。
- `crc16` は元 `AC v6` の値をコピーせず、縮小後のバイト列に対して再計算する。
- CRC 計算範囲は、PacketMv2 / PacketIv2では先頭18バイト、PacketBv2では先頭14バイトとする。
- header文字はv1と共通である。v1（M/I: 19 bytes、B: 15 bytes）とはパケット長とCRCが異なり、相互互換ではない。
