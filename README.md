# basestation_xbee_integration

STM32F446RE を使った基地局 XBee 統合ファームウェアです。

- `uplink/`, `downlink/`: v1 基板向け。1 枚の v1 基板上で uplink 側マイコンと downlink 側マイコンに分かれる。
- `integrated/`: v2 基板向け。1 個のマイコンで uplink/downlink、LCD、2 系統 XBee をまとめて扱う。

XBee は transparent mode 前提です。API frame の `0x7E` delimiter 解析や
7-bit/mask 変換は行わず、受信した UART ストリームをそのまま対象プロトコルとして解釈します。

## 共通データ形式

### Rover text

Rover 系データは改行終端の ASCII テキストとして扱います。

- 通常モード: `0x3...` または `0x4...` で始まる `0xHEX,DECIMAL...`
- science mode: `0x3xx,...` または `0x4xx,...`
- `\r` は無視し、`\n` で 1 行を確定する
- 受理した行は送信先へ本文 + `\r\n` として送る
- 行が長すぎる場合はその行を破棄する

通常モードでは CAN ID が `0x7FF` を超える行や、カンマを含まない行は破棄します。

### Science text

Science 系データは science mode でだけ使います。

- 形式: `0x5xx,...`
- `\r` は無視し、`\n` で 1 行を確定する
- 受理した行は送信先へ本文 + `\r\n` として送る

### Arm uplink: AC v6

Arm への uplink command は `AC v6` を扱います。

- 形式: `AC` で始まる固定長バイナリ
- サイズ: 39 bytes
- CRC: CRC-16/CCITT-FALSE、Little Endian、先頭 37 bytes に対して計算

XBee へ送る前に `AC v6` を縮小する経路では、`flags` の control mode
に応じて `PacketMv2` / `PacketIv2` / `PacketBv2` に変換し、CRC を再計算します。
各v2パケットはAC v6の `flags` をbyte `2`に全bitそのまま保持し、bit `0`でEnable / Disableを伝えます。

- `MANUAL`: `PacketMv2`（header `M`）、20 bytes
- `IK`: `PacketIv2`（header `I`）、20 bytes
- `KEYBOARD_AUTO`: `PacketBv2`（header `B`）、16 bytes

縮小の詳細は [AC_PACKET_XBEE_REDUCTION_SPEC.md](./AC_PACKET_XBEE_REDUCTION_SPEC.md) を参照してください。

### Arm downlink: JF

Arm からの downlink feedback は `JF` を扱います。

- 形式: `JF` で始まる固定長バイナリ
- サイズ: 16 bytes
- CRC: CRC-16/CCITT-FALSE、Little Endian、先頭 14 bytes に対して計算

通常モードの XBee/downlink ストリームでは、`J` の直後に `F` が来た場合に
`JF` packet 開始として扱います。そのため通常モードの Rover text 中に `JF` を含めるのは避けてください。

### USB feedback text: UF v2

USB memory 由来の text feedback は `UF v2` を扱います。

- 形式: `UF` で始まる固定長バイナリ
- サイズ: 40 bytes
- CRC: CRC-16/CCITT-FALSE、Little Endian、先頭 38 bytes に対して計算
- payload は最大 32 bytes。`payload_len` が 32 を超える packet は破棄する

## v1 基板: `uplink/` と `downlink/`

v1 は 1 枚の基板上に uplink 側マイコンと downlink 側マイコンが分かれている構成です。

- `uplink/`: Rover/Arm/Science から来た uplink を USB または XBee へ送る。
- `downlink/`: uplink 側マイコンから来た downlink ストリームを Rover/Arm/Science へ振り分ける。

### v1 ハードウェアと通信設定

| 項目 | 設定 |
| --- | --- |
| MCU | STM32F446RET6 x2、uplink 側 / downlink 側 |
| PlatformIO clock | 180 MHz |
| UART 設定 | 115200 bps, 8 data bits, no parity, 1 stop bit, no flow control |
| ボタン | `PC13`、pull-up、押下時 `LOW` |
| MODE LED | `PB0` / `TIM3_CH3` の WS2812 |
| STATE LED | `PB1` / `TIM3_CH4` の WS2812 |
| Buzzer | `TIM13_CH1` |

v1 には LCD はありません。状態は MODE LED、STATE LED、buzzer で確認します。

### v1 uplink の UART 割り当て

| UART | 役割 |
| --- | --- |
| `USART1` | Rover IN |
| `USART2` | Arm IN または Science IN |
| `USART3` | USB OUT / USB downlink RX |
| `USART6` | XBee OUT / XBee downlink RX |
| `UART4` | downlink 側マイコンへの OUT |

### v1 downlink の UART 割り当て

| UART | 役割 |
| --- | --- |
| `UART4` | uplink 側マイコンからの LINK IN |
| `USART1` | Rover OUT |
| `USART2` | Arm OUT または Science OUT |

`downlink/` 側の `USART3` / `USART6` は現行ルーティングでは入力元として使いません。

### v1 ボタン操作とモード切り替え

`uplink/` と `downlink/` のどちらも `PC13` の 1 ボタン操作です。

| 操作 | uplink 側マイコン | downlink 側マイコン |
| --- | --- | --- |
| 起動直後 | XBee OUT、science mode OFF | UART4 IN、science mode OFF |
| 長押し 1 秒以上 | 出力先を `USART6 XBee OUT` と `USART3 USB OUT` で切り替え | なし |
| 350 ms 以下の短押し 2 回 | science mode ON/OFF | science mode ON/OFF |
| 短押し 2 回の間隔 | 400 ms 以内 | 400 ms 以内 |

science mode は uplink 側マイコンと downlink 側マイコンで独立しているため、Science を使う場合は両方のマイコンで同じ mode にしてください。

### v1 mode の説明

#### Normal / Arm mode

Rover text と Arm binary を扱う通常モードです。

uplink:

```text
USART1 Rover IN  -- Rover text 0x3/0x4 --> selected OUT(USART3 or USART6)
USART2 Arm IN    -- AC v6 39 bytes -----> selected OUT(USART3 or USART6)
USART3 RX        -- raw downlink -------> UART4 TX
USART6 RX        -- raw downlink -------> UART4 TX
```

- Rover text は妥当な `0x3...` / `0x4...` 行だけを通す。
- Arm IN は `AC` に同期し、39 bytes 集めて CRC が正しい packet だけを通す。
- 出力先が XBee (`USART6`) の場合、`AC v6` は `PacketMv2` / `PacketIv2` / `PacketBv2` へ縮小してから送信する。
- 出力先が USB (`USART3`) の場合、`AC v6` は 39 bytes のまま送信する。

downlink:

```text
UART4 LINK IN -- Rover text 0x3/0x4 --> USART1 Rover OUT
UART4 LINK IN -- JF 16 bytes ---------> USART2 Arm OUT
```

- Rover text は妥当な `0x3...` / `0x4...` 行だけを通す。
- `JF` は 16 bytes 集め、CRC が正しい packet だけを `USART2` へ送る。

#### Science mode

Arm binary の代わりに Science text を扱うモードです。

uplink:

```text
USART1 Rover IN    -- 0x3xx/0x4xx text --> selected OUT(USART3 or USART6)
USART2 Science IN  -- 0x5xx text -------> selected OUT(USART3 or USART6)
USART3/USART6 RX   -- raw downlink -----> UART4 TX
```

downlink:

```text
UART4 LINK IN -- 0x3xx/0x4xx text --> USART1 Rover OUT
UART4 LINK IN -- 0x5xx text -------> USART2 Science OUT
```

science mode 中は `JF` binary の切り出しを行わず、改行終端の text line として分類します。

### v1 LED 表示

MODE LED は現在の通信 mode を示します。

| 状態 | 表示 |
| --- | --- |
| uplink: USB OUT | オレンジ `RGB(255, 96, 0)` |
| uplink: XBee OUT | 水色 `RGB(0, 160, 255)` |
| downlink: UART4 IN | 水色 `RGB(0, 160, 255)` |
| science mode ON | 基本色と紫 `RGB(160, 0, 255)` を 400 ms 周期で交互表示 |

STATE LED は受信 activity を示します。

| 状態 | 表示 |
| --- | --- |
| 待機 | 緑 `RGB(0, 255, 0)` |
| 受信中 | 半周期 60 ms で点滅 |
| 受信停止判定 | 最終受信から 250 ms |

受信中の色は直近 500 ms の受信 bytes/sec をもとに変わります。

- 低頻度: `50 B/s` 付近で水色 `RGB(0, 160, 255)`
- 中頻度: `800 B/s` 付近で黄 `RGB(255, 220, 0)`
- 高頻度: `3000 B/s` 以上で赤 `RGB(255, 32, 0)`

RGB LED は 800 kHz の WS2812 波形で駆動し、最大輝度は 50% に制限しています。

## v2 基板: `integrated/`

v2 は `integrated/` の 1 マイコン構成です。Rover、Arm/Science、外付け XBee、オンボード XBee、LCD を同時に扱います。

### v2 ハードウェアと通信設定

| 項目 | 設定 |
| --- | --- |
| MCU | STM32F446RET6 |
| PlatformIO clock | 160 MHz |
| UART 設定 | 115200 bps, 8 data bits, no parity, 1 stop bit, no flow control |
| LCD | I2C 16x2、7-bit address `0x3E` |
| I2C1 | `PB8` SCL、`PB9` SDA、100 kHz |
| LCD reset | `PB7` |
| Mode button | `PC13` / `PUSH_SWITCH_1`、pull-up、押下時 `LOW` |
| Display button | `PC14` / `PUSH_SWITCH_2`、pull-up、押下時 `LOW` |
| RSSI PWM input | `PA0` / `TIM2_CH1/CH2` |
| Buzzer | `PA6` / `TIM13_CH1`、4 kHz beep |

### v2 UART 割り当て

| UART | 役割 |
| --- | --- |
| `USART1` | Rover IN |
| `USART2` | Module IN。Arm mode では Arm IN、Science mode では Science IN |
| `UART4` | Rover OUT。Science mode の Rover downlink に使用 |
| `UART5` | USB4 / Module OUT。Arm mode では Rover/Arm/UF downlink、Science mode では Science OUT |
| `USART3` | External XBee |
| `USART6` | Onboard XBee |

### v2 ボタン操作とモード切り替え

#### `PUSH_SWITCH_1` / `PC13`: 通信 mode

| 操作 | 動作 | 音 |
| --- | --- | --- |
| 起動直後 | `Arm` / `External` | 起動時に 3 beep |
| 1 回押して離す | Module mode を `Arm` / `Science` で切り替え | 1 beep |
| 350 ms 以内に 2 回押す | XBee mode を `External` / `Onboard` で切り替え | 2 beep |

1 回押しは、2 回押し判定のため release 後 350 ms 待ってから確定します。

#### `PUSH_SWITCH_2` / `PC14`: LCD 表示

押して離すたびに LCD を `Status view` と `Rate view` で切り替えます。

- 起動直後は `Status view`
- debounce は 30 ms
- 切り替え時に 1 beep

### v2 mode の説明

#### Module mode: Arm

Rover text と Arm binary を扱う通常モードです。

```text
USART1 Rover IN  -- Rover text 0x3/0x4 --> active XBee(USART3 or USART6)
USART2 Arm IN    -- AC v6 39 bytes -----> active XBee(USART3 or USART6), M/I/B v2へ縮小
USART2 Arm IN    -- JF 16 bytes --------> active XBee(USART3 or USART6), raw
active XBee RX   -- Rover text 0x3/0x4 --> UART5 USB4 OUT
active XBee RX   -- JF 16 bytes --------> UART5 USB4 OUT
active XBee RX   -- UF v2 40 bytes -----> UART5 USB4 OUT
```

- `AC v6` は CRC 確認後に XBee 送信用の `PacketMv2` / `PacketIv2` / `PacketBv2` へ縮小する。
- `JF` は 16 bytes と CRC を確認して raw のまま転送する。
- Rover text は `0x3...` / `0x4...` で始まる妥当な行だけを通す。
- Arm mode の downlink は Rover text、Arm `JF`、`UF v2` を UART5 / USB4 から出力する。

#### Module mode: Science

Arm binary の代わりに Science text を扱うモードです。

```text
USART1 Rover IN    -- 0x3xx/0x4xx text --> active XBee(USART3 or USART6)
USART2 Science IN  -- 0x5xx text -------> active XBee(USART3 or USART6)
active XBee RX     -- 0x3xx/0x4xx text --> UART4 Rover OUT
active XBee RX     -- 0x5xx text -------> UART5 Science OUT
```

Science mode では port 間違いを検出します。たとえば `0x5xx` が Rover IN に来た場合や、
`0x3xx` / `0x4xx` が Science IN に来た場合は `WP` として扱い、転送しません。

#### XBee mode: External / Onboard

active XBee を選択する mode です。

- `External`: `USART3` を XBee として使う。起動直後の default。
- `Onboard`: `USART6` を XBee として使う。debug build では RSSI PWM を 1 秒ごとに log 出力する。

active ではない XBee UART から受信した bytes は読み捨て、downlink parser には入れません。

### v2 LCD 表示

LCD は 16 文字 x 2 行です。200 ms ごとに再描画します。

#### Startup

起動直後に約 1 秒表示します。

```text
KONNICHIWA

```

#### Status view

起動後の default 表示です。

```text
Arm External
UP:OK DOWN:--
```

1 行目は現在の `Module mode` と `XBee mode` です。

- `Arm External`
- `Arm Onboard`
- `Science External`
- `Science Onboard`

2 行目は uplink/downlink の直近 status です。

| 表示 | 意味 |
| --- | --- |
| `OK` | 正常に受理または送信 |
| `FM` | format error |
| `WP` | wrong port |
| `SY` | sync error |
| `CR` | CRC error |
| `OF` | overflow |
| `QF` | TX queue full |
| `ER` | UART/HAL error |
| `--` | 直近 250 ms に status なし |

#### Rate view

通信頻度を Hz 単位で表示します。

```text
RF:TX12Hz/RX4Hz
R:5/4 A:10/9
```

- 1 行目: active XBee 側の TX/RX rate。`999` で上限表示。
- 2 行目: `R:uplink_rx/downlink_rx` と `A:` または `S:` の `module_uplink_rx/module_downlink_rx`。
- 2 行目の各 rate は `99` で上限表示。

#### Error

LCD 初期化、再描画、UART 送信開始などで fatal error と判断した場合に表示します。

```text
ERROR OCCURED

```

`OCCURED` は実装上の表示文字列そのままです。

### v2 データフィルタリング概要

- 各 UART RX は DMA circular buffer で受け、`poll()` で差分を処理する。
- TX は queue に積み、DMA transmit 完了 callback で次の frame を送る。
- mode 変更時は parser 状態を reset し、XBee mode 変更時は inactive 側の受信位置を flush する。
- Rover/Science text は `\r` を捨て、`\n` で確定する。
- Arm mode の XBee downlink は、Rover text、`JF` binary、`UF v2` binary が混在する stream として処理する。
- Science mode の XBee downlink は text line だけを扱い、`0x3xx` / `0x4xx` を Rover、`0x5xx` を Science へ振り分ける。
- `AC v6`、`JF`、`UF v2` は CRC が一致した packet だけを転送する。

## ビルドと書き込み

```sh
make uplink
make downlink
make integrated
```

debug log 有効 build は次を使います。

```sh
make uplink-debug
make downlink-debug
make integrated-debug
```

AC v6からM/I/B v2への縮小変換は、ホスト上で次のテストを実行できます。

```sh
tests/run_ac_packet_reducer_tests.sh
```
