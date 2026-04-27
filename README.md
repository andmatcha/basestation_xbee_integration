# basestation_xbee_integration

## downlink

`downlink` は、`UART4` から受信したストリームを
`USART1 (Rover OUT)` と `USART2 (Arm OUT)` に振り分ける。

入力元は uplink の `UART4 TX` と接続される次の 1 系統。

- `UART4`: uplink bridge IN

出力先は次の 2 系統。

- `USART1`: Rover OUT
- `USART2`: Arm OUT

## モジュール構成

- `downlink/src/app.c`
  - `main.c` から呼ばれる `init()` / `poll()` を提供するアプリ層
- `downlink/src/modules/input_source_selector.c`
  - `PC13` のダブルクリックによる通常/サイエンスモード切り替えを管理する
- `downlink/src/modules/data_router.c`
  - `UART4` の入力ストリームを解釈し、rover / arm / science 用に振り分ける
- `downlink/src/modules/status_leds.c`
  - MODE LED / STATE LED の色と点滅状態を管理する
- `downlink/src/modules/rgb_led_driver.c`
  - `TIM3 CH3/CH4 (PB0/PB1)` を使って RGB LED 用の 1 線式波形を生成する

## 振り分け仕様

受信ストリームは 1 バイトずつ解釈され、通常は rover 用テキストとして扱う。
ただし、`J` の直後に `F` が来た場合は `JF` を arm パケットの先頭とみなし、
そこから固定長 16 バイトを arm 用データとして扱う。

### Rover データ形式

- 形式: 改行終端のテキスト行
- 終端: `\n`
- 無視する文字: `\r`
- 最大長: 64 バイト
- 受理条件: `0x3...` または `0x4...` で始まる `0xHEX,DECIMAL` 形式の行のみを rover へ送る
- 出力先: `USART1`
- 送信時の整形: 受信した 1 行をそのまま送信し、末尾に `\r\n` を付ける

補足:

- `J` 単体は rover データとして扱われる
- `JF` は rover 文字列ではなく arm パケット開始として解釈される
- 64 バイトを超えるとその時点の rover バッファは破棄される

### Arm データ形式

- 形式: `JF` で始まる固定長バイナリフレーム
- サイズ: 16 バイト固定
- 先頭 2 バイト: ASCII の `J` `F` (`0x4A 0x46`)
- 残り 14 バイト: 生バイト列
- 出力先: `USART2`
- 送信時の整形: 16 バイトをそのまま送信する

補足:

- arm パケット 16 バイトを受信し終えると、解釈モードは rover に戻る
- rover テキスト中に `JF` が現れた場合も arm パケット開始とみなされる

## 参考実装との対応

`ref/main.c` のフィルタロジックを `downlink` 向けの UART 割り当てに合わせて
モジュール化して移植している。

## LED 表示仕様

`PB0` は MODE LED、`PB1` は STATE LED として扱う。
どちらも `TIM3` の PWM 出力を使って 1 線式 RGB LED を駆動する。
表示色は `rgb_led_driver.c` 側で一律 50% までに制限している。

### MODE LED

- UART4 入力待受中: 水色 `RGB(0, 160, 255)`
- サイエンスモード中は紫との交互表示

### STATE LED

- 初期化完了後の待機状態: 緑 `RGB(0, 255, 0)` で点灯
- UART4 入力で受信が続いている間: 高速点滅
- 点滅の半周期: `60ms`
- 受信が止まったと判断する保持時間: `250ms`

受信頻度は「直近の受信バイト数/秒」を簡易指標として色へ反映している。
色としきい値は `downlink/src/modules/status_leds.c` のマクロで後から変更できる。

- 低頻度: `50 B/s` 付近で水色 `RGB(0, 160, 255)`
- 中頻度: `800 B/s` 付近で黄 `RGB(255, 220, 0)`
- 高頻度: `3000 B/s` 以上で赤 `RGB(255, 32, 0)`

この 3 点の間は線形補間でグラデーション表示する。

### RGB LED 信号形式

- タイマ周期: 約 `800kHz`
- 1 フレーム: `Green`, `Red`, `Blue` の順で各 8bit、合計 24bit
- `0` ビット: High 約 `25 ticks`、Low 残り
- `1` ビット: High 約 `58 ticks`、Low 残り
- Reset: Low `240` スロット以上、約 `300us`

`rgb_led_driver.c` では 24bit + reset 分の波形を先に展開し、`TIM3_UP` の DMA で
`CCR3/CCR4` を更新している。800kHz ごとの CPU 割り込みではなく DMA 駆動にして、
UART 受信処理と両立しやすい構成にしている。

補足:

- `TIM3_UP` が `DMA1_Stream2` を使うため、未使用だった `UART4 RX DMA` は無効化している
- `PB0/PB1` は LED 波形の立ち上がりを確保するため `GPIO_SPEED_FREQ_VERY_HIGH` にしている

## uplink

`uplink` は、次の 2 系統を同時に扱う。

- `USART1 (Rover IN)` と `USART2 (Arm/Science IN)` から受信したデータを、
  選択中の `USART3` または `USART6` へ送信する
- `USART3 (USB IN)` と `USART6 (XBee IN)` から受信したデータを、
  加工せず `UART4 TX` へ中継する

旧来の出力先切り替えは維持しており、`PC13` のタクトスイッチを長押しすると
`USART3` / `USART6` の出力先が切り替わる。ダブルクリックで通常/サイエンスモードを切り替える。

旧 uplink 経路の入力元は次の 2 系統。

- `USART1`: Rover IN
- `USART2`: Arm/Science IN

旧 uplink 経路の出力先は次の 2 系統。

- `USART3`: USB OUT
- `USART6`: XBee OUT

downlink bridge 経路は次の構成。

- `USART3`: USB IN
- `USART6`: XBee IN

- `UART4`: downlink bridge OUT

### モジュール構成

- `uplink/src/app.c`
  - `main.c` から呼ばれる `init()` / `poll()` を提供するアプリ層
- `uplink/src/modules/output_source_selector.c`
  - `PC13` による出力先切り替えと通常/サイエンスモード切り替えを管理する
- `uplink/src/modules/data_router.c`
  - `USART1` / `USART2` の入力を解釈して選択中の `USART3` / `USART6` へ送る
  - `USART3` / `USART6` の RX を circular DMA で読み、UART4 へ raw byte として送る
- `uplink/src/modules/status_leds.c`
  - MODE LED / STATE LED の色と点滅状態を管理する
- `uplink/src/modules/rgb_led_driver.c`
  - `TIM3 CH3/CH4 (PB0/PB1)` を使って RGB LED 用の 1 線式波形を生成する

### 旧 uplink 経路

- `USART1` の rover テキストは `0x3...` または `0x4...` の CAN テキストとして検証して送出する
- 通常モードでは `USART2` の arm バイナリを `AC` ヘッダと CRC で同期・検証して送出する
- サイエンスモードでは `USART2` の `0x5..,...` テキストを science として送出する
- `USART1` の `0x3..` / `0x4..` テキストはサイエンスモード中も rover として送出する
- 送出先は `output_source_selector` が選ぶ `USART3` または `USART6`

### downlink bridge 経路

- `USART3` / `USART6` の RX はどちらも circular DMA で受信する
- 受信したバイトは解釈・整形せず UART4 TX キューに入れる
- UART4 へ中継する RX は、旧 uplink 経路の出力先と同じ `USART3` または `USART6` のみを選択する
- rover / arm / science の最終的な振り分けは downlink 側で行う
- XBee は transparent mode 前提で扱い、API frame の `0x7E` delimiter 解析はしない

### LED 表示仕様

LED の点滅周期、受信頻度グラデーション、`TIM3` + DMA による駆動方式は
`downlink` と同一で、MODE LED は選択中の旧 uplink 出力先とサイエンスモードを示す。
表示色の最大光量も `rgb_led_driver.c` 側で 50% に制限している。

補足:

- `uplink` でも `TIM3_UP` が `DMA1_Stream2` を使うため、未使用の `UART4 RX DMA` は無効化している
