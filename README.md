# basestation_xbee_integration

## downlink

`downlink` は、選択中の入力 UART から受信したストリームを
`USART1 (Rover OUT)` と `USART2 (Arm OUT)` に振り分ける。

入力元は次の 2 系統で、`PC13` のタクトスイッチを 2 秒長押しすると切り替わる。

- `USART3`: USB IN
- `USART6`: XBee IN

出力先は次の 2 系統。

- `USART1`: Rover OUT
- `USART2`: Arm OUT

## モジュール構成

- `downlink/src/app.c`
  - `main.c` から呼ばれる `init()` / `poll()` を提供するアプリ層
- `downlink/src/modules/input_source_selector.c`
  - `PC13` の長押しによる入力元切り替えを管理する
- `downlink/src/modules/data_router.c`
  - 選択中の入力ストリームを解釈し、rover 用と arm 用に振り分ける

## 振り分け仕様

受信ストリームは 1 バイトずつ解釈され、通常は rover 用テキストとして扱う。
ただし、`J` の直後に `F` が来た場合は `JF` を arm パケットの先頭とみなし、
そこから固定長 16 バイトを arm 用データとして扱う。

### Rover データ形式

- 形式: 改行終端のテキスト行
- 終端: `\n`
- 無視する文字: `\r`
- 最大長: 64 バイト
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
