# STM32 向け WS2812 RGB LED 実装ガイド

## この文書の目的

この文書は、WS2812 / WS2812B 系 RGB LED を STM32 で再実装するときに、次の 3 点を満たすことを目的にしています。

- 人間が読んで、設計意図と移植手順を迷わず理解できること
- AI エージェントが読んで、必要な設定値とコード生成方針を再現できること
- 別の STM32 プロジェクトでも、同じ考え方で実装を再利用できること

このリポジトリの `ARES9_IndicatorLED` は、`TIM PWM + DMA` 方式で WS2812 を駆動しています。  
本書ではまず一般化した実装方法を示し、最後にこのプロジェクト固有の実例を対応付けます。

## 先に結論

STM32 で WS2812 を安定駆動する基本構成は次のとおりです。

1. タイマで 1.25us 周期の PWM を作る
2. 1bit ごとの High 幅を `0` 用と `1` 用の 2 種類の比較値で表現する
3. LED データを `GRB` 順に並べ、24bit 分の比較値列へ展開する
4. DMA でその比較値列をタイマの CCR レジスタへ流し込む
5. 最後に 50us 以上 Low を維持してラッチさせる

この方式は、GPIO をソフトウェアでビットバンギングする方式より再現性が高く、CPU 負荷も低いため、STM32 での定番構成です。

## 実装前に確定すべき入力情報

人間でも AI エージェントでも、実装前に最低限次の情報を確定してください。

- 使用 MCU 型番
- 実際のタイマ入力クロック `f_tim`
- 使用するタイマインスタンスとチャネル
- そのチャネルを出せる GPIO ピンと Alternate Function
- そのタイマチャネルに対応する DMA リクエスト / DMA チャネル
- LED 数
- LED 電源電圧と、必要ならレベルシフタの有無
- 他機能とタイマ / DMA を共有していないか

この情報が曖昧なまま実装を始めると、コードは書けても波形が再現できません。

## WS2812 プロトコルとして必要な条件

WS2812 は 1 線式ですが、中身は厳密なタイミング駆動です。

- 1bit 周期はおおむね `1.25us`
- ビットレートはおおむね `800kHz`
- `0` は短い High、`1` は長い High で表現する
- 1 LED あたり 24bit
- 送信順は通常 `G -> R -> B`
- 転送後に `50us 以上` Low を維持するとラッチされる

実務上は、次のような値に落とすと扱いやすいです。

- `T0H`: 約 `0.35us` から `0.45us`
- `T1H`: 約 `0.70us` から `0.90us`
- `Treset`: `50us 以上`

## 一般化した設計手順

### 1. 1bit 周期をタイマで作る

まず、タイマの 1 周期が WS2812 の 1bit に対応するように設定します。

式:

```text
bit_rate = 800000 Hz
bit_period = 1 / bit_rate = 1.25 us

period_counts = f_tim / bit_rate
ARR = period_counts - 1
PSC は通常 0 を優先
```

例:

- `f_tim = 64MHz`
- `period_counts = 64,000,000 / 800,000 = 80`
- `ARR = 79`

このとき、タイマ 1 周期がちょうど `1.25us` になります。

### 2. `0` と `1` の High 幅を比較値に変換する

PWM の比較値は、1 周期のうち何カウント分 High にするかを表します。

式:

```text
duty_0 = round(period_counts * T0H / 1.25us)
duty_1 = round(period_counts * T1H / 1.25us)
```

実用上は、次のように固定してよいです。

- `0` 用比較値: 周期の `30% から 35%` 前後
- `1` 用比較値: 周期の `60% から 70%` 前後

64MHz / 800kHz / 80counts の例:

- `duty_0 = 26` なら `26 / 64MHz = 0.40625us`
- `duty_1 = 52` なら `52 / 64MHz = 0.8125us`

### 3. リセット時間をバッファ末尾に入れる

WS2812 はデータ転送後、しばらく Low が続くとデータを確定します。  
PWM + DMA 方式では、比較値 0 のスロットを末尾に追加してこの時間を作ります。

式:

```text
reset_slots >= ceil(50us / 1.25us)
```

最低 40 スロット程度で足りますが、実務では 50 から 80 スロット程度に余裕を持たせると安定しやすいです。

### 4. 送信バッファ長を決める

1 LED は 24bit なので、必要なバッファ長は次のとおりです。

```text
buffer_size = led_count * 24 + reset_slots
```

## 推奨するソフトウェア構成

移植しやすさを優先するなら、次の責務分離が扱いやすいです。

- `ws2812.h`
  設定値、公開 API、移植時に変えるマクロを置く
- `ws2812.c`
  バッファ生成と DMA 送信処理を置く
- `main.c` などアプリ層
  点灯パターンや状態遷移だけを書く

これにより、別プロジェクトへ移すときは `ws2812.h/.c` と CubeMX 設定を主に差し替えれば済みます。

## CubeMX / HAL での再現チェックリスト

### タイマ

- PWM 出力を使う
- 使用チャネルを 1 つ決める
- `PSC = 0` を優先する
- `ARR = period_counts - 1`
- 初期 `Pulse = 0`
- PWM モードは `PWM1`

### GPIO

- タイマ出力ピンを Alternate Function にする
- `AF_PP`
- `NOPULL`

### DMA

- 方向は `MEMORY_TO_PERIPH`
- メモリインクリメントは有効
- ペリフェラルインクリメントは無効
- DMA 割り込みを有効化する

### NVIC

- DMA IRQ を有効化する
- `HAL_DMA_IRQHandler()` が呼ばれる ISR を生成する

### クロック

- タイマ入力クロック `f_tim` を必ず確認する
- APB 分周がある場合、タイマクロックが PCLK と一致しないことがある

ここは STM32 で最も誤解しやすい点です。  
`PCLK = 32MHz` でも、タイマクロックだけ `64MHz` になる構成は普通にあります。

## 最小構成の移植テンプレート

以下は、他の STM32 プロジェクトへ持ち込みやすい最小構成の例です。  
差し替え対象をマクロへ寄せているので、移植時はまず `TIM ハンドル / インスタンス / チャネル / 比較値 / LED 数` を更新してください。

### `ws2812.h`

```c
#ifndef WS2812_H
#define WS2812_H

#include "main.h"

#define WS2812_LED_COUNT      16U
#define WS2812_RESET_SLOTS    60U
#define WS2812_BUFFER_SIZE    ((WS2812_LED_COUNT * 24U) + WS2812_RESET_SLOTS)

#define WS2812_DUTY_0         26U
#define WS2812_DUTY_1         52U

#define WS2812_TIM_HANDLE     htim2
#define WS2812_TIM_INSTANCE   TIM2
#define WS2812_TIM_CHANNEL    TIM_CHANNEL_4

void WS2812_Init(void);
void WS2812_Clear(void);
void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
HAL_StatusTypeDef WS2812_Show(void);
uint8_t WS2812_IsBusy(void);

#endif
```

### `ws2812.c`

```c
#include "ws2812.h"

extern TIM_HandleTypeDef WS2812_TIM_HANDLE;

static uint32_t ws2812_pwm_data[WS2812_BUFFER_SIZE];
static volatile uint8_t ws2812_busy = 0U;

void WS2812_Init(void)
{
    WS2812_Clear();
}

void WS2812_Clear(void)
{
    for (uint32_t i = 0; i < WS2812_BUFFER_SIZE; i++) {
        ws2812_pwm_data[i] = 0U;
    }
}

void WS2812_SetPixel(uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index >= WS2812_LED_COUNT) {
        return;
    }

    uint32_t color = ((uint32_t)g << 16) | ((uint32_t)r << 8) | (uint32_t)b;
    uint32_t base = index * 24U;

    for (uint32_t bit = 0; bit < 24U; bit++) {
        ws2812_pwm_data[base + bit] =
            (color & (1UL << (23U - bit))) ? WS2812_DUTY_1 : WS2812_DUTY_0;
    }
}

HAL_StatusTypeDef WS2812_Show(void)
{
    if (ws2812_busy) {
        return HAL_BUSY;
    }

    ws2812_busy = 1U;
    return HAL_TIM_PWM_Start_DMA(&WS2812_TIM_HANDLE, WS2812_TIM_CHANNEL,
                                 ws2812_pwm_data, WS2812_BUFFER_SIZE);
}

uint8_t WS2812_IsBusy(void)
{
    return ws2812_busy;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == WS2812_TIM_INSTANCE) {
        HAL_TIM_PWM_Stop_DMA(htim, WS2812_TIM_CHANNEL);
        ws2812_busy = 0U;
    }
}
```

このテンプレートは、元プロジェクトより 1 点だけ強化しています。

- `busy` フラグで送信中の二重発行を防ぐ

別プロジェクトへ移植する場合、この形を基準にする方が安全です。

補足:

- HAL の `HAL_TIM_PWM_Start_DMA()` は `uint32_t *` を受けるため、バッファ型を `uint32_t[]` にしておくと扱いが素直です
- 実際の CCR レジスタは 16bit 幅でも問題ないことが多いですが、バッファ型と DMA 設定はそろえてください

## アプリ層の使い方

WS2812 ドライバは「バッファを作る関数」と「送信する関数」を分けて使います。

基本手順:

1. `WS2812_Clear()` で全消灯状態にする
2. `WS2812_SetPixel()` を必要回数呼ぶ
3. `WS2812_Show()` で一括送信する

例: 全 LED を緑にする

```c
WS2812_Clear();
for (uint16_t i = 0; i < WS2812_LED_COUNT; i++) {
    WS2812_SetPixel(i, 0, 255, 0);
}
WS2812_Show();
```

例: 1 個だけ流す

```c
for (uint16_t i = 0; i < WS2812_LED_COUNT; i++) {
    WS2812_Clear();
    WS2812_SetPixel(i, 255, 255, 0);
    while (WS2812_Show() == HAL_BUSY) {
    }
    HAL_Delay(100);
}
```

## AI エージェント向けの実装契約

AI エージェントがこの文書を使って別プロジェクトへ適用する場合、次の順序で進めると再現性が高くなります。

1. `.ioc`、`main.c`、`stm32xx_hal_msp.c` を読んで、使用可能なタイマ / チャネル / DMA / ピンを確定する
2. `f_tim` を計算または確認する
3. `ARR`、`duty_0`、`duty_1`、`reset_slots` を式に基づいて決める
4. `ws2812.h/.c` を追加または更新する
5. DMA IRQ と `HAL_TIM_PWM_PulseFinishedCallback()` の経路を確認する
6. 全消灯、単色、1 個流しの 3 パターンを最低限の検証コードとして用意する

AI エージェントが最終成果物として残すべきものは次のとおりです。

- 使用したタイマ / チャネル / ピン / DMA の明記
- クロック根拠
- 導出した `ARR` と比較値
- 実装ファイル一覧
- 動作確認用サンプル

## 実機検証チェックリスト

再現性を担保するには、コードだけでなく波形確認が重要です。

- オシロまたはロジアナで 1bit 周期が約 `1.25us` か確認する
- `0` と `1` で High 幅が変わっているか確認する
- 転送の最後に十分な Low 区間があるか確認する
- 全 LED 消灯、全 LED 白、流れるパターンの 3 つを確認する
- 連続送信時に色化けや欠けが出ないか確認する

## よくある失敗

- `RGB` 順で送ってしまい、色が入れ替わる
- タイマクロックを `PCLK` と誤認し、`ARR` 計算を間違える
- DMA IRQ を有効化しておらず、転送完了処理が走らない
- 送信中に `WS2812_Show()` を重ねて呼び、データが崩れる
- LED 数変更時にバッファ長を更新し忘れる
- 3.3V 出力で直接つないで不安定になる
- タイマや DMA を別機能と共有して競合する

## このリポジトリでの実例

`ARES9_IndicatorLED` では、上記の一般論を次の設定で実装しています。

### 使用リソース

- タイマ: `TIM2`
- チャネル: `CH4`
- 出力ピン: `PA3`
- GPIO AF: `GPIO_AF1_TIM2`
- DMA: `DMA1_Channel7`
- DMA リクエスト: `TIM2_CH2/CH4`

### クロックとタイミング

- システムクロック: `64MHz`
- PLL: `HSI(8MHz) -> HSI/2 = 4MHz -> x16 = 64MHz`
- TIM2 クロック: `64MHz`
- `PSC = 0`
- `ARR = 79`
- 1bit 周期: `80 / 64MHz = 1.25us`
- `WS_BIT_0 = 26`
- `WS_BIT_1 = 52`

### バッファ設定

- `LED_COUNT = 16`
- `RESET_SLOTS = 60`
- `TOTAL_BUF_SIZE = (16 * 24) + 60 = 444`

### 実装ファイル

- ドライバ API: `Core/Inc/ws2812.h`
- ドライバ本体: `Core/Src/ws2812.c`
- アプリ層利用例: `Core/Src/main.c`
- GPIO / DMA 関連 MSP: `Core/Src/stm32f3xx_hal_msp.c`
- DMA ISR: `Core/Src/stm32f3xx_it.c`
- CubeMX 設定: `ARES9_IndicatorLED.ioc`

## この文書を使うときの最小判断フロー

別の STM32 プロジェクトで再利用するなら、まず次だけ判断してください。

1. `TIM PWM + DMA` を使えるタイマチャネルがあるか
2. そのタイマの実クロックはいくつか
3. LED 数はいくつか
4. `ARR` と `duty_0 / duty_1` をどう置くか
5. DMA 完了後に止める設計にするか、循環 DMA にするか

通常の表示用途であれば、この文書の単発送信方式をそのまま採用すれば十分です。
