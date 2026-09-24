# Orion_F303_BLDC ハードウェア仕様

本ドキュメントは、現行ファームウェアのハードウェア設定を整理したものです。


## MCU/クロック
- MCU: STM32F303
- コア: Cortex-M4F
- システムクロック: HSE 8MHz、PLL x12、SYSCLK/HCLK 96MHz
- APB1: 24MHz、APB2: 48MHz
- 主制御周期: 1ms
- PWM/制御割り込み: 30kHz（M0/M1交互処理のため各モーター15kHz）

## モータ駆動
- 対象: 2モータ（M0/M1）
- PWM: TIM1 と TIM8 の 3相コンプリメンタリ出力（CH1-3 + CH1N-3N）
- PWM 設定: `Prescaler=1`, `Period=1600`, `DeadTime=10`
- PWM周波数: 30kHz
- フリーウィール制御: CH/CHN の有効無効を直接切り替え

## エンコーダ
- 種別: MT6835（SPI1、21bit角度、3bit status、CRC-8）
- センサ数: 2個（M0 CS: PB7、M1 CS: PB6）
- SPI1: Mode 3、SCK 12MHz、8bit、MSB first
- 角度: 21bit値を反転して制御に使用

## アナログ計測
- ADC: ADC1/ADC2/ADC3 を併用
- 計測対象:
  - バッテリ電圧
  - ゲートドライバ DCDC 電圧
  - モータ電流 x2
  - モータ温度 x2
  - FET温度 x2
- 電流センサ:
  - ZXCT1084（オフセット 0）
  - INA199（オフセット 2048）
  を起動時の生ADC値で切り替える実装

## 通信
- CAN: 1ch 使用（PA11/PA12）
  - 設定値: Prescaler=3、BS1=5TQ、BS2=2TQ、SJW=1TQ、1Mbps
  - フィルタで速度指令、電源有効、キャリブ開始などを受信
- UART1: 2,000,000 bps（PC4/PC5）
  - DMA送信バッファを使ったデバッグ出力

## GPIO（主な用途）
- スイッチ入力: PC0-PC3（Pull-up, Low active）
- LED出力: PC13-PC15
- エンコーダCS: PB6/PB7

## 電源系のしきい値（ソフト保護）
- 過電流/過熱/低電圧/過電圧/過負荷を監視し、異常時はPWM停止＋リセット
- しきい値実体は `control_limits.h` を参照
