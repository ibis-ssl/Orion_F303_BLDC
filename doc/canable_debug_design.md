# CANable デバッグCLI設計方針

## 目的

CANableを使い、ターゲットボードのCAN入出力をコマンドラインから確認できる最小デバッグ環境を作る。CangarooはGUIツールとして有用だがCLI機能はないため、独自CLIを実装する。

現状のCANableはWindows上で `canable gs_usb` として認識され、COMポートではなくUSBデバイスとして見えている。ドライバはlibusbへ切り替え済みの前提とする。

## 参照する実装

- candleLight firmware: CANable/CANtact/candleLight向けの `gs_usb` 互換ファームウェア。
- Linux `gs_usb` driver: SocketCANが使うホスト側プロトコル実装。
- CANable Getting Started: CANableのcandleLight firmwareはLinuxでは `can0` として扱われ、WindowsではCangarooが使えることを説明している。
- Cangaroo: Windows GUIではcandleLight/WinUSB対応があるが、CLIとして使う入口はない。

参考URL:

- https://github.com/candle-usb/candleLight_fw
- https://github.com/torvalds/linux/blob/master/drivers/net/can/usb/gs_usb.c
- https://canable.io/getting-started.html
- https://github.com/HubertD/cangaroo/releases

## 方針

Windows上でlibusb経由の `gs_usb` 最小ホスト実装を作る。SocketCANやCangarooへ依存せず、CANableのUSB control transferとbulk transferを直接扱う。

初期実装の範囲:

- CANableデバイス列挙。
- デバイス設定読み出し。
- 1ch目のCANビットタイミング設定。
- CANチャンネル開始/停止。
- クラシックCANフレーム受信。
- 標準ID/拡張IDのクラシックCANフレーム送信。
- ログ出力。
- ターゲットボード用の速度指令送信。

初期実装で扱わない範囲:

- CAN FD。
- ハードウェアフィルタ。
- ハードウェアタイムスタンプの高精度補正。
- 複数CANチャンネル。
- DBCデコード。
- GUI。

## 実装言語と配置

まずはPythonで実装する。理由は、プロトコル検証、ログ整形、送受信コマンド追加が速いためである。性能が不足する場合のみC実装へ移す。

配置案:

```text
Script/canable_cli.py
Script/can_capture.ps1
Script/can_send_speed.ps1
```

`canable_cli.py` はUSB/gs_usb本体を持つ。PowerShellスクリプトは引数の簡略化だけを担当する。

Python依存:

- `pyusb`
- Windows libusb backend

`python-can` は使わない。導入できれば参考にはなるが、今回の目的では直接実装の方が依存が少ない。

## USB/gs_usbプロトコル概要

CANable/candleLightはUSB vendor control requestで設定を行い、bulk endpointでCANフレームを送受信する。

既知のエンドポイント:

```text
Bulk IN   0x81
Bulk OUT  0x02
```

主なcontrol request:

```text
GS_USB_BREQ_HOST_FORMAT   = 0
GS_USB_BREQ_BITTIMING     = 1
GS_USB_BREQ_MODE          = 2
GS_USB_BREQ_BT_CONST      = 4
GS_USB_BREQ_DEVICE_CONFIG = 5
GS_USB_BREQ_TIMESTAMP     = 6
```

control transferの方向:

```text
OUT: USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_INTERFACE = 0x41
IN : USB_DIR_IN  | USB_TYPE_VENDOR | USB_RECIP_INTERFACE = 0xC1
```

起動シーケンス案:

1. VID/PID `1d50:606f` のUSBデバイスを開く。
2. configurationを設定し、interface 0をclaimする。
3. `HOST_FORMAT` でlittle endian指定を送る。
4. `DEVICE_CONFIG` を読み出し、channel数を確認する。
5. `BT_CONST` を読み出し、CANクロックと許容bit timingを確認する。
6. `MODE RESET` を送る。
7. `BITTIMING` を送る。
8. `MODE START` を送る。
9. Bulk INを継続的に読む。

## ターゲットボードのCAN設定

ターゲットファームのCAN設定:

```text
Prescaler = 4
BS1       = 4TQ
BS2       = 4TQ
SJW       = 1TQ
```

STM32F303のCANクロックが36MHzの場合、ビットレートは以下になる。

```text
bitrate = 36MHz / (4 * (1 + 4 + 4)) = 1Mbps
```

CANable側のbit timingは、まず以下を試す。

```text
prop_seg   = 0
phase_seg1 = 4
phase_seg2 = 4
sjw        = 1
brp        = 4
```

もしCANableの `BT_CONST` が `prop_seg=0` を許容しない場合は、同じ合計TQになる次の候補へ切り替える。

```text
prop_seg   = 1
phase_seg1 = 3
phase_seg2 = 4
sjw        = 1
brp        = 4
```

## CANフレーム構造

クラシックCANのみ扱う。USB上の `gs_host_frame` は次の構造を基本にする。

```text
u32 echo_id
u32 can_id
u8  can_dlc
u8  channel
u8  flags
u8  reserved
u8  data[8]
```

サイズは20byte。little endianでpack/unpackする。

受信フレームの `echo_id` は通常 `0xffffffff`。送信フレームは `echo_id` に0から9程度の値を入れると、送信エコーが返る可能性がある。初期実装では受信通常フレームと送信エコーをログ上で区別する。

CAN IDフラグ:

```text
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
```

標準IDは `can_id & 0x7ff`、拡張IDは `CAN_EFF_FLAG | (id & 0x1fffffff)` とする。

## CLI仕様

キャプチャ:

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\can_capture.ps1 -Bitrate 1000000 -DurationSec 10
python .\Script\canable_cli.py --bitrate 1000000 capture --duration 10
```

出力例:

```text
0.001234 RX 200 8 00 00 80 3f 12 34 56 78
0.002000 RX 201 8 ...
```

ターゲット向け速度指令:

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\can_send_speed.ps1 -Board 1 -Motor 0 -Rps 10.0
powershell -ExecutionPolicy Bypass -File .\Script\can_send_speed.ps1 -Board 1 -Motor 1 -Rps -10.0
python .\Script\canable_cli.py --bitrate 1000000 send-speed --board 1 --motor 0 --rps 10.0
```

PowerShellラッパーで詳細ログを出す場合は、PowerShell共通パラメータ `-Verbose` との衝突を避けるため `-DebugLog` を使う。

送信ID:

```text
M0: 0x100 + board_id * 2
M1: 0x101 + board_id * 2
```

ペイロードは現行ファームの `can_msg_buf_t.value[0]` に合わせ、float32 little endianで `rps` を入れる。残り4byteは0にする。

## ログ形式

人間が見るテキストログと、後解析用CSVの両方を出せるようにする。

テキスト:

```text
time_s dir id dlc data_hex decoded
```

CSV:

```text
time_s,dir,id,dlc,b0,b1,b2,b3,b4,b5,b6,b7
```

ターゲットボードの既知IDは簡易デコードする。

```text
0x200 + board*2 + motor: speed, angle
0x210 + board*2 + motor: voltage
0x220 + board*2 + motor: motor_temp, fet_temp
0x230 + board*2 + motor: current
0x500 + board*2 + motor: rps_per_v, zero_angle
0x000: fault id/info/value
```

## エラー処理

- USBデバイスが見つからない場合はVID/PIDとドライバ状態を表示する。
- `BT_CONST` が読めない場合は対応外として停止する。
- bulk read timeoutは正常扱いにし、指定時間まで継続する。
- bus-offやerror frameは通常フレームと同じログへ出す。
- Ctrl+Cでは `MODE RESET` を送ってから終了する。

## 検証手順

1. USB列挙で `1d50:606f` を確認する。
2. `DEVICE_CONFIG` と `BT_CONST` を読めることを確認する。
3. `MODE START` 後、ターゲットボードの周期送信を受信する。
4. 受信IDが `0x202/0x203`、`0x212/0x213`、`0x222/0x223`、`0x232/0x233`、`0x502/0x503` 付近で出ることを確認する。board_idが0ならそれぞれ `0x200/0x201` からになる。
5. `send-speed` で低速指令を送り、UARTログ側の `CAN rx` が増えることを確認する。
6. スイッチ指令がCAN指令より優先されることを確認する。

## 実装上の注意

- CANableが既にCangaroo等に開かれている場合、libusbからclaimできない可能性がある。
- Windowsではlibusbドライバの種類によりPyUSBから見えないことがある。`usb.core.find()` で0件の場合はドライバ/バックエンドを先に確認する。
- ターゲットボード側はCAN受信フィルタをboard_idに依存して設定している。送信テスト前にUART `i` でboard_idを確認する。
- モーターを回す送信テストでは、最初は `1 rps` 程度から始め、UART `space` で停止できる状態にしておく。
