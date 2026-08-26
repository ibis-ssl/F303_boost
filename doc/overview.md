# F303_boost 概要

## 目的

本ファームウェアは、昇圧回路、ストレート／チップキッカー、電源出力、各種電圧・電流・温度の監視、およびADNS3080光学センサの読み出しを制御します。

## 主要ファイル

- `Core/Src/main.c`: 起動処理、CANコマンド処理、昇圧・キック制御、保護処理、周期処理。
- `Core/Src/can.c`: CAN初期化、受信フィルタ、テレメトリ送信。
- `Core/Src/adns3080.c`: ADNS3080の初期化と移動量読み出し。
- `Core/Src/adc.c`: 電圧・電流・温度計測用ADC設定。
- `Core/Src/tim.c`: 昇圧、キッカー、ゲートドライバ電源用PWMと周期計測。
- `orion_F303_boost_v3.ioc`: STM32CubeMXの周辺機能設定。

## 起動とメインループ

起動時にGPIO、ADC、CAN、SPI、PWM、UARTを初期化し、ADNS3080の接続確認と電源系の事前検査を行います。事前検査完了後にIWDGを開始します。

メインループは概ね1 ms周期で、次の順に処理します。

1. IWDGを更新する。
2. 約11 msごとにADNS3080を更新し、移動量をCAN送信する。
3. ADC値を更新する。
4. 低電圧、過電圧、過電流、ゲートドライバ電源、温度を検査する。
5. ユーザースイッチとUART表示を処理する。
6. CANテレメトリを順次送信する。
7. 電源有効コマンドのタイムアウトを更新する。
8. 異常または無効状態なら全出力を停止し、正常時だけキックと昇圧を制御する。

## CANインターフェース

Classical CAN、標準11 bit ID、1 Mbit/sを使用します。バイト列内の`float`はSTM32のlittle endian表現です。

### 受信

| CAN ID | 内容 |
| --- | --- |
| `0x000` | 緊急停止。充電許可を解除する。 |
| `0x001` | エラー時リセット。先頭2 byteが`5A A5`の場合にリセットする。 |
| `0x010` | 電源有効および保護パラメータ設定。 |
| `0x110` | 目標昇圧電圧、充電許可、キッカー選択、キック指令。 |

`0x110`ではbyte 0を項目番号として使用します。

- `0`: byte 4..7の`float`で目標昇圧電圧を指定する（20 V以上、450 V以下）。
- `1`: byte 1で充電を有効／無効にする。
- `2`: byte 1でチップ／ストレートキッカーを選択する。
- `3`: byte 1でキック強度を指定する。

### 送信

| CAN ID | 内容 |
| --- | --- |
| `0x000` | エラー情報。node ID、エラービット、関連値。 |
| `0x215` | バッテリ電圧（float、4 byte）。 |
| `0x216` | 昇圧電圧（float、4 byte）。 |
| `0x224` | FET温度、コイル1温度、コイル2温度。 |
| `0x234` | バッテリ電流（float、4 byte）。 |
| `0x241` | ADNS3080のX/Y移動量とquality。 |

## 補助スクリプト

- `Script/build.ps1`: CubeIDE付属GNU MakeでDebug／Releaseをビルドする。
- `Script/flash.ps1`: `F303_boost.elf`をSTM32CubeProgrammerで書き込む。
- `Script/build_and_flash.ps1`: ビルド成功後に書き込む。
- `Script/monitor_uart.ps1`: 2,000,000 baudのUARTログを表示し、必要に応じてファイルへ保存する。

書き込みは実機の出力が安全な状態で行ってください。`flash.ps1`は実行後にMCUをリセットします。
# CAN OTA更新

先頭16KBを常駐アプリケーションブートローダー、`0x08004000`～`0x0801F7FF`をアプリ領域、末尾2KBをmetadata領域とする。OTA node IDは100、応答CAN IDは`0x6B4`である。

アプリがCAN ID `0x600`、payload `OFWUP + node 100`を受信すると、PB2の電源許可をLowにし、TIM2昇圧PWM、TIM3キックPWM、TIM4負電源PWMを停止してからmetadataを消去しresetする。bootloaderもC runtime開始前に同じ出力をLowへ固定し、IWDGを継続refreshしながら更新する。32 frame software FIFO、896 byte block、bitmap、block CRC32C、全体CRC32Cにより通信異常と不完全imageを検出する。

初回導入は`Script/build_bootloader.ps1`、`Script/build_application.ps1`を実行後、`Script/install_bootloader.ps1`をdry-runし、バックアップを確認してから`-Execute`を指定する。高電圧部を含む実機の電源が使用できないため、現時点では実機書込み・安全出力・CAN更新は未確認である。

## 開発用FW識別

- アプリ先頭`0x08004000`から`+0x400`へ`FWVR` magicとUnix秒build IDを配置する。
- CAN ID `0x611`でnode 100を指定すると、`0x6C4`でbuild IDとmetadataのimage CRC32Cを返す。通常アプリとbootloaderの双方に実装した。
- アプリ／bootloaderビルド時は`Script/Logs/Build/`へGit hashとdirty状態をJSON保存する。2026-08-27時点ではアプリとbootloaderのビルド成功のみ確認し、実機への書込み・CAN応答試験は行っていない。
