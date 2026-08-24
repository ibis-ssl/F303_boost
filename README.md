# F303_boost

STM32F303CBT6を使用した、Orion向け昇圧・キッカー・電源監視基板のファームウェアです。

## 開発環境

- STM32CubeIDE 1.17.0
- STM32Cube FW_F3 V1.11.5
- STM32CubeProgrammer / STM32CubeCLT
- ターゲット: STM32F303CBT6

## ビルド

STM32CubeIDEでDebugまたはRelease構成を生成した後、PowerShellから次を実行します。

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\build.ps1
powershell -ExecutionPolicy Bypass -File .\Script\build.ps1 -Configuration Release -Rebuild
```

ビルド後にST-Link経由で書き込む場合:

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\build_and_flash.ps1
```

UARTログを確認する場合:

```powershell
powershell -ExecutionPolicy Bypass -File .\Script\monitor_uart.ps1 -Port COM60
```

昇圧PWMパラメータのシミュレーションとプロットを生成する場合:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Script\run_boost_sim.ps1
```

詳細は [doc/overview.md](doc/overview.md) と [doc/hardware_spec.md](doc/hardware_spec.md) を参照してください。
