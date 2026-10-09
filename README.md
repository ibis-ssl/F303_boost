# F303_boost

STM32F303CBT6を使用した、Orion向け昇圧・キッカー・電源監視基板のファームウェアです。

## 開発環境

- STM32CubeIDE 1.17.0
- STM32Cube FW_F3 V1.11.5
- STM32CubeProgrammer / STM32CubeCLT
- ターゲット: STM32F303CBT6

## VS Codeでのコードブラウズ

このリポジトリのルートフォルダーをVS Codeで開き、MicrosoftのC/C++拡張機能（`ms-vscode.cpptools`）を使用します。`.vscode/settings.json`でIntelliSenseを有効にし、解析の競合を避けるため、このワークスペースではSTM32Cube clangdを無効にしています。

`.vscode/c_cpp_properties.json`の構成は次のとおりです。

- `STM32F303 Debug`: アプリケーション用。DebugのMakefileと同じインクルードパス、`DEBUG`、`USE_HAL_DRIVER`、`STM32F303xC`を使用します。
- `STM32F303 Bootloader`: `Bootloader/Src`用。ブートローダーのMakefileと同じインクルードパスと`STM32F303xC`を使用します。HALとアプリケーション側のヘッダーを混在させません。

両構成ともGNU C11、Cortex-M4、ハードウェア浮動小数点（`fpv4-sp-d16` / `hard`）で解析します。標準ライブラリのヘッダーとコンパイラ組み込みマクロは、インストール済みの`C:/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe`から取得します。別の環境では同JSONの`env.stm32Compiler`を実際のコンパイラパスへ変更してください。

設定変更後はコマンドパレットから`Developer: Reload Window`を実行してください。通常は`STM32F303 Debug`を選び、ブートローダーを読む際は`C/C++: Select a Configuration...`で構成を切り替えます。定義ジャンプは`F12`、定義のプレビューは`Alt+F12`、参照検索は`Shift+F12`です。古い解析結果が残る場合は`C/C++: Reset IntelliSense Database`を実行してください。

インクルードパスやマクロをCubeMXまたはMakefileで変更した場合は、この解析設定も更新してください。設定項目の詳細は[VS Code公式リファレンス](https://code.visualstudio.com/docs/cpp/customize-cpp-settings)を参照してください。

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

詳細は [doc/overview.md](doc/overview.md) と [doc/hardware_spec.md](doc/hardware_spec.md) を参照してください。
# OTAビルド

CAN OTA用のアプリと常駐ブートローダーは次の順で生成します。

```powershell
.\Script\build_bootloader.ps1
.\Script\build_application.ps1 -Configuration Debug
.\Script\install_bootloader.ps1
```

最後のコマンドは既定でバックアップだけを行います。実機へ書き込む場合だけ、安全状態を確認した上で`-Execute`を追加します。

ブートローダーとDebugアプリケーションをビルドし、メタデータとともに一括で書き込む場合:

```powershell
.\Script\flash_all.ps1
```

このコマンドは書き込み前に現在のFlashを`Script/Logs/ota_install_日時/flash_before.bin`へバックアップし、全Flashを消去して書き込みと検証を行った後、MCUをリセットします。実機の出力が安全な状態で実行してください。
