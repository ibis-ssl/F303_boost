# Backs up power-board Flash and installs the resident bootloader, application, and metadata.
param([ValidateSet("Debug","Release")][string]$Configuration="Debug",[string]$ProgrammerPath="C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",[string]$ProbeSerial="",[switch]$Execute)
$ErrorActionPreference="Stop"
$root=Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path);$log=Join-Path $root ("Script\Logs\ota_install_"+(Get-Date -Format "yyyyMMdd_HHmmss"));New-Item -ItemType Directory -Path $log -Force|Out-Null
$boot=Join-Path $root "Bootloader\Build\F303_boost_bootloader.bin";$app=Join-Path $root "$Configuration\F303_boost_app.bin";$meta=Join-Path $root "$Configuration\F303_boost_app.metadata.bin";foreach($p in @($ProgrammerPath,$boot,$app,$meta)){if(-not(Test-Path $p)){throw "Required file not found: $p"}}
$connection="port=SWD mode=UR freq=1000";if($ProbeSerial){$connection+=" sn=$ProbeSerial"};& $ProgrammerPath -c $connection -u 0x08000000 0x20000 (Join-Path $log "flash_before.bin");if($LASTEXITCODE){throw "Flash backup failed"}
if(-not $Execute){Write-Output "Dry run completed. No Flash write was performed.";exit 0}
foreach($sector in (0..63)){& $ProgrammerPath -c $connection -e $sector;if($LASTEXITCODE){throw "Page erase failed: $sector"}}
& $ProgrammerPath -c $connection --skipErase -w $boot 0x08000000 -v;if($LASTEXITCODE){throw "Bootloader write failed"};& $ProgrammerPath -c $connection --skipErase -w $app 0x08004000 -v;if($LASTEXITCODE){throw "Application write failed"};& $ProgrammerPath -c $connection --skipErase -w $meta 0x0801F800 -v -rst;if($LASTEXITCODE){throw "Metadata write failed"};Write-Output "Power-board OTA installation completed."
