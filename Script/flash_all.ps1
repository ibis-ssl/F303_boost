# Builds and flashes the resident bootloader, application, and OTA metadata.
$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$buildBootloaderScript = Join-Path $scriptDir "build_bootloader.ps1"
$buildApplicationScript = Join-Path $scriptDir "build_application.ps1"
$installBootloaderScript = Join-Path $scriptDir "install_bootloader.ps1"

& $buildBootloaderScript
if ($LASTEXITCODE -ne 0) {
  throw "Bootloader build step failed"
}

& $buildApplicationScript -Configuration Debug
if ($LASTEXITCODE -ne 0) {
  throw "Application build step failed"
}

& $installBootloaderScript -Configuration Debug -Execute
if ($LASTEXITCODE -ne 0) {
  throw "Flash-all step failed"
}
