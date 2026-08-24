param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",

  [switch]$Rebuild
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$buildScript = Join-Path $scriptDir "build.ps1"
$flashScript = Join-Path $scriptDir "flash.ps1"

$buildArgs = @{
  Configuration = $Configuration
}
if ($Rebuild) {
  $buildArgs.Rebuild = $true
}

& $buildScript @buildArgs
if ($LASTEXITCODE -ne 0) {
  throw "Build step failed"
}

& $flashScript -Configuration $Configuration
if ($LASTEXITCODE -ne 0) {
  throw "Flash step failed"
}
