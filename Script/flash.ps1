param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",

  [string]$ProgrammerPath = "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",

  [string]$ArtifactName = "F303_boost",

  [switch]$List,
  [switch]$ConnectOnly,
  [switch]$NoVerify,
  [switch]$NoReset
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$elfPath = Join-Path $repoRoot "$Configuration\$ArtifactName.elf"

if (-not (Test-Path -LiteralPath $ProgrammerPath -PathType Leaf)) {
  throw "STM32_Programmer_CLI.exe not found: $ProgrammerPath"
}

if ($List) {
  & $ProgrammerPath "-l" "stlink"
  if ($LASTEXITCODE -ne 0) {
    throw "ST-Link listing failed"
  }
  exit 0
}

if ($ConnectOnly) {
  & $ProgrammerPath "-c" "port=SWD mode=UR" "-rst"
  if ($LASTEXITCODE -ne 0) {
    throw "Target connection failed"
  }
  exit 0
}

if (-not (Test-Path -LiteralPath $elfPath -PathType Leaf)) {
  throw "ELF not found: $elfPath"
}

$programmerArgs = @(
  "-c", "port=SWD mode=UR",
  "-w", $elfPath
)

if (-not $NoVerify) {
  $programmerArgs += "-v"
}

if (-not $NoReset) {
  $programmerArgs += "-rst"
}

& $ProgrammerPath @programmerArgs
if ($LASTEXITCODE -ne 0) {
  throw "Flash failed for $Configuration"
}
