param(
  [string]$OutputDir = "sim\out",
  [double]$TargetV = 450.0,
  [double]$CapacitanceUf = 1360.0,
  [double]$InductanceUh = 10.0,
  [double]$FetResistanceOhm = 0.08,
  [double]$CoilResistanceOhm = 0.02,
  [double]$OffPathResistanceOhm = 0.02,
  [double]$DiodeDropV = 1.5
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
$python = (Get-Command python -ErrorAction SilentlyContinue).Source
if (-not $python) {
  throw "python not found. Add Python 3 to PATH."
}

$outputPath = Join-Path $repoRoot $OutputDir
$arguments = @(
  (Join-Path $repoRoot "sim\boost_sim.py"),
  "--output-dir", $outputPath,
  "--target-v", $TargetV.ToString([Globalization.CultureInfo]::InvariantCulture),
  "--capacitance-uf", $CapacitanceUf.ToString([Globalization.CultureInfo]::InvariantCulture),
  "--inductance-uh", $InductanceUh.ToString([Globalization.CultureInfo]::InvariantCulture),
  "--fet-resistance-ohm", $FetResistanceOhm.ToString([Globalization.CultureInfo]::InvariantCulture),
  "--coil-resistance-ohm", $CoilResistanceOhm.ToString([Globalization.CultureInfo]::InvariantCulture),
  "--off-path-resistance-ohm", $OffPathResistanceOhm.ToString([Globalization.CultureInfo]::InvariantCulture),
  "--diode-drop-v", $DiodeDropV.ToString([Globalization.CultureInfo]::InvariantCulture)
)

& $python @arguments
if ($LASTEXITCODE -ne 0) {
  exit $LASTEXITCODE
}
