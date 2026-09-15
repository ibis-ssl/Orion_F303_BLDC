# Flashes the bootloader, OTA metadata, and application in one operation.
param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug"
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$installScript = Join-Path $scriptDir "install_bootloader.ps1"

if (-not (Test-Path -LiteralPath $installScript)) {
  throw "install_bootloader.ps1 not found: $installScript"
}

& $installScript -Configuration $Configuration -Execute
