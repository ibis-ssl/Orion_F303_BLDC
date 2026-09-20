# SWDを1 MHzに設定し、アプリとメタデータの書き込み・接続確認を行う。
param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",

  [string]$ProgrammerPath = "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",

  [switch]$List,
  [switch]$ConnectOnly,
  [switch]$NoVerify,
  [switch]$NoReset
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$appPath = Join-Path $repoRoot "$Configuration\Orion_F303_BLDC_app.bin"
$metadataPath = Join-Path $repoRoot "$Configuration\Orion_F303_BLDC_app.metadata.bin"
$logDir = Join-Path $repoRoot ("Script\Logs\app_flash_" + (Get-Date -Format "yyyyMMdd_HHmmss"))
$backupPath = Join-Path $logDir "flash_before.bin"
$connection = "port=SWD mode=UR freq=1000"

if (-not (Test-Path $ProgrammerPath)) {
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
  & $ProgrammerPath "-c" $connection "-rst"
  if ($LASTEXITCODE -ne 0) {
    throw "Target connection failed"
  }
  exit 0
}

if (-not (Test-Path $appPath)) {
  throw "Application binary not found: $appPath"
}

if (-not (Test-Path $metadataPath)) {
  throw "Application metadata not found: $metadataPath"
}

New-Item -ItemType Directory -Path $logDir -Force | Out-Null

& $ProgrammerPath "-c" $connection "-u" "0x08000000" "0x20000" $backupPath
if ($LASTEXITCODE -ne 0) {
  throw "Flash backup failed for $Configuration"
}

$appArgs = @(
  "-c", $connection,
  "-w", $appPath, "0x08004000"
)

if (-not $NoVerify) {
  $appArgs += "-v"
}

& $ProgrammerPath @appArgs
if ($LASTEXITCODE -ne 0) {
  throw "Application flash failed for $Configuration"
}

$metadataArgs = @(
  "-c", $connection,
  "-w", $metadataPath, "0x08003800"
)

if (-not $NoVerify) {
  $metadataArgs += "-v"
}

if (-not $NoReset) {
  $metadataArgs += "-rst"
}

& $ProgrammerPath @metadataArgs
if ($LASTEXITCODE -ne 0) {
  throw "Metadata flash failed for $Configuration"
}

Write-Output "Application and metadata flashed; backup: $backupPath"

