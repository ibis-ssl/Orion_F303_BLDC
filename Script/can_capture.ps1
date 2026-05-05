param(
  [int]$Bitrate = 1000000,
  [int]$DurationSec = 10,
  [string]$CsvPath = "",
  [switch]$ListenOnly,
  [switch]$DebugLog
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$cli = Join-Path $scriptDir "canable_cli.py"
$args = @($cli, "--bitrate", $Bitrate)

if ($DebugLog) {
  $args += "--verbose"
}

$args += @("capture", "--duration", $DurationSec)
if ($CsvPath -ne "") {
  $args += @("--csv", $CsvPath)
}
if ($ListenOnly) {
  $args += "--listen-only"
}

python @args
