param(
  [Parameter(Mandatory = $true)]
  [ValidateSet(0, 1)]
  [int]$Motor,

  [Parameter(Mandatory = $true)]
  [float]$Rps,

  [int]$Board = 1,
  [int]$Bitrate = 1000000,
  [int]$Count = 1,
  [double]$PeriodSec = 0.0,
  [switch]$DebugLog
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$cli = Join-Path $scriptDir "canable_cli.py"
$args = @($cli, "--bitrate", $Bitrate)

if ($DebugLog) {
  $args += "--verbose"
}

$args += @(
  "send-speed",
  "--board", $Board,
  "--motor", $Motor,
  "--rps", $Rps,
  "--count", $Count,
  "--period", $PeriodSec
)

python @args
