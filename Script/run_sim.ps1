param(
    [ValidateSet("conventions", "step")]
    [string]$Scenario = "conventions",
    [ValidateSet("raw_pos_add", "raw_pos_sub", "raw_neg_add", "raw_neg_sub", "legacy")]
    [string]$AngleMode = "raw_neg_add",
    [double]$SpeedRps = 20.0,
    [double]$DurationSec = 1.0,
    [double]$Zero = 1.2,
    [ValidateSet("-1", "1")]
    [string]$EncoderDirection = "-1",
    [double]$PhaseTrimDeg = 0.0,
    [string]$OutputCsv = "",
    [switch]$SelfTest
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
$Python = (Get-Command python -ErrorAction SilentlyContinue).Source
if (-not $Python) {
    throw "python not found. Add Python 3 to PATH."
}

$ArgsList = @("$RepoRoot\sim\bldc_sim.py")
if ($SelfTest) {
    $ArgsList += "--self-test"
} else {
    $ArgsList += @("--scenario", $Scenario)
    $ArgsList += @("--angle-mode", $AngleMode)
    $ArgsList += @("--speed-rps", $SpeedRps.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--duration-s", $DurationSec.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--zero", $Zero.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--encoder-direction", $EncoderDirection)
    $ArgsList += @("--phase-trim-deg", $PhaseTrimDeg.ToString([Globalization.CultureInfo]::InvariantCulture))
    if ($OutputCsv -ne "") {
        $ArgsList += @("--output-csv", (Join-Path $RepoRoot $OutputCsv))
    }
}

& $Python @ArgsList
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
