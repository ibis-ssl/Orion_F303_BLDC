param(
    [ValidateSet("conventions", "step", "snapshot", "realtime")]
    [string]$Scenario = "conventions",
    [ValidateSet("simple", "dq")]
    [string]$Model = "simple",
    [ValidateSet("raw_pos_add", "raw_pos_sub", "raw_neg_add", "raw_neg_sub", "legacy")]
    [string]$AngleMode = "raw_neg_add",
    [double]$SpeedRps = 20.0,
    [double]$SpeedRpsM0 = 20.0,
    [double]$SpeedRpsM1 = 20.0,
    [double]$DurationSec = 1.0,
    [double]$Zero = 1.2,
    [ValidateSet("-1", "1")]
    [string]$EncoderDirection = "-1",
    [int]$EncoderDelaySteps = 0,
    [double]$PhaseTrimDeg = 0.0,
    [string]$OutputCsv = "",
    [int]$Raw = 0,
    [double]$LegacyElec = [double]::NaN,
    [double]$ActualElec = [double]::NaN,
    [double]$UqV = -2.0,
    [double]$UdV = 0.0,
    [double]$BatteryV = 24.0,
    [int]$PwmPeriod = 1800,
    [double]$InitialThetaM0 = 0.3,
    [double]$InitialThetaM1 = 0.6,
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
    $ArgsList += @("--model", $Model)
    $ArgsList += @("--angle-mode", $AngleMode)
    $ArgsList += @("--speed-rps", $SpeedRps.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--speed-rps-m0", $SpeedRpsM0.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--speed-rps-m1", $SpeedRpsM1.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--duration-s", $DurationSec.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--zero", $Zero.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--encoder-direction", $EncoderDirection)
    $ArgsList += @("--encoder-delay-steps", $EncoderDelaySteps.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--phase-trim-deg", $PhaseTrimDeg.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--raw", $Raw.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--legacy-elec", $LegacyElec.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--actual-elec", $ActualElec.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--uq-v", $UqV.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--ud-v", $UdV.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--battery-v", $BatteryV.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--pwm-period", $PwmPeriod.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--initial-theta-m0", $InitialThetaM0.ToString([Globalization.CultureInfo]::InvariantCulture))
    $ArgsList += @("--initial-theta-m1", $InitialThetaM1.ToString([Globalization.CultureInfo]::InvariantCulture))
    if ($OutputCsv -ne "") {
        $ArgsList += @("--output-csv", (Join-Path $RepoRoot $OutputCsv))
    }
}

& $Python @ArgsList
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
