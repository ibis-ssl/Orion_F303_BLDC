$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
$Python = (Get-Command python -ErrorAction SilentlyContinue).Source
if (-not $Python) {
    throw "python not found. Add Python 3 to PATH."
}

& $Python "$RepoRoot\sim\sim_gui.py"
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}
