# Backs up BLDC Flash and installs OTA images while preserving settings pages 62-63.
param([ValidateSet("Debug","Release")][string]$Configuration="Debug",[string]$ProgrammerPath="C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",[string]$ProbeSerial="",[switch]$Execute)
$ErrorActionPreference="Stop"
$root=Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path);$log=Join-Path $root ("Script\Logs\ota_install_"+(Get-Date -Format "yyyyMMdd_HHmmss"));New-Item -ItemType Directory -Path $log -Force|Out-Null
$boot=Join-Path $root "Bootloader\Build\Orion_F303_BLDC_bootloader.bin";$app=Join-Path $root "$Configuration\Orion_F303_BLDC_app.bin";$meta=Join-Path $root "$Configuration\Orion_F303_BLDC_app.metadata.bin";foreach($p in @($ProgrammerPath,$boot,$app,$meta)){if(-not(Test-Path $p)){throw "Required file not found: $p"}}
$connection="port=SWD mode=UR freq=4000";if($ProbeSerial){$connection+=" sn=$ProbeSerial"}
function Invoke-Programmer([object[]]$Arguments,[string]$Operation){
  for($attempt=1;$attempt -le 3;$attempt++){
    & $ProgrammerPath @Arguments
    if($LASTEXITCODE -eq 0){return}
    if($attempt -lt 3){Write-Warning "$Operation failed (attempt $attempt/3); retrying.";Start-Sleep -Milliseconds 250}
  }
  throw "$Operation failed after 3 attempts"
}
Invoke-Programmer -Arguments @("-c",$connection,"-u","0x08000000","0x20000",(Join-Path $log "flash_before.bin")) -Operation "Flash backup"
if(-not $Execute){Write-Output "Dry run completed; settings 0x0801F000-0x0801FFFF will be preserved.";exit 0}
for($firstSector=0;$firstSector -le 61;$firstSector+=8){
  $lastSector=[Math]::Min($firstSector+7,61)
  $eraseArguments=@("-c",$connection,"-e")+($firstSector..$lastSector)
  Invoke-Programmer -Arguments $eraseArguments -Operation "Application-area page erase $firstSector-$lastSector"
}
Invoke-Programmer -Arguments @("-c",$connection,"--skipErase","-w",$boot,"0x08000000","-v") -Operation "Bootloader write"
Invoke-Programmer -Arguments @("-c",$connection,"--skipErase","-w",$app,"0x08004000","-v") -Operation "Application write"
Invoke-Programmer -Arguments @("-c",$connection,"--skipErase","-w",$meta,"0x08003800","-v","-rst") -Operation "Metadata write"
Write-Output "BLDC OTA installation completed; settings pages were not erased."
