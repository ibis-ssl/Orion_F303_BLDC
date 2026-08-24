# Builds the relocated BLDC application and OTA metadata.
param([ValidateSet("Debug","Release")][string]$Configuration="Debug",[string]$ToolchainBin="C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin",[switch]$Rebuild)
$ErrorActionPreference="Stop";$root=Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path);$buildArgs=@{Configuration=$Configuration};if($Rebuild){$buildArgs.Rebuild=$true};& (Join-Path $root "Script\build.ps1") @buildArgs
$elf=Join-Path $root "$Configuration\Orion_F303_BLDC.elf";$bin=Join-Path $root "$Configuration\Orion_F303_BLDC_app.bin";$meta=Join-Path $root "$Configuration\Orion_F303_BLDC_app.metadata.bin";$map=Join-Path $root "$Configuration\Orion_F303_BLDC.map"
if($null-eq(Select-String -Path $map -Pattern '^FLASH\s+0x08004000\s+0x0001b000'|Select-Object -First 1)){throw "Application linker region mismatch"}
& (Join-Path $ToolchainBin "arm-none-eabi-objcopy.exe") -O binary $elf $bin;if($LASTEXITCODE){throw "objcopy failed"};& python (Join-Path $root "Script\generate_boot_metadata.py") $bin $meta;if($LASTEXITCODE){throw "metadata failed"}
