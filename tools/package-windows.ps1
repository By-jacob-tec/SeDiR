# Builds SeDiR and puts it with the receiver libraries it needs into
# dist\SeDiR-<version>-windows-x64.zip, which runs without installing anything.
#
# Libraries come from PothosSDR (https://downloads.myriadrf.org/builds/PothosSDR/).
# For RTL-SDR the RTL-SDR Blog build of rtlsdr.dll is preferred (it also drives the
# V4 dongle); give its path with -RtlSdrDll, otherwise PothosSDR's own is used.
#
#   powershell -ExecutionPolicy Bypass -File tools\package-windows.ps1 [-RtlSdrDll path]

param(
    [string]$Pothos = "$env:ProgramFiles\PothosSDR",
    [string]$RtlSdrDll = ""
)
$ErrorActionPreference = "Stop"
$root = Split-Path $PSScriptRoot -Parent
Set-Location $root

$version = (Select-String -Path Cargo.toml -Pattern '^version\s*=\s*"(.+)"').Matches[0].Groups[1].Value
$name = "SeDiR-$version-windows-x64"
$out = Join-Path $root "dist\$name"

cargo build --release
if ($LASTEXITCODE -ne 0) { throw "build failed" }

if (Test-Path $out) { Remove-Item -Recurse -Force $out }
$modules = Join-Path $out "lib\SoapySDR\modules0.8"
New-Item -ItemType Directory -Force $modules | Out-Null

Copy-Item target\release\sedir.exe $out
$bin = Join-Path $Pothos "bin"
# SoapySDR and its modules for RTL-SDR and HackRF; libiio for Pluto and LibreSDR;
# libusb, pthreads and the Visual C++ runtime they are built against.
foreach ($dll in "SoapySDR.dll", "hackrf.dll", "libiio.dll", "libusb-1.0.dll", "pthreadVC2.dll",
                 "vcruntime140.dll", "vcruntime140_1.dll", "msvcp140.dll") {
    Copy-Item (Join-Path $bin $dll) $out
}
if (-not $RtlSdrDll) { $RtlSdrDll = Join-Path $bin "rtlsdr.dll" }
Copy-Item $RtlSdrDll (Join-Path $out "rtlsdr.dll")
foreach ($m in "rtlsdrSupport.dll", "HackRFSupport.dll") {
    Copy-Item (Join-Path $Pothos "lib\SoapySDR\modules0.8\$m") $modules
}

Copy-Item LICENSE (Join-Path $out "LICENSE.txt")
Copy-Item tools\THIRD-PARTY.txt $out
Copy-Item README.md (Join-Path $out "README.md")

$zip = Join-Path $root "dist\$name.zip"
if (Test-Path $zip) { Remove-Item $zip }
Compress-Archive -Path $out -DestinationPath $zip
Write-Host "Packaged $zip"
