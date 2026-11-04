# One-shot setup for myCobot 280 M5 local environment (Windows PowerShell 5.1)
# Usage:
#   powershell -ExecutionPolicy Bypass -File .\setup.ps1
#   powershell -ExecutionPolicy Bypass -File .\setup.ps1 -PyExe "C:\Path\To\python.exe"

param(
    [string]$PyExe = "",
    [string]$Mirror = "https://pypi.tuna.tsinghua.edu.cn/simple",
    [switch]$SkipCheck
)

$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"
$Root = Split-Path -Parent $MyInvocation.MyCommand.Path

if ([string]::IsNullOrWhiteSpace($PyExe)) {
    $cmd = Get-Command python -ErrorAction SilentlyContinue
    if ($null -eq $cmd) {
        throw "python not found in PATH. Install Python 3.10+ and check 'Add Python to PATH', or pass -PyExe."
    }
    $PyExe = $cmd.Source
}

Write-Host "[1/4] Python : $PyExe"
& $PyExe --version
if ($LASTEXITCODE -ne 0) { throw "python failed to start" }

$Venv = Join-Path $Root ".venv"
$Vpy = Join-Path $Venv "Scripts\python.exe"
if (Test-Path $Vpy) {
    Write-Host "[2/4] venv already exists : $Venv"
} else {
    Write-Host "[2/4] Creating venv : $Venv"
    & $PyExe -m venv $Venv
    if ($LASTEXITCODE -ne 0) { throw "venv creation failed" }
}

Write-Host "[3/4] Installing requirements from $Mirror"
& $Vpy -m pip install --upgrade pip -i $Mirror
& $Vpy -m pip install -r (Join-Path $Root "requirements.txt") -i $Mirror
if ($LASTEXITCODE -ne 0) { throw "pip install failed" }

if (-not $SkipCheck) {
    Write-Host "[4/4] Running environment check"
    & $Vpy (Join-Path $Root "check_env.py")
} else {
    Write-Host "[4/4] Skipped (SkipCheck)"
}

Write-Host ""
Write-Host "DONE. Activate with: .\.venv\Scripts\Activate.ps1"
Write-Host "      Quick check : .\.venv\Scripts\python.exe check_env.py --port COM3"
