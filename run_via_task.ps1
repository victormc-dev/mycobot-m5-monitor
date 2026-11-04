# Workaround runner for this machine.
#
# Problem: the WorkBuddy tool session injects BOTH "HTTP_PROXY" and "http_proxy"
# (and HTTPS_PROXY / https_proxy). Windows env names are case-insensitive, so the
# .NET environment block becomes invalid and CreateProcess crashes with
# AccessViolationException in PowerShell / Bash tool hosts.
#
# Solution: let the Windows Task Scheduler service create the process instead
# (its environment is clean). The task is registered on demand and removed after.
#
# Usage (from PowerShell):
#   .\run_via_task.ps1 -Command ".venv\Scripts\python.exe check_env.py"
#   .\run_via_task.ps1 -Command ".venv\Scripts\python.exe examples\01_led_blink.py --port COM3" -TimeoutSec 120
#
# The command runs with the project root as working directory.

param(
    [Parameter(Mandatory = $true)][string]$Command,
    [string]$Log = "",
    [int]$TimeoutSec = 90
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$tmp = Join-Path $root ".wheels"
if (-not (Test-Path $tmp)) { New-Item -ItemType Directory -Path $tmp | Out-Null }
if ([string]::IsNullOrWhiteSpace($Log)) { $Log = Join-Path $tmp "runner_log.txt" }

$cmdFile = Join-Path $tmp "runner_cmd.cmd"
$marker = "WB_RUNNER_DONE"
@(
    "@echo off",
    "cd /d $root",
    $Command,
    "echo $marker"
) | Set-Content -Path $cmdFile -Encoding ASCII

if (Test-Path $Log) { Remove-Item $Log -Force }

$name = "WB_MyCobot_Runner"
$arg = '/c ' + $cmdFile + ' > ' + $Log + ' 2>&1'
$action = New-ScheduledTaskAction -Execute "cmd.exe" -Argument $arg
Register-ScheduledTask -TaskName $name -Action $action -Force | Out-Null

try {
    Start-ScheduledTask -TaskName $name
    $done = $false
    for ($i = 0; $i -lt $TimeoutSec; $i++) {
        if (Test-Path $Log) {
            $text = Get-Content $Log -Raw -Encoding Default -ErrorAction SilentlyContinue
            if ($text -and $text -match $marker) { $done = $true; break }
        }
        Start-Sleep -Seconds 1
    }
    if (Test-Path $Log) { Get-Content $Log -Raw -Encoding Default }
    Write-Output ("RUNNER_DONE=" + $done)
}
finally {
    Unregister-ScheduledTask -TaskName $name -Confirm:$false -ErrorAction SilentlyContinue
}
