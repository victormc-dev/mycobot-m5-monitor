@echo off
cd /d C:\Users\ThinkPad\Documents\MyCobot
.venv\Scripts\python.exe joint_monitor.py %*
if errorlevel 1 pause
