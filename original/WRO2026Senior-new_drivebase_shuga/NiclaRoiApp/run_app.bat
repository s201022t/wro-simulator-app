@echo off
cd /d "%~dp0"
python nicla_roi_app_gui.py
if errorlevel 1 pause
