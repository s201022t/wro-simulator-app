@echo off
cd /d "%~dp0"
python -m PyInstaller ^
  --noconfirm ^
  --clean ^
  --windowed ^
  --name NiclaRoiApp ^
  nicla_roi_app_gui.py
echo.
echo Test build:
echo %CD%\dist\NiclaRoiApp\NiclaRoiApp.exe
pause
