@echo off
cd /d "%~dp0"
python -m PyInstaller ^
  --noconfirm ^
  --clean ^
  --onefile ^
  --windowed ^
  --name NiclaRoiApp ^
  nicla_roi_app_gui.py
echo.
echo EXE:
echo %CD%\dist\NiclaRoiApp.exe
pause
