@echo off
rem Windows: double-click. The COM port is found automatically.
cd /d "%~dp0"
set PY=python
where py >nul 2>nul && set PY=py
%PY% dashboard.py --port auto --placeholder-geometry
pause
