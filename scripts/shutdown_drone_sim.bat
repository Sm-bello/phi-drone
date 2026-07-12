@echo off
echo ============================================
echo   PHI-Drone-Sim - Shutdown Sequence
echo ============================================
echo.

echo [1/2] Killing FlightGear...
taskkill /IM fgfs.exe /F 2>nul

echo [2/2] Killing SITL processes in WSL2...
wsl -d Ubuntu bash -c "pkill -f sim_vehicle.py; pkill -f arducopter; pkill -f mavproxy.py" 2>nul

echo.
echo All simulation processes terminated.
echo.
pause