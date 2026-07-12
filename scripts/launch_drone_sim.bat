@echo off
setlocal enabledelayedexpansion

echo ============================================
echo   PHI-Drone-Sim Pipeline - Startup Sequence
echo ============================================
echo.

:: ============================================================
:: 0. Cleanup old processes
:: ============================================================
echo [0/4] Cleaning up old processes...
taskkill /IM fgfs.exe /F 2>nul
wsl -d Ubuntu bash -c "pkill -f sim_vehicle.py; pkill -f arducopter; pkill -f mavproxy.py" 2>nul
timeout /t 3 >nul

:: ============================================================
:: 1. FlightGear (visual anchor) - WITH DAYTIME FIX
:: ============================================================
echo [1/4] Launching FlightGear (Daytime Fixed)...
start "FlightGear" "C:\Program Files\FlightGear 2024.1\bin\fgfs.exe" ^
--fg-aircraft="C:\FlightGearAircraft" ^
--aircraft=arducopter ^
--airport=KSFO ^
--native-fdm=socket,in,30,0.0.0.0,5503,udp ^
--fdm=external ^
--geometry=900x650 ^
--timeofday=noon ^
--time-offset=0 ^
--disable-hud-3d ^
--disable-horizon-effect ^
--disable-random-objects ^
--disable-ai-models ^
--fog-disable ^
--wind=0@0 ^
--disable-terrasync ^
--disable-real-weather-fetch ^
--prop:/sim/time/utc-offset=0 ^
--prop:/environment/visibility=40000 ^
--prop:/sim/weather/enabled=0 ^
--prop:/sim/rendering/precipitation-enable=false ^
--prop:/sim/time/frozen=true

echo.
echo Waiting 45 seconds for FlightGear to fully load scenery...
echo (This may take longer on first run while terrain caches build)
timeout /t 45 >nul

:: ============================================================
:: 2. SITL + MAVProxy (Your working command)
:: ============================================================
echo [2/4] Launching ArduPilot SITL in WSL2...
start "SITL + MAVProxy" wsl -d Ubuntu bash -c "cd ~/ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py --map --console --enable-fgview -A '--fg 127.0.0.1'"

echo.
echo Waiting 30 seconds for SITL to initialize and sync with FlightGear...
timeout /t 30 >nul

:: ============================================================
:: 3. Detection Pipeline (Anaconda)
:: ============================================================
echo [3/4] Launching YOLOv8 Detection + Telemetry Logger...
start "Detection Pipeline" cmd /k "conda activate aerospace && cd /d C:\Users\User\Desktop\COMPLETED_PROJECTS\Drone_Sim && python detect_and_log_unified.py"

echo.
:: ============================================================
:: 4. Instructions
:: ============================================================
echo ============================================
echo All systems launched successfully!
echo ============================================
echo.
echo NEXT STEPS:
echo.
echo 1. FLIGHTGEAR: Press 'v' repeatedly until you see a downward view
echo    (Cockpit, Helicopter, or Down/Nadir view)
echo.
echo 2. MAVPROXY CONSOLE: Enter these commands:
echo    mode guided
echo    arm throttle
echo    takeoff 10
echo    guided -35.363 149.170 30
echo.
echo 3. DETECTION PREVIEW: Confirm it shows the FlightGear scene
echo    (not desktop icons), then press 'c' to start recording.
echo.
echo 4. CHECK OUTPUTS: Detections saved to:
echo    %USERPROFILE%\Desktop\COMPLETED_PROJECTS\Drone_Sim\detections\
echo.
echo ============================================
echo.
echo To shutdown all processes, close this window
echo or run shutdown_drone_sim.bat
echo.
pause