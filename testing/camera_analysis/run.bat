@echo off
setlocal enabledelayedexpansion

:: ── Camera index (change if wrong camera opens) ───────────────────────────────
set CAMERA_INDEX=0

:: ─────────────────────────────────────────────────────────────────────────────
echo [1/3] Looking for OpenCV...

set OPENCV_DIR=
for %%D in (
    "C:\opencv"
    "C:\Program Files\opencv"
    "C:\Program Files (x86)\opencv"
    "%USERPROFILE%\Desktop\opencv"
    "%USERPROFILE%\Desktop\Other\opencv"
    "%USERPROFILE%\Downloads\opencv"
    "%USERPROFILE%\opencv"
) do (
    if exist "%%~D\build\x64\vc16\lib\OpenCVConfig.cmake" (
        set OPENCV_DIR=%%~D\build\x64\vc16\lib
    )
    if exist "%%~D\build\x64\vc17\lib\OpenCVConfig.cmake" (
        set OPENCV_DIR=%%~D\build\x64\vc17\lib
    )
)

if "%OPENCV_DIR%"=="" (
    echo Could not find OpenCV automatically.
    echo Please enter the full path to the folder containing OpenCVConfig.cmake:
    set /p OPENCV_DIR="OpenCV path: "
)

if not exist "%OPENCV_DIR%\OpenCVConfig.cmake" (
    echo ERROR: OpenCVConfig.cmake not found at: %OPENCV_DIR%
    pause
    exit /b 1
)

echo Found OpenCV at: %OPENCV_DIR%

:: ─────────────────────────────────────────────────────────────────────────────
echo [2/3] Building...

cmake -B build -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR="%OPENCV_DIR%" -Wno-dev
if errorlevel 1 (
    echo.
    echo CMake configuration failed. Check errors above.
    pause
    exit /b 1
)

cmake --build build --config Release
if errorlevel 1 (
    echo.
    echo Build failed. Check errors above.
    pause
    exit /b 1
)

:: ─────────────────────────────────────────────────────────────────────────────
echo [3/3] Running... (output saved to log.txt)
echo.
build\Release\trashcan.exe %CAMERA_INDEX% > log.txt 2>&1

echo Done. Output saved to log.txt
pause