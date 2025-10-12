@echo off
setlocal enabledelayedexpansion

echo ================================================
echo UniLabOS Environment Installation Script
echo ================================================
echo.

REM Get the directory where this script is located
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"

REM Find conda installation using 'where conda'
echo Searching for conda installation...
for /f "tokens=*" %%i in ('where conda 2^>nul') do (
    set "CONDA_PATH=%%i"
    goto :found_conda
)

echo ERROR: Could not find conda installation!
echo Please make sure conda/mamba is installed and in your PATH.
echo.
pause
exit /b 1

:found_conda
REM Extract base directory from conda path
REM Path looks like: C:\Users\10230\miniforge3\Library\bin\conda.bat
REM or: C:\Users\10230\miniforge3\Scripts\conda.exe
for %%i in ("%CONDA_PATH%") do set "CONDA_FILE=%%~nxi"
for %%i in ("%CONDA_PATH%") do set "CONDA_BASE=%%~dpi"

REM Go up two levels to get base directory
for %%i in ("%CONDA_BASE%..") do set "CONDA_BASE=%%~fi"
if "%CONDA_FILE%"=="conda.bat" (
    for %%i in ("%CONDA_BASE%..") do set "CONDA_BASE=%%~fi"
)

echo Found conda at: %CONDA_BASE%
echo.

REM Set target environment path
set "ENV_NAME=unilab"
set "ENV_PATH=%CONDA_BASE%\envs\%ENV_NAME%"

REM Check if environment already exists
if exist "%ENV_PATH%" (
    echo WARNING: Environment '%ENV_NAME%' already exists at %ENV_PATH%
    echo.
    set /p "OVERWRITE=Do you want to overwrite it? (y/n): "
    if /i not "!OVERWRITE!"=="y" (
        echo Installation cancelled.
        pause
        exit /b 0
    )
    echo Removing existing environment...
    rmdir /s /q "%ENV_PATH%"
)

REM Find the packed environment file
set "PACK_FILE="
for %%f in (unilab-env*.tar.gz) do (
    set "PACK_FILE=%%f"
    goto :found_pack
)

:found_pack
if "%PACK_FILE%"=="" (
    echo ERROR: Could not find unilab-env*.tar.gz file!
    echo Please make sure the packed environment file is in the same directory as this script.
    echo.
    pause
    exit /b 1
)

echo Found packed environment: %PACK_FILE%
echo.

REM Extract the packed environment
echo Extracting environment to %ENV_PATH%...
mkdir "%ENV_PATH%"

REM Extract using tar (available in Windows 10+)
tar -xzf "%PACK_FILE%" -C "%ENV_PATH%"
if errorlevel 1 (
    echo ERROR: Failed to extract environment!
    echo Make sure you have Windows 10 or later with tar support.
    pause
    exit /b 1
)

echo.
echo Unpacking conda environment...
echo Changing to environment directory: %ENV_PATH%
cd /d "%ENV_PATH%"

REM Run conda-unpack from the environment directory
if exist "Scripts\conda-unpack.exe" (
    echo Running: .\Scripts\conda-unpack.exe
    .\Scripts\conda-unpack.exe
) else if exist "Scripts\activate.bat" (
    echo Running: .\Scripts\activate.bat followed by conda-unpack
    call .\Scripts\activate.bat
    conda-unpack
) else (
    echo ERROR: Could not find Scripts\conda-unpack.exe or Scripts\activate.bat!
    echo Current directory: %CD%
    echo Expected location: %ENV_PATH%\Scripts\
    pause
    exit /b 1
)

if errorlevel 1 (
    echo ERROR: conda-unpack failed!
    pause
    exit /b 1
)

echo.
echo ================================================
echo Installation completed successfully!
echo ================================================
echo.
echo To activate the environment, run:
echo   conda activate %ENV_NAME%
echo.
echo or
echo.
echo   call %ENV_PATH%\Scripts\activate.bat
echo.
echo You can verify the installation by running:
echo   cd /d "%SCRIPT_DIR%"
echo   python verify_installation.py
echo.
pause

