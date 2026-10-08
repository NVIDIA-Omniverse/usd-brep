:: SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
:: SPDX-License-Identifier: Apache-2.0
@echo off
REM Setup environment variables for brep_validator
REM This batch file sets the required USD environment variables

echo Setting up environment variables for brep_validator...

REM Get the directory where this script is located
set "SCRIPT_DIR=%~dp0"

REM Calculate relative paths to USD installation
set "USD_ROOT=%SCRIPT_DIR%..\..\\_build\target-deps\usd\release"

REM Convert to absolute paths for environment variables
pushd "%USD_ROOT%" 2>nul
if errorlevel 1 (
    echo ERROR: Cannot navigate to USD path: %USD_ROOT%
    echo Please check if the USD build exists.
    pause
    exit /b 1
)
set "USD_ROOT=%CD%"
popd

if not exist "%USD_ROOT%\lib\python" (
    echo ERROR: USD installation not found at %USD_ROOT%
    echo Please ensure the USD build exists in the expected location.
    pause
    exit /b 1
)

REM Set USD Python path
set "PYTHONPATH=%USD_ROOT%\lib\python;%PYTHONPATH%"

REM Add USD binaries to PATH
set "PATH=%USD_ROOT%\bin;%PATH%"

REM Add USD libraries to PATH
set "PATH=%USD_ROOT%\lib;%PATH%"

echo Environment variables set successfully!
echo.
echo USD_ROOT: %USD_ROOT%
echo PYTHONPATH includes: %USD_ROOT%\lib\python
echo PATH includes: 
echo   - %USD_ROOT%\bin
echo   - %USD_ROOT%\lib

REM Optional: omniSolid plugin path
if not defined OMNISOLID_PLUGIN_PATH (
    set "OMNISOLID_PLUGIN_PATH=%SCRIPT_DIR%..\..\_build\schema\omniSolid\resources"
)
if exist "%OMNISOLID_PLUGIN_PATH%" (
    echo OMNISOLID_PLUGIN_PATH: %OMNISOLID_PLUGIN_PATH%
) else (
    echo WARNING: OMNISOLID_PLUGIN_PATH is not set to an existing path; set it to the omniSolid plugin resources directory.
)

REM Asset validator (omni.asset_validator / usd_validation_nvidia) is provided by the
REM `usd-validation-nvidia` pip package, installed into the repo Python's site-packages
REM (pip install "usd-validation-nvidia>=1.22.0,<2").
echo NOTE: usd_validation_nvidia is resolved from the Python environment; if missing, run: pip install "usd-validation-nvidia>=1.22.0,<2"
echo.
echo You can now run the brep_validator (from repo root):
echo   python tools\brep_validator_cli\brep_array_handler.py "path\to\your\file.usda"
echo.
echo Note: These environment variables are only set for this command prompt session.
