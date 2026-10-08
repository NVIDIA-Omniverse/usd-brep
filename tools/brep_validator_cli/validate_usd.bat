:: SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
:: SPDX-License-Identifier: Apache-2.0
@echo off
setlocal
REM Run validate_usd.py with repo Python and paths. Use from repo root:
REM   tools\brep_validator_cli\validate_usd.bat path\to\file.usda

set "SCRIPT_DIR=%~dp0"
set "REPO_ROOT=%SCRIPT_DIR%..\.."
pushd "%REPO_ROOT%" && set "REPO_ROOT=%CD%" && popd

set "PYTHONPATH=%REPO_ROOT%\tools;%PYTHONPATH%"

REM So python.exe from target-deps can find python3*.dll
if exist "%REPO_ROOT%\_build\target-deps\python\python.exe" (
    set "PATH=%REPO_ROOT%\_build\target-deps\python;%PATH%"
)

REM OmniSolid plugin path (for BrepArray prim discovery)
if not defined OMNISOLID_PLUGIN_PATH (
    if exist "%REPO_ROOT%\_build\schema\omniSolid\resources" (
        set "OMNISOLID_PLUGIN_PATH=%REPO_ROOT%\_build\schema\omniSolid\resources"
    ) else (
        echo WARNING: OMNISOLID_PLUGIN_PATH not set and schema directory not found. Run prebuild.bat first.
    )
)

REM USD/pxr: must use usd/release/lib/python (pxr lives there) and put USD bin/lib on PATH for DLLs
if exist "%REPO_ROOT%\_build\target-deps\usd\release\lib\python" (
    set "PYTHONPATH=%REPO_ROOT%\_build\target-deps\usd\release\lib\python;%PYTHONPATH%"
    set "PATH=%REPO_ROOT%\_build\target-deps\usd\release\bin;%REPO_ROOT%\_build\target-deps\usd\release\lib;%PATH%"
)

REM Use target-deps Python (matches USD/omni build); fall back to packman Python
if exist "%REPO_ROOT%\_build\target-deps\python\python.exe" (
    "%REPO_ROOT%\_build\target-deps\python\python.exe" "%SCRIPT_DIR%validate_usd.py" %*
) else (
    call "%REPO_ROOT%\tools\packman\python.bat" "%SCRIPT_DIR%validate_usd.py" %*
)
exit /b %errorlevel%
