:: SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
:: SPDX-License-Identifier: Apache-2.0
@echo off
setlocal enabledelayedexpansion

set SCRIPT_DIR=%~dp0

call "%SCRIPT_DIR%repo.bat" build -g -r --platform-target windows-x86_64 %*
if errorlevel 1 ( goto Error )

REM omniSolid USD schema.
REM
REM The compiled schema (generatedSchema.usda + plugInfo.json) is committed under
REM source\. If schema.usda was edited locally we regenerate the committed files
REM in source\ with usdGenSchema; then, either way, the compiled files are copied
REM into _build so the plugin resources sit alongside the build artifacts (CI
REM artifact collection, packaging).
REM
REM Regeneration needs the Python "jinja2" module. We do NOT vendor or install
REM jinja2 (it is not a dependency of this repo); if it is missing when needed we
REM warn with install instructions and fall back to the committed compiled schema.
set SCHEMA_SRC_DIR=%SCRIPT_DIR%source\schema\omniSolid\resources
set SCHEMA_OUT=%SCRIPT_DIR%_build\schema\omniSolid\resources
set SCHEMA_USDA=%SCHEMA_SRC_DIR%\schema.usda
set GENERATED=%SCHEMA_SRC_DIR%\generatedSchema.usda
set PYTHON=%SCRIPT_DIR%_build\target-deps\python\python.exe
set USD_GEN_SCHEMA=%SCRIPT_DIR%_build\target-deps\usd\release\bin\usdGenSchema

REM Detect a real edit to schema.usda: a working-tree change vs HEAD (so fresh
REM checkouts and CI do not regenerate or warn). Fall back to mtime if git is absent.
set "NEEDS_REGEN=0"
set "IS_GIT=0"
where git >nul 2>&1
if not errorlevel 1 (
    git -C "%SCRIPT_DIR%." rev-parse --is-inside-work-tree >nul 2>&1
    if not errorlevel 1 set "IS_GIT=1"
)
if "!IS_GIT!"=="1" (
    git -C "%SCRIPT_DIR%." diff --quiet HEAD -- "%SCHEMA_USDA%" >nul 2>&1
    if errorlevel 1 set "NEEDS_REGEN=1"
) else (
    if exist "%PYTHON%" if exist "%GENERATED%" (
        "%PYTHON%" -c "import os,sys; sys.exit(1 if os.path.getmtime(r'%SCHEMA_USDA%') > os.path.getmtime(r'%GENERATED%') else 0)" >nul 2>&1
        if errorlevel 1 set "NEEDS_REGEN=1"
    )
)

if "!NEEDS_REGEN!"=="1" (
    set "HAVE_JINJA=0"
    if exist "%PYTHON%" if exist "%USD_GEN_SCHEMA%" (
        "%PYTHON%" -c "import jinja2" >nul 2>&1
        if not errorlevel 1 set "HAVE_JINJA=1"
    )
    if "!HAVE_JINJA!"=="1" (
        echo prebuild: schema.usda changed; regenerating omniSolid USD schema in source...
        set "PYTHONPATH=%SCRIPT_DIR%_build\target-deps\usd\release\lib\python;%PYTHONPATH%"
        set "PATH=%SCRIPT_DIR%_build\target-deps\usd\release\bin;%SCRIPT_DIR%_build\target-deps\usd\release\lib;%PATH%"
        "%PYTHON%" "%USD_GEN_SCHEMA%" "%SCHEMA_USDA%" "%SCHEMA_SRC_DIR%"
        if errorlevel 1 ( goto Error )
        echo prebuild: Schema regeneration complete; remember to commit the updated files in %SCHEMA_SRC_DIR%
    ) else (
        echo prebuild: WARNING: schema.usda was edited but the compiled schema could not be regenerated.
        echo prebuild:          usdGenSchema needs the Python 'jinja2' module. Install it with:
        echo prebuild:              "%PYTHON%" -m pip install jinja2
        echo prebuild:          then re-run prebuild. Using the committed compiled schema for now.
    )
)

REM Copy the compiled schema from source into _build (always, so _build mirrors source).
if not exist "%SCHEMA_OUT%\" mkdir "%SCHEMA_OUT%"
copy /Y "%GENERATED%" "%SCHEMA_OUT%\" >nul
if errorlevel 1 ( goto Error )
copy /Y "%SCHEMA_SRC_DIR%\plugInfo.json" "%SCHEMA_OUT%\" >nul
if errorlevel 1 ( goto Error )
echo prebuild: Copied omniSolid USD schema to %SCHEMA_OUT%

:Success
exit /b 0

:Error
exit /b %errorlevel%
