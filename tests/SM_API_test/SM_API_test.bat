:: SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
:: SPDX-License-Identifier: Apache-2.0
@echo off

cd /d "%~dp0"

set WORKSPACE_ROOT=%CD%\..\..
set TEST_EXE=%WORKSPACE_ROOT%\_build\windows-x86_64\release\SM_API_test_app.exe
if not exist "%TEST_EXE%" set TEST_EXE=%WORKSPACE_ROOT%\tests\bin\SM_API_test_app.exe
set PATH=%WORKSPACE_ROOT%\_build\windows-x86_64\release;%WORKSPACE_ROOT%\lib;%WORKSPACE_ROOT%\extraLibs;%PATH%

ECHO Run SM_API_test_app.exe...
call "%TEST_EXE%"
ECHO Error level after SM_API_test_app.exe:  %ERRORLEVEL%

if %ERRORLEVEL% == 0 (
ECHO Tests passed
) else (
ECHO Tests failed
)
