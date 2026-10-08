:: SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
:: SPDX-License-Identifier: Apache-2.0
@echo off

cd /d "%~dp0"

set WORKSPACE_ROOT=%CD%\..\..
set TEST_EXE=%WORKSPACE_ROOT%\_build\windows-x86_64\release\usd_brep_utils_test.exe

REM The test links the USD_BREP_UTILS shared lib, which links USD core libs. No omniSolid
REM plugin / schema registry is needed (the tested functions are pure math).
set PATH=%WORKSPACE_ROOT%\_build\windows-x86_64\release;%WORKSPACE_ROOT%\_build\target-deps\usd\release\lib;%WORKSPACE_ROOT%\_build\target-deps\python;%PATH%

ECHO Run usd_brep_utils_test.exe...
call "%TEST_EXE%"
set TEST_STATUS=%ERRORLEVEL%
ECHO Error level after usd_brep_utils_test.exe:  %TEST_STATUS%

if "%TEST_STATUS%"=="0" (
ECHO Tests passed
) else (
ECHO Tests failed
)
exit /b %TEST_STATUS%
