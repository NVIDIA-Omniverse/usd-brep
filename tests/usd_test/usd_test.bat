:: SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
:: SPDX-License-Identifier: Apache-2.0
@echo off

cd tests\usd_test

REM Add all required DLL paths to PATH
set PATH=..\..\_build\target-deps\usd\release\lib;..\..\_build\target-deps\python;%PATH%
set OMNISOLID_PLUGIN_PATH=..\..\_build\schema\omniSolid\resources

ECHO Run usd_test_app.exe...
call "..\..\_build\windows-x86_64\release\usd_test_app.exe"
ECHO Error level after usd_test_app.exe:  %ERRORLEVEL%

if %ERRORLEVEL% == 0 (

ECHO Tests passed

) else (

ECHO Tests failed, run again to see if reproducible...

REM Reset errorcode to zero just in case it somehow affects a second run
ver > nul
ECHO Error level before rerun of usd_test_app.exe:  %ERRORLEVEL%

call "..\..\_build\windows-x86_64\release\usd_test_app.exe"
ECHO Error level after retrying usd_test_app.exe:  %ERRORLEVEL%

)