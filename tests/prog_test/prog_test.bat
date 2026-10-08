:: SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
:: SPDX-License-Identifier: Apache-2.0
@echo off

cd tests\prog_test

ECHO Run prog_test_app.exe...
call "..\..\_build\windows-x86_64\release\prog_test_app.exe" %*
ECHO Error level after prog_test_app.exe:  %ERRORLEVEL%

if %ERRORLEVEL% == 0 (
ECHO Tests passed
) else (
ECHO Tests failed
)
