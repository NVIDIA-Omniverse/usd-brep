:: SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
:: SPDX-License-Identifier: Apache-2.0
@echo off
setlocal
set "WORKSPACE_ROOT=%~dp0..\.."
call "%WORKSPACE_ROOT%\_build\target-deps\python\python.exe" "%~dp0run_tests.py"
exit /b %ERRORLEVEL%
