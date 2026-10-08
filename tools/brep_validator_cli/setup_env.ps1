<#
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
#>

# Setup environment variables for brep_validator
# This PowerShell script sets the required USD environment variables

Write-Host "Setting up environment variables for brep_validator..." -ForegroundColor Green

# Get the directory where this script is located
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

# Calculate relative path to USD installation
$UsdRoot = Join-Path $ScriptDir "..\..\\_build\target-deps\usd\release"

# Convert to absolute path
$UsdRoot = Resolve-Path $UsdRoot -ErrorAction SilentlyContinue

if (-not $UsdRoot -or -not (Test-Path (Join-Path $UsdRoot "lib\python"))) {
    Write-Host "ERROR: USD installation not found at expected location" -ForegroundColor Red
    Write-Host "Expected: $UsdRoot" -ForegroundColor Red
    Write-Host "Please ensure the USD build exists in the expected location." -ForegroundColor Red
    exit 1
}

# Set USD Python path
$env:PYTHONPATH = "$UsdRoot\lib\python;$env:PYTHONPATH"

# Add USD binaries to PATH
$env:PATH = "$UsdRoot\bin;$env:PATH"

# Add USD libraries to PATH
$env:PATH = "$UsdRoot\lib;$env:PATH"

Write-Host "Environment variables set successfully!" -ForegroundColor Green
Write-Host ""
Write-Host "USD_ROOT: $UsdRoot" -ForegroundColor Yellow
Write-Host "PYTHONPATH includes: $UsdRoot\lib\python" -ForegroundColor Yellow
Write-Host "PATH includes:" -ForegroundColor Yellow
Write-Host "  - $UsdRoot\bin" -ForegroundColor Yellow
Write-Host "  - $UsdRoot\lib" -ForegroundColor Yellow
Write-Host "" -ForegroundColor Yellow

# Optional: omniSolid plugin path
if (-not $env:OMNISOLID_PLUGIN_PATH) {
    $defaultPluginPath = Join-Path $ScriptDir "..\..\_build\schema\omniSolid\resources"
    if (Test-Path $defaultPluginPath) {
        $env:OMNISOLID_PLUGIN_PATH = $defaultPluginPath
    }
}
if ($env:OMNISOLID_PLUGIN_PATH) {
    Write-Host "OMNISOLID_PLUGIN_PATH: $env:OMNISOLID_PLUGIN_PATH" -ForegroundColor Yellow
} else {
    Write-Host "WARNING: OMNISOLID_PLUGIN_PATH is not set; set it to the omniSolid plugin resources directory." -ForegroundColor Yellow
}

# Asset validator (omni.asset_validator / usd_validation_nvidia) is provided by the
# `usd-validation-nvidia` pip package, installed into the repo Python's site-packages
# (pip install "usd-validation-nvidia>=1.22.0,<2").
Write-Host 'NOTE: usd_validation_nvidia is resolved from the Python environment; if missing, run: pip install "usd-validation-nvidia>=1.22.0,<2"' -ForegroundColor Yellow
Write-Host ""
Write-Host "You can now run the brep_validator (from repo root):" -ForegroundColor Cyan
Write-Host '  python tools\brep_validator_cli\brep_array_handler.py "path\to\your\file.usda"' -ForegroundColor Cyan
Write-Host ""
Write-Host "Note: These environment variables are only set for this PowerShell session." -ForegroundColor Magenta
