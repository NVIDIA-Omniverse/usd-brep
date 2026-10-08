#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
# Setup environment variables for brep_validator
# This bash script sets the required USD environment variables

echo "Setting up environment variables for brep_validator..."

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Calculate relative path to USD installation (may be Windows binaries if built on Windows)
USD_ROOT="$SCRIPT_DIR/../../_build/target-deps/usd/release"

# Calculate relative path to bundled Python (matches USD build)
PY_ROOT="$SCRIPT_DIR/../../_build/target-deps/python"

# Convert to absolute paths only if they exist
if [ -d "$USD_ROOT" ]; then
    USD_ROOT="$(cd "$USD_ROOT" && pwd)"
fi
if [ -d "$PY_ROOT" ]; then
    PY_ROOT="$(cd "$PY_ROOT" && pwd)"
fi

# If we're on Linux/WSL and only Windows pxr binaries are present, try a packman Linux cache fallback.
if [ -d "$USD_ROOT/lib/python/pxr/Tf" ] && [ ! -f "$USD_ROOT/lib/python/pxr/Tf/_tf.so" ] && [ -f "$USD_ROOT/lib/python/pxr/Tf/_tf.pyd" ]; then
    fallback_found=false
    cache_root="$HOME/.cache/packman/chk/usd.py312.manylinux_2_35_x86_64.stock.release"
    for candidate in "$cache_root"/*; do
        if [ -d "$candidate/lib/python/pxr/Tf" ] && [ -f "$candidate/lib/python/pxr/Tf/_tf.so" ]; then
            USD_ROOT="$candidate"
            echo "Detected Windows USD in _build; using Linux USD from packman cache: $USD_ROOT"
            fallback_found=true
            break
        fi
    done
    if [ "$fallback_found" = false ]; then
        echo "ERROR: Detected Windows USD at $USD_ROOT but no Linux USD found under $cache_root"
        exit 1
    fi
fi

if [ ! -d "$USD_ROOT" ] || [ ! -d "$USD_ROOT/lib/python" ]; then
    echo "ERROR: USD installation not found at expected location"
    echo "Expected: $USD_ROOT"
    echo "Please ensure the USD build exists in the expected location."
    exit 1
fi

# Prefer bundled Python (matches USD build); fall back to system python otherwise
PY_BIN="$PY_ROOT/bin/python3"
if [ -x "$PY_BIN" ]; then
    export PATH="$PY_ROOT/bin:$PATH"
    # Add Python libs so libpython3.x is found
    if [ -d "$PY_ROOT/lib" ]; then
        export LD_LIBRARY_PATH="$PY_ROOT/lib:$LD_LIBRARY_PATH"
    fi
    if [ -d "$PY_ROOT/lib64" ]; then
        export LD_LIBRARY_PATH="$PY_ROOT/lib64:$LD_LIBRARY_PATH"
    fi
else
    echo "WARNING: Bundled Python not found at $PY_ROOT; using system python."
fi

# Emit a warning if python3 still isn't on PATH after prepending the bundled bin
if ! command -v python3 >/dev/null 2>&1; then
    echo "WARNING: python3 not found on PATH after setup; ensure a compatible Python is available."
fi

# Set USD Python path
export PYTHONPATH="$USD_ROOT/lib/python:$PYTHONPATH"

# Add USD binaries to PATH
export PATH="$USD_ROOT/bin:$PATH"

# Add USD libraries to LD_LIBRARY_PATH (Linux equivalent of adding DLLs to PATH)
export LD_LIBRARY_PATH="$USD_ROOT/lib:$LD_LIBRARY_PATH"

echo "Environment variables set successfully!"
echo ""
echo "USD_ROOT: $USD_ROOT"
echo "PYTHONPATH includes: $USD_ROOT/lib/python"
echo "PATH includes: $USD_ROOT/bin"
echo "LD_LIBRARY_PATH includes: $USD_ROOT/lib"
echo ""

# Optional: omniSolid plugin path
if [ -z "${OMNISOLID_PLUGIN_PATH:-}" ]; then
    default_plugin_path="$SCRIPT_DIR/../../_build/schema/omniSolid/resources"
    if [ -d "$default_plugin_path" ]; then
        export OMNISOLID_PLUGIN_PATH="$default_plugin_path"
    fi
fi

if [ -n "${OMNISOLID_PLUGIN_PATH:-}" ]; then
    echo "OMNISOLID_PLUGIN_PATH: $OMNISOLID_PLUGIN_PATH"
else
    echo "WARNING: OMNISOLID_PLUGIN_PATH is not set; set it to the omniSolid plugin resources directory."
fi

# Asset validator (omni.asset_validator / usd_validation_nvidia) is provided by the
# `usd-validation-nvidia` pip package, installed into the repo Python's site-packages
# (`pip install "usd-validation-nvidia>=1.22.0,<2"`).
if command -v python3 >/dev/null 2>&1 && python3 -c "import usd_validation_nvidia" >/dev/null 2>&1; then
    echo "usd_validation_nvidia: found in the active Python environment"
else
    echo 'WARNING: usd_validation_nvidia not found; install it with: pip install "usd-validation-nvidia>=1.22.0,<2"'
fi
echo ""
echo "You can now run the brep_validator (from repo root):"
echo '  python3 tools/brep_validator_cli/brep_array_handler.py "path/to/your/file.usda"'
echo ""
echo "Note: These environment variables are only set for this shell session."
echo "To make them persistent, you can source this script in your shell profile:"
echo "  source $(realpath "$0")"
