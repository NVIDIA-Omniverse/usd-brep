#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
# Run validate_usd.py with repo Python and paths. Use from repo root:
#   tools/brep_validator_cli/validate_usd.sh path/to/file.usda

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

export PYTHONPATH="$REPO_ROOT/tools:$PYTHONPATH"
if [ -d "$REPO_ROOT/_build/target-deps/python/bin" ]; then
    export PATH="$REPO_ROOT/_build/target-deps/python/bin:$PATH"
    [ -d "$REPO_ROOT/_build/target-deps/python/lib" ] && export LD_LIBRARY_PATH="$REPO_ROOT/_build/target-deps/python/lib:${LD_LIBRARY_PATH:-}"
fi
if [ -z "${OMNISOLID_PLUGIN_PATH:-}" ]; then
    if [ -d "$REPO_ROOT/_build/schema/omniSolid/resources" ]; then
        export OMNISOLID_PLUGIN_PATH="$REPO_ROOT/_build/schema/omniSolid/resources"
    else
        echo "WARNING: OMNISOLID_PLUGIN_PATH not set and schema directory not found. Run ./prebuild.sh first." >&2
    fi
fi
if [ -d "$REPO_ROOT/_build/target-deps/usd/release/lib/python" ]; then
    export PYTHONPATH="$REPO_ROOT/_build/target-deps/usd/release/lib/python:$PYTHONPATH"
    export PATH="$REPO_ROOT/_build/target-deps/usd/release/bin:$REPO_ROOT/_build/target-deps/usd/release/lib:$PATH"
    export LD_LIBRARY_PATH="$REPO_ROOT/_build/target-deps/usd/release/lib:${LD_LIBRARY_PATH:-}"
fi

if [ -x "$REPO_ROOT/_build/target-deps/python/bin/python3" ]; then
    exec "$REPO_ROOT/_build/target-deps/python/bin/python3" -u "$SCRIPT_DIR/validate_usd.py" "$@"
else
    exec "$REPO_ROOT/tools/packman/python.sh" -u "$SCRIPT_DIR/validate_usd.py" "$@"
fi
