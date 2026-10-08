#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Source this before running view_usd.py / smb_to_usd.py / smtess_usd directly
# from a shell, so the omniSolid USD schema plugin and USD/pybind runtime
# libraries resolve correctly against this checkout's build.
#
# Usage (from anywhere, after building release):
#   source tools/scripts/run_view_env.sh
#   python3 tools/scripts/view_usd.py some_model.usd

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

case "$(uname -s)" in
    Darwin) PLATFORM="macos-universal" ;;
    Linux)
        case "$(uname -m)" in
            x86_64) PLATFORM="linux-x86_64" ;;
            aarch64 | arm64) PLATFORM="linux-aarch64" ;;
            *) echo "run_view_env.sh: unsupported architecture $(uname -m)" >&2; return 1 2>/dev/null || exit 1 ;;
        esac
        ;;
    *) echo "run_view_env.sh: unsupported platform $(uname -s)" >&2; return 1 2>/dev/null || exit 1 ;;
esac

export OMNISOLID_PLUGIN_PATH="$REPO_ROOT/_build/schema/omniSolid/resources"
LIB_PATHS="$REPO_ROOT/_build/$PLATFORM/release:$REPO_ROOT/_build/target-deps/usd/release/lib"
# Append the prior value only when it's actually set: a trailing bare ":"
# leaves an empty entry, which glibc's loader and Python both treat as the
# current directory -- an implicit, surprising addition to the search path.
if [ "$PLATFORM" = "macos-universal" ]; then
    export DYLD_LIBRARY_PATH="$LIB_PATHS${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
else
    export LD_LIBRARY_PATH="$LIB_PATHS${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
PY_PATH="$REPO_ROOT/_build/$PLATFORM/release"
export PYTHONPATH="$PY_PATH${PYTHONPATH:+:$PYTHONPATH}"
