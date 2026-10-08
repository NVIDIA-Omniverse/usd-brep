#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
set -e

if [ $# -eq 0 ]; then
    echo "Usage: usd_brep_utils_test.sh <platform>"
    echo "Platform can be: linux-x86_64, linux-aarch64"
    exit 1
fi

platform="$1"
echo "Running usd_brep_utils_test on $platform"

# Always run from script directory so the relative path to the test binary is correct.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE}")" && pwd)"
cd "$SCRIPT_DIR"

WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

test_bin="$WORKSPACE_ROOT/_build/$platform/release/usd_brep_utils_test"
if [ ! -x "$test_bin" ]; then
    echo "usd_brep_utils_test binary not found at $test_bin"
    exit 1
fi

# The test links the USD_BREP_UTILS shared lib, which links USD core libs. No omniSolid
# plugin / schema registry is needed (the tested functions are pure math).
# Capture status explicitly so the report below still runs under `set -e`.
if LD_LIBRARY_PATH="$WORKSPACE_ROOT/_build/$platform/release:$WORKSPACE_ROOT/_build/target-deps/usd/release/lib:$WORKSPACE_ROOT/_build/target-deps/python/lib:${LD_LIBRARY_PATH:-}" "$test_bin"; then
    status=0
else
    status=$?
fi

if [ $status -eq 0 ]; then
    echo "usd_brep_utils_test passed"
else
    echo "usd_brep_utils_test failed with exit code $status"
fi
exit $status
