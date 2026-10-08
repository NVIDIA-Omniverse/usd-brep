#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
set -e

if [ $# -eq 0 ]; then
    echo "Usage: SM_API_test.sh <platform>"
    echo "Platform can be: linux-x86_64, macos-universal"
    exit 1
fi

platform="$1"
echo "Running tests on $platform"

# Always run from script directory so relative paths are correct.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$SCRIPT_DIR"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

test_exe="$WORKSPACE_ROOT/_build/$platform/release/SM_API_test_app"
if [ ! -x "$test_exe" ]; then
    test_exe="$WORKSPACE_ROOT/tests/bin/SM_API_test_app"
fi

export LD_LIBRARY_PATH="${WORKSPACE_ROOT}/_build/${platform}/release:${WORKSPACE_ROOT}/lib:${WORKSPACE_ROOT}/extraLibs:${LD_LIBRARY_PATH:-}"

echo "Setting script_dir to " "$SCRIPT_DIR"
echo "Running tests from directory: " ${PWD}

# Run test binary
echo "Running tests"
set +e
"$test_exe"
status=$?
set -e
if [ $status -eq 0 ]; then
    echo "Tests process completed, tests passed"
else
    echo "Tests process completed, tests failed"
fi
exit $status
