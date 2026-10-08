#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
set -e

if [ $# -eq 0 ]; then
    echo "Usage: usd_test.sh <platform>"
    echo "Platform can be: linux-x86_64, linux-aarch64"
    exit 1
fi

platform="$1"
echo "Running tests on $platform"

# Always run from script directory so relative path to test is correct
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE}")" && pwd)"
cd $SCRIPT_DIR

WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
export OMNISOLID_PLUGIN_PATH="$WORKSPACE_ROOT/_build/schema/omniSolid/resources"
echo "OMNISOLID_PLUGIN_PATH: $OMNISOLID_PLUGIN_PATH"

echo "Setting script_dir to " $SCRIPT_DIR
echo "Running tests from directory: " ${PWD}

# Run test binary
echo "Running tests"
../../_build/$platform/release/usd_test_app

# Check if tests passed
status=$?
if [ $status -eq 0 ]; then
    echo "Tests process completed, tests passed"
else
    echo "Tests process completed, tests failed"
fi
exit $status


