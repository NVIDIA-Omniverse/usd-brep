#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
set -e

if [ $# -eq 0 ]; then
    echo "Usage: prog_test.sh <platform> [prog_test_app args...]"
    echo "Platform can be: linux-x86_64, macos-universal"
    echo "Examples: prog_test.sh linux-x86_64 --list-suites"
    echo "          prog_test.sh linux-x86_64 --suite booleans"
    exit 1
fi

platform="$1"
shift
echo "Running tests on $platform"

# Always run from script directory so relative path to test is correct
SCRIPT_DIR="$(dirname "${BASH_SOURCE}")"
cd $SCRIPT_DIR

echo "Setting script_dir to " $SCRIPT_DIR
echo "Running tests from directory: " ${PWD}

# Run test binary
echo "Running tests"
"../../_build/${platform}/release/prog_test_app" "$@"

# Check if tests passed
status=$?
if [ $status -eq 0 ]; then
    echo "Tests process completed, tests passed"
else
    echo "Tests process completed, tests failed"
fi
exit $status
