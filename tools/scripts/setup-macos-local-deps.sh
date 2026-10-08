#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Stage local macOS dependencies for Tier 1 builds when Packman USD/Python packages
# are not published. Run from repo root before ./repo.sh build -p macos-universal.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd -P)"
cd "${REPO_ROOT}"

USD_ROOT="${SMLIB_USD_ROOT:-${HOME}/opt/usd-25.11-py312}"
PM_PYTHON_ROOT="${SMLIB_PYTHON_ROOT:-${HOME}/Library/Application Support/packman-cache/python/3.12.13-macos-aarch64}"

if [[ "$(uname -s)" != "Darwin" ]]; then
    echo "setup-macos-local-deps.sh is for macOS only." >&2
    exit 1
fi

if [[ ! -d "${USD_ROOT}/include/pxr" ]]; then
    echo "USD install not found at ${USD_ROOT} (expected include/pxr)." >&2
    echo "Set SMLIB_USD_ROOT to your USD prefix." >&2
    exit 1
fi

if [[ ! -d "${PM_PYTHON_ROOT}/include/python3.12" ]]; then
    echo "Packman Python 3.12 not found at ${PM_PYTHON_ROOT}." >&2
    echo "Run ./repo.sh build -p macos-universal once (or set SMLIB_PYTHON_ROOT)." >&2
    exit 1
fi

mkdir -p _build/target-deps/usd
rm -rf _build/target-deps/usd/release _build/target-deps/usd/debug _build/target-deps/python

ln -sfn "${USD_ROOT}" _build/target-deps/usd/release
ln -sfn "${USD_ROOT}" _build/target-deps/usd/debug
ln -sfn "${PM_PYTHON_ROOT}" _build/target-deps/python

echo "Staged: USD -> _build/target-deps/usd/release, Python -> _build/target-deps/python"
echo "Build: ./repo.sh build -p macos-universal"
