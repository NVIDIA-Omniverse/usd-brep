#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

usage() {
    cat <<'USAGE'
Usage: tools/agent/run_smpylib_tests.sh [--platform linux-x86_64] [--target unittest.name]

Runs SmPyLib Python binding tests with the same runtime environment used by tests.sh.

Examples:
  tools/agent/run_smpylib_tests.sh
  tools/agent/run_smpylib_tests.sh --target test_omni_solid.TestBooleans
  tools/agent/run_smpylib_tests.sh --platform linux-aarch64
USAGE
}

platform="linux-x86_64"
target=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --platform)
            if [[ $# -lt 2 ]]; then
                echo "error: --platform requires a value" >&2
                exit 2
            fi
            platform="$2"
            shift 2
            ;;
        --target)
            if [[ $# -lt 2 ]]; then
                echo "error: --target requires a unittest target" >&2
                exit 2
            fi
            target="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "error: unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
repo_root="$(cd "${script_dir}/../.." && pwd -P)"
release_dir="${repo_root}/_build/${platform}/release"
python_bin="${repo_root}/_build/target-deps/python/bin/python3"

if ! find "${release_dir}" -maxdepth 1 -name '_omni_solid*' -print -quit 2>/dev/null | grep -q .; then
    echo "error: _omni_solid module not found in ${release_dir}" >&2
    echo "hint: run ./repo.sh build first, or pass --platform for an existing build" >&2
    exit 1
fi

if [[ ! -x "${python_bin}" ]]; then
    echo "error: repo Python not found or not executable: ${python_bin}" >&2
    echo "hint: run ./repo.sh setup or ./repo.sh build first" >&2
    exit 1
fi

export OMNISOLID_PLUGIN_PATH="${repo_root}/_build/schema/omniSolid/resources"
export LD_LIBRARY_PATH="${release_dir}:${repo_root}/_build/target-deps/usd/release/lib:${repo_root}/_build/target-deps/python/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

cd "${repo_root}"

if [[ -n "${target}" ]]; then
    export PYTHONPATH="${release_dir}:${repo_root}/source/SmPyLib/tests:${repo_root}/source/SmPyDevLib/tests"
    exec "${python_bin}" -m unittest "${target}" -v
fi

export PYTHONPATH="${release_dir}"
"${python_bin}" -m unittest discover -s source/SmPyLib/tests -p "test_*.py" -v
"${python_bin}" -m unittest discover -s source/SmPyDevLib/tests -p "test_*.py" -v