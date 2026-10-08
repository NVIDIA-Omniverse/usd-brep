#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
set -e

if [ $# -eq 0 ]; then
    echo "Usage: tests.sh <platform>"
    echo "Platform can be: linux-x86_64, linux-aarch64, macos-universal"
    exit 1
fi

platform="$1"
echo "Running tests on $platform"

# Always run from script directory so relative path to test is correct
SCRIPT_DIR="$(dirname "${BASH_SOURCE}")"
cd "$SCRIPT_DIR"

echo "Setting script_dir to " "$SCRIPT_DIR"
echo "Running tests from directory: " "${PWD}"

# Runtime library search path (Linux: LD_LIBRARY_PATH; macOS: DYLD_LIBRARY_PATH).
runtime_lib_dirs() {
    echo "${PWD}/_build/${platform}/release:${PWD}/_build/target-deps/usd/release/lib:${PWD}/_build/target-deps/python/lib"
}

apply_runtime_lib_path() {
    local dirs
    dirs="$(runtime_lib_dirs)"
    case "$platform" in
        macos-*)
            export DYLD_LIBRARY_PATH="${dirs}${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
            ;;
        *)
            export LD_LIBRARY_PATH="${dirs}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
            ;;
    esac
}

repo_python() {
    if [ -x "${PWD}/_build/target-deps/python/bin/python3" ]; then
        echo "${PWD}/_build/target-deps/python/bin/python3"
    else
        echo "${PWD}/_build/target-deps/python/python"
    fi
}

ensure_omnisolid_schema() {
    local src="${PWD}/source/schema/omniSolid/resources"
    local dst="${PWD}/_build/schema/omniSolid/resources"
    if [ ! -f "$dst/plugInfo.json" ] && [ -f "$src/plugInfo.json" ]; then
        mkdir -p "$dst"
        cp -f "$src/generatedSchema.usda" "$src/plugInfo.json" "$dst/"
        echo "Copied omniSolid USD schema to $dst"
    fi
}

# macOS python-only builds omit USD targets; other platforms must have them built.
ensure_binary_or_handle() {
    local label="$1"
    local path="$2"
    if [ -x "$path" ]; then
        return 0
    fi
    case "$platform" in
        macos-*)
            echo "Skipping ${label} (${path} not built for ${platform})"
            return 1
            ;;
        *)
            echo "${label} failed: required binary missing (${path}), aborting"
            exit 1
            ;;
    esac
}

ensure_binaries_or_handle() {
    local label="$1"
    shift
    for path in "$@"; do
        if [ ! -x "$path" ]; then
            case "$platform" in
                macos-*)
                    echo "Skipping ${label} (binaries not built for ${platform})"
                    return 1
                    ;;
                *)
                    echo "${label} failed: required binary missing (${path}), aborting"
                    exit 1
                    ;;
            esac
        fi
    done
    return 0
}

run_test_script() {
    local label="$1" binary="$2" script="$3"
    echo "Running ${label}"
    if ensure_binary_or_handle "$label" "$binary"; then
        "$script" "$platform"
        status=$?
        if [ $status -ne 0 ]; then
            echo "${label} failed with exit code $status, aborting"
            exit $status
        fi
    fi
}

ensure_omnisolid_schema
apply_runtime_lib_path

# Run test binaries one by one

run_test_script "usd tests" "${PWD}/_build/${platform}/release/usd_test_app" \
    ./tests/usd_test/usd_test.sh

# Unit tests for the kernel-free USD_BREP_UTILS math (knot conversions + analytic/NURBS
# evaluators + inverse projection). Pure functions, no USD stage required.
run_test_script "usd_brep_utils_test" "${PWD}/_build/${platform}/release/usd_brep_utils_test" \
    ./tests/usd_brep_utils_test/usd_brep_utils_test.sh




echo "Running prog_test"
./tests/prog_test/prog_test.sh "$platform"
status=$?
if [ $status -ne 0 ]; then
    echo "prog_test failed with exit code $status, aborting"
    exit $status
fi

echo "Running SM_API_test"
./tests/SM_API_test/SM_API_test.sh "$platform"
status=$?
if [ $status -ne 0 ]; then
    echo "SM_API_test failed with exit code $status, aborting"
    exit $status
fi

run_test_script "SM_API_USD_test" "${PWD}/_build/${platform}/release/SM_API_USD_test_app" \
    ./tests/SM_API_USD_test/SM_API_USD_test.sh

# The hdUsdBrep plugin is built only against an OpenUSD with imaging (usd_has_imaging() in premake5-libraries.lua).
if [ -d "${PWD}/_build/target-deps/usd/release/include/pxr/imaging/hd" ]; then
    run_test_script "HYDRA_USD_BREP_test" "${PWD}/_build/${platform}/release/HYDRA_USD_BREP_test_app" \
        ./tests/HYDRA_USD_BREP_test/HYDRA_USD_BREP_test.sh
else
    echo "Skipping HYDRA_USD_BREP_test (OpenUSD built without imaging)"
fi

if ensure_binary_or_handle "brep_geometry_validator Python tests" "${PWD}/_build/${platform}/release/brep_geometry_validator"; then
    echo "Running brep_geometry_validator Python tests"
    apply_runtime_lib_path
    BREP_GEOMETRY_VALIDATOR="${PWD}/_build/${platform}/release/brep_geometry_validator" \
    OMNISOLID_PLUGIN_PATH="${PWD}/_build/schema/omniSolid/resources" \
    PYTHONPATH="${PWD}/_build/target-deps/usd/release/lib/python${PYTHONPATH:+:$PYTHONPATH}" \
    "$(repo_python)" -m unittest discover -s tools/brep_geometry_validator/tests -p "test_*.py"
    status=$?
    if [ $status -ne 0 ]; then
        echo "brep_geometry_validator Python tests failed with exit code $status, aborting"
        exit $status
    fi
fi

echo "Running heal_file Python tests"
apply_runtime_lib_path
OMNISOLID_PLUGIN_PATH="${PWD}/_build/schema/omniSolid/resources" \
PYTHONPATH="${PWD}/_build/target-deps/usd/release/lib/python:${PWD}/_build/${platform}/release" \
"$(repo_python)" -m unittest discover -s source/SmPyLib/tests_usd -p "test_*.py"
status=$?
if [ $status -ne 0 ]; then
    echo "heal_file Python tests failed with exit code $status, aborting"
    exit $status
fi

smpylib_so=$(find "${PWD}/_build/${platform}/release" -maxdepth 1 -name '_omni_solid*' 2>/dev/null | head -1)
if [ -n "$smpylib_so" ]; then
    echo "Running SmPyLib (Python bindings) tests"
    apply_runtime_lib_path
    set +e
    OMNISOLID_PLUGIN_PATH="${PWD}/_build/schema/omniSolid/resources" \
    PYTHONPATH="${PWD}/_build/${platform}/release" \
    "$(repo_python)" -m unittest discover -s source/SmPyLib/tests -p "test_*.py" -v
    status=$?
    set -e
    if [ $status -ne 0 ]; then
        echo "SmPyLib tests failed with exit code $status, aborting"
        exit $status
    fi
else
    echo "Skipping SmPyLib tests (_omni_solid module not found in _build/${platform}/release)"
fi

if compgen -G "${PWD}/_build/${platform}/release/_smlib_dev*.so" > /dev/null; then
    echo "Running SmPyDevLib tests"
    apply_runtime_lib_path
    OMNISOLID_PLUGIN_PATH="${PWD}/_build/schema/omniSolid/resources" \
    PYTHONPATH="${PWD}/_build/${platform}/release" \
    "$(repo_python)" -m unittest discover -s source/SmPyDevLib/tests -p "test_*.py" -v || exit $?
else
    echo "Skipping SmPyDevLib tests (_smlib_dev module not built)"
fi

if ensure_binary_or_handle "brep_validator Python unittests" "${PWD}/_build/${platform}/release/usd_test_app"; then
    echo "Running brep_validator Python unittests"
    apply_runtime_lib_path
    # usd-validation-nvidia is pure Python. Install it into _build/pydeps, not into the
    # interpreter, which can be a link into a shared packman cache. The folder is named
    # after the version range, so changing the range installs afresh. Change both lines.
    validator_requirement="usd-validation-nvidia>=1.22.0,<2"
    pydeps="${PWD}/_build/pydeps/usd-validation-nvidia-1.22-2"
    if [ ! -d "${pydeps}/usd_validation_nvidia" ]; then
        "$(repo_python)" -m pip install --quiet --disable-pip-version-check --target "${pydeps}" \
            "${validator_requirement}"
    fi
    PYTHONPATH="${pydeps}${PYTHONPATH:+:$PYTHONPATH}" \
    "$(repo_python)" -m unittest discover -s tools/brep_validator_test -p "test_*.py"
    status=$?
    if [ $status -ne 0 ]; then
        echo "brep_validator Python unittests failed with exit code $status, aborting"
        exit $status
    fi
fi

# OCCT occt_to_usd importer tests run everywhere (including CI) as a hard gate:
# they convert the committed .brep corpus to USD, validate the schema, and run a
# SMLib AssertValid round-trip. usd_test_app (spawned by --assert-valid) links the
# SMLib shared libs in _build/<platform>/release and needs the omniSolid plugin, so
# set those explicitly (base_test_case bootstrap only adds the usd/python libs).
echo "Running OCCT occt_to_usd importer tests"
# Hard gate: pin the exact platform binaries and fail fast if either is missing, so the Python suite
# cannot silently skip (or auto-discover a stale build artifact for the wrong platform).
occt_to_usd="${PWD}/_build/${platform}/release/occt_to_usd"
occt_usd_test_app="${PWD}/_build/${platform}/release/usd_test_app"
if ensure_binaries_or_handle "OCCT occt_to_usd importer tests" "$occt_to_usd" "$occt_usd_test_app"; then
    apply_runtime_lib_path
    OCCT_TO_USD="$occt_to_usd" \
    USD_TEST_APP="$occt_usd_test_app" \
    OMNISOLID_PLUGIN_PATH="${PWD}/_build/schema/omniSolid/resources" \
    PYTHONPATH="${PWD}/_build/${platform}/release${PYTHONPATH:+:$PYTHONPATH}" \
    "$(repo_python)" -m unittest discover -s tools/occt_to_usd_test -p "test_*.py"
    status=$?
    if [ $status -ne 0 ]; then
        echo "OCCT occt_to_usd importer tests failed with exit code $status, aborting"
        exit $status
    fi
fi

# OCCT usd_to_occt exporter tests: round-trip the committed corpus through the exporter
# (occt_to_usd seed import -> usd_to_occt export -> occt_to_usd re-import -> validate). Reuses the
# importer binary and usd_test_app, so pin all three and fail fast if any is missing.
echo "Running OCCT usd_to_occt exporter tests"
usd_to_occt="${PWD}/_build/${platform}/release/usd_to_occt"
if ensure_binaries_or_handle "OCCT usd_to_occt exporter tests" "$usd_to_occt" "$occt_to_usd" "$occt_usd_test_app"; then
    apply_runtime_lib_path
    USD_TO_OCCT="$usd_to_occt" \
    OCCT_TO_USD="$occt_to_usd" \
    USD_TEST_APP="$occt_usd_test_app" \
    OMNISOLID_PLUGIN_PATH="${PWD}/_build/schema/omniSolid/resources" \
    PYTHONPATH="${PWD}/_build/${platform}/release${PYTHONPATH:+:$PYTHONPATH}" \
    "$(repo_python)" -m unittest discover -s tools/usd_to_occt_test -p "test_*.py"
    status=$?
    if [ $status -ne 0 ]; then
        echo "OCCT usd_to_occt exporter tests failed with exit code $status, aborting"
        exit $status
    fi
fi

exit $status
