#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# inject "-p linux-<arch>" as the default if no target platform is specified
for i in "$@"
do
    case $i in
        -p=*|--platform-target=*|-p|--platform-target)
        PLATFORM_TARGET_SET=1
        ;;
    esac
done

ARGS_ARRAY=("$@")
if [ -z "$PLATFORM_TARGET_SET" ]; then
    HOST_ARCH="$(uname -m)"
    ARGS_ARRAY+=("-p=linux-${HOST_ARCH}")
fi

./repo.sh build -g -r "${ARGS_ARRAY[@]}" || exit $?

# omniSolid USD schema.
#
# The compiled schema (generatedSchema.usda + plugInfo.json) is committed under
# source/. If schema.usda was edited locally we regenerate the committed files
# in source/ with usdGenSchema; then, either way, the compiled files are copied
# into _build so the plugin resources sit alongside the build artifacts (CI
# artifact collection, packaging).
#
# Regeneration needs the Python "jinja2" module. We do NOT vendor or install
# jinja2 (it is not a dependency of this repo); if it is missing when needed we
# warn with install instructions and fall back to the committed compiled schema.
SCHEMA_SRC_DIR="$SCRIPT_DIR/source/schema/omniSolid/resources"
SCHEMA_OUT="$SCRIPT_DIR/_build/schema/omniSolid/resources"
SCHEMA_USDA="$SCHEMA_SRC_DIR/schema.usda"
GENERATED="$SCHEMA_SRC_DIR/generatedSchema.usda"
USD_GEN_SCHEMA="$SCRIPT_DIR/_build/target-deps/usd/release/bin/usdGenSchema"

# system Python is arch-independent (usdGenSchema is a script); developer supplies jinja2.
PYTHON="$(command -v python3 || command -v python || true)"

# Detect a real edit to schema.usda: a working-tree change vs HEAD (so fresh
# checkouts and CI do not regenerate or warn). Fall back to mtime if not a git repo.
schema_needs_regen=0
if git -C "$SCRIPT_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    if ! git -C "$SCRIPT_DIR" diff --quiet HEAD -- "$SCHEMA_USDA" 2>/dev/null; then
        schema_needs_regen=1
    fi
elif [ "$SCHEMA_USDA" -nt "$GENERATED" ]; then
    schema_needs_regen=1
fi

if [ "$schema_needs_regen" = "1" ]; then
    if [ -n "$PYTHON" ] && [ -f "$USD_GEN_SCHEMA" ] && "$PYTHON" -c "import jinja2" >/dev/null 2>&1; then
        echo "prebuild: schema.usda changed; regenerating omniSolid USD schema in source..."
        PYTHONPATH="$SCRIPT_DIR/_build/target-deps/usd/release/lib/python${PYTHONPATH:+:$PYTHONPATH}" \
        LD_LIBRARY_PATH="$SCRIPT_DIR/_build/target-deps/usd/release/lib:$SCRIPT_DIR/_build/target-deps/python/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
        "$PYTHON" "$USD_GEN_SCHEMA" "$SCHEMA_USDA" "$SCHEMA_SRC_DIR"
        echo "prebuild: Schema regeneration complete; remember to commit the updated files in $SCHEMA_SRC_DIR"
    else
        echo "prebuild: WARNING: schema.usda was edited but the compiled schema could not be regenerated." >&2
        echo "prebuild:          usdGenSchema needs the Python 'jinja2' module. Install it with:" >&2
        echo "prebuild:              ${PYTHON:-python3} -m pip install jinja2" >&2
        echo "prebuild:          then re-run prebuild. Using the committed compiled schema for now." >&2
    fi
fi

# Copy the compiled schema from source into _build (always, so _build mirrors source).
mkdir -p "$SCHEMA_OUT"
cp -f "$GENERATED" "$SCHEMA_SRC_DIR/plugInfo.json" "$SCHEMA_OUT/"
echo "prebuild: Copied omniSolid USD schema to $SCHEMA_OUT"
