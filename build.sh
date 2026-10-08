#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
set -e

SCRIPT_DIR="$(dirname "${BASH_SOURCE}")"
source "$SCRIPT_DIR/repo.sh" build "$@" || exit $?
