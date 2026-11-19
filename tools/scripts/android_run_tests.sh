#!/usr/bin/env bash
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Run the Android kernel test apps (prog_test, SM_API_test, sms_bugz) on a connected device or
# emulator with adb. Build first (BUILDING.md, "Android"). Usage: android_run_tests.sh [release|debug]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd -P)"
cd "${REPO_ROOT}"

config="${1:-release}"
build_dir="_build/android-aarch64/${config}"
# The repo layout is mirrored on the device so the tests' relative paths to TestFiles work.
device_root=/data/local/tmp/smlib
adb="${ANDROID_HOME:+${ANDROID_HOME}/platform-tools/}adb"

if [[ ! -x "${build_dir}/prog_test_app" ]]; then
    echo "No Android ${config} build in ${build_dir}." >&2
    exit 1
fi
libcxx="${ANDROID_NDK_ROOT:?Set ANDROID_NDK_ROOT to the NDK}/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so"

"${adb}" wait-for-device
"${adb}" shell "rm -rf ${device_root} && mkdir -p ${device_root}/_build/android-aarch64 ${device_root}/tests/prog_test ${device_root}/tests/SM_API_test ${device_root}/source/sms_bugz"
"${adb}" push -q "${build_dir}" "${device_root}/_build/android-aarch64/"
"${adb}" push -q "${libcxx}" "${device_root}/${build_dir}/"
"${adb}" push -q TestFiles "${device_root}/"

failed=0
for test in tests/prog_test:prog_test_app tests/SM_API_test:SM_API_test_app source/sms_bugz:sms_bugz_app; do
    dir="${test%%:*}"
    app="${test##*:}"
    echo "Running ${app} (${config})"
    # adb shell returns the remote exit status.
    if "${adb}" shell "cd ${device_root}/${dir} && ../../${build_dir}/${app}"; then
        echo "${app} passed"
    else
        echo "${app} failed"
        failed=1
    fi
done
exit ${failed}
