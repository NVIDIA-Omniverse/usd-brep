-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

-- Dev-only Python bindings (_smlib_tests): in-process prog_test runner for the GUI.
-- No USD / SM_API_USD dependency — build on macOS without OpenUSD (like _smlib_dev).

project "_smlib_tests"
    kind "SharedLib"
    location (workspace_dir.."/%{prj.name}")
    targetprefix ""

    files { "tests_binding/**.cpp", "tests_binding/**.h" }

    includedirs {
        root.."/tests/prog_test/inc",
        root.."/source/SMLib/inc",
    }

    externalincludedirs {
        root.."/_build/target-deps/pybind11/include",
    }

    links { "prog_test", "SMLib" }

    filter { "system:windows" }
        externalincludedirs { root.."/_build/target-deps/python/include" }
        libdirs { root.."/_build/target-deps/python/libs" }
        targetextension ".pyd"
    filter { "system:linux", "platforms:x86_64" }
        externalincludedirs { root.."/_build/target-deps/python/include/python3.12" }
        libdirs { root.."/_build/target-deps/python/lib" }
        links { "python3.12" }
        targetextension ".cpython-312-x86_64-linux-gnu.so"
        buildoptions { "-fPIC", "-fvisibility=hidden" }
    filter { "system:linux", "platforms:aarch64" }
        externalincludedirs { root.."/_build/target-deps/python/include/python3.12" }
        libdirs { root.."/_build/target-deps/python/lib" }
        links { "python3.12" }
        targetextension ".cpython-312-aarch64-linux-gnu.so"
        buildoptions { "-fPIC", "-fvisibility=hidden" }
    filter { "system:macosx" }
        externalincludedirs { root.."/_build/target-deps/python/include/python3.12" }
        libdirs { root.."/_build/target-deps/python/lib" }
        links { "python3.12" }
        targetextension ".cpython-312-darwin.so"
        buildoptions (concat_arrays({"-fPIC", "-fvisibility=hidden"}, macos_common_buildoptions()))
        linkoptions (concat_arrays({"-rpath @loader_path"}, macos_common_linkoptions()))
    filter {}
