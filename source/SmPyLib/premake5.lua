-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

-- Stable Python bindings (_omni_solid). Dev-only _smlib_tests lives in premake5_tests.lua.

project "_omni_solid"
    kind "SharedLib"
    location (workspace_dir.."/%{prj.name}")
    targetprefix ""

    files { "src/**.cpp", "src/**.h" }

    includedirs {
        root.."/source/SM_API/inc",
        root.."/source/SM_API_USD/inc",
        root.."/source/SMLib/inc",
        root.."/source/BREP_SM_USD/inc",
        root.."/source/BREP_USD_DATA/inc",
    }

    externalincludedirs {
        root.."/_build/target-deps/pybind11/include",
    }

    links { "SMLib", "SM_API", "SM_API_USD" }

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
        use_python()
    filter { "system:macosx" }
        targetextension ".cpython-312-darwin.so"
        buildoptions (concat_arrays({"-fPIC", "-fvisibility=hidden"}, macos_common_buildoptions()))
        linkoptions (concat_arrays({"-rpath @loader_path"}, macos_common_linkoptions()))
    filter {}
