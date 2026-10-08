-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

-- Developer-only Python bindings (_smlib_dev).  Sibling to SmPyLib (_omni_solid);
-- shares binding helpers (SmPyCommon, SmPyError) from source/SmPyLib/src.

project "_smlib_dev"
    kind "SharedLib"
    location (workspace_dir.."/%{prj.name}")
    targetprefix ""

    files {
        "src/**.cpp",
        "src/**.h",
        root.."/source/SmPyLib/src/SmPyError.cpp",
        root.."/source/SmPyLib/src/SmPyError.h",
    }

    includedirs {
        root.."/source/SM_API/inc",
        root.."/source/SMLib/inc",
        root.."/source/SmPyLib/src",
        "src",
    }

    externalincludedirs {
        root.."/_build/target-deps/pybind11/include",
    }

    links { "SMLib", "SM_API" }

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
