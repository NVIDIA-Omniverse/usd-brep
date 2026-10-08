-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

project "BREP_STAGING"
    kind "StaticLib"
    location (workspace_dir.."/%{prj.name}")

    use_usd()
    use_usd_tbb()

    files { "src/*.cpp", "inc/*.h" }

    includedirs {
        root.."/source/BREP_STAGING/inc",
        root.."/source/BREP_USD_DATA/inc"
    }
    vpaths { [''] = "*.cpp" }

    filter { "system:windows" }
        buildoptions {"/W4"}
    filter { "system:linux" }
        buildoptions {"-fPIC -fvisibility=hidden -Wall -std=c++17 -Wsign-conversion -Wfloat-conversion -Wold-style-cast -Wno-deprecated -Werror" }
    filter {}
