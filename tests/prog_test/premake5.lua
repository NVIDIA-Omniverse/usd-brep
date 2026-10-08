-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

project "prog_test"
    kind "SharedLib"
    location (workspace_dir.."/%{prj.name}")
    staticruntime "Off"
    use_usd_tbb()

    -- source code to compile
    files { "src/*.cpp", "inc/*.h" }
    removefiles { "src/prog_test_app.cpp" }

    includedirs {
    root.."/tests/prog_test/inc",
	root.."/source/SMLib/inc",
    }

    links {'SMLib'}
    
    filter { "system:windows" }
        defines { "_CRT_SECURE_NO_WARNINGS", "SM_CLASSIFY_UPDATE", "SM_TOLERANT_WATCH"}
        buildoptions {"/W4 /bigobj"}
	links { }
    filter { "system:linux" }
        buildoptions {"-ansi -Wall -std=c++17"}
    filter { "system:macosx" }
        buildoptions (macos_common_buildoptions())
	linkoptions (macos_common_linkoptions())
    filter {}

project "prog_test_app"
    kind "ConsoleApp"
    location (workspace_dir.."/%{prj.name}")
    staticruntime "Off"
    use_usd_tbb()

    -- source code to compile
    files { "src/prog_test_app.cpp" }

    includedirs {
      root.."/tests/prog_test/inc",
	root.."/source/SMLib/inc",
    }

    links {'prog_test', 'SMLib',}
    
    filter { "system:windows" }
        defines { "_CRT_SECURE_NO_WARNINGS", "SM_CLASSIFY_UPDATE", "SM_TOLERANT_WATCH" }
        buildoptions {"/W4 /bigobj"}
    filter { "system:linux" }
        buildoptions {"-ansi -Wall -std=c++17"}
    filter { "system:macosx" }
        buildoptions (macos_common_buildoptions())
        linkoptions (macos_common_linkoptions())
    filter {}
