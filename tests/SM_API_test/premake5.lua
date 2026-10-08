-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

local function configure_sm_api_test_common()
    use_usd_tbb()
    files { "src/*.cpp", "inc/*.h" }
    files { root.."/Examples/SM_API/boolean_bounds.cpp" }
    files { root.."/Examples/SMLib/step_derivatives.cpp" }

    includedirs {
        root.."/tests/SM_API_test/inc",
        root.."/source/SM_API/inc",
        root.."/source/SMLib/inc",
        root.."/Examples/SM_API",
        root.."/Examples/SMLib",
    }

    links { "SMLib", "SM_API" }

    filter { "system:windows" }
        defines { "_CRT_SECURE_NO_WARNINGS" }
        buildoptions {"/W4"}
        pchheader "StdAfx.h"
        pchsource "src/StdAfx.cpp"
        removeflags{ "NoPCH" }
    filter { "system:linux" }
        linkoptions { "-Wl,-rpath,'$$ORIGIN/../../lib'" }
        buildoptions {"-ansi -Wall -Wno-write-strings -std=c++17"}
    filter { "system:macosx" }
        buildoptions (macos_common_buildoptions())
        linkoptions (macos_common_linkoptions())
    filter {}
    filter { "files:**/Examples/SM_API/boolean_bounds.cpp" }
        flags { "NoPCH" }
    filter { "files:**/Examples/SMLib/step_derivatives.cpp" }
        flags { "NoPCH" }
    filter {}
end

project "SM_API_test"
    kind "SharedLib"
    location (workspace_dir.."/%{prj.name}")
    configure_sm_api_test_common()

project "SM_API_test_app"
    kind "ConsoleApp"
    location (workspace_dir.."/%{prj.name}")
    staticruntime "Off"
    configure_sm_api_test_common()
    defines { "SM_API_TEST_STANDALONE" }
