-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

local function configure_sm_api_usd_test_common()
    use_usd()
    use_usd_tbb()
    link_usd_common()

    files { "src/*.cpp", "inc/*.h" }

    includedirs {
        root.."/tests/SM_API_USD_test/inc",
        root.."/source/SM_API_USD/inc",
        root.."/source/SM_API/inc",
        root.."/source/SMLib/inc",
        root.."/source/BREP_SM_USD/inc",
        root.."/source/BREP_USD_DATA/inc",
    }

    links { "SMLib", "SM_API", "SM_API_USD", "BREP_SM_USD", "BREP_USD_DATA" }

    debugenvs{ "OMNISOLID_PLUGIN_PATH=../../_build/schema/omniSolid/resources" }

    filter { "system:windows" }
        defines { "_CRT_SECURE_NO_WARNINGS" }
        buildoptions {"/W4"}
    filter { "system:linux" }
        linkoptions { "-Wl,-rpath,'$$ORIGIN/../../lib'" }
        buildoptions {"-ansi -Wall -Wno-deprecated -std=c++17"}
    filter {}
end

project "SM_API_USD_test"
    kind "SharedLib"
    location (workspace_dir.."/%{prj.name}")
    configure_sm_api_usd_test_common()

project "SM_API_USD_test_app"
    kind "ConsoleApp"
    location (workspace_dir.."/%{prj.name}")
    staticruntime "Off"
    configure_sm_api_usd_test_common()
    defines { "SM_API_USD_TEST_STANDALONE" }
