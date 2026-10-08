-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

project "usd_test"
    kind "SharedLib"
    location (workspace_dir.."/%{prj.name}")
    use_usd()
    use_usd_tbb()
    -- Specifies the external libraries required by the binary
    link_usd_common()
    
    files { "src/*.cpp", "inc/*.h" }
    includedirs {
        root.."/source/SMLib/inc",
        root.."/source/BREP_SM_USD/inc",
        root.."/tests/usd_test/inc",
        root.."/source/BREP_USD_DATA/inc",
    }
    vpaths { [''] = "*.cpp" }
    
    links { "SMLib", "BREP_SM_USD", "BREP_USD_DATA" }
    
    debugenvs{ "OMNISOLID_PLUGIN_PATH=../../_build/schema/omniSolid/resources" }

    filter { "system:windows" }
    filter { "system:linux" }
        buildoptions {"-ansi -Wall -Wno-deprecated -std=c++17"}
        links { "dl" }
    filter {}

project "usd_test_app"
    kind "ConsoleApp"
    location (workspace_dir.."/%{prj.name}")
    staticruntime "Off"
    use_usd()
    use_usd_tbb()
    -- Specifies the external libraries required by the binary
    link_usd_common()
      
    -- source code to compile
    files { "src/*.cpp", "inc/*.h" }

    includedirs {
        root.."/source/SMLib/inc",
        root.."/source/BREP_SM_USD/inc",
        root.."/tests/usd_test/inc",
        root.."/source/BREP_USD_DATA/inc",
    }

    links { "SMLib", "BREP_SM_USD", "BREP_USD_DATA" }

    filter { "system:windows" }
        defines { "_CRT_SECURE_NO_WARNINGS"  }
        buildoptions {"/W4"}
    filter { "system:linux" }
        buildoptions {"-ansi -Wall -Wno-deprecated -std=c++17"}
        links { "dl" }
    filter {}

