-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

project "BREP_SM_USD"
    kind "SharedLib"
    location (workspace_dir.."/%{prj.name}")
    use_usd()
    use_usd_tbb()
    -- Specifies the external libraries required by the binary
    link_usd_common()

    files { "src/*.cpp", "inc/*.h" }
    includedirs {
        root.."/source/BREP_SM_USD/inc",
        root.."/source/SMLib/inc",
        root.."/source/BREP_USD_DATA/inc",
    }
    vpaths { [''] = "*.cpp" }

    links { "SMLib", "BREP_USD_DATA" }

    filter { "system:windows" }
        defines { "SMU_ENABLE_EXPORTS"  }
    filter { "system:linux" }
        buildoptions {"-Wno-deprecated"}
    filter {}

