-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

project "HYDRA_USD_BREP_test_app"
    kind "ConsoleApp"
    location (workspace_dir.."/%{prj.name}")
    staticruntime "Off"
    use_usd()
    use_usd_tbb()
    link_usd_common()
    files { "src/*.cpp", "inc/*.h" }
    includedirs {
        root.."/tests/HYDRA_USD_BREP_test/inc",
        root.."/source/HYDRA_USD_BREP/inc",
        root.."/source/BREP_USD_DATA/inc",
        root.."/source/SM_API/inc",
        root.."/source/SM_API_USD/inc",
        root.."/source/SMLib/inc",
    }
    vpaths { [''] = "*.cpp" }
    links { "HYDRA_USD_BREP", "SM_API_USD", "usd_hd", "usd_hdGp", "usd_usdImaging" }
    filter { "system:linux" }
        buildoptions { "-Wno-deprecated" }
    filter {}
