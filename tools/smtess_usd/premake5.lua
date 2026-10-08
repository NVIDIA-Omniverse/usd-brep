-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

project "smtess_usd"
    kind "ConsoleApp"
    location (workspace_dir.."/%{prj.name}")
    staticruntime "Off"
    use_usd()
    use_usd_tbb()
    link_usd_common()

    files { "src/*.cpp" }

    includedirs {
        root.."/source/SMLib/inc",
        root.."/source/BREP_SM_USD/inc",
        root.."/source/BREP_USD_DATA/inc",
    }

    links { "SMLib", "BREP_SM_USD", "BREP_USD_DATA" }

    filter { "system:windows" }
        defines { "_CRT_SECURE_NO_WARNINGS" }
        buildoptions {"/W4"}
    filter { "system:linux" }
        buildoptions {"-Wno-deprecated"}
        links { "dl" }
    filter {}
