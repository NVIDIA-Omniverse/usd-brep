-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

-- brep_geometry_validator — CLI: import USD BrepArray prims into SmBreps and run
-- SmBrep::AssertValid (kernel-level geometry/topology validation). Sibling of the schema-level
-- brep_validator: the schema validator checks the USD data, this checks the built solid.

project "brep_geometry_validator"
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

    -- BREP_SM_USD references SMLib + BREP_USD_DATA symbols; list in dependency order for GNU ld.
    links { "BREP_SM_USD", "SMLib", "BREP_USD_DATA" }

    filter { "system:windows" }
        defines { "_CRT_SECURE_NO_WARNINGS" }
        buildoptions { "/W4" }
    filter { "system:linux" }
        buildoptions { "-Wno-deprecated" }
        links { "dl" }
    filter {}
