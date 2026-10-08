-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

-- CLI tool for the standalone OCCT `.brep` exporter:
--   usd_to_occt  — convert a USD file's BrepArray prims into OCCT-format `.brep` files.

local occt_includes = {
    root.."/source/OCCT_BREP_EXPORT/inc",
    root.."/source/OCCT_BREP_IMPORT/inc",
    root.."/source/USD_BREP_UTILS/inc",
    root.."/source/BREP_USD_DATA/inc",
}

-- occt_brep_export references USD_BREP_UTILS symbols, which reference BREP_USD_DATA symbols;
-- list them in dependency order for GNU ld's single pass.
local occt_links = { "OCCT_BREP_EXPORT", "USD_BREP_UTILS", "BREP_USD_DATA" }

project "usd_to_occt"
    kind "ConsoleApp"
    location (workspace_dir.."/%{prj.name}")
    staticruntime "Off"

    use_usd()
    use_usd_tbb()
    link_usd_common()

    files { "usd_to_occt.cpp" }
    includedirs(occt_includes)
    links(occt_links)

    filter { "system:windows" }
        defines { "_CRT_SECURE_NO_WARNINGS" }
        buildoptions { "/W4" }
    filter { "system:linux" }
        buildoptions { "-Wno-deprecated" }
        links { "dl" }
    filter {}
