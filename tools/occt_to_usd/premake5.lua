-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

-- CLI tools for the standalone OCCT `.brep` importer:
--   occt_to_usd  — convert a single `.brep` file into a USD BrepArray.
--   occt_dump    — inspect/parse a `.brep` file and print its record structure.

local occt_includes = {
    root.."/source/OCCT_BREP_IMPORT/inc",
    root.."/source/BREP_STAGING/inc",
    root.."/source/BREP_USD_DATA/inc",
}

-- occt_brep_import references BREP_STAGING symbols, which reference BREP_USD_DATA symbols;
-- list them in dependency order for GNU ld's single pass.
local occt_links = { "OCCT_BREP_IMPORT", "BREP_STAGING", "BREP_USD_DATA" }

project "occt_to_usd"
    kind "ConsoleApp"
    location (workspace_dir.."/%{prj.name}")
    staticruntime "Off"

    use_usd()
    use_usd_tbb()
    link_usd_common()

    files { "occt_to_usd.cpp" }
    includedirs(occt_includes)
    links(occt_links)

    filter { "system:windows" }
        defines { "_CRT_SECURE_NO_WARNINGS" }
        buildoptions { "/W4" }
    filter { "system:linux" }
        buildoptions { "-Wno-deprecated" }
        links { "dl" }
    filter {}

project "occt_dump"
    kind "ConsoleApp"
    location (workspace_dir.."/%{prj.name}")
    staticruntime "Off"

    use_usd()
    use_usd_tbb()
    link_usd_common()

    files { "occt_dump.cpp" }
    includedirs(occt_includes)
    links(occt_links)

    filter { "system:windows" }
        defines { "_CRT_SECURE_NO_WARNINGS" }
        buildoptions { "/W4" }
    filter { "system:linux" }
        buildoptions { "-Wno-deprecated" }
        links { "dl" }
    filter {}
