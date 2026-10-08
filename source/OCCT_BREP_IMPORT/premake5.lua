-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

-- Standalone importer for OCCT-format ASCII/binary `.brep` files. Unlike the PRC front end this
-- links no kernel SDK: it parses the `.brep` stream directly, resolves the topology, and drives the
-- shared source-neutral BREP_STAGING pipeline to author USD BrepArray prims.

project "OCCT_BREP_IMPORT"
    kind "StaticLib"
    location (workspace_dir.."/%{prj.name}")

    use_usd()
    use_usd_tbb()

    files { "src/*.cpp", "inc/*.h" }

    includedirs {
        root.."/source/OCCT_BREP_IMPORT/inc",
        root.."/source/BREP_STAGING/inc",   -- source-neutral staging data structures + USD writer
        root.."/source/BREP_USD_DATA/inc",  -- UsdBrep arrays (pulled in transitively by the staging writer)
    }
    vpaths { [''] = "*.cpp" }

    -- The standalone parser uses pragmatic C-style numeric parsing, so it omits the strict
    -- conversion/-Werror set the generic brep libs enforce.
    filter { "system:windows" }
        buildoptions {"/W4"}
    filter { "system:linux" }
        buildoptions {"-fPIC -fvisibility=hidden -Wall -std=c++17" }
    filter {}
