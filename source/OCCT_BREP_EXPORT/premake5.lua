-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

-- Standalone exporter: turns a USD BrepArray (UsdBrepArrayData) into OCCT-format ASCII `.brep`
-- text. Like OCCT_BREP_IMPORT it links no kernel SDK; it consumes USD via USD_BREP_UTILS and
-- reuses the importer's record structs (occbrep::SBrepModel) for symmetry.

project "OCCT_BREP_EXPORT"
    kind "StaticLib"
    location (workspace_dir.."/%{prj.name}")

    use_usd()
    use_usd_tbb()

    files { "src/*.cpp", "inc/*.h" }

    includedirs {
        root.."/source/OCCT_BREP_EXPORT/inc",
        root.."/source/OCCT_BREP_IMPORT/inc",  -- occbrep::SBrepModel records (header-only here)
        root.."/source/USD_BREP_UTILS/inc",    -- UsdBrepView iteration + geometry helpers
        root.."/source/BREP_USD_DATA/inc",     -- UsdBrepArrayData + tokens
    }
    vpaths { [''] = "*.cpp" }

    filter { "system:windows" }
        -- /wd4251: UsdBrepView (from USD_BREP_UTILS) exposes std::vector members across the DLL
        -- boundary; the same suppression is used by USD_BREP_UTILS itself.
        buildoptions {"/W4", "/wd4251"}
    filter { "system:linux" }
        buildoptions {"-fPIC -fvisibility=hidden -Wall -std=c++17" }
    filter {}
