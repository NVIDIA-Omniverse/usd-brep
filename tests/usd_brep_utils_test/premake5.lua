-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

-- Unit tests for the kernel-free, pure-math surface of USD_BREP_UTILS:
-- knot-vector conversions (UsdBrepKnots) and the stand-alone analytic / NURBS evaluators
-- and inverse-projection routines (UsdBrepGeometryEval). These need only the USD core
-- libraries at runtime (no BrepArray stage / omniSolid plugin), so this is a plain ConsoleApp.

project "usd_brep_utils_test"
    kind "ConsoleApp"
    location (workspace_dir.."/%{prj.name}")
    staticruntime "Off"

    use_usd()
    use_usd_tbb()
    link_usd_common()

    files { "src/*.cpp" }

    includedirs {
        root.."/source/USD_BREP_UTILS/inc",
        root.."/source/BREP_USD_DATA/inc",
    }
    vpaths { [''] = "*.cpp" }

    -- USD_BREP_UTILS depends on BREP_USD_DATA; list in dependency order for GNU ld's single pass.
    links { "USD_BREP_UTILS", "BREP_USD_DATA" }

    filter { "system:windows" }
        defines { "_CRT_SECURE_NO_WARNINGS" }
        -- /wd4251: the USD_BREP_UTILS views expose std::vector / GfVec2d members across the
        -- DLL boundary; suppress here as the library does for its own TUs (these headers are
        -- included directly, not as external, so the library's suppression doesn't reach us).
        buildoptions { "/W4", "/wd4251" }
    filter { "system:linux" }
        buildoptions { "-ansi -Wall -Wno-deprecated -std=c++17" }
        links { "dl" }
    filter {}
