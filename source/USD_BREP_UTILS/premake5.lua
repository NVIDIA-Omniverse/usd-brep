-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

-- Kernel-agnostic helpers for reading and iterating USD BrepArray prims: topology iteration
-- (UsdBrepView/UsdBrepIterator), analytic geometry evaluation, NURBS knot-vector conversions,
-- topology queries, packing and stage utilities.

project "USD_BREP_UTILS"
    kind "SharedLib"
    location (workspace_dir.."/%{prj.name}")

    use_usd()
    use_usd_tbb()
    -- Specifies the external libraries required by the binary
    link_usd_common()

    files { "src/*.cpp", "inc/*.h" }

    includedirs {
        root.."/source/USD_BREP_UTILS/inc",
        root.."/source/BREP_USD_DATA/inc",
    }
    vpaths { [''] = "*.cpp" }

    defines { "USD_BREP_ENABLE_EXPORTS" }

    links { "BREP_USD_DATA" }

    filter { "system:windows" }
        -- /wd4251: exported views expose std::vector members across the DLL boundary.
        buildoptions {"/W4", "/wd4251"}
    filter { "system:linux" }
        -- -Wno-deprecated: a USD header transitively includes the deprecated <ext/hash_set>,
        -- whose #warning would otherwise escalate to an error under -Werror.
        -- -Wno-psabi: the static NURBS evaluators return std::pair<GfVec2d/3d, double> by
        -- value; GCC 10.1 changed how these small aggregates are passed vs. GCC <7.1. The
        -- note is a cross-compiler ABI advisory only and is irrelevant here (the functions
        -- have internal linkage and the whole build uses one toolchain), so silence the noise.
        buildoptions {"-fvisibility=hidden -Wsign-conversion -Wfloat-conversion -Wold-style-cast -Werror -Wno-deprecated -Wno-psabi" }
    filter {}
