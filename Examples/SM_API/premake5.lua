-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

project "sm_api_boolean_bounds_example"
    kind "ConsoleApp"
    location (workspace_dir.."/%{prj.name}")
    use_usd_tbb()
    files { "boolean_bounds.cpp", "boolean_bounds.h" }
    includedirs { root.."/source/SM_API/inc", root.."/source/SMLib/inc" }
    links { "SM_API", "SMLib" }
    defines { "SM_BOOLEAN_BOUNDS_EXAMPLE_STANDALONE" }
    filter { "system:macosx" }
        buildoptions (macos_common_buildoptions())
        linkoptions (macos_common_linkoptions())
    filter {}
