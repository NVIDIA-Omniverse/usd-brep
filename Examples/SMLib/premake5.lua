-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

project "smlib_step_derivatives_example"
    kind "ConsoleApp"
    location (workspace_dir.."/%{prj.name}")
    use_usd_tbb()
    files { "step_derivatives.cpp", "step_derivatives.h" }
    includedirs { root.."/source/SMLib/inc" }
    links { "SMLib" }
    defines { "SM_STEP_DERIVATIVES_EXAMPLE_STANDALONE" }
    filter { "system:macosx" }
        buildoptions (macos_common_buildoptions())
        linkoptions (macos_common_linkoptions())
    filter {}
