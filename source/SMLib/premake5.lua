-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

project "SMLib"
    kind "SharedLib"
    location (workspace_dir.."/%{prj.name}")
    use_usd_tbb()
    
    -- Build PCH on Windows
    filter { "system:windows" }
        pchheader "StdAfx.h"
        pchsource "src/StdAfx.cpp"
        files { "src/StdAfx.cpp" }
        removeflags{ "NoPCH" }
    filter {}

    -- Regular build files
    files { "src/*.cpp", "inc/*.h" }
    includedirs {
        root.."/source/SMLib/inc",
    }
    vpaths { [''] = "*.cpp" }
    
    -- Compiler flags
    filter { "system:windows" }
        defines { "NL_DEBUG_CODE" }
        buildoptions {}
    filter {"configurations:debug", "system:windows"}
        links { "tbbmalloc_debug","tbb_debug"}
    filter {"configurations:release", "system:windows"}
        links { "tbbmalloc","tbb"}
    filter {"configurations:debug", "system:linux"}
        links { "tbbmalloc_debug","tbb_debug"}
    filter {"configurations:release", "system:linux"}
        links { "tbbmalloc","tbb"}
    filter { "system:macosx" }
        buildoptions (macos_common_buildoptions())
        linkoptions (macos_common_linkoptions())
    filter {}






