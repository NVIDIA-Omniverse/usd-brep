-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

project "SM_API"
    kind "SharedLib"
    location (workspace_dir.."/%{prj.name}")

    links {
        "SMLib",
    }

    -- Build PCH on Windows
    filter { "system:windows" }
        pchheader "StdAfx.h"
        pchsource "src/StdAfx.cpp"
        files { "src/StdAfx.cpp" }
        removeflags{ "NoPCH" }
    filter {}

    files { "src/*.cpp", "inc/*.h" }
    includedirs {
        root.."/source/SM_API/inc",
        root.."/source/SMLib/inc",
    }
    vpaths { [''] = "*.cpp" }

    defines { "SM_API_EXPORTS" }

    filter { "system:linux" }
        buildoptions {"-Wno-write-strings"}
    filter { "system:macosx" }
        buildoptions (macos_common_buildoptions())
        linkoptions (macos_common_linkoptions())
    filter {}
