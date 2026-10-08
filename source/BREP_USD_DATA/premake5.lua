-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

project "BREP_USD_DATA"
    kind "SharedLib"
    location (workspace_dir.."/%{prj.name}")

    use_usd()
    use_usd_tbb()
    -- Specifies the external libraries required by the binary
    link_usd_common()
    
    files { "src/*.cpp", "inc/*.h" }

    includedirs {
        root.."/source/BREP_USD_DATA/inc"
    }
    vpaths { [''] = "*.cpp" }

    defines { "USDBREP_ENABLE_EXPORTS"  }
        
    filter { "system:windows" }
        buildoptions {"/W4"}
    filter { "system:linux" }
        buildoptions {"-fvisibility=hidden -Wsign-conversion -Wfloat-conversion -Wold-style-cast -Werror" }
    filter {} 
