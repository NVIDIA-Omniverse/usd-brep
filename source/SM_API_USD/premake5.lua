-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

project "SM_API_USD"
    kind "SharedLib"
    location (workspace_dir.."/%{prj.name}")
    use_usd()
    use_usd_tbb()
    link_usd_common()

    files { "src/*.cpp", "inc/*.h" }
    includedirs {
        root.."/source/SM_API_USD/inc",
        root.."/source/SM_API/inc",
        root.."/source/SMLib/inc",
        root.."/source/BREP_SM_USD/inc",
        root.."/source/BREP_USD_DATA/inc",
    }
    vpaths { [''] = "*.cpp" }

    links { "SMLib", "SM_API", "BREP_SM_USD", "BREP_USD_DATA" }

    defines { "SM_API_USD_EXPORTS" }

    filter { "system:windows" }
        buildoptions {"/W4"}
    filter { "system:linux" }
        buildoptions {"-Wno-deprecated"}
    filter {}

-- Stage the OpenUSD runtime for the release package, as usd-optimize does: repo.toml ships
-- extraLibs/ (OpenUSD and Python runtime libraries plus OpenUSD's plugin resources) and
-- OpenUSD's pxr modules as usdpy/, so the package runs without an external OpenUSD.
local extra_libs_dir = target_dir.."/extraLibs"
repo_build.prebuild_copy {
    {target_deps.."/usd/%{config}/lib/usd", extra_libs_dir.."/usd"},
    -- OpenUSD's Hydra renderers (Storm) and image readers, which usdview needs. OpenUSD finds
    -- them through its built-in "../plugin/usd" path, relative to its libraries in extraLibs/.
    {target_deps.."/usd/%{config}/plugin/usd", target_dir.."/plugin/usd"},
}
if os.target() == "windows" then
    repo_build.prebuild_copy {
        {target_deps.."/usd/%{config}/lib/*.dll", extra_libs_dir},
        -- OpenUSD 25.11 keeps some runtime DLLs (TBB, MaterialX) in bin/.
        {target_deps.."/usd/%{config}/bin/*.dll", extra_libs_dir},
        {target_deps.."/python/python*.dll", extra_libs_dir},
    }
else
    repo_build.prebuild_copy {
        {target_deps.."/usd/%{config}/lib/*.so*", extra_libs_dir},
        {target_deps.."/python/lib/libpython*.so*", extra_libs_dir},
    }
end
