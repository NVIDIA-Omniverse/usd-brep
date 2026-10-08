-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

-- Point the plugin manifest at the built library.
local library = "libHYDRA_USD_BREP.so"
if os.target() == "windows" then
    library = "HYDRA_USD_BREP.dll"
elseif os.target() == "macosx" then
    library = "libHYDRA_USD_BREP.dylib"
end
local manifest_dir = root.."/_build/generated/hdUsdBrep"
os.mkdir(manifest_dir)
local input = assert(io.open(root.."/source/HYDRA_USD_BREP/resources/plugInfo.json", "r"))
local manifest = input:read("*a")
input:close()
manifest = manifest:gsub("@PLUG_INFO_LIBRARY_PATH@", "../../"..library)
local output = assert(io.open(manifest_dir.."/plugInfo.json", "w"))
output:write(manifest)
output:close()

project "HYDRA_USD_BREP"
    kind "SharedLib"
    location (workspace_dir.."/%{prj.name}")
    use_usd()
    use_usd_tbb()
    link_usd_common()
    files { "src/*.cpp", "inc/*.h", "resources/plugInfo.json", "python/**.py" }
    vpaths { [''] = "*.cpp" }
    defines { "HDUSDBREP_EXPORTS" }
    includedirs {
        root.."/source/HYDRA_USD_BREP/inc",
        root.."/source/SM_API/inc",
        root.."/source/SM_API_USD/inc",
        root.."/source/SMLib/inc",
        root.."/source/BREP_USD_DATA/inc",
    }
    links {
        "SM_API_USD",
        "usd_hd", "usd_hdGp", "usd_usdImaging", "usd_hf", "usd_pxOsd",
    }
    postbuildcommands {
        '{MKDIR} "%{cfg.targetdir}/plugins/hdUsdBrep"',
        '{COPYFILE} "'..manifest_dir..'/plugInfo.json" "%{cfg.targetdir}/plugins/hdUsdBrep/plugInfo.json"',
    }
    filter { "files:python/**.py" }
        buildcommands {
            '{MKDIR} "%{cfg.targetdir}/hdUsdBrepUsdview"',
            '{COPYFILE} "%{file.abspath}" "%{cfg.targetdir}/hdUsdBrepUsdview/%{file.name}"',
        }
        buildoutputs { "%{cfg.targetdir}/hdUsdBrepUsdview/%{file.name}" }
    filter { "system:windows" }
        -- USD 25.11 registry constructors must survive MSVC unused-data elimination.
        -- Set via the project property (/Zc:inline-) rather than buildoptions, which
        -- would add a second, conflicting flag after MSBuild's default /Zc:inline (D9025).
        removeunreferencedcodedata "Off"
    filter { "system:linux" }
        buildoptions { "-Wno-deprecated" }
    filter {}
