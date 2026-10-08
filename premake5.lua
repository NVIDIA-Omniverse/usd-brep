-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

------------------------------------------------------------------------------------------------------------------------
------------------                              Premake utilities                                     ------------------
------------------------------------------------------------------------------------------------------------------------


-- Note: Most of this should probably be relocated to repo_build and
-- loaded from there instead of copying around everywhere. #TODO

include("premake5-public.lua")

------------------------------------------------------------------------------------------------------------------------
------------------                      Other globals beyond premake5-public                          ------------------
------------------------------------------------------------------------------------------------------------------------

target_dir = target_dir or bin_dir
plgs_dir = plgs_dir or target_dir.."/plugins"
python_dir = python_dir or target_deps.."/python"

------------------------------------------------------------------------------------------------------------------------
------------------                          Premake Helpers                                           ------------------
------------------------------------------------------------------------------------------------------------------------

include("premake5-utils.lua")
include("premake5-libraries.lua")

------------------------------------------------------------------------------------------------------------------------
------------------                          The USDRT Workspace                                       ------------------
------------------------------------------------------------------------------------------------------------------------

repo_build.setup_options()

local smlib_no_debug_gfx_check = os.getenv("SMLIB_NO_DEBUG_GFX_CHECK") == "1"
local smlib_target_dir = smlib_no_debug_gfx_check
    and build_dir.."/no-debug-gfx-check/"..platform.."/%{config}"
    or target_dir
local smlib_obj_dir = build_dir.."/intermediate/"..platform.."/%{prj.name}"
if smlib_no_debug_gfx_check then
    smlib_obj_dir = build_dir.."/intermediate/no-debug-gfx-check/"..platform.."/%{prj.name}"
end

workspace "usd-brep"
    location (workspace_dir)
    targetdir (smlib_target_dir)
    configurations { "debug", "release" }
    platforms { "x86_64", "aarch64" }

    if smlib_no_debug_gfx_check then
        defines { "SM_NO_DEBUG_CODE", "SM_NO_GFX_OUTPUT_CODE" }
    end

    filter { "platforms:x86_64" }
        architecture "x86_64"

    -- set prebuild file
    repo_build.set_prebuild_file("_build/generated/prebuild.toml")

    -- usd TBB allocator for linux and windows
    filter { "system:windows", "configurations:debug" }
        defines { "_DEBUG", "NOMINMAX" }
        if not smlib_no_debug_gfx_check then
            defines { "SM_DEBUG_CODE=1", "SM_GFX_CODE=1", "SM_GRAPHICS_CALLBACKS=1" }
        end
        optimize "Off"
        runtime "Debug"
    filter { "system:windows", "configurations:release" }
        -- SM_GRAPHICS_CALLBACKS=1 keeps the SMLib draw-callback dispatch compiled
        -- into release builds so the dev/GUI prog_test draw-event capture works
        -- outside debug. Callbacks default to null, so there is no behavior change
        -- unless a consumer installs one; the dispatch is only reached on
        -- debug/dump draw paths, not hot geometry code.
        defines { "NDEBUG", "NOMINMAX" }
        if not smlib_no_debug_gfx_check then
            defines { "SM_GRAPHICS_CALLBACKS=1" }
        end
        optimize "On"
        runtime "Release"
    filter { "system:linux", "configurations:debug" }
        defines { "_DEBUG", "NOMINMAX" }
        if not smlib_no_debug_gfx_check then
            defines { "SM_DEBUG_CODE=1", "SM_GFX_CODE=1", "SM_GRAPHICS_CALLBACKS=1" }
        end
        optimize "Off"
        runtime "Debug"
    filter { "system:linux", "configurations:release" }
        -- SM_GRAPHICS_CALLBACKS=1: see the windows release note above. Required for
        -- dev/GUI prog_test draw-event capture in the default release workflow.
        defines { "NDEBUG", "NOMINMAX" }
        if not smlib_no_debug_gfx_check then
            defines { "SM_GRAPHICS_CALLBACKS=1" }
        end
        optimize "On"
        runtime "Release"
    filter { "system:macosx", "configurations:debug" }
        defines { "_DEBUG", "NOMINMAX" }
        if not smlib_no_debug_gfx_check then
            defines { "SM_DEBUG_CODE=1", "SM_GFX_CODE=1", "SM_GRAPHICS_CALLBACKS=1" }
        end
        optimize "Off"
        runtime "Debug"
    filter { "system:macosx", "configurations:release" }
        defines { "NDEBUG", "NOMINMAX" }
        if not smlib_no_debug_gfx_check then
            defines { "SM_GRAPHICS_CALLBACKS=1" }
        end
        optimize "On"
        runtime "Release"
    filter {}

    objdir (smlib_obj_dir)
    symbols "On"
    exceptionhandling "On"
    rtti "On"
    staticruntime "Off"
    cppdialect "C++17"

    -- Treat compiler warnings as errors on every platform (MSVC /WX, GCC/Clang -Werror).
    -- NOTE: this premake build emits externalincludedirs as plain -I (no -isystem) and has no
    -- externalwarnings API, so third-party headers are NOT auto-shielded. Warnings from external
    -- SDKs (USD/OCCT/Python/TBB) are suppressed per-consumer via -Wno-*/wd* and defines; any
    -- warning not so suppressed -- first- or third-party -- now fails the build.
    flags { "FatalCompileWarnings" }


    filter { "system:windows" }
        -- add .editorconfig to all projects so that VS 2017 automatically picks it up
        files {".editorconfig"}
        flags { "MultiProcessorCompile", warnings "Extra" }
        editandcontinue "Off"
        setup_msvc_toolchain ()
        -- all of our source strings and executable strings are utf8
        buildoptions {"/utf-8 /wd4244 /wd4305 /wd4267 /bigobj -D_SCL_SECURE_NO_WARNINGS"}
    filter { "system:linux" }
        -- -Wshadow=local mirrors MSVC C4456/C4457, already enforced by /W4 /WX. Not plain
        -- -Wshadow: members and globals add many hits, nearly all Foo(int x) : x(x).
        buildoptions { "-Wall", "-Wshadow=local", "-Wunused-parameter", "-D_GLIBCXX_USE_CXX11_ABI=1"}
        -- ../extraLibs: the OpenUSD runtime bundled in the release package, so its bin/ tools and
        -- lib/ libraries find it without LD_LIBRARY_PATH.
        linkoptions { "-Wl,--disable-new-dtags", "-Wl,-rpath,'$$ORIGIN/../lib'", "-Wl,-rpath,'$$ORIGIN/../extraLibs'" }
	    runpathdirs { target_dir }
    filter { "system:macosx" }
        platforms { "universal" }
        -- Change paths from 'macosx' to 'macos'. Premake likes 'macosx', but the rest of our
        -- repo tools prefer 'macos'
        -- this doesn't work
        if os.outputof('uname -s') == "Darwin" then
            platform = "macos-%{cfg.platform}"
            -- platform = "macos-universal"
            platformDir = path.join(os.getcwd(), "_build/"..platform)
            targetDir = platformDir.."/%{cfg.buildcfg}"
            targetdir (targetDir)
            objdir (build_dir.."/intermediate/"..platform.."/%{prj.name}")
        end
    filter {}


------------------------------------------------------------------------------------------------------------------------
------------------                              projects                                              ------------------
------------------------------------------------------------------------------------------------------------------------


include("source/SMLib/premake5.lua")
include("source/SM_API/premake5.lua")
include("Examples/SM_API/premake5.lua")
include("Examples/SMLib/premake5.lua")
include("tests/SM_API_test/premake5.lua")
include("tests/prog_test/premake5.lua")
include("source/SmPyDevLib/premake5.lua")
include("source/SmPyLib/premake5_tests.lua")

local function include_usd_dependent_stack()
    include("source/BREP_USD_DATA/premake5.lua")
    include("source/USD_BREP_UTILS/premake5.lua")
    include("tests/usd_brep_utils_test/premake5.lua")
    include("source/BREP_STAGING/premake5.lua")
    include("source/OCCT_BREP_IMPORT/premake5.lua")
    include("source/OCCT_BREP_EXPORT/premake5.lua")
    include("tools/occt_to_usd/premake5.lua")
    include("tools/usd_to_occt/premake5.lua")
    include("source/BREP_SM_USD/premake5.lua")
    include("source/SM_API_USD/premake5.lua")
    if usd_has_imaging() then
        include("source/HYDRA_USD_BREP/premake5.lua")
        include("tests/HYDRA_USD_BREP_test/premake5.lua")
    end
    include("tests/SM_API_USD_test/premake5.lua")
    include("source/SmPyLib/premake5.lua")
    include("tests/usd_test/premake5.lua")
    include("tools/smtess_usd/premake5.lua")
    include("tools/brep_geometry_validator/premake5.lua")
end

-- USD/OCCT on macOS when local deps are staged (see setup-macos-local-deps.sh);
-- macos_usd_staged() returns true on non-macOS platforms.
if macos_usd_staged() then
    include_usd_dependent_stack()
end
