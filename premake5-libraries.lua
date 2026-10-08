-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

------------------------------------------------------------------------------------------------------------------------
------------------                                 Overview                                           ------------------
------------------------------------------------------------------------------------------------------------------------

-- This file defines functions that can be called to use a library
-- in a project. This generally sets up the include paths, library path,
-- and sometimes library to link. It's easier than maintaining
-- a mishmash of includedirs and whatever per-project.


------------------------------------------------------------------------------------------------------------------------
------------------                                 Libraries                                          ------------------
------------------------------------------------------------------------------------------------------------------------

function macos_usd_staged()
    if os.target() ~= "macosx" then
        return true
    end
    -- target_deps embeds the literal premake token "%{root}", which os.isdir()
    -- does not expand; use the real absolute root path for the filesystem probe.
    -- Accept either config so debug-only (or debug-first) staging still counts.
    return os.isdir(root .. "/_build/target-deps/usd/release/include/pxr")
        or os.isdir(root .. "/_build/target-deps/usd/debug/include/pxr")
end

-- The OpenUSD 26.08 packages are built without imaging (no Hydra, usdImaging or pxOsd).
function usd_has_imaging()
    return os.isdir(root .. "/_build/target-deps/usd/release/include/pxr/imaging/hd")
        or os.isdir(root .. "/_build/target-deps/usd/debug/include/pxr/imaging/hd")
end

function use_usd()
    use_python()
    externalincludedirs {
        target_deps.."/usd/"..config.."/include",
    }
    includedirs {
        "include",
    }
    libdirs { 
        target_deps.."/usd/"..config.."/lib",
    }

    filter { "system:windows" }
    includedirs { "include", }
    defines { "_CRT_SECURE_NO_WARNINGS" }

    filter { "system:linux" }
    externalincludedirs {
        target_deps.."/usd/"..config.."/include/python",
    }
    runpathdirs { target_deps.."/usd/"..config.."/lib" }

    filter {"system:macosx" }
        externalincludedirs {
            target_deps.."/usd/"..config.."/include/python",
            target_deps.."/python/include/python3.12",
        }
        linkoptions {
            "-rpath @loader_path",
            "-rpath @loader_path/../../target-deps/usd/"..config.."/lib",
            "-rpath @loader_path/../../target-deps/python/lib",
        }

    filter {}
    macos_usd_build_filters()
    -- Note: USD libraries to link are defined per-project via link_usd_common().
end

function use_usd_tbb()
    if os.target() == "macosx" and not macos_usd_staged() then
        return
    end
    -- suppress TBB warnings from the fact it uses old headers
    defines {"TBB_SUPPRESS_DEPRECATED_MESSAGES"}
    libdirs {target_deps.."/usd/"..config.."/lib" }
    externalincludedirs {target_deps.."/usd/"..config.."/include" }
    filter { "configurations:debug" }
        defines { "TBB_USE_DEBUG=1" }
        links { "tbb_debug" }
    filter { "configurations:release" }
        defines { "TBB_USE_DEBUG=0" }
        links { "tbb" }
    filter {}
    filter {"system:macosx" }
        linkoptions {"-rpath @executable_path/../../target-deps/usd/"..config.."/lib"}
    filter {}
end

function link_usd_common()
    USD_LIB_PREFIX = "usd_"
    links { 
        USD_LIB_PREFIX.."ar",
        USD_LIB_PREFIX.."arch",
        USD_LIB_PREFIX.."gf",
        USD_LIB_PREFIX.."js",
        USD_LIB_PREFIX.."kind",
        USD_LIB_PREFIX.."pcp",
        USD_LIB_PREFIX.."plug",
        USD_LIB_PREFIX.."python",
        USD_LIB_PREFIX.."sdf",
        USD_LIB_PREFIX.."sdr",
        USD_LIB_PREFIX.."tf",
        USD_LIB_PREFIX.."trace",
        USD_LIB_PREFIX.."usd",
        USD_LIB_PREFIX.."usdGeom",
        USD_LIB_PREFIX.."usdLux",
        USD_LIB_PREFIX.."usdShade",
        USD_LIB_PREFIX.."usdUtils",
        USD_LIB_PREFIX.."usdVol",
        USD_LIB_PREFIX.."vt",
        USD_LIB_PREFIX.."work",
    }
end

function use_python()
    filter { "system:linux" }
        links { "python3.12"}
        libdirs { target_deps.."/python/lib" }
        runpathdirs { target_deps.."/python/lib" }
        externalincludedirs { target_deps.."/python/include/python3.12" }
    filter { "system:windows" }
        libdirs { target_deps.."/python/libs" }
        externalincludedirs { target_deps.."/python/include" }
    filter { "system:macosx" }
        links { "python3.12" }
        libdirs { target_deps.."/python/lib" }
        linkoptions { "-Wl,-rpath," .. target_deps .. "/python/lib" }
        externalincludedirs { target_deps.."/python/include/python3.12" }
    filter {}
end
