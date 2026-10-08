-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

------------------------------------------------------------------------------------------------------------------------
------------------                                 Overview                                           ------------------
------------------------------------------------------------------------------------------------------------------------

-- This file defines common variables and helper functions to be used both inside SDK projects and outside.
-- External repos can include that file, redefine some variables and use common functions.
-- To enable redefining global variables "x = x or something" pattern used everywhere. Thus defining a variable before
-- including that file will override them.


------------------------------------------------------------------------------------------------------------------------
------------------                              premake Options                                       ------------------
------------------------------------------------------------------------------------------------------------------------

newoption {
    trigger     = "platform-host",
    description = "(Optional) Specify host platform for cross-compilation"
}

------------------------------------------------------------------------------------------------------------------------
------------------                              omni.repo.build                                       ------------------
------------------------------------------------------------------------------------------------------------------------

-- Shared build scripts from repo_build package, explicit path:
repo_build = repo_build or require("_repo/deps/repo_build/lua/omni/repo/build")

-- Enable /sourcelink flag for VS
repo_build.enable_vstudio_sourcelink()

-- Remove /JMC parameter for visual studio
repo_build.remove_vstudio_jmc()

-- Packman pins different premake builds per host OS (see deps/host-deps.packman.xml):
--   windows/linux -> 5.0.0-beta2+nv1
--   macOS         -> 5.0.0-beta1+nv1-macos-universal
-- beta1 does not expose externalincludedirs, but premake5-libraries.lua and this file
-- call it for USD/Python/MSVC paths. Map to includedirs on macOS only so gmake2
-- generation succeeds; Linux/Windows keep the real API from beta2.
if externalincludedirs == nil then
    externalincludedirs = includedirs
end


------------------------------------------------------------------------------------------------------------------------
------------------                              Common Variables                                      ------------------
------------------------------------------------------------------------------------------------------------------------

local current_file_dir = repo_build.get_abs_path(".")

-- Repo root
root = root or current_file_dir

-- Target platform name, e.g. windows-x86_64
platform = platform or "%{cfg.system}-%{cfg.platform}"

-- Host platform name, e.g. windows-x86_64
platform_host = platform_host or _OPTIONS["platform-host"] or platform;

-- Target config, e.g. debug, release
config = config or "%{cfg.buildcfg}"

-- Constant with all configurations we build
ALL_CONFIGS = ALL_CONFIGS or { "debug", "release" }

-- Folder to store solution in. _ACTION is compilation target, e.g.: vs2017, make etc.
workspace_dir = workspace_dir or "%{root}/_compiler/".._ACTION

-- Various commonly used repo paths:
build_dir = build_dir or "%{root}/_build"
target_deps = target_deps or build_dir.."/target-deps"
host_deps = host_deps or build_dir.."/host-deps"
bin_dir = bin_dir or build_dir.."/%{platform}/%{config}"
exts_dir = exts_dir or bin_dir.."/exts"

-- "dev" folder is where current file is. Root for local build and "dev" folder SDK package. It is used for build-time paths,
-- like `include`, `deps` folders etc.
dev_dir = dev_dir or current_file_dir

-- Define the canonical way to call the bundled version of Python
if os.target() == "windows" then
    -- Windows custom build tools merge everything together into one script so the Python batch file
    -- has to be "called" or it will only run the first one.
    pythonScriptPath = "call "..dev_dir.."/tools/packman/python.bat"
else
    -- path.getabsolute gives the realpath rather than the symbolic path in linux
    pythonScriptPath = dev_dir.."/tools/packman/python.sh"
end

------------------------------------------------------------------------------------------------------------------------
------------------                           Common Helper Functions                                  ------------------
------------------------------------------------------------------------------------------------------------------------

-- Returns absolute path to folder where currently executed premake lua file is located.
function get_current_lua_file_dir()
    return path.getabsolute(".")
end

-- Returns folder name where currently executed premake lua file is located.
function get_current_dir_name()
    return path.getname(get_current_lua_file_dir())
end

-- Get table value by key or default
function get_value_or_default(table, key, default)
    if table[key] ~= nil then
        return table[key]
    end
    return default
end

-- Merge table2 into table1
function merge_tables(table1, table2)
    for k,v in pairs(table2) do table1[k] = v end
    return table1
end

-- Concat array1 + array2
function concat_arrays(array1, array2)
    for _, v in pairs(array2) do table.insert(array1, v) end
    return array1
end

-- String formatting with interpolation, e.g.: string_fmt_vars("${a}+%{b}", {a=2, b=3}) => "2+3"
-- both ${} and %{} syntax support
function string_fmt_vars(str, vars)
    return (str:gsub('([$%%]%b{})', function(w) return vars[w:sub(3, -2)] or w end))
end

-- String formatting with interpolation multiple times for recursive subsituition,
-- e.g.: string_fmt_vars_recursive("${a}", {a="${b}", b=3}) => "3"
function string_fmt_vars_recursive(str, vars)
    local s0 = str
    for i=1,16 do
        local s1 = string_fmt_vars(s0, vars)
        if s0 == s1 then break end
        s0 = s1
    end
    return s0
end

function is_string_empty(s)
    return s == nil or s == ''
end

-- Add folder with source files to solution under specified vpath/ virtual group (helper)
-- @vpath: Virtual folder in VS project
-- @dir: Directory with files
-- @pattern: Wildcard pattern to search for files with. By default **. Can be just a file name.
function add_files(vpath, dir, pattern)
    local pattern = pattern or "**"
    files { dir.."/"..pattern }
    vpaths {
        [vpath.."/*"] = dir.."/*"
    }
end

-- Configure the MSVC toolchain. The compiler and Windows SDK come from the host Visual Studio
-- install (msbuild.link_host_toolchain in repo.toml), so MSBuild resolves the paths itself.
function setup_msvc_toolchain()
    if os.target() ~= "windows" then
        return
    end
    -- SDK and toolset follow the host install and msbuild.vs_version, rather than pinned here.
    systemversion "latest"
end
