-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

------------------------------------------------------------------------------------------------------------------------
------------------                                 Overview                                           ------------------
------------------------------------------------------------------------------------------------------------------------

-- This file defines common variables, functions, and project templates that help reduce boilerplate in defining builds.
-- To enable redefining global variables the "x = x or something" pattern is used everywhere.  Thus defining a variable
-- before including that file will override them.


------------------------------------------------------------------------------------------------------------------------
------------------                           Build options                                            ------------------
------------------------------------------------------------------------------------------------------------------------

newoption {
    trigger     = "some-option",
    description = "(Optional) You can specify custom premake command line options like this"
}

-- You can access custom command line options using _OPTIONS["some-option"]

------------------------------------------------------------------------------------------------------------------------
------------------                           Common Helper Functions                                  ------------------
------------------------------------------------------------------------------------------------------------------------

function get_prebuild_files()
    return { os.matchfiles("premake5.lua"), os.matchfiles("deps/*"), os.matchfiles("tools/buildscripts/*.*") }
end

-- Based on current folder return a tuple: (plg_name, plg_tag). E.g. "omni.renderer-ui" -> ("omni.renderer", "ui")
function get_current_plugin_name_and_tag()
    local folder = get_current_dir_name()
    if string.match(folder, "-") then
        return table.unpack(repo_build.split(folder, "-"))
    else
        return folder, ""
    end
end

-- Build plugin id out of name and tag
function get_plugin_id(plg_name, plg_tag)
    local plg_id = plg_name
    if plg_tag ~= "" then
        plg_id = plg_id.."-"..plg_tag
    end
    return plg_id
end

-- Get a table of default paths and info (id, name, tag etc.) for the current plugin.
function get_current_plugin_info()
    local plg = {}

    -- Basics
    plg.name, plg.tag = get_current_plugin_name_and_tag()
    plg.id = get_plugin_id(plg.name, plg.tag)

    -- Target directory where the plugin is built
    plg.target_dir = target_dir

    -- Typical folder where binaries of the plugin go
    plg.bin_dir = plg.target_dir

    -- Project group (for VS)
    plg.group = "plugins"

    return plg
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

-- Clang warnings that dominate legacy SMLib builds on macOS but are not actionable
-- without a large-scale header refactor (SM_COMMON macros, unused debug locals, etc.).
-- Must be appended after -Wall; otherwise -Wall re-enables them.
function macos_warning_suppressions()
    return {
        "-Wno-inconsistent-missing-override",
        "-Wno-overloaded-virtual",
        "-Wno-unused-but-set-variable",
        "-Wno-unused-variable",
        "-Wno-parentheses-equality",
    }
end

-- When SMLIB_MACOS_ARM64_ONLY=1, build arm64-only instead of universal.
function macos_arch_flags()
    local arm64_only = os.getenv("SMLIB_MACOS_ARM64_ONLY")
    if arm64_only ~= nil and arm64_only ~= "" and arm64_only ~= "0" then
        return "-arch arm64"
    end
    return "-arch x86_64 -arch arm64"
end

function macos_common_buildoptions()
    return concat_arrays({
        "-ansi", "-Wall", "-Wno-missing-braces", "-std=c++17",
        macos_arch_flags(),
    }, macos_warning_suppressions())
end

function macos_common_linkoptions()
    return { macos_arch_flags() }
end

function macos_usd_build_filters()
    -- Do not add -fvisibility=hidden here: dylib targets that share header-instantiated
    -- types across library edges need default visibility for vtable linking on macOS.
    filter { "system:macosx" }
        buildoptions (concat_arrays({"-Wno-deprecated"}, macos_common_buildoptions()))
        linkoptions (macos_common_linkoptions())
    filter {}
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

-- ----------------------------------------------------------------------
-- Lua has no easy sorting function so this function provides an iterator ordered by key name
-- @tableToIterate: Table to be iterated
-- @sortFunction: Alternate sort function, use lexical ordering if not specified
-- @return: Function that iterates over a sorted version of the table
function pairsSortedByKeys (tableToIterate, sortFunction)
    local sortedTable = {}
    for index in pairs(tableToIterate) do
        table.insert(sortedTable, index)
    end
    table.sort(sortedTable, sortFunction)
    local i = 0
    local iter = function ()   -- iterator function
        i = i + 1
        if sortedTable[i] == nil then
            return nil
        else
            return sortedTable[i], tableToIterate[sortedTable[i]]
        end
    end
    return iter
end

-- ----------------------------------------------------------------------
-- Helper function to print a string representation of a table
-- @tableToPrint: Table whose contents are to be sorted and printed
-- @indentLevel: sets the initial level of indentation.
function printTable (tableToPrint, indentLevel)
    if not indentLevel then
        indentLevel = 0
    end

    for key, value in pairsSortedByKeys(tableToPrint) do
        formatting = string.rep("    ", indentLevel) .. key .. ": "
        if type(value) == "table" then
            print(formatting)
            printTable(value, indentLevel + 1)
        elseif type(value) == 'boolean' then
            print(formatting .. tostring(value))
        else
            print(formatting .. value)
        end
    end
end

