// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "SmPyError.h"

#include <stdexcept>
#include <string>
#include <cstring>
#include <atomic>

#include <SmThreadLocalStorage.h>
#include <SmTypes.h>

// ---------------------------------------------------------------------------
//  SmStatus -> identifier table.  Mirrors source/SMLib/inc/SmMessages.h.
//  Keep alphabetised by code value so future inserts diff cleanly.
// ---------------------------------------------------------------------------
namespace
{
    struct StatusEntry { SmStatus code; const char* name; };

    constexpr StatusEntry kStatusTable[] = {
        { SM_SUCCESS,                       "SM_SUCCESS"                     },
        { SM_ERR,                           "SM_ERR"                         },
        { SM_ERR_OUT_OF_MEMORY,             "SM_ERR_OUT_OF_MEMORY"           },
        { SM_ERR_UNKNOWN,                   "SM_ERR_UNKNOWN"                 },
        { SM_ERR_FATAL,                     "SM_ERR_FATAL"                   },
        { SM_ERR_ASSERT_FAILURE,            "SM_ERR_ASSERT_FAILURE"          },
        { SM_ERR_NULL_POINTER,              "SM_ERR_NULL_POINTER"            },
        { SM_ERR_INVALID_INPUT,             "SM_ERR_INVALID_INPUT"           },
        { SM_ERR_NON_NULL_OUTPUT_POINTER,   "SM_ERR_NON_NULL_OUTPUT_POINTER" },
        { SM_ERR_ASSERTVALID_FAILURE,       "SM_ERR_ASSERTVALID_FAILURE"     },
        { SM_ERR_OUTSIDE_OF_DOMAIN,         "SM_ERR_OUTSIDE_OF_DOMAIN"       },
        { SM_ERR_NOT_WITHIN_TOLERANCE,      "SM_ERR_NOT_WITHIN_TOLERANCE"    },
        { SM_ERR_NOT_CONVERGING,            "SM_ERR_NOT_CONVERGING"          },
        { SM_ERR_WARNING,                   "SM_ERR_WARNING"                 },
        { SM_ERR_MESSAGE,                   "SM_ERR_MESSAGE"                 },
        { SM_ERR_AXIS_INSIDE_FACE,          "SM_ERR_AXIS_INSIDE_FACE"        },
        { SM_ERR_LICENSE_EXPIRED,           "SM_ERR_LICENSE_EXPIRED"         },
        { SM_ERR_BAD_INTERSECTIONS,         "SM_ERR_BAD_INTERSECTIONS"       },
        { SM_ERR_BAD_COINCIDENT_VERTICES,   "SM_ERR_BAD_COINCIDENT_VERTICES" },
        { SM_ERR_BAD_SURFACE_POINT,         "SM_ERR_BAD_SURFACE_POINT"       },
        { SM_ERR_BAD_TANGENT_DROP,          "SM_ERR_BAD_TANGENT_DROP"        },
        { SM_ERR_DEGENERATE_SURFACE,        "SM_ERR_DEGENERATE_SURFACE"      },
        { SM_ERR_BAD_FIND_DEGEN_PARAM,      "SM_ERR_BAD_FIND_DEGEN_PARAM"    },
        { SM_ERR_METHOD_FAILURE_QUITING,    "SM_ERR_METHOD_FAILURE_QUITING"  },
        { SM_ERR_NOTYET_HEAL_FACE,          "SM_ERR_NOTYET_HEAL_FACE"        },
        { SM_ERR_TESS_FAIL_FACE,            "SM_ERR_TESS_FAIL_FACE"          },
    };
}

const char* sm_status_name(SmStatus s)
{
    for (const auto& e : kStatusTable) {
        if (e.code == s) return e.name;
    }
    return "SM_ERR_???";
}

namespace
{
    // Strip directory prefix from a path so trace messages stay compact.
    std::string basenameOf(const std::string& path)
    {
        auto p = path.find_last_of("/\\");
        return (p == std::string::npos) ? path : path.substr(p + 1);
    }

    // Soft cap on per-call trail size.  An error storm in one CHECK_STATUS
    // shouldn't balloon the exception message.
    constexpr size_t kMaxTrailBytes = 4096;
}

// ---------------------------------------------------------------------------
//  Thread-local trail.
// ---------------------------------------------------------------------------
namespace
{
    thread_local std::string t_trail;
    thread_local bool        t_trailTruncated = false;
}

void SmPyErrorTrail::clear()
{
    t_trail.clear();
    t_trailTruncated = false;
}

void SmPyErrorTrail::append(SmStatus code, const char* file, unsigned long line, const char* msg)
{
    if (t_trailTruncated) return;

    std::string entry = sm_status_name(code);
    if (file && *file) {
        entry += " at ";
        entry += basenameOf(file);
        entry += ":";
        entry += std::to_string(line);
    }
    if (msg && *msg) {
        entry += " (";
        entry += msg;
        entry += ")";
    }

    if (!t_trail.empty()) t_trail += "; ";
    t_trail += entry;

    if (t_trail.size() >= kMaxTrailBytes) {
        t_trail += " ...[truncated]";
        t_trailTruncated = true;
    }
}

std::string SmPyErrorTrail::take()
{
    std::string out;
    out.swap(t_trail);
    t_trailTruncated = false;
    return out;
}

// ---------------------------------------------------------------------------
//  Identifier extraction from the macro-stringified expression.
//  Input  : "SmApiCreatePipeSweep(radius, bsp, cap ? TRUE : FALSE, r)"
//  Output : "SmApiCreatePipeSweep"
//  Falls back to the full expression if no '(' is present.
// ---------------------------------------------------------------------------
namespace
{
    std::string extractCEntryName(const char* exprText)
    {
        if (!exprText) return "<unknown>";
        const char* p = exprText;
        while (*p == ' ' || *p == '\t') ++p;
        const char* start = p;
        while (*p && *p != '(' && *p != ' ' && *p != '\t') ++p;
        if (p == start) return std::string(exprText);
        return std::string(start, p - start);
    }
}

// ---------------------------------------------------------------------------
//  SmPyRaise -- format and throw.
// ---------------------------------------------------------------------------
[[noreturn]] void SmPyRaise(SmStatus code,
                            const char* exprText,
                            const char* binderFile,
                            int binderLine)
{
    std::string entry = extractCEntryName(exprText);
    std::string trail = SmPyErrorTrail::take();

    std::string msg;
    msg.reserve(128 + trail.size());
    msg += entry;
    msg += " failed: ";
    msg += sm_status_name(code);
    msg += " (";
    msg += std::to_string(static_cast<long>(code));
    msg += ")";

    if (!trail.empty()) {
        msg += ". Kernel trace: ";
        msg += trail;
    } else if (binderFile) {
        // No SER() trace available: fall back to the binding-site location
        // so the user at least sees which CHECK_STATUS in the bindings fired.
        msg += " at ";
        msg += basenameOf(binderFile);
        msg += ":";
        msg += std::to_string(binderLine);
    }

    throw std::runtime_error(msg);
}

// ---------------------------------------------------------------------------
//  Kernel error callback.  Daisy-chains over whatever was previously
//  installed, so any previously installed error callback keeps functioning
//  if it wraps a region that calls into the bindings.
// ---------------------------------------------------------------------------
namespace
{
    SmErrorCallbackFunctionPtr s_prevErrorCallback = nullptr;
    std::atomic<bool>          s_callbackInstalled{ false };

    void smPyErrorCallback(SmStatus error, const TCHAR* file_name,
                           ULONG line_num, const TCHAR* message)
    {
        // Forward to whatever was previously installed first, so chained
        // consumers see every event.
        if (s_prevErrorCallback) {
            s_prevErrorCallback(error, file_name, line_num, message);
        }

        // Skip non-failures: warnings and informational messages are emitted
        // routinely by the kernel and would drown the trail.
        if (error == SM_SUCCESS || error == SM_ERR_WARNING || error == SM_ERR_MESSAGE) {
            return;
        }

        std::string fileUtf8 = smos_FromTChar(file_name);
        std::string msgUtf8  = smos_FromTChar(message);
        SmPyErrorTrail::append(error,
                               fileUtf8.c_str(),
                               static_cast<unsigned long>(line_num),
                               msgUtf8.c_str());
    }
}

void install_sm_error_callback()
{
    bool expected = false;
    if (!s_callbackInstalled.compare_exchange_strong(expected, true)) {
        return; // already installed
    }
    s_prevErrorCallback = SmThreadLocalStorage::GetErrCallbackFunction();
    smos_SetErrorCallback(smPyErrorCallback);
}
