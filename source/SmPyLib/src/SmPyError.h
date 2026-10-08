// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Error reporting helpers for the _omni_solid Python bindings.
//
// Goal: when SMLib returns a non-success SmStatus, the Python caller should
// see *which* C entry point failed, what the symbolic SmStatus name is, and
// the deepest kernel SER() site (file:line + message) that fired on the way
// out -- not just the integer code.
//
// Design:
//   * sm_status_name() -- maps SmStatus integer to its SM_* identifier.
//   * SmPyErrorTrail   -- thread-local string sink that captures the kernel's
//                         smos_SetErrorCallback messages between CHECK_STATUS
//                         calls.
//   * SmPyRaise()      -- formats { c_call, status_name, code, kernel_trace }
//                         into a single std::runtime_error message and throws.
//   * install_sm_error_callback() -- daisy-chains a single global error
//                         callback over whatever was previously installed
//                         (e.g. another consumer's RAII guard) so consumers coexist.
//
// CHECK_STATUS in SmPyCommon.h reads #expr and __FILE__/__LINE__ and forwards
// to SmPyRaise on non-success.  No per-binding-site diff is required.

#ifndef __SmPyError_H__
#define __SmPyError_H__

#include <string>

#include <SmTypes.h>
#include <SmMessages.h>

/// Returns the SM_* identifier for a given SmStatus, or "SM_ERR_???" when
/// the code isn't in the table mirrored from SmMessages.h.  Never returns
/// NULL; the returned pointer has static storage duration.
const char* sm_status_name(SmStatus s);

/// Thread-local rolling sink for kernel SER() messages.  CHECK_STATUS clears
/// before each wrapped call; the kernel error callback appends; SmPyRaise
/// consumes on failure.  Capped at a few KB to keep error storms bounded.
namespace SmPyErrorTrail
{
    /// Empty the current thread's trail.  Cheap (string::clear retains
    /// capacity).
    void clear();

    /// Append one entry produced from a smos_SetErrorCallback invocation.
    /// Filters SM_ERR_WARNING / SM_ERR_MESSAGE upstream of the call.
    void append(SmStatus code, const char* file, unsigned long line, const char* msg);

    /// Move-out the accumulated trail (clears the sink).  Returns "" when
    /// nothing was captured.
    std::string take();
}

/// Formats and throws a std::runtime_error with the canonical message:
///
///   <c_call> failed: <status_name> (<code>). Kernel trace: <trail>
///
/// `exprText` is the macro-stringified call expression (e.g.
/// "SmApiCreatePipeSweep(radius, bsp, cap ? TRUE : FALSE, r)").  The leading
/// C identifier is extracted as the c_call.  `binderFile`/`binderLine` are
/// recorded only as a fallback when no kernel trace is available.
[[noreturn]] void SmPyRaise(SmStatus code,
                            const char* exprText,
                            const char* binderFile,
                            int binderLine);

/// Installs the kernel error callback used to populate SmPyErrorTrail.
/// Daisy-chains over any previously installed callback so other consumers
/// (e.g. another consumer's error callback) keep working.  Safe to call once at
/// PYBIND11_MODULE startup; subsequent calls are no-ops.
void install_sm_error_callback();

#endif // __SmPyError_H__
