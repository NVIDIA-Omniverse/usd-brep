// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepDiagnostics.h
 * PURPOSE: Diagnostic macros for the BREP libraries, routed through pxr Tf.
 *
 * NOTES: These post to the global pxr TfDiagnosticMgr, so diagnostics are picked up by whatever delegate the
 *        embedding application has installed (and fall back to Tf's default stderr output when standalone).
 *        They depend only on pxr, keeping the BREP libraries converter-agnostic. The "[brep]" prefix lets a
 *        consumer distinguish BREP diagnostics without coupling to any application-specific logging prefix.
 * ******************************************************************************************************************/

#ifndef _USD_BREP_DIAGNOSTICS_H_
#define _USD_BREP_DIAGNOSTICS_H_

#include <pxr/base/tf/callContext.h>
#include <pxr/base/tf/diagnostic.h>
#include <pxr/base/tf/diagnosticHelper.h>
#include <pxr/base/tf/stringUtils.h>

// Neutral, converter-agnostic prefix for BREP diagnostics.
#define USDBREP_LOG_PREFIX "[brep] "

// Fully-qualified equivalent of pxr's TF_CALL_CONTEXT. We post the Tf diagnostics directly through the
// fully-qualified helpers rather than the TF_WARN/TF_RUNTIME_ERROR/TF_STATUS macros, because those leave
// TfCallContext, the post helpers, and the diagnostic-type enum unqualified - which only resolves under a
// "using namespace pxr;". The BREP libraries intentionally avoid that, so we qualify everything here.
#define USDBREP_CALL_CONTEXT pxr::TfCallContext(__ARCH_FILE__, __ARCH_FUNCTION__, __LINE__, __ARCH_PRETTY_FUNCTION__)

// Recoverable issue.
#define USDBREP_WARN(...) pxr::Tf_PostWarningHelper(USDBREP_CALL_CONTEXT, "%s%s", USDBREP_LOG_PREFIX, pxr::TfStringPrintf(__VA_ARGS__).c_str())

// Runtime/data error.
#define USDBREP_ERROR(...)                                                                                                                           \
    pxr::Tf_PostErrorHelper(                                                                                                                         \
        USDBREP_CALL_CONTEXT,                                                                                                                        \
        pxr::TF_DIAGNOSTIC_RUNTIME_ERROR_TYPE,                                                                                                       \
        "%s%s",                                                                                                                                      \
        USDBREP_LOG_PREFIX,                                                                                                                          \
        pxr::TfStringPrintf(__VA_ARGS__).c_str()                                                                                                     \
    )

// Informational status.
#define USDBREP_STATUS(...) pxr::Tf_PostStatusHelper(USDBREP_CALL_CONTEXT, "%s%s", USDBREP_LOG_PREFIX, pxr::TfStringPrintf(__VA_ARGS__).c_str())

#endif // no _USD_BREP_DIAGNOSTICS_H_
