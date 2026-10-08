// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmuTokens.h
* PURPOSE: Header file for local Tokens used by USD
**********************************************************************/

#ifndef _SMU_TOKENS_H_
#define _SMU_TOKENS_H_

#include "UsdBrepSuppressPixarWarningsPush.h"                 // turn off known compile warnings for code owned by pixar
#         include <pxr/base/tf/staticTokens.h>
#include "UsdBrepSuppressPixarWarningsPop.h"                  // done loading pixar code - restore suspended compile warnings

#include "SmuConfig.h"

/// Access the token by using the key as though it were a pointer, like this:
///
/// \code
///    SmuTessTokens->primvarNormals;
/// \endcode
///
/// An additional member, allTokens, is a std::vector<TfToken> populated
/// with all of the generated token members.
///
/// Tokens defined by ((TOKEN_NAME, "TOKEN_STRING"))

PXR_NAMESPACE_OPEN_SCOPE 

#ifndef SMU_TESS_TOKENS
    #define SMU_TESS_TOKENS \
        ((primvarNormals,         "primvars:normals" )) \
        ((primvarNormalsIndices,  "primvars:normals:indices" ))
#endif // no SMU_TESS_TOKENS

// TF_DECLARE_PUBLIC_TOKENS three argument version exports declaresd tokens from a DLL on windows
// needed if any project other than BREP_SM_USD will be using these tokens
TF_DECLARE_PUBLIC_TOKENS(SmuTessTokens, SMU_EXPORT, SMU_TESS_TOKENS);

PXR_NAMESPACE_CLOSE_SCOPE

#endif // no _SMU_TOKENS_H_
