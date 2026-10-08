// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __SmImportExport_H__
#define __SmImportExport_H__

#ifndef __SmApiTypes_H__
#include <SmApiTypes.h>
#endif

#include <SmTArray.h>

class SmBrep;
class SmCurve;
class SmSurface;

/// Read one Brep from a native SMLib .smb file.
///
/// @param pFileName             [in ]: Input file path; must not be NULL
/// @param bAscii                [in ]: TRUE for ASCII, FALSE for binary
/// @param bRebuildUVTrimCurves  [in ]: TRUE to rebuild UV trim curves after loading
/// @param rpBrep                [out]: Newly allocated Brep; caller takes ownership
/// @return SM_SUCCESS on success, error code otherwise
SMAPI_EXPORT SmApiStatus SmApiReadBrepFromFile
(
    const TCHAR * pFileName,
    SmBoolean     bAscii,
    SmBoolean     bRebuildUVTrimCurves,
    SmBrep      *& rpBrep
);

/// Write one Brep to a native SMLib .smb file.
///
/// @param pBrep      [in ]: Brep to serialize; must not be NULL
/// @param pFileName  [in ]: Output file path; must not be NULL
/// @param bAscii     [in ]: TRUE for ASCII, FALSE for binary
/// @return SM_SUCCESS on success, error code otherwise
SMAPI_EXPORT SmApiStatus SmApiWriteBrepToFile
(
    const SmBrep * pBrep,
    const TCHAR  * pFileName,
    SmBoolean      bAscii
);

/// Read a native SMLib part container.
///
/// All returned objects are newly allocated and owned by the caller.
/// Output arrays must be empty on entry.
///
/// @param pFileName          [in ]: Input file path; must not be NULL
/// @param bAscii             [in ]: TRUE for ASCII, FALSE for binary
/// @param rCurves            [out]: Standalone curves
/// @param rSurfaces          [out]: Standalone surfaces
/// @param rBooleanTreeNodes  [out]: Boolean tree node data
/// @param rBreps             [out]: Brep models
/// @return SM_SUCCESS on success, error code otherwise
SMAPI_EXPORT SmApiStatus SmApiReadPartFromFile
(
    const TCHAR          * pFileName,
    SmBoolean              bAscii,
    SmTArray<SmCurve*>   & rCurves,
    SmTArray<SmSurface*> & rSurfaces,
    SmTArray<long>       & rBooleanTreeNodes,
    SmTArray<SmBrep*>    & rBreps
);

#endif
