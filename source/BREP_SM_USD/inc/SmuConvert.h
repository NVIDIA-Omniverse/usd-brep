// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmuConvert.h
* PURPOSE: Header file for USD / SMLib object conversions
**********************************************************************/

#ifndef _SMU_CONVERT_H_
#define _SMU_CONVERT_H_

#include "SmuConfig.h"  // defines (or omits for debug)
#include "SmMessages.h"
#include "SmTypes.h"

#include <cstdint>
#include <vector>

// pixar includes
#include "UsdBrepSuppressPixarWarningsPush.h" // turn off compile warnings for problematic pixar include files
#include <pxr/pxr.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usdGeom/gprim.h>
#include <pxr/usd/usdGeom/capsule.h>
#include "UsdBrepSuppressPixarWarningsPop.h"  // done loading problematic pixar headers - restore compile warnings
#include <pxr/usd/usdGeom/cone.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/sphere.h>
#include <pxr/usd/usdGeom/mesh.h>
             
namespace UsdBrepData{ class UsdBrepArrayData; 
                        class UsdBrepArraySpans;
                      }

class SmBrep;
class SmBrepData;
class SmPolyBrep;
class SmAttribute;
template<class TYPE> class SmTArray ;

// character buffer size
#define SMU_TBLOCK_SIZE 1024

// load buffer:[size=SMU_TBLOCK_SIZE] with formatted string: - always use 1 or more specifiers and
// string precision specifiers
#ifdef _UNICODE
template <typename... Args>
void inline smu_sprintf(wchar_t* buff, wchar_t* format, Args... args)
{
    _sntprintf(buff, (size_t)(SMU_TBLOCK_SIZE - 1), format, args...);
    buff[SMU_TBLOCK_SIZE - 1] = L'\0';
}
#else
template <typename... Args>
void inline smu_sprintf(char* buff, const char* format, Args... args)
{
    snprintf(buff, (size_t)(SMU_TBLOCK_SIZE), format, args...);
}
#endif

// send char buffers to targets: on windows     in debug - tgt OutputDebugString()
//                               on Linux/macOS in debug - tgt stderr
//                               if(b_OutputLong)        - tgt LongFile (if pBuff use pBuff else pBuffForFile)
//                               if(b_OutputThin)        - tgt ThinFile (if pBuffForFile use pBuffForFile else pBuff)

// pBuff:        in: string for output window and longFile when b_OutputLong=true
// pBuffForFile: in: string for thinFile when b_OutputThin=true
void smu_WriteBuffer(const TCHAR* pBuff, const TCHAR* pBuffForFile = NULL);

namespace SMU_BrepConvert
{

    struct BrepImportResult
    {
        SmBrep* pBrep = nullptr;
        std::uint32_t iPackedBrepIndex = 0;
        SmStatus status = SM_ERR;
        bool bRemainingMembersSkipped = false;
    };

    // SMLib to USD

    // Append vector of SmBreps to UsdBrepArray
    SMU_EXPORT SmStatus BrepAppend_SMLibToUsd
     (std::vector<SmBrep*>& rSmBreps,                       // i/o: in : 'from' SmBreps to convert to a UsdBrepArrayData
                                                            //      out: an SmSdfPathAttribute(UsdBrepArraypath) is added to each SmBrep_ii.
      pxr::UsdPrim& crUsdBrepArray,                         // in : Tgt UsdBrepArray being appended
      bool bExportUVCurves = true,                          // in : when true, export Edgeuse UV trim curves (UVCurves) to USD
      bool bBoundUnboundedFaceRanges = false );             // in : when true, replace unbounded face ranges with trim-curve bounds in USD only
                                                            //      0x0 to ignore.

    // Move one 
    SMU_EXPORT SmStatus AppendSmBrepData_ToUsdBrepArrayData
     (SmBrepData                     * pSmBrepData,         // i/o: in : 'from' SmBrepData to convert to a UsdBrepArrayData
                                                            //      out: a SmSdfPathAttribute( UsdBrepArrayPath ) may be added to each SmBrep_ii.
      SmTArray<SmAttribute*>         & rAttributes,         // in : attributes for items in pSmBrepData (from SmBrepData::FromBrep() )
      UsdBrepData::UsdBrepArrayData & rArrays,             // i/o: in : UsdBrepArrayData with preexisting Brep data
                                                            //      out: Augmented with pSmBrepData content
      SmBrep                         * pOptSmBrep,          // in : optional associated SmBrep - required for attribute propagation - NULL to ignore
      pxr::SdfPathVector             * pOptPaths);          // in : optional SdfPathVector of UsdBrepArrayPaths to be added to each SmBrep_ii as an SMLib attribute

    // Append vector of SmBreps to UsdBrepArrayData
    SMU_EXPORT SmStatus BrepAppend_SMLibToUsdBrep
     (std::vector<SmBrep*>           & rSmBreps,            // i/o: target SmBreps to add to rArrays, (a SmSdfPathAttribute(UsdBrepArraypath) is added to each SmBrep_ii)
      UsdBrepData::UsdBrepArrayData & rArrays,             // i/o: UsdBrepArrayData augmented with rSmBreps data
      pxr::SdfPathVector             * pOptPaths=nullptr,   // in : optional SdfPathVector of UsdBrepArrayPaths to be added to each SmBrep_ii as an SMLib attribute
                                                            //      0x0 to ignore. default:[0x0]
      bool bExportUVCurves = true,                          // in : when true, export Edgeuse UV trim curves (UVCurves) into rArrays for later USD writing
      bool bBoundUnboundedFaceRanges = false ) ;            // in : when true, replace unbounded face ranges with trim-curve bounds in USD only

    // USD to SMLib

    // Move all UsdGeomGprim Brep members to an array of SmBreps. Conversion
    // is all-or-nothing: existing caller-owned entries are preserved and new
    // Breps are appended only after every packed member converts successfully.
    SMU_EXPORT SmStatus BrepMove_UsdToSMLib
     (const SmContext          & crSmContext,         // in : Context in which prSmBrep is created            
      const pxr::UsdGeomGprim  & crUsdBrepArray,      // in : Usd prim being converted to an SmBrep
      std::vector<SmBrep*>     & rSmBreps,            // i/o: Existing entries preserved; new Breps appended on success
      SmBoolean                  bHealerIsEnabled = TRUE // in : default TRUE: healer is enabled  FALSE: disable healer
     );

    // Attempt every locatable member of one UsdGeomGprim BrepArray. Results
    // are appended in packed-member order. A nonNULL pBrep is owned by the
    // caller; failed members have pBrep == NULL and their conversion status.
    // A malformed container that prevents member enumeration returns an error
    // without appending results. A malformed member boundary appends one
    // failure with bRemainingMembersSkipped == true and stops that BrepArray.
    // Set bStopAfterFailure for the legacy strict caller's fail-fast behavior.
    // Process-level failures always return immediately without appending.
    SMU_EXPORT SmStatus BrepMove_UsdToSMLibWithResults
     (const SmContext                  & crSmContext,
      const pxr::UsdGeomGprim          & crUsdBrepArray,
      std::vector<BrepImportResult>    & rResults,
      SmBoolean                          bHealerIsEnabled = TRUE,
      SmBoolean                          bStopAfterFailure = FALSE
     );

    // move one UsdGeomGprim Brep member to SmBrep
    SMU_EXPORT SmStatus BrepMove_OneUsdBrepToSMLib
    (
        const SmContext                       & crSmContext,    // in : Context in which prSmBrep is created            
        const UsdBrepData::UsdBrepArrayData  & rArrays,        // in : all brep attribute values from UsdBrepData::BrepReadFromUsdStage()
        UsdBrepData::UsdBrepArraySpans       & rSpans,         // i/o: Container for Brep_ii object count and 1stIndex values computed from the rArrays member arrays.
        uint32_t                                iBrepIndex,     // in : UsdBrepArrayData tgt item index to convert to SmBrep
        SmBrep                               *& prSmBrep_ii,    // out: Newly created SmBrep, or NULL on failure
        SmBoolean                               bHealerIsEnabled = TRUE // in : default TRUE: healer is enabled  FALSE: disable healer
    );                                                 

    // move one AppendedBrepArrayData member to SmBrepData
    SMU_EXPORT SmStatus BrepMove_OneUsdBrepToSMLibData
    (
        const SmContext                       & crSmContext,     // in : Context in which prSmBrep is created            
        const UsdBrepData::UsdBrepArrayData  & rArrays,         // in : all brep attribute values from UsdBrepData::BrepReadFromUsdStage()
        UsdBrepData::UsdBrepArraySpans       & rSpans,          // i/o: Container for Brep_ii object count and 1stIndex values computed from the rArrays member arrays.
        uint32_t                                iBrepIndex,      // in : UsdBrepArrayData tgt item index to convert to SmBrep 
        SmBrepData                           *& prSmBrepData_ii, // i/o: Must be NULL; receives a new SmBrepData on success
        SmTArray<SmAttribute*>                & rAttributes      // out: Referenced attributes; empty on failure
    );                                                             

    // Create SmBrep from UsdGeomCapsule
    SMU_EXPORT SmStatus CreateSmBrep_FromUsdCapsule
    (
        const SmContext            & crSmContext,  // in : Context in which prSmBrep is created       
        const pxr::UsdGeomCapsule  & crUsdCapsule, // in : Usd prim being converted to an SmBrep      
              SmBrep              *& prSmBrep      // out: The newly created SmBrep                   
    );

    // Create SmBrep from UsdGeomCone
    SMU_EXPORT SmStatus CreateSmBrep_FromUsdCone
    (
        const SmContext         & crSmContext, // in : Context in which prSmBrep is created       
        const pxr::UsdGeomCone  & crUsdCone,   // in : Usd prim being converted to an SmBrep      
              SmBrep           *& prSmBrep     // out: The newly created SmBrep                   
    );
    
    // Create SmBrep from UsdGeomCube
    SMU_EXPORT SmStatus CreateSmBrep_FromUsdCube
    (
        const SmContext         & crSmContext, // in : Context in which prSmBrep is created       
        const pxr::UsdGeomCube  & crUsdCube,   // in : Usd prim being converted to an SmBrep      
              SmBrep           *& prSmBrep     // out: The newly created SmBrep                   
    );
        
    // Create SmBrep from UsdGeomCylinder
    SMU_EXPORT SmStatus CreateSmBrep_FromUsdCylinder
    (
        const SmContext             & crSmContext,   // in : Context in which prSmBrep is created       
        const pxr::UsdGeomCylinder  & crUsdCylinder, // in : Usd prim being converted to an SmBrep      
              SmBrep               *& prSmBrep       // out: The newly created SmBrep                   
    );

    // Create SmBrep from UsdGeomSphere
    SMU_EXPORT SmStatus CreateSmBrep_FromUsdSphere
    (
        const SmContext           & crSmContext, // in : Context in which prSmBrep is created       
        const pxr::UsdGeomSphere  & crUsdSphere, // in : Usd prim being converted to an SmBrep      
              SmBrep             *& prSmBrep     // out: The newly created SmBrep                   
    );

    // Create SmPolyBrep from UsdGeomMesh
    SMU_EXPORT SmStatus CreateSmPolyBrep_FromUsdMesh
    (
        const SmContext         & crSmContext,   // in : Context in which prSmPolyBrep is created       
        const pxr::UsdGeomMesh  & crUsdGeomMesh, // in : Usd prim being converted to an SmPolyBrep  
              SmPolyBrep       *& prSmPolyBrep   // out: The newly created SmPolyBrep               
    );

} // end namespace SMU_BrepConvert

#endif // no _SMU_CONVERT_H_
