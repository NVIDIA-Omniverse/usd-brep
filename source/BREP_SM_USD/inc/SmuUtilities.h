// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmuUtilities.h
* PURPOSE: Header file for USD / SMLib utilities
**********************************************************************/

#ifndef _SMU_UTILITIES_H_
#define _SMU_UTILITIES_H_

#include "SmuConfig.h"

// pixar includes
#include <pxr/usd/usdGeom/xformOp.h>

// SMLib includes
#include "SmTypes.h"

class SmBrep;
class SmCurve;
class SmBSplineSurface;
class SmVector3d;
class SmAxis2Placement;

namespace SMU_BrepConvert
{
    //
    // Convenience functions to apply a UsdXformOp to an SMLib object
    //

    // apply a UsdGeomXformOp to an SmCurve
    SMU_EXPORT SmStatus ApplyUsdXformOp
    (
              SmCurve             & rCurve,    // in : Curve to which the transform is applied                     
        const pxr::UsdGeomXformOp & crXformOp  // in : Xform to apply to rCurve                                    
    );

    // Note: At some point we'll need to apply transforms to Analytics and offsets. When scale is not uniform, surfaces must be converted to BSSurfaces.
    // apply a UsdGeomXformOp to an SmBSplineSurface
    SMU_EXPORT SmStatus ApplyUsdXformOp
    (
              SmBSplineSurface    & rBSSurface,  // in : Surface to which the transform is applied                     
        const pxr::UsdGeomXformOp & crXformOp    // in : Xform to apply to rBSSurface                                  
    );

    // apply a UsdGeomXformOp to an SmBrep
    SMU_EXPORT SmStatus ApplyUsdXformOp
    (
              SmBrep              & rBrep,       // in : Brep to which the transform is applied                        
        const pxr::UsdGeomXformOp & crXformOp    // in : Xform to apply to rBrep                                    
    );

    //
    // Translate transformations between SMLib and Usd formats
    //

    // Convert a UsdXform to SMLib compatible scale and transform
    SMU_EXPORT SmStatus UsdXformToSmTransform
    (
        const pxr::UsdGeomXformOp & crXformOp,   // in : Xform that will be converted to SMLib standard             
              SmVector3d          & rScale,      // out: Scaling output                                             
              SmAxis2Placement    & rTransform   // out: Rotation and translation                                    
    );

    // Convert an SMLib scale and tranform to an Usd 4x4 matrix
    SMU_EXPORT SmStatus SmTransformToUsdXform
    (
        const SmVector3d          & rScale,      // in : Scaling                                                    
        const SmAxis2Placement    & rTransform,  // in : Rotation and translation                                   
              pxr::GfMatrix4d     & r4x4         // out: Transformation as 4x4 matrix                               
    );

    // Convert an SMLib scale and tranform to an UsdGeomXformOp
    SMU_EXPORT SmStatus SmTransformToUsdXform
    (
        const SmVector3d          & rScale,      // in : Scaling                                                      
        const SmAxis2Placement    & rTransform,  // in : Rotation and translation                                   
              pxr::UsdGeomXformOp & rXformOp     // out: Transformation as UsdGeomXformOp                           
    );

    //
    // Functions to access the SDF API, for stage authoring within an SdfChangeBlock
    //

    SMU_EXPORT pxr::SdfPrimSpecHandle smuGenerateBrepMetadata(const pxr::SdfPrimSpecHandle & rParentSpecHandle,
                                                              const std::string              sName,
                                                              std::string                  * pOptDisplayName,
                                                              pxr::GfMatrix4d              & rBrepMat,
                                                              ULONG                          uIndex,
                                                              const pxr::SdfPath             materialPath);
    
    SMU_EXPORT pxr::SdfPrimSpecHandle smuGenerateMeshMetadata(const pxr::SdfPrimSpecHandle & rParentSpecHandle,
                                                              const std::string              sName,
                                                              std::string                  * pOptDisplayName,
                                                              pxr::GfMatrix4d              & rBrepMat,
                                                              bool                           bDoubleSided,
                                                              ULONG                          uIndex,
                                                              const pxr::SdfPath             materialPath);
                                                            
} // end namespace SMU_BrepConvert

#endif // no _SMU_UTILITIES_H_
