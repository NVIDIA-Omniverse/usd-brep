// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmNurbsrf.h
* PURPOSE: Header file for local copies of nurb functions.
*
**********************************************************************/

#ifndef __Sm_Nurbs_SRF_H__
#define __Sm_Nurbs_SRF_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __Sm_Nurbs_CRV_H__
#include <SmNurbsCrv.h>
#endif

SM_EXPORT SmStatus sm_FindNetExtrema
(
  const gw_SURFACE * pSur,                   // in : Target Surface  
  const SmPoint3d  & rTestPoint,             // in : Target Point  
  ULONG              lNumToFind,             // in : Number of controlPoints to find and place in output arrays  
  double           * adMinDistSq,            // out: array of TargetPoint/ControlPoint distances sorted by distance sized:[lNumToFind]  
  ULONG              alMinUVIndex[10][2],    // out: associated controlPoint [iu, iv] indices for every adMinDistSq entry  
  double           * adMaxDistSq,            // out: array of TargetPoint/ControlPoint distances sorted by distance sized:[lNumToFind]  
  ULONG              alMaxUVIndex[10][2],    // out: associated controlPoint [iu, iv] indices for every adMaxDistSq entry  
  ULONG            & rlNumMinFound,          // out: number of MinDist ControlPoints found [rlNumMinFound <= lNumToFind]  
  ULONG            & rlNumMaxFound           // out: number of MaxDist ControlPoints found [rlNumMinFound <= lNumToFind]  
);

// not currently called in SMLib
SM_EXPORT SmStatus sm_ComputePartialNetConstants
(
  const gw_SURFACE *cpSur,                   // in :   
  ULONG lU0Span,                             // in :   
  ULONG lV0Span,                             // in :   
  ULONG lU1Span,                             // in :   
  ULONG lV1Span,                             // in :   
  SmExtent3d  * pBoundingBox,                // out:   
  SmPseudoBox * pPseudoBox,                  // out:   
  SmPoint3d & rU0V0,                         // out:   
  SmPoint3d & rU1V0,                         // out:   
  SmPoint3d & rU0V1,                         // out:   
  SmPoint3d & rU1V1,                         // out:   
  double * pdUChordHeight,                   // out:   
  double * pdVChordHeight,                   // out:   
  double * pdUAngleDeg,                      // out:   
  double * pdVAngleDeg                       // out:   
);

// compute approximate chordheight and turning angle values for a patch
SM_EXPORT SmStatus sm_ComputeNetConstants
(
  const SmSurface  * pSurface,         // in : optional surface expected to be nonNULL for non SmBSplineSurface types  
  const SmExtent2d * pUVDomain,        // in : only used when pSurface != NULL                                         
  const gw_SURFACE * cpSur,            // in : Shape being tested - always used                                        
  SmZoneTol3d      & rZoneTol3d,       // in : Tolerance distance for distinct 3d points
  double           * pdUChordHeight,   // out: max Udir controlPoint dist from polygon BasePlane dist, NULL to ignore  
  double           * pdVChordHeight,   // out: max Vdir controlPoint dist from polygon BasePlane dist, NULL to ignore  
  double           * pdUAngleDeg,      // out: max Udir polygon endTangent angle, NULL to ignore                       
  double           * pdVAngleDeg,      // out: max Vdir polygon endTangent angle, NULL to ignore                       
  double           * pdUVChordHeight   // out: max UV controlPoint dist from polygon BasePlane dist, NULL to ignore    
);

// compute approximate chordheight and turning angle values for a patch without a net
SM_EXPORT SmStatus sm_ComputeSurfConstants
(
  const SmSurface  * pSurface,           // in : optional surface expected to be nonNULL for non SmBSplineSurface types  
  const SmExtent2d & pUVDomain,          // in : only used when pSurface != NULL  
  double           * pdUChordHeight,     // out: max Udir controlPoint dist from polygon BasePlane dist, NULL to ignore  
  double           * pdVChordHeight,     // out: max Vdir controlPoint dist from polygon BasePlane dist, NULL to ignore  
  double           * pdUAngleDeg,        // out: max Udir polygon endTangent angle, NULL to ignore  
  double           * pdVAngleDeg,        // out: max Vdir polygon endTangent angle, NULL to ignore  
  double           * pdUVChordHeight     // out: max UV controlPoint dist from polygon BasePlane dist, NULL to ignore  
);

SM_EXPORT SmStatus sm_CopyNurbSurface
(
  const gw_SURFACE * cpFrom,             // in :   
  gw_SURFACE       * pTo                 // out:   
);

SM_EXPORT ULONG sm_ComputeNurbSurfaceSize(const gw_SURFACE *cpSurface) ;

SM_EXPORT ULONG sm_ComputeNurbSurfaceSize
(
  gw_INDEX  lUCPointHighestIndex,         // in : max U ControlPoint index value = U ControlPoint_count - 1  
  gw_INDEX  lVCPointHighestIndex,         // in : max V ControlPoint index value = V ControlPoint_count - 1  
  gw_INDEX  lUKnotsHighestIndex,          // in : max U knot index value = U knot_count - 1  
  gw_INDEX  lVKnotsHighestIndex           // in : max V knot index value = V knot_count - 1  
);

SM_EXPORT void sm_InitNurbSurfaceMemory
(
  gw_SURFACE * pSurfaceMemory,            // in : pointer to surface memory block to init, sized:[sm_ComputeNurbSurfaceSize()]  
  gw_INDEX     lUCPointHighestIndex,      // in : max U ControlPoint index value = U ControlPoint_count - 1                     
  gw_INDEX     lVCPointHighestIndex,      // in : max V ControlPoint index value = V ControlPoint_count - 1                     
  gw_DEGREE    lUDegree,                  // in : U dir degree                                                                  
  gw_DEGREE    lVDegree,                  // in : V dir degree                                                                  
  gw_INDEX     lUKnotsHighestIndex,       // in : max U knot index value = U knot_count - 1                                     
  gw_INDEX     lVKnotsHighestIndex        // in : max V knot index value = V knot_count - 1                                     
);

SM_EXPORT char *sm_AllocateBlockOfNurbSurfaces
(
  gw_INDEX          lNumberOfSurfaces,     // in :   
  gw_INDEX          lUCPointHighestIndex,  // in :   
  gw_INDEX          lVCPointHighestIndex,  // in :   
  gw_DEGREE         lUDegree,              // in :   
  gw_DEGREE         lVDegree,              // in :   
  gw_INDEX          lUKnotsHighestIndex,   // in :   
  gw_INDEX          lVKnotsHighestIndex,   // in :   
  SmTArray<void*> & rSurfaces              // out:   
);

SM_EXPORT gw_SURFACE * sm_AllocateNurbSurface (gw_INDEX          lUCPointHighestIndex,  // in : max U ControlPoint index value = U ControlPoint_count - 1  
                                               gw_INDEX          lVCPointHighestIndex,  // in : max V ControlPoint index value = V ControlPoint_count - 1  
                                               gw_DEGREE         lUDegree,              // in : U dir degree                                               
                                               gw_DEGREE         lVDegree,              // in : V dir degree                                               
                                               gw_INDEX          lUKnotsHighestIndex,   // in : max U knot index value = U knot_count - 1                  
                                               gw_INDEX          lVKnotsHighestIndex) ; // in : max V knot index value = V knot_count - 1                  

SM_EXPORT gw_SURFACE * sm_AllocateAndCopyNurbSurface(const gw_SURFACE * cpSrcSur) ;    // in : target surface to copy  

SM_EXPORT gw_FLAG  sm_DecomposeSrf
(
  const gw_SURFACE       * surP,            // in :   
  SmTArray<gw_SURFACE *> & rBeziers         // out:   
);

SM_EXPORT gw_FLAG  sm_SplitSrf(gw_SURFACE      * sur,          // in : tgt surface
                               gw_PARAMETER      dSplitParam,  // in : split param
                               gw_FLAG           dir,          // in : SplitDir: NL_UDIR=Split KnotVectorU, use - SM_SURFPARAM_TO_NLDIR(eSurfParam)
                                                               //                NL_VDir=Split KnotVectorV        to convert SmSurfParamType to NL_DIR types
                               gw_SURFACE     *& surL,         // out: Split surface result, Ivl=[MinParam, TgtParam]
                               gw_SURFACE     *& surR) ;       // out: Split surface result, Ivl=[TgtParam, MaxParam]

void Dump_NSrf
 (
  const gw_SURFACE * pSur,           // in : target representation                                    
  SmBoolean          bAbbrev=TRUE    // in : TRUE = decimate CPt and Knot reports, default:[FALSE]    
 );

SmBoolean sm_IsNSrfDegenerate(const gw_SURFACE *pSur, double d3DTol=SM_EFF_ZERO) ;
SmBoolean sm_HasRepeatedEndControlPoints(double dTol, const gw_SURFACE *pSur, SmBSplineSurface *pSurface=NULL) ;
SmBoolean sm_HasKnotMultiplicityGreaterThanDegree(const gw_SURFACE *pSur,             // in : target representation
                                                  SmBSplineSurface *pSurface=NULL) ;  // NotUsed: in : only used for debug, default:[NULL]

#endif // !__Sm_Nurbs_SRF_H__

