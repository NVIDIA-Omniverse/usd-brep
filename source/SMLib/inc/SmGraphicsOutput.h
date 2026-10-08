// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmGraphicsOutput.h
* PURPOSE: Interface to core graphics primitives.
**********************************************************************/

#ifndef __SMGFX_OUTPUT_H__
#define __SMGFX_OUTPUT_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMGFXVERTEXARRAY_H__
#include <SmGraphicsVertexArray.h>
#endif

#if defined(SM_GFX_OUTPUT_CODE)
// 
// #ifndef __SMVECTOR3D_H__
// #include <SmVector3d.h>
// #endif
// 
// #ifndef __SMPOINTSET_H__
// #include <SmPointSet.h>
// #endif
// 
// #ifndef __SMAXIS2PLACEMENT_H__
// #include <SmAxis2Placement.h>


// forward declarations
enum SmColorRuleType ;

// output color for (entity,ColorRule) pair to drawList - uses DefaultColor, ColorStack, and applicationColors

SM_EXPORT double     smgfx_GetOutputPointSize  (SmGfxArraySet *pOptGfxSet = NULL);
SM_EXPORT double     smgfx_GetOutputLineWidth  (SmGfxArraySet *pOptGfxSet = NULL);
SM_EXPORT SmBoolean  smgfx_GetOutputDashedLines(SmGfxArraySet *pOptGfxSet = NULL);
SM_EXPORT SmVector3d smgfx_GetOutputColor      (SmGfxArraySet *pOptGfxSet = NULL);

SM_EXPORT SmVector3d smgfx_OutputObjectColor(const SmObject *pObject, 
                                             SmColorRuleType eColorRule=SM_CR_STANDARD, SmGfxArraySet *pOptGfxSet = NULL) ;
SM_EXPORT double     smgfx_OutputPointSize  (double dPointSize,                         SmGfxArraySet *pOptGfxSet = NULL) ;
SM_EXPORT double     smgfx_OutputLineWidth  (double dLineWidth,                         SmGfxArraySet *pOptGfxSet = NULL) ;
SM_EXPORT SmBoolean  smgfx_OutputDashedLines(SmBoolean bDoDashedLines,                  SmGfxArraySet *pOptGfxSet = NULL) ;
SM_EXPORT SmVector3d smgfx_OutputColor      (double red, double green, double blue,     SmGfxArraySet *pOptGfxSet = NULL) ;
SM_EXPORT SmVector3d smgfx_OutputColor      (const SmVector3d &Color,                   SmGfxArraySet *pOptGfxSet = NULL) ;
SM_EXPORT SmVector3d smgfx_ChangeOutputColor(                                           SmGfxArraySet *pOptGfxSet = NULL) ;


SM_EXPORT void smgfx_OutputPoint   (double dX,  double dY,  double dZ,
                                    SmGfxArraySet * pOptGfxSet=NULL) ; 
SM_EXPORT void smgfx_OutputLine    (double dX1, double dY1, double dZ1,
                                    double dX2, double dY2, double dZ2,
                                    SmGfxArraySet * pOptGfxSet=NULL) ;     
SM_EXPORT void smgfx_OutputPolyline(double        * pts,
                                    long            npts,                 //   sizeof:[pts] = dStrideInDoubles * npts                  
                                    long            dStrideInDoubles=3,   // in : number of doubles between distinct points in the *pts array
                                                                          //      ex: 3 = pts a packed array of 3d vectors
                                                                          //          6 = plot every other pts 3d vector.
                                    SmGfxArraySet * pOptGfxSet=NULL) ;    // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                                                          //      NULL to ignore. default:[NULL]
SM_EXPORT void smgfx_OutputComb(double        * vecs,                     
                                double        * basePts,
                                long            npts,                     //   sizeof:[pts] = dStrideInDoubles * npts                  
                                double          dScale=1.0,               // scale applied to every vector
                                long            lVecStrideInDoubles=3,    // in : number of doubles between distinct points in the *pts array
                                long            lBasePtStrideInDoubles=3, // in : number of doubles between start of each pt in the *pts array
                                                                          //      ex: 3 = pts a packed array of 3d vectors
                                                                          //          6 = plot every other pts 3d vector.
                                SmGfxArraySet * pOptGfxSet=NULL) ;        // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                                                          //      NULL to ignore. default:[NULL]

SM_EXPORT void smgfx_OutputTriangle(const SmPoint3d & crP1,
                                    const SmPoint3d & crP2,
                                    const SmPoint3d & crP3,
                                    const SmVector3d * cpNormal1,
                                    const SmVector3d * cpNormal2,
                                    const SmVector3d * cpNormal3,
                                    SmGfxArraySet    * pOptGfxSet=NULL) ; // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                                                          //      NULL to ignore. default:[NULL]
SM_EXPORT void smgfx_OutputQuad(const SmPoint3d & crP1,
                                const SmPoint3d & crP2,
                                const SmPoint3d & crP3,
                                const SmPoint3d & crP4,
                                const SmVector3d * cpNormal1,
                                const SmVector3d * cpNormal2,
                                const SmVector3d * cpNormal3,
                                const SmVector3d * cpNormal4,
                                SmGfxArraySet    * pOptGfxSet=NULL) ;  // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                                                       //      NULL to ignore. default:[NULL]
SM_EXPORT void smgfx_OutputCircularArc(const SmPoint3d &crOrigin,         // in : arc origin
                                       const SmVector3d &crX,             // in : defines zero degree start for arc
                                       const SmVector3d &crY,             // in : specifies circle plane and 90 degree direction for arc
                                       double dRadius,                    // in : arc radius
                                       double dStartAngleDeg,             // in : arc start point in degrees
                                       double dEndAngleDeg,               // in : arc end point in degrees
                                       double dMaxHeightError,            // in : max arc height error allowed
                                       SmGfxArraySet * pOptGfxSet=NULL) ; // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                                                          //      NULL to ignore. default:[NULL]
                                                                          // note: number of polysegments drawn
                                                                          //       is determined by the radius and
                                                                          //       the dMaxHieghtError

#else // SM_GFX_OUTPUT_CODE
SM_EXPORT double     smgfx_GetOutputPointSize  (SmGfxArraySet *pOptGfxSet = NULL);
SM_EXPORT double     smgfx_GetOutputLineWidth  (SmGfxArraySet *pOptGfxSet = NULL);
SM_EXPORT SmBoolean  smgfx_GetOutputDashedLines(SmGfxArraySet *pOptGfxSet = NULL);
SM_EXPORT SmVector3d smgfx_GetOutputColor      (SmGfxArraySet *pOptGfxSet = NULL);
SM_EXPORT SmVector3d smgfx_OutputColor      (double, double, double, SmGfxArraySet* pOptGfxSet = NULL) ;  
SM_EXPORT SmVector3d smgfx_OutputColor      (const SmVector3d &Color, SmGfxArraySet *pOptGfxSet = NULL) ;
SM_EXPORT double     smgfx_OutputLineWidth  (double, SmGfxArraySet *pOptGfxSet = NULL) ;  
SM_EXPORT double     smgfx_OutputPointSize  (double dPointSize, SmGfxArraySet *pOptGfxSet = NULL) ;
SM_EXPORT void       smgfx_OutputPolyline   (double*, long, long, SmGfxArraySet*);

#endif // SM_GFX_OUTPUT_CODE

#endif // !__SMGFX_OUTPUT_H__

