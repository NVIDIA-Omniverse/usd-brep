// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmAdvSurfaceIntersector.h
* PURPOSE: Header file for SmAdvSurfaceIntersector object.
**********************************************************************/

#ifndef __SMADVSURFACEINTERSECTOR_H__
#define __SMADVSURFACEINTERSECTOR_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMSURFACEINTERSECTOR_H__
#include <SmSurfaceIntersector.h>
#endif

/*******************************************************************//**
PURPOSE: The advanced surface intersector class enhances the basic
    surface intersector class and adds the ability to process tangency curves
    and singularities.  This class also provides improves performance
    over the basic surface intersector.

NOTES: 
***********************************************************************/
class SM_EXPORT SmAdvSurfaceIntersector : public SmSurfaceIntersector
{
private:
  double       m_dClosestValue;        // in SmAdvSurfaceIntersector::SolveBranch stores closest intersection result seen
  SmPoint2d    m_sUVS[2];              // init to SM_UNDEF_DOUBLE
  SmTreeNode * m_apNodes[2];           // 
  SmBoolean    m_bSeperateGaussMaps;   // in SmAdvSurfaceIntersector::SolveBranch FALSE=run BranchIsGaussSeparate()

public:
  // constructor
  SmAdvSurfaceIntersector
  (
    const SmSurface  & crSurface1,                      ///< [in] :                                                       <br>
    const SmExtent2d & crUVDomain1,                     ///< [in] :                                                       <br>
    const SmSurface  & crSurface2,                      ///< [in] :                                                       <br>
    const SmExtent2d & crUVDomain2,                     ///< [in] :                                                       <br>
    SmBoolean          bFromFilletIntersector=FALSE     ///< [in] : TRUE = Constructing a FilletIntersector object        <br>
  );

  // destructor
  virtual ~SmAdvSurfaceIntersector() { };

  //
  virtual SmStatus FindInteriorCurves
  (
    SmTArray<SmCurve*>         & r3DCurves,             ///< [out]:    <br>
    SmTArray<SmCurve*>         & rSurface1UVCurves,     ///< [out]:    <br>
    SmTArray<SmCurve*>         & rSurface2UVCurves,     ///< [out]:    <br>
    SmTArray<SmTsectCurveType> & rCurveTypes,           ///< [out]:    <br>
    SmTArray<double>           & rDeviations            ///< [out]:    <br>
  );

  //
  virtual SmStatus ComputeSingularityPoint
  (
    SmPoint2d    aUVValues[2],                          ///<      : NOT USED: in : surf1 and surf2 xSectPoint UV values                      <br>
    SmTsectPnt & rTsectPnt,                             ///< [out]: xSectPoint Type,                                                    <br>
                                                        ///<      :       3DCurve position & tangent,                              <br>
                                                        ///<      :       3DCurve position & tangent projected to each surface,    <br>
                                                        ///<      :       and Surface position & derivatives                           <br>
    SmTsectPnt * pOptPreviousPnt                        ///< [in] : Last intersection point on curve being stepped out, NULL to ignore  <br>
                                                        ///<      : When supplied, used to handle singularity cases.                    <br>
                                                        ///<      : NOTE: only one vector in this is used: CrvDeriv(), the              <br>
                                                        ///<      : first derivative of the intersection curve; if everything           <br>
                                                        ///<      : else is unset, that's ok.                                           <br>
  );

  //
  virtual SmStatus LocalSolve
  (
    SmTreeNode * apBranch[SM_GS_MAX_TREES],             ///< [in] : candidate nodes for solution      <br>
    SmBoolean  & rbNeedsMoreSubdivision                 ///< [out]: TRUE =                            <br>
  );

  //
  SmStatus MakeDupin
  (
    SmVector3d           aSurfaceEval[3][3],            ///< [in] : surface point evaluation matrix                                          <br>
                                                        ///<      : with positions, tangents, and 2nd parametric derivatives                 <br>
    const SmVector3d   & crNormal,                      ///< [in] : Reference Surface Normal (either parallel or opposing Surface Normal)    <br>
    SmTArray<SmConic*> & crAboveDupinCurves,            ///< [out]: Dupin indicatrix curves associated with positive principal curvature     <br>
                                                        ///<      : The curve generated by intersecing surface at this point with a plane    <br>
    SmTArray<SmConic*> & crBelowDupinCurves,            ///< [out]: Dupin indicatrix curves associated with negative principal curvature     <br>
                                                        ///<      : The curve generated by intersecing surface at this point with a plane    <br>
    SmVector3d         & rPrinK1Vec,                    ///< [out]: Surface Tangent vector in 1st principal curvature direction              <br>
    SmVector3d         & rPrinK2Vec,                    ///< [out]: Surface Tangent vector in 2nd principal curvature direction              <br>
    double             & rdPrinK1,                      ///< [out]: 1st principal curvature                                                  <br>
    double             & rdPrinK2,                      ///< [out]: 2nd principal curvature                                                  <br>
    SmVector3d         & rEFG,                          ///< [out]: 1st fundamental form                                                     <br>
    SmVector3d         & rLMN                           ///< [out]: 2nd fundamental form
  ) const ;

  //
  virtual SmStatus SolveBranch(SmTreeNode * apBranch[SM_GS_MAX_TREES]);

  //
  SmStatus BranchIsGaussSeparate
  (
    SmTreeNode * apBranch[SM_GS_MAX_TREES],             ///< [in] :     <br>
    SmBoolean & rbIsSeperable                           ///< [in] :     <br>
  );

  //
  virtual SmStatus FindBestSubdivisionNode
  (
    SmTreeNode * apBranch[SM_GS_MAX_TREES],             ///< [in] :      <br>
    ULONG      & rlBestSubdivisionIndex,                ///< [out]:      <br>
    SmBoolean  & rbSwapTraversalOrder                   ///< [out]:      <br>
  ) const;

  // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
  SM_COMMON(SmAdvSurfaceIntersector, SmSurfaceIntersector, SmAdvSurfaceIntersector_TYPE) ;

} ; // end class SmAdvSurfaceIntersector

#endif // !__SMADVSURFACEINTERSECTOR_H__


