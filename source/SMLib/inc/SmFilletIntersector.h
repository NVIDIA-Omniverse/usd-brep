// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmFilletIntersector.h
* PURPOSE: Header file for SmFilletIntersector object.
**********************************************************************/

#ifndef __SMFILLETINTERSECTOR_H__
#define __SMFILLETINTERSECTOR_H__

#ifndef __SMADVSURFACEINTERSECTOR_H__
#include <SmAdvSurfaceIntersector.h>
#endif

#ifndef __SMFILLETSOLVER_H__
#include <SmFilletSolver.h>
#endif

/*******************************************************************//**
PURPOSE: The advanced surface intersector class enhances the basic
    surface intersector class and adds the ability to process tangency curves
    and singularities.  This class also provides improves performance
    over the basic surface intersector.

NOTES: 
***********************************************************************/
class SmFilletIntersector : public SmAdvSurfaceIntersector
{
private:
  SmFilletSolver & m_rFilletSolver;             // edge fillet manager containing filleting information for one edge
                                                // defining the implicit geometry

  SmFilletGeom   * m_pCurrFilletGeom;           // explict edge fillet geometry container

  SmBoolean        m_bCurrFilletTouchBoundary;  // default:[FALSE], may be set TRUE in SmFilletIntersector::FlushCurve()
                                                //   TRUE = last flushed curve had one end or other on Surface Natural boundary
                                                //   FALSE=  otherwise
                                                // checked in SmFilletSolver::CalcFilletGeom() to know when fillet stepping algorithm quit early 

  SmBoolean        m_bSelfIntersect;            // default:[FALSE], set TRUE in SmFilletSolver::CalcFilletGeom when m_bCurrFilletTouchBoundary == FALSE
                                                //   TRUE =
                                                //   FALSE=
                                                // no longer used - replaced by check if(pSelfIntHandler != NULL)

public:
  virtual ~SmFilletIntersector() {};

  SmFilletIntersector(const SmSurface  & crSurface1, 
                      const SmExtent2d & crUVDomain1,
                      const SmSurface  & crSurface2,
                      const SmExtent2d & crUVDomain2,
                      SmFilletSolver   & rFilletSolver,
                      SmFilletGeom     * pCurrFilletGeom = NULL);

  virtual SmStatus ComputeNextPoint(SmTsectPnt & rTsectPnt, 
                                    double       dStepSize, 
                                    SmTsectPnt & rNextTSP,
                                    SmBoolean  & rbFoundGoodPoint,
                                    double     & rdDeviationFound,
                                    double     & rdAngleFoundRad,
                                    SmBoolean  & rbClipped,
                                    SmBoolean  & rbBoundaryHit);

  virtual SmStatus ComputePointValues(SmPoint2d    aUVValues[2],
                                      SmTsectPnt & rTsectPnt,
                                      SmTsectPnt * pOptPreviousPnt,
                                      double     * pdStepSize);

  virtual SmStatus ComputePointValuesDelta(SmPoint2d    aUVValues[2],      // in : TgtParam      rail Surf UVPt values to evaluate
                                           SmPoint2d    aUVValuesDelta[2], // in : TgtDeltaParam rail Surf UVPt values to evaluate
                                           SmTsectPnt & rTsectPnt,         // out: container for TgtParam surface and Crv 3dPt, UVPos and derivative values
                                           SmTsectPnt & rDeltaPnt);        // out: container for TgtDeltaParam surface and Crv 3dPt, UVPos and derivative values

  virtual SmStatus EvaluateLawPoint
                    (double              dParameter,              // NotUsed: in : filletCurve parameter - used to compute current fillet radius
                     const SmPoint2d   & crUV0,                   // in : guess uv point on m_cpSurface[0]
                     const SmPoint2d   & crUV1,                   // in : guess uv point on m_cpSurface[1] 
                     const SmFilletLaw & crLawCurve,              // NotUsed: in : FilletLaw to compute fillet-radius for every Curve param 
                     SmBoolean           bLawOrient,              // NotUsed: in : bReverseOrientation: TRUE = parameters run from interval end to interval start
                     const SmExtent1d  & crPointCurveInterval,    // NotUsed: in : crCurveInterval = =interval defining range of fillet edge 
                     double              dSurfaceOrientations[2], // NotUsed: in : Offset directons for m_cpSurface[0,1], either -1.0 or 1.0 values. 
                     const SmPoint3d   & rPlaneOrigin,            // in : Origin of Plane(origin, normal)
                     const SmVector3d  & rPlaneNormal,            // in : Normal of Plane(origin, normal)
                     double            & rdCurveParam,            // out: Curve Param
                     SmPoint2d           sUVs[2]);                // out: Surf Params of m_pSurface[0]/m_pSurface[0]/plane XSect result

  virtual SmStatus FlushCurve(SmTArray<SmCurve*>         & r3DCurves,         // NotUsed: in :
                              SmTArray<SmCurve*>         & rSurface1UVCurves, // NotUsed: in :
                              SmTArray<SmCurve*>         & rSurface2UVCurves, // NotUsed: in :
                              SmTArray<SmTsectCurveType> & rCurveTypes,       // out:
                              SmTArray<double>           & rDeviations);      // out:

  SmFilletSolver * GetFilletSolver()            const { return(&m_rFilletSolver) ; }
  SmFilletGeom   * GetCurrFilletGeom()          const { return( m_pCurrFilletGeom) ; }
  SmBoolean        GetCurrFilletTouchBoundary() const { return( m_bCurrFilletTouchBoundary) ; }
  SmBoolean        GetSelfIntersect()           const { return( m_bSelfIntersect) ; }                                             
                                                  
  SmBoolean        IsCurrFilletTouchBoundary()  const { return(m_bCurrFilletTouchBoundary) ; }

  void SetSelfIntersect(SmBoolean bIsSelfIntersect)   { m_bSelfIntersect = bIsSelfIntersect ; }

  virtual SmStatus TestSpanAccuracy(SmTsectPnt & rTsectPnt,        // in :
                                    SmTsectPnt & rNextTSP,         // in :
                                    SmTsectPnt * pOptMidPnt,       // in :
                                    SmBoolean  & rbFoundGoodPoint, // out:
                                    double     & rdDeviationFound, // NotUsed: out:
                                    double     & rdAngleFoundRad); // out:

  // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
  SM_COMMON(SmFilletIntersector, SmAdvSurfaceIntersector, SmFilletIntersector_TYPE) ;

} ; // end class SmFilletIntersector

#endif // !__SMFILLETINTERSECTOR_H__


