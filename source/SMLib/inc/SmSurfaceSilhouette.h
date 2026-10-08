// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSurfaceSilhouette.h
* PURPOSE: Header file for SmSurfaceSilhouette object.
**********************************************************************/

#ifndef __SMSURFACESILHOUETTE_H__
#define __SMSURFACESILHOUETTE_H__

#ifndef __SMSURFACETRACER_H__
#include <SmSurfaceTracer.h>
#endif

/*******************************************************************//**
PURPOSE: The surface silhouette class provides the ability to create
    silhouette curves for a given surface.

NOTES: Currently planar sectioning requires G1 surfaces with
    the given domain.

  NOTE - use the SmBSplineSurface::CreateSilhouetteCurves as an 
     an interface to this object.  Do not try to use this directly.
***********************************************************************/
class SmSurfaceSilhouette : public SmSurfaceTracer
{
  // inherited:
  //  SmGlobalSolver::m_eSolverOperation    
  //  SmGlobalSolver::m_eOperationCategory;     
  //  SmGlobalSolver::m_bProjectedOperation;    
  //  SmGlobalSolver::m_eSolutionRequested    
  //  SmGlobalSolver::m_lNumTrees;        
  //  SmGlobalSolver::m_lNumVariables;    
  //  SmGlobalSolver::m_apTrees[SM_GS_MAX_TREES];    
  //  SmGlobalSolver::m_d3dTolerance;                
  //  SmGlobalSolver::m_dBestAnswerSoFarSq;    
  //  SmGlobalSolver::m_dAtDistance;           
  //  SmGlobalSolver::m_dAtDistanceSq;    
  //  SmGlobalSolver::m_cpOptVectors;     
  //  SmGlobalSolver::m_pSolutions    
  //  
  //  SmSurfaceTracer::m_cpContext;           
  //  SmSurfaceTracer::m_cpSurface;                // Target Surface
  //  SmSurfaceTracer::m_vUVDomain;                // Target Surface Domain
  //  SmSurfaceTracer::m_dThisAngTolRad;    
  //  SmSurfaceTracer::m_dThisApproxTol3d; 
  //  SmSurfaceTracer::m_vTrcPntMgr;               // SmMemBlockMgr - manages blocs of SmTracePnts for m_vCurvePoints and m_vStartPoints
  //  SmSurfaceTracer::m_pStartPoint;         
  //  SmSurfaceTracer::m_vCurvePoints;             // SmTList<SmTracePnt> - contains TracePnts managed by m_vTrcPntMgr
  //  SmSurfaceTracer::m_dCurveTraceDirection;
  //  SmSurfaceTracer::m_bCurveIsClosed;      
  //  SmSurfaceTracer::m_vStartPoints;             // array of all valid drop curve interval start points - contains TracePnts managed by m_vTrcPntMgr

private:
    SmBoolean     m_bPerspective;
    SmPoint3d     m_vEye;

public:
    virtual ~SmSurfaceSilhouette() {};

    SmSurfaceSilhouette
    (
      const SmSurface  * cpSurface,           ///< [in ]: <br>
      const SmExtent2d & crUVDomain,          ///< [in ]: <br>
      SmBoolean          bPerspective,        ///< [in ]: <br>
      const SmVector3d & crEye                ///< [in ]: <br>
    );

    SmStatus ComputePointValues
    (
      const SmPoint2d & crUV,                 ///< [in ]: <br>
      SmTracePnt      & rCurrPnt,             ///< [in ]: <br>
      SmTracePnt      * pPrevPnt=NULL,        ///< [in ]: <br>
      double          * pdStepSize=NULL       ///< [in ]: <br>
    );

    SmStatus LocalPointSolve
    (
      const SmTracePnt & crCurrPnt,           ///< [in ]: <br>
      const SmPoint2d  & crGuessUV,           ///< [in ]: <br>
      SmTracePnt       & crNextPnt,           ///< [in ]: <br>
      SmBoolean        & rbFoundAnswer,       ///< [in ]: <br>
      SmPoint2d        & rUVFound             ///< [in ]: <br>
    ) const;

    virtual SmStatus FindSolutionsOnBoundaryCurve
    (
      const SmBSplineCurve & crBoundaryCurve,    ///< [in ]: <br>
      const SmExtent1d     & crInterval,         ///< [in ]: <br>
      ULONG                  lSide,              ///< NotUsed: [in ]: <br>
      SmSurfParamType        eSurfParam,         ///< NotUsed: [in ]: <br>
      SmSolutionArray      & rSolutions          ///< [out]: <br>
    ) const;

    SmBoolean BranchMayContainAnswers(SmTreeNode * apBranch[SM_GS_MAX_TREES]);

    SmBoolean DoesPointLieOnCurve(SmPoint2d & rUV, SmBoolean bRefinePoint) const;

    // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
    SM_COMMON(SmSurfaceSilhouette, SmSurfaceTracer, SmSurfaceSilhouette_TYPE) ;

} ; // end class SmSurfaceSilhouette


#endif // __SMSURFACESILHOUETTE_H__

