// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSurfaceDropCurve.h
* PURPOSE: Header file for SmSurfaceDropCurve object.
**********************************************************************/

#ifndef __SMSURFACEDROPCURVE_H__
#define __SMSURFACEDROPCURVE_H__

#ifndef __SMSURFACETRACER_H__
#include <SmSurfaceTracer.h>
#endif

/*******************************************************************//**
PURPOSE: The surface drop curve class provides the ability to drop 
    curves which lie on or very near a surface onto that surface.  It 
    works with surfaces and curves which are C0 or better. 

NOTES: Please do not use this object directly.  
    See the SmSurface::DropAndTrimCurve method for the interface.
***********************************************************************/
class SmSurfaceDropCurve : public SmSurfaceTracer
{
private:
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
  //  SmSurfaceTracer::m_dNodeConverganceTol; 
  //  SmSurfaceTracer::m_vTrcPntMgr;               // SmMemBlockMgr - manages blocs of SmTracePnts for m_vCurvePoints and m_vStartPoints
  //  SmSurfaceTracer::m_pStartPoint;         
  //  SmSurfaceTracer::m_vCurvePoints;             // SmTList<SmTracePnt> - contains TracePnts managed by m_vTrcPntMgr
  //  SmSurfaceTracer::m_dCurveTraceDirection;
  //  SmSurfaceTracer::m_bCurveIsClosed;      
  //  SmSurfaceTracer::m_vStartPoints;             // array of all valid drop curve interval start points - contains TracePnts managed by m_vTrcPntMgr

  // GWC:BEND_REPLACE_ONE_LINE
  //      const SmBSplineCurve & m_crCurveToDrop;           // target curve
  const SmCurve        & m_crCurveToDrop;           // target curve
  SmExtent1d             m_vInterval;               // target curve interval
  SmExtent1d             m_vWorkingInterval;        // scratch sub interval of targetCurve     
  SmExtent2d             m_vWorkingUVDomain;        // scratch sub domain of targetSurface     
  double                 m_dMaxDistanceToSurface;   //      
  SmBoolean              m_bCurveIsOnSurface;       // default:[FALSE], TRUE = TargetCurve is on Surface - use tighter drop tolerances
                                                    //                  FALSE= TargetCurve is only near Surface
  SmTArray<double>     * m_pCurveKnotVector;        // Local copy of targetCurve knots     
  SmTArray<double>     * m_pCurveBreaks;            // ordered list of TargetCurve paramValues for
                                                    //   targetCurve start/end points
                                                    //   every surfBoundary/TargetCurve intersection
                                                    //   every surfDiscontinuityIsoParameterLine/TargetCurve intersection
                                                    //   every targetCurve discontinuity point

public:
  // constructor
  // GWC:BEND_REPLACE_ONE_LINE                     
  //      const SmBSplineCurve & crCurveToDrop, 
  SmSurfaceDropCurve
  (
    const SmSurface  * cpSurface,                    ///< [in ]: target surface              <br>
    const SmExtent2d & crUVDomain,                   ///< [in ]: allowed surface subdomain   <br>
    const SmCurve    & crCurveToDrop,                ///< [in ]: target curve                <br>
    const SmExtent1d & crInterval                    ///< [in ]: Curve interval to drop      <br>
  );

  // destructor
  virtual ~SmSurfaceDropCurve();

  // simple access
  const SmCurve    & GetCurve()            { return m_crCurveToDrop ; }
  const SmExtent1d & GetInterval()         { return m_vInterval ; }
  double             GetMaxDistToSurface() { return m_dMaxDistanceToSurface ; }
  SmBoolean          GetCurveIsOnSurface() { return m_bCurveIsOnSurface ; }

  void SetCurveIsOnSurface(SmBoolean bBool) { m_bCurveIsOnSurface = bBool; }

  virtual SmStatus DoTrace
  (
    const SmContext     & crContext,               ///< [in ]: context for new object construction                                                   <br>
    const SmApproxTol3d * pdOptApproxTol3d,        ///< [in ]: max allowed deviation of TrimCurve from ideal Projection, NULL for m_dApproxTol3d     <br>
    const double        * pdOptAngTolRad,          ///< [in ]:                                                                                       <br>
    SmTArray<SmCurve*>  * p3DCurves,               ///< [out]: Surface Curves - not always a 3d curve for every UVDropCurve                          <br>
    SmTArray<SmCurve*>  * pSurfaceUVCurves,        ///< [out]: UVDropCurves (more than 1 if projected to seam or in/out of boundary)                 <br>
    SmTArray<double>    * pMaxDropDists,           ///< [out]: max 3dCurveSmpPoint to DropSurfPoint dist for each p3DCurves                          <br>
    SmTArray<double>    * pDeviations              ///< [out]: max 3dCurveSmpPoint to DropSurfNormLine dist for each p3DCurves                       <br>
  );


  virtual SmStatus FindInteriorCurves
  (
    SmTArray<SmCurve*> & r3DCurves,                ///< NotUsed: [out]: not modified                                                        <br>
    SmTArray<SmCurve*> & rSurfaceUVCurves,         ///< [out]: resulting Dropped UVCurves                                          <br>
    SmTArray<double>   & pMaxDropDists,            ///< [out]: Max Drop dist from 3dCurve(s) to Surface(drop_UV).                  <br>
    SmTArray<double>   & rDeviations               ///< [out]: Max gap dist between 3dCurve(s) and Surface(UVTrimCurve(s)).        <br>
  );

  virtual SmStatus ComputePointValues
  (
    const SmPoint2d & crUV,                        ///< [in ]: target SurfacePoint                                                                       <br>
    SmTracePnt      & rCurrPnt,                    ///< [in,out]: target CurvePoint, set with SurfacePoint properties                                    <br>
                                                   ///<      : m_dCurveParameter (in)                                                                    <br>
                                                   ///<      : m_vSurfacePV[0][0] = Surf Position  for crUV                                              <br>
                                                   ///<      : m_vSurfacePV[1][0] = Surf 1stDerivU for crUV                                              <br>
                                                   ///<      : m_vSurfacePV[0][1] = Surf 1stDerivV for crUV                                              <br>
                                                   ///<      : m_vSurfacePV[2][2] = Surf Unit Normal    for crUV                                         <br>
                                                   ///<      : m_v3DCurvePV[0]    = Drop 3DCurve Pos (equals m_vSurfacePV[0][0])                         <br>
                                                   ///<      : m_v3DCurvePV[1]    = Drop 3DCurve Tan (e3d CurveTan proj to SurfNorm plane)               <br>
                                                   ///<      : m_vUVCurvePV[0]    = UVCurve pos (equals crUV)                                            <br>
                                                   ///<      : m_vUVCurvePV[1]    = UVCurve Tan (3DCurveTan projected in 1stDeriv space)                 <br>
                                                   ///<      : m_dSurfDropDist    = DropCurve(CurveParam) to DropSurfPoint dist in DropSurfNormLine dir  <br>
    SmTracePnt      * pPrevPnt = NULL,             ///<      : m_dSurfNormLineDist=  DropCurve(CurveParam) to DropSurfNormLine min dist                  <br>
                                                   ///< [in ]: Last intersection point on curve being stepped out, NULL to ignore                        <br>
                                                   ///<      : When supplied used to handle singularity cases.                                           <br>
    double          * pdStepSize = NULL            ///< [in ]: pdStepSize = NOT USED                                                                     <br>
                                                   ///<      : distance to step back from singularities to try and find                                  <br>
  );                                               ///<      : a nearby neighbor to use to computePointValues, NULL to ignore                            <br>

  virtual SmStatus LocalPointSolve
  (
    const SmTracePnt & crCurrPnt,      ///< NotUsed: [in ]: curr TracePnt                                                                <br>
    const SmPoint2d  & crGuessUV,      ///< [in ]: UVPnt guess for next TracePnt drop                                           <br>
    SmTracePnt       & crNextPnt,      ///< [in,out]:                                                                           <br>
                                       ///<      : m_dCurveParameter (in)                                                       <br>
                                       ///<      : m_vUVCurvePV        = guess for NextPntUV                                    <br>
                                       ///<      : m_dSurfDropDist     = 3d dist from 3dPt to surface in SurfNormLine dir       <br>
                                       ///<      : m_dSurfNormLineDist = 3d Dist from 3dPt to SurfNormLine                      <br>
    SmBoolean        & rbFoundAnswer,  ///< [out]: TRUE = NextPnt3d less than m_dApproxTol3d from                               <br>
                                       ///<      :    SurfNormLine starting at NextPnt.SurfacePnt                               <br>
    SmPoint2d        & rUVFound        ///< [out]: UVPnt found by NR iteration                                                  <br>
  ) const;

  virtual SmStatus FindBoundaryStartPoints
  (
    SmTArray<SmCurve*> & r3DCurves,        ///< NotUsed: [out]: Not Used - drops m_crCurveToDrop - does not gen any 3d curves                   <br>
    SmTArray<SmCurve*> & rSurfaceUVCurves, ///< [out]: all curve intervals that drop to SurfaceIsoCurves                               <br>
    SmTArray<double>   & pMaxDropDists,    ///< [out]: max m_crCurveToDrop SmpPoint to SurfDropPt dist for every output curve          <br>
    SmTArray<double>   & rDeviations       ///< [out]: max m_crCurveToDrop SmpPoint to DropSurfNormLine dist for every output curve    <br>
  );

  virtual SmStatus ComputeStepSize
  (
    SmTracePnt & rTracePnt,               ///< [in ]: Last found DropPoint                             <br>
    double       dOldStepSize,            ///< [in ]: StepSize to last found DropPoint                 <br>
    double     & rdNewStepSize            ///< [out]: StepSize to next Point to Drop.                  <br>
                                          ///<      : 0.0 = no more steps, CurrPnt is on a boundary    <br>
  );

  virtual SmStatus TraceCurve2
  (
    SmTracePnt         & rStartPoint,         ///< [in ]:                                                                                             <br>
    SmTArray<SmCurve*> & r3DCurves,           ///< [out]: used when dropCurve walks in/out of a boundary and SmSurfaceTracer::TraceCurve() is called  <br>
    SmTArray<SmCurve*> & rSurfaceUVCurves,    ///< [out]: UVDropCurves (more than 1 when dropping to a seam or wlaking in/out of a boundary)          <br>
    SmTArray<double>   & pMaxDropDists,       ///< [out]: max drop distances                                                                          <br>
    SmTArray<double>   & rDeviations          ///< [out]: deviations from normal                                                                      <br>
  );

  virtual SmStatus ComputeSpanDeviation
  (
    const SmTracePnt & crCurrPnt,              ///< [in ]:       <br>
    const SmTracePnt & crNextPnt,              ///< [in ]:       <br>
    double             dStepSize,              ///< [in ]:       <br>
    double           & rdDeviationFound,       ///< [in ]:       <br>
    SmBoolean        & rbSatisfiesTolerances   ///< [in ]:       <br>
  );

  virtual SmStatus ComputeNextPoint
  (
    SmTracePnt & rTracePnt,                   ///< [in ]: Curr Trace point                                                                       <br>
    double       dStepSize,                   ///< [in ]: Param StepSize for Curve being dropped                                                 <br>
    SmTracePnt & rNextTSP,                    ///< [out]: Next TracePnt = Dropped(Curve(TracePnt.m_dCurveParameter + dStepSize)                  <br>
    SmBoolean  & rbFoundGoodPoint,            ///< [out]: TRUE =                                                                                 <br>
    double     & rdDeviationFound,            ///< [out]: distance from NextPnt3d to Surface                                                     <br>
    double     & rdAngleFoundRad,             ///< [out]:                                                                                        <br>
    SmBoolean  & rbClipped,                   ///< [out]: TRUE = TracePnt.m_dCurveParameter + dStepSize > m_vWorkingInterval.GetMax              <br>
    SmBoolean  & rbBoundaryHit                ///< [out]: TRUE = TracePnt on m_vUVDomain boundary and                                            <br>
                                              ///<      : TracePnt.m_vUVCurvePnt + dStepSize * TracePnt.m_vUVCurveTangent is out of m_vUVDomain  <br>
  );

  virtual SmStatus TestPoint
  (
    SmTracePnt & rNextTSP,                    ///< [in ]: Point to test                      <br>
    SmBoolean & rbDone                        ///< [out]: FALSE = Not Done tracing Curve     <br>
  );

  virtual SmBoolean IsPointOnCurve
  (
    const SmPoint3d         & crPointToTest,     ///< [in ]:   <br>
    const SmTArray<SmCurve*> & cr3DCurves        ///< [in ]:   <br>
  ) const;
                           
  // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
  SM_COMMON(SmSurfaceDropCurve, SmSurfaceTracer, SmSurfaceDropCurve_TYPE) ;

} ; // end class SmSurfaceDropCurve


#endif // __SMSURFACEDROPCURVE_H__

