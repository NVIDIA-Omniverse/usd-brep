// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSurfaceTracer.h
* PURPOSE: Header file for SmSurfaceTracer object.
**********************************************************************/

#ifndef __SMSURFACETRACER_H__
#define __SMSURFACETRACER_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMEXTENT3D_H__
#include <SmExtent3d.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMGLOBALSOLVER_H__
#include <SmGlobalSolver.h>
#endif

#ifndef __SMSURFACE_H__
#include <SmSurface.h>
#endif

#ifndef __SMTLIST_H__
#include <SmTList.h>
#endif

enum SmTracePointType {
    SM_TP_UNDEF,
    SM_TP_NORMAL,
    SM_TP_SINGULARITY,
    SM_TP_TANGENT_POINT,
    SM_TP_TANGENT_LINE
};

/*******************************************************************//**
PURPOSE: Provide the storage for a trace point.

NOTES: static class object - don't add virtual methods to SmTracePnt
***********************************************************************/
class SmTracePnt : public SmListNode
{
public:
    double            m_dCurveParameter;
    SmVector3d        m_v3DCurvePV[3];      // [0] = DropCurve 3d Pos
                                            // [1] = DropCurve 3d tangent (not unitized)
                                            // [2] = Not Used - room for DropCurve 2nd derivative
    SmVector2d        m_vUVCurvePV[3];      // [0] = UVDropCurve 2d position
                                            // [1] = UVDropCurve 2d tangent
    SmVector3d        m_vSurfacePV[3][3];   // 3d pos, tang_udir, tang_vdir etc 
                                            // [0][0] = 3d pos
                                            // [1][0] = du = tang_udir
                                            // [0][1] = dv = tang_vdir
                                            // [1][1] = duv
                                            // [2][0] = duu
                                            // [0][2] = dvv
                                            // [2][2] = unitized surf_norm
    double            m_dSurfDropDist ;     // DropCurve(CurveParam) to DropSurfPoint dist in DropSurfNormLine dir
    double            m_dSurfNormLineDist ; // DropCurve(CurveParam) to DropSurfNormLine min dist
    SmTracePointType  m_ePointType;
    long              m_lUserLong;    // User values should be #defined to
    double            m_dUserDouble1; // something with a reasonable name when
    double            m_dUserDouble2; // being used.

    SmTracePnt() : m_dCurveParameter(SM_UNDEF_DOUBLE),
                   m_dSurfDropDist(SM_UNDEF_DOUBLE),
                   m_dSurfNormLineDist(SM_UNDEF_DOUBLE),
                   m_ePointType(SM_TP_UNDEF),
                   m_lUserLong(SM_UNDEF_ULONG),
                   m_dUserDouble1(SM_UNDEF_DOUBLE),
                   m_dUserDouble2(SM_UNDEF_DOUBLE)
                   { }

   ~SmTracePnt()  
   { 
     SmListNode::ReSet() ;   // clear Next/Prev pointers
     m_dCurveParameter   = SM_UNDEF_DOUBLE ;
     m_dSurfDropDist     = SM_UNDEF_DOUBLE ;
     m_dSurfNormLineDist = SM_UNDEF_DOUBLE ;
     m_ePointType        = SM_TP_UNDEF ;
     m_lUserLong         = SM_UNDEF_ULONG ;
     m_dUserDouble1      = SM_UNDEF_DOUBLE ;
     m_dUserDouble2      = SM_UNDEF_DOUBLE ;
   }

    SmDisplayList *Draw(const SmContext *pContext) const ; // NotUsed: in : pContext
    void Dump() const ;

} ; // end class SmTracePnt

/*******************************************************************//**
PURPOSE: The surface tracer class provides an abstract base class 
    which handles much of the framework for tracing curves on a single
    surface.  

NOTES: Surface tracer requires G1 continuity within the given domain.
***********************************************************************/
class SmSurfaceTracer : public SmGlobalSolver
{
protected:
  const SmContext   * m_cpContext;              //      
  const SmSurface   * m_cpSurface;              // The surface upon which the curve is being traced     
  SmExtent2d          m_vUVDomain;              //      
  double              m_dThisAngTolRad;         // Max allowed CurveTangent angle change between any pair of TracePnts     
  double              m_dThisApproxTol3d;       // Max allowed Curve deviation between any pair of TracePnts        
  SmMemBlockMgr       m_vTrcPntMgr;             // manages blocks or SmTsectPnts for m_vCurvePoints and m_vStartPoints    
  SmTracePnt*         m_pStartPoint;            //      
  SmTList<SmTracePnt> m_vCurvePoints;           // trace curve sample points - contains TsectPnts allocated by m_vTSPntMgr     
  double              m_dCurveTraceDirection;   //      
  SmBoolean           m_bCurveIsClosed;         //      
  SmTList<SmTracePnt> m_vStartPoints;           // list of start points for each curve to be traced
                                                // no duplicates - no end points - contains TsectPnts allocated by m_vTSPntMgr 
public:
  // constructor
  SmSurfaceTracer
  (
    const SmSurface  * cpSurface, 
    const SmExtent2d & crUVDomain
  );

  // destructor
  virtual ~SmSurfaceTracer() {};

  double GetThisApproxTol3d() const { SM_ASSERT_TOL(m_dThisApproxTol3d) ; return m_dThisApproxTol3d; } 
  double GetThisAngTolRad  () const { SM_ASSERT_TOL(m_dThisAngTolRad) ; return m_dThisAngTolRad; } 

  // top level trace interface for SmSurfaceDropCurve and SmSurfaceSilhouette
  virtual SmStatus DoTrace
  (
    const SmContext     & crContext,               ///< [in ]: context for new object construction                                                            <br>
    const SmApproxTol3d * pOptApproxTol3d,         ///< [in ]: opt AppoxTol3d  val to store in m_dThisApproxTol3d                                             <br>
    const double        * pdOptAngTolRad,          ///< [in ]: opt AngleTolRad val to store in m_dThisAngTolRad                                               <br>
    SmTArray<SmCurve*>  * p3DCurves,               ///< [out]: traced 3dcurves                                                                                <br>
    SmTArray<SmCurve*>  * pSurfaceUVCurves,        ///< [out]: traced UVCurves                                                                                <br>
    SmTArray<double>    * rMaxDropDists,           ///< [out]: max m_crCurveToDrop or SilhouetteCrv SmpPoint to SurfDropPt dist for every output curve        <br>
    SmTArray<double>    * pDeviations              ///< [out]: max m_crCurveToDrop or SilhouetteCrv SmpPoint to DropSurfNormLine dist for every output curve  <br>
  );            

  virtual SmBoolean IsPointOnCurve
  (
    const SmPoint3d          & crPointToTest,      ///< [in ]: point to test                        <br>
    const SmTArray<SmCurve*> & cr3DCurves          ///< [in ]: Previously traced curve solutions    <br>
  ) const;
                             
  virtual SmStatus FindBoundaryStartPoints
  (
    SmTArray<SmCurve*> & r3DCurves,                ///< [out]: 3d curves found by SmSurfaceSilhouett - not used by SmSurfaceDropCurve        <br>
    SmTArray<SmCurve*> & rSurfaceUVCurves,         ///< [out]: all curve intervals that drop to SurfaceIsoCurves                             <br>
    SmTArray<double>   & rMaxDropDists,            ///< [out]: max m_crCurveToDrop SmpPoint to SurfDropPt dist for every output curve        <br>
    SmTArray<double>   & rDeviation                ///< [out]: max m_crCurveToDrop SmpPoint to DropSurfNormLine dist for every output curve  <br>
  );

  virtual SmStatus ComputeSpanDeviation
  (
    const SmTracePnt & crCurrPnt,                  ///< [in ]:       <br>
    const SmTracePnt & crNextPnt,                  ///< [in ]:       <br>
    double             dStepSize,                  ///< [in ]:       <br>
    double           & rdDeviationFound,           ///< [in ]:       <br>
    SmBoolean        & rbSatisfiesTolerances       ///< [in ]:       <br>
  );

  virtual SmStatus LocalPointSolve
  (
    const SmTracePnt & crCurrPnt,      
    const SmPoint2d  & crGuessUV,
    SmTracePnt       & crNextPnt,
    SmBoolean        & rbFoundAnswer,
    SmPoint2d        & rUVFound
  ) const;

  virtual SmStatus TraceCurve
  (
    SmTracePnt         & rStartPoint,      ///< [in ]: StartPnt for this trace sequence                                           <br>
    SmTArray<SmCurve*> & r3DCurves         ///< [in ]: previously traced curves, prevents steeping a previously found solution    <br>
  );

  SmStatus FlushCurve
  (
    SmTArray<SmCurve*> & r3DCurves,            ///< [out]: add new 3d BSplineCurve = Bezier cubic interp of m_vCurvePoints           <br>
    SmTArray<SmCurve*> & rSurfaceUVCurves,     ///< [out]: add new 2d UVTrimCurve  = Bezier cubic interp of m_vCurvePoints           <br>
    SmTArray<double>   & rMaxDropDists,        ///< [out]: When pOptCurve!=NULL, Max(Dot(pOptCurve(s)-NewCurve,SurfNorm), else 0.0   <br>
    SmTArray<double>   & rDeviations,          ///< [out]: Max(m_vCurvePoints.Deviation = Dist(                                      <br>
    const SmCurve      * pOptCurve = NULL      ///< [in ]: default:[NULL}                                                            <br>
  );

  virtual SmStatus FindInteriorCurves
  (
    SmTArray<SmCurve*> & r3DCurves,            ///< [in,out]: already-found sols; Added to with new solutions    <br>
    SmTArray<SmCurve*> & rSurfaceUVCurves,     ///< [in,out]: already-found sols; Added to with new solutions    <br>
    SmTArray<double>   & rMaxDropDists,        ///< [in,out]: already-found sols; Added to with new solutions    <br>
    SmTArray<double>   & rDeviations           ///< [in,out]: already-found sols; Added to with new solutions    <br>
  );

  virtual SmStatus ComputeStepSize
  (
    SmTracePnt & rTracePnt,                    ///< NotUsed: [in ]: TracePnt to step from                             <br>
    double       dOldStepSize,                 ///< [in ]: last (or appropriate) TraceCurve UVstep size      <br>
    double     & rdNewStepSize                 ///< [out]: TraceCurve UVstepSize from crCurrPnt to NextPnt   <br>
  );

  virtual SmStatus ComputePointValues
  (
    const SmPoint2d & crUV,                   ///< [in ]: target SurfacePoint                                                  <br>
    SmTracePnt      & rCurrPnt,               ///< [in,out]: target CurvePoint, set with SurfacePoint properties               <br>
    SmTracePnt      * pPrevPnt = NULL,        ///< [in ]: Last intersection point on curve being stepped out, NULL to ignore   <br>
                                              ///<      : When supplied used to handle singularity cases.                      <br>
    double          * pdStepSize = NULL       ///< [in ]: pdStepSize = NOT USED                                                <br>
                                              ///<      : distance to step back from singularities to try and find             <br>
                                              ///<      : a nearby neighbor to use to computePointValues, NULL to ignore       <br>
  );

  virtual SmStatus ComputeNextPoint
  (
    SmTracePnt & rTracePnt,                   ///< [in ]: Current TraceCurve TracePnt                              <br>
    double       dStepSize,                   ///< [in ]: UVStepSize from rCurrPnt to rNextPnt                     <br>
    SmTracePnt & rNextTSP,                    ///< [out]: Next TraceCurve TracePnt                                 <br>
    SmBoolean  & rbFoundGoodPoint,            ///< [out]: we were able to find a good point.                       <br>
    double     & rdDeviationFound,            ///< [out]:                                                          <br>
    double     & rdAngleFoundRad,             ///< [out]:                                                          <br>
    SmBoolean  & rbClipped,                   ///< [out]: step would cross a boundary, and is clipped.             <br>
    SmBoolean  & rbBoundaryHit                ///< [out]: CurrPnt was already on a boundary, and step is leaving.  <br>
  );

  virtual SmStatus TestIsoCurve
  (
    SmSurfParamType eSurfParam,                ///< [in ]: oneof SM_SP_U=const U Cuve or SM_SP_V=const V Curve     <br>
    double          dParam,                    ///< [in ]: constant param value                                    <br>
    SmBoolean     & rbBoundaryIsSolution       ///< [out]: TRUE = IsoParamCurve is solution, FALSE=not             <br>
  );

  virtual SmStatus ReverseCurveDirection();

  virtual SmStatus FindSolutionsOnBoundaryCurve
  (
    const SmBSplineCurve & crBoundaryCurve,  
    const SmExtent1d     & crInterval,
    ULONG                  lSide, 
    SmSurfParamType        eSurfParam,
    SmSolutionArray      & crSolutions
  ) const;

  virtual SmBoolean DoesPointLieOnCurve
  (
    SmPoint2d & rUV,          
    SmBoolean   bRefinePoint
  ) const;

  SmStatus AddPointToCurve(SmTracePnt & rNextTSP);

  SmStatus AddStartPoint(SmTracePnt & rStartTSP);

  // 
  virtual SmStatus TestPoint
  (
    SmTracePnt & rNextTSP, ///< [in ]: Point to test - gets snapped for close to closed trace cases     <br>
    SmBoolean  & rbDone    ///< [out]: TRUE = Done tracing Curve,                                       <br>
                           ///<      : when TestPnt is at a tangency or a singularity,                  <br>
                           ///<      :       or close to this TraceCurve's StartPnt                     <br>
                           ///<      :       or on the SurfaceUVBoundary.                               <br>
  );

  // GetType(), GetTypeString() GetClassType(), GetClassTypeString(), IsKindOf(), Dump() (Dump needs implementation)
  SM_COMMON(SmSurfaceTracer, SmGlobalSolver, SmSurfaceTracer_TYPE) ;

} ; // end class SmSurfaceTracer


#endif // __SMSURFACETRACER_H__

