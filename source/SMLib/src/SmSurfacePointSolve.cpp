// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmSurfacePointSolve.cpp
* PURPOSE: Header and implementation of new versions of SmSurface
*             methods GlobalPointSolve() and LocalPointSolve().
**********************************************************************/

#include "StdAfx.h"

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#include <SmGraphicsExtern.h>
#include <SmGraphicsOutput.h>

#include <SmGeomUtility.h>       // for smgu_LineClosestPoint

#include <SmLocalSolve1d.h>   // for SmTerminationReasonType
#include <SmBSplineSurface.h>
#include <SmOffsetSurface.h>
#include <SmSurfaceCache.h>
#include <SmIsoCurve.h>


/*******************************************************************//**
PURPOSE: Base class for Surface-Point solves.

NOTES:
  This abstract base class simply holds the data that is common to
  an instance of a surface-point solve operation.
  Global and local solvers may be derived from this.

  Allowed operations are SM_SP_MINIMIZE, SM_SP_NORMALIZE, and SM_SP_INTERSECT.
  SM_SP_MAXIMIZE is only partially implemented; the new solvers are not
  currently used for that.

  These are used only within this file, and so do not use SM_EXPORT.
  That could change if it's ever required, but that is not forseen.
***********************************************************************/
class SmSurfacePointSolver
{
 protected:
  // Data for this problem:
  const SmSurface        * m_pSrf;      // We do not own the surface.
  SmExtent2d               m_sDomain;
  SmPoint3d                m_sTestPt;
  SmPoint3d                m_sDirVec;   // For direction operations.
  SmSolverOperationType    m_eSolverOp;

  const SmBSplineSurface * m_pBSplSrf;  // Need an SmBSplineSurface for some operations.
                                           // This is the same object as m_pSrf.
  double                   m_dTightTol ;
  double                   m_dLooseTol;

 protected:
  // Constructors, destructors, assignment:
  SmSurfacePointSolver(const SmSurface             * pSurf,                 // in : 
                       const SmExtent2d            & crUVDomain,            // in : 
                             SmSolverOperationType   eSolverOp,             // in : 
                       const SmPoint3d             & crTestPoint,           // in : 
                             double                  dDistanceTolerance,    // in : 
                       const double                * cpdOptTargetDistance,  // NotUsed: in : 
                             SmSolutionRequestedType eSolutionRequested );  // NotUsed: in : 

  SmSurfacePointSolver(const SmSurfacePointSolver & crOther );

 virtual ~SmSurfacePointSolver() {}

  SmSurfacePointSolver & operator=(const SmSurfacePointSolver &crOther );

 public:

  // Access methods.

  const SmSurface      * SurfacePtr() const { return m_pSrf;      }
  const SmPoint3d      & TestPoint()  const { return m_sTestPt;   }
  const SmExtent2d     & Domain()     const { return m_sDomain;   }
  SmSolverOperationType  SolverOp()   const { return m_eSolverOp; }

  double TightTol()                   const { return m_dTightTol; }
  double LooseTol()                   const { return m_dLooseTol; }

} ; // end abstract base class SmSurfacePointSolver

/*******************************************************************//**
// TODO:
// - cpdOptTargetDistance and SmSolutionRequestedType eSolutionRequested in Global...
// - multiple hits for NORM?
// - Get rid of CreateIsoParametricCurve
// - don't call FindBoundarySolution() more than once for any edge.
//   - First find out whether it is (whether it gets into it).
// - const PrevEval's ?
// - Remove boundary-hit checking.
// - remove Crease handling?
***********************************************************************/

/*******************************************************************//**
PURPOSE: A class to solve global surface-point operations.

NOTES: This class is just a container for the global-solve methods.
***********************************************************************/
class SmSurfPtGlobalSolver : public SmSurfacePointSolver
{
 private:
  // Other info:
  SmBoolean m_bClosedInU; // Expensive to calculate, so do it once up front.
  SmBoolean m_bClosedInV;

 public:
  // Constructor:
  SmSurfPtGlobalSolver ( const SmSurface             * pSurf,
                         const SmExtent2d            & crUVDomain,
                               SmSolverOperationType   eSolverOp,
                         const SmPoint3d             & crTestPoint,
                               double                  dDistanceTolerance,
                         const double                * cpdOptTargetDistance,
                               SmSolutionRequestedType eSolutionRequested );

  SmStatus  Solve( SmSolutionArray & rSolutions );

  SmBoolean ClosedSurface( SmSurfParamType eWhichDir );

 private:
  // methods:
  SmStatus FindRoughGuess(SmPoint2d & sUVGuess,
                          ULONG     & lBestIdxU,
                          ULONG     & lBestIdxV );

  SmStatus FindPolygonGuess(SmPoint2d & sUVGuess,
                            ULONG     & lBestIdxU,
                            ULONG     & lBestIdxV,
                            SmBoolean & rbSuspicious );

  // Find ControlPointMesh triangle with UVPoint that maps closest to target m_sTestPt
  SmStatus FindPolygonTriangleGuess(SmPoint2d & sUVGuess,
                                    ULONG     & lBestIdxU,
                                    ULONG     & lBestIdxV,
                                    double    & rdBestDist,
                                    double    & rdAveCPtDist,
                                    SmBoolean & bSuspicious );

  SmStatus RefineGuess          (SmPoint2d & rUVGuess, ULONG & lBestIdxU, ULONG & lBestIdxV); // not used: out: lBestIdxU, lBestIdxV
  SmStatus RefineGuessPolygonLeg(SmPoint2d & rUVGuess, ULONG   lBestIdxU, ULONG   lBestIdxV);
  SmStatus RefineGuessByGrid    (SmPoint2d & rUVGuess, ULONG & lBestIdxU, ULONG & lBestIdxV);
  SmStatus RefinePatchByGrid    (const SmExtent2d & crSubDomain,
                                 int                iNumU, 
                                 int                iNumV,
                                 SmPoint2d        & rUVGuess,
                                 double           & rBestFuncVal );

  SmBoolean CheckBadGuessStride(   SmSurfaceEval & crGridPt0,     // in
                                /* SmSurfaceEval & crGridPt1,     // obsolete.  */
                                   SmSurfaceEval & rNewGridPt,    // out, if return is True
                                   double        & rdNewFuncVal); // out, if return is True
  double    CalcFuncVal( const SmPoint2d & rUVGuess );
  SmBoolean PushingBoundary(SmSurfaceEval    & sSurfEval,
                            double             dTol,
                            SmSurfParamType  * peWhichBound = NULL );
  SmBoolean IsLocalMin(SmSurfaceEval & rSurfEval, double dTol );
  SmBoolean IsLocalMax(SmSurfaceEval & rSurfEval, double dTol );
  SmStatus  CheckGuessForSeams(SmPoint2d & rUVGuess );
  SmStatus  CheckGuessForSeams(ULONG &rlBestIdxU,
                              ULONG &rlBestIdxV,
                              ULONG lNumU,
                              ULONG lNumV,
                              double *pCPts,
                              SmBoolean bRational );
  SmStatus  GetCreaseSubdomains(SmTArray< SmExtent2d > & sDomainList );
  SmStatus  CheckSeams(double             dTol,          // NotUsed: in : 
                       const SmSolution & crSol,         // out: 
                       SmSolutionArray  & rSolutions );  // out: 

  // Obsolete, removed:
  // SmStatus CheckGuessForCreases( const SmPoint2d & crUVGuess, SmBoolean & bFoundCrease );

}; // end class SmSurfPtGlobalSolver

/*******************************************************************//**
PURPOSE: Error Norm types

NOTES: These are used only within this file, and so do
           not use SM_EXPORT. That could change if it's ever required.
***********************************************************************/
enum SmSrfPtErrorNormType
{
  SM_SPN_SUM_ABS, // |F| + |G|, the L1 norm: the usual one.
  SM_SPN_SUM,     // F + G
  SM_SPN_F,
  SM_SPN_G,
  SM_SPN_F_ABS,
  SM_SPN_G_ABS,
  SM_SPN_MAX,     // ( |F| >= |G| ) ? F : G
  SM_SPN_MAX_ABS, // L-infinity norm.
  SM_SPN_GAP      // 3d distance.
};

/*******************************************************************//**
PURPOSE: A class to represent an iteration guess,
    and to find the next guess.

NOTES: Used by the Local solver.
    This is the main workhorse of surface-point solving.

IMPLEMENTATION NOTES ---
- This class is derived from SmSurfacePointSolver.  It is not really
  a solver, just part of a solver.  It is derived from SmSurfacPointSolver
  for the convenience of the problem data.  Because this class is copied
  a lot (unlike the Global solver), it might make sense not to derive it
  from the parent, but to keep a pointer to an SmSurfacePointSolver object,
  to avoid copying the data.  It could be changed to that, but the convenience
  appears to outweigh the overhead of coyping.

- Boundary hits: m_iBndHitU, m_iBndHitV, BoundHitsU(), BoundHitsV().
  These are not truly necessary, their function is accomplished by
  PushingBoundary() and FindBoundarySolution().  However, they do
  occasionally come into play (only when other mechanisms are failing), and
  they don't cost much, so they are not being removed at the present time.
***********************************************************************/
class SmSurfPtSolveStepper : public SmSurfacePointSolver
{
 private:
  SmSurfaceEval m_sSrfEval;     // point on surface.

  SmBoolean     m_bSet;         // allow lazy evaluation.

  int           m_iIter;        // current iteration count
  int           m_iBndHitU;     // boundary hit counts: see implementation notes.
  int           m_iBndHitV;
  double        m_dMaxStepFrac; // Largest allowable step (fraction of domain size)
  SmBoolean     m_bNRSuspect;   // if true, don't trust the Newton step,
                                // always double-check with a plane step.

  // Numerical values for this uv guess.  Valid iff m_bSet.
  SmVector3d    m_sGap ;
  SmVector3d    m_sGapU ;
  SmVector3d    m_sGapV ;
  double        m_dGapLen ;

  double        m_dFVal ;
  double        m_dGVal ;

  double        m_dFCheck ;
  double        m_dGCheck ;
  double        m_dSumCheck ;

 private:
  // Internal methods.
  SmStatus Eval();

 public:
  // Access methods:
  void   SetUV(SmPoint2d const &sUVPos ) { m_sSrfEval.SetUV( sUVPos ) ;
                                           m_bSet = FALSE;
                                         }

  const  SmPoint2d & UV() const          { return m_sSrfEval.GetUV(); }
  double MaxStep()        const          { return m_dMaxStepFrac; }

  SmPoint3d  & Pos()                     { return m_sSrfEval.Pos(); }
  SmVector3d & Su()                      { return m_sSrfEval.Su (); }
  SmVector3d & Sv()                      { return m_sSrfEval.Sv (); }
  SmVector3d & Suu()                     { return m_sSrfEval.Suu(); }
  SmVector3d & Suv()                     { return m_sSrfEval.Suv(); }
  SmVector3d & Svv()                     { return m_sSrfEval.Svv(); }

  SmVector3d & Normal()                  { return m_sSrfEval.Normal(); }

  SmVector3d & Gap()                     { Eval(); return m_sGap; }
  SmVector3d & GapU()                    { Eval(); return m_sGapU; }
  SmVector3d & GapV()                    { Eval(); return m_sGapV; }
  double       GapLength()               { Eval(); return m_dGapLen; }

  int          IterCount()               { return m_iIter; }
  SmBoolean    NRStepSuspect()           { return m_bNRSuspect; }

  void SetIterCount(int iNewCount)        { m_iIter = iNewCount; }
  void SetNRSuspect(SmBoolean bIsSuspect) { m_bNRSuspect = bIsSuspect; }

  int  BoundHitsU()                      { return m_iBndHitU; }
  int  BoundHitsV()                      { return m_iBndHitV; }

  // Constructors, destructor, assignment:
  SmSurfPtSolveStepper
    (SmSurface  const * pSurf,
     SmExtent2d const & crDomain,
     SmPoint3d  const & crTestPt,
     SmPoint2d  const & rUV,
     double dTightTol,
     double dLooseTol,
     SmSolverOperationType eSolverOp)    : SmSurfacePointSolver(pSurf,     crDomain,
                                                                eSolverOp, crTestPt,
                                                                dLooseTol,
                                                                NULL,           // cpdOptTargetDistance,
                                                                SM_SR_SINGLE ),
                                           m_sSrfEval( pSurf ),
                                           m_bSet( FALSE ),
                                           m_iIter(0),
                                           m_iBndHitU(0),
                                           m_iBndHitV(0),
                                           m_bNRSuspect( FALSE )

                                         {
                                           m_dTightTol = dTightTol;

                                           // Maximum single step: say 40% of domain...
                                           m_dMaxStepFrac = 0.40;

                                           // ...unless the surface is planar (or even just bilinear).
                                           // Note: if the surface is rectangular (zero twist vector; actually
                                           // any parallelogram qualifies), then a single Newton step will hit it
                                           // precisely.  But a bilinear surface (2x2 control points) need not be
                                           // a parallologram, and that case is not within NR's precision.  That
                                           // case is so simple, however, that a huge step is not dangerous; our
                                           // boundary checking will always catch it right away.
                                           // So allow big steps for any bilinear surface, not just parallelograms.

                                           if ( m_pBSplSrf != NULL )
                                             {
                                               if (     m_pBSplSrf->GetNumberControlPoints( SM_SP_U ) == 2
                                                    &&  m_pBSplSrf->GetNumberControlPoints( SM_SP_V ) == 2
                                                  )
                                                 {
                                                   m_dMaxStepFrac = 2.00; // allow anything.
                                                 }
                                             }

                                           m_sSrfEval.SetUV( rUV );
                                         }

  // Copy Constructor.
  // Do not copy the evaluated data, because in practice,
  // this would never be called with the expectation that it will
  // be at the same uv as the original.
  SmSurfPtSolveStepper( const SmSurfPtSolveStepper & crOther );
 virtual ~SmSurfPtSolveStepper() {}

  // Assignment operator.
  // This does copy the evaluated data.
  SmSurfPtSolveStepper &operator=( const SmSurfPtSolveStepper &crOther );

  // 'Working methods:'
  SmStatus AdjustStep(SmSurfPtSolveStepper & rPrevEval,  // in :
                      SmBoolean  & rbAdjusted,           // out:
                      double     & rdScale);             // out:

  SmBoolean CheckBounds(SmPoint2d  const & crCurrUV,  // in :
                        SmPoint2d  const & crNextUV,  // in :
                        SmExtent2d const & crDomain,  // in :
                        int & riHitsU,                // out: -1 low bound, +1 high, 0 neither.
                        int & riHitsV,                // out:
                        SmPoint2d & rClampedUV,       // out: unset if not clamped.
                        SmPoint2d & rClampedUVScaled, // out: unset if not clamped.
                        double    & rdStepScale);     // out:

  SmBoolean CheckCrease(SmSurfPtSolveStepper & rPrevEval,    // in :
                        SmBoolean  & rbFoundCreaseSolution); // out:

  void      Clamp()                   { SetUV( m_sDomain.ClampPoint2d( m_sSrfEval.GetUV() )); }
  SmBoolean Converged          ( double dTol );
  SmBoolean Converged          ( SmBoolean bUseTightTol );
  SmBoolean ConvergedOnBoundary(double            dTol,
                                SmSurfParamType * peWhichBound = NULL );
  SmBoolean DropToIsoCurve     (SmSurfParamType eWhichParam,
                                double dCreaseParamVal,
                                double dParamOnCrease);

  double    ErrorNorm           (SmSrfPtErrorNormType eNormType = SM_SPN_SUM_ABS );
  SmStatus  FindBoundarySolution(SmBoolean & rbIsOBSolution );
  SmStatus  FindNextStep        (SmSurfPtSolveStepper & rPrevEval,     // in :
                                 SmTerminationReasonType & reReason);  // out:

  void      HitBound            (SmSurfParamType eWhich, int iTopOrBottom );
  SmBoolean IsLocalMin          (double dTol );
  SmBoolean IsLocalMax          (double dTol );
  SmBoolean IsSingular          (SmSurfParamType * peWhichParam = NULL );
  SmStatus  LineSearch          (SmSurfPtSolveStepper & rPrevEval );
  SmStatus  LineSearchBracketted(SmSurfPtSolveStepper & rPrevEval,   // in :
                                 SmSrfPtErrorNormType   eNormType);  // in : must be one that allows negative values.
  SmBoolean PushingBoundary     (double dTol, SmSurfParamType *peWhichBound = NULL );
  SmBoolean SnapToBoundaries    (double dTol );

  SmTerminationReasonType Step  (SmSurfPtSolveStepper & rNextEval, SmBoolean bDoPlaneStep = FALSE );

  void Dump (int iDumpLevel = 0, const TCHAR *cMsg = NULL, SmStatus eStat = SM_SUCCESS );

  // Obsolete, removed:
  // SmStatus FindDirection(
  //       SmSurfPtSolveStepper & rCurrEval, // in
  //       SmBoolean  & rbFoundDirection );  // out
  // SmBoolean IsSolution( SmBoolean bUseTightTol );

};  // end class SmSurfPtSolveStepper

/*******************************************************************//**
PURPOSE: Implementation of SmSurfacePointSolver (abstract base class)

NOTES: Constructors, destructors and assignment operator only.
***********************************************************************/
SmSurfacePointSolver::SmSurfacePointSolver
 (const SmSurface         * pSurf,                // in : 
  const SmExtent2d        & crUVDomain,           // in : 
  SmSolverOperationType     eSolverOp,            // in : 
  const SmPoint3d         & crTestPoint,          // in : 
  double                    dDistanceTolerance,   // in : 
  const double            * cpdOptTargetDistance, // NotUsed: in : //cbi: do this.
  SmSolutionRequestedType   eSolutionRequested)   // NotUsed: in : //cbi: do this.
 : m_pSrf      (pSurf ),
   m_sDomain   (crUVDomain ),
   m_sTestPt   (crTestPoint ),
   m_eSolverOp (eSolverOp ),
   m_dLooseTol (dDistanceTolerance )
{
  SM_REF2(cpdOptTargetDistance, eSolutionRequested) ;
  // We need an SmBSplineSurface for some operations.
  // If it's an offset surface, work from the base surface.
  const SmSurface       * pBaseSrf = pSurf;
  const SmOffsetSurface * pOffSrf  = SM_CAST_PTR( SmOffsetSurface, pSurf );
  if ( pOffSrf != NULL )
    {
      pBaseSrf = pOffSrf->GetBaseSurface();
    }

  m_pBSplSrf = SM_CAST_PTR( SmBSplineSurface, pBaseSrf );
  if ( m_pBSplSrf && (!m_pBSplSrf->GetGwNurbPointer()))
    m_pBSplSrf = NULL;

  // Tight tolerance: tight, but it should be achievable.
  m_dTightTol = SM_EFF_ZERO * (1.0 + crTestPoint.GetMaxDimension());

  // For direction operations:
  m_sDirVec.Set( 0, 0, 0 );
  if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
    {
      //cbi maybe move or copy this comment:
      // The direction comes in as the test point.
      // Set the direction in m_sDirVec, and set m_sTestPt to be
      // on the appropriate face of the ordinary bounding box,
      // modestly expanded.
      //
      // Note that our objective functions use Gap = ( S(u,v) - TestPt ),
      // not ( TestPt - S(u,v) ), just to avoid extra minus signs,
      // so to find the maximum x-value, for instance, the gap vector
      // would be from high x back to the surface, ( -x, 0, 0 ).
      // So we flip the input vector; to maximize x we actually
      // minimize -x.

      m_sDirVec = - crTestPoint;

      m_sDirVec.Unitize();

      // Find a point to define a plane: anything in the right direction
      // that doesn't intersect the surface.
      // We'll use one corner of the bounding box, expanded.
      // That point gets put into m_sTestPt.
      SmExtent3d sBBox;
      m_pSrf->CalculateBoundingBox( m_sDomain, &sBBox, NULL, NULL );
      sBBox.ExpandRelative( 0.2 );

      m_sTestPt.x = m_sDirVec.x > 0 ? sBBox.GetMin().x : sBBox.GetMax().x;
      m_sTestPt.y = m_sDirVec.y > 0 ? sBBox.GetMin().y : sBBox.GetMax().y;
      m_sTestPt.z = m_sDirVec.z > 0 ? sBBox.GetMin().z : sBBox.GetMax().z;
    }
} // end SmSurfacePointSolver::SmSurfacePointSolver constructor

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmSurfacePointSolver::SmSurfacePointSolver
 ( SmSurfacePointSolver const &crOther)
 : m_pSrf      (crOther.m_pSrf ),
   m_sDomain   (crOther.m_sDomain ),
   m_sTestPt   (crOther.m_sTestPt ),
   m_sDirVec   (crOther.m_sDirVec ),
   m_eSolverOp (crOther.m_eSolverOp ),
   m_pBSplSrf  (crOther.m_pBSplSrf ),
   m_dTightTol (crOther.m_dTightTol ),
   m_dLooseTol (crOther.m_dLooseTol )
{

} // end SmSurfacePointSolver::SmSurfacePointSolver copy constructor

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmSurfacePointSolver & SmSurfacePointSolver::operator=
 (const SmSurfacePointSolver &crOther )
{
  m_pSrf      = crOther.m_pSrf;
  m_sDomain   = crOther.m_sDomain;
  m_sTestPt   = crOther.m_sTestPt;
  m_sDirVec   = crOther.m_sDirVec;
  m_eSolverOp = crOther.m_eSolverOp;
  m_pBSplSrf  = crOther.m_pBSplSrf;
  m_dTightTol = crOther.m_dTightTol;
  m_dLooseTol = crOther.m_dLooseTol;

  return *this;

} // end SmSurfacePointSolver::operator=

/*******************************************************************//**
PURPOSE: Implementation of global solver, SmSurfPtGlobalSolver

NOTES: One constructor, and all the methods.
***********************************************************************/
SmSurfPtGlobalSolver::SmSurfPtGlobalSolver
 (const SmSurface        * pSurf,
  const SmExtent2d       & crUVDomain,
  SmSolverOperationType    eSolverOp,
  const SmPoint3d        & crTestPoint,
  double                   dDistanceTolerance,
  const double           * cpdOptTargetDistance, //cbi: do this.
  SmSolutionRequestedType  eSolutionRequested)   //cbi: do this.
 : SmSurfacePointSolver(pSurf,
                        crUVDomain,
                        eSolverOp,
                        crTestPoint,
                        dDistanceTolerance,
                        cpdOptTargetDistance,
                        eSolutionRequested)
{
  // Evaluate 'lazily', in ClosedSurface().
  m_bClosedInU = UNSURE;
  m_bClosedInV = UNSURE;

} // end SmSurfPtGlobalSolver::SmSurfPtGlobalSolver constructor

/*******************************************************************//**
PURPOSE: SmSurfPtGlobalSolver access method.

NOTES:
***********************************************************************/
SmBoolean SmSurfPtGlobalSolver::ClosedSurface
 (SmSurfParamType eWhichDir )
{
  // Lazy evaluation: only as needed.
  SmBoolean bRet = FALSE;
  if ( eWhichDir == SM_SP_U )
    {
      if ( m_bClosedInU == UNSURE )
        { m_bClosedInU = m_pSrf->IsClosed( m_sDomain, SM_SP_U ); }

      bRet = m_bClosedInU;
    }
  else if ( eWhichDir == SM_SP_V )
    {
      if ( m_bClosedInV == UNSURE )
        { m_bClosedInV = m_pSrf->IsClosed( m_sDomain, SM_SP_V ); }

      bRet = m_bClosedInV;
    }
  return bRet;

} // end SmSurfPtGlobalSolver::ClosedSurface

/*******************************************************************//**
PURPOSE: Helper for gridding-refinement: calculate the function value
    at a surface point, according to the solver-operation type.

NOTES:
   Returns SM_BIG_DOUBLE in case of failure.
***********************************************************************/
double SmSurfPtGlobalSolver::CalcFuncVal( const SmPoint2d & crUV )
{
  double dRetVal = 0;
  SmPoint3d sSrfPt;

  // Use the distance for all operations other than NORMALIZE:
  // use the angle from the normal for that.

  if ( m_eSolverOp == SM_SO_NORMALIZE )
  {
      SmVector3d sSu, sSv;
      SmStatus eStat = m_pSrf->Evaluate1stDerivatives( crUV, FALSE, FALSE, sSrfPt, sSu, sSv );
      if ( eStat != SM_SUCCESS || sSrfPt.IsUndef())
        { return SM_BIG_DOUBLE; }

      SmVector3d sGap( sSrfPt - m_sTestPt );

      sSu.Unitize();
      sSv.Unitize();

      dRetVal += smos_Fabs( sGap.Dot( sSu ) );
      dRetVal += smos_Fabs( sGap.Dot( sSv ) );
  }
  else
  {
      SmStatus eStat = m_pSrf->EvaluatePoint( crUV, sSrfPt );
      if ( eStat != SM_SUCCESS || sSrfPt.IsUndef() )
        { return SM_BIG_DOUBLE; }

      SmVector3d sGap( sSrfPt - m_sTestPt );
      if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
      {
          double dDot = sGap.Dot( m_sDirVec );
          sGap = dDot * m_sDirVec;
      }
      dRetVal = sGap.Length();

      if ( m_eSolverOp == SM_SO_MAXIMIZE )
          { dRetVal = -dRetVal; }
  }

  return dRetVal;

} // end SmSurfPtGlobalSolver::CalcFuncVal

/*******************************************************************//**
PURPOSE: Find a first rough guess for global surface-point solving.

NOTES: There are a couple of things that will change the sensitivity
  of finding a rough guess.  One is the grid size, in this routine.
  Another is in RefinePatchByGrid(), the variable used to decide whether
  to do a second plane step.  And there is also the criteria in
  FindPolygonGuess() as to whether to return bSuspicious.  They are all
  tradeoffs between efficiency and robustness; robustness is of course
  essential, but we can try to be as efficient as possible.

  Note, it would always be possible to offer some control of this to the
  caller, at the GlobalPointSolve() level.  If speed is crucial and they
  know that their data is fairly well behaved, they could request a faster
  search, but with tough data [B51] they could crank up the robustness.
  Perhaps an argument that's a scale of 1 to 10, which we could use for
  setting these parameters.
***********************************************************************/
SmStatus SmSurfPtGlobalSolver::FindRoughGuess
 (SmPoint2d & rUVGuess,    // out:
  ULONG     & rlBestIdxU,  // out:
  ULONG     & rlBestIdxV)  // out:
{
  // init output
  rlBestIdxU = rlBestIdxV = 0;

  // locals
  SmBoolean bSuspicious = FALSE;

  // Find a guess from a BSplineSurface's control polygon
  SmStatus ePolyStat = FindPolygonGuess( rUVGuess, rlBestIdxU, rlBestIdxV, bSuspicious );

  // when FindPolygonGuess() worked without setting the bSuspicious flag 
  if ( ePolyStat == SM_SUCCESS && ! bSuspicious )
      { return SM_SUCCESS; }

  // arrive here - when FindPolygonGuess() did not yield a GuessUV or found a GuessUV with bSuspicious == TRUE

  // Next: Find the best BuessUV out of a Grid of UV samples.

  // Grid spacing can be more lenient now that RefinePatchByGrid() adds the use
  // of a plane step guess.  Also, the plane step works better as the span gets
  // smaller, so we can decrease the grid size as we proceed.
  // Grid 7x7, then 5x5, then 3x3.  [080718; B46; B51; B394]

  // locals
  int iIter;
  int iGridSizeU = 7; // 5 is too small [B394]
  int iGridSizeV = 7;
  int iNumIters  = 3;

  //cbi: m_pSrf->ApproximateSize() ?

  // If it's a single linear patch, don't subdivide.
  // Similar for just a few control points.
  int iNumCPts = m_pSrf->GetNumberControlPoints( SM_SP_U );
  if ( iNumCPts < iGridSizeU && iNumCPts > 0 )
    { iGridSizeU = iNumCPts; }

  iNumCPts = m_pSrf->GetNumberControlPoints( SM_SP_V );
  if ( iNumCPts < iGridSizeV && iNumCPts > 0 )
    { iGridSizeV = iNumCPts; }

  double dFractionU;
  double dFractionV;
  SmVector2d sDomainSize = m_sDomain.GetSize();
  SmVector2d sUVMin, sUVMax;

  SmExtent2d sSubDomain( m_sDomain );
  SmPoint2d  sGridUVGuess;
  SmStatus   eGridStat = SM_ERR;

  double dBestFuncVal = SM_BIG_DOUBLE;

  // for a sequence of ever smaller sub-domains centered on the last iterations best Guess
  // search a grid pf UVSamples for the best UVGuess point
  for(iIter=0;iIter<iNumIters;iIter++)
    {
      // Find the best GuessUVPoint by examing a grid of UVPoints
      eGridStat = RefinePatchByGrid( sSubDomain, iGridSizeU, iGridSizeV, sGridUVGuess, dBestFuncVal );
      if ( eGridStat != SM_SUCCESS )
        { break; }
      if ( iGridSizeU < 2 || iGridSizeV < 2 )  // Just to be safe.
        { break; }

      dFractionU = 1.0 / ( iGridSizeU-1 ) + 0.01;
      dFractionV = 1.0 / ( iGridSizeV-1 ) + 0.01;
      sDomainSize.x *= dFractionU;
      sDomainSize.y *= dFractionV;
      sUVMin = m_sDomain.ClampPoint2d( sGridUVGuess - sDomainSize );
      sUVMax = m_sDomain.ClampPoint2d( sGridUVGuess + sDomainSize );
      sSubDomain.SetMinMax( sUVMin, sUVMax );

      // Decrease grid size for the next iteration: 7-5-3.
      if ( iGridSizeU > 4 ) { iGridSizeU--; }
      if ( iGridSizeV > 4 ) { iGridSizeV--; }
      if ( iGridSizeU > 2 ) { iGridSizeU--; }
      if ( iGridSizeV > 2 ) { iGridSizeV--; }
    } // end iter sequence of zoomed in subDomains looking for refined UVGuessPoints

  // GWC NOTE: The code has evolved - There are now surfaces which are not derived from SmBSplineSurface
  //           and it's possible to arrive here when FindPolygonGUess failed and only RefinePatchByGrid() worked.
  //           Need to handle two cases:
  //           1. FindPolygonGuess failed due to a nonBSplineSurface : Use sGridUVGuess
  //           2. FindPolygonGuess worked with bSuspicious == TRUE  : Use better of sGridUVGuess and current rUVGuess from FindPolygonGuess.

  // If guesses from both FindPolygonGuess() and RefinePatchByGid(), pick the better one.
  if(   ePolyStat == SM_SUCCESS 
     && eGridStat == SM_SUCCESS )
    {
      // Get quality of both guesses to the m_sTestPt
      double dPolyDist = CalcFuncVal( rUVGuess ) ;
      double dGridDist = CalcFuncVal( sGridUVGuess ) ;

      // when either guess is uninitialized - quit 
      if(   dPolyDist >= SM_BIG_DOUBLE 
         && dGridDist >= SM_BIG_DOUBLE )
        { SER( SM_ERR ); }

      // when the GridGuess is better than the PolyGuess, use it
      if ( dGridDist < dPolyDist )
        {
          rUVGuess = sGridUVGuess;
        }
    } // end FindPolygonGuess() worked with bSuspicious == TRUE branch
  else if(eGridStat == SM_SUCCESS)
    {
      rUVGuess = sGridUVGuess;
    } // end FindPolygonGuess() failed and RefinePatchByGrid() search worked branch
  else
    {
      // both RefinePatchByGrid() and RefinePatchByGrid() failed
      rUVGuess.SetUninitialized() ;
      return(SM_ERR) ;
    }

  // all done
  return SM_SUCCESS;

} // end SmSurfPtGlobalSolver::FindRoughGuess

/*******************************************************************//**
PURPOSE: Find a UVguess using the control polygon that maps to a point 
         near the stored target point, m_sTestPt, for global surface-point 
         solving.

NOTES:
 - This routine works from the B-Spline control points.
   If no SmBSplineSurface is available, it will return SM_ERR.
   In that case, the caller must do something different (such as gridding).

   This routine makes that assumption that the control points are stored
   as 4-tuples of doubles, with v- varying faster.
***********************************************************************/
SmStatus SmSurfPtGlobalSolver::FindPolygonGuess
 (SmPoint2d & rUVGuess,     // out:
  ULONG     & rlBestIdxU,   // out:
  ULONG     & rlBestIdxV,   // out:
  SmBoolean & rbSuspicious) // out:
{
  // We need a SmBSplineSurface to work with control points.
  if ( m_pBSplSrf == NULL )
    {
      return SM_ERR;
    }

  // If PolygonTriangleGuess is suspicious, also check distance.
  rbSuspicious = FALSE;

  constexpr SmBoolean sbDoTriangleGuess=TRUE;
  if ( sbDoTriangleGuess )
    {
      double dBestDist, dAveCPtDist;

      // find closest ControlPoint triangle with point closest to target, m_sTestPt
      SmStatus eStat = FindPolygonTriangleGuess(rUVGuess,       // out: UVValue that maps through a ControlPoint Polygon closest to m_sTestPt
                                                rlBestIdxU,     // out: UIndex of ControlPolygon triangle vertex
                                                rlBestIdxV,     // out: VIndex of ControlPolygon triangle vertex
                                                dBestDist,      // out: Dist from m_sTestPt to ControlPolygon triangle
                                                dAveCPtDist,    // out: Measure of Polygon quality for approximations- average dist between control points
                                                rbSuspicious ); // out: TRUE = ControlPolygon likely to be far from surface (happens for rational surfaces)
                                                                //      FALSE= ControlPolygon guess is likely to be a good guess

      // When worried ControlPoint Mesh may be far from Surface
      if(   rbSuspicious
         && eStat       == SM_SUCCESS
         && m_eSolverOp != SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
        {
          // when the BestDist is small compared to the Average spacing between ControlPoints
          if ( dBestDist < dAveCPtDist / 100.0 ) // [100: B51 #6]
            {
              // Also check distance to the surface, not just to the polygon. [091002]
              SmPoint3d sSrfPt;
              m_pSrf->EvaluatePoint( rUVGuess, sSrfPt );
              double dSurfDist = sSrfPt.DistanceBetween( m_sTestPt );

              // when dist between the guessed surfPoint and the TestPt is small compared to average ControlPoint Spacing
              if ( dSurfDist < dAveCPtDist / 100.0 ) // [100: B51 #6]
                {
                  // clear our suspecions of this grid - things are working out Okay.
                  rbSuspicious = FALSE;
                }
            }
        } // end found a UVGuessPoint on a possibly suspicious ControlPointMesh

      // If it looks likt the UVGuess is good - return
      if ( ! rbSuspicious )
        {
          return SM_SUCCESS;
        }
    } // end asked to guess from ControlPointMesh Triangles check

  // arrive here - when ControlPointMesh Triangle trick was skipped or it 
  //               gave a suspicious answer - typically: a large GuessPoint 
  //               to TestPt distance on a rational surface.

  // Next try: Iter every ControlPoint and find the one closest to the m_sTestPt target

  // locals
  ULONG ii, jj;
  ULONG     lNumU, lNumV;
  double  * pCPts     = NULL;
  SmBoolean bRational = m_pBSplSrf->IsRational();
  SmStatus  eStat     = m_pBSplSrf->GetControlPointsPointer( lNumU, lNumV, pCPts );
  if ( eStat != SM_SUCCESS )
    { return SM_ERR; }

  int iTiedIdxU = -1;
  int iTiedIdxV = -1;
  ULONG lNumTies = 0;
  double dThisDist, dBestDist = SM_BIG_DOUBLE;
  if ( m_eSolverOp == SM_SO_MAXIMIZE )
    { dBestDist = -dBestDist; }
  double *pThisCPt = NULL;
  SmPoint3d sThisPt;

  // for every control Point in the ControlPointMesh
  for ( ii=0; ii<lNumU; ii++ )
    {
      for ( jj=0; jj<lNumV; jj++ )
        {
          pThisCPt = pCPts + 4 * ( ii * lNumV + jj );

          sThisPt.Set( pThisCPt[0], pThisCPt[1], pThisCPt[2] );
          if ( bRational )
            { sThisPt /= pThisCPt[3]; }

          // Set dThisDist: for m_eSolverOp so that the best dThisDist has the smallest signed value
          //   SM_SO_MAXIMIZE                    : - Dist from m_sTestPt to sThisPt
          //   SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE :   sThisPt.Dot( m_sDirVec )
          //   else                              :   Dist from m_sTestPt to sThisPt
          if ( m_eSolverOp == SM_SO_MAXIMIZE )
            {
              dThisDist = - m_sTestPt.DistanceBetween( sThisPt );
            }
          else if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
            {
              dThisDist = sThisPt.Dot( m_sDirVec );
            }
          else
            {
              dThisDist = m_sTestPt.DistanceBetween( sThisPt );
            }

          // Note: also check NORM.

          // save best found case
          if ( dThisDist < dBestDist - SM_EFF_ZERO )
            {
              dBestDist  = dThisDist;
              rlBestIdxU = ii;
              rlBestIdxV = jj;
              // No tie; reset.
              iTiedIdxU = iTiedIdxV = -1;
              lNumTies = 0;
            }
          // else if two equivalent choices are found
          else if ( dThisDist < dBestDist + SM_EFF_ZERO )
            {
              // 'Equal': record a tie.
              lNumTies++;
              iTiedIdxU = rlBestIdxU;
              iTiedIdxV = rlBestIdxV;
              rlBestIdxU = ii;
              rlBestIdxV = jj;
            }

        } // end iter j, every control point in the ControlPointMesh
    } // end iter i, every control point in the ControlPointMesh

  // Have to check here for seams.
  CheckGuessForSeams( rlBestIdxU, rlBestIdxV, lNumU, lNumV, pCPts, bRational );

  // Refine along the legs of the control polygon.

  SmPoint2d sUVGuess_Poly;
  eStat = RefineGuessPolygonLeg(sUVGuess_Poly,
                                rlBestIdxU,
                                rlBestIdxV) ;

  // Check for ties -- coincident control points.
  // If all u-pts the same, then v-idx would be the same.

  // Assume that either adjacent u-points are coincident,
  // or adjcent v-points are, but not both.
  // (That would be an illegal surface.)

  // Set this flag: if eStat doesn't get set, neither will this.
  dBestDist = SM_BIG_DOUBLE;

  SmPoint2d sThisPolyGuess;
  SmPoint3d sSrfPt;

  if ( lNumTies >= 2 && lNumTies == lNumU-1 && (ULONG)iTiedIdxV == rlBestIdxV )
    {
      // Adjacent u-points with the same value (presumably coincident).
      for ( ii = 0; ii < lNumU; ii++ )
        {
          eStat = RefineGuessPolygonLeg( sThisPolyGuess, ii, rlBestIdxV );

          if ( eStat == SM_SUCCESS )
            {
              m_pSrf->EvaluatePoint( sThisPolyGuess, sSrfPt );
              dThisDist = sSrfPt.DistanceBetween( m_sTestPt );
              if ( dThisDist < dBestDist )
                {
                  dBestDist = dThisDist;
                  sUVGuess_Poly = sThisPolyGuess;
                  rlBestIdxU = ii;

                  // Quit if we get a direct hit.
                  if ( dThisDist < LooseTol() )
                    { break; }
                }
            }
        } // end for

      // Reset this flag:
      eStat = ( dBestDist < SM_BIG_DOUBLE / 2 ) ? SM_SUCCESS : SM_ERR;
    }
  else if ( lNumTies >= 2 && lNumTies == lNumV-1 && (ULONG)iTiedIdxU == rlBestIdxU )
    {
      // Adjacent v-points.
      for ( ii = 0; ii < lNumV; ii++ )
        {
          eStat = RefineGuessPolygonLeg( sThisPolyGuess, rlBestIdxU, ii );

          if ( eStat == SM_SUCCESS )
            {
              m_pSrf->EvaluatePoint( sThisPolyGuess, sSrfPt );
              dThisDist = sSrfPt.DistanceBetween( m_sTestPt );
              if ( dThisDist < dBestDist )
                {
                  dBestDist = dThisDist;
                  sUVGuess_Poly = sThisPolyGuess;
                  rlBestIdxV = ii;

                  // Quit if we get a direct hit.
                  if ( dThisDist < LooseTol() )
                    { break; }
                }
            }
        } // end for

      // Reset this flag:
      eStat = ( dBestDist < SM_BIG_DOUBLE / 2 ) ? SM_SUCCESS : SM_ERR;
    } // end tied-value check.

  if ( eStat == SM_SUCCESS )
    {
      rUVGuess = sUVGuess_Poly;
    }
  else
    {
      // Use the Greville abscissae for the control point indices.
      rUVGuess.x = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_U, rlBestIdxU );
      rUVGuess.y = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_V, rlBestIdxV );
    }

  // set output
  rUVGuess = m_sDomain.ClampPoint2d( rUVGuess );

  // all done
  return SM_SUCCESS;

} // end SmSurfPtGlobalSolver::FindPolygonGuess

/*******************************************************************//**
PURPOSE: Given an index into the control net, find the closest approach
         of the test point to either of two triangles defined by the
         four local control points.

NOTES:
   Get the four control points: [[i j] [i+1 j] [i j+1] [i+1 j+1]] 
   where i and j are indices into the Controloints array, pCPts.

   Find the UVPoint on the two triangls [ [i j]     [i+1 j] [i j+1] ]
                                        [ [i+1 j+1] [i j+1] [i+1 j] ]
   closest to the target crTestPt
               [i j+1]---[i+1 j+1]
                  |   \    |
                  |    \   |
                  |     \  |
                [i j]----[i j+1]
       The two triangles in the Control Polygon tested by this call.

   This routine makes that assumption that the control points are stored
   as 4-tuples of doubles, with v- varying faster.
***********************************************************************/
static double GetTrianglesDist
 (const SmPoint3d & crTestPt,   // in : Point to which distance is minimized
  const double    * pCPts,      // in : Control point Array, sized:[lNumU, lNumV]
  SmBoolean         bRational,  // in : TRUE = BSpline is rational, FALSE=not
  ULONG             i,          // in : lowest i index of ControlPoint Square [[i j] [i+1 j] [i j+1] [i+1 j+1]]
  ULONG             j,          // in : lowest j index of ControlPoint Square [[i j] [i+1 j] [i j+1] [i+1 j+1]]
  ULONG             lNumU,      // NotUsed: in : size of ControlPoint array
  ULONG             lNumV,      // in : Size of ControlPoint array
  double          & rdU,        // out: U value of point on ControlPoint triangle closest to crTestPt
  double          & rdV)        // out: V value of point on ControlPoint triangle closest to crTestPt
{
  SM_REF1(lNumU) ;
  // get the four corners of this 'square'.
  SmPoint3d sPt00, sPt10, sPt01, sPt11;

  const double *pThisCPt = pCPts + 4 * ( i * lNumV + j );
  sPt00.Set( pThisCPt[0], pThisCPt[1], pThisCPt[2] );
  if ( bRational )
    { sPt00 /= pThisCPt[3]; }

  pThisCPt = pCPts + 4 * ( (i+1) * lNumV + j );
  sPt10.Set( pThisCPt[0], pThisCPt[1], pThisCPt[2] );
  if ( bRational )
    { sPt10 /= pThisCPt[3]; }

  pThisCPt = pCPts + 4 * ( i * lNumV + j+1 );
  sPt01.Set( pThisCPt[0], pThisCPt[1], pThisCPt[2] );
  if ( bRational )
    { sPt01 /= pThisCPt[3]; }

  pThisCPt = pCPts + 4 * ( (i+1) * lNumV + j+1 );
  sPt11.Set( pThisCPt[0], pThisCPt[1], pThisCPt[2] );
  if ( bRational )
    { sPt11 /= pThisCPt[3]; }

  double dDist1, dDist2;
  double dU1, dV1, dU2, dV2;
  SmPoint3d sDummy;
  smgu_TrianglePointDistance( sPt00, sPt10, sPt01, crTestPt, dDist1, sDummy, dU1, dV1 );
  smgu_TrianglePointDistance( sPt11, sPt01, sPt10, crTestPt, dDist2, sDummy, dU2, dV2 );

  double dRetVal;
  if ( dDist1 <= dDist2 )
    {
      dRetVal = dDist1;
      rdU = dU1;
      rdV = dV1;
    }
  else
    {
      dRetVal = dDist2;
      rdU = 1.0 - dU2;
      rdV = 1.0 - dV2;
    }
  return dRetVal;

} // end GetTrianglesDist

/*******************************************************************//**
PURPOSE: Given an index into the control net, find whether the control
            net is poorly-behaved, so that FindPolygonTriangleGuess()
            method could be fooled.

NOTES:
   This routine makes that assumption that the control points are stored
   as 4-tuples of doubles, with v- varying faster.
***********************************************************************/
static SmBoolean WellBehavedControlNet
(const SmPoint3d & crTestPt,    // NotUsed: in : 
 const double    * pCPts,       // in : 
 SmBoolean         bRational,   // in : 
 ULONG             ii,          // in : 
 ULONG             jj,          // in : 
 ULONG             lNumU,       // in : 
 ULONG             lNumV)       // in : 
{
  SM_REF1(crTestPt) ;
  // Method: check for sharp angles.  Check u- and v-directions.
  // Limit must be no greater than -0.61. [080830]
  // Also check angle between u and v directions (angle can't be too acute or too oblique)
  double dCosine;
  constexpr double dCosineLimit0 = -0.61; // about 127.6 degrees

  SmPoint3d sPt0, sPt1, sPt2;

  // Get the mid point, sPt1.
  const double *pThisCPt = pCPts + 4 * ( ii * lNumV + jj );
  sPt1.Set( pThisCPt[0], pThisCPt[1], pThisCPt[2] );
  if ( bRational )
    { sPt1 /= pThisCPt[3]; }

  // First check in u-direction.
  if ( ii > 0 && ii < lNumU-1 )
  {
      // get the neighboring points in this direction.
      pThisCPt = pCPts + 4 * ( (ii-1) * lNumV + jj );
      sPt0.Set( pThisCPt[0], pThisCPt[1], pThisCPt[2] );
      if ( bRational )
        { sPt0 /= pThisCPt[3]; }

      pThisCPt = pCPts + 4 * ( (ii+1) * lNumV + jj );
      sPt2.Set( pThisCPt[0], pThisCPt[1], pThisCPt[2] );
      if ( bRational )
        { sPt2 /= pThisCPt[3]; }

      SmVector3d sVec1( sPt1 - sPt0 );
      SmVector3d sVec2( sPt2 - sPt1 );

      // Save a call to AngleBetween (and acos), because we don't need the
      // angle, just a limit.  (Also, AngleBetween generates a warning message
      // on zero-length vectors, which are common here.)
      double dLen1 = sVec1.Length();
      double dLen2 = sVec2.Length();

      if ( dLen1 < SM_EFF_ZERO || dLen2 < SM_EFF_ZERO )
      {
          // One or both zero-vectors (coincident control points).
          // This is ok only if we're on the low or high end of v-range.
          if ( jj == 0 || jj >= lNumV-1 )
          {
              // At a singularity: tests work fine here.
              return TRUE;
          }
          else
          {
              // Coincident control points on the interior: suspicious.
              return FALSE;
          }
      }

      dCosine = sVec1.Dot( sVec2 ) / ( dLen1 * dLen2 );

      if ( dCosine < dCosineLimit0 )
        { return FALSE; }

  } // end if on interior in U.

  // Repeat check in v-direction.
  if ( jj > 0 && jj < lNumV-1 )
  {
      // get the neighboring points in this direction.
      pThisCPt = pCPts + 4 * ( ii * lNumV + jj-1 );
      sPt0.Set( pThisCPt[0], pThisCPt[1], pThisCPt[2] );
      if ( bRational )
        { sPt0 /= pThisCPt[3]; }

      pThisCPt = pCPts + 4 * ( ii * lNumV + jj+1 );
      sPt2.Set( pThisCPt[0], pThisCPt[1], pThisCPt[2] );
      if ( bRational )
        { sPt2 /= pThisCPt[3]; }

      SmVector3d sVec1( sPt1 - sPt0 );
      SmVector3d sVec2( sPt2 - sPt1 );

      double dLen1 = sVec1.Length();
      double dLen2 = sVec2.Length();

      if ( dLen1 < SM_EFF_ZERO || dLen2 < SM_EFF_ZERO )
      {
          // One or both zero-vectors (coincident control points).
          // This is ok only if we're on the low or high end of u-range.
          if ( ii == 0 || ii >= lNumU-1 )
          {
              // At a singularity: tests work fine here.
              return TRUE;
          }
          else
          {
              // Coincident control points on the interior: suspicious.
              return FALSE;
          }
      }

      dCosine = sVec1.Dot( sVec2 ) / ( dLen1 * dLen2 );

      if ( dCosine < dCosineLimit0 )
        { return FALSE; }

  } // end if on interior in U.

  // Otherwise: arbitrary guess to limit how often `return TRUE` occurs.
  constexpr double dCosineLimit1 = -0.80; // about 143.1 degrees

  // Check angle between U and V directions
  if ( ii < lNumU -1 && jj < lNumV-1)
  {
      pThisCPt = pCPts + 4 * ( (ii+1) * lNumV + jj );
      sPt0.Set( pThisCPt[0], pThisCPt[1], pThisCPt[2] );
      if ( bRational )
        { sPt0 /= pThisCPt[3]; }

      pThisCPt = pCPts + 4 * ( ii * lNumV + jj+1 );
      sPt2.Set( pThisCPt[0], pThisCPt[1], pThisCPt[2] );
      if ( bRational )
        { sPt2 /= pThisCPt[3]; }

      SmVector3d sVec1( sPt0 - sPt1 );
      SmVector3d sVec2( sPt2 - sPt1 );

      double dLen1 = sVec1.Length();
      double dLen2 = sVec2.Length();

      if ( dLen1 < SM_EFF_ZERO )
      {
          // One zero-vectors (coincident control points).
          // This is ok only if we're on the low or high end of v-range.
          if ( jj == 0 || jj >= lNumV-1 )
          {
              // At a singularity: tests work fine here.
              return TRUE;
          }
          else
          {
              // Coincident control points on the interior: suspicious.
              return FALSE;
          }
      }
      else if ( dLen2 < SM_EFF_ZERO )
      {
          // One zero-vectors (coincident control points).
          // This is ok only if we're on the low or high end of u-range.
          if ( ii == 0 || ii >= lNumU-1 )
          {
              // At a singularity: tests work fine here.
              return TRUE;
          }
          else
          {
              // Coincident control points on the interior: suspicious.
              return FALSE;
          }
      }

      dCosine = sVec1.Dot( sVec2 ) / ( dLen1 * dLen2 );

      if (    dCosine <  dCosineLimit1
           || dCosine > -dCosineLimit1 )
        { return FALSE; }
  }

  // No complaints.
  return TRUE;

} // end WellBehavedControlNet


/*******************************************************************//**
PURPOSE: Find a guess for global surface-point solving
            using the control net.

NOTES:
 - This routine works from the B-Spline control points.
   If no SmBSplineSurface is available, it will return SM_ERR.
   In that case, the caller must do something different (such as gridding).

   This routine makes that assumption that the control points are stored
   as 4-tuples of doubles, with v- varying faster.

   This routine finds the closest point on any triangle in the
   control net.  It has been demonstrated that this is more
   robust than working from control points, or polygon legs.
   However, this can still be fooled, if the net is badly behaved,
   with sharp corners, etc.  There can be points on the surface that
   are closer to a distant part of the control net than to the region
   that defines the surface locally.
***********************************************************************/
SmStatus SmSurfPtGlobalSolver::FindPolygonTriangleGuess
 (SmPoint2d & sUVGuess,       // out: UVValue that maps through a ControlPoint Polygon Triangle closest to m_sTestPt              
  ULONG     & rlBestIdxU,     // out: UIndex of ControlPolygon triangle vertex                                           
  ULONG     & rlBestIdxV,     // out: VIndex of ControlPolygon triangle vertex                                           
  double    & rdBestDist,     // out: Dist from m_sTestPt to ControlPolygon triangle                                     
  double    & rdAveCPtDist,   // out: Measure of Polygon quality for approximations- average dist between control points 
  SmBoolean & rbSuspicious )  // out: TRUE = ControlPolygon likely to be far from surface (happens for rational surfaces)
                              //      FALSE= ControlPolygon guess is likely to be a good guess                           
{
  // We need a SmBSplineSurface to work with control points.
  if ( m_pBSplSrf == NULL )
    {
      return SM_ERR;
    }

  // locals
  ULONG ii, jj;
  ULONG         lNumU, lNumV;
  double       *pCPts     = NULL;
  SmStatus      eStat     = m_pBSplSrf->GetControlPointsPointer( lNumU, lNumV, pCPts );
  SmBoolean     bRational = m_pBSplSrf->IsRational();
  double        dThisDist, dBestDist = SM_BIG_DOUBLE;
  double        dThisLocalU = -1, dBestLocalU = -1;
  double        dThisLocalV = -1, dBestLocalV = -1;
  const double *pThisCPt    = pCPts;
  SmPoint3d sThisPt;

  // For collecting average distances:
  SmPoint3d sPt0, sPt1;
  double dAccumDist = 0;
  ULONG lDistCount = 0;

  // warn the caller!
  // Rational surfaces can have polygons whose shapes are very different
  // from the surface, such as cylinders with weights 5, etc. [B29]
  rbSuspicious = bRational ? TRUE : FALSE ;

  // check state - quit when GetControlPointsPointer fails
  if ( eStat != SM_SUCCESS )
    { return SM_ERR; }

  // For operation MAXIMIZE - negate the BestDist guess
  if ( m_eSolverOp == SM_SO_MAXIMIZE )
    { dBestDist = -dBestDist; }


  // for every grid point
  for ( ii=0; ii<lNumU; ii++ )
    {
      for ( jj=0; jj<lNumV; jj++ )
        {
          // First, check for a badly-behaved control net.
          // Sometimes a point right on the surface can be closer to
          // a distant part of the control net than to the part that
          // controls the surface near the point.  [080718, 080830]
          if(   ! rbSuspicious
             && ! WellBehavedControlNet(m_sTestPt, pCPts, bRational,
                                        ii, jj, lNumU, lNumV ))
            {
              rbSuspicious = TRUE; // Warn the caller.
            }

          // Set this ctrl pt, sPt0.
          pThisCPt = pCPts + 4 * ( ii * lNumV + jj );
          sPt0.Set( pThisCPt[0], pThisCPt[1], pThisCPt[2] );
          if ( bRational ) { sPt0 /= pThisCPt[3]; }

          // Set dThisDist.
          // In all cases, the best is the smallest signed value.

          if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
            {
              dThisDist = sPt0.Dot( m_sDirVec );
              dThisLocalU = dThisLocalV = -1;
            }
          else // not m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE 
            {
              // Note: GetTrianglesDist() works on the current control point,
              // and the points 'ahead' of it: ii+1, jj+1.
              dThisDist = SM_BIG_DOUBLE;
              if ( ii < lNumU-1 && jj < lNumV-1 )
                {
                  dThisDist = GetTrianglesDist(m_sTestPt, pCPts, bRational,
                                               ii, jj, lNumU, lNumV, 
                                               dThisLocalU, dThisLocalV) ;

                  if ( m_eSolverOp == SM_SO_MAXIMIZE )
                    {
                      dThisDist = - dThisDist;
                    }
                } // end skip call for last row and column of control points check
            } // end m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE branch

          // Note: also check NORM.

          // save the best found case
          if ( dThisDist < dBestDist )
            {
              dBestDist   = dThisDist;
              rlBestIdxU  = ii;
              rlBestIdxV  = jj;
              dBestLocalU = dThisLocalU;
              dBestLocalV = dThisLocalV;
            }

          // Collect info on control point spacing.

          // same col control point distances
          if ( ii > 0 )
            {
              // accumulate dist between same col control points
              pThisCPt = pCPts + 4 * ( (ii-1) * lNumV + jj );
              sPt1.Set( pThisCPt[0], pThisCPt[1], pThisCPt[2] );
              if ( bRational ) { sPt1 /= pThisCPt[3]; }

              dAccumDist += sPt0.DistanceBetween( sPt1 );
              lDistCount++;
            }

          // same row control point distances
          if ( jj > 0 )
            {
              // accumulate dist between same row control points
              pThisCPt = pCPts + 4 * ( ii * lNumV + jj-1 );
              sPt1.Set( pThisCPt[0], pThisCPt[1], pThisCPt[2] );
              if ( bRational ) { sPt1 /= pThisCPt[3]; }

              dAccumDist += sPt0.DistanceBetween( sPt1 );
              lDistCount++;
            }

        } // end iter jj, every control points.
    } // end iter ii, every control points.

  // Other outputs
  rdBestDist   = dBestDist;
  rdAveCPtDist = dAccumDist / lDistCount;

  // Have to check here for seams.
  CheckGuessForSeams( rlBestIdxU, rlBestIdxV, lNumU, lNumV, pCPts, bRational );

  // Use the Greville abscissae for the control point indices.
  sUVGuess.x = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_U, rlBestIdxU );
  sUVGuess.y = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_V, rlBestIdxV );

  // Refine according to dBestLocalU and dBestLocalV.
  // Note, this trick doesn't help in this case:
  if ( m_eSolverOp != SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
    {
      if ( dBestLocalU > SM_EFF_ZERO_SQRT  &&  rlBestIdxU < lNumU-1 )
        {
          double dNextGreville = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_U, rlBestIdxU+1 );
          sUVGuess.x = sUVGuess.x + dBestLocalU * ( dNextGreville - sUVGuess.x );
        }
      if ( dBestLocalV > SM_EFF_ZERO_SQRT  &&  rlBestIdxV < lNumV-1 )
        {
          double dNextGreville = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_V, rlBestIdxV+1 );
          sUVGuess.y = sUVGuess.y + dBestLocalV * ( dNextGreville - sUVGuess.y );
        }
    }

  // set output
  sUVGuess = m_sDomain.ClampPoint2d( sUVGuess );

  // all done
  return SM_SUCCESS;

} // end SmSurfPtGlobalSolver::FindPolygonTriangleGuess.

/*******************************************************************//**
PURPOSE: Given a closest control point, find the closest approach to
   one of the four polygon legs of that control point.

NOTES:
 - This routine works from the B-Spline control points.
   If no SmBSplineSurface is available, it will return SM_ERR.
   In that case, the caller must do something different (such as gridding).

 - This routine makes that assumtion that the control points are stored
   as 4-tuples of doubles, with v- varying faster.

***********************************************************************/
SmStatus SmSurfPtGlobalSolver::RefineGuessPolygonLeg(
    SmPoint2d & rUVGuess, // in/out
    ULONG lBestIdxU,      // in
    ULONG lBestIdxV )     // in
{
  // Method: drop to the legs of the control polygon
  // surrounding the indicated control point.

  // This requires an SmBSplineSurface.
  if ( m_pBSplSrf == NULL )
    { return SM_ERR; }

  // Also, this won't help in this case:
  if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
    { return SM_ERR; }

  ULONG lNumU, lNumV;
  double *pCPts = NULL;
  SmStatus eStat = m_pBSplSrf->GetControlPointsPointer( lNumU, lNumV, pCPts );
  if ( eStat != SM_SUCCESS )
      { return SM_ERR; }

  SmBoolean bRational = m_pBSplSrf->IsRational();

  double *pPtPtr = pCPts + 4 * ( lBestIdxU * lNumV + lBestIdxV );
  SmPoint3d sBestPt( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
  if ( bRational )
      { sBestPt /= pPtPtr[3]; }

  SmPoint3d sThisPt;

#if SM_DEBUG_CODE
static ULONG bDebugLevel = 0;
  if ( bDebugLevel > 0 )
  {
      smgfx_Erase();
      smgfx_SetLook(1,6, 1,0,0) ; m_sTestPt.Draw() ; sm_GraphicsLoop();
      smgfx_SetLook(1,4, 0,0,1) ;  sBestPt.Draw() ; sm_GraphicsLoop();
      SmPoint2d sSrfUV;
      const SmContext *pContext = m_pSrf->GetContext();

      if ( lBestIdxU > 0 ) {
          pPtPtr = pCPts + 4 * ( (lBestIdxU-1) * lNumV + lBestIdxV );
          sThisPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
          if ( bRational ) { sThisPt /= pPtPtr[3]; }
          smgfx_SetLook(1,4, 1,0,1) ;  sThisPt.DrawPointToPoint(sBestPt, pContext) ; sm_GraphicsLoop();
          if ( bDebugLevel > 1 ) {
              sSrfUV.x = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_U, lBestIdxU-1 );
              sSrfUV.y = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_V, lBestIdxV );
              m_pBSplSrf->DrawAt( sSrfUV, 0 ); sm_GraphicsLoop();
          }
      }
      if ( lBestIdxU < lNumU-1 ) {
          pPtPtr = pCPts + 4 * ( (lBestIdxU+1) * lNumV + lBestIdxV );
          sThisPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
          if ( bRational ) { sThisPt /= pPtPtr[3]; }
          smgfx_SetLook(1,4, 0,1,1) ;  sThisPt.DrawPointToPoint(sBestPt, pContext) ; sm_GraphicsLoop();
          if ( bDebugLevel > 1 ) {
              sSrfUV.x = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_U, lBestIdxU+1 );
              sSrfUV.y = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_V, lBestIdxV );
              m_pBSplSrf->DrawAt( sSrfUV, 0 ); sm_GraphicsLoop();
          }
      }
      if ( lBestIdxV > 0 ) {
          pPtPtr = pCPts + 4 * ( lBestIdxU * lNumV + lBestIdxV-1 );
          sThisPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
          if ( bRational ) { sThisPt /= pPtPtr[3]; }
          smgfx_SetLook(1,4, 1,1,0) ;  sThisPt.DrawPointToPoint(sBestPt, pContext) ; sm_GraphicsLoop();
          if ( bDebugLevel > 1 ) {
              sSrfUV.x = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_U, lBestIdxU );
              sSrfUV.y = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_V, lBestIdxV-1 );
              m_pBSplSrf->DrawAt( sSrfUV, 0 ); sm_GraphicsLoop();
          }
      }
      if ( lBestIdxV < lNumV-1 ) {
          pPtPtr = pCPts + 4 * ( lBestIdxU * lNumV + lBestIdxV+1 );
          sThisPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
          if ( bRational ) { sThisPt /= pPtPtr[3]; }
          smgfx_SetLook(1,4, 0,1,0) ;  sThisPt.DrawPointToPoint(sBestPt, pContext) ; sm_GraphicsLoop();
          if ( bDebugLevel > 1 ) {
              sSrfUV.x = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_U, lBestIdxU );
              sSrfUV.y = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_V, lBestIdxV+1 );
              m_pBSplSrf->DrawAt( sSrfUV, 0 ); sm_GraphicsLoop();
          }
      }
      sm_GraphicsLoop();
  }
#endif

  rUVGuess.x = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_U, lBestIdxU );
  rUVGuess.y = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_V, lBestIdxV );

#if SM_DEBUG_CODE
  if ( bDebugLevel > 0 )
  {
      m_pSrf->DrawAt( rUVGuess, 1 ); sm_GraphicsLoop();
  }
#endif

  double dDownShift = 0, dUpShift = 0;

  // Drop to the two legs running in the u-direction.
  // See whether it drops within the leg.
  // Note, it should never drop more than half-way to the
  // next control point, because then it would be closer
  // to the next control point, which should never happen.
  // (But we allow for numerical noise in the assert.)

  if ( lBestIdxU > 0 )
  {
      pPtPtr = pCPts + 4 * ( (lBestIdxU-1) * lNumV + lBestIdxV );
      sThisPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
      if ( bRational )
          { sThisPt /= pPtPtr[3]; }

      smgu_LineClosestPoint( sBestPt, (sThisPt-sBestPt), m_sTestPt, dDownShift );
      SM_ASSERT( dDownShift <= 0.5+SM_EFF_ZERO ); // otherwise, sThisPt was closer

      if ( dDownShift < 0 )
          dDownShift = 0;
      else if ( dDownShift > 0.5 )
          dDownShift = 0.5;
  }
  if ( lBestIdxU < lNumU - 1 )
  {
      pPtPtr = pCPts + 4 * ( (lBestIdxU+1) * lNumV + lBestIdxV );
      sThisPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
      if ( bRational )
          { sThisPt /= pPtPtr[3]; }

      smgu_LineClosestPoint( sBestPt, (sThisPt-sBestPt), m_sTestPt, dUpShift );
      SM_ASSERT( dUpShift <= 0.5+SM_EFF_ZERO ); // otherwise, sThisPt was closer

      if ( dUpShift < 0 )
          dUpShift = 0;
      else if ( dUpShift > 0.5 )
          dUpShift = 0.5;
  }

  // If it dropped interior to either or both, shift the parameter.
  // Linearly interpolate on the Greville abscissae.
  double dOtherPrm;

  double dShift = dUpShift - dDownShift;
  if ( smos_Fabs( dShift ) > SM_EFF_ZERO_SQRT )
  {
      if ( dShift > 0 )
      {
          dOtherPrm = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_U, lBestIdxU+1 );
      }
      else
      {
          dOtherPrm = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_U, lBestIdxU-1 );
          dShift = -dShift;  // make it positive.
      }

      // Linearly interpolate
      rUVGuess.x = (1-dShift) * rUVGuess.x + dShift * dOtherPrm;
  }


  // Now repeat the whole thing for the v-direction.

  dDownShift = dUpShift = 0;

  if ( lBestIdxV > 0 )
  {
      pPtPtr = pCPts + 4 * ( lBestIdxU * lNumV + lBestIdxV-1 );
      sThisPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
      if ( bRational )
          { sThisPt /= pPtPtr[3]; }

      smgu_LineClosestPoint( sBestPt, (sThisPt-sBestPt), m_sTestPt, dDownShift );
      SM_ASSERT( dDownShift <= 0.5+SM_EFF_ZERO ); // otherwise, sThisPt was closer

      if ( dDownShift < 0 )
          dDownShift = 0;
      else if ( dDownShift > 0.5 )
          dDownShift = 0.5;
  }
  if ( lBestIdxV < lNumV - 1 )
  {
      pPtPtr = pCPts + 4 * ( lBestIdxU * lNumV + lBestIdxV+1 );
      sThisPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
      if ( bRational )
          { sThisPt /= pPtPtr[3]; }

      smgu_LineClosestPoint( sBestPt, (sThisPt-sBestPt), m_sTestPt, dUpShift );
      SM_ASSERT( dUpShift <= 0.5+SM_EFF_ZERO ); // otherwise, sThisPt was closer

      if ( dUpShift < 0 )
          dUpShift = 0;
      else if ( dUpShift > 0.5 )
          dUpShift = 0.5;
  }

  dShift = dUpShift - dDownShift;
  if ( smos_Fabs( dShift ) > SM_EFF_ZERO_SQRT )
  {
      if ( dShift > 0 )
      {
          dOtherPrm = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_V, lBestIdxV+1 );
      }
      else
      {
          dOtherPrm = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_V, lBestIdxV-1 );
          dShift = -dShift;  // make it positive.
      }

      // Linearly interpolate
      rUVGuess.y = (1-dShift) * rUVGuess.y + dShift * dOtherPrm;
  }

#if SM_DEBUG_CODE
  if ( bDebugLevel > 0 )
  {
      m_pSrf->DrawAt( rUVGuess, 1 );
      sm_GraphicsLoop();
  }
#endif

  return SM_SUCCESS;

} // end SmSurfPtGlobalSolver::RefineGuessPolygonLeg

/*******************************************************************//**
PURPOSE: Refine a guess uv, to get it to where Newton iteration should work.

NOTES: This does not use control points.
***********************************************************************/
SmStatus SmSurfPtGlobalSolver::RefineGuess
(SmPoint2d & rUVGuess,   // in : 
 ULONG     & lBestIdxU,  // NotUsed: out: 
 ULONG     & lBestIdxV)  // NotUsed: out: 
{
  SM_REF2(lBestIdxU, lBestIdxV) ;
    // Note: don't use a Newton step here, we're presumably not close enough.
    // Try a couple of plane steps.
    CheckGuessForSeams( rUVGuess ); // Since we do this on subsequent guesses...

    SmPoint2d sThisUV( rUVGuess );
    SmPoint2d sNextUV( rUVGuess );
    double dThisFVal;
    double dNextFVal = SM_BIG_DOUBLE;

    double dUExtent = m_sDomain.GetUInterval().GetLength();
    double dVExtent = m_sDomain.GetVInterval().GetLength();

    // Don't ever take very big steps here: might not be close,
    // and C1 discontinuities (or just big changes) can
    // cause very big steps.
    double dMaxUShift = dUExtent * 0.10;
    double dMaxVShift = dVExtent * 0.10;

    // For directed operations, we can't get a proper plane step,
    // all we can get is a direction.  We have to guess a step size.
    // Start at 1/20 of the domain, and shrink each iteration.
    double dDomainSize    = m_sDomain.GetSize().Length();
    double dStepSizeGuess = 0.05 * dDomainSize;

    SmPoint3d sPt;
    SmVector3d sSu, sSv;

    int iter;
    for ( iter = 0; iter < 3; iter++ )
    {
        dThisFVal = dNextFVal;

        dNextFVal = CalcFuncVal( sNextUV );

        if ( smos_Fabs( dNextFVal ) < SM_EFF_ZERO_SQRT )
        {
            rUVGuess = sNextUV;
            return SM_SUCCESS;  // very close, definitely good enough guess.
        }

        if ( dNextFVal >= dThisFVal )
        {
            // if no movement, then we're close enough.
            if (   smos_Fabs( sNextUV.x - sThisUV.x ) < dUExtent / 1000
                && smos_Fabs( sNextUV.y - sThisUV.y ) < dVExtent / 1000 )
            {
                break;
            }

            if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
            {
                sNextUV = ( sThisUV + sNextUV ) / 2;
                continue;
            }
            else
              { break; }    // have to try something different.
        }


        // Take a plane step.  Set these two:
        double dUShift, dVShift;

        SER( m_pSrf->Evaluate1stDerivatives( sNextUV, FALSE, FALSE, sPt, sSu, sSv ));

        if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
        {
            // Plane step doesn't work here.  All we can get is a direction.
            SmVector2d sUVStep;
            smgu_VecLinCombTwoVectors( sSu, sSv, -m_sDirVec, sUVStep );
            double dMag = sUVStep.Length();
            if ( dMag < SM_EFF_ZERO_SQRT )
              { break; }

            sUVStep *= dStepSizeGuess / dMag;
            dStepSizeGuess /= 2;  // shrink on each iteration.

            dUShift = sUVStep.x;
            dVShift = sUVStep.y;
        }
        else
        {
            // 'Point' cases.
            SmVector3d sGap( sPt - m_sTestPt );

            double dF = sGap.Dot( sSu );  // These are the right-hand side (b-vector)
            double dG = sGap.Dot( sSv );

            // Solve 2x2 to get step
            double dFu = sSu.Dot( sSu );
            double dFv = sSu.Dot( sSv );
            double dGu = dFv;
            double dGv = sSv.Dot( sSv );

            double dDet =    dFu * dGv - dFv * dGu;
            dUShift     = -( dF  * dGv - dFv * dG  );  // numerators
            dVShift     = -( dFu * dG  - dF  * dGu );  // to be divided by dDet.

            // For our purposes, if singular, just do what we can.
            // Our job is just to try to improve the guess.
            if( SM_IS_ZERO(dDet) || SM_IS_ZERO(dUShift) )
            {
                dUShift = 0;
            }
            else if ( smos_Fabs(dDet) > smos_Fabs(dUShift) * SM_EFF_ZERO_SQRT )
            {
                dUShift /= dDet;
            }
            else
            {
                dUShift = 0;
            }


            if( SM_IS_ZERO(dDet) || SM_IS_ZERO(dVShift) )
            {
                dVShift = 0;
            }
            else if ( smos_Fabs(dDet) > smos_Fabs(dVShift) * SM_EFF_ZERO_SQRT )
            {
                dVShift /= dDet;
            }
            else
            {
                dVShift = 0;
            }

            // if no movement, then we're close enough.
            if (   smos_Fabs( dUShift ) < dUExtent / 1000
                && smos_Fabs( dVShift ) < dVExtent / 1000 )
            {
                rUVGuess = sNextUV;
                return SM_SUCCESS;
            }

            // Don't take big steps.
            if ( smos_Fabs( dUShift ) > dMaxUShift ) {
                dUShift = ( dUShift > 0 ) ? dMaxUShift : -dMaxUShift;
            }
            if ( smos_Fabs( dVShift ) > dMaxVShift ) {
                dVShift = ( dVShift > 0 ) ? dMaxVShift : -dMaxVShift;
            }

        } // end else, not directed operation.

        // State: dUShift and dVShift are set.

        // Set up for next iter.
        sThisUV = sNextUV;

        sNextUV.Set( sThisUV.x + dUShift, sThisUV.y + dVShift );

        sNextUV = m_sDomain.ClampPoint2d( sNextUV );

        CheckGuessForSeams( sNextUV );

    } // end iteration on plane steps.

    // We didn't return with success during the loop.

    dThisFVal = dNextFVal;
    dNextFVal = CalcFuncVal( sNextUV );

    if ( dNextFVal < dThisFVal )
    {
        rUVGuess = sNextUV;
    }
    else
    {
        rUVGuess = sThisUV;
    }
    return SM_SUCCESS;

} // end SmSurfPtGlobalSolver::RefineGuess


/*******************************************************************//**
PURPOSE: Given a grid point, guard against taking too big a stride.
  (See comments in code.)

NOTES:
  This is used to test for a grid step being too large.
  An example is a turbine blade, where cross sections are closed loops,
  and the surface is thin in the middle.  If the test point is in the
  middle of its correct closest span, a grid point on the the other side
  of the surface could be closer to the test point then any of the
  correct grid points.

  Former PURPOSE: Given two grid points, test whether there might be
  a surface point in between them that's very much better than either of
  the two given points.
***********************************************************************/
SmBoolean SmSurfPtGlobalSolver::CheckBadGuessStride(
              SmSurfaceEval & crGridPt0,   // in
           /* SmSurfaceEval & crGridPt1,   // ( obsolete )  */
              SmSurfaceEval & rNewGridPt,  // out, if return is True
              double        & rdNewFuncVal // out, if return is True
    )
{
  // The problem is that surfaces can double back on themselves
  // (picture the thin part of a turbine blade), and a sample point
  // on the other side of the surface can be geometrically closer
  // to the given point than a point on the same side.  The point
  // on the other side will converge on the other side of the shape.
  //
  // The first approach was to look at the triangle of the two grid
  // points and the test point.  If it's a long flat triangle with
  // the test point close to the line between the two grid points,
  // then it's suspicious, so try a point interpolated along that
  // line segment.  (Originally a line search.)  But counterexamples
  // showed that this can miss certain cases.
  //
  // The next version was:
  //  If the surface normals point generally in the same direction,
  //  then we drop the test point to the line connecting the two guess
  //  points.  If it drops outside the segment, then we're good.
  //  If it drops inside, then we interpolate the two guess points
  //  according to the line-point drop and return that.  This case
  //  would normally be ok anyway, but since we've done the work
  //  anyway, this will usually give a much better guess.
  //
  //  If the normals are in opposite directions, then the stride has
  //  turned the corner, and the points are on opposite sides of the shape.
  //  In this case, we drop the test point to each tangent plane, and
  //  take a plane-approximate uv step towards it -- that's a plane step
  //  from each.  The one on the correct side will be much closer to
  //  the test point.
  //
  // This works, but the same can be accomplished more simply: just take
  // a plane step once from each grid point.  This accomplishes the tasks
  // described here, but with fewer evaluations and a simpler algorithm.

  SmVector2d sUVStep;
  SmVector3d sGap = m_sTestPt - crGridPt0.Pos();
  SmStatus eStat = smgu_VecLinCombTwoVectors( crGridPt0.Su(), crGridPt0.Sv(), sGap, sUVStep );
  if ( eStat != SM_SUCCESS )
    { return FALSE; }  // Just forget this check.

  SmPoint2d sNewUV0 = crGridPt0.GetUV() + sUVStep;
  sNewUV0 = m_sDomain.ClampPoint2d( sNewUV0 ); // Clamp to whole domain: ok if outside 'subdomain'.
  rNewGridPt.SetUV( sNewUV0 );
  rdNewFuncVal = CalcFuncVal( sNewUV0 );

  // If the step was big, take another one.  [B51]
  // Since it's only a plane step, the limit should be pretty small.
  double dMaxStepSq = 0.10 * 0.10; // 10%, squared.

  if ( sUVStep.LengthSquared() > dMaxStepSq * m_sDomain.GetSize().LengthSquared() )
  {
      // watch out for failing Evaluations
      if(rNewGridPt.FailsEval(FALSE))  // FALSE = expect possible eval failures so don't report them in debug mode
        { return TRUE ;} // Just use the first result.

      sGap = m_sTestPt - rNewGridPt.Pos();
      eStat = smgu_VecLinCombTwoVectors( rNewGridPt.Su(), rNewGridPt.Sv(), sGap, sUVStep );
      if ( eStat != SM_SUCCESS )
        { return TRUE; }  // Just use the first result.

      sNewUV0 = rNewGridPt.GetUV() + sUVStep;
      sNewUV0 = m_sDomain.ClampPoint2d( sNewUV0 ); // ok if outside 'subdomain'.
      double dTempFuncVal = CalcFuncVal( sNewUV0 );
      if ( dTempFuncVal < rdNewFuncVal )
      {

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
if ( bDebugMe ) {
          smgfx_SetLook( 1,3, 0,1,0 ); rNewGridPt.Pos().DrawPointToPoint( m_sTestPt ); sm_GraphicsLoop();
          sm_GraphicsLoop();
}
#endif
          rNewGridPt.SetUV( sNewUV0 );
          rdNewFuncVal = dTempFuncVal;
      }
  }

  return TRUE;

} // end CheckBadGuessStride

/*******************************************************************//**
PURPOSE: Grid up a (sub)domain of a surface to find the best
    point to a given 3d point.

NOTES: Sometimes a coarse grid can find a point that's better
  than one found by a finer grid, just by luck.  So the argument
  rdBestFuncVal records the best of all sequential calls to this
  routine: if this call doesn't find anything as good as a previous
  call, keep the previous best guess and continue from there.  [B51]
***********************************************************************/
SmStatus SmSurfPtGlobalSolver::RefinePatchByGrid
 (const SmExtent2d & crSubDomain,   // in :
  int                iNumU,         // in : grid size
  int                iNumV,         // in : grid size
  SmPoint2d        & rUVGuess,      // out:
  double           & rdBestFuncVal) // i/o: if nothing better found, return rUVGuess unchanged.
{
  int ii, jj;
  SmPoint2d sThisUV, sBestUV;
  double dThisVal, dTempVal;

  SmSurfaceEval sThisGridPt ( m_pSrf );
  SmSurfaceEval sTempSrfPt  ( m_pSrf );

  if ( iNumU < 2 ) iNumU = 2;
  if ( iNumV < 2 ) iNumV = 2;

  for ( ii = 0; ii < iNumU; ii++ )
  {
      double dFracU = (double)ii / (double)(iNumU-1);

      for ( jj = 0; jj < iNumV; jj++ )
      {
          double dFracV = (double)jj / (double)(iNumV-1);

          sThisUV = crSubDomain.Evaluate( dFracU, dFracV );

          sThisGridPt.SetUV( sThisUV );

          dThisVal = CalcFuncVal( sThisUV );

#ifdef SM_DEBUG_CODE
SmPoint3d sSrfPt;
SmBoolean bDebugMe1 = FALSE;
          if ( bDebugMe1 )
             {
              if ( FALSE )
                {
                  smgfx_Erase();
                  smgfx_SetLook( 1,2, 0,1,1 ); m_pSrf->DrawUV(4,4); sm_GraphicsLoop();
                }
              smgfx_SetLook( 1,3, 0,0,1 ); sThisGridPt.Pos().DrawPointToPoint( m_sTestPt ); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          sBestUV = sThisUV; // Best of: this uv, or one of the refined ones.

          // Check for too-big strides.  [B46, B51]
          if ( CheckBadGuessStride( sThisGridPt, sTempSrfPt, dTempVal ) )
          {

#ifdef SM_DEBUG_CODE
              if ( bDebugMe1 )
                {
                  smgfx_SetLook( 2,4, 1,0,0 ); sTempSrfPt.Pos().DrawPointToPoint( m_sTestPt ); sm_GraphicsLoop();
                  sm_GraphicsLoop();
              }
#endif // SM_DEBUG_CODE

              if ( dTempVal < dThisVal )
              {
                  dThisVal = dTempVal;
                  sBestUV  = sTempSrfPt.GetUV();
              }
          }

          if ( dThisVal < rdBestFuncVal )
          {
              rUVGuess = sBestUV;
              rdBestFuncVal = dThisVal;

#ifdef SM_DEBUG_CODE
              SmBoolean bDebugMe2 = FALSE;
              if ( bDebugMe2 )
                {
                  m_pSrf->EvaluatePoint( sBestUV, sSrfPt );
                  smgfx_SetLook( 2,5, 1,0,0 ); sSrfPt.DrawPointToPoint( m_sTestPt ); sm_GraphicsLoop();
                  sm_GraphicsLoop();
              }
#endif
          }
      } // end inner loop on jj.
  } // end outer loop on ii.

  return SM_SUCCESS;

} // end SmSurfPtGlobalSolver::RefinePatchByGrid

/*******************************************************************//**
PURPOSE: Try a third refinement of a uv guess.

NOTES:
  This is called if LocalSolve failed with the original refined guess.
  The first refinement used tangent-plane iterations.  That won't work
  if there are waves in the surface between the guess and the solution.
  So here we'll be more brute-force, looking only at goodness values at
  individual points, not moving according to derivative information.
***********************************************************************/
SmStatus SmSurfPtGlobalSolver::RefineGuessByGrid(
    SmPoint2d & rUVGuess,
    ULONG & lBestIdxU,  // in: used only if m_pBSplSrf != NULL.
    ULONG & lBestIdxV)
{
  // Method: block out an area of the surface that surrounds the
  // given control point.  Then grid that area up and find the best point.

  // Sometimes the original guess is better than any gridded value.
  // In that case, return the original.
  SmPoint2d sBestUVGuess( rUVGuess );
  double dBestVal = CalcFuncVal( sBestUVGuess );

  // Do the gridding.
  // Find a subdomain around rUVGuess.
  SmPoint2d sMinUV, sMaxUV;

  if ( m_pBSplSrf != NULL )
  {
      ULONG lNumU = m_pBSplSrf->GetNumberControlPoints( SM_SP_U );
      ULONG lNumV = m_pBSplSrf->GetNumberControlPoints( SM_SP_V );

      ULONG lLoU = ( lBestIdxU > 0 ) ? lBestIdxU-1 : 0;
      ULONG lHiU = ( lBestIdxU < lNumU-1 ) ? lBestIdxU+1 : lNumU-1;
      ULONG lLoV = ( lBestIdxV > 0 ) ? lBestIdxV-1 : 0;
      ULONG lHiV = ( lBestIdxV < lNumV-1 ) ? lBestIdxV+1 : lNumV-1;

      sMinUV.x = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_U, lLoU );
      sMaxUV.x = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_U, lHiU );
      sMinUV.y = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_V, lLoV );
      sMaxUV.y = m_pBSplSrf->GetGrevilleAbscissa( SM_SP_V, lHiV );
  }
  else
  {
      // No B-Spline surface.
      // Use 1/4 of the domain.
      SmVector2d sDomainSize = m_sDomain.GetSize();
      sMinUV = rUVGuess - sDomainSize / 4;
      sMaxUV = rUVGuess + sDomainSize / 4;
  }

  sMinUV = m_sDomain.ClampPoint2d( sMinUV );
  sMaxUV = m_sDomain.ClampPoint2d( sMaxUV );

  SmExtent2d sSubDomain( sMinUV, sMaxUV );

  int iGridSize = 6;

  double dThisVal = SM_BIG_DOUBLE;

  RefinePatchByGrid( sSubDomain, iGridSize, iGridSize, rUVGuess, dThisVal );

  // Check whether it improved.
  if ( dThisVal < dBestVal )
  {
      dBestVal = dThisVal;
      sBestUVGuess = rUVGuess;
  }

  // Smaller grid:
  double dDeltaU = sMaxUV.x - sMinUV.x;
  double dDeltaV = sMaxUV.y - sMinUV.y;

  sMinUV.x = rUVGuess.x - dDeltaU / ( iGridSize-1 );
  sMaxUV.x = rUVGuess.x + dDeltaU / ( iGridSize-1 );
  sMinUV.y = rUVGuess.y - dDeltaV / ( iGridSize-1 );
  sMaxUV.y = rUVGuess.y + dDeltaV / ( iGridSize-1 );

  sMinUV = m_sDomain.ClampPoint2d( sMinUV );
  sMaxUV = m_sDomain.ClampPoint2d( sMaxUV );

  sSubDomain.SetMinMax( sMinUV, sMaxUV );

  RefinePatchByGrid( sSubDomain, iGridSize, iGridSize, rUVGuess, dThisVal );

  // Check whether it improved.
  if ( dThisVal < dBestVal )
  {
      dBestVal = dThisVal;
      sBestUVGuess = rUVGuess;
  }

  // One more refinement
  dDeltaU = sMaxUV.x - sMinUV.x;
  dDeltaV = sMaxUV.y - sMinUV.y;

  sMinUV.x = rUVGuess.x - dDeltaU / ( iGridSize-1 );
  sMaxUV.x = rUVGuess.x + dDeltaU / ( iGridSize-1 );
  sMinUV.y = rUVGuess.y - dDeltaV / ( iGridSize-1 );
  sMaxUV.y = rUVGuess.y + dDeltaV / ( iGridSize-1 );

  sMinUV = m_sDomain.ClampPoint2d( sMinUV );
  sMaxUV = m_sDomain.ClampPoint2d( sMaxUV );

  sSubDomain.SetMinMax( sMinUV, sMaxUV );

  RefinePatchByGrid( sSubDomain, iGridSize, iGridSize, rUVGuess, dThisVal );

  // Check whether it improved.
  if ( dThisVal > dBestVal )
  {
      rUVGuess = sBestUVGuess;
  }

  return SM_SUCCESS;
} // end SmSurfPtGlobalSolver::RefineGuessByGrid


/*******************************************************************//**
PURPOSE: If a guess uv is on a seam, decide which end of the domain
    (low or high) is better.

NOTES:
 - Might flip rUVGuess across the seam.
 - This version works from a uv point on the surface.
***********************************************************************/
SmStatus SmSurfPtGlobalSolver::CheckGuessForSeams( SmPoint2d & rUVGuess )
{
  if ( ! m_pSrf->IsOnSeam( rUVGuess ) ) {
      return SM_SUCCESS;
  }

  SmPoint3d  sPt;
  SmVector3d sSu, sSv, sGap;

  // Check u-direction.
  if (    rUVGuess.x < m_sDomain.GetMin().x + SM_EFF_ZERO
       || rUVGuess.x > m_sDomain.GetMax().x - SM_EFF_ZERO
     )
  {
      if ( ClosedSurface( SM_SP_U ) )
      {
          // Check whether low or high bound is better.
          m_pSrf->Evaluate1stDerivatives( rUVGuess,
              FALSE, FALSE, sPt, sSu, sSv );

          sGap = sPt - m_sTestPt;
          if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
          {
              double dDot = sGap.Dot( m_sDirVec );
              sGap = dDot * m_sDirVec;
          }

          rUVGuess.x = ( sGap.Dot( sSu ) > 0 )
              ? m_sDomain.GetMax().x
              : m_sDomain.GetMin().x;
      }
  }

  // Check v-direction.
  if (    rUVGuess.y < m_sDomain.GetMin().y + SM_EFF_ZERO
       || rUVGuess.y > m_sDomain.GetMax().y - SM_EFF_ZERO
     )
  {
      if ( ClosedSurface( SM_SP_V ) )
      {
          // Check whether low or high bound is better.
          m_pSrf->Evaluate1stDerivatives( rUVGuess,
              FALSE, FALSE, sPt, sSu, sSv );

          sGap = sPt - m_sTestPt;
          if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
          {
              double dDot = sGap.Dot( m_sDirVec );
              sGap = dDot * m_sDirVec;
          }

          rUVGuess.y = ( sGap.Dot( sSv ) > 0 )
              ? m_sDomain.GetMax().y
              : m_sDomain.GetMin().y;
      }
  }
  return SM_SUCCESS;

} // end SmSurfPtGlobalSolver::CheckGuessForSeams

/*******************************************************************//**
PURPOSE: If a guess uv is on a seam, decide which end of the domain
    (low or high) is better.

NOTES:
 - Might flip rUVGuess across the seam.
 - This version works from control points.
***********************************************************************/
SmStatus SmSurfPtGlobalSolver::CheckGuessForSeams(
        ULONG & rlIdxU, ULONG & rlIdxV, // in/out: indices of the control point
        ULONG   lNumU,  ULONG   lNumV,  // in: control point counts
        double * pCPts,                 // in: each point is [4] doubles
        SmBoolean bRational             // in:
    )
{
  if ( rlIdxU > 0 && rlIdxU < lNumU-1 && rlIdxV > 0 && rlIdxV < lNumV-1 )
  {
      return SM_SUCCESS;  // not on boundary.
  }

  SmPoint3d sSeamPt, sOtherPt;
  double *pPtPtr = NULL;

  // Check u-direction.
  if ( lNumU > 2 && ( rlIdxU == 0 || rlIdxU == lNumU-1 ) )
  {
      // Check whether it's a seam.
      ULONG lIdx = 0;
      pPtPtr = pCPts + 4 * ( lIdx * lNumV + rlIdxV );
      sSeamPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
      if ( bRational ) { sSeamPt /= pPtPtr[3]; }

      lIdx = lNumU-1;
      pPtPtr = pCPts + 4 * ( lIdx * lNumV + rlIdxV );
      sOtherPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
      if ( bRational ) { sOtherPt /= pPtPtr[3]; }

      if ( sSeamPt.CloserThan( SM_EFF_ZERO, sOtherPt ) )
      {
          double dDownShift, dUpShift;

          lIdx = lNumU-2;
          pPtPtr = pCPts + 4 * ( lIdx * lNumV + rlIdxV );
          sOtherPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
          if ( bRational ) { sOtherPt /= pPtPtr[3]; }

          if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
            { dDownShift = - m_sDirVec.Dot( (sOtherPt-sSeamPt) ); }
          else
            { smgu_LineClosestPoint( sSeamPt, (sOtherPt-sSeamPt), m_sTestPt, dDownShift ); }

          lIdx = 1;
          pPtPtr = pCPts + 4 * ( lIdx * lNumV + rlIdxV );
          sOtherPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
          if ( bRational ) { sOtherPt /= pPtPtr[3]; }

          if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
            { dUpShift = - m_sDirVec.Dot( (sOtherPt-sSeamPt) ); }
          else
            { smgu_LineClosestPoint( sSeamPt, (sOtherPt-sSeamPt), m_sTestPt, dUpShift ); }

          // Ideally, these would have opposite signs (or both zero).
          // Lean towards the one that's positive.
          double dDiff = dUpShift - dDownShift;

          rlIdxU = ( dDiff >= 0 ) ? 0 : lNumU-1;

      } // end on-U-seam case.
  } // end checking u-direction.

  // Repeat for v-direction.
  if ( lNumV > 2 && ( rlIdxV == 0 || rlIdxV == lNumV-1 ) )
  {
      // Check whether it's a seam.
      ULONG lIdx = 0;
      pPtPtr = pCPts + 4 * ( rlIdxU * lNumV + lIdx );
      sSeamPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
      if ( bRational ) { sSeamPt /= pPtPtr[3]; }

      lIdx = lNumV-1;
      pPtPtr = pCPts + 4 * ( rlIdxU * lNumV + lIdx );
      sOtherPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
      if ( bRational ) { sOtherPt /= pPtPtr[3]; }

      if ( sSeamPt.CloserThan( SM_EFF_ZERO, sOtherPt ) )
      {
          double dDownShift, dUpShift;

          lIdx = lNumV-2;
          pPtPtr = pCPts + 4 * ( rlIdxU * lNumV + lIdx );
          sOtherPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
          if ( bRational ) { sOtherPt /= pPtPtr[3]; }

          if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
            { dDownShift = - m_sDirVec.Dot( (sOtherPt-sSeamPt) ); }
          else
            { smgu_LineClosestPoint( sSeamPt, (sOtherPt-sSeamPt), m_sTestPt, dDownShift ); }

          lIdx = 1;
          pPtPtr = pCPts + 4 * ( rlIdxU * lNumV + lIdx );
          sOtherPt.Set( pPtPtr[0], pPtPtr[1], pPtPtr[2] );
          if ( bRational ) { sOtherPt /= pPtPtr[3]; }

          if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
            { dUpShift = - m_sDirVec.Dot( (sOtherPt-sSeamPt) ); }
          else
            { smgu_LineClosestPoint( sSeamPt, (sOtherPt-sSeamPt), m_sTestPt, dUpShift ); }

          // Ideally, these would have opposite signs (or both zero).
          // Lean towards the one that's positive.
          double dDiff = dUpShift - dDownShift;

          rlIdxV = ( dDiff >= 0 ) ? 0 : lNumV-1;

      } // end on-V-seam case.
  } // end checking v-direction.

  return SM_SUCCESS;

} // end SmSurfPtGlobalSolver::CheckGuessForSeams

/*******************************************************************//**
PURPOSE: Break up a uv domain along creases.

NOTES:  (Currently not used.)
   This requires the B-Spline surface.  Returns the whole domain if not given.
***********************************************************************/
SmStatus SmSurfPtGlobalSolver::GetCreaseSubdomains( SmTArray< SmExtent2d > & sDomainList )
{
  sDomainList.ReSet();
  sDomainList.Add( m_sDomain );

// Currently not used.
static constexpr SmBoolean sbDoCreaseSubdomains = FALSE;
if ( ! sbDoCreaseSubdomains )
  { return SM_SUCCESS; }

  // This requires a B-Spline surface.
  if ( m_pBSplSrf == NULL )
    { return SM_SUCCESS; }

//cbi TODO: don't just check multiple knots: all 'circles' have them.

  SmTArray< double > aKnots;
  SmTArray< ULONG  > aMults;
  ULONG lNumKnots, lDeg;
  ULONG ii, jj;

  // u-knots first.
  SmExtent1d sDomain = m_sDomain.GetUInterval();
  double dTol = sDomain.GetLength() / 1000;
  dTol = -dTol;  // on-boundary means Outside.
  m_pBSplSrf->GetKnots( SM_SP_U, aKnots, &aMults );
  lNumKnots = aMults.GetSize();
  lDeg = m_pBSplSrf->GetDegree( SM_SP_U );
  // For each interior knot:
  for ( ii = 1; ii+1 < lNumKnots; ii++ ) // note: can't say lNumKnots-1
  {
      if ( aMults[ii] >= lDeg ) // required for non-C1 jump.
      {
          if ( sDomain.ContainsValue( aKnots[ii], dTol ) )
          {
              // Split every subdomain that contains aKnots[i] into two pieces.
              // Add one piece to the end of the list, and replace
              // the original subdomain with the other piece.
              ULONG lListSize = sDomainList.GetSize();
              for ( jj = 0; jj < lListSize; jj++ )
              {
                  SmExtent1d sNewUDomain = sDomainList[jj].GetUInterval();
                  if ( !sNewUDomain.ContainsValue( aKnots[ii], dTol ) )
                      { continue; }
                  double dHighEnd = sNewUDomain.GetMax();
                  // Reset in place to one piece, copy that to the end,
                  // then reset to the other piece.
                  sNewUDomain.SetMinMax( sNewUDomain.GetMin(), aKnots[ii] );
                  sDomainList[jj].SetUInterval( sNewUDomain );

                  sDomainList.Add( sDomainList[jj] );
                  sNewUDomain.SetMinMax( aKnots[ii], dHighEnd );
                  sDomainList[jj].SetUInterval( sNewUDomain );
              }
          }
      }
  }

  // Now v-knots.
  sDomain = m_sDomain.GetVInterval();
  dTol = sDomain.GetLength() / 1000;
  dTol = -dTol;  // on-boundary means Outside.
  m_pBSplSrf->GetKnots( SM_SP_V, aKnots, &aMults );
  lNumKnots = aMults.GetSize();
  lDeg = m_pBSplSrf->GetDegree( SM_SP_V );
  // For each interior knot:
  for ( ii = 1; ii+1 < lNumKnots; ii++ ) // note: can't say lNumKnots-1
  {
      if ( aMults[ii] >= lDeg ) // required for non-C1 jump.
      {
          if ( sDomain.ContainsValue( aKnots[ii], dTol ) )
          {
              // Split every subdomain that contains aKnots[i] into two pieces.
              // Add one piece to the end of the list, and replace
              // the original subdomain with the other piece.
              ULONG lListSize = sDomainList.GetSize();
              for ( jj = 0; jj < lListSize; jj++ )
              {
                  SmExtent1d sNewVDomain = sDomainList[jj].GetVInterval();
                  if ( !sNewVDomain.ContainsValue( aKnots[ii], dTol ) )
                      { continue; }
                  double dHighEnd = sNewVDomain.GetMax();
                  // Reset in place to one piece, copy that to the end,
                  // then reset to the other piece.
                  sNewVDomain.SetMinMax( sNewVDomain.GetMin(), aKnots[ii] );
                  sDomainList[jj].SetVInterval( sNewVDomain );

                  sDomainList.Add( sDomainList[jj] );
                  sNewVDomain.SetMinMax( aKnots[ii], dHighEnd );
                  sDomainList[jj].SetVInterval( sNewVDomain );
              }
          }
      }
  }
  return SM_SUCCESS;

}  // end SmSurfPtGlobalSolver::GetCreaseSubdomains

/*******************************************************************//**
PURPOSE: If a solution is on a seam (or two), add solutions on the
   other side of the seam(s).

NOTES:
 - The global solver returns both solutions if on a seam.
 - Returns four solutions if on both seams (torus).
 - Multiple solutions are sorted low to high parameter values, u first, then v.
***********************************************************************/
SmStatus SmSurfPtGlobalSolver::CheckSeams
 (double             dTol,        // NotUsed: in :
  const SmSolution & crSol,       // out:
  SmSolutionArray  & rSolutions)  // out:
{
  SM_REF1(dTol) ;
  // Method: first check whether we're close to a boundary.
  // If so, then we test the point on the other side of the seam,
  // and add it in if it test ok.
  //
  // Note, don't just copy over the values, in case the given point
  // is not quite on the seam (but close enough that the cross-seam
  // point is also a solution). [081031]

  rSolutions.ReSet();
  rSolutions.Add( crSol );

  SmPoint2d sUV( crSol.m_vStart[0], crSol.m_vStart[1] );

  SmSurfParamType eWhichBound;

  // Check with a looser tol, because we will check whether the cross-seam
  // point is a solution, and reject it if not.
  // But not m_dLooseTol, it can be way too loose. (Besides, it's 3d, not 2d.)
  SmTol3d dDomainTol = 0.001 * m_sDomain.GetSize().Length();
  if ( ! m_pSrf->IsOnBoundary( m_sDomain, sUV, eWhichBound, &dDomainTol ) )
  {
      return SM_SUCCESS; // can't be on a seam.
  }

  SmSolution sOtherSol = crSol;


  // Make two steppers, to evaluate and compare.
  SmSurfPtSolveStepper sThisEval(
      m_pSrf, m_sDomain, m_sTestPt, sUV, m_dTightTol, m_dLooseTol, m_eSolverOp );
  SmSurfPtSolveStepper sOtherEval( sThisEval );

  // For comparison:
  SmSrfPtErrorNormType eNormType = SM_SPN_GAP;
  if ( m_eSolverOp == SM_SO_NORMALIZE )
  {
      eNormType = SM_SPN_SUM_ABS;
  }
  double dNearSeamU, dFarSeamU, dNearSeamV, dFarSeamV;


  // Do U direction.
  if ( ClosedSurface( SM_SP_U ) && ( eWhichBound == SM_SP_U || eWhichBound == SM_SP_BOTH ) )
  {
      SmExtent1d sDom = m_sDomain.GetUInterval();
      if ( sUV.x < sDom.GetMid() )
      {
          dNearSeamU = sDom.GetMin();
          dFarSeamU  = sDom.GetMax();
      }
      else
      {
          dNearSeamU = sDom.GetMax();
          dFarSeamU  = sDom.GetMin();
      }
      // If not quite on the seam, see whether right on is better.
      if ( smos_Fabs( sUV.x - dNearSeamU ) > m_dTightTol )
      {
          sOtherEval.SetUV( SmPoint2d( dNearSeamU, sUV.y ) );
          if ( sOtherEval.ErrorNorm( eNormType ) <= sThisEval.ErrorNorm( eNormType ) )
          {
              sUV.x =  rSolutions[0].m_vStart[0]      = dNearSeamU;
              rSolutions[0].m_vStart.m_dSolutionValue = sOtherEval.GapLength();
          }
      }

      // Now check across the seam: is it a solution too?
      sOtherEval.SetUV( SmPoint2d( dFarSeamU, sUV.y ) );
//    if ( sOtherEval.Converged( m_dLooseTol ) )
      if ( sOtherEval.ErrorNorm( eNormType ) < sThisEval.ErrorNorm( eNormType ) + SM_EFF_ZERO_SQRT )
      {
          sOtherSol.m_vStart[0] = dFarSeamU;
          sOtherSol.m_vStart[1] = sUV.y;
          sOtherSol.m_vStart.m_dSolutionValue = sOtherEval.GapLength();
          sOtherSol.m_lNumObjects    = 1 ;
          sOtherSol.m_apObjects[0] = (SmSurface *)m_pSrf ;
          rSolutions.AddSortedSolution( sOtherSol, SM_SK_BY_FIRST_TWO_PARAMETERS );
      }
  } // end u-direction seam check.

  // Now do V direction.
  if ( ClosedSurface( SM_SP_V ) && ( eWhichBound == SM_SP_V || eWhichBound == SM_SP_BOTH ) )
  {
      SmExtent1d sDom = m_sDomain.GetVInterval();
      if ( sUV.y < sDom.GetMid() )
      {
          dNearSeamV = sDom.GetMin();
          dFarSeamV  = sDom.GetMax();
      }
      else
      {
          dNearSeamV = sDom.GetMax();
          dFarSeamV  = sDom.GetMin();
      }
      // If not quite on the seam, see whether right on is better.
      if ( smos_Fabs( sUV.y - dNearSeamV ) > m_dTightTol )
      {
          sOtherEval.SetUV( SmPoint2d( sUV.x, dNearSeamV ) );
          if ( sOtherEval.ErrorNorm( eNormType ) <= sThisEval.ErrorNorm( eNormType ) )
          {
              sUV.y =  rSolutions[0].m_vStart[1]      = dNearSeamV;
              rSolutions[0].m_vStart.m_dSolutionValue = sOtherEval.GapLength();
          }
      }

      // Now check across the seam: is it a solution too?
      sOtherEval.SetUV( SmPoint2d( sUV.x, dFarSeamV ) );
//    if ( sOtherEval.Converged( m_dLooseTol ) )
      if ( sOtherEval.ErrorNorm( eNormType ) < sThisEval.ErrorNorm( eNormType ) + SM_EFF_ZERO_SQRT )
      {
          sOtherSol.m_vStart[0] = sUV.x;
          sOtherSol.m_vStart[1] = dFarSeamV;
          sOtherSol.m_vStart.m_dSolutionValue = sOtherEval.GapLength();
          sOtherSol.m_lNumObjects = 1 ;
          sOtherSol.m_apObjects[0] = (SmSurface*)SurfacePtr() ;
          rSolutions.AddSortedSolution( sOtherSol, SM_SK_BY_FIRST_TWO_PARAMETERS );
      }
  } // end v-direction seam check.

  // Finally, check both directions (torus).
  if ( ClosedSurface( SM_SP_U ) && ClosedSurface( SM_SP_V ) && eWhichBound == SM_SP_BOTH )
  {
      SmExtent1d sDom = m_sDomain.GetUInterval();
      dFarSeamU = ( sUV.x < sDom.GetMid() ) ? sDom.GetMax() : sDom.GetMin();
      sDom = m_sDomain.GetVInterval();
      dFarSeamV = ( sUV.y < sDom.GetMid() ) ? sDom.GetMax() : sDom.GetMin();

      // At this point, we've already checked almost-right-on.

      // Now check across both seams: is it a solution too?
      sOtherEval.SetUV( SmPoint2d( dFarSeamU, dFarSeamV ) );
//    if ( sOtherEval.Converged( m_dLooseTol ) )
      if ( sOtherEval.ErrorNorm( eNormType ) < sThisEval.ErrorNorm( eNormType ) + SM_EFF_ZERO_SQRT )
      {
          sOtherSol.m_vStart[0] = dFarSeamU;
          sOtherSol.m_vStart[1] = dFarSeamV;
          sOtherSol.m_vStart.m_dSolutionValue = sOtherEval.GapLength();
          rSolutions.AddSortedSolution( sOtherSol, SM_SK_BY_FIRST_TWO_PARAMETERS );
      }
  } // end both-directions seam check.

  return SM_SUCCESS;

} // end SmSurfPtGlobalSolver::CheckSeams

/*******************************************************************//**
PURPOSE: Check whether a uv position is a local minimum,
            with respect to a given test point.

NOTES:
 - Argument is SmSurfaceEval, to avoid re-evaluation of the surface.
***********************************************************************/
SmBoolean SmSurfPtGlobalSolver::IsLocalMin(
        SmSurfaceEval   & rSurfEval,    // in
        double dTol                     // in
    )
{
  SmVector3d sGap( rSurfEval.Pos() - m_sTestPt );
  if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
  {
      double dDot = sGap.Dot( m_sDirVec );
      sGap = dDot * m_sDirVec;
  }
  double dDist = sGap.Length();
  if ( dDist <= dTol ) { return TRUE; }

  // We are not right on the surface.
  // Notational, for readability:
  const SmVector3d & rSu = rSurfEval.Su();
  const SmVector3d & rSv = rSurfEval.Sv();

  if ( smos_Fabs( sGap.Dot( rSu ) ) > dTol ) { return FALSE; }
  if ( smos_Fabs( sGap.Dot( rSv ) ) > dTol ) { return FALSE; }

  // We are off the surface and on the surface normal.
  // It's a relative min if we're on the 'convex' side of the surface,
  // or if we're within the radius of curvature on the concave side.
  // An easy way to test this is: beyond the center of curvature,
  // a Newton step will try to increase the distance, but a plane step
  // will always decrease it; they go in opposite directions.  Of course
  // we can't actually take a step, because we're converged (on the normal;
  // the numerator would be zero), but: the difference between the Newton
  // and plane steps is the denominator.  For NR, it is
  // S' dot S'  +  Gap dot S'', and for a plane step, simply leave off the
  // second term.  A Newton step will go 'backwards' if the denominator
  // changes sign.  The first term is always positive, so we're at a relative
  // maximum if and only if the sum is negative.

  // What about right at the center of curvature?
  // Design decision: We'll say that's both a min and a max.

  // Do it separately for both parameters.  If either one indicates
  // a maximum, call it a maximum.  (Consider a cylinder: it will
  // always be a minimum in one direction, and if it's a maximum
  // in the other, then it's a maximum overall.)

  double dNoiseLim = SM_EFF_ZERO * 1000;

  const SmVector3d & rSuu = rSurfEval.Suu();
  const SmVector3d & rSvv = rSurfEval.Svv();

  SmVector3d sGapU( rSu );
  SmVector3d sGapV( rSv );
  if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
  {
      double dDot = sGapU.Dot( m_sDirVec );
      sGapU = dDot * m_sDirVec;
      dDot = sGapV.Dot( m_sDirVec );
      sGapV = dDot * m_sDirVec;
  }

  // If on a boundary, don't check in the direction of the boundary
  // that we're on; like being on the end of a curve [B243].
  ULONG lBdryFlags = this->Domain().GetPoint2dBoundaries( rSurfEval.GetUV() );

  if ( ! ( ( lBdryFlags & SM_SS_UMIN )  ||  ( lBdryFlags & SM_SS_UMAX ) ) )
  {
      double dF1 = sGapU.Dot( rSu  );
      double dF2 = sGap .Dot( rSuu );

      // If flat, don't return positive: make sure dF1 is not zero.
      if ( smos_Fabs(dF1) > dNoiseLim  &&  dF1 + dF2 < -dNoiseLim )
        { return TRUE; }
  }

  if ( ! ( ( lBdryFlags & SM_SS_VMIN )  ||  ( lBdryFlags & SM_SS_VMAX ) ) )
  {
      double dF1 = sGapV.Dot( rSv  );
      double dF2 = sGap .Dot( rSvv );

      if ( smos_Fabs(dF1) > dNoiseLim  &&  dF1 + dF2 < -dNoiseLim )
        { return TRUE; }
  }

  return FALSE;

} // end SmSurfPtGlobalSolver::IsLocalMin

/*******************************************************************//**
PURPOSE: Check whether a uv position is a local maximum,
            with respect to a given test point.

NOTES:
 - Argument is SmSurfaceEval, to avoid re-evaluation of surface.
***********************************************************************/
SmBoolean SmSurfPtGlobalSolver::IsLocalMax(
        SmSurfaceEval   & rSurfEval,    // in
        double dTol                     // in
    )
{
  SmVector3d sGap( rSurfEval.Pos() - m_sTestPt );
  if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
  {
      double dDot = sGap.Dot( m_sDirVec );
      sGap = dDot * m_sDirVec;
  }
  double dDist = sGap.Length();
  if ( dDist <= dTol ) { return FALSE; }

  // We are not right on the surface.
  // Notational, for readability:
  const SmVector3d & rSu  = rSurfEval.Su();
  const SmVector3d & rSv  = rSurfEval.Sv();

  if ( smos_Fabs( sGap.Dot( rSu ) ) > dTol ) { return FALSE; }
  if ( smos_Fabs( sGap.Dot( rSv ) ) > dTol ) { return FALSE; }

  // We are off the surface and on the surface normal.
  // See notes in IsLocalMin().

  double dNoiseLim = SM_EFF_ZERO * 1000;

  const SmVector3d & rSuu = rSurfEval.Suu();
  const SmVector3d & rSvv = rSurfEval.Svv();

  SmVector3d sGapU( rSu );
  SmVector3d sGapV( rSv );
  if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
  {
      double dDot = sGapU.Dot( m_sDirVec );
      sGapU = dDot * m_sDirVec;
      dDot = sGapV.Dot( m_sDirVec );
      sGapV = dDot * m_sDirVec;
  }

  // If on a boundary, don't check in the direction of the boundary
  // that we're on; like being on the end of a curve [B243].
  ULONG lBdryFlags = this->Domain().GetPoint2dBoundaries( rSurfEval.GetUV() );

  if ( ! ( ( lBdryFlags & SM_SS_UMIN )  ||  ( lBdryFlags & SM_SS_UMAX ) ) )
  {
      double dF1 = sGapU.Dot( rSu );
      double dF2 = sGap .Dot( rSuu );

      // If flat, don't return positive: make sure dF1 is not zero.
      if ( smos_Fabs(dF1) > dNoiseLim  &&  dF1 + dF2 < -dNoiseLim )
        { return TRUE; }
  }

  if ( ! ( ( lBdryFlags & SM_SS_VMIN )  ||  ( lBdryFlags & SM_SS_VMAX ) ) )
  {
      double dF1 = sGapV.Dot( rSv );
      double dF2 = sGap .Dot( rSvv );

      if ( smos_Fabs(dF1) > dNoiseLim  &&  dF1 + dF2 < -dNoiseLim )
        { return TRUE; }
  }

  return FALSE;

} // end SmSurfPtGlobalSolver::IsLocalMax

/*******************************************************************//**
PURPOSE: Is this uv guess on a boundary, and pushing outside the domain
            to approach the solution?

NOTES:
  If pushing both u and v (off a corner), will return only SM_SP_BOTH,
  no indication of low or high.
  (If you need that, you will have to add to the SmSurfParamType enum.)

  If right on a boundary (i.e., on the surface normal, not really 'pushing'),
  then this will return False if dTol is positive; i.e., you can differentiate
  between 'on' a boundary and 'pushing' a boundary.  You can make it return
  True for a right-on point by passing in a negative dTol.

  Singularities: Depending on the value of the degenerate parameter, it can
  appear that we're pushing off of a degenerate boundary; this really can't
  happen.  We will not return True for this case.

  The argument is an SmSurfaceEval, to avoid re-evaluation of surface.

  Outputs: peWhichBound: if given, one of
    SM_SP_UMIN, _UMAX, _VMIN, _VMAX, _NEITHER, or _BOTH.

  There are two versions of this method, one for SmSurfPtGlobalSolver and
  one for SmSurfPtSolveStepper.  Their behavior is the same, but the
  information available is different: the arguments are different.
***********************************************************************/
SmBoolean SmSurfPtGlobalSolver::PushingBoundary(
        SmSurfaceEval   & rSurfEval,   // in: current point
        double            dTol,        // in: uv-space tol for on-boundary check.
        SmSurfParamType * peWhichBound // out: see Usage Notes.
    )
{
  if ( peWhichBound != NULL )
    { *peWhichBound = SM_SP_NEITHER; }

  if ( ! m_sDomain.IsPoint2dOnBoundary( rSurfEval.GetUV(), smos_Fabs( dTol ) ) )
    { return FALSE; }

  SmVector3d sGap( m_sTestPt - rSurfEval.Pos() );
  if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
  {
      double dDot = - ( sGap.Dot( m_sDirVec ));
      sGap = dDot * m_sDirVec;
  }

  // Method: find the uv direction that the 3d step will take us,
  // and see whether it's heading out of bounds.

  // Notational, for readability:
  const SmPoint2d  & rUV = rSurfEval.GetUV();
  const SmVector3d & rSu = rSurfEval.Su();
  const SmVector3d & rSv = rSurfEval.Sv();

  // Finding the uv step is just writing the 3d step as
  // a linear combination of Su and Sv.
  SmVector2d sUVStep;
  SmStatus eStat = smgu_VecLinCombTwoVectors( rSu, rSv, sGap, sUVStep );

  if ( eStat != SM_SUCCESS )
    { return FALSE; }

  SmSurfParamType eRetVal = SM_SP_NEITHER;

  double dBdryTol    = SM_EFF_ZERO;
  double dPushingTol = dTol;

  if ( rUV.x < m_sDomain.GetUInterval().GetMin() + dBdryTol && sUVStep.x < -dPushingTol )
  {
      eRetVal = SM_SP_UMIN;
  }
  else if ( rUV.x > m_sDomain.GetUInterval().GetMax() - dBdryTol && sUVStep.x > dPushingTol )
  {
      eRetVal = SM_SP_UMAX;
  }

  if ( rUV.y < m_sDomain.GetVInterval().GetMin() + dBdryTol && sUVStep.y < -dPushingTol )
  {
      eRetVal = ( eRetVal == SM_SP_NEITHER ) ? SM_SP_VMIN : SM_SP_BOTH;
  }
  else if ( rUV.y > m_sDomain.GetVInterval().GetMax() - dBdryTol && sUVStep.y > dPushingTol )
  {
      eRetVal = ( eRetVal == SM_SP_NEITHER ) ? SM_SP_VMAX : SM_SP_BOTH;
  }

  if ( peWhichBound != NULL ) { *peWhichBound = eRetVal; }

  return eRetVal != SM_SP_NEITHER;

} // end SmSurfPtGlobalSolver::PushingBoundary

/*******************************************************************//**
PURPOSE: The main method of the newer version of SmSurfPtGlobalSolve.

NOTES:
  See Usage Notes for SmSurface::GlobalPointSolve().

***********************************************************************/
SmStatus SmSurfPtGlobalSolver::Solve( SmSolutionArray & rSolutions )
{
  rSolutions.ReSet();

  // We have to enable out-of-bounds evaluations.
  // Otherwise, the evaluators will silently clamp,
  // leading to wrong answers.  (Can happen if too-big domain passed in.)
  SmBoolean bTmp0 = FALSE ; 
  SmBSplineSurface * pBSplineSurface0 = SM_CAST_PTR(SmBSplineSurface, m_pSrf) ; 
  SmTemporaryChangeValue<SmBoolean> sClean0(pBSplineSurface0 ? pBSplineSurface0->GetOutOfBoundsEnabled() : bTmp0, TRUE) ;

  // First find a rough guess.
  SmPoint2d sUVGuessRough, sUVGuess;
  ULONG lBestIdxU, lBestIdxV;
  ULONG ii;

  SmSolution sSol;
  sSol.m_lNumVariables = 2;
  SmBoolean bFoundAnswer = FALSE;

  SmStatus eStat = FindRoughGuess( sUVGuessRough, lBestIdxU, lBestIdxV );

  if ( eStat != SM_SUCCESS )
      { SER( SM_ERR ); }

  // Check for creases.  LocalSolve does not always succeed in crease cases,
  // so solve each sub-domain separately.
  // Note: this is currently not used.

  SmTArray< SmExtent2d > sDomainList;
  GetCreaseSubdomains( sDomainList );

  SmSolutionArray sSolArray;
  for ( ii = 0; ii < sDomainList.GetSize(); ii++ )
  {
      SmExtent2d sThisDomain = sDomainList[ii];

      // Use only those guesses that are in or near this subdomain.
      if ( ! sThisDomain.ContainsPoint2d( sUVGuessRough, SM_EFF_ZERO_SQRT ) )
          { continue; }

      // Refine the guess.
      sUVGuess = sUVGuessRough;
      eStat = RefineGuess( sUVGuess, lBestIdxU, lBestIdxV );

      // We have to check here for seams: start on the proper side.
      CheckGuessForSeams( sUVGuess );

      // Do the local solve.
      SmPoint3d sPtForLocal = ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
                             ? -m_sDirVec : m_sTestPt;
      eStat = m_pSrf->LocalPointSolve( sThisDomain, m_eSolverOp, sPtForLocal,
              sUVGuess, bFoundAnswer, sSol );

      // Double-check this solution;
      // see whether we should try it again with a more refined guess.
      //           if Found:               if not Found, try again?
      // MIN    if local max, not found ->    Yes
      // MAX    if local min, not found ->    Yes
      // INT    if dist>tol,  not found ->    local min? No; pushing boundary? No; else Yes.
      // NORM   Done.                         pushing boundary? No; else Yes.

      SmBoolean bTryAgain = FALSE;
      SmSurfaceEval sSurfEval( m_pSrf, sSol.m_vStart[0], sSol.m_vStart[1] );

      switch ( m_eSolverOp )
      {
          case SM_SO_MINIMIZE:
          case SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE:
              if ( bFoundAnswer && IsLocalMax( sSurfEval, m_dLooseTol ) )
                  { bFoundAnswer = FALSE; }
              if ( ! bFoundAnswer ) { bTryAgain = TRUE; }
              break;

          case SM_SO_MAXIMIZE:
              if ( bFoundAnswer && IsLocalMin( sSurfEval, m_dLooseTol ) )
                  { bFoundAnswer = FALSE; }
              if ( ! bFoundAnswer ) { bTryAgain = TRUE; }
              break;

          case SM_SO_INTERSECT:
              if ( bFoundAnswer && sSol.m_vStart.m_dSolutionValue > m_dLooseTol )
                  { bFoundAnswer = FALSE; }
              if ( ! bFoundAnswer )
              {
                  if (   ! IsLocalMin( sSurfEval, m_dLooseTol )
                      && ! PushingBoundary( sSurfEval, m_dLooseTol ) )
                  {
                      bTryAgain = TRUE;
                  }
              }
              break;

          case SM_SO_NORMALIZE:
              if ( ! bFoundAnswer )
              {
                  // Note: Even if it's pushing a boundary, try again anyway.
                  // With a guess that's not good enough, the local iteration
                  // can wander to a boundary and push, while a better guess
                  // will succeed.
                  // if ( ! PushingBoundary( sSurfEval, m_dLooseTol ) )
                  // {
                      bTryAgain = TRUE;
                  // }
              }
              break;

          default:
              break;

      } // end switch, checking result of local solve.

      //
      if ( bTryAgain )
      {
          SmPoint2d sSaveGuess( sUVGuess );
          eStat = RefineGuessByGrid( sUVGuess, lBestIdxU, lBestIdxV );

          if ( eStat == SM_SUCCESS )
          {
              if ( ! sUVGuess.CloserThan( SM_EFF_ZERO_SQRT, sSaveGuess ) )
              {
                  eStat = m_pSrf->LocalPointSolve( sThisDomain, m_eSolverOp, sPtForLocal,
                          sUVGuess, bFoundAnswer, sSol );

                  if ( eStat == SM_SUCCESS && bFoundAnswer )
                  {
                      sSurfEval.SetUV( sSol.m_vStart[0], sSol.m_vStart[1] );
                  }
              }
          }
      } // end if bTryAgain

      // check the final result to see whether it's really a solution.
      if ( bFoundAnswer )
      {
          switch ( m_eSolverOp )
          {
              case SM_SO_MINIMIZE:
              case SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE:
                  if ( IsLocalMax( sSurfEval, m_dLooseTol ) )
                      { bFoundAnswer = FALSE; }
                  break;
              case SM_SO_MAXIMIZE:
                  if ( IsLocalMin( sSurfEval, m_dLooseTol ) )
                      { bFoundAnswer = FALSE; }
                  break;
              case SM_SO_INTERSECT:
                  if ( sSol.m_vStart.m_dSolutionValue > m_dLooseTol )
                      { bFoundAnswer = FALSE; }
                  break;
              case SM_SO_NORMALIZE:
                  if ( PushingBoundary( sSurfEval, m_dLooseTol ) )
                      { bFoundAnswer = FALSE; }
                  break;

              default:
                  break;

          } // end switch, checking result of local solve.
      }

      if ( eStat == SM_SUCCESS && bFoundAnswer )
      {
          if(sSol.m_lNumVariables == 0 )
          {
              sSol.m_lNumVariables = 2;
          }
          sSolArray.Add( sSol );
      }

  } // end loop over subdomains.

  // Sort out multiple solutions.
  // (Unnecessary if not checking creases.)
  double dBestSolValue = SM_BIG_DOUBLE;
  for ( ii = 0; ii < sSolArray.GetSize(); ii++ )
  {
      if ( sSolArray[ii].m_vStart.m_dSolutionValue < dBestSolValue )
      {
          sSol = sSolArray[ii];
          dBestSolValue = sSol.m_vStart.m_dSolutionValue;
          bFoundAnswer  = TRUE;
      }
  }

  // If on a seam, add across-seam solutions.
  if ( eStat == SM_SUCCESS && bFoundAnswer )
  {
      eStat = CheckSeams( m_dTightTol, sSol, rSolutions ); // TightTol: [080520]
  }

  return SM_SUCCESS;

} // end SmSurfPtGlobalSolver::Solve

/*******************************************************************//**
PURPOSE: Given a point in Euclidian space determine the corresponding
     extrema points on the surface.

NOTES:
  This is the newer of two versions of GlobalPointSolve().
  See Usage Notes there as well.

  For SM_SO_MINIMIZE and SM_SO_MAXIMIZE, this will return the closest
  point on a boundary, if the point is not on the surface normal
  within the domain.  It will always return a solution unless the
  result is a local maximum (MIN case) or local minimum (MAX case).

  This version is not completely implemented for the SM_SO_MAXIMIZE case;
  the base version will always call the older version for that.

  Returns a solution:
                  Off surface,    Outside domain
    Solver Op:     on normal:     (off normal; out of bounds):
  SM_SO_MINIMIZE      Yes             Yes: Closest point on boundary
  SM_SO_MAXIMIZE      Yes             Yes: Farthest point on boundary
  SM_SO_NORMALIZE     Yes             No
  SM_SO_INTERSECT     No              No

IMPLEMENTATION NOTES ---
  Find a guess point, then call LocalPointSolve() to refine it.
  If LocalPointSolve() fails, find a better guess point and try again.

***********************************************************************/
SmStatus SmSurface::GlobalPointSolve_1
 (const SmExtent2d         & crUVDomain,
  SmSolverOperationType      eSolverOp,
  const SmPoint3d          & crTestPoint,
  double                     dDistanceTolerance,
  const double             * cpdOptTargetDistance,
  SmSolutionRequestedType    eSolutionRequested,
  SmSolutionArray          & rSolutions)
 const
{
  SmSurfPtGlobalSolver sSolver( this,
                                crUVDomain,
                                eSolverOp,
                                crTestPoint,
                                dDistanceTolerance,
                                cpdOptTargetDistance,
                                eSolutionRequested );

  return sSolver.Solve( rSolutions );

} // end SmSurface::GlobalPointSolve_1


/*******************************************************************//**
 ***********************************************************************

        Local solver implementation: class SmSurfPtSolveStepper

***********************************************************************
***********************************************************************/

// Copy constructor
SmSurfPtSolveStepper::SmSurfPtSolveStepper( const SmSurfPtSolveStepper & crOther )
 : SmSurfacePointSolver( crOther ),
   m_sSrfEval( crOther.m_pSrf )
{
  m_sSrfEval.SetUV( crOther.UV() );
  m_bSet         = FALSE;  // see note above.
  m_iIter        = crOther.m_iIter;
  m_iBndHitU     = crOther.m_iBndHitU;
  m_iBndHitV     = crOther.m_iBndHitV;
  m_dMaxStepFrac = crOther.m_dMaxStepFrac;
  m_bNRSuspect   = crOther.m_bNRSuspect;
}

// Assignment operator.
// This does copy the evaluated data, if available.
SmSurfPtSolveStepper & SmSurfPtSolveStepper::operator=( const SmSurfPtSolveStepper &crOther )
{
  SmSurfacePointSolver::operator=( crOther );

  m_sSrfEval.SetUV( crOther.UV() );
  m_bSet       = crOther.m_bSet;
  m_iIter      = crOther.m_iIter;
  m_iBndHitU   = crOther.m_iBndHitU;
  m_iBndHitV   = crOther.m_iBndHitV;
  m_dMaxStepFrac = crOther.m_dMaxStepFrac;
  m_bNRSuspect = crOther.m_bNRSuspect;

  if ( m_bSet )
    {
      m_sSrfEval = crOther.m_sSrfEval;
      m_sGap    = crOther.m_sGap;
      m_sGapU   = crOther.m_sGapU;
      m_sGapV   = crOther.m_sGapV;
      m_dGapLen = crOther.m_dGapLen;
      m_dFVal   = crOther.m_dFVal;
      m_dGVal   = crOther.m_dGVal;
      m_dFCheck = crOther.m_dFCheck;
      m_dGCheck = crOther.m_dGCheck;
      m_dSumCheck = crOther.m_dSumCheck;
    }
  return *this;

} // end SmSurfPtSolveStepper::operator= assignment operator

/*******************************************************************//**
PURPOSE: Evaluate our data.

NOTES: Internal (private) method.
***********************************************************************/
SmStatus SmSurfPtSolveStepper::Eval()
{
  if ( m_bSet )
    { return SM_SUCCESS; }

  m_sGap  = Pos() - m_sTestPt;
  m_sGapU = Su();
  m_sGapV = Sv();
  if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
    {
      m_sGap  = m_sDirVec * ( m_sGap. Dot( m_sDirVec ) );
      m_sGapU = m_sDirVec * ( m_sGapU.Dot( m_sDirVec ) );
      m_sGapV = m_sDirVec * ( m_sGapV.Dot( m_sDirVec ) );
    }

  m_dGapLen = m_sGap.Length();

  m_dFVal = m_sGap.Dot( Su() );
  m_dGVal = m_sGap.Dot( Sv() );

  // Convert fcn vals to cosines for convergence test.
  double dSuLen = Su().Length();
  double dSvLen = Sv().Length();
  // Check zero-divide: crude, but mainly, if we're near a singularity,
  // don't allow that to make convergence impossible.
  double dMinDerivLength = 0.0001;
  if ( dSuLen < dMinDerivLength ) dSuLen = dMinDerivLength;
  if ( dSvLen < dMinDerivLength ) dSvLen = dMinDerivLength;

  // Note: do not divide by the distance.  It will cause failure to converge
  // on a plane, for example: the point is in the plane, and one derivative
  // points directly towards it, then the Check values will always be 1.0,
  // so a closer guess looks no better than a farther one.
  // // Divide by gap length, but again, if the point is
  // // right on the surface, don't disallow convergence.
  double dDist = 1.0;   // smos_Max( m_dGapLen, 1.0 );

  // We do need to divide by the derivative magnitudes though.
  // If not, then stepping across a G1-but-not-C1 discontinuity
  // will not work, because the eval on one side will be improperly
  // penalized by the big derivatives.

  m_dFCheck = m_dFVal / ( dDist * dSuLen );
  m_dGCheck = m_dGVal / ( dDist * dSvLen );
  m_dSumCheck = smos_Fabs( m_dFCheck ) + smos_Fabs( m_dGCheck );

  m_bSet = TRUE;

  return SM_SUCCESS;

}  // end SmSurfPtSolveStepper::Eval

/*******************************************************************//**
PURPOSE: Adjust a proposed step (from rPrevEval to 'this') according
            to domain limits, step size, etc.

NOTES: This does several things:
 - Limits step size to a certain fraction of the domain;
 - Clips to domain boundaries: tries both a simple clamp and a
   proportional, scaled clip, and uses the better;
 - Records any boundary hits.

 Error return:
 - returns SM_ERR if adjustment was appropriate but could not be accomplished.
   This can happen if this eval is at the same uv as prev, and outside the domain.
   (That has happened because of bit noise.)
***********************************************************************/
SmStatus SmSurfPtSolveStepper::AdjustStep
 (SmSurfPtSolveStepper & rPrevEval,    // in :
  SmBoolean            & rbAdjusted,   // out:
  double               & rdScale)      // out:
{
  rbAdjusted = FALSE;
  rdScale = 1.0;

  SmVector2d sStep( this->UV() - rPrevEval.UV() );
  double dOrigStepSize = sStep.Length();

  if ( dOrigStepSize < SM_EFF_ZERO_SQ )
  {
      if ( m_sDomain.ContainsPoint2d( UV() ) )
      {
          return SM_SUCCESS;
      }
      else
      {
          SetUV( m_sDomain.ClampPoint2d( UV() ) );
          rbAdjusted = TRUE;
          rdScale = 0.0;
          return SM_ERR;
      }
  }

  // Never take too big a step.
  // Scale the two parameters individually [ Fillet 2:234 ]
  double dSizeScale = 1.0;

  double dMaxStep = MaxStep();

  double dMaxUStep = dMaxStep * m_sDomain.GetUInterval().GetLength();
  if ( smos_Fabs( sStep.x ) > dMaxUStep )
  {
      rbAdjusted = TRUE;
      dSizeScale = dMaxUStep / smos_Fabs( sStep.x );
      sStep.x *= dSizeScale;
  }

  double dMaxVStep = dMaxStep * m_sDomain.GetVInterval().GetLength();
  if ( smos_Fabs( sStep.y ) > dMaxVStep )
  {
      rbAdjusted = TRUE;
      double dScaleY = smos_Min( dMaxVStep / smos_Fabs( sStep.y ), dSizeScale );
      sStep.y *= dScaleY;
      dSizeScale = smos_Min( dSizeScale, dScaleY );
  }

  // Apply the step and check out of bounds.
  SmPoint2d sCurrUV = rPrevEval.UV();
  SmPoint2d sNextUV( sCurrUV + sStep );

  int iHitBndU, iHitBndV;

  SmPoint2d sClampedUV, sClampedUVScaled;
  double dBoundsScale = 1.0;

  SmBoolean bClamped = CheckBounds( sCurrUV, sNextUV, m_sDomain,
      iHitBndU, iHitBndV, sClampedUV, sClampedUVScaled, dBoundsScale );

  this->HitBound( SM_SP_U, iHitBndU );
  this->HitBound( SM_SP_V, iHitBndV );

  if ( bClamped == FALSE )
  {
      this->SetUV( sNextUV );
  }
  else
  {
      // Clamped.  Use the better of scaled and unscaled.
      if ( dBoundsScale > 1.0 - SM_EFF_ZERO )
      {
          this->SetUV( sClampedUV );  // both are the same
      }
      else if ( dBoundsScale < SM_EFF_ZERO )
      {
          this->SetUV( sClampedUV );  // scaled would be zero step.
      }
      else
      {
          // Evaluate both and use the better one.
          this->SetUV( sClampedUV );

          SmSurfPtSolveStepper sScaledEval( *this );
          sScaledEval.SetUV( sClampedUVScaled );

          if ( sScaledEval.ErrorNorm() < this->ErrorNorm() )
          {
              *this = sScaledEval;
              // Using the scaled step, if both directions were out of bounds,
              // then probably only one is still OB.
              // Note, negative tol returns Inside only if not on boundary.
              if ( m_sDomain.GetUInterval().ContainsValue(
                      this->UV().x, -SM_EFF_ZERO ) )
                  { iHitBndU = 0; }
              if ( m_sDomain.GetVInterval().ContainsValue(
                      this->UV().y, -SM_EFF_ZERO ) )
                  { iHitBndV = 0; }
          }
      }
  }

  if ( dSizeScale < 1 || bClamped )
  {
      double dFinalStepSize = ( this->UV() - rPrevEval.UV() ).Length();
      rdScale = dFinalStepSize / dOrigStepSize;
      rbAdjusted = TRUE;
  }

  return SM_SUCCESS;

} // end SmSurfPtSolveStepper::AdjustStep

/*******************************************************************//**
PURPOSE: Record a boundary hit.

NOTES:
***********************************************************************/
void SmSurfPtSolveStepper::HitBound(
        SmSurfParamType eWhich,      // in: either SM_SP_U or SM_SP_V
        int             iTopOrBottom // in: must be +1 or -1, or 0.
    )
{
  // If the argument is 0, that could mean that we're just
  // sitting on a boundary, as opposed to being strictly
  // inside the domain.  If that case, leave the hit count
  // where it is.
  if ( iTopOrBottom == 0 )
  {
      if ( eWhich == SM_SP_U )
      {
          if ( m_iBndHitU != 0 )
          {
              if ( m_sDomain.GetUInterval().ContainsValue( m_sSrfEval.GetUV().x,
                          -SM_EFF_ZERO ))
                  { m_iBndHitU = 0; }
          }
      }
      else
      {
          if ( m_iBndHitV != 0 )
          {
              if ( m_sDomain.GetVInterval().ContainsValue( m_sSrfEval.GetUV().y,
                          -SM_EFF_ZERO ))
                  { m_iBndHitV = 0; }
          }
      }
  }
  else  // TopOrBottom is not 0: pushing outward.
  {
      if ( eWhich == SM_SP_U )
      {
          m_iBndHitU = ( m_iBndHitU*iTopOrBottom > 0 )
              ? m_iBndHitU + iTopOrBottom // pushing same direction
              : iTopOrBottom;  // no previous bound, or pushing opposite
      }
      else
      {
          m_iBndHitV = ( m_iBndHitV*iTopOrBottom > 0 )
              ? m_iBndHitV + iTopOrBottom // pushing same direction
              : iTopOrBottom;  // no previous bound, or pushing opposite
      }
  }
}  // end SmSurfPtSolveStepper::HitBound

/*******************************************************************//**
PURPOSE: Return a measure of goodness of an evalution.

NOTES:
  This is used to compare two evaluations, and to test convergence.
  The argument depends on what it is being used for.
***********************************************************************/
double SmSurfPtSolveStepper::ErrorNorm( SmSrfPtErrorNormType eNormType )
{
  Eval();  // Make sure we're initialized.

  // Method: the 'Check' values use unitized derivatives.
  // We will use them when comparing values, to allow stepping over
  // a C1 discontinuity: don't penalize because srf derivs get bigger.
  // [ See the surface in file ../igesfiles/drop/PS10_drp_Srf3.dat. ]

  switch ( eNormType )
  {
      case SM_SPN_SUM_ABS: { return m_dSumCheck; }
      case SM_SPN_SUM:     { return m_dFCheck + m_dGCheck; }
      case SM_SPN_F:       { return m_dFCheck; }
      case SM_SPN_G:       { return m_dGCheck; }
      case SM_SPN_F_ABS:   { return smos_Fabs( m_dFCheck ); }
      case SM_SPN_G_ABS:   { return smos_Fabs( m_dGCheck ); }
      case SM_SPN_MAX:     { return ( smos_Fabs( m_dFCheck ) >= smos_Fabs( m_dGCheck ) )
                                    ? m_dFCheck : m_dGCheck; }
      case SM_SPN_MAX_ABS: { return ( smos_Fabs( m_dFCheck ) >= smos_Fabs( m_dGCheck ) )
                                    ? smos_Fabs( m_dFCheck ) :  smos_Fabs( m_dGCheck ); }
      case SM_SPN_GAP:     { return m_dGapLen; }
  }

  // default: SM_SPN_SUM_ABS.  (Should actually be unreachable code.)
  return smos_Fabs( m_dFVal ) + smos_Fabs( m_dGVal );

} // end SmSurfPtSolveStepper::ErrorNorm()

/*******************************************************************//**
PURPOSE--- Check whether this evaluation has converged.

USAGE NOTES---
  This checks only whether we're at a solution of the NR iteration,
  which is to say, the point is on the surface normal,
  and continued iteration will not help.
  It's up to the caller to check out-of-bounds, crease,
  or anything that depends on the solver operation type.
***********************************************************************/
SmBoolean SmSurfPtSolveStepper::Converged( SmBoolean bUseTightTol )
{
    double dTol = ( bUseTightTol ) ? m_dTightTol : m_dLooseTol;

    return Converged( dTol );

} // end SmSurfPtSolveStepper::Converged

/**************************************************
// Purpose--- Check whether this evaluation has converged.
//
// Usage Notes---
//   See the other Converged().
***************************************************/
SmBoolean SmSurfPtSolveStepper::Converged( double dTol )
{
  Eval();

  // If surface point equals test point, we're done.
  if ( ErrorNorm( SM_SPN_GAP ) < dTol ) { return TRUE; }

  // Check on the normal.  Not reliable for singularities however.
  SmSurfParamType eSingDir;
  if ( ! IsSingular( &eSingDir ) )
  {
      return ( ErrorNorm( SM_SPN_SUM_ABS ) < dTol );
  }

  // We're at a singularity.

  SmVector3d & sNorm = this->Normal();

  if ( sNorm.LengthSquared() < SM_EFF_ZERO_SQ )
  {
      return FALSE;
  }

  // For this case, converged is when the normals line up.
  // Either min or max, as per comments.
  if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
  {
      // Both are unit vectors, so if lined up, then either adding
      // or subracting would be a zero vector.
      SmVector3d sDiff = ( sNorm.Dot( m_sDirVec ) > 0 )
          ? sNorm - m_sDirVec : sNorm + m_sDirVec;

      return ( sDiff.LengthSquared() <= dTol*dTol );
  }

  // All other cases: check distance from test pt to surface normal.
  double dLineT;
  SmStatus eStat = smgu_LineClosestPoint( Pos(), sNorm, m_sTestPt, dLineT );
  if ( eStat != SM_SUCCESS )
  {
      return FALSE;
  }

  SmVector3d sLinePnt = Pos() + dLineT * sNorm;
  SmVector3d sDiff = sLinePnt - m_sTestPt;

  // Scale by how far the point is from the surface.
  // Angle tol is what matters here.
  double dDistToSurf = sLinePnt.DistanceBetween( Pos() );
  if ( dDistToSurf < 1.0 ) { dDistToSurf = 1.0; }

  if ( sDiff.Length() < dTol * dDistToSurf ) {
      return TRUE;
  }

  return FALSE;

} // end SmSurfPtSolveStepper::Converged

/*******************************************************************//**
PURPOSE: Checks whether the eval is on the surface normal,
            and is a local distance minimum.

NOTES: Purely geometric, doesn't check SolverOp.
***********************************************************************/
SmBoolean SmSurfPtSolveStepper::IsLocalMin( double dTol )
{
  Eval();

  if ( ErrorNorm( SM_SPN_GAP     ) <= dTol ) { return TRUE; }
  if ( ErrorNorm( SM_SPN_SUM_ABS ) >  dTol ) { return FALSE; }

  // We're off the surface, and on the surface normal.
  // It's a relative min if we're on the 'convex' side of the surface,
  // or if we're within the radius of curvature.
  // An easy way to test this is: beyond the center of curvature,
  // a Newton step will try to increase the distance, but a plane
  // step will always decrease it.  Of course we can't actually
  // take a step, because we're converged (on the normal; the numerator
  // would be zero), but: the difference between the two steps is the
  // denominator.  For NR, it is S' dot S'  +  Gap dot S'', and for
  // a plane step, simply leave off the second term.  A Newton step
  // will go 'backwards' if the denominator changes sign.  The first
  // term is always positive, so we're at a relative maximum if and
  // only if the sum is negative.

  // So what if we're right at the center of curvature?
  // Design decision: We'll say that's both a min and a max.

  // Do it separately for both parameters.  If either one indicates
  // a maximum, call it a maximum.  (Consider a cylinder: it will
  // always be a minimum in one direction, and if it's a maximum
  // in the other, then it's a maximum overall.)

  double dF1 = m_sGapU.Dot( Su() );
  double dF2 = m_sGap .Dot( Suu() );

  if ( dF1 + dF2 <  0 ) { return FALSE; }
  if ( dF1 + dF2 == 0 ) { return TRUE; }

  dF1 = m_sGapV.Dot( Sv() );
  dF2 = m_sGap .Dot( Svv() );

  return ( dF1 + dF2 >= 0 );

} // end SmSurfPtSolveStepper::IsLocalMin

/*******************************************************************//**
PURPOSE: Checks whether the eval is on the surface normal,
            and is a local distance maximum.

NOTES: Purely geometric, doesn't check SolverOp.
***********************************************************************/
SmBoolean SmSurfPtSolveStepper::IsLocalMax( double dTol )
{
  // See comments in IsLocalMin().
  Eval();

  if ( ErrorNorm( SM_SPN_GAP     ) <= dTol ) { return FALSE; }
  if ( ErrorNorm( SM_SPN_SUM_ABS ) >  dTol ) { return FALSE; }

  double dF1 = m_sGapU.Dot( Su() );
  double dF2 = m_sGap .Dot( Suu() );

  if ( dF1 + dF2 <= 0 ) { return TRUE; }

  dF1 = m_sGapV.Dot( Sv() );
  dF2 = m_sGap .Dot( Svv() );

  return ( dF1 + dF2 <= 0 );

} // end SmSurfPtSolveStepper::IsLocalMax

/*******************************************************************//**
PURPOSE: Check whether this evaluation is near a singularity.

NOTES:
  "Near a singularity", for our purposes, means that one derivative
  is going to zero.  We check for one being much smaller than the other.
  A factor of 10000 is a good indicator.  (5000 can be too small.)
  But, the ratio is not enough to determine a singular situation:
  For example, some 'infinite' analytic surfaces can have a domain that
  extends to 1234567.0 in one direction, and roughly unity in the other.
  We do not want to treat that situation as a singularity, so we have to
  check some absolute values in addition to relative ones.  [Reg_060324.cone]
***********************************************************************/
SmBoolean SmSurfPtSolveStepper::IsSingular(
        SmSurfParamType * peWhichParam // out: if not null, one of SM_SP_U, SM_SP_V, SM_SP_NEITHER.
    )
{
  Eval();

  double dRatioLimitSquared = 10000 * 10000;
  double dAbsMagLimit       = SM_EFF_ZERO_PARAM;

  SmSurfParamType eLocalWhich = SM_SP_NEITHER;

  double dMagSuSq = Su().Dot( Su() );
  double dMagSvSq = Sv().Dot( Sv() );

  if ( dMagSuSq < dMagSvSq / dRatioLimitSquared && dMagSuSq < dAbsMagLimit )
    { eLocalWhich = SM_SP_U; }

  if ( dMagSvSq < dMagSuSq / dRatioLimitSquared && dMagSvSq < dAbsMagLimit )
    { eLocalWhich = SM_SP_V; }

  if ( peWhichParam != NULL )
    { *peWhichParam = eLocalWhich; }

  return ( eLocalWhich != SM_SP_NEITHER );

} // end SmSurfPtSolveStepper::IsSingular

/*******************************************************************//**
PURPOSE: Take a simple step.

NOTES:
 Will take either a NR step or a Plane step.
 Does some checks on the step.
 Does not check out-of-bounds.
 Returns one of these three:
 - SM_TR_STILL_ITERATING: the step worked, should be an improvement.
 - SM_TR_FOUND_ANSWER_CONVERGED: Found an end to iteration.
   It might not be a perfect solution within tight tolerance, but we know
   that more steps won't help, so don't bother continuing to search.
 - SM_TR_UNABLE_TO_CONVERGE: couldn't find any good step.

***********************************************************************/
SmTerminationReasonType SmSurfPtSolveStepper::Step(
        SmSurfPtSolveStepper & rPrevEval,
        SmBoolean bDoPlaneStep
    )
{
  this->Eval();
  rPrevEval.Eval();

  // Special case: Plane step with directed operations:
  // Can get direction but not step size.
  // The caller could help out by doing a line search.
  if ( bDoPlaneStep && SolverOp() == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
  {
      SmVector2d sStep;
      smgu_VecLinCombTwoVectors( rPrevEval.Su(), rPrevEval.Sv(), -m_sDirVec, sStep );
      double dMag = sStep.Length();
      if ( dMag > SM_EFF_ZERO_SQRT / 100 )
      {
          constexpr double dStepSizeGuess = 0.05;
          double dDomainSize = Domain().GetSize().Length();
          sStep *= ( dStepSizeGuess * dDomainSize / dMag );
      }
      this->SetUV( rPrevEval.UV() + sStep );

      return SM_TR_STILL_ITERATING;
  }

  SmBoolean bGotStep = FALSE;
  SmStatus  eStat;

  // Check for near singularity: if one deriv is much smaller than the other.
  // Unfortunately, this isn't implemented for signed-directed case.
  if ( rPrevEval.IsSingular() && m_eSolverOp != SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
  {
      SmPoint2d sSingUV( rPrevEval.UV() );
      SmTerminationReasonType eReason;

      // Near singularity, use the surface's own domain. [B50]
      SmExtent2d sSrfDomain = m_pSrf->GetNaturalUVDomain();

      eStat = m_pSrf->InvertPointNearSingularity(
                  m_sTestPt, sSrfDomain, sSingUV, m_dTightTol, eReason );

      if ( eStat == SM_SUCCESS )
      {
          this->SetUV( sSingUV );
          return SM_TR_FOUND_ANSWER_CONVERGED; // Signal to end iteration.
      }
  }

  // Calculate the function values, and their derivatives for a step.

  // We use unitized first derivatives.  See the white paper PointInversion.docx.
  // Need to calculate the following:
  SmVector3d sSuN, sSvN;                 // Normalized 1st derivs: sSu/dMU, sSv/dMV
  double dMU, dMV;                       // magnitudes of sSu, sSv

  dMU = rPrevEval.Su().Length();
  dMV = rPrevEval.Sv().Length();
  // We already checked for singularity, so this is unlikely.
  if ( dMU < SM_EFF_ZERO || dMV < SM_EFF_ZERO )
    {
      return SM_TR_UNABLE_TO_CONVERGE;
    }

  sSuN = rPrevEval.Su() / dMU;
  sSvN = rPrevEval.Sv() / dMV;

  // Now the function values.
  double dF = rPrevEval.m_sGap.Dot( sSuN );
  double dG = rPrevEval.m_sGap.Dot( sSvN );

  // Now the derivatives of the function values: Jacobian.

  double dFu, dFv, dGu, dGv;

  // First we need the four derivatives of the unitized first derivatives.
  SmVector3d sSuNu = rPrevEval.Su().UnitizedDerivative( rPrevEval.Suu() );
  SmVector3d sSuNv = rPrevEval.Su().UnitizedDerivative( rPrevEval.Suv() );
  SmVector3d sSvNu = rPrevEval.Sv().UnitizedDerivative( rPrevEval.Suv() );
  SmVector3d sSvNv = rPrevEval.Sv().UnitizedDerivative( rPrevEval.Svv() );

  // Keep the two terms of Fu and Gv separate: their relative
  // sizes and signs can tell us something about the iteration.

  double dFu1 = rPrevEval.m_sGapU.Dot( sSuN  );
  double dFu2 = rPrevEval.m_sGap .Dot( sSuNu );

  // About these two: dFu1 is always positive.
  // If dFu2 is negative, that means the point is on the
  // 'inside' of the u isocurve: i.e., on the same side as
  // the center of curvature.  Roughly, (on a circle,
  // precisely), when dFu2 == -dFu1, the point is at
  // the center of curvature, and there's a corresponding
  // zero denominator.  Beyond the center of curvature,
  // the denominator will change sign.  It can be seen
  // geometrically that ( -dF / dFu1 ) will always result
  // in a step that goes physically closer to the test point.
  // So if we're trying to move closer, we don't want the
  // denominator to change sign.
  // So we limit the magnitude of dFu2, if it's negative.
  // This also precludes a zero-divide.

  //cbi: in practice, -1.5 might work better: PT_22Feb07.

  constexpr double d2ndDerivTermLimit = 0.90;

  if ( dFu2 < -dFu1 * d2ndDerivTermLimit ) // Note: dFu1 always positive.
  {
      if (   SolverOp() == SM_SO_MINIMIZE || SolverOp() == SM_SO_INTERSECT
          || SolverOp() == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
      {
          this->SetNRSuspect( TRUE );
          dFu2 = -dFu1 * d2ndDerivTermLimit;
      }
  }


  double dGv1 = rPrevEval.m_sGapV.Dot( sSvN  );
  double dGv2 = rPrevEval.m_sGap .Dot( sSvNv );

  if ( dGv2 < -dGv1 * d2ndDerivTermLimit ) // Note: dGv1 always positive.
  {
      if ( SolverOp() == SM_SO_MINIMIZE || SolverOp() == SM_SO_INTERSECT )
      {
          this->SetNRSuspect( TRUE );
          dGv2 = -dGv1 * d2ndDerivTermLimit;
      }
  }

  if ( bDoPlaneStep == FALSE )
  {
      // Newton step.
      dFu = dFu1 + dFu2;
      dGv = dGv1 + dGv2;
      dFv = rPrevEval.m_sGapV.Dot( sSuN ) + rPrevEval.m_sGap.Dot( sSuNv );
      dGu = rPrevEval.m_sGapU.Dot( sSvN ) + rPrevEval.m_sGap.Dot( sSvNu );
  }
  else
  {
      // planar step: ignore 2nd deriv terms.
      dFu = dFu1;
      dGv = dGv1;
      dFv = rPrevEval.m_sGapV.Dot( sSuN );
      dGu = rPrevEval.m_sGapU.Dot( sSvN );
  }

  // 2x2 for the steps: Cramer's rule
  double dDet    =  ( dFu * dGv - dGu * dFv );
  double dNumerU = -( dF  * dGv - dG  * dFv );
  double dNumerV = -( dFu * dG  - dGu * dF  );

  if (   smos_Fabs( dDet ) <= SM_EFF_ZERO * smos_Fabs( dNumerU )
      || smos_Fabs( dDet ) <= SM_EFF_ZERO * smos_Fabs( dNumerV ) )
  {
      this->SetNRSuspect( TRUE );
  }
  else
  {
      // Calculate the step.
      SmVector2d sStep;

      bGotStep = TRUE;

      sStep.Set( dNumerU/dDet, dNumerV/dDet );

      this->SetUV( rPrevEval.UV() + sStep );

  }  // end calculating the step.

  return ( bGotStep ) ? SM_TR_STILL_ITERATING : SM_TR_UNABLE_TO_CONVERGE;

} // end SmSurfPtSolveStepper::Step().

/*******************************************************************//**
PURPOSE: Check whether the next uv guess is out of bounds.

NOTES:
  return value: True iff crNextUV is outside of domain
  The output uv values are unset if not clamped.
***********************************************************************/
SmBoolean SmSurfPtSolveStepper::CheckBounds(
        SmPoint2d  const & crCurrUV,  // in: starting point
        SmPoint2d  const & crNextUV,  // in: the point to check
        SmExtent2d const & crDomain,  // in
        int & riHitsU,                // out: -1 low bound, +1 high, 0 neither.
        int & riHitsV,                // out:
        SmPoint2d & rClampedUV,       // out: simple clamping to 2d boundary.
        SmPoint2d & rClampedUVScaled, // out: scaled: on the line from Curr to Next.
        double    & rdStepScale       // out: ratio of clamped step size to original.
    )
{
  riHitsU = riHitsV = 0;

  rdStepScale = 1.0;

  double dUMin = crDomain.GetMin().x;
  double dVMin = crDomain.GetMin().y;
  double dUMax = crDomain.GetMax().x;
  double dVMax = crDomain.GetMax().y;

  double dThisScale;

  if ( crNextUV.x < dUMin )
  {
      riHitsU = -1;
      if ( crCurrUV.x >= dUMin )
        { rdStepScale = ( dUMin - crCurrUV.x ) / ( crNextUV.x - crCurrUV.x ); }
      else
        { rdStepScale = 0.0; }
  }
  else if ( crNextUV.x > dUMax )
  {
      riHitsU = 1;
      if ( crCurrUV.x <= dUMax )
        { rdStepScale = ( dUMax - crCurrUV.x ) / ( crNextUV.x - crCurrUV.x ); }
      else
        { rdStepScale = 0.0; }
  }

  if ( crNextUV.y < dVMin )
  {
      riHitsV = -1;
      if ( crCurrUV.y >= dVMin )
        { dThisScale = ( dVMin - crCurrUV.y ) / ( crNextUV.y - crCurrUV.y ); }
      else
        { dThisScale = 0.0; }

      if ( dThisScale < rdStepScale )
          { rdStepScale = dThisScale; }
  }
  else if ( crNextUV.y > dVMax )
  {
      riHitsV = 1;
      if ( crCurrUV.y <= dVMax )
        { dThisScale = ( dVMax - crCurrUV.y ) / ( crNextUV.y - crCurrUV.y ); }
      else
        { dThisScale = 0.0; }

      if ( dThisScale < rdStepScale )
          { rdStepScale = dThisScale; }
  }

  SmBoolean bRetVal = FALSE;

  if ( riHitsU != 0 || riHitsV != 0 )
  {
      bRetVal = TRUE;
      rClampedUVScaled = crCurrUV + rdStepScale * ( crNextUV - crCurrUV );

      rClampedUV.Set( crNextUV.x, crNextUV.y );

      if ( riHitsU < 0 )
      {
          rClampedUV.Set( dUMin, rClampedUV.y );
      }
      else if ( riHitsU > 0 )
      {
          rClampedUV.Set( dUMax, rClampedUV.y );
      }

      if ( riHitsV < 0 )
      {
          rClampedUV.Set( rClampedUV.x, dVMin );
      }
      else if ( riHitsV > 0 )
      {
          rClampedUV.Set( rClampedUV.x, dVMax );
      }
  }

  return bRetVal;

} // end SmSurfPtSolveStepper::CheckBounds

/*******************************************************************//**
PURPOSE: Try to find a better solution between the previous guess and this one.

NOTES:
***********************************************************************/
SmStatus SmSurfPtSolveStepper::LineSearch(
        SmSurfPtSolveStepper & rPrevEval  // in
    )
{
  // Originally tried a quadratic three-point search, but the
  // functions proved to be too ill-behaved.  So we use a more
  // brute-force algorithm: just sample along the line.

  // If one function value (F or G) is much bigger than the other,
  // and it changes sign (its zero is bracketted),
  // iterate on that one only, bringing it near zero.

  if ( rPrevEval.ErrorNorm( SM_SPN_F ) * this->ErrorNorm( SM_SPN_F ) < 0 )
  {
      if ( ( rPrevEval.ErrorNorm( SM_SPN_F_ABS ) > 10 * rPrevEval.ErrorNorm( SM_SPN_G_ABS ) )
        && ( this->ErrorNorm( SM_SPN_F_ABS ) > 10 * this->ErrorNorm( SM_SPN_G_ABS ) )
         )
      {
          return LineSearchBracketted( rPrevEval, SM_SPN_F );
      }
  }
  if ( rPrevEval.ErrorNorm( SM_SPN_G ) * this->ErrorNorm( SM_SPN_G ) < 0 )
  {
      if ( ( rPrevEval.ErrorNorm( SM_SPN_G_ABS ) > 10 * rPrevEval.ErrorNorm( SM_SPN_F_ABS ) )
        && ( this->ErrorNorm( SM_SPN_G_ABS ) > 10 * this->ErrorNorm( SM_SPN_F_ABS ) )
         )
      {
          return LineSearchBracketted( rPrevEval, SM_SPN_G );
      }
  }


  // Since we're likely in a problem area, use 3d gap for the error,
  // unless we're Normalizing or doing a directed operation.
  SmSrfPtErrorNormType eNormType = SM_SPN_GAP;
  if (   SolverOp() == SM_SO_NORMALIZE
      || SolverOp() == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
  {
      eNormType = SM_SPN_SUM_ABS;
  }

  // Note: currently, just try to find something better,
  // don't work hard to really minimize.

  SmSurfPtSolveStepper sThirdEval( *this ); // init to this: bnds counts, etc.

  // Set dBestErr and dBestVal.
  double dErr0 = rPrevEval.ErrorNorm( eNormType );
  double dErr1 = this->ErrorNorm( eNormType );

  double dBestErr = dErr0;
  double dBestVal = 0.0;
  if ( dErr1 < dBestErr )
  {
      dBestErr = dErr1;
      dBestVal = 1.0;
  }
  double dBestInitErr = dBestErr;

  const int clNumGuesses = 4;  // was: 7;
  double adVals[7] = { 0.1, 0.2, 0.4, 0.7 }; // was: { 0.5, 0.2, 0.8, 0.4, 0.6, 0.1, 0.9 };
  double dFraction, dThisErr;
  int ii;

  for ( ii=0; ii<clNumGuesses; ii++ )
  {
      // If this is not better than PrevEval, do two things:
      // - Check whether the fcns are improving locally, at PrevEval in the
      //   direction of this;
      //   - If so, then set the third eval to an improved place
      //     between Curr and this;
      //   - If not, find a direction of improvement at PrevEval,
      //     set the third eval to it,
      //     and set this to a farther point in that direction.

      dFraction = adVals[ii];

      sThirdEval.SetUV( ( 1-dFraction ) * rPrevEval.UV() + dFraction * this->UV() );

      dThisErr = sThirdEval.ErrorNorm( eNormType );
      if ( dThisErr < dBestErr )
      {
          // If good enough, return it,
          // else record it and keep looking.

          if ( dThisErr < dBestInitErr * 0.50 ) //cbi: 0.50?
          {
              *this = sThirdEval;
              return SM_SUCCESS;
          }

          dBestErr = dThisErr;
          dBestVal = dFraction;
      }
  } // end for


  // If no minimum between the two inputs,
  // try outside the range.
  if ( dBestVal < 0.01 && dErr0 < dErr1 )
  {
      // Appears to be monotonically increasing.
      dFraction = -1.0;
      sThirdEval.SetUV( ( 1-dFraction ) * rPrevEval.UV() + dFraction * this->UV() );
      // Need to clamp this.
      sThirdEval.Clamp();

      dThisErr = sThirdEval.ErrorNorm( eNormType );
      if ( dThisErr < dBestErr )
      {
          dBestErr = dThisErr;
          dBestVal = dFraction;
      }
  }

  if ( dBestVal > 0.99 && dErr1 < dErr0 )
  {
      // Appears to be monotonically decreasing.
      dFraction = 2.0;
      sThirdEval.SetUV( ( 1-dFraction ) * rPrevEval.UV() + dFraction * this->UV() );
      // Need to clamp this.
      sThirdEval.Clamp();

      dThisErr = sThirdEval.ErrorNorm( eNormType );
      if ( dThisErr < dBestErr )
      {
          dBestErr = dThisErr;
          dBestVal = dFraction;
      }
  }


  this->SetUV( ( 1-dBestVal ) * rPrevEval.UV() + dBestVal * this->UV() );

  this->Clamp(); // In case not between the two inputs.

  return SM_SUCCESS;

}  // end SmSurfPtSolveStepper::LineSearch

/*******************************************************************//**
PURPOSE: Perform a line search between two Evals that bracket the solution.

NOTES:
***********************************************************************/
SmStatus SmSurfPtSolveStepper::LineSearchBracketted(
        SmSurfPtSolveStepper & rPrevEval, // in
        SmSrfPtErrorNormType eNormType // in: must be on that allows negative values.
    )
{
  if (    eNormType == SM_SPN_SUM_ABS
       || eNormType == SM_SPN_F_ABS
       || eNormType == SM_SPN_G_ABS
       || eNormType == SM_SPN_MAX_ABS
       || eNormType == SM_SPN_GAP
     )
  {
      return SM_ERR;
  }

  double dY0 = rPrevEval.ErrorNorm( eNormType );
  double dY1 = this->ErrorNorm( eNormType );

  if ( dY0 * dY1 > 0 )
  {
      return SM_ERR;
  }

  double dBestInputY = smos_Min( smos_Fabs( dY0 ), smos_Fabs( dY1 ) );

  int ii, iMaxIter = 5; // Don't spend too much time on a line search.

  // Call the initial points the unit interval:
  double dX0 = 0;
  double dX1 = 1;

  double dXMid, dYMid;
  SmSurfPtSolveStepper sMidEval( rPrevEval );

  for ( ii=0; ii<iMaxIter; ii++ )
  {
      // The secant step: find the zero on the line between the evals.
      // The division should be fine because dY0 and dY1 have opposite signs.
      dXMid = dX0 - dY0 * ( dX1-dX0 ) / ( dY1-dY0 );

      sMidEval.SetUV( ( 1-dXMid ) * rPrevEval.UV() + dXMid * this->UV() );

      // Due to numerical noise, this interpolation on dXMid can result in
      // a value that is not between prev and this, particularly if dXMid is
      // very near 0 or 1.  This will cause trouble if prev and this were
      // right on a boundary, and the result is out by a bit or two.  So:
      sMidEval.SetUV( m_sDomain.ClampPoint2d( sMidEval.UV() ) );

      dYMid = sMidEval.ErrorNorm( eNormType );

      if ( smos_Fabs( dYMid ) < SM_EFF_ZERO_SQRT ) {
          *this = sMidEval;
          return SM_SUCCESS;
      }
      // Another check: the fcn value can get worse with this method.
      // If it's not improving, try something different: a bisection.
      if ( ii >= iMaxIter-1 && smos_Fabs( dYMid ) >= dBestInputY )
      {
          iMaxIter += 5;
          if ( iMaxIter > 30 )
          {
              break;
          }
          dXMid = ( dX0 + dX1 ) / 2;

          sMidEval.SetUV( ( 1-dXMid ) * rPrevEval.UV() + dXMid * this->UV() );
          dYMid = sMidEval.ErrorNorm( eNormType );
      }

      if ( dYMid * dY0 < 0 )
      {
          dX1 = dXMid;
          dY1 = dYMid;
      }
      else
      {
          dX0 = dXMid;
          dY0 = dYMid;
      }
  } // end iter

  *this = sMidEval;

  return SM_SUCCESS;

} // end SmSurfPtSolveStepper::LineSearchBracketted

/* ***************************************************************
PURPOSE: Is this eval on a boundary, and converged?

NOTES:
  If on a boundary, this checks whether the eval is converged,
  i.e., whether it is the closest (or farthest) point to the
  test point, so that iterating further will not help.

  Outputs: peWhichBound: if given, one of
    SM_SP_UMIN, _UMAX, _VMIN, _VMAX, _NEITHER, or _BOTH.
*************************************************************** */
SmBoolean SmSurfPtSolveStepper::ConvergedOnBoundary( double dTol, SmSurfParamType *peWhichBound )
{
    SmSurfParamType eRetVal = SM_SP_NEITHER;

    if ( !PushingBoundary( -SM_EFF_ZERO, &eRetVal ) ) // Neg tol: if right on, do the check.
    {
        return FALSE;
    }

    if ( peWhichBound != NULL ) { *peWhichBound = eRetVal; }

    SM_ASSERT( eRetVal == SM_SP_UMIN || eRetVal == SM_SP_UMAX
            || eRetVal == SM_SP_VMIN || eRetVal == SM_SP_VMAX
            || eRetVal == SM_SP_BOTH );

    Eval();

    // For u or v: check whether it's normal to the boundary curve.
    // For u, that would be the value of the G-function; F-function for v.
    // If not normal, then check whether it's pushing off the end of
    // the boundary curve; i.e., off a corner of the domain.

    if ( eRetVal == SM_SP_UMIN || eRetVal == SM_SP_UMAX || eRetVal == SM_SP_BOTH )
    {
        if ( ErrorNorm( SM_SPN_G_ABS ) < dTol )
          { return TRUE; } // perpendicular to boundary curve.

        double dParam = UV().y;
        SmExtent1d sCrvDomain = m_sDomain.GetVInterval();
        if ( dParam < sCrvDomain.GetMin() + dTol )
        {
            return ( ErrorNorm( SM_SPN_G ) > -dTol );
        }
        else if ( dParam > sCrvDomain.GetMax() - dTol )
        {
            return ( ErrorNorm( SM_SPN_G ) <  dTol );
        }
    }
    if ( eRetVal == SM_SP_VMIN || eRetVal == SM_SP_VMAX || eRetVal == SM_SP_BOTH )
    {
        if ( ErrorNorm( SM_SPN_F_ABS ) < dTol )
          { return TRUE; } // perpendicular to boundary curve.

        double dParam = UV().x;
        SmExtent1d sCrvDomain = m_sDomain.GetUInterval();
        if ( dParam < sCrvDomain.GetMin() + dTol )
        {
            return ( ErrorNorm( SM_SPN_F ) > -dTol );
        }
        else if ( dParam > sCrvDomain.GetMax() - dTol )
        {
            return ( ErrorNorm( SM_SPN_F ) <  dTol );
        }
    }

    return FALSE;
} // end SmSurfPtSolveStepper::ConvergedOnBoundary

/* ***************************************************************
PURPOSE: Is this uv guess on a boundary, and pushing outside the domain
            to approach the solution?

NOTES:
  If pushing both u and v (off a corner), will return only SM_SP_BOTH,
  no indication of low or high.
  (If you need that, you will have to add new values to the SmSurfParamType enum.)

  If right on a boundary (i.e., on the surface normal, not really 'pushing'),
  then this will return False if dTol is positive; i.e., you can differentiate
  between 'on' a boundary and 'pushing' a boundary.  You can make it return
  True for a right-on point by passing in a negative dTol.

  Singularities:  At the pole of a sphere, for example, if the value of
  the degenerate parameter is on the 'wrong side' of the pole, then it can
  appear to be pushing off of the degenerate boundary.  Formerly, this
  function would not return True in that case, but that caused a problem
  in the cone-type degeneracy case: there, it really is pushing off a
  boundary.  So the behavior now is to return True in any case; the
  sphere-type singularity case must be caught at a higher level.
  [090212; 090227]


  Outputs: peWhichBound: if given, one of
    SM_SP_UMIN, _UMAX, _VMIN, _VMAX, _NEITHER, or _BOTH.

  There are two versions of this method, one for SmSurfPtGlobalSolver and
  one for SmSurfPtSolveStepper.  Their behavior is the same, but the
  information available is different: the arguments are different.
*************************************************************** */
SmBoolean SmSurfPtSolveStepper::PushingBoundary( double dTol, SmSurfParamType *peWhichBound )
{
  SmSurfParamType eRetVal = SM_SP_NEITHER;

  Eval();

  SmVector3d sGap( m_sTestPt - Pos() );
  if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
  {
      double dDot = sGap.Dot( m_sDirVec );
      sGap = dDot * m_sDirVec;
  }

  // Method: find the uv direction that the 3d step will take us,
  // and see whether it's heading out of bounds.

  // Finding the uv step is just writing the 3d step as
  // a linear combination of Su and Sv.
  SmVector2d sUVStep;
  SmStatus eStat = smgu_VecLinCombTwoVectors( Su(), Sv(), sGap, sUVStep );

  if ( eStat != SM_SUCCESS )
    { return FALSE; }


  double dBdryTol    = SM_EFF_ZERO;
  double dPushingTol = dTol;

//  // Check for singularity.
//  SmSurfParamType eSingDir;
//  SmBoolean bIsSingular = this->IsSingular( & eSingDir );

//  if ( eSingDir != SM_SP_V )
//  {

      if ( UV().x < m_sDomain.GetUInterval().GetMin() + dBdryTol && sUVStep.x < -dPushingTol )
      {
          eRetVal = SM_SP_UMIN;
      }
      else if ( UV().x > m_sDomain.GetUInterval().GetMax() - dBdryTol && sUVStep.x >  dPushingTol )
      {
          eRetVal = SM_SP_UMAX;
      }
//  }

//  if ( eSingDir != SM_SP_U )
//  {
      if ( UV().y < m_sDomain.GetVInterval().GetMin() + dBdryTol && sUVStep.y < -dPushingTol )
      {
          eRetVal = ( eRetVal == SM_SP_NEITHER ) ? SM_SP_VMIN : SM_SP_BOTH;
      }
      else if ( UV().y > m_sDomain.GetVInterval().GetMax() - dBdryTol && sUVStep.y >  dPushingTol )
      {
          eRetVal = ( eRetVal == SM_SP_NEITHER ) ? SM_SP_VMAX : SM_SP_BOTH;
      }
//  }

  if ( peWhichBound != NULL ) { *peWhichBound = eRetVal; }

  return ( eRetVal != SM_SP_NEITHER );

} // end SmSurfPtSolveStepper::PushingBoundary

/* ***************************************************************
PURPOSE: If a step is very near a boundary, and the nearby boundary
  has a better solution value, snap to that boundary.

NOTES: Returns True iff 'this' is changed.

METHOD --- If near a boundary, snap to that boundary, and check the solution.
*************************************************************** */
SmBoolean SmSurfPtSolveStepper::SnapToBoundaries( double dTol )
{
  SmPoint2d sUV( UV() );

  if ( ! m_sDomain.IsPoint2dOnBoundary( sUV, dTol ) )
    { return FALSE; }

  if (   m_sDomain.IsPoint2dOnBoundary( sUV, 0.0 ) )
    { return FALSE; }

  // Ok, we're within dTol of a boundary but not exactly on it.
  // Snap to the boundary(ies) and check if better.
  // Get a current reading:
  SmSrfPtErrorNormType eNormType = SM_SPN_GAP;
  if (   SolverOp() == SM_SO_NORMALIZE
      || SolverOp() == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
  {
      eNormType = SM_SPN_SUM_ABS;
  }
  double dCurrentError = ErrorNorm( eNormType );

  SmPoint2d sSaveUV( sUV );

  if ( sUV.x <= m_sDomain.GetMin().x + dTol )
    { sUV.x = m_sDomain.GetMin().x; }
  else if ( sUV.x >= m_sDomain.GetMax().x - dTol )
    { sUV.x = m_sDomain.GetMax().x; }

  if ( sUV.y <= m_sDomain.GetMin().y + dTol )
    { sUV.y = m_sDomain.GetMin().y; }
  else if ( sUV.y >= m_sDomain.GetMax().y - dTol )
    { sUV.y = m_sDomain.GetMax().y; }

  SetUV( sUV );

  double dBdryError = ErrorNorm( eNormType );

  SmBoolean bRetVal = TRUE;

  // If not better, restore original uv.
  if ( dBdryError > dCurrentError )  // If exactly equal, leave on bdry.
  {
      SetUV( sSaveUV );
      bRetVal = FALSE;
  }

  return bRetVal;

} // end SmSurfPtSolveStepper::SnapToBoundaries


/* ***************************************************************
PURPOSE: If a step is on a boundary, find the exact solution
  along that boundary curve.

NOTES: Returns True iff the returned value is a solution
  on the boundary, i.e., the closest point on the surface to the
  target point.  So if True, the result will be good for
  SM_SO_MINIMIZE, but not necessarily INTERSECT or NORMALIZE.
  (SM_SO_MAXIMIZE is not implemented.)

METHOD --- If on a boundary, drop to that boundary, and check the solution.
*************************************************************** */
SmStatus SmSurfPtSolveStepper::FindBoundarySolution(
        SmBoolean  & rbIsOBSolution // out:
    )
{
  rbIsOBSolution = FALSE;

  SmSurfParamType eWhichBound;

  if ( ! PushingBoundary( -m_dTightTol, &eWhichBound ) ) // Neg tol: if right on, do the check.
    {  return SM_SUCCESS; }

  this->SnapToBoundaries( m_dLooseTol ); // Changes uv only if better.

  // What if we're off both bounds, at a corner?
  // Then this should work either way, with u or v.
  if ( eWhichBound == SM_SP_BOTH )
  {
      // First check whether it's really pushing,
      // not just sitting exactly on the boundary.
      if ( PushingBoundary( m_dTightTol, &eWhichBound ) )
      {
          rbIsOBSolution = TRUE;
          return SM_SUCCESS;
      }

      // Just use u.  Set UMin or UMax.
      eWhichBound = ( UV().x < Domain().GetUInterval().GetMid() )
              ? SM_SP_UMIN : SM_SP_UMAX;
  }

  // Locals:

  SmSurfParamType eWhichParam;
  double dSurfParam, dGuessParam;
  SmVector3d sCrossDeriv1, sCrossDeriv2;
  SmExtent1d sIsoDomain,   sCrossDomain;

  if ( eWhichBound == SM_SP_UMIN )
  {
      // min-u edge, solve for v-parameter, at low-u edge.
      eWhichParam  = SM_SP_U;
      dSurfParam   = this->Domain().GetMin().x;
      dSurfParam   =
          this->SurfacePtr()->GetNaturalUVDomain().GetUInterval().ClampValue( dSurfParam );
      dGuessParam  = this->UV().y;
      sIsoDomain   = this->Domain().GetVInterval();
      sCrossDomain = this->Domain().GetUInterval();
  }
  else if ( eWhichBound == SM_SP_UMAX )
  {
      // max-u edge, solve for v-parameter, at high-u edge.
      eWhichParam  = SM_SP_U;
      dSurfParam   = this->Domain().GetMax().x;
      dSurfParam   =
          this->SurfacePtr()->GetNaturalUVDomain().GetUInterval().ClampValue( dSurfParam );
      dGuessParam  = this->UV().y;
      sIsoDomain   = this->Domain().GetVInterval();
      sCrossDomain = this->Domain().GetUInterval();
  }

  else if ( eWhichBound == SM_SP_VMIN )
  {
      // min-v edge, solve for u-parameter, at low-v edge.
      eWhichParam  = SM_SP_V;
      dSurfParam   = this->Domain().GetMin().y;
      dSurfParam   =
          this->SurfacePtr()->GetNaturalUVDomain().GetVInterval().ClampValue( dSurfParam );
      dGuessParam  = this->UV().x;
      sIsoDomain   = this->Domain().GetUInterval();
      sCrossDomain = this->Domain().GetVInterval();
  }
  else if ( eWhichBound == SM_SP_VMAX )
  {
      // max-v edge, solve for u-parameter, at high-v edge.
      eWhichParam  = SM_SP_V;
      dSurfParam   = this->Domain().GetMax().y;
      dSurfParam   =
          this->SurfacePtr()->GetNaturalUVDomain().GetVInterval().ClampValue( dSurfParam );
      dGuessParam  = this->UV().x;
      sIsoDomain   = this->Domain().GetUInterval();
      sCrossDomain = this->Domain().GetVInterval();
  }
  else
  {
      // not on a boundary.
      return SM_SUCCESS;
  }

  // Now solve for the curve.

  // More locals:
  SmPoint2d sNewUV( this->UV() );
  SmBoolean bFoundSolution;
  SmStatus  eStat;

  SmBoolean  bFromLeft = ( dSurfParam < sCrossDomain.GetMid() );
  SmIsoCurve sBdryCrv( *( this->SurfacePtr() ), eWhichParam, dSurfParam, bFromLeft );

  SmSolution sSol;

  // Can't use LocalPointSolve() for directed operations.
  if ( SolverOp() != SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
  {

      // Don't call DropPoint(): if its LocalSolve call fails, it will call
      // GlobalSolve, and that doesn't work for SmIsoCurve's, because of caching.

      eStat = sBdryCrv.LocalPointSolve(
          sIsoDomain,
          SM_SO_MINIMIZE,  // don't reject off-boundary solutions.
          this->TestPoint(),
          NULL, NULL, NULL,
          dGuessParam,
          bFoundSolution,
          sSol
      );

  }
  else
  {
      eStat = sBdryCrv.LocalPropertyAnalysis(
          sIsoDomain,
          SM_CP_PERPENDICULAR_TO_VECTOR,
          dGuessParam,
          NULL,
          &m_sDirVec,
          bFoundSolution,
          sSol
      );
  }

  if ( bFoundSolution == FALSE || eStat != SM_SUCCESS )
  {
      return SM_SUCCESS;
  }

  double dNewParam = sSol.m_vStart[0];

  // Make a temp eval: if it's better than SurfEval, set SurfEval to it.
  SmSurfPtSolveStepper sBdryEval( *this );

  if ( eWhichParam == SM_SP_U )
  {
      sNewUV.y = dNewParam;
      sBdryEval.SetUV( sNewUV );
      sCrossDeriv1 = sBdryEval.Su();
      sCrossDeriv2 = sBdryEval.Suu();
  }
  else
  {
      sNewUV.x = dNewParam;
      sBdryEval.SetUV( sNewUV );
      sCrossDeriv1 = sBdryEval.Sv();
      sCrossDeriv2 = sBdryEval.Svv();
  }

  // Note: always used to do this:
  // Note: reverse for MAX.
  SmSrfPtErrorNormType eNormType = ( eWhichParam == SM_SP_U ) ? SM_SPN_G_ABS : SM_SPN_F_ABS;
  if ( sBdryEval.ErrorNorm( eNormType ) < this->ErrorNorm( eNormType ) )
  {
      *this = sBdryEval;
  }

  // Check out of bounds.
  //cbi: just call PushingBoundary().
  // Check against the derivative that's pushing out of bounds.
  // Note, Gap() is ( srf pt - test pt )

  // To find out which direction it's pushing:
  // - for MIN and INTERSECT, we want to minimize 3d distance;
  // - for MAX we want to maximize 3d distance;
  // - for NORMALIZE we want to minimize the standard Newton-Raphson functional.
  // To min/max the 3d distance, we take a plane step, don't consider curvature.
  // In that case, the step direction would be determined by the dot product
  // of the gap vector with the cross derivative.
  // But for Normalize, the correct step could push away from the test point,
  // in particular, if the point is beyond the center of curvature.
  // In that case, we look at the direction of a true Newton step, which is
  // the NR functional divided by its derivative.  We really need only to see
  // the sign of the derivative, whether it would flip the direction of the step.

  SmBoolean bFlip = FALSE;
  if ( SolverOp() == SM_SO_MAXIMIZE )
  {
      bFlip = TRUE;  // Always move farther.
  }
  else if ( SolverOp() == SM_SO_NORMALIZE )
  {
      // Move farther iff the functional's deriv is negative:
      double dDerivTerm1 = sCrossDeriv1.Dot( sCrossDeriv1 );
      double dDerivTerm2 = this->Gap() .Dot( sCrossDeriv2 );
      bFlip = ( dDerivTerm1 + dDerivTerm2 < 0.0 );
  }

  double dError = this->Gap().Dot( sCrossDeriv1 );

  if ( bFlip )
      { dError = -dError; }

  if ( dSurfParam < sCrossDomain.GetMid() )
  {
      rbIsOBSolution = dError > -m_dTightTol; // was   this->TightTol(); try neg/10
  }
  else
  {
      rbIsOBSolution = dError <  m_dTightTol; // was  -this->TightTol(); try neg/10
  }

  return SM_SUCCESS;

}  // end SmSurfPtSolveStepper::FindBoundarySolution

/* ***************************************************************

DropToIsoCurve()

PURPOSE: Drop the test point to a given crease.

NOTES: Currently not used.
*************************************************************** */
SmBoolean SmSurfPtSolveStepper::DropToIsoCurve
       (SmSurfParamType eWhichParam, // in: one of SM_SP_U or SM_SP_V
        double dCreaseParamVal,
        double dParamOnCrease
    )
{
  //cbi todo: use an SmIsoCurve instead.
  SmBSplineCurve *pIsoCurve = NULL;
  m_pSrf->CreateIsoParametricCurve( *m_pSrf->GetContext(),
                                     eWhichParam,
                                     dCreaseParamVal,
                                     m_dLooseTol,
                                     pIsoCurve );

  if ( pIsoCurve == NULL )
    { return FALSE; }
  SmObjDelete sClean(pIsoCurve);

  SmExtent1d sDomain = ( eWhichParam == SM_SP_U )
      ? m_sDomain.GetVInterval()
      : m_sDomain.GetUInterval();

  // We have to check the iso-curve itself for having kinks.
  // [ Some of our unit tests... ]
  SmContinuityType eMinCont;
  SmTArray< SmContinuityType > sConts;
  pIsoCurve->CalculateContinuities( eMinCont, sConts );

  SmBoolean bGotIsoDrop = false, bGotThisDrop = false;
  double dDist;
  double dMinDist = SM_BIG_DOUBLE;
  double dMinParam = sDomain.GetMin();

  // Drop the point to each continuous section.
  // Loop on every knot after the first one.
  // If no kinks, this will catch only the entire interval.

  SmTArray< double > sKnots;
  SmTArray< ULONG  > sMults;
  pIsoCurve->GetKnots( sKnots, &sMults ); //cbi: not using sMults?
  SmExtent1d sIvl( sKnots[0], sKnots[0] );
  ULONG ii;
  for ( ii=1; ii<sConts.GetSize(); ii++ )
  {
      if ( sConts[ii] <= SM_CT_G1 )
      {
          sIvl.SetMinMax( sIvl.GetMin(), sKnots[ii] );

          // in case surface domain is smaller:
          sIvl.Intersect( sDomain, sIvl );

          double dGuessParam = sIvl.ClampValue( dParamOnCrease );

          pIsoCurve->DropPoint(sIvl,          // in : target curve allowed domain
                               m_sTestPt,     // in : Point to drop to curve
                               NULL,          // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                              //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                              //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                               m_dTightTol,   // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                              //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                              //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                              //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                               &dGuessParam,  // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                               bGotThisDrop,  // out: TRUE = found a drop point
                               dGuessParam,   // out: found drop curve param
                               dDist) ;       // out: found drop distance
                                              // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                              //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                              //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                              //      default:[SM_SO_MINIMIZE] to preserve original behavior

          if ( bGotThisDrop  &&  dDist < dMinDist )
          {
              bGotIsoDrop = TRUE;
              dMinDist    = dDist;
              dMinParam   = dGuessParam;
          }

          // Set up for next interval.
          sIvl.SetMinMax( sIvl.GetMax(), sIvl.GetMax() );
      }
  }

  if ( bGotIsoDrop )
  {
      SmPoint2d sCreaseUV;
      if ( eWhichParam == SM_SP_U )
      {
          sCreaseUV.Set( dCreaseParamVal, dMinParam );
      }
      else
      {
          sCreaseUV.Set( dMinParam, dCreaseParamVal );
      }

//cbi: here I have to check that it is indeed a local min,

      SetUV( sCreaseUV );
      return TRUE;
  }

  return FALSE;

} // end SmSurfPtSolveStepper::DropToIsoCurve

/* ***************************************************************
PURPOSE: Handle a crease in the surface.

NOTES: Currently not used.

 - This requires the B-Spline surface.  Returns False if not present.
 - Returns TRUE iff this Eval should be used as the next step.
 - The Boolean argument is returned True iff a crease was found,
   and an optimum solution is on the crease (set in this).
 - When moving from the wrong side of a crease to the right side,
   the error metric will quite likely increase.  The call must
   allow that, if this routine returns TRUE.
 - Assumes that creases occur only at internal multiple knots.
*************************************************************** */
SmBoolean SmSurfPtSolveStepper::CheckCrease(
              SmSurfPtSolveStepper & rPrevEval, // in
              SmBoolean  & rbFoundCreaseSolution  // out
    )
{
  rbFoundCreaseSolution = FALSE;

// Currently not used.
constexpr SmBoolean sbDoCreaseSubdomains = FALSE;
if ( ! sbDoCreaseSubdomains )
  { return FALSE; }


  // This routine requires the B-Spline surface.
  if ( m_pBSplSrf == NULL )
    { return FALSE; }

  // This is not yet implemented for this case:
  if ( m_eSolverOp == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
    { return FALSE; }

  // Unless we learn differently, C2 discontinuities should be ok --
  // what we're looking for is a sudden change in the surface normal.
  // This can happen only with knot multiplicity == degree.
  // (If mult == deg-1, we're guaranteed C1 continuity.)


  // Find a uv point on a crease, in between (or at) Curr and Next.
  // Design decision: check for only one crease between Curr & Next.
  SmPoint2d sCreaseUV;
  SmBoolean bGotCreaseUV = FALSE;
  SmSurfParamType eWhichParam = SM_SP_U;
  double dCreaseParamVal = 0.0, dParamOnCrease = 0.0;

  // First check whether Curr or Next is on a crease.
  // A common scenario is, Curr is on a crease but evaluating
  // the 'wrong' side; the step is in the right direction,
  // so Next properly steps off the crease into the right section,
  // but has a higher function value than Curr due to the way
  // normals line up, etc.

//  SmVector3d sNorm1, sNorm2;
//  m_pBSplSrf->EvaluateNormal( UV(), FALSE, FALSE, sNorm1 );
//  m_pBSplSrf->EvaluateNormal( UV(), TRUE,  TRUE,  sNorm2 );
//  if ( sNorm1.Dot( sNorm2 ) < 0.999847695156 ) // cosine of 1 deg

//  this->EvalAcrossCrease();



  ULONG ii, lDeg;
  SmTArray< double > aKnots;
  SmTArray< ULONG  > aMults;
  double dMin, dMax;

  // Find knots between and including Prev and this.
  // Search u and v separately, u first.

  if ( rPrevEval.UV().x < this->UV().x )
  {
      dMin = rPrevEval.UV().x;
      dMax = this->UV().x;
  }
  else
  {
      dMin = this->UV().x;
      dMax = rPrevEval.UV().x;
  }

  m_pBSplSrf->GetKnots( SM_SP_U, aKnots, &aMults );
  lDeg = m_pBSplSrf->GetDegree( SM_SP_U );
  SmExtent1d sInterval = m_sDomain.GetUInterval();
  double dTol = sInterval.GetLength() / 10000;

  // Looking for internal creases, don't check the end knots.
  for ( ii=1; ii<aKnots.GetSize() - 1; ii++ )
  {
      if ( !sInterval.ContainsValue( aKnots[ii], -dTol ) )
          { continue; }  // not in our domain of interest.

      if ( aKnots[ii] > dMax + SM_EFF_ZERO )
      {
          break;
      }

      if ( aKnots[ii] >= dMin )
      {
           if ( aMults[ii] >= lDeg ) // required for non-C1 jump.
           {
               dCreaseParamVal = aKnots[ii];

               // Find the uv-value at this knot line that's
               // on the straight line between Curr and Next.
               // Note, it's quite likely right on Curr or Next.
               double denom = this->UV().x - rPrevEval.UV().x;
               if ( smos_Fabs( denom ) < SM_EFF_ZERO )
               {
                   // ouch -- they're both on the crease.
                   // No problem, just use Next value.
                   dParamOnCrease  = this->UV().y;
               }
               else
               {
                   double dFrac = ( aKnots[ii] - rPrevEval.UV().x ) / denom;
                   dParamOnCrease =  rPrevEval.UV().y
                          + dFrac * ( this->UV().y - rPrevEval.UV().y );
               }
               sCreaseUV.Set( dCreaseParamVal, dParamOnCrease );
               bGotCreaseUV = TRUE;
               eWhichParam  = SM_SP_U;
               break;

           } // end if high knot multiplicity
      } // end if aKnots[i] between Curr and Next
  }  // end for loop on u-knots


  // Repeat for v-knots if we didn't find one in u-direction.
  if ( ! bGotCreaseUV )
  {
      if ( rPrevEval.UV().y < this->UV().y )
      {
          dMin = rPrevEval.UV().y;
          dMax = this->UV().y;
      }
      else
      {
          dMin = this->UV().y;
          dMax = rPrevEval.UV().y;
      }

      m_pBSplSrf->GetKnots( SM_SP_V, aKnots, &aMults );
      lDeg = m_pBSplSrf->GetDegree( SM_SP_V );

      sInterval = m_sDomain.GetVInterval();
      dTol = sInterval.GetLength() / 10000;

      for ( ii=1; ii<aKnots.GetSize() - 1; ii++ )
      {
          if ( !sInterval.ContainsValue( aKnots[ii], -dTol ) )
              { continue; }  // not in our domain of interest.

          if ( aKnots[ii] > dMax + SM_EFF_ZERO )
          {
              break;
          }

          if ( aKnots[ii] >= dMin )
          {
               dCreaseParamVal = aKnots[ii];

               if ( aMults[ii] >= lDeg ) // required for non-C1 jump.
               {
                   // Find the uv-value at this knot line that's
                   // on the straight line between Curr and Next.
                   // Note, it's quite likely right on Curr or Next.
                   double denom = this->UV().y - rPrevEval.UV().y;
                   if ( smos_Fabs( denom ) < SM_EFF_ZERO )
                   {
                       // ouch -- they're both on the crease.
                       // No problem, just new Next value.
                       dParamOnCrease = this->UV().x;
                   }
                   else
                   {
                       double dFrac = ( aKnots[ii] - rPrevEval.UV().y ) / denom;
                       dParamOnCrease =  rPrevEval.UV().x
                              + dFrac * ( this->UV().x - rPrevEval.UV().x );
                   }
                   sCreaseUV.Set( dParamOnCrease, dCreaseParamVal );
                   bGotCreaseUV = TRUE;
                   eWhichParam  = SM_SP_V;
                   break;

               } // end if high knot multiplicity
          } // end if aKnots[i] between Curr and Next
      }  // end for loop on v-knots
  } // end if need to check v-direction also.

  if ( ! bGotCreaseUV )
    { return FALSE; }  // We have nothing to offer.

  // cbi: could make the following a subroutine.

  // Check whether the crease point is a local minimum.
  // Evaluate 'above' and 'below' and see which way each would
  // try to move.  If 'below' tries to push upward, and 'above'
  // pushes downward, then we can't move off the crease.

  SmPoint3d sPt;
  SmVector3d sSu, sSv;
  SmVector3d sDerivLo, sDerivHi;
  m_pSrf->Evaluate1stDerivatives( sCreaseUV, FALSE, FALSE, sPt, sSu, sSv );
  sDerivLo = ( eWhichParam == SM_SP_U ) ? sSu : sSv;
  m_pSrf->Evaluate1stDerivatives( sCreaseUV,  TRUE,  TRUE, sPt, sSu, sSv );
  sDerivHi = ( eWhichParam == SM_SP_U ) ? sSu : sSv;
  SmVector3d sGap( sPt - m_sTestPt );

  if ( sGap.Length() <= m_dTightTol )  // Just in case...
  {
      this->SetUV( sCreaseUV );
      rbFoundCreaseSolution = TRUE;
      return TRUE;
  }

  // But: if the test point is right on the seam, and we're heading
  // towards it, then these vector tests can be misleading.
  // In that case, drop to the isocurve anyway, and check the results.
  // Note, the deriv in that direction should be the same above and below.
  SmVector3d sCreaseDir = ( eWhichParam == SM_SP_U ) ? sSv : sSu;
  sCreaseDir.Unitize();
  SmVector3d sGapDir( sGap );
  sGapDir.Unitize();
  double dDot = sGapDir.Dot( sCreaseDir );

  SmBoolean bDidCreaseDrop = FALSE;
  if ( smos_Fabs( dDot ) > 0.9 ) // about 25 degrees
  {
      bDidCreaseDrop = TRUE;
      if ( this->DropToIsoCurve( eWhichParam, dCreaseParamVal, dParamOnCrease ) )
      {
          rbFoundCreaseSolution = TRUE;
          return TRUE;
      }
  }

  SmBoolean bPushingLow  = sGap.Dot( sDerivLo ) >  m_dTightTol;
  SmBoolean bPushingHigh = sGap.Dot( sDerivHi ) < -m_dTightTol;


  if ( ! bPushingLow && ! bPushingHigh )
  {
      // The point on the crease is a local minimum.
      // We need to find the best solution along the crease.

      if ( ! bDidCreaseDrop )
      {
          if ( this->DropToIsoCurve( eWhichParam, dCreaseParamVal, dParamOnCrease ) )
          {
              rbFoundCreaseSolution = TRUE;
              return TRUE;
          }
      }
  }
  else if ( bPushingLow && ! bPushingHigh )
  {
      // Pushing away from the crease on the low side.
      // Set output slightly below the crease.
      if ( eWhichParam == SM_SP_U )
      {
          sCreaseUV.x -= 100 * SM_EFF_ZERO;
      }
      else
      {
          sCreaseUV.y -= 100 * SM_EFF_ZERO;
      }
      this->SetUV( sCreaseUV );
      return TRUE;
  }
  else if ( bPushingHigh && ! bPushingLow )
  {
      // Pushing away from the crease on the high side.
      // Set output slightly above the crease.
      if ( eWhichParam == SM_SP_U )
      {
          sCreaseUV.x += 100 * SM_EFF_ZERO;
      }
      else
      {
          sCreaseUV.y += 100 * SM_EFF_ZERO;
      }
      this->SetUV( sCreaseUV );
      return TRUE;
  }
  else if ( bPushingHigh && bPushingLow )
  {
      // We're on the inside (smaller) sector of a crease.
      // There is probably a solution in each direction,
      // we want the closer one.
      // TODO: Could we spawn a second search, tell the caller
      // to try two paths?
      // In the meantime, we'll just do it here ... lots of code,
      // but straightforwrd.
      // Method: take a few steps on each side,
      // and see which is a better solution.
      // Use plane steps and a distance criterion, because
      // we could be in unstable territory.

      SmBoolean bAdjusted;
      double dScale;

      SmSurfPtSolveStepper sHiEval1( rPrevEval ), sHiEval2( rPrevEval );
      SmSurfPtSolveStepper sLoEval1( rPrevEval ), sLoEval2( rPrevEval );
      if ( eWhichParam == SM_SP_U )
      {
          sCreaseUV.x -= 100 * SM_EFF_ZERO;
          sLoEval1.SetUV( sCreaseUV );
          sCreaseUV.x += 200 * SM_EFF_ZERO;
          sHiEval1.SetUV( sCreaseUV );
      }
      else
      {
          sCreaseUV.y -= 100 * SM_EFF_ZERO;
          sLoEval1.SetUV( sCreaseUV );
          sCreaseUV.y += 200 * SM_EFF_ZERO;
          sHiEval1.SetUV( sCreaseUV );
      }

      for ( ii=0; ii<4; ii++ )
      {
          // First step on the low side:
          sLoEval1.Step( sLoEval2, TRUE ); // TRUE: plane step

          sLoEval2.AdjustStep( sLoEval1, bAdjusted, dScale );

          if ( sLoEval2.Converged( TRUE ) )
          {
              *this = sLoEval2;
              rbFoundCreaseSolution = TRUE;
              return TRUE;
          }

          if ( sLoEval2.ErrorNorm() < sLoEval1.ErrorNorm() )
              { sLoEval1 = sLoEval2; }

          // Now the high side:
          sHiEval1.Step( sHiEval2, TRUE );

          sHiEval2.AdjustStep( sHiEval1, bAdjusted, dScale );

          if ( sHiEval2.Converged( TRUE ) )
          {
              *this = sHiEval2;
              rbFoundCreaseSolution = TRUE;
              return TRUE;
          }

          if ( sHiEval2.ErrorNorm() < sHiEval1.ErrorNorm() )
              { sHiEval1 = sHiEval2; }

      } // end for 4 iters, stepping both sides of the crease.

      // Return the best we have.
      if ( sLoEval1.ErrorNorm() < sHiEval1.ErrorNorm() )
          { *this = sLoEval1; }
      else
          { *this = sHiEval1; }

      if ( this->ErrorNorm() < rPrevEval.ErrorNorm() )
          { return TRUE; }

  } // end if pushing both low and high, away from crease.

  // Not good.

  return FALSE;

} // end SmSurfPtSolveStepper::CheckCrease

/*******************************************************************//**
PURPOSE: The main routine for taking a step towards the solution.

NOTES:
  This routine does a lot of work:
  - It checks whether the solution is on a boundary, and if so,
    iterates on the boundary for the solution.
  - If near a singularity, will find the solution there.
    (That functionality is below, in ::Step().)
  - It tries a Newton-Raphson step and checks how good it is.
    If not good enough, it compares with a plane step.
  - It will try line searches if appropriate.

 output: reReason : This is unchanged if an acceptable step was found.
     Currently the only other return values are
     SM_TR_OUT_OF_BOUNDS and SM_TR_UNABLE_TO_CONVERGE

IMPLEMENTATION NOTES ---
  "Plane step": This is a simplification of the standard Newton-Raphson step.
  Mathematically, it just ignores the second derivative terms in the Jacobian.
  Geometrically, it is the step that would be taken if the surface were a plane.
  When close to solution, it will not converge as well as the NR step,
  but it is more robust when outside the 'NR zone'.  In particular, a (small)
  plane step will always move physically closer to the test point.
  It is used when convergence indicates that it's not in the NR zone: NRSuspect.
***********************************************************************/
SmStatus SmSurfPtSolveStepper::FindNextStep(
        SmSurfPtSolveStepper & rPrevEval, // in
        SmTerminationReasonType & reReason // out:
    )
{
  // Init output.
  reReason = SM_TR_STILL_ITERATING; // default

  // First, check for being on a boundary, and pushing outward.
  // On the first iteration, though, if the guess is an actual solution,
  // (i.e., not off the surface normal), force one iteration:
  // same reason as in the serendipity check.

  SmBoolean bFoundOBSol = FALSE;
  SmSurfParamType eWhichBound = SM_SP_NEITHER;
  this->PushingBoundary( m_dTightTol, &eWhichBound );
  if ( m_iIter > 0 || eWhichBound != SM_SP_NEITHER )
  {
      if ( eWhichBound == SM_SP_BOTH )
      {
          // Pushing off a corner: we're not going to move anywhere.
          reReason = SM_TR_OUT_OF_BOUNDS;
          return SM_SUCCESS;
      }

      // Search along the boundary.
      this->FindBoundarySolution( bFoundOBSol );
      if ( bFoundOBSol )
      {
          reReason = SM_TR_FOUND_ANSWER_CONVERGED;
          if ( this->PushingBoundary( m_dTightTol ) )
            { reReason = SM_TR_OUT_OF_BOUNDS; }
          return SM_SUCCESS;
      }
  }

//cbi: these should be in the class:
  // Some iteration constants.

  constexpr int iMaxBoundaryHits = 3;

  // How much improvement an NR step should achieve
  // before we call it suspicious ...
  constexpr double dImprovementRatio = 0.75;

  // ... and how long before we start checking for it:
  // static int lImprovementIterCount = 5;

  SmStatus eStat;

  SmBoolean bGotNRStep    = FALSE;
  SmBoolean bGotPlaneStep = FALSE;

  // Get a Newton-Raphson step.

  // If we're not in the 'NR zone', then try a plane step.
  // Criteria for NOT being in the NR zone:
  // - 2nd terms of Fu or Gv are large negative relative to 1st terms;
  // - proposed step too big;
  // - function values not decreasing enough.
  // - unstable: near-zero determinant
  // So we need to evaluate at the NR step (if possible), before trying plane step.
  // Step() will set the flag in sNREval if appropriate.
  // If not, then we will check a couple of other things.

  this -> SetNRSuspect( FALSE );

  SmSurfPtSolveStepper sNREval( rPrevEval );
  SmBoolean bAdjusted;
  double dScale;

  reReason = sNREval.Step( rPrevEval, FALSE );  // FALSE: NR step.
  if ( reReason == SM_TR_FOUND_ANSWER_CONVERGED )
  {
      *this = sNREval;
      return SM_SUCCESS;
  }
  bGotNRStep = ( reReason == SM_TR_STILL_ITERATING );

  if ( bGotNRStep )
  {
      eStat = sNREval.AdjustStep( rPrevEval, bAdjusted, dScale );

      if ( eStat == SM_SUCCESS )
      {
          if ( sNREval.Converged( TRUE ) )
          {
              *this = sNREval;
              return SM_SUCCESS;
          }

          if ( bAdjusted && dScale < SM_EFF_ZERO_SQ )
          {
              // zero step, possibly pushing outwards from a corner
              //cbi:    bGotNRStep     = FALSE;
              this->SetNRSuspect( TRUE );
          }
      }
      else
      {
          // A real problem.
          this->SetNRSuspect( TRUE );
          bGotNRStep = FALSE;
      }
  }
  else
  {
      // Did not get a good NR step.
      this->SetNRSuspect( TRUE );
  }
  // end calculating the NR step.

  // One more check on whether we have to do a plane step:

  // Criteria: within the NR zone,
  // - step is not huge;
  // - improvement is good, unless nearly at solution.
  // Check those two things, and set bNRSuspect accordingly.

  SmBoolean bNRSuspect = this->NRStepSuspect() || sNREval.NRStepSuspect();
  // Check step size:
  if ( ! bNRSuspect )
  {
      SmVector2d sStep( sNREval.UV() - this->UV() );
      double dFactor = MaxStep() / 2.0;
      if (    fabs( sStep.x ) > dFactor * m_sDomain.GetUInterval().GetLength()
          ||  fabs( sStep.y ) > dFactor * m_sDomain.GetVInterval().GetLength()
         )
      { bNRSuspect = TRUE; }
  }
  // Check improvement:
  if ( ! bNRSuspect )
  {
      // In the NR zone, we'll be working with the functionals.
      if (    sNREval.ErrorNorm ( SM_SPN_SUM_ABS )
           > rPrevEval.ErrorNorm( SM_SPN_SUM_ABS ) * dImprovementRatio
         )
      {
          if ( sNREval.ErrorNorm ( SM_SPN_SUM_ABS ) > m_dLooseTol )
          { bNRSuspect = TRUE; }
      }
  }
  this->SetNRSuspect( bNRSuspect );


  // If we got a NR step that we like, that's good enough.
  if ( bGotNRStep && ! this->NRStepSuspect() )
  {
      *this = sNREval;
      return SM_SUCCESS;
  }

  // Well, something is not copacetic.
  // Try a planar step.
  this->SetNRSuspect( TRUE );

  SmSurfPtSolveStepper sPlaneEval( rPrevEval );

  reReason = sPlaneEval.Step( rPrevEval, TRUE );  // TRUE: plane step.
  if ( reReason == SM_TR_FOUND_ANSWER_CONVERGED )
  {
      *this = sNREval;
      return SM_SUCCESS;
  }
  bGotPlaneStep = ( reReason == SM_TR_STILL_ITERATING );

  if ( bGotPlaneStep )
  {
      eStat = sPlaneEval.AdjustStep( rPrevEval, bAdjusted, dScale );

      if ( eStat == SM_SUCCESS )
      {
          if ( sPlaneEval.Converged( TRUE ) )
          {
              *this = sPlaneEval;
              return SM_SUCCESS;
          }
      }
      else
      {
          // A real problem.
          bGotPlaneStep = FALSE;
      }
  }


  // Now we have attempted both NR step and plane step.
  // Sort out what we have.

  if ( ! bGotNRStep && ! bGotPlaneStep )
  {
      *this = rPrevEval;
      if (    sNREval.BoundHitsU() != 0
           || sNREval.BoundHitsV() != 0 )
      {
          reReason = SM_TR_OUT_OF_BOUNDS;
          return SM_SUCCESS; // Let the caller do what they want with this.
      }

      reReason = SM_TR_UNABLE_TO_CONVERGE;
      return SM_ERR_NOT_CONVERGING;  //cbi not sure of this
  }


  // Ok, we've got one or both trial steps.

  SmSrfPtErrorNormType eNormType = SM_SPN_GAP;
  if ( m_eSolverOp == SM_SO_NORMALIZE )
  {
      eNormType = SM_SPN_SUM_ABS;
  }

  // Check for out-of-bounds or creases.
  if ( bGotPlaneStep )
  {
      sPlaneEval.FindBoundarySolution( bFoundOBSol );
      if ( bFoundOBSol )
      {
          // But don't accept it if we have something better.
          if (      sPlaneEval.ErrorNorm( eNormType ) <= rPrevEval.ErrorNorm( eNormType )
              && ( ! bGotNRStep
                 || sPlaneEval.ErrorNorm( eNormType ) <= sNREval.ErrorNorm( eNormType ) ) )
          {
              *this = sPlaneEval;
              if ( PushingBoundary( m_dTightTol ) )
              {
                  reReason = SM_TR_OUT_OF_BOUNDS;
                  return SM_ERR_OUTSIDE_OF_DOMAIN;
              }
              else
              {
                  reReason = SM_TR_FOUND_ANSWER_CONVERGED;
                  return SM_SUCCESS;
              }
          }
      }

      if ( sPlaneEval.CheckCrease( rPrevEval, bFoundOBSol ) )
      {
          // But don't accept it if we have something better.
          if (       sPlaneEval.ErrorNorm( eNormType ) < rPrevEval.ErrorNorm( eNormType ) + SM_EFF_ZERO
              && ( ! bGotNRStep
                  || sPlaneEval.ErrorNorm( eNormType ) < sNREval.ErrorNorm( eNormType )   + SM_EFF_ZERO ) )
          {
              // CheckCrease returns True if it found and processed a crease.
              *this = sPlaneEval;
              if ( bFoundOBSol )
              {
                  // Found a local min/max on a crease: signal to stop iterating.
                  if ( PushingBoundary( m_dTightTol ) )
                  {
                      reReason = SM_TR_OUT_OF_BOUNDS;
                      return SM_ERR_OUTSIDE_OF_DOMAIN;
                  }
                  else
                  {
                      reReason = SM_TR_FOUND_ANSWER_CONVERGED;
                      return SM_SUCCESS;
                  }
              }

              // CheckCrease() didn't find solution, but set next eval on correct side
              // of crease, so keep iterating.
              reReason = SM_TR_STILL_ITERATING;
              return SM_SUCCESS;
          }
      }
  } // end if got plane step

  if ( bGotNRStep )
  {
      sNREval.FindBoundarySolution( bFoundOBSol );
      if ( bFoundOBSol )
      {
          // But don't accept it if we have something better.
          if (       sNREval.ErrorNorm( eNormType ) < rPrevEval.ErrorNorm( eNormType )  + SM_EFF_ZERO
              && ( ! bGotPlaneStep
                  || sNREval.ErrorNorm( eNormType ) < sPlaneEval.ErrorNorm( eNormType ) + SM_EFF_ZERO ) )
          {
              *this = sNREval;
              // Found a local min/max on a crease: signal to stop iterating.
              if ( PushingBoundary( m_dTightTol ) )
              {
                  reReason = SM_TR_OUT_OF_BOUNDS;
                  return SM_ERR_OUTSIDE_OF_DOMAIN;
              }
              else
              {
                  reReason = SM_TR_FOUND_ANSWER_CONVERGED;
                  return SM_SUCCESS;
              }
          }
      }

      if ( sNREval.CheckCrease( rPrevEval, bFoundOBSol ) )
      {
          // But don't accept it if we have something better.
          if (       sNREval.ErrorNorm( eNormType ) <= rPrevEval.ErrorNorm( eNormType )
              && ( ! bGotPlaneStep
                  || sNREval.ErrorNorm( eNormType ) <= sPlaneEval.ErrorNorm( eNormType ) ) )
          {
              // CheckCrease returns True if it found and processed a crease.
              *this = sNREval;
              if ( bFoundOBSol )
              {
                  // Found a local min/max on a crease: signal to stop iterating.
                  if ( PushingBoundary( m_dTightTol ) )
                  {
                      reReason = SM_TR_OUT_OF_BOUNDS;
                      return SM_ERR_OUTSIDE_OF_DOMAIN;
                  }
                  else
                  {
                      reReason = SM_TR_FOUND_ANSWER_CONVERGED;
                      return SM_SUCCESS;
                  }
              }

              // CheckCrease() didn't find solution, but set next eval on correct side
              // of crease, so keep iterating.
              reReason = SM_TR_STILL_ITERATING;
              return SM_SUCCESS;
          }
      }
  } // end if got NR step


  // At this point, we know we're having problems,
  // so try line searches.
  // Do a line search on both before comparing.
  if ( bGotNRStep )
  {
      sNREval.LineSearch( rPrevEval );
  }
  if ( bGotPlaneStep )
  {
      sPlaneEval.LineSearch( rPrevEval );
  }

  // Now set *this to the better of NR and Plane steps.

  if ( ! bGotNRStep && ! bGotPlaneStep )
  {
      SM_ASSERT( FALSE ); // This should never happen.

      *this = rPrevEval;
      if (    sNREval.BoundHitsU() != 0
           || sNREval.BoundHitsV() != 0 )
      {
          reReason = SM_TR_OUT_OF_BOUNDS;
          return SM_SUCCESS; // Let the caller do what they want with this.
      }

      reReason = SM_TR_UNABLE_TO_CONVERGE;
      return SM_ERR_NOT_CONVERGING;  //cbi not sure of this
  }
  else if ( bGotNRStep && ! bGotPlaneStep )
  {
      *this = sNREval;  // it's all we've got
  }
  else if ( ! bGotNRStep && bGotPlaneStep )
  {
      *this = sPlaneEval;  // it's all we've got
  }
  else  // have both steps, use the better one.
  {

      if ( sNREval.ErrorNorm( eNormType ) < sPlaneEval.ErrorNorm( eNormType ) )
      {
          *this = sNREval;
      }
      else
      {
          *this = sPlaneEval;
      }
  }

  if (    smos_Iabs( this->BoundHitsU() ) > iMaxBoundaryHits
       || smos_Iabs( this->BoundHitsV() ) > iMaxBoundaryHits )
  {
      reReason = SM_TR_OUT_OF_BOUNDS;
      return SM_SUCCESS; // Let the caller do what they want with it.
  }

  // Now we've decided on a step.

  // After all that, check that this step is ok.
  // If not, there's nothing more we can do.
  if (     this->ErrorNorm( eNormType ) >= rPrevEval.ErrorNorm( eNormType )
      || ( this->UV() - rPrevEval.UV() ).Length() < SM_EFF_ZERO_SQ )
  {
      if (    smos_Iabs( this->BoundHitsU() ) > 1
           || smos_Iabs( this->BoundHitsV() ) > 1 )
      {
          reReason = SM_TR_OUT_OF_BOUNDS;
          return SM_SUCCESS; // Let the caller figure it out.
      }

      reReason = SM_TR_UNABLE_TO_CONVERGE;
      return SM_ERR_NOT_CONVERGING;  //cbi not sure of this
  }

  return SM_SUCCESS;

} // end SmSurfPtSolveStepper::FindNextStep()

/*******************************************************************//**
PURPOSE: Dump.

NOTES:
***********************************************************************/
void SmSurfPtSolveStepper::Dump( int iDumpLevel, const TCHAR *cMsg, SmStatus eStat )
{
    TCHAR sBuff[SM_TBLOCK_SIZE];

    if ( iDumpLevel < 0 ) return;

    if ( cMsg != NULL )
        smos_WriteBuffer( cMsg );

    if ( eStat != SM_SUCCESS )
    {
        switch ( eStat )
        {
          case SM_ERR_NOT_CONVERGING: {
            smos_sprintf( sBuff,_T("%s"), _T("  Status is SM_ERR_NOT_CONVERGING") );
            break;
          }
          case SM_ERR_OUTSIDE_OF_DOMAIN: {
            smos_sprintf( sBuff,_T("%s"), _T("  Status is SM_ERR_OUTSIDE_OF_DOMAIN") );
            break;
          }
          default: {
            smos_sprintf( sBuff, _T("  Status is %ld"), eStat );
          }

        }
        smos_WriteBuffer(sBuff);
    }

    Eval();
    smos_sprintf(sBuff, _T("\nSmSurfPtSolveStepper Dump iter %3d       U %18.14f V %18.14f"),
        m_iIter, UV().x, UV().y );
    smos_WriteBuffer(sBuff);
    smos_sprintf(sBuff, _T("\n   FVal %18.14f GVal %18.14f"),
        m_dFVal, m_dGVal );
    smos_WriteBuffer(sBuff);
    smos_sprintf(sBuff, _T("\n   SumF %18.14f Dist %18.14f"),
        smos_Fabs(m_dFVal)+smos_Fabs(m_dGVal), m_dGapLen );
    smos_WriteBuffer(sBuff);

    if ( iDumpLevel < 1 ) return;

    // Dump more if desired...

} // end SmSurfPtSolveStepper::Dump


/*******************************************************************//**
PURPOSE: Given a 3d position and a uv guess point,
   refine the uv to solve the specified operation.

NOTES:
  This is the newer of two implementations of SmSurface::LocalPointSolve.
  See Usage Notes for that method as well.

  For SM_SO_MINIMIZE and SM_SO_MAXIMIZE, this will return the closest
  point on a boundary, if the point is not on the surface normal
  within the domain.  It will always return a solution unless the
  result is a local maximum (MIN case) or local minimum (MAX case).

  This version is not completely implemented for the SM_SO_MAXIMIZE case;
  the base version will always call the older version for that.

  Returns a solution:
                  Off surface,    Outside domain
    Solver Op:     on normal:     (off normal; out of bounds):
  SM_SO_MINIMIZE      Yes             Yes: Closest point on boundary
<<SM_SO_MAXIMIZE      Yes             Yes: Farthest point on boundary>>
  SM_SO_NORMALIZE     Yes             No
  SM_SO_INTERSECT     No              No

- If the system does not converge, results will be set to best values of
  any iteration, which in practice are usually fairly close, and usable.
  Whether to use the results depends on the application;
  the caller can check just how bad the result is, and
  whether it's important.

IMPLEMENTATION NOTES ---
- Function values: the values used for stepping are not exactly
  the same as the quantities used to test convergence and to compare
  steps (ErrorNorm()). The convergence/comparison criterion is the sum
  of the dot products of the gap vector with the unitized derivatives.
  For function values for the iteration, however, we don't divide by the
  magnitudes: it's not necessary (because it will converge to the proper
  solution either way), and it would make the function derivatives (the
  Jacobian) MUCH more expensive to calculate.

***********************************************************************/
SmStatus SmSurface::LocalPointSolve_1
  (const SmExtent2d     & crUVDomain,            // in : Surface Domain limit
   SmSolverOperationType  eSolverOperation,      // in : one of SM_SO_MINIMIZE,
                                                 //             SM_SO_MAXIMIZE,
                                                 //             SM_SO_NORMALIZE,
                                                 //             SM_SO_INTERSECT.
   const SmPoint3d      & crTestPoint,           // in : Target Point
   const SmPoint2d      & crUVGuess,             // in : UV Guess Point
   SmBoolean            & rbFoundSolution,       // out: TRUE = Found a Point
   SmSolution           & rSolution,             // out: Contains UV Found Point

   SmTerminationReasonType &cbi_Reason,
   int &cbi_IterCount
  )
 const
{

cbi_Reason = SM_TR_UNABLE_TO_CONVERGE;
cbi_IterCount = 0;

  // check input
  SM_ASSERT(   eSolverOperation == SM_SO_MINIMIZE
            || eSolverOperation == SM_SO_MAXIMIZE
            || eSolverOperation == SM_SO_NORMALIZE
            || eSolverOperation == SM_SO_INTERSECT
            || eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE );

#ifdef SM_DEBUG_CODE
static constexpr int iDebugLevel = -1;
SmBoolean bDebugMe = FALSE;
  if (bDebugMe)
    {
      SmFace *pFace = (SmFace *)GetFace();
      SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL;
      SmPoint3d sGuessPoint;
      EvaluatePoint(crUVGuess, sGuessPoint);

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,0,1); if(pBrep) pBrep->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,1,0); DrawUV(3,3); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); if(pFace) pFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
      smgfx_SetLook(5,6, 1,0,0); crTestPoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(5,8, 0,1,0); sGuessPoint.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  // Clamp input guess.
  // Not an error, just clamp.
  SmPoint2d sUVGuessClamped = crUVDomain.ClampPoint2d( crUVGuess );

  // Init return values.
  rbFoundSolution = FALSE;
  rSolution.m_eSolutionType = SM_ST_SINGLE_VALUE; // This will always be.
  rSolution.m_vStart[0]     = sUVGuessClamped.x;
  rSolution.m_vStart[1]     = sUVGuessClamped.y;
  rSolution.m_lNumObjects   = 1 ;
  rSolution.m_apObjects[0]  = (SmSurface*)this ;

  // We have to enable out-of-bounds evaluations.
  // Otherwise, the evaluators will silently clamp,
  // leading to wrong answers.  (Can happen if too-big domain passed in.)
  SmBoolean bTmp0 = FALSE ; 
  SmBSplineSurface * pBSplineSurface0 = SM_CAST_NONNULL_PTR(SmBSplineSurface, this) ; 
  SmTemporaryChangeValue<SmBoolean> sClean0(pBSplineSurface0 ? pBSplineSurface0->GetOutOfBoundsEnabled() : bTmp0, TRUE) ;

  // Set up some constants.
  int iMaxIter = 35;  // actually very generous.

  // Tolerance: tight, but it should be achievable.
  // We'll use this for both spatial distance, and angular; see notes above.
  double dTightTol = SM_EFF_ZERO * (1.0 + crTestPoint.GetMaxDimension());

  // We need two other tolerances.
  // For INTERSECT, the point must be 'on' the surface.
  // For NORMAL, must be 'on' the surface normal (i.e., not out of bounds)
  // No tolerances are passed in.
  // We'll use the old criterion for NORMALIZE:

  double dLooseTol;
  if ( GetOwner() )
  { dLooseTol = SmTol::GetZoneTol3d( GetOwner() ); }
  else
  {
      dLooseTol = SM_EFF_ZERO_SQRT * 100;
      if ( eSolverOperation == SM_SO_INTERSECT )
      { dLooseTol = 0.01; } // cbi: mainly for EvaluateBinormal(); let the caller decide.
  }

  // -----------------------

  // Start of executable code---
  SmSurfPtSolveStepper sCurrEval( this,
                                  crUVDomain,
                                  crTestPoint,
                                  sUVGuessClamped,
                                  dTightTol,
                                  dLooseTol,
                                  eSolverOperation );

  rSolution.m_vStart.m_dSolutionValue = sCurrEval.GapLength();

  // First serendipity check: check the given guess.
  // Note, make this a really tight tolerance, so that if it's
  // very close but not exactly on, we take at least one step.

  if ( sCurrEval.Converged( SM_EFF_ZERO / 1000 ) )
  {
      // If we're near a singularity, don't trust this.
      // TODO: put a check into Converged().
      if ( ! sCurrEval.IsSingular()
          || sCurrEval.GapLength() < sCurrEval.TightTol() )
      {

#ifdef SM_DEBUG_CODE
          sCurrEval.Dump( iDebugLevel,
              _T("\nLocalPointSolve: starting guess was good" ));
#endif

          cbi_Reason = SM_TR_FOUND_ANSWER_CONVERGED;

          rbFoundSolution = TRUE;
          return SM_SUCCESS;
      }
  }

#ifdef SM_DEBUG_CODE
  sCurrEval.Dump( iDebugLevel, _T("\nLocalPointSolve: Enter:") );
#endif

  // Time to iterate.
  // Locals for the iteration:
  int iterCount  = 0;
  SmStatus eStat = SM_SUCCESS;
  SmTerminationReasonType eReason = SM_TR_STILL_ITERATING;
  SmSurfParamType eWhichBound;

  // Currently not used:
  // // If a surface has interior creases, standard NR iteration
  // // can converge to an incorrect solution, or not converge at all.
  // // The only way to deal with this is to double-check each NR step
  // // with a plane step.
  // SmTArray< double > aCreasesU;
  // SmTArray< double > aCreasesV;
  // this->FindKnotsByMaxContinuity( SM_CT_C0, FALSE, aCreasesU, aCreasesV );
  // //cbi: double-check each crease: compare normals on each side; just one eval in middle.
  // if ( aCreasesU.GetSize() > 0 || aCreasesV.GetSize() > 0 )
  //     { sCurrEval.SetNRSuspect( TRUE ); }

  SmSurfPtSolveStepper sNextEval( sCurrEval );

  for ( iterCount = 0; iterCount < iMaxIter; iterCount++ )
  {
      // State at top of loop:
      // We have the function evaluations,
      // and we know we're not at solution yet.

      eStat = sNextEval.FindNextStep( sCurrEval, eReason );

      // Check convergence.
      if ( sNextEval.Converged( TRUE ) )
      {
#ifdef SM_DEBUG_CODE
          sNextEval.Dump( iDebugLevel,
              _T("\nLocalPointSolve: Step() Converged:"), eStat );
#endif

          eReason = SM_TR_FOUND_ANSWER_CONVERGED;
          break;
      }
      else if ( sNextEval.ConvergedOnBoundary( sNextEval.TightTol(), &eWhichBound ) )
      {
          sNextEval.SnapToBoundaries( sNextEval.LooseTol() );
#ifdef SM_DEBUG_CODE
          sNextEval.Dump( iDebugLevel,
              _T("\nLocalPointSolve: Step() Converged on Boundary:"), eStat );
#endif

          // If the solution is still off, check for a seam to jump
          if ( sNextEval.GapLength() > sNextEval.LooseTol() )
          {
              SmPoint2d sNewPoint( sNextEval.UV() );
              SmSurfParamType eOnSeam;
              SmTArray<SmPoint2d> sCrossSeamPoints;
              SmBoolean bOnSeam = IsOnSeam( sNextEval.UV(), NULL, &eOnSeam, &sCrossSeamPoints );

              // Coarsen eWhichBound
              eWhichBound = ( eWhichBound == SM_SP_VMIN || eWhichBound == SM_SP_VMAX ) ? SM_SP_V :
                  ( eWhichBound == SM_SP_UMIN || eWhichBound == SM_SP_UMAX ) ? SM_SP_U : eWhichBound;

              // If the solution is pushing a seam and still far from tolerance, try jumping the seam
              if ( bOnSeam )
              {
                  sNewPoint = ( eOnSeam     != SM_SP_BOTH && ( eOnSeam == eWhichBound || eWhichBound == SM_SP_BOTH ) ) ? sCrossSeamPoints[0] : // On one seam, pushing that same seam 
                              ( eOnSeam     != SM_SP_BOTH &&   eOnSeam != eWhichBound ) ? sNextEval.UV() :                                     // On one seam, not pushing that seam
                              ( eWhichBound != SM_SP_BOTH ) ? sCrossSeamPoints[eWhichBound] : sCrossSeamPoints[2];                             // on Both seams, pushing either or both
              }

              // If we jumped the seam, set the new UV and do another iteration
              if ( sNewPoint != sNextEval.UV() )
              { sNextEval.SetUV( sNewPoint ); }
              // Else we done solving
              else
              {
                  eReason = SM_TR_OUT_OF_BOUNDS;
                  break;
              }
          }
          else
          {
              eReason = SM_TR_OUT_OF_BOUNDS;
              break;
          }
      }

      if ( eStat != SM_SUCCESS  ||  eReason != SM_TR_STILL_ITERATING )
      {
#ifdef SM_DEBUG_CODE
          sNextEval.Dump( iDebugLevel,
              _T("\nLocalPointSolve: Step() breaking iteration:"), eStat );
#endif
          break;
      }


      // Update for next iteration.
      sCurrEval = sNextEval;
      sCurrEval.SetIterCount( iterCount+1 );

#ifdef SM_DEBUG_CODE
      sCurrEval.Dump( iDebugLevel, _T("\nLocalPointSolve: next step:") );
#endif

  }  // end iteration


  // Load results (best achieved, which are in sNextEval) and return.
  // The solution value that we return is the distance between
  // the test point and the surface point.

  rSolution.m_vStart[0] = sNextEval.UV().x;
  rSolution.m_vStart[1] = sNextEval.UV().y;
  rSolution.m_vStart.m_dSolutionValue =
      ( eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
          ? sNextEval.ErrorNorm( SM_SPN_SUM_ABS )
          : sNextEval.ErrorNorm( SM_SPN_GAP );

  // Set rbFoundSolution for return.
  // At this point, if eReason is CONVERGED, then we got a tight solution.
  // Otherwise, check for a close solution.
  // Use the same tols as the earlier version, for compatibility.
  // Note, for INTERSECT, the old solver didn't check spatial proximity,
  // the caller did that.

  rbFoundSolution = sNextEval.Converged( FALSE ); // loose tol

  if ( ! rbFoundSolution )
  {
      if (   eSolverOperation == SM_SO_MINIMIZE
          || eSolverOperation == SM_SO_MAXIMIZE
          || eSolverOperation == SM_SO_3D_SIGNED_DIRECTED_MAXIMIZE )
      {
          // always:      if ( eReason == SM_TR_OUT_OF_BOUNDS )
          {
              rbFoundSolution = TRUE;
          }
      }

      // Also set output flag properly.
      if ( iterCount >= iMaxIter && eReason != SM_TR_OUT_OF_BOUNDS )
      {
          eReason = SM_TR_UNABLE_TO_CONVERGE;
      }
  }


#ifdef SM_DEBUG_CODE
  sNextEval.Dump( iDebugLevel, _T("\nLocalPointSolve: Leaving:"), eStat );
#endif

cbi_IterCount = iterCount;
cbi_Reason = eReason;

  return SM_SUCCESS;

}  // end LocalPointSolve_1
