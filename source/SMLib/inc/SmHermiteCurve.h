// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmHermiteCurve.h
* PURPOSE: Header file for cubic Hermite curve.
**********************************************************************/

#ifndef __SMHERMITECURVE_H__
#define __SMHERMITECURVE_H__

#ifndef __SMCURVE_H__
#include <SmCurve.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

/*******************************************************************//**
PURPOSE: The SmHermiteCurve class defines a cubic curve which is defined
    by two points with two corresponding derivative vectors. This class is 
    intended only for use within the numerical algorithms.

NOTES: 

   let H(t) =  ( 2*t**3 - 3*t**2 + 1) * P0   for t:[0.0 1.0]
             + (   t**3 - 2*t**2 + t) * M0
             + (-2*t**3 + 3*t**2 + 0) * P1
             + (   t**3 -   t**2 + 0) * M1

       H'(t) =  ( 6*t**2 - 6*t + 0) * P0   for t:[0.0 1.0]
              + ( 3*t**2 - 4*t + 1) * M0
              + (-6*t**2 + 6*t + 0) * P1
              + ( 3*t**2 -   t + 0) * M1

***********************************************************************/
class SM_EXPORT SmHermiteCurve : public SmCurve
{
 private:
  SmPoint3d        m_vP1;  // Initial point
  SmVector3d       m_vD1;  // Initial direction
  SmPoint3d        m_vP2;  // Final point
  SmVector3d       m_vD2;  // Final direction
  SmVector3d       m_vC;   // C of power basis eqn.
  SmVector3d       m_vD;   // D of power basis eqn.

 public:
  // constructors
  SmHermiteCurve(const SmPoint3d  & crP1, 
                 const SmVector3d & crV1,
                 const SmPoint3d  & crP2,
                 const SmVector3d & crV2,
                 ULONG              lDimension=3) ;

  SmHermiteCurve(const SmPoint2d  & crP1, 
                 const SmVector2d & crV1,
                 const SmPoint2d  & crP2,
                 const SmVector2d & crV2,
                 ULONG              lDimension=2) ;

  SmHermiteCurve(ULONG                  lDimension,
                 const SmHermiteCurve & crHermiteToOffset,
                 const SmVector3d     & crOffsetPlaneNormal,
                 double dOffsetDistance) ;
  
  // empty constructor for I/O
  SmHermiteCurve(ULONG lDim=3) : SmCurve(lDim) { }
  
  // destructor
  virtual ~SmHermiteCurve() { }
  
  // equality operator
  virtual SmBoolean operator==(const SmCurve &crOther) const;
  
  virtual SmStatus CalculateContinuities(SmContinuityType           & reMinContinuityInCurve,      // out: 
                                         SmTArray<SmContinuityType> & rContinuitiesAtKnots,        // out: 
                                         double dContinuityAngleTol = SM_CONTINUITY_ANGLE) const;  // NotUsed: in : 
  
  virtual SmStatus CalculateBoundingBox(const SmExtent1d & crInterval,               // NotUsed: in : 
                                        SmExtent3d       * pNormalBox = NULL,        // out: 
                                        SmPseudoBox      * pPseudoBox = NULL,        // out: 
                                        SmPolarBox       * pPolarBox  = NULL,        // out: 
                                        SmBoolean          bExpandBox = TRUE) const; // NotUsed: in : 
  
  double ComputeChordHeight() const;
  
  double ComputeAngleSpannedInDegrees() const;
  
  void GetBezierPoints(SmPoint3d & rP1, 
                       SmPoint3d & rP2,
                       SmPoint3d & rP3,
                       SmPoint3d & rP4) const;
  
  virtual SmExtent1d GetNaturalInterval() const { return SmExtent1d(0.0,1.0); }
  
  virtual SmStatus   GetKnots(SmTArray<double> & rKnots,                       // out: Unique knot vector                                         
                              SmTArray<ULONG>  * pKnotMultiplicities = NULL,   // out: multiplicity value for each knot                           
                              const SmExtent1d * pOptIvl = NULL) const;        // in : interval of interest, NULL=Natural Interval, default:[NULL]
  
  virtual SmStatus Evaluate(double     dParameter,              // in : tgt param
                            ULONG      lNumDerivatives,         // in : 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .
                            SmBoolean  bFromLeft,               // in : if P is on interval boundary
                                                                //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                                                //      FALSE = evaluate P in lower interval where P is on the right of the interval
                            SmVector3d aPointAndDerivatives[],  // out: (pos, tang, 2nd, ...) sized:[lNumDerivatives+1]
                            SmBoolean  bNonZeroTangents=TRUE)   // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                              const;                            //      FALSE= return exact tangent values
                                                                //      note: Surprisingly TRUE is the common choice because most tangent uses
                                                                //            are for their direction (Binorm, SurfNorm comps), but when the 
                                                                //            tangent is being used for its magnitude (like an arc-length comp)
                                                                //            then set this to FALSE.
                                                                //      default:[TRUE]
  
  virtual SmStatus EvaluatePoint(double dParameter, SmPoint3d & rPoint) const;
  
  SmStatus GetBSplineControlPolygon(SmTArray<SmPoint3d> &rPoly) const ;
  virtual SmBSplineCurve * GetRootCurve()        const { return( NULL ) ; } /* When available return equivalent BSplineCurve */ 
  
  // get Curve Memory size - not its attributes
  virtual ULONG GetMemoryUsed( ULONG & rlMemoryAllocated,                 // out: bigger size of all allocated memory in bytes
                               SmMarkType eMarkType = SM_MT_NOMARK) const // in : uses without increment eMarkType value
                             { SM_REF1(eMarkType) ;
                               ULONG lThisAllocated;
                               ULONG lUsed = rlMemoryAllocated = sizeof(*this) ;
                               
                               // + cache memory
                               if(m_pCacheObj)
                                 { lUsed += m_pCacheObj->GetMemoryUsed( lThisAllocated );
                                   rlMemoryAllocated += lThisAllocated;
                                 }
                               return(lUsed) ;
                             }
  
  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                          //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                          //      default:[SM_LEVEL_0] 
                                SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                               const ;
  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
  
  virtual SmStatus WriteToDB (SmDatabaseIO & rDB,                      // in : target output stream
                              ULONG          lDBVersionNumber)         // NotUsed: in : database version to get proper sequence of writes  
                              const ;

  static  SmStatus ReadFromDB(SM_TYPE           lType,              // NotUsed: in : Object type to be read
                              SmDatabaseIO    & rDB,                // in : target output stream
                              ULONG             lDim,               // NotUsed: in : curve image space dim, 2 or 3                                                                                    
                              const SmContext & crContext,          // in : context for new object construction
                              SmCurve         *&rpNewCurve,         // out:    NULL on input = new object allocated in this routine built from stream data
                                                                    //      NotNULL on input = pointer to an empty object to be filled by this routine
                              ULONG             lDBVersionNumber) ; // NotUsed: in : database version to get proper sequence of writes
  
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmHermiteCurve,SmCurve,SmHermiteCurve_TYPE);
  
} ; // end class SmHermiteCurve

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmHermiteCurve::SmHermiteCurve
  (const SmPoint3d  & crP1, 
   const SmVector3d & crD1,
   const SmPoint3d  & crP2,
   const SmVector3d & crD2,
   ULONG lDimension)
 : SmCurve(lDimension),
   m_vP1(crP1), 
   m_vD1(crD1), 
   m_vP2(crP2), 
   m_vD2(crD2) 
{
    // Store C and D of power form
   m_vC = -3.0*m_vP1 - 2.0*m_vD1 + 3.0*m_vP2 - m_vD2;
   m_vD =  2.0*m_vP1 +     m_vD1 - 2.0*m_vP2 + m_vD2;

} // end SmHermiteCurve::SmHermiteCurve

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmHermiteCurve::SmHermiteCurve
  (const SmPoint2d  & crP1, 
   const SmVector2d & crD1,
   const SmPoint2d  & crP2,
   const SmVector2d & crD2,
   ULONG lDimension)
 : SmCurve(lDimension),
   m_vP1(crP1), 
   m_vD1(crD1), 
   m_vP2(crP2), 
   m_vD2(crD2) 
{
    // Store C and D of power form
   m_vC = -3.0*m_vP1 - 2.0*m_vD1 + 3.0*m_vP2 - m_vD2;
   m_vD = 2.0*m_vP1 + m_vD1 - 2.0*m_vP2 + m_vD2;

} // end SmHermiteCurve::SmHermiteCurve


/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline void SmHermiteCurve::GetBezierPoints
  (SmPoint3d & rP1, 
   SmPoint3d & rP2,
   SmPoint3d & rP3,
   SmPoint3d & rP4) 
  const
{
    rP1 = m_vP1;
    rP2 = m_vP1 + m_vD1 / 3.0;
    rP3 = m_vP2 - m_vD2 / 3.0;
    rP4 = m_vP2;

} // end SmHermiteCurve::GetBezierPoints


#endif // !__SMHERMITECURVE_H__



