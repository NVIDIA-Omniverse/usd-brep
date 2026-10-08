// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmOffsetCurve.h
* PURPOSE: Header file for implicit Offset curve.
**********************************************************************/

#ifndef __SMOFFSETCURVE_H__
#define __SMOFFSETCURVE_H__

#ifndef __SMCURVE_H__
#include <SmCurve.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

/*******************************************************************//**
PURPOSE: The SmOffsetCurve class defines a curve which is an implicit
    offset of another curve.  The offset is defined by the curve, a
    normal to the offset plane and a distance.  Note that a positive
    distance produces an offset to the right hand side of the curve as
    viewed from above the offset plane.

NOTES: 
***********************************************************************/
class SM_EXPORT SmOffsetCurve : public SmCurve
{
 protected:
  SmCurve        * m_pCurve = NULL ;                      // base curve
  SmVector3d       m_vOffsetPlaneNorm;                    // Normal to the plane containing the OffsetCurve
                                                          // Positive values offset to the right when viewed from above the OffsetPlane
                                                          // Negative values offset to the left
  double           m_dOffsetDistance = SM_UNDEF_DOUBLE ;  // dist from BaseCurve to OffsetCurve
  SmExtent1d       m_vCurveInterval;                      // domain of the BaseCurve valid for this offset
  SmBoolean        m_bIsReversed = UNSURE ;               // TRUE =
                                                          // FALSE=
  SmBoolean        m_bOwnsCurve = UNSURE ;                // TRUE = delete m_pCurve in destructor, 
                                                          // FALSE = don't
  
 public:
  // constructor
  SmOffsetCurve(ULONG              lDimension,
                const SmCurve    & crCurve,
                const SmExtent1d & crCurveInterval,
                const SmVector3d & crOffsetPlaneNorm,
                double             dOffsetDistance,
                SmBoolean          bOwnsCurve=FALSE) ;
  
  // empty constructor for I/O
  SmOffsetCurve() : SmCurve(3) { }
  
  // copy constructor
  SmOffsetCurve(const SmOffsetCurve & crCurveToCopy) ;
  
  // copy
  virtual SmStatus Copy(const SmContext & crContext,
                        SmCurve        *& rpNewCurve)
                       const ;
  
  // destructor
  virtual ~SmOffsetCurve()  { if(m_bOwnsCurve) {delete m_pCurve ; m_pCurve = NULL ; } }
  
  // equality operator
  virtual SmBoolean operator==(const SmCurve &crOther) const ;
  
  // evaluators
  virtual SmStatus Evaluate(double     dParameter,              // in : tgt param
                            ULONG      lNumDerivatives,         // in : 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .
                            SmBoolean  bFromLeft,               // in : if P is on interval boundary
                                                                //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                                                //      FALSE = evaluate P in lower interval where P is on the right of the interval
                            SmVector3d aPointAndDerivatives[],  // out: (pos, tang, 2nd, ...) sized:[lNumDerivatives+1]
                            SmBoolean  bNonZeroTangents=TRUE)   // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                           const ;                              //      FALSE= return exact tangent values
                                                                //      note: Surprisingly TRUE is the common choice because most tangent uses
                                                                //            are for their direction (Binorm, SurfNorm comps), but when the 
                                                                //            tangent is being used for its magnitude (like an arc-length comp)
                                                                //            then set this to FALSE
  
  virtual SmStatus EvaluatePoint(double dParameter, SmPoint3d & rPoint) const ;
  
  // simple data access
  
  // Get methods
  virtual SmExtent1d       GetNaturalInterval()                                  const { return m_vCurveInterval; }
  const   SmCurve        * GetBaseCurve()                                        const { return m_pCurve ; }
  virtual SmBSplineCurve * GetRootCurve()                                        const { return( m_pCurve->GetRootCurve() ) ; }
  double                   GetOffsetDistance()                                   const { return m_dOffsetDistance ; }
  SmVector3d               GetOffsetNormal()                                     const { return m_vOffsetPlaneNorm ; }
  virtual ULONG            GetNumberNaturalKnots()                               const ;
  virtual SmExtent1d       GetMaxAnalyticDomain()                                const {return m_pCurve->GetMaxAnalyticDomain(); }
  virtual SmStatus         GetKnots(SmTArray<double> & rKnots,                            // out:
                                    SmTArray<ULONG>  * pKnotMultiplicities=NULL,          // out:
                                    const SmExtent1d * pOptIvl=NULL)             const ;  // in : ivl of interest, NULL=Natural Interval, default:[NULL]
                                   
  
  // Set methods
  void SetOffsetDistance(double dOffsetDistance) ;
  void SetContext(const SmContext * cpContext) { m_cpContext = cpContext ;
                                                 if (m_bOwnsCurve && m_pCurve ) m_pCurve->SetContext(cpContext); }
  
  // computed data access
  virtual SmStatus CalculateContinuities(SmContinuityType           & reMinContinuityInCurve,
                                         SmTArray<SmContinuityType> & rContinuitiesAtKnots,
                                         double                       dContinuityAngleTol = SM_CONTINUITY_ANGLE)
                                        const { return m_pCurve->CalculateContinuities(reMinContinuityInCurve,
                                                                                       rContinuitiesAtKnots,
                                                                                       dContinuityAngleTol) ;
                                              }
  
  // modifiers
  virtual SmStatus ReverseParameterization(const SmExtent1d & crOldInterval,
                                           SmExtent1d & rNewInterval) ;

  virtual SmStatus Trim(SmExtent1d & crTrimInterval,          // in : desired new Trim Ivl - this virtual method does not snap TrimIvl
                        SmBoolean    bNotify=TRUE,            // in : internal use: use default value
                        SmBoolean    bSkipDebugCheck=FALSE) ; // NotUsed: in : internal use: use default value

  // get Curve Memory size - not its attributes
  virtual ULONG GetMemoryUsed(ULONG    & rlMemoryAllocated,               // out: bigger size of all allocated memory in bytes
                              SmMarkType eMarkType = SM_MT_NOMARK) const  // in : uses without increment eMarkType value
                             { SM_REF1(eMarkType) ;
                               ULONG lThisAllocated;
                               ULONG lUsed = rlMemoryAllocated = sizeof(*this) ;
                              
                               // + cache memory
                               if(m_pCacheObj)
                                 { lUsed += m_pCacheObj->GetMemoryUsed( lThisAllocated) ;
                                   rlMemoryAllocated += lThisAllocated ;
                                 }
                               return(lUsed) ;
                             }
  
  virtual SmStatus WriteToDB(SmDatabaseIO & rDB,                      // in : target output stream
                             ULONG          lDBVersionNumber)         // in : database version to get proper sequence of writes
                            const ;   
  
  static  SmStatus ReadFromDB(SM_TYPE           lType,              // NotUsed: in : Object type to be read
                              SmDatabaseIO    & rDB,                // in : target output stream
                              ULONG             lDim,               // NotUsed: in : curve image space dim, 2 or 3                                                                                    
                              const SmContext & crContext,          // in : context for new object construction
                              SmCurve         *&rpNewCurve,         // out:    NULL on input = new object allocated in this routine built from stream data
                                                                    //      NotNULL on input = pointer to an empty object to be filled by this routine
                              ULONG             lDBVersionNumber) ; // in : database version to get proper sequence of writes

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmOffsetCurve,SmCurve,SmOffsetCurve_TYPE);
  
  virtual SmBoolean AssertValid(SmAssertArray    * pAList = NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                SmAssertTestLevel  eTestLevel = SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                            //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                            //      default:[SM_LEVEL_0] 
                                SmAssertWalking    eWalkTree = SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                SmTArray<ULONG>  * pTestRequests = NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                               const ;
  
} ; // end class SmOffsetCurve

#endif // !__SMOFFSETCURVE_H__
