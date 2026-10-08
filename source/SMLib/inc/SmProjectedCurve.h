// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmProjectedCurve.h
* PURPOSE: Header file for projected curve defined as either
*    parallel- or perspective-projection of a space curve onto a view plane
**********************************************************************/

#ifndef __SMPROJECTEDCURVE_H__
#define __SMPROJECTEDCURVE_H__

#ifndef __SMCURVE_H__
#include <SmCurve.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

/*******************************************************************//**
PURPOSE: The SmProjectedCurve class defines a projected image of a 3D
    curve onto a view-plane. Currently, only parallel-projection is
    implemented.

NOTES: 
***********************************************************************/
class SM_EXPORT SmProjectedCurve : public SmCurve
{
 private:
  const SmCurve *   m_cpCurve = NULL ;             // Curve being projected, not owned by SmProjectedCurve
                                                   
  SmPoint3d         m_sProjPoint;                  // parallel   : Point on the view plane
                                                   // perspective: Point on the view plane
                                                   // rotation   : Point on the rotation axis
                                                   
  SmVector3d        m_sProjVec;                    // parallel   : unit-Normal of the view plane
                                                   // perspective: unit-Normal of the view plane
                                                   // rotation   : rotation axis vector
                                                   
  SmVector3d        m_sAuxData;                    // parallel   : not used
                                                   // perspective: eye point
                                                   // rotation   : unit-XAxis vector of plane spanned by vectors [AuxData, sProjVec]
  
  SmProjectionType  m_eProjType = SM_PT_UNKNOWN ;  // oneof SM_PT_PARALLEL, SM_PT_PERSPECTIVE, SM_PT_ROTATION
  
  SmBoolean         m_bOwnsCurve = UNSURE ;        // TRUE = delete m_cpCurve in destructor, FALSE = don't, default:[FALSE]
  
 public:
  // default constructor (also empty constructor for I/O)
  SmProjectedCurve(const SmCurve    *cpCurve    =NULL,          // in : Curve being projected
                   const SmPoint3d  *cpProjPoint=NULL,          // in : parallel: pt on view plane,  rot: pt on rot axis,      perspective: pt on view plane
                   const SmVector3d *cpProjVec  =NULL,          // in : parallel: view plane normal, rot: pt on rot axis,      perspective: view plane unitNormal  
                   const SmVector3d *cpAuxData  =NULL,          // in : parallel: not used,          rot: XAxis of proj plane, perspective: eye point         
                   SmProjectionType   eProjType =SM_PT_UNKNOWN, // in : oneof SM_PT_PARALLEL, SM_PT_PERSPECTIVE, SM_PT_ROTATION
                   SmBoolean          bOwnsCurve=FALSE,         // in : TRUE = delete m_cpCurve in destructor, FALSE = don't
                   const SmContext  * cpContext =NULL) ;        // in : must be given for automatic variables, optional for
                                                                //      stack variables built with overloaded new.
         
  
  // copy constructor
  SmProjectedCurve(const SmProjectedCurve & crCurveToCopy);
  
  // virtual copy method
  virtual SmStatus Copy(const SmContext & crContext,
                        SmCurve        *& crNewCurve) const;
  
  // destructor
  virtual ~SmProjectedCurve() ;
  
  // equality operator
  virtual SmBoolean operator==(const SmCurve &crOther) const;
  
  void     SetContext(const SmContext * cpContext) { m_cpContext = cpContext ;
                                                     if (m_bOwnsCurve && m_cpCurve ) SM_CONST_CAST(SmCurve*, m_cpCurve)->SetContext(cpContext); }
  
  SmStatus SetCanonical(const SmCurve    & crCurve,             // in : Curve being projected
                        const SmPoint3d  & crProjPoint,         // in : parallel: pt on view plane,  rot: pt on rot axis,      perspective: pt on view plane
                        const SmVector3d & crProjVec,           // in : parallel: view plane normal, rot: pt on rot axis,      perspective: view plane unitNormal
                        const SmVector3d & crAuxData,           // in : parallel: not used,          rot: XAxis of proj plane, perspective: eye point             
                        SmProjectionType   eProjType,           // in : oneof SM_PT_PARALLEL, SM_PT_PERSPECTIVE, SM_PT_ROTATION
                        SmBoolean          bOwnsCurve=FALSE,    // in : TRUE = delete m_cpCurve in destructor, FALSE = don't
                        const SmContext  * cpContext =NULL) ;   // NotUsed: in : must be given for automatic variables, optional for
                                                                //      stack variables built with overloaded new.
  
  SmStatus MakeOrthoNormal() ;                                  // eff: unitize and orthoganlize ProjVec and AuxData
                                                                // rtn: SM_ERR when ProjVec or AuxData are uninit or zero, else rtns SM_SUCCESS
  // Change NURB parameterization of curve to given extent range - no Analytic-STEP range change
  virtual SmStatus   EditParameterization(const SmExtent1d & crNewParameterization,
                                          SmBoolean          bNotify=TRUE) ; 
  
  virtual SmExtent1d GetNaturalInterval() const;
  
  virtual SmStatus   GetKnots(SmTArray<double> & rKnots,                       // out: Unique knot vector                                         
                              SmTArray<ULONG>  * pKnotMultiplicities = NULL,   // out: multiplicity value for each knot                           
                              const SmExtent1d * pOptIvl = NULL)               // in : interval of interest, NULL=Natural Interval, default:[NULL]
                             const ;

  virtual ULONG      GetNumberNaturalKnots() const ;
  
  virtual SmStatus   CalculateContinuities(SmContinuityType           & reMinContinuityInCurve,
                                           SmTArray<SmContinuityType> & rContinuitiesAtKnots,
                                           double dContinuityAngleTol = SM_CONTINUITY_ANGLE)
                                          const;
  
  virtual SmBSplineCurve * GetRootCurve() const { return( m_cpCurve->GetRootCurve() ) ; }  
                                          
  virtual SmBoolean        IsAnalytic()   const { return FALSE; }
  virtual SmBoolean        IsBounded()    const { return m_cpCurve->IsBounded(); } // TRUE=finite (FALSE=infinite) parameter range
  
  virtual SmStatus  ReverseParameterization(const SmExtent1d & crOldInterval,
                                            SmExtent1d       & rNewInterval) ;

  virtual SmStatus  Transform(const SmAxis2Placement & crRotateNMove,
                              const SmVector3d * cpOptScale) ;
                    
  virtual SmStatus  Trim(SmExtent1d & crTrimInterval,          // i/o: desired new Trim Ivl - can be snapped by tol to existing knots
                         SmBoolean    bNotify=TRUE,            // in : internal use: use default value
                         SmBoolean    bSkipDebugCheck=FALSE) ; // in : internal use: use default value
                     
  virtual SmStatus  Evaluate(double dParameter,                 // in : tgt param
                             ULONG lNumDerivatives,             // in : 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .
                             SmBoolean bFromLeft,               // in : if P is on interval boundary
                                                                //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                                                //      FALSE = evaluate P in lower interval where P is on the right of the interval
                             SmVector3d aPointAndDerivatives[], // out: (pos, tang, 2nd, ...) sized:[lNumDerivatives+1]
                             SmBoolean bNonZeroTangents=TRUE)   // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
                            const ;                             //      FALSE= return exact tangent values
                                                                //      note: Surprisingly TRUE is the common choice because most tangent uses
                                                                //            are for their direction (Binorm, SurfNorm comps), but when the 
                                                                //            tangent is being used for its magnitude (like an arc-length comp)
                                                                //            then set this to FALSE.
                                                                //      default:[TRUE]
  
  virtual SmStatus EvaluatePoint(double dParameter, SmPoint3d & rPoint) const;
  
  virtual SmDisplayList * DrawPolygon( SmGfxArraySet    * pOptGfxSet = NULL ) const;
  
  // get Curve Memory size - not its attributes
  virtual ULONG GetMemoryUsed (ULONG    & rlMemoryAllocated,             // out: bigger size of all allocated memory in bytes
                               SmMarkType eMarkType=SM_MT_NOMARK) const  // in : uses without increment eMarkType value
                             { SM_REF1(eMarkType) ;
                               ULONG lThisAllocated ;
                               ULONG lUsed = rlMemoryAllocated = sizeof(*this) ;
                             
                               // + cache memory
                               if(m_pCacheObj)
                                 { lUsed += m_pCacheObj->GetMemoryUsed(lThisAllocated) ;
                                   rlMemoryAllocated += lThisAllocated;
                                 }
                               return( lUsed );
                             }
  
  virtual SmStatus WriteToDB (SmDatabaseIO & rDB,                   // in : target output stream
                              ULONG          lDBVersionNumber)      // in : database version to get proper sequence of writes  
                             const ;

  static  SmStatus ReadFromDB(SM_TYPE           lType,              // NotUsed: in : Object type to be read
                              SmDatabaseIO    & rDB,                // in : target output stream
                              ULONG             lDim,               // NotUsed: in : curve image space dim, 2 or 3                                                                                    
                              const SmContext & crContext,          // in : context for new object construction
                              SmCurve         *&rpNewCurve,         // out:    NULL on input = new object allocated in this routine built from stream data
                                                                    //      NotNULL on input = pointer to an empty object to be filled by this routine
                              ULONG             lDBVersionNumber) ; // in : database version to get proper sequence of writes
  
  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                          //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                          //      default:[SM_LEVEL_0] 
                                SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                               const ;

  // obsolete
  // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmProjectedCurve,SmCurve,SmProjectedCurve_TYPE);
  
} ; // end class SmProjectedCurve

#endif // !__SMPROJECTEDCURVE_H__


