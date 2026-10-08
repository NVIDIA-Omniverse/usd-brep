// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmIsoCurve.h
* PURPOSE: Header file for implicit iso parametric curve.
**********************************************************************/

#ifndef __SMISOCURVE_H__
#define __SMISOCURVE_H__

#ifndef __SMCURVE_H__
#include <SmCurve.h>
#endif

#ifndef __SMSURF_TYPES_H__
#include <SmSurfTypes.h>
#endif

/*******************************************************************//**
PURPOSE: The SmIsoCurve class defines a curve which is the isoparametric
    (constant U or V curve) curve on a surface.  This class is intended
    for use only within the numerical algorithms.

NOTES: Rather than construct an isoParameter curve (which
  could be approximate for non-BSPline Surfaces), this just does
  nested evaluations to project from curve domain to surface to domain
  to image space as:

   C(s) = S(C(s)).

   This is a good choice for efficiency and accuracy when 
     - doing just a few evaluations on an iso-parameter curve, or
     - working with non BSpline surface.  
     
   Otherwise consider SmSurface::CreateIsoParametricCurve() which pays
       a small constructor price once to build the curve and then
       saves a little expense for every evaluation.  Some evaluations
       are extremely cheap when the IsoParameter Curve is an analytic. 
***********************************************************************/
class SM_EXPORT SmIsoCurve : public SmCurve
{
 private:
  const SmSurface * m_cpSurface;                         // Base surface, not owned by SmIsoCurve
  SmSurfParamType   m_eSurfParam = SM_SP_UNKNOWN ;       // Is it a constant U or V
  double            m_dIsoParameter = SM_UNDEF_DOUBLE ;  // Constant U or V value
  SmBoolean         m_bFromLeft = UNSURE ;               // in : if P is on interval boundary
                                                         //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                                         //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmBoolean         m_bOwnsSurface = UNSURE ;            // TRUE = delete m_cpSurface in destructor, FALSE = don't
  
 public:
  // constructor
  SmIsoCurve(const SmSurface & crSurface, 
             SmSurfParamType   eSurfParam,
             double            dIsoParameter,
             SmBoolean         bFromLeft,
             SmBoolean         bClampInput = TRUE) ;
  
  // empty constructor for I/O
  SmIsoCurve() : SmCurve(3) { }
  
  // equality operator
  virtual SmBoolean operator==(const SmCurve &crOther) const ;
  
  // destructor
  virtual ~SmIsoCurve() ;
  
  // create the UV IsoLine Obj for the C(s) UVTrimCurve part of the S(C(s)) compound curve 
  SmStatus           CreateUVIsoLine(SmCurve         *& pUVIsoLine,   // out: New allocated UVIsocCurve, NULL on input
                                     const SmContext  * cpContext)    // in : Context for obj construction
                                    const ;
                     
  virtual SmStatus   CalculateContinuities(SmContinuityType           & reMinContinuityInCurve,     // out:
                                           SmTArray<SmContinuityType> & rContinuitiesAtKnots,       // out:
                                           double dContinuityAngleTol = SM_CONTINUITY_ANGLE)        // NotUsed: in :
                                          const ;
                     
  virtual SmExtent1d GetNaturalInterval() const;

  void SetContext(const SmContext * cpContext) { m_cpContext = cpContext ;
                                                 if (m_bOwnsSurface && m_cpSurface ) SM_CONST_CAST(SmSurface*,m_cpSurface)->SetContext(cpContext); }
  
  virtual SmStatus   Evaluate(double     dParameter,              // in : tgt param
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
                                                                  //            then set this to FALSE.
                                                                  //      default:[TRUE]
                     
  virtual SmStatus   EvaluatePoint(double dParameter, SmPoint3d & rPoint) const ;

  virtual SmStatus GetKnots(SmTArray<double> & rKnots, 
                            SmTArray<ULONG>  * pKnotMultiplicities = NULL,
                            const SmExtent1d * pOptIvl = NULL)             // in : interval of interest, NULL=Natural Interval, default:[NULL]
                           const ;
  
  virtual SmBSplineCurve * GetRootCurve() const { return( NULL ) ; } /* When available return equivalent BSplineCurve */ 
  
  // get Curve Memory size - not its attributes
  virtual ULONG GetMemoryUsed(ULONG    & rlMemoryAllocated,            // out: bigger size of all allocated memory in bytes
                              SmMarkType eMarkType=SM_MT_NOMARK) const // in : uses without increment eMarkType value
                             { SM_REF1(eMarkType) ;
                               ULONG lThisAllocated ;
                               ULONG lUsed = rlMemoryAllocated = sizeof( *this ) ;
                               
                               // + cache memory
                               if(m_pCacheObj)
                                 { lUsed += m_pCacheObj->GetMemoryUsed( lThisAllocated) ;
                                   rlMemoryAllocated += lThisAllocated ;
                                 }
                               return(lUsed) ;
                             }
  
  virtual SmStatus WriteToDB (SmDatabaseIO & rDB,                      // in : target output stream
                              ULONG          lDBVersionNumber) const ; // in : database version to get proper sequence of writes  
  
  static  SmStatus ReadFromDB(SM_TYPE           lType,              // NotUsed: in : Object type to be read
                              SmDatabaseIO    & rDB,                // in : target output stream
                              ULONG             lDim,               // NotUsed: in : curve image space dim, 2 or 3                                                                                    
                              const SmContext & crContext,          // in : context for new object construction
                              SmCurve        *& rpNewCurve,         // out:    NULL on input = new object allocated in this routine built from stream data
                                                                    //      NotNULL on input = pointer to an empty object to be filled by this routine
                              ULONG             lDBVersionNumber) ; // in : database version to get proper sequence of writes
  
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmIsoCurve,SmCurve,SmIsoCurve_TYPE);
  
} ; // end class SmIsoCurve

#endif // !__SMISOCURVE_H__



