// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmConic.h
* PURPOSE: Header file for simple rational parametric conic curve.
**********************************************************************/

#ifndef __SMCONIC_H__
#define __SMCONIC_H__

#ifndef __SMCURVE_H__
#include <SmCurve.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

/*******************************************************************//**
PURPOSE: This flag defines the type of conic being evaluated by
    the SmConic class.

NOTES: 
***********************************************************************/
enum SmConicType {
    SM_CT_ELLIPSE,
    SM_CT_HYPERBOLA,
    SM_CT_PARABOLA,
    SM_CT_LINE,
    SM_CT_UNKNOWN
};


/*******************************************************************//**
PURPOSE: The SmConic class defines a Conic curve which is parameterized
    from - infinity to + infinity and may not be bounded.  The conic resides
    in its natural position.  The following equations are used to define 
    the parameterization of the conics.

Example:
     Ellipse:   x(t) = a (1 - t^2) / (1 + t^2),  y(t) = b (2t) / (1 + t^2)
     Hyperbola: x(t) = a (1 + t^2) / (1 - t^2),  y(t) = b (2t) / (1 - t^2)
     Parabola:  x(t) = a t^2,                    y(t) = 2at
     Line:      x(t) = t,                        y(t) = a

NOTES: SmConic Curves are 2 dimensional
***********************************************************************/
class SM_EXPORT SmConic : public SmCurve
{
private:
  SmConicType   m_eConicType = SM_CT_UNKNOWN ;      // Type of conic: SM_CT_ELLIPSE, SM_CT_HYPERBOLA, SM_CT_PARABOLA, or SM_CT_LINE
  double        m_dA         = SM_UNDEF_DOUBLE ;    // a and b of above conic equations.
  double        m_dB         = SM_UNDEF_DOUBLE ;
  SmPoint2d     m_vOrigin ;                         // Origin of conic
  double        m_dThetaRad  = SM_UNDEF_DOUBLE ;    // 2D rotation angle in radians of conic relative to
                                                    // counter clockwise rotation from X axis.
  double        m_dSinTheta  = SM_UNDEF_DOUBLE ;    // Sine of rotation angle.
  double        m_dCosTheta  = SM_UNDEF_DOUBLE ;    // Cosine of rotation angle.
  
public:
  // constructor
  SmConic(SmConicType eConicType,
          double dA,
          double dB);
  
  // empty constructor for I/O
  SmConic() : SmCurve(2),
              m_eConicType(SM_CT_UNKNOWN), 
              m_dA        (SM_UNDEF_DOUBLE),
              m_dB        (SM_UNDEF_DOUBLE),
              m_dThetaRad (SM_UNDEF_DOUBLE),  
              m_dSinTheta (SM_UNDEF_DOUBLE),
              m_dCosTheta (SM_UNDEF_DOUBLE)
            { }
  
  // destructor
  virtual ~SmConic() { }
  
  // equality operator
  virtual SmBoolean  operator==(const SmCurve &crOther) const;
  
  virtual SmStatus   CalculateContinuities(SmContinuityType           & reMinContinuityInCurve, // out:
                                           SmTArray<SmContinuityType> & rContinuitiesAtKnots,   // out:
                                           double dContinuityAngleTol = SM_CONTINUITY_ANGLE)    // NotUsed: in :
                                          const;
  
  virtual SmExtent1d GetNaturalInterval() const { SmExtent1d sRet;
                                                  if(m_eConicType == SM_CT_PARABOLA ||
                                                      m_eConicType == SM_CT_LINE) sRet = SmExtent1d( -999.0, 999.0 );
                                                  else if(m_eConicType == SM_CT_ELLIPSE) sRet = SmExtent1d( -1.0, 3.0 );
                                                  else sRet = SmExtent1d( -1.0 + SM_EFF_ZERO_SQRT, 1.0 - SM_EFF_ZERO_SQRT );
                                                  return sRet;
                                                }
  
  virtual SmStatus   GetKnots(SmTArray<double> & rKnots,                       ///< [out]: Unique knot vector                                          <br>
                              SmTArray<ULONG>  * pKnotMultiplicities = NULL,   ///< [out]: multiplicity value for each knot                            <br>
                              const SmExtent1d * pOptIvl = NULL)               ///< [in] : interval of interest, NULL=Natural Interval, default:[NULL] <br>
                             const;        
  
  virtual SmBSplineCurve * GetRootCurve()        const { return( NULL ) ; } /* When available return equivalent BSplineCurve */ 
  
  virtual SmStatus Evaluate(double    dParameter,               ///< [in] : tgt param                                                                      <br>
                            ULONG     lNumDerivatives,          ///< [in] : 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .                                       <br>
                            SmBoolean bFromLeft,                ///< [in] : if P is on interval boundary                                                   <br>
                                                                ///<        TRUE  = evaluate P in upper interval where P is on the left of the interval    <br>
                                                                ///<        FALSE = evaluate P in lower interval where P is on the right of the interval   <br>
                            SmVector3d aPointAndDerivatives[],  ///< [out]: (pos, tang, 2nd, ...) sized:[lNumDerivatives+1]                                <br>
                            SmBoolean  bNonZeroTangents=TRUE)   ///< [in] : TRUE = replace zero tangent vectors with properly oriented tol sized vectors   <br>
                                                                ///<        FALSE= return exact tangent values                                             <br>
                                                                ///<        note: Surprisingly TRUE is the common choice because most tangent uses         <br>
                                                                ///<              are for their direction (Binorm, SurfNorm comps), but when the           <br>
                                                                ///<              tangent is being used for its magnitude (like an arc-length comp)        <br>
                                                                ///<              then set this to FALSE.                                                  <br>
                           const;
  
  virtual SmStatus EvaluatePoint(double dParameter, SmPoint3d & rPoint) const;
  
  virtual SmStatus IntersectImplicitConic(double adConicCoeff[6],
                                          SmTArray<double> & rIntersectionResults,
                                          SmTArray<ULONG> & rIntersectionMultiplicity)
                                         const;
  
  virtual SmStatus ComputeImplicitEquation(double adConicCoeff[6]) const;
  
  virtual SmStatus Transform2D(double dCCWRotationAngleRadians,
                               const SmPoint2d & crTranslation) ;
  
  SmStatus InvertPointOnConic(const SmPoint3d & crPoint,
                              SmBoolean & rbSuccess,
                              double & rdParameter)
                             const;
  
  SmStatus IntersectConic(const SmExtent1d & crInterval,
                          const SmConic & crOtherConic,
                          const SmExtent1d & crOtherInterval,
                          double dDistanceTolerance,
                          SmSolutionArray & rSolutions)
                         const;
  
  // get Curve Memory size - not its attributes
  virtual ULONG GetMemoryUsed(ULONG    & rlMemoryAllocated,               ///< [out]: bigger size of all allocated memory in bytes  <br>
                              SmMarkType eMarkType = SM_MT_NOMARK) const  ///< [in] : uses without increment eMarkType value        <br>
                            { SM_REF1(eMarkType) ;
                              ULONG lThisAllocated;
                              ULONG lUsed = rlMemoryAllocated = sizeof(*this) ;
                            
                              // + cache memory
                              if(m_pCacheObj)
                                { lUsed += m_pCacheObj->GetMemoryUsed(lThisAllocated) ;
                                  rlMemoryAllocated += lThisAllocated ;
                                }
                              return(lUsed) ;
                            }
  
  virtual SmStatus WriteToDB(SmDatabaseIO & rDB,               ///< [in] : target output stream                               <br>
                             ULONG          lDBVersionNumber)  ///< NotUsed: [in] : database version to get proper sequence of writes  <br>
                            const ;   
  
  static  SmStatus ReadFromDB(SM_TYPE           lType,              ///< NotUsed: [in] : Object type to be read                                                       <br>
                              SmDatabaseIO    & rDB,                ///< [in] : target output stream                                                         <br>
                              ULONG             lDim,               ///< NotUsed: [in] : curve image space dim, 2 or 3                                                <br>                                    
                              const SmContext & crContext,          ///< [in] : context for new object construction                                          <br>
                              SmCurve         *&rpNewCurve,         ///< [out]: NULL on input = new object allocated in this routine built from stream data  <br>
                                                                    ///<        NotNULL on input = pointer to an empty object to be filled by this routine   <br>
                              ULONG             lDBVersionNumber) ; ///< NotUsed: [in] : database version to get proper sequence of writes                            <br>
  
  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmConic,SmCurve,SmConic_TYPE);
  
} ; // end SmConic

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
inline SmConic::SmConic
  (SmConicType eConicType,
   double dA,
   double dB)
 : SmCurve(2),
   m_eConicType(eConicType), 
   m_dA(dA), 
   m_dB(dB), 
   m_vOrigin(0.0,0.0), 
   m_dThetaRad(0.0), 
   m_dSinTheta(0.0), 
   m_dCosTheta(1.0) 
{

} // end SmConic::SmConic constructor

#endif // !__SMCONIC_H__



