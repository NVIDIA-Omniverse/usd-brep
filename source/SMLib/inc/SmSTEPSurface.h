// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSTEPSurface.h
* PURPOSE: Header file for STEP Surface class.
**********************************************************************/

#ifndef __SMSTEPSURFACE_H__
#define __SMSTEPSURFACE_H__

#ifndef __SMSURFACE_H__
#include <SmSurface.h>
#endif

/*******************************************************************//**
PURPOSE: Evaluator wrapper for a surface's STEP parameterization.
   Evaluate forwards to EvaluateSTEP, and EvaluatePoint forwards to
   EvaluateSTEPPoint.

NOTES: This surface forwards derivative evaluation to its wrapped surface.
   Its primary purpose is to help lift UV curves from a STEP surface
   parameterization to 3D.

   For STEP-parameterized analytic surfaces (cylinder, cone, sphere, torus,
   and surfaces of revolution), angular parameters are in degrees and
   EvaluateSTEP returns derivatives with respect to those degree-valued
   parameters.
***********************************************************************/
class SM_EXPORT SmSTEPSurface : public SmSurface
{
private:
    SmSurface       * m_pSurface;         // Base surface
    SmBoolean         m_bOwnsSurface;     // If TRUE the STEP surface owns the base surface
                                          // and is responsible for deleting it.

public:
    // constructor
    SmSTEPSurface
    (
      SmSurface & rSurface,                ///< [in ]:      <br>
      SmBoolean   bOwnsSurface=FALSE       ///< [in ]:      <br>
    );

    // empty constructor for I/O
    SmSTEPSurface() { }

    // destructor
    virtual ~SmSTEPSurface();

    // equality operator
    virtual SmBoolean operator==(const SmSurface &crOther) const;

    virtual SmStatus EvaluatePoint
    (
      const SmPoint2d & crUV,         ///< [in ]:        <br>
      SmPoint3d & rPoint              ///< [out]:        <br>
    ) const;

    virtual SmStatus Evaluate
    (
      const SmPoint2d & crUV,          ///< [in ]: param value to evaluate                                                                   <br>
       ULONG lHighestUDeriv,           ///< [in ]: number of U derivatives                                                                   <br>
       ULONG lHighestVDeriv,           ///< [in ]: number of V derivatives to compute                                                        <br>
       SmBoolean bUFromLeft,           ///< [in ]: if P is on U interval boundary                                                            <br>
                                       ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval               <br>
                                       ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval              <br>
       SmBoolean bVFromLeft,           ///< [in ]: if P is on V interval boundary                                                            <br>
                                       ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval               <br>
                                       ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval              <br>
       SmBoolean bOnlyUpperHalf,       ///< [in ]: TRUE=compute upper half of matrix only                                                    <br>
                                       ///<      : ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value             <br>
                                       ///<      :           [Dv --]       [Dv   Duv   ---]          (the memory has to be allocated)        <br>
                                       ///<      :                         [Dvv  ---   ---]                                                  <br>
       SmVector3d *aDerivatives,       ///< [out]: matrix of evaluations values                                                              <br>
                                       ///<      : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                                <br>
                                       ///<      : 2d organized: [D    Du    Duu    Duuu    Duuuu   ]  (the same no matter the value of)     <br>
                                       ///<      :               [Dv   Duv   Duuv   Duuuv   Duuuuv  ]  (  bOnlyUpperHalf               )     <br>
                                       ///<      :               [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]                                        <br>
                                       ///<      :               [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]                                        <br>
                                       ///<      : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]  <br>
      SmBoolean bNonZeroTangents=TRUE, ///< [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors              <br>
                                       ///<      : FALSE= return exact tangent values                                                        <br>
                                       ///<      : note: Not used for SmSTEPSurface evals.                                                   <br>
      SmBoolean bDoZeroSampling=TRUE   ///< [in ]: for internal use only, always set to TRUE, default:[TRUE]                                 <br>
    ) const ;

    virtual SmExtent2d GetDomainWithContinuity
    (
      SmContinuityType eMinimumContinuity,
      const SmPoint2d & crUV,
      const SmVector2d & crUVVector
    ) const;

    virtual SmBSplineSurface * GetRootSurface()           const { return NULL ; } /* When available return equivalent BSplineSurface */

    virtual SmStatus GetKnots
    (
      SmSurfParamType    eSurfParam,                   ///< [in ]:                                                               <br>
      SmTArray<double> & rKnots,                       ///< [in ]:                                                               <br>
      SmTArray<ULONG>  * pKnotMultiplicities = NULL,   ///< [in ]:                                                               <br>
      const SmExtent1d * pOptIvl = NULL                ///< [in ]: interval of interest, NULL=Natural Interval, default:[NULL]   <br>
    ) const;

    virtual SmExtent2d GetNaturalUVDomain() const;

    // get memory used for curve but not its attributes
    virtual ULONG GetMemoryUsed
    (
      ULONG    & rlMemoryAllocated,      ///< [out]: bigger size of all allocated memory in bytes                                   <br>
      SmMarkType eMarkType=SM_MT_NOMARK  ///< [in ]: uses without increment eMarkType value                                         <br>
    ) const ;

    void SetContext(const SmContext * cpContext) { m_cpContext = cpContext ;
                                                   if (m_bOwnsSurface && m_pSurface ) m_pSurface->SetContext(cpContext); }
    virtual SmStatus WriteToDB
    (
      SmDatabaseIO & rDB,                      ///< [in ]: target output stream                                                      <br>
      ULONG          lDBVersionNumber          ///< [in ]: database version to get proper sequence of writes                         <br>
    ) const ;                                                                        

    static  SmStatus ReadFromDB
    (
      SM_TYPE           lType,              ///< NotUsed: [in ]: Object type to be read                                                        <br>
      SmDatabaseIO    & rDB,                ///< [in ]: target output stream                                                          <br>
      const SmContext & crContext,          ///< [in ]: context for new object construction                                           <br>
      SmSurface      *& rpNewSurface,       ///< [out]: NULL on input = new object allocated in this routine built from stream data   <br>
                                            ///<      : NotNULL on input = pointer to an empty object to be filled by this routine    <br>
      ULONG             lDBVersionNumber    ///< [in ]: database version to get proper sequence of writes                             <br>
    );

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmSTEPSurface,SmSurface,SmSTEPSurface_TYPE);

} ; // end class SmSTEPSurface



#endif // __SMSTEPSURFACE_H__
