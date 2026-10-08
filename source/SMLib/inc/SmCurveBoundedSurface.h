// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCurveBoundedSurface.h
* PURPOSE: Header file for curve bounded surface also known as
*   a trimmed surface.
**********************************************************************/

#ifndef __SMCURVEBOUNDEDSURFACE_H__
#define __SMCURVEBOUNDEDSURFACE_H__

#ifndef __SMSURFACE_
#include <SmSurface.h>
#endif

#ifndef __SMCOMPOSITECURVE_H__
#include <SmCompositeCurve.h>
#endif

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

/*******************************************************************//**
PURPOSE: The SmCurveBoundedSurface class defines trimmed surface
            only to be used for IGES I/O in SmIgesReader.[h,cpp].

NOTES: The curve bounded surface owns its surface and boundary
   curves.  The deletion of the curve bounded surface will delete the
   surface and boundary curves.
***********************************************************************/
class SM_EXPORT SmCurveBoundedSurface : public SmSurface
{
private:
    SmSurface          * m_pBasisSurface;       // always deleted by destructor
    SmTArray<SmCurve*> & m_v3DBoundaries;       // members always deleted by destructor
    SmTArray<SmCurve*> & m_vUVBoundaries;       // members always deleted by destructor
    SmBoolean            m_bImplicitOuter;

public:
    // empty constructor for I/O
    SmCurveBoundedSurface() : SmSurface(),
                              m_pBasisSurface(NULL),
                              m_v3DBoundaries(*(new (*GetContext()) SmTArray<SmCurve*>(*GetContext()))),
                              m_vUVBoundaries(*(new (*GetContext()) SmTArray<SmCurve*>(*GetContext()))),
                              m_bImplicitOuter(FALSE)
                            { }

    // constructor
    SmCurveBoundedSurface
    (
      SmSurface                & rBasisSurface,
      const SmTArray<SmCurve*> * cp3DBoundaries,
      const SmTArray<SmCurve*> * cpUVBoundaries,
      SmBoolean bImplicitOuter=FALSE
    );

    // destructor
    virtual ~SmCurveBoundedSurface();

    // equality operator
    virtual SmBoolean operator==(const SmSurface &crOther) const;

    SmStatus CreateTrimmedSurfaceInBrep
    (
      SmBrep * pExistingBrep,
      SmFace *& rpNewFace
    );

    virtual SmBSplineSurface * GetRootSurface() const  { return m_pBasisSurface->GetRootSurface() ; }

    virtual SmExtent2d         GetNaturalUVDomain() const { return(m_pBasisSurface->GetNaturalUVDomain()) ; }
  
    virtual SmStatus EvaluateNormal
    (
      const SmPoint2d & crUV,              ///< [in] : surface point within surface domain                                                  <br>
      SmBoolean         bUFromLeft,        ///< [in] : if crUV is on U interval boundary, TRUE=Use upper interval, FALSE = lower interval   <br>
      SmBoolean         bVFromLeft,        ///< [in] : if crUV is on V interval boundary, TRUE=Use upper interval, FALSE = lower interval   <br>
      SmVector3d      & rSurfaceNormal     ///< [out]: unit-vect normal = crossProduct(Wu,Wv)                                               <br>
    )  const
    { 
      return (m_pBasisSurface->EvaluateNormal( crUV, bUFromLeft, bVFromLeft, rSurfaceNormal ));  // pass the call along
    }


    virtual SmStatus Evaluate
    (
      const SmPoint2d & crUV,           ///< [in] : param value to evaluate                                                                     <br>
      ULONG lHighestUDeriv,             ///< [in] : number of U derivatives                                                                     <br>
      ULONG lHighestVDeriv,             ///< [in] : number of V derivatives to compute                                                          <br>
      SmBoolean bUFromLeft,             ///< [in] : if P is on U interval boundary                                                              <br>
                                        ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval                 <br>
                                        ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval                <br>
      SmBoolean bVFromLeft,             ///< [in] : if P is on V interval boundary                                                              <br>
                                        ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval                 <br>
                                        ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval                <br>
      SmBoolean bOnlyUpperHalf,         ///< [in] : TRUE=compute upper half of matrix only                                                      <br>
                                        ///<      : ex. 1,1 = [D  Du] 2,2 = [D    Du    Duu] where -- = an untouched memory value               <br>
                                        ///<      :           [Dv --]       [Dv   Duv   ---]          (the memory has to be allocated)          <br>
                                        ///<      :                         [Dvv  ---   ---]                                                    <br>
      SmVector3d *aDerivatives,         ///< [out]: matrix of evaluations values                                                                <br>
                                        ///<      : sized:[lHighestUDeriv+1][lHighestVDeriv+1]                                                  <br>
                                        ///<      : 2d organized: [D    Du    Duu    Duuu    Duuuu   ]  (the same no matter the value of)       <br>
                                        ///<      :               [Dv   Duv   Duuv   Duuuv   Duuuuv  ]  (  bOnlyUpperHalf               )       <br>
                                        ///<      :               [Dvv  Duvv  Duuvv  Duuuvv  Duuuuvv ]                                          <br>
                                        ///<      :               [Dvvv Duvvv Duuvvv Duuuvvv Duuuuvvv]                                          <br>
                                        ///<      : 1d organized: [D, Dv, Dvv, Dvvv,.. Du, Duv, Duvv, Duvvv,.. Duu, Duuv, Duuvv, Duuvvv,...]    <br>
     SmBoolean bNonZeroTangents = TRUE, ///< [in] : TRUE = replace zero tangent vectors with properly oriented tol sized vectors                <br>
                                        ///<      : FALSE= return exact tangent values                                                          <br>
                                        ///<      : note: Surprisingly TRUE is the common choice because most tangent uses                      <br>
                                        ///<      :       are for their direction (Binorm, SurfNorm comps), but when the                        <br>
                                        ///<      :       tangent is being used for its magnitude (like an arc-length comp)                     <br>
                                        ///<      :       then set this to FALSE.                                                               <br>
     SmBoolean bDoZeroSampling = TRUE   ///< [in] : for internal use only, always set to TRUE, default:[TRUE]                                   <br>
    ) const
    {
      return (m_pBasisSurface->Evaluate( crUV, lHighestUDeriv, lHighestVDeriv,
               bUFromLeft, bVFromLeft,
               bOnlyUpperHalf, aDerivatives,
               bNonZeroTangents, bDoZeroSampling ));
    }

    // evaluate surface without any adjustments for ZeroTangents
    virtual SmStatus EvaluateSimple
    (
      const SmPoint2d & crUV,                       ///< [in] :                                                                                    <br>
      ULONG             lHighestUDeriv,             ///< [in] :                                                                                    <br>
      ULONG             lHighestVDeriv,             ///< [in] :                                                                                    <br>
      SmBoolean         bUFromLeft,                 ///< [in] : if crUV is on U interval boundary, TRUE=Use upper interval, FALSE = lower interval <br>
      SmBoolean         bVFromLeft,                 ///< [in] : if crUV is on V interval boundary, TRUE=Use upper interval, FALSE = lower interval <br>
      SmBoolean         bOnlyUpperHalf,             ///< [in] : Should be TRUE                                                                     <br>
      SmVector3d      * aDerivatives                ///< [out]:
    ) const
    {
      return (m_pBasisSurface->EvaluateSimple( crUV, lHighestUDeriv, lHighestVDeriv,
               bUFromLeft, bVFromLeft,
               bOnlyUpperHalf, aDerivatives ));
    }

    virtual SmStatus EvaluateNormalAtSingularity
    (
      const SmPoint2d & crUV,
      SmVector3d      & rSurfaceNormal,
      double            d3dTol = SM_EFF_ZERO
    ) const
    {
      return (m_pBasisSurface->EvaluateNormalAtSingularity( crUV, rSurfaceNormal, d3dTol ));
    }

    virtual SmStatus EvaluateGeometric
    (
      const SmPoint2d & crUV,                       ///< [in] : Target Surface Parameter                                                              <br>
      SmBoolean         bUFromLeft,                 ///< [in] : if crUV is on U interval boundary, TRUE=Use upper interval, FALSE = lower interval    <br>
      SmBoolean         bVFromLeft,                 ///< [in] : if crUV is on V interval boundary, TRUE=Use upper interval, FALSE = lower interval    <br>
      double          & rdGaussianCurvature,        ///< [out]: GaussianCurvature = K1*K2 (principal curvatures)                                      <br>
      double          & rdNormalCurvature,          ///< [out]: MeanCurvature = 1/2(K1 + K2)                                                          <br>
      double          & rdPrincipleCurvature1,      ///< [out]: 1st principal curvature value, K1                                                     <br>
      double          & rdPrincipleCurvature2,      ///< [out]: 2nd principal curvature value, K2                                                     <br>
      SmVector3d      & rEFGOfFirstFundForm,        ///< [out]: 1st fundamental form, [E, F, G]                                                       <br>
      SmVector3d      & rLMNOfSecondFundForm,       ///< [out]: 2nd fundamental form, [L, M, N]                                                       <br>
      SmVector3d      & rPrincipleCurvatureVector1, ///< [out]: 3d vector tangent to surface in 1st principal direction                               <br>
      SmVector3d      & rPrincipleCurvatureVector2  ///< [out]: 3d vector tangent to surface in 2nd principal direction                               <br>
    ) const
    {
      return (m_pBasisSurface->EvaluateGeometric( crUV, bUFromLeft, bVFromLeft,
               rdGaussianCurvature, rdNormalCurvature,
               rdPrincipleCurvature1, rdPrincipleCurvature2,
               rEFGOfFirstFundForm,
               rLMNOfSecondFundForm,
               rPrincipleCurvatureVector1,
               rPrincipleCurvatureVector2 ));
    }

    virtual SmStatus EvaluateNormalSection
    (
      const SmPoint2d  & crUV,                   ///< [in] : Target Point                                                                          <br>
      SmBoolean          bUFromLeft,             ///< [in] : if crUV is on U interval boundary, TRUE=Use upper interval, FALSE = lower interval    <br>
      SmBoolean          bVFromLeft,             ///< [in] : if crUV is on V interval boundary, TRUE=Use upper interval, FALSE = lower interval    <br>
      const SmVector3d & crDirection,            ///< [in] : Direction which projects to the tangent of a curve on                                 <br>
                                                 ///<      : the surface at which the evaluation is being done. Non-unit is OK.                    <br>
      SmVector3d       & rTangentPlaneDirection, ///< [out]: Unit-Vector in direction of crDirection projected into tangent plane.                 <br>
      double           & rdCurvatureValue,       ///< [out]: Curvature value of the theoretical surface curve whose tangent                        <br>
                                                 ///<      : direction is rTangentPlaneDirection.                                                  <br>
      SmVector3d       & rSurfaceNormalVector    ///< [out]: unit-Normal vector to the surface at that point 
    )  const
    {
      return (m_pBasisSurface->EvaluateNormalSection( crUV, bUFromLeft, bVFromLeft,
               crDirection,
               rTangentPlaneDirection,
               rdCurvatureValue,
               rSurfaceNormalVector ));
    }

    virtual SmStatus EvaluatePoint
    (
      const SmPoint2d & crUV,
      SmPoint3d        & rPoint
    ) const
    {
      return (m_pBasisSurface->EvaluatePoint( crUV, rPoint ));
    }


    virtual SmStatus EvaluateSTEP
    (
      const SmPoint2d & crUV,                       ///< [in] :                                                                                      <br>
      ULONG             lHighestUDeriv,             ///< [in] :                                                                                      <br>
      ULONG             lHighestVDeriv,             ///< [in] :                                                                                      <br>
      SmBoolean         bUFromLeft,                 ///< [in] : if crUV is on U interval boundary, TRUE=Use upper interval, FALSE = lower interval   <br>
      SmBoolean         bVFromLeft,                 ///< [in] : if crUV is on V interval boundary, TRUE=Use upper interval, FALSE = lower interval   <br>
      SmBoolean         bOnlyUpperHalf,             ///< [in] : Should be TRUE                                                                       <br>
      SmVector3d      * aDerivatives
    ) const
    {
      return (m_pBasisSurface->EvaluateSTEP( crUV, lHighestUDeriv, lHighestVDeriv,
               bUFromLeft, bVFromLeft,
               bOnlyUpperHalf, aDerivatives ));
    }

    virtual SmStatus EvaluateSTEPPoint
    (
      const SmPoint2d & crUV,
      SmPoint3d       & rPoint
    ) const
    {
      return (m_pBasisSurface->EvaluateSTEPPoint( crUV, rPoint ));
    }


    void SetContext(const SmContext * cpContext) { m_cpContext = cpContext ;
                                                   m_pBasisSurface->SetContext(cpContext);
                                                   for (ULONG ii = 0; ii < m_v3DBoundaries.GetSize(); ++ii)
                                                     { m_v3DBoundaries[ii]->SetContext(cpContext);} 
                                                   for (ULONG ii = 0; ii < m_vUVBoundaries.GetSize(); ++ii)
                                                     { m_vUVBoundaries[ii]->SetContext(cpContext);} 
                                                 }

    // get memory used for curve but not its attributes
    virtual ULONG GetMemoryUsed
    (
      ULONG    & rlMemoryAllocated,      ///< [out]: bigger size of all allocated memory in bytes  <br>
      SmMarkType eMarkType=SM_MT_NOMARK  ///< [in] : uses without increment eMarkType value        <br>
    ) const ;

    virtual SmStatus WriteToDB
    (
      SmDatabaseIO & rDB,                ///< [in] : target output stream                                 <br>
      ULONG          lDBVersionNumber    ///< [in] : database version to get proper sequence of writes    <br>
    ) const ;                                                                        

    static  SmStatus ReadFromDB
    (
      SM_TYPE           lType,              ///< NotUsed: [in] : Object type to be read                                                       <br>
      SmDatabaseIO    & rDB,                ///< [in] : target output stream                                                         <br>
      const SmContext & crContext,          ///< [in] : context for new object construction                                          <br>
      SmSurface      *& rpNewSurface,       ///< [out]: NULL on input = new object allocated in this routine built from stream data  <br>
                                            ///<      : NotNULL on input = pointer to an empty object to be filled by this routine   <br>
      ULONG             lDBVersionNumber    ///< [in] : database version to get proper sequence of writes                            <br>
    ) ; 

    virtual SmBoolean AssertValid
    (
      SmAssertArray    * pAList=NULL,           ///< {in,out]: Accumulating list of failed Asserts, NULL to ignore                                           <br>
      SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                       <br>
                                                ///<      : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                   <br>
                                                ///<      : default:[SM_LEVEL_0]                                                                             <br>
      SmAssertWalking    eWalkTree=SM_WALK,     ///< [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] <br>
      SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                 <br>
    ) const ;

    // obsolete
    // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmCurveBoundedSurface,SmSurface,SmCurveBoundedSurface_TYPE);

    // draw surface and crossHatch lines, or NumBetweenU == NumBetweenV == 999: Draw as 4 corners connected by lines
    virtual SmDisplayList * DrawUV 
    (
      ULONG              lNumBetweenU = 8,             ///< [in] : number of U IsoParameter lines between knots       <br>
      ULONG              lNumBetweenV = 8,             ///< [in] : number of V IsoParameter lines between knots       <br>                       
      SmBoolean          bVaryCrossHatchColor = FALSE, ///< [in] : TRUE = Draw U Lines in ObjectColor                 <br>
                                                       ///<      :        Draw V lines in m_VaryCrossHatchColor       <br>
                                                       ///<      : FALSE= Draw both U and V Lines in ObjectColor      <br>
      const SmExtent2d * pOptUVDomain = NULL,          ///< [in] : UVDomain to crossHatch, NULL=use NaturalUVDomain   <br>
      SmBoolean          bAddToUIPickList = FALSE,     ///< [in] : TRUE=Add to UI pick list, FALSE=don't         <br>
      SmGfxArraySet    * pOptGfxSet = NULL             ///< [in,out]: When given output GfxVertexArrays not GL calls. <br>
    ) const;

}; // end class SmCurveBoundedSurface                               


#endif // !__SMCURVEBOUNDEDSURFACE_H__



