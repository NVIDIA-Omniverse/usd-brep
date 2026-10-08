// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTangentField.h
* PURPOSE: Header file for SmTangentField object.
**********************************************************************/

#ifndef __SMTANGENTFIELD_H__
#define __SMTANGENTFIELD_H__

#ifndef __SMCRVONSURF_H__
#include <SmCrvOnSurf.h>
#endif

enum SmTangentDirType 
{
  SM_CONSTANT_DIR,
  SM_INTERPOLATE_DIR,
  SM_PERPENDICULAR_DIR
};

/*******************************************************************//**
PURPOSE: This class can be used to compute Tangent Field 
   (or Cross-boundary derivatives) of an edge of a Coons 
   or N-sided patch.

NOTES: 
  The tangent "value" of the Tangent field is interpolated between the
  stored StartTangent and EndTangent values and then projected
  to the tangent plane of the m_pSurface at the m_pUVCurve(dParameter) point.

  The interpolation is cleverly designed so that the twist vector of
  the tangent values (dTangent/dParam) is zero at the end point values.  
  That interpolation is done as

  T = .5 * cos(NormalizedParam) + .5
      so that 
      T(0) = 1.0 and dT(0)/dNormalizedParam = 0.0
      T(1) = 0.0 and dT(1)/dNormalizedParam = 0.0

  and
  Tangent = T*StartTangent + (1-T)*EndTangent

  and
  TangentValue(Param) = Tangent projection onto normal plane
                        of m_pSurface at m_pUVCurve(param).
***********************************************************************/
class SM_EXPORT SmTangentField : public SmCrvOnSurf
{
protected:
    SmTangentDirType m_eType;           // oneof: SM_CONSTANT_DIR,    
                                        //        SM_INTERPOLATE_DIR, 
                                        //        SM_PERPENDICULAR_DIR
                                        // FOR NOW: only SM_INTERPOLATE_DIR is supported

    SmVector3d   m_vStartTangentField;  // Start Point CrossTangent vector
    SmVector3d   m_vEndTangentField;    // End   Point CrossTangent vector

public:
    // constructor
    SmTangentField
    (
      const SmCurve    & crUVCurve,            ///< [in ]: TrimCurve                                             <br>
      const SmSurface  & crSurface,            ///< [in ]: associated Surface                                    <br>
      SmTangentDirType   eType,                ///< [in ]: oneof: SM_CONSTANT_DIR,                               <br>
                                               ///<      :        SM_INTERPOLATE_DIR, <== only supported value   <br>
                                               ///<      :        SM_PERPENDICULAR_DIR                           <br>
      SmVector3d         vStartTangentField,   ///< [in ]: TangentStartValue                                     <br>
      SmVector3d         vEndTangentField,     ///< [in ]: TangentEndValue                                       <br>
      const SmExtent2d * cpUVDomain = NULL     ///< [in ]: opt subDomain of projection surface                   <br>
    );
   
    // empty constructor for I/O
    SmTangentField() { }

    // destructor
    virtual ~SmTangentField();

    // equality operator
    virtual SmBoolean operator==(const SmCurve &crOther) const;

    // convenience construtor: define endTangents by 
    static SmStatus Create
    (
      const SmContext      & crContext,                 ///< [in ]: context for new object construction                                        <br>
      double                 dThisApproxTol3d,          ///< [in ]: max allowed 3D deviation from BaseCurve to BaseSurface                     <br>
      const SmCurve        & crBase3DCurve,             ///< [in ]: 3DCurve - only used for checking input                                     <br>
      const SmBSplineCurve & crBaseUVCurve,             ///< [in ]: associated UVTrimCurve on                                                  <br>
      const SmSurface      & crSurface,                 ///< [in ]: associated Surface                                                         <br>
      const SmCurve        & crStart3DCurve,            ///< [in ]: Curve connected to Base3DCurve start point                                 <br>
                                                        ///<      : Defines the start value of this VectorField                                <br>
      double                 dStartParam,               ///< [in ]: parameter of mated startPoint                                              <br>
      SmOrientType           bStartOrient,              ///< [in ]: SM_OT_SAME    : StartTangent =  Start3DCurve->EvaluateTangent(StartParam)  <br>
                                                        ///<      : SM_OT_OPPOSITE: StartTangent = -Start3DCurve->EvaluateTangent(StartParam)  <br>
      const SmCurve        & crEnd3DCurve,              ///< [in ]: Curve connected to Base3DCurve end point                                   <br>
                                                        ///<      : Defines the end value of this VectorField                                  <br>
      double                 dEndParam,                 ///< [in ]: parameter of mated endPoint                                                <br>
      SmOrientType           bEndOrient,                ///< [in ]: SM_OT_SAME    : EndTangent =  End3DCurve->EvaluateTangent(EndParam)        <br>
                                                        ///<      : SM_OT_OPPOSITE: EndTangent = -End3DCurve->EvaluateTangent(EndParam)        <br>
      SmTangentField      *& rpNewTangentField          ///< [out]:                                                                            <br>

    );

    // eval TangentField(dParam) and optionally deriv dTangentField/dParam
    virtual SmStatus Evaluate
    (
      double     dParameter,              ///< [in ]: tgt param                                                                         <br>
      ULONG      lNumDerivatives,         ///< [in ]: 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .                                          <br>
      SmBoolean  bFromLeft,               ///< NotUsed: [in ]: if P is on interval boundary                                                      <br>
                                          ///<      : TRUE  = evaluate P in upper interval where P is on the left of the interval       <br>
                                          ///<      : FALSE = evaluate P in lower interval where P is on the right of the interval      <br>
      SmVector3d aPointAndDerivatives[],  ///< [out]: (pos, tang, 2nd, ...) sized:[lNumDerivatives+1]                                   <br>
      SmBoolean  bNonZeroTangents=TRUE    ///< NotUsed: [in ]: TRUE = replace zero tangent vectors with properly oriented tol sized vectors      <br>
                                          ///<      : FALSE= return exact tangent values                                                <br>
                                          ///<      : note: Surprisingly TRUE is the common choice because most tangent uses            <br>
                                          ///<      :       are for their direction (Binorm, SurfNorm comps), but when the              <br>
                                          ///<      :       tangent is being used for its magnitude (like an arc-length comp)           <br>
                                          ///<      :       then set this to FALSE.                                                     <br>
    ) const;
    
    // eval TangentField(dParam)
    virtual SmStatus EvaluatePoint(double dParameter, SmPoint3d & rTangentField) const;

    // get Curve Memory size - plus its attributes
    virtual ULONG GetMemoryUsed
    (
      ULONG    & rlMemoryAllocated,      ///< [out]: bigger size of all allocated memory in bytes    <br>
      SmMarkType eMarkType=SM_MT_NOMARK  ///< [in ]: uses without increment eMarkType value          <br>
    ) const ;

    virtual SmStatus WriteToDB 
    (
      SmDatabaseIO & rDB,                 ///< [in ]: target output stream                                  <br>
      ULONG          lDBVersionNumber     ///< [in ]: database version to get proper sequence of writes     <br>
    ) const;

    static  SmStatus ReadFromDB
    (
      SM_TYPE           lType,              ///< NotUsed: [in ]: Object type to be read                                                       <br>
      SmDatabaseIO    & rDB,                ///< [in ]: target output stream                                                         <br>
      ULONG             lDim,               ///< [in ]: curve image space dim, 2 or 3                                                <br>                                  
      const SmContext & crContext,          ///< [in ]: context for new object construction                                          <br>
      SmCurve         *&rpNewCurve,         ///< [out]: NULL on input = new object allocated in this routine built from stream data  <br>
                                            ///<      : NotNULL on input = pointer to an empty object to be filled by this routine   <br>
      ULONG             lDBVersionNumber    ///< [in ]: database version to get proper sequence of writes                            <br>
    );

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmTangentField,SmCrvOnSurf,SmTangentField_TYPE);

    virtual SmDisplayList * Draw
    (
      const SmExtent1d * pInterval=NULL,            ///< [in ]: Target Interval, NULL = Use Natural Interval, default:[NULL]                <br>
      SmBoolean          bAddToUIPickList = FALSE,  ///< [in ]: TRUE = Add this Curve to UI pick interface for debugging, default:[FALSE]   <br>
      SmPlane          * pOptOutPlane = NULL,       ///< NotUsed: [in ]: Draw on this plane, NULL=draw on z=0 plane, default:[NULL]                  <br>
      SmGfxArraySet    * pOptGfxSet = NULL          ///< [in ]: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.     <br>
                                                    ///<      : NULL to ignore. default:[NULL]                                              <br>
    ) const;


} ; // end class SmTangentField


#endif // !__SMTANGENTFIELD_H__


