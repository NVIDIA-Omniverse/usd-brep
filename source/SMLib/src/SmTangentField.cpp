// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmTangentField.cpp 
* PURPOSE: Source file for SmTangentField object.
**********************************************************************/

#include "StdAfx.h"

#include <SmBSplineCurve.h>
#include <SmSurface.h>
#include <SmTangentField.h>
#include <SmGraphicsExtern.h>
#include <SmDatabaseIO.h>
#include <SmContext.h>

/*******************************************************************//**
PURPOSE: Constructor for SmTangentField. 

NOTES: This will own the input UV curve but not the base surface
***********************************************************************/
SmTangentField::SmTangentField
  (const SmCurve    & crUVCurve,             // in : domain curve
   const SmSurface  & crBaseSurface,         // in : projection surface
   SmTangentDirType   eType,                 // in : for now only SM_INTERPOLATE_DIR is supported
   SmVector3d         vStartTangentField,    // in : TangentStartValue
   SmVector3d         vEndTangentField,      // in : TangentEndValue
   const SmExtent2d * cpUVDomain)            // in : opt subDomain of projection surface
 : SmCrvOnSurf(SM_CONST_CAST(SmCurve&,   crUVCurve),
               SM_CONST_CAST(SmSurface&, crBaseSurface), 
               cpUVDomain),
   m_eType(eType), 
   m_vStartTangentField(vStartTangentField),
   m_vEndTangentField  (vEndTangentField)
{

} // end SmTangentField::SmTangentField constructor

/*******************************************************************//**
PURPOSE: Destructor for SmTangentField. 

NOTES: 
***********************************************************************/
SmTangentField::~SmTangentField()
{

} // end SmTangentField::~SmTangentField destructor

/*******************************************************************//**
PURPOSE: Equality operator for SmTangentField

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmTangentField::operator==
  (const SmCurve& crOther) 
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = SmCrvOnSurf::operator ==(crOther) ;

  if(bRtn)
    {
      // OK to cast
      SmTangentField &rOther = (SmTangentField &)crOther ;

      // check equivalence of these objects
      bRtn = (   m_eType              == rOther.m_eType
              && m_vStartTangentField == rOther.m_vStartTangentField
              && m_vEndTangentField   == rOther.m_vEndTangentField) ;
    }

  // all done
  return bRtn ;

} // end SmTangentField::operator==

/*******************************************************************//**
PURPOSE: Convenience SmTangentField Constructor

NOTES:
   The start and end cross-tangent values are defined by tangent values on
   input endCurves.  This is convenient for designing NSidedPatch input
   that must share common cross-tangent values across curve boundaries.

   The tangent field evalutions will be forced to lie tangent to the
   BaseSurface at the BaseUVCurve(param) point.  
   
   The input Base3DCurve is only used for error checking. The ends
   of the Base3DCurve must connect to the ends of the Start3DCurve and
   End3DCurve.  The Base3DCurve is not used to create the returned
   SmTangentField nor is it stored.

NOTES: 
***********************************************************************/
SmStatus SmTangentField::Create
  (const SmContext      & crContext,           // in : context for new object construction
   double                 dThisApproxTol3d,    // in : max allowed 3D deviation from BaseCurve to BaseSurface
   const SmCurve        & crBase3DCurve,       // in : 3DCurve - only used for checking input
   const SmBSplineCurve & crBaseUVCurve,       // in : associated UVTrimCurve on
   const SmSurface      & crBaseSurface,       // in : associated Surface
    const SmCurve        & crStart3DCurve,     // in : Curve connected to Base3DCurve start point        
                                               //      Defines the start value of this VectorField       
   double                 dStartParam,         // in : parameter of mated startPoint
   SmOrientType           bStartOrient,        // in : SM_OT_SAME    : StartTangent =  Start3DCurve->EvaluateTangent(StartParam)
                                               //      SM_OT_OPPOSITE: StartTangent = -Start3DCurve->EvaluateTangent(StartParam)
    const SmCurve        & crEnd3DCurve,       // in : Curve connected to Base3DCurve end point 
                                               //      Defines the end value of this VectorField
   double                 dEndParam,           // in : parameter of mated endPoint             
   SmOrientType           bEndOrient,          // in : SM_OT_SAME    : EndTangent =  End3DCurve->EvaluateTangent(EndParam)
                                               //      SM_OT_OPPOSITE: EndTangent = -End3DCurve->EvaluateTangent(EndParam)              
   SmTangentField      *& rpNewTangentField)   // out:
{
  // StartCurve position and tangent
  SmVector3d sPV[2], sStartPnt, sEndPnt;
  SER(crStart3DCurve.Evaluate(dStartParam,1,TRUE,sPV));
  if (bStartOrient == SM_OT_OPPOSITE) sPV[1] = - sPV[1];
  SmVector3d sStartTangentField = sPV[1];

  // Base3DCurve start position
  SmExtent1d sIvl = crBase3DCurve.GetNaturalInterval();
  SER(crBase3DCurve.EvaluatePoint(sIvl.Evaluate(0.0),sStartPnt));

  // check state: Start points must match
  double dDist = sPV[0].DistanceBetween(sStartPnt);
  if (dDist > dThisApproxTol3d) 
    { SER(SM_ERR); // Ends don't match
    }

  // EndCurve position and tangent
  SER(crEnd3DCurve.Evaluate(dEndParam,1,TRUE,sPV));
  if (bEndOrient == SM_OT_OPPOSITE) sPV[1] = -sPV[1];
  SmVector3d sEndTangentField = sPV[1];

  // base3DCurve end position
  SER(crBase3DCurve.EvaluatePoint(sIvl.Evaluate(1.0),sEndPnt));
  
  // check state: End points must match
  dDist = sPV[0].DistanceBetween(sEndPnt);
  if (dDist > dThisApproxTol3d) 
    { SER(SM_ERR); // Ends don't match
    }

#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_SetColor(1,0,0);
      sStartTangentField.Draw(&sStartPnt);
      smgfx_SetColor(0,1,0);
      sEndTangentField.Draw(&sEndPnt);
      sm_GraphicsLoop();
    }
#endif

  // allocate SmTangentField
  SmTangentField * pTangentField = new(crContext) SmTangentField(crBaseUVCurve,
                                                                 crBaseSurface,
                                                                 SM_INTERPOLATE_DIR,  // for now, only value supported
                                                                 sStartTangentField,
                                                                 sEndTangentField);
  // all done
  rpNewTangentField = pTangentField;
  return SM_SUCCESS;

} // end SmTangentField::Create

/*******************************************************************//**
PURPOSE: Given the parametric value of the curve determine the
    corresponding tangent "value" on the curve.      

NOTES: The tangent "value" is interpolated between the
  stored StartTangent and EndTangent values and then projected
  to the tangent plane of the m_pSurface at the m_pUVCurve(dParameter) point.

  The interpolation is cleverly designed so that the twist vector of
  the tangent values is zero at the end point values.  That interpolation
  is given as

  T = .5 * cos(NormalizedParam) + .5
      so that 
      T(0) = 1.0 and dT(0)/dNormalizedParam = 0.0
      T(1) = 0.0 and dT(1)/dNormalizedParam = 0.0

  and
  Tangent = T*StartTangent + (1-T)*EndTangent

  and returned TangentValue is
  TangentValue(Param) = Tangent projection onto normal plane
                        of m_pSurface at m_pUVCurve(param).
  
***********************************************************************/
SmStatus SmTangentField::EvaluatePoint
  (double dParameter,
   SmPoint3d & rTangentField) 
  const
{
  // locals
  double     dParamTol = 1.0e-8;
  SmExtent1d sIvl      = GetNaturalInterval();

  // Start Param Value - return stored StartTangent
  if(dParameter < sIvl.GetMin() + dParamTol + SM_EFF_ZERO) 
    {
      rTangentField = m_vStartTangentField;
      return SM_SUCCESS;
    }

  // End Param Value - return stored EndTangent
  if (dParameter > sIvl.GetMax() - dParamTol - SM_EFF_ZERO) 
    {
      rTangentField = m_vEndTangentField;
      return SM_SUCCESS;
    }

  // linear normalized parameter
  double dT = (dParameter-sIvl.GetMin()) / sIvl.GetLength();

  // Cosine smoothed normalized parameter - dT/dParameter at 0 and 1 = 0
  // This yields a vanishing twist vector at the ends of the tangent interpolation.
  // note: dT = 1.0 at minParam and dT = 0.0 at maxParam 
  //              (opposite of normal expectations but not a problem) 
  dT *= SM_PI;
  dT = 0.5*cos(dT) + 0.5;

  // sTangentField(dParam) value is an interpolated value of the stored endTangentField values
  SmVector3d sTangentField = m_vStartTangentField*dT + m_vEndTangentField*(1.0-dT);

  //if (m_vStartTangentField.LengthSquared() < SM_EFF_ZERO_SQ) 
  //  {
  //    dT *= SM_PI/2.0;
  //    dT = 0.5*(1.0-smos_Cosine(dT));
  //    sTangent = dT*m_vEndTangentField;
  //  }
  //else if (m_vEndTangentField.LengthSquared() < SM_EFF_ZERO_SQ) 
  //  {
  //    dT *= SM_PI/2.0;
  //    dT = 1.0-smos_Sine(dT);
  //    sTangent = dT*m_vStartTangentField;
  //  }
  //else 
  //  {
  //    dT *= SM_PI;
  //    dT = 0.5*cos(dT) + 0.5;
  //    sTangent = m_vStartTangentField*dT + m_vEndTangentField*(1.0-dT);
  //  }

#ifdef SM_DEBUG_CODE
  if(   !m_pUVCurve->GetNaturalInterval().ContainsValue(dParameter, SM_EFF_ZERO))
    {
      SmExtent1d sInterval = m_pUVCurve->GetNaturalInterval() ;
      SM_ASSERT_MSG( sInterval.ContainsValue(dParameter, SM_EFF_ZERO),
                    _T("SmCrvOnSurf found a param for not contained in Curve Natural Interval.")) ;
    }
#endif // SM_DEBUG_CODE

  // Get NormalPlane to m_pSurface at the m_pUVCurve(dParameter) point
  SmPoint3d sPnt;
  SER(m_pUVCurve->EvaluatePoint(dParameter,sPnt));
  SmPoint2d sUV(sPnt[0], sPnt[1]);

#ifdef SM_DEBUG_CODE
  if(   !m_pSurface->GetNaturalUVDomain().ContainsPoint2d(sUV, SM_EFF_ZERO)
     && !m_pSurface->IsKindOf(SmOffsetSurface_TYPE))
    {
      SmExtent2d sDomain = m_pSurface->GetNaturalUVDomain() ;
      SM_ASSERT_MSG(   sDomain.ContainsPoint2d(sUV, SM_EFF_ZERO)
                    || m_pSurface->IsKindOf(SmOffsetSurface_TYPE),   // these are extendable surfaces
                    _T("SmTangentField found a param for which the CurveUV is not contained in the Surface Domain.")) ;
    }
#endif // SM_DEBUG_CODE

  SmVector3d sUnitNormal;
  SER(m_pSurface->EvaluateNormal(sUV,TRUE,TRUE,sUnitNormal));

  // return value = sTangentField projection onto the tangent plane
  rTangentField = sTangentField.ProjectToPlane(sUnitNormal);

  // all done
  return SM_SUCCESS;

} // end SmTangentField::EvaluatePoint


/*******************************************************************//**
PURPOSE: Given the parametric value of the curve determine the
    corresponding tangent "value" on the curve and optionally its
    derivative value (lNumDerivatives = 1).      

NOTES: 
***********************************************************************/
SmStatus SmTangentField::Evaluate
 (double     dParameter,              // in : tgt param
  ULONG      lNumDerivatives,         // in : 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .
  SmBoolean  bFromLeft,               // NotUsed: in : if P is on interval boundary
                                      //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                      //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmVector3d aPointAndDerivatives[],  // out: array or (pos, tang, 2nd deriv, ...), sized:[lNumDerivatives+1]
  SmBoolean  bNonZeroTangents)        // NotUsed: in : bNonZeroTangents : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
    const                             //      FALSE= return exact tangent values
                                      //      note: Surprisingly TRUE is the common choice because most tangent uses
                                      //            are for their direction (Binorm, SurfNorm comps), but when the 
                                      //            tangent is being used for its magnitude (like an arc-length comp)
                                      //            then set this to FALSE.
                                      //      default:[TRUE]
{
  SM_REF2(bFromLeft, bNonZeroTangents) ;
  if (lNumDerivatives == 0) 
    {
      SER(EvaluatePoint(dParameter, aPointAndDerivatives[0]));
      return SM_SUCCESS;
    }

  // get 1st derivative using finite differences
  if (lNumDerivatives == 1) 
    {
      // Currently just use divided diff.
      // Note, central diff is much more accurate.
      // We should also check if we're at the bottom or top of domain.
      SER(EvaluatePoint(dParameter,aPointAndDerivatives[0]));
      double dDeltaT = 1.0e-8;
      double dParam1 = dParameter + dDeltaT;
      SmVector3d sTangent1;
      SER(EvaluatePoint(dParam1,sTangent1));
      SmVector3d sDiff = sTangent1 - aPointAndDerivatives[0];
      aPointAndDerivatives[1] = sDiff / dDeltaT;
      return SM_SUCCESS;
    }
  SER(SM_ERR); // Can not go higher than 1st derivative 

  return SM_SUCCESS;

} // end SmTangentField::Evaluate

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmTangentField.

NOTES: Does not include the curve's attribute memory
***********************************************************************/
ULONG SmTangentField::GetMemoryUsed    // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated,       // out: bigger size of all allocated memory in bytes
   SmMarkType eMarkType)               // in : uses without increment eMarkType value
  const
{
  // in case this method is called directly - get a mark for attribute memory usage
  SmNewMarkAndLock sMarkLock ;
  if(eMarkType == SM_MT_NOMARK)
    {
      eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
    }

  // locals
  ULONG lUsed, lCurveAllocated=0, lSurfaceAllocated=0 ;

  // this + m_pNurb memory
  lUsed =   sizeof(*this) 
          + (  (m_pUVCurve && m_lOwnerFlag > 0)
             ? m_pUVCurve->GetMemoryUsed(lCurveAllocated, eMarkType) 
             : 0)
          + (  (m_pSurface && m_lOwnerFlag > 1) 
             ? m_pSurface->GetMemoryUsed(lSurfaceAllocated, eMarkType) 
             : 0) ;

  rlMemoryAllocated = sizeof(*this) + lCurveAllocated + lSurfaceAllocated ;

  // + attribute memory
  ULONG lThisAllocated ;
  lUsed += rlMemoryAllocated + this->GetAttributeMemoryUsed(lThisAllocated, 
                                                            eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated += lThisAllocated ;

  // + cache memory
  if ( m_pCacheObj )
  {
      lUsed += m_pCacheObj->GetMemoryUsed( lThisAllocated );
      rlMemoryAllocated += lThisAllocated;
  }

  // all done
  return(lUsed) ;

} // end SmTangentField::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Write SmTangentField to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmTangentField::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type
  SmFileType      eType    = rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();

  // curve locals
  SmTangentDirType lTanDirType = m_eType ;
      
  if (eType == SM_ASCII) 
    {
      rFileOut << lTanDirType                                      << " SmTangentField Tangent Dir Interpolation Type \n";
      rFileOut << m_vStartTangentField.x << " " << m_vStartTangentField.y << " " << m_vStartTangentField.z << " SmTangentField Start Point CrossTangent vector \n";
      rFileOut << m_vEndTangentField.x   << " " << m_vEndTangentField.y   << " " << m_vEndTangentField.z   << " SmTangentField End Point CrossTangent vector \n";
    }
  else 
    {
      SER(rDB.WriteLong(lTanDirType));

      SER(rDB.WriteDouble(m_vStartTangentField.x));
      SER(rDB.WriteDouble(m_vStartTangentField.y));
      SER(rDB.WriteDouble(m_vStartTangentField.z));

      SER(rDB.WriteDouble(m_vEndTangentField.x));
      SER(rDB.WriteDouble(m_vEndTangentField.y));
      SER(rDB.WriteDouble(m_vEndTangentField.z));

    }

  // Write the base object
  SmCrvOnSurf::WriteToDB(rDB, lDBVersionNumber) ;

  // all done
  return SM_SUCCESS ;

} // end SmTangentField::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmTangentField from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmTangentField::ReadFromDB
 (SM_TYPE           lType,              // NotUsed: in : curve type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  ULONG             lDim,               // in : curve image space dim, 2 or 3                                                                                    
  const SmContext & crContext,          // in : context for new object construction
  SmCurve         *&rpNewCurve,         // out:    NULL on input = new object allocated in this routine built from stream data
                                        //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG             lDBVersionNumber)   // in : database version to get proper sequence of writes
{
  SM_REF1(lType) ;
  // check input
  SER(  (   rpNewCurve == NULL
         || rpNewCurve->IsKindOf(SmTangentField_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmTangentField *pTangentField =   (rpNewCurve == NULL)
                                  ? new (crContext) SmTangentField()
                                  : (SmTangentField *)rpNewCurve ;

  // file type
  SmFileType     eType   = rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();
      
  // locals
  ULONG      lTangentDirType ;
  SmVector3d sStartTangentField ;
  SmVector3d sEndTangentField ;
             
  if (eType == SM_ASCII) 
    {
      rFileIn >> lTangentDirType ;                                                        rDB.GoToNextLine() ;
      rFileIn >> sStartTangentField.x >> sStartTangentField.y >> sStartTangentField.z ;   rDB.GoToNextLine() ;
      rFileIn >> sEndTangentField.x   >> sEndTangentField.y   >> sEndTangentField.z ;     rDB.GoToNextLine() ;
    }
  else 
    {
      SER(rDB.ReadLong(lTangentDirType));

      SER(rDB.ReadDouble(sStartTangentField.x));
      SER(rDB.ReadDouble(sStartTangentField.y));
      SER(rDB.ReadDouble(sStartTangentField.z));

      SER(rDB.ReadDouble(sEndTangentField.x));
      SER(rDB.ReadDouble(sEndTangentField.y));
      SER(rDB.ReadDouble(sEndTangentField.z));

    }

  // load the obj
  pTangentField->m_eType               = (SmTangentDirType) lTangentDirType ;
  pTangentField-> m_vStartTangentField = sStartTangentField ;
  pTangentField-> m_vEndTangentField   = sEndTangentField ;

  // set the output
  rpNewCurve = pTangentField ;

  // read the base type
  SmCrvOnSurf::ReadFromDB(SmCrvOnSurf_TYPE, rDB, lDim, crContext, rpNewCurve, lDBVersionNumber) ; 

  // all done
  return SM_SUCCESS;

} // end SmTangentField::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmTangentField::IsKindOf( SM_TYPE t ) const
{
  return ((SmTangentField_TYPE == t) ? TRUE : SmCrvOnSurf::IsKindOf( (t) ));
}


/*******************************************************************//**
PURPOSE: Dump out curve data.      

NOTES: 
***********************************************************************/
void SmTangentField::Dump
  () 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_WriteBuffer(_T("\nBegin SmTangentField::Dump()")) ;

  // output cache data
  SmCurve::Dump(FALSE) ;

  smos_sprintf(sBuff,       _T("\nSmTangentField = 0x%p "),this);
  smos_sprintf(sBuffForFile,_T("\nSmTangentField = %s "),  _T("notNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  SmVector3d sStartPV[2], sEndPV[2] ;
  SmExtent1d sIvl = m_pUVCurve->GetNaturalInterval() ;
  this->SmCrvOnSurf::Evaluate( sIvl.GetMin(), 1, TRUE, sStartPV) ;
  this->SmCrvOnSurf::Evaluate( sIvl.GetMax(), 1, TRUE, sEndPV) ;

  // m_eType
  smos_sprintf(sBuff, _T("\n  Type = %s of SM_CONSTANT_DIR, SM_INTERPOLATE_DIR, SM_PERPENDICULAR_DIR"),
               m_eType == SM_CONSTANT_DIR ?      _T("SM_CONSTANT_DIR")   
             : m_eType == SM_INTERPOLATE_DIR ?   _T("SM_INTERPOLATE_DIR") 
             : m_eType == SM_PERPENDICULAR_DIR ? _T("SM_PERPENDICULAR_DIR")
             : _T("UNKNOWN")) ;

  // m_vStartTangentField, m_vEndTangentField
  smos_WriteBuffer(_T("\n  Start TangentField  = ")) ; m_vStartTangentField.Dump() ;
  smos_WriteBuffer(_T("\n  Start Curve Tangent = ")) ; sStartPV[1].Dump() ;
  smos_WriteBuffer(_T("\n  End TangentField    = ")) ; m_vEndTangentField.Dump() ;
  smos_WriteBuffer(_T("\n  End Curve Tangent   = ")) ; sEndPV[1].Dump() ;

  smos_WriteBuffer(_T("\n End SmTangentField::Dump()\n")) ;

} // end SmTangentField::Dump

/*******************************************************************//**
PURPOSE: Draw TangentField.      

NOTES: 
***********************************************************************/
SmDisplayList * SmTangentField::Draw
  (const SmExtent1d * pInterval,             // in : Target Interval, NULL = Use Natural Interval, default:[NULL]
   SmBoolean          bAddToUIPickList,      // in : TRUE = Add this Curve to UI pick interface for debugging, default:[FALSE]
   SmPlane          * pOptOutPlane,          // NotUsed: in : Draw on this plane, NULL=draw on z=0 plane, default:[NULL]
   SmGfxArraySet    * pOptGfxSet)            // in : used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                             //      NULL to ignore. default:[NULL]
 const
{  
  SM_REF1(pOptOutPlane) ;
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

    // when asked - add this Curve to UI pick list
    SmCurve *pCurve = (SmCurve*) this;
    if(bAddToUIPickList) 
      { sm_GraphicsAddToBrepList(pCurve) ; }

    smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);
    SmExtent1d sIvl =   pInterval
                      ? *pInterval
                      : GetNaturalInterval();
    for (double dT=0.0; dT<=1.0; dT=dT+0.1) 
      {
        SmPoint3d sStartPnt;
        SmCrvOnSurf::EvaluatePoint(sIvl.Evaluate(dT), sStartPnt);
        SmVector3d sTangent;
        EvaluatePoint(sIvl.Evaluate(dT), sTangent);
        sTangent.Draw(&sStartPnt,NULL,pOptGfxSet);
      }
    smgfx_SetColor(0,1,0,pOptGfxSet);
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF3(pInterval, bAddToUIPickList, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmTangentField::Draw



