// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmIsoCurve.cpp
* PURPOSE: Source file for SmIsoCurve methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmBSplineCurve.h>
#include <SmIsoCurve.h>
#include <SmVector2d.h>
#include <SmSurface.h>
#include <SmDatabaseIO.h>
#include <SmLine.h>

/*******************************************************************//**
PURPOSE: create the UV IsoLine Obj for the C(s) UVTrimCurve part 
         of the S(C(s)) compound curve
    
NOTES: return curve type = SmLine_TYPE
***********************************************************************/
SmStatus SmIsoCurve::CreateUVIsoLine
 (SmCurve         *& pUVIsoLine, // out: Newly allocated UVIsoCurve, NULL on input
  const SmContext  * cpContext)  // in : Context for obj construction
  const
{
  // locals
  SmExtent2d sDomain = m_cpSurface->GetNaturalUVDomain() ;
  SmExtent1d sLineIvl ;
  SmPoint3d  sLinePoint = sDomain.GetMin() ;
  SmVector3d sLineUnitVec ;
  if     (m_eSurfParam == SM_SP_U) { sLineIvl     = sDomain.GetUInterval() ;
                                     sLinePoint.x = m_dIsoParameter ;
                                     sLineUnitVec.Set(0,1,0) ; 
                                   }
  else if(m_eSurfParam == SM_SP_U) { sLineIvl     = sDomain.GetVInterval() ;
                                     sLinePoint.y = m_dIsoParameter ;
                                     sLineUnitVec.Set(1,0,0) ;
                                   }
  else                             { SER_MSG(SM_ERR_INVALID_INPUT, _T("SmIsoCurve::CreateUVIsoLine: Bad value for m_eSurfParam")) ; }

  // create IsoLine
  pUVIsoLine = new (cpContext) SmLine(sLinePoint,        // in : P     of line = P + s*scale*unitV 
                                      sLineUnitVec,      // in : V     of line = P + s*scale*unitV
                                                         //      will be unitized before storing
                                      sLineIvl,          // in : limits on s, crInterval.Min <= s <= crInterval.Max
                                      1.0,               // in : scale of line = P + s*scale*unitV        
                                      2,                 // in : sizeof LinePoint and LineVector, default:[3]
                                      cpContext) ;       // in : required when making an automatic variable
                                                         //      optionally when using overloaded new.
  // all done
  return(SM_SUCCESS) ;

} // end SmIsoCurve::CreateUVIsoLine

/*******************************************************************//**
PURPOSE: Calculate the minimum continuity of the curve and the 
    continuities at each of the unique knots
    
NOTES: SmIsoCurves inherit the continuities from the surface.
***********************************************************************/
SmStatus SmIsoCurve::CalculateContinuities
  (SmContinuityType           & reMinContinuityInCurve,  // out:
   SmTArray<SmContinuityType> & rContinuitiesAtKnots,    // out:
   double                       dContinuityAngleTol)     // NotUsed: in :
  const
{
  SM_REF1(dContinuityAngleTol) ;
  // init output
  rContinuitiesAtKnots.ReSet() ;
  
  // pass the call along
  SmStatus eRtn = m_cpSurface->CalculateContinuities(m_eSurfParam, reMinContinuityInCurve, rContinuitiesAtKnots) ;
   
  // all done
  return eRtn ;

} // end SmIsoCurve::CalculateContinuities

/*******************************************************************//**
PURPOSE: Given the parameteric value of the curve determine the
    corresponding Euclidian point and optionally derivatives.

NOTES: For discontinuous points on the curve the derivatives
    may be obtained either from the left or from the right of the 
    discontinuity.
***********************************************************************/
SmStatus SmIsoCurve::Evaluate
 (double     dParameter,              // in : tgt param
  ULONG      lNumDerivatives,         // in : 0=pos, 1=pos+tang, 2=pos+tang+2nd, . . .
  SmBoolean  bFromLeft,               // in : if P is on interval boundary
                                      //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                      //      FALSE = evaluate P in lower interval where P is on the right of the interval
  SmVector3d aPointAndDerivatives[],  // out: array or (pos, tang, 2nd deriv, ...), sized:[lNumDerivatives+1]
  SmBoolean  bNonZeroTangents)        // in : TRUE = replace zero tangent vectors with properly oriented tol sized vectors
    const                             //      FALSE= return exact tangent values
                                      //      note: Surprisingly TRUE is the common choice because most tangent uses
                                      //            are for their direction (Binorm, SurfNorm comps), but when the 
                                      //            tangent is being used for its magnitude (like an arc-length comp)
                                      //            then set this to FALSE.
                                      //      default:[TRUE]
{ 
  // Note: the 'bOnlyUpperHalf' arg to Surface::Evaluate() must be
  // FALSE if the numbers of u- and v-derivs required are not equal.

  //SmVector3d sV[2][2];
  if (m_eSurfParam == SM_SP_U) 
    { // Constant U curve
      SmPoint2d sUV(m_dIsoParameter,dParameter);
      SER(m_cpSurface->Evaluate(sUV,
                                0,
                                lNumDerivatives,
                                m_bFromLeft,
                                bFromLeft,
                                FALSE,
                                aPointAndDerivatives, 
                                bNonZeroTangents));
//        SER(m_cpSurface->Evaluate(sUV,1,1,
//            m_bFromLeft,bFromLeft,FALSE,sV[0]));    
//        SmVector3d sDV = sV[0][1];
//        if (lNumDerivatives > 0) {
//            SM_ASSERT(sDV.DistanceBetween(aPointAndDerivatives[1]) < SM_EFF_ZERO);
//        }
    }
  else 
    { // Constant V curve
      SmPoint2d sUV(dParameter,m_dIsoParameter);

      //   (expanded SER Macro for debug convenience)
      // was: SER(m_cpSurface->Evaluate(sUV,
      //                                lNumDerivatives,
      //                                0,
      //                                bFromLeft,
      //                                m_bFromLeft,
      //                                FALSE,
      //                                aPointAndDerivatives, 
      //                                bNonZeroTangents));
        { SmStatus sErr = (m_cpSurface->Evaluate(sUV,
                                                 lNumDerivatives,
                                                 0,
                                                 bFromLeft,
                                                 m_bFromLeft,
                                                 FALSE,
                                                 aPointAndDerivatives, 
                                                 bNonZeroTangents));      
            if (sErr != SM_SUCCESS)  
              { smos_ErrorMessage(sErr,FILE_NAME,LINE_NUMBER,NULL,NULL,FUNC_NAME);  
                return sErr;          
              }   
         }
    }

  // all done
  return SM_SUCCESS;

} // end SmIsoCurve::Evaluate

/*******************************************************************//**
PURPOSE: Constructor for iso curve.  It loads values and checks
    to make sure that the iso parameter is valid.

NOTES: 
***********************************************************************/
SmIsoCurve::SmIsoCurve
  (const SmSurface & crSurface, 
   SmSurfParamType eSurfParam,
   double dIsoParameter,
   SmBoolean bFromLeft,               // in : if P is on interval boundary
                                      //      TRUE  = evaluate P in upper interval where P is on the left of the interval
                                      //      FALSE = evaluate P in lower interval where P is on the right of the interval
   SmBoolean bClampInput
  )
 : SmCurve(3), 
   m_cpSurface(&crSurface), 
   m_eSurfParam(eSurfParam),
   m_dIsoParameter(dIsoParameter), 
   m_bFromLeft(bFromLeft),
   m_bOwnsSurface(FALSE)
{
    if ( bClampInput == FALSE )
      return;

    SmExtent2d sUVDomain = crSurface.GetNaturalUVDomain();
    if (eSurfParam == SM_SP_U) 
      {
        if (sUVDomain.GetMin().x > dIsoParameter) 
          {
            m_dIsoParameter = sUVDomain.GetMin().x;
          }
        if (dIsoParameter > sUVDomain.GetMax().x) 
          {
            m_dIsoParameter = sUVDomain.GetMax().x;
          }
      }
    else 
      {
        if (sUVDomain.GetMin().y > dIsoParameter) 
          {
            m_dIsoParameter = sUVDomain.GetMin().y;
          }
        if (dIsoParameter > sUVDomain.GetMax().y) 
          {
            m_dIsoParameter = sUVDomain.GetMax().y;
          }
      }

} // end SmIsoCurve::SmIsoCurve constructor

/*******************************************************************//**
PURPOSE: Destructor for iso curve.

NOTES: 
***********************************************************************/
SmIsoCurve::~SmIsoCurve() 
{ 
  if(m_bOwnsSurface && m_cpSurface) { delete m_cpSurface ; m_cpSurface = NULL ; }

} // end SmIsoCurve::~SmIsoCurve destructor

/*******************************************************************//**
PURPOSE: Equality operator for SmIsoCurve

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmIsoCurve::operator==
  (const SmCurve& crOther) 
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = SmCurve::operator ==(crOther) ;

  if(bRtn)
    {
      // OK to cast
      SmIsoCurve &rOther = (SmIsoCurve &)crOther ;

      // check equivalence of these objects
      bRtn = (   (   ( m_cpSurface == rOther.m_cpSurface)
                  || ( m_cpSurface == NULL && rOther.m_cpSurface == NULL)
                  || (   m_cpSurface != NULL && rOther.m_cpSurface != NULL
                      && *m_cpSurface == *rOther.m_cpSurface))
              && m_eSurfParam  == rOther.m_eSurfParam
              && SM_IS_ZERO(m_dIsoParameter - rOther.m_dIsoParameter)
              && m_bFromLeft   == rOther.m_bFromLeft) ;
    }

  // all done
  return bRtn ;

} // end SmIsoCurve::operator==

/*******************************************************************//**
PURPOSE: Given the parametric value of the curve determine the
    corresponding Euclidian point on the curve.      

NOTES: This methods assumes the curve is not discontinuous
    in its position.  The parameter must be within the natural range
    of the curve.
***********************************************************************/
SmStatus SmIsoCurve::EvaluatePoint
  (double dParameter, 
   SmPoint3d & rPoint) 
  const
{ 
    if (m_eSurfParam == SM_SP_U) 
      { // Constant U curve
        SmPoint2d sUV(m_dIsoParameter,dParameter);
        SER(m_cpSurface->EvaluatePoint(sUV,rPoint));
      }
    else 
      { // Constant V curve
        SmPoint2d sUV(dParameter,m_dIsoParameter);
        SER(m_cpSurface->EvaluatePoint(sUV,rPoint));
      }
    return SM_SUCCESS;

} // end SmIsoCurve::EvaluatePoint

/*******************************************************************//**
PURPOSE: Get the interval of the iso curve by looking at the domain
    of its base surface.

NOTES: 
***********************************************************************/
SmExtent1d SmIsoCurve::GetNaturalInterval() const
{
    SmExtent2d sUVDomain = m_cpSurface->GetNaturalUVDomain();
    if (m_eSurfParam == SM_SP_U) 
      {
        return SmExtent1d(sUVDomain.GetMin().y,sUVDomain.GetMax().y);
      }
    return SmExtent1d(sUVDomain.GetMin().x,sUVDomain.GetMax().x);

} // end SmIsoCurve::GetNaturalInterval

/*******************************************************************//**
PURPOSE: Get list of unique knots and optionally knot multiplicities 
     of a curve.  It gets the knots of the underlying surface of the
     iso curve.

NOTES: This is the STEP compatible form of the knots not the
    typical knot vector associated with NURBS.
***********************************************************************/
SmStatus SmIsoCurve::GetKnots
  (SmTArray<double> & rKnots, 
   SmTArray<ULONG>  * pKnotMultiplicities,
   const SmExtent1d * pOptIvl)              // in : interval of interest, NULL=Natural Interval, default:[NULL] 
  const
{
  SmSurfParamType eSP = (m_eSurfParam == SM_SP_U) ? SM_SP_V : SM_SP_U;
  SER(m_cpSurface->GetKnots(eSP, rKnots, pKnotMultiplicities, pOptIvl));

  // all done
  return SM_SUCCESS;

} // end SmIsoCurve::GetKnots

/*******************************************************************//**
PURPOSE: Write SmIsoCurve to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmIsoCurve::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type, ASCII or BINARY
  SmFileType      eType    =  rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();

  // curve locals
  ULONG lSurfParam = m_eSurfParam ;
  
  if (eType == SM_ASCII) 
    {
      rFileOut << lSurfParam      << " SmIsoCurve Surf Param Flag \n" ;
      rFileOut << m_dIsoParameter << " SmIsoCurve IsoParameter \n" ;
      rFileOut << m_bFromLeft     << " SmIsoCurve From Left evaluation Flag \n" ;
    }
  else 
    {
      SER(rDB.WriteLong   ( lSurfParam )) ;  
      SER(rDB.WriteDouble ( m_dIsoParameter )) ;  
      SER(rDB.WriteBoolean( m_bFromLeft )) ;  
    }

  // compounding surface
  if (eType == SM_ASCII) { rFileOut << " IsoCurve->Surface \n"; }
  SER(rDB.WriteType(m_cpSurface->GetType())) ; 
  SER(m_cpSurface->WriteToDB(rDB, lDBVersionNumber)) ;

  // all done
  return SM_SUCCESS;

} // end SmIsoCurve::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmIsoCurve from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmIsoCurve::ReadFromDB
 (SM_TYPE           lType,              // NotUsed: in : curve type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  ULONG             lDim,               // NotUsed: in : curve image space dim, 2 or 3                                                                                    
  const SmContext & crContext,          // in : context for new object construction
  SmCurve         *&rpNewCurve,         // out:    NULL on input = new object allocated in this routine built from stream data
                                        //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG             lDBVersionNumber)   // in : database version to get proper sequence of writes
{
  SM_REF2(lType, lDim) ;
  // check input
  SER(  (   rpNewCurve == NULL
         || rpNewCurve->IsKindOf(SmIsoCurve_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmIsoCurve *pIsoCurve =   (rpNewCurve == NULL)
                            ? new (crContext) SmIsoCurve()
                            : (SmIsoCurve *)rpNewCurve ;
  // file type
  SmFileType     eType   = rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();

  // curve locals
  ULONG     lSurfParam ;
  double    dIsoParameter = 0.0;
  SmBoolean bFromLeft ;
      
  if (eType == SM_ASCII) 
    {
      rFileIn >> lSurfParam  ;    rDB.GoToNextLine() ;
      rFileIn >> dIsoParameter ;  rDB.GoToNextLine() ;
      rFileIn >> bFromLeft ;      rDB.GoToNextLine() ;

    }
  else 
    {
      SER(rDB.ReadLong   ( lSurfParam )) ;  
      SER(rDB.ReadDouble ( dIsoParameter )) ;  
      SER(rDB.ReadBoolean( bFromLeft )) ;  
    }

  // compounding surface locals
  SmSurface *pSurface=NULL ;
  SM_TYPE    lSurfaceType ;  

  // compounding surface
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SER(rDB.ReadType(lSurfaceType)) ; 
  SER(SmSurface::ReadFromDB(lSurfaceType, rDB, crContext, pSurface, lDBVersionNumber)) ;
  
  // load obj
  pIsoCurve->m_cpSurface     = pSurface ;   
  pIsoCurve->m_eSurfParam    = (SmSurfParamType) lSurfParam ;  
  pIsoCurve->m_dIsoParameter = dIsoParameter ;
  pIsoCurve->m_bFromLeft     = bFromLeft ;
  pIsoCurve->m_bOwnsSurface  = TRUE ; 
  
  // all done
  rpNewCurve = pIsoCurve ;
  return SM_SUCCESS;

} // end SmIsoCurve::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmIsoCurve::IsKindOf( SM_TYPE t ) const
{
  return ((SmIsoCurve_TYPE == t) ? TRUE : SmCurve::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmIsoCurve::Dump
  () 
 const
{
  smos_WriteBuffer(_T("\nBegin SmIsoCurve::Dump()")) ;

  // output cache data
  SmCurve::Dump(FALSE) ;

  smos_WriteBuffer(_T(" End SmIsoCurve::Dump()\n")) ;

} // end SmIsoCurve::Dump
