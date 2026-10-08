// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmCompositeCurve.cpp
* PURPOSE: Source file for SmCompositeCurve methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmCompositeCurve.h>
#include <SmGeomUtility.h>
#include <SmOffsetCurve.h>


/*******************************************************************//**
PURPOSE: Equality operator for SmCompositeCurveSegment

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmCompositeCurveSegment::operator==
  (const SmCompositeCurveSegment& crOther)
 const
{
  // low work
  if(this == &crOther) { return TRUE ; }

  // first check the base
  SmBoolean bRtn = TRUE ;

  if(bRtn)
    {
      // check equivalence of these objects
      bRtn = ( (m_eTransition == crOther.m_eTransition) && (m_bSameSense == crOther.m_bSameSense)
              && (   ( m_pParentCurve == crOther.m_pParentCurve)
                  || ( m_pParentCurve == NULL && crOther.m_pParentCurve == NULL)
                  || (    m_pParentCurve != NULL && crOther.m_pParentCurve != NULL
                      && *m_pParentCurve == *crOther.m_pParentCurve))
              && SM_IS_ZERO(m_dGapStart - crOther.m_dGapStart)
              && SM_IS_ZERO(m_dGapEnd   - crOther.m_dGapEnd) ) ;
    }

  // all done
  return bRtn ;

} // end SmCompositeCurveSegment::operator==

/*******************************************************************//**
PURPOSE: Write SmCompositeCurveSegment to given output stream.

NOTES:
***********************************************************************/
SmStatus SmCompositeCurveSegment::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes
 const
{
  // file type, ASCII or BINARY
  SmFileType      eType    =  rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();

  // locals
  SmBoolean bHasCurve           = (m_pParentCurve != NULL) ;   // always write curve - when read it becomes owned
  ULONG     lTransitionType     = m_eTransition ;

  if (eType == SM_ASCII)
    {
      rFileOut << lTransitionType            << " SmCompositeCurveSegment Continuity Transition Type \n";
      rFileOut << m_bSameSense               << " SmCompositeCurveSegment Same Sense \n" ;
      rFileOut << m_dGapStart                << " SmCompositeCurveSegment Gap Start \n" ;
      rFileOut << m_dGapEnd                  << " SmCompositeCurveSegment Gap End \n" ;
      rFileOut << bHasCurve                  << " SmCompositeCurveSegment bHasCurve \n" ;
    }
  else
    {
      SER(rDB.WriteLong   (lTransitionType)) ;
      SER(rDB.WriteBoolean(m_bSameSense)) ;
      SER(rDB.WriteDouble (m_dGapStart)) ;
      SER(rDB.WriteDouble (m_dGapEnd)) ;
      SER(rDB.WriteBoolean(bHasCurve)) ;
    }

  // has curve
  if(bHasCurve) { ULONG lCurveDim = m_pParentCurve->GetDim() ;
                  if (eType == SM_ASCII)  { rFileOut << " SmCompositeCurve->CurveBeingConverted \n"; }
                  SER(rDB.WriteType(m_pParentCurve->GetType(), &lCurveDim)) ;
                  SER(m_pParentCurve->WriteToDB(rDB, lDBVersionNumber)) ;
                }
  else          { if (eType == SM_ASCII)  { rFileOut << "Has No Copy of SmPolarConversion->CurveBeingConverted \n"; }
                }

  // all done
  return SM_SUCCESS;

} // end SmCompositeCurveSegment::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmCompositeCurveSegment from a given stream

NOTES:
***********************************************************************/
SmStatus SmCompositeCurveSegment::ReadFromDB
 (SmDatabaseIO             & rDB,              // in : target output stream
  const SmContext          & crContext,        // in : context for new object construction
  SmCompositeCurveSegment  *&rpNewSegment,     // out:    NULL on input = new object allocated in this routine built from stream data
                                               //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG                      lDBVersionNumber) // in : database version to get proper sequence of writes
{
  // check input
  SER(  (   rpNewSegment == NULL
         || rpNewSegment->IsKindOf(SmCompositeCurveSegment_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmCompositeCurveSegment *pNewSegment =   (rpNewSegment == NULL)
                                         ? new SmCompositeCurveSegment()
                                         : (SmCompositeCurveSegment *)rpNewSegment ;

  // file type
  SmFileType     eType   =  rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();

  // locals
  ULONG      lTransitionType ;
  SmBoolean  bSameSense ;
  double     dGapStart = 0.0;
  double     dGapEnd = 0.0;
  SmBoolean  bHasCurve ;

  if (eType == SM_ASCII)
    {
      rFileIn >> lTransitionType ; rDB.GoToNextLine() ;
      rFileIn >> bSameSense ;      rDB.GoToNextLine() ;
      rFileIn >> dGapStart ;       rDB.GoToNextLine() ;
      rFileIn >> dGapEnd ;         rDB.GoToNextLine() ;
      rFileIn >> bHasCurve ;       rDB.GoToNextLine() ;
    }
  else
    {
      SER(rDB.ReadLong   ( lTransitionType )) ;
      SER(rDB.ReadBoolean( bSameSense )) ;
      SER(rDB.ReadDouble ( dGapStart )) ;
      SER(rDB.ReadDouble ( dGapEnd )) ;
      SER(rDB.ReadBoolean( bHasCurve )) ;
    }

  // ParentCurve
  SmCurve *pParentCurve = NULL ;
  SM_TYPE  lParentType ;
  ULONG    lParentDim ;

  // contained curve
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  if (bHasCurve)         { SER(rDB.ReadType(lParentType, &lParentDim)) ;
                           SER(SmCurve::ReadFromDB(lParentType,
                                                   rDB,
                                                   lParentDim,
                                                   crContext,
                                                   pParentCurve,
                                                   lDBVersionNumber)) ;
                         }

  // load object
  pNewSegment->m_eTransition  = (SmContinuityType) lTransitionType ;
  pNewSegment->m_bSameSense   = bSameSense ;
  pNewSegment->m_pParentCurve = pParentCurve ;
  pNewSegment->m_dGapStart    = dGapStart ;
  pNewSegment->m_dGapEnd      = dGapEnd ;
  pNewSegment->m_bOwnsCurve   = TRUE ;

  // set output
  rpNewSegment = pNewSegment ;

  // all done
  return SM_SUCCESS;

} // end SmCompositeCurveSegment::ReadFromDB

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmCompositeCurveSegment::Dump
  (ULONG i)
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("\n%3ld "), i);
  smos_WriteBuffer(sBuff);
  this->Dump();

} // end SmCompositeCurveSegment::Dump

/*******************************************************************//**
PURPOSE: Pretty Print SmCompositeCurveSegment

NOTES: One line dump - no LineFeeds to support SmCompositeCurve::Dump pretty printing
***********************************************************************/
void SmCompositeCurveSegment::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // Curve Segment Length
  double dApproxLength = m_pParentCurve->ApproximateLength(m_pParentCurve->GetNaturalInterval(),5) ;

  smos_sprintf(sBuff,       _T("SmCompositeCurveSegment:[0x%p]"), this) ;
  smos_sprintf(sBuffForFile,_T("SmCompositeCurveSegment:[%s]"), _T("notNULL") );
  smos_WriteBuffer(sBuff, sBuffForFile);

  smos_sprintf(sBuff, _T(", Orient:[%s], GapStart:[%16.16lf], Len:[%16.16lf], GapEnd:[%16.16lf], EndPivot:[%s], OwnsCurve:[%s]"),
             m_bSameSense ? _T("Same") : _T("Opposite"),
             m_dGapStart,
             dApproxLength,
             m_dGapEnd,
             SmGetContinuityTypeString(m_eTransition),  
             m_bOwnsCurve ? _T("TRUE") : _T("FALSE")) ;
  smos_WriteBuffer(sBuff);

  // no nested Curve dump - that's done explicitly in SmCompositeCurve::Dump
  // // Parent Curve
  // smos_WriteBuffer(_T("\n Parent Curve")) ;
  // m_pParentCurve->Dump() ;

} // end SmCompositeCurveSegment::Dump

/*******************************************************************//**
PURPOSE: Constructor for a composite curve region.

NOTES:
***********************************************************************/
SmCompositeCurveRegion::SmCompositeCurveRegion
  (const SmTArray<ULONG> & crOuterLoops,
   const SmTArray<SmCompositeCurve*> & crCompositeCurves)
{
    m_vOuterLoops.Append(crOuterLoops);
    m_vCompositeCurves.Append(crCompositeCurves);

} // end SmCompositeCurveRegion::SmCompositeCurveRegion



/*******************************************************************//**
PURPOSE: Make a copy of this attribute in a virtual way such that
    a general call can be made to copy all attributes.

NOTES:
***********************************************************************/
SmAttribute * SmOffsetMapAttribute::MakeCopy(const SmContext & crContext) const
{
    SmOffsetMapAttribute *pRet = new (crContext) SmOffsetMapAttribute(*this);
    return pRet;

} // end SmOffsetMapAttribute::MakeCopy

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmOffsetMapAttribute::IsKindOf( SM_TYPE t ) const
{
  return ((SmOffsetMapAttribute_TYPE == t) ? TRUE : SmAttribute::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmOffsetMapAttribute::Dump
  ()
 const
{
    smos_WriteBuffer(_T("SmOffsetMapAttribute - "));

} // end SmOffsetMapAttribute::Dump


/*******************************************************************//**
PURPOSE: Get a pointer to the composite curve segment structure.

NOTES:
***********************************************************************/
const SmCompositeCurveSegment * SmCompositeCurve::GetCurveSegment
  (ULONG lSegmentIndex)
 const
{
    if (lSegmentIndex >= m_lNumSegments) {
        SE(SM_ERR);
        return NULL;
    }
    return &m_paSegments[lSegmentIndex];

} // end SmCompositeCurve::GetCurveSegment


// Local routine:
/*******************************************************************//**
PURPOSE: Convert SmCurves to SmBSplineCurves.

NOTES: Transfers an Attribute of type SM_AI_OFFSETMAP.
***********************************************************************/
static void sm_convert_to_BSplineCurves
  (const SmContext           & crContext,     // in : context for new object construction
   const SmTArray<SmCurve*>  & crCurves,      // in : input array of curves to be approximated by BSpline curves
   double                      dApproxTol3D,  // in : Max allowed distance between input and approx curves
   SmBoolean                   bMatchPz,      // in : TRUE = approxCurve(param) within 3DApproxTol of ThisCurve(param) for all param values
                                              //      FALSE= ApproxCurve(param) within 3d ApproxTol of ThisCurve(AnyParam)
                                              //      note: FALSE produces lower control point count curves for slightly more cost.
   SmTArray<SmBSplineCurve*> & rBSplCurves,   // out: array of input curve approximations to tol = dApproxTol3D
   SmTArray<double>          & rdMaxGap3d,    // out: max error seen in making approximations
   SmBoolean      bJustCopyBSplines=FALSE)    // in : TRUE = if BSplineCurve just copy it (preserves CrvParams)
                                              //      FALSE= approximate BSplineCurves (changes CrvParams), NonBSplineCrvs always Approximated
                                              //      default:[FALSE]
{
  // init output
  rBSplCurves.ReSet();
  rdMaxGap3d.ReSet();

  // locals
  SmExtent1d       sIvl;
  SmBSplineCurve  *pBSplApprox;
  double           dMaxGap3d = 0;
  SmTArray<double> sBreaks;
  ULONG            lNumCurves = crCurves.GetSize();
  ULONG            lCrvIdx;

  // for every trimmed offset curve in crCurves - approx with BSplineCurve
  for(lCrvIdx = 0; lCrvIdx < lNumCurves; lCrvIdx++ )
    {
      SmCurve *pThisCrv = crCurves[lCrvIdx];
      sIvl              = pThisCrv->GetNaturalInterval();

      // next offset curve interval
      sBreaks.ReSet();
      sBreaks.Add( sIvl.GetMin() );
      sBreaks.Add( sIvl.GetMax() );
      pBSplApprox = NULL;

      // approximate offset curve with a BSpline curve
      SmStatus eStat = pThisCrv->ApproximateCurve(crContext,           // in : memory context for new object
                                                  sBreaks,             // in : input curve params exactly interpolated by approx curve.
                                                                       //      Always representing Interval end points
                                                                       //      and commonly also representing orig curve discontinuities.
                                                  dApproxTol3D,        // in : Max dist allowed between approx and original curves.
                                                  dMaxGap3d,           // out: Actual Max dist between approx and original curves.
                                                                       //      note: value is not exact, only based on sampling.
                                                  pBSplApprox,         // out: the new BSpline
                                                  TRUE,                // in : TRUE = create derived type analytics (SmLine, SmCircle, ...) when possible
                                                                       //      FALSE= don't, default:[TRUE]
                                                  bMatchPz,            // in : TRUE = approxCurve(param) within 3DApproxTol of ThisCurve(param) for all param values
                                                                       //      FALSE= ApproxCurve(param) within 3d ApproxTol of ThisCurve(AnyParam)
                                                                       //      default:[FALSE], FALSE produces lower control point count curves for slightly more cost.
                                                  bJustCopyBSplines) ; // in : TRUE = if BSplineCurve just copy it (preserves CrvParams)
                                                                       //      FALSE= approximate BSplineCurves (changes CrvParams), NonBSplineCrvs always Approximated
                                                                       //      default:[FALSE]
      if(eStat != SM_SUCCESS )
        {
          // Delete anything it might have created, but don't quit.
          if(pBSplApprox != NULL )
              { delete pBSplApprox; pBSplApprox = NULL; }
        }

      // when approx BSpline curve was build
      if(pBSplApprox != NULL )
        {
          // Transfer the OffsetMap attribute
          SmAttribute *pAttr = pThisCrv->FindAttribute( SM_AI_OFFSETMAP );

          // when used for offsetting - will have an OFFSETMAP attribute
          if(pAttr != NULL )
            {
              pThisCrv->RemoveAttribute( pAttr, TRUE ); // True: doesn't delete it.
              pBSplApprox->AddAttribute( pAttr );
            }

          // set outputs
          rBSplCurves.Add( pBSplApprox );

          rdMaxGap3d.Add( dMaxGap3d ) ;
        } // end successful approx curve construction check
    } // end iter offset curves, converting to SmBSplineCurve.

} // end sm_convert_to_BSplineCurves

/*******************************************************************//**
PURPOSE: Calculate the bounding box of a Composite curve
         as the union of the SmCompositeCurveSegment bounding boxes

NOTES:
***********************************************************************/
SmStatus SmCompositeCurve::CalculateBoundingBox
  (const SmExtent1d & crInterval,  // in : target interval of interest - this is used
   SmExtent3d       * pBBox,       // out: Axis alligned box, NULL to ignore, default:[NULL]
   SmPseudoBox      * pPseudoBox,  // out: Non-axis aligned box, NULL to ignore, default:[NULL]
   SmPolarBox       * pPolarBox,   // out: Curve tangent vector field bounding box, NULL to ignore, default:[NULL]
   SmBoolean          bExpandBox)  // in : TRUE = expand BBoxes prior to return
                                   //      FALSE= don't
                                   //      default:[TRUE]
  const
{
  // init output
  pBBox->Init() ;
  if(pPseudoBox) { pPseudoBox->Init() ; }
  if(pPolarBox)  { pPolarBox->ReSet() ; }

  // no work - no segments
  if(m_lNumSegments <= 0)
    {
      return SM_SUCCESS ;
    }

  // locals
  ULONG ii ;
  SmBoolean   bFirstUnion = TRUE ;
  SmExtent3d  sThisBBox,      * pThisBBox      = pBBox      ? &sThisBBox      : NULL ;
  SmPseudoBox sThisPseudoBox, * pThisPseudoBox = pPseudoBox ? &sThisPseudoBox : NULL ;
  SmPolarBox  sThisPoloarBox, * pThisPolarBox  = pPolarBox  ? &sThisPoloarBox : NULL ;
  double      sIvlOffset = 0.0 ;
  SmExtent1d  sTgtIvl, sSegIvl ;

  // for every interval
  for(ii=0;ii<m_lNumSegments;ii++,sIvlOffset += sSegIvl.GetLength())
    {
      sSegIvl = m_paSegments[ii].m_pParentCurve->GetNaturalInterval() ;

      // get target Ivl for this segment
      sSegIvl.Translate(sIvlOffset) ;
      sSegIvl.Intersect(crInterval, sTgtIvl) ;

      // when TgtIvl is not negative
      if(!sTgtIvl.HasNegativeLength())
        {
          // get its bounding boxes
          m_paSegments[ii].GetCurve()->CalculateBoundingBox(sTgtIvl,
                                                            pThisBBox,
                                                            pThisPseudoBox,
                                                            pThisPolarBox,
                                                            bExpandBox) ;
          // union bounding boxes
          if(pThisBBox)      { pBBox->Union(*pThisBBox, *pBBox) ; }
          if(pThisPseudoBox) { pPseudoBox->Union(*pThisPseudoBox,
                                                  bFirstUnion ? pThisPseudoBox->GetBasis() : pPseudoBox->GetBasis(),
                                                 *pPseudoBox) ;
                             }
          if(pThisPolarBox)  { pPolarBox->Union(*pThisPolarBox, *pPolarBox) ; }


          // mark state - done with first union
          bFirstUnion = FALSE ;

        } // end segment interval is within requested interval check
    } // end iter every segment accumulating bounding boxes

  // all done
  return(SM_SUCCESS) ;

} // end SmCompositeCurve::CalculateBoundingBox

/*******************************************************************//**
PURPOSE: Get composite curve interval as the sum of all SmCompositeCurveSegment
          intervals.

NOTES: This is an equivalent interval found by stacking all the segment
intervals end to end.
***********************************************************************/
SmExtent1d SmCompositeCurve::GetNaturalInterval
  ()
 const
{
  // no work - no segments
  if(m_lNumSegments <= 0)
    {
      return SmExtent1d() ;
    }

  // locals
  ULONG ii ;
  SmExtent1d sSegIvl = m_paSegments[0].m_pParentCurve->GetNaturalInterval() ;
  SmExtent1d sIvl(sSegIvl.GetMin(), sSegIvl.GetMax()) ;

  // for every subsequent interval
  for(ii=1;ii<m_lNumSegments;ii++)
    {
      sSegIvl = m_paSegments[ii].m_pParentCurve->GetNaturalInterval() ;

      // increment the effective interval
      sIvl.SetMinMax(sIvl.GetMin(), sIvl.GetMax() + sSegIvl.GetLength()) ;

    } // end iter every segment adding up intervals

  // all done
  return(sIvl) ;

} // end SmCompositeCurve::GetNaturalInterval

/*******************************************************************//**
PURPOSE: Get list of unique knots and optionally knot multiplicities
     of a BSpline curve.

NOTES: This is an equivalent knot vector made by concatenating all
 the segment knots to one another after shifting the knot values
 to be consequtive.
***********************************************************************/
SmStatus SmCompositeCurve::GetKnots
  (SmTArray<double> & rKnots,               // out: Unique knot vector
   SmTArray<ULONG>  * pKnotMultiplicities,  // out: multiplicity value for each knot
   const SmExtent1d * pOptIvl)              // in : interval of interest, NULL=Natural Interval, default:[NULL]
  const
{
  // init output
  rKnots.ReSet() ;
  if(pKnotMultiplicities) { pKnotMultiplicities->ReSet() ; }

  // no work - no segments
  if(m_lNumSegments <= 0)
    {
      return SM_SUCCESS ;
    }

  // locals
  ULONG ii, jj ;
  SER(m_paSegments[0].m_pParentCurve->GetKnots(rKnots, pKnotMultiplicities, pOptIvl)) ;
  ULONG lLastIndex   = rKnots.GetSize() ;

  SmTArray<double> sTheseKnots ;
  SmTArray<ULONG>  sTheseKnotMults ;
  SmTArray<ULONG>  *pTheseKnotMults = pKnotMultiplicities ? &sTheseKnotMults : NULL ;

  // for every subsquent interval
  for(ii=1;ii<m_lNumSegments;ii++)
    {
      SmCurve *pLastCrv = m_paSegments[ii-1].m_pParentCurve ;
      SmCurve *pThisCrv = m_paSegments[ii].m_pParentCurve ;

      // next segment knots
      SER(pThisCrv->GetKnots(sTheseKnots, pTheseKnotMults)) ;

      // knot offset to shift These knots consecutive to previous knots
      double dAdj = rKnots.GetAt(lLastIndex) - sTheseKnots.GetAt(0) ;

      // adjust Knot count for transition knot
      if(pTheseKnotMults)
        {
          pKnotMultiplicities->SetAt(lLastIndex,
                                     smos_Min(pKnotMultiplicities->GetAt(lLastIndex) + pTheseKnotMults->GetAt(0),
                                              smos_Max(pLastCrv->GetDegree(), pThisCrv->GetDegree()) - 1)) ;
        }

      // accumulate the rest of the knots
      for(jj=1;jj<sTheseKnots.GetSize();jj++)
        {
          rKnots.Add(sTheseKnots.GetAt(jj) + dAdj) ;
          if(pTheseKnotMults)
            {
              pKnotMultiplicities->Add(pTheseKnotMults->GetAt(jj)) ;
            }
        }

      // update the lastIndex
      lLastIndex = rKnots.GetSize() - 1 ;

    } // end iter every segment adding up intervals

  // trim output to OptIvl
  if(pOptIvl)
    {
      // for every knot
      for(ii=rKnots.GetSize()-1;ii>=0;ii--)
        {
          // remove the ones outside of pOptIvl
          if(!pOptIvl->IsContainedBy(rKnots.GetAt(ii)))
            {
              rKnots.RemoveAt(ii, 1) ;
              if(pKnotMultiplicities)
                {
                  pKnotMultiplicities->RemoveAt(ii, 1) ;
                }
            } // end knot out of range check
        } // end iter every knot
    } // end given optIvl check

  // all done
  return(SM_SUCCESS) ;

} // end SmCompositeCurve::GetKnots

/*******************************************************************//**
PURPOSE: Approximate each segment with a hermite curve approximation
  and then create a single curve by joing the segments.

NOTES: Returns SM_SUCCESS and rpNewBSplineCurve == NULL
       for curves that are C0 in the interior - those are illegal curves within SMLib
***********************************************************************/
SmStatus SmCompositeCurve::ApproximateCurve
  (const SmContext        & crContext,           // in : memory context for new object
   const SmTArray<double> & crBreakParams,       // NotUsed: in : input curve params exactly interpolated by approx curve.
                                                 //      Always representing Interval end points
                                                 //      and commonly also representing orig curve discontinuities.
   double                   d3DApproxTol,        // in : Max dist allowed between approx and original curves.
   double                 & rdMaxGap3d,          // out: Actual Max dist between approx and original curves.
                                                 //      note: value is not exact, only based on sampling.
   SmBSplineCurve        *& rpNewBSplineCurve,   // out: the new BSpline
   SmBoolean                bOptCreateAnalytics, // NotUsed: in : TRUE = create derived type analytics (SmLine, SmCircle, ...) when possible
                                                 //      FALSE= don't, default:[TRUE]
   SmBoolean          bOptMatchParameterization, // NotUsed: in : TRUE = approxCurve(param) within 3DApproxTol of ThisCurve(param) for all param values
                                                 //      FALSE= ApproxCurve(param) within 3d ApproxTol of ThisCurve(AnyParam)
                                                 //      default:[FALSE], FALSE produces lower control point count curves for slightly more cost.
   SmBoolean          bJustCopyBSplines)         // NotUsed: in : TRUE = if this Curve is a BSpline just copy it
                                                 //      FALSE= approximate the curve
                                                 //      default:[FALSE]
  const
{
  SM_REF4(crBreakParams, bOptCreateAnalytics, bOptMatchParameterization, bJustCopyBSplines) ;
  // init output
  rdMaxGap3d        = 0.0 ;
  rpNewBSplineCurve = NULL ;

  // locals
  ULONG ii ;
  SmContinuityType           eMinContType = SM_CT_CINFINITY ;
  SmTArray<SmCurve *>        sOrigSegments ;
  SmTArray<SmBSplineCurve *> sApproxSegments ;
  SmTArray<double>           sMaxGap3d(m_lNumSegments, NULL, m_lNumSegments) ;
  sMaxGap3d.SetAll(0) ;

  // gather the segments
  for(ii=0;ii<m_lNumSegments;ii++)
    {
      // this curve
      SmCurve *pSegCurve = m_paSegments[ii].m_pParentCurve ;

      // build segment array
      sOrigSegments.Add(pSegCurve) ;

      // min continuity
      if(    (ii < m_lNumSegments - 1)
          && (m_paSegments[ii].m_eTransition < eMinContType))
        {
          eMinContType = m_paSegments[ii].m_eTransition ;
        }

      // max gap
      if(    (ii < m_lNumSegments - 1)
          && (m_paSegments[ii].m_dGapEnd > rdMaxGap3d))
        {
          rdMaxGap3d = m_paSegments[ii].m_dGapEnd ;
        }

    } // end iter every segment gathering curves

  // approximate every segment as a BSpline
  sm_convert_to_BSplineCurves( crContext,                 // in : context for new object construction
                               sOrigSegments,             // in : input array of curves to be approximated by BSpline curves
                               d3DApproxTol,              // in : Max allowed distance between input and approxi curves
                               FALSE,                     // in : TRUE = approxCurve(param) within 3DApproxTol of ThisCurve(param) for all param values
                                                          //      FALSE= ApproxCurve(param) within 3d ApproxTol of ThisCurve(AnyParam)
                                                          //      note: FALSE produces lower control point count curves for slightly more cost.
                               sApproxSegments,           // out: array of input curve approximations to tol = d3DApproxTol
                               sMaxGap3d,                 // out: max error seen in making approximations
                               TRUE) ;                    // in : TRUE = Just copy BSplines, FALSE = Approximate them
                                                          //      default:[FALSE]
  // max achieved tol
  for(ii=0;ii<m_lNumSegments;ii++)
    {
      if(sMaxGap3d[ii] > rdMaxGap3d)
        {
          rdMaxGap3d = sMaxGap3d[ii] ;
        }
    }

  // join the Approximations as a single curve
  SmStatus eRtn = SmBSplineCurve::CreateByJoining(crContext, sApproxSegments, NULL, rpNewBSplineCurve) ;

  // check for valid curve - many might be C0 in their interiors
  if(FALSE == rpNewBSplineCurve->AssertValid())
    {
      // the curve is not valid
      if(rpNewBSplineCurve) { delete rpNewBSplineCurve ; rpNewBSplineCurve = NULL ; }
    }

  // all done
  return(eRtn) ;

} // end SmCompositeCurve::ApproximateCurve

/*******************************************************************//**
PURPOSE: Compute the orientation of loop relative to its projection
            into a plane and optionally the 2d boundary.

NOTES: SM_OT_SAME     = CCW Outer Loop
       SM_OT_OPPOSITE = CW  Inner Loop

   If the given loop has zero area, this will return SM_SUCCESS,
   with reLoopOrient set to SM_OT_UNKNOWN.

METHOD ---
  Sample all the curves to make a polygon.
  Walk the polygon to get its signed area from edge to edge.

  When the signed area is positive the walk was around a CCW loop
    enclosing some area - return SM_OT_SAME for an 'outerLoop'
  When the signed area is negative the walk was around a CW loop
    enclosing some area - return SM_OT_OPPOSITE for an 'innerLoop'

***********************************************************************/
SmStatus SmCompositeCurve::ComputeProjectedLoopOrientation
  (const SmTArray<SmCurve*>  & crCurves,     // in : Ordered list of
                                             //      curves that go around the loop.
   const SmTArray<SmBoolean> & crSenses,     // in : Specifies the orientation
                                             //      of the curve relative to the loop.  TRUE means that the
                                             //      curve has the same orientation as the loop.
   const SmVector3d          & crNormal,     // in : Normal of the projection plane
   SmOrientType              & reLoopOrient, // out: SM_OT_SAME     = CCW walk (outer loop)
                                             //      SM_OT_OPPOSITE = CW  walk (inner loop)
                                             //      SM_OT_UNKNOWN  = zero area, neither CW or CCW.
   SmExtent2d                * p2DBounds)    // out: UVBoundingBox of all UVTrimCurve SamplePoints
                                             //      NULL to ignore.
{
  // init outputs
  reLoopOrient = SM_OT_UNKNOWN;

  // locals
  SmTArray<SmPoint3d> sPolygon;
  SmTArray<SmCurve *> sCurveOfPoints;
  SmTArray<SmPoint3d> sBSCPoly;
  double dAngleDeg = 100.0;

  ULONG jj, lIter;

  // (see below)
  double dPrevRatio = 0;

  // Now make a polygon from the trim boundary.

  // Do 3 iterations, getting a more accurate tessellation each time.
  for(lIter=0; lIter<3; lIter++ )
    {
      // init locals - reduce angle limit
      sPolygon.ReSet();
      sCurveOfPoints.ReSet();
      sBSCPoly.ReSet();
      dAngleDeg = dAngleDeg / 10.0;  // For a more accurate tessellation.

      // Tessellate every curve.
      ULONG lThisCurve, lNumCurves = crCurves.GetSize();
      for(lThisCurve=0; lThisCurve < lNumCurves; lThisCurve++ )
        {
          SmCurve *pBSC = crCurves[lThisCurve];

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe)
            {
              SM_DUMP_AND_ASSERT_VALID(pBSC) ;

              if (lThisCurve==0)
                { smgfx_Erase(); }
              smgfx_ChangeColor(lIter != 0);
              pBSC->DrawWDeriv(pBSC->GetNaturalInterval(),0);
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
          // For lines, we need only the two end points.
          if (pBSC->IsLinear())
            {
              SmPoint3d sPnt;
              SmExtent1d sIvl = pBSC->GetNaturalInterval();
              SER(pBSC->EvaluatePoint(sIvl.GetMin(),sPnt));
              sBSCPoly.ReSet();
              sBSCPoly.Add(sPnt);
              SER(pBSC->EvaluatePoint(sIvl.GetMax(),sPnt));
              sBSCPoly.Add(sPnt);
            }
          else // not a line branch - add tessellate by bisection sample points
            {
              // tesselate curve to current dAngleDeg - skip failures
              if (SM_SUCCESS != pBSC->Tessellate(pBSC->GetNaturalInterval(),
                                                 0.0, dAngleDeg, 3,
                                                 NULL, &sBSCPoly))
                {
                  WARN(_T("UVTrimCurve failed to tessellate")) ;
                  continue;
                }
            } // end Getting Curve Sample Points

          // when asked - build UVBoundingBox of all SamplePoints
          if(p2DBounds )
            {
              for(jj=0; jj<sBSCPoly.GetSize(); jj++ )
                {
                  p2DBounds->AddPoint2d( SmPoint2d( sBSCPoly[jj].x, sBSCPoly[jj].y ));
                }
            } // end bulding UVBoundingBox

          // add every CurveSamplePoints found to growing polygon
          ULONG lNumCrvPts = sBSCPoly.GetSize();
          for(jj=0; jj<lNumCrvPts; jj++ )
            {
              if (crSenses[lThisCurve] == TRUE)
                {
                  sPolygon.Add( sBSCPoly[jj] );
                }
              else
                {
                  // Opposite orientation of curve
                  sPolygon.Add( sBSCPoly[ lNumCrvPts-jj-1 ] );
                }
              sCurveOfPoints.Add(pBSC);

            } // end iter every CurveSamplePoint

        } // end iter every curve


      // Ok, now we have tessellated the loop of curves into sPolygon,
      // we're going to calculate its signed area.

      // Check polygon has more than 3 points
      ULONG lNumPoints = sPolygon.GetSize();
      SM_ASSERT( lNumPoints > 3 );

      // We're going to calculate a 3d bounding box, to compare against
      // our calculated loop area (to make a relative check instead of absolute).
      SmExtent3d sBBox;

      // We'll also need to know which is the smallest component of the
      // direction vector, for reasons explained later.
      ULONG lSmallestComponent = 0;
      double dSmallestVal = smos_Fabs( crNormal.x );
      if(smos_Fabs( crNormal.y ) < dSmallestVal )
        {
          lSmallestComponent = 1;
          dSmallestVal = smos_Fabs( crNormal.y );
        }
      if(smos_Fabs( crNormal.z ) < dSmallestVal )
        {
          lSmallestComponent = 2;
        }

      // init polygon walk parameters
      double dTotalArea = 0.0;

      // 'previous' point is size-2: polygon is closed, so pt[size-1] == pt[0].
      SmPoint3d sPoint1 = sPolygon[ 0 ];

      // Accumulate bounding box.
      sBBox.AddPoint3d( sPoint1 );

      // Don't use vectors from the origin to the points, in case it's far away;
      // using a point near the polygon will give much better accuracy.
      // The midpoint would be good, but since we don't have that,
      // we can just use the start point.
      SmPoint3d sBasePt = sPolygon[0];

      SmVector3d sVec1( sPoint1 - sBasePt );
      SmVector3d sVec1Proj, sVec2Proj;
      SER( smgu_VectorProjectToPlane( sVec1, crNormal, sVec1Proj ));

      // For every pair of polygon points - total the polygon area.
      ULONG lThisPoint;
      for(lThisPoint=1; lThisPoint<lNumPoints; lThisPoint++ )
        {
          SmPoint3d sPoint2 = sPolygon[lThisPoint];
          sBBox.AddPoint3d( sPoint2 );
          SmVector3d sVec2( sPoint2 - sBasePt );
          SER( smgu_VectorProjectToPlane( sVec2, crNormal, sVec2Proj ));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
          if (bDebugMe2)
            {
              smgfx_SetLook( 1,4, 1,0,0); sPoint1.Draw(); sm_GraphicsLoop();
              smgfx_SetLook( 1,3, 0,0,1); sPoint2.Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // Compute and total the signed area of a triangle between
          // the base point, sPoint1, and sPoint2.
          // (Actually, this number is twice the area, but that's ok.)
          SmVector3d sCross = sVec1Proj * sVec2Proj;
          dTotalArea += sCross.Dot( crNormal );

          // continue the walk
          sPoint1   = sPoint2;
          sVec1Proj = sVec2Proj;

        } // end iter every polygon point

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe)
        {
          smgfx_Erase();
          for (jj=0; jj<crCurves.GetSize(); jj++)
            {
              if (jj%3==0) smgfx_SetColor(1,0,0);
              if (jj%3==1) smgfx_SetColor(1,1,0);
              if (jj%3==2) smgfx_SetColor(0,0,1);
              SmCurve *pBSC = crCurves[jj];
              pBSC->DrawWDeriv(pBSC->GetNaturalInterval(),0);
              sm_GraphicsLoop();
            }
        }
#endif // SM_DEBUG_CODE

      // when totalArea > 0 we have a CCW OuterLoop
      //      totalArea < 0 we have a CW InnerLoop
      //      totalArea == 0 we can't say - try again with more tessellation points
      //        totalArea == 0 can happen on closed surfaces with missing SeamEdges.
      //          consider the area enclosed by a cylinder whose outer loop is just the
      //          circle boundary at the top of the cylinder.  That's a single line
      //          in UVSpace not enclosing any area.

      // For relative sizes: compare to 'area' of bounding box.
      // Note, box is often 2d, but can be 3d, so what's an 'area'?
      // We just need something rough.  Use the diagonal plane (one that
      // contains four corner points and crosses the interior) that is
      // most nearly normal to the input vector.  Its area is the extent
      // in the direction most nearly perpendicular to the vector times
      // diagonal of the other two directions.  The direction most nearly
      // perpendicular is the smallest component of the vector.

      double dExtent = 0.0;
      SmVector2d sDiagonal;
      switch(lSmallestComponent )
        {
        case 0:
          dExtent = sBBox.XLength();
          sDiagonal.Set( sBBox.YLength(), sBBox.ZLength() );
          break;
        case 1:
          dExtent = sBBox.YLength();
          sDiagonal.Set( sBBox.ZLength(), sBBox.XLength() );
          break;
        case 2:
          dExtent = sBBox.ZLength();
          sDiagonal.Set( sBBox.XLength(), sBBox.ZLength() );
          break;
        }

      // About the tolerances here.  There have been problems on very small
      // geometry, where areas (which are lengths squared) look like zero,
      // with noise.
      // We have calculated TotalArea (signed), and the area of its bounding
      // box. TotalArea (in absolute value) will be no greater than BoxArea.
      // (Well, except that dTotalArea is actually twice the contained area,
      // so the ratio has an upper limit of 2.0.)  So there should be no issue
      // of a zero-divide.
      //
      // The only problem that might arise is if the area is actually 'zero',
      // so that both quantities are 'zero'.  In that case, both quantities
      // are likely to be noise, so the result will be noise1 / noise2.
      // [Fillet test 202]
      //
      // So the only thing we'll guard against is the case of a zero-area loop.
      // We know we want a relative test (check the sign of dRatio), but we
      // want to make sure that it's not meaningless.
      //
      // A zero-area loop might result in a bounding box that is either a line
      // or a point (plus noise).  The bounding box area is calculated as the
      // product of dExtent and dDiagLen, two sides of a rectangle.  If it's
      // due only to noise, then one or both of those quantities will be 'zero';
      // we check that against SM_EFF_ZERO.
      //
      // But what about a legitimate case where both dimensions are so small
      // that their product is as small as 'noise'?  To avoid that, we'll check
      // before they are multiplied together: if either dimension of the bounding
      // box is smaller than SM_EFF_ZERO, then the box will be considered to have
      // zero area.  And we know that's an upper bound on the area of the loop.
      // [bd 090624]

      // If either dimension of the bounding box is 'zero', then the area is zero.
      double dTol1    = SM_EFF_ZERO;
      double dDiagLen = sDiagonal.Length();
      double dBoxArea =(dExtent > dTol1 && dDiagLen > dTol1 ) ? dExtent * dDiagLen : 0.0;

      // Tolerance for the ratio check: these are now both area measurements,
      // and we've decided that they are legitimate (not zero plus noise),
      // so use SM_EFF_ZERO squared.  Also, dTotalArea is a summation of many numbers,
      // so that error can accumulate; multiply by the number of terms in the sum.
      // (But note that BoxArea should always be at least as big as TotalArea
      // (to a factor of 2), and we've taken out the degenerate case, so this
      // tolerance shouldn't even matter: BoxArea is either big enough, or set to zero.)
      double dTol2  = SM_EFF_ZERO_SQ * lNumPoints;
      double dRatio =(dBoxArea > smos_Fabs( dTotalArea ) * dTol2 ) ? dTotalArea / dBoxArea : 0.0;

      // However, the sampling algorithm itself introduces lots of noise,
      // especially on the first pass.  So use a much bigger tol for this.
      // That just causes another iteration at a tighter sampling.
      // [bd 6/3/11]
      double dTol3 = 0.01;

      // By the third time through, the sampling is quite dense, so the result
      // should be quite precise, so use a tighter tolerance.
      if(lIter >= 2 )
        { dTol3 = 0.000001; }

      // Also check to see if the calculated area changes sign on successive
      // iterations -- that's a sure sign that it should be zero.
      if(lIter > 0 )
      {
        if(dRatio * dPrevRatio < 0.0 )
        {
          reLoopOrient = SM_OT_UNKNOWN;
          return SM_SUCCESS;
        }
      }
      dPrevRatio = dRatio;


      if(dRatio > dTol3 )
        {
          reLoopOrient = SM_OT_SAME;  // Positive means CCW.
          return SM_SUCCESS;
        }
      else if(dRatio < -dTol3 )
        {
          reLoopOrient = SM_OT_OPPOSITE;
          return SM_SUCCESS;
        }
// No reason to do this; see the return values. [bd 090624]
//    else
//      {
//        // Tentatively set result without any tolerance consideration,
//        // and try again with more tessellation points.
//        reLoopOrient =  (dTotalArea >= 0.0)
//                       ? SM_OT_SAME
//                       : SM_OT_OPPOSITE ;
//      }
    } // end iter 3 times

  // When we arrive here - the algorithm couldn't classify the loop.
  // This means that the loop has zero area.
  // We will not return SM_ERR here, because nothing went wrong in
  // this routine, but we will communicate the fact that there is
  // no orientation with SM_OT_UNKNOWN.

  reLoopOrient = SM_OT_UNKNOWN;
  return SM_SUCCESS;

} // end SmCompositeCurve::ComputeProjectedLoopOrientation

/*******************************************************************//**
PURPOSE: Constructor for the composite curve.

NOTES:
***********************************************************************/
SmCompositeCurve::SmCompositeCurve
  (ULONG                       lDimension,   // in : 3d or 2d, should match curve dimensions
   const SmTArray<SmCurve*>  & crCurves,     // in : ordered array of curve segments
   SmBoolean                   bIsClosed,    // in : TRUE = last curve is connected to first, FALSE = open
   const SmTArray<SmBoolean> * cpSenses,     // in : bit for each curve, TRUE = reverse curve in composite
                                             //      NULL = every curve assumed to be same sense.
   const SmTArray<double>    * cpGaps)       // in : gap for each curve from its end to next curve's start
                                             //      NULL = every cached gap set to 0.0
 : SmCurve(lDimension),
   m_lNumSegments(crCurves.GetSize()),
   m_paSegments(NULL),
   m_bClosed(bIsClosed)
{
  // block size and allocation for m_paSegments array
  ULONG lSize    = sizeof(SmCompositeCurveSegment) * m_lNumSegments;

  // okay to use smos_Calloc on static class (SmCompositeCurveSegment) objects.
  m_paSegments   = (SmCompositeCurveSegment*)smos_Calloc(lSize,1);

  // for every curve segment - load the SmCompositeCurveSegment object
  for (ULONG i=0; i<m_lNumSegments; i++)
    {
      SmBoolean     bSameSense = TRUE;
      if (cpSenses) bSameSense = (SmBoolean)(*cpSenses)[i];

      m_paSegments[i].m_eTransition  = SM_CT_C0 ;
      m_paSegments[i].m_bSameSense   = bSameSense ;
      m_paSegments[i].m_pParentCurve = (SmCurve*)crCurves[i] ;
      m_paSegments[i].m_dGapStart    = 0.0 ;
      m_paSegments[i].m_dGapEnd      = 0.0 ;

     // old code that was confusing and called extra constructors/destructors
     // m_paSegments[i] = SmCompositeCurveSegment(((SmCurve*)crCurves[i]), bSameSense);
    }

  // when given gaps - load the gap values
  if (cpGaps)
    {
      double prevGap;
      if (bIsClosed)
          prevGap = (double)(*cpGaps)[m_lNumSegments-1];
      else
          prevGap = SM_BIG_DOUBLE;

      for (ULONG i=0; i<m_lNumSegments; i++)
        {
          m_paSegments[i].m_dGapStart = prevGap;
          m_paSegments[i].m_dGapEnd   = prevGap = (double)(*cpGaps)[i];
        }
    }

} // end SmCompositeCurve::SmCompositeCurve constructor

/*******************************************************************//**
PURPOSE: Copy constructor for a Composite curve.

NOTES:
    This constructor should only be used in the case of a new allocated
    curve or one which is being declared on the stack.  It should not
    be used to assign to a preexisting curve.
***********************************************************************/
SmCompositeCurve::SmCompositeCurve
  (const SmCompositeCurve & crSourceCurve) // in : Curve to copy
: SmCurve(crSourceCurve),
   m_lNumSegments(crSourceCurve.m_lNumSegments),
   m_paSegments(NULL),
   m_bClosed(crSourceCurve.m_bClosed)
{
  // block size and allocation for m_paSegments array
  ULONG lSize    = sizeof(SmCompositeCurveSegment) * m_lNumSegments;

  // okay to use smos_Calloc on static class (SmCompositeCurveSegment) objects.
  m_paSegments   = (SmCompositeCurveSegment*)smos_Calloc(lSize,1);

  // for every curve segment - load the SmCompositeCurveSegment object
  for (ULONG i=0; i<m_lNumSegments; i++)
    {
      m_paSegments[i].m_eTransition  = crSourceCurve.m_paSegments[i].m_eTransition ;
      m_paSegments[i].m_bSameSense   = crSourceCurve.m_paSegments[i].m_bSameSense ;

      crSourceCurve.m_paSegments[i].m_pParentCurve->Copy(*crSourceCurve.GetContext(),
                                                         m_paSegments[i].m_pParentCurve) ;

      m_paSegments[i].m_dGapStart    = crSourceCurve.m_paSegments[i].m_dGapStart ;
      m_paSegments[i].m_dGapEnd      = crSourceCurve.m_paSegments[i].m_dGapEnd ;

      m_paSegments[i].m_dGapStart    = crSourceCurve.m_paSegments[i].m_dGapStart ;
      m_paSegments[i].m_dGapEnd      = crSourceCurve.m_paSegments[i].m_dGapEnd ;
    }

} // end SmCompositeCurve::SmCompositeCurve copy constructor

/*******************************************************************//**
PURPOSE: Equality operator for SmCompositeCurve

NOTES: Call base equivalence to check type and then check
       members for equivalence
***********************************************************************/
SmBoolean SmCompositeCurve::operator==
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
      SmCompositeCurve &rOther = (SmCompositeCurve &)crOther ;

      // check equivalence of these objects
      bRtn = (   m_lNumSegments == rOther.m_lNumSegments
              && m_bClosed      == rOther.m_bClosed) ;

      // check segments
      if(bRtn)
        {
          ULONG ii ;
          for(ii=0;ii<m_lNumSegments;ii++)
            {
              bRtn &= m_paSegments[ii] == rOther.m_paSegments[ii] ;
            }
        }
    }

  // all done
  return bRtn ;

} // end SmCompositeCurve::operator==

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
class SmCurveItem
{
public:
    SmCurve *     m_pCurve;          // init : NULL
    SmBoolean     m_bSense;          // init : TRUE
    SmPoint3d     m_Start;           // init : [SM_UNDEF_DOUBLE, SM_UNDEF_DOUBLE, SM_UNDEF_DOUBLE]
    SmPoint3d     m_End;             // init : [SM_UNDEF_DOUBLE, SM_UNDEF_DOUBLE, SM_UNDEF_DOUBLE]
    SmCurveItem * m_pNext;           // init : NULL
    double        m_DistToNext;      // init : SM_BIG_DOUBLE
    SmBoolean     m_bUsed;           // init : FALSE

    // constructor
    SmCurveItem() : m_pCurve(NULL),
                    m_bSense(TRUE),
                    m_pNext(NULL),
                    m_DistToNext(SM_BIG_DOUBLE),
                    m_bUsed(FALSE)
                    { }

} ; // end class class SmCurveItem

#define AppendSeg(pSegInList, pNewSeg) \
    ((pNewSeg)->m_pNext    = (pSegInList)->m_pNext, \
     (pSegInList)->m_pNext = pNewSeg)


/*******************************************************************//**
PURPOSE: Turn a NULL terminated linked list of curves into a composite curve
            optionally adding line segments between large curve end-point gaps.

            Very large gaps between curve segments are not connected. Those gaps
            start and stop individual composite curves returned in the output.

            Closed composites are returned when the last point of the last segment
            is within tolerance of the first point of the first segment.

NOTES: Only called from SmCompositeCurve::BuildCompositesFromCurves().

     Various strategies were invisioned to connect curve end-points
     based on the gap size between the curve ends, only one was built;
     optionally, close an extra large gap by a line segment.

   Implementation:
     if      gap < dSamePointTolerance,    curves are connected as is.
     else if gap < DistanceToCreateLine,   curves are connected by a line segment bounding curve end-points
                                           if(DistanceToCreateLine == 0) don't add line segments.
     else if gap > max(dSamePointTolerance, dSamePointTolerance) don't connect the curves.

   As Invisioned:
     if      gap < dSamePointTolerance,    curves are connected as is.
     else if gap < DistanceToAverage,      curves are connected to a vertex placed at the average location - NOT IMPLEMENTED
     else if gap < DistanceToExtendTrim,   curves are trimmed to a vertex placed at the average location   - NOT IMPLEMENTED
     else if gap < DistanceToCreateLine,   curves are connected by a line segment bounding curve end-points
     else if gap < DistanceToCreateBlend,  curves are connected by a blend segment bounding curve end-points - NOT IMPLEMENTED
     default: curves are not connected, ending one composite curve and starting another.

***********************************************************************/
static SmStatus sm_FlushCrv
  (const SmContext & crContext,               // in : context to hold new geometry
   SmCurveItem     * pListHead,               // in : pListHead->Next starts composite curve, pListHead is not part of the composite.
   double            dTol,                    // in : max tolerance between matched curve end-points
   SmBoolean ,                                //       bMakeCurvesHomogeneous  = approx all curves as degree 3 NUBs
   double    ,                                //       dThisApproxTol3d = approx tolerance
   double            dSamePointTolerance,     // in : if      gap < dSamePointTolerance, connect
   double    ,                                // in : else if gap < dOptDistanceToAverage,    snap to avg and connect - not implemented
   double    ,                                // in : else if gap < dOptDistanceToExtendTrim, trim and connect        - not implemented
   double            dOptDistanceToCreateLine,// in : else if gap < dOptDistanceToCreateLine, insert line seg and connect,
                                              //      0.0 to ignore
   double    ,                                // in : else if gap < dOptDistanceToCreateBlend, insert blend and connect
   SmTArray<SmCompositeCurve*> & rComposites) // out: final composite curve
{
  // locals
  SmTArray<SmCurve*>  sCurves;  // final list of curves to be built into a composite - things are done for gaps
  SmTArray<SmBoolean> sSenses;  // for each curve, TRUE = add curve as is, FALSE = add curve reversed
  SmTArray<double>    sGaps;
  double              dDist;
  SmCurveItem       * first = NULL;
  SmCurveItem       * curr  = pListHead;

  // while curve remains open
  while (curr->m_pNext != NULL)
    {
      SmCurveItem * next = NULL ;

      SmBoolean bCreateCompositeCurve = FALSE;
      SmBoolean bIsClosed             = FALSE;

      // increment curr. curr is the last curve in the linked-list
      curr = curr->m_pNext;

      // add next curve segment to arrays
      sCurves.Add(curr->m_pCurve);
      sSenses.Add(curr->m_bSense);

      // remember the first curve segment (pListHead->Next starts the composite, pListHead is not in the composite)
      if (sCurves.GetSize() == 1)
        { first = curr; }

      // if gap to next curve segment is smaller than tolerance.
      if (curr->m_DistToNext < dTol)
        {
          // let next be the next input curve
          next = curr->m_pNext;
        }

      else // gap to next curve segment is larger than tolerance
        {
          // this composite curve is done
          bCreateCompositeCurve = TRUE;

          // if the first curve segment is same as the last curve segment
          // note: this branch will only run for single curve composites because the function is only
          //       called from SmCompositeCurve::BuildCompositesFromCurves()
          //       which only passes in LinkedLists in which each curve segment is listed just once.
          if (first == curr)
            {
              // Calculate Length of the segment
              SmExtent1d sNaturalInterval = curr->m_pCurve->GetNaturalInterval();
              dDist                       = curr->m_pCurve->ApproximateLength(sNaturalInterval, 5);

              // skip 1st curves too small
              if (dDist < dSamePointTolerance)
                {
                  // skip this curve - remove it from scratch arrays and start over
                  sCurves.RemoveAll();
                  sSenses.RemoveAll();
                  sGaps.RemoveAll();
                  continue;
                }
              else // add this curve's next gap to the nextGap array
                {
                  sGaps.Add(curr->m_DistToNext);
                }
            } // end first segment check

          // Check if it closes the composite curve.
          if ((dDist = curr->m_End.DistanceBetween(first->m_Start)) < dTol)
            {
              curr->m_DistToNext = dDist;
              bIsClosed          = TRUE;
              next               = first;
            }
        } // end if next gap is larger than tolerance branch

      // Here's a chance to fill in gaps when asked and if needed
      //   if      NextGap < dSamePointTolerance - connect as is, add next gap to gap array
      //   else if NextGap < dOptDistanceToCreateLine - create line seg, add line and gaps to curve and gap arrays
      //   else Composite curve is ending - add next gap to gap array

      // if NextGap < SamePointTolerance - connect as is, add next gap to gap array
      if (curr->m_DistToNext < dSamePointTolerance)
        {
          // add gap to array - curves will be connected as is with this gap.
          sGaps.Add(curr->m_DistToNext);
        }

      //      else if(  dOptDistanceToAverage != 0.0
      //               && curr->m_DistToNext < dOptDistanceToAverage)
      //        {
      //          // no action implemented yet
      //        }

      //      else if(  dOptDistanceToExtendTrim != 0.0
      //               && curr->m_DistToNext < dOptDistanceToExtendTrim)
      //        {
      //          // no action implemented yet
      //        }

      // else if NextGap < dOptDistanceToCreateLine
      else if(   dOptDistanceToCreateLine != 0.0
              && curr->m_DistToNext < dOptDistanceToCreateLine)
        {
          // fill gap with line segment - update curve and gap arrays
          SmBSplineCurve * pLine = NULL ;
          SmBSplineCurve::CreateLineSegment(crContext,
                                            3,
                                            curr->m_End,
                                            next->m_Start,
                                            pLine);
          sGaps.Add(0.0);
          sCurves.Add(pLine);
          sSenses.Add(TRUE);
          sGaps.Add(0.0);
        }

      //      else if(  dOptDistanceToCreateBlend != 0.0
      //               && curr->m_DistToNext < dOptDistanceToCreateBlend)
      //        {
      //          // no action implemented yet
      //        }

      else // all branches skipped - this is the end of an open composite curve
           // no gap to fill
        {
          // place final gap size on gap list
          sGaps.Add(curr->m_DistToNext);
        }

      // when a composite curve needs to be created
      if (bCreateCompositeCurve)
        {
          // turn sCurves array into composite curve
         SmCompositeCurve * pCompCurve = NULL;

         // build the SmCompositeCurve
         //  (expect to always use this branch; unless there is an error - sGaps.GetSize == sCurves.GetSize)
         if (sGaps.GetSize() == sCurves.GetSize())
             pCompCurve = new(crContext) SmCompositeCurve(3,sCurves,bIsClosed,&sSenses,&sGaps);
         else // ignore the end-gaps  (don't expect to ever use this branch)
             pCompCurve = new(crContext) SmCompositeCurve(3,sCurves,bIsClosed,&sSenses,NULL);

          // add composite curve to output
          rComposites.Add(pCompCurve);

          // reset scratch arrays
          sCurves.ReSet();
          sSenses.ReSet();
          sGaps.ReSet();

        } // end need to output a composite curve check
    } // end while curves exist to be connected into a composite

  // all done - empty input list and return
  pListHead->m_pNext = NULL ;
  return SM_SUCCESS;

} // end sm_FlushCrv

/*******************************************************************//**
PURPOSE:  Connect a set of unordered and possibly unconnected individual
    Bspline curves into one (or more) composite curves based on the
    end-point gap distances.  Can return closed or open composite curves.

METHOD --- Curves are connected to neighbors whose end-points are closest.

    when the smallest end-gap < dSamePointTolerance the curves are connected as is.
    when the smallest end-gap < dOptDistanceToCreateLine a small line segment
                                is connected between the curves filling in the gap.
                                0.0 to ignore.
    when the smallest end-gap > max(dSamePointTolerance, dOptDistanceToCreateLine)
                                the curve is not connected.

    when dOptDistanceToCreateLine == 0.0, no lines will be added between curves.
    Composite curves may be closed but must have more than 1 curve in each composite curve.

NOTES:

    Others options were invisioned for filling in large gaps as:
     if      gap < dSamePointTolerance,    curves are connected as is.
     else if gap < DistanceToAverage,      curves are connected to a vertex placed at the average location - NOT IMPLEMENTED
     else if gap < DistanceToExtendTrim,   curves are trimmed to a vertex placed at the average location   - NOT IMPLEMENTED
     else if gap < DistanceToCreateLine,   curves are connected by a line segment bounding curve end-points
     else if gap < DistanceToCreateBlend,  curves are connected by a blend segment bounding curve end-points - NOT IMPLEMENTED
     else if gap > largest tolerance, curves are not connected.

  Please contact customer service if your application can use any of these unimplmented features.
***********************************************************************/
SmStatus SmCompositeCurve::BuildCompositesFromCurves
  (const SmContext & crContext,                // in : context for new object construction
   const SmTArray<SmBSplineCurve*> & crCurves, // in : unordered curves to connect into composites
                                               //      Included curves move to the output composites;
                                               //      tiny omitted curves remain caller-owned.
   SmBoolean ,                                 // in : bMakeCurvesHomogeneous : TRUE = Approx curves with deg 3 NUBs
   double    ,                                 // in : dThisApproxTol3d = tol used if an approximation is required.
   double         dSamePointTolerance,         // in : if      gap < dSamePointTolerance, connect
   double    ,                                 // in : else if gap < dOptDistanceToAverage,    snap to avg and connect - not implemented
   double    ,                                 // in : else if gap < dOptDistanceToExtendTrim, trim and connect        - not implemented
   double         dOptDistanceToCreateLine,    // in : else if gap < dOptDistanceToCreateLine, insert line seg and connect
   double    ,                                 // in : else if gap < dOptDistanceToCreateBlend, insert blend and connect
   SmTArray<SmCompositeCurve*> & rComposites)  // out :
{
  // check input
  SER_MSG((dSamePointTolerance > 0.0 ? SM_SUCCESS : SM_ERR), _T("Must have a nonzero dSamePointTolerance value")) ;
  SER_MSG((dOptDistanceToCreateLine == 0.0 || dOptDistanceToCreateLine > dSamePointTolerance) ? SM_SUCCESS : SM_ERR, _T("dOptDistanceToCreateLine must be larger than dSamePointDistance")) ;

  // this is over constrained - the system does not require this.  Perhaps we can remove this check
  // additionally, sm_FlushCrv is written to return one-curve Composites when the curve segment end-gaps are large.
  // SER_MSG((crCurves.GetSize() > 1 ? SM_SUCCESS : SM_ERR), _T("Must have more than 1 curve to build a composite")) ;

  // Get the max input tolerance.
  double dTol = //   (dOptDistanceToCreateBlend > 0.0) ? dOptDistanceToCreateBlend
                     (dOptDistanceToCreateLine  > 0.0) ? dOptDistanceToCreateLine
                // : (dOptDistanceToExtendTrim  > 0.0) ? dOptDistanceToExtendTrim
                // : (dOptDistanceToAverage     > 0.0) ? dOptDistanceToAverage
                :    (dSamePointTolerance) ;

  // init output
  rComposites.ReSet();

  // locals
  ULONG i;
  ULONG lTotalCurves = crCurves.GetSize();
  if (lTotalCurves == 0) return SM_SUCCESS;

  // create an array of link-list curve objects
  SmCurveItem * pCurveItem;
  SmCurveItem * pList = new SmCurveItem[lTotalCurves];
  SmTypedArrayDelete<SmCurveItem> sCleanList(pList) ;


  // for every curve - init a LinkList object
  for(i=0; i<lTotalCurves; i++ )
    {
      pCurveItem = &pList[i];

      pCurveItem->m_pCurve = (SmCurve*)crCurves[i];
      crCurves[i]->GetEnds( pCurveItem->m_Start, pCurveItem->m_End );

    } // end iter every curve initializing LinkList Objects

  // while loop locals
  ULONG         lEnd = 0 ; // Sort direction, 0 = Append to LinkList End boundary
                           //                 1 = Append to LinkList Start boundary
  double        dDist;
  SmCurveItem   sListHead;
  SmCurveItem * pStart        = NULL;
  SmCurveItem * pEnd          = NULL;
  SmBoolean     bInsertBefore = FALSE;
  SmBoolean     bReverse      = FALSE;
  SmBoolean     bDone = FALSE;

  // Point ListHead->Next to pList[0] and set while pointers
  AppendSeg( &sListHead, &pList[0] );
  pList[0].m_bUsed = TRUE;
  pStart           = &pList[0];
  pEnd             = &pList[0];

  // while curves are not yet placed into an ordered link list
  //   matching curve ends into a single profile
  while(!bDone )
    {
      double        dMinDist   = SM_BIG_DOUBLE;
      SmCurveItem * pFoundItem = NULL;

      // for every curve - find the best curve (smallest end-gap) to append to current linkList
      for(i=0; i<lTotalCurves; i++ )
        {
          pCurveItem = &pList[i];

          // skip marked items
          if(pCurveItem->m_bUsed ) continue;

          // when looking at the LinkList End boundary
          if(lEnd == 0 )
            {
              // when this curve's start is closer to the LinkList->End
              if (( dDist = pCurveItem->m_Start.DistanceBetween( pEnd->m_End ) ) < dMinDist )
                {
                  // remember to insert this curve after current LinkList->End  - no reversing
                  bInsertBefore = FALSE;
                  dMinDist      = dDist;
                  bReverse      = FALSE;
                  pFoundItem    = pCurveItem;
                }

              // when this curve's end is closer to the LinkList->End
              if (( dDist = pCurveItem->m_End.DistanceBetween( pEnd->m_End )) < dMinDist )
                {
                  // remember to insert this curve after current LinkList->End  - reversing
                  bInsertBefore = FALSE;
                  dMinDist      = dDist;
                  bReverse      = TRUE;
                  pFoundItem    = pCurveItem;
                }
            } // end when looking at the LinkList End boundary branch
          // else, when looking at the LinkList Start boundary
          else  // lEnd == 1
            {
              // when this curve's start is closer to the LinkList->Start
              if (( dDist = pCurveItem->m_Start.DistanceBetween( pStart->m_Start )) < dMinDist )
                {
                  // remember to insert this curve before current LinkList->Start  - reversing
                  bInsertBefore = TRUE;
                  dMinDist      = dDist;
                  bReverse      = TRUE;
                  pFoundItem    = pCurveItem;
                }
              // when this curve's end is closer to the LinkList->Start
              if (( dDist = pCurveItem->m_End.DistanceBetween( pStart->m_Start )) < dMinDist )
                {
                  // remember to insert this curve before current LinkList->End  - no reversing
                  bInsertBefore = TRUE;
                  dMinDist      = dDist;
                  bReverse      = FALSE;
                  pFoundItem    = pCurveItem;
                }
            }
        } // end iter every input curve searching for best one (smallest end-gap) to append to LinkList end

      // skip out of tolerance ends
      if(dMinDist > dTol ) pFoundItem = NULL;

      // When no curve is found to append to LinkList end boundary - try again appending from Start boundary
      if(pFoundItem == NULL && lEnd == 0 ) { lEnd = 1; continue; }

      // if no curve is found to append to LinkList start or end boundaries
      if(pFoundItem == NULL && lEnd == 1 )
        {
          // properties of curves on Linked List - NULL terminated
          //   ListHead - is not part of the composite curve, composite starts with ListHead->Next
          //   All linkedList curve segments will have a m_DistToNext value < dTol except
          //   the last curve segment will have        a m_DistToNext value > dTol (even if it's a closed composite curve)
          //   the last curve segment will have m_pNext = NULL.
          //   Curve segments are listed only once

          // Flush the curve and reinitialize the list
          SmBoolean bUnused = FALSE;
          double    dUnused = 0.0 ;
          sm_FlushCrv( crContext,                // in : context for new object construction
                          &sListHead,               // in : pListHead->Next starts composite curve, pListHead isnot part of the composite.
                                                    //      this is a NULL terminated LinkedList
                          dTol,                     // in : max gap-size between curve segment end-points
                          bUnused,                  // in : bMakeCurvesHomogeneous,
                          dUnused,                  // in : dThisApproxTol3d,
                          dSamePointTolerance,      // in : gap-size < dSamePointTolerance, connect curves as is.
                          dUnused,                  // in : dOptDistanceToAverage,
                          dUnused,                  // in : dOptDistanceToExtendTrim,
                          dOptDistanceToCreateLine, // in : gap-size < dOptDistanceToCreateLine, add line seg and connect.
                                                    //      0.0 to ignore
                          dUnused,                  // in : dOptDistanceToCreateBlend,
                          rComposites );            // out: composite curve

          // Now look for 1st remaining curve segment to start another composite curve
          for(ULONG kk=0; kk<lTotalCurves; kk++ )
            {
              SmCurveItem *pCI = &pList[kk];

              // skipped used curve segments
              if (pCI->m_bUsed) continue;

              // start next composite on first unused curve segment
              sListHead.m_pNext = pCI;
              pCI->m_bUsed     = TRUE;
              pStart           = pCI;
              pEnd             = pCI;
              break;
            } // end look for another segment

          // exit case - no new segments found to start another composite curve
          bDone =(sListHead.m_pNext == NULL );

          lEnd = 0;
          continue;
        } // end if no curve is found to append to LinkList start or end boundaries

      // arrive here when a curve segment was found to add Linked List

      // remember the segment has been used
      pFoundItem->m_bUsed = TRUE;

      // When curve segment needs to be reversed - reverse its data
      if(bReverse )
        {
          pFoundItem->m_bSense = FALSE;
          SmPoint3d tt = pFoundItem->m_Start;
          pFoundItem->m_Start = pFoundItem->m_End;
          pFoundItem->m_End = tt;
        }

      // if appended to start - insert segment after pListHead, update pStart value
      if(bInsertBefore )
        {
          pFoundItem->m_DistToNext = dMinDist;
          AppendSeg( &sListHead, pFoundItem );
          pStart = pFoundItem;
        }
      else // appended to end - insert segment after pEnd, update pEnd value
        {
          pEnd->m_DistToNext = dMinDist;
          AppendSeg( pEnd, pFoundItem );
          pEnd = pFoundItem;
        }
    } // end while not done

  // all done
  // GWC: avoid memory leaks replaced following with SmTypedArrayDelete<TYPE> style of delete
  // SM_ASSERT(pList != NULL) ; delete [] pList ; pList = NULL ;
  return SM_SUCCESS;

} // end SmCompositeCurve::BuildCompositesFromCurves

/*******************************************************************//**
PURPOSE: Destructor for the surface curve.  Parent curves are destroyed.

NOTES:
***********************************************************************/
SmCompositeCurve::~SmCompositeCurve()
{
  for (ULONG i=0; i<m_lNumSegments; i++)
    {
      SM_ASSERT(m_paSegments[i].m_pParentCurve != NULL) ;
      delete m_paSegments[i].m_pParentCurve ;
      m_paSegments[i].m_pParentCurve = NULL ;
    }
  smos_Free(m_paSegments); m_paSegments = NULL ;

} // end SmCompositeCurve::~SmCompositeCurve destructor

/*******************************************************************//**
PURPOSE: Get the canonical representation of the composite curve.

NOTES:
***********************************************************************/
SmStatus SmCompositeCurve::GetCanonical
  (ULONG               & rlDimension,  // out: 3d or 2d
   SmTArray<SmCurve*>  & rCurves,      // out: array of curves in the composite
   SmBoolean           & rbClosed,     // out: TRUE = last curve is connected to the first curve, FALSE = not
   SmTArray<SmBoolean> * pSenses)      // out: optional sense bit for each curve, NULL to ignore
  const
{
  rlDimension = GetDim();
  rbClosed    = m_bClosed;

  rCurves.ReSet();
  if(pSenses != NULL ) pSenses->ReSet();

  ULONG i;
  for(i=0; i<m_lNumSegments; i++ )
    {
      rCurves.Add( m_paSegments[i].m_pParentCurve );
      if(pSenses != NULL )
        {
          pSenses->Add( m_paSegments[i].m_bSameSense );
        }
    }

  // all done
  return SM_SUCCESS;

} // end SmCompositeCurve::GetCanonical


/*******************************************************************//**
PURPOSE: Add a curve and corresponding sense to the beginning or
    to the end of the composite curve.

NOTES:
If you have a composite curve where the direction is P0 -> P1 and the end point is P1....
then you add another segment P2 -> P3
If P1 == P2 the sense  for P2 -> P3 is TRUE;
if P1 == P3 the sense for P2 -> P3 is FALSE.
***********************************************************************/
SmStatus SmCompositeCurve::AddCurve
  (SmCurve *pCurve,                    // Note the curve
                                       // is consumed by the composite.
   SmBoolean bSense,                   // Sense of the curve
   SmBoolean bAddToBeginning,          //
                                       // TRUE - add to beginning of composite
                                       // FALSE - add to end of composite
   SmBoolean bMakesCompositeClosed     // If TRUE
                                       // this curve closes the composite into
                                       // a closed loop.
   )
{
    SmTArray<SmCurve*> sCurves;
    ULONG lDim;
    SmBoolean bClosed;
    SmTArray<SmBoolean> sSenses;
    SER( GetCanonical( lDim, sCurves, bClosed, &sSenses ));
    if(bAddToBeginning )
    {
        sCurves.InsertAt( 0, pCurve, 1 );
        sSenses.InsertAt( 0, bSense, 1 );
    }
    else
    {
        sCurves.Add( pCurve );
        sSenses.Add( bSense );
    }

    smos_Free( m_paSegments ); m_paSegments = NULL ;
    m_lNumSegments = sCurves.GetSize();
    ULONG lSize = sizeof(SmCompositeCurveSegment) * m_lNumSegments;

    // okay to use smos_Calloc on static class (SmCompositeCurveSegment) objects.
    m_paSegments = (SmCompositeCurveSegment*)smos_Calloc( lSize, 1 );
    ULONG i;
    for(i=0; i<m_lNumSegments; i++ )
    {
        SmBoolean bSameSense = sSenses[i];
        m_paSegments[i] = SmCompositeCurveSegment( sCurves[i], bSameSense );
    }

    if(bMakesCompositeClosed )
    {
        m_bClosed = TRUE;
    }

    return SM_SUCCESS;

} // end SmCompositeCurve::AddCurve

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
static SmStatus sm_CreateCorner
  (const SmContext   & crContext,       // in : context for new object construction
   const SmPoint3d   & crVertexPoint,   // in : Vertex point of origin curve ends,
                                        //        only used for SM_OC_FILLET_CORNER as center of new fillet corner.
   const SmPoint3d   & crP1,            // in : position at end of offset segment ending at the corner
   const SmVector3d  & crV1,            // in : tangent  at end of offset segment ending at the corner
   const SmPoint3d   & crP2,            // in : position at beg of offset segment beginning at the corner
   const SmVector3d  & crV2,            // in : tangent  at beg of offset segment beginning at the corner
   SmOffsetCornerType  eOffsetCorner,   // in : SM_OC_LINEAR_EXTENSION: corner = 2 lines from given ends to common linear extension xsect point.
                                        //      SM_OC_FILLET_CORNER: corner = fillet centered on crVertexPoint running to given end points
                                        //      SM_OC_LINEAR_CHAMFER: corner = line between endpoints crP1 and crP2 (result is within offset distance)
   const double dOffsetDistance,        // in : only used for SM_OC_LINEAR_CHAMFER
   SmXSectTol3d        dXSectTol3d,     // in : XSect tolerance used in SM_OC_LINEAR_CHAMFER. NULL to ignore
   SmCurve          *& rpCornerCurve1,  // out: SM_OC_LINEAR_EXTENSION: line from crP1 to xsect point,
                                        //                              or NULL when Len(crV1) == 0.0
                                        //      SM_OC_FILLET_CORNER:    circle fillet
                                        //      SM_OC_LINEAR_CHAMFER:   line segment for extension to curve 1
   SmCurve          *& rpCornerCurve2,  // out: SM_OC_LINEAR_EXTENSION: line from xsect point to crP2,
                                        //                              or NULL when Len(crV1) == 0.0
                                        //      SM_OC_FILLET_CORNER:    NULL
                                        //      SM_OC_LINEAR_CHAMFER:   line segment for extension to curve 2
   SmCurve          *& rpCornerCurve3)  // out: SM_OC_LINEAR_EXTENSION: NULL
                                        //      SM_OC_FILLET_CORNER:    NULL
                                        //      SM_OC_LINEAR_CHAMFER:   line segment for chamfer
{
  // init output
  rpCornerCurve1 = NULL;
  rpCornerCurve2 = NULL;
  rpCornerCurve3 = NULL;

  // case SM_OC_LINEAR_EXTENSION - build lines from given end points to xsect point between linear extensions
  //                             - there will be a tolerance sized gap between the line segments when
  //                               the linear offsets don't intersect exactly
  if(eOffsetCorner == SM_OC_LINEAR_EXTENSION )
    {
      // find linear extension xsect point - handle zero length tangents
      double dT1 = 0.0;
      double dT2 = 0.0;
      if    (  crV1.LengthSquared() > SM_EFF_ZERO_SQ
              && crV2.LengthSquared() > SM_EFF_ZERO_SQ) { SER( smgu_LineLineClosestPoint( crP1, crV1, crP2, crV2, dT1, dT2 )); }
      else if(   crV1.LengthSquared() > SM_EFF_ZERO_SQ) { SER( smgu_LineClosestPoint( crP1, crV1, crP2, dT1 )); }
      else                                              { SER( smgu_LineClosestPoint( crP2, crV2, crP1, dT2 )); }

      // when crV1 is zero length - no linear extension
      if(SM_IS_ZERO(dT1) )
        {
          rpCornerCurve1 = NULL;
        }
      else // add a line from crP1 to the xsect point
        {
          SmBSplineCurve *pTempBSPlCurve;
          SmPoint3d sEnd1 = crP1 + dT1 * crV1;
          SER( SmBSplineCurve::CreateLineSegment( crContext, 3, crP1, sEnd1, pTempBSPlCurve ));
          rpCornerCurve1 = pTempBSPlCurve;
        }

      // when crV2 is zero length - no linear extension
      if(SM_IS_ZERO(dT2) )
        {
          rpCornerCurve2 = NULL;
        }
      else // add a line from the xsect point to crP2
        {
          SmBSplineCurve *pTempBSPlCurve;
          SmPoint3d sEnd2 = crP2 + dT2 * crV2;
          SER( SmBSplineCurve::CreateLineSegment( crContext, 3, sEnd2, crP2, pTempBSPlCurve ));

          rpCornerCurve2 = pTempBSPlCurve;
        }
    } // end if SM_OC_LINEAR_EXTENSION

  // case SM_OC_FILLET_CORNER - build fillet arc in the corner running from crP1 to crP2 centered on crVertexPoint.
  //                          - because this is for a single offset operation we expect:
  //                             o. Length(crP1 - crVertexPoint) == Length(crP2 - crVertexPoint)
  //                             o. fillet arc to be tangent to both given point tangents
  else if(eOffsetCorner == SM_OC_FILLET_CORNER )
    {
      // Create a fillet in the corner.
      SmVector3d sV1 = crP1 - crVertexPoint;
      SmVector3d sV2 = crP2 - crVertexPoint;
      SmVector3d sX, sY, sZ;
      SmVector3d sCross = sV1 * sV2;

      // Handle the 180 degree case by using crV1 as the direction on the
      // side of the curve to keep.
      if(sCross.LengthSquared() < SM_EFF_ZERO )
        {
          SER( sV1.MakeUnitOrthoVectors( &crV1, sX, sY, sZ ));
        }
      else
        {
          SER( sV1.MakeUnitOrthoVectors (&sV2, sX, sY, sZ ));
        }
      double dAngle;
      SER( sZ.CCWAngleBetween( sV1, sV2, dAngle ));
      dAngle = smos_Fabs(dAngle);
      SmAxis2Placement sA2P;
      sA2P.SetCanonical( crVertexPoint, sX, sY );
      SmBSplineCurve *pTempBSPlCurve;
      SER( SmBSplineCurve::CreateCircleSegment( crContext,            // in : context for new object construction
                                                3,                    // in : image space dimension: 2 or 3
                                                sA2P,                 // in : circle centered on orig lies on xy plane with zero degrees defined by x
                                                sV1.Length(),         // in : radius
                                                0,                    // in : start angle, degrees
                                                dAngle*180.0/SM_PI,   // in : end angle, degrees
                                                SM_CO_QUADRATIC,      // in : circle is degree 2 rational
                                                pTempBSPlCurve ));    // out: the new circle
      rpCornerCurve1 = pTempBSPlCurve;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe)
        {
          sm_GraphicsLoop();
          smgfx_SetColor(1,0,0);
          rpCornerCurve1->DrawWDeriv(rpCornerCurve1->GetNaturalInterval(),0);
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

  } // end else if SM_OC_FILLET_CORNER

  // case SM_OC_LINEAR_CHAMFER - extend offsets to their potential intersection
  // build offset chamfer offsetDistance away from vertex and perpendicular to the line
  // that goes from the vertex to the potential intersection
  else if(eOffsetCorner == SM_OC_LINEAR_CHAMFER )
  {
      // find linear extension xsect point - handle zero length tangents
      double dT1 = 0.0;
      double dT2 = 0.0;
      if( crV1.LengthSquared() > SM_EFF_ZERO_SQ && crV2.LengthSquared() > SM_EFF_ZERO_SQ) {
          SER( smgu_LineLineClosestPoint( crP1, crV1, crP2, crV2, dT1, dT2 ));
      }
      else if( crV1.LengthSquared() > SM_EFF_ZERO_SQ) {
          SER( smgu_LineClosestPoint( crP1, crV1, crP2, dT1 ));
      }
      else {
          SER( smgu_LineClosestPoint( crP2, crV2, crP1, dT2 ));
      }

      // Keep track of intersection between offset extensions
      SmPoint3d xPnt;

      // when crV1 is zero length - no linear extension
      if(SM_IS_ZERO(dT1) ) {
          rpCornerCurve1 = NULL;
      }
      else {
          SmBSplineCurve *pTempBSPlCurve;
          SmPoint3d sEnd1 = crP1 + dT1 * crV1;
          SER( SmBSplineCurve::CreateLineSegment( crContext, 3, crP1, sEnd1, pTempBSPlCurve ));
          rpCornerCurve1 = pTempBSPlCurve;

          xPnt = sEnd1;
      }

      // when crV2 is zero length - no linear extension
      if(SM_IS_ZERO(dT2) ) {
          rpCornerCurve2 = NULL;
      }
      else {
          SmBSplineCurve *pTempBSPlCurve;
          SmPoint3d sEnd2 = crP2 + dT2 * crV2;
          SER( SmBSplineCurve::CreateLineSegment( crContext, 3, crP2, sEnd2, pTempBSPlCurve ));

          rpCornerCurve2 = pTempBSPlCurve;

          xPnt = sEnd2;
      }

      // Both offset extensions need to exist to calculate chamfer
      if( rpCornerCurve1 == NULL || rpCornerCurve2 == NULL )
          return SM_SUCCESS;

      // create chamfer

      // create vector between vertex and extension intersection
      SmVector3d sVertexPtToXPt = xPnt - crVertexPoint;
      sVertexPtToXPt.Unitize();

      // Take cross product with plane normal to get vector of chamfer
      SmTArray<SmCurve*> sOffsets;
      sOffsets.Add( rpCornerCurve1 );
      sOffsets.Add( rpCornerCurve2 );

      SmVector3d sPlaneNormal(0,0,1);
      SmCurve::ComputeCrvsNormal( sOffsets, sPlaneNormal );

      SmVector3d sChamferVector = sVertexPtToXPt * sPlaneNormal;
      SmPoint3d sChamferPt = crVertexPoint + dOffsetDistance * sVertexPtToXPt;

      // Now add and subtract along chamferVector
      // This should always extend past the extended offsets
      SmBSplineCurve *pTempBSPlCurve;
      SmPoint3d sChamferStartPt = sChamferPt + dOffsetDistance * sChamferVector;
      SmPoint3d sChamferEndPt = sChamferPt - dOffsetDistance * sChamferVector;
      SER( SmBSplineCurve::CreateLineSegment( crContext, 3, sChamferStartPt, sChamferEndPt, pTempBSPlCurve ));

      rpCornerCurve3 = pTempBSPlCurve;

      // Now let's trim off unnecessary pieces because IntersectAndTrimOffsets doesn't do it
      SmExtent1d sRange1 = rpCornerCurve1->GetNaturalInterval();
      SmExtent1d sRange2 = rpCornerCurve2->GetNaturalInterval();
      SmExtent1d sRange3 = rpCornerCurve3->GetNaturalInterval();

      // Intersect the first offset with the chamfer
      SmSolutionArray sSolutions1;
      rpCornerCurve1->GlobalCurveIntersect( sRange1, *rpCornerCurve3,  sRange3, dXSectTol3d, sSolutions1 );

      // Trim the first offset - we assume there is one solution
      SmExtent1d sTrimIvl( sRange1.GetMin(), sSolutions1[0].m_vStart[0] );
      SER( rpCornerCurve1->Trim( sTrimIvl )) ; // may snap sIvl by tol to existing knots

      // Intersect the second offset with the chamfer
      SmSolutionArray sSolutions2;
      rpCornerCurve2->GlobalCurveIntersect( sRange2, *rpCornerCurve3,  sRange3, dXSectTol3d, sSolutions2 );

      // Trim the second offset - we assume there is one solution
      SmExtent1d sTrimIv2( sRange2.GetMin(), sSolutions2[0].m_vStart[0] );
      SER( rpCornerCurve2->Trim( sTrimIv2 )) ; // may snap sIvl by tol to existing knots

      // Trim the chamfer
      SmExtent1d sTrimIv3;
      double dP1 = sSolutions1[0].m_vStart[1], dP2 = sSolutions2[0].m_vStart[1];
      if ( dP1 < dP2 )
          sTrimIv3.SetMinMax( dP1, dP2 );
      else
          sTrimIv3.SetMinMax( dP2, dP1 );
      SER( rpCornerCurve3->Trim( sTrimIv3 )) ; // may snap sIvl by tol to existing knots


#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe)
        {
          sm_GraphicsLoop();
          smgfx_SetColor(1,0,0);
          rpCornerCurve1->DrawWDeriv(rpCornerCurve1->GetNaturalInterval(),0);
          sm_GraphicsLoop();
          if( rpCornerCurve2 ) {
              rpCornerCurve2->DrawWDeriv(rpCornerCurve2->GetNaturalInterval(),0);
            sm_GraphicsLoop();
          }
          if( rpCornerCurve3 ) {
            rpCornerCurve3->DrawWDeriv(rpCornerCurve3->GetNaturalInterval(),0);
            sm_GraphicsLoop();
          }
        }
#endif // SM_DEBUG_CODE
    } // end else if SM_OC_LINEAR_CHAMFER
  else
    {
      SER( SM_ERR );  // No other corner cases implemented yet
    }

  // all done
  return SM_SUCCESS;

} // end sm_CreateCorner

/*******************************************************************//**
PURPOSE: Given one of the trimmed offset curves, decide whether or not
   it should be kept or discarded.

NOTES:
***********************************************************************/
static SmStatus sm_CurveClassify
( const SmCurve                  & crTrimmedOffset, // in : Trimmed Offset Curve to classify
  double                           dOffsetDistance, // in : size of the offset
  double                           dXSectTol3d,     // in : passed to GlobalPointSolve() as the dDistanceTolerance, GWC:should be XSectTol3d
  const SmTArray<const SmCurve*> & crOrigCurves,    // in: originals that were offset
  SmBoolean                      & rbKeepCurve )    // out: TRUE = keep curve being checked
                                                    //      FALSE= discard curve being checked
{
    // Init output
    rbKeepCurve = FALSE;

    // No longer necessary.  [bd, 11 March 09]
//  // Get some tolerances.  Note: this is the same as in sm_CheckBowTies(),
//  // so if you change this, check that too.
//  double dFudgeFactor = 3.0;
//  if (dApproxTol > 0.01 * dOffsetDistance) {
//      dFudgeFactor = 1.1;
//  }
//  if (dApproxTol > 0.1 * dOffsetDistance) {
//      dFudgeFactor = 1.02;
//  }
//  double dFudgeTol = dApproxTol * dFudgeFactor;

    // locals
    SmExtent1d sIvl = crTrimmedOffset.GetNaturalInterval();
    SmAttribute *pAttr = crTrimmedOffset.FindAttribute( SM_AI_OFFSETMAP );
    SmOffsetMapAttribute *pMapAttr = SM_CAST_PTR( SmOffsetMapAttribute, pAttr );
    const SmCurve        * pOriginator = ( pMapAttr != NULL ) ? pMapAttr->GetOrigCurve() : NULL;

    // Quick Check mid point for being in a bow-tie.  It's a cheap test.
    // In that case, the originator point should be at almost
    // exactly the same parameter value on both curves.  Since
    // the radius of curvature is smaller than the offset distance,
    // the curves will be moving in opposite directions.
    if ( pOriginator != NULL )
    {
        double dMidParam = sIvl.Evaluate( 0.5 );
        SmVector3d sPVThis[2], sPVOrig[2];
        crTrimmedOffset.Evaluate( dMidParam, 1, FALSE, sPVThis );
        pOriginator->Evaluate( dMidParam, 1, FALSE, sPVOrig );
        if ( sPVThis[1].Dot( sPVOrig[1] ) < 0.0 )
        {
            rbKeepCurve = FALSE;
            return SM_SUCCESS;
        }
    } // end if bow-tie check.


    // Method: Offset curve pieces to be discarded are those that are closer
    // than the offset distance to any of the original curves.  The offsets
    // have already been trimmed, so each piece is either entirely ok, or
    // entirely in violation.
    //
    // We check this by using GlobalPointSolve to drop sample points to each
    // original curve, and then check whether the closest point is closer than
    // the offset distance.
    //
    // We have to correlate the tolerance used for the solver with that used
    // to compare the drop-point distance to the offset distance.  We want
    // the tolerance for comparison to be as small as possible, because
    // near-tangent curves (which are not uncommon) result in very small
    // trimmed pieces that are extremely close to the actual offset distance.
    //
    // Fortunately, any error in the GlobalPointSolve will result in a point
    // that is farther away than the actual closest approach, because it's a
    // minimization.  So we don't really have to correlate the tolerances,
    // just figure a separate tolerance to use for the comparison.
    //
    // This means that this is one of those rare instances where a
    // lower-level routine (this function) should make up its own tolerance,
    // without guidance from the caller.

    // Use something that is plenty big enough to cover numerical noise.
    double dCutoffDist = dOffsetDistance * ( 1.0 - SM_EFF_ZERO );

    // For GlobalPointSolve:
    double dMinDist = dOffsetDistance + 2.0 * dXSectTol3d;
    SmSolution aData[10];
    SmSolutionArray sSolutions( 10, aData );
    rbKeepCurve = TRUE;

    // Loop on all orig curves:
    // drop points from 0.1, 0.5, and 0.9 on crTrimmedOffset to each orig curve in loop.
    SmTArray< SmPoint3d > aSamplePts;
    SmPoint3d sPnt;
    // Note: per the analysis above, we don't have to do
    // more than one (expensive) GlobalPointSolve.
//  SER( crTrimmedOffset.EvaluatePoint( sIvl.Evaluate(0.1), sPnt ));
//  aSamplePts.Add( sPnt );
    SER( crTrimmedOffset.EvaluatePoint( sIvl.Evaluate( 0.5 ), sPnt ) );
    aSamplePts.Add( sPnt );
    //  SER( crTrimmedOffset.EvaluatePoint( sIvl.Evaluate(0.9), sPnt ));
    //  aSamplePts.Add( sPnt );

    ULONG i, j;
    for ( i = 0; i < crOrigCurves.GetSize(); i++ )
    {
        const SmCurve *pOrig = crOrigCurves[i];

        for(j = 0; j < aSamplePts.GetSize(); j++ )
        {
            sPnt = aSamplePts[j];
            SER( pOrig->GlobalPointSolve
                   (pOrig->GetNaturalInterval(), // in : search curve interval
                    SM_SO_MINIMIZE,              // in : which solver operation to perform
                    sPnt,                        // in : Euclidean target point
                    dXSectTol3d,                 // in : Basically the distance tolerance sets up a range for the
                                                 //      distance measurements where additional answers may exist.
                                                 //      For example if the distance between two local minima/maxima
                                                 //      is less than this tolerance, both answers will be returned.
                    &dMinDist,                   // in : If not NULL it will be:
                                                 //        SM_SO_AT_DISTANCE       = target distance,
                                                 //        SM_SO_MINIMIZE/MAXIMIZE = distance limit.
                                                 //      For example, to find a minimum value only if it less than
                                                 //      the target distance or maximum value only if it is greater than
                                                 //      the target distance
                    NULL,                        // in : Vectors used in some of the solvers.
                                                 //      SM_SO_RAYFIRE,                  SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
                                                 //      SM_SO_SIGNED_DIRECTED_MINIMIZE, SM_SO_DIRECTED_MINIMIZE
                                                 //      SM_SO_DIRECTED_MAXIMIZE,        SM_SO_PROJECTED_MINIMIZE
                                                 //      SM_SO_PROJECTED_MAXIMIZE
                                                 //      SM_SO_PROJECTED_TANGENT_THROUGH_POINT
                    SM_SR_SINGLE,                // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions
                    sSolutions)) ;               // out: array of problem solutions reported as curve parameter values

            if(sSolutions.GetSize() > 0 )
            {
                double dThisDist = sSolutions[0].m_vStart.m_dSolutionValue;
                if(dThisDist < dCutoffDist )
                {
                    // Within offset distance of this curve.
                    rbKeepCurve = FALSE;
                    return SM_SUCCESS;
                }
            }
        } // end loop on sample points of this curve
    } // end loop on all orig curves

    // all done
    return SM_SUCCESS;

} // end sm_CurveClassify

/*******************************************************************//**
PURPOSE:     Insert a double value into a sorted double array as
    the first entry - removing any values in the array less than the given value

NOTES: Simple linear search to insert proper sort
***********************************************************************/
static SmStatus sm_TrimFirstBreak
 (SmTArray<double> & rDoubleArray,     // i/o: increasing value sorted array
  double             dNewDouble)       // in : target value to insert
{
  ULONG ii ;

  // search for array member larger than dNewDouble value
  for(ii=0; ii<rDoubleArray.GetSize(); ii++ )
    {
      // replace repeated values
      if(SM_ARE_SAME( dNewDouble, rDoubleArray[ii] ) )
        {
          rDoubleArray.SetAt(ii, dNewDouble) ;
          if(ii > 0) { rDoubleArray.RemoveAt(0, ii) ; }
          return SM_SUCCESS;
        }

      // insert prior to first value larger than NewDouble
      if(dNewDouble < rDoubleArray[ii] )
        {
          // insert the value at that location
          rDoubleArray.InsertAt( ii, dNewDouble, 1 );
          if(ii > 0) { rDoubleArray.RemoveAt(0, ii) ; }
          return SM_SUCCESS;
        }
    }

  // arrive here when dNewDouble is larger than all entries - replace array
  rDoubleArray.ReSet() ;
  rDoubleArray.Add( dNewDouble );

  // all done
  return SM_SUCCESS;

} // end sm_TrimFirstBreak

/*******************************************************************//**
PURPOSE:     Insert a double value into a sorted double array as
    the last entry - removing any values in the array greater than the given value

NOTES: Simple linear search to insert proper sort
***********************************************************************/
static SmStatus sm_TrimLastBreak
 (SmTArray<double> & rDoubleArray,     // i/o: increasing value sorted array
  double             dNewDouble)       // in : target value to insert
{
  ULONG ii ;

  // search for array member larger than dNewDouble value
  for(ii=0; ii<rDoubleArray.GetSize(); ii++ )
    {
      // replace repeated values
      if(SM_ARE_SAME( dNewDouble, rDoubleArray[ii] ) )
        {
          rDoubleArray.SetAt(ii, dNewDouble) ;
          if(ii + 1 < rDoubleArray.GetSize()) { rDoubleArray.RemoveAt(ii+1, rDoubleArray.GetSize() - ii - 1) ; }
          return SM_SUCCESS;
        }

      // insert prior to first value larger than NewDouble
      if(dNewDouble < rDoubleArray[ii] )
        {
          // insert the value at that location
          rDoubleArray.InsertAt( ii, dNewDouble, 1 );
          if(ii + 1 < rDoubleArray.GetSize()) { rDoubleArray.RemoveAt(ii+1, rDoubleArray.GetSize() - ii - 1) ; }
          return SM_SUCCESS;
        }
    }

  // arrive here when dNewDouble is larger than all entries - add to end of array
  rDoubleArray.Add( dNewDouble );

  // all done
  return SM_SUCCESS;

} // end sm_TrimLastBreak

/*******************************************************************//**
PURPOSE:     Insert a double value into a sorted double array

NOTES: Simple linear search to insert proper sort
***********************************************************************/
static SmStatus sm_InsertSortedBreaks
 (SmTArray<double> & rDoubleArray,     // i/o: increasing value sorted array
  double             dNewDouble)       // in : target value to insert
{
  ULONG ii ;

  // search for array member larger than dNewDouble value
  for(ii=0; ii<rDoubleArray.GetSize(); ii++ )
    {
      // skip repeated values
      if(SM_ARE_SAME( dNewDouble, rDoubleArray[ii] ) )
        { return SM_SUCCESS; }

      if(dNewDouble < rDoubleArray[ii] )
        {
          // insert the value at that location
          rDoubleArray.InsertAt( ii, dNewDouble, 1 );
          return SM_SUCCESS;
        }
    }

  // arrive here when dNewDouble is larger than all entries - add to end of array
  rDoubleArray.Add( dNewDouble );

  // all done
  return SM_SUCCESS;

} // end sm_InsertSortedBreaks


/*******************************************************************//**
PURPOSE:
  Check for 'incomplete bow-ties' at the ends of a composite curve to be offset.

NOTES:
  This checks only the first and last curve in crOffsets.

METHOD ---
  When an offset curve doubles back on itself (due to the base curve's
  radius of curvature becoming smaller than the offset distance), it will:
     double back on itself in a cusp,
     then move retrograde while its curvature is too tight,
     then double back in another cusp,
     and finally, cross itself at a point before the first cusp.

  This is a 'bow-tie'.  Bow-ties are removed by recording the intersection point
  and trimming the offset curve there.  The parts of the offset curve between
  the intersections (containing the cusps) will all be closer to the base
  curve than the offset distance.  These get classified as such and marked as
  not to be kept.

  However, if the curve ends before the self-intersection, then the
  offset will not be trimmed there. The curve will continue into the
  "forbidden zone."
    (forbidden zone = closer to any of the original base curves than the offset distance)
  Later when the curve is classified, this forbidden
  zone violation will be found and the entire curve will be rejected.
  We only want the section in the forbidden zone rejected.

  The reliable way to test for this condition is:
    if the start or end point of the offset are within the forbidden zone.
  That's what sm_CheckBowTies() does --
    checks for endpoints within the forbidden zone
    and puts any such violating points in sPointsToTest.

  It's best to do this before trimming all of the curves at the sorted
  breaks -- we just have to find and add new breaks.  sm_FixBowTie()
  does that.
  [081028]
***********************************************************************/
static SmStatus sm_CheckBowTies
 (const  SmTArray<SmCurve*>       & crOffsets,       // in : raw offsets of the Original Curves
  double                            dOffsetDistance, // in : Offset Distance
  double                            dXSectTol3d,     // in : max distance between coincident points
  const  SmTArray<const SmCurve*> & crOrigCurves,    // in : to classify against
  SmTArray<SmPoint3d>             & sPointsToTest)   // out: open CompositeCurve EndPoints
                                                     //       found to be in the 'forbidden zone'
                                                     //      (closer than dOffsetDistance to some crOrigCurves.)
{
  sPointsToTest.ReSet();

  ULONG lNumOrig = crOrigCurves.GetSize();

  // Method: if the base composite curve is not closed, then
  //         check its start and end points.
  //         If either is in the' forbidden zone'
  //           -- closer to any of the original base curves than the offset distance --
  //         then an intersection of the offset curves will be missed,
  //         so we have to process it specially.

  // We need to check only curves that are open:
  // If the original curve(s) are contiguous (closed),
  // then the main algorithm will handle it.

  // composite curve start and end points
  SmPoint3d sStartPt, sEndPt;
  const SmCurve *pStartCurve = crOrigCurves[0];
  const SmCurve *pEndCurve   = crOrigCurves[lNumOrig-1];
  SmExtent1d     sStartIvl   = pStartCurve[0].GetNaturalInterval();
  SmExtent1d     sEndIvl     = pEndCurve[0].GetNaturalInterval();
  pStartCurve->EvaluatePoint( sStartIvl.GetMin(), sStartPt );
  pEndCurve  ->EvaluatePoint( sEndIvl  .GetMax(), sEndPt   );

  // no work - composite curve is closed
  if(sStartPt.DistanceBetween( sEndPt ) < dXSectTol3d )
    { return SM_SUCCESS; }

  // composite curve is open.

  // No longer necessary.  [bd, 11 March 09]
//  // Get some tolerances.  Note: this is the same as in sm_CurveClassify(),
//  // so if you change this, check that too.
//  double dFudgeFactor = 3.0;
//  if (dXSectTol3d > 0.01 * dOffsetDistance) {
//      dFudgeFactor = 1.1;
//  }
//  if (dXSectTol3d > 0.1 * dOffsetDistance) {
//      dFudgeFactor = 1.02;
//  }
//
//  double dFudgeTol = dXSectTol3d * dFudgeFactor;
  //SmBoolean bFixStart = FALSE, bFixEnd = FALSE;

  // locals
  SmSolutionArray sSolutions;
  ULONG ii;

  // Tolerance for checking too close:
  // see discussion in sm_CurveClassify().
  double dCutoffDistance = dOffsetDistance *(1.0 - SM_EFF_ZERO );

  // Check start point.
  SmPoint3d sStartOffsetPt;
  crOffsets[0]->EvaluatePoint( crOffsets[0]->GetNaturalInterval().GetMin(), sStartOffsetPt );

  // for every offset curve - check for curves closer than dCutoffDistance to end point
  for(ii=0;ii<crOrigCurves.GetSize();ii++ )
    {
      const SmCurve *pOrig = crOrigCurves[ii];

      // find closest distance to curve
      SER( pOrig->GlobalPointSolve
            (pOrig->GetNaturalInterval(),  // in : search curve interval
             SM_SO_MINIMIZE,               // in : which solver operation to perform
             sStartOffsetPt,               // in : Euclidean target point
             dXSectTol3d,                  // in : Basically the distance tolerance sets up a range for the
                                           //      distance measurements where additional answers may exist.
                                           //      For example if the distance between two local minima/maxima
                                           //      is less than this tolerance, both answers will be returned.
             NULL,                         // in : If not NULL it will be:
                                           //        SM_SO_AT_DISTANCE       = target distance,
                                           //        SM_SO_MINIMIZE/MAXIMIZE = distance limit.
                                           //      For example, to find a minimum value only if it less than
                                           //      the target distance or maximum value only if it is greater than
                                           //      the target distance
             NULL,                         // in : Vectors used in some of the solvers.
                                           //      SM_SO_RAYFIRE,                  SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
                                           //      SM_SO_SIGNED_DIRECTED_MINIMIZE, SM_SO_DIRECTED_MINIMIZE
                                           //      SM_SO_DIRECTED_MAXIMIZE,        SM_SO_PROJECTED_MINIMIZE
                                           //      SM_SO_PROJECTED_MAXIMIZE
                                           //      SM_SO_PROJECTED_TANGENT_THROUGH_POINT
             SM_SR_SINGLE,                 // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions
             sSolutions)) ;                // out: array of problem solutions reported as curve parameter values

      // when there are solutions
      if(sSolutions.GetSize() > 0 )
        {
          // that place the start point in the forbidden zone
          double dThisDist = sSolutions[0].m_vStart.m_dSolutionValue;
          if(dThisDist < dCutoffDistance )
            {
              // add that to the list of curve end points to be tested later on.
              sPointsToTest.Add( sStartPt );
              break;
            }
        }
    } // end iter on all orig curves for start offset

  // Now check end point.
  SmPoint3d sEndOffsetPt;
  crOffsets[lNumOrig-1]->EvaluatePoint( crOffsets[lNumOrig-1]->GetNaturalInterval().GetMax(), sEndOffsetPt );

  // for every origCurve - check for curves closer than dCutoffDistance to end point
  for(ii=0; ii<crOrigCurves.GetSize(); ii++ )
    {
      const SmCurve *pOrig = crOrigCurves[ii];

      // find closest distance to curve
      SER( pOrig->GlobalPointSolve
            (pOrig->GetNaturalInterval(), // in : search curve interval 
             SM_SO_MINIMIZE,              // in : which solver operation to perform
             sEndOffsetPt,                // in : Euclidean target point
             dXSectTol3d,                 // in : Basically the distance tolerance sets up a range for the
                                          //      distance measurements where additional answers may exist.
                                          //      For example if the distance between two local minima/maxima
                                          //      is less than this tolerance, both answers will be returned.
             NULL,                        // in : If not NULL it will be:
                                          //        SM_SO_AT_DISTANCE       = target distance,
                                          //        SM_SO_MINIMIZE/MAXIMIZE = distance limit.
                                          //      For example, to find a minimum value only if it less than
                                          //      the target distance or maximum value only if it is greater than
                                          //      the target distance
             NULL,                        // in : Vectors used in some of the solvers.
                                          //      SM_SO_RAYFIRE,                  SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
                                          //      SM_SO_SIGNED_DIRECTED_MINIMIZE, SM_SO_DIRECTED_MINIMIZE
                                          //      SM_SO_DIRECTED_MAXIMIZE,        SM_SO_PROJECTED_MINIMIZE
                                          //      SM_SO_PROJECTED_MAXIMIZE
                                          //      SM_SO_PROJECTED_TANGENT_THROUGH_POINT
             SM_SR_SINGLE,                // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions
             sSolutions)) ;               // out: array of problem solutions reported as curve parameter values

      // when there are solutions
      if(sSolutions.GetSize() > 0 )
        {
          // that place the end point in the forbidden zone
          double dThisDist = sSolutions[0].m_vStart.m_dSolutionValue;
          if(dThisDist < dCutoffDistance )
            {
              // add that to the list of curve end points to be tested later on.
              sPointsToTest.Add( sEndPt );
              break;
            }
        }
    } // end loop on all orig curves for end offset

  // all done
  return SM_SUCCESS;

} // end sm_CheckBowTies

/*******************************************************************//**
PURPOSE: Find where a curve comes within the offset distance
            of a given point.  Add parameter values into sBreaks.

NOTES:
   The way to fix this is to trim the offset curves at the point where
   they enter the 'forbidden zone'.  That will be where the curve
   approaches within the offset distance of the corresponding end point
   of the base curve.  Solve for where the curve crosses the offset
   distance of that end point, and add a break there.
   The higher algorithm will then classify each piece.
***********************************************************************/
SmStatus sm_FixBowTie
 (const  SmCurve               * pOffCrv,        // in : curve to test
  const  SmTArray< SmPoint3d > & sPointsToTest,  // in : list of Open CompositeCurve EndPoints known to be
                                                 //       closer to one of the raw offset curves (in the forbidden zone)
  double                         dOffsetDist,    // in : find point/curve distance
  double                         d3DApproxTol,   // NotUsed: in : not used
  SmTArray<double>             & sBreaks )       // in/out: growing list appended to with
                                                 //         all locations on pOffCrv a dOffsetDist
                                                 //         from any of the sPointsToTest.
{
  SM_REF1(d3DApproxTol) ;
  ULONG lNumPts = sPointsToTest.GetSize();

  // no work - no test points
  if(lNumPts < 1 )
    { return SM_SUCCESS; }

  // locals
  ULONG ii, jj;
  SmExtent1d sIvl = pOffCrv->GetNaturalInterval();
  SmSolutionArray sSolutions;

  // for every test point - find curve locations a fixed distance to the point
  for(ii=0;ii<lNumPts;ii++)
    {
      // find locations on the curve a fixed distance to the point
      SmPoint3d PtToTest = sPointsToTest[ii];
      pOffCrv->GlobalPropertyAnalysis( sIvl,
                                       SM_CP_DISTANCE_TO_POINT,
                                      &dOffsetDist,
                                      &PtToTest,
                                       SM_EFF_ZERO,
                                       sSolutions );

      // for every solution
      for(jj = 0; jj < sSolutions.GetSize(); jj++ )
        {
          SmSolution & rSol = sSolutions[jj];
          double       dT   = rSol.m_vStart[0];
          SER( sm_InsertSortedBreaks( sBreaks, dT ));
        }
    }

  // all done
  return SM_SUCCESS;

} // end sm_FixBowTie

/*******************************************************************//**
PURPOSE: Simultaneous offset of a set of composite curves.  The offset
    may contain loops of different orientation (i.e. outer loop with holes
    or adjacent outer loops or any combination).  The offsets will be done
    simultaneously which means that if the offset curves of one come too
    close to another composite curve they will be removed.

    Note that the resulting curves will have attributes (SmOffsetMapAttribute)
    which describes where it came from and how it was generated.

NOTES: The original composite curves should not intersect each
    other and should not be self-intersecting.  The same requirements
    apply as for CreateTrimmedOffsets.
***********************************************************************/
SmStatus SmCompositeCurve::CreateOffsetsOfManyCurves
  (const SmContext                   & crContext,            // in : context for new object construction
   const SmTArray<SmCompositeCurve*> & crCurvesToOffset,     // in : Array of composite curves to offset
   const SmTArray<ULONG>             & crOffsetDirections,   // in : Corresponding direction of each composite
                                                             //      curve to offset.  1-LEFT, 2-RIGHT, 3-BOTH
   double                              dXSectTol3d,          // in : Minimum distance at which offset curve end-gaps are filled with corner curves
   double                              dApproxTol3d,         // in : Tolerance to which BSpline Approximations to exact offset curves are built
   const SmVector3d                  & crOffsetPlaneNormal,  // in : Defines, along with the curve's parameter direction,
                                                             //      the right and left hand offset directions.
   SmOffsetCornerType                  eOffsetCornerType,    // in : SM_OC_LINEAR_EXTENSION: corner = 2 lines from given ends to common linear extension xsect point.
                                                             //      SM_OC_FILLET_CORNER: corner = fillet arc centered on crVertexPoint running to given end points
                                                             //      SM_OC_LINEAR_CHAMFER: corner = line between endpoints (result is actually within offset distance so bTrimResults must = false).
   SmBoolean                           bTrimResults,         // in : TRUE = concave raw offsets are intersected and trimmed back to common intersection points
                                                             //      FALSE= skip trim step
   double                              dOffsetDistance,      // in : offset distance (a negative value negates the offset direction)
   SmTArray<SmBSplineCurve*>         & rTrimmedOffsets,      // out: Resulting offset curves are SmBSplineCurves and will have 
                                                             //      attribute attached describing origination of curve
   SmBoolean                      bOptMatchParameterization, // in : TRUE = if BSplineCurve just copy it (preserves CrvParams)
                                                             //      FALSE= approximate BSplineCurves (changes CrvParams), NonBSplineCrvs always Approximated
                                                             //      default:[FALSE]
   SmTArray<double>                  * pOptMaxGaps3d)        // out: optional MaxGap for each output approximation
{
  // init output
  rTrimmedOffsets.ReSet();
  if(pOptMaxGaps3d) { pOptMaxGaps3d->ReSet() ; }

  // Offset distance must be positive.
  SmBoolean bFlip = FALSE;
  if(dOffsetDistance < 0.0)
    {
      dOffsetDistance = - dOffsetDistance;
      bFlip = TRUE;
    }

  // The algorithm used here just offsets each curve.
  // Then it generates extensions and intersects and classifies everything.

  // locals
  ULONG ii, jj;
  SmTArray<const SmCurve*> sOrigCurves;
  SmTArray<const SmCurve*> sTotalOrigCurves;
  SmTArray<SmCurve*>       sOffsets;
  SmTArray<SmBoolean>      sClosedCurves;
  SmObjsDelete<const SmCurve*> sCleanUp1(&sOrigCurves);
  SmObjsDelete<const SmCurve*> sCleanUpTot(&sTotalOrigCurves);
  SmObjsDelete<      SmCurve*> sCleanUpOffsets(&sOffsets);

  // For every Composite Curve - accumulate the arrays,
  //  - sTotalOrigCurves  = appended array of all copied input curve segments in a positive orientation
  //  - sClosedCurves[ii] = TRUE for a closed composite curve else FALSE for an open curve
  //  - sOffsets          = raw offset curve for every parent skipping degenerate offsets and having
  //                              two entries for each parent when crOffsetDirections == 3 (for both)
  //                              plus any corner curves built to fill in offset gaps.
  //  - in each iteration build
  //      sOrigCurves = copy of each segment curve in a positive orientation for current CompositeCurve
  //                    This array gets appended to sTotalOrigCurves and ReSet at the end of each iteration.
  for(ii=0;ii<crCurvesToOffset.GetSize();ii++)
    {
      SmCompositeCurve *pCompCrv = crCurvesToOffset[ii];

      sOrigCurves.ReSet();
      sClosedCurves.Add( FALSE );

      // eOffsetDirection: Don't forget negative offset distances remembered in bFlip value.
      SmOffsetDirectionType eOffsetDirection = SM_OD_UNKNOWN;
      if(crOffsetDirections[ii] == 1 )
        {
          eOffsetDirection = bFlip ? SM_OD_RIGHT_HAND_SIDE : SM_OD_LEFT_HAND_SIDE;
        }
      else if(crOffsetDirections[ii] == 2 )
        {
          eOffsetDirection = bFlip ? SM_OD_LEFT_HAND_SIDE  : SM_OD_RIGHT_HAND_SIDE;
        }
      else if(crOffsetDirections[ii] == 3 )
        {
          eOffsetDirection = SM_OD_BOTH_SIDES;
        }
      else { SER(SM_ERR); }

      // for every segment of this Comp Curve.
      SmPoint3d sStartPt;
      for(jj=0; jj<pCompCrv->m_lNumSegments; jj++ )
        {
          SmCurve *pThisParent = pCompCrv->m_paSegments[jj].m_pParentCurve;

          // add copy of segment base curve to sOrigCurves after ensuring a positive parameter direction
          SmCurve *pParentCopy = NULL;
          pThisParent->Copy( crContext, pParentCopy );

          SmExtent1d sIvl = pParentCopy->GetNaturalInterval();

          if(!pCompCrv->m_paSegments[jj].m_bSameSense )
            { pParentCopy->ReverseParameterization( sIvl, sIvl ) ; }

          sOrigCurves.Add( pParentCopy );

          // Check for closed loop.
          // Check start point of 1st curve in this CompositeCurve, and end point of last.
          if(jj==0 )
            { // save start point of 1st segment
              SER( pParentCopy->EvaluatePoint( sIvl.GetMin(), sStartPt ));
            }
          if(jj == pCompCrv->m_lNumSegments-1 )
            { // save end point of last segment
              SmPoint3d sEndPoint;
              SER( pParentCopy->EvaluatePoint( sIvl.GetMax(), sEndPoint ));

              // when distance between start/end points is less than scaled tolerance
              double dScale =(1.0 + sStartPt.GetMaxDimension() );
              if(sStartPt.DistanceBetween(sEndPoint) < SM_EFF_ZERO * dScale )
                {
                  // Have closed curve.
                  sClosedCurves[ ii ] = TRUE;
                }

              // when composite curve knows it's a closed curve
              if (pCompCrv->m_bClosed)
                {
                  // Have closed curve.
                  sClosedCurves[ ii ] = TRUE;
                }
            } // end closed-CompositeCurve check.

          // Generate segment offset curve.
          double dMaxError = 0;

          // Offset to the Right if appropriate.
          if(  eOffsetDirection == SM_OD_RIGHT_HAND_SIDE
              || eOffsetDirection == SM_OD_BOTH_SIDES
             )
            {
              SmOffsetCurve *pOffsetCurve =
                      new(crContext) SmOffsetCurve(3,                   // in : dimension - 2 or 3
                                                   *pParentCopy,        // in : base curve
                                                   sIvl,                // in : target interval
                                                   crOffsetPlaneNormal, // in : normal to plane of offset
                                                   dOffsetDistance );   // in : distance of offset, a pos dist produces an
                                                                        //      offset to the right when viewed from above the offset plane.
              // discard degnerate offsets
              if(pOffsetCurve != NULL && pOffsetCurve->IsDegenerate() && bTrimResults)
                {
                  delete pOffsetCurve;
                  pOffsetCurve = NULL;
                }

              // when offset construction was successful
              if(pOffsetCurve != NULL )
                {
                  // add raw offset to sOffsets
                  sOffsets.Add( pOffsetCurve );

                  // tag the offset curve with a OffsetMapAttribute so that the offset remembers its parent curve.
                  SmOffsetMapAttribute *pMap = new(crContext) SmOffsetMapAttribute
                                                (SM_AI_OFFSETMAP,
                                                 FALSE, FALSE,
                                                 ii, jj,
                                                 0, pParentCopy, dMaxError );
                  pOffsetCurve->AddAttribute(pMap);
                }
            } // end offsetting to the Right if appropriate.

          // Offset to the Left if appropriate.
          if(  eOffsetDirection == SM_OD_LEFT_HAND_SIDE
              || eOffsetDirection == SM_OD_BOTH_SIDES)
            {
              SmOffsetCurve *pOffsetCurve =
                     new(crContext) SmOffsetCurve(3,                   // in : dimension - 2 or 3
                                                  *pParentCopy,        // in : base curve
                                                  sIvl,                // in : target interval
                                                  crOffsetPlaneNormal, // in : normal to plane of offset
                                                  -dOffsetDistance );  // in : distance of offset, a pos dist produces an
                                                                       //      offset to the right when viewed from above the offset plane.

              // discard degnerate offsets
              if(pOffsetCurve != NULL && pOffsetCurve->IsDegenerate() && bTrimResults)
                {
                  delete pOffsetCurve;
                  pOffsetCurve = NULL;
                }

              // when offset construction was successful
              if(pOffsetCurve != NULL )
                {
                  // add raw offset to sOffsets
                  sOffsets.Add( pOffsetCurve );

                  // tag the offset curve with a OffsetMapAttribute so that the offset remembers its parent curve.
                  SmOffsetMapAttribute *pMap = new(crContext) SmOffsetMapAttribute
                                                (SM_AI_OFFSETMAP,
                                                 FALSE, TRUE,
                                                 ii, jj,
                                                 0, pParentCopy, dMaxError );
                  pOffsetCurve->AddAttribute(pMap);
                }
            } // end offsetting to the Left if appropriate.

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
SmBoolean bEraseMe=FALSE;
          if (bDebugMe)
            {
              if(bEraseMe)
                { smgfx_Erase() ;
                  smgfx_ChangeColor(FALSE) ;
                }
              SmCurve *pOff = sOffsets.GetLast();
              SmExtent1d sTemp1( pParentCopy->GetNaturalInterval() );
              SmExtent1d sTemp2( pOff->GetNaturalInterval() );
              smgfx_SetLook(1,2) ; smgfx_ChangeColor(TRUE); pParentCopy->Draw(&sTemp1,TRUE); sm_GraphicsLoop();
              smgfx_SetLineWidth(4) ; pOff->Draw(&sTemp2,TRUE); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
        } // end iter every jj, copying and offsetting all original
          //                              curves of this CompositeCurve.

      // State: For this CompositeCurve (within the loop on all CompCurves):
      // - sOrigCurves       = ordered, and filled with a positively oriented copy of every composite curve segment base curve
      // - sClosedCurves[ii] = TRUE for a closed composite curve or FALSE for an open curve
      // - sOffsets          = sOrigCurves raw offsets, degenerate curves are omitted
      //                       else one curve for right and left offsets or two curves for both offsets
      // - every offset curve has a SmOffsetMapAttribute marking the segment curve which was its base.

      // Now Generate Corners for concave cases

      ULONG lInsertionDelta = 0;  // number of corner curves added to sOffsets to fill corner gaps

      for(jj=0; jj<sOrigCurves.GetSize(); jj++ )
        {
          ULONG lPrevIdx = 0 ;

          // Previous curve index
          if     (jj > 0 )              { lPrevIdx = jj-1; }
          else if(sClosedCurves[ ii ] ) { lPrevIdx = sOrigCurves.GetSize() - 1; }
          else
            { continue ; } // skip beginning of un-closed composite curve corner.

          // orig curves and CurveEndTangents making up this corner
          SmVector3d sPV1[2], sPV2[2];
          const SmCurve *pPrevCrv = sOrigCurves[ lPrevIdx ];
          const SmCurve *pThisCrv = sOrigCurves[ jj ];
          SmExtent1d     sPrevIvl = pPrevCrv->GetNaturalInterval();
          SmExtent1d     sThisIvl = pThisCrv->GetNaturalInterval();
          SER( pPrevCrv->Evaluate( sPrevIvl.GetMax(), 1, TRUE, sPV1 ));
          SER( pThisCrv->Evaluate( sThisIvl.GetMin(), 1, TRUE, sPV2 ));

          // cornerPoint = PrevCrv endPoint (the center of a circular arc fillet corner)
          SmPoint3d sCorner( sPV1[0] );

          // Check the angle between the two orig curves where they meet.
          double dAngleRad;
          SER( crOffsetPlaneNormal.CCWAngleBetween( sPV1[1], sPV2[1], dAngleRad ));

          // Check whether we need to create a corner on the right side.
          //  Generate corner when origCorner angle is non zero up to just under 180 degrees
          if(  dAngleRad > SM_EFF_ZERO_SQRT
              && dAngleRad < SM_PI - SM_EFF_ZERO_SQRT
              &&(  eOffsetDirection == SM_OD_RIGHT_HAND_SIDE
                  || eOffsetDirection == SM_OD_BOTH_SIDES) )
            {
              // Create an outside corner.
              // Get the two offset points and curve directions, and connect them with a corner.

              // build offset curves again, just in case, because degenerate offset curves were not saved
              SmOffsetCurve sPrevOff( 3, *pPrevCrv, sPrevIvl,
                                      crOffsetPlaneNormal,
                                      dOffsetDistance );
              sPrevOff.SetContext( NULL );
              SmOffsetCurve sCurrOff( 3, *pThisCrv, sThisIvl,
                                      crOffsetPlaneNormal,
                                      dOffsetDistance );
              sCurrOff.SetContext( NULL );

              // get offset curveEnd positions and tangents
              SER( sPrevOff.Evaluate(sPrevIvl.GetMax(), 1, TRUE, sPV1 ));
              SER( sCurrOff.Evaluate(sThisIvl.GetMin(), 1, TRUE, sPV2 ));

              // when the distance between end points is greater than tolerance
              if(sPV1[0].DistanceBetween(sPV2[0]) >= dXSectTol3d )
                {
                  // build a corner curve(s) to fill in the corner gap
                  SmCurve *pCorner1 = NULL;
                  SmCurve *pCorner2 = NULL;
                  SmCurve *pCorner3 = NULL;
                  SER( sm_CreateCorner( crContext,                           // in : context for new object construction
                                        sCorner,                             // in : center of circular arc fillet corners
                                        sPV1[0], sPV1[1], sPV2[0], sPV2[1],  // in : corner beg and end, position and tangents
                                        eOffsetCornerType,                   // in : SM_OC_LINEAR_EXTENSION: corner = 2 lines from given ends to common linear extension xsect point.
                                                                             //      SM_OC_FILLET_CORNER: corner = fillet arc centered on crVertexPoint running to given end points
                                                                             //      SM_OC_LINEAR_CHAMFER: corner = line between given ends
                                        dOffsetDistance,                     // in : Only for SM_OC_LINEAR_CHAMFER
                                        dXSectTol3d,                         // in : only used for SM_OC_LINEAR_CHAMFER
                                        pCorner1, pCorner2, pCorner3 ));     // out: new corner curves or NULL

                  SmBoolean bC1IsDegenerate = (pCorner1 == NULL) ? TRUE : pCorner1->ApproximateLength( pCorner1->GetNaturalInterval(), 3 ) < dXSectTol3d;
                  SmBoolean bC2IsDegenerate = (pCorner2 == NULL) ? TRUE : pCorner2->ApproximateLength( pCorner2->GetNaturalInterval(), 3 ) < dXSectTol3d;
                  SmBoolean bC3IsDegenerate = (pCorner3 == NULL) ? TRUE : pCorner3->ApproximateLength( pCorner3->GetNaturalInterval(), 3 ) < dXSectTol3d;

                  // If no usable results, try to connect points with a simple curve [B632]
                  if ( bC1IsDegenerate && bC2IsDegenerate && bC3IsDegenerate )
                  {
                      // find linear extension xsect point - handle zero length tangents
                      SmBSplineCurve *pBSC = NULL;
                      SmStatus        eStat = SM_ERR;
                      double dT1 = 0.0;
                      double dT2 = 0.0;
                      if ( sPV1[1].LengthSquared() > SM_EFF_ZERO_SQ && sPV2[1].LengthSquared() > SM_EFF_ZERO_SQ )
                      {
                          eStat = smgu_LineLineClosestPoint( sPV1[0], sPV1[1], sPV2[0], sPV2[1], dT1, dT2 );
                      }

                      if ( eStat == SM_SUCCESS )
                      {
                          SmPoint3d sEnd1 = sPV1[0] + dT1 * sPV1[1];
                          SmPoint3d sEnd2 = sPV2[0] + dT2 * sPV2[1];
                          SmPoint3d xPnt = ( sEnd1 + sEnd2 ) / 2.;

                          SmTArray<SmPoint3d> sCPs;
                          SmTArray<ULONG>     sKMs;
                          SmTArray<double>    sKns;
                          sCPs.Add( sPV1[0] );
                          sCPs.Add( xPnt );
                          sCPs.Add( sPV2[0] );
                          sKMs.Add( 2 );
                          sKMs.Add( 1 );
                          sKMs.Add( 2 );
                          sKns.Add( 0. );
                          sKns.Add( .5 );
                          sKns.Add( 1. );

                          SE( SmBSplineCurve::CreateCanonical( crContext, 3, 1, sCPs, SM_CF_POLYLINE_FORM, sKMs, sKns, SM_KT_UNIFORM_KNOTS, NULL, NULL, pBSC ) );
                      }
                      else
                      {
                          SE( SmBSplineCurve::CreateLineSegment( crContext, 3, sPV1[0], sPV2[0], pBSC ) );
                      }

                      if ( pBSC != NULL)
                      {
                          pCorner3 = pBSC;
                          // remember the parent curve in an attribute
                          SmOffsetMapAttribute *pMap = new( crContext ) SmOffsetMapAttribute
                          ( SM_AI_OFFSETMAP, TRUE, FALSE, ii,
                            jj, lPrevIdx,
                            NULL, 0.0 );
                          pCorner3->AddAttribute( pMap );

                          // and add the curve to sOffsets
                          sOffsets.Add( pCorner3 );
                          lInsertionDelta++;
                      }
                  }

                  // If pCorner1 is good, use it
                  if( !bC1IsDegenerate )
                    {
                      // remember the parent curve in an attribute
                      SmOffsetMapAttribute *pMap = new( crContext ) SmOffsetMapAttribute
                                                       (SM_AI_OFFSETMAP, TRUE, FALSE, ii,
                                                        lPrevIdx, jj,
                                                        NULL, 0.0 );
                      pCorner1->AddAttribute( pMap );

                      // and add the curve to sOffsets
                      sOffsets.Add( pCorner1 );
                      lInsertionDelta++;
                    }

                  // when given pCorner2
                  if( !bC2IsDegenerate )
                    {
                      // remember the parent curve in an attribute
                      SmOffsetMapAttribute *pMap = new( crContext ) SmOffsetMapAttribute
                                                        (SM_AI_OFFSETMAP, TRUE, FALSE, ii,
                                                         jj, lPrevIdx,
                                                         NULL, 0.0 );
                      pCorner2->AddAttribute( pMap );

                      // and add the curve to sOffsets
                      sOffsets.Add( pCorner2 );
                      lInsertionDelta++;
                    }

                  // when given pCorner3
                  if( !bC3IsDegenerate )
                    {
                      // remember the parent curve in an attribute
                      SmOffsetMapAttribute *pMap = new( crContext ) SmOffsetMapAttribute
                                                        (SM_AI_OFFSETMAP, TRUE, FALSE, ii,
                                                         jj, lPrevIdx,
                                                         NULL, 0.0 );
                      pCorner3->AddAttribute( pMap );

                      // and add the curve to sOffsets
                      sOffsets.Add( pCorner3 );
                      lInsertionDelta++;
                    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
                  if (bDebugMe)
                    {
                      sm_GraphicsLoop();
                      if(pCorner1 != NULL )
                        { smgfx_SetLook(3,5, 1,0,0); pCorner1->Draw(); sm_GraphicsLoop(); }
                      if(pCorner2 != NULL )
                        { smgfx_SetLook(3,5, 1,0,0); pCorner2->Draw(); sm_GraphicsLoop(); }
                      if(pCorner3 != NULL )
                        { smgfx_SetLook(3,5, 1,0,0); pCorner3->Draw(); sm_GraphicsLoop(); }
                      sm_GraphicsLoop();
                    }
#endif // SM_DEBUG_CODE

                } // end if offset points are far enough apart.
            } // end if need to create a corner on the right-hand side.

          // Check whether we need to create a corner on the left side.
          //  Generate corner when origCorner angle is non zero down to just over -180 degrees
          if(  dAngleRad < -SM_EFF_ZERO_SQRT
              && dAngleRad > -SM_PI + SM_EFF_ZERO_SQRT
              &&(  eOffsetDirection == SM_OD_LEFT_HAND_SIDE
                  || eOffsetDirection == SM_OD_BOTH_SIDES) )
            {
              // Create an outside corner.
              // Get the two offset points and curve directions, and connect them with a corner.

              // build offset curves again, just in case, because degenerate offset curves were not saved
              SmOffsetCurve sPrevOff( 3, *pPrevCrv, sPrevIvl,
                                      crOffsetPlaneNormal,
                                     -dOffsetDistance );
              sPrevOff.SetContext( NULL );
              SmOffsetCurve sCurrOff( 3, *pThisCrv, sThisIvl,
                                      crOffsetPlaneNormal,
                                      -dOffsetDistance );
              sCurrOff.SetContext( NULL );

              // get offset curveEnd positions and tangents
              SER( sPrevOff.Evaluate( sPrevIvl.GetMax(), 1, TRUE, sPV1 ));
              SER( sCurrOff.Evaluate( sThisIvl.GetMin(), 1, TRUE, sPV2 ));

              // when the distance between end points is greater than tolerance
              if(sPV1[0].DistanceBetween(sPV2[0]) >= dXSectTol3d )
                {
                  // build a corner curve(s) to fill in the corner gap
                  SmCurve *pCorner1 = NULL;
                  SmCurve *pCorner2 = NULL;
                  SmCurve *pCorner3 = NULL;
                  SER( sm_CreateCorner( crContext,                          // in : context for new object construction
                                        sCorner,                            // in : center of circular arc fillet corners
                                        sPV1[0], sPV1[1], sPV2[0], sPV2[1], // in : corner beg and end, position and tangents
                                        eOffsetCornerType,                  // in : SM_OC_LINEAR_EXTENSION: corner = 2 lines from given ends to common linear extension xsect point.
                                                                            //      SM_OC_FILLET_CORNER: corner = fillet arc centered on crVertexPoint running to given end points
                                                                            //      SM_OC_LINEAR_CHAMFER: corner = line between given end points
                                        dOffsetDistance,                    // in : Only for SM_OC_LINEAR_CHAMFER
                                        dXSectTol3d,                        // in : only used for SM_OC_LINEAR_CHAMFER
                                        pCorner1, pCorner2, pCorner3 ));    // out: new corner curves or NULL

                  SmBoolean bC1IsDegenerate = (pCorner1 == NULL) ? TRUE : pCorner1->ApproximateLength( pCorner1->GetNaturalInterval(), 3 ) < dXSectTol3d;
                  SmBoolean bC2IsDegenerate = (pCorner2 == NULL) ? TRUE : pCorner2->ApproximateLength( pCorner2->GetNaturalInterval(), 3 ) < dXSectTol3d;
                  SmBoolean bC3IsDegenerate = (pCorner3 == NULL) ? TRUE : pCorner3->ApproximateLength( pCorner3->GetNaturalInterval(), 3 ) < dXSectTol3d;

                  // If no usable results, try to connect points with a line
                  if ( bC1IsDegenerate && bC2IsDegenerate && bC3IsDegenerate )
                  {
                      // find linear extension xsect point - handle zero length tangents
                      SmBSplineCurve *pBSC = NULL;
                      SmStatus        eStat = SM_ERR;
                      double dT1 = 0.0;
                      double dT2 = 0.0;
                      if ( sPV1[1].LengthSquared() > SM_EFF_ZERO_SQ && sPV2[1].LengthSquared() > SM_EFF_ZERO_SQ )
                      {
                          eStat = smgu_LineLineClosestPoint( sPV1[0], sPV1[1], sPV2[0], sPV2[1], dT1, dT2 );
                      }

                      if ( eStat == SM_SUCCESS )
                      {
                          SmPoint3d sEnd1 = sPV1[0] + dT1 * sPV1[1];
                          SmPoint3d sEnd2 = sPV2[0] + dT2 * sPV2[1];
                          SmPoint3d xPnt = ( sEnd1 + sEnd2 ) / 2.;

                          SmTArray<SmPoint3d> sCPs;
                          SmTArray<ULONG>     sKMs;
                          SmTArray<double>    sKns;
                          sCPs.Add( sPV1[0] );
                          sCPs.Add( xPnt );
                          sCPs.Add( sPV2[0] );
                          sKMs.Add( 2 );
                          sKMs.Add( 1 );
                          sKMs.Add( 2 );
                          sKns.Add( 0. );
                          sKns.Add( .5 );
                          sKns.Add( 1. );

                          SE( SmBSplineCurve::CreateCanonical( crContext, 3, 1, sCPs, SM_CF_POLYLINE_FORM, sKMs, sKns, SM_KT_UNIFORM_KNOTS, NULL, NULL, pBSC ) );
                      }
                      else
                      {
                          SE( SmBSplineCurve::CreateLineSegment( crContext, 3, sPV1[0], sPV2[0], pBSC ) );
                      }

                      if ( pBSC != NULL)
                      {
                          pCorner3 = pBSC;
                          // remember the parent curve in an attribute
                          SmOffsetMapAttribute *pMap = new( crContext ) SmOffsetMapAttribute
                          ( SM_AI_OFFSETMAP, TRUE, FALSE, ii,
                            jj, lPrevIdx,
                            NULL, 0.0 );
                          pCorner3->AddAttribute( pMap );

                          // and add the curve to sOffsets
                          sOffsets.Add( pCorner3 );
                          lInsertionDelta++;
                      }
                  }
                  // when given pCorner1
                  if( !bC1IsDegenerate )
                    {
                      // remember the parent curve in an attribute
                      SmOffsetMapAttribute *pMap = new( crContext ) SmOffsetMapAttribute
                                                       (SM_AI_OFFSETMAP, TRUE, FALSE, ii,
                                                        lPrevIdx, jj,
                                                        NULL, 0.0 );
                      pCorner1->AddAttribute( pMap );

                      // and add the curve to sOffsets
                      sOffsets.Add( pCorner1 );
                      lInsertionDelta++;
                    }

                  // when given pCorner2
                  if( !bC2IsDegenerate )
                    {
                      // remember the parent curve in an attribute
                      SmOffsetMapAttribute *pMap = new( crContext ) SmOffsetMapAttribute
                                                       (SM_AI_OFFSETMAP, TRUE, FALSE, ii,
                                                        jj, lPrevIdx,
                                                        NULL, 0.0 );
                      pCorner2->AddAttribute( pMap );

                      // and add the curve to sOffsets
                      sOffsets.Add( pCorner2 );
                      lInsertionDelta++;
                    }

                  // when given pCorner3
                  if( !bC3IsDegenerate )
                    {
                      // remember the parent curve in an attribute
                      SmOffsetMapAttribute *pMap = new( crContext ) SmOffsetMapAttribute
                                                        (SM_AI_OFFSETMAP, TRUE, FALSE, ii,
                                                         jj, lPrevIdx,
                                                         NULL, 0.0 );
                      pCorner3->AddAttribute( pMap );

                      // and add the curve to sOffsets
                      sOffsets.Add( pCorner3 );
                      lInsertionDelta++;
                    }
                } // end if offset points are far enough apart.
            } // end if need to create a corner on the left-hand side.

        } // end loop on segs of this CompositeCurve, doing corners.

      // append sOrigCurve array to sTotalOrigCurves array
      sTotalOrigCurves.Append( sOrigCurves );
      sOrigCurves.ReSet();

    } // end iter every CompositeCurves to offset.

    if( sOffsets.GetSize() <= 0 )
        return( SM_ERR );

  // State:
  //  - sTotalOrigCurves  = appended array of all copied input curve segments in a positive orientation
  //  - sClosedCurves[ii] = TRUE for a closed composite curve else FALSE for an open curve
  //  - sOffsets          = raw offset curve for every parent skipping degenerate offsets and having
  //                              two entries for each parent when crOffsetDirections == 3 (for both)
  //                              plus any corner curves built to fill in offset gaps.

  // If we are not trimming then we are done here.
  if(bTrimResults )
    {
      // Move sOffsets into sUnTrimmedOffsets - that's the input of next call.
      // sOffset array becomes the output of next call.
      SmTArray< SmCurve* > sUnTrimmedOffsets;
      sUnTrimmedOffsets.Append( sOffsets );
      SmObjsDelete< SmCurve* > sClean1( &sUnTrimmedOffsets );

      // split Untrimmed offsets at every UnTrimmedCurve/UnTrimmedCurve intersection point
      SER( SmCompositeCurve::IntersectAndTrimOffsets
             (crContext,           // in : context for new object construction
              sUnTrimmedOffsets,   // in : raw offsets of the Original Curves
              dOffsetDistance,     // in : distance curves are being offset
              dXSectTol3d,         // in : used as XSectTol3d and divided by 100 for a ShortLine length
              sTotalOrigCurves,    // in : to classify against
              sOffsets)) ;         // out: Trimmed actual offsets built from input raw offsets minus 
                                   //      those in the 'forbidden zone.' Those closer to some other
                                   //       UnTrimmedCurve than dOffsetDistance.
    }

  // State: sOffsets contains all offset curves, trimmed or not.

  // Convert sOffsets curves to SmBSplineCurves for output.
  SmTArray<double> sMaxGaps3d ;
  sm_convert_to_BSplineCurves( crContext,                 // in : context for new object construction
                               sOffsets,                  // in : input array of curves to be approximated by BSpline curves
                               dApproxTol3d,              // in : Max allowed distance between input and approx curves
                               bOptMatchParameterization, // in : TRUE = approxCurve(param) within 3DApproxTol of ThisCurve(param) for all param values
                                                          //      FALSE= ApproxCurve(param) within 3d ApproxTol of ThisCurve(AnyParam)
                                                          //      note: FALSE produces lower control point count curves for slightly more cost.
                               rTrimmedOffsets,           // out: array of input curve approximations to tol = d3DApproxTol
                               sMaxGaps3d,                // out: max error seen in making approximations
                               FALSE) ;                   // in : TRUE = if BSplineCurve just copy it (preserves CrvParams)
                                                          //      FALSE= approximate BSplineCurves (changes CrvParams), NonBSplineCrvs always Approximated
                                                          //      default:[FALSE]

  // set output
  if(pOptMaxGaps3d) { pOptMaxGaps3d->Append(sMaxGaps3d) ; }

  // Here we have completed successfully.
  return SM_SUCCESS;

} // end SmCompositeCurve::CreateOffsetsOfManyCurves

/*******************************************************************//**
PURPOSE: Given a set of raw offset curves and the original curves,
     find actual offset by intersecting and trimming.

NOTES: Subdivide offset curves at intersections and remove any
      curves within the offset distance of original
***********************************************************************/
SmStatus SmCompositeCurve::IntersectAndTrimOffsets
 (const SmContext                & crContext,              // in : context for new object construction
  const SmTArray<SmCurve*>       & crOffsets,              // in : raw offsets of the Original Curves
  double                           dOffsetDistance,        // in : distance curves are being offset
  double                           dXSectTol3d,            // in : used as XSectTol3d and divided by 100 for a ShortLine length
  const SmTArray<const SmCurve*> & crTotalOriginalCurves,  // in : to classify against
  SmTArray<SmCurve*>             & rTrimmedOffsets)        // out: Trimmed actual offsets built from input raw offsets minus 
                                                           //      those in the 'forbidden zone.' Those closer to some other
                                                           //      UnTrimmedCurve than dOffsetDistance.
{
  // init output
  rTrimmedOffsets.ReSet() ;

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe=FALSE;
  SmBoolean bEraseMe=FALSE;
  SmVector3d sCurrColor ;
#endif // SM_DEBUG_CODE

  // First, check for 'incomplete bow-ties' that can exist at the end of a curve.
  // See comments in sm_CheckBowTies().
  //   Get list of all open CompositeCurve EndPoints that are within
  //   the 'forbidden zone' - closer than dOffsetDistance, to one of the Original Curves.
  SmTArray<SmPoint3d> sPointsToTest;
  sm_CheckBowTies(crOffsets,             // in : raw offsets of the Original Curves 
                  dOffsetDistance,       // in : Offset Distance 
                  dXSectTol3d,           // in : max distance between coincident points, GWC: Should be an XSectTol3d
                  crTotalOriginalCurves, // in : to classify against 
                  sPointsToTest) ;       // out: open CompositeCurve EndPoints 
                                         //       found to be in the 'forbidden zone' 
                                         //      (closer than dOffsetDistance to some crOrigCurves.) 
  // locals
  ULONG                  lNumOffs = crOffsets.GetSize();
  SmTArray<SmCurve*>     sTrimmedOffsets;
  SmObjsDelete<SmCurve*> sCleanUp3(&sTrimmedOffsets);
  SmSolutionArray        sSolutions;
  SmTArray<double>      *pBreaks = new SmTArray<double> [lNumOffs] ;
  SmTypedArrayDelete<SmTArray<double> > sClean(pBreaks) ;

  //  SmTArray< SmTArray<double> >       sBreaks;
  //  sBreaks.SetSize(lNumOffs) ;
  //  SmArraysDelete< SmTArray<double> > sCleanArray(&sBreaks) ;

  // Intersect every offset curve with every offset curve, including itself.

  // for every curve - split curve at all curve/curve intersections and put segments into sTrimmedOffsets
  for(ULONG ii=0; ii<lNumOffs; ii++ )
    {
      SmCurve          * pCurr       = crOffsets[ii];
      SmTArray<double> & rThisBreaks = pBreaks[ii] ;
      SmExtent1d         sCurrIvl    = pCurr->GetNaturalInterval();

      // build list of all intersections of pCurr with all other offset curves
      SER( sm_InsertSortedBreaks( rThisBreaks, sCurrIvl.GetMin() ));
      SER( sm_InsertSortedBreaks( rThisBreaks, sCurrIvl.GetMax() ));

      // Complete double loop on all offset curves: inner loop.
      // We have to loop over all other curves for this curve,
      // to collect all of the intersections with the original curves,
      // and then trim the outer-loop curve to all of them in one shot.
      for(ULONG jj=ii;jj<lNumOffs;jj++)
        {
          // Find all intersections between this pair of curves (possibly the same curve).
          SmCurve          * pOther       = crOffsets[jj];
          SmTArray<double> & rOtherBreaks = pBreaks[jj] ;
          SmExtent1d         sOtherIvl    = pOther->GetNaturalInterval() ;
          
          // was SER [B383] // GWC: why the very tight tolerance
          SE( pCurr->GlobalCurveIntersect
                ( sCurrIvl,                // in : thisCurve's interval for intersection
                 *pOther,                  // in : target OtherCurve
                  sOtherIvl,               // in : OtherCurve's interval for intersection
                  dXSectTol3d/1000.0,      // in : max 3d distance between intersecting points (GWC:why the tight tolerance)
                  sSolutions)) ;           // out: Array of Found Solutions: sSol.m_vStart[0] = thisCurve param
                                           //                                sSol.m_vStart[1] = otherCurve param
                                           // in : TRUE = Skip global coincidence check, default:[FALSE]
#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              pCurr->Dump();
              pOther->Dump();
              sSolutions.Dump();

              if(jj == 0 || bEraseMe )
                { smgfx_Erase(); }
              smgfx_SetLook( 2,3 ); smgfx_ChangeColor(jj != ii) ;
              if(jj==0) sCurrColor = smgfx_GetColor() ;
              else      smgfx_SetColor(sCurrColor) ;

              pCurr->Draw(&sCurrIvl,0); sm_GraphicsLoop();
              smgfx_SetLook( 3,4 ) ; smgfx_ChangeColor(jj != ii) ; pOther->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook( 5,6, 1,0,0) ; sSolutions.Draw() ; sm_GraphicsLoop() ;
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // Record each intersection parameter value in pBreaks in an ascending order
          for(ULONG kk=0;kk<sSolutions.GetSize();kk++)
            {
              // add this curve param to rThisBreaks array
              SmSolution & rSol = sSolutions[kk];

              // GlobalCurveIntersect() sometimes returns parameters that are
              // slightly outside the given domains.  On very short curves, the
              // parameter discrepancy can be much larger than the 3d distance.
              // That parameter difference can be big enough for Trim() to fail,
              // which causes the whole thing to fail.
              // We will assume that GlobalCurveIntersect() is returning values
              // that are within 3d tolerance of the ends (and the tolerance
              // passed to it is quite small), so clamping the values will not
              // move the points by more than tolerance.  // [B386]

              rSol.m_vStart[0] = sCurrIvl .ClampValue( rSol.m_vStart[0] );
              rSol.m_vStart[1] = sOtherIvl.ClampValue( rSol.m_vStart[1] );

              // If range of values at interval ends - remove one segment from the intervals
              //   with overlapping end curves we only need to keep one interval  [B127]
              SmBoolean bDoneWithSol = FALSE ;
              if(rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
                {

                  // (See note above. [B386])
                  rSol.m_vEnd[0] = sCurrIvl .ClampValue( rSol.m_vEnd[0] );
                  rSol.m_vEnd[1] = sOtherIvl.ClampValue( rSol.m_vEnd[1] );

                  // classify the ends
                  SmExtentPointType eCurrStart  = sCurrIvl.ClassifyPoint(rSol.m_vStart[0]) ;
                  SmExtentPointType eCurrEnd    = sCurrIvl.ClassifyPoint(rSol.m_vEnd[0]) ;
                  SmExtentPointType eOtherStart = sOtherIvl.ClassifyPoint(rSol.m_vStart[1]) ;
                  SmExtentPointType eOtherEnd   = sOtherIvl.ClassifyPoint(rSol.m_vEnd[1]) ;

                  // if intervals are overlapping end
                  if(   (eCurrStart  != SM_EP_INSIDE || eCurrEnd  != SM_EP_INSIDE)
                     && (eOtherStart != SM_EP_INSIDE || eOtherEnd != SM_EP_INSIDE))
                    {
                      // trim the overlap off the break point array
                      SM_ASSERT(rSol.m_vStart[0] < rSol.m_vEnd[0])
                      SmExtent1d sTrimIvl ;
                      if (eCurrStart == SM_EP_START)
                        {
                          sm_TrimFirstBreak( rThisBreaks, rSol.m_vEnd[0]) ;
                          sTrimIvl.SetMinMax( rSol.m_vEnd[0], sCurrIvl.GetMax()) ;
                        }
                      else
                        {
                          SM_ASSERT(eCurrEnd == SM_EP_END) ;
                          sm_TrimLastBreak( rThisBreaks, rSol.m_vStart[0]) ;
                          sTrimIvl.SetMinMax( sCurrIvl.GetMin(), rSol.m_vStart[0]) ;
                        }
                      pCurr->Trim( sTrimIvl ) ; // may snap sIvl by tol to existing knots
                      sCurrIvl     = sTrimIvl ;
                      bDoneWithSol = TRUE ;
                    } // end overlapping range sol branch
                } // end first sol is range check

              // point solutions and nonOverlappingEnd range solutions
              if(!bDoneWithSol)
                {
                  // add Curr param to rThisBreaks array
                  double dT = rSol.m_vStart[0];
                  SER( sm_InsertSortedBreaks( rThisBreaks, dT ));

                  // add other curve param to rOtherBreaks array
                  dT = rSol.m_vStart[1];
                  SER( sm_InsertSortedBreaks( rOtherBreaks, dT ));

                  // If range of values not at interval ends - add those points as well
                  if(rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES )
                    {
                      dT = rSol.m_vEnd[0];
                      SER( sm_InsertSortedBreaks( rThisBreaks, dT ));

                      // add other curve param to rOtherBreaks array
                      dT = rSol.m_vEnd[1];
                      SER( sm_InsertSortedBreaks( rOtherBreaks, dT ));
                    } // end range-of-values check
                } // end Point and nonOverlappingEnd range solution branch
            } // end iter kk - for each solution found
        } // end iter jj - inner for every offset curves.

      // Before we classify, we have to check for 'incomplete bow-ties':
      // see description in sm_CheckBowTies().
      sm_FixBowTie( pCurr,                // in : curve to test
                    sPointsToTest,        // in : list of Open CompositeCurve EndPoints known to be
                                          //       closer to one of the raw offset curves (in the forbidden zone)
                    dOffsetDistance,      // in : find point/curve distance
                    dXSectTol3d,          // in : not used
                    rThisBreaks );        // in/out: growing list appended to with
                                          //         all locations on pCurr dOffsetDistance
                                          //         from any of the sPointsToTest.

      // Now that we have all of the sorted breaks[ii] of the curve --

      // copy and trim pCurr into intervals for every pair of break points and save those in sTrimmedOffsets
      for(ULONG jj=1; jj<rThisBreaks.GetSize(); jj++ )
        {
          // Filter out really small spans.
          if(rThisBreaks[jj]-rThisBreaks[jj-1] < SM_EFF_ZERO_SQRT )
            { continue; }

          // Copy and trim the curve.
          SmCurve *pTrimmed;
          SER( pCurr->Copy( crContext, pTrimmed ));
          SmExtent1d sTrimIvl( rThisBreaks[jj-1], rThisBreaks[jj] );
          SER( pTrimmed->Trim( sTrimIvl )) ; // may snap sIvl by tol to existing knots

          // Filter out really small curves.
          if(pTrimmed->ApproximateLength(sTrimIvl,5) < dXSectTol3d / 100.0 )
            {
              delete pTrimmed; pTrimmed = NULL ;
            }
          else
            {
              sTrimmedOffsets.Add( pTrimmed );
            }
        } // end iter jj - each break - making a new split curve segment to place into sTrimmedOffsets
    } // end iter ii - for every offset curves

  // Now that we have all of the trimmed offsets --
  // classify each of them to see which are kept.

  SmTArray<SmCurve*> sClassifiedOffsets;
  SmObjsDelete<SmCurve*> sCleanUp4(&sClassifiedOffsets);

  SmCurve *pCurr = NULL, *pKeep = NULL;
  SmBoolean bKeepCurve = false;

  // for every raw offset interval
  for (ULONG ii=0; ii<sTrimmedOffsets.GetSize(); ii++ )
    {
      pCurr = sTrimmedOffsets[ii];

      // see if it's closer than dOffsetDistance to any of the OriginalCurves
      SER( sm_CurveClassify(*pCurr,                  // in : Trimmed Offset Curve to classify
                             dOffsetDistance,        // in : size of the offset
                             dXSectTol3d,            // in : passed to GlobalPointSolve() as the dDistanceTolerance, GWC:should be XSectTol3d
                             crTotalOriginalCurves,  // in : originals that were offset
                             bKeepCurve)) ;          // out: TRUE = keep curve being checked
                                                     //      FALSE= discard curve being checked

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          smgfx_SetLook( 4,6, 0,0,0 );
          pCurr->Dump();
          if(bEraseMe )
            { smgfx_Erase(); }
          sm_GraphicsLoop();
          smgfx_ChangeColor(ii != 0);
          pCurr->DrawWDeriv(pCurr->GetNaturalInterval(),0); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
      // place a copy of the keep intervals into sClassifiedOffsets
      if(bKeepCurve )
        {
          SER( pCurr->Copy( crContext, pKeep ));
          sClassifiedOffsets.Add( pKeep );
        }
    } // end iter every trimmed interval - placing copies of keep intervals in sClassifiedOffsets

  // Here we have completed successfully.  Load results and return.
  rTrimmedOffsets.Append( sClassifiedOffsets );
  sCleanUp4.Clear();

  // all done
  return SM_SUCCESS;

} // end SmCompositeCurve::IntersectAndTrimOffsets

/*******************************************************************//**
PURPOSE: Offset the composite curve.  This offset will extend joints
    which are convex and intersect things which are concave.  Note that the
    offset distance should always be positive and the direction of the
    offset is controlled by eOffsetDirection.  It will offset either to the
    left relative to the positive side of the offset plane, to the right,
    or to both sides at the same time.

    Note that the resulting curves will have attributes (SmOffsetMapAttribute)
    which describes where it came from and how it was generated.

NOTES:
    convenience routine - passes call to CreateOffsetsOfManyCurves().

    There are some situations where the current algorithm may
    fail.  These are the self-intersections and near self intersection cases
    where only one offset direction is chosen.  The work around for offsetting
    a self-intersecting curve in one direction is to split the curve at the
    self intersections and process each segment separately.

    Please note that the components of the composite curve must be smooth
    B-Spline curves (G1) but that the composite itself may have corners
    between segments of the composite (C0).  If you wish to offset a B-Spline
    curve which has corners, you must break it up into separate segments and
    create a composite.

    This offsetting algorithm DOES NOT require that the curves are all in a
    plane whose normal is the offset plane normal.  However, the results
    of offsetting non-planar curves are somewhat less predictable.

    Suggestion:This may create 'cubic' lines on rTrimmedOffset, so you
    might call pBSC->DegreeReduction

    WARNING: The trimmed Offsets may be returned in a 'random' order,
    not necessarily contiguous.

***********************************************************************/
SmStatus SmCompositeCurve::CreateTrimmedOffsets           // eff: call CreateOffsetsOfManyCurves for this curve.
  (const SmContext           & crContext,                 // in : context for new object construction
   double                      dXSectTol3d,               // in : Minimum distance at which offset curve end-gaps are filled with corner curves
   double                      dApproxTol3d,              // in : Tolerance to which BSpline Approximations to exact offset curves are built
   const SmVector3d          & crOffsetPlaneNormal,       // in : Defines, along with the curve's parameter direction,
                                                          //      the right and left hand offset directions.
   SmOffsetCornerType          eOffsetCornerType,         // in : SM_OC_LINEAR_EXTENSION: corner = 2 lines from given ends to common linear extension xsect point.
                                                          //      SM_OC_FILLET_CORNER: corner = fillet arc centered on crVertexPoint running to given end points
                                                          //      SM_OC_LINEAR_CHAMFER: corner = line between given end points (result is actually within offset distance so bTrimResults must = false)
   SmOffsetDirectionType       eOffsetDirection,          // in : Corresponding direction of each composite
                                                          //      curve member to offset.  1-LEFT, 2-RIGHT, 3-BOTH
   SmBoolean                   bTrimResults,              // in : TRUE = concave raw offsets are intersected and trimmed back to common intersection points
                                                          //      FALSE= skip trim step
   double                      dOffsetDistance,           // in : offset distance (a negative value negates the offset direction)
   SmTArray<SmBSplineCurve*> & rTrimmedOffsets,           // out: Resulting offset curves are SmBSplineCurves and will have 
                                                          //      attribute attached describing origination of curve
   SmBoolean                   bOptMatchParameterization) // in : TRUE = if BSplineCurve just copy it (preserves CrvParams)
                                                          //      FALSE= approximate BSplineCurves (changes CrvParams), NonBSplineCrvs always Approximated
                                                          //      default:[FALSE]
  const
{
  rTrimmedOffsets.ReSet();

  // Offset distance must be positive.
  if(dOffsetDistance < 0.0)
    {
      dOffsetDistance = - dOffsetDistance;
      if     (eOffsetDirection == SM_OD_LEFT_HAND_SIDE  ) { eOffsetDirection =  SM_OD_RIGHT_HAND_SIDE; }
      else if(eOffsetDirection == SM_OD_RIGHT_HAND_SIDE ) { eOffsetDirection =  SM_OD_LEFT_HAND_SIDE; }
    }

  // local for CreateOffsetsOfManyCurves call
  SmTArray<SmCompositeCurve*> sComposites;
  sComposites.Add( SM_CONST_CAST( SmCompositeCurve*, this ));

  // local for CreateOffsetsOfManyCurves call
  SmTArray<ULONG> sSides;
  if     (eOffsetDirection == SM_OD_LEFT_HAND_SIDE  ) { sSides.Add(1); }
  else if(eOffsetDirection == SM_OD_RIGHT_HAND_SIDE ) { sSides.Add(2); }
  else                                                  { sSides.Add(3); }

  // pass the call along
  SER( SmCompositeCurve::CreateOffsetsOfManyCurves
         (crContext,                    // in : context for new object construction
          sComposites,                  // in : Array of composite curves to offset
          sSides,                       // in : Corresponding direction of each composite
                                        //      curve to offset.  1-LEFT, 2-RIGHT, 3-BOTH
          dXSectTol3d,                  // in : Minimum distance at which offset curve end-gaps are filled with corner curves
          dApproxTol3d,                 // in : Tolerance to which BSpline Approximations to exact offset curves are built
          crOffsetPlaneNormal,          // in : Defines, along with the curve's parameter direction,
                                        //      the right and left hand offset directions.
          eOffsetCornerType,            // in : SM_OC_LINEAR_EXTENSION: corner = 2 lines from given ends to common linear extension xsect point.
                                        //      SM_OC_FILLET_CORNER: corner = fillet arc centered on crVertexPoint running to given end points
                                        //      SM_OC_LINEAR_CHAMFER: corner = line between endpoints (result is actually within offset distance so bTrimResults must = false).
          bTrimResults,                 // in : TRUE = concave raw offsets are intersected and trimmed back to common intersection points
                                        //      FALSE= skip trim step
          dOffsetDistance,              // in : offset distance (a negative value negates the offset direction)
          rTrimmedOffsets,              // out: Resulting offset curves will have attribute attached
                                        //      describing origination of curve
          bOptMatchParameterization));  // in : TRUE = if BSplineCurve just copy it (preserves CrvParams)
                                        //      FALSE= approximate BSplineCurves (changes CrvParams), NonBSplineCrvs always Approximated
                                        //      default:[FALSE]
                                        // out: optional MaxGap for each output approximation
  // all done
  return SM_SUCCESS;

} // end SmCompositeCurve::CreateTrimmedOffsets

/*******************************************************************//**
PURPOSE: Make a single SmBSplineCurve from this composite curve.

NOTES:  The composite curves segments are modified to have the same
  degree and dimension.  Otherwise its memory is left alone.

  The output SmBSplineCurve contains copies of all the control point,
  knot, and weight values of the input curve segments.
***********************************************************************/
SmStatus SmCompositeCurve::MakeCompositeNurb
  (const SmContext & crContext,         // in : context for new object construction
   SmBSplineCurve *& rpCompositeNurb)   // out: new single Nurb curve spanning all segments
  const
{
  // locals
  SmTArray<SmBSplineCurve*> sSegments;
  SmTArray<SmBoolean>       sSenses;

  // for every composite segment
  for (ULONG i=0; i<m_lNumSegments; i++)
    {
      // get arrays of composite curve segments and their senses
      SmBSplineCurve *pBSC = SM_CAST_PTR(SmBSplineCurve,m_paSegments[i].m_pParentCurve);
      sSegments.Add(pBSC);
      sSenses.Add(m_paSegments[i].m_bSameSense);
    }

  // create the joined curve
  SmBSplineCurve *pCompositeBSC = NULL ;
  SER(SmBSplineCurve::CreateByJoining(crContext,sSegments,&sSenses,pCompositeBSC));
  NER(pCompositeBSC);

  // all done - set output and return
  rpCompositeNurb = pCompositeBSC;
  return SM_SUCCESS;

} // end SmCompositeCurve::MakeCompositeNurb

/*******************************************************************//**
PURPOSE: Scale, rotate and translate a Composite curve.
   If scale is passed in apply it first then the rotation and
   move defined by the placement.

NOTES:
***********************************************************************/
SmStatus SmCompositeCurve::Transform
  (const SmAxis2Placement & crRotateNMove, // in : affine rotate and move transformation      
   const SmVector3d       * cpOptScale)    // in : optional scaling about current origin point before RotateNMove
                                           //      BSplines, planes, lines, PolyBreps - support nonisotropic scaling
                                           //      other geom types only support isoptropic scaling
{
    SmVector3d sIdentityScale(1, 1, 1);

    // no work - identity transform
    if (crRotateNMove.IsIdentity() && (cpOptScale == NULL || *cpOptScale == sIdentityScale))
    {
        return SM_SUCCESS;
    }

    for (ULONG i=0; i<m_lNumSegments; i++) {
        SmCurve* pCurve = m_paSegments[i].m_pParentCurve;
        pCurve->Transform(crRotateNMove,cpOptScale);
    }
    return SM_SUCCESS;

} // end SmCompositeCurve::Transform

/*******************************************************************//**
PURPOSE: Do a trimmed offset of a single curve.

NOTES: If the curve is open and BOTH sides are specified the
    ends will be closed by 1/2 circles.
    Suggestion:This may create 'cubic' lines on rTrimmedOffset, so you
    might call pBSC->DegreeReduction

***********************************************************************/
SmStatus SmCompositeCurve::CreateOffsetsOfCurve
  (const SmContext           & crContext,              // in : context for new object construction
   const SmBSplineCurve      * cpCurveToOffset,        // in : Single curve to offset
   ULONG                       lOffsetDirection,       // in : Corresponding direction of each composite
                                                       //      curve to offset.  1-LEFT, 2-RIGHT, 3-BOTH
   double                      dXSectTol3d,            // in : min distance between distinct points
   double                      dApproxTol3d,           // in : max allowed distance allowed between a curve and its approximation
   const SmVector3d          & crOffsetPlaneNormal,    // in : Defines, along with the curve's parameter direction,
                                                       //      the right and left hand offset directions.
   SmBoolean                   bTrimResults,           // in : TRUE = trim RawOffset to close offsets of closed lines and remove bowties
                                                       //      FALSE= don't
   double                      dOffsetDistance,        // in : distance curve is being offset
   SmTArray<SmBSplineCurve*> & rTrimmedOffsets,        // out: Resulting offset curves will have attribute attached
                                                       //      describing origination of curve
   SmBoolean               bOptMatchParameterization,  // in : TRUE = approxCurve(param) within 3DApproxTol of ThisCurve(param) for all param values
                                                       //      FALSE= ApproxCurve(param) within 3d ApproxTol of ThisCurve(AnyParam)
                                                       //      note: FALSE produces lower control point count curves for slightly more cost.
   SmTArray<double>          * pOptMaxGap3d)           // out: optional array of MaxGap3d values for each TrimmedOffset
                                                       //      NULL to ignore.  default:[NULL]
{
  // The algrothm used here offsets each curve.
  // Then it generates extensions and then it intersects
  // and classifies everything.

  // init output
  rTrimmedOffsets.ReSet();
  if(pOptMaxGap3d) { pOptMaxGap3d->ReSet(); }

  // Offset distance must be positive.
  SmBoolean bFlip = FALSE;
  if(dOffsetDistance < 0.0)
    {
      dOffsetDistance = - dOffsetDistance;
      bFlip = TRUE;
    }

  // locals
  SmTArray<SmCurve*>     sOffsets;
  SmObjsDelete<SmCurve*> sCleanUpOffsets( &sOffsets );
  SmPoint3d              sStartPt, sEndPoint;
  SmExtent1d             sIvl = cpCurveToOffset->GetNaturalInterval();

  // Set our local eOffsetDirection.
  // Don't forget bFlip.
  SmOffsetDirectionType eOffsetDirection = SM_OD_UNKNOWN;
  if(lOffsetDirection == 1 )
    {
      eOffsetDirection = bFlip ? SM_OD_RIGHT_HAND_SIDE : SM_OD_LEFT_HAND_SIDE;
    }
  else if(lOffsetDirection == 2 )
    {
      eOffsetDirection = bFlip ? SM_OD_LEFT_HAND_SIDE  : SM_OD_RIGHT_HAND_SIDE;
    }
  else if(lOffsetDirection == 3 )
    {
      eOffsetDirection = SM_OD_BOTH_SIDES;
    }
  else { SER(SM_ERR); }

  // get Curve end points
  SER( cpCurveToOffset->EvaluatePoint( sIvl.GetMin(), sStartPt ));
  SER( cpCurveToOffset->EvaluatePoint( sIvl.GetMax(), sEndPoint ));

  // have closed curve
  double dScaleZero = SM_EFF_ZERO * (1.0 + sStartPt.GetMaxDimension());
  SmBoolean bClosedCurve = sStartPt.DistanceBetween( sEndPoint ) < dScaleZero ;

  // Generate individual offset curves
  if(  eOffsetDirection == SM_OD_RIGHT_HAND_SIDE
      || eOffsetDirection == SM_OD_BOTH_SIDES )
    {
      // build an explicit offset of Curve
      SmOffsetCurve *pOffsetCurve = new(crContext) SmOffsetCurve(3,
                                                                *cpCurveToOffset,
                                                                sIvl,
                                                                crOffsetPlaneNormal,
                                                                dOffsetDistance );

      if(pOffsetCurve != NULL && pOffsetCurve->IsDegenerate() )
        {
          delete pOffsetCurve;
          pOffsetCurve = NULL;
        }
      if(pOffsetCurve != NULL )
        {
            sOffsets.Add( pOffsetCurve );
        }
    }
  if(  eOffsetDirection == SM_OD_LEFT_HAND_SIDE
      || eOffsetDirection == SM_OD_BOTH_SIDES )
    {
      // build an explicit offset of Curve
      SmOffsetCurve *pOffsetCurve = new(crContext) SmOffsetCurve(3,
                                                                 *cpCurveToOffset,
                                                                 sIvl,
                                                                 crOffsetPlaneNormal,
                                                                 -dOffsetDistance );

      if(pOffsetCurve != NULL && pOffsetCurve->IsDegenerate() )
        {
          delete pOffsetCurve;
          pOffsetCurve = NULL;
        }
      if(pOffsetCurve != NULL )
        {
          sOffsets.Add( pOffsetCurve );
        }
    }
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
  if (bDebugMe)
    {
      SmCurve *pOff = sOffsets.GetLast();

      sm_GraphicsLoop();
      pOff->DrawWDeriv(pOff->GetNaturalInterval(),0); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Now Generate Corners for convex cases
  ULONG lInsertionDelta = 0;

  SmVector3d sPV1[2], sPV2[2];

  if(!bClosedCurve && eOffsetDirection == SM_OD_BOTH_SIDES )
    {
      // Make 2 half-circles to close ends of offset
      SmOffsetCurve sLastOff( 3, *cpCurveToOffset, sIvl, crOffsetPlaneNormal,
                              dOffsetDistance );
      sLastOff.SetContext(NULL);

      SmOffsetCurve sCurrOff( 3, *cpCurveToOffset, sIvl, crOffsetPlaneNormal,
                             -dOffsetDistance);
      sCurrOff.SetContext(NULL);

      SmPoint3d sCenter;

      SER( cpCurveToOffset->EvaluatePoint( sIvl.GetMin(), sCenter ));
      SER( sLastOff.Evaluate( sIvl.GetMin(), 1, TRUE, sPV1 ));
      SER( sCurrOff.Evaluate( sIvl.GetMin(), 1, TRUE, sPV2 ));
      SmCurve *pCorner1 = NULL ;
      SmCurve *pCorner2 = NULL ;
      SmCurve *pCorner3 = NULL ;
      sPV1[1] = - sPV1[1];
      sPV2[1] = - sPV2[1];
      SER( sm_CreateCorner( crContext, sCenter, sPV1[0], sPV1[1], sPV2[0], sPV2[1],
                            SM_OC_FILLET_CORNER, dOffsetDistance, dXSectTol3d, pCorner1, pCorner2, pCorner3 ));
      if(pCorner1) sOffsets.Add( pCorner1 );

      SER( cpCurveToOffset->EvaluatePoint( sIvl.GetMax(), sCenter ));
      SER( sLastOff.Evaluate( sIvl.GetMax(), 1, TRUE, sPV1 ));
      SER( sCurrOff.Evaluate( sIvl.GetMax(), 1, TRUE, sPV2 ));
      SER( sm_CreateCorner( crContext, sCenter, sPV1[0], sPV1[1], sPV2[0], sPV2[1],
          SM_OC_FILLET_CORNER, dOffsetDistance, dXSectTol3d, pCorner1, pCorner2, pCorner3 ));
      if (pCorner1) sOffsets.Add( pCorner1 );

    } // end if not closed and offset Both sides: close ends with two half-circles.

  else if(bClosedCurve )
    {
      // If the offset curve is not closed, see about capping it.
      // Check angle it makes with itself.
      SER( cpCurveToOffset->Evaluate( sIvl.GetMin(), 1, TRUE, sPV1 ));
      SER( cpCurveToOffset->Evaluate( sIvl.GetMax(), 1, TRUE, sPV2 ));
      SmPoint3d sCorner( sPV1[0] );
      double dAngle;
      SER( crOffsetPlaneNormal.CCWAngleBetween( sPV1[1], sPV2[1], dAngle ));

      // If angle is non zero up to 180 degrees then we will try
      // to generate a corner for it.

      if(   dAngle < -SM_EFF_ZERO_SQRT
           && dAngle > -SM_PI + SM_EFF_ZERO_SQRT
           &&(   eOffsetDirection == SM_OD_RIGHT_HAND_SIDE
                || eOffsetDirection == SM_OD_BOTH_SIDES) )
        {
          SmOffsetCurve sLastOff( 3, *cpCurveToOffset, sIvl, crOffsetPlaneNormal,
              dOffsetDistance );
          sLastOff.SetContext(NULL);

          SmOffsetCurve sCurrOff(3, *cpCurveToOffset, sIvl, crOffsetPlaneNormal,
              dOffsetDistance );
          sCurrOff.SetContext(NULL);

          SER( sLastOff.Evaluate( sIvl.GetMax(), 1, TRUE, sPV1 ));
          SER( sCurrOff.Evaluate( sIvl.GetMin(), 1, TRUE, sPV2 ));
          if(sPV1[0].DistanceBetween( sPV2[0] ) >= dXSectTol3d )  // gwc: really an XSectTol3d
            {
              SmCurve *pCorner1 = NULL;
              SmCurve *pCorner2 = NULL;
              SmCurve *pCorner3 = NULL;
              SER(sm_CreateCorner(crContext, 
                                  sCorner,
                                  sPV1[0], sPV1[1], 
                                  sPV2[0], sPV2[1],
                                  SM_OC_FILLET_CORNER, 
                                  dOffsetDistance, dXSectTol3d,
                                  pCorner1, pCorner2, pCorner3)) ;

              if (pCorner1)
                {
                  sOffsets.Add( pCorner1 );
                  lInsertionDelta ++;
                }
              if (pCorner2)
                {
                  sOffsets.Add( pCorner2 );
                  lInsertionDelta ++;
                }
            }
        }

      if(   dAngle > SM_EFF_ZERO_SQRT
           && dAngle < SM_PI - SM_EFF_ZERO_SQRT
           &&(   eOffsetDirection == SM_OD_LEFT_HAND_SIDE
                || eOffsetDirection == SM_OD_BOTH_SIDES) )
        {
          SmOffsetCurve sLastOff( 3, *cpCurveToOffset, 
                                 sIvl, 
                                 crOffsetPlaneNormal,
                                 -dOffsetDistance );
          sLastOff.SetContext(NULL);

          SmOffsetCurve sCurrOff( 3, *cpCurveToOffset, 
                                 sIvl, 
                                 crOffsetPlaneNormal,
                                 -dOffsetDistance );
          sCurrOff.SetContext(NULL);

          SER( sLastOff.Evaluate( sIvl.GetMax(), 1, TRUE, sPV1 ));
          SER( sCurrOff.Evaluate( sIvl.GetMin(), 1, TRUE, sPV2 ));
          if(sPV1[0].DistanceBetween( sPV2[0] ) >= dXSectTol3d ) // GWC: used as a XSectTol3d
            {
              SmCurve *pCorner1 = NULL;
              SmCurve *pCorner2 = NULL;
              SmCurve *pCorner3 = NULL;
              SER( sm_CreateCorner(crContext, 
                                   sCorner,
                                   sPV1[0], sPV1[1], 
                                   sPV2[0], sPV2[1],
                                   SM_OC_FILLET_CORNER, 
                                   dOffsetDistance, dXSectTol3d,
                                   pCorner1, pCorner2, pCorner3 ));

              if(pCorner1 != NULL )
                {
                  sOffsets.Add( pCorner1 );
                  lInsertionDelta++;
                }
              if(pCorner2 != NULL )
                {
                  sOffsets.Add( pCorner2 );
                  lInsertionDelta++;
                }
            }
        }
    } // end if Closed curve.


  // State: sOffsets contains all untrimmed offset curves.

  // Trim offsets if requested.
  if(bTrimResults )
    {
      // Move the offsets into another array, and have
      // IntersectAndTrimOffsets() put the trimmed offsets into sOffsets.
      SmTArray< SmCurve* > sUnTrimmedOffsets;
      sUnTrimmedOffsets.Append( sOffsets );

      SmTArray< const SmCurve* > sTotalOrigCurves;
      sTotalOrigCurves.Add( cpCurveToOffset );

      SER( SmCompositeCurve::IntersectAndTrimOffsets(crContext,          // in : context for new object construction
                                                     sUnTrimmedOffsets,  // in : raw offsets of the Original Curves
                                                     dOffsetDistance,    // in : distance curves are being offset
                                                     dXSectTol3d,        // in : used as XSectTol3d and divided by 100 for a ShortLine length
                                                     sTotalOrigCurves,   // in : to classify against
                                                     sOffsets)) ;        // out: Trimmed actual offsets built from input raw offsets minus 
                                                                         //      those in the 'forbidden zone.' Those closer to some other
                                                                         //      UnTrimmedCurve than dOffsetDistance.
    }

  // State: sOffsets contains all offset curves, trimmed or not.
  // Convert them to SmBSplineCurve's for output.

  SmTArray<double> sMaxGaps3d = 0;
  sm_convert_to_BSplineCurves( crContext,                 // in : context for new object construction
                               sOffsets,                  // in : input array of curves to be approximated by BSpline curves
                               dApproxTol3d,              // in : Max allowed distance between input and approx curves
                               bOptMatchParameterization, // in : TRUE = approxCurve(param) within 3DApproxTol of ThisCurve(param) for all param values
                                                          //      FALSE= ApproxCurve(param) within 3d ApproxTol of ThisCurve(AnyParam)
                                                          //      note: FALSE produces lower control point count curves for slightly more cost.
                               rTrimmedOffsets,           // out: array of input curve approximations to tol = d3DApproxTol
                               sMaxGaps3d,                // out: max error seen in making approximations
                               FALSE) ;                   // in : TRUE = Just copy BSplines, FALSE = Approximate them
                                                          //      default:[FALSE]

  // Here we have completed successfully.

  // set output
  if(pOptMaxGap3d) { pOptMaxGap3d->Append(sMaxGaps3d) ; }

  // all done
  return SM_SUCCESS;

} // end SmCompositeCurve::CreateOffsetsOfCurve

/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the SmCompositeCurve.

NOTES: Does not include the curve's attribute memory
***********************************************************************/
ULONG SmCompositeCurve::GetMemoryUsed  // rtn: smaller size of actually used memory in bytes
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

  // inits and locals
  rlMemoryAllocated = 0 ;
  ULONG lThisAllocated ;

  // memory for this + m_paSegments memory block
  rlMemoryAllocated = sizeof(*this) + m_lNumSegments * sizeof(SmCompositeCurveSegment) ;

  // + attribute memory
  ULONG lUsed       = rlMemoryAllocated + this->GetAttributeMemoryUsed(lThisAllocated,
                                                                       eMarkType) ;  // note: uses without increment eMarkType value
  rlMemoryAllocated = rlMemoryAllocated + lThisAllocated ;

  // + cache memory
  if ( m_pCacheObj )
  {
      lUsed += m_pCacheObj->GetMemoryUsed( lThisAllocated );
      rlMemoryAllocated += lThisAllocated;
  }

  // add in memory for SmCurve within each m_paSegments[i]
  for(ULONG ii=0;ii<m_lNumSegments;ii++)
    {
      lUsed += m_paSegments[ii].m_pParentCurve->GetMemoryUsed(lThisAllocated, eMarkType) ;
      rlMemoryAllocated += lThisAllocated ;

    }
  // all done
  return(lUsed) ;

} // end SmCompositeCurve::GetMemoryUsed

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/

SmDisplayList * SmCompositeCurve::Draw
(
  const SmExtent1d *pInterval,     // NotUsed: in : Target Interval, NULL = Use Natural Interval
                                   //      default:[NULL], Currently not used here.
  SmBoolean bAddToUIPickList,      // in : TRUE = Add this Curve to UI pick interface for debugging
                                   //      default:[FALSE]
  SmPlane * pOptOutPlane,          // NotUsed: in : Draw on this plane, NULL=draw on z=0 plane, default:[NULL]

  SmGfxArraySet    *pOptGfxSet     // in : used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                   //      NULL to ignore. default:[NULL]
) const
{
  SM_REF2(pInterval, pOptOutPlane) ;
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // when asked - add this Curve to UI pick list
  SmCurve *pCurve = (SmCurve*) this;
  if(bAddToUIPickList)
    { sm_GraphicsAddToBrepList(pCurve) ; }

  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  for (ULONG i=0; i<m_lNumSegments; i++)
    {
      SmCurve* pCrv = m_paSegments[i].m_pParentCurve;
      pCrv->Draw(NULL, FALSE, NULL, pOptGfxSet);
    }

  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(bAddToUIPickList, pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmCompositeCurve::Draw

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmDisplayList * SmCompositeCurve::DrawParams
  (const SmExtent1d *pInterval, // NotUsed: in : Target Interval, NULL = Use Natural Interval
                                //      default:[NULL], Currently not used here.
   SmBoolean bAddToUIPickList)  // in : TRUE = Add this Curve to UI pick interface for debugging
                                //      default:[FALSE]
 const
{
  SM_REF1(pInterval) ;
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // when asked - add this Curve to UI pick list
  SmCurve *pCurve = (SmCurve*) this;
  if(bAddToUIPickList)
    { sm_GraphicsAddToBrepList(pCurve) ; }

  smgfx_Open(smgfx_GetRuleColor(this));

  for (ULONG i=0; i<m_lNumSegments; i++)
    {
      SmCurve* pCrv = m_paSegments[i].m_pParentCurve;
      pCrv->DrawParams();
    }

  pRtn = smgfx_Close() ;

#else
  SM_REF1(bAddToUIPickList);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmCompositeCurve::DrawParams

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmCompositeCurveRegion::Draw
  ()
 const
{
    for (ULONG i=0; i<m_vCompositeCurves.GetSize(); i++) {
        m_vCompositeCurves[i]->Draw();
    }

} // end SmCompositeCurveRegion::Draw

/*******************************************************************//**
PURPOSE: Write SmCompositeCurve to given output stream.

NOTES:
***********************************************************************/
SmStatus SmCompositeCurve::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes
 const
{
  // file type, ASCII or BINARY
  SmFileType      eType    =  rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();

  // locals
  ULONG     ii ;

  if (eType == SM_ASCII)
    {
      rFileOut << m_lNumSegments << " SmCompositeCurve Segment Count \n";
      rFileOut << m_bClosed      << " SmCompositeCurve Closed Flag \n" ;
    }
  else
    {
      SER(rDB.WriteLong   (m_lNumSegments)) ;
      SER(rDB.WriteBoolean(m_bClosed)) ;
    }

  // for every CompositeCurveSegment
  for(ii=0;ii<m_lNumSegments;ii++)
    {
      m_paSegments[ii].WriteToDB(rDB, lDBVersionNumber) ;

    } // end iter every CompositeCurveSegment

  // all done
  return SM_SUCCESS;

} // end SmCompositeCurve::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmCompositeCurve from a given stream

NOTES:
***********************************************************************/
SmStatus SmCompositeCurve::ReadFromDB
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
         || rpNewCurve->IsKindOf(SmCompositeCurve_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmCompositeCurve *pCompositeCurve =   (rpNewCurve == NULL)
                                      ? new (crContext) SmCompositeCurve(lDim)
                                      : (SmCompositeCurve *)rpNewCurve ;

  // file type
  SmFileType     eType   = rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();

  if (eType == SM_ASCII)
    {
      rFileIn >> pCompositeCurve->m_lNumSegments ; rDB.GoToNextLine() ;
      rFileIn >> pCompositeCurve->m_bClosed ;      rDB.GoToNextLine() ;
    }
  else
    {
      SER(rDB.ReadLong(pCompositeCurve->m_lNumSegments));
      SER(rDB.ReadBoolean(pCompositeCurve->m_bClosed));
    }

  // allocate room for the segments
  ULONG lSize    = sizeof(SmCompositeCurveSegment) * pCompositeCurve->m_lNumSegments;

  // okay to use smos_Calloc on static class (SmCompositeCurveSegment) objects.
  pCompositeCurve->m_paSegments   = (SmCompositeCurveSegment*)smos_Calloc(lSize,1);

  // for every CompositeCurveSegment
  ULONG ii ;
  for(ii=0;ii<pCompositeCurve->m_lNumSegments;ii++)
    {
      SmCompositeCurveSegment *pTgtSegment = &pCompositeCurve->m_paSegments[ii] ;
      SmCompositeCurveSegment::ReadFromDB(rDB, crContext, pTgtSegment, lDBVersionNumber) ;

    } // end iter every CompositeCurveSegment

  // set the output
  rpNewCurve = pCompositeCurve ;

  // all done
  return SM_SUCCESS;

} // end SmCompositeCurve::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmCompositeCurve::IsKindOf( SM_TYPE t ) const
{
  return ((SmCompositeCurve_TYPE == t) ? TRUE : SmCurve::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmCompositeCurve::Dump(ULONG i) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T("\n%ld "), i);
  smos_WriteBuffer(sBuff);
  this->Dump();

} // end SmCompositeCurve::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmCompositeCurve::Dump(const TCHAR * message) const
{
     TCHAR sBuff[SM_TBLOCK_SIZE];
     smos_sprintf(sBuff,_T("\n%s "), message);
     smos_WriteBuffer(sBuff);
     this->Dump();

} // end SmCompositeCurve::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmCompositeCurve::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
  smos_WriteBuffer(_T("\nBegin SmCompositeCurve::Dump()")) ;

  // output cache data
  SmCurve::Dump(FALSE) ;

  smos_sprintf(sBuff,       _T("SmCompositeCurve 0x%p,  Num Segments = %ld, Closed =%d\n"),
      this,this->m_lNumSegments,this->m_bClosed);
  smos_sprintf(sBuffForFile,_T("SmCompositeCurve %s,  Num Segments = %ld, Closed =%d\n"),
      _T("notNULL"),this->m_lNumSegments,this->m_bClosed);
  smos_WriteBuffer(sBuff, sBuffForFile);
  for (ULONG i=0; i<m_lNumSegments; i++) {
      smos_sprintf(sBuff,       _T("     Curve[%ld]=0x%p,  Same Sense=%d\n"),
          i,m_paSegments[i].m_pParentCurve,m_paSegments[i].m_bSameSense);
      smos_sprintf(sBuffForFile,_T("     Curve[%ld]=%s,  Same Sense=%d\n"),
          i,m_paSegments[i].m_pParentCurve ? _T("notNULL") : _T("NULL"),m_paSegments[i].m_bSameSense);
      smos_WriteBuffer(sBuff, sBuffForFile);
      m_paSegments[i].m_pParentCurve->Dump();
  }

  smos_WriteBuffer(_T(" End SmCompositeCurve::Dump()\n")) ;

} // end SmCompositeCurve::Dump

//      /*******************************************************************//**
//      PURPOSE: Write a CompositeCurve out to a file.
//
//      NOTES: This method is primarily for use when debugging.
//      ***********************************************************************/
//      SmStatus SmCompositeCurve::WriteToFile
//        (const TCHAR *cOutputFileName,  // in : target file name
//         SmBoolean ,                    // in : bSkipHeaderWrite = TRUE = Omit Header label for this write
//         SmBoolean bNewFile)            // in : TRUE = open file and rewrite contents
//        const                           //      FALSE= open file and append to end
//      {
//          SM_ASSERT_VALID_NO_STREAM( this );
//
//          FILE *pFile;
//
//          // open file for write or append
//          if(bNewFile) { pFile = N_FileOpen((TCHAR*)cOutputFileName, _T("w+")); }
//          else         { pFile = N_FileOpen((TCHAR*)cOutputFileName, _T("a+")); }
//          if (pFile == NULL) return SM_ERR;
//
//          //      if(bNewFile)
//          //          pFile= SM_FOPEN((const TCHAR *)cOutputFileName,_T("w+"));
//          //      else
//          //          pFile= SM_FOPEN((const TCHAR *)cOutputFileName,_T("a+"));
//
//          if (pFile == NULL) return SM_ERR;
//
//          // write number of curves (segments)
//          SM_FPRINTF( pFile, _T("%d\n"),  m_lNumSegments);
//          for (ULONG i=0; i<m_lNumSegments; i++) {
//              SER(N_CrvWriteToFilePtr(((SmBSplineCurve *)m_paSegments[i].m_pParentCurve)->GetOrCreateGwNurbPointer(),(FILE*)pFile));
//          }
//          fclose(pFile);
//          return SM_SUCCESS;
//      } // end SmCompositeCurve::WriteToFile

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmCompositeCurveRegion::IsKindOf( SM_TYPE t ) const
{
  return ((SmCompositeCurveRegion_TYPE == t) ? TRUE : SmAObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmCompositeCurveRegion::Dump
  (void)
 const
{
    TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
    smos_sprintf(sBuff,       _T("SmCompositeCurveRegion 0x%p,  Num Composites = %ld\n"),
        this,this->m_vCompositeCurves.GetSize());
    smos_sprintf(sBuffForFile,_T("SmCompositeCurveRegion %s,  Num Composites = %ld\n"),
        _T("notNULL"),this->m_vCompositeCurves.GetSize());
    smos_WriteBuffer(sBuff, sBuffForFile);

    for (ULONG i=0; i<m_vCompositeCurves.GetSize(); i++) {
        m_vCompositeCurves[i]->Dump();
    }

} // end SmCompositeCurveRegion::Dump

#if 0
// begin removed code block
/*******************************************************************//**
PURPOSE: Compute the total size of the memory used by the surface cache.

NOTES:
***********************************************************************/
void SmCCRegionCache::Dump
  (void)
 const
{
    TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
    SmTArray<void*> sNodes;
    m_pTree->m_sNodeMgr.GetActiveElements(sNodes);
    smos_sprintf(sBuff,       _T("SmCCRegionCache = 0x%p, Number of Nodes = %ld"),this,sNodes.GetSize());
    smos_sprintf(sBuffForFile,_T("SmCCRegionCache = %s, Number of Nodes = %ld"),this ? _T("notNULL") : _T("NULL"),sNodes.GetSize());
    smos_WriteBuffer(sBuff, sBuffForFile);
    smos_WriteBuffer(_T("\n"));

} // end SmCCRegionCache::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmCCRegionCache::Draw
  (SmBoolean )
 const
{
#ifdef SM_GFX_CODE
  m_pCCRegion->Draw();

  SmTArray<void*> sNodes;
  m_pTree->m_sNodeMgr.GetActiveElements(sNodes);

  smgfx_Open(smgfx_GetRuleColor(this));
  for(ULONG i=0; i<sNodes.GetSize(); i++)
    {
      SmTreeNode *pNode = (SmTreeNode*)sNodes[i];

      // skip non-surface subdivision tree nodes
      if(   pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE
         && pNode->m_eAuxDataType != SM_AD_AUX_DATA
         && pNode->m_eAuxDataType != SM_AD_BEZIER_SURFACE_HEAD)
        { continue ; }

      // skip parent nodes
      if (pNode->m_pChild1 != NULL)
        { continue ; }

      SmBezierPatch  *pBezElem = (SmBezierPatch*)pNode->m_pData;
      SmBezierAux2d  *pAux     = &pBezElem->m_sAux;

      // Draw discontinuities a different color and weight
      smgfx_SetColor(0,0,0);
      pAux->m_sUVDomain.Draw();
      if (pAux->m_eNodeClass == SM_NC_ON_BOUNDARY)
        {
          SmPoint3d sMid(pAux->m_sUVDomain.Evaluate(0.5,0.5));
          smgfx_SetColor(1,0,0);
          sMid.Draw();
          smgfx_SetColor(0,0,0);
        }
      else if (pAux->m_eNodeClass == SM_NC_INSIDE)
        {
          smgfx_SetColor(0,1,0);
          SmPoint3d sMid(pAux->m_sUVDomain.Evaluate(0.5,0.5));
          sMid.Draw();
          smgfx_SetColor(0,0,0);
        }
      else if (pAux->m_eNodeClass == SM_NC_OUTSIDE)
        {
          smgfx_SetColor(0,0,1);
          SmPoint3d sMid(pAux->m_sUVDomain.Evaluate(0.5,0.5));
          sMid.Draw();
          smgfx_SetColor(0,0,0);
        }
    } // end iter every Subdivision Tree node

  // all done
  smgfx_Close();
#endif // SM_GFX_CODE

} // end SmCCRegionCache::Draw

// end removed code block
#endif // 0
