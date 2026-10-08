// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmOffsetCurve.cpp
* PURPOSE: Source file for SmOffsetCurve methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmOffsetCurve.h>
#include <SmVector2d.h>
#include <SmPseudoBox.h>
#include <SmExtent3d.h>
#include <SmDatabaseIO.h>
#include <SmAssertArray.h>

/*******************************************************************//**
PURPOSE: Constructor for Offset curve.

NOTES: 
***********************************************************************/
SmOffsetCurve::SmOffsetCurve
  (ULONG              lDimension,        // in : dimension of this curve
   const SmCurve    & crCurve,           // in : target curve being offset - not modified
   const SmExtent1d & crCurveInterval,   // in : interval of target curve being offset
   const SmVector3d & crOffsetPlaneNorm, // in : Normal to plane of offset
   double             dOffsetDistance,   // in : amount of offset distance
   SmBoolean          bOwnsCurve)        // in : TRUE = delete m_cpCurve in destructor, FALSE = don't
                                         //      default:[FALSE]
 : SmCurve(lDimension), 
   m_pCurve((SmCurve *)&crCurve), 
   m_vOffsetPlaneNorm(crOffsetPlaneNorm), 
   m_dOffsetDistance(dOffsetDistance),
   m_vCurveInterval(crCurveInterval),
   m_bIsReversed(FALSE),
   m_bOwnsCurve(bOwnsCurve)
{
  if ( m_cpContext == NULL )
    { m_cpContext = crCurve.GetContext(); }

  SE(m_vOffsetPlaneNorm.Unitize());

  if ( m_bOwnsCurve == TRUE ) { m_pCurve->SetOwner(this) ; }
} // end SmOffsetCurve::SmOffsetCurve  constructor

/*******************************************************************//**
PURPOSE: Copy constructor for SmOffsetCurve object.

NOTES: The base curve is copied when owned by crCurveToCopy, else another
       reference to the base curve is made 
***********************************************************************/
SmOffsetCurve::SmOffsetCurve
  (const SmOffsetCurve & crCurveToCopy)
 : SmCurve(crCurveToCopy),
   m_pCurve(crCurveToCopy.m_pCurve),
   m_vOffsetPlaneNorm(crCurveToCopy.m_vOffsetPlaneNorm),
   m_dOffsetDistance(crCurveToCopy.m_dOffsetDistance),
   m_vCurveInterval(crCurveToCopy.m_vCurveInterval), 
   m_bIsReversed(crCurveToCopy.m_bIsReversed),
   m_bOwnsCurve(crCurveToCopy.m_bOwnsCurve) 
{
  if(crCurveToCopy.m_bOwnsCurve == FALSE)
    { m_pCurve = crCurveToCopy.m_pCurve ; }
  else
    { 
      const SmContext *cpContext =   this->GetContext() ? this->GetContext()
                                   : crCurveToCopy.GetContext() ? crCurveToCopy.GetContext()
                                   : crCurveToCopy.m_pCurve->GetContext() ;
      SE_MSG(cpContext != NULL,
             _T("SmOffsetCurve constructor could not find context for base curve copy")) ;
      crCurveToCopy.m_pCurve->Copy(*cpContext, m_pCurve) ;
      m_bOwnsCurve = TRUE ; 
    } 

} // end SmOffsetCurve::SmOffsetCurve copied constructor

/*******************************************************************//**
PURPOSE: Equality operator for SmOffsetCurve

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmOffsetCurve::operator==
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
      SmOffsetCurve &rOther = (SmOffsetCurve &)crOther ;

      // check equivalence of these objects
      bRtn = (   (   ( m_pCurve == rOther.m_pCurve)
                  || ( m_pCurve == NULL && rOther.m_pCurve == NULL)
                  || (   m_pCurve != NULL && rOther.m_pCurve != NULL
                      && *m_pCurve == *rOther.m_pCurve))
              && m_vOffsetPlaneNorm  == rOther.m_vOffsetPlaneNorm
              && SM_IS_ZERO(m_dOffsetDistance - rOther.m_dOffsetDistance)
              && m_vCurveInterval    == rOther.m_vCurveInterval
              && m_bIsReversed       == rOther.m_bIsReversed) ;
    }

  // all done
  return bRtn ;

} // end SmOffsetCurve::operator==

/*******************************************************************//**
PURPOSE: Copy a SmOffsetCurve in a generic way.

NOTES:
***********************************************************************/
SmStatus SmOffsetCurve::Copy
  (const SmContext & crContext,
   SmCurve        *& rpNewCurve)
  const
{
    rpNewCurve = new (crContext) SmOffsetCurve(*this);
    NER(rpNewCurve);
    return SM_SUCCESS;

} // end Copy

/*******************************************************************//**
PURPOSE: Given the parameteric value of the curve determine the
    corresponding Euclidian point and optionally derivatives.  Note that
    it is possible to evaluate up to the second derivative for planar
    curves.  Non planar curves can only evaluate the first derivative.

NOTES: For discontinuous points on the curve the derivatives
    may be obtained either from the left or from the right of the 
    discontinuity.
***********************************************************************/
SmStatus SmOffsetCurve::Evaluate
 (double     dInputParameter,         // in : tgt param
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
    // Adjust parameter if reversed: mirror within the interval.
    double dParameter = dInputParameter;
    if ( m_bIsReversed )
      {
        dParameter =   m_vCurveInterval.GetMin()
                     + m_vCurveInterval.GetMax() - dInputParameter;
      }

    // If offset dist is zero, just evaluate base curve.
    if ( smos_Fabs(m_dOffsetDistance) < SM_EFF_ZERO )
      {
        SER( m_pCurve->Evaluate( dParameter, lNumDerivatives, bFromLeft,
                                   aPointAndDerivatives ));
      }

    // init output.
    ULONG i;
    for ( i = 0; i <= lNumDerivatives; i++ )
      {
        aPointAndDerivatives[i].Set( 0, 0, 0 );
      }

    // How many can we actually calculate: 3.
    const ULONG clMaxDerivs = 3;
    ULONG lNumToCalculate = smos_Min( lNumDerivatives, clMaxDerivs );

    // Eval base curve: one more deriv than we need.
    SmVector3d sPV[clMaxDerivs+2]; // One for the point, plus one extra deriv.
    if ( SM_IS_ZERO( dParameter - m_vCurveInterval.GetMax() )) bFromLeft = FALSE;
    SER( m_pCurve->Evaluate( dParameter, lNumToCalculate+1, bFromLeft, sPV, TRUE ));

    // Check for zero 1st deriv.
    if (sPV[1].LengthSquared() < SM_EFF_ZERO_SQ)
      {
        double dSign = 1.0;
        if (dParameter > m_vCurveInterval.GetMax() - 
            SM_EFF_ZERO_SQRT * m_vCurveInterval.GetLength()) 
          {
            dSign = -1.0;
            dParameter = dParameter - (SM_EFF_ZERO_SQRT * m_vCurveInterval.GetLength());
          }
        else 
          {
            dParameter = dParameter + (SM_EFF_ZERO_SQRT * m_vCurveInterval.GetLength());
          }
        SmPoint3d sPV2[4];
        SER(m_pCurve->Evaluate(dParameter,lNumToCalculate,bFromLeft,sPV2));
        sPV[1] = (sPV2[0] - sPV[0]) / (dSign * SM_EFF_ZERO_SQRT * m_vCurveInterval.GetLength());
        if (lNumToCalculate > 1) 
          {
            sPV[2] = (sPV2[1] - sPV[1]) / (dSign * SM_EFF_ZERO_SQRT * m_vCurveInterval.GetLength());
          }
        if (lNumToCalculate > 2) 
          {
            sPV[3] = (sPV2[2] - sPV[2]) / (dSign * SM_EFF_ZERO_SQRT * m_vCurveInterval.GetLength());
          }

        if (sPV[1].LengthSquared() < SM_EFF_ZERO_SQ) 
          {
            SER(SM_ERR);
          }
    } // end zero-1st-deriv check.

    // Here we have a usable first derivative.

    // Offset curve is defined as:   O(t) = C(t) + D * UB(t)
    // where UB(t) is unitized binormal of C at t.
    // The binormal is B(t) = C'(t) x N, N is our constant m_vOffsetPlaneNorm,
    // and UB(t) = B(t) / || B(t) ||.
    // The First derivative of the offset is:  O'(t) = C'(t) + D * UB'(t);
    // The derivative(s) of a unitized vector can get pretty hairy,
    // but that's already taken care of in the SmVector::UnitizedDerivative(n) methods.

    // Position:
    SmVector3d sBin = sPV[1] * m_vOffsetPlaneNorm;  // Bin is right hand side of curve
    SmVector3d sUBin( sBin );
    SER( sUBin.Unitize() );
    aPointAndDerivatives[0] = sPV[0] + sUBin * m_dOffsetDistance;

    if ( lNumToCalculate < 1) 
      {
        return SM_SUCCESS;
      }

    // Derivatives: call only one of the UnitizedDerivative methods,
    // otherwise there's a lot of repeated calculation.
    SmVector3d  sBin_t,  sBin_tt,  sBin_ttt;
    SmVector3d sUBin_t, sUBin_tt, sUBin_ttt;

    if ( lNumToCalculate == 1 )
      {
        sBin_t = sPV[2] * m_vOffsetPlaneNorm;
        sUBin_t = sBin.UnitizedDerivative( sBin_t );
      }
    else if ( lNumToCalculate == 2 )
      {
        sBin_t  = sPV[2] * m_vOffsetPlaneNorm;
        sBin_tt = sPV[3] * m_vOffsetPlaneNorm;
        SER( sBin.UnitizedDerivative2( sBin_t, sBin_tt, sUBin_t, sUBin_tt ) );
      }
    else if ( lNumToCalculate == 3 )
      {
        sBin_t   = sPV[2] * m_vOffsetPlaneNorm;
        sBin_tt  = sPV[3] * m_vOffsetPlaneNorm;
        sBin_ttt = sPV[4] * m_vOffsetPlaneNorm;
        SER( sBin.UnitizedDerivative3( sBin_t, sBin_tt, sBin_ttt, sUBin_t, sUBin_tt, sUBin_ttt ) );
      }

    // 1st deriv.
    aPointAndDerivatives[1] = sPV[1] + m_dOffsetDistance * sUBin_t;
    if ( m_bIsReversed )
        { aPointAndDerivatives[1] = -aPointAndDerivatives[1]; }

    // 2nd deriv.
    if ( lNumToCalculate >= 2 )
      {
        aPointAndDerivatives[2] = sPV[2] + m_dOffsetDistance * sUBin_tt;
      }

    // 3rd deriv.
    if ( lNumToCalculate >= 3 )
      {
        aPointAndDerivatives[3] = sPV[3] + m_dOffsetDistance * sUBin_ttt;
        if ( m_bIsReversed )
            { aPointAndDerivatives[3] = -aPointAndDerivatives[3]; }
      }

  // When asked, try to find a direction for zero-length first derivatives based on the
  //  2nd derivative value
  // (This is the same trick as is used for surface evaluations.)
  if(bNonZeroTangents)
    {
      double dScaledZero = SM_EFF_ZERO * (1.0 + aPointAndDerivatives[0].GetMaxDimension());
      double dNewLen     = 1.1 * dScaledZero ;

      // see if the 1st order derivative is smaller than dNewLen (this gives a continuous modified function)
      if(   lNumDerivatives >= 1
         && aPointAndDerivatives[1].LengthSquared() < dNewLen * dNewLen)
        {
          // make sure 2nd order derivatives are available
          if(lNumDerivatives < 2)
            { 
              // recurse back to this function so the next section sets the 1st derivative
              SmVector3d sVals[3] ;
              Evaluate(dParameter, 2, bFromLeft, sVals) ;

              // save the 1st derivative
              aPointAndDerivatives[1] = sVals[1] ;
            }
          else // fix the zero 1st derivative function here
            {
              double dLen  = aPointAndDerivatives[2].Length();
              if(dLen > dNewLen)
                {
                  aPointAndDerivatives[1] = (dNewLen/dLen) * aPointAndDerivatives[2] ;

                  // If we're at the 'top' of the domain, i.e., the 'good' parameter is
                  // entering a singularity instead of leaving it, then the degenerate
                  // derivative is shrinking, so its change (the 2nd derivative) is in the opposite
                  // direction of the derivative.
                  if(dParameter > GetNaturalInterval().GetMid() )
                    {
                      aPointAndDerivatives[1] *= -1;
                    }
                } // end 2nd derivative is non-zero check
            } // end fix the zero 1st derivative function here branch
        } // end if the 1st derivative is zero check
    } // if fixing zero 1st derivative direction with 2nd derivative values check

    return SM_SUCCESS;

} // end Evalute

/*******************************************************************//**
PURPOSE: Given the parametric value of the curve determine the
    corresponding Euclidian point on the curve.      

NOTES: This methods assumes the curve is not discontinuous
    in its position.  The parameter must be within the natural range
    of the curve.
***********************************************************************/
SmStatus SmOffsetCurve::EvaluatePoint
  (double dInputParameter, 
   SmPoint3d & rPoint) 
 const
{ 
    // Adjust parameter if reversed: mirror within the interval.
    double dParameter = dInputParameter;
    if ( m_bIsReversed )
    {
        dParameter =   m_vCurveInterval.GetMin()
                     + m_vCurveInterval.GetMax() - dInputParameter;
    }

    if ( smos_Fabs( m_dOffsetDistance ) < SM_EFF_ZERO )
    {
        SER( m_pCurve->EvaluatePoint( dParameter, rPoint ));
        return SM_SUCCESS;
    }
    SmPoint3d sPV[2];
    SmBoolean bFromLeft = TRUE;
    if ( SM_IS_ZERO( dParameter - m_vCurveInterval.GetMax() )) bFromLeft = FALSE;
    SER( m_pCurve->Evaluate( dParameter, 1, bFromLeft, sPV ));
    if ( sPV[1].LengthSquared() < SM_EFF_ZERO_SQ )
    {
        double dSign = 1.0;
        if ( dParameter > m_vCurveInterval.GetMax() - 
                SM_EFF_ZERO_SQRT * m_vCurveInterval.GetLength() )
        {
            dSign = -1.0;
            dParameter = dParameter - (SM_EFF_ZERO_SQRT * m_vCurveInterval.GetLength());
        }
        else {
            dParameter = dParameter + (SM_EFF_ZERO_SQRT * m_vCurveInterval.GetLength());
        }
        SmPoint3d sPV2[2];
        SER( m_pCurve->Evaluate( dParameter, 1, bFromLeft, sPV2 ));
        sPV[1] = (sPV2[0] - sPV[0]) / (dSign * SM_EFF_ZERO_SQRT * m_vCurveInterval.GetLength());
        if (sPV[1].LengthSquared() < SM_EFF_ZERO_SQ) {
            SER(SM_ERR);
        }
    }

    // Here we have a usable first derivative.

    SmVector3d sTan = sPV[1];
    SmVector3d sBin = sTan * m_vOffsetPlaneNorm;  // Bin is right hand side of curve
    if (sBin.LengthSquared() < SM_EFF_ZERO_SQ) SER(SM_ERR);
    sBin.Unitize();
    rPoint = sPV[0] + sBin * m_dOffsetDistance;

    return SM_SUCCESS;

} // end SmOffsetCurve::EvaluatePoint

/*******************************************************************//**
PURPOSE: Get list of unique knots and optionally knot multiplicities 
     of a curve.  Just get the knots of the underlying generator curve.

NOTES:
   - This is the STEP compatible form of the knots not the
     typical knot vector associated with NURBS.
   - Just returns the base curve's knots.
***********************************************************************/
SmStatus SmOffsetCurve::GetKnots
  (SmTArray<double> & rKnots, 
   SmTArray<ULONG>  * pKnotMultiplicities,
   const SmExtent1d * pOptIvl)              // in : interval of interest, NULL=Natural Interval, default:[NULL] 
  const
{
    SER(m_pCurve->GetKnots(rKnots,pKnotMultiplicities,pOptIvl));
    return SM_SUCCESS;

} // end SmOffsetCurve::GetKnots

/*******************************************************************//**
PURPOSE: Get the number of natural knots.

NOTES:
   - The number of natural knots is the number of knots if the knots are
     represented as a single array with duplicated knot values.
   - Just returns the base curve's value.
***********************************************************************/
ULONG SmOffsetCurve::GetNumberNaturalKnots()
  const
{
    return m_pCurve->GetNumberNaturalKnots();

} // end SmOffsetCurve::GetKnots

/*******************************************************************//**
PURPOSE: Reverse the parameterization of this offset curve.

NOTES:
   This does not change the offset direction.
   The new Interval must not exceed the base curve's domain.

METHOD --- Simply adjust our local reversal flag.
***********************************************************************/
SmStatus SmOffsetCurve::ReverseParameterization
 (const SmExtent1d & crOldInterval,
        SmExtent1d & rNewInterval)
{
    // Calculate the return argument.
    // Note, if the input interval equals our interval,
    // then the output interval will be the same as well.
    double dOldTMin, dOldTMax;
    m_vCurveInterval.Inversion( crOldInterval.GetMin(), dOldTMin );
    m_vCurveInterval.Inversion( crOldInterval.GetMax(), dOldTMax );

    double dNewTMin = m_vCurveInterval.Evaluate( 1 - dOldTMax );
    double dNewTMax = m_vCurveInterval.Evaluate( 1 - dOldTMin );

    rNewInterval.SetMinMax( dNewTMin, dNewTMax );

    // Do the reversal.
    Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);
    m_bIsReversed = !m_bIsReversed;
    Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);

    return SM_SUCCESS;

} // end SmOffsetCurve::ReverseParameterization

/*******************************************************************//**
PURPOSE: Trim this curve to a given interval.

NOTES: The new Interval must not exceed the base curve's domain.

METHOD --- Simply adjust our local interval.
***********************************************************************/
SmStatus SmOffsetCurve::Trim
 (SmExtent1d & rTrimInterval,    // in: Desired new interval - this virtual method does not snap TrimIvl
  SmBoolean    bNotify,          // in : internal use only - use default, default:[TRUE]
                                 //      TRUE  = call Notify after trimming (previous behavior)
                                 //      FALSE = skip Notify after trimming
                                 //      UNSURE= skip notify, skip trimming, just recompute TrimInterval
  SmBoolean    bSkipDebugCheck)  // NotUsed: in : internal use only - use default, default:[FALSE]
                                 //      FALSE= in debug mode silently run this->AssertValid()
                                 //      TRUE = don't run AssertValid() before returning
{
  SM_REF1(bSkipDebugCheck) ; 
    // Check compatibility with base curve.
    SmExtent1d sBaseIvl = m_pCurve->GetNaturalInterval();
    if ( ! m_vCurveInterval.IsContainedBy( sBaseIvl, SM_EFF_ZERO ) )
      { return SM_ERR; }

    // Trim when asked
    if( bNotify != UNSURE )
      {
        // ready the world
        Notify(SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL);

        // And trim:
        m_vCurveInterval = rTrimInterval;
      }

    // inform the public - when asked
    if ( bNotify == TRUE )
      {
        Notify(SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL);
      }

    return SM_SUCCESS;

} // end SmOffsetCurve::Trim

/*******************************************************************//**
PURPOSE: Reset the offset distance.

NOTES: We also have to reset the cache: Notify().
***********************************************************************/
void SmOffsetCurve::SetOffsetDistance(double dOffsetDistance)
{
  // prepare the object
  Notify( SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER(this), NULL );

  m_dOffsetDistance = dOffsetDistance;
    
  // inform the object
  Notify( SM_NO_POST_EDIT, this, SM_NO_GET_OWNER(this), NULL );
}

/*******************************************************************//**
PURPOSE: Write SmOffsetCurve to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmOffsetCurve::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // in : database version to get proper sequence of writes                                                                       
 const
{
  // file type
  SmFileType      eType    = rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();
      
  if (eType == SM_ASCII) 
    {
      rFileOut << m_vOffsetPlaneNorm.x << " " << m_vOffsetPlaneNorm.y << " "    << m_vOffsetPlaneNorm.z << " SmOffsetCurve Offset Plane Normal \n";
      rFileOut << m_dOffsetDistance                                             << " SmOffsetCurve Offset Distance \n";
      rFileOut << m_vCurveInterval.GetMin() << " " << m_vCurveInterval.GetMax() << " SmOffsetCurve Offset Curve Interval \n";
      rFileOut << m_bIsReversed                                                 << " SmOffsetCurve IsReversed \n";
    }
  else 
    {
      SER(rDB.WriteDouble(m_vOffsetPlaneNorm.x));
      SER(rDB.WriteDouble(m_vOffsetPlaneNorm.y));
      SER(rDB.WriteDouble(m_vOffsetPlaneNorm.z));
      SER(rDB.WriteDouble(m_dOffsetDistance));
      SER(rDB.WriteDouble(m_vCurveInterval.GetMin()));
      SER(rDB.WriteDouble(m_vCurveInterval.GetMax()));
      SER(rDB.WriteBoolean(m_bIsReversed));
    }

  // base curve
  ULONG lBaseDim = m_pCurve->GetDim() ;
  if (eType == SM_ASCII) { rFileOut << " OffsetCurve->BaseCurve \n"; }
  SER(rDB.WriteType(m_pCurve->GetType(), &lBaseDim)) ;
  SER(m_pCurve->WriteToDB(rDB, lDBVersionNumber)) ;

  // all done
  return SM_SUCCESS ;

} // end SmOffsetCurve::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmOffsetCurve from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmOffsetCurve::ReadFromDB
 (SM_TYPE           lType,              // NotUsed: in : curve type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  ULONG             lDim,               // NotUsed: in : curve image space dim, 2 or 3                                                                                    
  const SmContext & crContext,          // in : context for new object construction
  SmCurve        *& rpNewCurve,         // out:    NULL on input = new object allocated in this routine built from stream data
                                        //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG             lDBVersionNumber)   // in : database version to get proper sequence of writes
{
  SM_REF2(lType, lDim) ; 
  // check input
  SER(  (   rpNewCurve == NULL
         || rpNewCurve->IsKindOf(SmOffsetCurve_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmOffsetCurve *pOffsetCurve =   (rpNewCurve == NULL)
                            ? new (crContext) SmOffsetCurve()
                            : (SmOffsetCurve *)rpNewCurve ;

  // file type
  SmFileType     eType   = rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();
      
  // locals
  SmVector3d sOffsetPlaneNormal ;
  double     dOffsetDistance = 0.0;
  double     dCurveIntervalMin = 0.0, dCurveIntervalMax = 0.0;
  SmBoolean  bIsReversed ; 

  if (eType == SM_ASCII) 
    {
      rFileIn >> sOffsetPlaneNormal.x >> sOffsetPlaneNormal.y >> sOffsetPlaneNormal.z ;  rDB.GoToNextLine() ;
      rFileIn >> dOffsetDistance ;                                                       rDB.GoToNextLine() ;
      rFileIn >> dCurveIntervalMin >> dCurveIntervalMax ;                                rDB.GoToNextLine() ;
      rFileIn >> bIsReversed ;                                                           rDB.GoToNextLine() ;
    }
  else 
    {
      SER(rDB.ReadDouble(sOffsetPlaneNormal.x));
      SER(rDB.ReadDouble(sOffsetPlaneNormal.y));
      SER(rDB.ReadDouble(sOffsetPlaneNormal.z));
      SER(rDB.ReadDouble(dOffsetDistance));
      SER(rDB.ReadDouble(dCurveIntervalMin));
      SER(rDB.ReadDouble(dCurveIntervalMax));
      SER(rDB.ReadBoolean(bIsReversed));
    }

  // curve locals
  SmCurve *pBaseCurve=NULL ;
  SM_TYPE  lBaseType ;  
  ULONG    lBaseDim ;

  // base curve
  if (eType == SM_ASCII) { rDB.GoToNextLine() ; }
  SER(rDB.ReadType(lBaseType, &lBaseDim)) ; 
  SER(SmCurve::ReadFromDB(lBaseType, rDB, lBaseDim, crContext, pBaseCurve, lDBVersionNumber)) ; 

  // load the obj
  pOffsetCurve->m_pCurve            = pBaseCurve ;         
  pOffsetCurve->m_pCurve->SetOwner(pOffsetCurve) ;
  pOffsetCurve->m_vOffsetPlaneNorm  = sOffsetPlaneNormal ;
  pOffsetCurve->m_dOffsetDistance   = dOffsetDistance ;
  pOffsetCurve->m_vCurveInterval.SetMinMax(dCurveIntervalMin, dCurveIntervalMax) ;   
  pOffsetCurve->m_bIsReversed       = bIsReversed ;     
  pOffsetCurve->m_bOwnsCurve        = TRUE ;

  // all done
  rpNewCurve = pOffsetCurve ;
  return SM_SUCCESS;

} // end SmOffsetCurve::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmOffsetCurve::IsKindOf( SM_TYPE t ) const
{
  return ((SmOffsetCurve_TYPE == t) ? TRUE : SmCurve::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmOffsetCurve::Dump
  ()
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_WriteBuffer(_T("\nBegin SmOffsetCurve::Dump()")) ;

  // Output name and pointer, and cache data
  SmCurve::Dump( FALSE );

  // output representations
  smos_sprintf( sBuff, _T("\n   Offset Dist   = %16.16lf\n   Domain        = "), m_dOffsetDistance );
  smos_WriteBuffer( sBuff, sBuff );
  m_vCurveInterval.Dump();

  smos_sprintf( sBuff, _T("%s"), _T("\n   Offset Normal = ") );
  smos_WriteBuffer( sBuff, sBuff );
  m_vOffsetPlaneNorm.Dump();

  smos_sprintf( sBuff, _T("\n   Reversed = %s"), m_bIsReversed ? _T("TRUE") : _T("FALSE") );
  smos_WriteBuffer( sBuff, sBuff );

  smos_sprintf( sBuff, _T( "%s" ), _T("\n   Dump of Base Curve:\n") );
  smos_WriteBuffer( sBuff, sBuff );
  m_pCurve->Dump();

  smos_WriteBuffer(_T(" End SmOffsetCurve::Dump()\n")) ;

} // end SmOffsetCurve::Dump

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertOffsetCurve_list[] =
{
/*  0 */ {SM_AT_POINTER,          _T("Context"),             _T("m_pSurface shares the same context") },
/*  1 */ {SM_AT_POINTER,          _T("Bad Curve Owner"),   _T("Base Curve owner must be this SmOffsetCurve object") }
} ;

/*******************************************************************//**
PURPOSE: Make sure Base Curve and Surface parameterizations are
            compatible.

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmOffsetCurve::AssertValid
(SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
    SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                      //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                      //      default:[SM_LEVEL_0]
    SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
    SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
    const
{
  SM_REF1(eWalkTree) ; 
    SmBoolean bRtn = TRUE;

    // call the base class AssertValid
    bRtn &= ((eTestLevel != SM_LEVEL_GIVEN)
        ? SmCurve::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests)
        : TRUE);

    // all contained pointers to crContext should be the same
    /*  0 */ // m_pSurface shares the same context
    bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_pCurve == NULL || m_cpContext == m_pCurve->GetContext()), _T(""));

    /*  1 */ // Bad m_pSurface Owner
    bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, m_pCurve->GetOwner() == this, _T(""));

#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    // draw
    if (bDebugMe)
    {

    }
#endif // SM_DEBUG_CODE

    // all done
    // SM_ASSERT(bRtn) ;

  return(bRtn);

} // end SmOffsetCurve::AssertValid
