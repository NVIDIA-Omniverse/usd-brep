// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmConic.cpp
* PURPOSE: Source file for SmConic methods.
**********************************************************************/

  
#include "StdAfx.h"
#include <SmConic.h>
#include <SmPolynomial.h>

/*******************************************************************//**
PURPOSE: Equality operator for SmConic

NOTES: Call base equivalence to check type and then check 
       members for equivalence
***********************************************************************/
SmBoolean SmConic::operator==
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
      SmConic &rOther = (SmConic &)crOther ;

      // check equivalence of these objects
      bRtn = (   m_eConicType  == rOther.m_eConicType
              && SM_IS_ZERO(m_dA - rOther.m_dA )
              && SM_IS_ZERO(m_dB - rOther.m_dB )
              && m_vOrigin     == rOther.m_vOrigin   
              && SM_IS_ZERO(m_dThetaRad - rOther.m_dThetaRad)) ;
    }

  // all done
  return bRtn ;

} // end SmConic::operator==

/*******************************************************************//**
PURPOSE: Calculate the minimum continuity of the curve and the 
    continuities at each of the unique knots
    
NOTES: Conics are continuous throughout.  Return array 
***********************************************************************/
SmStatus SmConic::CalculateContinuities
  (SmContinuityType           & reMinContinuityInCurve, // out:
   SmTArray<SmContinuityType> & rContinuitiesAtKnots,   // out:
   double                       dContinuityAngleTol)    // NotUsed: in :
  const
{
  SM_REF1(dContinuityAngleTol) ;
  // init output
  rContinuitiesAtKnots.ReSet() ;

  // set output
  reMinContinuityInCurve = SM_CT_CINFINITY ;
  rContinuitiesAtKnots.Add( SM_CT_DISCONTINUOUS ) ;
  rContinuitiesAtKnots.Add( SM_CT_DISCONTINUOUS ) ;
   
  // all done
  return SM_SUCCESS;

} // end SmConic::CalculateContinuities

/*******************************************************************//**
PURPOSE: Get list of unique knots and optionally knot multiplicities 
     of a BSpline curve.

NOTES: This is an equivalent knot vector made of degree 3 entries for
 the curve domain
***********************************************************************/
SmStatus SmConic::GetKnots
  (SmTArray<double> & rKnots,               // out: Unique knot vector                     
   SmTArray<ULONG>  * pKnotMultiplicities,  // out: multiplicity value for each knot
   const SmExtent1d * pOptIvl)              // in : interval of interest, NULL=Natural Interval, default:[NULL]
  const
{
  // init output
  rKnots.ReSet() ;
  if(pKnotMultiplicities) { pKnotMultiplicities->ReSet() ; }

  // locals
  SmExtent1d sIvl = GetNaturalInterval() ;

  // interval start
  if(pOptIvl == NULL || pOptIvl->ContainsValue(sIvl.GetMin()))
    {
      rKnots.Add(sIvl.GetMin()) ;
      if(pKnotMultiplicities)
        {
          pKnotMultiplicities->Add(3) ;
        }
    }

  // interval end
  if(pOptIvl == NULL || pOptIvl->ContainsValue(sIvl.GetMax()))
    {
      rKnots.Add(sIvl.GetMax()) ;
      if(pKnotMultiplicities)
        {
          pKnotMultiplicities->Add(3) ;
        }
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmConic::GetKnots

/*******************************************************************//**
PURPOSE: Given the parameteric value of the curve determine the
    corresponding Euclidian point and optionally derivatives.

NOTES: 
***********************************************************************/
SmStatus SmConic::Evaluate
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
    SER(EvaluatePoint(dParameter,aPointAndDerivatives[0]));
    double dTSq = dParameter * dParameter;
    double dT = dParameter;

    if (lNumDerivatives == 0) return SM_SUCCESS;

    SmVector3d sTan(0,0,0);
    SmVector3d sSec(0,0,0);

    switch (m_eConicType) {

    case SM_CT_ELLIPSE:
        {
        double dSign = 1.0;
        if (dParameter > 1.0) 
          {
            dT = dParameter - 2.0;
            dTSq = dT * dT;
            dSign = -1.0;
          }
        double dDenom = 1.0 + 2.0*dTSq + dTSq*dTSq;
        sTan.x = dSign * m_dA * ( -4.0 * dT ) / dDenom;
        sTan.y = dSign * m_dB * 2.0 * ( 1.0 - dTSq ) / dDenom;
        double dDenomSq = dDenom*dDenom;
        double dDenomPrim = (4.0*dT + 4.0*dT*dTSq);
        if (lNumDerivatives > 1) 
          {
            sSec.x = dSign * m_dA * (-4.0 * dDenom + 4.0 * dT * dDenomPrim) / dDenomSq;
            sSec.y = dSign * m_dB * (-4.0 * dT * dDenom - (2.0 - 2.0*dTSq)*dDenomPrim) / dDenomSq;
          }
        }
        break;


    case SM_CT_HYPERBOLA:
        {
        double dDenom = 1.0 - 2.0*dTSq + dTSq*dTSq;
        sTan.x = m_dA * ( 4.0 * dT ) / dDenom;
        sTan.y = m_dB * 2.0 * ( 1.0 + dTSq ) / dDenom;
        double dDenomSq = dDenom*dDenom;
        double dDenomPrim = (-4.0*dT + 4.0*dT*dTSq);
        if (lNumDerivatives > 1) 
          {
            sSec.x = m_dA * (4.0 * dDenom - 4.0 * dT * dDenomPrim) /  dDenomSq;
            sSec.y = m_dB * (4.0 * dT * dDenom - (2.0 + 2.0*dTSq) * dDenomPrim) / dDenomSq;
          }
        }
        break;

    case SM_CT_PARABOLA:
        sTan.x = 2.0 * m_dA * dT;
        sTan.y = 2.0 * m_dA;
        if (lNumDerivatives > 1) 
          {
            sSec.x = 2.0*m_dA;
            sSec.y = 0.0;
          }
        break;

    case SM_CT_LINE:
        sTan.x = 1.0;
        sTan.y = 0.0;
        if (lNumDerivatives > 1) 
          {
            sSec.x = 0.0;
            sSec.y = 0.0;
          }
        break;
    case SM_CT_UNKNOWN:
        break;
    }
 
    if (m_dThetaRad != 0.0) 
      {
        SmVector3d sVec(sTan.x*m_dCosTheta + sTan.y*m_dSinTheta,
                        sTan.y*m_dCosTheta - sTan.x*m_dSinTheta,
                        0.0);
        sTan = sVec;
        if (lNumDerivatives > 1) 
          {
            SmVector3d sVec2(sSec.x*m_dCosTheta + sSec.y*m_dSinTheta,
                             sSec.y*m_dCosTheta - sSec.x*m_dSinTheta,
                             0.0);
            sSec = sVec2;
          }

      }

    aPointAndDerivatives[1] = sTan;

    if (lNumDerivatives == 1) return SM_SUCCESS;

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

    aPointAndDerivatives[2] = sSec;

    if (lNumDerivatives == 2) return SM_SUCCESS;

    SE(SM_ERR);
    return SM_ERR;

} // end SmConic::Evaluate

/*******************************************************************//**
PURPOSE: Given the parametric value of the curve determine the
    corresponding Euclidian point on the curve.      

NOTES: This methods assumes the curve is not discontinuous
    in its position.  The parameter must be within the natural range
    of the curve.
***********************************************************************/
SmStatus SmConic::EvaluatePoint
  (double dParameter, 
   SmPoint3d & rPoint) 
 const
{ 
    rPoint.z = 0.0;
    double dTSq = dParameter * dParameter;
    double dT = dParameter;

    switch (m_eConicType) 
      {

        case SM_CT_ELLIPSE:
            {
            double dSign = 1.0;
            if (dParameter > 1.0) 
              {
                dT = dParameter - 2.0;
                dTSq = dT * dT;
                dSign = -1.0;
              }
            double dDenom = 1.0 + dTSq;
            rPoint.x = dSign * m_dA * ( 1.0 - dTSq ) / dDenom;
            rPoint.y = dSign * m_dB * ( 2.0 * dT ) / dDenom;
            }
            break;


        case SM_CT_HYPERBOLA:
            {
            double dDenom = 1.0 - dTSq;
            rPoint.x = m_dA * ( 1.0 + dTSq ) / dDenom;
            rPoint.y = m_dB * ( 2.0 * dT ) / dDenom;
            }
            break;

        case SM_CT_PARABOLA:
            rPoint.x = m_dA * dTSq;
            rPoint.y = 2.0 * m_dA * dT;
            break;

        case SM_CT_LINE:
            rPoint.x = dT;
            rPoint.y = m_dA;
            break;
        case SM_CT_UNKNOWN:
            break;
      }
 
    if (m_dThetaRad != 0.0) 
      {
        SmPoint3d sPnt(rPoint.x*m_dCosTheta + rPoint.y*m_dSinTheta,
                       rPoint.y*m_dCosTheta - rPoint.x*m_dSinTheta,
                       0.0);
        rPoint = sPnt;
      }
    if (m_vOrigin.x != 0.0 || m_vOrigin.y != 0.0) 
      {
        rPoint = rPoint + m_vOrigin;
      }

    return SM_SUCCESS;

} // end SmConic::EvaluatePoint

/*******************************************************************//**
PURPOSE: Compute the implicit equation for the given conic.

NOTES: 
***********************************************************************/
SmStatus SmConic::ComputeImplicitEquation
  (double adConicCoeff[6]) 
 const
{
    double A=0.0,B=0.0,C=0.0,D=0.0,E=0.0,F=0.0;

    switch (m_eConicType) {
    case SM_CT_ELLIPSE:
        {
           A = 1.0 / (m_dA*m_dA);
           B = 0.0;
           C = 1.0 / (m_dB*m_dB);
           D = 0.0;
           E = 0.0;
           F = -1.0;
        }
        break;
        
    case SM_CT_HYPERBOLA:
        {
           A = 1.0 / (m_dA*m_dA);
           B = 0.0;
           C = -1.0 / (m_dB*m_dB);
           D = 0.0;
           E = 0.0;
           F = -1.0;
        }
        break;

    case SM_CT_PARABOLA:
        {
           A = 0.0;
           B = 0.0;
           C = 1.0;
           D = -4.0 * m_dA;
           E = 0.0;
           F = 0.0;
        }
        break;

    case SM_CT_LINE:
        {
            A = 0.0;
            B = 0.0;
            C = 0.0;
            D = 0.0;
            E = -1.0/m_dA;
            F = 1.0;
        }
        break;
     case SM_CT_UNKNOWN:
        break;

    }

    // Now do rotation - plug counter clockwise rotation into
    // implicit equation.
    //
    // x' = x*Cos + y*Sin
    // y' = -x*Sin + Y*Cos
    //   2           2
    // Ax  + Bxy + Cy  + Dx + Ey + F = 0
    //
    //
    double dSin = smos_Sine(-m_dThetaRad);
    double dCos = smos_Cosine(-m_dThetaRad);
    double dSinSq = dSin*dSin;
    double dCosSq = dCos*dCos;
    double dSinCos = dSin*dCos;

    double A_2 = A*dCosSq - B*dSinCos + C*dSinSq;
    double B_2 = 2.0*A*dSinCos + B*dCosSq - B*dSinSq - 2.0*C*dSinCos;
    double C_2 = A*dSinSq + B*dSinCos + C*dCosSq;
    double D_2 = D*dCos - E*dSin;
    double E_2 = D*dSin + E*dCos;
    double F_2 = F;

    // Here is where we do tranlation.
    double c1 = -m_vOrigin.x;
    double c2 = -m_vOrigin.y;

    adConicCoeff[0] = A_2;
    adConicCoeff[1] = B_2;
    adConicCoeff[2] = C_2;
    adConicCoeff[3] = 2.0*A_2*c1 + B_2*c2 + D_2;
    adConicCoeff[4] = B_2*c1 + 2.0*C_2*c2 + E_2;
    adConicCoeff[5] = A_2*c1*c1 + B_2*c1*c2 + C_2*c2*c2 + D_2*c1 + E_2*c2 + F_2;

    return SM_SUCCESS;

} // end SmConic::ComputeImplicitEquation



/*******************************************************************//**
PURPOSE: Intersect this conic with another conic given in implicit form.

NOTES: Right now we assume that the parametric conic is in natural
    position.  
***********************************************************************/
SmStatus SmConic::IntersectImplicitConic
  (double adConicCoeff[6],
   SmTArray<double> & rIntersectionResults,
   SmTArray<ULONG> & )                      // rIntersectionMultiplicity
 const
{
    double A = adConicCoeff[0];  // x^2 component
    double B = adConicCoeff[1];  // xy component
    double C = adConicCoeff[2];  // y^2 component
    double D = adConicCoeff[3];  // x component
    double E = adConicCoeff[4];  // y component
    double F = adConicCoeff[5];  // constant

    // Run through two times.  First for the right hand side and then
    // for the left hand side of the Ellipse. 
    for (ULONG lSide=0; lSide<=1; lSide++) {
        // Only do it two times for ellipse.
        if (lSide == 1 && m_eConicType != SM_CT_ELLIPSE) break;
        // Plug the Implicit equ. into the Parametric and solve for
        // values of T.
        double dSign = 1.0;
        if (lSide == 1) dSign = -1.0;
        double adQCoeff[5];
        double a = m_dA * dSign;
        double a_sq = a * a;
        double b = m_dB * dSign;
        double b_sq = b * b;
        if (m_eConicType == SM_CT_ELLIPSE) {
            adQCoeff[0] = A*a_sq + D*a + F;
            adQCoeff[1] = 2.0 * (B*a*b + E*b);
            adQCoeff[2] = 2.0 * (-A*a_sq + 2*C*b_sq + F);
            adQCoeff[3] = 2.0 * (-B*a*b + E*b);
            adQCoeff[4] = A*a_sq - D*a + F;
        }
        else if (m_eConicType == SM_CT_HYPERBOLA) {
            adQCoeff[0] = A*a_sq + D*a + F;
            adQCoeff[1] = 2.0 * (B*a*b + E*b);
            adQCoeff[2] = 2.0 * (A*a_sq + 2*C*b_sq - F);
            adQCoeff[3] = 2.0 * (B*a*b - E*b);
            adQCoeff[4] = A*a_sq - D*a + F;
        }
        else if (m_eConicType == SM_CT_PARABOLA) {
            adQCoeff[0] = A*a_sq;
            adQCoeff[1] = 2.0 * (B*a_sq);
            adQCoeff[2] = (4.0*C*a_sq + D*a);
            adQCoeff[3] = 2.0*E*a;
            adQCoeff[4] = F;
        }
        else if (m_eConicType == SM_CT_LINE) {
            adQCoeff[0] = C*a_sq + E*a + F;
            adQCoeff[1] = B*a + D;
            adQCoeff[2] = A;
            adQCoeff[3] = 0.0;
            adQCoeff[4] = 0.0;
        }
        ULONG lNumQSols;
        double adQSols[4];
        SER(SmPolynomial::SolveQuarticEqn(adQCoeff,SM_EFF_ZERO,lNumQSols,adQSols));
        for (ULONG i=0; i<lNumQSols; i++) {
            if (m_eConicType == SM_CT_ELLIPSE) {
                if (smos_Fabs(adQSols[i]) <= 1.0+SM_EFF_ZERO) {
                    if (dSign < 0.0) { 
                        if (smos_Fabs(adQSols[i]) < 1.0) {
                            adQSols[i] = 2.0 + adQSols[i];
                            rIntersectionResults.Add(adQSols[i]);
                        }
                    }
                    else rIntersectionResults.Add(adQSols[i]);
                }
            }
            else if (m_eConicType == SM_CT_HYPERBOLA) {
                // Make case where hyperbola's intersect at their axes
                // at infinity work.
                SmExtent1d sCrvIvl = GetNaturalInterval();
                if (smos_Fabs(adQSols[i]) < 1.0+SM_EFF_ZERO_SQRT) {
                    adQSols[i] = sCrvIvl.ClampValue(adQSols[i]);
                }
                if (sCrvIvl.ContainsValue(adQSols[i])) {
                    rIntersectionResults.Add(adQSols[i]);
                }
            }
            else {
                SmExtent1d sCrvIvl = GetNaturalInterval();
                if (sCrvIvl.ContainsValue(adQSols[i])) {
                    rIntersectionResults.Add(adQSols[i]);
                }
            }
        }
    } // For each side of the ellipse.

    return SM_SUCCESS;

} // end SmConic::IntersectImplicitConic


/*******************************************************************//**
PURPOSE: Intersect two 2D conics using direct analytical methods.

NOTES: As with most analytical intersection techniques,
   this method will only find actual intersections.  It will not find
   near intersections.  Note that right now things do not work well
   for cases where the 'this' conic is transformed.
***********************************************************************/
SmStatus SmConic::IntersectConic
  (const SmExtent1d & crInterval,
   const SmConic & crOtherConic,
   const SmExtent1d & crOtherInterval,
   double dDistanceTolerance,
   SmSolutionArray & rSolutions) 
 const
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,0);
        DrawWDeriv(crInterval,0);
        smgfx_SetColor(0,0,1);
        crOtherConic.DrawWDeriv(crOtherInterval,0);
        sm_GraphicsLoop();
    }
#else
    SM_REF2(crInterval, crOtherInterval);
#endif

    rSolutions.ReSet();
    double adCoeff[6];
    SmConic sTmpConic(crOtherConic.m_eConicType,crOtherConic.m_dA,
        crOtherConic.m_dB);
    sTmpConic.Transform2D(-crOtherConic.m_dThetaRad,crOtherConic.m_vOrigin);
    sTmpConic.Transform2D(0.0,SmPoint2d(-m_vOrigin));
    sTmpConic.Transform2D(m_dThetaRad,SmPoint2d(0,0));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2=FALSE;
    if (bDebugMe2) {
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,0);
        sTmpConic.DrawWDeriv(crOtherInterval,0);
        smgfx_SetColor(0,0,1);
        SmConic sTmpConic2(m_eConicType,m_dA,m_dB);
        sTmpConic2.DrawWDeriv(crInterval,0);
        sm_GraphicsLoop();
    }
#endif

    // Handle special case of two hyperbolas which have coincident 
    // origins and have a 90 degree difference in their orientation
    // We need to look for intersections along the axes.
#if 0
    if ( sTmpConic.m_eConicType == SM_CT_HYPERBOLA && 
         m_eConicType           == SM_CT_HYPERBOLA &&
         smos_Fabs(smos_Fabs(sTmpConic.m_dThetaRad)-SM_PI/2.0) < SM_EFF_ZERO_SQRT) 
      {
        SmPoint3d sPnt1, sPnt2;
        SmExtent1d sIvl1 = GetNaturalInterval();
        SmExtent1d sIvl2 = GetNaturalInterval();
        double dT1 = sIvl1.GetMin();
        double dT2 = sIvl2.GetMax();
        if (sTmpConic.m_dThetaRad < 0.0) 
          {  // 90 degree clockwise rotation
            dT1 = sIvl1.GetMax();
            dT2 = sIvl2.GetMin();
          }
        SER(EvaluatePoint(dT1,sPnt1));
        SER(crOtherConic.EvaluatePoint(dT2,sPnt2));
        double dScale = dDistanceTolerance * (1.0 + sPnt1.GetMaxDimension());
        double dDist = sPnt1.DistanceBetween(sPnt2);
        if (dDist < dScale) 
          {
            SmSolution sSol;
            sSol.m_vStart[0] = dT1;
            sSol.m_vStart[1] = dT2;
            sSol.m_vStart.m_dSolutionValue = dDist;
            rSolutions.Add(sSol);
            return SM_SUCCESS;
          }
      }
#endif // if 0 unused code

    SER(sTmpConic.ComputeImplicitEquation(adCoeff));
    SmTArray<double> sTsectParams;
    SmTArray<ULONG> sMultiplicities;
    SER(IntersectImplicitConic(adCoeff,sTsectParams,sMultiplicities));
    // Need to invert points and then determine multiplicities by
    // comparing tangent vectors.
    for (ULONG i=0; i<sTsectParams.GetSize(); i++) 
      {
        double dParam = sTsectParams[i];
        SmSolution sSol;
        sSol.m_vStart[0] = dParam;
        SmBoolean bSuccess;
        SmPoint3d sPnt;
        SER(EvaluatePoint(dParam,sPnt));
        double dParam2;
#ifdef SM_DEBUG_CODE
        if (bDebugMe) 
          {
            SmConic sTmpConic2(m_eConicType,m_dA,m_dB);
            SmPoint3d sPnt2;
            SER(sTmpConic2.EvaluatePoint(dParam,sPnt2));
            sm_GraphicsLoop();
            smgfx_SetColor(1,1,0);
            sPnt2.Draw();
            smgfx_SetColor(1,0,0);
            sPnt.Draw();
            sm_GraphicsLoop();
          }
#endif
        SER(crOtherConic.InvertPointOnConic(sPnt,bSuccess,dParam2));
        if (!bSuccess) continue;
        SmPoint3d sPnt2;
        SER(crOtherConic.EvaluatePoint(dParam2,sPnt2));
        double dDist;
        if ((dDist = sPnt2.DistanceBetween(sPnt)) > dDistanceTolerance) 
          {
            continue;
          }
        sSol.m_vStart[1] = dParam2;
        sSol.m_vStart.m_dSolutionValue = dDist;

        rSolutions.Add(sSol);

      }

    return SM_SUCCESS;

} // end SmConic::IntersectConic




/*******************************************************************//**
PURPOSE: Given a point which is know to lie on the conic, determine
    the corresponding parameter.

NOTES:  
***********************************************************************/
SmStatus SmConic::InvertPointOnConic
  (const SmPoint3d & crPoint,
   SmBoolean & rbSuccess,
   double & rdParameter) 
  const
{
    // First move the point into the local parameter space of the
    // conic.
    rbSuccess = FALSE;
    double dParam =0.0;
    double dParam2=0.0;

    SmPoint3d sTmp = crPoint - m_vOrigin;
    double dCos = smos_Cosine(-m_dThetaRad);
    double dSin = smos_Sine(-m_dThetaRad);
    SmPoint2d sLocalPnt(sTmp.x*dCos + sTmp.y*dSin,
                        sTmp.y*dCos - sTmp.x*dSin);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        crPoint.Draw();
        sLocalPnt.Draw();
        sm_GraphicsLoop();
    }
#endif

    switch (m_eConicType) {

    case SM_CT_HYPERBOLA:
        {
//      double tTest = smos_Sqrt(  smos_Fabs(sLocalPnt.x-m_dA) 
//                               / smos_Fabs(sLocalPnt.x+m_dA) );
            
        double adQCoeff[3];
        adQCoeff[0] = -sLocalPnt.y;
        adQCoeff[1] = 2.0 * m_dB;
        adQCoeff[2] = sLocalPnt.y;
        double adQSol[2];
        ULONG lNumQSol;
        SER(SmPolynomial::SolveQuadraticEqn(adQCoeff,SM_EFF_ZERO,lNumQSol,adQSol));
        if (lNumQSol == 0) return SM_SUCCESS;
        SmExtent1d sIvl = GetNaturalInterval();
        if (smos_Fabs(adQSol[0]) <= 1.0+0.0001) {
            dParam = sIvl.ClampValue(adQSol[0]);
            if (lNumQSol > 1 && smos_Fabs(adQSol[1]) <= 1.0+0.0001) {
                dParam2 = sIvl.ClampValue(adQSol[1]);
            }
        }
        else if (lNumQSol > 1 && smos_Fabs(adQSol[1]) <= 1.0+0.0001) {
            dParam = sIvl.ClampValue(adQSol[1]);
        }
        else {
            return SM_SUCCESS;
        }
        }
        break;

    case SM_CT_ELLIPSE:
        {
        double adQCoeff[3];
        adQCoeff[0] = sLocalPnt.y;
        adQCoeff[1] = -2.0 * m_dB;
        adQCoeff[2] = sLocalPnt.y;
        double adQSol[2];
        ULONG lNumQSol;
        SER(SmPolynomial::SolveQuadraticEqn(adQCoeff,SM_EFF_ZERO,lNumQSol,adQSol));
        if (lNumQSol == 0) return SM_SUCCESS;
        if (smos_Fabs(adQSol[0]) <= 1.0+SM_EFF_ZERO) {
            dParam = adQSol[0];
        }
        else if (lNumQSol > 1 && smos_Fabs(adQSol[1]) <= 1.0+SM_EFF_ZERO) {
            dParam = adQSol[1];
        }
        else {
            return SM_SUCCESS;
        }
        // If point is on left side flip the parameter.
        if (sLocalPnt.x < 0.0) {
            dParam = 2 - dParam;
        }
        }
        break;

    case SM_CT_PARABOLA:
        dParam = sLocalPnt.y / (2.0 * m_dA);
        break;

    case SM_CT_LINE:
        dParam = sLocalPnt.x;
        break;
    case SM_CT_UNKNOWN:
        break;
    }

    SmPoint3d sTestPnt;
    SER(EvaluatePoint(dParam,sTestPnt));
    double dDiff = sTestPnt.DistanceBetween(crPoint);

#ifdef SM_DEBUG_CODE
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_SetColor(1,0,0);
        crPoint.Draw();
        smgfx_SetColor(0,0,1);
        sTestPnt.Draw();
        sm_GraphicsLoop();
    }
#endif

    double dScale = 1.0 + sTestPnt.GetMaxDimension();
    if (dDiff > SM_EFF_ZERO_SQRT * dScale) {
        if (m_eConicType == SM_CT_HYPERBOLA) {
            dParam = dParam2;
            SER(EvaluatePoint(dParam,sTestPnt));
            dDiff = sTestPnt.DistanceBetween(crPoint);
            if (dDiff > SM_EFF_ZERO_SQRT * dScale) {
                return SM_SUCCESS;
            }
        }
        else {
            return SM_SUCCESS;
        }
    }
    rbSuccess = TRUE;
    rdParameter = dParam;

    return SM_SUCCESS;

} // end SmConic::InvertPointOnConic



/*******************************************************************//**
PURPOSE: Perform a 2D transformation of a conic.  Apply rotation  
     and then the translation.

NOTES: 
***********************************************************************/
SmStatus SmConic::Transform2D
  (double dCCWRotationAngleRadians,
   const SmPoint2d & crTranslation)
{
    Notify( SM_NO_PRE_EDIT, this, SM_NO_GET_OWNER( this ), NULL );

    m_dThetaRad -= dCCWRotationAngleRadians;
    double dCos = smos_Cosine(-dCCWRotationAngleRadians);
    double dSin = smos_Sine(-dCCWRotationAngleRadians);
    SmPoint2d sTmp(m_vOrigin.x*dCos + m_vOrigin.y*dSin,
                   m_vOrigin.y*dCos - m_vOrigin.x*dSin);
    m_vOrigin = sTmp + crTranslation;
    m_dCosTheta = smos_Cosine(m_dThetaRad);
    m_dSinTheta = smos_Sine(m_dThetaRad);
    return SM_SUCCESS;

} // end SmConic::Transform2D

/*******************************************************************//**
PURPOSE: Write SmConic to given output stream.

NOTES: 
***********************************************************************/
SmStatus SmConic::WriteToDB
 (SmDatabaseIO & rDB,                // in : target output stream
  ULONG          lDBVersionNumber)   // NotUsed: in : database version to get proper sequence of writes                                                                       
 const
{
  SM_REF1(lDBVersionNumber) ;
  // file type, ASCII or BINARY
  SmFileType      eType    =  rDB.GetFileType();
  std::ostream  & rFileOut = *rDB.GetOutStreamPtr();

  // curve locals
  ULONG lConicType = m_eConicType ; 
  
  if (eType == SM_ASCII) 
    {
      rFileOut << lConicType                  << " SmConic conic type \n" ;
      rFileOut << m_dA                        << " SmConic A of conic eqn \n" ;
      rFileOut << m_dB                        << " SmConic B of conic eqn \n" ;
      rFileOut << m_vOrigin.x << m_vOrigin.y  << " SmConic origin of displaced conic curve \n" ;
      rFileOut << m_dThetaRad                 << " SmConic rotation from X in radians of displaced conic curve\n" ;

    }
  else 
    {
      SER(rDB.WriteLong  ( lConicType)) ;
      SER(rDB.WriteDouble( m_dA)) ;  
      SER(rDB.WriteDouble( m_dB)) ;  
      SER(rDB.WriteDouble( m_vOrigin.x)) ;  
      SER(rDB.WriteDouble( m_vOrigin.y)) ;  
      SER(rDB.WriteDouble( m_dThetaRad)) ;  
    }

  // all done
  return SM_SUCCESS;

} // end SmConic::WriteToDB

/*******************************************************************//**
PURPOSE: static method to Read a SmConic from a given stream  

NOTES: 
***********************************************************************/
SmStatus SmConic::ReadFromDB
 (SM_TYPE           lType,              // NotUsed: in : curve type to be read
  SmDatabaseIO    & rDB,                // in : target output stream
  ULONG             lDim,               // NotUsed: in : curve image space dim, 2 or 3                                                                                    
  const SmContext & crContext,          // in : context for new object construction
  SmCurve         *&rpNewCurve,         // out:    NULL on input = new object allocated in this routine built from stream data
                                        //      NotNULL on input = pointer to an empty object to be filled by this routine
  ULONG             lDBVersionNumber)   // NotUsed: in : database version to get proper sequence of writes
{
  SM_REF3(lType, lDim, lDBVersionNumber) ;
  // check input
  SER(  (   rpNewCurve == NULL
         || rpNewCurve->IsKindOf(SmConic_TYPE)) ? SM_SUCCESS : SM_ERR) ;

  // init output object
  SmConic *pConic =   (rpNewCurve == NULL)
                    ? new (crContext) SmConic()
                    : (SmConic *)rpNewCurve ;

  // file type
  SmFileType     eType   =  rDB.GetFileType();
  std::istream & rFileIn = *rDB.GetInStreamPtr();

  // locals
  ULONG lConicType ;
  double dA = 0.0, dB = 0.0, dOrigX = 0.0, dOrigY = 0.0, dThetaRad = 0.0;
      
  if (eType == SM_ASCII) 
    {
      rFileIn >> lConicType ;        rDB.GoToNextLine() ;         
      rFileIn >> dA ;                rDB.GoToNextLine() ;       
      rFileIn >> dB ;                rDB.GoToNextLine() ;       
      rFileIn >> dOrigX >> dOrigY ;  rDB.GoToNextLine() ;
      rFileIn >> dThetaRad ;         rDB.GoToNextLine() ;       
    }
  else 
    {
      SER(rDB.ReadLong  ( lConicType )) ;
      SER(rDB.ReadDouble( dA )) ;  
      SER(rDB.ReadDouble( dB )) ;  
      SER(rDB.ReadDouble( dOrigX )) ;  
      SER(rDB.ReadDouble( dOrigY )) ;  
      SER(rDB.ReadDouble( dThetaRad )) ;  
    }

  // load obj
  pConic->m_eConicType = (SmConicType) lConicType  ;
  pConic->m_dA         = dA          ;        
  pConic->m_dB         = dB          ;        
  pConic->m_vOrigin.Set(dOrigX, dOrigY) ;   
  pConic->m_dThetaRad  = dThetaRad   ;
  pConic->m_dSinTheta  = smos_Sine(dThetaRad) ;
  pConic->m_dCosTheta  = smos_Cosine(dThetaRad) ;

  // all done
  rpNewCurve = pConic ; 
  return SM_SUCCESS;

} // end SmConic::ReadFromDB

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmConic::IsKindOf( SM_TYPE t ) const
{
  return ((SmConic_TYPE == t) ? TRUE : SmCurve::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmConic::Dump
  () 
 const
{
  smos_WriteBuffer(_T("\nBegin SmConic::Dump()")) ;

  // output cache data
  SmCurve::Dump(FALSE) ;

  smos_WriteBuffer(_T(" End SmConic::Dump()\n")) ;

} // end SmConic::Dump
