// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmLocalSolve1d.cpp
* PURPOSE: Methods for the SmLocalSolve1d object.
**********************************************************************/

#include "StdAfx.h"

#include <SmLocalSolve1d.h>
#include <SmTArray.h>

#define ZEPS 1.0e-12
#define SMSIGN(a,b) ((b) >= 0.0 ? smos_Fabs(a) : -smos_Fabs(a));
#define MOV3(a,b,c, d,e,f) (a)=(d);(b)=(e);(c)=(f);

/**********************************************************************//**
PURPOSE: Determine whether the iteration has been oscillating between two values.

NOTES:
**********************************************************************/
SmBoolean SmIterQueue1d::IsOscillating( ULONG lStep )
{
  // For now, just check for step of two.
  if ( lStep != 2 ) { return FALSE; }

  // Require three cycles to declare oscillating.
  if ( m_lCount < 3 * lStep ) { return FALSE; }

  // Check all entries that we have.
  ULONG lTop = m_bFull ? m_lSize : m_lNextIdx;
  ULONG i;

  double dSum1=0, dSum2=0;
  for ( i = 0; i < lTop; i++ )
  {
      if ( i % lStep == 0 )
          { dSum1 += m_vArray[i].m_dPrevStep; }
      else
          { dSum2 += m_vArray[i].m_dPrevStep; }
  }

  double dSumAll = smos_Fabs( dSum1 + dSum2 );  // Near zero if oscillating.
  double dSumAbs = smos_Fabs( dSum1 ) + smos_Fabs( dSum2 );

  return ( dSumAll < dSumAbs / 100 );

} // end SmIterQueue1d::IsOscillating


/*******************************************************************//**
PURPOSE: Solve for a one-dimensional min/max solution using Brent's method.

NOTES:
  This routine is used for minimizing or maximizing a function,
  not for zeroing a function, as SolveIt() does (using Newton's method,
  and sometimes bisection, if Newton fails.)
  This routine uses something similar to Brent's method to minimize
  a function.  It takes either Secant steps on the first derivative
  of the function (to zero in on a min/max), or bisection.
  It does not use inverse-quadratic steps.

  For inputs, bx should strictly between ax and cx, and f(bx) should
  be better than both f(ax) and f(cx).  Also, f'(ax) and f'(cx)
  should have opposite signs.

  Newton's method is much more efficient than Brent's method; Brent's
  is like an enhanced bisection.  If the derivative of a function is
  available, find its min/max by using Newton's method to zero the
  derivative.  If no derivative is available, use Brent's method for
  min/max, or bisection for zeroing.

  To maximize a function, it would probably work better to use minimization
  by using either the negative or the reciprocal of the actual function.
***********************************************************************/
SmStatus SmLocalSolve1d::SolveByBrent // eff: find local maxima
 (double     ax,                      // in : bracket parameter bound   : f(bx) better than f(ax)
  double     bx,                      // in : inbetween parameter value :
  double     cx,                      // in : bracket parameter bound   : f(bx) better than f(cx)
  double     dTol,                    // in :
  SmBoolean  bMaximize,               // in : TRUE = Find local Maximum
                                      //      FALSE= Find local Minimum
  double   & rdFoundT,                // out:
  double   & rdFoundFOfT)             // out:
{
  double dBracketMin = (ax < cx ? ax : cx); // lower bracket x-value
  double dBracketMax = (ax > cx ? ax : cx); // upper bracket x-value

  double u, fu, du; // Next evals
  double x = bx;    // Current evals
  double fx;
  double dx;
  double v = bx;    // Previous evals
  double fv=0;
  double dv=0;
  double w = bx;
  double fw=0;
  double dw=0;

  // Other locals.
  ULONG iter;
  double dDeltaX = 0;
  double e = 0.0;
  double u1, u2;
  double d1, d2;
  SmBoolean bFoundAnswer;

  //
  if ( m_rFunctionEvaluator.Evaluate( x, fx, dx, bFoundAnswer) != SM_SUCCESS )
    { return SM_ERR ; }

  //
  if (bMaximize) { fx = -fx ;
                   dx = -dx ;
                 }

  //
  for (iter=1; iter<=m_lMaxIter; iter++)
    {
      //double xm   = 0.5*(dBracketMin+dBracketMax);
      double tol1 = dTol*smos_Fabs(x)+ZEPS;
      double tol2 = 2.0*tol1;
//      // Don't do this: it will happen any time there are two
//      // consecutive bisections, and also on the first iteration
//      // whenever bx is the midpoint of ax and cx. [bd 090309]
//      if (fabs(x-xm) <= (tol2-0.5*(dBracketMax-dBracketMin))) {
//          rdFoundT=x;
//          rdFoundFOfT=fx;
//          return SM_SUCCESS;
//      }

      // Calculate dDeltaX, by either secant or bisection.
      if ( fabs(e) > tol1 )
        {
          // Trial secant steps, between x and w, and between x and v.
          // The secant steps are on the function derivatives, which
          // should have opposite signs.
          // Init the steps to something too big.
          d2 = d1 = 2.0 * (dBracketMax-dBracketMin);

          // Secant steps on derivatives:
          if (dw != dx && w != x ) { d1 = (w-x) * dx / (dx-dw); }
          if (dv != dx && v != x ) { d2 = (v-x) * dx / (dx-dv); }
          u1 = x + d1;
          u2 = x + d2;

          //  ok if:        between dBracketMin and dBracketMax   &&  slope has opposite sign.
          SmBoolean ok1 = (dBracketMin-u1)*(u1-dBracketMax) > 0.0 && dx*d1 <= 0.0;
          SmBoolean ok2 = (dBracketMin-u2)*(u2-dBracketMax) > 0.0 && dx*d2 <= 0.0;

          //
          double olde = e;
          e = dDeltaX;

          if ( ok1 || ok2 )
            {
              //
              if (ok1 && ok2) // If both ok, use the smaller step.
                { dDeltaX = (smos_Fabs(d1) < smos_Fabs(d2) ? d1 : d2); }
              else if (ok1)
                { dDeltaX = d1; }
              else
                { dDeltaX = d2; }

              //
              if (smos_Fabs(dDeltaX) <= smos_Fabs(0.5*olde))
                {
                  // Test whether this would be too close to dBracketMin or dBracketMax.
                  u = x + dDeltaX;
                  if ( u-dBracketMin < tol2 || dBracketMax-u < tol2 )
                    {
                      dDeltaX = SMSIGN( tol1, dDeltaX  /* xm-x */ );
                    }
                }
              else
                {
                  // Bisection.
                  dDeltaX = 0.5 * ( e = (dx >= 0.0 ? dBracketMin-x : dBracketMax-x) );
                }
            }
          else
            {
              // Bisection.
              dDeltaX = 0.5 * ( e = (dx >= 0.0 ? dBracketMin-x : dBracketMax-x) );
            }
        }
      else
        {
          // Bisection.
          dDeltaX = 0.5 * ( e = (dx >= 0.0 ? dBracketMin-x : dBracketMax-x) );
        }

      // Apply dDeltaX.
      if ( smos_Fabs( dDeltaX ) >= tol1 )
        {
          u = x + dDeltaX;
          if ( u == x )
            {
              // Step is below machine precision.
              rdFoundT=x;
              rdFoundFOfT=fx;
              return SM_SUCCESS;
            }

          //
          if (m_rFunctionEvaluator.Evaluate(u,fu,du,bFoundAnswer) != SM_SUCCESS)
            {
              return SM_ERR;
            }
          if (bMaximize) { fu = -fu ;
                           du = -du ;
                         }
        }
      else //
        {
          //
          u = x + SMSIGN( tol1, dDeltaX );

          // Step must stay within dBracketMin and dBracketMax.
          if ( u <= dBracketMin )
            { u = ( x + dBracketMin ) / 2.0; }
          if ( u >= dBracketMax )
            { u = ( x + dBracketMax ) / 2.0; }
          if (m_rFunctionEvaluator.Evaluate(u,fu,du,bFoundAnswer) != SM_SUCCESS)
            {
              return SM_ERR;
            }
          if (bMaximize) { fu = -fu ;
                           du = -du ;
                         }

          // Termination:
          // - If u==x, then we're within machine precision.
          // - If fu is not better than fx, since the step was so small,
          //   we're down in the 'noise' domain.
          if ( u == x || fu >= fx )
            {
              rdFoundT=x;
              rdFoundFOfT=fx;
              return SM_SUCCESS;
            }
        } // end branch

      // Update values.
      if ( fu <= fx )
        {
          //
          if (u >= x) dBracketMin=x;
          else        dBracketMax=x;

          MOV3(v,fv,dv, w,fw,dw);
          MOV3(w,fw,dw, x,fx,dx);
          MOV3(x,fx,dx, u,fu,du);
        }
      else //
        {
          if (u < x) dBracketMin=u;
          else       dBracketMax=u;

          if (fu <= fw || w == x)
            {  // w == x means w,fw,dw haven't yet been set.
              MOV3(v,fv,dv, w,fw,dw);
              MOV3(w,fw,dw, u,fu,du);
            }
          else if (fu < fv || v==x || v==w)
            {
              MOV3(v,fv,dv, u,fu,du);
            }
        }
    } // end iter until max_iter.

  // We should have returned within the loop.
  SER(SM_ERR);
  return SM_ERR;

} // end SmLocalSolve1d::SolveByBrent

/*******************************************************************//**
PURPOSE: Solve by bisection (splitting interval into halves).

NOTES: This method assumes that the answer is already getting
  quite close.  It also assumes that the lower and upper have opposite
  signs with the lower being negative and the upper being positive.
  (So, necessarily, dLowerT need not be less than dUpperT.)
***********************************************************************/
SmStatus SmLocalSolve1d::SolveByBisection
  (ULONG    lMaxDivisionLevel,
   double   dLowerT,
   double   dUpperT,
   double   dLowerFOfT,
   double   dUpperFOfT,
   double & rdFoundT,
   double & rdFoundFOfT)
{
    // See if we have recursed far enough
    if ( lMaxDivisionLevel == 0 )
    {
        // Return the better value.  (Remember, Low is neg, High is pos.)
        if ( dLowerFOfT + dUpperFOfT >= 0 )
        {
            rdFoundT    = dLowerT;
            rdFoundFOfT = dLowerFOfT;
        }
        else
        {
            rdFoundT    = dUpperT;
            rdFoundFOfT = dUpperFOfT;
        }
        return SM_SUCCESS;
    }

    // Basically do bisection until interval collapses.
    // We don't actually use bisection.  Since we have the function
    // values (not just + or -), we can use Regula Falsi (the
    // secant method), which has much better convergence.
    // [ BD, July 2007, revised Dec 2007 ]

//  double dMidT = ( dLowerT + dUpperT ) / 2.0;

    double dFrac = - dLowerFOfT / ( dUpperFOfT - dLowerFOfT );
    // For case such as: low F == -1e-8, high F == 775.0 : would just creep along.
    if ( dFrac < 0.15 ) { dFrac = 0.15; }
    if ( dFrac > 0.85 ) { dFrac = 0.85; }
    double dMidT = dLowerT + dFrac * ( dUpperT - dLowerT );


    double dScale = 1.0 * smos_Fabs(dMidT);
    if (   smos_Fabs(dMidT-dUpperT) < SM_EFF_ZERO_SQ*dScale
        || smos_Fabs(dMidT-dLowerT) < SM_EFF_ZERO_SQ*dScale )
    {
        if (smos_Fabs(dLowerFOfT) < smos_Fabs(dUpperFOfT))
        {
            rdFoundT    = dLowerT;
            rdFoundFOfT = dLowerFOfT;
        }
        else if (smos_Fabs(dUpperFOfT) < smos_Fabs(dLowerFOfT))
        {
            rdFoundT    = dUpperT;
            rdFoundFOfT = dUpperFOfT;
        }
        else // They can be identical.  Use the midpoint.
        {
            rdFoundT    = ( dLowerT    + dUpperT    ) / 2.0;
            rdFoundFOfT = ( dLowerFOfT + dUpperFOfT ) / 2.0;
        }
        return SM_SUCCESS;
    }

    double dTmp, dMidFOfT;
    SmBoolean bFoundAnswer;
    if ( m_rFunctionEvaluator.Evaluate( dMidT, dMidFOfT, dTmp, bFoundAnswer ) != SM_SUCCESS )
    {
        return SM_ERR;
    }
    // If we have found the correct answer or we hit zero exactly we are done.
    if ( bFoundAnswer || dMidFOfT == 0.0 )
    {
        rdFoundT    = dMidT;
        rdFoundFOfT = dMidFOfT;
        return SM_SUCCESS;
    }

    // No, don't do this: if it's bracketed, there must be a solution
    // somewhere in between.
//  //
//  // If the middle function value is worse than either end value
//  // then return minimum of the two ends.
//  // Note: we know that dLowerFOfT < 0 and dUpperFOfT > 0.
//  if ( dMidFOfT < dLowerFOfT || dMidFOfT > dUpperFOfT )
//  {
//      if (smos_Fabs(dLowerFOfT) < smos_Fabs(dUpperFOfT)) {
//          rdFoundT    = dLowerT;
//          rdFoundFOfT = dLowerFOfT;
//      }
//      else {
//          rdFoundT    = dUpperT;
//          rdFoundFOfT = dUpperFOfT;
//      }
//      return SM_SUCCESS;
//  }

    // Continue with recursion, using whichever interval brackets zero.
    if ( dMidFOfT > 0.0 )
    {
        return SolveByBisection( lMaxDivisionLevel-1,
                dLowerT,    dMidT,
                dLowerFOfT, dMidFOfT,
                rdFoundT,   rdFoundFOfT );

    }
    else // dMidFOfT < 0.0
    {
        return SolveByBisection( lMaxDivisionLevel-1,
                dMidT,    dUpperT,
                dMidFOfT, dUpperFOfT,
                rdFoundT, rdFoundFOfT );

    }

} // end SolveByBisection

/*******************************************************************//**
PURPOSE: Find a solution when Newton itertaion is oscillating.

NOTES:
   Output: besides the two output arguments, output is also left
   in this class' data member m_vCurr: contains the final t and F values.
***********************************************************************/
SmStatus SmLocalSolve1d::SolveOscillating(
       double      dGuessT,         // in:  given guess parameter
       SmBoolean & rbFoundSolution, // out: TRUE = NR converged to a solution
       double    & rdFoundT         // out: Converged Parameter Value
    )
{
  // If ready for bisection, do that.
  // Do bisection any time the solution is bracketed, don't bother
  // checking how close to solution we are, because we're oscillating,
  // and nothing else is going to work.

  // Locals
  ULONG i;
  SmStatus eStat;
  SmBoolean bFoundAnswer;

  // Save the best in-between function value.
  // If we search the interior, these will be set there;
  // initialize them in case we don't search.
  SmIterationValue sTempIter;
  sTempIter.m_dT = 0.5 * ( m_v1Prev.m_dT + m_vCurr.m_dT );
  eStat = m_rFunctionEvaluator.Evaluate( sTempIter.m_dT,
                                 sTempIter.m_dFOfT,
                                 sTempIter.m_dFPrimeOfT,
                                 bFoundAnswer );
  // Check if evaluator failed
  // When eStat is not success, sTempIter.m_dFOfT and sTempIter.m_dFPrimeOfT are not initialized
  // When bFoundAnswer is false, they seem at least initialized
  if( eStat != SM_SUCCESS ) {
        return eStat;
  }

  double dBestInteriorT = sTempIter.m_dT;
  double dBestInteriorF = sTempIter.m_dFOfT;

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  TCHAR sBuff[SM_TBLOCK_SIZE];
#endif // SM_DEBUG_CODE

  // If it's not bracketed, try to find brackets.
  if ( ! m_bHaveLowerBracket || ! m_bHaveUpperBracket )
    {
      // Not bracketed.  Try to find a bracket.
      // We know that Curr and Prev have the same signs, otherwise
      // both of our Bracket flags would be set.
      // See if there's a value in between with the opposite sign.
      // Try at 1/4, 1/2, 3/4, and in between those.
      // (Note, we already did 1/2.)
      // Also, since the slopes are presumably opposite (otherwise
      // it wouldn't oscillate), evaluate very near the ends.
      const ULONG clNumGuesses = 8;
      static const double sGuesses[clNumGuesses] = { 0.25, 0.75,
                                                     0.13, 0.38, 0.82, 0.87,
                                                     0.01, 0.99 };
      double dCurrT    = m_vCurr.m_dT;
      double dPrevT    = m_v1Prev.m_dT;
      double dCurrFVal = m_vCurr.m_dFOfT;

      for ( i = 0; i < clNumGuesses; i++ )
        {
          double dMidT = dPrevT + sGuesses[i] * ( dCurrT - dPrevT );
          sTempIter.m_dT = dMidT;
          if ( m_rFunctionEvaluator.Evaluate( sTempIter.m_dT,
                                              sTempIter.m_dFOfT,
                                              sTempIter.m_dFPrimeOfT,
                                              bFoundAnswer ) != SM_SUCCESS)
            {
              continue;  // just keep trying...
            } // end if function evaluator failed.

          // Save the best in-between function value.
          if ( smos_Fabs( sTempIter.m_dFOfT ) < smos_Fabs( dBestInteriorF ) )
            {
              dBestInteriorF = sTempIter.m_dFOfT;
              dBestInteriorT = dMidT;
            }

          if ( sTempIter.m_dFOfT * dCurrFVal <= 0 )
            {
              break; // Found one: opposite sign.
            }
        } // end loop on clNumGuesses interior values looking for sign change

      // See if we found a sign change.
      if ( sTempIter.m_dFOfT * dCurrFVal <= 0 )
        {
          // Found a sign change.  Set our member bracketing data.

          // First, Curr F and Prev F have the same sign, opposite of Mid F,
          // so there are two zero crossings, one on each side of Mid.
          // Choose Curr or Prev based on which is closer to the given guess param.
          // Set the new low and high brackets in these variables:
          double dNewLoT, dNewLoF, dNewHiT, dNewHiF;
          if ( sTempIter.m_dFOfT > 0 )
            {
              dNewHiT = sTempIter.m_dT;
              dNewHiF = sTempIter.m_dFOfT;
              if ( dGuessT < sTempIter.m_dT )
                {
                  if ( dCurrT < dPrevT )
                    {
                      dNewLoT = dCurrT;
                      dNewLoF = dCurrFVal;
                    }
                    else
                    {
                      dNewLoT = dPrevT;
                      dNewLoF = m_v1Prev.m_dFOfT;
                    }
                }
                else
                {
                  if ( dCurrT > dPrevT )
                    {
                      dNewLoT = dCurrT;
                      dNewLoF = dCurrFVal;
                    }
                    else
                    {
                      dNewLoT = dPrevT;
                      dNewLoF = m_v1Prev.m_dFOfT;
                    }
                }
            }
            else // Mid is neg, Curr and Prev are pos.
            {
              dNewLoT = sTempIter.m_dT;
              dNewLoF = sTempIter.m_dFOfT;
              if ( dGuessT < sTempIter.m_dT )
                {
                  if ( dCurrT < dPrevT )
                    {
                      dNewHiT = dCurrT;
                      dNewHiF = dCurrFVal;
                    }
                    else
                    {
                      dNewHiT = dPrevT;
                      dNewHiF = m_v1Prev.m_dFOfT;
                    }
                }
                else
                {
                  if ( dCurrT > dPrevT )
                    {
                      dNewHiT = dCurrT;
                      dNewHiF = dCurrFVal;
                    }
                    else
                    {
                      dNewHiT = dPrevT;
                      dNewHiF = m_v1Prev.m_dFOfT;
                    }
                }
            } // end if-else, setting new low and high brackets.

          // Now we have set dNewLoT, dNewLoF, dNewHiT, dNewHiF.
          // If we already had a bracket, use that instead if it's better.
          // If we have a choice, use the values whose parameters are
          // closer together, not whose function values are closer to zero.
          // The function could increase in the middle, but bisection
          // doesn't care, it just closes in, so start as close as possible.

          if ( m_bHaveLowerBracket )
            {
              // Use whichever is closer to dNewHiT.
              if ( smos_Fabs( m_dLowerBracketT - dNewHiT ) < smos_Fabs( dNewLoT - dNewHiT ) )
                {
                  dNewLoT = m_dLowerBracketT;
                  dNewLoF = m_dLowerBracketF;
                }
            }
          if ( m_bHaveUpperBracket )
           {
              // Use whichever is closer to dNewLoT.
              if ( smos_Fabs( m_dUpperBracketT - dNewLoT ) < smos_Fabs( dNewHiT - dNewLoT ) )
                {
                  dNewHiT = m_dUpperBracketT;
                  dNewHiF = m_dUpperBracketF;
                }
            }

          // Announce our findings.
          m_bHaveLowerBracket = TRUE; m_dLowerBracketT = dNewLoT; m_dLowerBracketF = dNewLoF;
          m_bHaveUpperBracket = TRUE; m_dUpperBracketT = dNewHiT; m_dUpperBracketF = dNewHiF;

        } // end if found a sign change.

    } // end if not initially bracketed, tried to find one.

  // Note, here, if still not bracketed, we could try looking outside of the
  // interval between Prev and Curr: step out to the limits of m_vInterval.

  // when zero crossing has been bracketed
  if ( m_bHaveLowerBracket && m_bHaveUpperBracket )
    {
      // Solve with bisection.
      // Use a deeper level here, because we might not be very close.
      eStat = SolveByBisection( 40, m_dLowerBracketT, m_dUpperBracketT,
          m_dLowerBracketF, m_dUpperBracketF,
          m_vCurr.m_dT, m_vCurr.m_dFOfT );

      // Return success even if the function value is too big.
      // In case of a discontinuity, the caller would rather have
      // the best solution instead of none.
      // Example: finding an extremum of a curve at a kink.  [B545]

      if ( eStat == SM_SUCCESS )  // && smos_Fabs( m_vCurr.m_dFOfT ) < m_dAcceptableAccuracy
        {
          // set output
          rbFoundSolution      = TRUE;
          rdFoundT             = m_vCurr.m_dT;
          m_dFoundAccuracy     = m_vCurr.m_dFOfT;
          m_eTerminationReason =
                  ( smos_Fabs( m_vCurr.m_dFOfT ) < m_dDesiredAccuracy )
                       ? SM_TR_FOUND_ANSWER_CONVERGED
                       : SM_TR_FOUND_ANSWER_CLOSE;

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              smos_sprintf(sBuff,_T("%s"),_T("Newton iteration Oscillating, returning Bisection:\n"));
              smos_WriteBuffer(sBuff);
              smos_sprintf(sBuff,_T("curr T = %16.16lf, F = %16.16lf\n"),
                  m_vCurr.m_dT, m_vCurr.m_dFOfT );
              smos_WriteBuffer(sBuff);
            }
#endif // SM_DEBUG_CODE
          return SM_SUCCESS;

        } // end bisection-succeeded check

    } // end can-try-bisection because bracketed check

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      smos_sprintf(sBuff,_T("%s"),_T("Breaking iteration: Oscillating, no solution between.\n"));
      smos_WriteBuffer(sBuff);
    }
#endif // SM_DEBUG_CODE

  // Ok, we weren't able to bracket the solution.
  // We're oscillating between two values, both with the same sign function,
  // and no sign change in between.  Looks like no solution.
  // If we're doing Brent min or max, then we go ahead and call Brent
  // and return, otherwise there's not much we can do.

  // (Note: with oscillation, two remedies might be, try a guess
  // in between the two values, or cut the step size.
  // In practice, the former will settle right back to the original
  // oscillation values, and the latter will end up oscillating on
  // values that are closer together.  So, it's tough to break
  // oscillation using Newton iteration; something more brute-force
  // such as bisection or Brent is indicated.)

  // Set m_vCurr.m_dT, using Brent if appropriate.

  if (   m_eLocalSolverType == SM_SA_BRENT_MINIMIZE
      || m_eLocalSolverType == SM_SA_BRENT_MAXIMIZE )
    {
      // We are doing Brent min or max.  Go ahead with that.
#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          smos_sprintf(sBuff,_T("%s"),_T("   Returning Brent min/max solution.\n"));
          smos_WriteBuffer(sBuff);
        }
#endif // SM_DEBUG_CODE

      double dTemp;
      SmBoolean bMaximize = ( m_eLocalSolverType == SM_SA_BRENT_MAXIMIZE );
      eStat = SolveByBrent( m_v1Prev.m_dT, dBestInteriorT, m_vCurr.m_dT,
                             m_dDesiredAccuracy, bMaximize,
                             rdFoundT, dTemp);
      if ( eStat != SM_SUCCESS )
        {
          // SolveByBrent() often fails; it's not worth a warning message.
          return ( eStat );
          // SER( eStat );
          // SER_MSG( eStat, _T("Failed SolveByBrent - Probably not significant") );
        }

      m_vCurr.m_dT = rdFoundT;
    }
  else
    {
      // Not doing Brent min/max, so quit.
      // But if our TempIter is better than CurrIter,
      // replace CurrIter with it.
      if ( smos_Fabs( dBestInteriorF ) < smos_Fabs( m_vCurr.m_dFOfT ) )
        {
          m_vCurr.m_dT = dBestInteriorT;
        }
      else
        {
          // just try midpt
          m_vCurr.m_dT = 0.5 * ( m_v1Prev.m_dT + m_vCurr.m_dT );
        }
    }

  // The caller expects the solution in m_vCurr.
  eStat = m_rFunctionEvaluator.Evaluate( m_vCurr.m_dT,
      m_vCurr.m_dFOfT,
      m_vCurr.m_dFPrimeOfT,
      bFoundAnswer );

  // What if the evaluator failed here?
  if( eStat != SM_SUCCESS ) {
      rbFoundSolution = FALSE;
      return ( eStat );
  }

  rdFoundT = m_vCurr.m_dT;
  if (   m_eLocalSolverType == SM_SA_BRENT_MINIMIZE
      || m_eLocalSolverType == SM_SA_BRENT_MAXIMIZE )
    {
      rbFoundSolution = TRUE;  // For min/max, tol doesn't matter.
    }
  else
    {
      rbFoundSolution = bFoundAnswer || ( smos_Fabs( m_vCurr.m_dFOfT ) <= m_dAcceptableAccuracy );
    }

  return SM_SUCCESS;

} // end SolveOscillating

/*******************************************************************//**
PURPOSE: This method uses various iteration techniques to try to
    find a root of a function.  The function being solved must be supplied
    in the m_rFunctionEvaluator.

NOTES:
   Output: besides the two output arguments, output is also left
   in this class' data member m_vCurr: contains the final t and F values.

   Function returns SM_SUCCESS (even if no solution found) unless
   something went fundamentally wrong, such as failure of evaluator.
***********************************************************************/
SmStatus SmLocalSolve1d::SolveIt
 (double      dGuessT,              // in : initial guess
  double      dAcceptableAccuracy,  // in : Solution found when f(t) < dAcceptableAccuracy
                                    //      f(t) varies for different SmEvalFunctionObject Objects
                                    //      SmCurvePropertyEFO     - one of 20 different functions
                                    //      SmFindClippedRadiusEFO -
                                    //      SmFindPCExtremaEFO     - Func = (f(t) - P) * f'(t)
                                    //      SmFindSilSingEFO       -
  SmBoolean & rbFoundSolution,      // out: TRUE = NR converged to a solution
  double    & rdFoundT)             // out: Converged Parameter Value
{
  // init output
  rbFoundSolution = FALSE;

  // init local iteration parameters
  m_bHaveLowerBracket   = FALSE;
  m_bHaveUpperBracket   = FALSE;
  m_dLowerBracketF      = - SM_BIG_DOUBLE;
  m_dUpperBracketF      =   SM_BIG_DOUBLE;
  m_lIter               = 0;
  m_lLowerBoundHits     = 0;
  m_lUpperBoundHits     = 0;
  m_lMaxBoundaryHits    = 2;
  m_eTerminationReason  = SM_TR_UNABLE_TO_CONVERGE;
  m_dAcceptableAccuracy = dAcceptableAccuracy;

  m_vCurr.m_dPrevStep   = 0;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
TCHAR sBuff[SM_TBLOCK_SIZE];
#endif // SM_DEBUG_CODE

  // check input - initial guess must be within problem interval
  if (     m_bUseInterval
      && ! m_vInterval.ContainsValue( dGuessT ))
    {
       // Don't give up in this case.  [bd 07 Nov 07]
       if ( ! m_vInterval.ContainsValue( dGuessT, SM_EFF_ZERO_SQRT ))
         { SE( SM_ERR_INVALID_INPUT ); }

       dGuessT = m_vInterval.ClampValue( dGuessT );
    }


  // Evaluate func at the given guess value.
  m_vCurr.m_dT = dGuessT;
  SmBoolean bFoundAnswer;
  SmStatus eStat = m_rFunctionEvaluator.Evaluate( m_vCurr.m_dT,
                                                  m_vCurr.m_dFOfT,
                                                  m_vCurr.m_dFPrimeOfT,
                                                  bFoundAnswer,
                                                  FALSE);               // in : TRUE = signal errors, FALSE=errors anticipated, don't signal, default:[TRUE]

  // If that call failed:
  if ( eStat != SM_SUCCESS )
    {
#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          smos_sprintf(sBuff,_T("%s"),_T("\nEnter SolveIt( 1 dim ): bad first eval, tweaking.\n"));
          smos_WriteBuffer(sBuff);
        }
#endif // SM_DEBUG_CODE

      // Have a bad start point - if we are solving in an interval
      // then just take another start point close to the bad one
      if ( m_bUseInterval )
        {
          // Tweak the point by 1/1000th of interval.
          double dSize = m_vInterval.GetLength();
          m_vCurr.m_dT = dGuessT + dSize / 1000.0;
          if ( !m_vInterval.ContainsValue( m_vCurr.m_dT ))
            {
              m_vCurr.m_dT = dGuessT - dSize / 1000.0;
            }

          // ReEvaluate at tweaked parameter.
          eStat = m_rFunctionEvaluator.Evaluate( m_vCurr.m_dT,
              m_vCurr.m_dFOfT, m_vCurr.m_dFPrimeOfT, bFoundAnswer );
          if ( eStat != SM_SUCCESS )
            {
              // when that fails - quit

#ifdef SM_DEBUG_CODE
              if (bDebugMe)
                {
                  smos_sprintf(sBuff,_T("%s"),_T("Enter SolveIt( 1 dim ): Tweaked eval failed, quitting.\n"));
                  smos_WriteBuffer(sBuff);
                }
#endif // SM_DEBUG_CODE

              return SM_ERR;
            }
        }
    } // end Evaluate failure branch

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      smos_sprintf(sBuff,_T("\nEnter SolveIt( 1 dim ): Initial guess: T = %16.16lf, F = %16.16lf\n"),
          m_vCurr.m_dT, m_vCurr.m_dFOfT );
      smos_WriteBuffer(sBuff);
    }
#endif // SM_DEBUG_CODE

  // Check serendipity, whether the given guess was a solution.
  if ( bFoundAnswer )
    {
      // save solution and exit
      rbFoundSolution      = TRUE;
      rdFoundT             = m_vCurr.m_dT;
      m_dFoundAccuracy     = m_vCurr.m_dFOfT;
      m_eTerminationReason = SM_TR_FOUND_ANSWER_CONVERGED;

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          smos_sprintf(sBuff,_T("%s"),_T("Enter SolveIt( 1 dim ): Initial guess was good.\n"));
          smos_WriteBuffer(sBuff);
        }
#endif // SM_DEBUG_CODE

      return SM_SUCCESS;
    }

  // Arrive here when given guess was not a solution, and we must iterate.
  // State: m_vCurr contains evaluation at input guess parameter.

  // init iteration values
  SmBoolean bNewtonFailing = FALSE;
  double    dCutFactor     = 1.0;
  double    dMinimum       = smos_Fabs(m_vCurr.m_dFOfT);
  ULONG     lMinCount      = 0;

  SmIterQueue1d sIterQueue( 8 );

  // for up to max iterations - improve the guessPoint
  for ( m_lIter = 0; m_lIter < m_lMaxIter; m_lIter++ )
    {
      // Bracketing: used only for bisection, after Newton loop
      // if Newton loop fails to converge.

      // Set Lower Bracket value for zero crossings
      if ( m_vCurr.m_dFOfT < 0.0 )
        {
          m_bHaveLowerBracket = TRUE;
          if (m_vCurr.m_dFOfT > m_dLowerBracketF)
            {
              m_dLowerBracketT = m_vCurr.m_dT;
              m_dLowerBracketF = m_vCurr.m_dFOfT;
            }
        }

      // Set Upper Bracket value for zero crossings
      if ( m_vCurr.m_dFOfT > 0.0 )
        {
          m_bHaveUpperBracket = TRUE;
          if (m_vCurr.m_dFOfT < m_dUpperBracketF)
            {
              m_dUpperBracketT = m_vCurr.m_dT;
              m_dUpperBracketF = m_vCurr.m_dFOfT;
            }
        }

      // Check convergence - successful exit point.
      if ( smos_Fabs( m_vCurr.m_dFOfT ) < m_dDesiredAccuracy )
        {
          // Success - save solution and exit
          rbFoundSolution      = TRUE;
          rdFoundT             = m_vCurr.m_dT;
          m_dFoundAccuracy     = m_vCurr.m_dFOfT;
          m_eTerminationReason = SM_TR_FOUND_ANSWER_CONVERGED;

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              smos_sprintf(sBuff,_T("Converged, %lu iterations, T = %16.16lf, F = %16.16lf\n"),
                  m_lIter, m_vCurr.m_dT, m_vCurr.m_dFOfT );
              smos_WriteBuffer(sBuff);
            }
#endif // SM_DEBUG_CODE

          return SM_SUCCESS;

        } // end converged check

      // State: have not yet achieved DesiredAccuracy

      // /Check for oscillation.
      SmBoolean bOscillating = sIterQueue.IsOscillating();
      if ( bOscillating )
        {
          eStat = SolveOscillating( dGuessT, rbFoundSolution, rdFoundT );

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              smos_sprintf(sBuff,_T("Oscillating: Found solution? %d; returning status %lu; T = %16.16lf, F = %16.16lf\n"),
                  rbFoundSolution, eStat, m_vCurr.m_dT, m_vCurr.m_dFOfT );
              smos_WriteBuffer(sBuff);
            }
#endif // SM_DEBUG_CODE

          return eStat;
        }

      // Normal case: calculate and apply the Newton step.
      if ( !bNewtonFailing )
        {
          // Handle f'(t) goes to zero
          if (SM_IS_ZERO(m_vCurr.m_dFPrimeOfT))
            {
              // Set f'(t) = +/- 1.0.
              // Won't get good convergence, but might nudge it in the right direction.
              if (m_vCurr.m_dFPrimeOfT < 0.0) { m_vCurr.m_dFPrimeOfT = -1.0; }
              else                            { m_vCurr.m_dFPrimeOfT =  1.0; }
            }

          // Newton step: -f(t) / f'(t)
          double dDeltaT = - m_vCurr.m_dFOfT / m_vCurr.m_dFPrimeOfT;

          // Don't ever take a huge step.
          // Note, even 0.10 can be too big.
          double dStepLimit = m_vInterval.GetLength() * 0.08 + SM_EFF_ZERO;

          if ( fabs( dDeltaT ) > dStepLimit )
          {
              dStepLimit *= 0.99;  // just to avoid exact half
              if ( dDeltaT < 0 )
                { dStepLimit = -dStepLimit; }
              dDeltaT = dStepLimit;
          }

          // Before we give up try cutting the step size.

          if ( m_lIter > 20 && m_lIter >= m_lMaxIter - 1 )
            {
              dCutFactor *= 0.5;
              if (dCutFactor > 0.0001 )
                {
                  // Cut by a factor and give it 20 more tries to converge.
                  m_lIter = m_lIter - 20;
                }
            } // end cutting step size.

          dDeltaT = dDeltaT * dCutFactor;

          // Apply the step.
          double dNewT = m_vCurr.m_dT + dDeltaT;

       //         // when zero crossing has been bracketed - currently disabled
       //         if(   m_bHaveLowerBracket
       //            && m_bHaveUpperBracket)
       //           {
       //             SmExtent1d sTInterval;
       //             sTInterval.AddValue(m_dLowerBracketT);
       //             sTInterval.AddValue(m_dUpperBracketT);
       //             if (!sTInterval.ContainsValue(dNewT))
       //               {
       //                 // We are diverging - failing
       //                 // disable bracketing for now        bNewtonFailing = TRUE;
       //               }
       //           } // end zero crossing has been bracketed check

          // When nextGuess leaves given problem domain.
          if (    m_bUseInterval
              && !m_vInterval.ContainsValue( dNewT ))
            {
              // Handle periodicity: a local solve should not jump across seams unless specified.
              if ( m_bIsPeriodic )
                {
                  dNewT = m_vInterval.PeriodicWrap(dNewT);
                }
              else // clamp nextGuess and count boundary hits
                {
                  // clamp nextGuess to interval
                  // Also reset the flag if this iteration didn't hit that side.
                  // That way, for example, oscillating iterations will be caught
                  // and solved for.  [B545]
                  if ( dNewT < m_vInterval.GetMin() )
                  {
                      dNewT = m_vInterval.GetMin();
                      m_lLowerBoundHits ++;
                      m_lUpperBoundHits = 0;
                  }
                  else if ( dNewT > m_vInterval.GetMax() )
                  {
                      dNewT = m_vInterval.GetMax();
                      m_lUpperBoundHits ++;
                      m_lLowerBoundHits = 0;
                  }

                  // quit when stepping out of bounds persists
                  if(    m_lLowerBoundHits > m_lMaxBoundaryHits
                     ||  m_lUpperBoundHits > m_lMaxBoundaryHits)
                    {
                      rdFoundT = dNewT;
                      m_eTerminationReason = SM_TR_OUT_OF_BOUNDS;
#ifdef SM_DEBUG_CODE
                      if (bDebugMe)
                        {
                          smos_sprintf(sBuff,_T("%s"),_T("Breaking iteration: Pushing Boundary.\n"));
                          smos_WriteBuffer(sBuff);
                        }
#endif // SM_DEBUG_CODE
                      break;
                    }
                } // end interval is NOT periodic branch
            } // end NextGuess leaves given problem domain.
          else
            {
              // Reset if not hit on this iteration.  [B545]
              m_lLowerBoundHits = m_lUpperBoundHits = 0;
            }

          // State: dNewT has been set, including damping, clamping, etc.

          // save last two answers
          m_vCurr.m_dPrevStep = ( m_lIter > 0 )
                     ? m_vCurr.m_dT - m_v1Prev.m_dT
                     : 0;
          m_v2Prev =  ( m_lIter > 1 )
                     ? m_v1Prev
                     : m_vCurr;
          m_v1Prev = m_vCurr;

          sIterQueue.Add( m_v1Prev );


          // Now do the function evaluation, into m_vCurr.
          m_vCurr.m_dT = dNewT;
          eStat = m_rFunctionEvaluator.Evaluate( m_vCurr.m_dT,
                                                 m_vCurr.m_dFOfT,
                                                 m_vCurr.m_dFPrimeOfT,
                                                 bFoundAnswer );
          if ( eStat != SM_SUCCESS )
            {
              rbFoundSolution = FALSE;

#ifdef SM_DEBUG_CODE
              if (bDebugMe)
                {
                  smos_sprintf(sBuff,_T("%s"),_T("Quitting: Function evaluator failed.\n"));
                  smos_WriteBuffer(sBuff);
                }
#endif // SM_DEBUG_CODE
              return SM_ERR; // Had an error in evaluator
            }

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              smos_sprintf(sBuff,_T("%3lu: T = %16.16lf, F = %16.16lf, dDeltaT = %16.16lf\n"),
                  m_lIter, m_vCurr.m_dT, m_vCurr.m_dFOfT, dDeltaT );
              smos_WriteBuffer(sBuff);
            }
#endif // SM_DEBUG_CODE

          // Exit when Evaluate says it converged.
          if ( bFoundAnswer  || smos_Fabs( m_vCurr.m_dFOfT ) < m_dDesiredAccuracy )
            {
              rbFoundSolution      = TRUE;
              rdFoundT             = m_vCurr.m_dT;
              m_dFoundAccuracy     = m_vCurr.m_dFOfT;
              m_eTerminationReason = SM_TR_FOUND_ANSWER_CONVERGED;

#ifdef SM_DEBUG_CODE
              if (bDebugMe)
                {
                  smos_sprintf(sBuff,_T("%s"),_T("Quitting: Function evaluator reports success.\n"));
                  smos_WriteBuffer(sBuff);
                }
#endif // SM_DEBUG_CODE
              return SM_SUCCESS;
            }

          // Note that if we don't push down the minimum after 4 iterations
          // we are stuck in a loop - try bisection instead.
          if ( smos_Fabs( m_vCurr.m_dFOfT ) >= dMinimum )
            {
              lMinCount ++;
              if (   lMinCount >= 4
                  && smos_Fabs( m_vCurr.m_dFOfT ) < m_dAcceptableAccuracy )
                {
#ifdef SM_DEBUG_CODE
                  if (bDebugMe)
                    {
                      smos_sprintf(sBuff,_T("%s"),_T("Breaking iteration: Function value increasing.\n"));
                      smos_WriteBuffer(sBuff);
                    }
#endif // SM_DEBUG_CODE
                  break;  // get out and try bisection
                }
            }
          else // reset count of iterations since last decrease in Func() value
            {
              lMinCount = 0;
              dMinimum  = smos_Fabs( m_vCurr.m_dFOfT );
            }

          // If we are doing BRENT minimization/maximization wait until we have
          // somewhat bracketed the minimum or maximum before we call solver
          if(   m_eLocalSolverType == SM_SA_BRENT_MAXIMIZE
             && m_lIter > 1 )
            {
              if (   m_vCurr.m_dFOfT > m_v1Prev.m_dFOfT
                  && m_vCurr.m_dFOfT > m_v2Prev.m_dFOfT )
                {
                  SmExtent1d sTmpExt( m_v1Prev.m_dT );
                  sTmpExt.AddValue( m_v2Prev.m_dT );
                  if ( sTmpExt.ContainsValue( m_vCurr.m_dT ))
                    {
                      double dTemp;
                      eStat = SolveByBrent( m_v1Prev.m_dT, m_vCurr.m_dT, m_v2Prev.m_dT,
                                             m_dDesiredAccuracy, TRUE,
                                             rdFoundT, dTemp);
                      if ( eStat != SM_SUCCESS )
                        {
                          SER( eStat );
                          // SER_MSG( eStat, _T("Failed SolveByBrent - Probably not significant"));
                        }
                      rbFoundSolution = TRUE;
                      return SM_SUCCESS;
                    }
                }
            } // end wait for bracketed zero to start Brent maximize solver check

          if(  m_eLocalSolverType == SM_SA_BRENT_MINIMIZE
            && m_lIter > 1 )
            {
              if(   m_vCurr.m_dFOfT < m_v1Prev.m_dFOfT
                 && m_vCurr.m_dFOfT < m_v2Prev.m_dFOfT )
                {
                  SmExtent1d sTmpExt( m_v1Prev.m_dT );
                  sTmpExt.AddValue( m_v2Prev.m_dT );
                  if ( sTmpExt.ContainsValue( m_vCurr.m_dT ))
                    {
                      double dTemp;
                      eStat = SolveByBrent( m_v1Prev.m_dT, m_vCurr.m_dT, m_v2Prev.m_dT,
                                             m_dDesiredAccuracy, FALSE,
                                             rdFoundT, dTemp);
                      if ( eStat != SM_SUCCESS )
                        {
                          // SolveByBrent() often fails; it's not worth a warning message.
                          return ( eStat );
                          // SER( eStat );
                          // SER_MSG( eStat, _T("Failed SolveByBrent - Probably not significant"));
                        }
                      rbFoundSolution = TRUE;
                      return SM_SUCCESS;
                    }
                }
            } // end wait for bracketed zero to start Brent minimizer solver check

          // If we are getting larger and on same side of answer
          // It is unfair to kick it out if it is going back and forth.

          if(   smos_Fabs( m_vCurr.m_dFOfT ) >= smos_Fabs( m_v1Prev.m_dFOfT )
             && m_vCurr.m_dFOfT * m_v1Prev.m_dFOfT > 0.0)
            {
              // Function got worse, without a sign change.
              // Check whether we've moved on this iteration.

              if(   m_vCurr.m_dFOfT != m_v1Prev.m_dFOfT
                 && m_vCurr.m_dT    != m_v1Prev.m_dT )
                {
                  // when zero crossing is bracketed
                  if(   m_bHaveLowerBracket
                     && m_bHaveUpperBracket)
                    {
#ifdef SM_DEBUG_CODE
                      if (bDebugMe)
                        {
                          smos_sprintf(sBuff,_T("%s"),_T("Breaking iteration: Function increasing, bracketed.\n"));
                          smos_WriteBuffer(sBuff);
                        }
#endif // SM_DEBUG_CODE
                      break; // If have a bracket then break out and try
                             // to do minimization.
                    }
                }

              // Exit with or without solution when nextStep goes to zero
              if ( smos_Fabs( dDeltaT ) < SM_EFF_ZERO )
                {
                  // set output and exit
                  double dFMag = smos_Fabs( m_vCurr.m_dFOfT );
                  rbFoundSolution      = ( dFMag < m_dAcceptableAccuracy );
                  rdFoundT             = m_vCurr.m_dT;
                  m_dFoundAccuracy     = m_vCurr.m_dFOfT;
                  m_eTerminationReason =
                      ( dFMag < m_dDesiredAccuracy    ) ? SM_TR_FOUND_ANSWER_CONVERGED
                    : ( dFMag < m_dAcceptableAccuracy ) ? SM_TR_FOUND_ANSWER_CLOSE
                                                        : SM_TR_UNABLE_TO_CONVERGE;
#ifdef SM_DEBUG_CODE
                  if (bDebugMe)
                    {
                      smos_sprintf(sBuff,_T("Quitting: zero step size: %16.16lf.\n"),
                          dDeltaT );
                      smos_WriteBuffer(sBuff);
                    }
#endif // SM_DEBUG_CODE
                  return SM_SUCCESS;
                }
            } // end function is diverging monotonically check
        } // end Newton/Raphson is working check
    } // End NR iteration

  // arrive here when iteration failed to converge because
  //  1. max iterations was exceeded
  //  2. repeatedly stepped out of bounds
  //  3. Func Evals were monotonically diverging over several iterations
  // the function has already returned if iterations converged

  // Try bisection when applicable before quitting.

  if ( m_bHaveLowerBracket && m_bHaveUpperBracket )
    {
      // solve with bisection
      SolveByBisection( 20, m_dLowerBracketT, m_dUpperBracketT,
          m_dLowerBracketF, m_dUpperBracketF,
          m_vCurr.m_dT, m_vCurr.m_dFOfT );
    }

  if ( smos_Fabs( m_vCurr.m_dFOfT ) < m_dAcceptableAccuracy )
    {
      // set output
      rbFoundSolution      = TRUE;
      rdFoundT             = m_vCurr.m_dT;
      m_dFoundAccuracy     = m_vCurr.m_dFOfT;
      double dFMag         = smos_Fabs( m_vCurr.m_dFOfT );
      m_eTerminationReason =
          ( dFMag < m_dDesiredAccuracy    ) ? SM_TR_FOUND_ANSWER_CONVERGED
        : ( dFMag < m_dAcceptableAccuracy ) ? SM_TR_FOUND_ANSWER_CLOSE
                                            : SM_TR_UNABLE_TO_CONVERGE;

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          smos_sprintf(sBuff,_T("%s"),_T("Newton iteration failed, returning Bisection:\n"));
          smos_WriteBuffer(sBuff);
          smos_sprintf(sBuff,_T("curr T = %16.16lf, F = %16.16lf\n"),
              m_vCurr.m_dT, m_vCurr.m_dFOfT );
          smos_WriteBuffer(sBuff);
        }
#endif // SM_DEBUG_CODE
      return SM_SUCCESS;

    } // end if bisection worked

  // At least set best answer so that min/max will work
  // even if we don't converge here.
  rdFoundT         = m_vCurr.m_dT;
  m_dFoundAccuracy = m_vCurr.m_dFOfT;

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      smos_sprintf(sBuff,_T("%s"),_T("Newton iteration failed, no Bisection, returning:\n"));
      smos_WriteBuffer(sBuff);
      // smos_sprintf(sBuff,_T("curr T = %16.16lf, F = %16.16lf\n"),
      //     m_vCurr.m_dT, m_vCurr.m_dFOfT );
      // smos_WriteBuffer(sBuff);
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;  // Note that success does not necessarily mean that
                      // we found an answer

} // end SmLocalSolve1d::SolveIt
