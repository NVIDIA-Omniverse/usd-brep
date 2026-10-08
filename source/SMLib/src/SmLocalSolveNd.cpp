// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmLocalSolveNd.cpp
* PURPOSE: Implementation of SmLocalSolveNd methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmLocalSolveNd.h>

/*******************************************************************//**
PURPOSE: Solve an N-dimensional problem using Newton/Raphson iteration.

USER NOTES -- Find a set of parameter values to make a set of function
    values go to zero. The functions are implemented in the
    derived virtual function ClassDerivedFromSmLocalSolveNd::Evaluate().

    Local Newton/Raphson (NR) converges when 
      1. TotalErrorSize = Sum Of Function Values, drops below m_dDesiredAccuracy
      2. Distance from LastGuessPoint to NextGuessPoint drops below tolerance
          and TotalErrorSize < m_dAcceptableAccuracy

     NR fails because
      1. Initial Guess point is too far from Solution Point or
      2. Actual Solution is Out Of Bounds when given limits on parameters.
      3. Distance from LastGuessPoint to NextGuessPoint drops below tolerance
          and TotalErrorSize > tolerance
      4. If any of the nested evaluations fail

    When LocalSolve fails try again using the GlobalSolve function because
    LocalSolve can't distinguish between the various failure modes.

METHOD  --- 
  Find param values p0, p1, .. pn to make a set of equations, Fn, go to zero as
    
    FA(p0,..pn) = 0 
    . . .
    FN(p0,..pn) = 0

    The basic Newton-Raphson algorithm is modified to prevent chasing bad steps. 
    Flags within the SmLocalsSolveND object control which actions are taken.
    These preventative measures include:

    Compute next param values
    limit next guesses to stay within interval limits
    test next guess, check sum of all fabs(function values).
      when next guess is good - return a success
      when next guess step is very small and Sum of Fn is large - return a failure
      when next guess is worse than last guess and StepCutting is on,
         - shorten step and try again for next step
         - when step gets very short (< .001) negate step direction and keep trying.
         - when negative step gets very short without finding a good guess - return a failure
      when next guess is better than last guess - carry on with iteration
      when good next guess can't be found - return a failure

BASIC Newton-Raphson
           Taylor Series to first order for the Fn equations are

           0 = FA(p0+d0,..pn+dn) = FA(p0,..) + FA0(p0,..)*d0 + FA1(p0,..)*d1 + ...
           . . .
           0 = FN(p0+d0,..pn+dn) = FN(p0,..) + FN0(p0,..)*d0 + FN1(p0,..)*d1 + ...

               where FA0 = d(FA)/dp0, 
                     FA1 = d(FA)/dp1,
                     ...
                     FN0 = d(FN)/dp0
                     ...
                     FNn = d(FN)/dpn
        
           To find parameters to make all Fn zero, solve above eqns for all di.
           Rewriting the above equations in matrix form yields,

              [FA0 FA1 ... FAn]   [d0 ]    [-FA]
              [     ...       ]   [...]  = [...]
              [FN0 FN1 ... FNn]   [dn ]    [-FB]

              'jacobi matrix' * 'deltas' = 'FunRHS'
 
           update parameter values with the new di values as
           p0 += d0
           ...
           pn += dn
 
           iterate until all the FA,..,FN equations are close to zero.        

NOTES: 
***********************************************************************/
SmStatus SmLocalSolveNd::SolveIt
  (const SmTArray<double> & dGuessVector,  // in : array of guess parameters, sized:[N], N = assoc sEvalFun DOF count 
                                           //      The meaning of each parameter values depends
                                           //      on  on eval function.
                                           //      The size of this array sets the size of the problem 
   double             dAcceptableAccuracy, // in : max allowed error value.
                                           //      NR iterates until a solution good to m_dDesiredAccuracy is found
                                           //      If that is not possible the best solution is returned
                                           //      as converged if its SumOfAllErrors < dAcceptableAccuracy.
                                           //      The geometrical meaning, if any, of this tolerance
                                           //      depends on the evaluation functions implemented
                                           //      SmLocalSolveNd::SmEvalNFunctionsObject
   SmBoolean        & rbFoundSolution,     // out: TRUE=found solution,FALSE=didn't
   SmTArray<double> & rSolutionVector)     // out: array of found parameters, sized:[N], N = assoc sEvalFun DOF count
                                           //      same size as dGuessVector
{
  // init output                                         
  rbFoundSolution       = FALSE;
  m_eTerminationReason  = SM_TR_UNABLE_TO_CONVERGE; // if iteration reaches max count.

  // locals
  ULONG      ii ;
  ULONG      lNumFun    = dGuessVector.GetSize();
  double     dBestSumF  = SM_BIG_DOUBLE;
  SmBoolean  bFoundAnswer;
  SmMatrix   sJacobian (lNumFun,lNumFun);
  double     adFunRHSData[10], adSolVecData[10], adBestVecData[10], adDeltaData[10];
  SmTArray<double> sEquationValues(10,adFunRHSData, lNumFun);
  SmTArray<double> sNextGuess     (10,adSolVecData, lNumFun);
  SmTArray<double> sBestGuess     (10,adBestVecData,lNumFun);
  SmTArray<double> sStepUnclamped (10,adDeltaData,  lNumFun);
  rSolutionVector.SetSize(lNumFun);
  sEquationValues.SetSize(lNumFun);
  sNextGuess.     SetSize(lNumFun);
  sStepUnclamped. SetSize(lNumFun);

  // For Function Evaluators that want to look at the current intervals 
  if(m_rFunctionEvaluator.HasIntervals())
    {
      // store current intervals
      SER(m_rFunctionEvaluator.SetIntervals(m_cpIntervals,        // in : NULL or sized:[DOF COUNT of contained SmEvalNFunctionsObject]
                                                                  //      bounds on problem parameters.
                                                                  //      Set m_cpIntervals[i].SetUnbounded() for ivl whose prob params don't get clamped
                                            m_cpPeriodicities)) ; // in : NULL or sized:[number of eqns being solved]
                                                                  //      ivl[i] == TRUE = Interval[i] is periodic and values get wrapped
                                                                  //      ivl[i] == FALSE= Interval[i] is NOT periodic and values get clamped
    }

  // Additional stuff for checking two kinds of clamping:
  SmBoolean  bFoundAnswer2;
  SmMatrix   sJacobian2(lNumFun,lNumFun);
  double adClampedSolVecData1[10], adClampedSolVecData2[10];
  SmTArray<double> sNextGuessClamped         (10,adClampedSolVecData1, lNumFun);
  SmTArray<double> sNextGuessClampedScaled   (10,adClampedSolVecData2, lNumFun);
  sNextGuessClamped.      SetSize(lNumFun);
  sNextGuessClampedScaled.SetSize(lNumFun);
  double adFunRHSData2[10];
  SmTArray<double> sEquationValues2( 10, adFunRHSData2, lNumFun );
  sEquationValues2.SetSize(lNumFun);
  for (ULONG i=0; i<lNumFun; i++) sEquationValues2[i] = 0.0;

  // init 'this' SmLocalSolveNd member values
  m_dAcceptableAccuracy = dAcceptableAccuracy;
  m_pSolutionVector     = & rSolutionVector ;

  // Initialize things.
  for( ii=0; ii<lNumFun; ii++ )
    {
      // Let LastGuessPoint = GuessVector stored in rSolutionVector
      rSolutionVector[ii] = dGuessVector[ii];

      sEquationValues[ii] = SM_BIG_DOUBLE;
    }

  // Make 1st Evaluation - get Equation values and gradients for Guess
  //   Set bFoundAnswer == TRUE when all equations are satisfied to Tol.
  //     GWC:NOTE many derived class Evaluate functions never set bFoundAnswer to TRUE
  //       SmCCIntersectENFO ::Evaluate,
  //       SmCSIntersectENFO ::Evaluate,   but some do:                                
  //       SmCSNormalizeENFO ::Evaluate,      SmFindPSNormalENFO        ::Evaluate,         
  //       SmSSSIntersectENFO::Evaluate,      SmFilletSphereSolveENFO   ::Evaluate,         
  //       SmCCTangentIntENFO::Evaluate,      SmFindCCNormalENFO        ::Evaluate,         
  //       SmCPDistAtENFO    ::Evaluate,      SmRailRailIntersectENFO   ::Evaluate,         
  //       SmFindStepOffENFO ::Evaluate,      SmRailUVCurveIntersectENFO::Evaluate,         
  //       SmSSINormalizeENFO::Evaluate,      SmSSPIntersectENFO        ::Evaluate          
  //       SmSSNormalizeENFO ::Evaluate,
  //       SmSSSIntersectENFO::Evaluate
  if(SM_SUCCESS != m_rFunctionEvaluator.Evaluate(rSolutionVector,  // in : Current x values of F = Eqns(x)
                                                 sEquationValues,  // out: F values         of F = Eqns(x)
                                                 &sJacobian,       // out: Gradient of F =  dEqns/dX at F, ordered
                                                                   //      [dF0/dx0 dF0/dx1 ... dF0/dxn, dF1/dx0 ... dF1/dxn, ...]
                                                 bFoundAnswer))    // out: TRUE = F within tolerance, 
    { // exit case - Evaluate() call failed                        //       note: not set by many derived functions
      rbFoundSolution      = TRUE;
      m_eTerminationReason = SM_TR_UNABLE_TO_CONVERGE;
      return SM_SUCCESS;
    }

  // exit case - Evaluate() signaled good next F values (they all equal 0.0 to Tol)
  if(bFoundAnswer == TRUE) 
    {
      rbFoundSolution      = TRUE;
      m_eTerminationReason = SM_TR_FOUND_ANSWER_CONVERGED;
      return SM_SUCCESS;
    }

  // accumulate the errors
  double dSumF = 0.0;
  for ( ii=0; ii<lNumFun; ii++ )
      { dSumF += smos_Fabs(sEquationValues[ii]); }

#ifdef SM_DEBUG_CODE
static    ULONG lDebugLevel = 0 ;  // greater than 0 - print initial guess and func vals, 
                           //                        if line search is getting worse
                           //                        exit reason == SM_TR_BAD_JACOBIAN
                           //                                       SM_TR_FOUND_ANSWER_CLOSE
                           //                                       SM_TR_OUT_OF_BOUNDS
                           //                                       SM_TR_UNABLE_TO_CONVERGE 
                           //                                       SM_TR_FOUND_ANSWER_CONVERGED
                           //                        after iteration - restored best guess
                           //                                          achieved accuracy
                           // greater than 1 - print iter param steps, 
                           //                             next guess, 
                           //                             function values, SumF
                           //                             next guess after clamping, clamp scaling factor
                           //                             next function values, SumF after clamping
  TCHAR sBuffer[SM_TBLOCK_SIZE];
  if (lDebugLevel>0)
    {
      smos_WriteBuffer(_T("\nEnter SolveIt(): dGuessVector Initial guess: "));
      SM_DUMP_TARRAY(dGuessVector);
      smos_sprintf( sBuffer,_T("  Initial Function values: SumF:[%16.16lf], sEquationValues: "), dSumF );
      smos_WriteBuffer( sBuffer );
      SM_DUMP_TARRAY(sEquationValues);
    }
#endif // SM_DEBUG_CODE

  // exit case - initial guess was good enough.
  if (   dSumF < m_dDesiredAccuracy
      && m_rFunctionEvaluator.IsConverged())
    {
      rbFoundSolution      = TRUE;
      m_eTerminationReason = SM_TR_FOUND_ANSWER_CONVERGED;
      return SM_SUCCESS;
    } 

  // arrive here when 1st guess is not good enough - need to iterate to better solution

  // Remember our best solution. [070316]
  dBestSumF = dSumF;
  sBestGuess.ReSet();
  sBestGuess.Append( dGuessVector );

  // prepare iteration
  ULONG lInnerLoopCounts   = 0;      // number of NR iterations
  ULONG lBoundaryHits      = 0;      // number of boundaries hit by next guess point
  ULONG lHitAllBoundsCount = 0;      // accumulated number of next guess point boundary hits
  SmBoolean bClamped       = FALSE ; // TRUE = next guess was out of bounds and was clamped
  ULONG sData[20];
  SmTArray<ULONG> sOutOfBoundsCounts(10,sData);
  double dNextSumF=0.0;

  // Now do the iteration:
  for(m_lCurrentIteration = 0; 
      m_lCurrentIteration < m_lMaximumIterations; 
      m_lCurrentIteration++) 
    {
      // State at top of loop:
      //   Current Guess               = rSolutionVector
      //   Current F (Eqn Errs)        = sEquationValues
      //   Current Eqn Jacobi Matrix   = sJacobian
      //   Current Sum F               = dSumF

      //   Then: Solve for ParamSteps
      //          Build NextGuess 
      //          Update sEquationValues, sJacobian, dSumF 
      //          check exit criteria
      //          Set Current Guess = NextGuess and iterate         

      // negate RHS of Fn(p0,..pn) = RHS 
      for ( ii=0; ii<lNumFun; ii++ )
        {
          sEquationValues[ii] = -sEquationValues[ii];
        }

      // get NextGuess parameter steps by solving jacobi matrix eqn:
      //       [FA0 FA1 ... FAn] [d0 ]    [-FA] for the deltas
      //       [     ...       ] [...]  = [...]
      //       [FN0 FN1 ... FNn] [dn ]    [-FB]

      if(sJacobian.SolveLinearSystem(sEquationValues,sStepUnclamped) == SM_ERR) 
        {
          // exit case - jacobi matrix equation not solvable
          m_eTerminationReason = SM_TR_BAD_JACOBIAN;

          SM_DBG_MSG(lDebugLevel>0, _T("Breaking iteration: Bad Jacobian.\n") );

          // Exit NR iteration and see if we have good enough answer yet
          break; 
        } 

#ifdef SM_DEBUG_CODE
      if (lDebugLevel>1) 
        {
          smos_sprintf( sBuffer, _T("\nLoop iter %2lu: sStepUnclamped raw Param steps: "),m_lCurrentIteration);
          smos_WriteBuffer( sBuffer );
          SM_DUMP_TARRAY(sStepUnclamped);
        }
#endif // SM_DEBUG_CODE

      // Note: put a limit on step size?
      // See PT_30Oct06

      // Compute NextGuess = LastGuess + scale * Deltas
      //    - modify next-guess values when we have to.
      //      - Clamp OutOfBounds guesses
      //      - Use iteration and scaling to avoid overstepping solutions
      // loop logic
      //   While bHaveGoodStep == FALSE
      //      1. Compute NextGuess    = LastGuess + Scale * Step
      //                 NextStepSize = Sum(Fabs( Scale * Step[i])
      //      2. Exit When NextStepSize is close to zero
      //           if ErrSize < loose tol - terminateReason = AnswerClose
      //           else                     terminateReason = UnableToConverge
      //      2. Clamp NextGuess to parameter boundaries
      //      3. Make next Evaluate Call - update sEquationValues and Jacobi
      //          if(bFoundSolution == TRUE) exit with answer
      //      4. When NewSumF increases, commonly step size jumps over solution
      //           try sequence of decreasing step sizes
      //           As a last gasp - try negating step direction looking
      //           for a NextGuess that reduces NewSumF
      //      5. Check NewSumF For exit criteria

      // iterate until a NextGuess is found that reduces SumF
      // Loop summary:
      //  - apply step
      //  - check too-small step
      //  - clamp
      //  - evaluate functions
      //  - check function values
      //  - maybe cut step size

      SmBoolean bHaveGoodStep = FALSE;
      double dStepScale=1.0;
      while( !bHaveGoodStep )   // line search
        {
          lInnerLoopCounts ++;  //cbi: never checked.
          double dStepSize = 0.0;
          bHaveGoodStep    = TRUE;

          // get NextGuess and StepSize
          for(ii=0; ii<lNumFun; ii++) 
            { 
              double dThisStep  = dStepScale * sStepUnclamped[ii];
              dStepSize       += smos_Fabs(dThisStep);
              sNextGuess[ii]   = rSolutionVector[ii] + dThisStep;
            }

#ifdef SM_DEBUG_CODE
          if (lDebugLevel>1)
            {
              smos_sprintf( sBuffer,_T(" dStepScale:[%16.16lf], sNextGuess = rSolutionVector + dStepScale * sStepUnclamped: "), dStepScale );
              smos_WriteBuffer( sBuffer );
              SM_DUMP_TARRAY(sNextGuess);
            }
#endif // SM_DEBUG_CODE

          // exit condition - step size less than tight tolerance
          if(dStepSize < m_dDesiredAccuracy*m_dDesiredAccuracy) 
            {
              // success - sum of deltas and sum of Fns are small
              if (dSumF < m_dAcceptableAccuracy) 
                {
                  rbFoundSolution      = TRUE;
                  m_eTerminationReason = SM_TR_FOUND_ANSWER_CLOSE;

                  SM_DBG_MSG(lDebugLevel>0, _T("Returning: Zero step, Acceptable solution.\n") );
                  return SM_SUCCESS;
                }
              else // failure - sum of Fns is not small
                {
                  rbFoundSolution = FALSE;
                  if ( bClamped )
                    { m_eTerminationReason = SM_TR_OUT_OF_BOUNDS; }
                  else
                    { m_eTerminationReason = SM_TR_UNABLE_TO_CONVERGE; }

                  SM_DBG_MSG(lDebugLevel>0, _T("Returning: Zero step, NOT Acceptable solution.\n") );
                  return SM_SUCCESS;
                }
            } // end zero step size check

          // clamp NextGuess to parameter boundaries
          sNextGuessClamped = sNextGuess;
          if ( m_cpIntervals )
            {
              // clamp NextGuess (in NewSolutionVector) to given Intervals, returns TRUE when at least 1 parameter was clamped
              bClamped = m_cpIntervals->ClampVector(sNextGuess,             // in : Vector of double values to be clamped
                                                    m_cpPeriodicities,      // in : ivl[i] == TRUE = Interval[i] is periodic and values get wrapped
                                                                            //      ivl[i] == FALSE= Interval[i] is NOT periodic and values get clamped
                                                                            //       when ivl[i] == ivl[i]SetUnbounded() VectorToClamp[i] is never clamped or wrapped
                                                    sNextGuessClamped,      // out: The resulting clamped vector.
                                                    &sOutOfBoundsCounts);   // out: An out of bounds counter for each interval.  Note that
#ifdef SM_DEBUG_CODE                                                        //      this method increments existing counts in the vector.
              if (lDebugLevel>1 && bClamped) 
                {
                  smos_WriteBuffer(_T("  sNextGuessClamped guess clamped: "));
                  SM_DUMP_TARRAY(sNextGuessClamped);
                }
#endif // SM_DEBUG_CODE
            } // end if m_pIntervals check

          // GWC:Change - 07/19/04 
          // original behavior -   When any next guess parameter in sNextGuess
          //                       was clamped then all deltas were scaled by the same 
          //                       amount and a new next guess was recomputed. 
          //                       This kept the next iteration step in the 
          //                       same direction as the computed next guess.
          //     example:          However, if a solution happened to be on a 
          //  2 ways to modify     limit boundary, then it took a lot of iterations
          // an out-of-bounds      to get there and the limit on the number
          //   next guess.         of times a parameter can step out of bounds
          //  +-------------+      caused this function to quit before it 
          //  |             |      had a chance to converge.
          //  |            #*  x
          //  |             | / 
          //  |             |/     * = actual solution
          //  |             @      o = current guess location             
          //  |            /|      x = next guess that needs clamping.    
          //  |           o |      @ = clamped next guess when scaled    
          //  |             |      # = clamped next guess without scaling   
          //  +-------------+         
          //  NR pts of interest          
          //   on a limited        For solutions on boundaries where the next
          //   problem domain      step is trying to walk along the boundary,
          //                       clamping without scaling can converge much         
          //                       more quickly than clamping with scaling.            
          //            
          // new behavior - No scaling of non-clamped parameter values.

          // [ bd change, later ]
          // However, sometimes scaling does work better, depending on the
          // problem being solved.  For example, for dropping a point to a
          // surface, it's better not to scale, but for intersecting two
          // curves (both 2-dimensional problems), scaling does work better.
          // Therefore, look at both scaled and unscaled clamped vectors,
          // and use the better one.
          // [ 060628; also fixes Fillet regression 2:233 ]


          // Evaluate Equations for NextGuess param values
          SmBoolean bFirstFailed = FALSE;
          if(SM_SUCCESS != m_rFunctionEvaluator.Evaluate(sNextGuessClamped,
                                                         sEquationValues,
                                                         &sJacobian,
                                                         bFoundAnswer))
            {
              // exit case - evaluate call fails
              m_eTerminationReason = SM_TR_UNABLE_TO_CONVERGE;
              bFirstFailed = TRUE;
              if ( !bClamped ) // if clamped, we get to try again
                {
                  SM_DBG_MSG(lDebugLevel>0, _T("Breaking iteration: Evaluator failed\n") );
                  break; // Get out of inner loop and do a check to see if we are good enough
                }
            }

          // accumulate errors
          dNextSumF = 0.0;
          for ( ii=0; ii<lNumFun; ii++ )
            { dNextSumF += smos_Fabs( sEquationValues[ii] ) ; }

#ifdef SM_DEBUG_CODE
          if (lDebugLevel>1) 
            {
              smos_sprintf( sBuffer,_T("  Function values: next SumF:[%16.16lf], sEquationValues: "), dNextSumF );
              smos_WriteBuffer( sBuffer );
              SM_DUMP_TARRAY(sEquationValues);
            }
#endif // SM_DEBUG_CODE
          // If clamped, compare the unscaled and scaled locations.
          if ( bClamped )
            {
              // Scale the step by the most severe clamping.
              double dScaleFactor = 1.0;
              for ( ii = 0; ii < lNumFun; ii++ )
              {
                  double dBaseStep = sNextGuess[ii] - rSolutionVector[ii]; // BaseStep = dStepScale * sStepUnclamped[ii]
                  if ( smos_Fabs( dBaseStep ) > SM_EFF_ZERO )
                  {
                      double dThisScale =
                          ( sNextGuessClamped[ii] - rSolutionVector[ii] )
                              / dBaseStep;
                      if ( dThisScale < dScaleFactor )
                          dScaleFactor = dThisScale;
                  }
              }

#ifdef SM_DEBUG_CODE
              if (lDebugLevel>1) 
                {
                  smos_sprintf( sBuffer, _T("  Clamped: scaling factor dScaleFactor:[%lf]\n"), dScaleFactor );
                  smos_WriteBuffer( sBuffer );
                }
#endif // SM_DEBUG_CODE
              // Check whether the step was clamped noticeably,
              // or whether it was completely truncated.
              // Note, on the lower end, even a tiny step can make a big
              // difference in practice, so compare against 0.0.
              if ( dScaleFactor < 1.0-SM_EFF_ZERO  && dScaleFactor > 0.0 )
              {
                  // It was scaled: the clamped-and-scaled location is
                  // strictly between the previous position and the
                  // clamped-but-not-scaled location.  See which is better.

                  // Get the scaled-clamped location...
                  for ( ii = 0; ii < lNumFun; ii++ )
                  {
                      double dThisStep  = dStepScale * sStepUnclamped[ii];
                      sNextGuessClampedScaled[ii] =
                          rSolutionVector[ii] + dScaleFactor * dThisStep;
                  }

#ifdef SM_DEBUG_CODE
                      if (lDebugLevel>1) 
                        {
                          smos_WriteBuffer(_T("  sNextGuessClampedScaled guess (raw) : "));
                          SM_DUMP_TARRAY(sNextGuessClamped);
                        }
#endif // SM_DEBUG_CODE

                  // gwc tweak: fix numerical roundoff over the boundary steps by clamping again
                  if ( m_cpIntervals )
                    {
                      // clamp NextGuess (in NewSolutionVector) to given Intervals, return TRUE when at least 1 parameter was clamped
                      m_cpIntervals->ClampVector(sNextGuessClampedScaled,  // in : Vector of double values to be clamped
                                                 m_cpPeriodicities,        // in : ivl[i] == TRUE = Interval[i] is periodic and values get wrapped
                                                                           //      ivl[i] == FALSE= Interval[i] is NOT periodic and values get clamped
                                                                           //       when ivl[i] == ivl[i]SetUnbounded() VectorToClamp[i] is never clamped or wrapped
                                                 sNextGuessClampedScaled,  // out: The resulting clamped vector.
                                                 &sOutOfBoundsCounts);     // out: An out of bounds counter for each interval.  Note that
                                                                           //      this method increments existing counts in the vector.
#ifdef SM_DEBUG_CODE                                                       
                      if (lDebugLevel>1) 
                        {
                          smos_WriteBuffer(_T("  sNextGuessClampedScaled guess (2nd clamp) : "));
                          SM_DUMP_TARRAY(sNextGuessClamped);
                        }
#endif // SM_DEBUG_CODE
                   }                                                            

                  // ... evaluate there...
                  if( SM_SUCCESS != m_rFunctionEvaluator.Evaluate(sNextGuessClampedScaled,
                                                                  sEquationValues2,
                                                                  &sJacobian2,
                                                                  bFoundAnswer2 )
                     ) 
                    {
                      if ( bFirstFailed )
                      {
                          // exit case - both evaluate calls fails
                          m_eTerminationReason = SM_TR_UNABLE_TO_CONVERGE;
                          SM_DBG_MSG(lDebugLevel>0, _T("Breaking iteration: Evaluator failed, clamped.\n") );
                          break; // Get out of inner loop and do a check to see if we are good enough
                      }
                    }

                  // ... and see how good it is:
                  double dNextSumF2 = 0.0;
                  for( ii=0; ii<lNumFun; ii++ )
                      { dNextSumF2 += smos_Fabs( sEquationValues2[ii] ); }

#ifdef SM_DEBUG_CODE
                  if (lDebugLevel>1) 
                    {
                      smos_sprintf( sBuffer, _T("  scaled clamped function values: NextSumF2:[%16.16lf], sEquationValues2"), dNextSumF2 );
                      smos_WriteBuffer( sBuffer );
                      SM_DUMP_TARRAY(sEquationValues2);
                    }
#endif // SM_DEBUG_CODE

                  // Replace with the scaled step if it's better.
                  // (Also, if first failed and second succeeded,
                  // just use the second one.)
                  if ( dNextSumF2 < dNextSumF  ||  bFirstFailed )
                    {
                      sNextGuessClamped = sNextGuessClampedScaled;
                      sEquationValues = sEquationValues2;
                      dNextSumF = dNextSumF2;
                    }
                }  // end if scaling was tested
            }  // end if bClamped.

          // Record the better clamped value
          //  need to do this with or without clamping because an Evaluate() Function
          //  can change the sNextGuessClamped value.
          sNextGuess = sNextGuessClamped;

          // State:
          //   sNextGuess:
          //   sEquationValues:
          //   dNextSumF
          // if clamped, these are the better of scaled and unscaled.

          // exit case - Evaluate call signals converged solution,
          //   or we say it's ok because of converged function values.
          // Note: most Evaluate() functions never set bFoundAnswer == TRUE

          // If small enough, call it good.
          bFoundAnswer |= ( dNextSumF < m_dDesiredAccuracy );

          if ( bFoundAnswer )
            {
              // save NextGuess as Solution and exit
              rbFoundSolution      = TRUE;
              m_eTerminationReason = SM_TR_FOUND_ANSWER_CONVERGED;
              for ( ii=0; ii<lNumFun; ii++ )
                { rSolutionVector[ii] = sNextGuess[ii]; }

#ifdef SM_DEBUG_CODE
              if (lDebugLevel>0) 
                {
                  smos_sprintf( sBuffer, _T("  Converged, iteration:[%2lu]\n"), m_lCurrentIteration );
                  smos_WriteBuffer( sBuffer );
                  smos_sprintf(sBuffer,_T("%s"),_T("    Solution:\n") );
                  smos_WriteBuffer(sBuffer);
                  SM_DUMP_TARRAY(sNextGuess);
                  smos_sprintf( sBuffer, _T("    Function values: next SumF:[%16.16lf]:\n "), dNextSumF );
                  smos_WriteBuffer(sBuffer);
                  SM_DUMP_TARRAY(sEquationValues);
                  smos_sprintf(sBuffer,_T("%s"), _T("    Solution:\n") );
                  smos_WriteBuffer(sBuffer);
                  SM_DUMP_TARRAY(sNextGuess);
                  smos_sprintf( sBuffer, _T("    Function values: next SumF:[%16.16lf]:\n "), dNextSumF );
                  smos_WriteBuffer(sBuffer);
                  SM_DUMP_TARRAY(sEquationValues);
                  smos_WriteBuffer(_T("\n"));
                }
#endif // SM_DEBUG_CODE
              return SM_SUCCESS;

            } // end if(bFoundAnswer)

          // When NextSumF increases - commonly, solution is overstepped,
          //   try smaller steps until step size is near zero
          //     or NewSumF decreases.
          //   As a last ditch effort - try opposite direction NextGuess steps
          //      to reduce NewSumF values

          if ( dNextSumF >= dSumF && m_bStepCuttingEnabled )
            {
              if (    dSumF > m_dAcceptableAccuracy
                   && smos_Fabs( dStepScale ) > 0.0001 )
                {
                  dStepScale /= 10.0;
                  // If we are failing miserably then perhaps we have a wrong
                  // sign for one of the deltas - try negating scale and try 
                  // again from the top.
                  if(   dStepScale > 0.0 
                     && dStepScale < 0.001) 
                    { 
                      dStepScale = - 1.0;
                    }
                  bHaveGoodStep = FALSE;

#ifdef SM_DEBUG_CODE
                  if (lDebugLevel>0) 
                    {
                      smos_sprintf( sBuffer, _T("  Line search: getting worse, cut step, dStepScale:[%16.16lf]\n"), dStepScale );
                      smos_WriteBuffer( sBuffer );
                      smos_WriteBuffer(_T("\n"));
                    }
#endif // SM_DEBUG_CODE
                }
            } // end if step-cutting

        } // while ! good deltas  (line search)

      // arrive here after picking NextGuess and recomputing EquationValues and dNextSumF
      //   (A search was made to try and find a NextGuess that reduced dNextSumF)

      // exit when NextSumF > LastSumF after 2 iterations 
      //      but LastSumF is good enough.
      // This commonly happens after convergence where errors vary by
      //  machine precision and NextSumF changes randomly based on 
      //  bit round-off
      if (   m_lCurrentIteration > 2
          && dNextSumF  >= dSumF 
          && dSumF      < m_dAcceptableAccuracy/100.0 )
        {
          m_eTerminationReason = SM_TR_FOUND_ANSWER_CLOSE;  // SM_TR_UNABLE_TO_CONVERGE;
          SM_DBG_MSG(lDebugLevel>0, _T("Breaking iteration: Functions increasing, and Acceptable.\n") );
          break; // Get out of outer loop and do a check to see if we are good enough
        }

      // exit when NextSumF is not changing after 2 iterations
      //      and dSumF might NOT be good enough 
      double dChangeFactor = 0.000001;
      if (   m_lCurrentIteration > 2
          && smos_Fabs(dNextSumF-dSumF) < dSumF * dChangeFactor
          && dSumF > m_dAcceptableAccuracy/100.0 
         )
        {
          m_eTerminationReason = SM_TR_UNABLE_TO_CONVERGE;
          SM_DBG_MSG(lDebugLevel>0, _T("Breaking iteration: Functions unchanging, and possibly not Acceptable.\n") );
          break; // Get out of outer loop and do a check to see if we are good enough
        }

      // Remember our best solution. [070123]
      if ( dNextSumF < dBestSumF )
        {
          dBestSumF = dNextSumF;
          sBestGuess.ReSet();
          sBestGuess.Append( sNextGuess );
        }

      // save last solution as best solution when
      //      we can't find a better next solution
      //      and last solution is best-seen good-enough solution 
      if ( dNextSumF > dSumF  &&  dSumF < m_dAcceptableAccuracy )
        {
          if ( dSumF < dBestSumF )
            {
              dBestSumF = dSumF;
              sBestGuess.ReSet();
              sBestGuess.Append(rSolutionVector);
            }
        }

      // set up for next iter - set dSumF and CurrentGuess
      dSumF = dNextSumF;

      // set CurrentGuess = NextGuess
      double dStepSize = 0.0;
      for ( ii=0; ii<lNumFun; ii++ )
        {
          dStepSize          += smos_Fabs(rSolutionVector[ii]-sNextGuess[ii]);
          rSolutionVector[ii] = sNextGuess[ii];
        }

      // exit when StepSize is close to zero - fail
      if (dStepSize < SM_EFF_ZERO_SQ) 
        {
          m_eTerminationReason = SM_TR_UNABLE_TO_CONVERGE;
          SM_DBG_MSG(lDebugLevel>0, _T("Breaking iteration: Zero Stepsize.\n") );
          break; // Get out of outer loop and do a check to see if we are good enough
        }

      // when GuessPoint OutOfBounds behavior is an exit criteria 
      //   - count total number of boundary hits
      //   - when total hits exceeds limit 
      //        - exit NR iteration to pick best solution to return
      if(   m_eBoundaryHandler == SM_BH_TOTAL_BOUNDARY_HITS
         || m_eBoundaryHandler == SM_BH_HITS_ALL_BOUNDARIES) 
        {
          SmBoolean bHitsAllBoundaries = TRUE;

          // count the number of parameters stepping out of bounds
          for(ii=0; ii<sOutOfBoundsCounts.GetSize(); ii++) 
            {
              // when this param goes out of bounds
              if(sOutOfBoundsCounts[ii] != 0) 
                {
                  // clear the counter
                  sOutOfBoundsCounts[ii] = 0;  

                  // count hit only when this parameter's step is 'large'
                  // compared to its range
                  // - under counting hits causes more iterations
                  // - over counting hits may cause an exit before an
                  //   existing solution is found
                  // so - pick a relatively loose definition of 'large',
                  // currently .001 of span.
                  SmExtent1d sExt( (*m_cpIntervals)[ii] );
                  if ( smos_Fabs( sStepUnclamped[ii] )
                          > 1.0e-3 * sExt.GetLength() )
                    {
                      lBoundaryHits ++;
                    }
                }  // end out of bounds check
              else { // remember if any boundaries were not exceeded
                     bHitsAllBoundaries = FALSE;
                     if ( m_eBoundaryHandler == SM_BH_HITS_ALL_BOUNDARIES )
                       { break; }
                   } // end missed a boundary check
            } // end counting OutOfBounds parameter

          // count times all param values go OutOfBounds simultaneously 
          if(   bHitsAllBoundaries == TRUE
             && sOutOfBoundsCounts.GetSize() > 0) 
            { lHitAllBoundsCount ++; }

          // exit criteria - when we have stepped OutOfBounds repeatedly
          if(   (   m_eBoundaryHandler == SM_BH_TOTAL_BOUNDARY_HITS
                 && lBoundaryHits       > m_lMaxBoundaryHits)
             || (   m_eBoundaryHandler == SM_BH_HITS_ALL_BOUNDARIES
                 && lHitAllBoundsCount  > 2)) 
            {
              m_eTerminationReason = SM_TR_OUT_OF_BOUNDS;

              // make last guess the solution 
              for ( ii=0; ii<lNumFun; ii++ )
                  { rSolutionVector[ii] = sNextGuess[ii]; }
              
              // Exit NR iteration and see if we have good enough answer yet
              SM_DBG_MSG(lDebugLevel>0, _T("Breaking iteration: Boundary hits.\n") );
              break ;
            } // end boundaryHits exceeds max count check
        } // end NextGuess OutOfBounds exit criteria check

    } // end NR iteration

  // use best solution seen
  if (dSumF > dBestSumF && fabs(dBestSumF) <SM_BIG_DOUBLE) 
    {
      SM_DBG_MSG(lDebugLevel>0, _T("  After iteration, restoring dBestGuess.\n") );
      dSumF = dBestSumF;
      rSolutionVector.ReSet();
      rSolutionVector.Append(sBestGuess);
      m_rFunctionEvaluator.Evaluate(rSolutionVector,sEquationValues,&sJacobian,bFoundAnswer);
    }

  // return converged case
  if(dSumF < m_dDesiredAccuracy)
    { 
      rbFoundSolution      = TRUE;
      m_eTerminationReason = SM_TR_FOUND_ANSWER_CONVERGED;

      SM_DBG_MSG(lDebugLevel>0, _T("  After iteration, achieved Desired accuracy.\n") );
      return SM_SUCCESS;
    }

  // return nearly converged case
  if(dSumF < m_dAcceptableAccuracy) 
    {
      rbFoundSolution      = TRUE;
      m_eTerminationReason = SM_TR_FOUND_ANSWER_CLOSE;

      SM_DBG_MSG(lDebugLevel>0, _T("  After iteration, achieved Acceptable accuracy only.\n") );
      return SM_SUCCESS;
    }

  // failed to converge
  // Don't reset this, it was already set appropriately. [B234]
  //   m_eTerminationReason = SM_TR_UNABLE_TO_CONVERGE;

  SM_DBG_MSG( (lDebugLevel>0), _T("  After iteration, Failed to converge.\n") );
  return SM_SUCCESS;

} // end SmLocalSolveNd::SolveIt

/*******************************************************************//**
PURPOSE: This method computes the Jacobian matrix by finite
    differences.  It just tweaks each parameter and looks for the effect.

NOTES: Uses central differences, although that can turn into
   forward or backwards finite differences due to clamping to the domains.

GWC:TODO - Come back and use 2nd order finite differences here.
           The 3 needed solutions are already being called.
***********************************************************************/
SmStatus SmEvalNFunctionsObject::ComputeJacobian
  (SmTArray<double>       & crX,          // in : current parameter values
   const SmTArray<double> & rF,           // in : current equation values
   SmMatrix               & rJacobian,    // out: jacobian finite differences
   SmTArray<SmExtent1d>   * pOptIntervals) //in : optional domains (intervals)
{
  // Temporary evaluation arrays for low and high evaluation.
  double sDataLo[16];
  SmTArray<double> sFTmpLo(16,sDataLo);
  sFTmpLo.SetSize( rF.GetSize() );
  double sDataHi[16];
  SmTArray<double> sFTmpHi(16,sDataHi);
  sFTmpHi.SetSize( rF.GetSize() );

  double dLoT, dHiT;
  SmBoolean bFoundAnswer;

  // Loop over each parameter value.
  ULONG jj, ii, lNumEqns = rF.GetSize();
  for ( jj=0; jj<lNumEqns; jj++ )
    {
      double dBaseParam = crX[jj];
      double dDelta = SM_EFF_ZERO_SQRT * (1.0 + smos_Fabs(dBaseParam));
      // can't happen:  if (smos_Fabs(dDelta) < SM_EFF_ZERO_SQRT/10.0) { dDelta=SM_EFF_ZERO_SQRT; }

      // increment the jjth parameter
      dLoT = dBaseParam - dDelta;
      dHiT = dBaseParam + dDelta;

      // Clamp to the domain if present.
      if ( pOptIntervals != NULL )
        {
          dLoT = (*pOptIntervals)[jj].ClampValue( dLoT );
          dHiT = (*pOptIntervals)[jj].ClampValue( dHiT );
        }

      // evaluate all equations at Low T and High T -- no Jacobian.
      crX[jj] = dLoT;
      SER( Evaluate( crX, sFTmpLo, NULL, bFoundAnswer ));

      crX[jj] = dHiT;
      SER( Evaluate( crX, sFTmpHi, NULL, bFoundAnswer ));

      dDelta = dHiT - dLoT;  // use the actual difference, whatever happened.

      if ( smos_Fabs(dDelta) < SM_EFF_ZERO )
        { SER( SM_ERR ); } // We must fail.  Could try bigger steps?

      // Compute jjth Jacobian row by 1st order central finite differences.
      for ( ii=0; ii<lNumEqns; ii++ )
        {
          rJacobian[ii][jj] = ( sFTmpHi[ii] - sFTmpLo[ii] ) / dDelta;
        }

      // restore original value
      crX[jj] = dBaseParam;
    }

  // all done
  return SM_SUCCESS;

} // end SmEvalNFunctionsObject::ComputeJacobian

//      /**********************************************************************//**
//      * FILE NAME --- SmLocalSolveNd.cpp
//      * PURPOSE: Implementation of SmLocalSolveNd methods.
//      *
//      **********************************************************************/
//      
//      #include <SmLocalSolveNd.h>
//      
//      /*******************************************************************//**
//      PURPOSE: Solve an N-dimensional problem using Newton/Raphson iteration.
//      
//      USER NOTES -- Find a set of parameter values to make a set of function
//          values go to zero. The functions are implemented in the
//          derived virtual function ClassDerivedFromSmLocalSolveNd::Evaluate().
//      
//          Local Newton/Raphson (NR) converges when 
//            1. TotalErrorSize = Sum Of Function Values, drops below m_dDesiredAccuracy
//            2. Distance from LastGuessPoint to NextGuessPoint drops below tolerance
//                and TotalErrorSize < m_dAcceptableAccuracy
//      
//           NR fails because
//            1. Initial Guess point is too far from Solution Point or
//            2. Actual Solution is Out Of Bounds when given limits on parameters.
//            3. Distance from LastGuessPoint to NextGuessPoint drops below tolerance
//                and TotalErrorSize > tolerance
//            4. If any of the nested evaluations fail
//      
//          When LocalSolve fails try again using the GlobalSolve function because
//          LocalSolve can't distinguish between the various failure modes.
//      
//      METHOD  --- 
//        Find param values p0, p1, .. pn to make a set of equations, Fn, go to zero as
//          
//          FA(p0,..pn) = 0 
//          . . .
//          FN(p0,..pn) = 0
//      
//          The basic Newton-Raphson algorithm is modified to prevent chasing bad steps. 
//          Flags within the SmLocalsSolveND object control which actions are taken.
//          These preventative measures include:
//      
//          Compute next param values
//          limit next guesses to stay within interval limits
//          test next guess, check sum of all fabs(function values).
//            when next guess is good - return a success
//            when next guess step is very small and Sum of Fn is large - return a failure
//            when next guess is worse than last guess and StepCutting is on,
//               - shorten step and try again for next step
//               - when step gets very short (< .001) negate step direction and keep trying.
//               - when negative step gets very short without finding a good guess - return a failure
//            when next guess is better than last guess - carry on with iteration
//            when good next guess can't be found - return a failure
//      
//      BASIC Newton-Raphson
//                 Taylor Series to first order for the Fn equations are
//      
//                 0 = FA(p0+d0,..pn+dn) = FA(p0,..) + FA0(p0,..)*d0 + FA1(p0,..)*d1 + ...
//                 . . .
//                 0 = FN(p0+d0,..pn+dn) = FN(p0,..) + FN0(p0,..)*d0 + FN1(p0,..)*d1 + ...
//      
//                     where FA0 = d(FA)/dp0, 
//                           FA1 = d(FA)/dp1,
//                           ...
//                           FN0 = d(FN)/dp0
//                           ...
//                           FNn = d(FN)/dpn
//              
//                 To find parameters to make all Fn zero, solve above eqns for all di.
//                 Rewriting the above equations in matrix form yields,
//      
//                    [FA0 FA1 ... FAn]   [d0 ]    [-FA]
//                    [     ...       ]   [...]  = [...]
//                    [FN0 FN1 ... FNn]   [dn ]    [-FB]
//      
//                    'jacobi matrix' * 'deltas' = 'FunRHS'
//       
//                 update parameter values with the new di values as
//                 p0 += d0
//                 ...
//                 pn += dn
//       
//                 iterate until all the FA,..,FN equations are close to zero.        
//      
//      NOTES: 
//      ***********************************************************************/
//      SmStatus SmLocalSolveNd::SolveIt
//        (const SmTArray<double> & dGuessVector,  // in : array of guess parameters - 
//                                                 //      The meaning of each parameter values depends
//                                                 //      on  on eval function
//                                                 //      the size of this array sets the size of the problem 
//         double             dAcceptableAccuracy, // in : max allowed error value
//         SmBoolean        & rbFoundSolution,     // out: TRUE=found solution,FALSE=didn't
//         SmTArray<double> & rSolutionVector)     // out: array of found parameters
//                                                 //      same size as dGuessVector
//      {                                          
//        // init output
//        ULONG lNumFun   = dGuessVector.GetSize();
//        rbFoundSolution = FALSE;
//        rSolutionVector.SetSize(lNumFun);
//      
//        // locals
//        m_dAcceptableAccuracy   = dAcceptableAccuracy;
//        m_pSolutionVector       = & rSolutionVector ;
//        double    dBestSolution = SM_BIG_DOUBLE;
//        SmMatrix  sJacobian(lNumFun,lNumFun);
//        SmBoolean bFoundAnswer;
//        ULONG     ii ;
//      
//        // local arrays - sized to dGuessVector size
//        double adFunRHSData[10], adSolVecData[10], adBestVecData[10], adDeltaData[10];
//        ULONG  sData[10];
//        SmTArray<double> sFunRHS            (10,adFunRHSData);  sFunRHS.SetSize            (lNumFun);
//        SmTArray<double> sNewSolutionVector (10,adSolVecData);  sNewSolutionVector.SetSize (lNumFun);
//        SmTArray<double> sBestSolutionVector(10,adBestVecData); sBestSolutionVector.SetSize(lNumFun);
//        SmTArray<double> sDeltas            (10,adDeltaData);   sDeltas.SetSize            (lNumFun);
//        SmTArray<ULONG>  sOutOfBoundsCounts (10,sData);
//      
//        // let current solution = guess
//        for(ii=0; ii<lNumFun; ii++) { rSolutionVector[ii] = dGuessVector[ii] ; }
//      
//        // Get CurrentSoluiton Function Values and Jacobi Matrix
//        //   1. bFoundAnswer = TRUE when all equations are satisfied.
//        //   2. GWC:NOTE Many derived SmEvalNFunctionsObject Evaluate() implementations
//        //      don't set bFoundAnswer in which case its always set to FALSE.
//        if(SM_SUCCESS != m_rFunctionEvaluator.Evaluate(rSolutionVector, 
//                                                       sFunRHS, &sJacobian, bFoundAnswer)) 
//          {
//            rbFoundSolution      = TRUE ;
//            m_eTerminationReason = SM_TR_UNABLE_TO_CONVERGE;
//            return SM_SUCCESS;
//          }
//      
//        // dTotalErrSize = Sum of current errors
//        double dTotalErrSize = 0.0;
//        for (ii=0; ii<lNumFun; ii++) { dTotalErrSize += smos_Fabs(sFunRHS[ii]) ; }
//      
//        // Stop if GuessPoint is good enough
//        // Stop when sum of all errors for given guess is less then tight tolerance
//        if (   bFoundAnswer
//            || dTotalErrSize < m_dDesiredAccuracy) 
//          {
//            rbFoundSolution      = TRUE;
//            m_eTerminationReason = SM_TR_FOUND_ANSWER_CONVERGED;
//            return SM_SUCCESS;
//          } // end tight tolerance exit check
//      
//        // GWC:NOTE many derived class Evaluate functions do not set bFoundAnswer
//        //    SmCCIntersectENFO ::Evaluate,      and some that do:                       
//        //    SmCSIntersectENFO ::Evaluate,         SmFindPSNormalENFO        ::Evaluate,
//        //    SmCSNormalizeENFO ::Evaluate,         SmFilletSphereSolveENFO   ::Evaluate,
//        //    SmSSSIntersectENFO::Evaluate,         SmFindCCNormalENFO        ::Evaluate,
//        //    SmCCTangentIntENFO::Evaluate,         SmRailRailIntersectENFO   ::Evaluate,
//        //    SmCPDistAtENFO    ::Evaluate,         SmRailUVCurveIntersectENFO::Evaluate,
//        //    SmFindStepOffENFO ::Evaluate,         SmSSPIntersectENFO        ::Evaluate 
//        //    SmSSINormalizeENFO::Evaluate,
//        //    SmSSNormalizeENFO ::Evaluate,
//        //    SmSSSIntersectENFO::Evaluate
//      
//        // iteration parameter OutOfBounds counts
//        ULONG lBoundaryHits      = 0;  // number of parameters stepped out of bounds on a large step
//        ULONG lAllBoundaryHits   = 0;  // number of parameters stepped out of bounds on any size step
//        ULONG lInnerLoopCounts   = 0;  // count number of times parameters are stepped
//        ULONG lHitAllBoundsCount = 0;  // number of times all parameters stepped out of bounds in one iteration
//        SmBoolean bClamped       = FALSE ;
//      
//        // for up to a max number of iterations
//        for(m_lCurrentIteration=0 ; 
//            m_lCurrentIteration < m_lMaximumIterations ; 
//            m_lCurrentIteration++) 
//          {
//            double dNewErrSize = 0.0;
//      
//            // get NextStep from CurrentGuessPoint to NextGuessPoint
//            //        [FA0 FA1 ... FAn][d0 ]   [-FA] 
//            // Solve  [        ...    ][...] = [...] for the deltas.
//            //        [FN0 FN1 ... FNn][dn ]   [-FB]
//            for(ii=0; ii<lNumFun; ii++) { sFunRHS[ii] = -sFunRHS[ii] ; }
//            if(sJacobian.SolveLinearSystem(sFunRHS,sDeltas) == SM_ERR) 
//              {
//                m_eTerminationReason = SM_TR_BAD_JACOBIAN;
//                break; // Get out of loop and see if we have good enough answer yet.
//              } // end indeterminate jacobi matrix check
//      
//            // Get NextGuessPoint, ReEvaluate, and get NextErrSize
//            //   1. NextGuessPoint = CurrentGuessPoint + deltas, STOP when StepSize drops to zero
//            //   2. Clamp(NextGuessPoint) 
//            //   3. Evaluate Functions and Jacobi Matrix, STOP when evaluateFunctions signal success
//            //   4. If m_bStepCuttingEnabled and NextErrorSize > LastErrorSize 
//            //        iterate while changing step size (not direction) until a better guess is found
//            SmBoolean bHaveGoodDeltas = FALSE;
//            SmBoolean bDoStepCut      = TRUE ; // flag used to end StepCutting
//            double    dDeltaScale     = 1.0 ;  // when StepCutting, amount StepSizes are changed while searching for a good guess
//            while(!bHaveGoodDeltas)            // when StepCutting, until a step that yields a small TotalErrSize is found
//              {
//                lInnerLoopCounts ++;
//                bHaveGoodDeltas       = TRUE;
//                double dTotalStepSize = 0.0;    // Sum of all step sizes
//      
//                // 1. Get NextGuessPoint = LastGuessPoint + Deltas and TotalStepSize
//                for(ULONG k=0; k<lNumFun; k++) 
//                  {
//                    // let NewParams = CurrentParam + DeltaScale * Delta 
//                    double dThisStep       = dDeltaScale * sDeltas[k];
//                    dTotalStepSize        += smos_Fabs(dThisStep);
//                    sNewSolutionVector[k]  = rSolutionVector[k] + dThisStep;
//                  }
//      
//                // Stop when TotalStepSize drops to NearZero
//                if(dTotalStepSize < m_dDesiredAccuracy*m_dDesiredAccuracy) 
//                  {
//                    // success or failure depends on TotalErrSize
//                    if (dTotalErrSize < m_dAcceptableAccuracy) 
//                      { rbFoundSolution      = TRUE;
//                        m_eTerminationReason = SM_TR_FOUND_ANSWER_CLOSE;
//                      }
//                    else                                       
//                      { rbFoundSolution      = FALSE;
//                        m_eTerminationReason = SM_TR_UNABLE_TO_CONVERGE;
//                     }
//                    return SM_SUCCESS;
//                  }
//      
//                // 2. Clamp NextGuessPoint when given parameter intervals
//                if(m_cpIntervals) 
//                  {
//                    // clamp NewSolutionVector to given Intervals
//                    //   sOutOfBoundsCounts[ii] set to 1 when ith Param had to be clamped  
//                    bClamped = m_cpIntervals->ClampVector(sNewSolutionVector,   // in : Vector of double values to be clamped
//                                                          m_cpPeriodicities,    // in : ivl[i] == TRUE = Interval[i] is periodic and values get wrapped
//                                                                                //      ivl[i] == FALSE= Interval[i] is NOT periodic and values get clamped
//                                                                                //       when ivl[i] == ivl[i]SetUnbounded() VectorToClamp[i] is never clamped or wrapped
//                                                          sNewSolutionVector,   // out: The resulting clamped vector.
//                                                          &sOutOfBoundsCounts); // out: An out of bounds counter for each interval.  Note that
//                                                                                //      this method increments existing counts in the vector.
//                    // GWC:Change - 07/19/04 
//                    // original behavior -   When any next guess parameter in sNewSolutionVector
//                    //                       was clamped then all deltas were scaled by the same 
//                    //                       amount and a new next guess was recomputed. 
//                    //                       This kept the next iteration step in the 
//                    //                       same direction as the computed next guess.
//                    //     example:          However, if a solution happened to be on a 
//                    //  2 ways to modify     limit boundary, then it took a lot of iterations
//                    // an out-of-bounds      to get there and the limit on the number
//                    //   next guess.         of times a parameter can step out of bounds
//                    //  +-------------+      caused this function to quit before it 
//                    //  |             |      had a chance to converge.
//                    //  |            #*  x
//                    //  |             | / 
//                    //  |             |/     * = actual solution
//                    //  |             @      o = current guess location             
//                    //  |            /|      x = next guess that needs clamping.    
//                    //  |           o |      @ = clamped next guess when scaled    
//                    //  |             |      # = clamped next guess without scaling   
//                    //  +-------------+         
//                    //  NR pts of interest          
//                    //   on a limited        For solutions on boundaries where the next                                          
//                    //   problem domain      step is trying to walk along the boundary,
//                    //                       clamping without scaling can converge much         
//                    //                       more quickly than clamping with scaling.            
//                    //            
//                    // new behavior - No scaling of non-clamped parameter values.
//      
//                  } // end if m_pIntervals check
//      
//                // 3. ReEvaluate Functions and Jacobi Matrix for NextGuessPoint 
//                if(m_rFunctionEvaluator.Evaluate(sNewSolutionVector,
//                                                 sFunRHS, &sJacobian, bFoundAnswer)
//                   != SM_SUCCESS) 
//                  {
//                    m_eTerminationReason = SM_TR_UNABLE_TO_CONVERGE;
//                    dNewErrSize          = SM_BIG_DOUBLE;
//                    break; // Get out of loop and do a check to see if we are good enough
//                  }
//      
//                // Stop when evaluator bothered to signal a successful convergence (not all do)
//                if(bFoundAnswer) 
//                  {
//                    rbFoundSolution      = TRUE;
//                    m_eTerminationReason = SM_TR_FOUND_ANSWER_CONVERGED;
//                    for(ii=0; ii<lNumFun; ii++) { rSolutionVector[ii] = sNewSolutionVector[ii]; }
//                    return SM_SUCCESS;
//                  }
//      
//                // get NewErrSize
//                dNewErrSize = 0.0;
//                for(ii=0; ii<lNumFun; ii++) { dNewErrSize += smos_Fabs(sFunRHS[ii]) ; }
//      
//                // 4. When m_bStepCuttingEnabled == TRUE and TotalErrSize has increased.
//                //    Try to find a getter NextGuessPoint.
//                if(   m_bStepCuttingEnabled == TRUE  
//                   && dTotalErrSize >  m_dAcceptableAccuracy
//                   && dNewErrSize   >= dTotalErrSize
//                   && bDoStepCut)
//                  {
//                    //  An Increased TotalErrSize is commonly caused by stepping the  
//                    //  parameters over the answer by a long distance or in the wrong direction.
//      
//                    // 1st try shrinking the step size - then negate - then give up and go back to 1st guess
//                    if(dDeltaScale > 0.0015 )  { dDeltaScale /= 10.0; }
//                    else if(dDeltaScale > 0.0) { dDeltaScale = -1.0;  }
//                    else                       { dDeltaScale =  1.0;  
//                                                 bDoStepCut  = FALSE; 
//                                               }
//                    bHaveGoodDeltas = FALSE;
//                  } // end StepCutting with bad NextGuessPoint check
//              } // end Get NextGuessPoint iteration (1 Pass for m_bStepCuttingEnabled == FALSE)
//      
//            // arrive here with NextGuessPoint, a ReEvaluation, and a dNewErrSize 
//      
//            // when dNewErrSize is worse than current good solution
//            // - break to check to see if we are good enough.
//            // This happens after convergence where errors are now set by machine precision
//            // and the value of the solution will change randomly based on bit round-off
//            if (   dNewErrSize >= dTotalErrSize 
//                && dTotalErrSize < m_dAcceptableAccuracy/100.0 
//                && m_lCurrentIteration > 2) 
//              {
//                m_eTerminationReason = SM_TR_UNABLE_TO_CONVERGE;
//                break; // Get out of loop and do a check to see if we are good enough
//              }
//      
//            // When last solve was better than this solve and it was good enough - save last solve         
//            if (   dTotalErrSize < dNewErrSize 
//                && dTotalErrSize < dBestSolution 
//                && dTotalErrSize < m_dAcceptableAccuracy) 
//              {
//                dBestSolution = dTotalErrSize;
//                sBestSolutionVector.ReSet();
//                sBestSolutionVector.Append(rSolutionVector);
//              }
//      
//            // set up for next iter - Move NextValues into LastValue slots
//            double dSolutionChangeSize = 0.0;
//            dTotalErrSize              = dNewErrSize;
//            for (ii=0; ii<lNumFun; ii++) 
//              {
//                dSolutionChangeSize += smos_Fabs(rSolutionVector[ii]-sNewSolutionVector[ii]);
//                rSolutionVector[ii] = sNewSolutionVector[ii];
//              }
//      
//            // STOP when Change in ErrSize  drops to NearZero and current solution is unacceptable
//            //      or   SolutionStepSize   drops to NearZero
//            if(   (   m_lCurrentIteration > 2
//                   && SM_IS_ZERO(dNewErrSize-dTotalErrSize) 
//                   && dTotalErrSize > m_dAcceptableAccuracy/100.0)
//               || (   dSolutionChangeSize < SM_EFF_ZERO_SQ))
//              {
//                // may arrive here when out of bounds by a small amount
//                m_eTerminationReason =   (bClamped)
//                                       ? SM_TR_OUT_OF_BOUNDS
//                                       : SM_TR_UNABLE_TO_CONVERGE ;
//                break; // Get out of loop and do a check to see if we are good enough
//              }
//      
//            // When using BoundaryHandler and GuessPoint was clamped
//            if(   m_eBoundaryHandler != SM_BH_IGNORE_BOUNDARY_HITS
//               && bClamped) 
//              {
//                // count the number of parameters stepping out of bounds
//                ULONG lHits = 0 ;
//                for(ii=0; ii<sOutOfBoundsCounts.GetSize(); ii++) 
//                  {
//                    // when ith param was stepped OutOfBounds
//                    if(sOutOfBoundsCounts[ii] != 0) 
//                      {
//                        // clear OutOfBounds flag
//                        sOutOfBoundsCounts[ii] = 0;  
//                        
//                        // count hits in various ways
//                        lAllBoundaryHits ++;  // all boundary hits
//                        lHits ++ ;            // all boundary hits this iteration
//      
//                        // all boundary hits associated with a large parameter step
//                        //  under counting hits causes more iterations
//                        //  over  counting hits may cause an exit before an existing solution is found
//                        // so - pick a relatively loose definition of 'large', currently .001 of span.
//                        if (sDeltas[ii] > 1.0e-3 * (*m_cpIntervals)[ii].GetLength()) 
//                          { lBoundaryHits ++; }  
//                      }  // end parameter out of bounds check
//                  } // end iter every out of bounds parameter
//      
//                // when all params stepped out of bounds this iteration
//                if(lHits == sOutOfBoundsCounts.GetSize()) { lHitAllBoundsCount ++; }
//      
//                // Stop after params have repeatedly stepped OutOfBounds 
//                if(   (   m_eBoundaryHandler == SM_BH_TOTAL_BOUNDARY_HITS
//                       && (   lBoundaryHits    > m_lMaxBoundaryHits
//                           || lAllBoundaryHits > 2 * m_lMaxBoundaryHits))
//                   || (   m_eBoundaryHandler == SM_BH_HITS_ALL_BOUNDARIES
//                       && lHitAllBoundsCount   > 2)) 
//                  {
//                    m_eTerminationReason = SM_TR_OUT_OF_BOUNDS;
//                    break ; // Get out of loop and see if we have good enough answer yet.
//      
//                  } // end boundaryHits exceeds Stop criteria check
//              } // end BoundaryHandler is on check
//          } // end NR iteration
//      
//        // arrive here after one of the Stop conditions has broken out of the NR iteration
//      
//        // use best solution seen
//        if (dTotalErrSize > dBestSolution) 
//          {
//            dTotalErrSize = dBestSolution;
//            rSolutionVector.ReSet();
//            rSolutionVector.Append(sBestSolutionVector);
//      #ifdef SM_DEBUG_CODE
//            m_rFunctionEvaluator.Evaluate(rSolutionVector,sFunRHS,&sJacobian,bFoundAnswer) ;
//      #endif                      
//          }
//      
//        // return converged case, nearly converged, or failed to converge cases
//        if     (dTotalErrSize < m_dDesiredAccuracy)    { rbFoundSolution      = TRUE;
//                                                         m_eTerminationReason = SM_TR_FOUND_ANSWER_CONVERGED;
//                                                       }
//        else if(dTotalErrSize < m_dAcceptableAccuracy) { rbFoundSolution      = TRUE;
//                                                         m_eTerminationReason = SM_TR_FOUND_ANSWER_CLOSE;
//                                                       }
//        else                                           { rbFoundSolution      = FALSE;
//                                                         m_eTerminationReason = SM_TR_UNABLE_TO_CONVERGE;
//                                                       }
//        // gwc:note - Do not return SM_TR_OUT_OF_BOUNDS from this local solve
//        //            Current design is to run GlobalSolve to get answers for such cases.
//        // all done
//        return SM_SUCCESS;
//      
//      } // end SmLocalSolveNd::SolveIt
//      
//      /*******************************************************************//**
//      PURPOSE: This method computes the Jacobian matrix by finite
//          differences.  It just tweaks each parameter and looks for the effect.
//      
//      NOTES: Stores the largest value of computing the
//         derivatives with forward and bacwards finite differences
//      
//      GWC:TODO - Come back and use 2nd order finite differences here.
//                 The 3 needed solutions are already being called.
//      ***********************************************************************/
//      SmStatus SmEvalNFunctionsObject::ComputeJacobian
//        (SmTArray<double>       & crX,          // in : current parameter values
//         const SmTArray<double> & rF,           // in : current equation values
//         SmMatrix               & rJacobian)    // out: jacobian finite differences
//      {
//        double sData[16];
//        SmTArray<double> sFTmp(16,sData);
//        sFTmp.SetSize(rF.GetSize());
//      
//        // for every equation
//        for (ULONG j=0; j<rF.GetSize(); j++) 
//          {
//            double temp = crX[j];
//            double h    = SM_EFF_ZERO_SQRT * (1.0 + smos_Fabs(temp));
//            if (smos_Fabs(h) < SM_EFF_ZERO_SQRT/10.0) h=SM_EFF_ZERO_SQRT;
//      
//            // increment the jth parameter
//            crX[j] = temp+h;
//      
//            // evaluate all equations
//            SmBoolean bFoundAnswer;
//            SER(Evaluate(crX,sFTmp,NULL,bFoundAnswer));
//      
//            // get actual h (may have been clipped by the evaluator)
//            h=crX[j]-temp;
//      
//            // compute jth jacobian row by 1st order forward finite differences
//            if (smos_Fabs(h) > SM_EFF_ZERO) 
//              {
//                for (ULONG i=0; i<rF.GetSize(); i++) 
//                  {
//                    rJacobian[i][j]=(sFTmp[i]-rF[i])/h;
//                  }
//              }
//            
//            // decrement the jth parameter
//            crX[j] = temp-h; // Try step back instead
//            SER(Evaluate(crX,sFTmp,NULL,bFoundAnswer));
//      
//            // get actual h (may have been clipped by the evaluator)
//            h=crX[j]-temp;
//            
//            // compute jth jacobian row by 1st order forward finite differences
//            if (smos_Fabs(h) > SM_EFF_ZERO) 
//              {
//                for (ULONG i=0; i<rF.GetSize(); i++) 
//                  {
//                   double dVal = (sFTmp[i]-rF[i])/h;
//      
//                   // save the larger value
//                   if (smos_Fabs(dVal) > smos_Fabs(rJacobian[i][j])) 
//                     {
//                       rJacobian[i][j] = dVal;
//                     }
//                  }
//              }
//            
//            // restore original value    
//            crX[j] = temp;
//          }
//      
//        // all done
//        return SM_SUCCESS;
//      
//      } // end SmEvalNFunctionsObject::ComputeJacobian
