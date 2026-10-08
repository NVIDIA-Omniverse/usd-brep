// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmPolynomial.cpp
* PURPOSE: Implementation of polynomial methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmPolynomial.h>
#include <SmMath.h>

/*******************************************************************//**
PURPOSE: Solve a quadratic polynomial equation:
Example:
          2  
        Ax  +  Bx + C = 0   where C is adCoefficients[0], 
                                  B is adCoefficients[1],
                             and  A is adCoefficients[2]

NOTES: The tolerance should typically be between 1.0e-8 and
    1.0e-12.  This method is taken from Graphic Gems I.
***********************************************************************/
SmStatus SmPolynomial::SolveQuadraticEqn // eff: solve  Ax**2 + Bx + C = 0                     
  (double adCoefficients[3],             // in : coefficients, ordered:[C B A]                 
   double  dZeroTolerance,               // in : Max allowed deviation from zero for a solution
   ULONG & rlNumSolutions,               // out: 0 - imaginary roots                           
                                         //      1 - a single double root                      
                                         //      2 - two real roots                            
   double  adSolutions[2])               // out: Param values of zero crossings                
{
  adSolutions[0] = adSolutions[1] = 0.0; // protect the caller

  // If first term is zero then solve linear equation.
  if ( smos_Fabs(adCoefficients[2]) < dZeroTolerance )
    {
      // No quadratic term, it's a linear equation.  Check for constant.
      if ( smos_Fabs(adCoefficients[1]) < dZeroTolerance )
        {
          // Constant equation: independent of the variable,
          // so there's no meaningful solution.
          rlNumSolutions = 0;
        }
      else
        {
          // Linear, solve linear equation
          adSolutions[0] = - adCoefficients[0] / adCoefficients[1];
          rlNumSolutions = 1;
        }
    }
  else // A (adCoefficients[2]) != 0 branch
    {
      // If the constant term is zero, then it's x * ( Ax + B ) = 0,
      // so one solution is x = 0, the other is -B/A.
      if ( smos_Fabs( adCoefficients[0] ) < dZeroTolerance )
        {
          // zero solution
          adSolutions[0] = 0.0;

          // 2nd solution - skip if it is also zero
          if ( smos_Fabs( adCoefficients[1]) < dZeroTolerance )
            {
              rlNumSolutions = 1;
            }
          else
            {
              adSolutions[1] = - adCoefficients[1] / adCoefficients[2];
              rlNumSolutions = 2;
            }
        }
      else // A (adCoefficients[2]) and C (adCoefficients[0]) != 0 branch
        {
          // Solve the quadratic.
          double p = adCoefficients[1] / (2.0 * adCoefficients[2]);
          double q = adCoefficients[0] / adCoefficients[2];

          double dDisc = p*p - q;

          // If discriminant is zero, just one 'grazing' solution.
          if (   smos_Fabs(dDisc) < dZeroTolerance 
              || ( dDisc < 0.0 && smos_Fabs(dDisc) < SM_EFF_ZERO_SQRT ))
            {
              adSolutions[0] = - p;
              rlNumSolutions = 1;
            }
          else if ( dDisc < 0.0 )
            {
              rlNumSolutions = 0;
            }
          else
            {
              // dDisc must be > 0
              double dSqrtDisc = smos_Sqrt(dDisc);
              adSolutions[0] =  dSqrtDisc - p;
              adSolutions[1] = -dSqrtDisc - p;
              rlNumSolutions = 2;
            }
        } // end A (adCoefficients[2]) and C (adCoefficients[0]) != 0 branch
    } // end A (adCoefficients[2]) != 0 branch

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      // for every solution
      for(ULONG ii=0;ii<rlNumSolutions;ii++)
        {
          // check solution
          double dCheck =    adCoefficients[0] 
                           + adSolutions[ii] * (  adCoefficients[1] 
                                                + adSolutions[ii] * adCoefficients[2]) ;
          double dDev = smos_Fabs(dCheck) ;
          if(dDev > 100 * dZeroTolerance)
            {
              SM_ASSERT_MSG( (dDev < 100 * dZeroTolerance), _T("SmPolynomial::SolveQuadraticEqn returned a bad solution")) ;
            }
        }
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmPolynomial::SolveQuadraticEqn

/*******************************************************************//**
PURPOSE: Solve a polynomial which is a cubic equation:
Example:
        3      2
      Ax  +  Bx  + Cx + D = 0  - where D is adCoefficients[0],
                                       C is adCoefficients[1],
                                       B is adCoefficients[2],
                                  and  A is adCoefficients[3]
                                       

NOTES: The tolerance should typically be between 1.0e-8 and
    1.0e-12.  This method is taken from Graphics Gems I.
***********************************************************************/
SmStatus SmPolynomial::SolveCubicEqn  // eff: solve  Ax**3 + Bx**2 + Cx + D = 0                            
 (double adCoefficients[4],           // in : coefficients, ordered:[D C B A]                              
  double dZeroTolerance,              // in : Max allowed deviation from zero for a solution               
  ULONG & rlNumSolutions,             // out: Real zero crossing cnt (imaginary and mult-roots not counted)
  double adSolutions[3])              // out: Param values of zero crossings                               
{
    // If not it is not a valid cubic - pop out and
    // solve quadratic equation instead of error.
    if (smos_Fabs(adCoefficients[3]) < dZeroTolerance) 
      {
        double adQCoeff[3];
        adQCoeff[0] = adCoefficients[0];
        adQCoeff[1] = adCoefficients[1];
        adQCoeff[2] = adCoefficients[2];
        double adQSol[2];
        SER(SmPolynomial::SolveQuadraticEqn(adQCoeff,dZeroTolerance,
            rlNumSolutions,adQSol));
        for (ULONG i=0; i<rlNumSolutions; i++) 
          {
            adSolutions[i] = adQSol[i];
          }
        return SM_SUCCESS;
      }

    double A = adCoefficients[2] / adCoefficients[3];
    double B = adCoefficients[1] / adCoefficients[3];
    double C = adCoefficients[0] / adCoefficients[3];

    double sq_A = A * A;
    double p = 1.0/3 * (-1.0/3 * sq_A + B);
    double q = 1.0/2 * (2.0/27 * A * sq_A - 1.0/3 * A * B + C);
    double cb_p = p * p * p;
    double D = q * q + cb_p;

    if (smos_Fabs(D) < dZeroTolerance) 
      {
        if (smos_Fabs(q) < dZeroTolerance) 
          { // One tripple root
            adSolutions[0] = 0.0;
            rlNumSolutions = 1;
          }
        else 
          { // One single and one double solution
            double u = smos_CubeRoot(-q); 
            adSolutions[0] = 2 * u;
            adSolutions[1] = - u;
            rlNumSolutions = 2;
          }
      }
    else if (D < 0) 
      { // Three solutions
        double phi = 1.0/3 * smos_ArcCosine(-q / sqrt(-cb_p));
        double t = 2 * smos_Sqrt(-p);

        adSolutions[0] = t * smos_Cosine(phi);
        adSolutions[1] = - t * smos_Cosine (phi + SM_PI / 3);
        adSolutions[2] = -t * smos_Cosine (phi - SM_PI / 3);
        rlNumSolutions = 3;
      }
    else 
      { // One real solution
        double sqrt_D = smos_Sqrt(D);
        double u = smos_CubeRoot(sqrt_D - q);
        double v = - smos_CubeRoot(sqrt_D + q);
        adSolutions[0] = u + v;
        rlNumSolutions = 1;
      }

    double sub = 1.0/3 * A;
    for (ULONG i=0; i<rlNumSolutions; ++i) 
      {
        adSolutions[i] -= sub;
      }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      // for every solution
      for(ULONG ii=0;ii<rlNumSolutions;ii++)
        {
          // check solution
          double dCheck =    adCoefficients[0] 
                           + adSolutions[ii] * (  adCoefficients[1] 
                                                + adSolutions[ii] * (  adCoefficients[2] 
                                                                     + adSolutions[ii] * adCoefficients[3])) ;
          double dDev = smos_Fabs(dCheck) ;
          if(dDev > 100 * dZeroTolerance)
            {
                SM_ASSERT_ERR;
              //SM_ASSERT_ERR( dDev < 100 * dZeroTolerance, _T("SmPolynomial::SolveCubicEqn returned a bad solution")) ;
            }
        }
    }
#endif // SM_DEBUG_CODE

    return SM_SUCCESS;

} // end SmPolynomial::SolveCubicEqn

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/

static void sm_AddSolution
 (double dSolValue, 
  double dZeroTolerance,
  ULONG & rlNumSolutions,
  double adSolutions[4])
{
  SmBoolean bDuplicate = FALSE;
  for (ULONG kk=0; kk<rlNumSolutions; kk++) 
    {
      if (smos_Fabs(dSolValue - adSolutions[kk]) < dZeroTolerance) 
        {
          bDuplicate = TRUE;
          break;
        }
    }
  if (!bDuplicate) adSolutions[rlNumSolutions++] = dSolValue;

} // end sm_AddSolution

/*******************************************************************//**
PURPOSE: Solve a polynomial which is a quartic (degree 4) equation:
Example:
        4      3     2
      Ax  +  Bx  + Cx + Dx + E = 0  - where E is adCoefficients[0],
                                            D is adCoefficients[1],
                                            C is adCoefficients[2],
                                            B is adCoefficients[3]
                                       and  A is adCoefficients[4]
                                       

NOTES: The tolerance should typically be between 1.0e-8 and
    1.0e-12.  This method is taken from Graphics Gems I.
***********************************************************************/
SmStatus SmPolynomial::SolveQuarticEqn  // eff: solve  Ax**4 + Bx**3 + Cx**2 + Dx + E = 0                    
 (double adCoefficients[5],             // in : coefficients, ordered:[E D C B A]                            
  double dZeroTolerance,                // in : Max allowed deviation from zero for a solution               
  ULONG & rlNumSolutions,               // out: Real zero crossing cnt (imaginary and mult-roots not counted)
  double adSolutions[4])                // out: Param values of zero crossings                               
{
  rlNumSolutions = 0;
  // If not it is not a valid quartic - pop out and
  // solve cubic equation instead of error.
  if (smos_Fabs(adCoefficients[4]) < dZeroTolerance) 
    {
      double adCCoeff[4];
      adCCoeff[0] = adCoefficients[0];
      adCCoeff[1] = adCoefficients[1];
      adCCoeff[2] = adCoefficients[2];
      adCCoeff[3] = adCoefficients[3];
      double adCSol[3];
      ULONG lNumCSol;

      // seek cubic, quadratic, or linear solutions
      SER(SmPolynomial::SolveCubicEqn(adCCoeff,dZeroTolerance, lNumCSol,adCSol)) ;

      // set output
      for (ULONG i=0; i<lNumCSol; i++) 
        {
          sm_AddSolution(adCSol[i],dZeroTolerance,rlNumSolutions,adSolutions);
        }
      return SM_SUCCESS;
    }
  else // A (adCoefficients[4]) != 0 branch
    {
      // Convert to x**4 + Ax**3 + Bx**2 + Cx + D = 0, normal form 
      double A = adCoefficients[3] / adCoefficients[4];
      double B = adCoefficients[2] / adCoefficients[4];
      double C = adCoefficients[1] / adCoefficients[4];
      double D = adCoefficients[0] / adCoefficients[4];

      // Substitute x = y - A/4 to eliminate cubic term:
      // y^4 + py^2 + qy + r = 0

      double sq_A = A * A;
      double p    = -3.0/8.0 * sq_A + B;
      double q    = 1.0/8.0 * sq_A * A - 1.0/2.0 * A * B + C;
      double r    = (- 3.0/256.0 * sq_A * sq_A) + (1.0/16.0 * sq_A * B) - (1.0/4.0 * A * C) + D;

      if (smos_Fabs(r) < dZeroTolerance) 
        {
          // No absolute term: y(y^3 + py + q) = 0
          // Solve cubic and add zero to solution
          double adCCoeff[4];
          adCCoeff[0] = q;
          adCCoeff[1] = p;
          adCCoeff[2] = 0.0;
          adCCoeff[3] = 1.0;
          double adCSol[3];
          ULONG lNumCSol;

          // seek cubic, quadratic, or linear solutions
          SER(SmPolynomial::SolveCubicEqn(adCCoeff,dZeroTolerance, lNumCSol,adCSol));

          // set output
          for (ULONG i=0; i<lNumCSol; i++) 
            {
              sm_AddSolution(adCSol[i],dZeroTolerance,rlNumSolutions,adSolutions);
            }
          
          // add zero solution
          sm_AddSolution(0.0,dZeroTolerance,rlNumSolutions,adSolutions);
        }
      else // r != 0 branch 
        { 
          // Solve the resolvant cubic ...
          // ... and take the one real solution ...
          // ... to build two quadratic equations 
          double adCCoeff[4];
          adCCoeff[0] = 1.0/2.0 * r * p - 1.0 / 8.0 * q * q;
          adCCoeff[1] = - r;
          adCCoeff[2] = - 1.0/2.0 * p;
          adCCoeff[3] = 1.0;
          double adCSol[3];
          ULONG lNumCSol;

          // seek solutions to cubic
          SER(SmPolynomial::SolveCubicEqn(adCCoeff,dZeroTolerance, lNumCSol,adCSol));
  //        if (lNumCSol != 1) SE(SM_ERR);

          double z = adCSol[0];  // one real solution
          double u = z * z - r;
          double v = 2.0 * z - p;

          //
          if      (smos_Fabs(u) < dZeroTolerance) { u = 0.0 ; }
          else if ( u > 0.0 )                     { u = smos_Sqrt(u) ; }
          else                                    { rlNumSolutions = 0 ;
                                                    return SM_SUCCESS ;
                                                  }

          //
          if      (smos_Fabs(v) < dZeroTolerance) { v = 0.0 ; }
          else if (v > 0.0)                       { v = smos_Sqrt(v) ; }
          else                                    { rlNumSolutions = 0 ;
                                                    return SM_SUCCESS ;
                                                  }

          //
          double adQCoeff[3];
          adQCoeff[0] = z - u;
          adQCoeff[1] = q < 0 ? -v : v;
          adQCoeff[2] = 1.0;
          double adQSol[2];
          ULONG lNumQSol;

          SER(SmPolynomial::SolveQuadraticEqn(adQCoeff,dZeroTolerance,lNumQSol,adQSol));

          rlNumSolutions = 0;
          for (ULONG j=0; j<lNumQSol; j++) 
            {
              sm_AddSolution(adQSol[j],dZeroTolerance,rlNumSolutions,adSolutions);
            }

          //
          adQCoeff[0] = z + u;
          adQCoeff[1] = q < 0 ? v : -v;
          adQCoeff[2] = 1.0;

          SER(SmPolynomial::SolveQuadraticEqn(adQCoeff,dZeroTolerance,lNumQSol,adQSol));

          for (ULONG jj=0; jj<lNumQSol; jj++) 
            {
              sm_AddSolution(adQSol[jj],dZeroTolerance,rlNumSolutions,adSolutions);
            }

        } // end r != 0 branch

      // Now resubstitute to scale back parameters
      double sub = 1.0/4.0 * A;
      for (ULONG k=0; k<rlNumSolutions; k++) 
        {
          adSolutions[k] -= sub;
        }
    } // end A (adCoefficients[4]) != 0 branch

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      // for every solution
      for(ULONG ii=0;ii<rlNumSolutions;ii++)
        {
          // check solution
          double dCheck =   adCoefficients[0] 
                          + adSolutions[ii] * (  adCoefficients[1] 
                                               + adSolutions[ii] * (  adCoefficients[2] 
                                                                    + adSolutions[ii] * (  adCoefficients[3]
                                                                                         + adSolutions[ii] * adCoefficients[4]))) ;
          double dDev = smos_Fabs(dCheck) ;
          if(dDev > 100 * dZeroTolerance)
            {
                SM_ASSERT_ERR;
              //SM_ASSERT_ERR( dDev < 100 * dZeroTolerance, _T("SmPolynomial::SolveCubicEqn returned a bad solution")) ;
            }
        }
    }
#endif // SM_DEBUG_CODE

  return SM_SUCCESS;

} // end SmPolynomial::SolveQuarticEqn





