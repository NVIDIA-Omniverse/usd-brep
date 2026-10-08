// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmPolynomial.h
* PURPOSE: Performs operations on polynomial equations including
*   solving them. 
**********************************************************************/

#ifndef __SMPOLYNOMIAL_H__
#define __SMPOLYNOMIAL_H__

#include <SmTypes.h>

/*******************************************************************//**
PURPOSE: This object defines polynomial operations.

NOTES: Coeff[i] is the coefficient of the x^i term: the equation is
    y = Coeff[0] + Coeff[1]*x + Coeff[2]*x^2 + ...
***********************************************************************/
class SM_EXPORT SmPolynomial
{
public:
    static SmStatus SolveQuadraticEqn  // eff: solve  Ax**2 + Bx + C = 0   
       (double  adCoefficients[3],     // in : coefficients, ordered:[C B A]                            
        double  dZeroTolerance,        // in : Max allowed deviation from zero for a solution 
        ULONG & rlNumSolutions,        // out: 0 - imaginary roots
                                       //      1 - a single double root
                                       //      2 - two real roots       
        double  adSolutions[2]) ;      // out: Param values of zero crossings                 

    static SmStatus SolveCubicEqn      // eff: solve  Ax**3 + Bx**2 + Cx + D = 0   
       (double  adCoefficients[4],     // in : coefficients, ordered:[D C B A]                            
        double  dZeroTolerance,        // in : Max allowed deviation from zero for a solution 
        ULONG & rlNumSolutions,        // out: Real zero crossing cnt (imaginary and mult-roots not counted)      
        double  adSolutions[3]) ;      // out: Param values of zero crossings                 
         
    static SmStatus SolveQuarticEqn    // eff: solve  Ax**4 + Bx**3 + Cx**2 + Dx + E = 0
       (double  adCoefficients[5],     // in : coefficients, ordered:[E D C B A]
        double  dZeroTolerance,        // in : Max allowed deviation from zero for a solution
        ULONG & rlNumSolutions,        // out: Real zero crossing cnt (imaginary and mult-roots not counted)
        double  adSolutions[4]) ;      // out: Param values of zero crossings


// This is a temporary wrapper, for renaming Quadric to Quadratic.
// Installed 22 March 2006; remove it eventually.
    static SmStatus SolveQuadricEqn(
        double adCoefficients[3], double dZeroTolerance,
        ULONG & rlNumSolutions, double adSolutions[2] )
    {
        return SolveQuadraticEqn( 
            adCoefficients, dZeroTolerance, rlNumSolutions, adSolutions );
    }


} ; // end class SmPolynomial



#endif // !__SMPOLYNOMIAL_H__
