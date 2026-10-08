// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* CrvFit.h: Fit curve related funcitons                              */
/**********************************************************************/

#ifndef _CRVFIT_H
#define _CRVFIT_H

/* contains :                         */

/* U-MAP AND KNOT VECTOR BUILDING: */
/* N_FitCalcCrvParamValues()        Calc CrvParam values for a set of 3d Points to be used by other curve fitting algorithms  */
/* N_FitCrvCalcKnotVector()         Build KnotVector for global curve interpolation with EndPt cstrnts from Param array; EndKnots = EndParam values, InternalKnots U[i] = Avg(u[i],u[i+1]...u[i+p-1])  */
/* N_FitCalcKnotVectorEndDerivs()   Build KnotVector for global curve interpolation with EndPt and both EndDeriv cstrnts from Param array */
/* N_FitCalcKnotVectorDeriv()       Build KnotVector for global curve interpolation with EndPt and either Start or End Deriv cstrnt from param array */
/* N_FitCalcKnotVectorCrvApprox()   Build KnotVector for global curve interpolation with EndPt cstrnts from Param array; EndKnots = EndParam values, InternalKnots U[i] = Avg(Avg(u[i],u[i+1]...u[i+p-1]))  */
/* N_FitCalcKnotsDerivs()           Build knotVector for global curve interpolation for PtSet with Deriv data from Param array  */
/* N_FitCalcKnotsEndDerivs()        Build knotVector for global curve approximation with arbitrary end derivatives from param array and curve CPt count and degree, and end deriv count  */
/* N_FitCalcKnotsHighEndDerivs()    Build knotVector for global curve interpolation with arbitrary end derivatives from param array and curve CPt count and degree, and end deriv count  */
/* N_FitCalcFuncParamValues()       Build knotVector for global function interpolation. */

/* KNOT REMOVAL:    */
/* N_FitRemoveKnots()               Remove least sensitive knots while disp at set of given CrvPts remains in tol */
/* N_FitRemoveKnotsAndDerivs()      Remove least sensitive knots while disp at set of given CrvPts remains in tol while preserving specified end pt and derivatives */
/* N_FitRemoveKnotsPriorities()     Remove knots in specified order while disp at set of given CrvPts remains in tol while preserving specified end pt and derivatives */

/* INTERPOLATE FUNCTIONS:   */
/* N_FitArcToEndPtsAndTangents()    Make deg 2 piecwise bezier BSpline CPts to fit given end pts and tangents - usually 2 segments can be more */
/* N_FitBiArc()                     Less general version of N_FitArcToEndPtsAndTangents() that can't build a result for some end pt and tangent combinations */
/* N_FitCrvArcs()                   Interp PtSet with a piecewise circular G1 curve */

/* N_FitFuncInterp()                Interp set of 1d values with NL_CFUN with specified deg finding param values with specified strategy  */
/* N_FitFuncInterpGivenParams()     Interp set of 1d values with NL_CFUN with spedified deg and param values  */

/* N_FitCrvInterp()                 Interp PtSet with NL_CURVE with specified deg finding param values with specified strategy  */
/* N_FitCrvInterpGivenParams()      Interp PtSet with NL_CURVE with specified deg and Knot vector  */
/* N_FitCrvDerivs()                 Interp PtSet and two end derivs with specified deg finding param values with specified strategy  */
/* N_FitCrvTangents()               Interp PtSet and two end tangents with specified deg finding param values with specified strategy  */
/* N_FitCrvKnotsAndDerivs()         Interp PtSet and two end derivs given params for 3d pts, with NL_CURVE of specified degree and knot vector */
/* N_FitCrvKnotsAndDeriv()          Interp PtSet and one end derivs given params for 3d pts, with NL_CURVE of specified degree and knot vector */
/* N_FitCubicSplineInterp()         Interp PtSet and two end derivs with piecewise cubic BSpline curve picking param values with specified strategy */
/* N_FitCrvKnotsAndTangents()       Interp PtSet and opt end derivs given params and an optional suggested knot vector */
/* N_FitCrvShape()                  Interp PtSet and opt end derivs with a starting with a base Hermite or piecewise cubic curve  */
/* N_FitCrvCubic()                  Interp PtSet with a C1 continuous cubic BSpline  */
/* N_FitCrvCubicTangents()          Interp PtSet pos and tans and opt end derivatives with a cubic BSpline */
/* N_FitCrvConics()                 Interp PtSet with a G1 continuous conic curve */
/* N_FitHermite()                   Interp 2 Pt Positions with deriviatives with a Hermite segment */

/* APPROXIMATING FUNCTIONS: */
/* N_FitLineToPts()                 Approx unparam PtSet with best-fit line  */
/* N_FitConicToPts()                Approx unparam PtSet with best-fit conic  */
/* N_FitLocalCubicApprox()          Internal for N_FitCrvCubicApprox() - Approx unparam PtSet with best-fit cubic */
/* N_FitArcToPts()                  Approx unparam PtSet with a best-fit circle or circular arc - returns line when line is better fit  */
/* N_FitCrvArcApprox()              Approx unparam PtSet with opt tangents with a best-fit piecewise quadratic biarc curve  */
/* N_FitCrvParabArcs()              Approx unparam PtSet with a best-fit piecewise parabolic curve with C1 or G1 continuity, while smoothing or keeping cusps/corners */

/* LEAST-SQAURES BSPLINE APPROX: These functions approx a sequenced Point Set with a least-squares computed BSpline using various combinations of inputs */
/* +----------------------------------+------+------+----------+------+-----------+-----------------+----+-------+-------+----------+  */
/* |Approx PtSet with least-squares   |Pt    |Pt    |Pt Cstrn  |PtTans|Closed/    |Opt              |Tol |CPoint |degree |KntVector |  */
/* |BSpline with data and cur inputs  |Set   |Params|Pt Wts    |Derivs|Open       |EndTangent/Derivs|    | count |       |          |  */
/* +----------------------------------+------+------+----------+------+-----------+-----------------+----+-------+-------+----------+  */
/* | N_FitCrvApproxKnots()            |input |input |no        |no    |Open       |no               |no  |input  |input  |input     |  */ 
/* | N_FitCrvTangentsKnotsParams()    |input |input |no        |no    |Open       |yes              |no  |input  |input  |input     |  */ 
/* | N_FitCrvApproxKnotsTangents()    |input |input |no        |no    |Open       |yes              |no  |input  |input  |suggested |  */ 
/* | N_FitCrvApproxLstSq()            |input |found |no        |no    |Open       |no               |no  |input  |input  |found     |  */ 
/* | N_FitCrvApproxTangents()         |input |found |no        |no    |Open       |yes              |no  |input  |input  |found     |  */ 
/* | N_FitCrvApprox()                 |input |found |no        |no    |Open/Closed|yes              |yes |input  |found  |found     |  iterates through increasing degrees until tol is met */  
/* | N_FitCrvCubicApprox()            |input |found |no        |optTan|open       |no               |yes |found  |3      |found     |  iterates to find number of required cubic pieces */
/* | N_FitCrvApproxKnotsTol()         |input |found |no        |no    |open       |yes              |yes |found  |input  |suggested |  CPt cnt = PtSet size - then knots are removed */
/* | N_FitCrvFirstDeriv()             |input |found |no        |derivs|open       |no               |no  |found  |2 or 3 |found     |  CPt cnt = 2*PtSet size */
/* | N_FitCrvFirstDerivAndKnots()     |input |input |no        |derivs|open       |no               |no  |input  |2 or 3 |input     |  */
/* | N_FitCrvConicsApprox()           |input |found |no        |optTan|open       |no               |yes |found  |2      |found     |  iter: fit conic to as many pts as possible within tol, repeat */
/* | N_FitCrvWeightedLstSq()          |input |found |Cstrn & Wt|optDer|open       |no               |no  |input  |input  |found     |  */
/* | N_FitCrvWeightedLstSqPeriodic()  |input |input |CStrn & Wt|optDer|closed     |no               |no  |input  |input  |input     |  */
/* | N_FitCrvWeightedLstSqKnots()     |input |input |Cstrn & Wt|optDer|open       |no               |no  |input  |input  |input     |  */
/* | N_FitCrvApproxClosed()           |input |found |no        |no    |closed     |yes              |no  |input  |input  |found     |  */
/* | N_FitCrvApproxDerivs()           |input |found |no        |no    |open       |mult end derivs  |no  |input  |input  |found     |  */
/* | N_FitCrvApproxClosedConditions() |input |found |no        |no    |closed     |mult end derivs  |no  |input  |input  |found     |  */
/* | N_FitCrvHighDerivs()             |input |found |no        |no    |open       |mult end derivs  |no  |found  |input  |found     |  CPt cnt = PtSetSize + end Cstrn count */
/* | N_FitCrvClosedDerivsParamsKnots()|input |input |no        |no    |closed     |mult end derivs  |no  |input  |input  |input     |  */
/* | N_FitCrvDerivsKnots()            |input |input |no        |no    |open       |mult end derivs  |no  |input  |input  |input     |  */
/* | N_FitCrvLstSqEnds()              |input |found |no        |no    |open       |yes              |no  |input  |input  |input     |  */
/* | N_FitCrvPtsNormals()             |input |found |no        |nrmls |open       |no               |no  |found  |2 or 3 |found     |  */
/* | N_FitPeriodicCubic()             |input |input |no        |no    |closed     |no               |no  |found  |3      |found     | CPt cnt = PtSet size + 2 */
/* | N_FitCrvApproxKnotsAndTangentsTol|input |found |no        |no    |open       |yes              |yes |found  |ps & pr|found     | iterates Cpt cnt = PtSet size + 2 followed by knot removal - final degree = pr*/
/* +----------------------------------+------+------+----------+------+-----------+-----------------+----+-------+-------+----------+  */

/* MISC FUNCTIONS:             */
/* N_FitCrvCPtsFromSrfData()        Internal for N_FitSrfToPts() - find interpolating BSpline CPt positions for a row or col out of a pt array */
/* N_FitCrvDerivsMatrix()           Compute control points for a nurbs curve interpolating a PtSet and two EndDerivs given LU-decomposed interpolation matrix  */
/* N_FitCrvMatrix()                 Compute control points for a nurbs curve interpolating a PtSet given LU-decomposed interpolation matrix  */
/* N_FitCrvDerivMatrix()            Compute control points for a nurbs curve interpolating a PtSet and one EndDeriv given LU-decomposed interpolation matrix  */
/* N_FitCalcMatrix()                Compute LU-decomposed matrix necessary for curve interpolating with or without tangent constraints  */
/* N_FitSmoothPts()                 Smooth 3D points in preparation for curve fitting  */


/* U-MAP AND KNOT VECTOR BUILDING: */
GW_EXPORT NL_FLAG N_FitCalcCrvParamValues( NL_VOID *, NL_INDEX, NL_FLAG, NL_FLAG, NL_PARAMETER * );
GW_EXPORT NL_VOID N_FitCrvCalcKnotVector( NL_REAL *, NL_INDEX, NL_DEGREE, NL_KNOTVECTOR * );
GW_EXPORT NL_VOID N_FitCalcKnotVectorEndDerivs( NL_REAL *, NL_INDEX, NL_DEGREE, NL_KNOTVECTOR * );
GW_EXPORT NL_FLAG N_FitCalcKnotVectorDeriv( NL_REAL *, NL_INDEX, NL_DEGREE, NL_FLAG, NL_KNOTVECTOR * );
GW_EXPORT NL_FLAG N_FitCalcKnotVectorCrvApprox( NL_REAL *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_KNOTVECTOR * );
GW_EXPORT NL_VOID N_FitCalcKnotsDerivs( NL_REAL *, NL_INDEX, NL_DEGREE, NL_KNOTVECTOR * );
GW_EXPORT NL_FLAG N_FitCalcKnotsEndDerivs( NL_REAL *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_INDEX, NL_INDEX, NL_KNOTVECTOR * );
GW_EXPORT NL_VOID N_FitCalcKnotsHighEndDerivs( NL_REAL *, NL_INDEX, NL_DEGREE, NL_INDEX, NL_INDEX, NL_KNOTVECTOR * );
GW_EXPORT NL_FLAG N_FitCalcFuncParamValues( NL_REAL *, NL_INDEX, NL_FLAG, NL_PARAMETER * );

/* KNOT REMOVAL:    */
GW_EXPORT NL_FLAG N_FitRemoveKnots( NL_CURVE *, NL_REAL *, NL_REAL *, NL_INDEX, NL_REAL );
GW_EXPORT NL_FLAG N_FitRemoveKnotsAndDerivs( NL_CURVE *, NL_REAL *, NL_REAL *, NL_INDEX, NL_REAL, NL_FLAG, NL_INDEX );
GW_EXPORT NL_FLAG N_FitRemoveKnotsPriorities( NL_CURVE *, NL_INDEX *, NL_REAL *, NL_REAL *, NL_INDEX, NL_REAL, NL_FLAG, NL_INDEX );

/* INTERPOLATE FUNCTIONS:   */
GW_EXPORT NL_FLAG N_FitArcToEndPtsAndTangents( NL_POINT, NL_VECTOR, NL_POINT, NL_VECTOR, NL_CPOINT *, NL_INDEX * );
GW_EXPORT NL_FLAG N_FitBiArc( NL_POINT, NL_VECTOR, NL_POINT, NL_VECTOR, NL_CPOINT *, NL_INDEX * );
GW_EXPORT NL_FLAG N_FitCrvArcs( NL_POINT *, NL_INDEX, NL_FLAG, NL_FLAG, NL_CURVE *, NL_STACKS * );

GW_EXPORT NL_FLAG N_FitFuncInterp( NL_REAL *, NL_INDEX, NL_DEGREE, NL_FLAG, NL_CFUN *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitFuncInterpGivenParams( NL_REAL *, NL_INDEX, NL_DEGREE, NL_PARAMETER *, NL_CFUN *, NL_STACKS * );

GW_EXPORT NL_FLAG N_FitCrvInterp( NL_POINT *, NL_INDEX, NL_DEGREE, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvInterpGivenParams( NL_VOID *, NL_INDEX, NL_FLAG, NL_PARAMETER *, NL_KNOTVECTOR *, NL_DEGREE, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvDerivs( NL_POINT *, NL_INDEX, NL_DEGREE, NL_VECTOR, NL_VECTOR, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvTangents( NL_POINT *, NL_INDEX, NL_DEGREE, NL_VECTOR, NL_VECTOR, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvKnotsAndDerivs( NL_VOID *, NL_INDEX, NL_FLAG, NL_PARAMETER *, NL_KNOTVECTOR *, NL_DEGREE, NL_VOID *, NL_VOID *, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvKnotsAndDeriv( NL_VOID *, NL_INDEX, NL_FLAG, NL_PARAMETER *, NL_KNOTVECTOR *, NL_DEGREE, NL_VOID *, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCubicSplineInterp( NL_POINT *, NL_INDEX, NL_VECTOR, NL_VECTOR, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvKnotsAndTangents( NL_POINT *, NL_INDEX, NL_PARAMETER *, NL_VECTOR *, NL_VECTOR *, NL_FLAG, NL_KNOTVECTOR **, NL_REAL, NL_DEGREE, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvShape( NL_POINT *, NL_INDEX, NL_VECTOR *, NL_VECTOR *, NL_INDEX, NL_FLAG, NL_FLAG, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvCubic( NL_POINT *, NL_INDEX, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvCubicTangents( NL_POINT *, NL_INDEX, NL_VECTOR *, NL_VECTOR *, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvConics( NL_POINT *, NL_INDEX, NL_FLAG, NL_FLAG, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitHermite( NL_POINT, NL_POINT, NL_VECTOR, NL_VECTOR, NL_CURVE *, NL_STACKS * );

/* APPROXIMATING FUNCTIONS: */
GW_EXPORT NL_FLAG N_FitLineToPts( NL_POINT *, NL_INDEX, NL_CURVE *, NL_REAL *, NL_REAL *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitConicToPts( NL_POINT *, NL_INDEX, NL_INDEX, NL_VECTOR, NL_VECTOR, NL_REAL, NL_REAL, NL_REAL, NL_CPOINT *, NL_INDEX * );
GW_EXPORT NL_FLAG N_FitLocalCubicApprox( NL_POINT *, NL_INDEX, NL_VECTOR *, NL_INDEX, NL_INDEX, NL_REAL *, NL_REAL, NL_REAL, NL_REAL, NL_CPOINT *, NL_INDEX * );
GW_EXPORT NL_FLAG N_FitArcToPts( NL_POINT *, NL_INDEX, NL_REAL, NL_FLAG, NL_FLAG, NL_FLAG, NL_CURVE *, NL_FLAG *, NL_REAL *, NL_REAL *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvArcApprox( NL_POINT *, NL_INDEX, NL_VECTOR *, NL_FLAG, NL_INDEX, NL_REAL, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvParabArcs( NL_POINT *, NL_INDEX, NL_FLAG, NL_FLAG, NL_FLAG, NL_FLAG, NL_CURVE *, NL_STACKS * );

/* LEAST-SQAURES BSPLINE APPROX: These functions approx a sequenced Point Set with a least-squares computed BSpline using various combinations of inputs */
GW_EXPORT NL_FLAG N_FitCrvApproxKnots( NL_VOID *, NL_INDEX, NL_FLAG, NL_PARAMETER *, NL_KNOTVECTOR *, NL_INDEX, NL_DEGREE, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvTangentsKnotsParams( NL_VOID *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_VOID *, NL_VOID *, NL_FLAG, NL_FLAG, NL_PARAMETER *, NL_KNOTVECTOR *, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvApproxKnotsTangents( NL_POINT *, NL_INDEX, NL_PARAMETER *, NL_VECTOR *, NL_VECTOR *, NL_FLAG, NL_KNOTVECTOR **, NL_REAL, NL_INDEX, NL_DEGREE, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvApproxLstSq( NL_POINT *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvApproxTangents( NL_POINT *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_VECTOR *, NL_VECTOR *, NL_FLAG, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvApprox( NL_POINT *, NL_INDEX, NL_FLAG, NL_DEGREE, NL_DEGREE, NL_REAL, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvCubicApprox( NL_POINT *, NL_INDEX, NL_VECTOR *, NL_FLAG, NL_FLAG, NL_INDEX, NL_REAL, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvApproxKnotsTol( NL_POINT *, NL_INDEX, NL_DEGREE, NL_VECTOR *, NL_VECTOR *, NL_FLAG, NL_REAL, NL_KNOTVECTOR **, NL_FLAG, NL_CURVE *, NL_STACKS *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvFirstDeriv( NL_POINT *, NL_VECTOR *, NL_INDEX, NL_DEGREE, NL_FLAG, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvFirstDerivAndKnots( NL_VOID *, NL_VOID *, NL_INDEX, NL_FLAG, NL_PARAMETER *, NL_KNOTVECTOR *, NL_DEGREE, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvConicsApprox( NL_POINT *, NL_INDEX, NL_VECTOR *, NL_FLAG, NL_INDEX, NL_REAL, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvWeightedLstSq( NL_POINT *, NL_REAL *, NL_INDEX, NL_VECTOR *, NL_REAL *, NL_INDEX *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvWeightedLstSqPeriodic( NL_VOID *, NL_REAL *, NL_INDEX, NL_VOID *, NL_REAL *, NL_INDEX *, NL_INDEX, NL_FLAG, NL_PARAMETER *, NL_KNOTVECTOR *, NL_INDEX, NL_DEGREE, NL_FLAG, NL_CURVE *, NL_STACKS * );

// build Approx BsplineCrv of a TgtPt sequence (with opt Tgt1stDerivs) whose knot vector is independent of the TgtPt set count and spacing - uses a global least squares point penalty method
GW_EXPORT NL_FLAG N_FitCrvWeightedLstSqKnots(NL_VOID *,         // in : TgtPoint position constraint values to be approximated by points on the ApproxCrv
                                                                //      when ptp == NL_EPOINT, ptr to NL_POINT array
                                                                //      when ptp == NL_HPOINT, ptr to NL_CPOINT array
                                             NL_REAL *,         // in : Array of TgtPt weights for TgtPt-to-PtOnCrv dist penalty method 
                                                                //      (weight is stiffness of a spring between TgtPt and PtOnCrv - bigger weights tighter fits and less smoothness)
                                                                //      when wp[i] > 0.0: E[i] is unconstrained
                                                                //      when wp[i] < 0.0: E[i] is constrained
                                             NL_INDEX,          // in : highest index in TgtPoint array, (TgtPtArray Size = Highest Index + 1)
                                             NL_VOID *,         // in : Tgt 1st derivative constraint values to be approximated by points on the ApproxCrv
                                                                //      when ptp == NL_EPOINT, ptr to NL_VECTOR array
                                                                //      when ptp == NL_HPOINT, ptr to NL_CVECTOR array
                                             NL_REAL *,         // in : Array of Tgt1stDeriv weights for TgtPt1stDeriv-to-PtOnCrv1stDeriv penalty method 
                                                                //       (weight of stiffness of a rotational spring between Tgt1stDreriv and 1stDeriv val at PtOnCrv.
                                                                //        bigger weights force the direction of the ApproxCrv closer to the Tgt1stDeriv value.
                                                                //        can be used to add a 'damping' effect to prevent the polynomial ApproxCurve from deviating between TgtPts when
                                                                //        constraint 1st Deriv values point from one TgtPoint position to the next.)
                                                                //      when wd[i] > 0.0: F[i] is unconstrained
                                                                //      when wd[i] < 0.0: F[i] is constrained
                                             NL_INDEX *,        // in : Index array, the Tgt 1st derivative value at E[I[j]] is F[j]
                                             NL_INDEX,          // in : Highest index in arrays F, wd, and I, (Array Size = HIghest Index + 1)
                                             NL_FLAG,           // in : NL_EPOINT: TgtPts and Tgt1stDerivs are Euclidean pts and vectors.
                                                                //      NL_HPOINT: TgtPts and Tgt1stDerivs are Homogeneous pts and vectors.
                                             NL_PARAMETER *,    // in : Associated array of TgtCrv param values for each TgtPt value 
                                                                //      (the point on the curve that is made to approximate the associated TgtPt pos value) 
                                             NL_KNOTVECTOR *,   // in : Knot vector of approximating curve 
                                             NL_INDEX,          // in : Highest index of approximating curve CPts, (ApproxCrv CPT count = Highest Index + 1)
                                             NL_DEGREE,         // in : approximating curve degree (between 3 and 5 works well for pt sets representing sampled curve sequences)
                                             NL_CURVE *,        // out: Approximating Curve
                                             NL_STACKS * );     // i/o: approximating curve's memory stack

GW_EXPORT NL_FLAG N_FitCrvApproxClosed( NL_POINT *, NL_INDEX, NL_VECTOR *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvApproxDerivs( NL_POINT *, NL_INDEX, NL_VECTOR *, NL_INDEX, NL_VECTOR *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvApproxClosedConditions( NL_POINT *, NL_INDEX, NL_DEGREE, NL_VECTOR *, NL_INDEX, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvHighDerivs( NL_POINT *, NL_INDEX, NL_DEGREE, NL_VECTOR *, NL_INDEX, NL_VECTOR *, NL_INDEX, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvClosedDerivsParamsKnots( NL_VOID *, NL_INDEX, NL_FLAG, NL_VOID *, NL_INDEX, NL_PARAMETER *, NL_KNOTVECTOR *, NL_INDEX, NL_DEGREE, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvDerivsKnots( NL_VOID *, NL_INDEX, NL_FLAG, NL_VOID *, NL_INDEX, NL_VOID *, NL_INDEX, NL_PARAMETER *, NL_KNOTVECTOR *, NL_INDEX, NL_DEGREE, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvLstSqEnds( NL_POINT *, NL_INDEX, NL_VECTOR *, NL_VECTOR *, NL_FLAG, NL_KNOTVECTOR *, NL_DEGREE, NL_CURVE *r, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvPtsNormals( NL_POINT *, NL_VECTOR *, NL_INDEX, NL_DEGREE, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitPeriodicCubic( NL_POINT *, NL_REAL *, NL_INDEX, NL_FLAG, NL_CURVE *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitCrvApproxKnotsAndTangentsTol( NL_POINT *, NL_INDEX, NL_FLAG, NL_VECTOR *, NL_VECTOR *, NL_FLAG, NL_DEGREE, NL_DEGREE, NL_REAL, NL_FLAG, NL_CURVE *, NL_STACKS * );

/* MISC FUNCTIONS:             */
GW_EXPORT NL_FLAG N_FitCrvCPtsFromSrfData( NL_VOID **, NL_INDEX, NL_INDEX, NL_FLAG, NL_INDEX, NL_FLAG, NL_RMATRIX *, NL_CPOINT * );
GW_EXPORT NL_FLAG N_FitCrvDerivsMatrix( NL_VOID *, NL_INDEX, NL_FLAG, NL_DEGREE, NL_KNOTVECTOR *, NL_VOID *, NL_VOID *, NL_RMATRIX *, NL_CPOINT * );
GW_EXPORT NL_FLAG N_FitCrvMatrix( NL_VOID *, NL_INDEX, NL_FLAG, NL_DEGREE, NL_RMATRIX *, NL_CPOINT * );
GW_EXPORT NL_FLAG N_FitCrvDerivMatrix( NL_VOID *, NL_INDEX, NL_FLAG, NL_DEGREE, NL_KNOTVECTOR *, NL_VOID *, NL_FLAG, NL_RMATRIX *, NL_CPOINT * );
GW_EXPORT NL_FLAG N_FitCalcMatrix( NL_KNOTVECTOR *, NL_DEGREE, NL_PARAMETER *, NL_INDEX, NL_FLAG, NL_RMATRIX *, NL_STACKS * );
GW_EXPORT NL_FLAG N_FitSmoothPts( NL_POINT *, NL_INDEX, NL_DEGREE, NL_INDEX, NL_INDEX, NL_FLAG, NL_POINT * );

#endif /* _CRVFIT_H */
