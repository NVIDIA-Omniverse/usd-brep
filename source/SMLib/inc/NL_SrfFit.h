// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* SrfFit.h: Surface Fitting related funcitons                        */
/**********************************************************************/

#ifndef _SRFFIT_H
#define _SRFFIT_H

/* contains :                                                                                              */
/* UV-MAP AND KNOT VECTOR BUILDING: */
/* N_FitCalcSrfParamValues()       compute grid-of-points uv-mapping.                                      */
/* N_FitSrfInterpParams()          compute u or v param values for grid-of-points.                         */
/* N_FitSrfCalcParams()            compute cloud-of-points uv-mapping. Best fit plane.                     */
/* N_FitCalcSrfParamsBoundary()    compute cloud-of-points uv-mapping. Base surface from boundary curves.  */
/* N_FitCalcSrfParamsBoundarySrf() compute cloud-of-points uv-mapping. Given a base surface.               */
/* N_FitSrfCalcKnotVectors()       compute cloud-of-points knot vectors.                                   */
/* N_FitCalcKnotsRandom()          compute cloud-of-points knot vectors.                                   */
                             
/* KNOT REMOVAL:    */
/* N_FitSrfTangentError()          test if single knot removal violates cross-tangent tolerance.           */
/* N_FitSrfRemovalBoundary()       removes one knot from a surface and updates error.                      */
/* N_FitSrfApproxRemoveKnots()     removes max number of knots from surface keeping tolerance.             */
/* N_FitSrfInterpRemoveKnots()     remove knots while interpolating sample-point positions.                */
/* N_FitSrfApproxRemoveKnotsTangents()  remove knots while preserving sample-point positions to tolerance. */

/* INTERPOLATE FUNCTIONS:   */
/* N_FitSrfToPtsKnots()            interpolate grid-of-points. One control point per sample-point.          */
/* N_FitSrfInterpBicubic()         interpolate grid-of-points with a C11 bicubic nurb fill surface.         */
/* N_FitSrfFuncInterp()            interpolate grid-of-points with given uv-mapping. One control point per sample-point. */
/* N_FitSrfInterpVariablePts()     interpolate rows-of-points with nurbs surface of specified degree.       */
/* N_FitSrfInterpBoundary()        interpolate cloud-of-points and boundary curves and                      */
/* N_FitSrfInterpShape()           interpolate cloud-of-points and boundary curves with optional uv-mapping.*/ 

/* APPROXIMATINGE FUNCTIONS:   */    
/* N_FitSrfToPts()                 fit grid-of-points with a nurbs surface of specified degrees.            */
/* N_FitPtsNormals()               git grid-of-points with normals constraints.                             */
/* N_FitSrfApproxTol()             fit grid-of-points to tolerance with specified degrees.                  */
/* N_FitSrfLstSqApprox()           fit grid-of-points to tolerance with specified degrees and control point counts. */
/* N_FitSrfLstSqKnots()            fit grid-of-points with given uv-mapping and knots to specified, degrees and knot vectors. */
/* N_FitSrfInterpTangents()        fit grid-of-points and optional boundary cross-tangents.                 */
/* N_FitSrfApproxTangentsTol()     fit grid-of-points with optional boundary cross-tangent to tolerance.    */ 
/* N_FitSrfToPtsAndBoundary()      fit grid-of-points and boundary curves.                                  */
/* N_FitSrfToVariablePts()         fit rows-of-points with a dense nurbs surface.                           */
/* N_FitSrfApproxShape()           fit cloud-of-points, boundary curves, and optional cross-tangent constraints to a tol. */

/* MISC FUNCTIONS:             */
/* N_FitPlaneToPts()               fit cloud-of-points to best fitting planar patch.                        */
/* N_FitSphereToPtsGlobal()        fit cloud-of-points to best fitting sphere or plane. Global segmenting.  */  
/* N_FitSphereToPtsLocal()         fit cloud-of-points to best fitting sphere or plane. Local segmenting.   */

/* UV-MAP AND KNOT VECTOR BUILDING: */

/* compute grid-of-points uv-mapping using a uniform,  */
/* chordlength, or centripetal rule.                   */
GW_EXPORT NL_FLAG N_FitCalcSrfParamValues( NL_VOID **, NL_INDEX, NL_INDEX, NL_FLAG, NL_FLAG, NL_PARAMETER *, NL_PARAMETER * );
                         
/* compute u or v param values for grid-of-points using   */
/* equal spacing, chordlength, or centripetal rule.       */
GW_EXPORT NL_FLAG N_FitSrfInterpParams( NL_VOID **, NL_INDEX, NL_INDEX, NL_FLAG, NL_FLAG, NL_FLAG, NL_PARAMETER * );

/* compute cloud-of-points uv-mapping by projecting points to a  */
/* best-fit or input plane.                                      */
GW_EXPORT NL_FLAG N_FitSrfCalcParams( NL_POINT *, NL_INDEX, NL_FLAG, NL_POINT, NL_VECTOR, NL_VECTOR, NL_VECTOR, NL_PARAMETER, 
                                      NL_PARAMETER, NL_PARAMETER, NL_PARAMETER, NL_PARAMETER *, NL_PARAMETER * );
                         
/* compute cloud-of-points uv-mapping by mapping to a base surface  */
/* made as a blended coons patch from given boundary curves.        */
GW_EXPORT NL_FLAG N_FitCalcSrfParamsBoundary( NL_POINT *, NL_INDEX, NL_CURVE **, NL_PARAMETER *, NL_PARAMETER *, 
                                              NL_INDEX *, NL_INDEX **, NL_INDEX **, NL_STACKS * );
                         
/* compute cloud-of-points uv-mapping by mapping to a base  */
/* surface.  Base surface can be given or computed from     */
/* boundary curves.  */
GW_EXPORT NL_FLAG N_FitCalcSrfParamsBoundarySrf( NL_POINT *, NL_INDEX, NL_CURVE **, NL_CURVE **, NL_FLAG, 
                                                 NL_FLAG, NL_REAL, NL_REAL, NL_INDEX, NL_INDEX, NL_SURFACE *, 
                                                 NL_POINT **, NL_REAL **, NL_REAL **, NL_INDEX *, NL_STACKS * );
                         
/* set knot values of sized knot vectors based on a cloud of  */
/* points parameter data.                                     */
GW_EXPORT NL_FLAG N_FitSrfCalcKnotVectors( NL_REAL *, NL_REAL *, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_FLAG, NL_REAL, NL_REAL, 
                                           NL_REAL, NL_REAL, NL_KNOTVECTOR *, NL_KNOTVECTOR *, NL_KNOTVECTOR *, NL_KNOTVECTOR * );
                
GW_EXPORT NL_FLAG N_FitCalcKnotsRandom( NL_REAL *, NL_REAL *, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_INDEX, NL_INDEX, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_FLAG, NL_REAL *, NL_REAL *, NL_INDEX, NL_INDEX, NL_REAL, NL_KNOTVECTOR *, NL_KNOTVECTOR * );
                             
/* KNOT REMOVAL:    */

/* test whether the removal of one knot will change a surface                */
/* croos-derivative at specifed uv params by more than specified tolerance.  */
GW_EXPORT NL_FLAG N_FitSrfTangentError( NL_SURFACE *, NL_SURFACE *, NL_INDEX, NL_INDEX, NL_FLAG, NL_FLAG, NL_FLAG, NL_VECTOR ***, 
                                        NL_VECTOR ***, NL_REAL *, NL_REAL *, NL_INDEX, NL_INDEX, NL_REAL, NL_REAL, NL_FLAG * );

/* removes one knot from a surface and updates error  */
/* bound to a set of sample points.                   */
GW_EXPORT NL_FLAG N_FitSrfRemovalBoundary( NL_SURFACE *, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_FLAG, NL_REAL **, NL_REAL * );

/* removes max number of knots from surface while keeping    */
/* surface within tolerance of a given set of sample points. */
GW_EXPORT NL_FLAG N_FitSrfApproxRemoveKnots( NL_SURFACE *, NL_REAL *, NL_REAL *, NL_REAL **, NL_INDEX, NL_INDEX, NL_REAL, NL_FLAG );

/* remove knots while interpolating positions at  */
/* given parameter points.                        */
GW_EXPORT NL_FLAG N_FitSrfInterpRemoveKnots( NL_SURFACE *, NL_REAL *, NL_REAL *, NL_INDEX, NL_INDEX, NL_REAL, NL_REAL, NL_REAL );

/* remove knots while preserving position tolerances at given  */
/* parameter points and opt tangent constraints on boundaries.      */
GW_EXPORT NL_FLAG N_FitSrfApproxRemoveKnotsTangents( NL_SURFACE *, NL_REAL *, NL_REAL *, NL_VECTOR ***, NL_VECTOR ***, NL_INDEX, 
                                                    NL_INDEX, NL_FLAG, NL_FLAG, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL );


/* INTERPOLATE FUNCTIONS:   */
     
/* interpolate grid-of-points with given uv-mapping to         */
/* a nurbs surface with one control point for each grid point. */
GW_EXPORT NL_FLAG N_FitSrfToPtsKnots( NL_VOID **, NL_INDEX, NL_INDEX, NL_FLAG, NL_PARAMETER *, NL_PARAMETER *, NL_KNOTVECTOR *, 
                                      NL_KNOTVECTOR *, NL_DEGREE, NL_DEGREE, NL_SURFACE *, NL_STACKS * );
                         
/* interpolate grid-of-points with a C11 bicubic  */
/* nurbs fill of the data.                           */
GW_EXPORT NL_FLAG N_FitSrfInterpBicubic( NL_POINT **, NL_INDEX, NL_INDEX, NL_FLAG, NL_SURFACE *, NL_STACKS * ); 

/* interpolate grid-of-points with given uv-mapping to a nurbs   */
/* surface function with one control point for each grid point.  */
GW_EXPORT NL_FLAG N_FitSrfFuncInterp( NL_REAL **, NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_PARAMETER *, 
                                      NL_PARAMETER *, NL_SFUN *, NL_STACKS * );
                         
/* interpolate rows-of-points with nurbs surface of specified degree */
/* and a param that trades off interpolant quality with number       */
/* of control points */
GW_EXPORT NL_FLAG N_FitSrfInterpVariablePts( NL_POINT **, NL_INDEX, NL_INDEX *, NL_DEGREE, NL_DEGREE, NL_REAL, 
                                             NL_CURVE **, NL_CURVE **, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
                         
/* interpolate cloud-of-points and boundary curves and  */
/* optional uv-mapping with nurbs surface.              */
GW_EXPORT NL_FLAG N_FitSrfInterpBoundary( NL_POINT **, NL_INDEX, NL_INDEX, NL_CURVE **, NL_CURVE **, 
                                          NL_REAL *, NL_REAL *, NL_SURFACE *, NL_STACKS * );
                         
/* interpolate cloud-of-points, boundary curves, and optional  */
/* cross-tangent boundary constraints to a tolerance           */
/* with a nurbs surface algorithm finds uv-mapping,            */
/* knot vectors, and control points using input boundary       */
/* curve properties as a start.                                */
GW_EXPORT NL_FLAG N_FitSrfInterpShape( NL_POINT *, NL_INDEX, NL_CURVE **, NL_CURVE **, NL_CURVE **, NL_CURVE **, 
                                       NL_FLAG, NL_SURFACE *, NL_STACKS *, NL_STACKS * );

/* APPROXIMATINGE FUNCTIONS:       */
                             
/* fit grid-of-points to a nurbs surface of specified  */
/* degrees. The algorithm finds the uv-mapping,        */
/* knotVectors, and controlPoint counts.               */
GW_EXPORT NL_FLAG N_FitSrfToPts( NL_POINT **, NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_FLAG, NL_SURFACE *, NL_STACKS * );
                         
/* fit grid-of-points with optional surface normal constraints.                 */
/* The algorithm finds the uv-mapping, knotVectors, and controlPoint counts.    */
GW_EXPORT NL_FLAG N_FitPtsNormals( NL_POINT **, NL_VECTOR **, NL_INDEX, NL_INDEX, NL_FLAG, NL_SURFACE *, NL_STACKS * ) ;

/* fit grid-of-points to given tolerance with nurb surface  */
/* of specified degrees. The algorithm finds uv-mapping,    */
/* knotVectors, and ControlPoint counts.                    */
GW_EXPORT NL_FLAG N_FitSrfApproxTol( NL_POINT **, NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_DEGREE, NL_DEGREE, NL_REAL, 
                                     NL_FLAG, NL_FLAG, NL_SURFACE *, NL_STACKS * );

/* fit grid-of-points to a nurbs surface of specified    */
/* degrees and controlPoint counts. The algorithm finds  */
/* the uv-mapping and knot vectors.                      */
GW_EXPORT NL_FLAG N_FitSrfLstSqApprox( NL_POINT **, NL_INDEX, NL_INDEX, NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_FLAG, 
                                       NL_SURFACE *, NL_STACKS * );
                         
/* fit grid-of-points with given uv-mapping to a nurbs  */
/* surface of specified degrees, knotVectors,           */
/* and controlPoint counts.                             */
GW_EXPORT NL_FLAG N_FitSrfLstSqKnots( NL_VOID **, NL_INDEX, NL_INDEX, NL_FLAG, NL_PARAMETER *, NL_PARAMETER *, NL_KNOTVECTOR *, 
                                      NL_KNOTVECTOR *, NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_SURFACE *, NL_STACKS * );
                         
/* fit grid-of-points with optional boundary cross-tangent     */
/* direction or vector constraints (i.e. magnitude either      */
/* free or fixed) with a nurbs surface.                        */
/*  The uv-mapping may be given or computed by the algorithm.  */
/*  The knotVectors may be given or computed by the algorithm. */
/*  The controlPoint counts are found by the algorithm.        */
GW_EXPORT NL_FLAG N_FitSrfInterpTangents( NL_POINT **, NL_INDEX, NL_INDEX, NL_PARAMETER *, NL_PARAMETER *, NL_KNOTVECTOR *, NL_KNOTVECTOR *, 
                                          NL_DEGREE, NL_DEGREE, NL_VECTOR ***, NL_VECTOR ***, NL_FLAG, NL_SURFACE *, NL_STACKS * );

/* fit grid-of-points with optional boundary cross-tangent     */
/* constraints with a nurbs surface of specified degrees. The  */
/* algorithm finds the uv-mapping and control point counts.    */
GW_EXPORT NL_FLAG N_FitSrfApproxTangentsTol( NL_POINT **, NL_INDEX, NL_INDEX, NL_DEGREE, NL_DEGREE, NL_VECTOR ***, NL_VECTOR ***, 
                                             NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_REAL, NL_SURFACE *, NL_STACKS * );

/* fit grid-of-points with nurbs surface defined by given        */
/* boundary curves, optionally allow extra knots to tighten fit  */
GW_EXPORT NL_FLAG N_FitSrfToPtsAndBoundary( NL_POINT **, NL_INDEX, NL_INDEX, NL_REAL *, NL_REAL *, NL_CURVE **, 
                                            NL_CURVE **, NL_INDEX, NL_INDEX, NL_INDEX, NL_SURFACE *, NL_STACKS * );
                         
/* fit rows-of-points with a dense nurbs surface of specified   */
/* degree followed by knot removal algorithm finds uv-mapping,  */
/* knotVectors, and controlPoint counts */
GW_EXPORT NL_FLAG N_FitSrfToVariablePts( NL_POINT **, NL_INDEX, NL_INDEX *, NL_DEGREE, NL_DEGREE, NL_REAL, 
                                         NL_REAL, NL_REAL, NL_REAL, NL_CURVE **, NL_CURVE **, NL_SURFACE *, NL_STACKS *, NL_STACKS * );
                         
/* fit cloud-of-points, boundary curves, and optional     */
/* cross-tangent boundary constraints to a tolerance      */
/* with a nurbs surface algorithm finds uv-mapping,       */
/* knot vectors, and control points using input boundary  */
/* curve values as a start                                */
GW_EXPORT NL_FLAG N_FitSrfApproxShape( NL_POINT *, NL_INDEX, NL_CURVE **, NL_CURVE **, NL_CURVE **, NL_CURVE **, 
                                       NL_REAL, NL_SURFACE *, NL_FLAG *, NL_STACKS *, NL_STACKS * );

/* MISC FUNCTIONS: */

/* fit cloud-of-points to best fitting planar patch */
GW_EXPORT NL_FLAG N_FitPlaneToPts( NL_POINT *, NL_INDEX, NL_REAL, NL_SURFACE *, NL_REAL *, NL_REAL *, NL_STACKS * );
                         
/* fit cloud-of-points to best fitting sphere or  */
/* planar patch, sphere segmented globally        */
GW_EXPORT NL_FLAG N_FitSphereToPtsGlobal( NL_POINT *, NL_INDEX, NL_REAL, NL_FLAG, NL_FLAG, NL_REAL, NL_FLAG, 
                                          NL_FLAG, NL_SURFACE *, NL_FLAG *, NL_REAL *, NL_REAL *, NL_STACKS * );
                         
/* fit cloud-of-points to best fitting sphere or  */
/* planar patch, sphere segmented locally         */
GW_EXPORT NL_FLAG N_FitSphereToPtsLocal( NL_POINT *, NL_INDEX, NL_REAL, NL_FLAG, NL_FLAG, NL_REAL, NL_FLAG, 
                                         NL_FLAG, NL_SURFACE *, NL_FLAG *, NL_REAL *, NL_REAL *, NL_STACKS * );
    
#endif /* _SRFFIT_H */
