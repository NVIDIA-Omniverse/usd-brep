// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* SrfLstSqFit.c: Surface fitting routines using least squares        */
/**********************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>
#include <NL_Gauss.h>

#include <NL_BasisAdv.h>    /* Advanced NL_KNOTVECTOR functions */

static NL_REAL ST_calcSurfSize(NL_INDEX numU, NL_INDEX numV, NL_CPOINT** ctrlPts);

static NL_FLAG ST_findStep(NL_SURFACE* srfPtr, NL_FLAG fixEdges_u,                                             /* are the surface edges allowed to move?  */
    NL_FLAG fixEdges_v, NL_INDEX numDataPoints, NL_POINT xPts[], NL_REAL xParams_u[], /* must be initialized to previous-step values */
    NL_REAL xParams_v[], NL_REAL alpha, NL_VECTOR** ctrlPtMoves,                      /* the main output of this routine */
    NL_REAL* errorDist, NL_REAL* errorSD, NL_REAL allErrors[]);

static NL_BOOLEAN ST_doneIterating(NL_INDEX iter, NL_INDEX maxIter, NL_REAL errorSD, NL_REAL errorDist, NL_REAL errorStop,
    NL_REAL surfSize, NL_INDEX numU, NL_INDEX numV, NL_VECTOR** ctrlPtMoves);

static NL_FLAG ST_GetRowsColMatrix1(NL_INDEX, NL_INDEX, NL_REAL*);

/* Local constants for SrfLeastSquaresFit.c */
#define MAXITER 100

/* Prototypes only referenced within SrfLstSqFit.c */


/* static NL_FLAG ST_GetRowsColMatrix2( NL_INDEX, NL_INDEX, NL_REAL * ); */
/* static NL_FLAG ST_GetRowsColMatrix3( NL_INDEX, NL_INDEX, NL_REAL * ); */


NL_VOID N_AddStiffnessToSrfAMatrix
  (NL_REAL     alpha,          /* in : Resistance to stretch weight term              */
   NL_REAL     beta,           /* in : Resistance to beta weight term                 */
   NL_SURFACE *pSurface,       /* in : Surf making the stiffness matrix               */
   NL_REAL   **A,              /* i/o: A matrix that recieves new terms.              */
                               /*      sized:[nu x nu]                                */
   NL_INDEX  **OptMap,         /* in : Index mapping: GlobalDofIndex->AIndex          */
                               /*      sized:[  pSurface->UControlPointCount          */
                               /*             x pSurface->VControlPointCount]         */
                               /*      NULL to ignore,                                */
                               /*      when NULL    nu = N,                           */
                               /*      when NotNUll nu = OptNu                        */
                               /*      N =   pSurface->UControlPointCount             */
                               /*          * pSurface->VControlPointCount             */
   NL_CPOINT **OptSw,          /* in : Constrained DOF values, unconstrained          */
                               /*      are not used - but conceptually zero.          */
                               /*      sized:[  pSurface->UControlPointCount          */
                               /*             x pSurface->VControlPointCount]         */
   NL_POINT   *OptNrhs,        /* in : Rhs to be augmented by constrained DOF values  */
                               /*      sized:[OptNu]                                  */
   NL_INDEX   *OptNu) ;        /* in : Size Param for constrained DOF mapping,        */
                               /*      sizeof OptMap and OptNrhs.                     */             


/* static variables to facilitate static functions */
NL_PRIVATE_TLS NL_REAL ** glo_a;
NL_PRIVATE_TLS NL_INDEX ** glo_cind;
NL_PRIVATE_TLS NL_INDEX ** glo_uvsp;
NL_PRIVATE_TLS NL_INDEX glo_me;
NL_PRIVATE_TLS NL_INDEX glo_nu;
NL_PRIVATE_TLS NL_DEGREE glo_p;
NL_PRIVATE_TLS NL_DEGREE glo_q;
/*******************************************************************//**

   DESCRIPTION:

   This fitting routine improves the fit of a given NURBS surface to
   a given set of points.  The input points need not be ordered, and
   parameter values for them are not required.  An input surface
   (non rational) must be provided as an initial guess.

   The edges of the surface may be fixed if desired.  U- and v-edges
   may be fixed independently.  If the flags are greater than zero,
   the corresponding edges will be fixed.  (For terminology, "u-edges"
   means the edges along which u varies, i.e., v==0 and v==1.)

   An optional argument may be passed in, to return the error for
   each data point.  The values are the perpendicular distance from
   the data point to the surface.  If not NULL, this array must be
   allocated by the caller, to the same size as dataPoints.

   Constants for a regularization term may be passed in.
   Regularization helps to maintain a fair surface, and is used because,
   in theory,the very "best" surface for the data, in terms of minimizing
   the distance to each data point, would be full of kinks and loops.
   The regularization term used is a simplified thin plate energy.
   It works best if its weighting term "alpha" decreases as the iteration
   proceeds.  Therefore, starting and ending values may be passed into
   the routine.  If the passed-in values are negative, default values
   will be used.

   Smaller values of alpha will result in a better fit, but the surface
   might develop kinks or loops.  The values should not be large: the
   current defaults are alpha_0 = 0.005 and alpha_1 = 0.001.

   Unfortunately, the regularization is specific to bicubic surfaces.
   If the input surface is not bicubic, regularization will not be
   applied, and the resulting surface might not be "fair".

   A typical calling example is as follows:

     NL_POINT   dataPoints[];
     NL_INDEX   dataPointCount;
     NL_SURFACE *srfPtr;
     NL_FLAG  fixEdges_u, fixEdges_v;
     NL_REAL  alpha_0, alpha_1;
     NL_REAL  allErrors[];
     ...
     (get dataPoints, srfPtr)
     ...
     N_FitSurfApproxPoints( dataPoints, dataPointCount, srfPtr,
       fixEdges_u, fixEdges_v,
       alpha_0, alpha_1, allErrors );


  ACCESS:

   dataPoints    , input ,  array of data points to be fit
   dataPointCount, input ,  highest index in dataPoints
   srfPtr        , input/output ,  approximating surface
   fixEdges_u    , input ,  > 0: fix the positions of the surface's u-edges
                            <= 0: no constraints on u-edges.
   fixEdges_v    , input ,  same for the surface's v-edges
   alpha_0, alpha_1, input, start and end values of alpha regularization term,
                            use default values if negative.
   allErrors       , output , error values for each data point, or NULL


  RETURN CODES:

   0 : No error
   1 : Error saved in NL_ERROR


  Possible Enhancements:
- The regularization is done only for bicubics.  The linear case (in either
  direction) should be added, to allow the user to use ruled surfaces.

  ***********************************************************************/

/* ----------------------------------------------------------------- */
/**********************************************************************/
/* Comments on the algorithms.  Moved here to keep the code cleaner.  */
/**********************************************************************/

/*
    Setting up the A-matrix
    -----------------------

   First, for readability in this discussion, we'll assume that the surface
   is bicubic.  If not, then change each "four" to "degree+1" and each
   "12" to "(degree+1)*dimension".

   Each entry of the A matrix is basis functions times dot-product-like
   products of components of the three local basis vectors.  Since each
   coordinate of the point-shift vectors is an independent unknown, we do
   it all coordinate by coordinate.  And since it's cubic, at any point,
   only a four-by-four square of control point (shifts) are affected.
   (Indicated by the 'span' argument from N_BasisEval.)  So for a bicubic,
   this loop will increment four groups of 12 consecutive entries in the
   b-vector (four points, three coords), and a 12 x 12 square in the A matrix.

   There are only nine combinations of the dot-product-like products.
   If s == sdTerm, N is vec_e3, etc:
  s*Tx*Tx + Ox*Ox + Nx*Nx;
  s*Tx*Ty + Ox*Oy + Nx*Ny;  s*Tx*Tz + Ox*Oz + Nx*Nz
  s*Ty*Tx + Oy*Ox + Ny*Nx;
  s*Ty*Ty + Oy*Oy + Ny*Ny;  s*Ty*Tz + Oy*Oz + Ny*Nz
  s*Tz*Tx + Oz*Ox + Nz*Nx;
  s*Tz*Ty + Oz*Oy + Nz*Ny;  s*Tz*Tz + Oz*Oz + Nz*Nz
   Since it's symmetrical, there are actually only six different ones.
   We'll calculate those before the loop.  If we call them SMat[i][j]
   (since they come from the Surface evaluation), the entries of A will be:

  B0*B0*S00 B0*B0*S01 B0*B0*S02  B0*B1*S00 B0*B1*S01 B0*B1*S02  B0*B2*S00  ...
  B0*B0*S10 B0*B0*S11 B0*B0*S12  B0*B1*S10 B0*B1*S11 B0*B1*S12  B0*B2*S10  ...
  B0*B0*S20 B0*B0*S21 B0*B0*S22  B0*B1*S20 B0*B1*S21 B0*B1*S22  B0*B2*S20  ...

  B1*B0*S00 B1*B0*S01 B1*B0*S02  B1*B1*S00 B1*B1*S01 B1*B1*S02  B1*B2*S00  ...
  B1*B0*S10 B1*B0*S11 B1*B0*S12  B1*B1*S10 B1*B1*S11 B1*B1*S12  B1*B2*S10  ...
  B1*B0*S20 B1*B0*S21 B1*B0*S22  B1*B1*S20 B1*B1*S21 B1*B1*S22  B1*B2*S20  ...

  B2*B0*S00 B2*B0*S01 B2*B0*S02  B2*B1*S00 B2*B1*S01 B2*B1*S02  B2*B2*S00  ...
  ...

   where B0, B1, B2 and B3 are the cubic basis functions.

   Note that we have to do each coordinate separately, because the
   coordinates mix together (e.g., s*Ty*Tx + Oy*Ox + Ny*Nx).  This means
   that we can't do it as in N_FitCrvApproxLstSq, where the size of the matrix is
   related to the number of variable control points, and the matrix
   solvers (N_RealMatrixLuDecompose and N_RealMatrixForBack) work with CPOINTs in the x- and
   B-vectors.  We must use a[3*numPpoints] square matrix, and
   use N_RealMatrixLuDecomposePivot and N_RealMatrixForBackPivot, as in N_RealMatrixLstSqSolve.


    The Loops
    ---------
  For each data point, we loop over all of the affected control points,
  which is a 4x4 square (for bicubics) in the surface grid.  The control
  point grid is of course linearized in the B- and x-vectors, so that
  a 4x4 grid on the surface becomes four separate bunches of four
  consecutive entries in B and x.  Then since the A matrix is essentially
  the B-vector squared, this becomes a 4x4 grid of 4x4-point chunks in A.
  And of course since the coordinates (x,y,z) are intermixed, everything
  is multiplied by three.

    For k each data pt
      get surface data

      For i1 each u-row of ctrl pts in span (which affect this data pt)
        For j1 each ctrl pt in v-col of this u_row
          For rowCoord (x,y,z)
            B-vec[i1*numV + j1 + rowCoord] = ...

            For i2 each u-row of ctrl pts in span
              For j2 each ctrl pt in v-col of this u_row
                For colCoord (x,y,z)
                  A-mat[i1*numV + j1 + rowCoord][i2*numV + j2 + colCoord] = ...

  This is the same as for curves except that the two double-loops
  "For i1 ... For j1" and "For i2 ... For j2" are, for curves,
  single loops "For i" and "For j".


    Matrices: Fixing the Edge Points
    --------------------------------

  If the edges are to be fixed, then the point moves returned from
  findStep should be zero for the control points on two or four edges.
  Leaving these out of the matrix complicates the indexing and logic.
  We could use the full matrix, and ensure that the fixed point moves
  (in the x-vector) are zero, by setting the corresponding diagonal
  element in A to 1, zeroing out the corresponding row and column in A,
  as well as the corresponding element in B.  However, that can mean
  much bigger matrices.  For example, for a surface with 6 x 10 control
  points with all edges fixed, the full matrix would be 180 x 180, but
  if we leave out the edge elements, the actual matrix that is used is
  96 x 96.  This would mean solving a matrix with 32,400 entries of which
  23,184 are 0's and 1's, versus solving the reduced, dense matrix with
  9,216 entries.  So it's worth messing around with the indexing to leave
  out the zero entries.


    Regularization Term
    -------------------

  We're minimizing the distances from the data points to the surface.
  This just means that we're making a surface that goes close to each point;
  it doesn't say anything about what else the surface does.  The surface may,
  and often does, develop kinks or loops, or fly far away and come back.
  The purpose of the regularization term is to prevent that, and keep
  the surface close to the data points.

  The regularization term used is that suggested by Pottmann and
  Leopoldseder in the "Concept" paper: Suu^2 + 2*Suv^2 + Svv^2,
  which is the "simplified thin plate energy", and works quite well
  in practice.  It works best if its weighting term "alpha" decreases
  as the iteration proceeds.  Therefore, starting and ending values may
  be passed into the routine.  If the passed-in values are negative,
  default values will be used.


    Integrating the Regularization Term
    -----------------------------------

  When written out in terms of control points and basis functions,
  the thin plate energy functional for a bicubic B-Spline surface is:
  the double integral, over the span, of:

             1    3                      i + 2     i + 1    i
       6 * SUM  SUM[N1i(u) * N3j(v) ( p    -2*p    + p )] ^2
           i = 0  j = 0                      j       j      j

             2    2                      i + 1    i + 1    i      i
 + 2 * 9 * SUM  SUM[N2i(u) * N2j(v) ( p    - p    - p    + p )] ^2
           i = 0  j = 0                      j + 1    j      j + 1    j

             3    1                      i       i      i
     + 6 * SUM  SUM[N3i(u) * N1j(v) ( p    -2*p    + p )] ^2
           i = 0  j = 0                      j + 2     j + 1    j

  For our application, each point p_ij is actually c_ij + d_ij, where
  c_ij are the current control points, and d_ij are the displacements
  that we're trying to find.  Writing them out like this and differentiating
  with respect to the displacements gives the linear system of equations.
  The b-vector term for a row is the A-matrix row times the column of
  current control points.

  The A-matrix coefficients are then double integrals of products of
  B-Spline basis functions.  Fortunately they're separable into u and
  v, but unfortunately, they include integrals of 6th degree polynomials.
  The integration is done using a 4-point Gaussian integration, which is
  precise to degree 7.  The values of the basis functions are first
  calculated at the 4x4 Gauss abscissae, then multiplied and added together
  according to the deriviatives of the energy functional, above.

  Unfortunately, the calculations are specific to cubics.  Other degrees
  would have to be done separately.  Currently, if the surface is not
  bicubic, these corrections cannot be applied.  It would probably make
  sense to add the linear case: it would not be too difficult, and would
  allow users to use ruled surfaces.

*/
/* ----------------------------------------------------------------- */

NL_FLAG N_FitSurfApproxPoints  
  (NL_POINT     xPts [],       /* in : sample points, sized:[xPtsCount+1] */
   NL_INDEX     xPtsCount,     /* in : size argument          */
   NL_SURFACE  *srfPtr,        /* i/o: approximating surface  */
   NL_FLAG      fixEdges_u,    /* in : 1 = fix srfPtr u edges */
                               /*      0 = don't              */
   NL_FLAG      fixEdges_v,    /* in : 1 = fix srfPtr v edges */
                               /*      0 = don't              */
   NL_REAL      alpha_0,       /* in : start alpha regularization term, -1 = use default value, only used for bicubic surfaces */
   NL_REAL      alpha_1,       /* in : end   alpha regularization term, -1 = use default value, only used for bicubic surfaces */
   NL_REAL      allErrors [] ) /* out: opt error values for each data point, NULL to ignore */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSurfApproxPoints");
    NL_FLAG error = NL_NO;

    NL_INDEX numDataPoints = xPtsCount + 1; /* actual point count */

    NL_INDEX i, j;
    NL_INDEX iter;

    /* error terms: dist squared, and their Squared Error term. */
    NL_REAL errorDist, errorSD;
    NL_REAL errorStop;
    NL_REAL surfSize;

    /* Surface info */
    NL_INDEX numCtrlPts_u, numCtrlPts_v;
    NL_CPOINT ** ctrlPts;

    NL_CPOINT ** bestCtrlPts; /* Save the best results, in case we diverge. */
    NL_REAL bestErrorSD = NL_BIGD;
    /* NL_REAL bestErrorDist = NL_BIGD; */
    NL_REAL *bestAllErrors = NULL;

    /* Points and parameters on the surface of data points (dropped) */
    NL_REAL *xParams_u;
    NL_REAL *xParams_v;
    NL_POINT *projPts;
    NL_INDEX projCount;        /* (because of the way N_gsrpsp works) */

    NL_VECTOR ** ctrlPtMoves; /* calculated on each iteration */

    NL_REAL damping;
    NL_REAL iterFrac;

    NL_STACKS localStacks;

    NL_REAL alpha;

    /* Start of executable code */

    N_InitNurbs( &localStacks );

    /* Check whether the caller provided a starting surface. */
    /* If not, we don't do anything (for now anyway).  */
    if( N_SrfAreArraysNULL( srfPtr ) == NL_TRUE )
        NL_QUIT;

    /* get surface control points */
    N_SrfGetCPts( srfPtr, &numCtrlPts_u, &numCtrlPts_v, &ctrlPts );

    /* Rename these variables as actual counts instead of "highest index":  */
    numCtrlPts_u++;
    numCtrlPts_v++;

    /* We should have a surface at this point. */
    if( numCtrlPts_u < 2 || numCtrlPts_v < 2 )
        return NL_YES;

    /* Project all data points to the surface, and get starting uv parameters */

    /* Note: for the initial projection to the surface, we're currently
    * using N_SrfProjectPts.  It's not exactly what we want, but seems to be
    * the closest thing in NLib.
    * For one thing, it says it requires the surface and points to be
    * "simple", which means generally flat, able to be projected to a
    * plane without overlap.  On preliminary tests, however, it seems
    * to work well enough on a closed surface.  We should write something
    * analogous to the curve routine N_CrvClosestPtMultiple.
    */
    error = N_SrfProjectPts( srfPtr, xPts, xPtsCount, NL_NO, NL_NO, 0.001, &projPts, &xParams_u, &xParams_v, &projCount, &localStacks );

    if( error == NL_YES )
            NL_OUT;

    /* Allocate memory for various things. */

    /* Note: N_SrfProjectPts allocates xParams_u and xParams_v itself. */

    ctrlPtMoves = N_AllocPt2dArray( numCtrlPts_u - 1, numCtrlPts_v - 1, &localStacks );

    if( ctrlPtMoves == NULL )
        NL_ERROR( NL_MEM_ERR );

    if( allErrors != NULL )
    {
        bestAllErrors = N_AllocReal1dArray( numDataPoints - 1, &localStacks );
    }

    bestCtrlPts = N_AllocCPt2dArray( numCtrlPts_u - 1, numCtrlPts_v - 1, &localStacks );

    if( bestCtrlPts == NULL )
        NL_ERROR( NL_MEM_ERR );

    /* do we need to use default values for alpha? */
    if( alpha_0 < 0 )
        alpha_0 = 0.005;

    if( alpha_1 < 0 )
        alpha_1 = 0.001;

    errorStop = NL_MTOL * numDataPoints; /* pretty tight: each pt within tol  */

    errorSD = errorDist = 2 * errorStop; /* just to get it started. */

    /* Calculate a characteristic size, for a stopping criterion:  */
    /* if the control point moves get really tiny we can quit.     */
    surfSize = ST_calcSurfSize( numCtrlPts_u, numCtrlPts_v, ctrlPts );

    iter = 0;

    while( !ST_doneIterating( iter, MAXITER, errorSD, errorDist, errorStop, surfSize, numCtrlPts_u, numCtrlPts_v, ctrlPtMoves ) )
    {
        iter++;

        /* Set up alpha: decrease as we proceed. */
        if( iter < 40 )
        {
            iterFrac = iter / 40.0;
            alpha = alpha_0 + iterFrac * (alpha_1 - alpha_0);
        }
        else
            alpha = alpha_1;

        error = ST_findStep( srfPtr, fixEdges_u, fixEdges_v, numDataPoints, xPts, xParams_u, xParams_v, /* Updated at each step */
                             alpha, ctrlPtMoves, &errorDist, &errorSD, allErrors );                     /* error measures returned */
        

        if( error != NL_NO )
            break; /* findStep() will have set NL_ERROR */

        /*  Note, we're using errorSD to check progress, but we're returning
        *  errorDist.  That makes sense, because errorSD is what we're
        *  minimizing, and errorDist is what the caller is interested in.
*/
        if( errorSD < bestErrorSD )
        {
            bestErrorSD = errorSD;
            /* bestErrorDist = errorDist; */

            for ( i = 0; i < numCtrlPts_u; i++ )
                for ( j = 0; j < numCtrlPts_v; j++ )
                    bestCtrlPts[i][j] = ctrlPts[i][j];

            if( allErrors != NULL && bestAllErrors != NULL )
            {
                for ( i = 0; i < numDataPoints; i++ )
                    bestAllErrors[i] = allErrors[i];
            }
        }

        /* Now update the control points according to the point moves: */
        damping = (iter < 40) ? (NL_REAL)iter / 40.0 : 1.0;

        for ( i = 0; i < numCtrlPts_u; i++ )
        {
            for ( j = 0; j < numCtrlPts_v; j++ )
            {
                ctrlPts[i][j].x += damping * ctrlPtMoves[i][j].x;
                ctrlPts[i][j].y += damping * ctrlPtMoves[i][j].y;
                ctrlPts[i][j].z += damping * ctrlPtMoves[i][j].z;
            }
        }
    }

    /* Done iterating.  See whether we ended up diverging */

    if( errorSD > bestErrorSD )
    {
        for ( i = 0; i < numCtrlPts_u; i++ )
            for ( j = 0; j < numCtrlPts_v; j++ )
                ctrlPts[i][j] = bestCtrlPts[i][j];

        if( allErrors != NULL && bestAllErrors != NULL )
        {
            for ( i = 0; i < numDataPoints; i++ )
                allErrors[i] = bestAllErrors[i];
        }
    }

    EXIT:
    /*  Clean up  */
    N_EndNurbs( &localStacks );

    return error;

} /* end N_FitSurfApproxPoints */

/**********************************************************************/
/* N_FitRandomPN: Least squares surface fit to random points and normals */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS surface interpolating a given
     list of points  and normals  at  those points.  The degree (p, q)  
     must be  2 or 3.  If the output surface is initialized to the NULL 
     surface, memory is allocated locally. Otherwise  it is checked if 
     enough memory is passed in.
     A typical calling example is:

       NL_POINT       *P;
       NL_VECTOR      *N;
       NL_INDEX       np;
       NL_INDEX       n, m;
       NL_DEGREE      p, q;    
       NL_SURFACE     sur;
       NL_STACKS      SG;
       ...
       (get points, normals, and choose the degree 2 or 3 
        and choose the number of control points);
       ...
       N_SrfInitArrays(&cur);
       N_FitRandomPN(P , N , np, n, m, p, q, &sur, &SG);

   ACCESS:
   
     P    , input  ,  NL_POINT  data as interpolation points
     N    , input  ,  NL_VECTOR as unit normal at interpolation points
     np   , input  ,  Highest index of pts and nrm
     n, m  , input  ,  Highest indexes of the surface control points
                      (the surface will have(n + 1)x(m + 1) control points
     p, q , input  ,  Degree in u, v   (2 or 3)
     sur  , output ,  Interpolating surface
     SG   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_FitRandomPN
  ( NL_POINT   *P,                   /* Sample Points to fit, sized:[np+1] */
    NL_VECTOR  *N,                   /* Sample Point Normals, sized:[np+1] */
    NL_INDEX    np,                  /* size param */
    NL_INDEX    n,                   /* Output Surface U dir control point count */
    NL_INDEX    m,                   /* Output Surface V dir control point count */
    NL_DEGREE   p,                   /* Output Surface U dir degree */
    NL_DEGREE   q,                   /* Output Surface V dir degree */
    NL_SURFACE *sur,                 /* Output Surface */
    NL_STACKS  *SG )                 /* sur memory stack */
{
  /* pass the call along */
  return( N_FitRandomPNWithPlane
           (P,                   /* Sample Points to fit, sized:[np+1] */
            N,                   /* Sample Point Normals, sized:[np+1] */
            np,                  /* size param */
            NULL, NULL, NULL,    /* without the optional projection plane origin, X axis, and Y axis */
            n,                   /* Output Surface U dir control point count */
            m,                   /* Output Surface V dir control point count */
            p,                   /* Output Surface U dir degree */
            q,                   /* Output Surface V dir degree */
            sur,                 /* Output Surface */
            SG ) ) ;             /* sur memory stack */

} /* end N_FitRandomPN */


/**********************************************************************/
/* N_FitRandomPNWithPlaneToTol: Least squares surface fit to random points and normals */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS surface approximating a given
     list of points and normals to a specified tolerance.
     
     This algorithm is order(n**3) in the number of control points,
     so a limit has been added that stops the approximation algorithm
     when the approximation surface gets to be degree 4 and has 52x52
     control points.  At which time the best approximation found up
     until that moment is returned. 
     
     The approximation is built in two phases, compute a UV value for
     every data point then iterate an approximation to the data to 
     meet the given tolerance requirement.

     Computing the UV values for the data points begins by projecting
     each DataPoint to a plane.  The plane may be specified or is found
     by examining the data.  Then a surface of the input degree and
     control point counts is fitted to the position intput data.
     That 1st approximation surface is used to build the UV mapping by
     assigning every data point a UV value based on where it projects
     to this 1st approximating surface.  The input normal constraints
     are turned into TangentU and TangentV constraints by rotating the
     given surface normal vector 90 degrees in the TangentU and the TangentV
     directions found at the projection point being used as the UV
     value for the associated data point.
     
     Once all the data points are assigned a UV value and each surface
     normal is converted into a pair of U and V tangent constraints,
     an iteration is begun to build an approximating surface to within
     tolerances.  A surface of the given degrees and control point
     counts is fit to the position and tangent constraints.  The max
     distance between the datapoints and the surface is found.  If
     the max Distance is larger than tolerance, then the control
     point counts are doubled in each direction and the larger surface
     is fit again until tolerances are satisfied.
     
     If the output surface is initialized to the NULL 
     surface, memory is allocated locally. Otherwise  it is checked if 
     enough memory is passed in.  A uv-mapping is made by projecting 
     to the given plane, else it's made by projecting to a best fit plane.
     The algorithm fails if the uv domain of any control point
     has no data points that map to it.

     A typical calling example is:

       NL_POINT       *P;
       NL_VECTOR      *N;
       NL_INDEX       np;
       NL_REAL        tol;
       NL_INDEX       n, m;
       NL_DEGREE      p, q;    
       NL_SURFACE     sur;
       NL_STACKS      SG;
       ...
       (get points, normals, and choose the degree 2 or 3 
        and choose the number of control points);
       ...
       N_SrfInitArrays(&cur);
       N_FitRandomPN(P , N , np, tol, n, m, p, q, &sur, &SG);

   ACCESS:
   
     P   ,              input  ,  NL_POINT  data as interpolation points
     N   ,              input  ,  NL_VECTOR as unit normal at interpolation points
     np  ,              input  ,  Highest index of pts and nrm
     tol ,              input  ,  max distance allowed between given points and 
                                  approximating surface, A value of 0.0 means to 
                                  return the initial surface without checking tolerances.
     OptProjectionOrig, input  ,  optional origin of projection plane, NULL=compute plane, NULL to ignore.  
     OptProjectionX,    input  ,  optional X axis of projection plane, NULL to ignore.  
     OptProjectionY,    input  ,  optional Y axis of projection plane, NULL to ignore. 
     n, m ,             input  ,  Highest indexes of the surface control points
                                  (the surface will have(n + 1)x(m + 1) control points
     p, q ,             input  ,  Degree in u, v   (typically 2 or 3)
     sur  ,             output ,  Interpolating surface
     SG   ,             input  ,  sur's memory stack

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
        
NL_FLAG N_FitRandomPNWithPlaneToTol
  ( NL_POINT   *P,                   /* Sample Points to fit, sized:[np+1] */
    NL_VECTOR  *N,                   /* Sample Point Normals, sized:[np+1] */
    NL_INDEX    np,                  /* highest index of arrays P and N    */
    NL_REAL     tol,                 /* max dist between points P and approx srf, 0.0 = ignore */
    NL_POINT   *OptProjectionOrig,   /* optional origin of projection plane, NULL=compute plane, NULL to ignore. */
    NL_VECTOR  *OptProjectionX,      /* optional X axis of projection plane, NULL to ignore. */
    NL_VECTOR  *OptProjectionY,      /* optional Y axis of projection plane, NULL to ignore. */
    NL_INDEX    n,                   /* Output Surface U dir control point count */
    NL_INDEX    m,                   /* Output Surface V dir control point count */
    NL_DEGREE   p,                   /* Output Surface U dir degree */
    NL_DEGREE   q,                   /* Output Surface V dir degree */
    NL_SURFACE *sur,                 /* Output Surface */
    NL_STACKS  *SG )                 /* sur memory stack */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitRandomPNWithPlaneToTol");
    NL_FLAG error;
    NL_VECTOR *Tdiru, *Tdirv, M;
    NL_SURFACE srf;               /* first surface solution used to improve UV mappings */
    NL_INDEX i, j, BigCount ;
    NL_POINT ** PntDer, ** SD;
    NL_REAL     MaxDist, ThisDist ;
    NL_PARAMETER ui, vi;
    NL_REAL tol0 = 1.0E-4;
    NL_REAL tol1 = 1.0E-4;
    NL_STACKS S;                   /* local stack */
    NL_PARAMETER *uu;
    NL_PARAMETER *vv;
    NL_POINT Origin;
    NL_VECTOR X, Y, Z;
    NL_FLAG   planeFlag ;

    /* at least one control point in U and V directions */
    if( n LT 1 OR m LT 1)
        NL_ERROR( NL_INP_ERR );

    /* enough control points to support the given degrees */
    if( n LT p OR m LT q )
        NL_ERROR( NL_INP_ERR );

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Allocate memory for a UVPoint u and v coord for each sample point */
    uu = N_AllocReal1dArray( np, &S );
    vv = N_AllocReal1dArray( np, &S );
    SD = N_AllocPt2dArray( 2, 2, &S );  /* output arg for N_GetClosestPtOnSrf() calls */

    /* when opt origin is missing select axis aligned coordinate system at the origin */
    if(OptProjectionOrig == NULL) 
      {
        planeFlag = NL_NO ;
        Origin.x = Y.x = Z.x = 0.0;
        Origin.y = X.y = Z.y = 0.0;
        Origin.z = X.z = Y.z = 0.0;
        X.x = Y.y = Z.z = 1.0;
      }
    else /* use given projection plane */
      {
        planeFlag = NL_YES ;
        N_CopyPt(*OptProjectionOrig, &Origin) ;
        N_CopyPt(*OptProjectionX,    &X) ;
        N_CopyPt(*OptProjectionY,    &Y) ;
        N_VectorCross(X, Y, &Z) ;
      }

    /* compute UV values  */
    N_FitSrfCalcParams( P, np,               /* Points, highest P index */
                        planeFlag,           /* YES=Plane and normals are input, NO=to be computed  */
                        Origin, X, Y, Z,     /* Opt Plane: Origin, x, y, and z axes      */
                        0.0, 1.0, 0.0, 1.0,  /* U Param [start, end], V Param[start end] */
                        uu, vv );            /* output U and V param values per point, sized:[np+1] */ 

    /* set all Srf internal pointers to NULL */
    N_SrfInitArrays( &srf );

    /* Fit surface to (xyz uv) points   (np >  p+1 * q+1) - no derivatives       */
    /* this surface is used to improve the UV mapping by updating every UV point */
    /*    with the UV value found by projecting the point to this surface.       */
    error = N_FitSrfLstSqDerivs( P, NULL, uu, vv, np,    /* Points, LeastSquaresWeights, UArray, VArray, SizeOfArrays */
                                 NULL, NULL, -1, NULL,   /* Du, LeastSquaresWeights, SizeOfArrays, OptIndexMap */
                                 NULL, NULL, -1, NULL,   /* Dv, LeastSquaresWeights, SizeOfArrays, OptIndexMap */
                                 n, m, p, q,             /* output surface: CPtCntU, CptCntV, DegU, DegV */
                                 NULL, NULL,             /* Optional KnotU, Optional KnotV               */
                                 &srf, &S );             /* output surface, srf's memory stack           */
    if( error )
        NL_OUT;

    /* get tangent direction array, one for each point */
    Tdiru  = N_AllocPt1dArray( np, &S );
    Tdirv  = N_AllocPt1dArray( np, &S );
    PntDer = N_AllocPt2dArray( 1, 1, &S );   /* output arg for N_GetClosestPtOnSrf() */

    /* for every point - improve UV mapping by projecting to srf solution */
    for(i=0, MaxDist=0.0; i <= np; i++ )
      {
        /* Obtain parameter value for this point */
        error = N_GetClosestPtOnSrf( &srf, P[i], uu[i], vv[i], tol0, tol1, &ui, &vi, *PntDer, SD );

        if( error )
            NL_OUT;

        /* improve computed u and v values with projection guesses */
        uu[i] = ui; 
        vv[i] = vi;

        /* evaluate surface, getting point and derivatives in u and v */
        error = N_SrfDerivs( &srf, ui, vi, 
                             NL_LEFT, NL_RIGHT, NL_TRUE, 
                             1, 1, 
                             PntDer ); /* Output: SD[0][0]=Pos, SD[1][0]=Du, SD[0][1]=Dv, etc. , sized:[udr+1][vdr+1] */

        if( error )
            NL_OUT;

        /* save max dist from target point to surface         */
        N_DistPtPt(PntDer[0][0], P[i], &ThisDist) ;
        if(MaxDist < ThisDist) MaxDist = ThisDist ;

        /* Rotate input Surface Normal in direction of TangentU by 90 degrees */
        /*  to turn surface normal into a surface tangentU constraint in upcoming LstSqFit call */
        N_VectorCross( N[i], PntDer[1][0], &M );
        N_VectorCross( M, N[i], &Tdiru[i] );       /* <==== potential problem when Tdiru[i] is small */

        /* Rotate input Surface Normal in direction of TangentV by 90 degrees */
        /*  to turn surface normal into a surface tangentV constraint in upcoming LstSqFit call */
        N_VectorCross( N[i], PntDer[0][1], &M );
        N_VectorCross( M, N[i], &Tdirv[i] );      /* <==== potential problem when Tdiru[i] is small */

      } /*  end iter every point, improving UV guesses and getting Tangent constraints from Surface Normal data */

    /* Fit approx surface to                                                                            */
    /*  given Points P,                                                                                 */
    /*  computed DomainPoints uu and vv,                                                                */
    /*  TangU constraints computed by rotating SurfNorm input 90 degrees in the UTangent direction, and */
    /*  TangV constraints computed by rotatinf SurfNorm input 90 degrees in the VTangent direction.     */

    /* iterate until convergence or 4 times */
    for(i=0,MaxDist=0.0;i<4;i++)
      {
        /* fit point and tangent targets with a surface of specified size */
        /* memory management for sur happens within N_FitSrfLstSqDerivs() */
        error = N_FitSrfLstSqDerivs( P, NULL, uu, vv, np,     /* Points, LeastSquaresWeights, UArray, VArray, SizeOf P, Tdiru, TdirV*/
                                     Tdiru, NULL, -1, NULL,   /* Du, LeastSquaresWeights, SizeOf OptIndexMap, OptIndexMap */       
                                     Tdirv, NULL, -1, NULL,   /* Dv, LeastSquaresWeights, SizeOf OptIndexMap, OptIndexMap */       
                                     n, m, p, q,              /* output surface, CPtCntU, CptCntV, DegU, DegV */             
                                     NULL, NULL,              /* Optional KnotU, Optional KnotV               */             
                                     sur, SG );               /* output surface, srf's memory stack           */ 
        
        if( error )
            NL_OUT;

        /*  all done when asked to ignore tolerances */
        if(tol == 0.0)
          break ;
                                                   
        /* for every point - check distance */
        for(j=0, MaxDist=0.0, BigCount=0; 
            j <= np && (MaxDist < tol || i >= 2); 
            j++)
          {
            /* Obtain parameter value for this point */
            error = N_GetClosestPtOnSrf( sur, P[j], uu[j], vv[j], tol0, tol1, &ui, &vi, *PntDer, SD );

            if( error )
                NL_OUT;

            /* evaluate surface */
            error = N_SrfEvalPt( sur, ui, vi, NL_LEFT, NL_RIGHT, &(PntDer[0][0]) );

            if( error )
                NL_OUT;

            /* save max dist from target point to surface */
            N_DistPtPt(PntDer[0][0], P[j], &ThisDist) ;
            if(MaxDist < ThisDist) MaxDist = ThisDist ;
            if(ThisDist > tol)     BigCount += 1 ; 

          } /* end iter every point getting max distance */

        /* all done when tol is satisfied */
        if(MaxDist <= tol)
          break ;

        /* all done - mostly good */
        if(i >= 2 && BigCount < ((NL_REAL)np)/100.0)
          break ;

        /* all done that's as good as we are going to get */
        if( m > 80 || n > 80) 
          break ;

        /* when MaxDist exceeds tol - try again with bigger surface */
        if(MaxDist > tol)
          {
            /* unless the approximating surface as big as we are willing to go */
            if(   p >= 4 && q >= 4
               && m >= 52 && n >= 52)
              break ;

            /* increase the surface parameters - limit to degree 4 */
            p = (p < 3) ? 3 : ((i%2) == 1) ? 4 : p ;
            q = (q < 3) ? 3 : ((i%2) == 1) ? 4 : q ;

            m = (m * 2 < 52) ? m*2 : 52 ; 
            n = (n * 2 < 52) ? n*2 : 52 ;

          } /* end tolerance check */

      } /* end iter until MaxDist is less than tol */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);

} /* end N_FitRandomPNWithPlaneToTol */


/**********************************************************************/
/* N_FitRandomPNWithPlane: Least squares surface fit to random points and normals */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This fitting routine computes a NURBS surface approximating a given
     list of points  and normals  at  those points.  The degree (p, q)  
     must be  2 or 3.  If the output surface is initialized to the NULL 
     surface, memory is allocated locally. Otherwise  it is checked if 
     enough memory is passed in.  A uv-mapping is made by projecting 
     to the given plane, else it's made by projecting to a best fit plane.
     The algorithm fails if the uv domain of any control point
     has no data points that map to it.

     A typical calling example is:

       NL_POINT       *P;
       NL_VECTOR      *N;
       NL_INDEX       np;
       NL_INDEX       n, m;
       NL_DEGREE      p, q;    
       NL_SURFACE     sur;
       NL_STACKS      SG;
       ...
       (get points, normals, and choose the degree 2 or 3 
        and choose the number of control points);
       ...
       N_SrfInitArrays(&cur);
       N_FitRandomPN(P , N , np, n, m, p, q, &sur, &SG);

   ACCESS:
   
     P    , input  ,  NL_POINT  data as interpolation points
     N    , input  ,  NL_VECTOR as unit normal at interpolation points
     np   , input  ,  Highest index of pts and nrm
     n, m  , input  ,  Highest indexes of the surface control points
                      (the surface will have(n + 1)x(m + 1) control points
     p, q , input  ,  Degree in u, v   (2 or 3)
     sur  , output ,  Interpolating surface
     SG   , input  ,  sur's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
        
NL_FLAG N_FitRandomPNWithPlane
  ( NL_POINT   *P,                   /* Sample Points to fit, sized:[np+1] */
    NL_VECTOR  *N,                   /* Sample Point Normals, sized:[np+1] */
    NL_INDEX    np,                  /* size param */
    NL_POINT   *OptProjectionOrig,   /* optional origin of projection plane, NULL=compute plane, NULL to ignore. */
    NL_VECTOR  *OptProjectionX,      /* optional X axis of projection plane, NULL to ignore. */
    NL_VECTOR  *OptProjectionY,      /* optional Y axis of projection plane, NULL to ignore. */
    NL_INDEX    n,                   /* Output Surface U dir control point count */
    NL_INDEX    m,                   /* Output Surface V dir control point count */
    NL_DEGREE   p,                   /* Output Surface U dir degree */
    NL_DEGREE   q,                   /* Output Surface V dir degree */
    NL_SURFACE *sur,                 /* Output Surface */
    NL_STACKS  *SG )                 /* sur memory stack */
{
  /* NL_PRIVATE NL_STRING rname = _T("N_FitRandomPNWithPlane"); */

  /* pass the call along */
  return( N_FitRandomPNWithPlaneToTol
             (P, N, np, 0.0, 
             OptProjectionOrig, OptProjectionX, OptProjectionY,
             n, m, p, q,
             sur, SG) ) ;

} /* end N_FitRandomPNWithPlane */

/**********************************************************************/
/* N_FitSrfLstSqBoundary: Least squares surface approximation to random points     */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting  routine  computes  a  least squares b-spline surface 
     approximation to a random (non-NxM) set of points. Boundary curves
     may be input  as  constraints.  If boundary curves are input, they
     must be nonrational and  already compatible; i.e. the two u-bound-
     eries must be compatible with one another (in the b-spline sense),
     and the two v-boundaries must  be  compatible  with  one  another.  
     Cross-boundary first derivatives may also be specified. The system
     of equations which is set up may be solved within this routine, or
     by a user-supplied method. If the output surface is initialized to  
     the NULL surface,  memory is allocated locally.  Otherwise  it  is  
     checked  if  enough memory is passed in. A typical calling example 
     is:
 
       NL_POINT      *P;
       NL_CURVE      **bdys, **ders;
       NL_PARAMETER  *uu, *vv;
       NL_REAL       *wts
       NL_INDEX      m, n, np;
       NL_DEGREE     p, q;
       NL_SURFACE    sur;
       NL_STACKS     SG;
       ...
       (define arrays P, wts, uu, vv, U, V, define the boundary curves
        and boundary cross-derivatives, and choose p, q , n and m);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfLstSqBoundary(P,wts,uu,vv,np,bdys,ders,n,m,p,q,U,V,NL_FULL,NL_SVD,
                NL_YES,NULL,&sur,&SG);
 
 
   ACCESS:
   
     P    , input  ,  Points to be approximated
     wts  , input  ,  Least squares point weights (wts[i] >= 0  for  all
                      i).  The  larger  wts[i],  the  closer the surface
                      comes to P[i]. wts[i] < 1 lessens the influence of
                      P[i]. If wts[i] = 0, then P[i] is not used.  If no
                      weighting is desired, set wts = NULL
     uu   , input  ,  The u-parameters of the points in P
     vv   , input  ,  The v-parameters of the points in P
     np   , input  ,  The high index of the arrays: P,wts,uu,vv
     bdys , input  ,  bdys[i], 0<=i<=3, is a boundary curve of the surf-
                      ace ,  ordered:  i=0=NL_BOTTOM , i=1=NL_RIGHT , i=2=NL_TOP,
                      i=3=NL_LEFT. Set bdys[i]=NULL if the i-th boundary is
                      not specified  (but others are);  set bdys=NULL if
                      no boundaries are specified.  If two u-curves  (or
                      v-curves) are given,  they must already be compat-
                      ible in the b-spline sense.  Furthermore,  the u,v
                      values in arrays uu and vv should fall within  the
                      parameter domains of the boundary curves in bdys[]
                      (doesn't matter if the corresponding wts[i] = 0)
     ders , input  ,  ders[i], 0<=i<=3,  is  a cross-boundary derivative
                      curve  ordered:  i=0=NL_BOTTOM , i=1=NL_RIGHT , i=2=NL_TOP,
                      i=3=NL_LEFT.  Set ders[i]=NULL if the i-th derivative 
                      is not specified  (but others are);  set ders=NULL 
                      if no derivatives are specified.  The  derivatives
                      must already be pairwise compatible and compatible
                      with  the  corresponding  bdys[i].  For  example , 
                      bdys[0], bdys[2], ders[0] and ders[2] must all  be
                      compatible in the b-spline sense. Furthermore, the
                      derivatives must match up,  mathematically, at the
                      corners; i.e. ders[0] evaluated at its start  must
                      equal the derivative evaluated from bdys[3]. Twist
                      derivatives must also match. ders[i] may be speci-
                      fied only if bdys[i] is given.  In determining the
                      surface control points, ders[0] and ders[2] depend
                      on the first 2 v-knots,  and ders[1]  and  ders[3] 
                      depend on the first 2 u-knots.  Hence, if any ders
                      are specified, the corresponding surface knots  (U
                      or V) must be passed in (see below)
     n,m  , input  ,  High indexes of the surface  control  points  (the
                      surface will have (n+1)x(m+1) control points).  If
                      boundaries are given, n and m must be at least  as
                      big as the corresponding indexes in the curves
     p,q  , input  ,  Degrees of the surface  (must be greater  than  or
                      equal to the boundary degrees if bdys[] is given)
     U,V  , input  ,  Knots for the surface.  If  U=NULL or V=NULL,  the
                      corresponding knots are  computed in this routine.
                      If given, there must be n+p+2 u-knots and/or m+q+2
                      v-knots.  The values in the uu and vv arrays  must
                      correspond to the knot ranges (unless wts[i] = 0), 
                      and if  boundaries are specified in bdys[],  their
                      knots must be a subset of the knots in U and V.  U
                      (V) must be input if  ders[1] or ders[3]  (ders[0]
                      or ders[2]) are input
     mflg , input  ,  Flag:
                        NL_FULL  : Use a full coefficient matrix  which re-
                                quires  (np+1)*(n+1)*(m+1)  elements  of 
                                type NL_REAL
                        NL_SPARSE: Use a sparse scheme to store the coeffi-
                                cient matrix.  Sparsity  of  the  matrix 
                                grows as n-p and m-q grow.  This  option
                                is slower than NL_FULL, but it is more mem-
                                ory efficient if np*(n-p)*(m-q) is large
                                (see below for more detail)
     aflg , input  ,  Flag:
                        NL_SVD   : Use the Single Value Decomposition  method
                                to solve the over-determined  system  of
                                equations. This method requies that there
                                be as many or more sample points than control points and
                                that the sample points be distributed so that there
                                is at least one sample point in every patch
                                of the output surface.
                        NL_LUPIV : Set up the normal equations and  use  LU 
                                decomposition with partial  pivoting  to
                                solve. This method works for partial sampling,
                                i.e. fewer sample points (or sample points all
                                grouped together) than control points. 
     iflg , input  ,  Flag:
                        NL_YES   : Solve internally in this routine,  using
                                the method prescribed by the flag, aflg
                        NL_NO    : The system of equations will  be  solved 
                                in a caller-supplied function (fsol)
     fsol , input  ,  An optional caller-supplied function to solve  the
                      over-determined system of linear equations set  up
                      in this routine. This function is required only if
                      iflg = NL_NO. See below for a description of fsol
     sur  , output ,  Approximating surface
     SG   , input  ,  sur's memory stack


     Additional details for the flags: mflg, aflg, and iflg:

       A NL_REAL matrix is set up which has 
       size (np+1)*(n+1)*(m+1) if  mflg = NL_FULL, and 
       size (np+1)*(p+q+2)     if  mflg = NL_SPARSE. 
       The mflg = NL_SPARSE option is not allowed if iflg = NL_YES and aflg = 

       NL_SVD (input error).  The NL_SVD method is noticeably slower than
       the NL_LUPIV method,  but  it  is less susceptible to numerical
       problems arising from ill-conditioned systems.  If there are
       plenty of points covering the fit region, and the number  of
       internal u- and v-knots is not too  high,  then  the  system
       should be well-conditioned,  and  NL_LUPIV  can  be  used (with 
       either  NL_FULL or NL_SPARSE option).  By  setting  iflg = NL_NO  and 
       supplying a function, fsol,  the caller can supply  his  own
       equation solver. fsol must have the following structure:
       mflg=NL_FULL   : NL_FLAG fsol(NL_REAL **a, NL_LLONG me, NL_LLONG nu,
                               NL_POINT *rhs, NL_CPOINT *sol)
                     a is the coefficient matrix with me  equations
                     and nu unknowns (me > nu).  rhs  is the  right
                     hand side vector,  and  sol  is  the  solution 
                     vector (allocated in N_FitSrfLstSqBoundary).  fsol computes
                     sol, and returns error = NL_YES or NL_NO.
       mflg=NL_SPARSE : NL_FLAG fsol(NL_LLONG me, NL_LLONG nu, 
                               NL_FLAG (*fetcha)(), NL_POINT *rhs, 
                               NL_CPOINT *sol)
                     fetcha() is a function which fetches a column,
                     row,  or  single  element  of  the  underlying 
                     me x nu matrix (me > nu). fetcha is defined in
                     N_FitSrfLstSqBoundary as:
                     NL_FLAG fetcha(NL_INDEX i, NL_INDEX j, NL_REAL *item)
                     i>=0,j<0 : the i-th row is returned in item
                     i<0,j>=0 : the j-th column is returned in item
                     i>=0,j>=0: the single (i,j)-th element is  re-
                                turned in item
                     It is an error if  i>me, j>nu, or i<0 and j<0.
                     Memory for item must be allocated and  deallo-
                     cated in fsol. rhs is the right hand side vec-
                     tor, and sol is the solution vector (allocated 
                     in N_FitSrfLstSqBoundary).  fsol computes sol,  and returns 
                     error = NL_YES or NL_NO.
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/
NL_FLAG N_FitSrfLstSqBoundary
  (NL_POINT     *P,                                  /* in : sample points, sized:[np+1] */
   NL_REAL      *wts,                                /* in : opt least squares weights, sized:[np+1], */
                                                     /*      NULL to ignore, sized:[np+1] */
   NL_REAL      *uu,                                 /* in : u param for every P point, sized:[np+1] */
   NL_REAL      *vv,                                 /* in : v param for every P point, sized:[np+1] */
   NL_INDEX      np,                                 /* in : size param, size of P, uu, and vv arrays */
   NL_CURVE   ** bdys,                               /* in : opt compatible bdry curves: 0=Bottom, 1=Right, 2=Top, 3=Left,  */
                                                     /*      bdys[i] = NULL to ignore one or more curves, */
                                                     /*      bdys = NULL to ignore all curves */
   NL_CURVE   ** ders,                               /* in : opt associated cross-derivative curves, sized:[4], NULL to ignore.           */
                                                     /*      Curves must be compatible and match bdys[i] derivative values in the corners */
   NL_INDEX      n,                                  /* in : output sur highest ControlPoint index U */
   NL_INDEX      m,                                  /* in : output sur highest ControlPoint index V */
   NL_DEGREE     p,                                  /* in : output sur degree U */
   NL_DEGREE     q,                                  /* in : output sur degree V */
   NL_REAL      *U,                                  /* in : opt sur KnotVector U, NULL=function computes a likely knot vector */
   NL_REAL      *V,                                  /* in : opt sur KnotVector V, NULL=function computes a likely knot vector */
   NL_FLAG       mflg,                               /* in : memory flag: NL_FULL   = more memory, faster, */
                                                     /*                   NL_SPARSE = less memory, slower  */
   NL_FLAG       aflg,                               /* in : solver flag: NL_SVD=SingleValueDecomposition, (requires full or over samplind sampling) */
                                                     /*                   NL_LUPIV=LU Decomposition with partial pivoting, allows sparse sampling */
   NL_FLAG       iflg,                               /* in : solver cntrl: NL_YES=use the aflg internal solver,        */
                                                     /*                    NL_NO=use the user supplied callback solver */
   NL_FLAG (*fsol)                                   /* in : opt solver callback, use this if you own a better solver, used when iflg = NL_NO  */
                                                     /*      for | Full matrices                   | For Sparse matrices                       */
             (NL_LLONG,  /* MatPtr cast to int */    /*          | in : A MatrixPtr of A X = B eqn | m of ASize:[m,n]                          */
              NL_LLONG,                              /*          | in : m of ASize:[m,n]           | n of ASize:[m,n]                          */
              NL_LLONG,  /* FncPtr cast to int */    /*          | in : n of ASize:[m,n]           | ptr to ST_GetRowsColMatrix1(i,j,A) to rtn A[i][j] ptrs */
              NL_POINT *,                            /*          | in : B MatrixPtr of A X = B eqn | B MatrixPtr of A X = B eqn                */
              NL_CPOINT * ),                         /*          | in : X MatrixPtr of A X = B eqn | X MatrixPtr of A X = B eqn                */
   NL_SURFACE   *sur,                                /* out: the approximated surface */
   NL_STACKS    *SG )                                /* in : sur's memory stack */
{
  NL_PRIVATE NL_STRING rname = _T("N_FitSrfLstSqBoundary");

  NL_FLAG error = NL_NO, pflg;

  NL_INDEX n1, n2, n3, n4, ii, jj, kk, nb, mb, ** map, me, nu, uspan = 0, vspan = 0, j1, k1, *perm, ** uvsp = NULL, ** cind = NULL;
  NL_INDEX i0, j0, ii0, jj0, ii1, jj1, i1, USpan, VSpan, iStart, jStart, mm, ll ;
  NL_REAL *pBu, *pBv, dBm ; 
  NL_REAL alpha, beta, PtStiffness ;

  NL_DEGREE pp;

  NL_CURVE *loc_bdys[4], *loc_ders[4];

  NL_REAL *kts1 = NULL, *kts2 = NULL, *kts3 = NULL, *kts4 = NULL, *UB, *VB, *US, *VS, *ufuns, *vfuns, ** a_menu = NULL, ** a_nunu, *svd_w, dd, w, u1, u2, v1, v2, ** a_pq = NULL, u0, v0;
  /* NL_REAL *col1, *col2 ; */ 

  NL_CPOINT *Pw, *Qw, *Dw, ** Sw;

  NL_POINT P1, *rhs, *nrhs;

  NL_KNOTVECTOR knt, *knu, *knv, *knus, *knvs;

  NL_RMATRIX rma;

  NL_STACKS SL;

  /* Start NURBS */

  N_InitNurbs( &SL );

  /* Check for input errors */

  /* at least 2 control points in u and v */
  if( n LT 1 OR m LT 1)
      NL_ERROR( NL_INP_ERR );

  /* when using SVD solver make sure that there are at least as many sample equations as dofs         */
  /*   else switch to LUPIV solver.  GWC:Bug151                                                       */                        
  if( aflg EQ NL_SVD AND (np + 1 LT( n + 1 ) * (m + 1)) )
    { 
      /* switch to the LUPIV solver - it can handle subsampling */
      aflg = NL_LUPIV ;
    }

  /* enough control points for at least one patch or requested degree */
  if( n LT p OR m LT q )
      NL_ERROR( NL_INP_ERR );

  /* Must have boundaries when given optional cross derivative input */
  if( bdys EQ NULL AND ders NEQ NULL )
      NL_ERROR( NL_INP_ERR );

  /* when using internal solver and SingleValueDecomposition, must have a FULL matrix*/
  if( mflg EQ NL_SPARSE AND aflg EQ NL_SVD AND iflg EQ NL_YES )
      NL_ERROR( NL_INP_ERR );

  nb = mb = -1;
  UB = VB = NULL;

  /* initialise just to avoid compiler complaint */
  u1 = v1 = 0.0;
  u2 = v2 = 1.0;

  /* when given optional boundaries - make sure                                          */
  /*  1. all curve degrees are same, elevate degree as needed                            */
  /*  2. surface has more control points in u or v direction than appropriate boundaries */
  /*  3. all matched boundary curves are compatible                                      */
  if( bdys NEQ NULL )
    {
      /* local boundary copies */
      for ( ii = 0; ii <= 3; ii++ )
          loc_bdys[ii] = bdys[ii];

      /* local cross-der boundary copies */
      if( ders NEQ NULL )
          for ( ii = 0; ii <= 3; ii++ )
              loc_ders[ii] = ders[ii];

      /* when given either bot or top bdy curve */
      if( loc_bdys[0]NEQ NULL OR loc_bdys[2]NEQ NULL )
        {
          n1 = n2 = n3 = n4 = -1;

          /* when given bot curve */
          if( loc_bdys[0]NEQ NULL )
            {
              if( N_IsCrvRat( loc_bdys[0] ) )
                  NL_ERROR( NL_INP_ERR );
              N_CrvGetCPtsDegreeAndKnots( loc_bdys[0], &n1, &Pw, &pp, &ii, &kts1 );

              /* check: bot curve degree is higher than requested output surface degree */
              if( pp GT p )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else if( pp LT p ) /* elevate bot curve degree to surface degree */
                {
                  loc_bdys[0] = N_AllocCrv( &SL );
                  N_CrvInitArrays( loc_bdys[0] );
                  error = N_CrvElevateDegree( bdys[0], p - pp, loc_bdys[0], SG, &SL );

                  if( error EQ NL_YES )
                      NL_OUT;
                  N_CrvGetCPtsDegreeAndKnots( loc_bdys[0], &n1, &Pw, &pp, &ii, &kts1 );
                }
            } /* end given bot bndry curve check */

          /* when given top curve */
          if( loc_bdys[2]NEQ NULL )
            {
              if( N_IsCrvRat( loc_bdys[2] ) )
                  NL_ERROR( NL_INP_ERR );
              N_CrvGetCPtsDegreeAndKnots( loc_bdys[2], &n2, &Pw, &pp, &ii, &kts2 );

              /* check: top curve degree is higher than requested output surface degree */
              if( pp GT p )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else if( pp LT p ) /* elevate top curve degree to surface degree */
                {
                  loc_bdys[2] = N_AllocCrv( &SL );
                  N_CrvInitArrays( loc_bdys[2] );
                  error = N_CrvElevateDegree( bdys[2], p - pp, loc_bdys[2], SG, &SL );

                  if( error EQ NL_YES )
                      NL_OUT;
                  N_CrvGetCPtsDegreeAndKnots( loc_bdys[2], &n2, &Pw, &pp, &ii, &kts2 );
                }
            } /* end given top bndry curve check */

          /* when given optional cross-derivative boundary constraints */
          if( ders NEQ NULL )
            {
              /* when given bot cross-der data */
              if( loc_ders[0]NEQ NULL )
                {
                  if( N_IsCrvRat( loc_ders[0] ) )
                      NL_ERROR( NL_INP_ERR );

                  /* must have bot bdry curve */
                  if( loc_bdys[0]EQ NULL )
                      NL_ERROR( NL_INP_ERR );

                  /* must have optional V Knot vector */
                  if( V EQ NULL )
                      NL_ERROR( NL_INP_ERR );
                  N_CrvGetCPtsDegreeAndKnots( loc_ders[0], &n3, &Pw, &pp, &ii, &kts3 );

                  /* check: bot der curve degree is higher than output sur degree */
                  if( pp GT p )
                    {
                      NL_ERROR( NL_INP_ERR );
                    }
                  else if( pp LT p ) /* elevate bot cross-der curve degree to surface degree */
                    {
                      loc_ders[0] = N_AllocCrv( &SL );
                      N_CrvInitArrays( loc_ders[0] );
                      error = N_CrvElevateDegree( ders[0], p - pp, loc_ders[0], SG, &SL );

                      if( error EQ NL_YES )
                          NL_OUT;
                      N_CrvGetCPtsDegreeAndKnots( loc_ders[0], &n3, &Pw, &pp, &ii, &kts3 );
                    }
                } /* end given bot cross-der data check */

              /* when given top cross-der data */
              if( loc_ders[2]NEQ NULL )
                {
                  if( N_IsCrvRat( loc_ders[2] ) )
                      NL_ERROR( NL_INP_ERR );

                  /* must have top bdry curve */
                  if( loc_bdys[2]EQ NULL )
                      NL_ERROR( NL_INP_ERR );

                  /* must have optional V Knot vector */
                  if( V EQ NULL )
                      NL_ERROR( NL_INP_ERR );
                  N_CrvGetCPtsDegreeAndKnots( loc_ders[2], &n4, &Pw, &pp, &ii, &kts4 );

                  /* check: top der curve degree is higher than output sur degree */
                  if( pp GT p )
                    {
                      NL_ERROR( NL_INP_ERR );
                    }
                  else if( pp LT p ) /* elevate top cross-der curve degree to surface degree */
                    {
                      loc_ders[2] = N_AllocCrv( &SL );
                      N_CrvInitArrays( loc_ders[2] );
                      error = N_CrvElevateDegree( ders[2], p - pp, loc_ders[2], SG, &SL );

                      if( error EQ NL_YES )
                          NL_OUT;
                      N_CrvGetCPtsDegreeAndKnots( loc_ders[2], &n4, &Pw, &pp, &ii, &kts4 );
                    }
                } /* end given top cross-der data check */
            } /* end given optional cross-derivative boundary constraints check */

          /* highest CPoint index in top/bot pos, top/bot der, and all top/bot boundary curves */
          ii = NL_MAX( n1, n2 );
          jj = NL_MAX( n3, n4 );
          nb = NL_MAX( ii, jj );

          /* check: more cPts in any top/bot bndry curve than the output surface UDir */
          if( nb GT n )
              NL_ERROR( NL_INP_ERR );
          
          /* when given bot curve - use it's knot vector for output sur UDir knot vector */
          if( n1 GT 0 )
            {
              if( n1 NEQ nb )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else
                  UB = kts1;
            }

          /* when given top curve - use it's knot vector for output sur UDir knot vector  */
          if( n2 GT 0 )
            {
              if( n2 NEQ nb )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else
                  UB = kts2;
            }

          /* when given bot der curve - use it's knot vector for output sur UDir knot vector  */
          if( n3 GT 0 )
            {
              if( n3 NEQ nb )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else
                  UB = kts3;
            }

          /* when given top der curve - use it's knot vector for output sur UDir knot vector  */
          if( n4 GT 0 )
            {
              if( n4 NEQ nb )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else
                  UB = kts4;
            }

          /* kk = output sur UDir knot count */
          kk = nb + 1;

          /* check: all top/bot bdry and der curves have same knots */
          for ( ii = p; ii <= kk; ii++ )
            {
              if( n1 GT 0 )
                  if( kts1[ii]NEQ UB[ii] )
                      NL_ERROR( NL_INP_ERR );

              if( n2 GT 0 )
                  if( kts2[ii]NEQ UB[ii] )
                      NL_ERROR( NL_INP_ERR );

              if( n3 GT 0 )
                  if( kts3[ii]NEQ UB[ii] )
                      NL_ERROR( NL_INP_ERR );

              if( n4 GT 0 )
                  if( kts4[ii]NEQ UB[ii] )
                      NL_ERROR( NL_INP_ERR );
            }

          /* check: bdryCurve/OutputSur UDir knot compatibility */
          if( U NEQ NULL ) 
            {
              /* check: bdry curve and output sur UDir knot intervals the same */
              if( UB[0]NEQ U[0]OR UB[kk]NEQ U[n + 1] )
                  NL_ERROR( NL_INP_ERR );

              ii = jj = p + 1;

              /* check if UB (curve knots) is a subset of U (output sur UDir knots) */
              while( ii LT kk )
                {
                  if( U[jj]GT UB[ii] )
                    {
                      NL_ERROR( NL_INP_ERR );
                    }
                  else if( U[jj]EQ UB[ii] )
                    {
                      ii += 1;
                      jj += 1;
                    }
                  else
                      while( U[jj]LT UB[ii] )
                          jj += 1;
                } /* end while matching top/bot curve knots with output sur UDir knots */
            } /* end bdryCurve/OutputSur UDir knot compatibility check */
        } /* end when given either bot or top bdy curve */

      /* when given either rgt or lft bdy curve */
      if( loc_bdys[1]NEQ NULL OR loc_bdys[3]NEQ NULL )
        {
          n1 = n2 = n3 = n4 = -1;

          /* when given rgt curve */
          if( loc_bdys[1]NEQ NULL )
            {
              if( N_IsCrvRat( loc_bdys[1] ) )
                  NL_ERROR( NL_INP_ERR );
              N_CrvGetCPtsDegreeAndKnots( loc_bdys[1], &n1, &Pw, &pp, &ii, &kts1 );

              /* check: rgt curve degree is higher than requested output surface degree */
              if( pp GT q )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else if( pp LT q ) /* elevate rgt curve degree to surface degree */
                {
                  loc_bdys[1] = N_AllocCrv( &SL );
                  N_CrvInitArrays( loc_bdys[1] );
                  error = N_CrvElevateDegree( bdys[1], q - pp, loc_bdys[1], SG, &SL );

                  if( error EQ NL_YES )
                      NL_OUT;
                  N_CrvGetCPtsDegreeAndKnots( loc_bdys[1], &n1, &Pw, &pp, &ii, &kts1 );
                }
            } /* end given rgt bndry curve check */

          /* when given lft curve */
          if( loc_bdys[3]NEQ NULL )
            {
              if( N_IsCrvRat( loc_bdys[3] ) )
                  NL_ERROR( NL_INP_ERR );
              N_CrvGetCPtsDegreeAndKnots( loc_bdys[3], &n2, &Pw, &pp, &ii, &kts2 );

              /* check: lft curve degree is higher than requested output surface degree */
              if( pp GT q )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else if( pp LT q ) /* elevate lft curve degree to surface degree */
                {
                  loc_bdys[3] = N_AllocCrv( &SL );
                  N_CrvInitArrays( loc_bdys[3] );
                  error = N_CrvElevateDegree( bdys[3], q - pp, loc_bdys[3], SG, &SL );

                  if( error EQ NL_YES )
                      NL_OUT;
                  N_CrvGetCPtsDegreeAndKnots( loc_bdys[3], &n2, &Pw, &pp, &ii, &kts2 );
                }
            } /* end given lft bndry curve check */

          /* when given optional cross-derivative boundary constraints */
          if( ders NEQ NULL )
            {
              /* when given rgt cross-der data */
              if( loc_ders[1]NEQ NULL )
                {
                  if( N_IsCrvRat( loc_ders[1] ) )
                      NL_ERROR( NL_INP_ERR );

                  /* must have rgt bdry curve */
                  if( loc_bdys[1]EQ NULL )
                      NL_ERROR( NL_INP_ERR );

                  /* must have optional U Knot vector */
                  if( U EQ NULL )
                      NL_ERROR( NL_INP_ERR );
                  N_CrvGetCPtsDegreeAndKnots( loc_ders[1], &n3, &Pw, &pp, &ii, &kts3 );

                  /* check: rgt der curve degree is higher than output sur degree */
                  if( pp GT q )
                    {
                      NL_ERROR( NL_INP_ERR );
                    }
                  else if( pp LT q ) /* elevate rgt cross-der curve degree to surface degree */
                    {
                      loc_ders[1] = N_AllocCrv( &SL );
                      N_CrvInitArrays( loc_ders[1] );
                      error = N_CrvElevateDegree( ders[1], q - pp, loc_ders[1], SG, &SL );

                      if( error EQ NL_YES )
                          NL_OUT;
                      N_CrvGetCPtsDegreeAndKnots( loc_ders[1], &n3, &Pw, &pp, &ii, &kts3 );
                    }
                } /* end given rgt cross-der data check */

              /* when given lft cross-der data */
              if( loc_ders[3]NEQ NULL )
                {
                  if( N_IsCrvRat( loc_ders[3] ) )
                      NL_ERROR( NL_INP_ERR );

                  /* must have lft bdry curve */
                  if( loc_bdys[3]EQ NULL )
                      NL_ERROR( NL_INP_ERR );

                  /* must have optional U Knot vector */
                  if( U EQ NULL )
                      NL_ERROR( NL_INP_ERR );
                  N_CrvGetCPtsDegreeAndKnots( loc_ders[3], &n4, &Pw, &pp, &ii, &kts4 );

                  /* check: lft der curve degree is higher than output sur degree */
                  if( pp GT q )
                    {
                      NL_ERROR( NL_INP_ERR );
                    }
                  else if( pp LT q ) /* elevate lft cross-der curve degree to surface degree */
                    {
                      loc_ders[3] = N_AllocCrv( &SL );
                      N_CrvInitArrays( loc_ders[3] );
                      error = N_CrvElevateDegree( ders[3], q - pp, loc_ders[3], SG, &SL );

                      if( error EQ NL_YES )
                          NL_OUT;
                      N_CrvGetCPtsDegreeAndKnots( loc_ders[3], &n4, &Pw, &pp, &ii, &kts4 );
                    }
                } /* end given lft cross-der data check */
            } /* end given optional cross-derivative boundary constraints check */

          /* highest CPoint index in rgt/lft pos, rgt/lft der, and all rgt/lft boundary curves */
          ii = NL_MAX( n1, n2 );
          jj = NL_MAX( n3, n4 );
          mb = NL_MAX( ii, jj );

          /* check: more cPts in any rgt/lft bndry curve than the output surface VDir */
          if( mb GT m )
              NL_ERROR( NL_INP_ERR );

          /* when given rgt curve - use it's knot vector for output sur vDir knot vector */
          if( n1 GT 0 )
            {
              if( n1 NEQ mb )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else
                  VB = kts1;
            }

          /* when given lft curve - use it's knot vector for output sur VDir knot vector  */
          if( n2 GT 0 )
            {
              if( n2 NEQ mb )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else
                  VB = kts2;
            }

          /* when given rgt der curve - use it's knot vector for output sur VDir knot vector  */
          if( n3 GT 0 )
            {
              if( n3 NEQ mb )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else
                  VB = kts3;
            }

          /* when given lft der curve - use it's knot vector for output sur VDir knot vector  */
          if( n4 GT 0 )
            {
              if( n4 NEQ mb )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else
                  VB = kts4;
            }

          /* kk = output sur VDir knot count */
          kk = mb + 1;

          /* check: all rgt/lft bdry and der curves have same knots */
          for ( ii = q; ii <= kk; ii++ )
            {
              if( n1 GT 0 )
                  if( kts1[ii]NEQ VB[ii] )
                      NL_ERROR( NL_INP_ERR );

              if( n2 GT 0 )
                  if( kts2[ii]NEQ VB[ii] )
                      NL_ERROR( NL_INP_ERR );

              if( n3 GT 0 )
                  if( kts3[ii]NEQ VB[ii] )
                      NL_ERROR( NL_INP_ERR );

              if( n4 GT 0 )
                  if( kts4[ii]NEQ VB[ii] )
                      NL_ERROR( NL_INP_ERR );
            }

          /* check: bdryCurve/OutputSur VDir knot compatibility */
          if( V NEQ NULL ) 
            {
              /* check: bdry curve and output sur VDir knot intervals the same */
              if( VB[0]NEQ V[0]OR VB[kk]NEQ V[m + 1] )
                  NL_ERROR( NL_INP_ERR );

              ii = jj = q + 1;

              /* check if VB (curve knots) is a subset of V (output sur VDir knots) */
              while( ii LT kk )
                {
                  if( V[jj]GT VB[ii] )
                    {
                      NL_ERROR( NL_INP_ERR );
                    }
                  else if( V[jj]EQ VB[ii] )
                    {
                      ii += 1;
                      jj += 1;
                    }
                  else
                      while( V[jj]LT VB[ii] )
                          jj += 1;
                } /* end while matching rgt/lft curve knots with output sur VDir knots */
            } /* end bdryCurve/OutputSur VDir knot compatibility check */
        } /* end when given either rgt or lft bdy curve */
    } /* end when given optional boundaries check */

  /* arrive here when optional boundary pos and cross-der curves are */
  /*  1. the same degree as the output sur                           */
  /*  2. have compatible knot vectors amongst themselves             */
  /*  3. have compatible knot vectors with the output sur            */

  /* check and if needed allocate output sur memory */
  error = N_SrfSizeArrays( sur, n, m, p, q, n + p + 1, m + q + 1, rname, SG );
  if( error EQ NL_YES )
      NL_OUT;

  /* sur locals */
  N_SrfGetCPtsAndKnots( sur, &Sw, &US, &VS );
  N_SrfGetKnotVectors( sur, &knu, &knv );

  /* Get the surface u- and v-knots */

  pflg = NL_NO;

  /* when given optional output sur U Knots - use them */
  if( U NEQ NULL )
    {
      kk = n + p + 1;

      /* set output sur U knots to U Knots */
      for ( ii = 0; ii <= kk; ii++ )
          US[ii] = U[ii];
      knus = knu;
    }
  else /* not given option sur U knots - copy from bot/top bdry if possible or set knus = NULL */
    {
      kk = 0;

      /* remember when given an optional bot/top bdry curve */
      if( bdys NEQ NULL )
          if( bdys[0]NEQ NULL OR bdys[2]NEQ NULL )
              kk = 1;

      /* when given an optional bot/top bdry curve */
      if( kk EQ 1 )
        {
          /* set knus = optional bot/top bdry curve knot vector */
          if( bdys[0]NEQ NULL )
              N_CrvGetKnotVector( loc_bdys[0], &knus );
          else
              N_CrvGetKnotVector( loc_bdys[2], &knus );
        }
      else /* not given a bot/top bdry curve */
        {
          /* set knus to NULL for now */
          knus = NULL;

          /* when given optional wts vector */
          if( wts NEQ NULL )
            {
              /* set pflg to use given u min/max values for upcoming N_FitSrfCalcKnotVectors call */
              pflg = NL_YES;
              u1 = 1.0e+20;
              u2 = -u1;

              /* store max/min input point u values in u1/u2 */
              for ( ii = 0; ii <= np; ii++ )
                  if( wts[ii]NEQ 0.0 )
                    {
                      if( u1 GT uu[ii] )
                          u1 = uu[ii];

                      if( u2 LT uu[ii] )
                          u2 = uu[ii];
                    }
            } /* end given optional wts vector check - then get min/max input point u value check */
        } /* end not given a bot/top bdry curve branch */
    } /* end not given option sur U knots - copy from bot/top bdry if possible or set knus = NULL branch */

  /* when given optional output sur V Knots - use them */
  if( V NEQ NULL )
    {
      kk = m + q + 1;

      /* set output sur V knots to V Knots */
      for ( ii = 0; ii <= kk; ii++ )
          VS[ii] = V[ii];
      knvs = knv;
    }
  else /* not given option sur V knots - copy from rgt/lft bdry if possible or set knvs = NULL */
    {
      kk = 0;

      /* remember when given an optional rgt/lft bdry curve */
      if( bdys NEQ NULL )
          if( bdys[1]NEQ NULL OR bdys[3]NEQ NULL )
              kk = 1;

      /* when given an optional rgt/lft bdry curve */
      if( kk EQ 1 )
        {
          /* set knvs = optional rgt/lft bdry curve knot vector */
          if( bdys[1]NEQ NULL )
              N_CrvGetKnotVector( loc_bdys[1], &knvs );
          else
              N_CrvGetKnotVector( loc_bdys[3], &knvs );
        }
      else /* not given a rgt/lft bdry curve */
        {
          /* set knvs to NULL for now */
          knvs = NULL;

          /* when given optional wts vector */
          if( wts NEQ NULL )
            {
              /* set pflg to use given v min/max values for upcoming N_FitSrfCalcKnotVectors call */
              pflg = NL_YES;
              v1 = 1.0e+20;
              v2 = -v1;

              /* store max/min input point v values in v1/v2 */
              for ( ii = 0; ii <= np; ii++ )
                  if( wts[ii]NEQ 0.0 )
                    {
                      if( v1 GT vv[ii] )
                          v1 = vv[ii];

                      if( v2 LT vv[ii] )
                          v2 = vv[ii];
                    }
            } /* end given optional wts vector check - then get min/max input point v value check */
        } /* end not given a rgt/lft bdry curve branch */
    } /* end not given option sur V knots - copy from rgt/lft bdry if possible or set knvs = NULL branch */

  /* arrive here when:                                                                     */
  /*  1. optional bdry curves are compatible with each other and the output sur            */
  /*  2. knus and knvs are set if option U or V vector or optional bdry curves are given   */
  /*  3. knus and knvs are NULL if not set by input and                                    */
  /*       if given optional least squares wts vector,                                     */
  /*       then min/max input point u and v values are saved in u1/u2 and v1/v2            */

  /* when not given optional U and V output sur knot vectors */
  if( knus NEQ knu OR knvs NEQ knv )
    {
      error = N_FitSrfCalcKnotVectors
        ( uu,       /* in : u parameter values, sized:[nn+1] */                                               
          vv,       /* in : v parameter values, sized:[nn+1] */                                               
          np,       /* in : highest u and v index */                                                          
          p,        /* in : approximating surface degree U */                                                 
          q,        /* in : approximating surface degree V */                                                 
          pflg,     /* in : NL_YES = use us,ue,vs,ve values to set max/min u and v knot values */             
                    /*      NL_NO  = find max/min u and v knot values in u and v arrays        */             
          u1,       /* in : min U param value, overridden by any input knu value   */                         
          u2,       /* in : max U param value, overridden by any input knu value   */                         
          v1,       /* in : min V param value, overridden by any input knv value   */                         
          v2,       /* in : max V param value, overridden by any input knv value   */                         
          knus,     /* in : opt starter U knots, if given these knots will be in the output, NULL to ignore */
          knvs,     /* in : opt starter V knots, if given these knots will be in the output, NULL to ignore */
          knu,      /* i/o: sized knot vector whose knot values are to be determined */                       
          knv );    /* i/o: sized knot vector whose knot values are to be determined */                       

      if( error EQ NL_YES )
          NL_OUT;
    } /* end need to Calc Knot Vectors check */

  /* Refine the boundary curves if their knots are not */
  /* the same as the surface knots.                    */

  /* when given optional bdry curves and they have less control points than output sur */
  if( nb GT 0 AND nb LT n )
    {
      /* get memory for knot values to be added to the optional bdry curves */
      kts1 = N_AllocReal1dArray( n - nb - 1, &SL );

      if( kts1 EQ NULL )
          NL_QUIT;

      kk = 0;
      ii = jj = p + 1;

      /* get list of U knot values in outputSur that are not in boundary curves */
      while( jj <= n )
          if( UB[ii]EQ US[jj] )  /* skip knots in both bdry and sur knot vectors */
            {
              ii += 1;
              jj += 1;
            }
          else /* load kts1 with unique sur knot values */
            {
              kts1[kk++] = US[jj++];
            }

      /* make a KnotVector from the kts1 array of unique surface knot values */
      kk -= 1;
      N_KnotVectorFromRealArray( &knt, kts1, kk );

      /* when given bot curve */
      if( loc_bdys[0]NEQ NULL )
        {
          /* insert sur unique knots into bot curve */
          if( loc_bdys[0]NEQ bdys[0] )
              error = N_CrvRefine( loc_bdys[0], &knt, loc_bdys[0], &SL, &SL );
          else
            {
              loc_bdys[0] = N_AllocCrv( &SL );
              N_CrvInitArrays( loc_bdys[0] );
              error = N_CrvRefine( bdys[0], &knt, loc_bdys[0], SG, &SL );
            }

          if( error EQ NL_YES )
              NL_OUT;
        } /* end need to insert knots into bot curve check */

      /* when given top curve */
      if( loc_bdys[2]NEQ NULL )
        {
          /* insert sur unique knots into top curve */
          if( loc_bdys[2]NEQ bdys[2] )
              error = N_CrvRefine( loc_bdys[2], &knt, loc_bdys[2], &SL, &SL );
          else
            {
              loc_bdys[2] = N_AllocCrv( &SL );
              N_CrvInitArrays( loc_bdys[2] );
              error = N_CrvRefine( bdys[2], &knt, loc_bdys[2], SG, &SL );
            }

          if( error EQ NL_YES )
              NL_OUT;
        } /* end need to insert knots into top curve check */

      /* when given optional cros-der input */
      if( ders NEQ NULL )
        {
          /* when given bot der */
          if( loc_ders[0]NEQ NULL )
            {
              /* insert sur unique knots into bot der curve */
              if( loc_ders[0]NEQ ders[0] )
                  error = N_CrvRefine( loc_ders[0], &knt, loc_ders[0], &SL, &SL );
              else
                {
                  loc_ders[0] = N_AllocCrv( &SL );
                  N_CrvInitArrays( loc_ders[0] );
                  error = N_CrvRefine( ders[0], &knt, loc_ders[0], SG, &SL );
                }

              if( error EQ NL_YES )
                  NL_OUT;
            } /* end need to insert knots into bot der curve check */

          /* when given top der */
          if( loc_ders[2]NEQ NULL )
            {
              /* insert sur unique knots into top der curve */
              if( loc_ders[2]NEQ ders[2] )
                  error = N_CrvRefine( loc_ders[2], &knt, loc_ders[2], &SL, &SL );
              else
                {
                  loc_ders[2] = N_AllocCrv( &SL );
                  N_CrvInitArrays( loc_ders[2] );
                  error = N_CrvRefine( ders[2], &knt, loc_ders[2], SG, &SL );
                }

              if( error EQ NL_YES )
                  NL_OUT;
            } /* end need to insert knots into top der curve check */
        } /* end given optional cros-der input check */
    } /* end given optional bdry curves and they have less control points than output sur check */

  /* when given optional bdry curves and they have less control points than output sur */
  if( mb GT 0 AND mb LT m )
    {
      /* get memory for knot values to be added to the optional bdry curves */
      kts1 = N_AllocReal1dArray( m - mb - 1, &SL );

      if( kts1 EQ NULL )
          NL_QUIT;

      kk = 0;
      ii = jj = q + 1;

      /* get list of V knot values in outputSur that are not in boundary curves */
      while( jj <= m )
          if( VB[ii]EQ VS[jj] ) /* skip knots in both bdry and sur knot vectors */
            {
              ii += 1;
              jj += 1;
            }
          else /* load kts1 with unique sur knot values */
            {
              kts1[kk++] = VS[jj++];
            }

      /* make a KnotVector from the kts1 array of unique surface knot values */
      kk -= 1;
      N_KnotVectorFromRealArray( &knt, kts1, kk );

      /* when given rgt curve */
      if( loc_bdys[1]NEQ NULL )
        {
          /* insert sur unique knots into rgt curve */
          if( loc_bdys[1]NEQ bdys[1] )
              error = N_CrvRefine( loc_bdys[1], &knt, loc_bdys[1], &SL, &SL );
          else
            {
              loc_bdys[1] = N_AllocCrv( &SL );
              N_CrvInitArrays( loc_bdys[1] );
              error = N_CrvRefine( bdys[1], &knt, loc_bdys[1], SG, &SL );
            }

          if( error EQ NL_YES )
              NL_OUT;
        } /* end need to insert knots into rgt curve check */

      /* when given lft curve */
      if( loc_bdys[3]NEQ NULL )
        {
          /* insert sur unique knots into lft curve */
          if( loc_bdys[3]NEQ bdys[3] )
              error = N_CrvRefine( loc_bdys[3], &knt, loc_bdys[3], &SL, &SL );
          else
            {
              loc_bdys[3] = N_AllocCrv( &SL );
              N_CrvInitArrays( loc_bdys[3] );
              error = N_CrvRefine( bdys[3], &knt, loc_bdys[3], SG, &SL );
            }

          if( error EQ NL_YES )
              NL_OUT;
        } /* end need to insert knots into lft curve check */

      /* when given optional cros-der input */
      if( ders NEQ NULL )
        {
          /* when given rgt der */
          if( loc_ders[1]NEQ NULL )
            {
              /* insert sur unique knots into rgt der curve */
              if( loc_ders[1]NEQ ders[1] )
                  error = N_CrvRefine( loc_ders[1], &knt, loc_ders[1], &SL, &SL );
              else
                {
                  loc_ders[1] = N_AllocCrv( &SL );
                  N_CrvInitArrays( loc_ders[1] );
                  error = N_CrvRefine( ders[1], &knt, loc_ders[1], SG, &SL );
                }

              if( error EQ NL_YES )
                  NL_OUT;
            } /* end need to insert knots into rgt der curve check */

          /* when given lft der */
          if( loc_ders[3]NEQ NULL )
            {
              /* insert sur unique knots into lft der curve */
              if( loc_ders[3]NEQ ders[3] )
                  error = N_CrvRefine( loc_ders[3], &knt, loc_ders[3], &SL, &SL );
              else
                {
                  loc_ders[3] = N_AllocCrv( &SL );
                  N_CrvInitArrays( loc_ders[3] );
                  error = N_CrvRefine( ders[3], &knt, loc_ders[3], SG, &SL );
                }

              if( error EQ NL_YES )
                  NL_OUT;
            } /* end need to insert knots into lft der curve check */
        } /* end given optional cros-der input check */
    } /* end given optional bdry curves and they have less control points than output sur check */

  /* arrive here when                                                                 */
  /*  1. output sur has been assigned knot vectors                                    */
  /*  2. Sur knot vectors are compatible with all given optional bdry and der curves  */

  /* next set up maps; mapping between sur controlPoints(ii,jj) to least squares dofs[kk]  */
  /* nu         = highest index of least squares dof (dof count = nu + 1)                          */
  /* map[ii,jj] = LeastSquares dof index kk for sur ControlPoint[ii,jj], -1 = no dof (constrained) */
  /*              for all ii, jj where 0 <= ii <= n, 0<= jj <= m                                   */
  /* cind[kk,0] = associated control point ii value                                                */
  /* cind[kk.1] = associated control point jj value                                                */

  /* Allocate memory for the ControlPoint->DofIndex map */
  map = N_AllocInt2dArray( n, m, &SL );

  if( map EQ NULL )
      NL_QUIT;

  /* when needed */
  if( mflg EQ NL_SPARSE )
    {
      /* Allocate memory for the DofIndex->ControlPoint map */
      cind = N_AllocInt2dArray( (n + 1) * (m + 1) - 1, 1, &SL );

      if( cind EQ NULL )
          NL_QUIT;
    }

  /* Define map : ControlPoint(i,j)->DofIndex[k]  and  */
  /*        cind: DofIndex[k,0]->ControlPoint(i)       */
  /*              DofIndex[k,1]->ControlPoint(j)       */

  nu = -1; /* least squares highest dof index, number of dofs = nu+1         */
           /* nu = TotalControlPointCount - ConstrainedControlPointCount - 1 */

  /* for every outputSur V control point - assign a least squares dof index value */
  for ( jj = 0; jj <= m; jj++ )
    {
      /* clever iter limits to help set up map and cind indices */
      n1 = 0;
      n2 = n;

      /* when given optinoal bdry curves - adjust n1 and n2 values */
      if( bdys NEQ NULL )
        {
          /* kk flag: kk = 1 for working on a given boundary curve */
          /*          kk = 0 otherwise */
          kk = 0;

          /* remember when targeting a given bot curve */
          if( jj EQ 0 AND bdys[0]NEQ NULL )
              kk = 1;

          /* remember when targeting a given top curve */
          else if( jj EQ m AND bdys[2]NEQ NULL )
              kk = 1;

          /* when given der curves */
          if( kk EQ 0 AND ders NEQ NULL )
            {
              /* remember when targeting a given bot der curve */
              if( jj EQ 1 AND ders[0]NEQ NULL )
                  kk = 1;

              /* remember when targeting a given top der curve */
              else if( jj EQ m - 1 AND ders[2]NEQ NULL )
                  kk = 1;
            }

          /* when working on a targeted given bdry curve - set n1/n2 iterator bounds */
          if( kk EQ 1 )
            {
              n1 = n + 1;
              n2 = n;
            }
          else /* not working on a given bdry curve - modify n1/n2 iterator bounds to mark constrained dofs */
            {
              if( bdys[1]NEQ NULL )
                  n2 = n - 1;

              if( bdys[3]NEQ NULL )
                  n1 = 1;

              if( ders NEQ NULL )
                {
                  if( ders[1]NEQ NULL )
                      n2 = n - 2;

                  if( ders[3]NEQ NULL )
                      n1 = 2;
                }
            }
        } /* end need to adjust n1/n2 values due to given optional bdry curves check */

      /* mark constrained control points */
      for ( ii = 0; ii < n1; ii++ )
          map[ii][jj] = -1;

      /* assign dof indices to unconstrained control points */
      for ( ii = n1; ii <= n2; ii++ )
        {
          nu += 1;
          map[ii][jj] = nu;

          if( mflg EQ NL_SPARSE )
            {
              cind[nu][0] = ii;
              cind[nu][1] = jj;
            }
        }

      /* mark constrained control points */
      for ( ii = n2 + 1; ii <= n; ii++ )
          map[ii][jj] = -1;
  
  } /* end iter every outputSur V Control Point */

  /* arrive here when map and cind are built as */
  /* map : ControlPoint(i,j)->DofIndex[k], (map[i][j] == -1 for constrained ControlPoint[i][j])  and  */
  /* cind: DofIndex[k,0]->ControlPoint(i)                                                             */
  /*       DofIndex[k,1]->ControlPoint(j)                                                             */

  /* Load the surface's constrained control points directly into Sw */
  /* Sw = outpuSur control Point array                              */
  if( bdys NEQ NULL )
    {
      /* when given bot curve */
      if( loc_bdys[0]NEQ NULL )
        {
          N_CrvGetCPts( loc_bdys[0], &kk, &Pw ); /* kk is equal to n */

          /* load bot control points into Sw array */
          for ( ii = 0; ii <= kk; ii++ )
              N_CopyCPt( Pw[ii], &Sw[ii][0] );

          /* when given bot der */
          if( ders NEQ NULL AND loc_ders[0]NEQ NULL )
            {
              N_CrvGetCPts( loc_ders[0], &kk, &Dw );
              dd = (VS[q + 1] - VS[q]) / q;

              /* load bot der 2nd row control points into Sw array */
              for ( ii = 0; ii <= kk; ii++ )
                {
                  N_CopyCPt( Pw[ii], &Sw[ii][1] );
                  N_VectorBlendCPt( dd, Dw[ii], &Sw[ii][1] );
                }
            } /* end given bot der check */
        } /* end given bot curve check */

      /* when given top curve */
      if( loc_bdys[2]NEQ NULL )
        {
          N_CrvGetCPts( loc_bdys[2], &kk, &Pw ); /* kk is equal to n */

          /* load top control points into Sw array */
          for ( ii = 0; ii <= kk; ii++ )
              N_CopyCPt( Pw[ii], &Sw[ii][m] );

          /* when given top der */
          if( ders NEQ NULL AND loc_ders[2]NEQ NULL )
            {
              N_CrvGetCPts( loc_ders[2], &kk, &Dw );
              dd = -(VS[m + 1] - VS[m]) / q;

              /* load top der 2nd row control points into Sw array */
              for ( ii = 0; ii <= kk; ii++ )
                {
                  N_CopyCPt( Pw[ii], &Sw[ii][m - 1] );
                  N_VectorBlendCPt( dd, Dw[ii], &Sw[ii][m - 1] );
                }
            } /* end given top der check */
        } /* end given top curve check */

      /* when given lft curve */
      if( loc_bdys[3]NEQ NULL )
        {
          N_CrvGetCPts( loc_bdys[3], &kk, &Pw ); /* kk is equal to m */

          /* load lft control points into Sw array */
          for ( ii = 0; ii <= kk; ii++ )
              N_CopyCPt( Pw[ii], &Sw[0][ii] );

          /* when given lft der */
          if( ders NEQ NULL AND loc_ders[3]NEQ NULL )
            {
              N_CrvGetCPts( loc_ders[3], &kk, &Dw );
              dd = (US[p + 1] - US[p]) / p;

              /* load lft der 2nd row control points into Sw array */
              for ( ii = 0; ii <= kk; ii++ )
                {
                  N_CopyCPt( Pw[ii], &Sw[1][ii] );
                  N_VectorBlendCPt( dd, Dw[ii], &Sw[1][ii] );
                }
            } /* end given lft der check */
        } /* end given lft curve check */

      /* when given rgt curve */
      if( loc_bdys[1]NEQ NULL )
        {
          N_CrvGetCPts( loc_bdys[1], &kk, &Pw ); /* kk is equal to m */

          /* load rgt control points into Sw array */
          for ( ii = 0; ii <= kk; ii++ )
              N_CopyCPt( Pw[ii], &Sw[n][ii] );

          /* when given rgt der */
          if( ders NEQ NULL AND loc_ders[1]NEQ NULL )
            {
              N_CrvGetCPts( loc_ders[1], &kk, &Dw );
              dd = -(US[n + 1] - US[n]) / p;

              /* load rgt der 2nd row control points into Sw array */
              for ( ii = 0; ii <= kk; ii++ )
                {
                  N_CopyCPt( Pw[ii], &Sw[n - 1][ii] );
                  N_VectorBlendCPt( dd, Dw[ii], &Sw[n - 1][ii] );
                }
            } /* end given rgt der check */
        } /* end given rgt curve check */
    } /* end given bdry curves check */

  /* arrive here when map and cind are built as */
  /* map : ControlPoint(i,j)->DofIndex[k], (map[i][j] == -1 for constrained ControlPoint[i][j])  and  */
  /* cind: DofIndex[k,0]->ControlPoint(i)                                                             */
  /*       DofIndex[k,1]->ControlPoint(j)                                                             */
  /* Sw  : (outputSur ControlPoint array) set with all constrained bdry curve values                  */

  /* next: */

  /* Allocate rhs and Qw array memory for matrix equation [A] [Qw] =  [rhs] */
  /*  sized nu = totalControlPointCount - ConstrainedControlPointCount      */
  Qw = N_AllocCPt1dArray( nu, &SL );
  if( Qw EQ NULL )
      NL_QUIT;

  rhs = N_AllocPt1dArray( np, &SL );
  if( rhs EQ NULL )
      NL_QUIT;

  /* Now set up the overdetermined system of equations */

  /* outputSur domain intervals */
  u1 = US[0];
  u2 = US[n + 1];
  v1 = VS[0];
  v2 = VS[m + 1];

  w = 1.0;
  u0 = u1 - 1.0;
  v0 = v1 - 1.0;

  me = -1; /* number of points used for setting up the least squares problem       */
           /* points are culled when outsied the outputSur domain or when optional */
           /* least squares weight is set to a negative value                      */

  /* when asked for Full matrices */ 
  if( mflg EQ NL_FULL )
    { 
      /* use full storage scheme for matrix eqn [a_menu] * [Sw] = [rhs]                                     */
      /*  a_menu = a_menu [InputPointCount x DofCount]                                                      */
      /*     each row of a_menu is a weighted expression of                                                 */
      /*     "let the surface shape equal the sample point position" as                                     */
      /*       sqrt(w) * (sur(ui,vi) = Pi)                                                                  */
      /*          where: sur(ui,vi) = Sum_i(Sum_j( BU[i]*BV[j] * Q[i,j]))                                   */
      /*                 rhs[i]     = sqrt(weight)*Pi                                                       */
      /*                 Sw[i,j]    = outputSur control points                                              */
      /*                 BU[i]      = nonZero U basis function values for UVPoint(ui,vi)                    */       
      /*                 BV[j]      = nonZero V basis function values for UVPoint(ui,vi)                    */
      /*  with one twiddle for enforcing constrained dof values:                                            */
      /*    All column entries of a_menu associated with a constrained dof are                              */
      /*    multipled by the constrained dof value and subtracted from the rhs vector.                      */
      /*  So  [a_menu] * Sw = [rhs] becomes                                                                 */
      /*      [a_menu'] * Qw = [rhs] - [a_menu] * Sw                                                        */
      /*       a_menu' = a_menu with all columns associated with constrained dofs set to 0.0                */
      /*       QsConstrained = Sw but with all nonConstrained dofs set to 0.0                               */
      /*                                                                                                    */
      /*  ufuns  = ufuns  [UDeg+1] stores nonVanishing OutputSur U basis function values for given u value  */
      /*  vfuns  = vfuns  [VDeg+1] stores nonVanishing OutputSur V basis function values for given v value  */

      a_menu = N_AllocReal2dArray( np, nu, &SL );
      if( a_menu EQ NULL )
          NL_QUIT;

      ufuns = N_AllocReal1dArray( p + q + 1, &SL );
      if( ufuns EQ NULL )
          NL_QUIT;

      vfuns = &ufuns[p + 1];

      /* init a_menu[i,j] = 0.0 */
      for ( ii = 0; ii <= np; ii++ )
          for ( jj = 0; jj <= nu; jj++ )
              a_menu[ii][jj] = 0.0;

      /* set up system */

      /* for every SamplePoint - build the OutputSur(ui,vi) = Pi eqns, skipping points outside OutputSur domain */
      for ( ii = 0; ii <= np; ii++ )
        {
          /* skip points outside the sur U interval */
          if( uu[ii]LT u1 OR uu[ii]GT u2 )
              continue;

          /* skip points outside the sur V interval */
          if( vv[ii]LT v1 OR vv[ii]GT v2 )
              continue;

          /* when given point weights */
          if( wts NEQ NULL )
            {
              /* skip nonPositive weighted points */
              if( wts[ii]LE 0.0 )
                  continue;
              else /* set w with this point's sqrt(weight value) */
                {
                  if( wts[ii]EQ 1.0 )
                      w = 1.0;
                  else
                      w = sqrt( wts[ii] );
                }
            } /* end given point weights check */

          /* count the number of sample points used */
          me += 1;

          /* when uu[ii] is different than last uu value */
          if( uu[ii]NEQ u0 )
            {
              u0 = uu[ii];

              /* get nonVanishing U basis values */
              error = N_BasisEval( knu, p, uu[ii], NL_LEFT, ufuns, &uspan );

              if( error EQ NL_YES )
                  NL_OUT;
            } /* end uu[ii] is new check */

          /* when vv[ii] is different than last vv value */
          if( vv[ii]NEQ v0 )
            {
              v0 = vv[ii];
              
              /* get nonVanishing V basis values */
              error = N_BasisEval( knv, q, vv[ii], NL_LEFT, vfuns, &vspan );

              if( error EQ NL_YES )
                  NL_OUT;
            } /* end vv[ii] is new check */

          /* load rhs with SamplePoint value */
          N_CopyPt( P[ii], &rhs[me] );

          /* when sample point weight is nonUnit */
          if( w NEQ 1.0 )
            {
              u0 = u1 - 1.0; /* remember that ufuns changed */

              /* apply nonUnit weight to ufuns and PointValue */
              for ( jj = 0; jj <= p; jj++ )
                  ufuns[jj] *= w;
              N_ScalePt( w, rhs[me], &rhs[me] );
            }

          /* for every nonZero u basis function for this sample point */
          for ( jj = 0; jj <= p; jj++ )
            {
              /* U effected Control point index */
              j1 = uspan - p + jj;

              /* for every nonZero v basis function for this sample point */
              for ( kk = 0; kk <= q; kk++ )
                {
                  /* V effected control point index */
                  k1 = vspan - q + kk;

                  /* when effected control point is not constrained */
                  if( map[j1][k1]GE 0 )
                    {
                      /* set A term due to this sample point */
                      a_menu[me][map[j1][k1]] = ufuns[jj] * vfuns[kk];
                    }
                  else /* add this sample point's effect to the rhs vector */ 
                    {
                      N_CPtToPtEuclid( Sw[j1][k1], &P1 );
                      N_VectorBlendPt( -(ufuns[jj] * vfuns[kk]), P1, &rhs[me] );
                    }
                } /* end iter kk, ever nonZero v basis function for this Sample point */
            } /* end iter jj, ever nonZero u basis function for this Sample point  */
        } /* end iter ii, every Sample Point */
    } /* end asked for full matrix branch */
  else /* asked for sparse matrix */
    { 
      /* use sparse storage scheme */
      a_pq = N_AllocReal2dArray( np, p + q + 1, &SL );

      if( a_pq EQ NULL )
          NL_QUIT;

      /* sur span index for every sample point */
      uvsp = N_AllocInt2dArray( np, 1, &SL );

      if( uvsp EQ NULL )
          NL_QUIT;

      glo_a    = a_pq;
      glo_cind = cind;
      glo_uvsp = uvsp;

      /* set up system */

      /* for every sample Point */
      for ( ii = 0; ii <= np; ii++ )
        {
          /* skip points outside the sur U interval */
          if( uu[ii]LT u1 OR uu[ii]GT u2 )
              continue;

          /* skip points outside the sur V interval */
          if( vv[ii]LT v1 OR vv[ii]GT v2 )
              continue;

          /* when given point weights */
          if( wts NEQ NULL )
            {
              /* skip nonPositive weighted points */
              if( wts[ii]LE 0.0 )
                  continue;
              else /* set w with this point's sqrt(weight value) */
                {
                  if( wts[ii]EQ 1.0 )
                      w = 1.0;
                  else
                      w = sqrt( wts[ii] );
                }
            } /* end given point weights check */

          /* count the number of sample points used */
          me += 1;

          /* when uu[ii] is different than last uu value */
          if( uu[ii]NEQ u0 )
            {
              u0 = uu[ii];
              
              /* get nonVanishing U basis values */
              error = N_BasisEval( knu, p, uu[ii], NL_LEFT, &a_pq[me][0], &uspan );

              if( error EQ NL_YES )
                  NL_OUT;
            } /* end uu[ii] is new check */
          else 
            {
              /* just copy last nonVnaishing U basis values */
              for ( jj = 0; jj <= p; jj++ )
                  a_pq[me][jj] = a_pq[me - 1][jj];
            }

          /* when vv[ii] is different than last vv value */
          if( vv[ii]NEQ v0 )
            {
              v0 = vv[ii];
              
              /* get nonVanishing V basis values */
              error = N_BasisEval( knv, q, vv[ii], NL_LEFT, &a_pq[me][p + 1], &vspan );

              if( error EQ NL_YES )
                  NL_OUT;
            } /* end vv[ii] is new check */
          else
            {
              /* just copy last nonVnaishing U basis values */
              for ( jj = 0; jj <= q; jj++ )
                  a_pq[me][jj + p + 1] = a_pq[me - 1][jj + p + 1];
            }

          /* save Sample Point span indices */
          uvsp[me][0] = uspan;
          uvsp[me][1] = vspan;

          /* copy Point value into rhs */
          N_CopyPt( P[ii], &rhs[me] );

          /* when given nonUnit weight */
          if( w NEQ 1.0 )
            {
              
              u0 = u1 - 1.0; /* remember that a_pq[me][0] changed */

              /* apply the weight */
              for ( jj = 0; jj <= p; jj++ )
                  a_pq[me][jj] *= w;
              N_ScalePt( w, rhs[me], &rhs[me] );
            }

          /* for every nonZero u basis function for this sample point */
          for ( jj = 0; jj <= p; jj++ )
            {
              /* U effected Control point index */
              j1 = uspan - p + jj;

              /* for every nonZero v basis function for this sample point */
              for ( kk = 0; kk <= q; kk++ )
                {
                  /* V effected control point index */
                  k1 = vspan - q + kk;

                  /* when effected controlPoint is constrained */
                  if( map[j1][k1]LT 0 )
                    {
                      /* subtract its effect from the rhs vector */
                      N_CPtToPtEuclid( Sw[j1][k1], &P1 );
                      N_VectorBlendPt( -(a_pq[me][jj] * a_pq[me][p + kk + 1]), P1, &rhs[me] );
                    }
                } /* end iter kk, ever nonZero v basis function for this Sample point */
            } /* end iter jj, ever nonZero u basis function for this Sample point  */
        } /* end iter every ii, every sample Point */
    } /* end asked for sparse matrix branch */

  /* when using SVD solver make sure that there are at least as many sample equations as dofs         */
  /*   else switch to LUPIV solver.  GWC:Bug151                                                       */                        
  if( aflg EQ NL_SVD AND me LE nu )
    { 
      /* switch to the LUPIV solver - it can handle subsampling */
      aflg = NL_LUPIV ; 
    }

  glo_me = me;
  glo_nu = nu;
  glo_p = p;
  glo_q = q;

  /* Now solve the system, either internally or externally */

  /* next build least squares equation set */
  /* for solver = NL_LUPIV                 */
  /*    a_nunu * Qw = rhs'                 */
  /*    a_nunu = a_menu_transpose * a_menu */
  /*    rhs'   = a_menu * rhs              */
  /* for solver = NL_SVD                   */
  /*    apply SVD directly to eqn          */
  /*    a_menu * Sw = rhs                  */

  /* when asked - solve internally */
  if( iflg EQ NL_YES )
    { 
      /* allocate A matrix */
      a_nunu = N_AllocReal2dArray( nu, nu, &SL );
      if( a_nunu EQ NULL )
          NL_QUIT;

      /* when asked solve with Singular Value Decomposition */ 
      if( aflg EQ NL_SVD )
        { 
          
          svd_w = N_AllocReal1dArray( nu, &SL );
          if( svd_w EQ NULL )
              NL_QUIT;

          /* decompose A into = U * w * T(V) where T(V) = transpose(V) */
          /*   changes a_menu into U */
          error = N_RealMatrixSingleValueDecompose( a_menu, svd_w, a_nunu, me, nu );
          if( error EQ NL_YES )
              NL_OUT;

          /* back substitute the solution */
          error = N_SingleValueDecomposeSolve( a_menu, svd_w, a_nunu, me, nu, (NL_VOID *)rhs, NL_EPOINT, NL_YES, (NL_VOID *)Qw );
          if( error EQ NL_YES )
              NL_OUT;
        }

      /* when asked solve with LU Decomposition */ 
      if( aflg EQ NL_LUPIV )
        { /* Solve via Normal Equations and LU Decomposition */
          /* Set up Normal Equations and rhs */

          /* add stiffness terms to add to A matrix */
          alpha       = 1.0 ;  /* stretch resistance */
          beta        = 10.0 ; /* bending resistance */
          PtStiffness = 1.0E10 ; 

          /* */
          nrhs = N_AllocPt1dArray( nu, &SL );
          if( nrhs EQ NULL )
              NL_QUIT;

          /* for FULL memory scheme */
          if( mflg EQ NL_FULL )
            {
              /* for every dof */
              for ( ii = 0; ii <= nu; ii++ )
                {
                  N_CopyPt( NL_ZERO, &nrhs[ii] );

                  /* nrhs = a_menu_transpose * rhs */
                  for ( jj = 0; jj <= me; jj++ )
                      N_VectorBlendPt( PtStiffness * a_menu[jj][ii], rhs[jj], &nrhs[ii] );

                  /* build A_nunu = a_menu_transpose * a_menu (upper triangle) */
                  for ( jj = ii; jj <= nu; jj++ )
                    {
                      dd = 0.0;

                      for ( kk = 0; kk <= me; kk++ )
                          dd += PtStiffness * a_menu[kk][ii] * a_menu[kk][jj];
                      a_nunu[ii][jj] = dd;
                    }

                  /* build A_nunu (lower triangle */
                  for ( jj = ii + 1; jj <= nu; jj++ )
                      a_nunu[jj][ii] = a_nunu[ii][jj];
                } /* end iter ii, every dof */
            } /* end full memory scheme branch */
          else /* use sparse memory scheme */
            {

             /* build A and B matrices */

             /* init A and B to zero - these clears are tuned to memory layout done */
             /* in  N_AllocPt1dArray() and N_AllocReal2dArray()                     */
             N_MemSet(nrhs,0,(nu + 1) * sizeof( NL_POINT )) ;
             N_MemSet(a_nunu[0], 0, (nu + 1) * (nu + 1) * sizeof( NL_REAL )  );

             /* for every data point */
             for(kk=0;kk<=me;kk++)
               {
                 /* u and v basis values for kkth data point */
                 pBu = a_pq[kk] ;
                 pBv = pBu + p + 1 ;

                 /* u and v spans for the kkth data point    */
                 USpan = uvsp[kk][0] ;
                 VSpan = uvsp[kk][1] ;

                 /* smallest global dof number for USpan/VSpan element */
                 iStart = USpan - p;
                 jStart = VSpan - q;

                 /* for every V Basis function */
                 for(j0=0;j0<=q;j0++)
                   {
                     /* map i0,j0 to global mm index */
                     jj0 = jStart + j0;

                     /* for every U Basis function */
                     for(i0=0;i0<=p;i0++)
                       {
                         ii0 = iStart + i0;
                         mm = map[ii0][jj0];
                         if ( mm == -1 )
                         { continue; }

                         /* Basis function B_m */
                         dBm = PtStiffness * pBu[i0] * pBv[j0] ; /* use this line when using alpha, beta, and PtStiffness */
                         /* dBm = pBu[i0] * pBv[j0] ;  */

                         /* add terms to B matrix B[mm] += rhs[kk] * B_m(u_k,v_k) */
                         N_VectorBlendPt(dBm , rhs[kk], &nrhs[mm] );

                         /* add B_m(uk,vk)*B_l(uk,vk) terms to A[m,l] matrix elements */
                         for(j1=0;j1<=q;j1++)
                           {
                             /* map i1,j1 to global ll index */
                             jj1 = jStart + j1;

                             for(i1=0;i1<=p;i1++)
                               {
                                 /* add terms to A matrix */
                                 ii1 = iStart + i1;
                                 ll = map[ii1][jj1];

                                 if ( ll == -1 )
                                 { continue; }

                                 a_nunu[mm][ll] +=  dBm * pBu[i1] * pBv[j1] ;
                      
                               } /* end iter every U Basis function */
                           } /* end iter every V Basis function - making A terms */

                       } /* end iter every U Basis function */
                   } /* end iter every V Basis function - making B and A terms */
               } /* end iter every pos, u-deriv and v-deriv input point */

              /*      allocate memory for columns of the coeff matrix                  */
              /*                                                                       */
              /*      col1 = N_AllocReal1dArray( me, &SL );                            */
              /*                                                                       */
              /*      if( col1 EQ NULL )                                               */
              /*          NL_QUIT;                                                     */
              /*      col2 = N_AllocReal1dArray( me, &SL );                            */
              /*                                                                       */
              /*      if( col2 EQ NULL )                                               */
              /*          NL_QUIT;                                                     */
              /*                                                                       */
              /*      for ( ii = 0; ii <= nu; ii++ )                                   */
              /*      {                                                                */
              /*          error = ST_GetRowsColMatrix1( -1, ii, col1 );                */
              /*                                                                       */
              /*          if( error EQ NL_YES )                                        */
              /*              NL_OUT;                                                  */
              /*                                                                       */
              /*          N_CopyPt( NL_ZERO, &nrhs[ii] );                              */
              /*                                                                       */
              /*          for ( jj = 0; jj <= me; jj++ )                               */
              /*              N_VectorBlendPt( col1[jj], rhs[jj], &nrhs[ii] );         */
              /*                                                                       */
              /*          for ( jj = ii; jj <= nu; jj++ )                              */
              /*          {                                                            */
              /*              error = ST_GetRowsColMatrix1( -1, jj, col2 );            */
              /*                                                                       */
              /*              if( error EQ NL_YES )                                    */
              /*                  NL_OUT;                                              */
              /*                                                                       */
              /*              dd = 0.0;                                                */
              /*                                                                       */
              /*              for ( kk = 0; kk <= me; kk++ )                           */
              /*                  dd += col1[kk] * col2[kk];                           */
              /*              a_nunu[ii][jj] = dd;                                     */
              /*          }                                                            */
              /*                                                                       */
              /*          for ( jj = ii + 1; jj <= nu; jj++ )                          */
              /*              a_nunu[jj][ii] = a_nunu[ii][jj];                         */
              /*      }                                                                */
            } /* end sparse memory scheme branch */

          /* stablize computation when sample points don't cover whole domain    */
          /* add stiffness terms to A matrix                                     */
          /*      alpha       = 1.0 ;                                            */
          /*      beta        = 10.0 ;                                           */
          /*      PtStiffness = 1.0E10 ; (this term weights the constraint eqns) */

          /* Mapping for A_nunu_index->Global_Dof_index                                      */
          /* 1. map : ControlPoint(i,j)->DofIndex[k]  and                                    */
          /*    cind: DofIndex[k,0]->ControlPoint(i)                                         */
          /*          DofIndex[k,1]->ControlPoint(j)                                         */
          /* 2. Sw (outputSur ControlPoint array) set with all constrained bdry curve values */

          /* bug 151 - a_nunu is set up to the A matrix after the rows and columns         */
          /*           of the constrained ControlPoints have been removed from the general */ 
          /*           A stiffness matrix.  For Bug151, additional arguments,              */
          /*           map, Sw, nrhs, &nu, were added to call N_AddStiffnessToSrfAMatrix() */
          /*           and that function was modified to work with a reduced A matrix.     */
          N_AddStiffnessToSrfAMatrix(alpha, beta, sur, a_nunu, map, Sw, nrhs, &nu) ; 

          /* Now solve via LU decomposition (Crout with partial piv) */

          perm = N_AllocInt1dArray( nu, &SL );

          if( perm EQ NULL )
              NL_QUIT;

          N_CreateRealMatrix( &rma, nu, nu, a_nunu, NL_MT_FULL, nu );

          error = N_RealMatrixLuDecomposePivot( &rma, perm );

          if( error EQ NL_YES )
              NL_OUT;

          error = N_RealMatrixRightForBackPivot( &rma, perm, nrhs, Qw );

          if( error EQ NL_YES )
              NL_OUT;
        }
    } /* end solve internal branch */
  else
  { /* solve via external method */
    /* gwc: Bad practice here - passing ptrs as ints:                   */
    /*      ptrs are being cast to ints so that two different           */
    /*      user-supplied function signatures can be passed to this     */
    /*      function through a single argument to support user-supplied */
    /*      solver functions for full and sparse matrices.              */
    /*      In better practice - the two different methods should be    */
    /*        two different input arguments to this method with proper  */
    /*        typing of their call arguments.                           */
    /*      For backward compatibility - leave it alone.                */
    /*        some user must have asked for this support and            */
    /*        is still using it.  Odds are other users won't            */
    /*        be using this feature.                                    */
	  if (mflg EQ NL_FULL) 
     {
		     /* Convert from pointer to NL_INDEX (long)
		        For 32 bit this is even
		        For 64 bit a pointer is twice the size of long
		        NL_INDEX* tmp = (int *)(((char *)a_menu) + 2); */       
		     error = fsol((NL_LLONG)a_menu,     /* in : ptr to A matrix of A X= B matrix equation  */
                    me,                   /* in : m of ASize:[m,n]                           */
                    nu,                   /* in : n of ASize:[m,n]                           */
                    rhs,                  /* in : ptr to B matrix of A X = B matrix equation */
                    Qw);                  /* out: ptr to X matrix of A X = B matrix equation */
	    }
	  else /* sparse matrix branch */
     {
       /* gwc: I don't see how this branch can work as written  */
       /*      The user callback arguments do not contain a ptr */
       /*      to the A Matrix which is needed to make the      */
       /*      ST_GetRowsColMatrix1() call work                 */
       /*      the call seems to be missing a ptr to A argument */
		     /* Convert from NL_FLAG (short) to NL_INDEX (long)
		       For 32 bit this is even
		       For 64 bit a short is half the size of long
		       NL_INDEX* tmp = (int *)(((char *)ST_GetRowsColMatrix1) + 2); */
		      error = fsol(me,                              /* in : m of ASize:[m,n]                            */
                     nu,                              /* in : n of ASize:[m,n]                            */
                     (NL_LLONG)ST_GetRowsColMatrix1,  /* in : ptr to fnc that rtns ptrs to A[i][j] vals   */
                     rhs,                             /* in : ptr to B matrix of A X = B matrix equation  */
                     Qw);                             /* out: ptr to X matrix of A X = B matrix equation  */
	    }

      if( error EQ NL_YES )
          NL_ERROR( NL_CAL_ERR );
    
    } /* end solve external branch */

  /* arrive here after solving */
  /* next move solution back into outputSur control point array */

  /* for every u control point */
  for ( ii = 0; ii <= n; ii++ )
    {
      /* for every v control point */
      for ( jj = 0; jj <= m; jj++ )
        {
          /* copy unconstrained dof values */
          if( map[ii][jj]GE 0 )
              N_CopyCPt( Qw[map[ii][jj]], &Sw[ii][jj] );

        } /* end iter every v control point */
    } /* end iter every u control point - seting output control pout values */

  /* End NURBS and Exit */

  EXIT:

  N_EndNurbs( &SL );

  /* all done */
  return (error);

} /* end N_FitSrfLstSqBoundary */

/**********************************************************************/
/* N_DeformableFitSrfLstSqBoundary: Deformable Least squares surface approximation to random points     */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting  routine  computes  a  least squares b-spline surface 
     approximation to a random (non-NxM) set of points. Boundary curves
     may be input  as  constraints.  If boundary curves are input, they
     must be nonrational and  already compatible; i.e. the two u-bound-
     eries must be compatible with one another (in the b-spline sense),
     and the two v-boundaries must  be  compatible  with  one  another.  
     Cross-boundary first derivatives may also be specified. The system
     of equations which is set up may be solved within this routine, or
     by a user-supplied method. If the output surface is initialized to  
     the NULL surface,  memory is allocated locally.  Otherwise  it  is  
     checked  if  enough memory is passed in. A typical calling example 
     is:
 
       NL_POINT      *P;
       NL_CURVE      **bdys, **ders;
       NL_PARAMETER  *uu, *vv;
       NL_REAL       *wts
       NL_INDEX      m, n, np;
       NL_DEGREE     p, q;
       NL_SURFACE    sur;
       NL_STACKS     SG;
       ...
       (define arrays P, wts, uu, vv, U, V, define the boundary curves
        and boundary cross-derivatives, and choose p, q , n and m);
       ...
       N_SrfInitArrays(&sur);
       N_FitSrfLstSqBoundary(P,wts,uu,vv,np,bdys,ders,n,m,p,q,U,V,NL_FULL,NL_SVD,
                NL_YES,NULL,&sur,&SG);
 
 
   ACCESS:
   
     P    , input  ,  Points to be approximated
     wts  , input  ,  Least squares point weights (wts[i] >= 0  for  all
                      i).  The  larger  wts[i],  the  closer the surface
                      comes to P[i]. wts[i] < 1 lessens the influence of
                      P[i]. If wts[i] = 0, then P[i] is not used.  If no
                      weighting is desired, set wts = NULL
     uu   , input  ,  The u-parameters of the points in P
     vv   , input  ,  The v-parameters of the points in P
     np   , input  ,  The high index of the arrays: P,wts,uu,vv
     bdys , input  ,  bdys[i], 0<=i<=3, is a boundary curve of the surf-
                      ace ,  ordered:  i=0=NL_BOTTOM , i=1=NL_RIGHT , i=2=NL_TOP,
                      i=3=NL_LEFT. Set bdys[i]=NULL if the i-th boundary is
                      not specified  (but others are);  set bdys=NULL if
                      no boundaries are specified.  If two u-curves  (or
                      v-curves) are given,  they must already be compat-
                      ible in the b-spline sense.  Furthermore,  the u,v
                      values in arrays uu and vv should fall within  the
                      parameter domains of the boundary curves in bdys[]
                      (doesn't matter if the corresponding wts[i] = 0)
     ders , input  ,  ders[i], 0<=i<=3,  is  a cross-boundary derivative
                      curve  ordered:  i=0=NL_BOTTOM , i=1=NL_RIGHT , i=2=NL_TOP,
                      i=3=NL_LEFT.  Set ders[i]=NULL if the i-th derivative 
                      is not specified  (but others are);  set ders=NULL 
                      if no derivatives are specified.  The  derivatives
                      must already be pairwise compatible and compatible
                      with  the  corresponding  bdys[i].  For  example , 
                      bdys[0], bdys[2], ders[0] and ders[2] must all  be
                      compatible in the b-spline sense. Furthermore, the
                      derivatives must match up,  mathematically, at the
                      corners; i.e. ders[0] evaluated at its start  must
                      equal the derivative evaluated from bdys[3]. Twist
                      derivatives must also match. ders[i] may be speci-
                      fied only if bdys[i] is given.  In determining the
                      surface control points, ders[0] and ders[2] depend
                      on the first 2 v-knots,  and ders[1]  and  ders[3] 
                      depend on the first 2 u-knots.  Hence, if any ders
                      are specified, the corresponding surface knots  (U
                      or V) must be passed in (see below)
     n,m  , input  ,  High indexes of the surface  control  points  (the
                      surface will have (n+1)x(m+1) control points).  If
                      boundaries are given, n and m must be at least  as
                      big as the corresponding indexes in the curves
     p,q  , input  ,  Degrees of the surface  (must be greater  than  or
                      equal to the boundary degrees if bdys[] is given)
     U,V  , input  ,  Knots for the surface.  If  U=NULL or V=NULL,  the
                      corresponding knots are  computed in this routine.
                      If given, there must be n+p+2 u-knots and/or m+q+2
                      v-knots.  The values in the uu and vv arrays  must
                      correspond to the knot ranges (unless wts[i] = 0), 
                      and if  boundaries are specified in bdys[],  their
                      knots must be a subset of the knots in U and V.  U
                      (V) must be input if  ders[1] or ders[3]  (ders[0]
                      or ders[2]) are input
     mflg , input  ,  Flag:
                        NL_FULL  : Use a full coefficient matrix  which re-
                                quires  (np+1)*(n+1)*(m+1)  elements  of 
                                type NL_REAL
                        NL_SPARSE: Use a sparse scheme to store the coeffi-
                                cient matrix.  Sparsity  of  the  matrix 
                                grows as n-p and m-q grow.  This  option
                                is slower than NL_FULL, but it is more mem-
                                ory efficient if np*(n-p)*(m-q) is large
                                (see below for more detail)
     aflg , input  ,  Flag:
                        NL_SVD   : Use the Single Value Decomposition  method
                                to solve the over-determined  system  of
                                equations. This method requies that there
                                be as many or more sample points than control points and
                                that the sample points be distributed so that there
                                is at least one sample point in every patch
                                of the output surface.
                        NL_LUPIV : Set up the normal equations and  use  LU 
                                decomposition with partial  pivoting  to
                                solve. This method works for partial sampling,
                                i.e. fewer sample points (or sample points all
                                grouped together) than control points. 
     iflg , input  ,  Flag:
                        NL_YES   : Solve internally in this routine,  using
                                the method prescribed by the flag, aflg
                        NL_NO    : The system of equations will  be  solved 
                                in a caller-supplied function (fsol)
     fsol , input  ,  An optional caller-supplied function to solve  the
                      over-determined system of linear equations set  up
                      in this routine. This function is required only if
                      iflg = NL_NO. See below for a description of fsol
     sur  , output ,  Approximating surface
     SG   , input  ,  sur's memory stack


     Additional details for the flags: mflg, aflg, and iflg:

       A NL_REAL matrix is set up which has 
       size (np+1)*(n+1)*(m+1) if  mflg = NL_FULL, and 
       size (np+1)*(p+q+2)     if  mflg = NL_SPARSE. 
       The mflg = NL_SPARSE option is not allowed if iflg = NL_YES and aflg = 

       NL_SVD (input error).  The NL_SVD method is noticeably slower than
       the NL_LUPIV method,  but  it  is less susceptible to numerical
       problems arising from ill-conditioned systems.  If there are
       plenty of points covering the fit region, and the number  of
       internal u- and v-knots is not too  high,  then  the  system
       should be well-conditioned,  and  NL_LUPIV  can  be  used (with 
       either  NL_FULL or NL_SPARSE option).  By  setting  iflg = NL_NO  and 
       supplying a function, fsol,  the caller can supply  his  own
       equation solver. fsol must have the following structure:
       mflg=NL_FULL   : NL_FLAG fsol(NL_REAL **a, NL_LLONG me, NL_LLONG nu,
                               NL_POINT *rhs, NL_CPOINT *sol)
                     a is the coefficient matrix with me  equations
                     and nu unknowns (me > nu).  rhs  is the  right
                     hand side vector,  and  sol  is  the  solution 
                     vector (allocated in N_FitSrfLstSqBoundary).  fsol computes
                     sol, and returns error = NL_YES or NL_NO.
       mflg=NL_SPARSE : NL_FLAG fsol(NL_LLONG me, NL_LLONG nu, 
                               NL_FLAG (*fetcha)(), NL_POINT *rhs, 
                               NL_CPOINT *sol)
                     fetcha() is a function which fetches a column,
                     row,  or  single  element  of  the  underlying 
                     me x nu matrix (me > nu). fetcha is defined in
                     N_FitSrfLstSqBoundary as:
                     NL_FLAG fetcha(NL_INDEX i, NL_INDEX j, NL_REAL *item)
                     i>=0,j<0 : the i-th row is returned in item
                     i<0,j>=0 : the j-th column is returned in item
                     i>=0,j>=0: the single (i,j)-th element is  re-
                                turned in item
                     It is an error if  i>me, j>nu, or i<0 and j<0.
                     Memory for item must be allocated and  deallo-
                     cated in fsol. rhs is the right hand side vec-
                     tor, and sol is the solution vector (allocated 
                     in N_FitSrfLstSqBoundary).  fsol computes sol,  and returns 
                     error = NL_YES or NL_NO.
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitSrfLstSqDeformablePeriodic
  (NL_POINT     *P,                                  /* in : sample points, sized:[np+1] */
   NL_REAL      *wts,                                /* in : opt least squares weights, sized:[np+1], */
                                                     /*      NULL to ignore, sized:[np+1] */
   NL_REAL      *uu,                                 /* in : u param for every P point, sized:[np+1] */
   NL_REAL      *vv,                                 /* in : v param for every P point, sized:[np+1] */
   NL_INDEX      np,                                 /* in : size param, size of P, uu, and vv arrays */
   NL_CURVE   ** bdys,                               /* in : opt compatible bdry curves: 0=Bottom, 1=Right, 2=Top, 3=Left,  */
                                                     /*      bdys[i] = NULL to ignore one or more curves, */
                                                     /*      bdys = NULL to ignore all curves */
   NL_INDEX      n,                                  /* in : output sur highest ControlPoint index U */
   NL_INDEX      m,                                  /* in : output sur highest ControlPoint index V */
   NL_DEGREE     p,                                  /* in : output sur degree U */
   NL_DEGREE     q,                                  /* in : output sur degree V */
   NL_REAL      *U,                                  /* in : opt sur KnotVector U, NULL=function computes a likely knot vector */
   NL_REAL      *V,                                  /* in : opt sur KnotVector V, NULL=function computes a likely knot vector */
   NL_REAL       alpha,                              /* in : Resistance to stretch weight term */
   NL_REAL       beta,                               /* in : Resistance to bending weight term */          
   NL_REAL       PtStiffness,                        /* in : Scales stiffness matrix. Higher value lessens impact of alpha and beta */       
   NL_FLAG       uper,                               /* in : u periodic flag: NL_YES = periodic in u, NL_NO = not    */                       
   NL_FLAG       vper,                               /* in : v periodic flag: NL_YES = periodic in v, NL_NO = not    */   
   NL_FLAG       mflg,                               /* in : memory flag: NL_FULL   = more memory, faster, */
                                                     /*                   NL_SPARSE = less memory, slower  */
   NL_FLAG       aflg,                               /* in : solver flag: NL_SVD=SingleValueDecomposition, (requires full or over samplind sampling) */
                                                     /*                   NL_LUPIV=LU Decomposition with partial pivoting, allows sparse sampling */
   NL_FLAG       iflg,                               /* in : solver cntrl: NL_YES=use the aflg internal solver,        */
                                                     /*                    NL_NO=use the user supplied callback solver */
   NL_FLAG (*fsol)                                   /* in : opt solver callback, use this if you own a better solver, used when iflg = NL_NO  */
                                                     /*      for | Full matrices                   | For Sparse matrices                       */
             (NL_LLONG,  /* MatPtr cast to int */    /*          | in : A MatrixPtr of A X = B eqn | m of ASize:[m,n]                          */
              NL_LLONG,                              /*          | in : m of ASize:[m,n]           | n of ASize:[m,n]                          */
              NL_LLONG,  /* FncPtr cast to int */    /*          | in : n of ASize:[m,n]           | ptr to ST_GetRowsColMatrix1(i,j,A) to rtn A[i][j] ptrs */
              NL_POINT *,                            /*          | in : B MatrixPtr of A X = B eqn | B MatrixPtr of A X = B eqn                */
              NL_CPOINT * ),                         /*          | in : X MatrixPtr of A X = B eqn | X MatrixPtr of A X = B eqn                */
   NL_SURFACE   *sur,                                /* out: the approximated surface */
   NL_STACKS    *SG)                                 /* in : sur's memory stack */
{
  NL_PRIVATE NL_STRING rname = _T("N_DeformableFitSrfLstSqBoundary");

  NL_FLAG error = NL_NO, pflg;

  NL_INDEX n1, n2, ii, jj, kk, nb, mb, ** map, me, nu, uspan = 0, vspan = 0, j1, k1, *perm, ** uvsp = NULL, ** cind = NULL;
  NL_INDEX i0, j0, i1, USpan, VSpan, mm, ll ;
  NL_REAL *pBu, *pBv, dBm ; 
  /* NL_REAL alpha, beta, PtStiffness; */

  NL_DEGREE pp;

  NL_CURVE * loc_bdys[4];

  NL_REAL *kts1 = NULL, *kts2 = NULL, *UB, *VB, *US, *VS, *ufuns, *vfuns, ** a_menu = NULL, ** a_nunu, *svd_w, dd, w, u1, u2, v1, v2, ** a_pq = NULL, u0, v0;
  /* NL_REAL *col1, *col2 ; */ 

  NL_CPOINT *Pw, *Qw, ** Sw;

  NL_POINT P1, *rhs, *nrhs;

  NL_KNOTVECTOR knt, *knu, *knv, *knus, *knvs;

  NL_RMATRIX rma;

  NL_STACKS SL;

  NL_REAL ** ND, ** ND2;
  NL_INDEX mc = -1, uspan2, vspan2;
  NL_REAL ** MM;
  NL_POINT * G, P2;
  NL_RMATRIX lma;


  /* Start NURBS */

  N_InitNurbs( &SL );

  /* Check for input errors */

  /* at least 2 control points in u and v */
  if( n LT 1 OR m LT 1)
      NL_ERROR( NL_INP_ERR );

  /* when using SVD solver make sure that there are at least as many sample equations as dofs         */
  /*   else switch to LUPIV solver.  GWC:Bug151                                                       */                        
  if( aflg EQ NL_SVD AND (np + 1 LT( n + 1 ) * (m + 1)) )
    { 
      /* switch to the LUPIV solver - it can handle subsampling */
      aflg = NL_LUPIV ;
    }

  /* enough control points for at least one patch or requested degree */
  if( n LT p OR m LT q )
      NL_ERROR( NL_INP_ERR );

  /* when using internal solver and SingleValueDecomposition, must have a FULL matrix*/
  if( mflg EQ NL_SPARSE AND aflg EQ NL_SVD AND iflg EQ NL_YES )
      NL_ERROR( NL_INP_ERR );

  nb = mb = -1;
  UB = VB = NULL;

  /* initialise just to avoid compiler complaint */
  u1 = v1 = 0.0;
  u2 = v2 = 1.0;

  /* when given optional boundaries - make sure                                          */
  /*  1. all curve degrees are same, elevate degree as needed                            */
  /*  2. surface has more control points in u or v direction than appropriate boundaries */
  /*  3. all matched boundary curves are compatible                                      */
  if( bdys NEQ NULL )
    {
      /* local boundary copies */
      for ( ii = 0; ii <= 3; ii++ )
          loc_bdys[ii] = bdys[ii];

      /* when given either bot or top bdy curve */
      if( loc_bdys[0]NEQ NULL OR loc_bdys[2]NEQ NULL )
        {
          n1 = n2 = -1;

          /* when given bot curve */
          if( loc_bdys[0]NEQ NULL )
            {
              if( N_IsCrvRat( loc_bdys[0] ) )
                  NL_ERROR( NL_INP_ERR );
              N_CrvGetCPtsDegreeAndKnots( loc_bdys[0], &n1, &Pw, &pp, &ii, &kts1 );

              /* check: bot curve degree is higher than requested output surface degree */
              if( pp GT p )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else if( pp LT p ) /* elevate bot curve degree to surface degree */
                {
                  loc_bdys[0] = N_AllocCrv( &SL );
                  N_CrvInitArrays( loc_bdys[0] );
                  error = N_CrvElevateDegree( bdys[0], p - pp, loc_bdys[0], SG, &SL );

                  if( error EQ NL_YES )
                      NL_OUT;
                  N_CrvGetCPtsDegreeAndKnots( loc_bdys[0], &n1, &Pw, &pp, &ii, &kts1 );
                }
            } /* end given bot bndry curve check */

          /* when given top curve */
          if( loc_bdys[2]NEQ NULL )
            {
              if( N_IsCrvRat( loc_bdys[2] ) )
                  NL_ERROR( NL_INP_ERR );
              N_CrvGetCPtsDegreeAndKnots( loc_bdys[2], &n2, &Pw, &pp, &ii, &kts2 );

              /* check: top curve degree is higher than requested output surface degree */
              if( pp GT p )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else if( pp LT p ) /* elevate top curve degree to surface degree */
                {
                  loc_bdys[2] = N_AllocCrv( &SL );
                  N_CrvInitArrays( loc_bdys[2] );
                  error = N_CrvElevateDegree( bdys[2], p - pp, loc_bdys[2], SG, &SL );

                  if( error EQ NL_YES )
                      NL_OUT;
                  N_CrvGetCPtsDegreeAndKnots( loc_bdys[2], &n2, &Pw, &pp, &ii, &kts2 );
                }
            } /* end given top bndry curve check */

          /* highest CPoint index in top/bot pos, and all top/bot boundary curves */
          nb = NL_MAX( n1, n2 );

          /* check: more cPts in any top/bot bndry curve than the output surface UDir */
          if( nb GT n )
              NL_ERROR( NL_INP_ERR );
          
          /* when given bot curve - use it's knot vector for output sur UDir knot vector */
          if( n1 GT 0 )
            {
              if( n1 NEQ nb )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else
                  UB = kts1;
            }

          /* when given top curve - use it's knot vector for output sur UDir knot vector  */
          if( n2 GT 0 )
            {
              if( n2 NEQ nb )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else
                  UB = kts2;
            }

          /* kk = output sur UDir knot count */
          kk = nb + 1;

          /* check: all top/bot bdry and der curves have same knots */
          for ( ii = p; ii <= kk; ii++ )
            {
              if( n1 GT 0 )
                  if( kts1[ii]NEQ UB[ii] )
                      NL_ERROR( NL_INP_ERR );

              if( n2 GT 0 )
                  if( kts2[ii]NEQ UB[ii] )
                      NL_ERROR( NL_INP_ERR );
            }

          /* check: bdryCurve/OutputSur UDir knot compatibility */
          if( U NEQ NULL ) 
            {
              /* check: bdry curve and output sur UDir knot intervals the same */
              if( UB[0]NEQ U[0]OR UB[kk]NEQ U[n + 1] )
                  NL_ERROR( NL_INP_ERR );

              ii = jj = p + 1;

              /* check if UB (curve knots) is a subset of U (output sur UDir knots) */
              while( ii LT kk )
                {
                  if( U[jj]GT UB[ii] )
                    {
                      NL_ERROR( NL_INP_ERR );
                    }
                  else if( U[jj]EQ UB[ii] )
                    {
                      ii += 1;
                      jj += 1;
                    }
                  else
                      while( U[jj]LT UB[ii] )
                          jj += 1;
                } /* end while matching top/bot curve knots with output sur UDir knots */
            } /* end bdryCurve/OutputSur UDir knot compatibility check */
        } /* end when given either bot or top bdy curve */

      /* when given either rgt or lft bdy curve */
      if( loc_bdys[1]NEQ NULL OR loc_bdys[3]NEQ NULL )
        {
          n1 = n2 = -1;

          /* when given rgt curve */
          if( loc_bdys[1]NEQ NULL )
            {
              if( N_IsCrvRat( loc_bdys[1] ) )
                  NL_ERROR( NL_INP_ERR );
              N_CrvGetCPtsDegreeAndKnots( loc_bdys[1], &n1, &Pw, &pp, &ii, &kts1 );

              /* check: rgt curve degree is higher than requested output surface degree */
              if( pp GT q )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else if( pp LT q ) /* elevate rgt curve degree to surface degree */
                {
                  loc_bdys[1] = N_AllocCrv( &SL );
                  N_CrvInitArrays( loc_bdys[1] );
                  error = N_CrvElevateDegree( bdys[1], q - pp, loc_bdys[1], SG, &SL );

                  if( error EQ NL_YES )
                      NL_OUT;
                  N_CrvGetCPtsDegreeAndKnots( loc_bdys[1], &n1, &Pw, &pp, &ii, &kts1 );
                }
            } /* end given rgt bndry curve check */

          /* when given lft curve */
          if( loc_bdys[3]NEQ NULL )
            {
              if( N_IsCrvRat( loc_bdys[3] ) )
                  NL_ERROR( NL_INP_ERR );
              N_CrvGetCPtsDegreeAndKnots( loc_bdys[3], &n2, &Pw, &pp, &ii, &kts2 );

              /* check: lft curve degree is higher than requested output surface degree */
              if( pp GT q )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else if( pp LT q ) /* elevate lft curve degree to surface degree */
                {
                  loc_bdys[3] = N_AllocCrv( &SL );
                  N_CrvInitArrays( loc_bdys[3] );
                  error = N_CrvElevateDegree( bdys[3], q - pp, loc_bdys[3], SG, &SL );

                  if( error EQ NL_YES )
                      NL_OUT;
                  N_CrvGetCPtsDegreeAndKnots( loc_bdys[3], &n2, &Pw, &pp, &ii, &kts2 );
                }
            } /* end given lft bndry curve check */

          /* highest CPoint index in rgt/lft pos, and all rgt/lft boundary curves */
          mb = NL_MAX( n1, n2 );

          /* check: more cPts in any rgt/lft bndry curve than the output surface VDir */
          if( mb GT m )
              NL_ERROR( NL_INP_ERR );

          /* when given rgt curve - use it's knot vector for output sur vDir knot vector */
          if( n1 GT 0 )
            {
              if( n1 NEQ mb )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else
                  VB = kts1;
            }

          /* when given lft curve - use it's knot vector for output sur VDir knot vector  */
          if( n2 GT 0 )
            {
              if( n2 NEQ mb )
                {
                  NL_ERROR( NL_INP_ERR );
                }
              else
                  VB = kts2;
            }

          /* kk = output sur VDir knot count */
          kk = mb + 1;

          /* check: all rgt/lft bdry and der curves have same knots */
          for ( ii = q; ii <= kk; ii++ )
            {
              if( n1 GT 0 )
                  if( kts1[ii]NEQ VB[ii] )
                      NL_ERROR( NL_INP_ERR );

              if( n2 GT 0 )
                  if( kts2[ii]NEQ VB[ii] )
                      NL_ERROR( NL_INP_ERR );
            }

          /* check: bdryCurve/OutputSur VDir knot compatibility */
          if( V NEQ NULL ) 
            {
              /* check: bdry curve and output sur VDir knot intervals the same */
              if( VB[0]NEQ V[0]OR VB[kk]NEQ V[m + 1] )
                  NL_ERROR( NL_INP_ERR );

              ii = jj = q + 1;

              /* check if VB (curve knots) is a subset of V (output sur VDir knots) */
              while( ii LT kk )
                {
                  if( V[jj]GT VB[ii] )
                    {
                      NL_ERROR( NL_INP_ERR );
                    }
                  else if( V[jj]EQ VB[ii] )
                    {
                      ii += 1;
                      jj += 1;
                    }
                  else
                      while( V[jj]LT VB[ii] )
                          jj += 1;
                } /* end while matching rgt/lft curve knots with output sur VDir knots */
            } /* end bdryCurve/OutputSur VDir knot compatibility check */
        } /* end when given either rgt or lft bdy curve */
    } /* end when given optional boundaries check */

  /* arrive here when optional boundary pos and cross-der curves are */
  /*  1. the same degree as the output sur                           */
  /*  2. have compatible knot vectors amongst themselves             */
  /*  3. have compatible knot vectors with the output sur            */

  /* check and if needed allocate output sur memory */
  error = N_SrfSizeArrays( sur, n, m, p, q, n + p + 1, m + q + 1, rname, SG );
  if( error EQ NL_YES )
      NL_OUT;

  /* sur locals */
  N_SrfGetCPtsAndKnots( sur, &Sw, &US, &VS );
  N_SrfGetKnotVectors( sur, &knu, &knv );

  /* Get the surface u- and v-knots */

  pflg = NL_NO;

  /* when given optional output sur U Knots - use them */
  if( U NEQ NULL )
    {
      kk = n + p + 1;

      /* set output sur U knots to U Knots */
      for ( ii = 0; ii <= kk; ii++ )
          US[ii] = U[ii];
      knus = knu;
    }
  else /* not given option sur U knots - copy from bot/top bdry if possible or set knus = NULL */
    {
      kk = 0;

      /* remember when given an optional bot/top bdry curve */
      if( bdys NEQ NULL )
          if( bdys[0]NEQ NULL OR bdys[2]NEQ NULL )
              kk = 1;

      /* when given an optional bot/top bdry curve */
      if( kk EQ 1 )
        {
          /* set knus = optional bot/top bdry curve knot vector */
          if( bdys[0]NEQ NULL )
              N_CrvGetKnotVector( loc_bdys[0], &knus );
          else
              N_CrvGetKnotVector( loc_bdys[2], &knus );
        }
      else /* not given a bot/top bdry curve */
        {
          /* set knus to NULL for now */
          knus = NULL;

          /* when given optional wts vector */
          if( wts NEQ NULL )
            {
              /* set pflg to use given u min/max values for upcoming N_FitSrfCalcKnotVectors call */
              pflg = NL_YES;
              u1 = 1.0e+20;
              u2 = -u1;

              /* store max/min input point u values in u1/u2 */
              for ( ii = 0; ii <= np; ii++ )
                  if( wts[ii]NEQ 0.0 )
                    {
                      if( u1 GT uu[ii] )
                          u1 = uu[ii];

                      if( u2 LT uu[ii] )
                          u2 = uu[ii];
                    }
            } /* end given optional wts vector check - then get min/max input point u value check */
        } /* end not given a bot/top bdry curve branch */
    } /* end not given option sur U knots - copy from bot/top bdry if possible or set knus = NULL branch */

  /* when given optional output sur V Knots - use them */
  if( V NEQ NULL )
    {
      kk = m + q + 1;

      /* set output sur V knots to V Knots */
      for ( ii = 0; ii <= kk; ii++ )
          VS[ii] = V[ii];
      knvs = knv;
    }
  else /* not given option sur V knots - copy from rgt/lft bdry if possible or set knvs = NULL */
    {
      kk = 0;

      /* remember when given an optional rgt/lft bdry curve */
      if( bdys NEQ NULL )
          if( bdys[1]NEQ NULL OR bdys[3]NEQ NULL )
              kk = 1;

      /* when given an optional rgt/lft bdry curve */
      if( kk EQ 1 )
        {
          /* set knvs = optional rgt/lft bdry curve knot vector */
          if( bdys[1]NEQ NULL )
              N_CrvGetKnotVector( loc_bdys[1], &knvs );
          else
              N_CrvGetKnotVector( loc_bdys[3], &knvs );
        }
      else /* not given a rgt/lft bdry curve */
        {
          /* set knvs to NULL for now */
          knvs = NULL;

          /* when given optional wts vector */
          if( wts NEQ NULL )
            {
              /* set pflg to use given v min/max values for upcoming N_FitSrfCalcKnotVectors call */
              pflg = NL_YES;
              v1 = 1.0e+20;
              v2 = -v1;

              /* store max/min input point v values in v1/v2 */
              for ( ii = 0; ii <= np; ii++ )
                  if( wts[ii]NEQ 0.0 )
                    {
                      if( v1 GT vv[ii] )
                          v1 = vv[ii];

                      if( v2 LT vv[ii] )
                          v2 = vv[ii];
                    }
            } /* end given optional wts vector check - then get min/max input point v value check */
        } /* end not given a rgt/lft bdry curve branch */
    } /* end not given option sur V knots - copy from rgt/lft bdry if possible or set knvs = NULL branch */

  /* arrive here when:                                                                     */
  /*  1. optional bdry curves are compatible with each other and the output sur            */
  /*  2. knus and knvs are set if option U or V vector or optional bdry curves are given   */
  /*  3. knus and knvs are NULL if not set by input and                                    */
  /*       if given optional least squares wts vector,                                     */
  /*       then min/max input point u and v values are saved in u1/u2 and v1/v2            */

  /* when not given optional U and V output sur knot vectors */
  if( knus NEQ knu OR knvs NEQ knv )
    {
      error = N_FitSrfCalcKnotVectors
        ( uu,       /* in : u parameter values, sized:[nn+1] */                                               
          vv,       /* in : v parameter values, sized:[nn+1] */                                               
          np,       /* in : highest u and v index */                                                          
          p,        /* in : approximating surface degree U */                                                 
          q,        /* in : approximating surface degree V */                                                 
          pflg,     /* in : NL_YES = use us,ue,vs,ve values to set max/min u and v knot values */             
                    /*      NL_NO  = find max/min u and v knot values in u and v arrays        */             
          u1,       /* in : min U param value, overridden by any input knu value   */                         
          u2,       /* in : max U param value, overridden by any input knu value   */                         
          v1,       /* in : min V param value, overridden by any input knv value   */                         
          v2,       /* in : max V param value, overridden by any input knv value   */                         
          knus,     /* in : opt starter U knots, if given these knots will be in the output, NULL to ignore */
          knvs,     /* in : opt starter V knots, if given these knots will be in the output, NULL to ignore */
          knu,      /* i/o: sized knot vector whose knot values are to be determined */                       
          knv );    /* i/o: sized knot vector whose knot values are to be determined */                       

      if( error EQ NL_YES )
          NL_OUT;
    } /* end need to Calc Knot Vectors check */

  /* Refine the boundary curves if their knots are not */
  /* the same as the surface knots.                    */

  /* when given optional bdry curves and they have less control points than output sur */
  if( nb GT 0 AND nb LT n )
    {
      /* get memory for knot values to be added to the optional bdry curves */
      kts1 = N_AllocReal1dArray( n - nb - 1, &SL );

      if( kts1 EQ NULL )
          NL_QUIT;

      kk = 0;
      ii = jj = p + 1;

      /* get list of U knot values in outputSur that are not in boundary curves */
      while( jj <= n )
          if( UB[ii]EQ US[jj] )  /* skip knots in both bdry and sur knot vectors */
            {
              ii += 1;
              jj += 1;
            }
          else /* load kts1 with unique sur knot values */
            {
              kts1[kk++] = US[jj++];
            }

      /* make a KnotVector from the kts1 array of unique surface knot values */
      kk -= 1;
      N_KnotVectorFromRealArray( &knt, kts1, kk );

      /* when given bot curve */
      if( loc_bdys[0]NEQ NULL )
        {
          /* insert sur unique knots into bot curve */
          if( loc_bdys[0]NEQ bdys[0] )
              error = N_CrvRefine( loc_bdys[0], &knt, loc_bdys[0], &SL, &SL );
          else
            {
              loc_bdys[0] = N_AllocCrv( &SL );
              N_CrvInitArrays( loc_bdys[0] );
              error = N_CrvRefine( bdys[0], &knt, loc_bdys[0], SG, &SL );
            }

          if( error EQ NL_YES )
              NL_OUT;
        } /* end need to insert knots into bot curve check */

      /* when given top curve */
      if( loc_bdys[2]NEQ NULL )
        {
          /* insert sur unique knots into top curve */
          if( loc_bdys[2]NEQ bdys[2] )
              error = N_CrvRefine( loc_bdys[2], &knt, loc_bdys[2], &SL, &SL );
          else
            {
              loc_bdys[2] = N_AllocCrv( &SL );
              N_CrvInitArrays( loc_bdys[2] );
              error = N_CrvRefine( bdys[2], &knt, loc_bdys[2], SG, &SL );
            }

          if( error EQ NL_YES )
              NL_OUT;
        } /* end need to insert knots into top curve check */
    } /* end given optional bdry curves and they have less control points than output sur check */

  /* when given optional bdry curves and they have less control points than output sur */
  if( mb GT 0 AND mb LT m )
    {
      /* get memory for knot values to be added to the optional bdry curves */
      kts1 = N_AllocReal1dArray( m - mb - 1, &SL );

      if( kts1 EQ NULL )
          NL_QUIT;

      kk = 0;
      ii = jj = q + 1;

      /* get list of V knot values in outputSur that are not in boundary curves */
      while( jj <= m )
          if( VB[ii]EQ VS[jj] ) /* skip knots in both bdry and sur knot vectors */
            {
              ii += 1;
              jj += 1;
            }
          else /* load kts1 with unique sur knot values */
            {
              kts1[kk++] = VS[jj++];
            }

      /* make a KnotVector from the kts1 array of unique surface knot values */
      kk -= 1;
      N_KnotVectorFromRealArray( &knt, kts1, kk );

      /* when given rgt curve */
      if( loc_bdys[1]NEQ NULL )
        {
          /* insert sur unique knots into rgt curve */
          if( loc_bdys[1]NEQ bdys[1] )
              error = N_CrvRefine( loc_bdys[1], &knt, loc_bdys[1], &SL, &SL );
          else
            {
              loc_bdys[1] = N_AllocCrv( &SL );
              N_CrvInitArrays( loc_bdys[1] );
              error = N_CrvRefine( bdys[1], &knt, loc_bdys[1], SG, &SL );
            }

          if( error EQ NL_YES )
              NL_OUT;
        } /* end need to insert knots into rgt curve check */

      /* when given lft curve */
      if( loc_bdys[3]NEQ NULL )
        {
          /* insert sur unique knots into lft curve */
          if( loc_bdys[3]NEQ bdys[3] )
              error = N_CrvRefine( loc_bdys[3], &knt, loc_bdys[3], &SL, &SL );
          else
            {
              loc_bdys[3] = N_AllocCrv( &SL );
              N_CrvInitArrays( loc_bdys[3] );
              error = N_CrvRefine( bdys[3], &knt, loc_bdys[3], SG, &SL );
            }

          if( error EQ NL_YES )
              NL_OUT;
        } /* end need to insert knots into lft curve check */
    } /* end given optional bdry curves and they have less control points than output sur check */

  /* arrive here when                                                                 */
  /*  1. output sur has been assigned knot vectors                                    */
  /*  2. Sur knot vectors are compatible with all given optional bdry curves  */

  /* next set up maps; mapping between sur controlPoints(ii,jj) to least squares dofs[kk]  */
  /* nu         = highest index of least squares dof (dof count = nu + 1)                          */
  /* map[ii,jj] = LeastSquares dof index kk for sur ControlPoint[ii,jj], -1 = no dof (constrained) */
  /*              for all ii, jj where 0 <= ii <= n, 0<= jj <= m                                   */
  /* cind[kk,0] = associated control point ii value                                                */
  /* cind[kk.1] = associated control point jj value                                                */

  /* Allocate memory for the ControlPoint->DofIndex map */
  map = N_AllocInt2dArray( n, m, &SL );

  if( map EQ NULL )
      NL_QUIT;

  /* when needed */
  if( mflg EQ NL_SPARSE )
    {
      /* Allocate memory for the DofIndex->ControlPoint map */
      cind = N_AllocInt2dArray( (n + 1) * (m + 1) - 1, 1, &SL );

      if( cind EQ NULL )
          NL_QUIT;
    }

  /* Define map : ControlPoint(i,j)->DofIndex[k]  and  */
  /*        cind: DofIndex[k,0]->ControlPoint(i)       */
  /*              DofIndex[k,1]->ControlPoint(j)       */

  nu = -1; /* least squares highest dof index, number of dofs = nu+1         */
           /* nu = TotalControlPointCount - ConstrainedControlPointCount - 1 */

  /* for every outputSur V control point - assign a least squares dof index value */
  for ( jj = 0; jj <= m; jj++ )
    {
      /* clever iter limits to help set up map and cind indices */
      n1 = 0;
      n2 = n;

      /* when given optinoal bdry curves - adjust n1 and n2 values */
      if( bdys NEQ NULL )
        {
          /* kk flag: kk = 1 for working on a given boundary curve */
          /*          kk = 0 otherwise */
          kk = 0;

          /* remember when targeting a given bot curve */
          if( jj EQ 0 AND bdys[0]NEQ NULL )
              kk = 1;

          /* remember when targeting a given top curve */
          else if( jj EQ m AND bdys[2]NEQ NULL )
              kk = 1;

          /* when working on a targeted given bdry curve - set n1/n2 iterator bounds */
          if( kk EQ 1 )
            {
              n1 = n + 1;
              n2 = n;
            }
          else /* not working on a given bdry curve - modify n1/n2 iterator bounds to mark constrained dofs */
            {
              if( bdys[1]NEQ NULL )
                  n2 = n - 1;

              if( bdys[3]NEQ NULL )
                  n1 = 1;
            }
        } /* end need to adjust n1/n2 values due to given optional bdry curves check */

      /* mark constrained control points */
      for ( ii = 0; ii < n1; ii++ )
          map[ii][jj] = -1;

      /* assign dof indices to unconstrained control points */
      for ( ii = n1; ii <= n2; ii++ )
        {
          nu += 1;
          map[ii][jj] = nu;

          if( mflg EQ NL_SPARSE )
            {
              cind[nu][0] = ii;
              cind[nu][1] = jj;
            }
        }

      /* mark constrained control points */
      for ( ii = n2 + 1; ii <= n; ii++ )
          map[ii][jj] = -1;
  
  } /* end iter every outputSur V Control Point */

  /* arrive here when map and cind are built as */
  /* map : ControlPoint(i,j)->DofIndex[k], (map[i][j] == -1 for constrained ControlPoint[i][j])  and  */
  /* cind: DofIndex[k,0]->ControlPoint(i)                                                             */
  /*       DofIndex[k,1]->ControlPoint(j)                                                             */

  /* Load the surface's constrained control points directly into Sw */
  /* Sw = outpuSur control Point array                              */
  if( bdys NEQ NULL )
    {
      /* when given bot curve */
      if( loc_bdys[0]NEQ NULL )
        {
          N_CrvGetCPts( loc_bdys[0], &kk, &Pw ); /* kk is equal to n */

          /* load bot control points into Sw array */
          for ( ii = 0; ii <= kk; ii++ )
              N_CopyCPt( Pw[ii], &Sw[ii][0] );
        } /* end given bot curve check */

      /* when given top curve */
      if( loc_bdys[2]NEQ NULL )
        {
          N_CrvGetCPts( loc_bdys[2], &kk, &Pw ); /* kk is equal to n */

          /* load top control points into Sw array */
          for ( ii = 0; ii <= kk; ii++ )
              N_CopyCPt( Pw[ii], &Sw[ii][m] );
        } /* end given top curve check */

      /* when given lft curve */
      if( loc_bdys[3]NEQ NULL )
        {
          N_CrvGetCPts( loc_bdys[3], &kk, &Pw ); /* kk is equal to m */

          /* load lft control points into Sw array */
          for ( ii = 0; ii <= kk; ii++ )
              N_CopyCPt( Pw[ii], &Sw[0][ii] );
        } /* end given lft curve check */

      /* when given rgt curve */
      if( loc_bdys[1]NEQ NULL )
        {
          N_CrvGetCPts( loc_bdys[1], &kk, &Pw ); /* kk is equal to m */

          /* load rgt control points into Sw array */
          for ( ii = 0; ii <= kk; ii++ )
              N_CopyCPt( Pw[ii], &Sw[n][ii] );
        } /* end given rgt curve check */
    } /* end given bdry curves check */

  /* arrive here when map and cind are built as */
  /* map : ControlPoint(i,j)->DofIndex[k], (map[i][j] == -1 for constrained ControlPoint[i][j])  and  */
  /* cind: DofIndex[k,0]->ControlPoint(i)                                                             */
  /*       DofIndex[k,1]->ControlPoint(j)                                                             */
  /* Sw  : (outputSur ControlPoint array) set with all constrained bdry curve values                  */

  /* next: */

  /* Allocate rhs and Qw array memory for matrix equation [A] [Qw] =  [rhs] */
  /*  sized nu = totalControlPointCount - ConstrainedControlPointCount      */
  Qw = N_AllocCPt1dArray( nu, &SL );
  if( Qw EQ NULL )
      NL_QUIT;

  rhs = N_AllocPt1dArray( np, &SL );
  if( rhs EQ NULL )
      NL_QUIT;

  /* Now set up the overdetermined system of equations */

  /* outputSur domain intervals */
  u1 = US[0];
  u2 = US[n + 1];
  v1 = VS[0];
  v2 = VS[m + 1];

  w = 1.0;
  u0 = u1 - 1.0;
  v0 = v1 - 1.0;

  me = -1; /* number of points used for setting up the least squares problem       */
           /* points are culled when outsied the outputSur domain or when optional */
           /* least squares weight is set to a negative value                      */

  /* when asked for Full matrices */ 
  if( mflg EQ NL_FULL )
    { 
      /* use full storage scheme for matrix eqn [a_menu] * [Sw] = [rhs]                                     */
      /*  a_menu = a_menu [InputPointCount x DofCount]                                                      */
      /*     each row of a_menu is a weighted expression of                                                 */
      /*     "let the surface shape equal the sample point position" as                                     */
      /*       sqrt(w) * (sur(ui,vi) = Pi)                                                                  */
      /*          where: sur(ui,vi) = Sum_i(Sum_j( BU[i]*BV[j] * Q[i,j]))                                   */
      /*                 rhs[i]     = sqrt(weight)*Pi                                                       */
      /*                 Sw[i,j]    = outputSur control points                                              */
      /*                 BU[i]      = nonZero U basis function values for UVPoint(ui,vi)                    */       
      /*                 BV[j]      = nonZero V basis function values for UVPoint(ui,vi)                    */
      /*  with one twiddle for enforcing constrained dof values:                                            */
      /*    All column entries of a_menu associated with a constrained dof are                              */
      /*    multipled by the constrained dof value and subtracted from the rhs vector.                      */
      /*  So  [a_menu] * Sw = [rhs] becomes                                                                 */
      /*      [a_menu'] * Qw = [rhs] - [a_menu] * Sw                                                        */
      /*       a_menu' = a_menu with all columns associated with constrained dofs set to 0.0                */
      /*       QsConstrained = Sw but with all nonConstrained dofs set to 0.0                               */
      /*                                                                                                    */
      /*  ufuns  = ufuns  [UDeg+1] stores nonVanishing OutputSur U basis function values for given u value  */
      /*  vfuns  = vfuns  [VDeg+1] stores nonVanishing OutputSur V basis function values for given v value  */

      a_menu = N_AllocReal2dArray( np, nu, &SL );
      if( a_menu EQ NULL )
          NL_QUIT;

      ufuns = N_AllocReal1dArray( p + q + 1, &SL );
      if( ufuns EQ NULL )
          NL_QUIT;

      vfuns = &ufuns[p + 1];

      /* init a_menu[i,j] = 0.0 */
      for ( ii = 0; ii <= np; ii++ )
          for ( jj = 0; jj <= nu; jj++ )
              a_menu[ii][jj] = 0.0;

      /* set up system */

      /* for every SamplePoint - build the OutputSur(ui,vi) = Pi eqns, skipping points outside OutputSur domain */
      for ( ii = 0; ii <= np; ii++ )
        {
          /* skip points outside the sur U interval */
          if( uu[ii]LT u1 OR uu[ii]GT u2 )
              continue;

          /* skip points outside the sur V interval */
          if( vv[ii]LT v1 OR vv[ii]GT v2 )
              continue;

          /* when given point weights */
          if( wts NEQ NULL )
            {
              /* skip nonPositive weighted points */
              if( wts[ii]LE 0.0 )
                  continue;
              else /* set w with this point's sqrt(weight value) */
                {
                  if( wts[ii]EQ 1.0 )
                      w = 1.0;
                  else
                      w = sqrt( wts[ii] );
                }
            } /* end given point weights check */

          /* count the number of sample points used */
          me += 1;

          /* when uu[ii] is different than last uu value */
          if( uu[ii]NEQ u0 )
            {
              u0 = uu[ii];

              /* get nonVanishing U basis values */
              error = N_BasisEval( knu, p, uu[ii], NL_LEFT, ufuns, &uspan );

              if( error EQ NL_YES )
                  NL_OUT;
            } /* end uu[ii] is new check */

          /* when vv[ii] is different than last vv value */
          if( vv[ii]NEQ v0 )
            {
              v0 = vv[ii];
              
              /* get nonVanishing V basis values */
              error = N_BasisEval( knv, q, vv[ii], NL_LEFT, vfuns, &vspan );

              if( error EQ NL_YES )
                  NL_OUT;
            } /* end vv[ii] is new check */

          /* load rhs with SamplePoint value */
          N_CopyPt( P[ii], &rhs[me] );

          /* when sample point weight is nonUnit */
          if( w NEQ 1.0 )
            {
              u0 = u1 - 1.0; /* remember that ufuns changed */

              /* apply nonUnit weight to ufuns and PointValue */
              for ( jj = 0; jj <= p; jj++ )
                  ufuns[jj] *= w;
              N_ScalePt( w, rhs[me], &rhs[me] );
            }

          /* for every nonZero u basis function for this sample point */
          for ( jj = 0; jj <= p; jj++ )
            {
              /* U effected Control point index */
              j1 = uspan - p + jj;

              /* for every nonZero v basis function for this sample point */
              for ( kk = 0; kk <= q; kk++ )
                {
                  /* V effected control point index */
                  k1 = vspan - q + kk;

                  /* when effected control point is not constrained */
                  if( map[j1][k1]GE 0 )
                    {
                      /* set A term due to this sample point */
                      a_menu[me][map[j1][k1]] = ufuns[jj] * vfuns[kk];
                    }
                  else /* add this sample point's effect to the rhs vector */ 
                    {
                      N_CPtToPtEuclid( Sw[j1][k1], &P1 );
                      N_VectorBlendPt( -(ufuns[jj] * vfuns[kk]), P1, &rhs[me] );
                    }
                } /* end iter kk, ever nonZero v basis function for this Sample point */
            } /* end iter jj, ever nonZero u basis function for this Sample point  */
        } /* end iter ii, every Sample Point */
    } /* end asked for full matrix branch */
  else /* asked for sparse matrix */
    { 
      /* use sparse storage scheme */
      a_pq = N_AllocReal2dArray( np, p + q + 1, &SL );

      if( a_pq EQ NULL )
          NL_QUIT;

      /* sur span index for every sample point */
      uvsp = N_AllocInt2dArray( np, 1, &SL );

      if( uvsp EQ NULL )
          NL_QUIT;

      glo_a    = a_pq;
      glo_cind = cind;
      glo_uvsp = uvsp;

      /* set up system */

      /* for every sample Point */
      for ( ii = 0; ii <= np; ii++ )
        {
          /* skip points outside the sur U interval */
          if( uu[ii]LT u1 OR uu[ii]GT u2 )
              continue;

          /* skip points outside the sur V interval */
          if( vv[ii]LT v1 OR vv[ii]GT v2 )
              continue;

          /* when given point weights */
          if( wts NEQ NULL )
            {
              /* skip nonPositive weighted points */
              if( wts[ii]LE 0.0 )
                  continue;
              else /* set w with this point's sqrt(weight value) */
                {
                  if( wts[ii]EQ 1.0 )
                      w = 1.0;
                  else
                      w = sqrt( wts[ii] );
                }
            } /* end given point weights check */

          /* count the number of sample points used */
          me += 1;

          /* when uu[ii] is different than last uu value */
          if( uu[ii]NEQ u0 )
            {
              u0 = uu[ii];
              
              /* get nonVanishing U basis values */
              error = N_BasisEval( knu, p, uu[ii], NL_LEFT, &a_pq[me][0], &uspan );

              if( error EQ NL_YES )
                  NL_OUT;
            } /* end uu[ii] is new check */
          else 
            {
              /* just copy last nonVnaishing U basis values */
              for ( jj = 0; jj <= p; jj++ )
                  a_pq[me][jj] = a_pq[me - 1][jj];
            }

          /* when vv[ii] is different than last vv value */
          if( vv[ii]NEQ v0 )
            {
              v0 = vv[ii];
              
              /* get nonVanishing V basis values */
              error = N_BasisEval( knv, q, vv[ii], NL_LEFT, &a_pq[me][p + 1], &vspan );

              if( error EQ NL_YES )
                  NL_OUT;
            } /* end vv[ii] is new check */
          else
            {
              /* just copy last nonVnaishing U basis values */
              for ( jj = 0; jj <= q; jj++ )
                  a_pq[me][jj + p + 1] = a_pq[me - 1][jj + p + 1];
            }

          /* save Sample Point span indices */
          uvsp[me][0] = uspan;
          uvsp[me][1] = vspan;

          /* copy Point value into rhs */
          N_CopyPt( P[ii], &rhs[me] );

          /* when given nonUnit weight */
          if( w NEQ 1.0 )
            {
              
              u0 = u1 - 1.0; /* remember that a_pq[me][0] changed */

              /* apply the weight */
              for ( jj = 0; jj <= p; jj++ )
                  a_pq[me][jj] *= w;
              N_ScalePt( w, rhs[me], &rhs[me] );
            }

          /* for every nonZero u basis function for this sample point */
          for ( jj = 0; jj <= p; jj++ )
            {
              /* U effected Control point index */
              j1 = uspan - p + jj;

              /* for every nonZero v basis function for this sample point */
              for ( kk = 0; kk <= q; kk++ )
                {
                  /* V effected control point index */
                  k1 = vspan - q + kk;

                  /* when effected controlPoint is constrained */
                  if( map[j1][k1]LT 0 )
                    {
                      /* subtract its effect from the rhs vector */
                      N_CPtToPtEuclid( Sw[j1][k1], &P1 );
                      N_VectorBlendPt( -(a_pq[me][jj] * a_pq[me][p + kk + 1]), P1, &rhs[me] );
                    }
                } /* end iter kk, ever nonZero v basis function for this Sample point */
            } /* end iter jj, ever nonZero u basis function for this Sample point  */
        } /* end iter every ii, every sample Point */
    } /* end asked for sparse matrix branch */

  /* when using SVD solver make sure that there are at least as many sample equations as dofs         */
  /*   else switch to LUPIV solver.  GWC:Bug151                                                       */                        
  if( aflg EQ NL_SVD AND me LE nu )
    { 
      /* switch to the LUPIV solver - it can handle subsampling */
      aflg = NL_LUPIV ; 
    }

  glo_me = me;
  glo_nu = nu;
  glo_p = p;
  glo_q = q;

    /*  Set up the periodic constraints  */
  /* The following works with constrained boundaries */
  /* It does not work if the control points adjacent to the boundaries are constrained */
  /* Assumption: If periodic, then the relevant boundary curves are compatible. That is, both */
  /* are null or both are defined by the same NURBS */

  ii  = NL_MAX( p, q );
  ND  = N_AllocReal2dArray( ii, ii, &SL );
  ND2 = N_AllocReal2dArray( ii, ii, &SL );

  if ( ND2 EQ NULL )
      NL_QUIT;

  if ( uper EQ NL_YES )
  {
      if ( bdys EQ NULL || loc_bdys[1] EQ NULL )
          mc += p * ( m + 1 );
      else
          mc += ( p - 1 ) * ( m + 1 );

      if ( bdys NEQ NULL )
      {
          if ( loc_bdys[0] NEQ NULL )
              mc -= p - 1;
          if ( loc_bdys[2] NEQ NULL )
              mc -= p - 1;
      }
  }

  if ( vper EQ NL_YES )
  {
      if ( bdys EQ NULL || loc_bdys[0] EQ NULL )
          mc += q * ( n + 1 );
      else
          mc += ( q - 1 ) * ( n + 1 );

      if ( bdys NEQ NULL )
      {
          if ( loc_bdys[1] NEQ NULL )
              mc -= q - 1;
          if ( loc_bdys[3] NEQ NULL )
              mc -= q - 1;
      }

#if 0
      if ( mc LT 0 )
      {
          mc += ( q - 1 ) * ( n + 1 );
          if ( bdys NEQ NULL )
          {
              if ( loc_bdys[1] NEQ NULL )
                  mc -= q - 1;
              if ( loc_bdys[3] NEQ NULL )
                  mc -= q - 1;
          }
      }
      else // uper == vper == NL_YES. In corners, only periodic in U
          mc += ( q - 1 ) * ( n + 1 - 2 * p );

      /* how to rewrite to enforce periodicity in both directions in corners ? */
      /* Do same example with more cpts ? Might make it clearer what's going on. */
#endif

  }

  error = N_SetRealMatrix( &lma, mc, nu, NL_MT_FULL, nu, &SL );

  if ( error EQ NL_YES )
      NL_OUT;

  G = N_AllocPt1dArray( mc, &SL );
  for ( ii = 0; ii <= mc; ii++ )
  { N_CopyPt( NL_ZERO, &G[ii] ); }

  N_GetRealMatrixPtr( &lma, &MM );

  for ( ii = 0; ii <= mc; ii++ )
      for ( jj = 0; jj <= nu; jj++ )
          MM[ii][jj] = 0.0;

  mc = -1;

  if ( uper EQ NL_YES )
  {
      error = N_BasisDerivs( knu, p, u1, NL_LEFT, p - 1, ND, &uspan );

      if ( error EQ NL_YES )
          NL_OUT;
      error = N_BasisDerivs( knu, p, u2, NL_RIGHT, p - 1, ND2, &uspan2 );

      if ( error EQ NL_YES )
          NL_OUT;

      NL_INDEX iStart = 0;
      NL_INDEX iEnd = m;
      NL_INDEX jStart = 0;
      if ( bdys NEQ NULL )
      {
          if ( loc_bdys[0] NEQ NULL )
              ++iStart;
          if ( loc_bdys[2] NEQ NULL )
              --iEnd;
          if ( loc_bdys[1] NEQ NULL )
              ++jStart;
      }

      for ( ii = iStart; ii <= iEnd; ii++ )
      {
          for ( jj = jStart; jj < p; jj++ )
          {
              mc += 1;

              for ( kk = 0; kk <= jj; kk++ )
              {
                  /* Boundaries defined compatibly by assumption (e.g., map[0][ii] == map[n][ii] when constrained)*/
                  if ( map[kk][ii] == -1 )
                  {
                      N_CPtToPt( Sw[kk][ii], &P1 );
                      N_CPtToPt( Sw[n - kk][ii], &P2 );
                      N_VectorBlendPt( -ND[jj][kk], P1, &G[mc] );
                      N_VectorBlendPt( ND2[jj][p - kk], P2, &G[mc] );
                  }
                  else
                  {
                      MM[mc][map[kk][ii]] = ND[jj][kk];
                      MM[mc][map[n - kk][ii]] = -ND2[jj][p - kk];
                  }
              }
          }
      }
  }

  if ( vper EQ NL_YES )
  {
      error = N_BasisDerivs( knv, q, v1, NL_LEFT, q - 1, ND, &vspan );

      if ( error EQ NL_YES )
          NL_OUT;
      error = N_BasisDerivs( knv, q, v2, NL_RIGHT, q - 1, ND2, &vspan2 );

      if ( error EQ NL_YES )
          NL_OUT;

      NL_INDEX iStart = 0;
      NL_INDEX iEnd = n;
      NL_INDEX jStart = 0;

      if ( bdys NEQ NULL )
      {
          if ( loc_bdys[1] NEQ NULL )
              ++iStart;
          if ( loc_bdys[3] NEQ NULL )
              --iEnd;
          if ( loc_bdys[0] NEQ NULL )
              ++jStart;
      }

#if 0
      /* why does(k1 = n - p) when uper&& vper ?    */
      /* I think the following lines should be removed... */
      if ( uper EQ NL_YES )
      {
          iStart += p - 1;
          iEnd   -= p - 1;
      }
#endif

      for ( ii = iStart; ii <= iEnd; ii++ )
      {
          for ( jj = jStart; jj < q; jj++ )
          {
              mc += 1;

              for ( kk = 0; kk <= jj; kk++ )
              {
                  /* Boundaries defined compatibly by assumption (e.g., map[0][ii] == map[n][ii] when constrained)*/
                  if ( map[ii][kk] == -1 )
                  {
                      N_CPtToPt( Sw[ii][kk]    , &P1 );
                      N_CPtToPt( Sw[ii][m - kk], &P2 );
                      N_VectorBlendPt( -ND[jj][kk],     P1, &G[mc] );
                      N_VectorBlendPt( ND2[jj][q - kk], P2, &G[mc] );
                  }
                  else
                  {
                      MM[mc][map[ii][kk]]     =  ND [jj][kk];
                      MM[mc][map[ii][m - kk]] = -ND2[jj][q - kk];
                  }
              }
          }
      }
  }

  /* Now solve the system, either internally or externally */

  /* next build least squares equation set */
  /* for solver = NL_LUPIV                 */
  /*    a_nunu * Qw = rhs'                 */
  /*    a_nunu = a_menu_transpose * a_menu */
  /*    rhs'   = a_menu * rhs              */
  /* for solver = NL_SVD                   */
  /*    apply SVD directly to eqn          */
  /*    a_menu * Sw = rhs                  */

  /* when asked - solve internally */
  if( iflg EQ NL_YES )
    { 
      /* allocate A matrix */
      a_nunu = N_AllocReal2dArray( nu, nu, &SL );
      if( a_nunu EQ NULL )
          NL_QUIT;

      /* when asked solve with Singular Value Decomposition */ 
      if( aflg EQ NL_SVD )
        { 
          
          svd_w = N_AllocReal1dArray( nu, &SL );
          if( svd_w EQ NULL )
              NL_QUIT;

          /* decompose A into = U * w * T(V) where T(V) = transpose(V) */
          /*   changes a_menu into U */
          error = N_RealMatrixSingleValueDecompose( a_menu, svd_w, a_nunu, me, nu );
          if( error EQ NL_YES )
              NL_OUT;

          N_AddStiffnessToSrfAMatrix(alpha, beta, sur, a_nunu, map, Sw, rhs, &nu) ; 

          /* back substitute the solution */
          error = N_SingleValueDecomposeSolve( a_menu, svd_w, a_nunu, me, nu, (NL_VOID *)rhs, NL_EPOINT, NL_YES, (NL_VOID *)Qw );
          if( error EQ NL_YES )
              NL_OUT;
        }

      /* when asked solve with LU Decomposition */ 
      if( aflg EQ NL_LUPIV )
        { /* Solve via Normal Equations and LU Decomposition */
          /* Set up Normal Equations and rhs */

          /* add stiffness terms to add to A matrix */
          /* alpha = 1.0; */
          /* beta = 10.0;  */
          /* PtStiffness = 1.0E10; */

          /* */
          nrhs = N_AllocPt1dArray( nu, &SL );
          if( nrhs EQ NULL )
              NL_QUIT;

          /* for FULL memory scheme */
          if( mflg EQ NL_FULL )
            {
              /* for every dof */
              for ( ii = 0; ii <= nu; ii++ )
                {
                  N_CopyPt( NL_ZERO, &nrhs[ii] );

                  /* nrhs = a_menu_transpose * rhs */
                  for ( jj = 0; jj <= me; jj++ )
                      N_VectorBlendPt( PtStiffness * a_menu[jj][ii], rhs[jj], &nrhs[ii] );

                  /* build A_nunu = a_menu_transpose * a_menu (upper triangle) */
                  for ( jj = ii; jj <= nu; jj++ )
                    {
                      dd = 0.0;

                      for ( kk = 0; kk <= me; kk++ )
                          dd += PtStiffness * a_menu[kk][ii] * a_menu[kk][jj];
                      a_nunu[ii][jj] = dd;
                    }

                  /* build A_nunu (lower triangle */
                  for ( jj = ii + 1; jj <= nu; jj++ )
                      a_nunu[jj][ii] = a_nunu[ii][jj];
                } /* end iter ii, every dof */
            } /* end full memory scheme branch */
          else /* use sparse memory scheme */
            {

             /* build A and B matrices */

             /* init A and B to zero - these clears are tuned to memory layout done */
             /* in  N_AllocPt1dArray() and N_AllocReal2dArray()                     */
             N_MemSet(nrhs,0,(nu + 1) * sizeof( NL_POINT )) ;
             N_MemSet(a_nunu[0], 0, (nu + 1) * (nu + 1) * sizeof( NL_REAL )  );

             /* for every data point */
             for(kk=0;kk<=me;kk++)
               {
                 /* u and v basis values for kkth data point */
                 pBu = a_pq[kk] ;
                 pBv = pBu + p + 1 ;

                 /* u and v spans for the kkth data point    */
                 USpan = uvsp[kk][0] ;
                 VSpan = uvsp[kk][1] ;

                 /* smallest global dof number for USpan/VSpan element */
                 NL_INDEX iStart = USpan - p;
                 NL_INDEX jStart = VSpan - q;
                 NL_INDEX iii, jjj, lll, kkk;

                 /* for every V Basis function */
                 for(j0=0;j0<=q;j0++)
                   {
                     /* map i0,j0 to global mm index */
                     jjj = jStart + j0;

                     /* for every U Basis function */
                     for(i0=0;i0<=p;i0++, mm++)
                       {
                         iii = iStart + i0;
                         mm = map[iii][jjj];
                         if ( mm == -1 )
                         { continue; }

                         /* Basis function B_m */
                         dBm = PtStiffness * pBu[i0] * pBv[j0] ; /* use this line when using alpha, beta, and PtStiffness */
                         /* dBm = pBu[i0] * pBv[j0] ;  */

                         /* add terms to B matrix B[mm] += rhs[kk] * B_m(u_k,v_k) */
                         N_VectorBlendPt(dBm , rhs[kk], &nrhs[mm] );

                         /* add B_m(uk,vk)*B_l(uk,vk) terms to A[m,l] matrix elements */
                         for(j1=0;j1<=q;j1++)
                           {
                             /* map i1,j1 to global ll index */
                             lll = jStart + j1;

                             for(i1=0;i1<=p;i1++)
                               {
                                 /* add terms to A matrix */
                                 kkk = iStart + i1;
                                 ll = map[kkk][lll];

                                 if ( ll == -1 )
                                 { continue; }

                                 a_nunu[mm][ll] +=  dBm * pBu[i1] * pBv[j1] ;
                      
                               } /* end iter every U Basis function */
                           } /* end iter every V Basis function - making A terms */

                       } /* end iter every U Basis function */
                   } /* end iter every V Basis function - making B and A terms */
               } /* end iter every pos, u-deriv and v-deriv input point */

              /*      allocate memory for columns of the coeff matrix                  */
              /*                                                                       */
              /*      col1 = N_AllocReal1dArray( me, &SL );                            */
              /*                                                                       */
              /*      if( col1 EQ NULL )                                               */
              /*          NL_QUIT;                                                     */
              /*      col2 = N_AllocReal1dArray( me, &SL );                            */
              /*                                                                       */
              /*      if( col2 EQ NULL )                                               */
              /*          NL_QUIT;                                                     */
              /*                                                                       */
              /*      for ( ii = 0; ii <= nu; ii++ )                                   */
              /*      {                                                                */
              /*          error = ST_GetRowsColMatrix1( -1, ii, col1 );                */
              /*                                                                       */
              /*          if( error EQ NL_YES )                                        */
              /*              NL_OUT;                                                  */
              /*                                                                       */
              /*          N_CopyPt( NL_ZERO, &nrhs[ii] );                              */
              /*                                                                       */
              /*          for ( jj = 0; jj <= me; jj++ )                               */
              /*              N_VectorBlendPt( col1[jj], rhs[jj], &nrhs[ii] );         */
              /*                                                                       */
              /*          for ( jj = ii; jj <= nu; jj++ )                              */
              /*          {                                                            */
              /*              error = ST_GetRowsColMatrix1( -1, jj, col2 );            */
              /*                                                                       */
              /*              if( error EQ NL_YES )                                    */
              /*                  NL_OUT;                                              */
              /*                                                                       */
              /*              dd = 0.0;                                                */
              /*                                                                       */
              /*              for ( kk = 0; kk <= me; kk++ )                           */
              /*                  dd += col1[kk] * col2[kk];                           */
              /*              a_nunu[ii][jj] = dd;                                     */
              /*          }                                                            */
              /*                                                                       */
              /*          for ( jj = ii + 1; jj <= nu; jj++ )                          */
              /*              a_nunu[jj][ii] = a_nunu[ii][jj];                         */
              /*      }                                                                */
            } /* end sparse memory scheme branch */

          /* stablize computation when sample points don't cover whole domain    */
          /* add stiffness terms to A matrix                                     */
          /*      alpha       = 1.0 ;                                            */
          /*      beta        = 10.0 ;                                           */
          /*      PtStiffness = 1.0E10 ; (this term weights the constraint eqns) */

          /* Mapping for A_nunu_index->Global_Dof_index                                      */
          /* 1. map : ControlPoint(i,j)->DofIndex[k]  and                                    */
          /*    cind: DofIndex[k,0]->ControlPoint(i)                                         */
          /*          DofIndex[k,1]->ControlPoint(j)                                         */
          /* 2. Sw (outputSur ControlPoint array) set with all constrained bdry curve values */

          /* bug 151 - a_nunu is set up to the A matrix after the rows and columns         */
          /*           of the constrained ControlPoints have been removed from the general */ 
          /*           A stiffness matrix.  For Bug151, additional arguments,              */
          /*           map, Sw, nrhs, &nu, were added to call N_AddStiffnessToSrfAMatrix() */
          /*           and that function was modified to work with a reduced A matrix.     */
          N_AddStiffnessToSrfAMatrix(alpha, beta, sur, a_nunu, map, Sw, nrhs, &nu) ; 

          /* Now solve via LU decomposition (Crout with partial piv) */

          perm = N_AllocInt1dArray( nu, &SL );

          if( perm EQ NULL )
              NL_QUIT;

          N_CreateRealMatrix( &rma, nu, nu, a_nunu, NL_MT_FULL, nu );

          NL_RMATRIX mt, mit, mi, imit, intwn;
          NL_POINT * INTWS, * VV, *UU, *A, B, C;

          N_InitRealMatrix( &intwn );
          error = N_RealMatrixInversePivot( &rma, &intwn, &SL );

          if ( error EQ NL_YES )
              NL_OUT;

          N_InitRealMatrix( &mt );
          error = N_RealMatrixTranspose( &lma, &mt, &SL, &SL );

          if ( error EQ NL_YES )
              NL_OUT;

          N_InitRealMatrix( &mit );
          error = N_RealMatrixMultiply( &intwn, &mt, &mit, &SL );

          if ( error EQ NL_YES )
              NL_OUT;

          N_InitRealMatrix( &mi );
          error = N_RealMatrixMultiply( &lma, &mit, &mi, &SL );

          if ( error EQ NL_YES )
              NL_OUT;

          N_InitRealMatrix( &imit );
          error = N_RealMatrixInversePivot( &mi, &imit, &SL );

          if ( error EQ NL_YES )
              NL_OUT;

          /*  Now solve  */

          INTWS = N_AllocPt1dArray( nu, &SL );

          if ( INTWS EQ NULL )
              NL_QUIT;

          VV = N_AllocPt1dArray( nu, &SL );

          if ( VV EQ NULL )
              NL_QUIT;

          UU = N_AllocPt1dArray( nu, &SL );

          if ( UU EQ NULL )
              NL_QUIT;

          A = N_AllocPt1dArray( mc, &SL );

          if ( A EQ NULL )
              NL_QUIT;

          error = N_RealMatrixMultiplyPtArray( &intwn, nrhs, INTWS );

          if ( error EQ NL_YES )
              NL_OUT;

          error = N_RealMatrixMultiplyPtArray( &lma, INTWS, VV );

          if ( error EQ NL_YES )
              NL_OUT;

          error = N_RealMatrixMultiplyPtArray( &imit, VV, A );

          if ( error EQ NL_YES )
              NL_OUT;

          error = N_RealMatrixMultiplyPtArray( &mit, A, VV );

          if ( error EQ NL_YES )
              NL_OUT;

          /* Solve for constraints on periodicity */
          error = N_RealMatrixMultiplyPtArray( &imit, G, A );

          if ( error EQ NL_YES )
              NL_OUT;

          error = N_RealMatrixMultiplyPtArray( &mit, A, UU );

          if ( error EQ NL_YES )
              NL_OUT;

          for ( ii = 0; ii <= nu; ii++ )
          {
              N_Diff2Pts( INTWS[ii], VV[ii], &B );
              N_Sum2Pts( B, UU[ii], &C );
              N_PtToCPt( C, &Qw[ii] );
          }

          /* error = N_RealMatrixLuDecomposePivot( &rma, perm ); */

          /* if( error EQ NL_YES )  */
          /*    NL_OUT;             */

          /* error = N_RealMatrixRightForBackPivot(&rma, perm, nrhs, Qw); */

          /* if( error EQ NL_YES )
              NL_OUT;  */
        }
    } /* end solve internal branch */
  else
  { /* solve via external method */
    /* gwc: Bad practice here - passing ptrs as ints:                   */
    /*      ptrs are being cast to ints so that two different           */
    /*      user-supplied function signatures can be passed to this     */
    /*      function through a single argument to support user-supplied */
    /*      solver functions for full and sparse matrices.              */
    /*      In better practice - the two different methods should be    */
    /*        two different input arguments to this method with proper  */
    /*        typing of their call arguments.                           */
    /*      For backward compatibility - leave it alone.                */
    /*        some user must have asked for this support and            */
    /*        is still using it.  Odds are other users won't            */
    /*        be using this feature.                                    */
	  if (mflg EQ NL_FULL) 
     {
		     /* Convert from pointer to NL_INDEX (long)
		        For 32 bit this is even
		        For 64 bit a pointer is twice the size of long
		        NL_INDEX* tmp = (int *)(((char *)a_menu) + 2); */       
		     error = fsol((NL_LLONG)a_menu,     /* in : ptr to A matrix of A X= B matrix equation  */
                    me,                   /* in : m of ASize:[m,n]                           */
                    nu,                   /* in : n of ASize:[m,n]                           */
                    rhs,                  /* in : ptr to B matrix of A X = B matrix equation */
                    Qw);                  /* out: ptr to X matrix of A X = B matrix equation */
	    }
	  else /* sparse matrix branch */
     {
       /* gwc: I don't see how this branch can work as written  */
       /*      The user callback arguments do not contain a ptr */
       /*      to the A Matrix which is needed to make the      */
       /*      ST_GetRowsColMatrix1() call work                 */
       /*      the call seems to be missing a ptr to A argument */
		     /* Convert from NL_FLAG (short) to NL_INDEX (long)
		       For 32 bit this is even
		       For 64 bit a short is half the size of long
		       NL_INDEX* tmp = (int *)(((char *)ST_GetRowsColMatrix1) + 2); */
		      error = fsol(me,                              /* in : m of ASize:[m,n]                            */
                     nu,                              /* in : n of ASize:[m,n]                            */
                     (NL_LLONG)ST_GetRowsColMatrix1,  /* in : ptr to fnc that rtns ptrs to A[i][j] vals   */
                     rhs,                             /* in : ptr to B matrix of A X = B matrix equation  */
                     Qw);                             /* out: ptr to X matrix of A X = B matrix equation  */
	    }

      if( error EQ NL_YES )
          NL_ERROR( NL_CAL_ERR );
    
    } /* end solve external branch */

  /* arrive here after solving */
  /* next move solution back into outputSur control point array */

  /* for every u control point */
  for ( ii = 0; ii <= n; ii++ )
    {
      /* for every v control point */
      for ( jj = 0; jj <= m; jj++ )
        {
          /* copy unconstrained dof values */
          if( map[ii][jj]GE 0 )
              N_CopyCPt( Qw[map[ii][jj]], &Sw[ii][jj] );

        } /* end iter every v control point */
    } /* end iter every u control point - seting output control pout values */

  /* End NURBS and Exit */

  EXIT:

  N_EndNurbs( &SL );

  /* all done */
  return (error);

} /* end N_FitSrfLstSqDeformablePeriodic  */

#if NLIB_UNUSED

#if 0  /* UNDER_CONSTRUCTION */
/************************************************************************************************/
/* N_FitSrfLstSqDerivsWithConstraints: Least squares surface approximation to random points     */
/************************************************************************************************/

/*******************************************************************//**
   NOTE: THIS FUNCTION IS UNDER CONSTRUCTION, NOT YET COMPLETED.
 
   DESCRIPTION:
 
     This fitting  routine  computes  a  least squares b-spline  surface 
     approximation to a random (non-NxM) set of  points.  First  partial
     derivative vectors with respect to u or v may (optionally) be input
     at each point.  When the number of sample points is large the
     algorithm can not interpolate every input point, it will find the
     best approximation to those points.
     
     If  the  output  surface is initialized to the NULL 
     surface,  memory is allocated locally.  Otherwise  it is checked if  
     enough memory is passed in. A typical calling example is:
 
       NL_FLAG        bdryFlag;
       NL_POINT      *P, *Q;
       NL_VECTOR     *DU, *DV;
       NL_PARAMETER  *uu, *vv, *qu, *qv;
       NL_REAL       *wts, *uwts, *vwts
       NL_INDEX      m, n, np, nq, nu, nv, *Iu, *Iv;
       NL_DEGREE     p, q;
       NL_SURFACE    sur;
       NL_STACKS     SG;

       ...
       N_SrfInitArrays(&sur);
       N_FitSrfLstSqDerivs(bdryFlag, P,wts,uu,vv,np,DU,uwts,nu,Iu,DV,
                vwts,nv,Iv,Q,qu,qv,nq,
                n,m,p,q,U,V,&sur,&SG);
 
 
   ACCESS:
   
   bdryFlag,input  ,  orof 0 = no boundary constraints              
                           1 = periodic in U                        
                           2 = U Min iso parameter curve is singular
                           4 = U Max iso parameter curve is singular
                           8 = periodic in V                        
                          16 = V Min iso parameter curve is singular
                          32 = V Max iso parameter curve is singular
     P    , input  ,  Points to be approximated
     wts  , input  ,  Least squares point weights (wts[i] >= 0  for  all
                      i).  The  larger  wts[i],  the  closer the surface
                      comes to P[i]. wts[i] < 1 lessens the influence of
                      P[i]. If wts[i] = 0, then P[i] is not used.  If no
                      weighting is desired, set wts = NULL
     uu   , input  ,  The u-parameters of the points in P
     vv   , input  ,  The v-parameters of the points in P
     np   , input  ,  The high index of the arrays: P,wts,uu,vv
     DU   , input  ,  First derivative vectors wrt u (optional). DU=NULL
                      means no u-derivatives specified
     uwts , input  ,  u-derivative weights (uwts[i] >= 0 for all i). The
                      larger uwts[i],  the  closer  the surface comes to 
                      assuming DU[i].  uwts[i] < 1 lessens the influence 
                      of DU[i].  If uwts[i] = 0, then DU[i] is not used.  
                      If no weighting is desired, set uwts = NULL
     nu   , input  ,  The high index of the arrays: DU, uwts, Iu (set to
                      -1 if no u-derivatives specified)
     Iu   , input  ,  DU[i] is the u-derivative at (uu[Iu[i]],vv[Iu[i]])
     DV   , input  ,  First derivative vectors wrt v (optional). DV=NULL
                      means no v-derivatives specified
     vwts , input  ,  v-derivative weights (vwts[i] >= 0 for all i). The
                      larger vwts[i],  the  closer  the surface comes to 
                      assuming DV[i].  vwts[i] < 1 lessens the influence 
                      of DV[i].  If vwts[i] = 0, then DV[i] is not used.  
                      If no weighting is desired, set vwts = NULL
     nv   , input  ,  The high index of the arrays: DV, vwts, Iv (set to
                      -1 if no v-derivatives specified)
     Iv   , input  ,  DV[i] is the v-derivative at (uu[Iv[i]],vv[Iv[i]])
     Q    , input  ,  Points to be interpolated
     qu   , input  ,  The u-parameters of the points in Q
     qv   , input  ,  The v-parameters of the points in Q
     nq   , input  ,  The high index of the arrays: Q,qu,qv
     n,m  , input  ,  High indexes of the surface  control  points  (the
                      surface will have (n+1)x(m+1) control points).  
     p,q  , input  ,  Degrees of the surface  
     U,V  , input  ,  Knots for the surface.  If  U=NULL or V=NULL,  the
                      corresponding knots are  computed in this routine.
                      If given, there must be n+p+2 u-knots and/or m+q+2
                      v-knots.  The values in the uu and vv arrays  must
                      correspond to the knot ranges (unless wts[i] = 0)
     sur  , output ,  Approximating surface
     SG   , input  ,  sur's memory stack

 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR

   NOTE:
     The surface domain is extended a small amount past the sample points.
     This resolves problems we've found when a surface constructed from
     sample points fails to span those sample points completely due to
     small tolerance errors.  Adding an extra bit of surface to the perimeter
     enables subsequent operations, like projection of a curve onto a surface
     known to lie on the boundary of this surface to succeed.  If you want
     exact boundaries on your surface, you must trim the returned BSpline
     to a UVTrimCurve constructed by projecting a 3d curve onto this surface.
     
     The surface is not extended beyond boundaries which are periodic or
     singular. 

    Sample code including SMLib routines:
    - compute UV values 

    N_FitSrfCalcParams(P, m, NL_NO, Origin, X, Y, Z, 0.0, 1.0, 0.0, 1.0, uu, vv);
    
    - Fit surface to (xyz uv) points
    error = N_FitSrfLstSqDerivs(P, NULL, uu, vv, m, NULL, NULL, -1, NULL, NULL, NULL, 
            -1, NULL, 4, 4, 3, 3, NULL, NULL, &sur, &S);

    SmBSplineSurface *pSrf = new(sContext) SmBSplineSurface(&sur);   

   ***********************************************************************/

NL_FLAG N_FitSrfLstSqDerivsWithConstraints
  (NL_FLAG    bdryFlag, /* in : orof 0 = no boundary constraints                      */
                        /*           1 = periodic in U                                */
                        /*           2 = U Min iso parameter curve is singular        */
                        /*           4 = U Max iso parameter curve is singular        */
                        /*           8 = periodic in V                                */
                        /*          16 = V Min iso parameter curve is singular        */
                        /*          32 = V Max iso parameter curve is singular        */
   NL_POINT   *P,       /* in : Points to be approximated,              sized:[np+1]  */ 
   NL_REAL    *wts,     /* in : Associated Least Squares Point weights, sized:[np+1]  */ 
   NL_REAL    *uu,      /* in : The u-parameters of the points in P,    sized:[np+1]  */ 
   NL_REAL    *vv,      /* in : The v-parameters of the points in P,    sized:[np+1]  */ 
   NL_INDEX    np,      /* in : size of matrices P, wts, uu, and vv                   */ 
   NL_VECTOR  *DU,      /* in : First derivative vectors wrt u (optional),     NULL to ignore, sized:[nu+1] */ 
   NL_REAL    *uwts,    /* in : Associated Least Squares u-derivative weights, NULL to ignore, sized:[nu+1] */ 
   NL_INDEX    nu,      /* in : size parameter                                        */ 
   NL_INDEX    *Iu,     /* in : DU and uwts index map,                  sized;[nu+1]  */
                        /*        e.g. DU[i] is u-derivative at (uu[Iu[i]],vv[Iu[i]]) */ 
   NL_VECTOR  *DV,      /* in : First derivative vectors wrt v (optional),     NULL to ignore, sized:[nv+1] */ 
   NL_REAL    *vwts,    /* in : Associated Least Squares v-derivative weights, NULL to ignore, sized:[nv+1] */ 
   NL_INDEX    nv,      /* in : size parameter                                        */                       
   NL_INDEX   *Iv,      /* in : DV and vwts index map,                  sized;[nv+1]  */                      
                        /*        e.g. DV[i] is v-derivative at (uu[Iv[i]],vv[Iv[i]]) */                      
   NL_POINT   *Q,       /* in : Points to be constrained,               sized:[nq+1]  */
   NL_REAL    *qu,      /* in : The u-parameters of the points in Q,    sized:[nq+1]  */ 
   NL_REAL    *qv,      /* in : The v-parameters of the points in Q,    sized:[nq+1]  */ 
   NL_INDEX    nq,      /* in : size of matrices Q, qu, and qv                        */ 
   NL_INDEX    n,       /* in : Output Surface U ControlPoint count = (n+1)x(m+1)     */ 
   NL_INDEX    m,       /* in : Output Surface V ControlPoint count = (n+1)x(m+1)     */ 
   NL_DEGREE   p,       /* in : Output Surface U Degree                               */ 
   NL_DEGREE   q,       /* in : Output Surface V Degree                               */ 
   NL_REAL    *U,       /* in : Optional Output Surface U Knot Vector, if given must have n+p+2 u-knots, Null to ignore */ 
   NL_REAL    *V,       /* in : Optional Output Surface V KNot Vector, if given must have m+q+2 v-knots, Null to ignore */ 
   NL_SURFACE *sur,     /* out: Approximating Surface */ 
   NL_STACKS  *SG )     /* in : sur's memory stack    */ 
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfLstSqDerivsWithConstraints");

    NL_FLAG error = NL_NO, pflg ;

    NL_INDEX ii, jj, kk, ** map, me, nuk, uspan, vspan, j1, k1, *perm, ** uvsp, ** cind, i2;

    NL_REAL *US, *VS, ** a_nunu, w, u1, u2, v1, v2, ** a_pq, u0, v0, ** ND;
    
    /* For change temp test only                  */
    /*      NL_REAL *col1, *col2, dd, dTestMax ;  */
    /*      NL_REAL **a_nunu_test ;               */
    /*      NL_POINT *nrhs_test ;                 */
    /*      NL_INDEX lTestCnt ;                   */

    NL_INDEX i0, j0, i1, ii0, jj0, ii1, jj1, USpan, VSpan, iStart, jStart, mm, ll ;

    NL_REAL *pBu, *pBv, dBm, scl, off ; 

    NL_CPOINT *Qw, ** Sw;

    NL_POINT P1, *rhs, *nrhs;

    NL_KNOTVECTOR knt, *knu, *knv, *knus, *knvs;

    NL_RMATRIX rma;

    NL_BOOLEAN bUpper, bLower ;

    /* these A matrix gain terms are kind of arbitrary and can be tuned by experiment */
    NL_REAL alpha, beta, PtStiffness ;

    NL_STACKS SL;

    /* NOTE: THIS FUNCTION IS UNDER CONSTRUCTION, NOT YET COMPLETED. */
    if(P || Q) return(NL_YES) ;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for input errors */

    if(    n LT 1                          /* no U Control Points */
        OR m LT 1)                          /* no V Control Points */
        /* OR np + 1 LT( n + 1 ) * (m + 1) )  - fewer Points than ControlPoints */ 
        NL_ERROR( NL_INP_ERR );

    if(    n LT p                          /* fewer output ControlPoints than degree+1 in U dir */
        OR m LT q )                        /* fewer output ControlPoints than degree+1 in V dir */
        NL_ERROR( NL_INP_ERR );

    /* Check validity of option input U Knotvector */
    if( U NEQ NULL )
    {
        N_KnotVectorFromRealArray( &knt, U, n + p + 1 );
        error = N_KnotVectorIsValid( &knt, p, rname );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Check validity of option input U Knotvector */
    if( V NEQ NULL )
    {
        N_KnotVectorFromRealArray( &knt, V, m + q + 1 );   
        error = N_KnotVectorIsValid( &knt, q, rname );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* remember NO input u-derivative data */
    if( nu LT 0 OR DU EQ NULL )
        nu = -1;

    /* remember NO input v-derivative data */
    if( nv LT 0 OR DV EQ NULL )
        nv = -1;

    /* initialise just to avoid compiler complaint */
    u1 = v1 = 0.0;
    u2 = v2 = 1.0;

    /* Check memory for output surface */
    /* allocate memory when sur arrays are NULL (via N_SrfInitArrays()) */

    error = N_SrfSizeArrays( sur, n, m, p, q, n + p + 1, m + q + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Sw, &US, &VS );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Get the surface u- and v-knots */

    u1 = u2 = 0.0;
    v1 = v2 = 0.0;

    pflg = NL_NO;

    /* when given Opt U KnotVector - set knus as given */
    if( U NEQ NULL )
    {
        kk = n + p + 1;

        for ( ii = 0; ii <= kk; ii++ )
            US[ii] = U[ii];
        knus = knu;
    }
    else /* set knus to NULL and get max and min u values from input uu array */
    {
        knus = NULL;

        /* when given weights */
        if( wts NEQ NULL )
        {
            pflg = NL_YES;
            u1 = 1.0e+20;
            u2 = -u1;

            /* get max and min uu array values */
            for ( ii = 0; ii <= np; ii++ )
                if( wts[ii]NEQ 0.0 )
                {
                    if( u1 GT uu[ii] )
                        u1 = uu[ii];

                    if( u2 LT uu[ii] )
                        u2 = uu[ii];
                }
        }
    }

    /* when given Opt U KnotVector - set knvs as given */
    if( V NEQ NULL )
    {
        kk = m + q + 1;

        for ( ii = 0; ii <= kk; ii++ )
            VS[ii] = V[ii];
        knvs = knv;
    }
    else  /* set knvs to NULL and get max and min v values from input uu array */
    {
        knvs = NULL;

        /* when wts are given */
        if( wts NEQ NULL )
        {
            pflg = NL_YES;
            v1 = 1.0e+20;
            v2 = -v1;

            /* get max and min vv array values */
            for ( ii = 0; ii <= np; ii++ )
                if( wts[ii]NEQ 0.0 )
                {
                    if( v1 GT vv[ii] )
                        v1 = vv[ii];

                    if( v2 LT vv[ii] )
                        v2 = vv[ii];
                }
        }
    }

    /* when surface knot vectors have not been specified */
    if( knus NEQ knu OR knvs NEQ knv )
    {
        /* construct surface knot vectors */
        error = N_FitSrfCalcKnotVectors
          ( uu,       /* in : u parameter values, sized:[nn+1] */                                               
            vv,       /* in : v parameter values, sized:[nn+1] */                                               
            np,       /* in : highest u and v index */                                                          
            p,        /* in : approximating surface degree U */                                                 
            q,        /* in : approximating surface degree V */                                                 
            pflg,     /* in : NL_YES = use us,ue,vs,ve values to set max/min u and v knot values */             
                      /*      NL_NO  = find max/min u and v knot values in u and v arrays        */             
            u1,       /* in : min U param value, overridden by any input knu value   */                         
            u2,       /* in : max U param value, overridden by any input knu value   */                         
            v1,       /* in : min V param value, overridden by any input knv value   */                         
            v2,       /* in : max V param value, overridden by any input knv value   */                         
            knus,     /* in : opt starter U knots, if given these knots will be in the output, NULL to ignore */
            knvs,     /* in : opt starter V knots, if given these knots will be in the output, NULL to ignore */
            knu,      /* i/o: sized knot vector whose knot values are to be determined */                       
            knv );    /* i/o: sized knot vector whose knot values are to be determined */                       

        if( error EQ NL_YES )
            NL_OUT;

        /* scale the knots so that the sample points won't lie exactly on the boundary */
        /* do not move singular or periodic boundaries */

        /* u vec */
        bUpper = ((bdryFlag & 1) || (bdryFlag & 32)) ? NL_FALSE : NL_TRUE ;
        bLower = ((bdryFlag & 1) || (bdryFlag & 16)) ? NL_FALSE : NL_TRUE ; 
        scl = 1.0 + (bUpper ? .0625 : 0.0) + (bLower ? .0625 : 0.0) ;
        off = (knu->U[0] - (bLower ? (knu->U[knu->m]-knu->U[0])*.0625 : 0.0)) - knu->U[0]*scl ;
        if(scl != 1.0) for(ii=0;ii<=knu->m;ii++) { knu->U[ii] = scl * knu->U[ii] + off ; }

        /* v vec */
        bUpper = ((bdryFlag & 8) || (bdryFlag & 2)) ? NL_FALSE : NL_TRUE ;
        bLower = ((bdryFlag & 8) || (bdryFlag & 1)) ? NL_FALSE : NL_TRUE ; 
        scl = 1.0 + (bUpper ? .0625 : 0.0) + (bLower ? .0625 : 0.0) ;
        off = (knu->U[0] - (bLower ? (knu->U[knu->m]-knu->U[0])*.0625 : 0.0)) - knu->U[0]*scl ;
        if(scl != 1.0) for(ii=0;ii<=knv->m;ii++) { knv->U[ii] = scl * knv->U[ii] + off ; }
    }

    /* Allocate memory for the index map */
    map = N_AllocInt2dArray( n, m, &SL );

    if( map EQ NULL )
        NL_QUIT;

    cind = N_AllocInt2dArray( (n + 1) * (m + 1) - 1, 1, &SL );

    if( cind EQ NULL )
        NL_QUIT;

    /* Define the map from Sw(i,j) to svd_a's column index                                */
    /* map[n+1][m+1]             = global index for every output surface control point    */
    /* glo_cind = cind[nuk+1][2] = Control Point ii,jj indices for every global index     */
    /*     cind[kk][0] = ii the control point's ii index for global index kk              */
    /*     cind[kk][1] = jj the control point's jj index for global index kk              */
    /* gwc note: these maps could be replaced by indexing functions as                    */
    /*   map[ii][jj] = (jj*(n+1))+ii ;                                                    */
    /*   cind[kk][0] =  kk-((kk%(n+1))*n+1)                                               */
    /*   cind[kk][1] =  kk%(n+1)                                                          */

    nuk = -1; /* nuk+1 unknowns */

    for ( jj = 0; jj <= m; jj++ )
    {
        for ( ii = 0; ii <= n; ii++ )
        {
            nuk += 1;
            map[ii][jj] = nuk;

            cind[nuk][0] = ii;
            cind[nuk][1] = jj;
        }
    }

    /* Allocate memory for the right hand side and solution vector */

    Qw = N_AllocCPt1dArray( nuk, &SL );

    if( Qw EQ NULL )
        NL_QUIT;

    me = np;
    me += (nu + 1);
    me += (nv + 1);

    rhs = N_AllocPt1dArray( me, &SL );

    if( rhs EQ NULL )
        NL_QUIT;

    /* Now set up the overdetermined system of equations */

    u1 = US[0];
    u2 = US[n + 1];
    v1 = VS[0];
    v2 = VS[m + 1];

    w = 1.0;
    u0 = u1 - 1.0;
    v0 = v1 - 1.0;

    a_pq = N_AllocReal2dArray( me, p + q + 1, &SL );

    if( a_pq EQ NULL )
        NL_QUIT;

    uvsp = N_AllocInt2dArray( me, 1, &SL );

    if( uvsp EQ NULL )
        NL_QUIT;

    me = -1; /* me+1 equations to solve */

    glo_a = a_pq;
    glo_cind = cind;
    glo_uvsp = uvsp;

    /* set up system */

    /* first, the equations for the sample point positions */

    /* for every sample point position - build                                                             */
    /*  glo_a = a_pq[me+1][p+q+2] = basis functions values for sample point uu,vv values                   */
    /*                              a_pq[ii][0:p]       = uu basis values for sample point ii              */
    /*                              a_pq[ii][(p+1)+0:q] = vv basis values for sample point ii              */
    /*  glo_uvsp = uvsp[me+1][2]  = knot spans for sample point uu, vv values                              */
    /*                              uvsp[ii][0] = max u knot index LE to uu value for sample point ii      */
    /*                              uvsp[ii][1] = max v knot index LE to vv value for sample point ii      */
    /*  rhs[me+1]                 = sample point positions                                                 */
    /*                              rhs[ii] = xyz position for sample point ii                             */
    /* when given optional wts, each rhs[ii] and a_pq[ii][0:p] uu basis values                             */
    /*   are multiplied by the given wts[ii] value        */
    /* ignore sample points when                          */
    /*       o. uu value is out of specified bounds       */
    /*       o. vv value is out of specified bounds       */
    /*       0. optional weight is LE 0.0                 */
    for ( ii = 0; ii <= np; ii++ )
    {
        if( uu[ii]LT u1 OR uu[ii]GT u2 )
            continue;

        if( vv[ii]LT v1 OR vv[ii]GT v2 )
            continue;

        if( wts NEQ NULL )
        {
            if( wts[ii]LE 0.0 )
                continue;
            else
            {
                if( wts[ii]EQ 1.0 )
                    w = 1.0;
                else
                    w = sqrt( wts[ii] );
            }
        }

        me += 1;

        if( uu[ii]NEQ u0 )
        {
            u0 = uu[ii];
            error = N_BasisEval( knu, p, uu[ii], NL_LEFT, &a_pq[me][0], &uspan );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] = a_pq[me - 1][jj];
        }

        if( vv[ii]NEQ v0 )
        {
            v0 = vv[ii];
            error = N_BasisEval( knv, q, vv[ii], NL_LEFT, &a_pq[me][p + 1], &vspan );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            for ( jj = 0; jj <= q; jj++ )
                a_pq[me][jj + p + 1] = a_pq[me - 1][jj + p + 1];
        }

        uvsp[me][0] = uspan;
        uvsp[me][1] = vspan;

        N_CopyPt( P[ii], &rhs[me] );

        if( w NEQ 1.0 )
        {
            u0 = u1 - 1.0;

            for ( jj = 0; jj <= p; jj++ )      /* <=== gwc: why isn't this jj <= p+q+1 */ 
                a_pq[me][jj] *= w;             /* because all left side terms are p*q products and this puts just 1 w on the equation left and right */
            N_ScalePt( w, rhs[me], &rhs[me] );
        }

        for ( jj = 0; jj <= p; jj++ )
        {
            j1 = uspan - p + jj;

            for ( kk = 0; kk <= q; kk++ )
            {
                k1 = vspan - q + kk;

                if( map[j1][k1]LT 0 ) /* gwc: I think this is never true */
                {
                    N_CPtToPtEuclid( Sw[j1][k1], &P1 );   /* gwc: I think this is uninitialized when sur allocated in this function */ 
                    N_VectorBlendPt( -(a_pq[me][jj] * a_pq[me][(p + 1) + kk]), P1, &rhs[me] );
                }
            } /* end iter every sur DegreeV value */
        } /* end iter every sur DegreeU value */
    } /* end iter every sample point - building a matrix position equations */

    /* now the equations for the u-derivatives  get appended to same arrays used for positions */
    /* for every sample point u-derivative, build */
    /*  glo_a = a_pq[(np+1)+0:nu][p+q+2] = 1st u deriv and v basis functions values for sample point uu,vv values */
    /*              a_pq[np+1+ii][0:p]       = 1st deriv uu basis values for sample u deriv ii  */
    /*              a_pq[np+1+ii][(p+1)+0:q] = vv basis values for sample point ii              */
    /*  glo_uvsp = uvsp[me+1][2]  = knot spans for sample point u-deriv uu, vv values                      */
    /*                              uvsp[np+1+ii][0] = max u knot index LE to uu value for sample point ii */
    /*                              uvsp[np+1+ii][1] = max v knot index LE to vv value for sample point ii */
    /*  rhs[me+1]                 = sample point u-derivs                                                  */
    /*                              rhs[np+1+ii] = DU vector for sample point u-deriv ii                   */

    ii = NL_MAX( p, q );
    ND = N_AllocReal2dArray( 1, ii, &SL );

    if( ND EQ NULL )
        NL_QUIT;

    w = 1.0;
    u0 = u1 - 1.0;
    v0 = v1 - 1.0;

    for ( ii = 0; ii <= nu; ii++ )
    {
        i2 = Iu[ii];

        if( uu[i2]LT u1 OR uu[i2]GT u2 )
            continue;

        if( vv[i2]LT v1 OR vv[i2]GT v2 )
            continue;

        if( uwts NEQ NULL )
        {
            if( uwts[ii]LE 0.0 )
                continue;
            else
            {
                if( uwts[ii]EQ 1.0 )
                    w = 1.0;
                else
                    w = sqrt( uwts[ii] );
            }
        }

        me += 1;

        if( uu[i2]NEQ u0 )
        {
            u0 = uu[i2];
            error = N_BasisDerivs( knu, p, uu[i2], NL_LEFT, 1, ND, &uspan );

            if( error EQ NL_YES )
                NL_OUT;

            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] = ND[1][jj];
        }
        else
        {
            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] = a_pq[me - 1][jj];
        }

        if( vv[i2]NEQ v0 )
        {
            v0 = vv[i2];
            error = N_BasisEval( knv, q, vv[i2], NL_LEFT, &a_pq[me][p + 1], &vspan );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            for ( jj = 0; jj <= q; jj++ )
                a_pq[me][jj + p + 1] = a_pq[me - 1][jj + p + 1];
        }

        uvsp[me][0] = uspan;
        uvsp[me][1] = vspan;

        N_CopyPt( DU[ii], &rhs[me] );

        if( w NEQ 1.0 )
        {
            u0 = u1 - 1.0;

            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] *= w;
            N_ScalePt( w, rhs[me], &rhs[me] );
        }
    }

    /* now the equations for the v-derivatives */
    /* for every sample point v-derivative, build */
    /*  glo_a = a_pq[(np+1)+(nu+1)+0:nv][p+q+2] = u basis and 1st v deriv functions values for sample point uu,vv values */
    /*              a_pq[np+1+nu+1+ii][0:p]       = uu basis values for sample v deriv ii  */
    /*              a_pq[np+1+nu+1+ii][(p+1)+0:q] = 1st v-deriv basis values for sample v deriv ii         */
    /*  glo_uvsp = uvsp[me+1][2]  = knot spans for sample point u-deriv uu, vv values                      */
    /*                              uvsp[np+1+nu+1+ii][0] = max u knot index LE to uu value for sample v deriv ii */
    /*                              uvsp[np+1+nu+1+ii][1] = max v knot index LE to vv value for sample v deriv ii */
    /*  rhs[me+1]                 = sample point v-derivs                                                  */
    /*                              rhs[np+1+nu+1+ii] = DV vector for sample point u-deriv ii              */

    w = 1.0;
    u0 = u1 - 1.0;
    v0 = v1 - 1.0;

    for ( ii = 0; ii <= nv; ii++ )
    {
        i2 = Iv[ii];

        if( uu[i2]LT u1 OR uu[i2]GT u2 )
            continue;

        if( vv[i2]LT v1 OR vv[i2]GT v2 )
            continue;

        if( vwts NEQ NULL )
        {
            if( vwts[ii]LE 0.0 )
                continue;
            else
            {
                if( vwts[ii]EQ 1.0 )
                    w = 1.0;
                else
                    w = sqrt( vwts[ii] );
            }
        }
        me += 1;

        if( uu[i2]NEQ u0 )
        {
            u0 = uu[i2];
            error = N_BasisEval( knu, p, uu[i2], NL_LEFT, &a_pq[me][0], &uspan );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] = a_pq[me - 1][jj];
        }

        if( vv[i2]NEQ v0 )
        {
            v0 = vv[i2];
            error = N_BasisDerivs( knv, q, vv[i2], NL_LEFT, 1, ND, &vspan );

            if( error EQ NL_YES )
                NL_OUT;

            for ( jj = 0; jj <= q; jj++ )
                a_pq[me][jj + p + 1] = ND[1][jj];
        }
        else
        {
            for ( jj = 0; jj <= q; jj++ )
                a_pq[me][jj + p + 1] = a_pq[me - 1][jj + p + 1];
        }

        uvsp[me][0] = uspan;
        uvsp[me][1] = vspan;

        N_CopyPt( DV[ii], &rhs[me] );

        if( w NEQ 1.0 )
        {
            u0 = u1 - 1.0;

            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] *= w;
            N_ScalePt( w, rhs[me], &rhs[me] );
        }
    }

    /* at this point 3 arrays have been built from the sample data and 2 arrays for indexing */
    /* DATA                                                                                  */
    /*   glo_a = a_pq    = every row stores the u and v basis values for a sample position, u-deriv, or v-deriv. */
    /*                     rows 0:np                   = [u basis values, v basis values]                        */
    /*                     rows np+1:np+1+nu           = [u 1st deriv basis values, v basis values]              */
    /*                     rows np+1+nu+1:np+1+nu+1+nv = [u basis values, v 1st deriv basis values]              */
    /*   rhs             = every row stores a sample point xyz pos, u-deriv or v-deriv value                     */
    /*                     rows 0:np                   = [ P[i]  ]                           */
    /*                     rows np+1:np+1+nu           = [ DU[i] ]                           */
    /*                     rows np+1+nu+1:np+1+nu+1+nv = [ DV[i] ]                           */
    /*   glo_uvsp = uvsp = every row store the u and v span index for every sample           */ 
    /* INDEXING                                                                              */
    /*   map[n+1][m+1]             = global index for every output surface control point     */
    /*   glo_cind = cind[nuk+1][2] = Control Point ii,jj indices for every global index      */
    /*       cind[kk][0] = ii the control point's ii index for global index kk               */
    /*       cind[kk][1] = jj the control point's jj index for global index kk               */

    /* check state more sample points than control points - no longer needed */
    /* if( me LE nuk )              */
    /*     NL_ERROR( NL_INP_ERR );  */

    /* set global size variables */
    glo_me = me;
    glo_nu = nuk;
    glo_p  = p;
    glo_q  = q;

    /* Now build and solve the system of equations                                  */
    /* let W(u,v) = Sum_l(w_l * B_l(u,v))                                           */
    /*    where w_l is the lth control point                                        */
    /*          B_l is the lth basis function                                       */
    /* for a tensor product surface B_l is separable as                             */
    /*     B_l(u,v) = B_i(u) * B_j(v)                                               */
    /*    where subscripts i and j are reserved for single direction                */
    /*          basis functions                                                     */
    /* Minimize 1/2 * (  Sum_ko(wt_k0*(W (u_k0,v_k0) - P_k0)**2       - positions   */
    /*                 + Sum_k1(wt_k1*(Wu(u_k1,v_k1) - DU_k1)**2      - u-derivs    */
    /*                 + Sum_k2(wt_k2*(Wv(u_k2,v_k2) - DV_k2)**2)     - v-derivs    */
    /*    Where Wu = d(W)/du and Wv = d(W)/dv                                       */
    /* To Solve take the partial of the cost function with respect to               */
    /*   each control point, w_l, and set those equations yielding a                */
    /*   matrix equation of the form                                                */
    /*    AX = B                                                                    */
    /*     where A[m,l] =   Sum_k0(B_m(u_k0,v_k0)  * B_l(u_k0,v_k0))                */
    /*                    + Sum_k1(Bu_m(u_k1,v_k1) * Bu_l(u_k1,v_k1))               */
    /*                    + Sum_k2(Bv_m(u_k2,v_k2) * Bv_l(u_k2,v_k2)) ;             */
    /*            X[m]  =  w_m, the mth control point                               */
    /*            B[m]  =   Sum_k0(B_m(u_k0,v_k0)  * P_k0)                          */
    /*                    + Sum_k1(Bu_m(u_k1,v_k1) * DU_k1)                         */
    /*                    + Sum_k2(Bv_m(u_k2,v_k2) * DV_k2) ;                       */
    /*     where Bu = d(B)/du and Bv = d(B)/dv                                      */
    /*     and   B_m (u,v) = B_mi(u)  * B_mj(v)                                     */
    /*           Bu_m(u,v) = Bu_mi(u) * B_mj(v)                                     */
    /*           Bv_m(u,v) = B_mi(u)  * Bv_mj(v)                                    */
    /*                                                                              */
    /* Solve AX = B via LU Decomposition                                            */

    a_nunu = N_AllocReal2dArray( nuk, nuk, &SL );

    if( a_nunu EQ NULL )
        NL_QUIT;

    /* Set up Normal Equations and nrhs */

    nrhs = N_AllocPt1dArray( nuk, &SL );

    if( nrhs EQ NULL )
        NL_QUIT;

    /* build A and B matrices */

    /* init A and B to zero - these clears are tuned to memory layout done */
    /* in  N_AllocPt1dArray() and N_AllocReal2dArray()                     */
    N_MemSet(nrhs,0,(nuk + 1) * sizeof( NL_POINT )) ;
    N_MemSet(a_nunu[0], 0, (nuk + 1) * (nuk + 1) * sizeof( NL_REAL )  );

    /* add stiffness terms to A matrix */
    alpha       = 1.0 ;
    beta        = 10.0 ; 
    PtStiffness = 1.0E10 ;

    N_AddStiffnessToSrfAMatrix(alpha, beta, sur, a_nunu, NULL, NULL, NULL, NULL) ;

    /* GWC place to add boundary lagrange constraints for periodicity and singularity */
    /* N_AddBoundaryConstraintsToAMatrix() */

    /* GWC place to add lagrange constraints to force interpolation of given boundary point in array Q */
    /* N_AddLagrangeConstraintsToAMatrix() */

    /* for every data point */
    for(kk=0;kk<=me;kk++)
      {
        /* u and v basis values for kkth data point */
        pBu = a_pq[kk] ;
        pBv = pBu + p + 1 ;

        /* u and v spans for the kkth data point    */
        USpan = uvsp[kk][0] ;
        VSpan = uvsp[kk][1] ;

        /* smallest global dof number for USpan/VSpan element */
        iStart = USpan - p;
        jStart = VSpan - q;

        /* for every V Basis function */
        for(j0=0;j0<=q;j0++)
          {
            /* map j0 to global j index */
            jj0 = jStart + j0;

            /* for every U Basis function */
            for(i0=0;i0<=p;i0++)
              {
                /* map i0 to global i index */
                ii0 = iStart + i0;

                /* map i0,j0 to global mm index */
                mm = map[ii0][jj0];

                /* Basis function B_m */
                dBm = PtStiffness * pBu[i0] * pBv[j0] ;

                /* add terms to B matrix B[mm] += rhs[kk] * B_m(u_k,v_k) */
                N_VectorBlendPt(dBm , rhs[kk], &nrhs[mm] );

                /* add B_m(uk,vk)*B_l(uk,vk) terms to A[m,l] matrix elements */
                for(j1=0;j1<=q;j1++)
                  {
                    /* map j1 to global j index */
                    jj1 = jStart + j1;

                    for(i1=0;i1<=p;i1++)
                      {
                        /* map i1 to global i index */
                        ii1 = iStart + i1;

                        /* map i1,j1 to global ll index */
                        ll = map[ii1][jj1];

                        /* add terms to A matrix */
                        a_nunu[mm][ll] +=  dBm * pBu[i1] * pBv[j1] ;
                        
                      } /* end iter every U Basis function */
                  } /* end iter every V Basis function - making A terms */

              } /* end iter every U Basis function */
          } /* end iter every V Basis function - making B and A terms */
      } /* end iter every pos, u-deriv and v-deriv input point */

    /* test new method of building a and b by comparing it to the old          */
    /*                                                                         */
    /*      a_nunu_test = N_AllocReal2dArray( nuk, nuk, &SL );                 */
    /*      if( a_nunu_test EQ NULL )                                          */
    /*          NL_QUIT;                                                       */
    /*                                                                         */
    /*      nrhs_test = N_AllocPt1dArray( nuk, &SL );                          */
    /*      if( nrhs_test EQ NULL )                                            */
    /*          NL_QUIT;                                                       */
    /*                                                                         */
    /*      col1 = N_AllocReal1dArray( me, &SL );                              */
    /*      if( col1 EQ NULL )                                                 */
    /*          NL_QUIT;                                                       */
    /*                                                                         */
    /*      col2 = N_AllocReal1dArray( me, &SL );                              */
    /*      if( col2 EQ NULL )                                                 */
    /*          NL_QUIT;                                                       */
    /*                                                                         */
    /*      for ( ii = 0; ii <= nuk; ii++ )                                    */
    /*      {                                                                  */
    /*          error = ST_GetRowsColMatrix2( -1, ii, col1 );                  */
    /*                                                                         */
    /*          if( error EQ NL_YES )                                          */
    /*              NL_OUT;                                                    */
    /*                                                                         */
    /*          N_CopyPt( NL_ZERO, &nrhs_test[ii] );                           */
    /*                                                                         */
    /*          for ( jj = 0; jj <= me; jj++ )                                 */
    /*              N_VectorBlendPt( col1[jj], rhs[jj], &nrhs_test[ii] );      */
    /*                                                                         */
    /*          for ( jj = ii; jj <= nuk; jj++ )                               */
    /*          {                                                              */
    /*              error = ST_GetRowsColMatrix2( -1, jj, col2 );              */
    /*                                                                         */
    /*              if( error EQ NL_YES )                                      */
    /*                  NL_OUT;                                                */
    /*                                                                         */
    /*              dd = 0.0;                                                  */
    /*                                                                         */
    /*              for ( kk = 0; kk <= me; kk++ )                             */
    /*                  dd += col1[kk] * col2[kk];                             */
    /*              a_nunu_test[ii][jj] = dd;                                  */
    /*          }                                                              */
    /*                                                                         */
    /*          for ( jj = ii + 1; jj <= nuk; jj++ )                           */
    /*              a_nunu_test[jj][ii] = a_nunu_test[ii][jj];                 */
    /*      }                                                                  */
    /*                                                                         */
    /*        now compare                                                      */
    /*      lTestCnt=0 ;                                                       */
    /*      dTestMax=0.0 ;                                                     */
    /*      for(ii=0;ii<=nuk;ii++)                                             */
    /*        {                                                                */
    /*          if(  fabs(nrhs[ii].x - nrhs_test[ii].x)                        */
    /*             + fabs(nrhs[ii].y - nrhs_test[ii].y)                        */
    /*             + fabs(nrhs[ii].z - nrhs_test[ii].z) > 1.0E-6)              */
    /*            lTestCnt++ ;                                                 */
    /*          if(  fabs(nrhs[ii].x - nrhs_test[ii].x)                        */
    /*             + fabs(nrhs[ii].y - nrhs_test[ii].y)                        */
    /*             + fabs(nrhs[ii].z - nrhs_test[ii].z) > dTestMax)            */
    /*            dTestMax = fabs(  fabs(nrhs[ii].x - nrhs_test[ii].x)         */
    /*                            + fabs(nrhs[ii].y - nrhs_test[ii].y)         */
    /*                            + fabs(nrhs[ii].z - nrhs_test[ii].z)) ;      */
    /*                                                                         */
    /*          for(jj=0;jj<=nuk;jj++)                                         */
    /*            {                                                            */
    /*              if(fabs(a_nunu[ii][jj] - a_nunu_test[ii][jj]) > 1.0E-6)    */
    /*                lTestCnt++ ;                                             */
    /*              if(fabs(a_nunu[ii][jj] - a_nunu_test[ii][jj]) > dTestMax)  */
    /*                dTestMax = fabs(a_nunu[ii][jj] - a_nunu_test[ii][jj]) ;  */
    /*            }                                                            */
    /*        }                                                                */
    /*        end change temp test code                                        */
                                                         

    /* Now solve via LU decomposition (Crout with partial piv) */

    perm = N_AllocInt1dArray( nuk, &SL );

    if( perm EQ NULL )
        NL_QUIT;

    N_CreateRealMatrix( &rma, nuk, nuk, a_nunu, NL_MT_FULL, nuk );

    /* Failure to solve occurs here if data not ok.                                     */
    /*   One kind of problem: Every control point has to be near sample points.         */
    /*      When no sample points are within the domain of the basis functions of a     */
    /*      surface point, the equation for that point will be all zero and the solver  */
    /*      will fail.                                                                  */
    error = N_RealMatrixLuDecomposePivot( &rma, perm );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_RealMatrixRightForBackPivot( &rma, perm, nrhs, Qw );

    if( error EQ NL_YES )
        NL_OUT;

    /* Load the control points into Sw[][] via the index map */

    for ( ii = 0; ii <= n; ii++ )
      {
        for ( jj = 0; jj <= m; jj++ )
          {
            if( map[ii][jj]GE 0 )
                N_CopyCPt( Qw[map[ii][jj]], &Sw[ii][jj] );
          }
      }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);

} /* end N_FitSrfLstSqDerivsWithConstraints */

#endif /* UNDER CONSTRUCTION */

#endif // NLIB_UNUSED

/**********************************************************************/
/* N_FITSRFLSTSQDERIVS: Least squares surface approximation to random points     */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This fitting  routine  computes  a  least squares b-spline  surface 
     approximation to a random (non-NxM) set of  points.  First  partial
     derivative vectors with respect to u or v may (optionally) be input
     at each point.  When the number of sample points is large the
     algorithm can not interpolate every input point, it will find the
     best approximation to those points.
     
     If  the  output  surface is initialized to the NULL 
     surface,  memory is allocated locally.  Otherwise  it is checked if  
     enough memory is passed in. A typical calling example is:
 
       NL_POINT      *P;
       NL_VECTOR     *DU, *DV;
       NL_PARAMETER  *uu, *vv;
       NL_REAL       *wts, *uwts, *vwts
       NL_INDEX      m, n, np, nu, nv, *Iu, *Iv;
       NL_DEGREE     p, q;
       NL_SURFACE    sur;
       NL_STACKS     SG;

       ...
       N_SrfInitArrays(&sur);
       N_FitSrfLstSqDerivs(P,wts,uu,vv,np,DU,uwts,nu,Iu,DV,vwts,nv,Iv,n,m,p,q,
                U,V,&sur,&SG);
 
 
   ACCESS:
   
     P    , input  ,  Points to be approximated
     wts  , input  ,  Least squares point weights (wts[i] >= 0  for  all
                      i).  The  larger  wts[i],  the  closer the surface
                      comes to P[i]. wts[i] < 1 lessens the influence of
                      P[i]. If wts[i] = 0, then P[i] is not used.  If no
                      weighting is desired, set wts = NULL
     uu   , input  ,  The u-parameters of the points in P
     vv   , input  ,  The v-parameters of the points in P
     np   , input  ,  The high index of the arrays: P,wts,uu,vv
     DU   , input  ,  First derivative vectors wrt u (optional). DU=NULL
                      means no u-derivatives specified
     uwts , input  ,  u-derivative weights (uwts[i] >= 0 for all i). The
                      larger uwts[i],  the  closer  the surface comes to 
                      assuming DU[i].  uwts[i] < 1 lessens the influence 
                      of DU[i].  If uwts[i] = 0, then DU[i] is not used.  
                      If no weighting is desired, set uwts = NULL
     nu   , input  ,  The high index of the arrays: DU, uwts, Iu (set to
                      -1 if no u-derivatives specified)
     Iu   , input  ,  DU[i] is the u-derivative at (uu[Iu[i]],vv[Iu[i]])
     DV   , input  ,  First derivative vectors wrt v (optional). DV=NULL
                      means no v-derivatives specified
     vwts , input  ,  v-derivative weights (vwts[i] >= 0 for all i). The
                      larger vwts[i],  the  closer  the surface comes to 
                      assuming DV[i].  vwts[i] < 1 lessens the influence 
                      of DV[i].  If vwts[i] = 0, then DV[i] is not used.  
                      If no weighting is desired, set vwts = NULL
     nv   , input  ,  The high index of the arrays: DV, vwts, Iv (set to
                      -1 if no v-derivatives specified)
     Iv   , input  ,  DV[i] is the v-derivative at (uu[Iv[i]],vv[Iv[i]])
     n,m  , input  ,  High indexes of the surface  control  points  (the
                      surface will have (n+1)x(m+1) control points).  
     p,q  , input  ,  Degrees of the surface  
     U,V  , input  ,  Knots for the surface.  If  U=NULL or V=NULL,  the
                      corresponding knots are  computed in this routine.
                      If given, there must be n+p+2 u-knots and/or m+q+2
                      v-knots.  The values in the uu and vv arrays  must
                      correspond to the knot ranges (unless wts[i] = 0)
     sur  , output ,  Approximating surface
     SG   , input  ,  sur's memory stack

 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR

   NOTE:
     The surface domain is extended a small amount past the sample points.
     This resolves problems we've found when a surface constructed from
     sample points fails to span those sample points completely due to
     small tolerance errors.  Adding an extra bit of surface to the perimeter
     enables subsequent operations, like projection of a curve onto a surface
     known to lie on the boundary of this surface to succeed.  If you want
     exact boundaries on your surface, you must trim the returned BSpline
     to a UVTrimCurve constructed by projecting a 3d curve onto this surface. 

    Sample code including SMLib routines:
    - compute UV values 

    N_FitSrfCalcParams(P, m, NL_NO, Origin, X, Y, Z, 0.0, 1.0, 0.0, 1.0, uu, vv);
    
    - Fit surface to (xyz uv) points
    error = N_FitSrfLstSqDerivs(P, NULL, uu, vv, m, NULL, NULL, -1, NULL, NULL, NULL, 
            -1, NULL, 4, 4, 3, 3, NULL, NULL, &sur, &S);

    SmBSplineSurface *pSrf = new(sContext) SmBSplineSurface(&sur);   

   ***********************************************************************/

NL_FLAG N_FitSrfLstSqDerivs
  (NL_POINT   *P,       /* in : P    = Points to be approximated,              sized:[np+1]  */ 
   NL_REAL    *wts,     /* in : wts  = Associated Least Squares Point weights, sized:[np+1]  */ 
   NL_REAL    *uu,      /* in : uu   = The u-parameters of the points in P,    sized:[np+1]  */ 
   NL_REAL    *vv,      /* in : vv   = The v-parameters of the points in P,    sized:[np+1]  */ 
   NL_INDEX    np,      /* in : np   = max index of P, wts, uu, and vv                  */ 
   NL_VECTOR  *DU,      /* in : DU   = First derivative vectors wrt u (optional),     NULL to ignore, sized:[nu+1] */ 
   NL_REAL    *uwts,    /* in : uwts = Associated Least Squares u-derivative weights, NULL to ignore, sized:[nu+1] */ 
   NL_INDEX    nu,      /* in : nu   = max index value of DU, uwts, and Iu arrays       */
   NL_INDEX   *Iu,      /* in : Iu   = DU and uwts index map,                  sized;[nu+1]  */
                        /*        e.g. DU[i] is u-derivative at (uu[Iu[i]],vv[Iu[i]]) */ 
   NL_VECTOR  *DV,      /* in : DV   = First derivative vectors wrt v (optional),     NULL to ignore, sized:[nv+1] */ 
   NL_REAL    *vwts,    /* in : vwts = Associated Least Squares v-derivative weights, NULL to ignore, sized:[nv+1] */ 
   NL_INDEX    nv,      /* in : nv   = max index value of DV, vwts, and Iv arrays       */                      
   NL_INDEX   *Iv,      /* in : Iv   = DV and vwts index map,                  sized;[nv+1]  */                      
                        /*        e.g. DV[i] is v-derivative at (uu[Iv[i]],vv[Iv[i]]) */                      
   NL_INDEX    n,       /* in : n   = Output Surface U ControlPoint max index, CptCount = (n+1)x(m+1)     */ 
   NL_INDEX    m,       /* in : m   = Output Surface V ControlPoint max index, CptCount = (n+1)x(m+1)     */ 
   NL_DEGREE   p,       /* in : p   = Output Surface U Degree                               */ 
   NL_DEGREE   q,       /* in : q   = Output Surface V Degree                               */ 
   NL_REAL    *U,       /* in : U   = Optional Output Surface U Knot Vector, if given must have n+p+2 u-knots, Null to ignore */ 
   NL_REAL    *V,       /* in : V   = Optional Output Surface V KNot Vector, if given must have m+q+2 v-knots, Null to ignore */ 
   NL_SURFACE *sur,     /* out: sur = Approximating Surface */ 
   NL_STACKS  *SG )     /* in : sur's memory stack    */ 
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfLstSqDerivs");

    NL_FLAG error = NL_NO, pflg;

    NL_INDEX ii, jj, kk, ** map, me, nuk, uspan = 0, vspan = 0, j1, k1, *perm, ** uvsp, ** cind, i2;

    NL_REAL *US, *VS, ** a_nunu, w, u1, u2, v1, v2, ** a_pq, u0, v0, ** ND;
    
    /* For change temp test only                  */
    /*      NL_REAL *col1, *col2, dd, dTestMax ;  */
    /*      NL_REAL **a_nunu_test ;               */
    /*      NL_POINT *nrhs_test ;                 */
    /*      NL_INDEX lTestCnt ;                   */

    NL_INDEX i0, j0, i1, USpan, VSpan, mStart, mm, ll ;

    NL_REAL *pBu, *pBv, dBm, scl, off ; 

    NL_CPOINT *Qw, ** Sw;

    NL_POINT P1, *rhs, *nrhs;

    NL_KNOTVECTOR knt, *knu, *knv, *knus, *knvs;

    NL_RMATRIX rma;

    /* these A matrix gain terms are kind of arbitrary and can be tuned by experiment */
    NL_REAL alpha, beta, PtStiffness ;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for input errors */

    if(    n LT 1                          /* no U Control Points */
        OR m LT 1)                          /* no V Control Points */
        /* OR np + 1 LT( n + 1 ) * (m + 1) )  - fewer Points than ControlPoints */ 
        NL_ERROR( NL_INP_ERR );

    if(    n LT p                          /* fewer output ControlPoints than degree+1 in U dir */
        OR m LT q )                        /* fewer output ControlPoints than degree+1 in V dir */
        NL_ERROR( NL_INP_ERR );

    /* Check validity of option input U Knotvector */
    if( U NEQ NULL )
    {
        N_KnotVectorFromRealArray( &knt, U, n + p + 1 );
        error = N_KnotVectorIsValid( &knt, p, rname );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Check validity of option input U Knotvector */
    if( V NEQ NULL )
    {
        N_KnotVectorFromRealArray( &knt, V, m + q + 1 );   
        error = N_KnotVectorIsValid( &knt, q, rname );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* remember NO input u-derivative data */
    if( nu LT 0 OR DU EQ NULL )
        nu = -1;

    /* remember NO input v-derivative data */
    if( nv LT 0 OR DV EQ NULL )
        nv = -1;

    /* initialise just to avoid compiler complaint */
    u1 = v1 = 0.0;
    u2 = v2 = 1.0;

    /* Check memory for output surface */
    /* allocate memory when sur arrays are NULL (via N_SrfInitArrays()) */

    error = N_SrfSizeArrays( sur, n, m, p, q, n + p + 1, m + q + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Sw, &US, &VS );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Get the surface u- and v-knots */

    u1 = u2 = 0.0;
    v1 = v2 = 0.0;

    pflg = NL_NO;

    /* when given Opt U KnotVector - set knus as given */
    if( U NEQ NULL )
    {
        kk = n + p + 1;

        for ( ii = 0; ii <= kk; ii++ )
            US[ii] = U[ii];
        knus = knu;
    }
    else /* set knus to NULL and get max and min u values from input uu array */
    {
        knus = NULL;

        /* when given weights */
        if( wts NEQ NULL )
        {
            pflg = NL_YES;
            u1 = 1.0e+20;
            u2 = -u1;

            /* get max and min uu array values */
            for ( ii = 0; ii <= np; ii++ )
                if( wts[ii]NEQ 0.0 )
                {
                    if( u1 GT uu[ii] )
                        u1 = uu[ii];

                    if( u2 LT uu[ii] )
                        u2 = uu[ii];
                }
        }
    }

    /* when given Opt U KnotVector - set knvs as given */
    if( V NEQ NULL )
    {
        kk = m + q + 1;

        for ( ii = 0; ii <= kk; ii++ )
            VS[ii] = V[ii];
        knvs = knv;
    }
    else  /* set knvs to NULL and get max and min v values from input uu array */
    {
        knvs = NULL;

        /* when wts are given */
        if( wts NEQ NULL )
        {
            pflg = NL_YES;
            v1 = 1.0e+20;
            v2 = -v1;

            /* get max and min vv array values */
            for ( ii = 0; ii <= np; ii++ )
                if( wts[ii]NEQ 0.0 )
                {
                    if( v1 GT vv[ii] )
                        v1 = vv[ii];

                    if( v2 LT vv[ii] )
                        v2 = vv[ii];
                }
        }
    }

    /* when surface knot vectors have not been specified */
    if( knus NEQ knu OR knvs NEQ knv )
    {
        /* construct surface knot vectors */
        error = N_FitSrfCalcKnotVectors
          ( uu,       /* in : u parameter values, sized:[nn+1] */                                               
            vv,       /* in : v parameter values, sized:[nn+1] */                                               
            np,       /* in : highest u and v index */                                                          
            p,        /* in : approximating surface degree U */                                                 
            q,        /* in : approximating surface degree V */                                                 
            pflg,     /* in : NL_YES = use us,ue,vs,ve values to set max/min u and v knot values */             
                      /*      NL_NO  = find max/min u and v knot values in u and v arrays        */             
            u1,       /* in : min U param value, overridden by any input knu value   */                         
            u2,       /* in : max U param value, overridden by any input knu value   */                         
            v1,       /* in : min V param value, overridden by any input knv value   */                         
            v2,       /* in : max V param value, overridden by any input knv value   */                         
            knus,     /* in : opt starter U knots, if given these knots will be in the output, NULL to ignore */
            knvs,     /* in : opt starter V knots, if given these knots will be in the output, NULL to ignore */
            knu,      /* i/o: sized knot vector whose knot values are to be determined */                       
            knv );    /* i/o: sized knot vector whose knot values are to be determined */                       

        if( error EQ NL_YES )
            NL_OUT;

        /* scale the knots so that the sample points won't lie exactly on the boundary */
        scl = 1.125 ;
        off = knu->U[0] - (knu->U[knu->m]-knu->U[0])*(scl-1)/2.0 - knu->U[0]*scl ; 
        for(ii=0;ii<=knu->m;ii++) { knu->U[ii] = scl * knu->U[ii] + off ; }

        off = knv->U[0] - (knv->U[knv->m]-knv->U[0])*(scl-1)/2.0 - knv->U[0]*scl ; 
        for(ii=0;ii<=knv->m;ii++) { knv->U[ii] = scl * knv->U[ii] + off ; }
    }

    /* Allocate memory for the index map */
    map = N_AllocInt2dArray( n, m, &SL );

    if( map EQ NULL )
        NL_QUIT;

    cind = N_AllocInt2dArray( (n + 1) * (m + 1) - 1, 1, &SL );

    if( cind EQ NULL )
        NL_QUIT;

    /* Define the map from Sw(i,j) to svd_a's column index                                */
    /* map[n+1][m+1]             = global index for every output surface control point    */
    /* glo_cind = cind[nuk+1][2] = Control Point ii,jj indices for every global index     */
    /*     cind[kk][0] = ii the control point's ii index for global index kk              */
    /*     cind[kk][1] = jj the control point's jj index for global index kk              */
    /* gwc note: these maps could be replaced by indexing functions as                    */
    /*   map[ii][jj] = (jj*(n+1))+ii ;                                                    */
    /*   cind[kk][0] =  kk-((kk%(n+1))*n+1)                                               */
    /*   cind[kk][1] =  kk%(n+1)                                                          */

    nuk = -1; /* nuk+1 unknowns */

    for ( jj = 0; jj <= m; jj++ )
    {
        for ( ii = 0; ii <= n; ii++ )
        {
            nuk += 1;
            map[ii][jj] = nuk;

            cind[nuk][0] = ii;
            cind[nuk][1] = jj;
        }
    }

    /* Allocate memory for the right hand side and solution vector */

    Qw = N_AllocCPt1dArray( nuk, &SL );

    if( Qw EQ NULL )
        NL_QUIT;

    me = np;
    me += (nu + 1);
    me += (nv + 1);

    rhs = N_AllocPt1dArray( me, &SL );

    if( rhs EQ NULL )
        NL_QUIT;

    /* Now set up the overdetermined system of equations */

    u1 = US[0];
    u2 = US[n + 1];
    v1 = VS[0];
    v2 = VS[m + 1];

    w = 1.0;
    u0 = u1 - 1.0;
    v0 = v1 - 1.0;

    a_pq = N_AllocReal2dArray( me, p + q + 1, &SL );

    if( a_pq EQ NULL )
        NL_QUIT;

    uvsp = N_AllocInt2dArray( me, 1, &SL );

    if( uvsp EQ NULL )
        NL_QUIT;

    me = -1; /* me+1 equations to solve */

    glo_a = a_pq;
    glo_cind = cind;
    glo_uvsp = uvsp;

    /* set up system */

    /* first, the equations for the sample point positions */

    /* for every sample point position - build                                                             */
    /*  glo_a = a_pq[me+1][p+q+2] = basis functions values for sample point uu,vv values                   */
    /*                              a_pq[ii][0:p]       = uu basis values for sample point ii              */
    /*                              a_pq[ii][(p+1)+0:q] = vv basis values for sample point ii              */
    /*  glo_uvsp = uvsp[me+1][2]  = knot spans for sample point uu, vv values                              */
    /*                              uvsp[ii][0] = max u knot index LE to uu value for sample point ii      */
    /*                              uvsp[ii][1] = max v knot index LE to vv value for sample point ii      */
    /*  rhs[me+1]                 = sample point positions                                                 */
    /*                              rhs[ii] = xyz position for sample point ii                             */
    /* when given optional wts, each rhs[ii] and a_pq[ii][0:p] uu basis values                             */
    /*   are multiplied by the given wts[ii] value        */
    /* ignore sample points when                          */
    /*       o. uu value is out of specified bounds       */
    /*       o. vv value is out of specified bounds       */
    /*       0. optional weight is LE 0.0                 */
    for ( ii = 0; ii <= np; ii++ )
    {
        if( uu[ii]LT u1 OR uu[ii]GT u2 )
            continue;

        if( vv[ii]LT v1 OR vv[ii]GT v2 )
            continue;

        if( wts NEQ NULL )
        {
            if( wts[ii]LE 0.0 )
                continue;
            else
            {
                if( wts[ii]EQ 1.0 )
                    w = 1.0;
                else
                    w = sqrt( wts[ii] );
            }
        }

        me += 1;

        if( uu[ii]NEQ u0 )
        {
            u0 = uu[ii];
            error = N_BasisEval( knu, p, uu[ii], NL_LEFT, &a_pq[me][0], &uspan );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] = a_pq[me - 1][jj];
        }

        if( vv[ii]NEQ v0 )
        {
            v0 = vv[ii];
            error = N_BasisEval( knv, q, vv[ii], NL_LEFT, &a_pq[me][p + 1], &vspan );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            for ( jj = 0; jj <= q; jj++ )
                a_pq[me][jj + p + 1] = a_pq[me - 1][jj + p + 1];
        }

        uvsp[me][0] = uspan;
        uvsp[me][1] = vspan;

        N_CopyPt( P[ii], &rhs[me] );

        if( w NEQ 1.0 )
        {
            u0 = u1 - 1.0;

            for ( jj = 0; jj <= p; jj++ )      /* <=== gwc: why isn't this jj <= p+q+1 */ 
                a_pq[me][jj] *= w;             /* because all left side terms are p*q products and this puts just 1 w on the equation left and right */
            N_ScalePt( w, rhs[me], &rhs[me] );
        }

        for ( jj = 0; jj <= p; jj++ )
        {
            j1 = uspan - p + jj;

            for ( kk = 0; kk <= q; kk++ )
            {
                k1 = vspan - q + kk;

                if( map[j1][k1]LT 0 ) /* gwc: I think this is never true */
                {
                    N_CPtToPtEuclid( Sw[j1][k1], &P1 );   /* gwc: I think this is uninitialized when sur allocated in this function */ 
                    N_VectorBlendPt( -(a_pq[me][jj] * a_pq[me][(p + 1) + kk]), P1, &rhs[me] );
                }
            } /* end iter every sur DegreeV value */
        } /* end iter every sur DegreeU value */
    } /* end iter every sample point - building a matrix position equations */

    /* now the equations for the u-derivatives  get appended to same arrays used for positions */
    /* for every sample point u-derivative, build */
    /*  glo_a = a_pq[(np+1)+0:nu][p+q+2] = 1st u deriv and v basis functions values for sample point uu,vv values */
    /*              a_pq[np+1+ii][0:p]       = 1st deriv uu basis values for sample u deriv ii  */
    /*              a_pq[np+1+ii][(p+1)+0:q] = vv basis values for sample point ii              */
    /*  glo_uvsp = uvsp[me+1][2]  = knot spans for sample point u-deriv uu, vv values                      */
    /*                              uvsp[np+1+ii][0] = max u knot index LE to uu value for sample point ii */
    /*                              uvsp[np+1+ii][1] = max v knot index LE to vv value for sample point ii */
    /*  rhs[me+1]                 = sample point u-derivs                                                  */
    /*                              rhs[np+1+ii] = DU vector for sample point u-deriv ii                   */

    ii = NL_MAX( p, q );
    ND = N_AllocReal2dArray( 1, ii, &SL );

    if( ND EQ NULL )
        NL_QUIT;

    w = 1.0;
    u0 = u1 - 1.0;
    v0 = v1 - 1.0;

    for ( ii = 0; ii <= nu; ii++ )
    {
        i2 = Iu[ii];

        if( uu[i2]LT u1 OR uu[i2]GT u2 )
            continue;

        if( vv[i2]LT v1 OR vv[i2]GT v2 )
            continue;

        if( uwts NEQ NULL )
        {
            if( uwts[ii]LE 0.0 )
                continue;
            else
            {
                if( uwts[ii]EQ 1.0 )
                    w = 1.0;
                else
                    w = sqrt( uwts[ii] );
            }
        }

        me += 1;

        if( uu[i2]NEQ u0 )
        {
            u0 = uu[i2];
            error = N_BasisDerivs( knu, p, uu[i2], NL_LEFT, 1, ND, &uspan );

            if( error EQ NL_YES )
                NL_OUT;

            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] = ND[1][jj];
        }
        else
        {
            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] = a_pq[me - 1][jj];
        }

        if( vv[i2]NEQ v0 )
        {
            v0 = vv[i2];
            error = N_BasisEval( knv, q, vv[i2], NL_LEFT, &a_pq[me][p + 1], &vspan );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            for ( jj = 0; jj <= q; jj++ )
                a_pq[me][jj + p + 1] = a_pq[me - 1][jj + p + 1];
        }

        uvsp[me][0] = uspan;
        uvsp[me][1] = vspan;

        N_CopyPt( DU[ii], &rhs[me] );

        if( w NEQ 1.0 )
        {
            u0 = u1 - 1.0;

            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] *= w;
            N_ScalePt( w, rhs[me], &rhs[me] );
        }
    }

    /* now the equations for the v-derivatives */
    /* for every sample point v-derivative, build */
    /*  glo_a = a_pq[(np+1)+(nu+1)+0:nv][p+q+2] = u basis and 1st v deriv functions values for sample point uu,vv values */
    /*              a_pq[np+1+nu+1+ii][0:p]       = uu basis values for sample v deriv ii  */
    /*              a_pq[np+1+nu+1+ii][(p+1)+0:q] = 1st v-deriv basis values for sample v deriv ii         */
    /*  glo_uvsp = uvsp[me+1][2]  = knot spans for sample point u-deriv uu, vv values                      */
    /*                              uvsp[np+1+nu+1+ii][0] = max u knot index LE to uu value for sample v deriv ii */
    /*                              uvsp[np+1+nu+1+ii][1] = max v knot index LE to vv value for sample v deriv ii */
    /*  rhs[me+1]                 = sample point v-derivs                                                  */
    /*                              rhs[np+1+nu+1+ii] = DV vector for sample point u-deriv ii              */

    w = 1.0;
    u0 = u1 - 1.0;
    v0 = v1 - 1.0;

    for ( ii = 0; ii <= nv; ii++ )
    {
        i2 = Iv[ii];

        if( uu[i2]LT u1 OR uu[i2]GT u2 )
            continue;

        if( vv[i2]LT v1 OR vv[i2]GT v2 )
            continue;

        if( vwts NEQ NULL )
        {
            if( vwts[ii]LE 0.0 )
                continue;
            else
            {
                if( vwts[ii]EQ 1.0 )
                    w = 1.0;
                else
                    w = sqrt( vwts[ii] );
            }
        }
        me += 1;

        if( uu[i2]NEQ u0 )
        {
            u0 = uu[i2];
            error = N_BasisEval( knu, p, uu[i2], NL_LEFT, &a_pq[me][0], &uspan );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] = a_pq[me - 1][jj];
        }

        if( vv[i2]NEQ v0 )
        {
            v0 = vv[i2];
            error = N_BasisDerivs( knv, q, vv[i2], NL_LEFT, 1, ND, &vspan );

            if( error EQ NL_YES )
                NL_OUT;

            for ( jj = 0; jj <= q; jj++ )
                a_pq[me][jj + p + 1] = ND[1][jj];
        }
        else
        {
            for ( jj = 0; jj <= q; jj++ )
                a_pq[me][jj + p + 1] = a_pq[me - 1][jj + p + 1];
        }

        uvsp[me][0] = uspan;
        uvsp[me][1] = vspan;

        N_CopyPt( DV[ii], &rhs[me] );

        if( w NEQ 1.0 )
        {
            u0 = u1 - 1.0;

            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] *= w;
            N_ScalePt( w, rhs[me], &rhs[me] );
        }
    }

    /* at this point 3 arrays have been built from the sample data and 2 arrays for indexing */
    /* DATA                                                                                  */
    /*   glo_a = a_pq    = every row stores the u and v basis values for a sample position, u-deriv, or v-deriv. */
    /*                     rows 0:np                   = [u basis values, v basis values]                        */
    /*                     rows np+1:np+1+nu           = [u 1st deriv basis values, v basis values]              */
    /*                     rows np+1+nu+1:np+1+nu+1+nv = [u basis values, v 1st deriv basis values]              */
    /*   rhs             = every row stores a sample point xyz pos, u-deriv or v-deriv value                     */
    /*                     rows 0:np                   = [ P[i]  ]                           */
    /*                     rows np+1:np+1+nu           = [ DU[i] ]                           */
    /*                     rows np+1+nu+1:np+1+nu+1+nv = [ DV[i] ]                           */
    /*   glo_uvsp = uvsp = every row store the u and v span index for every sample           */ 
    /* INDEXING                                                                              */
    /*   map[n+1][m+1]             = global index for every output surface control point     */
    /*   glo_cind = cind[nuk+1][2] = Control Point ii,jj indices for every global index      */
    /*       cind[kk][0] = ii the control point's ii index for global index kk               */
    /*       cind[kk][1] = jj the control point's jj index for global index kk               */

    /* check state more sample points than control points - no longer needed */
    /* if( me LE nuk )              */
    /*     NL_ERROR( NL_INP_ERR );  */

    /* set global size variables */
    glo_me = me;
    glo_nu = nuk;
    glo_p  = p;
    glo_q  = q;

    /* Now build and solve the system of equations                                  */
    /* let W(u,v) = Sum_l(w_l * B_l(u,v))                                           */
    /*    where w_l is the lth control point                                        */
    /*          B_l is the lth basis function                                       */
    /* for a tensor product surface B_l is separable as                             */
    /*     B_l(u,v) = B_i(u) * B_j(v)                                               */
    /*    where subscripts i and j are reserved for single direction                */
    /*          basis functions                                                     */
    /* Minimize 1/2 * (  Sum_ko(wt_k0*(W (u_k0,v_k0) - P_k0)**2       - positions   */
    /*                 + Sum_k1(wt_k1*(Wu(u_k1,v_k1) - DU_k1)**2      - u-derivs    */
    /*                 + Sum_k2(wt_k2*(Wv(u_k2,v_k2) - DV_k2)**2)     - v-derivs    */
    /*    Where Wu = d(W)/du and Wv = d(W)/dv                                       */
    /* To Solve take the partial of the cost function with respect to               */
    /*   each control point, w_l, and set those equations yielding a                */
    /*   matrix equation of the form                                                */
    /*    AX = B                                                                    */
    /*     where A[m,l] =   Sum_k0(B_m(u_k0,v_k0)  * B_l(u_k0,v_k0))                */
    /*                    + Sum_k1(Bu_m(u_k1,v_k1) * Bu_l(u_k1,v_k1))               */
    /*                    + Sum_k2(Bv_m(u_k2,v_k2) * Bv_l(u_k2,v_k2)) ;             */
    /*            X[m]  =  w_m, the mth control point                               */
    /*            B[m]  =   Sum_k0(B_m(u_k0,v_k0)  * P_k0)                          */
    /*                    + Sum_k1(Bu_m(u_k1,v_k1) * DU_k1)                         */
    /*                    + Sum_k2(Bv_m(u_k2,v_k2) * DV_k2) ;                       */
    /*     where Bu = d(B)/du and Bv = d(B)/dv                                      */
    /*     and   B_m (u,v) = B_mi(u)  * B_mj(v)                                     */
    /*           Bu_m(u,v) = Bu_mi(u) * B_mj(v)                                     */
    /*           Bv_m(u,v) = B_mi(u)  * Bv_mj(v)                                    */
    /*                                                                              */
    /* Solve AX = B via LU Decomposition                                            */

    a_nunu = N_AllocReal2dArray( nuk, nuk, &SL );

    if( a_nunu EQ NULL )
        NL_QUIT;

    /* Set up Normal Equations and nrhs */

    nrhs = N_AllocPt1dArray( nuk, &SL );

    if( nrhs EQ NULL )
        NL_QUIT;

    /* build A and B matrices */

    /* init A and B to zero - these clears are tuned to memory layout done */
    /* in  N_AllocPt1dArray() and N_AllocReal2dArray()                     */
    N_MemSet(nrhs,0,(nuk + 1) * sizeof( NL_POINT )) ;
    N_MemSet(a_nunu[0], 0, (nuk + 1) * (nuk + 1) * sizeof( NL_REAL )  );

    /* add stiffness terms to A matrix */
    alpha       = 1.0 ;
    beta        = 10.0 ; 
    PtStiffness = 1.0E10 ;
    N_AddStiffnessToSrfAMatrix(alpha, beta, sur, a_nunu, NULL, NULL, NULL, NULL) ;

    /* for every data point */
    for(kk=0;kk<=me;kk++)
      {
        /* u and v basis values for kkth data point */
        pBu = a_pq[kk] ;
        pBv = pBu + p + 1 ;

        /* u and v spans for the kkth data point    */
        USpan = uvsp[kk][0] ;
        VSpan = uvsp[kk][1] ;

        /* smallest global dof number for USpan/VSpan element */
        mStart = ((VSpan - q) * (n + 1)) + (USpan - p) ;

        /* for every V Basis function */
        for(j0=0;j0<=q;j0++)
          {
            /* map i0,j0 to global mm index */
            mm = mStart + (j0 * (n + 1)) ;

            /* for every U Basis function */
            for(i0=0;i0<=p;i0++, mm++)
              {
                /* Basis function B_m */
                dBm = PtStiffness * pBu[i0] * pBv[j0] ;

                /* add terms to B matrix B[mm] += rhs[kk] * B_m(u_k,v_k) */
                N_VectorBlendPt(dBm , rhs[kk], &nrhs[mm] );

                /* add B_m(uk,vk)*B_l(uk,vk) terms to A[m,l] matrix elements */
                for(j1=0;j1<=q;j1++)
                  {
                    /* map i1,j1 to global ll index */
                    ll = mStart + (j1 * (n + 1)) ;

                    for(i1=0;i1<=p;i1++,ll++)
                      {
                        /* add terms to A matrix */
                        a_nunu[mm][ll] +=  dBm * pBu[i1] * pBv[j1] ;
                        
                      } /* end iter every U Basis function */
                  } /* end iter every V Basis function - making A terms */

              } /* end iter every U Basis function */
          } /* end iter every V Basis function - making B and A terms */
      } /* end iter every pos, u-deriv and v-deriv input point */

    /* test new method of building a and b by comparing it to the old          */
    /*                                                                         */
    /*      a_nunu_test = N_AllocReal2dArray( nuk, nuk, &SL );                 */
    /*      if( a_nunu_test EQ NULL )                                          */
    /*          NL_QUIT;                                                       */
    /*                                                                         */
    /*      nrhs_test = N_AllocPt1dArray( nuk, &SL );                          */
    /*      if( nrhs_test EQ NULL )                                            */
    /*          NL_QUIT;                                                       */
    /*                                                                         */
    /*      col1 = N_AllocReal1dArray( me, &SL );                              */
    /*      if( col1 EQ NULL )                                                 */
    /*          NL_QUIT;                                                       */
    /*                                                                         */
    /*      col2 = N_AllocReal1dArray( me, &SL );                              */
    /*      if( col2 EQ NULL )                                                 */
    /*          NL_QUIT;                                                       */
    /*                                                                         */
    /*      for ( ii = 0; ii <= nuk; ii++ )                                    */
    /*      {                                                                  */
    /*          error = ST_GetRowsColMatrix2( -1, ii, col1 );                  */
    /*                                                                         */
    /*          if( error EQ NL_YES )                                          */
    /*              NL_OUT;                                                    */
    /*                                                                         */
    /*          N_CopyPt( NL_ZERO, &nrhs_test[ii] );                           */
    /*                                                                         */
    /*          for ( jj = 0; jj <= me; jj++ )                                 */
    /*              N_VectorBlendPt( col1[jj], rhs[jj], &nrhs_test[ii] );      */
    /*                                                                         */
    /*          for ( jj = ii; jj <= nuk; jj++ )                               */
    /*          {                                                              */
    /*              error = ST_GetRowsColMatrix2( -1, jj, col2 );              */
    /*                                                                         */
    /*              if( error EQ NL_YES )                                      */
    /*                  NL_OUT;                                                */
    /*                                                                         */
    /*              dd = 0.0;                                                  */
    /*                                                                         */
    /*              for ( kk = 0; kk <= me; kk++ )                             */
    /*                  dd += col1[kk] * col2[kk];                             */
    /*              a_nunu_test[ii][jj] = dd;                                  */
    /*          }                                                              */
    /*                                                                         */
    /*          for ( jj = ii + 1; jj <= nuk; jj++ )                           */
    /*              a_nunu_test[jj][ii] = a_nunu_test[ii][jj];                 */
    /*      }                                                                  */
    /*                                                                         */
    /*        now compare                                                      */
    /*      lTestCnt=0 ;                                                       */
    /*      dTestMax=0.0 ;                                                     */
    /*      for(ii=0;ii<=nuk;ii++)                                             */
    /*        {                                                                */
    /*          if(  fabs(nrhs[ii].x - nrhs_test[ii].x)                        */
    /*             + fabs(nrhs[ii].y - nrhs_test[ii].y)                        */
    /*             + fabs(nrhs[ii].z - nrhs_test[ii].z) > 1.0E-6)              */
    /*            lTestCnt++ ;                                                 */
    /*          if(  fabs(nrhs[ii].x - nrhs_test[ii].x)                        */
    /*             + fabs(nrhs[ii].y - nrhs_test[ii].y)                        */
    /*             + fabs(nrhs[ii].z - nrhs_test[ii].z) > dTestMax)            */
    /*            dTestMax = fabs(  fabs(nrhs[ii].x - nrhs_test[ii].x)         */
    /*                            + fabs(nrhs[ii].y - nrhs_test[ii].y)         */
    /*                            + fabs(nrhs[ii].z - nrhs_test[ii].z)) ;      */
    /*                                                                         */
    /*          for(jj=0;jj<=nuk;jj++)                                         */
    /*            {                                                            */
    /*              if(fabs(a_nunu[ii][jj] - a_nunu_test[ii][jj]) > 1.0E-6)    */
    /*                lTestCnt++ ;                                             */
    /*              if(fabs(a_nunu[ii][jj] - a_nunu_test[ii][jj]) > dTestMax)  */
    /*                dTestMax = fabs(a_nunu[ii][jj] - a_nunu_test[ii][jj]) ;  */
    /*            }                                                            */
    /*        }                                                                */
    /*        end change temp test code                                        */
                                                         

    /* Now solve via LU decomposition (Crout with partial piv) */

    perm = N_AllocInt1dArray( nuk, &SL );

    if( perm EQ NULL )
        NL_QUIT;

    N_CreateRealMatrix( &rma, nuk, nuk, a_nunu, NL_MT_FULL, nuk );

    /* Failure to solve occurs here if data not ok.                                     */
    /*   One kind of problem: Every control point has to be near sample points.         */
    /*      When no sample points are within the domain of the basis functions of a     */
    /*      surface point, the equation for that point will be all zero and the solver  */
    /*      will fail.                                                                  */
    error = N_RealMatrixLuDecomposePivot( &rma, perm );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_RealMatrixRightForBackPivot( &rma, perm, nrhs, Qw );

    if( error EQ NL_YES )
        NL_OUT;

    /* Load the control points into Sw[][] via the index map */

    for ( ii = 0; ii <= n; ii++ )
      {
        for ( jj = 0; jj <= m; jj++ )
          {
            if( map[ii][jj]GE 0 )
                N_CopyCPt( Qw[map[ii][jj]], &Sw[ii][jj] );
          }
      }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);

} /* end N_FitSrfLstSqDerivs */

#if NLIB_UNUSED

/**********************************************************************/
/* N_FITSRFLSTSQPERIODIC: Periodic least squares surface approximation to random points */
/**********************************************************************/

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This function is analogous to N_FitSrfLstSqDerivs, however, the surface is fit
     using periodic B-splines in either the u- or v-direction,  or both.
     Periodic means that the degree-1 derivatives are continuous  across
     the corresponding boundary; i.e.  u-periodic means that the surface
     is closed in the u-direction and the partial derivatives  with  re-
     spect to u up to and including order=degree-1 are continuous across
     the common v=min and v=max boundary.  An analogous statement  holds
     for v-periodic.  This  routine  computes  a  periodic least squares
     b-spline surface approximation to a random (non-NxM) set of points.
     First partial derivative vectors with  respect to u or v may  (opt-
     ionally) be input at each point,  however,  any such vector that is
     close enough to a periodic boundary to be influenced by  "periodic"
     control points will be ignored.  The  surface  can  be  returned as 
     either clamped or unclamped, however, unclamped should be used only
     for export, as Nlib functions are not designed to process unclamped
     surfaces.  If output surface is initialized to the  NULL  surface,
     memory is allocated locally. Otherwise it is checked if enough mem-
     ory is passed in. A typical calling example is:
 
       NL_POINT      *P;
       NL_VECTOR     *DU, *DV;
       NL_PARAMETER  *uu, *vv;
       NL_REAL       *wts, *uwts, *vwts, *U, *V;
       NL_INDEX      m, n, np, nu, nv, *Iu, *Iv;
       NL_FLAG       uper, vper;
       NL_DEGREE     p, q;
       NL_SURFACE    sur;
       NL_STACKS     SG;

       ...
       N_SrfInitArrays(&sur);
            N_FitSrfLstSqPeriodic(P,wts,uu,vv,np,uper,vper,DU,uwts,nu,Iu,DV,vwts,nv,
              Iv,n,m,p,q,U,V,NL_YES,&sur,&SG);
 
 
   ACCESS:
   
     P    , input  ,  Points to be approximated
     wts  , input  ,  Least squares point weights (wts[i] >= 0  for  all
                      i).  The  larger  wts[i],  the  closer the surface
                      comes to P[i]. wts[i] < 1 lessens the influence of
                      P[i]. If wts[i] = 0, then P[i] is not used.  If no
                      weighting is desired, set wts = NULL
     uu   , input  ,  The u-parameters of the points in P
     vv   , input  ,  The v-parameters of the points in P
     np   , input  ,  The high index of the arrays: P,wts,uu,vv
     uper , input  ,  Flag:
                        = NL_YES : surface periodic in u-direction
                        = NL_NO  : surface not periodic in u-direction
     vper , input  ,  Flag:
                        = NL_YES : surface periodic in v-direction
                        = NL_NO  : surface not periodic in v-direction
                      Note:  The surface should be periodic in at  least
                      one direction; otherwise, use N_FitSrfLstSqDerivs
     DU   , input  ,  First derivative vectors wrt u (optional). DU=NULL
                      means no u-derivatives specified
     uwts , input  ,  u-derivative weights (uwts[i] >= 0 for all i). The
                      larger uwts[i],  the  closer  the surface comes to 
                      assuming DU[i].  uwts[i] < 1 lessens the influence 
                      of DU[i].  If uwts[i] = 0, then DU[i] is not used.  
                      If no weighting is desired, set uwts = NULL
     nu   , input  ,  The high index of the arrays: DU, uwts, Iu (set to
                      -1 if no u-derivatives specified)
     Iu   , input  ,  DU[i] is the u-derivative at (uu[Iu[i]],vv[Iu[i]])
     DV   , input  ,  First derivative vectors wrt v (optional). DV=NULL
                      means no v-derivatives specified
     vwts , input  ,  v-derivative weights (vwts[i] >= 0 for all i). The
                      larger vwts[i],  the  closer  the surface comes to 
                      assuming DV[i].  vwts[i] < 1 lessens the influence 
                      of DV[i].  If vwts[i] = 0, then DV[i] is not used.  
                      If no weighting is desired, set vwts = NULL
     nv   , input  ,  The high index of the arrays: DV, vwts, Iv (set to
                      -1 if no v-derivatives specified)
     Iv   , input  ,  DV[i] is the v-derivative at (uu[Iv[i]],vv[Iv[i]])
     n,m  , input  ,  High indexes of the surface  control  points  (the
                      surface will have (n+1)x(m+1) control points).  
     p,q  , input  ,  Degrees of the surface  (degree in a periodic  di-
                      rection must be greater than 1)  
     U,V  , input  ,  Knots for the surface.  If  U=NULL or V=NULL,  the
                      corresponding knots are  computed in this routine.
                      If given,  the  knots must be clamped  (even for a
                      periodic direction),  and  there  must be n+p+2 u-
                      knots and/or m+q+2 v-knots.  The  values in the uu 
                      and vv arrays  must  correspond to the knot ranges
                      (unless wts[i] = 0)
     clp  , input  ,  Clamped flag:
                       = NL_YES : Output surface should be clamped
                       = NL_NO  : Output surface unclamped (for export only)
     sur  , output ,  Approximating surface
     SG   , input  ,  sur's memory stack

 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_FitSrfLstSqPeriodic
  (NL_POINT   *P,      /* in : sample points,              sized:[np+1] */
   NL_REAL    *wts,    /* in : opt P least square weights, sized:[np+1], NULL to ignore */
   NL_REAL    *uu,     /* in : u param value for P points, sized:[np+1]  */
   NL_REAL    *vv,     /* in : v param value for P points, sized:[np+1]  */
   NL_INDEX    np,     /* in : max index size param */
   NL_FLAG     uper,   /* in : u periodic flag: NL_YES = periodic in u, NL_NO = not */
   NL_FLAG     vper,   /* in : v periodic flag: NL_YES = periodic in v, NL_NO = not */

   NL_VECTOR  *DU,     /* in : opt associated U derivative values, sized:[nu+1], NULL to ignore */
   NL_REAL    *uwts,   /* in : opt DU least square weights,        sized:[nu+1], NULL to ignore */
   NL_INDEX    nu,     /* in : max index size param */
   NL_INDEX   *Iu,     /* in : DU index map, DU[i] is the u-derivative at (uu[Iu[i]],vv[Iu[i]]), sized:[nu+1] */

   NL_VECTOR  *DV,     /* in : opt associated V derivative values, sized:[nv+1], NULL to ignore */              
   NL_REAL    *vwts,   /* in : opt DV least square weights,        sized:[nv+1], NULL to ignore */              
   NL_INDEX    nv,     /* in : max index size param */                                                          
   NL_INDEX   *Iv,     /* in : DV index map, DV[i] is the v-derivative at (uu[Iv[i]],vv[Iv[i]]), sized:[nv+1] */

   NL_INDEX    n,      /* in : output sur ControlPoint max index U */
   NL_INDEX    m,      /* in : output sur ControlPoint max index V */
   NL_DEGREE   p,      /* in : output sur degree U */
   NL_DEGREE   q,      /* in : output sur degree V */
   NL_REAL    *U,      /* in : opt KnotVector U, NULL=function computes likely knotvector from data */
   NL_REAL    *V,      /* in : opt KnotVector V, NULL=function computes likely knotvector from data */
   NL_FLAG     clp,    /* in : clamp flag: NL_YES = output surface should be clamped                                 */
                       /*                           (i.e. multiple end knots) required for subsequent NLib computing */
                       /*                  NL_NO  = not                              */
   NL_SURFACE *sur,    /* out: approximating surface */
   NL_STACKS  *SG )    /* in : sur's memory stack */
{
    NL_PRIVATE NL_STRING rname = _T("N_FitSrfLstSqPeriodic");

    NL_FLAG error = NL_NO, pflg;

    NL_INDEX ii, jj, kk, ** map, me, nuk, uspan = 0, vspan = 0, j1, k1, ** uvsp, ** cind, i2, mc, uspan2, vspan2;
    NL_INDEX i0, j0, i1, ii0, jj0, ii1, jj1, USpan, VSpan, iStart, jStart, m0, l0 ;
    NL_REAL *pBu, *pBv, dBm ; 
    NL_REAL alpha, beta, PtStiffness ; 

    NL_REAL *US, *VS, ** a_nunu, w, u1, u2, v1, v2, ** a_pq, u0, v0, ** ND, ** ND2, ts, te;
    /* obsolete: NL_REAL *col1, *col2, dd ; */

    NL_CPOINT *Qw, ** Sw;

    NL_POINT P1, *rhs, *nrhs;

    NL_KNOTVECTOR knt, *knu, *knv, *knus, *knvs;

    NL_RMATRIX rma;

    /* Conversion from N_FitSrfLstSqDerivs and N_FitCrvWeightedLstSqPeriodic */

    NL_REAL ** MM;

    NL_RMATRIX mm, mt, mit, mi, imit, intwn;

    NL_POINT *INTWS, *VV, *A, B;

    /*****************************************/

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* If not periodic, just call N_FitSrfLstSqDerivs */

    if( uper EQ NL_NO AND vper EQ NL_NO )
    {
        error = N_FitSrfLstSqDerivs( P, wts, uu, vv, np, DU, uwts, nu, Iu, DV, vwts, nv, Iv, n, m, p, q, U, V, sur, SG );
        NL_OUT;
    }

    /* Check for input errors */

    if( n LT 1 OR m LT 1 OR np + 1 LT( n + 1 ) * (m + 1) )
        NL_ERROR( NL_INP_ERR );

    if( n LT p OR m LT q )
        NL_ERROR( NL_INP_ERR );

    if( uper EQ NL_YES AND p LE 1 )
        NL_ERROR( NL_INP_ERR );

    if( vper EQ NL_YES AND q LE 1 )
        NL_ERROR( NL_INP_ERR );

    if( U NEQ NULL )
    {
        N_KnotVectorFromRealArray( &knt, U, n + p + 1 );
        error = N_KnotVectorIsValid( &knt, p, rname );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( V NEQ NULL )
    {
        N_KnotVectorFromRealArray( &knt, V, m + q + 1 );
        error = N_KnotVectorIsValid( &knt, q, rname );

        if( error EQ NL_YES )
            NL_OUT;
    }

    if( nu LT 0 OR DU EQ NULL )
        nu = -1;

    if( nv LT 0 OR DV EQ NULL )
        nv = -1;

    /* Check memory for output surface */

    error = N_SrfSizeArrays( sur, n, m, p, q, n + p + 1, m + q + 1, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Sw, &US, &VS );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Get the surface u- and v-knots */

    u1 = u2 = 0.0;
    v1 = v2 = 0.0;

    pflg = NL_NO;

    if( U NEQ NULL )
    {
        kk = n + p + 1;

        for ( ii = 0; ii <= kk; ii++ )
            US[ii] = U[ii];
        knus = knu;
    }
    else
    {
        knus = NULL;

        if( wts NEQ NULL )
        {
            pflg = NL_YES;
            u1 = 1.0e+20;
            u2 = -u1;

            for ( ii = 0; ii <= np; ii++ )
                if( wts[ii]NEQ 0.0 )
                {
                    if( u1 GT uu[ii] )
                        u1 = uu[ii];

                    if( u2 LT uu[ii] )
                        u2 = uu[ii];
                }
        }
    }

    if( V NEQ NULL )
    {
        kk = m + q + 1;

        for ( ii = 0; ii <= kk; ii++ )
            VS[ii] = V[ii];
        knvs = knv;
    }
    else
    {
        knvs = NULL;

        if( wts NEQ NULL )
        {
            pflg = NL_YES;
            v1 = 1.0e+20;
            v2 = -v1;

            for ( ii = 0; ii <= np; ii++ )
                if( wts[ii]NEQ 0.0 )
                {
                    if( v1 GT vv[ii] )
                        v1 = vv[ii];

                    if( v2 LT vv[ii] )
                        v2 = vv[ii];
                }
        }
    }

    if( knus NEQ knu OR knvs NEQ knv )
    {
        error = N_FitSrfCalcKnotVectors
          ( uu,       /* in : u parameter values, sized:[nn+1] */                                               
            vv,       /* in : v parameter values, sized:[nn+1] */                                               
            np,       /* in : highest u and v index */                                                          
            p,        /* in : approximating surface degree U */                                                 
            q,        /* in : approximating surface degree V */                                                 
            pflg,     /* in : NL_YES = use us,ue,vs,ve values to set max/min u and v knot values */             
                      /*      NL_NO  = find max/min u and v knot values in u and v arrays        */             
            u1,       /* in : min U param value, overridden by any input knu value   */                         
            u2,       /* in : max U param value, overridden by any input knu value   */                         
            v1,       /* in : min V param value, overridden by any input knv value   */                         
            v2,       /* in : max V param value, overridden by any input knv value   */                         
            knus,     /* in : opt starter U knots, if given these knots will be in the output, NULL to ignore */
            knvs,     /* in : opt starter V knots, if given these knots will be in the output, NULL to ignore */
            knu,      /* i/o: sized knot vector whose knot values are to be determined */                       
            knv );    /* i/o: sized knot vector whose knot values are to be determined */                       

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* Allocate memory for the index map */

    map = N_AllocInt2dArray( n, m, &SL );

    if( map EQ NULL )
        NL_QUIT;

    cind = N_AllocInt2dArray( (n + 1) * (m + 1) - 1, 1, &SL );

    if( cind EQ NULL )
        NL_QUIT;

    /* Define the map from Sw(i,j) to svd_a's column index */

    nuk = -1; /* nuk+1 unknowns */

    for ( jj = 0; jj <= m; jj++ )
    {
        for ( ii = 0; ii <= n; ii++ )
        {
            nuk += 1;
            map[ii][jj] = nuk;

            cind[nuk][0] = ii;
            cind[nuk][1] = jj;
        }
    }

    /* Allocate memory for the right hand side and solution vector */

    Qw = N_AllocCPt1dArray( nuk, &SL );

    if( Qw EQ NULL )
        NL_QUIT;

    me = np;
    me += (nu + 1);
    me += (nv + 1);

    rhs = N_AllocPt1dArray( me, &SL );

    if( rhs EQ NULL )
        NL_QUIT;

    /* Now set up the overdetermined system of equations */

    u1 = US[0];
    u2 = US[n + 1];
    v1 = VS[0];
    v2 = VS[m + 1];

    w = 1.0;
    u0 = u1 - 1.0;
    v0 = v1 - 1.0;

    a_pq = N_AllocReal2dArray( me, p + q + 1, &SL );

    if( a_pq EQ NULL )
        NL_QUIT;

    uvsp = N_AllocInt2dArray( me, 1, &SL );

    if( uvsp EQ NULL )
        NL_QUIT;

    me = -1; /* me+1 equations to solve */

    glo_a = a_pq;
    glo_cind = cind;
    glo_uvsp = uvsp;

    /* set up system */

    /* first, the equations for the points */

    for ( ii = 0; ii <= np; ii++ )
    {
        if( uu[ii]LT u1 OR uu[ii]GT u2 )
            continue;

        if( vv[ii]LT v1 OR vv[ii]GT v2 )
            continue;

        if( wts NEQ NULL )
        {
            if( wts[ii]LE 0.0 )
                continue;
            else
            {
                if( wts[ii]EQ 1.0 )
                    w = 1.0;
                else
                    w = sqrt( wts[ii] );
            }
        }

        me += 1;

        if( uu[ii]NEQ u0 )
        {
            u0 = uu[ii];
            error = N_BasisEval( knu, p, uu[ii], NL_LEFT, &a_pq[me][0], &uspan );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] = a_pq[me - 1][jj];
        }

        if( vv[ii]NEQ v0 )
        {
            v0 = vv[ii];
            error = N_BasisEval( knv, q, vv[ii], NL_LEFT, &a_pq[me][p + 1], &vspan );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            for ( jj = 0; jj <= q; jj++ )
                a_pq[me][jj + p + 1] = a_pq[me - 1][jj + p + 1];
        }

        uvsp[me][0] = uspan;
        uvsp[me][1] = vspan;

        N_CopyPt( P[ii], &rhs[me] );

        if( w NEQ 1.0 )
        {
            u0 = u1 - 1.0;

            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] *= w;
            N_ScalePt( w, rhs[me], &rhs[me] );
        }

        for ( jj = 0; jj <= p; jj++ )
        {
            j1 = uspan - p + jj;

            for ( kk = 0; kk <= q; kk++ )
            {
                k1 = vspan - q + kk;

                if( map[j1][k1]LT 0 )
                {
                    N_CPtToPtEuclid( Sw[j1][k1], &P1 );
                    N_VectorBlendPt( -(a_pq[me][jj] * a_pq[me][p + kk + 1]), P1, &rhs[me] );
                }
            }
        }
    }

    /* now the equations for the u-derivatives */

    ii = NL_MAX( p, q );
    ND = N_AllocReal2dArray( ii, ii, &SL );

    if( ND EQ NULL )
        NL_QUIT;

    ts = US[2 * p];
    te = US[n - p];

    w = 1.0;
    u0 = u1 - 1.0;
    v0 = v1 - 1.0;

    for ( ii = 0; ii <= nu; ii++ )
    {
        i2 = Iu[ii];

        if( uu[i2]LT u1 OR uu[i2]GT u2 )
            continue;

        if( vv[i2]LT v1 OR vv[i2]GT v2 )
            continue;

        if( uper EQ NL_YES )
            if( uu[i2]LT ts OR uu[i2]GE te )
                continue;

        if( uwts NEQ NULL )
        {
            if( uwts[ii]LE 0.0 )
                continue;
            else
            {
                if( uwts[ii]EQ 1.0 )
                    w = 1.0;
                else
                    w = sqrt( uwts[ii] );
            }
        }

        me += 1;

        if( uu[i2]NEQ u0 )
        {
            u0 = uu[i2];
            error = N_BasisDerivs( knu, p, uu[i2], NL_LEFT, 1, ND, &uspan );

            if( error EQ NL_YES )
                NL_OUT;

            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] = ND[1][jj];
        }
        else
        {
            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] = a_pq[me - 1][jj];
        }

        if( vv[i2]NEQ v0 )
        {
            v0 = vv[i2];
            error = N_BasisEval( knv, q, vv[i2], NL_LEFT, &a_pq[me][p + 1], &vspan );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            for ( jj = 0; jj <= q; jj++ )
                a_pq[me][jj + p + 1] = a_pq[me - 1][jj + p + 1];
        }

        uvsp[me][0] = uspan;
        uvsp[me][1] = vspan;

        N_CopyPt( DU[ii], &rhs[me] );

        if( w NEQ 1.0 )
        {
            u0 = u1 - 1.0;

            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] *= w;
            N_ScalePt( w, rhs[me], &rhs[me] );
        }
    }

    /* now the equations for the v-derivatives */

    w = 1.0;
    u0 = u1 - 1.0;
    v0 = v1 - 1.0;

    ts = VS[2 * q];
    te = VS[m - q];

    for ( ii = 0; ii <= nv; ii++ )
    {
        i2 = Iv[ii];

        if( uu[i2]LT u1 OR uu[i2]GT u2 )
            continue;

        if( vv[i2]LT v1 OR vv[i2]GT v2 )
            continue;

        if( vper EQ NL_YES )
            if( vv[i2]LT ts OR vv[i2]GE te )
                continue;

        if( vwts NEQ NULL )
        {
            if( vwts[ii]LE 0.0 )
                continue;
            else
            {
                if( vwts[ii]EQ 1.0 )
                    w = 1.0;
                else
                    w = sqrt( vwts[ii] );
            }
        }

        me += 1;

        if( uu[i2]NEQ u0 )
        {
            u0 = uu[i2];
            error = N_BasisEval( knu, p, uu[i2], NL_LEFT, &a_pq[me][0], &uspan );

            if( error EQ NL_YES )
                NL_OUT;
        }
        else
        {
            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] = a_pq[me - 1][jj];
        }

        if( vv[i2]NEQ v0 )
        {
            v0 = vv[i2];
            error = N_BasisDerivs( knv, q, vv[i2], NL_LEFT, 1, ND, &vspan );

            if( error EQ NL_YES )
                NL_OUT;

            for ( jj = 0; jj <= q; jj++ )
                a_pq[me][jj + p + 1] = ND[1][jj];
        }
        else
        {
            for ( jj = 0; jj <= q; jj++ )
                a_pq[me][jj + p + 1] = a_pq[me - 1][jj + p + 1];
        }

        uvsp[me][0] = uspan;
        uvsp[me][1] = vspan;

        N_CopyPt( DV[ii], &rhs[me] );

        if( w NEQ 1.0 )
        {
            u0 = u1 - 1.0;

            for ( jj = 0; jj <= p; jj++ )
                a_pq[me][jj] *= w;
            N_ScalePt( w, rhs[me], &rhs[me] );
        }
    }

    if( me LE nuk )
        NL_ERROR( NL_INP_ERR );

    glo_me = me;
    glo_nu = nuk;
    glo_p = p;
    glo_q = q;

    /*  Set up the periodic constraints  */

    ii = NL_MAX( p, q );
    ND2 = N_AllocReal2dArray( ii, ii, &SL );

    if( ND2 EQ NULL )
        NL_QUIT;

    mc = -1;

    if( uper EQ NL_YES )
        mc += (p * (m + 1));

    if( vper EQ NL_YES )
    {
        if( mc LT 0 )
            mc += q * (n + 1);
        else
            mc += (q * (n - p + 1));
    }

    error = N_SetRealMatrix( &mm, mc, nuk, NL_MT_FULL, nuk, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &mm, &MM );

    for ( ii = 0; ii <= mc; ii++ )
        for ( jj = 0; jj <= nuk; jj++ )
            MM[ii][jj] = 0.0;

    mc = -1;

    if( uper EQ NL_YES )
    {
        error = N_BasisDerivs( knu, p, u1, NL_LEFT, p - 1, ND, &uspan );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_BasisDerivs( knu, p, u2, NL_RIGHT, p - 1, ND2, &uspan2 );

        if( error EQ NL_YES )
            NL_OUT;

        for ( ii = 0; ii <= m; ii++ )
        {
            for ( jj = 0; jj < p; jj++ )
            {
                mc += 1;

                for ( kk = 0; kk <= jj; kk++ )
                {
                    MM[mc][map[kk][ii]] = ND[jj][kk];
                    MM[mc][map[n - kk][ii]] = -ND2[jj][p - kk];
                }
            }
        }
    }

    if( vper EQ NL_YES )
    {
        error = N_BasisDerivs( knv, q, v1, NL_LEFT, q - 1, ND, &vspan );

        if( error EQ NL_YES )
            NL_OUT;
        error = N_BasisDerivs( knv, q, v2, NL_RIGHT, q - 1, ND2, &vspan2 );

        if( error EQ NL_YES )
            NL_OUT;

        if( uper EQ NL_YES )
            k1 = n - p;
        else
            k1 = n;

        for ( ii = 0; ii <= k1; ii++ )
        {
            for ( jj = 0; jj < q; jj++ )
            {
                mc += 1;

                for ( kk = 0; kk <= jj; kk++ )
                {
                    MM[mc][map[ii][kk]] = ND[jj][kk];
                    MM[mc][map[ii][m - kk]] = -ND2[jj][q - kk];
                }
            }
        }
    }

    /* Now set up the system and solve as in Chapter 9, NURBS Book */

    error = N_SetRealMatrix( &rma, nuk, nuk, NL_MT_FULL, nuk, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_GetRealMatrixPtr( &rma, &a_nunu );

    /* Set up Normal Equations and rhs */

    nrhs = N_AllocPt1dArray( nuk, &SL );

    if( nrhs EQ NULL )
        NL_QUIT;

    /* build A and B matrices */

    /* add stiffness terms to add to A matrix */
    alpha       = 1.0;
    beta        = 10.0;
    PtStiffness = 1.0E10;

    /* init A and B to zero - these clears are tuned to memory layout done */
    /* in  N_AllocPt1dArray() and N_AllocReal2dArray()                     */
    N_MemSet(nrhs,0,(nuk + 1) * sizeof( NL_POINT )) ;
    N_MemSet(a_nunu[0], 0, (nuk + 1) * (nuk + 1) * sizeof( NL_REAL )  );

    /* something one might try - if sample data is not complete, then add these terms to stabelize A matrix */
    /*      add stiffness terms to A matrix                          */                    
    /*      alpha       = 1.0 ;                                      */
    /*      beta        = 10.0 ;                                     */
    /*      PtStiffness = 1.0E10 ;                                   */
    /*      N_AddStiffnessToSrfAMatrix(alpha, beta, sur, a_nunu) ;   */

    /* for every data point */
    for(kk=0;kk<=me;kk++)
      {
        /* u and v basis values for kkth data point */
        pBu = a_pq[kk] ;
        pBv = pBu + p + 1 ;

        /* u and v spans for the kkth data point    */
        USpan = uvsp[kk][0] ;
        VSpan = uvsp[kk][1] ;

        iStart = USpan - p;
        jStart = VSpan - q;

        /* for every V Basis function */
        for(j0=0;j0<=q;j0++)
          {
            /* map i0,j0 to global m0 index */
            jj0 = jStart + j0;

            /* for every U Basis function */
            for(i0=0;i0<=p;i0++)
              {
                ii0 = iStart + i0;
                m0 = map[ii0][jj0];
                if ( m0 == -1 )
                { continue; }

                /* Basis function B_m */
                dBm = PtStiffness * pBu[i0] * pBv[j0] ;  /* when using alpha and beta use this line */
                /* dBm = pBu[i0] * pBv[j0] ; */

                /* add terms to B matrix B[m0] += rhs[kk] * B_m(u_k,v_k) */
                N_VectorBlendPt(dBm , rhs[kk], &nrhs[m0] );

                /* add B_m(uk,vk)*B_l(uk,vk) terms to A[m,l] matrix elements */
                for(j1=0;j1<=q;j1++)
                  {
                    jj1 = jStart + j1;

                    for(i1=0;i1<=p;i1++)
                      {
                        ii1 = iStart + i1;

                        /* map i1,j1 to global l0 index */
                        l0 = map[ii1][jj1];
                        if ( l0 == -1 )
                        { continue; }

                        /* add terms to A matrix */
                        a_nunu[m0][l0] +=  dBm * pBu[i1] * pBv[j1] ;
                        
                      } /* end iter every U Basis function */
                  } /* end iter every V Basis function - making A terms */

              } /* end iter every U Basis function */
          } /* end iter every V Basis function - making B and A terms */
      } /* end iter every pos, u-deriv and v-deriv input point */


    /* stablize computation when sample points don't cover whole domain    */
    /* add stiffness terms to A matrix                                     */
    /*      alpha       = 1.0 ;                                            */
    /*      beta        = 10.0 ;                                           */
    /*      PtStiffness = 1.0E10 ; (this term weights the constraint eqns) */
    N_AddStiffnessToSrfAMatrix(alpha, beta, sur, a_nunu, map, Sw, nrhs, &nu) ; 

    /*      allocate memory for columns of the coeff matrix                */
    /*                                                                      */
    /*      col1 = N_AllocReal1dArray( me, &SL );                           */
    /*                                                                      */
    /*      if( col1 EQ NULL )                                              */
    /*          NL_QUIT;                                                    */
    /*      col2 = N_AllocReal1dArray( me, &SL );                           */
    /*                                                                      */
    /*      if( col2 EQ NULL )                                              */
    /*          NL_QUIT;                                                    */
    /*                                                                      */
    /*      for ( ii = 0; ii <= nuk; ii++ )                                 */
    /*      {                                                               */
    /*          error = ST_GetRowsColMatrix3( -1, ii, col1 );               */
    /*                                                                      */
    /*          if( error EQ NL_YES )                                       */
    /*              NL_OUT;                                                 */
    /*                                                                      */
    /*          N_CopyPt( NL_ZERO, &nrhs[ii] );                             */
    /*                                                                      */
    /*          for ( jj = 0; jj <= me; jj++ )                              */
    /*              N_VectorBlendPt( col1[jj], rhs[jj], &nrhs[ii] );        */
    /*                                                                      */
    /*          for ( jj = ii; jj <= nuk; jj++ )                            */
    /*          {                                                           */
    /*              error = ST_GetRowsColMatrix3( -1, jj, col2 );           */
    /*                                                                      */
    /*              if( error EQ NL_YES )                                   */
    /*                  NL_OUT;                                             */
    /*                                                                      */
    /*              dd = 0.0;                                               */
    /*                                                                      */
    /*              for ( kk = 0; kk <= me; kk++ )                          */
    /*                  dd += col1[kk] * col2[kk];                          */
    /*              a_nunu[ii][jj] = dd;                                    */
    /*          }                                                           */
    /*                                                                      */
    /*          for ( jj = ii + 1; jj <= nuk; jj++ )                        */
    /*              a_nunu[jj][ii] = a_nunu[ii][jj];                        */
    /*      }                                                               */

    /* Get matrices to compute Lagrange multipliers */

    N_InitRealMatrix( &intwn );
    error = N_RealMatrixInversePivot( &rma, &intwn, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &mt );
    error = N_RealMatrixTranspose( &mm, &mt, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &mit );
    error = N_RealMatrixMultiply( &intwn, &mt, &mit, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &mi );
    error = N_RealMatrixMultiply( &mm, &mit, &mi, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_InitRealMatrix( &imit );
    error = N_RealMatrixInversePivot( &mi, &imit, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /*  Now solve  */

    INTWS = N_AllocPt1dArray( nuk, &SL );

    if( INTWS EQ NULL )
        NL_QUIT;

    VV = N_AllocPt1dArray( nuk, &SL );

    if( VV EQ NULL )
        NL_QUIT;

    A = N_AllocPt1dArray( mc, &SL );

    if( A EQ NULL )
        NL_QUIT;

    error = N_RealMatrixMultiplyPtArray( &intwn, nrhs, INTWS );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_RealMatrixMultiplyPtArray( &mm, INTWS, VV );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_RealMatrixMultiplyPtArray( &imit, VV, A );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_RealMatrixMultiplyPtArray( &mit, A, VV );

    if( error EQ NL_YES )
        NL_OUT;

    for ( ii = 0; ii <= nuk; ii++ )
    {
        N_Diff2Pts( INTWS[ii], VV[ii], &B );
        N_PtToCPt( B, &Qw[ii] );
    }

    /* Load the control points into Sw[][] via the index map */

    for ( ii = 0; ii <= n; ii++ )
        for ( jj = 0; jj <= m; jj++ )
            if( map[ii][jj]GE 0 )
                N_CopyCPt( Qw[map[ii][jj]], &Sw[ii][jj] );

    /* Unclamp if necessary */

    if( clp EQ NL_NO )
    {
        if( uper EQ NL_YES AND vper EQ NL_NO )
            N_SrfUnclamp( sur, NL_UDIR );

        if( vper EQ NL_YES AND uper EQ NL_NO )
            N_SrfUnclamp( sur, NL_VDIR );

        if( uper EQ NL_YES AND vper EQ NL_YES )
            N_SrfUnclamp( sur, NL_UVDIR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);


} /* end N_FitSrfLstSqPeriodic */

/* --------------------------------------------------------------- */
/*  End of global functions.  Static functions to follow.          */
/* --------------------------------------------------------------- */

#endif // NLIB_UNUSED

/* --------------------------------------------------------------- */
/**   Function to fetch rows, columns and single elements   **/
/**              from the sparse matrix a_pq                **/
/* --------------------------------------------------------------- */
NL_FLAG ST_GetRowsColMatrix1( NL_INDEX i, NL_INDEX j, NL_REAL *item )
{
    NL_PRIVATE NL_STRING rname = _T("ST_GetRowsColMatrix1");

    NL_INDEX usp, vsp, uidx, vidx, ii, usp_p, vsp_q, p1;

    NL_FLAG err = NL_NO;

    /* Check for errors */

    if( i LT 0 AND j LT 0 )
        err = NL_YES;

    if( i GE 0 AND i GT glo_me )
        err = NL_YES;

    if( j GE 0 AND j GT glo_nu )
        err = NL_YES;

    if( err EQ NL_YES )
    {
        N_ErrSet( NL_IND_ERR, rname );
        return (err);
    }

    p1 = glo_p + 1;

    /* Get either:  row, column, or single element */

    if( j LT 0 )
    {                         /* get i-th row */
        usp = glo_uvsp[i][0];
        vsp = glo_uvsp[i][1]; /* u and v spans */

        usp_p = usp - glo_p;
        vsp_q = vsp - glo_q;

        for ( ii = 0; ii <= glo_nu; ii++ )
        {
            uidx = glo_cind[ii][0];

            if( uidx LT usp_p OR uidx GT usp )
            {
                item[ii] = 0.0;
                continue;
            }

            vidx = glo_cind[ii][1];

            if( vidx LT vsp_q OR vidx GT vsp )
            {
                item[ii] = 0.0;
                continue;
            }

            item[ii] = glo_a[i][usp - usp_p] * glo_a[i][vsp - vsp_q + p1];
        }
    }
    else if( i LT 0 )
    { /* get j-th column */
        uidx = glo_cind[j][0] + glo_p;
        vidx = glo_cind[j][1] + glo_q;
        ;

        for ( ii = 0; ii <= glo_me; ii++ )
        {
            usp = uidx - glo_uvsp[ii][0];

            if( usp LT 0 OR usp GT glo_p )
            {
                item[ii] = 0.0;
                continue;
            }

            vsp = vidx - glo_uvsp[ii][1];

            if( vsp LT 0 OR vsp GT glo_q )
            {
                item[ii] = 0.0;
                continue;
            }

            item[ii] = glo_a[ii][usp] * glo_a[ii][vsp + p1];
        }
    }
    else
    { /* get (i,j)-th element */
        uidx = glo_cind[j][0] + glo_p;
        usp = uidx - glo_uvsp[i][0];

        if( usp LT 0 OR usp GT glo_p )
        {
            *item = 0.0;
            return (NL_NO);
        }

        vidx = glo_cind[j][1] + glo_q;
        ;
        vsp = vidx - glo_uvsp[i][1];

        if( vsp LT 0 OR vsp GT glo_q )
        {
            *item = 0.0;
            return (NL_NO);
        }

        *item = glo_a[i][usp] * glo_a[i][vsp + p1];
    }

    return (NL_NO);


} /* end ST_GetRowsColMatrix1 */


/* --------------------------------------------------------------- */                    
/* --------------------------------------------------------------- */                    

NL_FLAG ST_findStep
  ( NL_SURFACE *srfPtr, 
    NL_FLAG fixEdges_u, 
    NL_FLAG fixEdges_v, 
    NL_INDEX numDataPoints, 
    NL_POINT xPts [], 
    NL_REAL xPrms_u [], /* must be initialized to previous-step values */
    NL_REAL xPrms_v [], 
    NL_REAL alpha, 
    NL_VECTOR ** ctrlPtMoves,                                                                                  /* Allocated in caller */
    NL_REAL *errorDist, 
    NL_REAL *errorSD, 
    NL_REAL allErrors[] )
{
    NL_PRIVATE NL_STRING rname = _T("findStep");

    NL_REAL thisTerm;

    NL_INDEX i, j, k;
    NL_INDEX i1, i2, j1, j2;
    NL_INDEX rowPtIdx, colPtIdx; /* For indexing Mat "pointwise": bands of xyz. */
    NL_INDEX rowPtIdxU, colPtIdxU;
    NL_INDEX rowPtIdxV, colPtIdxV;
    NL_INDEX rowCoord, colCoord; /* For indexing within an xyz band. */

    /* SDM terms */
    NL_REAL d_k, rho1, rho2;
    NL_REAL sdTerm1, sdTerm2; /*  the Squared Distance terms,  d / ( d - rho ) */
    NL_REAL dot_e1, dot_e2, dot_e3;
    NL_REAL kappa1, kappa2;
    NL_REAL bMatTerm;

    NL_SURFJET sJet; /* data Point On Surface */

    NL_BOOLEAN retval;
    NL_VECTOR ptDiff;
    NL_VECTOR vec_e1, vec_e2, vec_e3;
    NL_REAL real_e1[3], real_e2[3], real_e3[3];

    NL_INDEX spanU, spanV;
    NL_INDEX basisIdx_u1, basisIdx_v1, basisIdx_u2, basisIdx_v2;
    NL_REAL *basisFcns_u;
    NL_REAL *basisFcns_v;
    NL_KNOTVECTOR knotVecU, knotVecV;

    NL_RMATRIX rMat; /* The matrices */
    NL_REAL ** AMat;
    NL_REAL *bMat;

    NL_INDEX *idxArray; /* Pivoting array for matrix solvers */

    NL_RMATRIX srfMat;  /* to help with the AMat terms: explanation below. */
    NL_REAL ** SMat;

    NL_FLAG error;

    /* surface data */
    NL_INDEX numCtrlPtsU, numCtrlPtsV, nKtsU, nKtsV;
    NL_DEGREE degU, degV;
    NL_CPOINT ** ctrlPts;
    NL_REAL *knotsU, *knotsV;

    NL_INDEX numVarPtsU, numVarPtsV; /* perhaps subsets of numCtrlPtsU, numCtrlPtsV */
    NL_INDEX numVarPts, numVars;

    NL_INDEX ctrlPtIdxU, ctrlPtIdxV; /* For indexing into the control point array*/
    NL_INDEX numSpansU, numSpansV;
    NL_VECTOR BVecPt;

    /* For alpha correction: */
    NL_INDEX ktIdxU, ktIdxV;
    NL_REAL alphaMat[16][16];

    NL_STACKS localStacks;

    /* Init */
    N_InitNurbs( &localStacks );

    /* get surface data */
    N_SrfGetCPtsDegreesAndKnots( srfPtr, &numCtrlPtsU, &numCtrlPtsV, &ctrlPts, &degU, &degV, &nKtsU, &nKtsV, &knotsU, &knotsV );

    numCtrlPtsU++; /* Make these actual counts, not "highest index". */
    numCtrlPtsV++;
    nKtsU++;
    nKtsV++;

    /* For later: */
    knotVecU.m = nKtsU - 1;
    knotVecU.U = knotsU;
    knotVecV.m = nKtsV - 1;
    knotVecV.U = knotsV;

    numVarPtsU = numCtrlPtsU;

    if( fixEdges_v >= 1 )
        numVarPtsU -= 2;

    if( fixEdges_v >= 3 )
        numVarPtsU -= 2; /* Fix two edge rows */

    numVarPtsV = numCtrlPtsV;

    if( fixEdges_u >= 1 )
        numVarPtsV -= 2;

    if( fixEdges_u >= 3 )
        numVarPtsV -= 2;

    N_surfJetInit( &sJet, srfPtr );

    basisFcns_u = N_AllocReal1dArray( degU, &localStacks );

    if( basisFcns_u == NULL )
        NL_ERROR( NL_MEM_ERR );
    basisFcns_v = N_AllocReal1dArray( degV, &localStacks );

    if( basisFcns_v == NULL )
        NL_ERROR( NL_MEM_ERR );

    /* Set up the A and b matrices, and init them to zero. */

    numVarPts = numVarPtsU * numVarPtsV;
    numVars = 3 * numVarPts;

    error = N_SetRealMatrix( &rMat, numVars - 1, numVars - 1, NL_MT_FULL, numVars - 1, &localStacks );

    if( error != NL_NO )
        NL_OUT;

    N_GetRealMatrixPtr( &rMat, &AMat );

    bMat = N_AllocReal1dArray( numVars - 1, &localStacks );

    if( bMat == NULL )
        NL_ERROR( NL_MEM_ERR );

    for ( i = 0; i < numVars; i++ )
    {
        bMat[i] = 0;

        for ( j = 0; j < numVars; j++ )
            AMat[i][j] = 0;
    }

    /* A local 3x3 array; see "Setting up the A-matrix" at end of this file. */
    error = N_SetRealMatrix( &srfMat, 2, 2, NL_MT_FULL, 2, &localStacks );
    N_GetRealMatrixPtr( &srfMat, &SMat );

    /* Now the loop over the data points. */

    *errorDist = *errorSD = 0;

    for ( k = 0; k < numDataPoints; k++ )
    {
        /* find foot on the surface for this data point */

        /* the input values should be a good guess */
        N_surfJetSetParam( &sJet, xPrms_u[k], xPrms_v[k] );
        retval = N_surfJetRelax( &sJet, &( xPts[k] ) );

        if( retval != NL_TRUE )
            continue; /* for now, just skip this point. */

        /* For next iteration (next call to this routine ): */
        xPrms_u[k] = sJet.u;
        xPrms_v[k] = sJet.v;

        /* Find d, rho1 and rho2 at this data point. */

        N_Diff2Pts( xPts[k], *( N_surfJetPos( &sJet ) ), &ptDiff );

        N_surfJetPrinCurvature( &sJet, &kappa1, &vec_e1, &kappa2, &vec_e2 );

        /* We're not going to worry about the relative signs of d
        * vs. rho1 and rho2: In one of the papers, "Geometry of the
        * squared distance function to curves and surfaces" by Pottmann
        * and Hofer (T.R. No. 90 at TU Wien, Jan 2002), on pp. 8, 9 and 10,
        * they say that in practice, one can just take the absolute
        * values of d_k and rho, and make everything positive.
        * For curves (where we tried both), this does indeed work better
        * than setting d_k to zero if it has the same sign as rho, as
        * suggested in the Wang-Pottmann-Liu paper.  So instead of all
        * the sign wrangling, we just do make everything positive, and add.
        */

        N_VectorMagnitudeRef( &ptDiff, &d_k ); /* vector magnitude */

        /* These are already unit: */
        /* N_VectorMagnitudeRef( &vec_e1, &kappa1 ); */
        /* N_VectorMagnitudeRef( &vec_e2, &kappa2 ); */
        kappa1 = fabs( kappa1 );
        kappa2 = fabs( kappa2 );

        *errorDist += d_k;

        if( allErrors != NULL )
            allErrors[k] = d_k;

        /* Get unit normal.  In the papers, this is e3, local z-axis. */
        N_surfJetNormal( &sJet, &vec_e3 );

        sdTerm1 = sdTerm2 = 0;

        /*  (The actual value of the tolerance term used here doesn't mean
        *  anything mathematically in this case, just avoid a zero-divide.)
        */
        if( d_k > 0 && kappa1 > NL_ZDTL )
        {
            rho1 = 1 / kappa1;
            sdTerm1 = d_k / (d_k + rho1);
        }

        if( d_k > 0 && kappa2 > NL_ZDTL )
        {
            rho2 = 1 / kappa2;
            sdTerm2 = d_k / (d_k + rho2);
        }

        /* Here's where their method actually happens.
        * (Note: leaving off a factor of 2.)
        *
        * Their x1 is (C-X+D) <dot> vec_e1, x2 is (C-X+D) <dot> vec_e2,
        * and x3 is (C-X+D) <dot> normal.
        * (Note C-X is ptDiff, backwards.)
        *
        * We calculate ptDiff <dot> unit[vec_e1, vec_e2, vec_e3] as the
        * part with no variables in it: no D, so it's for the B-vector.
        * Those are called dot_e1, dot_e2, dot_e3.
        */
        N_VectorDotRef( &ptDiff, &vec_e1, &dot_e1 ); /* dot product */
        N_VectorDotRef( &ptDiff, &vec_e2, &dot_e2 );
        N_VectorDotRef( &ptDiff, &vec_e3, &dot_e3 );

        /* because ptDiff = X-P, they use P-X. */
        dot_e1 = -dot_e1;
        dot_e2 = -dot_e2;
        dot_e3 = -dot_e3;

        /* cumulative error: figure their error term */
        *errorSD += sdTerm1 * dot_e1 * dot_e1 + sdTerm2 * dot_e2 * dot_e2 + dot_e3 * dot_e3;

        /* NOTE: see comment "Setting up the A-matrix" at end of file. */

        SMat[0][0] = sdTerm1 * vec_e1.x * vec_e1.x + sdTerm2 * vec_e2.x * vec_e2.x + vec_e3.x * vec_e3.x;
        SMat[0][1] = sdTerm1 * vec_e1.x * vec_e1.y + sdTerm2 * vec_e2.x * vec_e2.y + vec_e3.x * vec_e3.y;
        SMat[0][2] = sdTerm1 * vec_e1.x * vec_e1.z + sdTerm2 * vec_e2.x * vec_e2.z + vec_e3.x * vec_e3.z;
        SMat[1][0] = SMat[0][1];
        SMat[1][1] = sdTerm1 * vec_e1.y * vec_e1.y + sdTerm2 * vec_e2.y * vec_e2.y + vec_e3.y * vec_e3.y;
        SMat[1][2] = sdTerm1 * vec_e1.y * vec_e1.z + sdTerm2 * vec_e2.y * vec_e2.z + vec_e3.y * vec_e3.z;
        SMat[2][0] = SMat[0][2];
        SMat[2][1] = SMat[1][2];
        SMat[2][2] = sdTerm1 * vec_e1.z * vec_e1.z + sdTerm2 * vec_e2.z * vec_e2.z + vec_e3.z * vec_e3.z;

        /*  (Say 3-dimensional cubic, for this discussion.)
        *  We're filling in four chunks of 12 in the B vector,
        *  and (four by four) 12 x 12 squares within AMat (see comments
        *  "Setting up the A-Matrix" and "The Loops" at end of this file).
    */

        /* Can't index ( [i] ) into VECTORs, so do this: */
        real_e1[0] = vec_e1.x;
        real_e2[0] = vec_e2.x;
        real_e3[0] = vec_e3.x;
        real_e1[1] = vec_e1.y;
        real_e2[1] = vec_e2.y;
        real_e3[1] = vec_e3.y;
        real_e1[2] = vec_e1.z;
        real_e2[2] = vec_e2.z;
        real_e3[2] = vec_e3.z;

        /*  Get the basis functions for the surface at this location.  */
        N_BasisEval( &knotVecU, degU, sJet.u, NL_LEFT, basisFcns_u, &spanU );
        N_BasisEval( &knotVecV, degV, sJet.v, NL_LEFT, basisFcns_v, &spanV );

        /* Note: only (deg+1) of the basis fcns are non-zero.  */
        /* Indexing here is taken from N_CrvEval.  */

        /* First outer loop, on each u-row of control points */
        /* This is one chunk of (4x3) rows in the A-Matrix.  */

        for ( i1 = spanU - degU; i1 <= spanU; i1++ )
        {
            /* Adjust indexing for leaving out edge v-columns */
            /* NOTE: see note "Matrices: Fixing the Edge Points" at end of file. */

            rowPtIdxU = i1;

            basisIdx_u1 = i1 - spanU + degU;

            if( fixEdges_v >= 1 )
            {
                if( spanU == degU && basisIdx_u1 == 0 )
                {
                    /* first span, first basis fcn: means first v-col of ctrl points. */
                    continue;
                }

                if( spanU == nKtsU - degU - 2 && basisIdx_u1 == degU )
                {
                    /* last span, last basis fcn: means last row of ctrl points. */
                    continue;
                }

                /*  shift down one. */
                rowPtIdxU--;
            }

            if( fixEdges_v >= 3 ) /* Also check 1st rows in from ends */
            {
                if( (spanU == degU && basisIdx_u1 == 1) || (spanU == degU + 1 && basisIdx_u1 == 0) )
                {
                    /* means second row. */
                    continue;
                }

                if( (spanU == nKtsU - degU - 2 && basisIdx_u1 == degU - 1) || (spanU == nKtsU - degU - 3 && basisIdx_u1 == degU) )
                {
                    /* next to last row. */
                    continue;
                }

                /*  shift down another one. */
                rowPtIdxU--;
            }

            /* Second outer loop, over the v-control points in this u-row */
            /* This is each (3-d) row all the way across the A-Matrix.    */

            for ( i2 = spanV - degV; i2 <= spanV; i2++ )
            {
                /* Each iteration here is a single control point, hence a single
                ** (3-d) entry in the B-vector, and a whole row in the A-Matrix
                ** for its interation with all of the other involved control points. */

                /* Adjust indexing for leaving out edge u-rows */

                rowPtIdxV = i2;

                basisIdx_v1 = i2 - spanV + degV;

                if( fixEdges_u >= 1 )
                {
                    if( spanV == degV && basisIdx_v1 == 0 )
                    {
                        /* first span, first basis fcn: means first ctrl point in row. */
                        continue;
                    }

                    if( spanV == nKtsV - degV - 2 && basisIdx_v1 == degV )
                    {
                        /* last span, last basis fcn: means last ctrl point in row. */
                        continue;
                    }

                    /*  shift down one. */
                    rowPtIdxV--;
                }

                if( fixEdges_u >= 3 ) /* Also check 1st rows in from ends */
                {
                    if( (spanV == degV && basisIdx_v1 == 1) || (spanV == degV + 1 && basisIdx_v1 == 0) )
                    {
                        /* means second point. */
                        continue;
                    }

                    if( (spanV == nKtsV - degV - 2 && basisIdx_v1 == degV - 1) || (spanV == nKtsV - degV - 3 && basisIdx_v1 == degV) )
                    {
                        /* next to last point. */
                        continue;
                    }

                    /*  shift down another one. */
                    rowPtIdxV--;
                }

                /* Inner uv loop: we're at this point for each control-point move
                * that is affected by this data point (4x4 for cubics).
                * Here we will fill in one 3d entry in the B-vector, and one
                * 3d row in the A matrix, which is four chunks of four 3x3 entries.
                * The outermost loop (starting here) is on the three dimensions.
                */

                /* Moving across the row [rowPtIdx*dimension + rowCoord],
                * which is three rows, x, y and z.
                */

                rowPtIdx = rowPtIdxU * numVarPtsV + rowPtIdxV;

                for ( rowCoord = 0; rowCoord < 3; rowCoord++ )
                {
                    /* B-vector term  ( RHS ) */

                    bMatTerm = basisFcns_u[basisIdx_u1] * basisFcns_v[basisIdx_v1] * (sdTerm1 * dot_e1 * real_e1[rowCoord] + sdTerm2 * dot_e2 * real_e2[rowCoord] + dot_e3 * real_e3[rowCoord]);

                    /* Note: Use -= for b vec, because term is negative. */
                    bMat[rowPtIdx * 3 + rowCoord] -= bMatTerm;

                    /* If the end points are fixed, then those point moves are
                    * no longer variables, so their terms end up on the RHS
                    * instead of in AMat.  But, the term to be added here
                    * would be Bi*Bj * N.x * ( D0 dot N ) etc.
                    * But D0 is zero, so just leaving it out of AMat is all we do.
                    *  if ( fixEdges )
                    *  
                    {
                    *    endPointTerm = ....
                    *    bMat[rowPtIdx].x -= endPointTerm;
                    *  
                    }
                    */

                    /* Row of A-matrix terms: index through columns in row. */
                    /* This next double loop over j1 and j2 is the same as the */
                    /* outer double loop over i1 and i2.   */

                    /* First inner loop, on each u-row of control points */

                    for ( j1 = spanU - degU; j1 <= spanU; j1++ )
                    {
                        colPtIdxU = j1;

                        basisIdx_u2 = j1 - spanU + degU;

                        if( fixEdges_v >= 1 )
                        {
                            if( spanU == degU && basisIdx_u2 == 0 )
                                continue; /* first row */

                            if( spanU == nKtsU - degU - 2 && basisIdx_u2 == degU )
                                continue; /* last row */

                            colPtIdxU--;  /* shift down one */
                        }

                        if( fixEdges_v >= 3 )
                        {
                            if( (spanU == degU && basisIdx_u2 == 1) || (spanU == degU + 1 && basisIdx_u2 == 0) )
                            {
                                continue; /* second row. */
                            }

                            if( (spanU == nKtsU - degU - 2 && basisIdx_u2 == degU - 1) || (spanU == nKtsU - degU - 3 && basisIdx_u2 == degU) )
                            {
                                continue; /* next to last row. */
                            }

                            /*  shift down another one. */
                            colPtIdxU--;
                        }

                        /* Second inner loop, over the v-control points in this u-row */
                        for ( j2 = spanV - degV; j2 <= spanV; j2++ )
                        {
                            /* Adjust indexing for leaving out edge cols */

                            colPtIdxV = j2;

                            basisIdx_v2 = j2 - spanV + degV;

                            if( fixEdges_u >= 1 )
                            {
                                if( spanV == degV && basisIdx_v2 == 0 )
                                {
                                    continue; /* first ctrl point in row. */
                                }

                                if( spanV == nKtsV - degV - 2 && basisIdx_v2 == degV )
                                {
                                    continue; /* last ctrl point in row. */
                                }

                                /*  shift down one. */
                                colPtIdxV--;
                            }

                            if( fixEdges_u >= 3 ) /* Also check 1st rows in from ends */
                            {
                                if( (spanV == degV && basisIdx_v2 == 1) || (spanV == degV + 1 && basisIdx_v2 == 0) )
                                {
                                    /* means second row. */
                                    continue; /* second ctrl point in row. */
                                }

                                if( (spanV == nKtsV - degV - 2 && basisIdx_v2 == degV - 1) || (spanV == nKtsV - degV - 3 && basisIdx_v2 == degV) )
                                {
                                    /* next to last row. */
                                    continue; /* next to last ctrl point in row. */
                                }

                                /*  shift down another one. */
                                colPtIdxV--;
                            }

                            colPtIdx = colPtIdxU * numVarPtsV + colPtIdxV;

                            /* Index through the dimensions, for the three columns. */
                            for ( colCoord = 0; colCoord < 3; colCoord++ )
                            {
                                thisTerm = basisFcns_u[basisIdx_u1] * basisFcns_v[basisIdx_v1] * basisFcns_u[basisIdx_u2] * basisFcns_v[basisIdx_v2] * SMat[rowCoord][colCoord];

                                AMat[rowPtIdx * 3 + rowCoord][colPtIdx * 3 + colCoord] += thisTerm;
                            }
                        }
                    }
                }
            }
        }
    } /* End of loop over all data points (k), to set up matrices. */

    /*  Now add in the regularization terms.                    */
    /* NOTE: see comment "Regularization Terms" at end of file. */

    if( alpha > 0 && degU == 3 && degV == 3 ) /* This is specific to cubics. */
    {

        /* Minimize the functional Suu^2 + 2*Suv^2 + Svv^2.
        ** Integrate over each span.
        ** NOTE: see comment "Integrating Arc Length" at end of file.
        */

        /* Values of all the necessary basis functions at the four Gauss abscissae: */
        NL_REAL N10u[4], N11u[4];
        NL_REAL N10v[4], N11v[4];
        NL_REAL N20u[4], N21u[4], N22u[4];
        NL_REAL N20v[4], N21v[4], N22v[4];
        NL_REAL N30u[4], N31u[4], N32u[4], N33u[4];
        NL_REAL N30v[4], N31v[4], N32v[4], N33v[4];

        /* Coefficients of products of basis functions,
        * evaluated at the Gauss points: */
        NL_REAL coef_uu[16], coef_uv[16], coef_vv[16];

        /* The loop structure is pretty complex.  Summary:
        
          FOR spanU
          ctrlPtIdxU =
          knotIdxU =
          
            FOR spanV
            ctrlPtIdxV =
            knotIdxV =
            
              eval basis fcns at Gauss abscissae
              set up 16x16 matrix, double loop on 4x4 Gauss pts
              
                < Add 16x16 mat into main mat: double double loops >
                FOR ii: uPts in span (4)
                matRowIdxU = ctrlPtIdxU + ii - ( depends on whether u-edges fixed )
                matRowIdxU *= numVarPtsV;
                
                  FOR iii: vPts in span in u-row (4)
                  matRowIdxV = ctrlPtIdxV + iii - ( depends on whether v-edges fixed )
                  matRowPtIdx = matRowIdxU + matRowIdxV;
                  < this is one row in main matrix >
                  
                    B-Vector here.
                    
                      FOR jj: uPts in span (4)
                      matColIdxU = ctrlPtIdxU + jj - ( depends on u-edges fixed )
                      matColIdxU *= numVarPtsV;
                      
                        FOR jjj: vPts in span in u-row (4)
                        matColIdxV = ctrlPtIdxV + jjj - ( depends on v-edges fixed )
                        matColPtIdx = matColIdxU + matColIdxV;
                        < this is one 3x3 point entry in main matrix >
        */

        NL_INDEX ii, iii, jj, jjj, idx;
        NL_INDEX rowIdxU, rowIdxV, colIdxU, colIdxV;
        NL_INDEX coord;

        NL_REAL uPrm, vPrm;
        NL_REAL delU, delV, midU, midV;

        NL_REAL term, wt;

        /* The four-point Gaussian integration numbers.
        * The abscissae (on [-1,1]) are
        *  (+-) sqrt( 525 (+-) 70 * sqrt( 30 ) ) / 35;
        * and the weights are
        *  0.5 (+-) sqrt( 5 / 216 );
        */

        NL_INDEX gaussIdx;
        NL_REAL gaussAbsc[4], wts[4];

        gaussAbsc[3] = 0.8611363115940526;
        gaussAbsc[2] = 0.3399810435848563;
        gaussAbsc[1] = -gaussAbsc[2];
        gaussAbsc[0] = -gaussAbsc[3];

        wts[0] = wts[3] = 0.3478548451374539;
        wts[1] = wts[2] = 0.6521451548625461;

        /* Loop over all polynomial spans in the surface */

        numSpansU = numCtrlPtsU - degU;
        numSpansV = numCtrlPtsV - degV;

        for ( spanU = 0; spanU < numSpansU; spanU++ )
        {
            ctrlPtIdxU = spanU;

            ktIdxU = spanU + degU;

            delU = knotsU[ktIdxU + 1] - knotsU[ktIdxU];
            midU = (knotsU[ktIdxU + 1] + knotsU[ktIdxU]) / 2;

            for ( spanV = 0; spanV < numSpansV; spanV++ )
            {
                /* Here we're in one 4x4 span of control points. */

                ctrlPtIdxV = spanV;

                /* For the first deriv, the 4x4 is different for each span. */
                ktIdxV = spanV + degV;

                delV = knotsV[ktIdxV + 1] - knotsV[ktIdxV];
                midV = (knotsV[ktIdxV + 1] + knotsV[ktIdxV]) / 2;

                /* First eval the u and v basis functions at the four Gauss abscissae */
                /* For nonuniform B-Splines, these are different for each span. */

                for ( gaussIdx = 0; gaussIdx < 4; gaussIdx++ )
                {
                    uPrm = midU + gaussAbsc[gaussIdx] * delU / 2;
                    N10u[gaussIdx] = (knotsU[ktIdxU + 1] - uPrm) / delU;
                    N11u[gaussIdx] = (uPrm - knotsU[ktIdxU]) / delU;

                    /* The easiest way to calc the B-Spline basis functions  */
                    /* is just to use the recursion-relation definition:     */

                    N20u[gaussIdx] = N10u[gaussIdx] * (knotsU[ktIdxU + 1] - uPrm) / (knotsU[ktIdxU + 1] - knotsU[ktIdxU - 1]);

                    N21u[gaussIdx] = N10u[gaussIdx] * (uPrm - knotsU[ktIdxU - 1]) / (knotsU[ktIdxU + 1] - knotsU[ktIdxU - 1]) + N11u[gaussIdx] * (knotsU[ktIdxU + 2] - uPrm) / (knotsU[ktIdxU + 2] - knotsU[ktIdxU]);

                    N22u[gaussIdx] = N11u[gaussIdx] * (uPrm - knotsU[ktIdxU]) / (knotsU[ktIdxU + 2] - knotsU[ktIdxU]);

                    N30u[gaussIdx] = N20u[gaussIdx] * (knotsU[ktIdxU + 1] - uPrm) / (knotsU[ktIdxU + 1] - knotsU[ktIdxU - 2]);

                    N31u[gaussIdx] = N20u[gaussIdx] * (uPrm - knotsU[ktIdxU - 2]) / (knotsU[ktIdxU + 1] - knotsU[ktIdxU - 2]) + N21u[gaussIdx] * (knotsU[ktIdxU + 2] - uPrm) / (knotsU[ktIdxU + 2] - knotsU[ktIdxU - 1]);

                    N32u[gaussIdx] = N21u[gaussIdx] * (uPrm - knotsU[ktIdxU - 1]) / (knotsU[ktIdxU + 2] - knotsU[ktIdxU - 1]) + N22u[gaussIdx] * (knotsU[ktIdxU + 3] - uPrm) / (knotsU[ktIdxU + 3] - knotsU[ktIdxU]);

                    N33u[gaussIdx] = N22u[gaussIdx] * (uPrm - knotsU[ktIdxU]) / (knotsU[ktIdxU + 3] - knotsU[ktIdxU]);

                    /* Then the same for v  */
                    vPrm = midV + gaussAbsc[gaussIdx] * delV / 2;
                    N10v[gaussIdx] = (knotsV[ktIdxV + 1] - vPrm) / delV;
                    N11v[gaussIdx] = (vPrm - knotsV[ktIdxV]) / delV;

                    N20v[gaussIdx] = N10v[gaussIdx] * (knotsV[ktIdxV + 1] - vPrm) / (knotsV[ktIdxV + 1] - knotsV[ktIdxV - 1]);

                    N21v[gaussIdx] = N10v[gaussIdx] * (vPrm - knotsV[ktIdxV - 1]) / (knotsV[ktIdxV + 1] - knotsV[ktIdxV - 1]) + N11v[gaussIdx] * (knotsV[ktIdxV + 2] - vPrm) / (knotsV[ktIdxV + 2] - knotsV[ktIdxV]);

                    N22v[gaussIdx] = N11v[gaussIdx] * (vPrm - knotsV[ktIdxV]) / (knotsV[ktIdxV + 2] - knotsV[ktIdxV]);

                    N30v[gaussIdx] = N20v[gaussIdx] * (knotsV[ktIdxV + 1] - vPrm) / (knotsV[ktIdxV + 1] - knotsV[ktIdxV - 2]);

                    N31v[gaussIdx] = N20v[gaussIdx] * (vPrm - knotsV[ktIdxV - 2]) / (knotsV[ktIdxV + 1] - knotsV[ktIdxV - 2]) + N21v[gaussIdx] * (knotsV[ktIdxV + 2] - vPrm) / (knotsV[ktIdxV + 2] - knotsV[ktIdxV - 1]);

                    N32v[gaussIdx] = N21v[gaussIdx] * (vPrm - knotsV[ktIdxV - 1]) / (knotsV[ktIdxV + 2] - knotsV[ktIdxV - 1]) + N22v[gaussIdx] * (knotsV[ktIdxV + 3] - vPrm) / (knotsV[ktIdxV + 3] - knotsV[ktIdxV]);

                    N33v[gaussIdx] = N22v[gaussIdx] * (vPrm - knotsV[ktIdxV]) / (knotsV[ktIdxV + 3] - knotsV[ktIdxV]);
                }

/*
* Note: alphaMat depends only on the knot vectors.
* In this application, knot vectors don't change between iterations.
* Therefore, it would be possible to calculate them all ahead of time,
* presumably in the caller.  If we find that this runs too slowly,
* we might consider doing that.  Lots of storage though ... well, not
* really, 256 doubles is 1 kb per span.
*
* In practice, this takes no time at all compared to the loop over the
* data points, which includes a surface inversion for each point.
* At least, that's true with 81 control points and over 3000 data points.
*/

/* Just to make indexing a little clearer: */
#define p00 0
#define p01 1
#define p02 2
#define p03 3
#define p10 4
#define p11 5
#define p12 6
#define p13 7
#define p20 8
#define p21 9
#define p22 10
#define p23 11
#define p30 12
#define p31 13
#define p32 14
#define p33 15

                /* Integrete: double loop over the 4x4 Gauss points */

                for ( i = 0; i < 16; i++ )
                    for ( j = 0; j < 16; j++ )
                        alphaMat[i][j] = 0;

                for ( i = 0; i < 4; i++ )
                {
                    for ( j = 0; j < 4; j++ )
                    {
                        /* Here we're at one of the 4x4 Gauss parameters,
                        where we're calculating and accumulating the
                        function values, which are products of the B-spline
                        basis functions, evaluated there.
                            */

                        coef_uu[p00] = N10u[i] * N30v[j];
                        coef_uu[p01] = N10u[i] * N31v[j];
                        coef_uu[p02] = N10u[i] * N32v[j];
                        coef_uu[p03] = N10u[i] * N33v[j];
                        coef_uu[p10] = N11u[i] * N30v[j] - 2 * N10u[i] * N30v[j];
                        coef_uu[p11] = N11u[i] * N31v[j] - 2 * N10u[i] * N31v[j];
                        coef_uu[p12] = N11u[i] * N32v[j] - 2 * N10u[i] * N32v[j];
                        coef_uu[p13] = N11u[i] * N33v[j] - 2 * N10u[i] * N33v[j];
                        coef_uu[p20] = N10u[i] * N30v[j] - 2 * N11u[i] * N30v[j];
                        coef_uu[p21] = N10u[i] * N31v[j] - 2 * N11u[i] * N31v[j];
                        coef_uu[p22] = N10u[i] * N32v[j] - 2 * N11u[i] * N32v[j];
                        coef_uu[p23] = N10u[i] * N33v[j] - 2 * N11u[i] * N33v[j];
                        coef_uu[p30] = N11u[i] * N30v[j];
                        coef_uu[p31] = N11u[i] * N31v[j];
                        coef_uu[p32] = N11u[i] * N32v[j];
                        coef_uu[p33] = N11u[i] * N33v[j];

                        coef_uv[p00] = N20u[i] * N20v[j];
                        coef_uv[p01] = N20u[i] * N21v[j] - N20u[i] * N20v[j];
                        coef_uv[p02] = N20u[i] * N22v[j] - N20u[i] * N21v[j];
                        coef_uv[p03] = -N20u[i] * N22v[j];
                        coef_uv[p10] = N21u[i] * N20v[j] - N20u[i] * N20v[j];
                        coef_uv[p11] = N20u[i] * N20v[j] - N20u[i] * N21v[j] - N21u[i] * N20v[j] + N21u[i] * N21v[j];
                        coef_uv[p12] = N20u[i] * N21v[j] - N20u[i] * N22v[j] - N21u[i] * N21v[j] + N21u[i] * N22v[j];
                        coef_uv[p13] = N20u[i] * N22v[j] - N21u[i] * N22v[j];
                        coef_uv[p20] = N22u[i] * N20v[j] - N21u[i] * N20v[j];
                        coef_uv[p21] = N21u[i] * N20v[j] - N21u[i] * N21v[j] - N22u[i] * N20v[j] + N22u[i] * N21v[j];
                        coef_uv[p22] = N21u[i] * N21v[j] - N21u[i] * N22v[j] - N22u[i] * N21v[j] + N22u[i] * N22v[j];
                        coef_uv[p23] = N21u[i] * N22v[j] - N22u[i] * N22v[j];
                        coef_uv[p30] = -N22u[i] * N20v[j];
                        coef_uv[p31] = N22u[i] * N20v[j] - N22u[i] * N21v[j];
                        coef_uv[p32] = N22u[i] * N21v[j] - N22u[i] * N22v[j];
                        coef_uv[p33] = N22u[i] * N22v[j];

                        coef_vv[p00] = N30u[i] * N10v[j];
                        coef_vv[p01] = N30u[i] * N11v[j] - 2 * N30u[i] * N10v[j];
                        coef_vv[p02] = N30u[i] * N10v[j] - 2 * N30u[i] * N11v[j];
                        coef_vv[p03] = N30u[i] * N11v[j];
                        coef_vv[p10] = N31u[i] * N10v[j];
                        coef_vv[p11] = N31u[i] * N11v[j] - 2 * N31u[i] * N10v[j];
                        coef_vv[p12] = N31u[i] * N10v[j] - 2 * N31u[i] * N11v[j];
                        coef_vv[p13] = N31u[i] * N11v[j];
                        coef_vv[p20] = N32u[i] * N10v[j];
                        coef_vv[p21] = N32u[i] * N11v[j] - 2 * N32u[i] * N10v[j];
                        coef_vv[p22] = N32u[i] * N10v[j] - 2 * N32u[i] * N11v[j];
                        coef_vv[p23] = N32u[i] * N11v[j];
                        coef_vv[p30] = N33u[i] * N10v[j];
                        coef_vv[p31] = N33u[i] * N11v[j] - 2 * N33u[i] * N10v[j];
                        coef_vv[p32] = N33u[i] * N10v[j] - 2 * N33u[i] * N11v[j];
                        coef_vv[p33] = N33u[i] * N11v[j];

                        wt = wts[i] * wts[j];

                        for ( ii = p00; ii <= p33; ii++ )
                        {
                            for ( jj = p00; jj <= p33; jj++ )
                            {
                                alphaMat[ii][jj] += wt * (2 * coef_uu[ii] * coef_uu[jj] + 9 * coef_uv[ii] * coef_uv[jj] + 2 * coef_vv[ii] * coef_vv[jj]);
                            }
                        }
                    }
                }

                /* Now add this 16x16 into the actual matrix */

                /* double loop for the rows of the A Matrix,  */
                /* with a nested double loop for the columns. */

                for ( ii = 0; ii <= degU; ii++ )
                {
                    idx = ctrlPtIdxU + ii;

                    if( fixEdges_v >= 1 )
                        idx--;

                    if( fixEdges_v >= 3 )
                        idx--;

                    if( idx < 0 || idx >= numVarPtsU )
                        continue;

                    rowIdxU = idx;

                    for ( iii = 0; iii <= degV; iii++ )
                    {
                        rowIdxV = ctrlPtIdxV + iii;

                        if( fixEdges_u >= 1 )
                            rowIdxV--;

                        if( fixEdges_u >= 3 )
                            rowIdxV--;

                        if( rowIdxV < 0 || rowIdxV >= numVarPtsV )
                            continue;

                        rowPtIdx = rowIdxU * numVarPtsV + rowIdxV;

                        /* This is one row (3-d point row) in main matrix */

                        BVecPt.x = BVecPt.y = BVecPt.z = 0; /* For the B-vector */

                        /* Now move across this row in the main matrix */
                        for ( jj = 0; jj <= degU; jj++ )
                        {
                            idx = ctrlPtIdxU + jj;

                            if( fixEdges_v >= 1 )
                                idx--;

                            if( fixEdges_v >= 3 )
                                idx--;

                            /* Note: leave this iteration in, for the b-vector:
                            if ( idx < 0  ||  idx >= numVarPtsU )
                            continue;
                        */

                            colIdxU = idx;

                            for ( jjj = 0; jjj <= degV; jjj++ )
                            {
                                colIdxV = ctrlPtIdxV + jjj;

                                if( fixEdges_u >= 1 )
                                    colIdxV--;

                                if( fixEdges_u >= 3 )
                                    colIdxV--;

                                /* Note: leave this iteration in, for the b-vector:
                                if ( colIdxV < 0  ||  colIdxV >= numVarPtsV )
                                continue;
                            */

                                colPtIdx = colIdxU * numVarPtsV + colIdxV;

                                /* This is one entry (3x3 point) in main matrix */

                                /* This is a 'scalar' correction, in that there is no  */
                                /* mixing of coordinates.  So it gets added only to    */
                                /* the diagonal of each 3x3 entry for each ctrl point. */

                                term = alpha * alphaMat[ii * 4 + iii][jj * 4 + jjj];

                                if( rowIdxU >= 0 && rowIdxU < numVarPtsU && colIdxU >= 0 && colIdxU < numVarPtsU && rowIdxV >= 0 && rowIdxV < numVarPtsV && colIdxV >= 0 && colIdxV < numVarPtsV )
                                {
                                    for ( coord = 0; coord < 3; coord++ )
                                    {
                                        AMat[rowPtIdx * 3 + coord][colPtIdx * 3 + coord] += term;
                                    }
                                }

                                BVecPt.x += term * ctrlPts[ctrlPtIdxU + jj][ctrlPtIdxV + jjj].x;
                                BVecPt.y += term * ctrlPts[ctrlPtIdxU + jj][ctrlPtIdxV + jjj].y;
                                BVecPt.z += term * ctrlPts[ctrlPtIdxU + jj][ctrlPtIdxV + jjj].z;
                            }
                        }

                        if( rowPtIdx >= 0 && rowPtIdx < numVarPts )
                        {
                            bMat[rowPtIdx * 3 + 0] -= BVecPt.x;
                            bMat[rowPtIdx * 3 + 1] -= BVecPt.y;
                            bMat[rowPtIdx * 3 + 2] -= BVecPt.z;
                        }
                    }
                }
            } /* end of spanV: a single polynomial patch of the surface. */
        }     /* end of spanU: row of patches */
    }         /* end of alpha correction */

    /*  Ok, now we've got A and b, solve Ax = b for x,
    *  which is the displacements.
    *
    *  Use N_RealMatrixLuDecomposePivot and N_RealMatrixForBackPivot, as in N_RealMatrixLstSqSolve:
    */

    idxArray = N_AllocInt1dArray( numVars, &localStacks );

    if( idxArray == NULL )
        NL_ERROR( NL_MEM_ERR );

    error = N_RealMatrixLuDecomposePivot( &rMat, idxArray ); /* do L/U decomp on AMat */

    if( error != NL_NO )
        NL_ERROR( NL_SEQ_ERR );

    error = N_RealMatrixForBackPivot( &rMat, idxArray, bMat ); /* Solve L/U decomposed mat */

    if( error != NL_NO )
        NL_ERROR( NL_SEQ_ERR );

    /* Load up the results */
    colPtIdx = 0;

    if( fixEdges_u >= 1 )
    {
        colPtIdx = 1;

        for ( i = 0; i < numCtrlPtsU; i++ )
        {
            ctrlPtMoves[i][0] = NL_ZERO;
            ctrlPtMoves[i][numCtrlPtsV - 1] = NL_ZERO;
        }
    }

    if( fixEdges_u >= 3 )
    {
        colPtIdx = 2;

        for ( i = 0; i < numCtrlPtsU; i++ )
        {
            ctrlPtMoves[i][1] = NL_ZERO;
            ctrlPtMoves[i][numCtrlPtsV - 2] = NL_ZERO;
        }
    }

    rowPtIdx = 0;

    if( fixEdges_v >= 1 )
    {
        rowPtIdx = 1;

        for ( j = 0; j < numCtrlPtsV; j++ )
        {
            ctrlPtMoves[0][j] = NL_ZERO;
            ctrlPtMoves[numCtrlPtsU - 1][j] = NL_ZERO;
        }
    }

    if( fixEdges_v >= 3 )
    {
        rowPtIdx = 2;

        for ( j = 0; j < numCtrlPtsV; j++ )
        {
            ctrlPtMoves[1][j] = NL_ZERO;
            ctrlPtMoves[numCtrlPtsU - 2][j] = NL_ZERO;
        }
    }

    for ( i = 0; i < numVarPtsU; i++ )
    {
        k = colPtIdx;

        for ( j = 0; j < numVarPtsV; j++ )
        {
            ctrlPtMoves[rowPtIdx][k].x = bMat[(i * numVarPtsV + j) * 3 + 0];
            ctrlPtMoves[rowPtIdx][k].y = bMat[(i * numVarPtsV + j) * 3 + 1];
            ctrlPtMoves[rowPtIdx][k].z = bMat[(i * numVarPtsV + j) * 3 + 2];

            k++;
        }
        rowPtIdx++;
    }

    EXIT:

    N_EndNurbs( &localStacks );

    return error;

} /* end ST_findStep */


/* --------------------------------------------------------------- */
/*                                                                 */
/* --------------------------------------------------------------- */
static NL_BOOLEAN ST_doneIterating( NL_INDEX iter, NL_INDEX maxIter, NL_REAL errorSD, NL_REAL errorDist, NL_REAL errorStop, NL_REAL surfSize, NL_INDEX nctlU, NL_INDEX nctlV, NL_VECTOR ** ctrlPtMoves )
{
    static NL_REAL errors[MAXITER];

    /* whether to check converging, or just continue to MAXITER: */
    static constexpr NL_FLAG checkIter = NL_YES;

    NL_INDEX i, j;
    NL_REAL thisMove, totalMove;

    NL_BOOLEAN prevConverged, thisConverged;

    if( iter >= maxIter )
        return NL_TRUE;

    if( errorDist < errorStop )
        return NL_TRUE;

    errors[iter] = errorSD;

    if( iter < 1 )
        return NL_FALSE;

    /*  Check equilibrium.  Compare the average suggested point move
    *  to the characteristic surface size.
    *
    *  Note: in practice, if we see totalMove increasing (and
    *  error decreasing), then alpha is too small.
*/
    totalMove = 0;

    for ( i = 0; i < nctlU; i++ )
    {
        for ( j = 0; j < nctlV; j++ )
        {
            N_VectorDotRef( &( ctrlPtMoves[i][j] ), &( ctrlPtMoves[i][j] ), &thisMove );
            totalMove += thisMove;
        }
    }

    if( totalMove < (NL_REAL)nctlU * (NL_REAL)nctlV * (NL_REAL)surfSize * 0.000001 )
        return NL_TRUE;

    /* Check converging (diverging).  */
    /* Check two consecutive steps, because sometimes it will bump up
    just a bit, and then continue downward. */

    if( checkIter == NL_YES && iter >= 3 )
    {
        prevConverged = (errors[iter - 1] >= errors[iter - 2]) || (errors[iter - 1] / errors[iter - 2] > 0.998);

        if( prevConverged )
        {
            thisConverged = (errors[iter] >= errors[iter - 1]) || (errors[iter] / errors[iter - 1] > 0.999);

            if( thisConverged )
                return NL_TRUE;
        }
    }

    return NL_FALSE;

} /* end ST_doneIterating */



/* --------------------------------------------------------------- */
/* Calculate a characteristic size for the surface.                */
/* We'll use the average length (squared) of control net legs.     */
/* --------------------------------------------------------------- */
static NL_REAL ST_calcSurfSize( NL_INDEX numU, NL_INDEX numV, NL_CPOINT ** ctrlPts )
{
    NL_INDEX i, j;
    NL_CPOINT *p1, *p2;

    NL_REAL result = 0;

    for ( i = 0; i < numU; i++ )
    {
        for ( j = 0; j < numV; j++ )
        {
            p1 = &( ctrlPts[i][j] );

            if( i < numU - 1 )
            {
                p2 = &( ctrlPts[i + 1][j] );
                result += (p2->x - p1->x) * (p2->x - p1->x) + (p2->y - p1->y) * (p2->y - p1->y) + (p2->z - p1->z) * (p2->z - p1->z);
            }

            if( j < numV - 1 )
            {
                p2 = &( ctrlPts[i][j + 1] );
                result += (p2->x - p1->x) * (p2->x - p1->x) + (p2->y - p1->y) * (p2->y - p1->y) + (p2->z - p1->z) * (p2->z - p1->z);
            }
        }
    }

    return result / (NL_REAL)( 2 * (NL_REAL)numU * (NL_REAL)numV - (NL_REAL)numU - (NL_REAL)numV );

} /* end ST_calcSurfSize */

/* ---------------------------------------------------------------- */
/** Add stiffness terms to the A matrix of a least squares problem **/
/** to make A matrices solvable in the face of sparse data.        **/
/**                                                                **/
/** GWC:bug151 - function was extended to support full and reduced **/
/** A matrices (reduced A matrix = The stiffness matrix of an Ax=B **/
/** eqn after a set of constraints of the form x_i = value_i ;     **/
/** have been applied.)  Args, OptMap, OptSw, OptNrhs, OptNu       **/
/** describe the mapping from the unconstrained                    **/
/**   Ax=B eqns to the reduced eqns.                               **/
/* ---------------------------------------------------------------- */
NL_VOID N_AddStiffnessToSrfAMatrix
  (NL_REAL     alpha,          /* in : Resistance to stretch weight term              */
   NL_REAL     beta,           /* in : Resistance to beta weight term                 */
   NL_SURFACE *pSurface,       /* in : Surf making the stiffness matrix               */
   NL_REAL   **A,              /* i/o: A matrix that recieves new terms.              */
                               /*      sized:[nu x nu]                                */
   NL_INDEX  **OptMap,         /* in : Index mapping: GlobalDofIndex->AIndex          */
                               /*      sized:[  pSurface->UControlPointCount          */
                               /*             x pSurface->VControlPointCount]         */
                               /*      NULL to ignore,                                */
                               /*      when NULL    nu = N,                           */
                               /*      when NotNUll nu = OptNu                        */
                               /*      N =   pSurface->UControlPointCount             */
                               /*          * pSurface->VControlPointCount             */
   NL_CPOINT **OptSw,          /* in : Constrained DOF values, unconstrained          */
                               /*      are not used - but conceptually zero.          */
                               /*      sized:[  pSurface->UControlPointCount          */
                               /*             x pSurface->VControlPointCount]         */
   NL_POINT   *OptNrhs,        /* in : Rhs to be augmented by constrained DOF values  */
                               /*      sized:[OptNu]                                  */
   NL_INDEX   *OptNu)          /* in : Size Param for constrained DOF mapping,        */
                               /*      sizeof OptMap and OptNrhs.                     */             
{
    /* NL_PRIVATE NL_STRING rname = _T("N_AddStiffnessToSrfAMatrix"); */

    NL_INDEX nIntgrlDegree = 10, nSpanGaussPtCnt ;

    NL_FLAG error = NL_NO;
    NL_DEGREE p, q ;
    NL_KNOTVECTOR *knu, *knv ;  
    NL_REAL *US, *VS, dScaleU, dScaleV, dScaleUV ;
    NL_INDEX i, j, k, cnt, m, n, r, s ;
    NL_INDEX k0, k1, j0, j1, i0, i1, gpti, gptj ;
    NL_INDEX  nSpanCntU, nSpanCntV, nSampleCntU, nSampleCntV ;
    NL_REAL *dSampleUs, *dSampleVs, ***Bu, ***Bv ;
    NL_INTEGER *SampleSpanU, *SampleSpanV, USpan, VSpan ;
    NL_INTEGER mStart, mm, ll, mmMap, llMap ;
    NL_REAL  dAlphaBUm, dBeta_BUUm, dAlphaBVm, dBeta_BVVm ;
    NL_REAL  Ammll ;
    NL_POINT P1 ;
    NL_STACKS SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* surface locals */
    N_SrfGetKnotVectors ( pSurface, &knu, &knv );
    N_SrfGetDegrees     ( pSurface, &p, &q ) ;
    N_SrfGetArraySizes  ( pSurface, &n, &m, &r, &s) ;
    N_BasisGetSpanCount ( knu, p, &nSpanCntU ) ;
    N_BasisGetSpanCount ( knv, q, &nSpanCntV ) ;
    N_KnotVectorGetKnots( knu, &r, &US );
    N_KnotVectorGetKnots( knv, &s, &VS );

    /* linear gauss point counts */
    nSpanGaussPtCnt = N_LinearGaussPtCount(nIntgrlDegree) ;  
    nSampleCntU = nSpanGaussPtCnt * nSpanCntU ;
    nSampleCntV = nSpanGaussPtCnt * nSpanCntV ;

    /* alloc memory for parameter evals */
    dSampleUs = N_AllocReal1dArray( nSampleCntU-1, &SL) ;
    dSampleVs = N_AllocReal1dArray( nSampleCntV-1, &SL) ;

    /* select the U sample point values */
    /* for every u knot value */
    for ( i = p, cnt=0; i < r - p; i++ )
    {
      /* for each non-zero span in knot vector */
      if( US[i] NEQ US[i + 1] )
        {
          /* add nGassPtCnt Sample Values to dSample array */
          for(k=0;k<nSpanGaussPtCnt;k++)
            {
              dSampleUs[cnt] = N_SCALE_GPT_LOC(k, nSpanGaussPtCnt, US[i], US[i + 1]) ;
              cnt++ ;

            } /* end iter every gauss pt for this span */
        } /* end found a non-zero span check */
    } /* end iter every knot value looking for non-zero spans */

    /* select the V sample point values */
    /* for every v knot value */
    for ( j = q, cnt=0; j < s - q; j++ )
    {
      /* for each non-zero span in knot vector */
      if( VS[j] NEQ VS[j + 1] )
        {
          /* add nGassPtCnt Sample Values to dSample array */
          for(k=0;k<nSpanGaussPtCnt;k++)
            {
              dSampleVs[cnt] = N_SCALE_GPT_LOC(k, nSpanGaussPtCnt, VS[j], VS[j + 1]) ;
              cnt++ ;

            } /* end iter every gauss pt for this span */
        } /* end found a non-zero span check */
    } /* end iter every knot value looking for non-zero spans */

    /* gather B, Bu, Buu for every USample point */
    /* gather B, Bv, Bvv for every VSample point */
    
    /* alloc memory for basis evals */
    Bu = N_AllocReal3dArray( 2, p, nSampleCntU-1 ,&SL );  /* use: Bu[B/Bu/Buu][NonZeroBasisValue][SamplePoint] */
    Bv = N_AllocReal3dArray( 2, q, nSampleCntV-1 ,&SL );  /* use: Bv[B/Bv/Bvv][NonZeroBasisValue][SamplePoint] */
    SampleSpanU = N_AllocInt1dArray(nSampleCntU-1, &SL ); /* use: SampleSpanU[i] = span index for ith U sample point */
    SampleSpanV = N_AllocInt1dArray(nSampleCntV-1, &SL ); /* use: SampleSpanV[j] = span index for jth V sample point*/

    /* Get Basis Evals */
    error = N_BasisDerivsArray(knu, p, dSampleUs, nSampleCntU-1, NL_LEFT, 2, Bu, SampleSpanU) ; 
    if( error EQ NL_YES )
        NL_OUT;
    error = N_BasisDerivsArray(knv, q, dSampleVs, nSampleCntV-1, NL_LEFT, 2, Bv, SampleSpanV) ; 
    if( error EQ NL_YES )
        NL_OUT;

    /* compute and add stiffness terms to A matrix */

/* Use in nD: To use gauss integration in 2d or higher dimensions for rectilinear domains:                                */
/*                                                                                                                        */
/*                  b0        b1                           +1        +1                                                   */
/*               Integral (Integral(func(dU,dV) daDu dv)) = Integral (Integral(func(dU(dX),dV(dY)) daDu/dx dv/dy dx dy))  */
/*                  a0        a1                           -1        -1                                                   */
/*                                                                                                                        */
/*                  b0        b1                                                                                          */
/*               Integral (Integral(func(dU,dV) daDu dv)) = for(intgrl=0.0,gpti=0;gpti<gpti_count;gpti++)                 */
/*                  a0        a1                          {                                                               */
/*                                                          for(gptj=0;gptj<gptj_count;gptj++)                            */
/*                                                            {                                                           */
/*                                                              intgrl +=   (b0-a0)/2.0 * (b1-a1)/2.0                     */
/*                                                                        * N_gauss_wt[gpti_count][gpti]                  */
/*                                                                        * N_gauss_wt[gptj_count][gptj]                  */
/*                                                                        * func(N_SCALE_GPT_LOC(gpti,gpt_count,a0,b0),   */
/*                                                                               N_SCALE_GPT_LOC(gptj,gpt_count,a1,b1))   */
/*                                                            }                                                           */
/*                                                        }                                                               */
/*                                                                                                                        */
/*                NOTE: gpti_count does not have to equal gptj_count                                                      */

    /* for every U sample point */
    for(k0=0;k0<nSampleCntU;k0++)
      {
        USpan   = SampleSpanU[k0] ;
        gpti    = k0 % nSpanGaussPtCnt ;
        dScaleU =   (US[USpan+1] - US[USpan]) / 2.0
                  * (N_gauss_wt[nSpanGaussPtCnt][gpti]) ;

        /* for every V sample point */
        for(k1=0;k1<nSampleCntV;k1++)
          {
            VSpan   = SampleSpanV[k1] ;
            gptj    = k1 % nSpanGaussPtCnt ;
            dScaleV =   (VS[VSpan+1] - VS[VSpan]) / 2.0 
                      * (N_gauss_wt[nSpanGaussPtCnt][gptj]) ;
            dScaleUV = dScaleU * dScaleV ;

            /* smallest global dof number for USpan/VSpan element */
            mStart = ((VSpan - q) * (n + 1)) + (USpan - p) ;

            /* for every V Basis function */
            for(j0=0;j0<=q;j0++)
              {
                /* map i0,j0 to global mm index */
                mm = mStart + (j0 * (n + 1)) ;

                /* for every U Basis function */
                for(i0=0;i0<=p;i0++, mm++)
                  {

                    /* when working with constrained ControlPoints */
                    if(OptMap)
                      {
                        /* map mm to control point indices then to A indices */
                        mmMap = OptMap[USpan - p + i0][VSpan - q + j0] ;
                        if(mmMap < -1 || mmMap >= *OptNu)
                          { }
                      }                             
                    else
                      {
                        mmMap = mm ; 
                      }

                    /* don't build A rows for constrained control points */
                    if(mmMap == -1)
                      { continue ; }

                    /* Basis functions B_m */
                    dAlphaBUm  = alpha * Bu[1][i0][k0] * Bv[0][j0][k1] ;  /* alpha * Wu : Wu  = B_Uu [NonZeroBasis_i][SamplePoint_u] * B_V  [NonZeroBasis_j][SamplePoint_v] */
                    dBeta_BUUm = beta  * Bu[2][i0][k0] * Bv[0][j0][k1] ;  /* beta  * Wuu: Wuu = B_Uuu[NonZeroBasis_i][SamplePoint_u] * B_V  [NonZeroBasis_j][SamplePoint_v] */
                    dAlphaBVm  = alpha * Bu[0][i0][k0] * Bv[1][j0][k1] ;  /* alpha * Wv : Wv  = B_U  [NonZeroBasis_i][SamplePoint_u] * B_Vv [NonZeroBasis_j][SamplePoint_v] */
                    dBeta_BVVm = beta  * Bu[0][i0][k0] * Bv[2][j0][k1] ;  /* beta  * Wvv: Wvv = B_U  [NonZeroBasis_i][SamplePoint_u] * B_Vvv[NonZeroBasis_j][SamplePoint_v] */

                    /* add B_m(uk,vk)*B_l(uk,vk) terms to A[m,l] matrix elements */
                    for(j1=0;j1<=q;j1++)
                      {
                        /* map i1,j1 to global ll index */
                        ll = mStart + (j1 * (n + 1)) ;

                        for(i1=0;i1<=p;i1++,ll++)
                          {

                            /* when working with constrained ControlPoints */
                            if(OptMap)
                              {
                                /* map mm to control point indices then to A indices */
                                llMap = OptMap[USpan - p + i1][VSpan - q + j1] ;
                                if(llMap < -1 || llMap >= *OptNu)
                                  { }
                              }
                            else
                              {
                                llMap = ll ; 
                              }

                            Ammll =   dScaleUV
                                   * (  dAlphaBUm  * Bu[1][i1][k0] * Bv[0][j1][k1]
                                      + dAlphaBVm  * Bu[0][i1][k0] * Bv[1][j1][k1]
                                      + dBeta_BUUm * Bu[2][i1][k0] * Bv[0][j1][k1]
                                      + dBeta_BVVm * Bu[0][i1][k0] * Bv[2][j1][k1]) ;

                            /* add terms to A matrix for unconstrained columns */
                            if(llMap != -1)
                              {
                                A[mmMap][llMap] += Ammll ;
                              }
                            else /* subtract term from rhs */
                              {
                                N_CPtToPtEuclid( OptSw[USpan - p + i1][VSpan - q + j1], &P1 );
                                N_VectorBlendPt( -Ammll, P1, &OptNrhs[mmMap] );

                              }

                          } /* end iter every U Basis function */
                      } /* end iter every V Basis function - making A terms */
                  } /* end iter every U Basis function */
              } /* end iter every V Basis function - making B and A terms */
          } /* end iter every V sample point */
      } /* end iter every U sample point */

    EXIT:
    N_EndNurbs( &SL );
    return ;

} /* end N_AddStiffnessToSrfAMatrix */

#if NLIB_UNUSED

/* ---------------------------------------------------------------- */
/** Add stiffness terms to the A matrix of a least squares problem **/
/** to make A matrices solvable in the face of sparse data.        **/
/**                                                                **/
/** GWC:bug151 - function was extended to support full and reduced **/
/** A matrices (reduced A matrix = The stiffness matrix of an Ax=B **/
/** eqn after a set of constraints of the form x_i = value_i ;     **/
/** have been applied.)  Args, OptMap, OptSw, OptNrhs, OptNu       **/
/** describe the mapping from the unconstrained                    **/
/**   Ax=B eqns to the reduced eqns.                               **/
/* ---------------------------------------------------------------- */
NL_VOID N_AddStiffnessToCrvAMatrix
  (NL_REAL     alpha,          /* in : Resistance to stretch weight term              */
   NL_REAL     beta,           /* in : Resistance to beta weight term                 */
   NL_CURVE   *pCurve,         /* in : Curve making the stiffness matrix              */
   NL_REAL   **A,              /* i/o: A matrix that recieves new terms.              */
                               /*      sized:[nu x nu]                                */
   NL_INDEX   *OptMap,         /* in : Index mapping: GlobalDofIndex->AIndex          */
                               /*      sized:[  pCurve->UControlPointCount]           */
                               /*      NULL to ignore,                                */
                               /*      when NULL    nu = N,                           */
                               /*      when NotNUll nu = OptNu                        */
                               /*      N =   pCurve->UControlPointCount               */
   NL_CPOINT  *OptSw,          /* in : Constrained DOF values, unconstrained          */
                               /*      are not used - but conceptually zero.          */
                               /*      sized:[  pCurve->UControlPointCount]           */
   NL_POINT   *OptNrhs,        /* in : Rhs to be augmented by constrained DOF values  */
                               /*      sized:[OptNu]                                  */
   NL_INDEX   *OptNu)          /* in : Size Param for constrained DOF mapping,        */
                               /*      sizeof OptMap and OptNrhs.                     */             
{
    /* NL_PRIVATE NL_STRING rname = _T("N_AddStiffnessToCrvAMatrix"); */

    NL_INDEX nIntgrlDegree = 10, nSpanGaussPtCnt ;

    NL_FLAG error = NL_NO;
    NL_KNOTVECTOR *knu ;  
    NL_DEGREE      p ;
    NL_INDEX       r, cnt ;
    NL_INDEX       ii, jj, kk, gpti ;
    NL_INDEX       nSpanCntU, nSampleCntU ;
    NL_REAL       *US, dScaleU ;
    NL_REAL       *dSampleUs, ***Bu ;
    NL_INTEGER    *SampleSpanU, USpan ;
    NL_INTEGER     mStart, mm, ll, mmMap, llMap ;
    NL_REAL        dAlpha_BUm, dBeta_BUUm ;
    NL_REAL        Ammll ;
    NL_POINT       P1 ;
    NL_STACKS      SL;

    /* Start NURBS */
    N_InitNurbs( &SL );

    /* surface locals */
    N_CrvGetKnotVector  ( pCurve, &knu );          /* knu->m    = max knot index in U array */
                                                   /* knu->U    = Knot Array sized:[m+1] */
    N_CrvGetDegree      ( pCurve, &p ) ;           /* p         = degree */
    N_BasisGetSpanCount ( knu, p, &nSpanCntU ) ;   /* nSpanCntU = Number of nonZero spans in pCurve */
    N_KnotVectorGetKnots( knu, &r, &US );          /* r         = knu->m = highest index in U, knot array */
                                                   /* US        = knu->U = Knot Array sized:[m+1] */

    /* linear gauss point counts */
    nSpanGaussPtCnt = N_LinearGaussPtCount(nIntgrlDegree) ;  
    nSampleCntU     = nSpanGaussPtCnt * nSpanCntU ;

    /* alloc memory for parameter evals */
    dSampleUs = N_AllocReal1dArray( nSampleCntU-1, &SL) ;

    /* select the U sample point values - build array dSampleUs */
    /* for every u knot value */
    for ( ii = p, cnt=0; ii < r - p; ii++ )
      {
        /* for each non-zero span in knot vector */
        if( US[ii] NEQ US[ii + 1] )
          {
            /* add nGaussPtCnt Sample Values to dSample array */
            for(kk=0;kk<nSpanGaussPtCnt;kk++)
              {
                dSampleUs[cnt] = N_SCALE_GPT_LOC(kk, nSpanGaussPtCnt, US[ii], US[ii + 1]) ;
                cnt++ ;

              } /* end iter every gauss pt for this span */
          } /* end found a non-zero span check */
      } /* end iter every knot value looking for non-zero spans */

    /* Assert((cnt-1) == nSampleCntU) */

    /* gather B, Bu, Buu for every USample point */
    
    /* alloc memory for basis evals */
    Bu = N_AllocReal3dArray( 2, p, nSampleCntU-1 ,&SL );  /* use: Bu[B/Bu/Buu][NonZeroBasisValue][SamplePoint] */
    SampleSpanU = N_AllocInt1dArray(nSampleCntU-1, &SL ); /* use: SampleSpanU[ii] = span index for ith U sample point */

    /* Get Basis Evals */
    error = N_BasisDerivsArray(knu, p, dSampleUs, nSampleCntU-1, NL_LEFT, 2, Bu, SampleSpanU) ; 
    if( error EQ NL_YES )
        NL_OUT;

    /* compute and add stiffness terms to A matrix */

/* Use in nD: To use gauss integration in 2d or higher dimensions for rectilinear domains:               */
/*                                                                                                       */
/*                  b0                           +1                                                      */
/*               Integral(func(dU) daDu) = Integral(func(dU(dX)) daDu/dx dx))                            */
/*                  a0                           -1                                                      */
/*                                                                                                       */
/*                  b0                                                                                   */
/*               Integral(func(dU) daDu) = for(intgrl=0.0,gpti=0;gpti<gpti_count;gpti++)                 */
/*                  a0                       {                                                           */
/*                                             intgrl +=   (b0-a0)/2.0                                   */
/*                                                       * N_gauss_wt[gpti_count][gpti]                  */
/*                                                       * func(N_SCALE_GPT_LOC(gpti,gpt_count,a0,b0))   */
/*                                           }                                                           */
/*                                                                                                       */

    /* for every U sample point */
    for(kk=0;kk<nSampleCntU;kk++)
      {
        USpan   = SampleSpanU[kk] ;
        gpti    = kk % nSpanGaussPtCnt ;
        dScaleU =   (US[USpan+1] - US[USpan]) / 2.0
                  * (N_gauss_wt[nSpanGaussPtCnt][gpti]) ;

        /* smallest global dof number for USpan element */
        mStart = (USpan - p) ;

        /* map ii to global mm index */
        mm = mStart  ;

        /* for every U Basis function */
        for(ii=0;ii<=p;ii++, mm++)
          {

            /* when working with constrained ControlPoints */
            if(OptMap)
              {
                /* map mm to control point indices then to A indices */
                mmMap = OptMap[mm] ;   /* mm = USpan - p + ii */
                if(mmMap < -1 || mmMap >= *OptNu)
                  { }
              }                             
            else
              {
                mmMap = mm ; /* note: mm is incremented in the for loop - so - no need to add ii here */
              }

            /* don't build A rows for constrained control points */
            if(mmMap == -1)
              { continue ; }

            /* Basis functions B_m */
            dAlpha_BUm = alpha * Bu[1][ii][kk] ;  /* alpha * Wu : Wu  = B_Uu [NonZeroBasis_i][SamplePoint_u] */
            dBeta_BUUm = beta  * Bu[2][ii][kk] ;  /* beta  * Wuu: Wuu = B_Uuu[NonZeroBasis_i][SamplePoint_u] */

            /* add B_m(uk)*B_l(uk) terms to A[m,l] matrix elements */
            for(jj=0;jj<=p;jj++)
              {
                /* map jj to global ll index */
                ll = mStart + jj ;

                /* when working with constrained ControlPoints */
                if(OptMap)
                  {
                    /* map mm to control point indices then to A indices */
                    llMap = OptMap[ll] ;   /* ll = USpan - p + jj */
                    if(llMap < -1 || llMap >= *OptNu)
                      { }
                  }
                else
                  {
                    llMap = ll ; 
                  }

                Ammll =   dScaleU
                       * (  dAlpha_BUm * Bu[1][jj][kk]
                          + dBeta_BUUm * Bu[2][jj][kk]) ;

                /* add terms to A matrix for unconstrained columns */
                if(llMap != -1)
                  {
                    A[mmMap][llMap] += Ammll ;
                  }
                else /* subtract term from rhs */
                  {
                    N_CPtToPtEuclid( OptSw[ll], &P1 );   /* ll = USpan - p + jj */
                    N_VectorBlendPt( -Ammll, P1, &OptNrhs[mmMap] );

                  }

              } /* end iter jj, every NonZero U Basis function for U SamplePoint */
          } /* end iter ii, every NonZero U Basis function for U SamplePoint */
      } /* end iter kk, every U sample point */

    EXIT:
    N_EndNurbs( &SL );
    return ;

} /* end N_AddStiffnessToCrvAMatrix */

#endif // NLIB_UNUSED
