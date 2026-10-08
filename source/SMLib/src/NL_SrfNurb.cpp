// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/************************************************************************/
/* SrfNurb.c : NURB function Definitions that act on NL_SURFACE objects */
/************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>
#include <NLI_Math.h>



/*******************************************************************/ /**


    DESCRIPTION:

      This routine computes the basis functions

        NL_DEGREE           p;
        NL_PARAMETER        u;
        NL_REAL*            U;
        NL_INDEX            ku;
        NL_BASISFUNCTIONS   ndu;

        ...
        (define p, u, U, ku, ndu);
        ...
        N_ComputeBasisFunctions(p, u, U, ku, ndu);


    ACCESS:

      p     , input  ,  degree of knot vector?
      u     , input  ,  knot vector position?
      U     , input  ,  knot vector?
      ku    , input  ,  knot vector index?
      ndu   , output ,  basis functions


    RETURN CODES:

      0 : No error
      1 : Error saved in NL_ERROR

***********************************************************************/

static NL_INLINE NL_VOID N_ComputeBasisFunctions(
    NL_DEGREE p, NL_PARAMETER u, NL_REAL* U, NL_INDEX ku, NL_BASISFUNCTIONS ndu)
{
    /* Compute the basis functions */
    NL_REAL a[2][NL_MAXDEG + 1];

    NL_REAL* left = &a[0][0];
    NL_REAL* right = &a[1][0];

    ndu[0][0] = 1.0;

    for (NL_INDEX j = 1; j <= p; j++)
    {
        left[j] = u - U[ku + 1 - j];
        right[j] = U[ku + j] - u;
        NL_REAL saved = 0.0;

        for (NL_INDEX r = 0; r < j; r++)
        {
            ndu[j][r] = right[r + 1] + left[j - r];
            NL_REAL temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
}

/*******************************************************************/ /**


    DESCRIPTION:

      This  routine  swaps the two real pointers p1 and p2

        NL_REAL ** p1;
        NL_REAL ** p2;

        N_SwapRealPointers(p1, p2);


    ACCESS:

      p1     , both  ,     first pointer
      p2     , both  ,     second pointer



    RETURN CODES:

      none

    ***********************************************************************/
static NL_INLINE NL_VOID N_SwapRealPointers(NL_REAL** p1, NL_REAL** p2)
{
    NL_REAL* temp = *p1;
    *p1 = *p2;
    *p2 = temp;
}


/*******************************************************************/ /**


    DESCRIPTION:

      This template specializes the computation of N_ComputeDerivatives
      for either the fixed value mder2 or the argument value mder.

        ...
        (define p, mder, DU);
        ...
        (fix mder2);
        ...
        N_ComputeDerivativesImpl<mder2>(p, mder, ndu, DU);


    ACCESS:

      mder2 , input  ,     value for which the computation is specialized
      p     , input  ,     degree of knot vector
      mder  , input  ,     Maximum derivative index
      ndu,  , input  ,     basis functions
      DU    , output ,     computed derivatives


    RETURN CODES:

      0 : No error
      1 : Error saved in NL_ERROR

 ***********************************************************************/

template <NL_INDEX mder2>
static NL_NOINLINE NL_VOID N_ComputeDerivativesImpl(
    NL_DEGREE p, NL_INDEX mder1, const NL_BASISFUNCTIONS& ndu, NL_DERIVATIVES& DU)
{
    NL_INDEX mder = mder2 != std::numeric_limits<NL_INDEX>::max() ? mder2 : mder1;
    NL_REAL a[2][NL_MAXDEG + 1];

    for (NL_INDEX r = 0; r <= p; r++)
    {
        NL_REAL* a_s1 = a[0];
        NL_REAL* a_s2 = a[1];

        a_s1[0] = 1.0;

        for (NL_INDEX k = 1; k <= mder; k++)
        {
            NL_REAL dd = 0.0;
            NL_INDEX rk = r - k;
            NL_INDEX pk = p - k;

            if (r GE k)
            {
                a_s2[0] = a_s1[0] / ndu[pk + 1][rk];
                dd = a_s2[0] * ndu[rk][pk];
            }

            NL_INDEX j1 = (rk GE - 1) ? 1 : -rk;
            NL_INDEX j2 = ((r - 1) LE pk) ? (k - 1) : (p - r);

            for (NL_INDEX j = j1; j <= j2; j++)
            {
                a_s2[j] = (a_s1[j] - a_s1[j - 1]) / ndu[pk + 1][rk + j];
                dd += a_s2[j] * ndu[rk + j][pk];
            }

            if (r LE pk)
            {
                a_s2[k] = -a_s1[k - 1] / ndu[pk + 1][r];
                dd += a_s2[k] * ndu[r][pk];
            }
            DU[k][r] = dd;
            N_SwapRealPointers(&a_s1, &a_s2);
        }
    }
}

static NL_INLINE NL_VOID N_ComputeDerivatives(NL_DEGREE p, NL_INDEX mder, const NL_BASISFUNCTIONS& ndu, NL_DERIVATIVES& DU)
{
    if (mder == 1)
    {
        return N_ComputeDerivativesImpl<1>(p, mder, ndu, DU);
    }
    else
    {
        return N_ComputeDerivativesImpl<std::numeric_limits<NL_INDEX>::max()>(p, mder, ndu, DU);
    }
}

/*******************************************************************//**


   DESCRIPTION:

     This  routine  evaluates  all  non-vanishing  bivariate  rational  
     or non-rational basis functions and their  derivatives at a given  
     parameter value. It is assumed that the knot vectors are clamped, 
     i.e., they are repeated with multiplicity = degree + 1. A typical
     calling example is:

       NL_SURFACE            sur;
       NL_PARAMETER          u, v;
       NL_INDEX              udr, vdr, usp, vsp;
       NL_BASISDERIVATIVES & BD;
       ...
       (define sur; get u, v, udr and vdr);
       ...
       N_SrfBasisDerivs(&sur,u,v,NL_LEFT,NL_RIGHT,NL_TRUE,udr,vdr,BD,&usp,&vsp);


   ACCESS:
   
     sur     , input  ,  NURBS surface
     u,v     , input  ,  Parameter values
     ufl,vfl , input  ,  Flags:
                           NL_LEFT : t is in [t[j],t[j+1])
                                  (NL_RIGHT DERIVATIVES REQUIRED)
                           NL_RIGHT: t is in (t[j],t[j+1]]
                                  (NL_LEFT DERIVATIVES REQUIRED)
                           (t is either u or v)
     mfl     , input  ,  Flag: 
                           NL_TRUE : compute   upper  half  only  of  the 
                                  derivative matrix
                           NL_FALSE: compute full derivative matrix
                         (APPLICABLE ONLY IF udr=vdr!)
     udr,vdr , input  ,  Highest derivatives required
     BD      , output ,  Derivatives  computed at u; BD[k][l][i][j] is 
                         the (k,l)-th derivative of the basis function  
                         N[ku-p+i][kv-q+j],  where  u  is  in  {u[ku],
                         u[ku+1]} and  v is in  {v[kv],v[kv+1}. MEMORY  
                         FOR  BD  MUST  BE  ALLOCATED  IN THE  CALLING  
                         ROUTINE  TO  HOLD  UP TO  BD[udr][vdr][p][q], 
                         where p  and q are surface  degrees in u- and 
                         v-directions, respectively.
     usp,vsp , output ,  Indexes of knot spans u and v are in


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_SrfBasisDerivs */

NL_FLAG N_SrfBasisDerivs( NL_SURFACE *sur, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG mfl, NL_INDEX udr, NL_INDEX vdr, NL_BASISDERIVATIVES& BD, NL_INDEX *usp, NL_INDEX *vsp )
{
    NL_FLAG error = NL_NO;

    /* Get derivatives of basis functions */
    NL_PRIVATE NL_STRING rname = _T("N_SrfBasisDerivs");

    NL_INDEX i, j, k, ku, kv, l, r, s1, s2, mder;

    NL_BASISFUNCTIONS ndu;
    NL_REAL a[2][NL_MAXDEG + 1];
    NL_DERIVATIVES DU;
    NL_REAL *U, *V, DV[NL_MAXDER + 1][NL_MAXDEG + 1];

    NL_DEGREE p, q;

    NL_KNOTVECTOR *knu, *knv;


    /* Get local notation */

    p = sur->p;
    knu = sur->knu;
    s1 = knu->m;
    U = knu->U;
    q = sur->q;
    knv = sur->knv;
    s2 = knv->m;
    V = knv->U;

    /* Check parameters and order of derivatives */

    /* We ignore thie to allow for evaluation outside domain range
    if (u LT U[0] OR u GT U[s1] OR v LT V[0] OR v GT V[s2])
    {
      N_ErrSet(NL_PAR_ERR,rname);
      error = NL_YES;
      NL_OUT;
    }
    */

    if (udr GT NL_MAXDER OR vdr GT NL_MAXDER)
    {
        N_ErrSet(NL_MXD_ERR, rname);
        error = NL_YES;
        NL_OUT;
    }

    /* Compute all non-vanishing basis functions */

    error = N_BasisFindSpan(knu, p, u, ufl, &ku);

    if (error EQ NL_YES)
        NL_OUT;
    *usp = ku;

    /* Get maximum derivative index and set zero derivatives */

    mder = NL_MIN(p, udr);

    for (k = p + 1; k <= udr; k++)
    {
        for (j = 0; j <= p; j++)
        {
            DU[k][j] = 0.0;
        }
    }

    /* Compute the basis functions */
   N_ComputeBasisFunctions(p, u, U, ku, ndu);

    /* Load the basis functions */

    for (j = 0; j <= p; j++)
        DU[0][j] = ndu[j][p];

    /* Compute derivatives */
    N_ComputeDerivatives(p, mder, ndu, DU);

    /* Multiply through by the correct factors */

    r = p;

    for (k = 1; k <= mder; k++)
    {
        for (j = 0; j <= p; j++)
        {
            DU[k][j] *= r;
        }
        r *= (p - k);
    }

    /* Do the same for the v-direction */

    error = N_BasisFindSpan(knv, q, v, vfl, &kv);

    if (error EQ NL_YES)
        NL_OUT;
    *vsp = kv;

    /* Get maximum derivative index and set zero derivatives */

    mder = NL_MIN(q, vdr);

    for (k = q + 1; k <= vdr; k++)
    {
        for (j = 0; j <= q; j++)
        {
            DV[k][j] = 0.0;
        }
    }

    /* Compute the basis functions */
    N_ComputeBasisFunctions(q, v, V, kv, ndu);

    /* Load the basis functions */
    for (j = 0; j <= q; j++)
        DV[0][j] = ndu[j][q];

    /* Compute derivatives */
    N_ComputeDerivatives(q, mder, ndu, DV);

    /* Multiply through by the correct factors */

    r = q;

    for (k = 1; k <= mder; k++)
    {
        for (j = 0; j <= q; j++)
        {
            DV[k][j] *= r;
        }
        r *= (q - k);
    }

    if (N_IsSrfRat(sur))
    {
        NL_BASISDERIVATIVES& RD = BD;
        NL_REAL* tu = a[0];
        NL_REAL d[NL_MAXDER + 1][NL_MAXDER + 1];
        NL_CPOINT** Pw = sur->net->Pw;

        /* Get derivatives of denominator */

        s1 = ku - p;
        s2 = kv - q;

        for (l = 0; l <= vdr; l++)
        {
            for (i = 0; i <= p; i++)
            {
                tu[i] = 0.0;

                for (j = 0; j <= q; j++)
                {
                    NL_REAL w = Pw[s1 + i][s2 + j].w;
                    tu[i] += w * DV[l][j];
                }
            }

            for (k = 0; k <= udr; k++)
            {
                d[k][l] = 0.0;

                for (i = 0; i <= p; i++)
                {
                    d[k][l] += tu[i] * DU[k][i];
                }
            }
        }

        /* Compute derivatives of rational basis */

        /*         Pascal triangle first                              */
        /*                                                            */
        /*        tri[0][0] = 1;                                      */
        /*        tri[1][0] = 1;   tri[1][1] = 1;                     */
        /*        if ( tmp GT 1 )                                     */
        /*        {                                                   */
        /*          tri[2][0] = 1;   tri[2][1] = 2;   tri[2][2] = 1;  */
        /*          if ( tmp GT 2 )                                   */
        /*          {                                                 */
        /*            for( k=3; k<=tmp; k++ )                         */
        /*            {                                               */
        /*              tri[k][0] = 1;                                */
        /*              r = k/2;  j2 = 1;                             */
        /*              for( j=1; j<=r; j++ )                         */
        /*              {                                             */
        /*                j1          = tri[k-1][j];                  */
        /*                tri[k][j]   = tri[k-1][j]+j2;               */
        /*                tri[k][k-j] = tri[k][j];                    */
        /*                j2          = j1;                           */
        /*              }                                             */
        /*              tri[k][k] = 1;                                */
        /*            }                                               */
        /*          }                                                 */
        /*        }                                                   */

        if (N_FloatOpIsBad(1.0, d[0][0], NL_DIVISION))
            NL_ERROR(NL_CON_ERR);


        NL_REAL invW00 = 1.0 / d[0][0];

        NL_BOOLEAN upper_half = (mfl EQ NL_TRUE AND udr EQ vdr);

        for (NL_INDEX n = 0; n <= p; n++)
        {
            for (NL_INDEX m = 0; m <= q; m++)
            {
                NL_REAL w = Pw[s1 + n][s2 + m].w;

                for (k = 0; k <= udr; k++)
                {
                    for (l = 0; l <= (upper_half ? (udr - k) : vdr); l++)
                    {
                        NL_REAL v1 = w * DU[k][n] * DV[l][m];

                        for (j = 1; j <= l; j++)
                        {
                            v1 -= NL_PascalTri[l][j] * d[0][j] * RD[k][l - j][n][m];
                        }

                        for (i = 1; i <= k; i++)
                        {
                            v1 -= NL_PascalTri[k][i] * d[i][0] * RD[k - i][l][n][m];
                            NL_REAL v2 = 0.0;

                            for (j = 1; j <= l; j++)
                            {
                                v2 += NL_PascalTri[l][j] * d[i][j] * RD[k - i][l - j][n][m];
                            }
                            v1 -= NL_PascalTri[k][i] * v2;
                        }

                        RD[k][l][n][m] = v1 * invW00;
                    }
                }
            }
        }
    }
    else
    {
        /* Compute derivatives */
        NL_BOOLEAN upper_half = (mfl EQ NL_TRUE AND udr EQ vdr);

        /* Compute derivative matrix */
        for (k = 0; k <= udr; k++)
        {
            for (l = 0; l <= (upper_half ? (udr - k) : vdr); l++)
            {
                for (i = 0; i <= p; i++)
                {
                    for (j = 0; j <= q; j++)
                    {
                        BD[k][l][i][j] = DU[k][i] * DV[l][j];
                    }
                }
            }
        }
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_SrfBasisDerivs */

#if NLIB_UNUSED

/*******************************************************************/ /**


    DESCRIPTION:

      This  routine  evaluates  all  non-vanishing  bivariate nonrational
      basis functions  and  their  derivatives  at a  given  parameter
      value. It is  assumed that the knot  vectors are  clamped, i.e.,
      end knots are repeated with multiplicity = degree + 1. A typical
      calling example is:

        NL_SURFACE            sur;
        NL_PARAMETER          u, v;
        NL_INDEX              udr, vdr, usp, vsp;
        NL_BASISDERIVATIVES & BD;
        ...
        (define sur, get u, v, udr, vdr, and allocate memory for RD);
        ...
        N_SrfRatBasisDerivs(&sur,u,v,NL_LEFT,NL_RIGHT,NL_TRUE,udr,vdr,RD,&usp,&vsp);


    ACCESS:

      sur     , input  ,  NURBS surface
      u,v     , input  ,  Parameter values
      ufl,vfl , input  ,  Flags:
                            NL_LEFT : t is in [t[j],t[j+1])
                                   (NL_RIGHT DERIVATIVES REQUIRED)
                            NL_RIGHT: t is in (t[j],t[j+1]]
                                   (NL_LEFT DERIVATIVES REQUIRED)
                            (t is either u or v)
      mfl     , input  ,  Flag:
                            NL_TRUE : compute  upper  half  only  of the
                                   derivative matrix
                            NL_FALSE: compute full derivative matrix
                          (APPLICABLE ONLY IF udr=vdr!)
      udr,vdr , input  ,  Highest derivatives required
      RD      , output ,  Derivatives  computed  at u; RD[k][l][i][j]
                          is the  (k,l)-th  derivative  of  the basis
                          function  R[ku-p+i][kv-q+j], where  u is in
                          {u[ku],u[ku+1]} and v is in {v[kv],v[kv+1}.
                          MEMORY  FOR  RD  MUST  BE  ALLOCATED IN THE
                          CALLING  ROUTINE TO  HOLD DERIVATIVES UP TO
                          RD[udr][vdr][p][q].
      usp,vsp , output ,  Indexes of knot spans u and v are in


    RETURN CODES:

      0 : No error
      1 : Error saved in NL_ERROR

***********************************************************************/

/* NL_FLAG  N_SrfNonRatBasisDerivs */
NL_FLAG N_SrfNonRatBasisDerivs( NL_SURFACE *sur, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG mfl, NL_INDEX udr, NL_INDEX vdr, NL_BASISDERIVATIVES& BD, NL_INDEX *usp, NL_INDEX *vsp )
{
    return N_SrfBasisDerivs(sur, u, v, ufl, vfl, mfl, udr, vdr, BD, usp, vsp);
} /* end N_SrfNonRatBasisDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  evaluates  all  non-vanishing  bivariate  rational  
     or non-rational basis functions and their  derivatives at a given  
     parameter value. It is assumed that the knot vectors are clamped, 
     i.e., they are repeated with multiplicity = degree + 1. A typical
     calling example is:

       NL_SURFACE            sur;
       NL_PARAMETER          u, v;
       NL_INDEX              udr, vdr, usp, vsp;
       NL_BASISDERIVATIVES & BD;
       ...
       (define sur; get u, v, udr and vdr);
       ...
       N_SrfBasisDerivs(&sur,u,v,NL_LEFT,NL_RIGHT,NL_TRUE,udr,vdr,BD,&usp,&vsp);


   ACCESS:
   
     sur     , input  ,  NURBS surface
     u,v     , input  ,  Parameter values
     ufl,vfl , input  ,  Flags:
                           NL_LEFT : t is in [t[j],t[j+1])
                                  (NL_RIGHT DERIVATIVES REQUIRED)
                           NL_RIGHT: t is in (t[j],t[j+1]]
                                  (NL_LEFT DERIVATIVES REQUIRED)
                           (t is either u or v)
     mfl     , input  ,  Flag: 
                           NL_TRUE : compute   upper  half  only  of  the 
                                  derivative matrix
                           NL_FALSE: compute full derivative matrix
                         (APPLICABLE ONLY IF udr=vdr!)
     udr,vdr , input  ,  Highest derivatives required
     BD      , output ,  Derivatives  computed at u; BD[k][l][i][j] is 
                         the (k,l)-th derivative of the basis function  
                         N[ku-p+i][kv-q+j],  where  u  is  in  {u[ku],
                         u[ku+1]} and  v is in  {v[kv],v[kv+1}. MEMORY  
                         FOR  BD  MUST  BE  ALLOCATED  IN THE  CALLING  
                         ROUTINE  TO  HOLD  UP TO  BD[udr][vdr][p][q], 
                         where p  and q are surface  degrees in u- and 
                         v-directions, respectively.
     usp,vsp , output ,  Indexes of knot spans u and v are in


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_SrfRatBasisDerivs */
NL_FLAG N_SrfRatBasisDerivs( NL_SURFACE *sur, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG mfl, NL_INDEX udr, NL_INDEX vdr, NL_BASISDERIVATIVES& RD, NL_INDEX *usp, NL_INDEX *vsp )
{
    return N_SrfBasisDerivs(sur, u, v, ufl, vfl, mfl, udr, vdr, RD, usp, vsp);
} /* end N_SrfRatBasisDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  evaluates a  bivariate rational  basis  function 
     for a set of span indices and a parameter value.  It is assumed 
     that the knot vectors are clamped, i.e., end knots are repeated 
     with multiplicity = degree + 1. A typical calling example is:

       NL_SURFACE    sur;
       NL_INDEX      i, j;
       NL_PARAMETER  u, v;
       NL_REAL       R;
       ...
       (define sur, get i, j, u and v);
       ...
       N_SrfRatBasisIEval(&sur,i,j,u,v,NL_LEFT,NL_RIGHT,&R);


   ACCESS:
   
     sur     , input  ,  NURBS surface
     i,j     , input  ,  Indexes  of rational basis  function (0<=i<=n),
                         (0<=j<=m)
     u,v     , input  ,  Parameter values 
     ufl,vfl , input  ,  Flags:
                           NL_LEFT : t is in [t[j],t[j+1])
                           NL_RIGHT: t is in (t[j],t[j+1]]
                           (t is either u or v)
     R       , output ,  Basis function computed at (u,v)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_SrfRatBasisIEval */
NL_FLAG N_SrfRatBasisIEval( NL_SURFACE *sur, NL_INDEX i, NL_INDEX j, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_REAL *R )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfRatBasisIEval");

    NL_FLAG error = NL_NO;

    NL_INDEX n, m, k;

    NL_REAL ** w, *T, den, NU, NV;

    NL_DEGREE p, q;

    NL_KNOTVECTOR *knu, *knv;

    NL_SFUN sfn;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_SrfGetArraySizes( sur, &n, &m, &k, &k );
    N_SrfGetDegrees( sur, &p, &q );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Check parameters and indexes */

    error = N_KnotVectorIsParamOutOfBounds( knu, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knv, v, rname );

    if( error EQ NL_YES )
        NL_OUT;

    if( i LT 0 OR i GT n OR j LT 0 OR j GT m )
        NL_ERROR( NL_IND_ERR );

    /* Extract denominator */

    N_SFuncInitArrays( &sfn );
    error = N_SrfGetDenominatorFunc( sur, &sfn, &S );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncGetKnots( &sfn, &w, &T, &T );

    /* Compute the i-th and j-th B-splines */

    error = N_BasisIEval( knu, i, p, u, ufl, &NU );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisIEval( knv, j, q, v, vfl, &NV );

    if( error EQ NL_YES )
        NL_OUT;

    /* Evaluate the denominator */

    error = N_SrfFuncEvalPt( &sfn, u, v, ufl, vfl, &den );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute rational basis */

    *R = (w[i][j] * NU * NV) / den;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_SrfRatBasisIEval */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  evaluates  a  bivariate rational  basis function 
     and its  derivatives at a  given parameter value. It is assumed 
     that the knot vectors are clamped, i.e., end knots are repeated 
     with multiplicity = degree + 1. A typical calling example is:

       NL_SURFACE    sur;
       NL_INDEX      ib, jb, udr, vdr;
       NL_PARAMETER  u, v;
       NL_REAL       **RD;
       ...
       (define sur, get ib, jb, udr, vdr, u, v, and  allocate memory 
        for RD);
       ...
       N_SrfRatBasisIDerivs(&sur,ib,jb,u,v,NL_LEFT,NL_RIGHT,udr,vdr,RD);


   ACCESS:
   
     sur     , input  ,  NURBS surface
     i,j     , input  ,  Indexes of rational basis function
     u,v     , input  ,  Parameter values
     ufl,vfl , input  ,  Flags:
                           NL_LEFT : t is in [t[j],t[j+1])
                                  (NL_RIGHT NL_DERIVATIVE REQUIRED)
                           NL_RIGHT: t is in (t[j],t[j+1]]
                                  (NL_LEFT NL_DERIVATIVE REQUIRED)
                           (t is either u or v)
     udr,vdr , input  ,  Highest derivatives required
     RD      , output ,  Derivatives computed  at u; RD[k][l] is the 
                         k-th derivative in u-direction and the l-th
                         derivative  in  v-direction. MEMORY FOR RD, 
                         MUST BE ALLOCATED IN THE CALLING ROUTINE TO   
                         HOLD UP TO RD[udr][vdr].


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_SrfRatBasisIDerivs */
NL_FLAG N_SrfRatBasisIDerivs( NL_SURFACE *sur, NL_INDEX ib, NL_INDEX jb, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_INDEX udr, NL_INDEX vdr, NL_REAL ** RD )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfRatBasisIDerivs");

    NL_FLAG error = NL_NO;

    NL_INDEX n, m, i, j, k, l;

    NL_REAL ** d, ** w, *DU, *DV, v1, v2;

    NL_DEGREE p, q;

    NL_KNOTVECTOR *knu, *knv;

    NL_SFUN sfn;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_SrfGetArraySizes( sur, &n, &m, &i, &j );
    N_SrfGetDegrees( sur, &p, &q );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Check parameter and index */

    error = N_KnotVectorIsParamOutOfBounds( knu, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knv, v, rname );

    if( error EQ NL_YES )
        NL_OUT;

    if( ib LT 0 OR ib GT n OR jb LT 0 OR jb GT m )
        NL_ERROR( NL_IND_ERR );

    /* Extract denominator */

    N_SFuncInitArrays( &sfn );
    error = N_SrfGetDenominatorFunc( sur, &sfn, &S );

    if( error EQ NL_YES )
        NL_OUT;

    N_SFuncGetKnots( &sfn, &w, &DU, &DV );

    /* Get derivatives of numerator */

    DU = N_AllocReal1dArray( udr, &S );

    if( DU EQ NULL )
        NL_QUIT;

    DV = N_AllocReal1dArray( vdr, &S );

    if( DV EQ NULL )
        NL_QUIT;

    error = N_BasisIDerivs( knu, ib, p, u, ufl, udr, DU );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisIDerivs( knv, jb, q, v, vfl, vdr, DV );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get derivatives of denominator */

    d = N_AllocReal2dArray( udr, vdr, &S );

    if( d EQ NULL )
        NL_QUIT;

    error = N_SFuncDerivs( &sfn, u, v, ufl, vfl, udr, vdr, d );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute derivatives of rational basis */

    /* rok = N_AllocInt1dArray(udr,&S);     */
    /* if( rok EQ NULL )  NL_QUIT;    */

    /* rol = N_AllocInt1dArray(vdr,&S);     */
    /* if( rol EQ NULL )  NL_QUIT;    */

    for ( k = 0; k <= udr; k++ )
    {
        /* N_PascalTriIndex(rok,k);  gwc - replaced pascal tri computation with a global array */
        for ( l = 0; l <= vdr; l++ )
        {
            /* N_PascalTriIndex(rol,l); gwc - replaced pascal tri computation with a global array */
            v1 = w[ib][jb] * DU[k] * DV[l];

            for ( j = 1; j <= l; j++ )
            {
                /* v1 -= rol[j]*d[0][j]*RD[k][l-j];  */
                v1 -= NL_PascalTri[l][j] * d[0][j] * RD[k][l - j];
            }

            for ( i = 1; i <= k; i++ )
            {
                /* v1 -= rok[i]*d[i][0]*RD[k-i][l];  */
                v1 -= NL_PascalTri[k][i] * d[i][0] * RD[k - i][l];
                v2 = 0.0;

                for ( j = 1; j <= l; j++ )
                {
                    /* v2 += rol[j]*d[i][j]*RD[k-i][l-j]; */
                    v2 += NL_PascalTri[l][j] * d[i][j] * RD[k - i][l - j];
                }
                /* v1 -= rok[i]*v2; */
                v1 -= NL_PascalTri[k][i] * v2;
            }

            if( N_FloatOpIsBad( v1, d[0][0], NL_DIVISION ) )
                NL_ERROR( NL_CON_ERR );
            RD[k][l] = v1 / d[0][0];
        }
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_SrfRatBasisIDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This routine  computes the derivatives of all  non-vanishing bi-
     variate  rational  basis functions  with respect to a  knot. The 
     multiplicity of  the knot  in either direction must be less than 
     the respective degree. A typical calling example is:

       NL_SURFACE     sur;
       NL_INDEX       k, spn;
       NL_PARAMETER   u, v;
       NL_REAL        **Bk;
       ...
       (define sur; get k, u and v);
       ...
       N_SrfRatBasisKnotDeriv(&sur,k,u,v,NL_UDIR,NL_LEFT,NL_RIGHT,NL_LEFT,Bk,&spn);

     MEMORY TO STORE Bk[0][0],...,Bk[p+1][q] (OR Bk[p][q+1])  MUST BE  
     ALLOCATED IN THE CALLING ROUTINE.


   ACCESS:
   
     sur , input  ,  NURBS surface
     k   , input  ,  Index of knot, i.e. the derivative  with respect 
                     to t_k is computed (t is either u or v)
     u,v , input  ,  Parameter values
     dir , input  ,  Flag:
                       NL_UDIR: u-derivative required. Bk MUST  STORE UP
                             TO [p+1][q].
                       NL_VDIR: v-derivative required. Bk MUST  STORE UP
                             TO [p][q+1].
     flk , input  ,  Flag:
                       NL_LEFT : left  derivative. NL_INDEX  k MUST SATISFY
                              t_(k) != t_(k-1)
                       NL_RIGHT: right  derivative. NL_INDEX k MUST SATISFY
                              t_(k) != t_(k+1)
                              (t is either u or v)
     ulp , input  ,  Flag:
                       NL_LEFT : u is in [u[j],u[j+1]) 
                       NL_RIGHT: u is in (u[j],u[j+1]]
     vlp , input  ,  Flag:
                       NL_LEFT : v is in [v[j],v[j+1]) 
                       NL_RIGHT: v is in (v[j],v[j+1]]
     Bk  , output ,  Basis  function  derivatives  computed at (u,v). 
                     The  values are stored in Bk[0][0],...,Bk[a][b], 
                     where a = {p+1|p} and b = {q+1|q} (see above).
     spn , output ,  Span index


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_SrfRatBasisKnotDeriv */
NL_FLAG N_SrfRatBasisKnotDeriv( NL_SURFACE *sur, NL_INDEX k, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG dir, NL_FLAG flk, NL_FLAG ulp, NL_FLAG vlp, NL_REAL ** Bk, NL_INDEX *spn )
{

    NL_FLAG error = NL_NO;

    NL_DEGREE p, q;

    NL_INDEX i, j, kk, ll = 0;

    NL_REAL *U, *V;

    NL_KNOTVECTOR *knu, *knv;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get locals */

    N_SrfGetDegrees( sur, &p, &q );
    N_SrfGetKnots( sur, &i, &j, &U, &V );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Get basis functions */

    if( dir EQ NL_UDIR )
    {
        error = N_BasisFindSpan( knv, q, v, vlp, &ll );

        if( error EQ NL_YES )
            NL_OUT;

        kk = k;

        if( flk EQ NL_LEFT AND u GT U[k] )
            while( U[kk]EQ U[kk + 1] )
                kk++;

        if( flk EQ NL_RIGHT AND u LT U[k] )
            while( U[kk]EQ U[kk - 1] )
                kk--;

        for ( i = kk - p - 1; i <= kk; i++ )
        {
            for ( j = ll - q; j <= ll; j++ )
            {
                error = N_SrfRatBasisIKnotDeriv( sur, i, j, k, u, v, dir, flk, ulp, vlp, &Bk[i - kk + p + 1][j - ll + q] );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }
    }

    if( dir EQ NL_VDIR )
    {
        error = N_BasisFindSpan( knu, p, u, ulp, &ll );

        if( error EQ NL_YES )
            NL_OUT;

        kk = k;

        if( flk EQ NL_LEFT AND v GT V[k] )
            while( V[kk]EQ V[kk + 1] )
                kk++;

        if( flk EQ NL_RIGHT AND v LT V[k] )
            while( V[kk]EQ V[kk - 1] )
                kk--;

        for ( i = ll - p; i <= ll; i++ )
        {
            for ( j = kk - q - 1; j <= kk; j++ )
            {
                error = N_SrfRatBasisIKnotDeriv( sur, i, j, k, u, v, dir, flk, ulp, vlp, &Bk[i - ll + p][j - kk + q + 1] );

                if( error EQ NL_YES )
                    NL_OUT;
            }
        }
    }

    *spn = ll;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfRatBasisKnotDeriv */

/* Surfaces */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the  Gaussian, mean and principal  curvatures 
     at a surface point by setting various flags. Discontinuous surfaces 
     can also be handled by passing NL_LEFT/NL_RIGHT flags. A typical  calling 
     example is:

       NL_FLAG       pfl;
       NL_SURFACE    sur;
       NL_PARAMETER  u, v;
       NL_REAL       K, H, k1, k2;
       NL_POINT      P;
       NL_VECTOR     N, Su, Sv, U1, U2, T1, T2;
       ...
       (define sur and get u and v);
       ...
       N_SrfEvalPtCurvature(&sur,u,v,NL_LEFT,NL_RIGHT,NL_BOTH,&K,&H,&P,&N,&Su,&Sv,
                &k1,&k2,&U1,&U2,&T1,&T2,&pfl);

     NOTES: (1) AT A SPHERICAL  (UMBILICAL) POINT, THE NORMAL CURVATURES 
                ARE  THE SAME, I.E. NO  PRINCIPAL  DIRECTIONS  EXIST. AT 
                SUCH A  POINT A LOCAL  COORDINATE SYSTEM IS  SUPPLIED AS 
                FOLLOWS: T1=Su, T2=NxSu. U1 AND U2 ARE UNDEFINED. 
            (2) THE  LOCAL  FRAME <T1,T2,N> IS  ORTHONORMAL, HOWEVER, IT 
                IS NOT NECESSARILY A RIGHT HANDED SYSTEM.
            (3) TO OBTAIN  CURVATURES AT  SPHERICAL POINTS AND AT POINTS 
                OF ZERO DERIVATIVES, THE FOLLOWING GLOBAL TOLERANCES ARE
                USED:
                  NL_ZDTL = ZERO DISCRIMINANT TOLERANCE
                  NL_ZCTL = ZERO COEFFICIENT TOLERANCE
                THEY HELP SOLVING  QUADRATIC EQUATIONS IN  CASES OF ZERO
                DISCRIMINANTS AND ZERO  COEFFICIENTS. ALL TOLERANCES ARE
                RELATIVE,  I.E.  THE  COEFFICIENTS   OF  THE   QUADRATIC 
                EQUATION ARE  NORMALIZED!! ALL  GLOBAL TOLERANCES ARE IN
                "globals.h"


   ACCESS:
   
     sur     , input  ,  NURBS surface
     u,v     , input  ,  Parameter values 
     ufl,vfl , input  ,  Flags:
                           NL_LEFT : t is in [t[j],t[j+1])
                                  (NL_RIGHT DERIVATIVES REQUIRED)
                           NL_RIGHT: t is in (t[j],t[j+1]]
                                  (NL_LEFT DERIVATIVES REQUIRED)
                           (t is either u or v)
     dfl     , input  ,  Flag: 
                           NL_NO  : no  principal curvatures  are required,
                                 return    only   Gaussian   and    mean 
                                 curvatures
                           NL_YES : yes,  compute principal curvatures with
                                 principal directions
                           NL_BOTH: compute   both     Gaussian/mean    and 
                                 principal  curvatures  and   directions
     K,H     , output ,  Gaussian and mean curvatures
     P,SN    , output ,  Point and UNIT normal computed at (u,v)
     Su,Sv   , output ,  Partial  derivative  UNIT  vectors  computed at 
                         (u,v)
     k1,k2   , output ,  Principal  curvatures  returned  in  increasing
                         order (k1 <= k2)
     U1,U2   , output ,  Parameter  space  principal  directions  corre-
                         sponding to k1 and k2 (UNIT vectors)
     T1,T2   , output ,  Model space principal directions  corresponding
                         to k1 and k2 (UNIT vectors)
     pfl     , output ,  Flag:
                           NL_NO : no principal directions  exist  at (u,v)
                                (local frame is  supplied in T1  and T2)
                           NL_YES: principal directions returned in  T1, T2


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfEvalPtCurvature( NL_SURFACE *sur, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG dfl, NL_REAL *K, NL_REAL *H, NL_POINT *P, NL_VECTOR *SN, NL_VECTOR *Su, NL_VECTOR *Sv, NL_REAL *k1, NL_REAL *k2, NL_VECTOR *U1, NL_VECTOR *U2, NL_VECTOR *T1, NL_VECTOR *T2, NL_FLAG *pfl )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfEvalPtCurvature");

    NL_FLAG error = NL_NO;

    NL_DEGREE p, q;

    NL_INTEGER n;

    NL_REAL E, F, G, L, M, N, num, den, a, b, c, ud, vd, dot, a11, a12, a21, a22;

    NL_POINT ** D;

    NL_STACKS S;

#ifdef _DEBUG
    /* NL_REAL    calcK1, chkK1, chkMin1, calcK2, chkK2, chkMin2 ; */
#endif

    /* Start NURBS */

    N_InitNurbs( &S );

    /* init output */
    
    *pfl = NL_NO;

    /* Get derivatives */

    N_SrfGetDegrees( sur, &p, &q );

    D = N_AllocPt2dArray( 2, 2, &S );

    if( D EQ NULL )
        NL_QUIT;

    error = N_SrfDerivs( sur, u, v, ufl, vfl, NL_TRUE, 2, 2, D );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get first fundamental forms */

    N_VectorDot( D[1][0], D[1][0], &E );
    N_VectorDot( D[1][0], D[0][1], &F );
    N_VectorDot( D[0][1], D[0][1], &G );

    /* Get second fundamental forms */

    error = N_SrfEvalPtPtDerivNormal( sur, u, v, ufl, vfl, P, Su, Sv, SN );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorDot( *SN, D[2][0], &L );
    N_VectorDot( *SN, D[1][1], &M );
    N_VectorDot( *SN, D[0][2], &N );

    /* Get Gaussian and mean curvatures - no principal directions requested */

    if( dfl EQ NL_NO )
    {
        num = L * N - M * M;
        den = E * G - F * F;

        if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        *K = num / den;

        num = 0.5 *( E * N - 2.0 *F * M + G * L );

        if( N_FloatOpIsBad( num, den, NL_DIVISION ) )
            NL_ERROR( NL_NUM_ERR );

        *H = num / den;

        NL_OUT;
    }

    /* Get principal curvatures and directions */

    if( dfl EQ NL_YES OR dfl EQ NL_BOTH )
    {
        /* Compute principal curvatures first */

        a = E * G - F * F;
        b = 2.0 *M * F - E * N - L * G;
        c = L * N - M * M;

        error = N_SolveQuadraticEq( a, b, c, k1, k2, &n );

        if( error EQ NL_YES )
            NL_OUT;

        if( n LT 1 )
            NL_ERROR( NL_NUM_ERR );

        if( dfl EQ NL_BOTH )
        {
            *K = (*k1) * (*k2);
            *H = 0.5 *( ( *k1)+( *k2) );
        }

        if( n EQ 1 )
        {
            if( p EQ 1 )
                N_VectorCopy( *Su, T1 );

            else if( q EQ 1 )
                N_VectorCopy( *Sv, T1 );

            else
                N_VectorCopy( *Su, T1 );

            N_VectorCross( *SN, *T1, T2 );

            *pfl = NL_NO;

            NL_OUT;
        }

        /* Now compute principal directions */

        a11 = L - (*k1) * E;
        a12 = M - (*k1) * F;
        a21 = M - (*k1) * F;
        a22 = N - (*k1) * G;

        dot = a11 * a21 + a12 * a22;

        if( dot LT 0.0 )
        {
            a11 = -a11;
            a12 = -a12;
        }

        ud = -0.5 *( a12 + a22 );
        vd = 0.5 *( a11 + a21 );

        N_VectorCreate( ud, vd, 0.0, U1 );
        N_VectorCombine( ud, D[1][0], vd, D[0][1], T1 );

        a11 = L - (*k2) * E;
        a12 = M - (*k2) * F;
        a21 = M - (*k2) * F;
        a22 = N - (*k2) * G;

        dot = a11 * a21 + a12 * a22;

        if( dot LT 0.0 )
        {
            a11 = -a11;
            a12 = -a12;
        }

        ud = -0.5 *( a12 + a22 );
        vd = 0.5 *( a11 + a21 );

        N_VectorCreate( ud, vd, 0.0, U2 );
        N_VectorCombine( ud, D[1][0], vd, D[0][1], T2 );

        error = N_VectorNormalizeRef( U1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_VectorNormalizeRef( U2 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_VectorNormalizeRef( T1 );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_VectorNormalizeRef( T2 );

        if( error EQ NL_YES )
            NL_OUT;
        
        /* made it to here after computing valid principal curvature directions */

#ifdef _DEBUG
        /* check results - chkK1, chkMin1, chkK2, chkMin2 should all be zero */
        /* calcK1  =   (L*U1->x*U1->x + 2*M*U1->x*U1->y + N*U1->y*U1->y)     */
        /*           / (E*U1->x*U1->x + 2*F*U1->x*U1->y + G*U1->y*U1->y) ;   */
        /* chkK1   = calcK1 - *k1 ;                                          */
        /* chkMin1 =   (E*M - L*F)*U1->x*U1->x                               */
        /*           + (E*N - L*G)*U1->x*U1->y                               */
        /*           + (F*N - G*M)*U1->y*U1->y ;                             */
        /*                                                                   */
        /* calcK2  =   (L*U2->x*U2->x + 2*M*U2->x*U2->y + N*U2->y*U2->y)     */
        /*           / (E*U2->x*U2->x + 2*F*U2->x*U2->y + G*U2->y*U2->y) ;   */
        /* chkK2   = calcK2 - *k2 ;                                          */
        /* chkMin2 =   (E*M - L*F)*U2->x*U2->x                               */
        /*           + (E*N - L*G)*U2->x*U2->y                               */
        /*           + (F*N - G*M)*U2->y*U2->y ;                             */
#endif
        *pfl = NL_YES;

    } /* end compute principal directions check */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_SrfEvalPtCurvature */

/*******************************************************************//**

   DESCRIPTION:

     This template specializes computing a point on a NURBS surface by using
     curve evaluations only, for NOZ and NOW, by utilizing the specialized/inlined versions of
     N_VectorBlendCPt and N_CPtToPtEuclid. Calling examples and further notes are decribed in the
     description for N_SrfEvalPt.

   Notes:

   ***********************************************************************/
template<NL_BOOLEAN Pnoz, NL_BOOLEAN Pnow>
NL_FLAG N_SrfEvalPtImpl( NL_SURFACE *sur, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_POINT *S )
{

    NL_FLAG error = NL_NO;

    NL_INDEX tmp1, tmp2, usp, vsp;

    NL_DEGREE p, q;

    NL_REAL NU[NL_MAXDEG + 1], NV[NL_MAXDEG + 1];

    NL_KNOTVECTOR *knu, *knv;

    NL_CPOINT ** Pw, Tw, Sw;

    /* Get local notation */

    p = sur->p;
    knu = sur->knu;
    q = sur->q;
    knv = sur->knv;
    Pw = sur->net->Pw;

    /* Compute non-vanishing B-splines */

    error = N_BasisEval( knu, p, u, ufl, NU, &usp );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisEval( knv, q, v, vfl, NV, &vsp );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute the point on the surface */

    Sw = {};

    tmp1 = usp - p;
    tmp2 = vsp - q;

    for ( NL_INDEX i = 0; i <= p; i++ )
    {
        Tw = {};

        const NL_CPOINT * Pw_tmp1i = Pw[tmp1 + i];
        for ( NL_INDEX j = 0; j <= q; j++)
        {
            NI_VectorBlendCPt_specialized(Pnoz, Pnow, NV[j], Pw_tmp1i[tmp2 + j], &Tw);
        }

        NI_VectorBlendCPt_specialized(Pnoz, Pnow, NU[i], Tw, &Sw);
    }

    NI_CPtToPtEuclid_specialized(Pnoz, Pnow, Sw, S );

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_SrfEvalPtImpl */

/*******************************************************************/ /**

    DESCRIPTION:

      This arithmetic routine computes a point on a NURBS surface by using
      curve evaluations only. It utilizes the template N_SrfEvalPtImpl to specialize
      the code for NOZ and NOW.

      A typical calling example is:

        NL_SURFACE    sur;
        NL_PARAMETER  u, v;
        NL_POINT      S;
        ...
        (define sur, get u and v);
        ...
        N_SrfEvalPt(&sur,u,v,NL_LEFT,NL_RIGHT,&S);


    ACCESS:

      sur     , input  ,  NURBS surface
      u,v     , input  ,  Parameter values
      ufl,vfl , input  ,  Flags:
                            NL_LEFT : t is in [t[j],t[j+1])
                            NL_RIGHT: t is in (t[j],t[j+1]]
                            (t is either u or v)
      S       , output ,  Point on the surface


    RETURN CODES:

      0 : No error
      1 : Error saved in NL_ERROR

    ***********************************************************************/

NL_FLAG N_SrfEvalPt( NL_SURFACE *sur, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_POINT *S )
{
    NL_BOOLEAN Pnoz = sur->net->Pw[0][0].z == NL_NOZ;
    NL_BOOLEAN Pnow = sur->net->Pw[0][0].w == NL_NOW;

    if (Pnoz)
    {
        if (Pnow)
        {
            return N_SrfEvalPtImpl<NL_TRUE, NL_TRUE>(sur, u, v, ufl, vfl, S);
        }
        else
        {
            return N_SrfEvalPtImpl<NL_TRUE, NL_FALSE>(sur, u, v, ufl, vfl, S);
        }
    }
    else
    {
        if (Pnow)
        {
            return N_SrfEvalPtImpl<NL_FALSE, NL_TRUE>(sur, u, v, ufl, vfl, S);
        }
        else
        {
            return N_SrfEvalPtImpl<NL_FALSE, NL_FALSE>(sur, u, v, ufl, vfl, S);
        }
    }

} /* end N_SrfEvalPt */


#ifdef USE_NLIB_TESS

/*******************************************************************//**


   DESCRIPTION:

     This routine computes a point on a surface curve defined by a curve 
     in the  surface's parameter domain. That is, it evaluates the point
     S(C(t)_x,C(t)_y), where  S(u,v)  is  the  surface and  C(t)  is the 
     domain curve. A typical calling example is:

       NL_SURFACE    sur;
       NL_CURVE      cur;
       NL_PARAMETER  t;
       NL_POINT      S;
       ...
       (define sur, cur and get t);
       ...
       N_SrfEvalPtCrvOnSrf(&sur,&cur,t,&S);


   ACCESS:
   
     sur  , input  ,  NURBS surface
     cur  , input  ,  Parameter space curve
     t    , input  ,  Parameter of cur
     S    , output ,  Point on the surface curve


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG N_SrfEvalPtCrvOnSrf */
NL_FLAG N_SrfEvalPtCrvOnSrf( NL_SURFACE *sur, NL_CURVE *cur, NL_PARAMETER t, NL_POINT *S )
{

    NL_FLAG error = NL_NO;

    NL_REAL u, v, z;

    NL_POINT C;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Compute the point on the surface curve */

    error = N_CrvEval( cur, t, NL_LEFT, &C );

    if( error EQ NL_YES )
        NL_OUT;

    N_PtToXYZ( C, &u, &v, &z );
    N_ClampSrfAtParams( sur, &u, &v );

    error = N_SrfEvalPt( sur, u, v, NL_LEFT, NL_LEFT, S );

    if( error EQ NL_YES )
        NL_OUT;

    /* EndNURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfEvalPtCrvOnSrf */

#endif // USE_NLIB_TESS

/*******************************************************************/ /**


    DESCRIPTION:

      This routine computes a matrix of derivatives, specialized to the dimension and homogeneity
      of the the description of the surface and inlined/optimized. 

      Optimizations
        1.) Compute N_PtToEuclic only once -> noop, but required for further steps
        2.) Change loop iteration order from i,j,k,l to k, l, i,j. -> -2s 
           2.1) It's no longer to initialize SD[k][l] with 0.
           2.2) sd is local in registers -> no load, modify, write require for the fmad
                downside: P has to be read on each loop iteration which is why 1) is required
        3.) Reduce the amount of pointer indirections in inner loop (~5.1s->~4.7s)

        NL_BOOLEAN    noW;
        NL_BOOLEAN    noZ;
        NL_FLAG       upper_derivative_matrix_only;
        NL_INDEX      p,
        NL_INDEX      q,
        NL_INDEX      udr,
        NL_INDEX      vdr,
        NL_CPOINT **  Pw,
        NL_INDEX      l2,
        NL_INDEX      l3,
        NL_BASISDERIVATIVES& BD,
        NL_POINT **   SD 
        ...
        (define noW, noZ,upper_derivative_matrix_only, get p, q, udr, vdr, Pw, BD);
        ...
        N_ComputeDerivativeMatrixSpec(noW, noZ, upper_derivative_matrix_only, p, q,
                                      udr, vdr, Pw, l2, l3, BD, SD)

    ACCESS:

      noW                            , input ,   Boolean for NOW ( TRUE = nonhomogeneous )
      noZ                            , input ,   Boolean for NOZ ( TRUE = 2D )
      upper_derivative_matrix_only   , input ,   Flag for computation of upper triangular deriviative matrix ( TRUE = upper only )
      p, q                           , input ,   Degrees
      udr, vdr                       , input ,   Highest derivs required
      Pw                             , input ,   Control points
      l2, l3                         , input ,   Dimensions of control points
      BD,                            , input ,   BD[k][l][i][j] is 
                                                 the (k,l)-th derivative of the basis function  
                                                 N[ku-p+i][kv-q+j],  where  u  is  in  {u[ku],
                                                 u[ku+1]} and  v is in  {v[kv],v[kv+1}. 
      SD ,                           , output ,  Derivatives. SD[0][0]=Pos, SD[1][0]=Du, SD[0][1]=Dv, etc
                                                 Sized [udr+1][vdr+1]

    RETURN CODES:

      0 : No error
      1 : Error saved in NL_ERROR

    ***********************************************************************/
NL_INLINE NL_VOID N_ComputeDerivativeMatrixSpec
 (NL_BOOLEAN             noW,                          // in : TRUE = CPTs are not rational not using w coord, FALSE=CPTs are rational   
  NL_BOOLEAN             noZ,                          // in : TRUE = CPTs are 2d not using Z coord, FALSE=CPTs are 3d 
  NL_FLAG                upper_derivative_matrix_only, // in : TRUE = upper triangular deriviative matrix, FALSE = whole matrix  
  NL_INDEX               p,                            // in : U dir degree 
  NL_INDEX               q,                            // in : V dir degree 
  NL_INDEX               udr,                          // in : max deriv cnt to compute in U dir, 0=pos, 1=pos+1stDeriv, 2=pos+1stDeriv+2ndDeriv, ... 
  NL_INDEX               vdr,                          // in : max deriv cnt to compute in V dir, 0=pos, 1=pos+1stDeriv, 2=pos+1stDeriv+2ndDeriv, ... 
  NL_CPOINT           ** Pw,                           // in : CPT array 
  NL_INDEX               l2,                           /* in : offset Pw[l2],   better offset u? */ 
  NL_INDEX               l3,                           /* in : offset Pw[*][l3] better offset v? */ 
  NL_BASISDERIVATIVES  & BD,                           // out: Basis Derivatives for each BSpline span 
  NL_POINT            ** SD)                           // Out: SD[0][0]=Pos, SD[1][0]=Du, SD[0][1]=Dv, etc. , sized:[udr+1][vdr+1] 
{
  NL_POINT Pwl[NL_MAXDEG + 1][NL_MAXDEG + 1];
  
  for (NL_INDEX i = 0; i <= p; i++)
    {
      for (NL_INDEX j = 0; j <= q; j++)
        {
          NI_CPtToPtEuclid_specialized(noZ, noW, Pw[i + l2][j + l3], &Pwl[i][j]);
        }
    }
  
  for (NL_INDEX k = 0; k <= udr; k++)
    {
      for (NL_INDEX l = 0; l <= (upper_derivative_matrix_only ? (udr - k) : vdr); l++)
        {
          NL_POINT sd = {0,0,0};
          for (NL_INDEX i = 0; i <= p; i++)
            {
              NL_POINT* Pwl_i = Pwl[i];
              for (NL_INDEX j = 0; j <= q; j++)
                {
                  NL_POINT P = Pwl_i[j];
                  NL_REAL alpha = BD[k][l][i][j];
                  sd.x = alpha * P.x + sd.x;
                  sd.y = alpha * P.y + sd.y;
                  if (!noZ)
                    { sd.z = alpha * P.z + sd.z; }
                }
            }
          SD[k][l] = sd;
        }
    }
} // end N_ComputeDerivativeMatrixSpec

/*******************************************************************/ /**


    DESCRIPTION:

      This routine computes a matrix of derivatives, specialized to the dimension and homogeneity
      of the the description of the surface, by dispatching to N_ComputeDerivativeMatrixSpec.


        NL_BOOLEAN    noW;
        NL_BOOLEAN    noZ;
        NL_FLAG       upper_derivative_matrix_only;
        NL_INDEX      p,
        NL_INDEX      q,
        NL_INDEX      udr,
        NL_INDEX      vdr,
        NL_CPOINT **  Pw,
        NL_INDEX      l2,
        NL_INDEX      l3,
        NL_BASISDERIVATIVES& BD,
        NL_POINT **   SD
        ...
        (define noW, noZ,upper_derivative_matrix_only, get p, q, udr, vdr, Pw, BD);
        ...
        N_ComputeDerivativeMatrix(noW, noZ, upper_derivative_matrix_only, p, q,
                                      udr, vdr, Pw, l2, l3, BD, SD
)


    ACCESS:

      Same as N_ComputeDerivativeMatrixSpec

    RETURN CODES:

      0 : No error
      1 : Error saved in NL_ERROR

    ***********************************************************************/
/* NL_VOID N_ComputeDerivativeMatrix */
NL_NOINLINE NL_VOID N_ComputeDerivativeMatrix
 (NL_BOOLEAN            noW,                          // in : TRUE = CPTs are not rational not using w coord, FALSE=CPTs are rational  
  NL_BOOLEAN            noZ,                          // in : TRUE = CPTs are 2d not using Z coord, FALSE=CPTs are 3d
  NL_FLAG               upper_derivative_matrix_only, // in : TRUE = upper triangular deriviative matrix, FALSE = whole matrix 
  NL_INDEX              p,                            // in : U dir degree
  NL_INDEX              q,                            // in : V dir degree
  NL_INDEX              udr,                          // in : max deriv cnt to compute in U dir, 0=pos, 1=pos+1stDeriv, 2=pos+1stDeriv+2ndDeriv, ... 
  NL_INDEX              vdr,                          // in : max deriv cnt to compute in V dir, 0=pos, 1=pos+1stDeriv, 2=pos+1stDeriv+2ndDeriv, ...
  NL_CPOINT          ** Pw,                           // in : CPT array
  NL_INDEX              l2,                           /* in :  Dimensions of control points, offset Pw[l2],   better offset u? */
  NL_INDEX              l3,                           /* in :  Dimensions of control points, offset Pw[*][l3] better offset v? */
  NL_BASISDERIVATIVES & BD,                           // out: Basis Derivatives for each BSpline span
  NL_POINT           ** SD)                           // Out: SD[0][0]=Pos, SD[1][0]=Du, SD[0][1]=Dv, etc. , sized:[udr+1][vdr+1]
{
  NL_BOOLEAN bNoW   = (noW ? TRUE : FALSE) ;
  NL_BOOLEAN bNoZ   = (noZ ? TRUE : FALSE) ;
  NL_BOOLEAN bUpper = (upper_derivative_matrix_only ? TRUE : FALSE) ;
  N_ComputeDerivativeMatrixSpec(bNoW, bNoZ, bUpper, p, q, udr, vdr, Pw, l2, l3, BD, SD); 

// replaced this block
//   if(noW) 
//     { if(noZ) // no Z
//         { if (upper_derivative_matrix_only) { N_ComputeDerivativeMatrixSpec(NL_TRUE, NL_TRUE, NL_TRUE,   p, q, udr, vdr, Pw, l2, l3, BD, SD); }
//           else                              { N_ComputeDerivativeMatrixSpec(NL_TRUE, NL_TRUE, NL_FALSE,  p, q, udr, vdr, Pw, l2, l3, BD, SD); }
//         }
//       else    // using Z   
//         { if (upper_derivative_matrix_only) { N_ComputeDerivativeMatrixSpec(NL_TRUE, NL_FALSE, NL_TRUE,  p, q, udr, vdr, Pw, l2, l3, BD, SD); }
//           else                              { N_ComputeDerivativeMatrixSpec(NL_TRUE, NL_FALSE, NL_FALSE, p, q, udr, vdr, Pw, l2, l3, BD, SD); }
//         }
//     } // end no W
// 
//   else // using W   
//     { if (noZ) // no Z
//         { if(upper_derivative_matrix_only) { N_ComputeDerivativeMatrixSpec(NL_FALSE, NL_TRUE, NL_TRUE,   p, q, udr, vdr, Pw, l2, l3, BD, SD); }
//           else                             { N_ComputeDerivativeMatrixSpec(NL_FALSE, NL_TRUE, NL_FALSE,  p, q, udr, vdr, Pw, l2, l3, BD, SD); }
//         }
//       else    // using Z    
//         { if(upper_derivative_matrix_only) { N_ComputeDerivativeMatrixSpec(NL_FALSE, NL_FALSE, NL_TRUE,  p, q, udr, vdr, Pw, l2, l3, BD, SD); }
//           else                             { N_ComputeDerivativeMatrixSpec(NL_FALSE, NL_FALSE, NL_FALSE, p, q, udr, vdr, Pw, l2, l3, BD, SD); }
//         }
//     } // end using W
} // end N_ComputeDerivativeMatrix

/*******************************************************************//**


   DESCRIPTION:

     This routine computes derivatives of a NURBS surface  by evaluating 
     all   non-vanishing  basis  functions  and  their  derivatives, and 
     multiplying  them  by  appropriate  control  points.  Discontinuous  
     surfaces can also be handled by passing NL_LEFT/NL_RIGHT flags. A typical 
     calling example is:

       NL_SURFACE    sur;
       NL_PARAMETER  u, v;
       NL_INDEX      udr, vdr;
       NL_POINT      **SD;
       ...
       (define sur, get u, v, udr, vdr, and allocate memory for SD);
       ...
       N_SrfDerivs(&sur,u,v,NL_LEFT,NL_RIGHT,NL_TRUE,udr,vdr,SD);


   ACCESS:
   
     sur     , input  ,  NURBS surface
     u,v     , input  ,  Parameter values 
     ufl,vfl , input  ,  Flags:
                           NL_LEFT : t is in [t[j],t[j+1])
                                  (NL_RIGHT DERIVATIVES REQUIRED)
                           NL_RIGHT: t is in (t[j],t[j+1]]
                                  (NL_LEFT DERIVATIVES REQUIRED)
                           (t is either u or v)
     mfl     , input  ,  Flag: 
                           NL_TRUE : compute  upper  half  only  of the 
                                  derivative matrix
                           NL_FALSE: compute full derivative matrix
                         (APPLICABLE ONLY IF udr=vdr!)
     udr,vdr , input  ,  Highest derivatives required
     SD      , output ,  Derivatives;   SD[k][l]   is    the   (k,l)-th 
                         derivative. MEMORY FOR SD MUST BE ALLOCATED IN
                         THE CALLING ROUTINE TO BE SIZE:[udr+1][vdr+1].
                         see below under NOTES.

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   NOTES:
     The conversion from an  SD array involves the way NLib does arrays
     The User SD array is defined at application time as 
       NL_POINT  UserArray[2][2]
     and 
       int   udr = vdr = 1

     The NLib SD array is defined as NL_POINT** SD.
     An Array SD[2][2] is not an array of 4 points...it is an (1 d) array of
     pointers-to-points

     **  NL_POINT   Derivs[4];  results in 1d ordered array  or: **
     NL_POINT User_SD[2][2]; ** user defined conventional 2d array **
     NL_INDEX udr = 1; vdr = 1;

     NL_POINT *SD[NL_MAXDER+1];
     for (i=0; i<=udr; i++)
     {
             SD[i] = &UserArray[[i][0];
       ** or  SD[i] = &Derivs[i*(vdr+1)];  **
     }

    call NLib routine with udr, vdr and SD

   ***********************************************************************/
NL_FLAG N_SrfDerivs
  (NL_SURFACE   *sur,    /* target surface       */
   NL_PARAMETER  u,      /* target u param value */
   NL_PARAMETER  v,      /* target v param value */
   NL_FLAG       ufl,    /* U dir boundary flag NL_LEFT=t is in [t[j],t[j+1]), NL_RIGHT=t is in (t[j],t[j+1]] */
   NL_FLAG       vfl,    /* V dir boundary flag NL_LEFT=t is in [t[j],t[j+1]), NL_RIGHT=t is in (t[j],t[j+1]] */
   NL_FLAG       mfl,    /* NL_TRUE = only compute upper half of deriv Mat, NL_FALSE = compute total deriv Mat */
   NL_INDEX      udr,    /* highest u derivative required */
   NL_INDEX      vdr,    /* highest v derivative required */
   NL_POINT    **SD )    /* Output: SD[0][0]=Pos, SD[1][0]=Du, SD[0][1]=Dv, etc. , sized:[udr+1][vdr+1] */
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfDerivs");

    NL_FLAG  error = NL_NO;

    NL_INDEX usp, vsp, l2, l3;

    NL_DEGREE p, q;

    /* NL_KNOTVECTOR *knu, *knv; unused */

    NL_CPOINT ** Pw;

    NL_BASISDERIVATIVES BD;

    NL_BOOLEAN noW;
    NL_BOOLEAN noZ;

    
    if( sur == NULL )
        NL_ERROR( NL_CAL_ERR );

    /* Get local notation */
    p = sur->p;
    /* knu = sur->knu; */
    q = sur->q;
    /* knv = sur->knv; */
    Pw = sur->net->Pw;

    /* Make pointer assignments */

    /* Compute basis function derivatives */
    error = N_SrfBasisDerivs( sur, u, v, ufl, vfl, mfl, udr, vdr, BD, &usp, &vsp );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute derivatives */

    l2 = usp - p;
    l3 = vsp - q;

    noW = (Pw[0][0].w == NL_NOW);
    noZ = (Pw[0][0].z == NL_NOZ);

    N_ComputeDerivativeMatrix(noW, noZ,
        mfl EQ NL_TRUE AND udr EQ vdr, p, q, udr, vdr, Pw, l2, l3, BD, SD);
        
    /* Exit */

    EXIT:

    return (error);
} /* end N_SrfDerivs */


/*******************************************************************//**


   DESCRIPTION:

     This routine computes the min-max box of a NURBS surface. A 
     typical calling example is:

       NL_SURFACE    sur;
       NL_MINMAXBOX  box;
       ...
       (define sur);
       ...
       N_SrfGetBBox(&sur,&box);


   ACCESS:
   
     sur , input  ,  NURBS surface
     box , output ,  Min-max box


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_SrfGetBBox */
NL_FLAG N_SrfGetBBox( NL_SURFACE *sur, NL_MINMAXBOX *box )
{

    NL_FLAG error = NL_NO;

    NL_INDEX n, m, k;

    NL_ENET ntl;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_SrfGetArraySizes( sur, &n, &m, &k, &k );

    /* Map control net */

    error = N_SrfGetENet( sur, 0, n, 0, m, &ntl, &S );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get min-max box */

    N_ENetGetBBox( &ntl, box );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_SrfGetBBox */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the maximum magnitude of position vectors 
     of surface control points. A typical calling example is:

       NL_SURFACE  sur;
       NL_POINT    P;
       NL_REAL     mag;
       ...
       (define sur);
       ...
       N_SrfMaxMagnitudePosVectors(&sur,&P,&mag);


   ACCESS:
   
     sur , input  ,  NURBS surface
     P   , output ,  Longest position vector
     mag , output ,  Magnitude of longest position vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfMaxMagnitudePosVectors( NL_SURFACE *sur, NL_POINT *P, NL_REAL *mag )
{

    NL_INDEX i, j, m, n;

    NL_REAL len;

    NL_CPOINT ** Pw;

    NL_POINT Q;

    /* Get local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Compute maximum position vector */

    N_CPtToPtEuclid( Pw[0][0], &Q );
    N_PtMagnitude( Q, mag );
    N_CopyPt( Q, P );

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtToPtEuclid( Pw[i][j], &Q );
            N_PtMagnitude( Q, &len );

            if( len GT *mag )
            {
                N_CopyPt( Q, P );
                *mag = len;
            }
        }
    }
} /* end N_SrfMaxMagnitudePosVectors */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  routine  transforms a  NURBS surface given a general 4x4 
     transformation matrix. The  input data is  destroyed, i.e. the  
     transformation is done in place. A typical calling example is:
 
       NL_SURFACE  sur;
       NL_RMATRIX  rma;
       ...
       (define sur and rma);
       ...
       N_SrfTransform(&sur,&rma);


   ACCESS:
   
     sur , in/out ,  NURBS surface
     rma , input  ,  4x4 Matrix


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfTransform( NL_SURFACE *sur, NL_RMATRIX *rma )
{

    NL_INDEX i, j, n, m;

    NL_CPOINT ** Pw;

    /* Get local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Transform control points */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_TransformCPt( Pw[i][j], rma, &Pw[i][j] );
        }
    }
} /* end N_SrfTransform */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the minimum and maximum weights, and the
     minimum and  maximum magnitudes of position vectors of control
     points. A typical calling example is:

       NL_SURFACE  sur;
       NL_REAL     wmin, wmax, pmin, pmax;
       ...
       (define sur);
       ...
       N_SrfMinMaxWeightPosVectors(&sur,&wmin,&wmax,&pmin,&pmax);


   ACCESS:
   
     sur       , input  ,  NURBS surface
     wmin,wmax , output ,  Min-max weights
     pmin,pmax , output ,  Min-max position vectors


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfMinMaxWeightPosVectors( NL_SURFACE *sur, NL_REAL *wmin, NL_REAL *wmax, NL_REAL *pmin, NL_REAL *pmax )
{

    NL_INDEX i, j, n, m;

    NL_REAL mag, w;

    NL_CPOINT ** Pw;

    NL_POINT P;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Compute min-max weights */

    if( N_IsSrfRat( sur ) )
    {
        N_CPtGetW( Pw[0][0], wmin );
        N_CPtGetW( Pw[0][0], wmax );

        for ( i = 0; i <= n; i++ )
        {
            for ( j = 0; j <= m; j++ )
            {
                N_CPtGetW( Pw[i][j], &w );

                if( w LT *wmin )
                    *wmin = w;

                if( w GT *wmax )
                    *wmax = w;
            }
        }
    }
    else
    {
        *wmin = NL_NOW;
        *wmax = NL_NOW;
    }

    /* Compute min-max position vector magnitudes */

    N_CPtToPtEuclid( Pw[0][0], &P );
    N_VectorMagnitude( P, &mag );

    *pmin = mag;
    *pmax = mag;

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtToPtEuclid( Pw[i][j], &P );
            N_VectorMagnitude( P, &mag );

            if( mag LT *pmin )
                *pmin = mag;

            if( mag GT *pmax )
                *pmax = mag;
        }
    }

    /* End NURBS */

    N_EndNurbs( &S );
} /* end N_SrfMinMaxWeightPosVectors */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes a point, the unit partial derivatives, and  
     the unit normal at given parameter values. Discontinuos surfaces 
     are  handled by  passing a  NL_LEFT/NL_RIGHT  flag. A  typical calling 
     example is:

       NL_SURFACE    sur;
       NL_PARAMETER  u, v;
       NL_POINT      P;
       NL_VECTOR     SU, SV, N;
       ...
       (define sur, get u and v);
       ...
       N_SrfEvalPtPtDerivNormal(&sur,u,v,NL_LEFT,NL_RIGHT,&P,&SU,&SV,&N);


   ACCESS:
   
     sur     , input  ,  NURBS surface
     u,v     , input  ,  Parameter values 
     ufl,vfl , input  ,  Flag:
                           NL_LEFT : t is in [t[j],t[j+1]) 
                                  (NL_RIGHT DERIVATIVES NL_USED)
                           NL_RIGHT: t is in (t[j],t[j+1]] 
                                  (NL_LEFT DERIVATIVES NL_USED)
                           (t is either u or v)
     P       , output ,  Point on the surface
     SU,SV,N , output ,  Unit partial derivatives and the normal


   PREFERRED ACCESS:
     This avoids unwanted memory management around loops
     SD = N_AllocPt2dArray(1,1,&S);
     error = N_SrfEvalPtPtDerivNormalFast( sur, u, v, ufl, vfl, P, SU, SV,N, SD );

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfEvalPtPtDerivNormal( NL_SURFACE *sur, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_POINT *P, NL_VECTOR *SU, NL_VECTOR *SV, NL_VECTOR *N )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_SrfEvalPtPtDerivNormal"); */
    NL_POINT ** SD;
    NL_FLAG error;
    NL_STACKS S;

    /* Start NURBS */
    N_InitNurbs( &S );

    /* Compute surface derivatives */
    SD = N_AllocPt2dArray( 1, 1, &S );

    if( SD EQ NULL )
        NL_QUIT;

    error = N_SrfEvalPtPtDerivNormalFast( sur, u, v, ufl, vfl, P, SU, SV, N, SD );

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_SrfEvalPtPtDerivNormal */


/**********************************************************************/
/* N_SrfEvalPtPTDERIVNORMALFAST: Compute the surface normal at given parameter values     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This routine computes a point, the unit partial derivatives, and  
     the unit normal at given parameter values. This is a fast version 
     of N_SrfEvalPtPtDerivNormal with no NL_STACKS allocation and free.

       NL_SURFACE    sur;
       NL_PARAMETER  u, v;
       NL_POINT      P;
       NL_VECTOR     SU, SV, N;
       NL_POINT      **SD;
       ...
       (define sur, get u and v);
       ...
       N_SrfEvalPtPtDerivNormalFast(&sur,u,v,NL_LEFT,NL_RIGHT,&P,&SU,&SV,&N, SD);


   ACCESS:
   
     sur     , input  ,  NURBS surface
     u,v     , input  ,  Parameter values 
     ufl,vfl , input  ,  Flag:
                           NL_LEFT : t is in [t[j],t[j+1]) 
                                  (NL_RIGHT DERIVATIVES NL_USED)
                           NL_RIGHT: t is in (t[j],t[j+1]] 
                                  (NL_LEFT DERIVATIVES NL_USED)
                           (t is either u or v)
     P       , output ,  Point on the surface
     SU,SV,N , output ,  Unit partial derivatives and the normal
     SD      , input  ,  Array allocated for N_SrfDerivs() call


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfEvalPtPtDerivNormalFast( NL_SURFACE *sur, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_POINT *P, NL_VECTOR *SU, NL_VECTOR *SV, NL_VECTOR *N, NL_POINT ** SD )
{

    /* NL_PRIVATE NL_STRING rname = _T("N_SrfEvalPtPtDerivNormalFast"); */

    NL_FLAG error = NL_NO;

    NL_REAL mag;

/*  NL_KNOTVECTOR *knu, *knv; */

    /* Init outputs */
    N_VectorCopy( NL_ZERO, P  );
    N_VectorCopy( NL_ZERO, SU );
    N_VectorCopy( NL_ZERO, SV );
    N_VectorCopy( NL_ZERO, N  );

   /* Don't have to check out of bounds. */
   /* Get local notation */
/*
    N_SrfGetKnotVectors( sur, &knu, &knv );
    error = N_KnotVectorIsParamOutOfBounds( knu, u, rname );
    if( error EQ NL_YES )
       NL_OUT;
    error = N_KnotVectorIsParamOutOfBounds( knv, v, rname );
    if( error EQ NL_YES )
       NL_OUT;
*/

    /* Compute surface derivatives */

    error = N_SrfDerivs( sur, u, v, ufl, vfl, NL_TRUE, 1, 1, SD );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get point and tangents */

    N_CopyPt( SD[0][0], P );

    N_VectorCopy( SD[1][0], SU );
    N_VectorCopy( SD[0][1], SV );

    /* Get unit tangents and the normal */

    N_VectorMagnitude( *SU, &mag );

    if( mag LE 0.001 *NL_MTOL )
      { return N_SrfEvalPtPtDerivNormalPole( sur,u,v,ufl,vfl,P,SU,SV,N); }
      /* NL_ERROR( NL_GEO_ERR ); */  /* [B389]  */

    error = N_VectorNormalizeRef( SU );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorMagnitude( *SV, &mag );

    if( mag LE 0.001 *NL_MTOL )
      { return N_SrfEvalPtPtDerivNormalPole( sur,u,v,ufl,vfl,P,SU,SV,N); }
      /* NL_ERROR( NL_GEO_ERR ); */

    error = N_VectorNormalizeRef( SV );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorCross( *SU, *SV, N );
    N_VectorMagnitude( *N, &mag );

    if( mag LE NL_LTOL )
      { return N_SrfEvalPtPtDerivNormalPole( sur,u,v,ufl,vfl,P,SU,SV,N); }
      /* NL_ERROR( NL_GEO_ERR ); */

    error = N_VectorNormalizeRef( N );

    if( error EQ NL_YES )
        NL_OUT;

    EXIT:

    return (error);
} /* end N_SrfEvalPtPtDerivNormalFast */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine  unclamps a  NURBS surface  IN PLACE. That is, given a
     surface defined by  two knot vectors that have their end knots with
     multiplicities = (p+1,q+1), where p and q are the u- and v-degrees,
     respectively. This routine  computes a precise  representation with 
     two new knot vectors that do not have multiple knots at the ends. A
     typical calling example is:

       NL_SURFACE  sur;
       ...
       (define sur);
       ...
       N_SrfUnclamp(&sur,NL_UVDIR);


   ACCESS:
   
     sur , in/out ,  NURBS surface to be unclamped
     dir , input  ,  Flag:
                       NL_UDIR : Unclamp in u-direction
                       NL_VDIR : Unclamp in v-direction
                       NL_UVDIR: Unclamp in both directions


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfUnclamp( NL_SURFACE *sur, NL_FLAG dir )
{

    NL_INDEX i, j, k, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V, alf, bet;

    NL_CPOINT ** Pw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    /* Unlamp in u-direction */

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        /* Unclamp at the left end */

        for ( i = 0; i <= p - 2; i++ )
        {
            U[p - i - 1] = U[p - i] - U[n - i + 1] + U[n - i];

            for ( j = i; j >= 0; j-- )
            {
                alf = (U[p] - U[p + j - i - 1]) / (U[p + j + 1] - U[p + j - i - 1]);
                bet = alf / (alf - 1.0);
                alf = 1.0 - bet;

                for ( k = 0; k <= m; k++ )
                {
                    N_Combine2CPts( alf, Pw[j][k], bet, Pw[j + 1][k], &Pw[j][k] );
                }
            }
        }
        U[0] = U[1] - U[n - p + 2] + U[n - p + 1];

        /* Unlamp at the right end */

        for ( i = 0; i <= p - 2; i++ )
        {
            U[n + i + 2] = U[n + i + 1] + U[p + i + 1] - U[p + i];

            for ( j = i; j >= 0; j-- )
            {
                alf = (U[n + 1] - U[n - j]) / (U[n - j + i + 2] - U[n - j]);
                bet = (alf - 1.0) / alf;
                alf = 1.0 - bet;

                for ( k = 0; k <= m; k++ )
                {
                    N_Combine2CPts( alf, Pw[n - j][k], bet, Pw[n - j - 1][k], &Pw[n - j][k] );
                }
            }
        }
        U[r] = U[r - 1] + U[2 * p] - U[2 * p - 1];
    }

    /* Unlamp in v-direction */

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        /* Unclamp at the left end */

        for ( i = 0; i <= q - 2; i++ )
        {
            V[q - i - 1] = V[q - i] - V[m - i + 1] + V[m - i];

            for ( j = i; j >= 0; j-- )
            {
                alf = (V[q] - V[q + j - i - 1]) / (V[q + j + 1] - V[q + j - i - 1]);
                bet = alf / (alf - 1.0);
                alf = 1.0 - bet;

                for ( k = 0; k <= n; k++ )
                {
                    N_Combine2CPts( alf, Pw[k][j], bet, Pw[k][j + 1], &Pw[k][j] );
                }
            }
        }
        V[0] = V[1] - V[m - q + 2] + V[m - q + 1];

        /* Unlamp at the right end */

        for ( i = 0; i <= q - 2; i++ )
        {
            V[m + i + 2] = V[m + i + 1] + V[q + i + 1] - V[q + i];

            for ( j = i; j >= 0; j-- )
            {
                alf = (V[m + 1] - V[m - j]) / (V[m - j + i + 2] - V[m - j]);
                bet = (alf - 1.0) / alf;
                alf = 1.0 - bet;

                for ( k = 0; k <= n; k++ )
                {
                    N_Combine2CPts( alf, Pw[k][m - j], bet, Pw[k][m - j - 1], &Pw[k][m - j] );
                }
            }
        }
        V[s] = V[s - 1] + V[2 * q] - V[2 * q - 1];
    }

    /* End NURBS */

    N_EndNurbs( &S );
} /* end N_SrfUnclamp */

/*******************************************************************//**


   DESCRIPTION:

     This routine unclamps a NURBS  surface  IN PLACE. That is, given a
     surface defined by two knot vectors that have their end knots with
     multiplicities=(p+1,q+1), where  p and q are the u- and v-degrees,
     respectively. This routine  computes a precise representation with 
     two new  knot vectors that do not have multiple knots at the ends.
     The new knots at the ends  are given  in the  form of two new knot 
     vectors. The old & the new knot vectors must agree on the interior 
     and end knot values as shown below:

                  ||||_________|________|________||||  
                       (old clamped knot vector)

         |___|__|____|_________|________|________|____|___|____|     
                      (new unclamped knot vector)

     A typical calling example is:

       NL_SURFACE     sur;
       NL_KNOTVECTOR  knu, knv;
       ...
       (define sur, knu and knv);
       ...
       N_SrfUnclampKnotVector(&sur,&knu,&knv,NL_VDIR);


   ACCESS:
   
     sur , in/out ,  NURBS surface to be unclamped
     knu , input  ,  New u-knot vector
     knv , input  ,  New v-knot vector
     dir , input  ,  Flag:
                       NL_UDIR : Unclamp in u-direction
                       NL_VDIR : Unclamp in v-direction
                       NL_UVDIR: Unclamp in both directions


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfUnclampKnotVector( NL_SURFACE *sur, NL_KNOTVECTOR *knu, NL_KNOTVECTOR *knv, NL_FLAG dir )
{

    NL_INDEX i, j, k, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *U, *V, *US, *VS, alf, bet;

    NL_CPOINT ** Pw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &US, &VS );
    N_KnotVectorGetKnots( knu, &r, &U );
    N_KnotVectorGetKnots( knv, &s, &V );

    /* Unlamp in u-direction */

    if( dir EQ NL_UDIR OR dir EQ NL_UVDIR )
    {
        /* Unclamp at the left end */

        for ( i = 0; i <= p - 2; i++ )
        {
            for ( j = i; j >= 0; j-- )
            {
                alf = (U[p] - U[p + j - i - 1]) / (U[p + j + 1] - U[p + j - i - 1]);
                bet = alf / (alf - 1.0);
                alf = 1.0 - bet;

                for ( k = 0; k <= m; k++ )
                {
                    N_Combine2CPts( alf, Pw[j][k], bet, Pw[j + 1][k], &Pw[j][k] );
                }
            }
        }

        /* Unlamp at the right end */

        for ( i = 0; i <= p - 2; i++ )
        {
            for ( j = i; j >= 0; j-- )
            {
                alf = (U[n + 1] - U[n - j]) / (U[n - j + i + 2] - U[n - j]);
                bet = (alf - 1.0) / alf;
                alf = 1.0 - bet;

                for ( k = 0; k <= m; k++ )
                {
                    N_Combine2CPts( alf, Pw[n - j][k], bet, Pw[n - j - 1][k], &Pw[n - j][k] );
                }
            }
        }

        /* Copy knot vector */

        for ( i = 0; i <= r; i++ )
            US[i] = U[i];
    }

    /* Unlamp in v-direction */

    if( dir EQ NL_VDIR OR dir EQ NL_UVDIR )
    {
        /* Unclamp at the left end */

        for ( i = 0; i <= q - 2; i++ )
        {
            for ( j = i; j >= 0; j-- )
            {
                alf = (V[q] - V[q + j - i - 1]) / (V[q + j + 1] - V[q + j - i - 1]);
                bet = alf / (alf - 1.0);
                alf = 1.0 - bet;

                for ( k = 0; k <= n; k++ )
                {
                    N_Combine2CPts( alf, Pw[k][j], bet, Pw[k][j + 1], &Pw[k][j] );
                }
            }
        }

        /* Unlamp at the right end */

        for ( i = 0; i <= q - 2; i++ )
        {
            for ( j = i; j >= 0; j-- )
            {
                alf = (V[m + 1] - V[m - j]) / (V[m - j + i + 2] - V[m - j]);
                bet = (alf - 1.0) / alf;
                alf = 1.0 - bet;

                for ( k = 0; k <= n; k++ )
                {
                    N_Combine2CPts( alf, Pw[k][m - j], bet, Pw[k][m - j - 1], &Pw[k][m - j] );
                }
            }
        }

        /* Copy knot vector */

        for ( j = 0; j <= s; j++ )
            US[j] = V[j];
    }

    /* End NURBS */

    N_EndNurbs( &S );
} /* end N_SrfUnclampKnotVector */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine scales a  NURBS surface  with respect to a point. A 
     typical calling example is:

       NL_POINT    C;
       NL_VECTOR   f;
       NL_SURFACE  sur;
       ...
       (define sur, get C and scaling vector f)
       ...
       N_SrfScale(&sur,C,f);

     The  scaling  is  done  in-place,  i.e. the  original surface is 
     destroyed.


   ACCESS:
   
     sur , in/out ,  NURBS surface
     C   , input  ,  Center of scaling
     f   , input  ,  Vector-valued scaling factor


   RETURN CODES:

     None

   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_VOID N_SrfScale( NL_SURFACE *sur, NL_POINT C, NL_VECTOR f )
{

    NL_INDEX i, j, n, m;

    NL_CPOINT ** Pw;

    /* Get local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Get scaled control points */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_ScaleCPtWithPtAndVector( Pw[i][j], C, f, &Pw[i][j] );
        }
    }
} /* end N_SrfScale */



/*******************************************************************//**


   DESCRIPTION:

     This  routine  translates a  NURBS  surface. A  typical  calling 
     example is as follows:

       NL_VECTOR   T;
       NL_SURFACE  sur;
       ...
       (define sur, get translation vector T);
       ...
       N_SrfTranslate(&sur,T);

     The  translation is done in-place,  i.e. the original surface is 
     destroyed.


   ACCESS:
   
     sur , in/out ,  NURBS surface
     T   , input  ,  Translation vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfTranslate( NL_SURFACE *sur, NL_VECTOR T )
{

    NL_INDEX i, j, n, m;

    NL_CPOINT ** Pw;

    /* Get local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Get translated control points */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_TranslateCPt( Pw[i][j], T, &Pw[i][j] );
        }
    }
} /* end N_SrfTranslate */


/*******************************************************************//**


   DESCRIPTION:

     This  routine  rotates a  NURBS surface  about a general axis. A 
     typical calling example is:

       NL_POINT    P;
       NL_VECTOR   V;
       NL_REAL     al;
       NL_SURFACE  sur;
       ...
       (define sur, get rotation parameters P, V and al);
       ...
       N_SrfRotateAtPt(&sur,P,V,al);

     The  rotation is  done  in-place,  i.e. the  original surface is 
     destroyed.


   ACCESS:
   
     sur , in/out ,  NURBS surface
     P,V , input  ,  Point and vector of rotation axis
     al  , input  ,  Rotation angle


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfRotateAtPt( NL_SURFACE *sur, NL_POINT P, NL_VECTOR V, NL_REAL al )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m;

    NL_RMATRIX rma;

    NL_CPOINT ** Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPts( sur, &n, &m, &Pw );

    /* Get rotation matrix */

    N_InitRealMatrix( &rma );
    error = N_CreateRotationMatrixAboutAxis( P, V, al, &rma, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get rotated control points */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_TransformCPt( Pw[i][j], &rma, &Pw[i][j] );
        }
    }

    /* End NURBS and exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfRotateAtPt */

#if NLIB_UNUSED


/*******************************************************************//**


   DESCRIPTION:

     This  surface  routine  projects a  surface onto  a  plane. Either 
     parallel  or  perspective  projection can  be used. If  the output  
     surface is  initialized to the  NULL surface,  memory to store new 
     control  points and  knots is  allocated. If the output surface is 
     the same as the input surface, projection is done in place and the 
     original  surface is  destroyed. A  typical calling example is: 

       NL_SURFACE  surP, surQ;
       NL_POINT    O;
       NL_VECTOR   N, E;
       NL_STACKS   SQ;
       ...
       (define surP, get projection parameters);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfProjectOntoPlane(&surP,O,N,E,NL_PARALLEL   ,&surQ,&SQ);
       N_SrfProjectOntoPlane(&surP,O,N,E,NL_PERSPECTIVE,&surP,&SQ);

     If memory is  available, surQ is not  initialized and the  routine
     assumes that  memory allocation  has been done. However, it checks  
     for the proper  amount by looking at the highest indexes in surQ's  
     knot vector and control net objects.


   ACCESS:
   
     surP , input  ,  NURBS surface
     O    , input  ,  Point on the plane of projection
     N    , input  ,  Normal to the plane of projection
     E    , input  ,  Direction of projection  (NL_PARALLEL) or the center 
                      of projection (NL_PERSPECTIVE)
     prj  , input  ,  Flag:
                        NL_PARALLEL   : parallel projection required
                        NL_PERSPECTIVE: perspective projection required
     surQ , output ,  Surface after projection
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfProjectOntoPlane( NL_SURFACE *surP, NL_POINT O, NL_VECTOR N, NL_VECTOR E, NL_FLAG prj, NL_SURFACE *surQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfProjectOntoPlane");

    NL_FLAG error = NL_NO, rat = NL_NO;

    NL_INDEX i, j, n, m, r, s;

    NL_DEGREE p, q;

    NL_REAL *UP, *VP, *UQ, *VQ, alf, bet, ne, nop, nep, neo, w;

    NL_POINT P, Q;

    NL_VECTOR OP, NN, EN, EO, EP;

    NL_CPOINT ** Pw, ** Qw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );

    /* See if memory is needed */

    if( surP EQ surQ )
    {
        N_SrfGetCPtsAndKnots( surP, &Qw, &UQ, &VQ );
    }
    else
    {
        error = N_SrfSizeArrays( surQ, n, m, p, q, r, s, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );
    }

    /* Compute projections */

    if( N_IsSrfRat( surP ) )
        rat = NL_YES;

    switch( prj )
    {
        case NL_PARALLEL:

            error = N_VectorNormalize( N, &NN, &ne );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_VectorNormalize( E, &EN, &ne );

            if( error EQ NL_YES )
                NL_OUT;

            N_VectorDot( NN, EN, &ne );

            if( fabs( ne )LT NL_LTOL )
                NL_ERROR( NL_INP_ERR );

            N_VectorDot( N, E, &ne );

            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= m; j++ )
                {
                    N_CPtToPtEuclid( Pw[i][j], &P );
                    N_VectorDiff( O, P, &OP );
                    N_VectorDot( N, OP, &nop );
                    bet = nop / ne;
                    N_Combine2Pts( 1.0, P, bet, E, &Q );

                    if( rat )
                    {
                        N_CPtGetW( Pw[i][j], &w );
                        N_Weight( Q, w, &Qw[i][j] );
                    }
                    else
                    {
                        N_PtToCPt( Q, &Qw[i][j] );
                    }
                }
            }
            break;

        case NL_PERSPECTIVE:

            N_VectorDiff( E, O, &EO );
            N_VectorDot( N, EO, &neo );

            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= m; j++ )
                {
                    N_CPtToPtEuclid( Pw[i][j], &P );
                    N_VectorDiff( E, P, &EP );
                    N_VectorDot( N, EP, &nep );

                    if( N_FloatOpIsBad( neo, nep, NL_DIVISION ) )
                        NL_ERROR( NL_NUM_ERR );

                    alf = neo / nep;
                    bet = 1.0 - alf;
                    N_Combine2Pts( alf, P, bet, E, &Q );

                    if( rat )
                    {
                        N_CPtGetW( Pw[i][j], &w );
                        w = w * nep;
                        N_Weight( Q, w, &Qw[i][j] );
                    }
                    else
                    {
                        N_PtToCPt( Q, &Qw[i][j] );
                    }
                }
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Copy knot vector if new surface is computed */

    if( surP NEQ surQ )
    {
        for ( i = 0; i <= r; i++ )
            UQ[i] = UP[i];

        for ( j = 0; j <= s; j++ )
            VQ[j] = VP[j];
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfProjectOntoPlane */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This surface routine reverses a  surface, i.e. it  computes a  new 
     surface that is traced out in  reverse order either in u- or in v-
     direction. If  the  output  surface  is  initialized  to the  NULL 
     surface,  memory  to  store  new   control  points  and  knots  is 
     allocated. If the output surface is the same as the input surface, 
     reversal is done in place and the original surface is destroyed. A 
     typical calling example is:

       NL_SURFACE  surP, surQ;
       NL_STACKS   SQ;
       ...
       (define surP);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfReverse(&surP,NL_UDIR,&surQ,&SQ);
       N_SrfReverse(&surP,NL_VDIR,&surP,&SQ);

     If memory is  available, surQ is not  initialized and the  routine
     assumes that  memory allocation  has been done. However, it checks  
     for the proper  amount by looking at the highest indexes in surQ's  
     knot vector and  polygon objects.


   ACCESS:
   
     surP , input  ,  NURBS surface to be reversed
     dir  , input  ,  Flag:
                        NL_UDIR: Reverse in u-direction
                        NL_VDIR: Reverse in v-direction
     surQ , output ,  Surface after reversal
     SQ   , input  ,  surQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfReverse( NL_SURFACE *surP, NL_FLAG dir, NL_SURFACE *surQ, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfReverse");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, n, m, r, s, k;

    NL_DEGREE p, q;

    NL_REAL *UP, *VP, *UQ = NULL, *VQ = NULL, c, a;

    NL_CPOINT ** Pw, ** Qw = NULL;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( surP, &n, &m, &Pw, &p, &q, &r, &s, &UP, &VP );

    /* See if memory is needed */

    if( surP NEQ surQ )
    {
        error = N_SrfSizeArrays( surQ, n, m, p, q, r, s, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_SrfGetCPtsAndKnots( surQ, &Qw, &UQ, &VQ );
    }

    /* Reverse surface */

    switch( dir )
    {
        case NL_UDIR:

            c = UP[0] + UP[r];

            if( surP NEQ surQ )
            {
                for ( i = 0; i <= p; i++ )
                {
                    UQ[i] = UP[i];
                    UQ[n + i + 1] = UP[n + i + 1];
                }
            }

            if( surP EQ surQ )
            {
                k = n / 2;

                for ( j = 0; j <= m; j++ )
                {
                    for ( i = 0; i <= k; i++ )
                    {
                        N_SwapCPts( &Pw[i][j], &Pw[n - i][j] );
                    }
                }

                k = r / 2 - p;

                for ( i = 1; i <= k; i++ )
                {
                    a = UP[r - p - i];
                    UP[r - p - i] = c - UP[p + i];
                    UP[p + i] = c - a;
                }
            }
            else
            {
                for ( j = 0; j <= m; j++ )
                {
                    for ( i = 0; i <= n; i++ )
                    {
                        N_CopyCPt( Pw[n - i][j], &Qw[i][j] );
                    }
                }

                k = r - 2 * p - 1;

                for ( i = 1; i <= k; i++ )
                    UQ[r - p - i] = c - UP[p + i];

                for ( j = 0; j <= s; j++ )
                    VQ[j] = VP[j];
            }
            break;

        case NL_VDIR:

            c = VP[0] + VP[s];

            if( surP NEQ surQ )
            {
                for ( j = 0; j <= q; j++ )
                {
                    VQ[j] = VP[j];
                    VQ[m + j + 1] = VP[m + j + 1];
                }
            }

            if( surP EQ surQ )
            {
                k = m / 2;

                for ( i = 0; i <= n; i++ )
                {
                    for ( j = 0; j <= k; j++ )
                    {
                        N_SwapCPts( &Pw[i][j], &Pw[i][m - j] );
                    }
                }

                k = s / 2 - q;

                for ( j = 1; j <= k; j++ )
                {
                    a = VP[s - q - j];
                    VP[s - q - j] = c - VP[q + j];
                    VP[q + j] = c - a;
                }
            }
            else
            {
                for ( i = 0; i <= n; i++ )
                {
                    for ( j = 0; j <= m; j++ )
                    {
                        N_CopyCPt( Pw[i][m - j], &Qw[i][j] );
                    }
                }

                k = s - 2 * q - 1;

                for ( i = 0; i <= r; i++ )
                    UQ[i] = UP[i];

                for ( j = 1; j <= k; j++ )
                    VQ[s - q - j] = c - VP[q + j];
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfReverse */

#if NLIB_UNUSED

/**********************************************************************/
/* N_SrfLargeExtendToCrv: Extend a surface to a curve for large NL_CMAX extensions*/
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This surface routine extends a Nurbs surface to a curve. That is,
     the given curve becomes the new boundary of  the  surface. Either 
     the start or end of either the u- or v-direction can be extended.  
     After extension the continuity is NL_CMAX and the original parameter
     original parameter domain still maps to the original surface (ie. 
     the parameter domain is also extended. This is special case of 
     N_SrfExtendToCrv when large NL_CMAX extensions are required but the extension
     creates a bad, possibly self-intersecting surface. 
     A typical calling example is:

       NL_SURFACE   surP, surQ;
       NL_CURVE     newbdy;
       NL_STACKS    SP, SQ;
       NL_REAL      xFactor
       ...
       (define surP and newbdy);
       ...
       N_SrfInitArrays(&surQ);
       N_SrfLargeExtendToCrv(&surP,&newbdy,NL_UDIR,NL_START,xFactor,&surQ,&SP,&SQ);
       N_SrfLargeExtendToCrv(&surP,&newbdy,NL_VDIR,NL_END,xFactor,&surP,&SP,&SP);

     If memory is  available, surQ is not  initialized and the  routine
     assumes  that memory  allocation has been done. However, it checks  
     for the proper amount  by looking at the highest indexes in surQ's  
     knot vector and polygon objects.


   ACCESS:
   
     surP   , input  ,  NURBS surface 
     newbdy , input  ,  New boundary curve
     dflg   , input  ,  Flag:
                         NL_UDIR : The surface is extended across either the
                                u=umin or u=umax boundary
                         NL_VDIR : The surface is extended across either the  
                                v=vmin or v=vmax boundary
     end    , input  ,  Flag:
                         NL_START: The surface is extended across either the 
                                u=umin or v=vmin boundary (see dflg)
                         NL_END  : The surface is extended across either the 
                                u=umax or v=vmax boundary (see dflg)
     xFactor, input  ,  NL_REAL:   0 < sFactor < 0.2  0.1 is a good choice
                                This contols the max knot spacing of the
                                two knots that are add at the end of the 
                                of the surface
     surQ , output ,  Surface after extension
     SP   , input  ,  surP's stack
     SQ   , input  ,  surQ's stack

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfLargeExtendToCrv( NL_SURFACE *surP, NL_CURVE *newbdy, NL_FLAG dirUorV, NL_FLAG atSorE, NL_REAL xFactor, NL_SURFACE *surQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_SrfLargeExtendToCrv"); */
    /* The algorithm is designed to give an improved NL_CMAX extension 
           in the case when NL_CMAX fails but a NL_G1 extension is acceptable
           The process is to build a NL_CMAX extension and a NL_G1 extension 
           and then reset two CPOINTs of the NL_CMAX to those of NL_G1. */

    NL_SURFACE SrfG1;
    NL_SURFACE *SrfCMAX;

    /* get the defining data for each srf */
    NL_INDEX nC, mC, rC, sC;
    NL_DEGREE pC, qC;
    NL_REAL *UC, *VC;
    NL_CPOINT ** PwC;
    NL_INDEX nG, mG, rG, sG;
    NL_DEGREE pG, qG;
    NL_REAL *UG, *VG;
    NL_CPOINT ** PwG;
    NL_INDEX ii;
    NL_REAL dt, uNew, vNew;
    NL_REAL t0, t1, t2;
    NL_INDEX rp;
    NL_REAL tn, tn_1, tn_2;
    NL_INDEX sq;

    NL_FLAG error = NL_NO;
    NL_STACKS SL; /* local NL_STACKS for the NL_G1 extension. */

    /* Start NURBS */

    N_InitNurbs( &SL );

    N_SrfInitArrays( &SrfG1 );

    /* SrfG1 starts as a copy of surP */
    error = N_SrfCopy( surP, &SrfG1, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    SrfCMAX = surQ;
    error = N_SrfExtendToCrv( surP, newbdy, dirUorV, atSorE, NL_CMAX, SrfCMAX, SP, SQ );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SrfExtendToCrv( &SrfG1, newbdy, dirUorV, atSorE, NL_G1, &SrfG1, &SL, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsDegreesAndKnots( SrfCMAX, &nC, &mC, &PwC, &pC, &qC, &rC, &sC, &UC, &VC );

    N_SrfGetCPtsDegreesAndKnots( &SrfG1, &nG, &mG, &PwG, &pG, &qG, &rG, &sG, &UG, &VG );

    if( dirUorV == NL_UDIR )
    {
        if( atSorE == NL_START )
        {
            /* this extension was at the start in the U dir so work around U[p]
             initially U[p-1] = 0
             add two knots to SrfCMAX between U[p] and U[p+1] */

            t0 = UC[pC];     /* start param in U dir */
            t1 = UC[pC + 1]; /* should be start parm of srf before any extension */
            t2 = UC[pC + 2];
            dt = t2 - t1;    /* length of first span before and extension  */
            /* require dt to be < mFact*(t1-t0) */
            if( dt > xFactor * (t1 - t0) )
                dt = xFactor * (t1 - t0);

            /* add two u knots to SrfCMAX  */
            uNew = t1 - 0.5 *dt;
            error = N_SrfInsertKnot( SrfCMAX, uNew, 1, NL_UDIR, SrfCMAX, SQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
            uNew = t1 - dt;
            error = N_SrfInsertKnot( SrfCMAX, uNew, 1, NL_UDIR, SrfCMAX, SQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;

            /* SrfCMAX now has two more u spans at the start of the srf */
            /* reset PwC[1] and PwC[2] by using PwG1[1] and PwG1[2] of SrfG1  */
            /* need to get the defining data again since knots have been added */
            N_SrfGetCPtsDegreesAndKnots( SrfCMAX, &nC, &mC, &PwC, &pC, &qC, &rC, &sC, &UC, &VC );

            for ( ii = 0; ii <= mC; ii++ )
            {
                N_CopyCPt( PwG[1][ii], &PwC[1][ii] );
                N_CopyCPt( PwG[2][ii], &PwC[2][ii] );
            }
        }
        else /* at end in NL_UDIR */
        {
            /* this extension was at the end in the U dir so work around U[r-p] */
            /* initially U[r-p-1] = 1  */
            /* add two knots to SrfCMAX between U[r-p-1] and U[r-p]  */
            rp = rC - pC;
            tn = UC[rp];       /* end param in U dir */
            tn_1 = UC[rp - 1]; /* should be end parm of srf before any extension */
            tn_2 = UC[rp - 2];
            dt = tn_1 - tn_2;  /* length of last span before and extension */
            /* require dt to be < mFact*(tn-tn_1) */
            if( dt > xFactor * (tn - tn_1) )
                dt = xFactor * (tn - tn_1);

            /* add two u knots to SrfCMAX */
            uNew = tn_1 + 0.5 *dt;
            error = N_SrfInsertKnot( SrfCMAX, uNew, 1, NL_UDIR, SrfCMAX, SQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
            uNew = tn_1 + dt;
            error = N_SrfInsertKnot( SrfCMAX, uNew, 1, NL_UDIR, SrfCMAX, SQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;

            /* SrfCMAX now has two more u spans at the end of the srf */
            /* reset PwC[nC-2] and PwC[nC-1] by using PwG1[nG-2] and PwG1[nG-1] of SrfG1 */
            /* need to get the defining data again since knots have been added  */
            N_SrfGetCPtsDegreesAndKnots( SrfCMAX, &nC, &mC, &PwC, &pC, &qC, &rC, &sC, &UC, &VC );

            for ( ii = 0; ii <= mC; ii++ )
            {
                N_CopyCPt( PwG[nG - 2][ii], &PwC[nC - 2][ii] );
                N_CopyCPt( PwG[nG - 1][ii], &PwC[nC - 1][ii] );
            }
        }
    }
    else /* dirUorV = NL_VDIR  */
    {
        if( atSorE == NL_START )
        {
            /* this extension was at the start in the V dir so work around V[q] */
            /* initially V[q-1] = 0  */
            /* add two knots to SrfCMAX between V[q] and V[q+1] */

            t0 = VC[qC];     /* start param in V dir */
            t1 = VC[qC + 1]; /* should be start parm of srf before any extension */
            t2 = VC[qC + 2];
            dt = t2 - t1;    /* length of first span before and extension */
            /* require dt to be < mFact*(t1-t0)  */
            if( dt > xFactor * (t1 - t0) )
                dt = xFactor * (t1 - t0);

            /* add two v knots to SrfCMAX  */
            vNew = t1 - 0.5 *dt;
            error = N_SrfInsertKnot( SrfCMAX, vNew, 1, NL_VDIR, SrfCMAX, SQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
            vNew = t1 - dt;
            error = N_SrfInsertKnot( SrfCMAX, vNew, 1, NL_VDIR, SrfCMAX, SQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;

            /* SrfCMAX now has two more v spans at the start of the srf */
            /* reset PwC[1] and PwC[2] by using PwG1[1] and PwG1[2] of SrfG1  */
            /* need to get the defining data again since knots have been added */
            N_SrfGetCPtsDegreesAndKnots( SrfCMAX, &nC, &mC, &PwC, &pC, &qC, &rC, &sC, &UC, &VC );

            for ( ii = 0; ii <= nC; ii++ )
            {
                N_CopyCPt( PwG[ii][1], &PwC[ii][1] );
                N_CopyCPt( PwG[ii][2], &PwC[ii][2] );
            }
        }
        else /* at end in NL_VDIR */
        {
            /* this extension was at the end in the V dir so work around U[s-q] */
            /* initially V[s-q-1] = 1 */
            /* add two knots to SrfCMAX between V[s-q-1] and V[s-q] */
            sq = sC - qC;
            tn = VC[sq];       /* end param in V dir */
            tn_1 = VC[sq - 1]; /* should be end parm of srf before any extension */
            tn_2 = VC[sq - 2];
            dt = tn_1 - tn_2;  /* length of last span before and extension */
            /* require dt to be < mFact*(tn-tn_1) */
            if( dt > xFactor * (tn - tn_1) )
                dt = xFactor * (tn - tn_1);

            /* add two u knots to SrfCMAX  */
            vNew = tn_1 + 0.5 *dt;
            error = N_SrfInsertKnot( SrfCMAX, vNew, 1, NL_VDIR, SrfCMAX, SQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;
            vNew = tn_1 + dt;
            error = N_SrfInsertKnot( SrfCMAX, vNew, 1, NL_VDIR, SrfCMAX, SQ, SQ );

            if( error EQ NL_YES )
                NL_OUT;

            /* SrfCMAX now has two more u spans at the end of the srf */
            /* reset PwC[mC-2] and PwC[mC-1] by using PwG1[mG-2] and PwG1[mG-1] of SrfG1 */
            /* need to get the defining data again since knots have been added */
            N_SrfGetCPtsDegreesAndKnots( SrfCMAX, &nC, &mC, &PwC, &pC, &qC, &rC, &sC, &UC, &VC );

            for ( ii = 0; ii <= nC; ii++ )
            {
                N_CopyCPt( PwG[ii][mG - 2], &PwC[ii][mC - 2] );
                N_CopyCPt( PwG[ii][mG - 1], &PwC[ii][mC - 1] );
            }
        }
    }

    EXIT:
    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfLargeExtendToCrv */

#endif // NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This  routine  determines  the  type of a  given surface.  Planes, 
     spheres, toruses,  cylinders, cones, surfaces of revolution, ruled
     surfaces, extruded surfaces and free-form surfaces are recognized. 
     Patches from spheres, tori, cylinders and cones  are  also  recog-
     nized. A typical calling example:

       NL_SURFACE  sur;
       NL_CURVE    *curA, *curB;
       NL_REAL     r1, r2, al, ac, h;
       NL_POINT    AA, BB, CC, DD;
       NL_VECTOR   XX, YY;
       NL_FLAG     stp, dir;
       NL_STACKS   SG;
       ...
       (define sur and get tol);
       ...
       N_SrfType(&sur,tol,&curA,&curB,&AA,&BB,&CC,&DD,&XX,&YY,&r1,&r2,
                &ac,&al,&h,&stp,&dir,&SG);

     IT IS ASSUMED THAT sur IS A VALID NON-DEGENERATE NL_SURFACE, I.E. NOT
     A NL_POINT OR A LINE. MEMORY FOR  curA  AND  curB IS ALLOCATED INSIDE 
     THE ROUTINE.


   ACCESS:
   
     sur   , input  ,  NURBS surface
     tol   , input  ,  Tolerance to check distances of points to lines,
                       differences of radii of circles and distances of
                       Euclidean points
     curA  , output ,  Generator curve;
                         revolution: profile curve
                         ruling    : boundary curve at umin or vmin
                         extrusion : generator curve
                         others    : NULL
     curB  , output ,  Generator curve;
                         revolution: NULL
                         ruling    : boundary curve at umax or vmax
                         extrusion : NULL
                         others    : NULL
     AA    , output ,  Point/vector;
                         plane     : lower left hand corner
                         revolution: point on axis of revolution
                         extrusion : direction of extrusion
                         others    : NL_ZERO
                       For cones and cylinders  AA is the center of the 
                       bottom circle
     BB    , output ,  Point/vector;
                         plane     : lower right hand corner
                         revolution: direction of axis of revolution
                         sphere,torus,: direction of axis of revolution
                         cone, cylinder
                         others    : NL_ZERO
     CC    , output ,  Point/vector;
                         plane     : upper right hand corner
                         revolution: unit normal to plane of profile if
                                     planar, NL_ZERO otherwise
                         others    : NL_ZERO
     DD    , output ,  Point/vector;
                         plane     : upper left hand corner
                         sphere    : center of sphere/generating circle
                         torus     : center of generating circle  (also
                                     known as the "small circle")
                         others    : NL_ZERO
     XX,YY , output ,  Point/vector;
                         sphere    : local  coordinate  system  in  the 
                                     plane of the generating circle
                         torus     : local  coordinate  system  in  the
                                     plane of the small circle
                         cone/cyl  : local  coordinate  system  of base
                                     circle
                         others    : NL_ZERO
                       XX points toward the start point of the generat-
                       ing circle (for sphere, torus, cone & cylinder)
     r1,r2 , output ,  Values;
                         sphere    : radius (r1)
                         torus     : radius of circle of  rotation (r1)
                                     (small circle),  distance  of  the 
                                     center of the  small circle to the 
                                     axis of revolution (r2)
                         cylinder  : bottom/top radius (r1)
                         cone      : bottom (r1) top (r2) radii
                         extrusion : distance of extrusion (r1)
                         others    : 0.0
     ac    , output ,  Value;
                         sphere    : sweep angle  of  generator  circle
                                     measured  in  the  system  <XX,YY> 
                                     from XX toward YY
                         torus     : same as in the sphere
                         others    : 0.0
     al    , output ,  Value;
                         revolution: angle of rotation
                         sphere    : same as in revolution
                         torus     : same as in revolution
                         others    : 0.0;
     h     , output ,  Value;
                         cone/cyl  : height of cone/cylinder 
                         others    : 0.0;
     stp   , output ,  Surface type;
                         NL_NPLANE     : plane (bilinear surface)
                         NL_NSPHERE    : sphere or spherical patch
                         NL_NTORUS     : torus or toroidal patch
                         NL_NCYLINDER  : right circular cylinder/patch
                         NL_NCONE      : right circular cone/patch
                         NL_NREVOLUTION: surface of revolution 
                         NL_NRULED     : ruled surface
                         NL_NEXTRUSION : extruded surface
                         NL_NFREEFORM   : free-form surface
     dir   , output ,  Direction of construction: NL_UDIR or NL_VDIR;
                         plane     : NL_NONE
                         cone/cyl  : surface is linear in dir
                         revolution: direction of revolution
                         sphe/torus: as in revolution
                         ruled     : direction of ruling
                         extrusion : direction of extrusion
                         free-form : NL_NONE
     SG    , input  ,  Memory stack of curA and curB


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfType( NL_SURFACE *sur, NL_REAL tol, NL_CURVE ** curA, NL_CURVE ** curB, NL_POINT *AA, NL_POINT *BB, NL_POINT *CC, NL_POINT *DD, NL_VECTOR *XX, NL_VECTOR *YY, NL_REAL *r1, NL_REAL *r2, NL_REAL *ac, NL_REAL *al, NL_REAL *h, NL_FLAG *stp, NL_FLAG *dir, NL_STACKS *SG )
{

    NL_FLAG flt, ctp, prj, error = NL_NO;

    NL_DEGREE p, q, pq;

    NL_INDEX i, j, k, kl, kh, n, m, r, s;

    NL_REAL *U, *V, *a, aa, u, v, ul, ur, ui, vl, vr, vi, f, f1, f2, f3, f4, max, d;

    NL_POINT *P, Pa, P1, P2, P3, P4;

    NL_VECTOR *N, Na, N1, N2, N3, N4, VV, V1, V2, V3, V4, A;

    NL_CPOINT ** Pw;

    NL_CURVE *curP, *curS, curT;

    NL_LINESEG lsg;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Initialize and get locals */

    *curA = NULL;
    *curB = NULL;
    *r1 = 0.0;
    *r2 = 0.0;
    *ac = 0.0;
    *al = 0.0;
    *h = 0.0;
    *dir = NL_NONE;
    *stp = NL_NFREEFORM;

    N_VectorCopy( NL_ZERO, AA );
    N_VectorCopy( NL_ZERO, BB );
    N_VectorCopy( NL_ZERO, CC );
    N_VectorCopy( NL_ZERO, DD );
    N_VectorCopy( NL_ZERO, XX );
    N_VectorCopy( NL_ZERO, YY );

    N_SrfGetCPts( sur, &n, &m, &Pw );
    N_SrfGetDegrees( sur, &p, &q );
    N_SrfGetKnots( sur, &r, &s, &U, &V );

    /*************************/
    /* Check if sur is plane */
    /*************************/

    /* Fit plane to the four corners */

    N_CPtToPtEuclid( Pw[0][0], &P1 );
    N_CPtToPtEuclid( Pw[n][0], &P2 );
    N_CPtToPtEuclid( Pw[n][m], &P3 );
    N_CPtToPtEuclid( Pw[0][m], &P4 );

    N_VectorDiff( P2, P1, &V1 );
    N_VectorDiff( P3, P2, &V2 );
    N_VectorDiff( P4, P3, &V3 );
    N_VectorDiff( P1, P4, &V4 );

    N_VectorCross( V4, V1, &N1 );
    N_VectorMagnitude( N1, &f1 );
    N_VectorCross( V1, V2, &N2 );
    N_VectorMagnitude( N2, &f2 );
    N_VectorCross( V2, V3, &N3 );
    N_VectorMagnitude( N3, &f3 );
    N_VectorCross( V3, V4, &N4 );
    N_VectorMagnitude( N4, &f4 );

    max = f1;
    A = N1;

    if( f2 GT max )
    {
        max = f2;
        A = N2;
    }

    if( f3 GT max )
    {
        max = f3;
        A = N3;
    }

    if( f4 GT max )
    {
        max = f4;
        A = N4;
    }

    if( max LT NL_MTOL )
        goto revolution;

    N_VectorDot( A, N1, &f1 );

    if( f1 LT 0.0 )
        N_VectorReverse( N1, &N1 );
    N_VectorDot( A, N2, &f2 );

    if( f2 LT 0.0 )
        N_VectorReverse( N2, &N2 );
    N_VectorDot( A, N3, &f3 );

    if( f3 LT 0.0 )
        N_VectorReverse( N3, &N3 );
    N_VectorDot( A, N4, &f4 );

    if( f4 LT 0.0 )
        N_VectorReverse( N4, &N4 );

    N_Combine4Pts( 0.25, P1, 0.25, P2, 0.25, P3, 0.25, P4, &Pa );
    N_Combine4Pts( 0.25, N1, 0.25, N2, 0.25, N3, 0.25, N4, &Na );

    error = N_VectorNormalizeRef( &Na );

    if( error EQ NL_YES )
        NL_OUT;

    N_VectorDot( Pa, Na, &f4 );
    N_PtToXYZ( Na, &f1, &f2, &f3 );
    f4 = -f4;

    /* Check flatness */

    for ( i = 0; i <= n; i++ )
    {
        for ( j = 0; j <= m; j++ )
        {
            N_CPtToPtEuclid( Pw[i][j], &Pa );
            N_DistSignedPtPlane( f1, f2, f3, f4, Pa, &d );

            if( fabs( d )GT tol )
                goto revolution;
        }
    }

    /* Check straightness of boundaries */

    for ( i = 1; i <= n - 1; i++ )
    {
        N_CPtToPtEuclid( Pw[i][0], &Pa );

        error = N_DistPtLineSeg( Pa, P1, P2, &d );

        if( error EQ NL_YES )
            NL_OUT;

        if( d GT tol )
            goto revolution;

        N_CPtToPtEuclid( Pw[i][m], &Pa );

        error = N_DistPtLineSeg( Pa, P4, P3, &d );

        if( error EQ NL_YES )
            NL_OUT;

        if( d GT tol )
            goto revolution;
    }

    for ( j = 1; j <= m - 1; j++ )
    {
        N_CPtToPtEuclid( Pw[0][j], &Pa );

        error = N_DistPtLineSeg( Pa, P1, P4, &d );

        if( error EQ NL_YES )
            NL_OUT;

        if( d GT tol )
            goto revolution;

        N_CPtToPtEuclid( Pw[n][j], &Pa );

        error = N_DistPtLineSeg( Pa, P2, P3, &d );

        if( error EQ NL_YES )
            NL_OUT;

        if( d GT tol )
            goto revolution;
    }

    /* Output defining entities */

    N_VectorCopy( P1, AA );
    N_VectorCopy( P2, BB );
    N_VectorCopy( P3, CC );
    N_VectorCopy( P4, DD );

    *stp = NL_NPLANE;

    NL_OUT;

    /*****************************************/
    /* Check if sur is surface of revolution */
    /*****************************************/

    revolution:

    i = NL_MAX( n, m );
    pq = NL_MAX( p, q );
    k = NL_MAX( r, s );
    kl = 2 * i * (pq + 1);

    curP = N_AllocCrvAndArrays( i, pq, k, SG );

    if( curP EQ NULL )
        NL_QUIT;

    curS = N_AllocCrvAndArrays( i, pq, k, SG );

    if( curS EQ NULL )
        NL_QUIT;

    P = N_AllocPt1dArray( kl, &SL );

    if( P EQ NULL )
        NL_QUIT;

    N = N_AllocPt1dArray( kl, &SL );

    if( N EQ NULL )
        NL_QUIT;

    a = N_AllocReal1dArray( kl, &SL );

    if( a EQ NULL )
        NL_QUIT;

    /* See if revolution in the u-direction */

    N_CrvSetSizeIndices( curS, n, p, r );
    N_VectorCopy( NL_ZERO, &Pa );
    N_VectorCopy( NL_ZERO, &Na );

    vl = V[q];
    j = q + 1;
    i = 0;
    aa = 0.0;
    kh = 2 * q;

    while( j LT s )
    {
        if( j EQ q + 1 )
            kl = 0;
        else
            kl = 1;

        while( j LT s AND V[j]EQ V[j + 1] )
            j++;
        vr = V[j];
        vi = (vr - vl) / kh;

        for ( k = kl; k <= kh; k++ )
        {
            if( k EQ kh )
                v = vr;
            else
                v = vl + k * vi;

            error = N_SrfExtractIsoCrv( sur, v, NL_UDIR, curS, SG );

            if( error EQ NL_YES )
                NL_OUT;

            if( N_CrvIsDegen( curS ) )
                continue;

            error = N_CrvGetType( curS, tol, &P[i], &N[i], &V1, &V2, &f1, &a[i], &ctp );

            if( error EQ NL_YES )
                goto tryvrev;

            if( ctp NEQ NL_NCIRCLE )
                goto tryvrev;

            N_VectorSum( Pa, P[i], &Pa );
            N_VectorSum( Na, N[i], &Na );

            aa += a[i];
            i++;
        }

        vl = vr;
        j++;
    }

    f = 1.0 / i;
    aa = f * aa;
    N_VectorScale( Pa, f, &Pa );
    N_VectorScale( Na, f, &Na );
    N_VectorMagnitude( Na, &d );

    if( d LT tol )
        goto tryvrev;

    N_CreateLineStartDirVector( &lsg, Pa, Na, NL_UNBOUNDED );

    for ( k = 0; k <= i - 1; k++ )
    {
        error = N_DistPtInfLine( lsg, P[k], &d );

        if( error EQ NL_YES )
            NL_OUT;

        if( d GT tol )
            goto tryvrev;

        N_VectorDiff( N[k], Na, &A );
        N_VectorMagnitude( A, &d );

        if( d GT tol )
            goto tryvrev;

        if( fabs( aa - a[k] )GT tol )
            goto tryvrev;
    }

    /* Output revolution defining data */

    N_CrvSetSizeIndices( curP, m, q, s );

    error = N_SrfExtractIsoCrv( sur, U[0], NL_VDIR, curP, SG );

    if( error EQ NL_YES )
        NL_OUT;

    *curA = curP;
    *al = aa;

    N_VectorCopy( Pa, AA );
    N_VectorCopy( Na, BB );

    /* Determine sub-type */

    error = N_CrvGetType( curP, tol, &P3, &N3, &V1, &V2, &f1, &f2, &ctp );

    if( error EQ NL_YES )
        goto tryvrev;

    if( ctp EQ NL_NLINE )
    {
        N_VectorSum( P3, N3, &P4 );

        error = N_ProjectPtLine( lsg, P3, &P1, &f3, &prj );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_ProjectPtLine( lsg, P4, &P2, &f3, &prj );

        if( error EQ NL_YES )
            NL_OUT;

        N_DistPtPt( P1, P3, &f1 );
        N_DistPtPt( P2, P4, &f2 );

        if( fabs( f1 - f2 )LT tol )
            *stp = NL_NCYLINDER;
        else
            *stp = NL_NCONE;

        if( f1 GT f2 )
            N_VectorDiff( P3, P1, &V1 );
        else
            N_VectorDiff( P4, P2, &V1 );

        N_DistPtPt( P1, P2, h );
        N_VectorCross( Na, V1, CC );
        N_VectorCopy( *CC, YY );
        N_VectorCopy( P1, AA );
        N_VectorCopy( V1, XX );

        error = N_VectorNormalizeRef( CC );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_VectorNormalizeRef( XX );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_VectorNormalizeRef( YY );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvLineFromPtAndVector( P3, N3, curP, SG );

        if( error EQ NL_YES )
            NL_OUT;

        *r1 = f1;
        *r2 = f2;
        *dir = NL_VDIR;
    }
    else if( ctp EQ NL_NCIRCLE )
    {
        N_VectorCopy( P3, DD );
        N_VectorCopy( N3, CC );
        N_VectorCopy( V1, XX );
        N_VectorCopy( V2, YY );

        error = N_DistPtInfLine( lsg, P3, &d );

        if( error EQ NL_YES )
            NL_OUT;

        if( d LT tol )
            *stp = NL_NSPHERE;
        else
            *stp = NL_NTORUS;

        *r1 = f1;
        *r2 = d;
        *ac = f2;
        *dir = NL_UDIR;
    }
    else
    {
        error = N_CrvIsPlanar( curP, tol, &Pa, &Na, &flt, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        if( flt EQ NL_TRUE )
            N_VectorCopy( Na, CC );

        *dir = NL_UDIR;
        *stp = NL_NREVOLUTION;
    }

    NL_OUT;

    /* Try now v-directional revolution */

    tryvrev:

    N_CrvSetSizeIndices( curS, m, q, s );
    N_VectorCopy( NL_ZERO, &Pa );
    N_VectorCopy( NL_ZERO, &Na );

    ul = U[p];
    i = p + 1;
    j = 0;
    aa = 0.0;
    kh = 2 * p;

    while( i LT r )
    {
        if( i EQ p + 1 )
            kl = 0;
        else
            kl = 1;

        while( i LT r AND U[i]EQ U[i + 1] )
            i++;
        ur = U[i];
        ui = (ur - ul) / kh;

        for ( k = kl; k <= kh; k++ )
        {
            if( k EQ kh )
                u = ur;
            else
                u = ul + k * ui;

            error = N_SrfExtractIsoCrv( sur, u, NL_VDIR, curS, SG );

            if( error EQ NL_YES )
                NL_OUT;

            if( N_CrvIsDegen( curS ) )
                continue;

            error = N_CrvGetType( curS, tol, &P[j], &N[j], &V1, &V2, &f3, &a[j], &ctp );

            if( error EQ NL_YES )
                goto ruled;

            if( ctp NEQ NL_NCIRCLE )
                goto ruled;

            N_VectorSum( Pa, P[j], &Pa );
            N_VectorSum( Na, N[j], &Na );

            aa += a[j];
            j++;
        }

        ul = ur;
        i++;
    }

    f = 1.0 / j;
    aa = f * aa;
    N_VectorScale( Pa, f, &Pa );
    N_VectorScale( Na, f, &Na );
    N_VectorMagnitude( Na, &d );

    if( d LT tol )
        goto ruled;

    N_CreateLineStartDirVector( &lsg, Pa, Na, NL_UNBOUNDED );

    for ( k = 0; k <= j - 1; k++ )
    {
        error = N_DistPtInfLine( lsg, P[k], &d );

        if( error EQ NL_YES )
            NL_OUT;

        if( d GT tol )
            goto ruled;

        N_VectorDiff( N[k], Na, &A );
        N_VectorMagnitude( A, &d );

        if( d GT tol )
            goto ruled;

        if( fabs( aa - a[k] )GT tol )
            goto ruled;
    }

    /* Output revolution defining data */

    N_CrvSetSizeIndices( curP, n, p, r );

    error = N_SrfExtractIsoCrv( sur, V[0], NL_UDIR, curP, SG );

    if( error EQ NL_YES )
        NL_OUT;

    *curA = curP;
    *al = aa;

    N_VectorCopy( Pa, AA );
    N_VectorCopy( Na, BB );

    /* Determine sub-type */

    error = N_CrvGetType( curP, tol, &P3, &N3, &V1, &V2, &f1, &f2, &ctp );

    if( error EQ NL_YES )
        goto ruled;

    if( ctp EQ NL_NLINE )
    {
        N_VectorSum( P3, N3, &P4 );

        error = N_ProjectPtLine( lsg, P3, &P1, &f3, &prj );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_ProjectPtLine( lsg, P4, &P2, &f3, &prj );

        if( error EQ NL_YES )
            NL_OUT;

        N_DistPtPt( P1, P3, &f1 );
        N_DistPtPt( P2, P4, &f2 );

        if( fabs( f1 - f2 )LT tol )
            *stp = NL_NCYLINDER;
        else
            *stp = NL_NCONE;

        if( f1 GT f2 )
            N_VectorDiff( P3, P1, &V1 );
        else
            N_VectorDiff( P4, P2, &V1 );

        N_DistPtPt( P1, P2, h );
        N_VectorCross( Na, V1, CC );
        N_VectorCopy( *CC, YY );
        N_VectorCopy( P1, AA );
        N_VectorCopy( V1, XX );

        error = N_VectorNormalizeRef( CC );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_VectorNormalizeRef( XX );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_VectorNormalizeRef( YY );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvLineFromPtAndVector( P3, N3, curP, SG );

        if( error EQ NL_YES )
            NL_OUT;

        *r1 = f1;
        *r2 = f2;
        *dir = NL_UDIR;
    }
    else if( ctp EQ NL_NCIRCLE )
    {
        N_VectorCopy( P3, DD );
        N_VectorCopy( N3, CC );
        N_VectorCopy( V1, XX );
        N_VectorCopy( V2, YY );

        error = N_DistPtInfLine( lsg, P3, &d );

        if( error EQ NL_YES )
            NL_OUT;

        if( d LT tol )
            *stp = NL_NSPHERE;
        else
            *stp = NL_NTORUS;

        *r1 = f1;
        *r2 = d;
        *ac = f2;
        *dir = NL_VDIR;
    }
    else
    {
        error = N_CrvIsPlanar( curP, tol, &Pa, &Na, &flt, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        if( flt EQ NL_TRUE )
            N_VectorCopy( Na, CC );

        *stp = NL_NREVOLUTION;
        *dir = NL_VDIR;
    }

    NL_OUT;

    /***********************************/
    /* Now see if sur is ruled surface */
    /***********************************/

    ruled:

    /* Check if ruled in the u-direction */

    N_CrvSetSizeIndices( curP, n, p, r );

    vl = V[q];
    j = q + 1;
    i = 0;
    kh = 2 * q;
    N_VectorCopy( NL_ZERO, &VV );

    while( j LT s )
    {
        if( j EQ q + 1 )
            kl = 0;
        else
            kl = 1;

        while( j LT s AND V[j]EQ V[j + 1] )
            j++;
        vr = V[j];
        vi = (vr - vl) / kh;

        for ( k = kl; k <= kh; k++ )
        {
            if( k EQ kh )
                v = vr;
            else
                v = vl + k * vi;

            error = N_SrfExtractIsoCrv( sur, v, NL_UDIR, curP, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvGetType( curP, tol, &Pa, &Na, &V1, &V2, &f1, &f2, &ctp );

            if( error EQ NL_YES )
                goto tryvrul;

            if( ctp NEQ NL_NLINE )
                goto tryvrul;

            N_VectorSum( VV, Na, &VV );
            i++;
        }

        vl = vr;
        j++;
    }

    /* Output ruled surface defining data */

    N_CrvSetSizeIndices( curP, m, q, s );
    N_CrvSetSizeIndices( curS, m, q, s );

    error = N_SrfExtractIsoCrv( sur, U[0], NL_VDIR, curP, SG );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SrfExtractIsoCrv( sur, U[r], NL_VDIR, curS, SG );

    if( error EQ NL_YES )
        NL_OUT;

    *dir = NL_UDIR;
    *curA = curP;

    f = 1.0 / i;
    N_VectorScale( VV, f, &VV );

    N_CrvInitArrays( &curT );
    error = N_CrvCopy( curP, &curT, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvTranslate( &curT, VV );

    if( N_CrvsAreCoincident( &curT, curS, tol, &SL ) )
    {
        N_VectorCopy( VV, AA );
        N_VectorMagnitude( VV, r1 );

        *stp = NL_NEXTRUSION;
    }
    else
    {
        *stp = NL_NRULED;
        *curB = curS;
    }

    NL_OUT;

    /* Check if ruled in the v-direction */

    tryvrul:

    N_CrvSetSizeIndices( curP, m, q, s );

    ul = U[p];
    i = p + 1;
    j = 0;
    kh = 2 * p;
    N_VectorCopy( NL_ZERO, &VV );

    while( i LT r )
    {
        if( i EQ p + 1 )
            kl = 0;
        else
            kl = 1;

        while( i LT r AND U[i]EQ U[i + 1] )
            i++;
        ur = U[i];
        ui = (ur - ul) / kh;

        for ( k = kl; k <= kh; k++ )
        {
            if( k EQ kh )
                u = ur;
            else
                u = ul + k * ui;

            error = N_SrfExtractIsoCrv( sur, u, NL_VDIR, curP, &SL );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_CrvGetType( curP, tol, &Pa, &Na, &V1, &V2, &f1, &f2, &ctp );

            if( error EQ NL_YES )
                NL_OUT;

            if( ctp NEQ NL_NLINE )
                NL_OUT;

            N_VectorSum( VV, Na, &VV );
            j++;
        }

        ul = ur;
        i++;
    }

    /* Output ruled surface defining data */

    N_CrvSetSizeIndices( curP, n, p, r );
    N_CrvSetSizeIndices( curS, n, p, r );

    error = N_SrfExtractIsoCrv( sur, V[0], NL_UDIR, curP, SG );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_SrfExtractIsoCrv( sur, V[s], NL_UDIR, curS, SG );

    if( error EQ NL_YES )
        NL_OUT;

    *dir = NL_VDIR;
    *curA = curP;

    f = 1.0 / j;
    N_VectorScale( VV, f, &VV );

    N_CrvInitArrays( &curT );
    error = N_CrvCopy( curP, &curT, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvTranslate( &curT, VV );

    if( N_CrvsAreCoincident( &curT, curS, tol, &SL ) )
    {
        N_VectorCopy( VV, AA );
        N_VectorMagnitude( VV, r1 );

        *stp = NL_NEXTRUSION;
    }
    else
    {
        *stp = NL_NRULED;
        *curB = curS;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfType */


/*******************************************************************//**


   DESCRIPTION:

     This surface routine scales a surface's  knot  vectors  to  a  given
     rectangle,  [us,ue]x[vs,ve].  This does not change the surface  geo-
     metry. The operation is done in place. A typical calling example is:

       NL_SURFACE    sur;
       NL_PARAMETER  us, ue, vs, ve;
       ...
       (define sur, and choose us,ue,vs,ve);
       ...

       N_SrfReparam(&sur,us,ue,vs,ve);


   ACCESS:
   
     sur  , in/out ,  NURBS surface whose knot vectors are to be rescaled
     us   , input  ,  Start u parameter of surface after scaling
     ue   , input  ,  End u parameter of surface after scaling
     vs   , input  ,  Start v parameter of surface after scaling
     ve   , input  ,  End v parameter of surface after scaling


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_SrfReparam( NL_SURFACE *sur, NL_PARAMETER us, NL_PARAMETER ue, NL_PARAMETER vs, NL_PARAMETER ve )
{

    NL_RECTANGLE R;

    NL_REAL u1, u2, v1, v2;

    NL_FLAG scaledirs;

    /* Make new parameter rectangle */

    N_CreateRectangle( &R, us, ue, vs, ve );

    /* Determine which directions need scaling and call the utility */

    N_SrfGetParameterBounds( sur, &u1, &u2, &v1, &v2 );

    scaledirs = (((us NEQ u1 OR ue NEQ u2) ? NL_UDIR : 0) + ((vs NEQ v1 OR ve NEQ v2) ? NL_VDIR : 0));

    N_SrfReparamToInterval( sur, R, scaledirs );
} /* end N_SrfReparam */


/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This surface routine finds all degenerate patch strips of a surface.
     The strip [u1,u2] is defined to  be  degenerate if its image surface
     strip between the isocurves S(u1,v) and S(u2,v) has width less  than
     some given tolerance. A similar statement defines a degenerate strip
     between [v1,v2]. A typical calling example is:
 
       NL_SURFACE  sur;
       NL_REAL     tol;
       NL_REAL     **ustps, **vstps;
       NL_INDEX    nu, nv;
       NL_STACKS   SG;
       ...
       (define sur and choose tol);
       ...
       N_SrfFindDegenPatch(&sur,tol,NL_UVDIR,&ustps,&nu,&vstps,&nv,&SG);
 
 
   ACCESS:
   
     sur   , input  ,  NURBS surface
     tol   , input  ,  Tolerance.  A strip is degenerate  if its width is
                       less than tol
     dflg  , input  ,  Flag:
                        NL_UDIR : only look for degenerate u-strips
                        NL_VDIR : only look for degenerate v-strips
                        NL_UVDIR: look for degenerate u- and v-strips
     ustps , output ,  The parameter values  defining  the  degenerate u-
                       strips. ustps[i][0] and ustps[i][1]  are the start
                       and end parameters of the i-th degenerate u-strip,
                       respectively. This array is allocated in this rou-
                       tine (only if nu >= 0)
     nu    , output ,  High index of u-strips (nu+1 u-strips).  If  there
                       are no degenerate u-strips, nu = -1  and  ustps is
                       not allocated
     vstps , output ,  The parameter values  defining  the  degenerate v-
                       strips. vstps[i][0] and vstps[i][1]  are the start
                       and end parameters of the i-th degenerate v-strip,
                       respectively. This array is allocated in this rou-
                       tine (only if nv >= 0)
     nv    , output ,  High index of v-strips (nv+1 v-strips).  If  there
                       are no degenerate v-strips, nv = -1  and  vstps is
                       not allocated
     SG    , output ,  Memory stack for ustps and vstps
 
 
   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

   // ** NOT REQUIRED BY SMLIB. ONLY USED BY HW **

NL_FLAG N_SrfFindDegenPatch( NL_SURFACE *sur, NL_REAL tol, NL_FLAG dflg, NL_REAL *** ustps, NL_INDEX *nu, NL_REAL *** vstps, NL_INDEX *nv, NL_STACKS *SG )
{

    NL_FLAG error = NL_NO;

    NL_INDEX nuu, nvv, n, m, r, s, ii, jj, k1, k2, nn, nsamp = 1;

    NL_DEGREE p, q, pq;

    NL_REAL u, v, ** ustp, ** vstp, ** stp, *U, *V, samp[1];

    NL_CURVE cur;

    NL_CPOINT ** Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    samp[0] = 0.4522; /* sample between knots */

    /* Get local notation */

    N_SrfGetCPtsDegreesAndKnots( sur, &n, &m, &Pw, &p, &q, &r, &s, &U, &V );

    /* Allocate memory for isocurves */

    pq = NL_MAX( p, q );
    ii = NL_MAX( n, m );

    error = N_AllocCrvArrays( &cur, ii, pq, ii + pq + 1, &SL );

    if( error EQ NL_YES )
        NL_OUT;

    /* We find degenerate strips by looking for degenerate segments  */
    /* in the isocurves. We take isocurves at the knots, and also at */
    /* nsamp locations between the knots.                            */

    *nu = -1;
    *nv = -1;
    ustp = vstp = NULL;

    /* First look for degenerate u-strips */

    if( dflg EQ NL_UDIR OR dflg EQ NL_UVDIR )
    {
        error = N_SrfExtractIsoCrv( sur, V[0], NL_UDIR, &cur, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvGetDegenSegs( &cur, tol, &ustp, &nuu, SG );

        if( error EQ NL_YES )
            NL_OUT;

        while( nuu >= 0 )
        {
            ii = q;    /* current knot span */

            while( 1 ) /* loop through v-knot vector */
            {
                for ( jj = 0; jj <= nsamp; jj++ )
                {
                    if( jj EQ nsamp )
                        v = V[ii + 1];
                    else
                        v = (1.0 - samp[jj]) * V[ii] + samp[jj] * V[ii + 1];

                    error = N_SrfExtractIsoCrv( sur, v, NL_UDIR, &cur, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    error = N_CrvGetDegenSegs( &cur, tol, &stp, &nn, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    if( nn LT 0 )
                    {
                        nuu = -1;
                        break;
                    }

                    for ( k1 = 0; k1 <= nuu; k1++ )
                    {
                        for ( k2 = 0; k2 <= nn; k2++ )
                            if( stp[k2][0]EQ ustp[k1][0]AND stp[k2][1]EQ ustp[k1][1] )
                                break;

                        if( k2 GT nn ) /* ustp[k1] is not degenerate - delete it */
                        {
                            for ( k2 = k1 + 1; k2 <= nuu; k2++ )
                            {
                                ustp[k2 - 1][0] = ustp[k2][0];
                                ustp[k2 - 1][1] = ustp[k2][1];
                            }

                            nuu -= 1;
                            k1 -= 1;
                        }
                    }

                    N_FreeReal2dArray( stp, &SL );

                    if( nuu LT 0 )
                        break;
                }

                if( nuu LT 0 OR ii GE m )
                    break;

                ii += 1; /* go to next span */

                while( ii < s AND V[ii]EQ V[ii + 1] )
                    ii += 1;

                if( ii GE s )
                    break;
            }

            break;
        }

        *nu = nuu;
        *ustps = ustp;
    }

    /* Now look for degenerate v-strips */

    if( dflg EQ NL_VDIR OR dflg EQ NL_UVDIR )
    {
        N_CrvSetSizeIndices( &cur, m, q, s );

        error = N_SrfExtractIsoCrv( sur, U[0], NL_VDIR, &cur, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_CrvGetDegenSegs( &cur, tol, &vstp, &nvv, SG );

        if( error EQ NL_YES )
            NL_OUT;

        while( nvv >= 0 )
        {
            ii = p;    /* current knot span */

            while( 1 ) /* loop through u-knot vector */
            {
                for ( jj = 0; jj <= nsamp; jj++ )
                {
                    if( jj EQ nsamp )
                        u = U[ii + 1];
                    else
                        u = (1.0 - samp[jj]) * U[ii] + samp[jj] * U[ii + 1];

                    error = N_SrfExtractIsoCrv( sur, u, NL_VDIR, &cur, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    error = N_CrvGetDegenSegs( &cur, tol, &stp, &nn, &SL );

                    if( error EQ NL_YES )
                        NL_OUT;

                    if( nn LT 0 )
                    {
                        nvv = -1;
                        break;
                    }

                    for ( k1 = 0; k1 <= nvv; k1++ )
                    {
                        for ( k2 = 0; k2 <= nn; k2++ )
                            if( stp[k2][0]EQ vstp[k1][0]AND stp[k2][1]EQ vstp[k1][1] )
                                break;

                        if( k2 GT nn ) /* vstp[k1] is not degenerate - delete it */
                        {
                            for ( k2 = k1 + 1; k2 <= nvv; k2++ )
                            {
                                vstp[k2 - 1][0] = vstp[k2][0];
                                vstp[k2 - 1][1] = vstp[k2][1];
                            }

                            nvv -= 1;
                            k1 -= 1;
                        }
                    }

                    N_FreeReal2dArray( stp, &SL );

                    if( nvv LT 0 )
                        break;
                }

                if( nvv LT 0 OR ii GE n )
                    break;

                ii += 1; /* go to next span */

                while( ii < r AND U[ii]EQ U[ii + 1] )
                    ii += 1;

                if( ii GE r )
                    break;
            }

            break;
        }

        *nv = nvv;
        *vstps = vstp;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfFindDegenPatch */

/*******************************************************************//**


   DESCRIPTION:

     This routine determines if a surface is Gn smoothly closed to with-
     in specified tolerances. By definition, Gn continuity requires Gn-1
     continuity, and hence, G0,...,Gn continuity is checked in this rou-
     tine.  Currently, this routine can determine only G0 and NL_G1 contin-
     uity. A typical calling example is:

       NL_SURFACE    sur;
       NL_FLAG       gnflg;
       NL_REAL       tols[2];
       ...
       (define sur and set tols);
       ...
       N_SrfIsClosedSmooth(&sur,1,tols,NL_UVDIR,&gnflg);


   ACCESS:
   
     sur   , input  ,  NURBS surface
     n     , input  ,  The level of G-continuity to  check  for  (n=0 or
                       n=1 must hold)
     tols  , input  ,  An array containing the tolerances:
                        [0]: distance tolerance for G0 closure
                        [1]: angular tolerance (degrees) for  NL_G1 closure
                             (required only if n>0)
     dflg  , input  ,  Flag:
                        NL_UDIR : check for closure only in u-direction
                        NL_VDIR : check for closure only in v-direction
                        NL_UVDIR: check for closure in u- and v-directions
     gnflg , output ,  Flag: 
                        NL_NO   : surface not Gn continuously closed
                        NL_UDIR : surface Gn closed in u-direction
                        NL_VDIR : surface Gn closed in v-direction
                        NL_UVDIR: surface Gn closed in u- and v-directions


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfIsClosedSmooth( NL_SURFACE *sur, NL_INDEX n, NL_REAL *tols, NL_FLAG dflg, NL_FLAG *gnflg )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfIsClosedSmooth");

    NL_FLAG error = NL_NO, gflag;

    NL_INDEX r, s, ii, jj;

    NL_POINT ** D1, ** D2;

    NL_DEGREE p, q;

    NL_REAL *U, *V, dd, du, dv, u, v;

    NL_BOOLEAN uboo, vboo;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Check for bad input data */

    if( n LT 0 OR n GT 1 )
        NL_ERROR( NL_INP_ERR );

    gflag = NL_NO;
    uboo = vboo = NL_FALSE;

    /* Handle case n = 0 */

    if( n EQ 0 )
    {
        dd = NL_MTOL;
        NL_MTOL = tols[0];

        if( dflg EQ NL_UDIR OR dflg EQ NL_UVDIR )
            uboo = N_SrfIsClosed( sur, NL_UDIR );

        if( dflg EQ NL_VDIR OR dflg EQ NL_UVDIR )
            vboo = N_SrfIsClosed( sur, NL_VDIR );

        NL_MTOL = dd;

        if( uboo AND vboo )
            gflag = NL_UVDIR;

        else if( uboo )
            gflag = NL_UDIR;

        else if( vboo )
            gflag = NL_VDIR;

        *gnflg = gflag;
        NL_OUT;
    }

    /* Get local notation */

    N_SrfGetKnots( sur, &r, &s, &U, &V );
    N_SrfGetDegrees( sur, &p, &q );

    /* allocate memory for derivatives */

    D1 = N_AllocPt2dArray( 1, 1, &SL );

    if( D1 EQ NULL )
        NL_QUIT;
    D2 = N_AllocPt2dArray( 1, 1, &SL );

    if( D2 EQ NULL )
        NL_QUIT;

    /* Loop through knot spans and check continuity */

    if( dflg EQ NL_UDIR OR dflg EQ NL_UVDIR )
    {
        ii = q; /* check u-closure */

        while( 1 )
        {
            dv = (V[ii + 1] - V[ii]) / q;

            for ( jj = 0; jj <= q; jj++ )
            {
                if( jj EQ q )
                {
                    v = V[ii + 1];
                    error = N_SrfDerivs( sur, U[0], v, NL_LEFT, NL_RIGHT, NL_TRUE, 1, 1, D1 );

                    if( error EQ NL_YES )
                        NL_OUT;
                    error = N_SrfDerivs( sur, U[r], v, NL_RIGHT, NL_RIGHT, NL_TRUE, 1, 1, D2 );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
                else
                {
                    v = V[ii] + jj * dv;
                    error = N_SrfDerivs( sur, U[0], v, NL_LEFT, NL_LEFT, NL_TRUE, 1, 1, D1 );

                    if( error EQ NL_YES )
                        NL_OUT;
                    error = N_SrfDerivs( sur, U[r], v, NL_RIGHT, NL_LEFT, NL_TRUE, 1, 1, D2 );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                /* check G0 */

                N_DistPtPt( D1[0][0], D2[0][0], &dd );

                if( dd GT tols[0] )
                    break;

                /* check NL_G1 */

                error = N_VectorsAngle( D1[1][0], D2[1][0], &dd );

                if( error EQ NL_YES )
                    break;

                else if( dd GT tols[1] )
                    break;
            }

            if( jj LE q )
                break;

            ii += 1;

            if( ii GE s - q )
            {
                uboo = NL_TRUE;
                break;
            }

            /* go to next span */

            while( ii LT s AND V[ii]EQ V[ii + 1] )
                ii += 1;

            if( ii GE s )
                NL_ERROR( NL_KNT_ERR );
        }
    }

    if( dflg EQ NL_VDIR OR dflg EQ NL_UVDIR )
    {
        ii = p; /* check v-closure */

        while( 1 )
        {
            du = (U[ii + 1] - U[ii]) / p;

            for ( jj = 0; jj <= p; jj++ )
            {
                if( jj EQ p )
                {
                    u = U[ii + 1];
                    error = N_SrfDerivs( sur, u, V[0], NL_RIGHT, NL_LEFT, NL_TRUE, 1, 1, D1 );

                    if( error EQ NL_YES )
                        NL_OUT;
                    error = N_SrfDerivs( sur, u, V[s], NL_RIGHT, NL_RIGHT, NL_TRUE, 1, 1, D2 );

                    if( error EQ NL_YES )
                        NL_OUT;
                }
                else
                {
                    u = U[ii] + jj * du;
                    error = N_SrfDerivs( sur, u, V[0], NL_LEFT, NL_LEFT, NL_TRUE, 1, 1, D1 );

                    if( error EQ NL_YES )
                        NL_OUT;
                    error = N_SrfDerivs( sur, u, V[s], NL_LEFT, NL_RIGHT, NL_TRUE, 1, 1, D2 );

                    if( error EQ NL_YES )
                        NL_OUT;
                }

                /* check G0 */

                N_DistPtPt( D1[0][0], D2[0][0], &dd );

                if( dd GT tols[0] )
                    break;

                /* check NL_G1 */

                error = N_VectorsAngle( D1[0][1], D2[0][1], &dd );

                if( error EQ NL_YES )
                    break;

                else if( dd GT tols[1] )
                    break;
            }

            if( jj LE p )
                break;

            ii += 1;

            if( ii GE r - p )
            {
                vboo = NL_TRUE;
                break;
            }

            /* go to next span */

            while( ii LT r AND U[ii]EQ U[ii + 1] )
                ii += 1;

            if( ii GE r )
                NL_ERROR( NL_KNT_ERR );
        }
    }

    /* set the output flag */

    if( uboo AND vboo )
        gflag = NL_UVDIR;

    else if( uboo )
        gflag = NL_UDIR;

    else if( vboo )
        gflag = NL_VDIR;

    *gnflg = gflag;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_SrfIsClosedSmooth */


/*******************************************************************//**


   DESCRIPTION:

       This routine computes a point, the unit partial derivatives,  and  
       the unit normal at given parameter values. The parameter location
       may be at a pole. Discontinuous surfaces are handled by  passing a  
       NL_LEFT/NL_RIGHT flag. A typical calling example is: 

       NL_SURFACE    sur;
       NL_PARAMETER  u, v;
       NL_POINT      P;
       NL_VECTOR     SU, SV, N;
       ...
       (define sur, get u and v);
       ...
       N_SrfEvalPtPtDerivNormalPole(&sur,u,v,NL_LEFT,NL_RIGHT,&P,&SU,&SV,&N);


   ACCESS:
   
     sur     , input  ,  NURBS surface
     u,v     , input  ,  Parameter values 
     ufl,vfl , input  ,  Flag:
                           NL_LEFT : t is in [t[j],t[j+1]) 
                                  (NL_RIGHT DERIVATIVES NL_USED)
                           NL_RIGHT: t is in (t[j],t[j+1]] 
                                  (NL_LEFT DERIVATIVES NL_USED)
                           (t is either u or v)
     P       , output ,  Point on the surface
     SU,SV,N , output ,  Unit partial derivatives and the normal
                         (SU or SV may be zero length)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_SrfEvalPtPtDerivNormalPole( NL_SURFACE *sur, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_POINT *P, NL_VECTOR *SU, NL_VECTOR *SV, NL_VECTOR *N )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_SrfEvalPtPtDerivNormalPole"); */

    NL_POINT ** SD;
    NL_FLAG error = NL_NO;
    NL_STACKS S;

    N_InitNurbs( &S );

    SD = N_AllocPt2dArray( 1, 1, &S );

    if( SD EQ NULL )
        NL_QUIT;

    error = N_SrfEvalPtNormalAtPole( sur, u, v, ufl, vfl, P, SU, SV, N, SD );

    /* End NURBS and Exit */
    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_SrfEvalPtPtDerivNormalPole */


/**********************************************************************/
/* N_SrfEvalPtNORMALATPOLE: Compute the surface normal at a pole                     */
/**********************************************************************/

/*******************************************************************//**


   DESCRIPTION:

     This routine computes a point, the unit partial derivatives,  and  
     the unit normal at given parameter values. This is called by N_SrfEvalPtPtDerivNormalPole:
     It can also be called directly if you supply NL_POINT array **P.

       NL_SURFACE    sur;
       NL_PARAMETER  u, v;
       NL_POINT      P;
       NL_VECTOR     SU, SV, N;
       NL_POINT      **P
       ...
       (define sur, get u and v);
       ...
       N_SrfEvalPtNormalAtPole(&sur,u,v,NL_LEFT,NL_RIGHT,&P,&SU,&SV,&N, P);


   ACCESS:
   
     sur     , input  ,  NURBS surface
     u,v     , input  ,  Parameter values 
     ufl,vfl , input  ,  Flag:
                           NL_LEFT : t is in [t[j],t[j+1]) 
                                  (NL_RIGHT DERIVATIVES NL_USED)
                           NL_RIGHT: t is in (t[j],t[j+1]] 
                                  (NL_LEFT DERIVATIVES NL_USED)
                           (t is either u or v)
     P       , output ,  Point on the surface
     SU,SV,N , output ,  Unit partial derivatives and the normal
                         (SU or SV may be zero length)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfEvalPtNormalAtPole( NL_SURFACE *sur, NL_PARAMETER u, NL_PARAMETER v, NL_FLAG ufl, NL_FLAG vfl, NL_POINT *P, NL_VECTOR *SU, NL_VECTOR *SV, NL_VECTOR *N, NL_POINT ** SD )
{

    NL_PRIVATE NL_STRING rname = _T("N_SrfEvalPtNormalAtPole");

    NL_FLAG error = NL_NO;

    NL_INDEX pole = 0;

    NL_REAL mag;

    NL_PARAMETER us, ue, vs, ve;


   /* No, don't check parameters. */
   /* Check parameters */

/* 
   NL_KNOTVECTOR *knu, *knv; 
   N_SrfGetKnotVectors( sur, &knu, &knv );
   error = N_KnotVectorIsParamOutOfBounds( knu, u, rname );
   if( error EQ NL_YES )
       NL_OUT;
   error = N_KnotVectorIsParamOutOfBounds( knv, v, rname );
   if( error EQ NL_YES )
       NL_OUT;
 */

    /* Compute surface derivatives */

    error = N_SrfDerivs( sur, u, v, ufl, vfl, NL_FALSE, 1, 1, SD );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get point and tangents */

    N_CopyPt( SD[0][0], P );

    N_VectorCopy( SD[1][0], SU );
    N_VectorCopy( SD[0][1], SV );

    /* Get unit tangents and the normal */

    N_VectorMagnitude( *SU, &mag );
    error = N_VectorNormalizeRef( SU );

    if( mag LE 0.001 *NL_MTOL OR error EQ NL_YES )
    {
        pole = 1;
        N_CopyPt( NL_ZERO, SU );
    }

    N_VectorMagnitude( *SV, &mag );
    error = N_VectorNormalizeRef( SV );

    if( mag LE 0.001 *NL_MTOL OR error EQ NL_YES )
    {
        N_CopyPt( NL_ZERO, SV );

        if( pole EQ 1 )
        {
            NL_ERROR( NL_GEO_ERR );
        }
        else
            pole = 2;
    }

    switch( pole )
    {
        case 0:

            N_VectorCross( *SU, *SV, N );
            break;

        case 1:

            N_SrfGetParameterBounds( sur, &us, &ue, &vs, &ve );

            if( v EQ ve )
                N_VectorScale( SD[0][1], -1.0, &SD[0][1] );
            N_VectorCross( SD[1][1], SD[0][1], N );
            break;

        case 2:

            N_SrfGetParameterBounds( sur, &us, &ue, &vs, &ve );

            if( u EQ ue )
                N_VectorScale( SD[1][0], -1.0, &SD[1][0] );
            N_VectorCross( SD[1][0], SD[1][1], N );
            break;
    }

    N_VectorMagnitude( *N, &mag );

    if( mag LT 0.001 *NL_MTOL )
        NL_ERROR( NL_GEO_ERR );

    error = N_VectorNormalizeRef( N );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:
    return (error);
} /* end N_SrfEvalPtNormalAtPole */


/*******************************************************************//**


   DESCRIPTION:

     This routine computes the  largest extent of a NURBS surface which 
     is the diagonal of its bounding box. A typical calling example is:

       NL_SURFACE  sur;
       NL_REAL     d;
       ...
       (define sur);
       ...
       N_SrfMaxDiagDistBBox(&sur,&d);


   ACCESS:
   
     sur , input  ,  NURBS surface
     d   , output ,  Length of the diagonal of the bounding box


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfMaxDiagDistBBox( NL_SURFACE *sur, NL_REAL *d )
{

    NL_FLAG error = NL_NO;

    NL_MINMAXBOX box;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Compute diagonal */

    error = N_SrfGetBBox( sur, &box );

    if( error EQ NL_YES )
        NL_OUT;

    N_BBoxGetDiagonal( &box, d );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_SrfMaxDiagDistBBox */

#if NLIB_UNUSED

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the derivative of a NURBS surface, at a spec-
     ific(u, v) location,  with  respect  to a control point being moved 
     along a specific vector direction. A typical calling example is:

       NL_SURFACE    sur;
       NL_INDEX      ic, jc;
       NL_PARAMETER  u, v;
       NL_VECTOR     Vd;
       NL_VECTOR     Dr;
       ...
       (define sur, and specify ic, jc, u, v, and Vd);
       ...
       N_SrfEvalPtDerivCPt(&sur, ic, jc, u, v, &Vd, &Dr);


   ACCESS:
   
     sur     , input  ,  NURBS surface to be differentiated
     ic, jc  , input  ,  Indexes of the  control  point  with respect to
                         which the derivative is being taken(0 <= ic <= n),
                         (0 <= jc <= m)
     u, v    , input  ,  Parameter values 
     Vd      , input  ,  Vector direction(must be normalized vector)
     Dr      , output ,  Derivative of surface at(u, v) wrt Vd


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_SrfEvalPtDerivCPt( NL_SURFACE *sur, NL_INDEX ic, NL_INDEX jc, NL_PARAMETER u, NL_PARAMETER v, NL_VECTOR *Vd, NL_VECTOR *Dr )
{
    NL_PRIVATE NL_STRING rname = _T("N_SrfEvalPtDerivCPt");
    NL_FLAG error = NL_NO;

    NL_INDEX n, m, k;

    NL_REAL ** w, *T, den, NU, NV, der;

    NL_DEGREE p, q;

    NL_KNOTVECTOR *knu, *knv;

    NL_SFUN sfn;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_SrfGetArraySizes( sur, &n, &m, &k, &k );
    N_SrfGetDegrees( sur, &p, &q );
    N_SrfGetKnotVectors( sur, &knu, &knv );

    /* Check parameters and indexes */

    error = N_KnotVectorIsParamOutOfBounds( knu, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knv, v, rname );

    if( error EQ NL_YES )
        NL_OUT;

    if( ic LT 0 OR ic GT n OR jc LT 0 OR jc GT m )
        NL_ERROR( NL_IND_ERR );

    /* Compute the ic - th and jc - th B - splines */

    error = N_BasisIEval( knu, ic, p, u, NL_LEFT, &NU );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisIEval( knv, jc, q, v, NL_LEFT, &NV );

    if( error EQ NL_YES )
        NL_OUT;

    der = NU * NV;

    /*  Branch on rational case  */

    if( N_IsSrfRat( sur ) )
    {
        /* Extract denominator */

        N_SFuncInitArrays( &sfn );
        error = N_SrfGetDenominatorFunc( sur, &sfn, &S );

        if( error EQ NL_YES )
            NL_OUT;

        N_SFuncGetKnots( &sfn, &w, &T, &T );

        /* Evaluate the denominator */

        error = N_SrfFuncEvalPt( &sfn, u, v, NL_LEFT, NL_LEFT, &den );

        if( error EQ NL_YES )
            NL_OUT;

        der = (der * w[ic][jc]) / den;
    }

    /* Compute the derivative vector */

    N_VectorScale( *Vd, der, Dr );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_SrfEvalPtDerivCPt */

#endif // NLIB_UNUSED

/*******************************************************************//**
 

   DESCRIPTION:
 
     This common surfaces routine creates a ruled  surface(generalized
     cone) between two elliptic ends.A typical calling example is:
 
       NL_SURFACE  sur;
       NL_STACKS   S;
       NL_REAL     radius_front, radius_rear, height;
       NL_REAL     TiltAngleY_front, TiltAngleY_rear;
       NL_REAL     TiltAngleX_front, TiltAngleX_rear;
       ...(get input values)
       ...
       N_SrfInitArrays(&sur);
       N_ConeEllipticEnds(radius_front, radius_rear, Height,
                TiltAngleY_front, TiltAngleY_rear,
                TiltAngleX_front, TiltAngleX_rear,
                &sur, 
                &S)

       The ruling is done in the v Direction 
       The cone is constructed in the XY plane with the z axis as the height,
       The end curves are ellipses, depending on the tilt
 

   ACCESS:
     radius_front      input  Front radius of the section prior to tilt
     radius_rear       input  Rear Radius of the section prior to tilt
     Height            input  Height (in z) between the end-ellipse origins
     TiltAngleY_front  input  Tilt in degrees in Y
     TiltAngleY_rear   input  Tilt in degrees in Y
     TiltAngleX_front  input  Tilt in degrees in X
     TiltAngleX_rear   input  Tilt in degrees in X
     sur              output  Ruled NURBS surface(generalized cone)
     S                 input  Stack of sur
 

   RETURN CODES:
 
     0 : No error
     1 : Error saved in NL_ERROR
 
   ***********************************************************************/

NL_FLAG N_ConeEllipticEnds( NL_REAL radius_front, NL_REAL radius_rear, NL_REAL Height, NL_REAL TiltAngleY_front, NL_REAL TiltAngleY_rear, NL_REAL TiltAngleX_front, NL_REAL TiltAngleX_rear, NL_SURFACE *sur, NL_STACKS *S )
{
    /* NL_PRIVATE NL_STRING rname = _T("N_ConeEllipticEnds"); */
    NL_FLAG error;
    NL_REAL zeroAngle = 1.e-8;
    NL_REAL myPI = 3.141592653589793238462643383;
    NL_REAL theta_Y_front, theta_X_front, theta_Y_rear, theta_X_rear;
    NL_REAL DegToRad = myPI / 180.0;
    NL_POINT x_axis, y_axis, z_axis;
    NL_POINT front_origin, rear_origin;
    NL_INDEX nu, nv, ru, rv, iv, ii;
    NL_DEGREE pu, pv;
    NL_REAL *Uknts, *Vknts;
    NL_CPOINT ** Pw;
    NL_CURVE ellipFront, ellipRear;
    NL_INDEX nf, mf, nr, mr;
    NL_DEGREE pf, pr;
    NL_CPOINT *PwFront, *PwRear;
    NL_REAL *Vfront, *Vrear;
    NL_REAL tanYfront, tanXfront, tanYrear, tanXrear;
    NL_VECTOR FrontNormal, RearNormal;
    NL_REAL denom, aF, bF, cF, deltaZfront, aR, bR, cR, deltaZrear; 
    /* NL_BOOLEAN bFrontTilt = NL_TRUE, bRearTilt = NL_TRUE;  unused */
    NL_PLANE fPln, rPln;
    NL_LINESEG lnseg;
    NL_POINT P0, P1, Q;
    NL_VECTOR dP;
    NL_REAL t, mag, w0, w1;
    NL_FLAG fPlnIntersect, rPlnIntersect;
    NL_INDEX ic;
    NL_FLAG dir = NL_VDIR; /* no choice here */
    NL_STACKS SL;

    N_InitNurbs( &SL );

    x_axis.x = 1.0;
    x_axis.y = 0.0;
    x_axis.z = 0.0;
    y_axis.x = 0.0;
    y_axis.y = 1.0;
    y_axis.z = 0.0;
    z_axis.x = 0.0;
    z_axis.y = 0.0;
    z_axis.z = 1.0;

    /* set the z - axis location of the front and rear planes */
    front_origin.x = front_origin.y = front_origin.z = 0.0;
    rear_origin.x = rear_origin.y = 0.0;
    rear_origin.z = Height;

    /* convert these to radians */
    theta_Y_front = TiltAngleY_front * DegToRad;
    theta_X_front = TiltAngleX_front * DegToRad;
    theta_Y_rear = TiltAngleY_rear * DegToRad;
    theta_X_rear = TiltAngleX_rear * DegToRad;

    /* construct a cone from the two radii and the length*/
    error = N_CreateCylCone( front_origin, x_axis, y_axis, radius_front, radius_rear, 0.0, 360.0, Height, NL_QUADRATIC, dir, sur, S );

    if( error )
        NL_QUIT;

    N_SrfGetCPtsDegreesAndKnots( sur, &nu, &nv, &Pw, &pu, &pv, &ru, &rv, &Uknts, &Vknts );

    /* extract the circle from each end*/
    N_CrvInitArrays( &ellipFront );
    error = N_SrfExtractIsoCrv( sur, 0.0, NL_VDIR, &ellipFront, &SL );

    if( error )
        NL_QUIT;

    N_CrvInitArrays( &ellipRear );
    error = N_SrfExtractIsoCrv( sur, 1.0, NL_VDIR, &ellipRear, &SL );

    if( error )
        NL_QUIT;

    /* the cpoints and knots may need to be reset to build
       the intersection curve at the front and the rear */
    N_CrvGetCPtsDegreeAndKnots( &ellipFront, &nf, &PwFront, &pf, &mf, &Vfront );

    if( nf != nv || pf != pv || mf != rv )
        NL_QUIT;

    /* use the same knots as the cone */
    for ( iv = 0; iv <= mf; iv++ )
        Vfront[iv] = Vknts[iv];

    N_CrvGetCPtsDegreeAndKnots( &ellipRear, &nr, &PwRear, &pr, &mr, &Vrear );

    if( nr != nv || pr != pv || mr != rv )
        return 1;

    /* use the same knots as the cone*/
    for ( iv = 0; iv <= mf; iv++ )
        Vrear[iv] = Vknts[iv];

    /* equation for the front plane is:
         
       Z_front - front_origin.z = X*tan(theta_X_front) + Y*tan(theta_Y_front)
       The normalized equation for a plane is  a*x + b*y + c*z + d = 0 
       with a*a + b*b + c*c = 1
       and we have X*tan(theta_X_front) + Y*tan(theta_Y_front) - Z = 0 */

    tanYfront = tan( theta_Y_front );
    tanXfront = tan( theta_X_front );

    /* since front_origin = 0.0; */

    if( fabs( TiltAngleY_front ) < zeroAngle && fabs( TiltAngleX_front ) < zeroAngle )
    { /* there is no tilt at the front */
      aF = bF = 0.0;
      cF = 1.0;         /* we want to use the plane with normal in the positive z dir*/

        /* bFrontTilt = NL_FALSE; */
        deltaZfront = 0.0;
        FrontNormal.x = z_axis.x;
        FrontNormal.y = z_axis.y;
        FrontNormal.z = z_axis.z;

        for ( ii = 0; ii <= nv; ii++ )
        {
            N_CopyCPt( Pw[0][ii], &PwFront[ii] ); /* P0 is at front, z = 0*/
        }
    }
    else
    {
        denom = sqrt( tanYfront * tanYfront + tanXfront * tanXfront + 1.0 );
        aF = -tanXfront / denom; /* for normal in positive z dir */

        bF = -tanYfront / denom;
        cF = 1.0 / denom;
        FrontNormal.x = aF;
        FrontNormal.y = bF;
        FrontNormal.z = cF;

        /* determine where the cone intersects the front plane
        so as to set dZfront, the z axis extension at the front */

        N_CreatePlanePtNormal( &fPln, front_origin, FrontNormal );

        /* need the min in z so start with z at origin of the front*/

        deltaZfront = 0.0;

        for ( ii = 0; ii < nv; ii++ )
        {
            N_CPtToPtEuclid( Pw[0][ii], &P0 ); /* P0 is at front, z = 0 */

            N_CPtToPtEuclid( Pw[1][ii], &P1 ); /* P1 is at rear,  z = length*/

            N_Diff2Pts( P0, P1, &dP );
            error = N_VectorNormalize( dP, &dP, &mag );

            if( error )
                NL_QUIT;
            N_CreateLineStartDirVector( &lnseg, P0, dP, NL_UNBOUNDED );
            N_IsectLinePlane( lnseg, fPln, &Q, &t, &fPlnIntersect );

            if( !fPlnIntersect )
                NL_QUIT;

            if( deltaZfront > (Q.z - front_origin.z) )
                deltaZfront = Q.z - front_origin.z;
            w0 = Pw[0][ii].w;
            N_CPtFromWxWyWz( w0 * Q.x, w0 * Q.y, w0 * Q.z, w0, &PwFront[ii] );
        }
        N_CopyCPt( PwFront[0], &PwFront[nv] );
    }

    /* equation for the rear plane is:     
       L = rear_origin.z
       Z_rear = L + X*tan(theta_X_rear) + Y*tan(theta_Y_rear)
       The normalized equation for a plane is  a*x + b*y + c*z + d = 0 
       with a*a + b*b + c*c = 1     
       and we have X*tan(theta_X_rear) + Y*tan(theta_Y_rear) - Z + L = 0 */

    tanYrear = tan( theta_Y_rear );
    tanXrear = tan( theta_X_rear );

    if( fabs( TiltAngleY_rear ) < zeroAngle && fabs( TiltAngleX_rear ) < zeroAngle )
    {             /* there is no tilt at the rear*/
        aR = bR = 0.0;
        cR = 1.0; /* for normal in positive z dir */

        /* dR = rear_origin.z; */
        /* bRearTilt = NL_FALSE; */
        deltaZrear = 0.0;
        RearNormal.x = z_axis.x;
        RearNormal.y = z_axis.y;
        RearNormal.z = z_axis.z;

        for ( ii = 0; ii <= nv; ii++ )
        {
            N_CopyCPt( Pw[1][ii], &PwRear[ii] ); /* P1 is at rear, z = length */
        }
    }
    else
    {
        denom = sqrt( tanYrear * tanYrear + tanXrear * tanXrear + 1.0 );
        aR = -tanXrear / denom; /* for normal in positive z dir */

        bR = -tanYrear / denom;
        cR = 1.0 / denom;
        RearNormal.x = aR;
        RearNormal.y = bR;
        RearNormal.z = cR;

        /* determine where the cone intersects the rear plane
        so as to set dZrear, the z axis extension at the rear */

        N_CreatePlanePtNormal( &rPln, rear_origin, RearNormal );

        /* need the max in z so start with z at origin of the rear */

        deltaZrear = 0.0;
        /* rear_origin.z;  no effect */

        for ( ii = 0; ii < nv; ii++ )
        {
            N_CPtToPtEuclid( Pw[0][ii], &P0 );
            N_CPtToPtEuclid( Pw[1][ii], &P1 );
            N_Diff2Pts( P1, P0, &dP );
            error = N_VectorNormalize( dP, &dP, &mag );

            if( error )
                NL_QUIT;
            N_CreateLineStartDirVector( &lnseg, P1, dP, NL_UNBOUNDED );
            N_IsectLinePlane( lnseg, rPln, &Q, &t, &rPlnIntersect );

            if( !rPlnIntersect )
                NL_QUIT;

            if( deltaZrear < (Q.z - rear_origin.z) )
                deltaZrear = Q.z - rear_origin.z;
            w1 = Pw[1][ii].w;
            N_CPtFromWxWyWz( w1 * Q.x, w1 * Q.y, w1 * Q.z, w1, &PwRear[ii] );
        }
        N_CopyCPt( PwRear[0], &PwRear[nv] );
    }

    /* reset the coefs of the cone to those of the ellipse 
    at each end. That should be equivalent to trimming */

    for ( ic = 0; ic <= nv; ic++ )
    {
        N_CopyCPt( PwFront[ic], &Pw[0][ic] );
        N_CopyCPt( PwRear[ic], &Pw[1][ic] );
    }

    EXIT:

    N_EndNurbs( &SL );

    return (error);
}
/* end N_ConeEllipticEnds */
