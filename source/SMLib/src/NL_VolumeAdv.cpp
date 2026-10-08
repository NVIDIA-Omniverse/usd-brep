// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*****************************************************************************/
/* VolumeAdv.c : Advanced Function Definitions that act on NL_VOLUME objects */
/*****************************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

#if NLIB_UNUSED


/*******************/
/*    VolumeAdv.c  */
/*******************/

/******************************/
/* File Function Declarations */
/******************************/

NL_FLAG ST_igsvcp( NL_VOLUME *vol, NL_PARAMETER ub[2], NL_PARAMETER vb[2], NL_PARAMETER wb[2], NL_STACKS *S );
NL_FLAG ST_igsvsk( NL_VOLUME *vol, NL_STACKS *S );

NL_VOID ST_VolumeReparam( NL_VOLUME *vol, NL_MINMAXBOX R, NL_FLAG dir );

/*******************************************************************//**


   DESCRIPTION:

     This error routine checks if (m+p+1) equals ir, (n+q+1) equals
     is, and (o+r+1) equals it, i.e. it  checks if  the number of  
     control points, the  degree
     and the number of knots are properly related in all  directions.
     It also checks if all  control points  have the same rationality.
     A typical callig example is:

       NL_VOLUME  vol;
       NL_STRING  rname;
       ...
       (define vol and get rname);
       ...
       N_VolumeAreCountsValid(&vol,rname);


   ACCESS:
   
     vol   , input  ,  NURBS volume
     rname , input  ,  Name of routine in which error is checked


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_VolumeAreCountsValid( NL_VOLUME *vol, NL_STRING rname )
{
    NL_FLAG oldrat, newrat, error = NL_NO;

    NL_INDEX i, j, k, m, n, o, ir, is, it;

    NL_DEGREE p, q, r;

    NL_REAL *U, *V, *W, w;

    NL_CPOINT *** Pw;

    /* Convert to local notation */
    N_VolumeGetCPtsDegreesAndKnots( vol, &m, &n, &o, &Pw, &p, &q, &r, &ir, &is, &it, &U, &V, &W );

    /* Check definition and degrees */
    if( (m + p + 1)NEQ ir OR( n + q + 1 )NEQ is OR( o + r + 1 )NEQ it )
        NL_ERROR( NL_SUR_ERR );

    if( p GT NL_DMAX OR q GT NL_DMAX OR r GT NL_DMAX )
        NL_ERROR( NL_DEG_ERR );

    /* Check for consistency */
    N_CPtGetW( Pw[0][0][0], &w );

    if( w EQ NL_NOW )
        oldrat = 0;
    else
        oldrat = 1;

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            for ( k = 0; k <= o; k++ )
            {
                N_CPtGetW( Pw[i][j][k], &w );

                if( w EQ NL_NOW )
                    newrat = 0;
                else
                    newrat = 1;

                if( newrat NEQ oldrat )
                    NL_ERROR( NL_SUR_ERR );
            }
        }
    }

    /* Exit */
    EXIT:

    return (error);
} /* end N_VolumeAreCountsValid */

/*******************************************************************//**


   DESCRIPTION:

     This error routine checks whether all the volume weights are 
     within the  range <NL_WMIN,NL_WMAX>. These weight limits are set in
     "globals.h". A typical calling example is:

       NL_VOLUME  vol;
       NL_STRING   rname;
       ...
       (define vol and get rname);
       ...
       N_VolumeAreWeightsValid(&vol,rname);


   ACCESS:
   
     vol   , input  ,  NURBS volume
     rname , input  ,  Name of routine in which error is checked


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_VolumeAreWeightsValid( NL_VOLUME *vol, NL_STRING rname )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, m, n, o;

    NL_REAL w;

    NL_CPOINT *** Pw;

    /* Convert to local notation */

    N_VolumeGetCPts( vol, &m, &n, &o, &Pw );

    /* Check volume weights */

    if( N_VolumeIsRat( vol ) )
    {
        for ( i = 0; i <= m; i++ )
        {
            for ( j = 0; j <= n; j++ )
            {
                for ( k = 0; k <= o; k++ )
                {
                    N_CPtGetW( Pw[i][j][k], &w );

                    if( w LT NL_WMIN OR w GT NL_WMAX )
                    {
                        N_ErrSet( NL_WEI_ERR, rname );
                        error = NL_YES;
                        break;
                    } /* end error check */
                }     /* end iter k */

                if( error EQ NL_YES )
                    break;
            } /* end iter j */

            if( error EQ NL_YES )
                break;
        } /* end iter i */
    }     /* end if VolumeIsRational */

    return (error);
}         /* end N_VolumeAreWeightsValid */

/*******************************************************************//**


   DESCRIPTION:

     This error routine performs a complete volume check, i.e. it 
     checks the input data for:
       (1) volume definition constants,
       (2) volume weights, and
       (3) the knot vectors.
     Other error routines are used to check  each type of error. A
     typical calling example is:
 
       NL_VOLUME  vol;
       NL_STRING   rname;
       ...
       (define vol and get rname);
       ...
       N_VolumeIsValid(&vol,rname);


   ACCESS:
   
     vol   , input  ,  NURBS volume
     rname , input  ,  Name of routine in which error is checked


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_VolumeIsValid( NL_VOLUME *vol, const TCHAR * rname )
{
    NL_FLAG error;

    NL_DEGREE p, q, r;

    NL_KNOTVECTOR *knu, *knv, *knw;

    /* Convert to local notation */

    N_VolumeGetKnotVectors( vol, &knu, &knv, &knw );
    N_VolumeGetDegrees( vol, &p, &q, &r );

    /* Check volume definition */

    error = N_VolumeAreCountsValid( vol, (NL_STRING)rname );

    if( error EQ NL_YES )
        return (1);

    /* Check volume weights */

    error = N_VolumeAreWeightsValid( vol, (NL_STRING)rname );

    if( error EQ NL_YES )
        return (1);

    /* Check knot vectors */

    error = N_KnotVectorIsValid( knu, p, (NL_STRING)rname );

    if( error EQ NL_YES )
        return (1);

    error = N_KnotVectorIsValid( knv, q, (NL_STRING)rname );

    if( error EQ NL_YES )
        return (1);

    error = N_KnotVectorIsValid( knw, r, (NL_STRING)rname );

    if( error EQ NL_YES )
        return (1);

    /* Exit */

    return (0);
} /* end N_VolumeIsValid */


/*******************************************************************//**


  DESCRIPTION:

     This error routine checks if any two consecutive control points
     within the interior of a volume are equal. Only the first indices of
     equal (within Tolerance) control points returned ( 0 = all are different).
     Use Tol = NL_MTOL for strict equality.

       NL_VOLUME   vol;
       ...
       (define vol);
       ...
       N_VolumeHasEqualCPts(&vol, Tol, &Nu, &Nv, &Nw);


   ACCESS:   
     Vol       , input  ,  NURBS volume
     Tol       , input  ,  Distance tolerance between adjacent control points
     Nu, Nv, Nw, output ,  index of first CPoint found within tolerance of another
                           Not set when when return code is 0.

   RETURN CODES:

     0 : No control points are equal (within Tol)
     1 : Control Points Pw[Nu+1][Nv][Nw] and Pw[Nu][Nv][Nw] are equal
     2 ; Control Points Pw[Nu][Nv+1][Nw] and Pw[Nu][Nv][Nw] are equal
     3 ; Control Points Pw[Nu][Nv][Nw+1] and Pw[Nu][Nv][Nw] are equal
   ***********************************************************************/
NL_FLAG N_VolumeHasEqualCPts( NL_VOLUME *vol, NL_REAL Tol, NL_INDEX *Nu, NL_INDEX *Nv, NL_INDEX *Nw )
{
    /* Convert To Local Notation */

    NL_CPOINT Cp0, CpU, CpV, CpW, Rw;
    NL_REAL d;
    NL_INDEX i, j, k, m, n, o;
    m = vol->mesh->m;
    n = vol->mesh->n;
    o = vol->mesh->o;

    /*  For Every Control Point */

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 1; j <= n; j++ )
        {
            for ( k = 1; k <= n; k++ )
            {
                Cp0 = vol->mesh->Pw[i][j][k];

                /* When it has an upward U neighbor - check it */
                if( i < m )
                {
                    CpU = vol->mesh->Pw[i + 1][j][k];
                    N_Diff2CPts( Cp0, CpU, &Rw );
                    N_CPtMagnitude( Rw, &d );

                    if( d < Tol )
                    {
                        *Nu = i;
                        *Nv = j;
                        *Nw = k;
                        return (1);
                    }
                } /* end U dir check */

                /* When it has an upward V neighbor - check it */
                if( j < n )
                {
                    CpV = vol->mesh->Pw[i][j + 1][k];
                    N_Diff2CPts( Cp0, CpV, &Rw );
                    N_CPtMagnitude( Rw, &d );

                    if( d < Tol )
                    {
                        *Nu = i;
                        *Nv = j;
                        *Nw = k;
                        return (2);
                    }
                } /* end U dir check */

                /* When it has an upward W neighbor - check it */
                if( k < o )
                {
                    CpW = vol->mesh->Pw[i][j][k + 1];
                    N_Diff2CPts( Cp0, CpW, &Rw );
                    N_CPtMagnitude( Rw, &d );

                    if( d < Tol )
                    {
                        *Nu = i;
                        *Nv = j;
                        *Nw = k;
                        return (3);
                    }
                } /* end U dir check */
            }     /* end iter W direction */
        }         /* end iter V direction */
    }             /* end iter U direction */

    /* no equal control Points */

    return (0);
} /* end N_VolumeHasEqualCPts */


/*******************************************************************//**


   DESCRIPTION:

     This error routine checks if a volume  structure has  sufficient
     memory to store given control points and knots. A typical calling
     example is:

       NL_VOLUME  vol;
       NL_INDEX    np, mp, op, rk, sk, tk;
       NL_STRING   rname;
       ...
       (define vol, get np, mp, op, rk, sk, tk and rname);
       ...
       N_VolumeIsSized(&vol,np,mp,op,rk,sk,tk,rname);


   ACCESS:
   
     vol       , input ,  NURBS volume
     np,mp,op  , input ,  Expected highest indexes in control point array
     rk,sk,tk  , input ,  Expected highest indexes in knot vector arrays
     rname     , input ,  Routine name error is checked in


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VolumeIsSized( NL_VOLUME *vol, NL_INDEX np, NL_INDEX mp, NL_INDEX op, NL_INDEX rk, NL_INDEX sk, NL_INDEX tk, NL_STRING rname )
{
    NL_FLAG error = NL_NO;

    NL_INDEX n, m, o, ir, is, it;

    /* Get local notation */

    N_VolumeGetArraySizes( vol, &m, &n, &o, &ir, &is, &it );

    /* Check storage */

    if( m LT mp OR n LT np OR o LT op OR ir LT rk OR is LT sk OR it LT tk )
    {
        N_ErrSet( NL_STO_ERR, rname );
        error = NL_YES;
    }

    /* Exit */

    return (error);
} /* end N_VolumeIsSized */

/*******************************************************************//**
   DESCRIPTION:

     This routine creates an Nlib NURBS volume from component
     volume data.  The  resulting Nlib NURBS volume will 
     be clamped  and have no internal knots of multiplicity greater than
     degree. A typical calling example is as follows:

       NL_INDEX      k1, k2, k3;
       NL_DEGREE     m1, m2, m3;
       NL_PARAMETER  ru[2], rv[2], rw[2];
       NL_REAL       *U, *V, *W, ***w;
       NL_POINT      ***P;
       NL_VOLUME     vol;
       NL_STACKS     SG;
       ...
       (allocate and load arrays)
       ...

       N_VolumeInitArrays(&vol);
       N_VolumeConstruct(k1,k2,k3,m1,m2,m3,U,V,W,w,P,ru,rv,rw,&vol,&SG);

     If memory is  available, vol is  not initialized  and  the  routine 
     assumes that memory allocation has been done.  However,  it  checks  
     for the proper amount by looking  at the highest indexes  in  vol's  
     knot vector and control mesh objects.  NOTICE  that  the  five PROPS 
     flags and the Form Number are not required


   ACCESS:

     k1  , input  ,  Upper index of first sum ( k1+1 control  points  in 
                     the U-direction)
     k2  , input  ,  Upper index of second sum ( k2+1 control points  in 
                     the V-direction)
     k3  , input  ,  Upper index of third sum ( k3+1 control points  in 
                     the W-direction)
     m1  , input  ,  Degree of the U-basis functions
     m2  , input  ,  Degree of the V-basis functions
     m3  , input  ,  Degree of the W-basis functions
     U   , input  ,  The k1+m1+2 U-knots
     V   , input  ,  The k2+m2+2 V-knots
     W   , input  ,  The k3+m3+2 W-knots
     w   , input  ,  The (k1+1)x(k2+1)x(k3+1) weights.  
                     w[i][j][k] with 0<=i<=k1 , 0<=j<=k2 , 0<=k<=k3
     P   , input  ,  The  (k1+1)x(k2+1)x(k3+1)  Euclidean control points  
                     (not weighted). 
                     P[i][j][k] with 0<=i<=k1 , 0<=j<=k2 , 0<=k<=k3
     ru,rv,rw , input  ,  The bounding parameters
     vol , output ,  Nlib NURBS volume
     SG  , input  ,  vol's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR


   ***********************************************************************/

NL_FLAG N_VolumeConstruct( NL_INDEX k1, NL_INDEX k2, NL_INDEX k3, NL_DEGREE m1, NL_DEGREE m2, NL_DEGREE m3, NL_REAL *U, NL_REAL *V, NL_REAL *W, NL_REAL *** w, NL_POINT *** P, NL_PARAMETER ru[2], NL_PARAMETER rv[2], NL_PARAMETER rw[2], NL_VOLUME *vol, NL_STACKS *SG )
{
    NL_PRIVATE NL_STRING rname = _T("N_VolumeConstruct");

    NL_FLAG error = NL_NO, rat = NL_NO;

    NL_INDEX ii, jj, kk, mm, nn, oo, ir, is, it;

    NL_DEGREE pp, qq, rr;

    NL_REAL *UU, *VV, *WW, xx, yy, zz, ww, w000;

    NL_CPOINT *** Pw;

    /* Get Nlib notation */

    pp = m1;
    qq = m2;
    rr = m3;
    mm = k1;
    nn = k2;
    oo = k3;
    ir = mm + pp + 1;
    is = nn + qq + 1;
    it = oo + rr + 1;

    /* Check for various errors: degree, weights, knots */
    if( pp LT 1 OR pp GT NL_DMAX OR qq LT 1 OR qq GT NL_DMAX OR rr LT 1 OR rr GT NL_DMAX )
    {
        N_ErrSet( NL_DEG_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    w000 = w[0][0][0];

    for ( ii = 0; ii <= mm; ii++ ) /* also set rational flag */
        for ( jj = 0; jj <= nn; jj++ )
            for ( kk = 0; kk <= oo; kk++ )
            {
                if( w[ii][jj][kk]LT NL_WMIN OR w[ii][jj][kk]GT NL_WMAX )
                {
                    N_ErrSet( NL_WEI_ERR, rname );
                    error = NL_YES;
                    NL_OUT;
                }

                if( w[ii][jj][kk]NEQ w000 )
                    rat = NL_YES;
            }

    error = N_IgesValidate( U, ir, pp, ru );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_IgesValidate( V, is, qq, rv );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_IgesValidate( W, it, rr, rw );

    if( error EQ NL_YES )
        NL_OUT;

    /* Make a NURBS volume in the Nlib structures */
    /* (may not be a valid Nlib NURBS volume yet. */
    error = N_VolumeSizeArrays( vol, mm, nn, oo, pp, qq, rr, ir, is, it, rname, SG );

    if( error EQ NL_YES )
        NL_OUT;

    N_VolumeGetCPtsDegreesAndKnots( vol, &mm, &nn, &oo, &Pw, &pp, &qq, &rr, &ir, &is, &it, &UU, &VV, &WW );

    /* load the knots */

    for ( ii = 0; ii <= ir; ii++ )
        UU[ii] = U[ii];

    for ( jj = 0; jj <= is; jj++ )
        VV[jj] = V[jj];

    for ( kk = 0; kk <= it; kk++ )
        WW[kk] = W[kk];

    for ( ii = 0; ii <= mm; ii++ ) /* Weight and load the control points */
        for ( jj = 0; jj <= nn; jj++ )
            for ( kk = 0; kk <= oo; kk++ )
            {
                N_PtToXYZ( P[ii][jj][kk], &xx, &yy, &zz );

                if( rat == NL_YES )
                {
                    ww = w[ii][jj][kk];
                    xx = ww * xx;
                    yy = ww * yy;
                    zz = ww * zz;
                }
                else
                    ww = NL_NOW;
                N_CPtFromWxWyWz( xx, yy, zz, ww, &Pw[ii][jj][kk] );
            }

    /* Now clamp it to the NL_IGES bounds */
    error = ST_igsvcp( vol, ru, rv, rw, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* Sanitize the interior knots */
    error = ST_igsvsk( vol, SG );

    if( error EQ NL_YES )
        NL_OUT;

    /* It is now a valid Nlib NURBS volume */

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_VolumeConstruct */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to store members of a volume
     structure. Proper  error check is  performed in case memory 
     allocation fails. A typical calling example is:

       NL_VOLUME  *vol;
       NL_STACKS   S;
       ...
       vol = N_AllocVolume(&S);


   ACCESS:
   
     S , input  ,  Memory stack pointer


   RETURN CODES:

     vol  : Pointer to structure if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_VOLUME *N_AllocVolume( NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_AllocVolume");

    NL_VOLUME *vol;

    NL_VOLNODE *vod;

    /* Allocate memory for the structure */

    vol = (NL_VOLUME *)N_Malloc( sizeof( NL_VOLUME ) );

    if( vol EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        return NULL;
    }

    /* Put pointer on memory stack */

    vod = (NL_VOLNODE *)N_Malloc( sizeof( NL_VOLNODE ) );

    if( vod EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        N_Free( vol );
        vol = NULL;
        return NULL;
    }

    vod->ptr = vol;
    vod->next = S->vol;
    S->vol = vod;

    /* Exit */

    return vol;
} /* end N_AllocVolume */

/*******************************************************************//**


   DESCRIPTION:

     This routine allocates memory to  store a volume defined by 
     the usual parameters <m,n,o,p,q,r,ir,is,it>. Proper error  checks are 
     performed in case memory allocations fail. A typical calling
     example is:

       NL_VOLUME  *vol;
       NL_INDEX    m, n, o, ir, is, it;
       NL_DEGREE   p, q, r;
       NL_STACKS   S;
       ...
       (get m, n, 0, p, q, r, ir, is and it);
       ...
       vol = N_AllocVolumeAndArrays(m,n,0,p,q,r,ir,is,it,&S);


   ACCESS:
   
     m,n,o    , input  ,  Highest indexes in control point array
     p,q,r    , input  ,  Degrees in u- and v-directions
     ir,is,it , input  ,  Highest indexes in knot vector arrays
     S        , input  ,  Memory stacks pointer


   RETURN CODES:

     vol  : Pointer to volume if no error
     NULL : Memory allocation fails

   ***********************************************************************/

NL_VOLUME *N_AllocVolumeAndArrays( NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, NL_INDEX ir, NL_INDEX is, NL_INDEX it, NL_STACKS *S )
{
    NL_CMESH *msh;
    NL_KNOTVECTOR *knu, *knv, *knw;
    NL_VOLUME *vol;

    /* Allocate memory */
    msh = N_AllocCMeshAndArrays( m, n, o, S );

    if( msh EQ NULL )
        return NULL;

    knu = N_AllocKnotVectorAndArray( ir, S );

    if( knu EQ NULL )
        return NULL;

    knv = N_AllocKnotVectorAndArray( is, S );

    if( knv EQ NULL )
        return NULL;

    knw = N_AllocKnotVectorAndArray( it, S );

    if( knw EQ NULL )
        return NULL;

    vol = N_AllocVolume( S );

    if( vol EQ NULL )
        return NULL;

    /* Build volume structure */
    N_VolumeFromCMeshAndKnotVectors( vol, msh, p, q, r, knu, knv, knw );

    return vol;
} /* end N_AllocVolumeAndArrays */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if the volume is initialized to 
     the NULL volume. If yes, NL_TRUE is returned. Otherwise, NL_FALSE 
     is  returned. This  routine  is  used  to  check  if  memory 
     allocation is needed. A typical calling example is:

       NL_VOLUME  vol;
       ...
       if( N_AreVolumeArraysNULL(&vol) )  --> allocate memory;
     

   ACCESS:
   
     vol , input ,  NURBS volume


   RETURN CODES:

     NL_TRUE:  Volume is initialized to NULL (need memory)
     NL_FALSE: Volume is NOT initialized to NULL (no memory needed)

   ***********************************************************************/

NL_BOOLEAN N_AreVolumeArraysNULL( NL_VOLUME *vol )
{
    NL_DEGREE p, q, r;
    NL_CMESH *mesh;
    NL_KNOTVECTOR *knu, *knv, *knw;

    /* Get local notation */
    N_VolumeGetMeshAndKnotVectors( vol, &mesh, &p, &q, &r, &knu, &knv, &knw );

    /* Check initialization */
    if( mesh EQ NULL OR p EQ - 1 OR q EQ - 1 OR r EQ - 1 OR knu EQ NULL OR knv EQ NULL OR knw EQ NULL )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_AreVolumeArraysNULL */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine initializes a volume structure by  setting
     pointers to NULL and the degrees to -1. It is  used to  check if
     memory allocation is needed, i.e. if the surface is  initialized 
     to the  NULL surface, memory  is allocated to  hold the  control 
     net  and  knot vectors objects.  Otherwise  it is  assumed  that 
     memory has already been allocated. A typical calling example is:

       NL_VOLUME  vol;
       ...
       N_VolumeInitArrays(&vol);
     

   ACCESS:
   
     vol , in/out ,  NURBS surface


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeInitArrays( NL_VOLUME *vol )
{
    vol->mesh = NULL;
    vol->p = -1;
    vol->q = -1;
    vol->r = -1;
    vol->knu = NULL;
    vol->knv = NULL;
    vol->knw = NULL;
} /* end N_VolumeInitArrays */

/*******************************************************************//**


   DESCRIPTION:

     Given a volume object, this routine allocates memory to store
     control points and knots. A typical calling example is:

       NL_VOLUME  vol;
       NL_STACKS   S;
       ...
       N_AllocVolumeArrays(&vol,m,n,o,p,q,r,ir,is,it,&S);

     where  <m,n,o,p,q,r> and  <ir,is,it>  are the usual volume parameters. 
     Since the declaration "NL_VOLUME vol"  defines the data type and 
     allocates  memory, memory  is needed to  store the control net 
     and knot vector objects only.


   ACCESS:
   
     vol      , in/out ,  NURBS volume data type
     m,n,o    , input  ,  Highest indexes in control point array
     p,q,r    , input  ,  Degrees of the volume
     ir,is,it , input  ,  Highest indexes in knot vector arrays
     S        , input  ,  vol's memory stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_AllocVolumeArrays( NL_VOLUME *vol, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, NL_INDEX ir, NL_INDEX is, NL_INDEX it, NL_STACKS *S )
{
    NL_CMESH *mesh;

    NL_KNOTVECTOR *knu, *knv, *knw;

    /* Allocate memory */

    mesh = N_AllocCMeshAndArrays( m, n, o, S );

    if( mesh EQ NULL )
        return (1);

    knu = N_AllocKnotVectorAndArray( ir, S );

    if( knu EQ NULL )
        return (1);

    knv = N_AllocKnotVectorAndArray( is, S );

    if( knv EQ NULL )
        return (1);

    knw = N_AllocKnotVectorAndArray( it, S );

    if( knw EQ NULL )
        return (1);

    /* Build volume structure */

    N_VolumeFromCMeshAndKnotVectors( vol, mesh, p, q, r, knu, knv, knw );

    /* Exit */

    return (0);
} /* end N_AllocVolumeArrays */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine sets parameters to complete volume 
     definition. A typical calling example is:

       NL_VOLUME   vol;
       NL_INDEX    m, n, o, ir, is, it;
       NL_DEGREE   p, q, r;
       ...
       N_VolumeSetSizeIndices(&vol,n,m,p,q,r,s);


   ACCESS:
   
     vol      , in/out ,  NURBS volume
     n,m,o    , input  ,  Highest indexes in control mesh array
     p,q,r    , input  ,  Degrees
     ir,is,it , input  ,  Highest indexes in knot vector arrays


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeSetSizeIndices( NL_VOLUME *vol, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, NL_INDEX ir, NL_INDEX is, NL_INDEX it )
{
    vol->mesh->m = m;
    vol->mesh->n = n;
    vol->mesh->o = o;
    vol->p = p;
    vol->q = q;
    vol->r = r;
    vol->knu->m = ir;
    vol->knv->m = is;
    vol->knw->m = it;
} /* end N_VolumeSetSizeIndices */



/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if memory is needed to store a volume.
     If the volume is initiaized to the NULL volume  (via N_VolumeInitArrays()),
     memory is  allocated. If not, the  routine checkes if enough memory 
     is available. A typical calling example is:

       NL_VOLUME  vol;
       NL_INDEX    m, n, o, ir, is, it;
       NL_DEGREE   p, q, r;
       NL_STRING   rname;
       NL_STACKS   S;
       ...
       (get n, m, o, p, q, r, ir, is, it, and rname);
       ...
       N_VolumeInitArrays(&vol);
       N_VolumeSizeArrays(&vol,m,n,o,p,q,r,ir,is,it,rname,&S);

     IT IS  ASSUMED THAT MEMORY TO STORE THE NL_VOLUME STRUCTURE ITSELF IS 
     ALLOCATED IN THE CALLING ROUTINE.


   ACCESS:
   
     vol                  , in/out ,  NURBS volume to be created
     m,n,o,p,q,r,ir,is,it , input  ,  Usual volume parameters
     rname                , input  ,  Routine name
     S                    , input  ,  vol's stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VolumeSizeArrays( NL_VOLUME *vol, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, NL_INDEX ir, NL_INDEX is, NL_INDEX it, NL_STRING rname, NL_STACKS *S )
{
    NL_FLAG error;

    /* See if memory is needed */

    /* when vol is initialized to NULL */
    if( N_AreVolumeArraysNULL( vol ) )
    {
        /* allocate memory in vol for input size parameters */
        error = N_AllocVolumeArrays( vol, m, n, o, p, q, r, ir, is, it, S );

        if( error EQ 1 )
            return (1);
    }
    else
    {
        /* check current vol size values are greater than or equal to given size values */
        error = N_VolumeIsSized( vol, m, n, o, ir, is, it, rname );

        if( error EQ 1 )
            return (1);

        /* store input size values in vol */
        N_VolumeSetSizeIndices( vol, m, n, o, p, q, r, ir, is, it );
    }

    /* Exit */

    return (0);
} /* end N_VolumeSizeArrays */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine copies a given volume into a new volume.
     A typical calling example is:

       NL_VOLUME  volP, volQ;
       NL_STACKS   S;
       ...
       (define volP);
       ...
       N_VolumeInitArrays(&volQ);
       N_VolumeCopy(&volP,&volQ,&S);

     If  the  volume is  initialized to  the empty  volume (NULL), 
     memory will be allocated  for volQ.  Otherwise,  it is  assumed 
     that memory is already available.


   ACCESS:
   
     volP , input  ,  NURBS volume to be copied
     volQ , output ,  Copied volume
     S    , input  ,  volQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VolumeCopy( NL_VOLUME *volP, NL_VOLUME *volQ, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_VolumeCopy");

    NL_FLAG error;

    NL_INDEX i, j, k, n, m, o, ir, is, it;

    NL_DEGREE p, q, r;

    NL_CPOINT *** Pw, *** Qw;

    NL_REAL *UP, *VP, *WP, *UQ, *VQ, *WQ;

    /* Get local notation */

    N_VolumeGetCPtsDegreesAndKnots( volP, &n, &m, &o, &Pw, &p, &q, &r, &ir, &is, &it, &UP, &VP, &WP );

    /* See if memory is needed */

    error = N_VolumeSizeArrays( volQ, n, m, o, p, q, r, ir, is, it, rname, S );

    if( error EQ NL_YES )
        return (1);

    N_VolumeGetCPtsAndKnots( volQ, &Qw, &UQ, &VQ, &WQ );

    /* Copy the volume */

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            for ( k = 0; k <= o; k++ )
                N_CopyCPt( Pw[i][j][k], &Qw[i][j][k] );
        }
    }

    for ( i = 0; i <= ir; i++ )
        UQ[i] = UP[i];

    for ( j = 0; j <= is; j++ )
        VQ[j] = VP[j];

    for ( k = 0; k <= it; k++ )
        WQ[j] = WP[j];

    /* Exit */

    return (0);
} /* end N_VolumeCopy */



/*******************************************************************//**


   DESCRIPTION:

     This utility routine deallocates memory that stores volume data,
     i.e.   control  mesh   structure,  control   points,  knot  vector 
     structures  and knots. IT DOES NOT  DEALLOCATE MEMORY THAT STORES 
     THE NL_VOLUME STRUCTURE ITSELF. A typical calling example is:

       NL_VOLUME  *vol;
       NL_STACKS   S;
       ...
       N_FreeVolume(vol,&S);


   ACCESS:
   
     vol , input  ,  Surface pointer
     S   , input  ,  vol's stack


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_FreeVolume( NL_VOLUME *vol, NL_STACKS *S )
{
    NL_DEGREE p, q, r;

    NL_REAL *U, *V, *W;

    NL_CPOINT *** Pw;

    NL_CMESH *mesh;

    NL_KNOTVECTOR *knu, *knv, *knw;

    if( vol == NULL )
        return;

    /* Get locals */
    N_VolumeGetMeshAndKnotVectors( vol, &mesh, &p, &q, &r, &knu, &knv, &knw );

    if( mesh == NULL || knu == NULL || knv == NULL || knw == NULL )
        return;

    N_VolumeGetCPtsAndKnots( vol, &Pw, &U, &V, &W );

    /* Kill surface constituents */
    N_FreeCMesh( mesh, S );
    N_FreeCPt3dArray( Pw, S );
    N_FreeKnotVector( knu, S );
    N_FreeKnotVector( knv, S );
    N_FreeKnotVector( knw, S );
    N_FreeReal1dArray( U, S );
    N_FreeReal1dArray( V, S );
    N_FreeReal1dArray( W, S );
} /* end N_FreeVolume */


/*******************************************************************//**


   DESCRIPTION:

     This utility routine makes proper pointer assignments to define
     a volume  object  from  control mesh  and knot  vector objects.
     Memory for the various structures  is allocated  in the calling
     routine; only the  pointers are passed down. A  typical calling
     example is:
 
       NL_VOLUME      vol;
       NL_CMESH       mesh;
       NL_DEGREE      p, q, r;
       NL_KNOTVECTOR  knu, knv, knw;
       ...
       (define mesh, knu, knv, knw);
       ...
       N_VolumeFromCMeshAndKnotVectors(&vol,&mesh,p,q,r,&knu,&knv,&knw);


   ACCESS:
   
     vol         , in/out ,  NURBS volume
     mesh        , input  ,  Control mesh
     p,q,r       , input  ,  Degrees
     knu,knv,knw , input  ,  Knot vectors

   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeFromCMeshAndKnotVectors( NL_VOLUME *vol, NL_CMESH *mesh, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, NL_KNOTVECTOR *knu, NL_KNOTVECTOR *knv, NL_KNOTVECTOR *knw )
{
    vol->mesh = mesh;
    vol->p = p;
    vol->q = q;
    vol->r = r;
    vol->knu = knu;
    vol->knv = knv;
    vol->knw = knw;
} /* end N_VolumeFromCMeshAndKnotVectors */

/*******************************************************************//**


   DESCRIPTION:

     This routine  generates a  volume object given control points 
     and knots. It  allocates  memory to store the control mesh and 
     knot vector objects, and makes  proper pointer  assignments to
     create the volume object. All other memory allocation is done
     in  the calling  routine; only  pointers  are  passed  down. A 
     typical calling example is:

       NL_VOLUME   vol;
       NL_CPOINT   **Pw;
       NL_INDEX    n, m, o, ir, is, it;
       NL_DEGREE   p, q, r;
       NL_REAL     *U, *V, *W;
       NL_STACKS   S;
       ...
       (allocate memory for Pw, U and V);
       ...
       N_VolumeFromCPtsAndKnots(&vol,Pw,n,m,o,p,q,r,U,V,W,ir,is,it,&S);


   ACCESS:
   
     vol       , in/out ,  NURBS volume
     Pw        , input  ,  Control points
     n,m,o     , input  ,  Highest indexes in Pw
     p,q,r     , input  ,  Degrees
     U,V,W     , input  ,  Knots
     ir,is,it  , input  ,  Highest indexes in U, V, and W
     S         , input  ,  Stacks pointer
 

   RETURN CODES:

     0 : No error
     1 : Error detected and saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VolumeFromCPtsAndKnots( NL_VOLUME *vol, NL_CPOINT *** Pw, NL_INDEX m, NL_INDEX n, NL_INDEX o, NL_DEGREE p, NL_DEGREE q, NL_DEGREE r, NL_REAL *U, NL_REAL *V, NL_REAL *W, NL_INDEX ir, NL_INDEX is, NL_INDEX it, NL_STACKS *S )
{
    NL_CMESH *mesh;

    NL_KNOTVECTOR *knu, *knv, *knw;

    /* Allocate memory for control net and knotvector structures */

    mesh = N_AllocCMesh( S );

    if( mesh EQ NULL )
        return (1);

    knu = N_AllocKnotVector( S );

    if( knu EQ NULL )
        return (1);

    knv = N_AllocKnotVector( S );

    if( knv EQ NULL )
        return (1);

    knw = N_AllocKnotVector( S );

    if( knw EQ NULL )
        return (1);

    /* Make pointer assignments */

    N_CMeshFromCPts( mesh, Pw, m, n, o );
    N_KnotVectorFromRealArray( knu, U, ir );
    N_KnotVectorFromRealArray( knv, V, is );
    N_KnotVectorFromRealArray( knw, W, it );
    N_VolumeFromCMeshAndKnotVectors( vol, mesh, p, q, r, knu, knv, knw );

    /* Exit */

    return (0);
} /* end N_VolumeFromCPtsAndKnots */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine creates a volume from data saved in a file.
     The file pointer is passed in.  The file is assumed to be  opened
     and correctly positioned to read the volume. The data is assumed
     to be arranged as follows:

           m n o                 --> highest indexes in cp array
           p q r                 --> degrees    
           rat                   --> rationality (0-no,1-yes)
           x000 y000 z000 (w000) -->
           x001 y001 z001 (w001) -->
           . . .                 --> 
           x00o y00o z00o (w00o) -->
           x010 y010 z010 (w010) --> xyz(w) components of cp's
           x011 y011 z011 (w011) -->
           . . .                 -->
           x0no y0no z0no (w0no) -->
           . . .                 -->
           xmno ymno zmno (wmno) -->
           u0                    --> u-knots
           u1                    -->
            . . .                -->
           uir                   -->   where: ir = m+p+1   
           v0                    --> v-knots
           v1                    -->
           . . .                 -->
           vis                   -->   where: is = n+q+1
           w0                    --> w-knots
           w1                    -->
           . . .                 -->
           wit                   -->   where: it = o+r+1

     If memory is available, the data is copied into the  approriate
     members  of   the   volume  structure.  Otherwise,  memory  is 
     allocated first. A typical calling example is:

       NL_VOLUME  vol;
       FILE      *fptr;
       NL_STACKS  S;
       ...
       (open file, assign pointer, and position for surface read);
       ...
       N_VolumeInitArrays(&vol);
       N_VolumeReadFromFile(&vol,fptr,NL_YES,&S);

     IF NL_FLAG chk == NL_YES, THE FOLLOWING CHECKS ARE DONE:
       (1) CONSISTENCY, I.E. ir = m+p+1, is = n+q+1, it = o+r+1;
       (2) DEGREES ARE LESS THEN THE NL_MAXIMUM ALLOWED DEGREES;
       (3) WEIGHTS ARE IN THE ALLOWED RANGE; AND
       (4) INTERNAL KNOT MULTIPLICITIES ARE <= THE NL_DEGREE.
     IF chk = NL_YES, THE FOLLOWING SIMPLIFICATION IS PERFORMED:
       (1) IF ALL THE  WEIGHTS ARE  EQUAL, THE  NL_VOLUME IS CONVERTED 
           INTO A NON-RATIONAL NL_VOLUME.
     IF chk = NL_NO, THE NL_VOLUME IS TAKEN AS IS!!!!
     Rational volumes are stored, where the control points are read and
     written in homogeneous  format   wX, wY, wZ, W. 
     They can be converted to Euclidean by dividing thru by the weight.


   ACCESS:
   
     vol   , in/out ,  NURBS volume to be created
     fptr  , input  ,  Pointer to data file
     chk   , input  ,  Flag:
                         NL_YES: check volume, check for nonRational
                         NL_NO : do not check volume; use it as it is
     S     , input  ,  vol's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VolumeReadFromFile( NL_VOLUME *vol, FILE *fptr, NL_FLAG chk, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_VolumeReadFromFile");

    NL_FLAG rat, error = NL_NO;

    NL_INDEX i, j, k, m, n, o, ir, is, it;

    NL_DEGREE p, q, r;

    NL_CPOINT *** Pw;

    NL_REAL *U, *V, *W, x, y, z, wx, wy, wz, w;

    /* Get parameters from the top */
	if (EOF == N_FSCANF(fptr, _T("%ld%ld%ld"), &m, &n, &o)) { NL_ERROR(NL_SCN_ERR); }
	if (EOF == N_FSCANF(fptr, _T("%hd%hd%hd"), &p, &q, &r)) { NL_ERROR(NL_SCN_ERR); }
	if (EOF == N_FSCANF(fptr, _T("%hd"), &rat)) { NL_ERROR(NL_SCN_ERR); }

    ir = m + p + 1;
    is = n + q + 1;
    it = o + r + 1;

    /* See if memory is needed */
    error = N_VolumeSizeArrays( vol, m, n, o, p, q, r, ir, is, it, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_VolumeGetCPtsAndKnots( vol, &Pw, &U, &V, &W );

    /* Read in data */
    switch( rat )
    {
        case NL_NO: /* Non-rational */
            for ( i = 0; i <= m; i++ )
            {
                for ( j = 0; j <= n; j++ )
                {
                    for ( k = 0; k <= o; k++ )
                    {
						if (EOF == N_FSCANF(fptr, _T("%lf%lf%lf"), &x, &y, &z)) { NL_ERROR(NL_SCN_ERR); }
                        N_CPtFromWxWyWz( x, y, z, NL_NOW, &Pw[i][j][k] );
                    }
                }
            }
            break;

        case NL_YES: /* Rational */
            for ( i = 0; i <= m; i++ )
            {
                for ( j = 0; j <= n; j++ )
                {
                    for ( k = 0; k <= o; k++ )
                    {
						if (EOF == N_FSCANF(fptr, _T("%lf%lf%lf%lf"), &wx, &wy, &wz, &w)) { NL_ERROR(NL_SCN_ERR); }
                        N_CPtFromWxWyWz( wx, wy, wz, w, &Pw[i][j][k] );
                    }
                }
            }
            break;

        default: /* Wrong type */

            NL_ERROR( NL_CAL_ERR );
    }

    /* read Knots */
    for ( i = 0; i <= ir; i++ )
	if (EOF == N_FSCANF(fptr, _T("%lf"), &U[i])) { NL_ERROR(NL_SCN_ERR); }

    for ( j = 0; j <= is; j++ )
	if (EOF == N_FSCANF(fptr, _T("%lf"), &V[j])) { NL_ERROR(NL_SCN_ERR); }

    for ( k = 0; k <= it; k++ )
	if (EOF == N_FSCANF(fptr, _T("%lf"), &W[k])) { NL_ERROR(NL_SCN_ERR); }

    /* Check volume and prune */
    if( chk EQ NL_YES )
    {
        error = N_VolumeIsValid( vol, rname );

        if( error EQ NL_YES )
            NL_OUT;

        N_VolumePruneRat( vol );
    }

    /* Exit */
    EXIT:

    return (error);
} /* end N_VolumeReadFromFile */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine writes a volume to a file. The file pointer 
     is passed in.  The file is assumed to  be  opened  and  correctly 
     positioned to write the volume.  The  data  is assumed to be ar-
     ranged as follows:

           m n o                 --> highest indexes in cp array
           p q r                 --> degrees    
           rat                   --> rationality (0-no,1-yes)
           x000 y000 z000 (w000) -->
           x001 y001 z001 (w001) -->
           . . .                 --> 
           x00o y00o z00o (w00o) -->
           x010 y010 z010 (w010) --> xyz(w) components of cp's
           x011 y011 z011 (w011) -->
           . . .                 -->
           x0no y0no z0no (w0no) -->
           . . .                 -->
           xmno ymno zmno (wmno) -->
           u0                    --> u-knots
           u1                    -->
            . . .                -->
           uir                   -->   where: ir = m+p+1   
           v0                    --> v-knots
           v1                    -->
           . . .                 -->
           vis                   -->   where: is = n+q+1
           w0                    --> w-knots
           w1                    -->
           . . .                 -->
           wit                   -->   where: it = o+r+1

     A typical calling example is:

       NL_VOLUME  vol;
       FILE     *fptr;
       ...
       (open file, assign pointer, and position for volume write);
       ...
       N_VolumeWriteToFile(&vol,fptr);


   ACCESS:
   
     vol   , input  ,  NURBS volume to be saved
     fptr  , output ,  Pointer to the file


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VolumeWriteToFile( NL_VOLUME *vol, FILE *fptr )
{
    NL_PRIVATE NL_STRING rname = _T("N_VolumeWriteToFile");

    NL_FLAG rat, error = NL_NO;

    NL_INDEX i, j, k, m, n, o, ir, is, it;

    NL_DEGREE p, q, r;

    NL_CPOINT *** Pw;

    NL_REAL *U, *V, *W, wx, wy, wz, w;

    /* Get local notation */
    N_VolumeGetCPtsDegreesAndKnots( vol, &m, &n, &o, &Pw, &p, &q, &r, &ir, &is, &it, &U, &V, &W );

    /* Set rational flag */
    if( N_VolumeIsRat( vol ) )
        rat = NL_YES;
    else
        rat = NL_NO;

    /* Create the output file */
    N_FPRINTF( fptr, _T("%ld %ld %ld\n"), m, n, o );
    N_FPRINTF( fptr, _T("%hd %hd %hd\n"), p, q, r );
    N_FPRINTF( fptr, _T("%hd\n"), rat );

    /* output control points */
    switch( rat )
    {
        case NL_NO: /* Non-rational - don't write w component */
            for ( i = 0; i <= m; i++ )
            {
                for ( j = 0; j <= n; j++ )
                {
                    for ( k = 0; k <= o; k++ )
                    {
                        N_CPtToWxWyWz( Pw[i][j][k], &wx, &wy, &wz, &w );
                        N_FPRINTF( fptr, _T("%18.16f %18.16f %18.16f\n"), wx, wy, wz );
                    }
                }
            }
            break;

        case NL_YES: /* Rational - write w component */
            for ( i = 0; i <= m; i++ )
            {
                for ( j = 0; j <= n; j++ )
                {
                    for ( k = 0; k <= o; k++ )
                    {
                        N_CPtToWxWyWz( Pw[i][j][k], &wx, &wy, &wz, &w );
                        N_FPRINTF( fptr, _T("%18.16f %18.16f %18.16f %18.16f\n"), wx, wy, wz, w );
                    }
                }
            }
            break;

        default: /* Wrong type */
            NL_ERROR( NL_CAL_ERR );
    }

    /* output knots */
    for ( i = 0; i <= ir; i++ )
        N_FPRINTF( fptr, _T("%18.16f\n"), U[i] );

    for ( j = 0; j <= is; j++ )
        N_FPRINTF( fptr, _T("%18.16f\n"), V[j] );

    for ( k = 0; k <= it; k++ )
        N_FPRINTF( fptr, _T("%18.16f\n"), W[k] );

    /* Exit */
    EXIT:

    return (error);
} /* end N_VolumeWriteToFile */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine prunes a rational volume, i.e. it checks if 
     the weights are  equal, and if so, the  volume is converted into 
     non-rational form.

       NL_VOLUME  vol;
       ...
       N_VolumePruneRat(&vol);


   ACCESS:
   
     vol , in/out ,  NURBS volume


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumePruneRat( NL_VOLUME *vol )
{
    if( N_VolumeIsRat( vol ) )
    {
        if( N_VolumeAreWeightsEqual( vol ) )
            N_VolumeMakeNonRat( vol );
    }
} /* end N_VolumePruneRat */

/*******************************************************************//**


   DESCRIPTION:

     This  utility routine  maps a rational  volume to Euclidean space. 
     That is,  for each  control point Pw = (xw,yw,zw,w), it  computes a  
     new control point Qw = (x,y,z,NL_NOW). A typical calling example is:

       NL_VOLUME  vol;
       ...
       N_VolumeMakeNonRat(&vol);


   ACCESS:
   
     vol , in/out ,  NURBS volume


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeMakeNonRat( NL_VOLUME *vol )
{
    NL_INDEX i, j, k, m, n, o;

    NL_CPOINT *** Pw;

    /* Get net */

    N_VolumeGetCPts( vol, &m, &n, &o, &Pw );

    /* Map to Euclidean space */

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            for ( k = 0; k <= o; k++ )
            {
                N_CPtToPtNoW( Pw[i][j][k], &Pw[i][j][k] );
            }
        }
    }
} /* end N_VolumeMakeNonRat */

/*******************************************************************//**


   DESCRIPTION:

     Given a volume object, this routine extracts the denominator and
     creates a volume function from it. A typical calling example is:

       NL_VOLUME   vol;
       NL_VFUN     vfn;
       NL_STACKS   S;
       ...
       (define volume);
       ...
       N_VFuncInitArrays(&vfn);
       N_VolumeGetDenominatorFunc(&vol,&vfn,&S);

     Since  the declarations  "NL_VOLUME vol" and  "NL_VFUN vfn" define the 
     data types and allocate  memory, only the pointers are passed in. 
     If vfn is  initialized  to  NULL,  memory is  allocated  locally.
     Otherwise, it is assumed  that memory has  been allocated  in the
     calling routine.


   ACCESS:
   
     vol , input  ,  NURBS volume
     vfn , in/out ,  Volume function
     S   , input  ,  vfn's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VolumeGetDenominatorFunc( NL_VOLUME *vol, NL_VFUN *vfn, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_VolumeGetDenominatorFunc");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, m, n, o, ir, is, it;

    NL_DEGREE p, q, r;

    NL_REAL *U, *V, *W, *** fuvw, *UF, *VF, *WF;

    NL_CPOINT *** Pw;

    /* Get local notation */

    N_VolumeGetCPtsDegreesAndKnots( vol, &m, &n, &o, &Pw, &p, &q, &r, &ir, &is, &it, &U, &V, &W );

    /* See if memory is needed */

    error = N_VFuncSizeArrays( vfn, m, n, o, p, q, r, ir, is, it, rname, S );

    if( error EQ NL_YES )
        NL_OUT;

    N_VFuncGetKnots( vfn, &fuvw, &UF, &VF, &WF );

    /* Copy data */

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            for ( k = 0; k <= o; k++ )
            {
                N_CPtGetW( Pw[i][j][k], &fuvw[i][j][k] );
            }
        }
    }

    for ( i = 0; i <= ir; i++ )
        UF[i] = U[i];

    for ( j = 0; j <= is; j++ )
        VF[j] = V[j];

    for ( k = 0; k <= it; k++ )
        WF[k] = W[k];

    /* Exit */

    EXIT:

    return (error);
} /* end N_VolumeGetDenominatorFunc */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets control points and knots from a volume 
     object. A typical calling example is:

       NL_VOLUME   vol;
       NL_CPOINT   ***Pw;
       NL_REAL     *U, *V, *W;
       ...
       N_VolumeGetCPtsAndKnots(&sur,&Pw,&U,&V,&W);


   ACCESS:
   
     vol , input  ,  NURBS volume
     Pw  , output ,  Control point array
     U   , output ,  U-knot vector array
     V   , output ,  V-knot vector array
     W   , output ,  W-knot vector array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeGetCPtsAndKnots( NL_VOLUME *vol, NL_CPOINT **** Pw, NL_REAL ** U, NL_REAL ** V, NL_REAL ** W )
{
    *Pw = vol->mesh->Pw;
    *U = vol->knu->U;
    *V = vol->knv->U;
    *W = vol->knw->U;
} /* end N_VolumeGetCPtsAndKnots */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine detaches the control mesh and the knot  vector
     objects from a given volume object. A typical calling example is:

       NL_VOLUME      vol;
       NL_CMESH      *mesh;
       NL_DEGREE      p, q, r;
       NL_KNOTVECTOR  *knu, *knv, *knw;
       ...
       N_VolumeGetMeshAndKnotVectors(&vol,&mesh,&p,&q,&r,&knu,&knv,&knw);


   ACCESS:
   
     vol         , input  ,  NURBS volume
     mesh        , output ,  Control mesh
     p,q,r       , output ,  Degrees
     knu,knv,knw , output ,  Knot vectors


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeGetMeshAndKnotVectors( NL_VOLUME *vol, NL_CMESH ** mesh, NL_DEGREE *p, NL_DEGREE *q, NL_DEGREE *r, NL_KNOTVECTOR ** knu, NL_KNOTVECTOR ** knv, NL_KNOTVECTOR ** knw )
{
    *mesh = vol->mesh;
    *p = vol->p;
    *q = vol->q;
    *r = vol->r;
    *knu = vol->knu;
    *knv = vol->knv;
    *knw = vol->knw;
} /* end N_VolumeGetMeshAndKnotVectors */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine  breaks  a volume  object  down  to its 
     components, i.e. indexes, control point array and knot vectors.
     A typical calling example is:

       NL_VOLUME  vol;
       NL_INDEX    n, m, r, s;
       NL_CPOINT   **Pw;
       NL_DEGREE   p, q;
       NL_REAL     *U, *V;
       ...
       N_VolumeGetCPtsDegreesAndKnots(&vol,&n,&m,&o,&Pw,
                                         &p,&q,&r,&ir,&is,&it,&U,&V,&W);


   ACCESS:
   
     vol      , input  ,  NURBS volume
     n,m,o    , output ,  Highest indexes in Pw
     Pw       , output ,  Control points
     p,q,r    , output ,  Degrees
     ir,is,it , output ,  Highest indexes in U, V, and W
     U,V,W    , output ,  Knot vectors


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeGetCPtsDegreesAndKnots( NL_VOLUME *vol, NL_INDEX *m, NL_INDEX *n, NL_INDEX *o, NL_CPOINT **** Pw, NL_DEGREE *p, NL_DEGREE *q, NL_DEGREE *r, NL_INDEX *ir, NL_INDEX *is, NL_INDEX *it, NL_REAL ** U, NL_REAL ** V, NL_REAL ** W )
{
    *m = vol->mesh->m;
    *n = vol->mesh->n;
    *o = vol->mesh->o;
    *Pw = vol->mesh->Pw;
    *p = vol->p;
    *q = vol->q;
    *r = vol->r;
    *ir = vol->knu->m;
    *is = vol->knv->m;
    *it = vol->knw->m;
    *U = vol->knu->U;
    *V = vol->knv->U;
    *W = vol->knw->U;
} /* end N_VolumeGetCPtsDegreesAndKnots */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the highest indexes in volume 
     definition. A typical calling example is:

       NL_VOLUME  vol;
       NL_INDEX    m, n, o, ir, is, it;
       ...
       N_VolumeGetArraySizes(&vol,&m,&n,&o,&r,&s,&t);


   ACCESS:
   
     vol      , input  ,  NURBS volume
     m,n,o    , output ,  Highest indexes in Pw
     ir,is,it , output ,  Highest indexes in U, V, and W


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeGetArraySizes( NL_VOLUME *vol, NL_INDEX *m, NL_INDEX *n, NL_INDEX *o, NL_INDEX *ir, NL_INDEX *is, NL_INDEX *it )
{
    *m = vol->mesh->m;
    *n = vol->mesh->n;
    *o = vol->mesh->o;
    *ir = vol->knu->m;
    *is = vol->knv->m;
    *it = vol->knw->m;
} /* end N_VolumeGetArraySizes */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the degrees of a volume. A typical 
     calling example is:

       NL_VOLUME  vol;
       NL_DEGREE   p, q, r;
       ...
       N_VolumeGetDegrees(&vol,&p,&q);


   ACCESS:
   
     vol , input  ,  NURBS volume
     p,q , output ,  Degrees


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeGetDegrees( NL_VOLUME *vol, NL_DEGREE *p, NL_DEGREE *q, NL_DEGREE *r )
{
    *p = vol->p;
    *q = vol->q;
    *r = vol->r;
} /* end N_VolumeGetDegrees */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets knot vector objects from volume object. 
     A typical calling example is:

       NL_VOLUME     vol;
       NL_KNOTVECTOR  *knu, *knv, *knw;
       ...
       N_VolumeGetKnotVectors(&vol,&knu,&knv,&knw);


   ACCESS:
   
     vol         , input  ,  NURBS volume
     knu,knv,knw , output ,  Knot vector objects


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeGetKnotVectors( NL_VOLUME *vol, NL_KNOTVECTOR ** knu, NL_KNOTVECTOR ** knv, NL_KNOTVECTOR ** knw )
{
    *knu = vol->knu;
    *knv = vol->knv;
    *knw = vol->knw;
} /* end N_VolumeGetKnotVectors */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets the parameter bounds from a volume 
     object. A typical calling example is:

       NL_VOLUME    vol;
       NL_PARAMETER  ul, ur, vb, vt;
       ...
       N_VolumeGetParamBounds(&vol,&ul,&ur,&vb,&vt);


   ACCESS:
   
     vol   , input  ,  NURBS volume
     ul,ur , output ,  Parameter bounds in U-knot vector
     vb,vt , output ,  Parameter bounds in V-knot vector
     wf,wb , output ,  Parameter bounds in W-knot vector


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeGetParamBounds( NL_VOLUME *vol, NL_PARAMETER *ul, NL_PARAMETER *ur, NL_PARAMETER *vb, NL_PARAMETER *vt, NL_PARAMETER *wf, NL_PARAMETER *wb )
{
    *ul = vol->knu->U[0];
    *ur = vol->knu->U[vol->knu->m];
    *vb = vol->knv->U[0];
    *vt = vol->knv->U[vol->knv->m];
    *wf = vol->knw->U[0];
    *wb = vol->knw->U[vol->knw->m];
} /* end N_VolumeGetParamBounds */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets knot vectors info from volume object.
     A typical calling example is:

       NL_VOLUME  vol;
       NL_INDEX    r, s, t;
       NL_REAL     *U, *V, *W;
       ...
       N_VolumeGetKnots(&vol,&r,&s,&t,&U,&V,&W);


   ACCESS:
   
     vol   , input  ,  NURBS volume
     r,s,t , output ,  Highest indexes in U and V
     U,V,W , output ,  Knot vectors


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeGetKnots( NL_VOLUME *vol, NL_INDEX *r, NL_INDEX *s, NL_INDEX *t, NL_REAL ** U, NL_REAL ** V, NL_REAL ** W )
{
    *r = vol->knu->m;
    *s = vol->knv->m;
    *t = vol->knw->m;
    *U = vol->knu->U;
    *V = vol->knv->U;
    *W = vol->knw->U;
} /* end N_VolumeGetKnots */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets control mesh info from volume object.
     A typical calling example is:

       NL_VOLUME   vol; 
       NL_INDEX    m, n, o;
       NL_CPOINT   **Pw;
       ...
       N_VolumeGetCPts(&vol,&m,&n,&o,&Pw);


   ACCESS:
   
     vol   , input  ,  NURBS volume
     m,n,o , output ,  Highest indexes in Pw
     Pw    , output ,  Control points


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeGetCPts( NL_VOLUME *vol, NL_INDEX *m, NL_INDEX *n, NL_INDEX *o, NL_CPOINT **** Pw )
{
    *m = vol->mesh->m;
    *n = vol->mesh->n;
    *o = vol->mesh->o;
    *Pw = vol->mesh->Pw;
} /* end N_VolumeGetCPts */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine gets control points, knot vectors and knots 
     from a volume object. A typical calling example is:

       NL_VOLUME      vol;
       NL_CPOINT      ***Pw;
       NL_KNOTVECTOR  *knu, *knv, *knw;
       NL_REAL        *U, *V, *W;
       ...
       N_VolumeGetCPtsKnotVectorAndKnots(&vol,&Pw,&knu,&knv,&knw,&U,&V,&W);


   ACCESS:
   
     vol         , input  ,  NURBS volume
     Pw          , output ,  Control point array
     knu,knv,knw , output ,  Knot vector objects
     U,V,W       , output ,  Knot vector arrays


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeGetCPtsKnotVectorAndKnots( NL_VOLUME *vol, NL_CPOINT **** Pw, NL_KNOTVECTOR ** knu, NL_KNOTVECTOR ** knv, NL_KNOTVECTOR ** knw, NL_REAL ** U, NL_REAL ** V, NL_REAL ** W )
{
    *Pw = vol->mesh->Pw;
    *knu = vol->knu;
    *knv = vol->knv;
    *knw = vol->knw;
    *U = vol->knu->U;
    *V = vol->knv->U;
    *W = vol->knw->U;
} /* end N_VolumeGetCPtsKnotVectorAndKnots */

/*******************************************************************//**


   DESCRIPTION:

     Given a  volume  object, this  routine  maps  the  control mesh to
     Euclidean space. Memory for Euclidean points is allocated locally,
     however, the Euclidean  mesh  data  type  is  declared  (allocated)
     in the calling routine. A typical calling example is:

       NL_INDEX    ku, lu, kv, lv;
       NL_VOLUME   vol;
       NL_EMESH    msh;
       NL_STACKS   S;
       ...
       (define volume);
       ...
       N_VolumeGetEMesh(&vol,ku,lu,kv,lv,kw,lw,&msh,&S);

     Since  the  declarations  "NL_VOLUME vol" and  "NL_EMESH msh" define the 
     data types and allocate memory, only the pointers are passed in. 


   ACCESS:
   
     vol               , input  ,  NURBS volume
     ku,lu,kv,lv,kw,lw , input  ,  Start and  end  indexes in  uvw-directions.
                                   Only the  control  points  Pw[ku][kv][kw],...,
                                   Pw[lu][lv][lw]  are   mapped.  The   Euclidean 
                                   points   are   stored    in   P[0][0][0],...,
                                   P[lu-ku][lv-kv][lw-kw].
     msh               , output ,  Point mesh (MEMORY  TO  STORE  VERTICES  IS
                                   ALLOCATED LOCALLY)
     S                 , input  ,  msh's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VolumeGetEMesh( NL_VOLUME *vol, NL_INDEX ku, NL_INDEX lu, NL_INDEX kv, NL_INDEX lv, NL_INDEX kw, NL_INDEX lw, NL_EMESH *msh, NL_STACKS *S )
{
    NL_PRIVATE NL_STRING rname = _T("N_VolumeGetEMesh");

    NL_INDEX i, j, k, m, n, o;

    NL_CPOINT *** Pw;

    NL_POINT *** P;

    /* Get local notation */

    N_VolumeGetCPts( vol, &m, &n, &o, &Pw );

    /* Check indexes */

    if( ku GT lu OR ku LT 0 OR lu GT m OR kv GT lv OR kv LT 0 OR lv GT n OR kw GT lw OR kw LT 0 OR lw GT o )
    {
        N_ErrSet( NL_IND_ERR, rname );
        return (1);
    }

    /* Map control points */

    P = N_AllocPt3dArray( lu - ku, lv - kv, lw - kw, S );

    if( P EQ NULL )
        return (1);

    for ( i = ku; i <= lu; i++ )
    {
        for ( j = kv; j <= lv; j++ )
        {
            for ( k = kw; k <= lw; k++ )
            {
                N_CPtToPtEuclid( Pw[i][j][k], &P[i - ku][j - kv][k - kw] );
            }
        }
    }

    /* Build point mesh structure */

    N_EMeshFromPts( msh, lu - ku, lv - kv, lw - kw, P );

    /* Exit */

    return (0);
} /* end N_VolumeGetEMesh */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes the min-max box of a NURBS volume. A 
     typical calling example is:

       NL_VOLUME     vol;
       NL_MINMAXBOX  box;
       ...
       (define vol);
       ...
       N_volbox(&vol,&box);


   ACCESS:
   
     vol , input  ,  NURBS volume
     box , output ,  Min-max box


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_volbox */
NL_FLAG N_VolumeGetBBox( NL_VOLUME *vol, NL_MINMAXBOX *box )
{
    NL_FLAG error = NL_NO;

    NL_INDEX m, n, o, k;

    NL_EMESH msh;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_VolumeGetArraySizes( vol, &m, &n, &o, &k, &k, &k );

    /* Map control net */

    error = N_VolumeGetEMesh( vol, 0, m, 0, n, 0, o, &msh, &S );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get min-max box */

    N_EMeshGetBBox( &msh, box );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_VolumeGetBBox */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine sets control point and knot vector pointers 
     of a volume object. A typical calling example is:

       NL_VOLUME  vol;
       NL_CPOINT   **Pw;
       NL_REAL     *U, *V, *W;
       ...
       N_VolumeSetCPtsAndKnots(&vol,Pw,U,V,W);


   ACCESS:
   
     vol , in/out ,  NURBS volume
     Pw  , input  ,  Control point array
     U   , input  ,  U-knot vector array
     V   , input  ,  V-knot vector array
     W   , input  ,  W-knot vector array


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeSetCPtsAndKnots( NL_VOLUME *vol, NL_CPOINT *** Pw, NL_REAL *U, NL_REAL *V, NL_REAL *W )
{
    vol->mesh->Pw = Pw;
    vol->knu->U = U;
    vol->knv->U = V;
    vol->knw->U = W;
} /* end N_VolumeSetCPtsAndKnots */

/*******************************************************************//**


   DESCRIPTION:

     Given a volume object, this routine scales the knot vectors to a 
     given MinMaxBox. A typical calling example is:

       NL_VOLUME    vol;
       NL_MINMAXBOX  R;
       ...
       (get MinMaxBox R);
       ...
       ST_VolumeReparam(&vol,R,dir);

     The  knot vectors  are rescaled IN-PLACE, i.e. the original knots 
     are destroyed.


   ACCESS:
   
     vol , in/out ,  NURBS volume
     R   , input  ,  Parameter rectangle
     dir , input  ,  Flag:
                       NL_UDIR : Rescale u-knot vector
                       NL_VDIR : Rescale v-knot vector
                       NL_WDIR : Rescale w-knot vector
                       NL_UVDIR: Rescale u & v knot vectors
                       NL_UWDIR: Rescale u & w knot vectors
                       NL_VWDIR: Rescale v & w knot vectors
                       NL_UVWDIR: Rescale u, v & w knot vectors


   RETURN CODES:

     None

   ***********************************************************************/
NL_VOID ST_VolumeReparam( NL_VOLUME *vol, NL_MINMAXBOX R, NL_FLAG dir )
{
    NL_INDEX i, j, k, ir, is, it;

    NL_DEGREE p, q, r;

    NL_REAL *U, *V, *W, fac, a, b, c, d, e, f, u0, v0, w0;

    /* Get local notation */

    N_VolumeGetDegrees( vol, &p, &q, &r );
    N_VolumeGetKnots( vol, &ir, &is, &it, &U, &V, &W );
    N_GetBBoxData( &R, &a, &b, &c, &d, &e, &f );

    /* Compute new knots */

    if( dir & NL_UDIR )
    {
        if( a NEQ U[0]OR b NEQ U[ir] )
        {
            u0 = U[0];
            fac = (b - a) / (U[ir] - U[0]);

            for ( i = 0; i <= p; i++ )
                U[i] = a;

            for ( i = p + 1; i <= ir - p - 1; i++ )
                U[i] = fac * (U[i] - u0) + a;

            for ( i = ir - p; i <= ir; i++ )
                U[i] = b;
        }
    }

    if( dir & NL_VDIR )
    {
        if( c NEQ V[0]OR d NEQ V[is] )
        {
            v0 = V[0];
            fac = (d - c) / (V[is] - V[0]);

            for ( j = 0; j <= q; j++ )
                V[j] = c;

            for ( j = q + 1; j <= is - q - 1; j++ )
                V[j] = fac * (V[j] - v0) + c;

            for ( j = is - q; j <= is; j++ )
                V[j] = d;
        }
    }

    if( dir & NL_WDIR )
    {
        if( e NEQ W[0]OR f NEQ W[it] )
        {
            w0 = W[0];
            fac = (f - e) / (W[it] - W[0]);

            for ( k = 0; k <= r; k++ )
                W[k] = e;

            for ( k = r + 1; k <= it - r - 1; k++ )
                W[k] = fac * (W[k] - w0) + e;

            for ( k = it - r; k <= it; k++ )
                W[k] = f;
        }
    }
} /* end ST_VolumeReparam */

/*******************************************************************//**


   DESCRIPTION:

     This  utility  routine compacts  control  point and  knot  vector 
     arrays. That  is, given  a volume  with  control point  and knot 
     vector arrays larger than required. This routine redefines  these 
     arrays to the appropriate  sizes which  makes volume  definition 
     more memory efficient. A typical calling example is:

       NL_VOLUME  vol;
       NL_STACKS   SG;
       ...
       (define volume);
       ...
       N_VolumeCompress(&vol,&SG);

     SG MUST BE vol'is STACK, I.E.  ALL MEMORY ALLOCATED FOR  vol, MUST 
     BE ON SG.


   ACCESS:
   
     vol , in/out ,  NURBS volume
     SG  , input  ,  vol'is stack
 

   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VolumeCompress( NL_VOLUME *vol, NL_STACKS *SG )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, m, n, o, ir, is, it;

    NL_REAL *U, *V, *W, *UU, *VV, *WW;

    NL_CPOINT *** Pw, *** Qw;

    /* Get volume data */

    N_VolumeGetArraySizes( vol, &m, &n, &o, &ir, &is, &it );
    N_VolumeGetCPtsAndKnots( vol, &Pw, &U, &V, &W );

    /* Allocate memory for new control point and knot vector arrays */

    Qw = N_AllocCPt3dArray( m, n, o, SG );

    if( Qw EQ NULL )
        NL_QUIT;

    UU = N_AllocReal1dArray( ir, SG );

    if( UU EQ NULL )
        NL_QUIT;

    VV = N_AllocReal1dArray( is, SG );

    if( VV EQ NULL )
        NL_QUIT;

    WW = N_AllocReal1dArray( it, SG );

    if( WW EQ NULL )
        NL_QUIT;

    /* Copy control points and knots */

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            for ( k = 0; k <= o; k++ )
            {
                N_CopyCPt( Pw[i][j][k], &Qw[i][j][k] );
            }
        }
    }

    for ( i = 0; i <= ir; i++ )
        UU[i] = U[i];

    for ( j = 0; j <= is; j++ )
        VV[j] = V[j];

    for ( k = 0; k <= it; k++ )
        WW[k] = W[k];

    /* Redefine volume and kill old memory */
    N_VolumeSetCPtsAndKnots( vol, Qw, UU, VV, WW );

    N_FreeCPt3dArray( Pw, SG );
    N_FreeReal1dArray( U, SG );
    N_FreeReal1dArray( V, SG );
    N_FreeReal1dArray( W, SG );

    /* Exit */

    EXIT:

    return (error);
} /* end N_VolumeCompress */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if the volume is rational or not. A
     typical calling example is:

       NL_VOLUME  vol;
       ...
       if( N_VolumeIsRat(&vol) )  --> handle rational case;
     

   ACCESS:
   
     vol , input ,  NURBS volume


   RETURN CODES:

     NL_TRUE:  Volume is rational
     NL_FALSE: Volume is NOT rational

   ***********************************************************************/

NL_BOOLEAN N_VolumeIsRat( NL_VOLUME *vol )
{
    NL_INDEX m, n, o;

    NL_REAL w;

    NL_CPOINT *** Pw;

    /* Get local notation */

    N_VolumeGetCPts( vol, &m, &n, &o, &Pw );

    /* Check rationality */

    N_CPtGetW( Pw[0][0][0], &w );

    if( w NEQ NL_NOW )
    {
        return NL_TRUE;
    }
    else
    {
        return NL_FALSE;
    }
} /* end N_VolumeIsRat */

/*******************************************************************//**
 
 
   DESCRIPTION:
 
     This utility routine checks if a NURBS volume is degenerate to a
     single point. A typical calling example is:
 
       NL_VOLUME  vol;
       ...
       if( N_VolumeIsDegen(&vol) )  --> handle point;
     
 
   ACCESS:
   
     vol  , input ,  NURBS volume
 
 
   RETURN CODES:
 
     NL_TRUE : Volume is a point
     NL_FALSE: Volume is NOT a point
 
   ***********************************************************************/

NL_BOOLEAN N_VolumeIsDegen( NL_VOLUME *vol )
{
    NL_FLAG dst = NL_YES;

    NL_INDEX i, j, k, m, n, o;

    NL_REAL d, fac;

    NL_POINT Q, M;

    NL_CPOINT *** Pw;

    /* Get local notation */

    N_VolumeGetCPts( vol, &m, &n, &o, &Pw );

    /* Check if volume is a point */

    /* get point center of gravity */
    fac = 1.0 / ((m + (NL_REAL)1) * (n + (NL_REAL)1) * (o + (NL_REAL)1));
    N_CopyPt( NL_ZERO, &M );

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            for ( k = 0; k <= o; k++ )
            {
                N_CPtToPtEuclid( Pw[i][j][k], &Q );
                N_Sum2Pts( M, Q, &M );
            }
        }
    }

    N_ScalePt( fac, M, &M );

    /* check point distances from center point */

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            for ( k = 0; k <= o; k++ )
            {
                N_CPtToPtEuclid( Pw[i][j][k], &Q );
                N_DistPtPt( Q, M, &d );

                if( d GT NL_MTOL )
                {
                    dst = NL_NO;
                    break;
                }
            }

            if( dst EQ NL_NO )
                break;
        }

        if( dst EQ NL_NO )
            break;
    }

    if( dst EQ NL_YES )
        return NL_TRUE;
    else
        return NL_FALSE;
} /* end N_VolumeIsDegen */

/*******************************************************************//**


   DESCRIPTION:

     This utility routine checks if volume weights are equal or not. A 
     typical calling example is:

       NL_VOLUME  vol;
       ...
       if( N_VolumeAreWeightsEqual(&vol) )  --> volume weights are equal;
     

   ACCESS:
   
     vol , input ,  NURBS volume


   RETURN CODES:

     NL_TRUE : Volume weights are equal
     NL_FALSE: Volume weights are NOT equal

   ***********************************************************************/
NL_BOOLEAN N_VolumeAreWeightsEqual( NL_VOLUME *vol )
{
    NL_INDEX i, j, k, m, n, o;

    NL_REAL w, wmin, wmax;

    NL_CPOINT *** Pw;

    /* Get mesh */
    N_VolumeGetCPts( vol, &m, &n, &o, &Pw );

    /* Get min and max weights */
    N_CPtGetW( Pw[0][0][0], &w );
    wmin = wmax = w;

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            for ( k = 0; k <= o; k++ )
            {
                N_CPtGetW( Pw[i][j][k], &w );

                if( w LT wmin )
                    wmin = w;

                if( w GT wmax )
                    wmax = w;
            }
        }
    }

    /* Return equality */
    return (fabs( wmax - wmin )LT NL_WTOL ? NL_TRUE : NL_FALSE);
} /* end N_VolumeAreWeightsEqual */

/*******************************************************************//**


   DESCRIPTION:

     This utility  routine checks if two volumes are equal or not.
     Two volumes are equal when they have the same degrees, number of knots,
     weights, and control points and all those values are within
     tolerance of one another.

     A typical calling example is:

       NL_REAL    tol3d, tol1d;
       NL_VOLUME volP, volQ;
       NL_STACKS  SG;
       ...
       (get volP and volQ, choose tolerances);
       ...
       if( N_VolumesAreEqual(&volP,&volQ,tol3d,tol1d) ) --> handle equal case;
     

   ACCESS:
   
     volP , input ,  NURBS volume
     volQ , input ,  NURBS volume
     tol3d, input ,  Tolerance for coincident control point checking
     tol1d, input ,  Tolerance for coincident knot checking


   RETURN CODES:

     NL_TRUE : Volumes are equal
     NL_FALSE: Volumes are NOT equal

   ***********************************************************************/

NL_BOOLEAN N_VolumesAreEqual( NL_VOLUME *volP, NL_VOLUME *volQ, NL_REAL tol3d, NL_REAL tol1d )
{
    NL_INTEGER ii, jj, kk ;

    NL_INDEX   Pn, Pm, Po, Qn, Qm, Qo ;         /* Highest indices in control point arrays */ 
                          
    NL_CPOINT ***Pw, ***Qw ;                    /* array of control points */
                          
    NL_DEGREE  Pp, Pq, Pr, Qp, Qq, Qr ;         /* volume degrees */
                          
    NL_INDEX   Pir, Pis, Pit, Qir, Qis, Qit ;   /* Highest indices in KnotArrays */
                          
    NL_REAL   *PU, *PV, *PW, *QU, *QV, *QW ;    /* arrays of knots (multiple knots are represented multiple times ) */

    NL_BOOLEAN bRtn ;

    NL_REAL d;

    NL_CPOINT Rw;

    /* volume data */
    N_VolumeGetCPtsDegreesAndKnots(volP, &Pm, &Pn, &Po, &Pw, &Pp, &Pq, &Pr, &Pir, &Pis, &Pit, &PU, &PV, &PW ) ;
    N_VolumeGetCPtsDegreesAndKnots(volQ, &Qm, &Qn, &Qo, &Qw, &Qp, &Qq, &Qr, &Qir, &Qis, &Qit, &QU, &QV, &QW ) ;

    /* check sizes */
    bRtn = (   Pp  == Qp  && Pq  == Qq  && Pr  == Qr     /* same degrees */
            && Pm  == Qm  && Pn  == Qn  && Po  == Qo     /* same control point counts */
            && Pir == Qir && Pis == Qis && Pit == Qit) ; /* same knot counts */

    /* check control points */
    if(bRtn)
      {
        for(ii=0;ii<=Pm && bRtn ;ii++)
          {
            for(jj=0;jj<=Pn && bRtn;jj++)
              {
                for(kk=0;kk<=Po && bRtn;kk++)
                  {
                    /* distance between control points */
                    N_Diff2CPts( Pw[ii][jj][kk], Qw[ii][jj][kk], &Rw );
                    N_CPtMagnitude( Rw, &d );

                    /* check for equivalent locations */
                    bRtn &= (d <= tol3d) ;

                  } /* end iter kk, every control point */
              } /* end iter jj, every control point */
          } /* end iter ii, every control point */
      } /* end control points check */

    /* check U knots */
    if(bRtn)
      {
        /* multiple knots are represented multiple times */
        for(ii=0;ii<=Pir && bRtn ;ii++)
          {
            /* check for equivalent locations */
            bRtn &= (fabs(PU[ii] - QU[ii]) <= tol1d) ;

          } /* end iter every control point */
      } /* end knots check */

    /* check V knots */
    if(bRtn)
      {
        /* multiple knots are represented multiple times */
        for(ii=0;ii<=Pis && bRtn ;ii++)
          {
            /* check for equivalent locations */
            bRtn &= (fabs(PV[ii] - QV[ii]) <= tol1d) ;

          } /* end iter every control point */
      } /* end knots check */

    /* check W knots */
    if(bRtn)
      {
        /* multiple knots are represented multiple times */
        for(ii=0;ii<=Pit && bRtn ;ii++)
          {
            /* check for equivalent locations */
            bRtn &= (fabs(PW[ii] - QW[ii]) <= tol1d) ;

          } /* end iter every control point */
      } /* end knots check */

    /* all done */
    return bRtn;

} /* end N_VolumesAreEqual */

/*******************************************************************//**


   DESCRIPTION:

     This routine  computes a point on a NURBS volume by using
     curve evaluations only. A typical calling example is:

       NL_VOLUME    vol;
       NL_PARAMETER  u, v, w;
       NL_POINT      S;
       ...
       (define vol, get u, v, and w);
       ...
       N_VolumeEval(&vol,u,v,w,NL_LEFT,NL_RIGHT,NL_LEFT,&S);


   ACCESS:
   
     vol         , input  ,  NURBS volume
     u,vw        , input  ,  Parameter values 
     ufl,vfl,wfl , input  ,  Flags: NL_LEFT : t is in [t[j],t[j+1])
                                    NL_RIGHT: t is in (t[j],t[j+1]]
                                    (t is either u or v)
     S           , output ,  Point on the volume


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_VolumeEval( NL_VOLUME *vol, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, NL_POINT *S )
{
    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, tmp1, tmp2, tmp3, usp, vsp, wsp;

    NL_DEGREE p, q, r;

    NL_REAL NU[NL_MAXDEG + 1], NV[NL_MAXDEG + 1], NW[NL_MAXDEG + 1];

    NL_KNOTVECTOR *knu, *knv, *knw;

    NL_CPOINT *** Pw, Tw, Sw, Rw;

    /* Get local notation */

    p = vol->p;
    knu = vol->knu;
    q = vol->q;
    knv = vol->knv;
    r = vol->r;
    knw = vol->knw;
    Pw = vol->mesh->Pw;

    /* Compute non-vanishing B-splines */

    error = N_BasisEval( knu, p, u, ufl, NU, &usp );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisEval( knv, q, v, vfl, NV, &vsp );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisEval( knw, r, w, wfl, NW, &wsp );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute the point on the volume */

    tmp1 = usp - p;
    tmp2 = vsp - q;
    tmp3 = wsp - r;

    Sw.x = 0.0;
    Sw.y = 0.0;
    Sw.z = 0.0;
    Sw.w = 0.0;

    for ( i = 0; i <= p; i++, tmp1++ )
    {
        Tw.x = 0.0;
        Tw.y = 0.0;
        Tw.z = 0.0;
        Tw.w = 0.0;

        for ( j = 0; j <= q; j++ )
        {
            Rw.x = 0.0;
            Rw.y = 0.0;
            Rw.z = 0.0;
            Rw.w = 0.0;

            for ( k = 0; k <= r; k++ )
            {
                N_VectorBlendCPt( NW[k], Pw[tmp1][tmp2 + j][tmp3 + k], &Rw );
            }
            N_VectorBlendCPt( NV[j], Rw, &Tw );
        }
        N_VectorBlendCPt( NU[i], Tw, &Sw );
    }

    /*        Sw.x = 0.0;   Sw.y = 0.0;   Sw.z = 0.0;   Sw.w = 0.0;       */
    /*        for ( i=0; i<=p; i++, tmp1++ )                              */
    /*        {                                                           */
    /*          Tw.x = 0.0;   Tw.y = 0.0;   Tw.z = 0.0;   Tw.w = 0.0;     */
    /*          for ( j=0; j<=q; j++ )                                    */
    /*          {                                                         */
    /*            N_VectorBlendCPt(NV[j],Pw[tmp1][tmp2+j],&Tw);                   */
    /*          }                                                         */
    /*                                                                    */
    /*          alpha = NU[i];                                            */
    /*          Sw.x = Sw.x + alpha*Tw.x;                                 */
    /*          Sw.y = Sw.y + alpha*Tw.y;                                 */
    /*          Sw.z = Sw.z + alpha*Tw.z;                                 */
    /*          if ( Tw.w NEQ NL_NOW )   Sw.w = Sw.w + alpha*Tw.w;          */
    /*          else                   Sw.w = NL_NOW;                       */
    /*        }                                                           */

    N_CPtToPtEuclid( Sw, S );

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_VolumeEval */

/* local statics to avoid repeated allocs and free on every evaluate 
** BD[udr][vdr][wdr][p][q][r] = all nonZero basis functions for each 
**                              requested derivative where           
**          BD[i][j][k] begins the nonZero basis functions for the   
**          volume's ith U, jth V, and kth W derivatives, eg         
**       BD[0][1][0] = d (Volume)/dv
**       BD[1][0][1] = d2(Volume)/dudw
**       BD[1][1][1] = d3(Volume)/dudvdw
*/

static NL_REAL ****** BD = NULL, ***** BD5 = NULL, **** BD4 = NULL, *** BD3 = NULL, ** BD2 = NULL, *BD1 = NULL;
static NL_INDEX Xudr = 0;
static NL_INDEX Xvdr = 0;
static NL_INDEX Xwdr = 0;
static NL_INDEX Xp = 0;
static NL_INDEX Xq = 0;
static NL_INDEX Xr = 0;

static void N_volder_alloc( NL_INDEX udr, NL_INDEX vdr, NL_INDEX wdr, NL_INDEX p, NL_INDEX q, NL_INDEX r )
{
    NL_INDEX i, j, k, l, m, l2, l3, l4, l5, l6;

    BD = (NL_REAL ****** )N_Malloc( (udr + 1) * sizeof( NL_REAL ***** ), NULL );
    BD5 = (NL_REAL ***** )N_Malloc( (udr + 1) * (vdr + 1) * sizeof( NL_REAL **** ), NULL );
    BD4 = (NL_REAL **** )N_Malloc( (udr + 1) * (vdr + 1) * (wdr + 1) * sizeof( NL_REAL *** ), NULL );
    BD3 = (NL_REAL *** )N_Malloc( (udr + 1) * (vdr + 1) * (wdr + 1) * (p + 1) * sizeof( NL_REAL ** ), NULL );
    BD2 = (NL_REAL ** )N_Malloc( (udr + 1) * (vdr + 1) * (wdr + 1) * (p + 1) * (q + 1) * sizeof( NL_REAL * ), NULL );
    BD1 = (NL_REAL *)N_Malloc( (udr + 1) * (vdr + 1) * (wdr + 1) * (p + 1) * (q + 1) * (r + 1) * sizeof( NL_REAL ), NULL );
    Xudr = udr;
    Xvdr = vdr;
    Xwdr = wdr;
    Xp = p;
    Xq = q;
    Xr = r;

    /* set indirection pointers From BD to BD1 */
    l2 = l3 = l4 = l5 = l6 = 0;

    for ( i = 0; i <= udr; i++ )
    {
        BD[i] = &BD5[l2];

        for ( j = 0; j <= vdr; j++ )
        {
            BD5[l2 + j] = &BD4[l3];

            for ( k = 0; k <= wdr; k++ )
            {
                BD4[l3 + k] = &BD3[l4];

                for ( l = 0; l <= p; l++ )
                {
                    BD3[l4 + l] = &BD2[l5];

                    for ( m = 0; m <= q; m++ )
                    {
                        BD2[l5 + m] = &BD1[l6];
                        l6 = l6 + r + 1;
                    }
                    l5 = l5 + q + 1;
                }
                l4 = l4 + p + 1;
            }
            l3 = l3 + wdr + 1;
        }
        l2 = l2 + vdr + 1;
    }

    return;
} /* end N_volder_alloc */

/* make this accessible for external N_volder_free */
NL_VOID N_volder_free()
{
    if( BD NEQ NULL )
    {
        N_Free( BD );
        BD = NULL;
    }

    if( BD1 NEQ NULL )
    {
        N_Free( BD1 );
        BD1 = NULL;
    }

    if( BD2 NEQ NULL )
    {
        N_Free( BD2 );
        BD2 = NULL;
    }

    if( BD3 NEQ NULL )
    {
        N_Free( BD3 );
        BD3 = NULL;
    }

    if( BD4 NEQ NULL )
    {
        N_Free( BD4 );
        BD4 = NULL;
    }

    if( BD5 NEQ NULL )
    {
        N_Free( BD5 );
        BD5 = NULL;
    }
    Xudr = Xvdr = Xwdr = Xp = Xq = Xr = 0;
} /* end N_volder_free */

static NL_INDEX N_volder_test( NL_INDEX udr, NL_INDEX vdr, NL_INDEX wdr, NL_INDEX p, NL_INDEX q, NL_INDEX r )
{
    if( BD == NULL || BD5 == NULL || BD4 == NULL || BD3 == NULL || BD2 == NULL || BD1 == NULL )
        return (1);

    if( udr > Xudr || vdr > Xvdr || wdr > Xwdr || p > Xp || q > Xq || r > Xr )
        return (1);
    return (0);
} /* end N_volder_test */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes derivatives of a NURBS volume  by evaluating 
     all   non-vanishing  basis  functions  and  their  derivatives, and 
     multiplying  them  by  appropriate  control  points.  Discontinuous  
     volumes can also be handled by passing NL_LEFT/NL_RIGHT flags. A typical 
     calling example is:

       NL_VOLUME     vol;
       NL_PARAMETER  u, v, w;
       NL_INDEX      udr, vdr, wdr;
       NL_POINT      **SD;
       ...
       (define vol, get u, v, w, udr, vdr, wdr, and allocate memory for SD);
       ...
       N_VolumeDerivs(&vol,u,v,w,NL_LEFT,NL_RIGHT,NL_LEFT,NL_TRUE,udr,vdr,wdr,SD);

       After repeated use, you can call N_volder_free() to free up static memory


   ACCESS:
   
     vol         , input  ,  NURBS volume
     u,v         , input  ,  Parameter values 
     ufl,vfl,wfl , input  ,  Flags:
                           NL_LEFT : t is in [t[j],t[j+1])
                                  (NL_RIGHT DERIVATIVES REQUIRED)
                           NL_RIGHT: t is in (t[j],t[j+1]]
                                  (NL_LEFT DERIVATIVES REQUIRED)
                           (t is either u or v)
     mfl         , input  ,  Flag: 
                                  NL_TRUE AND udr == vdr == wdr: compute upper  
                                         half only of the derivative matrix
                                  ELSE: compute full derivative matrix
     udr,vdr,wdr , input  ,  Highest derivatives required
     SD          , output ,  Derivatives;  SD[k][l][m]  is   the   (k,l,m)-th 
                             derivative.  MEMORY  FOR SD  MUST BE ALLOCATED 
                             IN THE CALLING ROUTINE TO HOLD SD[udr][vdr][wdr].
                             Only computes the upper triangle terms, that
                             is udr + vdr + wdr < Max(udr,vdr,wdr)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

/* NL_FLAG  N_volder */
NL_FLAG N_VolumeDerivs( NL_VOLUME *vol, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, NL_FLAG mfl, NL_INDEX udr, NL_INDEX vdr, NL_INDEX wdr, NL_POINT *** SD )
{
    NL_PRIVATE NL_STRING rname = _T("N_VolumeDerivs");

    NL_FLAG jump, error = NL_NO;

    NL_INDEX i, j, k, l, m, n, usp, vsp, wsp, l2, l3, l4;

    NL_DEGREE p, q, r;

    NL_REAL alpha;

    /* NL_KNOTVECTOR *knu, *knv, *knw;  unused */

    NL_POINT P, *pPoint;

    NL_CPOINT *** Pw;

    /* Get local notation */

    p = vol->p;
    /* knu = vol->knu; */
    q = vol->q;
    /* knv = vol->knv; */
    r = vol->r;
    /* knw = vol->knw; */
    Pw = vol->mesh->Pw;

    /* if the saved is not the same Allocate 6D array */

    if( N_volder_test( udr, vdr, wdr, p, q, r ) )
    {
        N_volder_free();
        N_volder_alloc( udr, vdr, wdr, p, q, r );
    }

    if( BD EQ NULL OR BD5 EQ NULL OR BD4 EQ NULL OR BD3 EQ NULL OR BD2 EQ NULL OR BD1 EQ NULL )
    {
        N_ErrSet( NL_MEM_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    /* Make pointer assignments to non-zero basis array */

    /*
    BD, output,  Derivatives computed at u,v,w; BD[l][m][n][i][j][k] 
    is the  (l,m,n)-th  derivative  of  the  basis 
    function NU[ku-p+i]*NV[kv-q+j]*NW[kw-r+k], 
    where  u is in  {u[ku],u[ku+1]}, 
    NL_PRIVATE  NL_STRING  rname = "N_VolumeNonRatBasisDerivs");
    v is in  {v[kv],v[kv+1]},
    w is in  {w[kw],w[kw+1]}.
    MEMORY  FOR  BD  MUST  BE  ALLOCATED  IN THE 
    CALLING  ROUTINE TO  HOLD DERIVATIVES  UP TO 
    BD[udr][vdr][wdr][p][q][r].
    
    */

    /* Compute basis function derivatives */

    if( N_VolumeIsRat( vol ) )
        error = N_VolumeRatBasisDerivs( vol, u, v, w, ufl, vfl, wfl, mfl, udr, vdr, wdr, BD, &usp, &vsp, &wsp );
    else
        error = N_VolumeNonRatBasisDerivs( vol, u, v, w, ufl, vfl, wfl, mfl, udr, vdr, wdr, BD, &usp, &vsp, &wsp );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute derivatives */

    if( mfl EQ NL_TRUE AND udr EQ vdr AND udr EQ wdr )
        jump = 1;
    else
        jump = 2;
    l2 = usp - p;
    l3 = vsp - q;
    l4 = wsp - r;

    switch( jump )
    {
        case 1: /* Compute upper half of derivative matrix */
            for ( l = 0; l <= udr; l++ )
            {
                for ( m = 0; m <= vdr - l; m++ )
                {
                    for ( n = 0; n <= wdr - l - m; n++ )
                    {
                        pPoint = &( SD[l][m][n] );
                        pPoint->x = 0.0;
                        pPoint->y = 0.0;
                        pPoint->z = 0.0;
                    }
                }
            }

            for ( i = 0; i <= p; i++ )
            {
                for ( j = 0; j <= q; j++ )
                {
                    for ( k = 0; k <= r; k++ )
                    {
                        N_CPtToPtEuclid( Pw[i + l2][j + l3][k + l4], &P );

                        for ( l = 0; l <= udr; l++ )
                        {
                            for ( m = 0; m <= vdr - l; m++ )
                            {
                                for ( n = 0; n <= wdr - l - m; n++ )
                                {
                                    alpha = BD[l][m][n][i][j][k];
                                    pPoint = &( SD[l][m][n] );
                                    pPoint->x += alpha * P.x;
                                    pPoint->y += alpha * P.y;
                                    pPoint->z += alpha * P.z;
                                }
                            }
                        }
                    }
                }
            }

            break;

        case 2: /* Compute full derivative matrix */
            for ( l = 0; l <= udr; l++ )
            {
                for ( m = 0; m <= vdr; m++ )
                {
                    for ( n = 0; n <= wdr; n++ )
                    {
                        pPoint = &( SD[l][m][n] );
                        pPoint->x = 0.0;
                        pPoint->y = 0.0;
                        pPoint->z = 0.0;
                    }
                }
            }

            for ( i = 0; i <= p; i++ )
            {
                for ( j = 0; j <= q; j++ )
                {
                    for ( k = 0; k <= r; k++ )
                    {
                        N_CPtToPtEuclid( Pw[i + l2][j + l3][k + l4], &P );

                        for ( l = 0; l <= udr; l++ )
                        {
                            for ( m = 0; m <= vdr; m++ )
                            {
                                for ( n = 0; n <= wdr; n++ )
                                {
                                    alpha = BD[l][m][n][i][j][k];
                                    pPoint = &( SD[l][m][n] );
                                    pPoint->x += alpha * P.x;
                                    pPoint->y += alpha * P.y;
                                    pPoint->z += alpha * P.z;
                                }
                            }
                        }
                    }
                }
            }

            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* Exit */

    EXIT:

    return (error);
} /* end N_VolumeDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  evaluates  all  non-vanishing  trivariate  rational  
     or non-rational basis functions and their  derivatives at a given  
     parameter value. It is assumed that the knot vectors are clamped, 
     i.e., they are repeated with multiplicity = degree + 1. A typical
     calling example is:

       NL_VOLUME    vol;
       NL_PARAMETER  u, v, w;
       NL_INDEX      udr, vdr, wdr, usp, vsp, wsp;
       NL_REAL       ****BDer;
       ...
       (define vol; get u, v, w, udr, vdr, and wdr);
       ...
       N_VolumeBasisDerivs(&vol,u,v,NL_LEFT,NL_RIGHT,NL_LEFT,NL_TRUE,udr,vdr,wdr,BDer,&usp,&vsp,&wsp);


   ACCESS:
   
     vol         , input  ,  NURBS volume
     u,v,w       , input  ,  Parameter values
     ufl,vfl,wfl , input  ,  Flags:
                           NL_LEFT : t is in [t[j],t[j+1])
                                  (NL_RIGHT DERIVATIVES REQUIRED)
                           NL_RIGHT: t is in (t[j],t[j+1]]
                                  (NL_LEFT DERIVATIVES REQUIRED)
                           (t is either u or v)
     mfl         , input  ,  Flag: 
                           NL_TRUE AND udr == vdr == wdr: compute upper  
                                  half only of the derivative matrix
                           ELSE: compute full derivative matrix
     udr,vdr,wdr , input  ,  Highest derivatives required
     BDer        , output ,  Derivatives  computed at u; BDer[l][m][n][i][j][k] is 
                             the (l,m,n)-th derivative of the basis function  
                             N[ku-p+i][kv-q+j][kw-r-k],  where  
                               u  is in {u[ku], u[ku+1]} and  
                               v  is in {v[kv], v[kv+1]} and
                               w  is in {w[kw], w[kw+1]}. 
                             MEMORY FOR BD MUST BE ALLOCATED IN THE CALLING 
                             ROUTINE TO HOLD UP TO BD[udr][vdr][wdr][p][q][r], 
                             where p, q, and r are volume degrees in u-, 
                             v-, and w-directions, respectively.
     usp,vsp,wsp , output ,  Indices of knot spans containing u, v, and w.


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VolumeBasisDerivs( NL_VOLUME *vol, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, NL_FLAG mfl, NL_INDEX udr, NL_INDEX vdr, NL_INDEX wdr, NL_REAL ****** BDer, NL_INDEX *usp, NL_INDEX *vsp, NL_INDEX *wsp )
{

    NL_FLAG error = NL_NO;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get derivatives of basis functions */

    if( N_VolumeIsRat( vol ) )
    {
        error = N_VolumeRatBasisDerivs( vol, u, v, w, ufl, vfl, wfl, mfl, udr, vdr, wdr, BDer, usp, vsp, wsp );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        error = N_VolumeNonRatBasisDerivs( vol, u, v, w, ufl, vfl, wfl, mfl, udr, vdr, wdr, BDer, usp, vsp, wsp );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_VolumeBasisDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This routine evaluates the rational or non  rational basis function 
     of a volume corresponding to a given index. It is assumed that the 
     end knots  are  repeated with  multiplicity = degree + 1. A typical 
     calling example is:

       NL_VOLUME    vol;
       NL_INDEX      i, j, k;
       NL_PARAMETER  u, v, w;
       NL_REAL       R;
       ...
       (define vol, get i, j, k, u, v, and w);
       ...
       N_VolumeBasisIEval(&vol,i,j,k,u,v,w,NL_LEFT,NL_RIGHT,NL_LEFT,&R);


   ACCESS:
   
     vol         , input  ,  NURBS volume
     i,j,k       , input  ,  Indices of basis function
     u,v,w       , input  ,  Parameter values
     ufl,vfl,wfl , input  ,  Flag:
                           NL_LEFT : t is in [t[j],t[j+1]) 
                           NL_RIGHT: t is in (t[j],t[j+1]]
                           (t is either u or v) 
     R           , output ,  Basis function computed at (u,v)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VolumeBasisIEval( NL_VOLUME *vol, NL_INDEX i, NL_INDEX j, NL_INDEX k, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, NL_REAL *R )
{

    NL_FLAG error = NL_NO;

    NL_DEGREE p, q, r;

    NL_REAL NU, NV, NW;

    NL_KNOTVECTOR *knu, *knv, *knw;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Compute basis function */

    if( N_VolumeIsRat( vol ) )
    {
        error = N_VolumeRatBasisIEval( vol, i, j, k, u, v, w, ufl, vfl, wfl, R );

        if( error EQ NL_YES )
            NL_OUT;
    }
    else
    {
        N_VolumeGetKnotVectors( vol, &knu, &knv, &knw );
        N_VolumeGetDegrees( vol, &p, &q, &r );

        error = N_BasisIEval( knu, i, p, u, ufl, &NU );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisIEval( knv, j, q, v, vfl, &NV );

        if( error EQ NL_YES )
            NL_OUT;

        error = N_BasisIEval( knw, k, r, w, wfl, &NW );

        if( error EQ NL_YES )
            NL_OUT;

        *R = NU * NV * NW;
    }

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_VolumeBasisIEval */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  evaluates  all  non-vanishing  trivariate rational  
     basis functions  and  their  derivatives  at a  given  parameter 
     value. It is  assumed that the knot  vectors are  clamped, i.e., 
     end knots are repeated with multiplicity = degree + 1. A typical
     calling example is:

       NL_VOLUME    vol;
       NL_PARAMETER  u, v, w;
       NL_INDEX      udr, vdr, wdr, usp, vsp, wsp;
       NL_REAL       ****RD;
       ...
       (define vol, get u, v, w, udr, vdr, wdr, and allocate memory for RD);
       ...
       N_VolumeRatBasisDerivs(&vol,u,v,w,NL_LEFT,NL_RIGHT,NL_LEFT,NL_TRUE,udr,vdr,wdr,RD,&usp,&vsp,&wsp);


   ACCESS:
   
     vol         , input  ,  NURBS volume
     u,v,w       , input  ,  Parameter values
     ufl,vfl,wfl , input  ,  Flags:
                               NL_LEFT : t is in [t[j],t[j+1])
                                      (NL_RIGHT DERIVATIVES REQUIRED)
                               NL_RIGHT: t is in (t[j],t[j+1]]
                                      (NL_LEFT DERIVATIVES REQUIRED)
                               (t is either u or v)
     mfl         , input  ,  Flag: 
                               NL_TRUE AND udr == vdr == wdr: compute upper  
                                      half only of the derivative matrix
                               ELSE: compute full derivative matrix
     udr,vdr,wdr , input  ,  Highest derivatives required
     RD          , output ,  Derivatives  computed  at u; RD[l][m][n][i][j][k] 
                             is the  (l,m,n)-th  derivative  of  the basis 
                             function  R[ku-p+i][kv-q+j][kw-r+k], where  
                               u is in {u[ku],u[ku+1]} and 
                                 NL_PRIVATE  NL_STRING  rname = "N_VolumeRatBasisDerivs");
v is in {v[kv],v[kv+1]} and
                               w is in {w[kw],w[kw+1]}.
                             MEMORY  FOR  RD  MUST  BE  ALLOCATED IN THE 
                             CALLING  ROUTINE TO  HOLD DERIVATIVES UP TO 
                             RD[udr][vdr][wdr][p][q][r].
     usp,vsp,wsp , output ,  Indices of knot spans containing u, v, and w.


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_VolumeRatBasisDerivs( NL_VOLUME *vol, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, NL_FLAG mfl, NL_INDEX udr, NL_INDEX vdr, NL_INDEX wdr, NL_REAL ****** RD, NL_INDEX *usp, NL_INDEX *vsp, NL_INDEX *wsp )
{

    NL_PRIVATE NL_STRING rname = _T("N_VolumeRatBasisDerivs");

    NL_FLAG jump, error = NL_NO;

    NL_INDEX i, j, k, l, m, n, ku, kv, kw, rr, s1, s2, s3, rk, pk, j1, j2, mder, t1, t2, t3;

    /*       NL_INTEGER     tri[NL_MAXDER+1][NL_MAXDER+1];         */

    NL_REAL *left, *right, ndu[NL_MAXDEG + 1][NL_MAXDEG + 1], saved, temp, a[2][NL_MAXDEG + 1], DU[NL_MAXDER + 1][NL_MAXDEG + 1], DV[NL_MAXDER + 1][NL_MAXDEG + 1], DW[NL_MAXDER + 1][NL_MAXDEG + 1], *U, *V, *W, ww, v1, v2, v3, d[NL_MAXDER + 1][NL_MAXDER + 1][NL_MAXDER + 1], dd, *tu, ( *tv)[NL_MAXDEG + 1];

    NL_DEGREE p, q, r;

    NL_KNOTVECTOR *knu, *knv, *knw;

    NL_CPOINT *** Pw;

    /* Assign local pointers */

    left = &a[0][0];
    right = &a[1][0];
    tu = left;
    tv = ndu;

    /* Get local notation */

    p = vol->p;
    knu = vol->knu;
    s1 = knu->m;
    U = knu->U;
    q = vol->q;
    knv = vol->knv;
    s2 = knv->m;
    V = knv->U;
    r = vol->r;
    knw = vol->knw;
    s3 = knw->m;
    W = knw->U;
    Pw = vol->mesh->Pw;

    /* Check parameters and order of derivatives */

    if( u LT U[0]OR v LT V[0]OR w LT W[0]OR u GT U[s1]OR v GT V[s2]OR w GT W[s2] )
    {
        N_ErrSet( NL_PAR_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    if( udr GT NL_MAXDER OR vdr GT NL_MAXDER OR wdr GT NL_MAXDER )
    {
        N_ErrSet( NL_MXD_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    /* Get derivatives of the u-direction basis functions */

    error = N_BasisFindSpan( knu, p, u, ufl, &ku );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get maximum u-direction derivative index and set zero derivatives */

    mder = NL_MIN( p, udr );

    for ( k = p + 1; k <= udr; k++ )
    {
        for ( j = 0; j <= p; j++ )
        {
            DU[k][j] = 0.0;
        }
    }

    /* Compute the u-direction basis functions */

    ndu[0][0] = 1.0;

    for ( j = 1; j <= p; j++ )
    {
        left[j] = u - U[ku + 1 - j];
        right[j] = U[ku + j] - u;
        saved = 0.0;

        for ( rr = 0; rr < j; rr++ )
        {
            ndu[j][rr] = right[rr + 1] + left[j - rr];
            temp = ndu[rr][j - 1] / ndu[j][rr];
            ndu[rr][j] = saved + right[rr + 1] * temp;
            saved = left[j - rr] * temp;
        }
        ndu[j][j] = saved;
    }

    /* Load the u-direction basis functions */

    for ( j = 0; j <= p; j++ )
        DU[0][j] = ndu[j][p];

    /* Compute u-direction derivatives */

    for ( rr = 0; rr <= p; rr++ )
    {
        s1 = 0;
        s2 = 1;
        a[0][0] = 1.0;

        for ( k = 1; k <= mder; k++ )
        {
            dd = 0.0;
            rk = rr - k;
            pk = p - k;

            if( rr GE k )
            {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                dd = a[s2][0] * ndu[rk][pk];
            }

            if( rk GE - 1 )
                j1 = 1;
            else
                j1 = -rk;

            if( (rr - 1)LE pk )
                j2 = k - 1;
            else
                j2 = p - rr;

            for ( j = j1; j <= j2; j++ )
            {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                dd += a[s2][j] * ndu[rk + j][pk];
            }

            if( rr LE pk )
            {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][rr];
                dd += a[s2][k] * ndu[rr][pk];
            }
            DU[k][rr] = dd;
            N_SwapIntegers( &s1, &s2 );
        }
    }

    /* Multiply through by the correct factors for u-direction derivatives */

    rr = p;

    for ( k = 1; k <= mder; k++ )
    {
        for ( j = 0; j <= p; j++ )
        {
            DU[k][j] *= rr;
        }
        rr *= (p - k);
    }

    /* Do the same for the v-direction */

    error = N_BasisFindSpan( knv, q, v, vfl, &kv );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get maximum v-direction derivative index and set zero derivatives */

    mder = NL_MIN( q, vdr );

    for ( k = q + 1; k <= vdr; k++ )
    {
        for ( j = 0; j <= q; j++ )
        {
            DV[k][j] = 0.0;
        }
    }

    /* Compute the v-direction basis functions */

    ndu[0][0] = 1.0;

    for ( j = 1; j <= q; j++ )
    {
        left[j] = v - V[kv + 1 - j];
        right[j] = V[kv + j] - v;
        saved = 0.0;

        for ( rr = 0; rr < j; rr++ )
        {
            ndu[j][rr] = right[rr + 1] + left[j - rr];
            temp = ndu[rr][j - 1] / ndu[j][rr];
            ndu[rr][j] = saved + right[rr + 1] * temp;
            saved = left[j - rr] * temp;
        }
        ndu[j][j] = saved;
    }

    /* Load the v-direction basis functions */

    for ( j = 0; j <= q; j++ )
        DV[0][j] = ndu[j][q];

    /* Compute v-direction derivatives */

    for ( rr = 0; rr <= q; rr++ )
    {
        s1 = 0;
        s2 = 1;
        a[0][0] = 1.0;

        for ( k = 1; k <= mder; k++ )
        {
            dd = 0.0;
            rk = rr - k;
            pk = q - k;

            if( rr GE k )
            {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                dd = a[s2][0] * ndu[rk][pk];
            }

            if( rk GE - 1 )
                j1 = 1;
            else
                j1 = -rk;

            if( (rr - 1)LE pk )
                j2 = k - 1;
            else
                j2 = q - rr;

            for ( j = j1; j <= j2; j++ )
            {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                dd += a[s2][j] * ndu[rk + j][pk];
            }

            if( rr LE pk )
            {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][rr];
                dd += a[s2][k] * ndu[rr][pk];
            }
            DV[k][rr] = dd;
            N_SwapIntegers( &s1, &s2 );
        }
    }

    /* Multiply through by the correct factors for the v-direction derivatives */

    rr = q;

    for ( k = 1; k <= mder; k++ )
    {
        for ( j = 0; j <= q; j++ )
        {
            DV[k][j] *= rr;
        }
        rr *= (q - k);
    }

    /* Do the same for the w-direction */

    error = N_BasisFindSpan( knw, r, w, wfl, &kw );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get maximum w-direction derivative index and set zero derivatives */

    mder = NL_MIN( r, wdr );

    for ( k = r + 1; k <= wdr; k++ )
    {
        for ( j = 0; j <= r; j++ )
        {
            DW[k][j] = 0.0;
        }
    }

    /* Compute the w-direction basis functions */

    ndu[0][0] = 1.0;

    for ( j = 1; j <= r; j++ )
    {
        left[j] = w - W[kw + 1 - j];
        right[j] = W[kw + j] - w;
        saved = 0.0;

        for ( rr = 0; rr < j; rr++ )
        {
            ndu[j][rr] = right[rr + 1] + left[j - rr];
            temp = ndu[rr][j - 1] / ndu[j][rr];
            ndu[rr][j] = saved + right[rr + 1] * temp;
            saved = left[j - rr] * temp;
        }
        ndu[j][j] = saved;
    }

    /* Load the w-direction basis functions */

    for ( j = 0; j <= r; j++ )
        DW[0][j] = ndu[j][r];

    /* Compute w-direction derivatives */

    for ( rr = 0; rr <= r; rr++ )
    {
        s1 = 0;
        s2 = 1;
        a[0][0] = 1.0;

        for ( k = 1; k <= mder; k++ )
        {
            dd = 0.0;
            rk = rr - k;
            pk = r - k;

            if( rr GE k )
            {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                dd = a[s2][0] * ndu[rk][pk];
            }

            if( rk GE - 1 )
                j1 = 1;
            else
                j1 = -rk;

            if( (rr - 1)LE pk )
                j2 = k - 1;
            else
                j2 = r - rr;

            for ( j = j1; j <= j2; j++ )
            {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                dd += a[s2][j] * ndu[rk + j][pk];
            }

            if( rr LE pk )
            {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][rr];
                dd += a[s2][k] * ndu[rr][pk];
            }
            DW[k][rr] = dd;
            N_SwapIntegers( &s1, &s2 );
        }
    }

    /* Multiply through by the correct factors for the w-direction derivatives */

    rr = r;

    for ( k = 1; k <= mder; k++ )
    {
        for ( j = 0; j <= r; j++ )
        {
            DW[k][j] *= rr;
        }
        rr *= (r - k);
    }

    /* Get derivatives of denominator                   */
    /* d[l][m][n] =  Sum_ijk(  Pw[s1+i][s2+j][s3+k].w   */
    /*                       * DU[l][i]                 */
    /*                       * DV[m][j]                 */
    /*                       * DW[n][k]                 */

    s1 = ku - p;
    s2 = kv - q;
    s3 = kw - r;

    for ( n = 0; n <= wdr; n++ )
    {
        for ( i = 0; i <= p; i++ )
        {
            for ( j = 0; j <= q; j++ )
            {
                tv[i][j] = 0.0;

                for ( k = 0; k <= r; k++ )
                {
                    tv[i][j] += Pw[s1 + i][s2 + j][s3 + k].w * DW[n][k];
                }
            }
        }

        for ( m = 0; m <= vdr; m++ )
        {
            for ( i = 0; i <= p; i++ )
            {
                tu[i] = 0.0;

                for ( j = 0; j <= q; j++ )
                {
                    tu[i] += tv[i][j] * DV[m][j];
                }
            }

            for ( l = 0; l <= udr; l++ )
            {
                d[l][m][n] = 0.0;

                for ( i = 0; i <= p; i++ )
                {
                    d[l][m][n] += tu[i] * DU[l][i];
                }
            }
        }
    }

    /* Compute derivatives of rational basis */

    /*        tmp = NL_MAX3(udr,vdr,wdr);                                */
    /*                                                                */
    /*           Pascal triangle first                                */
    /*        tri[0][0] = 1;                                          */
    /*        tri[1][0] = 1;   tri[1][1] = 1;                         */
    /*        if ( tmp GT 1 )                                         */
    /*        {                                                       */
    /*          tri[2][0] = 1;   tri[2][1] = 2;   tri[2][2] = 1;      */
    /*          if ( tmp GT 2 )                                       */
    /*          {                                                     */
    /*            for( k=3; k<=tmp; k++ )                             */
    /*            {                                                   */
    /*              tri[k][0] = 1;                                    */
    /*              rr = k/2;  j2 = 1;                                */
    /*              for( j=1; j<=rr; j++ )                            */
    /*              {                                                 */
    /*                j1          = tri[k-1][j];                      */
    /*                tri[k][j]   = tri[k-1][j]+j2;                   */
    /*                tri[k][k-j] = tri[k][j];                        */
    /*                j2          = j1;                               */
    /*              }                                                 */
    /*              tri[k][k] = 1;                                    */
    /*            }                                                   */
    /*          }                                                     */
    /*        }                                                       */

    if( mfl EQ NL_TRUE AND udr EQ vdr AND udr EQ wdr )
        jump = 1;
    else
        jump = 2;

    switch( jump )
    {
        case 1: /* Compute upper half of derivative matrix */
            /* RD[l,m,n] = 1/d[0,0,0] * (  Sum_ijk=1_to_lmn( Comb(l,i) * Comb(m,j) * Comb(n,k) * RD[l-i,m-j,n-k]  */
            /*                           - Sum_i  =1_to_l  ( Comb(l,i) * d[i,0,0] * RD[l-i,m,n]                   */
            /*                           - Sum_j  =1_to_m  ( Comb(m,j) * d[0,j,0] * RD[l,m-j,n]                   */
            /*                           - Sum_k  =1_to_n  ( Comb(n,k) * d[0,0,k] * RD[l,m,n-k]                   */
            /*                           - Sum_ij =1_to_lm ( Comb(l,i) * Comb(m,j) * d[i,j,0] * RD[l-i,m-j,n]     */
            /*                           - Sum_jk =1_to_mn ( Comb(m,j) * Comb(n,k) * d[0,j,k] * RD[l,m-j,n-k]     */
            /*                           - Sum_ki =1_to_nl ( Comb(n,k) * Comb(l,i) * d[i,0,k] * RD[l-i,m,n-k]     */
            /* where Comb(l,i)  = NL_PascalTri[l][i]                                                              */
            /*       d[l][m][n] = (l,m,n-th) derivative of denominator                                            */
            /*      RD[l][m][n] = (l,m,n-th) derivative of rational function                                      */

            for ( i = 0; i <= p; i++ )
            {
                for ( j = 0; j <= q; j++ )
                {
                    for ( k = 0; k <= r; k++ )
                    {
                        ww = Pw[s1 + i][s2 + j][s3 + k].w;

                        for ( l = 0; l <= udr; l++ )
                        {
                            for ( m = 0; m <= udr - l; m++ )
                            {
                                for ( n = 0; n <= udr - l - m; n++ )
                                {
                                    v1 = ww * DU[l][i] * DV[m][j] * DW[n][k];

                                    for ( t1 = 1; t1 <= l; t1++ )
                                    {
                                        v1 -= NL_PascalTri[l][t1] * d[t1][0][0] * RD[l - t1][m][n][i][j][k];
                                        v2 = 0.0;

                                        for ( t2 = 1; t2 <= m; t2++ )
                                        {
                                            v2 += NL_PascalTri[m][t2] * d[t1][t2][0] * RD[l - t1][m - t2][n][i][j][k];

                                            v3 = 0.0;

                                            for ( t3 = 1; t3 <= n; t3++ )
                                            {
                                                v3 += NL_PascalTri[n][t3] * d[t1][t2][t3] * RD[l - t1][m - t2][n - t3][i][j][k];
                                            }
                                            v2 += NL_PascalTri[m][t2] * v3;
                                        }
                                        v1 -= NL_PascalTri[l][t1] * v2;
                                    }

                                    for ( t2 = 1; t2 <= m; t2++ )
                                    {
                                        v1 -= NL_PascalTri[m][t2] * d[0][t2][0] * RD[l][m - t2][n][i][j][k];
                                        v2 = 0.0;

                                        for ( t3 = 1; t3 <= n; t3++ )
                                        {
                                            v2 += NL_PascalTri[n][t3] * d[0][t2][t3] * RD[l][m - t2][n - t3][i][j][k];
                                        }
                                        v1 -= NL_PascalTri[l][t1] * v2;
                                    }

                                    for ( t3 = 1; t3 <= n; t3++ )
                                    {
                                        v1 -= NL_PascalTri[n][t3] * d[0][0][t3] * RD[l][m][n - t3][i][j][k];
                                        v2 = 0.0;

                                        for ( t1 = 1; t1 <= l; t1++ )
                                        {
                                            v2 += NL_PascalTri[l][t1] * d[t1][0][t3] * RD[l - t1][m][n - t2][i][j][k];
                                        }
                                        v1 -= NL_PascalTri[n][t3] * v2;
                                    }

                                    RD[l][m][n][i][j][k] = v1 / d[0][0][0];
                                }
                            }
                        }
                    }
                }
            }

            /*            for( i=0; i<=p; i++ )                                         */
            /*            {                                                             */
            /*              for( j=0; j<=q; j++ )                                       */
            /*              {                                                           */
            /*                ww = Pw[s1+i][s2+j].ww;                                   */
            /*                for( l=0; l<=udr; l++ )                                   */
            /*                {                                                         */
            /*                  for( m=0; m<=udr-l; m++ )                               */
            /*                  {                                                       */
            /*                    v1 = ww*DU[l][i]*DV[m][j];                            */
            /*                    for( t2=1; t2<=m; t2++ )                              */
            /*                    {                                                     */
            /*                      v1 -= NL_PascalTri[m][t2]*d[0][t2]*RD[l][m-t2][i][j];  */
            /*                    }                                                     */
            /*                    for( t1=1; t1<=l; t1++ )                              */
            /*                    {                                                     */
            /*                      v1 -= NL_PascalTri[l][t1]*d[t1][0]*RD[l-t1][m][i][j];  */
            /*                      v2 = 0.0;                                           */
            /*                      for( t2=1; t2<=m; t2++ )                            */
            /*                      {                                                   */
            /*                        v2 += NL_PascalTri[m][t2]*d[t1][t2]*RD[l-t1][m-t2][i][j];  */
            /*                      }                                                   */
            /*                      v1 -= NL_PascalTri[l][t1]*v2;                          */
            /*                    }                                                     */
            /*                    RD[l][m][i][j] = v1/d[0][0];                          */
            /*                  }                                                       */
            /*                }                                                         */
            /*              }                                                           */
            /*            }                                                             */
            break;

        case 2: /* Compute full derivative matrix                                                                     */
            /* RD[l,m,n] = 1/d[0,0,0] * (  Sum_ijk=1_to_lmn( Comb(l,i) * Comb(m,j) * Comb(n,k) * RD[l-i,m-j,n-k]  */
            /*                           - Sum_i  =1_to_l  ( Comb(l,i) * d[i,0,0] * RD[l-i,m,n]                   */
            /*                           - Sum_j  =1_to_m  ( Comb(m,j) * d[0,j,0] * RD[l,m-j,n]                   */
            /*                           - Sum_k  =1_to_n  ( Comb(n,k) * d[0,0,k] * RD[l,m,n-k]                   */
            /*                           - Sum_ij =1_to_lm ( Comb(l,i) * Comb(m,j) * d[i,j,0] * RD[l-i,m-j,n]     */
            /*                           - Sum_jk =1_to_mn ( Comb(m,j) * Comb(n,k) * d[0,j,k] * RD[l,m-j,n-k]     */
            /*                           - Sum_ki =1_to_nl ( Comb(n,k) * Comb(l,i) * d[i,0,k] * RD[l-i,m,n-k]     */
            /* where Comb(l,i)  = NL_PascalTri[l][i]                                                                 */
            /*       d[l][m][n] = (l,m,n-th) derivative of denominator                                            */
            /*      RD[l][m][n] = (l,m,n-th) derivative of rational function                                      */
            for ( i = 0; i <= p; i++ )
            {
                for ( j = 0; j <= q; j++ )
                {
                    for ( k = 0; k <= r; k++ )
                    {
                        ww = Pw[s1 + i][s2 + j][s3 + k].w;

                        for ( l = 0; l <= udr; l++ )
                        {
                            for ( m = 0; m <= vdr; m++ )
                            {
                                for ( n = 0; n <= wdr; n++ )
                                {
                                    v1 = ww * DU[l][i] * DV[m][j] * DW[n][k];

                                    for ( t1 = 1; t1 <= l; t1++ )
                                    {
                                        v1 -= NL_PascalTri[l][t1] * d[t1][0][0] * RD[l - t1][m][n][i][j][k];
                                        v2 = 0.0;

                                        for ( t2 = 1; t2 <= m; t2++ )
                                        {
                                            v2 += NL_PascalTri[m][t2] * d[t1][t2][0] * RD[l - t1][m - t2][n][i][j][k];

                                            v3 = 0.0;

                                            for ( t3 = 1; t3 <= n; t3++ )
                                            {
                                                v3 += NL_PascalTri[n][t3] * d[t1][t2][t3] * RD[l - t1][m - t2][n - t3][i][j][k];
                                            }
                                            v2 += NL_PascalTri[m][t2] * v3;
                                        }
                                        v1 -= NL_PascalTri[l][t1] * v2;
                                    }

                                    for ( t2 = 1; t2 <= m; t2++ )
                                    {
                                        v1 -= NL_PascalTri[m][t2] * d[0][t2][0] * RD[l][m - t2][n][i][j][k];
                                        v2 = 0.0;

                                        for ( t3 = 1; t3 <= n; t3++ )
                                        {
                                            v2 += NL_PascalTri[n][t3] * d[0][t2][t3] * RD[l][m - t2][n - t3][i][j][k];
                                        }
                                        v1 -= NL_PascalTri[l][t1] * v2;
                                    }

                                    for ( t3 = 1; t3 <= m; t3++ )
                                    {
                                        v1 -= NL_PascalTri[n][t3] * d[0][0][t3] * RD[l][m][n - t3][i][j][k];
                                        v2 = 0.0;

                                        for ( t1 = 1; t1 <= l; t1++ )
                                        {
                                            v2 += NL_PascalTri[l][t1] * d[t1][0][t3] * RD[l - t1][m][n - t2][i][j][k];
                                        }
                                        v1 -= NL_PascalTri[l][t1] * v2;
                                    }

                                    RD[l][m][n][i][j][k] = v1 / d[0][0][0];
                                }
                            }
                        }
                    }
                }
            }

            /*            for( i=0; i<=p; i++ )                                            */
            /*            {                                                                */
            /*              for( j=0; j<=q; j++ )                                          */
            /*              {                                                              */
            /*                ww = Pw[s1+i][s2+j].ww;                                      */
            /*                for( l=0; l<=udr; l++ )                                      */
            /*                {                                                            */
            /*                  for( m=0; m<=vdr; m++ )                                    */
            /*                  {                                                          */
            /*                    v1 = ww*DU[l][i]*DV[m][j];                               */
            /*                    for( t2=1; t2<=m; t2++ )                                 */
            /*                    {                                                        */
            /*                      v1 -= NL_PascalTri[m][t2]*d[0][t2]*RD[l][m-t2][i][j];     */
            /*                    }                                                        */
            /*                    for( t1=1; t1<=l; t1++ )                                 */
            /*                    {                                                        */
            /*                      v1 -= NL_PascalTri[l][t1]*d[t1][0]*RD[l-t1][m][i][j];     */
            /*                      v2 = 0.0;                                              */
            /*                      for( t2=1; t2<=m; t2++ )                               */
            /*                      {                                                      */
            /*                        v2 += NL_PascalTri[m][t2]*d[t1][t2]*RD[l-t1][m-t2][i][j];*/
            /*                      }                                                      */
            /*                      v1 -= NL_PascalTri[l][t1]*v2;                             */
            /*                    }                                                        */
            /*                    RD[l][m][i][j] = v1/d[0][0];                             */
            /*                  }                                                          */
            /*                }                                                            */
            /*              }                                                              */
            /*            }                                                                */
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    } /* end full/upper matrix switch */

    *usp = ku;
    *vsp = kv;
    *wsp = kw;

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_VolumeRatBasisDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  evaluates  a  trivariate rational  basis function 
     and its  derivatives at a  given parameter value. It is assumed 
     that the knot vectors are clamped, t1.e., end knots are repeated 
     with multiplicity = degree + 1. A typical calling example is:

       NL_VOLUME    vol;
       NL_INDEX      i, j, k, udr, vdr, wdr;
       NL_PARAMETER  u, v, w;
       NL_REAL       ***RD;
       ...
       (define vol, get i, j, k, udr, vdr, wdr, u, v, w, and  allocate memory 
        for RD);
       ...
       N_VolumeRatBasisIDerivs(&vol,i,j,k,u,v,w,NL_LEFT,NL_RIGHT,NL_LEFT,NL_TRUE,udr,vdr,wdr,RD);


   ACCESS:
   
     vol         , input  ,  NURBS volface
     i,j,k       , input  ,  Indices of rational basis function
     u,v,w       , input  ,  Parameter values
     ufl,vfl,wfl , input  ,  Flags:
                               NL_LEFT : t is in [t[j],t[j+1])
                                      (NL_RIGHT NL_DERIVATIVE REQUIRED)
                               NL_RIGHT: t is in (t[j],t[j+1]]
                                      (NL_LEFT NL_DERIVATIVE REQUIRED)
                               (t is either u or v)
     mfl         , input  ,  Flag: 
                               NL_TRUE AND udr == vdr == wdr: compute upper  
                                      half only of the derivative matrix
                               ELSE: compute full derivative matrix
     udr,vdr,wdr , input  ,  Highest derivatives required
     RD          , output ,  Derivatives computed  at u; RD[l][m][n] is the 
                             l-th derivative in u-direction, the m-th
                             derivative  in  v-direction, and the n-th
                             derivative  in  w-direction. MEMORY FOR RD, 
                             MUST BE ALLOCATED IN THE CALLING ROUTINE TO   
                             HOLD UP TO RD[udr][vdr][wdr].


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VolumeRatBasisIDerivs( NL_VOLUME *vol, NL_INDEX i, NL_INDEX j, NL_INDEX k, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, NL_FLAG mfl, NL_INDEX udr, NL_INDEX vdr, NL_INDEX wdr, NL_REAL *** RD )
{
    NL_PRIVATE NL_STRING rname = _T("N_VolumeRatBasisIDerivs");

    NL_FLAG jump, error = NL_NO;

    NL_INDEX im, in, io, t1, t2, t3, l, m, n;

    NL_REAL *** d, *** ww, *DU, *DV, *DW, v1, v2, v3;

    NL_DEGREE p, q, r;

    NL_KNOTVECTOR *knu, *knv, *knw;

    NL_VFUN vfn;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_VolumeGetArraySizes( vol, &im, &in, &io, &t1, &t2, &t3 );
    N_VolumeGetDegrees( vol, &p, &q, &r );
    N_VolumeGetKnotVectors( vol, &knu, &knv, &knw );

    /* Check parameter and index */

    error = N_KnotVectorIsParamOutOfBounds( knu, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knv, v, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knw, w, rname );

    if( error EQ NL_YES )
        NL_OUT;

    if( i LT 0 OR i GT im OR j LT 0 OR j GT in OR k LT 0 OR k GT io )
        NL_ERROR( NL_IND_ERR );

    /* Extract denominator */

    N_VFuncInitArrays( &vfn );
    error = N_VolumeGetDenominatorFunc( vol, &vfn, &S );

    if( error EQ NL_YES )
        NL_OUT;

    N_VFuncGetKnots( &vfn, &ww, &DU, &DV, &DW );

    /* Get derivatives of numerator */

    DU = N_AllocReal1dArray( udr, &S );

    if( DU EQ NULL )
        NL_QUIT;

    DV = N_AllocReal1dArray( vdr, &S );

    if( DV EQ NULL )
        NL_QUIT;

    DW = N_AllocReal1dArray( wdr, &S );

    if( DW EQ NULL )
        NL_QUIT;

    error = N_BasisIDerivs( knu, i, p, u, ufl, udr, DU );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisIDerivs( knv, j, q, v, vfl, vdr, DV );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisIDerivs( knw, k, r, w, wfl, wdr, DW );

    if( error EQ NL_YES )
        NL_OUT;

    /* Get derivatives of denominator */

    d = N_AllocReal3dArray( udr, vdr, wdr, &S );

    if( d EQ NULL )
        NL_QUIT;

    error = N_VFuncDerivs( &vfn, u, v, w, ufl, vfl, wfl, udr, vdr, wdr, d );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute derivatives of rational basis */

    if( mfl EQ NL_TRUE AND udr EQ vdr AND udr EQ wdr )
        jump = 1;
    else
        jump = 2;

    switch( jump )
    {
        case 1: /* Compute upper half of derivative matrix */
            /* RD[l,m,n] = 1/d[0,0,0] * (  Sum_ijk=1_to_lmn( Comb(l,i) * Comb(m,j) * Comb(n,k) * RD[l-i,m-j,n-k]  */
            /*                           - Sum_i  =1_to_l  ( Comb(l,i) * d[i,0,0] * RD[l-i,m,n]                   */
            /*                           - Sum_j  =1_to_m  ( Comb(m,j) * d[0,j,0] * RD[l,m-j,n]                   */
            /*                           - Sum_k  =1_to_n  ( Comb(n,k) * d[0,0,k] * RD[l,m,n-k]                   */
            /*                           - Sum_ij =1_to_lm ( Comb(l,i) * Comb(m,j) * d[i,j,0] * RD[l-i,m-j,n]     */
            /*                           - Sum_jk =1_to_mn ( Comb(m,j) * Comb(n,k) * d[0,j,k] * RD[l,m-j,n-k]     */
            /*                           - Sum_ki =1_to_nl ( Comb(n,k) * Comb(l,i) * d[i,0,k] * RD[l-i,m,n-k]     */
            /* where Comb(l,i)  = NL_PascalTri[l][i]                                                                 */
            /*       d[l][m][n] = (l,m,n-th) derivative of denominator                                            */
            /*      RD[l][m][n] = (l,m,n-th) derivative of rational function                                      */

            for ( l = 0; l <= udr; l++ )
            {
                for ( m = 0; m <= udr - l; m++ )
                {
                    for ( n = 0; n <= udr - l - m; n++ )
                    {
                        v1 = ww[i][j][k] * DU[l] * DV[m] * DW[n];

                        for ( t1 = 1; t1 <= l; t1++ )
                        {
                            v1 -= NL_PascalTri[l][t1] * d[t1][0][0] * RD[l - t1][m][n];
                            v2 = 0.0;

                            for ( t2 = 1; t2 <= m; t2++ )
                            {
                                v2 += NL_PascalTri[m][t2] * d[t1][t2][0] * RD[l - t1][m - t2][n];

                                v3 = 0.0;

                                for ( t3 = 1; t3 <= n; t3++ )
                                {
                                    v3 += NL_PascalTri[n][t3] * d[t1][t2][t3] * RD[l - t1][m - t2][n - t3];
                                }
                                v2 += NL_PascalTri[m][t2] * v3;
                            }
                            v1 -= NL_PascalTri[l][t1] * v2;
                        }

                        for ( t2 = 1; t2 <= m; t2++ )
                        {
                            v1 -= NL_PascalTri[m][t2] * d[0][t2][0] * RD[l][m - t2][n];
                            v2 = 0.0;

                            for ( t3 = 1; t3 <= n; t3++ )
                            {
                                v2 += NL_PascalTri[n][t3] * d[0][t2][t3] * RD[l][m - t2][n - t3];
                            }
                            v1 -= NL_PascalTri[l][t1] * v2;
                        }

                        for ( t3 = 1; t3 <= m; t3++ )
                        {
                            v1 -= NL_PascalTri[n][t3] * d[0][0][t3] * RD[l][m][n - t3];
                            v2 = 0.0;

                            for ( t1 = 1; t1 <= l; t1++ )
                            {
                                v2 += NL_PascalTri[l][t1] * d[t1][0][t3] * RD[l - t1][m][n - t2];
                            }
                            v1 -= NL_PascalTri[l][t1] * v2;
                        }

                        RD[l][m][n] = v1 / d[0][0][0];
                    }
                }
            }

            break;

        case 2: /* Compute full derivative matrix                                                                     */
            /* RD[l,m,n] = 1/d[0,0,0] * (  Sum_ijk=1_to_lmn( Comb(l,i) * Comb(m,j) * Comb(n,k) * RD[l-i,m-j,n-k]  */
            /*                           - Sum_i  =1_to_l  ( Comb(l,i) * d[i,0,0] * RD[l-i,m,n]                   */
            /*                           - Sum_j  =1_to_m  ( Comb(m,j) * d[0,j,0] * RD[l,m-j,n]                   */
            /*                           - Sum_k  =1_to_n  ( Comb(n,k) * d[0,0,k] * RD[l,m,n-k]                   */
            /*                           - Sum_ij =1_to_lm ( Comb(l,i) * Comb(m,j) * d[i,j,0] * RD[l-i,m-j,n]     */
            /*                           - Sum_jk =1_to_mn ( Comb(m,j) * Comb(n,k) * d[0,j,k] * RD[l,m-j,n-k]     */
            /*                           - Sum_ki =1_to_nl ( Comb(n,k) * Comb(l,i) * d[i,0,k] * RD[l-i,m,n-k]     */
            /* where Comb(l,i)  = NL_PascalTri[l][i]                                                                 */
            /*       d[l][m][n] = (l,m,n-th) derivative of denominator                                            */
            /*      RD[l][m][n] = (l,m,n-th) derivative of rational function                                      */
            for ( l = 0; l <= udr; l++ )
            {
                for ( m = 0; m <= vdr; m++ )
                {
                    for ( n = 0; n <= wdr; n++ )
                    {
                        v1 = ww[i][j][k] * DU[l] * DV[m] * DW[n];

                        for ( t1 = 1; t1 <= l; t1++ )
                        {
                            v1 -= NL_PascalTri[l][t1] * d[t1][0][0] * RD[l - t1][m][n];
                            v2 = 0.0;

                            for ( t2 = 1; t2 <= m; t2++ )
                            {
                                v2 += NL_PascalTri[m][t2] * d[t1][t2][0] * RD[l - t1][m - t2][n];

                                v3 = 0.0;

                                for ( t3 = 1; t3 <= n; t3++ )
                                {
                                    v3 += NL_PascalTri[n][t3] * d[t1][t2][t3] * RD[l - t1][m - t2][n - t3];
                                }
                                v2 += NL_PascalTri[m][t2] * v3;
                            }
                            v1 -= NL_PascalTri[l][t1] * v2;
                        }

                        for ( t2 = 1; t2 <= m; t2++ )
                        {
                            v1 -= NL_PascalTri[m][t2] * d[0][t2][0] * RD[l][m - t2][n];
                            v2 = 0.0;

                            for ( t3 = 1; t3 <= n; t3++ )
                            {
                                v2 += NL_PascalTri[n][t3] * d[0][t2][t3] * RD[l][m - t2][n - t3];
                            }
                            v1 -= NL_PascalTri[l][t1] * v2;
                        }

                        for ( t3 = 1; t3 <= m; t3++ )
                        {
                            v1 -= NL_PascalTri[n][t3] * d[0][0][t3] * RD[l][m][n - t3];
                            v2 = 0.0;

                            for ( t1 = 1; t1 <= l; t1++ )
                            {
                                v2 += NL_PascalTri[l][t1] * d[t1][0][t3] * RD[l - t1][m][n - t2];
                            }
                            v1 -= NL_PascalTri[l][t1] * v2;
                        }

                        RD[l][m][n] = v1 / d[0][0][0];
                    }
                }
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    } /* end full/upper matrix switch */

    /*  for( l=0; l<=udr; l++ )                                      */
    /*  {                                                            */
    /*    for( m=0; m<=vdr; m++ )                                    */
    /*    {                                                          */
    /*      v1 = w[i][j]*DU[l]*DV[m];                                */
    /*      for( t2=1; t2<=m; t2++ )                                 */
    /*      {                                                        */
    /*        v1 -= NL_PascalTri[m][t2]*d[0][t2]*RD[l][m-t2];           */
    /*      }                                                        */
    /*      for( t1=1; t1<=l; t1++ )                                 */
    /*      {                                                        */
    /*        v1 -= NL_PascalTri[l][t1]*d[t1][0]*RD[l-t1][m];           */
    /*        v2 = 0.0;                                              */
    /*        for( t2=1; t2<=m; t2++ )                               */
    /*        {                                                      */
    /*          v2 += NL_PascalTri[m][t2]*d[t1][t2]*RD[l-t1][m-t2];     */
    /*        }                                                      */
    /*        v1 -= NL_PascalTri[l][t1]*v2;                             */
    /*      }                                                        */
    /*      RD[l][m] = v1/d[0][0];                                   */
    /*    }                                                          */
    /*  }                                                            */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_VolumeRatBasisIDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  evaluates a  trivariate rational  basis  function 
     for a set of span indices and a parameter value.  It is assumed 
     that the knot vectors are clamped, i.e., end knots are repeated 
     with multiplicity = degree + 1. A typical calling example is:

       NL_VOLUME     vol;
       NL_INDEX      i, j, k;
       NL_PARAMETER  u, v, w;
       NL_REAL       R;
       ...
       (define vol, get i, j, k, u, v, and w);
       ...
       N_VolumeRatBasisIEval(&vol,i,j,k,u,v,w,NL_LEFT,NL_RIGHT,NL_LEFT,&R);


   ACCESS:
   
     vol         , input  ,  NURBS volume
     i,j,k       , input  ,  Indices  of rational basis  function 
                             (0<=i<=m), (0<=j<=n), (0<=k<=m)
     u,v,w       , input  ,  Parameter values 
     ufl,vfl,wfl , input  ,  Flags:
                               NL_LEFT : t is in [t[j],t[j+1])
                               NL_RIGHT: t is in (t[j],t[j+1]]
                               (t is either u or v)
     R           , output ,  Basis function computed at (u,v)


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_VolumeRatBasisIEval( NL_VOLUME *vol, NL_INDEX i, NL_INDEX j, NL_INDEX k, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, NL_REAL *R )
{
    NL_PRIVATE NL_STRING rname = _T("N_VolumeRatBasisIEval");

    NL_FLAG error = NL_NO;

    NL_INDEX m, n, o, s;

    NL_REAL *** ww, *T, den, NU, NV, NW;

    NL_DEGREE p, q, r;

    NL_KNOTVECTOR *knu, *knv, *knw;

    NL_VFUN vfn;

    NL_STACKS S;

    /* Start NURBS */

    N_InitNurbs( &S );

    /* Get local notation */

    N_VolumeGetArraySizes( vol, &m, &n, &o, &s, &s, &s );
    N_VolumeGetDegrees( vol, &p, &q, &r );
    N_VolumeGetKnotVectors( vol, &knu, &knv, &knw );

    /* Check parameters and indexes */

    error = N_KnotVectorIsParamOutOfBounds( knu, u, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knv, v, rname );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_KnotVectorIsParamOutOfBounds( knw, w, rname );

    if( error EQ NL_YES )
        NL_OUT;

    if( i LT 0 OR i GT m OR j LT 0 OR j GT n OR k LT 0 OR k GT o )
        NL_ERROR( NL_IND_ERR );

    /* Extract denominator */

    N_VFuncInitArrays( &vfn );
    error = N_VolumeGetDenominatorFunc( vol, &vfn, &S );

    if( error EQ NL_YES )
        NL_OUT;

    N_VFuncGetKnots( &vfn, &ww, &T, &T, &T );

    /* Compute the i-th and j-th B-splines */

    error = N_BasisIEval( knu, i, p, u, ufl, &NU );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisIEval( knv, j, q, v, vfl, &NV );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisIEval( knw, k, r, w, wfl, &NW );

    if( error EQ NL_YES )
        NL_OUT;

    /* Evaluate the denominator */

    error = N_VFuncEval( &vfn, u, v, w, ufl, vfl, wfl, &den );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute rational basis */

    *R = (ww[i][j][k] * NU * NV * NW) / den;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &S );

    return (error);
} /* end N_VolumeRatBasisIEval */

/*******************************************************************//**


   DESCRIPTION:

     This routine evaluates all non-vanishing  trivariate non-rational  
     basis functions  and  their  derivatives  at  a  given parameter 
     value. It is  assumed that the knot vectors  are  clamped, i.e., 
     end knots are repeated with multiplicity = degree + 1. A typical
     calling example is:

       NL_VOLUME     vol;
       NL_PARAMETER  u, v, w;
       NL_INDEX      udr, vdr, wdr, usp, vsp, wsp;
       NL_REAL       ****BDer;
       ...
       (define vol, get u, v, w, udr, vdr, wdr, and allocate memory for BDer);
       ...
       N_VolumeNonRatBasisDerivs(&vol,u,v,w,NL_LEFT,NL_RIGHT,NL_LEFT,NL_TRUE,udr,vdr,wdr,BDer,&usp,&vsp,&wsp);   
 

   ACCESS:
   
     vol         , input  ,  NURBS volume
     u,v,w       , input  ,  Parameter values
     ufl,vfl,wfl , input  ,  Flags:
                                    NL_LEFT : t is in [t[j],t[j+1])
                                           (NL_RIGHT NL_DERIVATIVE REQUIRED)
                                    NL_RIGHT: t is in (t[j],t[j+1]]
                                           (NL_LEFT NL_DERIVATIVE REQUIRED)
                                    (t is either u or v)
     mfl         , input  ,  Flag: 
                                  NL_TRUE AND udr == vdr == wdr: compute upper  
                                         half only of the derivative matrix
                                  ELSE: compute full derivative matrix
     udr,vdr,wdr , input  ,  Highest derivatives required
     BDer        , output ,  Derivatives computed at u,v,w; BDer[l][m][n][i][j][k] 
                             is the  (l,m,n)-th  derivative  of  the  basis 
                             function NU[ku-p+i]*NV[kv-q+j]*NW[kw-r+k], 
                             where  u is in  {u[ku],u[ku+1]}, 
                                      NL_PRIVATE  NL_STRING  rname = "N_VolumeNonRatBasisDerivs");
                                    v is in  {v[kv],v[kv+1]},
                                    w is in  {w[kw],w[kw+1]}.
                             MEMORY  FOR  BDer  MUST  BE  ALLOCATED  IN THE 
                             CALLING  ROUTINE TO  HOLD DERIVATIVES  UP TO 
                             BDer[udr][vdr][wdr][p][q][r].
     usp,vsp,wsp , output ,  Indices of knot spans containing u, v, and w.


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
NL_FLAG N_VolumeNonRatBasisDerivs( NL_VOLUME *vol, NL_PARAMETER u, NL_PARAMETER v, NL_PARAMETER w, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, NL_FLAG mfl, NL_INDEX udr, NL_INDEX vdr, NL_INDEX wdr, NL_REAL ****** BDer, NL_INDEX *usp, NL_INDEX *vsp, NL_INDEX *wsp )
{

    NL_PRIVATE NL_STRING rname = _T("N_VolumeNonRatBasisDerivs");

    NL_FLAG jump, error = NL_NO;

    NL_INDEX i, j, k, l, m, n, rr, s1, s2, rk, pk, j1, j2, mder;

    /* NL_REAL DT[NL_MAXDER + 1][NL_MAXDEG + 1];  unused */
    NL_REAL DW[NL_MAXDER + 1][NL_MAXDEG + 1];
    NL_REAL DV[NL_MAXDER + 1][NL_MAXDEG + 1];
    NL_REAL DU[NL_MAXDER + 1][NL_MAXDEG + 1];

    NL_REAL *left, *right, ndu[NL_MAXDEG + 1][NL_MAXDEG + 1], saved, temp, d, a[2][NL_MAXDEG + 1], *U, *V, *W;

    NL_DEGREE p, q, r;

    NL_KNOTVECTOR *knu, *knv, *knw;

    /* Assign local pointers */

    left = &a[0][0];
    right = &a[1][0];

    /* Get local notation */

    p = vol->p;
    knu = vol->knu;
    s1 = knu->m;
    U = knu->U;
    q = vol->q;
    knv = vol->knv;
    s2 = knv->m;
    V = knv->U;
    r = vol->r;
    knw = vol->knw;
    W = knw->U;

    /* Check parameters and order of derivatives */

    /* following the example of N_SrfNonRatBasisDerivs, we ignore this to allow for evaluation outside domain range
    if( u LT U[0] OR u GT U[s1] OR v LT V[0] OR v GT V[s2] OR w LT W[0] OR w GT W[knw->m] )
    {
        N_ErrSet( NL_PAR_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }
    */

    if( udr GT NL_MAXDER OR vdr GT NL_MAXDER OR wdr GT NL_MAXDER )
    {
        N_ErrSet( NL_MXD_ERR, rname );
        error = NL_YES;
        NL_OUT;
    }

    /* Compute all non-vanishing u-direction basis functions */

    error = N_BasisFindSpan( knu, p, u, ufl, &i );

    if( error EQ NL_YES )
        NL_OUT;
    *usp = i;

    /* Get u-direction maximum derivative index and set zero derivatives */

    mder = NL_MIN( p, udr );

    for ( l = p + 1; l <= udr; l++ )
    {
        for ( j = 0; j <= p; j++ )
        {
            DU[l][j] = 0.0;
        }
    }

    /* Compute the u-direction basis functions */

    ndu[0][0] = 1.0;

    for ( j = 1; j <= p; j++ )
    {
        left[j] = u - U[i + 1 - j];
        right[j] = U[i + j] - u;
        saved = 0.0;

        for ( rr = 0; rr < j; rr++ )
        {
            ndu[j][rr] = right[rr + 1] + left[j - rr];
            temp = ndu[rr][j - 1] / ndu[j][rr];
            ndu[rr][j] = saved + right[rr + 1] * temp;
            saved = left[j - rr] * temp;
        }
        ndu[j][j] = saved;
    }

    /* Load the u-direction basis functions */

    for ( j = 0; j <= p; j++ )
        DU[0][j] = ndu[j][p];

    /* Compute u-direction derivatives */

    for ( rr = 0; rr <= p; rr++ )
    {
        s1 = 0;
        s2 = 1;
        a[0][0] = 1.0;

        for ( l = 1; l <= mder; l++ )
        {
            d = 0.0;
            rk = rr - l;
            pk = p - l;

            if( rr GE l )
            {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }

            if( rk GE - 1 )
                j1 = 1;
            else
                j1 = -rk;

            if( (rr - 1)LE pk )
                j2 = l - 1;
            else
                j2 = p - rr;

            for ( j = j1; j <= j2; j++ )
            {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                d += a[s2][j] * ndu[rk + j][pk];
            }

            if( rr LE pk )
            {
                a[s2][l] = -a[s1][l - 1] / ndu[pk + 1][rr];
                d += a[s2][l] * ndu[rr][pk];
            }
            DU[l][rr] = d;
            N_SwapIntegers( &s1, &s2 );
        }
    }

    /* Multiply through by the correct factors for the u-direction */

    rr = p;

    for ( l = 1; l <= mder; l++ )
    {
        for ( j = 0; j <= p; j++ )
        {
            DU[l][j] *= rr;
        }
        rr *= (p - l);
    }

    /* Do the same for the v-direction */

    error = N_BasisFindSpan( knv, q, v, vfl, &i );

    if( error EQ NL_YES )
        NL_OUT;
    *vsp = i;

    /* Get maximum v-direction derivative index and set zero derivatives */

    mder = NL_MIN( q, vdr );

    for ( l = q + 1; l <= vdr; l++ )
    {
        for ( j = 0; j <= q; j++ )
        {
            DV[l][j] = 0.0;
        }
    }

    /* Compute the v-direction basis functions */

    ndu[0][0] = 1.0;

    for ( j = 1; j <= q; j++ )
    {
        left[j] = v - V[i + 1 - j];
        right[j] = V[i + j] - v;
        saved = 0.0;

        for ( rr = 0; rr < j; rr++ )
        {
            ndu[j][rr] = right[rr + 1] + left[j - rr];
            temp = ndu[rr][j - 1] / ndu[j][rr];
            ndu[rr][j] = saved + right[rr + 1] * temp;
            saved = left[j - rr] * temp;
        }
        ndu[j][j] = saved;
    }

    /* Load the v-direction basis functions */

    for ( j = 0; j <= q; j++ )
        DV[0][j] = ndu[j][q];

    /* Compute v-direction derivatives */

    for ( rr = 0; rr <= q; rr++ )
    {
        s1 = 0;
        s2 = 1;
        a[0][0] = 1.0;

        for ( l = 1; l <= mder; l++ )
        {
            d = 0.0;
            rk = rr - l;
            pk = q - l;

            if( rr GE l )
            {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }

            if( rk GE - 1 )
                j1 = 1;
            else
                j1 = -rk;

            if( (rr - 1)LE pk )
                j2 = l - 1;
            else
                j2 = q - rr;

            for ( j = j1; j <= j2; j++ )
            {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                d += a[s2][j] * ndu[rk + j][pk];
            }

            if( rr LE pk )
            {
                a[s2][l] = -a[s1][l - 1] / ndu[pk + 1][rr];
                d += a[s2][l] * ndu[rr][pk];
            }
            DV[l][rr] = d;
            N_SwapIntegers( &s1, &s2 );
        }
    }

    /* Multiply through by the correct factors for the v-direction */

    rr = q;

    for ( l = 1; l <= mder; l++ )
    {
        for ( j = 0; j <= q; j++ )
        {
            DV[l][j] *= rr;
        }
        rr *= (q - l);
    }

    /* Do the same for the w-direction */

    error = N_BasisFindSpan( knw, r, w, wfl, &i );

    if( error EQ NL_YES )
        NL_OUT;
    *wsp = i;

    /* Get maximum w-direction derivative index and set zero derivatives */

    mder = NL_MIN( r, wdr );

    for ( l = r + 1; l <= wdr; l++ )
    {
        for ( j = 0; j <= r; j++ )
        {
            DW[l][j] = 0.0;
            /* DT[l][j] = DW[l][j]; */
        }
    }

    /* Compute the w-direction basis functions */

    ndu[0][0] = 1.0;

    for ( j = 1; j <= r; j++ )
    {
        left[j] = w - W[i + 1 - j];
        right[j] = W[i + j] - w;
        saved = 0.0;

        for ( rr = 0; rr < j; rr++ )
        {
            ndu[j][rr] = right[rr + 1] + left[j - rr];
            temp = ndu[rr][j - 1] / ndu[j][rr];
            ndu[rr][j] = saved + right[rr + 1] * temp;
            saved = left[j - rr] * temp;
        }
        ndu[j][j] = saved;
    }

    /* Load the w-direction basis functions */

    for ( j = 0; j <= r; j++ )
    {
        DW[0][j] = ndu[j][r];
        /* DT[0][j] = DW[0][j]; */
    }

    /* Compute w-direction derivatives */

    for ( rr = 0; rr <= r; rr++ )
    {
        s1 = 0;
        s2 = 1;
        a[0][0] = 1.0;

        for ( l = 1; l <= mder; l++ )
        {
            d = 0.0;
            rk = rr - l;
            pk = r - l;

            if( rr GE l )
            {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }

            if( rk GE - 1 )
                j1 = 1;
            else
                j1 = -rk;

            if( (rr - 1)LE pk )
                j2 = l - 1;
            else
                j2 = r - rr;

            for ( j = j1; j <= j2; j++ )
            {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                d += a[s2][j] * ndu[rk + j][pk];
            }

            if( rr LE pk )
            {
                a[s2][l] = -a[s1][l - 1] / ndu[pk + 1][rr];
                d += a[s2][l] * ndu[rr][pk];
            }
            DW[l][rr] = d;
            /* DT[l][rr] = DW[l][rr]; */
            N_SwapIntegers( &s1, &s2 );
        }
    }

    /* Multiply through by the correct factors for the w-direction */

    rr = r;

    for ( l = 1; l <= mder; l++ )
    {
        for ( j = 0; j <= r; j++ )
        {
            DW[l][j] *= rr;
            /* DT[l][j] = DW[l][j]; */
        }
        rr *= (r - l);
    }

    /* Compute the volume derivatives */

    if( mfl EQ NL_TRUE AND udr EQ vdr AND udr EQ wdr )
        jump = 1;
    else
        jump = 2;

    switch( jump )
    {
        case 1: /* Compute upper half of derivative matrix */
            for ( i = 0; i <= p; i++ )
            {
                for ( j = 0; j <= q; j++ )
                {
                    for ( k = 0; k <= r; k++ )
                    {
                        for ( l = 0; l <= udr; l++ )
                        {
                            for ( m = 0; m <= udr - l; m++ )
                            {
                                for ( n = 0; n <= udr - l - m; n++ )
                                {
                                  BDer[l][m][n][i][j][k] = DU[l][i] * DV[m][j] * DW[n][k];
                                }
                            }
                        }
                    }
                }
            }
            break;

        case 2: /* Compute full derivative matrix */
            for ( i = 0; i <= p; i++ )
            {
                for ( j = 0; j <= q; j++ )
                {
                    for ( k = 0; k <= r; k++ )
                    {
                        for ( l = 0; l <= udr; l++ )
                        {
                            for ( m = 0; m <= vdr; m++ )
                            {
                                for ( n = 0; n <= wdr; n++ )
                                {
                                  BDer[l][m][n][i][j][k] = DU[l][i] * DV[m][j] * DW[n][k];
                                }
                            }
                        }
                    }
                }
            }
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end N_VolumeNonRatBasisDerivs */

/*******************************************************************//**


   DESCRIPTION:

     This routine computes a grid of points on a NURBS volume given a
     set of u-, v-, and w-values. A typical calling example is:

       NL_VOLUME     vol;
       NL_PARAMETER  *u, *v, *w;
       NL_INDEX      mm, nn, oo;
       NL_POINT      ***S;
       ...
       (define vol; get u, v and w, allocate memory for S);
       ...
       N_VolumeEvalGrid(&vol,u,v,w,mm,nn,oo,NL_LEFT,NL_RIGHT,NL_RIGHT,S);


   ACCESS:
   
     vol         , input  ,  NURBS volume
     u,v,w       , input  ,  Arrays of INCREASING parameters
     mm,nn,oo    , input  ,  Highest indexes in u, v, and w
     ufl,vfl,wfl , input  ,  Flags:
                           NL_LEFT : t is in [t[j],t[j+1])
                           NL_RIGHT: t is in (t[j],t[j+1]]
                           (t is either u, v, or w)
     S       , output ,  Points  on the  volume.  S[i][j][k] is  a point 
                         computed at (u[i],v[j],w[k]). MEMORY FOR S MUST BE
                         ALLOCATED IN  THE CALLING  ROUTINE TO HOLD UP 
                         TO S[mm][nn][oo].


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VolumeEvalGrid( NL_VOLUME *vol, NL_PARAMETER *u, NL_PARAMETER *v, NL_PARAMETER *w, NL_INDEX mm, NL_INDEX nn, NL_INDEX oo, NL_FLAG ufl, NL_FLAG vfl, NL_FLAG wfl, NL_POINT *** S )
{
    NL_PRIVATE NL_STRING rname = _T("N_VolumeEvalGrid");

    NL_FLAG error = NL_NO;

    NL_INDEX *usp, *vsp, *wsp, i, j, k, mu, nv, ow, ir, is, it;

    NL_INDEX m, n, o, s1, s2, s3;

    NL_DEGREE p, q, r;

    NL_REAL ** NU, ** NV, ** NW, *U, *V, *W;

    NL_KNOTVECTOR *knu, *knv, *knw;

    NL_CPOINT *** Pw, ** Tw2, *Tw1, Sw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_VolumeGetCPtsDegreesAndKnots( vol, &mu, &nv, &ow, &Pw, &p, &q, &r, &ir, &is, &it, &U, &V, &W );
    N_VolumeGetKnotVectors( vol, &knu, &knv, &knw );

    /* Check parameters */

    if( mm LT 0 OR nn LT 0 OR oo LT 0 )
        NL_ERROR( NL_IND_ERR );

    if( u[0]LT U[0]OR u[mm]GT U[ir] )
        NL_ERROR( NL_PAR_ERR );

    if( v[0]LT V[0]OR v[nn]GT V[is] )
        NL_ERROR( NL_PAR_ERR );

    if( w[0]LT W[0]OR w[oo]GT W[it] )
        NL_ERROR( NL_PAR_ERR );

    /* Compute non-vanishing B-splines */

    NU = N_AllocReal2dArray( mm, p, &SL );

    if( NU EQ NULL )
        NL_QUIT;

    NV = N_AllocReal2dArray( nn, q, &SL );

    if( NV EQ NULL )
        NL_QUIT;

    NW = N_AllocReal2dArray( oo, r, &SL );

    if( NV EQ NULL )
        NL_QUIT;

    usp = N_AllocInt1dArray( mm, &SL );

    if( usp EQ NULL )
        NL_QUIT;

    vsp = N_AllocInt1dArray( nn, &SL );

    if( vsp EQ NULL )
        NL_QUIT;

    wsp = N_AllocInt1dArray( oo, &SL );

    if( vsp EQ NULL )
        NL_QUIT;

    error = N_BasisEvalArray( knu, p, u, mm, ufl, NU, usp );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisEvalArray( knv, q, v, nn, vfl, NV, vsp );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_BasisEvalArray( knw, r, w, oo, wfl, NW, wsp );

    if( error EQ NL_YES )
        NL_OUT;

    /* Compute grid of points on the volume             */
    /* S[m][n][o] =  Sum_ijk(  Pw[s1+i][s2+j][s3+k]     */
    /*                       * NU[m][i]                 */
    /*                       * NV[n][j]                 */
    /*                       * NW[o][k]                 */

    Tw2 = N_AllocCPt2dArray( mu, nv, &SL );
    Tw1 = N_AllocCPt1dArray( nv, &SL );

    if( Tw1 EQ NULL )
        NL_QUIT;

    if( Tw2 EQ NULL )
        NL_QUIT;

    /* for every grid point */
    for ( o = 0; o <= oo; o++ )
    {
        s3 = wsp[o] - r;

        /* build T2[i][j] = Sum_k NW[o][k] * Pw[i][j][s3+k] */
        for ( i = usp[0] - p; i <= usp[mm]; i++ )
        {
            for ( j = vsp[0] - q; j <= vsp[nn]; j++ )
            {
                N_CopyCPt( NL_CZERO, &Tw2[i][j] );

                for ( k = 0; k <= r; k++ )
                {
                    N_VectorBlendCPt( NW[o][k], Pw[i][j][s3 + k], &Tw2[i][j] );
                } /* end iter every k basis value */
            }     /* end iter every j basis value */
        }         /* end iter every i basis value */

        for ( n = 0; n <= nn; n++ )
        {
            s2 = vsp[n] - q;

            /* build T1[i] = Sum_j NV[n][j] * T2[i][j] */
            for ( i = usp[0] - p; i <= usp[mm]; i++ )
            {
                N_CopyCPt( NL_CZERO, &Tw1[i] );

                for ( j = 0; j <= q; j++ )
                {
                    N_VectorBlendCPt( NV[n][j], Tw2[i][s2 + j], &Tw1[i] );
                } /* end iter every j basis value */
            }     /* end iter every i basis value */

            for ( m = 0; m <= mm; m++ )
            {
                s1 = usp[m] - p;
                N_CopyCPt( NL_CZERO, &Sw );

                /* iter every basis value */
                for ( i = 0; i <= p; i++ )
                {
                    /* buils S[m][n][0] = Sum_i Sum_j Sum_k (  NU[m][i]*NV[n][j]*NW[o][k] */
                    /*                                       * Pw[s1+i][s2+j][s3+k])      */
                    N_VectorBlendCPt( NU[m][i], Tw1[s1 + i], &Sw );
                } /* end iter every i basis value */

                N_CPtToPtEuclid( Sw, &S[m][n][o] );
            }     /* end iter every m */
        }         /* end iter every n */
    }             /* end iter every o */

    /* End NURBS and Exit */
    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_VolumeEvalGrid */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  transforms a NURBS volume given a general 4x4 
     transformation matrix. The  input data is  destroyed, i.e. the  
     transformation is done in place. A typical calling example is:
 
       NL_VOLUME  vol;
       NL_RMATRIX  rma;
       ...
       (define vol and rma);
       ...
       N_VolumeTransform(&vol,&rma);


   ACCESS:
   
     vol , in/out ,  NURBS volume
     rma , input  ,  4x4 Matrix


   RETURN CODES:

     None

   ***********************************************************************/
/* VlmNrb.c */
NL_VOID N_VolumeTransform( NL_VOLUME *vol, NL_RMATRIX *rma )
{

    NL_INDEX i, j, k, m, n, o;

    NL_CPOINT *** Pw;

    /* Get local notation */

    N_VolumeGetCPts( vol, &m, &n, &o, &Pw );

    /* Transform control points */

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            for ( k = 0; k <= o; k++ )
            {
                N_TransformCPt( Pw[i][j][k], rma, &Pw[i][j][k] );
            }
        }
    }
} /* end N_VolumeTransform */

/*******************************************************************//**


   DESCRIPTION:

     This routine scales a NURBS volume  with respect to a point. A 
     typical calling example is:

       NL_POINT    C;
       NL_VECTOR   f;
       NL_VOLUME  vol;
       ...
       (define vol, get C and scaling vector f)
       ...
       N_VolumeScale(&vol,C,f);

     The  scaling  is  done  in-place,  i.e. the  original volume is 
     destroyed.


   ACCESS:
   
     vol , in/out ,  NURBS volume
     C   , input  ,  Center of scaling
     f   , input  ,  Vector-valued scaling factor


   RETURN CODES:

     None

   ***********************************************************************/
/* VlmNrb.c */
NL_VOID N_VolumeScale( NL_VOLUME *vol, NL_POINT C, NL_VECTOR f )
{

    NL_INDEX i, j, k, m, n, o;

    NL_CPOINT *** Pw;

    /* Get local notation */

    N_VolumeGetCPts( vol, &m, &n, &o, &Pw );

    /* Get scaled control points */

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            for ( k = 0; k <= o; k++ )
            {
                N_ScaleCPtWithPtAndVector( Pw[i][j][k], C, f, &Pw[i][j][k] );
            }
        }
    }
} /* end N_VolumeScale */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  translates a  NURBS  volume. A  typical  calling 
     example is as follows:

       NL_VECTOR   T;
       NL_VOLUME  vol;
       ...
       (define vol, get translation vector T);
       ...
       N_VolumeTranslate(&vol,T);

     The  translation is done in-place,  i.e. the original volume is 
     destroyed.


   ACCESS:
   
     vol , in/out ,  NURBS volume
     T   , input  ,  Translation vector


   RETURN CODES:

     None

   ***********************************************************************/
/* VlmNrb.c */
NL_VOID N_VolumeTranslate( NL_VOLUME *vol, NL_VECTOR T )
{

    NL_INDEX i, j, k, m, n, o;

    NL_CPOINT *** Pw;

    /* Get local notation */

    N_VolumeGetCPts( vol, &m, &n, &o, &Pw );

    /* Get translated control points */

    for ( i = 0; i <= m; i++ )
    {
        for ( j = 0; j <= n; j++ )
        {
            for ( k = 0; k <= o; k++ )
            {
                N_TranslateCPt( Pw[i][j][k], T, &Pw[i][j][k] );
            }
        }
    }
} /* end N_VolumeTranslate */

/*******************************************************************//**


   DESCRIPTION:

     This  routine  rotates a  NURBS volume  about a general axis. A 
     typical calling example is:

       NL_POINT    P;
       NL_VECTOR   V;
       NL_REAL     al;
       NL_VOLUME   vol;
       ...
       (define vol, get rotation parameters P, V and al);
       ...
       N_VolumeRotateAtPoint(&vol,P,V,al);

     The  rotation is  done  in-place,  i.e. the  original volume is 
     destroyed.


   ACCESS:
   
     vol , in/out ,  NURBS volume
     P,V , input  ,  Point and vector of rotation axis
     al  , input  ,  Rotation angle


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG N_VolumeRotateAtPoint( NL_VOLUME *vol, NL_POINT P, NL_VECTOR V, NL_REAL al )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, n, m, o;

    NL_RMATRIX rma;

    NL_CPOINT *** Pw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_VolumeGetCPts( vol, &n, &m, &o, &Pw );

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
            for ( k = 0; k <= o; k++ )
            {
                N_TransformCPt( Pw[i][j][k], &rma, &Pw[i][j][k] );
            }
        }
    }

    /* End NURBS and exit */
    EXIT:
    N_EndNurbs( &SL );

    return (error);
} /* end N_VolumeRotateAtPoint */

/*******************************************************************//**


   DESCRIPTION:

     This volume routine scales a volume's  knot  vectors  to  a  given
     rectangular volume,  [us,ue]x[vs,ve]x[ws,we].  This does not change 
     the volume  geometry. The operation is done in place. A typical 
     calling example is:

       NL_VOLUME    vol;
       NL_PARAMETER  us, ue, vs, ve, ws, we;
       ...
       (define vol, and choose us,ue,vs,ve,ws,we);
       ...

       N_VolumeReparam(&vol,us,ue,vs,ve,ws,we);


   ACCESS:
   
     vol  , in/out ,  NURBS volume whose knot vectors are to be rescaled
     us   , input  ,  Start u parameter of volume after scaling
     ue   , input  ,  End u parameter of volume after scaling
     vs   , input  ,  Start v parameter of volume after scaling
     ve   , input  ,  End v parameter of volume after scaling
     ws   , input  ,  Start w parameter of volume after scaling
     we   , input  ,  End w parameter of volumen after scaling


   RETURN CODES:

     None

   ***********************************************************************/

NL_VOID N_VolumeReparam( NL_VOLUME *vol, NL_PARAMETER us, NL_PARAMETER ue, NL_PARAMETER vs, NL_PARAMETER ve, NL_PARAMETER ws, NL_PARAMETER we )
{

    NL_MINMAXBOX R;

    NL_REAL u1, u2, v1, v2, w1, w2;

    NL_FLAG scaledirs;

    /* Make new parameter box */

    N_BBoxDefine( &R, us, ue, vs, ve, ws, we );

    /* Determine which directions need scaling and call the utility */

    N_VolumeGetParamBounds( vol, &u1, &u2, &v1, &v2, &w1, &w2 );

    scaledirs = (((us NEQ u1 OR ue NEQ u2) ? NL_UDIR : 0) + ((vs NEQ v1 OR ve NEQ v2) ? NL_VDIR : 0) + ((ws NEQ w1 OR we NEQ w2) ? NL_WDIR : 0));

    ST_VolumeReparam( vol, R, scaledirs );
} /* end N_VolumeReparam */


/*******************************************************************//**


   DESCRIPTION:

     This  tools  routine  inserts one new  knot into a  NURBS volume 
     either in u-, v-, or w-direction. The  new knot must be an interior 
     knot and the  sum of the  multiplicities of  the old and  the new 
     knots must be less than or equal to the respective degree. If the  
     output  volume  is  initialized  to  NULL, memory  to  store new 
     control points and knots is allocated. A typical  calling example
     is as follows:

       NL_VOLUME    volP, volQ;
       NL_PARAMETER  t;
       NL_INDEX      mt;
       NL_STACKS     SP, SQ;
       ...
       (define volP, get t and mt);
       ...
       N_VolumeInitArrays(&volQ);
       N_VolumeInsertKnot(&volP,t,mt,NL_UDIR,&volQ,&SP,&SQ);
       N_VolumeInsertKnot(&volP,t,mt,NL_VDIR,&volP,&SP,&SP);

     If memory is  available, volQ is not  initialized and the routine
     assumes that memory allocation has been done. However, it  checks  
     for the proper amount by looking at the highest indexes in volQ's 
     knot vector and polygon objects. If volP is the same as volQ, the
     insertion is done in place.


   ACCESS:
   
     volP , input  ,  NURBS volume
     t    , input  ,  New knot to be inserted
     mt   , input  ,  Number of times t is to be inserted
     dir  , input  ,  Flag:
                        NL_UDIR: Insert in u-direction
                        NL_VDIR: Insert in v-direction
                        NL_WDIR: Insert in w-direction
     volQ , output ,  Volume after knot inserion
     SP   , input  ,  volP's stack
     SQ   , input  ,  volQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
/* VlmTool.c */
NL_FLAG N_VolumeInsertKnot( NL_VOLUME *volP, NL_PARAMETER t, NL_INDEX mt, NL_FLAG dir, NL_VOLUME *volQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_VolumeInsertKnot");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, m, n, o, ir, is, it, spu = 0, mlu = 0, spv = 0, mlv = 0, spw = 0, mlw = 0, a, ni, mi, oi, ri, si, ti;

    NL_DEGREE p, q, r;

    NL_REAL *UP, *VP, *WP, *UQ, *VQ, *WQ, ** alf, ** oma;

    NL_KNOTVECTOR *knu, *knv, *knw;

    NL_CPOINT *** Pw, *** Qw, *Rw;

    NL_VOLUME volA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_VolumeGetCPtsDegreesAndKnots( volP, &m, &n, &o, &Pw, &p, &q, &r, &ir, &is, &it, &UP, &VP, &WP );
    N_VolumeGetKnotVectors( volP, &knu, &knv, &knw );

    /* Check parameters and compute highest indexes */

    switch( dir )
    {
        case NL_UDIR:

            error = N_KnotVectorIsEndParam( knu, t, rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisFindSpanAndMult( knu, p, t, NL_LEFT, &spu, &mlu );

            if( error EQ NL_YES )
                NL_OUT;

            if( (mlu + mt)GT p OR mt LT 0 )
                NL_ERROR( NL_PAR_ERR );

            /* set new vol sizes */
            mi = m + mt;
            ni = n;
            oi = o;
            ri = ir + mt;
            si = is;
            ti = it;
            break;

        case NL_VDIR:

            error = N_KnotVectorIsEndParam( knv, t, rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisFindSpanAndMult( knv, q, t, NL_LEFT, &spv, &mlv );

            if( error EQ NL_YES )
                NL_OUT;

            if( (mlv + mt)GT q OR mt LT 0 )
                NL_ERROR( NL_PAR_ERR );

            /* set new vol sizes */
            mi = m;
            ni = n + mt;
            oi = o;
            ri = ir;
            si = is + mt;
            ti = it;
            break;

        case NL_WDIR:

            error = N_KnotVectorIsEndParam( knw, t, rname );

            if( error EQ NL_YES )
                NL_OUT;

            error = N_BasisFindSpanAndMult( knw, r, t, NL_LEFT, &spw, &mlw );

            if( error EQ NL_YES )
                NL_OUT;

            if( (mlw + mt)GT r OR mt LT 0 )
                NL_ERROR( NL_PAR_ERR );

            /* set new vol sizes */
            mi = m;
            ni = n;
            oi = o + mt;
            ri = ir;
            si = is;
            ti = it + mt;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    if( volP EQ volQ )
    {
        volA = *volP;

        error = N_AllocVolumeArrays( volP, mi, ni, oi, p, q, r, ri, si, ti, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_VolumeGetCPtsAndKnots( volP, &Qw, &UQ, &VQ, &WQ );
    }
    else
    {
        error = N_VolumeSizeArrays( volQ, mi, ni, oi, p, q, r, ri, si, ti, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_VolumeGetCPtsAndKnots( volQ, &Qw, &UQ, &VQ, &WQ );
    }

    /* Allocate local memory */

    a = NL_MAX3( p, q, r );
    alf = N_AllocReal2dArray( a, a, &SL );

    if( alf EQ NULL )
        NL_QUIT;

    oma = N_AllocReal2dArray( a, a, &SL );

    if( oma EQ NULL )
        NL_QUIT;

    Rw = N_AllocCPt1dArray( a, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    /* Insert the knot */

    if( dir EQ NL_UDIR )
    {
        /* Save the alpha's */

        for ( i = 1; i <= mt; i++ )
        {
            a = spu - p + i;

            for ( j = 0; j <= p - i - mlu; j++ )
            {
                alf[i][j] = (t - UP[a + j]) / (UP[spu + j + 1] - UP[a + j]);
                oma[i][j] = 1.0 - alf[i][j];
            }
        }

        /* For each row do */

        for ( j = 0; j <= n; j++ )
        {
            for ( k = 0; k <= o; k++ )
            {
                a = spu - p;

                /* Load auxiliary control points */

                for ( i = 0; i <= p - mlu; i++ )
                    N_CopyCPt( Pw[a + i][j][k], &Rw[i] );

                /* Save unaltered control points */

                for ( i = 0; i <= a; i++ )
                    N_CopyCPt( Pw[i][j][k], &Qw[i][j][k] );

                for ( i = spu - mlu; i <= m; i++ )
                    N_CopyCPt( Pw[i][j][k], &Qw[i + mt][j][k] );

                /* Now insert the knot */

                for ( i = 1; i <= mt; i++ )
                {
                    a = spu - p + i;

                    for ( l = 0; l <= p - i - mlu; l++ )
                    {
                        N_Combine2CPts( alf[i][l], Rw[l + 1], oma[i][l], Rw[l], &Rw[l] );
                    }
                    N_CopyCPt( Rw[0], &Qw[a][j][k] );
                    N_CopyCPt( Rw[p - i - mlu], &Qw[spu + mt - i - mlu][j][k] );
                }

                /* Load the remaining control points */

                for ( i = a + 1; i < spu - mlu; i++ )
                    N_CopyCPt( Rw[i - a], &Qw[i][j][k] );
            }
        } /* End for each row */

        /* Load knot vectors */

        for ( i = 0; i <= spu; i++ )
            UQ[i] = UP[i];

        for ( i = 1; i <= mt; i++ )
            UQ[i + spu] = t;

        for ( i = spu + 1; i <= ir; i++ )
            UQ[i + mt] = UP[i];

        for ( j = 0; j <= is; j++ )
            VQ[j] = VP[j];

        for ( k = 0; k <= it; k++ )
            WQ[k] = WP[k];
    } /* End of NL_UDIR */

    if( dir EQ NL_VDIR )
    {
        /* Save the alpha'is */

        for ( i = 1; i <= mt; i++ )
        {
            a = spv - q + i;

            for ( j = 0; j <= q - i - mlv; j++ )
            {
                alf[i][j] = (t - VP[a + j]) / (VP[spv + j + 1] - VP[a + j]);
                oma[i][j] = 1.0 - alf[i][j];
            }
        }

        /* For each column do */

        for ( i = 0; i <= m; i++ )
        {
            for ( k = 0; k <= o; k++ )
            {
                a = spv - q;

                /* Load auxiliary control points */

                for ( j = 0; j <= q - mlv; j++ )
                    N_CopyCPt( Pw[i][a + j][k], &Rw[j] );

                /* Save unaltered control points */

                for ( j = 0; j <= a; j++ )
                    N_CopyCPt( Pw[i][j][k], &Qw[i][j][k] );

                for ( j = spv - mlv; j <= n; j++ )
                    N_CopyCPt( Pw[i][j][k], &Qw[i][j + mt][k] );

                /* Now insert the knot */

                for ( j = 1; j <= mt; j++ )
                {
                    a = spv - q + j;

                    for ( l = 0; l <= q - j - mlv; l++ )
                    {
                        N_Combine2CPts( alf[j][l], Rw[l + 1], oma[j][l], Rw[l], &Rw[l] );
                    }
                    N_CopyCPt( Rw[0], &Qw[i][a][k] );
                    N_CopyCPt( Rw[q - j - mlv], &Qw[i][spv + mt - j - mlv][k] );
                }

                /* Load the remaining control points */

                for ( j = a + 1; j < spv - mlv; j++ )
                    N_CopyCPt( Rw[j - a], &Qw[i][j][k] );
            }
        } /* End for each column */

        /* Load knot vectors */

        for ( i = 0; i <= ir; i++ )
            UQ[i] = UP[i];

        for ( j = 0; j <= spv; j++ )
            VQ[j] = VP[j];

        for ( j = 1; j <= mt; j++ )
            VQ[j + spv] = t;

        for ( j = spv + 1; j <= is; j++ )
            VQ[j + mt] = VP[j];

        for ( k = 0; k <= it; k++ )
            WQ[k] = WP[k];
    } /* End of NL_VDIR */

    if( dir EQ NL_WDIR )
    {
        /* Save the alpha's */

        for ( i = 1; i <= mt; i++ )
        {
            a = spw - r + i;

            for ( j = 0; j <= r - i - mlw; j++ )
            {
                alf[i][j] = (t - WP[a + j]) / (WP[spw + j + 1] - WP[a + j]);
                oma[i][j] = 1.0 - alf[i][j];
            }
        }

        /* For each row do */

        for ( i = 0; i <= m; i++ )
        {
            for ( j = 0; j <= n; j++ )
            {
                a = spw - r;

                /* Load auxiliary control points */

                for ( k = 0; k <= r - mlw; k++ )
                    N_CopyCPt( Pw[i][j][a + k], &Rw[k] );

                /* Save unaltered control points */

                for ( k = 0; k <= a; k++ )
                    N_CopyCPt( Pw[i][j][k], &Qw[i][j][k] );

                for ( k = spw - mlw; k <= o; k++ )
                    N_CopyCPt( Pw[i][j][k], &Qw[i][j][k + mt] );

                /* Now insert the knot */

                for ( k = 1; k <= mt; k++ )
                {
                    a = spw - r + k;

                    for ( l = 0; l <= r - k - mlw; l++ )
                    {
                        N_Combine2CPts( alf[k][l], Rw[l + 1], oma[k][l], Rw[l], &Rw[l] );
                    }
                    N_CopyCPt( Rw[0], &Qw[i][j][a] );
                    N_CopyCPt( Rw[r - k - mlw], &Qw[i][j][spw + mt - k - mlw] );
                }

                /* Load the remaining control points */

                for ( k = a + 1; k < spw - mlw; k++ )
                    N_CopyCPt( Rw[k - a], &Qw[i][j][k] );
            }
        } /* End for each row */

        /* Load knot vectors */

        for ( i = 0; i <= ir; i++ )
            UQ[i] = UP[i];

        for ( j = 0; j <= is; j++ )
            VQ[j] = VP[j];

        for ( k = 0; k <= spw; k++ )
            WQ[k] = WP[k];

        for ( k = 1; k <= mt; k++ )
            WQ[k + spw] = t;

        for ( k = spw + 1; k <= it; k++ )
            WQ[k + mt] = WP[k];
    } /* End of NL_WDIR */

    /* If insertion is in place, kill old volume */

    if( volP EQ volQ )
        N_FreeVolume( &volA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_VolumeInsertKnot */


/*******************************************************************//**


   DESCRIPTION:

     This tool's routine extracts an  iso-surface from a NURBS volume at
     a given constant u, v, or w parameter value. If the  
     output surface is initialized to NULL, memory to store surface control 
     points and knots is allocated. A typical calling example is: 

       NL_VOLUME     vol;
       NL_PARAMETER  t;
       NL_VOLUME     vol;
       NL_STACKS     SC;
       ...
       (define vol, get t);
       ...
       N_SrfInitArrays(&sur);
       N_VolumeMakeIsoSurface(&vol,t,NL_UDIR,&sur,&SC);

     If memory is  available, sur  is  not  initialized and the routine
     assumes that memory allocation has  been done. However, it  checks  
     for the proper  amount by looking  at the highest indexes in sur's 
     knot vector and polygon objects. 


   ACCESS:
   
     vol , input  ,  NURBS volume
     t   , input  ,  Parameter at which surface is to be extracted
                     when dir == NL_UDIR, t = constant u value
                                 NL_VDIR, t = constant v value
                                 NL_WDIR, t = constant w value
     dir , input  ,  Flag:
                       NL_UDIR: Extract a vw-surface (at a fixed u-value)
                       NL_VDIR: Extract a uw-surface (at a fixed v-value)
                       NL_WDIR: Extract a uv-surface (at a fixed w-value)
     sur , output ,  Extracted surface
     SC  , input  ,  sur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
/* VlmTool.c */
NL_FLAG N_VolumeMakeIsoSurface( NL_VOLUME *vol, NL_PARAMETER t, NL_FLAG dir, NL_SURFACE *sur, NL_STACKS *SC )
{
    NL_PRIVATE NL_STRING rname = _T("N_VolumeMakeIsoSurface");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, l, m, n, o, kk, ir, is, it, spu = 0, mlu = 0, spv = 0, mlv = 0, spw = 0, mlw = 0, a, ne1, me1, ne2, me2;

    NL_DEGREE p, q, r, pe1, pe2;

    NL_REAL *U, *V, *W, *SU, *SV, ** alf, ** oma;

    NL_KNOTVECTOR *knu, *knv, *knw;

    NL_CPOINT *** Pw, ** Sw, *Rw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_VolumeGetCPtsDegreesAndKnots( vol, &m, &n, &o, &Pw, &p, &q, &r, &ir, &is, &it, &U, &V, &W );
    N_VolumeGetKnotVectors( vol, &knu, &knv, &knw );

    /* Check parameters and compute highest indexes */

    switch( dir )
    {
        case NL_UDIR:

            error = N_BasisFindSpanAndMult( knu, p, t, NL_LEFT, &spu, &mlu );

            if( error EQ NL_YES )
                NL_OUT;

            ne1 = n;
            pe1 = q;
            me1 = is;
            ne2 = o;
            pe2 = r;
            me2 = it;
            break;

        case NL_VDIR:

            error = N_BasisFindSpanAndMult( knv, q, t, NL_LEFT, &spv, &mlv );

            if( error EQ NL_YES )
                NL_OUT;

            ne1 = m;
            pe1 = p;
            me1 = ir;
            ne2 = o;
            pe2 = r;
            me2 = it;
            break;

        case NL_WDIR:

            error = N_BasisFindSpanAndMult( knw, r, t, NL_LEFT, &spw, &mlw );

            if( error EQ NL_YES )
                NL_OUT;

            ne1 = m;
            pe1 = p;
            me1 = ir;
            ne2 = n;
            pe2 = q;
            me2 = is;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    error = N_SrfSizeArrays( sur, ne1, ne2, pe1, pe2, me1, me2, rname, SC );

    if( error EQ NL_YES )
        NL_OUT;

    N_SrfGetCPtsAndKnots( sur, &Sw, &SU, &SV );

    /* See if boundary curve is required */

    /* constant NL_UDIR boundary surface */
    if( dir EQ NL_UDIR AND( t EQ U[p]OR t EQ U[ir - p] ) )
    {
        if( t EQ U[p] )
        {
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= o; j++ )
                {
                    N_CopyCPt( Pw[0][i][j], &Sw[i][j] );
                }
            }
        }
        else
        {
            for ( i = 0; i <= n; i++ )
            {
                for ( j = 0; j <= o; j++ )
                {
                    N_CopyCPt( Pw[m][i][j], &Sw[i][j] );
                }
            }
        }

        for ( i = 0; i <= is; i++ )
            SU[i] = V[i];

        for ( i = 0; i <= it; i++ )
            SV[i] = W[i];

        NL_OUT;
    }

    /* constant NL_VDIR boundary surface */
    if( dir EQ NL_VDIR AND( t EQ V[q]OR t EQ V[is - q] ) )
    {
        if( t EQ V[q] )
        {
            for ( i = 0; i <= m; i++ )
            {
                for ( j = 0; j <= o; j++ )
                {
                    N_CopyCPt( Pw[i][0][j], &Sw[i][j] );
                }
            }
        }
        else
        {
            for ( i = 0; i <= m; i++ )
            {
                for ( j = 0; j <= o; j++ )
                {
                    N_CopyCPt( Pw[i][n][j], &Sw[i][j] );
                }
            }
        }

        for ( i = 0; i <= ir; i++ )
            SU[i] = U[i];

        for ( i = 0; i <= it; i++ )
            SV[i] = W[i];

        NL_OUT;
    }

    /* constant NL_WDIR boundary surface */
    if( dir EQ NL_WDIR AND( t EQ W[r]OR t EQ W[it - r] ) )
    {
        if( t EQ W[r] )
        {
            for ( i = 0; i <= m; i++ )
            {
                for ( j = 0; j <= n; j++ )
                {
                    N_CopyCPt( Pw[i][j][0], &Sw[i][j] );
                }
            }
        }
        else
        {
            for ( i = 0; i <= m; i++ )
            {
                for ( j = 0; j <= n; j++ )
                {
                    N_CopyCPt( Pw[i][j][o], &Sw[i][j] );
                }
            }
        }

        for ( i = 0; i <= ir; i++ )
            SU[i] = U[i];

        for ( i = 0; i <= is; i++ )
            SV[i] = V[i];

        NL_OUT;
    }

    /* Allocate local memory */

    a = NL_MAX3( p, q, r );
    alf = N_AllocReal2dArray( a, a, &SL );

    if( alf EQ NULL )
        NL_QUIT;

    oma = N_AllocReal2dArray( a, a, &SL );

    if( oma EQ NULL )
        NL_QUIT;

    Rw = N_AllocCPt1dArray( a, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    /* Extract the constant u-surface */
    if( dir EQ NL_UDIR )
    {
        /* Save the u-alpha's */
        for ( i = 1; i <= p - mlu; i++ )
        {
            a = spu - p + i;

            for ( j = 0; j <= p - i - mlu; j++ )
            {
                alf[i][j] = (t - U[a + j]) / (U[spu + j + 1] - U[a + j]);
                oma[i][j] = 1.0 - alf[i][j];
            }
        }

        /* For each column compute curve control point */
        a = spu - p;

        for ( l = 0; l <= n; l++ )
        {
            for ( kk = 0; kk <= o; kk++ )
            {
                /* Load auxiliary control points */

                for ( i = 0; i <= p - mlu; i++ )
                    N_CopyCPt( Pw[a + i][l][kk], &Rw[i] );

                /* Now insert the knot */

                for ( i = 1; i <= p - mlu; i++ )
                {
                    for ( j = 0; j <= p - i - mlu; j++ )
                    {
                        N_Combine2CPts( alf[i][j], Rw[j + 1], oma[i][j], Rw[j], &Rw[j] );
                    }
                }

                /* Load surface control point */

                N_CopyCPt( Rw[0], &Sw[l][kk] );
            }
        } /* End for each column */

        /* Load surface knot vectors */
        for ( i = 0; i <= is; i++ )
            SU[i] = V[i];

        for ( i = 0; i <= it; i++ )
            SV[i] = W[i];
    } /* End of NL_UDIR */

    /* Extract the constant v-surface */
    if( dir EQ NL_VDIR )
    {
        /* Save the v-alpha's */
        for ( i = 1; i <= q - mlv; i++ )
        {
            a = spv - q + i;

            for ( j = 0; j <= q - i - mlv; j++ )
            {
                alf[i][j] = (t - V[a + j]) / (V[spv + j + 1] - V[a + j]);
                oma[i][j] = 1.0 - alf[i][j];
            }
        }

        /* For each column compute curve control point */
        a = spv - q;

        for ( l = 0; l <= m; l++ )
        {
            for ( kk = 0; kk <= o; kk++ )
            {
                /* Load auxiliary control points */

                for ( i = 0; i <= q - mlv; i++ )
                    N_CopyCPt( Pw[l][a + i][kk], &Rw[i] );

                /* Now insert the knot */

                for ( i = 1; i <= q - mlv; i++ )
                {
                    for ( j = 0; j <= q - i - mlv; j++ )
                    {
                        N_Combine2CPts( alf[i][j], Rw[j + 1], oma[i][j], Rw[j], &Rw[j] );
                    }
                }

                /* Load surface control point */

                N_CopyCPt( Rw[0], &Sw[l][kk] );
            }
        } /* End for each column */

        /* Load surface knot vectors */
        for ( i = 0; i <= ir; i++ )
            SU[i] = U[i];

        for ( i = 0; i <= it; i++ )
            SV[i] = W[i];
    } /* End of NL_VDIR */

    /* Extract the constant w-surface */
    if( dir EQ NL_WDIR )
    {
        /* Save the u-alpha's */
        for ( i = 1; i <= r - mlw; i++ )
        {
            a = spw - r + i;

            for ( j = 0; j <= r - i - mlw; j++ )
            {
                alf[i][j] = (t - W[a + j]) / (W[spw + j + 1] - W[a + j]);
                oma[i][j] = 1.0 - alf[i][j];
            }
        }

        /* For each column compute curve control point */
        a = spw - r;

        for ( l = 0; l <= m; l++ )
        {
            for ( kk = 0; kk <= n; kk++ )
            {
                /* Load auxiliary control points */

                for ( i = 0; i <= r - mlw; i++ )
                    N_CopyCPt( Pw[l][kk][a + i], &Rw[i] );

                /* Now insert the knot */

                for ( i = 1; i <= r - mlw; i++ )
                {
                    for ( j = 0; j <= r - i - mlw; j++ )
                    {
                        N_Combine2CPts( alf[i][j], Rw[j + 1], oma[i][j], Rw[j], &Rw[j] );
                    }
                }

                /* Load curve control point */

                N_CopyCPt( Rw[0], &Sw[l][kk] );
            }
        } /* End for each column */

        /* Load surface knot vectors */
        for ( i = 0; i <= ir; i++ )
            SU[i] = U[i];

        for ( i = 0; i <= is; i++ )
            SV[i] = V[i];
    } /* End of NL_WDIR */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_VolumeMakeIsoSurface */

/*******************************************************************//**


   DESCRIPTION:

     This tools  routine extracts an  iso-curve from a NURBS volume at
     given  parameter values either in  u-v, u-w, or v-w directions. If the  
     output curve is initialized to NULL, memory to store curve control 
     points and knots is allocated. A typical calling example is: 

       NL_VOLUME     vol;
       NL_PARAMETER  t1, t2;
       NL_CURVE      cur;
       NL_STACKS     SC;
       ...
       (define vol, get t);
       ...
       N_CrvInitArrays(&cur);
       N_VolumeMakeIsoCurve(&vol,t1,t2,NL_UDIR,&cur,&SC);

     If memory is  available, cur  is  not  initialized and the routine
     assumes that memory allocation has  been done. However, it  checks  
     for the proper  amount by looking  at the highest indexes in cur's 
     knot vector and polygon objects. 


   ACCESS:
   
     vol , input  ,  NURBS volume
     t1  , input  ,  Parameter at which curve is to be extracted
                     when dir == NL_UDIR, t1 = constant v value
                                 NL_VDIR, t1 = constant u value
                                 NL_WDIR, t1 = constant u value
     t2  , input  ,  Parameter at which curve is to be extracted
                     when dir == NL_UDIR, t2 = constant w value
                                 NL_VDIR, t2 = constant w value
                                 NL_WDIR, t2 = constant v value
     dir , input  ,  Flag:
                       NL_UDIR: Extract a u-curve (at a fixed v,w-value)
                       NL_VDIR: Extract a v-curve (at a fixed u,w-value)
                       NL_WDIR: Extract a w-curve (at a fixed u,v-value)
     cur , output ,  Extracted curve
     SC  , input  ,  cur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
/* VlmTool.c */
NL_FLAG N_VolumeMakeIsoCurve( NL_VOLUME *vol, NL_PARAMETER t1, NL_PARAMETER t2, NL_FLAG dir, NL_CURVE *cur, NL_STACKS *SC )
{
    NL_PRIVATE NL_STRING rname = _T("N_VolumeMakeIsoCurve");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, m, n, o, kk, ir, is, it, spu = 0, mlu = 0, spv= 0, mlv = 0, spw = 0, mlw = 0, a, a1, a2, ne1, me1;

    NL_DEGREE p, q, r, pe1;

    NL_REAL *U, *V, *W, *UC, ** alf1, ** oma1, ** alf2, ** oma2;

    NL_KNOTVECTOR *knu, *knv, *knw;

    NL_CPOINT *** Pw, *Cw, *Rw, *Qw;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_VolumeGetCPtsDegreesAndKnots( vol, &m, &n, &o, &Pw, &p, &q, &r, &ir, &is, &it, &U, &V, &W );
    N_VolumeGetKnotVectors( vol, &knu, &knv, &knw );

    /* Check parameters and compute highest indexes */

    switch( dir )
    {
        case NL_UDIR:

            error = N_BasisFindSpanAndMult( knv, q, t1, NL_LEFT, &spv, &mlv );

            if( error EQ NL_YES )
                NL_OUT;
            error = N_BasisFindSpanAndMult( knw, r, t2, NL_LEFT, &spw, &mlw );

            if( error EQ NL_YES )
                NL_OUT;

            ne1 = m;
            pe1 = p;
            me1 = ir;
            break;

        case NL_VDIR:

            error = N_BasisFindSpanAndMult( knu, p, t1, NL_LEFT, &spu, &mlu );

            if( error EQ NL_YES )
                NL_OUT;
            error = N_BasisFindSpanAndMult( knw, r, t2, NL_LEFT, &spw, &mlw );

            if( error EQ NL_YES )
                NL_OUT;

            ne1 = n;
            pe1 = q;
            me1 = is;
            break;

        case NL_WDIR:

            error = N_BasisFindSpanAndMult( knu, p, t1, NL_LEFT, &spu, &mlu );

            if( error EQ NL_YES )
                NL_OUT;
            error = N_BasisFindSpanAndMult( knv, q, t2, NL_LEFT, &spv, &mlv );

            if( error EQ NL_YES )
                NL_OUT;

            ne1 = o;
            pe1 = r;
            me1 = it;
            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    error = N_CrvSizeArrays( cur, ne1, pe1, me1, rname, SC );

    if( error EQ NL_YES )
        NL_OUT;

    N_CrvGetCPtsAndKnots( cur, &Cw, &UC );

    /* See if boundary curve is required */

    if( dir EQ NL_UDIR AND( t1 EQ V[q]OR t1 EQ V[is - q] )AND( t2 EQ W[r]OR t2 EQ W[it - r] ) )
    {
        if( t1 EQ V[q]AND t2 EQ W[r] )
        {
            for ( i = 0; i <= m; i++ )
                N_CopyCPt( Pw[i][0][0], &Cw[i] );
        }
        else if( t1 EQ V[q]AND t2 EQ W[it - r] )
        {
            for ( i = 0; i <= m; i++ )
                N_CopyCPt( Pw[i][0][o], &Cw[i] );
        }
        else if( t1 EQ V[is - q]AND t2 EQ W[r] )
        {
            for ( i = 0; i <= m; i++ )
                N_CopyCPt( Pw[i][n][0], &Cw[i] );
        }
        else
        {
            for ( i = 0; i <= m; i++ )
                N_CopyCPt( Pw[i][n][o], &Cw[i] );
        }

        for ( i = 0; i <= ir; i++ )
            UC[i] = U[i];

        NL_OUT;
    }

    if( dir EQ NL_VDIR AND( t1 EQ U[p]OR t1 EQ U[ir - p] )AND( t2 EQ W[r]OR t2 EQ W[it - r] ) )
    {
        if( t1 EQ U[p]AND t2 EQ W[r] )
        {
            for ( j = 0; j <= n; j++ )
                N_CopyCPt( Pw[0][j][0], &Cw[j] );
        }
        else if( t1 EQ U[p]AND t2 EQ W[it - r] )
        {
            for ( j = 0; j <= n; j++ )
                N_CopyCPt( Pw[0][j][o], &Cw[j] );
        }
        else if( t1 EQ U[ir - p]AND t2 EQ W[r] )
        {
            for ( j = 0; j <= n; j++ )
                N_CopyCPt( Pw[m][j][0], &Cw[j] );
        }
        else
        {
            for ( j = 0; j <= n; j++ )
                N_CopyCPt( Pw[m][j][o], &Cw[j] );
        }

        for ( j = 0; j <= is; j++ )
            UC[j] = V[j];

        NL_OUT;
    }

    if( dir EQ NL_WDIR AND( t1 EQ U[p]OR t1 EQ U[ir - p] )AND( t2 EQ V[q]OR t2 EQ V[is - q] ) )
    {
        if( t1 EQ U[p]AND t2 EQ V[q] )
        {
            for ( k = 0; k <= o; k++ )
                N_CopyCPt( Pw[0][0][k], &Cw[k] );
        }
        else if( t1 EQ U[p]AND t2 EQ V[is - q] )
        {
            for ( k = 0; k <= o; k++ )
                N_CopyCPt( Pw[0][n][k], &Cw[k] );
        }
        else if( t1 EQ U[ir - p]AND t2 EQ V[q] )
        {
            for ( k = 0; k <= o; k++ )
                N_CopyCPt( Pw[m][0][k], &Cw[k] );
        }
        else
        {
            for ( k = 0; k <= o; k++ )
                N_CopyCPt( Pw[m][n][k], &Cw[k] );
        }

        for ( k = 0; k <= it; k++ )
            UC[k] = W[k];

        NL_OUT;
    }

    /* Allocate local memory */

    a = NL_MAX3( p, q, r );
    alf1 = N_AllocReal2dArray( a, a, &SL );

    if( alf1 EQ NULL )
        NL_QUIT;

    alf2 = N_AllocReal2dArray( a, a, &SL );

    if( alf2 EQ NULL )
        NL_QUIT;

    oma1 = N_AllocReal2dArray( a, a, &SL );

    if( oma1 EQ NULL )
        NL_QUIT;

    oma2 = N_AllocReal2dArray( a, a, &SL );

    if( oma2 EQ NULL )
        NL_QUIT;

    Rw = N_AllocCPt1dArray( a, &SL );

    if( Rw EQ NULL )
        NL_QUIT;

    Qw = N_AllocCPt1dArray( a, &SL );

    if( Qw EQ NULL )
        NL_QUIT;

    /* Extract the u-curve */
    if( dir EQ NL_UDIR )
    {
        /* Save the v-alpha's */
        for ( i = 1; i <= q - mlv; i++ )
        {
            a = spv - q + i;

            for ( j = 0; j <= q - i - mlv; j++ )
            {
                alf1[i][j] = (t1 - V[a + j]) / (V[spv + j + 1] - V[a + j]);
                oma1[i][j] = 1.0 - alf1[i][j];
            }
        }

        /* Save the w-alpha's */
        for ( i = 1; i <= r - mlw; i++ )
        {
            a = spw - r + i;

            for ( j = 0; j <= r - i - mlw; j++ )
            {
                alf2[i][j] = (t2 - W[a + j]) / (W[spw + j + 1] - W[a + j]);
                oma2[i][j] = 1.0 - alf2[i][j];
            }
        }

        /* For each column compute curve control point */

        a1 = spv - q;
        a2 = spw - r;

        for ( k = 0; k <= m; k++ )
        {
            /* Load auxiliary control points compressed from dir2 */
            for ( i = 0; i <= q - mlv; i++ )
            {
                for ( j = 0; j <= r - mlw; j++ )
                {
                    N_CopyCPt( Pw[k][a1 + i][a2 + j], &Qw[j] );
                }

                /* Now insert the dir2 knot */
                for ( j = 1; j <= r - mlw; j++ )
                {
                    for ( kk = 0; kk <= r - j - mlw; kk++ )
                    {
                        N_Combine2CPts( alf2[j][kk], Qw[kk + 1], oma2[j][kk], Qw[kk], &Qw[kk] );
                    }
                }

                /* Load auxiliary control point */
                N_CopyCPt( Qw[0], &Rw[i] );
            } /* end dir2 knot insertion */
      
            /* Now insert the dir1 knot */

            for ( i = 1; i <= q - mlv; i++ )
            {
                for ( j = 0; j <= q - i - mlv; j++ )
                {
                    N_Combine2CPts( alf1[i][j], Rw[j + 1], oma1[i][j], Rw[j], &Rw[j] );
                }
            }

            /* Load curve control point */
            N_CopyCPt( Rw[0], &Cw[k] );
        }

        /* Load curve knot vector */
        for ( i = 0; i <= ir; i++ )
            UC[i] = U[i];
    } /* End of NL_UDIR */

    /* Extract the v-curve */
    if( dir EQ NL_VDIR )
    {
        /* Save the u-alpha's */
        for ( i = 1; i <= p - mlu; i++ )
        {
            a = spu - p + i;

            for ( j = 0; j <= p - i - mlu; j++ )
            {
                alf1[i][j] = (t1 - U[a + j]) / (U[spu + j + 1] - U[a + j]);
                oma1[i][j] = 1.0 - alf1[i][j];
            }
        }

        /* Save the w-alpha's */
        for ( i = 1; i <= r - mlw; i++ )
        {
            a = spw - r + i;

            for ( j = 0; j <= r - i - mlw; j++ )
            {
                alf2[i][j] = (t2 - W[a + j]) / (W[spw + j + 1] - W[a + j]);
                oma2[i][j] = 1.0 - alf2[i][j];
            }
        }

        /* For each column compute curve control point */
        a1 = spu - p;
        a2 = spw - r;

        for ( l = 0; l <= n; l++ )
        {
            /* Load auxiliary control points compressed from dir2 */
            for ( i = 0; i <= p - mlu; i++ )
            {
                for ( j = 0; j <= r - mlw; j++ )
                {
                    N_CopyCPt( Pw[a1 + i][l][a2 + j], &Qw[j] );
                }

                /* Now insert the dir2 knot */
                for ( j = 1; j <= r - mlw; j++ )
                {
                    for ( kk = 0; kk <= r - j - mlw; kk++ )
                    {
                        N_Combine2CPts( alf2[j][kk], Qw[kk + 1], oma2[j][kk], Qw[kk], &Qw[kk] );
                    }
                }

                /* Load auxiliary control point */
                N_CopyCPt( Qw[0], &Rw[i] );
            } /* end dir2 knot insertion */

            /* Now insert the dir1 knot */
            for ( i = 1; i <= p - mlu; i++ )
            {
                for ( j = 0; j <= p - i - mlu; j++ )
                {
                    N_Combine2CPts( alf1[i][j], Rw[j + 1], oma1[i][j], Rw[j], &Rw[j] );
                }
            }

            /* Load curve control point */
            N_CopyCPt( Rw[0], &Cw[l] );
        } /* End for each column */

        /* Load curve knot vector */
        for ( j = 0; j <= is; j++ )
            UC[j] = V[j];
    } /* End of NL_VDIR */

    /* Extract the w-curve */
    if( dir EQ NL_WDIR )
    {
        /* Save the u-alpha's */
        for ( i = 1; i <= p - mlu; i++ )
        {
            a = spu - p + i;

            for ( j = 0; j <= p - i - mlu; j++ )
            {
                alf1[i][j] = (t1 - U[a + j]) / (U[spu + j + 1] - U[a + j]);
                oma1[i][j] = 1.0 - alf1[i][j];
            }
        }

        /* Save the v-alpha's */
        for ( i = 1; i <= q - mlv; i++ )
        {
            a = spv - q + i;

            for ( j = 0; j <= q - i - mlv; j++ )
            {
                alf2[i][j] = (t1 - V[a + j]) / (V[spv + j + 1] - V[a + j]);
                oma2[i][j] = 1.0 - alf2[i][j];
            }
        }

        /* For each column compute curve control point */
        a1 = spu - p;
        a2 = spv - q;

        for ( l = 0; l <= o; l++ )
        {
            /* Load auxiliary control points compressed from dir2 */
            for ( i = 0; i <= p - mlu; i++ )
            {
                for ( j = 0; j <= q - mlv; j++ )
                {
                    N_CopyCPt( Pw[a1 + i][a2 + j][l], &Qw[j] );
                }

                /* Now insert the dir2 knot */
                for ( j = 1; j <= q - mlv; j++ )
                {
                    for ( kk = 0; kk <= q - j - mlv; kk++ )
                    {
                        N_Combine2CPts( alf2[j][kk], Qw[kk + 1], oma2[j][kk], Qw[kk], &Qw[kk] );
                    }
                }

                /* Load auxiliary control point */
                N_CopyCPt( Qw[0], &Rw[i] );
            } /* end dir2 knot insertion */

            /* Now insert the dir1 knot */
            for ( i = 1; i <= p - mlu; i++ )
            {
                for ( j = 0; j <= p - i - mlu; j++ )
                {
                    N_Combine2CPts( alf1[i][j], Rw[j + 1], oma1[i][j], Rw[j], &Rw[j] );
                }
            }

            /* Load curve control point */
            N_CopyCPt( Rw[0], &Cw[l] );
        } /* End for each column */

        /* Load curve knot vector */
        for ( j = 0; j <= it; j++ )
            UC[j] = W[j];
    } /* End of NL_VDIR */

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_VolumeMakeIsoCurve */

/*******************************************************************//**


   DESCRIPTION:

     This tools routine elevates the degree of a NURBS volume  from any
     degree to any higher degree. If the  output volume is  initialized 
     to NULL, memory to store new control points and knots is allocated. 
     If  the output  volume is  the same as  the input  volume, degree 
     elevation is done in place and the  original volume is  destroyed. 
     A typical calling example is:

       NL_VOLUME volP, volQ;
       NL_INDEX   t;
       NL_STACKS  SP, SQ;
       ...
       (define volP, get t);
       ...
       N_VolumeInitArrays(&volQ);
       N_VolumeElevateDegree(&volP,t,NL_UDIR,&volQ,&SP,&SQ);
       N_VolumeElevateDegree(&volP,t,NL_VDIR,&volP,&SP,&SP);
       N_VolumeElevateDegree(&volP,t,NL_WDIR,&volP,&SP,&SP);

     If memory is  available, volQ  is not  initialized and  the routine
     assumes  that memory  allocation  has been done. However, it checks  
     for the proper amount by looking  at the  highest indexes in volQ's  
     knot vector and control mesh objects.

   ACCESS:
   
     volP , input  ,  NURBS volume
     t    , input  ,  Degree Incement (new dir degree is old_degree+t)
     dir  , input  ,  Flag:
                        NL_UDIR: Elevate in u-direction
                        NL_VDIR: Elevate in v-direction
                        NL_WDIR: Elevate in w-direction
     volQ , output ,  Volume after degree elevation
     SP   , input  ,  volP's stack
     SQ   , input  ,  volQ's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
/* VlmTool.c */
NL_FLAG N_VolumeElevateDegree( NL_VOLUME *volP, NL_INDEX t, NL_FLAG dir, NL_VOLUME *volQ, NL_STACKS *SP, NL_STACKS *SQ )
{
    NL_PRIVATE NL_STRING rname = _T("N_VolumeElevateDegree");

    NL_FLAG error = NL_NO;

    NL_INDEX i, j, k, l, row, col, lev, mp, np, op, irp, isp, itp, mq, nq, oq, irq, isq, itq, r, s, a, b, mlt, kind, cind, pind = 0, lbz, rbz, bi, bj, bk, di, dj, dk, first, last, oldr, save;

    NL_DEGREE pp, qp, rp, pq, qq, rq;

    NL_REAL *UP, *VP, *WP, *UQ, *VQ, *WQ, ** ralf, ** roma, ** rbet, ** romb, *alfs, *omas, num, den;

    NL_RMATRIX dm;

    NL_KNOTVECTOR *knu, *knv, *knw;

    NL_CPOINT *** Pw, *** Qw, *** Bw, *** Nw, *** Cw;

    NL_VOLUME volA;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation */

    N_VolumeGetCPtsDegreesAndKnots( volP, &mp, &np, &op, &Pw, &pp, &qp, &rp, &irp, &isp, &itp, &UP, &VP, &WP );
    N_VolumeGetKnotVectors( volP, &knu, &knv, &knw );

    /* Check parameters and compute highest indexes */

    switch( dir )
    {
        case NL_UDIR:
            if( t LT 0 OR pp + t GT NL_DMAX )
                NL_ERROR( NL_DEG_ERR );

            error = N_KnotVectorIsValid( knu, pp, rname );

            if( error EQ NL_YES )
                NL_OUT;

            N_BasisGetSpanCount( knu, pp, &s );

            mq = mp + t * s;
            irq = irp + t * (s + 1);
            pq = (NL_DEGREE)(pp + t);
            nq = np;
            isq = isp;
            qq = qp;
            oq = op;
            itq = itp;
            rq = rp;

            bi = pp;
            bj = np;
            bk = op;
            di = pq;
            dj = np;
            dk = op;

            break;

        case NL_VDIR:
            if( t LT 0 OR qp + t GT NL_DMAX )
                NL_ERROR( NL_DEG_ERR );

            error = N_KnotVectorIsValid( knv, qp, rname );

            if( error EQ NL_YES )
                NL_OUT;

            N_BasisGetSpanCount( knv, qp, &s );

            mq = mp;
            irq = irp;
            pq = pp;
            nq = np + t * s;
            isq = isp + t * (s + 1);
            qq = (NL_DEGREE)(qp + t);
            oq = op;
            itq = itp;
            rq = rp;

            bi = mp;
            bj = qp;
            bk = op;
            di = mp;
            dj = qq;
            dk = op;

            break;

        case NL_WDIR:
            if( t LT 0 OR rp + t GT NL_DMAX )
                NL_ERROR( NL_DEG_ERR );

            error = N_KnotVectorIsValid( knw, rp, rname );

            if( error EQ NL_YES )
                NL_OUT;

            N_BasisGetSpanCount( knw, rp, &s );

            mq = mp;
            irq = irp;
            pq = pp;
            nq = np;
            isq = isp;
            qq = qp;
            oq = op + t * s;
            itq = itp + t * (s + 1);
            rq = (NL_DEGREE)(rp + t);

            bi = mp;
            bj = np;
            bk = rp;
            di = mp;
            dj = np;
            dk = rq;

            break;

        default:

            NL_ERROR( NL_CAL_ERR );
    }

    /* See if memory is needed */

    if( volP EQ volQ )
    {
        volA = *volP;

        error = N_AllocVolumeArrays( volP, mq, nq, oq, pq, qq, rq, irq, isq, itq, SP );

        if( error EQ NL_YES )
            NL_OUT;

        N_VolumeGetCPtsAndKnots( volP, &Qw, &UQ, &VQ, &WQ );
    }
    else
    {
        error = N_VolumeSizeArrays( volQ, mq, nq, oq, pq, qq, rq, irq, isq, itq, rname, SQ );

        if( error EQ NL_YES )
            NL_OUT;

        N_VolumeGetCPtsAndKnots( volQ, &Qw, &UQ, &VQ, &WQ );
    }

    /* See if elevation is required */
    if( t EQ 0 )
    {
        for ( row = 0; row <= mp; row++ )
        {
            for ( col = 0; col <= np; col++ )
            {
                for ( lev = 0; lev <= op; lev++ )
                    N_CopyCPt( Pw[row][col][lev], &Qw[row][col][lev] );
            }
        }

        for ( i = 0; i <= irp; i++ )
            UQ[i] = UP[i];

        for ( j = 0; j <= isp; j++ )
            VQ[j] = VP[j];

        for ( k = 0; k <= itp; k++ )
            WQ[k] = WP[k];

        NL_OUT;
    }

    /* Allocate local memory */
    a = NL_MAX3( pp, qp, rp );

    Bw = N_AllocCPt3dArray( bi, bj, bk, &SL );

    if( Bw EQ NULL )
        NL_QUIT;

    Nw = N_AllocCPt3dArray( bi, bj, bk, &SL );

    if( Nw EQ NULL )
        NL_QUIT;

    Cw = N_AllocCPt3dArray( di, dj, dk, &SL );

    if( Cw EQ NULL )
        NL_QUIT;

    ralf = N_AllocReal2dArray( a, 2 * a, &SL );

    if( ralf EQ NULL )
        NL_QUIT;

    roma = N_AllocReal2dArray( a, 2 * a, &SL );

    if( roma EQ NULL )
        NL_QUIT;

    rbet = N_AllocReal2dArray( a, 2 * a, &SL );

    if( rbet EQ NULL )
        NL_QUIT;

    romb = N_AllocReal2dArray( a, 2 * a, &SL );

    if( romb EQ NULL )
        NL_QUIT;

    alfs = N_AllocReal1dArray( a, &SL );

    if( alfs EQ NULL )
        NL_QUIT;

    omas = N_AllocReal1dArray( a, &SL );

    if( omas EQ NULL )
        NL_QUIT;

    /* Degree elevate in u-direction */
    if( dir EQ NL_UDIR )
    {
        a = pp;
        b = pp + 1;
        r = -1;
        cind = 1;
        kind = pq + 1;

        for ( i = 0; i <= pq; i++ )
            UQ[i] = UP[a];

        for ( j = 0; j <= isp; j++ )
            VQ[j] = VP[j];

        for ( k = 0; k <= itp; k++ )
            WQ[k] = WP[k];

        /* Initialize Bezier strip */

        for ( col = 0; col <= np; col++ )
        {
            for ( lev = 0; lev <= op; lev++ )
            {
                N_CopyCPt( Pw[0][col][lev], &Qw[0][col][lev] );

                for ( i = 0; i <= pp; i++ )
                {
                    N_CopyCPt( Pw[i][col][lev], &Bw[i][col][lev] );
                }
            }
        }

        /* Get degree elevation matrix */
        N_InitRealMatrix( &dm );
        error = N_BezGetDegreeElevationMatrix( pp, t, &dm, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /*************************************************************/
        /* Loop through the knot vector and do the following:        */
        /*   (1) Extract the i-th Bezier strip.                      */
        /*   (2) Degree elevate the strip.                           */
        /*   (3) Remove the knot between the i-th and the (i-1)-th   */
        /*       strip.                                              */
        /*************************************************************/

        while( b LT irp )
        {
            /* Get multiplicity of the knot */

            i = b;

            while( b LT irp AND UP[b]EQ UP[b + 1] )
                b++;
            mlt = b - i + 1;

            oldr = r;
            r = pp - mlt;

            if( oldr GT 0 )
                lbz = (oldr + 2) / 2;
            else
                lbz = 1;

            if( r GT 0 )
                rbz = pq - (r + 1) / 2;
            else
                rbz = pq;

            /* Save some entities */
            if( r GT 0 )
            {
                num = UP[b] - UP[a];

                for ( k = pp; k > mlt; k-- )
                {
                    alfs[k - mlt - 1] = num / (UP[a + k] - UP[a]);
                    omas[k - mlt - 1] = 1.0 - alfs[k - mlt - 1];
                }
            }

            first = kind - 2;
            last = kind;
            den = UP[b] - UP[a];

            for ( k = 1; k < oldr; k++ )
            {
                i = first;
                j = last;

                while( (j - i)GT k )
                {
                    if( i LT cind )
                    {
                        ralf[k][i - first] = (UP[b] - UQ[i]) / (UP[a] - UQ[i]);
                        roma[k][i - first] = 1.0 - ralf[k][i - first];
                    }

                    if( j GE lbz )
                    {
                        rbet[k][j - last] = (UP[b] - UQ[j - k]) / den;
                        romb[k][j - last] = 1.0 - rbet[k][j - last];
                    }
                    i++;
                    j--;
                }
                first--;
                last++;
            }

            /* For each col/lev, perform curve degree elevation */

            for ( col = 0; col <= np; col++ )
            {
                for ( lev = 0; lev <= op; lev++ )
                {
                    pind = cind;

                    /* Insert knot */

                    if( r GT 0 )
                    {
                        for ( j = 1; j <= r; j++ )
                        {
                            save = r - j;
                            s = mlt + j;

                            for ( k = pp; k >= s; k-- )
                            {
                                N_Combine2CPts( alfs[k - s], Bw[k][col][lev], omas[k - s], Bw[k - 1][col][lev], &Bw[k][col][lev] );
                            }
                            N_CopyCPt( Bw[pp][col][lev], &Nw[save][col][lev] );
                        }
                    } /* End of insert knot */

                    /* Now degree elevate Bezier */

                    error = N_BezVolumeDegreeElevate( Bw, pp, t, &dm, NL_UDIR, lbz, pq, col, lev, Cw );

                    if( error EQ NL_YES )
                        NL_OUT;

                    /* Remove the knot UP[a] */

                    if( oldr GT 1 )
                    {
                        first = kind - 2;
                        last = kind;

                        for ( k = 1; k < oldr; k++ )
                        {
                            i = first;
                            j = last;
                            l = j - kind + 1;

                            while( (j - i)GT k )
                            {
                                if( i LT cind )
                                {
                                    N_Combine2CPts( ralf[k][i - first], Qw[i][col][lev], roma[k][i - first], Qw[i - 1][col][lev], &Qw[i][col][lev] );
                                }

                                if( j GE lbz )
                                {
                                    N_Combine2CPts( rbet[k][j - last], Cw[l][col][lev], romb[k][j - last], Cw[l + 1][col][lev], &Cw[l][col][lev] );
                                }
                                i++;
                                j--;
                                l--;
                            }
                            first--;
                            last++;
                        }
                    } /* End of removing knot */

                    /* Load control points */

                    for ( i = lbz; i <= rbz; i++ )
                    {
                        N_CopyCPt( Cw[i][col][lev], &Qw[pind][col][lev] );
                        pind++;
                    }

                    /* Initialize for next pass through */

                    if( b LT irp )
                    {
                        for ( i = 0; i < r; i++ )
                            N_CopyCPt( Nw[i][col][lev], &Bw[i][col][lev] );

                        for ( i = r; i <= pp; i++ )
                            N_CopyCPt( Pw[b - pp + i][col][lev], &Bw[i][col][lev] );
                    }
                } /* end for each lev */
            }     /* End for each col */

            cind = pind;

            /* Load knot vector and prepare for next pass through */

            if( a NEQ pp )
            {
                for ( i = 0; i < pq - oldr; i++ )
                {
                    UQ[kind] = UP[a];
                    kind++;
                }
            }

            if( b LT irp )
            {
                a = b;
                b++;
            }
            else
            {
                for ( i = 0; i <= pq; i++ )
                    UQ[kind + i] = UP[b];
            }
        } /* End of while */
    }     /* End of NL_UDIR */

    /* Degree elevate in v-direction */

    if( dir EQ NL_VDIR )
    {
        a = qp;
        b = qp + 1;
        r = -1;
        cind = 1;
        kind = qq + 1;

        for ( i = 0; i <= irp; i++ )
            UQ[i] = UP[i];

        for ( j = 0; j <= qq; j++ )
            VQ[j] = VP[a];

        for ( k = 0; k <= itp; k++ )
            WQ[k] = WP[k];

        /* Initialize Bezier strip */

        for ( row = 0; row <= mp; row++ )
        {
            for ( lev = 0; lev <= op; lev++ )
            {
                N_CopyCPt( Pw[row][0][lev], &Qw[row][0][lev] );

                for ( j = 0; j <= qp; j++ )
                {
                    N_CopyCPt( Pw[row][j][lev], &Bw[row][j][lev] );
                }
            }
        }

        /* Get degree elevation matrix */

        N_InitRealMatrix( &dm );
        error = N_BezGetDegreeElevationMatrix( qp, t, &dm, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /*************************************************************/
        /* Loop through the knot vector and do the following:        */
        /*   (1) Extract the j-th Bezier strip.                      */
        /*   (2) Degree elevate the strip.                           */
        /*   (3) Remove the knot between the j-th and the (j-1)-th   */
        /*       strip.                                              */
        /*************************************************************/

        while( b LT isp )
        {
            /* Get multiplicity of the knot */

            i = b;

            while( b LT isp AND VP[b]EQ VP[b + 1] )
                b++;
            mlt = b - i + 1;

            oldr = r;
            r = qp - mlt;

            if( oldr GT 0 )
                lbz = (oldr + 2) / 2;
            else
                lbz = 1;

            if( r GT 0 )
                rbz = qq - (r + 1) / 2;
            else
                rbz = qq;

            /* Save some entities */

            if( r GT 0 )
            {
                num = VP[b] - VP[a];

                for ( k = qp; k > mlt; k-- )
                {
                    alfs[k - mlt - 1] = num / (VP[a + k] - VP[a]);
                    omas[k - mlt - 1] = 1.0 - alfs[k - mlt - 1];
                }
            }

            first = kind - 2;
            last = kind;
            den = VP[b] - VP[a];

            for ( k = 1; k < oldr; k++ )
            {
                i = first;
                j = last;

                while( (j - i)GT k )
                {
                    if( i LT cind )
                    {
                        ralf[k][i - first] = (VP[b] - VQ[i]) / (VP[a] - VQ[i]);
                        roma[k][i - first] = 1.0 - ralf[k][i - first];
                    }

                    if( j GE lbz )
                    {
                        rbet[k][j - last] = (VP[b] - VQ[j - k]) / den;
                        romb[k][j - last] = 1.0 - rbet[k][j - last];
                    }
                    i++;
                    j--;
                }
                first--;
                last++;
            }

            /* For each row/lev, perform curve degree elevation */

            for ( row = 0; row <= mp; row++ )
            {
                for ( lev = 0; lev <= op; lev++ )
                {
                    pind = cind;

                    /* Insert knot */

                    if( r GT 0 )
                    {
                        for ( j = 1; j <= r; j++ )
                        {
                            save = r - j;
                            s = mlt + j;

                            for ( k = qp; k >= s; k-- )
                            {
                                N_Combine2CPts( alfs[k - s], Bw[row][k][lev], omas[k - s], Bw[row][k - 1][lev], &Bw[row][k][lev] );
                            }
                            N_CopyCPt( Bw[row][qp][lev], &Nw[row][save][lev] );
                        }
                    } /* End of insert knot */

                    /* Now degree elevate Bezier */

                    error = N_BezVolumeDegreeElevate( Bw, qp, t, &dm, NL_VDIR, lbz, qq, row, lev, Cw );

                    if( error EQ NL_YES )
                        NL_OUT;

                    /* Remove the knot VP[a] */

                    if( oldr GT 1 )
                    {
                        first = kind - 2;
                        last = kind;

                        for ( k = 1; k < oldr; k++ )
                        {
                            i = first;
                            j = last;
                            l = j - kind + 1;

                            while( (j - i)GT k )
                            {
                                if( i LT cind )
                                {
                                    N_Combine2CPts( ralf[k][i - first], Qw[row][i][lev], roma[k][i - first], Qw[row][i - 1][lev], &Qw[row][i][lev] );
                                }

                                if( j GE lbz )
                                {
                                    N_Combine2CPts( rbet[k][j - last], Cw[row][l][lev], romb[k][j - last], Cw[row][l + 1][lev], &Cw[row][l][lev] );
                                }
                                i++;
                                j--;
                                l--;
                            }
                            first--;
                            last++;
                        }
                    } /* End of removing knot */

                    /* Load control points */

                    for ( j = lbz; j <= rbz; j++ )
                    {
                        N_CopyCPt( Cw[row][j][lev], &Qw[row][pind][lev] );
                        pind++;
                    }

                    /* Initialize for next pass through */

                    if( b LT isp )
                    {
                        for ( j = 0; j < r; j++ )
                            N_CopyCPt( Nw[row][j][lev], &Bw[row][j][lev] );

                        for ( j = r; j <= qp; j++ )
                            N_CopyCPt( Pw[row][b - qp + j][lev], &Bw[row][j][lev] );
                    }
                } /* End for each level */
            }     /* End for each row */

            cind = pind;

            /* Load knot vector and prepare for next pass through */

            if( a NEQ qp )
            {
                for ( j = 0; j < qq - oldr; j++ )
                {
                    VQ[kind] = VP[a];
                    kind++;
                }
            }

            if( b LT isp )
            {
                a = b;
                b++;
            }
            else
            {
                for ( j = 0; j <= qq; j++ )
                    VQ[kind + j] = VP[b];
            }
        } /* End of while */
    }     /* End of NL_VDIR */

    /* Degree elevate in w-direction */
    if( dir EQ NL_WDIR )
    {
        a = rp;
        b = rp + 1;
        r = -1;
        cind = 1;
        kind = rq + 1;

        for ( i = 0; i <= irp; i++ )
            UQ[i] = UP[i];

        for ( j = 0; j <= isp; j++ )
            VQ[j] = VP[j];

        for ( k = 0; k <= rq; k++ )
            WQ[k] = WP[a];

        /* Initialize Bezier strip */

        for ( row = 0; row <= mp; row++ )
        {
            for ( col = 0; col <= np; col++ )
            {
                N_CopyCPt( Pw[row][col][0], &Qw[row][col][0] );

                for ( k = 0; k <= rp; k++ )
                {
                    N_CopyCPt( Pw[row][col][k], &Bw[row][col][k] );
                }
            }
        }

        /* Get degree elevation matrix */
        N_InitRealMatrix( &dm );
        error = N_BezGetDegreeElevationMatrix( rp, t, &dm, &SL );

        if( error EQ NL_YES )
            NL_OUT;

        /*************************************************************/
        /* Loop through the knot vector and do the following:        */
        /*   (1) Extract the k-th Bezier strip.                      */
        /*   (2) Degree elevate the strip.                           */
        /*   (3) Remove the knot between the k-th and the (k-1)-th   */
        /*       strip.                                              */
        /*************************************************************/

        while( b LT itp )
        {
            /* Get multiplicity of the knot */

            i = b;

            while( b LT itp AND WP[b]EQ WP[b + 1] )
                b++;
            mlt = b - i + 1;

            oldr = r;
            r = rp - mlt;

            if( oldr GT 0 )
                lbz = (oldr + 2) / 2;
            else
                lbz = 1;

            if( r GT 0 )
                rbz = rq - (r + 1) / 2;
            else
                rbz = rq;

            /* Save some entities */
            if( r GT 0 )
            {
                num = WP[b] - WP[a];

                for ( k = rp; k > mlt; k-- )
                {
                    alfs[k - mlt - 1] = num / (WP[a + k] - WP[a]);
                    omas[k - mlt - 1] = 1.0 - alfs[k - mlt - 1];
                }
            }

            first = kind - 2;
            last = kind;
            den = WP[b] - WP[a];

            for ( k = 1; k < oldr; k++ )
            {
                i = first;
                j = last;

                while( (j - i)GT k )
                {
                    if( i LT cind )
                    {
                        ralf[k][i - first] = (WP[b] - WQ[i]) / (WP[a] - WQ[i]);
                        roma[k][i - first] = 1.0 - ralf[k][i - first];
                    }

                    if( j GE lbz )
                    {
                        rbet[k][j - last] = (WP[b] - WQ[j - k]) / den;
                        romb[k][j - last] = 1.0 - rbet[k][j - last];
                    }
                    i++;
                    j--;
                }
                first--;
                last++;
            }

            /* For each row/col, perform curve degree elevation */

            for ( row = 0; row <= mp; row++ )
            {
                for ( col = 0; col <= np; col++ )
                {
                    pind = cind;

                    /* Insert knot */

                    if( r GT 0 )
                    {
                        for ( j = 1; j <= r; j++ )
                        {
                            save = r - j;
                            s = mlt + j;

                            for ( k = rp; k >= s; k-- )
                            {
                                N_Combine2CPts( alfs[k - s], Bw[row][col][k], omas[k - s], Bw[row][col][k - 1], &Bw[row][col][k] );
                            }
                            N_CopyCPt( Bw[row][col][rp], &Nw[row][col][save] );
                        }
                    } /* End of insert knot */

                    /* Now degree elevate Bezier */

                    error = N_BezVolumeDegreeElevate( Bw, rp, t, &dm, NL_WDIR, lbz, rq, row, col, Cw );

                    if( error EQ NL_YES )
                        NL_OUT;

                    /* Remove the knot WP[a] */

                    if( oldr GT 1 )
                    {
                        first = kind - 2;
                        last = kind;

                        for ( k = 1; k < oldr; k++ )
                        {
                            i = first;
                            j = last;
                            l = j - kind + 1;

                            while( (j - i)GT k )
                            {
                                if( i LT cind )
                                {
                                    N_Combine2CPts( ralf[k][i - first], Qw[row][col][i], roma[k][i - first], Qw[row][col][i - 1], &Qw[row][col][i] );
                                }

                                if( j GE lbz )
                                {
                                    N_Combine2CPts( rbet[k][j - last], Cw[row][col][l], romb[k][j - last], Cw[row][col][l + 1], &Cw[row][col][l] );
                                }
                                i++;
                                j--;
                                l--;
                            }
                            first--;
                            last++;
                        }
                    } /* End of removing knot */

                    /* Load control points */

                    for ( k = lbz; k <= rbz; k++ )
                    {
                        N_CopyCPt( Cw[row][col][k], &Qw[row][col][pind] );
                        pind++;
                    }

                    /* Initialize for next pass through */

                    if( b LT itp )
                    {
                        for ( k = 0; k < r; k++ )
                            N_CopyCPt( Nw[row][col][k], &Bw[row][col][k] );

                        for ( k = r; k <= rp; k++ )
                            N_CopyCPt( Pw[row][col][b - rp + k], &Bw[row][col][k] );
                    }
                } /* End for each level */
            }     /* End for each row */

            cind = pind;

            /* Load knot vector and prepare for next pass through */

            if( a NEQ rp )
            {
                for ( k = 0; k < rq - oldr; k++ )
                {
                    WQ[kind] = WP[a];
                    kind++;
                }
            }

            if( b LT itp )
            {
                a = b;
                b++;
            }
            else
            {
                for ( k = 0; k <= rq; k++ )
                    WQ[kind + k] = WP[b];
            }
        } /* End of while */
    }     /* End of NL_WDIR */

    /* If insertion is in place, kill old volume */

    if( volP EQ volQ )
        N_FreeVolume( &volA, SP );

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_VolumeElevateDegree */

/*******************************************************************//**


   DESCRIPTION:

     This tools routine  extracts the twelve boundary curves from a  NURBS 
     volume. If the  output  curves  are initialized to NULL, memory to 
     store  curve  control  points  and  knots is  allocated. A  typical 
     calling example is: 

       NL_VOLUME   vol;
       NL_CURVE    curU00, curU01, curU11, curU10;
       NL_CURVE    curV00, curV01, curV11, curV10;
       NL_CURVE    curW00, curW01, curW11, curW10;
       NL_STACKS   SC;
       ...
       (define vol);
       ...
       N_CrvInitArrays(&curU00); N_CrvInitArrays(&curU01); N_CrvInitArrays(&curU11); N_CrvInitArrays(&curU10);
       N_CrvInitArrays(&curV00); N_CrvInitArrays(&curV01); N_CrvInitArrays(&curV11); N_CrvInitArrays(&curV10);
       N_CrvInitArrays(&curW00); N_CrvInitArrays(&curW01); N_CrvInitArrays(&curW11); N_CrvInitArrays(&curW10);
       N_VolumeMakeBoundaryCurves(&vol,
                                  &curU00, &curU01, &curU11, &curU10,
                                  &curV00, &curV01, &curV11, &curV10,
                                  &curW00, &curW01, &curW11, &curW10,
                                  &SC);

     If memory is available, the twelve curves are not initialized and the 
     routine assumes that memory allocations have been done. However, it  
     checks  for the proper  amount by looking at the highest indexes in 
     the knot vector and polygon objects. 


   ACCESS:
   
     vol   , input  ,  NURBS volume
     curU00, output ,  U IsoCurve for v = VMin, w = WMin
     curU01, output ,  U IsoCurve for v = VMin, w = WMax
     curU11, output ,  U IsoCurve for v = VMax, w = WMax
     curU10, output ,  U IsoCurve for v = VMax, w = WMin

     curV00, output ,  V IsoCurve for u = UMin, w = WMin
     curV01, output ,  V IsoCurve for u = UMin, w = WMax
     curV11, output ,  V IsoCurve for u = UMax, w = WMax
     curV10, output ,  V IsoCurve for u = UMax, w = WMin

     curW00, output ,  W IsoCurve for u = UMin, v = VMin
     curW01, output ,  W IsoCurve for u = UMin, v = VMax
     curW11, output ,  W IsoCurve for u = UMax, v = VMax
     curW10, output ,  W IsoCurve for u = UMax, v = VMin

     SC    , input  ,  cur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
/* VlmTool.c */
NL_FLAG N_VolumeMakeBoundaryCurves( NL_VOLUME *vol, NL_CURVE *curU00, NL_CURVE *curU01, NL_CURVE *curU11, NL_CURVE *curU10, NL_CURVE *curV00, NL_CURVE *curV01, NL_CURVE *curV11, NL_CURVE *curV10, NL_CURVE *curW00, NL_CURVE *curW01, NL_CURVE *curW11, NL_CURVE *curW10, NL_STACKS *SC )
{

    NL_FLAG error = NL_NO;

    NL_INDEX ir, is, it;

    NL_REAL *U, *V, *W;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation and extract boundaries */

    N_VolumeGetKnots( vol, &ir, &is, &it, &U, &V, &W );

    error = N_VolumeMakeIsoCurve( vol, V[0], W[0], NL_UDIR, curU00, SC );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_VolumeMakeIsoCurve( vol, V[0], W[it], NL_UDIR, curU01, SC );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_VolumeMakeIsoCurve( vol, V[is], W[it], NL_UDIR, curU11, SC );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_VolumeMakeIsoCurve( vol, V[is], W[0], NL_UDIR, curU10, SC );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VolumeMakeIsoCurve( vol, U[0], W[0], NL_UDIR, curV00, SC );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_VolumeMakeIsoCurve( vol, U[0], W[it], NL_UDIR, curV01, SC );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_VolumeMakeIsoCurve( vol, U[ir], W[it], NL_UDIR, curV11, SC );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_VolumeMakeIsoCurve( vol, U[ir], W[0], NL_UDIR, curV10, SC );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VolumeMakeIsoCurve( vol, U[0], V[0], NL_UDIR, curW00, SC );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_VolumeMakeIsoCurve( vol, U[0], V[is], NL_UDIR, curW01, SC );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_VolumeMakeIsoCurve( vol, U[ir], V[is], NL_UDIR, curW11, SC );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_VolumeMakeIsoCurve( vol, U[ir], V[0], NL_UDIR, curW10, SC );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_VolumeMakeBoundaryCurves */

/*******************************************************************//**


   DESCRIPTION:

     This tools routine  extracts the 6 boundary surfaces from a  NURBS 
     volume. If the  output  srfaces  are initialized to NULL, memory to 
     store  surface  control  points  and  knots is  allocated. A  typical 
     calling example is: 

       NL_VOLUME   vol;
       NL_SURFACE    surU0, surU1, 
       NL_SURFACE    surV0, surV1, 
       NL_SURFACE    surW0, surW1, 
       NL_STACKS   SC;
       ...
       (define vol);
       ...
       N_SrfInitArrays(&surU0); N_SrfInitArrays(&surU1); 
       N_SrfInitArrays(&surV0); N_SrfInitArrays(&surV1); 
       N_SrfInitArrays(&surW0); N_SrfInitArrays(&surW1); 
       N_VolumeMakeBoundarySurfaces(&vol,
                                    &surU0,  &surU1,
                                    &surV0,  &surV1,
                                    &surW0,  &surW1,
                                    &SC);

     If memory is available, the 6 surfaces are not initialized and the 
     routine assumes that memory allocations have been done. However, it  
     checks  for the proper  amount by looking at the highest indexes in 
     the knot vector and polygon objects. 


   ACCESS:
   
     vol   , input  ,  NURBS volume
     surU0, output ,  U IsoSurface for u = UMin
     surU1, output ,  U IsoSurface for u = UMin

     surV0, output ,  V IsoSurface for v = VMin
     surV1, output ,  V IsoSurface for v = VMin

     surW0, output ,  W IsoSurface for w = WMin
     surW1, output ,  W IsoSurface for w = WMin

     SC    , input  ,  cur's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/
/* VlmTool.c */
NL_FLAG N_VolumeMakeBoundarySurfaces( NL_VOLUME *vol, NL_SURFACE *surU0, NL_SURFACE *surU1, NL_SURFACE *surV0,  NL_SURFACE *surV1, NL_SURFACE *surW0,  NL_SURFACE *surW1, NL_STACKS *SC )
{

    NL_FLAG error = NL_NO;

    NL_INDEX ir, is, it;

    NL_REAL *U, *V, *W;

    NL_STACKS SL;

    /* Start NURBS */

    N_InitNurbs( &SL );

    /* Get local notation and extract boundaries */

    N_VolumeGetKnots( vol, &ir, &is, &it, &U, &V, &W );

    error = N_VolumeMakeIsoSurface( vol, U[0], NL_UDIR, surU0, SC );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_VolumeMakeIsoSurface( vol, U[ir], NL_UDIR, surU1, SC );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VolumeMakeIsoSurface( vol, V[0], NL_VDIR, surV0, SC );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_VolumeMakeIsoSurface( vol, V[is], NL_VDIR, surV1, SC );

    if( error EQ NL_YES )
        NL_OUT;

    error = N_VolumeMakeIsoSurface( vol, W[0], NL_WDIR, surW0, SC );

    if( error EQ NL_YES )
        NL_OUT;
    error = N_VolumeMakeIsoSurface( vol, W[it], NL_WDIR, surW1, SC );

    if( error EQ NL_YES )
        NL_OUT;

    /* End NURBS and Exit */

    EXIT:

    N_EndNurbs( &SL );

    return (error);
} /* end N_VolumeMakeBoundarySurfaces */

/*=====================================*/
/* File Functions                      */
/*=====================================*/

/*******************************************************************//**


   DESCRIPTION:

     This NL_VOLUME routine clamps an NL_VOLUME Entity volume  to  its
     valid parameter bounds. This function assumes that the surface has
     been loaded into Nlib structure, and:
       (1) ir = m+p+1,  is = n+q+1, and it = o+r+1
       (2) U[i] <= U[i+1],  V[j] <= V[j+1], and W[k] <= W[k+1]  
               for 0<=i<=ir , 0<=j<=is, 0<=k<=it
       (3) U[p] <= ub[0] < ub[1] <= U[ir-p]  (ub[0],ub[1] are bounds)
       (4) V[q] <= vb[0] < vb[1] <= V[is-q]  (vb[0],vb[1] are bounds)
       (5) W[r] <= wb[0] < wb[1] <= W[it-r]  (wb[0],wb[1] are bounds)
     Clamping is done in place. A typical calling example is:

       NL_VOLUME     vol;
       NL_PARAMETER  ub[2], vb[2], wb[2];
       NL_STACKS     S;
       ...
       (define vol, and load bounds in ub, vb, wb);
       ...
       ST_igsvcp(&vol,ub,vb,wb,&S);


   ACCESS:
   
     vol  , in/out ,  NURBS surface
     ub   , input  ,  u-bounds for clamping
     vb   , input  ,  v-bounds for clamping
     wb   , input  ,  w-bounds for clamping
     S    , input  ,  vol's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG ST_igsvcp( NL_VOLUME *vol, NL_PARAMETER ub[2], NL_PARAMETER vb[2], NL_PARAMETER wb[2], NL_STACKS *S )
{

    NL_FLAG error = NL_NO;

    NL_INDEX ii, jj, kk, i, j, k, ll, lk, lr, span1 = 0, mult1, span2 = 0, mult2, js, je, m, n, o, ir, is, it;

    NL_DEGREE p, q, r;

    NL_REAL *UP, *UQ, *VP, *VQ, *WP, *WQ, alf, oma, left;

    NL_CPOINT *** Pw, *** Qw;

    NL_VOLUME volA;

    /* Get local notation */

    N_VolumeGetCPtsDegreesAndKnots( vol, &m, &n, &o, &Pw, &p, &q, &r, &ir, &is, &it, &UP, &VP, &WP );

    /* First clamp in u, then in v , then in w */

    /* Find knot spans and set multiplicities of bounds (as knots) */
    mult1 = 0;

    for ( i = 0; i <= ir; i++ )
    {
        if( UP[i]EQ ub[0] )
            mult1 += 1;

        if( UP[i]GT ub[0] )
        {
            span1 = i - 1;
            break;
        }
    }
    mult2 = 0;

    for ( i = ir; i >= 0; i-- )
    {
        if( UP[i]EQ ub[1] )
            mult2 += 1;

        if( UP[i]LT ub[1] )
        {
            span2 = i + mult2;
            break;
        }
    }

    /* Handle case that junk is in first and last knots */
    if( span1 GE p AND mult1 EQ span1 )
    {
        UP[0] = ub[0];
        mult1 += 1;
    }

    if( span2 EQ ir - 1 AND mult2 GE p )
    {
        UP[ir] = ub[1];
        mult2 += 1;
        span2 += 1;
    }

    /* Check if it is already properly clamped (at ends) */
    if( mult1 NEQ span1 + 1 OR span2 NEQ ir OR mult2 LT p + 1 )
    { /* must clamp in u */

        /* Get new indices */
        js = span1 - p;
        je = span2 - mult2;

        m = je - js;
        ir = span2 - span1 - mult2 + 2 * p + 1;

        /* Get new memory for control points and knots */
        volA = *vol;
        error = N_AllocVolumeArrays( vol, m, n, o, p, q, r, ir, is, it, S );

        if( error EQ NL_YES )
            NL_OUT;
        N_VolumeGetCPtsAndKnots( vol, &Qw, &UQ, &VQ, &WQ );

        /* Get initial control points */
        for ( jj = 0; jj <= n; jj++ )
            for ( kk = 0; kk <= o; kk++ )
                for ( i = js; i <= je; i++ )
                    N_CopyCPt( Pw[i][jj][kk], &Qw[i - js][jj][kk] );

        /* Insert the left knot */
        ll = span1 - p;

        for ( i = 1; i <= p - mult1; i++ ) /* number of knots to insert */
        {
            for ( j = 0; j <= p - i - mult1; j++ )
            {
                left = UP[ll + i + j];
                alf = (ub[0] - left) / (UP[span1 + j + 1] - left);
                oma = 1.0 - alf;

                for ( jj = 0; jj <= n; jj++ )
                    for ( kk = 0; kk <= o; kk++ )
                        N_Combine2CPts( alf, Qw[j + 1][jj][kk], oma, Qw[j][jj][kk], &Qw[j][jj][kk] );
            }
        }

        /* Insert the right knot */
        lr = span2 - p;
        lk = m - p + mult2;

        for ( i = 1; i <= p - mult2; i++ )
        {
            for ( j = p - i - mult2; j >= 0; j-- )
            {
                k = lk + i + j;
                left = UP[lr + i + j];

                if( left LT ub[0] )
                    left = ub[0];

                alf = (ub[1] - left) / (UP[span2 + j + 1] - left);
                oma = 1.0 - alf;

                for ( jj = 0; jj <= n; jj++ )
                    for ( kk = 0; kk <= o; kk++ )
                        N_Combine2CPts( alf, Qw[k][jj][kk], oma, Qw[k - 1][jj][kk], &Qw[k][jj][kk] );
            }
        }

        /* Load the knot vectors */
        j = -1;

        for ( i = 0; i <= p; i++ )
            UQ[++j] = ub[0];

        for ( i = span1 + 1; i <= span2 - mult2; i++ )
            UQ[++j] = UP[i];

        for ( i = 0; i <= p; i++ )
            UQ[++j] = ub[1];

        for ( j = 0; j <= is; j++ )
            VQ[j] = VP[j];

        for ( k = 0; k <= it; k++ )
            WQ[k] = WP[k];

        /* Kill old surface and get new values for local notation */
        N_FreeVolume( &volA, S );
        N_VolumeGetCPtsDegreesAndKnots( vol, &m, &n, &o, &Pw, &p, &q, &r, &ir, &is, &it, &UP, &VP, &WP );
    } /* end of u-clamping */

    /* Now check if we need to clamp in v */

    /* Find knot spans and set multiplicities of bounds (as knots) */
    mult1 = 0;

    for ( i = 0; i <= is; i++ )
    {
        if( VP[i]EQ vb[0] )
            mult1 += 1;

        if( VP[i]GT vb[0] )
        {
            span1 = i - 1;
            break;
        }
    }
    mult2 = 0;

    for ( i = is; i >= 0; i-- )
    {
        if( VP[i]EQ vb[1] )
            mult2 += 1;

        if( VP[i]LT vb[1] )
        {
            span2 = i + mult2;
            break;
        }
    }

    /* Handle case that junk is in first and last knots */
    if( span1 GE q AND mult1 EQ span1 )
    {
        VP[0] = vb[0];
        mult1 += 1;
    }

    if( span2 EQ is - 1 AND mult2 GE q )
    {
        VP[is] = vb[1];
        mult2 += 1;
        span2 += 1;
    }

    /* Check if it is already properly clamped (at ends) */
    if( mult1 NEQ span1 + 1 OR span2 NEQ is OR mult2 LT q + 1 )
    { /* must clamp in v */

        /* Get new indices */
        js = span1 - q;
        je = span2 - mult2;

        n = je - js;
        is = span2 - span1 - mult2 + 2 * q + 1;

        /* Get new memory for control points and knots */

        volA = *vol;
        error = N_AllocVolumeArrays( vol, m, n, o, p, q, r, ir, is, it, S );

        if( error EQ NL_YES )
            NL_OUT;
        N_VolumeGetCPtsAndKnots( vol, &Qw, &UQ, &VQ, &WQ );

        /* Get initial control points */
        for ( ii = 0; ii <= m; ii++ )
            for ( j = js; j <= je; j++ )
                for ( kk = 0; kk <= o; kk++ )
                    N_CopyCPt( Pw[ii][j][kk], &Qw[ii][j - js][kk] );

        /* Insert the left knot */
        ll = span1 - q;

        for ( i = 1; i <= q - mult1; i++ )
        {
            for ( j = 0; j <= q - i - mult1; j++ )
            {
                left = VP[ll + i + j];
                alf = (vb[0] - left) / (VP[span1 + j + 1] - left);
                oma = 1.0 - alf;

                for ( ii = 0; ii <= m; ii++ )
                    for ( kk = 0; kk <= o; kk++ )
                        N_Combine2CPts( alf, Qw[ii][j + 1][kk], oma, Qw[ii][j][kk], &Qw[ii][j][kk] );
            }
        }

        /* Insert the right knot */
        lr = span2 - q;
        lk = n - q + mult2;

        for ( i = 1; i <= q - mult2; i++ )
        {
            for ( j = q - i - mult2; j >= 0; j-- )
            {
                k = lk + i + j;
                left = VP[lr + i + j];

                if( left LT vb[0] )
                    left = vb[0];

                alf = (vb[1] - left) / (VP[span2 + j + 1] - left);
                oma = 1.0 - alf;

                for ( ii = 0; ii <= m; ii++ )
                    for ( kk = 0; kk <= o; kk++ )
                        N_Combine2CPts( alf, Qw[ii][k][kk], oma, Qw[ii][k - 1][kk], &Qw[ii][k][kk] );
            }
        }

        /* Load the knot vectors */
        j = -1;

        for ( i = 0; i <= q; i++ )
            VQ[++j] = vb[0];

        for ( i = span1 + 1; i <= span2 - mult2; i++ )
            VQ[++j] = VP[i];

        for ( i = 0; i <= q; i++ )
            VQ[++j] = vb[1];

        for ( i = 0; i <= ir; i++ )
            UQ[i] = UP[i];

        for ( k = 0; k <= it; k++ )
            WQ[k] = WP[k];

        /* Kill old surface */
        N_FreeVolume( &volA, S );
        N_VolumeGetCPtsDegreesAndKnots( vol, &m, &n, &o, &Pw, &p, &q, &r, &ir, &is, &it, &UP, &VP, &WP );
    } /* end of v-clamping */

    /* Now check if we need to clamp in w */

    /* Find knot spans and set multiplicities of bounds (as knots) */
    mult1 = 0;

    for ( i = 0; i <= it; i++ )
    {
        if( WP[i]EQ wb[0] )
            mult1 += 1;

        if( WP[i]GT wb[0] )
        {
            span1 = i - 1;
            break;
        }
    }
    mult2 = 0;

    for ( i = it; i >= 0; i-- )
    {
        if( WP[i]EQ wb[1] )
            mult2 += 1;

        if( WP[i]LT wb[1] )
        {
            span2 = i + mult2;
            break;
        }
    }

    /* Handle case that junk is in first and last knots */
    if( span1 GE r AND mult1 EQ span1 )
    {
        WP[0] = wb[0];
        mult1 += 1;
    }

    if( span2 EQ it - 1 AND mult2 GE r )
    {
        WP[it] = wb[1];
        mult2 += 1;
        span2 += 1;
    }

    /* Check if it is already properly clamped (at ends) */
    if( mult1 NEQ span1 + 1 OR span2 NEQ it OR mult2 LT r + 1 )
    { /* must clamp in w */

        /* Get new indices */
        js = span1 - r;
        je = span2 - mult2;

        o = je - js;
        it = span2 - span1 - mult2 + 2 * r + 1;

        /* Get new memory for control points and knots */
        volA = *vol;
        error = N_AllocVolumeArrays( vol, m, n, o, p, q, r, ir, is, it, S );

        if( error EQ NL_YES )
            NL_OUT;
        N_VolumeGetCPtsAndKnots( vol, &Qw, &UQ, &VQ, &WQ );

        /* Get initial control points */
        for ( ii = 0; ii <= m; ii++ )
            for ( jj = 0; jj <= n; jj++ )
                for ( k = js; k <= je; k++ )
                    N_CopyCPt( Pw[ii][jj][k], &Qw[ii][jj][k - js] );

        /* Insert the left knot */
        ll = span1 - r;

        for ( i = 1; i <= r - mult1; i++ ) /* number of knots to insert */
        {
            for ( j = 0; j <= r - i - mult1; j++ )
            {
                left = WP[ll + i + j];
                alf = (wb[0] - left) / (WP[span1 + j + 1] - left);
                oma = 1.0 - alf;

                for ( ii = 0; ii <= m; ii++ )
                    for ( jj = 0; jj <= n; jj++ )
                        N_Combine2CPts( alf, Qw[ii][jj][j + 1], oma, Qw[ii][jj][j], &Qw[ii][jj][j] );
            }
        }

        /* Insert the right knot */
        lr = span2 - r;
        lk = o - r + mult2;

        for ( i = 1; i <= r - mult2; i++ )
        {
            for ( j = r - i - mult2; j >= 0; j-- )
            {
                k = lk + i + j;
                left = WP[lr + i + j];

                if( left LT wb[0] )
                    left = wb[0];

                alf = (wb[1] - left) / (WP[span2 + j + 1] - left);
                oma = 1.0 - alf;

                for ( ii = 0; ii <= m; ii++ )
                    for ( jj = 0; jj <= n; jj++ )
                        N_Combine2CPts( alf, Qw[ii][jj][k], oma, Qw[ii][jj][k - 1], &Qw[ii][jj][k] );
            }
        }

        /* Load the knot vectors */
        j = -1;

        for ( i = 0; i <= r; i++ )
            WQ[++j] = wb[0];

        for ( i = span1 + 1; i <= span2 - mult2; i++ )
            WQ[++j] = WP[i];

        for ( i = 0; i <= r; i++ )
            WQ[++j] = wb[1];

        for ( i = 0; i <= ir; i++ )
            UQ[i] = UP[i];

        for ( j = 0; j <= is; j++ )
            VQ[j] = VP[j];

        /* Kill old surface */
        N_FreeVolume( &volA, S );
    } /* end of w-clamping */

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end ST_igsvcp */

/*******************************************************************//**


   DESCRIPTION:

     This NL_VOLUME routine sanitizes, validates, and corrects knot multipli-
     cities of a volume that has  been  put  into  Nlib
     form and clamped. In particular, it ensures that:
       (1) end knot multiplicities are not greater than p+1, q+1, r+1
       (2) internal knot multiplicities are not greater than p, q, or r (and
           hence, the volume is at least C0 continuous)
     The sanitizing is done in place. A typical calling example is:

       NL_VOLUME  vol;
       NL_STACKS   S;
       ...
       (define vol);
       ...
       ST_igsvsk(&vol,&S);


   ACCESS:
   
     vol  , in/out ,  NURBS volume
     S    , input  ,  vol's stack


   RETURN CODES:

     0 : No error
     1 : Error saved in NL_ERROR

   ***********************************************************************/

NL_FLAG ST_igsvsk( NL_VOLUME *vol, NL_STACKS *S )
{

    NL_FLAG error = NL_NO;

    NL_INDEX i, ii, jj, kk, k1, k2, k3, k4, m, n, o, mm, nn, oo, ir, irr, is, iss, it, itt, mult;

    NL_DEGREE p, q, r;

    NL_PARAMETER us, ue, vs, ve, ws, we;

    NL_REAL d, *UP, *VP, *WP;

    NL_CPOINT *** Pw;

    NL_CMESH *mesh;

    NL_KNOTVECTOR *knu, *knv, *knw;

    /* Get local notation */
    N_VolumeGetCPtsDegreesAndKnots( vol, &m, &n, &o, &Pw, &p, &q, &r, &ir, &is, &it, &UP, &VP, &WP );
    mm = m;
    nn = n;
    oo = o;
    irr = ir;
    iss = is;
    itt = it;

    /* First the u-direction, then the v-direction, finally the w-direction */

    /* Eliminate internal multiplicities greater than p */
    us = UP[0];
    ue = UP[ir];
    i = p + 1;

    while( UP[i]EQ UP[i - 1] )
        i += 1;

    while( UP[i]LT ue )
    {
        /* get multiplicity of UP[i] */

        mult = 1;

        while( UP[i]EQ UP[i + mult] )
            mult += 1;

        if( mult GT p )         /* reduce to multiplicity p */
        {
            k1 = i - 1;         /* first control point to replace */
            k2 = k1 + mult - p; /* last  control point to replace */
            k3 = k2 - k1;
            d = 1.0 / ((NL_REAL)k3 + (NL_REAL)1);

            for ( jj = 0; jj <= nn; jj++ )
                for ( kk = 0; kk <= oo; kk++ )
                {
                    for ( ii = k1 + 1; ii <= k2; ii++ )
                        N_Sum2CPts( Pw[k1][jj][kk], Pw[ii][jj][kk], &Pw[k1][jj][kk] );
                    N_ScaleCPt( d, Pw[k1][jj][kk], &Pw[k1][jj][kk] );
                    k4 = k1 + 1;

                    for ( ii = k2 + 1; ii <= mm; ii++ )
                        N_CopyCPt( Pw[ii][jj][kk], &Pw[k4++][jj][kk] );
                }

            mm -= k3;
            k1 = i + p;

            for ( jj = i + mult; jj <= irr; jj++ )
                UP[k1++] = UP[jj];
            irr -= k3;
            mult = p;
        }

        i = i + mult;
    }

    /* Validate that UP[0] multiplicity not greater than p+1 */
    mult = 0;

    while( us EQ UP[p + mult + 1] )
        mult += 1;

    if( mult GT 0 )
    {
        k1 = p + 1;

        for ( ii = p + mult + 1; ii <= irr; ii++ )
            UP[k1++] = UP[ii];
        irr -= mult;

        for ( jj = 0; jj <= nn; jj++ )
            for ( kk = 0; kk <= oo; kk++ )
            {
                k1 = 1;

                for ( ii = mult + 1; ii <= mm; ii++ )
                    N_CopyCPt( Pw[ii][jj][kk], &Pw[k1++][jj][kk] );
            }
        mm -= mult;
    }

    /* Validate that UP[irr] multiplicity not greater than p+1 */
    mult = 0;

    while( ue EQ UP[irr - p - mult - 1] )
        mult += 1;

    if( mult GT 0 )
    {
        irr -= mult;

        for ( jj = 0; jj <= nn; jj++ )
            for ( kk = 0; kk <= oo; kk++ )
                N_CopyCPt( Pw[mm][jj][kk], &Pw[mm - mult][jj][kk] );
        mm -= mult;
    }

    /* Now the v-direction */

    /* Eliminate internal multiplicities greater than q */
    vs = VP[0];
    ve = VP[is];
    i = q + 1;

    while( VP[i]EQ VP[i - 1] )
        i += 1;

    while( VP[i]LT ve )
    {
        /* get multiplicity of VP[i] */

        mult = 1;

        while( VP[i]EQ VP[i + mult] )
            mult += 1;

        if( mult GT q )         /* reduce to multiplicity q */
        {
            k1 = i - 1;         /* first control point to replace */
            k2 = k1 + mult - q; /* last  control point to replace */
            k3 = k2 - k1;
            d = 1.0 / ((NL_REAL)k3 + (NL_REAL)1);

            for ( ii = 0; ii <= mm; ii++ )
                for ( kk = 0; kk <= oo; kk++ )
                {
                    for ( jj = k1 + 1; jj <= k2; jj++ )
                        N_Sum2CPts( Pw[ii][k1][kk], Pw[ii][jj][kk], &Pw[ii][k1][kk] );
                    N_ScaleCPt( d, Pw[ii][k1][kk], &Pw[ii][k1][kk] );
                    k4 = k1 + 1;

                    for ( jj = k2 + 1; jj <= nn; jj++ )
                        N_CopyCPt( Pw[ii][jj][kk], &Pw[ii][k4++][kk] );
                }

            nn -= k3;
            k1 = i + q;

            for ( jj = i + mult; jj <= iss; jj++ )
                VP[k1++] = VP[jj];
            iss -= k3;
            mult = q;
        }

        i = i + mult;
    }

    /* Validate that VP[0] multiplicity not greater than q+1 */
    mult = 0;

    while( vs EQ VP[q + mult + 1] )
        mult += 1;

    if( mult GT 0 )
    {
        k1 = q + 1;

        for ( jj = q + mult + 1; jj <= iss; jj++ )
            VP[k1++] = VP[jj];
        iss -= mult;

        for ( ii = 0; ii <= mm; ii++ )
            for ( kk = 0; kk <= oo; kk++ )
            {
                k1 = 1;

                for ( jj = mult + 1; jj <= nn; jj++ )
                    N_CopyCPt( Pw[ii][jj][kk], &Pw[ii][k1++][kk] );
            }
        nn -= mult;
    }

    /* Validate that VP[iss] multiplicity not greater than q+1 */
    mult = 0;

    while( ve EQ VP[iss - q - mult - 1] )
        mult += 1;

    if( mult GT 0 )
    {
        iss -= mult;

        for ( ii = 0; ii <= mm; ii++ )
            for ( kk = 0; kk <= oo; kk++ )
                N_CopyCPt( Pw[ii][nn][kk], &Pw[ii][nn - mult][kk] );
        nn -= mult;
    }

    /* Now the w-direction */

    /* Eliminate internal multiplicities greater than r */
    ws = WP[0];
    we = WP[it];
    i = r + 1;

    while( WP[i]EQ WP[i - 1] )
        i += 1;

    while( WP[i]LT we )
    {
        /* get multiplicity of WP[i] */

        mult = 1;

        while( WP[i]EQ WP[i + mult] )
            mult += 1;

        if( mult GT r )         /* reduce to multiplicity r */
        {
            k1 = i - 1;         /* first control point to replace */
            k2 = k1 + mult - r; /* last  control point to replace */
            k3 = k2 - k1;
            d = 1.0 / ((NL_REAL)k3 + (NL_REAL)1);

            for ( ii = 0; ii <= mm; ii++ )
                for ( jj = 0; jj <= nn; jj++ )
                {
                    for ( kk = k1 + 1; kk <= k2; kk++ )
                        N_Sum2CPts( Pw[ii][jj][k1], Pw[ii][jj][kk], &Pw[ii][jj][k1] );
                    N_ScaleCPt( d, Pw[ii][jj][k1], &Pw[ii][jj][k1] );
                    k4 = k1 + 1;

                    for ( kk = k2 + 1; kk <= nn; kk++ )
                        N_CopyCPt( Pw[ii][jj][kk], &Pw[ii][jj][k4++] );
                }

            oo -= k3;
            k1 = i + r;

            for ( jj = i + mult; jj <= itt; jj++ )
                WP[k1++] = WP[jj];
            itt -= k3;
            mult = r;
        }

        i = i + mult;
    }

    /* Validate that WP[0] multiplicity not greater than q+1 */
    mult = 0;

    while( ws EQ WP[r + mult + 1] )
        mult += 1;

    if( mult GT 0 )
    {
        k1 = r + 1;

        for ( kk = r + mult + 1; kk <= itt; kk++ )
            WP[k1++] = WP[kk];
        itt -= mult;

        for ( ii = 0; ii <= mm; ii++ )
            for ( jj = 0; jj <= nn; jj++ )
            {
                k1 = 1;

                for ( kk = mult + 1; kk <= nn; kk++ )
                    N_CopyCPt( Pw[ii][jj][kk], &Pw[ii][jj][k1++] );
            }
        oo -= mult;
    }

    /* Validate that WP[itt] multiplicity not greater than r+1 */
    mult = 0;

    while( we EQ WP[itt - r - mult - 1] )
        mult += 1;

    if( mult GT 0 )
    {
        itt -= mult;

        for ( ii = 0; ii <= mm; ii++ )
            for ( jj = 0; jj <= nn; jj++ )
                N_CopyCPt( Pw[ii][jj][oo], &Pw[ii][jj][oo - mult] );
        oo -= mult;
    }

    /* Compact the control point and knot arrays if required */

    if( mm LT m OR nn LT n OR oo LT o )
    {
        N_VolumeGetMeshAndKnotVectors( vol, &mesh, &p, &q, &r, &knu, &knv, &knw );
        N_KnotVectorFromRealArray( knu, UP, irr );
        N_KnotVectorFromRealArray( knv, VP, iss );
        N_KnotVectorFromRealArray( knw, WP, itt );
        N_CMeshFromCPts( mesh, Pw, mm, nn, oo );
        error = N_VolumeCompress( vol, S );

        if( error EQ NL_YES )
            NL_OUT;
    }

    /* End NURBS and Exit */

    EXIT:

    return (error);
} /* end ST_igsvsk */

#endif //NLIB_UNUSED
