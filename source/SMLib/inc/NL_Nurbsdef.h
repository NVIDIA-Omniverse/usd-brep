// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* NURBSDEF.H: Include file for all NURBS definitions                 */
/**********************************************************************/

#ifndef _NURBSDEF_H_INCLUDED
#define _NURBSDEF_H_INCLUDED

#include <NL_DataStruct.h>
#include <NL_Geometry.h>

#ifdef VS8
/* this eliminates compiler warning using Visual Studio 2005 V8 */
/* Visual Studio v6 doesn't like fscanf_s */
#define fscanf  fscanf_s

#endif

/* These are used only for the fast evaluators. */
/* NL_MAXDEG should agree with NL_DMAX in globals.h   */
/* Keep these small since they control array sizes used in evaluators */
/* Catia users need to go over 10 */
/* Evaluations need NL_MAGDEG+1 array sizes */
/* Basis Functions need NL_MAXDEG+2 array sizes */
/* See also NL_Globals.c AND globals.h */
#define NL_MAXDEG   17 /* 28 maximum degree */
#define NL_MAXDER   4  /*  5 maximum derivative order */
/*  If NL_MAXDER changes modify the NL_PascalTri initialization array to fit */

/* Define data types for basis function and derivative evaluation */
typedef NL_REAL NL_BASISFUNCTIONS[NL_MAXDEG + 1][NL_MAXDEG + 1];
typedef NL_REAL NL_DERIVATIVES[NL_MAXDER + 1][NL_MAXDEG + 1];

/* Struct to keep Basisderivates for worst case size */
/* The struct as wrapper around the 4 dimensional array is required to be able to pass */
/* around basis derivatives as reference to functions */
struct NL_BASISDERIVATIVES
{
    using BD1 = NL_REAL[NL_MAXDER + 1][NL_MAXDEG + 1][NL_MAXDEG + 1];

    /* Accessor to the outermost dimension so that this struct behaves like a 4D array */
    BD1& operator[](size_t idx)
    {
        return BD[idx];
    }

    /* Accessor to the outermost dimension so that this struct behaves like a 4D array */
    const BD1& operator[](size_t idx) const
    {
        return BD[idx];
    }

    NL_REAL BD[NL_MAXDER + 1][NL_MAXDER + 1][NL_MAXDEG + 1][NL_MAXDEG + 1];
};

/* Define NURBS specific data types */
typedef short NL_DEGREE;
typedef NL_REAL NL_PARAMETER;

/* Define knot vector object */
typedef struct knotvector 
{
  NL_INDEX      m; /* max knot index in U array */
  NL_REAL     * U; /* U sized:[m+1]             */
} NL_KNOTVECTOR;

/* Define curve object */
typedef struct curve 
{
  NL_CPOLYGON   * pol;
  NL_DEGREE       p;
  NL_KNOTVECTOR * knt;
} NL_CURVE;

/* Define surface object */
typedef struct surface 
{
  NL_CNET       * net;
  NL_DEGREE       p;
  NL_DEGREE       q;
  NL_KNOTVECTOR * knu;
  NL_KNOTVECTOR * knv;
} NL_SURFACE;

/* Define volume object */
typedef struct volume 
{
  NL_CMESH      * mesh;
  NL_DEGREE       p;
  NL_DEGREE       q;
  NL_DEGREE       r;
  NL_KNOTVECTOR * knu; 
  NL_KNOTVECTOR * knv; 
  NL_KNOTVECTOR * knw;
} NL_VOLUME;

/* Define curve value object */
typedef struct cvalue 
{
  NL_INDEX        n;
  NL_REAL       * fu;
} NL_CVALUE;

/* Define curve function object */
typedef struct cfun 
{
  NL_CVALUE     * cvl;
  NL_DEGREE       p;
  NL_KNOTVECTOR * knt;
} NL_CFUN;

/* Define surface value object */
typedef struct svalue 
{
  NL_INDEX        n;
  NL_INDEX        m;
  NL_REAL      ** fuv;
} NL_SVALUE;

/* Define surface function object */
typedef struct sfun 
{
  NL_SVALUE     * svl;
  NL_DEGREE       p; 
  NL_DEGREE       q;
  NL_KNOTVECTOR * knu; 
  NL_KNOTVECTOR * knv;
} NL_SFUN;

/* Define volume value object */
typedef struct vvalue 
{
  NL_INDEX        n; 
  NL_INDEX        m;   
  NL_INDEX        o;
  NL_REAL     *** fuvw;
} NL_VVALUE;

/* Define volume function object */
typedef struct vfun 
{
  NL_VVALUE     * vvl;
  NL_DEGREE       p; 
  NL_DEGREE       q; 
  NL_DEGREE       r;
  NL_KNOTVECTOR * knu; 
  NL_KNOTVECTOR * knv; 
  NL_KNOTVECTOR * knw;
} NL_VFUN;

/* Define interval object */
typedef struct interval 
{
  NL_PARAMETER    ul; 
  NL_PARAMETER    ur;
} NL_INTERVAL;

/* Define rectangle object */
typedef struct rectangle 
{
  NL_PARAMETER ul; 
  NL_PARAMETER ur; 
  NL_PARAMETER vb; 
  NL_PARAMETER vt;
} NL_RECTANGLE;

/* Define rectangular solid */
typedef struct minmaxbox 
{ /* MinPt = [xl, yb, zn] */
  /* MaxPt = [xr, yt, zf] */
  NL_REAL xl;          /* l = left,   r = right */
  NL_REAL xr;          
  NL_REAL yb;          /* b = bottom, t = top */
  NL_REAL yt;          
  NL_REAL zn;          /* n = near,   f = far */
  NL_REAL zf;          
} NL_MINMAXBOX;

/************************************************************************/
/* A struct for working with curve evaluations.
 * A "jet" is a function evaluation plus derivatives.
 * This struct encapsulates that for a point on a curve,
 * plus a couple of conveniences.
 *
 * This struct works with the following routines:
 *  N_crvJetInit()
 *  N_crvJetUnset()
 *  N_crvJetReset()
 *  N_crvJetSetParam()
 *  N_crvJetPos()
 *  N_crvJetDer1()
 *  N_crvJetDer2()
 *  N_crvJetPosCopy()
 *  N_crvJetDer1Copy()
 *  N_crvJetDer2Copy()
 *  N_crvJetCurvature()
 *  N_crvJetRelax()
 */
/************************************************************************/
typedef struct curveJet 
{
#define CURVEJET__MAX_DERIVS 2

  NL_CURVE * myCurve;
  NL_INDEX   numEval; /*  n: nth deriv is set;
                          0: pt (only) is set;
                         -1: param is set to a valid value, but not evaluated;
                         -2: nothing is set, not even param.
                      */

  NL_REAL    param;   /* Current parameter value, if set. */

  /* Point plus derivatives, if set. */
  /* Note: +1 because of how N_CrvDerivs() works... */
  NL_POINT   derivs[CURVEJET__MAX_DERIVS + 1];

  NL_INDEX MAX_DERIVS; /* == CURVEJET__MAX_DERIVS */
} NL_CURVEJET;

/************************************************************************/

/************************************************************************/
/* An analogous struct for working with surface evaluations.
 * A "jet" is a function evaluation plus derivatives.
 * This struct encapsulates that for a point on a surface,
 * plus a couple of conveniences.
 *
 * This struct works with the following routines:
 *  N_surfJetInit()
 *  N_surfJetUnset()
 *  N_surfJetReset()
 *  N_surfJetSetParam()
 *  N_surfJetPos()
 *  N_surfJetDer_u()
 *  N_surfJetDer_v()
 *  N_surfJetDer_uu()
 *  N_surfJetDer_uv()
 *  N_surfJetDer_vv()
 *  N_surfJetPosCopy()
 *  N_surfJetDeruCopy()
 *  N_surfJetDervCopy()
 *  N_surfJetDeruuCopy()
 *  N_surfJetDeruvCopy()
 *  N_surfJetDervvCopy()
 *  N_surfJetCurvature()
 *  N_surfJetRelax()
 */
/************************************************************************/
typedef struct surfJet 
{
#define SURFJET__MAX_DERIVS 2

  NL_SURFACE   * mySurf;
  NL_INDEX       numEval; /*  n: nth derivs are set;
                              0: pt (only) is set;
                             -1: u and v are set to a valid value, but not evaluated;
                             -2: nothing is set, not even u, v.

                             Note: the numbers of u- and v-derivs set are always equal:
                              1: d_u and d_v set
                              2: d_uu, d_uv and d_vv set
                          */

  NL_REAL        u;       /* Current parameter values, if set. */
  NL_REAL        v; 

  /* Point plus derivatives, if set. */
  /* cbi NL_POINT derivs[ SURFJET__MAX_DERIVS+1 ][ SURFJET__MAX_DERIVS+1 ]; */
  NL_POINT     * derivs[SURFJET__MAX_DERIVS + 1];
  NL_POINT       der0  [SURFJET__MAX_DERIVS + 1];
  NL_POINT       der1  [SURFJET__MAX_DERIVS + 1];
  NL_POINT       der2  [SURFJET__MAX_DERIVS + 1];

  NL_INDEX MAX_DERIVS; /* == CURVEJET__MAX_DERIVS */
} NL_SURFJET;

/* Define nodes for memory allocation */
typedef struct curnode 
{
  NL_CURVE         * ptr;
  struct curnode   * next;
} NL_CURNODE;

typedef struct cu2node 
{               
  NL_CURVE       ** ptr;
  struct cu2node  * next;
} NL_CU2NODE;

typedef struct cu3node 
{
  NL_CURVE      *** ptr;
  struct cu3node  * next;
} NL_CU3NODE;

typedef struct cu4node 
{
  NL_CURVE    **** ptr;
  struct cu4node * next;
} NL_CU4NODE;

typedef struct surnode 
{
  NL_SURFACE     * ptr;
  struct surnode * next;
} NL_SURNODE;

typedef struct su2node 
{
  NL_SURFACE    ** ptr;
  struct su2node * next;
} NL_SU2NODE;

typedef struct su3node 
{
  NL_SURFACE   *** ptr;
  struct su3node * next;
} NL_SU3NODE;

typedef struct volnode 
{
  NL_VOLUME      * ptr;
  struct volnode * next;
} NL_VOLNODE;

typedef struct kntnode 
{
  NL_KNOTVECTOR  * ptr;
  struct kntnode * next;
} NL_KNTNODE;

typedef struct kn2node 
{
  NL_KNOTVECTOR ** ptr;
  struct kn2node * next;
} NL_KN2NODE;

typedef struct cvlnode 
{
  NL_CVALUE      * ptr;
  struct cvlnode * next;
} NL_CVLNODE;

typedef struct svlnode 
{
  NL_SVALUE      * ptr;
  struct svlnode * next;
} NL_SVLNODE;

typedef struct vvlnode 
{
  NL_VVALUE      * ptr;
  struct vvlnode * next;
} NL_VVLNODE;

typedef struct cfnnode 
{
  NL_CFUN        * ptr;
  struct cfnnode * next;
} NL_CFNNODE;

typedef struct cf2node 
{
  NL_CFUN       ** ptr;
  struct cf2node * next;
} NL_CF2NODE;

typedef struct sfnnode 
{
  NL_SFUN        * ptr;
  struct sfnnode * next;
} NL_SFNNODE;

typedef struct sf2node 
{
  NL_SFUN       ** ptr;
  struct sf2node * next;
} NL_SF2NODE;

typedef struct sf3node 
{
  NL_SFUN      *** ptr;
  struct sf3node * next;
} NL_SF3NODE;

/* Define stacks for memory allocation and deallocation */
typedef struct stacks 
{
  NL_F1DNODE  * f1d;
  NL_F2DNODE  * f2d;
  NL_I1DNODE  * i1d;
  NL_I2DNODE  * i2d;
  NL_I3DNODE  * i3d;
  NL_R1DNODE  * r1d;
  NL_R2DNODE  * r2d;
  NL_R3DNODE  * r3d;
  NL_R4DNODE  * r4d;
  NL_P1DNODE  * p1d;
  NL_P2DNODE  * p2d;
  NL_P3DNODE  * p3d;
  NL_P4DNODE  * p4d;
  NL_C1DNODE  * c1d;
  NL_C2DNODE  * c2d;
  NL_C3DNODE  * c3d;
  NL_C4DNODE  * c4d;
  NL_CURNODE  * cur;
  NL_CU2NODE  * cu2;
  NL_CU3NODE  * cu3;
  NL_CU4NODE  * cu4;
  NL_SURNODE  * sur;
  NL_VOLNODE  * vol;
  NL_SU2NODE  * su2;
  NL_SU3NODE  * su3;
  NL_POLNODE  * pol;
  NL_PPLNODE  * ppl;
  NL_PP2NODE  * pp2;
  NL_NETNODE  * net;
  NL_MESHNODE * msh;
  NL_KNTNODE  * knt;
  NL_KN2NODE  * kn2;
  NL_CVLNODE  * cvl;
  NL_SVLNODE  * svl;
  NL_VVLNODE  * vvl;
  NL_CFNNODE  * cfn;
  NL_CF2NODE  * cf2;
  NL_SFNNODE  * sfn;
  NL_SF2NODE  * sf2;
  NL_SF3NODE  * sf3;
  NL_IMANODE  * ima;
  NL_RMANODE  * rma;
  NL_PMANODE  * pma;
  NL_CMANODE  * cma;
} NL_STACKS;  

#endif /* No _NURBSDEF_H_INCLUDED */

