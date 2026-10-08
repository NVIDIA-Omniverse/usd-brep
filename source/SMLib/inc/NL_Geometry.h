// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* GEOMETRY.H: Include file for all geometry structures used in Nlib  */
/**********************************************************************/

#ifndef _GEOMETRY_H_INCLUDED
#define _GEOMETRY_H_INCLUDED

#include <NL_DataStruct.h>

/* Define control point and control net objects */

typedef struct cpoint   /* Control point */
{ 
  NL_REAL       x;                                                                                         
  NL_REAL       y;                                                                                         
  NL_REAL       z;        /* use NL_NOZ for 2d points           */                                               
  NL_REAL       w;        /* use NL_NOW for non-rational points. Where Cartesian [XYZ] = Homogeneous [X/w Y/w Z/w] */
} NL_CPOINT;

typedef struct cpolygon /* Control polygon */
{ 
  NL_INDEX      n;        /* max index in Pw */
  NL_CPOINT   * Pw;       /* Pw sized:[n+1]  */
} NL_CPOLYGON;

typedef struct cnet     /* Control net */
{ 
  NL_INDEX      n;        /* max 1st index in Pw  */
  NL_INDEX      m;        /* max 2nd index in Pw  */
  NL_CPOINT  ** Pw;       /* Pw sized:[n+1][m+1]  */
} NL_CNET;

typedef struct cmesh    /* Control mesh */
{ 
  NL_INDEX      m;        /* max 1st index in Pw       */
  NL_INDEX      n;        /* max 2nd index in Pw       */
  NL_INDEX      o;        /* max 3rd index in Pw       */
  NL_CPOINT *** Pw;       /* Pw sized:[m+1][n+1][o+1]  */
} NL_CMESH;

/* Define some useful constants */
#ifndef NL_NOZ
  #define NL_NOZ       +NL_BIGD /* No z-component (2D case)           */
#endif 
#ifndef NL_NOW
  #define NL_NOW       -NL_BIGD /* No w-component (non-rational case) */
#endif
#define NL_INFINITE    +NL_BIGD
#define NL_UNDEFINED   -NL_BIGD

/* Objects for Euclidean geometry */

typedef struct nl_point 
{ 
  NL_REAL      x;
  NL_REAL      y;
  NL_REAL      z;
} NL_POINT;

typedef NL_POINT NL_VECTOR;
typedef NL_CPOINT NL_CVECTOR;

typedef struct lineseg 
{ 
  NL_POINT     P;         /* Start point                                */
  NL_VECTOR    V;         /* Vector with direction and magnitude        */
  NL_FLAG   bounded;      /* Flag: NL_FALSE = not bounded, NL_TRUE = bounded  */
} NL_LINESEG;

typedef struct plane 
{ 
  NL_POINT     P;         /* Any point    */
  NL_VECTOR    N;         /* Plane normal */
} NL_PLANE;

typedef struct epolygon /* Euclidean polygon */
{ 
  NL_INDEX     n;
  NL_POINT   * P;
} NL_EPOLYGON;

typedef struct enet  /* Euclidean net */
{ 
  NL_INDEX     n,     /* max 1st index in P   */
  m;                  /* max 2nd index in P   */
  NL_POINT  ** P;     /* p sized:[n+1][m+1]   */
} NL_ENET;

typedef struct emesh /* Control mesh */
{ 
  NL_INDEX     m;     /* max 1st index in P      */
  NL_INDEX     n;     /* max 2nd index in P      */
  NL_INDEX     o;     /* max 3rd index in P      */
  NL_POINT *** P;     /* p sized:[n+1][m+1][o+1] */
} NL_EMESH;

/* Define temporary structure for global curve projection */
typedef struct gcptemp 
{ 
  NL_INDEX     nks;  /* number of knot spans in curve                */
  NL_INDEX     nip;  /* number of points interior to each span       */
  NL_REAL  *** box;  /* minmax boxes of each span                    */
  NL_REAL    * us;   /* start parameter of each span                 */
  NL_POINT   * cog;  /* center of gravity of ctrl pts of each span   */
  NL_POINT   * pts;  /* points on curve covering all spans           */
  NL_FLAG    * pflg; /* indicates if points are available for a span */
} NL_GCPTEMP;

/* Point and control point matrices */
typedef struct pmatrix /* Point matrix         */
{ 
  NL_INDEX      n;       /* Highest row index      */
  NL_INDEX      m;       /* Highest column index   */
  NL_POINT   ** PM;      /* PM sized:[n+1][m+1]    */
  NL_MATRIXTYPE mtp;     /* Matrix type            */
  NL_INDEX      bw;      /* Bandwidth              */
} NL_PMATRIX;

typedef struct cmatrix /* Control point matrix */
{ 
  NL_INDEX      n;       /* Highest row index      */
  NL_INDEX      m;       /* Highest column index   */
  NL_CPOINT  ** CM;      /* CM sized:[n+1][m+1]    */
  NL_MATRIXTYPE mtp;     /* Matrix type            */
  NL_INDEX      bw;      /* Bandwidth              */
} NL_CMATRIX;

/* Define stack nodes for memory allocation and deallocation */
typedef struct p1dnode 
{ 
  NL_POINT       * ptr;
  struct p1dnode * next;
} NL_P1DNODE;

typedef struct p2dnode 
{ 
  NL_POINT      ** ptr;
  struct p2dnode * next;
} NL_P2DNODE;

typedef struct p3dnode 
{ 
  NL_POINT     *** ptr;
  struct p3dnode * next;
} NL_P3DNODE;

typedef struct p4dnode 
{ 
  NL_POINT    **** ptr;
  struct p4dnode * next;
} NL_P4DNODE;

typedef struct c1dnode 
{ 
  NL_CPOINT      * ptr;
  struct c1dnode * next;
} NL_C1DNODE;

typedef struct c2dnode 
{ 
  NL_CPOINT     ** ptr;
  struct c2dnode * next;
} NL_C2DNODE;

typedef struct c3dnode 
{ 
  NL_CPOINT    *** ptr;
  struct c3dnode * next;
} NL_C3DNODE;

typedef struct c4dnode 
{ 
  NL_CPOINT   **** ptr;
  struct c4dnode * next;
} NL_C4DNODE;

typedef struct polnode 
{ 
  NL_CPOLYGON    * ptr;
  struct polnode * next;
} NL_POLNODE;

typedef struct pplnode 
{ 
  NL_EPOLYGON    * ptr;
  struct pplnode * next;
} NL_PPLNODE;

typedef struct pp2node 
{ 
  NL_EPOLYGON   ** ptr;
  struct pp2node * next;
} NL_PP2NODE;

typedef struct netnode 
{ 
  NL_CNET        * ptr;
  struct netnode * next;
} NL_NETNODE;

typedef struct meshnode 
{ 
  NL_CMESH        * ptr;
  struct meshnode * next;
} NL_MESHNODE;

typedef struct pmanode 
{ 
  NL_PMATRIX     * ptr;
  struct pmanode * next;
} NL_PMANODE;

typedef struct cmanode 
{ 
  NL_CMATRIX     * ptr;
  struct cmanode * next;
} NL_CMANODE;

#endif /* _GEOMETRY_H_INCLUDED */ 
