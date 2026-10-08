// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* DATASTR.H: Include file for all data structures used in Nlib       */
/**********************************************************************/

#ifndef _DATASTR_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <errno.h>
//#include <NL_Defines.h>


/* When compiling for Debug - the following add various (read as slow) test */
/* sequentials to the normal flow of control. */
#ifdef GWC

#ifndef VSLIB
#define VSLIB 1
#endif

#ifndef SM_DEBUG_CODE
#define SM_DEBUG_CODE
#endif

#ifndef VSDEBUG      /* obsolete - replaced by VS_DEBUG_CODE */ 
#define VSDEBUG
#endif

#ifndef VS_DEBUG_CODE
#define VS_DEBUG_CODE
#endif

#ifndef SM_TOLERANT_WATCH
#define SM_TOLERANT_WATCH
#endif

#ifndef SM_VALIDATE_INTERSECTORS
#define SM_VALIDATE_INTERSECTORS
#endif

#ifndef SM_CLASSIFY_UPDATE
#define SM_VALIDATE_INTERSECTORS
#endif

#endif /* GWC */

#ifdef _UNICODE
 typedef wchar_t TCHAR;
#else /* no _UNICODE */
 typedef char TCHAR;
 #define _T(a) a
#endif /* no _UNICODE */ 

/* Define new data types */
typedef long NL_INTEGER;        /* Integer variable  */
typedef NL_INTEGER NL_INDEX;    /* Index variable    */
typedef long long NL_LLONG;     /* Long long variable*/
typedef short NL_FLAG;          /* Flag variable     */
typedef double NL_REAL;         /* Real variable     */
typedef const TCHAR *NL_STRING; /* String pointer    */
typedef NL_INTEGER NL_BOOLEAN;  /* Boolean type      */
typedef void NL_VOID;           /* Void type         */
// obsolete: typedef void *NL_MEMPOOL;       /* handle to SmartHeap memPools when compiled with NL_USING_SMARTHEAP */

#ifndef NL_VOID
#define NL_VOID void
#endif

/* Define machine dependent constants */
#ifndef NL_BIGD
  #define NL_BIGD  DBL_MAX     /* Largest double   */
#endif
#define NL_SMAD  DBL_MIN     /* Smallest double  */
#define NL_BIGI  INT_MAX     /* Largest integer  */
#define NL_SMAI  INT_MIN     /* Smallest integer */
#define NL_DEPS  DBL_EPSILON /* Double precision */

/* Define NL_PUBLIC and NL_PRIVATE */
#define NL_PUBLIC   extern /* External variable */
#define NL_PRIVATE      static             /* shared static: read-only NLib scratch */
#define NL_PRIVATE_TLS  static thread_local /* per-thread: mutable NLib globals (concurrency) */

/* Define some useful constants */
#define NL_TRUE               1
#define NL_FALSE              0

#define NL_YES                1
#define NL_NO                 0

#define NL_USED               1
#define NL_NOTUSED            0

#define NL_LEFT               1
#define NL_RIGHT              2
#define NL_BOTTOM             3
#define NL_TOP                4

#define NL_ONED               1
#define NL_TWOD               2
#define NL_THREED             3
#define NL_FOURD              4

#define NL_PLUS               1
#define NL_MINUS              2

#define NL_START              1
#define NL_END                2
#define NL_BOTH               3

#define NL_FLIP               1
#define NL_FLOP               2

/* the NL_xxDIR flags are a bit array so that NL_UVDIR = NL_UDIR + NL_VDIR etc. */
/*   note 1: don't reset these values                                                   */
/*   note 2: SM_SURFPARAM_TO_NLDIR(eSurfParam) converts SmSurfParamType to NL_DIR types */
#define NL_UDIR               1
#define NL_VDIR               2
#define NL_WDIR               4
#define NL_UVDIR              3
#define NL_UWDIR              5
#define NL_VWDIR              6
#define NL_UVWDIR             7

#define NL_XDIR               1
#define NL_YDIR               2
#define NL_ZDIR               3

#define NL_XCRD               1
#define NL_YCRD               2
#define NL_ZCRD               3

#define NL_PINCH              1
#define NL_TAPER              2
#define NL_TWIST              3
#define NL_SHEAR              4

#define NL_ELLIPSE            1
#define NL_HYPERBOLA          2
#define NL_PARABOLA           3

#define NL_LINEAR             1
#define NL_QUADRATIC          2
#define NL_CUBIC              3
#define NL_QUARTIC            4
#define NL_QUINTIC            5
#define NL_BILINEARCOONS      6
#define NL_BIQUADRATIC        7
#define NL_BICUBICCOONS       8

#define NL_BOUNDED            1
#define NL_UNBOUNDED          0

#define NL_PARALLEL           1
#define NL_PERSPECTIVE        2

#define NL_ADDITION           1
#define NL_SUBTRACTION        2
#define NL_MULTIPLICATION     3
#define NL_DIVISION           4

#define NL_EPOINT             1
#define NL_HPOINT             2
#define NL_RVALUE             3

#define NL_TANGENT            1
#define NL_DERIVATIVE         2

#define NL_PARAMETRIC         1
#define NL_PERPENDICULAR      2

#define NL_BESSEL             1
#define NL_AKIMA              2
#define NL_CIRCULAR           3

#define NL_CUSP               1
#define NL_NOCUSP             2

#define NL_SINGLE             1
#define NL_MULTIPLE           2

#define NL_C1                 1
#define NL_G1                 2
#define NL_G2                 3
#define NL_CMAX               4
#define NL_C2                 5
#define NL_G1R                6

#define NL_SMALL              1
#define NL_LARGE              2

#define NL_MINIMUM            1
#define NL_MAXIMUM            2
#define NL_AVERAGE            3

#define NL_NEW                1
#define NL_OLD                2

#define NL_UNIFORM            1
#define NL_CHORDLENGTH        2
#define NL_CENTRIPETAL        3
#define NL_INHERITED          4
#define NL_FUNCTION           5
#define NL_IGES               6
#define NL_NORMALIZED         7

#define NL_ABSOLUTE           1
#define NL_RELATIVE           2

#define NL_NODER              1
#define NL_ENDDER             2
#define NL_ALLDER             3

#define NL_PREPARE            1
#define NL_INTERACT           2
#define NL_CLEANUP            3

#define NL_TOUCHP             5
#define NL_TURNP              4
#define NL_ONANDOVER          3
#define NL_OVER               2
#define NL_INP                1
#define NL_ONP                0
#define NL_OUTP              -1
#define NL_NOSTATUS          -2
#define NL_EMPTY             -3

#define NL_NOTOUCH            0
#define NL_LEFTTOUCH          1
#define NL_RIGHTTOUCH         2
#define NL_TWOTOUCH           3
#define NL_REGULAR            4

#define NL_NOCORNER           0
#define NL_LOWERLEFT          1
#define NL_LOWERRIGHT         2
#define NL_UPPERLEFT          3
#define NL_UPPERRIGHT         4

#define NL_PARVALUE           3

#define NL_POINTS             1
#define NL_PARAMETERS         2

#define NL_FULL               1
#define NL_SPARSE             2

#define NL_SVD                1
#define NL_LUPIV              2

#define NL_NLINE              1
#define NL_NCIRCLE            2
#define NL_NPLANE             3
#define NL_NSPHERE            4
#define NL_NTORUS             5
#define NL_NCYLINDER          6
#define NL_NCONE              7
#define NL_NREVOLUTION        8
#define NL_NRULED             9
#define NL_NEXTRUSION        10
#define NL_NFREEFORM         11
#define NL_NONE              12

#define NL_PASSEDIN           1
#define NL_LOCALMETHOD        2
#define NL_BASECURVE          3

#define NL_SOLIDWORKS         1
#define NL_CLUSTERS           1

#define NL_HERMITE            1
#define NL_INTAPPR            2

#define NL_PI              3.1415926535897932
#define NL_RAD             0.0174532925199433
#define NL_LBIGD           log( NL_BIGD )
#define NL_LSMAD           log( NL_SMAD )
#define NL_EIA             NL_BIGI

/* Define logical operators */
#define LT   <
#define LE   <=
#define GT   >
#define GE   >=
#define EQ   ==
#define NEQ  !=

#define AND  &&
#define OR   ||
#define NOT  !

/* Define macros */
#ifndef NL_MIN
#define NL_MIN(x,y)     ( (x) < (y) ? (x) : ( y) )

#endif

#ifndef NL_MAX
#define NL_MAX(x,y)     ( (x) > (y) ? (x) : ( y) )

#endif

#ifndef NL_MIN3
#define NL_MIN3(x,y,z) (((x)<(y) && (x)<(z)) ? (x) : ((y)<(z)) ? (y) : (z))

#endif

#ifndef NL_MAX3
#define NL_MAX3(x,y,z) (((x)>(y) && (x)>(z)) ? (x) : ((y)>(z)) ? (y) : (z))

#endif

#define NL_SIGN(x)      ( (x) > 0.0 ? 1.0 : -1.0 )
#define NL_SIAB(a,b)    ( (b) >= 0.0 ? fabs(a) : -fabs(a) )

#define NL_QUIT        {  error = 1;  goto EXIT;  }
#define NL_OUT          goto EXIT

#define NL_ERROR(err)   {  N_ErrSet(err,rname);  NL_QUIT;  }
#define NL_PRINT_ERROR  {  N_PRINTF( _T("error = %d %s\n"), NL_ERROR.eno, NL_ERROR.fna ); exit(1);  }
#define NL_PAUSE        {  N_FPRINTF( stdout, _T("Hit any key to continue...\n") );  if(getchar() != EOF) {  ; } if(getchar() != EOF) {  ; } }

/* Define matrix structures */

typedef enum matrixtype {
    /* note: old names preserved for backward compatibility only */
    NL_MT_FULL       = 0, full       = 0, 
    NL_MT_LOWERLEFT  = 1, lowerleft  = 1, 
    NL_MT_UPPERRIGHT = 2, upperright = 2,  
    NL_MT_BANDED     = 3, banded     = 3
} NL_MATRIXTYPE;

typedef struct imatrix /* Integer matrix       */
{
NL_INDEX n,            /* Highest row index    */
m;                     /* Highest column index */
NL_INTEGER ** IM;      /* Pointer to matrix    */
NL_MATRIXTYPE mtp;     /* Matrix type          */
NL_INDEX bw;           /* Bandwidth            */
} NL_IMATRIX;

typedef struct rmatrix /* Real matrix          */
{
NL_INDEX n,            /* Highest row index    */
m;                     /* Highest column index */
NL_REAL ** RM;         /* Pointer to matrix    */
NL_MATRIXTYPE mtp;     /* Matrix type          */
NL_INDEX bw;           /* Bandwidth            */
} NL_RMATRIX;

/* Define stack nodes for memory allocation and deallocation */
typedef struct f1dnode {
NL_FLAG *ptr;
struct f1dnode *next;
} NL_F1DNODE;

typedef struct f2dnode {
NL_FLAG ** ptr;
struct f2dnode *next;
} NL_F2DNODE;

typedef struct i1dnode {
NL_INTEGER *ptr;
struct i1dnode *next;
} NL_I1DNODE;

typedef struct i2dnode {
NL_INTEGER ** ptr;
struct i2dnode *next;
} NL_I2DNODE;

typedef struct i3dnode {
NL_INTEGER *** ptr;
struct i3dnode *next;
} NL_I3DNODE;

typedef struct r1dnode {
NL_REAL *ptr;
struct r1dnode *next;
} NL_R1DNODE;

typedef struct r2dnode {
NL_REAL ** ptr;
struct r2dnode *next;
} NL_R2DNODE;

typedef struct r3dnode {
NL_REAL *** ptr;
struct r3dnode *next;
} NL_R3DNODE;

typedef struct r4dnode {
NL_REAL **** ptr;
struct r4dnode *next;
} NL_R4DNODE;

typedef struct imanode {
NL_IMATRIX *ptr;
struct imanode *next;
} NL_IMANODE;

typedef struct rmanode {
NL_RMATRIX *ptr;
struct rmanode *next;
} NL_RMANODE;

/* Define error structure */
typedef struct NL_ENODE {
NL_INTEGER eno;
NL_STRING fna;
} NL_ENODE;

/* Define error numbers */
#define NL_CUR_ERR  1  /* Curve definition error (e.g. n+p+1!=m)    */
#define NL_SUR_ERR  2  /* Surface definition error                  */
#define NL_DEG_ERR  3  /* Degree limit error (p>NL_MAXDEG)          */
#define NL_KNT_ERR  4  /* Knot vector error (e.g. u[i]>u[i-1])      */
#define NL_WEI_ERR  5  /* Weight error (negative or too small)      */
#define NL_PAR_ERR  6  /* Parameter value error (e.g. u<U[0])       */
#define NL_IND_ERR  7  /* Index range error (e.g. i<0)              */
#define NL_DER_ERR  8  /* Derivative value error (e.g. C'(u)=0.0)   */
#define NL_STO_ERR  9  /* Insufficient storage                      */
#define NL_MAT_ERR  10 /* Math function error (e.g. domain error)   */
#define NL_NUM_ERR  11 /* Numerical failure (e.g. zero division)    */
#define NL_CON_ERR  12 /* Convergence failure                       */
#define NL_CAL_ERR  13 /* Error in called routine                   */
#define NL_INP_ERR  14 /* Input data is corrupt                     */
#define NL_MEM_ERR  15 /* Memory allocation error                   */
#define NL_SEQ_ERR  16 /* System of equations error (e.g. singular) */
#define NL_GEO_ERR  17 /* Geometric operation failure               */
#define NL_FIL_ERR  18 /* Cannot open file                          */
#define NL_TOL_ERR  19 /* Tolerance error                           */
#define NL_MXD_ERR  20 /* Derivative limit error (der > NL_MAXDER)  */
#define NL_KML_ERR  21 /* Knot multiplicity error (kml > degree)    */
#define NL_CUR_REV  22 /* Curve end control points reverse direction*/
#define NL_SCN_ERR  23 /* N_FSCANF error                            */

/* Define assert function and macro for debug */

NL_VOID N_Assert( TCHAR *file_name, unsigned long line_num );

#ifdef NL_DEBUG_CODE
#define NL_ASSERT(a) { NL_BOOLEAN bBool = (a);                      \
                       if (!bBool) { N_Assert(__FILE__,__LINE__); } \
                     }

#else

#define NL_ASSERT(a)

#endif

#define _DATASTR_H_INCLUDED

#endif
