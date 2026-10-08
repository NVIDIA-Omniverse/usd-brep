// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* GLOBALS.H: Include file for global constants used in Nlib          */
/**********************************************************************/

#ifndef _GLOBALS_H_INCLUDED
#define _GLOBALS_H_INCLUDED

/* Variables declared here are initialized in "NL_Globals.c"
   When running an application program that needs to reference
   these variables, then you may need to include the following file
   in your compile and link list
     file = ../../NLib/src/NL_Globals.c
  in your application, since these values need to be initialised
  otherwise they show as a missing external.
 */

/*
 * Note: If global variables are modified in the course of an algorithm,
 * then parallel execution will not be possible.  For parallel processing,
 * any globals must be read-only.
 */

/* Error handle */

/* Set to TRUE if object code license            */
/* NL_PUBLIC NL_BOOLEAN NL_TEST_LICENSE = FALSE; */

NL_PUBLIC NL_ENODE NL_ERROR; /* { 0, " " };              */

/* Some specific points and vectors */
NL_PUBLIC NL_CPOINT NL_CZERO;         /* { 0.0, 0.0, 0.0, 0.0 }            */
NL_PUBLIC NL_POINT NL_ZERO;           /* { 0.0, 0.0, 0.0 }                 */
NL_PUBLIC NL_VECTOR NL_UNITX;         /* { 1.0, 0.0, 0.0 }                 */
NL_PUBLIC NL_VECTOR NL_UNITY;         /* { 0.0, 1.0, 0.0 }                 */
NL_PUBLIC NL_VECTOR NL_UNITZ;         /* { 0.0, 0.0, 1.0 }                 */
NL_PUBLIC NL_VECTOR NL_UNITV;         /* { 1.0, 1.0, 1.0 }                 */
NL_PUBLIC NL_INTERVAL NL_UNITSPAN;    /* { 0.0, 1.0 }                      */
NL_PUBLIC NL_RECTANGLE NL_UNITSQUARE; /* { 0.0, 1.0, 0.0, 1.0 }            */
NL_PUBLIC NL_MINMAXBOX NL_UNITBOX;    /* { 0.0, 1.0, 0.0, 1.0, 0.0, 1.0 }  */

/* Various tolerances */
NL_PUBLIC NL_REAL NL_MTOL; /* 1.0e-07   Model space tolerance       */
NL_PUBLIC NL_REAL NL_PTOL; /* 1.0e-12   Parameter space tolerance   */
NL_PUBLIC NL_REAL NL_WTOL; /* 1.0e-03   Weight tolerance            */
NL_PUBLIC NL_REAL NL_LTOL; /* 1.0e-12   Parallel line tolerance     - gwc: changed to 1.oe-9, will be pushed to 1.0e-4 eventually */
NL_PUBLIC NL_REAL NL_LUDT; /* 1.0e-09   LU-decomposition tolerance  */
NL_PUBLIC NL_REAL NL_ZDTL; /* 1.0e-08   Zero discriminant tolerance */
NL_PUBLIC NL_REAL NL_ZCTL; /* 1.0e-12   Zero coefficient tolerance  */

/* Various limits */
NL_PUBLIC NL_REAL NL_WMIN;   /* 1.0e-03   Minimum weight                  */
NL_PUBLIC NL_REAL NL_WMAX;   /* 100.0     Maximum weight                  */

NL_PUBLIC NL_DEGREE NL_DMAX; /* 16        Maximum degree.   This should   */
/*                    agree with NL_MAXDEG in nurbsdef.h */
/*                    if using the fast evaluators    */

NL_PUBLIC NL_INTEGER NL_ITLIM;      /* 50        Maximum number of iterations    */
NL_PUBLIC NL_INTEGER NL_RECURSELIM; /* 20        Maximum levels of recursion     */
NL_PUBLIC NL_INTEGER NL_CCPLIM;     /* 1000      Max number of curve control pts */
NL_PUBLIC NL_INTEGER NL_SCPLIM;     /* 10000     Max number of surf control pts  */

/* pascal's triangle - coefficients for derivatives using Liebnitz's rule*/
NL_PUBLIC NL_INTEGER NL_PascalTri[18][18]; /* changefrom 5,5 to 18,18  [NL_MAXDER+1][NL_MAXDER+1] ; */

/* tessellation parameters for ST_IsectLineWith2dPolygon.c, ST_TessPolygonizeTrimCrv.c, ST_TessSubdivideSrf.c */
/* NOTE: no longer using these, they prevent parallel processing from working. */
/* Now passing parameters instead. */
/* NL_PUBLIC NL_REAL NL_TES_UMIN; */ /* 0.0 */
/* NL_PUBLIC NL_REAL NL_TES_UMAX; */ /* 1.0 */
/* NL_PUBLIC NL_REAL NL_TES_VMIN; */ /* 0.0 */
/* NL_PUBLIC NL_REAL NL_TES_VMAX; */ /* 1.0 */
/* NL_PUBLIC NL_REAL NL_TES_AREA; */ /* 0.0 */
/* NL_PUBLIC NL_REAL NL_TES_LENU; */ /* 0.0 */
/* NL_PUBLIC NL_REAL NL_TES_LENV; */ /* 0.0 */

/* Inlining control */
#if defined(_MSC_VER)
#    define NL_INLINE __forceinline
#else
#    define NL_INLINE inline __attribute__((always_inline))
#endif

#if defined(_MSC_VER)
#    define NL_NOINLINE __declspec(noinline)
#else
#    define NL_NOINLINE __attribute__((noinline))
#endif

#endif
