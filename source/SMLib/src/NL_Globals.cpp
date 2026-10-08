// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* NL_Globals.c: Include file for all data structures used in Nlib    */
/**********************************************************************/

#include "StdAfx.h"

#include <nurbs.h>
#include <NL_Globals.h>

/* Variables declared here are initialized in "NL_Globals.c"
   When running an application program that needs to reference
   these variables, then you may need to include the following file
   in your compile and link list
     file = ../../NLib/src/NL_Globals.c
  in your application, since these values need to be initialised
  otherwise they show as a missing external.
 */

/* Error number and test */

NL_ENODE NL_ERROR =
    {
    0, _T(" ")
    };

/* Some specific points and vectors */
NL_CPOINT NL_CZERO =
    {
    0.0, 0.0, 0.0, 0.0
    };

NL_POINT NL_ZERO =
    {
    0.0, 0.0, 0.0
    };

NL_VECTOR NL_UNITX =
    {
    1.0, 0.0, 0.0
    };

NL_VECTOR NL_UNITY =
    {
    0.0, 1.0, 0.0
    };

NL_VECTOR NL_UNITZ =
    {
    0.0, 0.0, 1.0
    };

NL_VECTOR NL_UNITV =
    {
    1.0, 1.0, 1.0
    };

NL_INTERVAL NL_UNITSPAN =
    {
    0.0, 1.0
    };

NL_RECTANGLE NL_UNITSQUARE =
    {
    0.0, 1.0, 0.0, 1.0
    };

NL_MINMAXBOX NL_UNITBOX =
    {
    0.0, 1.0, 0.0, 1.0, 0.0, 1.0
    };

/* Various tolerances */
NL_REAL NL_MTOL = 1.0e-07; /* Model space tolerance       */
NL_REAL NL_PTOL = 1.0e-12; /* Parameter space tolerance   */
NL_REAL NL_WTOL = 1.0e-03; /* Weight tolerance            */
NL_REAL NL_LTOL = 1.0e-09; /* Parallel line tolerance - gwc: angle in rad for parallel lines, SMLib uses 1e-4 was 1e-12    */
                           /*                                eventually this tolerance should be increased to 1e-4          */
                           /*                                I'd do it now if we weren't being conservative to minimize regressions */
NL_REAL NL_LUDT = 1.0e-09; /* LU-decomposition tolerance  */
NL_REAL NL_ZDTL = 1.0e-08; /* Zero discriminant tolerance */
NL_REAL NL_ZCTL = 1.0e-12; /* Zero coefficient tolerance  */

/* Various limits */
NL_REAL NL_WMIN = 1.0e-03; /* Minimum weight                      */
NL_REAL NL_WMAX = 100.0;   /* Maximum weight                      */

NL_DEGREE NL_DMAX = NL_MAXDEG;    /* 17 = Maximum degree.   This should  */
/* agree with NL_MAXDEG in nurbsdef.h  */
/* if using the fast evaluators  */
/* Keep it small (17) since it affects**2 fixed array sizes */

NL_INTEGER NL_ITLIM = 50;      /* Maximum number of iterations    */
NL_INTEGER NL_RECURSELIM = 20; /* Maximum number of recursion levels */
NL_INTEGER NL_CCPLIM = 1000;   /* Max number of curve control pts */


/* pascal's triangle - coefficients for derivatives using Liebnitz's rule */
/* usually NL_MAXDER+1 is enough, except for N_BezGetDegreeElevationMatrix  */
/* If NL_MAXDEG changes modify the NL_PascalTri initialization array to fit */
NL_INTEGER NL_PascalTri[18][18] =
    {
    { 1 },
    { 1, 1 },
    { 1, 2, 1 },
    { 1, 3, 3, 1 },
    { 1, 4, 6, 4, 1 },
    { 1, 5, 10, 10, 5, 1 },
    { 1, 6, 15, 20, 15, 6, 1 },
    { 1, 7, 21, 35, 35, 21, 7, 1 },
    { 1, 8, 28, 56, 70, 56, 28, 8, 1 },
    { 1, 9, 36, 84, 126, 126, 84, 36, 9, 1 },
    { 1, 10, 45, 120, 210, 252, 210, 120, 45, 10, 1 },
    { 1, 11, 55, 165, 330, 462, 462, 330, 165, 55, 11, 1 },
    { 1, 12, 66, 220, 495, 792, 924, 792, 495, 220, 66, 12, 1 },
    { 1, 13, 78, 286, 715, 1287, 1716, 1716, 1287, 715, 286, 78, 13, 1 },
    { 1, 14, 91, 364, 1001, 2002, 3003, 3432, 3003, 2002, 1001, 364, 91, 14, 1 },
    { 1, 15, 105, 455, 1365, 3003, 5005, 6435, 6435, 5005, 3003, 1365, 455, 105, 15, 1 },
    { 1, 16, 120, 560, 1820, 4368, 8008, 11440, 12870, 11440, 8008, 4368, 1820, 560, 120, 16, 1 },
    { 1, 17, 136, 680, 2380, 6188, 12376, 19448, 24310, 24310, 19448, 12376, 6188, 2380, 680, 136, 17, 1 }
    };

