// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************/
/* NURBS.H: Top level include file with all function prototypes       */
/**********************************************************************/

/* Check for inclusion */

#ifndef _NURBS_H_INCLUDED
#define _NURBS_H_INCLUDED
#define NURBS

/* Include definitions */

#include <NL_Nurbsdef.h>

#ifdef _WIN32
 #define GW_EXPORT   __declspec( dllexport )
#elif defined(__GNUC__) && __GNUC__ >= 4
 #define GW_EXPORT __attribute__((visibility("default")))
#else
 #define GW_EXPORT
#endif

#ifdef _UNICODE
 #include <tchar.h>
#endif


#ifdef _UNICODE
 #define FILE_NAME _T(__FILE__)
 #define N_PRINTF(a) _tprintf(a);
 #define N_SPRINTF _stprintf
 #define N_FOPEN _wfopen
 #define N_FROPEN _tfreopen
 #define N_FSCANF  _ftscanf
 #define N_FPRINTF _ftprintf
 #define N_REMOVE _wremove
#else  /* no _UNICODE */
 #define FILE_NAME __FILE__
 #define N_PRINTF(a) { int cnt = printf(a); SM_ASSERT(cnt < SM_TBLOCK_SIZE && cnt > 0) ; }
 #define N_SPRINTF   sprintf
 #define N_SSCANF    sscanf      /* <=== need to check rtn count and make sure it's smaller than SM_TBLOCK_SIZE */
 #define N_FOPEN     fopen       /*   SM_TBLOCK_SIZE and VS_TBLOCK_SIZE need to be larger, maybe 2048 */
 #define N_FROPEN    freopen
 #define N_FPRINTF   fprintf
 #define N_FSCANF    fscanf
 #define N_REMOVE    remove
#endif /* no _UNICODE */

/******************************/
/* Memory allocation routines */
/******************************/

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Optionally include this if your application is using short NLib names */
/* This will automatically convert to new long names */
#include <NL_Defines2.h>

/* begin function declarations */

#include <NL_Util.h>        /* NLib utility  functions  */
#include <NL_Math.h>        /* NLib math     functions  */
#include <NL_Geom.h>        /* NLib geometry functions  */
#include <NL_Bezier.h>      /* NLib bezier   functions  */
#include <NL_Iges.h>        /* NLib NL_IGES     functions  */

#include <NL_BasisBasic.h>  /* Basic    NL_KNOTVECTOR functions */
#include <NL_FuncsBasic.h>  /* Basic    NL_CFUN, NL_CVALUE, NL_SFUN, and NL_SVALUE functions */
#include <NL_FrameBasic.h>  /* Basic    NL_CPOLYGON, NL_EPOLYGON, NL_CNET, and NL_ENET functions */

#include <NL_CrvBasic.h>    /* Basic               NL_CURVE functions */
#include <NL_CrvNurb.h>     /* NURB                NL_CURVE functions */
#include <NL_CrvPoly.h>     /* polynomial          NL_CURVE functions */
#include <NL_CrvTool.h>     /* Tool                NL_CURVE functions */
#include <NL_CrvGeom.h>     /* Geometry Processing NL_CURVE functions */
#include <NL_CrvCommon.h>   /* Common Curve        NL_CURVE functions */
#include <NL_CrvFit.h>      /* Curve Fitting       NL_CURVE functions */
#include <NL_CrvApprox.h>   /* Curve Approximation NL_CURVE functions */
#include <NL_CrvOffset.h>   /* Curve Offset        NL_CURVE functions */
#include <NL_CrvShape.h>    /* Curve Shape         NL_CURVE functions */

#include <NL_SrfBasic.h>    /* Basic               NL_SURFACE functions */
#include <NL_SrfNurb.h>     /* NURB                NL_SURFACE functions */
#include <NL_SrfPoly.h>     /* polynomial          NL_SURFACE functions */
#include <NL_SrfTool.h>     /* Tool                NL_SURFACE functions */
#include <NL_SrfGeom.h>     /* Geometry Processing NL_SURFACE functions */
#include <NL_SrfCommon.h>   /* Common Curve        NL_SURFACE functions */
#include <NL_SrfSymbol.h>   /* Symbolic Operator   NL_SURFACE functions */
#include <NL_SrfFit.h>      /* Surface Fitting     NL_SURFACE functions */
#include <NL_SrfLstSqFit.h> /* Surface Fitting     NL_SURFACE functions */
#include <NL_SrfApprox.h>   /* Surface Approximation NL_SURFACE functions */
#include <NL_SrfShape.h>    /* Surface Shape         NL_SURFACE functions */

#include <NL_BasisAdv.h>    /* Advanced NL_KNOTVECTOR functions */
#include <NL_CrvAdv.h>      /* Advanced NL_CURVE functions      */
#include <NL_SrfAdv.h>      /* Advanced NL_SURFACE functions    */
#include <NL_VolumeAdv.h>   /* Advanced NL_VOLUME functions     */
#include <NL_FuncsAdv.h>    /* Advanced NL_CFUN, NL_CVALUE, NL_SFUN, NL_SVALUE, NL_VFUN, and NL_VVALUE functions */
#include <NL_FrameAdv.h>    /* Advanced NL_CPOLYGON, NL_EPOLYGON, NL_CNET, NL_ENET, NL_CMESH, and NL_EMESH functions */
#include <NL_Tessellate.h>  /* Tessellation functions */
#include <NL_Spiral.h>      /* Spiral functions */

#ifdef __cplusplus
}
#endif /* __cplusplus */

/* Exit */

#endif /* _NURBS_H_INCLUDED */
