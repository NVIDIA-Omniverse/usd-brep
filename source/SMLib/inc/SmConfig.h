// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmConfig.h
* PURPOSE: File to run various configurations for the software.
**********************************************************************/

#ifndef __SMOS_CONFIG_H__
#define __SMOS_CONFIG_H__


// With 8.5.0 we move to version number with date 990mmddyy
#define I_W_VERSION_NUMBER 990100226
// SM_CURRENT_DATABASE_VERSION 30 = Persistence of Breps, SmBSplineCurve, SmBSplineSurfaces
// SM_CURRENT_DATABASE_VERSION 32 = Added Persistent attributes
// SM_CURRENT_DATABASE_VERSION 34 = Added Persistence to all SmCurve, SmSurface, and SmVolume derived types
// SM_CURRENT_DATABASE_VERSION 36 = Modified SmProjCurve member definitions
// SM_CURRENT_DATABASE_VERSION 38 = Added m_dMaxGap3d member definitions to SmFace, SmEdge, and SmVertex
// SM_CURRENT_DATABASE_VERSION 40 = Improved compatibility of binary files between 64 bit Unix and Windows systems
// SM_CURRENT_DATABASE_VERSION 42 = Introduces formal healing upon import and adds SM_HEALER_VERSION to files 
// SM_CURRENT_DATABASE_VERSION 44 = Upcoming NewTopologyModel Changes - not yet used
#define SM_CURRENT_DATABASE_VERSION 42

// SM_HEALER_VERSION 10 = Introduction of the formal healer, used by default on import
#define SM_HEALER_VERSION 10

// Users can use this string to identify Smlib version
// 8.11.x refers to the official release (no healer)
// 8.12.0.x refers to a healer release
#define SM_VERSION_STRING _T("10.0.0")
#define SM_VERSION_NUMBER 100000
#define SM_VERSION_DATE "10/02/26"

// Typical user compile options are:
//#define SM_NO_DEBUG_CODE
//#define SM_NO_GFX_OUTPUT_CODE

#ifdef _WIN32
#define SM_THREAD_LOCAL __declspec(thread)
#else
#define SM_THREAD_LOCAL thread_local
#endif

// This define allows us to identify code for the NLib tessellator
// We can use it if necessary or pull it out if we woul dlike ot decreaase the size of smlib
#define USE_NLIB_TESS

// Tessellation changes: mesh refinement at boundaries
//#define SM_USE_TESS_REFINE_MESH

// Tessellation changes: corner cutting weight against vertex degree
//#define SM_USE_TESS_DEGREE

// Tessellation changes: Split face using FindSplitLine before corner cutting
//#define SM_USE_TESS_SPLITFACE
// 
// Tessellation, reuse UV trim curves
//#define SM_REUSE_UVCURVES


// The following define determines if we use exception handling
// to notify of memory errors
#ifndef _DEBUG
#define SM_USE_EXCEPTIONS 1
#endif

// If we want a couple of extra fields in the topology
// for indexing turn this on
#define SM_INDEXING 1

// OpenGL extension for the X Window System
//#define USING_OPENGLX 1

// Use the following if you want a more liberal translator setting. (default on)
#define SM_TRANSLATORS_LIBERAL 1

// Deprecation is planned for the global cache. If you use this feature you can turn it
// on with the following define.
//#define SM_USE_GLOBAL_CACHE

#ifdef SM_USE_GLOBAL_CACHE
  // Control runtime Object Cache queue lengths and memory sizes
  // Each of the 4 cache queues (curves, surfaces, trimsrfs, and Breps)
  //      can be limited by item count or allocated byte size.
  // However, each queue has a minimum item count below which it will not go.
  //    The minimum item count limit ignores the MaxCount and ByteSize limits 
  //    when there is a conflict.

  // Default Object Cache MaxCounts used by SmContext constructors
  #define SM_DEFAULT_MAXCOUNT_CURVECACHE      1000
  #define SM_DEFAULT_MAXCOUNT_SURFACECACHE     200
  #define SM_DEFAULT_MAXCOUNT_TRIMSRFCACHE     200
  #define SM_DEFAULT_MAXCOUNT_BREPCACHE        200
  
  // Default Object Cache MaxByteSizes used by SmContext constructors
                                                   // Tested Values 1 // Tested Values 2 
  #define SM_DEFAULT_BYTESIZE_CURVECACHE        0  //  6000000        //   3000000       // 0 = no Byte limit
  #define SM_DEFAULT_BYTESIZE_SURFACECACHE      0  // 25000000        //  12500000       // 0 = no Byte limit
  #define SM_DEFAULT_BYTESIZE_TRIMSRFCACHE      0  // 25000000        //  12500000       // 0 = no Byte limit
  #define SM_DEFAULT_BYTESIZE_BREPCACHE         0  //  1000000        //    800000       // 0 = no Byte limit
  
                                                
  // Minimum Object Cache MaxCounts used by SmCache to ensure enough cache objects to
  // allow operations to complete without excessive cache reconstruction
  //   The system fails when these numbers are less than 2.
  //   They should be set to at least 5 to 10, more for curves for
  //     performance reasons.
  #define SM_MINCOUNT_CURVECACHE               100
  #define SM_MINCOUNT_SURFACECACHE              20
  #define SM_MINCOUNT_TRIMSRFCACHE              20
  #define SM_MINCOUNT_BREPCACHE                 20
#endif // SM_USE_GLOBAL_CACHE

// Turn off unreferenced formal parameter warnings in Windows
#if defined(_WIN32)

// disable compiler warning: code uses a deprecated function, class member, variable, or typedef 
#pragma warning(disable : 4996)

#endif // _WIN32

// Choose SM_DEBUG_CODE if you want to include debugging code in compilation
// NOTE: If SM_DEBUG_CODE is defined, SM_GFX_CODE also needs to be defined

#ifdef _DEBUG 
  #ifndef SM_NO_DEBUG_CODE
    #ifndef SM_DEBUG_CODE
      #define SM_DEBUG_CODE 1
    #endif
  #endif // SM_NO_DEBUG_CODE
#endif // _DEBUG

// Define SM_VALIDATE_TOPOLOGY to perform pBrep->ValidatePointers(); at key locations
//#define SM_VALIDATE_TOPOLOGY 1

// Define SM_VALIDATE_INTERSECTORS to check derived analytic 
// intersection results against more costly general SmBSpline intersectors.
//#define SM_VALIDATE_INTERSECTORS

// Choose SM_GFX_OUTPUT_CODE if you want to include graphics methods
// in compilation.  The graphics methods are in SmGraphicsOutput.cpp. They
// send output to registered draw callbacks or to SmGfxArraySet vertex arrays.

// if you need SM_GFX_OUT_CODE you must also set SM_GFX_CODE
#if !defined(SM_NO_GFX_OUTPUT_CODE)
  #define SM_GFX_OUTPUT_CODE 1
  #if !defined SM_GFX_CODE
    #define SM_GFX_CODE 1
  #endif
#endif

// Dev viewport/test runners can install draw callbacks to capture legacy debug
// graphics without requiring an OpenGL window. Keep release builds callback-free
// unless a consumer opts in explicitly.
#ifndef SM_GRAPHICS_CALLBACKS
  #if defined(SM_DEBUG_CODE) && defined(SM_GFX_CODE)
    #define SM_GRAPHICS_CALLBACKS 1
  #else
    #define SM_GRAPHICS_CALLBACKS 0
  #endif
#endif

// Use the following if you need to know about the creation 
// counts on a topology or polygon entity.  Normally only used
// for Heavy debugging exercises.
//#define USE_DEBUG_COUNTER 1

// Use the following when compiling with the Borland or Embarcadero compilers.
// Also use the following for any other compiler that only
//   supports a single overloaded operator new definition per class.
//#define SM_BORLAND

// If your compiler has a 32K local data limit (i.e. some MAC compilers) then remove the comments for the following define.
//#define SM_32K_LOCAL_DATA_LIMIT 1

// Please note that some compilers do not support ios::binary
// If that is the case use define SM_NO_IOS_BINARY in the compile definitions.


#ifndef SM_IOS
//  For Linux - SuSe 9.1    #define SM_IOS std::ios_base
#define SM_IOS std::ios_base
#endif

#if defined(SM_NO_IOS_BINARY)
#define SM_IOS_BINARY ((std::ios_base::openmode)0)
#else
#define SM_IOS_BINARY SM_IOS::binary
#endif


#define SM_IOS_NOCREATE ((std::ios_base::openmode)0)

// Some systems do not allow a reinterpret_cast and should define SM_NO_REINTERPRET_CAST in the compile definitions.
#if defined(SM_NO_REINTERPRET_CAST)
#define SM_REINTERPRET_CAST(type,var) ((type)(var))
#else
#define SM_REINTERPRET_CAST(type,var) reinterpret_cast<type>(var)
#endif

// Some systems do not allow a const_cast and should define SM_NO_CONST_CAST in the compile definitions.
#if defined (SM_NO_CONST_CAST) 
#define SM_CONST_CAST(type,var) ((type)(var))
#else 
#define SM_CONST_CAST(type,var) const_cast<type>(var)
#endif

// under development - changing FilletSurfaces from SmBSplineSurfaces to SmSurface objects
// new code define SM_FILLET_SURFS, old behavior omit SM_FILLET_SURFS
#define SM_FILLET_SURFS
#ifdef SM_FILLET_SURFS
 #define SM_FILLETSURF_TYPE SmSurface
 #define SM_CAST_FILLETSURF_PTR(T, p) ((p) != NULL && (p)->IsKindOf(SmSurface_TYPE) ? (SmSurface*)(p) : NULL)

#else  // no SM_FILLET_SURFS
 #define SM_FILLETSURF_TYPE SmBSplineSurface
 #define SM_CAST_FILLETSURF_PTR(T, p) ((p) != NULL && (p)->IsKindOf(SmBSplineSurface_TYPE) ? (SmBSplineSurface*)(p) : NULL)
#endif // no SM_FILLET_SURFS

// #define SM_EXACT_SPACEDEF_GEOMETRY  // with   : SpaceDef commands insert SmCrvInVolume and SmSrfInVolume objs into a Brep's topology graph
                                       // without: SpaceDef commands replace SmCrvInVolume and SmSrfInVolume objs with BSpline approximations

// NewTolerance Scheme make sure one and only one of SM_USE_NEWTOL and SM_USE_OLDTOL are defined
// under development - make sure the following are not defined  - not yet complete or ready for use
// #define SM_USE_NEWTOL             // with SM_USE_NEWTOL: SmTol::methods return Consistent tolerance values
#define SM_USE_OLDTOL                // with SM_USE_OLDTOL: SmTol::methods return old style tolerance values

//  SM_NEWTOL_LINE marks a line intended for the new tolerance model
//  SM_OLDTOL_LINE marks a line intended for the old tolerance model
//  SM_TOL_LINE    marks a line needing review for integrations with the new and old tolerance models
#define SM_NEWTOL_LINE
#define SM_OLDTOL_LINE
#define SM_TOL_LINE

// Linux could not compile the SM_SLASH Macro - removed for compliance with LINUX
// temporary and only for convenience - define SM_NEWTOL_LINE and SM_OLDTOL_LINE
//  //   when SM_USE_NEWTOL is defined,     lines starting with SM_NEWTOL_LINE are compiled and starting with SM_OLDTOL_LINE are not.
//  //   when SM_USE_NEWTOL is NOT defined, lines starting with SM_OLDTOL_LINE are compiled and starting with SM_NEWTOL_LINE are not.
//  
//  #define SM_SLASH(a) /##a
//  
//  // SM_USE_NEWTOL 
//  #ifdef SM_USE_NEWTOL
//    #define SM_NEWTOL_LINE
//    #define SM_OLDTOL_LINE SM_SLASH(/)
//  #else // SM_USE_OLDTOL
//    #define SM_NEWTOL_LINE SM_SLASH(/)
//    #define SM_OLDTOL_LINE
//  #endif // SM_USE_OLDTOL
//  
// end removed section - left until no longer needed or a way to implement for Linux is found

// #define SM_NMTLIB_7166  // temporary - to be removed - used to toggle between problems found in releasing the consistent tol model chenages
// #define SM_USE_NEWTOL_STRONG_TYPES// with    SM_USE_NEWTOL_STRONG_TYPES - no SmTol3d to double auto conversions
//                                   // without SM_USE_NEWTOL_STRONG_TYPES -    SmTol3d to double auto conversions

// #define SM_USE_CONSTRUCTOR_ASSERT_VALID  // run AssertValid in constructors on newly created objects

// The USE_ANALYTICS define enables analytically assisted NURBS.  
// This is used in SmBSplineCurve::CopyAndAddAnalytics
#define USE_ANALYTICS  1

// The ANALYTIC_TOL_SCALE value multiplies the SM_EFF_ZERO
//  tolerance value in functions that examine Nurb surfaces
//  to decide if they can be represented by analytic surfaces.
//  SM_EFF_ZERO is usually close to machine precision and many
//  orders of magnitude tighter than Brep tolerances.  
//  Using ANALYTIC_TOL_SCALE values increasingly greater than one 
//  allows SMLib to classify Nurb surface representations 
//  of analytic surfaces with increasingly
//  (yet still very small) amounts of noise as analytic surfaces. 
//
// The analytic tolerance scale can be increased from
//   1.0 (12 decimal places of accuracy as defined by SM_EFF_ZERO) 
//   to 100.0 (10 decimal places) to 1000000.0 (6 decimal places).
//   Note that this is a little dangerous because you should have nice 
//   correspondance between the analytic and NURBS.  
//   We recommend not going beyond 100.00
//   This must be set even if USE_ANALYTICS is not set.

#define ANALYTIC_TOL_SCALE 100.0

// Removing composite topology, SmCEdge and SmCFace: under development.
#define SM_NO_COMPOSITES 1

// The following defines help control some of the graphic display attributes.
#define SM_VERTEX_POINTSIZE 4.0
#define SM_SURF_CACHE_ANGLE_TOL 20.0*SM_PI/180.0

/* When compiling for Debug - the following add various (read as slow) tests */
/* sequentials to the normal flow of control. */

// define SM_DEFINED_HASH_ORDER to use ordered outputs from the hash tables
// This comes at a performance cost, but makes results consistent and easier to debug
// #define SM_DEFINED_HASH_ORDER 


// define SM_SBDV_LOG to get a stream of status messages for the Surface Cache subdivision scheme
#ifdef SM_DEBUG_CODE
  #define SM_SBDV_LOG
#endif // SM_DEBUG_CODE

#endif  // !__SMOS_CONFIG_H__
