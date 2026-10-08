// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmCacheMgrCrv.cpp
* PURPOSE: Source file for curve cache manager methods.
**********************************************************************/

#include "StdAfx.h"

#ifdef SM_USE_GLOBAL_CACHE

#include <SmCacheMgrCrv.h>
#include <SmCurveCache.h>

#ifdef SM_DEBUG_CODE
#include <SmSurface.h>
#include <SmBrep.h>
#include <SmFace.h>
#include <SmEdge.h>
#endif // SM_DEBUG_CODE


/*******************************************************************//**
PURPOSE: This is the constructor for the curve cache manager.  

NOTES: 
***********************************************************************/
SmCacheMgrCrv::SmCacheMgrCrv
  (const SmContext & crContext,
   ULONG lCurveCacheCount,            // in : max count for SmGlobalCache::m_spCaches[SM_OC_CURVE  ]                 
   ULONG lSurfaceCacheCount,          // in : max count for SmGlobalCache::m_spCaches[SM_OC_SURFACE]                 
   ULONG lTrimSrfCacheCount,          // in : max count for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF]                 
   ULONG lBrepCacheCount,             // in : max count for SmGlobalCache::m_spCaches[SM_OC_BREP   ]                 
                                                                                                                     
   ULONG lCurveCacheByteSize,         // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_CURVE  ], 0 = no limit
   ULONG lSurfaceCacheByteSize,       // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_SURFACE], 0 = no limit
   ULONG lTrimSrfCacheByteSize,       // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF], 0 = no limit
   ULONG lBrepCacheByteSize)          // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_BREP   ], 0 = no limit
 : SmCacheMgr(crContext,
              lCurveCacheCount,
              lSurfaceCacheCount,
              lTrimSrfCacheCount,
              lBrepCacheCount,

              lCurveCacheByteSize,  
              lSurfaceCacheByteSize,
              lTrimSrfCacheByteSize,
              lBrepCacheByteSize)   
{
}

/*******************************************************************//**
PURPOSE: This method validates an existing cache and/or creates
   a new cache for a curve.

NOTES:
 
The returned ObjectCache stores a pointer to the target Object.
The returned ObjectCache is not placed in the global cache queues.

The value of eObjectCacheType determines which type of ObjectCache 
is constructed.  A call to any SmCacheMgrDerivedType::CacheMakeOrValidate()
function starts by checking this value.  If it is appropriate an ObjectCache
pointer is returned else the call is passed onto the next level until 
a match between derived type and requested type is found. When no match
is found NULL is returned. 

The order of calls and return ObjectCache types are:

     call SmCacheMgrBrep::CacheMakeOrValidate 
             if(eObjectCacheType == SM_OC_BREP)      return SmBrepCache
else call SmCacheMgrTSrf::CacheMakeOrValidate 
             if(eObjectCacheType == SM_OC_TRIMSRF)   return SmTrimSrfCache
else call SmCacheMgrSrf::CacheMakeOrValidate 
             if(eObjectCacheType == SM_OC_SURFACE)   return SmSurfaceCache
else call SmCacheMgrCrv::CacheMakeOrValidate 
             if(eObjectCacheType == SM_OC_CURVE)     return SmCurveCache
else call SmCacheMgr::CacheMakeOrValidate 
                                                     return NULL
 
***********************************************************************/
SmStatus SmCacheMgrCrv::CacheMakeOrValidate
  (SmObjectCacheType eObjectCacheType,       // in : oneof SM_OC_CURVE
                                             //            SM_OC_SURFACE        
                                             //            SM_OC_TRIMSRF
                                             //            SM_OC_BREP
   const SmObject  * cpObject,               // in : target object
   SmCacheObj      * cpOldCache,             // in : existing target Object's ObjectCache or NULL
   SmCacheObj     *& rpNewCache)             // out: ptr to target object's ObjectCache
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
#endif // SM_DEBUG_CODE

  // init output
  rpNewCache = NULL;

  // when requested ObjectCache Type is not Curve try the SmCacheMgr construction
  //    currently SmCacheMgr construction just returns NULL.
  if (eObjectCacheType != SM_OC_CURVE) 
    { 
      return SmCacheMgr::CacheMakeOrValidate(eObjectCacheType,
                                             cpObject,
                                             cpOldCache,
                                             rpNewCache);
    }

  // low work - use OldCache when its given
  if (cpOldCache) 
    {

#ifdef SM_DEBUG_CODE
  // draw 
  if(bDebugMe)
    {
      SmCurveCache *pCurveCache = SM_CAST_PTR(SmCurveCache, cpOldCache) ; 
      pCurveCache->Dump() ;

      SmCurve   *pCurve   = SM_CAST_PTR(SmCurve, cpObject) ; 
      SmSurface *pSurface = SM_CAST_PTR(SmSurface, cpObject) ;
      SmBrep    *pBrep    = SM_CAST_PTR(SmBrep, cpObject) ;
      SmFace    *pFace = pSurface ? (SmFace *)pSurface->GetFace() : NULL ;
      SmEdge    *pEdge = pCurve ? (SmEdge *)pCurve->GetEdge() : NULL ;
      pBrep =   pBrep ? pBrep
              : pFace ? pFace->GetBrep() 
              : pEdge ? pEdge->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(pCurve) pCurve->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; if(pCurveCache) pCurveCache->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

      rpNewCache = (SmCacheObj*)cpOldCache;
      return SM_SUCCESS;
    }
          
  //create a new SmCurveCache containing a decomposition tree and store it with the curve.

  // cast Object to SmCurve and get its interval 
  const SmCurve *cpCurve  = (const SmCurve*)cpObject ;  SM_ASSERT (cpCurve != NULL) ;
  SmExtent1d     sIvl     = cpCurve->GetNaturalInterval();
  
  // pick dChordHeightTol and dAngularTol tessellation control parameter values 
         
  // get an Idea of the curve's size.
#define SAMPLE_POINTS 5
  double dLength = cpCurve->ApproximateLength(sIvl,SAMPLE_POINTS);
  int    NumberOfSpans = cpCurve->GetNumberNaturalKnots() -1;
  double ChordFactor   = smos_Min(250, 30*NumberOfSpans);  // 0.004 of length may be too tight
#undef SAMPLE_POINTS

  // set chordHeight tolerance based on curve size
  double dChordHeightTol = dLength/ChordFactor; 
          
  // set angular tolerance 
  //     - need to experiment a little to see which values are optimal 
  //       for the various global solver algorithms which we will utilize.
  double dAngularTolDeg = 15.0;  // 15 degrees might be ok
              
  // construct new CurveCache Object - mark it for delete if a failure exits scope
  const SmContext *cpContext = cpObject->GetContext(); NER(cpContext);
  SmCurveCache *pCurveCache  = new(*cpContext) SmCurveCache
          (*cpCurve,        // in : target curve   
           sIvl,            // in : target interval
           TRUE,            // in : bMakeFullTree: TRUE = don't simplify tree for parents that pass tessellation tests
           dChordHeightTol, // in : max leafNode control-polygon vertex to baseline distance, 0 = ignore
           dAngularTolDeg,  // in : max leafNode control-polygon vertex angle sum,            0 = ignore
           0.0,             // in : max leafNode off axis control-polygon BBox size,          0 = ignore
                            //        subdivides leaves into axis aligned near-linear segments.         
           0,               // in : limits max element size, 0 = ignore                  
           TRUE,            // in : TRUE = propagate child properties up to parent nodes.
                            //      FALSE= don't 
           0.0,             // in : max leafNode 3d control-polygon baseline size,  0 = ignore
           0.0001) ;        // in : min (leafNode Interval)/(Curve Interval) ratio, 0 = ignore
  
  NER(pCurveCache);                                                                           
  SmObjDelete sCleanup(pCurveCache);
              
  // build the CruveCache's curve subdivision tree
  SER(pCurveCache->Tessellate()); 

#ifdef SM_DEBUG_CODE
  // draw 
  if(bDebugMe)
    {
      // dump and draw CurveCache in context
      pCurveCache->Dump() ;

      SmCurve   *pCurve   = SM_CAST_PTR(SmCurve, cpObject) ; 
      SmSurface *pSurface = SM_CAST_PTR(SmSurface, cpObject) ;
      SmBrep    *pBrep    = SM_CAST_PTR(SmBrep, cpObject) ;
      SmFace    *pFace = pSurface ? (SmFace *)pSurface->GetFace() : NULL ;
      SmEdge    *pEdge = pCurve ? (SmEdge *)pCurve->GetEdge() : NULL ;
      pBrep =   pBrep ? pBrep
              : pFace ? pFace->GetBrep() 
              : pEdge ? pEdge->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pSurface) pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,0) ; if(pCurve) pCurve->Draw(NULL, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,1) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; if(pCurveCache) pCurveCache->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
  
  if (FALSE) 
    {
      // pretty print memory used
      TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
      ULONG lMemoryUsed, lMemoryAllocated ;
      lMemoryUsed = pCurveCache->GetMemoryUsed(lMemoryAllocated) ;
      smos_sprintf(sBuff,       _T("Caching Curve = 0x%p, Cache = 0x%p, Number Of Spans = %ld, Memory used/alloc = %ld/%ld\n"),
                 cpCurve, pCurveCache, pCurveCache->GetSpanCount(), lMemoryUsed, lMemoryAllocated);
      smos_sprintf(sBuffForFile,_T("Caching Curve = %s, Cache = %s, Number Of Spans = %ld, Memory used/alloc = %ld/%ld\n"),
                 cpCurve ? _T("notNULL") : _T("NULL"),pCurveCache ? _T("notNULL") : _T("NULL"),
                 pCurveCache->GetSpanCount(), lMemoryUsed, lMemoryAllocated);
      smos_WriteBuffer(sBuff, sBuffForFile);
    }
#endif // SM_DEBUG_CODE
          
  // subdivision worked - clear the delete mark and set output
  sCleanup.Clear();
  rpNewCache = pCurveCache;

  // all done
  return SM_SUCCESS;

} // end SmCacheMgrCrv::CacheMakeOrValidate

#endif

