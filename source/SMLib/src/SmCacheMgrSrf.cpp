// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmCacheMgrSrf.cpp
* PURPOSE: Source file for curve cache manager methods.
**********************************************************************/

#include "StdAfx.h"

#ifdef SM_USE_GLOBAL_CACHE

#include <SmCacheMgrSrf.h>
#include <SmSurface.h>
#include <SmSurfaceCache.h>

/*******************************************************************//**
PURPOSE: This is the constructor for the curve cache manager.  

NOTES: 
***********************************************************************/
SmCacheMgrSrf::SmCacheMgrSrf
  (const SmContext & crContext,                  
   ULONG lCurveCacheCount,            // in : max count for SmGlobalCache::m_spCaches[SM_OC_CURVE  ]                 
   ULONG lSurfaceCacheCount,          // in : max count for SmGlobalCache::m_spCaches[SM_OC_SURFACE]                 
   ULONG lTrimSrfCacheCount,          // in : max count for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF]                 
   ULONG lBrepCacheCount,             // in : max count for SmGlobalCache::m_spCaches[SM_OC_BREP   ]                 
                                                                                                                     
   ULONG lCurveCacheByteSize,         // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_CURVE  ], 0 = no limit
   ULONG lSurfaceCacheByteSize,       // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_SURFACE], 0 = no limit
   ULONG lTrimSrfCacheByteSize,       // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF], 0 = no limit
   ULONG lBrepCacheByteSize)          // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_BREP   ], 0 = no limit
 : SmCacheMgrCrv(crContext,
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
   a new cache for a surface.

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
SmStatus SmCacheMgrSrf::CacheMakeOrValidate
  (SmObjectCacheType eObjectCacheType,       // in : oneof SM_OC_CURVE
                                             //            SM_OC_SURFACE        
                                             //            SM_OC_TRIMSRF
                                             //            SM_OC_BREP
   const SmObject   * cpObject,              // in : target object
   SmCacheObj       * cpOldCache,            // in : existing target Object's ObjectCache or NULL
   SmCacheObj      *& rpNewCache)            // out: ptr to target object's ObjectCache
{
  // init output
  rpNewCache = NULL;

  // When requested ObjectCache Type is not Surface try CurveCache Construction 
  if (eObjectCacheType != SM_OC_SURFACE) 
    { 
      return SmCacheMgrCrv::CacheMakeOrValidate(eObjectCacheType,
                                                cpObject,
                                                cpOldCache,
                                                rpNewCache);
    }
  
  // cast Object to SmSurface
  const SmSurface *cpSurface  = (const SmSurface*)cpObject;
  SM_ASSERT (cpSurface != NULL);

  // low work - use OldCache when its given
  SmSurfaceCache *pOldSurfaceCache = (SmSurfaceCache*)cpOldCache;
  if (pOldSurfaceCache) 
    {
      rpNewCache = (SmCacheObj*)pOldSurfaceCache;
      return SM_SUCCESS;
    }
  
  // next create a new SmSurfaceCache and store it with the surface.
  
#ifdef SM_DEBUG_CODE
static ULONG lAvecTrimSrfCache = 0 ;
static ULONG lSansTrimSrfCache = 0 ;
SmBoolean bDebugMe = FALSE ;

    // gwc:study - check to see if when building a new TrimSrfCache if this
    //             object already has an SmTrimSrfCache
    SmTrimSrfCache *pTrimSrfCache = (SmTrimSrfCache *)SmCacheMgr::GetObjectCache(SM_OC_TRIMSRF, cpSurface) ;
    if(pTrimSrfCache)
      { lAvecTrimSrfCache++ ; }
    else
      { lSansTrimSrfCache++ ; }
    if(   bDebugMe
       || (lAvecTrimSrfCache + lSansTrimSrfCache) % 20 == 0)
      {
        TCHAR sBuff[SM_TBLOCK_SIZE] ;
        smos_sprintf(sBuff,_T("\n SurfaceCache creates[%lu] with TrimSrfCache[%lu], without TrimSrfCache[%lu]"),
                   lAvecTrimSrfCache + lSansTrimSrfCache,
                   lAvecTrimSrfCache,
                   lSansTrimSrfCache) ;
        smos_WriteBuffer(sBuff) ;
      } 

#endif // SM_DEBUG_CODE

  // Get some point positions scattered over the Surface's Natural Domain.
  SmExtent2d sUVDomain = cpSurface->GetNaturalUVDomain();
  SmPoint3d sPMid, sP1, sP2, sPMid2, sPMid3;
  SER(cpSurface->EvaluatePoint(sUVDomain.GetMin(),sP1));
  SER(cpSurface->EvaluatePoint(sUVDomain.Evaluate(0.5,0.5),sPMid));
  SER(cpSurface->EvaluatePoint(sUVDomain.Evaluate(0.9,0.1),sPMid2));
  SER(cpSurface->EvaluatePoint(sUVDomain.Evaluate(0.1,0.9),sPMid3));
  SER(cpSurface->EvaluatePoint(sUVDomain.GetMax(),sP2));

  // get an Idea of surface's size.
  double dSurfaceSizeScale = (  sPMid.DistanceBetween(sP1)
                              + sPMid.DistanceBetween(sP2) 
                              + sPMid.DistanceBetween(sPMid2) 
                              + sPMid.DistanceBetween(sPMid3)) / 2.0 ;

  // use surface size estimate to set chordHeight tolerance
  double dChordHeightTol = dSurfaceSizeScale/40.0;  // Tessellation CH is 1/40th of estimated surface size
  
  double dAngleTol = 20.0*SM_PI / 180.0; // Angle tolerance of 20 degrees

  // Loosen tolerances on analytic surfaces - they don't need as much subdivision.
  if (   cpSurface->GetDegree(SM_SP_U) <= 2
      && cpSurface->GetDegree(SM_SP_V) <= 2) 
    {
      dChordHeightTol = dSurfaceSizeScale / 15.0;  
      dAngleTol       = 2.0 * dAngleTol;
    }

  // construct new SurfaceCache Object - mark it for delete if a failure exits scope
  const SmContext *cpContext     = cpObject->GetContext() ; NER(cpContext);
  SmSurfaceCache  *pSurfaceCache = new(*cpContext) SmSurfaceCache(*cpSurface,
                                                                  sUVDomain,
                                                                  dChordHeightTol,
                                                                  dAngleTol);
  NER(pSurfaceCache);
  SmObjDelete sCleanup(pSurfaceCache);

  // build the surface subdivision tree
  SER(pSurfaceCache->BuildTree());
  
#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      smgfx_Erase() ;
      pSurfaceCache->Draw();
      sm_GraphicsLoop() ;
    }
  
  if (FALSE) 
    {
      SmTArray<SmTreeNode*> sNodes;
      pSurfaceCache->GetTree()->GetAllTreeNodes(sNodes);
      ULONG lMemoryUsed, lMemoryAllocated ;
      lMemoryUsed = pSurfaceCache->GetMemoryUsed(lMemoryAllocated) ;
      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff,_T("Surface Tessellation, Node Count: %ld, Memory Used/Alloc: %ld/%ld bytes\n"),
          sNodes.GetSize(), lMemoryUsed, lMemoryAllocated);
      smos_WriteBuffer(sBuff);
    }
#endif
  
  // subdivision worked - clear the delete mark and set output
  sCleanup.Clear();
  rpNewCache = (SmCacheObj*)pSurfaceCache;

  // all done
  return SM_SUCCESS;

} // end SmCacheMgrSrf::CacheMakeOrValidate

#endif //SM_USE_GLOBAL_CACHE
