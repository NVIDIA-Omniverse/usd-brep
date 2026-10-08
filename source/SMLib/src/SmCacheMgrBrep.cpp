// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmCacheMgrBrep.cpp
* PURPOSE: Source file for curve cache manager methods.
**********************************************************************/

#include "StdAfx.h"

#ifdef SM_USE_GLOBAL_CACHE

#include <SmCacheMgrBrep.h>
#include <SmSAGObject.h>
#include <SmBrepCache.h>

/*******************************************************************//**
PURPOSE: This is the constructor for the curve cache manager.  

NOTES: 
***********************************************************************/
SmCacheMgrBrep::SmCacheMgrBrep
  (const SmContext & crContext,       // in : owning context                                            
   ULONG lCurveCacheCount,            // in : max count for SmGlobalCache::m_spCaches[SM_OC_CURVE  ]                 
   ULONG lSurfaceCacheCount,          // in : max count for SmGlobalCache::m_spCaches[SM_OC_SURFACE]                 
   ULONG lTrmSrfCacheCount,           // in : max count for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF]                 
   ULONG lBrepCacheCount,             // in : max count for SmGlobalCache::m_spCaches[SM_OC_BREP   ]                 
                                                                                                                     
   ULONG lCurveCacheByteSize,         // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_CURVE  ], 0 = no limit
   ULONG lSurfaceCacheByteSize,       // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_SURFACE], 0 = no limit
   ULONG lTrimSrfCacheByteSize,       // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF], 0 = no limit
   ULONG lBrepCacheByteSize)          // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_BREP   ], 0 = no limit
                             
     : SmCacheMgrTSrf(crContext,   
                      lCurveCacheCount,
                      lSurfaceCacheCount,
                      lTrmSrfCacheCount,
                      lBrepCacheCount,

                      lCurveCacheByteSize,  
                      lSurfaceCacheByteSize,
                      lTrimSrfCacheByteSize,
                      lBrepCacheByteSize)   
{
}

/*******************************************************************//**
PURPOSE: This method validates an existing cache and/or creates
            a new cache for a Brep or PolyBrep Object. 

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
             if(eObjectCacheType == SM_OC_BREP)     return SmBrepCache
else call SmCacheMgrTSrf::CacheMakeOrValidate 
             if(eObjectCacheType == SM_OC_TRIMSRF)  return SmTrimSrfCache
else call SmCacheMgrSrf::CacheMakeOrValidate 
             if(eObjectCacheType == SM_OC_SURFACE)  return SmSurfaceCache
else call SmCacheMgrCrv::CacheMakeOrValidate 
             if(eObjectCacheType == SM_OC_CURVE)    return SmCurveCache
else call SmCacheMgr::CacheMakeOrValidate 
                                                    return NULL

***********************************************************************/
SmStatus SmCacheMgrBrep::CacheMakeOrValidate
  (SmObjectCacheType eObjectCacheType,       // in : oneof SM_OC_CURVE
                                             //            SM_OC_SURFACE        
                                             //            SM_OC_TRIMSRF
                                             //            SM_OC_BREP
   const SmObject  * cpObject,               // in : target object
   SmCacheObj      * cpOldCache,             // in : existing target Object's ObjectCache or NULL
   SmCacheObj     *& rpNewCache)             // out: ptr to target object's ObjectCache
{
  // init output
  rpNewCache = NULL;

  if (eObjectCacheType != SM_OC_BREP) 
    { 
      // When requested ObjectCache Type is not Brep try TrimSrfCache Construction 
      return SmCacheMgrTSrf::CacheMakeOrValidate(eObjectCacheType,
                                                 cpObject,
                                                 cpOldCache,
                                                 rpNewCache);
    }

  // cast Object to SmSAGObject 
  //  (SAG = Standalone with Graphics 
  //   derived Types include: SmBrep, SmPolyBrep, SmAssembly, and SmAssemblyInstance)
  SmSAGObject * pSAGObject = SM_CAST_PTR(SmSAGObject,cpObject);

  // check that object is an SmBrep or SmPolyBrep object
  SmBrep     * cpBrep     = SM_CAST_PTR(SmBrep,cpObject);
  SmPolyBrep * cpPolyBrep = SM_CAST_PTR(SmPolyBrep,cpObject);
  SM_ASSERT(cpBrep || cpPolyBrep) ;

  // low work - use OldCache when its given
  SmBrepCache *pBrepCache = (SmBrepCache*)cpOldCache;
  if (pBrepCache) 
    {
      rpNewCache = (SmCacheObj*)pBrepCache;
      return SM_SUCCESS;
    }
  
  // construct new SmBrepCache Object - mark it for delete if a failure exits scope
  pBrepCache = new (*pSAGObject->GetContext()) SmBrepCache(pSAGObject);
  NER(pBrepCache);
  SmObjDelete sCleanup(pBrepCache);

  // create the vertex, curve, and surface bounding-box spatial trees
  SER(pBrepCache->BuildTrees());
  
  // The cacheBuild worked.  Clear the BrepCache delete mark and set the output
  sCleanup.Clear();
  rpNewCache = (SmCacheObj*)pBrepCache;

  // all done
  return SM_SUCCESS;

} // end SmCacheMgrBrep::CacheMakeOrValidate

#endif //SM_USE_GLOBAL_CACHE

