// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCacheMgrBrep.h
* PURPOSE: Header file for the cache manager for surfaces.
**********************************************************************/

#ifdef SM_USE_GLOBAL_CACHE

#ifndef __SMCACHEMGRBREP_H_
#define __SMCACHEMGRBREP_H_

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMCACHEMGRTSRF_H__
#include <SmCacheMgrTSrf.h>
#endif

/*******************************************************************//**
PURPOSE: This object creates Brep caches.  

NOTES: This objects primary use is to provide a virtual method
    for creating brep caches.  This allows us to layer the cache manager
    properly.  
***********************************************************************/
class SM_EXPORT SmCacheMgrBrep : public SmCacheMgrTSrf
{
public:
   SmCacheMgrBrep
   (
     const SmContext & crContext,       ///< [in] : owning context                                     
     ULONG lCurveCacheCount      = SM_DEFAULT_MAXCOUNT_CURVECACHE  ,  ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_CURVE  ]                 
     ULONG lSurfaceCacheCount    = SM_DEFAULT_MAXCOUNT_SURFACECACHE,  ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_SURFACE]                 
     ULONG lTrimSrfCacheCount    = SM_DEFAULT_MAXCOUNT_TRIMSRFCACHE,  ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF]                 
     ULONG lBrepCacheCount       = SM_DEFAULT_MAXCOUNT_BREPCACHE   ,  ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_BREP   ]                 
                                                                                                                       
     ULONG lCurveCacheByteSize   = SM_DEFAULT_BYTESIZE_CURVECACHE  ,  ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_CURVE  ], 0 = no limit
     ULONG lSurfaceCacheByteSize = SM_DEFAULT_BYTESIZE_SURFACECACHE,  ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_SURFACE], 0 = no limit
     ULONG lTrimSrfCacheByteSize = SM_DEFAULT_BYTESIZE_TRIMSRFCACHE,  ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF], 0 = no limit
     ULONG lBrepCacheByteSize    = SM_DEFAULT_BYTESIZE_BREPCACHE      ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_BREP   ], 0 = no limit
   ); 

   virtual ~SmCacheMgrBrep() {}
   
   virtual SmStatus CacheMakeOrValidate
   (
     SmObjectCacheType  eObjectCacheType,     ///< [in] : oneof SM_OC_CURVE
                                              //            SM_OC_SURFACE        
                                              //            SM_OC_TRIMSRF
                                              //            SM_OC_BREP
     const SmObject   * pObject,              ///< [in] : target object
     SmCacheObj       * pOldCache,            ///< [in] : existing target Object's ObjectCache or NULL
     SmCacheObj      *& rpNewCache            // out: ptr to target object's ObjectCache
   );

};


#endif // !__SMCACHEMGRBREP_H_

#endif // SM_USE_GLOBAL_CACHE
