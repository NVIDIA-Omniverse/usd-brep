// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCacheMgrTSrf.h
* PURPOSE: Header file for the cache manager for surfaces.
**********************************************************************/

#ifdef SM_USE_GLOBAL_CACHE

#ifndef __SMCACHEMGRTSRF_H_
#define __SMCACHEMGRTSRF_H_

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMCACHEMGRSRF_H__
#include <SmCacheMgrSrf.h>
#endif

/*******************************************************************//**
PURPOSE: This object creates trimmed surface caches.  

NOTES: This objects primary use is to provide a virtual method
    for creating trimmed surface caches.  This allows us to layer the cache 
    manager properly.  
***********************************************************************/
class SM_EXPORT SmCacheMgrTSrf : public SmCacheMgrSrf
{
public:
   SmCacheMgrTSrf
   (
     const SmContext & crContext,
     ULONG lCurveCacheCount      = SM_MINCOUNT_CURVECACHE  ,  ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_CURVE  ]                 
     ULONG lSurfaceCacheCount    = SM_MINCOUNT_SURFACECACHE,  ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_SURFACE]                 
     ULONG lTrimSrfCacheCount    = SM_MINCOUNT_TRIMSRFCACHE,  ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF]                 
     ULONG lBrepCacheCount       = 0,                         ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_BREP   ]                 
                                                                                                                                             
     ULONG lCurveCacheByteSize   = 0,                         ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_CURVE  ], 0 = no limit
     ULONG lSurfaceCacheByteSize = 0,                         ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_SURFACE], 0 = no limit
     ULONG lTrimSrfCacheByteSize = 0,                         ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF], 0 = no limit
     ULONG lBrepCacheByteSize    = 0                          ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_BREP   ], 0 = no limit
   );                        

   virtual ~SmCacheMgrTSrf() {}
   
   virtual SmStatus CacheMakeOrValidate
   (
     SmObjectCacheType   eObjectCacheType,      ///< [in] : oneof SM_OC_CURVE
                                                ///<              SM_OC_SURFACE        
                                                ///<              SM_OC_TRIMSRF
                                                ///<              SM_OC_BREP
     const SmObject    * pObject,               ///< [in] : target object
     SmCacheObj        * pOldCache,             ///< [in] : existing target Object's ObjectCache or NULL
     SmCacheObj       *& rpNewCache             ///< [out]: ptr to target object's ObjectCache
   );

};

#endif // !__SMCACHEMGRTSRF_H_

#endif //SM_USE_GLOBAL_CACHE

