// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmCacheMgr.cpp
* PURPOSE: Source file for SmCacheMgr methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmCacheMgr.h>
#include <SmCacheMgrBrep.h>
#include <SmContext.h>
#include <SmCurveCache.h>
#include <SmSurfaceCache.h>
#include <SmTrimSrfCache.h>
#include <SmBrepCache.h>

#ifdef SM_USE_GLOBAL_CACHE
/*******************************************************************//**
PURPOSE: Constructor for global cache

NOTES: Note that a default cache size of 1000 curve caches, 
   200 surface caches, 200 trimmed surface caches, and 200 brep caches
   is used by default on context creation.
***********************************************************************/
SmGlobalCache::SmGlobalCache
  (ULONG lCurveCacheCount,       // in : max count for SmGlobalCache::m_spCaches[SM_OC_CURVE  ]                 
   ULONG lSurfaceCacheCount,     // in : max count for SmGlobalCache::m_spCaches[SM_OC_SURFACE]                 
   ULONG lTrimSrfCacheCount,     // in : max count for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF]                 
   ULONG lBrepCacheCount,        // in : max count for SmGlobalCache::m_spCaches[SM_OC_BREP   ]                 
                                                                                                                
   ULONG lCurveCacheByteSize,    // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_CURVE  ], 0 = no limit
   ULONG lSurfaceCacheByteSize,  // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_SURFACE], 0 = no limit
   ULONG lTrimSrfCacheByteSize,  // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF], 0 = no limit
   ULONG lBrepCacheByteSize)     // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_BREP   ], 0 = no limit
{
    m_spCaches[0]     = NULL;
    m_spCaches[1]     = NULL;
    m_spCaches[2]     = NULL;
    m_spCaches[3]     = NULL;
    m_slNumCaches     = SM_OC_CACHE_TYPE_COUNT ;
    m_spOuterCacheMgr = NULL;

#ifdef SM_DEBUG_CODE
    s_pCacheStack = NULL;
#endif
    const SmContext *cpContext = GetContext();
    SmContext       *pContext  = SM_CONST_CAST(SmContext*,cpContext);

    if (!pContext) { SE(SM_ERR); }
    // Note that the m_spOuterCacheMgr for the global cache is 
    // set by the following  constructor.
    pContext->SetGlobalCache(this);
    // pOuterCache is not used ??

    (void)new(*pContext) SmCacheMgrBrep(*pContext,
                                                             lCurveCacheCount,
                                                             lSurfaceCacheCount,
                                                             lTrimSrfCacheCount,
                                                             lBrepCacheCount,
                                                             
                                                             lCurveCacheByteSize,  
                                                             lSurfaceCacheByteSize,
                                                             lTrimSrfCacheByteSize,
                                                             lBrepCacheByteSize);

} // end SmGlobalCache::SmGlobalCache Constructor

/*******************************************************************//**
PURPOSE: Destructor for the global cache

NOTES: 
***********************************************************************/
SmGlobalCache::~SmGlobalCache()
{
  SM_ASSERT(m_spOuterCacheMgr != NULL) ; delete m_spOuterCacheMgr ; m_spOuterCacheMgr = NULL ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
//      TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

      // type, m_iElemCount, and m_iTotalBasisCount
//      smos_sprintf(sBuff       ,_T("\n  SmGlobalCache[0x%p]"),  this) ; 
//      smos_sprintf(sBuffForFile,_T("\n  SmGlobalCache[%s]"),    this ? _T("NotNULL") : _T("NULL")) ; 
//      smos_WriteBuffer(sBuff, sBuffForFile);

      m_sMaxCacheSizes.Dump(TRUE, TRUE) ;
    }

#endif // SM_DEBUG_CODE

//    for (ULONG i=0; i<m_slNumCaches; i++) {   // Don't think we need this
//        if (m_spCaches[i]) { delete m_spCaches[i]; m_spCaches[i] = NULL ; }
//    }
}

/*******************************************************************//**
PURPOSE: Constructor which either sets or resets the max sizes
   of the 4 global caches; curve, surface, TrimSrf, and Brep.  
   Note that the sizes of the
   caches may only increase as we add things to the stack.  As
   the destructor is called the sizes will be removed from the 
   stack and if needed the cache will be adjusted.  Note a minimum
   stack size of 10 or so should prevent us from deleting any 
   cache objects which are in use.  Typically 2 are in use at
   any one time.

NOTES: Perhaps the following example will clarify usage.

-  main()
-  {
-      SmCacheMgr sCacheMgr(crContext,10,10,10);  // default for entire 
-         // program execution is maximmum of 10 elements 
-         // in each cache.  
-
-      foo();
-
-      exit;  // Cache is automatically cleaned up by destructor
-      // for sCacheMgr.
-  }
-
-
-  foo()
-  {
-      SmCacheMgr sCacheMgrFoo(crContext,100,50,20);  // increases maximum 
-          // size of each cache for duration of foo from 10,10 to
-          // 100 curve cache elements, 50 surface cache
-          // elements and 20 trimmed surface cache elements.  
-          // Note that all of them may not be used
-          // but they are there if needed.  Once the limit is
-          // reached the 'oldest' cached items will be removed.
-
-       ...
-
-
-       return;  // destructor for sCacheMgrFoo will return
-       // cache to original size.  This may cause deletion of
-       // excess cached items which exceed our original 10,10
-       // limits.
-  }

***********************************************************************/

SmCacheMgr::SmCacheMgr
  (const SmContext & crOwningContext, // in : Context that owns this cache mgr
   ULONG lCurveCacheCount,            // in : max count for SmGlobalCache::m_spCaches[SM_OC_CURVE  ]                 
   ULONG lSurfaceCacheCount,          // in : max count for SmGlobalCache::m_spCaches[SM_OC_SURFACE]                 
   ULONG lTrimSrfCacheCount,          // in : max count for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF]                 
   ULONG lBrepCacheCount,             // in : max count for SmGlobalCache::m_spCaches[SM_OC_BREP   ]                 
                                                                                                                     
   ULONG lCurveCacheByteSize,         // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_CURVE  ], 0 = no limit
   ULONG lSurfaceCacheByteSize,       // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_SURFACE], 0 = no limit
   ULONG lTrimSrfCacheByteSize,       // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF], 0 = no limit
   ULONG lBrepCacheByteSize)          // in : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_BREP   ], 0 = no limit
 : m_crOwningContext(crOwningContext)
{
  // Branch on SmGlobalCache CacheSize nesting depth
  // 1. Start: if(crOwningContext::m_pGlobalContext ObjectType caches are Null)
  //           then allocate 0 size ObjectType SmCache objects in GlobalCache
  // 2. Nested: crOwningContext::m_pGlobalContext ObjectType caches already exist
  //           then increment the ObjectType cache m_sMaxCountStack depth
  SmGlobalCache *pGlobalCache = crOwningContext.GetGlobalCache();
  if (pGlobalCache->m_spOuterCacheMgr == NULL) 
    {
      pGlobalCache->m_spOuterCacheMgr = this;
    }

  // for every ObjectCacheType
  for (ULONG i=0; i<SM_OC_CACHE_TYPE_COUNT; i++) 
    {
      
      // when ObjectType cache is missing
      if (!pGlobalCache->m_spCaches[i]) 
        {
          // create and init the ObjectType cache 
          pGlobalCache->m_spCaches[i] = new(*pGlobalCache->m_spOuterCacheMgr->GetContext()) SmCache();
          pGlobalCache->m_spCaches[i]->m_sMaxCountStack.Add(0);
          pGlobalCache->m_spCaches[i]->m_sMaxByteSizeStack.Add(0);
        }

      // get cache queue count_stack depth
      ULONG lStackDepth = pGlobalCache->m_spCaches[i]->m_sMaxCountStack.GetSize();

      // get the requested depth for this cache
      ULONG lNewCount, lNewByteSize, lMinCount ;
      switch(i)
       { case SM_OC_CURVE   : lNewCount    = lCurveCacheCount ;
                              lNewByteSize = lCurveCacheByteSize ;
                              lMinCount    = SM_MINCOUNT_CURVECACHE ;          
                              break ;

         case SM_OC_SURFACE : lNewCount    = lSurfaceCacheCount ;
                              lNewByteSize = lSurfaceCacheByteSize ;          
                              lMinCount    = SM_MINCOUNT_SURFACECACHE ;          
                              break ;

         case SM_OC_TRIMSRF : lNewCount    = lTrimSrfCacheCount ;
                              lNewByteSize = lTrimSrfCacheByteSize ;          
                              lMinCount    = SM_MINCOUNT_TRIMSRFCACHE ;          
                              break ; 

         case SM_OC_BREP    : lNewCount    = lBrepCacheCount ;
                              lNewByteSize = lBrepCacheByteSize ;          
                              lMinCount    = SM_MINCOUNT_BREPCACHE ;          
                              break ;

         default            : SE(SM_ERR) ; 
                              lNewCount    = lCurveCacheCount ;
                              lNewByteSize = lCurveCacheByteSize ;          
                              lMinCount    = SM_MINCOUNT_CURVECACHE ;          
                              break ;             
       } // end switch on cacheType

      // for caches with previously nested cache_sizes
      if (lStackDepth > 0) 
        {
          // make new_size as large or larger than current_size
          lNewCount    = smos_Max(lNewCount,    pGlobalCache->m_spCaches[i]->m_sMaxCountStack[lStackDepth-1]);
          lNewByteSize = smos_Max(lNewByteSize, pGlobalCache->m_spCaches[i]->m_sMaxByteSizeStack[lStackDepth-1]);
        }

      // add the new_size to the Cache's stack of nested queue_sizes
      pGlobalCache->m_spCaches[i]->m_sMaxCountStack.Add(lNewCount);
      pGlobalCache->m_spCaches[i]->m_sMaxByteSizeStack.Add(lNewByteSize);
      pGlobalCache->m_spCaches[i]->SetMinCount(lMinCount) ; 
    }

#ifdef SM_DEBUG_CODE
    // when debugging make an array of all the created cacheMgrs
    if (pGlobalCache->s_pCacheStack == NULL) 
      {
        pGlobalCache->s_pCacheStack = new (*pGlobalCache->m_spOuterCacheMgr->GetContext()) 
                                          SmTArray<SmCacheMgr*>(*pGlobalCache->m_spOuterCacheMgr->GetContext());
      }
    pGlobalCache->s_pCacheStack->Add(this);
#endif
} // end SmCacheMgr::SmCacheMgr Default Constructor

/*******************************************************************//**
PURPOSE: Destructor for SmCacheMgr.  It automatically cleans up excess
    cache elements if size is reduced.  

NOTES: See SmCacheMgr constructor.
***********************************************************************/
SmCacheMgr::~SmCacheMgr()
{
  // get the Cache's managed by this object
  SmGlobalCache *pGlobalCache = m_crOwningContext.GetGlobalCache();

#ifdef SM_DEBUG_CODE
    if (pGlobalCache->s_pCacheStack->GetLast() != this) {
        SE(SM_ERR);  // Some sort of cache manager problem ??
    }
    pGlobalCache->s_pCacheStack->RemoveLast();
    if(pGlobalCache->s_pCacheStack->GetSize() == 0) {
        SM_ASSERT(pGlobalCache->s_pCacheStack != NULL) ; delete pGlobalCache->s_pCacheStack ;
        pGlobalCache->s_pCacheStack = NULL;
    }
#endif

  // for every cache in the GlobalCache container
  for (ULONG i=0; i<pGlobalCache->m_slNumCaches; i++) {
      // skip unused caches
      if (!pGlobalCache->m_spCaches[i]) continue;

      // get cache nested_size depth
      ULONG lOldStackDepth    = pGlobalCache->m_spCaches[i]->m_sMaxCountStack.GetSize();

      // pop the cache nested_size depth
      pGlobalCache->m_spCaches[i]->m_sMaxCountStack.RemoveLast(); 
      pGlobalCache->m_spCaches[i]->m_sMaxByteSizeStack.RemoveLast(); 

      // get the previous cache size limits
      ULONG lMaxCount    = pGlobalCache->m_spCaches[i]->m_sMaxCountStack.GetLast();
      ULONG lMaxByteSize = pGlobalCache->m_spCaches[i]->m_sMaxByteSizeStack.GetLast();

      // shrink the cache down to the previous cache size limits - one object at a time
      while(   lMaxCount    < pGlobalCache->m_spCaches[i]->m_sCacheObjs.GetSize()
            || (   lMaxByteSize > 0
                && lMaxByteSize < pGlobalCache->m_spCaches[i]->GetByteSize()))
        {
          SmObject *pDelCacheObj = (SmObject*)pGlobalCache->m_spCaches[i]->m_sCacheObjs.GetLast();
          SmObject *pDelBaseObj  = (SmObject*)pGlobalCache->m_spCaches[i]->m_sBaseObjs.GetLast();

          pGlobalCache->m_spCaches[i]->m_sCacheMap.RemoveKey(pDelBaseObj);
          pGlobalCache->m_spCaches[i]->m_sCacheObjs.RemoveLast();
          pGlobalCache->m_spCaches[i]->m_sBaseObjs.RemoveLast();

          SM_ASSERT(pDelCacheObj != NULL) ; delete pDelCacheObj ; pDelCacheObj = NULL ;

        } // end while cache length is larger than previous cache_size
          
      // when we are popping back to the level 1 cache_size depth
      if (lOldStackDepth == 2) 
        {
          // remove the GlobalCache ObjectType cache object
          SM_ASSERT(pGlobalCache->m_spCaches[i] != NULL) ; delete pGlobalCache->m_spCaches[i] ;
          pGlobalCache->m_spCaches[i]     = NULL;
          pGlobalCache->m_spOuterCacheMgr = NULL;
        }
  } // end iter every GlobalCache Cache Object

} // end SmCacheMgr::~SmCacheMgr destructor

/*******************************************************************//**
PURPOSE: Add an object cache element to the appropriate cache queue
    and if the cache size is exceeded then remove the oldest cache element.

NOTES: 
***********************************************************************/
void SmCacheMgr::AddToObjectCache
  (SmObjectCacheType eObjectCacheType,   // in : oneof SM_OC_CURVE
                                         //            SM_OC_SURFACE        
                                         //            SM_OC_TRIMSRF
                                         //            SM_OC_BREP
   const SmObject   * cpObject,          // in : Target Object
   const SmCacheObj * cpObjectCache)     // in : Its CacheObject
{
  // get object's context and the context's global cache queues
  const SmContext *cpContext = cpObject->GetContext();
  if (!cpContext) { SE(SM_ERR); 
                    return;
                  }

  SmGlobalCache *pGlobalCache = cpContext->GetGlobalCache();
  if (!pGlobalCache) { SE(SM_ERR); 
                       return; 
                     }

  // Add ObjectCache to appropriate global cache queue
  pGlobalCache->m_spCaches[eObjectCacheType]->AddTo(cpObject,cpObjectCache);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;

      // accumulate max cache queue sizes
      cpContext->GetGlobalCacheSize(pGlobalCache->m_sCurrentCacheSizes) ;
      pGlobalCache->m_sMaxCacheSizes.SaveMaximums(pGlobalCache->m_sCurrentCacheSizes) ;
      if(bDebugMe)
        {
          cpContext->Dump() ;
          pGlobalCache->m_sCurrentCacheSizes.Dump(FALSE, FALSE) ;
          pGlobalCache->m_sMaxCacheSizes.Dump(FALSE, TRUE) ;
        }

#endif // SM_DEBUG_CODE
  
  return;

} // end SmCacheMgr::AddToObjectCache

/*******************************************************************//**
PURPOSE: Destructor for SmCache - makes sure all caches are deleted

NOTES: 
***********************************************************************/
SmCache::~SmCache()
{
    for (ULONG i=0; i<m_sCacheObjs.GetSize(); i++) {
        SmObject *pCacheObj = (SmObject*)m_sCacheObjs[i];
        SM_ASSERT(pCacheObj != NULL) ; delete pCacheObj ; pCacheObj = NULL ;
    }

} // end SmCache::~SmCache


/*******************************************************************//**
PURPOSE: add entry to cache pointer lists, optionally delete old
            entries when the cache length is over full.

NOTES: 
  remove and SoftDelete old cache for cpBaseOBj if it exists, so
  It is not possible to add multiple caches for the same object.  
       
  remove and SoftDelete end-of-cache entries when queue is over full.
***********************************************************************/
void SmCache::AddTo
  (const SmObject   *cpBaseObj,     // in : Topology object
   const SmCacheObj *cpCacheObj)    // in : associated object cache
{
  ULONG lIndex;
  SmCacheObj *pExistingCacheObj = GetIndex(cpBaseObj,lIndex);

  // remove old versions of this object's cache
  if (pExistingCacheObj)
    {
      // remove the key, object, and cache pointers from their associated pointer lists
      m_sCacheMap.RemoveKey(cpBaseObj);
      m_sCacheObjs.RemoveAt(lIndex,1);
      m_sBaseObjs.RemoveAt(lIndex,1);

      // SoftDelete the old object cache when its unique
      if (pExistingCacheObj != cpCacheObj) 
        {
          SmCacheMgr::SoftDelete(pExistingCacheObj);
        }
    }

  // Insert at beginning of cache
  m_sBaseObjs.InsertAt(0,(SmObject*)cpBaseObj,1);
  m_sCacheObjs.InsertAt(0,(SmObject*)cpCacheObj,1);
  m_sCacheMap.SetAt((SmObject*)cpBaseObj,(SmObject*)cpCacheObj);

  // get cache size limits: 1. ItemCount, 2. Byte Size
  ULONG lMinCount    = GetMinCount() ;
  ULONG lMaxCount    = m_sMaxCountStack   [m_sMaxCountStack.GetSize()   -1]; 
  ULONG lMaxByteSize = m_sMaxByteSizeStack[m_sMaxByteSizeStack.GetSize()-1];
  ULONG lCount       = m_sCacheObjs.GetSize() ;
  ULONG lByteSize    = lMaxByteSize == 0 ? 0 : GetByteSize() ;

#ifdef SM_DEBUG_CODE
  SmObjectCacheType eCacheType = GetCacheType() ;
#endif
   
  // Remove and SoftDelete end of cache entries when cache length is too long
  while(   (   lMinCount > lCount)
        && (   lMaxCount < lCount
            || (   lMaxByteSize > 0
                && lMaxByteSize < lByteSize))) 
    {
#ifdef SM_DEBUG_CODE
      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff,_T("INFORM: Popped Full %s Cache Stack, Count:[%lu,%lu], ByteSize[%lu, %lu]\n"),
                       (  (eCacheType == SM_OC_CURVE  ) ? _T("Curve  ")
                        : (eCacheType == SM_OC_SURFACE) ? _T("Surface")
                        : (eCacheType == SM_OC_TRIMSRF) ? _T("TrimSrf")
                           : (eCacheType == SM_OC_BREP   ) ? _T("Brep   ") : _T("UnKnown")
                           ),
                       lCount,    lMaxCount,
                       lByteSize, lMaxByteSize);
      WARN(sBuff) ;
#endif

      SmCacheObj *pDelCacheObj = (SmCacheObj*)m_sCacheObjs.GetLast();
      SmObject   *pDelBaseObj  = (SmObject*)  m_sBaseObjs.GetLast();

      m_sCacheObjs.RemoveLast();
      m_sBaseObjs.RemoveLast();
      if (pDelBaseObj) { m_sCacheMap.RemoveKey(pDelBaseObj); }
      if (pDelCacheObj) 
        {
          SmCacheMgr::SoftDelete(pDelCacheObj);
        }

      // prepare for next iteration
      lCount       = m_sCacheObjs.GetSize() ;
      lByteSize    = lMaxByteSize == 0 ? 0 : GetByteSize() ;         

    } // end while need to remove cacheObjs from queue     

  // all done
  return;

} // end SmCache::AddTo          

/*******************************************************************//**
PURPOSE: get size in bytes of memory used and memory allocated for
  all Objects in the m_sCacheObjs list.

NOTES: 
***********************************************************************/
ULONG SmCache::GetMemoryUsed    // rtn: total memory currently used by the cache       
  (ULONG &rlMemoryAllocated,    // out: total larger memory sized pre-allocated for cache use
   ULONG &rlMaxCacheUsed,       // out: Max Single CacheObj used memory     
   ULONG &rlMaxCacheAllocated)  // out: Max Single CacheObj allocated memory
 const
{
  // init output
  rlMemoryAllocated            = 0 ;
  rlMaxCacheUsed               = 0 ;
  rlMaxCacheAllocated          = 0 ;
                               
  // locals                    
  ULONG lMemoryUsed            = 0 ;
  ULONG lThisMemoryUsed        = 0 ;
  ULONG lThisMemoryAllocated   = 0 ;
  SmObjectCacheType eCacheType = GetCacheType() ;

  // for every cache object
  for(ULONG ii=0;ii<m_sCacheObjs.GetSize();ii++)
    {
      // get cacheObj memory sizes
      switch(eCacheType)
        {
          case SM_OC_CURVE   : lThisMemoryUsed = ((SmCurveCache *)  m_sCacheObjs[ii])->GetMemoryUsed(lThisMemoryAllocated) ; break ;
          case SM_OC_SURFACE : lThisMemoryUsed = ((SmSurfaceCache *)m_sCacheObjs[ii])->GetMemoryUsed(lThisMemoryAllocated) ; break ;
          case SM_OC_TRIMSRF : lThisMemoryUsed = ((SmTrimSrfCache *)m_sCacheObjs[ii])->GetMemoryUsed(lThisMemoryAllocated) ; break ;
          case SM_OC_BREP    : lThisMemoryUsed = ((SmBrepCache *)   m_sCacheObjs[ii])->GetMemoryUsed(lThisMemoryAllocated) ; break ;
          case SM_OC_UNKNOWN : lThisMemoryUsed = 0 ; 
                               lThisMemoryAllocated = 0 ; 
                               break ;
          case SM_OC_CACHE_TYPE_COUNT:
              break;
        }

      // save the max
      if(lThisMemoryUsed      > rlMaxCacheUsed)      rlMaxCacheUsed      = lThisMemoryUsed ;
      if(lThisMemoryAllocated > rlMaxCacheAllocated) rlMaxCacheAllocated = lThisMemoryAllocated ;

      // accumulate the total
      lMemoryUsed       += lThisMemoryUsed ;
      rlMemoryAllocated += lThisMemoryAllocated ;

    } // end iter every cache object
 
 // all done
 return(lMemoryUsed) ; 

} // end SmCache::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Get cache queue type by looking at the type of the 1st stored Object cache

NOTES: 
***********************************************************************/
SmObjectCacheType SmCache::GetCacheType() const          
{ 
  // get 1st stored object cache item
  SmObject *pObjectCache = m_sCacheObjs.GetSize() > 0 ? m_sCacheObjs[0] : NULL ;

  // switch on object type to infer this cache queue's type
  return (  (pObjectCache == NULL)                        ? SM_OC_UNKNOWN
          : (pObjectCache->IsKindOf(SmTrimSrfCache_TYPE)) ? SM_OC_TRIMSRF
          : (pObjectCache->IsKindOf(SmSurfaceCache_TYPE)) ? SM_OC_SURFACE
          : (pObjectCache->IsKindOf(SmCurveCache_TYPE))   ? SM_OC_CURVE
          : (pObjectCache->IsKindOf(SmBrepCache_TYPE))    ? SM_OC_BREP 
          : SM_OC_UNKNOWN ) ;

} // end SmCache::GetCacheType 

/*******************************************************************//**
PURPOSE: Mark this cache object as being used so it will not be
   deleted.  Mark the number of times it is being used so that we
   will know when it is deletable.

NOTES: You should not really use this directly - use
   SmCacheCheckOutIn instead because it does automatic cleanup.
***********************************************************************/
void SmCacheMgr::CheckOutCache
  (SmCacheObj * pCache)         // in : target object cache
{
  if (pCache) 
    {
      (pCache->m_lUseCount)++;
    }

} // end SmCacheMgr::CheckOutCache

/*******************************************************************//**
PURPOSE: We are done using this cache.  If it is marked as being
   removed then it can be deleted until the use count gets back to 
   zero.

NOTES: You should not really use this directly - use
   SmCacheCheckOutIn instead because it does automatic cleanup.
***********************************************************************/
void SmCacheMgr::CheckInCache
  (SmCacheObj *pCache)        // in : target object cache
{
  if (pCache) 
    {
      (pCache->m_lUseCount)--;
      if (pCache->m_bDeleted) { SmCacheMgr::SoftDelete(pCache); }
    }

} // end SmCacheMgr::CheckInCache

/*******************************************************************//**
PURPOSE: Soft delete this cache.  Delete it only if there are no 
    uses of it.

NOTES: 
***********************************************************************/
void SmCacheMgr::SoftDelete
  (SmCacheObj *pCache)         // in : target Cache
{
  if((pCache->m_lUseCount) == 0) { SM_ASSERT(pCache != NULL) ; delete pCache ; pCache = NULL ; }
  else                         { (pCache->m_bDeleted) = TRUE; }

} // end SmCacheMgr::SoftDelete

/*******************************************************************//**
PURPOSE: Remove the cache of an object but do not delete the cache.

NOTES: 
***********************************************************************/
void SmCache::RemoveObjCache
  (ULONG lIndex)             // in : index for target ObjectCache
{
    SmObject *pDelBaseObj = m_sBaseObjs[lIndex];
    m_sCacheMap.RemoveKey(pDelBaseObj);
    m_sCacheObjs.RemoveAt(lIndex,1);
    m_sBaseObjs.RemoveAt(lIndex,1);

} // end SmCache::RemoveObjCache

/*******************************************************************//**
PURPOSE: remove all entries for cpBaseObj                           

NOTES: SoftDelete removed associated object caches
***********************************************************************/
void SmCache::DeleteObjsCache
  (const SmObject * cpBaseObj)  // in : target object
{
   while (TRUE) 
     {  // removes multiple occurrences
       ULONG lIndex;
       SmCacheObj *pCacheObj = GetIndex(cpBaseObj,lIndex);
       if (!pCacheObj) 
         break;


       SmObject *pDelBaseObj = m_sBaseObjs[lIndex];
       m_sCacheMap.RemoveKey(pDelBaseObj);
       m_sCacheObjs.RemoveAt(lIndex,1);
       m_sBaseObjs.RemoveAt(lIndex,1);


       SmCacheMgr::SoftDelete(pCacheObj);
     }
   return;  

} // end SmCache::DeleteObjsCache
    
/*******************************************************************//**
PURPOSE: Get an item from the cache corresponding to the base object.
    Note that if none exists it will return NULL.

NOTES: 
***********************************************************************/
SmCacheObj * SmCache::GetIndex(const SmObject * cpBaseObj, ULONG & rlIndexFound)
{
    rlIndexFound = 0;
    void *pCache = m_sCacheMap.GetValueAt((void*)cpBaseObj);
    if (!pCache) {
        return NULL;
    }
    for (ULONG i=0; i<m_sBaseObjs.GetSize(); i++) {
        if (cpBaseObj == (SmObject*)m_sBaseObjs[i]) {
            rlIndexFound = i;
            return (SmCacheObj*)m_sCacheObjs[i];
        }
    }
    return NULL;

} // end SmCache::GetIndex

/*******************************************************************//**
PURPOSE: Get an item from the cache corresponding to the base object.
    Note that if none exists it will return NULL.

NOTES: 
***********************************************************************/
SmCacheObj * SmCache::GetFrom(const SmObject * cpBaseObj)
{
    return  (SmCacheObj*)m_sCacheMap.GetValueAt( (void*)cpBaseObj );

} // end SmCache::GetFrom

//      /*******************************************************************//**
//      PURPOSE:  Return pointer to object's ObjectCache and remove the ObjectCache
//        from the global Cache Queue.  
//      
//      
//      NOTES:
//        After this call the ObjectCache is no longer in the global cache queue.
//                The calling function is responsible for deleting the ObjectCache.
//      
//        Returns NULL when ObjectCache does not exist, or the Object or the 
//                Global Cache Queues has a problem, in which case a warning
//                is output.
//      ***********************************************************************/
//      SmCacheObj* SmCacheMgr::GetAndRemoveObjectCache
//        (SmObjectCacheType eObjectCacheType,   // in : oneof SM_OC_CURVE
//                                               //            SM_OC_SURFACE        
//                                               //            SM_OC_TRIMSRF
//                                               //            SM_OC_BREP           
//         const SmObject * cpObject)            // in : target object
//      {
//        // Check state - Object has a Context
//        const SmContext *cpContext = cpObject->GetContext();
//        if (!cpContext) 
//          {
//            SE(SM_ERR);   // Should not try to create a cache for an object
//                          // which has no context.
//            return NULL;
//          }
//      
//        // get global cache queues through object's context
//        SmGlobalCache *pGlobalCache = cpContext->GetGlobalCache();
//      
//        // check state - global cache queue of type eObjectCacheType doesn't exist
//        if (pGlobalCache->m_spCaches[eObjectCacheType]==NULL) 
//          {
//            SE(SM_ERR);
//            return NULL;
//          }
//      
//        // retrieve the ObjectCache for the Object from the specified cache queue
//        SmCacheObj *pCache = pGlobalCache->m_spCaches[eObjectCacheType]->GetFrom(cpObject);
//        if (pCache == NULL) return NULL;
//      
//        // get ObjectCache's index in cache queue
//        ULONG lIndex;
//        pGlobalCache->m_spCaches[eObjectCacheType]->GetIndex(cpObject,lIndex);
//      
//        // remove the Object Cache from the cache queue
//        pGlobalCache->m_spCaches[eObjectCacheType]->RemoveObjCache(lIndex);
//      
//        return pCache;
//      
//      } // end SmCacheMgr::GetAndRemoveObjectCache

/*******************************************************************//**
PURPOSE: Gets the existing cache of a geometry object, or creates a new one
    if none exists, and registers it with the global cache queue so it will be available
    next time.

    1. Check State (cpObject has an SmContext and SmContext has an m_pGlobalCache)
    2. Get Object's existing cache of eObjectCacheType from
            m_pGlobalCache->m_spCaches[eObjectCacheType]
    3a. If(cache exists) 
        3a.1. If cache is near end of the queue in
              SmContext->m_pGlobalCache->m_spCaches[eObjectCacheType],
              Move it to the front.
    3b. else(cache does not exist)
        3b.1. Build the cache of eObjectCacheType
        3b.2. Add it to the cache_queue in
              SmContext->m_pGlobalCache->m_spCaches[eObjectCacheType].
    4. return the object's cache

NOTES: 
***********************************************************************/
SmCacheObj* SmCacheMgr::GetOrCreateObjectCache
  (SmObjectCacheType eObjectCacheType, // oneof: SM_OC_CURVE          
                                       //        SM_OC_SURFACE        
                                       //        SM_OC_TRIMSRF
                                       //        SM_OC_BREP           
   const SmObject * cpObject)          // in   : target object of type
                                       //          SmCurve
                                       //          SmSurface
                                       //          SmBrep
{
  // check state - the object must have a memory context.
  const SmContext *cpContext = cpObject->GetContext();
  if (!cpContext) { SE(SM_ERR) ; 
                    return NULL;
                  }

  // check state - the context's GlobalCache must have a cache_queue of type eObjectCacheType.
  SmGlobalCache *pGlobalCache = cpContext->GetGlobalCache();
  if (pGlobalCache->m_spCaches[eObjectCacheType]==NULL) { SE(SM_ERR);
                                                          return NULL;
                                                        }

  // get object's existing cache (NULL for none)
  SmCacheObj *pOldCache = pGlobalCache->m_spCaches[eObjectCacheType]->GetFrom(cpObject);

  // build or reuse object's existing cache
  //  virtual call to oneof SmCacheMgrBrep::CacheMakeOrValidate()  to build a SmBrepCache    object
  //                        SmCacheMgrTSrf::CacheMakeOrValidate()  to build a SmTrimSrfCache object
  //                        SmCacheMgrSrf ::CacheMakeOrValidate()  to build a SmSurfaceCache object
  //                        SmCacheMgrCrv ::CacheMakeOrValidate()  to build a SmCurveCache   object
  SmCacheObj *pCache;
  if(SM_SUCCESS != pGlobalCache->m_spOuterCacheMgr->CacheMakeOrValidate(eObjectCacheType,
                                                                        cpObject,
                                                                        pOldCache,
                                                                        pCache)) 
    { return NULL; }
  if (pCache == NULL) 
    { return NULL; }

  // when reusing an old cache - move it to the front of the cache queue
  if(pCache == pOldCache) 
    {
      // get cache queue length and index for this cache and cacheType
      ULONG lIndex;
      SmCacheObj *pCObj = pGlobalCache->m_spCaches[eObjectCacheType]->GetIndex(cpObject,lIndex);
      if ( pCObj == NULL )
        { NERN( NULL ) } // (for breakpoint)
      SmTArray<ULONG> & rMaxCountStack = pGlobalCache->m_spCaches[eObjectCacheType]->m_sMaxCountStack;

      // when cache's index is in 2nd half of the queue
      ULONG lTooCloseToEnd = (ULONG)(rMaxCountStack.GetLast()) / 2;
      if ( lIndex + lTooCloseToEnd > (ULONG)(rMaxCountStack.GetLast()) ) 
        {
          // remove and re-add pCache from its cache_queue to place it at the front.
          pGlobalCache->m_spCaches[eObjectCacheType]->RemoveObjCache(lIndex);
          pGlobalCache->m_spCaches[eObjectCacheType]->AddTo(cpObject,pCache);
        }
      
      // all done
      return pCache;

    } // end using an old cache check

  // else, using a new cache - add it to appropriate pGlobalCache cache queue

  // NOTE: For some reason trimmed surfaces add their own caches 
  //       to the GlobalCache->ObjectType cache_queue internally
  //       prior to tessellation.
  //       So don't do it here.
  //
  //       GWC note: I'm guessing this is done so that the preconditioning
  //                 for the 2nd phase of SmTrimSrfCache construction which 
  //                 builds UVTrimCurves does not end up causing nested
  //                 SmTrimSrfCache construction calls when the SmFace::CreateUVTrimCurves()
  //                 function starts calling the global solvers to actually build
  //                 missing UVTrimCurves.  Those Solve calls will call GetOrCreateObjectCache().
  if (eObjectCacheType != SM_OC_TRIMSRF) 
    {
      pGlobalCache->m_spCaches[eObjectCacheType]->AddTo(cpObject,pCache);
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;

      // accumulate max cache queue sizes
      cpContext->GetGlobalCacheSize(pGlobalCache->m_sCurrentCacheSizes) ;
      pGlobalCache->m_sMaxCacheSizes.SaveMaximums(pGlobalCache->m_sCurrentCacheSizes) ;
      if(bDebugMe)
        {
          cpContext->Dump() ;
          pGlobalCache->m_sCurrentCacheSizes.Dump(FALSE, FALSE) ;
          pGlobalCache->m_sMaxCacheSizes.Dump(FALSE, TRUE) ;
        }

#endif // SM_DEBUG_CODE

  // all done
  return pCache;

} // end SmCacheMgr::GetOrCreateObjectCache

/*******************************************************************//**
PURPOSE: Get an existing cache for the target object if one exists
            else return NULL.

NOTES: Note that if the object does not have a context it
   can not live in the cache.
***********************************************************************/
SmCacheObj * SmCacheMgr::GetObjectCache
  (SmObjectCacheType eObjectCacheType,  // in : oneof SM_OC_CURVE
                                        //            SM_OC_SURFACE        
                                        //            SM_OC_TRIMSRF
                                        //            SM_OC_BREP
   const SmObject * cpObject)           // in : target object
{
  // no work - no object
  if(cpObject == NULL) return NULL ;

  // locals
  SmCacheObj      * pOldCache = NULL;
  const SmContext * cpContext = cpObject->GetContext();

  // no work - no context
  if (!cpContext) 
    { return NULL; }

  // get context->GlobalCache
  SmGlobalCache *pGlobalCache = cpContext->GetGlobalCache();

  // when Caches of eObjectCacheType are available
  if (pGlobalCache->m_spCaches[eObjectCacheType]) 
    {
      // fetch the cashe for the object
      pOldCache = pGlobalCache->m_spCaches[eObjectCacheType]->GetFrom(cpObject);
    }

  // all done
  return pOldCache;

} // end SmCacheMgr::GetObjectCache

/*******************************************************************//**
PURPOSE: Delete the cache of the object.  Note that if the object does
    not have an owning context (NULL) it can not have a cache.

NOTES: Will not return an error if cache is not there.
***********************************************************************/
void SmCacheMgr::DeleteObjectsCache
  (SmObjectCacheType eObjectCacheType,    // in : oneof SM_OC_CURVE
                                          //            SM_OC_SURFACE        
                                          //            SM_OC_TRIMSRF
                                          //            SM_OC_BREP
   const SmObject * cpObject)             // in : target object
{
  const SmContext *cpContext = cpObject->GetContext();
  if (!cpContext) 
    return;

  SmGlobalCache *pGlobalCache = cpContext->GetGlobalCache();


  if (pGlobalCache->m_spCaches[eObjectCacheType]) 
    { pGlobalCache->m_spCaches[eObjectCacheType]->DeleteObjsCache(cpObject); }

} // end SmCacheMgr::DeleteObjectsCache

/*******************************************************************//**
PURPOSE: If this method is reached then there is no cache of the type
    requested.  Just return NULL.  Hopefully next time it will request
    something that is available.

NOTES: Use SmCacheMgrCrv, SmCacheMgrSrf, or SmCacheMgrTSrf
   to initialize the cache manager not this method.
***********************************************************************/
SmStatus SmCacheMgr::CacheMakeOrValidate
  (SmObjectCacheType ,                       // in : eObjectCacheType = oneof SM_OC_CURVE
                                             //            SM_OC_SURFACE        
                                             //            SM_OC_TRIMSRF
                                             //            SM_OC_BREP
   const SmObject * ,                        // in : cpObject = target object
   SmCacheObj     * ,                        // in : cpOldCache = existing target Object's ObjectCache or NULL
   SmCacheObj     *& rpNewCache)             // out: ptr to target object's ObjectCache
{
    rpNewCache = NULL;
    return SM_SUCCESS;

} // end SmCacheMgr::CacheMakeOrValidate

/*******************************************************************//**
PURPOSE: Debug Formatted print.

NOTES: 
***********************************************************************/

void SmCache::Dump(void) const
{
    smos_WriteBuffer(_T("     Dumping SmCache - MaxCountStack\n"));
    m_sMaxCountStack.Dump();
    smos_WriteBuffer(_T("     Dumping SmCache - MaxByteSizeStack\n"));
    m_sMaxByteSizeStack.Dump();
    smos_WriteBuffer(_T("     Dumping m_sBaseObjs\n"));
    m_sBaseObjs.Dump();
    smos_WriteBuffer(_T("     Dumping m_sCacheObjs\n"));
    m_sCacheObjs.Dump();

} // end SmCache::Dump

/*******************************************************************//**
PURPOSE: Debug Formatted print.

NOTES: 
***********************************************************************/
void SmCacheMgr::Dump(void) const
{
    smos_WriteBuffer(_T("Dumping SmCacheMgr - cache manager\n"));

} // end SmCacheMgr::Dump


/*******************************************************************//**
PURPOSE: clear all values

NOTES: 
***********************************************************************/
void SmGlobalCacheSize::ReSet()  
{
  // clear values
  m_lCurveQueueLength            = 0 ;  m_lSurfaceQueueLength          = 0 ;     
  m_lCurveQueueCount             = 0 ;  m_lSurfaceQueueCount           = 0 ;      
  m_lCurveQueueUsed              = 0 ;  m_lSurfaceQueueUsed            = 0 ;       
  m_lCurveQueueAllocated         = 0 ;  m_lSurfaceQueueAllocated       = 0 ;  
  m_lCurveQueueCountAtPeakSize   = 0 ;  m_lSurfaceQueueCountAtPeakSize = 0 ;

  m_lMaxCurveCacheUsed           = 0 ;  m_lMaxSurfaceCacheUsed         = 0 ;                     
  m_lMaxCurveCacheAllocated      = 0 ;  m_lMaxSurfaceCacheAllocated    = 0 ;
                  
                                                                                         
  m_lTrimSrfQueueLength          = 0 ;  m_lBrepQueueLength             = 0 ;                         
  m_lTrimSrfQueueCount           = 0 ;  m_lBrepQueueCount              = 0 ;                          
  m_lTrimSrfQueueUsed            = 0 ;  m_lBrepQueueUsed               = 0 ;                           
  m_lTrimSrfQueueAllocated       = 0 ;  m_lBrepQueueAllocated          = 0 ;                      
  m_lTrimSrfQueueCountAtPeakSize = 0 ;  m_lBrepQueueCountAtPeakSize    = 0 ;                                                                                

  m_lMaxTrimSrfCacheUsed         = 0 ;  m_lMaxBrepCacheUsed            = 0 ;                        
  m_lMaxTrimSrfCacheAllocated    = 0 ;  m_lMaxBrepCacheAllocated       = 0 ;
                     

} // end SmGlobalCacheSize::ReSet
                                                                                      
/*******************************************************************//**              
PURPOSE: Return TRUE when all values are zero

NOTES: 
***********************************************************************/
SmBoolean SmGlobalCacheSize::IsZero() 
  const 
{
  // check values
  return(   m_lCurveQueueLength         == 0 &&   m_lSurfaceQueueLength       == 0    
         && m_lCurveQueueCount          == 0 &&   m_lSurfaceQueueCount        == 0    
         && m_lCurveQueueUsed           == 0 &&   m_lSurfaceQueueUsed         == 0    
         && m_lCurveQueueAllocated      == 0 &&   m_lSurfaceQueueAllocated    == 0    
         && m_lMaxCurveCacheUsed        == 0 &&   m_lMaxSurfaceCacheUsed      == 0    
         && m_lMaxCurveCacheAllocated   == 0 &&   m_lMaxSurfaceCacheAllocated == 0      
                                                                                        
         && m_lTrimSrfQueueLength       == 0 &&   m_lBrepQueueLength          == 0         
         && m_lTrimSrfQueueCount        == 0 &&   m_lBrepQueueCount           == 0         
         && m_lTrimSrfQueueUsed         == 0 &&   m_lBrepQueueUsed            == 0         
         && m_lTrimSrfQueueAllocated    == 0 &&   m_lBrepQueueAllocated       == 0          
         && m_lMaxTrimSrfCacheUsed      == 0 &&   m_lMaxBrepCacheUsed         == 0         
         && m_lMaxTrimSrfCacheAllocated == 0 &&   m_lMaxBrepCacheAllocated    == 0) ;
          

} // end SmGlobalCacheSize::IsZero

/*******************************************************************//**
PURPOSE: store maximum values of rGlobalCacheSize in this object

NOTES: 
***********************************************************************/
void SmGlobalCacheSize::SaveMaximums
  (SmGlobalCacheSize &rGlobalCacheSize)
{
  if( m_lCurveQueueLength         < rGlobalCacheSize.m_lCurveQueueLength         )   m_lCurveQueueLength            = rGlobalCacheSize.m_lCurveQueueLength         ;
  if( m_lCurveQueueCount          < rGlobalCacheSize.m_lCurveQueueCount          )   m_lCurveQueueCount             = rGlobalCacheSize.m_lCurveQueueCount          ;
  if( m_lCurveQueueUsed           < rGlobalCacheSize.m_lCurveQueueUsed           )   m_lCurveQueueUsed              = rGlobalCacheSize.m_lCurveQueueUsed           ;
  if( m_lCurveQueueAllocated      < rGlobalCacheSize.m_lCurveQueueAllocated      )   m_lCurveQueueAllocated         = rGlobalCacheSize.m_lCurveQueueAllocated      ;
  if( m_lMaxCurveCacheUsed        < rGlobalCacheSize.m_lMaxCurveCacheUsed        )   m_lMaxCurveCacheUsed           = rGlobalCacheSize.m_lMaxCurveCacheUsed        ;
  if( m_lMaxCurveCacheAllocated   < rGlobalCacheSize.m_lMaxCurveCacheAllocated   ) { m_lMaxCurveCacheAllocated      = rGlobalCacheSize.m_lMaxCurveCacheAllocated   ;
                                                                                     m_lCurveQueueCountAtPeakSize   = m_lCurveQueueCount ;
                                                                                   }                                
                                                                                                                    
  if( m_lSurfaceQueueLength       < rGlobalCacheSize.m_lSurfaceQueueLength       )   m_lSurfaceQueueLength          = rGlobalCacheSize.m_lSurfaceQueueLength       ;
  if( m_lSurfaceQueueCount        < rGlobalCacheSize.m_lSurfaceQueueCount        )   m_lSurfaceQueueCount           = rGlobalCacheSize.m_lSurfaceQueueCount        ;
  if( m_lSurfaceQueueUsed         < rGlobalCacheSize.m_lSurfaceQueueUsed         )   m_lSurfaceQueueUsed            = rGlobalCacheSize.m_lSurfaceQueueUsed         ;
  if( m_lSurfaceQueueAllocated    < rGlobalCacheSize.m_lSurfaceQueueAllocated    )   m_lSurfaceQueueAllocated       = rGlobalCacheSize.m_lSurfaceQueueAllocated    ;
  if( m_lMaxSurfaceCacheUsed      < rGlobalCacheSize.m_lMaxSurfaceCacheUsed      )   m_lMaxSurfaceCacheUsed         = rGlobalCacheSize.m_lMaxSurfaceCacheUsed      ;
  if( m_lMaxSurfaceCacheAllocated < rGlobalCacheSize.m_lMaxSurfaceCacheAllocated ) { m_lMaxSurfaceCacheAllocated    = rGlobalCacheSize.m_lMaxSurfaceCacheAllocated ;
                                                                                     m_lSurfaceQueueCountAtPeakSize = m_lSurfaceQueueCount ;
                                                                                   }                                
                                                                                                                    
  if( m_lTrimSrfQueueLength       < rGlobalCacheSize.m_lTrimSrfQueueLength       )   m_lTrimSrfQueueLength          = rGlobalCacheSize.m_lTrimSrfQueueLength       ;
  if( m_lTrimSrfQueueCount        < rGlobalCacheSize.m_lTrimSrfQueueCount        )   m_lTrimSrfQueueCount           = rGlobalCacheSize.m_lTrimSrfQueueCount        ;
  if( m_lTrimSrfQueueUsed         < rGlobalCacheSize.m_lTrimSrfQueueUsed         )   m_lTrimSrfQueueUsed            = rGlobalCacheSize.m_lTrimSrfQueueUsed         ;
  if( m_lTrimSrfQueueAllocated    < rGlobalCacheSize.m_lTrimSrfQueueAllocated    )   m_lTrimSrfQueueAllocated       = rGlobalCacheSize.m_lTrimSrfQueueAllocated    ;
  if( m_lMaxTrimSrfCacheUsed      < rGlobalCacheSize.m_lMaxTrimSrfCacheUsed      )   m_lMaxTrimSrfCacheUsed         = rGlobalCacheSize.m_lMaxTrimSrfCacheUsed      ;
  if( m_lMaxTrimSrfCacheAllocated < rGlobalCacheSize.m_lMaxTrimSrfCacheAllocated ) { m_lMaxTrimSrfCacheAllocated    = rGlobalCacheSize.m_lMaxTrimSrfCacheAllocated ;
                                                                                     m_lTrimSrfQueueCountAtPeakSize = m_lTrimSrfQueueCount ;
                                                                                   }                                
                                                                                                                    
  if( m_lBrepQueueLength          < rGlobalCacheSize.m_lBrepQueueLength          )   m_lBrepQueueLength             = rGlobalCacheSize.m_lBrepQueueLength          ;
  if( m_lBrepQueueCount           < rGlobalCacheSize.m_lBrepQueueCount           )   m_lBrepQueueCount              = rGlobalCacheSize.m_lBrepQueueCount           ;
  if( m_lBrepQueueUsed            < rGlobalCacheSize.m_lBrepQueueUsed            )   m_lBrepQueueUsed               = rGlobalCacheSize.m_lBrepQueueUsed            ;
  if( m_lBrepQueueAllocated       < rGlobalCacheSize.m_lBrepQueueAllocated       )   m_lBrepQueueAllocated          = rGlobalCacheSize.m_lBrepQueueAllocated       ;
  if( m_lMaxBrepCacheUsed         < rGlobalCacheSize.m_lMaxBrepCacheUsed         )   m_lMaxBrepCacheUsed            = rGlobalCacheSize.m_lMaxBrepCacheUsed         ;
  if( m_lMaxBrepCacheAllocated    < rGlobalCacheSize.m_lMaxBrepCacheAllocated    ) { m_lMaxBrepCacheAllocated       = rGlobalCacheSize.m_lMaxBrepCacheAllocated    ;
                                                                                     m_lBrepQueueCountAtPeakSize    = m_lBrepQueueCount ;
                                                                                   }                                

} // end SmGlobalCacheSize::SaveMaximums

/*******************************************************************//**
PURPOSE: Return Total Sum of all Allocated Memory

NOTES: 
***********************************************************************/
ULONG SmGlobalCacheSize::GetMemoryAllocated
  ()
{
  // Add Allocated memory
  ULONG lMemoryAllocated =   m_lCurveQueueAllocated
                           + m_lSurfaceQueueAllocated
                           + m_lTrimSrfQueueAllocated
                           + m_lBrepQueueAllocated ;

  // all done
  return(lMemoryAllocated) ;

} // end SmGlobalCacheSize::GetMemoryAllocated

/*******************************************************************//**
PURPOSE: Return Total Sum of all Used Memory

NOTES: 
***********************************************************************/
ULONG SmGlobalCacheSize::GetMemoryUsed
  ()
{
  // Add Used memory
  ULONG lMemoryUsed =   m_lCurveQueueUsed
                      + m_lSurfaceQueueUsed
                      + m_lTrimSrfQueueUsed
                      + m_lBrepQueueUsed ;

  // all done
  return(lMemoryUsed) ;

} // end SmGlobalCacheSize::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Debug Formatted print.

NOTES: 
***********************************************************************/
void SmGlobalCacheSize::Dump
  (SmBoolean bSkipZeroSize,    // in : TRUE = skip output when all sizes are zero
                               //      FALSE= always write the report
   SmBoolean bPeakSizeLabel)   // in : TRUE = output label 'Global Cache Peak Size'
                               //      FALSE= output label 'Global Cache Immediate Size'
  const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];

  // no work
  if(bSkipZeroSize && IsZero()) 
    { return ; }

  if(bPeakSizeLabel) 
     smos_WriteBuffer(_T("\nGlobal Cache Queue Peak Size       Total Memory          Max Single Memory"));
  else            
     smos_WriteBuffer(_T("\nGlobal Cache Queue Immediate Size  Total Memory          Max Single Memory"));


   smos_sprintf(sBuff,_T("%s"),_T("\nQUEUE         Count  Length       Used  Allocated         Used  Allocated ")); 
   smos_WriteBuffer(sBuff);


  // CurveCache
  smos_sprintf(sBuff,_T("\nCurve      %8lu%8lu   %8lu   %8lu     %8lu   %8lu   %8lu"), 
                   m_lCurveQueueCount,       
                   m_lCurveQueueLength,      
                   m_lCurveQueueUsed,        
                   m_lCurveQueueAllocated,
                   m_lCurveQueueCountAtPeakSize,   
                   m_lMaxCurveCacheUsed,     
                   m_lMaxCurveCacheAllocated) ;
  smos_WriteBuffer(sBuff);

  // SurfaceCache
  smos_sprintf(sBuff,_T("\nSurface    %8lu%8lu   %8lu   %8lu     %8lu   %8lu   %8lu"),
                   m_lSurfaceQueueCount,       
                   m_lSurfaceQueueLength,      
                   m_lSurfaceQueueUsed,        
                   m_lSurfaceQueueAllocated,
                   m_lSurfaceQueueCountAtPeakSize,   
                   m_lMaxSurfaceCacheUsed,     
                   m_lMaxSurfaceCacheAllocated) ;
  smos_WriteBuffer(sBuff);

  // TrimSrfCache
  smos_sprintf(sBuff,_T("\nTrimSrf    %8lu%8lu   %8lu   %8lu     %8lu   %8lu   %8lu"),
                   m_lTrimSrfQueueCount,       
                   m_lTrimSrfQueueLength,      
                   m_lTrimSrfQueueUsed,        
                   m_lTrimSrfQueueAllocated,
                   m_lTrimSrfQueueCountAtPeakSize,   
                   m_lMaxTrimSrfCacheUsed,     
                   m_lMaxTrimSrfCacheAllocated) ;
  smos_WriteBuffer(sBuff);

  // BrepCache
  smos_sprintf(sBuff,_T("\nBrep       %8lu%8lu   %8lu   %8lu     %8lu   %8lu   %8lu"),
                   m_lBrepQueueCount,       
                   m_lBrepQueueLength,      
                   m_lBrepQueueUsed,        
                   m_lBrepQueueAllocated,
                   m_lBrepQueueCountAtPeakSize,   
                   m_lMaxBrepCacheUsed,     
                   m_lMaxBrepCacheAllocated) ;
  smos_WriteBuffer(sBuff);
  smos_WriteBuffer(_T("\n\n"));


} // end SmGlobalCacheSize::Dump
#else

/*******************************************************************//**
PURPOSE: Mark this cache object as being used so it will not be
   deleted.  Mark the number of times it is being used so that we
   will know when it is deletable.

NOTES: You should not really use this directly - use
   SmCacheCheckOutIn instead because it does automatic cleanup.
***********************************************************************/
void SmCacheMgr::CheckOutCache
  (SmCacheObj * pCache)         // in : target object cache
{
  if (pCache) 
    {
      (pCache->m_lUseCount)++;
    }

} // end SmCacheMgr::CheckOutCache

/*******************************************************************//**
PURPOSE: We are done using this cache.  If it is marked as being
   removed then it can be deleted until the use count gets back to 
   zero.

NOTES: You should not really use this directly - use
   SmCacheCheckOutIn instead because it does automatic cleanup.
***********************************************************************/
void SmCacheMgr::CheckInCache
  (SmCacheObj *pCache)        // in : target object cache
{
  if (pCache) 
    {
      (pCache->m_lUseCount)--;
      if (pCache->m_bDeleted) { SmCacheMgr::SoftDelete(pCache); }
    }

} // end SmCacheMgr::CheckInCache

/*******************************************************************//**
PURPOSE: Soft delete this cache.  Delete it only if there are no 
    uses of it.

NOTES: 
***********************************************************************/
void SmCacheMgr::SoftDelete
  (SmCacheObj *pCache)         // in : target Cache
{
    if (pCache->m_lUseCount == 0)
    {
        SM_ASSERT(pCache != NULL);
        delete pCache;
        pCache = NULL;
    }
  else                         { pCache->m_bDeleted = TRUE; }

} // end SmCacheMgr::SoftDelete

/*******************************************************************//**
PURPOSE: Gets the existing cache of a geometry object, or creates a new one
    if none exists, and registers it with cpObject

    1. Check State (cpObject has an SmContext)
    2. Get Object's existing cache of eObjectCacheType from
            cpObject->GetCacheObj()
    3a. If(cache exists) 
        3a.1. GetCache of eObjectCacheType
    3b. else(cache does not exist)
        3b.1. Build the cache of eObjectCacheType
    4. return the object's cache

NOTES: 
The value of eObjectCacheType must match the type of cpObject:
     -----------------------------------------------------
     | cpObject Type     |     SmObjectCacheType Value   |
     |-------------------|-------------------------------|
     | SmCurve           |     SM_OC_CURVE               |
     |-------------------|-------------------------------|
     | SmSurface         |     SM_OC_SURFACE             |
     |                   |     SM_OC_TRIMSRF             |
     |-------------------|-------------------------------|
     | SmBrep            |     SM_OC_BREP                |
     | SmPolyBrep        |                               |
     -----------------------------------------------------
***********************************************************************/
SmCacheObj* SmCacheMgr::GetOrCreateObjectCache
  (SmObjectCacheType eObjectCacheType, // oneof: SM_OC_CURVE          
                                       //        SM_OC_SURFACE        
                                       //        SM_OC_TRIMSRF
                                       //        SM_OC_BREP           
   const SmAObject * cpObject)         // in   : target object of type
                                       //          SmCurve
                                       //          SmSurface
                                       //          SmBrep
{
  // check state - the object must have a memory context.
  const SmContext *cpContext = cpObject->GetContext();
  if (!cpContext) { SE(SM_ERR) ; 
                    return NULL;
                  }

  SmCacheObj * pOldCache = cpObject->GetCacheObj();

  // build or reuse object's existing cache
  //  virtual call to oneof SmSAGObject::CacheMakeOrValidate()  to build a SmBrepCache    object
  //                        SmSurface  ::CacheMakeOrValidate()  to build a SmTrimSrfCache object
  //                                                                  Or a SmSurfaceCache object
  //                        SmCurve    ::CacheMakeOrValidate()  to build a SmCurveCache   object
  SmCacheObj *pCache;
  if(SM_SUCCESS != cpObject->CacheMakeOrValidate(eObjectCacheType,
                                                 cpObject,
                                                 pOldCache,
                                                 pCache)) 
    { return NULL; }
  if (pCache == NULL) 
    { return NULL; }

  // all done
  return pCache;

} // end SmCacheMgr::GetOrCreateObjectCache

/*******************************************************************//**
PURPOSE: Get an existing cache for the target object if one exists
            else return NULL.

NOTES: Note that if the object does not have a context it
   can not live in the cache.
***********************************************************************/
SmCacheObj * SmCacheMgr::GetObjectCache
  (SmObjectCacheType eObjectCacheType,  // NotUsed: in : oneof SM_OC_CURVE
                                        //            SM_OC_SURFACE        
                                        //            SM_OC_TRIMSRF
                                        //            SM_OC_BREP
   const SmAObject * cpObject)          // in : target object
{
  SM_REF1(eObjectCacheType) ;
  // no work - no object
  if(cpObject == NULL) return NULL ;

  // locals
  SmCacheObj      * pOldCache = NULL;

  pOldCache = cpObject->GetCacheObj();

  // all done
  return pOldCache;

} // end SmCacheMgr::GetObjectCache

/*******************************************************************//**
PURPOSE: Delete the cache of the object.  Note that if the object does
    not have an owning context (NULL) it can not have a cache.

NOTES: Will not return an error if cache is not there.
***********************************************************************/
void SmCacheMgr::DeleteObjectsCache
  ( const SmAObject * cpObject)             // in : target object
{
    SmCacheObj *pCacheObj = cpObject->GetCacheObj();

    if ( pCacheObj )
    { SmCacheMgr::SoftDelete( pCacheObj ); }

    cpObject->SetCacheObj( NULL );

} // end SmCacheMgr::DeleteObjectsCache

/*******************************************************************//**
PURPOSE: Delete the cache of the object.  Note that if the object does
    not have an owning context (NULL) it can not have a cache.

NOTES: Will not return an error if cache is not there.
***********************************************************************/
void SmCacheMgr::DeleteObjectsCache
  (SmObjectCacheType eObjectCacheType,    // NotUsed: in : oneof SM_OC_CURVE
                                          //            SM_OC_SURFACE        
                                          //            SM_OC_TRIMSRF
                                          //            SM_OC_BREP
   const SmAObject * cpObject)            // in : target object
{
    SM_REF1(eObjectCacheType) ;
    SmCacheObj *pCacheObj = cpObject->GetCacheObj();

    if ( pCacheObj )
    { SmCacheMgr::SoftDelete( pCacheObj ); }

    cpObject->SetCacheObj( NULL );

} // end SmCacheMgr::DeleteObjectsCache


/*******************************************************************//**
PURPOSE: Debug Formatted print.

NOTES: 
***********************************************************************/
void SmCacheMgr::Dump(void) const
{
    smos_WriteBuffer(_T("Dumping SmCacheMgr - cache manager\n"));

} // end SmCacheMgr::Dump

#endif //SM_USE_GLOBAL_CACHE
