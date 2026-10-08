// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCacheMgr.h
* PURPOSE: Header file for cache manager.
**********************************************************************/

#ifndef __SMCACHEMGR_H__
#define __SMCACHEMGR_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMOBJECT_H__
#include <SmObject.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMMAPPTRTOPTR_H__
#include <SmMapPtrToPtr.h>
#endif

//#ifndef __MUTEX__
//#define __MUTEX__
//#include <mutex>
//#endif

#ifndef __ATOMIC__
#    define __ATOMIC__
#    include <atomic>
#endif

class SmCache;    // needed for gcc4.x
class SmCacheMgr; // needed for gcc4.x

// #define FS_LOCK_CACHE(cache) std::lock_guard<std::recursive_mutex> lock(cache -> mCacheMutex) // JLMCC removced to address compiler warning. Revisit.

/*******************************************************************//**
PURPOSE: Abstract class for all cache objects.

NOTES: 
***********************************************************************/
class SM_EXPORT SmCacheObj : public SmObject
{
    friend class SmCacheMgr;
protected:
    ULONG     m_lUseCount;   // used to delay deleting Cache Objects still in use.
                                           // When a cache object is deleted, the actual delete is delayed
                                           // until m_lUseCount goes to zero.
                                           //  SmCacheMgr::CheckOutCache(pCache) increments this value.
                                           //  SmCacheMgr::CheckInCache(pCache)  decrements this value.
                                           // These functions are called by the SmCacheCheckOutIn constructor/destructor.

    SmBoolean  m_bDeleted;    // TRUE  = waiting until m_lUseCount goes to zero to delete this object
                                           // FALSE = no request to delete this object seen yet.
public:
    SmCacheObj() : m_lUseCount(0), m_bDeleted(FALSE) {}

    // rtn: smaller size of actually used memory in bytes
    virtual ULONG GetMemoryUsed( ULONG & rlMemoryAllocated ) const // out: bigger size of all allocated memory in bytes
    { SM_REF1(rlMemoryAllocated) ; return sizeof( *this ); };

} ; // end class SmCacheObj

///*******************************************************************//**
//PURPOSE: List of object cache types.
//
//NOTES: 
//***********************************************************************/
//enum SmObjectCacheType {
//    SM_OC_CURVE     = 0,
//    SM_OC_SURFACE   = 1,
//    SM_OC_TRIMSRF   = 2,
//    SM_OC_BREP      = 3,
//    SM_OC_CACHE_TYPE_COUNT,
//    SM_OC_UNKNOWN
//};

#ifdef SM_USE_GLOBAL_CACHE
/*******************************************************************//**
PURPOSE: Hold various Smcache size measures

NOTES:
***********************************************************************/
class SM_EXPORT SmGlobalCacheSize
{
public:
  // Curve Queue
  ULONG m_lCurveQueueLength          = SM_UNDEF_ULONG ; // Curve Queue max allowed objectCache count        
  ULONG m_lCurveQueueCount           = SM_UNDEF_ULONG ; // Curve Queue current objectCache count    
  ULONG m_lCurveQueueUsed            = SM_UNDEF_ULONG ; // Total Curve Queue used memory            
  ULONG m_lCurveQueueAllocated       = SM_UNDEF_ULONG ; // Total Curve Queue allocated memory 
  ULONG m_lCurveQueueCountAtPeakSize = SM_UNDEF_ULONG ; // Curve Queue ObjectCache Count when m_lMaxCurveCacheAllocated
                                                        //   is updated in SaveMaximums(). 

  ULONG m_lMaxCurveCacheUsed         = SM_UNDEF_ULONG ; // Max Single CurveCache used memory
  ULONG m_lMaxCurveCacheAllocated    = SM_UNDEF_ULONG ; // Max Single CurveCache allocated memory
                                     
  // Surface Queue                                                                                    
  ULONG m_lSurfaceQueueLength          = SM_UNDEF_ULONG ; // Surface Queue max allowed objectCache count 
  ULONG m_lSurfaceQueueCount           = SM_UNDEF_ULONG ; // Surface Queue current objectCache count     
  ULONG m_lSurfaceQueueUsed            = SM_UNDEF_ULONG ; // Total Surface Queue used memory          
  ULONG m_lSurfaceQueueAllocated       = SM_UNDEF_ULONG ; // Total Surface Queue allocated memory     
  ULONG m_lSurfaceQueueCountAtPeakSize = SM_UNDEF_ULONG ; // Surface Queue ObjectCache Count when m_lMaxSurfaceCacheAllocated
                                                          //   is updated in SaveMaximums(). 

  ULONG m_lMaxSurfaceCacheUsed         = SM_UNDEF_ULONG ; // Max Single SurfaceCache used memory
  ULONG m_lMaxSurfaceCacheAllocated    = SM_UNDEF_ULONG ; // Max Single SurfaceCache allocated memory
         
  // TrimSrf Queue                                                                                    
  ULONG m_lTrimSrfQueueLength          = SM_UNDEF_ULONG ; // TrimSrf Queue max allowed objectCache count 
  ULONG m_lTrimSrfQueueCount           = SM_UNDEF_ULONG ; // TrimSrf Queue current objectCache count     
  ULONG m_lTrimSrfQueueUsed            = SM_UNDEF_ULONG ; // Total Trim Surface Queue used memory     
  ULONG m_lTrimSrfQueueAllocated       = SM_UNDEF_ULONG ; // Total Trim Surface Queue allocated memory
  ULONG m_lTrimSrfQueueCountAtPeakSize = SM_UNDEF_ULONG ; // TrimSrf Queue ObjectCache Count when m_lMaxTrimSrfCacheAllocated
                                                          //   is updated in SaveMaximums(). 
                                                                                    
  ULONG m_lMaxTrimSrfCacheUsed         = SM_UNDEF_ULONG ; // Max Single TrimSrfCache used memory
  ULONG m_lMaxTrimSrfCacheAllocated    = SM_UNDEF_ULONG ; // Max Single TrimSrfCache allocated memory

  // Brep Queue
  ULONG m_lBrepQueueLength             = SM_UNDEF_ULONG ; // Brep Queue max allowed objectCache count     
  ULONG m_lBrepQueueCount              = SM_UNDEF_ULONG ; // Brep Queue current objectCache count         
  ULONG m_lBrepQueueUsed               = SM_UNDEF_ULONG ; // Total Brep Queue used memory             
  ULONG m_lBrepQueueAllocated          = SM_UNDEF_ULONG ; // Total Brep Queue allocated memory        
  ULONG m_lBrepQueueCountAtPeakSize    = SM_UNDEF_ULONG ; // Brep Queue ObjectCache Count when m_lMaxBrepCacheAllocated
                                                          //   is updated in SaveMaximums(). 

  ULONG m_lMaxBrepCacheUsed            = SM_UNDEF_ULONG ; // Max Single BrepCache used memory
  ULONG m_lMaxBrepCacheAllocated       = SM_UNDEF_ULONG ; // Max Single BrepCache allocated memory

public:

  // constructor
  SmGlobalCacheSize() { ReSet() ; }

  // clear all values to zero
  void ReSet() ;
  SmBoolean IsZero() const ;

  // store maximum values of rGlobalCacheSize in this object
  void SaveMaximums(SmGlobalCacheSize &rGlobalCacheSize) ;

  // return convenient sums
  ULONG GetMemoryAllocated() ;
  ULONG GetMemoryUsed() ;

  // debug formatted print
  void Dump(SmBoolean bSkipZeroSize,    ///< [in] : TRUE = skip output when all sizes are zero
                                        //      FALSE= always write the report
            SmBoolean bPeakSizeLabel)   ///< [in] : TRUE = output label 'Global Cache Peak Size'
         const ;                        //      FALSE= output label 'Global Cache Immediate Size'

} ; // end class SmGlobalCacheSize


/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
class SmGlobalCache : public SmObject
{
   friend class SmCache;
   friend class SmCacheMgr;

protected:
   ULONG m_slNumCaches;                            // use: set to 4 for the 4 SmObjectCacheTypes
                                                  
   SmCache *m_spCaches[SM_OC_CACHE_TYPE_COUNT];    // use: There is one SmCache ptr for each SmObjectCacheType.
                                                   //      Each SmCache is an array of [baseObject, CacheObject] pairs.
                                                   //      SM_OC_CURVE     = 0,  [default cache array length = 1000] 
                                                   //      SM_OC_SURFACE   = 1,  [default cache array length = 200]
                                                   //      SM_OC_TRIMSRF   = 2,  [default cache array length = 200]
                                                   //      SM_OC_BREP      = 3   [default cache array length = 200]
                                                  
   SmCacheMgr *m_spOuterCacheMgr;                  // ptr to SmCacheMgr that initially allocated the m_spCaches.
                                                   // SmCacheMgr objects can nest with each level increasing the
                                                   // length of the m_spCaches. When a nested SmCacheMgr is destructed
                                                   // the cache returns to its previous size.
#ifdef SM_DEBUG_CODE                              
   SmTArray<SmCacheMgr*> * s_pCacheStack;         
                                                  
public:                                           
   SmGlobalCacheSize m_sMaxCacheSizes ;            // place to accumulate max cache queue and object cache counts and sizes
   SmGlobalCacheSize m_sCurrentCacheSizes ;        // place to store current cache queue and object cache  counts and sizes
                                                  
#endif                                            
                                                  
public:                                           
   // constructor, destructor                     
   SmGlobalCache(ULONG lCurveQueueCount,           ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_CURVE  ]
                 ULONG lSurfaceQueueCount,         ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_SURFACE]
                 ULONG lTrimSrfQueueCount,         ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF]
                 ULONG lBrepQueueCount,            ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_BREP   ]
                                             
                 ULONG lCurveCacheByteSize   = SM_DEFAULT_BYTESIZE_CURVECACHE  ,  ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_CURVE  ], 0 = no limit
                 ULONG lSurfaceCacheByteSize = SM_DEFAULT_BYTESIZE_SURFACECACHE,  ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_SURFACE], 0 = no limit
                 ULONG lTrimSrfCacheByteSize = SM_DEFAULT_BYTESIZE_TRIMSRFCACHE,  ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF], 0 = no limit
                 ULONG lBrepCacheByteSize    = SM_DEFAULT_BYTESIZE_BREPCACHE   ); ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_BREP   ], 0 = no limit

   virtual ~SmGlobalCache();

   // queries
   const SmCacheMgr *GetOuterCacheMgr()                          const { return m_spOuterCacheMgr; }
   const SmCache    *GetCacheQueue(SmObjectCacheType eCacheType) const { return m_spCaches[eCacheType] ; }

} ; // end class SmGlobalCache

/*******************************************************************//**
PURPOSE: This object defines an individual cache used by the 
   cache manager.  It essentially maps the object to its corresponding
   cache data.

   Caches are organized so that an expensive function
   can temporarily increase the cache stack size and have it revert
   back to its old sizes after running without losing any of the
   cache objects already in the cache stack.  Changing of cache stack
   length is done by SmCacheMgr constructor and destructor functions.

NOTES: 
  1. The SmCache contains an array of Objects and ObjectCaches and
     and an SmMapPtrToPtr object to associate the two lists.
  2. These array lengths equal the last entry in m_sMaxCountStack array
  3. The m_sMaxCountStack array provides memory for the idea of nesting 
     a sequence of stack length increases.  Each time a nested SmCacheMgr
     object is created sharing this SmCache, one new entry is added
     to m_sMaxCountStack and the cache length is resized to this number.
     Each time the most nested SmCacheMgr is deleted, the last entry
     is removed and the the cache length is resized to the old number.
     3a. The sequence of m_sMaxCountStack numbers is guaranteed to be
         constant or increasing, never decreasing.
  4. The expandable limit on cache sizes has been extended to include
     byte size limits.  The m_sMaxByteSizeStack contains the sequence
     of limit sizes created with each new SmCacheMgr construction call.
     The special value of zero means don't limit on byte size.
***********************************************************************/
class SmCache : public SmObject
{
  friend class SmCacheMgr;
private:
  
  ULONG           m_lMinCount = SM_UNDEF_ULONG ; // minimum number of elements in stack
  SmTArray<ULONG> m_sMaxByteSizeStack ;          // a stack of queue max byte size limits.
  SmTArray<ULONG> m_sMaxCountStack ;             // a stack of queue max count limits.
                                                 // Each nested call to SmCacheMgr::SmCachMgr
                                                 // adds one layer of queue sizes. 
                                                 // The current queue length equals the last
                                                 // entry on this stack.  When the nested
                                                 // SmCacheMgr object is deleted this
                                                 // stack will be popped by one and the
                                                 // queue max count will be reset to the 
                                                 // previous max count.  
                                       
  SmTArray<SmObject*> m_sBaseObjs;               // array of objects that have cached data
  SmTArray<SmObject*> m_sCacheObjs;              // array of associated cache objects containing cached data
  SmMapPtrToPtr       m_sCacheMap;               // array of [object,cacheObject] pairs

  // mutable std::recursive_mutex * mCacheMutex; // JLMCC removed to address compiler warnings. Revisit.

  // constructor, destructor
  SmCache()
  {
  }
  virtual ~SmCache()
  {
  };

  // queries
  SmCacheObj * GetFrom (const SmObject * cpBaseObj);
  SmCacheObj * GetIndex(const SmObject * cpBaseObj, ULONG & rlIndexFound);

  // add entry to pointer lists, 
  //   side effects: remove and SoftDelete old cache for cpBaseOBj if it exists, 
  //                 remove and SoftDelete end-of-cache entries when queue is over full. 
  void AddTo(const SmObject   * cpBaseObj, 
             const SmCacheObj * cpCacheObj);

  // remove all entries for cpBaseObj
  //   side effects: SoftDelete removed associated object caches
  //   - only used by SmCacheMgr::DeleteObjectsCache()
  void DeleteObjsCache(const SmObject * cpBaseObj);

  // remove entry from pointer lists without deleting the cache
  //   - only used when moving a cache entry up to the front of the list.
  void RemoveObjCache(ULONG lIndex);

  virtual void Dump(void)    const;

public:
  void         SetMinCount(ULONG lMinCount)        { m_lMinCount = lMinCount ; }
               
  ULONG        GetMinCount()    const              { return m_lMinCount ; }
  ULONG        GetMaxCount()    const              { //std::lock_guard<std::recursive_mutex> lock(*(mCacheMutex)); // JLMCC removed to addres compiler warnings. Revisit.
                                                     return m_sMaxCountStack[m_sMaxCountStack.GetSize() - 1]; }
  ULONG        GetCount()       const              { //std::lock_guard<std::recursive_mutex> lock(*(mCacheMutex));
                                                     return m_sBaseObjs.GetSize() ; }
  ULONG        GetMaxByteSize() const              { //std::lock_guard<std::recursive_mutex> lock(*(mCacheMutex));
                                                     return m_sMaxByteSizeStack[m_sMaxCountStack.GetSize()-1] ; }
  ULONG        GetByteSize()    const              { ULONG lMemoryAllocated, lMaxCacheUsed, lMaxCacheAllocated ;
                                                     GetMemoryUsed(lMemoryAllocated, 
                                                                   lMaxCacheUsed,    
                                                                   lMaxCacheAllocated) ;
                                                     return(lMemoryAllocated) ;
                                                   }
  SmObject   * GetBaseObjectAt (ULONG ii)          { return( m_sBaseObjs.GetAt(ii) ) ; }
  SmCacheObj * GetCacheObjectAt(ULONG ii)          { return( (SmCacheObj *)m_sCacheObjs.GetAt(ii) ) ; }
  ULONG        GetMemoryUsed                       // rtn: total memory currently used by the cache             
                  (ULONG &rlMemoryAllocated,       // out: total larger memory sized pre-allocated for cache use
                   ULONG &rlMaxCacheUsed,          // out: Max Single CacheObj used memory                      
                   ULONG &rlMaxCacheAllocated)     // out: Max Single CacheObj allocated memory                 
                   const ;

  // check type of 1st m_sCacheObjs item to find Cache type
  SmObjectCacheType GetCacheType() const ;   
                                       
} ; // end class SmCache

/*******************************************************************//**
PURPOSE: This object manages the cache objects stored within the
  m_crOwningContext->m_pGlobalCache Object, being responsbile for setting
  the cache queue lengths, creating, adding and removing individual
  object caches from the ObjectType cache queues. 
  
  The m_pGlobalCache Object stores one SmCache Object for each known
  object cache type as listed in SmObjectCacheType enum. The length of
  each ObjectType Cache is set by the SmCacheMgr Constructor.  Constructing
  a new SmCacheMgr with an existing SmContext will cause the new
  SmCacheMgr to share the existing SmContext::m_pGlobalCache object.  The
  ObjectType cache queue lengths can be increased with each newly 
  constructed SmCachMgr   object.  When that object is deleted the 
  shared cache queue lengths will be restored to their original sizes.

NOTES: To utilize curve and surface caching, simply declare
   a cache manager on stack with the desired size.  Exiting from the
   current scope will automatically return the cache size to its 
   previous state.  If cache manager declarations are nested, inner
   declarations can only increase the cache size.
***********************************************************************/
class SM_EXPORT SmCacheMgr : public SmObject 
{
private:
    const SmContext & m_crOwningContext; // contains m_pGlobalCache to be managed

public:
   // constructor, destructor
   SmCacheMgr
   (
     const SmContext & crOwningContext,
     ULONG lCurveQueueCount,            ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_CURVE  ]                 
     ULONG lSurfaceQueueCount,          ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_SURFACE]                 
     ULONG lTrimSrfQueueCount,          ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF]                 
     ULONG lBrepQueueCount,             ///< [in] : max count for SmGlobalCache::m_spCaches[SM_OC_BREP   ]                 
                                                                                                                       
     ULONG lCurveCacheByteSize   = 0,   ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_CURVE  ], 0 = no limit
     ULONG lSurfaceCacheByteSize = 0,   ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_SURFACE], 0 = no limit
     ULONG lTrimSrfCacheByteSize = 0,   ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF], 0 = no limit
     ULONG lBrepCacheByteSize    = 0    ///< [in] : max ByteSize for SmGlobalCache::m_spCaches[SM_OC_BREP   ], 0 = no limit
   );  

   virtual ~SmCacheMgr();
   
   // editors

   // Add [Object, ObjectCache] pair to global cache queue
   //   side effect: delete end of queue items when queue is over full.
   static void AddToObjectCache
   (
     SmObjectCacheType  eObjectCacheType,     ///< [in] : oneof SM_OC_CURVE
                                              ///<            SM_OC_SURFACE
                                              ///<            SM_OC_TRIMSRF
                                              ///<            SM_OC_BREP
     const SmObject   * cpObject,             ///< [in] : Target Object
     const SmCacheObj * cpObjectCache         ///< [in] : Its CacheObject
   );

//         // return Object's ObjectCache after removing it from global cache queue
//         static SmCacheObj* GetAndRemoveObjectCache(SmObjectCacheType eObjectCacheType,
//                                                    const SmObject * cpObject);

   // return Object's ObjectCache constructing and adding it to the global cache queue it if necessary
   static SmCacheObj* GetOrCreateObjectCache(SmObjectCacheType  eObjectCacheType,
                                             const SmObject   * cpObject);

   // return Object's ObjectCache or return NULL when Object has no ObjectCache
   static SmCacheObj * GetObjectCache(SmObjectCacheType  eObjectCacheType,
                                      const SmObject   * cpObject);

   // remove and delete an Object's ObjectCache
   static void DeleteObjectsCache(SmObjectCacheType  eObjectCacheType,
                                  const SmObject   * cpObject);

   // used by SmCacheCheckOutIn to set being-used mark on ObjectCache so it won't be deleted
   static void CheckOutCache(SmCacheObj * pCache);
   
   // used by SmCacheCheckOutIn to clear being-used mark on ObjectCache
   static void CheckInCache(SmCacheObj *pCache);

   // used by DeleteObjectsCache to delete ObjectCache when its useCount is 0 else set its m_bDeleted flag
   static void SoftDelete(SmCacheObj *pCache);


   // used by GetOrCreateObjectCache()
   // create a new or fetch an existing ObjectCache - do not place ObjectCache in global Cache Queues
   virtual SmStatus CacheMakeOrValidate
   (
     SmObjectCacheType  eObjectCacheType,    ///< [in] : eObjectCacheType = oneof SM_OC_CURVE
                                             ///<            SM_OC_SURFACE        
                                             ///<            SM_OC_TRIMSRF
                                             ///<            SM_OC_BREP
     const SmObject   * pObject,             ///< [in] : cpObject = target object
     SmCacheObj       * pOldCache,           ///< [in] : cpOldCache = existing target Object's ObjectCache or NULL
     SmCacheObj      *& rpPNewCache          ///< [out]: ptr to target object's ObjectCache
   );

   virtual void Dump(void) const;

} ; // end class SmCacheMgr
#else
/*******************************************************************//**
PURPOSE: This object is an accessor and manager of cache objects.
    This is largely a veneer, maintained for backwards compatibility.

NOTES: Local SmCacheObject* are stored in SmAObjects.
***********************************************************************/
class SM_EXPORT SmCacheMgr : public SmObject 
{
private:
    // const SmContext & m_crOwningContext; // Context the caches belong in

public:
   // constructor, destructor
   // SmCacheMgr( const SmContext & crOwningContext ) : m_crOwningContext( crOwningContext ) {};
   SmCacheMgr() {};

   virtual ~SmCacheMgr() {};
   
   // editors

   // return Object's ObjectCache. Construct Cache and set cpObject->m_pObjectCache if necessary
   static SmCacheObj* GetOrCreateObjectCache(SmObjectCacheType   eObjectCacheType,
                                             const SmAObject   * cpObject);

   // return Object's ObjectCache or return NULL when Object has no ObjectCache
   static SmCacheObj * GetObjectCache(SmObjectCacheType  eObjectCacheType, // NotUsed: in :
                                      const SmAObject  * cpObject);        // in :

   // remove and delete an Object's ObjectCache
   static void DeleteObjectsCache(const SmAObject  * cpObject);

   // remove and delete an Object's ObjectCache
   static void DeleteObjectsCache(SmObjectCacheType  eObjectCacheType, // NotUsed: in :
                                  const SmAObject  * cpObject);        // in :

   // used by SmCacheCheckOutIn to set being-used mark on ObjectCache so it won't be deleted
   static void CheckOutCache(SmCacheObj * pCache);
   
   // used by SmCacheCheckOutIn to clear being-used mark on ObjectCache
   static void CheckInCache(SmCacheObj *pCache);

   // used by DeleteObjectsCache to delete ObjectCache when its useCount is 0 else set its m_bDeleted flag
   static void SoftDelete(SmCacheObj *pCache);

   virtual void Dump(void) const;

} ; // end class SmCacheMgr
#endif  // SM_USE_GLOBAL_CACHE


/*******************************************************************//**
PURPOSE: This stack based class manages caches being checked in 
    and checked out. 
    
    A checked out cache delays when the cache can be deleted.  An attempt
    to delete a checked out cache is delayed until the cache is
    checked back in.

NOTES:  
  1. The SmCacheCheckOutIn constructor increments pCache->m_lUseCount.
  2. The SmCacheCheckOutIn constructor decrements pCache->m_lUseCount.

  SmCacheObj::m_lUseCount is used to delay deleting Cache Objects still
  in use.  When a cache object is deleted, the actual delete is delayed
  until m_lUseCount goes to zero.

***********************************************************************/
class SmCacheCheckOutIn
{
protected:
    SmCacheObj *m_pCache;

public:
    // constructor, destructor
    SmCacheCheckOutIn(SmCacheObj *pCache) 
            { m_pCache = pCache; SmCacheMgr::CheckOutCache(pCache); 
            }

    ~SmCacheCheckOutIn() 
            { SmCacheMgr::CheckInCache(m_pCache); 
            }

} ; // end class SmCacheCheckOutIn

#endif // !__SMCACHEMGR_H__





