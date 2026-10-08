// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmContext.cpp
* PURPOSE: Methods for SmContext.
**********************************************************************/

 
#include "StdAfx.h"
#include <SmContext.h>

#ifdef SM_USE_GLOBAL_CACHE
/*******************************************************************//**
PURPOSE: Constructor for the context which creates an initial
   cache manager.

NOTES: uses all the system default cache count sizes
          SM_DEFAULT_MAXCOUNT_CURVECACHE
          SM_DEFAULT_MAXCOUNT_SURFACECACHE
          SM_DEFAULT_MAXCOUNT_TRIMSRFCACHE
          SM_DEFAULT_MAXCOUNT_BREPCACHE
***********************************************************************/
SmContext::SmContext
  (double dThisModelSizeEstimate,   // in : estimated size of the Breps about to be modeled - 
                                    //      can be very approximate (Plus or Minus a factor of 10 is okay) 
                                    //      recommended: use the default value unless you are an expert.
                                    //      default:[SM_USE_DEFAULT]
   double dThisLargeSmallSizeRatio) // in : for modeling pinholes in battleships - 
                                    //      Always use default values unless you are an expert.
                                    //      default:[SM_USE_DEFAULT]
 : m_lCurrentMark(1),
   m_lCurrentMark2(1),
   m_lCurrentMark3(1),
   m_lCurrentMarkIO(1),
   m_lCurrentMarkAssert(1),
   m_bMarkLock(0),
   m_bMark2Lock(0),
   m_bMark3Lock(0),
   m_bMarkIOLock(0),
   m_bMarkAssertLock(0),
   m_bDoingBoolean(FALSE),
   m_pAttributeCallbacks(NULL),
   m_pGlobalCache(NULL),
   m_pGlobalUserData(NULL),
   m_pUserNotifyCallback(NULL),
   m_pSysNotifyCallBack(NULL)
{
  // When asked (should be extremly rare) set SystemTolerance sizes
// #ifdef SM_USE_NEWTOL
  SM_NEWTOL_LINE m_dThisLargeSmallSizeRatio = dThisLargeSmallSizeRatio ;
  SM_NEWTOL_LINE SetThisModelSizeEstimate(dThisModelSizeEstimate) ;
// #endif // SM_USE_NEWTOL

  // allocate and store this context's GlobalCache
  SmGlobalCache *pGlobalCache = new (*this) SmGlobalCache(SM_MINCOUNT_CURVECACHE,
                                                          SM_MINCOUNT_SURFACECACHE,
                                                          SM_MINCOUNT_TRIMSRFCACHE,
                                                          SM_MINCOUNT_BREPCACHE,

                                                          SM_DEFAULT_BYTESIZE_CURVECACHE,
                                                          SM_DEFAULT_BYTESIZE_SURFACECACHE,
                                                          SM_DEFAULT_BYTESIZE_TRIMSRFCACHE,
                                                          SM_DEFAULT_BYTESIZE_BREPCACHE) ;
  SetGlobalCache(pGlobalCache);

} // end SmContext::SmContext constructor

/*******************************************************************//**
PURPOSE: Constructor for the context which creates an initial
   cache manager.

NOTES: Note that a default cache size of 1000 curve caches,
   200 surface caches, 200 trimmed surface caches, and 200 brep caches
   is used by default on context creation. The minimum size used should
   be 100, 20, 20, 20.

***********************************************************************/
SmContext::SmContext
  (ULONG lCurveCacheCount,          // in : default:[1000], max count for SmGlobalCache::m_spCaches[SM_OC_CURVE  ]
   ULONG lSurfaceCacheCount,        // in : default:[ 200], max count for SmGlobalCache::m_spCaches[SM_OC_SURFACE]
   ULONG lTrimSrfCacheCount,        // in : default:[ 200], max count for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF]
   ULONG lBrepCacheCount,           // in : default:[ 200], max count for SmGlobalCache::m_spCaches[SM_OC_BREP   ]
                                    
   ULONG lCurveCacheByteSize,       // in : 0 = no limit, max ByteSize for SmGlobalCache::m_spCaches[SM_OC_CURVE  ]
   ULONG lSurfaceCacheByteSize,     // in : 0 = no limit, max ByteSize for SmGlobalCache::m_spCaches[SM_OC_SURFACE]
   ULONG lTrimSrfCacheByteSize,     // in : 0 = no limit, max ByteSize for SmGlobalCache::m_spCaches[SM_OC_TRIMSRF]
   ULONG lBrepCacheByteSize,        // in : 0 = no limit, max ByteSize for SmGlobalCache::m_spCaches[SM_OC_BREP   ]

   double dThisModelSizeEstimate,   // in : estimated size of the Breps about to be modeled - 
                                    //      can be very approximate (Plus or Minus a factor of 10 is okay) 
                                    //      recommended: use the default value unless you are an expert.
                                    //      default:[SM_USE_DEFAULT]
   double dThisLargeSmallSizeRatio) // in : for modeling pinholes in battleships - 
                                    //      Always use default values unless you are an expert.
                                    //      default:[SM_USE_DEFAULT]


 : m_lCurrentMark(1),
   m_lCurrentMark2(1),
   m_lCurrentMark3(1),
   m_lCurrentMarkIO(1),
   m_lCurrentMarkAssert(1),
   m_bMarkLock(0),
   m_bMark2Lock(0),
   m_bMark3Lock(0),
   m_bMarkIOLock(0),
   m_bDoingBoolean(FALSE),
   m_bMarkAssertLock(0),
   m_pAttributeCallbacks(NULL),
   m_pGlobalCache(NULL),
   m_pGlobalUserData(NULL),
   m_pUserNotifyCallback(NULL),
   m_pSysNotifyCallBack(NULL)
{
  // When asked (should be extremly rare) set SystemTolerance sizes
// #ifdef SM_USE_NEWTOL
  SM_NEWTOL_LINE m_dThisLargeSmallSizeRatio = dThisLargeSmallSizeRatio ;
  SM_NEWTOL_LINE SetThisModelSizeEstimate(dThisModelSizeEstimate) ;
// #endif // SM_USE_NEWTOL

  // select minimize cache queue sizes
  if (lCurveCacheCount    < SM_MINCOUNT_CURVECACHE  ) lCurveCacheCount    = SM_MINCOUNT_CURVECACHE ;
  if (lSurfaceCacheCount  < SM_MINCOUNT_SURFACECACHE) lSurfaceCacheCount  = SM_MINCOUNT_SURFACECACHE ;
  if (lTrimSrfCacheCount  < SM_MINCOUNT_TRIMSRFCACHE) lTrimSrfCacheCount  = SM_MINCOUNT_TRIMSRFCACHE ;
  if (lBrepCacheCount     < SM_MINCOUNT_BREPCACHE   ) lBrepCacheCount     = SM_MINCOUNT_BREPCACHE ;

  // allocate and store this context's GlobalCache
  SmGlobalCache *pGlobalCache = new (*this) SmGlobalCache(lCurveCacheCount,
                                                         lSurfaceCacheCount,
                                                         lTrimSrfCacheCount,
                                                         lBrepCacheCount,

                                                         lCurveCacheByteSize,
                                                         lSurfaceCacheByteSize,
                                                         lTrimSrfCacheByteSize,
                                                         lBrepCacheByteSize) ;

  SetGlobalCache(pGlobalCache);

} // end SmContext::SmContext constructor

/*******************************************************************//**
PURPOSE: Destructor for the context.

NOTES:
***********************************************************************/
SmContext::~SmContext()
{
    SmGlobalCache *pGlobalCache = GetGlobalCache();
    SM_ASSERT(pGlobalCache != NULL) ; delete pGlobalCache ; pGlobalCache = NULL ;

} // end SmContext::~SmContext destructor
#else

/*******************************************************************//**
PURPOSE: Constructor for the context

NOTES: 
***********************************************************************/
SmContext::SmContext
  (double dThisModelSizeEstimate,   // in : estimated size of the Breps about to be modeled - 
                                    //      can be very approximate (Plus or Minus a factor of 10 is okay) 
                                    //      recommended: use the default value unless you are an expert.
                                    //      default:[SM_USE_DEFAULT]
   double dThisLargeSmallSizeRatio) // in : for modeling pinholes in battleships - 
                                    //      Always use default values unless you are an expert.
                                    //      default:[SM_USE_DEFAULT]
 : m_lCurrentMark(1),
   m_lCurrentMark2(1),
   m_lCurrentMark3(1),
   m_lCurrentMarkIO(1),
   m_lCurrentMarkAssert(1),
   m_bMarkLock(0),
   m_bMark2Lock(0),
   m_bMark3Lock(0),
   m_bMarkIOLock(0),
   m_bMarkAssertLock(0),
   m_bDoingBoolean(FALSE),
   m_pAttributeCallbacks(NULL),
   m_pGlobalUserData(NULL),
   m_pUserNotifyCallback(NULL)
{
  // When asked (should be extremly rare) set SystemTolerance sizes
// #ifdef SM_USE_NEWTOL
  SM_NEWTOL_LINE m_dThisLargeSmallSizeRatio = dThisLargeSmallSizeRatio ;
  SM_NEWTOL_LINE SetThisModelSizeEstimate(dThisModelSizeEstimate) ;
// #endif // SM_USE_NEWTOL

} // end SmContext::SmContext constructor

#endif // SM_USE_GLOBAL_CACHE

#ifdef SM_USE_NEWTOL
SM_NEWTOL_LINE /*******************************************************************//**
SM_NEWTOL_LINE PURPOSE: Set ThisModelSizeEstimate to allow a Context to have
SM_NEWTOL_LINE          a different Estimate size than the current system value.
SM_NEWTOL_LINE 
SM_NEWTOL_LINE NOTES:
SM_NEWTOL_LINE ***********************************************************************/
SM_NEWTOL_LINE SmZoneTol3d SmContext::SetThisModelSizeEstimate
SM_NEWTOL_LINE  (double dThisModelSizeEstimate)  // in : ModelSizeEstimate -> ZoneTol3d val for this context only
SM_NEWTOL_LINE {
SM_NEWTOL_LINE   if(dThisModelSizeEstimate == SM_USE_DEFAULT)
SM_NEWTOL_LINE     {
SM_NEWTOL_LINE       m_dThisModelSizeEstimate   = SmTol::GetModelSizeEstimate() ;
SM_NEWTOL_LINE       m_dThisLargeSmallSizeRatio = SmTol::GetLargeSmallSizeRatio() ;
SM_NEWTOL_LINE       m_dThisZoneTol3d           = SmTol::GetZoneTol3d() ;
SM_NEWTOL_LINE     }
SM_NEWTOL_LINE   else
SM_NEWTOL_LINE     {
SM_NEWTOL_LINE       m_dThisModelSizeEstimate   = dThisModelSizeEstimate ;
SM_NEWTOL_LINE       m_dThisLargeSmallSizeRatio = SmTol::GetLargeSmallSizeRatio() ;
SM_NEWTOL_LINE       m_dThisZoneTol3d           = SmTol::SizeEstimateToZoneTol3d(m_dThisModelSizeEstimate) ;
SM_NEWTOL_LINE     }
SM_NEWTOL_LINE 
SM_NEWTOL_LINE   return(m_dThisZoneTol3d) ;
SM_NEWTOL_LINE 
SM_NEWTOL_LINE } // end SmContext::SetThisModelSizeEstimate
#endif // SM_USE_NEWTOL

/*******************************************************************//**
PURPOSE: Overrides the new operator for SmObject subclasses.

NOTES: SmObject *a = new SmObject();
   Try to use the context based news instead of this one if you wish
   to maintain the pool based memory capabilities.
***********************************************************************/
void *SmContext::operator new(size_t size)
{
  // okay to use smos_Calloc on static class (SmContext) objects.
  SmContext *pRet = (SmContext*)smos_Calloc(1,size);
  return (void*)pRet;
}

#ifdef SM_USE_GLOBAL_CACHE
/*******************************************************************//**
PURPOSE: Compute and report current global cache sizes

NOTES:
***********************************************************************/
void SmContext::GetGlobalCacheSize
  (SmGlobalCacheSize &rGlobalCacheSize)  // out: various measures of the global cache size
  const
{
  const SmCache *pCurveCache   = m_pGlobalCache->GetCacheQueue(SM_OC_CURVE) ;
  const SmCache *pSurfaceCache = m_pGlobalCache->GetCacheQueue(SM_OC_SURFACE) ;
  const SmCache *pTrimSrfCache = m_pGlobalCache->GetCacheQueue(SM_OC_TRIMSRF) ;
  const SmCache *pBrepCache    = m_pGlobalCache->GetCacheQueue(SM_OC_BREP) ;


  // set output
  rGlobalCacheSize.m_lCurveQueueLength   = pCurveCache->GetMaxCount() ;
  rGlobalCacheSize.m_lCurveQueueCount    = pCurveCache->GetCount() ;
  rGlobalCacheSize.m_lCurveQueueUsed     = pCurveCache->GetMemoryUsed(rGlobalCacheSize.m_lCurveQueueAllocated,
                                                                      rGlobalCacheSize.m_lMaxCurveCacheUsed,
                                                                      rGlobalCacheSize.m_lMaxCurveCacheAllocated) ;

  rGlobalCacheSize.m_lSurfaceQueueLength = pSurfaceCache->GetMaxCount() ;
  rGlobalCacheSize.m_lSurfaceQueueCount  = pSurfaceCache->GetCount() ;
  rGlobalCacheSize.m_lSurfaceQueueUsed   = pSurfaceCache->GetMemoryUsed(rGlobalCacheSize.m_lSurfaceQueueAllocated,
                                                                        rGlobalCacheSize.m_lMaxSurfaceCacheUsed,
                                                                        rGlobalCacheSize.m_lMaxSurfaceCacheAllocated) ;

  rGlobalCacheSize.m_lTrimSrfQueueLength = pTrimSrfCache->GetMaxCount() ;
  rGlobalCacheSize.m_lTrimSrfQueueCount  = pTrimSrfCache->GetCount() ;
  rGlobalCacheSize.m_lTrimSrfQueueUsed   = pTrimSrfCache->GetMemoryUsed(rGlobalCacheSize.m_lTrimSrfQueueAllocated,
                                                                        rGlobalCacheSize.m_lMaxTrimSrfCacheUsed,
                                                                        rGlobalCacheSize.m_lMaxTrimSrfCacheAllocated) ;

  rGlobalCacheSize.m_lBrepQueueLength    = pBrepCache->GetMaxCount() ;
  rGlobalCacheSize.m_lBrepQueueCount     = pBrepCache->GetCount() ;
  rGlobalCacheSize.m_lBrepQueueUsed      = pBrepCache->GetMemoryUsed(rGlobalCacheSize.m_lBrepQueueAllocated,
                                                                     rGlobalCacheSize.m_lMaxBrepCacheUsed,
                                                                     rGlobalCacheSize.m_lMaxBrepCacheAllocated) ;

} // end SmContext::GetGlobalCacheSize

/*******************************************************************//**
PURPOSE: retrieve saved maximum global cache sizes into output

NOTES:
***********************************************************************/
#ifdef SM_DEBUG_CODE
SmGlobalCacheSize & SmContext::GetMaxGlobalCacheSizes
  ()
  const
{
  return(m_pGlobalCache->m_sMaxCacheSizes) ;

} // end SmContext::GetMaxGlobalCacheSizes
#endif // SM_DEBUG_CODE
#endif //SM_USE_GLOBAL_CACHE

/*******************************************************************//**
PURPOSE: return requeste MarkType mark value.

NOTES: Objects whose Object mark value == Context mark value
       are marked - otherwise not.

       When eMarkType == SM_MT_NOMARK, returns largest current mark value + 1
                                       this makes sure that all objects
                                       are not marked for SM_MT_NOMARK
***********************************************************************/
ULONG SmContext::GetCurrentMark(SmMarkType eMarkType) const
{
  switch(eMarkType)
    {
      case SM_MT_MARK       : return (m_lCurrentMark) ;
      case SM_MT_MARK2      : return (m_lCurrentMark2) ;
      case SM_MT_MARK3      : return (m_lCurrentMark3) ;
      case SM_MT_MARKIO     : return (m_lCurrentMarkIO) ;
      case SM_MT_MARKASSERT : return (m_lCurrentMarkAssert) ;
      case SM_MT_NOMARK     : return (1 + smos_5Max(m_lCurrentMark,
                                                    m_lCurrentMark2,
                                                    m_lCurrentMark3,
                                                    m_lCurrentMarkIO,
                                                    m_lCurrentMarkAssert)) ;
      default:                return (m_lCurrentMarkIO) ;
    }

} // end SmContext::GetCurrentMark

/************************************************************
PURPOSE: private method to increment and returns the specified mark value

NOTES: When eMarkType == SM_MT_NOMARK, makes no changes,
                                       returns largest Mark value + 1.
       use class SmNewMarkAndLock to access NewMark()
***********************************************************************/
ULONG SmContext::NewMark
  (SmMarkType eMarkType,    // in : oneof SM_MT_MARK, SM_MT_MARK2, SM_MT_MARKIO
   SmBoolean  bIgnoreLock)  // in : TRUE  = Caller has locked this mark and now wants to increment it without a warning
                            //      FALSE = Normal use - caller increments mark and gets a debug warning when incremented if locked
                            //      default:[FALSE]
{
  switch(eMarkType)
    {
      case SM_MT_MARK       : SM_ASSERT_MSG_BREAK(bIgnoreLock == TRUE || 0==m_bMarkLock, _T("Incrementing SM_MT_MARK when locked")) ;
                              return (m_lCurrentMark += 2) ;

      case SM_MT_MARK2      : SM_ASSERT_MSG_BREAK(bIgnoreLock == TRUE || 0==m_bMark2Lock, _T("Incrementing SM_MT_MARK2 when locked")) ;
                              return (m_lCurrentMark2 += 2) ;

      case SM_MT_MARK3      : SM_ASSERT_MSG_BREAK(bIgnoreLock == TRUE || 0==m_bMark3Lock, _T("Incrementing SM_MT_MARK3 when locked")) ;
                              return (m_lCurrentMark3 += 2) ;

      case SM_MT_MARKIO     : SM_ASSERT_MSG_BREAK(bIgnoreLock == TRUE || 0==m_bMarkIOLock, _T("Incrementing SM_MT_MARKIO when locked")) ;
                              return (m_lCurrentMarkIO += 2) ;

      case SM_MT_MARKASSERT : SM_ASSERT_MSG_BREAK(bIgnoreLock == TRUE || 0==m_bMarkAssertLock, _T("Incrementing SM_MT_MARKASSERT when locked")) ;
                              return (m_lCurrentMarkAssert += 2) ;

      case SM_MT_NOMARK     : return GetCurrentMark(SM_MT_NOMARK) ;
      default:                return (m_lCurrentMarkIO += 2) ;
    }

  SM_REF1(bIgnoreLock);

} // end SmContext::NewMark

/*******************************************************************//**
PURPOSE: return lock status of specified MarkType

NOTES: currently mark locking only has a debug warning side effect
***********************************************************************/
SmBoolean SmContext::IsMarkLocked(SmMarkType eMarkType) const
{
  switch(eMarkType)
    { case SM_MT_MARK       : return (m_bMarkLock == TRUE) ;
      case SM_MT_MARK2      : return (m_bMark2Lock == TRUE) ;
      case SM_MT_MARK3      : return (m_bMark3Lock == TRUE) ;
      case SM_MT_MARKIO     : return (m_bMarkIOLock == TRUE) ;
      case SM_MT_MARKASSERT : return (m_bMarkAssertLock == TRUE) ;
      case SM_MT_NOMARK     : return (FALSE) ;
      default:                return (FALSE) ;
    }

} // end SmContext::IsMarkLocked

/*******************************************************************//**
PURPOSE: Lock a specified mark

NOTES: In debug mode only - outputs an assert if the mark
                is already locked
***********************************************************************/
void SmNewMarkAndLock::LockMark
  (SmContext *pContext,
   SmMarkType eMarkType)
{
  // check specified mark is unlocked and then lock it
  switch(eMarkType)
    {
      case SM_MT_NOMARK     : SM_ASSERT_ERR ;
                              break ;

      case SM_MT_MARK       : SM_ASSERT(pContext->m_bMarkLock == FALSE) ;
                              pContext->m_bMarkLock   = TRUE ;
                              break ;

      case SM_MT_MARK2      : SM_ASSERT(pContext->m_bMark2Lock == FALSE) ;
                              pContext->m_bMark2Lock  = TRUE ;
                              break ;

      case SM_MT_MARK3      : SM_ASSERT(pContext->m_bMark3Lock == FALSE) ;
                              pContext->m_bMark3Lock  = TRUE ;
                              break ;

      case SM_MT_MARKIO     : SM_ASSERT(pContext->m_bMarkIOLock == FALSE) ;
                              pContext->m_bMarkIOLock = TRUE ;
                              break ;

      case SM_MT_MARKASSERT : SM_ASSERT(pContext->m_bMarkAssertLock == FALSE) ;
                              pContext->m_bMarkAssertLock = TRUE ;
                              break ;
      default:
          break;

    }
} // end SmNewMarkAndLock::LockMark

/*******************************************************************//**
PURPOSE: Unlock a specified mark

NOTES: In debug mode only - executes an assert when
   specified mark is not currently locked.
***********************************************************************/
void SmNewMarkAndLock::UnlockMark
  (SmContext *pContext,
   SmMarkType eMarkType)
{
  // check specified mark is locked and then unlock it
  switch(eMarkType)
    { case SM_MT_NOMARK     : SM_ASSERT_ERR ;
                              break ;

      case SM_MT_MARK       : SM_ASSERT(pContext->m_bMarkLock == TRUE) ;
                              pContext->m_bMarkLock   = FALSE ;
                              break ;

      case SM_MT_MARK2      : SM_ASSERT(pContext->m_bMark2Lock == TRUE) ;
                              pContext->m_bMark2Lock  = FALSE ;
                              break ;

      case SM_MT_MARK3      : SM_ASSERT(pContext->m_bMark3Lock == TRUE) ;
                              pContext->m_bMark3Lock  = FALSE ;
                              break ;

      case SM_MT_MARKIO     : SM_ASSERT(pContext->m_bMarkIOLock == TRUE) ;
                              pContext->m_bMarkIOLock = FALSE ;
                              break ;

      case SM_MT_MARKASSERT : SM_ASSERT(pContext->m_bMarkAssertLock == TRUE) ;
                              pContext->m_bMarkAssertLock = FALSE ;
                              break ;
      default:
          break;
    }
} // end SmNewMarkAndLock::UnlockMark

/*******************************************************************//**
PURPOSE: return TRUE when specified mark is locked

NOTES:
***********************************************************************/
SmBoolean SmNewMarkAndLock::IsLockedMark
  (SmContext *pContext,
   SmMarkType eMarkType)
{
  // check specified mark is locked and then unlock it
  switch(eMarkType)
    { case SM_MT_NOMARK     : return(FALSE) ;
      case SM_MT_MARK       : return(pContext->m_bMarkLock) ;
      case SM_MT_MARK2      : return(pContext->m_bMark2Lock) ;
      case SM_MT_MARK3      : return(pContext->m_bMark3Lock) ;
      case SM_MT_MARKIO     : return(pContext->m_bMarkIOLock) ;
      case SM_MT_MARKASSERT : return(pContext->m_bMarkAssertLock) ;
      default:
          break;
    }
  return FALSE ;
} // end SmNewMarkAndLock::IsLockedMark

/*******************************************************************//**
PURPOSE: Find and return an unlocked mark

NOTES: When no suitable mark is available, a warning is
  output in debug mode and SM_MT_MARKIO is returned.
  That warning is a bug - if you get it, fix the code
  to find and eliminate mark clashes.

  This helps simple routines in using a mark without
  messing up higher level routines which have locked a mark
  for their own use.
***********************************************************************/
SmMarkType SmNewMarkAndLock::GetUnlockedMark
 (const SmContext & crContext,  // in : target context containing desired Mark
  SmBitArray        baMask)     // in : list of marks to check, orof
                                //         SM_MT_MARK
                                //         SM_MT_MARK2
                                //         SM_MT_MARK3
                                //         SM_MT_MARKIO
                                //         SM_MT_MARKASSERT
                                //       SM_MT_CODEMARKS = use SM_MT_MARK or SM_MT_MARK2 or SM_MT_MARK3
                                //       SM_MT_ALLMARKS  = use any unlocked mark
                                //       default:[SM_MT_CODEMARKS]
{
  if((baMask & SM_MT_MARK      ) && (!crContext.m_bMarkLock))       { return(SM_MT_MARK  ) ; }
  if((baMask & SM_MT_MARK2     ) && (!crContext.m_bMark2Lock))      { return(SM_MT_MARK2 ) ; }
  if((baMask & SM_MT_MARK3     ) && (!crContext.m_bMark3Lock))      { return(SM_MT_MARK3 ) ; }
  if((baMask & SM_MT_MARKIO    ) && (!crContext.m_bMarkIOLock))     { return(SM_MT_MARKIO) ; }
  if((baMask & SM_MT_MARKASSERT) && (!crContext.m_bMarkAssertLock)) { return(SM_MT_MARKASSERT) ; }

  // arrive here when all marks are locked
  SM_DBG_WARN(_T("Could not find a SmContext Mark with given mask which was not locked - debug your code changes")) ;
  if(baMask == SM_MT_MARK      ) return SM_MT_MARK       ;
  if(baMask == SM_MT_MARK2     ) return SM_MT_MARK2      ;
  if(baMask == SM_MT_MARK3     ) return SM_MT_MARK3      ;
  if(baMask == SM_MT_MARKIO    ) return SM_MT_MARKIO     ;
  if(baMask == SM_MT_MARKASSERT) return SM_MT_MARKASSERT ;
  return(SM_MT_MARKIO) ;

} // end SmNewMarkAndLock::GetUnlockedMark

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmContext::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  // begin header
  SM_SPRINTF(sBuff       ,_T("\nSmContext = 0x%p, "), this) ;
  SM_SPRINTF(sBuffForFile,_T("\nSmContext = %s, "), _T("notNULL") ) ;
  smos_WriteBuffer(sBuff, sBuffForFile);


  // ZoneTol
#ifdef SM_USE_NEWTOL
  SM_NEWTOL_LINE SM_SPRINTF(sBuff       ,_T("ThisModelSizeEstimate  :[%16.16lf], "), m_dThisModelSizeEstimate) ;
  SM_NEWTOL_LINE SM_SPRINTF(sBuff       ,_T("ThisLargeSmallSizeRatio:[%16.16lf], "), m_dThisLargeSmallSizeRatio) ;
  SM_NEWTOL_LINE SM_SPRINTF(sBuff       ,_T("ThisZoneTol3d          :[%16.16lf], "), m_sThisZoneTol3d) ;
  SM_NEWTOL_LINE smos_WriteBuffer(sBuff) ;
#endif // SM_USE_NEWTOL

  // marks
  SM_SPRINTF(sBuff       ,_T("Mark[%ld, %s], Mark2[%ld, %s], Mark3[%ld, %s], MarkIO[%ld, %s], MarkAssert[%ld, %s]"),
             m_lCurrentMark,       m_bMarkLock ?       _T("Locked") : _T("UnLocked"),
             m_lCurrentMark2,      m_bMark2Lock ?      _T("Locked") : _T("UnLocked"),
             m_lCurrentMark3,      m_bMark3Lock ?      _T("Locked") : _T("UnLocked"),
             m_lCurrentMarkIO,     m_bMarkIOLock ?     _T("Locked") : _T("UnLocked"),
             m_lCurrentMarkAssert, m_bMarkAssertLock ? _T("Locked") : _T("UnLocked"));
  smos_WriteBuffer(sBuff);

} // end SmContext::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmNewMarkAndLock::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // begin header
  SM_SPRINTF(sBuff       ,_T("\nSmNewMarkAndLock = 0x%p, "), this) ;
  SM_SPRINTF(sBuffForFile,_T("\nSmNewMarkAndLock = %s, "), _T("notNULL") ) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // 1st Mark
  SM_SPRINTF(sBuff       ,_T("\n  m_pContext1 = 0x%p, "), m_pContext1) ;
  SM_SPRINTF(sBuffForFile,_T("\n  m_pContext1 = %s, "), m_pContext1 ? _T("notNULL") : _T("NULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  SM_SPRINTF(sBuff       ,_T("Mark:[%s = %ld]"),
               m_eMarkType1 == SM_MT_NOMARK     ? _T("SM_MT_NOMARK    ")
             : m_eMarkType1 == SM_MT_MARK       ? _T("SM_MT_MARK      ")
             : m_eMarkType1 == SM_MT_MARK2      ? _T("SM_MT_MARK2     ")
             : m_eMarkType1 == SM_MT_MARK3      ? _T("SM_MT_MARK3     ")
             : m_eMarkType1 == SM_MT_MARKIO     ? _T("SM_MT_MARKIO    ")
             : m_eMarkType1 == SM_MT_MARKASSERT ? _T("SM_MT_MARKASSERT")
             : m_eMarkType1 == SM_MT_ALLMARKS   ? _T("SM_MT_ALLMARKS  ")
             : _T("Unknown"),
             m_pContext1 ? m_pContext1->GetCurrentMark(m_eMarkType1) : SM_MT_NOMARK) ;
  smos_WriteBuffer(sBuff);

  // binary dump
  if(m_pContext2 && m_pContext1 != m_pContext2)
    {
      // 2nd Mark
      SM_SPRINTF(sBuff       ,_T("\n  m_pContext2 = 0x%p, "), m_pContext2) ;
      SM_SPRINTF(sBuffForFile,_T("\n  m_pContext2 = %s, "), m_pContext2 ? _T("notNULL") : _T("NULL")) ;
      smos_WriteBuffer(sBuff, sBuffForFile);

      SM_SPRINTF(sBuff       ,_T("Mark:[%s = %ld]"),
                   m_eMarkType2 == SM_MT_NOMARK     ? _T("SM_MT_NOMARK    ")
                 : m_eMarkType2 == SM_MT_MARK       ? _T("SM_MT_MARK      ")
                 : m_eMarkType2 == SM_MT_MARK2      ? _T("SM_MT_MARK2     ")
                 : m_eMarkType2 == SM_MT_MARK3      ? _T("SM_MT_MARK3     ")
                 : m_eMarkType2 == SM_MT_MARKIO     ? _T("SM_MT_MARKIO    ")
                 : m_eMarkType2 == SM_MT_MARKASSERT ? _T("SM_MT_MARKASSERT")
                 : m_eMarkType2 == SM_MT_ALLMARKS   ? _T("SM_MT_ALLMARKS  ")
                 : _T("Unknown"),
                 m_pContext2->GetCurrentMark(m_eMarkType2)) ;
      smos_WriteBuffer(sBuff);

    }

} // end SmNewMarkAndLock::Dump
