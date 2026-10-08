// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmThreadLocalStorage.cpp
* PURPOSE: Source file for Thread Local Memory Management.
**********************************************************************/

#include "StdAfx.h"

#include <SmThreadLocalStorage.h>

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

/* convenience defines to simplify conditionals in code */
#if defined(_WIN32)
#define SM_USE_WIN_THREADS
#endif

#if !defined(_WIN32) && defined(SM_USE_THREADS)
#define SM_USE_PTHREADS
#endif

/*******************************************************************//**
PURPOSE: storage for global access to Thread Local Storage index

NOTES:  This number is unique to each thread.

***********************************************************************/

#ifdef _WIN32
static __declspec(thread) bool s_sBeenThroughNew;
#else
static __thread bool s_sBeenThroughNew;
#endif

#if defined(_WIN32) && defined(SM_USE_THREADS)

// including SmTArray macros here cause some compile conflicts
//  replace #include <SmTArray.h> with a local array definition supporting only
//   Add(), AddUnique(), FindElement(), and RemoveAt()
//  __declspec(thread) static SmBoolean s_bBeenThroughNew ;
//  __declspec(thread) static SmTArray<SmObject*> s_sBeenThroughNew ;
  static __declspec(thread)  SmObject * s_sBeenThroughNew[SM_MAX_THREAD_COUNT] ;
//  static __declspec(thread)  ULONG s_nSize = 0;   // JLMCC never used, removing

#elif defined(SM_USE_WIN_THREADS)

  #include <windows.h>
  static DWORD s_iTLSIndex = TLS_FAIL ;

  #ifdef SM_DEBUG_CODE
    static DWORD s_iProcessThreadId = 0 ;
  #endif // SM_DEBUG_CODE

#elif defined(SM_USE_PTHREADS)

  #include <pthread.h>
  static pthread_key_t  TlsIndex = TLS_FAIL;
  static pthread_once_t TlsInit = PTHREAD_ONCE_INIT;
  static std::uint32_t  TlsInitialized = TLS_FAIL;

  // internal TLS manager using Pthread
  struct SmPthreadTLSManager
  {
      SmPthreadTLSManager()
      {
        // make sure the TLS is initialized - it makes a key on its first call
        pthread_once(&TlsInit, SmPthreadTLSManager::make_key);
      }
    ~SmPthreadTLSManager()
    {
      pthread_key_delete(TlsIndex);
      // This does not compile on the mac. Is the next line necessary?
      // TlsInit = PTHREAD_ONCE_INIT;
    }
    static void make_key()
    {
      pthread_key_create(&TlsIndex, NULL);
      TlsInitialized = 1;
    }
  };

  // static initializer/cleanup
  static SmPthreadTLSManager tls_manager;

#else // no _WIN32, SM_USE_WIN_THREADS, SM_USE_PTHREADS

// static SmBoolean s_bBeenThroughNew ;
// static SmTArray<SmObject *> s_sBeenThroughNew ;
// static SmObject * s_sBeenThroughNew[SM_MAX_THREAD_COUNT] ; // JLMCC conflict with above declaration, error on Linux build
// static ULONG      s_sSize = 0 ; // JLMCC never used, removing.

#endif // end switch on thread type

/*******************************************************************//**
PURPOSE: Private Static Add a new element to the end of the templated array and
   expand the internal memory size of the array as necessary.

NOTES: Note that
   this operation always causes the size to increase by one (GetSize).
***********************************************************************/
ULONG SmThreadLocalStorage::Add
  (SmObject * pBeenThroughNew[SM_MAX_THREAD_COUNT],
   ULONG    * pSize,
   SmObject * sNewObject)
{
  ULONG nIndex = (*pSize);
  if (nIndex < SM_MAX_THREAD_COUNT)
    {
      pBeenThroughNew[nIndex] = sNewObject;
      (*pSize)++;
    }
  else
    {
      ERR_MSG(_T("SmThreadLocalStorage::Add - number of Objects in m_pBeenThroughNewArray exceeds SM_MAX_THREAD_COUNT\n   Report as bug - fixed sized list needs to be a dynamic array")) ;
      return SM_BIG_ULONG ; 
    }
  return nIndex;

} // end SmThreadLocalStorage::Add

/*******************************************************************//**
PURPOSE: Private Static Add an element to the array only if it is not already in the array 
    and return TRUE when element added or FALSE when element already exists.

NOTES:

RETURN --  TRUE  = object was added
           FALSE = object is already a part of the list and was not added this time
***********************************************************************/
SmBoolean SmThreadLocalStorage::AddUnique
  (SmObject * pBeenThroughNew[SM_MAX_THREAD_COUNT],
   ULONG    * pSize,
   SmObject * sNewElement)
{
  ULONG nFoundIndex;
  if (FindElement(pBeenThroughNew, pSize, sNewElement,nFoundIndex))
    { return FALSE ; }
  else
    { Add(pBeenThroughNew, pSize, sNewElement);
      return TRUE ;
    }

} // end SmThreadLocalStorage::AddUnique

/*******************************************************************//**
PURPOSE: Private Static Search the template array for first element of a given value.

NOTES: If the element is found set the nFoundIndex and return TRUE,
    otherwise return FALSE.
***********************************************************************/
SmBoolean SmThreadLocalStorage::FindElement
  (SmObject * pBeenThroughNew[SM_MAX_THREAD_COUNT],
   ULONG    * pSize,
   SmObject * pObjToFind,
   ULONG    & nFoundIndex)
{
  for (ULONG ii=0; ii<(*pSize); ii++)
    {
      SmObject * pObj = pBeenThroughNew[ii];
      if (pObj == pObjToFind)
        {
          nFoundIndex = ii;
          return TRUE;
        }
    }
  return FALSE;

} // end SmThreadLocalStorage::FindElement

/*******************************************************************//**
PURPOSE: Private Static Remove nCount elements from the template array starting at
    the given nIndex and compress the array afterwards.

NOTES: (*pSize) decreased by nCount
***********************************************************************/
void SmThreadLocalStorage::RemoveAt
  (SmObject * pBeenThroughNew[SM_MAX_THREAD_COUNT],
   ULONG    * pSize,
   ULONG nIndex,
   ULONG nCount)
{
  SM_ASSERT_BREAK(nIndex + nCount <= (*pSize));

  // just remove a range
  if ((*pSize) > nIndex + nCount)
    {
      ULONG nMoveCount = (*pSize) - (nIndex + nCount);
      for (ULONG i=0; i<nMoveCount; i++)
        {
          pBeenThroughNew[i+nIndex] = pBeenThroughNew[i+nIndex+nCount];
        }
    }

  (*pSize) = (nCount <= (*pSize)) ? (*pSize) - nCount : 0 ;

} // end SmThreadLocalStorage::RemoveAt

/*******************************************************************//**
PURPOSE: Set Static bBeenThroughNew Thread Local state value

NOTES: 
***********************************************************************/
void SmThreadLocalStorage::SetBeenThroughNew
  (SmBoolean bBeenThroughNew, // in : TRUE = remember pObject has been through new
                              //      FALSE = clear memory of pObject
   SmObject *pObject)         // NotUsed: in : Object to remember of forget
{
  SM_REF1(pObject) ;
	s_sBeenThroughNew = bBeenThroughNew;
  /*
#if defined(SM_USE_WIN_THREADS)

  if(s_iTLSIndex != TLS_FAIL) // PROCESS_ATTACH has run
    {
      // ensure TLS slot exists and is filled with ptr to SmThreadLocalStorage object
      SetupTLS(SM_TLS_SET) ;

      // Get the SmThreadLocalStorage object for this thread
      SmThreadLocalStorage *pTLS = (SmThreadLocalStorage *)TlsGetValue(s_iTLSIndex) ;

      // the allocation of m_pBeenThroughNew is here to avoid the constructor
      // getting into an infinite loop when starting up and building the first SmThreadLocalStorage object
      SM_ASSERT(pTLS->m_pBeenThroughNew != NULL) ;

      // Set the bBeenThroughValue
      // pTLS->m_bBeenThroughNew = bBeenThroughNew ;
      if(bBeenThroughNew) { AddUnique(pTLS->m_pBeenThroughNew, &(pTLS->m_lSize), pObject) ; }
      else                { ULONG lIndx ;
                            if( FindElement(pTLS->m_pBeenThroughNew, &(pTLS->m_lSize), pObject,lIndx) )
                              { RemoveAt(pTLS->m_pBeenThroughNew, &(pTLS->m_lSize), lIndx) ; }
                          }
    }

#elif defined(SM_USE_PTHREADS)

    if ( TlsInitialized != TLS_FAIL )
    {
        //   SetupTLS(SM_TLS_GET);
        //   pthread_setspecific(TlsIndex, reinterpret_cast<void*>(bBeenThroughNew));

        SetupTLS( SM_TLS_GET );

        // Get the SmThreadLocalStorage object for this thread
        SmThreadLocalStorage *pTLS = static_cast<SmThreadLocalStorage *>( pthread_getspecific( TlsIndex ) );
        SM_ASSERT( pTLS != NULL );

        // Set the bBeenThroughValue
        // pTLS->m_bBeenThroughNew = bBeenThroughNew ;
        if ( bBeenThroughNew ) { AddUnique( pTLS->m_pBeenThroughNew, &( pTLS->m_lSize ), pObject ); }
        else
        {
            ULONG lIndx;
            if ( FindElement( pTLS->m_pBeenThroughNew, &( pTLS->m_lSize ), pObject, lIndx ) )
            { RemoveAt( pTLS->m_pBeenThroughNew, &( pTLS->m_lSize ), lIndx ); }
        }
    }

#else // no SM_USE_WIN_THREADS, SM_USE_PTHREADS

  // s_bBeenThroughNew = bBeenThroughNew;
  if(bBeenThroughNew) { AddUnique(s_sBeenThroughNew, &s_sSize, pObject) ; }
  else                { ULONG lIndx ;
                        if( FindElement(s_sBeenThroughNew, &s_sSize, pObject,lIndx) )
                          { RemoveAt(s_sBeenThroughNew, &s_sSize, lIndx) ; }
                      }

#endif // end switch on thread type
*/
} // end SmThreadLocalStorage::SetBeenThroughNew

/*******************************************************************//**
PURPOSE: Get Static bBeenThroughNew Thread Local state value

NOTES: 
***********************************************************************/
SmBoolean SmThreadLocalStorage::GetBeenThroughNew
( SmObject * pObject )  // NotUsed: in : see if this object has been through new
{
  SM_REF1(pObject) ;
	return s_sBeenThroughNew;
  /*
#if defined(SM_USE_WIN_THREADS)

    if ( s_iTLSIndex != TLS_FAIL ) // PROCESS_ATTACH has run
    {
        // ensure TLS slot exists and is filled with ptr to SmThreadLocalStorage object
        SetupTLS( SM_TLS_GET );

        // Get the SmThreadLocalStorage object for this thread
        SmThreadLocalStorage *pTLS = (SmThreadLocalStorage *) TlsGetValue( s_iTLSIndex );

        // Get the bBeenThroughNew Value
        // return(pTLS->m_bBeenThroughNew) ;
        ULONG lIndx;
        return( FindElement( pTLS->m_pBeenThroughNew, &( pTLS->m_lSize ), pObject, lIndx ) );
    }
    else // before been through new memory is set up - just return FALSE
    { return FALSE; }

#elif defined(SM_USE_PTHREADS)

    if ( TlsInitialized != TLS_FAIL )
    {
        SetupTLS( SM_TLS_GET );

        // Get the SmThreadLocalStorage object for this thread
        SmThreadLocalStorage *pTLS = static_cast<SmThreadLocalStorage *>( pthread_getspecific( TlsIndex ) );
        SM_ASSERT( pTLS != NULL );

        // return(pTLS->m_bBeenThroughNew) ;
        ULONG lIndx;
        return( FindElement( pTLS->m_pBeenThroughNew, &( pTLS->m_lSize ), pObject, lIndx ) );
    }
    else
    { return FALSE; }

#else // no SM_USE_WIN_THREADS, SM_USE_PTHREADS
    // return s_bBeenThroughNew;
    ULONG lIndx ;
    return(FindElement(s_sBeenThroughNew, &s_sSize, pObject, lIndx)) ;
#endif // end switch on thread type
*/
} // end SmThreadLocalStorage::GetBeenThroughNew

// for no SM_USE_WIN_THREADS, SM_USE_PTHREADS - define global callback pointer
#if !defined(SM_USE_WIN_THREADS) && !defined(SM_USE_PTHREADS)
static SmErrorCallbackFunctionPtr s_pfErrCallbackFunction;
static ULONG s_lSERDepth = 0 ;
static ULONG s_lAssertValidDepth = 0 ;
#endif

/*******************************************************************//**
PURPOSE: Set Static pfErrCallbackFunction Thread Local state value

NOTES: 
***********************************************************************/
void SmThreadLocalStorage::SetErrCallbackFunction
  (SmErrorCallbackFunctionPtr pfErrCallbackFunction)
{
#if defined(SM_USE_WIN_THREADS)

  // ensure TLS slot exists and is filled with ptr to SmThreadLocalStorage object
  SetupTLS(SM_TLS_SET) ;

  // Get the SmThreadLocalStorage object for this thread
  SmThreadLocalStorage *pTLS = (SmThreadLocalStorage *)TlsGetValue(s_iTLSIndex) ;

  // Set the pfErrCallbackFunction
  pTLS->m_pfErrCallbackFunction = pfErrCallbackFunction ;

#elif defined(SM_USE_PTHREADS)

  SetupTLS(SM_TLS_GET);

  // Get the SmThreadLocalStorage object for this thread
  SmThreadLocalStorage *pTLS = static_cast<SmThreadLocalStorage *>(pthread_getspecific(TlsIndex)) ;

  // Set the pfErrCallbackFunction
  pTLS->m_pfErrCallbackFunction = pfErrCallbackFunction ;
  SM_ASSERT(pTLS != NULL) ;

#else // no SM_USE_WIN_THREADS, SM_USE_PTHREADS

  s_pfErrCallbackFunction = pfErrCallbackFunction ;

#endif // end switch on thread type

} // end SmThreadLocalStorage::SetErrCallbackFunction

/*******************************************************************//**
PURPOSE: increment SER depth level by 1

NOTES: Return incremented SER depth level
***********************************************************************/
ULONG SmThreadLocalStorage::IncSERDepth()
{
  
#if defined(SM_USE_WIN_THREADS)
  // Skip the count if this call is prior to first call to SetupTLS
  if(s_iTLSIndex == TLS_FAIL)
  {
      return(1);
  }

  // ensure TLS slot exists and is filled with ptr to SmThreadLocalStorage object
  SetupTLS(SM_TLS_SET) ;

  // Get the SmThreadLocalStorage object for this thread
  SmThreadLocalStorage *pTLS = (SmThreadLocalStorage *)TlsGetValue(s_iTLSIndex) ;

  // inc the lSERDepth level
  pTLS->m_lSERDepth += 1 ;
  return(pTLS->m_lSERDepth) ;

#elif defined(SM_USE_PTHREADS)
  // Skip the count if this call is prior to first call to SetupTLS
  if(TlsInitialized == TLS_FAIL)
  {
      return(1);
  }

  SetupTLS(SM_TLS_GET);

  // Get the SmThreadLocalStorage object for this thread
  SmThreadLocalStorage *pTLS = static_cast<SmThreadLocalStorage *>(pthread_getspecific(TlsIndex)) ;

  // Set the pfErrCallbackFunction
  pTLS->m_lSERDepth += 1 ;
  return(pTLS->m_lSERDepth) ;

#else // no SM_USE_WIN_THREADS, SM_USE_PTHREADS

  s_lSERDepth += 1 ;
  return(s_lSERDepth) ;

#endif // end switch on thread type

} // end SmThreadLocalStorage::IncSERDepth

/*******************************************************************//**
PURPOSE: decrement SER depth level by 1

NOTES: return decremented SER depth
***********************************************************************/
ULONG SmThreadLocalStorage::DecSERDepth()
{
#if defined(SM_USE_WIN_THREADS)
  // Skip the count if this call is prior to first call to SetupTLS
  if(s_iTLSIndex == TLS_FAIL)
  {
      return(0);
  }

  // ensure TLS slot exists and is filled with ptr to SmThreadLocalStorage object
  SetupTLS(SM_TLS_SET) ;

  // Get the SmThreadLocalStorage object for this thread
  SmThreadLocalStorage *pTLS = (SmThreadLocalStorage *)TlsGetValue(s_iTLSIndex) ;

  // decrement the lSERDepth level
  if(pTLS->m_lSERDepth > 0)
    {
      pTLS->m_lSERDepth -= 1 ;
    }
  return(pTLS->m_lSERDepth) ;

#elif defined(SM_USE_PTHREADS)
  // Skip the count if this call is prior to first call to SetupTLS
  if(TlsInitialized == TLS_FAIL)
  {
      return(0);
  }

  SetupTLS(SM_TLS_GET);

  // Get the SmThreadLocalStorage object for this thread
  SmThreadLocalStorage *pTLS = static_cast<SmThreadLocalStorage *>(pthread_getspecific(TlsIndex)) ;

  // decrement the lSERDepth level
  if(pTLS->m_lSERDepth > 0)
    {
      pTLS->m_lSERDepth -= 1 ;
    }
  return(pTLS->m_lSERDepth) ;

#else // no SM_USE_WIN_THREADS, SM_USE_PTHREADS

  // decrement the lSERDepth level
  if(s_lSERDepth > 0)
    {
      s_lSERDepth -= 1 ;
    }
  return(s_lSERDepth) ;

#endif // end switch on thread type

} // end SmThreadLocalStorage::DecSERDepth

/*******************************************************************//**
PURPOSE: increment AssertValid depth level by 1

NOTES: return incremented AssertValid depth level
***********************************************************************/
ULONG SmThreadLocalStorage::IncAssertValidDepth()
{
#if defined(SM_USE_WIN_THREADS)
  // Skip the count if this call is prior to first call to SetupTLS
  if(s_iTLSIndex == TLS_FAIL)
  {
      return(1);
  }

  // ensure TLS slot exists and is filled with ptr to SmThreadLocalStorage object
  SetupTLS(SM_TLS_SET) ;

  // Get the SmThreadLocalStorage object for this thread
  SmThreadLocalStorage *pTLS = (SmThreadLocalStorage *)TlsGetValue(s_iTLSIndex) ;

  // inc the lAssertValidDepth level
  pTLS->m_lAssertValidDepth += 1 ;
  return(pTLS->m_lAssertValidDepth) ;

#elif defined(SM_USE_PTHREADS)
  // Skip the count if this call is prior to first call to SetupTLS
  if(TlsInitialized == TLS_FAIL)
  {
      return(1);
  }

  SetupTLS(SM_TLS_GET);

  // Get the SmThreadLocalStorage object for this thread
  SmThreadLocalStorage *pTLS = static_cast<SmThreadLocalStorage *>(pthread_getspecific(TlsIndex)) ;

  // Set the pfErrCallbackFunction
  pTLS->m_lAssertValidDepth += 1 ;
  return(pTLS->m_lAssertValidDepth) ;

#else // no SM_USE_WIN_THREADS, SM_USE_PTHREADS

  s_lAssertValidDepth += 1 ;
  return(s_lAssertValidDepth) ;

#endif // end switch on thread type

} // end SmThreadLocalStorage::IncAssertValidDepth

/*******************************************************************//**
PURPOSE: decrement AssertValid depth level by 1

NOTES: return decremented AssertValid depth level
***********************************************************************/
ULONG SmThreadLocalStorage::DecAssertValidDepth()
{
#if defined(SM_USE_WIN_THREADS)
  // Skip the count if this call is prior to first call to SetupTLS
  if(s_iTLSIndex == TLS_FAIL)
  {
      return(0);
  }

  // ensure TLS slot exists and is filled with ptr to SmThreadLocalStorage object
  SetupTLS(SM_TLS_SET) ;

  // Get the SmThreadLocalStorage object for this thread
  SmThreadLocalStorage *pTLS = (SmThreadLocalStorage *)TlsGetValue(s_iTLSIndex) ;

  // decrement the lAssertValidDepth level
  if(pTLS->m_lAssertValidDepth > 0)
    {
      pTLS->m_lAssertValidDepth -= 1 ;
    }
  return(pTLS->m_lAssertValidDepth) ;

#elif defined(SM_USE_PTHREADS)
  // Skip the count if this call is prior to first call to SetupTLS
  if(TlsInitialized == TLS_FAIL)
  {
      return(0);
  }

  SetupTLS(SM_TLS_GET);

  // Get the SmThreadLocalStorage object for this thread
  SmThreadLocalStorage *pTLS = static_cast<SmThreadLocalStorage *>(pthread_getspecific(TlsIndex)) ;

  // decrement the lAssertValidDepth level
  if(pTLS->m_lAssertValidDepth > 0)
    {
      pTLS->m_lAssertValidDepth -= 1 ;
    }
  return(pTLS->m_lAssertValidDepth) ;

#else // no SM_USE_WIN_THREADS, SM_USE_PTHREADS

  // decrement the lAssertValidDepth level
  if(s_lAssertValidDepth > 0)
    {
      s_lAssertValidDepth -= 1 ;
    }
  return(s_lAssertValidDepth) ;

#endif // end switch on thread type

} // end SmThreadLocalStorage::DecAssertValidDepth

/*******************************************************************//**
PURPOSE: Get Static pfErrCallbackFunction Thread Local state value

NOTES: 
***********************************************************************/
SmErrorCallbackFunctionPtr SmThreadLocalStorage::GetErrCallbackFunction()
{
#if defined(SM_USE_WIN_THREADS)

    // ensure TLS slot exists and is filled with ptr to SmThreadLocalStorage object
    SetupTLS(SM_TLS_GET) ;

    // Get the SmThreadLocalStorage object for this thread
    SmThreadLocalStorage *pTLS = (SmThreadLocalStorage *)TlsGetValue(s_iTLSIndex) ;

    // Get the pfErrCallbackFunction
    return(pTLS->m_pfErrCallbackFunction) ;

#elif defined(SM_USE_PTHREADS)

    SetupTLS(SM_TLS_GET);

    // Get the SmThreadLocalStorage object for this thread
    SmThreadLocalStorage *pTLS = static_cast<SmThreadLocalStorage *>(pthread_getspecific(TlsIndex)) ;
    SM_ASSERT(pTLS != NULL) ;

    // Get the pfErrCallbackFunction
    return(pTLS->m_pfErrCallbackFunction) ;

#else // no SM_USE_WIN_THREADS, SM_USE_PTHREADS
    return s_pfErrCallbackFunction;

#endif // end switch on thread type

} // end SmThreadLocalStorage::GetErrCallbackFunction

/*******************************************************************//**
PURPOSE: get thread's SER() call depth value

NOTES: 
***********************************************************************/
ULONG SmThreadLocalStorage::GetSERDepth()
{
#if defined(SM_USE_WIN_THREADS)
  // Skip the count if this call is prior to first call to SetupTLS
  if (s_iTLSIndex == TLS_FAIL)
    {
      return(0);
    }

    // ensure TLS slot exists and is filled with ptr to SmThreadLocalStorage object
    SetupTLS(SM_TLS_GET) ;

    // Get the SmThreadLocalStorage object for this thread
    SmThreadLocalStorage *pTLS = (SmThreadLocalStorage *)TlsGetValue(s_iTLSIndex) ;

    // Get the SER depth level
    return(pTLS->m_lSERDepth) ;

#elif defined(SM_USE_PTHREADS)
  // Skip the count if this call is prior to first call to SetupTLS
  if (TlsInitialized == TLS_FAIL)
    {
      return(0);
    }

    SetupTLS(SM_TLS_GET);

    // Get the SmThreadLocalStorage object for this thread
    SmThreadLocalStorage *pTLS = static_cast<SmThreadLocalStorage *>(pthread_getspecific(TlsIndex)) ;
    SM_ASSERT(pTLS != NULL) ;

    // Get the SER depth level
    return(pTLS->m_lSERDepth) ;

#else // no SM_USE_WIN_THREADS, SM_USE_PTHREADS
    return s_lSERDepth;

#endif // end switch on thread type

} // end SmThreadLocalStorage::GetSERDepth

/*******************************************************************//**
PURPOSE: get thread's AssertValid() call depth value

NOTES: 
***********************************************************************/
ULONG SmThreadLocalStorage::GetAssertValidDepth()
{
#if defined(SM_USE_WIN_THREADS)
  // Skip the count if this call is prior to first call to SetupTLS
  if (s_iTLSIndex == TLS_FAIL)
    {
      return(0);
    }

    // ensure TLS slot exists and is filled with ptr to SmThreadLocalStorage object
    SetupTLS(SM_TLS_GET) ;

    // Get the SmThreadLocalStorage object for this thread
    SmThreadLocalStorage *pTLS = (SmThreadLocalStorage *)TlsGetValue(s_iTLSIndex) ;

    // Get the AssertValid depth level
    return(pTLS->m_lAssertValidDepth) ;

#elif defined(SM_USE_PTHREADS)
  // Skip the count if this call is prior to first call to SetupTLS
  if (TlsInitialized == TLS_FAIL)
    {
      return(0);
    }

    SetupTLS(SM_TLS_GET);

    // Get the SmThreadLocalStorage object for this thread
    SmThreadLocalStorage *pTLS = static_cast<SmThreadLocalStorage *>(pthread_getspecific(TlsIndex)) ;
    SM_ASSERT(pTLS != NULL) ;

    // Get the AssertValid depth level
    return(pTLS->m_lAssertValidDepth) ;

#else // no SM_USE_WIN_THREADS, SM_USE_PTHREADS
    return s_lAssertValidDepth;

#endif // end switch on thread type

} // end SmThreadLocalStorage::GetSERDepth

// NOTE: Temporarily backing out this change, it causes a heap corruption
// when the test application exits.  It needs to be investigated.  [B399]
#ifdef B399
/*******************************************************************//**
PURPOSE: Global list of allocated ThreadLocalStorage objects

NOTES: used to prevent memory leaks
***********************************************************************/
static SmTArray<SmThreadLocalStorage*> GetAllocatedThreadLocalStorages ()
{
  static SmTArray<SmThreadLocalStorage*> allocatedThreadLocalStorages;
  return allocatedThreadLocalStorages;
}
#endif // B399

/*******************************************************************//**
PURPOSE: Ensures that every thread will be assigned one thread local
         slot to be loaded with a pointer to its own SmThreadLocalStorage
         object.  Sets the global s_iTLSIndex value needed by each thread to
         access its TLS slot contents.

NOTES: Allocate a TLS slot for this and all future threads accessed by the s_iTLSIndex index value.
           Each TlsAlloc() call specifies that this and future threads are assigned one TLS slot
           whose contents are accessed by the index value returned by the call.
           The index value is common to the process, the contents of the slots are thread specific.
         After a thread is made,
           Slot data is set with              TlsSetValue(s_iTLSIndex, data) ;
           Slot data is retrieved with data = TlsGetValue(s_iTLSIndex) ;
         SMLib uses the TLS slots to store ptrs to an SmThreadLocalStorage objects.
         SmThreadLocalStorage objects contain all SMLib thread local storage data.
           So, it is expected that NMTlib will only use one slot of ThreadLocalStorage.
           If a 2nd slot is ever desired create and manage s_iTLSIndex_2.

***********************************************************************/
SmStatus SmThreadLocalStorage::SetupTLS
  (SmTLSCallType eTLSCallType)  // in : one of SM_TLS_GET               // not expected - called whenever TLS memory is being accessed
                                //             SM_TLS_PROCESS_ATTACH    // called when Process attaches
                                //             SM_TLS_THREAD_ATTACH     // called when Thread attaches
                                //             SM_TLS_THREAD_DETACH,    // not expected - called when Thread detaches
                                //             SM_TLS_PROCESS_DETACH    // not expected - called when Process detaches
{
  SM_REF1(eTLSCallType);

  // local
  SmStatus sRtn = SM_SUCCESS ;

#if defined(SM_USE_WIN_THREADS)

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
static ULONG lCount1= 1;  
static ULONG lCount = 0; lCount++; if(lCount >= SM_BIG_ULONG) { lCount = 1 ; lCount1++ ; }
static ULONG lDebugCount = 0;
static ULONG lDebugCount1= 1 ;
DWORD dThreadid = GetCurrentThreadId() ;
  if(bDebugMe || (lDebugCount == lCount && lDebugCount1 == lCount1))
    {
      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff,_T("\n  SetupTLS:Thread[%ld(0x%x), %s] current ProcessThreadId[%ld(0x%x) (0=uninit)]\n"),
                 dThreadid, dThreadid,
                   eTLSCallType == SM_TLS_GET            ? _T("SM_TLS_GET")
                 : eTLSCallType == SM_TLS_SET            ? _T("SM_TLS_SET")
                 : eTLSCallType == SM_TLS_PROCESS_ATTACH ? _T("SM_TLS_PROCESS_ATTACH")
                 : eTLSCallType == SM_TLS_THREAD_ATTACH  ? _T("SM_TLS_THREAD_ATTACH")
                 : eTLSCallType == SM_TLS_PROCESS_DETACH ? _T("SM_TLS_PROCESS_DETACH")
                 : eTLSCallType == SM_TLS_THREAD_DETACH  ? _T("SM_TLS_THREAD_DETACH")
                 : _T("Unknown Call Reason"),
                 s_iProcessThreadId, s_iProcessThreadId) ;
      smos_WriteBuffer(sBuff);
    }
#endif // SM_DEBUG_CODE

  // When this is the first call to init the TLS Slot memory for both SM_TLS_PROCESS_ATTACH and SM_TLS_THREAD_ATTACH
  if(s_iTLSIndex == TLS_FAIL)
    {
       // allocate the ThreadLocalStorage slot - store its global index value.
       //   notes: 1.) each call to TlsAlloc() sets up one TLS slot for this and all future threads
       //          2.) this call should only be made once per application thread
       s_iTLSIndex = TlsAlloc();

#ifdef SM_DEBUG_CODE
       s_iProcessThreadId = dThreadid ;
#endif // SM_DEBUG_CODE

       if (s_iTLSIndex==TLS_OUT_OF_INDEXES)
         {
           // exception should occur
           throw TLS_OUT_OF_INDEXES;
         }

       // for both SM_TLS_PROCESS_ATTACH and SM_TLS_THREAD_ATTACH
       // load this thread's slot value with a pointer to a new SmThreadLocalStorage object
       SmThreadLocalStorage *pTLS = new SmThreadLocalStorage() ;

#ifdef SM_DEBUG_CODE
       if(bDebugMe)
         {
           TCHAR sBuff[SM_TBLOCK_SIZE];
           smos_sprintf(sBuff,_T("\n  SetupTLS: new SmThreadLocalStorage[0x%p] for thread [%ld(0x%x)]\n"), pTLS, dThreadid, dThreadid) ;
           smos_WriteBuffer(sBuff);
         }
#endif // SM_DEBUG_CODE

       if (pTLS != NULL)
         {
           TlsSetValue(s_iTLSIndex, pTLS) ;

#ifdef SM_DEBUG_CODE
           SmThreadLocalStorage *pTLS_check = (SmThreadLocalStorage*)TlsGetValue(s_iTLSIndex) ;
           // don't use SM_ASSERT macro in this method - causes infinite loop
           // SM_ASSERT(pTLS_check == pTLS) ;
           { SmBoolean bBool = (pTLS_check == pTLS);  
            if (!bBool) 
             { smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE, FILE_NAME, LINE_NUMBER, NULL, NULL, 0, FUNC_NAME); } 
           }
#endif // SM_DEBUG_CODE

#ifdef B399
           GetAllocatedThreadLocalStorages ().Push(pTLS) ;
#endif // B399

         }

       // all done
       return(sRtn) ;

    } // end s_iTLSIndex == TLS_FAIL check

  //   // for all cases except SM_TLS_PROCESS_ATTACH - make sure thread has SmThreadLocalStorage
  //   if(eTLSCallType != SM_TLS_PROCESS_ATTACH)
  //     {

  // see if this thread has a SmThreadLocalStorage object
  SmThreadLocalStorage *pTLS = (SmThreadLocalStorage *)TlsGetValue(s_iTLSIndex) ;

  // when needed - bind a new SmThreadLocalStorage object to the thread
  if(pTLS == NULL)
    {
#ifdef SM_DEBUG_CODE
      // don't use SM_ASSERT macro in this method - causes infinite loop
      // SM_ASSERT_MSG(s_iProcessThreadId != dThreadid, _T("SetupTLS: confused - what we think is the process thread id is being passed as a thread id "));
      { SmBoolean bBool = (s_iProcessThreadId != dThreadid);
        if (!bBool) 
          { smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE, FILE_NAME, LINE_NUMBER, 
                                    _T("SetupTLS: confused - what we think is the process thread id is being passed as a thread id "),
                                    NULL, FALSE, FUNC_NAME); 
          }
      }
#endif // SM_DEBUG_CODE

      // load this thread's slot value with a pointer to a new SmThreadLocalStorage object
      pTLS = new SmThreadLocalStorage() ;
#ifdef SM_DEBUG_CODE
      if(bDebugMe)
        {
          TCHAR sBuff[SM_TBLOCK_SIZE];
          smos_sprintf(sBuff,_T("SetupTLS: new SmThreadLocalStorage[0x%p] for thread[%ld(0x%x)]\n"), pTLS, dThreadid, dThreadid) ;
          smos_WriteBuffer(sBuff);
        }
#endif // SM_DEBUG_CODE
      if (pTLS != NULL)
        { TlsSetValue(s_iTLSIndex, pTLS) ; }

     } // end thread specific SmThreadLocalStorage object existence check

// } // end not a ProcessAttach call check

#elif defined(SM_USE_PTHREADS)

 //   // for all cases except SM_TLS_PROCESS_ATTACH - make sure thread has SmThreadLocalStorage
 //   if(eTLSCallType != SM_TLS_PROCESS_ATTACH)
 //     {
        // make sure the TLS is initialized - it makes a key on its first call
        // pthread_once(&TlsInit, SmPthreadTLSManager::make_key);

        // see if this thread has a SmThreadLocalStorage object
        SmThreadLocalStorage *pTLS = static_cast<SmThreadLocalStorage *>(pthread_getspecific(TlsIndex)) ;

        // when this thread has no SmThreadLocalStorage object - bind a new one to the thread
        if (pTLS == NULL)
          {
            pTLS = new SmThreadLocalStorage() ;

#ifdef B399
            GetAllocatedThreadLocalStorages ().Push(pTLS) ;
#endif // B399

            pthread_setspecific(TlsIndex, pTLS);
          }
  //    } // end not a ProcessAttach call check

#else // no SM_USE_WIN_THREADS, SM_USE_PTHREADS
  // no action required

#endif // end switch on thread type

    // all done
    return(sRtn) ;

} // end SmThreadLocalStorage::SetupTLS

/*******************************************************************//**
PURPOSE: delete ThreadLocal memory block and TLS index box

NOTES: Also allocates the ThreadLocalStorad index value when needed
***********************************************************************/
SmStatus SmThreadLocalStorage::EndTLS
  (SmTLSCallType eTLSCallType)  // in : one of SM_TLS_GET               // not expected - called whenever TLS memory is being accessed
                                //             SM_TLS_PROCESS_ATTACH    // not expected - called when Process attaches
                                //             SM_TLS_THREAD_ATTACH     // not expected - called when Thread attaches
                                //             SM_TLS_THREAD_DETACH,    // called when Thread detaches
                                //             SM_TLS_PROCESS_DETACH    // called when Process detaches
{
  SM_REF1(eTLSCallType);

#if defined(SM_USE_WIN_THREADS)

  #ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE ;
    DWORD dThreadid = GetCurrentThreadId() ;
    if(bDebugMe)
      {
        TCHAR sBuff[SM_TBLOCK_SIZE];
        smos_sprintf(sBuff,_T("EndTLS:Thread[%ld(0x%x), %s], current ProcessThreadId[%ld(0x%x) (0=uninit)]\n"),
                   dThreadid, dThreadid,
                     eTLSCallType == SM_TLS_GET            ? _T("SM_TLS_GET           ")
                   : eTLSCallType == SM_TLS_SET            ? _T("SM_TLS_SET           ")
                   : eTLSCallType == SM_TLS_PROCESS_ATTACH ? _T("SM_TLS_PROCESS_ATTACH")
                   : eTLSCallType == SM_TLS_THREAD_ATTACH  ? _T("SM_TLS_THREAD_ATTACH ")
                   : eTLSCallType == SM_TLS_PROCESS_DETACH ? _T("SM_TLS_PROCESS_DETACH")
                   : eTLSCallType == SM_TLS_THREAD_DETACH  ? _T("SM_TLS_THREAD_DETACH ")
                   : _T("Unknown Call Reason"),
                   s_iProcessThreadId, s_iProcessThreadId) ;
        smos_WriteBuffer(sBuff);
     }
  #endif // SM_DEBUG_CODE

    // delete this thread's SmThreadLocalStorage object
    SmThreadLocalStorage *pTLS = (SmThreadLocalStorage *)TlsGetValue(s_iTLSIndex);
    if (pTLS != NULL)
      {
  #ifdef SM_DEBUG_CODE
         if(bDebugMe)
           {
             TCHAR sBuff[SM_TBLOCK_SIZE];
             smos_sprintf(sBuff,_T("EndTLS: delete SmThreadLocalStorage[0x%p] for thread[%ld(0x%x)]\n"), pTLS, dThreadid, dThreadid) ;
             smos_WriteBuffer(sBuff);
           }
  #endif // SM_DEBUG_CODE

#ifdef B399
        ULONG idx;
        if (GetAllocatedThreadLocalStorages ().FindElement (pTLS, idx))
          { GetAllocatedThreadLocalStorages ().RemoveAt (idx); }
#endif // B399

        delete pTLS ; pTLS = NULL ;
        TlsSetValue(s_iTLSIndex,0x0) ;
      }

    // When detaching a process
    if(eTLSCallType == SM_TLS_PROCESS_DETACH)
      {
#ifdef SM_DEBUG_CODE
        // don't use SM_ASSERT macro in this method - causes infinite loop
        // SM_ASSERT_MSG(s_iProcessThreadId == dThreadid, _T("EndTLS: confused - what we think is the process thread id isn't"));
        { SmBoolean bBool = (s_iProcessThreadId == dThreadid);
          if (!bBool)
            {
              smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE, FILE_NAME, LINE_NUMBER,
                                      _T("EndTLS: confused - what we think is the process thread id isn't"),
                                      NULL, FALSE, FUNC_NAME);
            }
        }
#endif // SM_DEBUG_CODE

#ifdef B399
        for (ULONG i = 0; i < GetAllocatedThreadLocalStorages ().GetSize(); ++i)
          { delete GetAllocatedThreadLocalStorages ()[i]; }
#endif // B399

        // Release the TLS index memory block.
        TlsFree(s_iTLSIndex);

        s_iTLSIndex = TLS_FAIL ;
      }

#elif defined(SM_USE_PTHREADS) // not SM_USE_WIN_THREADS

  // look for the SmThreadLocalStorage object for this thread
  SmThreadLocalStorage *pTLS = static_cast<SmThreadLocalStorage *>(pthread_getspecific(TlsIndex)) ;

  // if SmThreadLocalStorage object exists - delete it
  if (pTLS != NULL)
    {
#ifdef B399
      ULONG idx;
      if (GetAllocatedThreadLocalStorages ().FindElement (pTLS, idx))
        { GetAllocatedThreadLocalStorages ().RemoveAt (idx); }
#endif // B399
      delete pTLS ; pTLS = NULL ;
      pthread_setspecific(TlsIndex, NULL) ;
    }

  // when detaching the process - clean up the thread key
  if(eTLSCallType == SM_TLS_PROCESS_DETACH)
    {
#ifdef B399
      for (ULONG i = 0; i < GetAllocatedThreadLocalStorages ().GetSize(); ++i)
        { delete GetAllocatedThreadLocalStorages ()[i]; }
#endif // B399
      pthread_key_delete(TlsIndex);
      TlsInitialized = TLS_FAIL;
    }

#else // end switch on thread type
  // no action required

#endif // not SM_USE_WIN_THREADS

  return( SM_SUCCESS );

} // end SmThreadLocalStorage::EndTLS



