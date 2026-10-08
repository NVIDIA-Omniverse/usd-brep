// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmMemory.cpp
* PURPOSE: Source for memory management functions
**********************************************************************/

// include for memory leak detection with "visual leak detector"
// #include "C:\Program Files (x86)\Visual Leak Detector\include\vld.h"

#if defined(_WIN32) && defined(_DEBUG) // malloc.h is for _heapchk() in smos_HeapCheck, which only runs on Windows debug builds
#include <malloc.h>
#else
#include <stdlib.h>                   // all other platforms, including macOS, use this instead
#endif


#include "StdAfx.h"

#include <SmConfig.h>
#include <SmMemory.h>
#include <SmMessages.h>

#ifdef SM_USE_EXCEPTIONS
#include <new>
#endif

#ifdef SM_USE_TBB_SCALABLE
#include "tbb/scalable_allocator.h"
#endif

// Overloading delete/new to call scalable memory allocator of TBB parallelization.
// This is necessary since NLib memory allocators have been changed to TBB scalable
// counterparts.

#ifdef SM_USE_TBB_SCALABLE

void operator delete(void* ptr) throw()
{
    scalable_free(ptr);
}

void* operator new(size_t sz)
{
    void* res = scalable_malloc(sz);

    if (res == NULL)
        throw std::bad_alloc();

    return res;
}

#endif

#ifdef SM_DEBUG_CODE
static ULONG lDebugCount = 0;
#endif

void *badmem = (void*)0x0DFE1E08;

#ifndef SM_USING_MEMORY_MACROS

// GWC - unused function
//  /*******************************************************************//**
//  PURPOSE: Pool based interface for malloc - allocate memory block
//              without initialization.  
//  
//  NOTES: 
//   1. All memory allocated should pass through this function, smos_Calloc, or smos_Realloc.
//   2. Never use malloc, calloc, realloc or free directly.
//   3. gets memory with Malloc() call.
//  
//  ***********************************************************************/
  void* smos_Malloc
    (size_t size)                 // in : desired memory size in bytes
  {
    // init output
    void *ret = NULL ;
  
    // when using SmartHeap - get memory from SmartHeap Pools
    // else                   get memory with Calloc()
  #ifdef SM_USE_TBB_SCALABLE
    ret = (void*)scalable_malloc((size_t)size);
  #else
    ret = (void*)malloc((size_t)size);
  #endif
  
  #ifdef SM_DEBUG_CODE
    if (   lDebugCount != 0  
        && ret == badmem) 
      { MSG(_T("Found Debug Memory"));
      }
  #endif
  
    // respond to alloc failures - throw exception or output warning
    if (ret == NULL) 
      {
        SE(1002); // Memory Error Out of Memory
  #ifdef SM_USE_EXCEPTIONS
        throw std::bad_alloc();
  #endif
      }
  
    // all done
    return ret;
  
  } // end smos_Malloc


/*******************************************************************//**
PURPOSE: Pool based interface for calloc - allocate memory block
            and initialize it with 0 values.  

NOTES: 
 1. All memory allocated should pass through this function, smos_Calloc, or smos_Realloc.
 2. Never use malloc, calloc, realloc or free directly.
 3. gets memory with Malloc() call.
***********************************************************************/
void* smos_Calloc
 (size_t            num,        // in : num of memory blocks to allocate
  size_t            size)       // in : size of each block in bytes
{
  // init output
  void *ret = NULL ;

  //  every 10,000 calls - quit if user hit excape key event
  //  static ULONG lCounter = 0;
  //  lCounter ++;
  //  if (   lCounter % 10000 == 0
  //      && smos_ExcapeCallback()) { return NULL;
  //                                }

  // get memory with Calloc()
  #ifdef SM_USE_TBB_SCALABLE
  ret = (void*)scalable_calloc((size_t)num, (size_t)size);
  #else
  ret = (void*)calloc((size_t)num, (size_t)size);
  #endif

#ifdef SM_DEBUG_CODE
  if (   lDebugCount != 0
      && ret == badmem) 
    {
       MSG(_T("Found Debug Memory"));
    }
#endif

  // respond to alloc failures - throw exception or output warning
  if (ret == NULL) 
    {
#ifdef SM_USE_EXCEPTIONS
      throw std::bad_alloc();
#endif
      SE(1002); // Memory Error Out of Memory
    }

  // all done
  return ret;

} // end smos_Calloc

// GWC - unused function
//  /*******************************************************************//**
//  PURPOSE: Pool based interface for realloc - reallocate memory block
//              while preserving old values.  
//  
//  NOTES: 
//   1. All memory allocated should pass through this function, smos_Calloc, or smos_Realloc.
//   2. Never use malloc, calloc, realloc or free directly.
//   3. gets memory with Malloc() call.
//  ***********************************************************************/
//  void* smos_Realloc
//    (void* ptr_mem_block,         // in : pointer to existing block
//     size_t size)                 // in : desired new size for existing block
//  {
//    // init output
//    void *ret = NULL ;
//  
//    // when using SmartHeap - use SmartHeap memory pool
//    // else                 - use realloc() call
//    ret = (void*)realloc(SM_MALLOC_CAST(ptr_mem_block), (size_t)size);
//  
//  #ifdef SM_DEBUG_CODE
//      if (   lDebugCount != 0
//          && ret == badmem) 
//        {
//          MSG(_T("Found Debug Memory"));
//        }
//  #endif
//  
//      // respond to alloc failures - throw exception or output warning
//      if (ret == NULL) 
//        {
//          SE(1002); // Memory Error Out of Memory
//  #ifdef SM_USE_EXCEPTIONS
//          throw std::bad_alloc();
//  #endif
//        }
//  
//      // all done
//      return ret;
//  
//  } // end smos_Realloc
// GWC - end unused function

/*******************************************************************//**
PURPOSE: Free memory allocated by smos_Calloc, smos_Realloc, and 
    smos_Malloc.

NOTES: Never use malloc, calloc, realloc or free directly.
***********************************************************************/
void  smos_Free
  (void* ptr_mem_block)     // in : pointer to memory block to free
{
#ifdef SM_DEBUG_CODE
  if (   lDebugCount != 0
      && ptr_mem_block == badmem) 
    {
      MSG(_T("Found Debug Memory"));
    }
#endif // SM_DEBUG_CODE

  // - use Free()
  #ifdef SM_USE_TBB_SCALABLE
  scalable_free(SM_MALLOC_CAST(ptr_mem_block));
  #else
  free(SM_MALLOC_CAST(ptr_mem_block));
  #endif

} // end smos_Free

#endif // no SM_USING_MEMORY_MACROS

/*******************************************************************//**
PURPOSE: Checks to make sure no objects on heap have been corrupted.

NOTES: 
***********************************************************************/
SmStatus smos_HeapCheck
 (const TCHAR * /*message*/,
  long          /*num*/)
{
#if defined(_WIN32) && defined(_DEBUG)
  int heapstatus = _heapchk();
  if (heapstatus != _HEAPOK) 
    { SE(SM_ERR); }
    
#endif // defined(_WIN32) && defined(_DEBUG)
  return SM_SUCCESS;

} // end smos_HeapCheck

/*******************************************************************//**
PURPOSE: Check if there is a leak in the heap allocation when
            using SmartHeap - else always return success.

NOTES: Currently this method is unimplemented.
***********************************************************************/
SmStatus smos_HeapLeakCheck()
{
  return SM_SUCCESS;

} // end smos_HeapLeakCheck

/*******************************************************************//**
PURPOSE: Get the size of the heap which has been used when
            using SmartHeap else return 0.

NOTES: 
***********************************************************************/
unsigned long smos_HeapGetSizeUsed()
{
  return 0;

} // end smos_HeapGetSizeUsed

/*******************************************************************//**
PURPOSE: Get the total number of heap memory blocks allocated
            when using SmartHeap - else return 0

NOTES: 
***********************************************************************/
unsigned long smos_HeapGetTotalCount()
{
  return 0;

} // end smos_HeapGetTotalCount

/*******************************************************************//**
PURPOSE: Free the entire default memory pool when using SmartHeap,
            else do nothing.

NOTES: 
***********************************************************************/
void smos_FreeAllMemory()
{

} // end smos_FreeAllMemory

