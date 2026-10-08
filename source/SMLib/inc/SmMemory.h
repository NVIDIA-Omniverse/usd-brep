// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmMemory.h
* PURPOSE: Header file for memory manager.
**********************************************************************/

#ifndef __SMOS_MEMORY_H__
#define __SMOS_MEMORY_H__

#ifndef _INC_STDDEF
#include <stddef.h>
#endif

#ifndef _INC_SMOS_TYPES
#include <SmTypes.h>
#endif

#ifndef __SMMESSAGES_H__
#include <SmMessages.h>
#endif

#include <cstdint>

//#define SM_USING_MEMORY_MACROS

// Some platforms require char* instead of void* for their malloc/free/realloc calls.  
// If it does then define SM_MALLOC_TAKES_CHAR in the compile definitions.
#if defined(SM_MALLOC_TAKES_CHAR)
typedef char * SM_MALLOC_TYPE;
#else 
typedef void * SM_MALLOC_TYPE;
#endif

#define SM_MALLOC_CAST(v) ((SM_MALLOC_TYPE)(v))

/*******************************************************************//**
PURPOSE: interface for calloc/free/memcpy/memset/memmove.  
    All memory allocated should pass through these functions, 
      smos_Calloc  = allocate a block of memory of Num Sized objects init to 0
      smos_Free    = Free memory block allocated with smos_Calloc
      smos_MemCpy  = quick copy of one non-overlapping block of memory into another
      smos_MemSet  = quick set of memory block with a fill value
      smos_MemMove = copy of one overlapping block of memory into another

NOTES: 1. Never use calloc, free, memcpy, memset, or memmove directly.
       2. Never use these methods to allocate dynamic classes (classes with virtual functions)
            because the virtual function table for each object will not
            be properly initialized and will get corrupted by being overwritten.
          (using them for arrays of pointers to dynamic classes is just fine.)

Arguments: int          num       // in : number of objects in requested memory block
           int          size      // in : size in bytes of objects requested in memory block
           void       * dest      // in : pointer to target memory block
           const void * src       // in : pointer to source memory block
           size_t       count     // in : size of memory block
           int          character // in : fill value for memory block - usually 0
***********************************************************************/
#ifdef SM_USING_MEMORY_MACROS
#ifdef SM_USE_TBB_SCALABLE
  #define smos_Calloc(num, size)                       (void*)scalable_calloc((size_t)(num), (size_t)(size))
  #define smos_Free(ptr_mem_block)                            scalable_free(SM_MALLOC_CAST((ptr_mem_block)))
  #define smos_Malloc(size)                            (void*)scalable_malloc((size_t)(size))
#else
  #define smos_Calloc(num, size)                       (void*)calloc((size_t)(num), (size_t)(size))
  #define smos_Free(ptr_mem_block)                            free(SM_MALLOC_CAST((ptr_mem_block)))
  #define smos_Malloc(size)                            (void*)malloc((size_t)(size))
#endif
#else // no SM_USING_MEMORY_MACROS
  SM_EXPORT void * smos_Calloc(size_t num,                        // in : okay for base and static class arrays, not okay for dynamic class arrays
                               size_t size) ;                     // in : 
  SM_EXPORT void   smos_Free  (void * ptr_mem_block);
  SM_EXPORT void * smos_Malloc(size_t size);
#endif // no SM_USING_MEMORY_MACROS

  /*
#ifdef _MSC_VER
#define smos_MemCpy(dest,src,count)     memcpy_s ((void*)(dest),(rsize_t)(count),(src),(rsize_t)(count))          // not okay for dynamic class object arrays
#else
#define smos_MemCpy(dest,src,count)     memcpy ((void*)(dest),(src),(rsize_t)(count))          // not okay for dynamic class object arrays
#endif
*/

/*******************************************************************//**
PURPOSE:

NOTES: 
***********************************************************************/
inline SmStatus smos_MemCpy
 (void       * dest,        // out: ptr to mem array to copy to 
  const void * src,         // in : ptr to mem array to copy from
  size_t       src_count,   // in : size of src,  (typedef unsigned __int64 size_t;)
  size_t       dest_count)  // in : size of dest, (typedef unsigned __int64 size_t;)
{
  if (src_count > dest_count)
    { return SM_ERR ;}
  
  uintptr_t addr1 = uintptr_t(src);
  uintptr_t addr2 = uintptr_t(dest);
  
  SmBoolean bOverlap = FALSE;
  
  if (addr1 < addr2)
    {
      if (addr2 - addr1 >= src_count)
        {
          bOverlap = TRUE;
        }
    }
  else if (addr1 > addr2)
    {
      if (addr1 - addr2 >= dest_count)
        {
          bOverlap = TRUE;
        }
    }
  
  if (bOverlap)
    {
      memmove(dest, src, src_count);
    }
  else
    {
      memcpy(dest, src, src_count);
    }
  
  return SM_SUCCESS;

} // end smos_MemCpy

#define smos_MemSet(dest,CharVal,count) memset ((void*)(dest),(int)(CharVal),(size_t)(count)) // not okay for dynamic class object arrays
#define smos_MemMove(dest,src,count)    memmove((void*)(dest),(src),(size_t)(count))          // not okay for dynamic class object arrays

SM_EXPORT SmStatus smos_HeapCheck(const TCHAR* message,long num);
SM_EXPORT SmStatus smos_HeapLeakCheck();
SM_EXPORT unsigned long smos_HeapGetSizeUsed(void);
SM_EXPORT unsigned long smos_HeapGetTotalCount(void);
SM_EXPORT void smos_FreeAllMemory();

/*******************************************************************//**
PURPOSE: This object is a stack based deletion object for memory
         allocated with smos_Calloc().  

NOTES: This class only works on pointers to memory allocated with calls to smos_Calloc().
       This class calls smos_Free on its contained pointer when it goes out of scope.

       Use class SmObjDelete(SmObject *pObj) for objects derived from SmObject 
         and allocated with new() calls.

Example: Automatic Deletion of stack object a when exiting scope
         { char *a = smos_Calloc(25);
           SmMemDelete sCleanup(a); 
         } // automatically deletes a on exit of scope

Example: Survival of stack object a when exiting scope
         { char *a = smos_Calloc(25);
           SmMemDelete sCleanup(a);    // a is specified as a temporary object
            . . .                      // code changes a's status to a saved object
           sCleanup.Clear();           // allows a to exist outside of current scope
         } // a will now exist outside of this scope
***********************************************************************/
class SmMemDelete 
{
 protected:
  void *m_pMem;               // the memory to free when this object is deleted

 public:
  // constructors, destructor
  SmMemDelete()               { m_pMem = NULL; }
  SmMemDelete(void *pMem)     { m_pMem = pMem; }
  ~SmMemDelete()              { if (m_pMem) { smos_Free(m_pMem); m_pMem = NULL ; } }
                              
  // methods                  
  void Clear()                { m_pMem = NULL;}
  void SetMem(SmObject *pMem) { m_pMem = pMem; }
} ; // end class SmMemDelete

/*******************************************************************/ /**
 PURPOSE: Allocator for std library data members, specifically std::unordered_map. to force
          memory allocation through the same allocators as the rest of the library

 NOTES: 
 ***********************************************************************/

template <class T>
struct SmAllocator
{
  typedef T value_type;

  SmAllocator() = default;

  template <class U>
  constexpr SmAllocator(const SmAllocator<U>&) noexcept
  {
  }

  T* allocate(std::size_t n)
  {
//#ifdef SM_USE_EXCEPTIONS // following was removed to satisfy compiler for Alias connector.
//      #undef max
//      if (n > std::numeric_limits<std::size_t>::max() / sizeof(T))
//          throw std::bad_array_new_length();
//#endif
      auto p = static_cast<T*>(smos_Malloc(n * sizeof(T)));

      if (!p)
      {
#ifdef SM_USE_EXCEPTIONS
          throw std::bad_alloc();
#endif
      }

#ifdef SM_DEBUG_CODE
      report(p, n);
#else
      SM_REF1(n);
#endif

      return p;
  }

  void deallocate(T* p, std::size_t n) noexcept
  {
#ifdef SM_DEBUG_CODE
      report(p, n, 0);
#else
      SM_REF1(n);
#endif
      smos_Free((void*)p);
  }

  private:
#ifdef SM_DEBUG_CODE
  void report(T* p, std::size_t n, bool alloc = true) const
#else
  void report(T*, std::size_t, bool) const // included to avoid issues with unreferenced formal parameters in release.
#endif
  {
#ifdef SM_DEBUG_CODE
      SmBoolean bDebug = FALSE; // set to true for reporting of memory allocation/deallocation while in debug.
      if (bDebug)
      {
          TCHAR sBuff[SM_TBLOCK_SIZE];
          smos_sprintf(sBuff, _T("%s"), alloc ? _T("Alloc: ") : _T("Dealloc: "));
          smos_WriteBuffer(sBuff);

          smos_sprintf(sBuff, _T("[%zu] at location "), sizeof(T) * n);
          smos_WriteBuffer(sBuff);

          smos_sprintf(sBuff, _T("0x%p\n"),p);
          smos_WriteBuffer(sBuff);
      }
#endif
  }// end struct SmAllocator.

  //template <class T, class U>
  //bool operator==(const SmAllocator<T>&, const SmAllocator<U>&)
  //{
  //    return true;
  //}

  //template <class T, class U>
  //bool operator!=(const SmAllocator<T>&, const SmAllocator<U>&)
  //{
  //    return false;
  //}
}; 

#endif // !__SMOS_MEMORY_H__
