// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmMemBlockMgr.h
* PURPOSE: Header file for SmMemBlockMgr object.
**********************************************************************/

          
#ifndef __SMMEMBLOCKMGR_H__
#define __SMMEMBLOCKMGR_H__

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

/*******************************************************************//**
PURPOSE: A growable array of elements optimized to 
  minimize the number of memory allocation calls as the list
  grows. 

  This class only allows growing or indexing of its elements.
  It does no shrinking or compaction.

  This class works in void* pointers because at the time it
    was implemented templates were not working well.

  memory is managed with calloc/free calls.  So do not use 
  SmMemBlockMgr on dynamic class objects (classes with virtual functions).

NOTES: 
   The MemBlockMgr contains a variable length array of (void *) MemBlocks pointers.
   A MemBlock pointer will point to a memory block or be NULL.
   Each memory block is broken up into a set of same sized elements.
     The number of elements in one memory block is m_lNumElementsPerBlock.
     The size in bytes of one element is m_lSizeOfElements.
     The size in bytes of one memory block is m_lNumElementsPerBlock * m_lSizeOfElements.
     The number of total elements actually being used is m_lNumActiveElements
 
   Caveat: DO NOT USE SmMemBlockMgr TO ALLOCATE CLASSSES WITH VIRTUAL FUNCTIONS.
     
     It turns out SmMemBlockMgr allocation does not work 
     correctly for C++ objects with virtual functions.  
     When a C++ object has a virtual function
     an extra pointer is added to the object which points to the
     virtual function table.  The SmMemBlockMgr trick for memory
     management properly allocates enough room for the virtual
     function table pointer, but since the allocation is done without
     going through new(), that pointer value is not properly set.
     As I found out today, any call to a virtual function on an
     object created using the SmMemBlockMgr class will result
     in either random behavior or an access violation depending
     on what bits happen to be in the virtual function pointer
     member.
  
***********************************************************************/
class SM_EXPORT SmMemBlockMgr
{
 private:
  ULONG             m_lSizeOfElements      = SM_UNDEF_ULONG ; // size of each element in bytes
  ULONG             m_lNumElementsPerBlock = SM_UNDEF_ULONG ; // number of elements in each block
  SmTArray<void*>   m_sBlocks;                                // An array of void * pointers, each ptr gets assigned to a
                                                              //   fixed size elem memory block as the number of elems increases.
  ULONG             m_lNumActiveElements   = SM_UNDEF_ULONG ; // number of elements actually being used

  void            * m_dData[16] = {NULL} ;                    // pointers for 1st 16 blocks of memory 
                                                              //   saves an alloc for cases only needing 16 elem blocks or less
 public:
  // construct SmMemBlockMgr with all size parameters set to zero
  SmMemBlockMgr();   

  // construct SmMemBlockMgr with all size parameters specified - without allocating any memory blocks
  SmMemBlockMgr(ULONG lSizeOfElements, 
                ULONG lNumElementsPerBlock  = 25);

  // destructor
  ~SmMemBlockMgr();

  // set size parameters - without allocating any memory blocks
  void Initialize(ULONG lSizeOfElements, 
                  ULONG lNumElementsPerBlock = 25) ;

  ULONG GetElementSize()              { return m_lSizeOfElements ; }
  // get memory block for given index 
  char * GetOrCreateElementBlock(ULONG lBlockIndex);

  // grow element list by one and return pointer to new element memory
  void * GetNewElement();

  // place pointers to all active elements into a TArray
  void   GetActiveElements(SmTArray<void*> & rActiveElements) const;

  // grow element list by one and copy given element into new element memory
  void   AddElementByCopy(const void *cpElementToCopy);

  // Return pointer to element memory for given index. Grows list if required.
  void * GetAt(ULONG lIndex);

  // Copy values into element memory for given index. Grows list if required.
  void   SetAt(ULONG lIndex, const void* cpElementToCopy);

  // return number of active elements
  ULONG  GetNumActiveElements() const { return m_lNumActiveElements; }

  // return size of used and all allocated memory in bytes.
  ULONG  GetMemoryUsed                             // rtn: smaller size of actually used memory in bytes
           (ULONG &rlMemoryAllocated) const ;      // out: bigger size of all allocated memory in bytes

  // free all memory blocks - reset all size parameters to zero
  void   ReSet();

} ; // end class SmMemBlockMgr

/*******************************************************************//**
PURPOSE: Free all allocated memory

NOTES: 
***********************************************************************/
inline void SmMemBlockMgr::ReSet
  ()
{
  // for every memory block
  for (ULONG i=0; i<m_sBlocks.GetSize(); i++) 
    {
      // get the memory block pointer and free it
      void *pMem = m_sBlocks[i];
      smos_Free(pMem);
      m_sBlocks[i] = NULL ;
    }

  // set all size parameters to zero
  m_sBlocks.ReSet();
  m_lNumActiveElements = 0;

} // end SmMemBlockMgr::ReSet

/*******************************************************************//**
PURPOSE: Default Constructor with all size values set to zero and
    no memory blocks allocated.

NOTES: 
  The new SmMemBlockMgr->m_sBlocks variable length array of void * pointers
  is started off using the SmMemBlockMgr->m_pData as a borrowed array.
  - this saves an alloc call for the 1st 16 blocks of memory allocated.

***********************************************************************/
inline SmMemBlockMgr::SmMemBlockMgr
  ()
: m_lSizeOfElements(0), 
  m_lNumElementsPerBlock(0),
  //      m_sBlocks(*((SmTArray<void*> *)&m_sData[0])), // too early to use this pointer
  m_lNumActiveElements(0)
{
  m_sBlocks.m_lSize       = 0;
  m_sBlocks.m_lMaxSize    = 16;
  m_sBlocks.m_pData       = m_dData;  // TArray starts with the array stored in this MemBlockMgr
  m_sBlocks.m_bIsBorrowed = TRUE;

} // end SmMemBlockMgr::SmMemBlockMgr Default Constructor

/*******************************************************************//**
PURPOSE: Sized Constructor with all size parameters set but
  no memory blocks allocated

NOTES: 
  The new SmMemBlockMgr->m_sBlocks variable length array of void * pointers
  is started off using the SmMemBlockMgr->m_pData as a borrowed array.
  - this saves an alloc call for the 1st 16 blocks of memory allocated.

***********************************************************************/
inline SmMemBlockMgr::SmMemBlockMgr
  (ULONG lSizeOfElements,         // in : size of each elements in bytes
   ULONG lNumElementsPerBlock)    // in : number of elements in each block
: m_lSizeOfElements(ALIGN_SIZE(lSizeOfElements)), 
  m_lNumElementsPerBlock(lNumElementsPerBlock),
  m_lNumActiveElements(0)
{
    m_sBlocks.m_lSize       = 0;
    m_sBlocks.m_lMaxSize    = 16;
    m_sBlocks.m_pData       = m_dData;
    m_sBlocks.m_bIsBorrowed = TRUE;

} // end SmMemBlockMgr::SmMemBlockMgr Constructor

/*******************************************************************//**
PURPOSE: Set the SmMemBlockMgr size parameters without allocating
 any memory blocks.

NOTES: 
***********************************************************************/
inline void SmMemBlockMgr::Initialize
  (ULONG lSizeOfElements,         // in : size of each elements in bytes
   ULONG lNumElementsPerBlock)    // in : number of elements in each block
{
    m_lSizeOfElements      = ALIGN_SIZE(lSizeOfElements);
    m_lNumElementsPerBlock = lNumElementsPerBlock;
    m_lNumActiveElements   = 0;

} // end SmMemBlockMgr::Initialize

/*******************************************************************//**
PURPOSE: Destructor

NOTES: 
***********************************************************************/
inline SmMemBlockMgr::~SmMemBlockMgr()
{
  for (ULONG i=0; i<m_sBlocks.GetSize(); i++) 
    {
      void *pMem = m_sBlocks[i];
      smos_Free(pMem);
      pMem = NULL ;
    }
  m_sBlocks.SetSize(0);

} // end SmMemBlockMgr::~SmMemBlockMgr

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
inline char * SmMemBlockMgr::GetOrCreateElementBlock
  (ULONG lBlockIndex)          // in : target memory block
{
  ULONG lError = 0;

  // allocate one memory block at a time until the lBlockIndex block has been created
  while (m_sBlocks.GetSize() < lBlockIndex+1) 
    {
      // allocate one memory block
      // warning conversion from 'size_t' to 'ULONG', possible loss of data
      // only okay to use smos_Calloc on base (double, ptr,.. ) static class (SmVector3d, SmExtent1d,.. ) objects.
      char *pNew = (char*)smos_Calloc((size_t)m_lSizeOfElements*m_lNumElementsPerBlock,
                                      1);
      // remember and break for errors
      if (pNew == NULL) { lError = 1;
                          break;
                        }
      // insert block into block array
      m_sBlocks.Add(pNew);
      //      m_sBlocks.Add((SmObject*)pNew);
    
    } // end while creating memory blocks

  // set output
  char *pRet = (lError == 0)
               ? (char*)m_sBlocks[lBlockIndex]
               : NULL ;

  // signal errors
  if(lError != 0) { SE(SM_ERR) ;
                  }

  // all done
  return pRet;

} // end SmMemBlockMgr::GetOrCreateElementBlock

/*******************************************************************//**
PURPOSE: return pointer to next element memory and increment 
  active element count

NOTES: Allocate new memory blocks if required.
***********************************************************************/
inline void * SmMemBlockMgr::GetNewElement() 
{
  // compute the next memory block index and element index
  ULONG lBlockIndex = m_lNumActiveElements / m_lNumElementsPerBlock;
  ULONG lElemIndex  = m_lNumActiveElements % m_lNumElementsPerBlock;
  
  // get the memory block  
  char * pBlock = GetOrCreateElementBlock(lBlockIndex);
  void * pRet = NULL;
  if (pBlock != NULL) 
    {
      // get the element memory
      pRet = (void*)&pBlock[lElemIndex*m_lSizeOfElements];
      m_lNumActiveElements++;
    }
  else 
    {
      SE(SM_ERR);
    }
  return pRet;

} // end SmMemBlockMgr::GetNewElement

/*******************************************************************//**
PURPOSE: place pointers to all active elements into a TArray

NOTES: 
***********************************************************************/
inline void SmMemBlockMgr::GetActiveElements
  (SmTArray<void*> & rActiveElements) const   // in : 
{
  // init output
  rActiveElements.ReSet();

  // while there are more elements
  while (rActiveElements.GetSize() < m_lNumActiveElements) 
    {
      // get next memory block pointer
      ULONG lBlockIndex = rActiveElements.GetSize() / m_lNumElementsPerBlock;
      char * pBlock     = ((SmMemBlockMgr*)this)->GetOrCreateElementBlock(lBlockIndex);

      // while there are more elements in this memory block
      ULONG lElementIndex = 0;
      while (   rActiveElements.GetSize() < m_lNumActiveElements 
             && lElementIndex < m_lNumElementsPerBlock) 
       {
          // get element memory pointer
          void *pElem = (void*)&pBlock[lElementIndex*m_lSizeOfElements];

          // add element to TArray
          rActiveElements.Add((SmObject*)pElem);

          // increment element index
          lElementIndex++;
       } // end iter every element in this memory block
    } // end iter every element
}

/*******************************************************************//**
PURPOSE:  grow element list by one and copy given element into 
             new element memory

NOTES: cpElementToCopy can not be a dynamic class object.
       it can be a pointer to a dynamic class.
***********************************************************************/
inline void SmMemBlockMgr::AddElementByCopy
  (const void *cpElementToCopy)    // in : target element to copy
{
  void *pNewElement = GetNewElement();
  // only okay to use smos_MemCpy on base (double, ULONG. . .) and static class (gw_CPOINT, SmVector3d. . .) objects.
  SE(smos_MemCpy(pNewElement, cpElementToCopy, m_lSizeOfElements, m_lSizeOfElements));

} // end SmMemBlockMgr::AddElementByCopy

/*******************************************************************//**
PURPOSE: Return pointer to element memory for given index.
  Grows list if required.

NOTES: 
***********************************************************************/
inline void * SmMemBlockMgr::GetAt(ULONG lIndex) 
{
  // get memory block and element indices for lIndex
  ULONG lBlockIndex = lIndex / m_lNumElementsPerBlock;
  ULONG lElemIndex  = lIndex % m_lNumElementsPerBlock;

  // get memory block pointer
  char * pBlock = GetOrCreateElementBlock(lBlockIndex);

  void * pRet = NULL;
  if (pBlock != NULL) 
    {
      // get element memory pointer
      pRet = (void*)&pBlock[lElemIndex*m_lSizeOfElements];
      SM_ASSERT(pRet != NULL);
      m_lNumActiveElements = smos_Max(m_lNumActiveElements,lIndex+1);
    }
  else 
    {
      SE(SM_ERR);
    }
  return pRet;

} // end SmMemBlockMgr::GetAt

/*******************************************************************//**
PURPOSE:  Copy given element values into specified element memory 
          for given index. Grows list if required.

NOTES: cpElementToCopy can not be a dynamic class object.
       It can be a pointer to a dynamic class object.
***********************************************************************/
inline void SmMemBlockMgr::SetAt
  (ULONG lIndex,                  // in : target element index
   const void* cpElementToCopy)   // in : element values to copy
{
  // get memory block and element indices for given lIndex
  ULONG lBlockIndex = lIndex / m_lNumElementsPerBlock;
  ULONG lElemIndex  = lIndex % m_lNumElementsPerBlock;

  // get memory block pointer
  char * pBlock = GetOrCreateElementBlock(lBlockIndex);
  SM_ASSERT(pBlock != NULL);

  // get element memory pointer
  void * pElem = (void*)&pBlock[lElemIndex*m_lSizeOfElements];
  SM_ASSERT(pElem != NULL);

  // copy target element values into target memory slot.
  // only okay to use smos_MemCpy on base (double, ULONG. . .) and static class (gw_CPOINT, SmVector3d. . .) objects.
  SE(smos_MemCpy(pElem, cpElementToCopy, m_lSizeOfElements, lElemIndex * m_lSizeOfElements));
  m_lNumActiveElements = smos_Max(m_lNumActiveElements,lIndex+1);

} // end SmMemBlockMgr::SetAt

/*******************************************************************//**
PURPOSE: Get memory used and allocated

NOTES: returns size of actually used memory in Bytes
      argument gets loaded with amount of memory allocated in Bytes
***********************************************************************/
inline ULONG SmMemBlockMgr::GetMemoryUsed    // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated)             // out: bigger size of all allocated memory in bytes
  const
{
  // get the big number of all bytes allocated for use by this object
  rlMemoryAllocated =   sizeof( this )                                       // includes: this and contained SmTArray<void*> m_sBlocks object - but not its variable array
                      + (  (m_dData == m_sBlocks.GetDataArray())             // variable array of void * pointers
                         ? 0                                                 //  - void * array already counted
                         : sizeof(void *) * (size_t)m_sBlocks.GetDataSize()) //  - add in void * array memory
                      + (  (size_t)m_lNumElementsPerBlock                    // total memory allocated for nodes
                         * (size_t)m_lSizeOfElements 
                         * (size_t)m_sBlocks.GetSize()) ;                    

  // return size of 'used' memory in bytes including all overhead
  //   with perfect tuning - actual memory could be reduced to this size
  //   but perfect tuning would require way too many allocs and frees - so its a performance/memory tradeoff.
  return (  sizeof( this )                                       // includes: this and contained SmTArray<void*> & m_sBlocks object - but not its variable array
          + (  (m_dData == m_sBlocks.GetDataArray())             // variable array of void * pointers
             ? 0                                                 //  - void * array already counted    
             : sizeof(void *) * (size_t)m_sBlocks.GetDataSize()) //  - add in void * array memory      
          + (  (size_t)m_lNumActiveElements                      // used elem block memory
             * (size_t)m_lSizeOfElements) ) ;                    //      
                                                                 //      
} // end SmMemBlockMgr::GetMemoryUsed


/*******************************************************************/ /**
 PURPOSE:
   Replaces SmMemBlockMgr
     - for cases where no indexing logic like GetAt() is required
     - in order to have a flexible block size, e.g. to allow squared size increases
     - in order to mix objects of different types into one allocator, which reduces the total number of allocations
 ***********************************************************************/
class SM_EXPORT SmArenaMemBlockMgr
{
 private:
  std::size_t m_initialBlockSize;  // initial size in bytes to allocate
  std::size_t m_lastBlockSize;     // size in bytes of the previously allocated block
  std::size_t m_nextIndexInBlock;  // index for the next block
  std::size_t m_totalSize;         // total bytes allocated

  SmTArray<char*> m_blocks;

 public:
  SmArenaMemBlockMgr();
  ~SmArenaMemBlockMgr();

  void Initialize(std::size_t initialBlockSize);

  std::size_t NewBlockSize(std::size_t bytes) const;

  void* Allocate(std::size_t bytes);
  void Reserve(std::size_t bytes);

  std::size_t Size() const;

  void Reset();
};

#endif // !__SMMEMBLOCKMGR_H__


