// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmSArray.h
* PURPOSE: Header file for SmSArray object.  
**********************************************************************/

/* Array Object definitions */
#ifndef __SmSArray_H__
#define __SmSArray_H__

#define SmSArray SmTArray

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif // __SMTARRAY_H__

// replace all SmSArray uses with SmTArray
#if 0

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#include <SmAssertArray.h>

#ifdef _WIN32
#define SM_SARRAY_TEMPLATE_PREDECLARATION(x)  \
template class SM_EXPORT SmSArray<x>;
#else
#define SM_SARRAY_TEMPLATE_PREDECLARATION(x)
#endif


/*******************************************************************//**
PURPOSE: This object is templated container class which manages a 
    dynamic array of the given TYPE.  This type is specialized to work
    for arrays of structures or unknown types.

NOTES: This object is highly optimized because it is heavily
    used.
***********************************************************************/
// #define SmSArray SmTArray
template<class TYPE> class SmSArray : public SmObject
{
  friend class SmMemBlockMgr;

 // Implementation
 protected:
    ULONG     m_lSize;          // Current Array Size: Number of m_pData used spots      GetSize()
    ULONG     m_lMaxSize;       // Max Array Size:     Number of m_pData allocated spots GetDataSize()
    TYPE    * m_pData;          // the array of data 
    SmBoolean m_bIsBorrowed;    // TRUE =m_pData is borrowed, not freed when SmSArray is deleted
                                // FALSE=m_pData is local, freed when SmSarray is deleted or 
                                //                         size is set to 0.
    inline void SetAtGrow(ULONG nIndex, const TYPE & pNewElement);

public:
    ~SmSArray();
 
    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmSArray,SmObject,SmSArray_TYPE);
    inline SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                 SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                           //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                           //      default:[SM_LEVEL_0] 
                                 SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                 SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                                const ;


    // Construction
    inline SmSArray(ULONG lMaxSize=0, TYPE * pOptPtrArray=NULL, ULONG lSize=0);
    inline SmSArray(const SmContext & crContext);

    // Attributes
    inline ULONG  GetSize()      const   { return m_lSize; }
    inline ULONG  GetDataSize()  const   { return m_lMaxSize; }
    inline TYPE * GetDataArray() const   { return m_pData; }
    inline void SetSize(ULONG nNewSize);
    inline void SetDataSize(ULONG nNewDataSize);
    inline void ReSet();
    inline void RemoveAll();

    // Accessing elements
    inline TYPE GetAt(ULONG nIndex) const;                      // gwc: SmTArray returns TYPE &
    inline void SetAt(ULONG nIndex, const TYPE & pNewElement);  // gwc: SmTArray takes TYPE sNewElement  
    inline TYPE& operator[] (ULONG lIndex);
    inline TYPE operator[] (ULONG lIndex) const;                // gwc: SmTArray returns const TYPE &
    inline TYPE GetLast() const;

    // Potentially growing the array
    inline ULONG Add(const TYPE & pNewElement);
    inline ULONG Append(const SmSArray<TYPE>& rSrc);
    inline void Copy(const SmSArray<TYPE>& rSrc);
    inline void InsertAt(ULONG nIndex, const TYPE & pNewElement, ULONG nCount=1);   // gwc: SmTArray takes TYPE sNewElement
    inline void RemoveAt(ULONG nIndex, ULONG nCount=1);                             // gwc: SmTArray rewritten to avoid ULONG underflows
    inline void InsertAt(ULONG nStartIndex, SmSArray<TYPE>* pNewArray);
    inline void RemoveLast();

    inline ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const ; // return memory used and memory allocated in bytes                                 

} ; // end template<class TYPE> class SmSArray

/*******************************************************************//**
PURPOSE:

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
template<class TYPE> 
inline SmBoolean SmSArray<TYPE>::AssertValid
 (SmAssertArray    * pAList,                // i/o: Accumulating list of failed Asserts, NULL to ignore
  SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                            //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                            //      default:[SM_LEVEL_0] 
  SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
  SmTArray<ULONG>  * , pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const 
{
  // call the base class AssertValid
  SmBoolean bRtn = (  (eTestLevel != SM_LEVEL_GIVEN)
                    ? SmObject::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
                    : TRUE ) ;

  if (m_pData == NULL)
    {
      bRtn = (m_lSize    == 0) ? bRtn : FALSE ;
      bRtn = (m_lMaxSize == 0) ? bRtn : FALSE ;
    }
  else
    {
      bRtn = (m_lSize <= m_lMaxSize) ? bRtn : FALSE ;
    }

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmSArray<TYPE>::AssertValid

/*******************************************************************//**
PURPOSE: Return a reference to the element at the given index.  This
    reference may be used to either set or query the value of the element.

NOTES: The lIndex must be less then the current size (GetSize)
    or it will generate an assertion failure.
***********************************************************************/
template<class TYPE> 
inline TYPE & SmSArray<TYPE>::operator[] (ULONG lIndex)
{ SM_ASSERT(lIndex < m_lSize); return m_pData[lIndex]; }


/*******************************************************************//**
PURPOSE: Return the value of the element at the given index.

NOTES: The lIndex must be less then the current size (GetSize)
    or it will generate an assertion failure.
***********************************************************************/
template<class TYPE> 
inline TYPE SmSArray<TYPE>::operator[] (ULONG lIndex) const
{ SM_ASSERT(lIndex < m_lSize); return m_pData[lIndex]; }


#define MIN_ARRAY_SIZE 16

/*******************************************************************//**
PURPOSE: Set the size of the active elements for the template array.    

NOTES: This method will only free the data memory of this array
   if the size is set to zero.  
***********************************************************************/
template<class TYPE> 
inline void SmSArray<TYPE>::SetSize(ULONG nNewSize)
{
    SM_ASSERT_VALID_NO_STREAM(this); 

    if (nNewSize == 0)
    {
        // shrink to nothing
        if (m_pData && ! m_bIsBorrowed) { smos_Free(m_pData); m_pData = NULL ; }
        m_lSize       = 0 ;
        m_lMaxSize    = 0;
        m_pData       = NULL;
        m_bIsBorrowed = FALSE;
    }
    else if (m_pData == NULL)
    {
        ULONG nNewMaxSize = MIN_ARRAY_SIZE;
        while (nNewMaxSize < nNewSize) {
            nNewMaxSize = nNewMaxSize * 2;
        }
        m_pData = (TYPE*) smos_Calloc(1, nNewMaxSize * sizeof(TYPE));

        m_lSize    = nNewSize;
        m_lMaxSize = nNewMaxSize;
    }
    else if (nNewSize <= m_lMaxSize)
    {
        if (nNewSize > m_lSize)
        {
            // initialize the new elements
            smos_MemSet(&m_pData[m_lSize], 0, (nNewSize-m_lSize) * sizeof(TYPE));
        }
        m_lSize = nNewSize;
    }
    else
    {
        // otherwise, grow array
        ULONG nNewMaxSize = MIN_ARRAY_SIZE;
        while (nNewMaxSize < nNewSize) {
            nNewMaxSize = nNewMaxSize * 2;
        }

        SM_ASSERT(nNewMaxSize >= m_lMaxSize);  // no wrap around

        TYPE* pNewData = (TYPE*) smos_Calloc(1, nNewMaxSize * sizeof(TYPE));
        //        void** pNewData = (void**) new unsigned char[nNewMax * sizeof(void*)];

        // copy new data from old
        SE(smos_MemCpy(pNewData, m_pData, m_lSize * sizeof(TYPE), nNewMaxSize * sizeof(TYPE));

        // construct remaining elements
        SM_ASSERT(nNewSize > m_lSize);

        // smos_MemSet(&pNewData[m_lSize], 0, (nNewSize-m_lSize) * sizeof(void*));


        // get rid of old stuff (note: no destructors called)
        if (!m_bIsBorrowed) { smos_Free(m_pData); m_pData = NULL ; }
        m_lSize       = nNewSize;
        m_lMaxSize    = nNewMaxSize;
        m_pData       = pNewData;
        m_bIsBorrowed = FALSE;
    }
}

/*******************************************************************//**
PURPOSE: Set the element at the given index and optionally grow the
    array if the index is larger than the current size.

NOTES: 
***********************************************************************/
template<class TYPE> 
inline void SmSArray<TYPE>::SetAtGrow(ULONG nIndex,const TYPE & pNewElement)
{
  SM_ASSERT_VALID_NO_STREAM(this);

  if (nIndex >= m_lSize)
    { SetSize(nIndex+1); }
  m_pData[nIndex] = pNewElement;
}


/*******************************************************************//**
PURPOSE: Removes all elements from the templated array and returns the memory
    used by the array.

NOTES: 
***********************************************************************/
template<class TYPE> 
inline void SmSArray<TYPE>::RemoveAll()
    { SetSize(0); }

/*******************************************************************//**
PURPOSE: Return the element at a given index in the template array.

NOTES: The nIndex must be less then the current size (GetSize)
    or it will generate an assertion failure.
***********************************************************************/
template<class TYPE> 
inline TYPE SmSArray<TYPE>::GetAt(ULONG nIndex) const
    { SM_ASSERT(nIndex < m_lSize);
        return m_pData[nIndex]; }


/*******************************************************************//**
PURPOSE: Set the element at the given index in the template array.

NOTES: The nIndex must be less then the current size (GetSize)
    or it will generate an assertion failure.
***********************************************************************/
template<class TYPE> 
inline void SmSArray<TYPE>::SetAt(ULONG nIndex,const TYPE & pNewElement)
{ SM_ASSERT(nIndex < m_lSize);
  if(&pNewElement != & (m_pData[nIndex]))
    { m_pData[nIndex] = pNewElement; }
}

/*******************************************************************//**
PURPOSE: Return the value of the last element in the template array.

NOTES: This method is useful for doing stack based operations.
***********************************************************************/
template<class TYPE> 
inline TYPE SmSArray<TYPE>::GetLast() const
    { SM_ASSERT(m_lSize > 0);
        return m_pData[m_lSize-1]; }


/*******************************************************************//**
PURPOSE: Add a new element to the end of the templated array and 
   expand the internal memory size of the array as necessary.  Note that
   this operation always causes the size to increase by one (GetSize).

NOTES: 
***********************************************************************/
template<class TYPE> 
inline ULONG SmSArray<TYPE>::Add(const TYPE & pNewElement)
    { ULONG nIndex = m_lSize;
        if (nIndex < m_lMaxSize) { m_pData[nIndex]=pNewElement; m_lSize++; }
        else { SetAtGrow(nIndex, pNewElement);}
        return nIndex; }


/*******************************************************************//**
PURPOSE: This method sets the size of the array (GetSize) to zero without
    freeing the memory of the data array.  

NOTES:
***********************************************************************/
template<class TYPE> 
inline void SmSArray<TYPE>::ReSet() { m_lSize = 0; if (m_pData) smos_MemSet(m_pData,0,sizeof(void*)); }


/*******************************************************************//**
PURPOSE: This constructor allows four different options for the
    construction of a templated array.  
    
NOTES:
    If no arguments are given, a zero size
    array is constructed with no internal data array allocated.  
    
    If the lMaxSize is given and is not zero, an array is constructed with size
    of zero but which has an internal array allocated which can contain
    up to lMaxSize elements.  
    
    If the pOptPtrArray is given (not NULL) then the pointer is used as 
    the initial data pointer and must have lMaxSize elements in it.  
    In this case we refer to the data array as being borrowed from somewhere.  
    Note that this does not prevent the array from growing larger than this 
    size but if it does then the pOptPtrArray will no longer be used.  
    It will however not be freed as would normally happen to the
    internal data array.  Therefore the pOptPtrArray could exist on the
    stack, inside of another object, etc.  
    
    If the initial size is given, then the array is set to this size and 
    may be accessed using the [] operator.  

NOTES: It is recomended that only expert users utilize the pOptPtrArray.
***********************************************************************/
template<class TYPE> 
inline SmSArray<TYPE>::SmSArray(ULONG lMaxSize, TYPE * pOptPtrArray, ULONG lSize)
{ 
    SM_ASSERT(lSize <= lMaxSize);
    m_cpContext = NULL;
    if (pOptPtrArray) {
        m_lSize       = 0;
        m_lMaxSize    = lMaxSize;
        m_pData       = pOptPtrArray;
        m_bIsBorrowed = TRUE;
    }
    else {
        m_lSize       = 0 ;
        m_lMaxSize    = 0;
        m_pData       = NULL;
        m_bIsBorrowed = FALSE;
        if (lMaxSize > 0) SetSize(lMaxSize); 
        m_lSize = 0; 
    }
    m_lSize = lSize; 
}

/*******************************************************************//**
PURPOSE: Heap based constructor for SmSArray
    

NOTES: Use this constructor when creating an array on the heap.
***********************************************************************/
template<class TYPE> 
inline SmSArray<TYPE>::SmSArray(const SmContext & crContext)
{ 
    m_cpContext = &crContext;
    m_lSize       = 0;
    m_lMaxSize    = 0;
    m_pData       = NULL;
    m_bIsBorrowed = FALSE;
}

/*******************************************************************//**
PURPOSE: Remove the last element of the array and decrease the size
    by one.

NOTES: This method is often used in conjunction with GetLast to
    perform stack based operations.
***********************************************************************/
template<class TYPE> 
inline void SmSArray<TYPE>::RemoveLast()
    { SM_ASSERT(m_lSize > 0); m_lSize --; }

/*******************************************************************//**
PURPOSE: Destructor for the template array.  This destructor frees
   the internal data array if it was not a borrowed array.

NOTES: 
***********************************************************************/
template<class TYPE> 
inline SmSArray<TYPE>::~SmSArray()
{
  SM_ASSERT_VALID_NO_STREAM(this);
  if (m_pData && !m_bIsBorrowed) 
    {
      smos_Free(m_pData);
    }

  // clear member values
  m_lSize       = 0;
  m_lMaxSize    = 0 ;
  m_pData       = NULL;
  m_bIsBorrowed = FALSE ; 

}

/*******************************************************************//**
PURPOSE: Set the data array size of the template array.

NOTES: 
***********************************************************************/
template<class TYPE> 
inline void SmSArray<TYPE>::SetDataSize(ULONG nNewSize)
{
    if (nNewSize == m_lMaxSize) return;
    if (nNewSize < m_lMaxSize) {
        TYPE* pNewData = (TYPE*) smos_Calloc(1, nNewSize * sizeof(TYPE));
            
        // copy new data from old
        m_lSize = smos_Min(m_lSize,nNewSize);
        SE(smos_MemCpy(pNewData, m_pData, m_lSize * sizeof(TYPE),nNewSize * sizeof(TYPE)));
    
        if (!m_bIsBorrowed) { smos_Free(m_pData); m_pData = NULL ; }
        m_lMaxSize    = nNewSize;
        m_pData       = pNewData;    
        m_bIsBorrowed = FALSE;
    }
    else if (nNewSize > m_lMaxSize) {
        TYPE* pNewData = (TYPE*) smos_Calloc(1, nNewSize * sizeof(TYPE));
            
        // copy new data from old
        if (m_pData) {
            SE(smos_MemCpy(pNewData, m_pData, m_lSize * sizeof(TYPE),nNewSize * sizeof(TYPE)));
            if (!m_bIsBorrowed) { smos_Free(m_pData); m_pData = NULL ; }
            m_bIsBorrowed = FALSE;
        }
        m_lMaxSize = nNewSize;
        m_pData    = pNewData;    
    }
}


/*******************************************************************//**
PURPOSE: Append the contents of another a template array to the end 
    of this array.

NOTES: The types of both template arrays should be the same.
***********************************************************************/
template<class TYPE> 
inline ULONG SmSArray<TYPE>::Append(const SmSArray<TYPE>& rSource)
{
    SM_ASSERT_VALID_NO_STREAM(this);
    SM_ASSERT(this != &rSource);   // cannot append to itself

    if (rSource.m_lSize > 0) {
        ULONG nOldSize = m_lSize;
        SetSize(m_lSize + rSource.m_lSize);
        SE(smos_MemCpy(m_pData + nOldSize, rSource.m_pData, rSource.m_lSize * sizeof(TYPE),rSource.m_lSize * sizeof(TYPE)));
    }
    return m_lSize;
}

/*******************************************************************//**
PURPOSE: Copy the contents of a template array into this template
   array.

NOTES: 
***********************************************************************/
template<class TYPE> 
inline void SmSArray<TYPE>::Copy(const SmSArray<TYPE>& rSource)
{
    SM_ASSERT_VALID_NO_STREAM(this);
    SM_ASSERT(this != &rSource);   // cannot append to itself

    SetSize(rSource.m_lSize);
    if (rSource.m_lSize > 0) {
        SE(smos_MemCpy(m_pData, rSource.m_pData, rSource.m_lSize * sizeof(TYPE), rSource.m_lSize * sizeof(TYPE)));
    }

}

/////////////////////////////////////////////////////////////////////////////



/*******************************************************************//**
PURPOSE: Insert an element nCount times into a template array at a given index.

NOTES: 
***********************************************************************/
template<class TYPE> 
inline void SmSArray<TYPE>::InsertAt(ULONG nIndex, const TYPE & pNewElement, ULONG nCount)
{
    SM_ASSERT_VALID_NO_STREAM(this);

    if (nIndex >= m_lSize)
    {
        // adding after the end of the array
        SetSize(nIndex + nCount);  // grow so nIndex is valid
    }
    else
    {
        // inserting in the middle of the array
        ULONG nOldSize = m_lSize;
        SetSize(m_lSize + nCount);  // grow it to new size
        // shift old data up to fill gap
        smos_MemMove(&m_pData[nIndex+nCount], &m_pData[nIndex], (nOldSize-nIndex) * sizeof(TYPE));

        // re-init slots we copied from

        smos_MemSet(&m_pData[nIndex], 0, nCount * sizeof(TYPE));

    }

    // insert new value in the gap
    SM_ASSERT(nIndex + nCount <= m_lSize);
    while (nCount--)
        m_pData[nIndex++] = pNewElement;
}


/*******************************************************************//**
PURPOSE: Remove nCount elements from the template array starting at
    the given nIndex.

NOTES: 
***********************************************************************/
template<class TYPE> 
inline void SmSArray<TYPE>::RemoveAt(ULONG nIndex, ULONG nCount)
{
    SM_ASSERT_VALID_NO_STREAM(this);
    SM_ASSERT(nIndex + nCount <= m_lSize);

    // just remove a range
    ULONG nMoveCount = m_lSize - (nIndex + nCount);

    if (nMoveCount)
        SE(smos_MemCpy(&m_pData[nIndex], &m_pData[nIndex + nCount],
            nMoveCount * sizeof(TYPE), nMoveCount * sizeof(TYPE)));
    m_lSize -= nCount;
}


/*******************************************************************//**
PURPOSE: Insert the elements of a template array into this template
    array starting at the given index.

NOTES: 
***********************************************************************/
template<class TYPE> 
inline void SmSArray<TYPE>::InsertAt(ULONG nStartIndex, SmSArray<TYPE>* pNewArray)
{
    SM_ASSERT(pNewArray != NULL);
    SM_ASSERT_VALID_NO_STREAM(this);
    SM_ASSERT_VALID_NO_STREAM(pNewArray);

    if (pNewArray->GetSize() > 0)
    {
        InsertAt(nStartIndex, pNewArray->GetAt(0), pNewArray->GetSize());
        for (ULONG i = 0; i < pNewArray->GetSize(); i++)
            SetAt(nStartIndex + i, pNewArray->GetAt(i));
    }
}

/*******************************************************************//**
PURPOSE: Return in bytes all memory used and all memory allocated

NOTES: 
  Frees current m_pData pointer if its not NULL and not borrowed.
  Won't free the input pPtrArray, even if this array gets resized.
***********************************************************************/
template<class TYPE> 
inline ULONG SmSArray<TYPE>::GetMemoryUsed   // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated)             // out: bigger size of all allocated memory in bytes
  const
{
  // set output
  rlMemoryAllocated =    sizeof(SmSArray<TYPE>)
                      + (m_bIsBorrowed ? 0 : m_lMaxSize * sizeof(TYPE)) ;

  // return used memory value
  return(   sizeof(SmSArray<TYPE>)
         + (m_bIsBorrowed ? 0 : m_lSize * sizeof(TYPE))) ; 

} // end SmTArray<TYPE>::GetMemoryUsed

/////////////////////////////////////////////////////////////////////////////
// Diagnostics



template<class TYPE> 
inline void SmSArray<TYPE>::Dump() const
{
    TCHAR buff[SM_TBLOCK_SIZE];
    smos_sprintf(buff,_T("SmSArray size=%ld, alloc size=%ld  "),m_lSize,m_lMaxSize);
    smos_WriteBuffer(buff);
    SmObject::Dump();
    for (ULONG i = 0; i < m_lSize; i++) {
//        sm_DumpPrint(buff,i,m_pData[i]);
//        smos_WriteBuffer(buff);
    }
}



#endif // end replace all SmSArray uses with SmTArray 

#endif // !__SmSArray_H__

