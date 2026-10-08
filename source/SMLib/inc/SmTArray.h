// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTArray.h
* PURPOSE: Header file for SmTArray object.
**********************************************************************/

/* Array Object definitions */
#ifndef __SMTARRAY_H__
#define __SMTARRAY_H__

#include <SmExtent1d.h>
#include <SmExtent2d.h>
#include <SmExtent3d.h>
#include <SmPeriodicExtent1d.h>
#include <random>

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMOBJECT_H__
#include <SmObject.h>
#endif

#ifndef __SMCONTEXT_H__
#include <SmContext.h>
#endif

#ifdef _WIN32
#define SM_TARRAY_TEMPLATE_PREDECLARATION(x) template class SM_EXPORT SmTArray<x>
#else
#define SM_TARRAY_TEMPLATE_PREDECLARATION(x)
#endif

#ifndef SM_PTR_ARRAY
// Make SmTArray<type*> name(MemSize, &LocalArray[MemSize], ArraySize = 0)
// example: SM_PTR_ARRAY(sFaces, SmFace, 16) ; 
// makes  : SmTArray<SmFace*> sFaces; // with sFaces.GetSize() == 0, sFaces.GetDataSize() == 16.
#define SM_PTR_ARRAY(name, type, MemSize) \
    type* name##Data[MemSize]; \
    SmTArray<type*> name(MemSize, (type**)name##Data)
#endif

// Make SmTArray<type > name(MemSize, &LocalArray[MemSize], ArraySize = 0)
#define SM_OBJ_ARRAY(name, type, MemSize) \
    type name##Data[MemSize]; \
    SmTArray<type> name(MemSize, (type*)name##Data)

// obsolete
// workaround hack to output any m_pData array as a set of void * ptrs - it helps with debugging
//   Why is this here? See the comment in SmTArray<TYPE>::Dump()
// SM_EXPORT void smTArray_WritePtrArray(void ** pData, ULONG lCnt) ; // in : m_pData cast to (void *)

/*******************************************************************//**
PURPOSE: This object is a templated container class which manages a
         dynamic array of the given TYPE.

WARNING: INTENDED USE: arrays of pointers, base, and static types i.e. classes without virtual functions
         DO NOT USE  : arrays of dynamic types, i.e. classes with virtual functions
                       (while arrays of pointers to dynamic types are just fine.)
                       BECAUSE     : memory is managed with calloc/free/memset calls

         smos_Calloc() calls do not initialize object virtual function table pointers properly
         smos_MemSet() calls corrupt the virtual function table pointers
                       of the objects within m_pData by overwriting them
                       leading to subtle runtime memory errors.

NOTES: This object is highly optimized because it is heavily used.

    Block Memory: SmTArray objects allocate memory in blocks to reduce
      the number of alloc calls. This block memory gets used as elements
      get added to the array. Block memory is stored under the m_pData pointer.

    Array and Block Sizes: SmTArray maintain two sizes,
      1. m_lSize    =  size of the Array (number of elements used), and
      2. m_lMaxSize =  size of memory allocated for elements.
      where m_lSize <= m_lMaxSize.

    Block Allocations: Memory allocations happen when the array is constructed
      and when adding an element or setting the array size causes the m_lSize
      to exceed the m_lMaxSize in which case a new larger array is allocated
      and m_lMaxSize is increased.

    Block Sizes: block sizes start at MIN_ARRAY_SIZE = 16 and are doubled
      until they exceed the minimum requested size.

    Block Freeing: The m_pData memory is freed when the SmTArray is
      deleted or when the array size is set to zero by a call to SetSize(0) ;

    Block Initialization: SmTarray objects can be constructed with an initial
      memory block.  The initial memory block's location can be passed into
      the constructor or allocated on the heap as part of the construction
      process.  This feature can be used to avoid a heap alloc/dealloc at run
      time for small temporary arrays by using a stack array or another
      preallocated array.  See examples below.


Example 1: Allocate a temporary array with no initial memory block. Memory is
           allocated on the heap when elements are added to the array.

           SmTArray<double> sDoubleArray ;      // array of doubles
           SmTArray<SmFace*> sFaceArray ;       // array of object pointers
           SmTArray<ULONG> sULongArray ;        // array of ULONG
           SmTArray<SmPoint3d> sControlPoints;  // array of objects

Example 2: Slight optimization: Allocate a temporary array with an initial
           memory block allocated on the heap. After the initial memory allocate,
           additional memory allocates only occur when the initial size is exceeded.
           so select the initial array size to be big enough for most cases - but
           not so large that it is a resource hog except when it needs to be.

           SmTArray<double> sDoubleArray(16) ;  // double array with room for 16 elems on the heap
           SmTArray<SmFace*> sFaceArray(256) ;  // object pointer array with room for 256 elems on the heap

Example 3: Another optimization - allocate a temporary array with memory
           on the stack avoiding a heap alloc at construction time.

           double sDoubleData[16] ;
           SmTArray<double> sDoubleArray(16,sDoubleData) ; // double array with room for 16 elems on the stack

           SmEdgeuse * sEUData[64];
                    SmTArray<SmEdgeuse*> sEdgeuses(64,sEUData);    // object pointer array with room for 64 elems on the stack

***********************************************************************/
template<class TYPE> class SM_EXPORT SmTArray : public SmObject
{
 friend class SmMemBlockMgr;

 protected:
    // inherited from SmObject:
    //   SmContext * m_pContext ;// access to 'global' memory

    ULONG     m_lSize;           // Current Array Size: Number of m_pData used spots      GetSize()
    ULONG     m_lMaxSize;        // Max Array Size:     Number of m_pData allocated spots GetDataSize()
    TYPE    * m_pData;           // the array of data
    SmBoolean m_bIsBorrowed;     // TRUE =m_pData is borrowed, not freed when SmTArray is deleted
                                 // FALSE=m_pData is local, freed when SmTarray is deleted or
                                 //                         size is set to 0.

    inline ULONG SetAtGrow(ULONG lIndex, TYPE sNewElement);

 public:
     // Needed for rangebased loop
    class iterator
      {
	       public:
		        iterator(TYPE* ptr) : ptr(ptr) {}
		        iterator operator++() { ++ptr; return *this; }
		        bool operator!=(const iterator& other) const { return ptr != other.ptr; }
		        const TYPE& operator*() const { return *ptr; }
	       private:
          TYPE* ptr = NULL;
	      } ;
	   iterator begin() const { return iterator(m_pData); }
	   iterator end()   const { return iterator(m_pData + m_lSize); }

    // Constructors, destructor
    inline SmTArray
    (
      ULONG  lMaxSize     = 0,              // in : number of slots allocated for m_pData or Size of given pOptPtrArray 
      TYPE * pOptPtrArray = NULL,           // in : optional preallocated array to use for storage                      
      ULONG  lSize        = 0,              // in : number of slots used in m_pData, less than or equal to lMaxSize.    
      SmBoolean bCheckBeenThroughNew=TRUE   // in : bCheckBeenThrough new for internal use only - always set to TRUE    
    );

    // empty constructor
    inline SmTArray(const SmContext & crContext);

    // copy constructor - allocates new memory and copies values from crSource
    inline SmTArray(const SmTArray<TYPE> & crSource);

    // destructor
    virtual ~SmTArray();

    // equality operator
    inline SmBoolean operator==(const SmTArray<TYPE> &crOther) const;

    // assignment operator: shallow copy array values from src to this. No deep copies for pointers.
    inline SmTArray& operator=(const SmTArray<TYPE> & src);

    // simple data access
    inline ULONG     GetSize      () const          { return m_lSize; }
    inline ULONG     GetDataSize  () const          { return m_lMaxSize; }
    inline TYPE    * GetDataArray () const          { return m_pData; }
    inline SmBoolean GetIsBorrowed() const          { return(m_bIsBorrowed) ; }

    inline void  SetAll     (const TYPE) ;          // set array to given value
    inline ULONG SetSize    (ULONG lNewSize);       // bump or shrink m_lSize, bump m_lMaxSize if needed, memset new array memory to '0', no constructors called
    inline ULONG SetDataSize(ULONG lNewDataSize);   // set m_lMaxSize, when m_lMaxSize grows, m_lSize stays constant, when m_lMaxSize shrinks, m_lSize shrinks.
    inline void  ReSet      ();                     // set m_lSize = 0, don't free array memory
    inline void  RemoveAll  ();                     // set m_lSize = 0, do    free array memory
    
    inline void  SetArray                           // set m_pData to point to borrowed array
    (
      ULONG lMaxSize,      ///< [in ]: Number of Slots allocated for m_pData or Size of given pOptPtrArray
      TYPE *pPtrArray,     ///< [in ]: optional preallocated array to use for storage
      ULONG lSize          ///< [in ]: Number of slots used in m_pData, less than or equal to lMaxSize.
    );

    inline void  SetIsBorrowed(SmBoolean bIsBorrowed) { m_bIsBorrowed = bIsBorrowed ; }
    inline ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const;  // return memory used and memory allocated in bytes
                              
    // predicates - see functions defined below for
    // SM_ARE_ARRAYS_SAME(a, b) and
    // SM_ARE_ARRAYS_SAME_TO_TOL(a, b, tol)

    // predicates
    inline SmBoolean IsEmpty     () const          { return (m_lSize == 0); }

    // Search Array for element: for SmTArray<PtrToObj> arrays - compares ptrs, 
    //                           for SmTArray<Obj> arrays - do deep operator== compares
    inline SmBoolean FindElement (TYPE sObjToFind, ULONG & lFoundIndex) const;
    inline SmBoolean FindElements(TYPE sObjToFind, SmTArray<ULONG> &rFoundIndices) const;
    inline SmBoolean IsIn        (TYPE sObjToFind)                      const { ULONG nIndx ;
                                                                                return(FindElement(sObjToFind, nIndx)) ;
                                                                              }
    // Search Arrays storing uninit elems, i.e. only check elems marked by BitMask, ArraySizeLimit:[sizeof(ULONG)*8]
    inline SmBoolean FindElementInMaskedArray(TYPE   pObjToFind,          // in : Search Tgt
                                              ULONG  lBitMask,            // in : only search ith elem when (lBitMask & (1<<i)) == TRUE
                                              ULONG& nFoundIndex) const ; // out: Indx of first MarkedItem equal to pObjToFind

    // Accessing elements
    inline       TYPE &  GetAt       (ULONG lIndex) ;                    // use for SmTArray<PtrToObj> and SmTArray<<obj> arrays
    inline const TYPE &  GetAt       (ULONG lIndex) const;               // use for SmTArray<PtrToObj> and SmTArray<<obj> arrays
    inline       TYPE    GetLast     () const;                           // use for SmTArray<PtrToObj> and SmTArray<<obj> arrays
    inline       TYPE &  operator[]  (ULONG lIndex) ;                    // only use for SmTArray<Obj> arrays
    inline const TYPE &  operator[]  (ULONG lIndex) const;               // only use for SmTArray<Obj> arrays
    inline       void    SetAt       (ULONG lIndex, TYPE sNewElement) ;  // use for SmTArray<PtrToObj> and SmTArray<<obj> arrays

    // find elements on both of 2 lists, (simple n**2 algorithm)
    inline void      FindCommonElements(const SmTArray<TYPE> &crOther, SmTArray<TYPE> & rResult) const;
    inline SmBoolean HasCommonElements(const SmTArray<TYPE> &crOther) const;

    // set rResult = this - crOther.  Works in place when rResult = *this.
    inline void      RemoveElements   (const SmTArray<TYPE> &crOther, SmTArray<TYPE> & rResult) const;
    inline void      RemoveIndices    (const SmTArray<ULONG> &crIndicesToRm, SmTArray<TYPE> & rResult) const;

    // Subtract one list from another.  (Same as RemoveElements().)
    inline void      FindUniqueElements(const SmTArray<TYPE> &crOther, SmTArray<TYPE> & rResult) const;
    inline void      GetDuplicates(SmTArray<TYPE> & rResult) const;  // warn: expensive O(n**2) algorithm - for use on small arrays

    inline SmBoolean RemoveDuplicates() ;

    // Pick some elements at random from an Array
    inline SmStatus  PickSomeElements
    (
      ULONG            lNumToGet,        ///< [in ]: Number of samples to retrieve
      SmTArray<TYPE> & rSamples          // out: Array of samples
    ) const ; 

    // Setting array elements
    inline ULONG     Add               (TYPE sNewElement);                         // add element to end of array
    inline SmBoolean AddUnique         (TYPE sNewElement);
    inline ULONG     Push              (TYPE sNewElement);                         // LIFO Stack
    inline SmBoolean Pop               (TYPE & rLastElement);                      // LIFO Stack, rtn: TRUE=Stack has elems, FALSE=empty Stack(no pop)
    inline SmBoolean Peak              (TYPE & rLastElement);                      // LIFO Stack, rtn: TRUE=Stack has elems, FALSE=empty Stack(no peak)
    inline ULONG     Append            (const SmTArray<TYPE>& rSrc);               // append rSrc to end of array, rtn preAppend array size
    inline ULONG     AppendUnique      (const SmTArray<TYPE>& rSrc);               // append rSrc to end of array, rtn preAppend array size
    inline ULONG     Copy              (const SmTArray<TYPE>& rSrc);               // set array = copy of rSrc elements
    inline ULONG     InsertAt  (ULONG lIndex, TYPE sNewElement, ULONG nCount=1);   // insert elements while expanding array
    inline ULONG     InsertAt  (ULONG nStartIndex, SmTArray<TYPE>* pNewArray);     // insert elements while expanding array
    inline SmBoolean Remove    (TYPE sTgtElement) ;                                // remove element and compress array, rtn TRUE if Elem is removed
    inline void      RemoveAt  (ULONG lIndex, ULONG nCount=1);                     // remove elements and compress array
    inline void      RemoveLast();

    // Operations
    inline ULONG     Transpose    (ULONG lNumRows);

    inline void      ReverseArray 
    (
      ULONG lBegin,  ///< [in ]: First element to be swaped
      ULONG lEnd     ///< [in ]: This index is one greater than the largest element of the swap.  
                     ///<  : Typically this is the size of the array and lBegin is zero.
    );

    inline void      RotateArray  (ULONG lIndexToBeFirst);
    inline void      CompressZeros() ;

    inline void      ShellSort    (SmBoolean (*LessThan)(        ///< [in ]: Function Ptr that returns TRUE when sElem1 < sElem2
                                    TYPE sElem1,                 ///< [in ]: sElem1 of sElem1 < sElem2
                                    TYPE sElem2,                 ///< [in ]: sElem2 of sElem1 < sElem2
                                    void *pOptUserData),         ///< [in ]: Optional Data needed for compare computation
                                    void *pOptUserData = NULL ); ///< [in ]: Value passed to every LessThan compare computation, default:[NULL]

    inline TYPE      GetMaxValue  (SmBoolean (*LessThan)(        ///< [in ]: Function Ptr that returns TRUE when sElem1 < sElem2
                                    TYPE sElem1,                 ///< [in ]: sElem1 of sElem1 < sElem2
                                    TYPE sElem2,                 ///< [in ]: sElem2 of sElem1 < sElem2
                                    void *pOptUserData),         ///< [in ]: Optional Data needed for compare computation
                                    void *pOptUserData = NULL ); ///< [in ]: Value passed to every LessThan compare computation, default:[NULL]

    inline TYPE      GetMinValue  (SmBoolean (*LessThan)(        ///< [in ]: Function Ptr that returns TRUE when sElem1 < sElem2               
                                   TYPE sElem1,                  ///< [in ]: in : sElem1 of sElem1 < sElem2
                                   TYPE sElem2,                  ///< [in ]: in : sElem2 of sElem1 < sElem2
                                   void *pOptUserData),          ///< [in ]: in : Optional Data needed for compare computation
                                   void *pOptUserData = NULL );  ///< [in ]: Value passed to every LessThan compare computation, default:[NULL]


    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmTArray,SmObject,SmTArray_TYPE);

} ; // end template<class TYPE> class SmTArray

// special case ARE_SAME functions for SmTArray<ULONG> and SmTArray<double>
inline SmBoolean SM_ARE_ARRAYS_SAME
           (SmTArray<ULONG> &a,
            SmTArray<ULONG> &b)            { if(&a == &b)      { return TRUE ; }
                                             ULONG ii ; SmBoolean bRtn =  a.GetSize() == b.GetSize() ;
                                             if(bRtn == FALSE) { return FALSE ; }
                                             for(ii=0;ii<a.GetSize() && bRtn;ii++)
                                               { bRtn &= SmTol::InTol( ((a[ii])-(b[ii])), SmTol::GetScaledZero((a[ii]),(b[ii]))) ; }
                                             return(bRtn) ;
                                           }
inline SmBoolean SM_ARE_ARRAYS_SAME_TO_TOL
           (SmTArray<ULONG> &a,
            SmTArray<ULONG> &b,
            double          tol)           { if(&a == &b)      { return TRUE ; }
                                             ULONG ii ; SmBoolean bRtn =  a.GetSize() == b.GetSize() ;
                                             if(bRtn == FALSE) { return FALSE ; }
                                             for(ii=0;ii<a.GetSize() && bRtn;ii++)
                                               { bRtn &= SmTol::InTol( ((a[ii])-(b[ii])), SmScaledZero(tol)) ; }
                                             return(bRtn) ;
                                           }
inline SmBoolean SM_ARE_ARRAYS_SAME
           (SmTArray<double> &a,
            SmTArray<double> &b)           { if(&a == &b)      { return TRUE ; }
                                             ULONG ii ; SmBoolean bRtn =  a.GetSize() == b.GetSize() ;
                                             if(bRtn == FALSE) { return FALSE ; }
                                             for(ii=0;ii<a.GetSize() && bRtn;ii++)
                                               { bRtn &= SmTol::InTol( ((a[ii])-(b[ii])), SmTol::GetScaledZero((a[ii]),(b[ii]))) ; }
                                             return(bRtn) ;
                                           }
inline SmBoolean SM_ARE_ARRAYS_SAME_TO_TOL
           (SmTArray<double> &a,
            SmTArray<double> &b,
            double           tol)          { if(&a == &b)      { return TRUE ; }
                                             ULONG ii ; SmBoolean bRtn =  a.GetSize() == b.GetSize() ;
                                             if(bRtn == FALSE) { return FALSE ; }
                                             for(ii=0;ii<a.GetSize() && bRtn;ii++)
                                               { bRtn &= SmTol::InTol( ((a[ii])-(b[ii])), SmScaledZero(tol)) ; }
                                             return(bRtn) ;
                                           }



/*******************************************************************//**
PURPOSE: Copy Constructor

NOTES:
  Allocates m_pData memory and copies values from crSource.m_pData to this
  m_pData.
***********************************************************************/
template<class TYPE>
inline SmTArray<TYPE>::SmTArray(const SmTArray<TYPE> & crSrc)
{
  m_lSize       = 0;
  m_lMaxSize    = 0;
  m_pData       = NULL;
  m_bIsBorrowed = FALSE;
  m_cpContext   = NULL;
  Append(crSrc);

} // end SmTArray<TYPE>::SmTArray constructor

/*******************************************************************//**
PURPOSE: Equal operator

NOTES:
***********************************************************************/
template<class TYPE>
inline SmTArray<TYPE> & SmTArray<TYPE>::operator= (const SmTArray<TYPE> & crSrc)
{
  if(&crSrc == this)
    { return *this ; }

  // base
  m_cpContext = crSrc.m_cpContext ;

  // array members
  ReSet();
  Append(crSrc);
  return *this;

} // end SmTArray<TYPE>::operator=

/*******************************************************************//**
PURPOSE: Destructor for the template array.

NOTES: This destructor frees the internal data array
       without calling delete on each of the array members
       if it was not a borrowed array.

       For a deep destructor that calls delete on each member
       see class SmObjsDelete<class type>
***********************************************************************/
template<class TYPE>
inline SmTArray<TYPE>::~SmTArray()
{
  if (m_pData && !m_bIsBorrowed)
    {
      smos_Free(m_pData);
    }

  // clear member values
  m_lSize       = 0;
  m_lMaxSize    = 0 ;
  m_pData       = NULL;
  m_bIsBorrowed = FALSE ;

} // end SmTArray<TYPE>::~SmTArray destructor

/*******************************************************************//**
PURPOSE: Equality operator for SmTArray

NOTES: This only does a shallow check - so only pointer values
       and not objects are compared when comparing a pair of
       SmTArrays of pointers.
***********************************************************************/
template<class TYPE>
inline SmBoolean SmTArray<TYPE>::operator==(const SmTArray<TYPE>& crOther)
 const
{
  // low work - same objects
  if(&crOther == this)
    { return TRUE ; }

  // locals
  ULONG ii ;
  SmBoolean bRtn = (m_lSize == crOther.m_lSize) ;

  if(bRtn)
    {
      // check equivalence of these objects
      for(ii=0;ii<m_lSize && bRtn;ii++)
        {
          bRtn &= (m_pData[ii] == crOther.m_pData[ii]) ;
        }
    }

  // all done
  return bRtn ;

} // end  SmTArray<TYPE>::operator==

/*******************************************************************//**
PURPOSE: Return a reference to the element at the given index.

NOTES: This reference may be used to either set or query the value of the element.
    The lIndex must be less then the current size (GetSize)
    or it will generate an assertion failure.
***********************************************************************/
template<class TYPE>
inline TYPE & SmTArray<TYPE>::operator[](ULONG lIndex)
{
#ifdef SM_DEBUG_CODE
  if ( lIndex >= m_lSize )
    {
      SM_ASSERT_BREAK_MSG(lIndex < m_lSize, _T("operator[]: lIndex greater than array m_lSize - pointer error"));  // breakpoint.
    }
#endif // SM_DEBUG_CODE
  return m_pData[lIndex];

} // end SmTArray<TYPE>::operator[]

/*******************************************************************//**
PURPOSE: Return the value of the element at the given index.

NOTES: The lIndex must be less then the current size (GetSize)
    or it will generate an assertion failure.
***********************************************************************/
template<class TYPE>
inline const TYPE& SmTArray<TYPE>::operator[](ULONG lIndex) const
{
  SM_ASSERT_BREAK(lIndex < m_lSize);
  return m_pData[lIndex];

} // end SmTArray<TYPE>::operator[]

#define MIN_ARRAY_SIZE 16

/*******************************************************************//**
PURPOSE: Set the size of the active elements for the template array and manage
         m_pData memory block.

NOTES: for all cases: sets m_lSize     = lNewSize; 
                           m_lMaxSize >= lNewSize
   1. when lNewSize == 0       - all existing m_pData is freed and all cnts set to zero
   2. when lNewSize <= m_lSize - only m_lSize is changed, m_pData and m_lMaxSize stay the same
   3. when lNewSize  > m_lSize - a new memBlock >= lNewSize is allocated,
                                 old m_pData values are copied into new memBlock
                                 old m_pData is freed
                                 m_pData ptr set to new memBlock
                                 m_lSize = lNewSize
RETURNS: old size.
         If memory allocation failure, returns SM_BIG_ULONG.
***********************************************************************/
template<class TYPE>
inline ULONG SmTArray<TYPE>::SetSize(ULONG lNewSize)
{
  if (lNewSize >= SM_BIG_ULONG || lNewSize == SM_UNDEF_ULONG)
    {
      SM_ASSERT_BREAK(lNewSize < SM_BIG_ULONG && lNewSize != SM_UNDEF_ULONG) ;
      return SM_BIG_ULONG;
    }

  ULONG lOldSize = m_lSize;

  if (lNewSize == 0)
    {
      // shrink to nothing
      if (m_pData && ! m_bIsBorrowed) { smos_Free(m_pData); m_pData = NULL ; }
      m_lSize       = 0;
      m_lMaxSize    = 0;
      m_pData       = NULL;
      m_bIsBorrowed = FALSE;
    }
  else if (m_pData == NULL)
    {
      ULONG lNewMaxSize = MIN_ARRAY_SIZE;
      while (lNewMaxSize < lNewSize)
        {
          lNewMaxSize = lNewMaxSize * 2;
        }
      // only okay to use smos_Calloc on base (double, ptr,.. ) static class (SmVector3d, SmExtent1d,.. ) objects.
      m_pData = (TYPE*) smos_Calloc(1, lNewMaxSize * sizeof(TYPE));
      if ( m_pData == NULL )
        { SM_ASSERT_BREAK(FALSE) ;
          ReSet();
          return SM_BIG_ULONG;
        }

      m_lSize    = lNewSize;
      m_lMaxSize = lNewMaxSize;
    }

  // when NewSize is less than already allocated size
  else if (lNewSize <= m_lMaxSize)
    {
      // when newSize is more than used Size
      if (lNewSize > m_lSize)
        {
          // initialize the new elements with '0' characters -
          // WARNING: when TYPE has virtual functions, this call corrupts the virtual function table
          smos_MemSet((void*)(&m_pData[m_lSize]), 0, (lNewSize-m_lSize) * sizeof(TYPE));
        }

      // set size - this can shrink an array used size without redoing the memory allocation
      m_lSize = lNewSize;
    }
  else // m_pData != NULL && lNewSize > m_lMaxSize
    {
      // grow array
      ULONG lNewMaxSize = MIN_ARRAY_SIZE;
      while (lNewMaxSize < lNewSize)
        {
          lNewMaxSize = lNewMaxSize * 2;
        }

      SM_ASSERT_BREAK(lNewMaxSize >= m_lMaxSize);  // no wrap around

      // only okay to use smos_Calloc on base (double, ptr,.. ) static class (SmVector3d, SmExtent1d,.. ) objects.
      TYPE* pNewData = (TYPE*) smos_Calloc(1, lNewMaxSize * sizeof(TYPE));
      //        void** pNewData = (void**) new unsigned char[nNewMax * sizeof(void*)];

      if ( pNewData == NULL )
        { SM_ASSERT_BREAK(FALSE) ;
          ReSet();
          return SM_BIG_ULONG;
        }

      // copy new data from old.
      // only okay to use smos_MemCpy on base (double, ptrs, ULONG. . .) and static class (gw_CPOINT, SmVector3d. . .) objects.
      SE(smos_MemCpy((void*)pNewData, (void*)m_pData, m_lSize * sizeof(TYPE),  lNewMaxSize * sizeof(TYPE)));

      // construct remaining elements
      SM_ASSERT_BREAK(lNewSize > m_lSize);

      // GWC: why is this next line commented out, while the same line in the case above remains? Is this a bug? Is previous similar line a bug?

      // initialize the new elements with '0' characters
      // WARNING: when TYPE has virtual functions, this call corrupts the virtual function table
      // smos_MemSet(&pNewData[m_lSize], 0, (lNewSize-m_lSize) * sizeof(void*));

      // get rid of old stuff (note: no destructors called)
      if (!m_bIsBorrowed)
        { smos_Free(m_pData); m_pData = NULL ; }
      m_lSize       = lNewSize;
      m_lMaxSize    = lNewMaxSize;
      m_pData       = pNewData;
      m_bIsBorrowed = FALSE;
    }

  return lOldSize;

} // end SmTArray<TYPE>::SetSize

/*******************************************************************//**
PURPOSE: Set the element at the given index and optionally grow the
    array if the index is larger than the current size.

NOTES:
   Returns old size.
   If error, resets this and returns SM_BIG_ULONG.
***********************************************************************/
template<class TYPE>
inline ULONG SmTArray<TYPE>::SetAtGrow(ULONG lIndex, TYPE  sNewElement)
{
  SM_ASSERT_BREAK(lIndex < SM_BIG_ULONG && lIndex != SM_UNDEF_ULONG) ;
  ULONG lOldSize = m_lSize;
  if (lIndex >= m_lSize)
    {
      ULONG lErr = SetSize(lIndex+1);
      if ( lErr >= SM_BIG_ULONG )
        { return lErr; }
    }
  m_pData[lIndex] = sNewElement;

  return lOldSize;

} // end SmTArray<TYPE>::SetAtGrow

/*******************************************************************//**
PURPOSE: Reverse the orientation of a portion of an array.

NOTES: Here is the way to reverse the entire array:
      sMyArray.ReverseArray(0,sMyArray.GetSize());
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::ReverseArray(ULONG lBegin, ULONG lEnd)                 
{
  SM_ASSERT_BREAK(lBegin < GetSize());
  SM_ASSERT_BREAK(lEnd  <= GetSize());
  while (lBegin < lEnd)
    {
      TYPE tmp        = m_pData[lBegin];
      lEnd --;
      m_pData[lBegin] = m_pData[lEnd];
      m_pData[lEnd]   = tmp;
      lBegin++;
    }

} // end SmTArray<TYPE>::ReverseArray

/*******************************************************************//**
PURPOSE: Rotate the array in such a way as to make the given index first
   in the array.

NOTES:
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::RotateArray(ULONG lIndexToBeFirst)
{
  SM_ASSERT_BREAK(lIndexToBeFirst < GetSize());
  ReverseArray(0,GetSize());
  ReverseArray(0,GetSize()-lIndexToBeFirst);
  ReverseArray(GetSize()-lIndexToBeFirst,GetSize());

} // end SmTArray<TYPE>::RotateArray

/*******************************************************************//**
PURPOSE: Compress arrays omitting all zero values.  This is helpful
 for arrays of pointers to objects once some of the entries have been
 deleted and their pointer values in this array nave been set to NULL.

 Odds are there won't be a use for this function for arrays of things
 other than pointers to objects.

NOTES:
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::CompressZeros()
{
  ULONG ii = 0 ;
  ULONG i1 = 0 ;
  ULONG jj, jCnt = sizeof(TYPE) ;
  SmBoolean bZero ;
  char *pChar ;

  // for every current entry
  for(i1=0;i1<GetSize();i1++)
    {
      bZero = TRUE ;
      pChar = (char *)(m_pData + i1) ;

      // check for a zero value
      // We have to treat these like this because the values might not be pointers.
      for(jj=0;jj<jCnt;jj++)
        { if(pChar[jj] !=  0 )
            { bZero = FALSE ;
              break ;
            }
        }

      // compress non-zero values
      if ( !bZero )
        {
          if( ii != i1 )
            {
              // only okay to use smos_MemCpy on base (double, ptrs, ULONG. . .) and static class (gw_CPOINT, SmVector3d. . .) objects.
              SE(smos_MemCpy((void *)(m_pData+ii), (void *)(m_pData+i1), sizeof(TYPE), sizeof(TYPE)));
            }

          ii++ ;
        }
    } // end iter every current entry

  // reset the size of this array - without realloc or clearing of internal memory
  SetSize(ii) ;

} // end SmTArray<TYPE>::CompressZeros

/*******************************************************************//**
PURPOSE: Shell Sort elements of this array in Ascending Order

NOTES: 1. ShellSort calls the pointer to an input argument function
          called "LessThan" that must return TRUE when sElem1 < sElem2.

       2. ShellSort also takes an optional input argument void * pointer, pOptUserData.
          pOptUserData is not used in ShellSort but is passed to the
          LessThan method in case the LessThan method needs information to do its job.

       3. already implemented LessThan functions for doubles and SmPoint3ds called
            smgu_PointLess3dThan (SmPoint3d  sPoint1, SmPoint3d  sPoint2, void *pProjVec=NULL) ; // pProjVecType:[SmVector3d *]
            smgu_PointLess3dThan (SmPoint3d *pPoint1, SmPoint3d *pPoint2, void *pProjVec=NULL) ; // pProjVecType:[SmVector3d *]
            smgu_PointLess2dThan (SmPoint2d  sPoint1, SmPoint2d  sPoint2, void *pProjVec=NULL) ; // pProjVecType:[SmVector2d *]
            smgu_PointLess2dThan (SmPoint2d *pPoint1, SmPoint2d *pPoint2, void *pProjVec=NULL) ; // pProjVecType:[SmVector2d *]
            smgu_DoubleLessThan(double dVal1, double dVal2, void *pUnused=NULL) ;
            smgu_ULONGLessThan (ULONG  lVal1, ULONG  lVal2, void *pUnused=NULL) ;
            smgu_IntLessThan   (int    iVal1, int    iVal2, void *pUnused=NULL) ;
          are available for use and as examples

       4. Calls the empty constructor,   TYPE()
          Calls the assignment operator, TYPE::operator=

EXAMPLE 1: Sort SmPoint3d array to help look for coincident points
   {
     ULONG ii, jj ;
     SmVector3d sDir(1,1,1) ;       // pick a project line direction for the points
     SmTArray<SmPoint3d> sPoints ;  // get a set of points

     // order sPoints so that points that project to the same line point
     // are neighbors on the sPoints list. Runs in N*Log(N) time.
     sPoints.ShellSort(smgu_PointLessThan, &sDir) ;

     // check for possible coincidence in nearly linear time
     SmScaledZero dScaledZero   = SmTol::GetScaledZero(sPoints[0]) ;
     double       dScaledZeroSq = dScaledZero * dScaledZero ;
     for(ii=0;ii<sPoints.GetSize()-1;ii++) ;
       {
         double dProjS1 = sPoints[ii].Dot(sDir) ;
         for(jj=ii;jj<sPoints.GetSize();jj++)
           {
             double dProjS2 = sPoint[jj].Dot(sDir) ;
             if(!SM_IS_ZERO_TO_TOL(dProjS2 - dProjS1, dScaledZero))
               { break ; }
             double dDistSq = (sPoint[jj]-sPoint[ii]).LengthSquared() ;
             if(dDistSq < dScaledZeroSq) { // found coincident point }
           }
       }
   } // end EXAMPLE 1:

EXAMPLE 2: Sort a list of indices that have been associated with a double value
  {
    // locals
    class SmOrderedElem { public:
                          ULONG  m_lIndx  = SM_UNDEF_ULONG ;   
                          double m_dParam = SM_UNDEF_DOUBLE ; 
                          static SmBoolean LessThan(SmOrderedElem sElem1, SmOrderedElem sElem2, void *pUserData) 
                                                   { return sElem1.m_dParam < sElem2.m_dParam ; } 
                        } ; // end class SmOrderedElem
    SmTArray<SmOrderedElem> sOrderedIndices ;

    { ... code to build an unordered list in sOrderedIndices ... }

    // Order the indices
    sOrderedIndices.ShellSort(SmOrderedElem::LessThan, NULL) ;

  } // end EXAMPLE 2
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::ShellSort( SmBoolean( *LessThan )(     ///< [in ]: Function Ptr that returns TRUE when sElem1 < sElem2
                                       TYPE sElem1,                ///< [in ]: sElem1 of sElem1 < sElem2
                                       TYPE sElem2,                ///< [in ]: sElem2 of sElem1 < sElem2
                                       void *pOptUserData),        ///< [in ]: Optional Data needed for compare computation
                                       void *pOptUserData )        ///< [in ]: Value passed to every LessThan compare computation, default:[NULL]
{
  ULONG ii, flag = 1, d = m_lSize;
  TYPE sTemp = m_pData[0] ;
  while( flag || (d > 1))      // boolean flag (true when not equal to 0)
    {
      flag = 0;           // reset flag to 0 to check for future swaps
      d = (d+1) / 2;
      for (ii = 0; ii < (m_lSize - d); ii++)
        {
          if (LessThan(m_pData[ii + d], m_pData[ii], pOptUserData ))
            {
              // swap positions ii+d and ii
              sTemp           = m_pData[ii + d];
              m_pData[ii + d] = m_pData[ii];
              m_pData[ii]     = sTemp;

              // remember swap occurance
              flag = 1;
            }
        }
    }
  return;
} // end SmTArray<TYPE>::ShellSort

/*******************************************************************//**
PURPOSE: return Max Value in list using a linear search and
         given a callback method that can compare the value to two members

NOTES: when m_lSize == 0 return value is undefined.  It is whatever
       the compiler initializes a TYPE stack variable to.

EXAMPLE: Get Max double from array of doubles and of points
   {
     SmTArray<double>    sDoubles ;  // set of doubles
     SmTArray<SmPoint3d> sPoints ;   // set of 3d Points
     SmVector3d sDir(1,1,1) ;        // UserData
     . . .                        // load arrays with values

     double    dMaxDouble    = sDoubles.GetMaxValue(smgu_DoubleLessThan) ;
     double    dMaxDimension = sDoubles.GetMaxValue(smgu_FabsDoubleLessThan) ;
     SmPoint3d dMaxPoint     = sPoints.GetMaxValue(smgu_PointLessThan, &sDir) ;

   }
***********************************************************************/
template<class TYPE>
inline TYPE SmTArray<TYPE>::GetMaxValue( SmBoolean( *LessThan ) (  ///< [in ]: Function Ptr that returns TRUE when sElem1 < sElem2
                                         TYPE sElem1,              ///< [in ]: sElem1 of sElem1 < sElem2
                                         TYPE sElem2,              ///< [in ]: sElem2 of sElem1 < sElem2
                                         void *pOptUserData),      ///< [in ]: Optional Data needed for compare computation
                                         void *pOptUserData )      ///< [in ]: Value passed to every LessThan compare computation, default:[NULL]
{
  ULONG ii ;
  TYPE sMaxVal = m_pData[0];
  if(m_lSize > 0) { sMaxVal = m_pData[0] ; }
  for(ii=1;ii<m_lSize;ii++)
    {
      if(LessThan(sMaxVal, m_pData[ii], pOptUserData))
        { sMaxVal = m_pData[ii] ; }
    }

  // all done
  return(sMaxVal) ;

} // end SmTArray<TYPE>::GetMaxValue

/*******************************************************************//**
PURPOSE: return Min Value in list using a linear search and
         given a callback method that can compare the value to two members

NOTES: when m_lSize == 0 return value is undefined.  It is whatever
       the compiler initializes a TYPE stack variable to.

EXAMPLE: Get Min double from array of doubles and of points
   {
     SmTArray<double>    sDoubles ;  // set of doubles
     SmTArray<SmPoint3d> sPoints ;   // set of 3d Points
     SmVector3d sDir(1,1,1) ;        // UserData
     . . .                        // load arrays with values

     double    dMinDouble = sDoubles.GetMinValue(smgu_DoubleLessThan) ;
     double    dMinDimen  = sDoubles.GetMinValue(smgu_FabsDoubleLessThan) ;
     SmPoint3d dMinPoint  = sPoints.GetMinValue(smgu_PointLessThan, &sDir) ;

   }
***********************************************************************/
template<class TYPE>
inline TYPE SmTArray<TYPE>::GetMinValue( SmBoolean( *LessThan ) (    ///< [in ]: Function Ptr that returns TRUE when sElem1 < sElem2                           
                                         TYPE sElem1,                //      in : sElem1 of sElem1 < sElem2
                                         TYPE sElem2,                //      in : sElem2 of sElem1 < sElem2
                                         void *pOptUserData),        //      in : Optional Data needed for compare computation
                                         void * pOptUserData )       ///< [in ]: Value passed to every LessThan compare computation, default:[NULL]                  
{
  ULONG ii ;
  TYPE sMaxVal = m_pData[0];
  if(m_lSize > 0) { sMaxVal = m_pData[0] ; }
  for(ii=1;ii<m_lSize;ii++)
    {
      if(LessThan(m_pData[ii], sMaxVal, pOptUserData))
        { sMaxVal = m_pData[ii] ; }
    }

  // all done
  return(sMaxVal) ;

} // end SmTArray<TYPE>::GetMinValue

/*******************************************************************//**
PURPOSE: Removes all elements from the templated array and returns the memory
    used by the array.

NOTES:
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::RemoveAll()
{ SetSize(0); }

/*******************************************************************//**
PURPOSE: Set all of the array to given value

NOTES: SmTArrays can be arrays of complicated things like
SmTArray<SmSurfaceCache> which may not work for this method.
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::SetAll(const TYPE value)
{
  for (ULONG il = 0; il < GetSize(); il++)
    {
       SetAt(il, value);
    }
} // end SmTArray<TYPE>::SetAll

/*******************************************************************//**
PURPOSE: Set internal m_pData pointer to point to a borrowed array

NOTES:
  Frees current m_pData pointer if its not NULL and not borrowed.
  Won't free the input pPtrArray, even if this array gets resized.
    sets m_lSize       = lSize
         m_lMaxSize    = lMaxSize;
         m_pData       = pPtrArray ;
         m_bIsBorrowed = TRUE ;
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::SetArray
  (ULONG  lMaxSize,     ///< [in ]: new m_lMaxSize: Number of slots allocated for m_pData or Size of pPtrArray
   TYPE * pPtrArray,    ///< [in ]: optional preallocated array to use for storage
   ULONG  lSize)        ///< [in ]: new m_lSize: Number of slots used in m_pData
{
  SM_ASSERT_BREAK( lMaxSize < SM_BIG_ULONG    && lMaxSize    != SM_UNDEF_ULONG) ;
  SM_ASSERT_BREAK( lSize  < SM_BIG_ULONG && lSize  != SM_UNDEF_ULONG) ;
  if (lMaxSize == 0)
    {
      // shrink to nothing
      if (m_pData && ! m_bIsBorrowed) smos_Free(m_pData);
      m_lSize       = 0;
      m_lMaxSize    = 0;
      m_pData       = NULL;
      m_bIsBorrowed = FALSE;
    }
  else
    {
      // free existing data
      if(m_lMaxSize > 0 && m_pData && ! m_bIsBorrowed) { smos_Free(m_pData); m_pData = NULL ; }

      // save the data
      m_lSize       = lSize;
      m_lMaxSize    = lMaxSize;
      m_pData       = pPtrArray ;
      m_bIsBorrowed = TRUE ;
    }

} // end SmTArray<TYPE>::SetArray

/*******************************************************************//**
PURPOSE: Return in bytes all memory used and all memory allocated

NOTES:
  Frees current m_pData pointer if its not NULL and not borrowed.
  Won't free the input pPtrArray, even if this array gets resized.
***********************************************************************/
template<class TYPE>
inline ULONG SmTArray<TYPE>::GetMemoryUsed   // rtn: smaller size of actually used memory in bytes
  (ULONG    & rlMemoryAllocated)             // out: bigger size of all allocated memory in bytes
  const
{
  // set output
  rlMemoryAllocated =   sizeof(SmTArray<TYPE>)
                      + (m_bIsBorrowed ? 0 : m_lMaxSize * sizeof(TYPE)) ;

  // return used memory value
  return(  sizeof(SmTArray<TYPE>)
         + (m_bIsBorrowed ? 0 : m_lSize * sizeof(TYPE)) ) ;

} // end SmTArray<TYPE>::GetMemoryUsed

/*******************************************************************//**
PURPOSE: Return the element at a given index in the template array.

NOTES: The lIndex must be less then the current size (GetSize)
    or it will generate an assertion failure.
***********************************************************************/
template<class TYPE>
inline TYPE& SmTArray<TYPE>::GetAt
  (ULONG lIndex)
{
  SM_ASSERT_BREAK(lIndex < m_lSize);
  return m_pData[lIndex];

} // end SmTArray<TYPE>::GetAt

/*******************************************************************//**
PURPOSE: Return the element at a given index in the template array.

NOTES: The lIndex must be less then the current size (GetSize)
    or it will generate an assertion failure.
***********************************************************************/
template<class TYPE>
inline const TYPE& SmTArray<TYPE>::GetAt
  (ULONG lIndex)
 const
{
  SM_ASSERT_BREAK(lIndex < m_lSize);
  return m_pData[lIndex];

} // end SmTArray<TYPE>::GetAt

/*******************************************************************//**
PURPOSE: Set the element at the given index in the template array.

NOTES: The lIndex must be less then the current size (GetSize)
    or it will generate an assertion failure.
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::SetAt
 (ULONG lIndex,
  TYPE  sNewElement)
{ SM_ASSERT_BREAK(lIndex < m_lSize);
  m_pData[lIndex] = sNewElement;

} // end SmTArray<TYPE>::SetAt

/*******************************************************************//**
PURPOSE: Return the value of the last element in the template array.

NOTES: This method is useful for doing stack based operations.
                Do NOT call if TArray.GetSize() < 1  (bad error)
***********************************************************************/
template<class TYPE>
inline TYPE SmTArray<TYPE>::GetLast() const
{ SM_ASSERT_BREAK(m_lSize > 0);
  return (m_pData[m_lSize-1]);

} // end SmTArray<TYPE>::GetLast

/*******************************************************************//**
PURPOSE: Add a new element to the end of the templated array,
         return index of newly added element, and
         expand the internal memory size of the array as necessary.

NOTES:  this operation always causes the m_lSize to increase by one (GetSize).
***********************************************************************/
template<class TYPE>
inline ULONG SmTArray<TYPE>::Add   // rtn: index of Array slot used for sNewElement
  (TYPE sNewElement)               // in : the new element to add to the array
{
  ULONG lIndex = m_lSize;
  if (lIndex < m_lMaxSize)
    {
      m_pData[lIndex] = sNewElement;
      m_lSize++;
    }
  else
    {
      ULONG lErr = SetAtGrow(lIndex, sNewElement);
      if ( lErr >= SM_BIG_ULONG )
        { return SM_BIG_ULONG; }
    }
  return lIndex;

} // end SmTArray<TYPE>::Add

/*******************************************************************//**
PURPOSE: This method sets the size of the array (GetSize) to zero without
    freeing the memory of the data array.

NOTES:
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::ReSet()
{
  if (m_pData && m_lSize > 0)
    {
      // WARNING: when TYPE has virtual functions, this call corrupts the virtual function table
      smos_MemSet((void*)m_pData,0,m_lSize * sizeof(TYPE));
    }
  m_lSize = 0;

} // end SmTArray<TYPE>::ReSet

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

    It is recomended that only expert users utilize the pOptPtrArray.

    No longer: (Note - use this constructor only when the array is on the stack.)
***********************************************************************/
template<class TYPE>
inline SmTArray<TYPE>::SmTArray
 (ULONG  lMaxSize,       ///< [in ]: Number of slots allocated for m_pData
                         //         1. Desired initial allocation size for m_pArray
                         //      or 2. count of given pOptPtrArray
                         //      default:[0]
  TYPE * pOptPtrArray,   ///< [in ]: optional array ptr to use as the initial
                         //      m_pData array.  When array size exceeds
                         //      given lMaxSize, this array gets replaced
                         //      by a new heap array and is no longer used.
                         //      This array is never deleted by the SmTArray object.
                         //      Use this ptr to reduce the number of heap allocs.
                         //      default:[NULL]
  ULONG  lSize,          ///< [in ]: Number of slots used in m_pData,
                         //      must be less than equal to lMaxSize
                         //      default:[0]
  SmBoolean bCheckBeenThroughNew) ///< [in ]: bCheckBeenThroughNew for internal use only - only FALSE in SmThreadLocalStorage constructor
                                  //      otherwise always set to TRUE, default:[TRUE]
: SmObject(bCheckBeenThroughNew)  // eff: call SmObject constructor with option to skip BeenThroughNew calls
{
  // GWC:why is this here? - why limit this constructor to the stack?
  // cbi Removed.
  //m_cpContext = NULL; // We are created on the stack this way

  SM_ASSERT_BREAK(lMaxSize < SM_BIG_ULONG && lMaxSize != SM_UNDEF_ULONG);
  SM_ASSERT_BREAK(lSize  < SM_BIG_ULONG && lSize  != SM_UNDEF_ULONG);
  SM_ASSERT_BREAK(lSize <= lMaxSize);
  if (pOptPtrArray)
    {
      m_lSize       = lSize;
      m_lMaxSize    = lMaxSize;
      m_pData       = pOptPtrArray;
      m_bIsBorrowed = TRUE;
    }
  else
    {
      m_lSize       = 0;
      m_lMaxSize    = 0;
      m_pData       = NULL;
      m_bIsBorrowed = FALSE;
      if (lMaxSize > 0) { SetSize(lMaxSize); }
      m_lSize       = lSize;
    }

} // end SmTArray<TYPE>::SmTArray constructor

/*******************************************************************//**
PURPOSE: Heap based constructor for SmTArray


NOTES: Use this constructor when creating an array on the heap.
***********************************************************************/
template<class TYPE>
inline SmTArray<TYPE>::SmTArray(const SmContext & crContext)
{
    m_cpContext   = &crContext;
    m_lSize       = 0 ;
    m_lMaxSize    = 0 ;
    m_pData       = NULL;
    m_bIsBorrowed = FALSE;

} // end SmTArray<TYPE>::SmTArray constructor

/*******************************************************************//**
PURPOSE: Remove the last element of the array and decrease the size
    by one.

NOTES: This method is often used in conjunction with GetLast to
    perform stack based operations.
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::RemoveLast()
{ SM_ASSERT_BREAK(m_lSize > 0);
  if(m_lSize > 0)
    { m_lSize --; }
}

/*******************************************************************//**
PURPOSE: Set the m_lMaxSize array size of the template array.
         when m_lMaxSize grows   = m_lSize stays the same
         when m_lMaxSize shrinks = m_lSize shrinks.

NOTES:
   Returns old data size.
   If error, resets this and returns SM_BIG_ULONG.
   set m_lMaxSize    = lNewDataSize
       m_lSize       = Min(lNewDataSize, m_lSize) ;  // in case NewDataSize shrinks
       m_bIsBorrowed = FALSE

***********************************************************************/
template<class TYPE>
inline ULONG SmTArray<TYPE>::SetDataSize
  (ULONG lNewDataSize)
{
  SM_ASSERT_BREAK( lNewDataSize < SM_BIG_ULONG && lNewDataSize != SM_UNDEF_ULONG) ;

  if (lNewDataSize == m_lMaxSize) 
    { return m_lMaxSize; }

  ULONG lOldDataSize = m_lMaxSize;

  if (lNewDataSize < m_lMaxSize)
    {
      // only okay to use smos_Calloc on base (double, ptr,.. ) static class (SmVector3d, SmExtent1d,.. ) objects.
      TYPE* pNewData = (TYPE*) smos_Calloc(1, lNewDataSize * sizeof(TYPE));

      if ( pNewData == NULL )
        { SM_ASSERT_BREAK(FALSE) ;
          ReSet();
          return SM_BIG_ULONG;
        }

      // copy new data from old
      m_lSize = smos_Min(m_lSize,lNewDataSize);
      // only okay to use smos_MemCpy on base (double, ptrs, ULONG. . .) and static class (gw_CPOINT, SmVector3d. . .) objects.
      SE(smos_MemCpy(pNewData, m_pData, m_lSize * sizeof(TYPE),lNewDataSize * sizeof(TYPE)));

      if (!m_bIsBorrowed) { smos_Free(m_pData); m_pData = NULL ; }
      m_lMaxSize    = lNewDataSize;
      m_pData       = pNewData;
      m_bIsBorrowed = FALSE;
    }
  else if (lNewDataSize > m_lMaxSize)
    {
      // only okay to use smos_Calloc on base (double, ptr,.. ) static class (SmVector3d, SmExtent1d,.. ) objects.
      TYPE* pNewData = (TYPE*) smos_Calloc(1, lNewDataSize * sizeof(TYPE));

      if ( pNewData == NULL )
        { ReSet(); return SM_BIG_ULONG; }

      // copy new data from old
      if (m_pData)
        {
          // only okay to use smos_MemCpy on base (double, ptrs, ULONG. . .) and static class (gw_CPOINT, SmVector3d. . .) objects.
          SE(smos_MemCpy(pNewData, m_pData, m_lSize * sizeof(TYPE), lNewDataSize * sizeof(TYPE)));
          if (!m_bIsBorrowed) { smos_Free(m_pData); m_pData = NULL ; }
        }
      m_lMaxSize    = lNewDataSize;
      m_pData       = pNewData;
      m_bIsBorrowed = FALSE;
    }
  return lOldDataSize;

} // end SmTArray<TYPE>::SetDataSize

/*******************************************************************//**
PURPOSE: Copy the contents of another template array to the end
    of this array.

NOTES: The types of both template arrays should be the same.
***********************************************************************/
template<class TYPE>
inline ULONG SmTArray<TYPE>::Append(const SmTArray<TYPE>& rSource)
{
  SM_ASSERT_BREAK(this != &rSource);   // cannot append to itself

  ULONG nOldSize = m_lSize;
  if (rSource.m_lSize > 0)
    {
      ULONG lErr = SetSize(m_lSize + rSource.m_lSize);
      if ( lErr >= SM_BIG_ULONG )
        { return SM_BIG_ULONG; }

      // only okay to use smos_MemCpy on base (double, ptrs, ULONG. . .) and static class (gw_CPOINT, SmVector3d. . .) objects.
        SE(smos_MemCpy(m_pData + nOldSize, rSource.m_pData, rSource.m_lSize * sizeof(TYPE), m_lSize * sizeof(TYPE)));
    }
  return nOldSize;

} // end SmTArray<TYPE>::Append

/*******************************************************************//**
PURPOSE: Copy the unique members of another template array to the end
    of this array.

NOTES: The types of both template arrays should be the same.
***********************************************************************/
template<class TYPE>
inline ULONG SmTArray<TYPE>::AppendUnique(const SmTArray<TYPE> & rSource)
{
  SM_ASSERT_BREAK(this != &rSource);   // cannot append to itself

  ULONG ii, nOldSize = m_lSize;
  for(ii=0;ii<rSource.m_lSize;ii++)
    {
      AddUnique(rSource.m_pData[ii]) ;
    }

  return nOldSize;

} // end SmTArray<TYPE>::AppendUnique

/*******************************************************************//**
PURPOSE: Copy the contents of a template array into this template
   array.

NOTES:
   Returns old size.
   If error, resets this and returns SM_BIG_ULONG.
***********************************************************************/
template<class TYPE>
inline ULONG SmTArray<TYPE>::Copy
  (const SmTArray<TYPE>& rSource)
{
  SM_ASSERT_BREAK(this != &rSource);   // cannot append to itself

  ULONG lOldSize = SetSize(rSource.m_lSize);
  if ( lOldSize >= SM_BIG_ULONG )
    { return SM_BIG_ULONG; }

  if (rSource.m_lSize > 0)
    {
      // only okay to use smos_MemCpy on base (double, ptrs, ULONG. . .) and static class (gw_CPOINT, SmVector3d. . .) objects.
      SE(smos_MemCpy(m_pData, rSource.m_pData, rSource.m_lSize * sizeof(TYPE),rSource.m_lSize * sizeof(TYPE)));
    }
  return lOldSize;

} // end SmTArray<TYPE>::Copy

/////////////////////////////////////////////////////////////////////////////


/*******************************************************************//**
PURPOSE: Insert an element nCount times into a template array
            at a given index.

NOTES: 1. Elements from index to last are moved up the array by nCount.
       2. Returns old size.
       3. If error, resets this and returns SM_BIG_ULONG.
***********************************************************************/
template<class TYPE>
inline ULONG SmTArray<TYPE>::InsertAt
  (ULONG lIndex,        // in : Index of 1st value insertion
   TYPE  sNewElement,   // in : New element to insert 1 or more times
   ULONG nCount)        // in : opt Number of times to insert new element, default:[1]
{
  SM_ASSERT_BREAK( lIndex < SM_BIG_ULONG && lIndex != SM_UNDEF_ULONG) ;
  SM_ASSERT_BREAK( nCount < SM_BIG_ULONG && nCount != SM_UNDEF_ULONG) ;

  ULONG lOldSize = 0;
  if (lIndex >= m_lSize)
    {
      // adding after the end of the array
      lOldSize = SetSize(lIndex + nCount);  // grow so lIndex is valid
      if ( lOldSize >= SM_BIG_ULONG )
        { return SM_BIG_ULONG; }
    }
  else
    {
      // Inserting in the middle of the array.
      // Grow it to new size.
      // SetSize returns old size (or error).
      lOldSize = SetSize(m_lSize + nCount);

      if ( lOldSize >= SM_BIG_ULONG )
        { return SM_BIG_ULONG; }

      // shift old data up to fill gap
      // only okay to use smos_MemMove on base (double, ptr, ULONG,..) and static class (SmVector3d, SmSolution,..) objects.
      smos_MemMove(&m_pData[lIndex+nCount], &m_pData[lIndex], (lOldSize-lIndex) * sizeof(TYPE));

      // re-init slots we copied from
      // WARNING: when TYPE has virtual functions, this call corrupts the virtual function table
      smos_MemSet(&m_pData[lIndex], 0, nCount * sizeof(TYPE));

    }

  // insert new value in the gap
  SM_ASSERT_BREAK(lIndex + nCount <= m_lSize);
  while (nCount--)
    {
      m_pData[lIndex++] = sNewElement;
    }
  return lOldSize;

} // end SmTArray<TYPE>::InsertAt

/*******************************************************************//**
PURPOSE: Remove Element and compress array

RETURNS:  TRUE  = element was in list and removed
          FALSE = element was not in list
NOTES: m_lSize decreased by 1 when Element is in Array
***********************************************************************/
template<class TYPE>
inline SmBoolean SmTArray<TYPE>::Remove
  (TYPE sTgtElement)   // in : Tgt element to remove
{
  ULONG lFound = 0 ;

  // when the TgtElement is in the array - Remove it
  if(FindElement(sTgtElement, lFound))
    { RemoveAt(lFound, 1) ; 
      return(TRUE) ;
    }
  return(FALSE) ;
} // end SmTArray<TYPE>::Remove

/*******************************************************************//**
PURPOSE: Remove nCount elements from the template array starting at
    the given lIndex and compress the array afterwards.

NOTES: m_lSize decreased by nCount
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::RemoveAt
  (ULONG lIndex,
   ULONG nCount)
{
  SM_ASSERT_BREAK(lIndex + nCount <= m_lSize);

  // just remove a range
  if (m_lSize > lIndex + nCount)
    {
      ULONG nMoveCount = m_lSize - (lIndex + nCount);
      for (ULONG i=0; i<nMoveCount; i++)
        {
          m_pData[i+lIndex] = m_pData[i+lIndex+nCount];
        }
// only okay to use smos_MemMove on base (double, ptr, ULONG,..) and static class (SmVector3d, SmSolution,..) objects.
//        smos_MemMove(&m_pData[lIndex], &m_pData[lIndex + nCount], nMoveCount * sizeof(TYPE));
    }

  m_lSize = (nCount <= m_lSize) ? m_lSize - nCount : 0 ;

} // end SmTArray<TYPE>::RemoveAt

/*******************************************************************//**
PURPOSE: Insert the elements of a template array into this template
    array starting at the given index.

NOTES:
***********************************************************************/
template<class TYPE>
inline ULONG SmTArray<TYPE>::InsertAt
  (ULONG           nStartIndex,  ///< [in ]: insertion index
   SmTArray<TYPE>* pNewArray)    ///< [in ]: items to add to this array, not modified
{
  SM_ASSERT_BREAK(nStartIndex < SM_BIG_ULONG && nStartIndex != SM_UNDEF_ULONG) ;
  SM_ASSERT_BREAK(pNewArray != NULL);

  ULONG lOldSize = m_lSize;

  if (pNewArray->GetSize() > 0)
    {
      ULONG lErr = InsertAt(nStartIndex, pNewArray->GetAt(0), pNewArray->GetSize());
      if ( lErr >= SM_BIG_ULONG ) { return SM_BIG_ULONG; }

      for (ULONG i = 0; i < pNewArray->GetSize(); i++)
        {
          SetAt(nStartIndex + i, pNewArray->GetAt(i));
        }
    }
  return lOldSize;

} // end SmTArray<TYPE>::InsertAt

/*******************************************************************//**
PURPOSE: Search the template array for first element of a given value.

NOTES: If the element is found set the nFoundIndex and return TRUE,
    otherwise return FALSE.
***********************************************************************/
template<class TYPE>
inline SmBoolean SmTArray<TYPE>::FindElement
 (TYPE    sObjToFind,     // in : TgtElement to find
  ULONG & lFoundIndex)    // out: when found: Index of TgtElement, else undefined
 const
{
  for (ULONG ii=0; ii<this->GetSize(); ii++)
    {
      TYPE sElem = this->GetAt(ii);
      if (sElem == sObjToFind)
        {
          lFoundIndex = ii;
          return TRUE;
        }
    }
  return FALSE;

} // end SmTArray<TYPE>::FindElement

/*******************************************************************//**
PURPOSE: Search the template array for all elements of a given value.

NOTES: If the element is found one or more times,
    set the rFoundIndices and return TRUE, otherwise return FALSE.
***********************************************************************/
template<class TYPE>
inline SmBoolean SmTArray<TYPE>::FindElements
  (TYPE              sObjToFind,      // in : tgt object
   SmTArray<ULONG> & rFoundIndices)   // out: index values for every time tgt object is in this array
  const
{
  rFoundIndices.ReSet() ;
  for (ULONG ii=0; ii<this->GetSize(); ii++)
    {
      TYPE pElem = this->GetAt(ii);
      if (pElem == sObjToFind)
        {
          rFoundIndices.Add(ii) ;
        }
    }
  return(rFoundIndices.GetSize() > 0) ;

} // end SmTArray<TYPE>::FindElements

/*******************************************************************//**
PURPOSE: Search a BitMask marked Array for first element of a given value.

NOTES: Method is for arrays that store uninitialed element values.
       Only those elements marked by the BitMask as initialized are
       checked.

       Using a ULONG as a BitMask limits the size of the Array to
       sizeof(ULONG) * 8 ;
***********************************************************************/
template<class TYPE>
inline SmBoolean SmTArray<TYPE>::FindElementInMaskedArray
 (TYPE    pObjToFind,   // in : Search Tgt
  ULONG   lBitMask,     // in : only search ith elem when (lBitMask & (1<ii)) == TRUE
  ULONG & nFoundIndex)  // out: Indx of first Marked elem equal to pObjToFind
  const 
{
  SM_ASSERT_BREAK(GetSize() <= (sizeof(ULONG) * 8)) ;  // Make sure it is the right size

  // for every element
  for(ULONG ii=0;ii<this->GetSize();ii++)
    {
      // when element is initialized and equal to pObjToFind
      if(   (lBitMask & (1<ii))
         && (pObjToFind == this->GetAt(ii)))
        {
          nFoundIndex = ii;
          return TRUE;
        }
    } // end iter every element

  // all done - no find
  nFoundIndex = SM_UNDEF_ULONG ; 
  return FALSE;

} // end SmTArray<TYPE>::FindElementInMaskedArray

/*******************************************************************//**
PURPOSE: Add an element to the array only if it is not already in the array 
    and return TRUE when element added or FALSE when element already exists.

NOTES:

RETURN --  TRUE  = object was added
           FALSE = object is already a part of the list and was not added
***********************************************************************/
template<class TYPE>
inline SmBoolean SmTArray<TYPE>::AddUnique
  (TYPE sNewElement)
{
  ULONG nFoundIndex;
  if (this->FindElement(sNewElement,nFoundIndex))
    { return FALSE ; }
  else
    { this->Add(sNewElement);
      return TRUE ;
    }

} // end SmTArray<TYPE>::AddUnique

/*******************************************************************//**
PURPOSE: Transpose the elements of a template array by treating
    the array as a matrix with the given number of rows.

NOTES:
   Returns size.
   If error, resets this and returns SM_BIG_ULONG.
***********************************************************************/
template<class TYPE>
inline ULONG SmTArray<TYPE>::Transpose
  (ULONG lNumRows)
{
  if ( lNumRows < 1 )
    { return 0; }

  SM_ASSERT_BREAK(lNumRows < SM_BIG_ULONG && lNumRows != SM_UNDEF_ULONG) ;
  SM_ASSERT_BREAK(GetSize() % lNumRows == 0);  // Make sure it is the right size

  ULONG lNumColumns = GetSize() / lNumRows;

  SmTArray<TYPE> sTemp;
  ULONG lSize = sTemp.Append(*this);
  if ( lSize >= SM_BIG_ULONG ) { return SM_BIG_ULONG; }

  // Note, Append() returns the old size of sTemp, which was 0.
  lSize = GetSize();

  ULONG ii, jj;
  for (ii=0; ii<lNumRows; ii++)
    {
      for (jj=0; jj<lNumColumns; jj++)
        {
          (*this)[ii*lNumColumns + jj] = sTemp[jj*lNumRows + ii];
        }
    }
  return lSize;

} // end SmTArray<TYPE>::Transpose

/*******************************************************************//**
PURPOSE: Find common elements of two arrays.

NOTES:
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::FindCommonElements
  (const SmTArray<TYPE> & crOther,
   SmTArray<TYPE>       & rResult)
  const
{
  // protect against rResult == this or crOther cases
  SmTArray<TYPE> sTmp ;
  SmTArray<TYPE> *pResultArray = (   &rResult == this || &rResult == &crOther) ? &sTmp : &rResult ;

  // build the result list
  pResultArray->ReSet();
  for (ULONG i=0; i<crOther.GetSize(); i++)
    {
      TYPE pElem = crOther[i];
      ULONG lIndex;
      if (FindElement(pElem,lIndex))
        {
          pResultArray->Add(pElem);
        }
    }
  // when needed - set output
  if(   &rResult == this
     || &rResult == &crOther)
    {
      rResult.ReSet() ;
      rResult.Append(*pResultArray ) ;
    }

} // end SmTArray<TYPE>::FindCommonElements

/*******************************************************************//**
PURPOSE: return TRUE when two arrays share any common elements

NOTES: Just a cheaper version of FindCommonElements
***********************************************************************/
template<class TYPE>
inline SmBoolean SmTArray<TYPE>::HasCommonElements
  (const SmTArray<TYPE> & crOther)
  const
{
  SmBoolean bRtn = FALSE ;

  // look for 1st common element
  for (ULONG i=0; i<crOther.GetSize(); i++)
    {
      TYPE pElem = crOther[i];
      ULONG lIndex;
      if (FindElement(pElem,lIndex))
        {
          bRtn = TRUE ;
          break ;
        }
    }

  // all done
  return(bRtn) ;

} // end SmTArray<TYPE>::HasCommonElements

/*******************************************************************//**
PURPOSE: set rResult = this - crOther

NOTES: Works in place when rResult = *this.
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::RemoveElements
 (const SmTArray<TYPE> & crOther,  // in : OtherArray of Result = This - Other
        SmTArray<TYPE> & rResult)  // out: list of elems in this and not in crOther
  const
{
  // protect against rResult == this or crOther cases
  SmTArray<TYPE> sTmp ;
  SmTArray<TYPE> *pResultArray = (   &rResult == this || &rResult == &crOther) ? &sTmp : &rResult ;
  
  // build the result list
  pResultArray->ReSet();
  ULONG ii, lIndex;
  for (ii=0; ii<m_lSize; ii++)
    {
      TYPE pElem = m_pData[ii] ;
      if( !crOther.FindElement(pElem,lIndex) )
        { pResultArray->Add(pElem); }
    }

  // when needed - copy temp array to output array
  if(   &rResult == this
     || &rResult == &crOther)
    {
      rResult = *pResultArray;
    }
} // end SmTArray<TYPE>::RemoveElements

/*******************************************************************//**
PURPOSE: set rResult = this - crOther where the crOther list are 
         the indices of objects to be removed.

NOTES: Works in place when rResult = *this.
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::RemoveIndices
 (const SmTArray<ULONG> & crIndicesToRm,  // in : List of indices (in order) to remove 
        SmTArray<TYPE>  & rResult)        // out: list of remaining elems in this 
  const
{
  // locals
  ULONG iTo, iFrom, iTgt ;
                 
  // for every FromArray element
  for(iFrom=0,iTo=0,iTgt=0; iFrom<m_lSize; iFrom++)
    {
      // Save Element check
      if(   iTgt >= crIndicesToRm.GetSize()     // all RemoveIndices have been processed - save the rest of elements
         || iFrom != crIndicesToRm[iTgt]) // When this is not an index to be removed
        { 
          // Element to Save
          rResult[iTo] = (*this)[iFrom] ; 
          iTo++ ;
        } // end Save Element branch
      else // Rm Element branch
        { 
          iTgt++ ; 
        } // end Rm Element branch

    } // end iter entire list removing all specified elements

  // adjust the size of rResult
  rResult.SetSize(iTo) ;

} // end SmTArray<TYPE>::RemoveIndices

/*******************************************************************//**
PURPOSE: Find elements in this list that are not in the crOther list.
            Place results in rResult.
NOTES: Same as RemoveElements().
   'this' may be passed as either argument.
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::FindUniqueElements
 (const SmTArray<TYPE> & crOther,  ///< [in ]:
        SmTArray<TYPE> & rResult)  // out: list of elems in this and not in crOther
  const
{
  // protect against rResult == this or crOther cases
  SmTArray<TYPE> sTmp ;
  SmTArray<TYPE> *pResultArray = (   &rResult == this || &rResult == &crOther) ? &sTmp : &rResult ;

  // build the result list
  pResultArray->ReSet();
  ULONG ii, lIndex;
  for (ii=0; ii<m_lSize; ii++)
    {
      TYPE pElem = m_pData[ii] ;
      if( !crOther.FindElement(pElem,lIndex) )
        { pResultArray->Add(pElem); }
    }
  // when needed - set output
  if(   &rResult == this
     || &rResult == &crOther)
    {
      rResult = *pResultArray;
    }

} // end SmTArray<TYPE>::FindUniqueElements

/*******************************************************************//**
PURPOSE: Make a list of all duplicate items in this array

NOTES:  Result = Every duplicate entry, listed just once
        Warning - This is a O(n**2) naive algorithm.
        On large arrays: Try to be clever and think of another way
          ex: see SmHealData::Cache_CoinVertices()
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::GetDuplicates
 (SmTArray<TYPE> & rResult)
  const
{
  // protect against rResult == this case
  SmTArray<TYPE> sTmp ;
  SmTArray<TYPE> *pResultArray = (   &rResult == this) ? &sTmp : &rResult ;

  // build the result list
  pResultArray->ReSet();
  ULONG i, j ;
  for(i=0; i<GetSize(); i++)
    {
      TYPE pElem = this->GetAt(i) ;
      for(j=i+1;j<GetSize();j++)
        {
          if(pElem == this->GetAt(j))
            {
              pResultArray->AddUnique(pElem) ;
              break ;
            }
        }
    }
  // when needed - set output
  if( &rResult == this)
    {
      rResult.ReSet() ;
      rResult.Append(*pResultArray) ;
    }
} // end SmTArray<TYPE>::GetDuplicates

/*******************************************************************//**
PURPOSE: Remove all dupliate entries in this array

NOTES:  returns TRUE when duplicates were found, else
  returns FALSe
***********************************************************************/
template<class TYPE>
inline SmBoolean SmTArray<TYPE>::RemoveDuplicates()
{
  // build the result list
  SmTArray<TYPE> sResult ;
  ULONG i, j ;
  for(i=0; i<GetSize(); i++)
    {
      TYPE pElem = this->GetAt(i) ;
      SmBoolean bUnique = TRUE ;
      for(j=i+1;j<GetSize();j++)
        {
          if(pElem == this->GetAt(j))
            {
              bUnique = FALSE ;
              break ;
            }
        }
      if(bUnique) sResult.Add(pElem) ;
    }

  // when duplicates were found - replace this with result list
  if(GetSize() != sResult.GetSize())
    {
      this->ReSet() ;
      this->Append(sResult) ;
      return(TRUE) ;
    }
  return(FALSE) ;
} // end SmTArray<TYPE>::RemoveDuplicates

/*******************************************************************//**
PURPOSE: return a random sample of this array.

RETURNS: SM_SUCCESS found lNumToGet points to load into output rSamples  
***********************************************************************/
template<class TYPE> 
inline SmStatus SmTArray<TYPE>::PickSomeElements
 (ULONG                  lNumToGet,  ///< [in ]: Number of samples to retrieve
  SmTArray<TYPE>       & rSamples )  // out: Array of samples
 const
{
  // locals
  ULONG ii, lIndx ;
  SmTArray<ULONG> lIndices;
  ULONG lSize    = (*this).GetSize() ;
  ULONG lSkipCnt = lSize - lNumToGet ;

  // Seed the random number generator.
  std::default_random_engine generator;
  std::uniform_int_distribution<ULONG> distribution(0, lSize);

  // cases to be a little quicker: Get all
  //                               Get more than half
  //                               Get less than half
  if(lNumToGet == lSize) 
    { // getting all indices
      lIndices.SetSize(lSize) ;
      for(ii=0;ii<lSize;ii++) lIndices[ii] = ii ; 
    } // end get all branch

  // else - skipping less than half of the array
  else if(lSkipCnt < (lSize+1)/2)
    { 
      // find elements to skip
      SmTArray<ULONG> sSkips;
      while(sSkips.GetSize() < lSkipCnt)
        {
          ULONG lNum = distribution(generator);
          sSkips.AddUnique( lNum ); // this could be much faster if this were An AddUniqueOrdered function - not yet implemented
        }

      // build sample index array
      for(ii=0;lIndices.GetSize()<lNumToGet;ii++)
        {
          if(FALSE == sSkips.FindElement(ii, lIndx))
            { lIndices.Add(ii) ; }
        }
    } // end skipping fewer samples than keeping branch

  else // keeping fewer than getting branch
    {  
      // get sample indices
      while(lIndices.GetSize() < lNumToGet)
        {
          ULONG lNum = distribution(generator);
          lIndices.AddUnique( lNum ) ; // this could be much faster if this were An AddUniqueOrdered function - not yet implemented
        }
    } // end getting fewer samples than skipping branch

  // build return
  ULONG lCount = lIndices.GetSize();
  if ( lCount != lNumToGet )
    { return SM_ERR; }

  for(ii=0;ii<lCount;ii++)
    {
      rSamples.Add( (*this)[ lIndices[ii] ] );
    }

  // all done
  return SM_SUCCESS;

} // end SmTArray<TYPE>::PickSomeElements

/*******************************************************************//**
PURPOSE: Push an element off the stack
      (allows SmTArray to be treated as a LIFO stack (along with Pop and Peak)).

NOTES:
***********************************************************************/
template<class TYPE>
inline ULONG SmTArray<TYPE>::Push
  (TYPE sNewElement)
{
  return Add(sNewElement);

} // end SmTArray<TYPE>::Push

/*******************************************************************//**
PURPOSE: Pop an element off the stack
     (allows SmTArray to be treated as a LIFO stack (along with Push and Peak)).

NOTES: rtn: TRUE=Stack popped elem, FALSE=empty Stack, no pop
***********************************************************************/
template<class TYPE>
inline SmBoolean SmTArray<TYPE>::Pop
  (TYPE & rLastElement)       // out: the last pushed element - now popped from the stack
{
  if (m_lSize == 0) return false;
  rLastElement = GetLast();
  RemoveLast();
  return true;

} // end SmTArray<TYPE>::Pop

/*******************************************************************//**
PURPOSE: Peak at the element on the top of stack without popping
     (allows SmTArray to be treated as a LIFO stack (along with Push and Pop)).

NOTES: rtn: TRUE=Stack has elems, FALSE=empty Stack, no peak
***********************************************************************/
template<class TYPE>
inline SmBoolean SmTArray<TYPE>::Peak
  (TYPE & rLastElement)       // out: the last pushed element - not popped from the stack
{
  if (m_lSize == 0) return false;
  rLastElement = GetLast();
  return true;

} // end SmTArray<TYPE>::Peak

/////////////////////////////////////////////////////////////////////////////
// Diagnostics

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
template<class TYPE>
inline SmBoolean SmTArray<TYPE>::IsKindOf( SM_TYPE t ) const
{
  return ((SmTArray_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));

} // end SmTArray<TYPE>::IsKindOf

/*******************************************************************//**
PURPOSE: Do not use - instead use SM_DUMP_TARRAY(sArray)

NOTES: Don't bother calling SmTArray<TYPE>::Dump() when
       TYPE is not a pointer.  The output (if any) won't make any sense.
  
       The follosming Dump() method is onmly good for arrays of pointers and
       it can only dump those pointer values.  It can't dump an array of 
       Dump reports for every object in the array.

       The TYPE in SmTArray<TYPE> is used for both derived objects and pointers to derived objects
       I don't know how to make a template class that can distinguish between the two cases
       making it impossible to write a SmTArray<TYPE> method that calls a
       TYPE.method() or a TYPE->method().  So I don't know how to write a SmTArray<TYPE>::Dump() method
       that in turn calls its contained object dump methods without creating a lot of new work
       in our old method classes.  We cpould have two SmTArray implementations one for 
       objects and other for pointers to objects.  We could add a class argument to the template
       class looking something like SmTArray<SmFace *, SmFace> for an array of ptrs to SmFace and
       SmTArray<SmPoint3d, SmPoint3d> for an array of 3d points. Knowing the class name one
       could then run around buidlding two new static methods for each class with the static
       methods each taking one arg.  In one method the arg would be reference to an object
       and in the other it would be a pointer to an object.  Fortunately those methods
       just need to be wrappers for the already implemented SmClass::Dump() methods and
       so can mostly be made by extending the SM_COMMON macros.  However I don't like
       any of these ideas because they cause too many changes.
    
       So, for now, I'll just make the dump work for arrays that store TYPES the same size
       as a void * pointer.  In which case dump will just list those entries as pointer values
       which will be a help during debugging.  Don't bother calling SmTArray<TYPE>::Dump() when
       TYPE is not a pointer.  The output (if any) won't make any sense.

       ONLY USE when TYPE is a pointer value -  maybe the macro SM_DUMP_TARRAY(sArray) might be better
***********************************************************************/
template<class TYPE>
inline void SmTArray<TYPE>::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
  SmBoolean bPtrSize = sizeof(TYPE) == sizeof(void*) ;

  // header
  smos_sprintf(sBuff,        _T("\nStart Dump SmTArray:[0x%p] size=%ld, alloc size=%ld  "), this, m_lSize, m_lMaxSize) ;
  smos_sprintf(sBuffForFile, _T("\nStart Dump SmTArray:[%s] size=%ld, alloc size=%ld  "), _T("NotNULL"), m_lSize, m_lMaxSize) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;

  // when TYPE is the same size as void* pointer
  if(bPtrSize)
    {
      // treat m_pData as an array of void* ptrs - this lets us display ptr values for debug without having
      // to solve the problem that sometimes TYPE is a ptr and sometimes it is an object.
      void ** pData = (void**)m_pData ; 

      // obsolete
      // pretty print pointer values
      //smTArray_WritePtrArray((void**)m_pData, m_lSize) ;

      // for every m_pdata entry
      for(ULONG ii=0;ii<m_lSize;ii++)
        {
          // gwc - what we want is to print objects rather than addresses of array elements
          // for now just print the m_pData entries as pointer values
          smos_sprintf(sBuff,        _T("\n  [%3ld] = 0x%p"), ii, pData[ii]) ;
          smos_sprintf(sBuffForFile, _T("\n  [%3ld] = %s"), ii, (pData[ii]) ? _T("NotNULL") : _T("NULL")) ;
          smos_WriteBuffer(sBuff, sBuffForFile) ;

          /* old and useless            */ // sm_ItemDump(sBuffer,ii,&(m_pData[ii]));
          /* old and useless equivalent */ // smos_sprintf( sBuffer,_T("\t[%3ld] = 0x%p\n"), ii, (void*)m_pData[ii]));
          /* new and problematic        */ // sm_ItemDump(sBuffer,ii, (TYPE)(m_pData[ii]));
          // smos_WriteBuffer( sBuffer );
        }
    } // end TYPE is same size as void* ptr branch
  else // else TYPE is not same size as void* ptr branch
    {
      smos_WriteBuffer(_T("\n  SmTArray::Dump() warning: Not dumping item data.  SmTArray<TYPE>::Dump only dumps item vals when TYPE is the size of a ptr - compiler problems")) ;
    } // end not an array of ptrs branch

  // footer
  smos_sprintf(sBuff,        _T("\nEnd Dump SmTArray:[0x%p] size=%ld, alloc size=%ld  "), this, m_lSize, m_lMaxSize) ;
  smos_sprintf(sBuffForFile, _T("\nEnd Dump SmTArray:[%s] size=%ld, alloc size=%ld  "), _T("NotNULL"), m_lSize, m_lMaxSize) ;
  smos_WriteBuffer(sBuff, sBuffForFile) ;

} // end SmTArray<TYPE>::Dump - Do not use - instead use SM_DUMP_TARRAY(sArray)

/*******************************************************************//**
PURPOSE: This is a stack based deletion object.  It will automatically
   delete an array of objects when it goes out of scope.

NOTES:
- {
- SmTArray<SmObject*> sObjs;
- SmObjsDelete<SmObject*> sCleanupObjs(&sObjs);
- sObjs.Add(new SmObject());
- ...
- } // Going out of scope deletes all objects which have pointers in sObjs
***********************************************************************/
template<class TYPE> class SmObjsDelete
{
protected:
  const SmTArray<TYPE> * m_cpArray;  // Delete() calls delete m_cpArray[ii] for all ii, and
                                     //                m_cpArray->ReSet().
                                     // does not call  delete m_cpArray

public:
  SmObjsDelete()                                { m_cpArray = NULL; }
  SmObjsDelete(const SmTArray<TYPE> * cpArray)  { m_cpArray = cpArray; }
  ~SmObjsDelete()                               { Delete() ; }
  void Clear()                                  { m_cpArray = NULL; }
  void SetArray(const SmTArray<TYPE> * cpArray) { m_cpArray = cpArray; }
  void Delete() { if (m_cpArray) { ULONG ii, lNum = m_cpArray->GetSize();
                                   for (ii=0; ii<lNum; ii++)
                                     { TYPE pObj = (*m_cpArray)[ii];
                                       if (pObj) { delete pObj ;
                                                   ((SmTArray<TYPE> * )m_cpArray)->SetAt(ii, NULL) ;
                                                 }
                                     }
                                   ((SmTArray<TYPE> * )m_cpArray)->ReSet() ;
                                 }
                }

} ; // end template class SmObjsDelete

/*******************************************************************//**
PURPOSE: This is a stack based deletion object.  It will automatically
   delete an array of arrays when it goes out of scope.

NOTES:
- {
- SmTArray<SmTArray<double>> sArrays;
- SmArraysDelete<SmTArray<double>> sCleanupObjs(&sArrays);
- sArrays.SetSize(20) ; // allocates twenty SmTArray<double> arrays
- ...
- } // Going out of scope deletes all the m_pData arrays in all the Array members
***********************************************************************/
template<class TYPE> class SmArraysDelete
{
protected:
  SmTArray<TYPE> * m_cpArray;

public:
  SmArraysDelete()                                { m_cpArray = NULL; }
  SmArraysDelete(SmTArray<TYPE> * cpArray)        { m_cpArray = cpArray; }
  ~SmArraysDelete() { if (m_cpArray)
                      { ULONG ii, lNum = m_cpArray->GetSize();
                        for (ii=0; ii<lNum; ii++)
                          { m_cpArray->GetDataArray()[ii].SetSize(0) ;
                          }
                      }
                    }
  void Clear()                                  { m_cpArray = NULL; }
  void SetArray(const SmTArray<TYPE> * cpArray) { m_cpArray = cpArray; }

} ; // end template class SmArraysDelete

/*******************************************************************//****
PURPOSE: Gather all predeclarations of SmTArray<type> in one place and
  run them one time to force the library to instantiate the classes
  without instantiating them twice.

NOTES:
*************************************************************************/

#ifndef SM_TARRAY_BIND_TEMPLATES_H
#define SM_TARRAY_BIND_TEMPLATES_H

SM_TARRAY_TEMPLATE_PREDECLARATION(SmObject*) ;

SM_TARRAY_TEMPLATE_PREDECLARATION( SmTArray< SmEdge* > );

class SmAssemblyInstance ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmAssemblyInstance*) ;

class SmAObject ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmAObject*) ;

class SmTopology ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmTopology*) ;

class SmBSplineSurface ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmBSplineSurface*) ;

class SmBSplineVolume ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmBSplineVolume*) ;

class SmBrep ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmBrep*) ;

class SmRegion ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmRegion*) ;

class SmShell ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmShell*) ;

class SmCompositeCurve ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmCompositeCurve*) ;

class SmEdge ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmEdge*) ;

class SmEdgeuse ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmEdgeuse*) ;

class SmFace ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmFace*) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(const SmFace*) ;

class SmLoop ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmLoop*) ;

class SmFilletCorner ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletCorner*) ;

class SmFilletVertex ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletVertex*) ;

class SmFilletEdgeuse ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletEdgeuse*) ;

class SmFilletEdge ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletEdge*) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(const SmFilletEdge*) ;

class SmFilletGeom ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletGeom*) ;

class SmFilletSolver ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletSolver*) ;

class SmFilletSurfaceGenerator ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletSurfaceGenerator*) ;

class SmPolyRegion ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmPolyRegion*) ;

class SmPolyShell ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmPolyShell*) ;

class SmPolyFace ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmPolyFace*) ;

class SmCPolyFace ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmCPolyFace*) ;

class SmPolyEdge ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmPolyEdge*) ;

class SmPolyLoop ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmPolyLoop*) ;

class SmPolyVertex ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmPolyVertex*) ;

class SmPolyBrep ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmPolyBrep*) ;

class SmTriangle ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmTriangle*) ;

class SmLightSource ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmLightSource*) ;

class SmAssembly ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmAssembly*) ;

class SmSurface ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmSurface*) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(const SmSurface*) ;

class SmVolume ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmVolume*) ;

class SmSurfaceCache ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmSurfaceCache*) ;

class SmTrimSrfCache ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmTrimSrfCache*) ;

class SmVertex ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmVertex*) ;

class SmVertexuse ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmVertexuse*) ;

class SmAttribute ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmAttribute*) ;

class SmBSplineCurve ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmBSplineCurve *) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(const SmBSplineCurve*) ;

class SmCurve ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmCurve *) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(const SmCurve *) ;

enum SmOrientType ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmOrientType) ;

class SmGap ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmGap *) ;

class SmAssertReport ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmAssertReport *) ;

class SmGfxVertexArray ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmGfxVertexArray *) ;

class SmTreeVertex ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmTreeVertex*) ;

class SmNewMarkAndLock ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmNewMarkAndLock*) ;

enum SmContinuityType ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmContinuityType) ;

class SmVertexDefinition;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmVertexDefinition*);

class SmEdgeDefinition;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmEdgeDefinition*);

class SmProtoVertex;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmProtoVertex*);

class SmProtoEdge;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmProtoEdge*);

class SmProtoFace;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmProtoFace*);

class SmFeature;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmFeature*);

class SmFeatureLoop;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmFeatureLoop*);


//enum SmFilletErrorType ;
//SM_TARRAY_TEMPLATE_PREDECLARATION(SmFilletErrorType) ;

SM_TARRAY_TEMPLATE_PREDECLARATION(ULONG) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(long) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(double) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(double*) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(char) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmVector2d) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmVector3d) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmExtent1d) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmPeriodicExtent1d) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmExtent2d) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(SmBoolean) ;
SM_TARRAY_TEMPLATE_PREDECLARATION(void*) ;

#endif // SM_TARRAY_BIND_TEMPLATES_H

// SmTArray<ULONG> as histograms: each array element counts the number of some objects
// histogram for gaps and edge lengths are supported here with two macros each to map
// the real number line into a set of simple intervals
//   Macro 1: SM_HISTOGRAM_NAME_BUCKETCNT  NumberOfBuckets
//   Macro 2: SM_HISTOGRAM_NAME_INDEX(dRealNumber)  expression(dRealNumber) rtns: index of (0 <= index <= NumberOfBuckets-1)

// Histogram for Gap sizes: 11 buckets, bucket[ii] = Log10(a) + 9 with extreme values mapped to end buckets
  // All Gap Histograms use the same bucket scheme
  // indx:[ 0] = No. of Gaps with GapLen:[less than or equal 1e-10] 
  // indx:[ 1] = No. of Gaps with GapLen:[from     1e-10 to     1e-8] 
  // indx:[ 2] = No. of Gaps with GapLen:[from     1e-8  to     1e-6] 
  // indx:[ 3] = No. of Gaps with GapLen:[from     1e-6  to     1e-5] 
  // indx:[ 4] = No. of Gaps with GapLen:[from     1e-5  to     1e-4] 
  // indx:[ 5] = No. of Gaps with GapLen:[from     .0001 to    0.001] 
  // indx:[ 6] = No. of Gaps with GapLen:[from     .001  to    0.01 ]     
  // indx:[ 7] = No. of Gaps with GapLen:[from     .01   to    0.1  ]     
  // indx:[ 8] = No. of Gaps with GapLen:[from     .1    to    1.0  ]     
  // indx:[ 9] = No. of Gaps with GapLen:[from    1.0    to   10.0  ]       
  // indx:[10] = No. of Gaps with GapLen:[greater than 10.0]              
  // indx = floor(log10(EdgeLength))+9 trimmed to [0 10]
#define SM_HISTOGRAM_GAPSIZE_BUCKETCNT 11
#define SM_HISTOGRAM_GAPSIZE_INDEX(a) (  ((a) > 10.0)  ? 10                                       \
                                       : ((a) > 1e-6)  ? ((ULONG)(smos_Floor(smos_Log10(a)) + 9)) \
                                       : ((a) > 1e-8)  ? 2                                        \
                                       : ((a) > 1e-10) ? 1 : 0)

// Histogram for Edge lengths: 11 buckets, bucket[ii] = Log10(a) + 6 with extreme values mapped to end buckets
  // EdgeLength histogram of all Edges in m_pBrep
  // indx:[ 0] = No. of Edges with EdgeLen:[less than or equal 1e-5] 
  // indx:[ 1] = No. of Edges with EdgeLen:[from      1e-5 to     1e-4] 
  // indx:[ 2] = No. of Edges with EdgeLen:[from     .0001 to     .001] 
  // indx:[ 3] = No. of Edges with EdgeLen:[from     .001  to     .01 ] 
  // indx:[ 4] = No. of Edges with EdgeLen:[from     .01   to     .1  ] 
  // indx:[ 5] = No. of Edges with EdgeLen:[from     .1    to    1.0  ] 
  // indx:[ 6] = No. of Edges with EdgeLen:[from    1.0    to   10.0  ]     
  // indx:[ 7] = No. of Edges with EdgeLen:[from   10.0    to  100.0  ]     
  // indx:[ 8] = No. of Edges with EdgeLen:[from  100.0    to 1000.0  ]     
  // indx:[ 9] = No. of Edges with EdgeLen:[from  1e+3     to 1e+4    ]       
  // indx:[10] = No. of Edges with EdgeLen:[greater than 1e+4]              
  // indx = floor(log10(EdgeLength))+6 trimmed to [0 10]
#define SM_HISTOGRAM_EDGELEN_BUCKETCNT 11
#define SM_HISTOGRAM_EDGELEN_INDEX(a) (  ((a) < 1e-5) ?  0 \
                                       : ((a) > 1e+4) ? 10 \
                                       : ((ULONG)(smos_Floor(smos_Log10(a))) + 6) )

#endif // __SMTARRAY_H__

