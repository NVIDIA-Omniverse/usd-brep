// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmBucketSort.h
* PURPOSE: Header file for SmBucketSort object.  
**********************************************************************/

#ifndef __SMBUCKETSORT_H__
#define __SMBUCKETSORT_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMEXTENT1D_H__
#include <SmExtent1d.h>
#endif

/*******************************************************************//**
PURPOSE: This object is templated container class which manages
    bucket sort.  A bucket sort is an array based sort that just 
    throws elements into a bucket based roughly on a value within
    an interval the buckets live in.  Right now the bucket is optimized
    for creation and query not editing.

NOTES: Values falling outside of the range of the bucket 
    interval will be put into the first or last bucket.

    The buckets cannot be allocated as SmTArray<SmTArray<TYPE>>
    because SmTArray<TYPE> is a dynamic class and building
    an SmTArray<dynamic type> causes memory errors when 
    the SmTArray methods call smos_MemSet which corrupt
    the member Virtual function pointer tables.  Instead
    the buckets are implemented as an array of pointers
    to SmTArray<TYPE> objects.

***********************************************************************/
template<class TYPE> class SmBucketSort : public SmObject
{
protected:
    // SmTArray<SmTArray<TYPE>*> * m_pBuckets;
    SmTArray<TYPE>           ** m_pBuckets ;                   // an array of pointers to SmTArray<TYPE> objects
    ULONG                       m_lBucketCount ;               // size of m_pBuckets
    SmExtent1d                  m_sSortValueRange;             // The low and high end values of the scalar quantity
                                                               // being used to sort the elements added to the buckets 
    ULONG                       m_lNumberOfElementsInBuckets;  // total number of elements in all buckets

public:
    // Constructor
    SmBucketSort(ULONG              lBucketCount, 
                 const SmExtent1d & crSortValueRange) ;

    // Destructor
   ~SmBucketSort() ;

    // Add elements to the buckets
    void             AddElement         (TYPE     pNewElement, 
                                         double   dSortValue,
                                         ULONG  & rlBucketIndex) ;
    
    // simple data access
    SmExtent1d       GetSortValueRange  () const { return m_sSortValueRange ; }  // SortValue range for all buckets
    ULONG            GetBucketCount     () const { return m_lBucketCount ; }     // number of buckets
    ULONG            GetBucketIndex     (double dSortValue) const ;              // bucket index containing SortValue
    SmTArray<TYPE> * GetBucketArray     (ULONG lBucketIndex) const ;             // Get pointer to one bucket
    SmStatus         GetBucketRange     (ULONG        lBucketIndex,              // SortValue range for one bucket
                                         SmExtent1d & rBucketRange) const ;
    void             GetElementsInBucket(ULONG                  lBucketIndex,    // copy one bucket's elems into output array
                                         const SmTArray<TYPE> & rElements) const ;
public:
 
    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmBucketSort,SmObject,SmBucketSort_TYPE);

} ; // end class SmBucketSort

/*******************************************************************//**
PURPOSE: Get the array of a bucket.

NOTES: 
***********************************************************************/
template<class TYPE> 
inline SmTArray<TYPE>* SmBucketSort<TYPE>::GetBucketArray
  (ULONG lBucketIndex)  ///< [in] : target bucket
 const
{
  if (lBucketIndex >= m_lBucketCount) 
    {
      SE(SM_ERR);
      lBucketIndex = m_lBucketCount - 1;
    }
  return (m_pBuckets)[lBucketIndex];

} // end SmBucketSort<TYPE>::GetBucketArray

/*******************************************************************//**
PURPOSE: Get elements in a bucket and put them into the elements array.

NOTES: 
***********************************************************************/
template<class TYPE> 
inline void SmBucketSort<TYPE>::GetElementsInBucket
  (ULONG                  lBucketIndex,  ///< [in] : target bucket
   const SmTArray<TYPE> & rElements)     // out: copy of all elements in target bucket
 const
{
  rElements.ReSet();
  rElements.Append(GetBucketArray(lBucketIndex));

} // end SmBucketSort<TYPE>::GetElementsInBucket

/*******************************************************************//**
PURPOSE: Get the range of a particular bucket.

NOTES: Note that the range starts at the minimum value and
       contains all values less than the maximum value.
***********************************************************************/
template<class TYPE> 
inline SmStatus SmBucketSort<TYPE>::GetBucketRange
  (ULONG        lBucketIndex,      ///< [in] : target bucket
   SmExtent1d & rBucketRange)      // out: SortValue range for target bucket
 const
{
  double     dStartNormalized = (1.0*lBucketIndex)     / m_lBucketCount;
  double     dEndNormalized   = (1.0*(lBucketIndex+1)) / m_lBucketCount;
  SmExtent1d sIvl(0.0,1.0);

  dStartNormalized = sIvl.ClampValue(dStartNormalized);
  dEndNormalized   = sIvl.ClampValue(dEndNormalized);
  rBucketRange.SetMinMax(m_sSortValueRange.Evaluate(dStartNormalized),
                         m_sSortValueRange.Evaluate(dEndNormalized));
  return SM_SUCCESS;

} // end SmBucketSort<TYPE>::GetBucketRange

/*******************************************************************//**
PURPOSE: Return the bucket index of a given sort value.  Note that
    if the value is outside of the range for the bucket sort it will 
    return the first or last element.

NOTES: 
***********************************************************************/
template<class TYPE> 
inline ULONG SmBucketSort<TYPE>::GetBucketIndex
  (double dSortValue)       ///< [in] : target SortValue
 const
{ 
  double dVal        = m_sSortValueRange.ClampValue(dSortValue);
  double dPercent    =   (dVal - m_sSortValueRange.GetMin())
                       / m_sSortValueRange.GetLength();
  ULONG lBucket = dPercent * m_lBucketCount;
  if (lBucket >= m_pBuckets->GetSize()) 
    {
      lBucket = m_pBuckets->GetSize() - 1; // Just in case it falls 
                                           // right on the max value.
    }
  return lBucket;

} // end SmBucketSort<TYPE>::GetBucketIndex

/*******************************************************************//**
PURPOSE: Add an element to the bucket sort.

NOTES: 
***********************************************************************/
template<class TYPE> 
inline void SmBucketSort<TYPE>::AddElement
  (TYPE     pNewElement, 
   double   dSortValue,
   ULONG  & rlBucketIndex)
{
  ULONG            lBucketIndex = GetBucketIndex(dSortValue);
  SmTArray<TYPE> * pBucketArray = GetBucketArray(lBucketIndex);

  // when bucket has not yet been allocated
  if (pBucketArray == NULL) 
    {
      SmContext *pContext = GetContext();
      pBucketArray = new (*pContext) SmTArray<TYPE>;
      (m_pBuckets)[lBucketIndex] = pBucketArray;
    }

  // add the element to the bucket - unsorted
  pBucketArray->Add(pNewElement);
  rlBucketIndex = lBucketIndex;

} // end SmBucketSort<TYPE>::AddElement

/*******************************************************************//**
PURPOSE: Pretty Print

NOTES: 
***********************************************************************/
// template<class TYPE> 
// inline SmBoolean SmBucketSort<TYPE>::IsKindOf() const
// {
//   return ((SmBucketSort_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
// 
// } // end SmBucketSort<TYPE>::IsKindOf

/*******************************************************************//**
PURPOSE: Pretty Print

NOTES: 
***********************************************************************/
template<class TYPE> 
inline void SmBucketSort<TYPE>::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE];

  // header
  smos_sprintf(sBuff, _T("\nSmBucketSort: BucketCount [%ld]: SortValRange[%16.16lf, %16.16lf], Number of Elems in Buckets[%ld]"),
             m_lBucketCount,
             m_sSortValueRange.GetMin(),
             m_sSortValueRange.GetMax(),
             m_lNumberOfElementsInBuckets);
  smos_WriteBuffer(sBuff);

} // end SmBucketSort<TYPE>::Dump

/*******************************************************************//**
PURPOSE: constructor

NOTES: Allocate an array of pointers to SmTarray<TYPE> all init to NULL.
***********************************************************************/
template<class TYPE> 
inline SmBucketSort<TYPE>::SmBucketSort
  (ULONG              lBucketCount,        ///< [in] : number of buckets
   const SmExtent1d & crSortValueRange)    ///< [in] : SortValue range
 : m_lBucketCount(lBucketCount),
   m_sSortValueRange(crSortValueRange), 
   m_lNumberOfElementsInBuckets(0)
{ 
  m_pBuckets = new SmTArray<TYPE> * [lBucketCount] ;

  for (ULONG i=0; i<lBucketCount; i++) 
    { (m_pBuckets)[i] = NULL; }

} // end SmBucketSort<TYPE>::SmBucketSort

/*******************************************************************//**
PURPOSE: Destructor for the bucket array.  It deletes the buckets
  but not the contained elements.

NOTES: 
***********************************************************************/
template<class TYPE> 
inline SmBucketSort<TYPE>::~SmBucketSort()
{
  // delete the buckets
  for (ULONG i=0; i<m_lBucketCount; i++) 
    {
      SmTArray<TYPE> *pBucketArr = (m_pBuckets)[i];
      if (pBucketArr) { delete pBucketArr; pBucketArr = NULL ; }
    }

  // delete the bucket array
  SM_ASSERT(m_pBuckets != NULL) ; delete [] m_pBuckets ; m_pBuckets = NULL ;

} // end SmBucketSort<TYPE>::~SmBucketSort destructor

#endif // !__SMBUCKETSORT_H__


