// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTList.h
* PURPOSE: Header file for SmTList object.  
**********************************************************************/

/* Array Object definitions */
#ifndef __SMTLIST_H__
#define __SMTLIST_H__

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifdef _WIN32
#define SM_TLIST_TEMPLATE_PREDECLARATION(x)  template class SM_EXPORT SmTList<x>
#else
#define SM_TLIST_TEMPLATE_PREDECLARATION(x) 
#endif

/*******************************************************************//**
PURPOSE: This object is an abstract base class which elements need
    to subclass to produce objects which can go into the SmTList.

NOTES: static class object - don't add virtual methods to SmListNode.
***********************************************************************/
class SM_EXPORT SmListNode 
{
public:
  SmListNode * m_pNext;
  SmListNode * m_pPrev;

  SmListNode() : m_pNext(NULL), m_pPrev(NULL) { }
 ~SmListNode() { ReSet() ; }
  SmListNode *GetNext()   { return m_pNext ; }
  SmListNode *GetPrev()   { return m_pPrev ; }

  void         SetNext(SmListNode *pNext)  { m_pNext = pNext ; }
  void         SetPrev(SmListNode *pPrev)  { m_pPrev = pPrev ; }
  void         ReSet()                     { m_pNext = NULL ;
                                             m_pPrev = NULL ; 
                                           }

} ; // end class SmListNode

/*******************************************************************//**
PURPOSE: This object is templated container class which manages a linked list.
    Note: the linked list is doubly linked and circular.

NOTES: This object is highly optimized because it is heavily
    used.  
***********************************************************************/
template<class TYPE> class SmTList : public SmObject
{
  // Implementation
protected:
  TYPE * m_pListHead;  // Head of the linked list

#ifdef SM_DEBUG_CODE
  ULONG  m_lSize;      // number of elements on list
#endif

public:
  // Construction
  inline SmTList()   { m_pListHead = NULL; 
#ifdef SM_DEBUG_CODE
                       m_lSize     = 0 ;
#endif                       
                     }
  virtual ~SmTList() { }
  void Init()        { m_pListHead = NULL; 
#ifdef SM_DEBUG_CODE
                       m_lSize     = 0 ;
#endif                       
                     }

  // Accessing elements
  inline TYPE    * GetFirstNode() const;
  inline TYPE    * GetLastNode() const;
  inline TYPE    * GetNextNode(TYPE * pCurrentNode) const;
  inline TYPE    * GetPrevNode(TYPE * pCurrentNode) const;
  inline void      GetAllNodes(SmTArray<TYPE*> & rAllNodesInList) const;
  inline SmBoolean IsInList(TYPE *pElement) const;

  // Potentially growing the array
  inline void      Append (TYPE* pNewElement);       // add element to end of list
  inline void      Prepend(TYPE* pNewElement);       // add element to beg of list
  inline void      Remove (TYPE* pElementToRemove);  // rm  element from list. Don't call delete on removed element.
  inline void      InsertAfter(TYPE *pNewElement, 
                               TYPE *pElementToInsertAfter);
  inline void      RemoveLast(); // remove last element from list. Don't call delete on removed element.
  inline void      ReSet() ;     // calls delete on each list member - sets m_pListHead == NULL

  virtual void     Dump() const ;

} ; // end template class SmTList

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class TYPE> 
inline TYPE* SmTList<TYPE>::GetFirstNode() const
{ 
  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;
  return m_pListHead; 

} // end SmTList<TYPE>::GetFirstNode

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class TYPE> 
inline TYPE* SmTList<TYPE>::GetLastNode() const
{ 
  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;

  TYPE *pRet; 
  if (m_pListHead == NULL) {pRet = NULL;}
  else { pRet = (TYPE*)m_pListHead->m_pPrev; }
  return pRet;

} // end SmTList<TYPE>::GetLastNode

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class TYPE> 
inline TYPE* SmTList<TYPE>::GetNextNode(TYPE * pCurrentNode) const
{ 
  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;

  TYPE * pRet;
  if (pCurrentNode == NULL) 
    {
      SE(SM_ERR);
      pRet = NULL;
    }
  else { pRet = (TYPE*)pCurrentNode->m_pNext; }
  return pRet;

} // end SmTList<TYPE>::GetNextNode

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class TYPE> 
inline TYPE* SmTList<TYPE>::GetPrevNode(TYPE * pCurrentNode) const
{ 
  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;

  TYPE * pRet;
  if (pCurrentNode == NULL) 
    {
      SE(SM_ERR);
      pRet = NULL;
    }
  else { pRet = (TYPE*)pCurrentNode->m_pPrev; }
  return pRet;

} // end SmTList<TYPE>::GetPrevNode

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class TYPE> 
inline void SmTList<TYPE>::GetAllNodes(SmTArray<TYPE*> & rAllNodesInList) const
{
  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;

  rAllNodesInList.ReSet();
  TYPE *pNode = m_pListHead;
  while (pNode) 
    {
      rAllNodesInList.Add(pNode);
      pNode = (TYPE*)pNode->m_pNext;
      if (pNode == m_pListHead) pNode = NULL;
    }

} // end SmTList<TYPE>::GetAllNodes

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class TYPE> 
inline SmBoolean SmTList<TYPE>::IsInList(TYPE *pElement) const
{
  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;

  TYPE *pNode = m_pListHead;
  while (pNode) 
    {
      if(pNode == pElement) return(TRUE) ;
      pNode = (TYPE*)pNode->m_pNext;
      if (pNode == m_pListHead) pNode = NULL;
    }
  return(FALSE) ;

} // end SmTList<TYPE>::IsInList

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class TYPE> 
inline void SmTList<TYPE>::Append(TYPE* pNewElement)
{
  SM_ASSERT(pNewElement != NULL);
  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;

  if (m_pListHead == NULL) { m_pListHead          = pNewElement;
                             pNewElement->m_pNext = pNewElement;
                             pNewElement->m_pPrev = pNewElement;
                           }
  else                     { pNewElement->m_pPrev          = m_pListHead->m_pPrev;
                             m_pListHead->m_pPrev->m_pNext = pNewElement;
                             m_pListHead->m_pPrev          = pNewElement;
                             pNewElement->m_pNext          = m_pListHead;
                           }

  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;
#ifdef SM_DEBUG_CODE
  m_lSize++ ;
#endif                       

} // end SmTList<TYPE>::Append

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class TYPE> 
inline void SmTList<TYPE>::Prepend(TYPE* pNewElement)
{
  SM_ASSERT(pNewElement != NULL) ;
  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;

  if (m_pListHead == NULL) { m_pListHead          = pNewElement;
                             pNewElement->m_pNext = pNewElement;
                             pNewElement->m_pPrev = pNewElement;
                           }
  else                     { pNewElement->m_pPrev          = m_pListHead->m_pPrev;
                             m_pListHead->m_pPrev->m_pNext = pNewElement;
                             m_pListHead->m_pPrev          = pNewElement;
                             pNewElement->m_pNext          = m_pListHead;
                             m_pListHead                   = pNewElement;
                           }

  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;
#ifdef SM_DEBUG_CODE
  m_lSize++ ;
#endif                       

} // end SmTList<TYPE>::Prepend
 
/*******************************************************************//**
PURPOSE: Remove target element from list.  Don't call delete on removed element.

NOTES:
***********************************************************************/
template<class TYPE> 
inline void SmTList<TYPE>::Remove(TYPE* pElementToRemove) 
{
  SM_ASSERT(pElementToRemove != NULL) ;
  SM_ASSERT(m_pListHead != NULL) ;
  SM_ASSERT(IsInList(pElementToRemove)==TRUE) ;
  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;

  // do adjustment for case where remove element is list head
  if (m_pListHead == pElementToRemove) 
    {
      m_pListHead = (TYPE*)m_pListHead->m_pNext;
      if (m_pListHead == pElementToRemove) 
        {
          // Only one element on list
          m_pListHead = NULL;
        }
    }
  if (m_pListHead != NULL) 
    {
      pElementToRemove->m_pNext->m_pPrev = pElementToRemove->m_pPrev;
      pElementToRemove->m_pPrev->m_pNext = pElementToRemove->m_pNext;
    }
  pElementToRemove->m_pNext = NULL;
  pElementToRemove->m_pPrev = NULL;

  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;
#ifdef SM_DEBUG_CODE
  if(m_lSize > 0) m_lSize-- ;
#endif                       

} // end SmTList<TYPE>::Remove

/*******************************************************************//**
PURPOSE: Remove all elements from list. Call delete on every removed element.

NOTES:
***********************************************************************/
template<class TYPE> 
inline void SmTList<TYPE>::ReSet()
{
  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;

  // no work - no members
  if ( m_pListHead == NULL )
    { return ; }

  TYPE *pNodeNext=NULL, *pNode = m_pListHead;

  // delete all nodes
  for ( pNode = m_pListHead; pNode != NULL; pNode = pNodeNext )
    {
      // save next node
      pNodeNext = (TYPE*)pNode->m_pNext;

      // delete this node - prepare for next iter
      delete pNode; pNode = NULL;

      if ( pNodeNext == m_pListHead )
        {
          pNodeNext = NULL;  // m_pListHead has been deleted by now, stale pointer.
          break;
        }

    } // end iter every elem in TList

  m_pListHead = NULL;  // This was deleted in the first iteration.

#ifdef SM_DEBUG_CODE
  m_lSize = 0 ;
#endif

} // end SmTList<TYPE>::ReSet

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class TYPE> 
inline void SmTList<TYPE>::InsertAfter(TYPE *pNewElement, TYPE *pElementToInsertAfter)
{
  SM_ASSERT(pNewElement != NULL);
  SM_ASSERT(pElementToInsertAfter != NULL);
  SM_ASSERT(m_pListHead != NULL);
  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;

  pNewElement->m_pNext                    = pElementToInsertAfter->m_pNext;
  pElementToInsertAfter->m_pNext->m_pPrev = pNewElement;
  pNewElement->m_pPrev                    = pElementToInsertAfter;
  pElementToInsertAfter->m_pNext          = pNewElement;

  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;
#ifdef SM_DEBUG_CODE
  m_lSize++ ;
#endif                       

} // end SmTList<TYPE>::InsertAfter

/*******************************************************************//**
PURPOSE: Remove last element from list. Don't call delete on last element.

NOTES:
***********************************************************************/
template<class TYPE> 
inline void SmTList<TYPE>::RemoveLast()
{
  SM_ASSERT(m_pListHead != NULL);
  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;

  Remove(GetLastNode());

  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;
} // end SmTList<TYPE>::RemoveLast

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
template<class TYPE> 
inline void SmTList<TYPE>::Dump() const
{
  SM_ASSERT_BREAK(m_pListHead == NULL || (m_pListHead->m_pNext != NULL && m_pListHead->m_pPrev != NULL)) ;

  TCHAR sBuffer[SM_TBLOCK_SIZE] ;

  // get all nodes
  SmTArray<TYPE*> sAllNodesInList ;
  GetAllNodes(sAllNodesInList) ;

  // header
  //smos_sprintf(sBuffer, _T("\nSmTList : List Length : [%ld]\n  "), 
  //           sAllNodesInList.GetSize()); // note this does not work, use (below)
  smos_sprintf( sBuffer, _T("\nSmTList : List Length : [%ld]\n  "), sAllNodesInList.GetSize());
  smos_WriteBuffer( sBuffer ) ;

#ifdef SM_DEBUG_CODE
  smos_sprintf( sBuffer, _T("\n   Debug Check Length : [%ld] (should be the same as List Length)\n  "), m_lSize);
  smos_WriteBuffer(sBuffer) ;
#endif 
  
  // dump inherited data                    
  SmObject::Dump();
  
  // members
  for (ULONG i = 0; i < sAllNodesInList.GetSize(); i++) 
    {
      TYPE *pNode = sAllNodesInList[i];
      pNode->Dump() ;
    }

} // end SmTList<TYPE>::Dump


#endif // !__SMTLIST_H__


