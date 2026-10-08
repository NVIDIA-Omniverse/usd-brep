// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmOwningTopology.h
* PURPOSE: Header file for SmOwningTopology class.
**********************************************************************/

#ifndef __SMOWNINGTOPOLOGY_H__
#define __SMOWNINGTOPOLOGY_H__

#include <SmTopology.h>

/*******************************************************************//**
PURPOSE: This object is the abstract superclass of all topology
   objects which own a list of other topology objects.  Note that
   all list operations are methods on this object.

NOTES: Note that all list operations are methods on this object.
***********************************************************************/
class SM_EXPORT SmOwningTopology : public SmTopology
{
protected:
    SmTopology * m_pList;       // Pointer to 1st member of a doubly linked list of subordinate SmTopology objects
    ULONG        m_lListSize;   // Number of SmTopology objects in the list

public:
    // constructor, destructor
    SmOwningTopology() : m_pList(NULL), m_lListSize(0) {}
    virtual ~SmOwningTopology() { m_pList = NULL ;  m_lListSize = 0 ; }

    // list building
    inline SmStatus  Remove     (SmTopology * pObjectToRemove);
    inline SmStatus  InsertAfter(SmTopology * pObjectToInsert,
                                 SmTopology * pObjectCurrentlyOnList);
    inline SmStatus  PreInsert  (SmTopology * pObjectToInsert);
    inline SmStatus  PostInsert (SmTopology * pObjectToInsert);
    inline SmStatus  UpdateList (SmTopology * pListStart); // recompute m_lListSize ;
    inline SmStatus  ConcatenateList(SmTopology* pHead, SmTopology* pTail);
    inline SmStatus  UpdateListSize() ; // recomputes m_pListSize
    inline SmBoolean IsInList   (const SmTopology * pObject, ULONG *pOptIndx=NULL) const ;

    // list access
    inline ULONG        GetSize() const                        { return m_lListSize ; }
    inline SmTopology * GetList() const                        { return m_pList ; }
    inline SmStatus     GetAll(SmTArray<SmTopology*> & rArray, // place the m_pList->Next circular list members into an array
                               ULONG                 * pOptAttributeId=NULL,
                               const SmTopology      * pOptTgtObject=NULL) 
                              const;

    // walk m_pList->Next circular list making sure all backptrs are set, fix m_lListSize if needed
    SmStatus FixBackPointers(SmTArray<SmTopology*> & rChanges, SmBoolean &bMadeChanges) ;

    // Set ListHead element. (uses: init Linklist member. Changes 1st LinkList member when pListHead is already a LinkList member)
    inline void         SetList(SmTopology * pListHead)        { m_pList = pListHead ; }

    SmStatus            ReverseList();  // Reverse the doubly-linked Next/Last list in m_pList.

    virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore
                                  SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                                                            //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                                                            //      default:[SM_LEVEL_0] 
                                  SmAssertWalking    eWalkTree=SM_WALK,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
                                  SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                                 const ;
    // obsolete
    // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
    
    inline virtual SmBoolean TypicalListOwnerUse() const;

    // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
    SM_COMMON(SmOwningTopology,SmTopology,SmOwningTopology_TYPE);

} ; // end class SmOwningTopology

/*******************************************************************//**
PURPOSE: Make this SmOwningTopology Object the Owner of the
            given doubly linked SmTopology ObjectList.

NOTES: 
  1. Set this m_pList = ListStart
  2. For every pListStart->pSibling
      Set pSibling->m_pListOwner = this
  3. Set m_lListSize

  WARNING - In SMLib some SmTopology Objects don't use the m_pListOwner
            pointer to point back to the the SmOwningTopology Object.
            So only set backPointers when current backPoint is same type
            as this object - In such cases don't call this function
            just set m_pList = value and call
***********************************************************************/
inline SmStatus SmOwningTopology::UpdateList
  (SmTopology * pListStart)       // in : new list start
{
    m_lListSize = 0;
    m_pList = pListStart;
    if (m_pList == NULL) { return SM_SUCCESS; }
    
    // Traversing the list in both directions reduces time spent pointer chasing
    // by fetching two bits at the same time.

    SmTopology* pElemForward  = m_pList;
    SmTopology* pElemBackward = m_pList->m_pLast;

    do
      {
        pElemForward->m_pListOwner  = this;
        pElemBackward->m_pListOwner = this;
        SM_ASSERT(pElemForward->m_pNext->m_pLast  == pElemForward);
        SM_ASSERT(pElemBackward->m_pNext->m_pLast == pElemBackward);
        pElemForward  = pElemForward->m_pNext;
        pElemBackward = pElemBackward->m_pLast;
        m_lListSize += 2;
      } while (pElemForward != pElemBackward->m_pNext && pElemForward->m_pLast != pElemBackward->m_pNext);

    // Correction for overcounting list elements.

    if (pElemForward == pElemBackward || pElemForward->m_pLast == pElemBackward->m_pNext)
      {
        --m_lListSize;
      }

    return SM_SUCCESS;

} // end SmOwningTopology::UpdateList

/*******************************************************************/ /**
 PURPOSE: Reassigns the SmTopology objects between pHead and pTail to
 'this', assuming that the remainder of the SmTopology objects in the doubly-linked
 list which contains pHead and pTail already belong to 'this'

 NOTES: This method speeds the joining of SmPolyLoops in MakeManifoldEdge. Usage
 requires guaranteeing the hypotheses regarding ownership externally, it
 is not validated in the method.
 ***********************************************************************/

inline SmStatus SmOwningTopology::ConcatenateList(SmTopology * pHead, SmTopology * pTail)
{
    m_pList = pHead;
    if (m_pList == NULL) { return SM_SUCCESS; }

    SmTopology* pElemForward = m_pList;
    SmTopology* pElemBackward = pTail;

    SM_ASSERT(pElemForward->m_pLast->m_pListOwner == this);
    SM_ASSERT(pElemBackward->m_pNext->m_pListOwner == this);

    do
    {
        pElemForward->m_pListOwner = this;
        pElemBackward->m_pListOwner = this;
        SM_ASSERT(pElemForward->m_pNext->m_pLast == pElemForward);
        SM_ASSERT(pElemBackward->m_pNext->m_pLast == pElemBackward);
        pElemForward = pElemForward->m_pNext;
        pElemBackward = pElemBackward->m_pLast;
        m_lListSize += 2;
    } while (pElemForward != pElemBackward->m_pNext && pElemForward->m_pLast != pElemBackward->m_pNext);

    // Correction for overcounting list elements.

    if (pElemForward == pElemBackward || pElemForward->m_pLast == pElemBackward->m_pNext)
    {
        --m_lListSize;
    }

     --m_lListSize;

    return SM_SUCCESS;

} // end SmOwningTopology::ConcatenateList

/*******************************************************************//**
PURPOSE: recompute m_lListSize by walking the TopologyList pointers.

NOTES: Updates m_lListSize
***********************************************************************/
inline SmStatus SmOwningTopology::UpdateListSize
  ()
{
  m_lListSize = 0;
  if (m_pList != NULL)
    {
      SmTopology *pElem = m_pList;
      SM_ASSERT(pElem->m_pNext != NULL) ;
      if (pElem->m_pNext != NULL)
        {
          do
          {
            if (pElem->m_pNext->m_pLast != pElem) // check back pointers
              { SER(SM_ERR); }
            pElem = pElem->m_pNext;
            m_lListSize ++;
          } while (pElem != m_pList);
        }
    }
  return SM_SUCCESS;

} // end SmOwningTopology::UpdateListSize

/*******************************************************************//**
PURPOSE: Insert an SmTopology object into the list after the given object.

NOTES: 
***********************************************************************/
inline SmStatus SmOwningTopology::InsertAfter
  (SmTopology * pObjectToInsert,
   SmTopology * pObjectCurrentlyOnList)
{
    SmStatus sRet = SM_SUCCESS;
    if (pObjectCurrentlyOnList == NULL || pObjectCurrentlyOnList->m_pNext == NULL ||
       pObjectToInsert == NULL)
      {
        SE(SM_ERR);
        sRet = SM_ERR;
      }
    else
      {
        pObjectToInsert->m_pNext          = pObjectCurrentlyOnList->m_pNext;
        pObjectToInsert->m_pNext->m_pLast = pObjectToInsert;
        pObjectToInsert->m_pLast          = pObjectCurrentlyOnList;
        pObjectCurrentlyOnList->m_pNext   = pObjectToInsert;
        pObjectToInsert->m_pListOwner     = pObjectCurrentlyOnList->m_pListOwner;
        m_lListSize ++;
      }
    return sRet;

} // end SmOwningTopology::InsertAfter

/*******************************************************************//**
PURPOSE: Insert an SmTopology object at the beginning of this
           SmOwningTopology object's doubly-linked topology-object list.

NOTES: 1. Insert pObjectToInsert into the doubly-linked list of SmTopology objects
       2. set the this<=>pObjectToInsert owner<=>owned relationship
       3. increment List count.
***********************************************************************/
inline SmStatus SmOwningTopology::PreInsert
  (SmTopology * pObjectToInsert)
{
  // Insert pObjectToInsert into doubly linked list of m_pList objects
  if (m_pList == NULL)
    {
      SM_ASSERT(m_lListSize==0);
      pObjectToInsert->m_pNext = pObjectToInsert;
      pObjectToInsert->m_pLast = pObjectToInsert;
    }
  else
    {
      SM_ASSERT(m_lListSize > 0);
      SM_ASSERT(m_pList->m_pLast != NULL);
      pObjectToInsert->m_pNext = m_pList;
      SmTopology *pLast        = m_pList->m_pLast;
      pObjectToInsert->m_pLast = pLast;
      pLast->m_pNext           = pObjectToInsert;
      m_pList->m_pLast         = pObjectToInsert;
    }

  // tell pObjectToInsert that it belongs to this
  pObjectToInsert->m_pListOwner = this;

  // this->m_pList can point to any list member - let it point to the most recently added Object
  m_pList = pObjectToInsert;

  // increment the list length count
  m_lListSize ++;

  // all done
  return SM_SUCCESS;

} // end SmOwningTopology::PreInsert

/*******************************************************************//**
PURPOSE: Insert an SmTopology object at the end of this
  SmOwningTopology object's list and set the m_pListOwner backpointer.

NOTES: 
***********************************************************************/
inline SmStatus SmOwningTopology::PostInsert
  (SmTopology * pObjectToInsert)
{
  PreInsert(pObjectToInsert);         // add pObjToInsert to front of circular list, increments m_lListSize
  m_pList = pObjectToInsert->m_pNext; // restore pObjToInsert->Next as the m_pList target making pObjToInsert last on the list
  return SM_SUCCESS;

} // end SmOwningTopology::PostInsert

/*******************************************************************//**
PURPOSE: Remove SmTopology object from its SmOwningTopology object's list.

NOTES: No Objects Deleted - just removed

METHOD: Remove object from LinkedList by clearing Ptr values.

  SmOwningTopology object change:
    if(ObjToRemove is First ListMember) { Update this->m_pList value }

  SmTopology Object changes:
    pObjectToRemove->m_pNext      = NULL
         pObjectToRemove->m_pLast      = NULL
         pObjectToRemove->m_pListOwner = NULL

  If needed SmTopology Object Sibling changes
    pObjectToRemove->m_pNext->m_pLast = pObjectToRemove->m_pLast
    pObjectToRemove->m_pLast->m_pNext = pObjectToRemove->m_pNext
***********************************************************************/
inline SmStatus SmOwningTopology::Remove
  (SmTopology * pObjectToRemove)   // in : target topology object to remove from
{
  // check state - Object must be in our list.
  if ( pObjectToRemove->m_pListOwner != this )
    { return SM_ERR ; }

  // check state - look for problem circular list and ListOwner Ptrs
  if (   pObjectToRemove->m_pNext      == NULL
      || pObjectToRemove->m_pLast      == NULL
      || pObjectToRemove->m_pListOwner == NULL)
    { return SM_ERR ; }

  // if List has just one item
  if (pObjectToRemove->m_pNext == pObjectToRemove)
    {
      SM_ASSERT(m_lListSize == 1);
      m_lListSize --;
      m_pList                       = NULL;
      pObjectToRemove->m_pListOwner = NULL;
      pObjectToRemove->m_pNext      = NULL;
      pObjectToRemove->m_pLast      = NULL;
    } // end one Item branch
  else // List with more than one item
    {
      SM_ASSERT(m_lListSize > 1);
      // Handle case where this is the list head
      if (m_pList == pObjectToRemove)
        {
          m_pList = pObjectToRemove->m_pNext;
        }

      // Now remove this from double linked list
      pObjectToRemove->m_pNext->m_pLast = pObjectToRemove->m_pLast;
      pObjectToRemove->m_pLast->m_pNext = pObjectToRemove->m_pNext;

      pObjectToRemove->m_pListOwner = NULL;
      pObjectToRemove->m_pNext      = NULL;
      pObjectToRemove->m_pLast      = NULL;
      m_lListSize --;
    } // end more than one Item branch

  // all done
  return SM_SUCCESS;

} // end SmOwningTopology::Remove

/*******************************************************************//**
PURPOSE: rtn TRUE when input TgtObject is in SmOwningTopology linked list 
         of owned objects
NOTES:   linear search
***********************************************************************/
inline SmBoolean SmOwningTopology::IsInList
  (const SmTopology * pObject,     // in : Target topology object
   ULONG            * pOptIndx)    // out: optional, when rtn == TRUE : Item Indx for pObject in m_pList
                                   //                     rtn == FALSE: length of m_pList when rtn == FALSE
 const
{
  // init output
  if(pOptIndx) { *pOptIndx = 0 ; }

  // for every member on the list
  for(SmTopology *pTgt=m_pList; pTgt;)
    {
      // look for this object
      if(pObject == pTgt) return(TRUE) ;

      // increment iterator - check for exit
      pTgt = pTgt->m_pNext ;
      if(pOptIndx) { *pOptIndx = *pOptIndx + 1 ; }
      if(pTgt == m_pList) break ;
    }

  // arrive here when pObject is not on the list
  return(FALSE) ;

} // end SmOwningTopology::IsInList

/*******************************************************************//**
PURPOSE: Get a list of all SmTopology objects owned by
  this SmOwningTopology object.

NOTES: rArray is ReSet prior to being loaded with m_pList elements
***********************************************************************/
inline SmStatus SmOwningTopology::GetAll
  (SmTArray<SmTopology*> & rArray,      // out: array of all elements
                                        //      under m_pList doublyLinked list
                                        //      of m_pNext and m_pLast pointers
   ULONG *pOptAttributeId,              // in : only include objects containing an attribute with this id
                                        //      NULL to ignore, default:[NULL]
   const SmTopology *pOptTarget)        // in : only include this object if its in the list
                                        //      NULL to ignore, default:[NULL]
 const
// eff: place m_pList linked list items into input rArray
{
  ULONG lCount = 0;
  rArray.SetSize(m_lListSize);
  rArray.ReSet();

  // for nonNULL m_pList pointers
  if (m_pList != NULL)
    {
      // add all m_pList->m_pNext elements to rArray
      SmTopology *pElem = m_pList;

      // keep walking circular List back to head element
      if(   pOptAttributeId == NULL
         && pOptTarget      == NULL)
        {
          // add each elem to rArray
          do{ 
              // add elem and step to Next object
              rArray.Add(pElem);
              pElem = pElem->m_pNext;

              // check m_lListSize
              lCount ++;
              if (lCount > m_lListSize)
                { SER(SM_ERR) ; }

            } while (pElem && (pElem != m_pList));

          SM_ASSERT(m_lListSize == rArray.GetSize());
        } // end no attribute filtering branch
      else if(pOptAttributeId != NULL)
        { // add each elem with desired attribute to rArray
          do{ 
              if(   pElem->FindAttribute(*pOptAttributeId)
                 && (pOptTarget == NULL || pOptTarget == pElem)) 
                { rArray.Add(pElem) ; }
              
              pElem = pElem->m_pNext;

            } while (pElem && (pElem != m_pList));

        } // end only attribute filtering branch
      else
        { // pOptAttributeId == NULL, pOptTarget != NULL
          // add each elem that matches the target
          do{ 
              if(pElem == pOptTarget)
                { rArray.Add(pElem) ; }
              
              pElem = pElem->m_pNext;

            } while (pElem && (pElem != m_pList));

        } // end only target filtering branch
    } // end m_pList != NULL check

  // all done
  return SM_SUCCESS;

} // end SmOwningTopology::GetAll

/******************************
PURPOSE: Returns true when the items in our m_pList
    point back to us with their SmTopology::m_pListOwner pointer.

USAGE NOTES -- Helper for AssertValid() calls.
*********************/
inline SmBoolean SmOwningTopology::TypicalListOwnerUse() const
  { return TRUE ; }  // True for most SmOwningTopology.

#endif // !__SMOWNINGTOPOLOGY_H__
