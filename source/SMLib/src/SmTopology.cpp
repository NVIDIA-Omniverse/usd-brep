// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmTopology.cpp
* PURPOSE: Source file for SmTopology class methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmOwningTopology.h>
#include <SmAssertArray.h>
#include <SmContext.h>

/* Obsolete Style Note on use of Mark - see class SmNewMarkAndLock

Look at the function and you'll see that it simply sets a number from the
current global mark2 number.  It is basically used to mark things during
traversals so you will know what has been traversed.

pPolyBrep->NewMark(SM_MT_MARK2);  // Increments the Mark2 number

pPolyEdge->Mark(SM_MT_MARK2);  // Marks this edge

if (pPolyEdge->IsMarked(SM_MT_MARK2)) {  // Returns TRUE if the edge has been marked since
       // the previous call to NewMark(SM_MT_MARK2).
}

Basically what you want to do is to call NewMark2 before drawing.
Then after you draw an edge, Mark2 it and all of its radial edges
(SmPolyEdge::GetAllRadials).

*/

/*******************************************************************//**
PURPOSE: Increment the mark in the context by two

NOTES:
***********************************************************************/
// gwc:obsolete - replaced by SmNewMarkAndLock::NewMark
ULONG SmTopology::NewMark
    (SmMarkType eMarkType,        // in : oneof SM_MT_MARK, SM_MT_MARK2, SM_MT_MARK3, SM_MT_MARKIO, SM_MT_MARKASSERT
     SmBoolean  bIgnoreLock)      // in : TRUE  = increment a mark known to be locked (for callers that lock mark and then use it with intended increments)
                                  //      FALSE = output warning when incrementing locked mark (for backward compatible debugging of old unlockable marks)
                                  //      default:[FALSE]
   const
{
  // Get Context
  SmContext *pContext = SM_CONST_CAST(SmContext*,GetContext());

  // pass the call along
  return( pContext->NewMark(eMarkType, bIgnoreLock) ) ;

} // end SmTopology::NewMark

/*******************************************************************//**
PURPOSE: Mark this SmTopology object.

NOTES: When eMarkType == SM_MT_NOMARK, does nothing
***********************************************************************/
void SmTopology::Mark(SmMarkType eMarkType)
{

  if(eMarkType != SM_MT_NOMARK)
    {
      const SmContext* pContext = GetContext();
      if (pContext == NULL) {
          // This should not happen
          // Need warning or error
          return;
      }

      ULONG lMark = pContext->GetCurrentMark(eMarkType) ;
      switch(eMarkType)
        { case SM_MT_MARK       : m_lMark       = lMark ; break ;
          case SM_MT_MARK2      : m_lMark2      = lMark ; break ;
          case SM_MT_MARK3      : m_lMark3      = lMark ; break ;
          case SM_MT_MARKIO     : m_lMarkIO     = lMark ; break ;
          case SM_MT_MARKASSERT : m_lMarkAssert = lMark ; break ;
          default               : m_lMarkIO     = lMark ; break ;
        }
    }

} // end SmTopology::Mark

/*******************************************************************//**
PURPOSE: Remove mark from this SmTopology object.

NOTES: When eMarkType == SM_MT_NOMARK, does nothing
***********************************************************************/
void SmTopology::UnMark(SmMarkType eMarkType)
{

  if(eMarkType != SM_MT_NOMARK)
    {
      ULONG lMark = GetContext()->GetCurrentMark(eMarkType) ;
      switch(eMarkType)
        { case SM_MT_MARK       : if(m_lMark       == lMark && m_lMark       > 0) { m_lMark-- ; } break ;
          case SM_MT_MARK2      : if(m_lMark2      == lMark && m_lMark2      > 0) { m_lMark2-- ; } break ;
          case SM_MT_MARK3      : if(m_lMark3      == lMark && m_lMark3      > 0) { m_lMark3-- ; } break ;
          case SM_MT_MARKIO     : if(m_lMarkIO     == lMark && m_lMarkIO     > 0) { m_lMarkIO-- ; } break ;
          case SM_MT_MARKASSERT : if(m_lMarkAssert == lMark && m_lMarkAssert > 0) { m_lMarkAssert-- ; } break ;
          default               : if(m_lMarkIO     == lMark && m_lMarkIO     > 0) { m_lMarkIO-- ; } break ;
        }
    }

} // end SmTopology::UnMark

/*******************************************************************//**
PURPOSE: Get this topology's requested mark value

NOTES: when eMarkType == SM_MT_NOMARK, returns m_lMarkIO
       since something has to be returned.
***********************************************************************/
ULONG SmTopology::GetMark(SmMarkType eMarkType) const
{
  switch(eMarkType)
    { case SM_MT_MARK       : return (m_lMark) ;
      case SM_MT_MARK2      : return (m_lMark2) ;
      case SM_MT_MARK3      : return (m_lMark3) ;
      case SM_MT_MARKIO     : return (m_lMarkIO) ;
      case SM_MT_MARKASSERT : return (m_lMarkAssert) ;
      case SM_MT_NOMARK     : return (m_lMarkIO) ;
      default               : return (m_lMarkIO) ;
    }

} // end SmTopology::GetMark

/*******************************************************************//**
PURPOSE: Is this SmTopology object marked?

NOTES: when eMarkType == SM_MT_NOMARK always returns FALSE
***********************************************************************/
SmBoolean SmTopology::IsMarked(SmMarkType eMarkType) const
{
  return(GetMark(eMarkType) == (GetContext())->GetCurrentMark(eMarkType) ) ;

} // end SmTopology::IsMarked

/*******************************************************************//**
PURPOSE: Is this SmTopology object one that has been UnMarked since
         the last NewMark call?

NOTES: the following exmplifies the Mark states and use.
  {
    pTopology->NewMark(eMarkType) ;
    pTopology->IsMarked()   - returns FALSE
    pTopology->IsUnMarked() - returns FALSE

    pTopology->Mark(eMarkType) ;
    pTopology->IsMarked()   - returns TRUE
    pTopology->IsUnMarked() - returns FALSE

    pTopology->UnMark(eMarkType) ;
    pTopology->IsMarked()   - returns FALSE
    pTopology->IsUnMarked() - returns TRUE
  }
***********************************************************************/
SmBoolean SmTopology::IsUnMarked(SmMarkType eMarkType) const 
{ 
  return( GetMark(eMarkType) == ((GetContext())->GetCurrentMark(eMarkType) - 1) ) ;

} // end SmTopology::IsUnMarked

/*******************************************************************//**
PURPOSE: Return TRUE when specified MarkType is locked

NOTES:
***********************************************************************/
SmBoolean SmTopology::IsMarkLocked(SmMarkType eMarkType) const
{
  // Get Context
  SmContext *pContext = SM_CONST_CAST(SmContext*, GetContext());

  // pass the call along
  return( pContext->IsMarkLocked(eMarkType) ) ;

} // end SmTopology::IsMarkLocked

/*******************************************************************//**
PURPOSE: Notify attributed objects of changes occuring down at the
   lower level.

NOTES: The notify then takes the appropriate actions on the
   attributes.
***********************************************************************/
void SmTopology::Notify                // expected calls: caller->Notify(Event, pData1, pData2, pData2)
 (SmNotifyOperation  eNotifyOperation, //       event                | caller      |  pData1  | pData2                | pData3
  SmObject         * pData1,           //----------------------------+-------------+----------+-----------------------+--------------------------
  SmObject         * pData2,           // SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL
  SmObject         * pData3)           // SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2
                                       // SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep
                                       // SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                     
                                       // SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB
                                       // SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL
                                       // SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL
                                       // SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL
                                       // SM_NO_CONSTRUCTION         | NewObj      |  NewObj  | CopyFromObj or NULL   | NULL
                                       // SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL
                                       // SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL
                                       // SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL
                                       // SM_NO_SPLIT                | SplitGeomObj| Child1   | Child2                | SplitObj's Owner or NULL
                                       // SM_NO_MERGE                | MergeGeomObj| OrigObj1 | OrigObj2              | MergeObj's Owner or NULL
                                       // SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg
                                       // SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                    
{
  SmTopology *pThis   = NULL ;
  SmTopology *pOther1 = NULL ;
  SmTopology *pOther2 = NULL ;
  SmBoolean   bCopy   = FALSE ;
  SmBoolean   bMerge  = FALSE ;

  switch (eNotifyOperation)
    {
      case SM_NO_SPLIT_IN_BREP         : pThis   = (SmTopology*)pData1 ; // Orig Split obj
                                         pOther1 = (SmTopology*)pData2 ; // Orig obj as Child1
                                         pOther2 = (SmTopology*)pData3 ; // Child2
                                         break;
                                      
      case SM_NO_MERGE_IN_BREP         : // GWC:TODO try typo bug fix, pThis   = (SmTopology*)pData2
                                         pThis   = (SmTopology*)pData2 ; // SurvivingObj - usually surviving object
                                         //  pThis   = (SmTopology*)pData1 ; // SurvivingObj - usually surviving object
                                         pOther1 = (SmTopology*)pData1 ; // DeleteObj    - usually delete object
                                         pOther2 = (SmTopology*)pData2 ; // SurvivingObj - usually surviving object
                                         bMerge  = TRUE ;
                                         break ;
                                      
      case SM_NO_COPY :                  pThis   = (SmTopology*)this ;   // FromObject
                                         pOther1 = (SmTopology*)pData1 ; // ToObject
                                         pOther2 = NULL ;
                                         bCopy   = TRUE ;
                                         break ;
                                      
      case SM_NO_SPLIT:                  pThis   = (SmTopology*)this ;   // OrigObj as SplitObj
                                         pOther1 = (SmTopology*)pData1 ; // OrigObj reused as child1
                                         pOther2 = (SmTopology*)pData2 ; // new child2
                                         break ;
                                      
      case SM_NO_MERGE:                  // GWC:TODO try typo bug fix, pThis   = (SmTopology*)pData2
                                         //pThis   = (SmTopology*)pData2 ; // merge object - usually surviving object
                                         pThis   = this ;                // merge object - usually surviving object
                                         pOther1 = (SmTopology*)pData1 ; // origObj1     - usually surviving object
                                         pOther2 = (SmTopology*)pData2 ; // OrigObj2     - usually delete object
                                         bMerge  = TRUE ;
                                         break;
                                      
      case SM_NO_ADD_TO_BREP           : break ;
      case SM_NO_TRIM_NO_SPLIT_IN_BREP : break ;
      case SM_NO_RM_FROM_BREP          : break ;
      case SM_NO_COINCIDENT            : pThis   = (SmTopology*)pData2;
                                         pOther1 = (SmTopology*)pData1;
                                         break ;
      case SM_NO_CHANGE_GEOMETRY       : break ;
      case SM_NO_CHANGE_OWNER          : break ;
      case SM_NO_CONSTRUCTION          : if( pData2 != NULL )
                                           {
                                             pThis   = (SmTopology*)this ;   // FromObject
                                             pOther1 = (SmTopology*)pData2 ; // ToObject
                                             pOther2 = NULL ;
                                             bCopy   = TRUE ;
                                           }
                                         break ;
      case SM_NO_PRE_EDIT              : break ;
      case SM_NO_POST_EDIT             : break ;
      case SM_NO_DESTRUCTION           : break ;
      case SM_NO_REG_PROPAGATION       : break ;
      case SM_NO_UNKNOWN:                { SE_MSG(SM_ERR, _T("SmTopology::Notify - SM_NO_UNKNOWN event signalled")) ; }
                                         break ;
    } // end switch on eNotifyOperation

  // for merge
  if(pOther1 && pOther2 && bMerge)
    {
      pOther1->SetAllFlags(pOther1->GetAllFlags() | pOther2->GetAllFlags()) ;
      pOther2->SetAllFlags(pOther1->GetAllFlags() | pOther2->GetAllFlags()) ;
    }

  // propagate the values to pOther1
  if (pThis && pOther1 && pOther1 != pThis)
    {
      if(!bMerge) { pOther1->SetAllFlags(pThis->GetAllFlags()) ; }
      if(bCopy) { // GWC: why is this different than the other cases?
                  pOther1->m_lMark  = 0 ;  // init marks for next mark usage
                  pOther1->m_lMark2 = 0 ;  // don't copy marks - confuses new contexts
                  pOther1->m_lMark3 = 0 ;
                }
      else      { pOther1->m_lMark       = pThis->m_lMark ;
                  pOther1->m_lMark2      = pThis->m_lMark2 ;
                  pOther1->m_lMark3      = pThis->m_lMark3 ;
                }
#ifdef SM_INDEXING
      pOther1->m_lUserIndex1 = pThis->m_lUserIndex1;
      pOther1->m_lUserIndex2 = pThis->m_lUserIndex2;
      pOther1->m_pUserPtr1   = pThis->m_pUserPtr1;
#endif
    } // end pOther1 unique existence check

  // propagate the values to pOther2
  if (pThis && pOther2 && pOther2 != pThis)
    {
      if(!bMerge)
        { pOther2->SetAllFlags(pThis->GetAllFlags()) ; }

      pOther2->m_lMark       = pThis->m_lMark ;
      pOther2->m_lMark2      = pThis->m_lMark2 ;
      pOther2->m_lMark3      = pThis->m_lMark3 ;
#ifdef SM_INDEXING
      pOther2->m_lUserIndex1 = pThis->m_lUserIndex1 ;
      pOther2->m_lUserIndex2 = pThis->m_lUserIndex2 ;
      pOther2->m_pUserPtr1   = pThis->m_pUserPtr1 ;
#endif
    } // end pOther2 unique existence check

  // pass the call along to the base class
  SmAObject::Notify(eNotifyOperation,pData1,pData2,pData3);

} // end SmTopology::Notify

/*******************************************************************//**
PURPOSE: Destructor for topology.

NOTES: Will automatically clean
   up things which are in lists.
***********************************************************************/
SmTopology::~SmTopology()
{
  if (m_pListOwner) { m_pListOwner->Remove(this);
                      m_pListOwner = NULL ;
                    }

} // end SmTopology::~SmTopology destructor

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertTopology_list[] =
{
  /* 0 */ {SM_AT_POINTER, _T("Owner"),        _T("m_pListOwner is in list") },
  /* 1 */ {SM_AT_POINTER, _T("Context"),      _T("m_pListOwner shares the same context") },
  /* 2 */ {SM_AT_MARK,    _T("Mark"),         _T("m_lMark is less than context mark") },
  /* 3 */ {SM_AT_MARK,    _T("Mark2"),        _T("m_lMark2 is less than context mark2") },
  /* 4 */ {SM_AT_MARK,    _T("Mark3"),        _T("m_lMark3 is less than context mark3") },
  /* 5 */ {SM_AT_POINTER, _T("Equal Domain"), _T("m_pNext is non-NULL") },
  /* 6 */ {SM_AT_POINTER, _T("Equal Domain"), _T("m_pLast is non-NULL") },
  /* 7 */ {SM_AT_POINTER, _T("Linked List"),  _T("Circularly linked list corruption") },
  /* 8 */ {SM_AT_POINTER, _T("Counts"),       _T("Incorrect m_pList count") }
} ;

/*******************************************************************//**
PURPOSE: Check that the Owner points to a consistent set of
            doubly linked objects that includes this one

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmTopology::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                    //      default:[SM_LEVEL_0]
  SmAssertWalking    eWalkTree,     // NotUsed: in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  SM_REF1(eWalkTree) ;
  // init rtn value
  SmBoolean bOk;

  SmBoolean bRtn = TRUE ;

  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmAObject::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests)
           : TRUE ) ;

  // when this object is a member of an Owner's list
  if(m_pListOwner)
    {
      // validate owner's doubly linked list
      // make sure this object is on owner's list

      // remove next line - makes checks recursive
      //      bRtn = (m_pListOwner->AssertValid(pAList)  == TRUE) ? bRtn : FALSE;
      bOk = (m_pListOwner->IsInList(this)); SM_ASSERT(bOk);
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (bOk), _T("") ) ;

      // the owner and this object should have the same context
      const SmContext *pContext = GetContext();
      bOk = (m_pListOwner->GetContext() == pContext); SM_ASSERT(bOk);
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (bOk), _T("") ) ;

      // this objects Marks should be less than or equal to the context marks
      bOk = (m_lMark  <= pContext->GetCurrentMark(SM_MT_MARK) ); SM_ASSERT(bOk);
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (bOk), _T("") ) ;
      bOk = (m_lMark2 <= pContext->GetCurrentMark(SM_MT_MARK2)); SM_ASSERT(bOk);
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, (bOk), _T("") ) ;
      bOk = (m_lMark3 <= pContext->GetCurrentMark(SM_MT_MARK3)); SM_ASSERT(bOk);
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, (bOk), _T("") ) ;
    }
  else // has no owner
    {
      // should not be part of a peer list
      bOk = (m_pNext == NULL); SM_ASSERT(bOk);
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(5, SM_LEVEL_0, (bOk), _T("") ) ;
      bOk = (m_pLast == NULL); SM_ASSERT(bOk);
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(6, SM_LEVEL_0, (bOk), _T("") ) ;
    }

  // Check that the doubly-linked list is correct.
  // Look for loops in the list that don't include the first element.
  // Method: walk the list with two pointers, with one going only
  // half as fast as the other.  (Step every 2nd iteration.)
  // If there is a loop in the list, the faster pointer will
  // catch up with the slower, and they'll be equal at some point.
  //
  // We also count the list, then check against the owner's count.
  SmTopology *pCurr = this->m_pNext;
  SmTopology *pSlow = pCurr;
  ULONG lCount = 1;
  SmBoolean bMoveSlow = FALSE;

  while ( pCurr != this && pCurr != NULL ) // these ptrs can be Null.
    {
      lCount++;

      // Check next/last while we're at it.
      bOk &= pCurr->m_pNext->m_pLast == pCurr;
      bOk &= pCurr->m_pLast->m_pNext == pCurr;

      pCurr = pCurr->m_pNext;
      if ( bMoveSlow )
        { pSlow = pSlow->m_pNext; }
      bMoveSlow = !bMoveSlow;  // Toggle each time.

      if ( pSlow == pCurr )
        {
          bOk = FALSE; // Found a loop before getting back to start.
          break;
        }
    }

  bRtn &= SM_ASSERT_BOOLEAN_REPORT(7, SM_LEVEL_0, (bOk), _T("") ) ;

  // Also check that the owner's count is correct.
  bOk = TRUE;
  SmOwningTopology *pOwner = GetOwner();
  if ( pOwner != NULL  &&  pOwner->GetSize() != lCount )
    { bOk = FALSE; }
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(8, SM_LEVEL_0, (bOk), _T("") ) ;

  // all done
  // SM_ASSERT(bRtn);
  return(bRtn);

} // end SmTopology::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmTopology::AssertHeal
//  (SmAssertReport & rAReport,  // in : a report generated by AssertValid
//   SmAssertArray  * pAList)    // in : AssertArray holding rAReport
// {
//   SmBoolean bRtn = FALSE ;
// 
//   // check state - no work
//   if(rAReport.m_bOK == TRUE)
//     { return( TRUE ) ; }
// 
//   // check state - not the class that generated this report - pass call to parent class
//   if(rAReport.m_lReportingType != GetClassType())
//     {
//       // pass the call along to the parent - return ( Parent::AssertHeal(rAReport, pAList) ) ;
//       return ( SmAObject::AssertHeal(rAReport, pAList) ) ;
//     }
// 
//   // branch on the report type
//   switch(rAReport.m_lTestIndex)
//     {
//       case 99 : { // set case number appropriately - run fix code here
//                   // if fix works set rAReport.m_bOK = TRUE ;
//                 }
//                 break ;
// 
//       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;
//                rAReport.m_pHealMessage = _T("SmTopology::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmTopology::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE: Check our linked list for cycles.

NOTES: If a pointer chain has a cycle that does not contain 'this', it will
   cause an infinite loop in SmOwningTopology::GetAll(), and any other method
   that walks the pointer chain.  An example is two entities that point back
   to each other, while a third entity points to one of them.
***********************************************************************/
// Note: return True if bad, makes it easier to call.
SmBoolean SmTopology::CheckPointerChain()
{
  // If there are only one or two, we're ok.
  if ( m_pNext          == this ) { return FALSE; }
  if ( m_pNext->m_pNext == this ) { return FALSE; }

  // There are more than two entries.
  // Method: march the list with two pointers, where one goes twice as fast
  // as the other.  If there is a cycle, they will both point to the same
  // element at some point.

  const SmTopology * pStart = this;
  const SmTopology * ptr2 = pStart;
  const SmTopology * ptr1 = ptr2->m_pNext;
  // ptr1 is in the lead, and gets incremented twice in each loop.

  SmBoolean bRet = FALSE;

  while ( ptr1 != pStart )
    {
      if ( ptr2 == ptr1 ) {
        bRet = TRUE; // Error.
        break;
      }

    ptr1 = ptr1->m_pNext;

    if ( ptr1 == pStart )
      { break; } // done.

    if ( ptr2 == ptr1 )
      {
        bRet = TRUE; // Error.
        break;
      }

    ptr1 = ptr1->m_pNext;
    ptr2 = ptr2->m_pNext;

  } // end while

  return bRet;

} // end CheckPointerChain

/*******************************************************************//**
PURPOSE: This method should never be invoked.

NOTES: The default new for
   topology should be with a SmBrep.
***********************************************************************/
// #ifndef SM_BORLAND
//   void *SmTopology::operator new(size_t size) { SE(SM_ERR); return (void *)NULL; }
// #endif // no SM_BORLAND


/*******************************************************************//**
PURPOSE: This new operator optimizes memory allocation for the creation
   of topological objects within a Brep.

NOTES: Note that the optimization has not yet been implemented.
***********************************************************************/
void *SmTopology::operator new(size_t size, SmObject * pObjectToGetContext)
{
    SM_ASSERT(pObjectToGetContext!=NULL);
    const SmContext *pContext = pObjectToGetContext->GetContext();

    SmTopology *pRet = (SmTopology*)SmObject::operator new(size,*pContext);

    return (void*)pRet;

} // end *SmTopology::operator new

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmTopology::IsKindOf( SM_TYPE t ) const
{
  return ((SmTopology_TYPE == t) ? TRUE : SmAObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmTopology::Dump(void) const
{
  // locals
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // one line report
  SM_SPRINTF( sBuff,        _T("\nSmTopology object:[0x%p]"), this );
  SM_SPRINTF( sBuffForFile, _T("\nSmTopology object:[%s]"),   _T("NotNULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // owner
  SM_SPRINTF( sBuff,        _T(", ListOwner:[0x%p]"), m_pListOwner );
  SM_SPRINTF( sBuffForFile, _T(", ListOwner:[%s]"),   m_pListOwner ? _T("NotNULL") : _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // Next/Last
  SM_SPRINTF( sBuff,        _T(", Next/Last:[0x%p/0x%p]"), m_pNext, m_pLast );
  SM_SPRINTF( sBuffForFile, _T(", Next/Last:[%s/%s]"),     m_pNext ? _T("NotNULL") : _T("NULL"), m_pLast ? _T("NotNULL") : _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // Marks
  if(GetContext())
    { SM_SPRINTF( sBuff,        _T(", Marks:[M=%s, M2=%s, M3=%s, IO=%s, MA=%s]"), IsMarked(SM_MT_MARK) ? _T("Yes") : _T("No "),
                                                                                  IsMarked(SM_MT_MARK2) ? _T("Yes") : _T("No "),
                                                                                  IsMarked(SM_MT_MARK3) ? _T("Yes") : _T("No "),
                                                                                  IsMarked(SM_MT_MARKIO) ? _T("Yes") : _T("No "),
                                                                                  IsMarked(SM_MT_MARKASSERT) ? _T("Yes") : _T("No ") ) ;
    }
  else
    { SM_SPRINTF( sBuff,        _T(", Marks:[M=%5ld, M2=%5ld, M3=%5ld, IO=%5ld, MA=%5ld]"), m_lMark,
                                                                                            m_lMark2,
                                                                                            m_lMark3,
                                                                                            m_lMarkIO,
                                                                                            m_lMarkAssert ) ;

    }
  smos_WriteBuffer(sBuff);

  // flags
  SM_SPRINTF( sBuff,        _T(", Flags:[%lu]"), m_lFlags );
  smos_WriteBuffer(sBuff);

#ifdef SM_INDEXING
  SM_SPRINTF( sBuff,        _T(", User:[idx1=%ld, idx2=%ld, ptr=0x%p]"), m_lUserIndex1,
                                                                         m_lUserIndex2,
                                                                         m_pUserPtr1 );
  SM_SPRINTF( sBuffForFile,        _T(", User:[idx1=%ld, idx2=%ld, ptr=%s]"), m_lUserIndex1,
                                                                              m_lUserIndex2,
                                                                              m_pUserPtr1 ? _T("NotNULL") : _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);
#endif // SM_INDEXING

} // end SmTopology::Dump
