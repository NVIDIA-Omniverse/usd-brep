// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmOwningTopology.cpp
* PURPOSE: Source file for SmOwningTopology class methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmTopoTypes.h>
#include <SmOwningTopology.h>
#include <SmAssertArray.h>

/*******************************************************************//**
PURPOSE: walk m_pList->Next circular list making sure all backptrs are set,
         make sure the m_lListSize value is correct

NOTES:
RETURNS: SM_ERR when any element on the m_pList circular linked list
                has a NULL m_pNext pointer. List can't be fixed
         SM_SUCCESS when circular m_pList->m_pNext array was fine or fixed.
***********************************************************************/
inline SmStatus SmOwningTopology::FixBackPointers
  (SmTArray<SmTopology*> & rChanges,      // out: array appended with all elements that were changed
   SmBoolean             & rbMadeChanges) // out: TRUE = found bad backpointers that were fixed
                                          //      FALSE= circular array was ok

{
  // init output
  rbMadeChanges = FALSE ;

  // locals
  ULONG lCount = 0;
  SmBoolean bAdd = FALSE ;

  // for nonNULL m_pList pointers
  if (m_pList != NULL)
    {
      // walk all m_pList->m_pNext elements
      SmTopology *pElem = m_pList;

      // walk circular List back to head element - fix owner and last ptr values
      do{
          bAdd = FALSE ;

          // can't fix a list that has a NULL forward pointer
          if(pElem->m_pNext == NULL)
            { return SM_ERR ; }

          // check next Elem ListOwner ptrs
          if(pElem->m_pNext->m_pListOwner != this)
            {
              pElem->m_pNext->m_pListOwner = this ;
              bAdd = TRUE ;
            }

          // check next Elem back ptrs
          if(pElem->m_pNext->m_pLast != pElem)
            {
              pElem->m_pNext->m_pLast = pElem ;
              bAdd = TRUE ;
            }

          // when changes were made
          if(bAdd)
            {
              // add the Next Elem to the change list
              rChanges.Add(pElem->m_pNext) ;
              rbMadeChanges = TRUE ; 
            }

           // increment for next iter
           lCount++ ;
           pElem = pElem->m_pNext;

        } while (pElem && (pElem != m_pList));

    } // end m_pList != NULL check

  // check ListSize
  if(m_lListSize != lCount)
    {
      m_lListSize = lCount ;
      rbMadeChanges = TRUE ;
      rChanges.Add(this) ;
    }

  // all done
  return SM_SUCCESS;

} // end SmOwningTopology::FixBackPointers

/*******************************************************************//**
PURPOSE: Reverse the doubly-linked Next/Last list.

NOTES: 
***********************************************************************/
SmStatus SmOwningTopology::ReverseList()
{
  SmTopology *pStart = this->m_pList;
  SmTopology *pCurr  = pStart, *pTemp;
  do
  {
      pTemp = pCurr->m_pNext;
      pCurr->m_pNext = pCurr->m_pLast;
      pCurr->m_pLast = pTemp;

      // Proceed to next in list.
      pCurr = pTemp;  // pTemp was 'next'.

  } while ( pCurr != pStart );

  return SM_SUCCESS;

} // end ReverseList

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertOwningTopology_list[] =
{
  {SM_AT_POINTER, _T("Null"),         _T("pTgt linked list is Non-NULL") },
  {SM_AT_POINTER, _T("Back Pointer"), _T("pTgt back pointer is valid") },
  {SM_AT_POINTER, _T("Owner"),        _T("pTgt owner is valid") },
  {SM_AT_POINTER, _T("List Length"),  _T("pTgt->m_pNext == pStart") },
  {SM_AT_POINTER, _T("Context"),      _T("pTgt shares the same context") }
} ;

/*******************************************************************//**
PURPOSE: Check that the m_pList points to a consistent set of
            doubly linked objects and is length m_lListSize

RETURNS ---  TRUE  = OK
             FALSE = Problem
***********************************************************************/
SmBoolean SmOwningTopology::AssertValid
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
  SmBoolean bRtn = TRUE;
  
  // call the base class AssertValid
  bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
           ? SmTopology::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
           : TRUE ) ; 

  // when OwningTopology has a list of children
  if(m_pList)
    {
      // walk the child's doubly linked list 
      //   looking for this object
      //   checking next/last pointer reciprocity
      //   checking owner pointers
      //   checking list length
      //   checking children and parents use same context
      SmTopology *pTgt,  *pStart = m_pList ;
      ULONG       lCount, lSize  = m_lListSize ;

      for ( pTgt =  pStart,   lCount =  0;
            ( lCount == 0 ) || ( pTgt != pStart && lCount < lSize );
            pTgt = pTgt->m_pNext, lCount++)
        {
          // avoid NULL pointers
          if(   pTgt == NULL
             || pTgt->m_pNext == NULL
             || pTgt->m_pNext->m_pLast == NULL)
            {
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (FALSE), _T("") ) ;
              break ;
            }

          // check next/last pointer reciprocity
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (pTgt->m_pNext->m_pLast == pTgt), _T("") ) ;

          // check owner pointers
          if ( this->TypicalListOwnerUse() ) { 
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (pTgt->m_pListOwner == this), _T("") ) ;
          }

          // check list length
          if(lCount == lSize - 1) {
              bRtn &= SM_ASSERT_BOOLEAN_REPORT(3, SM_LEVEL_0, (pTgt->m_pNext == pStart), _T("") ) ;
          }

          // check for common contexts
          bRtn &= SM_ASSERT_BOOLEAN_REPORT(4, SM_LEVEL_0, (GetContext() == pTgt->GetContext()), _T("") ) ;


        } // end iter doubly linked list
    } // end has a list branch
  else // has no list
    {
      // should not be part of a peer list
      bRtn &= SM_ASSERT_BOOLEAN_REPORT(5, SM_LEVEL_0, (m_lListSize == 0), _T("") ) ;
    }

  // all done
  // SM_ASSERT(bRtn) ;
  return(bRtn) ;

} // end SmOwningTopology::AssertValid

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmOwningTopology::AssertHeal
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
//       return ( SmTopology::AssertHeal(rAReport, pAList) ) ;
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
//                rAReport.m_pHealMessage = _T("SmOwningTopology::AssertHeal fix not yet supported") ;  
//                 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmOwningTopology::AssertHeal
// end obsolete

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmOwningTopology::IsKindOf( SM_TYPE t ) const
{
  return ((SmOwningTopology_TYPE == t) ? TRUE : SmTopology::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmOwningTopology::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
  smos_sprintf(sBuff,       _T("SmOwningTopology object - ListSize = %ld, m_pList = 0x%p\n   "),m_lListSize,m_pList);
  smos_sprintf(sBuffForFile,_T("SmOwningTopology object - ListSize = %ld, m_pList = %s  \n   "),m_lListSize,m_pList ? _T("notNULL") : _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // locals
  ULONG       ii = 0;
  SmTopology *pThis = m_pList ;

  // for every member on the list of owned topology objects
  for(; m_pList && (   pThis != m_pList 
                    || ii    == 0); pThis = pThis->m_pNext, ii++)
    {
      // output the list members
      smos_sprintf(sBuff,       _T("-> 0x%p "),pThis);
      smos_sprintf(sBuffForFile,_T("-> %s ")  ,pThis ? _T("notNULL") : _T("NULL"));
      smos_WriteBuffer(sBuff, sBuffForFile);

      // limit 5 members per line
      if(ii%5 == 0 && ii != 0) { smos_WriteBuffer(_T("\n     ")) ; }

    } // end iter every owned member

} // end SmOwningTopology::Dump
