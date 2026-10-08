// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmCodeCount.cpp
* PURPOSE: Counts for code testing.
**********************************************************************/

#include "StdAfx.h"

#include <stdio.h> 
#include <SmCodeCount.h>

// one global counterlist
static SmCodeCounterList s_CounterList ;

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmCodeCounter::SmCodeCounter
  (TCHAR  *pName, 
   ULONG *pCount) 
 : m_pCount(pCount) 
{ 
  SM_SPRINTF(m_sName, _T("%s"), pName) ; 

} // end SmCodeCounter::SmCodeCounter constructor

/*******************************************************************//**
PURPOSE: Output counter value

NOTES:
***********************************************************************/
void SmCodeCounter::Dump
  (ULONG ii)    // in : index value used to label counter output string
{ 
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  
  // output the counter index, value, and name label 
  SM_SPRINTF(sBuff, _T("\n %4ld: %ld calls to "), ii, *m_pCount);
  smos_WriteBuffer(sBuff) ;
  smos_WriteBuffer(m_sName) ;

} // end SM_CodeCounter::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmCodeCounterList::~SmCodeCounterList() 
{ 
  SmObjsDelete<SmCodeCounter *> sClean(&m_sCodeCountList) ; 

} // end SmCodeCounterList::~SmCodeCounterList destructor
      
/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmCodeCounterList::Init()  
{
  // delete all the current code counters
  { SmObjsDelete<SmCodeCounter *> sClean(&m_sCodeCountList) ; 
  }

  // reset the array size
  m_sCodeCountList.ReSet() ;

} // end SmCodeCounterList::Init

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
void SmCodeCounterList::Dump() const  
{
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  ULONG ii, lTotalCount = 0 ;

  // Label the count output
  smos_WriteBuffer(_T("\n FUNCTION CALL COUNTS")) ;
  smos_WriteBuffer(_T("\n Index, Entries, Function Name")) ;

  // dump every counter 
  for(ii=0;ii<m_sCodeCountList.GetSize();ii++)
    {
      m_sCodeCountList[ii]->Dump(ii) ;
      lTotalCount += m_sCodeCountList[ii]->GetCount() ;
    }

  // output summary
  SM_SPRINTF(sBuff, _T("\n\n Total Function Entries Counted: %ld"), lTotalCount);
  smos_WriteBuffer(sBuff) ;

} // end SmCodeCounterList::Dump

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
ULONG SmCodeCounterList::AddAndIncrement(SmCodeCounter *pNewCounter)  
{
  // add Counter
  m_sCodeCountList.Add(pNewCounter) ;

  // increment
  pNewCounter->AddCount() ;

  // return index
  return(m_sCodeCountList.GetSize() - 1) ;

} // end SmCodeCounterList::AddAndIncrement

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
ULONG SmCodeCounterList::CountCall(ULONG lIndex)  
{
  // check input
  SM_ASSERT(lIndex < m_sCodeCountList.GetSize()) ;

  // increment the counter and return incemented value
  return(m_sCodeCountList[lIndex]->AddCount()) ;

} // end SmCodeCounterList::CountCall

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
ULONG SmCodeCounterList::GetCount(ULONG lIndex)  
{
  // check input
  SM_ASSERT(lIndex < m_sCodeCountList.GetSize()) ;

  // increment the counter and return incemented value
  return(m_sCodeCountList[lIndex]->GetCount()) ;

} // end SmCodeCounterList::GetCount

/*******************************************************************//**
PURPOSE: Global Functions used by the CallCount Macros

NOTES:
***********************************************************************/
void  SmInitCallCounts()           { s_CounterList.Init() ; }
void  SmDumpCounts()               { s_CounterList.Dump() ; }
void  SmCountCall   (ULONG lIndex) { s_CounterList.CountCall(lIndex) ; }
ULONG SmGetCallCount(ULONG lIndex) { return(s_CounterList.GetCount(lIndex)) ; }
ULONG SmFirstCountCall(TCHAR  *pCounterName,
                       ULONG *pCount) 
{ return(s_CounterList.AddAndIncrement(new SmCodeCounter(pCounterName, pCount))) ;
}
