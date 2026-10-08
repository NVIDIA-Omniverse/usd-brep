// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmCodeCount.h
* PURPOSE: Header file used for error handling.
**********************************************************************/

#ifndef __SMOS_CODE_COUNT_H__
#define __SMOS_CODE_COUNT_H__

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

/*******************************************************************//**
PURPOSE:  MACRO Based Code Counting Interface

NOTES: Code Counting is managed by 3 macros
    SM_INIT_CALL_COUNTS              // Clear all counters back to zero
    SM_DUMP_CALL_COUNTS              // Pretty Print all non-zero counters
    SM_COUNT_CALL(QuotedCounterName) // Placed where ever counting is desired.
                                     // It's count will be reported by the
                                     //    SM_DUMP_CALL_COUNTS macro and labeled
                                     //    with the QuotedCounterName string.

  These macros are only defined when compiled with SM_DEBUG_CODE,
  Otherwise they disappear and are not included in the compiled code.                                       
********************************************************************/
#ifdef SM_DEBUG_CODE

#define SM_INIT_CALL_COUNTS                SmInitCallCounts() 
#define SM_DUMP_CALL_COUNTS                SmDumpCounts() 
#define SM_COUNT_CALL(QuotedCounterName)           \
           { static ULONG lIndex, lCallCount = 0 ; \
             if(lCallCount == 0) { lIndex = SmFirstCountCall(QuotedCounterName, &lCallCount) ; } \
             else                { SmCountCall(lIndex) ; } \
           }

#else  // SM_DEBUG_CODE not defined - and macros must be removed from the compiled code

#define SM_BEGIN_CALL_COUNTS
#define SM_DUMP_CALL_COUNTS
#define SM_COUNT_CALL(QuotedCounterName)

#endif // branching on SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE -- Class for a function call counter

NOTES: Supports the macro interface - not to be called by users
********************************************************************/
class SmCodeCounter
{
 protected:
   // A single counter
   ULONG *m_pCount ;          // pointer to counter's count
   TCHAR  m_sName[SM_TBLOCK_SIZE] ;      // name used to label count output

 public:
   // constructor
   SmCodeCounter(TCHAR  *pName, 
                 ULONG *pCount) ;
   ~SmCodeCounter()                 { m_pCount = NULL ; } 

   // simple data access, counting and reporting
   void  Init()                    { *m_pCount = 0 ;       }
   ULONG AddCount()                { return(*m_pCount++) ; }
   ULONG GetCount() const          { return(*m_pCount  ) ; }   
   TCHAR *GetName()                { return m_sName ;      }
   void  Dump(ULONG ii) ;

} ; // end class SmCodeCounter

/*******************************************************************//**
PURPOSE: Class to contain active list of function call counters

NOTES: Users should never access this class directly.  All
  they need to use are the macros

  SM_BEGIN_CALL_COUNTS
  SM_COUNT_CALL(a)    
  SM_DUMP_CALL_COUNTS 
**********************************************************************/
class SmCodeCounterList
{
 protected:
  SmTArray<SmCodeCounter*> m_sCodeCountList ; // list of all nonZero code counters

 public:
   // constructors
   SmCodeCounterList() { }
   ~SmCodeCounterList() ;

   // modifiers
   void  Init() ;
   void  Dump() const ;
   ULONG GetCount (ULONG lIndex) ;
   ULONG CountCall(ULONG lIndex) ;
   ULONG AddAndIncrement(SmCodeCounter *pNewCounter) ;

} ; // end class SmCodeCounterList 

/*******************************************************************//**
PURPOSE: Global functions used by the MACRO interface

NOTES:  Users should never need to access these functions directly. 
  All users need are the macros,

  SM_BEGIN_CALL_COUNTS                                SmInitCallCounts()
  SM_COUNT_CALL(a)                                    SmDumpCounts()    
  SM_DUMP_CALL_COUNTS
*********************************************************************/
SM_EXPORT void  SmInitCallCounts() ; 
SM_EXPORT void  SmDumpCounts() ;
SM_EXPORT void  SmCountCall(ULONG lIndex) ;
SM_EXPORT ULONG SmGetCallCount(ULONG lIndex) ;
SM_EXPORT ULONG SmFirstCountCall(TCHAR *pCounterName, ULONG *pCount) ;

#endif  // !__SMOS_CODE_COUNT_H__
