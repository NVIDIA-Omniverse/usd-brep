// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmThreadLocalStorage.h
* PURPOSE: Header file for Thread Local Memory Management.
**********************************************************************/

#ifndef __SMTHREADLOCALSTORAGE_H_
#define __SMTHREADLOCALSTORAGE_H_

#define TLS_FAIL  0xFFFFFFFF

#include <SmTypes.h>
#include <SmMessages.h>

// including macros here cause some compile conflicts
//  replace #include <SmTArray.h> with a local array definition supporting only
//   AddUnique(), FindElement(), and RemoveAt()
class SmObject ;

/*******************************************************************//**
PURPOSE: Global access to ThreadLocalStorage index unique to 
            each thread.

NOTES: Does not need to be exported.
***********************************************************************/

enum SmTLSCallType
{
  SM_TLS_GET,              // called whenever TLS memory is being accessed for a fetch
  SM_TLS_SET,              // called whenever TLS memory is being accessed for a put
  SM_TLS_PROCESS_ATTACH,   // called when Process attaches
  SM_TLS_THREAD_ATTACH,    // called when Thread attaches
  SM_TLS_THREAD_DETACH,    // called when Thread detaches
  SM_TLS_PROCESS_DETACH    // called when Process detaches
};

#define SM_MAX_THREAD_COUNT 16  // only used to manage the size of the m_pBeenThroughNew array below

/*******************************************************************//**
PURPOSE:  The SmThreadLocalStorage object is used to store information
  that is unique to each thread running in a multi-thread environment.
    
NOTES: The use of thread local storage is set up in part
  by the DLL entry point function, DllMain().

  Within a function the following syntax can be used to get a TLS
  specific piece of data

  {
    ...
    // example: access to m_pBeenThroughNew value
    SmBoolean bBeenThroughNew = SmThreadLocalStorage::GetBeenThroughNew(pObject) ;

  }

***********************************************************************/
class SM_EXPORT SmThreadLocalStorage
{
protected:
  SmObject *m_pBeenThroughNew[SM_MAX_THREAD_COUNT] = {NULL} ;
  ULONG     m_lSize = 0 ;
  // SmTArray<SmObject *> * m_pBeenThroughNew ; // pObjects on this list have been through
                                                // SmObject::operate new() but have not yet 
                                                // been through the SmObject::constructor.
                                                // pObjects items are added in SmObject::operator new()
                                                // and removed in SmObject::constructor.
                                         // This is used as a workaround from the original
                                         // mistake of placing an assignment of the m_cpContext 
                                         // pointer in the overloaded new member.  It turns
                                         // out overloaded new runs when allocating objects
                                         // on the heap but does not run for objects
                                         // created on the stack.  The constructor which
                                         // runs for both cases needs to decide when to
                                         // initialize the m_cpContext pointer and it uses 
                                         // the value of m_bBeenThroubhNew to do that.

  ULONG m_lSERDepth = 0 ;                // counts nesting depth of SER() calls, InitValue:[0]
  ULONG m_lAssertValidDepth = 0 ;        // counts nesting depth of AssertValid() calls, InitValue:[0]
                                         
  // pointer to a function of the ErrCallbackFunction type
  SmErrorCallbackFunctionPtr 
       m_pfErrCallbackFunction = NULL ;  // Callback function pointer used by smos_ErrorMessage
                                         // to give users a chance to respond to errors given
                                         // the number and location of the error.

public:
  // constructor
  SmThreadLocalStorage()                 : // m_pBeenThroughNew(NULL),
                                           m_lSize(0),
                                           m_lSERDepth(0),     
                                           m_lAssertValidDepth(0),
                                           m_pfErrCallbackFunction(NULL) 
                                         {
                                           // special new that does not call SmObject::operator new(cpContext) to avoid infinite loop at startup
                                           // special SmTArray constructor use, FALSE = don't call GetBeenThroughNew()
                                           //   This is the only place in SMLib that uses the FALSE argument.
                                           // m_pBeenThroughNew = new (this) SmTArray<SmObject*>(0,NULL,0,FALSE) ; 
                                         }
                                         
  // destructor                          
 ~SmThreadLocalStorage()                 { // m_bBeenThroughNew = 0 ; 
                                           // if(m_pBeenThroughNew) { delete m_pBeenThroughNew ; m_pBeenThroughNew = NULL ; }
                                           m_lSize = 0 ;
                                           m_lSERDepth = 0 ;
                                           m_lAssertValidDepth = 0 ;
                                           m_pfErrCallbackFunction = NULL ; 
                                         }

  // Simple Data Access - these are static to be used as 'global' access to these thread specific values
  static void      SetBeenThroughNew(SmBoolean  bBeenThroughNew,      // in : TRUE = remember pObject, FALSE = forget pObject
                                     SmObject * pObject) ;            // NotUsed: in : used only to communicate state between 
  static void      SetErrCallbackFunction(SmErrorCallbackFunctionPtr  // in : Callback function ptr to store 
                                          pfErrCallbackFunction) ;    //      Called in smos_ErrorMessage()

  static SmBoolean                  GetBeenThroughNew(SmObject * pObject) ; // get: been through overloaded SmObject::new state value from SmObject constructors, // NotUsed: in : pObject 
  static SmErrorCallbackFunctionPtr GetErrCallbackFunction() ;              // get: Callback function ptr called by smos_ErrorMessage()

  static ULONG                      GetSERDepth() ;         // get: thread's SER() call depth value
  static ULONG                      GetAssertValidDepth() ; // get: thread's AssertValid() call depth value

  static ULONG                      IncSERDepth() ;         // eff: increment SER depth level by 1
  static ULONG                      DecSERDepth() ;         // eff: Decrement SER depth level by 1
  static ULONG                      IncAssertValidDepth() ; // eff: increment AssertValid depth level by 1
  static ULONG                      DecAssertValidDepth() ; // eff: Decrement AssertValid depth level by 1

  // internal methods for thread local memory management
  
  // allocated Thread Local memory block for one thread
  // also allocates the TLS index block when needed 
  static SmStatus SetupTLS(SmTLSCallType eTLSCallType) ;

  // delete ThreadLocal memory block and TLS index box
  static SmStatus EndTLS(SmTLSCallType eTLSCallType) ;

private:
  // Object list management: AddUnique(), FindElement(), and RemoveAt()
  static ULONG     Add         (SmObject * pBeenThroughNew[SM_MAX_THREAD_COUNT], ULONG *pSize, SmObject * sNewElement);
  static SmBoolean AddUnique   (SmObject * pBeenThroughNew[SM_MAX_THREAD_COUNT], ULONG *pSize, SmObject * pNewObject);
  static SmBoolean FindElement (SmObject * pBeenThroughNew[SM_MAX_THREAD_COUNT], ULONG *pSize, SmObject * pObjToFind, ULONG & nFoundIndex) ;
  static void      RemoveAt    (SmObject * pBeenThroughNew[SM_MAX_THREAD_COUNT], ULONG *pSize, ULONG nIndex, ULONG nCount=1);
} ; // end class SmThreadLocalStorage

#endif // !__SMTHREADLOCALSTORAGE_H_


