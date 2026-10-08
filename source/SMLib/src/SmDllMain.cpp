// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmDllMain.cpp
* PURPOSE: Source for dll entry point function
**********************************************************************/

#include "StdAfx.h"

// this implementation is only for windows
#if defined(_WIN32)



#include <SmThreadLocalStorage.h>
#include <windows.h>
#include <SmDllMain.h>

/*******************************************************************//**
PURPOSE: Entry Point function for the DLL used to set up 
         Thread Local Storage. 

NOTES:

This function is called by Windows for the various threads so each
call to this function will have a different memory.

1. SMLib branches on different system TLS memory models. All code
that manages TLS memory is encapsulated into the SmThreadLocalStorage
class in the files SmThreadLocalStorage.[h,cpp].  If you need to change
how TLS memory works in SMLib, put those changes into the methods
of SmThreadLocalStorage branching for the various supported systems
and only call those functions from this file. 

2. When a process uses load-time linking with this DLL, the entry-point function is 
sufficient to manage the thread local storage. Problems can occur with a process 
that uses run-time linking (LoadLibrary()) because the entry-point function is not called for 
threads that exist before the LoadLibrary function is called, so TLS memory 
is not allocated for these threads. The following example solves this problem 
by checking the value returned by the TlsGetValue function and allocating memory 
if the value indicates that the TLS slot for this thread is not set. 

LPVOID lpvData; 
 
// Retrieve a data pointer for the current thread.
lpvData = TlsGetValue(dTLSIndex_1); 
 
// If NULL, allocate memory for this thread.
if (lpvData == NULL) 
  { 
    lpvData = (LPVOID) LocalAlloc(LPTR, 256); 
    if (lpvData != NULL) 
        TlsSetValue(dTLSIndex_1, lpvData); 
  }
  
3. known windows memory leak
  windows is not calling DLL_THREAD_ATTACH for every thread
   attached to the process thread with a DLL_PROCESS_ATTACH call.
  Each DLL_THREAD_ATTACH call allocates one SmThreadLocalStorage object.
  Each DLL_THREAD_DETACH call deletes one SmThreadLocalStorage object.

  One SmThreadLocalStorage object leaks for every thread that is
  attached but not detached by windows. Currently there we don't know how to work
  around this leak.
***********************************************************************/

BOOL APIENTRY DllMain
( HINSTANCE hinstDLL,      // in : DLL module handle
  DWORD     fdwReason,     // in : reason called: 
                           //      one of DLL_PROCESS_ATTACH               
                           //             DLL_THREAD_ATTACH                
                           //             DLL_PROCESS_DETACH               
                           //             DLL_THREAD_DETACH 
  LPVOID    lpvReserved )   // in : reserved
{
    SmStatus  sRtn = SM_SUCCESS;

#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if(bDebugMe)
    {
        TCHAR sBuff[SM_TBLOCK_SIZE];
        DWORD dThreadid = GetCurrentThreadId();
        smos_sprintf( sBuff, _T( "DllMain:Reason Called[%s] for thread[%ld(0x%x)]]\n" ),
                    fdwReason == DLL_PROCESS_ATTACH ? _T( "SM_TLS_PROCESS_ATTACH" )
                    : fdwReason == DLL_THREAD_ATTACH ? _T( "SM_TLS_THREAD_ATTACH " )
                    : fdwReason == DLL_PROCESS_DETACH ? _T( "SM_TLS_PROCESS_DETACH" )
                    : fdwReason == DLL_THREAD_DETACH ? _T( "SM_TLS_THREAD_DETACH " )
                    : _T( "Unknown Call Reason" ),
                    dThreadid, dThreadid );
        smos_WriteBuffer( sBuff );
    }
#endif // SM_DEBUG_CODE


    // switch on reason for this call  
    switch (fdwReason)
    {
        // The DLL is loading due to process initialization or a call to LoadLibrary.
    case DLL_PROCESS_ATTACH: { // if critical sections are required this is the place
                               // to initialize the critical section object
                               // InitializeCriticalSection(&csSync);

                               // Allocate a TLS slot for this and all future threads with an index value
                               // and load its value with a pointer to a new SmThreadLocalStorage object
                               //   sets global dTLSIndex value.
                               //     each TlsAlloc() call specifies that this and future threads are assigned one TLS slot
                               //     whose contents are accessed by the index value returned by the call.
                               //     The index value is common to the process, the contents of the slots are thread specific.
                               //   Slot data is set with              TlsSetValue(dTLSIndex, data) ;
                               //   Slot data is retrieved with data = TlsGetValue(dTLSIndex) ;
                               //     SMLib uses the TLS slots to store ptrs to an SmThreadLocalStorage objects.
                               //     SmThreadLocalStorage objects contain all SMLib thread local storage data.
                               //      So, it is expected that SMLib will only use one slot of ThreadLocalStorage.
                               //     If a 2nd slot is ever desired create and manage dTLSIndex_2.
        sRtn = SmThreadLocalStorage::SetupTLS(SM_TLS_PROCESS_ATTACH);
    }
                             break;

                             // The attached process creates a new thread.
    case DLL_THREAD_ATTACH: { // fetch (allocate if needed) a TLS slot for this thread using the s_iLTSIndex index value
                               // and load the slot's value with a pointer to a new SmThreadLocalStorage object
                               //   Slot data is set with              TlsSetValue(dTLSIndex, data) ;
                               //   Slot data is retrieved with data = TlsGetValue(dTLSIndex) ;
        sRtn = SmThreadLocalStorage::SetupTLS(SM_TLS_THREAD_ATTACH);

    }
                            break;

                            // The thread of the attached process terminates.
    case DLL_THREAD_DETACH: { // delete the SmThreadLocalStorage object for this thread
        sRtn = SmThreadLocalStorage::EndTLS(SM_TLS_THREAD_DETACH);
    }
                            break;

                            // DLL unload due to process termination or FreeLibrary. 
    case DLL_PROCESS_DETACH: { // delete the SmThreadLocalStorage object for this thread, then release the TLS index memory block.
        sRtn = SmThreadLocalStorage::EndTLS(SM_TLS_PROCESS_DETACH);
    }
                             break;


    default:                sRtn = SM_SUCCESS;
        break;
    }  // end switch on fdwReason

  // all done
    return(sRtn == SM_SUCCESS);
    UNREFERENCED_PARAMETER(hinstDLL);
    UNREFERENCED_PARAMETER(lpvReserved);

} // end DllMain 

#endif
