// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- smos_error.h
* PURPOSE: Header file used for error handling.
**********************************************************************/

#ifndef __SMOS_ERROR_H__
#define __SMOS_ERROR_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef _INC_STRING
#include <string>
#endif

#ifndef _INC_TIME
#include <time.h>
#endif

#ifndef __SMTHREADLOCALSTORAGE_H_
#include <SmThreadLocalStorage.h>
#endif

// #ifndef _INC_STDARG
// #include <stdarg.h>   // variable length argument lists for SM_ASSERT_TESTS
// #endif

// forward declares
class SmAssertArray ;
// class SmAssertReport ;
class SmAxis2Placement ;  
class SmPseudoBox ;       
class SmPeriodicExtent1d ;
class SmExtent1d ;
class SmPolarConversion ;
class SmPolarBox ;
class SmExtent2d ;
class SmExtent3d ;

// CHANGED TO LONG INT
#define SM_SUCCESS                       1000L
#define SM_ERR                           1001L
#define SM_ERR_OUT_OF_MEMORY             1002L
#define SM_ERR_UNKNOWN                   1003L
#define SM_ERR_FATAL                     1004L
#define SM_ERR_ASSERT_FAILURE            1005L
#define SM_ERR_NULL_POINTER              1006L
#define SM_ERR_INVALID_INPUT             1007L
#define SM_ERR_NON_NULL_OUTPUT_POINTER   1008L
#define SM_ERR_ASSERTVALID_FAILURE       1010L
#define SM_ERR_OUTSIDE_OF_DOMAIN         1020L
#define SM_ERR_NOT_WITHIN_TOLERANCE      1021L
#define SM_ERR_NOT_CONVERGING            1022L
#define SM_ERR_WARNING                   1023L
#define SM_ERR_MESSAGE                   1024L
#define SM_ERR_AXIS_INSIDE_FACE          1025L
#define SM_ERR_LICENSE_EXPIRED           1026L
#define SM_ERR_BAD_INTERSECTIONS         1027L
#define SM_ERR_BAD_COINCIDENT_VERTICES   1028L
#define SM_ERR_BAD_SURFACE_POINT         1030L
#define SM_ERR_BAD_TANGENT_DROP          1032L
#define SM_ERR_DEGENERATE_SURFACE        1034L
#define SM_ERR_BAD_FIND_DEGEN_PARAM      1036L
#define SM_ERR_METHOD_FAILURE_QUITING    1038L
#define SM_ERR_NOTYET_HEAL_FACE          1040L
#define SM_ERR_TESS_FAIL_FACE            1041L

/***********************************************************************
Control Output File and Stream output for smos_WriteBuffer()
***********************************************************************/
SM_EXPORT void smSet_DoGraphics    (SmBoolean b_DoGraphics) ;
SM_EXPORT void smSet_OutputLong    (SmBoolean bDoLong) ;
SM_EXPORT void smSet_OutputThin    (SmBoolean bDoThin) ;
SM_EXPORT void smSet_OutputDebugLog(SmBoolean bDoDebugLog) ;
SM_EXPORT void smSet_OutputCommandLog(SmBoolean bDoCommandLog);
SM_EXPORT void smSet_OutputApiLog(SmBoolean bDoApiLog);

SM_EXPORT SmBoolean smGet_DoGraphics    () ;
SM_EXPORT SmBoolean smGet_OutputLong    () ;
SM_EXPORT SmBoolean smGet_OutputThin    () ;
SM_EXPORT SmBoolean smGet_OutputDebugLog() ;
SM_EXPORT SmBoolean smGet_OutputCommandLog() ;
SM_EXPORT SmBoolean smGet_OutputApiLog() ;

SM_EXPORT void smos_WriteBuffer( const TCHAR *pBuff,  
                                 const TCHAR *pBuffForFile = NULL,
                                 const TCHAR *pBuffForDebugLog = NULL) ;

// This is a new function for Unicode 
SM_EXPORT void smos_ErrorMessage(SmStatus            error_number, 
                                 const TCHAR * const file_name, 
                                 ULONG               line_number, 
                                 const TCHAR * const error_message,
                                 const TCHAR * const error_message2 = NULL,
                                 const TCHAR * const func_name=NULL) ; 


SM_EXPORT FILE *smos_DirectStdOutToFile  (TCHAR * pFileName,                  // open empty fileLong for write
                                          TCHAR * pFileNameThin = NULL,       // open empty fileThin for write
                                          TCHAR * pFileNameDebugLog = NULL) ; // open empty fileDebugLog for write

SM_EXPORT FILE* smos_DirectStdOutToFile(std::string* pFileName,                 // open empty fileLong for write
                                        std::string* pFileNameThin = NULL,      // open empty fileThin for write
                                        std::string* pFileNameDebugLog = NULL); // open empty fileDebugLog for write

SM_EXPORT FILE *smos_ReDirectStdOutToFile(TCHAR * pFileName) ;                // open existing file for append
SM_EXPORT void smos_RestoreStdOut(FILE *pStream) ;     // NOT USED: pStream

SM_EXPORT void smos_WriteDouble(const TCHAR* buff, double dValue);

SM_EXPORT void smos_WriteLong(const TCHAR* buff, ULONG lValue);


// passes call to smos_ErrorMessage
SM_EXPORT void smos_AssertErrorMessage(SmStatus            error, 
                                       const TCHAR * const file_name, 
                                       ULONG               line_num, 
                                       const TCHAR * const message,
                                       const TCHAR * const messageForFile=NULL,
                                       SmBoolean           bAssertBreak = FALSE,
                                       const TCHAR * const func_name=NULL) ;


SM_EXPORT void smos_SetUICallback(void (*pfUICallbackFunction)());

SM_EXPORT void smos_InvokeUICallback();

SM_EXPORT void smos_SetErrorCallback(SmErrorCallbackFunctionPtr pfErrorCallbackFunction);

SM_EXPORT void smos_InvokeErrorCallback(SmStatus            sError, 
                                        const TCHAR * const file_name, 
                                        ULONG               line_num, 
                                        const TCHAR * const message);

// Debug pick-list hooks. Draw() methods called with bAddToUIPickList = TRUE
// pass the object to the callback registered here.
// Nothing in this repository registers these callbacks, so that flag has no
// effect unless a host application does.
SM_EXPORT void smos_SetBrepListCallback(void (*pAddToBrepListFunction)(const SmObject *pObject)) ;
SM_EXPORT void smos_InvokeBrepListCallback(const SmObject *pObject) ;
SM_EXPORT void smos_SetBrepListClearCallback(void (*pRemoveFromBrepListFunction)()) ;
SM_EXPORT void smos_InvokeBrepListClearCallback() ;

SM_EXPORT void sm_PrintTime(const TCHAR* string, clock_t start, clock_t finish);

SM_EXPORT SmBoolean smos_ExcapeCallback();

SM_EXPORT void smos_SetExcapeCallback(int (*pfExcapeCallbackFun)());

/*******************************************************************//**
PURPOSE:  ERROR and ASSERT MACROS

 defined for all compiles:
   SE(a)          = Execute a - if(a!=SM_SUCCESS) Signal Error - continue processing
   SE_MSG(a,msg)  = Execute a - if(a!=SM_SUCCESS) signal Error with message - continue processing
   SER(a)         = Execute a - if(a!=SM_SUCCESS) Signal Error - return a (the ERROR CODE)
   SERN(a)        = Execute a - if(a!=SM_SUCCESS) Signal Error - return NULL VALUE
   SERZ(a)        = Execute a - if(a!=SM_SUCCESS) Signal Error - return void
   SER_MSG(a,msg) = Execute a - if(a!=SM_SUCCESS) Signal Error with message - return a (the ERROR CODE)
   SERN_MSG(a,msg)= Execute a - if(a!=SM_SUCCESS) Signal Error with message - return NULL VALUE
   NE(a)          = Execute a - if(a!=SM_SUCCESS) Signal NULL Pointer error - continue processing
   NER(a)         = Execute a - if(a!=SM_SUCCESS) Signal NULL Pointer error - return NULL POINTER ERROR
   NERN(a)        = Execute a - if(a!=SM_SUCCESS) Signal NULL Pointer error - return NULL VALUE

   AE(a)             = if(a!=TRUE) Signal SM_ERR_ASSERT_FAILURE
   AE_MSG(a,msg)     = if(a!=TRUE) Signal SM_ERR_ASSERT_FAILURE with a message
   AER(a)            = if(a!=TRUE) Signal SM_ERR_ASSERT_FAILURE and return false
   AER_MSG(a,msg)    = if(a!=TRUE) Signal SM_ERR_ASSERT_FAILURE with a message and return false
   AERN(a,n)         = if(a!=TRUE) Signal error n, return n
   AERN_MSG(a,n,msg) = if(a!=TRUE) Signal error n with message, return n
   AERS(a)           = if(a!=TRUE) Signal SM_ERR_ASSERT_FAILURE and return SM_ERR
   AERS_MSG(a,msg)   = if(a!=TRUE) Signal SM_ERR_ASSERT_FAILURE with a message and return SM_ERR

   Error signaling above is enabled by SM_DEBUG_CODE; return behavior is
   the same in all builds. AER/AER_MSG are for Boolean-returning functions.
   SmStatus functions should use AERN/AERN_MSG with an explicit error status,
   or AERS/AERS_MSG for SM_ERR. FALSE (0) is not a valid SmStatus.
   
   ERR(n)              = Signal Error number - no message
   ERR_MSG(msg)        = Signal SM_ERR_UNKNOWN with message msg
   ERR_MSG2(msg1,msg2) = Signal SM_ERR_UNKNOWN with message msg1 and messageForFile msg2
   WARN(msg)           = Signal SM_ERR_WARNING with message msg
   WARN2(msg1,msg2)    = Signal SM_ERR_WARNING with message msg1 and messageForFile msg2
   MSG(msg)            = Signal SM_ERR_MESSAGE with message msg
   MSG2(msg1,msg2)     = Signal SM_ERR_MESSAGE with message msg1 and messageForFile msg2

   RANGE_ER(low,mid,high) = mid is contained in [low high]       else signal SM_ERR_INVALID_INPUT         
   ARRAY_LT_ER(arr,index) = index is valid for array size        else signal SM_ERR_INVALID_INPUT
   ZERO_VEC_ER(vec)       = vec length greater than SM_EFF_ZERO  else signal SM_ERR_INVALID_INPUT
   LE_ZERO_ER(a)          = a >= SM_EFF_ZERO                     else signal SM_ERR_INVALID_INPUT
   LT_ZERO_ER(a)          = a >= 0.0                             else signal SM_ERR_INVALID_INPUT
   SAME_PNT_ER(p1,p2)     = Dist(a,b) >= SM_EFF_ZERO             else signal SM_ERR_INVALID_INPUT

 only defined when compiling with SM_DEBUG_CODE:
   SM_ASSERT(a)                   = when !a, Signal error 
   SM_ASSERT_ERR                  = always,  Signal error
   SM_ASSERT_ERR_MSG              = always,  Signal error with message
   SM_ASSERT_MSG(a,msg)           = when !a, Signal error with message 
   SM_ASSERT_BREAK(a)             = when !a, Output Signal error with a flag suitable for debugging break points
   SM_ASSERT_MSG_BREAK(a,msg)     = when !a, Output Signal error with message and a flag suitable for debugging break points

   SM_ASSERT_TOL(tol)             = when tol==SM_UNINIT_TOL, Output Signal error with a flag suitable for debugging break points
   SM_ASSERT_TOL2(tol1, tol2)     = when tol1 or tol2 == SM_UNINIT_TOL, Output Signal error with a flag suitable for debugging break points
   SM_ASSERT_TOLPTR(pTol)         = when pTol != NULL && *pTol == SM_UNINIT_TOL, Output Signal error with a flag suitable for debugging break points
   SM_ASSERT_TOLPTR2(p1, p2)      = when p1 or p2 !NULL & val == SM_UNINIT_TOL, Output Signal error with a flag suitable for debugging break points

   SM_DUMP_TARRAY(sArray)         = pretty print an array of objects - code required for each array type supported

   SM_DBG_ERR(msg)           = signal SM_ERR and output message
   SM_DBG_WARN(msg)          = output message
   SM_DBG_WARN_IF(a,msg)     = if(a) {output message msg }
   SM_DBG_WARN2(msg_a,msg_b) = output two part message

NOTES ---  Messages are not output when SM_DEBUG_CODE is not defined. 
***********************************************************************/
#ifdef SM_DEBUG_CODE

  // SE(a) = Execute a - Signal Error - continue processing
  #define SE(a)           { SmStatus sErr = (a);     \
                            if (sErr != SM_SUCCESS)  \
                              { smos_ErrorMessage(sErr,FILE_NAME,LINE_NUMBER,(TCHAR*)0,(TCHAR*)0,FUNC_NAME); } \
                          } 

  // SE_MSG(a) = Execute a - Signal Error with message - continue processing
  #define SE_MSG(a,msg)   { SmStatus sErr = (a);     \
                            if (sErr != SM_SUCCESS)  \
                              { smos_ErrorMessage(sErr,FILE_NAME,LINE_NUMBER,(const TCHAR * const )(msg),(TCHAR*)0,FUNC_NAME); } \
                          }    

  // NE(a) = Execute a - Signal NULL Pointer error - continue processing
  #define NE(a)           { const void* vVoid = (a); \
                            if (vVoid == NULL)       \
                              { smos_ErrorMessage(SM_ERR_NULL_POINTER,FILE_NAME,LINE_NUMBER,NULL,NULL,FUNC_NAME); } \
                          }
                        
  // NER(a) = Execute a - Signal NULL Pointer error - return NULL POINTER ERROR
  #define NER(a)          { const void* vVoid = (a); \
                            if (vVoid == NULL)       \
                              { smos_ErrorMessage(SM_ERR_NULL_POINTER,FILE_NAME,LINE_NUMBER,NULL,NULL,FUNC_NAME); \
                                return SM_ERR_NULL_POINTER; \
                              } \
                          }
                        
  // NER(a) = Execute a - Signal NULL Pointer error with message - return NULL POINTER ERROR
  // This macro has unreachable code
  #define NER_MSG(a,msg)  { const void* vVoid = (a); \
                            if (vVoid == NULL)       \
                              { smos_ErrorMessage(SM_ERR_NULL_POINTER,FILE_NAME,LINE_NUMBER,msg,NULL,FUNC_NAME); \
                                return SM_ERR_NULL_POINTER; \
                              } \
                          }
  // NERN(a) = Execute a - Signal NULL Pointer error - return NULL VALUE
  #define NERN(a)         { void* vVoid = (a);       \
                            if (vVoid == NULL)       \
                              { smos_ErrorMessage(SM_ERR_NULL_POINTER,FILE_NAME,LINE_NUMBER,NULL,NULL,FUNC_NAME); \
                                return NULL;         \
                              } \
                          }
                          
  // SER(a) = Execute a - Signal Error - return ERROR CODE
  #define SER(a)          { SmStatus sErr = (a);     \
                            if (sErr != SM_SUCCESS)  \
                              { smos_ErrorMessage(sErr,FILE_NAME,LINE_NUMBER,(TCHAR*)0,(TCHAR*)0,FUNC_NAME); \
                                return(sErr) ;       \
                              } \
                          }

  // SER_DELETE(a, p) = Execute a - Signal Error - return ERROR CODE - Delete pointer p
  #define SER_DELETE(a, p)  { SmStatus sErr = (a);     \
                            if (sErr != SM_SUCCESS)  \
                              { \
                                if (p) \
                                { \
                                   delete p; \
                                   p = NULL; \
                                } \
                                smos_ErrorMessage(sErr,FILE_NAME,LINE_NUMBER,(TCHAR*)0,(TCHAR*)0,FUNC_NAME); \
                                return(sErr) ;       \
                              } \
                          }
  // SER_DELETE_ALL_PARTS(a) = Execute a - Signal Error - return ERROR CODE - Delete all of parts
  #define SER_DELETE_ALL_PARTS(a, parts) { SmStatus sErr = (a);  \
                                            if (sErr != SM_SUCCESS) \
                                            { \
                                                 for (ULONG z = 0; z < (parts).GetSize(); z++)    \
                                                 {                                                \
                                                     SM_ASSERT((parts)[z] != NULL);               \
                                                     delete (parts)[z];                           \
                                                     (parts)[z] = NULL;                           \
                                                 }                                                \
                                                 (parts).ReSet();                                 \
                                                 smos_ErrorMessage(sErr, FILE_NAME, LINE_NUMBER, (TCHAR*)0, (TCHAR*)0, FUNC_NAME); \
                                                 return (sErr);                                   \
                                            } \
                                         }
  // SER_MSG(a,msg) = Execute a - Signal Error with message - return ERROR CODE
  #define SER_MSG(a,msg)  { SmStatus sErr = (a);     \
                            if (sErr != SM_SUCCESS)  \
                              { smos_ErrorMessage(sErr,FILE_NAME,LINE_NUMBER,msg,(TCHAR*)0,FUNC_NAME); \
                                return sErr;         \
                              } \
                          }
  // SERN(a) = Execute a - Signal Error - return NULL VALUE
  #define SERN(a)        { SmStatus sErr = (a);     \
                           if (sErr != SM_SUCCESS)  \
                             { smos_ErrorMessage(sErr,FILE_NAME,LINE_NUMBER,(TCHAR*)0,(TCHAR*)0,FUNC_NAME); \
                               return NULL;         \
                             } \
                         }

  // SERN(a) = Execute a - Signal Error with message - return NULL VALUE
  #define SERN_MSG(a,msg){ SmStatus sErr = (a);     \
                           if (sErr != SM_SUCCESS)  \
                             { smos_ErrorMessage(sErr,FILE_NAME,LINE_NUMBER,msg,(TCHAR*)0,FUNC_NAME); \
                               return NULL;         \
                             } \
                         }

  // SERZ(a) = Execute a - Signal Error - return void
  #define SERZ(a)        { SmStatus sErr = (a);     \
                           if (sErr != SM_SUCCESS)  \
                             { smos_ErrorMessage(sErr,FILE_NAME,LINE_NUMBER,(TCHAR*)0,(TCHAR*)0,FUNC_NAME); \
                               return;              \
                             } \
                         }
  // AE(a) = Assert a: if(a!=true) - Signal SM_ERR_ASSERT_FAILURE - continue processing              
  #define AE(a)          { if (!(a))  \
                             { smos_ErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,NULL,NULL,FUNC_NAME); } \
                         } 
                        
  // AE_MSG(a,msg) = Assert a with MSG: if(a!=true) - Signal SM_ERR_ASSERT_FAILURE with message - continue processing
  #define AE_MSG(a,msg)  { if (!(a))     \
                             { smos_ErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,(const TCHAR * const )(msg),NULL,FUNC_NAME); } \
                         }    

  // AER(a) = Assert a and return: if(a!=true) - Signal SM_ERR_ASSERT_FAILURE - return(FALSE)
  //   FALSE is not a status: in a function returning SmStatus use AERN(a, SM_ERR_ASSERT_FAILURE).
  #define AER(a)         { if ((a) != TRUE)     \
                             { smos_ErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,NULL,NULL,FUNC_NAME); \
                               return(FALSE) ;  \
                             } \
                         }
  // AER_MSG(a,msg) = Assert a with MSG and return: if(a!=true) - Signal SM_ERR_ASSERT_FAILURE with message - return(FALSE)
  #define AER_MSG(a,msg) { if ((a) != TRUE)     \
                             { smos_ErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,(const TCHAR * const )(msg),NULL,FUNC_NAME); \
                               return(FALSE) ;  \
                             } \
                         }
  // AERN(a,n) = Assert a and return: if(a!=true) - Signal error n - return(n)
  #define AERN(a,n)      { if ((a) != TRUE) \
                             { smos_ErrorMessage(n,FILE_NAME,LINE_NUMBER,NULL,NULL,FUNC_NAME); \
                               return(n) ;  \
                             } \
                         }
  // AERN_MSG(a,n,msg) = Assert a with MSG and return: if(a!=true) - Signal error n with message - return(n)
  #define AERN_MSG(a,n,msg) { if ((a) != TRUE)        \
                                { smos_ErrorMessage(n,FILE_NAME,LINE_NUMBER,(const TCHAR * const )(msg),NULL,FUNC_NAME); \
                                  return(n) ;         \
                                } \
                            }
  // AERS(a) = Assert a and return: if(a!=true) - Signal SM_ERR_ASSERT_FAILURE - return(SM_ERR)
  #define AERS(a)           { if ((a) != TRUE) \
                                { smos_ErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,NULL,NULL,FUNC_NAME); \
                                  return(SM_ERR) ;  \
                                } \
                            }
  // AERS_MSG(a,msg) = Assert a with MSG and return: if(a!=true) - Signal SM_ERR_ASSERT_FAILURE with message - return(SM_ERR)
  #define AERS_MSG(a,msg)   { if ((a) != TRUE)        \
                                { smos_ErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,(const TCHAR * const )(msg),NULL,FUNC_NAME); \
                                  return(SM_ERR) ;         \
                                } \
                            }
  // SM_ASSERT(a) = when !a, Signal error
  #define SM_ASSERT(a)   { SmBoolean bBool = (a) ;  \
                           if (!bBool) { smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,NULL,NULL,0,FUNC_NAME); } \
                         }

  #define SM_ASSERT_ERR  { smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,NULL,NULL,0,FUNC_NAME); \
                         }

  #define SM_ASSERT_ERR_MSG(msg)  { SmThreadLocalStorage::IncSERDepth() ; \
                                    smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,msg,NULL,0,FUNC_NAME); \
                                    SmThreadLocalStorage::DecSERDepth() ; \
                                  }

  #define SM_ASSERT_BREAK(a) { SmBoolean bBool = (a) ;  \
                               if (!bBool) { smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,NULL,NULL,1,FUNC_NAME); } \
                             }

  // SM_ASSERT_MSG(a,msg) = when !a, Signal error with message 
  #define SM_ASSERT_MSG(a,msg) { SmBoolean bBool = (a) ;    \
                                 if (!bBool) { smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,msg,NULL,FALSE,FUNC_NAME); } \
                               }

  #define SM_ASSERT_BREAK_MSG(a,msg) { SmBoolean bBool = (a) ;    \
                                       if (!bBool) { smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,msg,NULL,1,FUNC_NAME); } \
                                     }
  #define SM_ASSERT_MSG_BREAK(a,msg) { SmBoolean bBool = (a) ;    \
                                       if (!bBool) { smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,msg,NULL,1,FUNC_NAME); } \
                                     }


  #define SM_ASSERT_TOL(tol)    { if ((tol)==SM_UNINIT_TOL) { smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,_T("Uninit-Tol"),NULL,1,FUNC_NAME); } \
                                }
  #define SM_ASSERT_TOL2(t1,t2) { if ((t1)==SM_UNINIT_TOL || (t2) == SM_UNINIT_TOL) { smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,_T("Uninit Tolerance"),NULL,1,FUNC_NAME); } \
                                }
  #define SM_ASSERT_TOLPTR(pTol) { if ((pTol) != NULL && *(pTol)==SM_UNINIT_TOL) { smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,_T("Uninit Tolerance"),NULL,1,FUNC_NAME); } \
                                 }
  #define SM_ASSERT_TOLPTR2(p1,p2) { if (((p1) != NULL && *(p1)==SM_UNINIT_TOL) || ((p2) != NULL && *(p2)==SM_UNINIT_TOL)) { smos_AssertErrorMessage(SM_ERR_ASSERT_FAILURE,FILE_NAME,LINE_NUMBER,_T("Uninit Tolerance"),NULL,1,FUNC_NAME); } \
                                   }

  // SM_DBG_ERR(msg) = Signal error with message 
  #define SM_DBG_ERR(msg) { smos_AssertErrorMessage(SM_ERR,FILE_NAME,LINE_NUMBER,msg,NULL,FALSE,FUNC_NAME); }
  
  // SM_DBG_WARN(msg) = output warning message
  #define SM_DBG_WARN(msg)    WARN(msg)

  // SM_DBG_WARN_IF(a,b) = if(a) { output warning message b}
  #define SM_DBG_WARN_IF(a,msg)  { if(a) { WARN(msg) } }

  // SM_DBG_WARN2(a,b) = output two part warning message
  #define SM_DBG_WARN2(msg_a,msg_b) WARN2(msg_a,msg_b)
   
  // SM_DBG_MSG(b,msg) = output debug message if 'b' is true
  #define SM_DBG_MSG(b,msg) { if (b) { smos_WriteBuffer(msg); } }

  #define SM_ASSERT_DEFINED(a) \
  { (a)->AssertDefined() ; }

  // functions for SM_DUMP_TARRAY(sArray) support

  // double, float, ULONG, long, int, unsigned int
  inline void sm_ItemDump(TCHAR * buff, ULONG i, double         dDbl)   { smos_sprintf(buff,_T("\t[%3ld] = %16.16lf\n"),i,dDbl); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, double       * dDbl)   { smos_sprintf(buff,_T("\t[%3ld] = %16.16lf\n"),i,*dDbl); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, float          dFloat) { smos_sprintf(buff,_T("\t[%3ld] = %8.8f\n"),i,dFloat); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, float        * dFloat) { smos_sprintf(buff,_T("\t[%3ld] = %8.8f\n"),i,*dFloat); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, ULONG          lLong)  { smos_sprintf(buff,_T("\t[%3ld] = %3ld\n"),i,lLong); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, ULONG        * lLong)  { smos_sprintf(buff,_T("\t[%3ld] = %3ld\n"),i,*lLong); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, long           lLong)  { smos_sprintf(buff,_T("\t[%3ld] = %3ld\n"),i,lLong); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, long         * lLong)  { smos_sprintf(buff,_T("\t[%3ld] = %3ld\n"),i,*lLong); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, int            lInt)   { smos_sprintf(buff,_T("\t[%3ld] = %d\n"),i,lInt); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, int          * lInt)   { smos_sprintf(buff,_T("\t[%3ld] = %d\n"),i,*lInt); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, unsigned int   lInt)   { smos_sprintf(buff,_T("\t[%3ld] = %d\n"),i,lInt); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, unsigned int * lInt)   { smos_sprintf(buff,_T("\t[%3ld] = %d\n"),i,*lInt); }

  // void *, const void *
  inline void sm_ItemDump(TCHAR * buff, ULONG i, const void * pPointer)  { smos_sprintf(buff,_T("\t[%3ld] = 0x%p\n"),i,pPointer); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, const void **pPointer)  { smos_sprintf(buff,_T("\t[%3ld] = 0x%p\n"),i,*pPointer); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, void * pPointer)        { smos_sprintf(buff,_T("\t[%3ld] = 0x%p\n"),i,pPointer); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, void **pPointer)        { smos_sprintf(buff,_T("\t[%3ld] = 0x%p\n"),i,*pPointer); }

  // SmVector3d, SmVector2d
  class SmVector3d ;
  class SmVector2d ; 
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmVector3d  & rPnt) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmVector3d  * rPnt) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmVector2d  & rPnt) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmVector2d  * rPnt) ;

  // SmContinuityTYPE, SmOrientType, SmTsectCurveType
  inline void sm_ItemDump(TCHAR * buff, ULONG i, SmContinuityType    eCont)   { smos_sprintf(buff,_T("\t[%3ld] = %3ld\n"),i,(ULONG)eCont); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, SmContinuityType  * eCont)   { smos_sprintf(buff,_T("\t[%3ld] = %3ld\n"),i,(ULONG)(*eCont)); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, SmOrientType        eOrient) { smos_sprintf(buff,_T("\t[%3ld] = %3ld\n"),i,(ULONG)eOrient); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, SmOrientType      * eOrient) { smos_sprintf(buff,_T("\t[%3ld] = %3ld\n"),i,(ULONG)(*eOrient)); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, SmTsectCurveType    eTSType) { smos_sprintf(buff,_T("\t[%3ld] = %3ld\n"),i,(ULONG)eTSType); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, SmTsectCurveType  * eTSType) { smos_sprintf(buff,_T("\t[%3ld] = %3ld\n"),i,(ULONG)(*eTSType)); }

  // SmExtent1d, SmExtent2d, SmExtent3d
  class SmExtent1d ;
  class SmExtent2d ;
  class SmExtent3d ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmExtent1d & rIvl) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmExtent1d * pIvl) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmExtent2d & rIvl) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmExtent2d * pIvl) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmExtent3d & rIvl) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmExtent3d * pIvl) ;

  // SmTSectPnt
  class SmTsectPnt ;
  inline void sm_ItemDump(TCHAR * buff, ULONG i, SmTsectPnt * pPointer) { smos_sprintf(buff,_T("\t[%3ld] = 0x%p\n"),i,pPointer); }
  inline void sm_ItemDump(TCHAR * buff, ULONG i, SmTsectPnt **pPointer) { smos_sprintf(buff,_T("\t[%3ld] = 0x%p\n"),i,*pPointer); }

  //inline void sm_ItemDump(TCHAR* buff, ULONG i, SmObject *pPointer)  { smos_sprintf(buff,_T("\t[%3ld] = 0x%p\n"),i,pPointer); }

  // SmBrep, SmRegion, SmShell
  class SmRegion ;
  class SmShell ;
  // class SmBrep ;
  // SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmBrep   * pBrep)   ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmRegion * pRegion) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmShell  * pShell)  ;
                                                            
  // SmFace, SmEdge, SmVertex     
  class SmFace ;
  class SmEdge ;
  class SmVertex ;                            
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmFace   * pFace)   ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmEdge   * pEdge)   ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmVertex * pVertex) ;

  // SmIterationValue
  class SmIterationValue ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmIterationValue * pIVal) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmIterationValue & rIVal) ;

  // SmGapSample
  class SmGapSample ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmGapSample * pGSmp) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmGapSample & rGSmp) ;

  // SmLocalInterval
  class SmLocalInterval ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmLocalInterval * pLIvl) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmLocalInterval & rLIvl) ;

  // SmGfxVertex, SmGfxNVertex, SmGfxColoredVertex, SmGfxTexturedVertex
  class SmGfxVertex ;
  class SmGfxNVertex ;
  class SmGfxColoredVertex ;
  class SmGfxTexturedVertex ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxVertex & rPnt) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxVertex * rPnt) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxNVertex & rPnt) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxNVertex * rPnt) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxColoredVertex & rPnt) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxColoredVertex * rPnt) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxTexturedVertex & rPnt) ;
  SM_EXPORT void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxTexturedVertex * rPnt) ;

// The following need implementations - yuk
  // SmShellData, SmRegionData, SmFaceData, SmFaceuseData, SmLoopData, SmEUData, SmEdgeData, SmVertexData
  class SmShellData ;
  class SmRegionData ;
  class SmFaceData ; class SmFaceuseData ;
  class SmLoopData ;
  class SmEdgeData ; class SmEUData ;
  class SmVertexData ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmShellData * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmShellData & rDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmRegionData * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmRegionData & rDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmFaceData * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmFaceData & rDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmFaceuseData * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmFaceuseData & rDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmLoopData * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmLoopData & rDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmEUData * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmEUData & rDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmEdgeData * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmEdgeData & rDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmVertexData * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmVertexData & rDLst) ;

  // SmPolyRegionData, SmPolyShellData, SmPolyFaceData, SmPolyLoopData, SmPolyEdgeProp, SmPolyEdgeData, SmPolyVertexData
  class SmPolyRegionData ;
  class SmPolyShellData ;
  class SmPolyFaceData ;
  class SmPolyLoopData ;
  struct SmPolyEdgeProp ;
  class SmPolyEdgeData ;
  class SmPolyVertexData ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmPolyRegionData * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmPolyRegionData & rDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmPolyShellData * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmPolyShellData & rDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmPolyFaceData * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmPolyFaceData & rDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmPolyLoopData * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmPolyLoopData & rDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmPolyEdgeProp * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmPolyEdgeProp & rDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmPolyEdgeData * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmPolyEdgeData & rDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmPolyVertexData * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmPolyVertexData & rDLst) ;

  // SmDisplayList
  class SmDisplayList ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmDisplayList * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmDisplayList & rDLst) ;

  // SmTreeVertex
  class SmTreeVertex ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmTreeVertex * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmTreeVertex & rDLst) ;

  // SmIntersectionResults
  class SmIntersectionResults ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmIntersectionResults * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const SmIntersectionResults & rDLst) ;

  // StackData
  struct StackData ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const StackData * pDLst) ;
// needs implementation:  void sm_ItemDump(TCHAR * buff, ULONG i, const StackData & rDLst) ;


  /*******************************************************************//**
  PURPOSE: Debug convenience: Macro to pretty print some SmTArray<TYPES>

  NOTES: need a sm_ItemDump() for each supported kind of SmTArray<TYPE>
    GWC: there has to be a better way - I can't see it yet.
  ***********************************************************************/
  // indirection needed to concatenate macro variables into unique names using the __LINE__ macro
  #define SM_CAT2_NESTED(a,b) a##b
  #define SM_CAT(a,b) SM_CAT2_NESTED(a,b)
  // example use
  //  #define SM_MY_MACRO { int SM_CAT(lName,__LINE__) ; \ // this expands out to int lName777 ; (if you happen to be on file line #777)
  //                        SM_CAT(lName,__LINE__) = 7 ; } // this expands out to lName777 = 7 ; (the __LINE__ macro returns the line number for the 
  //                                                                                              start of the outer most nested macro. It can be used 
  //                                                                                              in a multi-line macro to always ref the same name)
  #define SM_DUMP_TARRAY(RefToTArray) \
    { TCHAR SM_CAT(sBuff,__LINE__)[SM_TBLOCK_SIZE] ; \
      smos_sprintf(SM_CAT(sBuff,__LINE__), _T("\nSmTArray Size:[%ld], AllocSize:[%ld]\n"),RefToTArray.GetSize(),RefToTArray.GetDataSize()); smos_WriteBuffer(SM_CAT(sBuff,__LINE__)); \
      for(ULONG i=0;i<RefToTArray.GetSize();i++) \
        { sm_ItemDump(SM_CAT(sBuff,__LINE__),i,RefToTArray.GetAt(i)); smos_WriteBuffer(SM_CAT(sBuff,__LINE__)) ; } \
    }
   
#else // no SM_DEBUG_CODE

  #define SE(a) ((void)(a))   
  #define SE_MSG(a,msg)  ((void)(a)) 

  #define NE(a)   
  #define NER(a)             \
  { const void* vVoid = (a); \
    if (vVoid == NULL)       \
      { return SM_ERR_NULL_POINTER; } \
  }

  #define NER_MSG(a,msg)     \
  { const void* vVoid = (a); \
    if (vVoid == NULL)       \
      { return SM_ERR_NULL_POINTER; } \
  }
  #define NERN(a)            \
  { void* vVoid = (a);       \
    if (vVoid == NULL)       \
      { return NULL; }       \
  }

  #define SER(a)             \
  { SmStatus sErr = (a);     \
    if (sErr != SM_SUCCESS)  \
      { return sErr; }       \
  }

  #define SER_DELETE(a, p)  { SmStatus sErr = (a);     \
                            if (sErr != SM_SUCCESS)  \
                              { \
                                if (p) \
                                { \
                                   delete p; \
                                   p = NULL; \
                                } \
                                return(sErr) ;       \
                              } \
                          }

  #define SER_DELETE_ALL_PARTS(a, parts) { SmStatus sErr = (a);  \
                                            if (sErr != SM_SUCCESS) \
                                            { \
                                                 for (ULONG z = 0; z < (parts).GetSize(); z++)    \
                                                 {                                                \
                                                     delete (parts)[z];                           \
                                                     (parts)[z] = NULL;                           \
                                                 }                                                \
                                                 (parts).ReSet();                                 \
                                                 return (sErr);                                   \
                                            } \
                                         }

  #define SER_MSG(a,msg)     \
  { SmStatus sErr = (a);     \
    if (sErr != SM_SUCCESS)  \
      { return sErr; }       \
  }   

  #define SERN(a)            \
  { SmStatus sErr = (a);     \
    if (sErr != SM_SUCCESS)  \
      { return NULL; }       \
  } 

  #define SERN_MSG(a)        \
  { SmStatus sErr = (a);     \
    if (sErr != SM_SUCCESS)  \
      { return NULL; }       \
  } 

  #define SERZ(a)            \
  { SmStatus sErr = (a);     \
    if (sErr != SM_SUCCESS)  \
      { return; }            \
  }     

  // AE(a) = Assert a macro set              
  #define AE(a)
  // Like AE: signal-and-continue in debug builds, nothing in release builds.
  #define AE_MSG(a,msg)
  #define AER(a)            { if ((a) != TRUE) { return(FALSE) ; }   } 
  #define AER_MSG(a,msg)    { if ((a) != TRUE) { return(FALSE) ; }   }
  #define AERS(a)           { if ((a) != TRUE) { return(SM_ERR) ; }  }
  #define AERS_MSG(a,msg)   { if ((a) != TRUE) { return(SM_ERR) ; }  }
  #define AERN(a,n)         { if ((a) != TRUE) { return(n) ;     }   } 
  #define AERN_MSG(a,n,msg) { if ((a) != TRUE) { return(n) ;     }   } 

  #define SM_ASSERT(a)   if(a) {}  
  #define SM_ASSERT_ERR
  #define SM_ASSERT_ERR_MSG(msg)
  #define SM_ASSERT_BREAK(a)  if(a) {}  
//  #define SM_ASSERT_MSG(a, msg) { SmBoolean bBool = (a); bBool = !bBool; }
  #define SM_ASSERT_MSG(a, msg) 

  #define SM_ASSERT_MSG_BREAK(a,msg)
  #define SM_ASSERT_BREAK_MSG(a,msg)
  #define SM_ASSERT_TOL(tol)
  #define SM_ASSERT_TOL2(tol1, tol2)
  #define SM_ASSERT_TOLPTR(pTol)
  #define SM_ASSERT_TOLPTR2(p1,p2) 
//  #define SM_TMP_ASSERT_ARRAY(ptr_name)  SmAssertArray *ptr_name = NULL ;
  #define SM_TMP_ASSERT_ARRAY(ptr_name)
  #define SM_DUMP_ASSERT_ARRAY(name)
  #define SM_DUMP_TARRAY(rArray)  


  // #define SM_GRAPHICSLOOP() 
  #define SM_ASSERT_DEFINED(a)
  #define SM_DBG_ERR(msg)
  #define SM_DBG_WARN(msg)
  #define SM_DBG_WARN_IF(a,msg)   
  #define SM_DBG_WARN2(msg_a,msg_b) 
  #define SM_DBG_MSG(b,msg)

#endif // no SM_DEBUG_CODE

// a silly macro to use formal paramters within dummy functions to allow
//   the functions to be easily read for documentation, but to compile
//   without an endless list of 'unreferenced formal parameter' warnings
//   and without compiling with a pragma which turns off this useful warning
//   for all cases.

//#define SM_REF1(a)             if(&a) { long i=0 ; i++ ; }
//#define SM_REF2(a,b)           if(&a || &b) { long i=0 ; i++ ; }
//#define SM_REF3(a,b,c)         if(&a || &b || &c) { long i=0 ; i++ ; }
//#define SM_REF4(a,b,c,d)       if(&a || &b || &c || &d) { long i=0 ; i++ ; }
//#define SM_REF5(a,b,c,d,e)     if(&a || &b || &c || &d || &e) { long i=0 ; i++ ; }
//#define SM_REF6(a,b,c,d,e,f)   if(&a || &b || &c || &d || &e || &f) { long i=0 ; i++ ; }
//#define SM_REF7(a,b,c,d,e,f,g) if(&a || &b || &c || &d || &e || &f || &g) { long i=0 ; i++ ; }

#define SM_REF1(a)             ((void)a)
#define SM_REF2(a,b)           ((void)a); ((void)b)
#define SM_REF3(a,b,c)         ((void)a); ((void)b); ((void)c)
#define SM_REF4(a,b,c,d)       ((void)a); ((void)b); ((void)c); ((void)d)
#define SM_REF5(a,b,c,d,e)     ((void)a); ((void)b); ((void)c); ((void)d); ((void)e)
#define SM_REF6(a,b,c,d,e,f)   ((void)a); ((void)b); ((void)c); ((void)d); ((void)e); ((void)f)
#define SM_REF7(a,b,c,d,e,f,g) ((void)a); ((void)b); ((void)c); ((void)d); ((void)e); ((void)f); ((void)g)
#define SM_REF8(a,b,c,d,e,f,g,h) ((void)a); ((void)b); ((void)c); ((void)d); ((void)e); ((void)f); ((void)g); ((void)h)
#define SM_REF9(a,b,c,d,e,f,g,h,i) ((void)a); ((void)b); ((void)c); ((void)d); ((void)e); ((void)f); ((void)g); ((void)h); ((void)i)
#define SM_REF10(a,b,c,d,e,f,g,h,i,j) ((void)a); ((void)b); ((void)c); ((void)d); ((void)e); ((void)f); ((void)g); ((void)h); ((void)i); ((void)j)
#define SM_REF11(a,b,c,d,e,f,g,h,i,j,k) ((void)a); ((void)b); ((void)c); ((void)d); ((void)e); ((void)f); ((void)g); ((void)h); ((void)i); ((void)j); ((void)k)
#define SM_REF12(a,b,c,d,e,f,g,h,i,j,k,l) ((void)a); ((void)b); ((void)c); ((void)d); ((void)e); ((void)f); ((void)g); ((void)h); ((void)i); ((void)j); ((void)k); ((void)l )

/*******************************************************************//**
PURPOSE: document and Trace (when compiled with SM_DEBUG_TRACE)
debugging changes at run time.
***********************************************************************/

#ifdef SM_DEBUG_TRACE
  // when debug tracing is desired
  #define SMS_CHANGE(initials, date, comment)                     \
  { static int first_hit = 1 ;                                    \
    if(first_hit) { SMS_print_change(_T("initials"), _T("date"), comment, \
                                     __FILE__, __LINE__) ;        \
                    first_hit = 0 ;                               \
                  }                                               \
  }

  void smos_PrintChange               // eff: add debug history line to FILE *fp
    (const TCHAR *initials,            // in : change author's ID
     const TCHAR *version,             // in : version number in which change is made
     const TCHAR *comment,             // in : short explanation for the change
     const TCHAR *file,                // in : file name for change call
     int line,                        // in : line number for change call
     FILE *fp=stdout) ;               // in : target output stream

#else // when debug tracing is not defined
  #define SMS_CHANGE(initials, version, comment)

#endif // SM_DEBUG_TRACE not defined branch
     

#define ERR(a) smos_ErrorMessage((a),FILE_NAME,LINE_NUMBER,NULL,NULL,FUNC_NAME);

#define ERR_MSG(a)    smos_ErrorMessage(SM_ERR_UNKNOWN,(const TCHAR * const)FILE_NAME,LINE_NUMBER,(const TCHAR * const)a,NULL,FUNC_NAME);
#define ERR_MSG2(a,b) smos_ErrorMessage(SM_ERR_UNKNOWN,(const TCHAR * const)FILE_NAME,LINE_NUMBER,(const TCHAR * const)a,(const TCHAR * const)b,FUNC_NAME);
#define ERR_MSG3(a)   smos_ErrorMessage(SM_ERR_UNKNOWN,(const TCHAR * const)FILE_NAME,LINE_NUMBER,(const TCHAR * const)a,NULL,FUNC_NAME);

#define WARN(a)    smos_ErrorMessage(SM_ERR_WARNING, (const TCHAR * const)FILE_NAME, LINE_NUMBER,(const TCHAR * const)a,NULL,FUNC_NAME);
#define WARN2(a,b) smos_ErrorMessage(SM_ERR_WARNING, (const TCHAR * const)FILE_NAME, LINE_NUMBER,(const TCHAR * const)a, (const TCHAR * const)b,FUNC_NAME);

#define MSG(a)    smos_ErrorMessage(SM_ERR_MESSAGE,(const TCHAR * const)FILE_NAME,LINE_NUMBER,(const TCHAR * const)a,NULL,FUNC_NAME);
#define MSG2(a,b) smos_ErrorMessage(SM_ERR_MESSAGE,(const TCHAR * const)FILE_NAME,LINE_NUMBER,(const TCHAR * const)a,(const TCHAR * const)b,FUNC_NAME);

// Signal SM_ERR_INVALID_INPUT error when conditions are not met
#define RANGE_ER(low,mid,high) { if( !( (mid) >= (low) && (mid) <= (high)        ) ) { SER(SM_ERR_INVALID_INPUT); } } /* mid contained in [low high]         */ 
#define ARRAY_LT_ER(arr,index) { if( !( (arr).GetSize() >= (index)               ) ) { SER(SM_ERR_INVALID_INPUT); } } /* index is valid for array size       */ 
#define ZERO_VEC_ER(vec)       { if( !( (vec).LengthSquared() >= SM_EFF_ZERO_SQ  ) ) { SER(SM_ERR_INVALID_INPUT); } } /* vec length greater than SM_EFF_ZERO */ 
#define LE_ZERO_ER(a)          { if( !( (a)   >= SM_EFF_ZERO                     ) ) { SER(SM_ERR_INVALID_INPUT); } } /* a >= SM_EFF_ZERO                    */ 
#define LT_ZERO_ER(a)          { if( !( (a)   >= 0.0                             ) ) { SER(SM_ERR_INVALID_INPUT); } } /* a >= 0.0                            */ 
#define SAME_PNT_ER(p1,p2)     { if( !( (p1).DistanceBetween((p2)) >= SM_EFF_ZERO) ) { SER(SM_ERR_INVALID_INPUT); } } /* Dist(a,b) >= SM_EFF_ZERO            */ 
                                      
#endif  // !__SMOS_ERROR_H__
