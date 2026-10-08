// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME: SmMessages.cpp
* PURPOSE: Error handling functions
**********************************************************************/

#include "StdAfx.h"

#include <SmMessages.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMREGION_H__
#include <SmRegion.h>
#endif

#ifndef __SMSHELL_H__
#include <SmShell.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#ifndef __SMEDGE_H__
#include <SmEdge.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#include <SmGraphicsVertexArray.h>

#ifdef SM_USE_EXCEPTIONS
#include <new>
#endif // SM_USE_EXCEPTIONS  

#if defined(_WIN32) && defined(SM_DEBUG_CODE)
  // included for debugging declaration of OutputDebugString(buff);
  #include <windows.h>
#endif


extern "C" {
   void (*c3d_smlib_error_handler)(int error, const TCHAR *file_name, int line_num, const TCHAR *message);
}

/***********************************************************************
Control Output File and Stream output for smos_WriteBuffer()
***********************************************************************/
static SmBoolean b_DoGraphics = false;       // TRUE = make graphics calls when running a test suite
static SmBoolean b_OutputLong = false;       // TRUE = smos_WriteBuffer calls send sBuff TCHAR array to stream 
static SmBoolean b_OutputThin = false;       // TRUE = smos_WriteBuffer calls send sBuffForFile array to stream
static SmBoolean b_OutputDebugLog = false;   // TRUE = smos_WriteBuffer calls send sBuffForDebugLog array to stream
static SmBoolean b_OutputCommandLog = false; // TRUE = smos_WriteBuffer calls send sBuffForCommandLog array to stream
static SmBoolean b_OutputApiLog = false;     // TRUE = smos_WriteBuffer calls send sBuffForApiLog array to stream

void smSet_DoGraphics    (SmBoolean bDoGraphics) { b_DoGraphics     = bDoGraphics ; }
void smSet_OutputLong    (SmBoolean bDoLong)     { b_OutputLong     = bDoLong ; }
void smSet_OutputThin    (SmBoolean bDoThin)     { b_OutputThin     = bDoThin ; }
void smSet_OutputDebugLog(SmBoolean bDoDebugLog) { b_OutputDebugLog = bDoDebugLog ; }
void smSet_OutputCommandLog(SmBoolean bDoCommandLog) { b_OutputCommandLog = bDoCommandLog ; }
void smSet_OutputApiLog(SmBoolean bDoApiLog) { b_OutputApiLog = bDoApiLog ; }
                                                 
SmBoolean smGet_DoGraphics    ()                 { return b_DoGraphics ; }
SmBoolean smGet_OutputLong    ()                 { return b_OutputLong ; }
SmBoolean smGet_OutputThin    ()                 { return b_OutputThin ; }
SmBoolean smGet_OutputDebugLog()                 { return b_OutputDebugLog ; }
SmBoolean smGet_OutputCommandLog()               { return b_OutputCommandLog ; }
SmBoolean smGet_OutputApiLog()                   { return b_OutputApiLog ; }

// Convenience file name strings for managing prog_test log files - when both fileNames are empty - stdout has not been redirected
static SM_THREAD_LOCAL TCHAR sOutputFile        [SM_TBLOCK_SIZE] = _T("") ; // set in smos_DirectStdOutToFile() when pFileName != NULL 
static SM_THREAD_LOCAL TCHAR sOutputFileThin    [SM_TBLOCK_SIZE] = _T("") ; // set in smos_DirectStdOutToFile() when pFileNameThin != NULL
static SM_THREAD_LOCAL TCHAR sOutputFileDebugLog[SM_TBLOCK_SIZE] = _T("") ; // set in smos_DirectStdOUtToFile() when pFileNameDebugLog != NULL

/*******************************************************************//**
PURPOSE: This function sets the UI callback function.  This function
   is the one which gets the UI up and running.  It will be used in
   debugging.

NOTES:
***********************************************************************/
static void (*s_pfUICallbackFun)() = NULL;

void smos_SetUICallback
  (void (*pfUICallbackFunction)())
{
    s_pfUICallbackFun = pfUICallbackFunction;

} // end smos_SetUICallback

/*******************************************************************//**
PURPOSE: This function sets the Excape callback function.

NOTES:
***********************************************************************/
static int (*s_pfExcapeCallbackFun)() = NULL;

void smos_SetExcapeCallback
  (int (*pfExcapeCallbackFun)())
{
    s_pfExcapeCallbackFun = pfExcapeCallbackFun;

} // end smos_SetExcapeCallback

/*******************************************************************//**
PURPOSE: This function calls the excape callback function to test
   for excape key being hit.  If TRUE then the excape key has been hit.

NOTES:
***********************************************************************/
SmBoolean smos_ExcapeCallback()
{
  // when the escape key has been hit
  if (   s_pfExcapeCallbackFun
      && s_pfExcapeCallbackFun())
    {
#ifdef SM_USE_EXCEPTIONS
      throw std::bad_alloc();
#else
      return TRUE;
#endif
    }

  // else
  return FALSE;

} // end smos_ExcapeCallback

/*******************************************************************//**
PURPOSE: Invoke the UI callback function if it is initialized;

NOTES:
***********************************************************************/
void smos_InvokeUICallback()
{
    if (s_pfUICallbackFun) { s_pfUICallbackFun(); }

} // end smos_InvokeUICallback

/*******************************************************************//**
PURPOSE: This function sets the error callback function.

NOTES:
***********************************************************************/
// static void (*s_pfErrorCallbackFun)
//   (SmStatus sErrorCode,
//    const TCHAR * const file_name,
//    ULONG line_num,
//    const TCHAR * const  message) = NULL;
//
// void smos_SetErrorCallback
//  (void (*pfErrorCallbackFunction)
//     (SmStatus sErrorCode,
//      const TCHAR * const file_name,
//      ULONG line_num,
//      const TCHAR * const message
//  )  )
void smos_SetErrorCallback
  (SmErrorCallbackFunctionPtr pfErrorCallbackFunction)
{
  // s_pfErrorCallbackFun = pfErrorCallbackFunction;
  SmThreadLocalStorage::SetErrCallbackFunction(pfErrorCallbackFunction) ;

} // end smos_SetErrorCallback

/*******************************************************************//**
PURPOSE: Invoke the UI callback function if it is initialized;

NOTES:
***********************************************************************/
void smos_InvokeErrorCallback
  (SmStatus            sErrorCode,
   const TCHAR * const file_name = NULL,
   ULONG               line_num=0,
   const TCHAR * const message = NULL)
{
  // if (s_pfErrorCallbackFun) { s_pfErrorCallbackFun(sErrorCode, file_name, line_num,message); }
  SmErrorCallbackFunctionPtr pfErrorCallbackFunction = SmThreadLocalStorage::GetErrCallbackFunction() ;

  if(pfErrorCallbackFunction)
    {
      pfErrorCallbackFunction(sErrorCode, file_name, line_num,message) ;
    }

} // end smos_InvokeErrorCallback

/*******************************************************************//**
PURPOSE: This function sets the Add to UI BrepList callback function.

NOTES:
***********************************************************************/
static void (*s_pfBrepListFun)(const SmObject *pObject) = NULL;

void smos_SetBrepListCallback(void (*pAddToBrepListFunction)(const SmObject *pObject))
{
  s_pfBrepListFun = pAddToBrepListFunction;

} // end smos_SetBrepListCallback

/*******************************************************************//**
PURPOSE: Invoke the add to UI BrepList function if it is initialized;

NOTES:
***********************************************************************/
void smos_InvokeBrepListCallback
  (const SmObject *pObject)
{
  if (s_pfBrepListFun)
    { s_pfBrepListFun(pObject);
    }

} // end smos_InvokeBrepListCallback

/*******************************************************************//**
PURPOSE: This function sets the clear UI BrepList callback function.

NOTES:
***********************************************************************/
static void (*s_pfBrepListClearFun)() = NULL ;

void smos_SetBrepListClearCallback
  (void (*pRemoveFromBrepListClearFunction)())
{
  s_pfBrepListClearFun = pRemoveFromBrepListClearFunction;

} // end smos_SetBrepListClearCallback

/*******************************************************************//**
PURPOSE: Invoke the Clear UI BrepList function if it is initialized;

NOTES:
***********************************************************************/
void smos_InvokeBrepListClearCallback()
{
  if (s_pfBrepListClearFun) { s_pfBrepListClearFun(); }

} // end smos_InvokeBrepListClearCallback

/*******************************************************************//**
PURPOSE: Write an error message out to the screen.

NOTES: When debugging it is a good idea to set a break in this function.
***********************************************************************/
void smos_AssertErrorMessage
  (SmStatus            error,            // in :
   const TCHAR * const file_name,        // in :
   ULONG               line_num,         // in :
   const TCHAR * const message,          // in :
   const TCHAR * const messageForFile,   // in : default:[NULL]
   SmBoolean           bAssertBreak,     // in : TRUE = came from SM_ASSERT_BREAK macro
                                         //      FALSE= came from SM_ASSERT macro
                                         //      default:[FALSE]
   const TCHAR * const func_name)        // in : default:[NULL]
{
   // locals
   TCHAR sBuff[SM_TBLOCK_SIZE] ;
   smos_WStrCpy(sBuff, SM_TBLOCK_SIZE, file_name) ;

   // for debugging - Place break points here as:
   if(bAssertBreak)             // <== break here for any failed SM_ASSERT() calls including SM_ASSERT_BREAK() calls 
     { bAssertBreak = TRUE ; }  // <== break here for just failed SM_ASSERT_BREAK() calls

   // Break here just for failed SM_ASSERT and SM_ASSERT_BREAK calls from a particular file.
   // Change the quoted string to the filename of interest.
   // Example: "SmBrep" to break on SM_ASSERT and SM_ASSERT_BREAK calls from files SmBrep.[h,cpp]
   if(smos_WStrStr(sBuff, _T("Vector")))
     {
       // a place for failed SM_ASSERT and SM_ASSERT_BREAK breakpoints from files containing the above quoted string
       smos_ErrorMessage(error, sBuff, line_num, message, messageForFile,func_name) ;
     }
   else
     {
       // a place for failed SM_ASSERT and SM_ASSERT_BREAK breakpoints from files not containing the above quoted string
       smos_ErrorMessage(error, sBuff, line_num, message, messageForFile,func_name) ;
     }

} // end smos_AssertErrorMessage

/*******************************************************************//**
PURPOSE:

NOTES: Arrive here from failed SER() macro calls
***********************************************************************/
void smos_ErrorMessage
  (SmStatus            error,          // in :
   const TCHAR * const file_name,      // in :
   ULONG               line_num,       // in :
   const TCHAR * const message,        // in :
   const TCHAR * const messageForFile, // in :
   const TCHAR * const func_name)      // in :

{
  if( c3d_smlib_error_handler != NULL ) 
    {
        c3d_smlib_error_handler(error, file_name, line_num, messageForFile ? messageForFile : message);
        return;
    }

  smos_InvokeErrorCallback(error, file_name, line_num, message); 

  // pretty print errors and messages
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  TCHAR sBuffForFile[SM_TBLOCK_SIZE] ;
  TCHAR sLabel[SM_TBLOCK_SIZE] ;

  SmBoolean bToLog = FALSE ;

  // error line label (and bToLog state, gwc:maybe warning and messages should be logged too?)
  //cbi: Leaving the comment lines out entirely.  Maybe use a different criterion.
  if     (error == SM_ERR_WARNING)
                                               { smos_sprintf(sLabel, _T("%.1024s"), _T("Warning") );            bToLog = FALSE ; }
  else if(error == SM_ERR_MESSAGE)
                                               { smos_sprintf(sLabel, _T("%.1024s"), _T("Message") );            bToLog = FALSE ; }
  else if(error == SM_ERR_ASSERT_FAILURE)
                                               { smos_sprintf(sLabel, _T("%.1024s"), _T("Assert Error") );       bToLog = TRUE ; }
  else if(error == SM_ERR_ASSERTVALID_FAILURE)
                                               { smos_sprintf(sLabel, _T("%.1024s"), _T("AssertValid Error") );  bToLog = TRUE ; }
  else
                                               { smos_sprintf(sLabel, _T("Error #%6ld"), error ); bToLog = TRUE ; }

  // Error Line
  if (file_name) 
    {
      // Same output for now
      if (bToLog)
        { smos_sprintf( sBuff, _T( "\n%s(%6ld)" ), file_name, line_num ); }
        // { smos_sprintf(sBuff, _T("\n// %s(%6ld)"), file_name, line_num); }
      else
        { smos_sprintf(sBuff, _T("\n%s(%6ld)"), file_name, line_num); }

      if (b_OutputThin)    { smos_sprintf(sBuffForFile, _T("\n%.512s()"), file_name); }
    }

  smos_WriteBuffer(sBuff, (b_OutputThin) ? sBuffForFile : NULL) ;

  // func_name
  if (func_name) 
    {
      smos_sprintf(sBuff, _T(": in:[%.512s()]"), func_name);
      smos_WriteBuffer(sBuff);
    }

  // label
  if (file_name) 
    {
      smos_sprintf(sBuff, _T(": %.512s\n"), sLabel);

      if (b_OutputThin)
        {
          smos_sprintf(sBuffForFile, _T(": %.512s\n"), sLabel);
        }
    }
  else  // without file_name 
    {
        smos_sprintf(sBuff, _T("\n%.512s -- \n"), sLabel);
      if (b_OutputThin)
        {
          smos_sprintf(sBuffForFile, _T("\n%.512s -- \n"), sLabel);
        }
    }
  smos_WriteBuffer(sBuff, (b_OutputThin) ? sBuffForFile : NULL) ;

  // add a message
  if (message && messageForFile)
    {
      smos_sprintf(sBuff,       _T("         Message=%.512s\n"),message);
      smos_sprintf(sBuffForFile,_T("         Message=%.512s\n"),messageForFile);
      smos_WriteBuffer(sBuff, sBuffForFile) ;
    }
  else if (message)
    {
      smos_sprintf(sBuff,_T("         Message=%.512s\n"),message);
      smos_WriteBuffer(sBuff, sBuff);
    }

} // end smos_ErrorMessage

/*******************************************************************//**
PURPOSE: Write a debug history line out to the screen.
NOTES: Called by the macro SMS_CHANGE when compiled
                with the SM_DEBUG_TRACE compile time constant
***********************************************************************/
#ifdef SM_DEBUG_TRACE
void smos_PrintChange               // eff: add debug history line to FILE *fp
  (const TCHAR *initials,           // in : change author's ID
   const TCHAR *version,            // in : version number in which change is made
   const TCHAR *comment,            // in : short explanation for the change
   const TCHAR *file,               // in : file name for change call
   int line,                        // in : line number for change call
   FILE *fp)                        // in : target output stream
// modifies: fp, adds a line to the output stream
// effects : adds a formatted error message to the output stream
{

  // print a change notification
  SM_FPRINTF( fp, _T("\n *** %s:DEBUG [%s::%s:%s] %s\n"),
           initials,
           version,
           file,
           line,
           comment) ;

  // flush the stream buffer
  fflush( fp );

} // end smos_PrintChange

#endif // SM_DEBUG_TRACE

/*******************************************************************//**
PURPOSE: Write out a string and a double.

NOTES:
***********************************************************************/
void smos_WriteDouble
  (const TCHAR* buff,              // in : write this string
   double dValue)                 // in : then write this value
{
    smos_WriteBuffer(buff);
    TCHAR sBuff[SM_TBLOCK_SIZE];
    smos_sprintf(sBuff,_T("%16.16lf"),dValue);
    smos_WriteBuffer(sBuff);

} // end smos_WriteDouble

/*******************************************************************//**
PURPOSE: Write out a string and a double.

NOTES:
***********************************************************************/
void smos_WriteLong
  (const TCHAR* buff,          // in : write this string
   ULONG lValue)               // in : then write this value
{
  smos_WriteBuffer(buff);
  TCHAR sBuff[SM_TBLOCK_SIZE];
  smos_sprintf(sBuff,_T(" %ld\n"),lValue);
  smos_WriteBuffer(sBuff);

} // end smos_WriteLong

/*******************************************************************//**
PURPOSE: Write out a string to an appropriate place depending upon
   the platform.

NOTES:
***********************************************************************/
void smos_WriteBuffer
( const TCHAR* pBuff,             // in : output string sent to stderr also echoed to stdout when buffForFile == NULL
  const TCHAR* pBuffForFile,      // in : When not NULL string sent to stdout instead of pBuff, default NULL
  const TCHAR* pBuffForDebugLog)  // in : When not NULL string sent to stdout instead of pBuff, default NULL
{
  // check input
  SmBoolean bInput = (pBuff || pBuffForFile || pBuffForDebugLog) ;

  // no work - no input
  if(bInput == FALSE) { return ; }

  // Write to the debugger on Windows when debug code is enabled.
  // windows.h, which declares OutputDebugString, is included only then.
  #if defined(_WIN32) && defined(_DEBUG) && defined(SM_DEBUG_CODE)
    if(pBuff)                 { OutputDebugString( (LPCTSTR)pBuff ); }
    else if (pBuffForFile)    { OutputDebugString( (LPCTSTR)pBuffForFile ); }
    else if(pBuffForDebugLog) { OutputDebugString( (LPCTSTR)pBuffForDebugLog ); }
  #endif // _WIN32 && _DEBUG && SM_DEBUG_CODE

  // Write to stderr when not on Windows (Linux, macOS) and in debug
  #if !defined(_WIN32) && defined(SM_DEBUG_CODE)
    if(pBuff)                 { SM_FPRINTF( stderr, _T( "%s" ), pBuff ); }
    else if (pBuffForFile)    { SM_FPRINTF( stderr, _T( "%s" ), pBuffForFile ); }
    else if(pBuffForDebugLog) { SM_FPRINTF( stderr, _T( "%s" ), pBuffForDebugLog ); }
    fflush( stderr );
  #endif // no _WIN32 && SM_DEBUG_CODE

  // when writing to OutputLong file - Echo pBuff messages to OutputLong file and if pBuff is NULL try pBuffForFile
  if(b_OutputLong && bInput)
    {
        // Direct stdout to sOutputFile
        smos_ReDirectStdOutToFile( sOutputFile );
    
        // if pBuff exists write it to file buffer  - else try pBuffForFile 
        if     (pBuff)        { SM_PRINTF( pBuff ); }
        else if(pBuffForFile) { SM_PRINTF( pBuffForFile ); }

        // // when given always write pBuffForDebugLog to file buffer
        // if(pBuffForDebugLog) { SM_PRINTF( pBuffForDebugLog ); }

        // flush file buffer
        fflush( stdout );
    } // end Output LongFile check

  // when writing to OutputThin file - Echo pBuffForFile messages to OutputThin file and if pBuffForFile is NULL try pBuff
  if(b_OutputThin && bInput)
    {
        // Direct stdout to sOutputFileThin
        smos_ReDirectStdOutToFile( sOutputFileThin );
    
        // if pBuffForFile exists write it to file buffer  - else try pBuff 
        if     (pBuffForFile) { SM_PRINTF( pBuffForFile ); }
        else if(pBuff)        { SM_PRINTF( pBuff ); }

        // // when given always write pBuffForDebugLog to file buffer
        // if(pBuffForDebugLog) { SM_PRINTF( pBuffForDebugLog ); }

        // flush file buffer
        fflush( stdout );
    } // end Output thin file check

  // when writing to OutputDebugLog file - Echo pBuffForDebugLog messages to OutputDebugLog file
  if(b_OutputDebugLog && pBuffForDebugLog)
    {
        smos_ReDirectStdOutToFile( sOutputFileDebugLog );
    
        if(pBuffForDebugLog) { SM_PRINTF( pBuffForDebugLog ); }
        fflush( stdout );
    } // end Output DebugLog check

} // end smos_WriteBuffer

/*******************************************************************//**
PURPOSE: Print out duration of process

NOTES:
***********************************************************************/
void sm_PrintTime
  (const TCHAR* pString,     // in : output this string
   clock_t sStart,           // in : then output duration = sFinish - sStart
   clock_t sFinish)          // in :
{
    TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

    double duration = (double)(sFinish-sStart) / CLOCKS_PER_SEC;
    smos_sprintf(sBuff,       _T("%s %lf\n"), pString, duration);
    smos_sprintf(sBuffForFile,_T("%s %lf\n"), pString, floor(duration));

    smos_WriteBuffer(sBuff, sBuffForFile);

} // end sm_PrintTime

/*******************************************************************//**
PURPOSE: Send smos_WriteBuffer stdout output to new empty file

NOTES:
  1. creates and opens filestreams for all FileNames which are longer than 0 length
     - when file already exists - empties existing file
     - when file does not exist - create empty file
  2. Saves NonZeroLength FileNames 
      if(pFileName)         sFileName         = pFileName ;         
      if(pFileNameThin)     sFileNameThin     = pFileNameThin ;     
      if(pFileNameDebugLog) sFileNameDebugLog = pFileNameDebugLog ; 
  3. Redirects stdout to one of the open filestreams as
      if(pFileName)              sFileName         = pFileName ;         stdout => pFileStream ;         return pFileStream
      else if(pFileNameThin)     sFileNameThin     = pFileNameThin ;     stdout => pFileThinStream ;     return pFileThinStream
      else if(pFileNameDebugLog) sFileNameDebugLog = pFileNameDebugLog ; stdout => pFileDebugLogStream ; return pFileDebugLogStream
      else                                                                                               return stdout  

***********************************************************************/
FILE *smos_DirectStdOutToFile
 (TCHAR *pFileName,           // in : target file for future stdout writes
  TCHAR *pFileNameThin,       // in : default:[NULL], optional 2nd path name to save for FileThin, NULL to ignore 
  TCHAR *pFileNameDebugLog)   // in : default:[NULL], optional 3rd path name to save for FileDebugLog, NULL to ignore
{
  // return value
  FILE *pStream = NULL ;

  // process order of file names is important - the last processed file wins.
  //    redirects stdout to last processed FileName
  //    returns last opened FileStream
  
  // when pFileNameDebugLog is given - open file for write - save pFileNameDebugLog for later redirection
  if(pFileNameDebugLog && SM_STRLEN(pFileNameDebugLog) > 0 )
    {
      pStream = SM_FROPEN(pFileNameDebugLog, _T("w"), stdout);
      if (pStream == NULL) { exit(-1); }
  
      // save filename in static global so we can redirect later
      smos_sprintf(sOutputFileDebugLog, _T("%s"), pFileNameDebugLog);

      // Send smos_WriteBuffer stdout output to append to existing file
      smos_ReDirectStdOutToFile(pFileNameDebugLog);

    } // end given pFileName branch
  
  // when pFileNameThin is given - open file for write - save pFileNameThin for later redirection
  if(pFileNameThin && SM_STRLEN(pFileNameThin) > 0 )
    {
      pStream = SM_FROPEN(pFileNameThin, _T("w"), stdout);
      if (pStream == NULL) { exit(-1); }
  
      // save filename in static global so we can redirect later
      smos_sprintf(sOutputFileThin, _T("%s"), pFileNameThin);

      // Send smos_WriteBuffer stdout output to append to existing file
      smos_ReDirectStdOutToFile(pFileNameThin);

    } // end given pFileName branch
  
  // when pFileName is given - open file for write - save pFileName for later redirection
  if(pFileName && SM_STRLEN(pFileName) > 0 )
    {
      pStream = SM_FROPEN(pFileName, _T("w"), stdout);
      if (pStream == NULL) { exit(-1); }
  
      // save filename in static global so we can redirect later
      smos_sprintf(sOutputFile, _T("%s"), pFileName);

      // Send smos_WriteBuffer stdout output to append to existing file
      smos_ReDirectStdOutToFile(pFileName);

    } // end given pFileName branch

  // all done
  return(pStream ? pStream : stdout);

} // end smos_DirectStdOutToFile

/*******************************************************************/ /**
 PURPOSE: Send smos_WriteBuffer stdout output to new empty file

 NOTES:
   This version of smos_DirectStdOutToFile takes std::strin as input
   converts to TCHAR so the existing smos_DirectStdOutToFile can be used
   Temporary: Eventually smos_DirectStdOutToFile will use standard strings

 ***********************************************************************/
FILE* smos_DirectStdOutToFile
(
    std::string* pFileNameStr,        // in : target file for future stdout writes
    std::string* pFileNameThinStr,    // in : default:[NULL], optional 2nd path name to save for FileThin, NULL to ignore
    std::string* pFileNameDebugLogStr // in : default:[NULL], optional 3rd path name to save for FileDebugLog, NULL to ignore
)
{
    // std::string -> TCHAR for the existing smos_DirectStdOutToFile (TEMP).
    // smos_ToTChar decodes UTF-8 and picks the TCHAR width.
    const TCHAR* pFileName = NULL;
    std::basic_string<TCHAR> sName;
    if (pFileNameStr && pFileNameStr->size() > 0)
    {
        sName = smos_ToTChar(pFileNameStr->c_str());
        pFileName = sName.c_str();
    }

    const TCHAR* pFileNameThin = NULL;
    std::basic_string<TCHAR> sNameThin;
    if (pFileNameThinStr && pFileNameThinStr->size() > 0)
    {
        sNameThin = smos_ToTChar(pFileNameThinStr->c_str());
        pFileNameThin = sNameThin.c_str();
    }

    const TCHAR* pFileNameDebugLog = NULL;
    std::basic_string<TCHAR> sNameLog;
    if (pFileNameDebugLogStr)
    {
        sNameLog = smos_ToTChar(pFileNameDebugLogStr->c_str());
        pFileNameDebugLog = sNameLog.c_str();
    }

    return (smos_DirectStdOutToFile(const_cast<TCHAR*> (pFileName), const_cast<TCHAR*> (pFileNameThin), const_cast<TCHAR*> (pFileNameDebugLog)));

} // end smos_DirectStdOutToFile

/*******************************************************************//**
PURPOSE: Send smos_WriteBuffer stdout output to new empty file

NOTES: Create file when it doesn't exist, else empties
  existing file leaving it ready for future writes.
***********************************************************************/
FILE *smos_DirectGeneratedToFile( TCHAR *pFileName )      // in : Path name to save, NULL to ignore, default:[NULL]
{
  FILE *pStream = NULL;
  if (pFileName && SM_STRLEN(pFileName) > 0 )
    {
      pStream = SM_FROPEN(pFileName, _T("w"), stdout);
      if (pStream == NULL) { exit(-1); }
 
      // save filename in static global so we can redirect later
      smos_sprintf(sOutputFile, _T("%s"), pFileName);
    }
 
  return(pStream);

} // end smos_DirectGeneratedToFile

/*******************************************************************//**
PURPOSE: Send smos_WriteBuffer stdout output to append to existing file

NOTES: Opens file for append when it exists, else creates
       file leaving it ready for future writes.
***********************************************************************/
FILE *smos_ReDirectStdOutToFile(TCHAR *pFileName)   // in : target file for future stdout writes
{
  FILE *pStream = SM_FROPEN(pFileName, _T("a+"), stdout);

  if(pStream == NULL)
    { exit(-1); }
  
  return(pStream);
  
} // end smos_ReDirectStdOutToFile

/*******************************************************************//**
PURPOSE: Direct stdout back to the console stopping stdout from
         writing to any files it may have been targeting.

NOTES:
***********************************************************************/
void smos_RestoreStdOut
  (FILE *pStream)        // out: NOT USED 
{
  size_t lFileCnt         = SM_STRLEN(sOutputFile) ;
  size_t lFileThinCnt     = SM_STRLEN(sOutputFileThin) ;
  size_t lFileDebugLogCnt = SM_STRLEN(sOutputFileDebugLog) ;

  // when global file names are nonempty - stdout has probably been redirected - reset it console
  if(   lFileCnt         > 0 
     || lFileThinCnt     > 0
     || lFileDebugLogCnt > 0)
    {
      // redirect stdout to the "CON"sole for write
      pStream = SM_FROPEN(_T("CON"), _T("w"), stdout);

      // We are still using sOutputFile to write combined output - do not empty
      //smos_sprintf(sOutputFile, _T("")) ;
      //smos_sprintf(sOutputFileThin, _T("")) ;
      //smos_sprintf(sOutputFileDebugLog, _T("")) ;
    }

} // end smos_RestoreStdOut

/*******************************************************************//**
PURPOSE: class dependent sm_ItemDump() methods

NOTES: only compiled for debug code
***********************************************************************/
#ifdef SM_DEBUG_CODE

  // SmVector3d, SmVector2d, SmGfxVertex
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmVector3d  & rPnt) { smos_sprintf(buff,_T("\t[%3ld] = [%16.16lf, %16.16lf, %16.16lf]\n"),i,rPnt.x,rPnt.y,rPnt.z); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmVector3d  * pPnt) { smos_sprintf(buff,_T("\t[%3ld] = [%16.16lf, %16.16lf, %16.16lf]\n"),i,pPnt->x,pPnt->y,pPnt->z); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmVector2d  & rPnt) { smos_sprintf(buff,_T("\t[%3ld] = [%16.16lf, %16.16lf]\n"),i,rPnt.x,rPnt.y); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmVector2d  * pPnt) { smos_sprintf(buff,_T("\t[%3ld] = [%16.16lf, %16.16lf]\n"),i,pPnt->x,pPnt->y); }
  
  // SmExtent1d, SmExtent2d, SmExtent3d
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmExtent1d & rIvl)  { smos_sprintf(buff,_T("\t[%3ld] = [%16.16lf, %16.16lf]\n"),i,rIvl.GetMin(),rIvl.GetMax()); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmExtent1d * pIvl)  { smos_sprintf(buff,_T("\t[%3ld] = [%16.16lf, %16.16lf]\n"),i,pIvl->GetMin(),pIvl->GetMax()); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmExtent2d & rIvl)  { smos_sprintf(buff,_T("\t[%3ld] = [%16.16lf, %16.16lf] x [%16.16lf, %16.16lf]\n"),i,rIvl.GetUMin(),rIvl.GetUMax(),rIvl.GetVMin(),rIvl.GetVMax()); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmExtent2d * pIvl)  { smos_sprintf(buff,_T("\t[%3ld] = [%16.16lf, %16.16lf] x [%16.16lf, %16.16lf]\n"),i,pIvl->GetUMin(),pIvl->GetUMax(),pIvl->GetVMin(),pIvl->GetVMax()); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmExtent3d & rIvl)  { smos_sprintf(buff,_T("\t[%3ld] = [%16.16lf, %16.16lf] x [%16.16lf, %16.16lf] x [%16.16lf, %16.16lf]\n"),
                                                                                 i,rIvl.GetUMin(),rIvl.GetUMax(),rIvl.GetVMin(),rIvl.GetVMax(),rIvl.GetWMin(),rIvl.GetWMax()); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmExtent3d * pIvl)  { smos_sprintf(buff,_T("\t[%3ld] = [%16.16lf, %16.16lf] x [%16.16lf, %16.16lf] x [%16.16lf, %16.16lf]\n"),
                                                                                 i,pIvl->GetUMin(),pIvl->GetUMax(),pIvl->GetVMin(),pIvl->GetVMax(),pIvl->GetWMin(),pIvl->GetWMax()); }
  // SmBrep, SmRegion, SmShell
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmBrep   * pBrep)   { smos_sprintf(buff,_T("\t[%3ld] = 0x%p, %s\n"),i,pBrep  ,pBrep  ->GetTypeString()) ; }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmRegion * pRegion) { smos_sprintf(buff,_T("\t[%3ld] = 0x%p, %s\n"),i,pRegion,pRegion->GetTypeString()) ; }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmShell  * pShell)  { smos_sprintf(buff,_T("\t[%3ld] = 0x%p, %s\n"),i,pShell ,pShell ->GetTypeString()) ; }
                                                                                                  
  // SmFace, SmEdge, SmVertex                                                                       
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmFace   * pFace)   { smos_sprintf(buff,_T("\t[%3ld] = 0x%p, %s\n"),i,pFace  ,pFace  ->GetTypeString()) ; }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmEdge   * pEdge)   { smos_sprintf(buff,_T("\t[%3ld] = 0x%p, %s\n"),i,pEdge  ,pEdge  ->GetTypeString()) ; }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmVertex * pVertex) { smos_sprintf(buff,_T("\t[%3ld] = 0x%p, %s\n"),i,pVertex,pVertex->GetTypeString()) ; }

  // SmIterationValue
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmIterationValue * pIVal) { smos_sprintf(buff,_T("\t[%3ld] = T:[%16.16lf] FOfT:[%16.16lf] FPrimeOfT:[%16.16lf] PrevStep:[%16.16lf]\n"),i,pIVal->m_dT,pIVal->m_dFOfT,pIVal->m_dFPrimeOfT,pIVal->m_dPrevStep); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmIterationValue & rIVal) { smos_sprintf(buff,_T("\t[%3ld] = T:[%16.16lf] FOfT:[%16.16lf] FPrimeOfT:[%16.16lf] PrevStep:[%16.16lf]\n"),i,rIVal.m_dT,rIVal.m_dFOfT,rIVal.m_dFPrimeOfT,rIVal.m_dPrevStep); }

  // SmGapSample
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmGapSample * pGSmp) { smos_sprintf(buff,_T("\t[%3ld] = GapDropType:[%3d], Len:[%16.16lf], Params:[%16.16lf, %16.16lf]\n"),i,pGSmp->GetGapDropType(),pGSmp->GetLength(),pGSmp->GetThisParam(0),pGSmp->GetOtherParam(0)); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmGapSample & rGSmp) { smos_sprintf(buff,_T("\t[%3ld] = GapDropType:[%3d], Len:[%16.16lf], Params:[%16.16lf, %16.16lf]\n"),i,rGSmp.GetGapDropType() ,rGSmp.GetLength() ,rGSmp.GetThisParam(0) ,rGSmp.GetOtherParam(0) ); }

  // SmLocalInterval
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmLocalInterval * pLIvl) { smos_sprintf(buff,_T("\t[%3ld] = Min:[Len:[%16.16lf], Params:[%16.16lf, %16.16lf]], Max:[Len:[%16.16lf], Params:[%16.16lf, %16.16lf]]\n"),
    i,pLIvl->GetMin().GetLength(),pLIvl->GetMin().GetThisParam(0),pLIvl->GetMin().GetOtherParam(0),
      pLIvl->GetMax().GetLength(),pLIvl->GetMax().GetThisParam(0),pLIvl->GetMax().GetOtherParam(0)); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmLocalInterval & rLIvl) { smos_sprintf(buff,_T("\t[%3ld] = Min:[Len:[%16.16lf], Params:[%16.16lf, %16.16lf]], Max:[Len:[%16.16lf], Params:[%16.16lf, %16.16lf]]\n"),
    i,rLIvl. GetMin().GetLength(),rLIvl. GetMin().GetThisParam(0),rLIvl. GetMin().GetOtherParam(0),
      rLIvl. GetMax().GetLength(),rLIvl. GetMax().GetThisParam(0),rLIvl. GetMax().GetOtherParam(0)); }

  // SmGfxVertex, SmGfxNVertex, SmGfxColoredVertex, SmGfxTexturedVertex
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxVertex & rPnt)  { smos_sprintf(buff,_T("\t[%3ld] = [%16.16lf, %16.16lf, %16.16lf]\n"),i,rPnt.m_fX,rPnt.m_fY,rPnt.m_fZ); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxVertex * pPnt)  { smos_sprintf(buff,_T("\t[%3ld] = [%16.16lf, %16.16lf, %16.16lf]\n"),i,pPnt->m_fX,pPnt->m_fY,pPnt->m_fZ); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxNVertex & rPnt) { smos_sprintf(buff,_T("\t[%3ld] = P:[%16.16lf, %16.16lf, %16.16lf] N:[%16.16lf, %16.16lf, %16.16lf]\n"),i,rPnt. m_fX,rPnt. m_fY,rPnt. m_fZ,rPnt. m_fNX,rPnt. m_fNY,rPnt. m_fNZ); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxNVertex * pPnt) { smos_sprintf(buff,_T("\t[%3ld] = P:[%16.16lf, %16.16lf, %16.16lf] N:[%16.16lf, %16.16lf, %16.16lf]\n"),i,pPnt->m_fX,pPnt->m_fY,pPnt->m_fZ,pPnt->m_fNX,pPnt->m_fNY,pPnt->m_fNZ); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxColoredVertex & rPnt) { smos_sprintf(buff,_T("\t[%3ld] = P:[%16.16lf, %16.16lf, %16.16lf] N:[%16.16lf, %16.16lf, %16.16lf] C:[%16.16lf, %16.16lf, %16.16lf]\n"),i,rPnt. m_fX,rPnt. m_fY,rPnt. m_fZ,rPnt. m_fNX,rPnt. m_fNY,rPnt. m_fNZ,rPnt. m_fR,rPnt. m_fG,rPnt. m_fB); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxColoredVertex * pPnt) { smos_sprintf(buff,_T("\t[%3ld] = P:[%16.16lf, %16.16lf, %16.16lf] N:[%16.16lf, %16.16lf, %16.16lf] C:[%16.16lf, %16.16lf, %16.16lf]\n"),i,pPnt->m_fX,pPnt->m_fY,pPnt->m_fZ,pPnt->m_fNX,pPnt->m_fNY,pPnt->m_fNZ,pPnt->m_fR,pPnt->m_fG,pPnt->m_fB); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxTexturedVertex & rPnt) { smos_sprintf(buff,_T("\t[%3ld] = P:[%16.16lf, %16.16lf, %16.16lf] N:[%16.16lf, %16.16lf, %16.16lf] UV:[%16.16lf, %16.16lf]\n"),i,rPnt. m_fX,rPnt. m_fY,rPnt. m_fZ,rPnt. m_fNX,rPnt. m_fNY,rPnt. m_fNZ,rPnt. m_fU,rPnt. m_fV); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmGfxTexturedVertex * pPnt) { smos_sprintf(buff,_T("\t[%3ld] = P:[%16.16lf, %16.16lf, %16.16lf] N:[%16.16lf, %16.16lf, %16.16lf] UV:[%16.16lf, %16.16lf]\n"),i,pPnt->m_fX,pPnt->m_fY,pPnt->m_fZ,pPnt->m_fNX,pPnt->m_fNY,pPnt->m_fNZ,pPnt->m_fU,pPnt->m_fV); }

  // SmDisplayList
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmDisplayList * pDLst) { smos_sprintf(buff,_T("\t[%3ld] = ID:[%3d]\n"),i,pDLst->GetDisplayListId()); }
  void sm_ItemDump(TCHAR * buff, ULONG i, const SmDisplayList & rDLst) { smos_sprintf(buff,_T("\t[%3ld] = ID:[%3d]\n"),i,rDLst. GetDisplayListId()); }


#endif // SM_DEBUG_CODE
