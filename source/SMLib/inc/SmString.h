// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmString.h
* PURPOSE: Interface for string functions.
**********************************************************************/

#ifndef __SMOS_STRING_H__
#define __SMOS_STRING_H__

#include <string.h>

#define SM_TBLOCK_SIZE 1024

#ifdef _UNICODE

#define smos_WStrCmp _tcscmp

// char * smos_StrCpy( char *sTgt, size_t lTgtAllocLen, const char *sSrc );
//#define smos_WStrCpy(a, b) _tcsnccpy(a, b, SM_TBLOCK_SIZE)
void inline smos_WStrCpy(TCHAR * pTgt, size_t lTgtAllocLen, const TCHAR * pSrc)
{
    // Early return for invalid inputs
    if (!pTgt || !pSrc || lTgtAllocLen == 0) {
        return;
    }

    // Calculate safe copy length
    size_t lSrcLen = wcsnlen(pSrc, lTgtAllocLen - 1);
    if (lSrcLen >= lTgtAllocLen) {
        lSrcLen = lTgtAllocLen - 1;
    }

    // Safe copy with null termination
    wmemcpy(pTgt, pSrc, lSrcLen);
    pTgt[lSrcLen] = L'\0';
} // end smos_WStrCpy

// char * smos_StrNCpy( char *sTgt, const char *sSrc, size_t length);
#define smos_WStrNCpy _tcsnccpy

// int smos_StrStr( const char *string1, const char *subString );
#define smos_WStrStr _tcsstr

// size_t smos_StrLen( const char *string );
#define smos_WStrLen(a)    wcsnlen((a), SM_TBLOCK_SIZE)
#define smos_WStrNLen(a,b) wcsnlen((a), (b))

// char * smos_StrCat( char *sTgt, const char *string_to_concatonate );
#define smos_WStrCat _tcscat

// char * smos_StrTok( char *string1, const char *deliminator, char* string_to_compare );
#define smos_WStrTok wcstok

#else // no _UNICODE

// int smos_StrCmp( const char *string1, const char *string2 );
#define smos_WStrCmp strcmp

void inline smos_WStrCpy(char * pTgt, size_t lTgtAllocLen, const char * pSrc)
{
    // Early return for invalid inputs
    if (!pTgt || !pSrc || lTgtAllocLen == 0) {
        return;
    }

    // Calculate safe copy length
    size_t lSrcLen = strnlen(pSrc, lTgtAllocLen - 1);
    if (lSrcLen >= lTgtAllocLen) {
        lSrcLen = lTgtAllocLen - 1;
    }

    // Safe copy with null termination
    memcpy(pTgt, pSrc, lSrcLen);
    pTgt[lSrcLen] = '\0';
} // end smos_WStrCpy

// perhaps we need a conditional compile that switches between wcsnLen/strnlen and wcsncpy/strncpy as needed
//  void inline smos_WStrCpy(TCHAR * pTgt, size_t lTgtAllocLen, const TCHAR * pSrc)
//  {
//    // check state - lTgtAllocLen too long
//    if(lTgtAllocLen > SM_TBLOCK_SIZE)
//      { return ; }
//  
//    // locals
//  << HEAD
//    size_t lSrcLen = wcsnlen(pSrc, SM_TBLOCK_SIZE) ;
//  =======
//    size_t lSrcLen = strnlen(pSrc, SM_TBLOCK_SIZE) ;
//  >> master
//    size_t lCpyLen = ( (lSrcLen < lTgtAllocLen-1  ) ? lSrcLen : lTgtAllocLen-1   ) ;
//  
//    // only when char arrays do not overlap - do the copy
//    if(   (pTgt + lTgtAllocLen < pSrc)
//       || (pSrc + lSrcLen      < pTgt))
//  << HEAD
//      { wcsncpy(pTgt, pSrc, lCpyLen) ;
//  =======
//      { strncpy(pTgt, pSrc, lCpyLen) ;
//  >> master
//        pTgt[lCpyLen] = '\0' ;
//      }
//  } // end smos_WStrCpy

// char * smos_StrNCpy( char *sTgt, const char *sSrc, size_t length);
#define smos_WStrNCpy strncpy

// int smos_StrStr( const char *string1, const char *subString );
#define smos_WStrStr strstr

// size_t smos_StrLen( const char *string );
// perhaps we need a conditional compile that switches between wcsnlen/strnlen
// #define smos_WStrLen(a)    wcsnlen((a), SM_TBLOCK_SIZE)
// #define smos_WStrNLen(a,b) wcsnlen((a), (b))

#define smos_WStrLen(a)    strnlen((a), SM_TBLOCK_SIZE)
#define smos_WStrNLen(a,b) strnlen((a), (b))

// char * smos_StrCat( char *sTgt, const char *string_to_concatonate );
#define smos_WStrCat strcat

// int smos_StrStr( const char *string1, const char *subString );
// #define smos_StrStr strstr

// char * smos_StrTok( char *string1, const char *deliminator, char* string_to_compare );
#define smos_WStrTok strtok

#endif // no _UNICODE


//#else
// int smos_StrCmp( const char *string1, const char *string2 );
//#define smos_WStrCmp strcmp

// char * smos_StrCpy( char *sTgt, const char *sSrc );
//#define smos_WStrCpy strcpy

// char * smos_StrNCpy( char *sTgt, const char *sSrc, size_t length);
//#define smos_WStrNCpy strncpy

// int smos_StrStr( const char *string1, const char *subString );
//#define smos_WStrStr strstr

// size_t smos_StrLen( const char *string );
//#define smos_WStrLen strlen

// char * smos_StrCat( char *sTgt, const char *string_to_concatonate );
//#define smos_WStrCat strcat

//#endif

// int smos_StrCmp( const char *string1, const char *string2 );
//#define smos_StrCmp strcmp

// char * smos_StrCpy( char *sTgt, const char *sSrc );
//#define smos_StrCpy strcpy

// int smos_StrStr( const char *string1, const char *subString );
//#define smos_StrStr strstr

// size_t smos_StrLen( const char *string );
//#define smos_StrLen strlen


#endif // !__SMOS_STRING_H__
