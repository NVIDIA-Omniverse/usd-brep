// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME: SmTypes.cpp
* PURPOSE: Error handling functions
**********************************************************************/

#include "StdAfx.h"
#include <SmTypes.h>

#if defined(_UNICODE) && defined(_WIN32)
  #include <windows.h>   // MultiByte/WideChar conversions for the wide TCHAR paths
#endif

/*******************************************************************//**
PURPOSE: Narrow (UTF-8) C string -> build-width TCHAR string.

NOTES: UTF-8 -> UTF-16 on _UNICODE, narrow copy otherwise.
***********************************************************************/
std::basic_string<TCHAR> smos_ToTChar(const char* pNarrowUtf8)
{
    if (pNarrowUtf8 == NULL || pNarrowUtf8[0] == '\0') { return std::basic_string<TCHAR>(); }

#ifdef _UNICODE
    // Decode UTF-8 -> UTF-16 (flags 0 = best-effort, as the old SM_TCONVERT did).
    // -1 length includes the terminator, which is trimmed off the result.
    int len = MultiByteToWideChar(CP_UTF8, 0, pNarrowUtf8, -1, NULL, 0);
    if (len <= 0) { return std::wstring(); }
    std::wstring sWide(static_cast<size_t>(len), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, pNarrowUtf8, -1, &sWide[0], len);
    if (!sWide.empty() && sWide.back() == L'\0') { sWide.pop_back(); }
    return sWide;
#else
    return std::basic_string<TCHAR>(pNarrowUtf8);
#endif // no _UNICODE

} // end smos_ToTChar

/*******************************************************************//**
PURPOSE: Build-width TCHAR C string -> narrow (UTF-8) string.

NOTES: UTF-16 -> UTF-8 on _UNICODE, narrow copy otherwise.  Inverse of
       smos_ToTChar().
***********************************************************************/
std::string smos_FromTChar(const TCHAR* pTChar)
{
    if (pTChar == NULL || pTChar[0] == _T('\0')) { return std::string(); }

#ifdef _UNICODE
    // Encode UTF-16 -> UTF-8 (flags 0 = best-effort, as smos_ToTChar does).
    // -1 length includes the terminator, which is trimmed off the result.
    int len = WideCharToMultiByte(CP_UTF8, 0, pTChar, -1, NULL, 0, NULL, NULL);
    if (len <= 0) { return std::string(); }
    std::string sNarrow(static_cast<size_t>(len), '\0');
    WideCharToMultiByte(CP_UTF8, 0, pTChar, -1, &sNarrow[0], len, NULL, NULL);
    if (!sNarrow.empty() && sNarrow.back() == '\0') { sNarrow.pop_back(); }
    return sNarrow;
#else
    return std::string(pTChar);
#endif // no _UNICODE

} // end smos_FromTChar

/*******************************************************************//**
PURPOSE: Pretty Print SmCrvDirOnFaceType

NOTES:
***********************************************************************/
void DumpCrvDirOnFaceType
 (const TCHAR *pOptLabel,            // in : optional Character string label, NULL to ignore
  SmCrvDirOnFaceType eCrvDirOnFace)  // in : Target eCrvDirOnFace to pretty print
{
  TCHAR sBuff[64] ;
  if(pOptLabel) { smos_WriteBuffer(pOptLabel) ; } 
  smos_snprintf(sBuff, 64, _T("[%s]"),   eCrvDirOnFace == SM_CD_UNINIT         ? _T("UNINIT")         
                                       : eCrvDirOnFace == SM_CD_NONE           ? _T("NONE")           
                                       : eCrvDirOnFace == SM_CD_IN             ? _T("IN")             
                                       : eCrvDirOnFace == SM_CD_ON             ? _T("ON")             
                                       : eCrvDirOnFace == SM_CD_OUT            ? _T("OUT")            
                                       : eCrvDirOnFace == SM_CD_LAMINA         ? _T("LAMINA")         
                                       : eCrvDirOnFace == SM_CD_SEAM           ? _T("SEAM")           
                                       : eCrvDirOnFace == SM_CD_POLE           ? _T("POLE")           
                                       : eCrvDirOnFace == SM_CD_POLE_NO_VERTEX ? _T("POLE_NO_VERTEX") 
                                       :                                         _T("UnexpectedType")) ;  
  smos_WriteBuffer(sBuff) ;
} // end DumpCrvDirOnFaceType
