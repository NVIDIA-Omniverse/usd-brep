// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmVertexProps.cpp
* PURPOSE: Source file for implementation of SmEdgeProps methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmConfig.h>
#include <SmObjProps.h>
#include <SmGraphicsOutput.h>

/*******************************************************************//**
PURPOSE: SmObj Base Class Methods

NOTES:
***********************************************************************/

/*******************************************************************//**
PURPOSE: Pretty Print

NOTES:
***********************************************************************/
void SmObjProps::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // Begin
  SM_SPRINTF(sBuff,        _T("\n  Begin SmObjProps:[0x%p] Dump "), this) ;
  SM_SPRINTF(sBuffForFile, _T("\n  Begin SmObjProps:[%s] Dump "), _T("NotNULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // BackPointers - Set in SmHealData::Fix_BackPointers()
  SM_SPRINTF(sBuff,        _T("\n     BackPointers:[ %1lu = %s"),
             m_bBadBackPointer,
               m_bBadBackPointer == TRUE   ? _T("TRUE] Bad")
             : m_bBadBackPointer == FALSE  ? _T("FALSE] Okay")
             : m_bBadBackPointer == UNSURE ? _T("UNSURE] Not Yet Set")
             :                               _T("UnKnown Val] Error - fix bug")) ; 
  smos_WriteBuffer(sBuff) ;

  // all done
  SM_SPRINTF(sBuff,        _T("\n  End SmObjProps:[0x%p] Dump "), this) ;
  SM_SPRINTF(sBuffForFile, _T("\n  End SmObjProps:[%s] Dump "), _T("NotNULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);
  
} // end SmObjProps::Dump

