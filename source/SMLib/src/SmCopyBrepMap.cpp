// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmCopyBrepMap.cpp
* PURPOSE: Source file for SmCopyBrepMap class methods.
**********************************************************************/

/// Visual Leak Detector includes
//#include "C:\Program Files (x86)\Visual Leak Detector\include\vld.h"

#include "StdAfx.h"
#include <SmCopyBrepMap.h>

/*******************************************************************//**
PURPOSE: Add a (Copy Pointer, Orig Pointer) pair entry to
            the SmCopyBrepMap

NOTES:
***********************************************************************/
void SmCopyBrepMap::SetAt
  (SmTopology *pCopyObj,      // in : Copy of orig Topology object
   SmTopology *pOrigObj)      // in : Orig Topology object being copied
{
  SM_ASSERT(pCopyObj != nullptr);
  SM_ASSERT(pOrigObj != nullptr);
  SM_ASSERT(pCopyObj->GetType() == pOrigObj->GetType()) ;

  m_sMap.Insert(pCopyObj, pOrigObj);

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      this->Dump(FALSE) ;
      this->Dump(TRUE) ;
    }
#endif // SM_DEBUG_CODE

} // end SmCopyBrepMap::SetAt

/*******************************************************************//**
PURPOSE: Get an Orig Pointer value given a Copy Pointer value.

NOTES:
***********************************************************************/
SmTopology *SmCopyBrepMap::GetAt
  (SmTopology *pCopyObj)  const    // in : Orig Topology object being copied
{
  if ( pCopyObj == nullptr )
    { return nullptr; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw
  if(bDebugMe)
    {
      this->Dump(FALSE) ;
      this->Dump(TRUE) ;
    }
#endif // SM_DEBUG_CODE

  SmTopology* pOrigObj = m_sMap.At(pCopyObj);

  SM_ASSERT (pOrigObj != nullptr);
  SM_ASSERT (pCopyObj->GetType() == pOrigObj->GetType());

  return pOrigObj;
} // end SmCopyBrepMap::GetValueAt

/*******************************************************************//**
PURPOSE: Pretty print

NOTES:
***********************************************************************/
void SmCopyBrepMap::Dump() const
{
  // pass the call along
  Dump(FALSE) ;

} // end SmCopyBrepMap::Dump

/*******************************************************************//**
PURPOSE: Pretty print

NOTES:
***********************************************************************/
void SmCopyBrepMap::Dump(SmBoolean bFULL) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  // SmCopyBrepMap Pointer
  SM_SPRINTF(sBuff,_T("\nSmCopyBrepMap = 0x%p, Orig/Copy Breps[0x%p, 0x%p] "),
             this,
             m_pOrigBrep,
             m_pCopyBrep);
  SM_SPRINTF(sBuffForFile,_T("\nSmCopyBrepMap = %s, Orig/Copy Breps[%s, %s] "),
             _T("notNULL"),
             m_pOrigBrep ? _T("notNULL") : _T("NULL"),
             m_pCopyBrep ? _T("notNULL") : _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // labels, counts, and maps
  SM_SPRINTF(sBuff,_T("\nm_sMap [items = %lu]"), m_sMap.Count());
  smos_WriteBuffer(sBuff);

  if(bFULL) { m_sMap.Dump() ; }

} // end SmCopyBrepMap::Dump
