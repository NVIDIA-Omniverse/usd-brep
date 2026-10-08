// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmHashTable.cpp
* PURPOSE: Utility Hash table management.
**********************************************************************/

#include "StdAfx.h"

#include <SmHashTable.h>

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertHashTable_list[] =
{
  /*  0 */ {SM_AT_CACHE, _T("Bad HashTable BucketItem[ii]->BucketIndex"),            _T("HashTable BucketItem BucketIndex points to an unused Bucket - should be in bucket and bucket should be used") },
  /*  1 */ {SM_AT_CACHE, _T("Bad HashTable Total BucketItem count"),                 _T("HashTable Total BucketItem Count is less than Removed BucketItem Count - Should be Total = Used + Removed BucketItem counts") },
  /*  2 */ {SM_AT_CACHE, _T("Bad HashTable Total, Removed, Used BucketItem counts"), _T("HashTable Total, Used, and Removed BucketItem Counts are bad - Should be Total = Used + Removed BucketItem counts") },
  /*  3 */ {SM_AT_CACHE, _T("Bad HashTable Used BucketItem counts"),                 _T("HashTable Counted-Used != StoredUsed BucketItem counts - Should be equal") }
} ;

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertObjsInVoxels_list[] =
{
  /*  0 */ {SM_AT_CACHE, _T("Bad ObjsInVoxels contained m_sHashTable"),                          _T("ObjsInVoxels contained m_sHashTable failed its m_sHashTable->AssertValid() call") },
  /*  1 */ {SM_AT_CACHE, _T("Bad conflicting ObjsInVoxels Voxel and bucket counts"),             _T("ObjsInVoxels voxel count not equal to HashTable bucket count - should be equal") },
  /*  2 */ {SM_AT_CACHE, _T("Bad conflicting ObjsInVoxels PointItems and BucketItems counts"),   _T("ObjsInVoxels point items count not == HashTable total BucketItems count - should be less than or equal") },
  /*  3 */ {SM_AT_CACHE, _T("Bad conflicting BucketItem and PointItem iith elements"),           _T("ObjsInVoxels m_sHashTable.BucketItem[ii]->m_sItem != associated m_sHashTable.m_sPointItems[ii] - should be equal") },
  /*  4 */ {SM_AT_CACHE, _T("Bad ObjsInVoxels m_sPointItems[ii] Bad VoxelIndx"),                 _T("ObjsInVoxels m_sHashTable->BucketItem[ii]->BucketIndex is not within 1 voxel of the VoxelIndex computed from m_sPointItems[ii]->m_sTgtPoint3d") },
  /*  5 */ {SM_AT_CACHE, _T("Bad ObjsInVoxels m_sHashTable.m_sBucketItems[ii] Bad VoxelIndx"),   _T("ObjsInVoxels m_sHashTable.m_sBuckets[m_sHashTable.m_sBucketItems[ii].BucketIndex] does not contain BucketItem[ii]") },
  /*  6 */ {SM_AT_CACHE, _T("Bad Conflicting BucketItem back m_sItem and m_sItemId values"),     _T("ObjsInVoxels &m_sPointItems[m_sHashTable.m_sBucektItems[ii].m_sItemId] != m_sHashTable.m_sBucketItems[ii].m_sItem - they should be the same.") },
  /*  7 */ {SM_AT_CACHE, _T("Bad ObjsInVoxels m_sHashTable.m_sBucketItems[ii].m_sItemId value"), _T("ObjsInVoxels m_sHashTable.m_sBucketItems[ii].m_sItemId index value is not contained in range:[0, m_sPointItems.GetSize()]") },
} ;

#ifndef __SMOS_MEMORY_H__
#include <SmMemory.h>
#endif // __SMOS_MEMORY_H__

// Eliminate linux error (maybe belongs elsewhere?)
#define __int64 long long

