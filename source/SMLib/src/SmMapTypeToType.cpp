// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmMapTypeToType.cpp
* PURPOSE: Implementation of SmMapTypeToType methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmMapTypeToType.h>

/*******************************************************************//**
PURPOSE: Allocate one SmPlex object (that contains a LinkPtr
         to the next SmPlex object plus memory for nMax number 
         of cbElement sized objects) and place it at the head of
         the growing LinkedList of SmPlex objects

NOTES: This is old style low level memory management code.
       Allocates block = { 1 SmPlex object
                           + mem block, SizedInBytes:[nMax*cbElement]
                         }
***********************************************************************/
SmPlex * SmPlex::Create
 (SmPlex   *& rpPlexListHead, // i/o: current head of SmPlex linked list
  ULONG       nMax,           // in : number of elements to allocate in SmPlex data block
  ULONG       cbElement)      // in : size of each element in bytes  in SmPlex data block.
                              //      elements must be either base (double, ptr,..) or static class (SmVector3d,..) objects
                              //      because an array of cbElements is allocated using smos_Calloc
{
  SM_ASSERT(nMax > 0 && cbElement > 0);

  // may throw exception
  // only okay to use smos_Calloc on base or static class (SmPlex) objects.
  SmPlex * pPlex = (SmPlex*) smos_Calloc(sizeof(SmPlex) + ((size_t)nMax * (size_t)cbElement),  // in : Num of memory blocks
                                         1) ;                                                  // in : size of each block in bytes
  pPlex->m_pNext = rpPlexListHead;
  rpPlexListHead = pPlex ;  // change head (adds in reverse order for simplicity)
  return pPlex ;

} // end SmPlex::Create

/*******************************************************************//**
PURPOSE: free the memory of a LinkedList of SmPlex objects

NOTES: frees the memory for this SmPlex and all all SmPlex objects
       in the LinkedList of SmPlex objects
***********************************************************************/
void SmPlex::FreeDataChain()     
{
  SmPlex* pPlex = this;
  while (pPlex != NULL)
    {
      unsigned char* bytes = (unsigned char*) pPlex;
      SmPlex* pNextOne = pPlex->m_pNext;
      smos_Free(bytes); bytes = NULL ;
      pPlex = pNextOne;
    }

} // end SmPlex::FreeDataChain
