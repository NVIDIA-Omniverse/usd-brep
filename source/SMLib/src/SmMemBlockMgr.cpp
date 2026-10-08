// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmMemBlockMgr.cpp
* PURPOSE: Source file for memory block manager
**********************************************************************/

#include "StdAfx.h"

#include <SmMemBlockMgr.h>

SmArenaMemBlockMgr::SmArenaMemBlockMgr()
{
  Reset();
}

SmArenaMemBlockMgr::~SmArenaMemBlockMgr()
{
  Reset();
}

void SmArenaMemBlockMgr::Initialize(std::size_t initialBlockSize)
{
  Reset();
  m_initialBlockSize = ALIGN_SIZE(initialBlockSize);
  Reserve(initialBlockSize);
}

std::size_t SmArenaMemBlockMgr::NewBlockSize(std::size_t bytes) const
{
  SM_ASSERT(m_initialBlockSize > 0); // Error: Initialize() has not been called

  if (m_lastBlockSize == 0) // for the first block
  {
    if (bytes <= m_initialBlockSize)
    {
      return m_initialBlockSize;
    }
    else
    {
      return ALIGN_SIZE(bytes);
    }
  }
  else
  {
    if (bytes <= m_lastBlockSize)
    {
      return 2 * m_lastBlockSize;
    }
    else
    {
      return ALIGN_SIZE(bytes);
    }
  }
}

void* SmArenaMemBlockMgr::Allocate(std::size_t bytes)
{
  Reserve(bytes);

  void* ret = (void*)&(m_blocks.GetLast()[m_nextIndexInBlock]);
  m_nextIndexInBlock += bytes;
  return ret;
}

void SmArenaMemBlockMgr::Reserve(std::size_t bytes)
{
  SM_ASSERT(m_initialBlockSize > 0); // Error: Initialize() has not been called

  if (m_nextIndexInBlock + bytes > m_lastBlockSize)
  {
    m_lastBlockSize = NewBlockSize(bytes);
    m_blocks.Add((char*) smos_Calloc(m_lastBlockSize, 1));
    m_nextIndexInBlock = 0;
    m_totalSize += m_lastBlockSize;
  }
}

std::size_t SmArenaMemBlockMgr::Size() const
{
  return m_totalSize;
}

void SmArenaMemBlockMgr::Reset()
{
  m_initialBlockSize = 0;
  m_lastBlockSize = 0;
  m_nextIndexInBlock = 0;
  m_totalSize = 0;
  for (char* p : m_blocks)
  {
    smos_Free((void*) p);
  }
  m_blocks.RemoveAll();
}
