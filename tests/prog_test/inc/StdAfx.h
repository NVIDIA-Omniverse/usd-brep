// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// StdAfx.h : include file for standard system include files,
// or project specific include files that are used frequently,
// but are changed infrequently

#ifndef STDAFX_H
#define STDAFX_H

#ifndef _SECURE_ATL
#define _SECURE_ATL 1
#endif 

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN        // Exclude rarely-used stuff from Windows headers
#endif 

// Since afx includes tchar.h, we must undefine _T so we can define it ourselves in SmTypes.h
#ifndef _UNICODE
#undef _T
#define _FN_WIDE 0
#endif

#include <SmSmlibAll.h>

#endif // no STDAFX_H 
