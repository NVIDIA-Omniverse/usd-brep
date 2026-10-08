// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmDllMain.h
* PURPOSE: External header file for DLL entry point function.
**********************************************************************/

#ifndef __SMOS_DLLMAIN_H__
#define __SMOS_DLLMAIN_H__

#ifndef __SMOS_WINDEF_H__
#define __SMOS_WINDEF_H__
#include <windef.h>
#endif // no __SMOS_WINDEF_H__ 

// Main entry point for this DLL - run by system
//  when application or threads start and stop.
BOOL APIENTRY DllMain
  (HINSTANCE hinstDLL,      // in : DLL module handle
   DWORD     fdwReason,     // in : reason called
   LPVOID    lpvReserved);  // in : reserved

#endif  // !__SMOS_DLLMAIN_H__
