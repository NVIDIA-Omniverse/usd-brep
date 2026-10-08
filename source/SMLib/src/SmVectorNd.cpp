// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmVectorNd.cpp
* PURPOSE: Implementation of SmVectorNd methods
**********************************************************************/

 
#include "StdAfx.h"

#include <SmVectorNd.h>

void SmVectorNd::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE]; 
  SmBoolean bUnInit = FALSE ;
                                  
  SM_SPRINTF(sBuff,_T("%s"),_T("Vec-Nd ["));

  smos_WriteBuffer(sBuff);                 
  for (ULONG i=0; i<GetSize(); i++) 
    {
      if((*this)[i] == SM_UNDEF_DOUBLE) 
        { bUnInit = TRUE ;
          SM_SPRINTF(sBuff,_T("%s"),_T("UNINITIALIZED"));
        } 
      else
        SM_SPRINTF(sBuff,_T("%16.16lf,"),(*this)[i]);
      smos_WriteBuffer(sBuff);
    }
  SM_ASSERT_BREAK(bUnInit == FALSE) ;

  SM_SPRINTF(sBuff,_T("%s"),_T("]"));
  smos_WriteBuffer(sBuff);

} // end SmVectorNd::Dump

