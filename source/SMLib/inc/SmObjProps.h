// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmVertexProps.h
* PURPOSE: Header file for SmEdgeProps class.
**********************************************************************/

#ifndef __SMOBJPROPSH__
#define __SMOBJPROPSH__


#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif // no __SMOS_TYPES_H__

/*******************************************************************//**
PURPOSE: Base class for the SmObjProps derived class hierarchy

NOTES: leave as a simple container class
***********************************************************************/
class SM_EXPORT SmObjProps
{
 public:
  // a Base class for managing collections of SmObjProps
  
  // Set in SmHealData::Fix_BackPointers(): Face degeneracy bit
  SmBooleanUL   m_bBadBackPointer = UNSURE;  // link: SmHealData::m_sBadObjProps_BackPointers (note: SmBooleanUL = booleans declared as ULONG)
                                             // set : [Fix_BackPointers()]
                                             // use : TRUE = Objects found through forward connect pointers that fail to point back to the forward object
                                             //       FALSE = never had a bad backpointer or backpointers have been fixed
                                             //       if backpointers have been fixed obj will be in the m_sBadObjProps_BackPointers list
  virtual ~SmObjProps() { }
 public:
  // defines void Dump()
  SM_COMMON_BASE(SmObjProps, SmObjProps_TYPE) ;

} ; // end class SmObjProps

SM_TARRAY_TEMPLATE_PREDECLARATION(SmObjProps*);


#endif // !__SMOBJPROPSH__
