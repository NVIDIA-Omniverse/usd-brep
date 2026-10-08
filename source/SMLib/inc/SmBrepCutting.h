// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmBrepCutting.h
* PURPOSE: Header file for SmBrepCutting object.
**********************************************************************/

#ifndef __SMBREPCUTTING_H__
#define __SMBREPCUTTING_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMCUTTER_H__
#include <SmCutter.h>
#endif

/*******************************************************************//**
PURPOSE: The Brep Cutting object provides the ability to cut away
    portions of a Brep using a cutter.  The cutter is typically a 
    plane, other surface or another Brep.

NOTES: 
***********************************************************************/
class SM_EXPORT SmBrepCutting
{
private:
    SmBrep   * m_pBrepToCut;   // Brep to be cut
    SmCutter * m_pCutter;      // Object which defines cutting tool

public:
    SmBrepCutting(SmBrep *pBrepToCut);

    ~SmBrepCutting() {};

    SmStatus DoCut
    (
      SmCutter           * pCutter,               ///< [in] :     <br>
      double               d3DTolerance,          ///< [in] :     <br>
      SmBoolean            bRemoveCutPortions,    ///< [in] :     <br>
      SmTArray<SmFace *> * pOptSubsetFaces = NULL ///< [out]:     <br>
    );
               
    SmBrep   * GetBrepToCut() { return m_pBrepToCut ; }
    SmCutter * GetCutter   () { return m_pCutter ; }

} ; // end class SmBrepCutting

#endif // !__SMBREPCUTTING_H__


