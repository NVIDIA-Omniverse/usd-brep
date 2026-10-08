// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCubicBezierSurface.h
* PURPOSE: Header file for Offset Surface class.
**********************************************************************/

#ifndef __SMCUBICBEZIERSURFACE_H__
#define __SMCUBICBEZIERSURFACE_H__

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif


/*******************************************************************//**
PURPOSE: This object is fixed size bezier surface structure.  It is one of those
    internal objects used primarily for computation and is not intended
    to live permanently anywhere unless you imbed it in some other object.

NOTES: This is meant to be a light weight extremely efficient
    representation with minimal overhead.

Example:
 Sorted as rows P[0][0], P[0][1], P[0][2], P[0][3],
                P[1][0], P[1][1], ...
                P[2][0], P[2][1], ...
                P[3][0], P[3][1], ...
 Note that the rows represent constant U parameter columns.
 To subdivide in U we would build extra pointers to P[0][0], 
 P[1][0], P[2][0], P[3][0]
***********************************************************************/
class SM_EXPORT SmCubicBezierSurface 
{
private:
    SmPoint3d m_vPoints[16]; // Bezier surface points.

public:
    SmPoint3d *m_vP[4];  // Pointers into Bezier points to make 2 dimensional array

    // Construction
    SmCubicBezierSurface()
    {
      m_vP[0] = &m_vPoints[0]; m_vP[1] = &m_vPoints[4];
      m_vP[2] = &m_vPoints[8]; m_vP[3] = &m_vPoints[12];
    }

    ~SmCubicBezierSurface() {}

    SmStatus CalculateBoundingBox
    (
      SmExtent3d * pNormalBox,          ///< [out]: Axis alligned box
      SmPseudoBox * pPseudoBox,         ///< [out]: Non-axis aligned box
      SmPolarBox  * pPolarBox,                           ///< [out]: Polar box of surface normals  Note this is not used in this method
      SmBoolean     bExpandPosBoxesByZoneTol3d = FALSE)  ///< [in] : TRUE = returned Normal & Pseudo BBoxes = BBox->ExpandAbsoluate(ZoneTol3d)
     const;                                              ///<      : FALSE= returned Normal & Pseudo BBoxes = BBox with no expansion 
    
    SmStatus Split
    (
      SmSurfParamType eSurfParam,       ///< [in] : SM_SP_U splits colunms
      SmPoint3d sSplitPnt,              ///< [in] : Split point on the minimum row or column which corresponds to 1/2 of the
                                        ///<      : parameter domain of the original domain which created this bezier.
      SmVector3d sDU,                   ///<      :
      SmVector3d sDV,                   ///<      :
      SmVector3d sDUV,                  ///<      :
      SmPoint3d sSplitPnt2,             ///<      :
      SmVector3d sDU2,                  ///<      :
      SmVector3d sDV2,                  ///<      :
      SmVector3d sDUV2,                 ///<      :
      SmCubicBezierSurface & rSurf1,    ///<      :
      SmCubicBezierSurface & rSurf2     ///<      :
    ) const;

    SmStatus BuildBilinear
    (
      SmPoint3d sP00,                   ///< [in] :
      SmPoint3d sP10,                   ///< [in] :
      SmPoint3d sP01,                   ///< [in] :
      SmPoint3d sP11                    ///< [in] :
    );

    SmStatus ComputeNetConstants
    (
      double * pdUChordHeight,          ///< [out]:
      double * pdVChordHeight,          ///< [out]:
      double * pdUAngleDeg,             ///< [out]:
      double * pdVAngleDeg              ///< [out]:
    ) const;

    SmStatus GetCorners
    (
      SmPoint3d & rP00,                 ///< [out]:
      SmPoint3d & rP10,                 ///< [out]:
      SmPoint3d & rP11,                 ///< [out]:
      SmPoint3d & rP01                  ///< [out]:
    ) const
    {
      rP00 = m_vP[0][0];
      rP10 = m_vP[0][3];
      rP11 = m_vP[3][3];
      rP01 = m_vP[3][0];
      return SM_SUCCESS;
    }
        
    void Draw() const;
    void Dump() const;

} ; // end class SmCubicBezierSurface

#endif // !__SMCUBICBEZIERSURFACE_H__


