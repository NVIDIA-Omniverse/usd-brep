// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmGraphicsNLib.h
* PURPOSE: Interface to display list based graphics.
**********************************************************************/

#ifndef __SMGFX_NLIB_H__
#define __SMGFX_NLIB_H__

#include <SmTypes.h>
#include <SmCoreTypes.h>
#include <NL_Nurbsdef.h>

SM_EXPORT void smgfx_DrawNPt( NL_POINT *pnt );
SM_EXPORT void smgfx_DrawNPts( int n, NL_POINT *pnts );

SM_EXPORT void smgfx_DrawLine( SmPoint3d *P0, SmPoint3d *P1 );
SM_EXPORT void smgfx_DrawTri( SmPoint3d *P0, SmPoint3d *P1, SmPoint3d *P2 );
SM_EXPORT void smgfx_DrawNTri( NL_POINT *pnt1, NL_POINT *pnt2, NL_POINT *pnt3 );

SM_EXPORT void smgfx_DrawNCrv( NL_CURVE *crv );
SM_EXPORT void smgfx_DrawNCrvs( int ncrv, NL_CURVE **crvs );
SM_EXPORT void smgfx_DrawCrossTangentNCrv(NL_CURVE *pCrossTangCur, NL_CURVE *pBaseCur) ;

SM_EXPORT void smgfx_DrawNSrf( NL_SURFACE *srf );


#endif // !__SMGFX_NLIB_H__

