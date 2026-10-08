// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmSurfaces.cpp

PURPOSE: 
    Contains popular high level "C" type functions that create 
    primitive objects (cube, sphere, etc).

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SmPrimitiveCreation
**********************************************************************/

#include "StdAfx.h"

#include "SmApiSurfaces.h"
#include "SmApiGeneral.h"
#include <SmApiTypes.h>
#include "SmBSplineSurface.h"
#include "SmSurface.h"
#include "SmPrimitiveCreation.h"
#include <SmMapPtrToPtr.h>

#if SM_DEBUG_CODE
static const SmVector3d s_kBlack(0, 0, 0);
static const SmVector3d s_kBlue (0, 0, 1);
static const SmVector3d s_kRed  (1, 0, 0);
#endif

/*******************************************************************//**
PURPOSE ---   

NOTES --- 

***********************************************************************/
SmApiStatus SmApiCreateSurface
( 
    const SmTArray<SmPoint3d> & rControlPoints,   ///< [in ]: List of control points
    ULONG lNumRows,                         ///< [in ]: Number of rows
    ULONG lNumColumns,                      ///< [in ]: Number of columns
    SmSurface*&  rpResult                  ///< [out]: Resulting SmSurface
)
{
    
    ULONG nControlPts = rControlPoints.GetSize();
    if( lNumRows * lNumColumns != nControlPts )
        return( SM_ERR_INVALID_INPUT );

#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( rControlPoints, &s_kBlack, 1, 1 );
    }
#endif

    ULONG lDegreeU = 3;
    ULONG lDegreeV = 3;

    // A bicubic needs lDegree+1 control points in each direction; fewer would
    // underflow the knot counts below.
    if( lNumRows < lDegreeU + 1 || lNumColumns < lDegreeV + 1 )
        return( SM_ERR_INVALID_INPUT );

    ULONG lNumKnotsU = lNumRows - lDegreeU + 1;
    ULONG lNumKnotsV = lNumColumns - lDegreeV + 1;

    ULONG ii = 0;
    SmTArray<double> sKnotsU;
    sKnotsU.Add( 0.0 );
    for( ii = 1; ii < lNumKnotsU - 1; ii++ ) {
        sKnotsU.Add( ((double)ii)/(lNumKnotsU-1) );
    }
    sKnotsU.Add(1.0);

    SmTArray<double> sKnotsV;
    sKnotsV.Add( 0.0 );
    for( ii = 1; ii < lNumKnotsV - 1; ii++ ) {
        sKnotsV.Add( (double)ii/(lNumKnotsV-1) );
    }
    sKnotsV.Add(1.0);

    SmTArray<ULONG> aMultiplicitiesU;
    aMultiplicitiesU.Add(lDegreeU + 1 );
    for( ii = 1; ii < lNumKnotsU - 1; ii++ ) {
        aMultiplicitiesU.Add(1);
    }
    aMultiplicitiesU.Add(lDegreeU + 1 );

    SmTArray<ULONG> aMultiplicitiesV;
    aMultiplicitiesV.Add(lDegreeV + 1 );
    for( ii = 1; ii < lNumKnotsV - 1; ii++ ) {
        aMultiplicitiesV.Add(1);
    }
    aMultiplicitiesV.Add(lDegreeV + 1 );


    SmBSplineSurface* pNewSurface = NULL;
    SmApiStatus stat = SmBSplineSurface::CreateCanonical( *SmApiGetOrCreateContext(), lDegreeU, lDegreeV, rControlPoints, SM_SF_UNSPECIFIED,
                                                       aMultiplicitiesU, aMultiplicitiesV, sKnotsU, sKnotsV,
                                                       SM_KT_UNSPECIFIED, NULL, NULL, pNewSurface );

    if( stat != SM_SUCCESS )
        return( stat );

    rpResult = pNewSurface;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif

    return( SM_SUCCESS );

} // end SmApiCreateSurface

/*******************************************************************//**
PURPOSE ---   

NOTES --- 

***********************************************************************/
SmApiStatus SmApiCreateSurfaceFromCornerPoints
(
    const SmPoint3d& crCornerUminVmin,     ///< [in ]:
    const SmPoint3d& crCornerUmaxVmin,     ///< [in ]:
    const SmPoint3d& crCornerUminVmax,     ///< [in ]:
    const SmPoint3d& crCornerUmaxVmax,     ///< [in ]:
    SmSurface*& rpResult                  ///< [out]: Resulting SmSurface
)
{
 
#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( crCornerUminVmin, &s_kRed, 1, 0 );
        SmApiDraw( crCornerUmaxVmin, &s_kRed, 0, 0 );
        SmApiDraw( crCornerUminVmax, &s_kRed, 0, 0 );
        SmApiDraw( crCornerUmaxVmax, &s_kRed, 0, 0 );
    }
#endif

    rpResult = NULL;
    SmBSplineSurface* pNewSurface = NULL;
    SmApiStatus stat = SmBSplineSurface::CreateBilinearSurface( *SmApiGetOrCreateContext(), crCornerUminVmin, crCornerUmaxVmin, 
                                                                        crCornerUminVmax, crCornerUmaxVmax, pNewSurface);

    if( stat != SM_SUCCESS || pNewSurface == NULL )
        return ( stat != SM_SUCCESS ) ? stat : SM_ERR;

    rpResult = pNewSurface;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif



    return( SM_SUCCESS );

} // end SmApiCreateSurfaceFromCornerPoints

/*******************************************************************//**
PURPOSE ---   

NOTES --- 

***********************************************************************/
SmApiStatus SmApiCreateSurfaceFromOrderedPoints
( 
    const SmTArray<SmPoint3d> & rPoints,   ///< [in ]: points ordered: P[row][col] = p[row*lNumCols + col]
    ULONG lNumRows,                  ///< [in ]: Number of rows
    ULONG lNumColumns,               ///< [in ]: Number of columns
    SmSurface*& rpResult            ///< [out]: Resulting SmSurface
)
{

#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( rPoints, &s_kBlack, 1, 1 );
    }
#endif

    ULONG lDegreeU = 3;
    ULONG lDegreeV = 3;

    ULONG lNumPts = rPoints.GetSize();
    if( lNumRows * lNumColumns != lNumPts )
        return( SM_ERR_INVALID_INPUT );

    SmBSplineSurface* pNewSurface = NULL;
    SmApiStatus stat = SmBSplineSurface::ApproximatePoints( *SmApiGetOrCreateContext(), rPoints, lNumRows, lNumColumns, 
                                                         lDegreeU, lDegreeV, NULL, pNewSurface, true );

    if( stat != SM_SUCCESS )
        return( stat );

    rpResult = pNewSurface;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif

    return( SM_SUCCESS );

} // end SmApiCreateSurfaceFromPoints


/*******************************************************************//**
PURPOSE ---   

NOTES --- 

***********************************************************************/
SmApiStatus SmApiCreateSurfaceFromRandomPoints
( 
    const SmTArray<SmPoint3d> & rPoints,  ///< [in ]: Random points
    SmSurface*&  rpResult          ///< [out]: Resulting SmSurface
)
{
 #if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( rPoints, &s_kBlack, 1, 1 );
    }
#endif
    
    ULONG lNumPts = rPoints.GetSize();
    ULONG lMaxControlPtsU = lNumPts / 2;
    ULONG lMaxControlPtsV = lNumPts / 2;
    ULONG lDegreeU = 3;
    ULONG lDegreeV = 3;

    SmBSplineSurface* pNewSurface = NULL;
    SmApiStatus stat = SmBSplineSurface::ApproximateRandomPoints( *SmApiGetOrCreateContext(), rPoints, lMaxControlPtsU, lMaxControlPtsV, 
                                                          lDegreeU, lDegreeV, NULL, 0, pNewSurface );

    if( stat != SM_SUCCESS )
        return( stat );

    rpResult = pNewSurface;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif

    return( SM_SUCCESS );

} // end SmApiCreateSurfaceFromRandomPoints

/*******************************************************************//**
PURPOSE ---   

NOTES --- 

***********************************************************************/
SmApiStatus SmApiCreateExtrude
( 
    SmBSplineCurve* pCurve,          ///< [in ]: Curve to sweep/extrude
    const SmVector3d & rSweepVector,       ///< [in ]: Sweep vector
    SmSurface*& rpResult            ///< [out]: Resulting SmSurface
)
{
#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( pCurve, &s_kBlack, 1, 1 );
        SmApiDraw( rSweepVector, &s_kBlack, 0, 0 );
    }
#endif

    rpResult = NULL;
    SmApiStatus stat = SM_SUCCESS;
    SmBSplineSurface* pNewSurface = NULL;
    stat = SmBSplineSurface::CreateLinearSweep(*SmApiGetOrCreateContext(), *pCurve, rSweepVector, pNewSurface);

    if( stat != SM_SUCCESS || pNewSurface == NULL )
        return ( stat != SM_SUCCESS ) ? stat : SM_ERR;

    rpResult = pNewSurface;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif

    return( SM_SUCCESS );

} // end SmApiCreateExtrude

/*******************************************************************//**
PURPOSE ---   

NOTES --- 

***********************************************************************/
SmApiStatus SmApiCreateSweep
( 
    SmBSplineCurve* pCurve,          ///< [in ]: Curve to sweep/extrude
    SmBSplineCurve* pPath,           ///< [in ]: Path curve
    SmSurface*& rpResult            ///< [out]: Resulting SmSurface
)
{
#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( pCurve, &s_kBlack, 1, 1 );
        SmApiDraw( pPath, &s_kBlack, 0, 0 );
    }
#endif

    rpResult = NULL;
    SmApiStatus stat = SM_SUCCESS;
    SmBSplineSurface* pNewSurface = NULL;
    double dTol = 1.0e-5;

    stat = SmBSplineSurface::CreateSweepSurface(*SmApiGetOrCreateContext(), pCurve, pPath, NULL, FALSE, dTol, pNewSurface);

    if( stat != SM_SUCCESS || pNewSurface == NULL )
        return ( stat != SM_SUCCESS ) ? stat : SM_ERR;

    rpResult = pNewSurface;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif

    return( SM_SUCCESS );

} // end SmApiCreateSweep

/*******************************************************************//**
PURPOSE ---   

NOTES --- 

***********************************************************************/
SmApiStatus SmApiCreateRuledSurface
( 
    SmBSplineCurve* rCurve1,         ///< [in ]: 
    SmBSplineCurve* rCurve2,         ///< [in ]: 
    SmSurface*&  rpResult           ///< [out]: Resulting SmSurface
)
{
#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( rCurve1, &s_kBlack, 1, 1 );
        SmApiDraw( rCurve1, &s_kBlack, 0, 0 );
    }
#endif

    rpResult = NULL;
    SmBSplineSurface* pNewSurface = NULL;
    SmApiStatus stat = SmBSplineSurface::CreateRuledSurface( *SmApiGetOrCreateContext(), *rCurve1, *rCurve2, SM_SP_UNKNOWN, pNewSurface );

    if( stat != SM_SUCCESS || pNewSurface == NULL )
        return ( stat != SM_SUCCESS ) ? stat : SM_ERR;

    rpResult = pNewSurface;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif

    return( SM_SUCCESS );


} // end SmApiCreateRuledSurface


/*******************************************************************//**
PURPOSE ---   

NOTES --- 

***********************************************************************/
SmApiStatus SmApiCreateSurfaceRevolution
( 
    SmBSplineCurve* rCurve,          ///< [in ]: Curve to revolve
    const SmPoint3d& rOrigin,              ///< [in ]: Base point of axis of revolution
    const SmVector3d& rAxisDir,            ///< [in ]: Axis of revolution
    double dAngleDeg,                ///< [in ]: Angle of revolution
    SmSurface*& rpResult            ///< [out]: Resulting SmSurface
)
{
#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( rCurve, &s_kBlack, 1, 1 );
        SmApiDraw( rOrigin, rAxisDir, &s_kRed, 0, 0 );
    }
#endif

    rpResult = NULL;
    if( rCurve == NULL )
        return SM_ERR_INVALID_INPUT;

    SmContext* pContext = SmApiGetOrCreateContext();

    // Revolve a copy of the input: CreateRotationalSweep consumes the curve,
    // stripping the NURB out of the caller's SmBSplineCurve otherwise.
    SmCurve* pCurveCopy = NULL;
    SmStatus copyStat = ((SmCurve*)rCurve)->Copy( *pContext, pCurveCopy );
    if( copyStat != SM_SUCCESS || pCurveCopy == NULL )
        return SM_ERR;

    SmTArray<SmCurve*>sCurvesToSweep;
    sCurvesToSweep.Add( pCurveCopy );

    SmBrep* pBrep = new (*pContext) SmBrep();

    SmPrimitiveCreation pPrimitive( pBrep->GetInfiniteRegion(), 0, true );
    SmApiStatus stat = pPrimitive.CreateRotationalSweep( sCurvesToSweep, rOrigin, rAxisDir, dAngleDeg, 1, false );

    if( stat == SM_SUCCESS ) {
        SmTArray<SmSurface*> sSurfaces;
        pBrep->GetSurfaces( sSurfaces );
        if( sSurfaces.GetSize() == 0 )
            stat = SM_ERR;
        else {
            rpResult = sSurfaces[0];
            rpResult->SetOwner(NULL);
        }
    }

    delete pBrep;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif

    return( stat );

} // end SmApiCreateSurfaceRevolution

/*******************************************************************//**
PURPOSE ---   

NOTES ---   
***********************************************************************/
SmApiStatus SmApiCreateSkin
(
    const SmTArray<SmBSplineCurve*>& rProfiles,  ///< [in ]: Profile curves                                              <br>
    SmSurface*&  rpResult                 ///< [out]: Resulting SmSurface                                         <br>
)
{
    if( rProfiles.GetSize() < 2 )
        return SM_ERR_INVALID_INPUT;

    const SmContext& ctx = *SmApiGetOrCreateContext();

    SmBSplineSurface* pSrf = NULL;
    SmBSplineSurface* pDerivSurf[2] = { NULL, NULL };
    SmStatus stat = SmBSplineSurface::CreateSkinnedSurface(
        ctx,
        rProfiles,
        FALSE,             // bSyncronized
        SM_SP_U,           // cross-section curves become U isoparameters
        0.0,               // interpolate (not approximate)
        NULL,              // no rail 1
        NULL,              // no rail 2
        FALSE,             // rail1 not used as spine
        NULL,              // pIntersectionsWithRails
        pDerivSurf,        // derivative surfaces
        pSrf );

    delete pDerivSurf[0];
    delete pDerivSurf[1];
    pDerivSurf[0] = NULL;
    pDerivSurf[1] = NULL;

    if( stat != SM_SUCCESS || pSrf == NULL )
    {
        rpResult = NULL;
        return ( stat != SM_SUCCESS ) ? stat : SM_ERR;
    }

    rpResult = pSrf;

    return SM_SUCCESS;

} // End SmApiCreateSkin

/*******************************************************************//**
PURPOSE --- Offset a given surface  

NOTES ---   Alt: SmOffsetSurface::CreateOffsetSurface
***********************************************************************/
SmApiStatus SmApiCreateOffsetSurface
( 
    SmSurface* pSrfToOffset,             ///< [in ]: Surface to offset
    double dOffsetDistance,              ///< [in ]: Distance of offset ( positive: normal dir of input surface 
                                         //                           negative: opp normal dir of input surface )
    SmSurface*&  rpResult               ///< [out]: Resulting SmBSplineSurface
)
{
    rpResult = NULL;
    if( pSrfToOffset == NULL )
        return SM_ERR_INVALID_INPUT;

#if SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
    if( bDebugMe  ) {
        SmApiDraw( pSrfToOffset, &s_kBlack, 1, 1 );
    }
#endif

    // Determine tolerance as a funciton of size of input curve
    double dApproxTol = 0.001;
    SmStatus stat = pSrfToOffset->CreateOffsetSurface(*SmApiGetOrCreateContext(), dOffsetDistance, dApproxTol, rpResult);

    if( stat != SM_SUCCESS || rpResult == NULL ) {
        rpResult = NULL;
        return ( stat != SM_SUCCESS ) ? stat : SM_ERR;
    }

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif

    return( SM_SUCCESS );

} // end SmApiCreateOffsetSurface

/*******************************************************************//**
PURPOSE --- Evaluate a given surface for position

NOTES ---  
***********************************************************************/
SmApiStatus SmApiEvaluateSurfacePoint
( 
    const SmSurface* pSurface,                  ///< [in ]: Surface to evaluate
    const SmVector2d& crUV,                     ///< [in ]: Parameter (uv) at which to evaluate
    SmVector3d& crPoint                   ///< [out]: Resulting Point at this parameter
)
{
    if( pSurface == NULL )
        return( SM_ERR_INVALID_INPUT );

    // Negated in-range test so a NaN parameter is rejected too.
    SmExtent2d domain = pSurface->GetNaturalUVDomain();
    if( !( crUV.x >= domain.GetMin().x && crUV.x <= domain.GetMax().x  &&
           crUV.y >= domain.GetMin().y && crUV.y <= domain.GetMax().y ) )
        return( SM_ERR_INVALID_INPUT );

    // Evaluate into a local so a kernel failure leaves the output untouched.
    SmVector3d sPoint;
    SmStatus stat = pSurface->EvaluatePoint( crUV, sPoint );
    if( stat == SM_SUCCESS )
        crPoint = sPoint;
    return stat;

} // End SmApiEvaluateSurfacePoint


/*******************************************************************//**
PURPOSE --- Evaluate a given surface to obtain the normal

NOTES ---  
***********************************************************************/
SMAPI_EXPORT SmApiStatus SmApiEvaluateSurfaceNormal
( 
    const SmSurface* pSurface,                  ///< [in ]: Surface to evaluate
    const SmVector2d& crUV,                     ///< [in ]: Parameter (uv) at which to evaluate
    SmVector3d& crNormal                  ///< [out]: Surface Normal at parameter
)
{
    if( pSurface == NULL )
        return( SM_ERR_INVALID_INPUT );

    // Negated in-range test so a NaN parameter is rejected too.
    SmExtent2d domain = pSurface->GetNaturalUVDomain();
    if( !( crUV.x >= domain.GetMin().x && crUV.x <= domain.GetMax().x  &&
           crUV.y >= domain.GetMin().y && crUV.y <= domain.GetMax().y ) )
        return( SM_ERR_INVALID_INPUT );

    // Evaluate into a local so a kernel failure leaves the output untouched.
    SmVector3d sNormal;
    SmStatus stat = pSurface->EvaluateNormal( crUV, true, true, sNormal );
    if( stat == SM_SUCCESS )
        crNormal = sNormal;
    return stat;

} // End SmApiEvaluateSurfaceNormal

/*******************************************************************//**
PURPOSE --- Evaluate a given surface for position and derivatives

NOTES ---  
***********************************************************************/
SmApiStatus SmApiEvaluateSurfaceDerivatives
( 
    const SmSurface* pSurface,                  ///< [in ]: Surface to evaluate
    const SmVector2d& crUV,                     ///< [in ]: Parameter (uv) at which to evaluate
    SmVector3d& crPoint,                  ///< [out]: Resulting Point at this parameter
    SmVector3d& rDU,                      ///< [out]: Resulting U derivative
    SmVector3d& rDV                       ///< [out]: Resulting V derivative
)
{
    if( pSurface == NULL )
        return( SM_ERR_INVALID_INPUT );

    // Negated in-range test so a NaN parameter is rejected too.
    SmExtent2d domain = pSurface->GetNaturalUVDomain();
    if( !( crUV.x >= domain.GetMin().x && crUV.x <= domain.GetMax().x  &&
           crUV.y >= domain.GetMin().y && crUV.y <= domain.GetMax().y ) )
        return( SM_ERR_INVALID_INPUT );

    // Evaluate into locals so a kernel failure leaves the outputs untouched.
    SmVector3d sPoint, sDU, sDV;
    SmStatus stat = pSurface->Evaluate1stDerivatives( crUV, true, true, sPoint, sDU, sDV );
    if( stat == SM_SUCCESS )
    {
        crPoint = sPoint;
        rDU = sDU;
        rDV = sDV;
    }
    return stat;

} // End SmApiEvaluateSurfaceDerivatives



// This requires modification to just do offset

#if 0

SMAPI_EXPORT int SmConeOffset( int method, double dBaseRadius, double dTopRadius, double dHeight, double dOffset )
{
	SmContext sContext;
	
	SmBSplineSurface* pSrf = NULL;
	SmAxis2Placement sFrame;
	
	switch (method)
	{
		// SmBSplineSurface
		case 0:
		{
			SmBSplineSurface::CreateConePatch(sContext, sFrame, dBaseRadius, dTopRadius, 0.0, 360.0, dHeight, SM_CO_QUADRATIC, pSrf);
		}
			break;

		// SmSurfOfRevolution
		case 1:
		{
			SmVector3d sOrigin(0, 0, 0);
			SmVector3d zAxis(0, 0, dHeight);

			SmVector3d sStartPt(dBaseRadius, 0, 0);
			SmVector3d sEndPt(dTopRadius, 0, dHeight);

			SmLine* pLine = NULL;
			SmLine::CreateLineSegment(sContext, 3, sStartPt, sEndPt, pLine);

			SmSurfOfRevolution* pSrfOfRev = NULL;
			SmSurfOfRevolution::CreateCanonical(sContext, pLine, sOrigin, zAxis, pSrfOfRev);

			pSrf = pSrfOfRev;
		}
		break;

		// SmCone
		case 2:
		{
			SmCone* pCone = NULL;
			SmCone::CreateCanonical(sContext, sFrame, dBaseRadius, dTopRadius, dHeight, pCone);
			pSrf = pCone;
		}

		break;

		default: 
			break;

	}

	SM_ASSERT_VALID(pSrf);

#if SM_DEBUG_CODE
	//SmBoolean bDebugMe = FALSE;
	//if (bDebugMe) {
		SmApiDraw(pSrf, &s_kBlue, 2, 2);
	//}
#endif

	double dApproxTol = 0.0001;
	SmTArray<SmSurface*>pOffSrfs;
	SmApiStatus stat = pSrf->CreateOffsetSurface(sContext, dOffset, dApproxTol, pOffSrfs);

	if (stat == SM_SUCCESS) {
		// Might result in more than one surface
		// But draw only first one to see if we got the primary one

		SM_ASSERT_VALID(pOffSrfs[0]);

#if SM_DEBUG_CODE
		//SmBoolean bDebugMe = FALSE;
		//if (bDebugMe) {
			SmApiDraw(pOffSrfs[0], &s_kRed, 2, 2);
		//}
#endif
		
	}
	else {
		return SM_ERR;
	}

	return stat;
}



#endif
