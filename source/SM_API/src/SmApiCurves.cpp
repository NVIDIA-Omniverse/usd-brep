// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
FILE NAME: SmCurves.cpp

PURPOSE: 
    Contains popular high level "C" type functions that create/modify curves.
    Additional functions exist in SMLib

GENERAL NOTES: 
    High level functions may assume some input parameters 
    for ease of use. For maximum flexibility, related functions 
    can be found in SMLib
**********************************************************************/

#include "StdAfx.h"

#include "SmApiCurves.h"
#include "SmApiGeneral.h"
#include <SmApiTypes.h>
#include <SmAssertArray.h>
#include <SmMapPtrToPtr.h>
#include <SmLine.h>
#include <SmBSplineCurve.h>
#include <SmCurveTypes.h>
#include <SmSurface.h>
#include <SmCircle.h>
#include <SmTrimmingTools.h>
#include <SmMath.h>
#include <SmMessages.h>
#include <cmath>

#if SM_DEBUG_CODE
static const SmVector3d s_kBlack(0, 0, 0);
static const SmVector3d s_kBlue (0, 0, 1);
static const SmVector3d s_kRed  (1, 0, 0);
#endif


/*******************************************************************//**
PURPOSE --- Create a line segment given two points.  

NOTES --- Result is analytical SmLine

***********************************************************************/
SmApiStatus SmApiCreateLineSegment
(
    const SmPoint3d& crStartPt,           ///< [in ]: Start Point
    const SmPoint3d& crEndPt,             ///< [in ]: End Point
    SmLine*&  rpResult                   ///< [out]: Resulting SmLine
)
{

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( crStartPt, &s_kRed, 1, 0 );
        SmApiDraw( crEndPt, &s_kRed, 0, 0 );
    }
#endif // SM_DEBUG_CODE

    SmLine* pLine = NULL;
    SmApiStatus status = SmLine::CreateLineSegment(*SmApiGetOrCreateContext(), 3, crStartPt, crEndPt, pLine);
    if( status != SM_SUCCESS )
        return( status );

    rpResult = pLine;

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( rpResult );
#endif // SM_ASSERT_VALID

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 1, 1 );
    }
#endif // SM_DEBUG_CODE

    return( SM_SUCCESS );

} // end SmApiCreateLineSegment

/*******************************************************************//**
PURPOSE --- Create a line segment given two points.  

NOTES --- Result is analytical SmLine

***********************************************************************/
SmApiStatus SmApiCreateLine
( 
    const SmPoint3d& crStartPt,         ///< [in ]: Start Point
    const SmVector3d& crDirection,      ///< [in ]: Direction vector
    SmLine*&  rpResult           ///< [out]: Resulting SmLine
)
{
    rpResult = NULL;
    if( !SM_IS_VALID_DOUBLE( crDirection.x ) || !SM_IS_VALID_DOUBLE( crDirection.y ) ||
        !SM_IS_VALID_DOUBLE( crDirection.z ) || crDirection.IsZero() )
        return( SM_ERR_INVALID_INPUT );


#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( crStartPt, crDirection, &s_kRed, 1, 0 );
    }
#endif // SM_DEBUG_CODE

    SmLine* pLine = NULL;
    SmApiStatus status = SmLine::CreateCanonical(*SmApiGetOrCreateContext(), crStartPt, crDirection, pLine);
    if( status != SM_SUCCESS )
        return( status );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pLine );
#endif // SM_ASSERT_VALID

    rpResult = pLine;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 1, 1 );
    }
#endif // SM_DEBUG_CODE

    return( SM_SUCCESS );

} // end SmApiCreateLine


/*******************************************************************//**
PURPOSE --- Shared input validation for the SmApiCreateCircle overloads.

NOTES ---   A circle is built in the xy plane centered at crCenterPt with the
            given radius.  Beyond rejecting non-finite inputs, two boundary
            cases must be caught here rather than deep in the kernel (which
            would otherwise return a generic SM_ERR, or worse, a degenerate
            zero-length circle):

              * Radius at or below SM_EFF_ZERO (1e-12): SmCircle::CreateCanonical
                requires radius > SM_EFF_ZERO, so smaller radii are invalid
                input rather than a kernel failure.
              * Large-center / small-radius combinations whose derived extents
                are not representable in double precision, e.g.
                center=(1e18,0,0), radius=1 yields 1e18 + 1 == 1e18, collapsing
                the start point onto the center (radius()==1 but length()==0).

            SM_IS_VALID_DOUBLE accepts the finite range [-1e18, 1e18]
            (SM_BIG_DOUBLE/100); center coordinates and every derived extent
            must lie within it.
***********************************************************************/
// Validates a circle or ellipse: dRadiusX spans the X axis, dRadiusY the Y axis.
static bool SmApiConicInputsValid( const SmPoint3d& crCenterPt, double dRadiusX, double dRadiusY )
{
    if( crCenterPt.IsUndef() )
        return( false );

    // Center coordinates and radius must be valid, in-range doubles.
    if( !SM_IS_VALID_DOUBLE( crCenterPt.x ) ||
        !SM_IS_VALID_DOUBLE( crCenterPt.y ) ||
        !SM_IS_VALID_DOUBLE( crCenterPt.z ) ||
        !SM_IS_VALID_DOUBLE( dRadiusX ) ||
        !SM_IS_VALID_DOUBLE( dRadiusY ) )
        return( false );

    // Explicit minimum radius, matching SmCircle::CreateCanonical.
    if( dRadiusX <= SM_EFF_ZERO || dRadiusY <= SM_EFF_ZERO )
        return( false );

    // Derived extents must be representable and must not collapse onto the
    // center in double precision (which would yield a zero-length circle).
    const double dExtentXHi = crCenterPt.x + dRadiusX;
    const double dExtentXLo = crCenterPt.x - dRadiusX;
    const double dExtentYHi = crCenterPt.y + dRadiusY;
    const double dExtentYLo = crCenterPt.y - dRadiusY;
    if( !SM_IS_VALID_DOUBLE( dExtentXHi ) || !SM_IS_VALID_DOUBLE( dExtentXLo ) ||
        !SM_IS_VALID_DOUBLE( dExtentYHi ) || !SM_IS_VALID_DOUBLE( dExtentYLo ) )
        return( false );
    if( dExtentXHi == crCenterPt.x || dExtentXLo == crCenterPt.x ||
        dExtentYHi == crCenterPt.y || dExtentYLo == crCenterPt.y )
        return( false );

    return( true );
}

static bool SmApiCircleInputsValid( const SmPoint3d& crCenterPt, double dRadius )
{
    return SmApiConicInputsValid( crCenterPt, dRadius, dRadius );
}


/*******************************************************************//**
PURPOSE --- Create a bspline circle.  

NOTES ---   Assume circle is oriented on the x and y plane
            See associated SmCreateCircle for SmCircle
***********************************************************************/
SmApiStatus SmApiCreateCircle
(
    const SmPoint3d& crCenterPt, ///< [in ]: Start Point
    double dRadius,              ///< [in ]: Radius
    SmBSplineCurve*&  rpResult  ///< [out]: Resulting SmBSplineCurve
)
{
    // rpResult is only valid on SM_SUCCESS; guarantee a deterministic null on
    // every failure path.  Reject non-finite/out-of-range center, radius at or
    // below SM_EFF_ZERO, and center/radius combinations whose extents collapse
    // (see SmApiCircleInputsValid) so bad input cannot yield a degenerate circle.
    rpResult = NULL;
    if( !SmApiCircleInputsValid( crCenterPt, dRadius ) )
        return( SM_ERR_INVALID_INPUT );

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( crCenterPt, &s_kRed, 1, 0 );
    }
#endif // SM_DEBUG_CODE

    SmVector3d xAxis( 1,0,0 );
    SmVector3d yAxis( 0,1,0 );
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crCenterPt, xAxis, yAxis );

    SmBSplineCurve* pCircle = NULL;
    SmApiStatus status = SmBSplineCurve::CreateCircleSegment( *SmApiGetOrCreateContext(), 3, sRefFrame, dRadius, 0.0, 360.0, SM_CO_QUADRATIC, pCircle );
    if( status != SM_SUCCESS )
        return( status );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pCircle );
#endif // SM_ASSERT_VALID

    rpResult = pCircle;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif // SM_DEBUG_CODE

    return( SM_SUCCESS );

} // end SmApiCreateCircle


/*******************************************************************//**
PURPOSE --- Create an analytic circle.  

NOTES ---   Assume circle is oriented on the x and y plane
            See associated SmCreateCircle for SmBSplineCurve
***********************************************************************/
SmApiStatus SmApiCreateCircle
(
    const SmPoint3d& crCenterPt, ///< [in ]: Start Point
    double dRadius,              ///< [in ]: Radius
    SmCircle*&  rpResult        ///< [out]: Resulting SmCircle
)
{
    // rpResult is only valid on SM_SUCCESS; guarantee a deterministic null on
    // every failure path.  Reject non-finite/out-of-range center, radius at or
    // below SM_EFF_ZERO, and center/radius combinations whose extents collapse
    // (see SmApiCircleInputsValid) so bad input cannot yield a degenerate circle.
    rpResult = NULL;
    if( !SmApiCircleInputsValid( crCenterPt, dRadius ) )
        return( SM_ERR_INVALID_INPUT );

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe ) {
        SmApiDraw( crCenterPt, &s_kRed, 1, 0 );
    }
#endif // SM_DEBUG_CODE

    SmVector3d xAxis( 1,0,0 );
    SmVector3d yAxis( 0,1,0 );
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crCenterPt, xAxis, yAxis );

    SmCircle* pCircle = NULL;
    SmApiStatus status = SmCircle::CreateCanonical(*SmApiGetOrCreateContext(), sRefFrame, dRadius, pCircle);
    if( status != SM_SUCCESS )
        return( status );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pCircle );
#endif // SM_ASSERT_VALID

    rpResult = pCircle;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif // SM_DEBUG_CODE

    return( SM_SUCCESS );

} // end SmApiCreateCircle


/*******************************************************************//**
PURPOSE --- Create an analytic ellipse.  

NOTES ---  
***********************************************************************/
SmApiStatus SmApiCreateEllipse
( 
    const SmPoint3d& crOrigin,         ///< [in ]: Origin of the ellipse
    double dRadius1,             ///< [in ]: Dist from origin to ellipse circumference in XAxis direction
    double dRadius2,             ///< [in ]: Dist from origin to ellipse circumference in YAxis direction
    SmEllipse*&  rpResult       ///< [out]: Resulting SmEllipse
)        

{
    rpResult = NULL;
    if( !SmApiConicInputsValid( crOrigin, dRadius1, dRadius2 ) )
        return( SM_ERR_INVALID_INPUT );

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe ) {
        SmApiDraw( crOrigin, &s_kRed, 1, 0 );
    }
#endif // SM_DEBUG_CODE

    SmVector3d xAxis( 1,0,0 );
    SmVector3d yAxis( 0,1,0 );
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, xAxis, yAxis );

    SmEllipse* pEllipse = NULL;
    SmApiStatus status = SmEllipse::CreateCanonical(*SmApiGetOrCreateContext(), sRefFrame, dRadius1, dRadius2, pEllipse);
    if( status != SM_SUCCESS )
        return( status );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pEllipse );
#endif // SM_ASSERT_VALID

    rpResult = pEllipse;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif // SM_DEBUG_CODE

    return( SM_SUCCESS );

} // end SmApiCreateEllipse

/*******************************************************************//**
PURPOSE --- Create an bspline arc.  

NOTES ---   Assume arc is oriented on the x and y plane
            A similar function could create an SmCircle given three points
            A similar function could create an SmCircle with domain interval
***********************************************************************/
SmApiStatus SmApiCreateArc
( 
    const SmPoint3d& rCenterPt,        ///< [in ]: Start Point
    double dRadius,              ///< [in ]: Radius
    double dStartAngleDeg,       ///< [in ]: Start Angle in Degrees
    double dEndAngleDeg,         ///< [in ]: End Angle in Degrees
    SmBSplineCurve*&  rpResult  ///< [out]: Resulting SmBSplineCurve
)     
{

 #if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( rCenterPt, &s_kRed, 1, 0 );
    }
#endif // SM_DEBUG_CODE
    
    SmVector3d xAxis( 1,0,0 );
    SmVector3d yAxis( 0,1,0 );
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( rCenterPt, xAxis, yAxis );

    SmBSplineCurve* pArc = NULL;
    SmApiStatus status = SmBSplineCurve::CreateCircleSegment( *SmApiGetOrCreateContext(), 3, sRefFrame, dRadius, dStartAngleDeg, dEndAngleDeg, SM_CO_QUADRATIC, pArc );
    if( status != SM_SUCCESS )
        return( status );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pArc );
#endif // SM_ASSERT_VALID

    rpResult = pArc;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif // SM_DEBUG_CODE

    return( SM_SUCCESS );

} // end SmApiCreateArc

/*******************************************************************//**
PURPOSE --- Create a bspline helix.

NOTES ---   The helix winds about the axis through crOrigin parallel to +Z,
            spanning from crOrigin at the base to a distance dHeight along that
            axis, and starts on the +X side. The radius is linearly interpolated
            along the axis from dRadiusStart at the base to dRadiusEnd at the far
            end: equal radii give a constant-radius helix, unequal radii a conical
            helix.

            The full public input domain is validated here, before the reference
            frame is built and before SmBSplineCurve::CreateHelixSegment enters
            the legacy spiral fitter (N_CrvApproxSpiral). The fitter accepts
            +Inf, seeds its span count with a float->int conversion (undefined
            for +Inf, and a huge allocation / hang for very large finite turns),
            and only weakly guards degenerate radii, so every out-of-domain value
            must be rejected up front. On any invalid input this returns
            SM_ERR_INVALID_INPUT with rpResult == NULL. Valid domain:
              - crOrigin finite (all components)
              - dHeight      in [1e-7, 1e18]
              - dRadiusStart in [1e-7, 1e18]   (zero/negative rejected)
              - dRadiusEnd   in [1e-7, 1e18]   (zero/negative rejected)
              - dNumTurns    in [1e-7, 124.25] (upper bound = (NL_CCPLIM-6)/8,
                                                the fitter's control-point cap)
              - dTolerance   in [1e-6, 1e18]
              - crOrigin.z +/- dHeight and crOrigin.x/y +/- max(radius) finite

            dTolerance is a hard contract, not best effort. After the fit, the
            true maximum 3D deviation of the curve from the analytic helix is
            measured independently (dense sampling; each curve point paired with
            the helix point at the same axial height -- the full-3D
            generalization of the fitter's radial-only, single-midpoint check,
            and a conservative upper bound on the true curve-to-helix distance).
            If it exceeds dTolerance -- because the fitter floored the request,
            forced success at its span cap, or simply could not converge -- this
            returns SM_ERR_NOT_WITHIN_TOLERANCE with rpResult == NULL instead of
            handing back an out-of-tolerance curve.
***********************************************************************/
SmApiStatus SmApiCreateHelix
( 
    const SmPoint3d& crOrigin,         ///< [in ]: Base point
    double dHeight,              ///< [in ]: Axial length along +Z
    double dRadiusStart,         ///< [in ]: Radius at Z = 0
    double dRadiusEnd,           ///< [in ]: Radius at Z = dHeight
    double dNumTurns,            ///< [in ]: Number of 360-degree turns
    SmBoolean bRightHanded,      ///< [in ]: TRUE = right handed
    double dTolerance,           ///< [in ]: Max fit deviation from the true helix
    SmBSplineCurve*&  rpResult  ///< [out]: Resulting SmBSplineCurve (only valid on SM_SUCCESS)
)     
{
    // Deterministic null on any failure path; only overwritten on SM_SUCCESS below.
    rpResult = NULL;

    // Validate the complete public input domain before touching the reference
    // frame or the legacy spiral fitter (see NOTES). No NLib cleanup stack is
    // set up in this wrapper, so these early returns cannot free uninitialized
    // state; the fitter is simply never reached with out-of-domain values.
    const double kMinHeight      = 1.0e-7;                 // model tol; height must be positive
    const double kMinRadius      = 1.0e-7;                 // reject zero/negative radii (both-zero is degenerate)
    const double kMinTurns       = 1.0e-7;                 // fitter requires NumTurns >= NL_MTOL
    const double kMinTolerance   = 1.0e-6;                 // effective floor of the fit tolerance
    const double kMaxMagnitude   = SM_BIG_DOUBLE / 100.0;  // matches SM_IS_VALID_DOUBLE's finite cap (1e18)
    const double kMaxSpiralSpans = 994.0;                  // NL_CCPLIM (1000) - 6, the fitter's control-point cap
    const double kMaxTurns       = kMaxSpiralSpans / 8.0;  // seed spans = 8*turns must stay <= cap (~124.25)

    // crOrigin.IsUndef() rejects NaN / +/-Inf / undef in any component.
    if( crOrigin.IsUndef() )
        SER( SM_ERR_INVALID_INPUT );

    // RANGE_ER( low, mid, high ) also rejects NaN and +/-Inf: any comparison
    // against them is false, so the containment test fails and it signals
    // SM_ERR_INVALID_INPUT. The dNumTurns upper bound caps the fitter's seed
    // span count (8*turns) before its float->int conversion / allocation.
    RANGE_ER( kMinHeight,    dHeight,      kMaxMagnitude );
    RANGE_ER( kMinRadius,    dRadiusStart, kMaxMagnitude );
    RANGE_ER( kMinRadius,    dRadiusEnd,   kMaxMagnitude );
    RANGE_ER( kMinTurns,     dNumTurns,    kMaxTurns     );
    RANGE_ER( kMinTolerance, dTolerance,   kMaxMagnitude );

    // Reject inputs whose derived extents are not finite before any placement math.
    const double dMaxRadius = ( dRadiusStart > dRadiusEnd ) ? dRadiusStart : dRadiusEnd;
    if(    !SM_IS_VALID_DOUBLE( crOrigin.z + dHeight )
        || !SM_IS_VALID_DOUBLE( crOrigin.x + dMaxRadius ) || !SM_IS_VALID_DOUBLE( crOrigin.x - dMaxRadius )
        || !SM_IS_VALID_DOUBLE( crOrigin.y + dMaxRadius ) || !SM_IS_VALID_DOUBLE( crOrigin.y - dMaxRadius ) )
        SER( SM_ERR_INVALID_INPUT );

 #if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( crOrigin, &s_kRed, 1, 0 );
    }
#endif // SM_DEBUG_CODE
    
    SmVector3d xAxis( 1,0,0 );
    SmVector3d yAxis( 0,1,0 );
    
    SmAxis2Placement sRefFrame;
    sRefFrame.SetCanonical( crOrigin, xAxis, yAxis );

    SmBSplineCurve* pHelix = NULL;
    SmApiStatus status = SmBSplineCurve::CreateHelixSegment( *SmApiGetOrCreateContext(), sRefFrame, dHeight, dRadiusStart, dRadiusEnd, dNumTurns, bRightHanded, dTolerance, pHelix );
    if( status != SM_SUCCESS )
        return( status );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pHelix );
#endif // SM_ASSERT_VALID

    rpResult = pHelix;

    // Enforce dTolerance as a hard contract (see NOTES). The delegated fitter
    // only samples radial error at one midpoint per span, silently floors the
    // requested tolerance, forces success once it hits its span cap, and the
    // achieved error is then discarded. Independently measure the true maximum
    // 3D deviation of the fitted curve from the analytic helix and, if it
    // exceeds the caller's requested dTolerance, return SM_ERR_NOT_WITHIN_TOLERANCE
    // with no curve rather than hand back an out-of-tolerance approximation.
    {
        const double kTwoPi     = 6.2831853071795864;       // matches the fitter's TWOPI
        const double d2piTurns  = kTwoPi * dNumTurns;        // total sweep angle
        const double dRL        = bRightHanded ? 1.0 : -1.0; // right/left winding about +Z
        const SmExtent1d sIvl   = rpResult->GetNaturalInterval();
        const double dParamMin  = sIvl.GetMin();
        const double dParamSpan = sIvl.GetMax() - dParamMin;

        // Dense sampling: enough points per fitter span (~8*turns..994 spans) to
        // catch the mid-span error peaks the fitter's single-midpoint test misses.
        long lNumSamples = (long)( 200.0 * dNumTurns ) + 16;
        if( lNumSamples < 1024  ) lNumSamples = 1024;
        if( lNumSamples > 20000 ) lNumSamples = 20000;

        double dMaxDev = 0.0;
        for( long i = 0; i <= lNumSamples; ++i )
        {
            const double dU = dParamMin + dParamSpan * ( (double)i / (double)lNumSamples );
            SmPoint3d sOnCurve;
            if( rpResult->EvaluatePoint( dU, sOnCurve ) != SM_SUCCESS )
            {
                delete rpResult;
                rpResult = NULL;
                SER( SM_ERR );
            }

            // Pair each curve point with the analytic helix point at the same
            // axial height (the reference frame is identity-rotation at crOrigin,
            // so local = world - crOrigin, and z = dHeight*theta/(2*pi*turns)).
            // This is the full-3D generalization of the fitter's radial metric
            // and is a conservative upper bound on the true curve-to-helix distance.
            double dTheta = ( ( sOnCurve.z - crOrigin.z ) * d2piTurns ) / dHeight;
            if( dTheta < 0.0 )       dTheta = 0.0;
            if( dTheta > d2piTurns ) dTheta = d2piTurns;

            const double dRadius = ( dRadiusStart * ( d2piTurns - dTheta ) + dRadiusEnd * dTheta ) / d2piTurns;
            const double dAx = crOrigin.x + dRadius * cos( dTheta );
            const double dAy = crOrigin.y + dRL * dRadius * sin( dTheta );
            const double dAz = crOrigin.z + dHeight * dTheta / d2piTurns;

            const double ddx = sOnCurve.x - dAx;
            const double ddy = sOnCurve.y - dAy;
            const double ddz = sOnCurve.z - dAz;
            const double dDev = sqrt( ddx * ddx + ddy * ddy + ddz * ddz );
            if( dDev > dMaxDev )
                dMaxDev = dDev;
        }

        if( dMaxDev > dTolerance )
        {
            delete rpResult;
            rpResult = NULL;
            SER( SM_ERR_NOT_WITHIN_TOLERANCE );
        }
    }

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif // SM_DEBUG_CODE

    return( SM_SUCCESS );

} // end SmApiCreateHelix

/*******************************************************************//**
PURPOSE --- Create rectangle as four line segments

NOTES ---   Centered
***********************************************************************/
SmApiStatus SmApiCreateRectangle
(
    const SmPoint3d& crCenterPt,                 ///< [in ]: Center Point                                               <br>
    double dLength,                        ///< [in ]: Length along X axis                                                    <br>
    double dWidth,                         ///< [in ]: Width along Y axis                                    <br>
    SmTArray<SmBSplineCurve*>& rpResult   ///< [out]: Resulting SmBSplineCurve                                  <br>
)
{

    // Negated test so a NaN size is rejected too.
    if( !( dLength > 0.0 ) || !( dWidth > 0.0 ) )
        return SM_ERR_INVALID_INPUT;

    const ULONG lSizeOnEntry = rpResult.GetSize();
    double dHalfL = dLength * 0.5;
    double dHalfW = dWidth  * 0.5;

    SmPoint3d corners[4];
    corners[0].Set( crCenterPt.x - dHalfL, crCenterPt.y - dHalfW, crCenterPt.z );
    corners[1].Set( crCenterPt.x + dHalfL, crCenterPt.y - dHalfW, crCenterPt.z );
    corners[2].Set( crCenterPt.x + dHalfL, crCenterPt.y + dHalfW, crCenterPt.z );
    corners[3].Set( crCenterPt.x - dHalfL, crCenterPt.y + dHalfW, crCenterPt.z );

    // Rejects a NaN or infinite center or size, and corners outside the
    // valid double range.
    for( int ii = 0; ii < 4; ii++ )
    {
        if( !SM_IS_VALID_DOUBLE( corners[ii].x ) ||
            !SM_IS_VALID_DOUBLE( corners[ii].y ) ||
            !SM_IS_VALID_DOUBLE( corners[ii].z ) )
            return SM_ERR_INVALID_INPUT;
    }

    for( int ii = 0; ii < 4; ii++ )
    {
        int jj = ( ii + 1 ) % 4;
        SmLine* pSeg = NULL;
        SmStatus stat = SmLine::CreateLineSegment(
            *SmApiGetOrCreateContext(), 3, corners[ii], corners[jj], pSeg );
        if( stat != SM_SUCCESS || pSeg == NULL )
        {
            // Drop the segments this call already appended.
            for( ULONG kk = lSizeOnEntry; kk < rpResult.GetSize(); kk++ )
                delete rpResult[kk];
            rpResult.SetSize( lSizeOnEntry );
            return SM_ERR;
        }

        rpResult.Add( pSeg );
    }

    return( SM_SUCCESS );

}  // end SmApiCreateRectangle

/*******************************************************************//**
PURPOSE --- Remove unnecessary B-spline knots within the curve's effective approximation tolerance

NOTES ---   Non-B-spline curve types are unchanged.

***********************************************************************/
SmApiStatus SmApiRemoveCurveKnots
(
    SmCurve* pCurve                       ///< [in/out]: Curve to simplify
)
{
    if( pCurve == NULL )
        return SM_ERR_INVALID_INPUT;

    SmBSplineCurve* pBSC = dynamic_cast<SmBSplineCurve*>( pCurve );
    if( pBSC != NULL )
    {
        double dTol = SmTol::GetApproxTol3d(pCurve);
        return pBSC->RemoveKnots( dTol );
    }

    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE --- Convert this curve to lines and arcs

NOTES ---  
***********************************************************************/
SmApiStatus SmApiCreateRegularPolygon
(
    const SmPoint3d& crCenterPt,                 ///< [in ]: Center Point                                               <br>
    ULONG nSides,                          ///< [in ]:
    double dRadius,                        ///< [in ]: Radius of circumscribed  circle                                                    <br>
    SmTArray<SmBSplineCurve*>& rpResult   ///< [out]: Resulting SmBSplineCurve                                  <br>
)
{

    // Negated test so a NaN radius is rejected too.
    if( nSides < 3 || !( dRadius > 0.0 ) )
        return SM_ERR_INVALID_INPUT;

    // Rejects a NaN or infinite center or radius, and vertices outside the
    // valid double range.
    if( !SM_IS_VALID_DOUBLE( crCenterPt.x - dRadius ) ||
        !SM_IS_VALID_DOUBLE( crCenterPt.x + dRadius ) ||
        !SM_IS_VALID_DOUBLE( crCenterPt.y - dRadius ) ||
        !SM_IS_VALID_DOUBLE( crCenterPt.y + dRadius ) ||
        !SM_IS_VALID_DOUBLE( crCenterPt.z ) )
        return SM_ERR_INVALID_INPUT;

    const ULONG lSizeOnEntry = rpResult.GetSize();
    double dAngleStep = 2.0 * SM_PI / (double)nSides;

    for( ULONG ii = 0; ii < nSides; ii++ )
    {
        double a0 = dAngleStep * ii;
        double a1 = dAngleStep * ( ii + 1 );

        SmPoint3d pt0( crCenterPt.x + dRadius * cos(a0),
                       crCenterPt.y + dRadius * sin(a0),
                       crCenterPt.z );
        SmPoint3d pt1( crCenterPt.x + dRadius * cos(a1),
                       crCenterPt.y + dRadius * sin(a1),
                       crCenterPt.z );

        SmLine* pSeg = NULL;
        SmStatus stat = SmLine::CreateLineSegment(
            *SmApiGetOrCreateContext(), 3, pt0, pt1, pSeg );
        if( stat != SM_SUCCESS || pSeg == NULL )
        {
            // Drop the segments this call already appended.
            for( ULONG kk = lSizeOnEntry; kk < rpResult.GetSize(); kk++ )
                delete rpResult[kk];
            rpResult.SetSize( lSizeOnEntry );
            return SM_ERR;
        }

        rpResult.Add( pSeg );
    }

    return( SM_SUCCESS );

} // end SmApiCreateRegularPolygon


/*******************************************************************//**
PURPOSE --- Create an curve given control points.  

NOTES ---   Assume uniform knots with range 0 - 1
            See several similar curve creation functions
***********************************************************************/
SmApiStatus SmApiCreateCurve
( 
    const SmTArray<SmPoint3d>& crPoints,    ///< [in ]: Points
    SmBSplineCurve*&  rpResult       ///< [out]: Resulting SmBSplineCurve
)  
{
 #if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( crPoints, &s_kRed, 1, 0 );
    }
#endif // SM_DEBUG_CODE

    ULONG lDegree = 3;
    ULONG ii = 0;
    ULONG lNumControlPoints = crPoints.GetSize();

    // A cubic needs lDegree+1 control points; fewer would underflow lNumKnots.
    if( lNumControlPoints < lDegree + 1 )
        return( SM_ERR_INVALID_INPUT );

    ULONG lNumKnots = lNumControlPoints - lDegree + 1;

    SmTArray < double> sKnots;
    sKnots.Add(0.0);
    for( ii = 1; ii < lNumKnots - 1; ii++ ) {
        sKnots.Add((double)ii/lNumKnots);
    }
    sKnots.Add(1.0);

    SmTArray < ULONG> sKnotMult;
    sKnotMult.Add(lDegree + 1);
    for( ii = 1; ii < lNumKnots - 1; ii++ ) {
        sKnotMult.Add(1);
    }
    sKnotMult.Add(lDegree + 1);

    SmBSplineCurve* pCurve = NULL;
    SmApiStatus status = SmBSplineCurve::CreateCanonical(*SmApiGetOrCreateContext(), 3, 3, crPoints, SM_CF_UNSPECIFIED, sKnotMult, sKnots, SM_KT_UNSPECIFIED, NULL, NULL, pCurve);
    if( status != SM_SUCCESS )
        return( status );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pCurve );
#endif // SM_ASSERT_VALID

    rpResult = pCurve;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif // SM_DEBUG_CODE

    return( SM_SUCCESS );

} // end SmApiCreateCurve


/*******************************************************************//**
PURPOSE --- Create an curve given control points, knots, and weights.  

NOTES ---   Assume uniform knots with range 0 - 1
            See several similar curve creation functions
***********************************************************************/
SmApiStatus SmApiCreateCanonicalCurve
(
    const SmTArray<SmPoint3d>& crPoints,    ///< [in ]: Points
    const SmTArray<double>& crKnots,        ///< [in ]: Knots
    const SmTArray<ULONG>& crKnotMultiplicities, ///< [in ]: Knots multiplicities of the end knots must be lDegree+1 and
                                            ///<      : and internal multiplicities must be lDegree or less.
    const SmTArray<double>* crWeights,      ///< [in ]: Weights - optional
    ULONG lDegree,                          ///< [in ]: Degree
    SmBSplineCurveForm eBSplineCurveForm,   ///< [in ]: SM_CF_POLYLINE_FORM,  SM_CF_PARABOLIC_ARC, 
                                            ///<      : SM_CF_CIRCULAR_ARC,   SM_CF_HYPERBOLIC_ARC,
                                            ///<      : SM_CF_ELLIPTIC_ARC,   SM_CF_UNSPECIFIED,
                                            ///<      : SM_CF_HELICAL_ARC    
    SmBSplineCurve*&  rpResult             ///< [out]: Resulting SmBSplineCurve
)   
{
 #if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( crPoints, &s_kRed, 1, 0 );
    }
#endif // SM_DEBUG_CODE

    SmBSplineCurve* pCurve = NULL;
    SmApiStatus status = SmBSplineCurve::CreateCanonical( *SmApiGetOrCreateContext(), 3, lDegree, crPoints, eBSplineCurveForm, crKnotMultiplicities, crKnots, SM_KT_UNSPECIFIED, crWeights, NULL, pCurve);
    if( status != SM_SUCCESS )
        return( status );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pCurve );
#endif // SM_ASSERT_VALID

    rpResult = pCurve;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif // SM_DEBUG_CODE

    return( SM_SUCCESS );

} // end SmCreateCanonicalCurve


/*******************************************************************//**
PURPOSE --- Create an curve by approximating (fit) given points.  

NOTES ---   Assume uniform knots with range 0 - 1
            See several similar curve creation functions
***********************************************************************/
SmApiStatus SmApiCreateCurveApproxPoints
(
    const SmTArray<SmPoint3d>& crPoints,          ///< [in ]: Points
    SmCurve*&  rpResult                    ///< [out]: Resulting SmCurve
)
{
 #if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( crPoints, &s_kRed, 1, 0 );
    }
#endif // SM_DEBUG_CODE

    SmTArray<SmVector3d> sVectors;

    // Determine number of points -- This algorithm can be improved
    ULONG lMaxCntrlPts = 4;  // min number of points
    if( crPoints.GetSize() > 8 )
        lMaxCntrlPts = crPoints.GetSize() / 2;   // increase relative to number of input points
    if( crPoints.GetSize() > 16 )
        lMaxCntrlPts = crPoints.GetSize() / 4;   // increase relative to number of input points

    SmBSplineCurve* pCurve = NULL;
    SmApiStatus status = SmBSplineCurve::CreateApproximatingCurve( *SmApiGetOrCreateContext(), lMaxCntrlPts, 0, 3, 3, crPoints, sVectors, NULL, pCurve);
    if( status != SM_SUCCESS )
        return( status );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pCurve );
#endif // SM_ASSERT_VALID

    rpResult = pCurve;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif // SM_DEBUG_CODE

    return( SM_SUCCESS );

} // end SmApiCreateCurveApproxPoints

/*******************************************************************//**
PURPOSE --- Create an curve by interpolating given points.  

NOTES ---   Knot parameterization selectable via eParameterization
            (uniform by default); parameter range 0 - 1.
            See several similar curve creation functions
***********************************************************************/
SmApiStatus SmApiCreateCurveInterpPoints
( 
  const SmTArray<SmPoint3d>& crPoints,          ///< [in ]: Points
  SmCurve*&  rpResult,                    ///< [out]: Resulting SmCurve
  SmCurveParameterizationType eParameterization  ///< [in ]: Knot parameterization of the fit
)
{
    // rpResult is only valid on SM_SUCCESS; guarantee a deterministic null on
    // every failure path.  The degree-3 interpolating fit needs at least 4
    // points, and every point must be finite.
    rpResult = NULL;
    if( crPoints.GetSize() < 4 )
        return( SM_ERR_INVALID_INPUT );
    for( ULONG iPt = 0; iPt < crPoints.GetSize(); iPt++ )
        if( crPoints[iPt].IsUndef() )
            return( SM_ERR_INVALID_INPUT );

    // The kernel silently maps any unrecognized parameterization enum to
    // NL_UNIFORM, so raw C++ casts and Python values such as
    // CurveParameterization(999) would otherwise succeed.  Accept only the
    // three documented values and reject everything else.
    if( eParameterization != SM_CP_UNIFORM &&
        eParameterization != SM_CP_CHORDLENGTH &&
        eParameterization != SM_CP_CENTRIPETAL )
        return( SM_ERR_INVALID_INPUT );

    // Distance-based modes need distinct successive points; a coincident pair
    // has no defined spacing and would otherwise return a generic SM_ERR.
    if( eParameterization == SM_CP_CHORDLENGTH ||
        eParameterization == SM_CP_CENTRIPETAL )
        for( ULONG iPt = 1; iPt < crPoints.GetSize(); iPt++ )
            if( crPoints[iPt].DistanceBetweenSquared( crPoints[iPt - 1] ) <= SM_EFF_ZERO_SQ )
                return( SM_ERR_INVALID_INPUT );

 #if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( crPoints, &s_kRed, 1, 0 );
    }
#endif // SM_DEBUG_CODE

    SmTArray<SmVector3d> sVectors;

    SmBSplineCurve* pCurve = NULL;
    SmApiStatus status = SmBSplineCurve::CreateInterpolatingCurve( *SmApiGetOrCreateContext(), eParameterization, 3, 3, crPoints,  sVectors, NULL, false, pCurve);
    if( status != SM_SUCCESS )
        return( status );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pCurve );
#endif // SM_ASSERT_VALID

    rpResult = pCurve;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif // SM_DEBUG_CODE

    return( SM_SUCCESS );

} // end SmApiCreateCurveInterpPoints

/*******************************************************************//**
PURPOSE --- Offset a given curve  

NOTES ---   Alt: SmCompositeCurve::CreateOffsetsOfCurve
            Alt: SmOffsetCurve::SmOffsetCurve
            Alt: SmPrimitiveCreation::OffsetProfile
***********************************************************************/
SmApiStatus SmApiOffsetCurve
( 
    SmBSplineCurve* pCurveToOffset,         ///< [in ]: Curve to offset
    double dOffsetDistance,                 ///< [in ]: Distance of offset ( positive to right of input curve 
                                            ///<      :                      negative to left of input curve )
    SmBSplineCurve*&  rpResult             ///< [out]: Resulting SmBSplineCurve
) 
{
 #if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( pCurveToOffset, NULL, 1, 1 );
    }
#endif // SM_DEBUG_CODE

    // Determine tolerance as a funciton of size of input curve
    double dApproxTol = 0.001;

    // Determine plane of input curve
    SmTArray<SmCurve*> sCrvArray;
    sCrvArray.Add( pCurveToOffset );

    // A single straight curve has no plane of its own: ComputeCrvsNormal fails
    // and zeroes the normal, so offset it in the XY plane instead, or in the
    // XZ plane if the line is parallel to Z (the XY normal would be along it).
    SmVector3d sCrvNormal( 0,0,1 );
    if( SmCurve::ComputeCrvsNormal( sCrvArray, sCrvNormal ) != SM_SUCCESS )
    {
        sCrvNormal.Set( 0,0,1 );
        SmPoint3d sStart, sEnd;
        pCurveToOffset->GetEnds( sStart, sEnd );
        SmVector3d sDir = sEnd - sStart;
        if( sDir.Unitize() == SM_SUCCESS && ( sDir * sCrvNormal ).Length() < SM_EFF_ZERO )
            sCrvNormal.Set( 0,1,0 );
    }

    SmBSplineCurve* pCurve = NULL;
    double dAchievedTol = 0;
    SmApiStatus status = pCurveToOffset->CreateSimpleOffset( *SmApiGetOrCreateContext(), dApproxTol, sCrvNormal, dOffsetDistance, pCurve, dAchievedTol, false );
    if( status != SM_SUCCESS )
        return( status );

#ifdef SM_ASSERT_VALID
    SM_ASSERT_VALID( pCurve );
#endif // SM_ASSERT_VALID

    rpResult = pCurve;

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        SmApiDraw( rpResult, &s_kBlue, 0, 1 );
    }
#endif // SM_DEBUG_CODE

    return( SM_SUCCESS );

} // end SmApiOffsetCurve


/*******************************************************************//**
PURPOSE --- Evaluate a given curve for position, tangent, and second derivative 
            
NOTES ---  
***********************************************************************/
SmApiStatus SmApiEvaluateCurve
( 
    const SmCurve* pCurve,                        ///< [in ]: Curve to evaluate
    double dParameter,                      ///< [in ]: Parameter at which to evaluate
    SmVector3d*& crPoint,                   ///< [out]: Opt - Resulting Point at this parameter
    SmVector3d*& crDeriv1,                  ///< [out]: Opt - First Derivative (tangent) at this parameter
    SmVector3d*& crDeriv2                   ///< [opt]: - Second Derivative at this parameter
)     
{
    if( pCurve == NULL )
        return( SM_ERR_INVALID_INPUT );

    // Negated in-range test so a NaN parameter is rejected too.
    SmExtent1d domain = pCurve->GetNaturalInterval();
    if( !( dParameter >= domain.GetMin() && dParameter <= domain.GetMax() ) )
        return( SM_ERR_INVALID_INPUT );
 
    SmVector3d sPointAndDerivatives[3];
    SmStatus stat = pCurve->Evaluate( dParameter, 2, true, sPointAndDerivatives );
    if( stat != SM_SUCCESS )
        return( stat );

    if( crPoint )
        *crPoint = sPointAndDerivatives[0];
    if( crDeriv1 )
        *crDeriv1 = sPointAndDerivatives[1];
    if( crDeriv2 )
        *crDeriv2 = sPointAndDerivatives[2];

    return( SM_SUCCESS );

} // End SmApiEvaluateCurve


/*******************************************************************//**
PURPOSE --- Evaluate a given curve for position, tangent, and second derivative 
            In addition, the curve is evaluated from the left and the right
            to provide the continuity (G1, C1, etc) as well.
NOTES ---  
***********************************************************************/
SmApiStatus SmApiEvaluateContinuity
( 
    const SmCurve* pCurve,                        ///< [in ]: Curve to evaluate
    double dParameter,                      ///< [in ]: Parameter at which to evaluate
    SmContinuityType& crType                ///< [out]: Continuity at this parameter; valid on success
)
{
    if( pCurve == NULL )
        return( SM_ERR_INVALID_INPUT );

    // Written as a negated in-range test so a NaN parameter is rejected too.
    SmExtent1d domain = pCurve->GetNaturalInterval();
    if( !( dParameter >= domain.GetMin() && dParameter <= domain.GetMax() ) )
        return( SM_ERR_INVALID_INPUT );
 
    SmVector3d sPointAndDerivatives[4];
    return pCurve->EvaluateContinuity( dParameter, true, *pCurve, dParameter, false, crType, sPointAndDerivatives, NULL );

} // End SmApiEvaluateContinuity

/*******************************************************************//**
PURPOSE --- Convert this curve to lines and arcs

NOTES ---  
***********************************************************************/
SmApiStatus SmApiProjectCurve
(
    const SmBSplineCurve* pCurve,                  ///< [in ]: Curve to drop                                              <br>
    const SmSurface* pSurface,                     ///< [in ]: Surface                                                    <br>
    SmTArray<SmBSplineCurve*>* pOptCurves2d, ///< [out]: Resulting uv curves on surface                             <br>
    SmTArray<SmBSplineCurve*>* pOptCurves3d ///< [out]: Resulting 3d curves on surface                             <br>
)
{

    if( pCurve == NULL || pSurface == NULL )
        return SM_ERR_INVALID_INPUT;

    const SmContext& ctx = *SmApiGetOrCreateContext();
    SmTArray<SmBSplineCurve*> sCurvesOnSrf2d;
    double dMaxDrop = 0.0;
    double dMaxGap = 0.0;

    SmExtent3d sCrvBBox;
    pCurve->CalculateBoundingBox( pCurve->GetNaturalInterval(), &sCrvBBox );
    double dApproxTol = sCrvBBox.GetSize().Length();
    if( dApproxTol < 1.0 ) dApproxTol = 1.0;

    SmStatus stat = pSurface->DropAndTrimCurve(
        ctx,
        pSurface->GetNaturalUVDomain(),
        *pCurve,
        pCurve->GetNaturalInterval(),
        dApproxTol,
        dMaxDrop,
        dMaxGap,
        sCurvesOnSrf2d,
        TRUE,
        FALSE );

    if( stat != SM_SUCCESS )
        return stat;

    // Lift into a local array so the outputs are only touched on full success.
    SmTArray<SmBSplineCurve*> sCurves3d;
    if( pOptCurves3d )
    {
        for( ULONG ii = 0; ii < sCurvesOnSrf2d.GetSize(); ii++ )
        {
            SmBSplineCurve* p3d = NULL;
            double dMaxDistToSrf = 0;
            SmStatus liftStat = pSurface->LiftCurve(
                ctx, pSurface->GetNaturalUVDomain(),
                *sCurvesOnSrf2d[ii], sCurvesOnSrf2d[ii]->GetNaturalInterval(),
                SM_ZONE_TOL_3D, dMaxDistToSrf, p3d );
            if( liftStat != SM_SUCCESS || p3d == NULL )
            {
                for( ULONG jj = 0; jj < sCurves3d.GetSize(); jj++ )
                    delete sCurves3d[jj];
                for( ULONG jj = 0; jj < sCurvesOnSrf2d.GetSize(); jj++ )
                    delete sCurvesOnSrf2d[jj];
                return ( liftStat != SM_SUCCESS ) ? liftStat : SM_ERR;
            }
            sCurves3d.Add( p3d );
        }
    }

    for( ULONG ii = 0; ii < sCurvesOnSrf2d.GetSize(); ii++ )
    {
        if( pOptCurves2d )
            pOptCurves2d->Add( sCurvesOnSrf2d[ii] );
        else
            delete sCurvesOnSrf2d[ii];
    }
    for( ULONG ii = 0; ii < sCurves3d.GetSize(); ii++ )
        pOptCurves3d->Add( sCurves3d[ii] );

    return( SM_SUCCESS );

} // End SmApiProjectCurve

/*******************************************************************//**
PURPOSE --- Drop a curve onto a surface using surface normals

NOTES ---  
***********************************************************************/
SmApiStatus SmApiDropCurveToSrf
( 
    SmBSplineCurve* pCurve,                  ///< [in ]: Curve to drop
    SmSurface* pSurface,                     ///< [in ]: Surface
    SmTArray<SmBSplineCurve*>* pOptCurves2d, ///< [out]: Resulting uv curves on surface
    SmTArray<SmBSplineCurve*>* pOptCurves3d ///< [out]: Resulting 3d curves on surface
)     
{
#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( pCurve, &s_kBlack, 1, 1 );
        SmApiDraw( pSurface, &s_kBlack, 0, 1 );
    }
#endif // SM_DEBUG_CODE

    if( pCurve == NULL || pSurface == NULL )
        return( SM_ERR_INVALID_INPUT );

    double dApproxTol = 10;
    double dMaxDrop = 0.0;
    double dMaxGap = 0.0;

    const SmContext* pContext = pSurface->GetContext();

    SmTArray<SmBSplineCurve*> sCurvesUV;

    SmApiStatus status = pSurface->DropAndTrimCurve( *pContext, pSurface->GetNaturalUVDomain(), 
        *pCurve, pCurve->GetNaturalInterval(), dApproxTol, dMaxDrop, dMaxGap, sCurvesUV, TRUE, FALSE );
    if( status != SM_SUCCESS )
        return( status );

    // Lift into a local array so the outputs are only touched on full success.
    SmTArray<SmBSplineCurve*> sCurves3d;
    if( pOptCurves3d ) {

        double dMaxDistToSrf = 0;

        for( ULONG ii = 0; ii < sCurvesUV.GetSize(); ii++ ) {
            SmBSplineCurve* pCrv2d = sCurvesUV[ii];
            SmBSplineCurve* pCrv3d = NULL;
            SmStatus liftStat = pSurface->LiftCurve( *pContext, pSurface->GetNaturalUVDomain(), 
                *(pCrv2d), pCrv2d->GetNaturalInterval(), 0.001, dMaxDistToSrf, pCrv3d);               
            if( liftStat != SM_SUCCESS || pCrv3d == NULL ) {
                for( ULONG jj = 0; jj < sCurves3d.GetSize(); jj++ )
                    delete sCurves3d[jj];
                for( ULONG jj = 0; jj < sCurvesUV.GetSize(); jj++ )
                    delete sCurvesUV[jj];
                return ( liftStat != SM_SUCCESS ) ? liftStat : SM_ERR;
            }

#if SM_DEBUG_CODE
            if( bDebugMe  ) {
                SmApiDraw( pCrv3d, &s_kBlue, 0, 0 );
            }
#endif // SM_DEBUG_CODE

            sCurves3d.Add( pCrv3d );
        }
    }

    for( ULONG ii = 0; ii < sCurvesUV.GetSize(); ii++ ) {
        if( pOptCurves2d )
            pOptCurves2d->Add( sCurvesUV[ii] );
        else
            delete sCurvesUV[ii];
    }
    for( ULONG ii = 0; ii < sCurves3d.GetSize(); ii++ )
        pOptCurves3d->Add( sCurves3d[ii] );

    return( SM_SUCCESS );

}  // End SmApiDropCurveToSrf

/*******************************************************************//**
PURPOSE --- Convert this curve to lines and arcs

NOTES ---  
***********************************************************************/
SmApiStatus SmApiLiftUVCurveFromSrf
( 
    const SmBSplineCurve* pUVCurve,                ///< [in ]: UV curve to lift                                           <br>
    const SmSurface* pSurface,                     ///< [in ]: Surface                                                    <br>
    SmTArray<SmBSplineCurve*>* pCurves3d    ///< [out]: Resulting 3d curves on surface                             <br>
)
{

    if( pUVCurve == NULL || pSurface == NULL || pCurves3d == NULL )
        return SM_ERR_INVALID_INPUT;

    const SmContext& ctx = *SmApiGetOrCreateContext();
    SmBSplineCurve* p3d = NULL;
    double dMaxDistToSrf = 0;

    SmStatus stat = pSurface->LiftCurve(
        ctx, pSurface->GetNaturalUVDomain(),
        *pUVCurve, pUVCurve->GetNaturalInterval(),
        SM_ZONE_TOL_3D, dMaxDistToSrf, p3d );

    if( stat != SM_SUCCESS || p3d == NULL )
        return ( stat != SM_SUCCESS ) ? stat : SM_ERR;

    pCurves3d->Add( p3d );

    return( SM_SUCCESS );

} // End SmApiLiftUVCurveFromSrf

/*******************************************************************//**
PURPOSE --- Make B-spline curves compatible.

NOTES ---   The curves are rewritten in place. The kernel updates them one
            at a time, so a failure can leave some already modified.
***********************************************************************/
SmApiStatus SmApiMakeCurvesCompatible
(
    SmTArray<SmBSplineCurve*>& pCurves      ///< [in/out]: NURBS curves; modified in place                        <br>
)
{
    if( pCurves.GetSize() < 2 )
        return SM_ERR_INVALID_INPUT;

    SmStatus stat = SmBSplineCurve::MakeCurvesCompatible( pCurves );

    return stat;

} // End SmApiMakeCurvesCompatible

/*******************************************************************//**
PURPOSE --- Reorder curves in place into loop sequence

NOTES ---  Curves are not reversed. The per-curve orientation and loop
           boundaries computed by OrderCurvesIntoLoops are not reported,
           so consecutive curves need not meet head to tail.
***********************************************************************/
SmApiStatus SmApiOrderCurves
(
    SmTArray<SmBSplineCurve*>& pCurves      ///< [in/out]: Curves; on success reordered in place into loop order  <br>
)
{

    if( pCurves.GetSize() < 2 )
        return SM_ERR_INVALID_INPUT;

    SmTArray<SmCurve*> sCurves;
    for( ULONG ii = 0; ii < pCurves.GetSize(); ii++ )
        sCurves.Add( pCurves[ii] );

    SmTArray<SmCurve*> sOrderedCurves;
    SmTArray<ULONG> sLoopCounts;
    SmTArray<SmOrientType> sOrients;
    SmPoint3d sPlanePoint;
    SmVector3d sPlaneNormal;
    SmExtent3d sCurvesBBox;

    SmStatus stat = SmTrimmingTools::OrderCurvesIntoLoops(
        sCurves, SM_ZONE_TOL_3D,
        sOrderedCurves, sLoopCounts, sOrients,
        sPlanePoint, sPlaneNormal, sCurvesBBox );

    if( stat != SM_SUCCESS || sOrderedCurves.GetSize() == 0 )
        return ( stat != SM_SUCCESS ) ? stat : SM_ERR;

    pCurves.RemoveAll();
    for( ULONG ii = 0; ii < sOrderedCurves.GetSize(); ii++ )
    {
        SmBSplineCurve* pBSC = dynamic_cast<SmBSplineCurve*>( sOrderedCurves[ii] );
        if( pBSC == NULL )
            return SM_ERR;
        pCurves.Add( pBSC );
    }

    return( SM_SUCCESS );

} // End SmOrderCurves


/*******************************************************************//**
PURPOSE --- Convert this curve to lines and arcs

NOTES ---  
***********************************************************************/
SmApiStatus SmApiConvertToLinesAndArcs
( 
    SmBSplineCurve* pCurve,                  ///< [in ]: Curve to drop
    SmTArray<SmBSplineCurve*>& pLinesAndArcs ///< [out]: Resulting 3d curves
)
{
    // A curve smaller than the kernel's coincidence tolerance is a point.
    if( pCurve == NULL || pCurve->IsDegenerate( SmTol::GetZoneTol3d( pCurve ) ) )
        return( SM_ERR_INVALID_INPUT );

    // Fit tolerance for the lines and arcs. Hardcoded and not scaled to the
    // model; the context approximation tolerance (5e-6 by default) is far
    // tighter and much slower on large curves.
    double dTol = 0.001;

#if SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if( bDebugMe  ) {
        SmApiDraw( pCurve, &s_kBlack, 1, 1 );
    }
#endif // SM_DEBUG_CODE

    SmContext* pContext = (SmContext*)pCurve->GetContext();

    SmStatus stat = pCurve->SubdivideToLinesAndArcs( *pContext, dTol, pLinesAndArcs );
    if( stat != SM_SUCCESS )
        return( stat );

    // The kernel returns no pieces for a curve that is already a line or an
    // arc, or smaller than dTol; return a copy of it instead.
    if( pLinesAndArcs.GetSize() == 0 )
    {
        SmCurve* pCopy = NULL;
        SER( pCurve->Copy( *pContext, pCopy ) );
        pLinesAndArcs.Add( (SmBSplineCurve*)pCopy );
    }

#if SM_DEBUG_CODE
    if( bDebugMe  ) {
        //SmApiDraw( pLinesAndArcs, &s_kBlack, 1, 1 );
    }
#endif // SM_DEBUG_CODE

	return( SM_SUCCESS );

} // end SmApiConvertToLinesAndArcs
