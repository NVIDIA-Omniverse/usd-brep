// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//*******************************************************************
// Undistributed, user managed test file
// You should no longer need to modify any files under source 
// control to get testing flexibility
// The only thing that cannot change is the Test() function signature
//*******************************************************************

#include "StdAfx.h"

#include <SmApiBrep.h>
#include <SmApiCurves.h>
#include <SmApiGeneral.h>
#include <SmApiPolygons.h>
#include <SmApiPrimitives.h>
#include <SmApiQueries.h>
#include <SmApiSurfaces.h>
#include <SmApiTrimmedSurfaces.h>
#include <SmAssembly.h>
#include <SmAxis2Placement.h>
#include <SmCircle.h>
#include <SmCone.h>
#include <SmPrimitiveCreation.h>
#include <SmBSplineCurve.h>
#include <SmCylinder.h>
#include <SmSphere.h>
#include <SmSurfOfRevolution.h>
#include <SmTorus.h>
#include <SmEdge.h>
#include <SmFace.h>
#include <SmFeatureExecutive.h>
#include <SmLine.h>
#include <SmPlane.h>
#include <SmPoly.h>
#include <SmSurface.h>
#include <SmTrimmingTools.h>

#include <cmath>
#include <limits>

static SmBoolean sm_TestPointClose(const SmPoint3d& crActual, const SmPoint3d& crExpected)
{
    return crActual.DistanceBetween(crExpected) < 1.0e-8;
}

static SmBoolean sm_TestVectorClose(const SmVector3d& crActual, const SmVector3d& crExpected)
{
    return (crActual-crExpected).Length() < 1.0e-8;
}

static SmBoolean sm_TestVectorFinite(const SmVector3d& crVector)
{
    return std::isfinite(crVector.x) && std::isfinite(crVector.y) && std::isfinite(crVector.z);
}

static SmStatus sm_EvaluateSurfaceMidpoint(SmSurface* pSurface, SmPoint3d& rPoint)
{
    if( pSurface == NULL )
        return SM_ERR_INVALID_INPUT;

    const SmExtent2d sDomain = pSurface->GetNaturalUVDomain();
    SmVector2d sUV(0.5*(sDomain.GetMin().x+sDomain.GetMax().x),
                   0.5*(sDomain.GetMin().y+sDomain.GetMax().y));
    return SmApiEvaluateSurfacePoint(pSurface, sUV, rPoint);
}

struct SmPolyBrepTransformSnapshot
{
    SmTArray<SmPoint3d> m_sPoints;
    SmTArray<SmPolyVertex*> m_sVertices;
    SmTArray<ULONG> m_sVertexNormalCounts;
    SmTArray<SmVector3d> m_sVertexNormals;
    SmTArray<SmPolyVertAuxData*> m_sAuxData;
    SmTArray<SmVector3d> m_sAuxNormals;
};

static void sm_CapturePolyBrepTransformState(
    SmPolyBrep* pPolyBrep,
    SmPolyBrepTransformSnapshot& rSnapshot)
{
    rSnapshot.m_sPoints.ReSet();
    rSnapshot.m_sVertices.ReSet();
    rSnapshot.m_sVertexNormalCounts.ReSet();
    rSnapshot.m_sVertexNormals.ReSet();
    rSnapshot.m_sAuxData.ReSet();
    rSnapshot.m_sAuxNormals.ReSet();

    pPolyBrep->GetPolyPoints(rSnapshot.m_sPoints);
    pPolyBrep->GetPolyVertices(rSnapshot.m_sVertices);
    for( ULONG ii=0; ii<rSnapshot.m_sVertices.GetSize(); ++ii )
    {
        SmTArray<SmVector3d>& rNormals = rSnapshot.m_sVertices[ii]->GetNormalsRef();
        rSnapshot.m_sVertexNormalCounts.Add(rNormals.GetSize());
        for( ULONG jj=0; jj<rNormals.GetSize(); ++jj )
            rSnapshot.m_sVertexNormals.Add(rNormals[jj]);
    }

    pPolyBrep->GetAuxDataList(rSnapshot.m_sAuxData);
    for( ULONG ii=0; ii<rSnapshot.m_sAuxData.GetSize(); ++ii )
        rSnapshot.m_sAuxNormals.Add(rSnapshot.m_sAuxData[ii]->m_vNormal);
}

static SmBoolean sm_TestPolyBrepTransformStateClose(
    SmPolyBrep* pPolyBrep,
    const SmPolyBrepTransformSnapshot& crExpected)
{
    SmPolyBrepTransformSnapshot sActual;
    sm_CapturePolyBrepTransformState(pPolyBrep, sActual);

    if( sActual.m_sPoints.GetSize() != crExpected.m_sPoints.GetSize() ||
        !(sActual.m_sVertices == crExpected.m_sVertices) ||
        !(sActual.m_sVertexNormalCounts == crExpected.m_sVertexNormalCounts) ||
        sActual.m_sVertexNormals.GetSize() != crExpected.m_sVertexNormals.GetSize() ||
        !(sActual.m_sAuxData == crExpected.m_sAuxData) ||
        sActual.m_sAuxNormals.GetSize() != crExpected.m_sAuxNormals.GetSize() )
        return FALSE;

    for( ULONG ii=0; ii<sActual.m_sPoints.GetSize(); ++ii )
        if( !sm_TestPointClose(sActual.m_sPoints[ii], crExpected.m_sPoints[ii]) )
            return FALSE;

    for( ULONG ii=0; ii<sActual.m_sVertexNormals.GetSize(); ++ii )
        if( !sm_TestVectorClose(sActual.m_sVertexNormals[ii], crExpected.m_sVertexNormals[ii]) )
            return FALSE;

    for( ULONG ii=0; ii<sActual.m_sAuxNormals.GetSize(); ++ii )
        if( !sm_TestVectorClose(sActual.m_sAuxNormals[ii], crExpected.m_sAuxNormals[ii]) )
            return FALSE;

    return TRUE;
}

static SmStatus sm_GetExpectedPolyNormal(
    const SmVector3d& crNormal,
    const SmAxis2Placement& crRotation,
    const SmVector3d& crScale,
    SmVector3d& rExpected)
{
    crRotation.TransformVector(crNormal, rExpected);

    const ULONG lZeroScales = (crScale.x == 0.0) +
                              (crScale.y == 0.0) +
                              (crScale.z == 0.0);
    if( lZeroScales == 0 )
    {
        const double dMinAbsScale = crScale.GetMinDimension();
        const double dParity = ((crScale.x < 0.0) ^
                                (crScale.y < 0.0) ^
                                (crScale.z < 0.0)) ? -1.0 : 1.0;
        rExpected.Set(rExpected.x*(dParity*(dMinAbsScale/crScale.x)),
                      rExpected.y*(dParity*(dMinAbsScale/crScale.y)),
                      rExpected.z*(dParity*(dMinAbsScale/crScale.z)));
    }
    else if( lZeroScales == 1 )
    {
        if( crScale.x == 0.0 )
        {
            const double dSign = ((crScale.y < 0.0) ^ (crScale.z < 0.0)) ? -1.0 : 1.0;
            rExpected.Set(dSign*rExpected.x, 0.0, 0.0);
        }
        else if( crScale.y == 0.0 )
        {
            const double dSign = ((crScale.x < 0.0) ^ (crScale.z < 0.0)) ? -1.0 : 1.0;
            rExpected.Set(0.0, dSign*rExpected.y, 0.0);
        }
        else
        {
            const double dSign = ((crScale.x < 0.0) ^ (crScale.y < 0.0)) ? -1.0 : 1.0;
            rExpected.Set(0.0, 0.0, dSign*rExpected.z);
        }
    }
    else
    {
        rExpected.Set(0.0, 0.0, 0.0);
    }

    const double dMaxComponent = rExpected.GetMaxDimension();
    if( dMaxComponent > 0.0 )
    {
        rExpected /= dMaxComponent;
        SER( rExpected.Unitize() );
    }

    return SM_SUCCESS;
}

static SmStatus sm_TestPolyBrepNormalsAfterTransform(
    SmPolyBrep* pPolyBrep,
    const SmPolyBrepTransformSnapshot& crBefore,
    const SmAxis2Placement& crRotation,
    const SmVector3d& crScale)
{
    SmPolyBrepTransformSnapshot sAfter;
    sm_CapturePolyBrepTransformState(pPolyBrep, sAfter);

    if( !(sAfter.m_sVertices == crBefore.m_sVertices) ||
        !(sAfter.m_sVertexNormalCounts == crBefore.m_sVertexNormalCounts) ||
        sAfter.m_sVertexNormals.GetSize() != crBefore.m_sVertexNormals.GetSize() ||
        !(sAfter.m_sAuxData == crBefore.m_sAuxData) ||
        sAfter.m_sAuxNormals.GetSize() != crBefore.m_sAuxNormals.GetSize() )
        return SM_ERR;

    for( ULONG ii=0; ii<sAfter.m_sVertexNormals.GetSize(); ++ii )
    {
        SmVector3d sExpected;
        SER( sm_GetExpectedPolyNormal(crBefore.m_sVertexNormals[ii], crRotation, crScale, sExpected) );
        if( !sm_TestVectorClose(sAfter.m_sVertexNormals[ii], sExpected) )
            return SM_ERR;
    }

    for( ULONG ii=0; ii<sAfter.m_sAuxNormals.GetSize(); ++ii )
    {
        SmVector3d sExpected;
        SER( sm_GetExpectedPolyNormal(crBefore.m_sAuxNormals[ii], crRotation, crScale, sExpected) );
        if( !sm_TestVectorClose(sAfter.m_sAuxNormals[ii], sExpected) )
            return SM_ERR;
    }

    return SM_SUCCESS;
}

static SmStatus sm_TestAssemblyPolyBrepScale(
    SmPolyBrep* pSource,
    const SmVector3d& crScale)
{
    SmPolyBrepTransformSnapshot sSourceBefore;
    sm_CapturePolyBrepTransformState(pSource, sSourceBefore);

    SmAssembly* pAssembly = new (*pSource->GetContext()) SmAssembly(_T("PolyBrep transform test"));
    SmObjDelete sAssemblyCleanup(pAssembly);
    if( pAssembly == NULL )
        return SM_ERR;

    SmAxis2Placement sIdentity;
    SmAssemblyInstance* pInstance = new (*pSource->GetContext()) SmAssemblyInstance(
        _T("PolyBrep instance"), *pAssembly, pSource, sIdentity, crScale);
    SmObjDelete sInstanceCleanup(pInstance);
    if( pInstance == NULL )
        return SM_ERR;

    SmTArray<SmBrep*> sFlattenedBreps;
    SmTArray<SmPolyBrep*> sFlattenedPolyBreps;
    SmObjsDelete<SmBrep*> sFlattenedBrepCleanup(&sFlattenedBreps);
    SmObjsDelete<SmPolyBrep*> sFlattenedPolyBrepCleanup(&sFlattenedPolyBreps);
    SER( pAssembly->Flatten(sFlattenedBreps, sFlattenedPolyBreps) );

    if( sFlattenedBreps.GetSize() != 0 || sFlattenedPolyBreps.GetSize() != 1 )
        return SM_ERR;

    SmTArray<SmPoint3d> sFlattenedPoints;
    sFlattenedPolyBreps[0]->GetPolyPoints(sFlattenedPoints);
    if( sFlattenedPoints.GetSize() != sSourceBefore.m_sPoints.GetSize() )
        return SM_ERR;

    for( ULONG ii=0; ii<sFlattenedPoints.GetSize(); ++ii )
    {
        SmPoint3d sExpected(sSourceBefore.m_sPoints[ii].x*crScale.x,
                            sSourceBefore.m_sPoints[ii].y*crScale.y,
                            sSourceBefore.m_sPoints[ii].z*crScale.z);
        if( !sm_TestPointClose(sFlattenedPoints[ii], sExpected) )
            return SM_ERR;
    }

    if( !sm_TestPolyBrepTransformStateClose(pSource, sSourceBefore) )
        return SM_ERR;

    return SM_SUCCESS;
}

static SmStatus sm_TestPolyBrepKernelScaleCompatibility(SmPolyBrep* pSource)
{
    if( pSource == NULL )
        return SM_ERR_INVALID_INPUT;

    SmAxis2Placement sIdentity;

    {
        SmPolyBrep* pSigned = new (*pSource->GetContext()) SmPolyBrep(*pSource);
        SmObjDelete sSignedCleanup(pSigned);
        if( pSigned == NULL )
            return SM_ERR;

        SmPolyBrepTransformSnapshot sBefore;
        sm_CapturePolyBrepTransformState(pSigned, sBefore);
        SmTArray<SmPolyFace*> sFacesBefore;
        SmTArray<SmPolyEdge*> sEdgesBefore;
        pSigned->GetPolyFaces(sFacesBefore);
        pSigned->GetPolyEdges(sEdgesBefore);

        double dVolumeBefore = 0.0;
        SER( SmApiPolyBrepComputeVolume(pSigned, dVolumeBefore) );

        SmVector3d sSignedScale(-2.0, 3.0, 4.0);
        SER( pSigned->Transform(sIdentity, &sSignedScale) );

        SmPolyBrepTransformSnapshot sAfter;
        sm_CapturePolyBrepTransformState(pSigned, sAfter);
        if( sAfter.m_sPoints.GetSize() != sBefore.m_sPoints.GetSize() )
            return SM_ERR;

        for( ULONG ii=0; ii<sAfter.m_sPoints.GetSize(); ++ii )
        {
            SmPoint3d sExpected(sBefore.m_sPoints[ii].x*sSignedScale.x,
                                sBefore.m_sPoints[ii].y*sSignedScale.y,
                                sBefore.m_sPoints[ii].z*sSignedScale.z);
            if( !sm_TestPointClose(sAfter.m_sPoints[ii], sExpected) )
                return SM_ERR;
        }

        SmTArray<SmPolyFace*> sFacesAfter;
        SmTArray<SmPolyEdge*> sEdgesAfter;
        pSigned->GetPolyFaces(sFacesAfter);
        pSigned->GetPolyEdges(sEdgesAfter);
        if( !(sFacesAfter == sFacesBefore) || !(sEdgesAfter == sEdgesBefore) )
            return SM_ERR;

        SER( sm_TestPolyBrepNormalsAfterTransform(
            pSigned, sBefore, sIdentity, sSignedScale) );

        double dVolumeAfter = 0.0;
        SER( SmApiPolyBrepComputeVolume(pSigned, dVolumeAfter) );
        const double dExpectedVolume = dVolumeBefore*sSignedScale.x*sSignedScale.y*sSignedScale.z;
        if( !std::isfinite(dVolumeAfter) ||
            std::fabs(dVolumeAfter-dExpectedVolume) >
                std::max(1.0e-8, std::fabs(dExpectedVolume)*1.0e-8) )
            return SM_ERR;

        SER( sm_TestAssemblyPolyBrepScale(pSource, sSignedScale) );
    }

    const SmVector3d sSingularScales[] = {
        SmVector3d(1.0, 0.0, 1.0),
        SmVector3d(0.0, 0.0, 0.0)
    };
    for( ULONG ll=0; ll<2; ++ll )
    {
        SmPolyBrep* pSingular = new (*pSource->GetContext()) SmPolyBrep(*pSource);
        SmObjDelete sSingularCleanup(pSingular);
        if( pSingular == NULL )
            return SM_ERR;

        SmPolyBrepTransformSnapshot sBefore;
        sm_CapturePolyBrepTransformState(pSingular, sBefore);
        SmTArray<SmPolyFace*> sFacesBefore;
        SmTArray<SmPolyEdge*> sEdgesBefore;
        pSingular->GetPolyFaces(sFacesBefore);
        pSingular->GetPolyEdges(sEdgesBefore);

        const SmVector3d& sSingularScale = sSingularScales[ll];
        SER( pSingular->Transform(sIdentity, &sSingularScale) );
        SER( sm_TestPolyBrepNormalsAfterTransform(
            pSingular, sBefore, sIdentity, sSingularScale) );

        SmPolyBrepTransformSnapshot sAfter;
        sm_CapturePolyBrepTransformState(pSingular, sAfter);
        if( sAfter.m_sPoints.GetSize() != sBefore.m_sPoints.GetSize() )
            return SM_ERR;
        for( ULONG ii=0; ii<sAfter.m_sPoints.GetSize(); ++ii )
        {
            const SmPoint3d sExpected(sBefore.m_sPoints[ii].x*sSingularScale.x,
                                      sBefore.m_sPoints[ii].y*sSingularScale.y,
                                      sBefore.m_sPoints[ii].z*sSingularScale.z);
            if( !sm_TestPointClose(sAfter.m_sPoints[ii], sExpected) ||
                !sm_TestVectorFinite(sAfter.m_sPoints[ii]) )
                return SM_ERR;
        }
        for( ULONG ii=0; ii<sAfter.m_sVertexNormals.GetSize(); ++ii )
            if( !sm_TestVectorFinite(sAfter.m_sVertexNormals[ii]) )
                return SM_ERR;
        for( ULONG ii=0; ii<sAfter.m_sAuxNormals.GetSize(); ++ii )
            if( !sm_TestVectorFinite(sAfter.m_sAuxNormals[ii]) )
                return SM_ERR;

        SmTArray<SmPolyFace*> sFacesAfter;
        SmTArray<SmPolyEdge*> sEdgesAfter;
        pSingular->GetPolyFaces(sFacesAfter);
        pSingular->GetPolyEdges(sEdgesAfter);
        if( !(sFacesAfter == sFacesBefore) || !(sEdgesAfter == sEdgesBefore) )
            return SM_ERR;

        for( ULONG ii=0; ii<sFacesAfter.GetSize(); ++ii )
            if( !sm_TestVectorFinite(sFacesAfter[ii]->GetNormal(TRUE)) )
                return SM_ERR;

        SER( sm_TestAssemblyPolyBrepScale(pSource, sSingularScale) );
    }

    return SM_SUCCESS;
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmTransform()
{

    SmApiCreateContext();

    SmVector3d sPositon1(0.0,0.0,0.0);
   
    SmBrep* pCube = NULL;
    SmStatus stat = SmApiCreateBox(sPositon1, 5, 10, 15, pCube);

    SmAxis2Placement sTransform;
    sTransform.RotateAboutAxis( SM_DEGREES_TO_RADIANS( 45 ), SmVector3d( 0,1,0 ) );
    stat = SmApiTransform(pCube, sTransform);
    if( stat != SM_SUCCESS )
        return stat;

    SmPoint3d sLineStart(0.0, 0.0, 0.0);
    SmPoint3d sLineEnd(2.0, 0.0, 0.0);
    SmLine* pLine = NULL;
    SER( SmApiCreateLineSegment(sLineStart, sLineEnd, pLine) );
    SmObjDelete sLineCleanup(pLine);

    SmAxis2Placement sCurveTransform;
    sCurveTransform.Translate(SmVector3d(1.0, 2.0, 3.0));
    SER( SmApiTransform(pLine, sCurveTransform) );

    SmPoint3d sActualStart, sActualEnd;
    pLine->GetEnds(sActualStart, sActualEnd);
    if( !sm_TestPointClose(sActualStart, SmPoint3d(1.0, 2.0, 3.0)) ||
        !sm_TestPointClose(sActualEnd, SmPoint3d(3.0, 2.0, 3.0)) )
        return SM_ERR;

    SmPoint3d sCorner1(0.0, 0.0, 0.0);
    SmPoint3d sCorner2(2.0, 0.0, 0.0);
    SmPoint3d sCorner3(0.0, 2.0, 0.0);
    SmPoint3d sCorner4(2.0, 2.0, 0.0);
    SmSurface* pSurface = NULL;
    SER( SmApiCreateSurfaceFromCornerPoints(sCorner1, sCorner2, sCorner3, sCorner4, pSurface) );
    SmObjDelete sSurfaceCleanup(pSurface);
    if( pSurface == NULL || pSurface->GetOwner() != NULL )
        return SM_ERR;

    SmPoint3d sSurfacePointBefore;
    SER( sm_EvaluateSurfaceMidpoint(pSurface, sSurfacePointBefore) );
    SmVector3d sSurfaceTranslation(3.0, -2.0, 5.0);
    SmAxis2Placement sSurfaceTransform;
    sSurfaceTransform.Translate(sSurfaceTranslation);
    SER( SmApiTransform(pSurface, sSurfaceTransform) );

    SmPoint3d sSurfacePointAfter;
    SER( sm_EvaluateSurfaceMidpoint(pSurface, sSurfacePointAfter) );
    SmPoint3d sExpectedSurfacePoint(sSurfacePointBefore.x+sSurfaceTranslation.x,
                                    sSurfacePointBefore.y+sSurfaceTranslation.y,
                                    sSurfacePointBefore.z+sSurfaceTranslation.z);
    if( !sm_TestPointClose(sSurfacePointAfter, sExpectedSurfacePoint) )
        return SM_ERR;

    return SM_SUCCESS;
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmScale()
{

    SmApiCreateContext();

    SmVector3d sPositon1(-10.0,10.0,0.0);
   
    SmBrep* pCube = NULL;
    SmStatus stat = SmApiCreateBox(sPositon1, 5, 5, 5, pCube);

    SmVector3d sScale( 1, 2, 4 );
    SmApiScale(pCube, sScale);

    SmBrep* pCube2 = NULL;
    stat = SmApiCreateBox(sPositon1, 5, 5, 5, pCube2);

    SmVector3d sRefPoint( -10,10,0 );
    stat = SmApiScaleByPt( pCube2, sRefPoint, sScale);
    if( stat != SM_SUCCESS )
        return stat;

    SmPoint3d sCircleCenter(2.0, 0.0, 0.0);
    SmCircle* pCircle = NULL;
    SER( SmApiCreateCircle(sCircleCenter, 1.0, pCircle) );
    SmObjDelete sCircleCleanup(pCircle);

    SmVector3d sUniformScale(2.0, 2.0, 2.0);
    SER( SmApiScale(pCircle, sUniformScale) );
    SmAxis2Placement sCirclePlacement;
    double dCircleRadius = 0.0;
    SER( pCircle->GetCanonical(sCirclePlacement, dCircleRadius) );
    if( !sm_TestPointClose(sCirclePlacement.GetOrigin(), SmPoint3d(4.0, 0.0, 0.0)) ||
        std::fabs(dCircleRadius-2.0) > 1.0e-8 )
        return SM_ERR;

    SmVector3d sCurveRefPoint(1.0, 0.0, 0.0);
    SmVector3d sHalfScale(0.5, 0.5, 0.5);
    SER( SmApiScaleByPt(pCircle, sCurveRefPoint, sHalfScale) );
    SER( pCircle->GetCanonical(sCirclePlacement, dCircleRadius) );
    if( !sm_TestPointClose(sCirclePlacement.GetOrigin(), SmPoint3d(2.5, 0.0, 0.0)) ||
        std::fabs(dCircleRadius-1.0) > 1.0e-8 )
        return SM_ERR;

    const SmPoint3d sBeforeRejectedScale = sCirclePlacement.GetOrigin();
    const double dBeforeRejectedRadius = dCircleRadius;
    SmVector3d sNonUniformScale(2.0, 1.0, 1.0);
    if( SmApiScaleByPt(pCircle, sCurveRefPoint, sNonUniformScale) == SM_SUCCESS )
        return SM_ERR;
    SER( pCircle->GetCanonical(sCirclePlacement, dCircleRadius) );
    if( !sm_TestPointClose(sCirclePlacement.GetOrigin(), sBeforeRejectedScale) ||
        std::fabs(dCircleRadius-dBeforeRejectedRadius) > 1.0e-8 )
        return SM_ERR;

    return SM_SUCCESS;
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmRotate()
{

    SmApiCreateContext();

    SmVector3d sPositon1(0.0,0.0,0.0);
   
    SmBrep* pCube = NULL;
    SmStatus stat = SmApiCreateBox(sPositon1, 5, 5, 5, pCube);

    SmVector3d sAxis( 0,0,1 );
    double sAngleDeg = 45.0;
    stat = SmApiRotate(pCube, sPositon1, sAxis, sAngleDeg);
    if( stat != SM_SUCCESS )
        return stat;

    SmCircle* pCircle = NULL;
    SER( SmApiCreateCircle(sPositon1, 1.0, pCircle) );
    SmObjDelete sCircleCleanup(pCircle);

    SmVector3d sCurveAxis(0.0, 1.0, 0.0);
    SER( SmApiRotate(pCircle, sPositon1, sCurveAxis, 45.0) );
    SmAxis2Placement sCirclePlacement;
    double dCircleRadius = 0.0;
    SER( pCircle->GetCanonical(sCirclePlacement, dCircleRadius) );
    const double dRootHalf = std::sqrt(0.5);
    if( !sm_TestVectorClose(sCirclePlacement.GetZAxis(),
                            SmVector3d(dRootHalf, 0.0, dRootHalf)) )
        return SM_ERR;

    return SM_SUCCESS;
}


//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmTranslate()
{

    SmApiCreateContext();

    SmVector3d sPositon1(0.0,0.0,0.0);
   
    SmBrep* pCube = NULL;
    SmStatus stat = SmApiCreateBox(sPositon1, 5, 5, 5, pCube);

    SmVector3d sTranslate( 1, 2, 4 );
    stat = SmApiTranslate(pCube, sTranslate);
    if( stat != SM_SUCCESS )
        return stat;

    SmTArray<SmEdge*> sOwnedEdges;
    SER( SmApiGetEdges(pCube, sOwnedEdges) );
    if( sOwnedEdges.GetSize() == 0 ||
        SmApiTranslate(sOwnedEdges[0]->GetCurve(), sTranslate) == SM_SUCCESS )
        return SM_ERR;

    SmTArray<SmFace*> sOwnedFaces;
    SER( SmApiGetFaces(pCube, sOwnedFaces) );
    if( sOwnedFaces.GetSize() == 0 || sOwnedFaces[0]->GetSurface() == NULL )
        return SM_ERR;

    SmSurface* pOwnedSurface = sOwnedFaces[0]->GetSurface();
    SmPoint3d sOwnedSurfacePointBefore;
    SER( sm_EvaluateSurfaceMidpoint(pOwnedSurface, sOwnedSurfacePointBefore) );
    if( SmApiTranslate(pOwnedSurface, sTranslate) == SM_SUCCESS )
        return SM_ERR;

    SmPoint3d sOwnedSurfacePointAfter;
    SER( sm_EvaluateSurfaceMidpoint(pOwnedSurface, sOwnedSurfacePointAfter) );
    if( !sm_TestPointClose(sOwnedSurfacePointAfter, sOwnedSurfacePointBefore) )
        return SM_ERR;

    if( SmApiTranslate(sOwnedFaces[0], sTranslate) == SM_SUCCESS )
        return SM_ERR;
    if( !pCube->AssertValid() )
        return SM_ERR;

    SmCircle* pCircle = NULL;
    SER( SmApiCreateCircle(sPositon1, 1.0, pCircle) );
    SmObjDelete sCircleCleanup(pCircle);
    SER( SmApiTranslate(pCircle, sTranslate) );

    SmAxis2Placement sCirclePlacement;
    double dCircleRadius = 0.0;
    SER( pCircle->GetCanonical(sCirclePlacement, dCircleRadius) );
    if( !sm_TestPointClose(sCirclePlacement.GetOrigin(), SmPoint3d(1.0, 2.0, 4.0)) )
        return SM_ERR;

    SmPoint3d sPolyBoxPosition(10.0, 20.0, 30.0);
    SmBrep* pPolySource = NULL;
    SER( SmApiCreateBox(sPolyBoxPosition, 2.0, 3.0, 4.0, pPolySource) );
    SmObjDelete sPolySourceCleanup(pPolySource);
    if( pPolySource == NULL )
        return SM_ERR;

    SmPolyBrep* pPolyBrep = NULL;
    SmTessellationParams sPolyParams;
    sPolyParams.dChordHeightTol     = 0.1;
    sPolyParams.dCurveAngleTolDeg   = 15.0;
    sPolyParams.dSurfaceAngleTolDeg = 15.0;
    sPolyParams.dMaxEdgeLength      = 0.0;
    sPolyParams.dMaxAspectRatio     = 0.0;
    SER( SmApiTessellate(pPolySource, pPolyBrep, sPolyParams) );
    SmObjDelete sPolyBrepCleanup(pPolyBrep);
    if( pPolyBrep == NULL )
        return SM_ERR;

    SmTArray<SmPoint3d> sPolyPointsBefore;
    pPolyBrep->GetPolyPoints(sPolyPointsBefore);
    if( sPolyPointsBefore.GetSize() == 0 )
        return SM_ERR;

    SmVector3d sPolyTranslation(-4.0, 6.0, 2.0);
    SER( SmApiTranslate(pPolyBrep, sPolyTranslation) );
    SmTArray<SmPoint3d> sPolyPointsAfter;
    pPolyBrep->GetPolyPoints(sPolyPointsAfter);
    if( sPolyPointsAfter.GetSize() != sPolyPointsBefore.GetSize() )
        return SM_ERR;

    for( ULONG ii=0; ii<sPolyPointsBefore.GetSize(); ++ii )
    {
        SmPoint3d sExpectedPoint(sPolyPointsBefore[ii].x+sPolyTranslation.x,
                                 sPolyPointsBefore[ii].y+sPolyTranslation.y,
                                 sPolyPointsBefore[ii].z+sPolyTranslation.z);
        if( !sm_TestPointClose(sPolyPointsAfter[ii], sExpectedPoint) )
            return SM_ERR;
    }

    SmPolyBrepTransformSnapshot sBeforeNormalTransform;
    sm_CapturePolyBrepTransformState(pPolyBrep, sBeforeNormalTransform);
    if( sBeforeNormalTransform.m_sVertexNormals.GetSize() == 0 ||
        sBeforeNormalTransform.m_sAuxNormals.GetSize() == 0 )
        return SM_ERR;

    SmVector3d sPolyRotationCenter(0.0, 0.0, 0.0);
    SmVector3d sPolyRotationAxis(1.0, 2.0, 3.0);
    SER( sPolyRotationAxis.Unitize() );
    const double dPolyRotationDeg = 31.0;
    SmAxis2Placement sPolyRotation;
    sPolyRotation.RotateAboutAxisAtPoint(
        SM_DEGREES_TO_RADIANS(dPolyRotationDeg), sPolyRotationCenter, sPolyRotationAxis);
    SER( SmApiRotate(pPolyBrep, sPolyRotationCenter, sPolyRotationAxis, dPolyRotationDeg) );

    SmVector3d sPolyScale(2.0, 3.0, 4.0);
    SER( SmApiScale(pPolyBrep, sPolyScale) );
    SER( sm_TestPolyBrepNormalsAfterTransform(
        pPolyBrep, sBeforeNormalTransform, sPolyRotation, sPolyScale) );

    SmPolyBrepTransformSnapshot sBeforeRejectedScales;
    sm_CapturePolyBrepTransformState(pPolyBrep, sBeforeRejectedScales);

    SmVector3d sZeroScale(1.0, 0.0, 1.0);
    if( SmApiScale(pPolyBrep, sZeroScale) != SM_ERR_INVALID_INPUT ||
        !sm_TestPolyBrepTransformStateClose(pPolyBrep, sBeforeRejectedScales) )
        return SM_ERR;

    SmVector3d sNegativeScale(-1.0, 1.0, 1.0);
    if( SmApiScale(pPolyBrep, sNegativeScale) != SM_ERR_INVALID_INPUT ||
        !sm_TestPolyBrepTransformStateClose(pPolyBrep, sBeforeRejectedScales) )
        return SM_ERR;

    SmVector3d sNanScale(std::numeric_limits<double>::quiet_NaN(), 1.0, 1.0);
    if( SmApiScale(pPolyBrep, sNanScale) != SM_ERR_INVALID_INPUT ||
        !sm_TestPolyBrepTransformStateClose(pPolyBrep, sBeforeRejectedScales) )
        return SM_ERR;

    SmVector3d sInfiniteScale(std::numeric_limits<double>::infinity(), 1.0, 1.0);
    if( SmApiScale(pPolyBrep, sInfiniteScale) != SM_ERR_INVALID_INPUT ||
        !sm_TestPolyBrepTransformStateClose(pPolyBrep, sBeforeRejectedScales) )
        return SM_ERR;

    SER( sm_TestPolyBrepKernelScaleCompatibility(pPolyBrep) );

    return SM_SUCCESS;
}



//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmGetClosestPoint()
{
    SmApiCreateContext();

    SmVector3d sOrigin(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sOrigin, 10, 10, 10, pBox);
    if (!pBox)
        return SM_ERR;

    SmPoint3d sQueryPt(5.0, 5.0, 15.0);
    SmPoint3d sClosest;
    double dDist = 0;
    SmStatus stat = SmApiGetClosestPoint(pBox, sQueryPt, sClosest, dDist);

    delete pBox;

    if (stat != SM_SUCCESS)
        return stat;
    if (dDist < 4.0 || dDist > 6.0)
        return SM_ERR;

    return SM_SUCCESS;
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmGetEdges()
{
    SmApiCreateContext();

    SmVector3d sOrigin(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sOrigin, 10, 10, 10, pBox);
    if (!pBox)
        return SM_ERR;

    SmTArray<SmEdge*> sEdges;
    SmStatus stat = SmApiGetEdges(pBox, sEdges);

    delete pBox;

    if (stat != SM_SUCCESS)
        return stat;
    if (sEdges.GetSize() < 12)
        return SM_ERR;

    return SM_SUCCESS;
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmGetFaces()
{
    SmApiCreateContext();

    SmVector3d sOrigin(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sOrigin, 10, 10, 10, pBox);
    if (!pBox)
        return SM_ERR;

    SmTArray<SmFace*> sFaces;
    SmStatus stat = SmApiGetFaces(pBox, sFaces);

    delete pBox;

    if (stat != SM_SUCCESS)
        return stat;
    if (sFaces.GetSize() != 6)
        return SM_ERR;

    return SM_SUCCESS;
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmFindEdge()
{
    SmApiCreateContext();

    SmVector3d sOrigin(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sOrigin, 10, 10, 10, pBox);
    if (!pBox)
        return SM_ERR;

    // Each probe lies on a different box edge; the found edge and parameter
    // must evaluate back to the probe, not merely to some edge.
    const SmPoint3d sProbes[] = { SmPoint3d(5.0, 0.0, 0.0), SmPoint3d(0.0, 5.0, 0.0),
                                  SmPoint3d(10.0, 10.0, 5.0), SmPoint3d(0.0, 0.0, 5.0),
                                  SmPoint3d(5.0, 10.0, 10.0) };
    SmTArray<SmEdge*> sEdges;
    SmStatus stat = SmApiGetEdges(pBox, sEdges);
    for (ULONG ii = 0; stat == SM_SUCCESS && ii < sizeof(sProbes) / sizeof(sProbes[0]); ii++)
    {
        SmPoint3d sQueryPt = sProbes[ii];
        ULONG lEdgeIndex = 0;
        double dParam = 0;
        stat = SmApiFindEdge(pBox, sQueryPt, lEdgeIndex, dParam);
        if (stat != SM_SUCCESS)
            break;
        SmPoint3d sOnEdge;
        if (   lEdgeIndex >= sEdges.GetSize()
            || sEdges[lEdgeIndex]->GetCurve()->EvaluatePoint(dParam, sOnEdge) != SM_SUCCESS
            || sOnEdge.DistanceBetween(sQueryPt) > 1.0e-6)
            stat = SM_ERR;
    }

    delete pBox;
    return stat;
}

//*************************************************************************
// 
//*************************************************************************

SmStatus TestSmFindFaces()
{
    SmApiCreateContext();

    SmVector3d sOrigin(0.0, 0.0, 0.0);
    SmBrep* pBox = NULL;
    SmApiCreateBox(sOrigin, 10, 10, 10, pBox);
    if (!pBox)
        return SM_ERR;

    SmPoint3d sQueryPt(5.0, 5.0, 10.0);
    SmFace* pFace = NULL;
    SmStatus stat = SmApiFindFaces(pBox, sQueryPt, pFace);

    delete pBox;

    if (stat != SM_SUCCESS)
        return stat;
    if (pFace == NULL)
        return SM_ERR;

    return SM_SUCCESS;
}


//*************************************************************************
// SmApiPolyBrepComputeMassProperties must not depend on winding direction.
// A closed mesh wound inside-out (as imported, or after a mirror) passes
// IsManifoldSolid but has negated signed integrals; the API must return the
// same result as the correctly wound mesh rather than reject it.
//*************************************************************************

SmStatus TestSmPolyBrepMassPropertiesWinding()
{
    SmApiCreateContext();

    SmVector3d sCorner(1.0, 2.0, 3.0);
    SmBrep* pBox = NULL;
    SER( SmApiCreateBox(sCorner, 2.0, 4.0, 6.0, pBox) );
    SmPolyBrep* pMesh = NULL;
    // Explicit controls so the mesh does not depend on the library defaults.
    SmTessellationParams sParams;
    sParams.dChordHeightTol     = 0.1;
    sParams.dCurveAngleTolDeg   = 0.0;
    sParams.dSurfaceAngleTolDeg = 0.0;
    sParams.dMaxEdgeLength      = 0.0;
    sParams.dMaxAspectRatio     = 0.0;
    SER( SmApiTessellate(pBox, pMesh, sParams) );
    SmPolyBrep* pReversed = NULL;
    SER( SmApiPolyBrepCopy(pMesh, pReversed) );

    SmTArray<SmPolyFace*> sFaces;
    pReversed->GetPolyFaces(sFaces);
    for( ULONG ii=0; ii<sFaces.GetSize(); ++ii )
        SER( sFaces[ii]->ReverseOrientation() );

    // Premise: the flipped mesh is still a closed manifold with negative
    // signed volume, so it exercises the reversed-winding path.
    SmBoolean bManifold = FALSE;
    SER( SmApiPolyBrepIsManifoldSolid(pReversed, bManifold) );
    double dSignedVolume = 0.0;
    SER( SmApiPolyBrepComputeVolume(pReversed, dSignedVolume) );
    if( !bManifold || dSignedVolume >= 0.0 )
        return SM_ERR;

    const SmPoint3d sOrigin(0.0, 0.0, 0.0);
    double aArea[2], aVolume[2], aMass[2];
    SmPoint3d aCentroid[2];
    SmVector3d aMoi[2], aPoi[2];
    SmPolyBrep* apMesh[2] = { pMesh, pReversed };
    for( int kk=0; kk<2; ++kk )
        SER( SmApiPolyBrepComputeMassProperties(apMesh[kk], 2.0, sOrigin,
                 aArea[kk], aVolume[kk], aMass[kk], aCentroid[kk], aMoi[kk], aPoi[kk]) );

    auto close = [](double a, double b) { return std::fabs(a-b) <= 1.0e-9*std::max(1.0, std::fabs(b)); };
    if( !close(aVolume[0], 48.0) || !close(aVolume[1], 48.0) )
        return SM_ERR;
    if( !close(aArea[1], aArea[0]) || !close(aMass[1], aMass[0]) )
        return SM_ERR;
    for( int cc=0; cc<3; ++cc )
    {
        if(    !close(aCentroid[1][cc], aCentroid[0][cc])
            || !close(aMoi[1][cc], aMoi[0][cc])
            || !close(aPoi[1][cc], aPoi[0][cc]) )
            return SM_ERR;
    }

    return SM_SUCCESS;
}

//*************************************************************************
// smsurf_EvaluateGeometric must pair each principal direction with its own
// curvature when the UV frame is skewed. This exact evaluation has principal
// curvatures 1 and 0, with directions (5,-12,0)/13 and (12,5,0)/13.
//*************************************************************************

SmStatus TestSmPrincipalDirectionPairing()
{
    SmVector3d aEval[3][3];
    for( ULONG i=0; i<3; ++i )
        for( ULONG j=0; j<3; ++j )
            aEval[i][j] = SmVector3d(0.0, 0.0, 0.0);

    aEval[1][0] = SmVector3d(1.0, 0.0, 0.0);          // Du
    aEval[0][1] = SmVector3d(1.0, 1.0, 0.0);          // Dv
    aEval[2][0] = SmVector3d(0.0, 0.0, 25.0/169.0);   // Duu
    aEval[1][1] = SmVector3d(0.0, 0.0, -35.0/169.0);  // Duv
    aEval[0][2] = SmVector3d(0.0, 0.0, 49.0/169.0);   // Dvv

    double dK, dH, dK1, dK2;
    SmVector3d sEFG, sLMN, sV1, sV2;
    SER( smsurf_EvaluateGeometric(aEval, dK, dH, dK1, dK2, sEFG, sLMN, sV1, sV2) );

    auto close = [](double a, double b) { return smos_Fabs(a-b) <= 1.0e-12; };
    auto sameDirection = [](const SmVector3d& a, const SmVector3d& b)
        { return smos_Fabs(smos_Fabs(a.Dot(b))-1.0) <= 1.0e-12; };

    if( !close(dK, 0.0) || !close(dH, 0.5) || !close(dK1, 1.0) || !close(dK2, 0.0) )
        return SM_ERR;
    if( !close(sEFG.x, 1.0) || !close(sEFG.y, 1.0) || !close(sEFG.z, 2.0) )
        return SM_ERR;
    if( !close(sLMN.x, 25.0/169.0) || !close(sLMN.y, -35.0/169.0) || !close(sLMN.z, 49.0/169.0) )
        return SM_ERR;

    const SmVector3d sExpectedV1(5.0/13.0, -12.0/13.0, 0.0);
    const SmVector3d sExpectedV2(12.0/13.0, 5.0/13.0, 0.0);
    if( !sameDirection(sV1, sExpectedV1) || !sameDirection(sV2, sExpectedV2) )
        return SM_ERR;
    if( !close(sV1.Length(), 1.0) || !close(sV2.Length(), 1.0) || !close(sV1.Dot(sV2), 0.0) )
        return SM_ERR;
    if( !close((sV1*sV2).z, 1.0) )
        return SM_ERR;

    SmVector3d sSectionDirection;
    double dSectionCurvature;
    SER( smsurf_EvaluateNormalSection(aEval, sV1, sSectionDirection, dSectionCurvature) );
    if( !sameDirection(sSectionDirection, sV1) || !close(dSectionCurvature, dK1) )
        return SM_ERR;
    SER( smsurf_EvaluateNormalSection(aEval, sV2, sSectionDirection, dSectionCurvature) );
    if( !sameDirection(sSectionDirection, sV2) || !close(dSectionCurvature, dK2) )
        return SM_ERR;

    return SM_SUCCESS;
}

//*************************************************************************
// SmCone::EvaluateSTEP must return the cross derivative Duv, not zero. For a
// cone Duv = Rv * (-sinU X + cosU Y); check it against a central difference of
// EvaluateSTEP's own Du in V, so the check is independent of the U units.
//*************************************************************************

SmStatus TestSmConeSTEPCrossDerivative()
{
    SmApiCreateContext();
    SmContext& rContext = *SmApiGetOrCreateContext();
    SmAxis2Placement sPlacement(SmPoint3d(0, 0, 0), SmVector3d(1, 0, 0), SmVector3d(0, 1, 0));
    SmCone* pCone = NULL;
    SER( SmCone::CreateCanonical(rContext, sPlacement, 1.0, 2.0, 3.0, pCone) );
    if( pCone == NULL )
        return SM_ERR;
    SmObjDelete sConeCleanup(pCone);

    const double dH = 1.0e-5;
    const double aU[] = { 0.0, 37.0, 135.0, 250.0 };
    const double aV[] = { 0.5, 1.5, 2.5 };
    for( double dU : aU )
    {
        for( double dV : aV )
        {
            SmVector3d aD[9], aDm[9], aDp[9];   // 3x3: [D, Dv, Dvv, Du, Duv, Duvv, Duu, Duuv, Duuvv]
            SER( pCone->EvaluateSTEP(SmPoint2d(dU, dV),      2, 2, TRUE, TRUE, TRUE, aD) );
            SER( pCone->EvaluateSTEP(SmPoint2d(dU, dV - dH), 2, 2, TRUE, TRUE, TRUE, aDm) );
            SER( pCone->EvaluateSTEP(SmPoint2d(dU, dV + dH), 2, 2, TRUE, TRUE, TRUE, aDp) );
            const SmVector3d sFD  = (aDp[3] - aDm[3]) / (2.0 * dH);
            const SmVector3d sErr = aD[4] - sFD;
            if( sFD.Length() < 1.0e-3 || sErr.Length() > 1.0e-6 * sFD.Length() )
                return SM_ERR;
        }
    }
    return SM_SUCCESS;
}

//*************************************************************************
// Retained edge curves must not keep a stale owner. Exercise replacement,
// same-pointer assignment, ownership transfer, restoration, and destruction
// after detachment with notifications both enabled and disabled.
//*************************************************************************

SmStatus TestSmEdgeCurveOwnership()
{
    SmApiCreateContext();
    for( int iNotify = 0; iNotify < 2; ++iNotify )
    {
        const SmBoolean bNotify = iNotify != 0;
        SmVector3d sOrigin(0,0,0);
        SmBrep* pBrep = NULL;
        SER( SmApiCreateBox(sOrigin, 1, 1, 1, pBrep) );
        SmObjDelete sBrepCleanup(pBrep);
        SmTArray<SmEdge*> sEdges;
        pBrep->GetEdges(sEdges);
        if( sEdges.GetSize() < 2 )
            return SM_ERR;
        SmEdge* pEdge = sEdges[0];
        SmCurve* pOriginal = pEdge->GetCurve();
        if( pOriginal == NULL || pOriginal->GetOwner() != pEdge )
            return SM_ERR;

        // A same-pointer assignment must not clear ownership or delete it.
        pEdge->SetCurve(pOriginal, TRUE, FALSE, bNotify);
        if( pEdge->GetCurve() != pOriginal || pOriginal->GetOwner() != pEdge )
            return SM_ERR;

        SmCurve* pReplacement = NULL;
        SER( pOriginal->Copy(*pBrep->GetContext(), pReplacement) );
        SmObjDelete sReplacementCleanup(pReplacement);
        pEdge->SetCurve(pReplacement, FALSE, FALSE, bNotify);
        SmObjDelete sOriginalCleanup(pOriginal);
        if( pOriginal->GetOwner() != NULL || pReplacement->GetOwner() != NULL )
        {
            // Keep failure cleanup safe when run against the old setter.
            pEdge->SetCurve(NULL, FALSE, FALSE, FALSE);
            pOriginal->SetOwner(NULL);
            return SM_ERR;
        }
        // SetCurve does not assign the new owner; callers still do that.
        pReplacement->SetOwner(pEdge);
        sReplacementCleanup.Clear();

        // Restore a retained curve, as TrimCliffRails does for a closed rail.
        pEdge->SetCurve(pOriginal, TRUE, FALSE, bNotify);
        pOriginal->SetOwner(pEdge);
        sOriginalCleanup.Clear();

        // Transfer ownership before detaching from the previous edge. The
        // setter must not erase the owner already established by the caller.
        SmEdge* pOtherEdge = sEdges[1];
        pOtherEdge->SetCurve(pOriginal, TRUE, FALSE, bNotify);
        pOriginal->SetOwner(pOtherEdge);
        pEdge->SetCurve(NULL, FALSE, FALSE, bNotify);
        if( pOriginal->GetOwner() != pOtherEdge || pEdge->GetCurve() != NULL )
            return SM_ERR;

        SmPoint3d sBefore, sAfter;
        const double dParam = pOriginal->GetNaturalInterval().Evaluate(0.5);
        SER( pOriginal->EvaluatePoint(dParam, sBefore) );
        pOtherEdge->SetCurve(NULL, FALSE, FALSE, bNotify);
        SmObjDelete sDetachedCleanup(pOriginal);
        if( pOriginal->GetOwner() != NULL || pOtherEdge->GetCurve() != NULL )
            return SM_ERR;
        // A partial Brep with detached curves must be safe to destroy, while
        // the caller's geometry remains usable and independently deletable.
        sBrepCleanup.DeleteObj();
        SER( pOriginal->EvaluatePoint(dParam, sAfter) );
        if( !sm_TestPointClose(sAfter, sBefore) )
            return SM_ERR;
    }
    return SM_SUCCESS;
}

//*************************************************************************
// A failed SmApiCreatePlanarFaces must set rpResult to NULL and leave the input
// curves with the caller (it used to leak its empty Brep and leave rpResult
// unchanged). An open loop cannot bound a planar face.
//*************************************************************************

SmStatus TestSmPlanarFacesFailureCleanup()
{
    SmApiCreateContext();
    SmPoint3d sP0(0,0,0), sP1(1,0,0), sP2(0,1,0);
    SmLine *pL0 = NULL, *pL1 = NULL;
    SER( SmApiCreateLineSegment(sP0, sP1, pL0) );
    SER( SmApiCreateLineSegment(sP1, sP2, pL1) );
    SmTArray<SmCurve*> sOpenLoop;
    sOpenLoop.Add(pL0);
    sOpenLoop.Add(pL1);

    // Start from a non-NULL handle so an unchanged output is detected.
    SmVector3d sOrigin(0,0,0);
    SmBrep* pSentinel = NULL;
    SER( SmApiCreateBox(sOrigin, 1, 1, 1, pSentinel) );
    SmObjDelete sSentinelCleanup(pSentinel);
    SmBrep* pResult = pSentinel;

    const SmApiStatus eStat = SmApiCreatePlanarFaces(sOpenLoop, pResult);

    // The caller still owns the curves after a failure.
    delete pL0;
    delete pL1;

    if( eStat == SM_SUCCESS || pResult != NULL )
        return SM_ERR;
    return SM_SUCCESS;
}

//*************************************************************************
// Failed curve sweeps, including the face/edge wrappers, must set rpResult to
// NULL (they used to return a half-built Brep). Success and failure must both
// preserve the caller's scale curve, including its parameterization. Capping
// an open or nonplanar profile exercises cleanup after scale processing.
//*************************************************************************

SmStatus TestSmCurveSweepFailureCleanup()
{
    struct CurveSnapshot
    {
        SmExtent1d m_sInterval;
        SmPoint3d m_sStartPoint;
        SmPoint3d m_sMidPoint;
        SmPoint3d m_sEndPoint;
        double m_dLength;
    };

    auto SnapshotCurve = [](SmCurve* pCurve, CurveSnapshot& rSnapshot) -> SmStatus
    {
        if( pCurve == NULL )
            return SM_ERR_NULL_POINTER;
        rSnapshot.m_sInterval = pCurve->GetNaturalInterval();
        SER( pCurve->EvaluatePoint(rSnapshot.m_sInterval.GetMin(), rSnapshot.m_sStartPoint) );
        SER( pCurve->EvaluatePoint(rSnapshot.m_sInterval.Evaluate(0.5), rSnapshot.m_sMidPoint) );
        SER( pCurve->EvaluatePoint(rSnapshot.m_sInterval.GetMax(), rSnapshot.m_sEndPoint) );
        rSnapshot.m_dLength = pCurve->ApproximateLength(rSnapshot.m_sInterval, 20);
        return SM_SUCCESS;
    };

    auto CurveIsUnchanged = [&SnapshotCurve](SmCurve* pCurve, const CurveSnapshot& rExpected) -> SmBoolean
    {
        CurveSnapshot sActual;
        if( SnapshotCurve(pCurve, sActual) != SM_SUCCESS )
            return FALSE;
        return fabs(sActual.m_sInterval.GetMin() - rExpected.m_sInterval.GetMin()) < 1.0e-12 &&
               fabs(sActual.m_sInterval.GetMax() - rExpected.m_sInterval.GetMax()) < 1.0e-12 &&
               sActual.m_sStartPoint.DistanceBetween(rExpected.m_sStartPoint) < 1.0e-8 &&
               sActual.m_sMidPoint.DistanceBetween(rExpected.m_sMidPoint) < 1.0e-8 &&
               sActual.m_sEndPoint.DistanceBetween(rExpected.m_sEndPoint) < 1.0e-8 &&
               fabs(sActual.m_dLength - rExpected.m_dLength) < 1.0e-8;
    };

    SmApiCreateContext();
    const SmContext& rContext = *SmApiGetOrCreateContext();
    SmBSplineCurve *pPath = NULL, *pProfile = NULL, *pScaleCurve = NULL;
    SER( SmBSplineCurve::CreateLineSegment(rContext, 3, SmPoint3d(0,0,0), SmPoint3d(0,0,10), pPath) );
    SER( SmBSplineCurve::CreateLineSegment(rContext, 3, SmPoint3d(1,0,0), SmPoint3d(0,1,0), pProfile) );
    SER( SmBSplineCurve::CreateLineSegment(rContext, 3, SmPoint3d(1,0,0), SmPoint3d(2,0,10), pScaleCurve) );
    SmObjDelete sPathCleanup(pPath), sProfileCleanup(pProfile), sScaleCleanup(pScaleCurve);
    SER( pScaleCurve->EditParameterization(SmExtent1d(4.0, 9.0)) );
    CurveSnapshot sScaleSnapshot;
    SER( SnapshotCurve(pScaleCurve, sScaleSnapshot) );

    SmTArray<SmCurve*> sProfile;
    sProfile.Add(pProfile);

    SmSweepOptions sOptions;
    sOptions.bCapEndsArg = TRUE;   // an open profile cannot be capped

    // Start from a non-NULL handle so an unchanged output is detected.
    SmVector3d sOrigin(0,0,0);
    SmBrep* pSentinel = NULL;
    SER( SmApiCreateBox(sOrigin, 1, 1, 1, pSentinel) );
    SmObjDelete sSentinelCleanup(pSentinel);
    SmBrep* pResult = pSentinel;

    const SmApiStatus eStat = SmApiCreateCurveSweep(
        sProfile, pPath, pPath, pScaleCurve, &sOptions, pResult, NULL, NULL, NULL );
    if( eStat == SM_SUCCESS || pResult != NULL || !CurveIsUnchanged(pScaleCurve, sScaleSnapshot) )
        return SM_ERR;

    SmVector3d sCircleCenter(0,0,0);
    SmBSplineCurve* pCircle = NULL;
    SER( SmApiCreateArc(sCircleCenter, 1.0, 0.0, 360.0, pCircle) );
    SmObjDelete sCircleCleanup(pCircle);
    SmTArray<SmCurve*> sClosedProfile;
    sClosedProfile.Add(pCircle);
    SmBrep* pSuccessResult = NULL;
    const SmApiStatus eSuccessStat = SmApiCreateCurveSweep(
        sClosedProfile, pPath, pPath, pScaleCurve, &sOptions, pSuccessResult, NULL, NULL, NULL );
    SmObjDelete sSuccessCleanup(pSuccessResult);
    if( eSuccessStat != SM_SUCCESS || pSuccessResult == NULL ||
        !CurveIsUnchanged(pScaleCurve, sScaleSnapshot) )
        return SM_ERR;

    // A valid face has a closed boundary. Use a nonplanar bilinear patch so
    // the face wrapper reaches the kernel but cannot create planar end caps.
    // A single edge of the same patch supplies the open-profile edge case.
    SmSurface* pPatch = NULL;
    SER( SmApiCreateSurfaceFromCornerPoints(SmPoint3d(0,0,0), SmPoint3d(2,0,0),
        SmPoint3d(0,2,0), SmPoint3d(2,2,1), pPatch) );
    SmBrep* pProfileBrep = new (rContext) SmBrep();
    SmObjDelete sProfileBrepCleanup(pProfileBrep);
    SmFace* pProfileFace = NULL;
    SER( pProfileBrep->CreateFaceFromSurface(pPatch, pPatch->GetNaturalUVDomain(), pProfileFace) );
    SmTArray<SmFace*> sProfileFaces;
    sProfileFaces.Add(pProfileFace);
    SmTArray<SmEdge*> sBoundaryEdges;
    pProfileFace->GetEdges(sBoundaryEdges);
    if( sBoundaryEdges.GetSize() == 0 )
        return SM_ERR;
    SmTArray<SmEdge*> sProfileEdges;
    sProfileEdges.Add(sBoundaryEdges[0]);

    SmTArray<CurveSnapshot> sBoundarySnapshots;
    sBoundarySnapshots.SetSize(sBoundaryEdges.GetSize());
    for( ULONG ii = 0; ii < sBoundaryEdges.GetSize(); ++ii )
        SER( SnapshotCurve(sBoundaryEdges[ii]->GetCurve(), sBoundarySnapshots[ii]) );
    CurveSnapshot sPathSnapshot;
    SER( SnapshotCurve(pPath, sPathSnapshot) );

    for( int iWrapper = 0; iWrapper < 2; ++iWrapper )
    {
        // The same inputs succeed without caps and fail with caps. This
        // distinguishes a late cap failure from an early preflight rejection.
        for( int iCapEnds = 0; iCapEnds < 2; ++iCapEnds )
        {
            sOptions.bCapEndsArg = (iCapEnds != 0);
            pResult = pSentinel;
            const SmApiStatus eWrapperStat = iWrapper == 0 ?
                SmApiCreateCurveSweepFromFaces(sProfileFaces, pPath, pPath, pScaleCurve,
                    &sOptions, pResult, NULL, NULL, NULL) :
                SmApiCreateCurveSweepFromEdges(sProfileEdges, pPath, pPath, pScaleCurve,
                    &sOptions, pResult, NULL, NULL, NULL);
            // Do not delete the independently guarded sentinel if a wrapper
            // regresses to leaving the output unchanged on failure.
            SmObjDelete sWrapperResultCleanup(pResult == pSentinel ? NULL : pResult);
            if( iCapEnds != 0 )
            {
                if( eWrapperStat == SM_SUCCESS || pResult != NULL )
                    return SM_ERR;
            }
            else if( eWrapperStat != SM_SUCCESS || pResult == NULL || pResult == pSentinel )
                return SM_ERR;

            if( !CurveIsUnchanged(pScaleCurve, sScaleSnapshot) ||
                !CurveIsUnchanged(pPath, sPathSnapshot) )
                return SM_ERR;
            for( ULONG ii = 0; ii < sBoundaryEdges.GetSize(); ++ii )
                if( !CurveIsUnchanged(sBoundaryEdges[ii]->GetCurve(), sBoundarySnapshots[ii]) )
                    return SM_ERR;
        }
    }
    return SM_SUCCESS;
}

//*************************************************************************
// EvaluateSTEP derivatives must be with respect to the STEP parameters, whose
// angles are in degrees. Compare every populated derivative through order 3
// with central differences in the surface's own parameters. Also verify that
// the unused portion of the rectangular output array remains untouched.
//*************************************************************************

static SmStatus sm_CheckSTEPDerivatives(const SmSurface* pSurface, const SmPoint2d& crUV, double dHu, double dHv)
{
    const ULONG lOrder = 3;
    const ULONG lStride = lOrder+1;
    const ULONG lNumDerivatives = lStride*lStride;
    const SmVector3d sUntouched(12345.678, -23456.789, 34567.891);
    SmVector3d aD[16], aUm[16], aUp[16], aVm[16], aVp[16];
    auto initialize = [&](SmVector3d* aDerivatives)
        {
            for( ULONG ii=0; ii<lNumDerivatives; ++ii )
                aDerivatives[ii] = sUntouched;
        };
    initialize(aD);
    initialize(aUm);
    initialize(aUp);
    initialize(aVm);
    initialize(aVp);

    SER( pSurface->EvaluateSTEP(crUV,                                  lOrder, lOrder, TRUE, TRUE, TRUE, aD) );
    SER( pSurface->EvaluateSTEP(SmPoint2d(crUV.x - dHu, crUV.y),       lOrder, lOrder, TRUE, TRUE, TRUE, aUm) );
    SER( pSurface->EvaluateSTEP(SmPoint2d(crUV.x + dHu, crUV.y),       lOrder, lOrder, TRUE, TRUE, TRUE, aUp) );
    SER( pSurface->EvaluateSTEP(SmPoint2d(crUV.x,       crUV.y - dHv), lOrder, lOrder, TRUE, TRUE, TRUE, aVm) );
    SER( pSurface->EvaluateSTEP(SmPoint2d(crUV.x,       crUV.y + dHv), lOrder, lOrder, TRUE, TRUE, TRUE, aVp) );

    auto untouched = [&](const SmVector3d* aDerivatives)
        {
            for( ULONG i=1; i<=lOrder; ++i )
            {
                for( ULONG j=1; j<=lOrder; ++j )
                {
                    if( i+j <= lOrder )
                        continue;
                    const SmVector3d& rValue = aDerivatives[i*lStride+j];
                    if( rValue.x != sUntouched.x || rValue.y != sUntouched.y || rValue.z != sUntouched.z )
                        return FALSE;
                }
            }
            return TRUE;
        };
    if( !untouched(aD) || !untouched(aUm) || !untouched(aUp) || !untouched(aVm) || !untouched(aVp) )
        return SM_ERR;

    auto close = [](const SmVector3d& a, const SmVector3d& b)
        { return (a - b).Length() <= 1.0e-9 + 1.0e-4*b.Length(); };
    if( !close(aD[4],  (aUp[0] - aUm[0]) / (2.0*dHu)) ) return SM_ERR;  // Du
    if( !close(aD[1],  (aVp[0] - aVm[0]) / (2.0*dHv)) ) return SM_ERR;  // Dv
    if( !close(aD[8],  (aUp[4] - aUm[4]) / (2.0*dHu)) ) return SM_ERR;  // Duu
    if( !close(aD[5],  (aVp[4] - aVm[4]) / (2.0*dHv)) ) return SM_ERR;  // Duv
    if( !close(aD[2],  (aVp[1] - aVm[1]) / (2.0*dHv)) ) return SM_ERR;  // Dvv
    if( !close(aD[12], (aUp[8] - aUm[8]) / (2.0*dHu)) ) return SM_ERR;  // Duuu
    if( !close(aD[9],  (aVp[8] - aVm[8]) / (2.0*dHv)) ) return SM_ERR;  // Duuv
    if( !close(aD[6],  (aVp[5] - aVm[5]) / (2.0*dHv)) ) return SM_ERR;  // Duvv
    if( !close(aD[3],  (aVp[2] - aVm[2]) / (2.0*dHv)) ) return SM_ERR;  // Dvvv
    return SM_SUCCESS;
}

SmStatus TestSmSTEPDerivativeUnits()
{
    SmApiCreateContext();
    SmContext& rContext = *SmApiGetOrCreateContext();
    SmAxis2Placement sPlacement(SmPoint3d(0, 0, 0), SmVector3d(1, 0, 0), SmVector3d(0, 1, 0));

    SmCylinder* pCylinder = NULL;
    SER( SmCylinder::CreateCanonical(rContext, sPlacement, 2.0, pCylinder) );
    SmObjDelete sC1(pCylinder);
    if( pCylinder == NULL ) return SM_ERR;

    SmCone* pCone = NULL;
    SER( SmCone::CreateCanonical(rContext, sPlacement, 1.0, 2.0, 3.0, pCone) );
    SmObjDelete sC2(pCone);
    if( pCone == NULL ) return SM_ERR;

    SmSphere* pSphere = NULL;
    SER( SmSphere::CreateCanonical(rContext, sPlacement, 2.0, pSphere) );
    SmObjDelete sC3(pSphere);
    if( pSphere == NULL ) return SM_ERR;

    SmTorus* pTorus = NULL;
    SER( SmTorus::CreateCanonical(rContext, sPlacement, 5.0, 1.0, pTorus) );
    SmObjDelete sC4(pTorus);
    if( pTorus == NULL ) return SM_ERR;

    SmTArray<SmPoint3d> sGeneratorControlPoints;
    sGeneratorControlPoints.Add(SmPoint3d(2.0,  0.0, 0.0));
    sGeneratorControlPoints.Add(SmPoint3d(2.5,  0.4, 0.5));
    sGeneratorControlPoints.Add(SmPoint3d(3.2, -0.2, 1.7));
    sGeneratorControlPoints.Add(SmPoint3d(2.7,  0.3, 3.0));
    SmTArray<ULONG> sGeneratorKnotMultiplicities;
    sGeneratorKnotMultiplicities.Add(4);
    sGeneratorKnotMultiplicities.Add(4);
    SmTArray<double> sGeneratorKnots;
    sGeneratorKnots.Add(0.0);
    sGeneratorKnots.Add(1.0);
    SmBSplineCurve* pGenerator = NULL;
    SER( SmBSplineCurve::CreateCanonical(
        rContext, 3, 3, sGeneratorControlPoints, SM_CF_UNSPECIFIED,
        sGeneratorKnotMultiplicities, sGeneratorKnots, SM_KT_UNSPECIFIED, NULL, NULL,
        pGenerator) );
    SmObjDelete sGeneratorCleanup(pGenerator);
    if( pGenerator == NULL ) return SM_ERR;
    const SmExtent1d sGeneratorInterval = pGenerator->GetSTEPInterval();
    SmSurfOfRevolution* pRevolution = NULL;
    SmStatus eRevolutionStat = SmSurfOfRevolution::CreateCanonical(
        rContext, pGenerator, SmPoint3d(0.0, 0.0, 0.0), SmVector3d(0.0, 0.0, 1.0), pRevolution );
    if( pRevolution != NULL )
        sGeneratorCleanup.Clear();
    SmObjDelete sC5(pRevolution);
    SER( eRevolutionStat );
    if( pRevolution == NULL )
        return SM_ERR;

    // Steps: 1e-3 degrees for angular parameters, 1e-4 for the linear axis parameter.
    const double aU[] = { 20.0, 110.0, 250.0 };
    for( double dU : aU )
    {
        SER( sm_CheckSTEPDerivatives(pCylinder, SmPoint2d(dU, 1.0), 1.0e-3, 1.0e-4) );
        SER( sm_CheckSTEPDerivatives(pCone,     SmPoint2d(dU, 1.5), 1.0e-3, 1.0e-4) );
        SER( sm_CheckSTEPDerivatives(pSphere,   SmPoint2d(dU, 30.0), 1.0e-3, 1.0e-3) );
        SER( sm_CheckSTEPDerivatives(pTorus,    SmPoint2d(dU, 40.0), 1.0e-3, 1.0e-3) );
        SER( sm_CheckSTEPDerivatives(
            pRevolution, SmPoint2d(dU, sGeneratorInterval.Evaluate(0.4)), 1.0e-3,
            1.0e-4*sGeneratorInterval.GetLength()) );
    }
    return SM_SUCCESS;
}

//*************************************************************************
// Run all tests in this file
//*************************************************************************

SmStatus TestSmGeneral()
{
    SmStatus stat = TestSmTransform();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmScale();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmRotate();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmTranslate();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmPolyBrepMassPropertiesWinding();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmPrincipalDirectionPairing();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmSTEPDerivativeUnits();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmConeSTEPCrossDerivative();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmPlanarFacesFailureCleanup();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmEdgeCurveOwnership();
    if( stat != SM_SUCCESS )
        return( stat );

    stat = TestSmCurveSweepFailureCleanup();
    if( stat != SM_SUCCESS )
        return( stat );

    return( SM_SUCCESS );
}


//*************************************************************************
// The status-returning assert macros must return an error status, never
// FALSE (status 0), in every build. SM_API_test is built without
// SM_DEBUG_CODE in release, so this checks the release definitions.
//*************************************************************************
static SmStatus sm_AersProbe(SmBoolean bOk)
{
    AERS(bOk);
    return SM_SUCCESS;
}

static SmStatus sm_AersMsgProbe(SmBoolean bOk)
{
    AERS_MSG(bOk, _T("probe"));
    return SM_SUCCESS;
}

static SmStatus sm_AernProbe(SmBoolean bOk, SmStatus eFailure)
{
    AERN(bOk, eFailure);
    return SM_SUCCESS;
}

static SmStatus sm_AernMsgProbe(SmBoolean bOk, SmStatus eFailure)
{
    AERN_MSG(bOk, eFailure, _T("probe"));
    return SM_SUCCESS;
}

static SmBoolean sm_AerProbe(SmBoolean bOk)
{
    AER(bOk);
    return TRUE;
}

static SmBoolean sm_AerMsgProbe(SmBoolean bOk)
{
    AER_MSG(bOk, _T("probe"));
    return TRUE;
}

SmStatus TestSmStatusAssertMacros()
{
    if (   sm_AersProbe(TRUE) != SM_SUCCESS || sm_AersProbe(FALSE) != SM_ERR
        || sm_AersMsgProbe(TRUE) != SM_SUCCESS || sm_AersMsgProbe(FALSE) != SM_ERR)
        return SM_ERR;

    // AERN preserves the requested status, including the assertion status
    // used by the converted kernel call sites.
    const SmStatus aFailures[] = { SM_ERR_ASSERT_FAILURE, SM_ERR_INVALID_INPUT };
    for (SmStatus eFailure : aFailures)
    {
        if (   sm_AernProbe(TRUE, eFailure) != SM_SUCCESS
            || sm_AernProbe(FALSE, eFailure) != eFailure
            || sm_AernMsgProbe(TRUE, eFailure) != SM_SUCCESS
            || sm_AernMsgProbe(FALSE, eFailure) != eFailure)
            return SM_ERR;
    }

    // Boolean-returning callers still rely on AER returning FALSE.
    if (   sm_AerProbe(TRUE) != TRUE || sm_AerProbe(FALSE) != FALSE
        || sm_AerMsgProbe(TRUE) != TRUE || sm_AerMsgProbe(FALSE) != FALSE)
        return SM_ERR;

    // Exercise actual kernel guards as well as the macro definitions.
    // Defeaturing without selected faces must return an assertion failure,
    // not the Boolean FALSE previously returned by this status function.
    SmContext sContext;
    SmBrep sBrep;
    SmBrep* pBase = &sBrep;
    SmBrep* pFeature = NULL;
    SmTArray<SmFace*> sFaces;
    if (   SmFeatureExecutive::DefeatureAndRebuild(
               sContext, pBase, sFaces, pFeature) != SM_ERR_ASSERT_FAILURE
        || pBase != &sBrep || pFeature != NULL)
        return SM_ERR;

    // Empty trim input takes the existing AERS_MSG guard, before any
    // geometry is evaluated or output is appended.
    SmPlane sPlane(SmPoint3d(0, 0, 0), SmVector3d(0, 0, 1), &sContext);
    SmTArray<SmCurve*> sCurves, sOrderedCurves;
    SmTArray<SmOrientType> sOrients;
    if (   SmTrimmingTools::FindLoopCurveOrientations(
               sPlane, sCurves, SM_OT_SAME, sOrderedCurves, sOrients) != SM_ERR
        || sOrderedCurves.GetSize() != 0 || sOrients.GetSize() != 0)
        return SM_ERR;
    return SM_SUCCESS;
}
