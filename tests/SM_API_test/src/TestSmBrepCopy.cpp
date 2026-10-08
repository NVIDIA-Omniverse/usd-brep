// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "StdAfx.h"

#include <SmApiBrep.h>
#include <SmApiGeneral.h>
#include <SmApiPrimitives.h>
#include <SmApiQueries.h>
#include <SmBrep.h>
#include <SmBSplineCurve.h>
#include <SmBSplineSurface.h>
#include <SmEdge.h>
#include <SmFace.h>

#include <memory>

static SmStatus CheckCopyRepresentation(const SmBrep& source, const SmBrep& copy)
{
    SmTArray<SmFace*> sourceFaces, copyFaces;
    SmTArray<SmEdge*> sourceEdges, copyEdges;
    source.GetFaces(sourceFaces);
    copy.GetFaces(copyFaces);
    source.GetEdges(sourceEdges);
    copy.GetEdges(copyEdges);
    if (&source == &copy || sourceFaces.GetSize() != copyFaces.GetSize() ||
        sourceEdges.GetSize() != copyEdges.GetSize())
        return SM_ERR;

    for (ULONG ii = 0; ii < sourceFaces.GetSize(); ++ii)
    {
        SmSurface* original = sourceFaces[ii]->GetSurface();
        SmSurface* duplicate = copyFaces[ii]->GetSurface();
        const SmExtent2d a = sourceFaces[ii]->GetUVDomain();
        const SmExtent2d b = copyFaces[ii]->GetUVDomain();
        if (sourceFaces[ii] == copyFaces[ii] || original == duplicate ||
            original->GetType() != duplicate->GetType() ||
            a.GetUMin() != b.GetUMin() || a.GetUMax() != b.GetUMax() ||
            a.GetVMin() != b.GetVMin() || a.GetVMax() != b.GetVMax())
            return SM_ERR;
        const SmPoint2d uv = a.IsBounded() ? a.GetMin() + a.GetSize() * 0.37 : SmPoint2d(0.0, 0.0);
        SmPoint3d pa, pb;
        SER(original->EvaluatePoint(uv, pa));
        SER(duplicate->EvaluatePoint(uv, pb));
        if (pa.DistanceBetween(pb) > 1.0e-8)
            return SM_ERR;
    }
    for (ULONG ii = 0; ii < sourceEdges.GetSize(); ++ii)
    {
        SmCurve* original = sourceEdges[ii]->GetCurve();
        SmCurve* duplicate = copyEdges[ii]->GetCurve();
        const SmExtent1d a = sourceEdges[ii]->GetInterval();
        const SmExtent1d b = copyEdges[ii]->GetInterval();
        if (sourceEdges[ii] == copyEdges[ii] || original == duplicate ||
            original->GetType() != duplicate->GetType() ||
            a.GetMin() != b.GetMin() || a.GetMax() != b.GetMax())
            return SM_ERR;
        SmPoint3d pa, pb;
        SER(original->EvaluatePoint(a.Evaluate(0.37), pa));
        SER(duplicate->EvaluatePoint(b.Evaluate(0.37), pb));
        if (pa.DistanceBetween(pb) > 1.0e-8)
            return SM_ERR;
    }
    return SM_SUCCESS;
}

static SmStatus CheckApiCopy(SmBrep& source)
{
    SmBrep* rawCopy = NULL;
    const SmStatus status = SmApiBrepCopy(&source, rawCopy);
    std::unique_ptr<SmBrep> copy(rawCopy);
    SER(status);
    NER(rawCopy);
    return CheckCopyRepresentation(source, *copy);
}

SmStatus TestSmBrepCopyRepresentation()
{
    for (int shape = 0; shape < 5; ++shape)
    {
        const SmVector3d origin(0.0, 0.0, 0.0);
        SmBrep* rawSource = NULL;
        SmStatus status = SM_ERR;
        switch (shape)
        {
        case 0: status = SmApiCreateBox(origin, 4.0, 5.0, 6.0, rawSource); break;
        case 1: status = SmApiCreateCylinder(origin, 3.0, 8.0, rawSource); break;
        case 2: status = SmApiCreateCone(origin, 3.0, 1.0, 8.0, rawSource); break;
        case 3: status = SmApiCreateSphere(origin, 3.0, rawSource); break;
        case 4: status = SmApiCreateTorus(origin, 5.0, 1.0, rawSource); break;
        }
        std::unique_ptr<SmBrep> source(rawSource);
        SER(status);
        NER(rawSource);

        // Preserve analytics as analytics, not just NURBS as NURBS.
        SER(CheckApiCopy(*source));
        if (shape == 0)
        {
            // A mixed model must not be normalized to either representation.
            SmTArray<SmFace*> faces;
            source->GetFaces(faces);
            SmSurface* nurb = NULL;
            SER(faces[0]->GetSurface()->CopyAnalyticAsNurb(*source->GetContext(), nurb));
            NER(nurb);
            SER(source->ReplaceSurface(faces[0], nurb, FALSE));
            SER(CheckApiCopy(*source));
        }

        SER(SmApiTurnToNurbs(source.get()));
        SER(CheckApiCopy(*source));

        SmBrep strict(*source, NULL, NULL, TRUE, FALSE);
        SER(CheckCopyRepresentation(*source, strict));
        SmBrep merged;
        SER(merged.MergeBrep(*source, NULL, NULL, FALSE));
        SER(CheckCopyRepresentation(*source, merged));

#ifdef USE_ANALYTICS
        // Existing kernel callers retain analytic recognition by default.
        if (shape == 0)
        {
            SmBrep historical(*source);
            SmBrep historicalMerge;
            SER(historicalMerge.MergeBrep(*source));
            for (SmBrep* candidate : {&historical, &historicalMerge})
            {
                SmTArray<SmFace*> faces;
                candidate->GetFaces(faces);
                for (ULONG ii = 0; ii < faces.GetSize(); ++ii)
                {
                    if (faces[ii]->GetSurface()->GetType() == SmBSplineSurface_TYPE)
                        return SM_ERR;
                }
            }
        }
#endif
    }
    return SM_SUCCESS;
}
