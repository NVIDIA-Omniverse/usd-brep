// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Exact inverse of OcctAsciiParser.cpp (occt::ReadOcct). The token layout, section
// ordering and per-record field order all mirror the reader so that
// ReadOcct(WriteOcct(model)) reproduces the same SOcctModel. Output is always emitted
// in the "CASCADE Topology V3" dialect.

#include "OcctAsciiWriter.h"

#include <cstdio>
#include <string>

namespace occt
{
namespace
{

class CTextSink
{
public:

    void Char(char c)
    {
        m_sOut.push_back(c);
    }

    void Str(const char* p)
    {
        m_sOut.append(p);
    }

    void NewLine()
    {
        m_sOut.push_back('\n');
    }

    // GeomTools writes reals as whitespace-delimited tokens parsed back with strtod.
    // %.17g preserves every double exactly on re-read.
    void Real(double d)
    {
        char szBuf[32];
        std::snprintf(szBuf, sizeof(szBuf), "%.17g", d);
        m_sOut.append(szBuf);
    }

    void Int(int i)
    {
        m_sOut.append(std::to_string(i));
    }

    // Emit a real followed by a single separating space.
    void RealSp(double d)
    {
        Real(d);
        Char(' ');
    }

    void IntSp(int i)
    {
        Int(i);
        Char(' ');
    }

    std::string Take()
    {
        return std::move(m_sOut);
    }

private:

    std::string m_sOut;
};

void WritePnt3(CTextSink& rOut, const SPnt3& s)
{
    rOut.RealSp(s.dX);
    rOut.RealSp(s.dY);
    rOut.RealSp(s.dZ);
}

void WriteDir3(CTextSink& rOut, const SDir3& s)
{
    rOut.RealSp(s.dX);
    rOut.RealSp(s.dY);
    rOut.RealSp(s.dZ);
}

void WritePnt2(CTextSink& rOut, const SPnt2& s)
{
    rOut.RealSp(s.dX);
    rOut.RealSp(s.dY);
}

void WriteDir2(CTextSink& rOut, const SDir2& s)
{
    rOut.RealSp(s.dX);
    rOut.RealSp(s.dY);
}

const char* RegularityName(EContinuity e)
{
    switch (e)
    {
        case EContinuity::C0:
            return "C0";
        case EContinuity::G1:
            return "G1";
        case EContinuity::C1:
            return "C1";
        case EContinuity::G2:
            return "G2";
        case EContinuity::C2:
            return "C2";
        case EContinuity::C3:
            return "C3";
        case EContinuity::CN:
            return "CN";
        default:
            return "C0";
    }
}

// ---- Geometry: 3D curves ----

void WriteCurve3d(CTextSink& rOut, const SCurve3d& rC)
{
    const int iType = static_cast<int>(rC.eType);
    rOut.IntSp(iType);
    switch (iType)
    {
        case 1: // Line
            WritePnt3(rOut, rC.sLocation);
            WriteDir3(rOut, rC.sXAxis);
            break;
        case 2: // Circle
            WritePnt3(rOut, rC.sLocation);
            WriteDir3(rOut, rC.sAxis);
            WriteDir3(rOut, rC.sXAxis);
            WriteDir3(rOut, rC.sYAxis);
            rOut.RealSp(rC.dRadius);
            break;
        case 3: // Ellipse
            WritePnt3(rOut, rC.sLocation);
            WriteDir3(rOut, rC.sAxis);
            WriteDir3(rOut, rC.sXAxis);
            WriteDir3(rOut, rC.sYAxis);
            rOut.RealSp(rC.dRadius);
            rOut.RealSp(rC.dMinorRadius);
            break;
        case 4: // Parabola
            WritePnt3(rOut, rC.sLocation);
            WriteDir3(rOut, rC.sAxis);
            WriteDir3(rOut, rC.sXAxis);
            WriteDir3(rOut, rC.sYAxis);
            rOut.RealSp(rC.dFocal);
            break;
        case 5: // Hyperbola
            WritePnt3(rOut, rC.sLocation);
            WriteDir3(rOut, rC.sAxis);
            WriteDir3(rOut, rC.sXAxis);
            WriteDir3(rOut, rC.sYAxis);
            rOut.RealSp(rC.dRadius);
            rOut.RealSp(rC.dMinorRadius);
            break;
        case 6: // Bezier
        {
            rOut.IntSp(rC.bRational ? 1 : 0);
            rOut.IntSp(rC.iDegree);
            for (size_t ii = 0; ii < rC.sPoles.size(); ++ii)
            {
                WritePnt3(rOut, rC.sPoles[ii]);
                if (rC.bRational && ii < rC.dWeights.size())
                {
                    rOut.RealSp(rC.dWeights[ii]);
                }
            }
            break;
        }
        case 7: // BSpline
        {
            rOut.IntSp(rC.bRational ? 1 : 0);
            rOut.IntSp(rC.bPeriodic ? 1 : 0);
            rOut.IntSp(rC.iDegree);
            rOut.IntSp(static_cast<int>(rC.sPoles.size()));
            rOut.IntSp(static_cast<int>(rC.dKnots.size()));
            for (size_t ii = 0; ii < rC.sPoles.size(); ++ii)
            {
                WritePnt3(rOut, rC.sPoles[ii]);
                if (rC.bRational && ii < rC.dWeights.size())
                {
                    rOut.RealSp(rC.dWeights[ii]);
                }
            }
            for (size_t ii = 0; ii < rC.dKnots.size(); ++ii)
            {
                rOut.RealSp(rC.dKnots[ii]);
                rOut.IntSp(ii < rC.iMults.size() ? rC.iMults[ii] : 1);
            }
            break;
        }
        case 8: // Trimmed
            rOut.RealSp(rC.dFirst);
            rOut.RealSp(rC.dLast);
            if (rC.pBasis)
            {
                WriteCurve3d(rOut, *rC.pBasis);
            }
            break;
        case 9: // Offset
            rOut.RealSp(rC.dOffset);
            WriteDir3(rOut, rC.sOffsetDir);
            if (rC.pBasis)
            {
                WriteCurve3d(rOut, *rC.pBasis);
            }
            break;
        default:
            break;
    }
    rOut.NewLine();
}

// ---- Geometry: 2D curves ----

void WriteCurve2d(CTextSink& rOut, const SCurve2d& rC)
{
    const int iType = static_cast<int>(rC.eType);
    rOut.IntSp(iType);
    switch (iType)
    {
        case 1: // Line
            WritePnt2(rOut, rC.sLocation);
            WriteDir2(rOut, rC.sXAxis);
            break;
        case 2: // Circle
            WritePnt2(rOut, rC.sLocation);
            WriteDir2(rOut, rC.sXAxis);
            WriteDir2(rOut, rC.sYAxis);
            rOut.RealSp(rC.dRadius);
            break;
        case 3: // Ellipse
            WritePnt2(rOut, rC.sLocation);
            WriteDir2(rOut, rC.sXAxis);
            WriteDir2(rOut, rC.sYAxis);
            rOut.RealSp(rC.dRadius);
            rOut.RealSp(rC.dMinorRadius);
            break;
        case 4: // Parabola
            WritePnt2(rOut, rC.sLocation);
            WriteDir2(rOut, rC.sXAxis);
            WriteDir2(rOut, rC.sYAxis);
            rOut.RealSp(rC.dFocal);
            break;
        case 5: // Hyperbola
            WritePnt2(rOut, rC.sLocation);
            WriteDir2(rOut, rC.sXAxis);
            WriteDir2(rOut, rC.sYAxis);
            rOut.RealSp(rC.dRadius);
            rOut.RealSp(rC.dMinorRadius);
            break;
        case 6: // Bezier
        {
            rOut.IntSp(rC.bRational ? 1 : 0);
            rOut.IntSp(rC.iDegree);
            for (size_t ii = 0; ii < rC.sPoles.size(); ++ii)
            {
                WritePnt2(rOut, rC.sPoles[ii]);
                if (rC.bRational && ii < rC.dWeights.size())
                {
                    rOut.RealSp(rC.dWeights[ii]);
                }
            }
            break;
        }
        case 7: // BSpline
        {
            rOut.IntSp(rC.bRational ? 1 : 0);
            rOut.IntSp(rC.bPeriodic ? 1 : 0);
            rOut.IntSp(rC.iDegree);
            rOut.IntSp(static_cast<int>(rC.sPoles.size()));
            rOut.IntSp(static_cast<int>(rC.dKnots.size()));
            for (size_t ii = 0; ii < rC.sPoles.size(); ++ii)
            {
                WritePnt2(rOut, rC.sPoles[ii]);
                if (rC.bRational && ii < rC.dWeights.size())
                {
                    rOut.RealSp(rC.dWeights[ii]);
                }
            }
            for (size_t ii = 0; ii < rC.dKnots.size(); ++ii)
            {
                rOut.RealSp(rC.dKnots[ii]);
                rOut.IntSp(ii < rC.iMults.size() ? rC.iMults[ii] : 1);
            }
            break;
        }
        case 8: // Trimmed
            rOut.RealSp(rC.dFirst);
            rOut.RealSp(rC.dLast);
            if (rC.pBasis)
            {
                WriteCurve2d(rOut, *rC.pBasis);
            }
            break;
        case 9: // Offset (2D: no direction)
            rOut.RealSp(rC.dOffset);
            if (rC.pBasis)
            {
                WriteCurve2d(rOut, *rC.pBasis);
            }
            break;
        default:
            break;
    }
    rOut.NewLine();
}

// ---- Geometry: surfaces ----

void WriteAx3(CTextSink& rOut, const SSurface& rS)
{
    WritePnt3(rOut, rS.sLocation);
    WriteDir3(rOut, rS.sAxis);
    WriteDir3(rOut, rS.sXAxis);
    WriteDir3(rOut, rS.sYAxis);
}

void WriteSurface(CTextSink& rOut, const SSurface& rS)
{
    const int iType = static_cast<int>(rS.eType);
    rOut.IntSp(iType);
    switch (iType)
    {
        case 1: // Plane
            WriteAx3(rOut, rS);
            break;
        case 2: // Cylinder
            WriteAx3(rOut, rS);
            rOut.RealSp(rS.dRadius);
            break;
        case 3: // Cone
            WriteAx3(rOut, rS);
            rOut.RealSp(rS.dRadius);
            rOut.RealSp(rS.dSemiAngle);
            break;
        case 4: // Sphere
            WriteAx3(rOut, rS);
            rOut.RealSp(rS.dRadius);
            break;
        case 5: // Torus
            WriteAx3(rOut, rS);
            rOut.RealSp(rS.dRadius);
            rOut.RealSp(rS.dMinorRadius);
            break;
        case 6: // SurfaceOfLinearExtrusion
            WriteDir3(rOut, rS.sDirection);
            if (rS.pBasisCurve)
            {
                WriteCurve3d(rOut, *rS.pBasisCurve);
            }
            return; // basis curve writer already emitted the trailing newline
        case 7: // SurfaceOfRevolution
            WritePnt3(rOut, rS.sLocation);
            WriteDir3(rOut, rS.sDirection);
            if (rS.pBasisCurve)
            {
                WriteCurve3d(rOut, *rS.pBasisCurve);
            }
            return;
        case 8: // Bezier
        {
            rOut.IntSp(rS.bURational ? 1 : 0);
            rOut.IntSp(rS.bVRational ? 1 : 0);
            rOut.IntSp(rS.iUDegree);
            rOut.IntSp(rS.iVDegree);
            const bool bWeighted = rS.bURational || rS.bVRational;
            for (size_t ii = 0; ii < rS.sPoles.size(); ++ii)
            {
                WritePnt3(rOut, rS.sPoles[ii]);
                if (bWeighted && ii < rS.dWeights.size())
                {
                    rOut.RealSp(rS.dWeights[ii]);
                }
            }
            break;
        }
        case 9: // BSpline
        {
            rOut.IntSp(rS.bURational ? 1 : 0);
            rOut.IntSp(rS.bVRational ? 1 : 0);
            rOut.IntSp(rS.bUPeriodic ? 1 : 0);
            rOut.IntSp(rS.bVPeriodic ? 1 : 0);
            rOut.IntSp(rS.iUDegree);
            rOut.IntSp(rS.iVDegree);
            rOut.IntSp(rS.iNbUPoles);
            rOut.IntSp(rS.iNbVPoles);
            rOut.IntSp(static_cast<int>(rS.dUKnots.size()));
            rOut.IntSp(static_cast<int>(rS.dVKnots.size()));
            const bool bWeighted = rS.bURational || rS.bVRational;
            for (size_t ii = 0; ii < rS.sPoles.size(); ++ii)
            {
                WritePnt3(rOut, rS.sPoles[ii]);
                if (bWeighted && ii < rS.dWeights.size())
                {
                    rOut.RealSp(rS.dWeights[ii]);
                }
            }
            for (size_t ii = 0; ii < rS.dUKnots.size(); ++ii)
            {
                rOut.RealSp(rS.dUKnots[ii]);
                rOut.IntSp(ii < rS.iUMults.size() ? rS.iUMults[ii] : 1);
            }
            for (size_t ii = 0; ii < rS.dVKnots.size(); ++ii)
            {
                rOut.RealSp(rS.dVKnots[ii]);
                rOut.IntSp(ii < rS.iVMults.size() ? rS.iVMults[ii] : 1);
            }
            break;
        }
        case 10: // RectangularTrimmed
            rOut.RealSp(rS.dU1);
            rOut.RealSp(rS.dU2);
            rOut.RealSp(rS.dV1);
            rOut.RealSp(rS.dV2);
            if (rS.pBasisSurface)
            {
                WriteSurface(rOut, *rS.pBasisSurface);
            }
            return;
        case 11: // Offset
            rOut.RealSp(rS.dOffset);
            if (rS.pBasisSurface)
            {
                WriteSurface(rOut, *rS.pBasisSurface);
            }
            return;
        default:
            break;
    }
    rOut.NewLine();
}

// ---- Sections ----

void WriteLocations(CTextSink& rOut, const SOcctModel& rModel)
{
    rOut.Str("Locations ");
    rOut.Int(static_cast<int>(rModel.sLocations.size()));
    rOut.NewLine();
    for (const SLocation& rLoc : rModel.sLocations)
    {
        if (rLoc.bElementary)
        {
            rOut.IntSp(1);
            for (int iRow = 0; iRow < 3; ++iRow)
            {
                for (int iCol = 0; iCol < 4; ++iCol)
                {
                    rOut.RealSp(rLoc.dMatrix[iRow][iCol]);
                }
            }
            rOut.NewLine();
        }
        else
        {
            rOut.IntSp(2);
            for (const SLocationTerm& rTerm : rLoc.sTerms)
            {
                rOut.IntSp(rTerm.iLocationIndex);
                rOut.IntSp(rTerm.iPower);
            }
            rOut.Int(0); // terminator
            rOut.NewLine();
        }
    }
}

void WriteCurve2ds(CTextSink& rOut, const SOcctModel& rModel)
{
    rOut.Str("Curve2ds ");
    rOut.Int(static_cast<int>(rModel.sCurve2ds.size()));
    rOut.NewLine();
    for (const SCurve2d& rC : rModel.sCurve2ds)
    {
        WriteCurve2d(rOut, rC);
    }
}

void WriteCurves(CTextSink& rOut, const SOcctModel& rModel)
{
    rOut.Str("Curves ");
    rOut.Int(static_cast<int>(rModel.sCurves.size()));
    rOut.NewLine();
    for (const SCurve3d& rC : rModel.sCurves)
    {
        WriteCurve3d(rOut, rC);
    }
}

void WriteSurfaces(CTextSink& rOut, const SOcctModel& rModel)
{
    rOut.Str("Surfaces ");
    rOut.Int(static_cast<int>(rModel.sSurfaces.size()));
    rOut.NewLine();
    for (const SSurface& rS : rModel.sSurfaces)
    {
        WriteSurface(rOut, rS);
    }
}

void WritePolygon3D(CTextSink& rOut, const SOcctModel& rModel)
{
    rOut.Str("Polygon3D ");
    rOut.Int(static_cast<int>(rModel.sPolygons3d.size()));
    rOut.NewLine();
    for (const SPolygon3d& rP : rModel.sPolygons3d)
    {
        const int iNbNodes = static_cast<int>(rP.sNodes.size());
        const bool bHasParams = !rP.dParams.empty();
        rOut.IntSp(iNbNodes);
        rOut.IntSp(bHasParams ? 1 : 0);
        rOut.Real(rP.dDeflection);
        rOut.NewLine();
        for (const SPnt3& rN : rP.sNodes)
        {
            WritePnt3(rOut, rN);
        }
        rOut.NewLine();
        if (bHasParams)
        {
            for (double d : rP.dParams)
            {
                rOut.RealSp(d);
            }
            rOut.NewLine();
        }
    }
}

void WritePolygonOnTriangulations(CTextSink& rOut, const SOcctModel& rModel)
{
    rOut.Str("PolygonOnTriangulations ");
    rOut.Int(static_cast<int>(rModel.sPolygonsOnTri.size()));
    rOut.NewLine();
    for (const SPolygonOnTriangulation& rP : rModel.sPolygonsOnTri)
    {
        rOut.IntSp(static_cast<int>(rP.iNodes.size()));
        for (int iNode : rP.iNodes)
        {
            rOut.IntSp(iNode);
        }
        rOut.NewLine();
        rOut.Str("p ");
        rOut.RealSp(rP.dDeflection);
        const bool bHasParams = !rP.dParams.empty();
        rOut.IntSp(bHasParams ? 1 : 0);
        if (bHasParams)
        {
            for (double d : rP.dParams)
            {
                rOut.RealSp(d);
            }
        }
        rOut.NewLine();
    }
}

void WriteTriangulations(CTextSink& rOut, const SOcctModel& rModel)
{
    rOut.Str("Triangulations ");
    rOut.Int(static_cast<int>(rModel.sTriangulations.size()));
    rOut.NewLine();
    for (const STriangulation& rT : rModel.sTriangulations)
    {
        rOut.IntSp(rT.iNbNodes);
        rOut.IntSp(rT.iNbTriangles);
        rOut.IntSp(rT.bHasUV ? 1 : 0);
        rOut.IntSp(rT.bHasNormals ? 1 : 0); // V3 always carries the normals flag
        rOut.Real(rT.dDeflection);
        rOut.NewLine();
        for (const SPnt3& rN : rT.sNodes)
        {
            WritePnt3(rOut, rN);
        }
        if (rT.bHasUV)
        {
            for (const SPnt2& rUV : rT.sUVNodes)
            {
                WritePnt2(rOut, rUV);
            }
        }
        for (const std::array<int, 3>& rTri : rT.iTriangles)
        {
            rOut.IntSp(rTri[0]);
            rOut.IntSp(rTri[1]);
            rOut.IntSp(rTri[2]);
        }
        if (rT.bHasNormals)
        {
            for (const SDir3& rNrm : rT.sNormals)
            {
                WriteDir3(rOut, rNrm);
            }
        }
        rOut.NewLine();
    }
}

// ---- Topology ----

const char* ShapeCode(EShapeType eType)
{
    switch (eType)
    {
        case EShapeType::Vertex:
            return "Ve";
        case EShapeType::Edge:
            return "Ed";
        case EShapeType::Wire:
            return "Wi";
        case EShapeType::Face:
            return "Fa";
        case EShapeType::Shell:
            return "Sh";
        case EShapeType::Solid:
            return "So";
        case EShapeType::CompSolid:
            return "CS";
        case EShapeType::Compound:
            return "Co";
        default:
            return "Ve";
    }
}

void WriteVertexGeometry(CTextSink& rOut, const SVertexData& rV)
{
    rOut.Real(rV.dTolerance);
    rOut.NewLine();
    WritePnt3(rOut, rV.sPoint);
    rOut.NewLine();
    for (const SPointRepr& rR : rV.sReprs)
    {
        rOut.RealSp(rR.dParam);
        rOut.IntSp(rR.iKind);
        switch (rR.iKind)
        {
            case 1:
                rOut.IntSp(rR.iCurve);
                break;
            case 2:
                rOut.IntSp(rR.iPCurve);
                rOut.IntSp(rR.iSurface);
                break;
            case 3:
                rOut.RealSp(rR.dParam2);
                rOut.IntSp(rR.iSurface);
                break;
            default:
                break;
        }
        rOut.IntSp(rR.iLocation);
    }
    // Terminator: a dummy param followed by kind 0.
    rOut.IntSp(0);
    rOut.Int(0);
    rOut.NewLine();
}

void WriteEdgeGeometry(CTextSink& rOut, const SEdgeData& rE)
{
    rOut.RealSp(rE.dTolerance);
    rOut.IntSp(rE.bSameParameter ? 1 : 0);
    rOut.IntSp(rE.bSameRange ? 1 : 0);
    rOut.Int(rE.bDegenerated ? 1 : 0);
    rOut.NewLine();
    for (const SCurveRepr& rR : rE.sReprs)
    {
        rOut.IntSp(rR.iKind);
        switch (rR.iKind)
        {
            case 1: // Curve 3D
                rOut.IntSp(rR.iCurve3d);
                rOut.IntSp(rR.iLocation);
                rOut.RealSp(rR.dFirst);
                rOut.Real(rR.dLast);
                break;
            case 2: // Curve on surface
            case 3: // Curve on closed surface
                rOut.IntSp(rR.iPCurve);
                if (rR.iKind == 3)
                {
                    rOut.IntSp(rR.iPCurve2);
                    rOut.Str(RegularityName(rR.eContinuity));
                    rOut.Char(' ');
                }
                rOut.IntSp(rR.iSurface);
                rOut.IntSp(rR.iLocation);
                rOut.RealSp(rR.dFirst);
                rOut.Real(rR.dLast);
                break;
            case 4: // Regularity
                rOut.Str(RegularityName(rR.eContinuity));
                rOut.Char(' ');
                rOut.IntSp(rR.iSurface);
                rOut.IntSp(rR.iLocation);
                rOut.IntSp(rR.iSurface2);
                rOut.Int(rR.iLocation2);
                break;
            case 5: // Polygon3D
                rOut.IntSp(rR.iPolygon3d);
                rOut.Int(rR.iLocation);
                break;
            case 6: // Polygon on triangulation
            case 7: // Polygon on closed triangulation
                rOut.IntSp(rR.iPolyOnTri);
                if (rR.iKind == 7)
                {
                    rOut.IntSp(rR.iPolyOnTri2);
                }
                rOut.IntSp(rR.iTriangulation);
                rOut.Int(rR.iLocation);
                break;
            default:
                break;
        }
        rOut.NewLine();
    }
    rOut.Int(0); // repr terminator
    rOut.NewLine();
}

void WriteFaceGeometry(CTextSink& rOut, const SFaceData& rF)
{
    // Triangulation-only faces (read path iVal == 2) carry no surface and break early.
    if (rF.iSurface == 0 && rF.iTriangulation != 0)
    {
        rOut.IntSp(2);
        rOut.Int(rF.iTriangulation);
        rOut.NewLine();
        return;
    }

    rOut.IntSp(rF.bNaturalRestriction ? 1 : 0);
    rOut.RealSp(rF.dTolerance);
    rOut.IntSp(rF.iSurface);
    rOut.Int(rF.iLocation);
    rOut.NewLine();
    // The reader consumes one extra line here: either "2 <tri>" or the blank separator.
    if (rF.iTriangulation != 0)
    {
        rOut.IntSp(2);
        rOut.Int(rF.iTriangulation);
        rOut.NewLine();
    }
    else
    {
        rOut.NewLine(); // blank separator line (required for faces)
    }
}

void WriteFlags(CTextSink& rOut, const SShapeRecord& rRec)
{
    char szFlags[8];
    szFlags[0] = rRec.bFree ? '1' : '0';
    szFlags[1] = rRec.bModified ? '1' : '0';
    szFlags[2] = rRec.bChecked ? '1' : '0';
    szFlags[3] = rRec.bOrientable ? '1' : '0';
    szFlags[4] = rRec.bClosed ? '1' : '0';
    szFlags[5] = rRec.bInfinite ? '1' : '0';
    szFlags[6] = rRec.bConvex ? '1' : '0';
    szFlags[7] = '\0';
    rOut.Str(szFlags);
    rOut.NewLine();
}

char OrientationChar(EOrientation e)
{
    switch (e)
    {
        case EOrientation::Forward:
            return '+';
        case EOrientation::Reversed:
            return '-';
        case EOrientation::Internal:
            return 'i';
        case EOrientation::External:
            return 'e';
        default:
            return '+';
    }
}

void WriteSubShapeRef(CTextSink& rOut, const SSubShapeRef& rRef, int iNbShapes)
{
    rOut.Char(OrientationChar(rRef.eOrientation));
    rOut.Int(iNbShapes - rRef.iStorageIndex + 1);
    rOut.Char(' ');
    rOut.IntSp(rRef.iLocation);
}

void WriteShapes(CTextSink& rOut, const SOcctModel& rModel)
{
    const int iNbShapes = static_cast<int>(rModel.sShapes.size());
    rOut.Str("TShapes ");
    rOut.Int(iNbShapes);
    rOut.NewLine();
    for (const SShapeRecord& rRec : rModel.sShapes)
    {
        rOut.Str(ShapeCode(rRec.eType));
        rOut.NewLine();
        switch (rRec.eType)
        {
            case EShapeType::Vertex:
                WriteVertexGeometry(rOut, rRec.sVertex);
                break;
            case EShapeType::Edge:
                WriteEdgeGeometry(rOut, rRec.sEdge);
                break;
            case EShapeType::Face:
                WriteFaceGeometry(rOut, rRec.sFace);
                break;
            default:
                // Wire/Shell/Solid/CompSolid/Compound carry no geometry; the reader skips
                // whitespace before the flags, so emit the same blank separator line the
                // OCCT writer does.
                rOut.NewLine();
                break;
        }
        WriteFlags(rOut, rRec);
        for (const SSubShapeRef& rRef : rRec.sSubShapes)
        {
            WriteSubShapeRef(rOut, rRef, iNbShapes);
        }
        rOut.Str("*");
        rOut.NewLine();
    }
}

} // namespace

std::string WriteOcct(const SOcctModel& rModel)
{
    CTextSink sOut;

    // Banner (always emitted in the V3 dialect). The reader skips the leading
    // "DBRep_DrawableShape" line and the blank line before searching for the banner.
    sOut.Str("DBRep_DrawableShape");
    sOut.NewLine();
    sOut.NewLine();
    sOut.Str("CASCADE Topology V3, (c) Open Cascade");
    sOut.NewLine();

    // Section order is fixed by BRepTools_ShapeSet::ReadGeometry / WriteGeometry.
    WriteLocations(sOut, rModel);
    WriteCurve2ds(sOut, rModel);
    WriteCurves(sOut, rModel);
    WritePolygon3D(sOut, rModel);
    WritePolygonOnTriangulations(sOut, rModel);
    WriteSurfaces(sOut, rModel);
    WriteTriangulations(sOut, rModel);

    sOut.NewLine(); // blank line before TShapes (matches OCCT layout)
    WriteShapes(sOut, rModel);

    // Trailing root shape reference.
    sOut.NewLine();
    if (rModel.bHasRoot)
    {
        WriteSubShapeRef(sOut, rModel.sRoot, static_cast<int>(rModel.sShapes.size()));
        sOut.NewLine();
    }

    return sOut.Take();
}

} // namespace occt
