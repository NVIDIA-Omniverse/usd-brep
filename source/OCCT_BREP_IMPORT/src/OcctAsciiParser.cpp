// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "OcctAsciiParser.h"

#include <cctype>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <exception>

namespace occt
{
namespace
{

// The three ASCII topology format banners OCCT recognises (index == format version).
const char* const THE_VERSION_BANNERS[] = {
    "",
    "CASCADE Topology V1, (c) Matra-Datavision",
    "CASCADE Topology V2, (c) Matra-Datavision",
    "CASCADE Topology V3, (c) Open Cascade",
};

// Thrown internally on any malformed/unexpected token; caught at the top level.
struct SParseError
{
    std::string sMessage;
};

[[noreturn]] void Fail(const std::string& rMessage)
{
    throw SParseError{ rMessage };
}

// Whitespace-delimited token stream over the whole file buffer. Mirrors the subset of
// std::istream behaviour the OCCT readers rely on: operator>> (skip leading whitespace, read a
// token, leave the position at the delimiter) and getline (read through the next '\n').
class CTokenStream
{
public:

    explicit CTokenStream(const std::string& rText) : m_rText(rText)
    {
    }

    bool Eof() const
    {
        return m_uiPos >= m_rText.size();
    }

    // operator>> for a word. Returns false at end of stream.
    bool ReadWord(std::string& rWord)
    {
        SkipWhitespace();
        if (Eof())
        {
            rWord.clear();
            return false;
        }
        const size_t uiBegin = m_uiPos;
        while (m_uiPos < m_rText.size() && !IsSpace(m_rText[m_uiPos]))
        {
            ++m_uiPos;
        }
        rWord.assign(m_rText, uiBegin, m_uiPos - uiBegin);
        return true;
    }

    // operator>>(int): skip leading whitespace, parse the longest integer prefix, and leave the
    // position at the first non-digit. Crucially this does NOT require a whitespace delimiter:
    // OCCT writes e.g. closed-surface edges as "<pcurve2>CN" with no space, relying on int
    // extraction to stop at 'C' (BRepTools_ShapeSet.cxx).
    int ReadInt()
    {
        const char* pBegin = m_rText.c_str() + m_uiPos;
        char* pEnd = nullptr;
        errno = 0;
        const long lValue = std::strtol(pBegin, &pEnd, 10);
        if (pEnd == pBegin)
        {
            Fail("expected integer at offset " + std::to_string(m_uiPos));
        }
        // `long` is 64-bit on LP64 platforms, so a token larger than INT range would silently
        // truncate (e.g. a huge count becoming a negative allocation size). Reject it instead -- this
        // parses untrusted external files.
        if (errno == ERANGE || lValue < static_cast<long>(INT_MIN) || lValue > static_cast<long>(INT_MAX))
        {
            Fail("integer out of range at offset " + std::to_string(m_uiPos));
        }
        m_uiPos += static_cast<size_t>(pEnd - pBegin);
        return static_cast<int>(lValue);
    }

    // A non-negative element/section count, bounded so a malformed file cannot drive a huge
    // reserve() (-> std::bad_alloc / OOM) or an absurd loop bound. The cap is well above any real
    // OCCT part's section sizes while staying far inside int range.
    int ReadCount(const char* pWhat)
    {
        constexpr int kMaxCount = 100'000'000;
        const int iValue = ReadInt();
        if (iValue < 0 || iValue > kMaxCount)
        {
            Fail(std::string("invalid count for '") + pWhat + "': " + std::to_string(iValue));
        }
        return iValue;
    }

    // OCCT's GeomTools::GetReal: read one whitespace-delimited token, parse with strtod.
    double ReadReal()
    {
        std::string sTok;
        if (!ReadWord(sTok))
        {
            return 0.0; // GetReal returns 0 at eof
        }
        return std::strtod(sTok.c_str(), nullptr);
    }

    // OCCT reads bool via operator>> as 0/1.
    bool ReadBool()
    {
        return ReadInt() != 0;
    }

    // getline to '\n' (consumes the newline). Trailing '\r' is preserved in the returned string.
    bool GetLine(std::string& rLine)
    {
        if (Eof())
        {
            rLine.clear();
            return false;
        }
        const size_t uiBegin = m_uiPos;
        while (m_uiPos < m_rText.size() && m_rText[m_uiPos] != '\n')
        {
            ++m_uiPos;
        }
        rLine.assign(m_rText, uiBegin, m_uiPos - uiBegin);
        if (m_uiPos < m_rText.size())
        {
            ++m_uiPos; // consume '\n'
        }
        return true;
    }

private:

    static bool IsSpace(char c)
    {
        return std::isspace(static_cast<unsigned char>(c)) != 0;
    }

    void SkipWhitespace()
    {
        while (m_uiPos < m_rText.size() && IsSpace(m_rText[m_uiPos]))
        {
            ++m_uiPos;
        }
    }

    const std::string& m_rText;
    size_t m_uiPos = 0;
};

SPnt3 ReadPnt3(CTokenStream& rIn)
{
    SPnt3 s;
    s.dX = rIn.ReadReal();
    s.dY = rIn.ReadReal();
    s.dZ = rIn.ReadReal();
    return s;
}

SDir3 ReadDir3(CTokenStream& rIn)
{
    SDir3 s;
    s.dX = rIn.ReadReal();
    s.dY = rIn.ReadReal();
    s.dZ = rIn.ReadReal();
    return s;
}

SPnt2 ReadPnt2(CTokenStream& rIn)
{
    SPnt2 s;
    s.dX = rIn.ReadReal();
    s.dY = rIn.ReadReal();
    return s;
}

SDir2 ReadDir2(CTokenStream& rIn)
{
    SDir2 s;
    s.dX = rIn.ReadReal();
    s.dY = rIn.ReadReal();
    return s;
}

// ---- Geometry: 3D curves (GeomTools_CurveSet::ReadCurve) ----

std::shared_ptr<SCurve3d> ReadCurve3d(CTokenStream& rIn)
{
    auto pC = std::make_shared<SCurve3d>();
    const int iType = rIn.ReadInt();
    pC->eType = static_cast<ECurveType>(iType);
    switch (iType)
    {
        case 1: // Line
            pC->sLocation = ReadPnt3(rIn);
            pC->sXAxis = ReadDir3(rIn);
            break;
        case 2: // Circle
            pC->sLocation = ReadPnt3(rIn);
            pC->sAxis = ReadDir3(rIn);
            pC->sXAxis = ReadDir3(rIn);
            pC->sYAxis = ReadDir3(rIn);
            pC->dRadius = rIn.ReadReal();
            break;
        case 3: // Ellipse
            pC->sLocation = ReadPnt3(rIn);
            pC->sAxis = ReadDir3(rIn);
            pC->sXAxis = ReadDir3(rIn);
            pC->sYAxis = ReadDir3(rIn);
            pC->dRadius = rIn.ReadReal();
            pC->dMinorRadius = rIn.ReadReal();
            break;
        case 4: // Parabola
            pC->sLocation = ReadPnt3(rIn);
            pC->sAxis = ReadDir3(rIn);
            pC->sXAxis = ReadDir3(rIn);
            pC->sYAxis = ReadDir3(rIn);
            pC->dFocal = rIn.ReadReal();
            break;
        case 5: // Hyperbola
            pC->sLocation = ReadPnt3(rIn);
            pC->sAxis = ReadDir3(rIn);
            pC->sXAxis = ReadDir3(rIn);
            pC->sYAxis = ReadDir3(rIn);
            pC->dRadius = rIn.ReadReal();
            pC->dMinorRadius = rIn.ReadReal();
            break;
        case 6: // Bezier
        {
            pC->bRational = rIn.ReadBool();
            pC->iDegree = rIn.ReadInt();
            const int iNbPoles = pC->iDegree + 1;
            for (int ii = 0; ii < iNbPoles; ++ii)
            {
                pC->sPoles.push_back(ReadPnt3(rIn));
                if (pC->bRational)
                {
                    pC->dWeights.push_back(rIn.ReadReal());
                }
            }
            break;
        }
        case 7: // BSpline
        {
            pC->bRational = rIn.ReadBool();
            pC->bPeriodic = rIn.ReadBool();
            pC->iDegree = rIn.ReadInt();
            const int iNbPoles = rIn.ReadCount("BSpline poles");
            const int iNbKnots = rIn.ReadCount("BSpline knots");
            for (int ii = 0; ii < iNbPoles; ++ii)
            {
                pC->sPoles.push_back(ReadPnt3(rIn));
                if (pC->bRational)
                {
                    pC->dWeights.push_back(rIn.ReadReal());
                }
            }
            for (int ii = 0; ii < iNbKnots; ++ii)
            {
                pC->dKnots.push_back(rIn.ReadReal());
                pC->iMults.push_back(rIn.ReadInt());
            }
            break;
        }
        case 8: // Trimmed
            pC->dFirst = rIn.ReadReal();
            pC->dLast = rIn.ReadReal();
            pC->pBasis = ReadCurve3d(rIn);
            break;
        case 9: // Offset
            pC->dOffset = rIn.ReadReal();
            pC->sOffsetDir = ReadDir3(rIn);
            pC->pBasis = ReadCurve3d(rIn);
            break;
        default:
            Fail("unsupported 3D curve type " + std::to_string(iType));
    }
    return pC;
}

// ---- Geometry: 2D curves (GeomTools_Curve2dSet::ReadCurve2d) ----

std::shared_ptr<SCurve2d> ReadCurve2d(CTokenStream& rIn)
{
    auto pC = std::make_shared<SCurve2d>();
    const int iType = rIn.ReadInt();
    pC->eType = static_cast<ECurveType>(iType);
    switch (iType)
    {
        case 1: // Line
            pC->sLocation = ReadPnt2(rIn);
            pC->sXAxis = ReadDir2(rIn);
            break;
        case 2: // Circle
            pC->sLocation = ReadPnt2(rIn);
            pC->sXAxis = ReadDir2(rIn);
            pC->sYAxis = ReadDir2(rIn);
            pC->dRadius = rIn.ReadReal();
            break;
        case 3: // Ellipse
            pC->sLocation = ReadPnt2(rIn);
            pC->sXAxis = ReadDir2(rIn);
            pC->sYAxis = ReadDir2(rIn);
            pC->dRadius = rIn.ReadReal();
            pC->dMinorRadius = rIn.ReadReal();
            break;
        case 4: // Parabola
            pC->sLocation = ReadPnt2(rIn);
            pC->sXAxis = ReadDir2(rIn);
            pC->sYAxis = ReadDir2(rIn);
            pC->dFocal = rIn.ReadReal();
            break;
        case 5: // Hyperbola
            pC->sLocation = ReadPnt2(rIn);
            pC->sXAxis = ReadDir2(rIn);
            pC->sYAxis = ReadDir2(rIn);
            pC->dRadius = rIn.ReadReal();
            pC->dMinorRadius = rIn.ReadReal();
            break;
        case 6: // Bezier
        {
            pC->bRational = rIn.ReadBool();
            pC->iDegree = rIn.ReadInt();
            const int iNbPoles = pC->iDegree + 1;
            for (int ii = 0; ii < iNbPoles; ++ii)
            {
                pC->sPoles.push_back(ReadPnt2(rIn));
                if (pC->bRational)
                {
                    pC->dWeights.push_back(rIn.ReadReal());
                }
            }
            break;
        }
        case 7: // BSpline
        {
            pC->bRational = rIn.ReadBool();
            pC->bPeriodic = rIn.ReadBool();
            pC->iDegree = rIn.ReadInt();
            const int iNbPoles = rIn.ReadCount("BSpline poles");
            const int iNbKnots = rIn.ReadCount("BSpline knots");
            for (int ii = 0; ii < iNbPoles; ++ii)
            {
                pC->sPoles.push_back(ReadPnt2(rIn));
                if (pC->bRational)
                {
                    pC->dWeights.push_back(rIn.ReadReal());
                }
            }
            for (int ii = 0; ii < iNbKnots; ++ii)
            {
                pC->dKnots.push_back(rIn.ReadReal());
                pC->iMults.push_back(rIn.ReadInt());
            }
            break;
        }
        case 8: // Trimmed
            pC->dFirst = rIn.ReadReal();
            pC->dLast = rIn.ReadReal();
            pC->pBasis = ReadCurve2d(rIn);
            break;
        case 9: // Offset (2D: no direction)
            pC->dOffset = rIn.ReadReal();
            pC->pBasis = ReadCurve2d(rIn);
            break;
        default:
            Fail("unsupported 2D curve type " + std::to_string(iType));
    }
    return pC;
}

// ---- Geometry: surfaces (GeomTools_SurfaceSet::ReadSurface) ----

void ReadAx3(CTokenStream& rIn, SSurface& rS)
{
    rS.sLocation = ReadPnt3(rIn);
    rS.sAxis = ReadDir3(rIn);
    rS.sXAxis = ReadDir3(rIn);
    rS.sYAxis = ReadDir3(rIn);
}

std::shared_ptr<SSurface> ReadSurface(CTokenStream& rIn)
{
    auto pS = std::make_shared<SSurface>();
    const int iType = rIn.ReadInt();
    pS->eType = static_cast<ESurfaceType>(iType);
    switch (iType)
    {
        case 1: // Plane
            ReadAx3(rIn, *pS);
            break;
        case 2: // Cylinder
            ReadAx3(rIn, *pS);
            pS->dRadius = rIn.ReadReal();
            break;
        case 3: // Cone
            ReadAx3(rIn, *pS);
            pS->dRadius = rIn.ReadReal();
            pS->dSemiAngle = rIn.ReadReal();
            break;
        case 4: // Sphere
            ReadAx3(rIn, *pS);
            pS->dRadius = rIn.ReadReal();
            break;
        case 5: // Torus
            ReadAx3(rIn, *pS);
            pS->dRadius = rIn.ReadReal();
            pS->dMinorRadius = rIn.ReadReal();
            break;
        case 6: // SurfaceOfLinearExtrusion
            pS->sDirection = ReadDir3(rIn);
            pS->pBasisCurve = ReadCurve3d(rIn);
            break;
        case 7: // SurfaceOfRevolution
            pS->sLocation = ReadPnt3(rIn);
            pS->sDirection = ReadDir3(rIn);
            pS->pBasisCurve = ReadCurve3d(rIn);
            break;
        case 8: // Bezier
        {
            pS->bURational = rIn.ReadBool();
            pS->bVRational = rIn.ReadBool();
            pS->iUDegree = rIn.ReadInt();
            pS->iVDegree = rIn.ReadInt();
            pS->iNbUPoles = pS->iUDegree + 1;
            pS->iNbVPoles = pS->iVDegree + 1;
            const bool bWeighted = pS->bURational || pS->bVRational;
            for (int ii = 0; ii < pS->iNbUPoles; ++ii)
            {
                for (int jj = 0; jj < pS->iNbVPoles; ++jj)
                {
                    pS->sPoles.push_back(ReadPnt3(rIn));
                    if (bWeighted)
                    {
                        pS->dWeights.push_back(rIn.ReadReal());
                    }
                }
            }
            break;
        }
        case 9: // BSpline
        {
            pS->bURational = rIn.ReadBool();
            pS->bVRational = rIn.ReadBool();
            pS->bUPeriodic = rIn.ReadBool();
            pS->bVPeriodic = rIn.ReadBool();
            pS->iUDegree = rIn.ReadInt();
            pS->iVDegree = rIn.ReadInt();
            pS->iNbUPoles = rIn.ReadCount("BSpline surface U poles");
            pS->iNbVPoles = rIn.ReadCount("BSpline surface V poles");
            const int iNbUKnots = rIn.ReadCount("BSpline surface U knots");
            const int iNbVKnots = rIn.ReadCount("BSpline surface V knots");
            const bool bWeighted = pS->bURational || pS->bVRational;
            for (int ii = 0; ii < pS->iNbUPoles; ++ii)
            {
                for (int jj = 0; jj < pS->iNbVPoles; ++jj)
                {
                    pS->sPoles.push_back(ReadPnt3(rIn));
                    if (bWeighted)
                    {
                        pS->dWeights.push_back(rIn.ReadReal());
                    }
                }
            }
            for (int ii = 0; ii < iNbUKnots; ++ii)
            {
                pS->dUKnots.push_back(rIn.ReadReal());
                pS->iUMults.push_back(rIn.ReadInt());
            }
            for (int ii = 0; ii < iNbVKnots; ++ii)
            {
                pS->dVKnots.push_back(rIn.ReadReal());
                pS->iVMults.push_back(rIn.ReadInt());
            }
            break;
        }
        case 10: // RectangularTrimmed
            pS->dU1 = rIn.ReadReal();
            pS->dU2 = rIn.ReadReal();
            pS->dV1 = rIn.ReadReal();
            pS->dV2 = rIn.ReadReal();
            pS->pBasisSurface = ReadSurface(rIn);
            break;
        case 11: // Offset
            pS->dOffset = rIn.ReadReal();
            pS->pBasisSurface = ReadSurface(rIn);
            break;
        default:
            Fail("unsupported surface type " + std::to_string(iType));
    }
    return pS;
}

// ---- Sections ----

void ExpectKeyword(CTokenStream& rIn, const char* pKeyword)
{
    std::string sWord;
    if (!rIn.ReadWord(sWord) || sWord != pKeyword)
    {
        Fail(std::string("expected section keyword '") + pKeyword + "', got '" + sWord + "'");
    }
}

void ReadLocations(CTokenStream& rIn, SOcctModel& rModel)
{
    ExpectKeyword(rIn, "Locations");
    const int iNb = rIn.ReadCount("Locations");
    rModel.sLocations.reserve(static_cast<size_t>(iNb));
    for (int ii = 0; ii < iNb; ++ii)
    {
        SLocation sLoc;
        const int iTypLoc = rIn.ReadInt();
        if (iTypLoc == 1)
        {
            sLoc.bElementary = true;
            for (int iRow = 0; iRow < 3; ++iRow)
            {
                for (int iCol = 0; iCol < 4; ++iCol)
                {
                    sLoc.dMatrix[iRow][iCol] = rIn.ReadReal();
                }
            }
        }
        else if (iTypLoc == 2)
        {
            sLoc.bElementary = false;
            int iL1 = rIn.ReadInt();
            while (iL1 != 0)
            {
                SLocationTerm sTerm;
                sTerm.iLocationIndex = iL1;
                sTerm.iPower = rIn.ReadInt();
                sLoc.sTerms.push_back(sTerm);
                iL1 = rIn.ReadInt();
            }
        }
        else
        {
            Fail("unsupported location type " + std::to_string(iTypLoc));
        }
        rModel.sLocations.push_back(std::move(sLoc));
    }
}

void ReadCurve2ds(CTokenStream& rIn, SOcctModel& rModel)
{
    ExpectKeyword(rIn, "Curve2ds");
    const int iNb = rIn.ReadCount("Curve2ds");
    for (int ii = 0; ii < iNb; ++ii)
    {
        rModel.sCurve2ds.push_back(*ReadCurve2d(rIn));
    }
}

void ReadCurves(CTokenStream& rIn, SOcctModel& rModel)
{
    ExpectKeyword(rIn, "Curves");
    const int iNb = rIn.ReadCount("Curves");
    for (int ii = 0; ii < iNb; ++ii)
    {
        rModel.sCurves.push_back(*ReadCurve3d(rIn));
    }
}

void ReadSurfaces(CTokenStream& rIn, SOcctModel& rModel)
{
    ExpectKeyword(rIn, "Surfaces");
    const int iNb = rIn.ReadCount("Surfaces");
    for (int ii = 0; ii < iNb; ++ii)
    {
        rModel.sSurfaces.push_back(*ReadSurface(rIn));
    }
}

void ReadPolygon3D(CTokenStream& rIn, SOcctModel& rModel)
{
    ExpectKeyword(rIn, "Polygon3D");
    const int iNb = rIn.ReadCount("Polygon3D");
    for (int ii = 0; ii < iNb; ++ii)
    {
        SPolygon3d sPoly;
        const int iNbNodes = rIn.ReadCount("Polygon3D nodes");
        const bool bHasParams = rIn.ReadBool();
        sPoly.dDeflection = rIn.ReadReal();
        for (int jj = 0; jj < iNbNodes; ++jj)
        {
            sPoly.sNodes.push_back(ReadPnt3(rIn));
        }
        if (bHasParams)
        {
            for (int jj = 0; jj < iNbNodes; ++jj)
            {
                sPoly.dParams.push_back(rIn.ReadReal());
            }
        }
        rModel.sPolygons3d.push_back(std::move(sPoly));
    }
}

void ReadPolygonOnTriangulations(CTokenStream& rIn, SOcctModel& rModel)
{
    ExpectKeyword(rIn, "PolygonOnTriangulations");
    const int iNb = rIn.ReadCount("PolygonOnTriangulations");
    for (int ii = 0; ii < iNb; ++ii)
    {
        SPolygonOnTriangulation sPoly;
        const int iNbNodes = rIn.ReadCount("PolygonOnTriangulation nodes");
        for (int jj = 0; jj < iNbNodes; ++jj)
        {
            sPoly.iNodes.push_back(rIn.ReadInt());
        }
        std::string sP;
        rIn.ReadWord(sP); // "p"
        sPoly.dDeflection = rIn.ReadReal();
        const bool bHasParams = rIn.ReadBool();
        if (bHasParams)
        {
            for (int jj = 0; jj < iNbNodes; ++jj)
            {
                sPoly.dParams.push_back(rIn.ReadReal());
            }
        }
        rModel.sPolygonsOnTri.push_back(std::move(sPoly));
    }
}

void ReadTriangulations(CTokenStream& rIn, SOcctModel& rModel)
{
    ExpectKeyword(rIn, "Triangulations");
    const int iNb = rIn.ReadCount("Triangulations");
    for (int ii = 0; ii < iNb; ++ii)
    {
        STriangulation sTri;
        sTri.iNbNodes = rIn.ReadCount("Triangulation nodes");
        sTri.iNbTriangles = rIn.ReadCount("Triangulation triangles");
        sTri.bHasUV = rIn.ReadBool();
        if (rModel.iFormatVersion >= 3)
        {
            sTri.bHasNormals = rIn.ReadBool();
        }
        sTri.dDeflection = rIn.ReadReal();
        for (int jj = 0; jj < sTri.iNbNodes; ++jj)
        {
            sTri.sNodes.push_back(ReadPnt3(rIn));
        }
        if (sTri.bHasUV)
        {
            for (int jj = 0; jj < sTri.iNbNodes; ++jj)
            {
                sTri.sUVNodes.push_back(ReadPnt2(rIn));
            }
        }
        for (int jj = 0; jj < sTri.iNbTriangles; ++jj)
        {
            std::array<int, 3> aTri{};
            aTri[0] = rIn.ReadInt();
            aTri[1] = rIn.ReadInt();
            aTri[2] = rIn.ReadInt();
            sTri.iTriangles.push_back(aTri);
        }
        if (sTri.bHasNormals)
        {
            for (int jj = 0; jj < sTri.iNbNodes; ++jj)
            {
                sTri.sNormals.push_back(ReadDir3(rIn));
            }
        }
        rModel.sTriangulations.push_back(std::move(sTri));
    }
}

// ---- Topology (TShapes graph) ----

EShapeType ReadShapeEnum(CTokenStream& rIn)
{
    std::string sWord;
    if (!rIn.ReadWord(sWord) || sWord.size() < 2)
    {
        Fail("expected TShape type code");
    }
    const char c0 = sWord[0];
    const char c1 = sWord[1];
    switch (c0)
    {
        case 'V':
            return EShapeType::Vertex;
        case 'E':
            return EShapeType::Edge;
        case 'W':
            return EShapeType::Wire;
        case 'F':
            return EShapeType::Face;
        case 'S':
            return (c1 == 'h') ? EShapeType::Shell : EShapeType::Solid;
        case 'C':
            return (c1 == 'S') ? EShapeType::CompSolid : EShapeType::Compound;
        default:
            Fail(std::string("unknown TShape code '") + sWord + "'");
    }
}

EContinuity ReadRegularity(CTokenStream& rIn)
{
    std::string sWord;
    rIn.ReadWord(sWord);
    if (sWord.size() >= 2)
    {
        if (sWord[0] == 'C')
        {
            switch (sWord[1])
            {
                case '0':
                    return EContinuity::C0;
                case '1':
                    return EContinuity::C1;
                case '2':
                    return EContinuity::C2;
                case '3':
                    return EContinuity::C3;
                case 'N':
                    return EContinuity::CN;
            }
        }
        else if (sWord[0] == 'G')
        {
            switch (sWord[1])
            {
                case '1':
                    return EContinuity::G1;
                case '2':
                    return EContinuity::G2;
            }
        }
    }
    return EContinuity::C0;
}

void ReadVertexGeometry(CTokenStream& rIn, SVertexData& rV)
{
    rV.dTolerance = rIn.ReadReal();
    rV.sPoint = ReadPnt3(rIn);
    int iVal = 0;
    do
    {
        const double dParam = rIn.ReadReal();
        iVal = rIn.ReadInt();
        SPointRepr sRepr;
        sRepr.iKind = iVal;
        sRepr.dParam = dParam;
        switch (iVal)
        {
            case 1:
                sRepr.iCurve = rIn.ReadInt();
                break;
            case 2:
                sRepr.iPCurve = rIn.ReadInt();
                sRepr.iSurface = rIn.ReadInt();
                break;
            case 3:
                sRepr.dParam2 = rIn.ReadReal();
                sRepr.iSurface = rIn.ReadInt();
                break;
            default:
                break;
        }
        if (iVal > 0)
        {
            sRepr.iLocation = rIn.ReadInt();
            rV.sReprs.push_back(sRepr);
        }
    } while (iVal > 0);
}

void ReadEdgeGeometry(CTokenStream& rIn, SEdgeData& rE, int iFormatVersion)
{
    rE.dTolerance = rIn.ReadReal();
    rE.bSameParameter = rIn.ReadBool();
    rE.bSameRange = rIn.ReadBool();
    rE.bDegenerated = rIn.ReadBool();
    int iVal = 0;
    do
    {
        iVal = rIn.ReadInt();
        SCurveRepr sRepr;
        sRepr.iKind = iVal;
        switch (iVal)
        {
            case 1: // Curve 3D
                sRepr.iCurve3d = rIn.ReadInt();
                sRepr.iLocation = rIn.ReadInt();
                sRepr.dFirst = rIn.ReadReal();
                sRepr.dLast = rIn.ReadReal();
                rE.sReprs.push_back(sRepr);
                break;
            case 2: // Curve on surface
            case 3: // Curve on closed surface
            {
                const bool bClosed = (iVal == 3);
                sRepr.iPCurve = rIn.ReadInt();
                if (bClosed)
                {
                    sRepr.iPCurve2 = rIn.ReadInt();
                    sRepr.eContinuity = ReadRegularity(rIn);
                }
                sRepr.iSurface = rIn.ReadInt();
                sRepr.iLocation = rIn.ReadInt();
                sRepr.dFirst = rIn.ReadReal();
                sRepr.dLast = rIn.ReadReal();
                if (iFormatVersion == 2)
                {
                    // UV points: read and discard (4 reals)
                    rIn.ReadReal();
                    rIn.ReadReal();
                    rIn.ReadReal();
                    rIn.ReadReal();
                }
                rE.sReprs.push_back(sRepr);
                break;
            }
            case 4: // Regularity
                sRepr.eContinuity = ReadRegularity(rIn);
                sRepr.iSurface = rIn.ReadInt();
                sRepr.iLocation = rIn.ReadInt();
                sRepr.iSurface2 = rIn.ReadInt();
                sRepr.iLocation2 = rIn.ReadInt();
                rE.sReprs.push_back(sRepr);
                break;
            case 5: // Polygon3D
                sRepr.iPolygon3d = rIn.ReadInt();
                sRepr.iLocation = rIn.ReadInt();
                rE.sReprs.push_back(sRepr);
                break;
            case 6: // Polygon on triangulation
            case 7: // Polygon on closed triangulation
            {
                const bool bClosed = (iVal == 7);
                sRepr.iPolyOnTri = rIn.ReadInt();
                if (bClosed)
                {
                    sRepr.iPolyOnTri2 = rIn.ReadInt();
                }
                sRepr.iTriangulation = rIn.ReadInt();
                sRepr.iLocation = rIn.ReadInt();
                rE.sReprs.push_back(sRepr);
                break;
            }
            default:
                break;
        }
    } while (iVal > 0);
}

void ReadFaceGeometry(CTokenStream& rIn, SFaceData& rF)
{
    const int iVal = rIn.ReadInt(); // natural restriction, or 2 == triangulation only
    if (iVal == 0 || iVal == 1)
    {
        rF.bNaturalRestriction = (iVal == 1);
        rF.dTolerance = rIn.ReadReal();
        rF.iSurface = rIn.ReadInt();
        rF.iLocation = rIn.ReadInt();
    }
    else if (iVal == 2)
    {
        rF.iTriangulation = rIn.ReadInt();
        return; // OCCT breaks here, no trailing getlines
    }

    // BUC60769: consume the rest of the surface line, then peek the next line. If it begins with
    // '2' it carries a triangulation index; otherwise it is the blank separator line.
    std::string sLine;
    rIn.GetLine(sLine);
    rIn.GetLine(sLine);
    if (!sLine.empty() && sLine[0] == '2')
    {
        rF.iTriangulation = std::atoi(sLine.c_str() + 2);
    }
}

void SetFlags(SShapeRecord& rRec, const std::string& rFlags, int iFormatVersion)
{
    auto Bit = [&](size_t i)
    {
        return i < rFlags.size() && rFlags[i] == '1';
    };
    rRec.bFree = Bit(0);
    rRec.bModified = Bit(1);
    rRec.bChecked = (iFormatVersion >= 2) && Bit(2);
    rRec.bOrientable = Bit(3);
    rRec.bClosed = Bit(4);
    rRec.bInfinite = Bit(5);
    rRec.bConvex = Bit(6);
}

// Reads one sub-shape reference. Returns false when the null terminator '*' is read.
bool ReadSubShapeRef(CTokenStream& rIn, int iNbShapes, SSubShapeRef& rRef)
{
    std::string sWord;
    if (!rIn.ReadWord(sWord) || sWord.empty())
    {
        return false;
    }
    if (sWord[0] == '*')
    {
        return false;
    }
    switch (sWord[0])
    {
        case '+':
            rRef.eOrientation = EOrientation::Forward;
            break;
        case '-':
            rRef.eOrientation = EOrientation::Reversed;
            break;
        case 'i':
            rRef.eOrientation = EOrientation::Internal;
            break;
        case 'e':
            rRef.eOrientation = EOrientation::External;
            break;
        default:
            Fail(std::string("malformed sub-shape reference '") + sWord + "'");
    }
    // The token after the orientation char must be a positive integer in 1..iNbShapes. std::atoi
    // would silently accept a non-numeric suffix as 0 (and never range-checks), injecting an invalid
    // reference into the model. Parse strictly and fail on anything out of range.
    const char* pNum = sWord.c_str() + 1;
    char* pEnd = nullptr;
    const long lToken = std::strtol(pNum, &pEnd, 10);
    if (pEnd == pNum || *pEnd != '\0' || lToken < 1 || lToken > static_cast<long>(iNbShapes))
    {
        Fail(std::string("invalid sub-shape reference token '") + sWord + "'");
    }
    rRef.iFileToken = static_cast<int>(lToken);
    rRef.iStorageIndex = iNbShapes - rRef.iFileToken + 1;
    rRef.iLocation = rIn.ReadInt();
    return true;
}

void ReadShapes(CTokenStream& rIn, SOcctModel& rModel)
{
    ExpectKeyword(rIn, "TShapes");
    const int iNbShapes = rIn.ReadCount("TShapes");
    rModel.sShapes.reserve(static_cast<size_t>(iNbShapes));
    for (int ii = 0; ii < iNbShapes; ++ii)
    {
        SShapeRecord sRec;
        sRec.eType = ReadShapeEnum(rIn);
        switch (sRec.eType)
        {
            case EShapeType::Vertex:
                ReadVertexGeometry(rIn, sRec.sVertex);
                break;
            case EShapeType::Edge:
                ReadEdgeGeometry(rIn, sRec.sEdge, rModel.iFormatVersion);
                break;
            case EShapeType::Face:
                ReadFaceGeometry(rIn, sRec.sFace);
                break;
            default:
                break; // Wire/Shell/Solid/CompSolid/Compound carry no geometry
        }

        std::string sFlags;
        rIn.ReadWord(sFlags);
        SetFlags(sRec, sFlags, rModel.iFormatVersion);

        SSubShapeRef sRef;
        while (ReadSubShapeRef(rIn, iNbShapes, sRef))
        {
            sRec.sSubShapes.push_back(sRef);
            sRef = SSubShapeRef{};
        }

        rModel.sShapes.push_back(std::move(sRec));
    }
}

std::string TrimTrailingEol(const std::string& rLine)
{
    size_t uiEnd = rLine.size();
    while (uiEnd > 0 && (rLine[uiEnd - 1] == '\r' || rLine[uiEnd - 1] == '\n'))
    {
        --uiEnd;
    }
    return rLine.substr(0, uiEnd);
}

} // namespace

bool ReadOcct(const std::string& rText, SOcctModel& rModel, std::string& rError)
{
    rModel = SOcctModel{};
    CTokenStream sIn(rText);
    try
    {
        // Version banner: read lines until one matches a known banner (skips e.g.
        // "DBRep_DrawableShape" and blank lines written by DRAW's `save`).
        bool bFound = false;
        std::string sLine;
        while (sIn.GetLine(sLine))
        {
            const std::string sTrim = TrimTrailingEol(sLine);
            for (int iV = 1; iV <= 3; ++iV)
            {
                if (sTrim == THE_VERSION_BANNERS[iV])
                {
                    rModel.iFormatVersion = iV;
                    bFound = true;
                    break;
                }
            }
            if (bFound)
            {
                break;
            }
        }
        if (!bFound)
        {
            rError = "no recognised 'CASCADE Topology' version banner";
            return false;
        }

        ReadLocations(sIn, rModel);
        // Geometry section order is fixed by BRepTools_ShapeSet::ReadGeometry.
        ReadCurve2ds(sIn, rModel);
        ReadCurves(sIn, rModel);
        ReadPolygon3D(sIn, rModel);
        ReadPolygonOnTriangulations(sIn, rModel);
        ReadSurfaces(sIn, rModel);
        ReadTriangulations(sIn, rModel);
        ReadShapes(sIn, rModel);

        // Trailing root shape reference (DBRep / BRepTools::Read consume one here).
        SSubShapeRef sRoot;
        if (ReadSubShapeRef(sIn, static_cast<int>(rModel.sShapes.size()), sRoot))
        {
            rModel.sRoot = sRoot;
            rModel.bHasRoot = true;
        }
    }
    catch (const SParseError& rErr)
    {
        rError = rErr.sMessage;
        return false;
    }
    catch (const std::exception& rErr)
    {
        // Anything the parse triggers that is not an SParseError (e.g. std::bad_alloc from an
        // over-large allocation, std::length_error) is normalized into rError rather than escaping.
        rError = std::string("brep parse failed: ") + rErr.what();
        return false;
    }
    return true;
}

const char* CurveTypeName(ECurveType eType)
{
    switch (eType)
    {
        case ECurveType::Line:
            return "Line";
        case ECurveType::Circle:
            return "Circle";
        case ECurveType::Ellipse:
            return "Ellipse";
        case ECurveType::Parabola:
            return "Parabola";
        case ECurveType::Hyperbola:
            return "Hyperbola";
        case ECurveType::Bezier:
            return "Bezier";
        case ECurveType::BSpline:
            return "BSpline";
        case ECurveType::Trimmed:
            return "Trimmed";
        case ECurveType::Offset:
            return "Offset";
        default:
            return "Unknown";
    }
}

const char* SurfaceTypeName(ESurfaceType eType)
{
    switch (eType)
    {
        case ESurfaceType::Plane:
            return "Plane";
        case ESurfaceType::Cylinder:
            return "Cylinder";
        case ESurfaceType::Cone:
            return "Cone";
        case ESurfaceType::Sphere:
            return "Sphere";
        case ESurfaceType::Torus:
            return "Torus";
        case ESurfaceType::LinearExtrusion:
            return "LinearExtrusion";
        case ESurfaceType::Revolution:
            return "Revolution";
        case ESurfaceType::Bezier:
            return "Bezier";
        case ESurfaceType::BSpline:
            return "BSpline";
        case ESurfaceType::RectangularTrimmed:
            return "RectangularTrimmed";
        case ESurfaceType::Offset:
            return "Offset";
        default:
            return "Unknown";
    }
}

const char* ShapeTypeName(EShapeType eType)
{
    switch (eType)
    {
        case EShapeType::Vertex:
            return "Vertex";
        case EShapeType::Edge:
            return "Edge";
        case EShapeType::Wire:
            return "Wire";
        case EShapeType::Face:
            return "Face";
        case EShapeType::Shell:
            return "Shell";
        case EShapeType::Solid:
            return "Solid";
        case EShapeType::CompSolid:
            return "CompSolid";
        case EShapeType::Compound:
            return "Compound";
        default:
            return "Unknown";
    }
}

} // namespace occt
