// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// Hand-written reader for OCCT's binary `.brep` format (BinTools_ShapeSet). It mirrors
// BinTools_ShapeSet::Read / BinTools_{Curve,Curve2d,Surface,Location}Set::Read byte-for-byte and
// fills the same neutral SOcctModel the ASCII parser produces, so the downstream topology resolve +
// staging pipeline is unchanged.
//
// Wire format recap (BinTools): section headers are ASCII text ("Locations 5\n", "Curves 12\n", ...)
// embedded in an otherwise binary stream; scalars are raw host-order bytes (assumed little-endian):
//   integer = 4 bytes, real = 8-byte double, shortreal = 4-byte float, extchar = 2 bytes, bool/byte
//   = 1 byte. Geometry/shape type tags and shape orientation are single bytes.

#include "OcctBinaryParser.h"

#include <cerrno>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <cstring>

// The float/double scalar reads below reconstruct the 8-/4-byte IEEE-754 payload by memcpy'ing the
// little-endian wire bytes into a host integer. That reinterpretation is only correct on a
// little-endian host (x86-64, ARM-LE). Fail loudly at compile time on a big-endian target rather than
// silently producing wrong geometry. (__BYTE_ORDER__ is provided by GCC/Clang; both compilers used here.)
#if defined(__BYTE_ORDER__) && defined(__ORDER_LITTLE_ENDIAN__)
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "OCCT binary .brep parser assumes a little-endian host (real/shortreal byte order).");
#endif

namespace occt
{
namespace
{

// Binary topology format banners OCCT recognises (index == BinTools_FormatVersion).
const char* const THE_BIN_VERSION_BANNERS[] = {
    "", "Open CASCADE Topology V1 (c)", "Open CASCADE Topology V2 (c)", "Open CASCADE Topology V3 (c)", "Open CASCADE Topology V4, (c) Open Cascade",
};
constexpr int THE_BIN_VERSION_LOWER = 1;
constexpr int THE_BIN_VERSION_UPPER = 4;

struct SBinParseError
{
    std::string sMessage;
};

[[noreturn]] void Fail(const std::string& rMessage)
{
    throw SBinParseError{ rMessage };
}

// Byte cursor over the whole file buffer. Provides the binary scalar reads BinTools::Get* perform
// plus the text-token reads (operator>>) BinTools uses for section headers.
class CBinReader
{
public:

    explicit CBinReader(const std::string& rData) : m_rData(rData)
    {
    }

    size_t Pos() const
    {
        return m_uiPos;
    }

    void Seek(size_t uiPos)
    {
        m_uiPos = uiPos;
    }

    bool Eof() const
    {
        return m_uiPos >= m_rData.size();
    }

    // One raw byte (mirrors std::istream::get(): returns -1 at end of stream, does not throw).
    int Get()
    {
        if (m_uiPos >= m_rData.size())
        {
            return -1;
        }
        return static_cast<unsigned char>(m_rData[m_uiPos++]);
    }

    uint8_t GetByte()
    {
        if (m_uiPos >= m_rData.size())
        {
            Fail("unexpected end of file");
        }
        return static_cast<uint8_t>(m_rData[m_uiPos++]);
    }

    bool GetBool()
    {
        return GetByte() != 0;
    }

    int32_t GetInteger()
    {
        return static_cast<int32_t>(ReadLE(4));
    }

    uint16_t GetExtChar()
    {
        return static_cast<uint16_t>(ReadLE(2));
    }

    double GetReal()
    {
        const uint64_t uiBits = ReadLE(8);
        double dValue = 0.0;
        std::memcpy(&dValue, &uiBits, sizeof(dValue));
        return dValue;
    }

    float GetShortReal()
    {
        const uint32_t uiBits = static_cast<uint32_t>(ReadLE(4));
        float fValue = 0.0f;
        std::memcpy(&fValue, &uiBits, sizeof(fValue));
        return fValue;
    }

    // operator>>(std::string): skip leading whitespace, read up to the next whitespace byte.
    bool ReadWord(std::string& rWord)
    {
        SkipWhitespace();
        if (Eof())
        {
            rWord.clear();
            return false;
        }
        const size_t uiBegin = m_uiPos;
        while (m_uiPos < m_rData.size() && !IsSpace(m_rData[m_uiPos]))
        {
            ++m_uiPos;
        }
        rWord.assign(m_rData, uiBegin, m_uiPos - uiBegin);
        return true;
    }

    // operator>>(int): skip leading whitespace, parse the longest signed-integer prefix.
    int ReadTextInt()
    {
        SkipWhitespace();
        const char* pBegin = m_rData.c_str() + m_uiPos;
        char* pEnd = nullptr;
        errno = 0;
        const long lValue = std::strtol(pBegin, &pEnd, 10);
        if (pEnd == pBegin)
        {
            Fail("expected integer at offset " + std::to_string(m_uiPos));
        }
        if (errno == ERANGE || lValue < static_cast<long>(INT_MIN) || lValue > static_cast<long>(INT_MAX))
        {
            Fail("integer out of range at offset " + std::to_string(m_uiPos));
        }
        m_uiPos += static_cast<size_t>(pEnd - pBegin);
        return static_cast<int>(lValue);
    }

    // operator>>(char): skip leading whitespace, return one byte (used for shape orientation / '*').
    uint8_t ReadCharSkipWs()
    {
        SkipWhitespace();
        return GetByte();
    }

    // getline to '\n' (consumed). Trailing '\r' is stripped to match the OCCT banner check.
    bool GetLine(std::string& rLine)
    {
        if (Eof())
        {
            rLine.clear();
            return false;
        }
        const size_t uiBegin = m_uiPos;
        while (m_uiPos < m_rData.size() && m_rData[m_uiPos] != '\n')
        {
            ++m_uiPos;
        }
        size_t uiEnd = m_uiPos;
        if (m_uiPos < m_rData.size())
        {
            ++m_uiPos; // consume '\n'
        }
        while (uiEnd > uiBegin && (m_rData[uiEnd - 1] == '\r'))
        {
            --uiEnd;
        }
        rLine.assign(m_rData, uiBegin, uiEnd - uiBegin);
        return true;
    }

private:

    static bool IsSpace(char c)
    {
        const unsigned char u = static_cast<unsigned char>(c);
        return u == ' ' || u == '\t' || u == '\n' || u == '\r' || u == '\v' || u == '\f';
    }

    void SkipWhitespace()
    {
        while (m_uiPos < m_rData.size() && IsSpace(m_rData[m_uiPos]))
        {
            ++m_uiPos;
        }
    }

    uint64_t ReadLE(size_t uiCount)
    {
        if (m_uiPos + uiCount > m_rData.size())
        {
            Fail("unexpected end of file");
        }
        uint64_t uiValue = 0;
        for (size_t ii = 0; ii < uiCount; ++ii)
        {
            uiValue |= static_cast<uint64_t>(static_cast<unsigned char>(m_rData[m_uiPos + ii])) << (8 * ii);
        }
        m_uiPos += uiCount;
        return uiValue;
    }

    const std::string& m_rData;
    size_t m_uiPos = 0;
};

SPnt3 ReadPnt3(CBinReader& rIn)
{
    SPnt3 s;
    s.dX = rIn.GetReal();
    s.dY = rIn.GetReal();
    s.dZ = rIn.GetReal();
    return s;
}

SDir3 ReadDir3(CBinReader& rIn)
{
    SDir3 s;
    s.dX = rIn.GetReal();
    s.dY = rIn.GetReal();
    s.dZ = rIn.GetReal();
    return s;
}

SPnt2 ReadPnt2(CBinReader& rIn)
{
    SPnt2 s;
    s.dX = rIn.GetReal();
    s.dY = rIn.GetReal();
    return s;
}

SDir2 ReadDir2(CBinReader& rIn)
{
    SDir2 s;
    s.dX = rIn.GetReal();
    s.dY = rIn.GetReal();
    return s;
}

// ---- Geometry: 3D curves (BinTools_CurveSet::ReadCurve) ----

std::shared_ptr<SCurve3d> ReadCurve3d(CBinReader& rIn)
{
    auto pC = std::make_shared<SCurve3d>();
    const int iType = rIn.GetByte();
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
            pC->dRadius = rIn.GetReal();
            break;
        case 3: // Ellipse
            pC->sLocation = ReadPnt3(rIn);
            pC->sAxis = ReadDir3(rIn);
            pC->sXAxis = ReadDir3(rIn);
            pC->sYAxis = ReadDir3(rIn);
            pC->dRadius = rIn.GetReal();
            pC->dMinorRadius = rIn.GetReal();
            break;
        case 4: // Parabola
            pC->sLocation = ReadPnt3(rIn);
            pC->sAxis = ReadDir3(rIn);
            pC->sXAxis = ReadDir3(rIn);
            pC->sYAxis = ReadDir3(rIn);
            pC->dFocal = rIn.GetReal();
            break;
        case 5: // Hyperbola
            pC->sLocation = ReadPnt3(rIn);
            pC->sAxis = ReadDir3(rIn);
            pC->sXAxis = ReadDir3(rIn);
            pC->sYAxis = ReadDir3(rIn);
            pC->dRadius = rIn.GetReal();
            pC->dMinorRadius = rIn.GetReal();
            break;
        case 6: // Bezier
        {
            pC->bRational = rIn.GetBool();
            pC->iDegree = static_cast<int>(rIn.GetExtChar());
            const int iNbPoles = pC->iDegree + 1;
            for (int ii = 0; ii < iNbPoles; ++ii)
            {
                pC->sPoles.push_back(ReadPnt3(rIn));
                if (pC->bRational)
                {
                    pC->dWeights.push_back(rIn.GetReal());
                }
            }
            break;
        }
        case 7: // BSpline
        {
            pC->bRational = rIn.GetBool();
            pC->bPeriodic = rIn.GetBool();
            pC->iDegree = static_cast<int>(rIn.GetExtChar());
            const int iNbPoles = rIn.GetInteger();
            const int iNbKnots = rIn.GetInteger();
            for (int ii = 0; ii < iNbPoles; ++ii)
            {
                pC->sPoles.push_back(ReadPnt3(rIn));
                if (pC->bRational)
                {
                    pC->dWeights.push_back(rIn.GetReal());
                }
            }
            for (int ii = 0; ii < iNbKnots; ++ii)
            {
                pC->dKnots.push_back(rIn.GetReal());
                pC->iMults.push_back(rIn.GetInteger());
            }
            break;
        }
        case 8: // Trimmed
            pC->dFirst = rIn.GetReal();
            pC->dLast = rIn.GetReal();
            pC->pBasis = ReadCurve3d(rIn);
            break;
        case 9: // Offset
            pC->dOffset = rIn.GetReal();
            pC->sOffsetDir = ReadDir3(rIn);
            pC->pBasis = ReadCurve3d(rIn);
            break;
        default:
            Fail("unsupported 3D curve type " + std::to_string(iType));
    }
    return pC;
}

// ---- Geometry: 2D curves (BinTools_Curve2dSet::ReadCurve2d) ----

std::shared_ptr<SCurve2d> ReadCurve2d(CBinReader& rIn)
{
    auto pC = std::make_shared<SCurve2d>();
    const int iType = rIn.GetByte();
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
            pC->dRadius = rIn.GetReal();
            break;
        case 3: // Ellipse
            pC->sLocation = ReadPnt2(rIn);
            pC->sXAxis = ReadDir2(rIn);
            pC->sYAxis = ReadDir2(rIn);
            pC->dRadius = rIn.GetReal();
            pC->dMinorRadius = rIn.GetReal();
            break;
        case 4: // Parabola
            pC->sLocation = ReadPnt2(rIn);
            pC->sXAxis = ReadDir2(rIn);
            pC->sYAxis = ReadDir2(rIn);
            pC->dFocal = rIn.GetReal();
            break;
        case 5: // Hyperbola
            pC->sLocation = ReadPnt2(rIn);
            pC->sXAxis = ReadDir2(rIn);
            pC->sYAxis = ReadDir2(rIn);
            pC->dRadius = rIn.GetReal();
            pC->dMinorRadius = rIn.GetReal();
            break;
        case 6: // Bezier
        {
            pC->bRational = rIn.GetBool();
            pC->iDegree = static_cast<int>(rIn.GetExtChar());
            const int iNbPoles = pC->iDegree + 1;
            for (int ii = 0; ii < iNbPoles; ++ii)
            {
                pC->sPoles.push_back(ReadPnt2(rIn));
                if (pC->bRational)
                {
                    pC->dWeights.push_back(rIn.GetReal());
                }
            }
            break;
        }
        case 7: // BSpline
        {
            pC->bRational = rIn.GetBool();
            pC->bPeriodic = rIn.GetBool();
            pC->iDegree = static_cast<int>(rIn.GetExtChar());
            const int iNbPoles = rIn.GetInteger();
            const int iNbKnots = rIn.GetInteger();
            for (int ii = 0; ii < iNbPoles; ++ii)
            {
                pC->sPoles.push_back(ReadPnt2(rIn));
                if (pC->bRational)
                {
                    pC->dWeights.push_back(rIn.GetReal());
                }
            }
            for (int ii = 0; ii < iNbKnots; ++ii)
            {
                pC->dKnots.push_back(rIn.GetReal());
                pC->iMults.push_back(rIn.GetInteger());
            }
            break;
        }
        case 8: // Trimmed
            pC->dFirst = rIn.GetReal();
            pC->dLast = rIn.GetReal();
            pC->pBasis = ReadCurve2d(rIn);
            break;
        case 9: // Offset (2D: no direction)
            pC->dOffset = rIn.GetReal();
            pC->pBasis = ReadCurve2d(rIn);
            break;
        default:
            Fail("unsupported 2D curve type " + std::to_string(iType));
    }
    return pC;
}

// ---- Geometry: surfaces (BinTools_SurfaceSet::ReadSurface) ----

void ReadAx3(CBinReader& rIn, SSurface& rS)
{
    rS.sLocation = ReadPnt3(rIn);
    rS.sAxis = ReadDir3(rIn);
    rS.sXAxis = ReadDir3(rIn);
    rS.sYAxis = ReadDir3(rIn);
}

std::shared_ptr<SSurface> ReadSurface(CBinReader& rIn)
{
    auto pS = std::make_shared<SSurface>();
    const int iType = rIn.GetByte();
    pS->eType = static_cast<ESurfaceType>(iType);
    switch (iType)
    {
        case 1: // Plane
            ReadAx3(rIn, *pS);
            break;
        case 2: // Cylinder
            ReadAx3(rIn, *pS);
            pS->dRadius = rIn.GetReal();
            break;
        case 3: // Cone
            ReadAx3(rIn, *pS);
            pS->dRadius = rIn.GetReal();
            pS->dSemiAngle = rIn.GetReal();
            break;
        case 4: // Sphere
            ReadAx3(rIn, *pS);
            pS->dRadius = rIn.GetReal();
            break;
        case 5: // Torus
            ReadAx3(rIn, *pS);
            pS->dRadius = rIn.GetReal();
            pS->dMinorRadius = rIn.GetReal();
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
            pS->bURational = rIn.GetBool();
            pS->bVRational = rIn.GetBool();
            pS->iUDegree = static_cast<int>(rIn.GetExtChar());
            pS->iVDegree = static_cast<int>(rIn.GetExtChar());
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
                        pS->dWeights.push_back(rIn.GetReal());
                    }
                }
            }
            break;
        }
        case 9: // BSpline
        {
            pS->bURational = rIn.GetBool();
            pS->bVRational = rIn.GetBool();
            pS->bUPeriodic = rIn.GetBool();
            pS->bVPeriodic = rIn.GetBool();
            pS->iUDegree = static_cast<int>(rIn.GetExtChar());
            pS->iVDegree = static_cast<int>(rIn.GetExtChar());
            pS->iNbUPoles = rIn.GetInteger();
            pS->iNbVPoles = rIn.GetInteger();
            const int iNbUKnots = rIn.GetInteger();
            const int iNbVKnots = rIn.GetInteger();
            const bool bWeighted = pS->bURational || pS->bVRational;
            for (int ii = 0; ii < pS->iNbUPoles; ++ii)
            {
                for (int jj = 0; jj < pS->iNbVPoles; ++jj)
                {
                    pS->sPoles.push_back(ReadPnt3(rIn));
                    if (bWeighted)
                    {
                        pS->dWeights.push_back(rIn.GetReal());
                    }
                }
            }
            for (int ii = 0; ii < iNbUKnots; ++ii)
            {
                pS->dUKnots.push_back(rIn.GetReal());
                pS->iUMults.push_back(rIn.GetInteger());
            }
            for (int ii = 0; ii < iNbVKnots; ++ii)
            {
                pS->dVKnots.push_back(rIn.GetReal());
                pS->iVMults.push_back(rIn.GetInteger());
            }
            break;
        }
        case 10: // RectangularTrimmed
            pS->dU1 = rIn.GetReal();
            pS->dU2 = rIn.GetReal();
            pS->dV1 = rIn.GetReal();
            pS->dV2 = rIn.GetReal();
            pS->pBasisSurface = ReadSurface(rIn);
            break;
        case 11: // Offset
            pS->dOffset = rIn.GetReal();
            pS->pBasisSurface = ReadSurface(rIn);
            break;
        default:
            Fail("unsupported surface type " + std::to_string(iType));
    }
    return pS;
}

// ---- Sections ----

void ExpectKeyword(CBinReader& rIn, const char* pKeyword)
{
    std::string sWord;
    if (!rIn.ReadWord(sWord) || sWord != pKeyword)
    {
        Fail(std::string("expected section keyword '") + pKeyword + "', got '" + sWord + "'");
    }
}

// Reads "Keyword <count>\n" and consumes the trailing newline so the binary payload follows.
int ReadSectionHeader(CBinReader& rIn, const char* pKeyword)
{
    ExpectKeyword(rIn, pKeyword);
    const int iNb = rIn.ReadTextInt();
    rIn.Get(); // remove the single trailing LF
    return iNb;
}

void ReadLocations(CBinReader& rIn, SOcctModel& rModel)
{
    const int iNb = ReadSectionHeader(rIn, "Locations");
    rModel.sLocations.reserve(static_cast<size_t>(iNb));
    for (int ii = 0; ii < iNb; ++ii)
    {
        SLocation sLoc;
        const int iTypLoc = rIn.GetByte();
        if (iTypLoc == 1)
        {
            sLoc.bElementary = true;
            for (int iRow = 0; iRow < 3; ++iRow)
            {
                for (int iCol = 0; iCol < 4; ++iCol)
                {
                    sLoc.dMatrix[iRow][iCol] = rIn.GetReal();
                }
            }
        }
        else if (iTypLoc == 2)
        {
            sLoc.bElementary = false;
            int iL1 = rIn.GetInteger();
            while (iL1 != 0)
            {
                SLocationTerm sTerm;
                sTerm.iLocationIndex = iL1;
                sTerm.iPower = rIn.GetInteger();
                sLoc.sTerms.push_back(sTerm);
                iL1 = rIn.GetInteger();
            }
        }
        else
        {
            Fail("unsupported location type " + std::to_string(iTypLoc));
        }
        rModel.sLocations.push_back(std::move(sLoc));
    }
}

void ReadCurve2ds(CBinReader& rIn, SOcctModel& rModel)
{
    const int iNb = ReadSectionHeader(rIn, "Curve2ds");
    for (int ii = 0; ii < iNb; ++ii)
    {
        rModel.sCurve2ds.push_back(*ReadCurve2d(rIn));
    }
}

void ReadCurves(CBinReader& rIn, SOcctModel& rModel)
{
    const int iNb = ReadSectionHeader(rIn, "Curves");
    for (int ii = 0; ii < iNb; ++ii)
    {
        rModel.sCurves.push_back(*ReadCurve3d(rIn));
    }
}

void ReadPolygon3D(CBinReader& rIn, SOcctModel& rModel)
{
    const int iNb = ReadSectionHeader(rIn, "Polygon3D");
    for (int ii = 0; ii < iNb; ++ii)
    {
        SPolygon3d sPoly;
        const int iNbNodes = rIn.GetInteger();
        const bool bHasParams = rIn.GetBool();
        sPoly.dDeflection = rIn.GetReal();
        for (int jj = 0; jj < iNbNodes; ++jj)
        {
            sPoly.sNodes.push_back(ReadPnt3(rIn));
        }
        if (bHasParams)
        {
            for (int jj = 0; jj < iNbNodes; ++jj)
            {
                sPoly.dParams.push_back(rIn.GetReal());
            }
        }
        rModel.sPolygons3d.push_back(std::move(sPoly));
    }
}

void ReadPolygonOnTriangulations(CBinReader& rIn, SOcctModel& rModel)
{
    const int iNb = ReadSectionHeader(rIn, "PolygonOnTriangulations");
    for (int ii = 0; ii < iNb; ++ii)
    {
        SPolygonOnTriangulation sPoly;
        const int iNbNodes = rIn.GetInteger();
        for (int jj = 0; jj < iNbNodes; ++jj)
        {
            sPoly.iNodes.push_back(rIn.GetInteger());
        }
        sPoly.dDeflection = rIn.GetReal();
        const bool bHasParams = rIn.GetBool();
        if (bHasParams)
        {
            for (int jj = 0; jj < iNbNodes; ++jj)
            {
                sPoly.dParams.push_back(rIn.GetReal());
            }
        }
        rModel.sPolygonsOnTri.push_back(std::move(sPoly));
    }
}

void ReadSurfaces(CBinReader& rIn, SOcctModel& rModel)
{
    const int iNb = ReadSectionHeader(rIn, "Surfaces");
    for (int ii = 0; ii < iNb; ++ii)
    {
        rModel.sSurfaces.push_back(*ReadSurface(rIn));
    }
}

void ReadTriangulations(CBinReader& rIn, SOcctModel& rModel)
{
    const int iNb = ReadSectionHeader(rIn, "Triangulations");
    for (int ii = 0; ii < iNb; ++ii)
    {
        STriangulation sTri;
        sTri.iNbNodes = rIn.GetInteger();
        sTri.iNbTriangles = rIn.GetInteger();
        sTri.bHasUV = rIn.GetBool();
        if (rModel.iFormatVersion >= 4)
        {
            sTri.bHasNormals = rIn.GetBool();
        }
        sTri.dDeflection = rIn.GetReal();
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
            aTri[0] = rIn.GetInteger();
            aTri[1] = rIn.GetInteger();
            aTri[2] = rIn.GetInteger();
            sTri.iTriangles.push_back(aTri);
        }
        if (sTri.bHasNormals)
        {
            for (int jj = 0; jj < sTri.iNbNodes; ++jj)
            {
                SDir3 sN;
                sN.dX = rIn.GetShortReal();
                sN.dY = rIn.GetShortReal();
                sN.dZ = rIn.GetShortReal();
                sTri.sNormals.push_back(sN);
            }
        }
        rModel.sTriangulations.push_back(std::move(sTri));
    }
}

// ---- Topology (TShapes graph) ----

// OCCT writes shape type as the raw TopAbs_ShapeEnum byte:
//   0 Compound, 1 CompSolid, 2 Solid, 3 Shell, 4 Face, 5 Wire, 6 Edge, 7 Vertex.
EShapeType ShapeEnumFromByte(int iByte)
{
    switch (iByte)
    {
        case 0:
            return EShapeType::Compound;
        case 1:
            return EShapeType::CompSolid;
        case 2:
            return EShapeType::Solid;
        case 3:
            return EShapeType::Shell;
        case 4:
            return EShapeType::Face;
        case 5:
            return EShapeType::Wire;
        case 6:
            return EShapeType::Edge;
        case 7:
            return EShapeType::Vertex;
        default:
            Fail("unexpected topology type byte " + std::to_string(iByte));
    }
}

EContinuity ContinuityFromByte(int iByte)
{
    // GeomAbs_Shape: C0, G1, C1, G2, C2, C3, CN.
    switch (iByte)
    {
        case 0:
            return EContinuity::C0;
        case 1:
            return EContinuity::G1;
        case 2:
            return EContinuity::C1;
        case 3:
            return EContinuity::G2;
        case 4:
            return EContinuity::C2;
        case 5:
            return EContinuity::C3;
        case 6:
            return EContinuity::CN;
        default:
            return EContinuity::C0;
    }
}

void ReadVertexGeometry(CBinReader& rIn, SVertexData& rV, int iFormatVersion)
{
    rV.dTolerance = rIn.GetReal();
    rV.sPoint = ReadPnt3(rIn);
    int iVal = 0;
    do
    {
        double dParam = 0.0;
        // BinTools writes the representation kind byte first, then the parameter. Format V3 reads it
        // that way directly; all other versions use OCCT's tellg/seekg heuristic which tries the
        // legacy "real then kind" layout and falls back to "kind then real" when the kind is not
        // 1/2/3. Replicated here so we consume the exact same bytes as OCCT.
        if (iFormatVersion == 3)
        {
            iVal = rIn.GetByte();
            if (iVal > 0 && iVal <= 3)
            {
                dParam = rIn.GetReal();
            }
        }
        else
        {
            const size_t uiPos = rIn.Pos();
            dParam = rIn.GetReal();
            iVal = rIn.GetByte();
            if (iVal != 1 && iVal != 2 && iVal != 3)
            {
                rIn.Seek(uiPos);
                iVal = rIn.GetByte();
                if (iVal > 0 && iVal <= 3)
                {
                    dParam = rIn.GetReal();
                }
            }
        }

        SPointRepr sRepr;
        sRepr.iKind = iVal;
        sRepr.dParam = dParam;
        switch (iVal)
        {
            case 0:
                break;
            case 1:
                sRepr.iCurve = rIn.GetInteger();
                break;
            case 2:
                sRepr.iPCurve = rIn.GetInteger();
                sRepr.iSurface = rIn.GetInteger();
                break;
            case 3:
                sRepr.dParam2 = rIn.GetReal();
                sRepr.iSurface = rIn.GetInteger();
                break;
            default:
                Fail("unexpected vertex point representation " + std::to_string(iVal));
        }
        if (iVal > 0)
        {
            sRepr.iLocation = rIn.GetInteger();
            rV.sReprs.push_back(sRepr);
        }
    } while (iVal > 0);
}

void ReadEdgeGeometry(CBinReader& rIn, SEdgeData& rE, int iFormatVersion)
{
    rE.dTolerance = rIn.GetReal();
    rE.bSameParameter = rIn.GetBool();
    rE.bSameRange = rIn.GetBool();
    rE.bDegenerated = rIn.GetBool();
    int iVal = 0;
    do
    {
        iVal = rIn.GetByte();
        SCurveRepr sRepr;
        sRepr.iKind = iVal;
        switch (iVal)
        {
            case 0:
                break;
            case 1: // Curve 3D
                sRepr.iCurve3d = rIn.GetInteger();
                sRepr.iLocation = rIn.GetInteger();
                sRepr.dFirst = rIn.GetReal();
                sRepr.dLast = rIn.GetReal();
                rE.sReprs.push_back(sRepr);
                break;
            case 2: // Curve on surface
            case 3: // Curve on closed surface
            {
                const bool bClosed = (iVal == 3);
                sRepr.iPCurve = rIn.GetInteger();
                if (bClosed)
                {
                    sRepr.iPCurve2 = rIn.GetInteger();
                    sRepr.eContinuity = ContinuityFromByte(rIn.GetByte());
                }
                sRepr.iSurface = rIn.GetInteger();
                sRepr.iLocation = rIn.GetInteger();
                sRepr.dFirst = rIn.GetReal();
                sRepr.dLast = rIn.GetReal();
                // UV points were stored only in format versions 2 and 3 (read and discard).
                if (iFormatVersion == 2 || iFormatVersion == 3)
                {
                    rIn.GetReal();
                    rIn.GetReal();
                    rIn.GetReal();
                    rIn.GetReal();
                }
                rE.sReprs.push_back(sRepr);
                break;
            }
            case 4: // Regularity
                sRepr.eContinuity = ContinuityFromByte(rIn.GetByte());
                sRepr.iSurface = rIn.GetInteger();
                sRepr.iLocation = rIn.GetInteger();
                sRepr.iSurface2 = rIn.GetInteger();
                sRepr.iLocation2 = rIn.GetInteger();
                rE.sReprs.push_back(sRepr);
                break;
            case 5: // Polygon3D
                sRepr.iPolygon3d = rIn.GetInteger();
                sRepr.iLocation = rIn.GetInteger();
                rE.sReprs.push_back(sRepr);
                break;
            case 6: // Polygon on triangulation
            case 7: // Polygon on closed triangulation
            {
                const bool bClosed = (iVal == 7);
                sRepr.iPolyOnTri = rIn.GetInteger();
                if (bClosed)
                {
                    sRepr.iPolyOnTri2 = rIn.GetInteger();
                }
                sRepr.iTriangulation = rIn.GetInteger();
                sRepr.iLocation = rIn.GetInteger();
                rE.sReprs.push_back(sRepr);
                break;
            }
            default:
                Fail("unexpected edge curve representation " + std::to_string(iVal));
        }
    } while (iVal > 0);
}

void ReadFaceGeometry(CBinReader& rIn, SFaceData& rF)
{
    rF.bNaturalRestriction = rIn.GetBool();
    rF.dTolerance = rIn.GetReal();
    rF.iSurface = rIn.GetInteger();
    rF.iLocation = rIn.GetInteger();
    const int iByte = rIn.GetByte();
    if (iByte == 2)
    {
        rF.iTriangulation = rIn.GetInteger();
    }
}

// Reads one sub-shape reference. Returns false when the null terminator '*' is read.
bool ReadSubShapeRef(CBinReader& rIn, int iNbShapes, SSubShapeRef& rRef)
{
    const uint8_t uChar = rIn.ReadCharSkipWs();
    if (uChar == '*')
    {
        return false;
    }
    switch (uChar)
    {
        case 0:
            rRef.eOrientation = EOrientation::Forward;
            break;
        case 1:
            rRef.eOrientation = EOrientation::Reversed;
            break;
        case 2:
            rRef.eOrientation = EOrientation::Internal;
            break;
        case 3:
            rRef.eOrientation = EOrientation::External;
            break;
        default:
            Fail("malformed sub-shape orientation byte " + std::to_string(uChar));
    }
    rRef.iFileToken = rIn.GetInteger();
    rRef.iStorageIndex = iNbShapes - rRef.iFileToken + 1;
    rRef.iLocation = rIn.GetInteger();
    return true;
}

void ReadShapes(CBinReader& rIn, SOcctModel& rModel)
{
    const int iNbShapes = ReadSectionHeader(rIn, "TShapes");
    rModel.sShapes.reserve(static_cast<size_t>(iNbShapes));
    for (int ii = 0; ii < iNbShapes; ++ii)
    {
        SShapeRecord sRec;
        sRec.eType = ShapeEnumFromByte(rIn.GetByte());
        switch (sRec.eType)
        {
            case EShapeType::Vertex:
                ReadVertexGeometry(rIn, sRec.sVertex, rModel.iFormatVersion);
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

        // Flags: free, modified, checked, orientable, closed, infinite, convex.
        sRec.bFree = rIn.GetBool();
        sRec.bModified = rIn.GetBool();
        sRec.bChecked = rIn.GetBool();
        sRec.bOrientable = rIn.GetBool();
        sRec.bClosed = rIn.GetBool();
        sRec.bInfinite = rIn.GetBool();
        sRec.bConvex = rIn.GetBool();

        SSubShapeRef sRef;
        while (ReadSubShapeRef(rIn, iNbShapes, sRef))
        {
            sRec.sSubShapes.push_back(sRef);
            sRef = SSubShapeRef{};
        }

        rModel.sShapes.push_back(std::move(sRec));
    }
}

} // namespace

bool IsBinaryOcct(const std::string& rData)
{
    // The binary banners all begin "Open CASCADE Topology V"; the ASCII banners begin "CASCADE
    // Topology V". Scan the first handful of lines for an exact binary banner match.
    CBinReader sIn(rData);
    std::string sLine;
    for (int iLine = 0; iLine < 8 && sIn.GetLine(sLine); ++iLine)
    {
        for (int iV = THE_BIN_VERSION_LOWER; iV <= THE_BIN_VERSION_UPPER; ++iV)
        {
            if (sLine == THE_BIN_VERSION_BANNERS[iV])
            {
                return true;
            }
        }
    }
    return false;
}

bool ReadOcctBinary(const std::string& rData, SOcctModel& rModel, std::string& rError)
{
    rModel = SOcctModel{};
    CBinReader sIn(rData);
    try
    {
        // Version banner: read lines until one matches a known binary banner (skips the leading
        // blank line BinTools writes before the banner).
        bool bFound = false;
        std::string sLine;
        while (sIn.GetLine(sLine))
        {
            for (int iV = THE_BIN_VERSION_LOWER; iV <= THE_BIN_VERSION_UPPER; ++iV)
            {
                if (sLine == THE_BIN_VERSION_BANNERS[iV])
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
            rError = "no recognised binary 'Open CASCADE Topology' version banner";
            return false;
        }

        ReadLocations(sIn, rModel);
        // Geometry section order is fixed by BinTools_ShapeSet::ReadGeometry.
        ReadCurve2ds(sIn, rModel);
        ReadCurves(sIn, rModel);
        ReadPolygon3D(sIn, rModel);
        ReadPolygonOnTriangulations(sIn, rModel);
        ReadSurfaces(sIn, rModel);
        ReadTriangulations(sIn, rModel);
        ReadShapes(sIn, rModel);

        // Trailing root shape reference (BinTools::Read consumes one via ReadSubs).
        SSubShapeRef sRoot;
        if (ReadSubShapeRef(sIn, static_cast<int>(rModel.sShapes.size()), sRoot))
        {
            rModel.sRoot = sRoot;
            rModel.bHasRoot = true;
        }
    }
    catch (const SBinParseError& rErr)
    {
        rError = rErr.sMessage;
        return false;
    }
    return true;
}

} // namespace occt
