// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#include "OcctAsciiParser.h"
#include "OcctBinaryParser.h"

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace occt;

namespace
{

template <typename TEnum, typename TNameFn>
void PrintHistogram(const std::string& rLabel, const std::map<TEnum, int>& rHist, TNameFn pName)
{
    std::cout << "    " << rLabel << " types:";
    for (const auto& rEntry : rHist)
    {
        std::cout << " " << pName(rEntry.first) << "=" << rEntry.second;
    }
    std::cout << "\n";
}

// Count occurrences of a geometry/topology type tag across a record vector.
template <typename TRecord, typename TEnum>
std::map<TEnum, int> Histogram(const std::vector<TRecord>& rRecords, TEnum TRecord::*pField)
{
    std::map<TEnum, int> sHist;
    for (const auto& rRec : rRecords)
    {
        sHist[rRec.*pField]++;
    }
    return sHist;
}

// Validate that a 1-based index into a table is either 0 (none) or in [1, size].
bool CheckRef(int iIndex, size_t uiSize, const char* pWhat, std::vector<std::string>& rErrors)
{
    if (iIndex < 0 || static_cast<size_t>(iIndex) > uiSize)
    {
        std::ostringstream sErr;
        sErr << pWhat << " index " << iIndex << " out of range [0, " << uiSize << "]";
        rErrors.push_back(sErr.str());
        return false;
    }
    return true;
}

void ValidateModel(const SOcctModel& rModel, std::vector<std::string>& rErrors)
{
    const size_t uiNbLoc = rModel.sLocations.size();
    const size_t uiNbC2d = rModel.sCurve2ds.size();
    const size_t uiNbC3d = rModel.sCurves.size();
    const size_t uiNbSurf = rModel.sSurfaces.size();
    const size_t uiNbTri = rModel.sTriangulations.size();
    const size_t uiNbPolyTri = rModel.sPolygonsOnTri.size();
    const size_t uiNbPoly3d = rModel.sPolygons3d.size();
    const size_t uiNbShapes = rModel.sShapes.size();

    // Composite locations must reference earlier elementary locations.
    for (const auto& rLoc : rModel.sLocations)
    {
        for (const auto& rTerm : rLoc.sTerms)
        {
            CheckRef(rTerm.iLocationIndex, uiNbLoc, "location term", rErrors);
        }
    }

    for (const auto& rRec : rModel.sShapes)
    {
        switch (rRec.eType)
        {
            case EShapeType::Vertex:
                for (const auto& rPr : rRec.sVertex.sReprs)
                {
                    CheckRef(rPr.iCurve, uiNbC3d, "vertex curve", rErrors);
                    CheckRef(rPr.iPCurve, uiNbC2d, "vertex pcurve", rErrors);
                    CheckRef(rPr.iSurface, uiNbSurf, "vertex surface", rErrors);
                    CheckRef(rPr.iLocation, uiNbLoc, "vertex location", rErrors);
                }
                break;
            case EShapeType::Edge:
                for (const auto& rCr : rRec.sEdge.sReprs)
                {
                    CheckRef(rCr.iCurve3d, uiNbC3d, "edge curve3d", rErrors);
                    CheckRef(rCr.iPCurve, uiNbC2d, "edge pcurve", rErrors);
                    CheckRef(rCr.iPCurve2, uiNbC2d, "edge pcurve2", rErrors);
                    CheckRef(rCr.iSurface, uiNbSurf, "edge surface", rErrors);
                    CheckRef(rCr.iSurface2, uiNbSurf, "edge surface2", rErrors);
                    CheckRef(rCr.iLocation, uiNbLoc, "edge location", rErrors);
                    CheckRef(rCr.iLocation2, uiNbLoc, "edge location2", rErrors);
                    CheckRef(rCr.iPolygon3d, uiNbPoly3d, "edge polygon3d", rErrors);
                    CheckRef(rCr.iPolyOnTri, uiNbPolyTri, "edge polyOnTri", rErrors);
                    CheckRef(rCr.iPolyOnTri2, uiNbPolyTri, "edge polyOnTri2", rErrors);
                    CheckRef(rCr.iTriangulation, uiNbTri, "edge triangulation", rErrors);
                }
                break;
            case EShapeType::Face:
                CheckRef(rRec.sFace.iSurface, uiNbSurf, "face surface", rErrors);
                CheckRef(rRec.sFace.iLocation, uiNbLoc, "face location", rErrors);
                CheckRef(rRec.sFace.iTriangulation, uiNbTri, "face triangulation", rErrors);
                break;
            default:
                break;
        }
        for (const auto& rRef : rRec.sSubShapes)
        {
            if (rRef.iStorageIndex < 1 || static_cast<size_t>(rRef.iStorageIndex) > uiNbShapes)
            {
                std::ostringstream sErr;
                sErr << "sub-shape storage index " << rRef.iStorageIndex << " (token " << rRef.iFileToken << ") out of range [1, " << uiNbShapes
                     << "]";
                rErrors.push_back(sErr.str());
            }
            CheckRef(rRef.iLocation, uiNbLoc, "sub-shape location", rErrors);
        }
    }
}

bool ProcessFile(const std::string& rPath)
{
    std::ifstream sFile(rPath, std::ios::binary);
    if (!sFile)
    {
        std::cerr << "ERROR: cannot open " << rPath << "\n";
        return false;
    }
    // Bound the buffered input so a pathological/corrupt file cannot OOM the tool.
    constexpr std::streamoff kMaxBrepBytes = 256ll * 1024ll * 1024ll;
    sFile.seekg(0, std::ios::end);
    const std::streamoff iSize = sFile.tellg();
    if (iSize < 0 || iSize > kMaxBrepBytes)
    {
        std::cerr << "ERROR: input file too large or unreadable: " << rPath << "\n";
        return false;
    }
    sFile.seekg(0, std::ios::beg);
    std::string sText(static_cast<size_t>(iSize), '\0');
    if (iSize > 0 && !sFile.read(&sText[0], static_cast<std::streamsize>(iSize)))
    {
        std::cerr << "ERROR: failed to read " << rPath << "\n";
        return false;
    }

    SOcctModel sModel;
    std::string sError;
    std::cout << "==== " << rPath << " ====\n";
    const bool bBinary = IsBinaryOcct(sText);
    const bool bParsed = bBinary ? ReadOcctBinary(sText, sModel, sError) : ReadOcct(sText, sModel, sError);
    if (!bParsed)
    {
        std::cout << "  PARSE FAILED (" << (bBinary ? "binary" : "ascii") << "): " << sError << "\n\n";
        return false;
    }
    std::cout << "  encoding: " << (bBinary ? "binary" : "ascii") << "\n";

    std::cout << "  format V" << sModel.iFormatVersion << "\n";
    std::cout << "  Locations: " << sModel.sLocations.size() << "\n";
    std::cout << "  Curve2ds:  " << sModel.sCurve2ds.size() << "\n";
    PrintHistogram("Curve2ds", Histogram(sModel.sCurve2ds, &SCurve2d::eType), CurveTypeName);
    std::cout << "  Curves:    " << sModel.sCurves.size() << "\n";
    PrintHistogram("Curves", Histogram(sModel.sCurves, &SCurve3d::eType), CurveTypeName);
    std::cout << "  Surfaces:  " << sModel.sSurfaces.size() << "\n";
    PrintHistogram("Surfaces", Histogram(sModel.sSurfaces, &SSurface::eType), SurfaceTypeName);
    std::cout << "  Polygon3D: " << sModel.sPolygons3d.size() << "  PolygonOnTri: " << sModel.sPolygonsOnTri.size()
              << "  Triangulations: " << sModel.sTriangulations.size() << "\n";

    std::map<EShapeType, int> sShapeHist = Histogram(sModel.sShapes, &SShapeRecord::eType);
    int iSubRefs = 0;
    for (const auto& rRec : sModel.sShapes)
    {
        iSubRefs += static_cast<int>(rRec.sSubShapes.size());
    }
    std::cout << "  TShapes:   " << sModel.sShapes.size() << " (" << iSubRefs << " sub-shape refs)\n";
    PrintHistogram("TShape", sShapeHist, ShapeTypeName);
    if (sModel.bHasRoot)
    {
        std::cout << "  root -> storage #" << sModel.sRoot.iStorageIndex << "\n";
    }
    else
    {
        std::cout << "  root -> (none)\n";
    }

    std::vector<std::string> sErrors;
    ValidateModel(sModel, sErrors);
    if (sErrors.empty())
    {
        std::cout << "  validate: OK\n";
    }
    else
    {
        std::cout << "  validate: " << sErrors.size() << " issue(s)\n";
        int iShown = 0;
        for (const auto& rErr : sErrors)
        {
            std::cout << "    - " << rErr << "\n";
            if (++iShown >= 10)
            {
                std::cout << "    ... (" << (sErrors.size() - 10) << " more)\n";
                break;
            }
        }
    }
    std::cout << "\n";
    return sErrors.empty();
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "usage: occt_dump <file.brep> [more.brep ...]\n";
        return 2;
    }
    int iFailures = 0;
    for (int iArg = 1; iArg < argc; ++iArg)
    {
        if (!ProcessFile(argv[iArg]))
        {
            ++iFailures;
        }
    }
    std::cout << "==== summary: " << (argc - 1 - iFailures) << "/" << (argc - 1) << " files parsed+validated cleanly ====\n";
    return iFailures == 0 ? 0 : 1;
}
