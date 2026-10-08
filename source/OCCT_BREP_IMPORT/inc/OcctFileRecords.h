// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef OCCT_BREP_IMPORT_OCCT_FILE_RECORDS_H
#define OCCT_BREP_IMPORT_OCCT_FILE_RECORDS_H

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace occt
{

// Geometry type tags use the same integer codes OCCT writes into the file.
enum class ECurveType
{
    Unknown = 0,
    Line = 1,
    Circle = 2,
    Ellipse = 3,
    Parabola = 4,
    Hyperbola = 5,
    Bezier = 6,
    BSpline = 7,
    Trimmed = 8,
    Offset = 9,
};

enum class ESurfaceType
{
    Unknown = 0,
    Plane = 1,
    Cylinder = 2,
    Cone = 3,
    Sphere = 4,
    Torus = 5,
    LinearExtrusion = 6,
    Revolution = 7,
    Bezier = 8,
    BSpline = 9,
    RectangularTrimmed = 10,
    Offset = 11,
};

enum class EShapeType
{
    Unknown = 0,
    Vertex,
    Edge,
    Wire,
    Face,
    Shell,
    Solid,
    CompSolid,
    Compound,
};

enum class EOrientation
{
    Forward,
    Reversed,
    Internal,
    External,
};

enum class EContinuity
{
    C0,
    G1,
    C1,
    G2,
    C2,
    C3,
    CN,
};

struct SPnt3
{
    double dX = 0.0, dY = 0.0, dZ = 0.0;
};
struct SDir3
{
    double dX = 0.0, dY = 0.0, dZ = 0.0;
};
struct SPnt2
{
    double dX = 0.0, dY = 0.0;
};
struct SDir2
{
    double dX = 0.0, dY = 0.0;
};

// 3D curve record. Recursive for Trimmed/Offset (which wrap a basis curve).
struct SCurve3d
{
    ECurveType eType = ECurveType::Unknown;
    // Analytic (Line/Circle/Ellipse/Parabola/Hyperbola). sLocation is origin/center.
    SPnt3 sLocation;
    SDir3 sAxis; // 3D conic main axis (normal); unused by Line
    SDir3 sXAxis; // Line direction lives here too
    SDir3 sYAxis;
    double dRadius = 0.0; // circle radius / conic major radius
    double dMinorRadius = 0.0; // ellipse/hyperbola minor radius
    double dFocal = 0.0; // parabola focal
    // Bezier / BSpline.
    bool bRational = false;
    bool bPeriodic = false;
    int iDegree = 0;
    std::vector<SPnt3> sPoles;
    std::vector<double> dWeights;
    std::vector<double> dKnots;
    std::vector<int> iMults;
    // Trimmed.
    double dFirst = 0.0, dLast = 0.0;
    // Offset.
    double dOffset = 0.0;
    SDir3 sOffsetDir;
    // Basis for Trimmed/Offset.
    std::shared_ptr<SCurve3d> pBasis;
};

// 2D (parametric / pcurve) record. Recursive for Trimmed/Offset.
struct SCurve2d
{
    ECurveType eType = ECurveType::Unknown;
    SPnt2 sLocation;
    SDir2 sXAxis; // Line direction lives here too
    SDir2 sYAxis;
    double dRadius = 0.0;
    double dMinorRadius = 0.0;
    double dFocal = 0.0;
    bool bRational = false;
    bool bPeriodic = false;
    int iDegree = 0;
    std::vector<SPnt2> sPoles;
    std::vector<double> dWeights;
    std::vector<double> dKnots;
    std::vector<int> iMults;
    double dFirst = 0.0, dLast = 0.0;
    double dOffset = 0.0; // 2D offset has no direction
    std::shared_ptr<SCurve2d> pBasis;
};

struct SSurface
{
    ESurfaceType eType = ESurfaceType::Unknown;
    // Ax3 placement (Plane/Cylinder/Cone/Sphere/Torus).
    SPnt3 sLocation;
    SDir3 sAxis;
    SDir3 sXAxis;
    SDir3 sYAxis;
    double dRadius = 0.0; // cyl/sphere radius, cone ref radius, torus major radius
    double dMinorRadius = 0.0; // torus minor radius
    double dSemiAngle = 0.0; // cone half-angle
    // Extrusion / Revolution.
    SDir3 sDirection;
    std::shared_ptr<SCurve3d> pBasisCurve;
    // Bezier / BSpline (poles row-major: u outer, v inner).
    bool bURational = false, bVRational = false;
    bool bUPeriodic = false, bVPeriodic = false;
    int iUDegree = 0, iVDegree = 0;
    int iNbUPoles = 0, iNbVPoles = 0;
    std::vector<SPnt3> sPoles;
    std::vector<double> dWeights;
    std::vector<double> dUKnots, dVKnots;
    std::vector<int> iUMults, iVMults;
    // RectangularTrimmed.
    double dU1 = 0.0, dU2 = 0.0, dV1 = 0.0, dV2 = 0.0;
    // Offset.
    double dOffset = 0.0;
    // Basis for RectangularTrimmed/Offset.
    std::shared_ptr<SSurface> pBasisSurface;
};

struct SLocationTerm
{
    int iLocationIndex = 0; // 1-based index into SOcctModel::sLocations
    int iPower = 1;
};

struct SLocation
{
    bool bElementary = true;
    double dMatrix[3][4] = {}; // 3x4 affine (valid if elementary)
    std::vector<SLocationTerm> sTerms; // valid if !bElementary
};

// ---- Topology (TShapes graph) ----

struct SPointRepr
{
    int iKind = 0; // 1=on curve, 2=on curve-on-surface, 3=on surface
    double dParam = 0.0;
    double dParam2 = 0.0; // kind 3
    int iCurve = 0;
    int iPCurve = 0;
    int iSurface = 0;
    int iLocation = 0;
};

struct SVertexData
{
    double dTolerance = 0.0;
    SPnt3 sPoint;
    std::vector<SPointRepr> sReprs;
};

struct SCurveRepr
{
    int iKind = 0; // 1=curve3d, 2=curve-on-surf, 3=curve-on-closed-surf, 4=regularity, 5/6/7=poly
    int iCurve3d = 0;
    int iPCurve = 0;
    int iPCurve2 = 0;
    int iSurface = 0;
    int iSurface2 = 0;
    int iLocation = 0;
    int iLocation2 = 0;
    int iPolygon3d = 0;
    int iPolyOnTri = 0;
    int iPolyOnTri2 = 0;
    int iTriangulation = 0;
    EContinuity eContinuity = EContinuity::C0;
    double dFirst = 0.0, dLast = 0.0;
};

struct SEdgeData
{
    double dTolerance = 0.0;
    bool bSameParameter = false;
    bool bSameRange = false;
    bool bDegenerated = false;
    std::vector<SCurveRepr> sReprs;
};

struct SFaceData
{
    bool bNaturalRestriction = false;
    double dTolerance = 0.0;
    int iSurface = 0;
    int iLocation = 0;
    int iTriangulation = 0;
};

struct SSubShapeRef
{
    EOrientation eOrientation = EOrientation::Forward;
    int iFileToken = 0; // index exactly as written (counts from end of table)
    int iStorageIndex = 0; // resolved 1-based index in file/storage order
    int iLocation = 0;
};

struct SShapeRecord
{
    EShapeType eType = EShapeType::Unknown;
    SVertexData sVertex; // valid if eType==Vertex
    SEdgeData sEdge; // valid if eType==Edge
    SFaceData sFace; // valid if eType==Face
    bool bFree = false;
    bool bModified = false;
    bool bChecked = false;
    bool bOrientable = false;
    bool bClosed = false;
    bool bInfinite = false;
    bool bConvex = false;
    std::vector<SSubShapeRef> sSubShapes;
};

struct SPolygon3d
{
    double dDeflection = 0.0;
    std::vector<SPnt3> sNodes;
    std::vector<double> dParams; // empty if none
};

struct SPolygonOnTriangulation
{
    double dDeflection = 0.0;
    std::vector<int> iNodes;
    std::vector<double> dParams; // empty if none
};

struct STriangulation
{
    int iNbNodes = 0;
    int iNbTriangles = 0;
    bool bHasUV = false;
    bool bHasNormals = false;
    double dDeflection = 0.0;
    std::vector<SPnt3> sNodes;
    std::vector<SPnt2> sUVNodes;
    std::vector<std::array<int, 3>> iTriangles;
    std::vector<SDir3> sNormals;
};

struct SOcctModel
{
    int iFormatVersion = 0; // 1, 2 or 3
    std::vector<SLocation> sLocations; // [i] holds 1-based location i+1
    std::vector<SCurve2d> sCurve2ds;
    std::vector<SCurve3d> sCurves;
    std::vector<SPolygon3d> sPolygons3d;
    std::vector<SPolygonOnTriangulation> sPolygonsOnTri;
    std::vector<SSurface> sSurfaces;
    std::vector<STriangulation> sTriangulations;
    std::vector<SShapeRecord> sShapes; // file/storage order: [i] holds shape i+1
    SSubShapeRef sRoot;
    bool bHasRoot = false;
};

} // namespace occt

#endif // OCCT_BREP_IMPORT_OCCT_FILE_RECORDS_H
