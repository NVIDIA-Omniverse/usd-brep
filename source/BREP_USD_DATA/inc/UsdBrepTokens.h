// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepTokens.h
 * PURPOSE: Header file for Tokens used with BREP_USD_DATA prims
 *
 * NOTES: 1. defines UsdBrepSolidTokens set
 *        2. Access UsdBrepSolidTokens tokens using pointer syntax on the key as
 *              UsdBrepSolidTokens->wireEdge;
 *        3. An additional member, allTokens, is a std::vector<TfToken> populated
 *           with all of the generated token members.
 * ******************************************************************************************************************/

#ifndef _USD_BREP_TOKENS_H_
#define _USD_BREP_TOKENS_H_

#include "UsdBrepConfig.h"

// USD includes
#include "UsdBrepHeaders.h"

PXR_NAMESPACE_OPEN_SCOPE

#ifndef USDBREP_TOKENS

// clang-format off

// token "source" should not be duplicated in the following.
// fix uses of it in code and then remove it.

  #define USDBREP_TOKENS                                                                \
    ((source,                                 "source"))                            \
    ((brepPointAPI,                        "BrepPointAPI"))                         \
       ((vertexPoint,                         "vertexPoint"))                       \
       ((shellPoint,                          "shellPoint"))                        \
       ((brepPointPosition,                   "brep:__INSTANCE_NAME__:point:position")) \
       ((brepIntersectTol3d,                  "brep:intersectTol3d"))         \
       ((brepExtent,                          "brep:extent"))                 \
       ((brepRegionCount,                     "brep:regionCount"))            \
       ((regionShellCount,                    "region:shellCount"))           \
       ((regionType,                          "region:type"))                 \
          ((solidRegion,                         "solidRegion"))              \
          ((voidRegion,                          "voidRegion"))               \
       ((shellFaceuseCount,                   "shell:faceuseCount"))          \
       ((shellWireEdgeCount,                  "shell:wireEdgeCount"))         \
       ((shellPointType,                      "shell:pointType"))             \
          ((none,                                "none"))                     \
    ((faceuseFaceIndex,                    "faceuse:faceIndex"))              \
       ((faceuseOrientationType,              "faceuse:orientationType"))     \
          ((same,                                "same"))                     \
          ((opposite,                            "opposite"))                 \
       ((faceLoopCount,                       "face:loopCount"))              \
       ((faceSurfaceType,                     "face:surfaceType"))            \
       ((faceTrimType,                        "face:trimType"))               \
          ((rectangular,                         "rectangular"))              \
          ((general,                             "general"))                  \
       ((faceRange,                           "face:range"))                  \
       ((loopEdgeuseCount,                    "loop:edgeuseCount"))           \
       ((loopVertexIndex,                     "loop:vertexIndex"))            \
    ((edgeuseEdgeIndex,                    "edgeuse:edgeIndex"))              \
       ((edgeuseOrientationType,              "edgeuse:orientationType"))     \
       ((edgeuseNextRadialEUIndex,            "edgeuse:nextRadialEUIndex"))   \
       ((edgeuseThisRadialEntryType,          "edgeuse:thisRadialEntryType")) \
          ((topEntry,                            "topEntry"))                 \
          ((bottomEntry,                         "bottomEntry"))              \
       ((edgeCurveType,                       "edge:curveType"))              \
       ((edgeRange,                           "edge:range"))                  \
       ((edgeVertexIndices,                   "edge:vertexIndices"))          \
       ((wireEdgeCurveType,                   "wireEdge:curveType"))          \
       ((wireEdgeRange,                       "wireEdge:range"))              \
       ((wireEdgeVertexIndices,               "wireEdge:vertexIndices"))      \
       ((vertexPointType,                     "vertex:pointType"))            \
    ((brep,                                "brep"))                           \
    ((face,                                "face"))

// clang-format on

#endif // no USDBREP_TOKENS

#ifndef USDBREP_CURVE_TOKENS

// clang-format off

  #define USDBREP_CURVE_TOKENS                                                            \
    ((brepCurve3dNurbAPI,                  "BrepCurve3dNurbAPI"))                   \
       ((edge3dNurb,                          "edge3dNurb"))                        \
       ((wireEdge3dNurb,                      "wireEdge3dNurb"))                    \
       ((brep3dNurbControlVertices,           "brep:__INSTANCE_NAME__:curve3d:nurb:controlVertices")) \
       ((brep3dNurbVertexCount,               "brep:__INSTANCE_NAME__:curve3d:nurb:vertexCount"))     \
       ((brep3dNurbOrder,                     "brep:__INSTANCE_NAME__:curve3d:nurb:order"))           \
       ((brep3dNurbKnots,                     "brep:__INSTANCE_NAME__:curve3d:nurb:knots"))           \
       ((brep3dNurbWeights,                   "brep:__INSTANCE_NAME__:curve3d:nurb:weights"))         \
    ((brepCurve3dCircleAPI,                "BrepCurve3dCircleAPI"))                 \
       ((edge3dCircle,                        "edge3dCircle"))                      \
       ((wireEdge3dCircle,                    "wireEdge3dCircle"))                  \
       ((brep3dCircleCenter,                  "brep:__INSTANCE_NAME__:curve3d:circle:center"))        \
       ((brep3dCircleAxis,                    "brep:__INSTANCE_NAME__:curve3d:circle:axis"))          \
       ((brep3dCircleRefDirection,            "brep:__INSTANCE_NAME__:curve3d:circle:refDirection"))  \
       ((brep3dCircleRadius,                  "brep:__INSTANCE_NAME__:curve3d:circle:radius"))        \
    ((brepCurve3dLineAPI,                  "BrepCurve3dLineAPI"))                   \
       ((edge3dLine,                          "edge3dLine"))                        \
       ((wireEdge3dLine,                      "wireEdge3dLine"))                    \
       ((brep3dLineOrigin,                    "brep:__INSTANCE_NAME__:curve3d:line:origin"))          \
       ((brep3dLineDirection,                 "brep:__INSTANCE_NAME__:curve3d:line:direction"))       \
    ((brepCurve3dEllipseAPI,               "BrepCurve3dEllipseAPI"))                \
       ((edge3dEllipse,                       "edge3dEllipse"))                     \
       ((wireEdge3dEllipse,                   "wireEdge3dEllipse"))                 \
       ((brep3dEllipseCenter,                 "brep:__INSTANCE_NAME__:curve3d:ellipse:center"))            \
       ((brep3dEllipseAxis,                   "brep:__INSTANCE_NAME__:curve3d:ellipse:axis"))              \
       ((brep3dEllipseRefDirection,           "brep:__INSTANCE_NAME__:curve3d:ellipse:refDirection"))      \
       ((brep3dEllipseXRadius,             "brep:__INSTANCE_NAME__:curve3d:ellipse:xRadius"))   \
       ((brep3dEllipseYRadius,             "brep:__INSTANCE_NAME__:curve3d:ellipse:yRadius"))   \
    ((brepCurveUvNurbAPI,                  "BrepCurveUvNurbAPI"))                   \
       ((brepCurveUvNurbControlVertices,      "brep:curveUv:nurb:controlVertices")) \
       ((brepCurveUvNurbVertexCount,          "brep:curveUv:nurb:vertexCount"))     \
       ((brepCurveUvNurbOrder,                "brep:curveUv:nurb:order"))           \
       ((brepCurveUvNurbKnots,                "brep:curveUv:nurb:knots"))           \
       ((brepCurveUvNurbWeights,              "brep:curveUv:nurb:weights"))

// clang-format on

#endif // no USDBREP_CURVE_TOKENS

#ifndef USDBREP_SURFACE_TOKENS

// clang-format off

  #define USDBREP_SURFACE_TOKENS                                                          \
    ((brepSurfaceNurbAPI,                  "BrepSurfaceNurbAPI"))                   \
       ((brepSurfaceNurbControlVertices,      "brep:surface:nurb:controlVertices")) \
       ((brepSurfaceNurbUVertexCount,         "brep:surface:nurb:uVertexCount"))    \
       ((brepSurfaceNurbVVertexCount,         "brep:surface:nurb:vVertexCount"))    \
       ((brepSurfaceNurbUOrder,               "brep:surface:nurb:uOrder"))          \
       ((brepSurfaceNurbUKnots,               "brep:surface:nurb:uKnots"))          \
       ((brepSurfaceNurbVOrder,               "brep:surface:nurb:vOrder"))          \
       ((brepSurfaceNurbVKnots,               "brep:surface:nurb:vKnots"))          \
       ((brepSurfaceNurbWeights,              "brep:surface:nurb:weights"))         \
    ((brepSurfaceSphereAPI,                "BrepSurfaceSphereAPI"))                 \
       ((brepSurfaceSphereCenter,             "brep:surface:sphere:center"))        \
       ((brepSurfaceSphereAxis,               "brep:surface:sphere:axis"))          \
       ((brepSurfaceSphereRefDirection,       "brep:surface:sphere:refDirection"))  \
       ((brepSurfaceSphereRadius,             "brep:surface:sphere:radius"))        \
    ((brepSurfacePlaneAPI,                 "BrepSurfacePlaneAPI"))                  \
       ((brepSurfacePlaneOrigin,              "brep:surface:plane:origin"))         \
       ((brepSurfacePlaneAxis,                "brep:surface:plane:axis"))           \
       ((brepSurfacePlaneRefDirection,        "brep:surface:plane:refDirection"))   \
    ((brepSurfaceCylinderAPI,              "BrepSurfaceCylinderAPI"))               \
       ((brepSurfaceCylinderOrigin,           "brep:surface:cylinder:origin"))      \
       ((brepSurfaceCylinderAxis,             "brep:surface:cylinder:axis"))        \
       ((brepSurfaceCylinderRefDirection,     "brep:surface:cylinder:refDirection"))\
       ((brepSurfaceCylinderRadius,           "brep:surface:cylinder:radius"))      \
    ((brepSurfaceConeAPI,                  "BrepSurfaceConeAPI"))                   \
       ((brepSurfaceConeOrigin,               "brep:surface:cone:origin"))          \
       ((brepSurfaceConeAxis,                 "brep:surface:cone:axis"))            \
       ((brepSurfaceConeRefDirection,         "brep:surface:cone:refDirection"))    \
       ((brepSurfaceConeRadius,               "brep:surface:cone:radius"))          \
       ((brepSurfaceConeSemiAngle,            "brep:surface:cone:semiAngle"))       \
    ((brepSurfaceTorusAPI,                 "BrepSurfaceTorusAPI"))                  \
       ((brepSurfaceTorusOrigin,              "brep:surface:torus:origin"))         \
       ((brepSurfaceTorusAxis,                "brep:surface:torus:axis"))           \
       ((brepSurfaceTorusRefDirection,        "brep:surface:torus:refDirection"))   \
       ((brepSurfaceTorusMajorRadius,         "brep:surface:torus:majorRadius"))    \
       ((brepSurfaceTorusMinorRadius,         "brep:surface:torus:minorRadius"))

// clang-format on

#endif // no USDBREP_SURFACE_TOKENS

// TF_DECLARE_PUBLIC_TOKENS three argument version exports declared tokens from a DLL on windows
// needed if any project other than omni_solid_brep_data will be using these tokens
TF_DECLARE_PUBLIC_TOKENS(UsdBrepSolidTokens, USDBREP_EXPORT, USDBREP_TOKENS);
TF_DECLARE_PUBLIC_TOKENS(UsdBrepCurveTokens, USDBREP_EXPORT, USDBREP_CURVE_TOKENS);
TF_DECLARE_PUBLIC_TOKENS(UsdBrepSurfaceTokens, USDBREP_EXPORT, USDBREP_SURFACE_TOKENS);

PXR_NAMESPACE_CLOSE_SCOPE

#endif // no _USD_BREP_TOKENS_H_
