// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*********************************************************************************************************************
 * FILE NAME --- UsdBrepArrayData.h
 * PURPOSE: UsdBrepArrayData, UsdBrepArraySpans header file for UsdBrepArray translations
 *
 * NOTES: mirrors the omniSolid USD BrepArray schema
 *
 * CONTAINS:
 *  namespace: UsdBrepData{ // C++ classes for translation
 *                           UsdBrepArrayData
 *                           UsdBrepArraySpans
 *
 *                           // simple data functions
 *                           GetUsdBrepArray_BrepCount()
 *                         }
 * ******************************************************************************************************************/

#ifndef _USD_BREP_ARRAY_DATA_H_
#define _USD_BREP_ARRAY_DATA_H_

#include "UsdBrepConfig.h"

// USD includes
#include "UsdBrepHeaders.h"

// Math constants
inline constexpr unsigned long USDBREP_NO_OBJECT_INDEX = 9999999UL; // LoopVertexIndex val when Loops are edgeuse loops

namespace UsdBrepData
{
PXR_NAMESPACE_USING_DIRECTIVE

// utility
USDBREP_EXPORT uint32_t GetUsdBrepArray_BrepCount(const pxr::UsdPrim& crUsdBrepArray);

/*********************************************************************************************************************
 PURPOSE: a C++ class with a one-to-one representation of the
          data stored in the USD BrepArray class used to standardize and
          simplify writing translators that move CAD Brep data to and from USD format.

 NOTES: Brep Representation concepts, Translation concepts, USD BrepArray data summary.

  I: Brep representation concepts
    1. Brep shapes are represented by cyclic directed graphs in which the graph nodes
       are topology and geometry objects and the graph links are 'ConnectsTo' or 'HasA' relationships.
              For example: an edge connects to a pair of vertices is represented as
                           1 edge and 2 vertex nodes connected to each other with
                           2 EdgeToVertex links.
    2. Run time CAD Brep data is commonly implemented in an object oriented structure
       in which the topology and geometry objects are classes and links between objects
       are represented as object pointers stored as members within the topo and geom objs.
              For example: an edge connects to a pair of vertices is represented as
                EdgeObj.connectsPtr1 = address of VertexObj_1,
                EdgeObj.connectsPtr2 = address of VertexObj_2.
    3. USD scenes are sets of prims organized into a hierachical data structure like
       files in a directory tree. Prims are addressed by unique paths.  Connections
       between prims are represented as prim:property-data storing path-names to other prims.
        - minimizing scene prim counts improves performance.
    4. Rather than representing every Brep topology and geometry object as its own prim
       or even representing one-Brep-per-Prim, the BrepSchema has been designed to use
       1 prim to represent the data for a set of Breps. For example, a car model possibly
       consisting of 30,000 Breps can be represented by 1 BrepArray prim so that
       a scene with 100 cars can be represented with just 100 prims rather than 3 million prims.
    5. To make a one-Prim-per-SetOfBreps BrepArray storage scheme:
          - pointers between objects are replaced by a table-and-indexing scheme.
          - arrays of objects are replaced by sets of object->member associated-arrays.
              > object-member associated-arrays:
                  One array is created for every member type of every
                  topology and geometry object used in a Brep model. All the arrays are contained
                  in one top level Brep object that maps to a single USD Prim. The data from
                  each object in a set of Brep models gets loaded into a subset of these arrays
                  in a controlled order so that the data in the arrays can be read at a later
                  time to recreate the original topology and geometry object data.

                  These arrays are called associated because the data for all the members of a
                  single object distributed through the set of arrays is associated back to
                  the original object through common shared indices into those arrays.  For
                  example, it's possible to gather all the information for the 3rd edge
                  in the 4th Brep model knowing just those two index values.
  II: Translation concepts
     1. The UsdBrepArrayData class is part of supporting the following pipelines for CAD Brep data translation:

            import: ObjectOriented_SourceCADBrepSet -> FlatFormat -> AppendedArrayFormat -> Usd BrepArray prim
            export: Usd BrepArray prim -> AppendedArrayFormat -> FlatFormat -> ObjectOriented_SourceCADBrepSet

      a. ObjectOriented  : Brep structure  = a graph of topology and geometry C++ class objects
         SourceCADBrepSet: Obj connections = stored run-time object-pointer addresses.

      b. FlatFormat: Brep structure  = One Brep object with one object-array for every Brep::object type.
                     Obj Connections = indices into Brep's object-arrays stored as object members.
                     Translations to FlatFormats are commonly used to provide persistence to
                     Brep ObjectOriented models.

      c. UsdBrepArrayData: Brep structure  = One Brep object with 1 member-array for every Brep::object::member type.
                           Obj Connections = indices into Brep's member-arrays stored as index arrays in other
                                             Brep member-arrays.
                              * The set of member values making up one object are related to the object
                                by a common object-index value used to access data in the set of member-arrays
                                used by that object.
                              * Count values are added to the indexing scheme to support many-to-one
                                and one-to-many relationships. That makes the indexing scheme a bit
                                  complicated but that's just book keeping.

      d. UsdBrepArray: Brep Structure  = one BrepArray prim contains one attribute for every member-array.
                       Multiple Breps  = The data for multiple Breps is stacked into the member-arrays.
                                         That is the data for the 1st Brep is parsed into the member-arrays
                                         and then the data for subsequent Breps gets appended to those same arrays.
                       Obj Connections = indices into the attribute member-array are stored in separate
                                         attribute index-arrays. Data for a single object can be collected
                                         from the set of member-arrays using a common object-index values.
     2. The UsdBrepArrayData class is a one-to-one data represention of the USD BrepArray.

  III: USD BrepArray data summary
     1. primSpec data usage
        a) paths = "PrimPath.AttribTokenName" -> all attributes defined in the BrepArray schema

        b) paths = "PrimPath.AttribTokenName" -> all attributes defined in the BrepGeometry applied apis

        c) path  = "PrimPath.extent" -> attribute (token="extent", value= VtArray<GfVec3f>) = BBox (union of Breps)

        d) path  = "PrimPath.source" -> customData(token="source", value=<std::string>) = origin CAD string label

        e) Display Material Properties: - DisplayMaterials can be stored per BrepArray, per Brep, and per Brep->Face
                                        - Objects without DisplayMaterials inherit them as:
                                          Face      inheritsFrom Brep
                                          Brep      inheritsFrom BrepArray
                                          BrepArray inheritsFrom default
                                        - Display Materials stored in geomSubsets with a material path and a list of
                                           brep-or-face-indices that use that material.

          e.1) path = "PrimPath/subset_ii.elementType"     - attribute
                     (token:["elementType"], value:[token=oneof:["brep","face"]) // "brep" = indices
                                                                                 // "face" = indices
          e.2) path = "PrimPath/subset_ii.indices"        - attribute
                      (token:["indices"], value:[VtArray<uint32_t>])
              // brep or face indices to be rendered with this materialBinding

          e.3) path = "PrimPath/subset_ii.material:binding" - relationship(token:["materialBinding"])
              // material path to be used to render breps or faces in the indices list

IV: internal references, ordering, and indexing:
    - Region Topology in arrays:  // one entry per Region for all regions in all breps in BrepArray
                                  // order: in Brep blocks in order of appearance in m_sBrepRegionCountArray
                                  // sized: sum of counts in m_sBrepRegionCountArray
                                  // indexing: FirstRegion_inBrep_ii = Sum_jj(m_sRegionCountArray[jj])
                                  // for jj= 0 to ii-1
                                  // Number of Regions_inBrep_ii = m_sBrepRegionCountArray[ii]
                                  // referenced by counts in: m_sBrepRegionCountArray
        m_sRegionShellCountArray;
        m_sRegionShellIndexArray;

    - Shell Topology in arrays:   // one entry per Shell for all shells in all breps in BrepArray
                                  // order: in Region blocks in order of appearance in m_sRegionShellCountArray
                                  // sized: sum of counts in m_sBrepShellCountArray
                                  // indexing: FirstShell_inRegion_ii = Sum_jj(m_sRegionShellCountArray[jj])
                                  // for jj = 0 to ii-1
                                  // Number of Shells_inRegion_ii = m_sRegionShellCountArray[ii]
                                  // referenced by counts in: m_sRegionShellCountArray
        m_sShellFaceuseCountArray;
        m_sShellWireEdgeCountArray;
        m_sShellPointTypeArray;   with VertexShell geometry in:
            // one entry per VertexShell
            // in the order shells satisfy faceuseCount == 0, wireEdgeCount == 0, and
            // pointType == UsdBrepSolidTokens->brepPointAPI
            // sized: the number of shells satisfying all three point-shell conditions
            // indexing: VertexShellPoint_ii = count of shell indices jj in [0, ii)
            // satisfying all three conditions
            // Number of Vertices in VertexShell_ii = 1
            // referenced by the combined shell faceuse-count, wire-edge-count, and point-type discriminator
            // m_sShell_PointPositionArray

    - Faceuse Topology in arrays:
            // one entry per Faceuse for all faceuses in all breps in BrepArray
            // order: in Shell blocks in order of appearance in m_sShellFaceuseCountArray
            // sized: sum of counts in m_sShellFaceuseCountArray
            // indexing: FirstFaceuse_OfShell_ii = Sum_jj(m_sShellFaceuseCountArray[jj]) for jj= 0 to ii-1
            //           Number of Faceuses_inShell_ii = m_sShellFaceuseCountArray[ii] \
            // referenced by counts in: m_sShellFaceuseCountArray
        m_sFaceuseFaceIndexArray;
        m_sFaceuseOrientationTypeArray;

    - Face Topology in arrays:    // one entry per Face for all faces in all breps in BrepArray
                                  // order: not packed in an order defined by Faceuse entries
                                  // sized: sum of unique index values in m_sFaceuseFaceIndexArray
                                  // indexing: Face_OfFaceuse_ii = m_sFaceuseFaceIndexArray[ii]
                                  // Number of Faces_inFaceuse_ii = 1
                                  // referenced by indices in: m_sFaceuseFaceIndexArray
        m_sFaceLoopCountArray;
        m_sFaceSurfaceTypeArray;
        m_sFaceTrimTypeArray;
        m_sFaceRangeArray; with Face geometry in:
           // one surface block per Face whose m_sFaceSurfaceTypeArray[ii] == BrepSurfaceNurbAPI
           // for all faces in all breps in BrepArray
           // order: in order of appearance in m_sFaceSurfaceTypeArray when type == BrepSurfaceNurbAPI
           // sized: number of surface blocks =
           // sum of m_sFaceSurfaceTypeArray[ii] == UsdBrepSurfaceTokens->brepSurfaceNurbAPI values
           // indexing: SurfaceBlock_ofFace_ii =
           // Sum_jj(m_sFaceSurfaceTypeArray[jj] == UsdBrepSurfaceTokens->brepSurfaceNurbAPI)
           // for jj= 0 to ii-1
           // Number of SurfaceBlocks_inFace_ii = 1
           // referenced by counts in: m_sFaceSurfaceTypeArray[ii] when
           // m_sFaceSurfaceTypeArray[ii] == UsdBrepSurfaceTokens->brepSurfaceNurbAPI m_sFace_SurfaceNurb_UVertexCountArray;
           // sized: entry per Face for all faces in all breps in BrepArray m_sFace_SurfaceNurb_VVertexCountArray;
           // sized: entry per Face for all faces in all breps in BrepArray m_sFace_SurfaceNurb_UOrderArray;
           // sized: entry per Face for all faces in all breps in BrepArray
        m_sFace_SurfaceNurb_VOrderArray;          // sized: entry per Face for all faces in all breps in BrepArray
        m_sFace_SurfaceNurb_ControlVerticesArray; // sized: sum_ii(UvertexCount[ii] * VvertexCount[ii])
        m_sFace_SurfaceNurb_UKnotsArray;          // sized: sum_ii(UvertexCount[ii] + UOrder[ii])
        m_sFace_SurfaceNurb_VKnotsArray;          // sized: sum_ii(VvertexCount[ii] + VOrder[ii])
        m_sFace_SurfaceNurb_WeightsArray;         // sized: sum_ii(UvertexCount[ii] * VvertexCount[ii])
      - m_sFaceuseFaceIndexArray[ii] - the pointer from Faceuse[ii] to Face[index]

    - Loop Topology in arrays:
        // one entry per Loop for all loops in all breps in BrepArray
        // order: in Loop blocks in order of appearance in m_sFaceLoopCountArray
        // sized: sum of counts in m_sFaceLoopCountArray
        // indexing: FirstLoop_inFace_ii = Sum_jj(m_sFaceLoopCountArray[jj]) for jj= 0 to ii-1
        //           Number of Loops_inFace_ii = m_sFaceLoopCountArray[ii]
        // referenced by counts in: m_sFaceLoopCountArray
        m_sLoopEdgeuseCountArray;
        m_sLoopVertexIndexArray;

    - Edgeuse Topology in arrays:
        // one entry per Edgeuse for all edgeuses in all breps in BrepArray
        // order: in Edgeuse blocks in order of appearance in m_sLoopEdgeuseCountArray
        // sized: sum of counts in m_sLoopEdgeuseCountArray
        // indexing: FirstEdgeuse_inLoop_ii = Sum_jj(m_sLoopEdgeuseCountArray[jj]) for jj= 0 to ii-1
        // Number of Edgeuses_inLoop_ii = m_sLoopEdgeuseCountArray[ii]
        // referenced by indices in: m_sEdgeuseNextRadialEUIndexArray
        m_sEdgeuseEdgeIndexArray;
        m_sEdgeuseOrientationTypeArray;
        m_sEdgeuseNextRadialEUIndexArray;
        m_sEdgeuseThisRadialEntryTypeArray

    - Edge Topology in arrays:    // one entry per Edge for all edges in all breps in BrepArray
                                  // order: not packed in an order defined by Edgeuse entries
                                  // sized: sum of unique index values in m_sEdgeuseEdgeIndexArray
                                  // indexing: Edge_OfEdgeuse_ii = m_sEdgeuseEdgeIndexArray[ii]
                                  // Number of Edges_inEdgeuse_ii = 1
                                  // referenced by indices in: m_sEdgeuseEdgeIndexArray
        m_sEdgeCurveTypeArray;
        m_sEdgeRangeArray;
        m_sEdgeVertexIndicesArray
        with Edge geometry in :
            // one curve block per Edge for all edges in all breps in BrepArray
            // order: in order of appearance in m_sEdgeCurveTypeArray when type == BrepCurve3dNurbAPI
            // sized: number of curve blocks =
            // sum of m_sEdgeCurveTypeArray[ii] == UsdBrepCurveTokens->brepCurve3dNurbAPI values
            // indxing: CurveBlock_ofEdge_ii = Sum_jj(m_sEdgeCurveTypeArray[jj] == UsdBrepCurveTokens->brepCurve3dNurbAPI)
            // for jj= 0 to ii-1
            // Number of CurveBlocks_inEdge_ii = 1
            // referenced by counts in: m_sEdgeCurveTypeArray[ii] when
            // m_sEdgeCurveTypeArray[ii] == UsdBrepSolidTokens->BrepCurve3dNurbAPI m_sEdge3dNurb_ControlVerticesArray;
            // sized: sum_ii(VertexCount[ii])
               m_sEdge3dNurb_VertexCountArray;     // sized: one entry per Edge for all edges in BrepArray
               m_sEdge3dNurb_OrderArray;           // sized: one entry per Edge for all edges in BrepArray
               m_sEdge3dNurb_KnotsArray;           // sized: sum_ii(VertexCount[ii] + Order[ii])
               m_sEdge3dNurb_WeightsArray;         // sized: sum_ii(VertexCount[ii])

    - WireEdge Topology in arrays:
        // one entry per WireEdge for all wireEdges in all breps in BrepArray
        // order: one wireEdge block per Shell in order of appearance in m_sShellWireEdgeCountArray
        // sized: sum of counts in m_sShellWireEdgeCountArray
        // indexing: FirstWireEdge_ofShell_ii = Sum_jj(m_sShellWireEdgeCountArray[jj]) for jj= 0 to ii-1
        //           Number of WireEdges_inShell_ii = m_sShellWireEdgeCountArray[ii]
        // referenced by counts in : m_sShellWireEdgeCountArray
        m_sWireEdgeCurveTypeArray;
        m_sWireEdgeRangeArray;
        m_sWireEdgeVertexIndicesArray
        with WireEdge geometry in :
        // one curve block per WireEdge for all wireEdges in all breps in BrepArray
        // order: in order of appearance in m_sWireEdgeCurveTypeArray when type == BrepCurve3dNurbAPI
        // sized: number of curve blocks =
        // sum of m_sWireEdgeCurveTypeArray[ii] == UsdBrepSolidTokens->BrepCurve3dNurbAPI values
        // indexing: CurveBlock_ofWireEdge_ii = Sum_jj(m_sWireEdgeCurveTypeArray[jj] ==
        // UsdBrepSolidTokens->BrepCurve3dNurbAPI) for jj= 0 to ii-1
        // Number of CurveBlocks_inWireEdge_ii = 1
        // referenced by counts in: m_sWireEdgeCurveTypeArray[ii] when
        // m_sWireEdgeCurveTypeArray[ii] == UsdBrepSolidTokens->BrepCurve3dNurbAPI m_sWireEdge3dNurb_ControlVerticesArray;
        // sized: sum_ii(VertexCount[ii])
           m_sWireEdge3dNurb_VertexCountArray;     // sized: one per WireEdge for all WireEdges in BrepArray
           m_sWireEdge3dNurb_OrderArray;           // sized: one per WireEdge for all WireEdges in BrepArray
           m_sWireEdge3dNurb_KnotsArray;           // sized: sum_ii(VertexCount[ii] + Order[ii])
           m_sWireEdge3dNurb_WeightsArray;         // sized: sum_ii(VertexCount[ii])
      - m_sEdgeuseEdgeIndexArray[ii] - the pointer from Edgeuse[ii] to Edge[index]

    - Vertex Topology in arrays:
        // one entry per Vertex for all Edge, WireEdge, and Loop Vertices in all breps in BrepArray
        // order: not packed in an order defined by Loop or Edge entries
        // sized:   sum of unique index values in m_sEdgeVertexIndicesArray
        //        + sum of unique index values in m_sWireEdgeVertexIndicesArray
        //        +                               m_sLoopVertexIndexArray.size()
        // referenced by indices in: m_sLoopVertexIndexArray
        // referenced by indices in: m_sEdgeVertexIndicesArray
        // referenced by indices in: m_sWireEdgeVertexIndicesArray
        m_sVertexPointTypeArray;
        with vertex geometry in :
            // one PointPosition per Vertex for all Edge, WireEdge, and Loop Vertices in all breps in BrepArray
            // order: in order of appearance in m_sVertexPointTypeArray when type == BrepPointAPI
            // sized: number of VertexPoints = number of vertex topology objs
            // indexing: VertexPoint_ofVertex_ii = Sum_jj(m_sVertexPointTypeArray[jj] == UsdBrepSolidTokens->BrepPointAPI)
            // for jj= 0 to ii-1
            // Number of VertexPoints_inVertex_ii = 1
            // referenced by counts in: m_sVertexPointTypeArray[ii] when
            // m_sVertexPointTypeArray[ii] == UsdBrepSolidTokens->BrepPointAPI

 ********************************************************************************************************************/
class UsdBrepArrayData final
{
public:

    // Default constructor: compiler generates default initialization for all members
    UsdBrepArrayData() = default;

    // Non-virtual destructor is approriate because UsdBrepArrayData won't be used as base class.
    // noexcept specification provides better exception safety
    // = default: compiler generates destructor that calls destructors of all member objects
    ~UsdBrepArrayData() noexcept = default;

    // Disable copy semantics to prevent accidental expensive copies
    UsdBrepArrayData(const UsdBrepArrayData&) = delete;
    UsdBrepArrayData& operator=(const UsdBrepArrayData&) = delete;

    // note: TotalObjectCount values are not stored directly in this design but
    //       are implied through the length of arrays which store properties
    //       one-to-one with the object being counted. These methods exist to
    //       encapsulate how one gets an object count from the data available.
    //       examples: TotalBrepCount      = m_sBrepRegionCountArray.size();
    //                 TotalEdgeCount      = m_sEdgeCurveTypeArray.size();
    //                 TotalEdgeCurveCount = m_sEdge_CurveNurb_OrderArray.size()

    // Topology Object counts
    USDBREP_EXPORT uint32_t TotalBrepCount() const; //        total brep count
    USDBREP_EXPORT uint32_t TotalRegionCount() const; //      total region count = Sum(Brep_ii->regionCount)
    USDBREP_EXPORT uint32_t TotalShellCount() const; //       total shell count = Sum(Brep_ii->shellCount)
    USDBREP_EXPORT bool IsBrepPointShell(uint32_t iShellIndex) const; // exact, bounds-safe BrepPoint shell predicate
    USDBREP_EXPORT uint32_t TotalShellVertexCount() const; // total shells satisfying IsBrepPointShell()
    USDBREP_EXPORT uint32_t TotalFaceuseCount() const; //     total faceuse count = Sum(Brep_ii->faceuseCount)
    USDBREP_EXPORT uint32_t TotalFaceCount() const; //        total face count = Sum(Brep_ii->faceCount)
    USDBREP_EXPORT uint32_t TotalLoopCount() const; //        total loop count = Sum(Brep_ii->loopCount)
    USDBREP_EXPORT uint32_t TotalLoopVertexCount() const; //  total Vertexloop count = Sum(Brep_ii->loopCount when LoopType==LoopVertex)
    USDBREP_EXPORT uint32_t TotalEdgeuseCount() const; //     total edgeuse count = Sum(Brep_ii->edgeuseCount)
    USDBREP_EXPORT uint32_t TotalEdgeCount() const; //        total faceEdge count = Sum(Brep_ii->faceEdgeCount)
    USDBREP_EXPORT uint32_t TotalWireEdgeCount() const; //    total wireEdge count = Sum(Brep_ii->wireEdgeCount)
    USDBREP_EXPORT uint32_t TotalVertexCount() const; //      total vertex count = Sum(Brep_ii->LoopVertexCount + EdgeVertexCount)

    // Geometry Object counts
    USDBREP_EXPORT uint32_t TotalShellVertexPositionCount() const;
    USDBREP_EXPORT uint32_t TotalVertexPositionCount() const;
    USDBREP_EXPORT uint32_t TotalEdgeCurveCount() const;
    USDBREP_EXPORT uint32_t TotalEdgeControlVerticesCount() const;
    USDBREP_EXPORT uint32_t TotalEdgeKnotCount() const;
    USDBREP_EXPORT uint32_t TotalWireEdgeCurveCount() const;
    USDBREP_EXPORT uint32_t TotalWireEdgeControlVerticesCount() const;
    USDBREP_EXPORT uint32_t TotalWireEdgeKnotCount() const;
    USDBREP_EXPORT uint32_t TotalEdgeuseCurveCount() const; // Count all Edgeuse entries with and without Zero entries.
    USDBREP_EXPORT uint32_t TotalEdgeuseNonNullCurveCount() const; // Only count Edgeuse entries with nonZero entries.
                                                                   // UVTrimCurves are optional. When omitted, place '0' entries in
                                                                   //   m_sEdgeuse_CurveNurb_VertexCountArray and
                                                                   //   m_sEdgeuse_CurveNurb_OrderArray arrays.
    USDBREP_EXPORT uint32_t TotalEdgeuseControlVerticesCount() const;
    USDBREP_EXPORT uint32_t TotalEdgeuseKnotCount() const;
    USDBREP_EXPORT uint32_t TotalFaceSurfaceCount() const;
    USDBREP_EXPORT uint32_t TotalFaceControlVerticesCount() const;
    USDBREP_EXPORT uint32_t TotalFaceKnot_UCount() const;
    USDBREP_EXPORT uint32_t TotalFaceKnot_VCount() const;

    // map between BrepArray and lone Brep indices
    USDBREP_EXPORT static uint32_t MapGlobalToLocal(uint32_t uiGlobalIndex, uint32_t uiGlobalStartIndex);
    USDBREP_EXPORT static uint32_t MapLocalToGlobal(uint32_t uiLocalIndex, uint32_t uiGlobalStartIndex);

    // clear data structure
    USDBREP_EXPORT void ReSet();

public:

    // BrepArray metadata - data not within the Schema: inherited or intended USD usage.

    // inherited metadata data - to consider storing here for round-trip translations
    //   UsdDisplayColorAttribute,          // inherited from usdGeomPrim
    //   UsdOpacityAttribute,               // inherited from usdGeomPrim
    //   UsdDoubleSidedAttribute,           // inherited from usdGeomPrim
    //   UsdOrientationAttribute,           // inherited from usdGeomPrim
    //   XformOpOrder and XformData         // inherited from usdGeomXformable
    // {BrepArray XformOps}, one entry per BrepArray
    // UsdVisibilityAttribute,              // inherited from usdGeomImageable
    // UsdPurposeAttribute,                 // inherited from usdGeomImageable

    // context metadata data
    pxr::SdfPath m_sPrimPath; // {PrimPath} = UsdBrepArray.GetPrim(). Stored in Brep->Attributes,
                              // one entry per BrepArray
    pxr::GfRange3f m_sBrepArray_BBox; // {{MinXYZ, MaxXYZ], stored in usdGeomBoundable as a float array

    // Material-for-rendering attribute data
    pxr::SdfPath m_sBrepArray_MaterialPath; // MaterialPath for BrepArray used when all Breps use same material,
                                            // one entry per BrepArray

    pxr::SdfPathVector m_sBrepMaterial_BrepPathArray; // {MaterialPath} for perBrep materials when different Breps use
                                                      // different materials, one entry per BrepMaterial used in
                                                      // BrepArray
    pxr::VtArray<pxr::VtIntArray> m_sBrepMaterial_BrepIndexArray; // {global BrepIndex list} BrepIndex list of Breps
                                                                  // using BrepMaterialsPaths[ii] material, one entry
                                                                  // per BrepMaterial used in BrepArray

    pxr::SdfPathVector m_sFaceMaterial_FacePathArray; // {MaterialPath} for perFace materials when different faces use
                                                      // different materials,
                                                      // one entry per FaceMaterial used in BrepArray
    pxr::VtArray<pxr::VtIntArray> m_sFaceMaterial_FaceIndexArray; // {global FaceIndex list} faceIndex list of faces
                                                                  // using FaceMaterialsPaths[ii] material,
                                                                  // one entry per FaceMaterial used in BrepArray

    // perBrepArray Schema attribute - one value per BrepArray
    std::string m_sCADSource; // {CADSource string}, one entry per BrepArray
                              // may be added

    // perBrep Schema attributes arrays - one value per Brep
    VtArray<double> m_sBrepXSectTol3dArray; // {Brep XSectTol3d},               one per Brep, sized:[BrepCnt]
    VtArray<GfVec3d> m_sBrepExtentArray; // {Brep BBoxMin, BBoxMax}, two per Brep, sized:[2*BrepCnt]
    VtArray<uint32_t> m_sBrepRegionCountArray; // {Brep Region Cnt},               one per Brep, sized:[BrepCnt]

    // BrepArray Topology - one block of values per BrepTopologyObj - lists of Brep topology object values for all the
    // Breps in a BrepArray organized

    // Regions - (1st region of every Brep_ii block is Brep_ii's infinite region)
    VtArray<uint32_t> m_sRegionShellCountArray; // {Loop cnt},                            one per Region, sized:[TotalRegionCnt]
    VtArray<TfToken> m_sRegionTypeArray; //        {oneof:["solidRegion", "voidRegion"]}, one per Region, sized:[TotalRegionCnt]

    // Shells - one block for every Region entry
    VtArray<uint32_t> m_sShellFaceuseCountArray; //  {Faceuse cnt},                    one per Shell, sized:[TotalShellCnt]
    VtArray<uint32_t> m_sShellWireEdgeCountArray; // {WireEdge cnt},                   one per Shell, sized:[TotalShellCnt]
    VtArray<TfToken> m_sShellPointTypeArray; //      {oneof:["BrepPointAPI", "none"],} one per Shell, sized:[TotalShellCnt]
                                             //   in the future: may add option for MultiPoints:["BrepMultiPointAPI"]

    // Faceuses - one block for every Shell entry
    VtArray<uint32_t> m_sFaceuseFaceIndexArray; //      {global FaceIndex},           one per Faceuse, sized:[TotalFaceuseCnt]
    VtArray<TfToken> m_sFaceuseOrientationTypeArray; // {oneof:["same", "opposite"]}, one per Faceuse, sized:[TotalFaceuseCnt]
                                                     //   same     = face side in the face's surface normal direction.
                                                     //   opposite = the opposite side of the face.

    // Faces - one block for every brep_ii
    VtArray<uint32_t> m_sFaceLoopCountArray; // {LoopCount},                     one per Face, sized:[TotalFaceCnt]
    VtArray<TfToken>
        m_sFaceSurfaceTypeArray; // {oneof:["BrepSurfaceNurbAPI","BrepSurfaceSphereAPI","BrepSurfacePlaneAPI","BrepSurfaceCylinderAPI","BrepSurfaceConeAPI","BrepSurfaceTorusAPI"]},
                                 // one per Face, sized:[TotalFaceCnt]
    VtArray<TfToken> m_sFaceTrimTypeArray; //    {oneof:["rectangular", "general"]}, one per Face, sized:[TotalFaceCnt]
    VtArray<GfVec2d> m_sFaceRangeArray; // {UVDomain:[UVMin, UVMax] },           two per Face, sized:[2*TotalFaceCnt]

    // Loops - one block for every Face entry
    VtArray<uint32_t> m_sLoopEdgeuseCountArray; // {EdgeuseCount},      one per Loop, sized:[TotalLoopCnt]
    VtArray<uint32_t> m_sLoopVertexIndexArray; // {global VertexIndex}, one per Loop, sized:[TotalLoopCnt]
                                               //    when m_sLoopEdgeuseCountArray[ii] == 0,
                                               //    loop_ii is a vertexLoop and m_sLoopVertexIndexArray[ii] = VertexIndex
                                               //    otherwise the value is ignored and set to USDBREP_NO_OBJECT_INDEX.

    // Edgeuses - one block per Loop. Zero length blocks for loops of type VertexLoop. WireEdges do not connect to edgeuses.
    VtArray<uint32_t> m_sEdgeuseEdgeIndexArray; //          {global EdgeIndex},                  one per Edge_Edgeuse, sized:[TotalEdge_EdgeuseCnt]
    VtArray<TfToken> m_sEdgeuseOrientationTypeArray; //     {oneof:["same", "opposite"]          one per Edge_Edgeuse, sized:[TotalEdge_EdgeuseCnt]
                                                     //       same     = UVTrimCurve runs in edge's curve direction
                                                     //                  representing the binormal side of this edge-face connection.
                                                     //       opposite = UVTrimCurve runs in the opposite direction
                                                     //                  representing the other side of this edge-face connection.
    VtArray<uint32_t> m_sEdgeuseNextRadialEUIndexArray; //  {global nextRadialEdgeuseIndex},     one per Edge_Edgeuse, sized:[TotalEdge_EdgeuseCnt]
                                                        //   nextRadialEdgeuse = nextEdgeuse in a right-hand-rule traversal around the edgeuse->edge
    VtArray<TfToken> m_sEdgeuseThisRadialEntryTypeArray; // {oneof:["topEntry", "bottomEntry"]}, one per Edge_Edgeuse, sized:[TotalEdge_EdgeuseCnt]
                                                         //   topEntry    = {topFaceSideEntry botFaceSideExit} sequence in a traversal around edge
                                                         //   bottomEntry = {botFaceSideEntry topFaceSideExit} opposite sequence.

    // Edges - one block per Brep_ii
    VtArray<TfToken> m_sEdgeCurveTypeArray; //     {oneof:["BrepCurve3dNurbAPI","BrepCurve3dCircleAPI","BrepCurve3dLineAPI","BrepCurve3dEllipseAPI"]},
                                            //                                             one per Edge, sized:[TotalEdgeCnt]
    VtArray<double> m_sEdgeRangeArray; //          {CrvStartParam, CrvEndParam},                        two per Edge, sized:[2*TotalEdgeCnt]
    VtArray<GfVec2i> m_sEdgeVertexIndicesArray; // {Global StartVtxIndex, Global EndVtxIndex}, one per Edge, sized:[TotalEdgeCnt]
                                                // note: VertexIndices should be uint but usd has no GfVec2ui type.
                                                //       These can be cast to uint32_t without loss of data.

    // WireEdges - one block per Shell, Zero length blocks for VertexShells and FaceuseShells.
    VtArray<TfToken>
        m_sWireEdgeCurveTypeArray; //     {oneof:["BrepCurve3dNurbAPI","BrepCurve3dCircleAPI","BrepCurve3dLineAPI","BrepCurve3dEllipseAPI"]},
                                   //                                               one per WireEdge, sized:[TotalWireEdgeCnt]
    VtArray<double> m_sWireEdgeRangeArray; // {CrvStart, CrvEnd},                 two per WireEdge, sized:[2*TotalWireEdgeCnt]
    VtArray<GfVec2i> m_sWireEdgeVertexIndicesArray; // {Global StartVtxIndex, Global EndVtxIndex}, one per WireEdge, sized:[TotalWireEdgeCnt]
                                                    // note: VertexIndices should be uint but usd has no GfVec2u type.
                                                    //       These can be cast to uint32_t without loss of data.

    // Vertices - one block per Brep_ii
    VtArray<TfToken> m_sVertexPointTypeArray; // {oneof:["BrepPointAPI", "none"]}, one entry per Vertex, sized:[TotalVertexCnt]
                                              //   in the future: may add option for MultiPoints:["vertex_BrepMultiPointAPI"]

    /************************************************************************
    // geometry data array declarations:
    //   VertexPoint, ShellPoint, EdgeCurve, WireEdgeCurve, EdgeuseCurve, and FaceSurface
    ************************************************************************/

    // VertexShell geometry
    // packed in shell order for shells satisfying IsBrepPointShell(ii)
    VtArray<GfVec3d> m_sShell_PointPositionArray; // {Pnt3d},  one per BrepArray ShellVertex,
                                                  // sized:[TotalShellVertexCnt]

    // LoopVertex and EdgeVertex geometry
    // packed in order of appearance in m_sVertexPointTypeArray[ii] == UsdBrepSolidTokens->brepPointAPI
    VtArray<GfVec3d> m_sVertex_PointPositionArray; // {Pnt3d},  one per BrepArray Vertex,
                                                   // sized:[TotalVertexCnt]

    // Edge NurbCurve geom
    // packed in order of appearance in m_sEdgeCurveTypeArray[ii] == UsdBrepSolidTokens->BrepCurve3dNurbAPI
    // KnotCount = VertexCount + Order, (where: Order = Degreee + 1)
    VtArray<GfVec3d> m_sEdge_CurveNurb_ControlVerticesArray; // {(x,y,z)/CtrlPt},      one per ControlPoint,
                                                             // sized:[TotalControlPointCount for all Edge_NurbCurves]
    VtArray<uint32_t> m_sEdge_CurveNurb_VertexCountArray; // {CtrlPt Count/curve},     one per curve,
                                                          // sized:[total Edge_NurbCrvCount]
    VtArray<uint32_t> m_sEdge_CurveNurb_OrderArray; // {order/curve}, (order=degree+1) one  per curve,
                                                    // sized:[total Edge_NurbCrvCount]
    VtArray<double> m_sEdge_CurveNurb_KnotsArray; // {param/knot},                     one per knot,
                                                  // sized:[TotalKnotCount for all Edge_NurbCurves]
    VtArray<double> m_sEdge_CurveNurb_WeightsArray; // {wt/CtrlPt},                    one per ControlPoint,
                                                    // sized:[TotalControlPointCount for all Edge_NurbCurves]

    // Edge Circle curve geom
    // packed in order of appearance in m_sEdgeCurveTypeArray[ii] == UsdBrepCurveTokens->brepCurve3dCircleAPI
    VtArray<GfVec3d> m_sEdge_CurveCircle_CenterArray; // {x,y,z},   one per circle edge
    VtArray<GfVec3d> m_sEdge_CurveCircle_AxisArray; // {x,y,z},     one per circle edge
    VtArray<GfVec3d> m_sEdge_CurveCircle_RefDirectionArray; // {x,y,z}, one per circle edge
    VtArray<double> m_sEdge_CurveCircle_RadiusArray; // {radius},    one per circle edge

    // Edge Line curve geom
    // packed in order of appearance in m_sEdgeCurveTypeArray[ii] == UsdBrepCurveTokens->brepCurve3dLineAPI
    VtArray<GfVec3d> m_sEdge_CurveLine_OriginArray; // {x,y,z},     one per line edge
    VtArray<GfVec3d> m_sEdge_CurveLine_DirectionArray; // {x,y,z},   one per line edge

    // Edge Ellipse curve geom
    VtArray<GfVec3d> m_sEdge_CurveEllipse_CenterArray;
    VtArray<GfVec3d> m_sEdge_CurveEllipse_AxisArray;
    VtArray<GfVec3d> m_sEdge_CurveEllipse_RefDirectionArray;
    VtArray<double> m_sEdge_CurveEllipse_XRadiusArray;
    VtArray<double> m_sEdge_CurveEllipse_YRadiusArray;

    // WireEdge NurbCurve geom
    // packed in order packed in order of appearance in m_sWireEdgeCurveTypeArray[ii] ==
    // UsdBrepSolidTokens->BrepCurve3dNurbAPI KnotCount = VertexCount + Order, (where: Order = Degreee + 1)
    VtArray<GfVec3d> m_sWireEdge_CurveNurb_ControlVerticesArray; // {(x,y,z)/CtrlPt},      one per ControlPoint,
                                                                 // sized:[TotalCtrlPtCnt for all Wire_NurbCurves]
    VtArray<uint32_t> m_sWireEdge_CurveNurb_VertexCountArray; // {CtrlPt Count/curve},     one per curve,
                                                              // sized:[total WireEdge_NurbCrvCount]
    VtArray<uint32_t> m_sWireEdge_CurveNurb_OrderArray; // {order/curve}, (order=degree+1) one per crv,
                                                        // sized:[total WireEdge_NurbCrvCount]
    VtArray<double> m_sWireEdge_CurveNurb_KnotsArray; // {param/knot},                     one per knot,
                                                      // sized:[TotalKnotCount for all Wire_NurbCurves]
    VtArray<double> m_sWireEdge_CurveNurb_WeightsArray; // {wt/CtrlPt},                    one per ControlPoint,
                                                        // sized:[TotalCtrlPtCnt for all Wire_NurbCurves]

    // WireEdge Circle curve geom
    VtArray<GfVec3d> m_sWireEdge_CurveCircle_CenterArray;
    VtArray<GfVec3d> m_sWireEdge_CurveCircle_AxisArray;
    VtArray<GfVec3d> m_sWireEdge_CurveCircle_RefDirectionArray;
    VtArray<double> m_sWireEdge_CurveCircle_RadiusArray;

    // WireEdge Line curve geom
    VtArray<GfVec3d> m_sWireEdge_CurveLine_OriginArray;
    VtArray<GfVec3d> m_sWireEdge_CurveLine_DirectionArray;

    // WireEdge Ellipse curve geom
    VtArray<GfVec3d> m_sWireEdge_CurveEllipse_CenterArray;
    VtArray<GfVec3d> m_sWireEdge_CurveEllipse_AxisArray;
    VtArray<GfVec3d> m_sWireEdge_CurveEllipse_RefDirectionArray;
    VtArray<double> m_sWireEdge_CurveEllipse_XRadiusArray;
    VtArray<double> m_sWireEdge_CurveEllipse_YRadiusArray;

    // Edgeuse NurbCurve geom
    // There is a one-to-one relationship between Edgeuses and Edgeuse_UVTrimCurves.
    //  1. Edgeuse_UVTrimCurve data is packed in the same order as the Edgeuses.
    //  2. Edgeuse_UVTrimCurves are always represented as Nurbs curves.
    //  3. Edgeuse_UVTrimCurves are optional.  Some Edgeuses may have them and others not.
    //     +-----------------------------------------------+-----------------------------------+--------------------+
    //     | when sArray.Edgeuse[ii]                       | has a UVTrimCurve                 | has no UVTrimCurve |
    //     +-----------------------------------------------+-----------------------------------+--------------------+
    //     |  m_sEdgeuse_CurveNurb_VertexCountArray[ii]    | = UVTrimCurve ControlVertex count |  = 0               |
    //     |  m_sEdgeuse_CurveNurb_OrderArray[ii]          | = UVTrimCurve Order               |  = 0               |
    //     |  m_sEdgeuse_CurveNurb_ControlVerticesArray    |  gets VertexCount entries         |   gets no entries  |
    //     |  m_sEdgeuse_CurveNurb_WeightsArray            |  gets VertexCount entries         |   gets no entries  |
    //     |  m_sEdgeuse_CurveNurb_KnotsArray              |  gets VertexCount + Order entries |   gets no entries  |
    //     +-----------------------------------------------+-----------------------------------+--------------------+
    // KnotCount = VertexCount + Order, (where: Order = Degreee + 1)
    VtArray<GfVec2d> m_sEdgeuse_CurveNurb_ControlVerticesArray; // {(u,v)/CtrlPt},         one per ControlPoint,
                                                                // sized:[TotCtrlPtCnt for all Edgeuse_UV_NurbCurves]
    VtArray<uint32_t> m_sEdgeuse_CurveNurb_VertexCountArray; // {CtrlPt Count/curve},      one per curve,
                                                             // sized:[total Edgeuse_UV_NurbCurveCount]
    VtArray<uint32_t> m_sEdgeuse_CurveNurb_OrderArray; // {order/curve}, (order=degree+1), one per crv,
                                                       // sized:[total Edgeuse_UV_NurbCurveCount]
    VtArray<double> m_sEdgeuse_CurveNurb_KnotsArray; // {param/knot},                      one per knot,
                                                     // sized:[TotalKnotCnt for all Edgeuse_UV_NurbCurves]
    VtArray<double> m_sEdgeuse_CurveNurb_WeightsArray; // {wt/CtrlPt},                     one per ControlPoint,
                                                       // sized:[TotCtrlPtCnt for all Edgeuse_UV_NurbCurves]

    // Face NurbSurface geometry
    // packed in order of appearance in m_sFaceSurfaceTypeArray[ii] == UsdBrepSurfaceTokens->brepSurfaceNurbAPI
    // KnotCount_U = VertexCount_U + Order_U, (where: Order_U = Degreee_U + 1)
    // KnotCount_V = VertexCount_V + Order_V, (where: Order_V = Degreee_V + 1)
    VtArray<GfVec3d> m_sFace_SurfaceNurb_ControlVerticesArray; // {x,y,z/CtrlPt},           one per ControlPoint,
                                                               // sized:[TotalControlPointCount for all Face_NurbSurfaces]
    VtArray<uint32_t> m_sFace_SurfaceNurb_UVertexCountArray; // {CtrlPtU Count/Surface},    one per surface,
                                                             // sized:[total Face_NurbSurfaceCount]
    VtArray<uint32_t> m_sFace_SurfaceNurb_VVertexCountArray; // {CtrlPtV Count/Surface},    one per surface,
                                                             // sized:[total Face_NurbSurfaceCount]
    VtArray<uint32_t> m_sFace_SurfaceNurb_UOrderArray; // (orderU=degreeU+1/Surface),       one per surface,
                                                       // sized:[total Face_NurbSurfaceCount]
    VtArray<uint32_t> m_sFace_SurfaceNurb_VOrderArray; // (orderV=degreeV+1/Surface),       one per surface,
                                                       // sized:[total Face_NurbSurfaceCount]
    VtArray<double> m_sFace_SurfaceNurb_UKnotsArray; // {UParam/knotU},                     one per knotU,
                                                     // sized:[TotalKnotCount for all Face_NurbSurfaces]
    VtArray<double> m_sFace_SurfaceNurb_VKnotsArray; // {VParam/knotV},                     one per knotV,
                                                     // sized:[TotalKnotCount for all Face_NurbSurfaces]
    VtArray<double> m_sFace_SurfaceNurb_WeightsArray; // {wt/CtrlPt},                       one per ControlPt,
                                                      // sized:[TotalCtrlPtCount for all Face_NurbSurfaces]

    // Face Sphere Surface geometry
    // packed in order of appearance in m_sFaceSurfaceTypeArray[ii] == UsdBrepSurfaceTokens->brepSurfaceSphereAPI
    VtArray<GfVec3d> m_sFace_SurfaceSphere_CenterArray; // {x,y,z},   one per sphere face, sized:[TotalSphereFaceCount]
    VtArray<GfVec3d> m_sFace_SurfaceSphere_AxisArray; // {x,y,z},     one per sphere face, sized:[TotalSphereFaceCount]
    VtArray<GfVec3d> m_sFace_SurfaceSphere_RefDirectionArray; // {x,y,z}, one per sphere face, sized:[TotalSphereFaceCount]
    VtArray<double> m_sFace_SurfaceSphere_RadiusArray; // {radius},    one per sphere face, sized:[TotalSphereFaceCount]

    // Face Plane Surface geometry
    VtArray<GfVec3d> m_sFace_SurfacePlane_OriginArray;
    VtArray<GfVec3d> m_sFace_SurfacePlane_AxisArray;
    VtArray<GfVec3d> m_sFace_SurfacePlane_RefDirectionArray;

    // Face Cylinder Surface geometry
    VtArray<GfVec3d> m_sFace_SurfaceCylinder_OriginArray;
    VtArray<GfVec3d> m_sFace_SurfaceCylinder_AxisArray;
    VtArray<GfVec3d> m_sFace_SurfaceCylinder_RefDirectionArray;
    VtArray<double> m_sFace_SurfaceCylinder_RadiusArray;

    // Face Cone Surface geometry
    VtArray<GfVec3d> m_sFace_SurfaceCone_OriginArray;
    VtArray<GfVec3d> m_sFace_SurfaceCone_AxisArray;
    VtArray<GfVec3d> m_sFace_SurfaceCone_RefDirectionArray;
    VtArray<double> m_sFace_SurfaceCone_RadiusArray;
    VtArray<double> m_sFace_SurfaceCone_SemiAngleArray;

    // Face Torus Surface geometry
    VtArray<GfVec3d> m_sFace_SurfaceTorus_OriginArray;
    VtArray<GfVec3d> m_sFace_SurfaceTorus_AxisArray;
    VtArray<GfVec3d> m_sFace_SurfaceTorus_RefDirectionArray;
    VtArray<double> m_sFace_SurfaceTorus_MajorRadiusArray;
    VtArray<double> m_sFace_SurfaceTorus_MinorRadiusArray;

}; // end class UsdBrepArrayData

/*********************************************************************************************************************
 PURPOSE: one Brep_ii's list of global StartIndices and Counts
          for the arrays in a target UsdBrepArrayData object.

 NOTES: 1. Every [StartIndex, Count] pair define an IndexRange:[StartIndex, StartIndex + Count]
           for the corresponding arrays in the Tgt UsdBrepArrayData.

        2. Some geometry arrays don't store explicit start indices because
           in those cases StartIndex values can be computed from the counts as:
             StartIndex[jj] = Sum_kk(CountValue[kk]) for kk = 0 to jj-1
 ********************************************************************************************************************/
class USDBREP_EXPORT UsdBrepArraySpans final
{
public:

    // Default constructor: compiler generates default initialization for all members
    UsdBrepArraySpans() = default;

    // Non-virtual destructor is approriate because UsdBrepArraySpans won't be used as base class.
    // noexcept specification provides better exception safety
    // = default: compiler generates destructor that calls destructors of all member objects
    ~UsdBrepArraySpans() noexcept = default;

    // Disable copy semantics to prevent accidental expensive copies
    UsdBrepArraySpans(const UsdBrepArraySpans&) = delete;
    UsdBrepArraySpans& operator=(const UsdBrepArraySpans&) = delete;

    // set startIndices for the next BrepAdd using only the rArrays array lengths.
    bool SetStartsForNextBrepAdd(const UsdBrepArrayData& rArrays); // in : UsdBrepArrayData to parse

    // set Brep_ii start index and optional count values by walking rArrays data for every Brep up to Brep_ii
    bool SetStartsAndCountsForBrepIndex(
        const UsdBrepArrayData& rArrays, // in : UsdBrepArrayData to parse
        uint32_t iBrepIndex, //    in : Tgt Brep_ii index
        bool bSetStarts = true, // in : true  = start Indices set by walking rArrays
                                //              every Brep prior to Brep_ii.
                                //      false = use StartIndices as is assuming
                                //              they are set for Brep_ii.
                                //              default:[true]
        bool bSetCounts = false // in : true  = StartIndices and Counts set for Brep_ii
    ); //                               false = StartIndices set for Brep_ii, counts=0
       //                                       (sets rSpans for the next append call)
       //                                       default:[false]

    // increment StartIndex values with current counts.
    void IncrementStartIndices(bool bSaveCounts = false); // in : false = clear counts,
                                                          //      true  = don't, default:[false]
    // clear count values, don't touch StartIndices
    void ClearCounts();

    // clear count values and StartIndex values
    void ClearStartsAndCounts();

public:

    uint32_t m_lBrepIndex = 0; // Brep_ii index for the following StartIndices and Counts

    // Brep_ii topology object IndexRanges for arrays in tgt UsdBrepArrayData
    uint32_t m_lRegionStartIndex = 0; // for : m_sRegionShellCountArray,
    uint32_t m_lRegionCount = 0; //       m_sRegionTypeArray;

    uint32_t m_lShellStartIndex = 0; // for : m_sShellFaceuseCountArray,
    uint32_t m_lShellCount = 0; //       m_sShellPointTypeArray, m_sShellWireEdgeCountArray;

    uint32_t m_lShellVertexStartIndex = 0; // geom: m_sShell_PointPositionArray Brep_ii ShellVertex count
    uint32_t m_lShellVertexCount = 0; //            = number of IsBrepPointShell(jj) matches
                                      //              for jj=Brep_ii_ShellIndexRange

    uint32_t m_lFaceuseStartIndex = 0; // for : m_sFaceuseFaceIndexArray;
    uint32_t m_lFaceuseCount = 0; //       m_sFaceuseOrientationTypeArray;

    uint32_t m_lFaceStartIndex = 0; // for : m_sFaceLoopCountArray,
    uint32_t m_lFaceCount = 0; //       m_sFaceTrimTypeArray, m_sFaceSurfaceTypeArray,
                               //       and 2*count for m_sFaceRangeArray,
                               // geom: m_sFace_SurfaceNurb_UVertexCountArray,
                               //       m_sFace_SurfaceNurb_VVertexCountArray
                               //       m_sFace_SurfaceNurb_UOrderArray,
                               //       m_sFace_SurfaceNurb_VOrderArray

    uint32_t m_lLoopStartIndex = 0; // for: m_sLoopEdgeuseCountArray,
    uint32_t m_lLoopCount = 0; //      m_sLoopVertexIndexArray;

    uint32_t m_lLoopVertexCount = 0; // number of LoopVertices in Brep_ii Loop list
                                     // = Sum_jj(m_sLoopEdgeuseCountArray[jj] == 0) for jj=Brep_ii_LoopIndexRange

    uint32_t m_lEdgeuseStartIndex = 0; // for : m_sEdgeuseEdgeIndexArray,       m_sEdgeuseNextRadialEUIndexArray,
    uint32_t m_lEdgeuseCount = 0; //       m_sEdgeuseOrientationTypeArray, m_sEdgeuseThisRadialEntryTypeArray;
                                  // geom: m_sEdgeuse_CurveNurb_VertexCountArray, m_sEdgeuse_CurveNurb_OrderArray

    uint32_t m_lEdgeStartIndex = 0; // for : m_sEdgeCurveTypeArray,
    uint32_t m_lEdgeCount = 0; //       m_sEdgeVertexIndicesArray;
                               //       and  m_sEdgeRangeArray, sized:[2*m_lEdgeCount]
                               // geom: m_sEdge_CurveNurb_VertexCountArray, m_sEdge_CurveNurb_OrderArray;

    uint32_t m_lWireEdgeStartIndex = 0; // for : m_sWireEdgeCurveTypeArray,
    uint32_t m_lWireEdgeCount = 0; //       m_sWireEdgeVertexIndicesArray;
                                   //       and m_sWireEdgeRangeArray, sized:[2*m_lWireEdgeCount]
                                   // geom: m_sWireEdge_CurveNurb_VertexCountArray,
                                   //       m_sWireEdge_CurveNurb_OrderArray

    uint32_t m_lVertexStartIndex = 0; // for : m_sVertexPointTypeArray;
    uint32_t m_lVertexCount = 0; // geom: m_sVertex_PointPositionArray

    // VertexShell PointPositions
    uint32_t m_lShellPointPosition_StartIndex = 0; // for : m_sShell_PointPositionArray
    uint32_t m_lShellPointPosition_Count = 0; //       number of Brep_ii shells satisfying
                                              //       IsBrepPointShell(jj)

    // Vertex PointPositions
    uint32_t m_lVertexPointPosition_StartIndex = 0; // for : m_sVertex_PointPositionArray
    uint32_t m_lVertexPointPosition_Count = 0; //       number of unique Brep_ii->{edge->Start_EndVertices,
                                               //       wireEdge->Start_EndVertices, LoopVertices} with
                                               //       VertexPointType == "BrepPointAPI"

    // Edge NurbCurve geometry
    uint32_t m_lEdgeBSplineCurve3d_StartIndex = 0; // for : m_sEdge_CurveNurb_VertexCountArray,
                                                   // m_sEdge_CurveNurb_OrderArray
    uint32_t m_lEdgeBSplineCurve3d_Count = 0; // number of Brep_ii->edges with EdgeCurveType == "BrepCurve3dNurbAPI"
    uint32_t m_lEdgeBSplineCurve3d_ControlVerticesStartIndex = 0; // for : m_sEdge_CurveNurb_ControlVerticesArray,
                                                                  //       m_sEdge_CurveNurb_WeightsArray
    uint32_t m_lEdgeBSplineCurve3d_ControlVerticesCount = 0; //       ControlVertex count per Edge_NurbCurve for
                                                             //       IndexRange:[m_lEdgeBSplineCurve3d_StartIndex,
                                                             //       m_lEdgeBSplineCurve3d_Count]
    uint32_t m_lEdgeBSplineCurve3d_KnotStartIndex = 0; // for : m_sEdge_CurveNurb_KnotsArray
    uint32_t m_lEdgeBSplineCurve3d_KnotCount = 0; //       Knot          count per Edge_NurbCurve for
                                                  //       IndexRange:[m_lEdgeBSplineCurve3d_StartIndex,
                                                  //       m_lEdgeBSplineCurve3d_Count]

    // Edge CircleCurve geometry
    uint32_t m_lEdgeCircleCurve3d_StartIndex = 0;
    uint32_t m_lEdgeCircleCurve3d_Count = 0;

    // Edge LineCurve geometry
    uint32_t m_lEdgeLineCurve3d_StartIndex = 0;
    uint32_t m_lEdgeLineCurve3d_Count = 0;

    // Edge EllipseCurve geometry
    uint32_t m_lEdgeEllipseCurve3d_StartIndex = 0;
    uint32_t m_lEdgeEllipseCurve3d_Count = 0;

    // WireEdge NurbCurve geometry
    uint32_t m_lWireEdgeBSplineCurve3d_StartIndex = 0; // for : m_sWireEdge_CurveNurb_VertexCountArray,
                                                       //       m_sWireEdge_CurveNurb_OrderArray
    uint32_t m_lWireEdgeBSplineCurve3d_Count = 0; //       number of Brep_ii->wireEdges with
                                                  //       EdgeCurveType == "BrepCurve3dNurbAPI"
    uint32_t m_lWireEdgeBSplineCurve3d_ControlVerticesStartIndex = 0; // for :
                                                                      // m_sWireEdge_CurveNurb_ControlVerticesArray,
                                                                      // m_sWireEdge_CurveNurb_WeightsArray
    uint32_t m_lWireEdgeBSplineCurve3d_ControlVerticesCount = 0; // ControlVertex count per WireEdge_NurbCurve for
                                                                 // IdxRange:[m_lWireEdgeBSplineCurve3d_StartIndex,
                                                                 // m_lWireEdgeBSplineCurve3d_Count]
    uint32_t m_lWireEdgeBSplineCurve3d_KnotStartIndex = 0; // for : m_sWireEdge_CurveNurb_KnotsArray
    uint32_t m_lWireEdgeBSplineCurve3d_KnotCount = 0; //       Knot count per WireEdge_NurbCurve for
                                                      //       IndexRange:[m_lWireEdgeBSplineCurve3d_StartIndex,
                                                      //       m_lWireEdgeBSplineCurve3d_Count]

    // WireEdge CircleCurve geometry
    uint32_t m_lWireEdgeCircleCurve3d_StartIndex = 0;
    uint32_t m_lWireEdgeCircleCurve3d_Count = 0;

    // WireEdge LineCurve geometry
    uint32_t m_lWireEdgeLineCurve3d_StartIndex = 0;
    uint32_t m_lWireEdgeLineCurve3d_Count = 0;

    // WireEdge EllipseCurve geometry
    uint32_t m_lWireEdgeEllipseCurve3d_StartIndex = 0;
    uint32_t m_lWireEdgeEllipseCurve3d_Count = 0;

    // Edgeuse UVTrimCurve NurbCurve geometry
    uint32_t m_lEdgeuseBSplineCurve2d_StartIndex = 0; // for : m_sEdgeuse_CurveNurb_VertexCountArray,
                                                      //       m_sEdgeuse_CurveNurb_OrderArray
    uint32_t m_lEdgeuseBSplineCurve2d_Count = 0; //       number of Brep_ii->edgeuses

    uint32_t m_lEdgeuseBSplineCurve2d_ControlVerticesStartIndex = 0; // for : m_sEdgeuse_CurveNurb_ControlVerticesArray,
                                                                     //       m_sEdgeuse_CurveNurb_WeightsArray
    uint32_t m_lEdgeuseBSplineCurve2d_ControlVerticesCount = 0; //       ControlVertex count per Edgeuse_NurbCurve
                                                                // IndexRange:[m_lEdgeuseBSplineCurve3d_StartIndex,
                                                                //             m_lEdgeuseBSplineCurve3d_Count]
    uint32_t m_lEdgeuseBSplineCurve2d_KnotStartIndex = 0; // for : m_sEdgeuse_CurveNurb_KnotsArray
    uint32_t m_lEdgeuseBSplineCurve2d_KnotCount = 0; //       Knot count per Edgeuse_NurbCurve for
                                                     //       IndexRange:[m_lEdgeuseBSplineCurve3d_StartIndex,
                                                     //                   m_lEdgeuseBSplineCurve3d_Count]

    // Face NurbSurface geometry
    uint32_t m_lFaceBSplineSurface_StartIndex = 0; // for : m_sFace_SurfaceNurb_UVertexCountArray,
                                                   //       m_sFace_SurfaceNurb_UOrderArray,
                                                   //       m_sFace_SurfaceNurb_VVertexCountArray,
                                                   //       m_sFace_SurfaceNurb_VOrderArray;
    uint32_t m_lFaceBSplineSurface_Count = 0; // number of Brep_ii->Faces with FaceCurveType == "BrepSurface3dNurbAPI"
    uint32_t m_lFaceBSplineSurface_ControlVerticesStartIndex = 0; // for : m_sFace_SurfaceNurb_ControlVerticesArray,
                                                                  //       m_sFace_SurfaceNurb_WeightsArray
    uint32_t m_lFaceBSplineSurface_ControlVerticesCount = 0; //       ControlVertex count per Face_NurbSurface for
                                                             //       IndexRange:[m_lFaceBSplineSurface_StartIndex,
                                                             //       m_lFaceBSplineSurface_Count]
    uint32_t m_lFaceBSplineSurface_UKnotStartIndex = 0; // for : m_sFace_SurfaceNurb_UKnotsArray
    uint32_t m_lFaceBSplineSurface_UKnotCount = 0; //       Knot_U        count per Face_NurbSurface for
                                                   //       IndexRange:[m_lFaceBSplineSurface_StartIndex,
                                                   //       m_lFaceBSplineSurface_Count]
    uint32_t m_lFaceBSplineSurface_VKnotStartIndex = 0; // for : m_sFace_SurfaceNurb_VKnotsArray
    uint32_t m_lFaceBSplineSurface_VKnotCount = 0; //       Knot_V        count per Face_NurbSurface for
                                                   //       IndexRange:[m_lFaceBSplineSurface_StartIndex,
                                                   //       m_lFaceBSplineSurface_Count]

    // Face SphereSurface geometry
    uint32_t m_lFaceSphereSurface_StartIndex = 0;
    uint32_t m_lFaceSphereSurface_Count = 0;

    // Face PlaneSurface geometry
    uint32_t m_lFacePlaneSurface_StartIndex = 0;
    uint32_t m_lFacePlaneSurface_Count = 0;

    // Face CylinderSurface geometry
    uint32_t m_lFaceCylinderSurface_StartIndex = 0;
    uint32_t m_lFaceCylinderSurface_Count = 0;

    // Face ConeSurface geometry
    uint32_t m_lFaceConeSurface_StartIndex = 0;
    uint32_t m_lFaceConeSurface_Count = 0;

    // Face TorusSurface geometry
    uint32_t m_lFaceTorusSurface_StartIndex = 0;
    uint32_t m_lFaceTorusSurface_Count = 0;

    // arrays used to count totalLineCount and totalVertexCount
    VtArray<int32_t> m_iProcessedEdge_1stEdgeuses; // These entries will be edge->PrimEU values.
                                                   // The PrimaryEdgeuse is the first edgeuse in the radial list
                                                   // and the 2nd edgeuse must be that edgeuse's mate.
                                                   //   for TopEntry EUs use the top Edgeuse index:[iLocal]
                                                   //   for BotEntry EUs use the bot Edgeuse index:[-ilocal-1]
    VtArray<bool> m_bProcessedVertices;

}; // end class UsdBrepArraySpans

} // end namespace UsdBrepData

#endif // no _USD_BREP_ARRAY_DATA_H_
