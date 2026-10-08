// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmBrep.h
* PURPOSE: Header file for SmBrep class.
**********************************************************************/

#ifndef __SMBREP_H__
#define __SMBREP_H__

#ifndef __SMOS_TYPES_H__
#include <SmTypes.h>
#endif

#ifndef __SMSAGOBJECT_H__
#include <SmSAGObject.h>
#endif

#ifndef __SMCONTEXT_H__
#include <SmContext.h>
#endif

#ifndef __SMHEALDATA_H__
#include <SmHealData.h>
#endif

class SmGapArray;
class SmPolyBrep;
class SmPolyFace;
class SmTriangleBag;
class SmCopyBrepMap;
class SmDerivSurfDefinition;
class SmGfxArraySet ;
class SmDropCurveFail ;
class SmBrepData ;
class SmMerge;

// First failure per input GetFaces() index.
enum SmTessellationStage
{
  SM_TS_BOUNDARY_PREPARATION,
  SM_TS_FACE_TESSELLATION,
  SM_TS_POLYGON_OUTPUT
};

struct SmTessellationFailure
{
  ULONG lFaceIndex;
  SmTessellationStage eStage;
  SmStatus eStatus;
};

// Edge-incidence statistics describe generated output, not full solid validity.
struct SmTessellationReport
{
  SmTArray<SmTessellationFailure> failures;
  ULONG laminaEdges = 0;
  ULONG spineEdges = 0;
  SmBoolean inputManifold = FALSE;
  SmBoolean outputManifold = FALSE;
  SmBoolean hasMesh = FALSE;
};

/*******************************************************************//**
PURPOSE: This enum defines options for setting region IsVoid values
  when calling function FindAndSetInfiniteRegion().

NOTES:
***********************************************************************/
enum SmSetRegionVoidType {
  SM_VS_NESTEDSOLID,         //  Make a nested solid model
                             //    see SetRegionIsVoidFlagsForNestedSolids().
  SM_VS_INFINITEREGION_ONLY, //  Just set new infinite region IsVoid value to TRUE and
                             //   leave all other regions alone.
  SM_VS_INFINITEREGION_SWAP  //  Old behavior
                             //   set old infinite-region->IsVoid = new infinite-region->IsVoid() value
                             //   set new infinite-region->IsVoid = TRUE.
};

/*******************************************************************//**
PURPOSE: Result struct for SmBrep::CompareBreps round-trip validation.
         Accumulates differences between two SmBrep objects and applies
         an allowlist to determine pass/fail.
***********************************************************************/
struct SmBrepCompareResult
{
  SmBoolean  bPass ;

  // Level 1 - topology counts
  ULONG      lFaceCountA,    lFaceCountB ;
  ULONG      lEdgeCountA,    lEdgeCountB ;
  ULONG      lVertexCountA,  lVertexCountB ;

  // Level 2 - surface/curve type preservation
  ULONG      lSurfaceTypeMismatches ;
  ULONG      lCurveTypeMismatches ;

  // Level 3 - geometry tolerance
  double     dMaxVertexGap ;

  // Allowlist flags (caller sets before calling CompareBreps)
  SmBoolean  bAllowAnalyticToBSpline ;
  SmBoolean  bAllowVertexReordering ;
  double     dVertexTolerance ;         // 0.0 = use brep's GetTolerance(), >0 = override

  SmBrepCompareResult()
    : bPass(TRUE)
    , lFaceCountA(0), lFaceCountB(0)
    , lEdgeCountA(0), lEdgeCountB(0)
    , lVertexCountA(0), lVertexCountB(0)
    , lSurfaceTypeMismatches(0)
    , lCurveTypeMismatches(0)
    , dMaxVertexGap(0.0)
    , bAllowAnalyticToBSpline(FALSE)
    , bAllowVertexReordering(FALSE)
    , dVertexTolerance(0.0)
    {}
} ;

/*******************************************************************//**
PURPOSE: This class represents a topological Boundary Representation.
   It can be used to represent trimmed surfaces, solids, or
   open shells.  The Brep is the top level object which owns
   all contained faces, edges, curves, vertices, etc.

NOTES:
***********************************************************************/
class SM_EXPORT SmBrep : public SmSAGObject
{
  friend class SmBrepConstructor;
  friend class SmTol;
  friend class SmHealData ;

public: // public state enables calls: SmTemporaryChangeValue<SmBoolean> sChange(pBrep->m_bEditingEnabled, TRUE) ;
        //                             SmTemporaryChangeValue<SmBoolean> sChange(pBrep->m_bMakeComposites, TRUE) ;
        //                             SmTemporaryChangeValue<SmBoolean> sChange(pBrep->m_bValidateOnly,   TRUE) ;
  SmBoolean              m_bEditingEnabled;  // This flag will be set to TRUE if the brep is in an edit mode.
                                             // This changes how trimmed surface caches are created.
                                             //     - only build subdivision tree, skip trimmed face cache data.
                                             //    this skips building UVTrimCurves and point classifies,
                                             //    which only work if the Brep is complete
                                             //    and not in the middle of a construction step.
                                             // default:[FALSE] is optimized for query.
// Remove Composites
//  SmBoolean              m_bMakeComposites;  // If TRUE, splitting edges and faces
//                                             // will create composites and share the underlying geometry.
//                                             // The default is FALSE which causes the geometry to be copied.
//                                             // This is not a state variable, i.e. it does not indicate the
//                                             // existence of composite geometry in this brep.
  SmBoolean              m_bValidateOnly;    // This flag keeps track of the validateOnly flag passed into
                                             // ValidateAndUpdateTolerance to avoid changing all parts of the brep
                                             // default:[FALSE] .
protected:

  TCHAR*                 m_pName;            // name assigned to this Brep, NULL for standalone

  // inherited:
  // SmOwningTopology::m_pList      - pointer to head of Brep's Region link-list
  // SmOwningTopology::m_lListSize  - number of Regions in this Brep
  //
  // Not used:
  // SmTopology::m_pListOwner       - 
  // SmTopology::m_pNext            -
  // SmTopology::m_pLast            - 

  // { begin long temporary note on NewTolerance/OldTolerance model problems
       // New/Old Tolerance Problem:
       //          This code includes a confused mix of old an new tolerance data fields and needs to be cleaned
       //          up to just use the new tolerance model.
       // WorkAround: For now the two values m_sBrepZoneTol3d and m_sThisZoneTol3d are tied together so that
       //                  m_sBrepZoneTol3d == m_sThisZoneTol3d.
       // Real Solution: Implement the new tolerance model design for Brep tolerances
       //                The working tolerance used as the value to define the local neighborhoods for all Brep::Vertex, Edge, and Face objs
       //                is not stored in SmBrep but rather in the SmBrep->cpContext.  A pointer to that cpContext is in every Vertex, Edge, and Face definition.
       //                Changing the SmContext::m_sThisZoneTol3d will change the tolerance for all entities sharing this SmContext object.
       // 
       //                    Water tightness:  1. When a Vertex, Edge, or Face is connected to another topology object through a gap that is larger than
       //                                         the Context's working tolerance, the tolerances of the topology objects are increased so that
       //                                         the local neighborhoods of the two topology objects intersect.  In the NewTolerance model
       //                                         a topology's object is a function of it's Context->ThisZoneTol3d value and it's max Gap size.
       //                                      2. To avoid 'sticky' large tolerance when topology connections are made through large gaps, the
       //                                         local neighborhoods of the vertices, edges, and faces will not be simply increased, but rather
       //                                         a local neighborhood will be defined for the Vertexuses and Edgeuses associated with the large gaps.
       //                                         This idea increases the topology object's local neighborhood by a minimal amount to make sure that
       //                                         it's local neighborhood intersects the local neighborhood of the large-gap connected-neighbor wihtout
       //                                         inadvertantly getting such a large local neighborhood that it starts to have tolerant-intersections with
       //                                         geometry to which it is not supposed to be connected.
       //                    Picking the Context->m_sThisZoneTol3d value:
       //                                         Working SmContext::m_sThisZoneTol3d values should be selected based on the size of the Brep being modeled,
       //                                         ie. pinholes should use different tolerances than battleships. When a new Brep is constructed it
       //                                         won't have any geometry to size so the SmContext::ThisZoneTol3d value is computed from
       //                                         a ModelSizeEstimate value.  Large ranges of ModelSize measures get rounded into a small set
       //                                         of ModelSizeEstimates so that parts built on the same order of magnitude all share a common tolerance value.
       //                                         So, the initial value of SmContext::m_sThisZoneTol3d comes from a ModelSize measure or guess.  For parts being imported
       //                                         that already have geometry defined, the ModelSize can come from any measure of the models size, eg. The model's MaxEdgeLength,
       //                                         the Model's bounding box, or even the bounding box of just the model's vertices. For models being created
       //                                         interactively the ModelSize comes from a guess about how big the part will be when completed.  That guess
       //                                         can be supplied by the user or come from a default system value called SM_USE_DEFAULT.  A very large range
       //                                         of model sizes will all use the same working ZoneTol3d value and for parts that fit in a bounding box between
       //                                         0.5 and 500 units the default SM_USE_DEFAULT will be just fine.
       //                                          code:
       //                                           // no need to call SmTol::SizeToModelSizeEstimate() before calling SmTol::SizeEstimateToZoneTol3d()
       //                                           // both functions encode the same ModelSize_to_ZoneTol3d tables
       //                                           // SmBrep::m_dThisModelSizeEstimate = SmTol::SizeToModelSizeEstimate(AnyModelSizeMeasure,
       //                                           //                                                                   a User's model size guess, or the
       //                                           //                                                                   SM_USE_DEFAULT def value)   (<== recommended in most cases) 
       //                                           // SmBrep::m_dThisModelSizeEstimate = (AnyModelSizeMeasure,
       //                                           //                                     a User's model size guess, or the
       //                                           //                                     SM_USE_DEFAULT def value)   (<== recommended in most cases) 
       //                                           SmBrep::m_sThisZoneTol3d = SmTol::SizeEstimateToZoneTol3d(m_dThisModelSizeEstimate) ;
       //   Real Solution TODOs list:
       //                  1. Remove ZoneTol3d, ThisZoneTol3d, ThisModelSizeEstimate from Brep - update all the SmTol methods to fetch the right values.
       //                  2. Work on Context constructor to set m_dThisModelSizeEstimate and m_dThisZoneTol3d values
       //                       2a.  large ranges of ModelSize measures get rounded into a small set of ModelSizeEstimates with call
       //                            SmBrep::m_dThisModelSizeEstimate = (AnyModelSizeMeasure, a User's model size guess, or sys def SM_MODEL_SIZE_ESTIMATE)    
       //                       2b.  The mapping from ModelSizeEstimate to Brep ZoneTol3d values is roughly:
       //                                ModelSizeEstimate:[1.0e4    - 1.0e5    ] => ZoneTol3d:[1.0e-3]: rare
       //                                ModelSizeEstimate:[100      - 10,000   ] => ZoneTol3d:[1.0e-4]: occasionally
       //                                ModelSizeEstimate:[  1      -    100   ] => ZoneTol3d:[1.0e-5]: default
       //                                ModelSizeEstimate:[  0.01   -      1   ] => ZoneTol3d:[1.0e-6]: occasionally
       //                                ModelSizeEstimate:[  0.0001 -      0.01] => ZoneTol3d:[1.0e-7]: rare
       //                  3. Set Brep::GetThisModelSizeEstimate() = Brep->cpContext->GetThisModelSizeEstimate()
       //                  4. Set Brep::GetThisZoneTol3d()         = Brep->cpContext->GetThisZoneTol3d()
       //                  5. Cache MaxGap sizes on the vertexuses and edgeuses.
       //                  6. modify the classifications and intersectors to look for intersections with large-gap vertex uses and large-gap edge uses.
       //                  7. WorkAround: Until the New model/Old model implementation is cleaned up and Brep::m_sZoneTol3d is removed,
       //                            SmBrep::m_sBrepZoneTol3d = SmBrep::m_sThisZoneTol3d
       
       // The new and old tolerance models have been released simultaneously and need to be filtered down to a single NewTolModel value.
       //  Old TolModel: m_sBrepZoneTol3d is an independent value set by default, through the constructor, or explicitly by a call to SetTolerance()
       //  New TolModel: A SizeEstimate is stored by the Brep->cpContext, from which the following values are cached for convenience
       //                  SmContext::m_dThisModelSizeEstimate  ==>  SmContext::m_dThisZoneTol3d      = SmTol::SizeEstimateToZoneTol3d(SmContext::m_dThisModelSizeEstimate) ;
       //                                                       ==>  SmBrep::GetThisModelSizeEstimate() = pBrep->SmContext->GetThisModelSizeEstimate()
       //                                                       ==>  SmBrep::GetThisZoneTol3d()         = pBrep->SmContext->GetThisZoneTol3d()
       //                Seting SmContext::m_dThisModelSizeEstimate from Default           : SM_USE_DEFAULT ==> SM_MODEL_SIZE_ESTIMATE
       //                                                           From ModelSize measure : BrepSize Measure 
       //                                                           From User estimate     : BrepSize UserEstimate 
  // } end long temporary note on NewTolerance model

  SM_NEWTOL_LINE double m_dThisModelSizeEstimate ;    // To be moved to SmContext in New Tolerance model           
  SM_NEWTOL_LINE double m_dThisLargeSmallSizeRatio ;  // To be moved to SmContext in New Tolerance model           
  // SM_OLDTOL_LINE SmZoneTol3d m_sBrepZoneTol3d ;       // Old tolerance model - to be removed in New Tolerance model
  SM_NEWTOL_LINE SmZoneTol3d m_sThisZoneTol3d ;       // To be moved to SmContext in New Tolerance model           

  SmRegion             * m_pInfiniteRegion = NULL;  // infinite/outer region
  SmOwningTopology     * m_pFaceListHead   = NULL;  // Object heading Face list
  SmOwningTopology     * m_pEdgeListHead   = NULL;  // Object heading Edge list
  SmOwningTopology     * m_pVertexListHead = NULL;  // Object heading Vertex list

  SmTArray<SmSurface*> * m_pDeletedSurfaces = NULL; // Contains surfaces which have been deleted from
                                                    // the Brep. We need to keep these around until after
                                                    // operations done because they may be referenced.
#ifdef SM_DEBUG_CACHE_H
public:
  ULONG                  m_lBrepCacheCount ;
#endif // SM_DEBUG_CACHE_H

private:
  SmExtent3d           * m_pLockedBoundingBox;     // pointer to locked bounding box
public:

  // empty constructor
  SmBrep(double      dThisModelSizeEstimate  =SM_USE_DEFAULT,   // in : ModelSizeEstimate for Tols, don't change unless you're an expert
         double      dThisLargeSmallSizeRatio=SM_USE_DEFAULT,   // in : for Pinholes in Battleships Tols, don't change unless you're an expert
         SmContext * pOptContextForStack        = NULL) ;       // in : context for new object construction, NULL to use default context

  // copy constructor
  SmBrep(const SmBrep  & crBrepToCopy,                     // in : target Brep to copy
         SmCopyBrepMap * pOptCopyBrepMap = NULL,           // out: A map from copyBrep topology Obj ptrs to orig ptrs
                                                           //    : NULL to ignore, default:[NULL]
         SmCopyBrepMap * pOptReverseMap = NULL,            // out: A map from orig topology Obj ptrs to copyBrep ptrs
                                                           //       NULL to ignore, default:[NULL]
         SmBoolean       bCopyAttributes = TRUE,          // NotUsed: in : If true copy attributes to new brep default:[TRUE]
         SmBoolean       bAddAnalytics = TRUE) ;          // in : TRUE = recognize analytic surfaces; FALSE = preserve surface types. Default:[TRUE]

  // destructor
  virtual ~SmBrep() ;

  void AddDeletedSurface(SmSurface* pSurface) ;

  // get all face/face intersection curves and singular points between two Breps
  SmStatus BrepIntersect(const SmContext     & crContext,         // in : context for new object construction 
                         const SmBrep        * cpOtherBrep,       // in : Other Brep to intersect with this 
                         double                dThisApproxTol3d,  // in : How closely should we approximation the resulting curves 
                         double                dAngleTolDeg,      // in : What is the maximum angle for a span of the resulting curve 
                                                                  //    : in degrees - 20.0 degrees - is usually good. 
                         SmTArray<SmCurve*>  & r3DCurves,         // out: 3D curves produced 
                         SmTArray<SmPoint3d> * pOpt3DPoints)      // out: Optional 3D points produced. 
                        const ;                                   //    : These points are only singular points not points which are at the 
                                                                  //    : create displayList for the end of 3D intersection curve 

  virtual SmStatus BuildGraphicsStructure(const SmDisplayParameters & crDisplayParameters,   // in : graphics controlling parameters                                                      
                                          SmDisplayList             & rDisplayList,          // NotUsed: [out]: displayList name for this Brep Graphics                                              
                                          SmDisplayList             * pActiveDisplayListRef, // out: Ref to DisplayList Copy placed on the s_View.m_pActiveLists that actually gets drawn 
                                          SmGfxArraySet             * pOptGfxSet = NULL) ;   // in : When given output GfxVertexArrays not GL calls.                                      

  // calculate BoundingBox containing all Brep vertices, edges, and faces
  SmStatus CalculateBoundingBox(SmExtent3d & rBrepBBox,        // out: Bounding Box containing Brep 
                                SmBoolean    bApprox = FALSE,  // in : TRUE : output BBox = union(vertex boxes(if very large count) + edge boxes (if smaller)) 
                                                               //    : FALSE: output BBox = union(vertex, edge, and face BBoxes)...very slow on huge breps 
                                                                         //      default:[FALSE]
                                SmBoolean    bTight = FALSE,             // in : TRUE = compute minimal box for each contained face and edge (expensive) 
                                                               //    : FALSE= compute any box larger than each contained face and edge (cheaper) 
                                                                         //      default:[FALSE]
                                SmBoolean    bExpandByZoneTol3d = TRUE)  // in : TRUE = returned BBox = Union(Vertex,Edge,Face BBoxes expanded by their ZoneTol3d values)
                                                                         //    : FALSE= returned BBox = Union(Vertex,Edge,Face BBoxes with no expansions) 
                                                                         //      default:[TRUE] = previous behavior
                               const ;

  // calculate the precise BoundingBox of the Brep.
  SmStatus CalculateTightBoundingBox(SmExtent3d & rBrepBBox) const ;

  // Check for coincident topology in this Brep.
  SmBoolean ContainsCoincidentTopology(double                    dTol    = -1.0,          // in : min dist between distinct points, negative = use Brep tol 
                                       SmTArray< SmTopology* > * pCoinEnts1 = NULL,       // out: List of geometry that has coincident neighbors, NULL to ignore, 
                                       SmTArray< SmTopology* > * pCoinEnts2 = NULL,       // out: associated coincident neighbors of pCoinEnts1, NULL to ignore, 
                                       SmAssertArray           * pAList = NULL,           // in : optional ErrorReport List for every coincident geometry 
                                                                                          //    : Macros use the name 'pAList' - don't change it 
                                       SmAssertTestLevel         eTestLevel = SM_LEVEL_0, // in : SM_LEVEL_0 = fewest tests 
                                                                                          //    : SM_LEVEL_1 
                                                                                          //    : SM_LEVEL_2 = all tests, 
                                                                                          //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                       SmTArray<ULONG>         * pTestRequests = NULL,    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order 
                                       SmBoolean                 bFindAll = FALSE)        // in : TRUE=find all Coincident geometry, FALSE=find first coincident pair and quit 
                                      const ;

  // combine two coincident vertices updating topology graph as needed
  SmStatus CombineCoincidentVertices(SmVertex * pSurvivingVertex,  // in : Pointer to a vertex to combine 
                                     SmVertex * pVertexToDelete,   // in : Pointer to a vertex to combine 
                                     SmEdge  *& rpDeletedEdge) ;   // out: Pointer to the edge that has been deleted 

  // compute Brep area, volume, and moments using analytic techniques
  SmStatus ComputePreciseProperties(double            dRelativeAccuracy,       // in : Relative accuracy between 1.0e-1 and 1.0e-4 appears to be the best. 
                                                                               //    : You can go tighter but it gets really slow if you have complex trim curves. 
                                    const SmPoint3d & crOriginOfComputation,   // in : 
                                    const SmMassPropertyBehavior & crBehavior, // in : The data in this object determines behavior the mass property analysis. 
                                                                               //    : See the definition of the for a description of the controls. 
                                    double                       & rdArea,     // out: 
                                    double                       & rdVolume,   // out: 
                                    double                       & rdMass,     // out: Must specify density with crBehavior above 
                                    SmTArray<SmVector3d>         & rMoments)   // out: [0] Ix, Iy, Iz    - Area static (first) moments:       integral of (x, y, z) dA 
                                   const ;                                     //    : [1] Ixx, Iyy, Izz - Area second moments about the planes:  integral of (x^2, y^2, z^2) dA 
                                                                               //    : [2] Iyz, Izx, Ixy - Area products of inertia:              integral of (yz, zx, xy) dA 
                                                                               //    : [3] Ixx, Iyy, Izz - Area moments of inertia about the axes: integral of (y^2+z^2, x^2+z^2, x^2+y^2) dA 
                                                                               //    : [4] Ix, Iy, Iz    - Mass static (first) moments:       integral of (x, y, z) dm
                                                                               //    : [5] Ixx, Iyy, Izz - Mass second moments about the planes:  integral of (x^2, y^2, z^2) dm
                                                                               //    : [6] Iyz, Izx, Ixy - Mass products of inertia:             integral of (yz, zx, xy) dm
                                                                               //    : [7] Ixx, Iyy, Izz - Mass moments of inertia about the axes: integral of (y^2+z^2, x^2+z^2, x^2+y^2) dm
                                                                               //    : ([4..7] density-weighted (dm); Face / Region layers return geometric dV integrals)

  // Convert 3D and UV curves corresponding to SmLine curve when appropriate.
  SmStatus ConvertCurvesToAnalytics(ULONG & rlNum3DCurvesConverted,   // out: 
                                    ULONG & rlNumUVCurvesConverted) ; // out: 

  // copy face(s) to destination Brep without intersects or stitch
  SmStatus CopyFaces(const SmTArray<SmFace*> & crFacesToCopy,             // in : list of faces to copy from this Brep 
                     SmBrep                  * pDestinationBrep,          // in : target Brep 
                     SmTArray<SmFace*>       * pOptNewFaces = NULL,       // out: new face copies in Destination Brep 
                     SmBoolean                 bUseMerge = TRUE,          // in : TRUE  = use SmMerge - does extra region classifications and topology connections 
                                                                          //        : FALSE = don't use SmMerge, just copy the faces into the InfiniteRegion without intersection or connecting to existing geometry 
                     SmMerge                 * pOptMergeObject = NULL) ;  // out: allows caller access to the maps, etc. 
                                                                          //    : copy edge(s) as wire edges to destination Brep  

  // Copy edges from one brep into the other.                                                    
  SmStatus CopyEdges(const SmTArray<SmEdge*> & crEdgesToCopy,      // in : list of edges to copy 
                     SmBrep                  * pDestinationBrep,   // in : target brep 
                     SmTArray<SmEdge*>       * pNewEdges = NULL) ; // out: new edges copied into DestinationBrep 

  // copy vertices as shell vertices to destination Brep without intersector or stitch
  SmStatus CopyVertices(const SmTArray<SmVertex*> & crVerticesToCopy,      // in : list of vertices to copy 
                        SmBrep                    * pDestinationBrep,      // in : tgt brep 
                        SmTArray<SmVertex*>       * pNewVertices = NULL) ; // out: new vertices copied to DestinationBrep 

  void     ClearAllPolyVertices()  ;

  SmStatus CreateCornerBlend(const SmContext            & crContext,             // in : context for new object construction 
                             const SmTArray<SmEdgeuse*> & crEdgeuses,            // in : Geometrically connected but not ordered set of edgeuses 
                                                                                 //    : to blend.  The first Edgeuse will be the base if there 
                                                                                 //    : are only 3 edgeuses.  Wire edges will have no face 
                                                                                 //    : tangency.  Edgeuses which lie on a face will by default 
                                                                                 //    : be tangent to the new surface(s). 
                             const SmTArray<SmBoolean>  * cpOptTangentToFace,    // in : If given, allows removal of tangency requirement from some of the edges. 
                             double                       dThisApproxTol3d,      // in : Approximation tolerance 
                             double                       dTangencyTolRadians,   // in : Tangency tolerance in radians 
                             SmTArray<SmSurface*>       & rBlendSurfaces)        // out: Only a single surface is returned for 3 and 4 sided blends. 
                            const ;                                              //    : 5 or more sided blends will have one patch for every side. 
      
  // create a ruled or blend surface between two target edges
  static SmStatus CreateEdgeEdgeBlend(const SmContext        & crContext,        // in : context for new object construction 
                                      ULONG                    lBlendFlag,       // in : 0 = ruled, - not a blend surface 
                                                                                 //    : 1 = G1 blend surface, 
                                                                                 //    : 2 = G2 blend surface, 
                                                                                 //    : 3 = G3 blend surface 
                                      const SmEdgeuse        * cpEdgeuse1,       // in : one end of output surface 
                                      const SmEdgeuse        * cpEdgeuse2,       // in : other end of output surface 
                                      ULONG                    lCurvesDirFlag,   // in : 0 = Guess curve directions based on geometry 
                                                                                 //    : 1 = run blend between start of Edge1->Curve to start of Edge2->Curve 
                                                                                 //    : 2 = run blend between start of edge1->Curve to end of Edge2->Curve  
                                      SmDerivSurfDefinition  * pOptDSDef,        // in : optional blend options, NULL to ignore 
                                                                                 //    : NULL = use values hardcoded into this function 
                                      SmSurface             *& rpBlendSurface) ; // out: new ruled or blend surface 

  // create face and shell from surface and UVDomain in infinite region without connecting to existing topology
  SmStatus CreateFaceFromSurface(SmSurface        * pSurface,    // in : target face 
                                 const SmExtent2d & crUVDomain,  // in : desired face Nurb domain 
                                 SmFace          *& rpNewFace) ; // out: New Face connected to this Brep 

  // create face(s) and shell from surface in infinite region after splitting surface at discontinuities.
  SmStatus CreateFacesFromSurface(SmSurface         * pSurface,               // in : Tgt Surface  
                                  const SmExtent2d  & crUVDomain,             // NotUsed: in : desired domain 
                                  SmContinuityType    eOptContinuityToSplit,  // in : oneof:     SM_CT_CO = split face at C0 discontinuities  
                                                                              //             ... SM_CT_C1 = split face at C1 and C0 discontinuities  
                                                                              //             ... SM_CT_C2 = split face at C2, C1, and C0 discontinuities  
                                                                              //             ... and all other SmContinuityType values  
                                  SmTArray<SmFace*> & rNewFaces) ;            // out: 1 or more new faces 

  // convenience function - calls CreateFaceFromSurface() and
  SmStatus CreateEndCapsFromSurface(SmSurface *pSurface,  // in : Surface to be made into a face 
                                    int        dEndCap) ; // in : 0=none.  1 = at min (u,v) 2= at max(u,v)  3 = both 

  // create face and shell from surface and UVDomain in given region without connecting to existing topology
  SmStatus CreateFaceInRegionFromSurface(SmRegion         * pRegionArg,      // in : region to contain new face. NULL for infinite region. 
                                         SmSurface        * pSurfaceArg,     // in : target surface: consumed. 
                                         const SmExtent2d & crUVDomain,      // in : surface Nurb domain for new face, may be a sub-domain of the Surface Natural Trim Domain 
                                         SmShell         *& rpNewShellArg,   // out: New Shell 
                                         SmFace          *& rpNewFaceArg) ;  // out: New face 
                                      
  // create new edge, vertices, and shell without connecting new geometry with existing topology.
  SmStatus CreateWireEdgeFromCurve(SmCurve          * pCurve,          // in : consumed. 
                                   const SmExtent1d & crInterval,      // in : 
                                   SmEdge          *& rpNewWireEdge) ; // out: 

  // Create new faces from a list of curves and the underlying surface
  SmStatus CreateFacesWithCurves(SmRegion                      * pRegionArg,           // in : region to contain NewFace. NULL for infinite region. 
                                 const SmTArray<ULONG>         & crCurveLoopsArg,      // in : 1 entry per loop, 1st Loop=OuterLoop, Number of curves in each loop 
                                 const SmTArray<SmCurve*>      * cp3DCurvesArg,        // in : Ordered curve array 
                                 const SmTArray<SmOrientType>  & crCurveOrientations,  // in : Associated curve orientations 
                                 SmSurface                     * pSurfaceArg,          // in : NewFace target surface. consumed. 
                                 SmShell                      *& rpNewShellArg,        // out: NewShell when adding face splits region 
                                 SmTArray<SmFace*>             & rResultingFaces) ;    // out: NewFace 

  // Sample trimmed edges once; angle must be finite and positive (degrees).
  // Replaces points/counts and borrowed edges; leaves geometry unchanged.
  // Missing/failed curves: zero counts and SM_ERR; valid samples are retained.
  // Degenerate curves retain one point.
  SmStatus TessellateBoundaries(double dAngleTolDeg,
                               SmTArray<SmPoint3d>& rPoints,
                               SmTArray<ULONG>& rVertexCounts,
                               SmTArray<SmEdge*>& rEdges);

  // Build a PolyBrep from a Brep model with optionally specified tessellation parameters
  SmStatus ConvertToPolyBrep(SmPolyBrep     *& rpNewPolyBrep,               // out: the tessellation as a PolyBrep model                                                                                     
                             SmBoolean       & rbFailedFaces,               // out: TRUE = some Brep->Face failed to tessellate, FALSE = okay                                                                
                             ULONG           & rlNumLamina,                 // out: number of lamina edges.  When there are none then                                                                        
                                                                            //    : the output is a nice closed poly brep.                                                                                   
                             double            dCHTol = 0.0,                // in : dCHTol = Max dist3D between geom and tess segment, 0=ignore                                                              
                                                                            //    :          max allowed Element ControlPoint to BasePlane3d dist3d,      0.0 = ignore                                       
                             double            dCrvTessAngle = 25.0,        // in : dCrvTessAngle=Max angDeg between tess tangents,         0=ignore                                                         
                             double            dSrfTessAngle = 25.0,        // in : dSrfTessAngle=max allowed Element ControlPolygon turning ang3d (degrees),  0.0 = ignore                                  
                             double            dMax3DEdge = 0.0,            // in : dMax3DEdge   =Max dist3D between tess pts,              0=ignore                                                         
                                                                            //    :          max allowed Element BasePolygon side length3d,               0.0 = ignore                                       
                             double            dMaxAspect = 0.0,            // in : dMaxAspect   =max allowed Element BasePolygon aspect ratio3d,              0.0 = ignore                                  
                             SmBoolean         bAdvancingFront = FALSE,     // in : bAdvancingFront, FALSE = SmCurve::TessellateByBisection()                                                                
                                                                            //    :               if that fails - tessellate by SmCurveCache spatial decomposition                                           
                                                                            //    :               then in AddQuadBoundaries calls TriangulateFace(), many subdivision side effects                           
                                                                            //    :             TRUE  = SmBSplineCurve::EquallySpacedPoints()                                                                
                                                                            //    :               then in AddQuadBoundaries calls TriangulateFaceAF(), many subdivision side effects                         
                             SmBoolean         bSmoothResults = FALSE,      // in : bSmoothResults=select sTess.m_vSmoothingData params for SmTess::SmoothPolygons() passed to SmPolyBrep::SmoothPolygons()  
                                                                            //    : TRUE = do some smoothing - SmSmoothingData.m_lSmoothingPasses(8)                                                         
                                                                            //    :                       SmSmoothingData.m_dSmoothingStepSize(0.25)                                                         
                                                                            //    :                       SmSmoothingData.m_dMinSmoothingRatio(3.0)                                                          
                                                                            //    :                       SmSmoothingData.m_lSmoothingTechnique(1)                                                           
                                                                            //    : FALSE= don't  - SmSmoothingData.m_lSmoothingPasses(0)                                                                    
                                                                            //    :            SmSmoothingData.m_dSmoothingStepSize(0.0)                                                                     
                                                                            //    :            SmSmoothingData.m_dMinSmoothingRatio(1.0)                                                                     
                                                                            //    :            SmSmoothingData.m_lSmoothingTechnique(1)                                                                      
                             SmBoolean         bPropagateAttribs = FALSE,   // in : TRUE = Propagate attributes from Brep Faces to PolyBrep Faces                                                            
                                                                            //    : FALSE= don't, Note: only supported when compiled with SM_INDEXING
                             SmBoolean         bCreateUniqueContext = FALSE, // in : TRUE = Create new context for resulting SmPolyBrep. Caller responsible for managing context memory
                                                                            //    : FALSE= Use input SmBrep context (classic behavior)
                             SmTArray<SmTessellationFailure>* pFailures = NULL, // out: Replaced with first failure per original face, including on error
                             SmTessellationReport* pReport = NULL // out: Nonfatal statistics and face failures
                             );


  // create Brep from PolyBrep - stitch the whole thing together
  static SmStatus CreateFromPolyBrep(const SmContext  & crContext,   // in : 
                                     const SmPolyBrep & crPolyBrep,  // in : 
                                     SmBrep          *& rpBrep) ;    // out: 

  // create Brep from triangle bag - stitch the whole thing together
  static SmStatus CreateFromTriangleBag(const SmContext     & crContext,      // in : 
                                        const SmTriangleBag & crTriangleBag,  // in : 
                                        SmBrep             *& rpBrep) ;       // out: 

  static SmStatus CreateFromCrossCurves(const SmContext            & crContext,             // in : 
                                        SmApproxTol3d                dBrepTolerance,        // in : 
                                        const SmTArray<SmCurve*>   & crBaseCurves,          // in : 
                                        const SmVector3d           & crBaseVector,          // in : 
                                        const SmTArray<SmCurve*>   & crSideCurves,          // in : 
                                        const SmTArray<SmVector3d> & crSideCurveVectors,    // in : 
                                        SmBrep                    *& rpNewBrep) ;           // out: 

  SmStatus CreateNormalProjectionCurves(const SmContext      & crContext,             // in : context for new object construction 
                                        const SmBSplineCurve & crCurveToProject,      // in : Target Curve close to target faces 
                                        SmTArray<SmFace*>    & crFaces,               // in : List of target faces 
                                        SmApproxTol3d          dApproxTol3d,          // in : max allowed distance to project curve to faces and 
                                                                                      //    : max allowed deviation of projected curve to TRUE projection shape (tighter tol = more output knots) 
                                        SmTArray<SmCurve*>   * pOpt3DProjectedCurves, // out: 3D projected curve shapes, NULL to ignore 
                                        SmTArray<SmCurve*>   * pOptUVProjectedCurves, // out: UVTrimCurve projected curve shapes, NULL to ignore 
                                        SmTArray<SmFace*>    * pOptFaceOfUVCurves)    // out: associated face for each output UVTrimCurve, NULL to ignore 
                                       const ;

  SmStatus CreateParallelProjectionCurves(const SmContext      & crContext,                 // in : 
                                          const SmBSplineCurve & crCurveToProject,          // in : 
                                          const SmVector3d     & crProjectionVector,        // in : 
                                          const double         * cpdOptProjectionDistance,  // in : 
                                          const SmApproxTol3d  * cpdOpt3DApproximationTol,  // in : 
                                          const double         * cpdOptAngleTolRadians,     // in : 
                                          SmTArray<SmCurve*>   * pOpt3DProjectedCurves,     // out: 
                                          SmTArray<SmCurve*>   * pOptUVProjectedCurves,     // out: 
                                          SmTArray<SmFace*>    * pOptFaceOfUVCurves,        // out: 
                                          SmTArray<SmFace*>    * pOptSubsetFaces = NULL)    // out: 
                                         const ;

  // create bounded face(s) for unordered closed planar curve loops in given region
  // without connecting to existing topology or testing region containment
  SmStatus CreatePlanarFacesWith3DCurves(SmRegion                 * pRegionArg,      // in : region to contain newFace-not checked. NULL=infinite region. 
                                         const SmTArray<SmCurve*> & cr3DCurvesArg,   // in : planar curves to bound newFace. Consumed when successful. 
                                         double                     d3DToleranceArg, // NotUsed: in : 3D distance to check for planarity, degenerate curves and distinct points ; 
                                         SmTArray<SmFace*>        & rNewFaces) ;     // out: array of newFaces 

  // same as CreatePlanarFacesWith3DCurves() when only one face is being built
  SmStatus CreatePlanarFaceWith3DCurves(SmRegion                 * pRegionArg,      // in : region to contain newFace - not checked 
                                                                                    //    : NULL for infinite region. 
                                        const SmTArray<SmCurve*> & cr3DCurvesArg,   // in : array of planar curves to bound newFace - curves used by NewEdges 
                                        double                     d3DToleranceArg, // in : 3d Distance for planarity checks 
                                        SmFace                  *& rpNewFaceArg) ;  // out: new face 

  // create Brep crossSection curves by intersecting Brep with an infinite plane
  SmStatus CreatePlanarSectionCurves(const SmContext     & crContext,                  // in : context for new object construction 
                                     const SmPoint3d     & crPlanePoint,               // in : point on plane 
                                     const SmVector3d    & crPlaneNormal,              // in : plane's normal 
                                     const SmApproxTol3d * cpdOpt3DApproximationTol,   // in : max allowed distance between 3D SectionCurves and Surface 
                                     const double        * cpdOptAngleTolRadians,      // in : 
                                     SmTArray<SmCurve*>  * pOpt3DSectionCurves,        // out: 3d Planar curves 
                                     SmTArray<SmCurve*>  * pOptUVSectionCurves,        // out: 2d UVTrimCurves for each 3D section curve 
                                     SmTArray<SmFace*>   * pOptFaceOfUVCurves,         // out: list of faces, one for each u,v curves where uv applies 
                                     SmBoolean             bForceUVClassify = FALSE)   // in : TRUE = For those intersectors that don't produce a UVTrimCurve 
                                    const ;                                            //    : Project the intersection curve to Surface to get UVCurve 
                                                                                       //    : Replace 3DIntersection curve with UVTrimCurve 3D approximation 
                                                                                       //    : remove edge from topoogy graph and delete it and 
                                                                                       //    : FALSE= Set output UVSectionCurves to NULL for thoseits geometry, leaving bounding vertices behind 
                                                                                       //    : intersectors which don't produce UVTrimCurves 
                                                                                       //    : note: only used when pOptUVSectionCurves != NULL 

  // Create new Breps, one for each shell in the infinite region
  // Ownership of prBrepToSeparate:
  //   - one body: it is returned unchanged as sResultingBreps[0]; the pointer is left as is.
  //   - several bodies: it is deleted and prBrepToSeparate is set to NULL.
  //   - no bodies: it is deleted, prBrepToSeparate is set to NULL, sResultingBreps is
  //     empty, and the call still returns SM_SUCCESS.
  //   - on an error return the input may still be valid and may also appear in
  //     sResultingBreps; check both before deleting anything.
  static SmStatus CreateOneBrepPerBody
  (
      SmBrep           *& prBrepToSeparate,   ///< [in] : Brep to separate                                <br>
      SmTArray<SmBrep*> & sResultingBreps,    ///< [out]: New Breps                                       <br> 
      SmBoolean           bNewContexts=FALSE  ///< [in] : TRUE=Create new SmContexts for each new Brep    <br>
                                              ///<      : FALSE=Use this->GetContext() for all new Breps  <br>
                                              ///<      : default: [FALSE]                                <br>    
  ); 

  // cut this Brep with Breps in sBreps. Result = list of 1 Brep per post-cut connected face set from this and cut breps                   
  SmStatus CutBreps(const SmContext    & crContext,         // in : context for new object construction
                    SmTArray<SmBrep *> & sBreps,            // in : Input brep used to cut. These are all consumed here
                    ULONG                lCutOperation,     // NotUsed: in : 0 - Cut Faces,  1 - Cut Volumes
                    SmBoolean            bKeepOriginal,     // in : TRUE  = Add Merge SM_BO_EXTRACT_SEPARATE m_vOneBrepPer_OrigBrepConnectedFaceSet Breps to output
                                                            //      FALSE = don't
                    SmBoolean            bKeepOperands,     // in : TRUE  = Add Merge SM_BO_EXTRACT_SEPARATE m_vOneBrepPer_OtherBrepConnectedFaceSet Breps to output
                                                            //      FALSE = don't
                    SmTArray<SmBrep *> & rResultingBreps) ; // out: list of 1 Brep per post-cut connected face set

  SmStatus DeleteEdge(SmEdge    * pEdgeToDelete,
                      SmSurface * pOptSurfaceToKeep=NULL) ;

  // remove face from topoogy graph and delete it and its geometry, leaving bounding edges and vertices behind
  SmStatus DeleteFace(SmFace                 * pFaceToDelete,                   // in : target face 
                      SmBoolean                bDeleteEdgesVertices = FALSE,    // in : TRUE = delete connected edges and vertices, 
                                                                                //    : if they're not connected to anything else 
                                                                                //    : (i.e., if they become wire edges or shell 
                                                                                //    : vertices after the face is removed). 
                      SmBoolean                bDoRegionNesting = TRUE,         // in : TRUE = combine regions/shells as appropriate 
                                                                                //    : default:[TRUE] 
                      SmTArray<SmTopology *> * pOptDeletedEdgesVertices = NULL, // out: when bDeleteEdgesVertices == TRUE, 
                                                                                //    : all connected edges and vertics that were deleted. 
                                                                                //    : These are all stale ptrs - don't reference through them 
                      SmRegion               * pOptRegionToKeep = NULL) ;       // in : Opt Region kept when bDoRegionNesting = TRUE 

  // replaces a fillet face with a sharp edge in topology graph ad deletes fillet face and geometry
  SmStatus DeleteFilletFace(SmFace * pFaceToDelete,    // in : 
                            double * pd3dTol = NULL) ; // in : 

// Remove Composites
// // remove CEdge from topology graph and deletes it
//    SmStatus DeleteCEdge(SmCEdge *pCEdgeToDelete) ;

// Remove Composites
// // remove CFace from topology graph and deletes it
//    SmStatus DeleteCFace(SmCFace *pCFaceToDelete) ;

  // remove a vertex which splits two tangent edges and join the edges
  SmStatus DeleteTopologicalVertex(SmVertex * pVertexToDelete,      // in : vertex to remove if possible 
                                   SmEdge   * pEdgeToDelete=NULL) ; // in : if Null: use the shorter of our two edges. 

  // remove a vertex that is not connected to any edges (shell or loop vertex)
  SmStatus DeleteVertex(SmVertex *pVertexToDelete) ;

  // extend the domain of all surfaces a small amount in all directions
  SmStatus ExtendSurfaces(SmContinuityType eExtensionContinuity,    // in : 
                          double           dExtendDistance = 0.0) ; // in : 

  // find the infinite region of a Brep
  SmStatus FindInfiniteRegion(SmRegion  *& rpNewInfRegion,       // out: found infinite region 
                              SmShell    * pOptShellToTest=NULL, // in, opt: region created by splitting oldInfiniteRegion to test
                              SmExtent3d * pOptBrepBBox = NULL)  // in, opt: precomputed bbox for brep.
                              const ;

  // reset the infinite region to the given region, including all bookkeeping - increments an unused Mark value
  SmStatus ResetInfiniteRegion(SmRegion           * pNewInfRegion,                                  // in : infinite region to be reset 
                               SmSetRegionVoidType  eVoidSettingFlag = SM_VS_INFINITEREGION_SWAP) ; // in : option for setting region IsVoid values after changing the infinite region 
                                                                                                    //    : SM_VS_NESTEDSOLID = Make a nested solid model. 
                                                                                                    //    :            see SetRegionIsVoidFlagsForNestedSolids(). 
                                                                                                    //    : SM_VS_INFINITEREGION_ONLY = Just set new infinite region IsVoid value to TRUE and 
                                                                                                    //    :     leave all other regions alone. 
                                                                                                    //    : SM_VS_INFINITEREGION_SWAP = old behavior 
                                                                                                    //    :    set old infinite-region->IsVoid = new infinite-region->IsVoid() value 
                                                                                                    //    :    set new infinite-region->IsVoid = TRUE. 
                                                                                                    //    : default:[SM_VS_NESTEDSOLID] 

  // find and set the infinite region of a closed solid Brep model - increments an unused Mark value
  SmStatus FindAndSetInfiniteRegion(SmShell            * pOptShellToTest=NULL,                           // in : region created by splitting oldInfiniteRegion to test 
                                    SmSetRegionVoidType  eVoidSettingFlag = SM_VS_INFINITEREGION_SWAP,   // in : option for setting region IsVoid values after changing the infinite region 
                                                                                                         //    : SM_VS_NESTEDSOLID         = Make a nested solid model. 
                                                                                                         //    :                             see SetRegionIsVoidFlagsForNestedSolids(). 
                                                                                                         //    : SM_VS_INFINITEREGION_ONLY = Just set new infinite region IsVoid value to TRUE and 
                                                                                                         //    :                             leave all other regions alone. 
                                                                                                         //    : SM_VS_INFINITEREGION_SWAP = old behavior, set 
                                                                                                         //    :                             OldInfiniteRegion->IsVoid = newInfiniteRegion->IsVoid() 
                                                                                                         //    :                             NewInfiniteRegion->IsVoid = TRUE.
                                    SmExtent3d          * pOptBrepBBox = NULL); 
  // :--------------------------------------------------------------------------------------------:
  // :                          UpDim, DownDim, and SameDim Gaps                                  :
  // :----------+--------------------------+---------------------------+--------------------------+            
  // : Topology : UpDim Gap types          : DownDim Gap types         : SameDim Gap types        :      
  // :----------+--------------------------+---------------------------+--------------------------+       
  // :  Vertex  : Vertex/Edge, Vertex/Face : none                      : none                     :       
  // :  Edge    : Edge/Face                : Vertex/Edge               : none                     :       
  // :  Face    : none                     : Vertex/Face, Edge/Face    : EveryLoop: MaxLoopGap    :       
  // :  Loop    : none                     : none                      : EdgeEnd/EdgeEnd LoopGaps :       
  // :----------+--------------------------+---------------------------+--------------------------+   
  //        MaxGap3d     = Max( Vertex/Edge, Vertex/Face, Edge/Edge, Edge/Face Gaps ) 
  virtual const SmGap * GetMaxGap3d       (SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const ;

  // healing function - refine vertex locations and edge shapes from surface intersections when possible
  SmStatus RefineGeometry(SmTArray<SmEdge*>   & rEdgesWithGaps,    // in : edges to refine 
                          SmTArray<SmVertex*> & rVerticesWithGaps, // in : vertices to refine 
                          SmBoolean           & bMadeChange) ;     // out: TRUE = one or more object shapes were modified 
                                                                   //        FALSE= no object shapes were modified 

  // for parallelization of UV trim curve construction

  #ifdef SM_USE_TBB
  SmStatus CreateUVTrimCurvesParallel() ;
  #endif

  // healing function - refresh existing UVTrimCurves
  SmStatus RebuildUVTrimCurves() ;   

  // healing function - refresh all SmEdgeuse and SmVertexuse gap values
  SmStatus RefreshGaps() ;

  // healing function - refresh all cached tolerance values to SmTol::GetZoneTol3d values
  SmStatus RefreshTols() ;

  // healing function - set IsVoid flags to commonly used 'nested solid' state.
  // If model isn't a nested solid don't use this convenience function, set IsVoid flags as needed.
  SmStatus SetRegionIsVoidFlagsForNestedSolids() ;  // in : crements an unused Mark value 

  // simple data access

  // NewTolerance - A Brep has no MinZoneTol3d values only vertices, edges, and faces do.
  //                calling SmTol::GetZoneTol3d(pBrep) returns the SmContext::GetZoneTol3d() value.

  // OldTolerance - A Brep stores a MinZoneTol3d value for all its contained Faces, Edges, and Vertices.
  //                SmBrep::SetTolerance(sZoneTol3d) changes the SmContext::ZoneTol3d value and
  //                   refreshes all Face, Edge, and Vertex m_sZoneTol3d to the SmTol::GetZoneTol3d(pObj) values.
  SM_OLDTOL_LINE virtual SmZoneTol3d GetTolerance() const { return(m_sThisZoneTol3d) ; } // was return(m_sBrepZoneTol3d) ; 
  SM_OLDTOL_LINE virtual void        SetTolerance(SmZoneTol3d sNewZoneTol3d,             // in :
                                                  SmBoolean bUpdateOnlyIfLarger=TRUE,    // in :
                                                  SmBoolean bCascadeToBndries=TRUE) ;  // NotUsed: in :
  #ifndef SM_DEBUG_CODE
    protected: // Tolerance Management
  #else  // with SM_DEBUG_CODE
    public:
  #endif // with SM_DEBUG_CODE
  // public access to Tol values: SmTol::GetZoneTol3d(pBrep)            (don't change vals unless you're an expert)
  //                              SmTol::GetModelSizeEstimate(pBrep)    SmTol::SetModelSizeEstimate(pBrep, NewVal)
  //                              SmTol::GetLargeSmallSizeRatio(pBrep)  SmTol::SetLargeSmallSizeRatio(pBrep, NewVal)
// #ifdef SM_USE_NEWTOL
  SmZoneTol3d GetThisZoneTol3d          ()                              const { return m_sThisZoneTol3d ; }                                             
  double      GetThisModelSizeEstimate  (SmBoolean bChaseDefaults=TRUE) const { return   bChaseDefaults==FALSE ? m_dThisModelSizeEstimate                       
                                                                                       : m_dThisModelSizeEstimate == SM_USE_DEFAULT  
                                                                                       ? m_cpContext->GetThisModelSizeEstimate()                 
                                                                                       : m_dThisModelSizeEstimate ;                              
                                                                              }                                                                  
  double      GetThisLargeSmallSizeRatio(SmBoolean bChaseDefaults=TRUE) const { return   bChaseDefaults==FALSE ? m_dThisLargeSmallSizeRatio          
                                                                                       : m_dThisLargeSmallSizeRatio == SM_USE_DEFAULT            
                                                                                       ? m_cpContext->GetThisLargeSmallSizeRatio()                                        
                                                                                       : m_dThisLargeSmallSizeRatio ;                                                     
                                                                              }                                                                                           
  SmZoneTol3d SetThisModelSizeEstimate(double dThisModelSizeEstimate=SM_USE_DEFAULT)                                                         
                                                                              { m_dThisModelSizeEstimate = dThisModelSizeEstimate ;
                                                                                if(m_cpContext) { ((SmContext *)m_cpContext)->SetThisModelSizeEstimate(dThisModelSizeEstimate) ; }
                                                                                m_sThisZoneTol3d         = SmTol::SizeEstimateToZoneTol3d(m_dThisModelSizeEstimate) ;     
                                                                                return(m_sThisZoneTol3d) ;                                                                
                                                                              }                                                                                           
  void        SetThisLargeSmallSizeRatio(double dThisLargeSmallSizeRatio=SM_USE_DEFAULT)                                                     
                                                                              { m_dThisLargeSmallSizeRatio = dThisLargeSmallSizeRatio ;
                                                                                if(m_cpContext) { ((SmContext *)m_cpContext)->SetThisLargeSmallSizeRatio(dThisLargeSmallSizeRatio) ; }
                                                                              }
// #endif // SM_USE_NEWTOL
  public:

  SmRegion       * GetInfiniteRegion     () const  { return m_pInfiniteRegion; }
  void             GetRegions            (SmTArray<SmRegion*>    & rRegions,      ULONG *pOptAttributeId=NULL) const ;
  void             GetShells             (SmTArray<SmShell*>     & rShells,       ULONG *pOptAttributeId=NULL) const ;
  void             GetBreps              (SmTArray<SmBrep*>      & rBreps                                    ) const ;
  void             GetBrepFromRegion     (SmRegion* pRegion, SmBrep*& rBrep                                  ) const ;
  void             GetTopology           (SmTArray<SmTopology*>  & rTopology,     ULONG *pOptAttributeId=NULL) const ;

  void             GetFaces              (SmTArray<SmFace*>      & rFaces,        ULONG *pOptAttributeId=NULL) const ;
  void             GetFaceuses           (SmTArray<SmFaceuse*>   & rFaceuses,     ULONG *pOptAttributeId=NULL) const ;
  void             GetLoops              (SmTArray<SmLoop*>      & rLoops,        ULONG *pOptAttributeId=NULL) const ;
// Remove Composites
//  static void      GetFacesOfSurface     (const SmSurface        * pSurface,      SmTArray<SmFace*> & rFaces ) ;
// Remove Composites
//  void             GetCFaces             (SmTArray<SmCFace*>     & rCFaces,       ULONG *pOptAttributeId=NULL) const ;

  void             GetEdges              (SmTArray<SmEdge*>      & rEdges,        ULONG *pOptAttributeId=NULL) const ;
// Remove Composites
//  static void      GetEdgesOfCurve       (const SmCurve          * pCurve,        SmTArray<SmEdge*> & rEdges ) ;
  void             GetTopologicalEdges   (SmTArray<SmEdge*>      & rTopoEdges                                ) const ;

// Remove Composites
//  void             GetCEdges             (SmTArray<SmCEdge*>     & rCEdges,       ULONG *pOptAttributeId=NULL) const ;
  void             GetEdgeuses           (SmTArray<SmEdgeuse*>   & rEdgeuses,     ULONG *pOptAttributeId=NULL) const ;
  void             GetWireEdges          (SmTArray<SmEdge*>      & rWireEdges,    ULONG *pOptAttributeId=NULL) const ;
                                                                                                              
  void             GetVertices           (SmTArray<SmVertex*>    & rVertices,     ULONG *pOptAttributeId=NULL) const ;
  void             GetShellVertices      (SmTArray<SmVertex*>    & rShellVertices,ULONG *pOptAttributeId=NULL) const ;
  void             GetTopologicalVertices(SmTArray<SmVertex*>    & rTopoVertices                             ) const ;
  void             GetVertexuses         (SmTArray<SmVertexuse*> & rVertexuses,   ULONG *pOptAttributeId=NULL) const ;
  void             GetSurfaces           (SmTArray<SmSurface*>   & rSurfaces,     ULONG *pOptAttributeId=NULL) const ;
  void             GetCurves             (SmTArray<SmCurve*>     & rCurves,       ULONG *pOptAttributeId=NULL) const ;

  ULONG     GetNumFaces       () const { return m_pFaceListHead->GetSize() ; }
  ULONG     GetNumEdges       () const { return m_pEdgeListHead->GetSize() ; }
  ULONG     GetNumVertices    () const { return m_pVertexListHead->GetSize() ; }
  ULONG     GetTopologyIndex  (SmObject * pTgt) { ULONG lIndx = 99999 ;
                                                  if(pTgt->GetType() == SmFace_TYPE  ) { m_pFaceListHead->IsInList  ((SmTopology *)pTgt, &lIndx) ; }
                                                  if(pTgt->GetType() == SmEdge_TYPE  ) { m_pEdgeListHead->IsInList  ((SmTopology *)pTgt, &lIndx) ; }
                                                  if(pTgt->GetType() == SmVertex_TYPE) { m_pVertexListHead->IsInList((SmTopology *)pTgt, &lIndx) ; }
                                                  return lIndx ; 
                                                }

  // Returns TRUE when pTgt is currently a live Face, Edge, or Vertex member of this brep.
  //
  // Safe to call on a stale or already-freed pointer: it only compares pointer identity against this
  // brep's live topology lists and never dereferences pTgt (unlike GetTopologyIndex, which calls
  // pTgt->GetType()). Use this to guard against dangling entries left in change-tracking add lists when
  // a heal op frees temporary topology outside the tracked window.
  SmBoolean IsLiveTopologyMember(const SmObject * pTgt) const
                                                { if(pTgt == NULL) { return FALSE ; }
                                                  return    m_pFaceListHead  ->IsInList((const SmTopology *)pTgt)
                                                         || m_pEdgeListHead  ->IsInList((const SmTopology *)pTgt)
                                                         || m_pVertexListHead->IsInList((const SmTopology *)pTgt) ;
                                                }
// Remove Composites
//  ULONG     GetNumCFaces      () const { return m_pCFaceListHead->GetSize() ; }
//  ULONG     GetNumCEdges      () const { return m_pCEdgeListHead->GetSize() ; }

  TCHAR*    GetName           ()       { return m_pName; }
  void      SetName           (TCHAR& rName) { m_pName = &rName; }

  // return size and ptr to edge with the smallest bounding box diagonal
  void      GetMinEdgeBBoxSize(double & rdSize, SmEdge * &rpEdge )   const ;
// Remove Composites
//   SmBoolean GetMakeComposites () const  { return( m_bMakeComposites ) ; }

  // get every vertex->Point, curve->ControlPoint, and Surface->ControlPoint
  SmStatus  GetDeformPoints   (SmTArray<SmPoint3d>  & rDeformPoints) ;

  // Combine topology of two edges having common vertices but no common faces which need to be destroyed. limited to lamina edges.
  SmStatus  GlueEdges(SmEdge       * pSurvivingEdge,        // in : 
                      SmOrientType   eRelativeOrientation,  // in : 
                      double         dEdgeGap,              // in : 
                      SmEdge       * pEdgeToDelete) ;       // out: 

  // Combine topology of two edges having common vertices but no common faces which need to be destroyed. For all edge types.
  SmStatus GlueEdgesGeneral(SmEdge               * pSurvivingEdgeArg,        // in : edge to survive after merge 
                            SmOrientType           eRelativeOrientation,     // in : SM_OT_SAME     = edge->Curves have same relative orientation or 
                                                                             //    : SM_OT_OPPOSITE = edge->Curves are opposite to each other. 
                            double                 dEdgeGap,                 // in : Gap between the two edge->curves. 
                            SmBoolean              bDoRegionNesting,         // in : TRUE = worry about region stuff 
                                                                             //    : FALSE= don't 
                            SmEdge               * pEdgeToDeleteArg,         // in : edge to delete after merge 
                            SmTArray<SmFaceuse*> * pInsideFaceuses = NULL) ; // in : LocalMerge 

  // Combine topology of two vertices close enough to be one with no common edge between them: delete one, save the other.
  SmStatus GlueVertices(SmVertex                 * pSurvivingVertex,             // in : Vertex to survive 
                        SmVertex                 * pVertexToDelete,              // in : Vertex to delete after removing from topology graph 
                        SmTArray<SmEdge*>        * pOptWireEdgesInBrep = NULL) ; // in : optional list of wire edges in the pVertexToDelete Shell 
                                                                                 //    : to check for shell ownership when glueing these vertices 
                                                                                 //    : causes two shells to be merged. 
                                                                                 //    : If null, all shell->Brep edges are checked. 

  SmBoolean IsNonManifoldSolid   ( )                                const ; // at least one spineEdge, any number lamina & manifold edges, no VertexShells, no WireEdges
  SmBoolean IsManifoldSolid      ( )                                const ; // all edges connect to just two faces
  SmBoolean IsSheet              ( )                                const ; // all edges are manifold or lamina - just one shell
  SmBoolean IsOrientedSheet      ( )                                const ; // is sheet and all faces share common up/down orientation
  SmBoolean IsWireFrame          ( )                                const ; // only edges, no faces, no ShellVertices, one region
  SmBoolean IsPointSet           ( )                                const ; // only vertex shells, one region
  SmBoolean IsMixedModel         ( )                                const ; // any mix of the above model types
  SmBoolean IsEmptyModel         ( )                                const ; // Brep has no geometry
  SmBoolean IsAnalytic           ( )                                const ; // all faces are represented with analytic surfaces
  SmBoolean IsZoneTol3dConsistent(SmZoneTol3d * pOptZoneTol3d=NULL) const ; // TRUE = SM_ARE_SAME(m_sThisZoneTol3d == SmTol::GetZoneTol3d(this,0,TRUE))
  SmBoolean IsDegenerate         (double d3DTol=SM_EFF_ZERO)        const ; // TRUE = all geometries fit within BBox.MaxDim < d3DTol

  void Index(ULONG   lIndex,
             ULONG   lMaterialOffset) ;

  // classify a ray interval (CurveStart-to-XSectPt or CurveEnd-to-XSectPt) after finding ray's XSect with a Brep          
  SmStatus LocalCurveSector(const SmCurve               & crCurve,               // in : curve to classify 
                            const SmExtent1d            & crInterval,            // in : curve domain extent 
                            SmOrientType                  eOrientation,          // in : SM_OT_SAME     = classify start of curve moving in pos curve direction to XSect Pt 
                                                                                 //       : SM_OT_OPPOSITE = classify end of curve moving in neg curve direction to XSect Pt 
                            const SmPointClassification & crIntersectionOnBrep,  // in : known curve intersection point classification 
                            SmRegion                   *& rpRegionOfSector)      // out: region containing segment of curve classified 
                           const ;

  // replace face(s) surface(s) with new surface(s).
  SmStatus LocalOperation(SmTArray<SmFace*>    & rFaces,         // in : array of faces to get new surfaces 
                          SmTArray<SmSurface*> & rNewSurfaces) ; // in : array of associated new surfaces 

  // add an edge to an existing Face between existing Vertices given a Curve and its Interval range.
  virtual SmStatus MakeEdgeInFace(SmFace           * pFace,             // in : Face where new edge goes 
                                  SmVertex         * pStartVertex,      // in : Edge starts at this vertex 
                                  SmVertex         * pEndVertex,        // in : Edge ends at this vertex. StartVertex == EndVertex for closed curves. 
                                  SmCurve          * p3DCurve,          // in : 3D Geometry of edge 
                                  SmBSplineCurve   * pOptUVCurve,       // in : opt 2D Geometry of edge in UV of face, NULL to ignore 
                                  const SmExtent1d & crInterval,        // in : 3DCurve interval to use for edge. 
                                  SmOrientType       eCurveOrientation, // in : one of: SM_OT_SAME, SM_OT_OPPOSITE 
                                  double             dMaxGap3d,         // in : old: largest of (Brep Tol, Curve Tol (if intersected), Curve/Face max dist) 
                                                                        //    : new: MaxGap3d between pOptUVCurve and p3DCurve, not used when pOptUVCurve==NULL 
                                  SmEdge          *& rpNewEdge,         // out: Newly created manifold edge 
                                  SmLoop          *& rpNewLoop,         // out: New loop - created when vertices lie on existing loop and loop is split 
                                  SmFace          *& rpNewFace) ;       // out: New face - created when vertices lie on existing loop and face is split 

  // compute either a tight or loose bounding box, save it, and lock it
  SmStatus LockBoundingBox(SmBoolean bTight = FALSE) ;

  // unlock current bounding box
  void UnLockBoundingBox () ;

  // called by MakeEdgeInFace() after determining which Vertexuses
  // will bound the new Vertexuses of the new edge.
  virtual SmStatus MakeEdgeInFaceWithVU(SmFace           * pFace,                         // in : Face where new edge goes 
                                        SmVertexuse      * pStartVertexUse,               // in : Edge starts at this vertex 
                                        SmVertexuse      * pEndVertexUse,                 // in : Edge ends at this vertex. 
                                                                                          //    : The start and end vertex may be the same if the curve 
                                                                                          //    : is closed. 
                                        SmCurve          * p3DCurve,                      // in : 3D Geometry of edge - required 
                                        SmBSplineCurve   * pOptUVTrimCurve,               // in : 2D Geometry of edge in UV of face - optional 
                                                                                          //    : Consumed by this method -- do not delete. 
                                        const SmExtent1d & crInterval,                    // in : Interval of 3D curve which corresponds 
                                                                                          //    : to the start and end of the edge. 
                                        SmOrientType       eCurveOrientation,             // in : Orientation of the edge relative to the geometry.  If the 
                                                                                          //    : curve goes from pEndVertex to pStartVertex the orientation is 
                                                                                          //    : opposite. 
                                        SmZoneTol3d        sCurveZoneTol3d,               // in : old: Tolerance used to create curve associated with this edge or the distance  
                                                                                          //    : between the edge and surface of face which ever is largest. 
                                        SmEdge          *& rpNewEdge,                     // out: Always created new manifold edge 
                                        SmLoop          *& rpNewLoop,                     // out: New loop - created in cases where both 
                                                                                          //    : vertices lie on same loop 
                                        SmFace          *& rpNewFace,                     // out: New face - created in cases where both vertices lie on same loop and face is split 
                                        SmDropCurveFail  * pOptDropCurveFail = NULL) ;    // in : Optional state data when pOptUVTrimCurve was made 
                                                                                          //    : Consumed by this method -- do not delete. 

  // make edge between given vertices in given region - if two shells are merged, deletes the EndVertex shell
  SmStatus MakeEdgeTopology(SmRegion * pRegion,       // in : region to contain new edge 
                                                      //    : NULL for infinite region. 
                            SmVertex * pStartVertex,  // in : vertex to connect to edge start 
                            SmVertex * pEndVertex,    // in : vertex to connect to edge end 
                            SmEdge  *& rpNewEdge) ;   // out: new edge 

  // make edge and endVertex connected to a existing startVertex in given region
  SmStatus MakeEdgeVertexTopology(SmRegion  * pRegion,        // in : region to contain new topology 
                                                              //    : NULL for infinite region. 
                                  SmVertex  * pStartVertex,   // in : existing StartVertex in Region 
                                  SmEdge   *& rpNewEdge,      // out: New Edge Pointer 
                                  SmVertex *& rpNewVertex) ;  // out: New Vertex Pointer 

  // make a face given a loop on another face to fix case when given one face with two outer loops.
  SmStatus MakeFaceFromLoopOnFace(SmFace             * pFace,                    // in : The Tgt Face 
                                  SmLoop             * pLoopToMove,              // in : The Tgt Loop to become the OuterLoop of NewFace 
                                  SmFace            *& rpNewFace,                // out: NewFace Created for the LoopToMove 
                                  SmTArray<SmLoop *> * ContainedLoops=NULL,      // in : List of all TgtFace Loops contained within the TgtLoop 
                                  SmBoolean            bCheckVertexLoops=TRUE) ; // in : TRUE = Check VertexLoops for Moves even if ContainedLoops is given 
                                                                                 //        FALSE= VertexLoops are in ContainedLoops, don't check them 

  // create face from polyface in infinite region not connected to existing topology
  SmStatus MakeFaceFromPolyFace(const SmPolyFace * crPolyFace,  // in : 
                                SmFace           * rpBFace) ;   // out:  

  // create face from polyface in infinite region not connected to existing topology
  SmStatus MakeFaceFromPolyFaceFast(const SmPolyFace * crPolyFace,  // in : 
                                    SmFace           * rpBFace) ;   // NotUsed: [out]: 

  // create face given surface, region, and bounding edgeuses connected to existing topology
  SmStatus MakeFaceTopology(SmRegion                        * pRegion,                      // in : target region to receive new Face. NULL for infinite region. 
                            SmSurface                       * pSurface,                     // in : target surface for new Face 
                            SmOrientType                      eSurfaceOrientation,          // in : SM_OT_SAME     = OuterLoop runs CounterClockwise on Surface around  pSurface->Normal 
                                                                                            //    : SM_OT_OPPOSITE = OuterLoop runs CounterClockwise on Surface around -pSurface->Normal 
                            const SmTArray<ULONG>           & crEdgeLoops,                  // in : number of edgeuses in each loop. One entry per Loop, OuterLoop 1st entry. 
                            const SmTArray<SmEdgeuse*>      & crEdgeuses,                   // in : ordered edgeuse array, each series of crEdgeLoops[i] edgeuses is a loop 
                            const SmTArray<SmVertex*>       & crLoopVertices,               // in : Any vertices to become LoopVertices in this face (not attached to any edges) 
                            SmRegion                       *& rpNewRegion,                  // out: new region when new Face splits an existing region, NULL otherwise 
                            SmShell                        *& rpNewShell,                   // out: new shell when new Face splits an existing region, NULL otherwise 
                            SmFace                         *& rpNewFace,                    // out: new Face 
                            SmBoolean                         bVerifyRadialSectors = FALSE, // in : TRUE = For nonManifold Models - verify the new face is inserted to proper 
                                                                                            //    :      radial sectors on each edge 
                                                                                            //    : FALSE= Save time, the sectors are known to be good, don't check 
                            SmBoolean                         bCalculateTrims = FALSE,      // in : TRUE = attach or create UVTrimCurve to every Edgeuse, FALSE=don't 
                            const SmTArray<SmBSplineCurve*> * cpOptUVCurves = NULL) ;       // in : optional array of UVTrimCurves to assign to NewFace->Edgeuses,  

  // call MakeFaceTopology then if needed, support composite faces and containment for newly created regions - increments an unused Mark value
  SmStatus MakeFace(SmRegion                        * pRegion,                        // in : Region to contain face. Edges and verts must in this region.                      
                                                                                      //    : NULL for infinite region.                                                         
                    const SmTArray<ULONG>           & crEdgeLoops,                    // in : Number of edges in each loop.  One entry per loop, outer loop 1st.                
                    const SmTArray<SmEdgeuse*>      & crEdgeuses,                     // in : New Face edgeuses. ordered:[OuterLoop=counter clockwise, InnerLoop=clockwise]     
                                                                                      //    : relative to surface normal as modified by the surface orientation below.]         
                    const SmTArray<SmVertex*>       & crLoopVertices,                 // in : Any vertices to be made into LoopVertices (not connected to edges) in NewFace     
                    SmSurface                       * pSurface,                       // in : NewFace->Surface. Edge must lie on this surface (to within tolerance).            
                                                                                      //    : If surface has an owner, owner must be a face or composite face in this brep.     
                    const SmExtent2d                & crUVDomain,                     // in : Surface domain of interest for making NewFace.                                    
                    SmOrientType                      eSurfaceOrientation,            // in : SM_OT_SAME     = OuterLoop runs counterclockwise on PSurface to  pSurface->Normal 
                                                                                      //    : SM_OT_OPPOSITE = OuterLoop runs counterclockwise on PSurface to -pSurface->Normal 
                    SmRegion                       *& rpNewRegion,                    // out: If the face splits a region, then this is the newly created region.               
                    SmShell                        *& rpNewShell,                     // out: If the face splits a region, then this is the newly created shell.                
                    SmFace                         *& rpNewFace,                      // out: the newly created face.                                                           
                    SmBoolean                         bVerifyRadialSectors = FALSE,   // in : TRUE = For nonManifold Models - verify the new face is inserted to proper         
                                                                                      //    :      radial sectors on each edge                                                  
                                                                                      //    : FALSE= Save time, the sectors are known to be good, don't check                   
                                                                                      //    : default:[FALSE]                                                                   
                    SmBoolean                         bCalculateTrims = FALSE,        // in : TRUE = attach or create UVTrimCurve to every Edgeuse, FALSE=don't                 
                                                                                      //    : default:[FALSE]                                                                   
                    const SmTArray<SmBSplineCurve*> * cpOptUVCurves = NULL) ;         // in : optional array of UVTrimCurves to assign to NewFace->Edgeuses, default:[NULL]     

    // make face from surface and boundary curves, connects to existing topology when possible
  SmStatus MakeFaceWithCurvesUVHeal(SmRegion                     * pRegion,                    // in : region to contain new topology objects                                                         
                                    const SmTArray<ULONG>        & crCurveLoops,               // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop                                  
                                    SmTArray<SmBSplineCurve*>    * rUVCurves,                  // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts, curves owned by new face   
                                    const SmTArray<SmOrientType> & crCurveOrientations,        // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE                                                                       
                                    SmSurface                    * pSurface,                   // in : new face->Surface                                                                              
                                    const SmExtent2d             & crUVDomain,                 // in : domain of Surface used by face                                                                                                            
                                    SmBoolean                    & rbRunHeal,                  // out: Flag for use in connector code to run heal sequence after face construction.
                                    SmBoolean                    & rbSplitFace,                // out: Flag for use in connector code to split face after face construction.
                                    SmBoolean                    & bPeriodicU,                 // out: Flag for use in connector code, passed to HasSeamProblem.
                                    SmBoolean                    & bPeriodicV);                // out: Flag for use in connector code, passed to HasSeamProblem.

  // make face from surface and boundary curves, connects to existing topology when possible 
  SmStatus MakeFaceWithCurves(SmRegion                         * pRegion,                     // in : region to contain new topology objects                                                         
                              const SmTArray<ULONG>            & crCurveLoops,                // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop                                
                              const SmTArray<SmCurve*>         * cpOpt3DCurves,               // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts, curves owned by new face   
                              const SmTArray<SmBSplineCurve*>  * cpOptUVCurves,               // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts, curves owned by new face   
                              const SmTArray<SmOrientType>     & crCurveOrientations,         // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE                       
                              const SmTArray<SmPoint3d>        & crLoopPoints,                // in : Point positions to build SmVertex VertexLoops                                                  
                              SmSurface                        * pSurface,                    // in : new face->Surface                                                                              
                              const SmExtent2d                 & crUVDomain,                  // in : domain of Surface used by face                                                                 
                              SmOrientType                       eSurfaceOrientation,         // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE                                             
                              SmRegion                        *& rpNewRegion,                 // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids.             
                              SmShell                         *& rpNewShell,                  // out: New shell if any.  Trimmed surfaces always create a new shell.                                 
                              SmFace                          *& rpNewFace,                   // out: the new face                                                                                   
                              const SmBoolean                  * pbOptCrvOnSurf = NULL ) ;    // in : Optional flag to utilize CrvOnSurf for face construction from UV curves. NULL
                                                                                               //      to ignore. Default is NULL.

  SmStatus MakeFaceWithCurvesFast(SmRegion                         * pRegion,                 // in : Topological region in which the new face is to be constructed.                 
                                                                                              //    : NULL for infinite region.                                                      
                                  const SmTArray<ULONG>            & crCurveLoops,            // in : This array describes how the curves are arranged into loops.                   
                                                                                              //    : The size of this array (SmTArray::GetSize) is the number of loops in the       
                                                                                              //    : face we are creating.  The first number in the array defines                   
                                                                                              //    : the number of curves in the outer loop.  If there is more than                 
                                                                                              //    : one loop the remaining loops are inner loops and the count defines             
                                                                                              //    : the number of curves in each corresponding loop.  Also note                    
                                                                                              //    : that the curves are ordered relative to the loops and that there               
                                                                                              //    : should be an equivalent number of curves as the sum of the numbers             
                                                                                              //    : in the crCurveLoops array.                                                     
                                  const SmTArray<SmBSplineCurve*>  & crUVCurves,              // in : If 2D parameter space trimming curves are input, they are in this              
                                                                                              //    : array.  If this parameter is NULL then there must be model space curves.       
                                                                                              //    : If both model space and parameter space curves are input, their                
                                                                                              //    : parameterization must match and their orientations must match.                 
                                  const SmTArray<SmOrientType>     & crCurveOrientatitations, // in : This array defines the orientation of the curves relative to the               
                                                                                              //    : loop that they are in.  If the orientation is the same (SM_OT_SAME) than       
                                                                                              //    : the corresponding curves (cpOpt3DCurves and cpOptUVCurves) are oriented        
                                                                                              //    : the same as the loop.  If opposite (SM_OT_OPPOSITE) the curves are oriented    
                                                                                              //    : opposite relative to the loop.                                                 
                                  const SmTArray<SmPoint3d>        & crLoopPoints,            // in : If this array contains points. These points are used to create                 
                                                                                              //    : single vertex loops inside of the face.  Note that right now it                
                                                                                              //    : is not possible to have a single vertex loop which is the outer loop to a face 
                                  SmSurface                        * pSurface,                // in : Geometric definition of the shape of the face.                                 
                                  const SmExtent2d                 & crUVDomain,              // in : Domain of the surface used by the face                                         
                                  SmOrientType                       eSurfaceOrientation,     // in : Orientation of the surface used by the face.                                   
                                  SmRegion                        *& rpNewRegion,             // out: New region created.  It will always be NULL when dealing with just             
                                                                                              //    : trimmed surfaces.  When creating solids this parameter may be used.            
                                  SmShell                         *& rpNewShell,              // out: This is the new shell created by the face if any.  For the trimmed             
                                                                                              //    : surfaces, a new shell will always be created when a new face is created.       
                                  SmFace                          *& rpNewFace) ;             // out: This is the new face which is created during this operation.                   
                                                                  
  SmStatus MakeFacesWithCurves(SmRegion                         * pRegion,              // in : Owner of NewFace, NULL for infinite region.                                            
                               const SmTArray<ULONG>            & crCurveLoops,         // in : Loop Descriptions: LoopCount          = crCurveLoops.GetSize()                         
                                                                                        //    :                    Loop[ii] EdgeCount = crCurveLoops[ii]                                 
                               const SmTArray<SmCurve*>         * cpOpt3DCurves,        // in : Ordered Edge->Curve 3d shapes, NULL to ignore, if NULL must have cpOptUVCurves.        
                               const SmTArray<SmBSplineCurve*>  * cpOptUVCurves,        // in : Ordered Edge->UVTrimCurve UV shapes, NULL to ignore, if NULL must have cpOpt3DCurves.  
                                                                                        //    : note: when both cpOpt3DCurves and cpOptUVCurves,                                       
                                                                                        //    :their order, parameterizations, and orientations must match.                            
                               const SmTArray<SmOrientType>     & crCurveOrientations,  // in : orientation for each curve relative to the loop it's in.                               
                                                                                        //    : SM_OT_SAME     = Curve is traversed from start to end in its loop                      
                                                                                        //    : SM_OT_OPPOSITE = Curve is traversed from end to start in its loop                      
                               const SmTArray<SmPoint3d>        & crLoopPoints,         // in : points turned into single vertex loops. points must be within outer loop.              
                                                                                        //    : Empty to ignore.                                                                       
                               SmSurface                        * pSurface,             // in : the NewFace Surface.  C0 Surfaces get split to spawn multiple C1 Faces                 
                               const SmExtent2d                 & crUVDomain,           // in : Domain of the surface used by the face.                                                
                               SmOrientType                       eSurfaceOrientation,  // in : Orientation of the surface used by the face.                                           
                               SmRegion                        *& rpNewRegion,          // out: NewRegion when making NewFace splits an existing region                                
                               SmShell                         *& rpNewShell,           // out: NewShell when adding face to region splits an existing Shell or makes a new one        
                               SmTArray<SmFace*>                & rNewFaces) ;          // out: NewFaces. Typically one, but many when given a surface which is piecewise C1           

  // make a face given the surrounding boundary - makes NewFace with NewEdges and NewVertices 
  SmStatus MakeFaceWithBoundary(SmTArray<SmPoint3d> * cpOpt3DPoints,  // in : Optional if the boundary curve is input.     
                                SmBSplineCurve      * cpOpt3DCurve,   // in : Optional if the boundary points are input.   
                                SmFace*             & rNewFace) ;     // in : This is created face during this operation.  

  SmStatus MakeManifold(SmTArray<SmRegion*> * pOptRegionsToKeep = NULL,     // in : If NULL keep all non-infinite Regions.          
                                                                            //    : If specified keep only Regions in the array.    
                        SmBoolean             bKeepInteriorFaces = FALSE) ; // in : TRUE = Keep faces between two nonVoid Regions   

  SmStatus MakeShellVertexTopology(SmRegion  * pRegion,        // in : target region, NULL for infinite region.  
                                   SmShell  *& rpNewShell,     // out: new Shell                                 
                                   SmVertex *& rpNewVertex) ;  // out: New Vertex                                
                                 
  virtual SmStatus MakeShellVertex(SmRegion        * pRegion,       // in : target region NULL for infinite region. 
                                   const SmPoint3d & crPoint,       // in : target point location                   
                                   SmShell        *& rpNewShell,    // out: New Shell in pRegion                    
                                   SmVertex       *& rpNewVertex) ; // out: New Vertex in NewShell                  

  // load SmBrep from SmBrepData object - mirror function = SmBrepData::FromBrep
  SmStatus MakeTopologyFromData(SmBrepData             * pBrepData,                     // in : target BrepData object                                              
                                SmTArray<SmAttribute*> & rAllAttributes,                // in : list of all attributes to be assigned to Brep entities              
                                                                                        //    : currently contains one placeholder attribute                        
                                                                                        //    : for every attribute referenced within the SmBrep topology graph.    
                                SmBoolean                bMakeUVTrimCurves = TRUE,      // in : default TRUE = call SmFace::CreateUVTrimCurves for each face        
                                SmBoolean                bAddAnalytics = TRUE,          // in : default TRUE = replace every Surface and Curve with                 
                                                                                        //    : with a derived analytic type when appropriate.                      
                                SmBoolean                bForceSameTol = FALSE,         // in : default FALSE: when reading IGES, allow sensible reset of tolerance 
                                                                                        //    : when reading breps set TRUE (use original tol) ;                     
                                SmBoolean                bCheckUVTrimCurves = FALSE,    // in : default FALSE: Do checks on read-in uv trim curves.                 
                                                                                        //    : NOTE: currently not used. (Available...)
                                SmBoolean                bAlwaysHeal = FALSE,           // in : default FALSE: run healer on all read in BrepFiles
                                                                                        //      a flag for internal development
                                SmBoolean                bHealerIsEnabled = TRUE );     // in : default TRUE: healer is enabled  FALSE: disable healer in any case
                                                                                        

  // A MakeTopologyFromData helper function to fix common read in database problems
  SmStatus HealBrep(SmHealerOpType          eHealerOp,                 // in : Run Healer sequence up to and including this operation.  Oneof:
                                                                       //        SM_HO_NONE = 0,                           //  No Healer Steps run                                           
                                                                       //                                                                                                                    
                                                                       //        SM_HO_FIX_BACKPOINTERS,                   // Ran All Healer Steps thru SmHealData::Fix_BackPointers         
                                                                       //                                                                                                                    
                                                                       //        SM_HO_CACHE_EDGEPROPS,                    // Ran All Healer Steps thru SmHealData::Cache_EdgeProps()        
                                                                       //        SM_HO_CACHE_VERTEXPROPS,                  // Ran All Healer Steps thru SmHealData::Cache_VertexProps()         
                                                                       //        SM_HO_CACHE_FACEPROPS_GAPS,               // Ran All Healer Steps thru SmHealData::Cache_FaceProps_Gaps()   
                                                                       //                                                                                                                    
                                                                       //        SM_HO_FIX_TOLSIZES,                       // Ran All Healer Steps thru SmHealData::Fix_TolSizes()           
                                                                       //        SM_HO_CACHE_COIN_VERTICES,                // Ran All Healer Steps thru SmHealData::Cache_CoinVertices()     
                                                                       //        SM_HO_FIX_COIN_VERTICES,       /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_CoinVertices()       
                                                                       //                                                                                                                    
                                                                       //        SM_HO_CACHE_DEGEN_FACES,                  // Ran All Healer Steps thru SmHealData::Cache_DegenFaces()       
                                                                       //        SM_HO_FIX_DEGEN_FACES,                    // Ran All Healer Steps thru SmHealData::Fix_DegenFaces()         
                                                                       //                                                                                                                    
                                                                       //        SM_HO_FIX_DEGEN_EDGES,                    // Ran All Healer Steps thru SmHealData::Fix_DegenEdges()         
                                                                       //                                                                                                                    
                                                                       //        SM_HO_CACHE_COIN_EDGES,        /* TODO */ // Ran All Healer Steps thru SmHealData::Cache_CoinEdges()        
                                                                       //        SM_HO_FIX_COIN_EDGES,          /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_CoinEdges()          
                                                                       //                                                                                                                    
                                                                       //        SM_HO_CACHE_MISSED_EDGEXSECTS, /* TODO */ // Ran All Healer Steps thru SmHealData::Cache_MissedEdgeXSects() 
                                                                       //        SM_HO_FIX_MISSED_EDGEXSECTS,   /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_MissedEdgeXSects()   
                                                                       //                                                                                                                    
                                                                       //        SM_HO_FIX_UNCONTAINED_EDGES,   /* STUB */ // Ran All Healer Steps thru SmHealData::Fix_UncontainedEdges()   
                                                                       //        SM_HO_FIX_BADGAPS,             /* TODO */ // Ran All Healer Steps thru SmHealData::Fix_BadGaps()            
                                                                       //                                                                                                                    
                                                                       //        SM_HO_CACHE_FACEPROPS_2,                  // Ran All Healer Steps thru SmHealData::Cache_FaceProps_Stage2() 
                                                                       //                                                                                                                    
                                                                       //        SM_HO_FIX_MOVESEAM,                       // Ran All Healer Steps thru SmHealData::Fix_MoveSeam()           
                                                                       //        SM_HO_FIX_SPLITEDGE_ATSEAM,               // Ran All Healer Steps thru SmHealData::Fix_SplitEdgesAtSeam()   
                                                                       //        SM_HO_FIX_BADSHEETS,                      // Ran All Healer Steps thru SmHealData::Fix_BadSheets()          
                                                                       //                                                                                                                    
                                                                       //        SM_HO_CACHE_FACEPROPS_3,                  // Ran All Healer Steps thru SmHealData::Cache_FaceProps_Stage3() 
                                                                       //                                                                                                                    
                                                                       //        SM_HO_FIX_SPLITFACE_ATSEAMS,              // Ran All Healer Steps thru SmHealData::Fix_SplitFaceAtSeams()   
                                                                       //        SM_HO_FIX_BADLOOPS,                       // Ran All Healer Steps thru SmHealData::Fix_BadLoops()           
                                                                       //        SM_HO_MAKE_UVTRIMCURVES,                  // Ran All Healer Steps thru SmHealData::Make_UVTrimCurves()      
                                                                       //        SM_HO_FIX_INFINITE_REGIONS,               // Ran All Healer Steps thru SmHealData::Fix_InfiniteRegion()     
                                                                       //                                                                                                                    
                                                                       //        SM_HO_ALL                                 // Ran All Healer Steps                                           
                    SmTArray<SmVertex*>   * pOptVertices=NULL,         // in : optional TgtVertex list, default:[NULL]= all Vertices                     
                    SmTArray<SmEdge*>     * pOptEdges=NULL,            // in : optional EdgeProps list, default:[NULL]= all Edges                        
                    SmTArray<SmFace*>     * pOptFaces=NULL,            // in : optional FaceProps list, default:[NULL]= all Faces                        
                    SmBoolean               bMakeUVTrimCurves=FALSE) ; // in : TRUE=call SmFace::CreateUVTrimCurves for each face, default:[FALSE]=Don't 

  virtual SmStatus MakeVertexSplitEdge(SmEdge    * pEdgeToSplit,         // in : target edge                                               
                                       double      dEdgeSplitParameter,  // in : target parameter                                          
                                       SmEdge   *& rpNewEdge1,           // out: new Edge1 (by chance == pEdgeToSplit) or NULL             
                                       SmEdge   *& rpNewEdge2,           // out: new Edge2 (by chance == newly allocated edge) or NULL     
                                       SmVertex *& rpNewVertex) ;        // out: new vertex (newly allocated) or nearby endVertex or NULL  

  virtual SmStatus MakeVertexLoop(SmFace          * pFace,                      // in : target face                    
                                  const SmPoint3d & cr3DPointInFace,            // in :                                
                                  SmLoop         *& rpNewLoop,                  // out: new loop                       
                                  SmVertex       *& rpNewVertex,                // out: new vertex                     
                                  SmOrientType      eOrient = SM_OT_OPPOSITE) ; // in : Orientation for new Loopuses.  

  // make new wire edge connected to given vertices and contained in given region
  virtual SmStatus MakeWireEdge(SmRegion          * pRegion,            // in : region to contain new wire                                                             
                                                                        //    : NULL for infinite region.                                                              
                                SmVertex          * pStartVertex,       // in : vertex to connect to new wire start                                                    
                                SmVertex          * pEndVertex,         // in : vertex to connect to new wire end                                                      
                                SmCurve           * pCurve,             // in : shape of new wire                                                                      
                                const SmExtent1d  & crInterval,         // in : interval of pCurve for new wire shape                                                  
                                SmOrientType        eCurveOrientation,  // in : SM_OT_SAME     = edge goes from StartVertex to EndVertex as curve parameter increases  
                                                                        //    : SM_OT_OPPOSITE = edge goes from EndVertex to StartVertex as curve parameter increases  
                                SmEdge           *& rpNewEdge) ;        // out: The new edge                                                                           

  virtual SmStatus MakeWireEdgeVertex(SmRegion         * pRegion,           // in : NULL for infinite region.       
                                      SmVertex         * pStartVertex,      // in :                             
                                      SmCurve          * pCurve,            // in : consumed.                   
                                      const SmExtent1d & crInterval,        // in :                             
                                      SmOrientType       eCurveOrientation, // in :                             
                                      const SmPoint3d  & crEndPoint,        // in :                             
                                      SmEdge          *& rpNewEdge,         // out:                             
                                      SmVertex        *& rpNewVertex) ;     // out:                             

  SmStatus MateLaminaEdgeToSurface(SmEdge          * pLaminaEdgeToModify,       // in :                                                                           
                                   const SmSurface & crMatingSurface,           // in :                                                                           
                                   double            dSurfaceExtensionFactor,   // in : How far past measured distance to surface                                 
                                                                                //    : should we extend surface - 1.5 is default                                 
                                   double            dUVCurveExtensionFactor) ; // NotUsed: in : How many times should we extend the UV curve interval of adjacent edges   
                                                                                //    : on the new surface - 10 is the default. if the adjacent curves            
                                                                                //    : are very small and the extension is great this may need to be increased.  
                                                                                //    : Note that it does a linear extension of the UV curve interval.            
                                    

  // merge 3DCurves known to be on a surface into the surface->Face
  // possibly creating new vertices, edges and splitting Face into faces
  virtual SmStatus MergeCurvesOnSurface(const SmSurface                 & crSurface,            // in : Surface must be owned by a face in this Brep.                              
                                        double                            d3DTolerance,         // in : 3D Tol for curves greater than MaxCrvSrfGap and MaxCrvTrimCrvGap.          
                                        const SmTArray<SmCurve*>        & cr3DCurves,           // in : 3D Curves on surface not necessarily trimmed to face boundary.
                                                                                                //      input Crvs not consumed by this method, ie. still owned by caller on output             
                                        const SmTArray<SmCurve*>        * cpOptUVCurves,        // in : associated UV Curves having a dim of 2.                                    
                                                                                                //      input UVCrvs not consumed by this method, ie. still owned by caller on output             
                                        SmTArray<SmFace*>               & rNewFaces,            // out: Newly created faces, if any.                                               
                                        SmTArray<SmEdge*>               & rEdges,               // out: Brep Edges for cr3DCurves (new and/or existing)                            
                                                                                                //    : ordered:[CurveParameritization]                                            
                                        SmBoolean                         bUseProjection=FALSE, // in : TRUE = use projection for intersections to allow for tolerances.           
                                                                                                //    : FALSE= no projections for faces already toleranced.                        
                                        const SmSurface        * pOptIntersectionSurface=NULL,  // in : OtherSurface when cr3DCurves are from surf/surf XSects.                    
                                                                                                //    : As curves are merged into Brep - surf/surf/surf XSects are                 
                                                                                                //    : used for new verts and to improve Edge->Curve interpolation.               
                                                                                                //    : NULL to ignore, default:[NULL], NULL creates larger gaps.                  
                                        SmBoolean                         bUVSpaceOkay=TRUE) ;  // in : TRUE = try UVSpace Classification if 3Space try is dodgey (tolerant cases) 
                                                                                                //    : FALSE= don't try UVSpace because UVTrimCurves aren't yet valid             

  SmStatus MergeSelfIntersectionCurves(const SmSurface                 & crSurface,      // in :   
                                       double                            d3DTolerance,   // in :   
                                       const SmTArray<SmCurve*>        & cr3DCurves,     // in :   
                                       const SmTArray<SmBSplineCurve*> & crUVCurves1,    // in :   
                                       const SmTArray<SmBSplineCurve*> & crUVCurves2,    // in :   
                                       SmTArray<SmEdge*>               & rNewEdges) ;    // out:   

  // copy crBrepToMerge topology graph (with attributes) into this Brep without
  // intersection and coincidence checks
  // note: does not work on Breps with composite edges and composite faces
  SmStatus MergeBrep(const SmBrep        & crBrepToMerge,                        // in : Brep to merge                                                
                     SmCopyBrepMap       * pOptCopyBrepMap = NULL,               // out: A map from copyBrep topology Obj ptrs to orig ptrs           
                     SmCopyBrepMap       * pOptReverseMap = NULL,                // out: A map from orig topology Obj ptrs to copyBrep ptrs
                     SmBoolean             bAddAnalytics = TRUE) ;             // in : TRUE = recognize analytic surfaces; FALSE = preserve surface types. Default:[TRUE]

  SmStatus Mirror(const SmAxis2Placement & crMirrorPlane) ;

  void Notify(SmNotifyOperation  eNotifyOperation, // in |       event                | caller      |  pData1  | pData2                | pData3                    
              SmObject         * pData1,           //    |----------------------------+-------------+----------+-----------------------+-------------------------- 
              SmObject         * pData2,           //    | SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL  
              SmObject         * pData3) ;         //    | SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2                    
                                                   //    | SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep                      
                                                   //    | SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                     
                                                   //    | SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB                     
                                                   //    | SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL   
                                                   //    | SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL           
                                                   //    | SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL          
                                                   //    | SM_NO_CONSTRUCTION         | NewObj      | NewObj   | CopyFromObj or NULL   | NULL                      
                                                   //    | SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL   
                                                   //    | SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
                                                   //    | SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
                                                   //    | SM_NO_SPLIT                | SplitGeomObj| Child1   | Child2                | SplitObj's Owner or NULL  
                                                   //    | SM_NO_MERGE                | MergeGeomObj| OrigObj1 | OrigObj2              | MergeObj's Owner or NULL  
                                                   //    | SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg        
                                                   //    | SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                     

  // force all faces in a sheet body to share a common up/down
  SmStatus OrientSheetSurfaces(SmFace *pSheetFace,       // in : any face in the sheet                                    
                               SmBoolean bSame = TRUE) ; // in : TRUE = return sheet with pSheetFace current orientation  
                                                         //      FALSE= return sheet with pSheetFace toggled orientation      

  // force all faces in a manifold body (or one that can be stitched into a manifold body) to share common orientation
  SmStatus OrientTrimmedSurfaces(SmBoolean   bNormalsOutward,                   // in : TRUE = normals will point outward from solid   
                                 SmBoolean & rbMaybeNotClosedSolid,             //    : FALSE= normals will point inward.              
                                 SmBoolean   bJustUnifyNormals,                 // out: TRUE = inconsistencies in rayfiring.           
                                                                                //    : object may not be a closed solid               
                                                                                // in : TRUE = For closed solids don't fire rays,      
                                                                                //    : just try to unify normals using topology
                                 SmTArray<SmFace*> * pOptFacesToSwapUV = NULL); // in : Optional array of faces whose UV coorindates should be inverted. NULL to ignore. Default is NULL.
  
  SmStatus Point3DClassify(const SmPoint3d       & cr3DPointToClassify,      // in : point to classify                                                                 
                           SmZoneTol3d             sPointZoneTol3d,          // in : ZoneTol3d for PointToClassify                                                     
                           SmBoolean               bDoBoundaryIntersections, // in : TRUE = Look for Vert, Edge, and Face XSects before looking for containing region  
                                                                             //       : FALSE= only look for containing region                                            
                           SmPointClassification & rPointClassification)     // out: m_ePointClass:[oneof SM_PC_REGION, SM_PC_FACE, SM_PC_EDGE, SM_PC_VERTEX]          
                          const ;                                            //       : m_pOject:[containing object]                                                      

  // note: can increment an unused Mark value
  SmStatus ProjectAndTrim(const SmBSplineCurve & crCurveToProject,          // in : Curve to project to the faces                                   
                          const SmVector3d     & crProjectionDirection,     // in : Defines direction of projection                                 
                          const SmPoint3d      & crReferencePoint,          // in : Defines a side of the curve which is either kept or removed     
                                                                            //    : depending upon the trim type.                                   
                          SmTrimType             eTrimType,                 // in : oneof: SM_TT_KEEP_POINT   - keep TrimCurve 'side' with Ref Pt   
                                                                            //    :        SM_TT_DELETE_POINT - remove TrimCurve 'side' with Ref Pt 
                                                                            //    :        SM_TT_SPLIT        - keep both TrimCurve 'sides'.        
                          const double         * cpdOpt3DApproximationTol,  // in :                                                                 
                          const double         * cpdOptAngleTolRadians,     // in :                                                                 
                          SmTArray<SmFace*>    * pOptSubsetFaces = NULL) ;  // in : Optional, only search this subset of faces                      

  SmStatus Refine3DCurves( double dTol ) ;           // in : refinement tolerance >200*pBrepTol  

  // Healing function - Recalculate trim curves and reorder loops.
  SmStatus RebuildFaceFromEdgeCurves( SmFace *pFace ) ;

  // Healing function - Recreate each Face with re-dropped uv trim curves.
  SmStatus RebuildFacesFrom3DCurves( void ) ;

  SmStatus RemoveAttributeFromTopology( ULONG lAttribId ) ; // in : target Attirbute Id value to remove from Brep topology graph 

  SmStatus RemoveFaces(const SmTArray<SmFace*> & crFaces) ; 

  SmStatus RemoveAllTopology() ; // Deletes all Faces, Edges, Vertices.

  // classify a ray's origin and its first point of intersection with a Brep Object - no mark increments
  SmStatus RayFire(const SmPoint3d       & crRayStartPoint,      // in : target point                                               
                   const SmVector3d      & crRayVector,          // in : target direction                                           
                   SmZoneTol3d             sRayZoneTol3d,        // in : RayZoneTol3d - sizes XSectTol3d dist for finding objects   
                   SmRegion             *& rpRegionOfPoint,      // out: Region containing target point                             
                   SmPointClassification & rRayIntersection,     // out: Point Classification of Best Ray/Brep Intersection         
                   double                * pOptAnswerLimit=NULL) // in : max distance searched for intersections, NULL to ignore    
                  const ;

  // get position and normal of ray intersection with brep faces
  SmStatus RayIntersection(const SmPoint3d  & crRayPoint,        // in : fired ray's base point                                      
                           const SmVector3d * cpOptRayVector,    // in : fired ray's direction or NULL,                              
                                                                 //    : NULL = intersect Brep with crRayPoint.                      
                           SmZoneTol3d        sRayZoneTol3d,     // in : RayZoneTol3d - sizes XSectTol3d dist for finding objects    
                                                                 // in : If the ray misses by a small distance,                      
                                                                 //    : this value determines how far away it will still be a 'hit' 
                                                                 //    : Note: that this value is 'added' to the tolerance           
                                                                 //    : on the brep entities.  Make this value 0.0 if               
                                                                 //    : the tolerance on the Brep objects is acceptable.            
                           double             dRayStepoffValue,  // in : How far down the ray should we go before                    
                                                                 //    : accepting an answer.                                        
                                                                 //    : This should be a parameter value relative to                
                                                                 //    : the ray point and vector.                                   
                                                                 //    : A good value for this is something like                     
                                                                 //    : 2.0 * SM_EFF_ZERO * (1.0 + crRayPoint.GetMaxDimension())    
                           SmSolutionArray  & rSolutions)        // in : The solutions                                               
                                                                 //    : will contain 5 values sSolutions[i].m_vStart[0] - [4]       
                                                                 //    : The first and second values  are U,V of the surface         
                                                                 //    : and the third through the fifth values are the              
                                                                 //    : outward surface normal values.                              
                                                                 //    : The parameter on the ray at which the intersection          
                                                                 //    : occurs is at sSol.m_vStart.m_dSolutionValue                 
                          const ;

  SmStatus RemoveTopologicalEdgesAndVertices(SmTArray<SmEdge*>   * pOptKeepEdges = NULL,       // in : List of Edges to keep even if Topological, NULL to ignore, default:[NULL] 
                                             SmTArray<SmVertex*> * pOptKeepVertices = NULL) ;  // in : List of Vertices to keep even if Topological, NULL to ignore, default:[NULL] 
// Remove Composites
//  // Faces
//  int ReplaceCompositeFaces(SmTArray<SmFace *> *pOptFaces=NULL) ;

// Remove Composites
//  // Edges
//  int ReplaceCompositeEdges(SmTArray<SmEdge *> *pOptEdges=NULL) ;

  SmStatus ApproximateWithBSplines(double               dApproxTol,        // in : maximum allowed distance between approx and original shapes 
                                   double             & dMaxAchievedTol,   // [out]  : max of all max achieved tolerances 
                                   SmTArray<SmFace *> * pOptFaces=NULL,    // in : optional list of tgt Faces, NULL=Check all Faces, default:[NULL] 
                                   SmTArray<SmEdge *> * pOptEdges=NULL) ;  // in : optional list of tgt Edges, NULL=Check all Edges, default:[NULL] 

  // replace a face's surface, always delete face->UVTrimCurves, optionally delete or return OldSurface, and rebuildUVTrimCurves
  SmStatus ReplaceSurface(SmFace            * pFace,                         // in : Target Face to get new Surface                                        
                          SmSurface         * pNewSurfaceOfFace,             // in : Target Surface to replace current pFace->Surface                      
                          SmBoolean           bAddToDeleteSurfaceList,       // in : TRUE = add OldSurf to this->m_pDeletedSurfaces list                   
                                                                             //       : FALSE= delete OldSurf here                                            
                                                                             //       : UNSURE= don't delete OldSurf, don't add OldSurf to m_pDeletedSurfaces 
                          SmTArray<SmFace*> * rOptModifiedFaces=NULL,        // out: optional list of modified faces, default:[NULL]                       
                          SmBoolean           bOptCreateUVTrimCurves=TRUE) ; // in : TRUE = Call SmFace::CreateUVTrimCurves, default:[TRUE]                

  // replace a edge's curve, alsway delete edge->UVTrimCurves
  SmStatus ReplaceCurve(SmEdge            * pEdge,         // in : Target Edge to get new Curve                   
                        SmCurve           * pNewCurve) ;   // in : Target Curve to replace current pEdge->Curve   

  SmStatus RotateSection(const SmContext & crContext,   // in : Context for pBrep2, maybe not for this                       
                         SmPoint3d         sOrigin,     // in : Origin of Axis of rotation                                   
                         SmVector3d        sAxis,       // in : Direction of Axis                                            
                         SmVector3d        sDirection,  // in : Direction from origin of Section Cut plane                   
                         double            dDeg,        // in : degrees of rotation (-360.0 <= dDeg <= 360.0                 
                         int               EndCaps,     // in : Only of dDeg < 360: 0 = none, 1 = start, 2 = end, 3 = both.  
                         SmBrep         *& pBrep2) ;    // in : output brep;                                                 

  SmStatus Simplify(SmBoolean bRefineEdges,     // in :   
                    SmBoolean bRefineSurfaces,  // in :   
                    double    dTol) ;           // in :   

// Remove Composites
//   inline void SetMakeComposites(SmBoolean bMakeComposites) { m_bMakeComposites = bMakeComposites; }

  SmStatus Smooth( ) ;

  // squeeze a [vertex-edge-vertex] into a [vertex] updating the topology graph as needed
  SmStatus SqueezeEdge(SmEdge   * pEdgeToSqueeze,      // in : edge to delete 
                       SmVertex * pSurvivingVertex) ;  // in : vertex to retain 

  // Remove a face from the topology graph leaving one or more vertices and edges behind 
  SmStatus SqueezeFace(SmFace    * pFace,                         // in : Face to squeeze                                   
                       SmXSectTol3d * pOptStitchTol3d,            // in : optional stitching tolerance 
                       SmTArray<SmVertex*> * pOptDeletedVertices, // out: Vertices that were deleted (stale pointers)      
                       SmTArray<SmEdge*>   * pOptNewEdges,        // out: New Edges created                                
                       SmTArray<SmEdge*>   * pOptDeletedEdges) ;  // out: Edges that were deleted    (stale pointers)      
                   
  // Topology and Geometry edit: squeeze a face into one or more vertices and edges
  SmStatus SqueezeFaceIfDegenerate(SmFace              * pFace,               // in : Face to squeeze                                  
                                   SmBoolean           & rbWasSqueezed,       // out: TRUE == face was squeezed. FALSE == otherwise    
                                   SmTArray<SmVertex*> * pOptDeletedVertices, // out: Vertices that were deleted (stale pointers)      
                                   SmTArray<SmEdge*>   * pOptNewEdges,        // out: New Edges created                                
                                   SmTArray<SmEdge*>   * pOptDeletedEdges) ;  // out: Edges that were deleted    (stale pointers)      

  // Topology and Geometry edit: squeeze an array of faces into one or more vertices and edges where degenerate - not implemented yet
  SmStatus SqueezeFacesIfDegenerate(SmTArray<SmFace*>   & rFaces,        // in : Face to squeeze
                                    SmTArray<SmFace*>   & rSqueezedFaces,// out: list of stale pointers to faces that were squeezed
                                    SmTArray<SmEdge*>   & rNewEdges,     // out: New Edges created
                                    SmTArray<SmVertex*> & rNewVerts) ;   // out: New vertices created

  // make specified Region the infinite region
  inline void SetInfiniteRegion(SmRegion * pNewInfiniteRegion) { m_pInfiniteRegion = pNewInfiniteRegion; }

  // Set all Vertex->Point, Curve->ControlPoint, and Surface->ControlPoint positions
  SmStatus SetDeformPoints(const SmTArray<SmPoint3d> & crDeformPoints) ;

  SmStatus StitchFaces(double               dXSectTol3d,                 // in : max 3d distance between coincident vertex and edge pairs             
                       ULONG              & rlStitchedEdges,             // out: number of edges stitched                                                 
                       ULONG              & rlLaminaEdges,               // out: number of lamina edges remaining after stitch                           
                       double             & rdMaxVertexGap,              // out: max gap found between coincident vertices considered for glueing     
                       double             & rdMaxEdgeGap,                // out: max gap found between coincident edges    considered for glueing     
                       SmTArray<SmFace *> * pOptFaces = NULL,            // in : only stitch topology connected to these faces, NULL=all faces           
                                                                         //    : default:[NULL]                                                       
                       SmBoolean            bAllowSpineEdges = FALSE) ;  // in : if true this allows formation of spine edges. See c below.           
                                                                         //    : default:[FALSE]                                                      
                                                                         //    : NOTE: some coincident vertex and edge pairs do not get glued due to  
                                                                         //    : a. gaps exceeding tolerances                                         
                                                                         //    : b. a failure within the glue edge function                           
                                                                         //    : c. not being lamina when m_bMakingManifoldSolid == TRUE
           

  SmStatus StitchAndOrient(double dStitchTol3d = -0.1,                   // in : dXSectTol3d =  (dStitchTol3d < 0.0) ? 2.0 * ZoneTol3d,
                                                                         //    : (dStitchTol3d > 100 * SM_ZONE_TOL_3d) ? 100 *
                                                                         //      SM_ZONE_TOL_3D : dStitchTol3d : default:[-0.1]
                           SmBoolean bAllowSpineEdges = FALSE,           // in : TRUE = allow creation of spine edges, FALSE =
                                                                         //      don't
                           SmTArray<SmFace*> * pOptFacesToSwap = NULL,   // in : optional array of faces to swap UV after import.
                                                                         //      Included to reduce redundant calls to SwapUV.
                           SmBoolean * pbJustUnify = NULL);              // in : optional flag to force the orient call to only unify the normals.

  SmStatus ShrinkGeometry() ;

  // Replace all high ControlPoint count SurfOfRevolution surfaces with simpler representations
  SmStatus RefitSurfOfRevolutionSurfaces(const SmContext      & crContext,                          // in : context for new object construction                                 
                                         double                 dThisApproxTol3d = 0.001,           // in : max allowed deviation between returned surface and current surface  
                                         double                 dSurfOfRevolutionTolerance = 0.01,  // in : use 0.01                                                            
                                         ULONG                  lTestKnotCount = 100) ;

  // Split TgtFace at all Face->Surface discontinuous isoParamLine - creates new vertices and edges, and splits edges and faces when Surface has discontinuities  
  SmStatus SubdivideFaceAtDiscontinuities(SmFace            * pFace,                            // in : TgtFace 
                                          SmTArray<SmFace*> & rResultingFaces,                  // out: Split at face discontinuity resulting child faces (or orig face with no discontinuities)
                                          SmContinuityType    eOptContinuityToSplit=SM_CT_C0) ; // in : oneof:     SM_CT_CO = split face at C0 discontinuities  
                                                                                                //             ... SM_CT_C1 = split face at C1 and C0 discontinuities  
                                                                                                //             ... SM_CT_C2 = split face at C2, C1, and C0 discontinuities  
                                                                                                //             ... and all other SmContinuityType values    
  SmStatus SwapUV() ;  // increments unlocked mark value

  SmStatus FixAllFaceDiscontinuities() ;
  SmStatus FixAllRepeatedEndControlPoints() ;

  // note: increments an unused Mark value
  SmStatus TestClosureSplitRegion(SmBoolean               bDoInfiniteRegionTesting, // in : TRUE = find new infinte region            
                                  SmEdge                * pTestEdge,                // in : target Edge                               
                                  SmTArray< SmShell  *> & rpNewShells,              // out: not empty when pTestEdge splits a region  
                                  SmTArray< SmRegion *> & rpNewRegions) ;           // out: not empty when pTestEdge splits a region  

  // note: increments an unused Mark value
  SmStatus TestClosureSplitRegion(SmBoolean   bDoInfiniteRegionTesting, // in : TRUE = find new infinte region by RayCasting  
                                  SmFaceuse * pFUStart,                 //    : FALSE=                                        
                                  SmShell  *& rpNewShell,               // in : target Faceuse                                
                                  SmRegion *& rpNewRegion) ;            // out: not NULL when target Faceuse splits region    

  virtual SmStatus Transform(const SmAxis2Placement & crRotateNMove,     // in : affine rotate and move transformation      
                             const SmVector3d       * cpOptScale=NULL,   // in : optional scaling about current origin point before RotateNMove
                                                                         //      for isotrpoic scaling    - No geometry type changes
                                                                         //      for nonisotrpoic scaling - changes all geometry types to BSplines
                                   SmBoolean          bScaleTol=TRUE) ;  // in : When TRUE, scale object tolerances as needed by ModelSize changes due to cpOptScale scaling

  SmStatus         Transform(const double   sTransMatrix[4][4],          // in : arbitrary 4x4 transform matrix
                             SmBoolean      bScaleTol = TRUE) ;          // in : When TRUE, scale object tolerances as needed by ModelSize changes due to scaling

  // intersect set of UVTrimCurves to form new surface trim boundary
  SmStatus TrimSurfaces(const SmTArray<SmCurve*>   & cr3DTrimCurves,    // in : 3D model space trimming curves which lie on                      
                                                                        //    : faces and will be copied not consumed by                         
                                                                        //    : this methed. Each curve should have already                      
                                                                        //    : been trimmed by the surfaces' boundary                           
                        const SmTArray<SmSurface*> & crSurfaces,        // in : Array of surfaces that 'matches' the array of                    
                                                                        //    : trim-curves so that each curve is associated                     
                                                                        //    : to the surface it's on.                                          
                        const SmPoint3d            & crReferencePoint,  // in : Point which is 'close' enough to the                             
                                                                        //    : portion of surfaces that is to be trimmed away or kept.          
                        SmTrimType                   eTrimType) ;       // in : oneof: SM_TT_KEEP_POINT   - keep TrimCurve 'side' with Ref Pt    
                                                                        //    :        SM_TT_DELETE_POINT - remove TrimCurve 'side' with Ref Pt  
                                                                        //    :        SM_TT_SPLIT        - keep both TrimCurve 'sides'.         

  SmStatus TurnToNURBS() ;

  SmStatus TurnToNURBSAndSubdivide(ULONG  lDegree,         // in :  
                                   double dKnotSpacing) ;  // in :  

  void     UnStitch(SmTArray<SmFace *> *pOptFaces = NULL) ;

  SmStatus UpdateVertexEdgeIntervals(SmVertex *pVertex) ;

  SmStatus ValidatePointers(SmAssertArray    * pAList=NULL,           // [in,out]: Accumulating list of failed Asserts, NULL to ignore                            
                            SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                       
                                                                      //       : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                   
                                                                      //       : default:[SM_LEVEL_0]                                                             
                            SmTArray<ULONG>  * pTestRequests=NULL,    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
                            SmAssertWalking    eWalkTree=SM_NO_WALK)  // in : SM_WALK = a later AssertSubTopology will CheckLoop; skip CheckFace's CheckLoop
                           const ;

  SmStatus ValidateCounts(ULONG lFaces,         // in : expected number of Faces          
                          ULONG lEdges,         // in : expected number of Edges          
                          ULONG lVertices,      // in : expected number of Vertices       
                          ULONG lWireEdges,     // in : expected number of WireEdges      
                          ULONG lLaminaEdges,   // in : expected number of LaminaEdges    
                          ULONG lManifoldEdges, // in : expected number of ManifoldEdges  
                          ULONG lRegions,       // in : expected number of Regions        
                          ULONG lShells)        // in : expected number of Shells         
                         const ;

  SmBoolean CompareCounts(const SmBrep *pBrep2) const ;

  SmStatus ValidateAndUpdateTolerances(SmBoolean          bValidateOnly,                           // in : TRUE = Only report gap problems                                                  
                                                                                                   //    : FALSE= fix gap probs (increasing Obj tol or rebuild UVTrimCurve)                 
                                       SmBoolean        & rbTolerancesViolatedOrUpdated,           // out: TRUE=tolerances updated or violated                                              
                                       double           & rdMaxEdgeFaceTrimCurveGap,               // NotUsed: [out]: Max Edge/UVTrimCurve gap found                                                   
                                       double           & rdMaxVertexEdgeGap,                      // out: Max Vertex/Edge gap found                                                        
                                       double           & rdMaxVertexFaceGap,                      // out: Max Vertex/Surface gap found                                                     
                                       SmAssertArray    * pAList = NULL,                           // NotUsed: in : SmAssertReport Array of found errors: default:[NULL]                             
                                       SmAssertTestLevel  eTestLevel = SM_LEVEL_0,                 // NotUsed: in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                       
                                                                                                   //    : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                   
                                                                                                   //    : default:[SM_LEVEL_0]                                                             
                                       SmTArray<ULONG>  * pTestRequests = NULL,                    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL] 
                                       SmEdgeuse       ** pOptMaxEdgeFaceTrimCurveObj = NULL,      // NotUsed: [out]: Edgeuse with MaxEdgeFaceTrimCurveGap, NULL to ignore, default:[NULL]             
                                       SmVertexuse     ** pOptMaxVertexEdgeObj = NULL,             // out: Vertex with MaxVertexEdgeGap, NULL to ignore, default:[NULL]                     
                                       SmVertexuse     ** pOptMaxVertexFaceObj = NULL,             // out: Vertex with MaxVertexFaceGap, NULL to ignore, default:[NULL]                     
                                       SmEdgeuse       ** pOptMaxEdgeFaceObj = NULL,               // out: Edgeuse with MaxEdgeFaceGap, NULL to ignore, default:[NULL]                      
                                       SmEdgeuse       ** pOptMaxVertexFaceTrimCurveObj = NULL,    // NotUsed: [out]: Edgeuse with MaxVertexFaceTrimCurveGap, NULL to ignore, default:[NULL]           
                                       double           * pOptMaxEdgeFaceGap = NULL,               // out: Max Edge/Face gap found, NULL to ignore, default:[NULL]                          
                                       double           * pOptMaxVertexFaceTrimCurveGap = NULL) ;  // NotUsed: [out]: Max Vertex/UVTrimCurve gap found, NULL to ignore, default:[NULL]                 

  // write Brep to File without Attributes - see SmBrepData::WritePartToFile for attributes
  SmStatus WriteToFile(const TCHAR * cOutputFileName,          // in : target file name                                               
                       SmFileType    eType = SM_ASCII,         // in : one of SM_ASCII,SM_BINARY                                      
                                                               //    : default:[SM_ASCII]                                             
                       SmBoolean     bNewFile = TRUE,          // in : TRUE = open file and rewrite contents                          
                                                               //    : FALSE= open file and append to end                             
                                                               //    : default:[TRUE]                                                 
                       SmBoolean     bWriteAsBSplines = FALSE, // in : FALSE = write native geometry,                                 
                                                               //    : TRUE  = write NonBSpline geometry as BSpline Approximations    
                                                               //    : default:[FALSE]                                                
                       double        dApproxTol = 0.0)         // in : only used when bWriteAsBSplines==TRUE, 0.0 = use m_dApproxTol  
                      const ;

  // read Brep from File without Attributes - see SmBrepData::ReadPartFromFile for attributes
  SmStatus ReadFromFile(const SmContext & crContext,                      // in : context for new object construction                                         
                        const TCHAR     * cInputFileName,                 // in : target file to read                                                         
                        SmFileType        eType = SM_ASCII,               // in : one of SM_ASCII,SM_BINARY                                                   
                        SmBoolean         bRebuildUVTrimCurves = FALSE) ; // in : TRUE = rebuild UVTrimCurves with RebuildUVTrimCurves() call, FALSE = don't  

  void *operator new(size_t size, const SmContext & crContext) ;
  void  operator delete(void *ptr) { smos_Free(ptr) ; ptr = NULL ; }

#ifndef SM_BORLAND
  void *operator new(size_t size) ;
  void  operator delete(void *ptr, const SmContext &) { smos_Free(ptr) ; ptr = NULL ; }
#endif // no SM_BORLAND

  virtual void Dump(ULONG, SmBrepDumpType eDumpType=SM_BD_BASE_ONLY) const ;
  virtual void Dump(const TCHAR * message, SmBrepDumpType eDumpType=SM_BD_BASE_ONLY) const ;
  virtual void Dump(SmBrepDumpType) const ;
  void DumpTopology() const ;     // pretty print pointer values within the Brep Topology tree - good for debug
  void DumpAttributes() const ;   // pretty print topology object lists with their attributes
  void DumpTolerances() const ;   // pretty print topology object tolerances - very time consuming - but helpful to see when tolerances need to be reviewed
  void DumpRegionsAndAttributes(const TCHAR *pOptLabel=NULL) ; // pretty print Regions and their attributes

  // Compare two SmBrep objects for round-trip validation
  SmStatus CompareBreps(SmBrep * pOther, SmBrepCompareResult & rResult) const ;

  // Compare two .smb files: reads both, calls CompareBreps, reports result
  static SmStatus CompareFiles(const TCHAR * pFileA, const TCHAR * pFileB, SmBrepCompareResult & rResult) ;

  // make display list of Brep Graphics for given display parameters
  virtual SmStatus OutputGraphics(const SmDisplayParameters & crDisplayParameters,    // in : display control parameters
                                  SmDisplayList             * pActiveDisplayList,     // out: last or current active display list (s_View.m_pActiveLists[last] when s_View.m_bGraphicsInitialized else NULL) 
                                  SmGfxArraySet             * pOptGfxSet=NULL)        // [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                 const ;

  // build and add Brep display list for global display parameters to global display list array
  virtual SmDisplayList * Draw(SmBoolean bAddToUIPickList=FALSE,   // in :
                                SmGfxArraySet * pOptGfxSet=NULL)   // in : draw parameters from smgfx_RefGlobalDisplayParameters()
                               const ;

  SmDisplayList * DrawUV(SmBoolean bAddToUIPickList = FALSE, // in :  
                         SmGfxArraySet * pOptGfxSet = NULL)  // in :  
                        const                                { smgfx_SetDrawingMode( SM_DM_CROSSHATCH ) ;
                                                               SmDisplayList * pRtn = Draw( bAddToUIPickList, pOptGfxSet ) ;
                                                               smgfx_SetDrawingMode( SM_DM_WIREFRAME ) ;
                                                               return(pRtn) ;
                                                             }
                       
  // Draw Shells with colored Faceuse normals to show Infinite, solid, and void regions (dNormalGain:[0.0] = guess from Shell BBox Size)
  SmDisplayList * DrawShells(double dNormalGain=0.0, 
                             SmGfxArraySet * pOptGfxSet=NULL)
                            const ;

  SmDisplayList * DrawCoincidences(SmBoolean       bDrawCoinVertices=TRUE, // in : TRUE = draw Coincident Vertices, FALSE=don't, default:[TRUE]                    
                                   SmBoolean       bDrawCoinEdges = TRUE,  // in : TRUE = draw Coincident Edges, FALSE=don't, default:[TRUE]                       
                                   SmBoolean       bDrawCoinFaces = TRUE,  // in : TRUE = draw Coincident Faces, FALSE=don't, default:[TRUE]                       
                                   SmGfxArraySet * pOptGfxSet = NULL)      // [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. 
                                  const ; 

  // for debugging: interactive display of face->loop->edge sequences
  void StepDrawFacesLoopsAndEdges() ;

  // get all memory used by this Brep, its topology graph objects, and their attributes
  ULONG GetMemoryUsed(ULONG &rlMemoryAllocated) const ; // note: increments unlocked mark value

  virtual SmBoolean AssertValid(SmAssertArray    * pAList=NULL,           // [in,out]: Accumulating list of failed Asserts, NULL to ignore                                             
                                SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                        
                                                                          //       : SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                    
                                                                          //       : default:[SM_LEVEL_0]                                                                              
                                SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]  
                                SmTArray<ULONG>  * pTestRequests=NULL)    // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                  
                               const ;

  // obsolete
  //  virtual SmBoolean AssertHeal 
  //(//   SmAssertReport & rAReport,  // in : a report generated by AssertValid  
  //   SmAssertArray * pAList      // in : AssertArray holding rAReport       
  // ) ;

  static SmBoolean AssertSubTopology(SmTopology &rObject,                          // in :                                                        
                                     SmAssertArray    * pAList = NULL,             // [in,out]: Accumulating list of failed Asserts, NULL to ignore 
                                     SmAssertTestLevel  eTestLevel = SM_LEVEL_0) ; // in :                                                        

  SmStatus DeleteStaleCaches() ;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmBrep, SmSAGObject, SmBrep_TYPE) ;

#ifdef SM_DEBUG_CODE
  // test the projection of every vertex to every face
  ULONG TestVertexFaceProjections() ;    // rtn: number of failed projections
  ULONG TestEdgeCurveIntervalLengths() ; // rtn: number of inconsistent intervals
#endif // SM_DEBUG_CODE

} ; // end class SmBrep

// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmBrep*) ;


//      // For debugging only - a set of pointers that can be assigned and inspected through the watch window
//      #ifdef SM_DEBUG_CODE
//
//      extern SmBrep *dbgBrep1 ;
//      extern SmBrep *dbgBrep2 ;
//
//      #endif // SM_DEBUG_CODE

class SmLockBrepBox
{
 private:
  SmBrep *m_pBrep ;        // Brep whose bounding box is being locked

 public:
  // constructor/destructor
  SmLockBrepBox(SmBrep *pBrep) { m_pBrep = pBrep;
                                 m_pBrep->LockBoundingBox() ;
                               } ;
 ~SmLockBrepBox()              { m_pBrep->UnLockBoundingBox() ;
                                 m_pBrep = NULL;
                               }

} ; // end class SmLockBrepBox

#endif // !__SMBREP_H__
