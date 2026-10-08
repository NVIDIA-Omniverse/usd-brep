// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTopologyIntersector.h
* PURPOSE: Header file for SmTopologyIntersector object.
**********************************************************************/

#ifndef __SMTOPOLOGYINTERSECTOR_H__
#define __SMTOPOLOGYINTERSECTOR_H__

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMSURF_TYPES_H__
#include <SmSurfTypes.h>
#endif

#ifndef __SMEXTENT3D_H__
#include <SmExtent3d.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

#ifndef __SMSARRAY_H__
#include <SmSArray.h>
#endif

#ifndef __SMGLOBALSOLVER_H__
#include <SmGlobalSolver.h>
#endif

#ifndef __SMMAPPTRTOPTR_H__
#include <SmMapPtrToPtr.h>
#endif

#ifndef __SMSURFACE_H__
#include <SmSurface.h>
#endif

class SmTopologyIntersector;
class SmIntersectionTable;
class SmProtoTopologyManager;

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
class SmIntersectionResults
{
public:
    SmTArray<SmCurve*>         * m_p3DCurves;
    SmTArray<SmCurve*>         * m_pUVCurves1;
    SmTArray<SmCurve*>         * m_pUVCurves2;
    SmTArray<SmTsectCurveType> * m_pCurveTypes;
    SmTArray<double>           * m_pDeviations;
    
    SmIntersectionResults() : m_p3DCurves(NULL), 
                              m_pUVCurves1(NULL),
                              m_pUVCurves2(NULL), 
                              m_pCurveTypes(NULL), 
                              m_pDeviations(NULL) 
                            { }

} ; // end class SmIntersectResults

/*******************************************************************//**
PURPOSE: This table contains existing intersections.

NOTES: 
***********************************************************************/
class SmIntersectionTable : public SmObject
{
    friend class SmTopologyIntersector;
public:
    SmTArray<SmSurface*>            m_vBrepSurfaces;
    SmTArray<SmSurface*>            m_vOtherSurfaces;
    SmTArray<SmIntersectionResults> m_vResults;

             SmIntersectionTable() { }
    virtual ~SmIntersectionTable() ;

} ; // end class SmIntersectionTable

/*******************************************************************//**
PURPOSE: The topology intersector object provides the ability 
    to create edges and vertices corresponding to intersections
    in each Brep.  

NOTES: 
***********************************************************************/
class SM_EXPORT SmTopologyIntersector
{
  friend class SmMerge;

public:
  // algorithm control
  SmBoolean   m_bSkipSSI;                // TRUE = don't intersect surfaces.
                                         //   default:[FALSE]
  SmBoolean   m_bSkipManifoldEdges;      // TRUE = don't intersect Manifold edges, assume
                                         //        they are intersected with surfaces.
                                         //   default:[TRUE]
  SmBoolean   m_bIntersectTangentEdges;  // TRUE = check for tangent edges and add them add
                                         //        them to the list of edges to intersect.
                                         //   default:[TRUE]
  ULONG       m_lIntersectLaminaEdges;   // If 0 - no lamina edges to intersect - Default
                                         // If 1 - intersect lamina edges in both.
                                         // If 2 - intersect only lamina edges from Brep
                                         // If 3 - intersect only lamina edges from Other
  SmBoolean   m_bIntersectWireEdges;     // TRUE = add wire edges to list of edges to intersect.
                                         //   default:[TRUE]
  SmBoolean   m_bIntersectShellVertices; // TRUE = intersect shell vertices.
  SmBoolean   m_bIgnoreTangency;         // value is only passed to SmCurveClassification code where:
                                         // TRUE = work harder to remove coincident
                                         //        maybe which are usually tangency cases.
  SmBoolean   m_bDoGluing;               // TRUE = when coincident topology is found or created,
                                         //        glue the pairs, otherwise just Relate them.
                                         //        Not yet implemented.
                                         //   default:[FALSE]
  SmBoolean   m_bDoingFillets;           // TRUE = change coincident edge and vertex handling in SmCurveClassification for fillets (adds Fillet Data to SmCurveClass objs)
                                         // FALSE= Normal behavior used by Booleans and every other package.
                                         //   default:[FALSE]

  // algorithm state
  SmBoolean   m_bTopologyDeleted;        // TRUE = some topological objects have been
                                         //        deleted during subsequent operations.
private:
  const SmContext     & m_crContext;     // Context in which to create temporary data strucutres
                                         // used by the topology intersector.

  SmBrep              * m_pBrep;         // Brep which receives the merge
  SmBrep              * m_pOther;        // Brep to be merged into result
                                         
  SmMapPtrToPtr<SmObject, SmObject> * m_pBToO; // Map xsect created This  Brep objects to sibling Other objects
  SmMapPtrToPtr<SmObject, SmObject> * m_pOToB; // Map xsect created Other Brep objects to sibling This  objects
                                         
  SmBoolean             m_bSwappedOrder; // If TRUE we have swapped the order because
                                         // of the size of one of the objects.

  double                m_dThisApproxTol3d; // Tolerance used in creating curves.
  double                m_dThisAngTolRad;   // Angular tolerance used in creating curves - degrees.

  SmIntersectionTable * m_pIntersectionTable;
                                         // This table contains pre-existing surface/surface
                                         ///< [in ]:tersection results.

  SmMapPtrToPtr<SmObject, SmObject> * m_pSubset; // If given, this indicates a subset of m_pOther
                                                 //      objects to be intersected.
                                                 // The objects are geometry objects not topology objects.
                                                 //      Add SmSurface for each Face to include,
                                                 //      Add SmCurve   for each Edge to include,
                                                 //      Add SmVertex  for each Vertex to include.

  SmProtoTopologyManager * m_pProtoTopoMgr;
                                         // To store and handle ProtoTopology, during IntersectInsertRelate().

public:
    ~SmTopologyIntersector();

    SmTopologyIntersector(const SmContext & crContext);

    SmTopologyIntersector
    (
      const SmContext & crContext,
      SmBrep          * pBrep, 
      SmBrep          * pOther,
      double            dThisApproxTol3d,
      double            dthisAngTolRad
    );

    SmStatus CheckForSqueezeVertices
    (
      SmVertex            * pBrepVertex,         ///< [in ]: 1st of mated pair of vertices  <br>
      SmVertex            * pOtherVertex,        ///< [in ]: 2nd of mated pair of vertices  <br>
      SmTArray<SmVertex*> & rDeletedV,           ///< [out]: list of deleted vertices       <br>
      SmTArray<SmVertex*> & rSurvivingV,         ///< [out]: list of surviving vertices     <br>
      SmTArray<SmEdge*>   & rDeletedE            ///< [out]: list of deleted edges          <br>
    );

    SmStatus CheckIfHaveExistingSSI
    (
      const SmContext            & crContext,
      const SmSurface            * pBrepSurface,
      const SmSurface            * pOtherSurface,
      SmBoolean                  & rbHaveExistingSSI,
      SmTArray<SmCurve*>         & r3DCurves,
      SmTArray<SmCurve*>         & rUVCurves1,
      SmTArray<SmCurve*>         & rUVCurves2,
      SmTArray<SmTsectCurveType> & rOrientations,
      SmTArray<double>           & rDeviations
    );

    SmStatus RegisterExistingSSI
    (
      SmSurface                        * pBrepSurface,         ///< [in ]: surf1                                      <br>
      SmSurface                        * pOtherSurface,        ///< [in ]: surf2                                      <br>
      const SmTArray<SmCurve*>         & cr3DCurves,           ///< [in ]: 3d xsect curves                            <br>
      const SmTArray<SmCurve*>         & crUVCurves1,          ///< [in ]: associated surf1 UVTrimCurves              <br>
      const SmTArray<SmCurve*>         & crUVCurves2,          ///< [in ]: associated surf2 UVTrimCurves              <br>
      const SmTArray<SmTsectCurveType> & crOrientations,       ///< [in ]: associated intersection curve types        <br>
      const SmTArray<double>           & crDeviations          ///< [in ]: associated intersection curve deviations   <br>
    );

    SmStatus ClearExistingSSI();

    SmStatus CloseIntersectionLoop
    (
      SmVertex  * pBrepVertex,                                 ///< [in ]: target vertex from m_pBrep     <br>
      SmBoolean & rbMadeNewTopology                            ///< [out]: TRUE =                         <br>
                                                               ///<      : FALSE=                         <br>
    );

    SmStatus DeleteFaces
    (
      const SmTArray<SmFace*> & crFaces,                       ///< [in ]: Brep faces to delete                                <br>
      const SmTArray<SmEdge*> * pOptKeepAsWires = NULL         ///< [in ]: list of edges to turn into wires if all the faces   <br>
                                                               ///<      : to which they are attached happen to get deleted.   <br>
                                                               ///<      : Edges not on this list are deleted when all         <br>
                                                               ///<      : the faces to which they are attached are deleted.   <br>
                                                               ///<      : NULL to ignore, default:[NULL].                     <br>
    ) ;

    SmStatus FixGaps
    (
      const SmTArray<SmVertex*> & crSingleVertices,            ///< [in ]: list of single vertices   <br>
      SmBoolean                 & rbModifiedTopology,          ///< [out]: was topology modified?    <br>
      SmTArray< SmVertex* >     & rDeletedVertices,            ///< [out]: deleted verts in m_pBrep  <br>
      SmTArray< SmEdge*   >     & rDeletedEdges                ///< [out]: deleted edges in m_pBrep  <br>
    );

    SmStatus FixupClassMates
    (
      SmCurveClassification & rCurveClassBrep,                 ///< [in ]: first member of a pair of homogonized curveClassifications  <br>
      SmCurveClassification & rCurveClassOther                 ///< [in ]: the other member of the pair                                <br>
    ) ;

    SmStatus FixupEndPointClassMates
    (
      ULONG lPointDim,                                         ///< [in ]: Classification point dim, 2 or 3                     <br>
      SmPointClassification & rPCBrep,                         ///< [in ]: 1st of pair of matching end point classifications    <br>
      SmPointClassification & rPCOther,                        ///< [in ]: the other member of the pair                         <br>
      const SmCurve         * cpClassificationCurve            ///< NotUsed: [in ]: pointer to curve being classified -                  <br>
                                                               ///<      : currently used for error recovery and debugging      <br>
    ) ;

    SmStatus FixupMidPointClassMates
    (
      ULONG lPointDim,                                         ///< [in ]: Classification point dim, 2 or 3                   <br>
      SmPointClassification & rPCBrep,                         ///< [in ]: 1st of pair of matching mid point classifications  <br>
      SmPointClassification & rPCOther,                        ///< [in ]: the other member of the pair                       <br>
      const SmCurve         * cpClassificationCurve            ///< [in ]: pointer to curve being classified -                <br>
                                                               ///<      : currently used for error recovery and debugging    <br>
    ) ;

    SmStatus FixupSplitVertexPair
    (
      ULONG lIntervalIndex,                                    ///< [in ]: index of target interval                                     <br>
      SmCurveClassification & rCurveClassBrep,                 ///< [in ]: first member of a pair of homogonized curveClassifications   <br>
      SmCurveClassification & rCurveClassOther,                ///< [in ]: the other member of the pair                                 <br>
      SmBoolean             & bChangedClassifications          ///< [out]: TRUE = changed input curveClassifications                    <br>
                                                               ///<      : FALSE= didn't                                                <br>
    ) ;

    SmStatus IntersectInsertRelate();

    // Methods used by IntersectInsertRelate():
    SmStatus IntersectTopology();

    SmStatus ResolveIntersections();

    SmStatus ImprintIntersections();

    SmStatus SetRelationships();


    SmStatus IIRSurfs
    (
      SmSurface        * pSurface,                   ///< [in ]: target surface                              <br>
      SmSurface        * pOtherSurface,              ///< [in ]: other target surface to be intersected      <br>
      const SmExtent3d * cpIntersectionBBox = NULL   ///< [in ]: a limiting region of interest               <br>
    );

    SmStatus IIRVertexBrep
    (                                               
      SmVertex * pVertex,                            ///< [in ]: Target Vertex
      SmBrep   * pBrep,                              ///< [in ]: Target Brep to modify                               <br>
      SmBoolean  bSwapOrder                          ///< [in ]: TRUE = pVertex from m_pOther and pBrep is m_pBrep   <br>
                                                     ///<      : FALSE= pVertex from m_pBrep and pBrep is m_pOther   <br>
    );

    SmStatus IIREdgeBrep
    (
      SmEdge                * pEdge,                             ///< [in ]: target edge from OtherBrep                                              <br>
      SmBrep                * pBrep,                             ///< [in ]: target brep                                                             <br>
      SmBoolean               bSwapOrder,                        ///< [in ]: TRUE  = call MergeCurveClasses(pEdgeClassification, pBrepClassification)<br>
                                                                 ///<      : FALSE = call MergeCurveClasses(pBrepClassification, pEdgeClassification)<br>
      SmBoolean               bForceFaceIntersections = FALSE,   ///< [in ]: TRUE  = always intersect pEdge with pBrep->Surfaces                     <br>
                                                                 ///<      : FALSE = don't when pEdge is a wire                                      <br>
                                                                 ///<      : note: All calls to IIREdgeBrep are currently being made                 <br>
                                                                 ///<      : with bForceFaceIntersections == TRUE.                                   <br>
      SmTArray< SmEdge  * > * paDeletedEdges = NULL,             ///< [out]: ptrs to deleted edges                                                   <br>
      SmTArray< SmVertex* > * paDeletedVerts = NULL              ///< [out]: ptrs to deleted vertices                                                <br>
    );

    SmStatus Relate
    (                                                            
      SmObject * pBrepTopo,                                      ///< [in ]: Object in ThisBrep to relate    <br>
      SmObject * pOtherTopo                                      ///< [in ]: Object in OtherBrep to relate   <br>
    );

    const SmContext & GetContext()  const
    {
      return(m_crContext);
    }

    void GetCommonEdges       
    (
      SmTArray<SmEdge*> & rEdges, 
      SmTArray<SmEdge*> & rOtherEdges
    ) const;

    void  GetCommonVertices    
    (
      SmTArray<SmVertex*> & rVertices, 
      SmTArray<SmVertex*> & rOtherVertices
    ) const;

    SmMapPtrToPtr<SmObject, SmObject> * GetSubset() const
    {
      return m_pSubset;
    }
                                           
    SmObject        * GetOtherMate         (SmObject * pBrepEntity)               const;

    SmObject        * GetThisMate          (SmObject * pBrepEntity)               const;

    SmBoolean         IsMatedPair          (SmObject * pBrepEntity, SmObject * pOtherEntity) const;

    SmObject        * GetBrepMate          (SmObject * pOtherEntity)              const; 


    void GetMappedOtherObjects
    (
      SM_TYPE               eDesiredType,         ///< [in ]: oneof SmEdge_TYPE, SmVertex_TYPE, SmFace_TYPE, etc.   <br>
      SmTArray<SmObject*> & rList                 ///< [in ]: Display point size                                    <br>
    ) const;

    void GetMappedBrepObjects
    (
      SM_TYPE                eDesiredType,        ///< [in ]: oneof SmEdge_TYPE, SmVertex_TYPE, SmFace_TYPE, etc    <br>
      SmTArray<SmObject *> & rList                ///< [in ]: Display point size                                    <br>
    ) const ;

    // call Notify(SM_NO_COINCIDENT,...) on every MappedObject pair for attribute propagation
    void CoincidenceNotify() const ;

    SmStatus MergeCurveClasses
    (
      SmCurveClassification & rCrvClass1,                 ///< [in,out]: 1st Target Curve Classification             <br>
      SmCurveClassification & rCrvClass2,                 ///< [in,out]: 2nd Target Curve Classification             <br>
      SmBoolean               bMergeSingularities,        ///< [in ]: TRUE = Merge all point classifications         <br>
      SmBoolean             & rbTopologyWasDeleted,       ///< [out]: TRUE = small edges were deleted by squeeze     <br>
      SmTArray< SmEdge* >   * paDeletedEdges = NULL,      ///< [out]: ptrs to deleted edges                          <br>
      SmTArray< SmVertex* > * paDeletedVertices = NULL    ///< [out]: ptrs to deleted vertices                       <br>
    );

    SmStatus MergeCoincidentSurfaces
    (
      SmSurface         * pBSurface,                      ///< [in ]: 1st surface coincident with 2nd surface                <br>
      SmSurface         * pOSurface,                      ///< [in ]: 2nd surface coincident with 1st surface                <br>
      SmTArray<SmFace*> & rFaces,                         ///< [out]: list of BSurface faces coincident with OSurface faces  <br>
      SmTArray<SmFace*> & rOFaces                         ///< [out]: list of OSurface faces coincident with BSurface faces  <br>
    ); 

    // for a curve on 2 surfaces in different Breps, create & insert edges into the topology graphs 
    SmStatus MergeCurveOnSurfaces
    (
      SmSurface * pSurface,                              ///< [in ]: 1st Surface containing 3DCurve                           <br>
      SmSurface * pOtherSurface,                         ///< [in ]: 2nd Surface containing 3DCurve                           <br>
      double      dCurveTolerance,                       ///< [in ]: max length for a degenerateCurve --                      <br>
                                                         ///<      : a degenerateCurve is treated as a point                  <br>
      SmCurve   * p3DCurve,                              ///< [in ]: Curve known to lie on 2 surfaces                         <br>
      SmCurve   * pUVCurve1,                             ///< [in ]: 3DCurve UVTrimCurve on pSurface or NULL to ignore        <br>
      SmCurve   * pUVCurve2,                             ///< [in ]: 3DCurve UVTrimCurve on pOtherSurface or NULL to ignore   <br>
      SmEdge    * pBrepEdge,                             ///< [in ]: edge already representing 3DCurve in 1st Brep or NULL    <br>
      SmEdge    * pOtherEdge,                            ///< [in ]: edge already representing 3DCurve in 2nd Brep or NULL    <br>
      SmBoolean & rbTopologyWasDeleted,                  ///< [out]: TRUE iff vertices and edges were deleted                 <br>
                                                         ///<      : when multiply mated vertices are squeezed                <br>
      SmTArray< SmEdge* >   *paDeletedEdges    = NULL,   ///< [out]: ptrs to deleted edges                                    <br>
      SmTArray< SmVertex* > *paDeletedVertices = NULL    ///< [out]: ptrs to deleted vertices                                 <br>
    );

    SmStatus RemoveRelationship
    (
      SmTopology * pBrepEntity,
      SmTopology * pOtherEntity
    );

    SmStatus ReIntersectAtVertex
    (                                                     ///< [in ]:          <br>
      SmVertex  * pBrepVertex,                            ///< [in ]:          <br>
      SmVertex  * pOtherVertex,                           ///< [out]:          <br>
      SmBoolean & rbMadeNewTopology
    );

    SmStatus SetSubset(const SmTArray<SmObject*> & crSubsetGeometry);

    // Simple member access.
    SmBrep * GetPrimaryBrep()     const  { return m_pBrep;  }

    SmBrep * GetOtherBrep()       const  { return m_pOther; }

    double   GetThisApproxTol3d() const  { SM_ASSERT_TOL(m_dThisApproxTol3d) ; return m_dThisApproxTol3d; }

    double   GetThisAngTolRad()   const  { SM_ASSERT_TOL(m_dThisAngTolRad) ; return m_dThisAngTolRad; }

    void SetPrimaryBrep    (SmBrep *pBrep)           { m_pBrep            = pBrep;  }

    void SetOtherBrep      (SmBrep *pOther)          { m_pOther           = pOther; }

    void SetThisApproxTol3d(double dThisApproxTol3d) { m_dThisApproxTol3d = dThisApproxTol3d; }

    void SetThisAngTolRad  (double dThisAngTolRad)   { m_dThisAngTolRad   = dThisAngTolRad ; }

    // return TRUE if every mapped m_pBrep edge is connected to
    // vertices that are connected to at least two mapped edges
    SmBoolean AreIntersectionLoopsClosed() const;


    // The following six methods are generally for internal use.
    // They make use of already-processed topology.
    SmStatus VertexFaceIntersect
    ( 
      SmTArray< SmVertex* > & rVerts,          ///< [in,out]: entries set to Null when used.      <br>
      SmTArray< SmFace  * > & rFaces,          ///< [in ]:                                        <br>
      SmTArray< SmVertex* > & rProcessedVerts, ///< [out]:                                        <br>
      SmTArray< SmFace  * > & rProcessedFaces, ///< [out]:                                        <br>
      double & rdMaxVertexGap,                 ///< [out]:                                        <br>       
      double & rdMinUnstitchedVertGap,           ///< [out]:                                        <br>    
      SmBoolean bMakingManifoldSolid,          ///< NotUsed: [in ]:                               <br>            
      SmBoolean bIgnoreProblems                ///< [in ]:
    );

    SmStatus EdgeEdgeIntersect
    ( 
      SmTArray< SmEdge  * > & rInputEdges1,       ///< [in ]:           <br>
      SmTArray< SmEdge  * > & rInputEdges2,       ///< [in ]:           <br>
      SmTArray< SmEdge  * > & rSplitEdges1,       ///< [out]:           <br>
      SmTArray< SmEdge  * > & rNewEdges1,         ///< [out]:           <br>
      SmTArray< SmEdge  * > & rSplitEdges2,       ///< [out]:           <br>
      SmTArray< SmEdge  * > & rNewEdges2,         ///< [out]:           <br>
      SmTArray< SmVertex* > & rNewVerts,          ///< [out]:           <br>
      double & rdMaxEdgeGap,                      ///< [out]:           <br>
      double & rdMinUnstitchedEdgeGap,                ///< [out]:           <br>
      SmBoolean bMakingManifoldSolid,             ///< NotUsed: [in ]:           <br>
      SmBoolean bIgnoreProblems                   ///< [in ]:           <br>
    );

    SmStatus EdgeFaceIntersect
    ( 
      SmTArray< SmEdge*   > & rEdges,             ///< [in ]: unchanged     <br>
      SmTArray< SmFace*   > & rFaces,             ///< [in ]: unchanged     <br>
      SmTArray< SmVertex* > & rNewVerts,          ///< [out]:               <br>
      SmTArray< SmEdge  * > & rSplitEdges,        ///< [out]:               <br>
      SmTArray< SmEdge  * > & rNewEdges,          ///< [out]:               <br>
      SmTArray< SmFace  * > & rProcessedFaces,    ///< [out]:               <br>
      double & rdMaxVertexGap,                    ///< [out]:               <br>
      double & rdMinUnstitchedVertGap,              ///< [out]:               <br>
      SmBoolean bMakingManifoldSolid,             ///< NotUsed: [in ]:           <br>
      SmBoolean bIgnoreProblems                   ///< [in ]:           <br>
    );

    SmStatus EdgeFaceCoincidence
    ( 
      const SmTArray< SmVertex* > & rGivenVerts,     ///< [in ]:
      const SmTArray< SmEdge  * > & rExcludedEdges,  ///< [in ]:  don't try to imprint these.                              <br>
      SmTArray< SmEdge  * >       & rImprintedEdges, ///< [out]: edges imprinted into a face.                              <br>
      SmTArray< SmFace  * >       & rProcessedFaces, ///< [out]: faces with edges imprinted in them, including new faces.  <br>
      SmTArray< SmFace  * >       & rNewFaces,       ///< [out]: if an imprinted edge split a face.                        <br>
      SmTArray< SmFace  * >       & rSplitFaces,     ///< [out]: what sNewFaces were split from.                           <br>
      double & dMaxEdgeGap,
      double & dThisMinUnstitched,
      SmBoolean bIgnoreProblems,
      SmBoolean bDoRegionNesting 
    );

    SmStatus FaceFaceIntersect
    ( 
      SmTArray< SmVertex * > & rInputVerts,     ///< [in ]:                                   <br>
      SmTArray< SmFace   * > & rFaceList1,      ///< [in,out]:                                <br>
      SmTArray< SmFace   * > & rFaceList2,      ///< [in,out]:                                <br>
      SmTArray< SmFace   * > & rProcessedFaces, ///< [out]:                                   <br>
      SmTArray< SmFace   * > & rNewFaces,       ///< [out]: if faces were split               <br>
      SmTArray< SmEdge   * > & rNewEdges,       ///< NotUsed: [out]:                                   <br>
      double & rdMaxGap,                        ///< NotUsed: [out]:                            <br>
      double & rdMinUnstitched,                     ///< NotUsed: [out]:                            <br>
      SmBoolean bIgnoreProblems,                ///< [in ]:  not used in this routine.        <br>
      SmBoolean bDoRegionNesting 
    ); 

    SmStatus FaceFaceCoincidence
    ( 
      SmTArray< SmEdge * > & rInputEdges,       ///< [in ]: coincident faces will contain these edges  <br>
      SmTArray< SmFace * > & rFacesToDelete,    ///< [in ]: see Usage Notes.                           <br>
      SmTArray< SmFace * > & rProcessedFaces,   ///< [out]: surviving coincident faces                 <br>
      SmTArray< SmFace * > & rDeletedFaces,     ///< [out]: deleted   coincident faces                 <br>
      double & rdMaxGap,                        ///< [out]:                                            <br>
      double & rdMinUnstitched,                     ///< [out]:                                            <br>
      SmBoolean m_bIgnoreProblems,              ///< [in ]:                                            <br>
      SmBoolean m_bDoRegionNesting              ///< NotUsed: [in ]:  not used in this routine.                 <br>
    );


    // Utilities
    // draw mapped objs in default color
    void Draw        (double dThisLineWidth, double dOtherLineWidth, double dThisPointSize, double dOtherPointSize) const; 

    // draw mapped objs in default color
    void DrawOneAssoc(ULONG iIndx, double dThisLineWidth, double dOtherLineWidth, double dThisPointSize, double dOtherPointSize) const; 

    // draw maaped objs in varying colors, print mapped obj data
    void DrawDetails (double dLineWidth, double dPointSize) const; 

    void Dump        (SmBoolean bValidPtrs=TRUE, SmBoolean bDumpMapObjects=FALSE) const;

} ; // end class SmTopologyIntersector


#endif // !__SMTOPOLOGYINTERSECTOR_H__

