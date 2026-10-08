// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmStitch.h
* PURPOSE: Header file for SmStitch object.
**********************************************************************/

#ifndef __SMSTITCH_H__
#define __SMSTITCH_H__

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

/*******************************************************************//**
PURPOSE: This is the default stitching callback.  

NOTES: 
  This object has one method that decides which of two objects being
  stitched is to be kept and which to keep.
  
  Its implemented as a class so that new behaviors can be defined
  in derived classes.  

***********************************************************************/
class SM_EXPORT SmStitchCallback 
{
public:
  SmStitchCallback() {}

  virtual ~SmStitchCallback() {}

  // decide which of two coincident objects to keep and which to delete
  virtual SmStatus SelectTopologyToKeep 
  (
    SmTopology * cpTopologyElement1,  ///< [in] : 1st of two coincident objects to stitch together      <br>
    SmTopology * cpTopologyElement2,  ///< [in] : 2nd of two coincident objects to stitch together      <br>
    double       dDistBetween,        ///< [in] : max distance between coincident objects            <br>
    double     & rdMaxDistance,       ///< [out]: max distance found between two target objects      <br>
    SmBoolean  & rbKeepElement1       ///< [out]: TRUE = keep TopologyElement1                       <br>                                               
                                      ///<      : FALSE= keep TopologyElement2                       <br>
  );

} ; // end class SmStitchCallback                                                                                                 


/*******************************************************************//**
PURPOSE: This class provides a mechanism for stitching topological
    elements together. 

NOTES: 
***********************************************************************/
class SM_EXPORT SmStitch
{
private:                                          
                                                  
    SmStitchCallback & m_rStitchCallback;               // has method SelectTopologyToKeep(): to pick KeepItem and DeleteItem
                                                  //    from a glue (or squeeze) ItemPair
    double          m_dStitchTol3d;           // max 3d distance between coincident edges and vertices

public:
    SmBoolean       m_bDoRegionNesting;           // TRUE = find and set the infinite region by classifying a ray from a point outside the Brep
                                                  //        after glueing vertices and edges in DoStitching()
                                                  // default:[TRUE]

    SmBoolean       m_bValidateResult;            // TRUE = call ValidatePointers() after DoStitching() if compiled with SM_VALIDATE_TOPOLOGY
                                                  // default:[TRUE]

    SmBoolean       m_bIgnoreProblems;            // TRUE = Don't signal error and quit whenever 2 edges fail to glue in DoStitching()
                                                  // FALSE= Do    signal error and quit whenever 2 edges fail to glue in DoStitching()
                                                  // default:[FALSE]

    SmBoolean       m_bFastEdgeCompare;           // TRUE = only do a quick 5 point distance check when looking for coincident edges
                                                  //        do a precise distance check when looking for coincident edges
                                                  // default:[FALSE]
                                                   
    SmBoolean       m_bSqueezeSmallEdges;         // TRUE = remove and delete edges from topology graph 
                                                  //             in DoStitching() shorter than m_dStitchTol3d 
                                                  // default:[FALSE]

    SmBoolean       m_bSplitEdgesWithVertices;    // TRUE = split any edge that comes within tolerance of a vertex in DoStitching()
                                                  // default:[TRUE]

    SmBoolean       m_bMakingManifoldSolid;       // TRUE = only glue together pairs of lamina edges (and the vertices that attach to lamina edges)
                                                  //        modified SmBrep may contain coincident manifold edges.
                                                  // FALSE= glue any edges together possibly making nonManifold edges
                                                  // default:[TRUE]

    SmBoolean       m_bRemoveLaminarSlivers;      // TRUE = before stitching find and remove any sliver faces smaller than tolerance
                                                  // FALSE= don't
                                                  // default:[FALSE]

public:
    SmStitch
    (
      SmStitchCallback & rStitchCallback,
      double          dStitchTol3d,
      SmBoolean       bDoRegionNesting=TRUE,
      SmBoolean       bValidateResult=TRUE
    ) 
          : m_rStitchCallback(rStitchCallback), 
            m_dStitchTol3d(dStitchTol3d),
            m_bDoRegionNesting(bDoRegionNesting), 
            m_bValidateResult(bValidateResult),
            m_bIgnoreProblems(FALSE),
            m_bFastEdgeCompare(FALSE),
            m_bSqueezeSmallEdges(FALSE),
            m_bSplitEdgesWithVertices(TRUE),
            m_bMakingManifoldSolid(TRUE),
            m_bRemoveLaminarSlivers(FALSE)
          { }

    ~SmStitch() { }

    void SetStitchTol3d(double dStitchTol3d) { m_dStitchTol3d = dStitchTol3d; }

    // glue coincident vertex and edge pairs together, if asked
    //   split edges near vertices and/or squeeze out short edges
    // note: increments unlocked mark value
    SmStatus DoStitching
    (
      SmBrep                    * pBrepToStitch,                          // in : target Brep                                                          
      const SmTArray<SmVertex*> * cpOptCandidateVertices,                 // in : only glue coincident vertices on this list,                          
                                                                          //    : NULL = do all vertices                                               
      const SmTArray<SmEdge*>   * cpOptCandidateEdges,                    // in : only glue coincident edge pairs on this list,                        
                                                                          //    : NULL = do all edges                                                  
      ULONG                     & rlStitchedEdges,                        // out: number of edges stitched                                             
      ULONG                     & rlLaminaEdges,                          // out: number of lamina edges remaining after stitch                        
      double                    & rdMaxVertexGap,                         // out: max gap found between coincident vertices considered for gluing      
      double                    & rdMaxEdgeGap,                           // out: max gap found between coincident edges    considered for gluing      
                                                                          //    : NOTE: some coincident vertex and edge pairs do not get glued due to  
                                                                          //    :       a. gaps exceeding tolerances                                   
                                                                          //    :       b. geometries not listed within optional candidate lists       
                                                                          //    :       c. a failure within the glue edge function                     
                                                                          //    :       d. not being lamina when m_bMakingManifoldSolid == TRUE        
      double                    * pdMinUnstitchedVertGap = NULL,          // out: min gap found between vertices that did not get glued                
                                                                          //    : default:[NULL], NULL to ignore.                                      
      double                    * pdMinUnstitchedEdgeGap = NULL           // out: min gap found between edges that did not get glued                   
                                                                          //    : default:[NULL], NULL to ignore.                                      
    ); 

    // note: increments unlocked mark value
    SmStatus DoSimpleFaceStitch
    (
      SmBrep                  * pBrepToStitch,                               ///< [in] :      <br>
      const SmTArray<SmFace*> & rpFacesToKeep,                            ///< [out]:      <br>
      const SmTArray<SmFace*> & rpFacesToDelete,                          ///< [out]:      <br>
      double                  & rdMaxVertexGap,                           ///< [out]:      <br>
      double                  & rdMaxEdgeGap,                             ///< [out]:      <br>
      ULONG                   & rlStitchedEdges                               ///< [out]:      <br>
    );   
    
    static SmStatus StitchIntoSolid
    (
      SmBrep    * pBrepToStitch,                                             ///< [in] :     <br>
      SmBoolean & rbProducesASolid,                                       ///< [out]:     <br>
      ULONG     & rlStitchedEdges,                                            ///< [out]:     <br>
      double    & rdMaxVertexGap,                                         ///< [out]:     <br>
      double    & rdMaxEdgeGap                                            ///< [out]:     <br>
    );

    static SmStatus StitchIntoShell
    (
      SmBrep    * pBrepToStitch,                                             ///< [in] : target Brep                                <br>
      SmBoolean   bShellIsWellFormed,                                     ///< [in] : Only pass TRUE if you know that            <br>
                                                                          ///<      : the shell is of very good quality,         <br>
                                                                          ///<      : e.g., produced from a solid modeler.       <br>
                                                                          ///<      : Surface based models usually need this     <br>
                                                                          ///<      : to be FALSE                                <br>
      double      dMaxStitchingRatio,                                        ///< [in] : Max allowable separation between topology  <br>
                                                                          ///<      : objects that will get stitched here.           <br>
      ULONG     & rlStitchedEdges,                                            ///< [out]:                                            <br>
      double    & rdMaxVertexGap,                                         ///< [out]:                                            <br>
      double    & rdMaxEdgeGap                                            ///< [out]:                                            <br>
    );

    // note: increments unlocked mark value
    SmStatus UnifyNormals
    (
      SmBrep            * pBrepToOrient,
      SmFace            * pStartFace,
      ULONG               lNumSamples,
      SmTArray<SmFace*> & rFlippedFaces
    ); 

    // note: increments unlocked mark value
    SmStatus SelectRing
    (
      SmBrep                  * pBrepToOrient,
      const SmTArray<SmFace*> & crOriginalFaces,
      ULONG                     lNumSamples,
      SmTArray<SmFace*>       & rRingFaces,
      SmTArray<SmBoolean>     & rSameOrientation
    ); 

    // The following are for internal use.
    SmStatus StitchVertexPairs
    (
      SmTArray< SmVertex* > & rVertexList,                  ///< [in,out]: vertices to pair and glue - returned with glued vertices set to NULL           <br>
      SmTArray< SmVertex* > & rSurvivingVerts,              ///< [out]: List of surviving vertices                                                        <br>
      SmTArray< SmVertex* > & rDeletedVerts,                ///< [out]: List of deleted vertices
      SmTArray< SmEdge*   > & rDeletedEdges,                ///< [out]: List of Edges that were squeezed out                                              <br>
      double                & rdMaxVertexGap,               ///< [out]: largest gap seen between paired vertices                                          <br>
      double                & rdMinUnstitchedVertGap          ///< [out]: smallets gap seen between unpaired vertices                                       <br>
    );

    SmStatus StitchVertexPairs
    (
      SmTArray< SmVertex* > & rVertexList1,                 ///< [in,out]: kept vertices to pair and glue - returned with glued vertices set to NULL      <br>
      SmTArray< SmVertex* > & rVertexList2,                 ///< [in,out]: deleted vertices to pair and glue - returned with glued vertices set to NULL   <br>
      SmTArray< SmVertex* > & rProcessedVerts,              ///< [out]: added to, not reset.                                                              <br>
      SmTArray< SmVertex* > & rDeletedVerts,                ///< [out]: pointers to deleted vertices (Stale!)                                             <br>
      double                & rdMaxVertexGap,               ///< [out]: largest gap seen between paired vertices                                          <br>
      double                & rdMinUnstitchedVertGap          ///< [out]: smallets gap seen between unpaired vertices                                       <br>
    );

    SmStatus SplitEdgesWithVertices
    (
      SmTArray< SmVertex* > & sVerts,                        ///< [in,out]: Vertices they may split Edges. Entries set to Null when used.       <br>
      SmTArray< SmEdge  * > & rEdges,                        ///< [in,out]: In: Edges to split. Out: SplitEdges and NewEdges (i.e. all Edges)   <br>
      SmTArray< SmVertex* > & sProcessedVerts,               ///< [out]: Vertices that split an Edge.                                           <br>
      SmTArray< SmEdge  * > & rSplitEdges,                   ///< [out]: Edges that were split                                                  <br>
      SmTArray< SmEdge  * > & rNewEdges,                     ///< [out]: Edges that are created by the split                                    <br>
      double                & rdMaxVertexGap,                ///< [out]:                                                                        <br>
      double                & rdMinUnstitchedVertGap           ///< [out]:                                                                        <br>
    );

    SmStatus StitchEdgesOfVertices
    (
      SmTArray< SmVertex* > & sVerts,                        ///< [in] : Check these Vertices for coincident edges to stitch                       <br>
      SmTArray< SmEdge  * > & sGluedEdges,                   ///< [out]: Edges that survived being glued (extant)                               <br>
      SmTArray< SmEdge  * > & sDeletedEdges,                 ///< [out]: Edges that gluing deleted (stale pointers)                             <br>
      SmTArray< SmVertex* > & sProcessedVerts,               ///< NotUsed: [out]: Vertices that had Edges glued                                          <br>
      double                & rdMaxEdgeGap,                  ///< [out]:                                                                        <br>
      double                & pdMinUnstitchedEdgeGap             ///< [out]:                                                                        <br>
    );


} ; // end class SmStitch


#endif // !__SMSTITCH_H__
