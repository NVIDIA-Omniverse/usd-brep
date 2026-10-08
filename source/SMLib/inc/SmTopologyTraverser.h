// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmTopologyTraverser.h
* PURPOSE: Header file for SmTopologyTraverser object.
**********************************************************************/

#ifndef __SMTOPOLOGYTRAVERSER_H__
#define __SMTOPOLOGYTRAVERSER_H__

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMTARRAY_H__
#include <SmTArray.h>
#endif

/*******************************************************************//**
PURPOSE: The topology traverser object provides the ability 
    to traverse the topology in a Brep and collect or mark things.  

NOTES: 
***********************************************************************/
class SM_EXPORT SmTopologyTraverser
{
private:

public:
  SmTopologyTraverser() {}
 ~SmTopologyTraverser() {}

  // get all unmarked faces connected to StartFace through unmarked edge/face connections.
  // mark the collected faces (eMarkType) and their edges and vertices.
  // DOES NOT increment eMarkType mark value.  
  SmStatus CollectFaces
  (
    SmFace               * pStartFace,                     ///< [in ]: seed face                                                      <br>
    SmTArray<SmFace*>    & rCollectedFaces,                ///< [out]: unmarked faces connected to seed through unmarked edges        <br>
    SmMarkType             eMarkType = SM_MT_MARK2,        ///< [in ]: mark type to check, not incremented, default:[SM_MT_MARK2]     <br>
    SmTArray<SmEdge*>    * pOptCollectedEdges = NULL,      ///< [in ]: optional unmarked edges connected to any CollectedFace         <br>
                                                           ///<      : NULL to ignore, default:[NULL]                                 <br>
    SmTArray<SmVertex *> * pOptCollectedVertices = NULL    ///< [in ]: optional unmarked vertices connected to any CollectedFace      <br>
  );

  // get all unmarked wires connected to startWireEdge through unmarked edge/vertex connections.
  // mark the collected wires (eMarkType) and their vertices.
  // DOES NOT increment eMarkType mark value.  
  SmStatus CollectWireEdges
  (
    SmEdge               * pStartWireEdge,                 ///< [in ]: seed edge                                                      <br>
    SmTArray<SmEdge*>    & rCollectedEdges,                ///< [out]: unmarked wire edges connected to seed edge                     <br>
    SmMarkType             eMarkType = SM_MT_MARK2,        ///< [in ]: mark type to check, not incremented, default:[SM_MT_MARK2]     <br>
    SmTArray<SmVertex *> * pOptCollectedVertices = NULL    ///< [in ]: optional unmarked vertices connected to any CollectedEdge      <br>
  );

  // get all unmarked laminas connected to startLaminaEdge through unmarked edge/vertex connections.
  // mark the collected lamina (eMarkType) and their vertices.
  // DOES NOT increment eMarkType mark value.  
  SmStatus CollectLaminaEdges
  (
    SmEdge               * pStartLaminaEdge,               ///< [in ]: seed edge                                                      <br>
    SmTArray<SmEdge*>    & rCollectedEdges,                ///< [out]: unmarked lamina edges connected to seed edge                     <br>
    SmMarkType             eMarkType = SM_MT_MARK2,        ///< [in ]: mark type to check, not incremented, default:[SM_MT_MARK2]     <br>
    SmTArray<SmVertex *> * pOptCollectedVertices = NULL    ///< [in ]: optional unmarked vertices connected to any CollectedEdge      <br>
  );

  // get all curves in the same loop as pCrv on face
  // NO USE of marks.
  SmStatus CollectCurveLoop 
  (
    const SmCurve *,                    ///< [in ]:  Curve                                            <br>
    const SmFace  *,                    ///< [in ]: (optional) Face (or NULL)                         <br>
    SmTArray<SmCurve *> &               ///< [out]: list of curves (including pCrv) in same loop      <br>
  );     

  // Collect all regions connected to the given Region through Faceuses.
  // NO USE of marks.
  SmStatus CollectRegions 
  (
    const SmRegion *pStartRegion,       ///< [in ]:                                                            <br>
    SmTArray<SmRegion *> &rRegions      ///< [out]: list of connected Regions (not including pStartRegion)     <br>
  );

  // Find out if a faceuse is part of a closed shell by traversing faceuse/edgeuse connections.
  SmStatus ClosureTraversal
  (
    SmFaceuse            * pStartFaceuse, ///< [in ]: target faceuse (gets marked)                                 <br>
    SmBoolean            & rbIsClosed,    ///< [out]: TRUE=faceuse connects to closed faceuse set, FALSE=doesn't   <br>                                                             
    SmTArray<SmFaceuse*> * pOptFaceuses,  ///< [out]: List of all connected faces (get marked), NULL to ignore.    <br>
    SmNewMarkAndLock     & rMark          ///< [in ]: specify mark for target objects (incremented)                <br>
  );

  // Order an array of Edges into topological chains
  static SmStatus FixEdgeChainList
  ( 
    SmTArray<SmEdge*> & rEdges,             ///< [in,out]: List of edges to be checked      <br>
    SmTArray<ULONG>   & rChainStart         ///< [out]:                                     <br>
  );

  // Find out if a set of edges form a single closed loop involving all edges by traversing vertexuse connections
  static SmStatus EdgeLoopTraversal
  (
    const SmTArray<SmEdge*>  & rEdges,      ///< [in ]: List of edges to be checked                     <br>
    SmBoolean          & rIsClosed,         ///< [out]: TRUE  if rEdges forms a closed loop             <br>
    SmTArray<ULONG>    * pOrder = NULL      ///< [out]: Optional ordering of edges                      <br>
  );


  // traverse all unmarked faceuses connected to StartFaceuse through unmarked edgeuse/faceuse connections
  SmStatus FaceuseTraversal
  (
    SmFaceuse            * pFaceuse,      ///< [in ]: target faceuse (gets marked)                                  <br> 
    SmBoolean            & rbIsClosed,    ///< [out]: TRUE=faceuse connects to closed faceuse set, FALSE=doesn't    <br>
    SmTArray<SmFaceuse*> * pOptFaceuses,  ///< [out]: List of all connected faces (get marked), NULL to ignore.     <br>
    SmNewMarkAndLock     & rMark          ///< [in ]: specify mark for target objects (not incremented)             <br>
  );

  // Move all edgeuses and faceuses from shell being removed to shell being saved
  // NO USE of marks.
  SmStatus RemoveShell
  (
    SmShell *pShellToRemove,      ///< [in ]: target shell being removed                    <br>
    SmShell *pShellToReplace      ///< [in ]: owner shell to contain removeShell children   <br>
  );

  // Replace the shells of all faceuses which can be traversed by connectivity.
  SmStatus FaceuseShellReplacement
  (
    SmFaceuse            * pFaceuse,          ///< [in ]: target faceuse (gets marked)                      <br>
    SmShell              * pNewShell,         ///< [in ]: new Shell to hold connected faceuses              <br>
    SmTArray<SmFaceuse*> & rReplacedFaceuses, ///< [in ]: list of faceuses moved to NewShell (get marked)   <br>
    SmNewMarkAndLock     & rMark              ///< [in ]: specify mark for target objects (incremented)     <br> 
  );

  // mark vertices, faces, vertexuses, and faceuses connected to StartVertex through a target shell
  //  optionally collect all shell edgeuses and/or faceuses on lists.
  //  side-effect: increment eMarkType mark value
  SmStatus ShellTraversal
  (
    SmVertex             * pStartVertex,         ///< [in ]: target vertex                                    <br>
    SmShell              * pShellToTraverse,     ///< [in ]: target shell                                     <br>
    SmTArray<SmEdgeuse*> * pOptEdgeusesTouched,  ///< [out]: optional list of edges in shell (not marked)     <br>
    SmTArray<SmFaceuse*> * pOptFaceusesTouched,  ///< [out]: optional list of faces in shell (get marked)     <br>
    SmNewMarkAndLock     & rMark                 ///< [in ]: specify mark for target objects (incremented)    <br>
  );

  // Determine whether a closed collection of Faceuses is inner or outer.
  static SmStatus IsInnerShell
  ( 
    SmTArray< SmFaceuse* > &rFaceuses,            ///< [in ]: Facesuses of a closed shell to be checked               <br>
    SmBoolean & rbIsClosed                        ///< [out]: TRUE = is isomorphic to sphere's inside facing shell    <br>
                                                  ///<      : FALSE= is isomorphic to sphere's outside facing shell   <br>
  );

  // Add number attribute to all faces based on crease-angle
  SmStatus NumberFacesByCreaseAngle
  (
    SmBrep   *pBrep,          ///< [in ]:                                                                <br>
    double    angle,          ///< [in ]: Max angle (degrees) between faces for same face-number         <br>
    double    MinEdgeSize,    ///< [in ]: Dont test creases less than this length                        <br>
    int       FaceNumberId,   ///< [in ]: Attribute number to use for facenumbering  (use 9518)          <br>
    int      *pNewNumber      ///< [out]: Next Number to use for next face group                         <br>
                              ///<      : next number updated for every new face number created          <br>
  );

  // add all topology objects found in the topology graph under the input object to output arrays - all outputs are optional
  static SmStatus GetSubTopology
  (
    const SmTopology     &rObject,                  ///< [in ]: A member of an existing tree (not a use object)                                                <br>
    SmTArray<SmRegion *> *pRegions=NULL,            ///< [out]: opt unique regions list,  NULL to ignore, default:[NULL]                                       <br>
    SmTArray<SmShell  *> *pShells=NULL,             ///< [out]: opt unique shells list,   NULL to ignore, default:[NULL]                                       <br>
    SmTArray<SmFace   *> *pFaces=NULL,              ///< [out]: opt unique faces list,    NULL to ignore, default:[NULL]                                       <br>
    SmTArray<SmLoop   *> *pLoops=NULL,              ///< [out]: opt unique loops list,    NULL to ignore, default:[NULL]                                       <br>
    SmTArray<SmEdge   *> *pEdges=NULL,              ///< [out]: opt unique edges list,    NULL to ignore, default:[NULL]                                       <br>
    SmTArray<SmVertex *> *pVertices=NULL,           ///< [out]: opt unique vertices list, NULL to ignore, default:[NULL]                                       <br>
    SmBoolean             bReSetArrays=TRUE,        ///< [in ]: TRUE = reset arrays, FALSE = accumulate within arrays, Default:[TRUE]                          <br>
    SmMarkType            eMarkType=SM_MT_NOMARK    ///< [in ]: mark type to check, SM_MT_NOMARK = skip mark check, mark not incremented, default:[SM_MT_MARK2]<br>
  );

  // add all topology objects found in the topology graph under the input object to output arrays - all outputs are optional
  static SmStatus GetSubTopologies
  (
    SmTArray<SmTopology *> &rObjects,                 ///< [in ]: members of an existing tree (not a use object)                                                 <br>
    SmTArray<SmRegion *>   *pRegions=NULL,            ///< [out]: opt unique regions list,  NULL to ignore, default:[NULL]                                       <br>
    SmTArray<SmShell  *>   *pShells=NULL,             ///< [out]: opt unique shells list,   NULL to ignore, default:[NULL]                                       <br>
    SmTArray<SmFace   *>   *pFaces=NULL,              ///< [out]: opt unique faces list,    NULL to ignore, default:[NULL]                                       <br>
    SmTArray<SmLoop   *>   *pLoops=NULL,              ///< [out]: opt unique loops list,    NULL to ignore, default:[NULL]                                       <br>
    SmTArray<SmEdge   *>   *pEdges=NULL,              ///< [out]: opt unique edges list,    NULL to ignore, default:[NULL]                                       <br>
    SmTArray<SmVertex *>   *pVertices=NULL,           ///< [out]: opt unique vertices list, NULL to ignore, default:[NULL]                                       <br>
    SmBoolean               bReSetArrays=TRUE,        ///< [in ]: TRUE = reset arrays, FALSE = accumulate within arrays, Default:[TRUE]                          <br>
    SmMarkType              eMarkType=SM_MT_NOMARK    ///< [in ]: mark type to check, SM_MT_NOMARK = skip mark check, mark not incremented, default:[SM_MT_MARK2]<br>
  );

}; // end class SmTopologyTraverser

#endif // !__SMTOPOLOGYTRAVERSER_H__


