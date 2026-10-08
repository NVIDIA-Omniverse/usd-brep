// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************//**
* FILE NAME --- SmCFace.h
* PURPOSE: Header file for SmCFace class. 
**********************************************************************/

#ifndef __SMCFACE_H__
#define __SMCFACE_H__

// Remove Composites - removed entire class definition
// 
// 
// #ifndef __SMOS_TYPES_H__
// #include <SmTypes.h>
// #endif
// 
// #ifndef __SMFACE_H__
// #include <SmFace.h>
// #endif
// 
// 
// /*******************************************************************//**
// PURPOSE: This class represents a collection of topological faces
//      which have the same underlying surface in a Brep.  It is a Composite
//      Face representation.
// 
// NOTES:
//   Composite Face Representation
//      Each composite face is represented by a set of objects including
//      a. One SmCFace object containing a list of SmFace object pointers.
//      b. The set of composite face sibling SmFace objects.
// 
//   SmBrep Composite Face Management
//      a. The composite Face is placed on the         SmBrep::m_pCFaceListHead list
//      b. Each sibling SmFace object is placed on the SmBrep::m_pFaceListHead list
// 
//   Composite Face Surface, Domain, and Tolerance
//      a. The SmCFace and all of its Sibling SmFace objects use the same Surface.
//      b. The owner of the Surface, stored in SmSurface::m_pOwner is the SmCFace object.
//      c. The SmCFace Domain is the union of all its sibling SmFace objects.
//      d. The SmCFace Tolerance is the maximum of all its sibling SmFace objects
// 
//   HINT: A SmFace composite face sibling will not be the owner of its surface.
//         An SmFace not in a composite face will be the owner of its surface.
//          
//         A quick test to tell if an SmFace object is a sibling face of 
//         an SmCFace object is to check to see if the Face->Surface->Owner == Face. 
// 
// ***********************************************************************/
// class SM_EXPORT SmCFace : public SmFace
// {
//   friend class SmBrep;
// 
//   // inherited:
//   // SmTopology::m_pListOwner - when in a Brep - SmBrep::m_pCFaceListHead
//   // SmTopology::m_pNext      - Next member of CFace list owned by SmBrep::m_pCFaceListHead
//   // SmTopology::m_pLast      - Last member of CFace list owned by SmBrep::m_pCFaceListHead 
//   //
//   // SmOwningTopology::m_pList      - 
//   // SmOwningTopology::m_lListSize  - 
// 
// protected:
//   SmTArray<SmFace*> * m_pFaces ;    // All of the faces which use the underlying 
//                                     // surface in the Brep
//                                     // note: a composite face must have at least
//                                     //       two faces to exist.
// protected:
//   virtual ~SmCFace() ;
// 
// public:
//   SmCFace(SmFace *pFace1, SmFace *pFace2) ;
// 
//   SmStatus  AddFace(SmFace *pFaceToAdd) ;
// 
//  SmBoolean ContainsFace(const SmFace *pFace) const         
//  { 
//    ULONG idx ; 
//    return(m_pFaces->FindElement((SmFace *)pFace,idx)) ; 
//  }
// 
//  // simple data access
//  // note: have to replace virtual void     GetOuterLoopVertices (SmTArray<SmVertex*>    & rVertices)   const ;
//  // with a GetVertices(rvertices, NULL, bOuteLoopOnly) ; calls
//  virtual SmBrep    * GetBrep              ()                           const ;        
//  ULONG               GetNumFaces          ()                           const ;        
//  void                GetFaces             (SmTArray<SmFace*> & rFaces) const ;        
//  SmFace            * GetFace              (ULONG lIndx)                const { return(m_pFaces->GetAt(lIndx)) ; }           
//  virtual void        GetLoops             (SmTArray<SmLoop*>      & rLoops,                               SmBoolean bTgtLoopsOnly=FALSE, const SmTArray<SmLoop*> *pOptTgtLoops=NULL) const ;
//  virtual void        GetEdges             (SmTArray<SmEdge*>      & rEdges,    ULONG * pOptAttribId=NULL, SmBoolean bTgtLoopsOnly=FALSE, const SmTArray<SmLoop*> *pOptTgtLoops=NULL) const ;
//  virtual void        GetVertices          (SmTArray<SmVertex*>    & rVertices, ULONG * pOptAttribId=NULL, SmBoolean bTgtLoopsOnly=FALSE, const SmTArray<SmLoop*> *pOptTgtLoops=NULL) const ;
//  
//  virtual void        GetFaceuses          (SmFaceuse             *& rpFaceuse1, SmFaceuse *& rpFaceuse2) const ;
//  virtual SmFaceuse * GetUpwardFaceuse     () const ;                                    // one for Faces, 1stFace->Faceuse for CFaces
//  virtual void        GetUpwardFaceuses    (SmTArray<SmFaceuse*>   & rFaceuses) const ;  // one for Faces, AnyNumber for CFaces
//  virtual void        GetUpwardLoopuses    (SmTArray<SmLoopuse*>   & rLoopuses,   SmBoolean bTgtLoopsOnly=FALSE, const SmTArray<SmLoop*> *pOptTgtLoops=NULL) const ;
//  virtual void        GetUpwardVertexuses  (SmTArray<SmVertexuse*> & rVertexuses, SmBoolean bTgtLoopsOnly=FALSE, const SmTArray<SmLoop*> *pOptTgtLoops=NULL) const ;
//  virtual void        GetUpwardEdgeuses    (SmTArray<SmEdgeuse*>   & rEdgeuses,   SmBoolean bTgtLoopsOnly=FALSE, const SmTArray<SmLoop*> *pOptTgtLoops=NULL) const ;
// 
//   // classify (in or out) a UVPoint against the boundaries of all Faces
//   virtual SmStatus PointClassify
//   (
//     const SmPoint2d       & crUVPointToClassify,      ///< [in] : UVPoint to Classify <br>
//     SmZoneTol3d             sSrcZoneTol3d,            ///< [in] : Obj ZoneTol3d assoc with UVPoint, not this face  <br>
//                                                       ///<      :(if none, use: SmTol::GetZoneTol3d(Brep_Context_Or_NULL) <br>
//     SmBoolean               bDoBoundaryIntersections, ///< [in] : TRUE = return point face->Vertex/Edge intersections when found <br>
//                                                       ///<      : FALSE= return loop-containment classifications. <br>
//     SmBoolean               bUseUVClassification,     ///< [in] : No longer used <br>
//     SmPointClassification & rPointClassification,     ///< [out]: Classification of the point. <br>
//                                                       ///<      : when in : m_ePointClass = SM_PC_FACE <br>
//                                                       ///<      :      out: m_ePointClass = SM_PC_UNKNOWN <br>
//                                                       ///<      :      on : m_ePointClass = SM_PC_EDGE or SM_PC_VERTEX <br>
//     SmBoolean               bTgtLoopsOnly=FALSE,      ///< [in] : TRUE           = only classify against pOptTgtLoops loops   <br>
//                                                       ///<        default:[FALSE]= classify against all Face->Loops           <br>
//     SmTArray<SmLoop*>     * pOptTgtLoops=NULL,        ///< [in] : When bTgtLoopsOnly == TRUE                                
//                                                       ///<        default:[NULL or Size=0] = Only classify against the Face's OuterLoop
//                                                       ///<        Size>0                   = Only classify against the these Loops
//     SmBoolean               bUVSpaceOkay=TRUE         ///< [in] : TRUE = okay to try UVSpace Classification if 3Space classification is dodgey (tolerant cases) <br>
//                                                       ///<      : FALSE= don't use UVSpace because UVTrimCurves are not known to be valid, default:[TRUE] <br>
//   ) const ;
//     
//   SmStatus RemoveFace(SmFace *pFaceToRemove);
// 
//   // shrink surface domain to boundary of UVCurves and shrink curve geometry to boundaries of the edge
//   SmStatus ShrinkGeometry() ;
// 
//   // Sets childFaces->m_pSurface = pSurface,
//   //    calls SmFace::Notify(SM_NO_CHANGE_GEOMETRY, pSurface, SM_NO_GET_BREP(this), m_pSurface) ;
//   //    which tells all Face SmEdgeuses and SmVertexuses to delete all their cached data
//   virtual void SetSurface(SmSurface * pSurface, SmBoolean bDeleteCurrFace=FALSE);
// 
//   void SetUVDomains(const SmExtent2d & crNewUVDomain);
// 
//   // add face graphics to new or open displayList branching on global displayParameters
//   virtual SmDisplayList * Draw(SmGfxArraySet * pOptGfxSet=NULL) const;
// 
//   // add face->UVTrimCurves graphics (2d curves drawn on z=0 plane) to new or open displayList
//   virtual SmDisplayList * DrawUVCurves
//   (
//     SmBoolean       bDrawNearBy=TRUE,        ///< [in] : TRUE=Draw UVCurves on nearby Plane, FALSE=Draw on Z=0 plane, default:[TRUE]
//     SmGfxArraySet * pOptGfxSet=NULL
//   ) const;
// 
//   // convenience draw functions:
// 
//   // draw face with mode and optional xHatch counts
//   virtual SmDisplayList * Draw
//   (
//     SmDrawModeType lDrawingMode,        ///< [in] : Face Draw option, oneof                                                          <br>
//                                         ///<        SM_DM_WIREFRAME         SM_DM_OPENGL_CROSSHATCH                                  <br>
//                                         ///<        SM_DM_CROSSHATCH        SM_DM_NORMALS (WIREFRAME + NORMALS)                      <br>
//                                         ///<        SM_DM_NLIB_FACETS       SM_DM_HIDDENLINE                                         <br>
//                                         ///<        SM_DM_OPENGL_FACETS     SM_DM_NMTLIB_FACETS                                      <br>
//                                         ///<        SM_DM_OPENGL_WIREFRAME  SM_DM_CROSSHATCH_KNOTS                                   <br>
//     ULONG lUHatch = 8,                  ///< [in] : evenly spaced U IsoParameter line count drawn on Face->Surface                   <br>
//     ULONG lVHatch = 8,                  ///< [in] : evenly spaced V IsoParameter line count drawn on Face->Surface                   <br>
//     SmGfxArraySet * pOptGfxSet = NULL   ///< [in,out] : used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. <br>
//                                         ///<        NULL to ignore. default:[NULL]                                                   <br>
//   ) const ;  
// 
//   // draw face with crossHatch lines
//   virtual SmDisplayList * DrawUV
//   (
//     ULONG              lUHatch = 8,                   ///< [in] : number of U dir crosshatch lines                                                <br>
//     ULONG              lVHatch = 8,                   ///< [in] : number of V dir crosshatch lines                                                <br>
//     SmBoolean          bVaryCrossHatchColor = FALSE,  ///< [in] : TRUE = Draw U Lines in ObjectColor, Draw V lines in m_VaryCrossHatchColor       <br>
//                                                       ///<        FALSE= Draw both U and V Lines in ObjectColor                                   <br>
//     const SmExtent2d * pOptUVDomain = NULL,           ///< [in] : UVDomain to crossHatch                                                          <br>
//                                                       ///<        NULL=NaturalUVDomain                                                            <br>
//     SmBoolean          bAddToUIPickList = FALSE,      ///< NotUsed: [in] : TRUE = add to display list so this can be picked, FALSE=don't. default:[FALSE]  <br>
//     SmGfxArraySet    * pOptGfxSet = NULL              ///< [in,out]: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. <br>
//                                                       ///<        NULL to ignore. default:[NULL]                                                  <br>
//   ) const;      
//     
//   // get memory used by CFace and its attributes - not any of its child faces
//   virtual ULONG GetMemoryUsed
//   (
//     ULONG    & rlMemoryAllocated,      ///< [out]: bigger size of all allocated memory in bytes   <br>
//     SmMarkType eMarkType=SM_MT_NOMARK  ///< [in] : uses without increment eMarkType value     <br>
//   ) const ;
//     
//   virtual SmBoolean AssertValid
//   (
//     SmAssertArray    * pAList=NULL,           ///< [in,out]: Accumulating list of failed Asserts, NULL to ignore                                           <br>
//     SmAssertTestLevel  eTestLevel=SM_LEVEL_0, ///< [in] : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,                                       <br>
//                                               ///<        SM_LEVEL_GIVEN = run tests in order requested in pTestRequests                                   <br>
//     SmAssertWalking    eWalkTree=SM_WALK,     ///< [in] : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] <br>
//     SmTArray<ULONG>  * pTestRequests=NULL     ///< [in] : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]                 <br>
//   )  const ;
// 
//   // obsolete
//   // virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;
//     
//   // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
//   SM_COMMON(SmCFace,SmFace,SmCFace_TYPE);
//   void DumpTopology(ULONG lWalkDepth=0) const ;  // lWalkDepth[0] = no walk, [99] = walk to bottom
// 
// } ; // end class SmCFace

#endif // !__SMCFACE_H__

