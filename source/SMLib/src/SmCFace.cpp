// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmCFace.cpp 
* PURPOSE: Source file for SmCFace class methods.
**********************************************************************/

#include "StdAfx.h"
#include <SmCFace.h>

// Remove Composites
//  
//  /*******************************************************************//**
//  PURPOSE: Add another face to the composite face.
//  
//  NOTES: 
//  ***********************************************************************/
//  SmStatus SmCFace::AddFace
//    (SmFace *pFaceToAdd)    // in : target surface being declared a sibling to this composite face
//  {
//    // check state
//    if(pFaceToAdd->GetSurface() != m_pSurface) SER(SM_ERR);
//  
//    // return an error for faces already in the list - make no changes
//    for (ULONG i=0; i<m_pFaces->GetSize(); i++) 
//      { 
//        if (pFaceToAdd == (*m_pFaces)[i]) 
//          {
//            SER(SM_ERR);
//          }
//      } // end iter every current face
//  
//    // place the target face on the face list
//    m_pFaces->Add(pFaceToAdd);
//  
//    // increase the domain and pick the biggest tolerance
//    m_vUVDomain.Union(pFaceToAdd->m_vUVDomain, m_vUVDomain);
//  #ifdef SM_USE_NEWTOL
//    SM_NEWTOL_LINE // when pFaceToAdd uses an optional local ZoneTol3d default value - save that as the composite surface value
//    SM_NEWTOL_LINE if(pFaceToAdd->GetLocalZoneTol3d() != SM_UNDEF_DOUBLE)
//    SM_NEWTOL_LINE   {
//    SM_NEWTOL_LINE     m_sLocalZoneTol3d =   pFaceToAdd->GetLocalZoneTol3d() ;
//    SM_NEWTOL_LINE   }
//  #else // SM_USE_OLDTOL
//    SM_OLDTOL_LINE m_sZoneTol3d = smos_Max(pFaceToAdd->m_sZoneTol3d, m_sZoneTol3d) ; 
//  #endif // SM_USE_OLDTOL
//    return SM_SUCCESS;
//  
//  } // end SmCFace::AddFace
//  
//  /*******************************************************************//**
//  PURPOSE: Constructor for the composite face.  It creates a composite
//     from two regular faces.  Please note that a composite must have a
//     minimum of two faces to exist.  
//  
//  NOTES: 
//  ***********************************************************************/
//  SmCFace::SmCFace
//    (SmFace *pFace1,    // in : 1st sibling face of composite face
//     SmFace *pFace2)    // in : 2nd sibling face of composite face
//  {
//      SM_ASSERT(pFace1 != NULL && pFace2 != NULL);
//      SM_ASSERT(pFace1->GetSurface() == pFace2->GetSurface());
//  
//      SmBrep * pBrep = pFace1->GetBrep();
//      m_pFaces = new (*pBrep->GetContext()) SmTArray<SmFace*> (*pBrep->GetContext());
//      m_pFaces->Add(pFace1);
//      m_pFaces->Add(pFace2);
//      m_pSurface = pFace1->GetSurface();
//      m_pSurface->SetOwner(this);  // Composite now owns the surface.
//      pFace1->m_vUVDomain.Union(pFace2->m_vUVDomain,m_vUVDomain);
//  #ifdef SM_USE_NEWTOL
//      SM_NEWTOL_LINE m_sLocalZoneTol3d =  pFace1->GetLocalZoneTol3d() == SM_UNDEF_DOUBLE ? pFace2->GetLocalZoneTol3d()
//      SM_NEWTOL_LINE                       : pFace2->GetLocalZoneTol3d() == SM_UNDEF_DOUBLE ? pFace1->GetLocalZoneTol3d()
//      SM_NEWTOL_LINE                       : smos_Max(pFace1->GetLocalZoneTol3d(), pFace2->GetLocalZoneTol3d()) ;
//  #else // SM_USE_OLDTOL
//      SM_OLDTOL_LINE m_sZoneTol3d = smos_Max(pFace1->m_sZoneTol3d, pFace2->m_sZoneTol3d) ; 
//  #endif // SM_USE_OLDTOL
//      m_pFU        = NULL ;
//      
//      SmObject::Notify(SM_NO_CONSTRUCTION, this, NULL, NULL) ;
//  
//  } // end SmCFace::SmCFace constructor
//  
//  /*******************************************************************//**
//  PURPOSE: Destructor for the composite face.
//  
//  NOTES:
//    1. SmCFace objects are deleted through a SmBrep::DeleteCFace call.
//       It is the responsibility of the calling function to clear the
//       SmCFace->m_pSurface->m_pOwner pointer prior to making that call. 
//  ***********************************************************************/
//  SmCFace::~SmCFace() 
//  { 
//     Notify(SM_NO_DESTRUCTION, this, NULL, NULL) ;
//  
//     if (m_pFaces) { delete m_pFaces; m_pFaces = NULL ; }
//  
//  } // end SmCFace::~SmCFace destructor
//  
//  /*******************************************************************//**
//  PURPOSE: Get the brep which owns this cface.
//  
//  NOTES: 
//  ***********************************************************************/
//  SmBrep* SmCFace::GetBrep() const
//  {
//    if (m_pFaces->GetSize() == 0) 
//      {
//        SE(SM_ERR);
//        return NULL;
//      }
//    SmFace *pF = m_pFaces->GetLast();
//    return pF->GetBrep();
//  
//  } // end SmCFace::GetBrep
//  
//  /*******************************************************************//**
//  PURPOSE: Get the number of faces in this composite face.
//  
//  NOTES: 
//  ***********************************************************************/
//  ULONG SmCFace::GetNumFaces() const
//  {
//    return m_pFaces->GetSize();
//  
//  } // end SmCFace::GetNumFaces
//  
//  /*******************************************************************//**
//  PURPOSE: Get the actual faces in this composite face.
//  
//  NOTES: 
//  ***********************************************************************/
//  void SmCFace::GetFaces
//    (SmTArray<SmFace*> & rFaces) 
//   const
//  {
//    rFaces.ReSet();
//    rFaces.Append(*m_pFaces);
//  
//  } // end SmCFace::GetFaces
//  
//  /*******************************************************************//**
//  PURPOSE: Get the loops of a face.
//  
//  NOTES: If there is an outer loop (and there
//      usually is) it will be first in the list.
//  ***********************************************************************/
//  void SmCFace::GetLoops
//  (SmTArray<SmLoop*>       & rLoops,        // [out]: Face or CFace Loops
//   SmBoolean                 bTgtLoopsOnly, // in : TRUE           = Only classify against Loops pTgtLoops
//                                            //      default:[FALSE]= classify against all Face->Loops
//   const SmTArray<SmLoop*> * pOptTgtLoops   // in : When bTgtLoopsOnly == TRUE                                
//                                            //      default:[NULL or Size=0] = Only classify against the Face's OuterLoop
//                                            //      Size>0                   = Only classify against the these Loops
//  ) const
//  {
//    // init output
//    rLoops.ReSet() ;
//  
//    // locals
//    ULONG ii ;
//    SM_PTR_ARRAY(sThisLoops, SmLoop, 8) ; 
//  
//    // for every face
//    for(ii=0;ii<m_pFaces->GetSize();ii++)
//      {
//        SmFace * pFace = m_pFaces->GetAt(ii) ;
//        if(pFace)
//          {
//            pFace->GetLoops(sThisLoops, bTgtLoopsOnly, pOptTgtLoops) ;
//            rLoops.Append(sThisLoops) ;
//          }
//      } // end iter every face
//  
//  } // end SmCFace::GetLoops
//  
//  /*******************************************************************//**
//  PURPOSE: Get all unique edges used by all of this face's
//              children faces.
//  
//  NOTES: 
//  ***********************************************************************/
//  void SmCFace::GetEdges
//   (SmTArray<SmEdge*> & rEdges,          // out: array of all face edges
//    ULONG             * pOptAttributeId, // in : only include objects containing an attribute with this id
//                                         //      default:[NULL] to ignore
//   SmBoolean                 bTgtLoopsOnly,   // in : TRUE           = Only classify against Loops pTgtLoops
//                                         //    : default:[FALSE]= classify against all Face->Loops
//   const SmTArray<SmLoop*> * pOptTgtLoops     // in : When bTgtLoopsOnly == TRUE                                
//                                              //    : default:[NULL or Size=0] = Only classify against the Face's OuterLoop
//                                              //    : Size>0                   = Only classify against the these Loops
//  ) const
//  {
//    // init output
//    rEdges.ReSet();
//  
//    // locals
//    ULONG ii, jj ; 
//    SM_PTR_ARRAY(sEdges, SmEdge, 64) ; 
//  
//    // for every childFace
//    for(ii=0; ii<m_pFaces->GetSize(); ii++) 
//      {
//        SmFace *pFace = (*m_pFaces)[ii];
//  
//        if(pFace) { pFace->GetEdges(sEdges, pOptAttributeId, bTgtLoopsOnly, pOptTgtLoops) ; }
//  
//        // for every childFace->edge
//        for(jj=0; jj<sEdges.GetSize(); jj++) 
//          {
//            // add unique edges
//            SmEdge * pE = sEdges[jj];
//            rEdges.AddUnique(pE);
//  
//          } // end iter every childFace->edge
//      } // end iter every childFace
//  
//  } // end SmCFace::GetEdges
//  
//  /*******************************************************************//**
//  PURPOSE: Get all unique vertices used by all of this face's
//              children faces.
//  
//  NOTES: 
//  ***********************************************************************/
//  void SmCFace::GetVertices
//  (SmTArray<SmVertex*>     & rVertices,       // out: array of face vertices
//   ULONG                   * pOptAttributeId, // in : only include objects containing an attribute with this id
//                                              //    : default:[NULL] to ignore
//   SmBoolean                 bTgtLoopsOnly,   // in : TRUE           = Only classify against Loops pTgtLoops
//                                              //    : default:[FALSE]= classify against all Face->Loops
//   const SmTArray<SmLoop*> * pOptTgtLoops     // in : When bTgtLoopsOnly == TRUE                                
//                                              //    : default:[NULL or Size=0] = Only classify against the Face's OuterLoop
//                                              //    : Size>0                   = Only classify against the these Loops
//  ) const
//  {
//    rVertices.ReSet();
//  
//    SmVertex* aData2[64];
//    SmTArray<SmVertex*> sVertices(64,aData2);
//  
//    // for every child face
//    for (ULONG i=0; i<m_pFaces->GetSize(); i++) 
//      {
//        SmFace *pFace = (*m_pFaces)[i];
//  
//        // for every childFace->Vertex
//        pFace->GetVertices(sVertices, pOptAttributeId, bTgtLoopsOnly, pOptTgtLoops);
//        for (ULONG j=0; j<sVertices.GetSize(); j++) 
//          {
//            // add unique vertices
//            SmVertex *pV = sVertices[j];
//            rVertices.AddUnique(pV);
//  
//          } // end iter every childFace->Vertex
//      } // end iter every childFace
//  
//  } // end SmCFace::GetVertices
//  
//  /*******************************************************************//**
//  PURPOSE: Get the 1st face's faceuse pair.
//  
//  NOTES: 1st face for CFaces
//  ***********************************************************************/
//  void SmCFace::GetFaceuses
//   (SmFaceuse *& rpFaceuse1, 
//    SmFaceuse *& rpFaceuse2) 
//   const
//  {
//    if(m_pFaces->GetSize() > 0) { return m_pFaces->GetAt(0)->GetFaceuses(rpFaceuse1, rpFaceuse2) ; }
//    else                        { rpFaceuse1 = NULL ;
//                                  rpFaceuse2 = NULL ;
//                                }
//  
//  } // end SmCFace::GetFaceuses
//  
//  /*******************************************************************//**
//  PURPOSE: Get the faceuse on the side of the positive surface normal.
//  
//  NOTES: 1st face for CFaces
//  ***********************************************************************/
//  SmFaceuse* SmCFace::GetUpwardFaceuse() const
//  {
//    if(m_pFaces->GetSize() > 0) { return m_pFaces->GetAt(0)->GetUpwardFaceuse() ; }
//    else                        { return NULL ; }
//  
//  } // end SmCFace::GetUpwardFaceuse
//  
//  /*******************************************************************//**
//  PURPOSE: Get the faceuse on the side of the positive surface normal.
//  
//  NOTES: one for Faces, AnyNumber for CFaces
//  ***********************************************************************/
//  void SmCFace::GetUpwardFaceuses
//   (SmTArray<SmFaceuse*> & rFaceuses)      // out: one for Faces, AnyNumber for CFaces
//   const
//  {
//    // init output
//    rFaceuses.ReSet() ;
//  
//    // locals
//    ULONG ii ;
//  
//    // for every child Face
//    for(ii=0;ii<m_pFaces->GetSize();ii++)
//      {
//        SmFace    * pFace    = m_pFaces->GetAt(ii) ;
//        SmFaceuse * pFaceuse = pFace ? pFace->GetUpwardFaceuse() : NULL ; 
//        if(pFaceuse)   
//          { rFaceuses.Add(pFaceuse) ; }
//      } // end iter every Face
//  
//  } // end SmCFace::GetUpwardFaceuses
//  
//  /*******************************************************************//**
//  PURPOSE: Get the loopuses on the upward faceuse of this face.
//  
//  NOTES: 
//  ***********************************************************************/
//  void SmCFace::GetUpwardLoopuses
//   (SmTArray<SmLoopuse*> & rLoopuses,        // out: Face or CFace UpwardLoopuses
//   SmBoolean                 bTgtLoopsOnly,   // in : TRUE           = Only classify against Loops pTgtLoops
//                                              //    : default:[FALSE]= classify against all Face->Loops
//   const SmTArray<SmLoop*> * pOptTgtLoops     // in : When bTgtLoopsOnly == TRUE                                
//                                              //    : default:[NULL or Size=0] = Only classify against the Face's OuterLoop
//                                              //    : Size>0                   = Only classify against the these Loops
//  ) const
//  { 
//    // init output
//    rLoopuses.ReSet() ;
//  
//    // locals
//    ULONG ii ;
//    SM_PTR_ARRAY(sThisLoopuses, SmLoopuse, 16) ; 
//  
//    // for every Face
//    for(ii=0;ii<m_pFaces->GetSize();ii++)
//      {
//        SmFace    * pFace    = m_pFaces->GetAt(ii) ;
//        SmFaceuse * pFaceuse = pFace ? pFace->GetUpwardFaceuse() : NULL ; 
//  
//        if(pFaceuse)   
//          { 
//            pFaceuse->GetLoopuses(sThisLoopuses, bTgtLoopsOnly, pOptTgtLoops) ;
//            rLoopuses.Append(sThisLoopuses) ; 
//          }
//  
//      } // end iter every Face
//  
//  } // end SmCFace::GetUpwardLoopuses
//  
//  /*******************************************************************//**
//  PURPOSE: Get edgeuses of the all the child face upward faceuses.
//  
//  NOTES: 
//  ***********************************************************************/
//  void SmCFace::GetUpwardEdgeuses
//  (
//    SmTArray<SmEdgeuse*>   & rEdgeuses,       // out: array of all face edgeuses
//   SmBoolean                 bTgtLoopsOnly,   // in : TRUE           = Only classify against Loops pTgtLoops
//                                              //    : default:[FALSE]= classify against all Face->Loops
//   const SmTArray<SmLoop*> * pOptTgtLoops     // in : When bTgtLoopsOnly == TRUE                                
//                                              //    : default:[NULL or Size=0] = Only classify against the Face's OuterLoop
//                                              //    : Size>0                   = Only classify against the these Loops
//  ) const
//  {
//    // init output
//    rEdgeuses.ReSet();
//    
//    // locals
//    ULONG ii, jj ;
//    SM_PTR_ARRAY(sLoopuses, SmLoopuse, 10) ;
//    SM_PTR_ARRAY(sEdgeuses, SmEdgeuse, 20) ;
//  
//    // for every child face
//    for(ii=0; ii<m_pFaces->GetSize(); ii++) 
//      {
//        // get upward faceuse
//        SmFace    * pFace = (*m_pFaces)[ii];
//        SmFaceuse * pFU   = pFace ? pFace->GetUpwardFaceuse() : NULL ; 
//  
//        pFU->GetLoopuses(sLoopuses, bTgtLoopsOnly, pOptTgtLoops);
//  
//        // for every upwardFaceuse->Loopuse
//        for(jj=0; jj<sLoopuses.GetSize(); jj++) 
//          {
//            SmLoopuse *pLU = (SmLoopuse*)sLoopuses[jj];
//  
//            // accumulate edgeuses
//            if(pLU) { pLU->GetEdgeuses(sEdgeuses); 
//                      rEdgeuses.Append(sEdgeuses);
//                    }
//                     
//          } // end iter every UpwardFaceuse->Loopuse
//       } // end iter every child face
//  
//  } // end SmCFace::GetUpwardEdgeuses
//  
//  /*******************************************************************//**
//  PURPOSE: Get Vertexuses of the all the child face upward faceuses.
//  
//  NOTES: 
//  ***********************************************************************/
//  void SmCFace::GetUpwardVertexuses
//  (SmTArray<SmVertexuse*>  & rVertexuses,     // out: array of upward face vertexuses
//   SmBoolean                 bTgtLoopsOnly,   // in : TRUE           = Only classify against Loops pTgtLoops
//                                              //    : default:[FALSE]= classify against all Face->Loops
//   const SmTArray<SmLoop*> * pOptTgtLoops     // in : When bTgtLoopsOnly == TRUE                                
//                                              //    : default:[NULL or Size=0] = Only classify against the Face's OuterLoop
//                                              //    : Size>0                   = Only classify against the these Loops
//  ) const
//  {
//    // init output
//    rVertexuses.ReSet();
//    
//    // locals
//    ULONG ii, jj ;
//    SM_PTR_ARRAY(sLoopuses, SmLoopuse, 10) ;
//    SM_PTR_ARRAY(sVertexuses, SmVertexuse, 20) ;
//  
//    // for every child face
//    for(ii=0; ii<m_pFaces->GetSize(); ii++) 
//      {
//        // get upward faceuse
//        SmFace    * pFace    = (*m_pFaces)[ii];
//        SmFaceuse * pFaceuse = pFace ? pFace->GetUpwardFaceuse() : NULL ; 
//  
//        if(pFaceuse) { pFaceuse->GetLoopuses(sLoopuses, bTgtLoopsOnly, pOptTgtLoops); }
//  
//        // for every upwardFaceuse->Loopuse
//        for(jj=0; jj<sLoopuses.GetSize(); jj++) 
//          {
//            SmLoopuse *pLU = (SmLoopuse*)sLoopuses[jj];
//  
//            // accumulate vertexuses
//            if(pLU) { pLU->GetVertexuses(sVertexuses); 
//                      rVertexuses.Append(sVertexuses);
//                    }
//          }
//      } // end iter every child face
//  
//  } // end SmCFace::GetUpwardVertexuses
//  
//  /*******************************************************************//**
//  PURPOSE: Classify the point which is on the face's surface.
//  
//  NOTES:  If you already know that the point does not lie on
//      the boundary of the face, set bDoBoundaryIntersections to FALSE to
//      optimize the computation.  In that case only rayfiring will be done.
//  ***********************************************************************/
//  SmStatus SmCFace::PointClassify
//  (
//    const SmPoint2d       & crUVPointToClassify,      // [in] : UVPoint to Classify <br>
//    SmZoneTol3d             sSrcZoneTol3d,            // [in] : Obj ZoneTol3d assoc with UVPoint, not this face  <br>
//                                                      //       (if none, use: SmTol::GetZoneTol3d(Brep_Context_Or_NULL) <br>
//    SmBoolean               bDoBoundaryIntersections, // [in] : TRUE = return point face->Vertex/Edge intersections when found <br>
//                                                      //        FALSE= return loop-containment classifications. <br>
//    SmBoolean               bUseUVClassification,     // [in] : no longer used <br>
//    SmPointClassification & rPointClassification,     // [out]: Classification of the point. <br>
//                                                      //        when in : m_ePointClass = SM_PC_FACE <br>
//                                                      //             out: m_ePointClass = SM_PC_UNKNOWN <br>
//                                                      //             on : m_ePointClass = SM_PC_EDGE or SM_PC_VERTEX <br>
//    SmBoolean               bTgtLoopsOnly,            // in : TRUE           = only classify against pOptTgtLoops loops   
//                                                      //      default:[FALSE]= classify against all Face->Loops         
//    SmTArray<SmLoop*>     * pOptTgtLoops,             // in : When bTgtLoopsOnly == TRUE                                
//                                                      //      default:[NULL or Size=0] = Only classify against the Face's OuterLoop
//                                                      //      Size>0                   = Only classify against the these Loops
//    SmBoolean               bUVSpaceOkay              // in : default:[TRUE] = okay to try UVSpace Classification if 3Space classification is dodgey (tolerant cases)
//                                                      //      FALSE= don't use UVSpace because UVTrimCurves are not known to be valid
//  ) const
//  {
//    // init output
//    rPointClassification.SetClassObject( SM_PC_UNKNOWN, NULL );
//  
//    // locals
//    ULONG ii;
//  
//    // no work - no faces
//    if ( m_pFaces == NULL )
//      { return SM_SUCCESS; }
//  
//    // for every composite face
//    for(ii=0;ii<m_pFaces->GetSize();ii++)
//      {
//        // 
//        (*m_pFaces)[ii]->PointClassify(crUVPointToClassify,      // in : UVPoint to Classify  
//                                       sSrcZoneTol3d,            // in : Obj ZoneTol3d assoc with UVPoint, not this face  
//                                                                 //     (if none, use: SmTol::GetZoneTol3d(Brep_Context_Or_NULL) 
//                                       bDoBoundaryIntersections, // in : TRUE = return point face->Vertex/Edge intersections when found 
//                                                                 //      FALSE= return loop-containment classifications. 
//                                       bUseUVClassification,     // in : no longer used 
//                                       rPointClassification,     // out: Classification of the point. 
//                                                                 //      when in : m_ePointClass = SM_PC_FACE 
//                                                                 //           out: m_ePointClass = SM_PC_UNKNOWN 
//                                                                 //           on : m_ePointClass = SM_PC_EDGE or SM_PC_VERTEX 
//                                       bTgtLoopsOnly,            // in : TRUE           = only classify against pOptTgtLoops loops   
//                                                                 //      default:[FALSE]= classify against all Face->Loops         
//                                       pOptTgtLoops,             // in : When bTgtLoopsOnly == TRUE                                
//                                                                 //      default:[NULL or Size=0] = Only classify against the Face's OuterLoop
//                                                                 //      Size>0                   = Only classify against the these Loops
//                                       bUVSpaceOkay) ;           // in : default:[TRUE] = okay to try UVSpace Classification if 3Space classification is dodgey (tolerant cases)
//                                                                 //      FALSE= don't use UVSpace because UVTrimCurves are not known to be valid
//  
//        if ( rPointClassification.GetPointClass() != SM_PC_UNKNOWN )
//          {
//            return SM_SUCCESS;
//          }
//      } // end loop on all Faces
//  
//    // Didn't classify in or on any face.
//  
//    return SM_SUCCESS;
//  } // end SmCFace::PointClassify
//  
//  /*******************************************************************//**
//  PURPOSE: Removes a face from the composite and 
//     if the CFace member count drops to one, also makes the last member another 
//     stand alone face and destroys this CFace after removing it from its Brep.  
//       
//  NOTES: A CFace only exists if there are a minimum of two faces in it.
//  
//     1. This does not copy the surface: the removed face and this SmCFace
//        still point to the same surface.  (This method is often called before
//        deleting a Face.)  Therefore, if you are not deleting pFaceToRemove,
//        you should copy the surface and replace pFaceToRemove's surface with
//        the copy.
//  
//     2. returns SM_SUCCESS when pFaceToRemove was removed from this CFace
//        returns SM_ERR when pFaceToRemove is not initially a member of CFace
//  ***********************************************************************/
//  SmStatus SmCFace::RemoveFace
//    (SmFace *pFaceToRemove)
//  {
//    // for every Composite Face member
//    for (ULONG i=0; i<m_pFaces->GetSize(); i++) 
//      {
//        // When current CFace member is the target
//        if ((*m_pFaces)[i] == pFaceToRemove) 
//          {
//            // Remove the Face from the CFace member list
//            m_pFaces->RemoveAt(i,1);
//  
//            // when just one member is left in the CFace
//            if (m_pFaces->GetSize() == 1) 
//              {
//                // Make the Last Face a stand alone face
//                SmFace *pLastFace = m_pFaces->GetLast();
//                m_pSurface->SetOwner(pLastFace);
//  
//                // delete the CompositeFace - only works if CFace->Surface->Owner != CFace
//                pLastFace->GetBrep()->DeleteCFace(this);
//  
//              } // end just 1 member face remaining branch
//            else // multiple members left in CFACE
//              {
//                // Here we update the internal values
//                SmFace *pFirstF      = (*m_pFaces)[0];
//                m_vUVDomain          = pFirstF->m_vUVDomain;
//  #ifdef SM_USE_NEWTOL
//                SM_NEWTOL_LINE m_sLocalZoneTol3d = pFirstF->GetLocalZoneTol3d() ;
//  #else // SM_USE_OLDTOL
//                SM_OLDTOL_LINE m_sZoneTol3d         = pFirstF->GetTolerance() ;
//  #endif // SM_USE_OLDTOL
//  
//                // for every remaining CFace->Member - update CFace domain and tol as needed
//                for (ULONG j=1; j<m_pFaces->GetSize(); j++) 
//                  {
//                    SmFace *pF = (*m_pFaces)[j];
//                    m_vUVDomain.Union(pF->m_vUVDomain,m_vUVDomain);
//  #ifdef SM_USE_NEWTOL
//                    SM_NEWTOL_LINE m_sLocalZoneTol3d =  pF->GetLocalZoneTol3d() == SM_UNDEF_DOUBLE ? GetLocalZoneTol3d()
//                    SM_NEWTOL_LINE                       : GetLocalZoneTol3d()     == SM_UNDEF_DOUBLE ? pF->GetLocalZoneTol3d()
//                    SM_NEWTOL_LINE                       : smos_Max(pF->GetLocalZoneTol3d(), GetLocalZoneTol3d()) ;
//  #else // SM_USE_OLDTOL
//                    SM_OLDTOL_LINE m_sZoneTol3d = smos_Max(pF->GetTolerance(),GetTolerance()) ; 
//  #endif // SM_USE_OLDTOL
//                  } // end iter every remaining CFace->Member
//              } // end multiple remaining member faces branch
//  
//            // all done
//            return SM_SUCCESS;
//  
//          } // end current CFace member is the target check
//      } // end iter every CFace member
//  
//    // only arrive here when pFaceToRemove is not a member of this CFace - signal err and return
//    SE(SM_ERR);
//    return SM_ERR;
//  
//  } // end SmCFace::RemoveFace
//  
//  /*******************************************************************//**
//  PURPOSE: Shrink the domain of the CFace->Surface to the union of
//    all child face UVTrimBoundaries.
//  
//  NOTES:
//  ***********************************************************************/
//  SmStatus SmCFace::ShrinkGeometry()
//  {
//    // locals
//    ULONG ii, jj ;
//    SmTArray<SmFace*> sChildFaces;
//  
//    // check state
//    SM_ASSERT(GetSurface() != NULL) ;
//  
//  #ifdef SM_DEBUG_CODE
//  SmBoolean bDebugMe  = FALSE;
//    if (bDebugMe)
//      { 
//        GetFaces(sChildFaces);
//        SmBrep *pBrep = GetBrep() ;
//        
//        // draw brep, composite face, and each face indiviually
//        smgfx_Erase();
//        smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//        smgfx_SetLook(1,2, 0,1,1) ; this->Draw(); sm_GraphicsLoop();
//        smgfx_SetLook(2,3, 1,0,1) ; { for (ii=0; ii<sChildFaces.GetSize(); ii++)
//                                        { SmFace *pChildFace = sChildFaces[ii];
//                                          pChildFace->Draw(); sm_GraphicsLoop() ;
//                                        }  
//                                    }
//        sm_GraphicsLoop();
//      }
//  #endif // SM_DEBUG_CODE
//  
//    // Shrink the UVDomain of all faces to their UVTrimCurves
//    this->GetFaces(sChildFaces);
//  
//    // for every child face - shrink the ChildFace Domain to the union of its UVTrimCurve bounding boxes
//    //  don't call SmFace::ShrinkGeometry - causes an infinite loop.
//    for (ii=0; ii<sChildFaces.GetSize(); ii++)
//      {
//        SmFace *pChildFace = sChildFaces[ii];
//        SmExtent2d sChildFaceDomain ;
//  
//        // compute face domain from boundary curves
//        if (pChildFace->CalculateUVDomainFromUVTrimCurves(sChildFaceDomain) == SM_SUCCESS)
//          {
//            // get surface's current natural UVDomain
//            SmExtent2d sSurfDomain = pChildFace->GetSurface()->GetNaturalUVDomain();
//  
//            // If the face domain is very close to surface domain, choose the surface domain
//            SmPoint2d sMin = sChildFaceDomain.GetMin();
//            SmPoint2d sMax = sChildFaceDomain.GetMax();
//  
//            if (smos_Fabs(sMin.x-sSurfDomain.GetMin().x) < SM_EFF_ZERO)
//              { sMin.x = sSurfDomain.GetMin().x;
//              }
//            if (smos_Fabs(sMin.y-sSurfDomain.GetMin().y) < SM_EFF_ZERO)
//              { sMin.y = sSurfDomain.GetMin().y;
//              }
//            if (smos_Fabs(sMax.x-sSurfDomain.GetMax().x) < SM_EFF_ZERO)
//              { sMax.x = sSurfDomain.GetMax().x;
//              }
//            if (smos_Fabs(sMax.y-sSurfDomain.GetMax().y) < SM_EFF_ZERO)
//              { sMax.y = sSurfDomain.GetMax().y;
//              }
//  
//            // set the (revised) ChildFace domain
//            // note: there is no test here if the face domain is greater than
//            // the surface domain
//            SmExtent2d sNewChildDomain(sMin,sMax);
//  
//  #ifdef SM_DEBUG_CODE
//            if (bDebugMe)
//              {
//                 SmBrep *pBrep = GetBrep() ;
//        
//                // draw brep, composite face, and each face indiviually
//                smgfx_Erase();
//                smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//                smgfx_SetLook(1,2, 0,1,1) ; this->Draw(); sm_GraphicsLoop();
//                smgfx_SetLook(2,3, 1,0,1) ; pChildFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop() ;
//                smgfx_SetLook(3,4, 0,0,0) ; 
//                SmExtent2d range = pChildFace->GetUVDomain();
//                pChildFace->GetSurface()->DrawUV(8,8,FALSE,&range) ; sm_GraphicsLoop() ;
//                smgfx_SetLook(4,5, 0,1,0) ; pChildFace->GetSurface()->DrawUV(8,8,FALSE,&sNewChildDomain) ; sm_GraphicsLoop() ;
//                sm_GraphicsLoop();
//              }
//  #endif // SM_DEBUG_CODE
//  
//            pChildFace->SetUVDomain(sNewChildDomain);
//  
//          } // end CalculateUVDomainFromUVTrimCurves success check
//      } // end iter every face
//  
//    // Trim the surface and reset the surface domain to the CompositeFace's trim boundary.
//    SmSurface *pSurface = GetSurface() ;
//  
//    // get union of all child face domains
//    SmExtent2d sFaceDomain = sChildFaces[0]->GetUVDomain();
//    for (ii=1; ii<sChildFaces.GetSize(); ii++)
//      {
//        sFaceDomain.Union(sChildFaces[ii]->GetUVDomain(), sFaceDomain);
//      }
//  
//    // Make sure face domain is still within the domain of the surface
//    // by intersecting the two.
//    SmExtent2d sSurfDomain = pSurface->GetNaturalUVDomain();
//    sSurfDomain.Intersect(sFaceDomain, sFaceDomain);
//  
//    // Do a Hard trim of the surface 
//    pSurface->TrimWithDomain(sFaceDomain);
//  
//    // update the CFace domain
//    SetUVDomain(sFaceDomain) ;
//  
//    // for every composite face sibling
//    for (ii=0; ii<sChildFaces.GetSize(); ii++)
//      {
//        // intersect the face domain with the composite domain
//        SmFace     *pChildFace       = sChildFaces[ii];
//        SmExtent2d  sChildFaceDomain = pChildFace->GetUVDomain();
//        SER(sChildFaceDomain.Intersect(sFaceDomain, sChildFaceDomain));
//        pChildFace->SetUVDomain(sChildFaceDomain);
//  
//        // delete the associated UVTrimCurves
//        pChildFace->RemoveUVTrimCurves();
//  
//      } // end iter every composite face sibling
//  
//    // Now Trim all Edge->Curves to their edge domains
//    SmTArray<SmEdge*> sEdges, cEdges;
//    GetEdges(sEdges);
//  
//    // for every curve
//    for (ii=0; ii<sEdges.GetSize(); ii++)
//      {
//        SmEdge  *pEdge      = sEdges[ii] ;
//        SmCurve *pCurve     = pEdge->GetCurve();
//        SmEdge  *pOwnerEdge = (SmEdge*)pCurve->GetEdge() ;
//  
//        // skip curves without owners
//        if (pOwnerEdge == NULL) continue;
//  
//        // get all edges using this curve
//        cEdges.ReSet();
//        if (pOwnerEdge->IsKindOf(SmCEdge_TYPE)) { SmCEdge *pCEdge = (SmCEdge*)pOwnerEdge;
//                                                  pCEdge->GetEdges(cEdges);
//                                                }
//        else                                    { cEdges.Add(pOwnerEdge);
//                                                }
//  
//        // get union of all edge->intervals
//        SmExtent1d sIvl = cEdges[0]->GetInterval() ;
//        for (jj=1; jj<cEdges.GetSize(); jj++)
//          {
//            sIvl.Union(cEdges[jj]->GetInterval(),sIvl);
//          }
//  
//        // trim the curve to the used interval - some derived curve types don't trim
//        pCurve->Trim(sIvl) ;  // may snap sIvl by tol to existing knots
//  
//        // update compositeEdge intervals
//        if(pOwnerEdge->IsKindOf(SmCEdge_TYPE))
//          {
//            SmCEdge *pCEdge = (SmCEdge*)pOwnerEdge;
//            pCEdge->SetInterval(sIvl);
//          }
//      } // end iter every curve
//  
//    // all done
//    return SM_SUCCESS;
//  
//  } // end SmCFace::ShrinkGeometry
//  
//  /*******************************************************************//**
//  PURPOSE: Set the surface of the CFace and its subordinates.
//  
//  NOTES: calls ChildFace->SetSurface(pSurface)
//               which calls SmFace::Notify(SM_NO_CHANGE_GEOMETRY, pSurface, SM_NO_GET_BREP(this), m_pSurface) ;
//                     which tells all Face SmEdgeuses and SmVertexuses to delete their cached data
//  ***********************************************************************/
//  void SmCFace::SetSurface
//   (SmSurface * pSurface,        // in : New Surface to save
//    SmBoolean   bDeleteCurrFace) // in : TRUE = delete NonNULL current m_pSurface, FALSE=don't
//                                 //      default:[FALSE] original behavior 
//  {
//    if(pSurface != m_pSurface)
//      {
//        // when asked - delete m_pSurface (clears m_pOwner - calls Notify)
//        if(bDeleteCurrFace && m_pSurface)
//          {
//            delete m_pSurface ; m_pSurface = NULL ;
//          }
//  
//        // save Surface
//        m_pSurface = pSurface;
//  
//        // tell all the sibling Faces to save the surface
//        for (ULONG i=0; i<m_pFaces->GetSize(); i++) 
//          {
//            SmFace *pF = (*m_pFaces)[i];
//            pF->SetSurface(pSurface, FALSE);
//          }
//  
//        // Change the surface owner's to this CFace
//        pSurface->SetOwner(this) ;  
//  
//        // update the Face domain
//        SmExtent2d  sNewDomain = pSurface->GetNaturalUVDomain();
//        SetUVDomain(sNewDomain) ;
//  
//      } // end Surface is changing check
//  
//  } // end SmCFace::SetSurface
//  
//  /*******************************************************************//**
//  PURPOSE: Set the domain of the CFace and its subordinates.
//  
//  NOTES: To set only the CFace's domain, use SetUVDomain().
//  ***********************************************************************/
//  void SmCFace::SetUVDomains(const SmExtent2d & crNewUVDomain )
//  {
//    SmFace::SetUVDomain( crNewUVDomain );
//    for (ULONG i=0; i<m_pFaces->GetSize(); i++) 
//      {
//        SmFace *pF = (*m_pFaces)[i];
//        pF->SetUVDomain(crNewUVDomain);
//      }
//  
//  } // end SmCFace::SetUVDomain
//  
//  /*******************************************************************//**
//  PURPOSE: add Face graphics switching on global display parameters
//     to new or open displayList added to the global displayList array.
//  
//  NOTES:
//     m_eMode == 
//  ****************************************************************/
//  SmDisplayList * SmCFace::Draw
//   (SmGfxArraySet * pOptGfxSet)  // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
//                                 //      NULL to ignore. default:[NULL] 
//   const
//  {
//    SmDisplayList *pRtn = NULL ;
//  
//  #ifdef SM_GFX_CODE
//  
//    // start new displayList (unless one is already open)
//    smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);
//  
//    // pass the call along to every face
//    for(ULONG ii=0; ii<m_pFaces->GetSize() ; ii++)
//      {
//        (*m_pFaces)[ii]->Draw(pOptGfxSet) ; 
//  
//      } // end iter every face
//  
//    // end new displayList
//    pRtn = smgfx_Close(pOptGfxSet) ;
//  
//  #endif
//    return(pRtn) ;
//  
//  } // end SmCFace::Draw
//  
//  /*******************************************************************//**
//  PURPOSE: Add 2d graphics drawn in Z=0 plane 
//              for each face->UVTrimCurve to new or 
//         currently open DisplayList added to global displayList array.
//  
//  NOTES: 
//    Two ways to see the UVTrimLines in 3d 
//      1.    call SmCFace::Draw() with Global s_Disp.m_bDrawWireFrame = TRUE
//      2. or call SmCFace::Draw(SM_DM_WIREFRAM) ;
//  ****************************************************************/
//  SmDisplayList * SmCFace::DrawUVCurves
//   (SmBoolean       bDrawNearBy, // in : TRUE=Draw UVCurves on nearby Plane, FALSE=Draw on Z=0 plane, default:[TRUE]
//    SmGfxArraySet * pOptGfxSet)  // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
//                                 //      NULL to ignore. default:[NULL]
//    const
//  {
//    SmDisplayList *pRtn = NULL ;
//  
//  #ifdef SM_GFX_CODE
//  
//    // start new displayList (unless one is already open)
//    smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);
//  
//    // pass the call along to every face
//    for(ULONG ii=0; ii<m_pFaces->GetSize() ; ii++)
//      {
//        (*m_pFaces)[ii]->DrawUVCurves(bDrawNearBy, pOptGfxSet) ; 
//  
//      } // end iter every face
//  
//    // end new displayList
//    pRtn = smgfx_Close(pOptGfxSet) ;
//  
//  #endif
//    return(pRtn) ;
//  
//  } // end SmCFace::DrawUVCurves()
//  
//  /*******************************************************************//**
//  PURPOSE: convenience function:
//          Add Face graphics, with specified mode and hatch line counts,
//          to new or open DisplayList added to global displayList array.
//  
//  USAGE_NOTES --- 
//  
//  METHOD ---
//    1. Get GlobalDisplayParameters
//    2. SetDisplayParameters(eMode)
//    3. Do the Draw
//  ****************************************************************/
//  SmDisplayList * SmCFace::Draw
//    (SmDrawModeType eMode,       // in : Face Draw option, oneof
//                                 //      SM_DM_WIREFRAME         SM_DM_OPENGL_CROSSHATCH
//                                 //      SM_DM_CROSSHATCH        SM_DM_NORMALS (WIREFRAME + NORMALS)
//                                 //      SM_DM_NLIB_FACETS       SM_DM_HIDDENLINE       
//                                 //      SM_DM_OPENGL_FACETS     SM_DM_NMTLIB_FACETS    
//                                 //      SM_DM_OPENGL_WIREFRAME  SM_DM_CROSSHATCH_KNOTS 
//     ULONG          lUHatch,     // in : evenly spaced U IsoParameter line count drawn on Face->Surface
//     ULONG          lVHatch,     // in : evenly spaced V IsoParameter line count drawn on Face->Surface
//    SmGfxArraySet * pOptGfxSet)  // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
//                                 //      NULL to ignore. default:[NULL]
//    const
//  {
//    SmDisplayList *pRtn = NULL ;
//  
//  #ifdef SM_GFX_CODE
//  
//    // start new displayList (unless one is already open)
//    smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);
//  
//    // pass the call along to every face
//    for(ULONG ii=0; ii<m_pFaces->GetSize() ; ii++)
//      {
//        (*m_pFaces)[ii]->Draw(eMode, lUHatch, lVHatch, pOptGfxSet) ; 
//  
//      } // end iter every face
//  
//    // end new displayList
//    pRtn = smgfx_Close(pOptGfxSet) ;
//  
//  #endif
//    return(pRtn) ;
//  
//  } // SmCFace::Draw(Mode)
//  
//  /*******************************************************************//**
//  PURPOSE: Convenience Function:
//     Add Face->isoParameterCurve graphics to new or open
//     displayList added to the global displayList array.
//  
//  USAGE NOTES --
//    1. Set GlobalDrawingMode = SM_DM_CROSSHATCH
//       Set GlobalUVHatchCount
//    2. pass call to Draw()
//    3. Restore globalState
//  ****************************************************************/
//  SmDisplayList * SmCFace::DrawUV
//    (ULONG              lUHatch,               // in : number of U dir crosshatch lines
//     ULONG              lVHatch,               // in : number of V dir crosshatch lines
//     SmBoolean          bVaryCrossHatchColor,  // in : TRUE = Draw U Lines in ObjectColor 
//                                               //             Draw V lines in m_VaryCrossHatchColor  
//                                               //      FALSE= Draw both U and V Lines in ObjectColor
//     const SmExtent2d * pOptUVDomain,          // in : UVDomain to crossHatch
//                                               //      NULL=NaturalUVDomain
//     SmBoolean          bAddToUIPickList,      // NotUsed: in : TRUE = add to display list so this can be picked, FALSE=don't. default:[FALSE]
//    SmGfxArraySet     * pOptGfxSet)            // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
//                                               //      NULL to ignore. default:[NULL]
//   const
//  {
//    SM_REF1(bAddToUIPickList) ;
//    SmDisplayList *pRtn = NULL ;
//  
//  #ifdef SM_GFX_CODE
//  
//    // start new displayList (unless one is already open)
//    smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);
//  
//    // pass the call along to every face
//    for(ULONG ii=0; ii<m_pFaces->GetSize() ; ii++)
//      {
//        (*m_pFaces)[ii]->DrawUV(lUHatch, lVHatch, bVaryCrossHatchColor, pOptUVDomain, FALSE, pOptGfxSet) ; 
//  
//      } // end iter every face
//  
//    // end new displayList
//    pRtn = smgfx_Close(pOptGfxSet) ;
//  
//  #endif // SM_GFX_CODE
//    return(pRtn) ;
//  
//  } // end SmCFace::DrawUV
//  
//  /*******************************************************************//**
//  PURPOSE: Compute the total size of the memory used by the SmCFace.
//  
//  NOTES:  Does not include any of the CFace children Face memory.
//  ***********************************************************************/
//  ULONG SmCFace::GetMemoryUsed       // rtn: smaller size of actually used memory in bytes
//    (ULONG    & rlMemoryAllocated,   // out: bigger size of all allocated memory in bytes
//     SmMarkType eMarkType)           // in : uses without increment eMarkType value
//    const
//  {
//    // in case this method is called directly - get a mark for attribute memory usage
//    SmNewMarkAndLock sMarkLock ;
//    if(eMarkType == SM_MT_NOMARK)
//      {
//        eMarkType = sMarkLock.SetContext((SmContext *)GetContext()) ;
//      }
//  
//    // locals
//    ULONG lThisAllocated ;
//  
//    // this + attribute memory
//    ULONG lUsed        = sizeof(*this) + this->GetAttributeMemoryUsed(lThisAllocated, 
//                                                                      eMarkType) ;  // note: uses without increment eMarkType value
//    rlMemoryAllocated  = sizeof(*this) + lThisAllocated ; 
//  
//    // m_pFaces memory - but not any memory of the contained item Faces
//    if(m_pFaces)
//      {
//        lUsed             += m_pFaces->GetMemoryUsed(lThisAllocated) ;
//        rlMemoryAllocated += lThisAllocated ; 
//      }
//  
//    // all done
//    return(lUsed) ;
//  
//  } // end SmCFace::GetMemoryUsed
//  
//  /*******************************************************************//**
//  PURPOSE: AssertValid() Test reporting static labels
//  ***********************************************************************/
//  SmAssertReportLabel sAssertCFace_list[] =
//  {
//    {SM_AT_POINTER, _T("Context"), _T("pFace shares same context") }
//  } ;
//  
//  /*******************************************************************//**
//  PURPOSE:
//  
//  RETURNS ---  TRUE  = OK
//               FALSE = Problem
//  ***********************************************************************/
//  SmBoolean SmCFace::AssertValid
//   (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL] 
//    SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
//                                      //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
//                                      //      default:[SM_LEVEL_0] 
//    SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK] 
//    SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
//   const
//  {
//    // init rtn value
//    SmBoolean bRtn = TRUE;
//    
//    // call the base class AssertValid
//    bRtn &= (  (eTestLevel != SM_LEVEL_GIVEN)
//             ? SmFace::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) 
//             : TRUE ) ; 
//  
//    // CompositeFace and the objects it attaches to need to share common contexts
//    if(m_pFaces)  
//      { 
//        ULONG ii ;
//        for(ii=0;ii<m_pFaces->GetSize();ii++)
//          {  
//             SmFace *pFace = (*m_pFaces)[ii] ;
//             bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (GetContext() == pFace->GetContext() ), _T("") ) ;
//  
//             // check the member faces
//             bRtn &= pFace->AssertValid(pAList, eTestLevel, eWalkTree) ;
//  
//          } // end every member face
//      } // end face list check
//  
//    // all done
//    // SM_ASSERT(bRtn) ;
//    return(bRtn) ;
//  
//  } // end SmCFace::AssertValid
//  
//  // obsolete
//  // /*******************************************************************//**
//  // PURPOSE: Fix Assert Report failures reported by an AssertValid call
//  // 
//  // RETURNS ---  TRUE  = OK
//  //              FALSE = Problem
//  // ***********************************************************************/
//  // SmBoolean SmCFace::AssertHeal
//  //  (SmAssertReport & rAReport,  // in : a report generated by AssertValid
//  //   SmAssertArray  * pAList)    // in : AssertArray holding rAReport 
//  // {
//  //   SmBoolean bRtn = FALSE ;
//  // 
//  //   // check state - no work
//  //   if(rAReport.m_bOK == TRUE)
//  //     { return( TRUE ) ; }
//  // 
//  //   // check state - not the class that generated this report - pass call to parent class
//  //   if(rAReport.m_lReportingType != GetClassType())
//  //     {
//  //       // pass the call along to the parent - return ( Parent::AssertHeal(rAReport, pAList) ) ;
//  //       return ( SmFace::AssertHeal(rAReport, pAList) ) ;
//  //     }
//  //    
//  //   // branch on the report type
//  //   switch(rAReport.m_lTestIndex)
//  //     {
//  //       case 99 : { // set case number appropriately - run fix code here
//  //                   // if fix works set rAReport.m_bOK = TRUE ; 
//  //                 }
//  //                 break ;
//  // 
//  //       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;  
//  //                rAReport.m_pHealMessage = _T("SmCFace::AssertHeal fix not yet supported") ;  
//  //                 
//  //     }
//  // 
//  //   // all done
//  //   return(bRtn) ;
//  // 
//  // } // end SmCFace::AssertHeal
//  // end obsolete
//  
//  /*******************************************************************//**
//  PURPOSE:
//  NOTES:
//  ***********************************************************************/
//  SmBoolean SmCFace::IsKindOf( SM_TYPE t ) const
//  {
//    return ((SmCFace_TYPE == t) ? TRUE : SmFace::IsKindOf( (t) ));
//  }
//  
//  /*******************************************************************//**
//  PURPOSE:
//  
//  NOTES:
//  ***********************************************************************/
//  void SmCFace::Dump(void) 
//    const
//  {
//    smos_WriteBuffer(_T("SmCFace object\n"));
//    m_pFaces->Dump();
//  
//    ULONG i;
//    for ( i=0; i < m_pFaces->GetSize(); i++ )
//      {
//        m_pFaces->GetAt(i)->Dump( i ); 
//      }
//  
//  } // end SmCFace::Dump
//  
//  /*******************************************************************//**
//  PURPOSE:  Pretty print pointer values for this CompositeFace showing its
//               list of child Faces in the topology graph.
//  
//  NOTES: Only good for debugging because pointer values don't
//                  stay constant from run to run.
//  ***********************************************************************/
//  void SmCFace::DumpTopology
//    (ULONG lWalkDepth) // in : 0 = no walking, 1 = and faces, 2 = and their edges and vertices, ... 99 = walk to bottom
//   const
//  {
//    // locals
//    TCHAR        sBuff[SM_TBLOCK_SIZE] ;
//  
//    smos_sprintf(sBuff,_T("  CFace[0x%p] has %ld children \n"), this, m_pFaces->GetSize()) ;
//    smos_WriteBuffer(sBuff);
//  
//    ULONG ii;
//    for(ii=0;ii<m_pFaces->GetSize();ii++)
//      {
//        SmFace *pFace = (*m_pFaces)[ii] ;
//  
//        smos_sprintf(sBuff,_T("    CFace[0x%p] -> Face [0x%p] \n"), this, pFace) ;
//        smos_WriteBuffer(sBuff);
//      }
//  
//    if(lWalkDepth > 0)
//      {
//        for(ii=0;ii<m_pFaces->GetSize();ii++)
//          {
//            SmFace *pFace = (*m_pFaces)[ii] ;
//  
//            pFace->DumpTopology(lWalkDepth == 99 ? lWalkDepth : lWalkDepth - 1) ; 
//          }
//      }
//  
//  } // end SmCFace::DumpTopology
