// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
* FILE NAME --- SmSpaceBend.cpp
* PURPOSE: Implementation of brep bend/unbend methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmSpaceBend.h>
#include <SmGraphicsOutput.h>
#include <SmPrimitiveCreation.h>
#include <SmMerge.h>
#include <SmTopologyTraverser.h>
#include <SmBendVolume.h>
#include <SmCrvInVolume.h>
#include <SmSrfInVolume.h>
#include <SmAssertArray.h>
#include <SmSpaceUnbend.h>
#include <SmAttribute.h>
#include <SmFace.h>
#include <SmEdge.h>
// Remove Composites
//  #include <SmCFace.h>
// Remove Composites
//  #include <SmCEdge.h>

/*******************************************************************//**
PURPOSE:  create SpaceBend inverse of input SpaceUnbend

NOTES:  
***********************************************************************/
SmStatus SmSpaceBend::CreateBendInverse
 (const SmSpaceUnbend & crSpaceUnbend,  // in : the SpaceUnbend to invert
  SmBrep              & rBrep,          // in : Brep target for new rpSaceBend
  SmSpaceBend        *& rpSpaceBend)    // out: the new SpaceBend inverse to the input SpaceUnbend.
                                        //      NULL on input.
{
  // init input 
  SE_MSG(rpSpaceBend == NULL, _T("warning: SmSpaceBend::CreateBendInverse given a NonNULL rpSpaceBend input argument - potential memory leak")) ;
  rpSpaceBend = NULL ;

  // check input 
  AERN_MSG(crSpaceUnbend.AssertValid(),SM_ERR,_T("SmSpaceBend::CreateBendInverse given an invalid or uninitialized SpaceUnbend argument")) ;

  // locals
  SmBrep            * pBrep               = &rBrep ;
  double              dBendAngDeg         = crSpaceUnbend.GetUnbendAngDeg() ;
  double              dBendRadius         = crSpaceUnbend.GetNeutralRadius() ;
  SmExtent1d          rBendInterval       = crSpaceUnbend.GetUnbendInterval() ;
  double              dBendSheetThickness = crSpaceUnbend.GetSheetThickness() ;
  double              dBendKFactor        = crSpaceUnbend.GetKFactor() ;
  SmBendOrientTYPE    eBendOrient         = MapType(crSpaceUnbend.GetUnbendOrient()) ;  
  double              dBendApproxTol      = crSpaceUnbend.GetApproxTol() ; 
                                                                            
  // Unbend position is the BendPosition mapped through the InsideBendTransform ;
  SmPoint3d    sBendAxisPoint, sUnbendAxisPoint = crSpaceUnbend.GetUnbendAxisPoint() ;
  SmVector3d   sBendAxis,      sUnbendAxis      = crSpaceUnbend.GetUnbendAxis() ; 
  SmVector3d   sBendDirection, sUnbendDirection = crSpaceUnbend.GetUnbendDirection() ;
  SmAxis2Placement sInsideBendTransform = crSpaceUnbend.GetInsideUnbendTransform() ;

  sInsideBendTransform.TransformPoint (sUnbendAxisPoint, sBendAxisPoint) ;
  sInsideBendTransform.TransformVector(sUnbendAxis     , sBendAxis     ) ;
  sInsideBendTransform.TransformVector(sUnbendDirection, sBendDirection) ;
        
  // inverse SpaceUnbend constructor
  rpSpaceBend = new (pBrep->GetContext()) SmSpaceBend
   (*pBrep,              // in : target Brep to unbend
    dBendAngDeg,         // in : bend angle in degrees
    dBendRadius,         // in : bend radius at the neutral plane
    sBendAxisPoint,      // in : BendAxisPoint   of bend Centerline = BendAxisPoint + s * BendAxisUnitVec. 
    sBendAxis,           // in : BendAxisUnitVec of bend Centerline = BendAxisPoint + s * BendAxisUnitVec,
                         //      "the bend's Z Axis."
    sBendDirection,      // in : unit-vector from BendAxisPoint towards NeutralPlane center,
                         //      perpendicular to the BendAxis and perpendicular to the neutral plane.
                         //      bend's X Axis.
    rBendInterval,       // in : active segment of bend Centerline = BendAxisPoint + s * BendAxis.
                         //      use BendInterval.Init() for an unbounded bend line.
    dBendSheetThickness, // in : SheetThickness of NeutralRadius = InsideRadius + KFactor * SheetThickness
    dBendKFactor,        // in : KFactor        of NeutralRadius = InsideRadius + KFactor * SheetThickness
                         //      neutral plane normalized param between inside and outside sheet surfaces, commonly [.44]
    eBendOrient,         // selects the final orientation of the bent part. oneof:
                         // SM_UO_FIXED_BEFORE_BENDING = topology in front of bend is fixed - after topology is rotated into position
                         // SM_UO_CENTER_BENDING       = topology on either side of bend is rotated in equal and opposite directions 
                         // SM_UO_FIXED_AFTER_BENDING  = topology after bend is fixed - before topology is rotated into position
    dBendApproxTol) ;    //      default:[1.0e-5] - for now SmBrep::ApproximateWithBSplines() has trouble hitting
                         //                         this tolerance - currently bounded by 1.0e-4 or larger.

  // all done
  return(SM_SUCCESS) ;

} // end SmSpaceBend::CreateBendInverse

/*******************************************************************//**
PURPOSE:  Create on the heap and rtn a SpaceBend defining BendVolume

NOTES:  
***********************************************************************/
SmBendVolume * SmSpaceBend::CreateBendVolume
 (const SmContext &crContext)   // in : context for new obj construction
 const 
{
    SmVector3d sAxisPt = GetBendAxisPoint();      // in : InSpace (and OutSpace) point on the bend center line                                             
    SmVector3d sBendAxis = GetBendAxis();                    // in : InSpace (and OutSpace) direction of the bend center line marking W axis of Bend Coordinate System
    SmVector3d sBendDir = GetBendDirection();      // in : InSpace (and OutSpace) vector orthoganal to BendAxis marking U axis of Bend Coordinate System
    double sGetNeutralRad = GetNeutralRadius();

  // construct SmBendVolume object
  SmBendVolume * pBendVolume = new (crContext) SmBendVolume(
    crContext,          // in : context for new OrientMap construction 
    sAxisPt,            // in : InSpace (and OutSpace) point on the bend center line                                             
    sBendAxis,          // in : InSpace (and OutSpace) direction of the bend center line marking W axis of Bend Coordinate System
    sBendDir,           // in : InSpace (and OutSpace) vector orthoganal to BendAxis marking U axis of Bend Coordinate System
    sGetNeutralRad) ;   // in : InSpace (and OutSpace) distance along BendMidDir from BendOrigin to NeutralPlane.
                        // in : min dist between NonSingular pts and Volume's singularity at U=0,
                        //      dSingularityTol must be >= SM_ZONE_TOL_3D
                        // note: The BendMidDir/BendBinormal plane is the symmetric plane
                        //        of the Bend mapping
 

  // all done
  return( pBendVolume ) ;

} // end SmSpaceBend::CreateBendVolume

/*******************************************************************//**
PURPOSE:  Apply Bend to Brep

NOTES:  

METHOD: 
 1. Merge TgtBrep with boundaries of InDomain Brep model to ensure all 
    TgtBrep topology objects lie within only a single domain of the bend mapping.
     - sInFaces: Use Boolean Attribute labeling to build sInFaces list:[all faces with the Bend's InDomain region]
 2. Build Start and Stop Topology lists:[all verts and edges on the Start and Stop BendBoundaries] - uses Boolean to find those
     - sStartTopology and
     - sStopTopology 
 3. Build Bend Region Topology Lists of TgtBrep verts, edges, and faces
     - sBeforeFaces,    sAfterFaces      uses marks and SmTopologyTraverser
     - sBeforeEdges,    sAfterEdges      uses marks and SmTopologyTraverser
     - sBeforeVertices, sAfterVertices   uses marks and SmTopologyTraverser
// Remove Composites
//  4. Replace composite faces and edges spanning multiple Bend regions with non-Composite faces and edges
//     (this has to be done but also exposes existing Boolean bug of not working properly for case SmBrep::m_bMakeComposites == FALSE]
 5. Shrink all sInFaces geometry - that's useful later if surfaces need to be approximated
 6. Build  - BendVolume for InBend Geometry
           - rotate and move transform for BeforeBend Geometry
           - rotate and move transform for InsideBend Geometry
           - rotate and move transform for AfterBend  Geometry
 7. Bend and move Geometry
     - bend InBend geometry 
        - BendTopology(BendVolume, sInFace, sInEdges, sInVertices) ;
     - position regions with
        - TransformTopology(sBeforeBendTransform, sBeforeFaces, sBeforeEdges, sBeforeVertices) 
        - TransformTopology(sInsideBendTransform, sInFaces,     sInEdges,     sInVertices    ) 
        - TransformTopology(sAfterBendTransform,  sAfterFaces,  sAfterEdges,  sAfterVertices ) 
 8. When asked: Approximate SmSurfInVolume and SmCrvInVolume geometries with BSpline Geometry
     
***********************************************************************/
SmStatus SmSpaceBend::DoSpaceBend
 (SmBoolean bDoApproximations)  // in : TRUE=replace CrvInVolume and SrfInVolume geometry with BSpline Approximations
                                //      default:[TRUE]
{

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
ULONG di ;
  // draw 
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ; // pretty print: bend params, TgtBrep, InDomainBrep, StartBdryBrep, StopBdryBrep

      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(m_pBrep) { m_pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,1,1) ; this->Draw(FALSE, FALSE, TRUE) ;  sm_GraphicsLoop() ; // bDrawBrep, bAddToUIPickList, bDrawPreBendRegion 
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // locals
  ULONG ii, jj, kk ;
  SmFace          * pFace ;   // iteration variables
  SmEdge          * pEdge ;   // iteration variables
  SmVertex        * pVertex ; // iteration variables
  ULONG             lFoundIdx ;
  SmBrep          * pResult ; 
  ULONG             lIntersectAttributeId = SM_AI_BOOLEAN_INTERSECT ;
  ULONG             lSaveAttributeId      = SM_AI_BOOLEAN_SAVE ;
  ULONG             lDeleteAttributeId    = SM_AI_BOOLEAN_DELETE ;
  const SmContext * cpContext             = GetContext() ; 

  // locals for traversing
  SmTopologyTraverser    sTraverser ;
  SmTArray<SmFace *>     sInFaces, sCompositeInFaces ;
  SmTArray<SmEdge*>      sInEdges, sCompositeInEdges ;
  SmTArray<SmVertex*>    sInVertices ;
  SmTArray<SmTopology *> sStartTopology ;
  SmTArray<SmTopology *> sStopTopology ;

#ifdef SM_DEBUG_CODE
  // draw 
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(GetBendDomain()) ;        // brep modeling the in-segement of the bend
      SM_DUMP_AND_ASSERT_VALID(GetStartBendBoundary()) ; // A Brep representing the sheet start-bend boundary of m_pBendDomain
      SM_DUMP_AND_ASSERT_VALID(GetStopBendBoundary()) ;  // A Brep representing the sheet stop-bend boundary of m_pBendDomain
      SM_DUMP_AND_ASSERT_VALID(m_pBrep) ;                // tgt Brep to bend
      
      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(m_pBrep) { m_pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,0,0) ; if(GetBendDomain()) GetBendDomain()->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetDrawCrossHatch(TRUE) ;
      smgfx_SetLook(1,2, 0,1,0) ; if(GetStartBendBoundary()) GetStartBendBoundary()->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; if(GetStopBendBoundary()) GetStopBendBoundary()->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetDrawCrossHatch(FALSE) ;
      smgfx_SetLook( 1, 2, 0, 1, 1 ); this->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
  
  // STEP 1: build sInFaces list containing all Faces within the UnbendRegion
  { // imprint BendRegion on m_pBrep - gather faces inside the BendRegion and topology on BendRegion Boundary          
    SmBrep * pPreBendRegion     = GetBendDomain() ;     // builds PreBendRegion when needed

// Remove Composites
// // don't make composite edges and faces when geometry is split
//    // note: this feature does not work as expected - that's okay, we'll eliminate problem composites before executing the bend
//    m_pBrep->GetMakeComposites() ;
//    SmTemporaryChangeValue<SmBoolean> sChange (m_pBrep->m_bMakeComposites, FALSE);

    // construct merge object to imprint the BendRegion on m_pBrep
    SmMerge sMergeBendRegion(*cpContext, m_pBrep, pPreBendRegion) ;
    sMergeBendRegion.SetClassifyFaces(TRUE) ; // within booleans only IIR and label faces to be saved and removed.  

    // remove all attributes with attributeIds that are used to track topology in the upcoming NonManifoldBoolean() call
    m_pBrep->RemoveAttributeFromTopology(lIntersectAttributeId) ;
    m_pBrep->RemoveAttributeFromTopology(lSaveAttributeId) ;
    m_pBrep->RemoveAttributeFromTopology(lDeleteAttributeId) ;

    // imprint the PreBendRegion onto the target Brep
    //  1. Splits topology at the BendRegion boundaries making sure that
    //     all m_pBrep topology is either completely in, on, or out of the bendRegion
    //  2. Adds attributes with attributeIds SM_AI_BOOLEAN_INTERSECT to all topology on the BendRegion Boundaries
    //                                       SM_AI_BOOLEAN_SAVE      to all faces in the BendRegion    
    //                                       SM_AI_BOOLEAN_DELETE    to all faces out of the BendRegion  

#ifdef SM_DEBUG_CODE
    if(bDebugMe) // pre Boolean Split at Start and Stop InDomain boundarys
      { SM_DUMP_AND_ASSERT_VALID(pPreBendRegion) ; 
        SM_DUMP_AND_ASSERT_VALID(m_pBrep) ;    
      }
#endif // SM_DEBUG_CODE

    sMergeBendRegion.NonManifoldBoolean(SM_BO_INTERSECTION, pResult) ;  // SetClassify:[TRUE] = only IIR and label faces

#ifdef SM_DEBUG_CODE
    if(bDebugMe) // post Boolean Split at Start and Stop InDomain boundarys
      { SM_DUMP_AND_ASSERT_VALID(pPreBendRegion) ;
        SM_DUMP_AND_ASSERT_VALID(m_pBrep) ; // m_pBrep == pResult here  
      }
#endif // SM_DEBUG_CODE

    // get the faces in the bend 
    m_pBrep->GetFaces(sInFaces,    &lSaveAttributeId) ;

#ifdef SM_DEBUG_CODE
    // draw 
    if(bDebugMe)
      {
        smgfx_Erase() ;
        smgfx_SetLook( 1, 2, 0, 0, 1 ); if(m_pBrep) { m_pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
        smgfx_SetLook(1,2, 0,0,0) ; if(GetBendDomain()) GetBendDomain()->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetDrawCrossHatch(TRUE) ;
        smgfx_SetLook(1,2, 0,1,0) ; if(GetStartBendBoundary()) GetStartBendBoundary()->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetLook(1,2, 1,0,0) ; if(GetStopBendBoundary()) GetStopBendBoundary()->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetDrawCrossHatch(FALSE) ;
        smgfx_SetLook( 1, 2, 0, 1, 1 ); this->Draw(); sm_GraphicsLoop();
        smgfx_SetLook(1,2, 0,1,1) ; 
        for(di=0;di<sInFaces.GetSize();di++) 
        { 
            pFace = sInFaces[di];  
            if(pFace) { pFace->DrawUV( 9, 9 ); sm_GraphicsLoop(); }
        }
        sm_GraphicsLoop() ;
      }
#endif // SM_DEBUG_CODE

   // done later
   // // for all in faces - shrink geometry, that's useful later if surfaces need to be approximated
   // for(ii=0;ii<sInFaces.GetSize();ii++)
   //   {
   //     SmFace *pInFace = sInFaces[ii] ;
   //     pInFace->ShrinkGeometry() ;
   //   }

    // all done with the imprint attributes - clean up m_pBrep for next time
    m_pBrep->RemoveAttributeFromTopology(lIntersectAttributeId) ;
    m_pBrep->RemoveAttributeFromTopology(lSaveAttributeId) ;
    m_pBrep->RemoveAttributeFromTopology(lDeleteAttributeId) ;

  } // end STEP 1: imprint BendRegion on m_pBrep
  
  // STEP 2a: build sStartTopology List containing all Edges and Vertices on the StartBendBoundary
  { // Imprint MergeStartBoundary on m_pBrep to gather list of BendRegion StartBoundary Topology objects
    SmBrep * pStartBendBoundary = GetStartBendBoundary() ; // builds StartBendBoundary when needed

    // Set Merge objects for imprinting
    SmMerge sMergeStartBendBoundary(*cpContext, m_pBrep, pStartBendBoundary) ;
    sMergeStartBendBoundary.SetClassifyFaces(TRUE) ; // GWC: when SetClassifyFaces == TRUE, m_pBreBendRegion Brep not consumed in Boolean

    // remove attributes with attributeIds used to track set membership
    // imprint the PreBendRegion onto the target Brep
    //  1. Expect no topology changes - those are already done with the last SmMerge::NonManifoldBoolean() call.
    //  2. Adds attributes with attributeIds SM_AI_BOOLEAN_INTERSECT to all topology on the BendRegion Boundaries
    //                                       SM_AI_BOOLEAN_SAVE      to all faces in the BendRegion    
    //                                       SM_AI_BOOLEAN_DELETE    to all faces out of the BendRegion  
    sMergeStartBendBoundary.NonManifoldBoolean(SM_BO_INTERSECTION, pResult) ; // SetClassify:[TRUE] = only IIR and label faces
  
#ifdef SM_DEBUG_CODE
    if(bDebugMe) // post Boolean
      { SM_DUMP_AND_ASSERT_VALID(m_pBrep) ; // m_pBrep == pResult here
      }
#endif // SM_DEBUG_CODE

    // get the list of topology on the StartBoundary
    m_pBrep->GetTopology(sStartTopology, & lIntersectAttributeId) ;

    // all done with the imprint attributes - clean up m_pBrep for next time
    m_pBrep->RemoveAttributeFromTopology(lIntersectAttributeId) ;
    m_pBrep->RemoveAttributeFromTopology(lSaveAttributeId) ;
    m_pBrep->RemoveAttributeFromTopology(lDeleteAttributeId) ;

  } // end STEP 2a: imprint m_pBrep with BendRegion Start Boundary

  // STEP 2b: build sStopTopology list containing all Edges and Vertices on the StopUnbendBoundary
  { // Imprint MergeStopBoundary on m_pBrep to gather list of BendRegion StopBoudnary Topology objects
    SmBrep * pStopBendBoundary  = GetStopBendBoundary() ;  // builds StopBendBoundary when needed
    
    // Set Merge objects for imprinting
    SmMerge sMergeStopBendBoundary (*cpContext, m_pBrep, pStopBendBoundary ) ;
    sMergeStopBendBoundary.SetClassifyFaces(TRUE) ;

    // imprint the PreBendRegion onto the target Brep
    //  1. Expect no topology changes - those are already done with the last SmMerge::NonManifoldBoolean() call.
    //  2. Adds attributes with attributeIds SM_AI_BOOLEAN_INTERSECT to all topology on the BendRegion Boundaries
    //                                       SM_AI_BOOLEAN_SAVE      to all faces in the BendRegion    
    //                                       SM_AI_BOOLEAN_DELETE    to all faces out of the BendRegion  
    sMergeStopBendBoundary.NonManifoldBoolean(SM_BO_INTERSECTION, pResult) ; // SetClassify:[TRUE] = only IIR and label faces
  
#ifdef SM_DEBUG_CODE
    if(bDebugMe) // post Boolean
      { SM_DUMP_AND_ASSERT_VALID(m_pBrep) ;  // m_pBrep == pResult here
      }
#endif // SM_DEBUG_CODE

    // get the list of topology on the StopBoundary
    m_pBrep->GetTopology(sStopTopology, & lIntersectAttributeId) ;

    // all done with the imprint attributes - clean up m_pBrep for next time
    m_pBrep->RemoveAttributeFromTopology(lIntersectAttributeId) ;
    m_pBrep->RemoveAttributeFromTopology(lSaveAttributeId) ;
    m_pBrep->RemoveAttributeFromTopology(lDeleteAttributeId) ;

  } // end STEP 2b: imprint m_pBrep with BendRegion Stop Boundary
  
  // arrive here after
  // 1. m_pBrep has been split along the bend boundary regions.
  // 2. a. sInFaces contains all Faces within the BendRegion.
  //    b. sStartTopology contains all Edges and Vertices on the StartBendBoundary.
  //    c. sStopTopology  contains all Edges and Vertices on the StopBendBoundary.
   
  // gather the BeforeBend and AfterBend topology objects in these lists.
  SmTArray<SmFace *>   sBeforeFaces,    sAfterFaces,    sThisFaces, sInFacesFromComposites ;    
  SmTArray<SmEdge *>   sBeforeEdges,    sAfterEdges,    sThisEdges ;    
  SmTArray<SmVertex *> sBeforeVertices, sAfterVertices, sThisVertices ; 

  // STEP 3: Next a. Use sInFaces, SmTopologyTraverser::GetSubTopologies(), and the mark mechanism to build sInEdges and sInVertices.
  //              b. Use SmTopologyTraverser::CollectFaces(), the mark mechanism, 
  //                   and sStartTopology, to build sBeforeFaces, sBeforeEdges, and sBeforeVertices,
  //                   and sStopTopology, to build sAfterFaces, sAfterEdges, and sAfterVertices.
  {
    // increment the context mark used to track traversal progress
    SmNewMarkAndLock sMarkLock(GetContext(), SM_MT_ALLMARKS);
    SmMarkType eMarkType = sMarkLock.GetMarkType() ;

    // gather and mark all faces, edges, and vertices in the bend by traversing the topology graph from the inBend face list
    // Temp workaround to appease Linux gcc compiler
    // Original code with typecast has warning: dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]
    // sTraverser.GetSubTopologies((SmTArray<SmTopology *> &)(sInFaces),
    //                             NULL,         // i/o: regions
    //                             NULL,         // i/o: shells
    //                            &sInFaces,     // i/o: faces
    //                             NULL,         // i/o: loops
    //                            &sInEdges,     // i/o: edgse
    //                            &sInVertices,  // i/o: vertices
    //                             FALSE,        // in : bReSetArrays
    //                             eMarkType) ; // in : use this mark

    SmTArray<SmTopology*> sTopology;
    sTraverser.GetSubTopologies(sTopology,
                                NULL,         // i/o: regions
                                NULL,         // i/o: shells
                               &sInFaces,     // i/o: faces
                                NULL,         // i/o: loops
                               &sInEdges,     // i/o: edgse
                               &sInVertices,  // i/o: vertices
                                FALSE,        // in : bReSetArrays
                                eMarkType) ;  // in : use this mark
    sInFaces.ReSet();
    for (ii = 0;ii < sTopology.GetSize(); ii++) { sInFaces.Add( (SmFace*)sTopology[ii] ); };

    // iter locals
    SmTArray<SmFace *> sEdgeFaces ;
    SmTArray<SmEdge *> sVertexEdges ;
  
    // for two passes (once for edges, once for vertices)
    for(ii=0;ii<2;ii++)
      {
        for(jj=0;jj<sStartTopology.GetSize();jj++)
          {
            SmTopology *pTopology = sStartTopology[jj] ;
            pEdge                 = SM_CAST_PTR(SmEdge, pTopology) ;
            pVertex               = SM_CAST_PTR(SmVertex, pTopology) ;

            // on pass 0 - use sStartTopology edges to find beforeFaces
            if(ii == 0 && pEdge != NULL) 
              {
                // get all faces on Edge
                sEdgeFaces.ReSet() ;
                pEdge->GetFaces(sEdgeFaces) ;

                // find a face that is not marked - it will be outside the bend region
                for(kk=0;kk<sEdgeFaces.GetSize();kk++)
                  {
                    pFace = sEdgeFaces[kk] ;

                    // skip marked faces (that are inside the bend region or already processed)
                    if(pFace->IsMarked(eMarkType))
                      { continue ; }

                    // find all the unmarked faces, edges, and vertices connected to this one
                    sTraverser.CollectFaces(pFace, sThisFaces, eMarkType, &sThisEdges, &sThisVertices) ;

                    // accumulate the results
                    sBeforeFaces.Append(sThisFaces) ;
                    sBeforeEdges.Append(sThisEdges) ;
                    sBeforeVertices.Append(sThisVertices) ;

                  }
              } // end Found an intersection Edge with unmarked faces branch
          
            // on pass 1 - use sStartTopology vertices to find before WireEdges
            else if(ii == 1 && pVertex != NULL)
              {
                // get all faces on Edge
                sVertexEdges.ReSet() ;
                pVertex->GetEdges(sVertexEdges) ;

                // find an edge that is not marked - it will be outside the bend region and a wire
                for(kk=0;kk<sVertexEdges.GetSize();kk++)
                  {
                    SmEdge *pVEdge = sVertexEdges[kk] ;

                    // skip marked edges (that are inside the bend region or already processed)
                    if(pVEdge->IsMarked(eMarkType))
                      { continue ; }

                    // find all the unmarked edges and vertices connected to this one
                    sTraverser.CollectWireEdges(pVEdge, sThisEdges, eMarkType, &sThisVertices) ;

                    // accumulate the results
                    sBeforeEdges.Append(sThisEdges) ;
                    sBeforeVertices.Append(sThisVertices) ;

                  }
              } // end Found an intersection Vertex with unmarked Edges branch
          } // end iter every StartTopology Object
      } // end iter two times, ii==0 for edges and ii== 1 for vertices

    // restore the mark state so that all InBend topology is marked and all OutBend is not

    // increment the context mark used to track traversal progress
    sMarkLock.NewMark() ; 

    // mark the InBend geometry
    for(ii=0;ii<sInFaces.GetSize();ii++)    { sInFaces[ii]->Mark(eMarkType) ; }
    for(ii=0;ii<sInEdges.GetSize();ii++)    { sInEdges[ii]->Mark(eMarkType) ; }
    for(ii=0;ii<sInVertices.GetSize();ii++) { sInVertices[ii]->Mark(eMarkType) ; }

    // for two passes (once for edges, once for vertices)
    for(ii=0;ii<2;ii++)
      {
        for(jj=0;jj<sStopTopology.GetSize();jj++)
          {
            SmTopology *pTopology = sStopTopology[jj] ;
            pEdge                 = SM_CAST_PTR(SmEdge, pTopology) ;
            pVertex               = SM_CAST_PTR(SmVertex, pTopology) ;

            // on pass 0 - use sStopTopology edges to find AfterFaces
            if(ii == 0 && pEdge != NULL) 
              {
                // get all faces on Edge
                sEdgeFaces.ReSet() ;
                pEdge->GetFaces(sEdgeFaces) ;

                // find a face that is not marked - it will be outside the bend region
                for(kk=0;kk<sEdgeFaces.GetSize();kk++)
                  {
                    pFace = sEdgeFaces[kk] ;

                    // skip marked faces (that are inside the bend region or already processed)
                    if(pFace->IsMarked(eMarkType))
                      { continue ; }

                    // find all the unmarked faces, edges, and vertices connected to this one
                    sTraverser.CollectFaces(pFace, sThisFaces, eMarkType, &sThisEdges, &sThisVertices) ;

                    // accumulate the results
                    sAfterFaces.Append(sThisFaces) ;
                    sAfterEdges.Append(sThisEdges) ;
                    sAfterVertices.Append(sThisVertices) ;

                  }
              } // end Found an intersection Edge with unmarked faces branch
          
            // on pass 1 - use sStartTopology vertices to find after WireEdges
            else if(ii == 1 && pVertex != NULL)
              {
                // get all faces on Edge
                sVertexEdges.ReSet() ;
                pVertex->GetEdges(sVertexEdges) ;

                // find an edge that is not marked - it will be outside the bend region and a wire
                for(kk=0;kk<sVertexEdges.GetSize();kk++)
                  {
                    SmEdge * pVEdge = sVertexEdges[kk] ;

                    // skip marked edges (that are inside the bend region or already processed)
                    if(pVEdge->IsMarked(eMarkType))
                      { continue ; }

                    // find all the unmarked edges and vertices connected to this one
                    sTraverser.CollectWireEdges(pVEdge, sThisEdges, eMarkType, &sThisVertices) ;

                    // accumulate the results
                    sAfterEdges.Append(sThisEdges) ;
                    sAfterVertices.Append(sThisVertices) ;

                  }
              } // end Found an intersection Vertex with unmarked Edges branch
          } // end iter every StartTopology Object
      } // end iter two times, ii==0 for edges and ii== 1 for vertices
  } // end STEP 3: building spatially defined groupings of Topology objects

  // arrive here when all the geometry containing topology objects have been classified by region into the lists:
  //   sInFaces,    sBeforeFaces,    sAfterFaces
  //   sInEdges,    sBeforeEdges,    sAfterEdges
  //   sInVertices, sBeforeVertices, sAfterVertices.
  
// Remove Composites
//  // STEP 4: next replace composite faces and edges spanning bend and unbend regions with non-Composite faces and edges.
//   {
// #ifdef SM_DEBUG_CODE
//     // pretty print and draw m_pBrep and Before/In/After Face/Edge/Vertex lists 
//     if(bDebugMe) // dump and draw gathered list geometry before replace composites
//       {
//         if(m_pBrep) m_pBrep->Dump(SM_BD_GEOM_TYPES) ;
//         SM_ASSERT_VALID(m_pBrep) ;
// 
//         // list the contents of the topology list arrays
//         smos_WriteBuffer(_T("\nBEFORE REPLACE COMPOSITES sInFaces       : ")); SM_DUMP_TARRAY(sInFaces) ;
//         smos_WriteBuffer(_T("\nBEFORE REPLACE COMPOSITES sInEdges       : ")); SM_DUMP_TARRAY(sInEdges) ;
//         smos_WriteBuffer(_T("\nBEFORE REPLACE COMPOSITES sInVertices    : ")); SM_DUMP_TARRAY(sInVertices) ;
//         smos_WriteBuffer(_T("\n"));                                            
//         smos_WriteBuffer(_T("\nBEFORE REPLACE COMPOSITES sBeforeFaces   : ")); SM_DUMP_TARRAY(sBeforeFaces) ;
//         smos_WriteBuffer(_T("\nBEFORE REPLACE COMPOSITES sBeforeEdges   : ")); SM_DUMP_TARRAY(sBeforeEdges) ;
//         smos_WriteBuffer(_T("\nBEFORE REPLACE COMPOSITES sBeforeVertices: ")); SM_DUMP_TARRAY(sBeforeVertices) ;
//         smos_WriteBuffer(_T("\n"));                                            
//         smos_WriteBuffer(_T("\nBEFORE REPLACE COMPOSITES sAfterFaces    : ")); SM_DUMP_TARRAY(sAfterFaces) ;
//         smos_WriteBuffer(_T("\nBEFORE REPLACE COMPOSITES sAfterEdges    : ")); SM_DUMP_TARRAY(sAfterEdges) ;
//         smos_WriteBuffer(_T("\nBEFORE REPLACE COMPOSITES sAfterVertices : ")); SM_DUMP_TARRAY(sAfterVertices) ;
//         smos_WriteBuffer(_T("\n"));
// 
//         // draw the lists
//         smgfx_Erase() ;
//         smgfx_SetLook( 1, 2, 0, 0, 1 ); if(m_pBrep) { m_pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
//         smgfx_SetLook( 1, 2, 0, 1, 1 ); this->Draw(); sm_GraphicsLoop();
// 
//         smgfx_SetLook(1,2,0,0,.5) ; 
//         for(di=0;di<sInFaces.GetSize();di++) 
//         { 
//             pFace = sInFaces[di];
//             if(pFace) { pFace->DrawUV( 9, 9 ); sm_GraphicsLoop(); }
//         }
//         smgfx_SetLook(3,4,0,0, 1) ; 
//         for(di=0;di<sInEdges.GetSize();di++) 
//         { 
//             pEdge = sInEdges[di]; 
//             if(pEdge) { pEdge->Draw(); sm_GraphicsLoop(); }
//         }
//         smgfx_SetLook(7,8,0,0, 1) ; 
//         for(di=0;di<sInVertices.GetSize();di++) 
//         { 
//             pVertex = sInVertices[di];
//             if(pVertex) { pVertex->Draw(); sm_GraphicsLoop(); }
//         }
// 
//         smgfx_SetLook(1,2,0,.5,0) ; 
//         for(di=0;di<sBeforeFaces.GetSize();di++) 
//         { 
//             pFace = sBeforeFaces[di];
//             if(pFace) { pFace->DrawUV( 9, 9 ); sm_GraphicsLoop(); }
//         }
//         smgfx_SetLook(3,4, 0,1,0) ; 
//         for(di=0;di<sBeforeEdges.GetSize();di++) 
//         { 
//             pEdge = sBeforeEdges[di];
//             if(pEdge) { pEdge->Draw(); sm_GraphicsLoop(); }
//         }
//         smgfx_SetLook(7,8, 0,1,0) ; 
//         for(di=0;di<sBeforeVertices.GetSize();di++) 
//         { 
//             pVertex = sBeforeVertices[di];
//             if(pVertex) { pVertex->Draw(); sm_GraphicsLoop(); }
//         }
// 
//         smgfx_SetLook(1,2,.5,0,0) ; 
//         for(di=0;di<sAfterFaces.GetSize();di++) 
//         { 
//             pFace = sAfterFaces[di];
//             if(pFace) { pFace->DrawUV( 9, 9 ); sm_GraphicsLoop(); }
//         }
//         smgfx_SetLook(3,4, 1,0,0) ; 
//         for(di=0;di<sAfterEdges.GetSize();di++) 
//         { 
//             pEdge = sAfterEdges[di];
//             if(pEdge) { pEdge->Draw(); sm_GraphicsLoop(); }
//         }
//         smgfx_SetLook(7,8, 1,0,0) ; 
//         for(di=0;di<sAfterVertices.GetSize();di++) 
//         { 
//             pVertex = sAfterVertices[di];
//             if(pVertex) { pVertex->Draw(); sm_GraphicsLoop(); }
//         }
//         sm_GraphicsLoop() ;
//       }
// #endif // SM_DEBUG_CODE
// 
//     // gather the AfterBend topology objects in these lists.
//     SmTArray<SmFace *>   sCompositeBeforeFaces,    sCompositeAfterFaces,    sCommonFaces ;      
//     SmTArray<SmEdge *>   sCompositeBeforeEdges,    sCompositeAfterEdges,    sCommonEdges ;      
//     SmTArray<SmVertex *> sCompositeBeforeVertices, sCompositeAfterVertices, sCommonVertices ;  
// 
//     // watch out composite faces and edges - copy arrays and make sure top faces for member faces get added to the lists
//     sCompositeBeforeFaces.ReSet() ; sCompositeAfterFaces.ReSet() ; sCompositeInFaces.ReSet() ;
//     sCompositeBeforeEdges.ReSet() ; sCompositeAfterEdges.ReSet() ; sCompositeInEdges.ReSet() ;
// 
//     // make sure any CEdge parent is on the list with a CEdge child
//     if(m_pBrep->GetNumCEdges() > 0)
//       {
//         for(ii=0;ii<sBeforeEdges.GetSize();ii++) { pEdge = sBeforeEdges[ii] ;
//                                                    if(pEdge->IsKindOf(SmCEdge_TYPE))
//                                                      { sCompositeBeforeEdges.AddUnique(pEdge) ; }
//                                                    else if(pEdge->IsCompositeEdge())
//                                                      { sCompositeBeforeEdges.AddUnique(pEdge->GetCompositeEdgeOwner()) ; }
//                                                  }
//         for(ii=0;ii<sAfterEdges.GetSize();ii++)  { pEdge = sAfterEdges[ii] ;
//                                                    if(pEdge->IsKindOf(SmCEdge_TYPE))
//                                                      { sCompositeAfterEdges.AddUnique(pEdge) ; }
//                                                    else if(pEdge->IsCompositeEdge())
//                                                      { sCompositeAfterEdges.AddUnique(pEdge->GetCompositeEdgeOwner()) ; }
//                                                  }
//         for(ii=0;ii<sInEdges.GetSize();ii++)     { pEdge = sInEdges[ii] ;
//                                                    if(pEdge->IsKindOf(SmCEdge_TYPE))
//                                                      { sCompositeInEdges.AddUnique(pEdge) ; }
//                                                    else if(pEdge->IsCompositeEdge())
//                                                      { sCompositeInEdges.AddUnique(pEdge->GetCompositeEdgeOwner()) ; }
//                                                  }
//       } // end any CEdges check
// 
//     // make sure any CFace parent is on the list with a CFace child
//     if(m_pBrep->GetNumCFaces() > 0)
//       {
//         for(ii=0;ii<sBeforeFaces.GetSize();ii++) { pFace = sBeforeFaces[ii] ;
//                                                    if(pFace->IsKindOf(SmCFace_TYPE))
//                                                      { sCompositeBeforeFaces.AddUnique(pFace) ; }
//                                                    else if(pFace->IsCompositeFace())
//                                                      { sCompositeBeforeFaces.AddUnique(pFace->GetCompositeFaceOwner()) ; }
//                                                  }
//         for(ii=0;ii<sAfterFaces.GetSize();ii++)  { pFace = sAfterFaces[ii] ;
//                                                    if(pFace->IsKindOf(SmCFace_TYPE))
//                                                      { sCompositeAfterFaces.AddUnique(pFace) ; }
//                                                    else if(pFace->IsCompositeFace())
//                                                      { sCompositeAfterFaces.AddUnique(pFace->GetCompositeFaceOwner()) ; }
//                                                  }
//         for(ii=0;ii<sInFaces.GetSize();ii++)     { pFace = sInFaces[ii] ;
//                                                    if(pFace->IsKindOf(SmCFace_TYPE))
//                                                      { sCompositeInFaces.AddUnique(pFace) ; 
//                                                        // no need to add InFace composite children to sInFacesFromComposites here
//                                                        // because all children should be on the sInFaces list and will be added in due time.
//                                                      }
//                                                    else if(pFace->IsCompositeFace())
//                                                      { sCompositeInFaces.AddUnique(pFace->GetCompositeFaceOwner()) ;
//                                                        // remember all InFace composite children for later use
//                                                        sInFacesFromComposites.Add(pFace) ; 
//                                                      }
//                                                  }
//       } // end any CFaces check
// 
//       { // Find and Replace composite faces and edges that span more than 1 bend region with stand-alone faces and edges
// 
//         // find composites with members in both Before/After regions
//         sCompositeBeforeFaces.FindCommonElements(sCompositeAfterFaces, sCommonFaces) ;
//         sCompositeBeforeEdges.FindCommonElements(sCompositeAfterEdges, sCommonEdges) ;
// 
//         // find composites with members in both In/After regions
//         sCompositeInFaces.FindCommonElements(sCompositeAfterFaces, sThisFaces) ;  sCommonFaces.AppendUnique(sThisFaces) ;
//         sCompositeInEdges.FindCommonElements(sCompositeAfterEdges, sThisEdges) ;  sCommonEdges.AppendUnique(sThisEdges) ;
// 
//         // find composites with members in both Before/In regions
//         sCompositeBeforeFaces.FindCommonElements(sCompositeInFaces, sThisFaces) ; sCommonFaces.AppendUnique(sThisFaces) ;
//         sCompositeBeforeEdges.FindCommonElements(sCompositeInEdges, sThisEdges) ; sCommonEdges.AppendUnique(sThisEdges) ;
// 
//         // replace spanning composite faces with Non-Composites (leave composites contained in 1 region alone)
//         if(sCommonFaces.GetSize() > 0)
//           { 
//             // replace composite Faces
//             m_pBrep->ReplaceCompositeFaces(&sCommonFaces) ; 
//         
//             // clean up the Composite Face lists
//             sCompositeBeforeFaces.RemoveElements(sCommonFaces, sCompositeBeforeFaces) ; 
//             sCompositeAfterFaces.RemoveElements (sCommonFaces, sCompositeAfterFaces) ; 
//             sCompositeInFaces.RemoveElements    (sCommonFaces, sCompositeInFaces) ; 
//           }
// 
//         // replace spanning composite edges with Non-Composites (leave composites contained in 1 region alone)
//         if(sCommonEdges.GetSize() > 0)
//           { 
//             // replace composite Edges
//             m_pBrep->ReplaceCompositeEdges(&sCommonEdges) ; 
//       
//             // clean up the Composite Edge lists
//             sCompositeBeforeEdges.RemoveElements(sCommonEdges, sCompositeBeforeEdges) ; 
//             sCompositeAfterEdges.RemoveElements (sCommonEdges, sCompositeAfterEdges) ; 
//             sCompositeInEdges.RemoveElements    (sCommonEdges, sCompositeInEdges) ; 
//           }
//       } // end find and replace composite faces and edges that span more than 1 bend region
// 
//       { // error check: any Face/Edge/Vertex on any two Face/Edge/Vertex Region lists is an error
//         ULONG lErrorCnt = 0;
// 
//         sBeforeFaces.FindCommonElements   ( sAfterFaces,   sCommonFaces ) ; 
//         sBeforeEdges.FindCommonElements   ( sAfterEdges,   sCommonEdges ) ;
//         sBeforeVertices.FindCommonElements( sAfterVertices,sCommonVertices ) ;
//         lErrorCnt += sCommonFaces.GetSize() + sCommonEdges.GetSize() + sCommonVertices.GetSize() ;
// 
//         sInFaces.FindCommonElements   ( sBeforeFaces,   sCommonFaces ) ; 
//         sInEdges.FindCommonElements   ( sBeforeEdges,   sCommonEdges ) ;
//         sInVertices.FindCommonElements( sBeforeVertices,sCommonVertices ) ;
//         lErrorCnt += sCommonFaces.GetSize() + sCommonEdges.GetSize() + sCommonVertices.GetSize() ;
// 
//         sInFaces.FindCommonElements   ( sAfterFaces,   sCommonFaces ) ; 
//         sInEdges.FindCommonElements   ( sAfterEdges,   sCommonEdges ) ;
//         sInVertices.FindCommonElements( sAfterVertices,sCommonVertices ) ;
//         lErrorCnt += sCommonFaces.GetSize() + sCommonEdges.GetSize() + sCommonVertices.GetSize() ;
// 
//         if(lErrorCnt > 0)
//           { 
//             SER_MSG(SM_ERR, _T("SmSpaceBend::DoSpaceBend() bad bend request: geometry before the bend connects to after the bend")) ; 
// 
// #ifdef SM_DEBUG_CODE
//             // draw common(on boundary) Before and After geometry
//             if(bDebugMe)
//               {
//                 smgfx_Erase() ;
//                 smgfx_SetLook( 1, 2, 0, 0, 1 ); if(m_pBrep) { m_pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
//                 smgfx_SetLook( 1, 2, 0, 1, 1 ); this->Draw(); sm_GraphicsLoop();
// 
//                 smgfx_SetLook(1,2, .5,1,0) ; 
//                 for(di=0;di<sCommonFaces.GetSize();di++) 
//                 { 
//                     pFace = sCommonFaces[di];
//                     if(pFace) { pFace->DrawUV( 3, 3 ); }
//                 } 
//                 sm_GraphicsLoop();
//                 smgfx_SetLook(4,6, .5,1,0) ; 
//                 for(di=0;di<sCommonEdges.GetSize();di++) 
//                 { 
//                     pEdge = sCommonEdges[di];
//                     if(pEdge) { pEdge->Draw(); }
//                 } sm_GraphicsLoop();
//                 smgfx_SetLook(4,6, .5,1,0) ; 
//                 for(di=0;di<sCommonVertices.GetSize();di++) 
//                 { 
//                     pVertex = sCommonVertices[di];
//                     if(pVertex) { pVertex->Draw(); }
//                 } sm_GraphicsLoop();
//                 sm_GraphicsLoop() ;
//               }
// #endif // SM_DEBUG_CODE
//           } // end ErrorCnt check
//       } // end error check: any Face/Edge/Vertex on any two Face/Edge/Vertex Region lists is an error
// 
// #ifdef SM_DEBUG_CODE
//     // pretty print and draw m_pBrep and Before/In/After Face/Edge/Vertex lists 
//     if(bDebugMe) // dump and draw gathered list geometry after replace composites
//       {
//         if(m_pBrep) m_pBrep->Dump(SM_BD_GEOM_TYPES) ;
//         SM_ASSERT_VALID(m_pBrep) ;
// 
//         // list the contents of the topology list arrays
//         smos_WriteBuffer(_T("\nAFTER REPLACE COMPOSITES sInFaces       : ")); SM_DUMP_TARRAY(sInFaces) ;
//         smos_WriteBuffer(_T("\nAFTER REPLACE COMPOSITES sInEdges       : ")); SM_DUMP_TARRAY(sInEdges) ;
//         smos_WriteBuffer(_T("\nAFTER REPLACE COMPOSITES sInVertices    : ")); SM_DUMP_TARRAY(sInVertices) ;
//         smos_WriteBuffer(_T("\n"));                                            
//         smos_WriteBuffer(_T("\nAFTER REPLACE COMPOSITES sBeforeFaces   : ")); SM_DUMP_TARRAY(sBeforeFaces) ;
//         smos_WriteBuffer(_T("\nAFTER REPLACE COMPOSITES sBeforeEdges   : ")); SM_DUMP_TARRAY(sBeforeEdges) ;
//         smos_WriteBuffer(_T("\nAFTER REPLACE COMPOSITES sBeforeVertices: ")); SM_DUMP_TARRAY(sBeforeVertices) ;
//         smos_WriteBuffer(_T("\n"));                                            
//         smos_WriteBuffer(_T("\nAFTER REPLACE COMPOSITES sAfterFaces    : ")); SM_DUMP_TARRAY(sAfterFaces) ;
//         smos_WriteBuffer(_T("\nAFTER REPLACE COMPOSITES sAfterEdges    : ")); SM_DUMP_TARRAY(sAfterEdges) ;
//         smos_WriteBuffer(_T("\nAFTER REPLACE COMPOSITES sAfterVertices : ")); SM_DUMP_TARRAY(sAfterVertices) ;
//         smos_WriteBuffer(_T("\n"));
// 
//         // draw the lists
//         smgfx_Erase() ;
//         smgfx_SetLook( 1, 2, 0, 0, 1 ); if(m_pBrep) { m_pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
//         smgfx_SetLook( 1, 2, 0, 1, 1 ); this->Draw(); sm_GraphicsLoop();
// 
//         smgfx_SetLook(1,2,0,0,.5) ; 
//         for(di=0;di<sInFaces.GetSize();di++) 
//         { 
//             pFace = sInFaces[di];
//             if(pFace) { pFace->DrawUV( 9, 9 ); sm_GraphicsLoop(); }
//         }
//         smgfx_SetLook(3,4,0,0, 1) ; 
//         for(di=0;di<sInEdges.GetSize();di++) 
//         { 
//             pEdge = sInEdges[di];
//             if(pEdge) { pEdge->Draw(); sm_GraphicsLoop(); }
//         }
//         smgfx_SetLook(7,8,0,0, 1) ; 
//         for(di=0;di<sInVertices.GetSize();di++) 
//         { 
//             pVertex = sInVertices[di];
//             if(pVertex) { pVertex->Draw(); sm_GraphicsLoop(); }
//         }
// 
//         smgfx_SetLook(1,2,0,.5,0) ; 
//         for(di=0;di<sBeforeFaces.GetSize();di++) 
//         { 
//             pFace = sBeforeFaces[di];
//             if(pFace) { pFace->DrawUV( 9, 9 ); sm_GraphicsLoop(); }
//         }
//         smgfx_SetLook(3,4, 0,1,0) ; 
//         for(di=0;di<sBeforeEdges.GetSize();di++) 
//         { 
//             pEdge = sBeforeEdges[di];
//             if(pEdge) { pEdge->Draw(); sm_GraphicsLoop(); }
//         }
//         smgfx_SetLook(7,8, 0,1,0) ; 
//         for(di=0;di<sBeforeVertices.GetSize();di++) 
//         { 
//             pVertex = sBeforeVertices[di];
//             if(pVertex) { pVertex->Draw(); sm_GraphicsLoop(); }
//         }
// 
//         smgfx_SetLook(1,2,.5,0,0) ; 
//         for(di=0;di<sAfterFaces.GetSize();di++) 
//         { 
//             pFace = sAfterFaces[di];
//             if(pFace) { pFace->DrawUV( 9, 9 ); sm_GraphicsLoop(); }
//         }
//         smgfx_SetLook(3,4, 1,0,0) ; 
//         for(di=0;di<sAfterEdges.GetSize();di++) 
//         { 
//             pEdge = sAfterEdges[di];
//             if(pEdge) { pEdge->Draw(); sm_GraphicsLoop(); }
//         }
//         smgfx_SetLook(7,8, 1,0,0) ; 
//         for(di=0;di<sAfterVertices.GetSize();di++) 
//         { 
//             pVertex = sAfterVertices[di];
//             if(pVertex) { pVertex->Draw(); sm_GraphicsLoop(); }
//         }
//         sm_GraphicsLoop() ;
//       }
// #endif // SM_DEBUG_CODE
// 
//   } // end STEP 4: replace composite faces and edges spanning bend and unbend regions with non-Composite faces and edges

  // STEP 5: for all InFaces - shrink geometry. Useful later if surfaces need to be approximated 
  {
    for(ii=0;ii<sInFaces.GetSize();ii++)
      {
        pFace = sInFaces[ii] ;

        // skip InFaces which came from a spanning composite (they've already been shrunk)
        if(sInFacesFromComposites.FindElement(pFace,lFoundIdx))
          { continue ; }

        pFace->ShrinkGeometry() ;
      }
  } // end STEP 5: Shrink sInFaces geometry

  // BendVolume and Before/In/After bend transformations
  SmAxis2Placement sBeforeBendTransform ;
  SmAxis2Placement sInsideBendTransform ;
  SmAxis2Placement sAfterBendTransform ;
  SmBendVolume   * pBendVolume = NULL ;
  SmObjDelete      sClean(pBendVolume) ;  

  // STEP 6: build the BendVolume mapping and the Before/In/After affine transformations for the Bend transformation
  {
#ifdef SM_DEBUG_CODE
    // draw partitioned topology lists - post ShrinkGeometry
    if(bDebugMe)
      {
        if(m_pBrep) m_pBrep->Dump(SM_BD_GEOM_TYPES) ;
        SM_ASSERT_VALID(m_pBrep) ;

        // list the contenst of the topology list arrays
        smos_WriteBuffer(_T("\n sInFaces       : ")); sInFaces.Dump() ;
        smos_WriteBuffer(_T("\n sInEdges       : ")); sInEdges.Dump() ;
        smos_WriteBuffer(_T("\n sInVertices    : ")); sInVertices.Dump() ;
        smos_WriteBuffer(_T("\n")); 
        smos_WriteBuffer(_T("\n sBeforeFaces   : ")); sBeforeFaces.Dump() ;
        smos_WriteBuffer(_T("\n sBeforeEdges   : ")); sBeforeEdges.Dump() ;
        smos_WriteBuffer(_T("\n sBeforeVertices: ")); sBeforeVertices.Dump() ;
        smos_WriteBuffer(_T("\n"));
        smos_WriteBuffer(_T("\n sAfterFaces    : ")); sAfterFaces.Dump() ;
        smos_WriteBuffer(_T("\n sAfterEdges    : ")); sAfterEdges.Dump() ;
        smos_WriteBuffer(_T("\n sAfterVertices : ")); sAfterVertices.Dump() ;
        smos_WriteBuffer(_T("\n"));

        smgfx_Erase() ;
        smgfx_SetLook( 1, 2, 0, 0, 1 ); if(m_pBrep) { m_pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
        smgfx_SetLook(1,2, 0,1,1) ; this->Draw(FALSE, FALSE, TRUE) ;  sm_GraphicsLoop() ;
      
        smgfx_SetLook(4,8, 0,0,1) ; 
        for(di = 0; di < sInVertices.GetSize(); di++)
        {
            pVertex = sInVertices[di];
            if(pVertex) { pVertex->Draw();   sm_GraphicsLoop(); }
        }
        smgfx_SetLook( 4, 8, 0, 0, 1 ); for(di = 0; di < sInEdges.GetSize(); di++)
        {
            pEdge = sInEdges[di];
            if(pEdge) { pEdge->Draw();      sm_GraphicsLoop(); }
        }
        smgfx_SetLook(1,2,0,0,.5) ; 
        for(di = 0; di < sInFaces.GetSize(); di++)
        {
            pFace = sInFaces[di];
            if(pFace) { pFace->DrawUV( 3, 3 ); sm_GraphicsLoop(); }
        }
        smgfx_SetLook(6,10,0,1,0) ; 
        for(di = 0; di < sBeforeVertices.GetSize(); di++)
        {
            pVertex = sBeforeVertices[di];
            if(pVertex) { pVertex->Draw(); sm_GraphicsLoop(); }
        }
        smgfx_SetLook(6,10,0,1,0) ; 
        for(di = 0; di < sBeforeEdges.GetSize(); di++)
        {
            pEdge = sBeforeEdges[di];
            if(pEdge) { pEdge->Draw();    sm_GraphicsLoop(); }
        }
        smgfx_SetLook(1.2,.5,0,0) ;
        for(di = 0; di < sBeforeFaces.GetSize(); di++)
        {
            pFace = sBeforeFaces[di];
            if(pFace) { pFace->DrawUV();  sm_GraphicsLoop(); }
        }

        smgfx_SetLook(8,12,1,0,0) ; 
        for(di = 0; di < sAfterVertices.GetSize(); di++)
        {
            pVertex = sAfterVertices[di];
            if(pVertex) { pVertex->Draw(); sm_GraphicsLoop(); }
        }
        smgfx_SetLook(8,12,1,0,0) ; 
        for(di = 0; di < sAfterEdges.GetSize(); di++)
        {
            pEdge = sAfterEdges[di];
            if(pEdge) { pEdge->Draw();    sm_GraphicsLoop(); }
        }
        smgfx_SetLook(1,2,.5,0,0) ; 
        for(di = 0; di < sAfterFaces.GetSize(); di++)
        {
            pFace = sAfterFaces[di];
            if(pFace) { pFace->DrawUV();  sm_GraphicsLoop(); }
        }

        sm_GraphicsLoop() ;
      }
#endif // SM_DEBUG_CODE

    // build the BendVolume for the InBend geometry
    pBendVolume = CreateBendVolume(*cpContext) ;
    sClean.ReplaceObj(pBendVolume) ;  

    // define the rotate and move transforms for the before, inside and after bend geometry
    sBeforeBendTransform = GetBeforeBendTransform() ;
    sInsideBendTransform = GetInsideBendTransform() ; 
    sAfterBendTransform  = GetAfterBendTransform() ;  

#ifdef SM_DEBUG_CODE                                                                                                   
    // draw the SpaceBend - and Bend Volume
    if(bDebugMe)
      {
        smgfx_SetLook(1,2, 0,1,1) ; this->Draw(FALSE, FALSE, TRUE) ;  sm_GraphicsLoop() ; // bDrawBrep, bAddToUIPickList, bDrawPreBendRegion 
        smgfx_SetLook(1,2, 0,0,1) ; pBendVolume->Draw(TRUE, TRUE) ; sm_GraphicsLoop() ;
        sm_GraphicsLoop() ;
      }
#endif // SM_DEBUG_CODE
  } // end STEP 6: build BendVolume mapping and Before/In/After Affine Transformations
   
  // STEP 7: ModelTranformation: Bend InBendGeometry and apply AffineTransformations to Before/In/After bend geometries
  {
    // change Surface and Edge InBendGeometry into SmSrfinVolume and SmCrvInVolume objects 
    BendTopology(*pBendVolume,    // in : Bend mapping to apply
                 *m_pBrep,        // in : Brep containing all topology to be transformed
                 sInFaces,        // in : faces to transform
                 sInEdges,        // in : edges to transform 
                 sInVertices) ;   // in : vertices to transform 

    // transform Before/In/After geometry to align the topology pieces across the start/end bend planes 
    TransformTopology(sBeforeBendTransform, *m_pBrep, sBeforeFaces, sBeforeEdges, sBeforeVertices) ;
    TransformTopology(sInsideBendTransform, *m_pBrep, sInFaces,     sInEdges,     sInVertices) ;
    TransformTopology(sAfterBendTransform,  *m_pBrep, sAfterFaces,  sAfterEdges,  sAfterVertices) ;

#ifdef SM_DEBUG_CODE                                                                                                   
    // After Bend test and draw the transformed model
    if(bDebugMe)
      {
        ULONG dd ;
        SmTArray<SmSurface*> sSurfaces ;
        SmTArray<SmSrfInVolume*> sSrfInVolumes ;
        m_pBrep->GetSurfaces(sSurfaces) ;
        for(dd=0;dd<sSurfaces.GetSize();dd++) { if(sSurfaces[dd]->IsKindOf(SmSrfInVolume_TYPE)) { sSrfInVolumes.Add((SmSrfInVolume*)sSurfaces[dd]) ; }}
        for(dd=0;dd<sSrfInVolumes.GetSize();dd++)
          { SmObject * pOwner     = sSrfInVolumes[dd]->GetOwner() ;
            SmObject * pBaseOwner = sSrfInVolumes[dd]->GetSurface()->GetOwner() ; 
            SmBoolean  bOk = pBaseOwner == (sSrfInVolumes[dd]) && (pOwner->IsKindOf(SmFace_TYPE)) ;
            SM_ASSERT_MSG(bOk == TRUE, _T("SmSrfInVolume owner pointers not set up right")) ; 
          }

        if(m_pBrep) m_pBrep->Dump(SM_BD_GEOM_TYPES) ;
        SM_ASSERT_VALID(m_pBrep) ;

        smgfx_Erase() ;
        smgfx_SetLook(1,2, 0,0,1) ; if(m_pBrep) m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetLook(1,2, 0,1,1) ; this->Draw(FALSE, FALSE, TRUE) ;  sm_GraphicsLoop() ;
      
        sm_GraphicsLoop() ;
      }
#endif // SM_DEBUG_CODE
  } // end STEP 7: Bending InBendGeometry and transforming Before, In, After geometry

  // STEP 8: when asked replace SmSurfInVolume and SmCrvInVolume geometry with BSpline approximations
  {
    // when asked replace SmSurfInVolume and SmCrvInVolume geometry with BSpline approximations
    if(bDoApproximations)
      {
#ifdef SM_DEBUG_CODE                                                                                                   
        // PreApproximate target Brep AssertValid
        if(bDebugMe)
          { 
            if(m_pBrep) m_pBrep->Dump(_T("Brep BEFORE ApproximateWithBSplines - "),SM_BD_GEOM_TYPES) ;
            SM_ASSERT_VALID(m_pBrep) ;

            smgfx_Erase() ;
            smgfx_SetLook( 1, 2, 0, 0, 1 ); if(m_pBrep) { m_pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
            smgfx_SetLook(1,2, 0,1,1) ; this->Draw(FALSE, FALSE, TRUE) ;  sm_GraphicsLoop() ;
      
            smgfx_SetLook(4,8, 0,0,1) ; 
            for(di=0;di<sInVertices.GetSize();di++) 
            { 
                pVertex = sInVertices[di];
                if(pVertex) { pVertex->Draw();   sm_GraphicsLoop(); }
            }
            smgfx_SetLook(4,8, 0,0,1) ; 
            for(di=0;di<sInEdges.GetSize();di++)    
            { 
                pEdge = sInEdges[di];
                if(pEdge) { pEdge->Draw();      sm_GraphicsLoop(); }
            }
            smgfx_SetLook(1,2,0,0,.5) ; 
            for(di=0;di<sInFaces.GetSize();di++)    
            { 
                pFace = sInFaces[di];
                if(pFace) { pFace->DrawUV( 3, 3 ); sm_GraphicsLoop(); }
            }

            sm_GraphicsLoop() ;
          }
#endif // SM_DEBUG_CODE

        // when approximations are compiled
        //  (this happens when SM_EXACT_SPACEDEF_GEOMETRY compile constant is NOT defined) 
        //    - replace CrvInVolume curves with a BSplineCurve approximation
        //    - replace SrfInVolume surfaces with a BSplineSurface approximation
#ifndef SM_EXACT_SPACEDEF_GEOMETRY

        // Build the BSpline Surf and Curve approximations of the VolumeGeometry
        double dMaxAchievedTol ;
        m_pBrep->ApproximateWithBSplines(m_dApproxTol, dMaxAchievedTol, &sInFaces, &sInEdges) ; 
  
#endif // SM_EXACT_SPACEDEF_GEOMETRY

#ifdef SM_DEBUG_CODE                                                                                                   
        // PostApproximate target Brep AssertValid
        if(bDebugMe)
          {
            if(m_pBrep) m_pBrep->Dump(_T("Brep AFTER ApproximateWithBSplines - "),SM_BD_GEOM_TYPES) ;
            SM_ASSERT_VALID(m_pBrep) ;

            smgfx_Erase() ;
            smgfx_SetLook( 1, 2, 0, 0, 1 ); if(m_pBrep) { m_pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
            smgfx_SetLook(1,2, 0,1,1) ; this->Draw(FALSE, FALSE, TRUE) ;  sm_GraphicsLoop() ;
      
            smgfx_SetLook(4,8, 0,0,1) ; 
            for(di=0;di<sInVertices.GetSize();di++) 
            { 
                pVertex = sInVertices[di];
                if(pVertex) { pVertex->Draw();   sm_GraphicsLoop(); }
            }
            smgfx_SetLook(4,8, 0,0,1) ; 
            for(di=0;di<sInEdges.GetSize();di++)    
            { 
                pEdge = sInEdges[di];
                if(pEdge) { pEdge->Draw();      sm_GraphicsLoop(); }
            }
            smgfx_SetLook(1,2,0,0,.5) ; 
            for(di=0;di<sInFaces.GetSize();di++)    
            { 
                pFace = sInFaces[di];
                if(pFace) { pFace->DrawUV(); sm_GraphicsLoop(); }
            }

            sm_GraphicsLoop() ;
          }
#endif // SM_DEBUG_CODE
      } // end when asked to approximate geometry check
  } // end STEP 8: when asked replace SmSurfInVolume and SmCrvInVolume geometry with BSpline approximations 

#ifdef SM_DEBUG_CODE
  // draw 
  if(bDebugMe)
    {
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->Draw(FALSE, FALSE, TRUE) ;  sm_GraphicsLoop() ;

      smgfx_SetLook( 4, 8, 0, 0, 1 ); for(di = 0; di < sInVertices.GetSize(); di++)
      {
          pVertex = sInVertices[di];
          if(pVertex) { pVertex->Draw();   sm_GraphicsLoop(); }
      }
      smgfx_SetLook(4,8, 0,0,1) ; 
      for(di = 0; di < sInEdges.GetSize(); di++)
      {
          pEdge = sInEdges[di];
          if(pEdge) { pEdge->Draw();      sm_GraphicsLoop(); }
      }
      smgfx_SetLook(1,2,0,0,.5) ; 
      for(di = 0; di < sInFaces.GetSize(); di++)
      {
          pFace = sInFaces[di];
          if(pFace) { pFace->DrawUV(); sm_GraphicsLoop(); }
      }

      smgfx_SetLook(4,5, 0,1,0) ; for(di=0;di<sStartTopology.GetSize();di++) 
                                   { if(sStartTopology[di]->IsKindOf(SmVertex_TYPE)) { ((SmVertex *)sStartTopology[di])->Draw() ; sm_GraphicsLoop() ; }
                                     if(sStartTopology[di]->IsKindOf(SmEdge_TYPE))   { ((SmEdge *)  sStartTopology[di])->Draw() ; sm_GraphicsLoop() ; }
                                     if(sStartTopology[di]->IsKindOf(SmFace_TYPE))   { ((SmFace *)  sStartTopology[di])->DrawUV() ; sm_GraphicsLoop() ; }
                                   }

      smgfx_SetLook(4,5, 0,1,1) ; 
      for(di = 0; di < sBeforeVertices.GetSize(); di++)
      {
          pVertex = sBeforeVertices[di];
          if(pVertex) { pVertex->Draw(); sm_GraphicsLoop(); }
      }
      smgfx_SetLook( 4, 5, 0, 1, 1 ); for(di = 0; di < sBeforeEdges.GetSize(); di++)
      {
          pEdge = sBeforeEdges[di];
          if(pEdge) { pEdge->Draw();    sm_GraphicsLoop(); }
      }
      smgfx_SetLook(1,2, 0,1,1) ; 
      for(di = 0; di < sBeforeFaces.GetSize(); di++)
      {
          pFace = sBeforeFaces[di];
          if(pFace) { pFace->DrawUV();  sm_GraphicsLoop(); }
      }

      smgfx_SetLook(4,5, 1,0,0) ; for(di=0;di<sStopTopology.GetSize();di++) 
                                   { if(sStopTopology[di]->IsKindOf(SmVertex_TYPE)) { ((SmVertex *)sStopTopology[di])->Draw() ; sm_GraphicsLoop() ; }
                                     if(sStopTopology[di]->IsKindOf(SmEdge_TYPE))   { ((SmEdge *)  sStopTopology[di])->Draw() ; sm_GraphicsLoop() ; }
                                     if(sStopTopology[di]->IsKindOf(SmFace_TYPE))   { ((SmFace *)  sStopTopology[di])->DrawUV() ; sm_GraphicsLoop() ; }
                                   }

      smgfx_SetLook( 4, 5, 1, 0, 1 ); for(di = 0; di < sAfterVertices.GetSize(); di++)
      {
          pVertex = sAfterVertices[di];
          if(pVertex) { pVertex->Draw(); sm_GraphicsLoop(); }
      }
      smgfx_SetLook( 4, 5, 1, 0, 1 ); for(di = 0; di < sAfterEdges.GetSize(); di++)
      {
          pEdge = sAfterEdges[di];
          if(pEdge) { pEdge->Draw();    sm_GraphicsLoop(); }
      }
      smgfx_SetLook(1,2, 1,0,1) ; 
      for(di = 0; di < sAfterFaces.GetSize(); di++)
      {
          pFace = sAfterFaces[di];
          if(pFace) { pFace->DrawUV();  sm_GraphicsLoop(); }
      }

      smgfx_SetLook(1,2, 0,1,1) ; this->Draw(FALSE, FALSE, TRUE) ;  sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;

      SM_DUMP_AND_ASSERT_VALID(this) ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(m_pBrep) { m_pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ; 

} // end SmSpaceBend::DoSpaceBend

/*******************************************************************//**
PURPOSE: Build a Brep marking the preBend region

NOTES:  The BendDomain region is a rectilinear box defined with 
          origin = BendAxisPoint
          length = interval along the Bend XAxis = [InsideRadius,          OutsideRadius]
          width  = interval along the Bend YAxis = [-BendWidth/2.0,       +BendWidth/2.0]
          height = interval along the Bend ZAxis = [BendInterval.GetMin(), BendInterval.GetMax()]
***********************************************************************/
SmBrep * SmSpaceBend::GetBendDomain() const
{
  // low work - PreBendRegion already built
  if(m_pBendDomain != NULL)
    { return( m_pBendDomain ) ; }

  // an increase amount to avoid coincident geometry problems
  double dDelta   = 0.1 ;

  // Get PreBendRegion rectilinear position and orientation
  SmVector3d       sX    = GetBendDirection() ;   // length, to be increased by dDelta to avoid coincident geoemtry
  SmVector3d       sY    = GetBendBinormal() ;    // width,  left alone to properly place the Bend/RigidRotation boundary
  SmVector3d       sZ    = GetBendAxis() ;        // height, to be increased by dDelta to avoid coincident geoemtry
  SmPoint3d        sOrig = GetPreBendBeginCorner() - dDelta * sZ    // increased by dDelta to avoid coincident geometry
                                                   - dDelta * sX ;  // increased by dDelta to avoid coincident geometry
  SmAxis2Placement sRefFrame(sOrig, sX, sY) ; 

  // Get PreBendRegion sizes
  double dLength = m_dSheetThickness ; 
  double dWidth  = GetPreBendWidth() ; 
  double dHeight = GetBendSegmentLength() ;

  // Build empty PreBend Breps (building internal caches is not C++ const - but is for SMLib semantics)
  SmSpaceBend * pThisSpaceBend = (SmSpaceBend*) this;
  pThisSpaceBend->m_pBendDomain        = new (*GetContext()) SmBrep() ;
  pThisSpaceBend->m_pStartBendBoundary = new (*GetContext()) SmBrep() ;
  pThisSpaceBend->m_pStopBendBoundary  = new (*GetContext()) SmBrep() ;
  
  // define SmPrimitiveCreation object
  SmPrimitiveCreation sPreBendPC(m_pBendDomain->GetInfiniteRegion()) ; 
      
  // insert a Box into the PreBend infinite region  
  if(SM_SUCCESS != sPreBendPC.CreateBox(dLength + 2 * dDelta,   // in : X axis extent, increased by dDelta to avoid coincident geometry
                                        dWidth,                 // in : Y axis extent, left alone to properly place the Unbend/RigidRotation boundary
                                        dHeight + 2 * dDelta,   // in : Z axis extent, increased by dDelta to avoid coincident geometry
                                        sRefFrame))             // in : defines box min corner and orientation
    { return(NULL) ; }

 // select before and after faces of the 
 SmTArray<SmFace*> sFaces ;
 m_pBendDomain->GetFaces(sFaces) ;
 SmFace *pStartFace = sFaces[4] ;  // gwc - These indices were found by trial and error - not geometrically.
 SmFace *pStopFace  = sFaces[5] ;  // they will depend on the behavior of CreateBox to remain consistent.
                                   // that's not a good coding practice, but is good enough for now.
                                   // gwc: could replace this hack with a search based on surface type and surface normal orientation

 // build the begin boundary sheet model
 SmTArray<SmFace *> sStartFaces(1,NULL,1) ; sStartFaces[0] = pStartFace ; 
 SmTArray<SmFace *> sStopFaces(1,NULL,1) ;  sStopFaces[0]  = pStopFace ; 
 m_pBendDomain->CopyFaces(sStartFaces, m_pStartBendBoundary, NULL, FALSE, NULL) ;
 m_pBendDomain->CopyFaces(sStopFaces,  m_pStopBendBoundary, NULL, FALSE, NULL) ;
 
  // all done
  return(m_pBendDomain) ; 

} // end SmSpaceBend::GetBendDomain

/*******************************************************************//**
PURPOSE: Build and return the SmAxis2Placement object representing
         the rigid body transformation applied to the BeforeBend geometry.

NOTES:  The bend is defined as the transformation from 
        the point on the BendNeutral Axis at the BendRegion StartPlane
        to the point on the beginning of the BendNeutral after bend arc.
***********************************************************************/
SmAxis2Placement SmSpaceBend::GetBeforeBendTransform
 (SmBendOrientTYPE eOptOrientType)  // in : optional OrientType specification, default:[SM_BO_ORIENT_UNKNOWN=Usem_eBendOrient val]
 const
{ 
  // UNKNOWN_BENDING -> treat as SM_BO_CENTER_BENDING
  SmBendOrientTYPE eBendOrient =  eOptOrientType != SM_BO_ORIENT_UNKNOWN ? eOptOrientType 
                                 : m_eBendOrient != SM_BO_ORIENT_UNKNOWN ? m_eBendOrient
                                 :                                         SM_BO_CENTER_BENDING ;

  // low work - FIXED_START_BENDING = BeforeBend geometry does not rotate
  if(eBendOrient == SM_BO_FIXED_BEFORE_BENDING)
    {
      SmAxis2Placement sBeforeBendTransform ;  // identity transform
      return(sBeforeBendTransform) ;
    }

  // arrive here for CENTER_BENDING and FIXED_END_BENDING

  // define From Point, XAxis, and YAxis - point on neutral axis at the StartBend plane
  // double     dBendStartDist = GetPreBendWidth() ;
  SmVector3d sFromXAxis     = GetBendDirection() ;
  SmVector3d sFromYAxis     = GetBendAxis() ;
  SmVector3d sFromOrigin    = GetBendAxisPoint() + GetNeutralRadius()    * GetBendDirection() 
                                                 - GetPreBendWidth()/2.0 * GetBendBinormal() ;

  // define To Point, XAxis, and YAxis - to point on neutral axis at StartBend plane's final pos and orientation
  SmVector3d sToXAxis, sToYAxis, sToOrigin ;

  // CENTER_BENDING = BeforeBend geometry rotates by half of minus bend angle about bend axis
  if(eBendOrient == SM_BO_CENTER_BENDING)
    {
      // define To Point, XAxis and YAxis - point moved to start Arc position
      double dRotRad = -SM_DEG2RAD(m_dBendAngDeg/2.0) ;       // note: half angle
      sToXAxis       =   GetBendDirection() * smos_Cosine(dRotRad)  
                       + GetBendBinormal()  * smos_Sine(dRotRad) ;
      sToYAxis       = GetBendAxis() ;
      sToOrigin      = GetBendAxisPoint() + GetNeutralRadius() * sToXAxis ;
    }

  // FIXED_END_BENDING = BeforeBend geometry rotates by minus bend angle about point offset from bend axis by 1/2 PreBendWidth
  else if(eBendOrient == SM_BO_FIXED_AFTER_BENDING)
    {
      // define To Point, XAxis and YAxis - point moved to start Arc position
      double dRotRad = -SM_DEG2RAD(m_dBendAngDeg) ;  // note: full angle
      sToXAxis       =   GetBendDirection() * smos_Cosine(dRotRad)  
                       + GetBendBinormal()  * smos_Sine(dRotRad) ;
      sToYAxis       =   GetBendAxis() ;
      sToOrigin      =   GetBendAxisPoint() 
                       + GetPreBendWidth()/2.0 * GetBendBinormal()
                       + GetNeutralRadius() * sToXAxis ;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      smgfx_Erase() ;
      smgfx_SetLook(4,5, 0,.5,1) ; sFromXAxis.Draw(&sFromOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 0, 1,1) ; sFromYAxis.Draw(&sFromOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 0, 0,1) ; sFromOrigin.Draw() ; sm_GraphicsLoop() ;
                    
      smgfx_SetLook(4,5, 1, 0,.5) ; sToXAxis.Draw(&sToOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1, 0, 1) ; sToYAxis.Draw(&sToOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1, 0, 0) ; sToOrigin.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // build return object
  SmAxis2Placement sBeforeBendTransform( sFromOrigin, sFromXAxis, sFromYAxis,
                                         sToOrigin,   sToXAxis,   sToYAxis) ;

  // all done
  return( sBeforeBendTransform ) ;

} // end SmSpaceBend::GetBeforeBendTransform

/*******************************************************************//**
PURPOSE: Build and return the SmAxis2Placement object representing
         the rigid body transformation applied to the InsideBend geometry
         After its been bent.

NOTES:  The bend is defined as the transformation from 
   SM_BO_FIXED_BEFORE_BENDING = the point on the BendNeutral Axis at the BendRegion StartPlane after its been bent
   SM_BO_CENTER_BENDING       = the point on the BendNeutral Axis at the BendRegion CenterPlane
   SM_BO_FIXED_AFTER_BENDING  = the point on the BendNeutral Axis at the BendRegion EndPlane after its been bent 
        to its position after bending.
***********************************************************************/
SmAxis2Placement SmSpaceBend::GetInsideBendTransform
 (SmBendOrientTYPE eOptOrientType)  // in : optional OrientType specification, default:[SM_BO_ORIENT_UNKNOWN=Usem_eBendOrient val]
 const
{ 
  // UNKNOWN_BENDING -> treat as SM_BO_CENTER_BENDING
  SmBendOrientTYPE eBendOrient =  eOptOrientType != SM_BO_ORIENT_UNKNOWN ? eOptOrientType 
                                 : m_eBendOrient != SM_BO_ORIENT_UNKNOWN ? m_eBendOrient
                                 :                                         SM_BO_CENTER_BENDING ;

  // low work - Center_BENDING = InsideBend geometry does not rotate
  if(eBendOrient == SM_BO_CENTER_BENDING)
    {
      SmAxis2Placement sBeforeBendTransform ;  // identity transform
      return(sBeforeBendTransform) ;
    }

  // arrive here for FIXED_START_BENDING and FIXED_END_BENDING
  SmVector3d sFromXAxis, sFromYAxis, sFromOrigin ;
  SmVector3d sToXAxis,   sToYAxis,   sToOrigin ;

  // FIXED_BEFORE_BENDING = InsideBend geometry rotates by bend angle about point offset from bend axis by 1/2 PreBendWidth
  if(eBendOrient == SM_BO_FIXED_BEFORE_BENDING)
    {
      // define to Point, XAxis, and YAxis - point on neutral axis at the StartBend plane
      // double dBendStartDist = GetPreBendWidth() ;
      sToXAxis              = GetBendDirection() ;
      sToYAxis              = GetBendAxis() ;
      sToOrigin             = GetBendAxisPoint() + GetNeutralRadius()    * GetBendDirection() 
                                                 - GetPreBendWidth()/2.0 * GetBendBinormal() ;

      // define From Point, XAxis and YAxis - point moved to start Arc position
      double dRotRad = -SM_DEG2RAD(m_dBendAngDeg/2.0) ;       // note: half angle
      sFromXAxis     =   GetBendDirection() * smos_Cosine(dRotRad)  
                       + GetBendBinormal()  * smos_Sine(dRotRad) ;
      sFromYAxis     =   GetBendAxis() ;
      sFromOrigin    =   GetBendAxisPoint() 
                       + GetNeutralRadius() * sFromXAxis ;
    }

  // FIXED_AFTER_BENDING = BeforeBend geometry rotates by minus bend angle about point offset from bend axis by 1/2 PreBendWidth
  else if(eBendOrient == SM_BO_FIXED_AFTER_BENDING)
    {
      // define To Point, XAxis, and YAxis - point on neutral axis at the StartBend plane
      // double dBendStartDist = GetPreBendWidth() ;
      sToXAxis              = GetBendDirection() ;
      sToYAxis              = GetBendAxis() ;
      sToOrigin             = GetBendAxisPoint() + GetNeutralRadius()    * GetBendDirection() 
                                                 + GetPreBendWidth()/2.0 * GetBendBinormal() ;

      // define To Point, XAxis and YAxis - point moved to start Arc position
      double dRotRad =  SM_DEG2RAD(m_dBendAngDeg/2.0) ;  // note: half angle
      sFromXAxis     =   GetBendDirection() * smos_Cosine(dRotRad)  
                       + GetBendBinormal()  * smos_Sine(dRotRad) ;
      sFromYAxis     =   GetBendAxis() ;
      sFromOrigin    =   GetBendAxisPoint() 
                       + GetNeutralRadius() * sFromXAxis ;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      smgfx_Erase() ;
      smgfx_SetLook(4,5, 0,.5,1) ; sFromXAxis.Draw(&sFromOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 0, 1,1) ; sFromYAxis.Draw(&sFromOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 0, 0,1) ; sFromOrigin.Draw() ; sm_GraphicsLoop() ;
                    
      smgfx_SetLook(4,5, 1, 0,.5) ; sToXAxis.Draw(&sToOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1, 0, 1) ; sToYAxis.Draw(&sToOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1, 0, 0) ; sToOrigin.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // build return object
  SmAxis2Placement sInsideBendTransform( sFromOrigin, sFromXAxis, sFromYAxis,
                                         sToOrigin,   sToXAxis,   sToYAxis) ;

  // all done
  return( sInsideBendTransform ) ;

} // end SmSpaceBend::GetInsideBendTransform

/*******************************************************************//**
PURPOSE: Build and return the SmAxis2Placement object representing
         the rigid body transformation applied to the AfterBend geometry.

NOTES:  The bend is defined as the transformation from 
        the point on the BendNeutral Axis at the BendRegion StopPlane
        to the point on the end of the BendNeutral after bend arc.
***********************************************************************/
SmAxis2Placement SmSpaceBend::GetAfterBendTransform
 (SmBendOrientTYPE eOptOrientType)  // in : optional OrientType specification, default:[SM_BO_ORIENT_UNKNOWN=Usem_eBendOrient val]
 const
{ 
  // UNKNOWN_BENDING -> treat as SM_BO_CENTER_BENDING
  SmBendOrientTYPE eBendOrient =  eOptOrientType != SM_BO_ORIENT_UNKNOWN ? eOptOrientType 
                                 : m_eBendOrient != SM_BO_ORIENT_UNKNOWN ? m_eBendOrient
                                 :                                         SM_BO_CENTER_BENDING ;

  // low work - FIXED_START_BENDING = BeforeBend geometry does not rotate
  if(eBendOrient == SM_BO_FIXED_AFTER_BENDING)
    {
      SmAxis2Placement sBeforeBendTransform ;  // idnentity transform
      return(sBeforeBendTransform) ;
    }

  // define From Point, XAxis, and YAxis - point on neutral axis at the StopBend plane
  // double     dBendStartDist = GetPreBendWidth() ;
  SmVector3d sFromXAxis     = GetBendDirection() ;
  SmVector3d sFromYAxis     = GetBendAxis() ;
  SmVector3d sFromOrigin    = GetBendAxisPoint() + GetNeutralRadius()    * GetBendDirection() 
                                                 + GetPreBendWidth()/2.0 * GetBendBinormal() ;

  // define To Point, XAxis, and YAxis - to point on neutral axis at StartBend plane's final pos and orientation
  SmVector3d sToXAxis, sToYAxis, sToOrigin ;

  // CENTER_BENDING = BeforeBend geometry rotates by half of bend angle about bend axis
  if(eBendOrient == SM_BO_CENTER_BENDING)
    {
      // define To Point, XAxis and YAxis - point moved to start Arc position
      double dRotRad =   SM_DEG2RAD(m_dBendAngDeg/2.0) ;       // note: half angle
      sToXAxis       =   GetBendDirection() * smos_Cosine(dRotRad)  
                       + GetBendBinormal()  * smos_Sine(dRotRad) ;
      sToYAxis       = GetBendAxis() ;
      sToOrigin      = GetBendAxisPoint() + GetNeutralRadius() * sToXAxis ;
    }

  // FIXED_START_BENDING = AfterBend geometry rotates by bend angle about point offset from bend axis by -1/2 PreBendWidth
  else if(eBendOrient == SM_BO_FIXED_BEFORE_BENDING)
    {
      // define To Point, XAxis and YAxis - point moved to start Arc position
      double dRotRad =   SM_DEG2RAD(m_dBendAngDeg) ;  // note: full angle
      sToXAxis       =   GetBendDirection() * smos_Cosine(dRotRad)  
                       + GetBendBinormal()  * smos_Sine(dRotRad) ;
      sToYAxis       =   GetBendAxis() ;
      sToOrigin      =   GetBendAxisPoint() 
                       - GetPreBendWidth()/2.0 * GetBendBinormal()
                       + GetNeutralRadius() * sToXAxis ;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      smgfx_Erase() ;
      smgfx_SetLook(4,5, 0,.5,1) ; sFromXAxis.Draw(&sFromOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 0, 1,1) ; sFromYAxis.Draw(&sFromOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 0, 0,1) ; sFromOrigin.Draw() ; sm_GraphicsLoop() ;
                    
      smgfx_SetLook(4,5, 1, 0,.5) ; sToXAxis.Draw(&sToOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1, 0, 1) ; sToYAxis.Draw(&sToOrigin) ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1, 0, 0) ; sToOrigin.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // build return object
  SmAxis2Placement sAfterBendTransform( sFromOrigin, sFromXAxis, sFromYAxis,
                                         sToOrigin,   sToXAxis,   sToYAxis) ;

  // all done
  return( sAfterBendTransform ) ;

} // end SmSpaceBend::GetAfterBendTransform

/*******************************************************************//**
PURPOSE: Internal unit-test of the Bend Transforms

RETURNS: TRUE  = okay
         FALSE = problems

NOTES: Verify that the transforms are G1 continuous across the
       Before/Inside and Inside/After boundaries
***********************************************************************/
SmBoolean SmSpaceBend::TestBendTransforms() const
{
  // locals
  ULONG ii, jj ;

  // return
  SmBoolean bRtn = TRUE ;                          // TRUE  = okay
                                                   // FALSE = a non C1 point was found
  double    dThisGap = 0.0,       dMaxGap       = 0.0 ;  // max distance between projected point pairs
  double    dThisAngRad = 0.0,    dMaxAngRad    = 0.0 ;  // max angRad between projected vector pairs
  double    dThisVecMagGap = 0.0, dMaxVecMagGap = 0.0 ;  // max VecMag differences between projected vector pairs

  // SpaceBend geometry and PostBend transform objects
  SmBendVolume   * pBendVolume        = CreateBendVolume(*GetContext()) ;
  SmBrep         * pStartBendBoundary = GetStartBendBoundary() ;
  SmBrep         * pStopBendBoundary  = GetStopBendBoundary() ;
  SmAxis2Placement sBeforeTransform ;
  SmAxis2Placement sInsideTransform ;
  SmAxis2Placement sAfterTransform ;

  // BendBoundaries Vertices
  SmTArray<SmFace *>   sStartBoundaryFaces, sStopBoundaryFaces ;
  SmTArray<SmVertex *> sStartVertices, sStopVertices ;
  pStartBendBoundary->GetFaces(sStartBoundaryFaces) ;   SM_ASSERT_MSG(sStartBoundaryFaces.GetSize() == 1, _T("SmSpaceBend::TestBendTransforms - got an unexpected number of Boundary Faces")) ;
  pStopBendBoundary->GetFaces(sStopBoundaryFaces) ;     SM_ASSERT_MSG(sStopBoundaryFaces.GetSize() == 1, _T("SmSpaceBend::TestBendTransforms - got an unexpected number of Boundary Faces")) ;
  sStartBoundaryFaces[0]->GetVertices(sStartVertices) ; SM_ASSERT_MSG(sStartVertices.GetSize() == 4, _T("SmSpaceBend::TestBendTransforms - got an unexpected number of Boundary Vertices")) ;
  sStopBoundaryFaces[0]->GetVertices(sStopVertices) ;   SM_ASSERT_MSG(sStopVertices.GetSize() == 4, _T("SmSpaceBend::TestBendTransforms - got an unexpected number of Boundary Vertices")) ;

  // Sample points on the space bend boundaries used for PostTransform continuity checks
  SmPoint3d  sBeforePrePoint,  sBeforePostPoint,  sBeforePreDir,  sBeforePostDir ;
  SmPoint3d  sInsidePrePoint0, sInsidePostPoint0, sInsidePreDir0, sInsidePostDir0 ;
  SmPoint3d  sInsidePrePoint1, sInsidePostPoint1, sInsidePreDir1, sInsidePostDir1 ;
  SmPoint3d  sAfterPrePoint,   sAfterPostPoint,   sAfterPreDir,   sAfterPostDir ;
  SmVector3d sPD[2] ;  // output array for MapDirectionalDerivs() calls

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw SpaceBend->Brep, Start/Stop BendBoundaries, BendDomain, Boundary Vertices
  if(bDebugMe)
    {
      ULONG di ; 

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(GetBrep()) GetBrep()->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; if(pStartBendBoundary) pStartBendBoundary->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; if(pStopBendBoundary) pStopBendBoundary->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,0,0) ; if(GetBendDomain()) GetBendDomain()->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,1,0) ; for(di=0;di<sStartVertices.GetSize();di++) sStartVertices[di]->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0) ; for(di=0;di<sStopVertices.GetSize();di++) sStopVertices[di]->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // note: SmSpaceBends partition space into 3 domains divided by the
  //       StartBendBoundary() and the StopBendBoundary() Brep Shapes.
  //       Any points on those boundaries should project with C1 continuity 
  //       through the tansformations on either side of the boundary
  //       (an appropriate pair of BendVolume and SmAxis2Placement mappings) 
  
  // for the 3 possible SpaceBend orient cases
  for(ii=0;ii<3;ii++)
    {  
      SmBendOrientTYPE eOrientType =   (ii == 0) ? SM_BO_FIXED_BEFORE_BENDING 
                                     : (ii == 1) ? SM_BO_CENTER_BENDING      
                                                 : SM_BO_FIXED_AFTER_BENDING ;

      // PostBend Transforms for this OrientType
      sBeforeTransform = GetBeforeBendTransform(eOrientType) ;
      sInsideTransform = GetInsideBendTransform(eOrientType) ;
      sAfterTransform  = GetAfterBendTransform(eOrientType) ;
                                                 
      // for every Start/Stop BoundaryVertex  (number of StartVertices == number of Stop Vertices)
      for(jj=0;jj<sStartVertices.GetSize();jj++)
        {
          // startBoundaryPoint to project once through sBeforeTransform and once through VolumeMap/sInsideTransform
          sBeforePrePoint  = sStartVertices[jj]->GetPoint() ;  
          sInsidePrePoint0 = sStartVertices[jj]->GetPoint() ; 

          sBeforePreDir  = GetBendBinormal() ;
          sInsidePreDir0 = GetBendBinormal() ;

          // startBoundaryPoint to project once through sAfterTransform and once through VolumeMap/sInsideTransform
          sInsidePrePoint1 = sStopVertices[jj]->GetPoint() ;  
          sAfterPrePoint   = sStopVertices[jj]->GetPoint() ;   

          sInsidePreDir1 = GetBendBinormal() ;
          sAfterPreDir   = GetBendBinormal() ;

          // Map Before Inside point and dir from Bend InSpace to OutSpace ;
            // pBendVolume->MapPoint (sInsidePrePoint0, sInsidePostPoint0) ;
            // pBendVolume->MapVector(sInsidePrePoint0, sInsidePreDir0, sInsidePostDir0) ;
          pBendVolume->MapDirectionalDerivs(sInsidePrePoint0, sInsidePreDir0, 1, sPD) ; 
          sInsidePostPoint0 = sPD[0] ;
          sInsidePostDir0   = sPD[1] ;   

          // Map After Inside point and dir from Bend InSpace to OutSpace ;
            // pBendVolume->MapPoint(sInsidePrePoint1, sInsidePostPoint1) ;
            // pBendVolume->MapVector(sInsidePrePoint1, sInsidePreDir1, sInsidePostDir1) ;   
          pBendVolume->MapDirectionalDerivs(sInsidePrePoint1, sInsidePreDir1, 1, sPD) ; 
          sInsidePostPoint1 = sPD[0] ;
          sInsidePostDir1   = sPD[1] ;   

#ifdef SM_DEBUG_CODE
          // draw SpaceBend->Brep, Start/Stop BendBoundaries, BendDomain, Boundary Vertices
          if(bDebugMe)
            {
              ULONG di ; 

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(GetBrep()) GetBrep()->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ; if(pStartBendBoundary) pStartBendBoundary->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 1,0,0) ; if(pStopBendBoundary) pStopBendBoundary->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,0,0) ; if(GetBendDomain()) GetBendDomain()->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 0,1,0) ; for(di=0;di<sStartVertices.GetSize();di++) sStartVertices[di]->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 1,0,0) ; for(di=0;di<sStopVertices.GetSize();di++) sStopVertices[di]->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,8,  0,0,1) ; sInsidePrePoint0.Draw() ; sInsidePreDir0.Draw(&sInsidePrePoint0) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,10, 0,1,1) ; sInsidePostPoint0.Draw() ; sInsidePostDir0.Draw(&sInsidePostPoint0) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,8,  1,0,0) ; sInsidePrePoint1.Draw() ; sInsidePreDir1.Draw(&sInsidePrePoint1) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,10, 1,0,1) ; sInsidePostPoint1.Draw() ; sInsidePostDir1.Draw(&sInsidePostPoint1) ; sm_GraphicsLoop() ;

              smgfx_SetLook(2,12, 0,0,1) ; sBeforePrePoint.Draw() ; sBeforePreDir.Draw(&sBeforePrePoint) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,12, 1,0,0) ; sAfterPrePoint.Draw() ; sAfterPreDir.Draw(&sAfterPrePoint) ; sm_GraphicsLoop() ;

              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

          // next: apply PostBend transformations

          // BeforeTransform applied directly to beforeBoundary point
          sBeforeTransform.TransformPoint (sBeforePrePoint, sBeforePostPoint) ; 
          sBeforeTransform.TransformVector(sBeforePreDir,   sBeforePostDir) ; 

          // InsideTransform applied to BeforeBoundary point after BendVolume transform
          sInsideTransform.TransformPoint (sInsidePostPoint0, sInsidePostPoint0) ; 
          sInsideTransform.TransformVector(sInsidePostDir0,   sInsidePostDir0) ;  
          
          // InsideTransform applied to AfterBoundary point after BendVolume transform
          sInsideTransform.TransformPoint (sInsidePostPoint1, sInsidePostPoint1) ; 
          sInsideTransform.TransformVector(sInsidePostDir1,   sInsidePostDir1) ;  

          // AfterTransform applied directly to AfterBoundary point
          sAfterTransform.TransformPoint (sAfterPrePoint, sAfterPostPoint) ; 
          sAfterTransform.TransformVector(sAfterPreDir,   sAfterPostDir) ; 

#ifdef SM_DEBUG_CODE
          // draw SpaceBend->Brep, Start/Stop BendBoundaries, BendDomain, Boundary Vertices
          if(bDebugMe)
            {
              ULONG di ; 

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(GetBrep()) GetBrep()->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ; if(pStartBendBoundary) pStartBendBoundary->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 1,0,0) ; if(pStopBendBoundary) pStopBendBoundary->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,0,0) ; if(GetBendDomain()) GetBendDomain()->Draw(TRUE) ; sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 0,1,0) ; for(di=0;di<sStartVertices.GetSize();di++) sStartVertices[di]->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 1,0,0) ; for(di=0;di<sStopVertices.GetSize();di++) sStopVertices[di]->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,8,  0,0,1) ; sInsidePrePoint0.Draw() ; sInsidePreDir0.Draw(&sInsidePrePoint0) ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,10, 0,1,1) ; sInsidePostPoint0.Draw() ; sInsidePostDir0.Draw(&sInsidePostPoint0) ; sm_GraphicsLoop() ;
              smgfx_SetLook(2,8,  1,0,0) ; sInsidePrePoint1.Draw() ; sInsidePreDir1.Draw(&sInsidePrePoint1) ; sm_GraphicsLoop() ;
              smgfx_SetLook(4,10, 1,0,1) ; sInsidePostPoint1.Draw() ; sInsidePostDir1.Draw(&sInsidePostPoint1) ; sm_GraphicsLoop() ;

              smgfx_SetLook(3,12, 0,0,1) ; sBeforePrePoint.Draw() ; sBeforePreDir.Draw(&sBeforePrePoint) ; sm_GraphicsLoop() ;
              smgfx_SetLook(5,14, 0,1,1) ; sBeforePostPoint.Draw() ; sBeforePostDir.Draw(&sBeforePostPoint) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,12, 1,0,0) ; sAfterPrePoint.Draw() ; sAfterPreDir.Draw(&sAfterPrePoint) ; sm_GraphicsLoop() ;
              smgfx_SetLook(5,14, 1,0,1) ; sAfterPostPoint.Draw() ; sAfterPostDir.Draw(&sAfterPostPoint) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop() ;
            }
#endif // SM_DEBUG_CODE

          // check for C0 continuity
          dThisGap = sBeforePostPoint.DistanceBetween(sInsidePostPoint0) ;
          if(dMaxGap < dThisGap) { dMaxGap = dThisGap ; }

          // check for G1 continuity
          sBeforePostDir.AngleBetween(sInsidePostDir0, dThisAngRad) ;
          if(dMaxAngRad    < dThisAngRad   ) { dMaxAngRad    = dThisAngRad    ; }

          // gather C1 continuity data - transform is only G1 expect differences in VecMagGap
          dThisVecMagGap = smos_Fabs(sBeforePostDir.Length() - sInsidePostDir0.Length()) ; 
          if(dMaxVecMagGap < dThisVecMagGap) { dMaxVecMagGap = dThisVecMagGap ; }

        } // end iter every Start/StopVertices checking mapping continuity
    } // end iter ii, all possible SpaceBend orient cases
        
  // arrive here after computing dMaxGap, dMaxAngRad, dMaxVecMagGap for all sample points

  // make C0 and C1 continuity checks
  bRtn &= dMaxGap       < SM_EFF_ZERO ;       // C0 continuity
  bRtn &= dMaxAngRad    < SM_EFF_ZERO_RAD ;   // G1 continuity
  // SmBoolean bC1Continuity = dMaxVecMagGap < SM_EFF_ZERO ;  // C1 continuity - This should be false (it's a G1 Continuity transform)

  // all done
  return(bRtn) ;

} // end SmSpaceBend::TestBendTransforms

/*******************************************************************//**
PURPOSE: rigid body rotate and translate topology within a single Brep.

RETURNS: SM_SUCCESS when all given topology is transformed.
         SM_ERROR when any curve or surface Transform call fails. In which
                  case, the BREP is left in a mangled state - the
                  given topology is an unknown mix of 
                  transformed and untransformed objects.

NOTES: If subsets of the Brep's faces, edges, and vertices are
       transformed rather than all the Brep's topology, then
       after this call the Brep may no longer be valid in that
       the original topology connections between objects will be preserved
       but the new geometric relationships between objects may not be connected.

       It's up to the caller to manage moving the bits and pieces
       of a Brep until all the pieces that are connected topologically 
       are connected geometrically once again.

       So, don't run AssertValid() on the modified Brep until it's all put
       back together again.
***********************************************************************/
SmStatus SmSpaceBend::TransformTopology
  (const SmAxis2Placement   & crRotateNMove, // in : Rigid body transform to apply
         SmBrep             & rBrep,         // in : Brep containing all topology to be transformed
   const SmTArray<SmFace*>  & crFaces,       // in : faces to transform
   const SmTArray<SmEdge*>  & crEdges,       // in : edges to transform 
   const SmTArray<SmVertex*>& crVertices)    // in : vertices to transform 
{
  // notify Brep of editing operation
  rBrep.Notify(SM_NO_PRE_EDIT, this, NULL, NULL);  

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
#endif // SM_DEBUG_CODE

  // We can transform the topology of a brep by transforming its 3D geometry
  //  (points on vertices, curves on edges, and surfaces on faces.

  // locals
  ULONG ii ;
  SmTArray<SmSurface*> sSurfaces ;
// Remove Composites
// SmTArray<SmCurve*>   sCurves ;

  // vertices
  for(ii=0;ii<crVertices.GetSize();ii++) 
    {
      SmVertex * pVertex  = crVertices[ii] ;
      SmPoint3d sPrePoint = pVertex->GetPoint() ;
      SmPoint3d sPostPoint ;

      crRotateNMove.TransformPoint(sPrePoint, sPostPoint) ;

#ifdef SM_DEBUG_CODE
      // draw transformed vertex 
      if(bDebugMe)
        {
          smgfx_SetLook(2,3, 1,.5,0) ; sPostPoint.Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      pVertex->Notify(SM_NO_PRE_EDIT, pVertex, this, NULL) ;
      pVertex->SetPoint(sPostPoint) ;

    } // end iter ii, trqansforming every vertex->Point

// Remove Composites - added new block to replace old Composite sensitive block
  // edges
  for(ii=0;ii<crEdges.GetSize();ii++)
    {
      SmEdge  * pEdge  = crEdges[ii] ;
      SmCurve * pCurve = pEdge->GetCurve() ;
 
      pEdge->Notify(SM_NO_PRE_EDIT, pEdge, this, NULL) ;
      SER( pCurve->Transform(crRotateNMove) ) ;
 
#ifdef SM_DEBUG_CODE
      // draw transformed curve 
      if(bDebugMe)
        {
          smgfx_SetLook(2,3, 1,.5,0) ; pEdge->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE
    } // end iterii, transforming every Edge->Curve
 
// Remove Composites
//  // watch out for CEdges - don't apply tranformation to one curve multiple times
//  for(ii=0;ii<crEdges.GetSize();ii++)
//    {
//      SmEdge  * pEdge  = crEdges[ii] ;
//      SmCurve * pCurve = pEdge->GetCurve() ;
//
//      if(pEdge->IsKindOf(SmCEdge_TYPE)) { sCurves.AddUnique(pCurve) ; }
//      else                              { sCurves.Add(pCurve) ; }
//    }
//
//  // for every curve - transform it
//  for(ii=0;ii<sCurves.GetSize();ii++) 
//    {
//      SmCurve * pCurve = sCurves[ii] ; NER(pCurve) ;
//      SmEdge  * pEdge = (SmEdge*)pCurve->GetEdge() ; // Is either edge or cedge
//
//      pEdge->Notify(SM_NO_PRE_EDIT, pEdge, this, NULL) ;
//      SER( pCurve->Transform(crRotateNMove) ) ;
//
//#ifdef SM_DEBUG_CODE
//      // draw transformed curve 
//      if(bDebugMe)
//        {
//          smgfx_SetLook(2,3, 1,.5,0) ; pEdge->Draw() ; sm_GraphicsLoop() ;
//          sm_GraphicsLoop() ;
//        }
//#endif // SM_DEBUG_CODE
//    } // end iter every curve

  // surfaces

  // save face->RectangularTrim states
  SmTArray<SmBoolean> sRectTrimState(crFaces.GetSize(), NULL, crFaces.GetSize()) ;
  for(ii=0;ii<crFaces.GetSize();ii++) 
    { sRectTrimState[ii] = crFaces[ii]->GetRectangularTrim() ; }

// Remove Composites - added new block to replace old Composite sensitive block
  // faces
  for(ii=0;ii<crFaces.GetSize();ii++)
    {
      SmFace    * pFace    = crFaces[ii] ;
      SmSurface * pSurface = pFace->GetSurface() ;

      // call Notify (which clears the m_bRectangularTrim bit)
      pFace->Notify(SM_NO_PRE_EDIT, pFace, this, NULL);
      SER( pSurface->Transform(crRotateNMove) ) ;

#ifdef SM_DEBUG_CODE
      // draw transformed surface 
      if(bDebugMe)
        {
          // GWC: note - brep is in intermediate invalid state until all geometry is updated.
          //      now some edges and vertices in the bend have not yet been moved and won't
          //      lie on their face->Surfaces which have been moved. Drawing the face
          //      will generate error messages, drawing the surface won't
          smgfx_SetLook(1,2, 1,.5,0) ; pSurface->DrawUV(3,3) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE
    } // end iter ii, transforming every face->surface

// Remove Composites
//   // watch out for composite faces - don't apply transformation to one surface multiple times
//   for(ii=0;ii<crFaces.GetSize();ii++)
//     {
//       SmFace    * pFace    = crFaces[ii] ;
//       SmSurface * pSurface = pFace->GetSurface() ;
// 
//       if(pFace->IsKindOf(SmCFace_TYPE)) { sSurfaces.AddUnique(pSurface) ; }
//       else                              { sSurfaces.Add(pSurface) ; }
//     }
// 
//   // for every surface - transform it
//   for(ii=0;ii<sSurfaces.GetSize();ii++) 
//     {
//       SmSurface * pSurface = sSurfaces[ii] ; NER(pSurface);
//       SmFace    * pFace    = (SmFace*)pSurface->GetFace();
// 
//       // call Notify (which clears the m_bRectangularTrim bit)
//       pFace->Notify(SM_NO_PRE_EDIT, pFace, this, NULL);
//       SER( pSurface->Transform(crRotateNMove) ) ;
// 
// #ifdef SM_DEBUG_CODE
//       // draw transformed surface 
//       if(bDebugMe)
//         {
//           // GWC: note - brep is in intermediate invalid state until all geometry is updated.
//           //      now some edges and vertices in the bend have not yet been moved and won't
//           //      lie on their face->Surfaces which have been moved. Drawing the face
//           //      will generate error messages, drawing the surface won't
//           smgfx_SetLook(1,2, 1,.5,0) ; pSurface->DrawUV(3,3) ; sm_GraphicsLoop() ;
//           sm_GraphicsLoop() ;
//         }
// #endif // SM_DEBUG_CODE
//     } // end iter every surface

  // Restore face->RectangularTrim states
  for(ii=0;ii<crFaces.GetSize();ii++) 
    { crFaces[ii]->SetIsRectangularTrim(sRectTrimState[ii]) ; }

  // all done
  return SM_SUCCESS;

} // end SmSpaceBend::TransformTopology

/*******************************************************************//**
PURPOSE: Apply Bend map to topology within a single Brep.

RETURNS: SM_SUCCESS when all given topology is mapped.
         SM_ERROR when any curve or surface mapping call fails. In which
                  case, the BREP is left in a mangled state - the
                  given topology is an unknown mix of 
                  mapped and unmapped objects.

NOTES: If subsets of the Brep's faces, edges, and vertices are
       mapped rather than all the Brep's topology, then
       after this call the Brep may no longer be valid in that
       the original topology connections between objects will be preserved
       but the new geometric relationships between objects may not be connected.

       It's up to the caller to manage moving the bits and pieces
       of a Brep until all the pieces that are connected topologically 
       are connected geometrically once again.

       So, don't run AssertValid() on the modified Brep until it's all put
       back together again.
***********************************************************************/
SmStatus SmSpaceBend::BendTopology
  (const SmBendVolume       & crBendVolume,  // in : Bend mapping to apply
         SmBrep             & rBrep,         // in : Brep containing all topology to be transformed
   const SmTArray<SmFace*>  & crFaces,       // in : faces to transform
   const SmTArray<SmEdge*>  & crEdges,       // in : edges to transform 
   const SmTArray<SmVertex*>& crVertices)    // in : vertices to transform 
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
#endif // SM_DEBUG_CODE

  // locals
  ULONG ii ;
  const SmContext & crContext = *rBrep.GetContext() ;
// Remove Composites
// SmTArray<SmSurface*> sSurfaces ;
// SmTArray<SmCurve*>   sCurves ;

  // let the Brep clean up its stored caches
  rBrep.Notify(SM_NO_PRE_EDIT, this, NULL, NULL);

  // Bend vertices
  for(ii=0;ii<crVertices.GetSize();ii++) 
    { 
      SmVertex * pVertex   = crVertices[ii] ;
      SmPoint3d  sPrePoint = pVertex->GetPoint() ;
      SmPoint3d  sPostPoint ;

      crBendVolume.MapPoint(sPrePoint, sPostPoint) ;
      
      pVertex->Notify(SM_NO_PRE_EDIT, pVertex, this, NULL) ;
      pVertex->SetPoint(sPostPoint) ;

#ifdef SM_DEBUG_CODE
      // draw transformed vertex 
      if(bDebugMe)
        {
          smgfx_SetLook(4,5, 0,.5,1) ; sPostPoint.Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE
    } // end iter bend vertices

  // edge->curves

// Remove Composites - added new block to replace old Composite sensitive block
  // for every Edge->Curve
  for(ii=0;ii<crEdges.GetSize();ii++)
    {
      SmEdge  * pEdge  = crEdges[ii] ;
      SmCurve * pCurve = pEdge->GetCurve() ;
      SmCrvInVolume * pCrvInVolume = new (crContext) SmCrvInVolume
                                       (*pCurve,                  // in : projected Curve   - When(lOwnerFlag&1) rCurve->m_pOwner = this
                                        FALSE,                    // in : TRUE = rCurve is in rVolume's ParamSpace, FALSE= in InSpace
                                        (SmVolume &)crBendVolume, // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
                                        3,                        // in : 3 = copy both curve and volume
                                        3) ;                      // in : 3 = delete both curve and volume when destructed
      
      // replace the curve (calls pEdge->Notify() which clears existing Curve and UVTrimCurves)
      pEdge->Replace3DCurve(pCrvInVolume) ;

      // gwc: do not do BSpline replacement here - Brep is not stitched until caller moves everything back together
      //      ApproximateWithBSpline connects the curve shape with its connected vertex locs - right now a mistake
      //  #ifndef SM_EXACT_SPACEDEF_GEOMETRY
      //        // when disallowing NonBpsline Geometries - replace the CrvInVolume with a BSplineCurve approximation
      //        double dAchievedTol ;
      //        SmStatus sRtn = pEdge->ApproximateWithBSpline(m_dApproxTol, dAchievedTol) ;
      //        if( sRtn != SM_SUCCESS )
      //          {
      //            SM_ASSERT_MSG( sRtn == SM_SUCCESS, _T("DoSpaceBend::BendTopology - call SmEdge::ApproximateWithBSpline() failed")) ; 
      //          }
      //  
      //  #endif // SM_EXACT_SPACEDEF_GEOMETRY

#ifdef SM_DEBUG_CODE
      // draw bent curve 
      if(bDebugMe)
        {
          smgfx_SetLook(4,5, 0,.5,1) ; pEdge->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE
    }

// Remove Composites
//   // watch out for CEdges - don't apply transformation to one curve multiple times
//   for(ii=0;ii<crEdges.GetSize();ii++)
//     {
//       SmEdge  * pEdge  = crEdges[ii] ;
//       SmCurve * pCurve = pEdge->GetCurve() ;
// 
//       if(pEdge->IsKindOf(SmCEdge_TYPE)) { sCurves.AddUnique(pCurve) ; }
//       else                              { sCurves.Add(pCurve) ; }
//     }
// 
//     // for every curve - bend it
//     for(ii=0;ii<sCurves.GetSize();ii++)   
//       { 
//         SmCurve       * pCurve       = sCurves[ii] ;
//         SmEdge        * pEdge        = (SmEdge*)pCurve->GetEdge() ; // Is either edge or cedge
//         SmCrvInVolume * pCrvInVolume = new (crContext) SmCrvInVolume
//                                          (*pCurve,                  // in : projected Curve   - When(lOwnerFlag&1) rCurve->m_pOwner = this
//                                           FALSE,                    // in : TRUE = rCurve is in rVolume's ParamSpace, FALSE= in InSpace
//                                           (SmVolume &)crBendVolume, // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
//                                           3,                        // in : 3 = copy both curve and volume
//                                           3) ;                      // in : 3 = delete both curve and volume when destructed
//         
//         // replace the curve (calls pEdge->Notify() which clears existing Curve and UVTrimCurves)
//         pEdge->Replace3DCurve(pCrvInVolume) ;
// 
//         // gwc: do not do BSpline replacement here - Brep is not stitched until caller moves everything back together
//         //      ApproximateWithBSpline connects the curve shape with its connected vertex locs - right now a mistake
//         //  #ifndef SM_EXACT_SPACEDEF_GEOMETRY
//         //        // when disallowing NonBpsline Geometries - replace the CrvInVolume with a BSplineCurve approximation
//         //        double dAchievedTol ;
//         //        SmStatus sRtn = pEdge->ApproximateWithBSpline(m_dApproxTol, dAchievedTol) ;
//         //        if( sRtn != SM_SUCCESS )
//         //          {
//         //            SM_ASSERT_MSG( sRtn == SM_SUCCESS, _T("DoSpaceBend::BendTopology - call SmEdge::ApproximateWithBSpline() failed")) ; 
//         //          }
//         //  
//         //  #endif // SM_EXACT_SPACEDEF_GEOMETRY
// 
// #ifdef SM_DEBUG_CODE
//       // draw bent curve 
//       if(bDebugMe)
//         {
//           smgfx_SetLook(4,5, 0,.5,1) ; pEdge->Draw() ; sm_GraphicsLoop() ;
//           sm_GraphicsLoop() ;
//         }
// #endif // SM_DEBUG_CODE
//       }

  // face->Surfaces

  // save face->RectangularTrim states
  SmTArray<SmBoolean> sRectTrimState(crFaces.GetSize(), NULL, crFaces.GetSize()) ;
  for(ii=0;ii<crFaces.GetSize();ii++) 
    { sRectTrimState[ii] = crFaces[ii]->GetRectangularTrim() ; }

// Remove Composites - added new block to replace old Composite sensitive block
  // iter every Face->Surface
  for(ii=0;ii<crFaces.GetSize();ii++)
    {
      SmFace    * pFace    = crFaces[ii] ;
      SmSurface * pSurface = pFace->GetSurface() ;
      SmSrfInVolume * pSrfInVolume = new (crContext) SmSrfInVolume
                                       (*pSurface,                  // in : projected Surface - When(lOwnerFlag&1) rSurface->m_pOwner = this
                                        FALSE,                      // in : TRUE = m_pSurface is in m_pVolume's ParamSpace, FALSE = in InSpace
                                        (SmVolume &)crBendVolume,   // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
                                        2,                          // in : 2 = copy volume and save surface orig
                                        3) ;                        // in : 3 = delete both surface and volume when destructed

      // note: pSurface and crBendVolume copy are 'owned by pSrfInVolume and have m_pOwner = pSrfInVolume.
      //       But, pFace also 'owns' pSurface because pFace->GetSurface() == pSurface. 
      //       pSurface can't have two owners - that gets fixed by the following ReplaceSurface() call.

      // replace the surface (calls pFace->Notify() which clears the m_bRectangularTrim bit)
      SER( ReplaceSurface(pFace, pSrfInVolume ) ) ;

        // gwc: do not do BSpline replacement here - Brep is not stitched until caller moves everything back together
        //      ApproximateWithBSpline connects the surface shape with its connected edges shapes - right now a mistake
        //  #ifndef SM_EXACT_SPACEDEF_GEOMETRY
        //        // when disallowing NonBpsline Geometries - replace the SrfInVolume with a BSplineSurface approximation
        //        double dAchievedTol ;
        //        SmStatus sRtn = pFace->ApproximateWithBSpline(m_dApproxTol, dAchievedTol) ;
        //        if( sRtn != SM_SUCCESS )
        //          {
        //            SM_ASSERT_MSG( sRtn == SM_SUCCESS, _T("DoSpaceBend::BendTopology - call SmFace::ApproximateWithBSpline() failed")) ; 
        //          }
        //  
        //  #endif // SM_EXACT_SPACEDEF_GEOMETRY

#ifdef SM_DEBUG_CODE
      // draw bent surface 
      if(bDebugMe)
        {
          smgfx_SetLook(1,2, 0,.5,1) ; pFace->DrawUV(3,3) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE
    } // end iter every surface

// Remove Composites
//   // watch out for composite faces - don't apply transformation to one surface multiple times
//   for(ii=0;ii<crFaces.GetSize();ii++)
//     {
//       SmFace    * pFace    = crFaces[ii] ;
//       SmSurface * pSurface = pFace->GetSurface() ;
// 
//       if(pFace->IsKindOf(SmCFace_TYPE)) { sSurfaces.AddUnique(pSurface) ; }
//       else                              { sSurfaces.Add(pSurface) ; }
//     }
// 
//   // for every surface - bend it
//   for(ii=0;ii<sSurfaces.GetSize();ii++) 
//     {
//       SmSurface     * pSurface     = sSurfaces[ii] ; NER(pSurface);
//       SmFace        * pFace        = (SmFace*)pSurface->GetFace();
//       SmSrfInVolume * pSrfInVolume = new (crContext) SmSrfInVolume
//                                        (*pSurface,                  // in : projected Surface - When(lOwnerFlag&1) rSurface->m_pOwner = this
//                                         FALSE,                      // in : TRUE = m_pSurface is in m_pVolume's ParamSpace, FALSE = in InSpace
//                                         (SmVolume &)crBendVolume,   // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
//                                         2,                          // in : 2 = copy volume and save surface orig
//                                         3) ;                        // in : 3 = delete both surface and volume when destructed
// 
//       // note: pSurface and crBendVolume copy are 'owned by pSrfInVolume and have m_pOwner = pSrfInVolume.
//       //       But, pFace also 'owns' pSurface because pFace->GetSurface() == pSurface. 
//       //       pSurface can't have two owners - that gets fixed by the following ReplaceSurface() call.
// 
//       // replace the surface (calls pFace->Notify() which clears the m_bRectangularTrim bit)
//       SER( ReplaceSurface(pFace, pSrfInVolume ) ) ;
// 
//         // gwc: do not do BSpline replacement here - Brep is not stitched until caller moves everything back together
//         //      ApproximateWithBSpline connects the surface shape with its connected edges shapes - right now a mistake
//         //  #ifndef SM_EXACT_SPACEDEF_GEOMETRY
//         //        // when disallowing NonBpsline Geometries - replace the SrfInVolume with a BSplineSurface approximation
//         //        double dAchievedTol ;
//         //        SmStatus sRtn = pFace->ApproximateWithBSpline(m_dApproxTol, dAchievedTol) ;
//         //        if( sRtn != SM_SUCCESS )
//         //          {
//         //            SM_ASSERT_MSG( sRtn == SM_SUCCESS, _T("DoSpaceBend::BendTopology - call SmFace::ApproximateWithBSpline() failed")) ; 
//         //          }
//         //  
//         //  #endif // SM_EXACT_SPACEDEF_GEOMETRY
// 
// #ifdef SM_DEBUG_CODE
//       // draw bent surface 
//       if(bDebugMe)
//         {
//           smgfx_SetLook(1,2, 0,.5,1) ; pFace->DrawUV(3,3) ; sm_GraphicsLoop() ;
//           sm_GraphicsLoop() ;
//         }
// #endif // SM_DEBUG_CODE
//     } // end iter every surface

  // Restore face->RectangularTrim states
  for(ii=0;ii<crFaces.GetSize();ii++) 
    { crFaces[ii]->SetIsRectangularTrim(sRectTrimState[ii]) ; }

  // all done
  return(SM_SUCCESS) ;

} // end SmSpaceBend::BendTopology

/*******************************************************************//**
PURPOSE: Replace the surface of this face and all other composites.

NOTES: A specialized version of SmBrep::ReplaceSurface
       that knows that the NewSurface is a SmSrfInVolume replacement
       for the current OldSurface.  Having this Method saves a lot of 
       unneeded copies and deletes because

       1. The current face is known to be a member of the NewSurface
       2. The UVDomains of the current and new surfaces are the same
       3. The UVTrimCurves of the current and new surfaces are the same
***********************************************************************/
SmStatus SmSpaceBend::ReplaceSurface
  (SmFace            * pFace,        // in : Target Face to get new Surface
   SmSrfInVolume     * pNewSurface)  // in : Target Surface to replace current pFace->Surface
{
  // locals 
  SM_PTR_ARRAY(sChildFaces, SmFace, 32);
  SmSurface * pOld      = pFace->GetSurface() ;
  // SmFace    * pTopFace  = (SmFace*) pOld->GetFace() ;

  // inform the face
  pFace->Notify(SM_NO_PRE_EDIT, pFace, this, NULL) ;

  // change surface<->owner pointers
  pNewSurface->SetOwner(pFace) ;
  pFace->SetSurface(pNewSurface) ;

  // clear the old surface->Owner if it is not the base surface of SmSrfInVolume
  if(pNewSurface->GetSurface() != pOld)
    { pOld->SetOwner(NULL) ; } 

// Remove Composites
//  // for composite faces - set the Surface pointers on every member face
//  if(pFace->IsKindOf(SmCFace_TYPE)) 
//    {
//      SmCFace *pCF = (SmCFace*)pFace;
//      pCF->GetFaces(sChildFaces);
//
//      // for every composite face member
//      for(ii=0;ii<sChildFaces.GetSize();ii++) 
//        {
//          SmFace *pChildFace = sChildFaces[ii];
//
//          // update the member face Surface pointer
//          pChildFace->SetSurface(pNewSurface, FALSE); // FALSE = don't delete previous surface
//        }
//    } // end composite face check

  // all done
  return SM_SUCCESS;

} // end SmSpaceBend::ReplaceSurface

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertBend_list[] =
{
 /* 00 */ { SM_AT_POINTER,     _T("NULL Pointer"),     _T("Target Brep must be nonNull") },
 /* 01 */ { SM_AT_VALUES,      _T("BendAngle Range"),  _T("BendAngle must be in the range -180 to + 180 degrees") },
 /* 02 */ { SM_AT_VALUES,      _T("Neg BendRadius"),   _T("The rule: NeutralRadius >= KFactor * SheetThickness must be TRUE") },
 /* 03 */ { SM_AT_UNIT_VECTOR, _T("Unit Vector"),      _T("BendAxis vector must be unit length") },
 /* 04 */ { SM_AT_UNIT_VECTOR, _T("Unit Vector"),      _T("BendDirection vector must be unit length") },
 /* 05 */ { SM_AT_VECTOR,      _T("Parallel Vectors"), _T("BendAxis and BendDirection can't be parallel") },
 /* 06 */ { SM_AT_VALUES,      _T("KFactor Range"),    _T("KFactor must be in the range 0.0 to 1.0") }
} ;   

/*******************************************************************//**
PURPOSE:  Check that Bend Analytic and Nurb representations are
             equivalent.

NOTES:  returns TRUE = OK, FALSE = problem
***********************************************************************/
SmBoolean SmSpaceBend::AssertValid
 (SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore, default:[NULL]
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests
                                    //      default:[SM_LEVEL_0]
  SmAssertWalking    eWalkTree,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
 const
{
  // init return value
  SmBoolean bRtn = TRUE ;

  // call the base class AssertValid
  bRtn &= SmObject::AssertValid(pAList, eTestLevel, SM_NO_WALK, pTestRequests) ;

#ifdef SM_DEBUG_CODE
static constexpr ULONG bDebugMe = FALSE ;
  if(bDebugMe)
    {
      // call assert valid on the nested objects
      if(m_pBrep)       bRtn &= m_pBrep->AssertValid(pAList, eTestLevel, eWalkTree) ; 
      if(m_pBendDomain) bRtn &= m_pBendDomain->AssertValid(pAList, eTestLevel, eWalkTree) ;
    }
#else
  SM_REF1(eWalkTree);
#endif // SM_DEBUG_CODE

  // Target Brep must be nonNULL
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_pBrep != NULL), _T("")) ;

  // BendAngle must be in the range -180 to + 180 degrees
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (m_dBendAngDeg >= -180.0 && m_dBendAngDeg <= 180.0), _T("")) ;

  // The rule: NeutralRadius >= KFactor * SheetThickness must be TRUE
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (m_dNeutralRadius >= m_dKFactor * m_dSheetThickness), _T("")) ;

  // BendAxis vector must be unit length   
  double dLengthSq = m_vBendAxis.LengthSquared() ;                     
  bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0, SM_ARE_SAME_TO_TOL(dLengthSq, 1.0, SM_EFF_ZERO_SQ), SM_EFF_ZERO, dLengthSq, _T("")) ;

  // BendDirection vector must be unit length                  
  dLengthSq = m_vBendDirection.LengthSquared() ;                     
  bRtn &= SM_ASSERT_VALUE_REPORT(4, SM_LEVEL_0, SM_ARE_SAME_TO_TOL(dLengthSq, 1.0, SM_EFF_ZERO_SQ), SM_EFF_ZERO, dLengthSq, _T("")) ;

  // BendAxis and BendDirection can't be parallel
  double dAngRad ;
  m_vBendAxis.AngleBetween(m_vBendDirection, dAngRad) ;              
  bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0, (FALSE == m_vBendAxis.IsParallelTo(m_vBendDirection, SM_EFF_ZERO_DEG)), SM_EFF_ZERO_DEG, SM_RAD2DEG(dAngRad), _T("") ) ;

  // KFactor must be in the range 0.0 to 1.0                    
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(6, SM_LEVEL_0, (m_dKFactor >= 0.0 && m_dKFactor <= 1.0), _T("") ) ;

  // all done
  return(bRtn) ;

} // end SmSpaceBend::AssertValid

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSpaceBend::IsKindOf( SM_TYPE t ) const
{
  return ((SmSpaceBend_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump Bend data out for debugging.

NOTES: Pretty Print * Bend Parameters  
                    * Target Brep to bend
                    * BendDomain Brep model (the bit of space in to be bent)
                    * StartBendBoundary Brep (the sheet of target Brep geometry on the start boundary of BendDomain)
                    * StopBendBoundary  Brep (the sheet of target Brep geometry on the stop S boundary of BendDomain)
***********************************************************************/
void SmSpaceBend::Dump() const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
  smos_WriteBuffer(_T("\nBegin SmSpaceBend::Dump()")) ;

  // get the parent's dump
  SmObject::Dump();

  // output header and brep
  smos_sprintf(sBuff,       _T("\n pBend = 0x%p, pBrep = 0x%p"), this, m_pBrep) ;
  smos_sprintf(sBuffForFile,_T("\n pBend = %s, pBrep = %s"), _T("notNULL"), m_pBrep ? _T("notNULL") : _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // output basic bend and sheet data
  smos_sprintf(sBuff, _T("\n  BendAngDeg        : [%16.16lf]"), m_dBendAngDeg) ;     smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff, _T("\n  BendNeutralRad    : [%16.16lf]"), m_dNeutralRadius) ;  smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff, _T("\n  BendSheetThickness: [%16.16lf]"), m_dSheetThickness) ; smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff, _T("\n  BendKFactor       : [%16.16lf]"), m_dKFactor) ;        smos_WriteBuffer(sBuff);

  // output bend position, orientation, and extent data
  smos_WriteBuffer(_T("\n  Bend Axis Point   : ")) ; m_vBendAxisPoint.Dump() ;
  smos_WriteBuffer(_T("\n  Bend Neutral MidPt: ")) ; GetNeutralMidPoint().Dump() ;
  smos_WriteBuffer(_T("\n  Bend Axis (Z Axis): ")) ; m_vBendAxis.Dump() ;
  smos_WriteBuffer(_T("\n  Bend Direction (X): ")) ; m_vBendDirection.Dump() ;
  smos_WriteBuffer(_T("\n  Bend Binormal (Y) : ")) ; (m_vBendAxis * m_vBendDirection).Dump() ;
  smos_WriteBuffer(_T("\n  Bend Interval     : ")) ; m_vBendInterval.Dump() ;

  // output basic bend and sheet data
  smos_sprintf(sBuff, _T("\n  Final PartPosition:[%s]\n"),
               m_eBendOrient == SM_BO_FIXED_BEFORE_BENDING ? _T("Fixed Start Bending = before Bend geometry remains fixed")
             : m_eBendOrient == SM_BO_CENTER_BENDING       ? _T("Center Bending = before and after Bend geometries rotate equal and opposite dirs")
             : m_eBendOrient == SM_BO_FIXED_AFTER_BENDING  ? _T("Fixed End Bending = after Bend geometry remains fixed")
             :                                               _T("Unknown Unbending type")) ;  
  smos_WriteBuffer(sBuff);

  // dump nested objects
  // dump nested objects
  smos_WriteBuffer(_T("\nBegin Dump SmSpaceUnbend Nested Objects:[TgtBrep, BendDomainBrep, StartBendBndryBrep, StopBendBndryBrep]")) ;


  // m_pBrep
  smos_WriteBuffer(_T("\nBegin Nested SmSpaceBend->m_pBrep::Dump()")) ;
    if(m_pBrep) {  m_pBrep->Dump(SM_BD_GEOM_TYPES) ; }
    else        {  smos_WriteBuffer(_T("\n   SmSpaceBend->m_pBrep is NULL ")) ; }
  smos_WriteBuffer(_T("End    Nested SmSpaceBend->m_pBrep::Dump()\n")) ;

  // BendDomainRegion
  smos_WriteBuffer(_T("\nBegin Nested SmSpaceBend->m_pBendDomain::Dump()")) ;
    if(m_pBendDomain) {  m_pBendDomain->Dump(SM_BD_GEOM_TYPES) ; }
    else                 {  smos_WriteBuffer(_T("\n   SmSpaceBend->m_pBendDomain is NULL ")) ; }
  smos_WriteBuffer(_T("End    Nested SmSpaceBend->m_pBendDomain::Dump()\n")) ;

  // StartBendBoundary
  smos_WriteBuffer(_T("\nBegin Nested SmSpaceBend->m_pStartBendBoundary::Dump()")) ;
    if(m_pBendDomain) {  m_pStartBendBoundary->Dump(SM_BD_GEOM_TYPES) ; }
    else                 {  smos_WriteBuffer(_T("\n   SmSpaceBend->m_pStartBendBoundary is NULL ")) ; }
  smos_WriteBuffer(_T("End    Nested SmSpaceBend->m_pStartBendBoundary::Dump()\n")) ;

  // StopBendBoundary
  smos_WriteBuffer(_T("\nBegin Nested SmSpaceBend->m_pStopBendBoundary::Dump()")) ;
    if(m_pBendDomain) {  m_pStopBendBoundary->Dump(SM_BD_GEOM_TYPES) ; }
    else                 {  smos_WriteBuffer(_T("\n   SmSpaceBend->m_pStopBendBoundary is NULL ")) ; }
  smos_WriteBuffer(_T("End    Nested SmSpaceBend->m_pStopBendBoundary::Dump()\n")) ;

  // end dump nested objects
  smos_WriteBuffer(_T("\nEnd Dump SmSpaceUnbend Nested Objects:[TgtBrep, BendDomainBrep, StartBendBndryBrep, StopBendBndryBrep]")) ;

  // all done
  smos_WriteBuffer(_T("\nEnd SmSpaceBend::Dump()\n")) ;

} // end SmSpaceBend::Dump

/*******************************************************************//**
PURPOSE:  Add Bend graphics for global display Parameters
             to new DisplayList added to global DisplayList array.

NOTES:
***************************************************************/
SmDisplayList * SmSpaceBend::Draw
  (SmBoolean       bDrawBrep,          // in : TRUE = Draw target m_pBrep, default:[FALSE]
   SmBoolean       bAddToUIPickList,   // NotUsed: in : TRUE = Add target m_pBrep to UI pick interface for debugging
                                       //      default:[FALSE]
   SmBoolean       bDrawPreBendRegion, // in : TRUE = Draw preBendRegion, default:[FALSE] 
   SmGfxArraySet * pOptGfxSet)         // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
  const                                //      NULL to ignore. default:[NULL]
{
  SM_REF1(bAddToUIPickList) ;
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // start new DisplayList (unless displayList is already open)
  SmVector3d sColor = smgfx_GetOutputColor(pOptGfxSet) ;
  double dLineWidth = smgfx_GetLineWidth() ;
  double dPointSize = smgfx_GetPointSize() ;
  smgfx_Open(smgfx_GetRuleColor(this), NULL, NULL, FALSE, pOptGfxSet);

  // Draw the Brep
  if(bDrawBrep)
    { m_pBrep->Draw() ; }   // gwc: need to make SmBrep::Draw() pOptGfxSet aware

  // Draw the preBendRegion BendDomain, StartBendBoundary, and StopBendBoundary
  if(bDrawPreBendRegion)
    {
      // draw the preBendRegion
      smgfx_SetLook(dLineWidth+2, 1, 0,0,0, pOptGfxSet) ; GetBendDomain()->Draw(TRUE) ; // gwc: need to make SmBrep::Draw() pOptGfxSet aware

      // set Brep faces to draw crosshatched
      SmDisplayParameters & rDisp              = smgfx_RefGlobalDisplayParameters() ;
      SmBoolean             bOldDrawCrossHatch = rDisp.m_bDrawCrossHatch ; 
      rDisp.m_bDrawCrossHatch = TRUE ;
  
      // Draw the preBendRegion StartBoundary
      smgfx_SetLook(dLineWidth+2, 1, 0,1,0, pOptGfxSet) ; GetStartBendBoundary()->Draw() ; // gwc: need to make SmBrep::Draw() pOptGfxSet aware
      smgfx_SetLook(dLineWidth+2, 1, 1,0,0, pOptGfxSet) ; GetStopBendBoundary()->Draw() ;  // gwc: need to make SmBrep::Draw() pOptGfxSet aware

      // restore state
      smgfx_SetLook(dLineWidth, dPointSize, sColor, pOptGfxSet) ; 
      rDisp.m_bDrawCrossHatch = bOldDrawCrossHatch ;
    }

  // Bend Point on Neutral Axis
  SmPoint3d sNeutralPoint = GetNeutralMidPoint() ;
  smgfx_OutputPoint(sNeutralPoint.x, sNeutralPoint.y, sNeutralPoint.z, pOptGfxSet) ; 

  // Bend Axis
  SmPoint3d sBendStart = GetBendSegmentStart() ;
  SmPoint3d sBendStop  = GetBendSegmentStop() ;
  smgfx_OutputLine(sBendStart.x, sBendStart.y, sBendStart.z,
                   sBendStop.x,  sBendStop.y,  sBendStop.z, pOptGfxSet) ;  

  // Bend Graphics - center point
  SmPoint3d sAxisPoint = GetBendAxisPoint() ; 
  smgfx_OutputPoint( sAxisPoint.x, sAxisPoint.y, sAxisPoint.z, pOptGfxSet) ;
  
  // Bend Graphics - Bend Direction from CenterPoint to NeutralPoint
  smgfx_OutputLine(sAxisPoint.x,    sAxisPoint.y,    sAxisPoint.z,
                   sNeutralPoint.x, sNeutralPoint.y, sNeutralPoint.z, pOptGfxSet) ;

  // Bend Graphics - positive Bend cross-direction  (used to check Start and Stop faces)
  // with U = m_vBendAxis
  //      V = m_vBendDirection * m_vBendAxis 
  //      W = m_vBendDirection
  double     dHalfWidth    = GetPreBendWidth() / 2.0 ; 
  SmVector3d sVAxis        = GetBendBinormal() ;
  smgfx_SetLook(dLineWidth+1, dPointSize, 1,.647,0, pOptGfxSet) ; // orange
  smgfx_OutputLine(sNeutralPoint.x-dHalfWidth*sVAxis.x, 
                   sNeutralPoint.y-dHalfWidth*sVAxis.y, 
                   sNeutralPoint.z-dHalfWidth*sVAxis.z,
                   sNeutralPoint.x+dHalfWidth*sVAxis.x, 
                   sNeutralPoint.y+dHalfWidth*sVAxis.y, 
                   sNeutralPoint.z+dHalfWidth*sVAxis.z, pOptGfxSet) ;
  smgfx_SetLook(dLineWidth, dPointSize, sColor, pOptGfxSet) ; 

  // Bend Graphics - circular arcs at sheet inside, outside, and neutral surfaces
  ULONG ii, jj, kk ; 
  const ULONG lSideCnt = 7 ; 
  const ULONG lCnt     = 2 * lSideCnt + 1 ;  // an odd number
  double dRadius[3] ;
  dRadius[0] = GetInsideRadius() ;  
  dRadius[1] = GetOutsideRadius() ; 
  dRadius[2] = GetNeutralRadius() ;
  SmVector3d sX =  GetBendDirection() ;
  SmVector3d sY =  GetBendBinormal() ;  
  SmVector3d sZ =  GetBendAxis() ;
  SmPoint3d sPts[lCnt] ;
  double dRad, dIncRad = SM_DEG2RAD( m_dBendAngDeg/2.0/lSideCnt ) ;
  SmAxis2Placement sInsideBendTransform = GetInsideBendTransform() ; 

  // for kk slices along the length of the bend axis
  for(kk=0;kk<3;kk++)
    { 
      double dParam = (double)kk/(3.0 - 1.0) ;
      sAxisPoint = (1.0 - dParam) * sBendStart + (dParam) * sBendStop ;

      // for inside, outside, and neutral radius values
      for(ii=0;ii<3;ii++)
        { 
          // make the neutral arc a little heavier and orange
          if(ii==2) { smgfx_SetLook(dLineWidth+1, dPointSize, 1,.647,0, pOptGfxSet) ; } // orange
          else      { smgfx_SetLook(dLineWidth, dPointSize, sColor, pOptGfxSet) ; } // 
      
          // arc mid point
          sPts[lSideCnt] = sAxisPoint + dRadius[ii] * sX ; 
          sInsideBendTransform.TransformPoint(sPts[lSideCnt], sPts[lSideCnt]) ;

          // arc sample points (on either side of the arc mid point)
          for(jj=1, dRad=dIncRad;jj<=lSideCnt;jj++,dRad+=dIncRad)
            {
              double dCos = smos_Cosine(dRad) ;
              double dSin = smos_Sine(dRad) ;
              sPts[lSideCnt + jj] = sAxisPoint + dRadius[ii] * (dCos * sX + dSin * sY) ;
              sPts[lSideCnt - jj] = sAxisPoint + dRadius[ii] * (dCos * sX - dSin * sY) ;
              sInsideBendTransform.TransformPoint(sPts[lSideCnt + jj], sPts[lSideCnt + jj]) ;
              sInsideBendTransform.TransformPoint(sPts[lSideCnt - jj], sPts[lSideCnt - jj]) ;
            }

          // output arc
          smgfx_OutputPolyline((double *)sPts, lCnt, 3, pOptGfxSet) ; 
        } // end iter ii, inside, outside, and neutral sheetmetal bend arcs
    } // end iter kk, slices along the bend axis
  
  // all done - restore state and end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  smgfx_SetLineWidth(dLineWidth, pOptGfxSet) ;
  smgfx_SetPointSize(dPointSize, pOptGfxSet) ;
    
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF3(bDrawBrep, bDrawPreBendRegion, pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmSpaceBend::Draw

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmSpaceBend::AssertHeal
//  (SmAssertReport & rAReport,  // in : a report generated by AssertValid
//   SmAssertArray  * pAList)    // in : AssertArray holding rAReport
// {
//   SmBoolean bRtn = FALSE ;
// 
//   // check state - no work
//   if(rAReport.m_bOK == TRUE)
//     { return( TRUE ) ; }
// 
//   // check state - not the class that generated this report - pass call to parent class
//   if(rAReport.m_lReportingType != GetClassType())
//     {
//       // pass the call along to the parent - return ( Parent::AssertHeal(rAReport, pAList) ) ;
//       return( SmObject::AssertHeal(rAReport, pAList) ) ;
//     }
// 
//   // branch on the report type
//   switch(rAReport.m_lTestIndex)
//     {
//       case 99 : { // set case number appropriately - run fix code here
//                   // if fix works set rAReport.m_bOK = TRUE ;
//                 }
//                 break ;
// 
//       default: rAReport.m_eAssertType  = SM_AT_NO_HEAL_YET ;
//                rAReport.m_pHealMessage = _T("SmSpaceBend::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmSpaceBend::AssertHeal
// end obsolete
