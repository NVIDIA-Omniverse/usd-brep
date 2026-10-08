// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************
* FILE NAME --- SmSpaceUnbend.cpp
* PURPOSE: Implementation of brep bend/unbend methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmSpaceUnbend.h>
#include <SmGraphicsOutput.h>
#include <SmPrimitiveCreation.h>
#include <SmMerge.h>
#include <SmTopologyTraverser.h>
#include <SmUnbendVolume.h>
#include <SmCrvInVolume.h>
#include <SmSrfInVolume.h>
#include <SmAssertArray.h>
#include <SmSpaceBend.h>
#include <SmAttribute.h>
#include <SmFace.h>
// Remove Composites
// #include <SmCFace.h>
#include <SmEdge.h>
// Remove Composites
// #include <SmCEdge.h>

/*******************************************************************//**
PURPOSE:  Utility for mapping SpaceUnbend and SpaceBend enum values

NOTES:  
***********************************************************************/
SmBendOrientTYPE MapType(SmUnbendOrientTYPE eUnbendOrient)
{ switch(eUnbendOrient) { case SM_UO_ORIENT_UNKNOWN         : return SM_BO_ORIENT_UNKNOWN ;
                          case SM_UO_FIXED_BEFORE_UNBENDING : return SM_BO_FIXED_BEFORE_BENDING ;
                          case SM_UO_CENTER_UNBENDING       : return SM_BO_CENTER_BENDING ;
                          case SM_UO_FIXED_AFTER_UNBENDING  : return SM_BO_FIXED_AFTER_BENDING ;
                          default :                           return SM_BO_ORIENT_UNKNOWN ;
                        }
}

SmUnbendOrientTYPE MapType(SmBendOrientTYPE eBendOrient)
{ switch(eBendOrient) { case SM_BO_ORIENT_UNKNOWN       : return SM_UO_ORIENT_UNKNOWN ;              
                        case SM_BO_FIXED_BEFORE_BENDING : return SM_UO_FIXED_BEFORE_UNBENDING ;
                        case SM_BO_CENTER_BENDING       : return SM_UO_CENTER_UNBENDING ;     
                        case SM_BO_FIXED_AFTER_BENDING  : return SM_UO_FIXED_AFTER_UNBENDING ;
                        default :                         return SM_UO_ORIENT_UNKNOWN ;
                      }
}

/*******************************************************************//**
PURPOSE:  create SpaceUnbend inverse of input SpaceBend

NOTES:  
***********************************************************************/
SmStatus SmSpaceUnbend::CreateUnbendInverse
 (const SmSpaceBend & crSpaceBend,      // in : the SpaceBend to invert
  SmBrep            & rBrep,            // in : Brep target for new rpSaceUnbend
  SmSpaceUnbend    *& rpSpaceUnbend)    // out: the SpaceUnbend inverse to the input SpaceBend.
                                        //      NULL on input.
{
  // init output 
  AE_MSG(rpSpaceUnbend == NULL, _T("warning: SmSpaceUnbend::CreateUnbendInverse given a NonNULL rpSpaceUnbend input argument - potential memory leak")) ;
  rpSpaceUnbend = NULL ;

  // check input - quit when given bad input
  AERN_MSG(crSpaceBend.AssertValid(),SM_ERR,_T("SmSpaceUnbend::CreateUnbendInverse given an invalid or uninitialized SpaceBend argument")) ;

  // locals
  SmBrep            * pBrep                 = &rBrep ;
  double              dUnbendAngDeg         = crSpaceBend.GetBendAngDeg() ;
  double              dUnbendRadius         = crSpaceBend.GetNeutralRadius() ;
  SmExtent1d          rUnbendInterval       = crSpaceBend.GetBendInterval() ;
  double              dUnbendSheetThickness = crSpaceBend.GetSheetThickness() ;
  double              dUnbendKFactor        = crSpaceBend.GetKFactor() ;
  SmUnbendOrientTYPE  eUnbendOrient         = MapType(crSpaceBend.GetBendOrient()) ;  
  double              dUnbendApproxTol      = crSpaceBend.GetApproxTol() ; 
                                                                            
  // Unbend position is the BendPosition mapped through the InsideBendTransform ;
  SmPoint3d    sUnbendAxisPoint, sBendAxisPoint       = crSpaceBend.GetBendAxisPoint() ;
  SmVector3d   sUnbendAxis,      sBendAxis            = crSpaceBend.GetBendAxis() ; 
  SmVector3d   sUnbendDirection, sBendDirection       = crSpaceBend.GetBendDirection() ;
  SmAxis2Placement sInsideBendTransform = crSpaceBend.GetInsideBendTransform() ;

  sInsideBendTransform.TransformPoint (sBendAxisPoint, sUnbendAxisPoint) ;
  sInsideBendTransform.TransformVector(sBendAxis     , sUnbendAxis     ) ;
  sInsideBendTransform.TransformVector(sBendDirection, sUnbendDirection) ;
        
  // inverse SpaceUnbend constructor
  rpSpaceUnbend = new (*pBrep->GetContext()) SmSpaceUnbend
   (*pBrep,                // in : target Brep to unbend
    dUnbendAngDeg,         // in : unbend angle in degrees
    dUnbendRadius,         // in : unbend radius at the neutral plane
    sUnbendAxisPoint,      // in : UnbendAxisPoint   of unbend Centerline = UnbendAxisPoint + s * UnbendAxisUnitVec. 
    sUnbendAxis,           // in : UnbendAxisUnitVec of unbend Centerline = UnbendAxisPoint + s * UnbendAxisUnitVec,
                           //      "the unbend's Z Axis."
    sUnbendDirection,      // in : unit-vector from UnbendAxisPoint towards NeutralPlane center,
                           //      perpendicular to the UnbendAxis and perpendicular to the neutral plane.
                           //      unbend's X Axis.
    rUnbendInterval,       // in : active segment of unbend Centerline = UnbendAxisPoint + s * UnbendAxis.
                           //      use UnbendInterval.Init() for an unbounded unbend line.
    dUnbendSheetThickness, // in : SheetThickness of NeutralRadius = InsideRadius + KFactor * SheetThickness
    dUnbendKFactor,        // in : KFactor        of NeutralRadius = InsideRadius + KFactor * SheetThickness
                           //      neutral plane normalized param between inside and outside sheet surfaces, commonly [.44]
    eUnbendOrient,         // selects the final orientation of the bent part. oneof:
                           // SM_UO_FIXED_BEFORE_BENDING = topology in front of unbend is fixed - after topology is rotated into position
                           // SM_UO_CENTER_BENDING       = topology on either side of unbend is rotated in equal and opposite directions 
                           // SM_UO_FIXED_AFTER_BENDING  = topology after unbend is fixed - before topology is rotated into position
    dUnbendApproxTol) ;    //      default:[1.0e-5] - for now SmBrep::ApproximateWithBSplines() has trouble hitting
                           //                         this tolerance - currently bounded by 1.0e-4 or larger.

  // all done
  return(SM_SUCCESS) ;

} // end SmSpaceUnbend::CreateUnbendInverse

/*******************************************************************//**
PURPOSE:  Create on the heap and rtn a SpaceUnbend defining UnbendVolume

NOTES:  
***********************************************************************/
SmUnbendVolume * SmSpaceUnbend::CreateUnbendVolume
 (const SmContext &crContext)   // in : context for new obj construction
 const
{
    SmVector3d sAxisPt = GetUnbendAxisPoint();                                               
    SmVector3d sBendAxis = GetUnbendAxis();
    SmVector3d sBendDir = GetUnbendDirection();
    double sGetNeutralRad = GetNeutralRadius();

  // construct SmUnbendVolume object
  SmUnbendVolume * pUnbendVolume = new (crContext) SmUnbendVolume (
      crContext,    // in : context for new OrientMap construction 
      sAxisPt,        // in : InSpace (and OutSpace) point on the bend center line                                             
      sBendAxis,    // in : InSpace (and OutSpace) direction of the bend center line marking W axis of Unbend Coordinate System
      sBendDir,        // in : InSpace (and OutSpace) vector orthoganal to UnbendAxis marking U axis of Unbend Coordinate System
      sGetNeutralRad ) ; // in : InSpace (and OutSpace) distance along UnbendMidDir from UnbendOrigin to NeutralPlane.
                           // in : min dist between NonSingular pts and Volume's singularity at U=0,
                           //      dSingularityTol must be >= SM_ZONE_TOL_3D
                           //  note: The UnbendMidDir/UnbendBinormal plane is the symmetric plane
                           //        of the Unbend mapping
  
  // all done
  return( pUnbendVolume ) ;

} // end SmSpaceUnbend::CreateUnbendVolume

/*******************************************************************//**
PURPOSE:  Apply Unbend to Brep

NOTES:  

METHOD: 
 1. Merge TgtBrep with boundaries of InDomain Brep model to ensure all 
    TgtBrep topology objects lie within only a single domain of the bend mapping.
     - sInFaces: Use Boolean Attribute labeling to build sInFaces list:[all faces with the Unbend's InDomain region]
 2. Build Start and Stop Topology lists:[all verts and edges on the Start and Stop UnbendBoundaries] - uses Boolean to find those
     - sStartTopology and
     - sStopTopology 
 3. Build Unbend Region Topology Lists of TgtBrep verts, edges, and faces
     - sBeforeFaces,    sAfterFaces      uses marks and SmTopologyTraverser
     - sBeforeEdges,    sAfterEdges      uses marks and SmTopologyTraverser
     - sBeforeVertices, sAfterVertices   uses marks and SmTopologyTraverser
// Remove Composites
//  4. Replace composite faces and edges spanning multiple Unbend regions with non-Composite faces and edges
//     (this has to be done but also exposes existing Boolean bug of not working properly for case SmBrep::m_bMakeComposites == FALSE]
 5. Shrink all sInFaces geometry - that's useful later if surfaces need to be approximated
 6. Build  - UnbendVolume for InUnbend Geometry
           - rotate and move transform for BeforeUnbend Geometry
           - rotate and move transform for InsideUnbend Geometry
           - rotate and move transform for AfterUnbend  Geometry
 7. Unbend and move Geometry
     - bend InUnbend geometry 
        - UnbendTopology(UnbendVolume, sInFace, sInEdges, sInVertices) ;
     - position regions with
        - TransformTopology(sBeforeUnbendTransform, sBeforeFaces, sBeforeEdges, sBeforeVertices) 
        - TransformTopology(sInsideUnbendTransform, sInFaces,     sInEdges,     sInVertices    ) 
        - TransformTopology(sAfterUnbendTransform,  sAfterFaces,  sAfterEdges,  sAfterVertices ) 
 8. When asked: Approximate SmSurfInVolume and SmCrvInVolume geometries with BSpline Geometry

***********************************************************************/
SmStatus SmSpaceUnbend::DoSpaceUnbend
 (SmBoolean bDoApproximations)  // in : TRUE=replace CrvInVolume and SrfInVolume geometry with BSpline Approximations
                                //      default:[TRUE]
{

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
ULONG di ;
  // draw 
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(this) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(m_pBrep) m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; this->Draw(FALSE, FALSE, TRUE) ;  sm_GraphicsLoop() ; // bDrawBrep, bAddToUIPickList, bDrawPreUnbendRegion 
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // locals
  ULONG ii, jj, kk ;
  SmEdge          * pEdge = NULL;   // iteration variables
  SmVertex        * pVertex = NULL; // iteration variables
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
      SM_DUMP_AND_ASSERT_VALID(GetUnbendDomain()) ;    
      SM_DUMP_AND_ASSERT_VALID(GetStartUnbendBoundary()) ;
      SM_DUMP_AND_ASSERT_VALID(GetStopUnbendBoundary()) ;
      
      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(m_pBrep) { m_pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,0,0) ; if(GetUnbendDomain()) GetUnbendDomain()->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetDrawCrossHatch(TRUE) ;
      smgfx_SetLook(1,2, 0,1,0) ; if(GetStartUnbendBoundary()) GetStartUnbendBoundary()->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,0) ; if(GetStopUnbendBoundary()) GetStopUnbendBoundary()->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetDrawCrossHatch(FALSE) ;
      smgfx_SetLook(1,2, 0,1,1) ; this->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
  
  // STEP 1: build InFaces list containing all Faces within the UnbendRegion
  { // imprint UnbendRegion on m_pBrep - gather faces inside the UnbendRegion and topology on UnbendRegion Boundary          
    SmBrep * pPreUnbendRegion = GetUnbendDomain() ;     // builds PreUnbendRegion when needed

// Remove Composites
//  // don't make composite edges and faces when geometry is split
//  // note: this feature does not work as expected - that's okay, we'll eliminate problem composites before executing the bend
//  // SmBoolean bMakeComposites = m_pBrep->GetMakeComposites() ;
//  SmTemporaryChangeValue<SmBoolean> sChange (m_pBrep->m_bMakeComposites, FALSE);

    // construct merge object to imprint the UnbendRegion on m_pBrep
    SmMerge sMergeUnbendRegion(*cpContext, m_pBrep, pPreUnbendRegion) ;
    sMergeUnbendRegion.SetClassifyFaces(TRUE) ; // within booleans only IIR and label faces to be saved and removed.  

    // remove all attributes with attributeIds that are used to track topology in the upcoming NonManifoldBoolean() call
    m_pBrep->RemoveAttributeFromTopology(lIntersectAttributeId) ;
    m_pBrep->RemoveAttributeFromTopology(lSaveAttributeId) ;
    m_pBrep->RemoveAttributeFromTopology(lDeleteAttributeId) ;

    // imprint the PreUnbendRegion onto the target Brep
    //  1. Splits topology at the UnbendRegion boundaries making sure that
    //     all m_pBrep topology is either completely in, on, or out of the bendRegion
    //  2. Adds attributes with attributeIds SM_AI_BOOLEAN_INTERSECT to all topology on the UnbendRegion Boundaries
    //                                       SM_AI_BOOLEAN_SAVE      to all faces in the UnbendRegion    
    //                                       SM_AI_BOOLEAN_DELETE    to all faces out of the UnbendRegion  

#ifdef SM_DEBUG_CODE
    if(bDebugMe) // pre Boolean Split at Start and Stop InDomain boundarys
      { SM_DUMP_AND_ASSERT_VALID(pPreUnbendRegion) ; 
        SM_DUMP_AND_ASSERT_VALID(m_pBrep) ;    
      }
#endif // SM_DEBUG_CODE

    sMergeUnbendRegion.NonManifoldBoolean(SM_BO_INTERSECTION, pResult) ; 

#ifdef SM_DEBUG_CODE
    if(bDebugMe) // post Boolean Split at Start and Stop InDomain boundarys
      { SM_DUMP_AND_ASSERT_VALID(pPreUnbendRegion) ;
        SM_DUMP_AND_ASSERT_VALID(m_pBrep) ; // m_pBrep == pResult here  
      }
#endif // SM_DEBUG_CODE

    // get the faces in the bend 
    m_pBrep->GetFaces(sInFaces, &lSaveAttributeId) ;

#ifdef SM_DEBUG_CODE
    // draw 
    if(bDebugMe)
      {
        smgfx_Erase() ;
        smgfx_SetLook( 1, 2, 0, 0, 1 ); if(m_pBrep) { m_pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
        smgfx_SetLook(1,2, 0,0,0) ; if(GetUnbendDomain()) GetUnbendDomain()->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetDrawCrossHatch(TRUE) ;
        smgfx_SetLook(1,2, 0,1,0) ; if(GetStartUnbendBoundary()) GetStartUnbendBoundary()->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetLook(1,2, 1,0,0) ; if(GetStopUnbendBoundary()) GetStopUnbendBoundary()->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetDrawCrossHatch(FALSE) ;
        smgfx_SetLook(1,2, 0,1,1) ; this->Draw() ; sm_GraphicsLoop() ;
        smgfx_SetLook(1,2, 0,1,1) ; for(di=0;di<sInFaces.GetSize();di++) 
                                      { if(sInFaces[di]) sInFaces[di]->DrawUV(9,9) ; sm_GraphicsLoop() ; } 
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

  } // end STEP 1: imprint UnbendRegion on m_pBrep
  
  // STEP 2a: build sStartTopology List containing all Edges and Vertices on the StartUnbendBoundary
  { // Imprint MergeStartBoundary on m_pBrep to gather list of UnbendRegion StartBoundary Topology objects
    SmBrep * pStartUnbendBoundary = GetStartUnbendBoundary() ; // builds StartUnbendBoundary when needed

    // Set Merge objects for imprinting
    SmMerge sMergeStartUnbendBoundary(*cpContext, m_pBrep, pStartUnbendBoundary) ;
    sMergeStartUnbendBoundary.SetClassifyFaces(TRUE) ; // GWC: when SetClassifyFaces == TRUE, m_pBreUnbendRegion Brep not consumed in Boolean

    // remove attributes with attributeIds used to track set membership
    // imprint the PreUnbendRegion onto the target Brep
    //  1. Expect no topology changes - those are already done with the last SmMerge::NonManifoldBoolean() call.
    //  2. Adds attributes with attributeIds SM_AI_BOOLEAN_INTERSECT to all topology on the UnbendRegion Boundaries
    //                                       SM_AI_BOOLEAN_SAVE      to all faces in the UnbendRegion    
    //                                       SM_AI_BOOLEAN_DELETE    to all faces out of the UnbendRegion  
    sMergeStartUnbendBoundary.NonManifoldBoolean(SM_BO_INTERSECTION, pResult) ; 
  
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

  } // end STEP 2a: end imprint m_pBrep with UnbendRegion Start Boundary

  // STEP 2b: sStopTopology list containing all Edges and Vertices on the StopUnbendBoundary
  { // Imprint MergeStopBoundary on m_pBrep to gather list of UnbendRegion StopBoudnary Topology objects
    SmBrep * pStopUnbendBoundary  = GetStopUnbendBoundary() ;  // builds StopUnbendBoundary when needed
    
    // Set Merge objects for imprinting
    SmMerge sMergeStopUnbendBoundary (*cpContext, m_pBrep, pStopUnbendBoundary ) ;
    sMergeStopUnbendBoundary.SetClassifyFaces(TRUE) ;

    // imprint the PreUnbendRegion onto the target Brep
    //  1. Expect no topology changes - those are already done with the last SmMerge::NonManifoldBoolean() call.
    //  2. Adds attributes with attributeIds SM_AI_BOOLEAN_INTERSECT to all topology on the UnbendRegion Boundaries
    //                                       SM_AI_BOOLEAN_SAVE      to all faces in the UnbendRegion    
    //                                       SM_AI_BOOLEAN_DELETE    to all faces out of the UnbendRegion  
    sMergeStopUnbendBoundary.NonManifoldBoolean(SM_BO_INTERSECTION, pResult) ; 
  
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

  } // end STEP 2b: imprint m_pBrep with UnbendRegion Stop Boundary
  
  // arrive here after
  // 1. m_pBrep has been split along the bend boundary regions.
  // 2. a. sInFaces contains all Faces within the UnbendRegion.
  //    b. sStartTopology contains all Edges and Vertices on the StartUnbendBoundary.
  //    c. sStopTopology  contains all Edges and Vertices on the StopUnbendBoundary.
  
  // gather the beforeUnbend and AfterUnbend topology objects in these lists
  SmTArray<SmFace *>   sBeforeFaces,    sAfterFaces,    sThisFaces, sInFacesFromComposites ;
  SmTArray<SmEdge *>   sBeforeEdges,    sAfterEdges,    sThisEdges ;
  SmTArray<SmVertex *> sBeforeVertices, sAfterVertices, sThisVertices ;

  // STEP 3: a. Use sInFaces, SmTopologyTraverser::GetSubTopologies(), and the mark mechanism to build sInEdges and sInVertices.
  //         b. Use SmTopologyTraverser::CollectFaces(), the mark mechanim, 
  //              and sStartTopology, to build sBeforeFaces, sBeforeEdges, and sBeforeVertices,
  //              and sStopTopology, to build sAfterFaces, sAfterEdges, and sAfterVertices.
  {
    // increment the context mark used to track traversal progress
    SmNewMarkAndLock sMarkLock(GetContext(), SM_MT_ALLMARKS);
    SmMarkType eMarkType = sMarkLock.GetMarkType() ;

    // gather and mark all faces, edges, and vertices in the bend by traversing the topology graph from the inUnbend face list
    SmTArray<SmTopology*> sTopologies;
    for(ii = 0; ii < sInFaces.GetSize(); ii++)
    {
      sTopologies.Add((SmTopology*)sInFaces[ii]);
    }
    sTraverser.GetSubTopologies(sTopologies,
                                NULL,          // i/o: regions
                                NULL,          // i/o: shells
                                &sInFaces,     // i/o: faces
                                NULL,          // i/o: loops
                                &sInEdges,     // i/o: edgse
                                &sInVertices,  // i/o: vertices
                                FALSE,         // in : bReSetArrays
                                eMarkType) ;   // in : use this mark

    // iter locals
    SmTArray<SmFace *>   sEdgeFaces ;
    SmTArray<SmEdge *>   sVertexEdges ;
  
    // for two passes (once for edges, once for vertices) - set sBeforeFaces, sBeforeEdges, sBeforeVertices lists
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
                    SmFace *pFace = sEdgeFaces[kk] ;

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
      } // end iter two times, ii==0 for edges and ii== 1 for vertices - setting sBeforeFaces, sBeforeEdges, sBeforeVertices lists

    // restore the mark state so that all InUnbend topology is marked and all OutUnbend is not

    // increment the context mark used to track traversal progress
    sMarkLock.NewMark() ; 

    // mark the InUnbend geometry
    for(ii=0;ii<sInFaces.GetSize();ii++)    { sInFaces[ii]->Mark(eMarkType) ; }
    for(ii=0;ii<sInEdges.GetSize();ii++)    { sInEdges[ii]->Mark(eMarkType) ; }
    for(ii=0;ii<sInVertices.GetSize();ii++) { sInVertices[ii]->Mark(eMarkType) ; }

    // for two passes (once for edges, once for vertices) - set sAfterFaces, sAfterEdges, sAfterVertices lists
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
                    SmFace *pFace = sEdgeFaces[kk] ;

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
                    SmEdge *pVertexEdge = sVertexEdges[kk] ;

                    // skip marked edges (that are inside the bend region or already processed)
                    if(pVertexEdge->IsMarked(eMarkType))
                      { continue ; }

                    // find all the unmarked edges and vertices connected to this one
                    sTraverser.CollectWireEdges( pVertexEdge, sThisEdges, eMarkType, &sThisVertices) ;

                    // accumulate the results
                    sAfterEdges.Append(sThisEdges) ;
                    sAfterVertices.Append(sThisVertices) ;

                  }
              } // end Found an intersection Vertex with unmarked Edges branch
          } // end iter every StartTopology Object
      } // end iter two times, ii==0 for edges and ii== 1 for vertices - setting sAfterFaces, sAfterEdges, sAfterVertices lists
  } // end STEP 3: building spatially defined groupings of Topology objects

  // arrive here when all the geometry containing topology objects have been classified into the lists
  //   sInFaces,    sBeforeFaces,    sAfterFaces
  //   sInEdges,    sBeforeEdges,    sAfterEdges
  //   sInVertices, sBeforeVertices, sAfterVertices.

// Remove Composites
//   // STEP 4: next replace composite faces and edges spanning bend and unbend regions with non-Composite faces and edges.
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
//         smgfx_SetLook(1,2, 0,1,1) ; this->Draw() ; sm_GraphicsLoop() ;
// 
//         smgfx_SetLook(1,2,0,0,.5) ; for(di=0;di<sInFaces.GetSize();di++) 
//                                       { if(sInFaces[di]) sInFaces[di]->DrawUV(9,9) ; sm_GraphicsLoop() ; } 
//         smgfx_SetLook(3,4,0,0, 1) ; for(di=0;di<sInEdges.GetSize();di++) 
//                                       { if(sInEdges[di]) sInEdges[di]->Draw() ; sm_GraphicsLoop() ; } 
//         smgfx_SetLook(7,8,0,0, 1) ; for(di=0;di<sInVertices.GetSize();di++) 
//                                       { if(sInVertices[di]) sInVertices[di]->Draw() ; sm_GraphicsLoop() ; } 
// 
//         smgfx_SetLook(1,2,0,.5,0) ; for(di=0;di<sBeforeFaces.GetSize();di++) 
//                                       { if(sBeforeFaces[di]) sBeforeFaces[di]->DrawUV(9,9) ; sm_GraphicsLoop() ; } 
//         smgfx_SetLook(3,4, 0,1,0) ; for(di=0;di<sBeforeEdges.GetSize();di++) 
//                                       { if(sBeforeEdges[di]) sBeforeEdges[di]->Draw() ; sm_GraphicsLoop() ; } 
//         smgfx_SetLook(7,8, 0,1,0) ; for(di=0;di<sBeforeVertices.GetSize();di++) 
//                                       { if(sBeforeVertices[di]) sBeforeVertices[di]->Draw() ; sm_GraphicsLoop() ; } 
// 
//         smgfx_SetLook(1,2,.5,0,0) ; for(di=0;di<sAfterFaces.GetSize();di++) 
//                                       { if(sAfterFaces[di]) sAfterFaces[di]->DrawUV(9,9) ; sm_GraphicsLoop() ; } 
//         smgfx_SetLook(3,4, 1,0,0) ; for(di=0;di<sAfterEdges.GetSize();di++) 
//                                       { if(sAfterEdges[di]) sAfterEdges[di]->Draw() ; sm_GraphicsLoop() ; } 
//         smgfx_SetLook(7,8, 1,0,0) ; for(di=0;di<sAfterVertices.GetSize();di++) 
//                                       { if(sAfterVertices[di]) sAfterVertices[di]->Draw() ; sm_GraphicsLoop() ; } 
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
//         for(ii=0;ii<sBeforeEdges.GetSize();ii++) { if(sBeforeEdges[ii]->IsKindOf(SmCEdge_TYPE))
//                                                      { sCompositeBeforeEdges.AddUnique(sBeforeEdges[ii]) ; }
//                                                    else if(sBeforeEdges[ii]->IsCompositeEdge())
//                                                      { sCompositeBeforeEdges.AddUnique(sBeforeEdges[ii]->GetCompositeEdgeOwner()) ; }
//                                                  }
//         for(ii=0;ii<sAfterEdges.GetSize();ii++)  { if(sAfterEdges[ii]->IsKindOf(SmCEdge_TYPE))
//                                                      { sCompositeAfterEdges.AddUnique(sAfterEdges[ii]) ; }
//                                                    else if(sAfterEdges[ii]->IsCompositeEdge())
//                                                      { sCompositeAfterEdges.AddUnique(sAfterEdges[ii]->GetCompositeEdgeOwner()) ; }
//                                                  }
//         for(ii=0;ii<sInEdges.GetSize();ii++)     { if(sInEdges[ii]->IsKindOf(SmCEdge_TYPE))
//                                                      { sCompositeInEdges.AddUnique(sInEdges[ii]) ; }
//                                                    else if(sInEdges[ii]->IsCompositeEdge())
//                                                      { sCompositeInEdges.AddUnique(sInEdges[ii]->GetCompositeEdgeOwner()) ; }
//                                                  }
//       } // end any CEdges check
// 
//     // make sure any CFace parent is on the list with a CFace child
//     if(m_pBrep->GetNumCFaces() > 0)
//       {
//         for(ii=0;ii<sBeforeFaces.GetSize();ii++) { if(sBeforeFaces[ii]->IsKindOf(SmCFace_TYPE))
//                                                      { sCompositeBeforeFaces.AddUnique(sBeforeFaces[ii]) ; }
//                                                    else if(sBeforeFaces[ii]->IsCompositeFace())
//                                                      { sCompositeBeforeFaces.AddUnique(sBeforeFaces[ii]->GetCompositeFaceOwner()) ; }
//                                                  }
//         for(ii=0;ii<sAfterFaces.GetSize();ii++)  { if(sAfterFaces[ii]->IsKindOf(SmCFace_TYPE))
//                                                      { sCompositeAfterFaces.AddUnique(sAfterFaces[ii]) ; }
//                                                    else if(sAfterFaces[ii]->IsCompositeFace())
//                                                      { sCompositeAfterFaces.AddUnique(sAfterFaces[ii]->GetCompositeFaceOwner()) ; }
//                                                  }
//         for(ii=0;ii<sInFaces.GetSize();ii++)     { SmFace* pFace = sInFaces[ii] ;
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
//       } // end find and replace composite faces and edges that span more than 1 unbend region
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
//             SER_MSG(SM_ERR, _T("SmSpaceUnbend::DoSpaceBend() bad unbend request: geometry before the unbend connects to after the unbend")) ; 
// 
// #ifdef SM_DEBUG_CODE
//             // draw common(on boundary) Before and After faces
//             if(bDebugMe)
//               {
//                 smgfx_Erase() ;
//                 smgfx_SetLook( 1, 2, 0, 0, 1 ); if(m_pBrep) { m_pBrep->Draw( TRUE ); sm_GraphicsLoop(); }
//                 smgfx_SetLook(1,2, 0,1,1) ; this->Draw() ; sm_GraphicsLoop() ;
// 
//                 smgfx_SetLook(1,2, .5,1,0) ; for(di=0;di<sCommonFaces.GetSize();di++) { if(sCommonFaces[di]) { sCommonFaces[di]->DrawUV(3,3) ; } } sm_GraphicsLoop() ;
//                 smgfx_SetLook(4,6, .5,1,0) ; for(di=0;di<sCommonEdges.GetSize();di++) { if(sCommonEdges[di]) { sCommonEdges[di]->Draw() ; } } sm_GraphicsLoop() ;
//                 smgfx_SetLook(4,6, .5,1,0) ; for(di=0;di<sCommonVertices.GetSize();di++) { if(sCommonVertices[di]) { sCommonVertices[di]->Draw() ; } } sm_GraphicsLoop() ;
//                 sm_GraphicsLoop() ;
//               }
// #endif // SM_DEBUG_CODE
//           } // end error check
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
//         smgfx_SetLook(1,2, 0,1,1) ; this->Draw() ; sm_GraphicsLoop() ;
// 
//         // Not sure what pFace should be. pFace should be locally declared and set
//         // smgfx_SetLook(1,2,0,0,.5) ; for(di=0;di<sInFaces.GetSize();di++) 
//         //                              { if(pFace==sInFaces[di]) pFace->DrawUV(9,9) ; sm_GraphicsLoop() ; } 
//         smgfx_SetLook(3,4,0,0, 1) ; for(di=0;di<sInEdges.GetSize();di++) 
//                                       { if(pEdge==sInEdges[di]) pEdge->Draw() ; sm_GraphicsLoop() ; } 
//         smgfx_SetLook(7,8,0,0, 1) ; for(di=0;di<sInVertices.GetSize();di++) 
//                                       { if(pVertex==sInVertices[di]) pVertex->Draw() ; sm_GraphicsLoop() ; } 
// 
//         // Not sure what pFace should be. pFace should be locally declared and set
//         // smgfx_SetLook(1,2,0,.5,0) ; for(di=0;di<sBeforeFaces.GetSize();di++) 
//         //                             { if(pFace==sBeforeFaces[di]) pFace->DrawUV(9,9) ; sm_GraphicsLoop() ; } 
// 
//         smgfx_SetLook(3,4, 0,1,0) ; for(di=0;di<sBeforeEdges.GetSize();di++) 
//                                       { if(pEdge==sBeforeEdges[di]) pEdge->Draw() ; sm_GraphicsLoop() ; } 
//         smgfx_SetLook(7,8, 0,1,0) ; for(di=0;di<sBeforeVertices.GetSize();di++) 
//                                       { if(pVertex==sBeforeVertices[di]) pVertex->Draw() ; sm_GraphicsLoop() ; } 
// 
//         // Not sure what pFace should be. pFace should be locally declared and set
//         // smgfx_SetLook(1,2,.5,0,0) ; for(di=0;di<sAfterFaces.GetSize();di++) 
//         //                               { if(pFace==sAfterFaces[di]) pFace->DrawUV(9,9) ; sm_GraphicsLoop() ; } 
// 
//         smgfx_SetLook(3,4, 1,0,0) ; for(di=0;di<sAfterEdges.GetSize();di++) 
//                                       { if(pEdge==sAfterEdges[di]) pEdge->Draw() ; sm_GraphicsLoop() ; } 
//         smgfx_SetLook(7,8, 1,0,0) ; for(di=0;di<sAfterVertices.GetSize();di++) 
//                                       { if(pVertex==sAfterVertices[di]) pVertex->Draw() ; sm_GraphicsLoop() ; } 
//         sm_GraphicsLoop() ;
//       }
// #endif // SM_DEBUG_CODE
// 
//   } // end STEP 4: replace composite faces and edges spanning bend and unbend regions with non-Composite faces and edges

  // STEP 5: for all InFaces - shrink geometry. Useful later if surfaces need to be approximated 
  {
    for(ii=0;ii<sInFaces.GetSize();ii++)
      {
        SmFace* pFace = sInFaces[ii] ;

        // skip InFaces which came from a spanning composite (they've already been shrunk)
        if(sInFacesFromComposites.FindElement(pFace,lFoundIdx))
          { continue ; }

        pFace->ShrinkGeometry() ;
      }
  } // end STEP 5: Shrink sInFaces geometry

  // UnbendVolume and Before/In/After unbend transformations
  SmAxis2Placement sBeforeUnbendTransform ;
  SmAxis2Placement sInsideUnbendTransform ; 
  SmAxis2Placement sAfterUnbendTransform ;  
  SmUnbendVolume * pUnbendVolume = NULL ;
  SmObjDelete      sClean(pUnbendVolume) ;

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
        smgfx_SetLook(1,2, 0,0,1) ; if(m_pBrep) m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
        smgfx_SetLook(1,2, 0,1,1) ; this->Draw(FALSE, FALSE, TRUE) ;  sm_GraphicsLoop() ;
      
        smgfx_SetLook(4,8, 0,0,1) ; for(di=0;di<sInVertices.GetSize();di++) if(pVertex==sInVertices[di]) pVertex->Draw() ;   sm_GraphicsLoop() ;
        smgfx_SetLook(4,8, 0,0,1) ; for(di=0;di<sInEdges.GetSize();di++)    if(pEdge==sInEdges[di])      pEdge->Draw() ;      sm_GraphicsLoop() ;
        // Not sure what pFace should be. pFace should be locally declared and set
        // smgfx_SetLook(1,2,0,0,.5) ; for(di=0;di<sInFaces.GetSize();di++)    if(pFace==sInFaces[di])      pFace->DrawUV(3,3) ; sm_GraphicsLoop() ;

        smgfx_SetLook(6,10,0,1,0) ; for(di=0;di<sBeforeVertices.GetSize();di++) if(pVertex==sBeforeVertices[di]) pVertex->Draw() ; sm_GraphicsLoop() ;
        smgfx_SetLook(6,10,0,1,0) ; for(di=0;di<sBeforeEdges.GetSize();di++)    if(pEdge==sBeforeEdges[di])     pEdge->Draw() ;    sm_GraphicsLoop() ;
        // Not sure what pFace should be. pFace should be locally declared and set
        // smgfx_SetLook(1.2,.5,0,0) ; for(di=0;di<sBeforeFaces.GetSize();di++)    if(pFace==sBeforeFaces[di])     pFace->DrawUV() ;  sm_GraphicsLoop() ;

        smgfx_SetLook(8,12,1,0,0) ; for(di=0;di<sAfterVertices.GetSize();di++) if(pVertex==sAfterVertices[di]) pVertex->Draw() ; sm_GraphicsLoop() ;
        smgfx_SetLook(8,12,1,0,0) ; for(di=0;di<sAfterEdges.GetSize();di++)    if(pEdge==sAfterEdges[di])      pEdge->Draw() ;    sm_GraphicsLoop() ;
        // Not sure what pFace should be. pFace should be locally declared and set
        // smgfx_SetLook(1,2,.5,0,0) ; for(di=0;di<sAfterFaces.GetSize();di++)    if(pFace==sAfterFaces[di])      pFace->DrawUV() ;  sm_GraphicsLoop() ;

        sm_GraphicsLoop() ;
      }
#endif // SM_DEBUG_CODE

    // build the UnbendVolume for the InBend geometry
    pUnbendVolume = CreateUnbendVolume(*cpContext) ;
    sClean.ReplaceObj(pUnbendVolume) ;  

    // define the rotate and move transforms for the before, inside and after bend geometry
    sBeforeUnbendTransform = GetBeforeUnbendTransform() ;
    sInsideUnbendTransform = GetInsideUnbendTransform() ; 
    sAfterUnbendTransform  = GetAfterUnbendTransform() ;  

#ifdef SM_DEBUG_CODE                                                                                                   
    // draw the SpaceUnbend - and Unbend Volume
    if(bDebugMe)
      {
        smgfx_SetLook(1,2, 0,1,1) ; this->Draw(FALSE, FALSE, TRUE) ;  sm_GraphicsLoop() ; // bDrawBrep, bAddToUIPickList, bDrawPreUnbendRegion 
        smgfx_SetLook(1,2, 0,0,1) ; pUnbendVolume->Draw(TRUE, TRUE) ; sm_GraphicsLoop() ;
        sm_GraphicsLoop() ;
      }
#endif // SM_DEBUG_CODE
   
  } // end STEP 6: build BendVolume mapping and Before/In/After Affine Transformations
   
  // STEP 7: ModelTranformation: Bend InBendGeometry and apply AffineTransformations to Before/In/After bend geometries
  {
    // bend the InUnbend Geometry - leaves Brep in invalid state until all geometry is moved and bent
    UnbendTopology(*pUnbendVolume,  // in : Unbend mapping to apply
                   *m_pBrep,        // in : Brep containing all topology to be transformed
                   sInFaces,        // in : faces to transform
                   sInEdges,        // in : edges to transform 
                   sInVertices) ;   // in : vertices to transform 

    // transform before, inside and after geometry to align the topology pieces across the start/end bend planes 
    TransformTopology(sBeforeUnbendTransform, *m_pBrep, sBeforeFaces, sBeforeEdges, sBeforeVertices) ;
    TransformTopology(sInsideUnbendTransform, *m_pBrep, sInFaces,     sInEdges,     sInVertices) ;
    TransformTopology(sAfterUnbendTransform,  *m_pBrep, sAfterFaces,  sAfterEdges,  sAfterVertices) ;

#ifdef SM_DEBUG_CODE                                                                                                   
    // After Bend test and draw the transformed model
    if(bDebugMe)
      {
        SmTArray<SmSurface*> sSurfaces ;
        SmTArray<SmSrfInVolume*> sSrfInVolumes ;
        m_pBrep->GetSurfaces(sSurfaces) ;
        for(di=0;di<sSurfaces.GetSize();di++) { if(sSurfaces[di]->IsKindOf(SmSrfInVolume_TYPE)) { sSrfInVolumes.Add((SmSrfInVolume*)sSurfaces[di]) ; }}
        for(di=0;di<sSrfInVolumes.GetSize();di++)
          { SmObject * pOwner     = sSrfInVolumes[di]->GetOwner() ;
            SmObject * pBaseOwner = sSrfInVolumes[di]->GetSurface()->GetOwner() ; 
            SmBoolean  bOk = pBaseOwner == (sSrfInVolumes[di]) && (pOwner->IsKindOf(SmFace_TYPE)) ;
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
  // when asked
  if(bDoApproximations)
    {
#ifdef SM_DEBUG_CODE                                                                                                   
        // PreApproximate target Brep AssertValid
        if(bDebugMe)
          { 
            if(m_pBrep) m_pBrep->Dump(_T("Brep BEFORE ApproximateWithBSplines - "),SM_BD_GEOM_TYPES) ;
            SM_ASSERT_VALID(m_pBrep) ;

            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1) ; if(m_pBrep) m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
            smgfx_SetLook(1,2, 0,1,1) ; this->Draw(FALSE, FALSE, TRUE) ;  sm_GraphicsLoop() ;
      
            smgfx_SetLook(4,8, 0,0,1) ; for(di=0;di<sInVertices.GetSize();di++) { if(pVertex==sInVertices[di]) pVertex->Draw() ;   sm_GraphicsLoop() ;  }
            smgfx_SetLook(4,8, 0,0,1) ; for(di=0;di<sInEdges.GetSize();di++)    { if(pEdge==sInEdges[di])      pEdge->Draw() ;      sm_GraphicsLoop() ; }
            // Not sure what pFace should be. pFace should be locally declared and set
            // smgfx_SetLook(1,2,0,0,.5) ; for(di=0;di<sInFaces.GetSize();di++)    { if(pFace==sInFaces[di])      pFace->DrawUV(3,3) ; sm_GraphicsLoop() ; }

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
            smgfx_SetLook(1,2, 0,0,1) ; if(m_pBrep) m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
            smgfx_SetLook(1,2, 0,1,1) ; this->Draw(FALSE, FALSE, TRUE) ;  sm_GraphicsLoop() ;
      
            smgfx_SetLook(4,8, 0,0,1) ; for(di=0;di<sInVertices.GetSize();di++) { if(pVertex==sInVertices[di]) pVertex->Draw() ;   sm_GraphicsLoop() ;  }
            smgfx_SetLook(4,8, 0,0,1) ; for(di=0;di<sInEdges.GetSize();di++)    { if(pEdge==sInEdges[di])      pEdge->Draw() ;      sm_GraphicsLoop() ; }
            // Not sure what pFace should be. pFace should be locally declared and set
            // smgfx_SetLook(1,2,0,0,.5) ; for(di=0;di<sInFaces.GetSize();di++)    { if(pFace==sInFaces[di])      pFace->DrawUV() ; sm_GraphicsLoop() ; }

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

      smgfx_SetLook(4,8, 0,0,1) ; for(di=0;di<sInVertices.GetSize();di++) if(sInVertices[di]) sInVertices[di]->Draw() ;   sm_GraphicsLoop() ;
      smgfx_SetLook(4,8, 0,0,1) ; for(di=0;di<sInEdges.GetSize();di++)    if(sInEdges[di])    sInEdges[di]->Draw() ;      sm_GraphicsLoop() ;
      smgfx_SetLook(1,2,0,0,.5) ; for(di=0;di<sInFaces.GetSize();di++)    if(sInFaces[di])    sInFaces[di]->DrawUV(3,3) ; sm_GraphicsLoop() ;

      smgfx_SetLook(4,5, 0,1,0) ; for(di=0;di<sStartTopology.GetSize();di++) 
                                   { if(sStartTopology[di]->IsKindOf(SmVertex_TYPE)) { ((SmVertex *)sStartTopology[di])->Draw() ; sm_GraphicsLoop() ; }
                                     if(sStartTopology[di]->IsKindOf(SmEdge_TYPE))   { ((SmEdge *)  sStartTopology[di])->Draw() ; sm_GraphicsLoop() ; }
                                     if(sStartTopology[di]->IsKindOf(SmFace_TYPE))   { ((SmFace *)  sStartTopology[di])->DrawUV() ; sm_GraphicsLoop() ; }
                                   }

      smgfx_SetLook(4,5, 0,1,1) ; for(di=0;di<sBeforeVertices.GetSize();di++) if(sBeforeVertices[di]) sBeforeVertices[di]->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 0,1,1) ; for(di=0;di<sBeforeEdges.GetSize();di++)    if(sBeforeEdges[di])    sBeforeEdges[di]->Draw() ;    sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; for(di=0;di<sBeforeFaces.GetSize();di++)    if(sBeforeFaces[di])    sBeforeFaces[di]->DrawUV() ;  sm_GraphicsLoop() ;

      smgfx_SetLook(4,5, 1,0,0) ; for(di=0;di<sStopTopology.GetSize();di++) 
                                   { if(sStopTopology[di]->IsKindOf(SmVertex_TYPE)) { ((SmVertex *)sStopTopology[di])->Draw() ; sm_GraphicsLoop() ; }
                                     if(sStopTopology[di]->IsKindOf(SmEdge_TYPE))   { ((SmEdge *)  sStopTopology[di])->Draw() ; sm_GraphicsLoop() ; }
                                     if(sStopTopology[di]->IsKindOf(SmFace_TYPE))   { ((SmFace *)  sStopTopology[di])->DrawUV() ; sm_GraphicsLoop() ; }
                                   }

      smgfx_SetLook(4,5, 1,0,1) ; for(di=0;di<sAfterVertices.GetSize();di++) if(sAfterVertices[di]) sAfterVertices[di]->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 1,0,1) ; for(di=0;di<sAfterEdges.GetSize();di++)    if(sAfterEdges[di])    sAfterEdges[di]->Draw() ;    sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1) ; for(di=0;di<sAfterFaces.GetSize();di++)    if(sAfterFaces[di])    sAfterFaces[di]->DrawUV() ;  sm_GraphicsLoop() ;

      smgfx_SetLook(1,2, 0,1,1) ; this->Draw(FALSE, FALSE, TRUE) ;  sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;

      SM_DUMP_AND_ASSERT_VALID(this) ;
      smgfx_SetLook(1,2, 0,0,1) ; if(m_pBrep) m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return(SM_SUCCESS) ; 

} // end SmSpaceUnbend::DoSpaceUnbend

/*******************************************************************//**
PURPOSE: Build a Brep marking the preUnbend region

NOTES:  The UnbendDomain region is a cylindrical box defined with the faces
          1. cylindrical face 
                 CenterLine:[UnbendAxis], Radius:[UnbendRadius]
                 StartAngle:[-UnbendAngle/2], StopAngle:[UnBendAngle/2]
                 LengtInterval:[UnbendAxisPoint + UnbendInterval.Evaluate(s) * NeutralAxis for s from 0 to 1] 
          2. cylindrical face
                 CenterLine:[UnbendAxis], Radius:[UnbendRadius+Thickness]
                 StartAngle:[-UnbendAngle/2], StopAngle:[UnBendAngle/2]
                 LengtInterval:[UnbendAxisPoint + UnbendInterval.Evaluate(s) * NeutralAxis for s from 0 to 1] 
          3. Plane face
          4. Plane face
          5. Plane face
          6. Plane face
***********************************************************************/
SmBrep * SmSpaceUnbend::GetUnbendDomain() const
{
  // low work - PreUnbendRegion already built
  if(m_pUnbendDomain != NULL)
    { return( m_pUnbendDomain ) ; }

  // an increase amount to avoid coincident geometry problems
  double dDelta   = 0.1 ;

  // Get PreUnbendRegion rectilinear position and orientation
  SmVector3d       sX    = GetUnbendDirection() ;   
  SmVector3d       sY    = GetUnbendBinormal() ; 
  SmVector3d       sZ    = GetUnbendAxis() ;
  SmPoint3d        sOrig = GetUnbendAxisPoint() - dDelta * sZ ;  // increased by dDelta to avoid coincident geometry
  SmAxis2Placement sRefFrame(sOrig, sX, sY) ; 

  // Build empty PreUnbend Breps (building internal caches is not C++ const - but is for SMLib semantics)  (SmBrep *)m_pUnbendDomain        = new (*GetContext()) SmBrep() ; 
  SmSpaceUnbend * pThisSpaceUnbend = (SmSpaceUnbend*) this ;

  pThisSpaceUnbend->m_pUnbendDomain = new (*GetContext()) SmBrep() ;
  if(m_pStartUnbendBoundary == NULL) { pThisSpaceUnbend->m_pStartUnbendBoundary = new (*GetContext()) SmBrep() ; }
  if(m_pStopUnbendBoundary  == NULL) { pThisSpaceUnbend->m_pStopUnbendBoundary  = new (*GetContext()) SmBrep() ; }
  
  // define SmPrimitiveCreation object
  SmPrimitiveCreation sPreUnbendPC(m_pUnbendDomain->GetInfiniteRegion()) ; 
                                  
  // insert a Box into the PreUnbend infinite region  
  if(SM_SUCCESS != sPreUnbendPC.CreateCylindricalBox
    (GetUnbendSegmentLength() + 2 * dDelta, // in : Z Axis length starting at RefFram Origin, increased by dDelta to avoid coincident geometry
     GetInsideRadius() - dDelta,            // in : Inside Radius,                            increased by dDelta to avoid coincident geometry
     GetOutsideRadius() + dDelta,           // in : Outside Radius,                           increased by dDelta to avoid coincident geometry
     -GetUnbendAngDeg()/2.0,                // in : Start AngDeg from X Axis in YX plane,     left alone to properly place the Unbend/RigidRotation boundary
      GetUnbendAngDeg()/2.0,                // in : End AngDeg from X Axis in YX plane,       left alone to properly place the Unbend/RigidRotation boundary
     sRefFrame))                            // in : Z Axis  = Cylinder centers                                                           // orig already moved by -dDelta*sZ to avoid coincident geometry
    { return(NULL) ; }                      //      Origin  = Bot of CylBox                                                              
                                            //      XY Axes = Plane of Start and End angles measured from X Axis                                                              // in : length measured along Z Axis starting at RefFram Origin, [greater than zero]            

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      ULONG di ;
      SmTArray<SmFace*>    sFaces ;    m_pUnbendDomain->GetFaces(sFaces) ; 

      SM_DUMP_AND_ASSERT_VALID(m_pUnbendDomain) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(m_pUnbendDomain) m_pUnbendDomain->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,3, 0,0,0) ; for(di=0;di<sFaces.GetSize();di++) 
                                    { if(sFaces[di]) sFaces[di]->DrawUV(7,7,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

 // select start and end faces of the cylinderbox 
 SmTArray<SmFace*> sFaces ;
 m_pUnbendDomain->GetFaces(sFaces) ;
 SmFace *pStartFace = sFaces[2] ;  // gwc - These indices were found by trial and error - not geometrically.
 SmFace *pStopFace  = sFaces[3] ;  // they will depend on the behavior of CreateBox to remain consistent.
                                   // that's not a good coding practice, but is good enough for now.
                                   // gwc: could replace this hack with a search based on surface type and surface normal orientation
 
 // build the begin boundary sheet model
 SmTArray<SmFace *> sStartFaces(1,NULL,1) ; sStartFaces[0] = pStartFace ; 
 SmTArray<SmFace *> sStopFaces(1,NULL,1) ;  sStopFaces[0]  = pStopFace ; 
 m_pUnbendDomain->CopyFaces(sStartFaces, m_pStartUnbendBoundary, NULL, FALSE, NULL) ;
 m_pUnbendDomain->CopyFaces(sStopFaces,  m_pStopUnbendBoundary, NULL, FALSE, NULL) ;
 
  // all done
  return(m_pUnbendDomain) ; 

} // end SmSpaceUnbend::GetUnbendDomain

/*******************************************************************//**
PURPOSE: Build and return the SmAxis2Placement object representing
         the rigid body transformation applied to the BeforeUnbend geometry.

NOTES:  The bend is defined as the transformation from 
        the point on the UnbendNeutral Axis at the UnbendRegion StartPlane
        to the point on the beginning of the UnbendNeutral after bend arc.
***********************************************************************/
SmAxis2Placement SmSpaceUnbend::GetBeforeUnbendTransform 
 (SmUnbendOrientTYPE eOptOrientType)  // NotUsed: in : optional OrientType specification, default:[SM_UO_ORIENT_UNKNOWN=Use m_eBendOrient val]
 const
{
  SM_REF1(eOptOrientType) ;
  // UNKNOWN_UNBENDING -> treat as SM_UO_CENTER_UNBENDING
  SmUnbendOrientTYPE eUnbendOrient =   m_eUnbendOrient != SM_UO_ORIENT_UNKNOWN
                                     ? m_eUnbendOrient
                                     : SM_UO_CENTER_UNBENDING ;

  // low work - FIXED_START_UNBENDING = BeforeUnbend geometry does not rotate
  if(eUnbendOrient == SM_UO_FIXED_BEFORE_UNBENDING)
    {
      SmAxis2Placement sBeforeUnbendTransform ;  // identity transform
      return(sBeforeUnbendTransform) ;
    }

  // arrive here for CENTER_UNBENDING and FIXED_END_UNBENDING

  // define From Point, XAxis, and YAxis - point on neutral arc at the StartUnbend plane
  SmVector3d sFromOrigin      = GetNeutralBegPoint() ;
  SmVector3d sFromXAxis       = GetUnbendDirection().RotateVecAboutAxis(GetUnbendAxis(), SM_DEG2RAD(-GetUnbendAngDeg()/2.0)) ;
  SmVector3d sFromYAxis       = GetUnbendBinormal(). RotateVecAboutAxis(GetUnbendAxis(), SM_DEG2RAD(-GetUnbendAngDeg()/2.0)) ;

  // define To Point, XAxis, and YAxis - to point on neutral axis at StartUnbend plane's final pos and orientation
  SmVector3d sToXAxis, sToYAxis, sToOrigin ;

  // CENTER_UNBENDING = BeforeUnbend geometry InsideUnbend geometry rotates by -1/2 bend angle about Unbend center 
  //                    then translated 1/2 PostBendWidth in the now rotated UnbendBinormal() (YAxis) dir
  if(eUnbendOrient == SM_UO_CENTER_UNBENDING)
    {
      // define to Point, XAxis, and YAxis - Pre Unbend coordinate system rotated then translated
      double dTransDist = GetPostUnbendWidth()/2 ;
      sToXAxis          = GetUnbendDirection() ;  
      sToYAxis          = GetUnbendBinormal() ;
      sToOrigin         = GetNeutralMidPoint() - dTransDist * sToYAxis ; 
    }

  // FIXED_END_UNBENDING = BeforeUnbend geometry InsideUnbend geometry rotates by -bend angle about Unbend center 
  //                       then translated PostBendWidth in the now rotated UnbendBinormal() (YAxis) dir
  else if(eUnbendOrient == SM_UO_FIXED_AFTER_UNBENDING)
    {
      // define to Point, XAxis, and YAxis - Pre Unbend coordinate system rotated then translated
      double dTransDist = GetPostUnbendWidth() ;
      sToXAxis          = GetUnbendDirection().RotateVecAboutAxis(GetUnbendAxis(), SM_DEG2RAD(GetUnbendAngDeg()/2.0)) ;
      sToYAxis          = GetUnbendBinormal(). RotateVecAboutAxis(GetUnbendAxis(), SM_DEG2RAD(GetUnbendAngDeg()/2.0)) ;
      sToOrigin         = GetNeutralEndPoint() - dTransDist * sToYAxis ;
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
  SmAxis2Placement sBeforeUnbendTransform(sFromOrigin, sFromXAxis, sFromYAxis,
                                          sToOrigin,   sToXAxis,   sToYAxis) ;

  // all done
  return( sBeforeUnbendTransform ) ;

} // end SmSpaceUnbend::GetBeforeUnbendTransform

/*******************************************************************//**
PURPOSE: Build and return the SmAxis2Placement object representing
         the rigid body transformation applied to the InsideUnbend geometry
         After its been bent.

NOTES:  The bend is defined as the transformation from 
   SM_UO_FIXED_BEFORE_UNBENDING = the point on the UnbendNeutral Axis at the UnbendRegion StartPlane after its been bent
   SM_UO_CENTER_UNBENDING       = the point on the UnbendNeutral Axis at the UnbendRegion CenterPlane
   SM_UO_FIXED_AFTER_UNBENDING  = the point on the UnbendNeutral Axis at the UnbendRegion EndPlane after its been bent 
        to its position after bending.
***********************************************************************/
SmAxis2Placement SmSpaceUnbend::GetInsideUnbendTransform 
 (SmUnbendOrientTYPE eOptOrientType)  // NotUsed: in : optional OrientType specification, default:[SM_UO_ORIENT_UNKNOWN=Use m_eBendOrient val]
 const
{ 
  SM_REF1(eOptOrientType) ;
  // UNKNOWN_UNBENDING -> treat as SM_UO_CENTER_UNBENDING
  SmUnbendOrientTYPE eUnbendOrient =   m_eUnbendOrient != SM_UO_ORIENT_UNKNOWN
                                     ? m_eUnbendOrient
                                     : SM_UO_CENTER_UNBENDING ;

  // low work - Center_UNBENDING = InsideUnbend geometry does not rotate
  if(eUnbendOrient == SM_UO_CENTER_UNBENDING)
    {
      SmAxis2Placement sBeforeUnbendTransform ;  // identity transform
      return(sBeforeUnbendTransform) ;
    }

  // arrive here for FIXED_START_UNBENDING and FIXED_END_UNBENDING
  SmVector3d sFromXAxis  = GetUnbendDirection() ;
  SmVector3d sFromYAxis  = GetUnbendBinormal() ;
  SmVector3d sFromOrigin = GetNeutralMidPoint() ;
  SmVector3d sToXAxis, sToYAxis, sToOrigin ;

  // FIXED_BEFORE_UNBENDING = InsideUnbend geometry InsideUnbend geometry rotates by -1/2 bend angle about Unbend center 
  //                          then translated 1/2 PostBendWidth in the now rotated UnbendBinormal() (YAxis) dir
  if(eUnbendOrient == SM_UO_FIXED_BEFORE_UNBENDING)
    {
      // define to Point, XAxis, and YAxis - Pre Unbend coordinate system rotated then translated
      double dRotRad    = -SM_DEG2RAD(m_dUnbendAngDeg/2.0) ;       // note: half angle
      double dTransDist =  GetPostUnbendWidth()/2 ;
      sToXAxis          =  sFromXAxis.RotateVecAboutAxis(GetUnbendAxis(), dRotRad) ; 
      sToYAxis          =  sFromYAxis.RotateVecAboutAxis(GetUnbendAxis(), dRotRad) ;
      sToOrigin         =  GetNeutralBegPoint() + dTransDist * sToYAxis ; 
    }

  // FIXED_AFTER_UNBENDING = InsideUnbend geometry rotates by 1/2 bend angle about Unbend center 
  //                         then translated -1/2 PostBendWidth in the now rotated UnbendBinormal() (YAxis) dir
  else if(eUnbendOrient == SM_UO_FIXED_AFTER_UNBENDING)
    {
      // define to Point, XAxis, and YAxis - Pre Unbend coordinate system rotated then translated
      double dRotRad    =  SM_DEG2RAD(m_dUnbendAngDeg/2.0) ;       // note: half angle
      double dTransDist =  GetPostUnbendWidth()/2 ;
      sToXAxis          =  sFromXAxis.RotateVecAboutAxis(GetUnbendAxis(), dRotRad) ; 
      sToYAxis          =  sFromYAxis.RotateVecAboutAxis(GetUnbendAxis(), dRotRad) ;
      sToOrigin         =  GetNeutralEndPoint() - dTransDist * sToYAxis ; 
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
  SmAxis2Placement sInsideUnbendTransform( sFromOrigin, sFromXAxis, sFromYAxis,
                                           sToOrigin,   sToXAxis,   sToYAxis) ;

  // all done
  return( sInsideUnbendTransform ) ;

} // end SmSpaceUnbend::GetInsideUnbendTransform

/*******************************************************************//**
PURPOSE: Build and return the SmAxis2Placement object representing
         the rigid body transformation applied to the AfterUnbend geometry.

NOTES:  The bend is defined as the transformation from 
        the point on the UnbendNeutral Axis at the UnbendRegion StopPlane
        to the point on the end of the UnbendNeutral after bend arc.
***********************************************************************/
SmAxis2Placement SmSpaceUnbend::GetAfterUnbendTransform 
 (SmUnbendOrientTYPE eOptOrientType)  // NotUsed: in : optional OrientType specification, default:[SM_UO_ORIENT_UNKNOWN=Use m_eBendOrient val]
 const
{ 
  SM_REF1(eOptOrientType) ;
  // UNKNOWN_UNBENDING -> treat as SM_UO_CENTER_UNBENDING
  SmUnbendOrientTYPE eUnbendOrient =   m_eUnbendOrient != SM_UO_ORIENT_UNKNOWN
                                     ? m_eUnbendOrient
                                     : SM_UO_CENTER_UNBENDING ;

  // low work - FIXED_START_UNBENDING = BeforeUnbend geometry does not rotate
  if(eUnbendOrient == SM_UO_FIXED_AFTER_UNBENDING)
    {
      SmAxis2Placement sBeforeUnbendTransform ;  // idnentity transform
      return(sBeforeUnbendTransform) ;
    }

  // define From Point, XAxis, and YAxis - point on neutral arc at the StartUnbend plane
  SmVector3d sFromOrigin      = GetNeutralEndPoint() ;
  SmVector3d sFromXAxis       = GetUnbendDirection().RotateVecAboutAxis(GetUnbendAxis(), SM_DEG2RAD(GetUnbendAngDeg()/2.0)) ;
  SmVector3d sFromYAxis       = GetUnbendBinormal(). RotateVecAboutAxis(GetUnbendAxis(), SM_DEG2RAD(GetUnbendAngDeg()/2.0)) ;

  // define To Point, XAxis, and YAxis - to point on neutral axis at StartUnbend plane's final pos and orientation
  SmVector3d sToXAxis, sToYAxis, sToOrigin ;

  // CENTER_UNBENDING = AfterUnbend geometry InsideUnbend geometry rotates by 1/2 bend angle about Unbend center 
  //                    then translated -1/2 PostBendWidth in the now rotated UnbendBinormal() (YAxis) dir
  if(eUnbendOrient == SM_UO_CENTER_UNBENDING)
    {
      // define to Point, XAxis, and YAxis - Pre Unbend coordinate system rotated then translated
      double dTransDist = GetPostUnbendWidth()/2 ;
      sToXAxis          = GetUnbendDirection() ;  
      sToYAxis          = GetUnbendBinormal() ;
      sToOrigin         = GetNeutralMidPoint() + dTransDist * sToYAxis ; 
    }

  // FIXED_START_UNBENDING = AfterUnbend geometry InsideUnbend geometry rotates by bend angle about Unbend center 
  //                        then translated -PostBendWidth in the now rotated UnbendBinormal() (YAxis) dir
  else if(eUnbendOrient == SM_UO_FIXED_BEFORE_UNBENDING)
    {
      // define to Point, XAxis, and YAxis - Pre Unbend coordinate system rotated then translated
      double dTransDist = GetPostUnbendWidth() ;
      sToXAxis          = GetUnbendDirection().RotateVecAboutAxis(GetUnbendAxis(), SM_DEG2RAD(-GetUnbendAngDeg()/2.0)) ;
      sToYAxis          = GetUnbendBinormal(). RotateVecAboutAxis(GetUnbendAxis(), SM_DEG2RAD(-GetUnbendAngDeg()/2.0)) ;
      sToOrigin         = GetNeutralBegPoint() + dTransDist * sToYAxis ;
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
  SmAxis2Placement sAfterUnbendTransform( sFromOrigin, sFromXAxis, sFromYAxis,
                                          sToOrigin,   sToXAxis,   sToYAxis) ;

  // all done
  return( sAfterUnbendTransform ) ;

} // end SmSpaceUnbend::GetAfterUnbendTransform

/*******************************************************************//**
PURPOSE: Internal unit-test of the Unbend Transforms

RETURNS: TRUE  = okay
         FALSE = problems

NOTES: Verify that the transforms are C1 continuous across the
       Before/Inside and Inside/After boundaries
***********************************************************************/
SmBoolean SmSpaceUnbend::TestUnbendTransforms() const
{
  // locals
  ULONG ii, jj ;

  // return
  SmBoolean bRtn = TRUE ;                          // TRUE  = okay
                                                   // FALSE = a non C1 point was found
  double    dThisGap = 0.0,       dMaxGap       = 0.0 ;  // max distance between projected point pairs
  double    dThisAngRad = 0.0,    dMaxAngRad    = 0.0 ;  // max angRad between projected vector pairs
  double    dThisVecMagGap = 0.0, dMaxVecMagGap = 0.0 ;  // max VecMag differences between projected vector pairs

  // SpaceUnbend geometry and PostUnbend transform objects
  SmUnbendVolume   * pUnbendVolume        = CreateUnbendVolume(*GetContext()) ;
  SmBrep           * pStartUnbendBoundary = GetStartUnbendBoundary() ;
  SmBrep           * pStopUnbendBoundary  = GetStopUnbendBoundary() ;
  SmAxis2Placement   sBeforeTransform ;
  SmAxis2Placement   sInsideTransform ;
  SmAxis2Placement   sAfterTransform ;

  // UnbendBoundaries Vertices
  SmTArray<SmFace *>   sStartBoundaryFaces, sStopBoundaryFaces ;
  SmTArray<SmVertex *> sStartVertices, sStopVertices ;
  pStartUnbendBoundary->GetFaces(sStartBoundaryFaces) ; SM_ASSERT_MSG(sStartBoundaryFaces.GetSize() == 1, _T("SmSpaceUnbend::TestUnbendTransforms - got an unexpected number of Boundary Faces")) ;
  pStopUnbendBoundary->GetFaces(sStopBoundaryFaces) ;   SM_ASSERT_MSG(sStopBoundaryFaces.GetSize() == 1, _T("SmSpaceUnbend::TestUnbendTransforms - got an unexpected number of Boundary Faces")) ;
  sStartBoundaryFaces[0]->GetVertices(sStartVertices) ; SM_ASSERT_MSG(sStartVertices.GetSize() == 4, _T("SmSpaceUnbend::TestUnbendTransforms - got an unexpected number of Boundary Vertices")) ;
  sStopBoundaryFaces[0]->GetVertices(sStopVertices) ;   SM_ASSERT_MSG(sStopVertices.GetSize() == 4, _T("SmSpaceUnbend::TestUnbendTransforms - got an unexpected number of Boundary Vertices")) ;

  // Sample points on the space bend boundaries used for PostTransform continuity checks
  SmPoint3d  sBeforePrePoint,  sBeforePostPoint,  sBeforePreDir,  sBeforePostDir ;
  SmPoint3d  sInsidePrePoint0, sInsidePostPoint0, sInsidePreDir0, sInsidePostDir0 ;
  SmPoint3d  sInsidePrePoint1, sInsidePostPoint1, sInsidePreDir1, sInsidePostDir1 ;
  SmPoint3d  sAfterPrePoint,   sAfterPostPoint,   sAfterPreDir,   sAfterPostDir ;
  SmVector3d sPD[2] ;  // output array for MapDirectionalDerivs() calls

  double     dAngRad         = SM_DEG2RAD(GetUnbendAngDeg()) / 2.0 ;
  SmVector3d sUnbendBinormal = GetUnbendBinormal() ;
  SmVector3d sUnbendZAxis    = GetUnbendAxis() ; 

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw SpaceUnbend->Brep, Start/Stop UnbendBoundaries, UnbendDomain, Boundary Vertices
  if(bDebugMe)
    {
      ULONG di ; 

      smgfx_Erase() ;
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(GetBrep()) { GetBrep()->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(1,2, 0,1,1) ; this->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook( 2, 3, 0, 1, 0 ); if(pStartUnbendBoundary) { pStartUnbendBoundary->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook( 2, 3, 1, 0, 0 ); if(pStopUnbendBoundary) { pStopUnbendBoundary->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook( 2, 3, 0, 0, 0 ); if(GetUnbendDomain()) { GetUnbendDomain()->Draw( TRUE ); sm_GraphicsLoop(); }
      smgfx_SetLook(5,6, 0,1,0) ; for(di=0;di<sStartVertices.GetSize();di++) sStartVertices[di]->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,0,0) ; for(di=0;di<sStopVertices.GetSize();di++) sStopVertices[di]->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // note: SmSpaceUnbends partition space into 3 domains divided by the
  //       StartUnbendBoundary() and the StopUnbendBoundary() Brep Shapes.
  //       Any points on those boundaries should project with C1 continuity 
  //       through the tansformations on either side of the boundary
  //       (an appropriate pair of UnbendVolume and SmAxis2Placement mappings) 
  
  // for the 3 possible SpaceUnbend orient cases
  for(ii=0;ii<3;ii++)
    {  
      SmUnbendOrientTYPE eOrientType =   (ii == 0) ? SM_UO_FIXED_BEFORE_UNBENDING 
                                       : (ii == 1) ? SM_UO_CENTER_UNBENDING      
                                                   : SM_UO_FIXED_AFTER_UNBENDING ;

      // PostUnbend Transforms for this OrientType
      sBeforeTransform = GetBeforeUnbendTransform(eOrientType) ;
      sInsideTransform = GetInsideUnbendTransform(eOrientType) ;
      sAfterTransform  = GetAfterUnbendTransform(eOrientType) ;
                                                 
      // for every Start/Stop BoundaryVertex  (number of StartVertices == number of Stop Vertices)
      for(jj=0;jj<sStartVertices.GetSize();jj++)
        {
          // startBoundaryPoint to project once through sBeforeTransform and once through VolumeMap/sInsideTransform
          sBeforePrePoint  = sStartVertices[jj]->GetPoint() ;  
          sInsidePrePoint0 = sStartVertices[jj]->GetPoint() ; 

          sBeforePreDir    = sUnbendBinormal.RotateVecAboutAxis(sUnbendZAxis, -dAngRad) ;
          sInsidePreDir0   = sBeforePreDir ;

          // startBoundaryPoint to project once through sAfterTransform and once through VolumeMap/sInsideTransform
          sInsidePrePoint1 = sStopVertices[jj]->GetPoint() ;  
          sAfterPrePoint   = sStopVertices[jj]->GetPoint() ;   

          sAfterPreDir     = sUnbendBinormal.RotateVecAboutAxis(sUnbendZAxis, +dAngRad) ;
          sInsidePreDir1   = sAfterPreDir ;

          // Map Before Inside point and dir from Unbend InSpace to OutSpace ;
            // pUnbendVolume->MapPoint (sInsidePrePoint0, sInsidePostPoint0) ;
            // pUnbendVolume->MapVector(sInsidePrePoint0, sInsidePreDir0, sInsidePostDir0) ;
          pUnbendVolume->MapDirectionalDerivs(sInsidePrePoint0, sInsidePreDir0, 1, sPD) ; 
          sInsidePostPoint0 = sPD[0] ;
          sInsidePostDir0   = sPD[1] ;   

          // Map After Inside point and dir from Unbend InSpace to OutSpace ;
            // pUnbendVolume->MapPoint(sInsidePrePoint1, sInsidePostPoint1) ;
            // pUnbendVolume->MapVector(sInsidePrePoint1, sInsidePreDir1, sInsidePostDir1) ;   
          pUnbendVolume->MapDirectionalDerivs(sInsidePrePoint1, sInsidePreDir1, 1, sPD) ; 
          sInsidePostPoint1 = sPD[0] ;
          sInsidePostDir1   = sPD[1] ;   

#ifdef SM_DEBUG_CODE
          // draw SpaceBend->Brep, Start/Stop BendBoundaries, BendDomain, Boundary Vertices
          if(bDebugMe)
            {
              ULONG di ; 

              smgfx_Erase() ;
              smgfx_SetLook( 1, 2, 0, 0, 1 ); if(GetBrep()) { GetBrep()->Draw( TRUE ); sm_GraphicsLoop(); }
              smgfx_SetLook(1,2, 0,1,1) ; this->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook( 2, 3, 0, 1, 0 ); if(pStartUnbendBoundary) { pStartUnbendBoundary->Draw( TRUE ); sm_GraphicsLoop(); }
              smgfx_SetLook( 2, 3, 1, 0, 0 ); if(pStopUnbendBoundary) { pStopUnbendBoundary->Draw( TRUE ); sm_GraphicsLoop(); }
              smgfx_SetLook( 2, 3, 0, 0, 0 ); if(GetUnbendDomain()) { GetUnbendDomain()->Draw( TRUE ); sm_GraphicsLoop(); }
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

          // next: apply PostUnbend transformations

          // BeforeTransform applied directly to beforeBoundary point
          sBeforeTransform.TransformPoint (sBeforePrePoint, sBeforePostPoint) ; 
          sBeforeTransform.TransformVector(sBeforePreDir,   sBeforePostDir) ; 

          // InsideTransform applied to BeforeBoundary point after UnbendVolume transform
          sInsideTransform.TransformPoint (sInsidePostPoint0, sInsidePostPoint0) ; 
          sInsideTransform.TransformVector(sInsidePostDir0,   sInsidePostDir0) ;  
          
          // InsideTransform applied to AfterBoundary point after UnbendVolume transform
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
              smgfx_SetLook( 1, 2, 0, 0, 1 ); if(GetBrep()) { GetBrep()->Draw( TRUE ); sm_GraphicsLoop(); }
              smgfx_SetLook(1,2, 0,1,1) ; this->Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook( 2, 3, 0, 1, 0 ); if(pStartUnbendBoundary) { pStartUnbendBoundary->Draw( TRUE ); sm_GraphicsLoop(); }
              smgfx_SetLook( 2, 3, 1, 0, 0 ); if(pStopUnbendBoundary) { pStopUnbendBoundary->Draw( TRUE ); sm_GraphicsLoop(); }
              smgfx_SetLook( 2, 3, 0, 0, 0 ); if(GetUnbendDomain()) { GetUnbendDomain()->Draw( TRUE ); sm_GraphicsLoop(); }
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

          // check for C1 continuity
          sBeforePostDir.AngleBetween(sInsidePostDir0, dThisAngRad) ;
          dThisVecMagGap = smos_Fabs(sBeforePostDir.Length() - sInsidePostDir0.Length()) ; 
          if(dMaxAngRad    < dThisAngRad   ) { dMaxAngRad    = dThisAngRad    ; }
          if(dMaxVecMagGap < dThisVecMagGap) { dMaxVecMagGap = dThisVecMagGap ; }

        } // end iter every Start/StopVertices checking mapping continuity
    } // end iter ii, all possible SpaceUnbend orient cases
        
  // arrive here after computing dMaxGap, dMaxAngRad, dMaxVecMagGap for all sample points

  // make C0 and C1 continuity checks
  bRtn &= dMaxGap       < SM_EFF_ZERO ;        // C0 continuity
  bRtn &= dMaxAngRad    < SM_EFF_ZERO_RAD ;    // G1 continuity
  //SmBoolean bC1Continuity = dMaxVecMagGap < SM_EFF_ZERO ;  // C1 continuity - This should be false (it's a G1 Continuity transform)


  // all done
  return(bRtn) ;

} // end SmSpaceUnbend::TestUnbendTransforms

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
SmStatus SmSpaceUnbend::TransformTopology
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
// Remove Composites
//  SmTArray<SmSurface*> sSurfaces ;
//  SmTArray<SmCurve*>   sCurves ;

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

    } // end iter every vertex

  // curves

// Remove Composites - added block from block below
  // for every edge->curve
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
    } // end iter every curve

// Remove Composites
// // watch out for CEdges - don't apply tranformation to one curve multiple times
//   for(ii=0;ii<crEdges.GetSize();ii++)
//     {
//       SmEdge  * pEdge  = crEdges[ii] ;
//       SmCurve * pCurve = pEdge->GetCurve() ;
// 
//       if(pEdge->IsKindOf(SmCEdge_TYPE)) { sCurves.AddUnique(pCurve) ; }
//       else                              { sCurves.Add(pCurve) ; }
//     }
// 
//   // for every curve - transform it
//   for(ii=0;ii<sCurves.GetSize();ii++) 
//     {
//       SmCurve * pCurve = sCurves[ii] ; NER(pCurve) ;
//       SmEdge  * pEdge = (SmEdge*)pCurve->GetEdge() ; // Is either edge or cedge
// 
//       pEdge->Notify(SM_NO_PRE_EDIT, pEdge, this, NULL) ;
//       SER( pCurve->Transform(crRotateNMove) ) ;
// 
// #ifdef SM_DEBUG_CODE
//       // draw transformed curve 
//       if(bDebugMe)
//         {
//           smgfx_SetLook(2,3, 1,.5,0) ; pEdge->Draw() ; sm_GraphicsLoop() ;
//           sm_GraphicsLoop() ;
//         }
// #endif // SM_DEBUG_CODE
//     } // end iter every curve

  // surfaces

  // save face->RectangularTrim states
  SmTArray<SmBoolean> sRectTrimState(crFaces.GetSize(), NULL, crFaces.GetSize()) ;
  for(ii=0;ii<crFaces.GetSize();ii++) 
    { sRectTrimState[ii] = crFaces[ii]->GetRectangularTrim() ; }

// Remove Composites - added block from block below
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
    } // end iter every surface

// Remove Composites
// // watch out for composite faces - don't apply transformation to one surface multiple times
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

} // end SmSpaceUnbend::TransformTopology

/*******************************************************************//**
PURPOSE: Apply Unbend map to topology within a single Brep.

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
SmStatus SmSpaceUnbend::UnbendTopology
  (const SmUnbendVolume     & crUnbendVolume, // in : Unbend mapping to apply
         SmBrep             & rBrep,          // in : Brep containing all topology to be transformed
   const SmTArray<SmFace*>  & crFaces,        // in : faces to transform
   const SmTArray<SmEdge*>  & crEdges,        // in : edges to transform 
   const SmTArray<SmVertex*>& crVertices)     // in : vertices to transform 
{
  // locals
  ULONG ii ;
  const SmContext & crContext = *rBrep.GetContext() ;
  SmTArray<SmSurface*> sSurfaces ;
  SmTArray<SmCurve*>   sCurves ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
#endif // SM_DEBUG_CODE

  // let the Brep clean up its stored caches
  rBrep.Notify(SM_NO_PRE_EDIT, this, NULL, NULL);

  // Unbend vertices
  for(ii=0;ii<crVertices.GetSize();ii++) 
    { 
      SmVertex * pVertex   = crVertices[ii] ;
      SmPoint3d  sPrePoint = pVertex->GetPoint() ;
      SmPoint3d  sPostPoint ;

      crUnbendVolume.MapPoint(sPrePoint, sPostPoint) ;
      
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
    } // end iter unbend vertices

  // edge->curves

// Remove Composites - added block to replace the one below
  // for every Edge->Curve
  for(ii=0;ii<crEdges.GetSize();ii++)
    {
      SmEdge  * pEdge  = crEdges[ii] ;
      SmCurve * pCurve = pEdge->GetCurve() ;
      SmCrvInVolume * pCrvInVolume = new (crContext) SmCrvInVolume
                                       (*pCurve,                    // in : projected Curve   - When(lOwnerFlag&1) rCurve->m_pOwner = this
                                        FALSE,                      // in : TRUE = rCurve is in rVolume's ParamSpace, FALSE= in InSpace
                                        (SmVolume &)crUnbendVolume, // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
                                        3,                          // in : 3 = copy both curve and volume
                                        3) ;                        // in : 3 = delete both curve and volume when destructed
      
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
      //            SM_ASSERT_MSG( sRtn == SM_SUCCESS, _T("DoSpaceUnbend::UnbendTopology - call SmEdge::ApproximateWithBSpline() failed")) ; 
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
      } // end iter every curve

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
//                                          (*pCurve,                    // in : projected Curve   - When(lOwnerFlag&1) rCurve->m_pOwner = this
//                                           FALSE,                      // in : TRUE = rCurve is in rVolume's ParamSpace, FALSE= in InSpace
//                                           (SmVolume &)crUnbendVolume, // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
//                                           3,                          // in : 3 = copy both curve and volume
//                                           3) ;                        // in : 3 = delete both curve and volume when destructed
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
//         //            SM_ASSERT_MSG( sRtn == SM_SUCCESS, _T("DoSpaceUnbend::UnbendTopology - call SmEdge::ApproximateWithBSpline() failed")) ; 
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
//       } // end iter every curve

  // face->Surfaces

  // save face->RectangularTrim states
  SmTArray<SmBoolean> sRectTrimState(crFaces.GetSize(), NULL, crFaces.GetSize()) ;
  for(ii=0;ii<crFaces.GetSize();ii++) 
    { sRectTrimState[ii] = crFaces[ii]->GetRectangularTrim() ; }

// Remove Composites - added block to replace the one below
  // for every face->surface
  for(ii=0;ii<crFaces.GetSize();ii++)
    {
      SmFace    * pFace    = crFaces[ii] ;
      SmSurface * pSurface = pFace->GetSurface() ;
      SmSrfInVolume * pSrfInVolume = new (crContext) SmSrfInVolume
                                       (*pSurface,                   // in : projected Surface - When(lOwnerFlag&1) rSurface->m_pOwner = this
                                        FALSE,                       // in : TRUE = m_pSurface is in m_pVolume's ParamSpace, FALSE = in InSpace
                                        (SmVolume &)crUnbendVolume,  // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
                                        2,                           // in : 2 = copy volume and save surface orig
                                        3) ;                         // in : 3 = delete both surface and volume when destructed

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
        //            SM_ASSERT_MSG( sRtn == SM_SUCCESS, _T("DoSpaceUnbend::UnbendTopology - call SmFace::ApproximateWithBSpline() failed")) ; 
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
//                                        (*pSurface,                   // in : projected Surface - When(lOwnerFlag&1) rSurface->m_pOwner = this
//                                         FALSE,                       // in : TRUE = m_pSurface is in m_pVolume's ParamSpace, FALSE = in InSpace
//                                         (SmVolume &)crUnbendVolume,  // in : projecting Volume - When(lOwnerFlag&2) rVolume->m_pOwner = this
//                                         2,                           // in : 2 = copy volume and save surface orig
//                                         3) ;                         // in : 3 = delete both surface and volume when destructed
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
//         //            SM_ASSERT_MSG( sRtn == SM_SUCCESS, _T("DoSpaceUnbend::UnbendTopology - call SmFace::ApproximateWithBSpline() failed")) ; 
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

} // end SmSpaceUnbend::UnbendTopology

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
SmStatus SmSpaceUnbend::ReplaceSurface
  (SmFace            * pFace,        // in : Target Face to get new Surface
   SmSrfInVolume     * pNewSurface)  // in : Target Surface to replace current pFace->Surface
{
  // locals 
  SM_PTR_ARRAY(sChildFaces, SmFace, 32);
  //SmSurface * pOld      = pFace->GetSurface() ;
  //SmFace    * pTopFace  = (SmFace*) pOld->GetFace() ;

  // inform the face
  pFace->Notify(SM_NO_PRE_EDIT, pFace, this, NULL) ;

  // change surface<->owner pointers
  pNewSurface->SetOwner(pFace) ;
  pFace->SetSurface(pNewSurface) ;

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
//          SmFace* pF = sChildFaces[ii];
//
//          // update the member face Surface pointer
//          pF->SetSurface(pNewSurface);
//        }
//    } // end composite face check

  // all done
  return SM_SUCCESS;

} // end SmSpaceUnbend::ReplaceSurface

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sAssertUnbend_list[] =
{
 /* 00 */ { SM_AT_POINTER,     _T("NULL Pointer"),      _T("Target Brep must be nonNull") },
 /* 01 */ { SM_AT_VALUES,      _T("UnbendAngle Range"), _T("UnbendAngle must be in the range -180 to + 180 degrees") },
 /* 02 */ { SM_AT_VALUES,      _T("Neg UnbendRadius"),  _T("The rule: NeutralRadius >= KFactor * SheetThickness must be TRUE") },
 /* 03 */ { SM_AT_UNIT_VECTOR, _T("Unit Vector"),       _T("UnbendAxis vector must be unit length") },
 /* 04 */ { SM_AT_UNIT_VECTOR, _T("Unit Vector"),       _T("UnbendDirection vector must be unit length") },
 /* 05 */ { SM_AT_VECTOR,      _T("Parallel Vectors"),  _T("UnbendAxis and UnbendDirection can't be parallel") },
 /* 06 */ { SM_AT_VALUES,      _T("KFactor Range"),     _T("KFactor must be in the range 0.0 to 1.0") }
} ;   

/*******************************************************************//**
PURPOSE:  Check that Unbend Analytic and Nurb representations are
             equivalent.

NOTES:  returns TRUE = OK, FALSE = problem
***********************************************************************/
SmBoolean SmSpaceUnbend::AssertValid
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
      if(m_pBrep)         bRtn &= m_pBrep->AssertValid(pAList, eTestLevel, eWalkTree) ; 
      if(m_pUnbendDomain) bRtn &= m_pUnbendDomain->AssertValid(pAList, eTestLevel, eWalkTree) ;
    }
#else
  SM_REF1(eWalkTree);
#endif // SM_DEBUG_CODE

  // Target Brep must be nonNULL
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(0, SM_LEVEL_0, (m_pBrep != NULL), _T("")) ;

  // UnbendAngle must be in the range -180 to + 180 degrees
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(1, SM_LEVEL_0, (m_dUnbendAngDeg >= -180.0 && m_dUnbendAngDeg <= 180.0), _T("")) ;

  // The rule: NeutralRadius >= KFactor * SheetThickness must be TRUE
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(2, SM_LEVEL_0, (m_dNeutralRadius >= m_dKFactor * m_dSheetThickness), _T("")) ;

  // UnbendAxis vector must be unit length   
  double dLengthSq = m_vUnbendAxis.LengthSquared() ;                     
  bRtn &= SM_ASSERT_VALUE_REPORT(3, SM_LEVEL_0, SM_ARE_SAME_TO_TOL(dLengthSq, 1.0, SM_EFF_ZERO), SM_EFF_ZERO, dLengthSq - 1.0, _T("")) ;

  // UnbendDirection vector must be unit length                  
  dLengthSq = m_vUnbendDirection.LengthSquared() ;                     
  bRtn &= SM_ASSERT_VALUE_REPORT(4, SM_LEVEL_0, SM_ARE_SAME_TO_TOL(dLengthSq, 1.0, SM_EFF_ZERO), SM_EFF_ZERO, dLengthSq - 1.0, _T("")) ;

  // UnbendAxis and UnbendDirection can't be parallel
  double dAngRad ;
  m_vUnbendAxis.AngleBetween(m_vUnbendDirection, dAngRad) ;              
  bRtn &= SM_ASSERT_VALUE_REPORT(5, SM_LEVEL_0, (FALSE == m_vUnbendAxis.IsParallelTo(m_vUnbendDirection, SM_EFF_ZERO_DEG)), SM_EFF_ZERO_DEG, SM_RAD2DEG(dAngRad), _T("") ) ;

  // KFactor must be in the range 0.0 to 1.0                    
  bRtn &= SM_ASSERT_BOOLEAN_REPORT(6, SM_LEVEL_0, (m_dKFactor >= 0.0 && m_dKFactor <= 1.0), _T("") ) ;

  // all done
  return(bRtn) ;

} // end SmSpaceUnbend::AssertValid

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSpaceUnbend::IsKindOf( SM_TYPE t ) const
{
  return ((SmSpaceUnbend_TYPE == t) ? TRUE : SmObject::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Dump Unbend data out for debugging.

NOTES: 
***********************************************************************/
void SmSpaceUnbend::Dump
  () 
 const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;
  smos_WriteBuffer(_T("\nBegin SmSpaceUnbend::Dump()")) ;

  // get the parent's dump
  SmObject::Dump();

  // output header and brep
  smos_sprintf(sBuff,       _T("\n pSpaceUnbend = 0x%p, pBrep = 0x%p"), this, m_pBrep) ;
  smos_sprintf(sBuffForFile,_T("\n pSpaceUnbend = %s, pBrep = %s")
             , _T("notNULL")
             , m_pBrep ? _T("notNULL") : _T("NULL"));
  smos_WriteBuffer(sBuff, sBuffForFile);

  // output basic bend and sheet data
  smos_sprintf(sBuff, _T("\n  UnbendAngDeg        : [%16.16lf]"), m_dUnbendAngDeg) ;   smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff, _T("\n  UnbendNeutralRad    : [%16.16lf]"), m_dNeutralRadius) ;  smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff, _T("\n  UnbendSheetThickness: [%16.16lf]"), m_dSheetThickness) ; smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff, _T("\n  UnbendKFactor       : [%16.16lf]"), m_dKFactor) ;        smos_WriteBuffer(sBuff);

  // output bend position, orientation, and extent data
  smos_WriteBuffer(_T("\n  Unbend Axis Point   :")) ; m_vUnbendAxisPoint.Dump() ;
  smos_WriteBuffer(_T("\n  Unbend Neutral MidPt:")) ; GetNeutralMidPoint().Dump() ;
  smos_WriteBuffer(_T("\n  Unbend Axis (Z Axis):")) ; m_vUnbendAxis.Dump() ;
  smos_WriteBuffer(_T("\n  Unbend Direction (X):")) ; m_vUnbendDirection.Dump() ;
  smos_WriteBuffer(_T("\n  Unbend Binormal (Y) :")) ; (m_vUnbendAxis * m_vUnbendDirection).Dump() ;
  smos_WriteBuffer(_T("\n  Unbend Interval     :")) ; m_vUnbendInterval.Dump() ;

  // output basic bend and sheet data
  smos_sprintf(sBuff, _T("\n  Final Part Position : [%s]\n"),
               m_eUnbendOrient == SM_UO_FIXED_BEFORE_UNBENDING ? _T("Fixed Start Unbending = before Unbend geometry remains fixed")
             : m_eUnbendOrient == SM_UO_CENTER_UNBENDING       ? _T("Center Unbending = before and after Unbend geometries rotate equal and opposite dirs")
             : m_eUnbendOrient == SM_UO_FIXED_AFTER_UNBENDING  ? _T("Fixed End Unbending = after Unbend geometry remains fixed")
             :                                                   _T("Unknown Unbending type")) ; 
  smos_WriteBuffer(sBuff);

  // dump nested objects
  smos_WriteBuffer(_T("\nBegin Dump SmSpaceUnbend Nested Objects:[TgtBrep, UnbendDomainBrep, StartUnbendBndryBrep, StopUnbendBndryBrep]")) ;

  // m_pBrep
  smos_WriteBuffer(_T("\nBegin Nested SmSpaceUnbend->m_pBrep::Dump()")) ;
    if(m_pBrep) {  m_pBrep->Dump(SM_BD_GEOM_TYPES) ; }
    else        {  smos_WriteBuffer(_T("\n   SmSpaceUnbend->m_pBrep is NULL ")) ; }
  smos_WriteBuffer(_T("End    Nested SmSpaceUnbend->m_pBrep::Dump()\n")) ;

  // PreUnbendRegion
  smos_WriteBuffer(_T("\nBegin Nested SmSpaceUnbend->m_pUnbendDomain::Dump()")) ;
    if(m_pUnbendDomain) {  m_pUnbendDomain->Dump(SM_BD_GEOM_TYPES) ; }
    else                 {  smos_WriteBuffer(_T("\n   SmSpaceUnbend->m_pUnbendDomain is NULL ")) ; }
  smos_WriteBuffer(_T("End    Nested SmSpaceUnbend->m_pUnbendDomain::Dump()\n")) ;

  // StartUnbendBoundary
  smos_WriteBuffer(_T("\nBegin Nested SmSpaceUnbend->m_pStartUnbendBoundary::Dump()")) ;
    if(m_pUnbendDomain) {  m_pStartUnbendBoundary->Dump(SM_BD_GEOM_TYPES) ; }
    else                 {  smos_WriteBuffer(_T("\n   SmSpaceUnbend->m_pStartUnbendBoundary is NULL ")) ; }
  smos_WriteBuffer(_T("End    Nested SmSpaceUnbend->m_pStartUnbendBoundary::Dump()\n")) ;

  // StopUnbendBoundary
  smos_WriteBuffer(_T("\nBegin Nested SmSpaceUnbend->m_pStopUnbendBoundary::Dump()")) ;
    if(m_pUnbendDomain) {  m_pStopUnbendBoundary->Dump(SM_BD_GEOM_TYPES) ; }
    else                 {  smos_WriteBuffer(_T("\n   SmSpaceUnbend->m_pStopUnbendBoundary is NULL ")) ; }
  smos_WriteBuffer(_T("End    Nested SmSpaceUnbend->m_pStopUnbendBoundary::Dump()\n")) ;

  // end dump nested objects
  smos_WriteBuffer(_T("\nEnd Dump SmSpaceUnbend Nested Objects:[TgtBrep, UnbendDomainBrep, StartUnbendBndryBrep, StopUnbendBndryBrep]")) ;

  // all done
  smos_WriteBuffer(_T("\nEnd SmSpaceUnbend::Dump()\n")) ;

} // end SmSpaceUnbend::Dump

/*******************************************************************//**
PURPOSE:  Add Unbend graphics for global display Parameters
             to new DisplayList added to global DisplayList array.

NOTES:
***************************************************************/
SmDisplayList * SmSpaceUnbend::Draw
  (SmBoolean bDrawBrep,            // in : TRUE = Draw target m_pBrep, default:[FALSE]
   SmBoolean bAddToUIPickList,     // NotUsed: in : TRUE = Add target m_pBrep to UI pick interface for debugging
                                   //      default:[FALSE]
   SmBoolean bDrawPreUnbendRegion, // in : TRUE = Draw preUnbendRegion, default:[FALSE] 
   SmGfxArraySet * pOptGfxSet)     // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
  const                            //      NULL to ignore. default:[NULL]
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

  // Draw the preUnbendRegion UnBendDomain, StartUnbendBoundary, and StopUnbendBoundary
  if(bDrawPreUnbendRegion)
    {
      // draw the preUnbendRegion
      smgfx_SetLook(dLineWidth+2, dPointSize, 0,0,0, pOptGfxSet) ; GetUnbendDomain()->Draw(TRUE) ; // gwc: need to make SmBrep::Draw() pOptGfxSet aware

      // set Brep faces to draw crosshatched
      SmDisplayParameters & rDisp              = smgfx_RefGlobalDisplayParameters() ;
      SmBoolean             bOldDrawCrossHatch = rDisp.m_bDrawCrossHatch ; 
      rDisp.m_bDrawCrossHatch = TRUE ;
  
      // Draw the preUnbendRegion StartBoundary
      smgfx_SetLook(dLineWidth+2, dPointSize, 0,1,0, pOptGfxSet) ; GetStartUnbendBoundary()->Draw() ; // gwc: need to make SmBrep::Draw() pOptGfxSet aware
      smgfx_SetLook(dLineWidth+2, dPointSize, 1,0,0, pOptGfxSet) ; GetStopUnbendBoundary()->Draw() ;  // gwc: need to make SmBrep::Draw() pOptGfxSet aware

      // restore state
      smgfx_SetLook(dLineWidth, dPointSize, sColor, pOptGfxSet) ; 
      rDisp.m_bDrawCrossHatch = bOldDrawCrossHatch ;
    }

  SmPoint3d sNeutralMidPoint = GetNeutralMidPoint() ;
  SmPoint3d sNeutralBegPoint = GetNeutralBegPoint() ;
  SmPoint3d sNeutralEndPoint = GetNeutralEndPoint() ;

  // Points on Pre Unbend Neutral Arc - shows begin/end sides of the unbend region
  smgfx_SetPointSize(dPointSize+2, pOptGfxSet) ; 

  // Unbend Axis
  SmPoint3d sUnbendStart = GetUnbendSegmentStart() ;
  SmPoint3d sUnbendStop  = GetUnbendSegmentStop() ;
  smgfx_OutputLine(sUnbendStart.x, sUnbendStart.y, sUnbendStart.z,
                   sUnbendStop.x, sUnbendStop.y, sUnbendStop.z, pOptGfxSet) ;  

  // Unbend Graphics - center point of unbend coordinate system
  SmPoint3d sAxisPoint = GetUnbendAxisPoint() ; 
  smgfx_OutputPoint( sAxisPoint.x, sAxisPoint.y, sAxisPoint.z, pOptGfxSet) ;
  
  // Unbend Graphics - Unbend Direction from CenterPoint to NeutralPoint
  smgfx_OutputLine(sAxisPoint.x, sAxisPoint.y, sAxisPoint.z,
                   sNeutralMidPoint.x, sNeutralMidPoint.y, sNeutralMidPoint.z, pOptGfxSet) ;

  // Unbend Graphics - positive Unbend cross-direction  (used to check Start and Stop faces)
  // with U = m_vUnbendAxis
  //      V = m_vUnbendDirection * m_vUnbendAxis 
  //      W = m_vUnbendDirection
  double     dHalfWidth    = GetPostUnbendWidth() / 2.0 ; 
  SmVector3d sVAxis        = GetUnbendBinormal() ;
  smgfx_SetLook(dLineWidth+1, dPointSize+2, 1,.647,0, pOptGfxSet) ; // orange
  smgfx_OutputLine(sNeutralMidPoint.x-dHalfWidth*sVAxis.x, 
                   sNeutralMidPoint.y-dHalfWidth*sVAxis.y, 
                   sNeutralMidPoint.z-dHalfWidth*sVAxis.z,
                   sNeutralMidPoint.x+dHalfWidth*sVAxis.x, 
                   sNeutralMidPoint.y+dHalfWidth*sVAxis.y, 
                   sNeutralMidPoint.z+dHalfWidth*sVAxis.z, pOptGfxSet) ;
  smgfx_OutputPoint(sNeutralMidPoint.x, sNeutralMidPoint.y, sNeutralMidPoint.z, pOptGfxSet) ;
  smgfx_SetColor(0,1,0, pOptGfxSet) ; smgfx_OutputPoint(sNeutralBegPoint.x, sNeutralBegPoint.y, sNeutralBegPoint.z, pOptGfxSet) ; 
  smgfx_SetColor(1,0,0, pOptGfxSet) ; smgfx_OutputPoint(sNeutralEndPoint.x, sNeutralEndPoint.y, sNeutralEndPoint.z, pOptGfxSet) ; 
  smgfx_OutputColor(sColor, pOptGfxSet) ;

  smgfx_SetLook(dLineWidth, dPointSize, sColor, pOptGfxSet) ; 

  // Unbend Graphics - PreBend circular arcs and PostBend lines at sheet inside, outside, and neutral surfaces
  ULONG ii, jj, kk ; 
  const ULONG lSideCnt = 7 ; 
  const ULONG lCnt     = 2 * lSideCnt + 1 ;  // an odd number
  double dRadius[3] ;
  dRadius[0] = GetInsideRadius() ;  
  dRadius[1] = GetOutsideRadius() ; 
  dRadius[2] = GetNeutralRadius() ;
  SmVector3d sX =  GetUnbendDirection() ;
  SmVector3d sY =  GetUnbendBinormal() ;  
  SmVector3d sZ =  GetUnbendAxis() ;
  SmPoint3d sPrePts[lCnt], sPostPts[lCnt], sOrientedPts[lCnt] ;
  double dRad, dIncRad = SM_DEG2RAD( m_dUnbendAngDeg/2.0/lSideCnt ) ;
  SmAxis2Placement sInsideUnbendTransform = GetInsideUnbendTransform() ;
  SmUnbendVolume * pUnbendVolume = CreateUnbendVolume(*GetContext()) ;
  SmObjDelete sClean1(pUnbendVolume) ;

  // for kk slices along the length of the bend axis
  for(kk=0;kk<3;kk++)
    { 
      double dParam = (double)kk/(3.0 - 1.0) ;
      sAxisPoint = (1.0 - dParam) * sUnbendStart + (dParam) * sUnbendStop ;

      // for 3 sampled arcs
      for(ii=0;ii<3;ii++)
        { 
          // make the neutral arc a little heavier and orange
          if(ii==2) { smgfx_SetLook(dLineWidth+1, dPointSize, 1,.647,0, pOptGfxSet) ; } // orange
          else      { smgfx_SetLook(dLineWidth, dPointSize, sColor, pOptGfxSet) ; } // 

          // arc center point
          sPrePts[lSideCnt] = sAxisPoint + dRadius[ii] * sX ; 
          pUnbendVolume->MapPoint(sPrePts[lSideCnt], sPostPts[lSideCnt]) ;
          sInsideUnbendTransform.TransformPoint(sPostPts[lSideCnt], sOrientedPts[lSideCnt]) ;

          // for every pair of sampled arc points
          for(jj=1, dRad=dIncRad;jj<=lSideCnt;jj++,dRad+=dIncRad)
            {
              double dCos = smos_Cosine(dRad) ;
              double dSin = smos_Sine(dRad) ;
              sPrePts[lSideCnt + jj] = sAxisPoint + dRadius[ii] * (dCos * sX + dSin * sY) ;
              sPrePts[lSideCnt - jj] = sAxisPoint + dRadius[ii] * (dCos * sX - dSin * sY) ;
              pUnbendVolume->MapPoint(sPrePts[lSideCnt + jj], sPostPts[lSideCnt + jj]) ;
              pUnbendVolume->MapPoint(sPrePts[lSideCnt - jj], sPostPts[lSideCnt - jj]) ;
              sInsideUnbendTransform.TransformPoint(sPostPts[lSideCnt - jj], sOrientedPts[lSideCnt - jj]) ;
              sInsideUnbendTransform.TransformPoint(sPostPts[lSideCnt + jj], sOrientedPts[lSideCnt + jj]) ;
            }

          // output arc
          smgfx_OutputPolyline((double *)sPrePts, lCnt, 3, pOptGfxSet) ; 
          smgfx_OutputPolyline((double *)sPostPts, lCnt, 3, pOptGfxSet) ; 
          smgfx_SetLook(dLineWidth, dPointSize, 1,0,0, pOptGfxSet) ; smgfx_OutputPolyline((double *)sOrientedPts, lCnt, 3, pOptGfxSet) ; 
        } // end iter ii, inside and outside sheetmetal bend arcs
    } // end iter kk, slices along the bend axis
  
  // all done - restore state and end display list
  smgfx_OutputColor(sColor, pOptGfxSet) ;
  smgfx_SetLineWidth(dLineWidth, pOptGfxSet) ;
  smgfx_SetPointSize(dPointSize, pOptGfxSet) ;
    
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF3(bDrawBrep, bDrawPreUnbendRegion, pOptGfxSet);
#endif // SM_GFX_CODE
  return(pRtn) ;

} // end SmSpaceUnbend::Draw

// obsolete
// /*******************************************************************//**
// PURPOSE: Fix Assert Report failures reported by an AssertValid call
// 
// RETURNS ---  TRUE  = OK
//              FALSE = Problem
// ***********************************************************************/
// SmBoolean SmSpaceUnbend::AssertHeal
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
//                rAReport.m_pHealMessage = _T("SmSpaceUnbend::AssertHeal fix not yet supported") ;
// 
//     }
// 
//   // all done
//   return(bRtn) ;
// 
// } // end SmSpaceUnbend::AssertHeal
// end obsolete
