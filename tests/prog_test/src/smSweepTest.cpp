// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smsweep_test.cpp
* PURPOSE ---  Test sweep functions.
*
**********************************************************************/
/*___*/

#include "StdAfx.h"
#include <smSweepTest.h>
#include <SmTransSweepGeometry.h>
#include <SmRotationalSweepGeometry.h>


#define DELETE_ALL_PARTS(parts) \
{ for (ULONG z=0; z<(parts).GetSize(); z++) \
  { SM_ASSERT((parts)[z] != NULL) ; delete (parts)[z]; (parts)[z] = NULL ; } \
  (parts).ReSet(); \
}

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
void ucheck_mappings_sweep
  (SmTopologySweep           & /* rSweepArg                 */, 
   const SmTArray<SmFace*>   * /* pFacesToSweepHigherArg    */,
   const SmTArray<SmEdge*>   * /* pEdgesToSweepHigherArg    */,
   const SmTArray<SmVertex*> * /* pVerticesToSweepHigherArg */,
   const SmTArray<SmFace*>   * /* pFacesToSweepSameArg      */,
   const SmTArray<SmEdge*>   * /* pEdgesToSweepSameArg      */,
   const SmTArray<SmVertex*> * /* pVerticesToSweepSameArg   */)
{
#if 0

    // Every object in the arg list must have a map in To, and if that's
    // not itself (meaning that the sweep was degenerate and the object
    // sweeps to itself), the map must have a proimage which is the original

    if(pFacesToSweepHigherArg) {
        const SmTArray<SmFace*>& aF = *pFacesToSweepHigherArg;
        for(ULONG i=0; i<aF.GetSize();i++) {
            SmFace* pF = aF[i];
            SmFace* pIF = rSweepArg.GetToSame(pF);
            SM_ASSERT(pIF!=NULL);
            SM_ASSERT(pIF == pF || 
                (pIF != pF && rSweepArg.GetFromSame(pIF) == pF));

            SmTArray<SmEdge*> aE;
            SmTArray<SmVertex*> aV;
            pF->GetEdges(aE);
            pF->GetVertices(aV);
            for(ULONG j=0; j<aV.GetSize(); j++) {
                SmVertex* pV = aV[j];
                SmVertex* pIV = rSweepArg.GetToSame(pV);
                SM_ASSERT(pIV!=NULL);

                SM_ASSERT(pIV == pV || 
                    (pIV != pV && rSweepArg.GetFromSame(pIV) == pV));
            }
            for(ULONG k=0; k<aE.GetSize(); k++) {
                SmEdge* pE = aE[k];
                SmEdge* pIE = rSweepArg.GetToSame(pE);
                SM_ASSERT(pIE!=NULL);
                SM_ASSERT(pIE == pE || 
                         (pE != pIE && rSweepArg.GetFromSame(pIE) == pE));
            }
        }
    }
    if(pFacesToSweepSameArg) {
        const SmTArray<SmFace*>& aF = *pFacesToSweepSameArg;
        for(ULONG i=0; i<aF.GetSize();i++) {
            SmFace* pF = aF[i];

            SmFace* pIF = rSweepArg.GetToSame(pF);
            SM_ASSERT(pIF!=NULL);
            SM_ASSERT(pIF == pF || 
                (pIF != pF && rSweepArg.GetFromSame(pIF) == pF));

            SmTArray<SmEdge*> aE;
            SmTArray<SmVertex*> aV;
            pF->GetEdges(aE);
            pF->GetVertices(aV);
            for(ULONG j=0; j<aV.GetSize(); j++) {
                SmVertex* pV = aV[j];
                SmVertex* pIV = rSweepArg.GetToSame(pV);
                SM_ASSERT(pIV!=NULL);

                SM_ASSERT(pIV == pV || 
                    (pIV != pV && rSweepArg.GetFromSame(pIV) == pV));

            }
            for(ULONG k=0; k<aE.GetSize(); k++) {
                SmEdge* pE = aE[k];
                SmEdge* pIE = rSweepArg.GetToSame(pE);
                SM_ASSERT(pIE!=NULL);
                SM_ASSERT(pIE == pE || 
                         (pE != pIE && rSweepArg.GetFromSame(pIE) == pE));
            }
        }
    }
    if(pEdgesToSweepHigherArg) {
        const SmTArray<SmEdge*>& aE = *pEdgesToSweepHigherArg;
        for(ULONG i=0; i<aE.GetSize();i++) {
            SmEdge* pE = aE[i];
            SmEdge* pIE = rSweepArg.GetToSame(pE);
            SM_ASSERT(pIE!=NULL);
            SM_ASSERT(pIE == pE || 
                (pE != pIE && rSweepArg.GetFromSame(pIE) == pE));

            SmVertex* pV1 = pE->GetVertex();
            SmVertex* pV2 = pE->GetOtherVertex(pE->GetVertex());

            SmVertex* pIV1 = rSweepArg.GetToSame(pV1);
            SM_ASSERT(pIV1!=NULL);
            SmVertex* pIV2 = rSweepArg.GetToSame(pV2);
            SM_ASSERT(pIV2!=NULL);

            SM_ASSERT(pIV1 == pV1 || 
                      (pIV1 != pV1 && rSweepArg.GetFromSame(pIV1) == pV1));
            SM_ASSERT(pIV2 == pV2 || 
                      (pIV2 != pV2 && rSweepArg.GetFromSame(pIV2) == pV2));

        }
    }
    if(pEdgesToSweepSameArg) {
        const SmTArray<SmEdge*>& aE = *pEdgesToSweepSameArg;
        for(ULONG i=0; i<aE.GetSize();i++) {
            SmEdge* pE = aE[i];

            SmEdge* pIE = rSweepArg.GetToSame(pE);
            SM_ASSERT(pIE!=NULL);
            SM_ASSERT(pIE == pE || 
                (pE != pIE && rSweepArg.GetFromSame(pIE) == pE));

            SmVertex* pV1 = pE->GetVertex();
            SmVertex* pV2 = pE->GetOtherVertex(pE->GetVertex());

            SmVertex* pIV1 = rSweepArg.GetToSame(pV1);
            SM_ASSERT(pIV1!=NULL);
            SmVertex* pIV2 = rSweepArg.GetToSame(pV2);
            SM_ASSERT(pIV2!=NULL);

            SM_ASSERT(pIV1 == pV1 || 
                      (pIV1 != pV1 && rSweepArg.GetFromSame(pIV1) == pV1));
            SM_ASSERT(pIV2 == pV2 || 
                      (pIV2 != pV2 && rSweepArg.GetFromSame(pIV2) == pV2));

        }
    }
    if(pVerticesToSweepHigherArg) {
        const SmTArray<SmVertex*>& aV = *pVerticesToSweepHigherArg;
        for(ULONG i=0; i<aV.GetSize();i++) {
            SmVertex* pV = aV[i];

            SmVertex* pIV = rSweepArg.GetToSame(pV);
            SM_ASSERT(pIV!=NULL);
            SM_ASSERT(pIV == pV || 
                (pIV != pV && rSweepArg.GetFromSame(pIV) == pV));
        }
    }
    if(pVerticesToSweepSameArg) {
        const SmTArray<SmVertex*>& aV = *pVerticesToSweepSameArg;
        for(ULONG i=0; i<aV.GetSize();i++) {
            SmVertex* pV = aV[i];

            SmVertex* pIV = rSweepArg.GetToSame(pV);
            SM_ASSERT(pIV!=NULL);

            SM_ASSERT(pIV == pV || 
                (pIV != pV && rSweepArg.GetFromSame(pIV) == pV));
        }
    }
#endif

} // end ucheck_mappings_sweep

/**************************************************************
PURPOSE --- Create a test Brep containing
    1 face with 2 inner Loops holes
                1 vertex loop
                1 edgeLoop
    1 wire edge
    1 vertex shell

USAGE NOTES ---
**************************************************************/
static SmStatus my_create_test_brep
  (const SmContext & crContext,     // in : context for new object construction
   SmBrep         *& rpBrep)        // out: Brep to receive new geoemtry
{
  // add a single face with 2 inner edgeLoops to rpBrep
  SER(my_test_tsurf_creation(crContext,rpBrep));
#ifdef SM_DEBUG_CODE
#ifdef SM_VALIDATE_POINTERS  
  rpBrep->ValidatePointers() ;
#endif  
#endif  

  SmTemporaryChangeValue <SmBoolean > sChangeDB( ((SmContext*) &crContext)->GetDoingBooleanRef(), TRUE );

  // locals
  SmTArray<SmVertex*> sVertices;
  rpBrep->GetVertices(sVertices);
  SmTArray<SmFace*> sFaces;
  rpBrep->GetFaces(sFaces);
  SmFace *pFace = sFaces[0];
  
  // add a vertexLoop to rpBrep
  SmLoop   *pNewLoop;
  SmVertex *pNewVertex;
  SER(rpBrep->MakeVertexLoop(pFace,SmPoint3d(3.5,8.5,0.0),
                             pNewLoop, pNewVertex));
#ifdef SM_DEBUG_CODE
#ifdef SM_VALIDATE_POINTERS  
  rpBrep->ValidatePointers() ;
#endif  
#endif  

  // add a LineLoop to rpBrep
  SmBSplineCurve *pLine = NULL ;
  SER(SmBSplineCurve::CreateLineSegment(crContext,3,
      SmPoint3d(0,0,0),SmPoint3d(5,5,0),
      pLine));
  SmEdge *pNewEdge;
  SmFace *pNewFace;
  rpBrep->m_bEditingEnabled = TRUE;
  SER(rpBrep->MakeEdgeInFace(pFace,sVertices[0],sVertices[3],
                             pLine, NULL, pLine->GetNaturalInterval(), SM_OT_SAME, SM_ZONE_TOL_3D,
                             pNewEdge, pNewLoop, pNewFace));
  pLine = NULL ;
#ifdef SM_DEBUG_CODE
#ifdef SM_VALIDATE_POINTERS  
  rpBrep->ValidatePointers() ;
#endif  
#endif  
  
  // add a wire to rpBrep
  SER(SmBSplineCurve::CreateLineSegment(crContext,3,
                                        SmPoint3d(10,0,0),
                                        SmPoint3d(15,2.0,0),
                                        pLine));
  SER(rpBrep->MakeWireEdgeVertex(rpBrep->GetInfiniteRegion(),
                                 sVertices[1], pLine, pLine->GetNaturalInterval(), 
                                 SM_OT_SAME, SmPoint3d(15,2,0), pNewEdge, pNewVertex));
  pLine = NULL ;

#ifdef SM_DEBUG_CODE
#ifdef SM_VALIDATE_POINTERS  
  rpBrep->ValidatePointers() ;
#endif  
#endif  

  // add a shellVertex to rpBrep
  SmShell *pNewShell;
  SER(rpBrep->MakeShellVertex(rpBrep->GetInfiniteRegion(),
                              SmPoint3d(-5.0,0,0),
                              pNewShell, pNewVertex));
#ifdef SM_DEBUG_CODE
#ifdef SM_VALIDATE_POINTERS  
  rpBrep->ValidatePointers() ;
#endif  
#endif  

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics() && rpBrep)
    {
      rpBrep->Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; rpBrep->Draw(TRUE) ;
      sm_GraphicsLoop() ;
    }
#endif

  return SM_SUCCESS;

} // end my_create_test_brep

#include <SmMerge.h>

// Local functions to create the test parts for whole-Brep sweeps.

static SmStatus my_make_block_with_lines( SmBrep *pBrep )
{
  const SmContext *pContext = pBrep->GetContext();
  SmPrimitiveCreation sPC( pBrep->GetInfiniteRegion() );
  SmPoint3d sCenter( 8,0,0 );
  SmVector3d sXAxis( 1,0,0 );
  SmVector3d sYAxis( 0,1,0 );
  SmAxis2Placement sPlacement( sCenter, sXAxis, sYAxis );

  double dWidth = 4, dHeight = 3, dDepth = 1.5;
  sPC.CreateBox( dWidth, dHeight, dDepth, sPlacement );

  SmTArray< SmEdge* > sEdges;
  pBrep->GetEdges( sEdges );
  SmEdge *pNewE1, *pNewE2, *pE = sEdges[0];
  SmVertex *pNewV1, *pNewV2, *pNewV3, *pNewV4;
  SmTemporaryChangeValue< SmBoolean > sChange( pBrep->m_bEditingEnabled, TRUE );
  SmTemporaryChangeValue< SmBoolean > sChangeDB( ((SmContext*) pContext)->GetDoingBooleanRef(), TRUE );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      ULONG ii ;
      SM_ASSERT_VALID(pBrep) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      for(ii=0;ii<sEdges.GetSize();ii++)
        { smgfx_SetLook(3,4, 0,0,0) ; if(sEdges[ii]) sEdges[ii]->Draw() ; sm_GraphicsLoop() ; }
      sm_GraphicsLoop() ;                         
    }                                             
#endif // SM_DEBUG_CODE                           

  pBrep->MakeVertexSplitEdge( sEdges[1], 1.2, pNewE1, pNewE2, pNewV1 );
  pBrep->MakeVertexSplitEdge( sEdges[0], 0.9, pNewE1, pNewE2, pNewV2 );
  pBrep->MakeVertexSplitEdge( sEdges[4], 0.9, pNewE1, pNewE2, pNewV3 );
  pBrep->MakeVertexSplitEdge( sEdges[7], 1.2, pNewE1, pNewE2, pNewV4 );

  SmLine *pLine1, *pLine2, *pLine3;
  SmLine::CreateLineSegment( *pContext, 3, pNewV1->GetPoint(), pNewV2->GetPoint(), pLine1 );
  SmLine::CreateLineSegment( *pContext, 3, pNewV2->GetPoint(), pNewV3->GetPoint(), pLine2 );
  SmLine::CreateLineSegment( *pContext, 3, pNewV3->GetPoint(), pNewV4->GetPoint(), pLine3 );

  SmTArray< SmFace* > sFaces;
  pBrep->GetFaces( sFaces );

  double dTol = pBrep->GetTolerance();
  SmExtent1d sIvl = pLine1->GetNaturalInterval();
  SmLoop *pL;
  SmFace *pF;

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      ULONG ii ;
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      for(ii=0;ii<sFaces.GetSize();ii++)
        { smgfx_SetLook(1,2, 0,0,0) ; if(sFaces[ii]) sFaces[ii]->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ; }
      for(ii=0;ii<sEdges.GetSize();ii++)
        { smgfx_SetLook(3,4, 0,0,0) ; if(sEdges[ii]) sEdges[ii]->Draw() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(3,4, 1,0,1) ; if(pLine1) pLine1->Draw(&sIvl, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pLine2) pLine2->Draw(&sIvl, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pLine3) pLine3->Draw(&sIvl, TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,1,1) ; if(pNewV1) pNewV1->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,1,1) ; if(pNewV2) pNewV2->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,1,1) ; if(pNewV3) pNewV3->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,1,1) ; if(pNewV4) pNewV4->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;                         
    }                                             
#endif // SM_DEBUG_CODE                           


  pBrep->MakeEdgeInFace( sFaces[1], pNewV1, pNewV2, pLine1, NULL, sIvl, SM_OT_SAME, dTol,
      pE, pL, pF );

  sIvl = pLine2->GetNaturalInterval();
  pBrep->MakeEdgeInFace( sFaces[3], pNewV2, pNewV3, pLine2, NULL, sIvl, SM_OT_SAME, dTol,
      pE, pL, pF );

  sIvl = pLine3->GetNaturalInterval();
  pBrep->MakeEdgeInFace( sFaces[0], pNewV3, pNewV4, pLine3, NULL, sIvl, SM_OT_SAME, dTol,
      pE, pL, pF );

#ifdef SM_DEBUG_CODE
  if(bDebugMe) {
      smgfx_Erase(1);
      smgfx_SetLook(1,2, 0,0,1 ); pBrep->Draw(1); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif
//
//  sCenter.Set( 11,  1, 0 );
//  sXAxis .Set(  1, -2, 0 );
//  sYAxis .Set( -2, -1, 0 );
//  SmAxis2Placement sPlacement2( sCenter, sXAxis, sYAxis );
//
//  dWidth = 8; dHeight = 6; dDepth = 3;
//
//  smgfx_SetLook( 1,2, 0,0,0 ); pBrep->Draw(1); sm_GraphicsLoop();
//  smgfx_SetLook( 1,2, 0,1,0 ); pPlane->Draw(); sm_GraphicsLoop();
//  sm_GraphicsLoop();
//
//  SmBrep *pTempBrep = new ( *pContext ) SmBrep();

  return SM_SUCCESS;

} // end my_make_block_with_lines

static SmStatus my_make_hollow_box( SmBrep *pBrep )
{
  const SmContext *pContext = pBrep->GetContext();
  SmPrimitiveCreation sPC( pBrep->GetInfiniteRegion() );
  SmPoint3d sCenter( 0,0,0 );
  SmVector3d sXAxis( 1,0,0 );
  SmVector3d sYAxis( 0,1,0 );
  SmAxis2Placement sPlacement( sCenter, sXAxis, sYAxis );

  sPC.CreateBox( 10.0, 8.0, 6.0, sPlacement );

  SmBrep *pTempBrep = new ( *pContext ) SmBrep();
  SmPrimitiveCreation sPC2( pTempBrep->GetInfiniteRegion() );

  sCenter.Set( 1, 1, 0.5 );
  sPlacement.Translate( sCenter );
  sPC2.CreateBox( 8.0, 6.0, 5.0, sPlacement );

  SmBrep *pResult;
  SmMerge sMergeObj( *pContext, pBrep, pTempBrep, pBrep->GetTolerance(), 0.1 );
  sMergeObj.NonManifoldBoolean( SM_BO_DIFFERENCE, pResult );

  SM_ASSERT( pResult == pBrep );
  pBrep = pResult;

  return SM_SUCCESS;

} // end my_make_hollow_box

                 
/***********************************************************************
PURPOSE --- Run a specific test (nTest) or all tests in turn 
            (nTest omitted or -1)

USAGE NOTES ---  Used by prog_test via my_primsol_regression
***********************************************************************/
SmStatus my_primsol_demo
( 
  SmContext & crContext,            // in : context for new object construction
  SmTArray<SmBrep*> & rPartBreps,   // 
  ULONG nTestArg                    // in : test to run  nTestArg > 100 - run next test
)                   
                                   
{
  // output id string 
  MYPRINTF( _T( "\n Entered in my_primsol_demo\n" ) );

  // locals
  TCHAR sBuff[SM_TBLOCK_SIZE];
  ULONG nTest = nTestArg;
  ULONG lNumUVCurves, lNum3DCurves;

  // select test to run - either nTestArg or nextTest when nTestArg > 100
  static ULONG lCount = 0;
  if(nTest > 100)
  {
    lCount++;
    if(lCount == my_primsol_demo_count() + 1)
      lCount = 1;
    nTest = lCount;
  }


  if(FALSE)
  {
    SM_SPRINTF( sBuff,_T("%s"), _T( "\n Test 2 Linear sweep twice no caps \n" ) );
    smos_WriteBuffer( sBuff );

    SmBSplineCurve *pLine1 = NULL;
    SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
         SmPoint3d( 0, 0, 0 ), SmPoint3d( 0, 5, 0 ), pLine1 ) );

    SmBSplineCurve *pLine2 = NULL;
    SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
         SmPoint3d( 5, 5, 0 ), SmPoint3d( 0, 5, 0 ), pLine2 ) );

    SmBSplineCurve *pCircle1 = NULL;
    SmAxis2Placement sRefFrame1;
    sRefFrame1.SetCanonical( SmPoint3d( 3, 3, 0 ),
                           SmVector3d( -1, 0, 0 ), SmVector3d( 0, -1, 0 ) );
    SER( SmBSplineCurve::CreateCircleSegment( crContext, 3, sRefFrame1,
         0.5, 0.0, 360.0, SM_CO_QUADRATIC, pCircle1 ) );

    SmBSplineCurve *pCircle2 = NULL;
    SmAxis2Placement sRefFrame2;
    sRefFrame2.SetCanonical( SmPoint3d( 4, 4, 0 ),
                           SmVector3d( 1, 0, 0 ), SmVector3d( 0, -1, 0 ) );
    SER( SmBSplineCurve::CreateCircleSegment( crContext, 3, sRefFrame2,
         0.5, 0.0, 360.0, SM_CO_QUADRATIC, pCircle2 ) );

    SmBSplineCurve *pLine3 = NULL;
    SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
         SmPoint3d( 5, 0, 0 ), SmPoint3d( 5, 5, 0 ), pLine3 ) );

    SmBSplineCurve *pLine4 = NULL;
    SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
         SmPoint3d( 0, 0, 0 ), SmPoint3d( 5, 0, 0 ), pLine4 ) );

    SmBSplineCurve *pLine5 = NULL;
    SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
         SmPoint3d( 2, 1, 0 ), SmPoint3d( 1, 1, 0 ), pLine5 ) );

    SmBSplineCurve *pLine6 = NULL;
    SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
         SmPoint3d( 2, 1, 0 ), SmPoint3d( 2, 2, 0 ), pLine6 ) );

    SmBSplineCurve *pLine7 = NULL;
    SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
         SmPoint3d( 1, 1, 0 ), SmPoint3d( 2, 2, 0 ), pLine7 ) );

    SmTArray<SmCurve*> s3DCurves;
    s3DCurves.Add( pLine3 );
    s3DCurves.Add( pLine1 );
    s3DCurves.Add( pCircle1 );
    s3DCurves.Add( pLine6 );
    s3DCurves.Add( pLine5 );
    s3DCurves.Add( pLine2 );
    s3DCurves.Add( pLine4 );
    s3DCurves.Add( pCircle2 );
    s3DCurves.Add( pLine7 );

    DELETE_ALL_PARTS( rPartBreps );

    SmBrep *pBrep = new (crContext) SmBrep();
    SmObjDelete sCleanBrep( pBrep );
    pBrep->SetTolerance( 0.0001 );

    SmVector3d sDir( 0, 0, 1 );
    sDir.Unitize();

    SmPrimitiveCreation pPC( pBrep->GetInfiniteRegion(), 234556 );
    SER( pPC.CreateDraftSweep( s3DCurves,
         sDir,
         3.0, -2.0, // distance, angle
         TRUE ) ); // ends not capped
    SER( pBrep->ConvertCurvesToAnalytics( lNum3DCurves, lNumUVCurves ) );  // increments unlocked mark value

    sCleanBrep.Clear();
    //OFF: if cap ends is ON, GlueEdges crashes

    rPartBreps.Add( pBrep );
    pBrep->Dump();
    pBrep->ValidatePointers();

    //  # Faces = 18, # Edges = 45, # Vertices = 27
    // Wire Edges = 0,  Lamina Edges = 18,  Manifold Edges = 27
    // # Regions = 1,  # Shells = 4
    SER( pBrep->ValidateCounts( 11, 27, 18, 0, 0, 27, 2, 2 ) );
    return SM_SUCCESS;
  }

  switch(nTest)
  {
    case 1:
      {
        SM_SPRINTF( sBuff,_T("%s"), _T( "\n Test 3: rot sweep twice NO caps \n" ) );
        smos_WriteBuffer( sBuff );

        SmBSplineCurve *pLine1 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 10, 10, 10 ), SmPoint3d( 15, 10, 10 ), pLine1 ) );

        SmBSplineCurve *pLine2 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 15, 10, 10 ), SmPoint3d( 15, 15, 10 ), pLine2 ) );

        SmBSplineCurve *pLine3 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 15, 15, 10 ), SmPoint3d( 10, 15, 10 ), pLine3 ) );

        SmBSplineCurve *pLine4 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 10, 15, 10 ), SmPoint3d( 10, 10, 10 ), pLine4 ) );

        SmBSplineCurve *pCircle1 = NULL;
        SmAxis2Placement sRefFrame1;
        sRefFrame1.SetCanonical( SmPoint3d( 13, 13, 0 ),
                               SmVector3d( -1, 0, 0 ), SmVector3d( 0, -1, 0 ) );
        SER( SmBSplineCurve::CreateCircleSegment( crContext, 3, sRefFrame1,
             3.0, 0.0, 360.0, SM_CO_QUADRATIC, pCircle1 ) );

        SmBSplineCurve *pCircle2 = NULL;
        SmAxis2Placement sRefFrame2;
        sRefFrame2.SetCanonical( SmPoint3d( 14, 15, 20 ),
                               SmVector3d( -1, 0, 0 ), SmVector3d( 0, -1, 0 ) );
        SER( SmBSplineCurve::CreateCircleSegment( crContext, 3, sRefFrame2,
             2.0, 0.0, 360.0, SM_CO_QUADRATIC, pCircle2 ) );

        SmTArray<SmCurve*> s3DCurves;
        s3DCurves.Add( pCircle1 );
        s3DCurves.Add( pLine1 );
        s3DCurves.Add( pLine2 );
        s3DCurves.Add( pLine3 );
        s3DCurves.Add( pLine4 );
        s3DCurves.Add( pCircle2 );
        SmTArray<ULONG> sCounts;
        sCounts.Add( 1 );
        sCounts.Add( 4 );
        sCounts.Add( 1 );

        DELETE_ALL_PARTS( rPartBreps );

        SmBrep *pBrep = new (crContext) SmBrep();
        pBrep->SetTolerance( 0.0001 );

        SmTArray<SmFace*> sStartFaces, sSideFaces, sEndFaces;
        SmPrimitiveCreation pPC( pBrep->GetInfiniteRegion(), 54321 );
        SER( pPC.CreateSkinPrimitive( sCounts, s3DCurves, 1.0e-4, 3, TRUE,
             sStartFaces, sSideFaces, sEndFaces ) );

        //OFF: if cap ends is ON, GlueEdges crashes

        rPartBreps.Add( pBrep );
        pBrep->Dump();
        pBrep->ValidatePointers();

#ifdef SM_GFX_CODE
        // draw output
        if(smGet_DoGraphics() && pBrep)
        {
          smgfx_Erase();
          smgfx_SetLook( 1, 2, 0, 0, 1 ); pBrep->Draw();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        //   # Faces = 12, # Edges = 29, # Vertices = 17
        // Wire Edges = 1,  Lamina Edges = 12,  Manifold Edges = 16
        // # Regions = 1,  # Shells = 2
//            SER(pBrep->ValidateCounts(8,18,12,0,0,18,2,2));


      }
      break;

    case 2:
      {
        DELETE_ALL_PARTS( rPartBreps );

        SM_SPRINTF( sBuff,_T("%s"), _T( "\n Test 1: solid with 2 blocks \n" ) );
        smos_WriteBuffer( sBuff );


        SmBrep *pBrep = new (crContext) SmBrep();
        pBrep->SetTolerance( 0.0001 );

        SmAxis2Placement sRefFrame;


        SmPrimitiveCreation pPC( pBrep->GetInfiniteRegion(), 1234567 );
        sRefFrame.Translate( SmVector3d( 25, 25, 25 ) );
        SER( pPC.CreateBox( 10, 15, 20, sRefFrame ) );
        rPartBreps.Add( pBrep );

        //            SmTArray<SmCurve*> sCur;
        //            SmTArray<SmSurface*> sSur;
        //            SmTArray<long> sBoolTr;
        //            SmBrepData::WritePartToFile("TaggingTest.smp",sCur,sSur,sBoolTr,rPartBreps);
        //                

        pBrep = new (crContext) SmBrep();
        sRefFrame.Translate( SmVector3d( 5, 5, 5 ) );
        SmVector3d sAx( 1, 1, 1 );
        sAx.Unitize();
        sRefFrame.RotateAboutAxisAtPoint( 0.5, //  dAngleRadians, 
                            SmPoint3d( 30, 30, 30 ), //  sOrigin, 
                            sAx );


        pPC.Become( pBrep->GetInfiniteRegion() );
        SER( pPC.CreateBox( 10, 15, 20, sRefFrame ) );

        rPartBreps.Add( pBrep );
        pBrep->Dump();
        pBrep->ValidatePointers();

        //  # Faces = 12, # Edges = 24, # Vertices = 16
        // Wire Edges = 0,  Lamina Edges = 0,  Manifold Edges = 24
        // # Regions = 3,  # Shells = 4
        SER( pBrep->ValidateCounts( 6, 12, 8, 0, 0, 12, 2, 2 ) );

        SmAxis2Placement sRefFrame2;
        sRefFrame2.Translate( SmVector3d( -20, -20, 0 ) );
        SmBrep *pRect = SmPrimitiveCreation::CreateRectangle( crContext, SM_ZONE_TOL_3D, 5.0, 3.0, sRefFrame2 );
        rPartBreps.Add( pRect );

        sRefFrame2.Translate( SmVector3d( 5.0, 5.0, 0 ) );
        SmBrep *pCircle = SmPrimitiveCreation::CreateCircle( crContext, SM_ZONE_TOL_3D, 4.0, sRefFrame2 );

        SER( SmPrimitiveCreation::Boolean2D( pRect, pCircle, SM_2D_DIFFERENCE, pRect ) );  // note: increments unlocked mark value

        SmBrep *pBrep1;
        SER( my_test_tsurf_creation( crContext, pBrep1 ) );
        SmBrep *pBrep2;
        SER( my_test_tsurf_creation( crContext, pBrep2 ) );
        SmAxis2Placement sTranslate;
        sTranslate.Translate( SmVector3d( 3, 2.5, 0 ) );
        SER( pBrep2->Transform( sTranslate ) );
        SmVector3dAttribute *pNewAttr = new (crContext) SmVector3dAttribute( SM_AI_COLOR, SmVector3d( 0, 0, 1 ) );
        pBrep2->AddAttribute( pNewAttr );
        SER( SmPrimitiveCreation::Boolean2D( pBrep1, pBrep2, SM_2D_UNION, pBrep1 ) ); // note: increments unlocked mark value
        SER( pBrep1->ValidateCounts( 1, 25, 25, 0, 25, 0, 1, 1 ) );

        rPartBreps.Add( pBrep1 );
        //            rPartBreps.Add(pBrep2);
        pBrep1->Dump();

#ifdef SM_GFX_CODE
        // draw output
        if(smGet_DoGraphics() && pBrep)
        {
          smgfx_Erase();
          smgfx_SetLook( 1, 2, 0, 0, 1 ); pBrep->Draw();
          sm_GraphicsLoop();
        }
        if(smGet_DoGraphics() && pBrep1)
        {
          smgfx_Erase();
          smgfx_SetLook( 1, 2, 0, 0, 1 ); pBrep1->Draw();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
      }
      break;

    case 3:
      {
        SM_SPRINTF( sBuff,_T("%s"), _T( "\n Test 2 Linear sweep twice no caps \n" ) );
        smos_WriteBuffer( sBuff );

        SmBSplineCurve *pLine1 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 0, 0, 0 ), SmPoint3d( 0, 5, 0 ), pLine1 ) );

        SmBSplineCurve *pLine2 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 5, 5, 0 ), SmPoint3d( 0, 5, 0 ), pLine2 ) );

        SmBSplineCurve *pCircle1 = NULL;
        SmAxis2Placement sRefFrame1;
        sRefFrame1.SetCanonical( SmPoint3d( 3, 3, 0 ),
                               SmVector3d( -1, 0, 0 ), SmVector3d( 0, -1, 0 ) );
        SER( SmBSplineCurve::CreateCircleSegment( crContext, 3, sRefFrame1,
             0.5, 0.0, 360.0, SM_CO_QUADRATIC, pCircle1 ) );

        SmBSplineCurve *pCircle2 = NULL;
        SmAxis2Placement sRefFrame2;
        sRefFrame2.SetCanonical( SmPoint3d( 4, 4, 0 ),
                               SmVector3d( 1, 0, 0 ), SmVector3d( 0, -1, 0 ) );
        SER( SmBSplineCurve::CreateCircleSegment( crContext, 3, sRefFrame2,
             0.5, 0.0, 360.0, SM_CO_QUADRATIC, pCircle2 ) );

        SmBSplineCurve *pLine3 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 5, 0, 0 ), SmPoint3d( 5, 5, 0 ), pLine3 ) );

        SmBSplineCurve *pLine4 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 0, 0, 0 ), SmPoint3d( 5, 0, 0 ), pLine4 ) );

        SmBSplineCurve *pLine5 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 2, 1, 0 ), SmPoint3d( 1, 1, 0 ), pLine5 ) );

        SmBSplineCurve *pLine6 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 2, 1, 0 ), SmPoint3d( 2, 2, 0 ), pLine6 ) );

        SmBSplineCurve *pLine7 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 1, 1, 0 ), SmPoint3d( 2, 2, 0 ), pLine7 ) );

        SmTArray<SmCurve*> s3DCurves;
        s3DCurves.Add( pLine3 );
        s3DCurves.Add( pLine1 );
        s3DCurves.Add( pCircle1 );
        s3DCurves.Add( pLine6 );
        s3DCurves.Add( pLine5 );
        s3DCurves.Add( pLine2 );
        s3DCurves.Add( pLine4 );
        s3DCurves.Add( pCircle2 );
        s3DCurves.Add( pLine7 );



        DELETE_ALL_PARTS( rPartBreps );

        SmBrep *pBrep = new (crContext) SmBrep();
        SmObjDelete sCleanBrep( pBrep );
        pBrep->SetTolerance( 0.0001 );

        SmVector3d sDir( 1, 1, 1 );
        sDir.Unitize();

        SmPrimitiveCreation pPC( pBrep->GetInfiniteRegion(), 234556 );
        SER( pPC.CreateLinearSweep( s3DCurves,
             sDir,
             3.0, // distance
             1, // repetions
             TRUE ) ); // ends not capped
        SER( pBrep->ConvertCurvesToAnalytics( lNum3DCurves, lNumUVCurves ) );  // increments unlocked mark value

        sCleanBrep.Clear();
        //OFF: if cap ends is ON, GlueEdges crashes

        rPartBreps.Add( pBrep );
        pBrep->Dump();
        pBrep->ValidatePointers();

#ifdef SM_GFX_CODE
        // draw output
        if(smGet_DoGraphics() && pBrep)
        {
          smgfx_Erase();
          smgfx_SetLook( 1, 2, 0, 0, 1 ); pBrep->Draw();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        //  # Faces = 18, # Edges = 45, # Vertices = 27
        // Wire Edges = 0,  Lamina Edges = 18,  Manifold Edges = 27
        // # Regions = 1,  # Shells = 4
        SER( pBrep->ValidateCounts( 11, 27, 18, 0, 0, 27, 2, 2 ) );

      }
      break;

    case 4:
      {
        SM_SPRINTF( sBuff,_T("%s"), _T( "\n Test 3: rot sweep twice NO caps \n" ) );
        smos_WriteBuffer( sBuff );

        SmBSplineCurve *pLine1 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 0, 0, 0 ), SmPoint3d( 0, 5, 0 ), pLine1 ) );

        SmBSplineCurve *pLine2 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 5, 5, 0 ), SmPoint3d( 0, 5, 0 ), pLine2 ) );

        //            SmBSplineCurve *pCircle1 = NULL ;
        //            SmAxis2Placement sRefFrame1;
        //            sRefFrame1.SetCanonical(SmPoint3d(3,3,0),
        //                                   SmVector3d(-1,0,0),SmVector3d(0,-1,0));
        //            SER(SmBSplineCurve::CreateCircleSegment(crContext,3,sRefFrame1,
        //                                  0.5,0.0,360.0,  SM_CO_QUADRATIC,pCircle1));
        // 
        //            SmBSplineCurve *pCircle2 = NULL ;
        //            SmAxis2Placement sRefFrame2;
        //            sRefFrame2.SetCanonical(SmPoint3d(4,4,0),
        //                                   SmVector3d(1,0,0),SmVector3d(0,-1,0));
        //            SER(SmBSplineCurve::CreateCircleSegment(crContext,3,sRefFrame2,
        //                                  0.5,0.0,360.0,  SM_CO_QUADRATIC,pCircle2));
        //
        SmBSplineCurve *pLine3 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 5, 0, 0 ), SmPoint3d( 5, 5, 0 ), pLine3 ) );

        SmBSplineCurve *pLine4 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 0, 0, 0 ), SmPoint3d( 5, 0, 0 ), pLine4 ) );

        SmBSplineCurve *pLine5 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 2, 1, 0 ), SmPoint3d( 1, 1, 0 ), pLine5 ) );

        SmBSplineCurve *pLine6 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 2, 1, 0 ), SmPoint3d( 2, 2, 0 ), pLine6 ) );

        SmBSplineCurve *pLine7 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 1, 1, 0 ), SmPoint3d( 2, 2, 0 ), pLine7 ) );

        SmTArray<SmCurve*> s3DCurves;
        s3DCurves.Add( pLine3 );
        s3DCurves.Add( pLine1 );
        //OFF             s3DCurves.Add(pCircle1);
        s3DCurves.Add( pLine6 );
        s3DCurves.Add( pLine5 );
        s3DCurves.Add( pLine2 );
        s3DCurves.Add( pLine4 );
        //OFF             s3DCurves.Add(pCircle2);
        s3DCurves.Add( pLine7 );



        DELETE_ALL_PARTS( rPartBreps );

        SmBrep *pBrep = new (crContext) SmBrep();
        pBrep->SetTolerance( 0.0001 );

        SmPrimitiveCreation pPC( pBrep->GetInfiniteRegion(), 54321 );
        SER( pPC.CreateRotationalSweep( s3DCurves,
             SmPoint3d( 0, 0, 0 ), // rot center
             SmVector3d( 0, 1, 0 ), // rot dir
             20,                // angle grad
             1,                 // repetitions
             TRUE ) ); // ends capped

        rPartBreps.Add( pBrep );
        pBrep->Dump();
        pBrep->ValidatePointers();

#ifdef SM_GFX_CODE
        // draw output
        if(smGet_DoGraphics() && pBrep)
        {
          smgfx_Erase();
          smgfx_SetLook( 1, 2, 0, 0, 1 ); pBrep->Draw();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        //   # Faces = 12, # Edges = 29, # Vertices = 17
        // Wire Edges = 1,  Lamina Edges = 12,  Manifold Edges = 16
        // # Regions = 1,  # Shells = 2
        SER( pBrep->ValidateCounts( 8, 18, 12, 0, 0, 18, 2, 2 ) );


      }
      break;

    case 5:
      {
        SM_SPRINTF( sBuff,_T("%s"), _T( "\n Test 4 2 Cones \n" ) );
        smos_WriteBuffer( sBuff );

        DELETE_ALL_PARTS( rPartBreps );

        SmBrep *pBrep = new (crContext) SmBrep();
        pBrep->SetTolerance( 0.0001 );


        SmAxis2Placement sRefFrame1;
        sRefFrame1.SetCanonical( SmPoint3d( 20, 0, 0 ),
                    SmVector3d( 0, 0, 1 ), SmVector3d( 1, 0, 0 ) );
        SmPrimitiveCreation pPC( pBrep->GetInfiniteRegion(), 123455 );
        SER( pPC.CreateCone( 10, // height
             1, // base rad
             2, // top rad
             0, // start angle
             360, // end angle
             sRefFrame1 ) );

        SmAxis2Placement sRefFrame;
        sRefFrame.SetCanonical( SmPoint3d( 0, 0, 0 ),
                    SmVector3d( 0, 0, 1 ), SmVector3d( 1, 0, 0 ) );
        pPC.Become( pBrep->GetInfiniteRegion() );
        SER( pPC.CreateCone( 10, // height
             1, // base rad
             2, // top rad
             30, // start angle
             60, // end angle
             sRefFrame ) );


        pBrep->ValidatePointers();


        rPartBreps.Add( pBrep );
        pBrep->Dump();

#ifdef SM_GFX_CODE
        // draw output
        if(smGet_DoGraphics() && pBrep)
        {
          smgfx_Erase();
          smgfx_SetLook( 1, 2, 0, 0, 1 ); pBrep->Draw();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        // # Faces = 9, # Edges = 15, # Vertices = 10
        // Wire Edges = 0,  Lamina Edges = 1,  Manifold Edges = 11
        // # Regions = 3,  # Shells = 4
//            SER(pBrep->ValidateCounts(8,12,8,0,0,12,3,4));


      }
      break;

    case 6:
      {
        SM_SPRINTF( sBuff,_T("%s"), _T( "\n Test 5: 2 shperes \n" ) );
        smos_WriteBuffer( sBuff );

        DELETE_ALL_PARTS( rPartBreps );

        SmAxis2Placement sRefFrame1;
        sRefFrame1.SetCanonical( SmPoint3d( 20, 0, 0 ),
                    SmVector3d( 1, 0, 0 ), SmVector3d( 0, 1, 0 ) );
        SmBrep *pBrep2 = new (crContext) SmBrep();
        pBrep2->SetTolerance( 0.0001 );
        SmPrimitiveCreation pPC2( pBrep2->GetInfiniteRegion(), 780342 );
        SER( pPC2.CreateSphere( 10, // radius
             0, // start angle
             180, // end angle
             sRefFrame1 ) );


        SmBrep *pBrep = new (crContext) SmBrep();
        pBrep->SetTolerance( 0.0001 );

        SmAxis2Placement sRefFrame;
        sRefFrame.SetCanonical( SmPoint3d( 0, 0, 0 ),
                    SmVector3d( 1, 0, 0 ), SmVector3d( 0, 1, 0 ) );
        SmPrimitiveCreation pPC( pBrep->GetInfiniteRegion() );
        SER( pPC.CreateSphere( 10, // radius
             30, // start angle
             60, // end angle
             sRefFrame ) );


        pBrep->ValidatePointers();




        rPartBreps.Add( pBrep );
        rPartBreps.Add( pBrep2 );
        pBrep->Dump();

#ifdef SM_GFX_CODE
        // draw output
        if(smGet_DoGraphics() && pBrep)
        {
          smgfx_Erase();
          smgfx_SetLook( 1, 2, 0, 0, 1 ); pBrep->Draw();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        // # Faces = 9, # Edges = 15, # Vertices = 10
        // Wire Edges = 0,  Lamina Edges = 1,  Manifold Edges = 11
        // # Regions = 3,  # Shells = 4
        //SER(pBrep->ValidateCounts(9,15,10,0,1,11,3,4));



      }
      break;

    case 7:
      {
        SM_SPRINTF( sBuff,_T("%s"), _T( "\n Test 5: 2 shperes \n" ) );
        smos_WriteBuffer( sBuff );

        DELETE_ALL_PARTS( rPartBreps );

        SmBrep *pBrep = new (crContext) SmBrep();
        pBrep->SetTolerance( 0.0001 );

        SmPrimitiveCreation pPC( pBrep->GetInfiniteRegion(), 1324356 );

        SmAxis2Placement sRefFrame1;
        sRefFrame1.SetCanonical( SmPoint3d( 30, 0, 0 ),
                    SmVector3d( 1, 0, 0 ), SmVector3d( 0, 1, 0 ) );
        SER( pPC.CreateTorus( 20, // major radius
             5, // minor radius
             0, // start angle
             360, // end angle
             sRefFrame1 ) );


        SmAxis2Placement sRefFrame;
        sRefFrame.SetCanonical( SmPoint3d( 0, 0, 0 ),
                    SmVector3d( 1, 0, 0 ), SmVector3d( 0, 1, 0 ) );
        pPC.Become( pBrep->GetInfiniteRegion() );
        SER( pPC.CreateTorus( 20, // major radius
             5, // minor radius
             30, // start angle
             60, // end angle
             sRefFrame ) );


        pBrep->ValidatePointers();

        rPartBreps.Add( pBrep );
        pBrep->Dump();

#ifdef SM_GFX_CODE
        // draw output
        if(smGet_DoGraphics() && pBrep)
        {
          smgfx_Erase();
          smgfx_SetLook( 1, 2, 0, 0, 1 ); pBrep->Draw();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        // # Faces = 9, # Edges = 15, # Vertices = 10
        // Wire Edges = 0,  Lamina Edges = 1,  Manifold Edges = 11
        // # Regions = 3,  # Shells = 4
        //SER(pBrep->ValidateCounts(9,15,10,0,1,11,3,4));


      }
      break;

    case 8:
      {
        SM_SPRINTF( sBuff,_T("%s"), _T( "\n Test 8: Curve Sweep \n" ) );
        smos_WriteBuffer( sBuff );

        SmBSplineCurve *pLine1 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 10, 10, 0 ), SmPoint3d( 15, 10, 0 ), pLine1 ) );

        SmBSplineCurve *pLine2 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 15, 15, 0 ), SmPoint3d( 15, 10, 0 ), pLine2 ) );

        SmBSplineCurve *pCircle1 = NULL;
        SmAxis2Placement sRefFrame1;
        sRefFrame1.SetCanonical( SmPoint3d( 12, 12, 0 ),
                               SmVector3d( -1, 0, 0 ), SmVector3d( 0, -1, 0 ) );
        SER( SmBSplineCurve::CreateCircleSegment( crContext, 3, sRefFrame1,
             1.0, 0.0, 360.0, SM_CO_QUADRATIC, pCircle1 ) );

        SmBSplineCurve *pCircle2 = NULL;
        SmAxis2Placement sRefFrame2;
        sRefFrame2.SetCanonical( SmPoint3d( 14, 14, 0 ),
                               SmVector3d( 1, 0, 0 ), SmVector3d( 0, -1, 0 ) );
        SER( SmBSplineCurve::CreateCircleSegment( crContext, 3, sRefFrame2,
             0.8, 0.0, 360.0, SM_CO_QUADRATIC, pCircle2 ) );

        SmBSplineCurve *pLine3 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 10, 15, 0 ), SmPoint3d( 15, 15, 0 ), pLine3 ) );

        SmBSplineCurve *pLine4 = NULL;
        SER( SmBSplineCurve::CreateLineSegment( crContext, 3,
             SmPoint3d( 10, 10, 0 ), SmPoint3d( 10, 15, 0 ), pLine4 ) );

        //SmBSplineCurve *pLine5 = NULL ;
        //SER(SmBSplineCurve::CreateLineSegment(crContext,3,
        //    SmPoint3d(2,1,0),SmPoint3d(1,1,0), pLine5));

        //SmBSplineCurve *pLine6 = NULL ;
        //SER(SmBSplineCurve::CreateLineSegment(crContext,3,
        //    SmPoint3d(2,1,0),SmPoint3d(2,2,0), pLine6));

        //SmBSplineCurve *pLine7 = NULL ;
        //SER(SmBSplineCurve::CreateLineSegment(crContext,3,
        //    SmPoint3d(1,1,0),SmPoint3d(2,2,0), pLine7));

        SmTArray<SmCurve*> s3DCurves;
        s3DCurves.Add( pLine3 );
        s3DCurves.Add( pLine1 );
        //OFF             s3DCurves.Add(pCircle1);
        //            s3DCurves.Add(pLine6);
        //            s3DCurves.Add(pLine5);
        s3DCurves.Add( pLine2 );
        s3DCurves.Add( pLine4 );
        s3DCurves.Add( pCircle1 );
        s3DCurves.Add( pCircle2 );
        //            s3DCurves.Add(pLine7);


        SmTArray<SmPoint3d> sPnts1;
        sPnts1.Add( SmPoint3d( 0, 0.1, 0 ) );
        sPnts1.Add( SmPoint3d( 0, 0.15, 1 ) );
        sPnts1.Add( SmPoint3d( 0.0, 0.2, 1.5 ) );
        sPnts1.Add( SmPoint3d( 0.14, 0.3, 2.0 ) );
        sPnts1.Add( SmPoint3d( 0.2, 0.4, 2.5 ) );
        sPnts1.Add( SmPoint3d( 0.22, 0.5, 3.0 ) );
        SmBSplineCurve *pNurb1 = my_create_nurb( crContext, SmPoint3d( -3, 0, 0 ), sPnts1, 3, 0, 0, 0 );
        SmObjDelete sCleanNurb1( pNurb1 );
        SmAxis2Placement sPlace;
        SmVector3d sScale( 10, 10, 10 );
        pNurb1->Transform( sPlace, &sScale );

        DELETE_ALL_PARTS( rPartBreps );

        SmBrep *pBrep = new (crContext) SmBrep();
        pBrep->SetTolerance( 0.0001 );

        SmBrep *pResult;
        SmTArray<SmBoolean> sInputOwnershipTransferred;
        SER( SmPrimitiveCreation::OffsetProfile( crContext, s3DCurves, 0.0001, 0.8, 1, TRUE, TRUE,
                                                 pResult, sInputOwnershipTransferred ) );
        SmObjDelete sClean( pResult );
        pResult->GetCurves( s3DCurves );
        for(ULONG ii = 0; ii < s3DCurves.GetSize(); ii++)
        {
          SmCurve *pNewCurve;
          s3DCurves[ii]->Copy( crContext, pNewCurve );
          s3DCurves[ii] = pNewCurve;
        }

        SmSweepOptions sOptions;
        sOptions.bCapEndsArg = TRUE;
        sOptions.bTranslationalSweep = FALSE;
        sOptions.bMoveProfileCenterToPath = TRUE;
        sOptions.bOrientProfilePerpendicularToPath = TRUE;
        //sOptions.bKeepProfileLocationb = FALSE;

        SmObjsDelete<SmCurve*> sClean3DCurves( &s3DCurves );
        SmTArray<SmFace*> sStartFaces, sSideFaces, sEndFaces;
        SmPrimitiveCreation pPC( pBrep->GetInfiniteRegion(), 54321 );
        SER( pPC.CreateCurveSweep( s3DCurves, pNurb1, NULL, NULL, 1.0e-4, &sOptions,
             sStartFaces, sSideFaces, sEndFaces ) );
        //OFF: if cap ends is ON, GlueEdges crashes

        rPartBreps.Add( pBrep );
        pBrep->Dump();
        pBrep->ValidatePointers();

#ifdef SM_GFX_CODE
        // draw output
        if(smGet_DoGraphics() && pBrep)
        {
          pBrep->Dump() ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); pBrep->Draw(TRUE);
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        SER( pBrep->ValidateCounts( 16, 42, 28, 0, 0, 42, 2, 2 ) );

      }
      break;

    case 9:
      {
        SM_SPRINTF( sBuff,_T("%s"), _T( "\n Test 1 \n" ) );
        smos_WriteBuffer( sBuff );

        DELETE_ALL_PARTS( rPartBreps );

        SmBrep *pBrep = new (crContext) SmBrep();
        pBrep->SetTolerance( 0.0001 );

      }
      break;

    default:
      {
        SM_SPRINTF( sBuff,_T("%s"), _T( "\n Test 1 \n" ) );
        smos_WriteBuffer( sBuff );

        DELETE_ALL_PARTS( rPartBreps );

        SmBrep *pBrep = new (crContext) SmBrep();
        pBrep->SetTolerance( 0.0001 );

      }
  }

  // all done
  return SM_SUCCESS;

} // end my_primsol_demo

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
SmStatus my_primsol_regression()
{

  MYPRINTF( _T( "\nEntered: my_primsol_regression 1" ) );

  TCHAR sBuff[SM_TBLOCK_SIZE];
  SmContext sContext;
  SmTArray<SmBrep*> sBreps;
  for(ULONG i = 0; i < my_primsol_demo_count(); i++)
  {
    SM_SPRINTF( sBuff, _T( "\nEntered: my_primsol_demo %ld" ), i );
    MYPRINTF( sBuff );
    my_primsol_demo( sContext, sBreps, 1000 );
  }

  DELETE_ALL_PARTS( sBreps );

  // CreateCylindricalBox rejects invalid dimensions with SM_ERR_INVALID_INPUT
  // and builds nothing, in both debug and release builds.
  struct { double dLen, dIn, dOut, dStart, dEnd; SmStatus eExpect; } sCases[] =
  {
    { 10.0, 2.0, 5.0,   0.0,  90.0, SM_SUCCESS           },
    {  0.0, 2.0, 5.0,   0.0,  90.0, SM_ERR_INVALID_INPUT },  // zero length
    { 10.0, 0.0, 5.0,   0.0,  90.0, SM_ERR_INVALID_INPUT },  // zero inside radius
    { 10.0, 5.0, 2.0,   0.0,  90.0, SM_ERR_INVALID_INPUT },  // outside <= inside
    { 10.0, 2.0, 5.0, 400.0, 450.0, SM_ERR_INVALID_INPUT },  // start angle out of range
    { 10.0, 2.0, 5.0,  90.0,  10.0, SM_ERR_INVALID_INPUT },  // end before start
    { 10.0, 2.0, 5.0,   0.0, 400.0, SM_ERR_INVALID_INPUT },  // sweep over 360
  };
  SmAxis2Placement sFrame;
  for( ULONG ii = 0; ii < sizeof(sCases) / sizeof(sCases[0]); ii++ )
  {
    SmBrep* pBrep = new (sContext) SmBrep();
    SmPrimitiveCreation sPC( pBrep->GetInfiniteRegion() );
    SmStatus eStat = sPC.CreateCylindricalBox( sCases[ii].dLen, sCases[ii].dIn, sCases[ii].dOut,
                                               sCases[ii].dStart, sCases[ii].dEnd, sFrame );
    ULONG lFaces = pBrep->GetNumFaces();
    delete pBrep;
    if( eStat != sCases[ii].eExpect
        || ( eStat == SM_SUCCESS && lFaces == 0 )
        || ( eStat != SM_SUCCESS && lFaces != 0 ) )
    {
      SM_SPRINTF( sBuff, _T( "\nmy_primsol_regression: CreateCylindricalBox case %lu returned %ld (expected %ld), %lu faces" ),
                  ii, (long)eStat, (long)sCases[ii].eExpect, lFaces );
      MYPRINTF( sBuff );
      return SM_ERR;
    }
  }

  return SM_SUCCESS;

} // end my_primsol_regression


/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
ULONG my_primsol_demo_count(void)
{
    return 8;

} // end my_primsol_demo_count

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
ULONG my_sweep_demo_count(void)
{
    return 35;
} // end my_sweep_demo_count


/***********************************************************************
PURPOSE --- Run a specific test (nTest) or all tests in turn 
            (nTest omitted or -1)

USAGE NOTES --- Used by prog_test via my_sweep_regression
***********************************************************************/
SmStatus my_sweep_demo
  (const SmContext & crContext,     // in : context for new object construction
   SmTArray<SmBrep*> & rPartBreps,  // out: resulting breps
   ULONG nTestArg)              
{
  MYPRINTF(_T("\nEntered in my_sweep_demo")) ;

  TCHAR   sBuff[SM_TBLOCK_SIZE];
  ULONG   nTest = nTestArg;
  SmBrep *pBrep = NULL;

//    {
//        SmBrep * pNewBrep;
//        my_test_tsurf_cyl_creation_fast(crContext,pNewBrep);
//        rPartBreps.Add(pNewBrep);
//        return SM_SUCCESS;
//    }

  switch (nTest) 
    {
      case 1:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Translate Vertices higher, Edges same"));
          smos_WriteBuffer(sBuff);

          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmVector3d sVec(0,0,10);
          SmTArray<SmVertex*> sVertices;
          pBrep->GetShellVertices(sVertices);
          SmTArray<SmEdge*> sEdges;
          pBrep->GetEdges(sEdges);
          SmTArray<SmFace*> sFaces;
          pBrep->GetFaces(sFaces);
          SmTArray<SmFace*> sSameFaces;
          if(sFaces.GetSize() < 1) return(SM_ERR);
          sSameFaces.Add(sFaces.GetLast());
          sFaces.RemoveLast();
          SmTranslationalSweepGeometry sTSG(sVec);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sTSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
          sTopoSweep.SetDoMerge(TRUE);
          sTopoSweep.SetRepetitions(3);

#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),   // note: increments unlocked mark value
                                 NULL, NULL, &sVertices,  // sweep higher targets
                                 NULL, &sEdges, NULL));   // sweep same targets
                                 
          pBrep->Dump();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          //cbi This test is not working right.
          //cbi In DoSweep(), PiecewiseMerge() does only Faces, ignores wires/shell verts.
          //cbi pBrep->ValidateCounts(16,37,26,2,3,27,3,4);
        }
        break;
      


      case 2:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Translate Vertices same"));
          smos_WriteBuffer(sBuff);

          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
           SmVector3d sVec(0,0,10);
          SmTArray<SmVertex*> sVertices;
          pBrep->GetVertices(sVertices);
          SmTranslationalSweepGeometry sTSG(sVec);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sTSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep, pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 NULL, NULL, NULL,           // sweep higher targets
                                 NULL, NULL, &sVertices));   // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,NULL,NULL,NULL,NULL,&sVertices);


          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);

#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          // # Faces = 2, # Edges = 12, # Vertices = 26
          // Wire Edges = 1,  Lamina Edges = 10,  Manifold Edges = 1
          // # Regions = 1,  # Shells = 15
          pBrep->ValidateCounts(2,12,26,1,10,1,1,15);
        }
      break;

      case 3:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Translate Vertices higher"));
          smos_WriteBuffer(sBuff);
    
          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmVector3d sVec(0,0,10);
          SmTArray<SmVertex*> sVertices;
          pBrep->GetVertices(sVertices);
          SmTranslationalSweepGeometry sTSG(sVec);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sTSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep, pBrep->GetInfiniteRegion(),     // note: increments unlocked mark value
                                 NULL, NULL, &sVertices,  // sweep higher targets
                                 NULL, NULL, NULL));      // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,NULL,&sVertices,NULL,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE


          //   # Faces = 2, # Edges = 25, # Vertices = 26
          // Wire Edges = 14,  Lamina Edges = 10,  Manifold Edges = 1
          //     # Regions = 1,  # Shells = 2

          pBrep->ValidateCounts(2,25,26,14,10,1,1,2);
        }
        break;

      case 4:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Translate Edges same"));
          smos_WriteBuffer(sBuff);

          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmVector3d sVec(0,0,10);
          SmTArray<SmEdge*> sEdges;
          pBrep->GetEdges(sEdges);
          SmTranslationalSweepGeometry sTSG(sVec);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sTSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep, pBrep->GetInfiniteRegion(),     // note: increments unlocked mark value
                                 NULL, NULL, NULL,         // sweep higher targets
                                 NULL, &sEdges, NULL));    // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,NULL,NULL,NULL,&sEdges,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE


          //    # Faces = 2, # Edges = 24, # Vertices = 24
          //    Wire Edges = 13,  Lamina Edges = 10,  Manifold Edges = 1
          //    # Regions = 1,  # Shells = 5            
          pBrep->ValidateCounts(2,24,24,13,10,1,1,5);
        }
        break;

      case 5:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Translate Edges higher"));
          smos_WriteBuffer(sBuff);

          // edges high
          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmVector3d sVec(0,0,10);
          SmTArray<SmEdge*> sEdges;
          pBrep->GetEdges(sEdges);
          SmTranslationalSweepGeometry sTSG(sVec);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sTSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 NULL, &sEdges, NULL,  // sweep higher targets
                                 NULL, NULL, NULL));   // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,&sEdges,NULL,NULL,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          pBrep->ValidateCounts(14,35,24,0,14,17,1,2);
        }
        break;

      case 6:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Translate Faces same"));
          smos_WriteBuffer(sBuff);

          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmVector3d sVec(0,0,10);
          SmTArray<SmFace*> sFaces;
          pBrep->GetFaces(sFaces);
          SmTranslationalSweepGeometry sTSG(sVec);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sTSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 NULL, NULL, NULL,       // sweep higher targets
                                 &sFaces, NULL, NULL));  // sweep same targets  

          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,NULL,NULL,&sFaces,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          pBrep->ValidateCounts(4,23,24,1,20,2,1,3);
        }
        break;

      case 7:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Translate 2 faces high"));
          smos_WriteBuffer(sBuff);

          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmVector3d sVec(0,0,10);
          SmTArray<SmFace*> sFaces;
          pBrep->GetFaces(sFaces);
          SmTranslationalSweepGeometry sTSG(sVec);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sTSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
          sTopoSweep.SetDoMerge(TRUE);
          //SmTopologySweep sTopoSweep(sTSG, FALSE);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 &sFaces, NULL, NULL,    // sweep higher targets
                                 NULL, NULL, NULL));     // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 &sFaces,NULL,NULL,NULL,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          // # Faces = 15, # Edges = 34, # Vertices = 24
          // Wire Edges = 2,  Lamina Edges = 0,  Manifold Edges = 28
          // # Regions = 3,  # Shells = 4


          // Due to a fix in face_high, the number of edges has increased
          // to the correct value. The original count of 33 was incorrect.
          // Also, there is one more wire edge now.
          pBrep->ValidateCounts(15,34,26,2,0,28,3,5);

        }
        break;

      case 8:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Translate Whole-Brep, non-manifold Faces/Wire/Vertex"));
          smos_WriteBuffer(sBuff);

          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmVector3d sVec(0,0,10);
          SmTArray<SmEdge*> sEdges;
          pBrep->GetEdges(sEdges);
          SmTranslationalSweepGeometry sTSG(sVec);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sTSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
          sTopoSweep.SetDoMerge(TRUE);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 NULL, NULL, NULL,     // sweep higher targets
                                 NULL, NULL, NULL));   // sweep same targets  

          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,NULL,NULL,NULL,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop() ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          //lCount = 0;  faces, edges, vertices, wireEdges, laminaEdges, manifoldEdges, regions, shells
          // We have changed how sweep behaves when sweeping the whole Brep
          // (no Face/Edge/Vertex targets passed in):
          // Do not create interior Faces from Edges interior to a swept Face,
          // nor interior wire Edges from shell Vertices in a swept Face.
       // pBrep->ValidateCounts(16,37,28,2,3,27,3,5);
          pBrep->ValidateCounts(15,36,26,1,3,31,2,3);
        }
        break;

      case 9:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate Vertices same\n"));
          smos_WriteBuffer(sBuff);

          // vertices same
          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmPoint3d sCenter(-10, 0, 0);
          SmVector3d sAxis(0, 1, 0);
          double dAngle = 30;
          double dEps = pBrep->GetTolerance();
          SmTArray<SmVertex*> sVertices;
          pBrep->GetVertices(sVertices);
          SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
//            sTopoSweep.SetDoMerge(TRUE);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 NULL, NULL, NULL,           // sweep higher targets
                                 NULL, NULL, &sVertices));   // sweep same targets  

          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,NULL,NULL,NULL,NULL,&sVertices);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          pBrep->ValidateCounts(2, 12, 26, 1, 10, 1, 1, 15);
        }
        break;

      case 10:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate Vertices higher 3x \n"));
          smos_WriteBuffer(sBuff);

          // vertices high three times
          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmPoint3d sCenter(-10, 0, 0);
          SmVector3d sAxis(0, 1, 0);
          double dAngle = 30;
          double dEps = pBrep->GetTolerance();
          SmTArray<SmVertex*> sVertices;
          pBrep->GetVertices(sVertices);
          SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 NULL, NULL, &sVertices, // sweep higher targets
                                 NULL, NULL, NULL));     // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,NULL,&sVertices, // high
                                 NULL,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          //  # Faces = 2, # Edges = 25, # Vertices = 26
          //    Wire Edges = 14,  Lamina Edges = 10,  Manifold Edges = 1
          //    # Regions = 1,  # Shells = 2

          pBrep->ValidateCounts( 2,25,26,14, 10, 1, 1, 2);
        }
        break;

      case 11:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate Vertices higher full circle\n"));
          smos_WriteBuffer(sBuff);

          // vertices high, full circle
          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmPoint3d sCenter(-10, 0, 0);
          SmVector3d sAxis(0, 1, 0);
          double dAngle = 360;
          double dEps = pBrep->GetTolerance();
          SmTArray<SmVertex*> sVertices;
          pBrep->GetVertices(sVertices);
          SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 NULL, NULL, &sVertices,   // sweep higher targets
                                 NULL, NULL, NULL));       // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,NULL,&sVertices,NULL,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
                        
          // # Faces = 2, # Edges = 25, # Vertices = 13
          //Wire Edges = 14,  Lamina Edges = 10,  Manifold Edges = 1
          //# Regions = 1,  # Shells = 2

          pBrep->ValidateCounts(2,25,13,14,10,1,1,2);
        }
        break;

      case 12:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate Edges same\n"));
          smos_WriteBuffer(sBuff);

          // edges same
          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmPoint3d sCenter(-10, 0, 0);
          SmVector3d sAxis(0, -1, 0);
          double dAngle = 36;
          double dEps = pBrep->GetTolerance();
          SmTArray<SmEdge*> sEdges;
          pBrep->GetEdges(sEdges);
          SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 NULL, NULL, NULL,       // sweep higher targets
                                 NULL, &sEdges, NULL));  // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,NULL,NULL,NULL,&sEdges,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          // # Faces = 2, # Edges = 24, # Vertices = 24
          // Wire Edges = 13,  Lamina Edges = 10,  Manifold Edges = 1
          // # Regions = 1,  # Shells = 5

          pBrep->ValidateCounts(2, 24,24,13,10,1,1,5);
        }
        break;

      case 13:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate Faces same 3x\n"));
          smos_WriteBuffer(sBuff);

          // faces same
          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmPoint3d sCenter(-10, 0, 0);
          SmVector3d sAxis(0, -1, 0);
          double dAngle = 36;
          double dEps = pBrep->GetTolerance();
          SmTArray<SmFace*> sFaces;
          pBrep->GetFaces(sFaces);
          SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
          sTopoSweep.SetRepetitions(3);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 NULL, NULL, NULL,      // sweep higher targets
                                 &sFaces, NULL, NULL)); // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,NULL,NULL,&sFaces,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          //  # Faces = 8, # Edges = 45, # Vertices = 46
          //  Wire Edges = 1,  Lamina Edges = 40,  Manifold Edges = 4
          //  # Regions = 1,  # Shells = 5

          pBrep->ValidateCounts(8, 45,46,1,40,4,1,5);
        }
        break;

      case 14:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate Vertices higher 3x\n"));
          smos_WriteBuffer(sBuff);

          // vertices high three times
          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmPoint3d sCenter(-10, 0, 0);
          SmVector3d sAxis(0, 1, 0);
          double dAngle = 30;
          double dEps = pBrep->GetTolerance();
          SmTArray<SmVertex*> sVertices;
          pBrep->GetVertices(sVertices);
          SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 NULL, NULL, &sVertices, // sweep higher targets
                                 NULL, NULL, NULL));     // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,NULL,&sVertices, // high
                                 NULL,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          //  # Faces = 2, # Edges = 25, # Vertices = 26
          //    Wire Edges = 14,  Lamina Edges = 10,  Manifold Edges = 1
          //    # Regions = 1,  # Shells = 2

          pBrep->ValidateCounts( 2,25,26,14, 10, 1, 1, 2);
        }
        break;

      case 15:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate Faces same 3x, far axis\n"));
          smos_WriteBuffer(sBuff);

          // faces same
          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmPoint3d sCenter(-10, 0, 0);
          SmVector3d sAxis(0, -1, 0);
          double dAngle = 36;
          double dEps = pBrep->GetTolerance();
          SmTArray<SmFace*> sFaces;
          pBrep->GetFaces(sFaces);
          SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
          sTopoSweep.SetRepetitions(3);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 NULL, NULL, NULL,        // sweep higher targets
                                 &sFaces, NULL, NULL));   // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,NULL,NULL,&sFaces,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          // # Faces = 8, # Edges = 45, # Vertices = 46
          // Wire Edges = 1,  Lamina Edges = 40,  Manifold Edges = 4
          // # Regions = 1,  # Shells = 5

          pBrep->ValidateCounts(8,45,46,1,40,4,1,5);
        }
        break;

      case 16:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate Faces same through Edge\n"));
          smos_WriteBuffer(sBuff);

          // faces same
          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmPoint3d sCenter(-0, 0, 0);
          SmVector3d sAxis(0, -1, 0);
          double dAngle = 36;
          double dEps = pBrep->GetTolerance();
          SmTArray<SmFace*> sFaces;
          pBrep->GetFaces(sFaces);
          SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
          sTopoSweep.SetRepetitions(3);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 NULL, NULL, NULL,       // sweep higher targets
                                 &sFaces, NULL, NULL));  // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,NULL,NULL,&sFaces,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          // # Faces = 8, # Edges = 42, # Vertices = 40
          // Wire Edges = 1,  Lamina Edges = 36,  Manifold Edges = 4
          //  # Regions = 1,  # Shells = 2

          pBrep->ValidateCounts(8, 42,40,1,36,4,1,2);
        }
        break;

      case 17:
        {

          SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate Faces same through INTERN Edge\n"));
          smos_WriteBuffer(sBuff);

          // faces same
          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmPoint3d sCenter(1, 8, 0);
          SmVector3d sAxis(3, -3, 0);
          double dAngle = 36;
          double dEps = pBrep->GetTolerance();
          SmTArray<SmFace*> sFaces;
          pBrep->GetFaces(sFaces);
          SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
          sTopoSweep.SetRepetitions(3);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 NULL, NULL, NULL,        // sweep higher targets
                                 &sFaces, NULL, NULL));   // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,NULL,NULL,&sFaces,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          //   # Faces = 8, # Edges = 42, # Vertices = 40
          //   Wire Edges = 1,  Lamina Edges = 36,  Manifold Edges = 4
          //   # Regions = 1,  # Shells = 2

          pBrep->ValidateCounts(8, 42,40,1,36,4,1,2);

        }
        break;

      case 18:
        {

          SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate Faces same through 2 INTERN Edges\n"));
          smos_WriteBuffer(sBuff);

          // faces same
          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmPoint3d sCenter(0, 5, 0);
          SmVector3d sAxis(1, 0, 0);
          double dAngle = 36;
          double dEps = pBrep->GetTolerance();
          SmTArray<SmFace*> sFaces;
          pBrep->GetFaces(sFaces);
          SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
          //sTopoSweep.SetRepetitions(3);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 NULL, NULL, NULL,        // sweep higher targets
                                 &sFaces, NULL, NULL));   // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 NULL,NULL,NULL,&sFaces,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          // # Faces = 4, # Edges = 21, # Vertices = 20
          // Wire Edges = 1,  Lamina Edges = 16,  Manifold Edges = 4
          // # Regions = 1,  # Shells = 2

          pBrep->ValidateCounts(4, 21,20,1,16,4,1,2);
        }
        break;

      case 19:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate 1 Face higher\n"));
          // this involves edges high too
          smos_WriteBuffer(sBuff);


          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmPoint3d sCenter(-5, -5, 0);
          SmVector3d sAxis(-1, 1, 0);
          double dAngle = 90;
          double dEps = pBrep->GetTolerance();
          //SmTArray<SmEdge*> sEdges;
          //pBrep->GetEdges(sEdges);

          SmTArray<SmFace*> sBrepFaces;
          pBrep->GetFaces(sBrepFaces);

//            SmTArray<SmFace*> sFaces;
//           sFaces.Add(sBrepFaces[0]);

          SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
          sTopoSweep.SetDoMerge(TRUE);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 &sBrepFaces, NULL, NULL,   // sweep higher targets
                                 NULL, NULL, NULL));        // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 &sBrepFaces,NULL,NULL,NULL,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          // # Faces = 10, # Edges = 27, # Vertices = 21
          // Wire Edges = 2,  Lamina Edges = 4,  Manifold Edges = 20
          // # Regions = 2,  # Shells = 3

          pBrep->ValidateCounts(15, 34, 26,2,0,28,3,5);

        }
        break;

      case 20:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate 1 Face higher ax through corner\n"));
          // this involves edges high too
          smos_WriteBuffer(sBuff);

          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmPoint3d sCenter(0, 0, 0);
          SmVector3d sAxis(-1, 1, 0);
          double dAngle = 90;
          double dEps = pBrep->GetTolerance();
          //SmTArray<SmEdge*> sEdges;
          //pBrep->GetEdges(sEdges);

          SmTArray<SmFace*> sBrepFaces;
          pBrep->GetFaces(sBrepFaces);

//            SmTArray<SmFace*> sFaces;
//            sFaces.Add(sBrepFaces[0]);

          SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 &sBrepFaces, NULL, NULL,   // sweep higher targets
                                 NULL, NULL, NULL));        // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 &sBrepFaces,NULL,NULL,NULL,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          // # Faces = 10, # Edges = 26, # Vertices = 20
          // Wire Edges = 2,  Lamina Edges = 4,  Manifold Edges = 19
          // # Regions = 2,  # Shells = 3

          pBrep->ValidateCounts(15, 33,23,2,0,28,3,4);
        }
        break;

      case 21:
        {
          SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate 1 Face higher ax through edge\n"));
          // this involves edges high too
          smos_WriteBuffer(sBuff);

          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          SmPoint3d sCenter(0, 0, 0);
          SmVector3d sAxis(0, 1, 0);
          double dAngle = 90;
          double dEps = pBrep->GetTolerance();
          //SmTArray<SmEdge*> sEdges;
          //pBrep->GetEdges(sEdges);

          SmTArray<SmFace*> sBrepFaces;
          pBrep->GetFaces(sBrepFaces);

          SmTArray<SmFace*> sFaces;
          sFaces.Add(sBrepFaces[0]);

          SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
          SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
          SmTopologySweep sTopoSweep = * pTopoSweep;
          SmObjDelete sDelete(pTopoSweep);
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                 &sFaces, NULL, NULL,    // sweep higher targets
                                 NULL, NULL, NULL));     // sweep same targets  
          ucheck_mappings_sweep(sTopoSweep,
                                 &sFaces,NULL,NULL,NULL,NULL,NULL);

          pBrep->Dump();
          pBrep->ValidatePointers();
          rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
          // draw output
          if(smGet_DoGraphics() && pBrep)
            {
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE

          //  # Faces = 9, # Edges = 24, # Vertices = 19
          //  Wire Edges = 2,  Lamina Edges = 4,  Manifold Edges = 17
          //  # Regions = 2,  # Shells = 3

          pBrep->ValidateCounts(9, 24,19,2,4,17,2,3);
        }
        break;

      case 22:

        if (TRUE) 
          {
            SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate 1 Face higher FULL\n"));
            // this involves edges high too
            smos_WriteBuffer(sBuff);

            DELETE_ALL_PARTS(rPartBreps);
            my_create_test_brep(crContext,pBrep);
            SmPoint3d sCenter(-5, -5, 0);
            SmVector3d sAxis(-1, 1, 0);
            double dAngle = 360;
            double dEps = pBrep->GetTolerance();
            //SmTArray<SmEdge*> sEdges;
            //pBrep->GetEdges(sEdges);

            SmTArray<SmFace*> sBrepFaces;
            pBrep->GetFaces(sBrepFaces);

//            SmTArray<SmFace*> sFaces;
//            sFaces.Add(sBrepFaces[0]);

            SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
//OFF : Glue edges of different regions not allowed
            SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
            SmTopologySweep sTopoSweep = * pTopoSweep;
            SmObjDelete sDelete(pTopoSweep);
            sTopoSweep.SetDoMerge(TRUE);
//            SmTopologySweep sTopoSweep(sTSG, FALSE); // no stitching
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                   &sBrepFaces, NULL, NULL,  // sweep higher targets
                                   NULL, NULL, NULL));       // sweep same targets  
            ucheck_mappings_sweep(sTopoSweep,
                                   &sBrepFaces,NULL,NULL,NULL,NULL,NULL);

            pBrep->Dump();
            pBrep->ValidatePointers();
            rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
            // draw output
            if(smGet_DoGraphics() && pBrep)
              {
                smgfx_Erase() ;
                smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
                sm_GraphicsLoop() ;
              }
#endif // SM_GFX_CODE

            // # Faces = 10, # Edges = 28, # Vertices = 21
            // Wire Edges = 1,  Lamina Edges = 8,  Manifold Edges = 18
            // # Regions = 1,  # Shells = 2

            pBrep->ValidateCounts(13, 23,15,2,0,8,5,7);
          }

        break;

      case 23:

        if (TRUE) 
          {
            SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate 1 Face higher FULL ax through corner\n"));
            // this involves edges high too
            smos_WriteBuffer(sBuff);

            DELETE_ALL_PARTS(rPartBreps);
            my_create_test_brep(crContext,pBrep);
            SmPoint3d sCenter(0, 0, 0);
            SmVector3d sAxis(-1, 1, 0);
            double dAngle = 360;
            double dEps = pBrep->GetTolerance();
            //SmTArray<SmEdge*> sEdges;
            //pBrep->GetEdges(sEdges);

            SmTArray<SmFace*> sBrepFaces;
            pBrep->GetFaces(sBrepFaces);

            SmTArray<SmFace*> sFaces;
            sFaces.Add(sBrepFaces[0]);

            SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
//OFF : Glue edges of different regions not allowed
            SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
            SmTopologySweep sTopoSweep = * pTopoSweep;
            SmObjDelete sDelete(pTopoSweep);
            sTopoSweep.SetDoMerge(TRUE);
//            SmTopologySweep sTopoSweep(sTSG, FALSE); // no stitching
#ifdef SM_GFX_CODE
          // draw input
          if(smGet_DoGraphics() && pBrep)
            {
              pBrep->Dump() ;
              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
              sm_GraphicsLoop() ;
            }
#endif // SM_GFX_CODE
          SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),      // note: increments unlocked mark value
                                   &sFaces, NULL, NULL,    // sweep higher targets
                                   NULL, NULL, NULL));     // sweep same targets  
            ucheck_mappings_sweep(sTopoSweep,
                                   &sFaces,NULL,NULL,NULL,NULL,NULL);

            pBrep->Dump();
            pBrep->ValidatePointers();
            rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
            // draw output
            if(smGet_DoGraphics() && pBrep)
              {
                smgfx_Erase() ;
                smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
                sm_GraphicsLoop() ;
              }
#endif // SM_GFX_CODE

            // # Faces = 10, # Edges = 26, # Vertices = 20
            // Wire Edges = 1,  Lamina Edges = 6,  Manifold Edges = 18
            // # Regions = 1,  # Shells = 2
            pBrep->ValidateCounts(9, 19,15,2,4,6,3,4);
           }

         break;

      case 24:
        if (TRUE) 
          {
            SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate 1 Face higher FULL ax through edge\n"));
            // this involves edges high too
            smos_WriteBuffer(sBuff);

            DELETE_ALL_PARTS(rPartBreps);
            my_create_test_brep(crContext,pBrep);
            SmPoint3d sCenter(0, 0, 0);
            SmVector3d sAxis(0, 1, 0);
            double dAngle = 360;
            double dEps = pBrep->GetTolerance();
            //SmTArray<SmEdge*> sEdges;
            //pBrep->GetEdges(sEdges);

            SmTArray<SmFace*> sBrepFaces;
            pBrep->GetFaces(sBrepFaces);

            SmTArray<SmFace*> sFaces;
            sFaces.Add(sBrepFaces[0]);

            SmRotationalSweepGeometry sRSG(sCenter, sAxis, dAngle, dEps);
//OFF : Glue edges of different regions not allowed
            SmTopologySweep* pTopoSweep = new (crContext) SmTopologySweep(sRSG);
            SmTopologySweep sTopoSweep = *pTopoSweep;
            SmObjDelete sDelete(pTopoSweep);
            sTopoSweep.SetDoMerge(TRUE);
//            SmTopologySweep sTopoSweep(sTSG, FALSE); // no stitching
#ifdef SM_GFX_CODE
            // draw input
            if(smGet_DoGraphics() && pBrep)
              {
                pBrep->Dump() ;
                smgfx_Erase() ;
                smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
                sm_GraphicsLoop() ;
              }
#endif // SM_GFX_CODE
            SER(sTopoSweep.DoSweep(pBrep,pBrep->GetInfiniteRegion(),    // note: increments unlocked mark value
                                   &sFaces, NULL, NULL,  // sweep higher targets
                                   NULL, NULL, NULL));   // sweep same targets  
            ucheck_mappings_sweep(sTopoSweep,
                                   &sFaces,NULL,NULL,NULL,NULL,NULL);
  
            pBrep->Dump();
            pBrep->ValidatePointers();
            rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
            // draw output
            if(smGet_DoGraphics() && pBrep)
              {
                smgfx_Erase() ;
                smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop();
                sm_GraphicsLoop() ;
              }
#endif // SM_GFX_CODE

            // # Faces = 9, # Edges = 26, # Vertices = 19
            // Wire Edges = 1,  Lamina Edges = 10,  Manifold Edges = 14
            // # Regions = 1,  # Shells = 2

            pBrep->ValidateCounts(8,18,15,2,5,5,3,5);
          }

        break;

    case 25:
    {

      SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate whole Brep\n"));
      smos_WriteBuffer(sBuff);

      DELETE_ALL_PARTS(rPartBreps);
      pBrep = new(crContext) SmBrep();
      double dRadius = 4;
      SmPoint3d sCenter( 8,0,0 );
      SmVector3d sXAxis( 1,0,0 );
      SmVector3d sYAxis( 0,1,0 );
      SmAxis2Placement sPlacement( sCenter, sXAxis, sYAxis );
      SmPrimitiveCreation sPC( pBrep->GetInfiniteRegion() );
      sPC.CreateSphere( dRadius, 0, 360, sPlacement );

      SmPoint3d sBase ( -15, 0, 0 );
      SmVector3d sAxis(   0, 1, 0 );
      double dAngle = 45;
      double dEps = pBrep->GetTolerance();
      SmRotationalSweepGeometry sRSG( sBase, sAxis, dAngle, dEps );

      SmTopologySweep* pTopoSweep = new (crContext) SmTopologySweep(sRSG);
      SmTopologySweep sTopoSweep = *pTopoSweep;
      SmObjDelete sDelete(pTopoSweep);
      sTopoSweep.SetDoMerge ( TRUE );
      sTopoSweep.SetDoStitching( TRUE );
    
      // All NULL args: whole-body sweep.
      sTopoSweep.DoSweep(pBrep, pBrep->GetInfiniteRegion(),             // note: increments unlocked mark value
            NULL, NULL, NULL, NULL, NULL, NULL );


      ucheck_mappings_sweep(sTopoSweep, NULL,NULL,NULL,NULL,NULL,NULL);

      pBrep->Dump();
      pBrep->ValidatePointers();
      rPartBreps.Add(pBrep);

#ifdef SM_GFX_CODE
      // draw output
      if(smGet_DoGraphics() && pBrep) {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); pBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      pBrep->ValidateCounts( 3, 5, 4, 0, 0, 5, 2, 2 );

      break;
    }
    case 26:
    {

      SM_SPRINTF(sBuff,_T("%s"),_T(": Translate whole Brep: hollow box\n"));
      smos_WriteBuffer(sBuff);

      DELETE_ALL_PARTS(rPartBreps);
      pBrep = new(crContext) SmBrep();
      my_make_hollow_box( pBrep );
      SmVector3d sVec( 11, 22, 33 );
      SmTranslationalSweepGeometry sTSG(sVec);

      SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sTSG);
      SmTopologySweep sTopoSweep = * pTopoSweep;
      SmObjDelete sDelete(pTopoSweep);

      sTopoSweep.SetDoMerge ( TRUE );
      sTopoSweep.SetDoStitching( TRUE );
    
      sTopoSweep.DoSweep(pBrep, pBrep->GetInfiniteRegion(),             // note: increments unlocked mark value
            NULL, NULL, NULL, NULL, NULL, NULL );


      ucheck_mappings_sweep(sTopoSweep, NULL,NULL,NULL,NULL,NULL,NULL);

      pBrep->Dump();
      pBrep->ValidatePointers();
      rPartBreps.Add(pBrep);

#ifdef SM_GFX_CODE
      // draw output
      if(smGet_DoGraphics() && pBrep) {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); pBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      pBrep->ValidateCounts( 12, 24, 14, 0, 0, 24, 2, 2 );

      break;
    }
    case 27:  // dAngle = 45
    case 28:  // dAngle = 345
    case 29:  // dAngle = 360
    {

      if      ( nTest==27 ) { SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate whole Brep\n")); }
      else if ( nTest==28 ) { SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate whole Brep: self-intersect\n")); }
      else                  { SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate whole Brep FULL\n")); }
      smos_WriteBuffer(sBuff);

      DELETE_ALL_PARTS(rPartBreps);
      pBrep = new(crContext) SmBrep();
      double     dRadius = 4;
      SmVector3d sCenter( 8,0,0 );
      double     dTol = pBrep->GetTolerance();
      SmPrimitiveCreation::CreateSphereNoPole( &crContext, 
                                                dRadius, 
                                                sCenter, 
                                                dTol, 
                                                pBrep );
      if ( pBrep==NULL ) { return 6; }

#ifdef SM_GFX_CODE
      SM_DUMP_AND_ASSERT_VALID(pBrep) ;

      // draw input
      if(smGet_DoGraphics() && pBrep)
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); pBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      SmPoint3d  sBase( 0, 0, 0 );
      SmVector3d sAxis( 0, 0, 1 );
      double     dAngle =   (nTest == 27) ?  45 
                          : (nTest == 28) ? 345    // creates self-intersection.
                          :                 360;   // Full sweep: torus.  Should have no internal topo.
      double     dEps   = SM_ZONE_TOL_3D; // (Just a typical Brep tol.)
      SmRotationalSweepGeometry sRSG( sBase, sAxis, dAngle, dEps );

      SmTopologySweep * pTopoSweep = new (crContext) SmTopologySweep(sRSG);
      SmTopologySweep sTopoSweep = * pTopoSweep;
      SmObjDelete sDelete(pTopoSweep);
      sTopoSweep.SetDoMerge ( TRUE );
      sTopoSweep.SetDoStitching( TRUE );

      // do the sweep          // note: increments unlocked mark value
      sTopoSweep.DoSweep(pBrep,                      // in : target Brep to sweep, always required
                         pBrep->GetInfiniteRegion(), // in : The SmBrep of the region can be the
                                                     //      BrepToSweep or another SmBrep.
                                                     //      If no region given, the result will
                                                     //      be merged into BrepToSweepArg.
                         NULL,                       // in : sweep these faces into solids,   NULL to ingore, default:[NULL]
                         NULL,                       // in : sweep these edges into faces,    NULL to ignore, default:[NULL]
                         NULL,                       // in : sweep these vertices into edges, NULL to ignore, default:[NULL]
                         NULL,                       // in : copy these faces to their swept position,    NULL to ignore, default:[NULL]
                         NULL,                       // in : copy these edges to their swept position,    NULL to ignore, default:[NULL]
                         NULL );                     // in : copy these vertices to their swept position, NULL to ignore, default:[NULL]
                                                     // note: when all 6 input array ptrs are NULL, 
                                                     //       the whole Brep is swept. Otherwise only sweep 
                                                     //       the specified objects and their boundaries.

      ucheck_mappings_sweep(sTopoSweep, NULL,NULL,NULL,NULL,NULL,NULL);

      pBrep->Dump();
      pBrep->ValidatePointers();
      rPartBreps.Add(pBrep);

#ifdef SM_GFX_CODE
      // draw output
      if(smGet_DoGraphics() && pBrep)
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); pBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      if ( nTest == 27 || nTest == 28 ) { pBrep->ValidateCounts( 8, 14, 8, 0, 0, 14, 2, 2 ); } 
      else                              { pBrep->ValidateCounts( 2,  4, 2, 0, 0,  4, 2, 2 ); }

      break;
    }
    case 30:
    {
      SM_SPRINTF(sBuff,_T("%s"),_T(": Translate whole Brep: block with internal Edges\n"));
      smos_WriteBuffer(sBuff);

      DELETE_ALL_PARTS(rPartBreps);
      pBrep = new(crContext) SmBrep();
      my_make_block_with_lines( pBrep );
      SmVector3d sVec( -5, 10, -25 );
      SmTranslationalSweepGeometry sTSG( sVec );

      SmTopologySweep* pTopoSweep = new (crContext) SmTopologySweep(sTSG);
      SmTopologySweep sTopoSweep = *pTopoSweep;
      SmObjDelete sDelete(pTopoSweep);
      sTopoSweep.SetDoMerge ( TRUE );
      sTopoSweep.SetDoStitching( TRUE );
    
      sTopoSweep.DoSweep(pBrep, pBrep->GetInfiniteRegion(),             // note: increments unlocked mark value
            NULL, NULL, NULL, NULL, NULL, NULL );


      ucheck_mappings_sweep(sTopoSweep, NULL,NULL,NULL,NULL,NULL,NULL);

      pBrep->Dump();
      pBrep->ValidatePointers();
      rPartBreps.Add(pBrep);

#ifdef SM_GFX_CODE
      // draw output
      if(smGet_DoGraphics() && pBrep)
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); pBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      pBrep->ValidateCounts( 18, 37, 21, 0, 0, 37, 2, 2 );

      break;
    }
    case 31:
    case 32:
    {
      if ( nTest==31 )
        { SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate whole Brep: block with internal Edges\n")); }
      else
        { SM_SPRINTF(sBuff,_T("%s"),_T(": Rotate whole Brep: block with internal Edges FULL\n")); }
      smos_WriteBuffer(sBuff);

      DELETE_ALL_PARTS(rPartBreps);
      pBrep = new(crContext) SmBrep();
      my_make_block_with_lines( pBrep );

      SmPoint3d  sBase(  40, 20,   0 );
      SmVector3d sAxis( -20, 50,  10 );
      double dAngle = ( nTest == 31 ) ? 90 : 360;
      double dEps = pBrep->GetTolerance();
      SmRotationalSweepGeometry sRSG( sBase, sAxis, dAngle, dEps );

      SmTopologySweep* pTopoSweep = new (crContext) SmTopologySweep(sRSG);
      SmTopologySweep sTopoSweep = *pTopoSweep;
      SmObjDelete sDelete(pTopoSweep);
      sTopoSweep.SetDoMerge ( TRUE );
      sTopoSweep.SetDoStitching( TRUE );
    
      sTopoSweep.DoSweep(pBrep, pBrep->GetInfiniteRegion(),             // note: increments unlocked mark value
            NULL, NULL, NULL, NULL, NULL, NULL );


      ucheck_mappings_sweep(sTopoSweep, NULL,NULL,NULL,NULL,NULL,NULL);

      pBrep->Dump();
      pBrep->ValidatePointers();
      rPartBreps.Add(pBrep);
#ifdef SM_GFX_CODE
      // draw output
      if(smGet_DoGraphics() && pBrep) {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); pBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      if ( nTest == 31 ) {
          pBrep->ValidateCounts( 18, 37, 21, 0, 0, 37, 2, 2 );
      } else {
          pBrep->ValidateCounts(  9, 18,  9, 0, 0, 18, 2, 2 );
      }

      break;
    }

    case 33:
    case 34:
      {

        if(nTest == 33)
        {
          SM_SPRINTF( sBuff, _T( "%s" ), _T( ": Rotate sphere with pole on silhouette 45 degrees\n" ) );
        }
        else
        {
          SM_SPRINTF( sBuff, _T( "%s" ), _T( ": Rotate sphere with pole on silhouette FULL\n" ) );
        }
        smos_WriteBuffer( sBuff );

        DELETE_ALL_PARTS( rPartBreps );

        pBrep = new(crContext) SmBrep();
        double dRadius = 1;
        SmPoint3d sCenter( 0, 0, 5 );
        SmVector3d sXAxis( 1, 0, 0 );
        SmVector3d sYAxis( 0, 1, 0 );
        SmAxis2Placement sPlacement( sCenter, sXAxis, sYAxis );
        SmPrimitiveCreation sPC( pBrep->GetInfiniteRegion() );
        sPC.CreateSphere( dRadius, 0, 360, sPlacement );

#ifdef SM_GFX_CODE
        // draw output
        if(smGet_DoGraphics() && pBrep)
        {
          smgfx_Erase();
          smgfx_SetLook( 1, 2, 0, 0, 1 ); pBrep->Draw( TRUE ); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        SmPoint3d  sBase( 0, 0, 0 );
        // Neither x axis nor y axis work well
        SmVector3d sAxis( 1, 0, 0 );
        // SmVector3d sAxis( 0, 1, 0 );
        double dAngle = (nTest == 33) ? 45 : 360; // rotate Full for nTest 34.
        double dEps = pBrep->GetTolerance();
        SmRotationalSweepGeometry sRSG( sBase, sAxis, dAngle, dEps );

        SmTopologySweep* pTopoSweep = new (crContext) SmTopologySweep(sRSG);
        SmTopologySweep sTopoSweep = *pTopoSweep;
        SmObjDelete sDelete(pTopoSweep);
        sTopoSweep.SetDoMerge( TRUE );
        sTopoSweep.SetDoStitching( TRUE );

        sTopoSweep.DoSweep( pBrep, pBrep->GetInfiniteRegion(),             // note: increments unlocked mark value
              NULL, NULL, NULL, NULL, NULL, NULL );

        ucheck_mappings_sweep( sTopoSweep, NULL, NULL, NULL, NULL, NULL, NULL );

        pBrep->Dump();
        pBrep->ValidatePointers();
        rPartBreps.Add( pBrep );
#ifdef SM_GFX_CODE
        // draw output
        if(smGet_DoGraphics() && pBrep)
        {
          smgfx_Erase();
          smgfx_SetLook( 1, 2, 0, 0, 1 ); pBrep->Draw( TRUE ); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        if(nTest == 33)
        {
          pBrep->ValidateCounts( 4, 6, 4, 0, 0, 6, 2, 2 );
        }
        else
        {
          pBrep->ValidateCounts( 2, 4, 2, 0, 0, 4, 2, 2 );
        }

        break;
      }

      default:
        {
          DELETE_ALL_PARTS(rPartBreps);
          my_create_test_brep(crContext,pBrep);
          rPartBreps.Add(pBrep);
          pBrep->ValidatePointers();
          break;
        }
      } // end switch on nTest

  if (pBrep) pBrep->m_bEditingEnabled = FALSE;

  return SM_SUCCESS;

} // end my_sweep_demo

/**************************************************************
PURPOSE ---

USAGE NOTES ---
**************************************************************/
/***********************************************************************
PURPOSE --- Run one DoSweep and report the resulting topology counts

USAGE NOTES --- bNullRegion selects the documented default (NULL) instead of
                naming the Brep's own infinite region. Merging is disabled so
                the whole-Brep path reaches RemoveAllTopology().
***********************************************************************/
static SmStatus my_sweep_null_region_counts
 (SmBoolean bHollowWholeBrep,   // in : TRUE = sweep a whole hollow Brep, FALSE = sweep one box vertex
  SmBoolean bNullRegion,        // in : TRUE = pass NULL, FALSE = name the infinite region
  ULONG     rCounts[3],         // out: resulting face, edge and vertex counts
  ULONG     lRepetitions)       // in : number of sweeps, including after stitching
{
  SmContext   sContext ;
  SmBrep    * pBrep = new(sContext) SmBrep() ;
  SmObjDelete sCleanBrep( pBrep ) ;

  SmVector3d sSweepVec( 1.0, 1.0, 0.0 ) ;
  sSweepVec.Unitize() ;
  sSweepVec = sSweepVec * 7.0 ;

  SmTArray<SmVertex*> sSelection ;

  if( bHollowWholeBrep )
    {
      SER( my_make_hollow_box( pBrep ) ) ;
    }
  else
    {
      SmPrimitiveCreation sPrimitive( pBrep->GetInfiniteRegion() ) ;
      SER( sPrimitive.CreateBox( 16.0, 16.0, 16.0, SmAxis2Placement() ) ) ;

      SmTArray<SmVertex*> sVertices ;
      pBrep->GetVertices( sVertices ) ;
      if( sVertices.GetSize() == 0 )
        { return SM_ERR ; }
      sSelection.Add( sVertices[0] ) ;
    }

  SmRegion * pRegion = bNullRegion ? NULL : pBrep->GetInfiniteRegion() ;

  SmTranslationalSweepGeometry sSweepGeom( sSweepVec ) ;

  // Merging off: the whole-Brep path then empties the Brep with RemoveAllTopology(),
  // which deletes and replaces the infinite region while the sweep still holds it.
  SmTopologySweep sSweep( sSweepGeom, TRUE, FALSE, FALSE, FALSE ) ;
  sSweep.SetRepetitions( lRepetitions ) ;

  SER( sSweep.DoSweep( pBrep, pRegion,
                       NULL, NULL,
                       bHollowWholeBrep ? NULL : &sSelection,
                       NULL, NULL, NULL ) ) ;

  SmTArray<SmFace*>   sFaces ;
  SmTArray<SmEdge*>   sEdges ;
  SmTArray<SmVertex*> sVerts ;
  pBrep->GetFaces(    sFaces ) ;
  pBrep->GetEdges(    sEdges ) ;
  pBrep->GetVertices( sVerts ) ;
  rCounts[0] = sFaces.GetSize() ;
  rCounts[1] = sEdges.GetSize() ;
  rCounts[2] = sVerts.GetSize() ;

  return SM_SUCCESS ;

} // end my_sweep_null_region_counts

/***********************************************************************
PURPOSE --- SmTopologySweep::DoSweep must honour its declared null-region
            contract

USAGE NOTES --- SmTopologySweep.h documents pRegionToSweepInto as optional,
                meaning sweep into pBrepToSweep's own infinite region, but
                every use of it dereferences the pointer.

                Two cases: a selected sweep, and a whole hollow Brep with
                merging disabled. The second is the one that reaches
                RemoveAllTopology(), which replaces the infinite region
                mid-sweep, so it is where a stale cached region surfaces.
***********************************************************************/
static SmStatus my_test_dosweep_null_region()
{
  MYPRINTF( _T( "\nEntered: my_test_dosweep_null_region" ) ) ;

  const SmBoolean abHollowWholeBrep[2] = { FALSE, TRUE } ;

  // Repeat the whole-Brep case to expose freed-region reuse after stitching.
  // Also exercise repeated selected sweeps, which must reacquire the region.
  for( ULONG iPass = 0; iPass < 3; ++iPass )
    {
      for( ULONG iCase = 0; iCase < 2; ++iCase )
        {
          const ULONG lRepetitions = abHollowWholeBrep[iCase] ? 1 : iPass + 1 ;
          ULONG lExplicit[3] ;
          ULONG lNull[3] ;

          SER( my_sweep_null_region_counts( abHollowWholeBrep[iCase], FALSE, lExplicit, lRepetitions ) ) ;
          SER( my_sweep_null_region_counts( abHollowWholeBrep[iCase], TRUE,  lNull,     lRepetitions ) ) ;

          // The documented default has to match naming the region explicitly.
          for( ULONG ii = 0; ii < 3; ++ii )
            {
              if( lExplicit[ii] != lNull[ii] )
                { return SM_ERR ; }
            }

          // Guard against both passes agreeing on an empty result.
          if( lExplicit[0] == 0 || lExplicit[1] == 0 || lExplicit[2] == 0 )
            { return SM_ERR ; }

          // Sweeping one box vertex adds one edge and one vertex per repetition.
          // No absolute counts for the hollow whole-Brep case: they would only
          // re-state current output.
          if( !abHollowWholeBrep[iCase] )
            {
              if(   lExplicit[0] != 6
                 || lExplicit[1] != 12 + lRepetitions
                 || lExplicit[2] != 8 + lRepetitions )
                { return SM_ERR ; }
            }
        }
    }

  return SM_SUCCESS ;

} // end my_test_dosweep_null_region

SmStatus my_sweep_regression()
{

  MYPRINTF( _T( "\nEntered: my_sweep_regression 1" ) );

  TCHAR sBuff[SM_TBLOCK_SIZE];
  SmContext sContext;
  SmTArray<SmBrep*> sBreps;
  ULONG ii;
  ULONG lCnt = my_sweep_demo_count();

  // execute every sweep demo
  for(ii = 0; ii < lCnt; ii++)
  {
    SM_SPRINTF( sBuff, _T( "\nEntered: my_sweep_demo %ld" ), ii );
    MYPRINTF( sBuff );
    my_sweep_demo( sContext, sBreps, ii );
  }
  DELETE_ALL_PARTS( sBreps );

  SER( my_test_dosweep_null_region() );

  //   SER(my_test_primsol());

  return SM_SUCCESS;

} // end my_sweep_regression



