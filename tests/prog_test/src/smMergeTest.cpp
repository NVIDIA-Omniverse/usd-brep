// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smmerge_test.cpp  
* PURPOSE ---  Test SMLib merge and other SMLib functions.
* 

**********************************************************************/
/*___*/

#include "StdAfx.h"
#include <SmTypes.h>
#include <smMergeTest.h>
#include <SmMerge.h>
#include <SmOffsetSurface.h>
#include <SmBrepData.h>
#include <SmCrvOnSurf.h>

#ifdef SM_USE_EXCEPTIONS
#include <new>
#endif // SM_GFX_CODE // SM_USE_EXCEPTIONS

#include <SmCurveCache.h>


/***********************************************************************
PURPOSE --- Merge an array of Brep pairs into a set of Brep results

USAGE NOTES ---
   Place result of merging every Brep pair in rBreps into rResults.

   The Breps in rBreps get used up, but some of the rBreps Brep pointers
   get reused and placed in rResults. Don't depend on this unspecified
   behavior.
***********************************************************************/
SmStatus my_two_prim_test
  (SmTArray<SmBrep*> & rBreps,   // in : array of Breps to merge into 1
   ULONG lOperation,             // in : 0 - union, 1 - intersection, 2 - difference, 3 - merge
   ULONG & rlTotalEdges,         // out: total number of edges in rResults
   SmTArray<SmBrep*> & rResults) // out: one result Brep for every pair of Breps in rBreps

{
  // init output
  rResults.ReSet();

  // locals
  SmTArray<SmEdge*> sEdges;
  rlTotalEdges = 0;

  TCHAR sBuff[SM_TBLOCK_SIZE] ;


  // for every pair of Breps - unmatched Breps are ignored
  ULONG lBrepCount = rBreps.GetSize() ;
  for (ULONG i=1; i<lBrepCount; i+=2) 
    {
      // merge Brep[i] with Brep[i+1] and place result into rResults list
      SmBrep *pResult;

      SM_SPRINTF(sBuff, _T("\nCalling my_test_bbu 2 [iteration %ld] "),i) ;
      MYPRINTF(sBuff);
      if (my_test_bbu(rBreps[i-1],rBreps[i],lOperation,pResult) == SM_SUCCESS) 
        {
          rResults.Add(pResult);
          pResult->GetEdges(sEdges);
          pResult->Dump();
          rlTotalEdges += sEdges.GetSize();

          rBreps[i - 1] = NULL;
          rBreps[i]     = NULL;
        }
      else 
        {
          ERR_MSG(_T("**** BOOLEAN Failure *****\n"));
        }

#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE;
      // draw all results
      if (bDebugMe) 
        {
          smgfx_Erase() ;
          for (ULONG j=0; j<rResults.GetSize(); j++) 
            {
              SmBrep *pBrep = rResults[j];
              smgfx_SetLook(1,2) ; pBrep->Draw(); sm_GraphicsLoop() ;
            }
          sm_GraphicsLoop() ;
        }
#endif // SM_GFX_CODE
    } // end iter every pair of Breps



#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE;
  // draw all results
  if (bDebugMe) 
    {
      smgfx_Erase() ;
      for (ULONG j=0; j<rResults.GetSize(); j++) 
        {
          SmBrep *pBrep = rResults[j];
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
        }
      sm_GraphicsLoop() ;
    }
#endif // SM_GFX_CODE 

  return SM_SUCCESS;

} // end my_two_prim_test

/***********************************************************************
PURPOSE --- Given a list of Breps combine them into a single
 result using a boolean operation.

USAGE NOTES --- Breps are passed in on a list and combined into
a single result from back to front.  That is the last Brep is
combined with the pentultimate Brep to make a result.  That
result is combined with the 3rd to the last Brep and so on until
no more Breps are left to combine.

         Presult =  Op          Op := 0 - union,        
                   /  \               1 - intersection,
                Brep0  Op             2 - Subtract,    
                      /  \            3 - Merge        
                    Brep1 Op
                         /  \
                       . . . Op
                            /  \
                          BrepN Op

***********************************************************************/
SmStatus my_many_prim_test
  (SmTArray<SmBrep*> & rBreps,  // in : array of breps to process from
                                //      back to front, e.g.
                                //      rPResult = (((BrepN op BrepN-1) op BrepN-2) .. op Brep0)
   double dApproxTol,           // NotUsed: in : dApproxTol
   ULONG lOperation,            // in : 0 - union, 
                                //      1 - intersection, 
                                //      2 - Subtract, 
                                //      3 - Merge 
   ULONG & rlTotalEdges,        // out: total number of edges in output rpResult
   SmBrep *& rpResult)          // out: rpResult = (((BrepN op BrepN-1) op BrepN-2) .. op Brep0)
   
{
  SM_REF1(dApproxTol) ; 
  // init output
  rpResult = NULL ;

  // Build inputs to SmMerge::BooleanTrees() from rBreps array and lOperation value
  SmTArray<SmBrep*> sOrderedBreps;
  ULONG lCount = rBreps.GetSize();
  SmTArray<long> sBoolTree;
  sBoolTree.Add(0);
  if (rBreps.GetSize()< 1) return (SM_ERR);
  sOrderedBreps.Add(rBreps.GetLast());
  rBreps.RemoveLast();
  for (ULONG i=1; i<lCount; i++) 
    {
      sOrderedBreps.Add(rBreps.GetLast());
      rBreps.RemoveLast();
      sBoolTree.Add(i);
      sBoolTree.Add(0-(lOperation+1)); // Convert to different form of Boolean numbering
    }

  // make the BooleanTree call - signal error for a failed result
  SER(SmMerge::BooleanTrees(sOrderedBreps,sBoolTree));
  if (sOrderedBreps.GetSize() != 1) SER(SM_ERR); // Something went wrong

  // set output values
  rpResult = sOrderedBreps[0];
  SmTArray<SmEdge*> sEdges;
  rpResult->GetEdges(sEdges);
  rlTotalEdges = sEdges.GetSize();

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics() && rpResult)
    {
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(rpResult) rpResult->Draw() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_GFX_CODE


  // all done
  return SM_SUCCESS;

} // end my_many_prim_test


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_boolean_stress_test
(const SmContext & crContext,
    const TCHAR * pFileName,
    const TCHAR * pFileName2,
    ULONG lOperation,  // 0 - union, 1 - intersection, 2 - difference, 3 - merge, 5 - exclusive or
    SmBrep *& rpResult)
{
    SmBrep *pBrep1 = new (crContext) SmBrep();
    SmBrep *pBrep2 = new (crContext) SmBrep();
    SmObjDelete sClean1( pBrep1 ), sClean2( pBrep2 );
#ifdef SM_USE_EXCEPTIONS
    try {
#endif 
        SE(pBrep1->ReadFromFile(crContext, pFileName, SM_ASCII));
        SE(pBrep2->ReadFromFile(crContext, pFileName2, SM_ASCII));
        SE(pBrep1->ValidatePointers());
        SE(pBrep2->ValidatePointers());
        double dTol = 0.0; // pBrep1->GetTolerance();
        SmVector3dAttribute *pNewAttr = new (crContext) SmVector3dAttribute(SM_AI_COLOR, SmVector3d(0, 0, 1));
        pBrep2->AddAttribute(pNewAttr);

        SmMerge sMerge(crContext, pBrep1, pBrep2, dTol, 20.0*SM_PI / 180.0);
        if (pBrep1->IsManifoldSolid() && pBrep2->IsManifoldSolid())
        {
            switch (lOperation)
            {
            case 0:
                SER(sMerge.ManifoldBoolean(SM_BO_UNION, pBrep1)); pBrep2 = NULL;
                break;
            case 1:
                SER(sMerge.ManifoldBoolean(SM_BO_INTERSECTION, pBrep1)); pBrep2 = NULL;
                break;
            case 2:
                SER(sMerge.ManifoldBoolean(SM_BO_DIFFERENCE, pBrep1)); pBrep2 = NULL;
                break;
            case 3:
                SER(sMerge.ManifoldBoolean(SM_BO_MERGE, pBrep1)); pBrep2 = NULL;
                break;
            case 4:
                SER(sMerge.ManifoldBoolean(SM_BO_EXCLUSIVE_OR, pBrep1)); pBrep2 = NULL;
                break;
            default:
                SER(SM_ERR);
            }
        }
        else
        {
            switch (lOperation)
            {
            case 0:
                SER(sMerge.NonManifoldBoolean(SM_BO_UNION, pBrep1)); pBrep2 = NULL;
                break;
            case 1:
                SER(sMerge.NonManifoldBoolean(SM_BO_INTERSECTION, pBrep1)); pBrep2 = NULL;
                break;
            case 2:
                SER(sMerge.NonManifoldBoolean(SM_BO_DIFFERENCE, pBrep1)); pBrep2 = NULL;
                break;
            case 3:
                SER(sMerge.NonManifoldBoolean(SM_BO_MERGE, pBrep1)); pBrep2 = NULL;
                break;
            case 4:
                SER(sMerge.NonManifoldBoolean(SM_BO_EXCLUSIVE_OR, pBrep1)); pBrep2 = NULL;
                break;
            default:
                SER(SM_ERR);
            }
        }

        sClean1.Clear();
        sClean2.Clear();

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
            pBrep1->Draw();
            pBrep1->Dump();
            pBrep1->ValidatePointers();
            sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
        rpResult = pBrep1;

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            SmBoolean bDrawReg = FALSE;
            if (bDrawReg) {
                smgfx_Erase();
                if (rpResult) rpResult->Draw();
                sm_GraphicsLoop();
                smgfx_Erase();
                SmTArray<SmRegion*> sRegions;
                rpResult->GetRegions(sRegions);
                for (ULONG mm = 0; mm < sRegions.GetSize(); mm++) {
                    smgfx_Erase();
                    sRegions[mm]->Draw();
                    sm_GraphicsLoop();
                    if (sRegions[mm] == rpResult->GetInfiniteRegion()) {
                        SE(SM_ERR);
                    }
                    smgfx_Erase();
                    SmTArray<SmShell*> sRegShells;
                    sRegions[mm]->GetShells(sRegShells);
                    for (ULONG is = 0; is < sRegShells.GetSize(); is++) {
                        SmTArray<SmFaceuse*> sRegFaceuses;
                        sRegShells[is]->GetFaceuses(sRegFaceuses);
                        for (ULONG ir = 0; ir < sRegFaceuses.GetSize(); ir++) {
                            sRegFaceuses[ir]->GetFace()->DrawUV(4, 4);
                            sm_GraphicsLoop();
                        }
                    }
                }
            }
         }
#endif // SM_GFX_CODE
#ifdef SM_USE_EXCEPTIONS
    }
    catch (std::bad_alloc& e) {
        if(pBrep1) { delete pBrep1; pBrep1 = NULL ; }
        if(pBrep2) { delete pBrep2; pBrep2 = NULL ; }
        TCHAR sBuffer[SM_TBLOCK_SIZE];
        SM_SPRINTF(sBuffer, _T("std::bad_alloc: %s\n"), smos_ToTChar(e.what()).c_str());
        MYPRINTF(sBuffer);
    }
    
#endif // SM_USE_EXCEPTIONS
    SM_ASSERT(pBrep2 == NULL) ;
    return SM_SUCCESS;

} // end my_boolean_stress_test

/***********************************************************************
PURPOSE --- Test stitch sequence

USAGE NOTES ---  used by prog_test

 lCount = 0 : Read File "../../TestFiles/pt_TestFiles/Solids/fillet_ts.smb" into rBreps[0]
        = 1 : Call rBreps[0]->StitchFaces(.001)
        = 2 : Call rBreps[0]->StitchFaces(.005)
        = 3 : Read File "../../TestFiles/pt_TestFiles/Solids/fillet_cut.smb" into rBreps[1]
        = 4 : Call my_two_prim_test(sBreps)
***********************************************************************/
PT_EXPORT SmStatus my_stitching_demo()
{
  SmContext sContext;
  SmTArray<SmBrep*> rBreps;

  // read part into rBreps
  {
    MYPRINTF( _T( "\nEntered my_stitching_demo 1" ) );
    ULONG i;
    for(i = 0; i < rBreps.GetSize(); i++)
    {
      SM_ASSERT( rBreps[i] != NULL );
      delete rBreps[i];
      rBreps[i] = NULL;
    }
    rBreps.ReSet();

    SmBrep* pBrep = new(sContext) SmBrep();
    MYPRINTF( _T( "\nReading test file ../../TestFiles/pt_TestFiles/Solids/fillet_ts.smb" ) );
    pBrep->ReadFromFile( sContext, _T( "../../TestFiles/pt_TestFiles/Solids/fillet_ts.smb" ), SM_ASCII );
    rBreps.Add( pBrep );

#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      SmTArray<SmFace*> sFaces;
      if(rBreps[0]) rBreps[0]->GetFaces( sFaces );
      for(i = 0; i < sFaces.GetSize(); i++) sFaces[i]->Dump();
      MYPRINTF( _T( "\n  Running AssertValid on test file ../../TestFiles/pt_TestFiles/Solids/fillet_ts.smb" ) );
      MYPRINTF( _T( "\nBegin AssertValid on part known to have coincident geometry problems" ) );
      SM_DUMP_AND_ASSERT_VALID( rBreps[0] );
      MYPRINTF( _T( "\nEnd AssertValid on part known to have coincident geometry problems\n" ) );

      smgfx_Erase();
      smgfx_SetLook( 1, 2 );
      rBreps[0]->Draw();
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE
  }

  // call StitchFaces on rBreps[0], tol = .001
  {
    MYPRINTF( _T( "\nEntered my_stitching_demo 2" ) );
    if(rBreps.GetSize() < 1) SER( SM_ERR );
    SmBrep *pBrep = rBreps[0];
    ULONG lNumStitched, lNumLamina;
    double dMaxVGap, dMaxEGap;
    pBrep->Dump();
    pBrep->m_bEditingEnabled = TRUE;
    SE( pBrep->StitchFaces( 0.001, lNumStitched, lNumLamina, dMaxVGap, dMaxEGap ) );
    pBrep->m_bEditingEnabled = FALSE;
    TCHAR sBuff[SM_TBLOCK_SIZE];
    SM_SPRINTF( sBuff, _T( "Stitch Faces - # Stitched Edges = %ld, # Lamina = %ld,  Max V Gap = %le, Max E Gap = %le\n" ),
        lNumStitched, lNumLamina, dMaxVGap, dMaxEGap );
    smos_WriteBuffer( sBuff );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      SM_DUMP_AND_ASSERT_VALID( rBreps[0] );

      smgfx_Erase();
      smgfx_SetLook( 1, 2 ); rBreps[0]->Draw();
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE
  }

  // call StitchFaces on rBreps[0], tol = .005 
  {
    MYPRINTF( _T( "\nEntered my_stitching_demo 3" ) );
    if(rBreps.GetSize() < 1) SER( SM_ERR );
    SmBrep *pBrep = rBreps[0];
    ULONG lNumStitched, lNumLamina;
    double dMaxVGap, dMaxEGap;
    pBrep->Dump();
    pBrep->m_bEditingEnabled = TRUE;
    SE( pBrep->StitchFaces( 0.005, lNumStitched, lNumLamina, dMaxVGap, dMaxEGap ) );
    pBrep->m_bEditingEnabled = FALSE;
    TCHAR sBuff[SM_TBLOCK_SIZE];
    SM_SPRINTF( sBuff, _T( "Stitch Faces - # Stitched Edges = %ld, # Lamina = %ld,  Max V Gap = %le, Max E Gap = %le\n" ),
        lNumStitched, lNumLamina, dMaxVGap, dMaxEGap );
    smos_WriteBuffer( sBuff );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      SM_ASSERT_VALID( rBreps[0] );

      smgfx_Erase();
      smgfx_SetLook( 1, 2 ); rBreps[0]->Draw();
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE
  }

  // read 2nd part (Cylinder) into rBreps
  {
    MYPRINTF( _T( "\nEntered my_stitching_demo 4" ) );
    if(rBreps.GetSize() < 1)
      SER( SM_ERR );
    SmBrep* pBrep = new(sContext) SmBrep();
    MYPRINTF( _T( "\nReading test file ../../TestFiles/pt_TestFiles/Solids/fillet_cut.smb" ) );
    pBrep->ReadFromFile( sContext, _T( "../../TestFiles/pt_TestFiles/Solids/fillet_cut.smb" ), SM_ASCII );
    rBreps.Add( pBrep );

#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 0, 1 ); rBreps[0]->Draw();
      smgfx_SetLook( 1, 2, 0, 1, 0 ); rBreps[1]->Draw();
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE
  }

  // subtract cylinder from stitched part
  {
    MYPRINTF( _T( "\nEntered my_stitching_demo 5" ) );
    if(rBreps.GetSize() < 1) SER( SM_ERR );
    if(rBreps.GetSize() == 2)
    {
      SmTArray<SmBrep*> sBreps;
      sBreps.Append( rBreps );
      ULONG lTotEdges = 0;
      my_two_prim_test( sBreps, 2, lTotEdges, rBreps );
    }
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      SM_ASSERT_VALID( rBreps[0] );

      smgfx_Erase();
      smgfx_SetLook( 1, 2 ); if(rBreps[0]) rBreps[0]->Draw();
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE

  }

  if(rBreps[0] != NULL) { rBreps[0]->Dump() ; rBreps[0] -> ~SmBrep(); delete rBreps[0] ; rBreps[0] = NULL ; } // JLMCC memory leak

  // all done
  return SM_SUCCESS;

} // end my_stitching_demo

/***********************************************************************
PURPOSE --- Test SmBrep::CreateOneBrepPerShell

USAGE NOTES ---  used by prog_test
***********************************************************************/
PT_EXPORT SmStatus my_test_BrepPerShell()
{
  SmStatus bRtn = SM_SUCCESS;

    {
        SmContext sContext;
        SmRegion* pRegion;
        SmShell *pNewShell;
        SmFace* pFace;
        SmLoop* pLoop;
        SmEdge *pNewEdge;
        SmVertex *pNewVertex;
        SmVector3d sEdgeDir;
        SmVertex* pVertex;
        SmTArray<SmRegion*> sRegions;
        SmTArray<SmFace*> sFaces, sNewFaces;
        SmTArray<SmEdge*> sNewEdges;
        SmTArray<SmVertex*> sVertices;
        SmBrep* pBrep0 = new (sContext) SmBrep();
        SmBrep* pBrep1 = new (sContext) SmBrep();
        SmBrep* pBrep2 = new (sContext) SmBrep();
        SmPrimitiveCreation sPC0(pBrep0->GetInfiniteRegion());
        SmPrimitiveCreation sPC1(pBrep0->GetInfiniteRegion());
        SmPrimitiveCreation sPC2(pBrep0->GetInfiniteRegion());
        SmPrimitiveCreation sPC3(pBrep1->GetInfiniteRegion());
        SmPrimitiveCreation sPC4(pBrep2->GetInfiniteRegion());
        SmPrimitiveCreation sPC5(pBrep2->GetInfiniteRegion());
        SmPrimitiveCreation sPC6(pBrep2->GetInfiniteRegion());
        SmPrimitiveCreation sPC7(pBrep2->GetInfiniteRegion());

        // Make a cube. Will have internal shells
        sPC0.CreateBox(10, 10, 10, SmAxis2Placement());

        // Add an internal wire edge to the first cube
        {
            pBrep0->GetRegions(sRegions);
            pRegion = sRegions.GetLast();
            pBrep0->GetVertices(sVertices);
            pVertex = sVertices[0];
            SmPoint3d sBoxMid(5, 5, 5);
            sEdgeDir = .2 * (sBoxMid - pVertex->GetPoint());
            SmPoint3d sEndVertPt = pVertex->GetPoint() + sEdgeDir;
            SmLine* pLine = new (sContext) SmLine(pVertex->GetPoint(), sEndVertPt, 3, &sContext);
            pBrep0->MakeWireEdgeVertex(
                pRegion, pVertex, pLine, SmExtent1d(0, 1), SM_OT_SAME, sEndVertPt, pNewEdge, pNewVertex);
        }

        // Add a second cube (solid body) and a circle (sheet model)
        sPC1.CreateBox(10, 10, 10, SmAxis2Placement(20, 0, 0, 1, 0, 0, 0, 1, 0));

        // Add geometry to the second cube
        {
            pBrep0->m_bEditingEnabled = TRUE;
            // Add a Vertex loop to a face on the second cube
            pBrep0->GetFaces(sFaces);
            pFace = sFaces[7];
            SmTArray<SmPoint2d> sUVPoints;
            SmTArray<SmPoint3d> s3dPoints;
            pFace->GetPointsInFace(1, sUVPoints, s3dPoints);
            pBrep0->MakeVertexLoop(pFace, s3dPoints[0], pLoop, pNewVertex);

            // Add an hole to a face
            pFace = sFaces[8];
            pFace->GetVertices(sVertices);
            SmPoint3d sFaceCenter(0,0,0);
            SmVector3d sFaceNormal;
            SmExtent1d sIvl(0, 360);
            ULONG lNumVerts = sVertices.GetSize();
            for (ULONG ii = 0; ii < lNumVerts; ++ii)
            { sFaceCenter += sVertices[ii]->GetPoint(); }
            sFaceCenter /= lNumVerts;
            SmVector3d sXAxis, sYAxis;
            sXAxis = sVertices[0]->GetPoint() - sFaceCenter;
            sYAxis = sVertices[1]->GetPoint() - sFaceCenter;
            SmCircle* pCircle = new (sContext) SmCircle(sFaceCenter, sXAxis, sYAxis, sIvl, 3);
            SmTArray<SmCurve*> sCurves;
            sCurves.Add(pCircle);
            pBrep0->MergeCurvesOnSurface(*pFace->GetSurface(), pFace->GetTolerance(), sCurves, NULL, sNewFaces, sNewEdges);

            SmPoint3d sBoxCenter(25, 5, 5);
            pBrep0->GetRegions(sRegions);
            pRegion = sRegions.GetLast();
            pBrep0->MakeShellVertex(pRegion ,sBoxCenter, pNewShell, pNewVertex);
        }

        sPC2.CreateCircle(5, SmAxis2Placement(40, 0, 0, 1, 0, 0, 0, 1, 0));

        // Create small boxes to be booleaned into the first cube
        sPC3.CreateBox(6, 6, 6, SmAxis2Placement(2, 2, 2, 1, 0, 0, 0, 1, 0));
        sPC4.CreateBox(1, 1, 1, SmAxis2Placement(3, 3, 3, 1, 0, 0, 0, 1, 0));
        sPC5.CreateBox(1, 1, 1, SmAxis2Placement(5, 5, 5, 1, 0, 0, 0, 1, 0));
        sPC6.CreateBox(1, 1, 1, SmAxis2Placement(3, 3, 5, 1, 0, 0, 0, 1, 0));
        sPC7.CreateBox(1, 1, 1, SmAxis2Placement(5, 5, 3, 1, 0, 0, 0, 1, 0));

        // Create a void in the first cube
        SmMerge sMerge0(sContext, pBrep0, pBrep1);
        sMerge0.NonManifoldBoolean(SM_BO_DIFFERENCE, pBrep0);

        // Add solid regions inside in inner void of the first cube
        SmMerge sMerge1(sContext, pBrep0, pBrep2);
        sMerge1.NonManifoldBoolean(SM_BO_UNION, pBrep0);

        // Shell vertex hanging out in space
        pBrep0->MakeShellVertex(pBrep0->GetInfiniteRegion(), SmVector3d(0, 15, 0), pNewShell, pNewVertex);

        // Wire edge w/ CurveOnSurf
        SmPlane *pPlane = NULL;
        SmCrvOnSurf *pCrvOnSurf;
        SmOrTy sSame = SM_OT_SAME;
        SmExtent2d sDomain(0, 0, 5, 5);
        SmPlane::CreateCanonical(sContext, SmAxis2Placement(20, 15, 0, 1, 0, 0, 0, 1, 0), pPlane);
        pPlane->TrimWithDomain(sDomain);
        pPlane->CreateBoundaryCrvOnSurf(sContext, pPlane->GetNaturalUVDomain(), SM_SP_UMAX, pCrvOnSurf, sSame);
        pBrep0->CreateWireEdgeFromCurve(pCrvOnSurf, pCrvOnSurf->GetNaturalInterval(), pNewEdge);

#ifdef SM_DEBUG_CODE
        if(smGet_DoGraphics())
        {
            smgfx_Erase();
            pBrep0->Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif

        SM_ASSERT_VALID(pBrep0);
        SmTArray<SmBrep*> sNewBreps;
        SmObjsDelete<SmBrep*> sClean(&sNewBreps);
        SER(SmBrep::CreateOneBrepPerBody(pBrep0, sNewBreps, TRUE));
        bRtn = (sNewBreps.GetSize() == 5) ? SM_SUCCESS : SM_ERR;

        for (ULONG ii = 0; ii < sNewBreps.GetSize(); ++ii)
        { SM_ASSERT_VALID_AND_DUMP(sNewBreps[ii]); }

#ifdef SM_DEBUG_CODE
        if(smGet_DoGraphics())
        {
            smgfx_Erase();
            smgfx_SetLook(3, 5, 0, 1, .1, .1);
            sNewBreps[0]->Draw(TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3, 5, 0, .1, 1, .1);
            sNewBreps[1]->Draw(TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3, 5, 0, .1, .1, 1);
            sNewBreps[2]->Draw(TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3, 5, 0, .8, .1, .8);
            sNewBreps[3]->Draw(TRUE); sm_GraphicsLoop();
            sNewBreps[4]->Draw(TRUE); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif
    }

  return bRtn;
}

/***********************************************************************
PURPOSE --- A Boolean demo to build a computer mouse shape

USAGE NOTES --- Used by prog_test via my_test_merge
***********************************************************************/
PT_EXPORT SmStatus my_mouse_demo()
{
  SmContext sContext;

  SmTArray<SmBrep*> sBreps;

  MYPRINTF( _T( "\nEntered my_mouse_demo 1" ) );
  SmBrep* pBrep1 = new(sContext) SmBrep();
  MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/mouse_1.smb"));
  pBrep1->ReadFromFile( sContext, _T( "../../TestFiles/pt_TestFiles/Solids/mouse_1.smb" ), SM_ASCII );
  sBreps.Add( pBrep1 );

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
  {
    smgfx_Erase();
    smgfx_SetLook( 1, 2, 0, 0, 1 ); pBrep1->Draw(TRUE);
    sm_GraphicsLoop();
  }
#endif // SM_GFX_CODE

  // read in 2nd mouse shape
  MYPRINTF( _T( "\nEntered my_mouse_demo 2" ) );
  SmBrep* pBrep2 = new(sContext) SmBrep();
  MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/mouse_2.smb"));
  pBrep2->ReadFromFile( sContext, _T( "../../TestFiles/pt_TestFiles/Solids/mouse_2.smb" ), SM_ASCII );
  sBreps.Add( pBrep2 );

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
  {
    smgfx_Erase();
    smgfx_SetLook( 1, 2, 0, 0, 1 ); pBrep1->Draw(TRUE);
    smgfx_SetLook( 1, 2, 0, 1, 0 ); pBrep2->Draw(TRUE);
    sm_GraphicsLoop();
  }
#endif // SM_GFX_CODE

  // read in 3rd mouse shape and merge
  MYPRINTF( _T( "\nEntered my_mouse_demo 3" ) );
  ULONG lTotEdges;
  SmBrep *pResult;
  my_many_prim_test(sBreps, 0.001, 3, lTotEdges, pResult);
  sBreps.ReSet();
  sBreps.Add(pResult);
  pResult->Dump();

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0.1, 0.8, 0.5) ; sBreps[0]->Draw(TRUE) ;
          sm_GraphicsLoop() ;
        }
#endif // SM_GFX_CODE

  MYPRINTF( _T( "\nEntered my_mouse_demo 4" ) );
  SmBrep* pBrep3 = new(sContext) SmBrep();
  MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/mouse_3.smb"));
  pBrep3->ReadFromFile( sContext, _T( "../../TestFiles/pt_TestFiles/Solids/mouse_3.smb" ), SM_ASCII );
  sBreps.Add( pBrep3 );

  lTotEdges = 0; 
  pResult = NULL;
  my_many_prim_test(sBreps, 0.001, 3, lTotEdges, pResult);
  sBreps.ReSet();
  sBreps.Add(pResult);
  pResult->Dump();

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
  {
    smgfx_Erase();
    smgfx_SetLook( 1, 2, 0, 0, 1 ); sBreps[0]->Draw(TRUE);
    sm_GraphicsLoop();
  }
#endif // SM_GFX_CODE
  
  // read in 4th mouse shape and merge
  MYPRINTF( _T( "\nEntered my_mouse_demo 5" ) );
  SmBrep* pBrep4 = new(sContext) SmBrep();
  MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/mouse_4.smb"));
  pBrep4->ReadFromFile( sContext, _T( "../../TestFiles/pt_TestFiles/Solids/mouse_4.smb" ), SM_ASCII );
  sBreps.Add( pBrep4 );

  lTotEdges = 0;
  pResult = NULL;
  my_many_prim_test(sBreps, 0.001, 3, lTotEdges, pResult); 
  sBreps.ReSet();
  sBreps.Add(pResult);
  pResult->Dump();

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
  {
    smgfx_Erase();
    smgfx_SetLook( 1, 2, 0, 0, 1 ); sBreps[0]->Draw();
    sm_GraphicsLoop();
  }
#endif // SM_GFX_CODE


  // read in 5th mouse shape and merge
  MYPRINTF( _T( "\nEntered my_mouse_demo 6" ) );
  SmBrep* pBrep5 = new(sContext) SmBrep();
  MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/mouse_5.smb"));
  pBrep5->ReadFromFile( sContext, _T( "../../TestFiles/pt_TestFiles/Solids/mouse_5.smb" ), SM_ASCII );
  sBreps.Add( pBrep5 );

  lTotEdges = 0;
  pResult = NULL;
#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
  {
    smgfx_Erase();
    smgfx_SetLook( 1, 2, 0, 0, 1 ); sBreps[0]->Draw();
    smgfx_SetLook( 1, 2, 0, 1, 0 ); sBreps[1]->Draw();
    sm_GraphicsLoop();
  }
#endif // SM_GFX_CODE
  my_many_prim_test(sBreps, 0.001, 3, lTotEdges, pResult);
  sBreps.ReSet();
  sBreps.Add(pResult);
  pResult->Dump();

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
  {
    smgfx_Erase();
    smgfx_SetLook( 1, 2 ); sBreps[0]->Draw(TRUE);
    sm_GraphicsLoop();
  }
#endif // SM_GFX_CODE

  // MakeManifold and Orient trimmed surfaces
  MYPRINTF( _T( "\nEntered my_mouse_demo 7" ) );
  SmBrep *pBrep = sBreps[0];
  SER( pBrep->MakeManifold() );

  // Orient faces outward 
  SmBoolean bNormalsOutward = TRUE;
  SmBoolean rbMaybeNotClosedSolid = FALSE;
  pBrep->OrientTrimmedSurfaces( bNormalsOutward, rbMaybeNotClosedSolid, FALSE );
  pBrep->Dump();

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
  {
    smgfx_Erase();
    smgfx_SetLook( 1, 2, 0.1, 0.8, 0.5 ); sBreps[0]->Draw(TRUE);
    sm_GraphicsLoop();
  }
#endif // SM_GFX_CODE

  if(sBreps[0]) { sBreps[0]->Dump() ; delete sBreps[0] ; sBreps[0] = NULL ; }
  sBreps.ReSet();

  return SM_SUCCESS;

} // end my_mouse_demo

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
static SmBrep * my_make_vertex_brep
  (const SmContext & crContext,  // in : 
   const SmPoint3d & crPoint)    // NotUsed: in : crPoint
{
  SM_REF1(crPoint) ; 
    SmBrep *pRet = new (crContext) SmBrep();
    SmVertex *pNewV;
    SmShell *pNewS;
    SE(pRet->MakeShellVertex(pRet->GetInfiniteRegion(),SmPoint3d(0.5,0.5,0.5),pNewS,pNewV));
    return pRet;

} // end my_make_vertex_brep

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
static SmBrep * my_make_wire_brep
  (const SmContext & crContext, 
   const SmPoint3d & crStartPt,
   const SmPoint3d & crEndPt)
{
    SmBrep *pRet = new (crContext) SmBrep();
    SmBSplineCurve *pLine = NULL ;
    SE(SmBSplineCurve::CreateLineSegment(crContext,3,crStartPt,crEndPt,pLine));
    SmEdge *pE;
    SE(pRet->CreateWireEdgeFromCurve(pLine,pLine->GetNaturalInterval(),pE));
    return pRet;

} // end my_make_wire_brep

/***********************************************************************
PURPOSE --- Add a wire edge with SmLine geometry to target Brep

USAGE NOTES ---
***********************************************************************/
static void my_add_wire_to_brep
  (const SmContext & crContext,  // in : context for new object construction
   SmBrep          * pBrep,      // in : target brep
   const SmPoint3d & crStartPt,  // in : Wire start point
   const SmPoint3d & crEndPt)    // in : Wire end point
{
  // build the line geometry
  SmBSplineCurve *pLine = NULL ;
  SE(SmBSplineCurve::CreateLineSegment(crContext,3,crStartPt,crEndPt,pLine));

  // build the edge
  SmEdge *pE;
  SE(pBrep->CreateWireEdgeFromCurve(pLine,pLine->GetNaturalInterval(),pE));

} // end my_add_wire_to_brep

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_stress()
{
  SmTArray<SmBrep*> sBreps;
  SmContext sContext;

  if (TRUE)
  { // Union of Box and Shell Vertex
      SmBrep* pBrep = new(sContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
      pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"), SM_ASCII);

      SmBrep* pOther = new(sContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
      pOther->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"), SM_ASCII);

      pBrep->m_bEditingEnabled = TRUE;
      pOther->m_bEditingEnabled = TRUE;

      SmVector3d sTranslate(0, 0, 0);

      SmAxis2Placement sXform;
      sXform.SetCanonical(sTranslate, SmVector3d(1, 0, 0), SmVector3d(0, 1, 0));
      SmAxis2Placement sXform2;
      sXform2.RotateAboutAxis(0.1*SM_PI / 180.0, SmVector3d(0, 0, 1));
      sXform2.Translate(sTranslate);

      pBrep->Transform(sXform);
      pOther->Transform(sXform2);

      SmBrep *pResult;
      SER(my_test_bbu(pOther, pBrep, 2, pResult));
      pResult->Dump();
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics()) {
          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1); if (pResult) pResult->Draw();
          sm_GraphicsLoop();
      }
#endif // SM_GFX_CODE
      SM_ASSERT(pResult != NULL) ; delete pResult; pResult = NULL ;
    }

  if (TRUE) 
    { // Merge of extrudeHoles1 and extrudeHoles2
      MYPRINTF(_T("\nNote, the following test results in a zero-area Loop."));
      SmBrep *pResult;
      SER(my_boolean_stress_test(sContext,
          _T("../../TestFiles/pt_TestFiles/Boolean/extrudeHoles1.smb"),
          _T("../../TestFiles/pt_TestFiles/Boolean/extrudeHoles2.smb"), 3, pResult ));

      pResult->ValidateCounts(8,19,20,0,6,12,2,8);

      // More validation: there are six vertex shells, all should be in
      // the infinite region; some were getting mis-classified.
      // [ TestClosureSplitRegion, 1/12/11 ]
      SmTArray< SmRegion* > sRegions;
      SmTArray< SmShell*  > sShells;
      pResult->GetRegions( sRegions );
      SM_ASSERT( sRegions.GetSize() == 2 );
      sRegions[0]->GetShells( sShells );
      SM_ASSERT( sShells.GetSize() == 7 );
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics()) {
          if (pResult) pResult->Draw();
      }
#endif // SM_GFX_CODE
      SM_ASSERT(pResult != NULL) ; delete pResult; pResult = NULL ;
    }

  if (TRUE) 
    { // Merge of BoltMBox1 and BoltMBox1
      SmBrep *pResult;
      SER(my_boolean_stress_test(sContext,
          _T("../../TestFiles/pt_TestFiles/Boolean/BoltMBox1.smb"),
          _T("../../TestFiles/pt_TestFiles/Boolean/BoltMBox2.smb"),3, pResult));

      pResult->ValidateCounts(83,162,85,0,0,114,6,6);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics()) {
          if (pResult) pResult->Draw();
      }
#endif // SM_GFX_CODE
      SM_ASSERT(pResult != NULL) ; delete pResult; pResult = NULL ;
    }

  if (TRUE) 
    { // Union of BoxUCylTan1 and BoxUCylTan2
      SmBrep *pResult;
      SER(my_boolean_stress_test(sContext,
          _T("../../TestFiles/pt_TestFiles/Boolean/BoxUCylTan1.txt"),
          _T("../../TestFiles/pt_TestFiles/Boolean/BoxUCylTan2.txt"),0, pResult));

      pResult->ValidateCounts(9,21,14,0,0,21,2,2);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics()) {
          if (pResult) pResult->Draw();
      }
#endif // SM_GFX_CODE
      SM_ASSERT(pResult != NULL) ; delete pResult; pResult = NULL ;
    }

  return SM_SUCCESS;

} // end my_test_stress


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_non_manifold()
{
    SmTArray<SmBrep*> sBreps;
    SmTArray<SmBrep*> sResults;
    SmContext sContext; 

    if (TRUE) { // Union of Box and Wires
        SmBrep *pOther = my_make_wire_brep(sContext,SmPoint3d(5.0,5.0,-7.0),
            SmPoint3d(5.0,5.0,18.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(8.0,8.0,8.0),SmPoint3d(12.0,12.0,12.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(0.0,-2.0,0.0),SmPoint3d(0.0,12.0,0.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(6.0,-2.0,0.0),SmPoint3d(6.0,12.0,0.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(2.0,2.0,5.0),SmPoint3d(2.0,2.0,12.0));

        SmObjDelete sClean(pOther);

        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
        pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(0,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);
        sClean.Clear();
        MYPRINTF(_T("\nCalling my_test_bbu 3"));
        SER(my_test_bbu(pOther,pBrep,0,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        if(pResult) { delete pResult; pResult = NULL ; }
    }


    if (TRUE) { // Intersection of box and wires
        SmBrep *pOther = my_make_wire_brep(sContext,SmPoint3d(5.0,5.0,-7.0),
            SmPoint3d(5.0,5.0,18.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(8.0,8.0,8.0),SmPoint3d(12.0,12.0,12.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(0.0,-2.0,0.0),SmPoint3d(0.0,12.0,0.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(6.0,-2.0,0.0),SmPoint3d(6.0,12.0,0.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(2.0,2.0,5.0),SmPoint3d(2.0,2.0,12.0));

        SmObjDelete sClean(pOther);

        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
        pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(20,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);
        sClean.Clear();
        MYPRINTF(_T("\nCalling my_test_bbu 4"));
        SER(my_test_bbu(pBrep,pOther,1,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }

    if (TRUE) { // Difference of box and wires
        SmBrep *pOther = my_make_wire_brep(sContext,SmPoint3d(5.0,5.0,-7.0),
            SmPoint3d(5.0,5.0,18.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(8.0,8.0,8.0),SmPoint3d(12.0,12.0,12.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(0.0,-2.0,0.0),SmPoint3d(0.0,12.0,0.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(6.0,-2.0,0.0),SmPoint3d(6.0,12.0,0.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(2.0,2.0,5.0),SmPoint3d(2.0,2.0,12.0));

        SmObjDelete sClean(pOther);

        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
        pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(40,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);
        sClean.Clear();
        MYPRINTF(_T("\nCalling my_test_bbu 6"));
        SER(my_test_bbu(pBrep,pOther,2,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }

    if (TRUE) { // Difference of Wires and Box
        SmBrep *pOther = my_make_wire_brep(sContext,SmPoint3d(5.0,5.0,-7.0),
            SmPoint3d(5.0,5.0,18.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(8.0,8.0,8.0),SmPoint3d(12.0,12.0,12.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(0.0,-2.0,0.0),SmPoint3d(0.0,12.0,0.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(6.0,-2.0,0.0),SmPoint3d(6.0,12.0,0.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(2.0,2.0,5.0),SmPoint3d(2.0,2.0,12.0));

        SmObjDelete sClean(pOther);

        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
        pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(60,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);
        sClean.Clear();
        MYPRINTF(_T("\nCalling my_test_bbu 7"));
        SER(my_test_bbu(pOther,pBrep,2,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }


    if (TRUE) { // Merge of Box and Wires
        SmBrep *pOther = my_make_wire_brep(sContext,SmPoint3d(5.0,5.0,-7.0),
            SmPoint3d(5.0,5.0,18.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(8.0,8.0,8.0),SmPoint3d(12.0,12.0,12.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(0.0,-2.0,0.0),SmPoint3d(0.0,12.0,0.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(6.0,-2.0,0.0),SmPoint3d(6.0,12.0,0.0));
        my_add_wire_to_brep(sContext,pOther,SmPoint3d(2.0,2.0,5.0),SmPoint3d(2.0,2.0,12.0));

        SmObjDelete sClean(pOther);

        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
        pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(80,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);
        sClean.Clear();
        MYPRINTF(_T("\nCalling my_test_bbu 8"));
        SER(my_test_bbu(pBrep,pOther,3,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }


    if (TRUE) { // Union of Box and Shell Vertex
        SmBrep *pOther = my_make_vertex_brep(sContext,SmPoint3d(5,5,5));
        SmVertex *pNewV;
        SmShell *pNewS;
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(5,5,0),pNewS,pNewV));
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(0,0,0),pNewS,pNewV));
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(5,0,0),pNewS,pNewV));
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(5,5,-5),pNewS,pNewV));
        SmObjDelete sClean(pOther);

        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
        pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(0,20,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);
        sClean.Clear();
        MYPRINTF(_T("\nCalling my_test_bbu 9"));
        SER(my_test_bbu(pBrep,pOther,0,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }

    if (TRUE) { // Intersection of Box and Shell Vertex
        SmBrep *pOther = my_make_vertex_brep(sContext,SmPoint3d(5,5,5));
        SmVertex *pNewV;
        SmShell *pNewS;
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(5,5,0),pNewS,pNewV));
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(0,0,0),pNewS,pNewV));
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(5,0,0),pNewS,pNewV));
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(5,5,-5),pNewS,pNewV));
        SmObjDelete sClean(pOther);

        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
        pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(20,20,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);
        sClean.Clear();
        MYPRINTF(_T("\nCalling my_test_bbu 10"));
        SER(my_test_bbu(pBrep,pOther,1,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }

    if (TRUE) { // Box - vertices
        SmBrep *pOther = my_make_vertex_brep(sContext,SmPoint3d(5,5,5));
        SmVertex *pNewV;
        SmShell *pNewS;
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(5,5,0),pNewS,pNewV));
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(0,0,0),pNewS,pNewV));
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(5,0,0),pNewS,pNewV));
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(5,5,-5),pNewS,pNewV));
        SmObjDelete sClean(pOther);
        
        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
        pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(40,20,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);
        sClean.Clear();
        MYPRINTF(_T("\nCalling my_test_bbu 11"));
        SER(my_test_bbu(pBrep,pOther,2,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }

    if (TRUE) { // Vertices - box
        SmBrep *pOther = my_make_vertex_brep(sContext,SmPoint3d(5,5,5));
        SmVertex *pNewV;
        SmShell *pNewS;
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(5,5,0),pNewS,pNewV));
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(0,0,0),pNewS,pNewV));
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(5,0,0),pNewS,pNewV));
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(5,5,-5),pNewS,pNewV));
        SmObjDelete sClean(pOther);

        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
        pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(60,20,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);
        sClean.Clear();
        MYPRINTF(_T("\nCalling my_test_bbu 12"));
        SER(my_test_bbu(pOther,pBrep,2,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }

    if (TRUE) { // Merge of vertices and box
        SmBrep *pOther = my_make_vertex_brep(sContext,SmPoint3d(5,5,5));
        SmVertex *pNewV;
        SmShell *pNewS;
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(5,5,0),pNewS,pNewV));
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(0,0,0),pNewS,pNewV));
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(5,0,0),pNewS,pNewV));
        SER(pOther->MakeShellVertex(pOther->GetInfiniteRegion(),SmPoint3d(5,5,-5),pNewS,pNewV));
        SmObjDelete sClean(pOther);
        
        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
        pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(80,20,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);
        sClean.Clear();
        MYPRINTF(_T("\nCalling my_test_bbu 13"));
        SER(my_test_bbu(pBrep,pOther,3,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }

    if (TRUE) { // Union of Box and Plane
        SmBrep* pOther = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/plane.smb"));
        pOther->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/plane.smb"),SM_ASCII );

        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
        pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(-80,40,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);

        MYPRINTF(_T("\nCalling my_test_bbu 13"));
        SER(my_test_bbu(pBrep,pOther,0,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }

    if (TRUE) { // Intersection of Box and Plane
        SmBrep* pOther = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/plane.smb"));
        pOther->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/plane.smb"),SM_ASCII );
        
        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
        pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(-50,40,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);

        MYPRINTF(_T("\nCalling my_test_bbu 14"));
        SER(my_test_bbu(pBrep,pOther,1,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }

    if (TRUE) { // Box - plane
        SmBrep* pOther = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/plane.smb"));
        pOther->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/plane.smb"),SM_ASCII );

        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
        pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(-20,40,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);

        MYPRINTF(_T("\nCalling my_test_bbu 15"));
        SER(my_test_bbu(pBrep,pOther,2,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }

    if (TRUE) { // Plane - box
        SmBrep* pOther = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/plane.smb"));
        pOther->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/plane.smb"),SM_ASCII );

        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
        pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(10,40,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);

        MYPRINTF(_T("\nCalling my_test_bbu 16"));
        SER(my_test_bbu(pOther,pBrep,2,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }

    if (TRUE) { // Merge of Box and Plane
        SmBrep* pOther = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/plane.smb"));
        pOther->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/plane.smb"),SM_ASCII );
        
        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
        pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(40,40,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);
        
        MYPRINTF(_T("\nCalling my_test_bbu 17"));
        SER(my_test_bbu(pBrep,pOther,3,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }

    if (TRUE) { // Union of Two Wires
        SmBrep *pOther = my_make_wire_brep(sContext,SmPoint3d(0.0,0.0,-5.0),
            SmPoint3d(0.0,0.0,5.0));
        SmBrep *pBrep = my_make_wire_brep(sContext,SmPoint3d(0.0,0.0,-5.0),
            SmPoint3d(5.0,0.0,0.0));

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(-50,80,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);

        MYPRINTF(_T("\nCalling my_test_bbu 18"));
        SER(my_test_bbu(pOther,pBrep,0,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }

    if (TRUE) { // Union of Two Wires
        SmBrep *pOther = my_make_wire_brep(sContext,SmPoint3d(0.0,0.0,-5.0),
            SmPoint3d(0.0,0.0,5.0));
        SmBrep *pBrep = my_make_wire_brep(sContext,SmPoint3d(-5.0,0.0,0.0),
            SmPoint3d(5.0,0.0,0.0));

        SmBrep *pResult;
        pBrep->m_bEditingEnabled = TRUE;
        pOther->m_bEditingEnabled = TRUE;
        SmAxis2Placement sXform;
        sXform.SetCanonical(SmPoint3d(-20,80,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        pBrep->Transform(sXform);
        pOther->Transform(sXform);

        MYPRINTF(_T("\nCalling my_test_bbu 19"));
        SER(my_test_bbu(pOther,pBrep,0,pResult));
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (pResult) pResult->Draw();
        }
#endif // SM_GFX_CODE
        SM_ASSERT(pResult != NULL) ; delete pResult ; pResult = NULL ;
    }

    return SM_SUCCESS;

} // end my_test_non_manifold


/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_merge()
{
  SmTArray<SmBrep*> sBreps;
  SmTArray<SmBrep*> sResults;
  SmContext sContext;
  double dTol = SM_ZONE_TOL_3D;

  MYPRINTF(_T("\n\n"));
  MYPRINTF(_T("********** Entered: my_test_merge"));

  if (FALSE) 
    {
      SmBrep *pBrep = new (sContext) SmBrep();
      pBrep->ReadFromFile(sContext,_T("SphericalSurf2.sm"));
      SmTArray<SmCurve*> sProjectedCurves;
      SmTArray<SmSurface*> sSurfaces;
      SmTArray<SmCurve*> s3DCurves;
      SmTArray<SmFace*> sFaces;
      // Create a curve and project it down to the brep
      SmBSplineCurve * pCurve1 = NULL;
      SER(SmBSplineCurve::ReadFromFile(sContext,_T("Curve2.sm"),pCurve1));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          smgfx_Erase();
          smgfx_SetColor(1,0,0); pBrep->Draw(); sm_GraphicsLoop();
          smgfx_SetColor(0,1,0); pCurve1->DrawWDeriv(pCurve1->GetNaturalInterval(),0); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
      
      SE(pBrep->CreateParallelProjectionCurves(sContext,*pCurve1,
          SmVector3d(0,-1,0),NULL,NULL,NULL,&s3DCurves,NULL,&sFaces));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          smgfx_Erase();
          smgfx_SetColor(1,0,0); pBrep->Draw(); sm_GraphicsLoop();
          smgfx_SetColor(0,1,0); pCurve1->DrawWDeriv(pCurve1->GetNaturalInterval(),0); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
      return SM_SUCCESS;
    }

  // Timing tests for Uri
  if (FALSE) 
    { 
      SmContext Context;
      SmBrep *pBrep1 = new (Context) SmBrep();
      SmBrep *pBrep2 = new (Context) SmBrep();
#ifdef SM_USE_EXCEPTIONS
      try 
        {
#endif // SM_USE_EXCEPTIONS
//        SmBrep *pBrep3 = new (Context) SmBrep();
      
//        SE(pBrep1->ReadFromFile(Context,"problem11blank.txt",SM_ASCII));
//        SE(pBrep2->ReadFromFile(Context,"problem11tool.txt",SM_ASCII));
      SE(pBrep1->ReadFromFile(Context,_T("maxbrep1.txt"),SM_ASCII));
      SE(pBrep2->ReadFromFile(Context,_T("maxbrep2.txt"),SM_ASCII));
      SE(pBrep1->ValidatePointers());
      SE(pBrep2->ValidatePointers());
      dTol = smos_Max(pBrep1->GetTolerance(),pBrep2->GetTolerance());
      dTol = SM_ZONE_TOL_3D;
      dTol = smos_Max(pBrep1->GetTolerance(),pBrep2->GetTolerance());
      SmVector3dAttribute *pNewAttr = new (Context) SmVector3dAttribute(SM_AI_COLOR,SmVector3d(0,0,1));
      pBrep2->AddAttribute(pNewAttr);

//        SE(pBrep1->ReadFromFile(Context,"Brep64.dat"));
//        SE(pBrep2->ReadFromFile(Context,"Brep65.dat"));
//        double dTol = 0.00001;
//        SE(pBrep3->ReadFromFile(Context,"Brep39.dat"));
//        SE(pBrep1->ShrinkGeometry());
//        SE(pBrep2->ShrinkGeometry()); 
      
      double ang=0.1;
      SmAxis2Placement ap;
      ap.SetCanonical(
          SmVector3d(0,0,0),
          SmVector3d(cos(ang),sin(ang),0),
          SmVector3d(-sin(ang),cos(ang),0));


//        SmTArray<SmBrep*> sBreps1;
//        SmTArray<SmBrep*> sBreps2;
//        for (int i=0;i<100;i++)
//        {
//            sBreps1.Add(new (Context) SmBrep(*pBrep1));
//            sBreps2.Add(new (Context) SmBrep(*pBrep2));
//        }

//        SmBoolean bViolatedTolerances;
//        double dMaxEdgeGap, dMaxVertexGap;
//        SER(pBrep2->ValidateAndUpdateTolerances(FALSE,FALSE,bViolatedTolerances,
//            dMaxEdgeGap,dMaxVertexGap));
      
//        SmBrep *pBrep3;


//        dTol = smos_Min(pBrep1->GetTolerance(),pBrep2->GetTolerance());
//        dTol = SM_ZONE_TOL_3D;
      SmMerge sMerge(Context,pBrep1,pBrep2,dTol,20.0*SM_PI/180.0);
      if (pBrep1->IsManifoldSolid() && pBrep2->IsManifoldSolid()) 
        {
          SE(sMerge.ManifoldBoolean(SM_BO_UNION,pBrep1));
        }
      else 
        {
          SE(sMerge.NonManifoldBoolean(SM_BO_UNION,pBrep1));
        }

//       pBrep1->ShrinkGeometry();

//        for (ULONG jj=0; jj<sBreps1.GetSize(); jj++) {
//            SmMerge sMerge(Context,sBreps1[jj],sBreps2[jj],dTol,20.0*SM_PI/180.0);
//            SE(sMerge.ManifoldBoolean(SM_BO_DIFFERENCE,pBrep3));
//        }


//            SmMerge sMerge2(Context,pBrep1,pBrep2,0.000001,20.0*SM_PI/180.0);
//            SE(sMerge2.ManifoldBoolean(SM_BO_DIFFERENCE,pBrep1));
//            ULONG lStitchedE, lLaminaE;
//            double dMaxVG, dMaxEG;
//            SER(pBrep1->StitchFaces(0.00001,lStitchedE,lLaminaE,dMaxVG,dMaxEG));
//            SER(pBrep1->MakeManifold());
//        pBrep1->Dump();
//        pBrep1->ValidatePointers();

//            pBrep2->Transform(ap);

//        SmTArray<SmRegion*> Regions;
//        pBrep1->GetRegions(Regions);
//        SmTArray<SmRegion*> OptRegionsToKeep;
//        int iReg=2;
//        OptRegionsToKeep.Add(Regions[iReg]);
//        SE(pBrep1->MakeManifold(&OptRegionsToKeep));
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics()) {
          smgfx_Erase();
          pBrep1->Draw();
          pBrep1->Dump();
          pBrep1->ValidatePointers();
          sm_GraphicsLoop();
      }
#endif // SM_GFX_CODE

#ifdef SM_GFX_CODE
SmBoolean bDrawReg = FALSE;
      if (bDrawReg) {        
          smgfx_Erase();
          pBrep1->Draw();
          sm_GraphicsLoop();
          smgfx_Erase();
          SmTArray<SmRegion*> sRegions;
          pBrep1->GetRegions(sRegions);
          for (ULONG mm=0; mm<sRegions.GetSize(); mm++) {
              smgfx_Erase();
              sRegions[mm]->Draw();
              sm_GraphicsLoop();
              if (sRegions[mm] == pBrep1->GetInfiniteRegion()) {
                  SE(SM_ERR);
              }
              smgfx_Erase();
              SmTArray<SmShell*> sRegShells;
              sRegions[mm]->GetShells(sRegShells);
              for (ULONG is=0; is<sRegShells.GetSize(); is++) {
                  SmTArray<SmFaceuse*> sRegFaceuses;
                  sRegShells[is]->GetFaceuses(sRegFaceuses);
                  for (ULONG ir=0; ir<sRegFaceuses.GetSize(); ir++) {
                      sRegFaceuses[ir]->GetFace()->DrawUV(4,4);
                      sm_GraphicsLoop();
                  }
              }
          }
      }
#endif // SM_GFX_CODE
#ifdef SM_USE_EXCEPTIONS
        }
      catch (std::bad_alloc& e) {
          if(pBrep1) { delete pBrep1; pBrep1 = NULL ; }
          if(pBrep2) { delete pBrep2; pBrep2 = NULL ; }
          TCHAR sBuffer[SM_TBLOCK_SIZE];  
          SM_SPRINTF(sBuffer, _T("std::bad_alloc: %s\n"), smos_ToTChar(e.what()).c_str());
          MYPRINTF(sBuffer);
      }

#endif // SM_USE_EXCEPTIONS
      return SM_SUCCESS;
  }

  if (TRUE) 
    {
      my_test_non_manifold();
    }

  smos_WriteBuffer(_T("********* Testing Booleans with Primitives from Primitive Creation *********"));
  // Test some of the basic primitive creation with booleans
  if (TRUE) { // Two Boxes
      SmBrep *pBrep = new (sContext) SmBrep();
      pBrep->SetTolerance(SM_ZONE_TOL_3D);
      SmBrep *pBrep2 = new (sContext) SmBrep();
      pBrep2->SetTolerance(SM_ZONE_TOL_3D);
      SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
      SmAxis2Placement sRef;
      sPC.CreateBox(10,10,10,sRef);
      sRef.Translate(SmVector3d(2.0,2.0,0.0));
      SmPrimitiveCreation sPC2(pBrep2->GetInfiniteRegion());
      sPC2.CreateBox(10.0,7.0,5.0,sRef);
      dTol = pBrep->GetTolerance();
      SmMerge sMerge(sContext,pBrep,pBrep2,dTol,20.0*SM_PI/180.0);
      SE(sMerge.ManifoldBoolean(SM_BO_UNION,pBrep));
      SmTArray<SmEdge*> sEdges;
      pBrep->GetEdges(sEdges);
      SM_ASSERT(sEdges.GetSize() == 24);
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          smgfx_Erase();
          smgfx_SetLook(1,2) ; if(pBrep) pBrep->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
      if(pBrep) { pBrep->Dump() ; delete pBrep ; pBrep = NULL ; }
  }

  if (TRUE) { // Two Cylinders
      SmBrep *pBrep = new (sContext) SmBrep();
      pBrep->SetTolerance(SM_ZONE_TOL_3D);
      SmBrep *pBrep2 = new (sContext) SmBrep();
      pBrep2->SetTolerance(SM_ZONE_TOL_3D);
      SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
      SmAxis2Placement sRef;
      sPC.CreateCone(10.0,3.0,2.0,0.0,360.0,sRef);
      sRef.Translate(SmVector3d(2.0,2.0,0.0));
      SmPrimitiveCreation sPC2(pBrep2->GetInfiniteRegion());
      sPC2.CreateCone(10.0,4.0,3.0,0.0,360.0,sRef);
      dTol = pBrep->GetTolerance();
      SmMerge sMerge(sContext,pBrep,pBrep2,dTol,20.0*SM_PI/180.0);
      SE(sMerge.ManifoldBoolean(SM_BO_UNION,pBrep));
      SmTArray<SmEdge*> sEdges;
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          smgfx_Erase();
          smgfx_SetLook(1,2) ; if(pBrep) pBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
      if(pBrep) { pBrep->Dump() ; delete pBrep ; pBrep = NULL ; }
     // SM_ASSERT(sEdges.GetSize() == 9);
  }
  
  if (TRUE) { // Two Spheres
      SmBrep *pBrep = new (sContext) SmBrep();
      pBrep->SetTolerance(SM_ZONE_TOL_3D);
      SmBrep *pBrep2 = new (sContext) SmBrep();
      pBrep2->SetTolerance(SM_ZONE_TOL_3D);
      SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
      SmAxis2Placement sRef;
      sPC.CreateSphere(10.0,0.0,360.0,sRef);
      sRef.Translate(SmVector3d(5.0,2.0,0.0));
      SmPrimitiveCreation sPC2(pBrep2->GetInfiniteRegion());
      sPC2.CreateSphere(10.0,0.0,360.0,sRef);
      dTol = pBrep->GetTolerance();
      SmMerge sMerge(sContext,pBrep,pBrep2,dTol,20.0*SM_PI/180.0);
      SE(sMerge.ManifoldBoolean(SM_BO_UNION,pBrep));
      SmTArray<SmEdge*> sEdges;
      pBrep->GetEdges(sEdges);
      SM_ASSERT(sEdges.GetSize() == 6);
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          smgfx_Erase();
          smgfx_SetLook(1,2) ; if(pBrep) pBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
      if(pBrep) { pBrep->Dump() ; delete pBrep ; pBrep = NULL ; }
  }

  if (TRUE) { // Two Tori
      SmBrep *pBrep  = new (sContext) SmBrep();
      SmBrep *pBrep2 = new (sContext) SmBrep();
      pBrep->SetTolerance (SM_ZONE_TOL_3D);
      pBrep2->SetTolerance(SM_ZONE_TOL_3D);
      SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
      SmAxis2Placement sRef;
      sPC.CreateTorus(10.0,4.0,0.0,360.0,sRef);
      sRef.Translate(SmVector3d(0.0,10.0,0.0));
      SmPrimitiveCreation sPC2(pBrep2->GetInfiniteRegion());
      sPC2.CreateTorus(10.0,4.0,0.0,360.0,sRef);
      dTol = pBrep->GetTolerance();

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep)  pBrep ->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) pBrep2->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      SmMerge sMerge(sContext,pBrep,pBrep2,dTol,20.0*SM_PI/180.0);
      SE(sMerge.ManifoldBoolean(SM_BO_UNION,pBrep));
      //SmTArray<SmEdge*> sEdges;
      //pBrep->GetEdges(sEdges);
      //SM_ASSERT(sEdges.GetSize() == 22); // was 22 rclxx
      // Edge count depends on Tolerance of breps
      // New Analytic functions results in 28 edges
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          smgfx_Erase();
          smgfx_SetLook(1,2) ; if(pBrep) pBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
      if(pBrep) { pBrep->Dump() ; delete pBrep ; pBrep = NULL ; }
  }

  smos_WriteBuffer(_T("********* Testing Piecewise Merge *********"));
  if (TRUE) { // Piecewise Merge Test

      SmBrep* pBrep = new(sContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
      pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

      SmBrep *pOther = new (sContext) SmBrep();
      pOther->SetTolerance(pBrep->GetTolerance());
      SER(pOther->MergeBrep(*pBrep));

      SmAxis2Placement sMove;
      sMove.Translate(SmVector3d(5,5,5));
      pOther->Transform(sMove);

      pBrep->m_bEditingEnabled = TRUE;
      pOther->m_bEditingEnabled = TRUE;

      SmBrep *pResult = NULL; 
      SmMerge sMerge(sContext,pBrep,pOther);
      SER(sMerge.PiecewiseMerge(NULL,TRUE,FALSE,pResult));

      SM_ASSERT(pOther == sMerge.GetTopologyIntersector().GetOtherBrep()) ;
      SM_ASSERT(pResult != NULL) ; 
      SM_ASSERT(pOther  != NULL) ;
      if(pResult) { pResult->Dump(); delete pResult ; pResult = NULL ; }
      if(pOther)  { delete pOther ;  pOther  = NULL ; }
      sMerge.GetTopologyIntersector().SetOtherBrep(NULL) ;
  }

  if (TRUE) { // Piecewise Merge Test
      SmBrep* pBrep = new(sContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
      pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

      SmBrep *pOther = new (sContext) SmBrep();
      pOther->SetTolerance(pBrep->GetTolerance());

      // copy pBrep into empty pOther
      SER(pOther->MergeBrep(*pBrep));

      // move pOTher
      SmAxis2Placement sMove;
      sMove.Translate(SmVector3d(5,5,5));
      pOther->Transform(sMove);

      pBrep->m_bEditingEnabled = TRUE;
      pOther->m_bEditingEnabled = TRUE;

      SmBrep *pResult = NULL ;

      if(smGet_DoGraphics())
        {
          MYPRINTF(_T("\nCall pBrep->Dump() - Drawn in Blue")) ;
          pBrep->Dump() ;
          MYPRINTF(_T("\nCall pOther->Dump() - Drawn in Green")) ;
          pOther->Dump() ;
          MYPRINTF(_T("\nCall SmMerge sMerge(sContext,pBrep,pOther);")) ;
          MYPRINTF(_T("\nCall sMerge.PiecewiseMerge(NULL,TRUE,TRUE,pResult)")) ;
        }
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep)  pBrep ->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,0) ; if(pOther) pOther->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      SmMerge sMerge(sContext,pBrep,pOther);
      SER(sMerge.PiecewiseMerge(NULL,TRUE,TRUE,pResult));
      
      SM_ASSERT(pOther == sMerge.GetTopologyIntersector().GetOtherBrep()) ;
      SM_ASSERT(pResult != NULL) ;
      SM_ASSERT(pOther  != NULL) ;
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          smgfx_Erase();
          smgfx_SetLook(2,3, 1,0,0) ; if(pResult)  pResult ->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
      if(pResult) { if(smGet_DoGraphics()) { MYPRINTF(_T("\nCall pResult->Dump() - Drawn in Red")) ; }
                    pResult->Dump(); delete pResult ; pResult = NULL ; 
                  }
      if(pOther)  { delete pOther ;  pOther  = NULL ; }
      sMerge.GetTopologyIntersector().SetOtherBrep(NULL) ;
  }


  if (TRUE) { // Merge of Box with a copy of itself
      SmBrep* pBrep = new(sContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
      pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII );

      SmBrep *pOther = new (sContext) SmBrep();
      pOther->SetTolerance(pBrep->GetTolerance());
      SER(pOther->MergeBrep(*pBrep));

      pBrep->m_bEditingEnabled = TRUE;
      pOther->m_bEditingEnabled = TRUE;

      MYPRINTF(_T("\nCalling my_test_bbu 20"));

      SmBrep *pResult;
      SER(my_test_bbu(pOther,pBrep,3,pResult));
      SM_ASSERT(pResult != NULL) 
      if(pResult) { pResult->Dump() ; delete pResult ; pResult = NULL ; }
  }

  smos_WriteBuffer(_T("********* Testing Stitching Demo *********"));
  if (TRUE) {
      my_stitching_demo();
  }

  smos_WriteBuffer(_T("********* Testing Mouse Demo *********"));
  if (TRUE) {
      my_mouse_demo();
  }

  smos_WriteBuffer(_T("********* Testing Stress Booleans *********"));
  if (TRUE) 
    {
      my_test_stress();
    }

  smos_WriteBuffer(_T("********* Testing Non-Manifold Booleans *********"));
  if (TRUE) 
    {
      my_test_non_manifold();
    }

  smos_WriteBuffer(_T("********* Testing One Brep Per Shell *********"));
  if (TRUE) 
    {
      my_test_BrepPerShell();
    }
  // Real tests here and below
  
  if (TRUE) { // Union all of the primitives
      SmTArray<SmCurve*> s3DCurves;
      SmTArray<SmSurface*> sSurfaces;
      SmTArray<long> sBooleanTrees;
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/boolean_trees.smp"));
      SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/boolean_trees.smp"),
                                   s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );

      SmBrep * pResult = NULL ;
      ULONG lTotalEdges;
      my_many_prim_test(sBreps,dTol,0,lTotalEdges,pResult);
      
      SM_ASSERT(pResult != NULL) ; 
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics()) {
          if (pResult) pResult->Draw();
      }
#endif // SM_GFX_CODE
      if(pResult) {pResult->Dump(); delete pResult ; pResult = NULL ; }
      SM_ASSERT(lTotalEdges == 45);
  }

  if (TRUE) { // Merge all of these primitives
      SmTArray<SmCurve*> s3DCurves;
      SmTArray<SmSurface*> sSurfaces;
      SmTArray<long> sBooleanTrees;
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/boolean_trees.smp"));
      SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/boolean_trees.smp"),
                    s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );
      SmBrep * pResult;
      ULONG lTotalEdges;
      my_many_prim_test(sBreps,dTol,3,lTotalEdges,pResult);

      SM_ASSERT(pResult != NULL) ; 
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics()) {
          if (pResult) pResult->Draw();
      }
#endif // SM_GFX_CODE
      if(pResult) {pResult->Dump(); delete pResult ; pResult = NULL ; }
      SM_ASSERT(lTotalEdges == 59);
  }

  if (TRUE) {
      SmTArray<SmCurve*> s3DCurves;
      SmTArray<SmSurface*> sSurfaces;
      SmTArray<long> sBooleanTrees;
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box_box_grid.smp"));
      SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box_box_grid.smp"),
                    s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );

      SmObjsDelete<SmBrep*> sClean1(&sBreps);
      ULONG lTotalEdges;
      SmObjsDelete<SmBrep*> sClean2(&sResults);
      my_two_prim_test(sBreps,0,lTotalEdges,sResults);
      //sClean1.Clear();
      SM_ASSERT(lTotalEdges == 659);
  }

  if (TRUE) {
      SmTArray<SmCurve*> s3DCurves;
      SmTArray<SmSurface*> sSurfaces;
      SmTArray<long> sBooleanTrees;
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box_cyl_grid.smp"));
      SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box_cyl_grid.smp"),
                    s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe && smGet_DoGraphics())
    {
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

      SmObjsDelete<SmBrep*> sClean1(&sBreps);
      ULONG lTotalEdges;
      SmObjsDelete<SmBrep*> sClean2(&sResults);
      my_two_prim_test(sBreps,0,lTotalEdges,sResults);
      //sClean1.Clear();
      SM_ASSERT(lTotalEdges == 499);  // used to be 501 then 497 now 499
  }


  if (TRUE) {
      SmTArray<SmCurve*> s3DCurves;
      SmTArray<SmSurface*> sSurfaces;
      SmTArray<long> sBooleanTrees;
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box_cone_grid.smp"));
      SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Solids/box_cone_grid.smp"),
                    s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );

      SmObjsDelete<SmBrep*> sClean1(&sBreps);
      ULONG lTotalEdges;
      SmObjsDelete<SmBrep*> sClean2(&sResults);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe && smGet_DoGraphics())
    {
      SM_ASSERT_VALID(sBreps[0]) ;
      SM_ASSERT_VALID(sBreps[1]) ;
      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(sBreps[0]) sBreps[0]->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(sBreps[1]) sBreps[1]->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

      my_two_prim_test(sBreps,0,lTotalEdges,sResults);
      //sClean1.Clear();
      SM_ASSERT(lTotalEdges == 447);  // originally 444 then 447
  }

  // Test using an offset surface in a SmBrep
  if (TRUE) {
      SmBrep* pBrep = new(sContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Fillet/nurb_nurb_1.smb"));
      pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Fillet/nurb_nurb_1.smb"),SM_ASCII );

      SmBrep* pOther = new(sContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Fillet/nurb_nurb_2.smb"));
      pOther->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Fillet/nurb_nurb_2.smb"),SM_ASCII );

      SmObjDelete sCleanBrep( pBrep ), sCleanOther( pOther );

      SmTArray<SmFace*> sFaces;
      pBrep->GetFaces(sFaces);
      SmFace * pFace = sFaces[0];
      pOther->GetFaces(sFaces);
      SmFace * pFace2 = sFaces[0];
      SmSurface *pSur1 = pFace->GetSurface();
      SmSurface *pSur2 = pFace2->GetSurface();
      SER(pSur1->Copy(sContext,pSur1));
      SER(pSur2->Copy(sContext,pSur2));
      // Create the first offset surface 
      double dFilletRadius1 = 5.0;
      double dFilletRadius2 = 5.0;
      SmOffsetSurface * pOff1 = new(sContext) 
          SmOffsetSurface(dFilletRadius1,*pSur1,TRUE);
      
      // Create the second offset surface
      SmOffsetSurface * pOff2 = new(sContext) 
          SmOffsetSurface(dFilletRadius2,*pSur2,TRUE);
      
      SmFace *pNewF;
      
      SmBrep *pBrep3 = new(sContext) SmBrep();
      SER(pBrep3->CreateFaceFromSurface(pOff1,
          pOff1->GetNaturalUVDomain(),pNewF));
      SmBrep *pBrep4 = new(sContext) SmBrep();
      SER(pBrep4->CreateFaceFromSurface(pOff2,
          pOff2->GetNaturalUVDomain(),pNewF));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe && smGet_DoGraphics()) {
          smgfx_Erase();
          pBrep3->Draw();
          pBrep4->Draw();
      }
#endif // SM_DEBUG_CODE
      
      MYPRINTF(_T("\nCalling my_test_bbu 21"));

      SmBrep *pResult;
      SER(my_test_bbu(pBrep3,pBrep4,3,pResult));

      SmTArray<SmEdge*> sEdges;
      pResult->GetEdges(sEdges);
      SM_ASSERT(pResult != NULL) ; 
#ifdef SM_DEBUG_CODE
      if (bDebugMe && smGet_DoGraphics()) {
          smgfx_Erase();
          if(pResult) pResult->Draw();
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE
      if(pResult) { pResult -> ~SmBrep(); delete pResult ; pResult = NULL ; } // JLMCC memory leak
      SM_ASSERT(sEdges.GetSize() == 11);
  }

  dTol = 1.0e-3;

  return SM_SUCCESS;

} // end my_test_merge

/***********************************************************************
PURPOSE --- Boolean between 2 Breps - output results

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_test_bbu
 (SmBrep *  cpBrep1,       // in : target brep 1
  SmBrep *  cpBrep2,       // in : target brep 2
  ULONG     lOperation,    // in : 0 - union, 1 - intersection, 2 - difference, 3 - merge, 4 - exclusive or
                           //      31 - Merge with Imprintng      32 - Merge with CookieCutter
  SmBrep *& rpResult)      // out: the merge result
{
  MYPRINTF(_T(": In my_test_bbu")) ;

  // for I/O debugging - write Breps to file
static SmBoolean sbTestIO = FALSE;
  if (sbTestIO) 
    {

      // write breps to file - for later use
      SER(cpBrep1->WriteToFile(_T("../prog_test/OutputFiles/Brep1.smb")));
      SER(cpBrep2->WriteToFile(_T("../prog_test/OutputFiles/Brep2.smb")));
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
ULONG lCount = 1 ; lCount++ ;
ULONG lDebugCount = 0 ; 
  if (smGet_DoGraphics() && (bDebugMe || lDebugCount == lCount))
    {
      // check inputs
      SM_ASSERT_VALID(cpBrep1) ;
      SM_ASSERT_VALID(cpBrep2) ;
      SM_DUMP(cpBrep1) ;
      SM_DUMP(cpBrep2) ; 

#ifdef SM_GFX_CODE
      if (smGet_DoGraphics()) 
        {
          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1); cpBrep1->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0); cpBrep2->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
    }
#endif // SM_DEBUG_CODE

  // translate large parts to their BBox center   
  // Note: don't do this for debugging where you need the true values

  // get parts bbox
  SmExtent3d sBBox, sBBox2;
  cpBrep1->CalculateBoundingBox(sBBox);
  cpBrep2->CalculateBoundingBox(sBBox2);
  sBBox.Union(sBBox2, sBBox);
    
  // when bbox is very large - translate its center to the origin
  SmBoolean bTrans = FALSE;
  SmPoint3d sCenter = sBBox.Evaluate(0.5, 0.5, 0.5);
  if (sCenter.LengthSquared() > 1.0e7) 
    { 
      bTrans = TRUE;
      sCenter = -sCenter;
      SmAxis2Placement sRef;
      sRef.Translate(sCenter);
      cpBrep1->Transform(sRef);
      cpBrep2->Transform(sRef);
    }

#ifdef SM_DEBUG_CODE
    TCHAR sBuff[SM_TBLOCK_SIZE] ;
    SM_SPRINTF(sBuff, _T("\n  BooleanOp:[%s] "),
                 lOperation==0 ? _T("Union")
               : lOperation==1 ? _T("Intersection")
               : lOperation==2 ? _T("Difference")
               : lOperation==3 ? _T("Merge")
               : lOperation==31 ? _T("Merge with Imprinting")
               : lOperation==32 ? _T("Merge with Cookie Cutter")
               : lOperation==4 ? _T("Exclusive Or")
               : _T("unknown")) ;
    smos_WriteBuffer(sBuff);
#endif // SM_DEBUG_CODE

  SmMerge sMerge( *(cpBrep1->GetContext()), cpBrep1, cpBrep2 );

  switch(lOperation)
    {
      case 0: SER(sMerge.ManifoldBoolean(SM_BO_UNION,       rpResult)); break ;
      case 1: SER(sMerge.ManifoldBoolean(SM_BO_INTERSECTION,rpResult)); break ;
      case 2: SER(sMerge.ManifoldBoolean(SM_BO_DIFFERENCE,  rpResult)); break ;
      case 3: SER(sMerge.ManifoldBoolean(SM_BO_MERGE,       rpResult)); break ;
      case 31: 
              sMerge.SetImprinting(TRUE);  // Just project pBrep onto pBrep2
              SER(sMerge.ManifoldBoolean(SM_BO_MERGE,       rpResult)); 
              break ;
      case 32: 
              sMerge.SetCookieCutter(TRUE) ;    
              SER(sMerge.ManifoldBoolean(SM_BO_MERGE,       rpResult)); 
              break ;
      case 4: SER(sMerge.ManifoldBoolean(SM_BO_EXCLUSIVE_OR,rpResult)); break ;
      default: SER(SM_ERR);
    }
    
  // when needed - Transform the result back into original 3d space
  if (bTrans) 
    {
      sCenter = -sCenter;
      SmAxis2Placement sRef2;
      sRef2.Translate(sCenter);
      rpResult->Transform(sRef2);
    }

  // add a unique fact to the output data stream to help identify changes
#ifdef SM_DEBUG_CODE
if(rpResult) 
    { SmTArray<SmEdge*> sEdges ;
      rpResult->GetEdges(sEdges) ;
      SM_SPRINTF(sBuff, _T(": lResultEdgeCount[%ld] \n"),sEdges.GetSize()) ;
      MYPRINTF(sBuff) ;
    }

  // draw merge results
  if (bDebugMe || (smGet_DoGraphics() && sbTestIO == FALSE))
    {
      SM_ASSERT_VALID(rpResult);
#ifdef SM_GFX_CODE
      smgfx_Erase();
      smgfx_ClearColor(); if (rpResult) rpResult->Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
#endif // SM_GFX_CODE
    }
#endif // SM_DEBUG_CODE

  // when testing I/O - read in writtenOut Brep arguments - redo boolean with written/read breps - compare boolean results
  if (sbTestIO) 
    {
      SmTArray<SmVertex*> sVertices;
      SmTArray<SmEdge*>   sEdges;
// Remove Composites
//      SmTArray<SmCFace*>  sCFaces;
//      SmTArray<SmCEdge*>  sCEdges;

      rpResult->GetVertices(sVertices);
      rpResult->GetEdges(sEdges);
// Remove Composites
//      rpResult->GetCFaces(sCFaces);
//      rpResult->GetCEdges(sCEdges);

      // read parts from file
      SmContext sContext;
      SmBrep *pBrep1 = new (sContext) SmBrep();  
      SmBrep *pBrep2 = new (sContext) SmBrep();  
      SmBrep *pResult = NULL;
      SER(pBrep1->ReadFromFile(sContext,_T("../prog_test/OutputFiles/Brep1.smb"))); /* written before merge in this call */
      SER(pBrep2->ReadFromFile(sContext,_T("../prog_test/OutputFiles/Brep2.smb"))); /* written before merge in this call */

#ifdef SM_DEBUG_CODE
      if (bDebugMe && smGet_DoGraphics())
        {
          cpBrep1->Dump();
          cpBrep2->Dump();
#ifdef SM_GFX_CODE
          smgfx_Erase() ;
          smgfx_ClearColor() ; cpBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetColor(0,0,1) ; cpBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
          sm_GraphicsLoop() ;
#endif // SM_GFX_CODE
        }
#endif // SM_DEBUG_CODE

      // setup and re-execute the merge
      SmMerge sMerge2(sContext,pBrep1,pBrep2);
      switch(lOperation)
        {
          case 0: SER(sMerge2.ManifoldBoolean(SM_BO_UNION,       pResult)); break ;
          case 1: SER(sMerge2.ManifoldBoolean(SM_BO_INTERSECTION,pResult)); break ;
          case 2: SER(sMerge2.ManifoldBoolean(SM_BO_DIFFERENCE,  pResult)); break ;
          case 3: SER(sMerge2.ManifoldBoolean(SM_BO_MERGE,       pResult)); break ;
          case 4: SER(sMerge2.ManifoldBoolean(SM_BO_EXCLUSIVE_OR,pResult)); break ;
          default: SER(SM_ERR);
        }
        
      // compare read Result and originalBrep result entity counts
      if ( !pResult) return (SM_ERR);
      // vertices
      SmTArray<SmVertex*> sVertices2;
      pResult->GetVertices(sVertices2);
      SM_ASSERT(sVertices2.GetSize() == sVertices.GetSize());

      // edges
      SmTArray<SmEdge*> sEdges2;
      pResult->GetEdges(sEdges2);
      SM_ASSERT(sEdges2.GetSize() == sEdges.GetSize());

// Remove Composites
//      // composite faces
//      SmTArray<SmCFace*> sCFaces2;
//      pResult->GetCFaces(sCFaces2);
//      SM_ASSERT(sCFaces2.GetSize() == sCFaces.GetSize());
//
//      // composite edges
//      SmTArray<SmCEdge*> sCEdges2;
//      pResult->GetCEdges(sCEdges2);
//      SM_ASSERT(sCEdges2.GetSize() == sCEdges.GetSize());


#ifdef SM_DEBUG_CODE
      // draw original and I/O merge results (should be the same)
      if (bDebugMe || smGet_DoGraphics())
        {
          rpResult->Dump(); // boolean result of input arguments
          pResult->Dump();  // boolean result of write/read input arguments (hopefully the same)

          smgfx_Erase();
          smgfx_SetColor(1,0,0); if(rpResult) rpResult->Draw(); sm_GraphicsLoop() ;
          smgfx_SetColor(0,0,1); if(pResult) pResult->Draw(); sm_GraphicsLoop() ;  
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    } // end asked to check Brep I/O Check

  // all done
  return SM_SUCCESS;

} // end my_test_bbu
