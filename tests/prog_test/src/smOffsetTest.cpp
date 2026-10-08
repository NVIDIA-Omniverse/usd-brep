// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smoffset_test.cpp
* PURPOSE --- Source code file for testing the offsetting
*
**********************************************************************/
/*___*/

#include "StdAfx.h"
#include <SmOffsetGeometryCreation.h>
#include <SmOffsetExecutive.h>

#include <smOffsetTest.h>
#include <smMergeTest.h>


#define DELETE_ALL_PARTS2(parts) \
{ for (ULONG z=0; z<(parts).GetSize(); z++) \
    { SM_ASSERT((parts)[z] != NULL) ; delete (parts)[z]; (parts)[z] = NULL ; } \
  (parts).ReSet(); }

/***********************************************************************
PURPOSE --- Silent run of regression for offset

USAGE NOTES --- 
***********************************************************************/
SmStatus my_test_offset
  (const SmContext   & crContext,           // in : context for new object construction
   const TCHAR       * pFileName,           // in : file to read
   ULONG               lShellFace,          // in : This faceIndex->Face just gets copied, not offset
   double              dOffsetRadius,       // in : amount to offset faces (pos or neg ok)
   double              dTolerance,          // in : Tolerance passed to approximate shape functions
   SmTArray<SmBrep*> & rPartBreps)          // out: rPartBreps[0] should be the result
{
  // inform the log
  MYPRINTF(_T("\nEntered my_test_offset"));

  // read in the target file
  SmBrep* pBrep = new(crContext) SmBrep();
  pBrep->ReadFromFile( crContext, pFileName, SM_ASCII );
 
  SmObjDelete sClean(pBrep);

  // when asked - get ShellFaceIndex->Face
  SmTArray<SmFace*> sShellFaces;
  SmFace *pShellFace = NULL ;
  if (lShellFace > 0) 
    {
      pBrep->GetFaces(sShellFaces);
      pShellFace = sShellFaces[lShellFace-1];
      sShellFaces.ReSet();
      sShellFaces.Add(pShellFace);
    }

  // set inset flag and normalize offset distance
  SmBoolean bInset = dOffsetRadius < 0.0 ;
  if (bInset) { dOffsetRadius = smos_Fabs(dOffsetRadius); }

  // make the OffsetExecutive - set its parameters
  SmOffsetGeometryCreation sOGC(dOffsetRadius,dTolerance);
  SmOffsetExecutive sExec(crContext,SM_OO_SOLID_OFFSET,sOGC,pBrep);
  if (bInset)         { sExec.SetInset(TRUE); }
  if (lShellFace > 0) { rPartBreps.Add(pBrep);
                        sClean.Clear();
                        sExec.SetShellFaces(sShellFaces);
                      }
  sExec.SetExtendConvexEdges(FALSE);
  sExec.SetMergeResults(TRUE);

#ifdef SM_GFX_CODE
  // draw inputs
  if(smGet_DoGraphics() && pBrep)
    {
      SM_ASSERT_VALID_AND_DUMP(pBrep) ;
      

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pShellFace) pShellFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_GFX_CODE

  // build the offset Brep
  SmBrep *pResult = NULL;
  SER(sExec.DoSolidOffset(pResult));  // note: increments unlocked mark value

  /*
  {
      SmStatus sErr = (sExec.DoSolidOffset(pResult));
      if (sErr != SM_SUCCESS)
      {
          if (pResult)
          {
              delete pResult;
              pResult = NULL;
          }

          smos_ErrorMessage(sErr, FILE_NAME, LINE_NUMBER, (TCHAR*)0, (TCHAR*)0, FUNC_NAME);
          return (sErr);
      }
  }
  */

  // inform the public - save output
  pResult->Dump();
  pResult->ValidatePointers();
  rPartBreps.Add(pResult);

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics() && pResult)
    {
      smgfx_Erase() ;
      smgfx_SetLook(1,2) ; pResult->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_GFX_CODE

  // all done
  return SM_SUCCESS;

} // end my_test_offset


/***********************************************************************
PURPOSE --- Silent run of regression for offset

USAGE NOTES --- Used by prog_test, calls my_offset_demo
***********************************************************************/
PT_EXPORT SmStatus my_offset_regression()  
{
  MYPRINTF(_T("\nEntered my_offset_regression 2")) ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  SmContext sContext;
  ULONG i, lCount = 0;
  SmTArray<SmBrep*> sBreps;
  // there are 17 tests not 9 !
  for (i=0; i<17 && lCount < 17; i++)  
    {
      SM_SPRINTF(sBuff,_T("\nCalling my_offset_demo() iter:%ld"),i) ;
      // change lCount to jump to various test cases
      MYPRINTF(sBuff);
      my_offset_demo(sContext,sBreps,lCount);

#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe && smGet_DoGraphics())
        {
         sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
        DELETE_ALL_PARTS2(sBreps);
    }

  

  lCount = 0;
  for (i=0; i<16 && lCount <16; i++) 
    {
      SM_SPRINTF(sBuff,_T("\nCalling my_shell_demo() iter:%ld"),i) ;
      // change lCount to jump to various test cases
      MYPRINTF(sBuff);
      my_shell_demo(sContext, sBreps, lCount);

#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE ;
      if(bDebugMe && smGet_DoGraphics())
        {
          sm_GraphicsLoop() ;
        }
#endif // SM_GFX_CODE
        DELETE_ALL_PARTS2(sBreps);
    }

  

  return SM_SUCCESS;

} // end my_offset_regression

/***********************************************************************
PURPOSE --- Silent run of regression for offset

USAGE NOTES --- 
***********************************************************************/
PT_EXPORT SmStatus ShellBrep
( const SmContext       & crContext,       // in : context for new faces
 const TCHAR            * filename,        // in : file with starter Brep model
 double                  dOffsetDistance,  // in : Offset distance - if negative do an inset.
 SmBoolean               bExtendCorners,   // in : TRUE =
                                           //      FALSE= 
 SmBoolean               bMergeResults,    // NotUsed: in : Not used in this function
 const SmTArray<ULONG> & crFacesToShell,   // in : list of face indices to shell -
                                           //      They get copied without being offset
 SmTArray<SmBrep*>     & rPartBreps )      // out:
{
  SM_REF1(bMergeResults) ; 
  // publish entry
  MYPRINTF( _T( "\nEntered ShellBrep " ) );

  // read brep
  SmBrep *pBrep = new(crContext) SmBrep();

  SER( pBrep->ReadFromFile( crContext, filename, SM_ASCII ) );
  SmTArray<SmFace*> sShellFaces, sShellFaces1;

  // get brep faces
  pBrep->GetFaces( sShellFaces );

  // place every face specified by index onto sShellFaces1 list
  ULONG jj;
  for(jj = 0; jj < crFacesToShell.GetSize(); jj++)
  {
    SmFace *pShellFace1 = sShellFaces[crFacesToShell[jj]];
    sShellFaces1.Add( pShellFace1 );
  }

#ifdef SM_DEBUG_CODE
ULONG di ;
SmBoolean bDebugMe = FALSE ;
#    ifdef SM_GFX_CODE
  // draw and dump input Brep(blue)
  if(smGet_DoGraphics() && pBrep)
  {
    SmTArray<SmShell*> sShells ;
    pBrep->GetShells(sShells) ; 
    SM_DUMP_AND_ASSERT_VALID( pBrep );

    smgfx_Erase();
    smgfx_SetLook(1,4, 0,0,1); pBrep->Draw( TRUE ); sm_GraphicsLoop();
    for(di=0;di<sShellFaces1.GetSize();di++)
    {
      smgfx_SetLook(1,4, 0,0,0); sShellFaces1[di]->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop();
      smgfx_SetLook(1,4, 0,1,1); sShellFaces1[di]->DrawLoopusesForFaceuse(); sm_GraphicsLoop();
      smgfx_SetLook(1,4, 0,1,1); sShellFaces1[di]->GetSurface()->DrawUV(); sm_GraphicsLoop();
      smgfx_SetDrawCurvature( TRUE,  -55, 125 );
      smgfx_SetLook(1,4, 0,1,1); sShellFaces1[di]->GetSurface()->DrawCurvature(); sm_GraphicsLoop();
      smgfx_SetDrawCurvature( FALSE, -55, 125 );
      sm_GraphicsLoop();
    }

    smgfx_Erase();
    smgfx_SetLook(1,4, 0,0,1); pBrep->Draw( TRUE ); sm_GraphicsLoop();
    for (di = 0; di < sShells.GetSize(); di++)
    {
      smgfx_SetLook(3,4, 1,0,1); sShells[di]->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }

    sm_GraphicsLoop();
    sm_GraphicsBrepListClear(); // JLMCC hunting memory leaks
  }
#endif // SM_GFX_CODE
#endif // SM_DEBUG_CODE

  // pick shell parameters
  double dOffsetRadius = smos_Fabs( dOffsetDistance );
  double dTolerance = pBrep->GetTolerance();

  // locals
  SmBrep *pOffsetBrep = NULL;
  SmBrep *pOffset_Brep = NULL;

  {
    // construct the Offset Executive for offset into a New Brep
    SmOffsetGeometryCreation sOGC( dOffsetRadius, dTolerance );
    SmOffsetExecutive        sExec( crContext, SM_OO_SOLID_OFFSET, sOGC, pBrep );
    if(dOffsetDistance < 0.0)
    {
      sExec.SetInset( TRUE );
    }

    // set selected faces to be shelled (just copied not offset) into output
    sExec.SetShellFaces( sShellFaces1 );
    if(bExtendCorners)
    {
      sExec.SetExtendConvexEdges( TRUE ); // TRUE = extend and reintersect surfaces at convex edges- don't fillet 
      sExec.SetMergeResults( FALSE );     // FALSE= stitch results, don't merge
    }
    else
    {
      sExec.SetExtendConvexEdges( FALSE );
      sExec.SetMergeResults( TRUE );
    }

    // build the offset brep
    if(sExec.DoSolidOffset( pOffset_Brep ) != SM_SUCCESS)  // note: increments unlocked mark value
    {
        if(pOffset_Brep) { delete pOffset_Brep; pOffset_Brep = NULL; } // JLMCC hunting memory leaks
        if(pOffsetBrep) { delete pOffsetBrep; pOffsetBrep = NULL; }
        return(SM_ERR);
    }
    pOffsetBrep = sExec.GetOffsetBrep();
  }

  if(pOffsetBrep) { delete pOffsetBrep; pOffsetBrep = NULL; }

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics() && pOffset_Brep && pBrep)
  {
    SM_ASSERT_VALID( pBrep );
    SM_ASSERT_VALID( pOffset_Brep );

    smgfx_Erase();
    smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) pBrep->Draw( TRUE ); sm_GraphicsLoop();
    smgfx_SetLook( 1, 2, 0, 1, 0 ); if(pOffset_Brep) pOffset_Brep->Draw( TRUE ); sm_GraphicsLoop();
    smgfx_SetLook( 1, 2 );        if(pOffset_Brep) pOffset_Brep->Draw( TRUE ); sm_GraphicsLoop();
    sm_GraphicsLoop();
    sm_GraphicsBrepListClear(); // JLMCC hunting memory leaks
  }
#endif // SM_GFX_CODE

  pOffset_Brep->ValidatePointers();
  pOffset_Brep->Dump();

  SmObjDelete sCleanup0(pBrep);
  SmObjDelete sCleanup1(pOffset_Brep);

  // subtract input Brep from offset Brep to make a shell Brep
  SmStatus stat = SM_SUCCESS;
  if(dOffsetDistance < 0.0)
  {
    MYPRINTF( _T( "\nCalling my_test_bbu 22" ) );
    stat = my_test_bbu( pBrep, pOffset_Brep, 2, pBrep );
    if(stat == SM_SUCCESS)
      rPartBreps.Add( pBrep );
  }
  else
  {
    MYPRINTF( _T( "\nCalling my_test_bbu 23" ) );
    stat = my_test_bbu( pOffset_Brep, pBrep, 2, pBrep );
    if(stat == SM_SUCCESS)
      rPartBreps.Add( pOffset_Brep );
  }

  // check state - quit when Boolean subtract failed  
  if (stat != SM_SUCCESS)
    { return SM_ERR; }

  sCleanup0.Clear();
  sCleanup1.Clear();

  pBrep->ValidatePointers();
  pBrep->Dump();

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics() && pBrep)
  {
    SM_ASSERT_VALID( pBrep );

    smgfx_Erase();
    smgfx_SetLook( 1, 2, 0, 0, 0 );
    smgfx_ClearColor(); pBrep->Draw( TRUE ); sm_GraphicsLoop();
    sm_GraphicsLoop();
    sm_GraphicsBrepListClear(); // JLMCC hunting memory leaks
  }

#ifdef SM_DEBUG_CODE
    if(bDebugMe)
      {
        // draw and dump the Offset_Brep in detail
        if(smGet_DoGraphics() && pOffset_Brep && pBrep)
          {
            SM_DUMP_AND_ASSERT_VALID( pOffset_Brep );
            double dPtSize, dLineWidth ;
            SmTArray<SmFace*> sFaces ;      pOffset_Brep->GetFaces(sFaces) ;       ULONG lFaceCnt   = sFaces.GetSize() ;
            SmTArray<SmEdge*> sEdges ;      pOffset_Brep->GetEdges(sEdges) ;       ULONG lEdgeCnt   = sEdges.GetSize() ;
            SmTArray<SmVertex*> sVertices ; pOffset_Brep->GetVertices(sVertices) ; ULONG lVertexCnt = sVertices.GetSize() ;
          
            smgfx_Erase() ;

            /* draw input - Brep and faces not offset  */
            smgfx_SetLook(1,2, 0,0,1); pBrep->Draw( TRUE ); sm_GraphicsLoop();
            for(di=0;di<sShellFaces1.GetSize();di++)
              { smgfx_SetLook(1,2, 0,0,0 ); sShellFaces1[di]->Draw( SM_DM_CROSSHATCH ); sm_GraphicsLoop() ; }

            /* draw output - pOffset_brep, vertices, edges, and faces */
            smgfx_SetLook(1,2, 0,1,0); pOffset_Brep->Draw( TRUE ); sm_GraphicsLoop();
            for(di=0;di<lVertexCnt;di++) { dPtSize    = 4 + 10*(double)(di)/(double)(lVertexCnt-1) ; smgfx_SetLook(3,dPtSize, TRUE) ;    sVertices[di]->Draw() ; sm_GraphicsLoop() ; }
            for(di=0;di<lEdgeCnt;di++)   { dLineWidth = 2 +  8*(double)(di)/(double)(lEdgeCnt-1) ;   smgfx_SetLook(dLineWidth,2, TRUE) ; sEdges[di]->Draw() ;    sm_GraphicsLoop() ; }
            for(di=0;di<lFaceCnt;di++)   {                                                           smgfx_SetLook(1,2, TRUE) ;          sFaces[di]->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
            sm_GraphicsBrepListClear(); // JLMCC hunting memory leaks
          }
      }
#endif // SM_DEBUG_CODE
#endif // SM_GFX_CODE

  // all done
  return (SM_SUCCESS);

} // end ShellBrep

/***********************************************************************
PURPOSE --- Silent run of regression for offset

USAGE NOTES --- 
***********************************************************************/
SmStatus airfoil_offset
  (const SmContext & crContext,
   SmTArray<SmBrep*> & rPartBreps)
{
  SmAxis2Placement Ejes;
  Ejes.SetCanonical(SmVector3d(0,0,0), SmVector3d(1,0,0), SmVector3d(0,1,0));     
  SmVector3d *Ampl = new SmVector3d( 1000, 1000, 1000 );
  
  SmBrep *Airfoil = new( crContext ) SmBrep();
  Airfoil->ReadFromFile( crContext, _T("airfoil.brep"), SM_ASCII );
  
  //For changing to milimeter dimension
  SER_MSG(Airfoil->Transform( Ejes, Ampl ), _T("ReadFromFile(airfoil.brep) failed")) ;
  
  SER( Airfoil->ValidatePointers() );

  SmBoolean Result;
  double dMaxEdgeFaceTrimCurveGap, dMaxVertexEdgeGap, dMaxVertexFaceGap;
  SER(Airfoil->ValidateAndUpdateTolerances(TRUE,
                                           Result,
                                           dMaxEdgeFaceTrimCurveGap, 
                                           dMaxVertexEdgeGap, 
                                           dMaxVertexFaceGap));
  double Max_Tol =  smos_3Max(dMaxEdgeFaceTrimCurveGap, dMaxVertexEdgeGap, dMaxVertexFaceGap) ;

  if( Max_Tol > Airfoil->GetTolerance() ) {
    Airfoil->SetTolerance( Max_Tol );
  } 
  
  double dOffsetRadius=2; 
  
  SmTArray<SmFace*> sShellFaces;
  
  Airfoil->GetFaces(sShellFaces);

  rPartBreps.Add(Airfoil);

  SmFace *pShellFace1 = sShellFaces[0];
  SmFace *pShellFace2 = sShellFaces[1];
  
  sShellFaces.Add(pShellFace1);
  sShellFaces.Add(pShellFace2);
  
  {
    double dTolerance = 100.0 * Airfoil->GetTolerance();;
    SmOffsetGeometryCreation sOGC( dOffsetRadius, dTolerance);
    SmOffsetExecutive sExec(crContext,SM_OO_SOLID_OFFSET,sOGC,Airfoil);
    sExec.SetInset(FALSE);
   
    SmBrep *pOffset_Brep;

    SER(sExec.DoSolidOffset(pOffset_Brep));  // note: increments unlocked mark value
  
  //  system( "rm ..\\prog_test\\OutputFiles\\OffSetAirfoil.brep" );  
  //  pOffset_Brep->WriteToFile( "..\\prog_test\\OutputFiles\\OffSetAirfoil.brep" );    

    rPartBreps.Add(pOffset_Brep);
  }
  return SM_SUCCESS;

} // end airfoil_offset


/***********************************************************************
PURPOSE --- Silent run of regression for offset

USAGE NOTES --- 
***********************************************************************/
SmStatus my_shell_demo
  (const SmContext & crContext,
   SmTArray<SmBrep*> & rPartBreps,
   ULONG & lCount) 

{
  MYPRINTF(_T("  In my_shell_demo()")) ;

  lCount++;

#if 0
  if(lCount == 0)
  {
    DELETE_ALL_PARTS2( rPartBreps );
    SmTArray<SmFace*> sShellFaces;
    SmBrep *pBrep;
    pBrep = new(crContext) SmBrep();
    MYPRINTF( _T( "\nReading test file ShellBrep_471.smb" ) );
    pBrep->ReadFromFile( crContext, _T( "ShellBrep_471.smb" ), SM_ASCII );
    SmBrep *pOffset;
    SmBoolean bNC;
    pBrep->OrientTrimmedSurfaces( TRUE, bNC, FALSE );  // note: increments unlocked mark value
    SmOffsetExecutive::ShellBrep( crContext, pBrep, 10.0, TRUE, FALSE, TRUE, sShellFaces, pOffset );
    SmOffsetExecutive::ReplaceImplicitOffsets( pOffset, pOffset->GetTolerance() );
    if(pOffset)
    {
      pOffset->Dump();
      pOffset->ValidateCounts( 12, 24, 14, 0, 0, 24, 2, 2 );
    }
    rPartBreps.Add( pOffset );
    return SM_SUCCESS;
  }
#endif

  //
  // Problem: lCount == 1, 3, 4 (my_shell_demo iter: 0, 2, 3):
  // The fix for Bug 359 (SmOffsetExecutive.cpp, 2/22/2015) cleaned these up,
  // and resulted in fewer and much smaller tolerance/gap errors, but it also
  // caused these three tests to return 26 edges instead of 24.  24 is correct.
  // The fix is also correct, and this new behavior is a bug, that was uncovered
  // by the fix.  In the Boolean Difference that is done to create the shell,
  // surface intersections are not connecting up.  I believe the problem is in
  // either offsetting surfaces, or extending them.
  // I'm going to leave the error messages in, because it is indeed incorrect.
  //

  if (lCount == 1) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(1);
      MYPRINTF(_T("\nEntered ShellBrep 2"));
      ShellBrep(crContext,_T("../../TestFiles/pt_TestFiles/Shell/problemShell.smb"), -2.54,TRUE,FALSE,sShellFaces,rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(11,24,16,0,0,24,2,2);  // See note above.
                  }
      return SM_SUCCESS;
    }

  if (lCount == 2) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(4);
      MYPRINTF(_T("\nEntered ShellBrep 3"));
      ShellBrep(crContext,_T("../../TestFiles/pt_TestFiles/Shell/problemShell1.smb"),-4.58,TRUE,FALSE,sShellFaces,rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(11,24,16,0,0,24,2,2);
                  }
      return SM_SUCCESS;
    }

  if (lCount == 3) 
    { 
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(1);
      MYPRINTF(_T("\nEntered ShellBrep 4"));
      ShellBrep(crContext,_T("../../TestFiles/pt_TestFiles/Shell/problemShell2.smb"), -12.7,TRUE,FALSE,sShellFaces,rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(11,24,16,0,0,24,2,2);  // See note above.
                  }
      return SM_SUCCESS;
    }

  if (lCount == 4) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(2);
      MYPRINTF(_T("\nEntered ShellBrep 5"));
      ShellBrep(crContext,_T("../../TestFiles/pt_TestFiles/Shell/problemShell21.smb"),-12.7,TRUE,FALSE,sShellFaces,rPartBreps);
       
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(11,24,16,0,0,24,2,2);  // See note above.
                  }
      return SM_SUCCESS;
    }

  if (lCount == 5) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(0);
      MYPRINTF(_T("\nEntered ShellBrep 6"));
      ShellBrep(crContext,_T("../../TestFiles/pt_TestFiles/Shell/problemShell3.smb"), -5.08,TRUE,FALSE,sShellFaces,rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(11,24,16,0,0,24,2,2);
                  }
      return SM_SUCCESS;
    }

  if (lCount == 6) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(2);
      MYPRINTF(_T("\nEntered ShellBrep 7a"));
      ShellBrep(crContext,_T("../../TestFiles/pt_TestFiles/Shell/problemShell4.smb"), 5.08,TRUE,FALSE,sShellFaces,rPartBreps);
        
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(9999,7,4,0,0,4,2,2);  // Possible bug here
                  }
      return SM_SUCCESS;
    }

  if (lCount == 7) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(5);

      MYPRINTF(_T("\nEntered ShellBrep 7b"));
      ShellBrep(crContext,_T("../../TestFiles/pt_TestFiles/Shell/problemShell5.smb"),-5.08,TRUE,FALSE,sShellFaces,rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(11,24,16,0,0,24,2,2);
                  }
      return SM_SUCCESS;
    }

  if (lCount == 8) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(5);
      MYPRINTF(_T("\nEntered ShellBrep 8"));
      ShellBrep(crContext,_T("../../TestFiles/pt_TestFiles/Shell/problemShell6.smb"), -2.54,TRUE,FALSE,sShellFaces,rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(11,24,16,0,0,24,2,2);
                  }
      return SM_SUCCESS;
    }

  if (lCount == 9) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(5);
      MYPRINTF(_T("\nEntered ShellBrep 9"));
      ShellBrep(crContext,_T("../../TestFiles/pt_TestFiles/Shell/problemShell10.smb"),-12.7,TRUE,FALSE,sShellFaces,rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(11,24,16,0,0,24,2,2);
                  }
      return SM_SUCCESS;
    }

  if (lCount == 10) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(1);

      MYPRINTF(_T("\nEntered ShellBrep 10"));
      ShellBrep(crContext,_T("../../TestFiles/pt_TestFiles/Shell/problemShell16.smb"),-12.7,TRUE,FALSE,sShellFaces,rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(11,24,16,0,0,24,2,2);
                  }
      return SM_SUCCESS;
    }

  if (lCount == 11) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(1);

      MYPRINTF(_T("\nEntered ShellBrep 11a"));
      ShellBrep(crContext,_T("../../TestFiles/pt_TestFiles/Shell/problemShell10.smb"), -12.7,TRUE,FALSE,sShellFaces,rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(11,24,16,0,0,24,2,2);
                  }
      return SM_SUCCESS;
    }

#if 0
  if (lCount == 12) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<SmFace*> sShellFaces;
      SmBrep *pBrep;  
      pBrep = new( crContext ) SmBrep();

      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Shell/BoxVRFillet.smb"));
      pBrep->ReadFromFile( crContext,_T("../../TestFiles/pt_TestFiles/Shell/BoxVRFillet.smb"), SM_ASCII );
         SmBrep *pOffset;
      SmOffsetExecutive::ShellBrep(crContext,pBrep,
             10.0,TRUE,FALSE,FALSE,sShellFaces,pOffset);
      SmOffsetExecutive::ReplaceImplicitOffsets(pOffset,pOffset->GetTolerance());
      if(pOffset) { pOffset->Dump() ;
                    pOffset->ValidateCounts(14,30,20,0,0,30,3,4);
                  }
      rPartBreps.Add(pOffset);
      return SM_SUCCESS;
    }
#endif 
  
#if 0
  if (lCount == 12) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(0);
      MYPRINTF(_T("\nEntered ShellBrep 13"));
      ShellBrep(crContext,"../../TestFiles/pt_TestFiles/Shell/problemShell15.smb",-6.858,TRUE,FALSE,sShellFaces,rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(10,28,22,0,0,28,2,2);
                  }
      return SM_SUCCESS;
    }

  if (lCount == 13) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(4);
      MYPRINTF(_T("\nEntered ShellBrep 14"));
      ShellBrep(crContext,"../../TestFiles/pt_TestFiles/Shell/problemShell18.smb",-6.35,TRUE,FALSE,sShellFaces,rPartBreps);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      if(pOffset) { pOffset->ValidateCounts(11,24,16,0,0,24,2,2);
                  }
      return SM_SUCCESS;
    }

  if (lCount == 14) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sFaces;
      sFaces.Add(5);
      MYPRINTF(_T("\nEntered ShellBrep 15"));
      ShellBrep(crContext,"../../TestFiles/pt_TestFiles/Shell/ShellBrep_983.smb",1.0,TRUE,FALSE,sFaces,rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      SER(SmOffsetExecutive::ReplaceImplicitOffsets(pOffset,pOffset->GetTolerance()));
      if(pOffset) { pOffset->ValidateCounts(167,364,199,0,0,364,2,2);
                  }
    }

  if (0 && lCount == 15) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<SmFace*> sShellFaces;
      SmBrep *pBrep;  
      pBrep = new( crContext ) SmBrep();
      MYPRINTF("\nReading test file ../../TestFiles/pt_TestFiles/Shell/OpenBox.smb");
      pBrep->ReadFromFile( crContext,"../../TestFiles/pt_TestFiles/Shell/OpenBox.smb", SM_ASCII );
         SmBrep *pOffset;
      
         MYPRINTF(_T("\nEntered ShellBrep 16"));
         SmOffsetExecutive::ShellBrep(crContext,pBrep, 1.0,TRUE,FALSE,TRUE,sShellFaces,pOffset);
      if(pOffset) { pOffset->Dump() ;
                    pOffset->ValidateCounts(12,24,14,0,0,24,2,2);
                  }
      rPartBreps.Add(pOffset);
      return SM_SUCCESS;
    }
#endif // 0 - skipped code

    DELETE_ALL_PARTS2(rPartBreps);
  
    return SM_SUCCESS;

} // end my_shell_demo

/***********************************************************************
PURPOSE --- Regression tests for the offsetting.  

USAGE NOTES ---  Called by my_offset_regression
***********************************************************************/
PT_EXPORT SmStatus my_offset_demo
 (const SmContext   & crContext,
  SmTArray<SmBrep*> & rPartBreps,
  ULONG             & lCount) 
{
  MYPRINTF(_T("  In my_offset_demo()")) ;

  lCount ++; 

  if (lCount == 1) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(1);
      MYPRINTF(_T("\nEntered ShellBrep 12"));
      ShellBrep(crContext,_T("../../TestFiles/pt_TestFiles/Solids/OffsetBoxCylOnCorner_edited.smb"),-2.54,FALSE,TRUE,sShellFaces,rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      // (From Nov 2012 through Aug 2016 this changed to 43 edges and 29 vertices,
      // due to an extra vertex in one of the big circular edges.)
      if(pOffset) { pOffset->ValidateCounts(17,42,28,0,0,42,2,2); }
    }

  else if (lCount == 2) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(5);
      MYPRINTF(_T("\nEntered ShellBrep 13"));
      ShellBrep(crContext,_T("../../TestFiles/pt_TestFiles/Solids/OffsetBoxWithBoss.smb"),-2.54,FALSE,TRUE,sShellFaces,rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(16,32,21,0,0,32,2,2);
                  }
    }

  else if (lCount == 3) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(0);
      MYPRINTF(_T("\nEntered ShellBrep 14"));
      ShellBrep(crContext,_T("../../TestFiles/pt_TestFiles/Solids/OffsetBoxSphOnCorner.smb"),-2.54,FALSE,TRUE,sShellFaces,rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
       if(pOffset)pOffset->ValidateCounts(26,72,49,0,0,72,2,2);
    }

  else if (lCount == 4) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      MYPRINTF(_T("\nEntered my_test_offsets 8"));
      SER(my_test_offset(crContext,_T("../../TestFiles/pt_TestFiles/Solids/off_extrusion.smb"),0,0.3,1.0e-4,rPartBreps));
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(41,83,44,0,0,83,2,2);
                  }
    }

  else if (lCount == 5) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      MYPRINTF(_T("\nEntered my_test_offsets 9"));
      SER(my_test_offset(crContext,_T("../../TestFiles/pt_TestFiles/Solids/off_extrusion.smb"),0,2.2,1.0e-4,rPartBreps));
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      #ifdef SM_GFX_CODE
            SmBoolean bDebugMe = FALSE;
            if (bDebugMe && smGet_DoGraphics())
            {
                for (ULONG ii = 0; ii < rPartBreps.GetSize(); ii++)
                {
                    smgfx_Erase();
                    smgfx_SetLook(1, 2, 0, 0, 1);
                    rPartBreps[ii]->Draw(TRUE);
                    sm_GraphicsLoop();
                }
            }
      #endif // SM_GFX_CODE
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(40,78,44,0,0,78,2,2);
                  }
    }

  else if (lCount == 6) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      MYPRINTF(_T("\nEntered my_test_offsets 10"));
      SER(my_test_offset(crContext,_T("../../TestFiles/pt_TestFiles/Solids/off_extrusion.smb"),1,-0.3,1.0e-4,rPartBreps));
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset)
        {  pOffset->ValidateCounts(15,39,26,0,0,39,2,2);
           SmBrep *pOrig = rPartBreps[0];
           MYPRINTF(_T("\nCalling my_test_bbu 24"));
           SER(my_test_bbu(pOrig,pOffset,2,pOrig));
           rPartBreps.SetSize(1);
        }
    }

  else if (lCount == 7) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      MYPRINTF(_T("\nEntered my_test_offsets 11"));
      SER(my_test_offset(crContext,_T("../../TestFiles/pt_TestFiles/Solids/off_extrusion.smb"),0,1.0,1.0e-4,rPartBreps));
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(37,71,38,0,0,71,2,2);
                  }
    }

  else if (lCount == 8)  // Note: this fails under LINUX but 'works' in VisC++
    {
      DELETE_ALL_PARTS2(rPartBreps);
      MYPRINTF(_T("\nEntered my_test_offsets 12"));
      SER(my_test_offset(crContext,_T("../../TestFiles/pt_TestFiles/Solids/boolean_trees_result.smb"),0,0.3,1.0e-4,rPartBreps));
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(39,83,48,0,0,83,2,2);
                  }
    }

  else if (lCount == 9) // 1st bracket2 run: radius = 0.5
    {
      DELETE_ALL_PARTS2(rPartBreps);
      MYPRINTF(_T("\nEntered my_test_offsets 13"));
      SER(my_test_offset(crContext,_T("../../TestFiles/pt_TestFiles/Solids/offset_bracket2.smb"),0,0.5,SM_ZONE_TOL_3D,rPartBreps));
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(176,361,192,0,0,361,2,2);
                  }
    }

  else if (lCount == 10) // 2nd bracket2 run: radius = 1.5
    {
      DELETE_ALL_PARTS2(rPartBreps);
      MYPRINTF(_T("\nEntered my_test_offsets 14"));
      SER(my_test_offset(crContext,_T("../../TestFiles/pt_TestFiles/Solids/offset_bracket2.smb"),0,1.5,SM_ZONE_TOL_3D,rPartBreps));
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(167,364,199,0,0,364,2,2);
                  }
    }

  else if (lCount == 11) // 3rd bracket2 run: radius = 2.5
    {
      DELETE_ALL_PARTS2(rPartBreps);
      MYPRINTF(_T("\nEntered my_test_offsets 15"));
      SER(my_test_offset(crContext,_T("../../TestFiles/pt_TestFiles/Solids/offset_bracket2.smb"),0,2.5,SM_ZONE_TOL_3D,rPartBreps));
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
       if(pOffset) { pOffset->ValidateCounts(157,357,200,0,0,357,2,2);
                   }
    }

  else if (lCount == 12) // 4th bracket2 run: radius = 3.5
    {
      DELETE_ALL_PARTS2(rPartBreps);
      MYPRINTF(_T("\nEntered my_test_offsets 16"));
      SER(my_test_offset(crContext,_T("../../TestFiles/pt_TestFiles/Solids/offset_bracket2.smb"),0,3.5,SM_ZONE_TOL_3D,rPartBreps));
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
       if(pOffset) { pOffset->ValidateCounts(151,348,198,0,0,348,2,2);
                   }
    }

  else if (lCount == 13) // 5th bracket2 run: radius = 5.1
    {
      DELETE_ALL_PARTS2(rPartBreps);
      MYPRINTF(_T("\nEntered my_test_offsets 17"));
      SER(my_test_offset(crContext,_T("../../TestFiles/pt_TestFiles/Solids/offset_bracket2.smb"),0,5.1,SM_ZONE_TOL_3D,rPartBreps));
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
       if(pOffset) { pOffset->ValidateCounts(130,305,178,0,0,305,2,2);
                   }
    }

  else if (lCount == 14) // 6th bracket2 run: radius = 7.1 
    { // This one has a bug produces 6 regions
      DELETE_ALL_PARTS2(rPartBreps);
      MYPRINTF(_T("\nEntered my_test_offsets 18"));
      SER(my_test_offset(crContext,_T("../../TestFiles/pt_TestFiles/Solids/offset_bracket2.smb"),0,7.1,SM_ZONE_TOL_3D,rPartBreps));
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
       if(pOffset) { pOffset->ValidateCounts(124,292,170,0,0,292,2,2);
                   }
    }

  else if (lCount == 15) // 6th bracket2 run: offset at radius = 1.5, merge with Offset_bracket_box
                         //                   offset at radius = 2.6, merge with Offset_bracket_box
    {
      DELETE_ALL_PARTS2(rPartBreps);
      MYPRINTF(_T("\nEntered my_test_offsets 19"));
      SER(my_test_offset(crContext,_T("../../TestFiles/pt_TestFiles/Solids/offset_bracket2.smb"),0,1.5,SM_ZONE_TOL_3D,rPartBreps));
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();

      SmBrep* pCutter = new(crContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/offset_bracket_box.smb"));
      pCutter->ReadFromFile( crContext, _T("../../TestFiles/pt_TestFiles/Solids/offset_bracket_box.smb"), SM_ASCII );

      MYPRINTF(_T("\nCalling my_test_bbu 25"));
      SER(my_test_bbu(pOffset,pCutter,2,pOffset));      

      rPartBreps.SetSize(0);
      MYPRINTF(_T("\nEntered my_test_offsets 20"));
      SER(my_test_offset(crContext,_T("../../TestFiles/pt_TestFiles/Solids/offset_bracket2.smb"),0,2.6,SM_ZONE_TOL_3D,rPartBreps));
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset2 = rPartBreps.GetLast();

      SmBrep* pCutter2 = new(crContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/offset_bracket_box.smb"));
      pCutter2->ReadFromFile( crContext, _T("../../TestFiles/pt_TestFiles/Solids/offset_bracket_box.smb"), SM_ASCII );
  
      MYPRINTF(_T("\nCalling my_test_bbu 26"));
      SER(my_test_bbu(pOffset2,pCutter2,2,pOffset2));      

      rPartBreps.SetSize(2);
      rPartBreps[0] = pOffset;
      rPartBreps[1] = pOffset2;
      if(pOffset) { pOffset->Dump() ;
                    pOffset->ValidateCounts(126,302,178,0,0,302,2,2);
                  }
      if(pOffset2) { pOffset2->Dump() ;
                     pOffset2->ValidateCounts(117,297,180,0,0,297,2,2);
                   }
    }

  else if (lCount == 16) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(0);
      MYPRINTF(_T("\nEntered ShellBrep 15"));
      ShellBrep(crContext,
                _T("../../TestFiles/pt_TestFiles/Solids/InsetCone.smb"),
                -12.7,
                FALSE, 
                TRUE,
                sShellFaces,
                rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
      if(pOffset) { pOffset->ValidateCounts(3,4,4,0,0,4,2,2);
                  }
    }

  else if (lCount == 17) 
    {
      MYPRINTF(_T("\nNote, the following test results in a zero-area Loop."));
      DELETE_ALL_PARTS2(rPartBreps);
      SmTArray<ULONG> sShellFaces;
      sShellFaces.Add(2);
      MYPRINTF(_T("\nEntered ShellBrep 16"));
      ShellBrep(crContext,_T("../../TestFiles/pt_TestFiles/Solids/InsetTaperedExtrude.smb"),
          -12.7,TRUE,TRUE,sShellFaces,rPartBreps);
      if(rPartBreps.GetSize() < 1) return(SM_ERR);
      SmBrep *pOffset = rPartBreps.GetLast();
       if(pOffset) { pOffset->ValidateCounts(11,24,16,0,0,24,2,2);
                   }
    }

  else if (lCount == 18) 
    {
      DELETE_ALL_PARTS2(rPartBreps);
      airfoil_offset(crContext,rPartBreps);
      lCount = 0;
    }

  DELETE_ALL_PARTS2(rPartBreps);

  return SM_SUCCESS;

} // end my_offset_demo




