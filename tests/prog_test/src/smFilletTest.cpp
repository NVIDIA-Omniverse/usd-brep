// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smfillet_test.cpp
* PURPOSE --- Source code file for testing the filleting
*
**********************************************************************/
/*___*/

#include "StdAfx.h"
#include <SmTypes.h>
#include <smFilletTest.h>
#include <SmBrepData.h>
#include <SmFilletExecutive.h>
#include <SmFilletStandardSolver.h>

SmStatus my_test_surface_fillet(const SmContext & crContext,
    SmSurface *pSurface1,
    SmSurface *pSurface2,
    double dFilletRadius1,
    double dFilletRadius2,
    double dTolerance,
    SmBoundaryTrimmingType eTrimType,
    SmBoolean bReverseTrim,
    ULONG lCrossSectionType,
    double dCrossSectionAccuracy,
    SmBoolean bMirror,
    SmBoolean bComplement,
    SmBrep *& rpResult);


#define DELETE_ALL_PARTS(parts) { \
    for (ULONG z=0; z<(parts).GetSize(); z++) \
     { SM_ASSERT((parts)[z] != NULL) ; delete (parts)[z]; (parts)[z] = NULL ; } \
    (parts).ReSet(); }  

/*******************************************************
PURPOSE --- 

USAGE NOTES ---
********************************************************/
static SmStatus my_test_two_brep_fillet
  (const SmContext & crContext,
   SmBrep *pBrep,
   SmBrep *pBrep2,
   double dFilletRadius1,
   double dFilletRadius2,
   double dTolerance,
   SmBoundaryTrimmingType eTrimType,
   SmBrep *& rpResult);


/*******************************************************
PURPOSE --- 

USAGE NOTES ---
********************************************************/
static SmStatus my_test_one_surface_fillet
  (const SmContext & crContext,
   SmBrep* pBrep1,
   SmBrep* pBrep2,
   double dFilletRadius,
   double dTolerance,
   SmBoundaryTrimmingType eTrimType,
   SmTArray<SmBrep*> & rPartBreps)
{
    DELETE_ALL_PARTS(rPartBreps);

    SmBrep *pResult;
    SER(my_test_two_brep_fillet(
        crContext, pBrep1, pBrep2, dFilletRadius,
        dFilletRadius,dTolerance,
        eTrimType,pResult));

    rPartBreps.Add(pBrep1);
    rPartBreps.Add(pBrep2);
    
    if (pResult != NULL) {
        rPartBreps.Add(pResult);
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0,0,1);
            pResult->Draw();
            sm_GraphicsLoop();
        }
#endif
    }
    
    return SM_SUCCESS;

} // end my_test_one_surface_fillet

/*******************************************************
PURPOSE --- 

USAGE NOTES ---
********************************************************/
static SmStatus my_test_two_surface_fillet
  (const SmContext & crContext,
   const TCHAR * pFileName,
   const TCHAR * pFileName2,
   double dFilletRadius,
   double dTolerance,
   SmBoundaryTrimmingType eTrimType,
   SmTArray<SmBrep*> & rPartBreps)
{
    DELETE_ALL_PARTS(rPartBreps);
   
    SmBrep *pBrep  = new(crContext) SmBrep();
    SmBrep *pBrep2 = new(crContext) SmBrep();
    // Select breps for input
    pBrep->ReadFromFile (crContext, pFileName);     
    pBrep2->ReadFromFile(crContext, pFileName2);

    SmBrep *pResult;
    SER(my_test_two_brep_fillet(crContext,
                                pBrep,
                                pBrep2,
                                dFilletRadius,
                                dFilletRadius,
                                dTolerance,
                                eTrimType,
                                pResult));

    rPartBreps.Add(pBrep);
    rPartBreps.Add(pBrep2);
    
    if (pResult != NULL) {
        rPartBreps.Add(pResult);
        pResult->Dump();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            smgfx_Erase() ;
            smgfx_ClearColor(); pResult->Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
          }
#endif
    }
    
    return SM_SUCCESS;

} // end my_test_two_surface_fillet


// Utility function defined in smtess_test.cpp
SmStatus my_read_brep(const SmContext & crContext,
                      const TCHAR     * pFileName,
                      SmBrep         *& rpNewBrep,
                      SmBoolean         bRebuildUVTrimCurves);

/*****************************************************
PURPOSE --- Read a Brep model from file and
            Fillet the specified edges.

USAGE NOTES ---
  1. Edges to be filleted are indicated by index value where
     the index value is used in the array returned by
     SmBrep::GetEdges().  This is ok for debugging - but
     needs to be rethought in a proper application.
  2. Fillets can be
       constant radius,
       contant distance (distance from fillet edge to rail curves is constant)
       variable radius.
  3. The 3 differnt kinds of fillets can have
       linear or circular cross-sections.
  4. Corners can be beveled or not.
*****************************************************/
SmStatus my_test_model
  (const SmContext   & crContext,             // in : context for new object construction
   const TCHAR       * pFileName,             // in : target file to test
   double              dThisApproxTol3d,      // NotUsed: in : Not currently used - used to set max distance for coincident objects for Stitch
   double              dBallRadius,           // in : size of fillets, see eSolverType
   ULONG             * pIndexArr,             // in : a list of index intervals where every pair of entries
                                              //      indicate a sequence of edge indices.
                                              //      ex:[0,2,9,12] = indices:[0,1,2,9,10,11,12]
                                              //      sized:[lNumIntervals*2]
   ULONG               lNumIntervals,         // in : number of intervals in pIndexArr
   SmBoolean           bIsIncludeArr,         // in : TRUE = Only fillet edges specified by pIndexArr 
                                              //      FALSE= Fillet all edges but those specified by pIndexArr
   SmFilletSolverType  eSolverType,           // in : oneof: SM_FS_CONST_RADIUS    - create constant radius  =dBallRadius fillets
                                              //             SM_FS_CONST_DIST      - create constant distance=dBallRadius fillets
                                              //             SM_FS_VARIABLE_RADIUS - create variable radius(vary about dBallRadius size) fillets
   SmTArray<SmBrep*> & rPartBreps,            // out: loaded with after filleting Brep result
   SmBoolean           bRebuildUVTrimCurves,  // in   TRUE = call SmBrep::bRebuildUVTrimCurves within my_read_brep
   SmBoolean           bDoGlobalMerge,        // in : TRUE = utilize Global Merge to insert the fillet Brep into the 
                                              //             original Brep.  This is slower than the local operations
                                              //             (i.e when FALSE) but will enable the processing of many
                                              //             large radius cases where the fillet cuts off things.
                                              //      FALSE= Use local merge instead.
   SmBoolean           bMakeChamfer,          // in : TRUE = make linear cross-section fillets (Chamfers)
                                              //      FALSE= make cicular cross-section fillets
   SmBoolean           bMakeBevelCorner)      // in : TRUE = make all fillet corners 'bevel'
                                              //      FALSE= don't
   
{
  SM_REF1(dThisApproxTol3d) ;
  MYPRINTF(_T("\nEntered my_test_model")) ;

  // init output
  DELETE_ALL_PARTS(rPartBreps);

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics()) { smgfx_Erase() ; }
#endif

  // read Brep - stream Brep summary, optionally draw read Brep
  SmBrep *pNewBrep = NULL ;
  SER(my_read_brep(crContext,pFileName,pNewBrep,bRebuildUVTrimCurves)) ;

  SmStatus eStat = pNewBrep->ValidatePointers();
  if ( eStat != SM_SUCCESS )
    {
      SM_ASSERT_MSG( FALSE, _T("Fillet test: Brep fails ValidatePointers() upon read-in.") );
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pNewBrep) ;

      if(smGet_DoGraphics())
        {
          ULONG di ;
          SmTArray<SmSurface *> sNewSurfs ;
          pNewBrep->GetSurfaces(sNewSurfs) ;

          smgfx_Erase();
          smgfx_SetLook( 1,2, 0,0,1 ); if(pNewBrep) pNewBrep->Draw(TRUE); sm_GraphicsLoop();
          for(di=0;di<sNewSurfs.GetSize();di++)
            { if(sNewSurfs[di]) 
              { SM_ASSERT_VALID(sNewSurfs[di]) ;
                smgfx_SetLook( 1,2, 0,1,1 ); sNewSurfs[di]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
                if(sNewSurfs[di]->GetOwner()) 
                  { smgfx_SetLook(1,2, 0,0,0) ; ((SmFace *)sNewSurfs[di]->GetOwner())->Draw(SM_DM_CROSSHATCH,9,9) ; sm_GraphicsLoop() ; }
            } }
          sm_GraphicsLoop();
        }

      // GWC_EitherMoveOrRemoveFollowingLineAfterTestIsDone ;
      pNewBrep->ShrinkGeometry() ;

      SM_ASSERT_VALID_NO_STREAM(pNewBrep) ;
      if(smGet_DoGraphics())
        {
          ULONG di ;
          SmTArray<SmSurface *> sNewSurfs ;
          pNewBrep->GetSurfaces(sNewSurfs) ;

          smgfx_Erase();
          smgfx_SetLook( 1,2, 0,0,1 ); if(pNewBrep) pNewBrep->Draw(TRUE); sm_GraphicsLoop();
          for(di=0;di<sNewSurfs.GetSize();di++)
            { if(sNewSurfs[di]) 
              { SM_ASSERT_VALID(sNewSurfs[di]) ;
                smgfx_SetLook( 1,2, 0,1,1 ); sNewSurfs[di]->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
                if(sNewSurfs[di]->GetOwner()) 
                  { smgfx_SetLook(1,2, 0,0,0) ; ((SmFace *)sNewSurfs[di]->GetOwner())->Draw(SM_DM_CROSSHATCH,9,9) ; sm_GraphicsLoop() ; }
            } }
          sm_GraphicsLoop();
        }
    }
#endif // SM_DEBUG_CODE

  // was: double dTol = pNewBrep->GetTolerance();
  SmApproxTol3d sApproxTol3d = SmTol::GetApproxTol3d(pNewBrep) ;

  // Creates circular cross section fillet when used in SmFilletSolver::SetFilletSurfaceGenerator
  SmCircularCrossSectionFSG sFSGCircular(FALSE);

  // Creates Linear cross section fillet (chamfering) when used in SmFilletSolver::SetFilletSurfaceGenerator
  SmLinearCrossSectionFSG sFSGLinear;

  // Define self-intersection handler for edge fillets
  double dStepBackFactor = 1.0;
  SmMakeSurfaceBlendSIH sSIH(dStepBackFactor);

  // Create temporary SmFilletExecutive, a derived SmMerge object that organizes the filleting process
  SmFilletExecutive * pFilExec = new (crContext) SmFilletExecutive(crContext,pNewBrep);
  SmObjDelete sClean(pFilExec);

  // setup FilletExecutive state parameters
  pFilExec->SetSelfIntersectionHandler(&sSIH);

  // when asked ask remember to execute GlobalMerge
  if(bDoGlobalMerge) 
    { pFilExec->SetDoGlobalMerge(TRUE); }
                      
  // get all Brep edges
  SmTArray<SmEdge*> sEdges;
  pNewBrep->GetEdges(sEdges);

  // set up variable radius blend profile for variable-radius tests
  double adKData[2];
  SmPoint3d sData[4];
  ULONG alKMData[2];
  SmTArray<SmPoint3d> sCntrlPoly(4,sData);
  SmTArray<ULONG>     sKnotMult(2,alKMData,2);
  SmTArray<double>    sKnots(2,adKData,2);       // set to fillet edge interval when used
  sKnotMult[0] = 4; 
  sKnotMult[1] = 4;
  sCntrlPoly.Add(SmPoint3d(dBallRadius,0.0,0.0));
  sCntrlPoly.Add(SmPoint3d(dBallRadius*1.2,0.0,0.0));
  sCntrlPoly.Add(SmPoint3d(dBallRadius*0.8,0.0,0.0));
  sCntrlPoly.Add(SmPoint3d(dBallRadius,0.0,0.0));

  // create a temporary array for fillet-radius functions 
  SmTArray<SmFilletLaw*> sLaws;
  SmObjsDelete<SmFilletLaw*> sCleanLaws(&sLaws);

  // for every edge
  //  - skip edges that are not to be filleted
  //  - pick a pair of edgeuses bounding sector to be filleted
  //  - build a SmFilletSolver for this edge sector of proper derived type for given eSolverType value 
  //      with appropriate cross-section FilletSurfaceGenerator for given bMakeChamfer value
  //  - Load each FilletSolver into FilletExec 
  //      accumulating list of vertices to be filleted in m_vFilletedVertices
  for (ULONG i=0; i<sEdges.GetSize(); i++) 
    {
      // determine if this edge index is specified in the input pIndexArr
      SmBoolean bInIndexArr = FALSE;
      for(ULONG jj=0; jj<lNumIntervals; jj++)
        {
          if (   pIndexArr[jj*2] <= i 
              && i <= pIndexArr[jj*2+1]) 
            {
              bInIndexArr = TRUE;
              break;
            }
        }

      // skip edges not being filleted
      if(   ( bIsIncludeArr && !bInIndexArr)
         || (!bIsIncludeArr &&  bInIndexArr)) 
        { continue ; }

      // get edge locals - select an edge sector to be filleted
      SmEdge    * pE  = sEdges[i];
      SmEdgeuse * pEU = pE->GetBlendEdgeuse();

      // skip edges which can't be filleted
      if (pEU == NULL) 
        { continue; }

#ifdef SM_GFX_CODE
      // add edges to be filleted to current rendering
      if (smGet_DoGraphics())
        {
          smgfx_SetLook(3,4, 1,0,0); pE->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      // fillet control locals
      SmFilletSolver * pFS       = NULL;
      SmBSplineCurve * pLawCurve = NULL;
      SmFilletLaw    * pLaw      = NULL;
      SmExtent1d       sIvl      = pE->GetInterval();

      SmObjDelete sCleanupFS(pFS);
      SmObjDelete sCleanupLawCurve(pLawCurve);

      // switch on eSolverType to select SmFilletSolver type to construct
      switch (eSolverType) 
        {
          case SM_FS_CONST_RADIUS:
              pFS = new(crContext) SmConstantRadiusFS(crContext,
                                                      sApproxTol3d,
                                                      30.0*SM_PI/180.0,
                                                      2.0*SM_PI/180.0,
                                                      dBallRadius,
                                                      pEU);
              break;
          case SM_FS_CONST_DIST:
              pFS = new(crContext) SmConstantDistanceFS(crContext,
                                                        sApproxTol3d,
                                                        30.0*SM_PI/180.0,
                                                        2.0*SM_PI/180.0,
                                                        dBallRadius*smos_Sqrt(2.0),
                                                        pEU);
              break;
          case SM_FS_VARIABLE_RADIUS:
              sKnots[0] = sIvl.GetMin(); sKnots[1] = sIvl.GetMax();
              SER(SmBSplineCurve::CreateCanonical(crContext,
                                                  3,
                                                  3,
                                                  sCntrlPoly,
                                                  SM_CF_UNSPECIFIED,
                                                  sKnotMult,
                                                  sKnots,
                                                  SM_KT_UNSPECIFIED,
                                                  NULL,
                                                  NULL,
                                                  pLawCurve));
              pLaw = new(crContext) SmBSplineFilletLaw(pLawCurve,pE->GetInterval());
              sLaws.Add(pLaw);
              pFS = new(crContext) SmVariableRadiusFS(crContext,
                                                      sApproxTol3d,
                                                      30.0*SM_PI/180.0,
                                                      2.0*SM_PI/180.0,
                                                      1.0,
                                                      pEU,
                                                      *pLaw,
                                                      FALSE);
              break;
          default:
              SER(SM_ERR);
        } // end switch on eSolverType

#ifdef SM_DEBUG_CODE
      if ( (bDebugMe || smGet_DoGraphics()) && eSolverType == SM_FS_VARIABLE_RADIUS)
        {
          smgfx_SetLook(3,4, 1,0,1) ; if(pLawCurve) pLawCurve->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // set the SmFilletSolver cross-section style
      if (bMakeChamfer) { pFS->SetFilletSurfaceGenerator(&sFSGLinear);
                        }
      else              { pFS->SetFilletSurfaceGenerator(&sFSGCircular);            
                        }
      
      // Add this FilletSolver to pFilExec->m_vFilletSolvers list of FilletSolvers
      //   1. add each G1 continuity point in the filletEdge->Curve to the pFS->m_vG1Knots array
      //   2. add SmFilletSolver, pFS, to m_vFilletSolvers array
      //   3. Set pFS backPointer to this SmFilletExecutive
      //   4. for both filletEdge->Vertices and filletSector edgeuses
      //          - make sure vertex is listed in m_vFilletedVertices
      //          - make sure each edgeuse is listed in the vertices list
      //              of filleted edges stored in m_vVertexCornerMap
      pFilExec->LoadFilletSolver(pFS);

      sCleanupFS.Clear();
      sCleanupLawCurve.Clear();

    } // end iter every edge - adding a FilletSolver to pFilExec list of FilletSolvers for each edge to fillet

  // allocate an appropriate derived type SmFilletCorner object 
  // for each vertex on the m_vFilletedVertices array 
  SER(pFilExec->CreateFilletCorners());

  // when asked - Make Bevel Corners
  if (/*bMakeChamfer ||*/ bMakeBevelCorner) 
    {
      SmTArray<SmFilletCorner*> & rCorners = pFilExec->GetFilletCorners();
      for (ULONG j=0; j<rCorners.GetSize(); j++) 
        {
          rCorners[j]->SetBevel(TRUE);
        }
    }

  // done with setup - do the filleting
  eStat = pFilExec->DoFilleting();  // note: increments unlocked mark value
  if ( eStat != SM_SUCCESS )
    {
      pFilExec->GetFilletErrorInfo()->Report();
      if(pNewBrep) { delete pNewBrep; pNewBrep = NULL;}
      SER( eStat );
    }

  // dump the filleted Brep
  SM_DUMP_AND_ASSERT_VALID(pNewBrep) ;

#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
    {
      smgfx_Erase() ;
      smgfx_ClearColor(); pNewBrep->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE

  // set output
  rPartBreps.Add(pNewBrep);

  // all done
  return SM_SUCCESS;

} // end my_test_model

/*******************************************************
PURPOSE --- 

USAGE NOTES ---
********************************************************/
static SmStatus my_goto_next_line
  (FILE *pFile)
{
    ULONG lCount = 0;
    signed char sCh;
    while ((sCh = (char)fgetc(pFile)) != '\n') {
        lCount ++;
        if (lCount > 1000) {
            ERR_MSG(_T("Corrupt file - No Carriage return in 1000 characters\n"));
            SER(SM_ERR);
        }
        if (sCh == EOF) {
            ERR_MSG(_T("Illegal End Of File - Corrupt file\n"));
            SER(SM_ERR);
        }
        continue;
    }
    return SM_SUCCESS;

} // end my_goto_next_line

/*******************************************************
PURPOSE --- 

USAGE NOTES ---
********************************************************/
static FILE * my_goto_next_line_null
  (FILE *pFile)
{
    ULONG lCount = 0;
    signed char sCh;
    while ((sCh = (char)fgetc(pFile)) != '\n') {
        lCount ++;
        if (lCount > 1000) {
            ERR_MSG("Corrupt file - No Carriage return in 1000 characters\n");
            return NULL;
        }
        if (sCh == EOF) {
            ERR_MSG("Illegal End Of File - Corrupt file\n");
            return NULL;
        }
        continue;
    }
    return(pFile);

} // end my_goto_next_line_null



#define MAX_ARR_SIZE   20
#define INCLUDE_ARRAY  TRUE
#define EXCLUDE_ARRAY  FALSE


/*******************************************************
PURPOSE --- Test suite #0
  run my_test_model to fillet edges from Breps in files
     "../../TestFiles/pt_TestFiles/Fillet/cyl_cyl_219.smb",
     "../../TestFiles/pt_TestFiles/Fillet/cyl_cyl_2191.smb",
     "../../TestFiles/pt_TestFiles/Fillet/box_on_box.smb",
     "../../TestFiles/pt_TestFiles/Fillet/brick.smb",
     "../../TestFiles/pt_TestFiles/Fillet/l_shaped_box.smb",
     "../../TestFiles/pt_TestFiles/Fillet/l_shaped_box.smb",
     "../../TestFiles/pt_TestFiles/Fillet/cyl_on_box.smb",
     "../../TestFiles/pt_TestFiles/Fillet/case_int.smb",
     "../../TestFiles/pt_TestFiles/Fillet/case_ext.smb",
     "../../TestFiles/pt_TestFiles/Fillet/tilt_cyl_on_box.smb",
     "../../TestFiles/pt_TestFiles/Fillet/box1x1x2.smb",
     "../../TestFiles/pt_TestFiles/Fillet/extrusion1002.smb",
     "../../TestFiles/pt_TestFiles/Fillet/cyl_box.smb",
     "../../TestFiles/pt_TestFiles/Fillet/cyl_box_fil1.smb",
     "../../TestFiles/pt_TestFiles/Fillet/cyl_box_fil2.smb",
     "../../TestFiles/pt_TestFiles/Fillet/box1208.smb",
     "../../TestFiles/pt_TestFiles/Fillet/box1208_fil.smb",

USAGE NOTES ---
********************************************************/
SmStatus my_test_suite_0
  (const SmContext & crContext,          // in : context for new object construction
   SmTArray<SmBrep*> & rPartBreps,       // out: container for Brep Parts read from file
   ULONG & lCount,                       // i/o: target test case index, incremented with each call
   SmFilletSolverType eSolverType,       // in : oneof:
   SmBoolean & rbIsDone)                 // out: TRUE =lCount is incremented to the last test case index
                                         //      FALSE=lCount is incremented to a number less than last test case index
{
  lCount ++;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  SM_SPRINTF(sBuff,_T("\nEntered my_test_suite_0 iter: %ld"), lCount) ;
  MYPRINTF(sBuff) ;
  rPartBreps.ReSet();
    // CONSTANT RADIUS REGRESSION TESTS

  ULONG StartCount     = 1;
  ULONG lLastTestCount = 16;

  if (lCount < StartCount) 
      lCount = StartCount;

  if (lCount == 1)
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 2;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 6; aIndexArr[1] = 6; // Include i=6
      aIndexArr[2] =10; aIndexArr[3] =10; // Include i=10
      MYPRINTF(_T("\nEntered my_test_model 1"));
      SER(my_test_model(crContext, _T("../../TestFiles/pt_TestFiles/Fillet/cyl_cyl_219.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE)); // Nx1 Test
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 27);
    }

  if (lCount == 2) 
    {
      double dBallRadius = 0.4;
      ULONG lNumIntervals = 2;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 5; aIndexArr[1] = 6; // Include i=5,6
      aIndexArr[2] = 8; aIndexArr[3] = 9; // Include i=8,9
      MYPRINTF(_T("\nEntered my_test_model 2"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cyl_cyl_2191.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE)); // Nx1 Test
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 39);
    }

  if (lCount == 3) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 7; aIndexArr[1] = 7; // Include i=7
      MYPRINTF(_T("\nEntered my_test_model 3"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/box_on_box.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE)); // Nx1 Test
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 27);
    }

  if (lCount == 4) 
    {
      if (eSolverType == SM_FS_VARIABLE_RADIUS) 
        {
          return SM_SUCCESS; // Skip this test
        }
      double dBallRadius = 12.7;
      ULONG lNumIntervals = 2;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =12; aIndexArr[1] =12; // Include i=12
      aIndexArr[2] =17; aIndexArr[3] =17; // Include i=17
      MYPRINTF(_T("\nEntered my_test_model 4"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/brick.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));  // Nx1 coincidence Test
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 32);
    }

  if (lCount == 5) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =13; aIndexArr[1] =13; // Include i=13
      MYPRINTF(_T("\nEntered my_test_model 5"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/l_shaped_box.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));  // Nx1 concave Test
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 21);
    }

  if (lCount == 6) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 3; aIndexArr[1] = 3; // Include i=3
      MYPRINTF(_T("\nEntered my_test_model 6"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/l_shaped_box.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));  // Nx1 Add surface Test
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 21);
    }

  if (lCount == 7) 
    {
      double dBallRadius = 0.5;
      ULONG lNumIntervals = 2;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 4; aIndexArr[1] = 4; // Include i=4
      aIndexArr[2] =13; aIndexArr[3] =13; // Include i=13
      MYPRINTF(_T("\nEntered my_test_model 7"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cyl_on_box.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));  // Nx1 tan Test
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 19);
    }

  if (lCount == 8) 
    {
      if (eSolverType == SM_FS_VARIABLE_RADIUS) 
        {
          return SM_SUCCESS; // Skip this test
        }
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 0; aIndexArr[1] = 0; // Include i=0
      MYPRINTF(_T("\nEntered my_test_model 8"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/case_int.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));  // Nx1Closed non-tangent
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 8);
    }

  if (lCount == 9) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 0; aIndexArr[1] = 0; // Include i=0
      MYPRINTF(_T("\nEntered my_test_model 9"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/case_ext.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));  // Nx1Closed non-tangent
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 12);
    }

  if (lCount == 10) 
    {
      double dBallRadius = 10.0;
      ULONG lNumIntervals = 1; // 
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =12; aIndexArr[1] =12; // Include i=12
      MYPRINTF(_T("\nEntered my_test_model 10"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/tilt_cyl_on_box.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,SM_FS_CONST_DIST,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 17);
    }

  if (lCount == 11) 
    { // slice off corner of a rectilinear solid through a neighbor edge to make a a general quadralateral solid
      if (eSolverType == SM_FS_VARIABLE_RADIUS) 
        {
          return SM_SUCCESS; // Skip this test
        }
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 0; aIndexArr[1] = 0; // Include i=3
      SmBoolean bDoGlobalMerge = FALSE;
      SmBoolean bMakeChamfer = TRUE;

      MYPRINTF(_T("\nEntered my_test_model 11"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/box1x1x2.smb"),// Nx1 Chamfer
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        bDoGlobalMerge,bMakeChamfer,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 12);
    }

  if (lCount == 12) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 1; aIndexArr[1] = 1; // Include i=0,2
      MYPRINTF(_T("\nEntered my_test_model 12"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/extrusion1002.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        EXCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE)); // 2x1 on non-closed edge
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 13);
    }

  if (rPartBreps.GetSize() > 0)
  {
      SmBrep *pBrepToFillet = rPartBreps.GetLast();
      pBrepToFillet->Dump();
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics()) {
          smgfx_Erase(FALSE); // if not FALSE, we erase rPartBreps = UserBreps
          smgfx_ClearColor();
          pBrepToFillet->Draw(TRUE);
          sm_GraphicsLoop(); sm_GraphicsLoop();
      }
#endif 
    }  

  if (lCount >= lLastTestCount)
     { rbIsDone = TRUE; }

  return SM_SUCCESS;

} // end my_test_suite_0

/*******************************************************
PURPOSE --- Test suite #1
  run following tests 
    my_test_two_surface_fillet     "../../TestFiles/pt_TestFiles/Fillet/eg1_srf1.smb",
                                   "../../TestFiles/pt_TestFiles/Fillet/eg1_srf2.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/ext_w_hole.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/bolt.smb",
    my_test_one_surface_fillet     "../../TestFiles/pt_TestFiles/Fillet/nurb_nurb_1.smb",
    my_read_fillet_definition_file "../../TestFiles/pt_TestFiles/Drop/FilletTests/self_intersection_blend2.fdf",
    my_test_one_surface_fillet     "../../TestFiles/pt_TestFiles/Fillet/nurb_sph_1.smb",
    my_test_one_surface_fillet     "../../TestFiles/pt_TestFiles/Fillet/nurb_plane_w_notch.smb"
    my_test_model                  "../../TestFiles/pt_TestFiles/Solids/box.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/nurb_box.smb"
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/box_on_box.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/letter_g.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/box_on_box.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/cone_34.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/filleted_box.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/filleted_box.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/ext_w_hole.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/l_shaped_box.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/l_shaped_box.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/shell_fillet.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/6_sided_corner.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/cyl_sphere.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/corner_box.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Solids/box.smb", // NxN Bevel
    my_test_model                  "../../TestFiles/pt_TestFiles/Solids/pyramid.smb", // NxN Chamfer
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/torii_fillet.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/extrusion0206.smb",
    my_test_model                  "../../TestFiles/pt_TestFiles/Fillet/shell_fillet2.smb",
                                                                                                                                        
USAGE NOTES ---
********************************************************/
SmStatus my_test_suite_1
  (const SmContext & crContext,          // in : cotnext for new object construction
   SmTArray<SmBrep*> & rPartBreps,       // out: container for Brep Parts read from file
   ULONG & lCount,                       // i/o: target test case index, incremented with each call
   SmFilletSolverType eSolverType,       // in : oneof:
   SmBoolean & rbIsDone)                 // out: TRUE =lCount is incremented to the last test case index
                                         //      FALSE=lCount is incremented to a number less than last test case index
{
  lCount ++;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  SM_SPRINTF(sBuff,_T("\nEntered my_test_suite_1 iter: %ld"), lCount) ;
  MYPRINTF(sBuff) ;
    // CONSTANT RADIUS REGRESSION TESTS

    ULONG StartCount = 101;
    ULONG lLastTestCount = 128;

    if (lCount < StartCount) lCount = StartCount;

    if (lCount == 101) 
      {
        SER_DELETE_ALL_PARTS(my_test_two_surface_fillet(crContext,
            _T("../../TestFiles/pt_TestFiles/Fillet/eg1_srf1.smb"),
            _T("../../TestFiles/pt_TestFiles/Fillet/eg1_srf2.smb"),
            10.0,1.0e-4,SM_BT_MINIMAL,rPartBreps), rPartBreps);
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 4);
      }

    if (lCount == 102) 
      {
        double dBallRadius = 0.5;
        ULONG lNumIntervals = 0; // Includes all edges
        ULONG aIndexArr[MAX_ARR_SIZE];
        SmBoolean bDoGlobalMerge = FALSE;
        SmBoolean bMakeChamfer = TRUE;
        MYPRINTF(_T("\nEntered my_test_model 18"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/ext_w_hole.smb"),
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          EXCLUDE_ARRAY,eSolverType,rPartBreps, 
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          bDoGlobalMerge,bMakeChamfer, FALSE)); // NxN Chamfer
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 76);
        DELETE_ALL_PARTS(rPartBreps);
      }

    if (lCount == 103) 
      {
        double dBallRadius = 3.0;
        ULONG lNumIntervals = 0; // Include all edges
        ULONG aIndexArr[MAX_ARR_SIZE];
        MYPRINTF(_T("\nEntered my_test_model 19"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/bolt.smb"),
                          1.0e-4,dBallRadius,aIndexArr,lNumIntervals,
                          EXCLUDE_ARRAY,eSolverType,rPartBreps, 
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE));
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 76);
      }

    if (lCount == 104) 
      {
        SmBrep *pBrep1 = new(crContext) SmBrep();
        SmBrep *pBrep2 = new(crContext) SmBrep();

        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Fillet/nurb_nurb_1.smb"));
        pBrep1->ReadFromFile( crContext, _T("../../TestFiles/pt_TestFiles/Fillet/nurb_nurb_1.smb"), SM_ASCII );
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Fillet/nurb_nurb_2.smb"));
        pBrep2->ReadFromFile( crContext, _T("../../TestFiles/pt_TestFiles/Fillet/nurb_nurb_2.smb"), SM_ASCII );

        SER(my_test_one_surface_fillet(crContext, pBrep1, pBrep2,
            5.0,1.0e-4,SM_BT_BLEND,rPartBreps));

        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 4);
      }

    if (lCount == 105) 
      {
        // gwc_note: I looked at this briefly enough to see the bug is that
        //           the fillet surface is not being constructed along the
        //           fillet edges entire length.  It looks like the self
        //           intersection code is firing when it probably shouldn't 
        //           be.  To fix this step through the filletSurface
        //           construction sequence.
        SER(my_read_fillet_definition_file(crContext,_T("../../TestFiles/pt_TestFiles/Drop/FilletTests/self_intersection_blend2.fdf"),
            rPartBreps));
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 21) // USEDTOBE = 30);
      }

    if (lCount == 106) 
      {
        SmBrep *pBrep1 = new(crContext) SmBrep();
        SmBrep *pBrep2 = new(crContext) SmBrep();

        pBrep1->ReadFromFile( crContext, _T("../../TestFiles/pt_TestFiles/Fillet/nurb_sph_1.smb"), SM_ASCII );     
        pBrep2->ReadFromFile( crContext, _T("../../TestFiles/pt_TestFiles/Fillet/nurb_sph_2.smb"), SM_ASCII );

        SER(my_test_one_surface_fillet(crContext, pBrep1, pBrep2,
            1.0,1.0e-4,SM_BT_MINIMAL,rPartBreps));

        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 3);
      }

    if (lCount == 107) {
        SmBrep *pBrep1 = new(crContext) SmBrep();
        SmBrep *pBrep2 = new(crContext) SmBrep();

        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Fillet/nurb_plane_w_notch1.smb"));
        pBrep1->ReadFromFile( crContext, _T("../../TestFiles/pt_TestFiles/Fillet/nurb_plane_w_notch1.smb"), SM_ASCII );
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Fillet/nurb_plane_w_notch2.smb"));
        pBrep2->ReadFromFile( crContext, _T("../../TestFiles/pt_TestFiles/Fillet/nurb_plane_w_notch2.smb"), SM_ASCII );

        SER(my_test_one_surface_fillet(crContext, pBrep1, pBrep2,
            2.0,1.0e-4,SM_BT_MINIMAL,rPartBreps));

        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)
            rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 8);
      }

    if (lCount == 108) 
      {
      }
    if (lCount == 109) 
      {
      }
    if (lCount == 110) 
      {
        double dBallRadius = 1.0;
        ULONG lNumIntervals = 0; // Include all edges
        ULONG aIndexArr[MAX_ARR_SIZE];
        if (0) 
          {
            MYPRINTF(_T("\nEntered my_test_model 20"));
            SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Solids/box.smb"),
                              SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                              EXCLUDE_ARRAY,eSolverType,rPartBreps, 
                              FALSE, // bRebuildUVTrimCurves in my_read_brep()
                              FALSE,FALSE,FALSE)); // NxN
          }
        else 
          {
            MYPRINTF(_T("\nEntered my_test_model 21"));
            SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/nurb_box.smb"),
                              SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                              EXCLUDE_ARRAY,eSolverType,rPartBreps, 
                              FALSE, // bRebuildUVTrimCurves in my_read_brep()
                              FALSE,FALSE,FALSE)); // NxN
          }
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 48);
      }

    if (lCount == 111) 
      {
        double dBallRadius = 0.5;
        ULONG lNumIntervals = 3;
        ULONG aIndexArr[MAX_ARR_SIZE];
        //if (i<4 || (i>7 && i<16) || (i>16 && i<21)) continue;  // NxN Test
        aIndexArr[0] = 0; aIndexArr[1] = 3; // Exclude 0<=i<=3
        aIndexArr[2] = 8; aIndexArr[3] =15; // Exclude 8<=i<=15
        aIndexArr[4] =17; aIndexArr[5] =20; // Exclude 0<=i<=3
        MYPRINTF(_T("\nEntered my_test_model 22"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/box_on_box.smb"),
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          EXCLUDE_ARRAY,eSolverType,rPartBreps, 
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE));
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 52);
      }

    if (lCount == 112) 
      {
        // In this test, we have an edge with 5 G1 knots. Typically,
        // we need to split the face before filleting. However, splitting
        // did not take place here. Someday we need to fix the tracing
        // problems at those G1 knots.
        double dBallRadius = 0.5;
        ULONG lNumIntervals = 2;
        ULONG aIndexArr[MAX_ARR_SIZE];
        aIndexArr[0] =17; aIndexArr[1] =17; // Include i=17
        aIndexArr[2] =23; aIndexArr[3] =26; // Include i=23-26
        MYPRINTF(_T("\nEntered my_test_model 23"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/letter_g.smb"),
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          INCLUDE_ARRAY,eSolverType,rPartBreps,
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE));
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 45);  // was 57 now 45
      }

    if (lCount == 113) 
      {
        double dBallRadius = 0.5;
        ULONG lNumIntervals = 1;
        ULONG aIndexArr[MAX_ARR_SIZE];
        aIndexArr[0] = 4; aIndexArr[1] = 7; // Include 4<=i<=7
        MYPRINTF(_T("\nEntered my_test_model 24"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/box_on_box.smb"),
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          INCLUDE_ARRAY,eSolverType,rPartBreps, 
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE));  // Nx2 Test
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 40);
      }

    if (lCount == 114) 
      {
        double dBallRadius = 3.0;
        ULONG lNumIntervals = 3;
        ULONG aIndexArr[MAX_ARR_SIZE];
        aIndexArr[0] = 2; aIndexArr[1] = 2; // Exclude i=2
        aIndexArr[2] = 4; aIndexArr[3] = 4; // Exclude i=4
        aIndexArr[4] = 8; aIndexArr[5] = 8; // Exclude i=8
        MYPRINTF(_T("\nEntered my_test_model 25"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cone_34.smb"),
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          INCLUDE_ARRAY,eSolverType,rPartBreps, 
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE)); //Nx2
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        if (eSolverType == SM_FS_VARIABLE_RADIUS) 
          {
            SM_ASSERT(lTotalEdges == 18);
          }
        else 
          {
            SM_ASSERT(lTotalEdges == 17);
          }
      }

    if (lCount == 115) 
      {
        if (eSolverType == SM_FS_VARIABLE_RADIUS) 
          {
            return SM_SUCCESS; // Skip this test
          }
        double dBallRadius = 1.0;
        ULONG lNumIntervals = 8;
        ULONG aIndexArr[MAX_ARR_SIZE];
        aIndexArr[0] = 3; aIndexArr[1] = 3; // Include i=3
        aIndexArr[2] = 6; aIndexArr[3] = 6; // Include i=6
        aIndexArr[4] = 9; aIndexArr[5] = 9; // Include i=9
        aIndexArr[6] =12; aIndexArr[7] =12; // Include i=12
        aIndexArr[8] =15; aIndexArr[9] =15; // Include i=15
        aIndexArr[10]=17; aIndexArr[11]=17; // Include i=17
        aIndexArr[12]=20; aIndexArr[13]=20; // Include i=20
        aIndexArr[14]=23; aIndexArr[15]=23; // Include i=23
        MYPRINTF(_T("\nEntered my_test_model 25"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/filleted_box.smb"),
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          INCLUDE_ARRAY,eSolverType,rPartBreps, 
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE));  // Nx2 Test
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 36);
      }

    if (lCount == 116) 
      {
        double dBallRadius = 2.0;
        ULONG lNumIntervals = 8;
        ULONG aIndexArr[MAX_ARR_SIZE];
        aIndexArr[0] = 3; aIndexArr[1] = 3; // Include i=3
        aIndexArr[2] = 6; aIndexArr[3] = 6; // Include i=6
        aIndexArr[4] = 9; aIndexArr[5] = 9; // Include i=9
        aIndexArr[6] =12; aIndexArr[7] =12; // Include i=12
        aIndexArr[8] =15; aIndexArr[9] =15; // Include i=15
        aIndexArr[10]=17; aIndexArr[11]=17; // Include i=17
        aIndexArr[12]=20; aIndexArr[13]=20; // Include i=20
        aIndexArr[14]=23; aIndexArr[15]=23; // Include i=23
        MYPRINTF(_T("\nEntered my_test_model 26"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/filleted_box.smb"),
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          INCLUDE_ARRAY,eSolverType,rPartBreps,
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE));  // Nx2 - big radius Test
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 36);
      }

    if (lCount == 117) 
      {
        double dBallRadius = 1.0;
        ULONG lNumIntervals = 1;
        ULONG aIndexArr[MAX_ARR_SIZE];
        aIndexArr[0] = 0; aIndexArr[1] = 5; // Include 0<=i<=5
        MYPRINTF(_T("\nEntered my_test_model 27"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/ext_w_hole.smb"),
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          INCLUDE_ARRAY,eSolverType,rPartBreps, 
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE));  // Nx2 Test
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 35);
      }

    if (lCount == 118) 
      {
        double dBallRadius = 1.0;
        ULONG lNumIntervals = 1;
        ULONG aIndexArr[MAX_ARR_SIZE];
        aIndexArr[0] = 3; aIndexArr[1] = 4; // Include 3<=i<=4
        MYPRINTF(_T("\nEntered my_test_model 28"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/l_shaped_box.smb"),
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          INCLUDE_ARRAY,eSolverType,rPartBreps,
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE));  // Nx2 Test
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 25);
      }

    if (lCount == 119) 
      {
        double dBallRadius = 1.0;
        ULONG lNumIntervals = 2;
        ULONG aIndexArr[MAX_ARR_SIZE];
        aIndexArr[0] = 3; aIndexArr[1] = 4; // Include i=3,4
        aIndexArr[2] =13; aIndexArr[3] =13; // Include i=13
        MYPRINTF(_T("\nEntered my_test_model 29"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/l_shaped_box.smb"),
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          INCLUDE_ARRAY,eSolverType,rPartBreps, 
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE));  // Nx3 concave Test
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 28);
      }

    if (lCount == 120) 
      {
        double dBallRadius = 1.0;
        ULONG lNumIntervals = 0; // Include all edges
        ULONG aIndexArr[MAX_ARR_SIZE];
        MYPRINTF(_T("\nEntered my_test_model 30"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/shell_fillet.smb"),
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          EXCLUDE_ARRAY,eSolverType,rPartBreps, 
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE));
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 18);
      }

    if (lCount == 121) 
      {
        if (eSolverType == SM_FS_VARIABLE_RADIUS) {
            return SM_SUCCESS; // Skip this test
        }
        double dBallRadius = 1.0;
        ULONG lNumIntervals = 5;
        ULONG aIndexArr[MAX_ARR_SIZE];
        aIndexArr[0] = 5; aIndexArr[1] = 5; // Include i=5
        aIndexArr[2] = 7; aIndexArr[3] = 7; // Include i=7
        aIndexArr[4] =12; aIndexArr[5] =13; // Include i=12,13
        aIndexArr[6] =15; aIndexArr[7] =15; // Include i=15
        aIndexArr[8] =24; aIndexArr[9] =24; // Include i=24
        MYPRINTF(_T("\nEntered my_test_model 31"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/6_sided_corner.smb"),
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          INCLUDE_ARRAY,eSolverType,rPartBreps,
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE));
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 57);
      }

    if (lCount == 122) 
      {
        if (eSolverType == SM_FS_VARIABLE_RADIUS) {
            return SM_SUCCESS; // Skip this test
        }
        double dBallRadius = 0.5;
        ULONG lNumIntervals = 0; // Include all edges
        ULONG aIndexArr[MAX_ARR_SIZE];
        MYPRINTF(_T("\nEntered my_test_model 32"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cyl_sphere.smb"),
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          EXCLUDE_ARRAY,eSolverType,rPartBreps,
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE));
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 8);
      }

    if (lCount == 123) 
      {
        double dBallRadius = 1.0;
        ULONG lNumIntervals = 0; // Include all edges
        ULONG aIndexArr[MAX_ARR_SIZE];
        MYPRINTF(_T("\nEntered my_test_model 33"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/corner_box.smb"),
                          1.0e-4,dBallRadius,aIndexArr,lNumIntervals,
                          EXCLUDE_ARRAY,eSolverType,rPartBreps,
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE));
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 60);
      }

    if (lCount == 124) 
      {
        double dBallRadius = 1.0;
        ULONG lNumIntervals = 0; // Includes all edges
        ULONG aIndexArr[MAX_ARR_SIZE];
        SmBoolean bDoGlobalMerge = FALSE;
        SmBoolean bMakeChamfer = FALSE;
        SmBoolean bMakeBevelCorner = TRUE;
        MYPRINTF(_T("\nEntered my_test_model 34"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Solids/box.smb"), // NxN Bevel
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          EXCLUDE_ARRAY,eSolverType,rPartBreps, 
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          bDoGlobalMerge,bMakeChamfer,bMakeBevelCorner));
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 48);
      }

    if (lCount == 125) 
      {
        double dBallRadius = 10.0;
        ULONG lNumIntervals = 0; // Includes all edges
        ULONG aIndexArr[MAX_ARR_SIZE];
        SmBoolean bDoGlobalMerge = FALSE;
        SmBoolean bMakeChamfer = TRUE;
        MYPRINTF(_T("\nEntered my_test_model 35"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Solids/pyramid.smb"), // NxN Chamfer ))
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          EXCLUDE_ARRAY,eSolverType,rPartBreps, 
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          bDoGlobalMerge,bMakeChamfer,FALSE));
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 32); // pyramid
        //SM_ASSERT(lTotalEdges == 48); // box
      }

    if (lCount == 126) 
      {
        double dBallRadius = 0.5;
        ULONG lNumIntervals = 8;
        ULONG aIndexArr[MAX_ARR_SIZE];
        aIndexArr[0] = 4; aIndexArr[1] = 4; // Include i=4
        aIndexArr[2] = 5; aIndexArr[3] = 5; // Include i=5
        aIndexArr[4] =10; aIndexArr[5] =10; // Include i=10
        aIndexArr[6] =21; aIndexArr[7] =22; // Include i=21,22
        aIndexArr[8] =24; aIndexArr[9] =24; // Include i=24
        aIndexArr[10]=27; aIndexArr[11]=27; // Include i=27
        aIndexArr[12]=29; aIndexArr[13]=29; // Include i=29
        aIndexArr[14]=30; aIndexArr[15]=31; // Include i=30,31
        MYPRINTF(_T("\nEntered my_test_model 36"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/torii_fillet.smb"),
                          1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                          INCLUDE_ARRAY,eSolverType,rPartBreps, 
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE));
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 53);
      }

    if (lCount == 127) 
      {
        // NOTE: This one regressed, 5/19/2010.  To fix it, see the comments
        // in SmFindPCExtremaEFO::Evaluate(), in SmCurve.cpp.
        // This succeeds at dLim2dDeriv = 0.536360066877
        //     and fails at dLim2dDeriv = 0.536360066878
        // both of which return correct results from
        // SmCurve::LocalPointSolve() and GlobalPointSolve().

        double dBallRadius = 1.5 ; // 2.0 ; gwc: causes a rail curve with a cusp.
                                   //       Case is documented in file: ProgTest_Regression_my_test_suit_1_127_Rail_with_Cusp.jpg
        ULONG lNumIntervals = 2;
        ULONG aIndexArr[MAX_ARR_SIZE];
        aIndexArr[0] = 2; aIndexArr[1] = 3; // Include i=2,3
        aIndexArr[2] = 5; aIndexArr[3] = 5; // Include i=5
        SmBoolean bDoGlobalMerge = FALSE;
        SmBoolean bMakeChamfer = TRUE;
        MYPRINTF(_T("\nEntered my_test_model 36"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/extrusion0206.smb"),
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          INCLUDE_ARRAY,eSolverType,rPartBreps, 
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          bDoGlobalMerge,bMakeChamfer,FALSE)); // NxN Chamfer & Self-intersect
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 14);  // was 20 (?)

        // Also check face count: Radius 2.0 is bigger than the radius of
        // curvature of one edge and so it's a self-intersecting rail curve
        // (bow-tie).  The end result is the top face (z==22) is missing.
        SmTArray<SmFace*> sFaces;
        if (rPartBreps.GetSize()> 0) rPartBreps.GetLast()->GetFaces(sFaces);
        ULONG lTotalFaces = sFaces.GetSize();
        SM_ASSERT(lTotalFaces == 8);
      }

    if (lCount == 128) 
      {
        double dBallRadius = 0.5;
        ULONG lNumIntervals = 0; // Include all edges
        ULONG aIndexArr[MAX_ARR_SIZE];
        MYPRINTF(_T("\nEntered my_test_model 37"));
        SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/shell_fillet2.smb"),
                          SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                          EXCLUDE_ARRAY,eSolverType,rPartBreps, 
                          FALSE, // bRebuildUVTrimCurves in my_read_brep()
                          FALSE,FALSE,FALSE));
        SmTArray<SmEdge*> sEdges;
        if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
        ULONG lTotalEdges = sEdges.GetSize();
        SM_ASSERT(lTotalEdges == 22);
      }

    if (rPartBreps.GetSize() > 0)
    {
        SmBrep *pBrepToFillet = rPartBreps.GetLast();
        pBrepToFillet->Dump();

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (0) {
                smgfx_Erase();
            }
            pBrepToFillet->Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif 
    }  

    if (lCount >= lLastTestCount)
        rbIsDone = TRUE;

    return SM_SUCCESS;
} // end my_test_suite_1

/*******************************************************
PURPOSE --- Customer Test suite #2
    run following tests 
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/test1.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/fillet0114.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/torii_unioned.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/test4.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/test5.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/test6.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/test7.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/test8.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/test9.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/sphere_dif_box.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/cyl_cyl_shell.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/sphere_plane.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/bague.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/cap1.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/cap1.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/caps_coons.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/caps_nside.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/fillet1010.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/fillet0227.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/vase.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/vase1.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/vase2.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/vase3.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/0311.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/box_diff_cyl.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/blade_hub.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/fillet1018.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/crown.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/loft_brick.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/box_extrusion1.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/box_extrusion2.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/box_w_slot.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/coons_01_03_29.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/nurb_cone.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/cube_sphere2.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/Heart.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/knife.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/fillet1105.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/table_cloth.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/propeller.smb",
  my_test_model     "../../TestFiles/pt_TestFiles/Fillet/open_shell.smb",
                                                                                                                                                                                                                                                                                                                                                                                                                            
USAGE NOTES ---
********************************************************/
SmStatus my_test_suite_2
  (const SmContext & crContext,          // in : cotnext for new object construction
   SmTArray<SmBrep*> & rPartBreps,       // out: container for Brep Parts read from file
   ULONG & lCount,                       // i/o: target test case index, incremented with each call
   SmFilletSolverType eSolverType,       // in : oneof:
   SmBoolean & rbIsDone)                 // out: TRUE =lCount is incremented to the last test case index
                                         //      FALSE=lCount is incremented to a number less than last test case index
{
  lCount ++;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  SM_SPRINTF(sBuff,_T("\nEntered my_test_suite_2 iter: %ld"), lCount) ;
  MYPRINTF(sBuff) ;

  // CONSTANT RADIUS REGRESSION TESTS
  ULONG StartCount = 201;
  ULONG lLastTestCount = 237;


  if (lCount < StartCount) lCount = StartCount;

  // self-intersecting fillet
  if (lCount == 201) 
    {
      // GWC: BallRadius = 1.0 yields a self-intersecting fillet.
      //      Self-intersection needs to be debugged.  
      smos_WriteBuffer(_T("\n  Reducing Ball Radius from 1.0 to 0.5 to avoid self-intersecting fillet")) ;

      // double dBallRadius = 1.0;
      double dBallRadius = 0.5 ;

      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 0; aIndexArr[1] = 1; // Exclude 0=<i<=1
      MYPRINTF(_T("\nEntered my_test_model 38"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/test1.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        EXCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 5); // GWC TotalEdge count check will need to change
                                   //  when the self-intersecting dBallRadius = 1.0 is restored.
    }

  if (lCount == 202) 
    {
      // GWC: I think this is another self-intersecting fillet.  The following call
      //      stack builds a crossSection curve which winds around a tight corner twice
      //      creating a problem curve that gets into trouble in DropCurve.
      //       SmSurface::DropCurve(const SmContext & {...}, const SmExtent2d & {...}, const SmBSplineCurve & {...}, const SmExtent1d & {...}, double 0.0021414192285720002, double & 0.00000000000000000, double & 0.00000000000000000, SmTArray<SmBSplineCurve *> & {...}, int 1) line 4759 + 110 bytes
      //       sm_ExamineNFixFilletGeoms(SmFilletGeom * 0x071caee8, int 1, SmFilletGeom * 0x071caee8, int 0, SmFilletVertex * 0x06d9e2a8, SmFilletVertex * 0x0741ad30, double 0.0021414192285720002, int 0) line 4200 + 174 bytes
      //       SmFilletEdge::CalcCrossSection(const SmContext & {...}, SmFilletCorner * 0x072fc168) line 4754 + 47 bytes
      //       SmFilletEdge::CalcCornerEdgeGeom() line 4388 + 19 bytes
      //       SmFilletCorner::CalcCornerEdgeGeom() line 1261 + 11 bytes
      //       SmFilletExecutive::CreateFilletBrep(SmFilletErrorInfo * 0x00000000) line 1782 + 11 bytes
      //       SmFilletExecutive::DoFilleting(SmFilletErrorInfo * 0x00000000) line 2215 + 15 bytes

      double dBallRadius = 1.0 ;   // GWC: Changed to 1.0 until after DropCurve rewrite is complete
    // double dBallRadius = 3.175; //      after which it needs to be set back to 3.175 and debugged.
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 2; aIndexArr[1] = 2; // Include i=2
      MYPRINTF(_T("\nEntered my_test_model 39"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/fillet0114.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 5);  // while dBallRadius == 1.0, when dBALLRadius == 3.175 then SM_ASSERT(lTotalEdges == 11);
    }

  if (lCount == 203) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 2; aIndexArr[1] = 2; // Include i=2
      MYPRINTF(_T("\nEntered my_test_model 40"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/torii_unioned.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 7);
    }

  if (lCount == 204) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 2;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 2; aIndexArr[1] = 3; // Include i=2,3
      aIndexArr[2] = 7; aIndexArr[3] = 7; // Include i=7
      MYPRINTF(_T("\nEntered my_test_model 41"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/test4.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 23);
    }

  if (lCount == 205) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 3;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 2; aIndexArr[1] = 3; // Include i=2,3
      aIndexArr[2] = 5; aIndexArr[3] = 5; // Include i=5
      aIndexArr[4] = 9; aIndexArr[5] = 9; // Include i=9
      MYPRINTF(_T("\nEntered my_test_model 42"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/test5.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 20);
    }

  if (lCount == 206) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 4;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 1; aIndexArr[1] = 2; // Include i=1,2
      aIndexArr[2] = 4; aIndexArr[3] = 4; // Include i=4
      aIndexArr[4] = 6; aIndexArr[5] = 6; // Include i=6
      aIndexArr[6] = 8; aIndexArr[7] = 8; // Include i=8
      MYPRINTF(_T("\nEntered my_test_model 43"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/test6.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 35);
    }

  if (lCount == 207) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 2;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 0; aIndexArr[1] = 0; // Include i=0
      aIndexArr[2] =13; aIndexArr[3] =14; // Include i=13,14
      MYPRINTF(_T("\nEntered my_test_model 44"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/test7.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 26);
    }

  if (lCount == 208) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 3; aIndexArr[1] = 3; // Include i=3
      MYPRINTF(_T("\nEntered my_test_model 45"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/test8.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 9);
    }

  if (lCount == 209) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 2;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 7; aIndexArr[1] = 8; // Include i=7,8
      aIndexArr[2] =11; aIndexArr[3] =11; // Include i=11
      MYPRINTF(_T("\nEntered my_test_model 46"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/test9.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 21);
    }

  if (lCount == 210) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 0; // Include all edges
      ULONG aIndexArr[MAX_ARR_SIZE];
      MYPRINTF(_T("\nEntered my_test_model 47"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/sphere_dif_box.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        EXCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 55);
    }

  if (lCount == 211) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 2;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 2; aIndexArr[1] = 2; // Include i=2
      aIndexArr[2] = 5; aIndexArr[3] = 5; // Include i=5
      MYPRINTF(_T("\nEntered my_test_model 48"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cyl_cyl_shell.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 13);
    }

  if (lCount == 212) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 3; aIndexArr[1] = 3; // Include i=3
      MYPRINTF(_T("\nEntered my_test_model 49"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/sphere_plane.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        TRUE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 6);
    }

  if (lCount == 213) 
    {
      double dBallRadius = 0.1;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 7; aIndexArr[1] = 7; // Include i=7
      MYPRINTF(_T("\nEntered my_test_model 50"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/bague.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 13);
    }

  if (lCount == 214) 
    {
      // 1st time bDoGlobalMerge == TRUE
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 3; aIndexArr[1] = 3; // Include i=3
      SmBoolean bDoGlobalMerge = TRUE;
      MYPRINTF(_T("\nEntered my_test_model 51"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cap1.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        bDoGlobalMerge,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 10);
    }

  if (lCount == 215) 
    {
      double dBallRadius = 0.2;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 3; aIndexArr[1] = 3; // Include i=3
      SmBoolean bDoGlobalMerge = TRUE;
      MYPRINTF(_T("\nEntered my_test_model 52"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cap1.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        bDoGlobalMerge,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 10);
    }

  if (lCount == 216) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 0; aIndexArr[1] = 1; // Exclude 0<=i<=1
      SmBoolean bDoGlobalMerge = TRUE;
      MYPRINTF(_T("\nEntered my_test_model 53"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/caps_coons.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        EXCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        bDoGlobalMerge,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 14);
    }

  if (lCount == 217) 
    {
      double dBallRadius = 0.5;
      ULONG lNumIntervals = 4;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 2; aIndexArr[1] = 5; // Include 2<=i<=5
      aIndexArr[2] = 8; aIndexArr[3] = 9; // Include 8<=i<=9
      aIndexArr[4] =11; aIndexArr[5] =12; // Include 11<=i<=12
      aIndexArr[6] =14; aIndexArr[7] =15; // Include 14<=i<=15
      SmBoolean bDoGlobalMerge = TRUE;
      MYPRINTF(_T("\nEntered my_test_model 54"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/caps_nside.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        bDoGlobalMerge, FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 37);
    }

  if (lCount == 218) 
    {
      double dBallRadius = 38.1;
      ULONG lNumIntervals = 4;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 2; aIndexArr[1] = 2; // Include i=2
      aIndexArr[2] = 4; aIndexArr[3] = 4; // Include i=4
      aIndexArr[4] = 8; aIndexArr[5] = 8; // Include i=8
      aIndexArr[6] = 9; aIndexArr[7] = 9; // Include i=9
      MYPRINTF(_T("\nEntered my_test_model 55"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/fillet1010.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 16);
    }

  if (lCount == 219) 
    {
#if 1
//        GAC - regression where intersection edge of end face doesn't close
      double dBallRadius = 400.0;
      ULONG lNumIntervals = 2;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 2; aIndexArr[1] = 2; // Include i=2
      aIndexArr[2] = 4; aIndexArr[3] = 4; // Include i=4
      MYPRINTF(_T("\nEntered my_test_model 55"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/fillet0227.smb"),
                        0.1370,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 15);
#endif
    }

  if (lCount == 220) 
    {
      double dBallRadius = 6.35;
      ULONG lNumIntervals = 6;
      ULONG aIndexArr[MAX_ARR_SIZE];
      if (0) {
      lNumIntervals = 1;
      aIndexArr[0] = 2; aIndexArr[1] = 2; // Include i=1
      }
      else {
      aIndexArr[0] = 0; aIndexArr[1] = 0; // Include i=0
      aIndexArr[2] = 2; aIndexArr[3] = 2; // Include i=2
      aIndexArr[4] = 4; aIndexArr[5] = 4; // Include i=4
      aIndexArr[6] = 7; aIndexArr[7] = 7; // Include i=7
      aIndexArr[8] =17; aIndexArr[9] =19; // Include i=17-19
      aIndexArr[10]=21; aIndexArr[11]=21; // Include i=21
      }
      MYPRINTF(_T("\nEntered my_test_model 56"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/vase.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 48);
    }

  if (lCount == 221) 
    {
      double dBallRadius = 5.08;
      ULONG lNumIntervals = 8;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 2; aIndexArr[1] = 2; // Include i=2
      aIndexArr[2] = 5; aIndexArr[3] = 5; // Include i=5
      aIndexArr[4] = 6; aIndexArr[5] = 6; // Include i=6
      aIndexArr[6] =10; aIndexArr[7] =10; // Include i=10
      aIndexArr[8] =12; aIndexArr[9] =12; // Include i=12
      aIndexArr[10]=16; aIndexArr[11]=16; // Include i=16
      aIndexArr[12]=18; aIndexArr[13]=18; // Include i=18
      aIndexArr[14]=22; aIndexArr[15]=22; // Include i=22
      MYPRINTF(_T("\nEntered my_test_model 57"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/vase1.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 64);
    }

  if (lCount == 222) 
    {
#if 1
//       GAC - regression here need to fix
      double dBallRadius = 10.16;
      ULONG lNumIntervals = 8;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 1; aIndexArr[1] = 1; // Include i=1
      aIndexArr[2] = 3; aIndexArr[3] = 3; // Include i=3
      aIndexArr[4] = 5; aIndexArr[5] = 5; // Include i=5
      aIndexArr[6] = 8; aIndexArr[7] = 8; // Include i=8
      aIndexArr[8] =16; aIndexArr[9] =16; // Include i=16
      aIndexArr[10]=18; aIndexArr[11]=18; // Include i=18
      aIndexArr[12]=20; aIndexArr[13]=20; // Include i=20
      aIndexArr[14]=22; aIndexArr[15]=22; // Include i=22
      MYPRINTF(_T("\nEntered my_test_model 58"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/vase2.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 48);
#endif
    }

  if (lCount == 223) 
    {
      double dBallRadius = 7.62;
      ULONG lNumIntervals = 0;
      ULONG aIndexArr[MAX_ARR_SIZE];
      MYPRINTF(_T("\nEntered my_test_model 59"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/vase3.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        EXCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 112);
    }

  if (lCount == 224) 
    {
      double dBallRadius = 203.2;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 6; aIndexArr[1] = 7; // Include i=6,7
      MYPRINTF(_T("\nEntered my_test_model 60"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/0311.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 15);
    }
    
  if (lCount == 225) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =12; aIndexArr[1] =15; // Include i=12-15
      MYPRINTF(_T("\nEntered my_test_model 61"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/box_diff_cyl.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 38) // 38 after CoincidenceCheck overhaul 3/1/13
    }
    
  if (lCount == 226) // a fillet in non-Manifold model with topology of a sheet with one hole within it
    {
      double dBallRadius = 0.1;
      ULONG lNumIntervals = 4;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 5; aIndexArr[1] =5; // Include i=5
      aIndexArr[2] = 7; aIndexArr[3] =8; // Include i=7,8
      aIndexArr[4] =10; aIndexArr[5] =12; // Include i=10-12
      aIndexArr[6] =14; aIndexArr[7] =17; // Include i=14-17
      MYPRINTF(_T("\nEntered my_test_model 62"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/blade_hub.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 64);
    }

  if (lCount == 227) 
    {
      double dBallRadius = 3.0;
      ULONG lNumIntervals = 8;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 0; aIndexArr[1] = 1; // Include i=0,1
      aIndexArr[2] = 7; aIndexArr[3] = 7; // Include i=7
      aIndexArr[4] =10; aIndexArr[5] =10; // Include i=10
      aIndexArr[6] =13; aIndexArr[7] =13; // Include i=13
      aIndexArr[8] =16; aIndexArr[9] =16; // Include i=16
      aIndexArr[10]=19; aIndexArr[11]=19; // Include i=19
      aIndexArr[12]=25; aIndexArr[13]=25; // Include i=25
      aIndexArr[14]=22; aIndexArr[15]=22; // Include i=22
      MYPRINTF(_T("\nEntered my_test_model 63"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/fillet1018.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 45);
    }

  if (lCount == 228) 
    {  
      double dBallRadius = 0.2;
      ULONG lNumIntervals = 2;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =12; aIndexArr[1] =12; // Include i=12
      aIndexArr[2] =24; aIndexArr[3] =26; // Include i=24-26
      MYPRINTF(_T("\nEntered my_test_model 64"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/crown.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 48);
    }

  if (lCount == 229) 
    {
      double dBallRadius = 6.348;
      ULONG lNumIntervals = 4;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 1; aIndexArr[1] = 1; // Include i=1
      aIndexArr[2] = 5; aIndexArr[3] = 5; // Include i=5
      aIndexArr[4] = 8; aIndexArr[5] = 8; // Include i=8
      aIndexArr[6] =10; aIndexArr[7] =10; // Include i=10
      MYPRINTF(_T("\nEntered my_test_model 65"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/loft_brick.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 24);
    }

  if (lCount == 230) 
    {
      double dBallRadius = 0.5;
      ULONG lNumIntervals = 2;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =12; aIndexArr[1] =12; // Include i=12
      aIndexArr[2] =14; aIndexArr[3] =17; // Include i=14-17
      MYPRINTF(_T("\nEntered my_test_model 66"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/box_extrusion1.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE)); // Nx2- stress
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 43); // 43 after CoincidenceCheck overhaul 2/25/13
    }

  if (lCount == 231) 
    {
      double dBallRadius = 0.5;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =13; aIndexArr[1] =17; // Include i=13-17
      MYPRINTF(_T("\nEntered my_test_model 67"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/box_extrusion2.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE)); // Nx2- stress
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 45) ; // USEDTOBE = 46; SHOULD be 45.
        // (If 46, then there's a small edge that didn't get stitched. [bd Feb 07])
        // (gwc: this should be a manifold result - but it seems to missing a small FilletEdge at a FilletCorner)
    }

  if (lCount == 232) 
    {
      double dBallRadius = 0.01;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 4; aIndexArr[1] = 4; // Include i=4
      MYPRINTF(_T("\nEntered my_test_model 68"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/box_w_slot.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        TRUE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE)); // Nx2- stress
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 53);
    }

  if (lCount == 233) 
    {
      double dBallRadius = 2.0;
      ULONG lNumIntervals = 3;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 1; aIndexArr[1] = 1; // Include i=1
      aIndexArr[2] = 4; aIndexArr[3] = 4; // Include i=4
      aIndexArr[4] = 7; aIndexArr[5] = 7; // Include i=7
      MYPRINTF(_T("\nEntered my_test_model 69"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/coons_01_03_29.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 15);
    }

  if (lCount == 234) 
    {
      double dBallRadius = 0.25;
      ULONG lNumIntervals = 8;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 2; aIndexArr[1] = 2; // Include i=2
      aIndexArr[2] = 6; aIndexArr[3] = 6; // Include i=6
      aIndexArr[4] = 9; aIndexArr[5] = 9; // Include i=9
      aIndexArr[6] =12; aIndexArr[7] =12; // Include i=12
      aIndexArr[8] =15; aIndexArr[9] =15; // Include i=15
      aIndexArr[10]=18; aIndexArr[11]=18; // Include i=18
      aIndexArr[12]=21; aIndexArr[13]=21; // Include i=21
      aIndexArr[14]=23; aIndexArr[15]=23; // Include i=23
      MYPRINTF(_T("\nEntered my_test_model 70"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/nurb_cone.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 40);
    }

  if (lCount == 235) 
    {
      double dBallRadius = 0.2;
      ULONG lNumIntervals = 3;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =13; aIndexArr[1] =13; // Include i=13
      aIndexArr[2] =14; aIndexArr[3] =14; // Include i=14
      aIndexArr[4] = 7; aIndexArr[5] = 7; // Include i=7
      SmBoolean bDoGlobalMerge = FALSE;
      SmBoolean bMakeChamfer = TRUE;
      MYPRINTF(_T("\nEntered my_test_model 71"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cube_sphere2.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        bDoGlobalMerge,bMakeChamfer,FALSE)); // NxN Chamfer
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 24);
    }

  if (lCount == 236) 
    {
      double dBallRadius = 0.2;
      ULONG lNumIntervals = 0;
      ULONG aIndexArr[MAX_ARR_SIZE];

      MYPRINTF(_T("\nEntered my_test_model 72"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/Heart.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        EXCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE)); // Cubic-rail-rail-interpolation
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 26);
    }

  if (lCount == 237) 
    {
      double dBallRadius = 0.25;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 2; aIndexArr[1] = 3; // Include i=2,3
      MYPRINTF(_T("\nEntered my_test_model 73"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/knife.smb"),
                        1.0e-3,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        TRUE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 11);  // was 12, okay with 11. After review - could be 10.  Part has 1 topological vertex that could be removed.
                                     // I guess it once had two topological vertices.
    }

  if (rPartBreps.GetSize() > 0)
  {
      SmBrep *pBrepToFillet = rPartBreps.GetLast();
      pBrepToFillet->Dump();
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics()) {
          if (0) {
              smgfx_Erase();
          }
          smgfx_SetLook(1, 2); if (pBrepToFillet) pBrepToFillet->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // SM_GFX_CODE
    }

  if (lCount >= lLastTestCount) 
    {
      rbIsDone = TRUE;
    }

  return SM_SUCCESS;

} // end my_test_suite_2

/*******************************************************
PURPOSE --- Test suite #3
            
USAGE NOTES ---
********************************************************/
SmStatus my_test_suite_3
  (const SmContext & crContext,          // in : context for new object construction
   SmTArray<SmBrep*> & rPartBreps,       // out: container for Brep Parts read from file
   ULONG & lCount,                       // i/o: target test case index, incremented with each call
   SmBoolean & rbIsDone)                 // out: TRUE =lCount is incremented to the last test case index
                                         //      FALSE=lCount is incremented to a number less than last test case index
{
  lCount ++;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  SM_SPRINTF(sBuff,_T("\nEntered my_test_suite_3 iter: %ld"), lCount) ;
  MYPRINTF(sBuff) ;
  SmTArray<SmBrep*> sBreps;

  // Creates circular cross section fillet
  // that approximates the arc to within 0.1 of it's radius.
  SmCircularCrossSectionFSG sFSGCircular(FALSE, SM_ZONE_TOL_3D);
  SmLinearCrossSectionFSG sFSGLinear;          // Create Linear
  double dStepBackFactor = 1.0;
  SmMakeSurfaceBlendSIH sSIH(dStepBackFactor);
  
  ULONG StartCount = 301;
  ULONG lLastTestCount = 308;


  if (lCount < StartCount) lCount = StartCount;

  if (lCount == 301) 
    {
      DELETE_ALL_PARTS(rPartBreps);

      SmBrep *pBrep = new(crContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Fillet/l_shaped_box.smb"));
      SER(pBrep->ReadFromFile(crContext, _T("../../TestFiles/pt_TestFiles/Fillet/l_shaped_box.smb"),SM_ASCII));
      rPartBreps.Add(pBrep);

      SmTArray<SmEdge*> sEdges;
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif
      SmFilletExecutive * pFilExec =  new (crContext) SmFilletExecutive(crContext, pBrep);
      SmObjDelete sClean(pFilExec);
      pFilExec->SetSelfIntersectionHandler(&sSIH);

      SmTArray<SmLinearFilletLaw*> sLaws ;
      SmObjsDelete<SmLinearFilletLaw*> sCleanupLaws(&sLaws); 
      for (ULONG i=0; i<sEdges.GetSize(); i++) 
        {
          if (i != 0 && i != 13) continue;  // Nx1 Zero-Radius Test
          SmEdgeuse *pEU = sEdges[i]->GetBlendEdgeuse();
          if (pEU == NULL) continue;
      
          SmLinearFilletLaw  * pLaw = new(crContext) SmLinearFilletLaw(2.0,0.0); 
          SmVariableRadiusFS * pFS  = new(crContext) SmVariableRadiusFS(crContext,1.0e-3,
              30.0*SM_PI/180.0, 2.0*SM_PI/180.0, 1.0, pEU, *pLaw, FALSE);
          pFS->SetFilletSurfaceGenerator(&sFSGCircular);
          pFilExec->LoadFilletSolver(pFS);
          sLaws.Add(pLaw) ;
        }
      SER(pFilExec->CreateFilletCorners());
      SER(pFilExec->DoFilleting());  // note: increments unlocked mark value

      sEdges.ReSet();
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 22);
    }
  if (lCount == 302) 
    {
      DELETE_ALL_PARTS(rPartBreps);

      SmBrep *pBrep = new(crContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
      SER(pBrep->ReadFromFile(crContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII));
      rPartBreps.Add(pBrep);

      SmTArray<SmEdge*> sEdges;
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif
      SmFilletExecutive * pFilExec =  new (crContext) SmFilletExecutive(crContext, pBrep);
      pFilExec->SetSelfIntersectionHandler(&sSIH);
      SmObjDelete sClean(pFilExec);
      for (ULONG i=0; i<sEdges.GetSize(); i++) 
        {
          if (i > 1) break;//Nx2bevel
          SmEdgeuse *pEU = sEdges[i]->GetBlendEdgeuse();
          if (pEU == NULL) continue;

          double dBallRadius = 0.5;
          if (i>0) dBallRadius = 1.5;
          SmConstantRadiusFS * pFS = new(crContext) SmConstantRadiusFS(crContext,
              SM_ZONE_TOL_3D,30.0*SM_PI/180.0, 2.0*SM_PI/180.0, dBallRadius, pEU);
          pFS->SetFilletSurfaceGenerator(&sFSGCircular);

          // Load the solver into the fillet executive
          pFilExec->LoadFilletSolver(pFS);
        }
      SER(pFilExec->CreateFilletCorners());
      SER(pFilExec->DoFilleting());  // note: increments unlocked mark value

      sEdges.ReSet();
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 18);
    }

  if (lCount == 303) 
    {
      DELETE_ALL_PARTS(rPartBreps);
      
      SmBrep *pBrep = new(crContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
      SER(pBrep->ReadFromFile(crContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII));
      rPartBreps.Add(pBrep);
      
      SmTArray<SmEdge*> sEdges;
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pBrep) ;

          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif

      SmFilletExecutive * pFilExec =  new (crContext) SmFilletExecutive(crContext, pBrep);
      pFilExec->SetSelfIntersectionHandler(&sSIH);
      SmObjDelete sClean(pFilExec);
      for (ULONG i=0; i<sEdges.GetSize(); i++) 
        {
          //if (i > 1 && i != 5) continue; //3x3bevel Test
          SmEdgeuse *pEU = sEdges[i]->GetBlendEdgeuse();
          if (pEU == NULL) continue;

          double dBallRadius = 1.0;
          SmConstantRadiusFS * pFS = new(crContext) SmConstantRadiusFS
                                                      (crContext,          // in : crContext,        
                                                       SM_ZONE_TOL_3D,             // in : dThisApproxTol3d,
                                                       30.0*SM_PI/180.0,   // in : dAngleTolerance,  
                                                       2.0*SM_PI/180.0,    // in : dTangencyTolerance
                                                       dBallRadius,        // in : dFilletRadius,    
                                                       pEU);               // in : pEdgeuse          
          pFS->SetFilletSurfaceGenerator(&sFSGCircular);                   

          // Load the solver into the fillet executive
          pFilExec->LoadFilletSolver(pFS);
        }
      SER(pFilExec->CreateFilletCorners());
      SmTArray<SmFilletCorner*> & rCorners = pFilExec->GetFilletCorners();
      for (ULONG j=0; j<rCorners.GetSize(); j++) 
        {
          rCorners[j]->SetSetBackDist(1.25);
        }
      SER(pFilExec->DoFilleting());  // note: increments unlocked mark value

      sEdges.ReSet();
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 168);
    }

  if (lCount == 304) 
    {
      DELETE_ALL_PARTS(rPartBreps);
     
      SmBrep *pBrep = new(crContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
      SER(pBrep->ReadFromFile(crContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII));
      rPartBreps.Add(pBrep);
	  
      SmTArray<SmEdge*> sEdges;
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif
      SmFilletExecutive * pFilExec =  new (crContext) SmFilletExecutive(crContext, pBrep);
      pFilExec->SetSelfIntersectionHandler(&sSIH);
      SmObjDelete sClean(pFilExec);
      ULONG alIndex[] = { 0, 1, 5 };
      double adBallRadii[] = { 1.0, 2.0, 3.0 };
      for (ULONG i=0; i<3; i++) 
        {
          ULONG lIndex = alIndex[i];
          SmEdgeuse *pEU = sEdges[lIndex]->GetBlendEdgeuse();
          if (pEU == NULL) continue;

          double dRadius = adBallRadii[i];
          SmConstantRadiusFS * pFS = new(crContext) SmConstantRadiusFS(crContext,
              SM_ZONE_TOL_3D,30.0*SM_PI/180.0, 2.0*SM_PI/180.0, dRadius, pEU);
          if (0) 
            {
              pFS->SetFilletSurfaceGenerator(&sFSGLinear);
            }
          else 
            {
              pFS->SetFilletSurfaceGenerator(&sFSGCircular);
            }

          // Load the solver into the fillet executive
          pFilExec->LoadFilletSolver(pFS);
        }
      SER(pFilExec->CreateFilletCorners());
      SER(pFilExec->DoFilleting());  // note: increments unlocked mark value

      sEdges.ReSet();
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 22);
    }

  if (lCount == 305) 
    {
      DELETE_ALL_PARTS(rPartBreps);

      SmBrep *pBrep = new(crContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb"));
      SER(pBrep->ReadFromFile(crContext, _T("../../TestFiles/pt_TestFiles/Solids/box.smb"),SM_ASCII));
      rPartBreps.Add(pBrep);

      SmTArray<SmEdge*> sEdges;
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif
      SmFilletExecutive * pFilExec =  new (crContext) SmFilletExecutive(crContext, pBrep);
      pFilExec->SetSelfIntersectionHandler(&sSIH);
      SmObjDelete sClean(pFilExec);
      ULONG alIndex[] = { 0, 1, 5 };
      double adBallRadii[] = { 1.0, 2.0, 2.0 };
      for (ULONG i=0; i<3; i++) 
        {
          ULONG lIndex = alIndex[i];
          SmEdgeuse *pEU = sEdges[lIndex]->GetBlendEdgeuse();
          if (pEU == NULL) continue;

          double dRadius = adBallRadii[i];
          SmConstantRadiusFS * pFS = new(crContext) SmConstantRadiusFS(crContext,
              SM_ZONE_TOL_3D,30.0*SM_PI/180.0, 2.0*SM_PI/180.0, dRadius, pEU);
          if (i != 0) 
            {
              pFS->SetFilletSurfaceGenerator(&sFSGLinear);
            }
          else 
            {
              pFS->SetFilletSurfaceGenerator(&sFSGCircular);
            }

          // Load the solver into the fillet executive
          pFilExec->LoadFilletSolver(pFS);
        }
      SER(pFilExec->CreateFilletCorners());
      SER(pFilExec->DoFilleting());  // note: increments unlocked mark value

      sEdges.ReSet();
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 21);
    }

  if (lCount == 306) 
    {
      DELETE_ALL_PARTS(rPartBreps);
      SmBrep *pBrep = new(crContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Fillet/extrusion0210.smb"));
      SER(pBrep->ReadFromFile(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/extrusion0210.smb"),SM_ASCII));
      rPartBreps.Add(pBrep);
      SmTArray<SmEdge*> sEdges;
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif
      SmFilletExecutive * pFilExec =  new (crContext) SmFilletExecutive(crContext, pBrep);
      pFilExec->SetSelfIntersectionHandler(&sSIH);
      SmObjDelete sClean(pFilExec);
      SmObjDelete sCleanLaw;
      for (ULONG i=0; i<sEdges.GetSize(); i++) 
        {
          SmFilletSolver * pFS = NULL;
          if (i >= 10 && i <= 14) 
            {
              SmEdgeuse *pEU = sEdges[i]->GetBlendEdgeuse();
              if (pEU == NULL) continue;
              pFS = new(crContext) SmConstantRadiusFS(crContext,
                  SM_ZONE_TOL_3D,30.0*SM_PI/180.0, 2.0*SM_PI/180.0, 0.5, pEU);
            }
          else if (i == 24) 
            {
              SmEdgeuse *pEU = sEdges[i]->GetBlendEdgeuse();
              if (pEU == NULL) continue;
              SmLinearFilletLaw * pLaw = new(crContext) SmLinearFilletLaw(2.0,0.5);
              sCleanLaw.SetObj(pLaw); // JLMCC hunting memory leaks
              pFS = new(crContext) SmVariableRadiusFS(crContext,1.0e-3,
                  30.0*SM_PI/180.0, 2.0*SM_PI/180.0, 1.0, pEU, *pLaw, FALSE);
            }
          if (pFS) 
            {
              pFS->SetFilletSurfaceGenerator(&sFSGCircular);
              pFilExec->LoadFilletSolver(pFS);
            }
        }
      SER(pFilExec->CreateFilletCorners());
      SER(pFilExec->DoFilleting());  // note: increments unlocked mark value

      sEdges.ReSet();
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif
      // Total edges: Changed from 45 to 44, 9/27/18, because we changed
      // variable-radius functions to start and end where the final fillet
      // starts and ends, instead of just at the ends of the filleted edge.
      // This causes the end of the variable radius fillet to be slightly
      // smaller than it was (the actual end value), which causes the two
      // rails across from it to intersect, instead of coming up short and
      // being connected by an arc in their common face.  [B551 B570]
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 44);
    }

  if (lCount == 307) 
    {
      DELETE_ALL_PARTS(rPartBreps);

      SmBrep *pBrep = new(crContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Fillet/l_shaped_box.smb"));
      SER(pBrep->ReadFromFile(crContext, _T("../../TestFiles/pt_TestFiles/Fillet/l_shaped_box.smb"),SM_ASCII));
      rPartBreps.Add(pBrep);

      SmTArray<SmEdge*> sEdges;
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif
      SmFilletExecutive * pFilExec =  new (crContext) SmFilletExecutive(crContext, pBrep);
      pFilExec->SetSelfIntersectionHandler(&sSIH);
      SmObjDelete sClean(pFilExec);
      for (ULONG i=0; i<sEdges.GetSize(); i++) 
        {
          if (i != 0 && i != 13) continue;  // Nx1 Zero-Radius Test
          SmEdgeuse *pEU = sEdges[i]->GetBlendEdgeuse();
          if (pEU == NULL) continue;
      
          SmLinearFilletLaw * pLaw = new(crContext) SmLinearFilletLaw(2.0,0.0); 
          SmVariableRadiusFS  * pFS = new(crContext) SmVariableRadiusFS(crContext,1.0e-3,
              30.0*SM_PI/180.0, 2.0*SM_PI/180.0, 1.0, pEU, *pLaw, FALSE);
          pFS->SetFilletSurfaceGenerator(&sFSGCircular);
          pFilExec->LoadFilletSolver(pFS);
        }
      SER(pFilExec->CreateFilletCorners());
      SER(pFilExec->DoFilleting());  // note: increments unlocked mark value

      sEdges.ReSet();
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 22);
    }

  if (lCount == 308) 
    {
      DELETE_ALL_PARTS(rPartBreps);
      SmBrep *pBrep = new(crContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Fillet/nurb_box1.smb"));
      SER(pBrep->ReadFromFile(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/nurb_box1.smb"),SM_ASCII));
      rPartBreps.Add(pBrep);
      SmTArray<SmEdge*> sEdges;
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif
      SmFilletExecutive * pFilExec =  new (crContext) SmFilletExecutive(crContext, pBrep);
      pFilExec->SetSelfIntersectionHandler(&sSIH);
      SmObjDelete sClean(pFilExec);
      ULONG alIndex[] = { 3, 7, 8, 0, 2 };
      double adBallRadii[] = { 1.0, 2.0, 2.0, 3.0, 3.0 };
      for (ULONG i=0; i<5; i++) 
        {
          ULONG lIndex = alIndex[i];
          SmEdgeuse *pEU = sEdges[lIndex]->GetBlendEdgeuse();
          if (pEU == NULL) continue;

          double dRadius = adBallRadii[i];
          SmConstantRadiusFS * pFS = new(crContext) SmConstantRadiusFS(crContext,
              SM_ZONE_TOL_3D,30.0*SM_PI/180.0, 2.0*SM_PI/180.0, dRadius, pEU);
          pFS->SetFilletSurfaceGenerator(&sFSGCircular);

          // Load the solver into the fillet executive
          pFilExec->LoadFilletSolver(pFS);
        }
      SER(pFilExec->CreateFilletCorners());
      SER(pFilExec->DoFilleting());  // note: increments unlocked mark value

      sEdges.ReSet();
      pBrep->GetEdges(sEdges);
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
        {
          smgfx_Erase() ;
          smgfx_SetLook(1,2) ; pBrep->Draw(TRUE); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 26);
    }

  if (rPartBreps.GetSize() > 0)
  {
      SmBrep *pBrepToFillet = rPartBreps.GetLast();
      pBrepToFillet->Dump();
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics()) {
          smgfx_Erase();
          pBrepToFillet->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif
    }  

  if (lCount >= lLastTestCount) 
    {
      rbIsDone = TRUE;
    }

  return SM_SUCCESS;

} // end my_test_suite_3

/*******************************************************
PURPOSE --- Customer Test suite #4 (Rollover tests)

USAGE NOTES ---
********************************************************/
SmStatus my_test_suite_4
  (const SmContext & crContext,          // in : cotnext for new object construction
   SmTArray<SmBrep*> & rPartBreps,       // out: container for Brep Parts read from file
   ULONG & lCount,                       // i/o: target test case index, incremented with each call
   SmFilletSolverType eSolverType,       // in : oneof:
   SmBoolean & rbIsDone)                 // out: TRUE =lCount is incremented to the last test case index
                                         //      FALSE=lCount is incremented to a number less than last test case index
{
  lCount ++;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  SM_SPRINTF(sBuff,_T("\nEntered my_test_suite_4 iter: %ld"), lCount) ;
  MYPRINTF(sBuff) ;

  // ROLLOVER REGRESSION TESTS

  ULONG StartCount     = 401;
  ULONG lLastTestCount = 421;


  if (lCount < StartCount) lCount = StartCount;

  if (lCount == 401) 
    {
      SER(my_read_fillet_definition_file(crContext,_T("../../TestFiles/pt_TestFiles/Drop/FilletTests/ext_large_radius1.fdf"), rPartBreps));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      // gwc: Oct 2010, a modified intersection caused a short edge to be built in the output.
      //      The result is still good. I'm skipping this change until later.
      //      The new output edge count is 28. What needs to be done: See if the
      //      intersection results are generating an extra false positive intersection that
      //      causes another vertex to be added into the model.
      SM_ASSERT(lTotalEdges == 27 || lTotalEdges == 28);
    }

  if (lCount == 402) 
    {
      SER(my_read_fillet_definition_file(crContext,_T("../../TestFiles/pt_TestFiles/Drop/FilletTests/l_box_w_boss_large_radius.fdf"), rPartBreps));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 27);   // gwc: when it's 31 4 old edges were turned into topological edges and failed to delete
    }

  if (lCount == 403) 
    {
      double dBallRadius = 5.08;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 4; aIndexArr[1] = 4; // Include i=4
      SmBoolean bDoGlobalMerge = TRUE;
      MYPRINTF(_T("\nEntered my_test_model 79"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/fillet1019.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        bDoGlobalMerge, FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 24) ; // USEDTOBE = 24);  // back to 24 - seems to be okay but some curve has repeated coincident control points - needs more review
    }

  if (lCount == 404) 
    {
      double dBallRadius = 2.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 12; aIndexArr[1] = 12; // Include i=12
      MYPRINTF(_T("\nEntered my_test_model 80"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cyl_on_box1.smb"),
                        1.0e-4,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));  // Cliff-rollover Test
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 29);
    }

  if (lCount == 405) 
    {
      double dBallRadius = 4.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 12; aIndexArr[1] = 12; // Include i=12
      MYPRINTF(_T("\nEntered my_test_model 81"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cyl_on_box1.smb"),
                        1.0e-4,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));  // Cliff-rollover Test
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 17); // was 19 - 17 seems okay
    }

  if (lCount == 406) 
    {
      double dBallRadius = 25.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 15; aIndexArr[1] = 15; // Include i=15
      SmBoolean bDoGlobalMerge = TRUE;
      MYPRINTF(_T("\nEntered my_test_model 82"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/2cyls_on_box.smb"),
                        1.0e-4,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        bDoGlobalMerge,FALSE,FALSE));  // Cliff-rollover Test
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 23);
    }

  if (lCount == 407) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 2; aIndexArr[1] = 2; // Include i=2
      MYPRINTF(_T("\nEntered my_test_model 83"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cyl_on_box_w_fillet.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));  // Rollover Test 1
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
    //SM_ASSERT(lTotalEdges == 26); // 26 after CoincidenceCheck overhaul 2/25/13
      SM_ASSERT(lTotalEdges == 27); // back to 27 after bug fix 6/8/23 [OM-83685 230215]
    }

  if (lCount == 408) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 2; aIndexArr[1] = 2; // Include i=2
      MYPRINTF(_T("\nEntered my_test_model 84"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cyl_on_box_w_fillet1.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));  // Rollover Test 2
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 27);
    }

  if (lCount == 409) 
    {
      double dBallRadius = 2.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 6; aIndexArr[1] = 7; // Include i=2
      MYPRINTF(_T("\nEntered my_test_model 85"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cyl_on_box_w_fillet2.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));  // Rollover Test 3
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 40 );
    }

  if (lCount == 410) 
    {
      double dBallRadius = 3.5;
      ULONG lNumIntervals = 3;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 2; aIndexArr[1] = 2; // Include i=2
      aIndexArr[2] = 4; aIndexArr[3] = 4; // Include i=4
      aIndexArr[4] = 6; aIndexArr[5] = 7; // Include i=6,7
      MYPRINTF(_T("\nEntered my_test_model 86A"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/blade_hub_1.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        TRUE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      // SM_ASSERT(lTotalEdges == 131); // before fixes of 01 Apr 08
      SM_ASSERT(lTotalEdges == 128); // 128 after CoincidenceCheck overhaul 2/25/13
    }

  if (lCount == 411) 
    {
      double dBallRadius = 3.0;
      ULONG lNumIntervals = 3;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 6; aIndexArr[1] = 6; // Include i=6
      aIndexArr[2] = 8; aIndexArr[3] = 8; // Include i=8
      aIndexArr[4] =10; aIndexArr[5] =11; // Include i=10,11
      MYPRINTF(_T("\nEntered my_test_model 86B"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/blade_hub_2.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        TRUE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 92); // 92 after CoincidenceCheck overhaul 2/25/13
    }

  if (lCount == 412) 
    {
      double dBallRadius = 1.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =74; aIndexArr[1] =77; // Include i=74,75,76,77
      MYPRINTF(_T("\nEntered my_test_model 87"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/blade_hub_3.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        TRUE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      // SM_ASSERT(lTotalEdges == 140); // before fixes of 01 Apr 08
      SM_ASSERT(lTotalEdges == 137); // 137 after CoincidenceCheck overhaul 2/25/13
    }

  if (lCount == 413) 
    {
      double dBallRadius = 25.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =12; aIndexArr[1] =12; // Include i=12
      MYPRINTF(_T("\nEntered my_test_model 88"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cliff_3.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 20);
    }

  if (lCount == 414) 
    {
      double dBallRadius = 30.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =12; aIndexArr[1] =12; // Include i=12
      MYPRINTF(_T("\nEntered my_test_model 89"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cliff_4.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 17);
    }

  if (lCount == 415) 
    {
      double dBallRadius = 20.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =15; aIndexArr[1] =15; // Include i=15
      MYPRINTF(_T("\nEntered my_test_model 90"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cyl_on_box_w_hole.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 26); // 23 after CoincidenceCheck overhaul 3/1/13
    }

  if (lCount == 416) 
    {
      double dBallRadius = 13.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =10; aIndexArr[1] =10; // Include i=10
      MYPRINTF(_T("\nEntered my_test_model 91"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/rollover_corner.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
    //SM_ASSERT(lTotalEdges == 23); // 23 after CoincidenceCheck overhaul 2/25/13
      SM_ASSERT(lTotalEdges == 24); // back to 24 after bug fix 6/8/23 [OM-83685 230215]
    }

  if (lCount == 417) 
    {
      double dBallRadius = 10.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =17; aIndexArr[1] =17; // Include i=17
      MYPRINTF(_T("\nEntered my_test_model 92"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cyl_on_box_w_tan_hole.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        FALSE,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
    //SM_ASSERT(lTotalEdges == 34); // 34 after CoincidenceCheck overhaul 2/25/13
      SM_ASSERT(lTotalEdges == 35); // back to 35 after bug fix 6/8/23 [OM-83685 230215]
    }

  if (lCount == 418) 
    {
      double dBallRadius = 28.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =12; aIndexArr[1] =12; // Include i=12
      SmBoolean bDoGlobalMerge = TRUE;
      MYPRINTF(_T("\nEntered my_test_model 93"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/box_diff_cyl_big_rad.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        bDoGlobalMerge, FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 20);
    }

  if (lCount == 419) 
    {
      double dBallRadius = 22.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] =20; aIndexArr[1] =20; // Include i=20
      SmBoolean bDoGlobalMerge = TRUE;
      MYPRINTF(_T("\nEntered my_test_model 94"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/box_diff_3cyls.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps,
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        bDoGlobalMerge,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 37) ; // USEDTOBE 37); the 38 now 37
    }

  if (lCount == 420) 
    {
      double dBallRadius = 20.0;
      ULONG lNumIntervals = 1;
      ULONG aIndexArr[MAX_ARR_SIZE];
      aIndexArr[0] = 3; aIndexArr[1] = 3; // Include i=3
      SmBoolean bDoGlobalMerge = TRUE;
      MYPRINTF(_T("\nEntered my_test_model 95"));
      SER(my_test_model(crContext,_T("../../TestFiles/pt_TestFiles/Fillet/cyl_on_box_w_hole.smb"),
                        SM_ZONE_TOL_3D,dBallRadius,aIndexArr,lNumIntervals,
                        INCLUDE_ARRAY,eSolverType,rPartBreps, 
                        FALSE, // bRebuildUVTrimCurves in my_read_brep()
                        bDoGlobalMerge,FALSE,FALSE));
      SmTArray<SmEdge*> sEdges;
      if (rPartBreps.GetSize()> 0)rPartBreps.GetLast()->GetEdges(sEdges);
      ULONG lTotalEdges = sEdges.GetSize();
      SM_ASSERT(lTotalEdges == 27); // 27 after CoincidenceCheck overhaul 3/1/13
    }

  if (lCount == 421)
  {
	  SmBrep* pBrep = new(crContext) SmBrep();
	  pBrep->ReadFromFile(crContext, _T("../../TestFiles/pt_TestFiles/Fillet/blade_fillet_fails.smb"));
	  
	  int mFilletType = 0;
	  double dBallRadius = 5;
	  int mCrossSectionType = 2;
	  double blendValue0 = 1.5;

	  double tol1 = 30.0*SM_PI / 180.0;
	  double tol2 = 2.0*SM_PI / 180.0;
	  double dTolPercentRadius = 0.001; // 0.1 -> new Default value from sm api
	  double dThisApproxTol3d = pBrep->GetTolerance();

	  SmBlendCurveCrossSectionFSG sFSGBlend(blendValue0);
	  // Creates circular or approx circular cross section fillet
	  SmCircularCrossSectionFSG sFSGCircular(TRUE, dTolPercentRadius);
	  SmLinearCrossSectionFSG sFSGLinear;

	  // Specify the Cross Section type
	  SmFilletSurfaceGenerator *pFSG;

	  switch (mCrossSectionType) {
	  case 0: // CROSSSECTION_TYPE_LINEAR:
		  pFSG = &sFSGLinear;
		  break;
	  case 1: //CROSSSECTION_TYPE_BLEND:
		  pFSG = &sFSGBlend;
		  break;
	  default:
		  pFSG = &sFSGCircular;
	  }

	  double dStepBackFactor = 1.0;
	  SmMakeSurfaceBlendSIH sSIH(dStepBackFactor);

	  SmTArray<SmEdge*> sEdges;
	  SmTArray<SmEdge*> sEdgesForFillet;
	  pBrep->GetEdges(sEdges);

	  // add edges for current case
	  sEdgesForFillet.Add(sEdges[7]);
	  sEdgesForFillet.Add(sEdges[6]);
	  sEdgesForFillet.Add(sEdges[5]);
	  sEdgesForFillet.Add(sEdges[9]);
	  sEdgesForFillet.Add(sEdges[8]);

#ifdef SM_DEBUG_CODE
	  SmBoolean bDebugMe = FALSE;
	  // draw 
	  if (bDebugMe) {
		  if (smGet_DoGraphics()) {
			  pBrep->Draw(); sm_GraphicsLoop();
			  smgfx_SetLook(4, 4, 1, 0, 0); sEdges[7]->Draw(); sm_GraphicsLoop();
			  smgfx_SetLook(4, 4, 1, 0, 0); sEdges[6]->Draw(); sm_GraphicsLoop();
			  smgfx_SetLook(4, 4, 1, 0, 0); sEdges[5]->Draw(); sm_GraphicsLoop();
			  smgfx_SetLook(4, 4, 1, 0, 0); sEdges[9]->Draw(); sm_GraphicsLoop();
			  smgfx_SetLook(4, 4, 1, 0, 0); sEdges[8]->Draw(); sm_GraphicsLoop();
		  }
	  }
#endif // SM_DEBUG_CODE

	  {
		  SmFilletExecutive *pFilExec = new(crContext)SmFilletExecutive(crContext, pBrep);
		  int sih = 1;
		  if (sih) {
			  pFilExec->SetSelfIntersectionHandler(&sSIH);
		  }
		  pFilExec->SetDoClassification(true);
		  SmObjDelete sClean(pFilExec);
		  for (ULONG i = 0; i< sEdgesForFillet.GetSize(); ++i) {
			  // select edge to fillet
			  SmEdge *pE = sEdgesForFillet[i];
			  SmEdgeuse *pEU = pE->GetBlendEdgeuse();
			  if (!pEU) {
				  continue;
			  }
			  SmFilletSolver* pFS = nullptr;
			  switch (mFilletType) {
			  case 0: //FILLET_TYPE_CONSTANT_RADIUS:
				  pFS = new(crContext)SmConstantRadiusFS(crContext, dThisApproxTol3d, tol1, tol2, dBallRadius, pEU);
				  break;
			  case 1: //FILLET_TYPE_CONSTANT_DISTANCE:
				  pFS = new(crContext)SmConstantDistanceFS(crContext, dThisApproxTol3d, tol1, tol2, dBallRadius, pEU);
				  break;
			  }
			  pFS->SetFilletSurfaceGenerator(pFSG);
			  pFilExec->LoadFilletSolver(pFS);
		  }
		  SmStatus stat = pFilExec->CreateFilletCorners();
		  if (stat == SM_SUCCESS) {
			  stat = pFilExec->DoFilleting();
			  
			  if (stat == SM_SUCCESS) {
				  // set output
				  rPartBreps.RemoveAll();
				  rPartBreps.Add(pBrep);
			  }
		  }
#ifdef SM_DEBUG_CODE
		  if (bDebugMe) 
      {
			     if (smGet_DoGraphics())
          { smgfx_Erase(); pBrep->Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop(); 
          }
		    }
#endif // SM_DEBUG_CODE
	  }
	  
	  if (rPartBreps.GetSize() > 0) {
		  sEdges.RemoveAll();
		  rPartBreps.GetLast()->GetEdges(sEdges);
	  }
	  ULONG lTotalEdges = sEdges.GetSize();
	  SM_ASSERT(lTotalEdges == 39);
  } // end if (lCount == 421) check

  // done with case - Dump and Draw (when Compiled with SM_GFX_CODE)
  if (rPartBreps.GetSize() > 0)
  {
      SmBrep *pBrepToFillet = rPartBreps.GetLast();
      pBrepToFillet->Dump();
#ifdef SM_GFX_CODE
      if (smGet_DoGraphics()) {
          smgfx_Erase();
          smgfx_SetLook(1, 2); if (pBrepToFillet) pBrepToFillet->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // end SM_GFX_CODE
    }  

  if (lCount >= lLastTestCount)
    {
       rbIsDone = TRUE;
    }

  return SM_SUCCESS;

} // end my_test_suite_4


// Example of usage of 'sm_PrintTime':
//    clock_t start = clock();
//    NL_SER(N_FitSrfToVariablePts(P,r,m,degu,degv,dTolerance,dPercentTolU,
//        dPercentTolV,dPercentTolKnotsRemoval,
//        NULL,NULL,&sur,&SG,&SG));
//    clock_t finish = clock();
//    sm_PrintTime(_T("Time for approximation"),start,finish);
//

 
//#include <SmPoly.h>
//#include <SmPolySolver.h>
//#include <SmPolyBrepData.h>
//#include <SmPolyDecimate.h>
//#include <SmPolyMerge.h>
#include <SmMerge.h>
#include <SmPrimitiveCreation.h>
#include <SmTrimmingTools.h>

/*******************************************************
PURPOSE --- 

USAGE NOTES --- Used by prog_test
  calls my_test_suite_0
        my_test_suite_1
        my_test_suite_2
        my_test_suite_3
        my_test_suite_4
  repeatedly until all cases in each suite have been run
********************************************************/
PT_EXPORT SmStatus my_test_fillet_regression
(
  SmFilletSolverType eSolverType   //      SM_FS_CONST_RADIUS    = build constant radius fillets
                                   //      SM_FS_CONST_DIST      = build constant distance fillets
                                   //      SM_FS_VARIABLE_RADIUS = build variable radius fillets
)    
{
  MYPRINTF(_T("\nEntered my_test_fillet_regression")) ;

  SmContext sContext;
  SmTArray<SmBrep*> sBreps;
  ULONG lCount     = 0;

  // iterate enough times to exercise each case in each my_test_suite_X function.
  for (ULONG i=0; i<500; i++) 
    {
      // repeatedly call the same test_suite until that test_suite says its done all its cases
      ULONG lCase = 0;
      if (lCount >= 100) lCase = 1;
      if (lCount >= 200) lCase = 2;
      if (lCount >= 300) lCase = 3;
      if (lCount >= 400) lCase = 4;

      SmBoolean bIsDone     = FALSE;

      // switch between test suite cases
      switch (lCase) 
        {
          case 0:
              bIsDone = FALSE;
              my_test_suite_0(sContext,sBreps,lCount,eSolverType,bIsDone);
              if (bIsDone) lCount = 100;
              break;
          case 1:
              bIsDone = FALSE;
              my_test_suite_1(sContext,sBreps,lCount,eSolverType,bIsDone);
              if (bIsDone) lCount = 200;
              break;
          case 2:
              bIsDone = FALSE;
              if (eSolverType == SM_FS_CONST_DIST) { // Jump over to suite#4
                                                     lCount = 400;
                                                     break;
                                                   }
              my_test_suite_2(sContext,sBreps,lCount,eSolverType,bIsDone);
              if (bIsDone) lCount = 300;
              break;
          case 3:
              bIsDone = FALSE;
              my_test_suite_3(sContext,sBreps,lCount,bIsDone);
              if (bIsDone) lCount = 400;
              break;
          case 4:
              bIsDone = FALSE;
              my_test_suite_4(sContext,sBreps,lCount,eSolverType,bIsDone);
              if (bIsDone) 
                { return SM_SUCCESS; }
              break;
          default:
              break;
        }  // end switch on lCase

//cbi: look at last one:
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
        if (bDebugMe && smGet_DoGraphics())
          { sm_GraphicsLoop(); }
#endif // SM_DEBUG_CODE

      DELETE_ALL_PARTS( sBreps );

    } // end iter all cases

  return SM_SUCCESS;

} // end my_test_fillet_regression

/*******************************************************
PURPOSE --- 

USAGE NOTES ---
********************************************************/
PT_EXPORT SmStatus my_local_op_regression()
{
    MYPRINTF(_T("\nEntered: my_local_op_regression")) ;

    SmContext sContext;
    TCHAR sBuff[SM_TBLOCK_SIZE] ;
    ULONG lCount=0;
    for (ULONG i=0; i<8; i++) {
        SM_SPRINTF(sBuff, _T("\nEntered my_local_op iter: %ld"),i) ;
        MYPRINTF(sBuff);
        my_local_op(sContext, lCount);
    }

    return SM_SUCCESS;

} // end my_local_op_regression

/*******************************************************
PURPOSE --- 

USAGE NOTES ---
********************************************************/
PT_EXPORT SmStatus my_local_op( const SmContext & crContext, ULONG & rlCount )
{
  MYPRINTF( _T( "\nEntered my_local_op" ) );

  SmVector3d sX( 1.0, 0.0, 0.0 );
  SmVector3d sY( 0.0, 1.0, 0.0 );
  SmVector3d sZ( 0.0, 0.0, 1.0 );
  SmAxis2Placement sPos;

  ULONG lLastTestCount = 7;
  ULONG lCount = rlCount;
  rlCount++;

  if(lCount == 0)
  {
    SmBrep *pBrep = new (crContext) SmBrep();
    SmObjDelete sClean(pBrep);
    MYPRINTF( _T( "\nReading test file ../../TestFiles/pt_TestFiles/Fillet/nurb_box.smb" ) );
    SER( pBrep->ReadFromFile( crContext, _T( "../../TestFiles/pt_TestFiles/Fillet/nurb_box.smb" ) ) );

    if(smGet_DoGraphics())
    {
      smgfx_Erase();
      pBrep->Draw(); 
      sm_GraphicsLoop();
    }
  }

  if(lCount == 1)
  {
    SmBrep *pBrep = new (crContext) SmBrep();
    SmObjDelete sClean(pBrep);
    MYPRINTF( _T( "\nReading test file ../../TestFiles/pt_TestFiles/Fillet/nurb_box.smb" ) );
    SER( pBrep->ReadFromFile( crContext, _T( "../../TestFiles/pt_TestFiles/Fillet/nurb_box.smb" ) ) );

    SmTArray<SmFace*> sFaces;
    pBrep->GetFaces( sFaces );
    SmFace * pFace = sFaces[1];
    SmTArray<SmFace*> sNewFaces;
    sNewFaces.Add( pFace );

    SmVector3d sYZ( 0.0, 1.0, 1.0 );
    sYZ.Unitize();
    sPos.SetCanonical( SmPoint3d( 0.0, 0.0, 12.0 ), sX, sYZ );
    SmPlane * pNewPlane = NULL;
    SER( SmPlane::CreateCanonical( crContext, sPos, pNewPlane ) );
    SmPoint2d sUVMin( -15.0, -15.0 );
    SmPoint2d sUVMax( 25.0, 25.0 );
    SmExtent2d sAnalDomain( sUVMin, sUVMax );
    SER( pNewPlane->AdjustSTEPUVDomain( sAnalDomain ) );
    SmTArray<SmSurface*> sNewSurfaces;
    sNewSurfaces.Add( pNewPlane );

#ifdef SM_DEBUG_CODE
    if(smGet_DoGraphics())
    {
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) pBrep->Draw( ); sm_GraphicsLoop();
      smgfx_SetLook( 1, 2, 0, 0, 0 ); sNewFaces[0]->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook( 1, 2, 0, 1, 0 ); sNewSurfaces[0]->DrawUV( 10, 10 ); sm_GraphicsLoop();
      sm_GraphicsLoop();

    }
#endif // SM_DEBUG_CODE && SM_GFX_CODE

    SER( pBrep->LocalOperation( sNewFaces, sNewSurfaces ) );

#ifdef SM_DEBUG_CODE
    if(smGet_DoGraphics())
    {
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) pBrep->Draw( ); sm_GraphicsLoop();
      // stalePtr     smgfx_SetLook(1,2, 0,0,0) ; sNewFaces[0]->DrawUV() ; sm_GraphicsLoop() ;
      // stalePtr     smgfx_SetLook(1,2, 0,1,0) ; sNewSurfaces[0]->DrawUV(10,10); sm_GraphicsLoop() ;
      sm_GraphicsLoop();

    }
#endif // SM_DEBUG_CODE

  }

  if(lCount == 2)
  {
    SmBrep *pBrep = new (crContext) SmBrep();
    SmObjDelete sClean(pBrep);
    MYPRINTF( _T( "\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb" ) );
    SER( pBrep->ReadFromFile( crContext, _T( "../../TestFiles/pt_TestFiles/Solids/box.smb" ) ) );
    if(smGet_DoGraphics())
    {
      smgfx_Erase();
      pBrep->Draw();
      sm_GraphicsLoop();
    }
  }

  if(lCount == 3)
  {
    // cylinder with box
    SmBrep *pBrep = new (crContext) SmBrep();
    SmObjDelete sClean(pBrep);
    MYPRINTF( _T( "\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb" ) );
    SER( pBrep->ReadFromFile( crContext, _T( "../../TestFiles/pt_TestFiles/Solids/box.smb" ) ) );

    SmTArray<SmFace*> sFaces;
    pBrep->GetFaces( sFaces );
    SmFace * pFace = sFaces[1];
    SmTArray<SmFace*> sNewFaces;
    sNewFaces.Add( pFace );

    sPos.SetCanonical(SmPoint3d(5.0,0.0,12.0),-sX,sZ);
    SmCylinder * pNewCylinder = NULL;
    SER(SmCylinder::CreateCanonical(crContext,sPos,5.0,pNewCylinder));
    SmPoint2d sUVMin = SmPoint2d(0.0,0.0);
    SmPoint2d sUVMax = SmPoint2d(180.0,10.0);
    SmExtent2d sAnalDomain(sUVMin,sUVMax);
    SER(pNewCylinder->AdjustSTEPUVDomain(sAnalDomain));
    SmTArray<SmSurface*> sNewSurfaces;
    sNewSurfaces.Add(pNewCylinder);

#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if(bDebugMe && smGet_DoGraphics())
    {
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) pBrep->Draw( ); sm_GraphicsLoop();
      smgfx_SetLook( 1, 2, 0, 0, 0 ); sNewFaces[0]->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook( 1, 2, 0, 1, 0 ); sNewSurfaces[0]->DrawUV( 10, 10 ); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE && SM_GFX_CODE

    // needs fixing
    SER(pBrep->LocalOperation(sNewFaces,sNewSurfaces));

#ifdef SM_DEBUG_CODE
    if(bDebugMe && smGet_DoGraphics())
    {
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) pBrep->Draw( ); sm_GraphicsLoop();
      // stalePtr     smgfx_SetLook(1,2, 0,0,0) ; sNewFaces[0]->DrawUV() ; sm_GraphicsLoop() ;
      // stalePtr     smgfx_SetLook(1,2, 0,1,0) ; sNewSurfaces[0]->DrawUV(10,10); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif
  }

  if(lCount == 4)
  {
    SmBrep *pBrep = new (crContext) SmBrep();
    SmObjDelete sClean(pBrep);
    MYPRINTF( _T( "\nReading test file ../../TestFiles/pt_TestFiles/Solids/cylinder.smb" ) );
    SER( pBrep->ReadFromFile( crContext, _T( "../../TestFiles/pt_TestFiles/Solids/cylinder.smb" ) ) );
#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if(bDebugMe && smGet_DoGraphics())
    {
      smgfx_Erase();
      pBrep->Draw(); 
      sm_GraphicsLoop();
    }
#endif
  }

  if(lCount == 5)
  {

    // plane with cylinder
    SmBrep *pBrep = new (crContext) SmBrep();
    SmObjDelete sClean(pBrep);
    MYPRINTF( _T( "\nReading test file ../../TestFiles/pt_TestFiles/Solids/cylinder.smb" ) );
    SER( pBrep->ReadFromFile( crContext, _T( "../../TestFiles/pt_TestFiles/Solids/cylinder.smb" ) ) );

    SmTArray<SmFace*> sFaces;
    pBrep->GetFaces( sFaces );
    SmFace * pFace = sFaces[1];
    SmTArray<SmFace*> sNewFaces;
    sNewFaces.Add( pFace );

    sPos.SetCanonical(SmPoint3d(-3.0,0.0,5.0),-sY,sZ);
    SmPlane * pNewPlane = NULL;
    SER(SmPlane::CreateCanonical(crContext,sPos,pNewPlane));
    SmPoint2d sUVMin(-5.0,-10.0);
    SmPoint2d sUVMax(5.0,0.0);
    SmExtent2d sAnalDomain(sUVMin,sUVMax);
    SER(pNewPlane->AdjustSTEPUVDomain(sAnalDomain));
    SmTArray<SmSurface*> sNewSurfaces;
    sNewSurfaces.Add(pNewPlane);

#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE;
    if(bDebugMe && smGet_DoGraphics())
    {
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) pBrep->Draw( ); sm_GraphicsLoop();
      smgfx_SetLook( 1, 2, 0, 0, 0 ); sNewFaces[0]->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook( 1, 2, 0, 1, 0 ); sNewSurfaces[0]->DrawUV( 10, 10 ); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

    // Needs fixing
    SER(pBrep->LocalOperation(sNewFaces,sNewSurfaces));

#ifdef SM_DEBUG_CODE
    if(bDebugMe && smGet_DoGraphics())
    {
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) pBrep->Draw( ); sm_GraphicsLoop();
      // stalePtr     smgfx_SetLook(1,2, 0,0,0) ; sNewFaces[0]->DrawUV() ; sm_GraphicsLoop() ;
      // stalePtr     smgfx_SetLook(1,2, 0,1,0) ; sNewSurfaces[0]->DrawUV(10,10); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif

  }

  if(lCount == 6)
  {
    SmBrep *pBrep = new (crContext) SmBrep();
    SmObjDelete sClean(pBrep);
    MYPRINTF( _T( "\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb" ) );
    SER( pBrep->ReadFromFile( crContext, _T( "../../TestFiles/pt_TestFiles/Solids/box.smb" ) ) );

#ifdef SM_DEBUG_CODE
    if(smGet_DoGraphics())
    {
      smgfx_Erase();
      pBrep->Draw(); 
      sm_GraphicsLoop();
    }
#endif
  }

  if(lCount == 7)
  {

    // nurb-plane with box
    SmBrep *pBrep = new (crContext) SmBrep();
    SmObjDelete sClean(pBrep);
    MYPRINTF( _T( "\nReading test file ../../TestFiles/pt_TestFiles/Solids/box.smb" ) );
    SER( pBrep->ReadFromFile( crContext, _T( "../../TestFiles/pt_TestFiles/Solids/box.smb" ) ) );

    SmTArray<SmFace*> sFaces;
    pBrep->GetFaces( sFaces );
    SmFace * pFace = sFaces[4];
    SmTArray<SmFace*> sNewFaces;
    sNewFaces.Add( pFace );
    SmExtent2d sOrigDomain = pFace->GetSurface()->GetNaturalUVDomain();

    SmBSplineSurface * pInputSurface = NULL;
    MYPRINTF( _T( "\nReading test file ../../TestFiles/pt_TestFiles/offset_nurb.sms" ) );
    SER( SmBSplineSurface::ReadFromFile( crContext, _T( "../../TestFiles/pt_TestFiles/offset_nurb.sms" ), pInputSurface ) );
    pInputSurface->Reparameterize( sOrigDomain );
    SmTArray<SmSurface*> sNewSurfaces;
    sNewSurfaces.Add( pInputSurface );

#ifdef SM_DEBUG_CODE
    if(smGet_DoGraphics())
    {
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) pBrep->Draw( ); sm_GraphicsLoop();
      smgfx_SetLook( 1, 2, 0, 0, 0 ); sNewFaces[0]->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook( 1, 2, 0, 1, 0 ); sNewSurfaces[0]->DrawUV( 10, 10 ); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif 

    SER( pBrep->LocalOperation( sNewFaces, sNewSurfaces ) );

#ifdef SM_DEBUG_CODE
    if(smGet_DoGraphics())
    {
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pBrep) pBrep->Draw( ); sm_GraphicsLoop();
      // stalePtr     smgfx_SetLook(1,2, 0,0,0) ; sNewFaces[0]->DrawUV() ; sm_GraphicsLoop() ;
      // stalePtr     smgfx_SetLook(1,2, 0,1,0) ; sNewSurfaces[0]->DrawUV(10,10); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif 

  }

  if(lCount >= lLastTestCount)
    rlCount = 0;

  return SM_SUCCESS;

} // end my_local_op

/***********************************************************************
PURPOSE --- Read a FilletSurfaceGenerator description line from file,
  and return a new SmFilletSurfaceGenerator of appropriate type.

USAGE NOTES ---

  my_read_fsg() block 
    line 1 - { CrossSectionType, RelativeAccuracy, Mirror, Complement }

     where CrossSectionType == 1 returns an SmLinearCrossSectionFSG   object
           CrossSectionType == 2 returns an SmCircularCrossSectionFSG object
           CrossSectionType == 3 returns an SmCircularCrossSectionFSG object

  GWC NOTE: should CrossSectionType == 3 be changed to return a
    SmBlendCurveCrossSectionFSG object
***********************************************************************/
static SmFilletSurfaceGenerator * my_read_fsg
  (const SmContext & crContext,    // in : context for new object construction
   FILE *pFile)                    // in : target file
{
  // read line 1 - { CrossSectionType, RelativeAccuracy, Mirror, Complement }
  ULONG lCrossSectionType;
  double dRelativeAccuracy;
  ULONG lMirror, lComplement;
  int result = SM_FSCANF(pFile,_T("%ld %lf %ld %ld"),&lCrossSectionType,&dRelativeAccuracy,
      &lMirror,&lComplement);
  if (result != 4) {
    MYPRINTF(_T("Error reading fsg\n"));
    return NULL;
  }
  NERN(my_goto_next_line_null(pFile));
  SmBoolean bMirror     = (SmBoolean)lMirror;
  SmBoolean bComplement = (SmBoolean)lComplement;

  // create new FilletSurfaceGenerator - branch on lCrossSectionType
  if (lCrossSectionType == 1) 
    {
      SmLinearCrossSectionFSG *pFSGLinear = new(crContext) SmLinearCrossSectionFSG();
      return pFSGLinear;
    }
  if (lCrossSectionType == 2) 
    {
      SmBoolean bApprox = FALSE;
      SmCircularCrossSectionFSG *pFSGCircular = new(crContext) 
          SmCircularCrossSectionFSG(bApprox,dRelativeAccuracy,bMirror,bComplement);
      return pFSGCircular;
    }
  if (lCrossSectionType == 3) 
    {
      // gwc??? should CrossSectionType == 3 be changed to return a
      // SmBlendCurveCrossSectionFSG object

      SmBoolean bApprox = TRUE;
      SmCircularCrossSectionFSG *pFSGCircular = new(crContext)
          SmCircularCrossSectionFSG(bApprox,dRelativeAccuracy,bMirror,bComplement);
      return pFSGCircular;
    }

  // arrive here for unknonw lCrossSectionTypes
  SE(SM_ERR);

  return NULL;

} // end SmFilletSurfaceGenerator


/*******************************************************
PURPOSE ---  Build and return an SmFilletSolver from a
    FilletSolver description.  The description can be
    read from file or an input character buffer.

USAGE NOTES ---

  The function can either
    1: read file         - create and return an SmFilletSolver
    2: read file         - store data in output Buffer, return NULL
    3: read input buffer - create and return an SmFilletSolver
    4: read input buffer - store data in output Buffer, return NULL

  line 1 - { FilletTolerance, FilletSolverType, distance        } when FilletSolverType == 1
  line 1 - { FilletTolerance, FilletSolverType, radius          } when FilletSolverType == 2
  line 1 - { FilletTolerance, FilletSolverType, lawType, Orient } when FilletSolverType == 3 && lawType == 1
  line 1 - { FilletTolerance, FilletSolverType, lawType, Orient, Start Radius, End Radius } when FilletSolverType == 3 && lawType == 2

  when FilletSolverType == 3 && lawType == 2 
    { line 2                          = { Number of Control Points }
      line 3 to 2+NUmberControlPoints = { controlPoint.x controlPoint.y }
    }

  where FilletSolverType == 1, return SmConstantDistanceFS object
        FilletSolverType == 2, return SmConstantRadiusFS   object 
        FilletSolverType == 3, return SmVariableRadiusFS   object

        lawType == 1 variable radius blend with a linear  law
        lawType == 2 variable radius blend with a BSpline law
  
********************************************************/
static SmFilletSolver * my_read_edge_fillet_solver
  (const SmContext & crContext,       // in : context for new object construction
   FILE * pFile,                      // in : target file to read
   SmEdge *pEdge,                     // in : Edge to fillet
   SmTArray<SmFilletLaw*> & rLaws,    // out:
   TCHAR pBuff[256][256],             // in : NULL    = read from pFile
                                      //      notNULL = read from pBuff 
   TCHAR pOutBuff[256][256])          // out: notNULL = copy pFile reads into pOutBuff - return NULL
                                      //      NULL    = use reads to create and return new SmFilletSolver
{ 
  // Find an edgeuse on the inside of the solid with the same orientation
  // as the curve of the edge.
  SmEdgeuse *pEU = NULL ;
  if (!pOutBuff) 
    {
      pEU = pEdge->GetBlendEdgeuse();
      if (!pEU) return (NULL);
    }

  // read FilletTolerance, FilletSolverType from pFile or pBuff
  ULONG  lFilletSolverType;
  double dFilletTolerance;
  if (pBuff) { int result = SM_SSCANF(pBuff[0],_T("%lf %ld"), &dFilletTolerance, &lFilletSolverType);
             if (result != 2) {
               MYPRINTF(_T("Error reading fillet tolerance and solver type\n"));
               return NULL;
             }
             }
  else       { int result = SM_FSCANF(pFile,   _T("%lf %ld"), &dFilletTolerance, &lFilletSolverType);
               if (result != 2) {
                 MYPRINTF(_T("Error reading fillet tolerance and solver type\n"));
                 return NULL;
               }
               if (pOutBuff) 
                 { SM_SPRINTF(pOutBuff[0],_T("%16.16lf %ld"),dFilletTolerance, lFilletSolverType);
                 }
             }

  // branch on FilletSolverType and pOutBuff value to process SmFilletSolver read
  if (lFilletSolverType == 1) // Constant distance
    { 
      // read distance
      double dDistance;
      if (pBuff) { int result = SM_SSCANF(pBuff[1],_T("%lf"),&dDistance);
                 if (result != 1) {
                   MYPRINTF(_T("Error reading distance\n"));
                   return NULL;
                 }
                 }
      else       { int result = SM_FSCANF(pFile,_T("%lf"),&dDistance);
                   if (result != 1) {
                     MYPRINTF(_T("Error reading distance\n"));
                     return NULL;
                   }
                 NERN(my_goto_next_line_null(pFile));
                } 

      // when pOoutBuff != NULL - store distance, return NULL
      if (pOutBuff) 
        { 
          SM_SPRINTF(pOutBuff[1],_T("%16.16lf"),dDistance);
          return NULL;
        }
      else // create and return new SmConstantDistanceFS object
        {
          if (!pEU) return NULL;
          SmConstantDistanceFS * pFS = new(crContext) SmConstantDistanceFS(crContext,dFilletTolerance,
                                                                           30.0*SM_PI/180.0, 
                                                                           2.0*SM_PI/180.0, 
                                                                           dDistance, 
                                                                           pEU);
          if (pFS->GetEdgeuse(0) == NULL) 
            {
              SM_ASSERT(pFS != NULL) ; delete pFS ; pFS = NULL ;
            }
          return pFS;
        }
    } // end (lFilletSolverType == 1) branch

  if (lFilletSolverType == 2)  // Constant radius fillet solver
    {
      // read Radius
      double dRadius;
      if (pBuff) { int result = SM_SSCANF(pBuff[2],_T("%lf"),&dRadius);
                 if (result != 1) {
                   MYPRINTF(_T("Error reading radius\n"));
                   return NULL;
                 }
                 }
      else       { int result = SM_FSCANF(pFile,_T("%lf"),&dRadius);
                   if (result != 1) {
                     MYPRINTF(_T("Error reading radius\n"));
                     return NULL;
                   }
                 NERN(my_goto_next_line_null(pFile));
                 }

      // when pOoutBuff != NULL - store distance, return NULL
      if (pOutBuff) 
        {
          SM_SPRINTF(pOutBuff[2],_T("%16.16lf"),dRadius);
          return NULL;
        }
      else // create and return new SmConstantRadiusFS object
        {
          if (!pEU) return NULL;
          SmConstantRadiusFS * pFS = new(crContext) SmConstantRadiusFS(crContext,
                                                                       dFilletTolerance,
                                                                       30.0*SM_PI/180.0, 
                                                                       2.0*SM_PI/180.0, 
                                                                       dRadius, 
                                                                       pEU);
          if (pFS->GetEdgeuse(0) == NULL) 
            { SM_ASSERT(pFS != NULL) ; delete pFS ; pFS = NULL ;
            }
          return pFS;
        }
    } // end (lFilletSolverType == 2) branch

  if (lFilletSolverType == 3) // Variable radius fillet solver
    { 
      // read LawType and Orient 
      ULONG lLawType, lOrient;
      if (pBuff) { int result = SM_SSCANF(pBuff[3],_T("%ld %ld"),&lLawType,&lOrient);
                 if (result != 2) {
                   MYPRINTF(_T("Error reading law type and orient\n"));
                   return NULL;
                 }
                 }
      else       { int result = SM_FSCANF(pFile,_T("%ld %ld"),&lLawType,&lOrient);
                   if (result != 2) {
                     MYPRINTF(_T("Error reading law type and orient\n"));
                     return NULL;
                   }
                }
      if (pOutBuff) SM_SPRINTF(pOutBuff[3],_T("%ld %ld"),lLawType,lOrient);

      SmBoolean bOrientation =   (lOrient == 1)
                               ? TRUE
                               : FALSE;

      // switch on LawType to create SmFilletLaw
      SmFilletLaw        *pLaw = NULL;
      double              dStartRadius, dEndRadius;
      ULONG               lNumControlPoints, i;
      SmTArray<SmPoint3d> sPnts;
      switch(lLawType)
        {
          case 1 : if (pBuff) { int result = SM_SSCANF(pBuff[4],_T("%lf %lf"),&dStartRadius,&dEndRadius);
                                if (result != 2) {
                                  MYPRINTF(_T("Error reading start radius and end radius\n"));
                                  return NULL;
                                }
                              }
                   else       { int result = SM_FSCANF(pFile,_T("%lf %lf"),&dStartRadius,&dEndRadius);
                                if (result != 2) {
                                  MYPRINTF(_T("Error reading start radius and end radius\n"));
                                  return NULL;
                                }
                                NERN(my_goto_next_line_null(pFile));
                              }

                   if (pOutBuff) { SM_SPRINTF(pOutBuff[4],_T("%16.16lf %16.16lf"),dStartRadius,dEndRadius);
                                 }
                   else          { if (pEU)
                                     pLaw = new(crContext) SmLinearFilletLaw(dStartRadius,dEndRadius);
                                 }
                   break ;

          case 2 : if (pBuff) { int result = SM_SSCANF(pBuff[5],_T("%ld"),&lNumControlPoints);
                                if (result != 1) {
                                  MYPRINTF(_T("Error reading number of control points\n"));
                                  return NULL;
                                }
                              }
                   else       { NERN(my_goto_next_line_null(pFile));
                                int result = SM_FSCANF(pFile,_T("%ld"),&lNumControlPoints);
                                if (result != 1) {
                                  MYPRINTF(_T("Error reading number of control points\n"));
                                  return NULL;
                                }
                                NERN(my_goto_next_line_null(pFile));
                              }

                   if (pOutBuff) { SM_SPRINTF(pOutBuff[5],_T("%ld"),lNumControlPoints);
                                 }

                   for (i=0; i<lNumControlPoints; i++) 
                     {
                       SmPoint3d sPnt(0,0,0);
                       if (pBuff) { int result = SM_SSCANF(pBuff[6+i],_T("%lf %lf"),&sPnt.x,&sPnt.y);
                                  if (result != 2) {
                                    MYPRINTF(_T("Error reading control point\n"));
                                    return NULL;
                                  }
                                  }
                       else       { int result = SM_FSCANF(pFile,_T("%lf %lf"),&sPnt.x,&sPnt.y);
                                    if (result != 2) {
                                      MYPRINTF(_T("Error reading control point\n"));
                                      return NULL;
                                    }
                                    NERN(my_goto_next_line_null(pFile));
                                  }
                       if (pOutBuff) { SM_SPRINTF(pOutBuff[6+i],_T("%16.16lf %16.16lf"),sPnt.x,sPnt.y);
                                     }
                       sPnts.Add(sPnt);
                     }

                   if (!pOutBuff && pEU) 
                     {
                       SmBSplineCurve *pNurb   = my_create_nurb(crContext,SmPoint3d(0,0,0),sPnts,3,0,0,0);
                       SmExtent1d      sCrvIvl = pNurb->GetNaturalInterval();
                       SmExtent1d      sIvl(sCrvIvl.Evaluate(0.25),sCrvIvl.Evaluate(0.75));
                       pLaw = new(crContext) SmBSplineFilletLaw(pNurb,sIvl);
                     }
                   break ;

          default : SE(SM_ERR); 
                    return NULL; 
      
        } // end switch on lLawType

      // don't build an SmFilletSolver when
      //     pOoutBuff is given
      //  or we can't find an edgeuse to fillet
      //  or pLaw construction failed
      if (   pOutBuff != NULL
          || pEU      == NULL) { return NULL; }
      if (   pLaw     == NULL) { SE(SM_ERR); return NULL; }

      // remember the pLaw and build and return the SmFilletSolver 
      rLaws.Add(pLaw);
      SmVariableRadiusFS *pFS = new(crContext) SmVariableRadiusFS(crContext,
                                                                  dFilletTolerance,
                                                                  30.0*SM_PI/180.0, 
                                                                  2.0*SM_PI/180.0, 
                                                                  1.0, 
                                                                  pEU, 
                                                                  *pLaw, 
                                                                  bOrientation);
      if (pFS->GetEdgeuse(0) == NULL) 
        { SM_ASSERT(pFS != NULL) ; delete pFS ; pFS = NULL ;
        }
      return pFS;
  } // end (lFilletSolverType == 3) branch
     
  return NULL;

} // end my_read_edge_fillet_solver

/*******************************************************
PURPOSE --- Build a FilletSolver using a single FilletSolver
  description for every edge in the input rEdges edge array.

USAGE NOTES ---
  When aInBuff == NULL read one FilletSolver description from pFile
  when aInBuff != NULL read one FilletSolver description from aInBuff

********************************************************/
static SmStatus my_read_edges_fillet_solver
  (const SmContext & crContext,                  // in : context for new object construction
   FILE * pFile,                                 // in : target file to read
   SmTArray<SmEdge*> & rEdges,                   // in : array of edges to get FilletSolvers
   SmTArray<SmFilletSolver*> & rFilletSolvers,   // out: one FilletSolver for each rEdges edge
   SmTArray<SmFilletLaw*> & rLaws,               // in : array of Laws
   TCHAR aInBuff[256][256])                      // in : NULL   =
                                                 //      notNULL= 
{
  // init output
  rFilletSolvers.ReSet();

  // locals
  TCHAR aOutBuff[256][256];

  // Find an edgeuse on the inside of the solid with the same orientation
  // as the curve of the edge.

  // when aInBuff is NULL - read from file
  if (!aInBuff) 
    {
      // read file FilletSolver description into aOutBuff
      my_read_edge_fillet_solver(crContext,pFile,NULL,rLaws,NULL,aOutBuff);

      // for every edge - build a FilletSolver without reading the input file
      for (ULONG i=0; i<rEdges.GetSize(); i++) 
        {
          SmFilletSolver *pFS = my_read_edge_fillet_solver(crContext,pFile,rEdges[i],rLaws,aOutBuff,NULL);
          if (pFS) { rFilletSolvers.Add(pFS);
                   }
        }
    }
  else // read from aInBuff
    {
      // for every edge - build a FilletSolver without reading the input file 
      for (ULONG i=0; i<rEdges.GetSize(); i++) 
        {
          SmFilletSolver *pFS = my_read_edge_fillet_solver(crContext,pFile,rEdges[i],rLaws,aInBuff,NULL);
          if (pFS) { rFilletSolvers.Add(pFS);
                   }
        }
    }

  return SM_SUCCESS;

} // end my_read_edges_fillet_solver

/*******************************************************
PURPOSE --- read from file and execute a sequence of fillet tests. 

USAGE NOTES ---
  
  Fillet test file structure

  line 1 - { comment }
  line 2 - { Number Filleting Operations }
  
  for each Filleting Operation
    { line 1 - comment
      line 2 - comment
      line 3 - comment
      line 4 - { FileName containing Brep model(s) to read into sFileBreps }
      line 5 - { FilletingOption, TotalEdges, Translation Vector }

      if FilletingOption == 1
        { line 1 = { offset1, offset2, tolerance, boundaryTrimmingType, ReverseTrimFlag }
          line 2 = { CrossSectionType, RelativeAccuracy, MirrorFlag, Complement }
        }
      else if FilletingOption == 2
        {
          line 1 = { Number Topology Fillet Definitions }
            for each topology fillet definition
              { line 1 = { comment }
                line 2 = { lTopologyType }
                   case lTopologyType == 0 - no lines
                   case lTopologyType == 1 - { line 1 = { edge index }
                                               my_read_edge_fillet_solver() block
                                               my_read_fsg() block
                                             }
                   case lTopologyType == 2 - { line 1 = { face index }
                                               my_read_edge_fillet_solver() block
                                               my_read_fsg() block
                                             }
                   case lTopologyType == 3   { my_read_edge_fillet_solver() block
                                               my_read_fsg() block
                                             }
                   case lTopologyType == 4 - no lines
                   case lTopologyType == 5 - no lines
                   case lTopologyType == 6 - no lines
                   case lTopologyType == 7 - no lines
                   case lTopologyType == 8 - no lines
                   case lTopologyType == 9 - { my_read_edge_fillet_solver() block
                                               my_read_fsg() block
                                             }
              } // end iter every topology fillet definition
        }  // end FilletingOption == 2 branch

  1. my_read_edge_fillet_solver block
       line 1 - { FilletTolerance, FilletSolverType, distance        } when FilletSolverType == 1
       line 1 - { FilletTolerance, FilletSolverType, radius          } when FilletSolverType == 2
       line 1 - { FilletTolerance, FilletSolverType, lawType, Orient } when FilletSolverType == 3 && lawType == 1
       line 1 - { FilletTolerance, FilletSolverType, lawType, Orient, Start Radius, End Radius } when FilletSolverType == 3 && lawType == 2

       when FilletSolverType == 3 && lawType == 2 
         { line 2                          = { Number of Control Points }
           line 3 to 2+NUmberControlPoints = { controlPoint.x controlPoint.y }
         }

       where FilletSolverType == 1, return SmConstantDistanceFS object
             FilletSolverType == 2, return SmConstantRadiusFS   object 
             FilletSolverType == 3, return SmVariableRadiusFS   object

             lawType == 1 variable radius blend with a linear  law
             lawType == 2 variable radius blend with a BSpline law

  2. my_read_fsg() block 
       line 1 - { CrossSectionType, RelativeAccuracy, Mirror, Complement }
        where CrossSectionType == 1 returns an SmLinearCrossSectionFSG   object
              CrossSectionType == 2 returns an SmCircularCrossSectionFSG object
              CrossSectionType == 3 returns an SmCircularCrossSectionFSG object

********************************************************/
PT_EXPORT SmStatus my_read_fillet_definition_file
  (const SmContext & crContext,   // in : context for new object construction
   const TCHAR * cInputFileName,   // in : target file
   SmTArray<SmBrep*> & rBreps)    // out: contains the output Breps
                                  //      FilletingOperation == 1, Surface-based filleting
                                  //         rBreps[0] = pBrep1
                                  //         rBreps[1] = pBrep2
                                  //         rBreps[2] = pResult of filleting 1st face pairs
                                  //      FilletingOperation == 2, Topology Based filleting
                                  //         rBreps[0] to rBreps[n] 
                                  //         rBreps[n+1] = translated rBreps[0] after all rBreps are merged and stitched
{
  // output unique label string
  MYPRINTF(_T("\nEntered my_read_fillet_definition_file ")) ;

  // init output
  DELETE_ALL_PARTS(rBreps);

  // open target file
  FILE *pFile = SM_FOPEN(cInputFileName,_T("r+"));
  if (!pFile) return SM_ERR;
  
  // Read comment line
  SER(my_goto_next_line(pFile));
  
  // Read Number of Filleting Operations.
  ULONG lNumberFilletingOperations;
  int result = SM_FSCANF(pFile,_T("%ld"),&lNumberFilletingOperations);
  if (result != 1) {
    MYPRINTF(_T("Error reading number of filleting operations\n"));
    return SM_ERR;
  }
  SER(my_goto_next_line(pFile));
  
  // for every filleting operation
  for (ULONG i=0; i<lNumberFilletingOperations; i++) 
    {
      // read file filleting operation blocks - 
      SER(my_goto_next_line(pFile));// Comment section - 3 comments
      SER(my_goto_next_line(pFile));
      SER(my_goto_next_line(pFile)); 

      // Read the file name
      TCHAR sFileName[SM_TBLOCK_SIZE];
      result = SM_FSCANF(pFile,_T("%s"),sFileName);
      if (result != 1) {
        MYPRINTF(_T("Error reading file name\n"));
        return SM_ERR;
      }

      SER(my_goto_next_line(pFile)); // Finish reading line

      // read brep from file into sFileBreps - branch on filetype
      SmTArray<SmBrep*> sFileBreps;
      if (   smos_WStrStr(sFileName,_T(".smb")) || smos_WStrStr(sFileName,_T(".SMB"))) 
        { // SM Brep - ASCII
          SmBrep *pBrep = new(crContext) SmBrep();
          SER(pBrep->ReadFromFile(crContext,sFileName,SM_ASCII));
          sFileBreps.Add(pBrep);
        }
      else if (   smos_WStrStr(sFileName,_T(".smp")) || smos_WStrStr(sFileName,_T(".SMP"))) 
        { // SM Part - ASCII
          SmTArray<SmCurve*>   sCurves;
          SmTArray<SmSurface*> sSurfaces;
          SmTArray<long> sTrees;
          SER(SmBrepData::ReadPartFromFile(crContext,sFileName,
              sCurves,sSurfaces,sTrees,sFileBreps,SM_ASCII));
        } // end filetype branches to read sFileBreps
      
      // read filleting operation - total edges,  and trans vector
      ULONG lFilletingOperation;
      ULONG lExpectedTotalEdges;
      SmVector3d sTrans;
      result = SM_FSCANF(pFile,_T("%ld %ld %lf %lf %lf"),&lFilletingOperation,&lExpectedTotalEdges,&sTrans.x,&sTrans.y,&sTrans.z);
      if (result != 5) {
        MYPRINTF(_T("Error reading filleting operation\n"));
        return SM_ERR;
      }
      SER(my_goto_next_line(pFile));

      // build a translated coordinate system
      SmAxis2Placement sTranslate;
      SER(sTranslate.SetCanonical(sTrans,SmVector3d(1,0,0),SmVector3d(0,1,0)));
      
      // branch on lFilletingOperation
      if (lFilletingOperation == 1) 
        { // Surface-Based Filleting

          // check state - at least 2 entries in sFileBreps
          if (sFileBreps.GetSize() < 2) SER(SM_ERR);

          // get 1st 2 Breps from sFileBreps and translate them
          SmBrep *pBrep1 = sFileBreps[0];
          SmBrep *pBrep2 = sFileBreps[1];
          SER(pBrep1->Transform(sTranslate));
          SER(pBrep2->Transform(sTranslate));

          // enable pBrep1 and pBrep2 for editing
          SmTemporaryChangeValue<SmBoolean> sChange1(pBrep1->m_bEditingEnabled,TRUE);
          SmTemporaryChangeValue<SmBoolean> sChange2(pBrep2->m_bEditingEnabled,TRUE);

          // locals - let pFace1 and pFace2 = 1st face from pBrep1 and pBrep2
          rBreps.Add(pBrep1);
          rBreps.Add(pBrep2);
          SmTArray<SmFace*> sFaces;
          pBrep1->GetFaces(sFaces);
          if (sFaces.GetSize() < 1) SER(SM_ERR);
          SmFace *pFace1 = sFaces[0];
          pBrep2->GetFaces(sFaces);
          if (sFaces.GetSize() < 1) SER(SM_ERR);
          SmFace *pFace2 = sFaces[0];

          // read file for {offset1, offset2, tolerance, boundaryTrimmingType, ReverseTrimFlag}
          double dOffset1, dOffset2, dTolerance;
          ULONG lBoundaryTrimmingType, lCrossSectionType;
          double dRelativeAccuracy = 0.0;
          ULONG lReverseTrim;
          result = SM_FSCANF(pFile,_T("%lf %lf %lf %ld %ld"),&dOffset1,&dOffset2,&dTolerance,
              &lBoundaryTrimmingType,&lReverseTrim);
          if (result != 5) {
            MYPRINTF(_T("Error reading filleting operation\n"));
            return SM_ERR;
          }
          SER(my_goto_next_line(pFile));

          // eTrimType oneof: SM_BT_NONE,    // No fillet triming just generate entire fillet
          //                  SM_BT_MINIMAL, // Trim using ISO curves to minimal intersections on face boundaries
          //                  SM_BT_MAXIMAL, // Trim using ISO curves to maximal intersections on face boundaries
          //                  SM_BT_BEVEL,   // Create a bevel (line in parameter space) between minimal and maximal
          //                                 // intersections on each end of fillet.
          //                  SM_BT_BLEND    // Create a blend (Hermite in parameter space) between minimal and 
          //                                 // maximal intersections on each end of fillet.
          SmBoundaryTrimmingType eTrimType = (SmBoundaryTrimmingType)lBoundaryTrimmingType;

          // read file for { CrossSectionType, RelativeAccuracy, MirrorFlag, Complement}
          ULONG lMirror, lComplement;
          result = SM_FSCANF(pFile,_T("%ld %lf %ld %ld"),&lCrossSectionType,&dRelativeAccuracy,
              &lMirror,&lComplement);
          if (result != 4) {
            MYPRINTF(_T("Error reading fsg\n"));
            return SM_ERR;
          }
          SmBoolean bMirror      = (SmBoolean)lMirror;
          SmBoolean bComplement  = (SmBoolean)lComplement;
          
          // create a fillet between two faces along the intersection
          // line between the two face->surfaces
          SmBrep *pResult;
          SER(my_test_surface_fillet(crContext,            // in : Creation context for the new brep that contains the results.
              pFace1->GetSurface(), // in : Orig Surf1 to connect to new fillet (from Brep1)
              pFace2->GetSurface(), // in : Orig Surf2 to connect to new fillet (from Brep2)
              dOffset1,             // in : FilletSurfBdry signed dist from Surf/Surf XSectCrv on pSur1
              dOffset2,             // in : FilletSurfBdry signed dist from Surf/Surf XSectCrv on pSur2
              dTolerance,           // in : Tolerance: max dist rails to base surfaces
              eTrimType,            // in : SM_BT_NONE,   = No fillet triming just generate entire fillet
                                    //      SM_BT_MINIMAL = Trim using ISO curves to minimal intersections on face boundaries
                                    //      SM_BT_MAXIMAL = Trim using ISO curves to maximal intersections on face boundaries
                                    //      SM_BT_BEVEL,  = Create a bevel (line in parameter space) between minimal and maximal
                                    //                      intersections on each end of fillet.
                                    //      SM_BT_BLEND   = Create a blend (Hermite in parameter space) between minimal and 
                                    //                      maximal intersections on each end of fillet.
              lReverseTrim,         // in : TRUE = Trim inside of surfaces, FALSE=outside
              lCrossSectionType,    // in : 0 - linear, 1 - approx circular, 2 - circular (rational)
              dRelativeAccuracy,    // in : Relative accuracy of approx circular
              bMirror,              // in : TRUE = mirror fillet to produce an inside out fillet, FALSE=don't
              bComplement,          // in : TRUE = complement circular and approx circular cross sections, FALSE=don't.  
                                    //      It is possible to both mirror and complement circular cross sections.
              pResult));            // out: New Brep whose face(s) is(are) the resulting fillet surface(s)
                                    
          if (pResult) rBreps.Add(pResult);

          // check pBrep1 + pBrep2 + pResult edge count against saved edgecount
          SmTArray<SmEdge*> sEdges;
          pBrep1->GetEdges(sEdges);
          ULONG lTotalEdges = sEdges.GetSize();
          pBrep2->GetEdges(sEdges);
          lTotalEdges = lTotalEdges + sEdges.GetSize();
          if (pResult) 
            {
              pResult->GetEdges(sEdges);
              lTotalEdges = lTotalEdges + sEdges.GetSize();
            }
          SM_ASSERT(lTotalEdges == lExpectedTotalEdges);

        } // end (lFilletingOperation == 1) Surface-Based Filleting branch

      else if (lFilletingOperation == 2) 
        { // Topology-Based Filleting

          // locals - let pBrep = sFileBreps[0]
          SmTArray<SmEdge*> sEdges;
          SmBrep *pBrep = sFileBreps[0];

          // pBrep->StitchAndOrient();  // can't do this...it messes up the edge counts
          if (sFileBreps.GetSize() > 1) 
            {
              // Copy all sFileBreps[j] topology into sFileBreps[0]
              for (ULONG j=1; j<sFileBreps.GetSize(); j++) 
                {
                  SmBrep * pOtherBrep = sFileBreps[j];
                  SER(pBrep->MergeBrep(*pOtherBrep));
                }

              // locals for StitchFaces
              ULONG lNumStitched, lNumLamina;
              double dMaxVGap, dMaxEGap;
              double dThisApproxTol3d = 1.0e-3;
              pBrep->m_bEditingEnabled = TRUE;

              // stitch all the faces placed into the sFileBreps[0] Brep
              SER(pBrep->StitchFaces(dThisApproxTol3d,lNumStitched,lNumLamina,dMaxVGap,dMaxEGap));
              pBrep->m_bEditingEnabled = FALSE;

              // inform the public
              pBrep->Dump();

            } // end more than 1 Brep in sFileBreps check

          // place translated pBrep on rBreps list
          SER(pBrep->Transform(sTranslate));
          rBreps.Add(pBrep);

          // locals: pBrep->Edges, fillet surface generator temporary array, filletLaw temporary array 
          pBrep->GetEdges(sEdges);
          SmTArray<SmFilletSurfaceGenerator*> sFSGArray;
          SmObjsDelete<SmFilletSurfaceGenerator*> sCleanFSG(&sFSGArray);
          SmTArray<SmFilletLaw*> sLaws;
          SmObjsDelete<SmFilletLaw*> sCleanLaws(&sLaws);

          // Create temporary filletExecutive to control the overall filleting process. 
          SmFilletExecutive * pFilExec= new(crContext) SmFilletExecutive(crContext,pBrep);
          SmObjDelete sClean(pFilExec);

          // set FilletExecutive Options and self-intersection handler
          pFilExec->SetDoClassification(TRUE);
          pFilExec->SetDoGlobalMerge(TRUE);    // Either or both of these 2 are needed for rollover
          pFilExec->SetDoPiecewiseMerge(TRUE); // Either or both of these 2 are needed for rollover
          SmMakeSurfaceBlendSIH sSIH(1.0);
          pFilExec->SetSelfIntersectionHandler(&sSIH);

          // read number of topology fillet definitions
          ULONG lNumberTopologyFilletDefinitions;
          result = SM_FSCANF(pFile,_T("%ld"),&lNumberTopologyFilletDefinitions);
          if (result != 1) {
            MYPRINTF(_T("Error reading number of topology fillet definitions\n"));
            return SM_ERR;
          }
          SER(my_goto_next_line(pFile));

          // for every TopologyFilletDefinition
          ULONG lTopologyType = 0;
          for (ULONG j=0; j<lNumberTopologyFilletDefinitions; j++) 
            {
              // read lines = { comment } { TopologyType }
              SER(my_goto_next_line(pFile)); // Comment Line
              result = SM_FSCANF(pFile,_T("%ld"),&lTopologyType);
              if (result != 1) {
                MYPRINTF(_T("Error reading topology type\n"));
                return SM_ERR;
              }
              SER(my_goto_next_line(pFile)); 

              // switch on lTopologyType
              switch (lTopologyType) 
                {
                  case 0: // Vertex fillet
                          SER(SM_ERR);
                          break;

                  case 1: // Edge Fillet - read then add FilletSolver to FilletExecutive for indexed edge
                      {
                          ULONG lEdgeIndex;
                          result = SM_FSCANF(pFile,_T("%ld"),&lEdgeIndex);
                          if (result != 1) {
                            MYPRINTF(_T("Error reading edge index\n"));
                            return SM_ERR;
                          }
                          if (lEdgeIndex >= sEdges.GetSize()) { SE(SM_ERR); continue; }
                          SmFilletSolver *pFS = my_read_edge_fillet_solver(crContext,pFile,
                              sEdges[lEdgeIndex], sLaws, NULL, NULL);
#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE;
                          if (bDebugMe || smGet_DoGraphics()) {
                              smgfx_Erase() ; 
                              smgfx_SetColor(1,0,0);
                              smgfx_SetLineWidth(4);
                              sEdges[lEdgeIndex]->Draw(); sm_GraphicsLoop();
                              smgfx_SetColor(0,0,0);
                              smgfx_SetLineWidth(2);
                              pBrep->Draw(TRUE); sm_GraphicsLoop();
                              sm_GraphicsLoop();
                          }
#endif // SM_GFX_CODE
                          SmFilletSurfaceGenerator *pFSG = my_read_fsg(crContext,pFile);
                          NER(pFSG);
                          // Make sure pFSG gets cleaned up by putting into this array
                          sFSGArray.Add(pFSG);
                          if (pFS) 
                            {
                              // Hook up FSG
                              pFS->SetFilletSurfaceGenerator(pFSG);
                              // Load the solver into the fillet executive
                              pFilExec->LoadFilletSolver(pFS);
                            }
                      }
                      break;

                  case 2: // All Edges of a Face
                      {
                          // locals
                          SmTArray<SmEdge*> sFaceEdges;
                          SmTArray<SmFace*> sFaces;
                          SmTArray<SmFilletSolver*> sFilletSolvers;
                          pBrep->GetFaces(sFaces);
                          
                          // read face index and get face->edges
                          ULONG lFaceIndex;
                          result = SM_FSCANF(pFile,_T("%ld"),&lFaceIndex);
                          if (result != 1) {
                            MYPRINTF(_T("Error reading face index\n"));
                            return SM_ERR;
                          }
                          if (lFaceIndex >= sFaces.GetSize()) SER(SM_ERR);
                          SmFace *pFace = sFaces[lFaceIndex];
                          pFace->GetEdges(sFaceEdges);
#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE;
                          if (bDebugMe || smGet_DoGraphics()) {
                              smgfx_Erase() ; 
                              smgfx_SetColor(1,0,0);
                              smgfx_SetLineWidth(4);
                              pFace->Draw(); sm_GraphicsLoop();
                              smgfx_SetColor(0,0,0);
                              smgfx_SetLineWidth(2);
                              pBrep->Draw(TRUE);  sm_GraphicsLoop();
                              sm_GraphicsLoop();
                          }
#endif
                          // build 1 FilletSolver for every edge in sFaceEdges
                          // read just 1 FilletSolver description block from pFile
                          SER(my_read_edges_fillet_solver(crContext,pFile,sFaceEdges, sFilletSolvers, sLaws, NULL));

                          // read FilletSurfaceGenerator block
                          SmFilletSurfaceGenerator *pFSG = my_read_fsg(crContext,pFile);
                          NER(pFSG);
                          // Make sure pFSG gets cleaned up by putting into this array
                          sFSGArray.Add(pFSG);

                          // Hook up FSG
                          for (ULONG kkk=0; kkk<sFilletSolvers.GetSize(); kkk++)
                            {
                              SmFilletSolver *pFS = sFilletSolvers[kkk];
                              pFS->SetFilletSurfaceGenerator(pFSG);
                              pFilExec->LoadFilletSolver(pFS);
                            }
                      }
                      break;

                  case 3: // All Edges of a Brep
                      {
                          // build 1 FilletSolver for every edge in sEdges
                          // read just 1 FilletSolver description block from pFile
                          SmTArray<SmFilletSolver*> sFilletSolvers;
                          SER(my_read_edges_fillet_solver(crContext,pFile,sEdges, sFilletSolvers, sLaws, NULL));

                          // build 1 SmFilletSurfaceGenerator - 
                          // read just 1 FilletSurfaceGenerator description from pFile
                          SmFilletSurfaceGenerator *pFSG = my_read_fsg(crContext,pFile);
                          NER(pFSG);
                          // Make sure pFSG gets cleaned up by putting into this array
                          sFSGArray.Add(pFSG);
                          // Hook up FSG
                          for (ULONG kkk=0; kkk<sFilletSolvers.GetSize(); kkk++) {
                              SmFilletSolver *pFS = sFilletSolvers[kkk];
                              pFS->SetFilletSurfaceGenerator(pFSG);
                              pFilExec->LoadFilletSolver(pFS);
                          }

#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE;
                          if (bDebugMe || smGet_DoGraphics()) {
                              smgfx_Erase() ; 
                              smgfx_SetColor(0,0,0);
                              smgfx_SetLineWidth(2);
                              pBrep->Draw(TRUE); sm_GraphicsLoop();
                              sm_GraphicsLoop();
                          }
#endif
                      }
                      break;

                  case 4: // Edge and Face Blend
                      SER(SM_ERR);
                      break;

                  case 5: // Vertex and Face Blend
                      SER(SM_ERR);
                      break;

                  case 6: // Vertex and Edge Blend
                      SER(SM_ERR);
                      break;

                  case 7: // Blend 2 Edges
                      SER(SM_ERR);
                      break;

                  case 8: // Blend 2 Faces
                      SER(SM_ERR);
                      break;

                  case 9: // Complete Regression test - fillet each edge one-at-a-time
                          //                            fillet all edges around each face one face-at-a-time
                          //                            fillet all Brep edges at once
                      {
                          // read 1 FilletSolver description from pFile
                          TCHAR sBuff[256][256];
                          my_read_edge_fillet_solver(crContext,pFile,NULL,sLaws,NULL,sBuff);
                          
                          // read 1 FilletSurfaceGenerator description from pFile
                          SmFilletSurfaceGenerator *pFSG = my_read_fsg(crContext,pFile);
                          NER(pFSG);

                          // Make sure pFSG gets cleaned up by putting into this array
                          sFSGArray.Add(pFSG);

                          SmExtent3d sBrepBBox;
                          SER(pBrep->CalculateBoundingBox(sBrepBBox));

                          SmVector3d sSize = sBrepBBox.GetSize();
                          sSize = sSize * 2.5;

                          // First fillet each Brep->edge one at a time 
                          for (ULONG iii=0; iii<sEdges.GetSize(); iii++) 
                            {
                              // place a translated copy of pBrep into rBreps array
                              SmAxis2Placement sTrans3;
                              ULONG lColumn = iii % 10;
                              ULONG lRow    = iii / 10;
                              SmVector3d sOrigin(lColumn * sSize.x, (1+lRow) * sSize.y, 0.0);
                              SmBrep *pCopyBrep = new(crContext) SmBrep(*pBrep);
                              NER(pCopyBrep);
                              rBreps.Add(pCopyBrep);
                              sTrans3.SetCanonical(sOrigin,SmVector3d(1,0,0),SmVector3d(0,1,0));
                              pCopyBrep->Transform(sTrans3);

                              // get copied Brep edges
                              pCopyBrep->GetEdges(sEdges);

                              // make a FilletExecutive for the copy Brep
                              SmFilletExecutive * pFilExec2 = new(crContext) SmFilletExecutive(crContext,pCopyBrep);
                              SmObjDelete sClean1(pFilExec2);
                              pFilExec2->SetDoClassification(TRUE);
                              pFilExec2->SetDoGlobalMerge(TRUE);
                              SmMakeCurveBlendSIH pSIH(1.0);
                              pFilExec2->SetSelfIntersectionHandler(&sSIH);

                              // build filletSolver for this edge index from sBuff FilletSolver description
                              SmFilletSolver *pFS = my_read_edge_fillet_solver(crContext,pFile,sEdges[iii],sLaws,sBuff,NULL);
                              if (pFS) 
                                {
                                  // Hook up FSG
                                  pFS->SetFilletSurfaceGenerator(pFSG);
                                  // Load the solver into the fillet executive
                                  pFilExec2->LoadFilletSolver(pFS);
                                }
#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE;
                              if (bDebugMe || smGet_DoGraphics()) {
                                  smgfx_Erase() ;
                                  smgfx_SetColor(1,0,0);
                                  smgfx_SetLineWidth(4);
                                  sEdges[iii]->Draw(); sm_GraphicsLoop();
                                  smgfx_SetColor(0,0,0);
                                  smgfx_SetLineWidth(2);
                                  pCopyBrep->Draw(TRUE); sm_GraphicsLoop();
                                  sm_GraphicsLoop();
                              }
#endif
                              SmTArray<SmFilletSolver*> & rFilletSolvers = pFilExec2->GetFilletSolvers();

                              // if there are no edges to fillet - output an error msg
                              if (rFilletSolvers.GetSize() == 0) 
                                {
                                  TCHAR sBuffer[SM_TBLOCK_SIZE];
                                  SM_SPRINTF( sBuffer,_T("\n##################\n# FILLET WARNING # - No Edges To Fillet in # %ld in Test File - %s\n##################\n\n"),
                                      iii,cInputFileName);
                                  smos_ErrorMessage(SM_ERR_UNKNOWN,(const TCHAR * const)FILE_NAME,LINE_NUMBER, sBuffer );
                                }
                              // else do the pFilExec2 filleting - output messages for failures
                              else if (   pFilExec2->CreateFilletCorners() != SM_SUCCESS
                                       || pFilExec2->DoFilleting()         != SM_SUCCESS)   // note: increments unlocked mark value
                                {
                                  TCHAR sBuffer[SM_TBLOCK_SIZE];
                                  SM_SPRINTF( sBuffer,_T("\n################\n# FILLET ERROR # - Error Return in Edge Fillet # %ld in Test File - %s\n################\n\n"),
                                      iii,cInputFileName);
                                  smos_ErrorMessage(SM_ERR_UNKNOWN,(const TCHAR * const)FILE_NAME,LINE_NUMBER, sBuffer );
                                }
                              // arrive here when filleting succeeds - inform the public
                              else 
                                {
                                  TCHAR sBuffer[SM_TBLOCK_SIZE];
                                  SM_SPRINTF( sBuffer,_T("\n\nFILLET SUCCESS - Edge # %ld in Test File - %s\n"), iii,cInputFileName);
                                  smos_WriteBuffer( sBuffer );
                                  pCopyBrep->Dump();
                                  smos_WriteBuffer(_T("\n\n"));
#ifdef SM_GFX_CODE
                                 if (bDebugMe || smGet_DoGraphics()) {
                                     smgfx_SetColor(0,0,0);
                                     smgfx_SetLineWidth(2);
                                     pCopyBrep->Draw(TRUE); sm_GraphicsLoop();
                                     sm_GraphicsLoop();
                                 }
#endif
                                }

                            } // end iter every Brep->edge

                          // Now do individual faces

                          ULONG lStartRow = sEdges.GetSize() / 10 + 2;
                          SmTArray<SmEdge*> sFaceEdges;

                          // for every Brep->Face
                          SmTArray<SmFace*> sFaces;
                          pBrep->GetFaces(sFaces);
                          for (ULONG ii=0; ii<sFaces.GetSize(); ii++) 
                            {
                              // place a translated copy of Brep into rBreps
                              SmAxis2Placement sTrans1;
                              ULONG lColumn = ii % 10;
                              ULONG lRow    = ii / 10;
                              SmVector3d sOrigin(lColumn * sSize.x, (lStartRow+lRow) * sSize.y, 0.0);
                              SmBrep *pCopyBrep = new(crContext) SmBrep(*pBrep);
                              NER(pCopyBrep);
                              rBreps.Add(pCopyBrep);
                              sTrans1.SetCanonical(sOrigin,SmVector3d(1,0,0),SmVector3d(0,1,0));
                              pCopyBrep->Transform(sTrans1);

                              // get copyBrep->faces
                              pCopyBrep->GetFaces(sFaces);

                              // get current face->edges
                              SmFace *pFace = sFaces[ii];
                              pFace->GetEdges(sFaceEdges);
                              SmTArray<SmFilletSolver*> sFilletSolvers;

                              // build 1 FilletSolver for every edge in sFaceEdges
                              // using FilletSolver description in sBuff
                              SER(my_read_edges_fillet_solver(crContext,pFile,sFaceEdges,sFilletSolvers, sLaws, sBuff));

                              // create FilletExecutive for filleting this pBrepCopy
                              SmFilletExecutive * pFilExec2 = new(crContext) SmFilletExecutive(crContext,pCopyBrep);
                              SmObjDelete sClean2(pFilExec2);
                              pFilExec2->SetDoClassification(TRUE);
                              pFilExec2->SetDoGlobalMerge(TRUE);
                              SmMakeCurveBlendSIH pSIH(1.0);
                              pFilExec2->SetSelfIntersectionHandler(&sSIH);
                              
                              // Hook up FSG
                              for (ULONG kkk=0; kkk<sFilletSolvers.GetSize(); kkk++) 
                                {
                                  SmFilletSolver *pFS = sFilletSolvers[kkk];
                                  pFS->SetFilletSurfaceGenerator(pFSG);
                                  pFilExec2->LoadFilletSolver(pFS);
                                }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
                              if (bDebugMe || smGet_DoGraphics()) {
                                  smgfx_Erase() ; 
                                  smgfx_SetColor(1,0,0);
                                  smgfx_SetLineWidth(4);
                                  pFace->Draw(); sm_GraphicsLoop();
                                  smgfx_SetColor(0,0,0);
                                  smgfx_SetLineWidth(2);
                                  pCopyBrep->Draw(TRUE); sm_GraphicsLoop(); 
                                  sm_GraphicsLoop();
                              }
#endif

                              SmTArray<SmFilletSolver*> & rFilletSolvers = pFilExec2->GetFilletSolvers();

                              // when there are no edges to fillet - output an error
                              if (rFilletSolvers.GetSize() == 0) 
                                {
                                  TCHAR sBuffer[SM_TBLOCK_SIZE];
                                  SM_SPRINTF( sBuffer,_T("\n##################\n# FILLET WARNING # - No Edges To Fillet in # %ld in Test File - %s\n##################\n\n"), ii,cInputFileName);
                                  smos_ErrorMessage(SM_ERR_UNKNOWN,(const TCHAR * const)FILE_NAME,LINE_NUMBER, sBuffer );
                                }
                              // do filleting - when filleting fails - output an error
                              else  if (   pFilExec2->CreateFilletCorners() != SM_SUCCESS
                                        || pFilExec2->DoFilleting() != SM_SUCCESS)   // note: increments unlocked mark value
                                {
                                  TCHAR sBuffer[SM_TBLOCK_SIZE]; 
                                  SM_SPRINTF( sBuffer,_T("\n################\n# FILLET ERROR # - Error Return in Face Fillet # %ld in Test File - %s\n################\n\n"), ii,cInputFileName);
                                  smos_ErrorMessage(SM_ERR_UNKNOWN,(const TCHAR * const)FILE_NAME,LINE_NUMBER, sBuffer );
                                }
                              else // filletingt succeeded - inform the public
                                {
                                  TCHAR sBuffer[SM_TBLOCK_SIZE]; 
                                  SM_SPRINTF( sBuffer,_T("\n\nFILLET SUCCESS - Face # %ld in Test File - %s\n"),ii,cInputFileName);
                                  smos_WriteBuffer(sBuffer);
                                  pCopyBrep->Dump();
                                  smos_WriteBuffer(_T("\n\n"));
#ifdef SM_DEBUG_CODE
                                 if (bDebugMe || smGet_DoGraphics()) {
                                     smgfx_SetColor(0,0,0);
                                     smgfx_SetLineWidth(2);
                                     pCopyBrep->Draw(TRUE); sm_GraphicsLoop();
                                     sm_GraphicsLoop();
                                 }
#endif
                                }
                            } // end iter every Brep->Face


                          { // Fillet all edges of the Brep

                            // place translated copy of Brep into rBreps array
                            SmAxis2Placement sTrans2;
                            SmVector3d sOrigin(-30.0, -30.0, 0.0);
                            SmBrep *pCopyBrep = new(crContext) SmBrep(*pBrep);
                            NER(pCopyBrep);
                            rBreps.Add(pCopyBrep);
                            sTrans2.SetCanonical(sOrigin,SmVector3d(1,0,0),SmVector3d(0,1,0));
                            pCopyBrep->Transform(sTrans2);

                            // get all copyBrep edges
                            pCopyBrep->GetEdges(sEdges);

                            // build 1 FilletSolver for every edge in sEdges
                            // using FilletSolver description in sBuff
                            SmTArray<SmFilletSolver*> sFilletSolvers;
                            SER(my_read_edges_fillet_solver(crContext,pFile,sEdges, sFilletSolvers, sLaws, sBuff));

                            // create FilletExecutive to fillet Brep copy
                            SmFilletExecutive * pFilExec2 = new(crContext) SmFilletExecutive(crContext,pCopyBrep);
                            SmObjDelete sClean3(pFilExec2);
                            pFilExec2->SetDoClassification(TRUE);
                            pFilExec2->SetDoGlobalMerge(TRUE);
                            SmMakeCurveBlendSIH pSIH(1.0);
                            pFilExec2->SetSelfIntersectionHandler(&sSIH);
                            
                            // Hook up FSG
                            for (ULONG kkk=0; kkk<sFilletSolvers.GetSize(); kkk++) 
                              {
                                SmFilletSolver *pFS = sFilletSolvers[kkk];
                                pFS->SetFilletSurfaceGenerator(pFSG);
                                pFilExec2->LoadFilletSolver(pFS);
                              }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
                              if (bDebugMe || smGet_DoGraphics()) {
                                  smgfx_Erase() ;
                                  smgfx_SetColor(0,0,0);
                                  smgfx_SetLineWidth(2);
                                  pCopyBrep->Draw(TRUE);
                                  sm_GraphicsLoop();
                              }
#endif
                            // get all FilletSolvers
                            SmTArray<SmFilletSolver*> & rFilletSolvers = pFilExec2->GetFilletSolvers();

                            // when there are no FilletSolvers - output an error
                            if (rFilletSolvers.GetSize() == 0) 
                              {
                                TCHAR sBuffer[SM_TBLOCK_SIZE];
                                SM_SPRINTF( sBuffer,_T("\n##################\n# FILLET WARNING # - No Edges To Fillet All Edges Fillet in Test File - %s\n##################\n\n"),
                                    cInputFileName);
                                smos_ErrorMessage(SM_ERR_UNKNOWN,(const TCHAR * const)FILE_NAME,LINE_NUMBER, sBuffer );
                              }
                            // else do filleting - output error messages
                            else  if (   pFilExec2->CreateFilletCorners() != SM_SUCCESS
                                      || pFilExec2->DoFilleting()         != SM_SUCCESS)   // note: increments unlocked mark value
                              {
                                TCHAR sBuffer[SM_TBLOCK_SIZE];
                                SM_SPRINTF( sBuffer,_T("\n################\n# FILLET ERROR # - Error Return in All Edges Fillet in Test File - %s\n################\n\n"),
                                    cInputFileName);
                                smos_ErrorMessage(SM_ERR_UNKNOWN,(const TCHAR * const)FILE_NAME,LINE_NUMBER, sBuffer );
                              }
                            else // filleting succeeded - inform the public
                              {
                                TCHAR sBuffer[SM_TBLOCK_SIZE];
                                SM_SPRINTF( sBuffer,_T("\n\nFILLET SUCCESS - All Edges in Test File - %s\n"),
                                    cInputFileName);
                                smos_WriteBuffer( sBuffer );
                                pCopyBrep->Dump();
                                smos_WriteBuffer(_T("\n\n"));
#ifdef SM_DEBUG_CODE
                                if (bDebugMe || smGet_DoGraphics()) {
                                    smgfx_SetColor(0,0,0);
                                    smgfx_SetLineWidth(2);
                                    pCopyBrep->Draw(TRUE);
                                    sm_GraphicsLoop();
                                }
#endif
                              }
           
                          } // end fillet all Brep edges block
                      }
                      break;
                } // end Switch on lTopologyType
            } // end iter every TopologyFilletDefinition

          // quit for case == 9 - already done with the filleting
          if (lTopologyType == 9) { continue; }

#ifdef SM_DEBUG_CODE
          clock_t start = clock();
#endif // SM_DEBUG_CODE
          // check number of edges to be filleted
          SmTArray<SmFilletSolver*> & rFilletSolvers = pFilExec->GetFilletSolvers();

          // when no edges are filleted - output an error
          if (rFilletSolvers.GetSize() == 0) 
            {
              TCHAR sBuffer[SM_TBLOCK_SIZE];
              SM_SPRINTF( sBuffer,_T("\n##################\n# FILLET WARNING # - No Edges To Fillet in # %ld in Test File - %s\n##################\n\n"),
                  i,cInputFileName);
              smos_ErrorMessage(SM_ERR_UNKNOWN,(const TCHAR * const)FILE_NAME,LINE_NUMBER, sBuffer );
              continue;
            }
          // do the filleting - output error messages
          else 
            {
              if (   pFilExec->CreateFilletCorners() != SM_SUCCESS
                  || pFilExec->DoFilleting()         != SM_SUCCESS)   // note: increments unlocked mark value
                { 
                  TCHAR sBuffer[SM_TBLOCK_SIZE];
                  SM_SPRINTF( sBuffer,_T("\n################\n# FILLET ERROR # - Error Return in Operation # %ld in Test File - %s\n################\n\n"),
                      i,cInputFileName);
                  smos_ErrorMessage(SM_ERR_UNKNOWN,(const TCHAR * const)FILE_NAME,LINE_NUMBER, sBuffer );
                  continue;
                }
            }
#ifdef SM_DEBUG_CODE
          clock_t finish = clock();
          sm_PrintTime(_T("Time for Filleting"),start,finish);
#endif
          // check result by checking the total edge count - inform the public
          pBrep->GetEdges(sEdges);
          if (sEdges.GetSize() != lExpectedTotalEdges) 
            {
              TCHAR sBuffer[SM_TBLOCK_SIZE];
              SM_SPRINTF( sBuffer,_T("\n################\n# FILLET ERROR # - Edge Count (%ld) Failure in Operation # %ld in Test File - %s\n################\n"),
                  sEdges.GetSize(),i,cInputFileName);
              smos_ErrorMessage(SM_ERR_UNKNOWN,(const TCHAR * const)FILE_NAME,LINE_NUMBER, sBuffer );
              pBrep->Dump();
            }
          else 
            {
              TCHAR sBuffer[SM_TBLOCK_SIZE];
              SM_SPRINTF( sBuffer,_T("\n\nFILLET SUCCESS - Operation # %ld in Test File - %s\n"),
                  i,cInputFileName);
              smos_WriteBuffer( sBuffer );
              pBrep->Dump();
#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE;
              if (bDebugMe || smGet_DoGraphics()) {
                  smgfx_SetColor(0,0,0);
                  smgfx_SetLineWidth(2);
                  pBrep->Draw(TRUE) ; sm_GraphicsLoop();
                  sm_GraphicsLoop();
              }
#endif
            }
        } // end (lFilletingOperation == 2) Topology-Based Filleting branch
      else 
        { 
          // unexpected lFilletingOperation value
          SER(SM_ERR); 
        }
      
  } // end iter every filleting operation
  
  // all done
  fclose(pFile);
  return SM_SUCCESS;

} // end my_read_fillet_definition_file


/***********************************************************************
PURPOSE --- This is a generic routine to test Surface-Based Filleting.
    It assumes that the trimmed surfaces to be filleted are the only 
    faces in their respective Brep.  It tries four combinations by 
    varying the  sign of the surface offset to try to find an 
    intersection in the offset trimmed surfaces.  It does a relatively 
    good job of finding all possible fillets.  

USAGE NOTES --- 
***********************************************************************/
SmStatus my_test_two_brep_fillet
  (const SmContext      & crContext,      // in : Creation context for the new brep that contains the results. 
   SmBrep               * pBrep1,         // in : Brep1 which contains a single trimmed surface (face)           
   SmBrep               * pBrep2,         // in : Brep2 which contains a single trimmed surface (face)           
   double                 dFilletRadius1, // in : Radius of the fillet brep 1 surface (signed)            
   double                 dFilletRadius2, // in : Radius of the fillet brep 2 surface (signed)            
   double                 dTolerance,     // in : Tolerance defining            
   SmBoundaryTrimmingType eTrimType,      // in : SM_BT_NONE,   = No fillet triming just generate entire fillet
                                          //      SM_BT_MINIMAL = Trim using ISO curves to minimal intersections on face boundaries
                                          //      SM_BT_MAXIMAL = Trim using ISO curves to maximal intersections on face boundaries
                                          //      SM_BT_BEVEL,  = Create a bevel (line in parameter space) between minimal and maximal
                                          //                      intersections on each end of fillet.
                                          //      SM_BT_BLEND   = Create a blend (Hermite in parameter space) between minimal and 
                                          //                      maximal intersections on each end of fillet.
   SmBrep              *& rpResult)       // in : new Brep whose faces are the resulting fillet surface(s)        
{
  rpResult = NULL;

  // Enable editing on the two original Breps for the 
  // duration of this function.
  SmTemporaryChangeValue<SmBoolean> sChange1(pBrep1->m_bEditingEnabled,TRUE);
  SmTemporaryChangeValue<SmBoolean> sChange2(pBrep2->m_bEditingEnabled,TRUE);

#ifdef SM_GFX_CODE
  if (smGet_DoGraphics()) // Draw input Breps (expect one face each)
    {
      smgfx_SetLook(1,2, 0,0,1); pBrep1->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0); pBrep2->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE

  // For right now we assume there is just one face per Brep 
  // and that those faces get used for filleting.  
  SmTArray<SmFace*> sFaces1;
  SmTArray<SmFace*> sFaces2;
  pBrep1->GetFaces(sFaces1);
  pBrep2->GetFaces(sFaces2);

  if ( sFaces1.GetSize() < 1 || sFaces2.GetSize() < 1 )
	{ SER(SM_ERR); }

  // face locals
  SmFace     * pF1 = sFaces1[0];
  SmFace     * pF2 = sFaces2[0];
  SmExtent2d   sDomain1 = pF1->GetUVDomain();
  SmExtent2d   sDomain2 = pF2->GetUVDomain();
  SmSurface  * pSur1 = pF1->GetSurface();
  SmSurface  * pSur2 = pF2->GetSurface();
    
  // Do something real simple here to get orientations - just
  // use center of surfaces and normal vectors to take a guess
  // at which direction the offset should be made.
  // This should give us a good starting point for choosing the
  // correct orientation the first time for many of the cases.

  // sP    = Point near center of each surface, 
  // sNorm = Normal at those point
  SmPoint3d  sP1,    sP2;
  SmVector3d sNorm1, sNorm2;

  // Don't put the guesses exactly on the center because of the
  // way things tend to line up.
  SmPoint2d sGuess1 = sDomain1.Evaluate(0.5001,0.4999);
  SmPoint2d sGuess2 = sDomain2.Evaluate(0.4999,0.5001);

  SER(pSur1->EvaluatePoint(sGuess1,sP1));
  SER(pSur2->EvaluatePoint(sGuess2,sP2));

  SER(pSur1->EvaluateNormal(sGuess1,TRUE,TRUE,sNorm1));
  SER(pSur2->EvaluateNormal(sGuess2,TRUE,TRUE,sNorm2));
    
#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe && smGet_DoGraphics())
    {
      sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,0,1); pSur1->DrawAt(sGuess1,1); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,1,1); pSur2->DrawAt(sGuess2,1); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE
    
  // If the normal of the center is in the same direction
  // as the vector to the other surface the orientation is the same.
  SmVector3d sV1  = sP2 - sP1; // vec = vec from pt on surface 1 to pt on surface 2
                               // it's a very rough approximate guess of the shape of the fillet
  SmBoolean  bOrient1 = (  sV1 .Dot(sNorm1) >= 0.0) ? TRUE : FALSE ;
  SmBoolean  bOrient2 = ((-sV1).Dot(sNorm2) >= 0.0) ? TRUE : FALSE ;

  // If the initial guess does not yeild a good result try swapping
  // the orientations and keep going until we have tried all four
  // combinations of surface offsets.  If we don't get any answers
  // after all four tries then there is not a valid fillet between
  // the two trimmed surfaces of the given radius.  
    
  // This loop tries both sides of surface 1 offsets
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  for (ULONG i=0,cnt=0; i<2; i++) 
    {

      if (i==1) bOrient1 = !bOrient1;
      double dRadius1 = (!bOrient1) ? -dFilletRadius1 : dFilletRadius1 ;

      // This loop tries both sides of surface 2 offsets
      for (ULONG j=0; j<2; j++,cnt++) 
        {
          SmBoolean bOr2Tmp  = (j==1) ? !bOrient2 : bOrient2;
          double    dRadius2 = (!bOr2Tmp) ? -dFilletRadius2 : dFilletRadius2 ;
          SM_SPRINTF(sBuff,_T("\nEntered my_test_surface_fillet iter:%ld\n"), cnt) ;
          MYPRINTF(sBuff) ;

          if(SM_SUCCESS == my_test_surface_fillet
                             (crContext,    // in : Creation context for the new brep that contains the results.  
                              pSur1,        // in : Orig Surf1 to connect to new fillet (from Brep1) 
                              pSur2,        // in : Orig Surf2 to connect to new fillet (from Brep2) 
                              dRadius1,     // in : FilletSurfBdry signed dist from Surf/Surf XSectCrv on pSur1 
                              dRadius2,     // in : FilletSurfBdry signed dist from Surf/Surf XSectCrv on pSur2 
                              dTolerance,   // in : Tolerance: max dist rails to base surfaces 
                              eTrimType,    // in : SM_BT_NONE,   = No fillet triming just generate entire fillet
                                            //      SM_BT_MINIMAL = Trim using ISO curves to minimal intersections on face boundaries
                                            //      SM_BT_MAXIMAL = Trim using ISO curves to maximal intersections on face boundaries
                                            //      SM_BT_BEVEL,  = Create a bevel (line in parameter space) between minimal and maximal
                                            //                      intersections on each end of fillet.
                                            //      SM_BT_BLEND   = Create a blend (Hermite in parameter space) between minimal and 
                                            //                      maximal intersections on each end of fillet.
                              FALSE,        // in : TRUE = Trim inside of surfaces, FALSE=outside 
                              2,            // in : 0 - linear, 1 - approx circular, 2 - circular (rational) 
                              0.1,          // in : Relative accuracy of approx circular 
                              FALSE,        // in : TRUE = mirror fillet to produce an inside out fillet, FALSE=don't 
                              FALSE,        // in : TRUE = complement circular and approx circular cross sections, FALSE=don't.  
                                            //      It is possible to both mirror and complement circular cross sections.
                              rpResult))    // out: New Brep whose face(s) is(are) the resulting fillet surface(s) 
            {
              if (rpResult) return SM_SUCCESS;
            }
        } // end 2 iters for j - both sides of surface 2
    } // end 2 iters for i - both sides of surface 1

  return SM_SUCCESS ;

} // end my_test_two_brep_fillet    

/***********************************************************************
PURPOSE --- Do fillet between two faces using an SmSurfaceSurfaceFS
            object along the curves of the surface/surface intersection.

USAGE NOTES --- 
***********************************************************************/
SmStatus my_test_surface_fillet
  (const SmContext     & crContext,      // in : Creation context for the new brep that contains the results.
  SmSurface            * pSur1,          // in : Orig Surf1 to connect to new fillet (can be from different Breps)
  SmSurface            * pSur2,          // in : Orig Surf2 to connect to new fillet (can be from different Breps)
  double                 dFilletRadius1, // in : signed radius from pSur1 (FilletCenter = Surf/Surf XSect of pSur1Offset(SignedDist1))
  double                 dFilletRadius2, // in : signed radius from pSur2 (FilletCenter = Surf/Surf XSect of pSur2Offset(SignedDist2))
  double                 dTolerance,     // in : Tolerance: max dist3d from rails to base surfaces
  SmBoundaryTrimmingType eFilTrimType,   // in : SM_BT_NONE,   = No fillet triming just generate entire fillet
                                         //      SM_BT_MINIMAL = Trim using ISO curves to minimal intersections on face boundaries
                                         //      SM_BT_MAXIMAL = Trim using ISO curves to maximal intersections on face boundaries
                                         //      SM_BT_BEVEL,  = Create a bevel (line in parameter space) between minimal and maximal
                                         //                      intersections on each end of fillet.
                                         //      SM_BT_BLEND   = Create a blend (Hermite in parameter space) between minimal and 
                                         //                      maximal intersections on each end of fillet.
  SmBoolean              bReverseTrim,   // in : TRUE = Trim inside of surfaces, FALSE=outside
  ULONG                  lXSectType,     // in : 0 - linear, 1 - approx circular, 2 - circular (rational)
  double                 dXSectAccuracy, // in : Relative accuracy of approx circular
  SmBoolean              bMirror,        // in : TRUE = mirror fillet to produce an inside out fillet, FALSE=don't
  SmBoolean              bComplement,    // in : TRUE = complement circular and approx circular cross sections, FALSE=don't.  
                                         //      It is possible to both mirror and complement circular cross sections.
  SmBrep              *& rpResult)       // out: New Brep whose face(s) is(are) the resulting fillet surface(s)
{
  // send label to output stream
  MYPRINTF(_T("\nEntered my_test_surface_fillet ")) ;

  // init output
  rpResult = NULL;

#ifdef SM_GFX_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe || smGet_DoGraphics()) // draw input surfaces
    {
      smgfx_Erase() ;
      smgfx_SetColor(1,0,0); pSur1->Draw() ; sm_GraphicsLoop();
      smgfx_SetColor(0,0,1); pSur2->Draw() ; sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE

  // Convert arguments as needed.
  ULONG lBaseTrimType = ( bReverseTrim ) ? 2 : 1;

  // Create a fillet executive and call its surface-surface fillet method.
  SmFilletExecutive sFillExec( crContext );

  SmStatus eStat = sFillExec.SurfaceSurfaceFillet
    (crContext,       // in : Creation context for the new brep that contains the results.
     pSur1,           // in : Surface to be filleted (can be from different Breps)
     pSur2,           // in : Surface to be filleted (can be from different Breps)
     dFilletRadius1,  // in : signed radius from pSur1 (FilletCenter = Surf/Surf XSect of pSur1Offset(SignedDist1))
     dFilletRadius2,  // in : signed radius from pSur2 (FilletCenter = Surf/Surf XSect of pSur2Offset(SignedDist2))
     dTolerance,      // in : Tolerance: max dist from rails to base surfaces
     rpResult,        // out: Brep containing the new fillet surface(s)
     lXSectType,      // in : 0=linear, 1=approx circular, 2=circular (rational)
     dXSectAccuracy,  // in : Relative accuracy of approx circular
     eFilTrimType,    // in : SM_BT_NONE,   = No fillet triming just generate entire fillet
                      //      SM_BT_MINIMAL = Trim using ISO curves to minimal intersections on face boundaries
                      //      SM_BT_MAXIMAL = Trim using ISO curves to maximal intersections on face boundaries
                      //      SM_BT_BEVEL,  = Create a bevel (line in parameter space) between minimal and maximal
                      //                      intersections on each end of fillet.
                      //      SM_BT_BLEND   = Create a blend (Hermite in parameter space) between minimal and 
                      //                      maximal intersections on each end of fillet.
     lBaseTrimType,   // in : base surface trimming: 0=none; 1=normal; 2=reverse trim.
     bMirror,         // in : TRUE = mirror fillet to produce an inside out fillet, FALSE=don't
     bComplement);    // in : TRUE = complement circular and approx circular cross sections, FALSE=don't.

  if ( eStat != SM_SUCCESS )
    {
      { delete rpResult; rpResult = NULL; }
    }
  
  if ( rpResult == NULL )
    { return SM_SUCCESS; }

  // If the Brep has no edges, no valid fillet surfaces were made 
  // therefore we clean up the Brep.
  SmTArray<SmEdge*> sEdges;
  rpResult->GetEdges( sEdges );
  if ( sEdges.GetSize() == 0 )
    {
      // No fillet created just continue on to the next case
      SM_ASSERT(rpResult != NULL) ; delete rpResult ; 
      rpResult = NULL;
    }
  else 
    {
      // Just for the fun of it add a color to the fillet brep
      // and return.
      rpResult->AddAttribute( new (crContext)  SmVector3dAttribute( SM_AI_COLOR, SmVector3d(0,0,1) ));
    }

#ifdef SM_GFX_CODE
  if (bDebugMe || smGet_DoGraphics()) {
      smgfx_Erase() ;
      smgfx_SetColor(1,1,0);
      rpResult->Draw() ; sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif
  
  return SM_SUCCESS;

} // end my_test_surface_fillet


