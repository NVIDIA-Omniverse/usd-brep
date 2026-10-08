// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smadvanced_test.cpp
* PURPOSE ---
*
**********************************************************************/
/*___*/

#include "StdAfx.h"

#include <smAdvancedTest.h>
#include <SmBrepData.h>

// External decl
#ifdef SMJAVA
SmStatus my_test_java_interface();
#endif

/*********************************************************
PURPOSE ---

USER NOTES ---
*********************************************************/
PT_EXPORT SmStatus my_test_sec_adv()
{
  int lCount = 0;
  
  TCHAR sBuff[SM_TBLOCK_SIZE];
  SM_SPRINTF( sBuff, _T( "\nEntered my_test_sec_adv %d" ), lCount );
  MYPRINTF( sBuff );

  SmContext sContext;

  // Test singularities in advanced SSI
  if(lCount == 0 || lCount == 1)
  { 
    SmBrep* pBrep1 = new(sContext) SmBrep();
    MYPRINTF( _T( "\nReading test file ../../TestFiles/pt_TestFiles/torus.smb" ) );
    pBrep1->ReadFromFile( sContext, _T( "../../TestFiles/pt_TestFiles/torus.smb" ), SM_ASCII );
    SmObjDelete sClean1( pBrep1 );

    // Build transform
    SmAxis2Placement sA2P;
    sA2P.Translate( SmVector3d( 0, 0, 0 ) );

    SER( pBrep1->Transform( sA2P ) );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_SetColor( 0, 0, 0 );
      pBrep1->Draw(); sm_GraphicsLoop();
    }
#endif
    ULONG lNumFound;
    SmVector3d sVec( 0, 1, 0 );
    SE( my_test_tsurf_section( pBrep1, 1.0e-2, sVec, 7, lNumFound ) );
    SM_ASSERT( lNumFound == 14 || lNumFound == 16 );
  }

  if(lCount == 0 || lCount == 2)
  { // Test singularities in advanced SSI
    SmBrep* pBrep1 = new(sContext) SmBrep();
    MYPRINTF( _T( "\nReading test file ../../TestFiles/pt_TestFiles/torus.smb" ) );
    pBrep1->ReadFromFile( sContext, _T( "../../TestFiles/pt_TestFiles/torus.smb" ), SM_ASCII );
    SmObjDelete sClean1( pBrep1 );

    // Build transform
    SmAxis2Placement sA2P;
    sA2P.Translate( SmVector3d( 0, -40, 0 ) );

    SER( pBrep1->Transform( sA2P ) );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_SetColor( 0, 1, 0 );
      pBrep1->Draw(); sm_GraphicsLoop();
    }
#endif
    ULONG lNumFound;
    SmVector3d sVec( 0, 0, 1 );
    SE( my_test_tsurf_section( pBrep1, 1.0e-2, sVec, 7, lNumFound ) );
    SM_ASSERT( lNumFound == 13 );
  }

  if(lCount == 0 || lCount == 2)
  { // Test singularities in advanced SSI
    SmBrep* pBrep1 = new(sContext) SmBrep();
    MYPRINTF( _T( "\nReading test file ../../TestFiles/pt_TestFiles/torus.smb" ) );
    pBrep1->ReadFromFile( sContext, _T( "../../TestFiles/pt_TestFiles/torus.smb" ), SM_ASCII );
    SmObjDelete sClean1( pBrep1 );

    // Build transform
    SmAxis2Placement sA2P;
    sA2P.Translate( SmVector3d( -40, -40, 0 ) );

    SER( pBrep1->Transform( sA2P ) );
#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      smgfx_SetColor( 0, 0, 0 );
      pBrep1->Draw(); sm_GraphicsLoop();
    }
#endif
    ULONG lNumFound;
    SmVector3d sVec( 1, 0, 0 );
    SE( my_test_tsurf_section( pBrep1, 1.0e-2, sVec, 7, lNumFound ) );
    SM_ASSERT( lNumFound == 16 || lNumFound == 17 );
    lCount = 0;
  }

  return SM_SUCCESS;

} // end my_test_sec_adv

/*********************************************************
PURPOSE ---

USER NOTES ---
*********************************************************/
PT_EXPORT SmStatus my_test_sil_adv(void)
{
    SmContext sContext;

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 1"));

        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/sphere.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/sphere.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(-60, -150, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(1, 0, 0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 2);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 2"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/sphere.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/sphere.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(0, -150, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(0, 1, 0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 2);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 3"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/sphere.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/sphere.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(60, -150, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(0, 0, 1);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 1);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 4"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/sphere.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/sphere.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(90, -150, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(1, 1, 1);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 1);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 5"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/cone.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/cone.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(-60, -120, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(1, 0, 0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 0);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 6"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/cone.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/cone.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(0, -120, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(0, 1, 0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 2);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 7"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/cone.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/cone.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(60, -120, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(0, 0, 1);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 2);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 8"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/cone.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/cone.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(90, -120, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(1, 1, 1);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 2);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 9"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/plane_circ.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/plane_circ.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(-60, -180, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(1, 0, 0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 0);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 10"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/plane_circ.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/plane_circ.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(0, -180, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(0, 1, 0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 0 || lNumFound == 1);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 11"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/plane_circ.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/plane_circ.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(60, -180, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(0, 0, 1);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 0);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 12"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/plane_circ.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/plane_circ.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(90, -180, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(1, 1, 1);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 0);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 13"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/plane_square.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/plane_square.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(-60, -210, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(1, 0, 0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 6);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 14"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/plane_square.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/plane_square.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(0, -210, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(0, 1, 0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 6);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 15"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/plane_square.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/plane_square.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(60, -210, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(0, 0, 1);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 0);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 16"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/plane_square.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/plane_square.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(90, -210, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(1, 1, 1);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 0);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 17"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/toruswvy.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/toruswvy.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(-60, -90, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(1, 1, 1);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 19);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 18"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/toruswvy.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/toruswvy.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(0, -90, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(0, 0, 1);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 4);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 19"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/toruswvy.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/toruswvy.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(60, -90, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(1, 0, 0.0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 48);
    }


    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 20"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/torus.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/torus.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(-60, -40, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(0, 1, 0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 28);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 21"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/torus.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/torus.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(0, -40, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(1, 0, 0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 16);
    }
    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 22"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/torus.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/torus.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(60, -40, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(1, 0, 0.0000001);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 14);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 23"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/vase_rev.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/vase_rev.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(-60, 0, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(1, 1, 1);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 13);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 24"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/vase_rev.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/vase_rev.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(0, 0, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(1, 0.3, 0.1);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 5);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 25"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/vase_rev.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/vase_rev.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(60, 0, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = FALSE;
        SmVector3d sVec(1, 0, 0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 5);
    }

    // Now do same tests with perspective
    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 26"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/toruswvy.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/toruswvy.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(-60, 40, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = TRUE;
        SmVector3d sVec(100, 100, 100);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 6);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 27"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/toruswvy.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/toruswvy.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(0, 40, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = TRUE;
        SmVector3d sVec(0, 60, 100);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 4);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 28"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/toruswvy.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/toruswvy.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(60, 40, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = TRUE;
        SmVector3d sVec(160, 60, 0.0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 10);
    }


    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 29"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/torus.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/torus.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(-60, 90, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = TRUE;
        SmVector3d sVec(-60, 190, 0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 4);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 30"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/torus.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/torus.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(0, 90, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = TRUE;
        SmVector3d sVec(100, 90, 0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 5);
    }
    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 31"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/torus.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/torus.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(60, 90, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = TRUE;
        SmVector3d sVec(60, -90, 0.00001);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 4);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 32"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/vase_rev.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/vase_rev.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(-60, 120, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = TRUE;
        SmVector3d sVec(40, 220, 100);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 8);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 33"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/vase_rev.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/vase_rev.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(0, 120, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = TRUE;
        SmVector3d sVec(160, 30.0, 10.0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 5);
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_sil_adv 34"));
        SmBrep* pBrep1 = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/vase_rev.smb"));
        pBrep1->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/vase_rev.smb"), SM_ASCII);
        SmObjDelete sClean1(pBrep1);

        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(60, 120, 0));

        SER(pBrep1->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_SetColor(0, 0, 0);
            pBrep1->Draw();
        }
#endif

        SmBoolean bPerspective = TRUE;
        SmVector3d sVec(160, 120, 0);
        ULONG lNumFound;
        SE(my_test_tsurf_silhouette(pBrep1, 1.0e-3, sVec, bPerspective, lNumFound));
        SM_ASSERT(lNumFound == 5);
    }


    return SM_SUCCESS;

} // end my_test_sil_adv

/*********************************************************
PURPOSE ---

USER NOTES ---
*********************************************************/
SmStatus my_test_one_ssi
  (const TCHAR * pFileName, 
   const SmVector3d & rTranslate, 
   ULONG lNumFoundTest)
{
   MYPRINTF(_T("\nEntered in my_test_one_ssi")) ;

   SmContext sContext;

   SmTArray<SmCurve*> s3DCurves;
   SmTArray<SmSurface*> sSurfaces;
   SmTArray<SmBrep*> sBreps;
   SmTArray<long> sBooleanTrees;

   if (TRUE) { // Test bug where have a ruled surface with singularity as starting point
        
        SmBrepData::ReadPartFromFile(sContext, pFileName,
                    s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );

        SmBrep *pBrep1 = sBreps[0];
        SmBrep *pBrep2 = sBreps[1];
        SmObjDelete sClean1(pBrep1);
        SmObjDelete sClean2(pBrep2);

        // Delete excess breps not used in testing
        for (ULONG j=2; j<sBreps.GetSize(); j++) {
            SM_ASSERT(sBreps[j] != NULL) ; delete sBreps[j] ; sBreps[j] = NULL ;
        }
        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(rTranslate);

        SER(pBrep1->Transform(sA2P));
        SER(pBrep2->Transform(sA2P));
        ULONG lNumFound;
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            if (FALSE)
            {
                smgfx_Erase();
            }
            smgfx_SetLook(1, 2, 0, 0, 1); pBrep1->Draw(); sm_GraphicsLoop();
            smgfx_SetLook(2, 4, 0, 1, 0); pBrep2->Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif
        my_test_ssi(pBrep1,pBrep2,1.0e-3,lNumFound);
        if ( lNumFound != lNumFoundTest )
        {
            SM_ASSERT( FALSE );
            TCHAR sBuff[64];
            SM_SNPRINTF(sBuff, 64, _T("    Expected %3ld, Found %3ld\n"), lNumFoundTest, lNumFound );
            MYPRINTF(sBuff);
        }
    }
   return SM_SUCCESS;

} // end my_test_one_ssi

/*********************************************************
PURPOSE ---

USER NOTES ---
*********************************************************/
PT_EXPORT SmStatus my_test_ssi_adv(void)
{
    SmContext sContext;

    MYPRINTF(_T("\n\n*********************** Testing Advanced SSI *********************\n"));

    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 1"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_ext_ext_coin.smp"),SmVector3d(90,-120,0),11));
    }

    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 2"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_ext_cyl_none.smp"),SmVector3d(120,-120,0),0));
    }

    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 3"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_ext_cyl_touch.smp"),SmVector3d(150,-120,0),2));
    }
    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 4"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_ext_cyl_2int.smp"),SmVector3d(180,-120,0),2));
    }
    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 5"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_ext_ext_none.smp"),SmVector3d(210,-120,0),0));
    }

    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 6"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_ext_ext_inside.smp"),SmVector3d(90,-90,0),4));
    }
    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 7"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_ext_ext_opposite.smp"),SmVector3d(120,-90,0),4));
    }
    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 8"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_ext_ext_touch.smp"),SmVector3d(150,-90,0),4));
    }
    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 9"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_ext_ext_overlap.smp"),SmVector3d(180,-90,0),4));
    }
    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 10"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_ext_ext_same.smp"),SmVector3d(210,-90,0),4));
    }
    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 11"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_rev_rev_360.smp"),SmVector3d(-60,-120,0),3));
    }
    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 12"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_rev_rev_opposite.smp"),SmVector3d(-30,-120,0),3));
    }

    // Note: many of the tests starting at 13 were reviewed, after IsNurbSurfaceSurfOfRevolution()
    // was changed to return the derived type of the input, instead of always SmSurfOfRevolution.
    // This caused, for example, SmCone::IntersectWithCone() to be called where
    // SmSurfOfRevolution::IntersectWithSurfOfRevolution() had been called before. [bd 2/21/19]

    if (TRUE) { // Test simple analytical intersections: 3 intersections; one is edge-edge.
        MYPRINTF(_T("nEntered my_test_one_ssi 13"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_rev_rev_90.smp"),SmVector3d(0,-120,0),3));
    }

    if (TRUE) { // Test simple analytical intersections: 6 ints: 3 circles each split in half at surface seams
        MYPRINTF(_T("nEntered my_test_one_ssi 14"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_rev_rev_rot180.smp"),SmVector3d(30,-120,0),6));
    }

    if (TRUE) { // Test simple analytical intersections: 6 ints: just touching, 6 degenerate curves.
        MYPRINTF(_T("nEntered my_test_one_ssi 15"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_rev_rev_180_pts.smp"),SmVector3d(60,-120,0),6));
    }

    // Test 16 previously wanted 9 intersections, but 5 looks right.
    // The surfaces are partially coincident: that coincidence regions yeilds
    // four curves, around the perimeter of the coincidnt region,
    // and there is one additional intersection.
    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 16"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_rev_rev_coin.smp"),SmVector3d(-60,-90,0),5));
    }

    // Test 17 previously wanted 12 intersections, but 7 certainly looks right.
    // Partially coincident: 4 curves; 1 ordinary intersection.
    // Then they meet edge-to-edge, which produces 1 edge-edge int and 1 degenerate int.
    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 17"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_rev_rev_coin_touch.smp"),SmVector3d(-30,-90,0), 7));
    }

    // Test 18 A: two ints, both across a seam: 4 total.
    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 18 A"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_rev_sph_full.smp"),SmVector3d(0,-90,0),4));
    }

    // Test 18 B: three ints, all across a seam: 6 total.
    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 18 B"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_rev_cyl_full.smp"),SmVector3d(30,-90,0),6));
    }

    // Test 19: three ints, all across a seam: 6 total.
    if (TRUE) { // Test simple analytical intersections
        MYPRINTF(_T("nEntered my_test_one_ssi 19"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_rev_cone_full.smp"),SmVector3d(60,-90,0),6));
    }

    if (TRUE) { // Test singularities in advanced SSI
        MYPRINTF(_T("nEntered my_test_one_ssi 20"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_tor_purpen_tor_sing.smp"),SmVector3d(-60,-60,0),14));
    }

    if (TRUE) { // Test tangency of sphere and torus with sphere touching inside of torus.
        MYPRINTF(_T("nEntered my_test_one_ssi 21"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_tor_sph_tan.smp"),SmVector3d(-30,-60,0), 2));
    }

    if (TRUE) { // Test tangency of sphere and torus with sphere touching outside of
        // torus.
        MYPRINTF(_T("nEntered my_test_one_ssi 22"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_tor_sph_tan2.smp"),SmVector3d(0,-60,0),2));
    }

    if (TRUE) { // Test tangency two tori at hyperbolic points - coincident on insides of torus
        MYPRINTF(_T("nEntered my_test_one_ssi 23"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_tor_in_tor_tang_hyper.smp"),SmVector3d(30,-60,0), 1));
    }

    if (TRUE) { // Test tangency of two tori at elliptical points - coincident on outside of torus
        MYPRINTF(_T("nEntered my_test_one_ssi 24"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_tor_in_tor_tan_ellip.smp"),SmVector3d(60,-60,0), 1));
    }

    if (TRUE) { // Test simple plane intersection
        MYPRINTF(_T("nEntered my_test_one_ssi 25"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_2pl.smp"),SmVector3d(-60,-30,0), 1));
    }

    if (TRUE) { // Test simple loop intersection of two cylinders
        MYPRINTF(_T("nEntered my_test_one_ssi 26"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_cylcyl1.smp"),SmVector3d(-30,-30,0), 2));
    }

    if (TRUE) { // Plane with revolution - one closed loop in intersection
        MYPRINTF(_T("nEntered my_test_one_ssi 27"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_pln_rev1.smp"),SmVector3d(0,-30,0), 2));
    }
    if (TRUE) { // Plane with revolution - edges of rev are touching plane (BUG)
#ifdef USE_ANALYTICS
        MYPRINTF(_T("nEntered my_test_one_ssi 28"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_pln_rev2.smp"),SmVector3d(30,-30,0), 2));
#else
        MYPRINTF(_T("nEntered my_test_one_ssi 29"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_pln_rev2.smp"),SmVector3d(30,-30,0), 3));
#endif
    }
    if (TRUE) { // Plane with revolution - cutting revolution lengthwise
        MYPRINTF(_T("nEntered my_test_one_ssi 30"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_pln_rev3.smp"),SmVector3d(60,-30,0), 2));
    }

    if (TRUE) { // Plane with cylinder - corner of plane on cyl axis
        MYPRINTF(_T("nEntered my_test_one_ssi 31"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_plncyl1.smp"),SmVector3d(-60,0,0), 1));
    }
    if (TRUE) { // Plane with cylinder - elliptical cut through the plane
        MYPRINTF(_T("nEntered my_test_one_ssi 32"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_plncyl2.smp"),SmVector3d(-30,0,0), 1));
    }
 
    if (TRUE) { // Two revolutions - one interior loop
        MYPRINTF(_T("nEntered my_test_one_ssi 33"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_rev_rev1.smp"),SmVector3d(0,0,0), 2));
    }

    if (TRUE) { // Bumpy surface and plane producing a bunch of loops
        MYPRINTF(_T("nEntered my_test_one_ssi 34"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_bumppy_plane2.smp"),SmVector3d(30,0,0), 50));
    }

    if (FALSE) { // Bumppy surface and plane sitting on a bunch of surface singularities
        // Note that this one is extremely slow and is missing one segment - turned
        // off for now.
        MYPRINTF(_T("nEntered my_test_one_ssi 35"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_bumppy_plane.smp"),SmVector3d(60,0,0), 84));
    }
    if (TRUE) { // Test singularities where there are also tangent curves
    // Right now this one does not work quite right.  It does not find the
    // singularities.  Although it produces the curves it does not break them
    // at the singularity point.  NOTE - It appears to work correctly now!!!
        MYPRINTF(_T("nEntered my_test_one_ssi 36"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_rev_rev_sing.smp"),SmVector3d(-60,30,0), 6));
    }
    if (TRUE) { // Test cylinder/cylinder singularity - equal radii
#ifdef USE_ANALYTICS
        MYPRINTF(_T("nEntered my_test_one_ssi 37"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_cyl_cyl_sing.smp"),SmVector3d(-30,30,0),  4));
#else
        MYPRINTF(_T("nEntered my_test_one_ssi 38"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_cyl_cyl_sing.smp"),SmVector3d(-30,30,0),  6));
#endif

    }

    if (TRUE) { // Test cyl/cyl singularity - different radii
        MYPRINTF(_T("nEntered my_test_one_ssi 39"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_cyl_cyl_sing2.smp"),SmVector3d(0,30,0), 4));
    }

    if (TRUE) { // Test coincidence between two planes
        MYPRINTF(_T("nEntered my_test_one_ssi 40"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_coincident_planes.smp"),SmVector3d(30,30,0), 4));
    }

    if (TRUE) { // Test coincidence between two cylinders
    // Was 2, should be 3, now that SmNurbSurfaceSurfOfRev() returns SmCone
    // when input is SmCone.  [bd 2/21/19]
#ifdef USE_ANALYTICS
        MYPRINTF(_T("nEntered my_test_one_ssi 41"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_coincident_cyls.smp"),SmVector3d(60,30,0), 3));
#else
        MYPRINTF(_T("nEntered my_test_one_ssi 42"));
        SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_coincident_cyls.smp"),SmVector3d(60,30,0), 3));
#endif
    }

   if (TRUE) { // Test coincidence between two spheres with same orientation
       MYPRINTF(_T("nEntered my_test_one_ssi 43"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_coincident_spheres.smp"),SmVector3d(-60,60,0), 1));
    }

   if (TRUE) { // Test coincidence between two spheres with different orientation
       MYPRINTF(_T("nEntered my_test_one_ssi 44"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_coincident_spheres2.smp"),SmVector3d(-30,60,0), 4));
    }

   // Coincident surfaces, should return each surface's boundaries projected onto the other.
   // There are 8 such curves, although we now return each one twice, because
   // opposite surface boundaries are coincident in tori.  (Should we cull duplicates?)
   if (TRUE) { // Test coincidence between two tori with a rotation
       MYPRINTF(_T("nEntered my_test_one_ssi 45"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_coincident_torii.smp"),SmVector3d(0,60,0), 8));
    }

   if (TRUE) { // Test coincidence between torus and sphere with just one singular point touching.
       MYPRINTF(_T("nEntered my_test_one_ssi 46"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_tor_sph_touch.smp"),SmVector3d(30,60,0), 1));
    }

   if (TRUE) { // Test bug where have a ruled surface with singularity as starting point
       MYPRINTF(_T("nEntered my_test_one_ssi 47"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_rhino_bug1.smp"),SmVector3d(60,60,0), 1));
    }
    
   if (TRUE) { // Test case where plane goes through middle of sphere
       // and plane boundaries just touch the sphere.
       // No longer true: [ Returns 4 curves, a full circle broken at those tangent points. ]
       // SmPlane::TrimCurveToPlaneDomain() has been modified to not break curves
       // if they just graze the plane domain.  That would change a single circular
       // Edge into four Edges.  Also note, the general curve intersector returns
       // only one curve in this case, so now the analytic intersector behaves the same.
#ifdef USE_ANALYTICS
       MYPRINTF(_T("nEntered my_test_one_ssi 48"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_sph_plane1.smp"),SmVector3d(-60,90,0), 1));
#else
       MYPRINTF(_T("nEntered my_test_one_ssi 49"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_sph_plane1.smp"),SmVector3d(-60,90,0), 1));
#endif
    }
    
   if (TRUE) { // Same as tests 48/49.
#ifdef USE_ANALYTICS
       MYPRINTF(_T("nEntered my_test_one_ssi 50"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_sph_plane2.smp"),SmVector3d(-30,90,0), 1));
#else
       MYPRINTF(_T("nEntered my_test_one_ssi 51"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_sph_plane2.smp"),SmVector3d(-30,90,0), 5));
#endif
    }
    
   if (TRUE) { // Test case where plane cuts cone pole.
       MYPRINTF(_T("nEntered my_test_one_ssi 52"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_cone_plane1.smp"),SmVector3d(0,90,0),2));
    }
    
   if (TRUE) { // Test case where cylinder goes through cone pole on boundary of cylinder
       MYPRINTF(_T("nEntered my_test_one_ssi 53"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_cone_cyl1.smp"),SmVector3d(30,90,0), 2));
    }
    
   if (TRUE) { // Test case where cylinder goes through cone pole interior to cylinder
       MYPRINTF(_T("nEntered my_test_one_ssi 54"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_cone_cyl2.smp"),SmVector3d(60,90,0), 2));
    }
    
   if (TRUE) { // Test case where cylinder goes through cone pole interior to cylinder
       MYPRINTF(_T("nEntered my_test_one_ssi 55"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_cone_2pln.smp"),SmVector3d(-60,120,0), 1));
    }
    
   if (TRUE) { // Test case where cylinder goes through cone pole interior to cylinder
       MYPRINTF(_T("nEntered my_test_one_ssi 56"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_cyl_bicub.smp"),SmVector3d(-30,120,0), 2));
    }
    
   if (TRUE) { // Test case where cylinder goes through cone pole interior to cylinder
       MYPRINTF(_T("nEntered my_test_one_ssi 57"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_tor_overlap.smp"),SmVector3d(0,120,0), 6));
    }
    
   if (TRUE) { // Test case where cylinder goes through cone pole interior to cylinder
       MYPRINTF(_T("nEntered my_test_one_ssi 58"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_spiral.smp"),SmVector3d(30,120,0), 1));
    }
    
   // Two non-flat bilinear surfaces, boundaries intersect on all four sides.
   // Produces two intersection curves that intersect each other in the middle.
   // That intersection of int curves must be a singularity in the surface intersection.
   // Is our code supposed to break intersection curves at srf/srf singularities?
   // That would result in four int curves.
   if (TRUE) { // Test case where cylinder goes through cone pole interior to cylinder
       MYPRINTF(_T("nEntered my_test_one_ssi 59"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_clasic_ruled.smp"),SmVector3d(60,120,0), 4));
    }
    
   // Two quarter cylinders, perpendicular axis, like fillet surfaces on a cube meeting at a corner.
   // Surfaces are tangent at both ends of int curve.  1 intersection.
   if (TRUE) {
       MYPRINTF(_T("nEntered my_test_one_ssi 60"));
       SER(my_test_one_ssi(_T("../../TestFiles/pt_TestFiles/ssi_cylcyl_90_tan.smp"),SmVector3d(-60,150,0), 1));
    }
    
    return SM_SUCCESS;

} // end my_test_ssi_adv

#ifdef NCLIB
#include "machineBrep.h"
#endif

/*********************************************************
PURPOSE ---

USER NOTES ---
*********************************************************/
PT_EXPORT SmStatus my_test_tsurface_regression()
{
    if (FALSE) {
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 1")) ;
        my_test_trim_surfaces();
        return SM_SUCCESS;
    }

    SmContext sContext;

    if (TRUE) { // Test trimming of surfaces with curves
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 2")) ;
        SER(my_test_trim_surfaces());
//        return SM_SUCCESS;
    }


    if (TRUE) { // Test analytic trimmed surfaces
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 3")) ;
        SER(my_test_analytic_tsurf_creation(sContext));
    }

    if(FALSE){ //Test basic creation of a trimmed surface
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 4")) ;
 
        SmBrep *pBrep,*pBrep2;
        SER(my_test_tsurf_creation(sContext,pBrep));
        SER(my_test_tsurf_creation(sContext,pBrep2));
        SmVector3d rTranslate(4.5,3,0);
        // Build transform
        SmAxis2Placement sA2P;
        sA2P.Translate(rTranslate);

        SER(pBrep2->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()){
        pBrep->Draw();
        pBrep2->Draw();
        sm_GraphicsLoop();
          }
#endif
        
        ULONG lNumFound;
        my_test_ssi(pBrep2,pBrep,1.0e-3,lNumFound);// It is your function too
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            sm_GraphicsLoop();
        }
#endif
        SM_ASSERT(pBrep  != NULL) ; delete pBrep ;  pBrep  = NULL ;
        SM_ASSERT(pBrep2 != NULL) ; delete pBrep2 ; pBrep2 = NULL ;
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 7")) ;
        SmBSplineSurface *pCylBSS = NULL ;
        SmAxis2Placement sA2P;
        sA2P.SetCanonical(SmPoint3d(-2,0,0),SmVector3d(1,0,0),SmVector3d(0,1,0));
        SER(SmBSplineSurface::CreateConePatch(sContext,
            sA2P,1.0,1.0,0.0,360.0,1.0,SM_CO_QUADRATIC,pCylBSS));
        SmObjDelete sCU1(pCylBSS);

        double dArea;
        SER(pCylBSS->Area(pCylBSS->GetNaturalUVDomain(),1.0e-8,dArea));
        TCHAR sBuff[SM_TBLOCK_SIZE];
        SM_SPRINTF(sBuff,_T("Area = %16.16lf\n"),dArea);
        MYPRINTF(sBuff);
    }


    if (TRUE) { // Test trimming of surfaces with curves
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 8")) ;
        SER(my_test_trim_surfaces());
    }

    if (TRUE) { // Test basic distance finding
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 10")) ;

		SmTArray<SmCurve*> s3DCurves;
		SmTArray<SmSurface*> sSurfaces;
		SmTArray<SmBrep*> sBreps;
		SmTArray<long> sBooleanTrees;
        
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/ss_plane_sph1.smp"));
        SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/ss_plane_sph1.smp"), s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
        }
#endif
        SmBrep *pBrep1 = sBreps[0];
        SmBrep *pBrep2 = sBreps[1];
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        pBrep1->Draw(); pBrep2->Draw();
        }
#endif
        ULONG lCount;
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 4);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 8);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_SIGNED_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 4);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        }
#endif
        SM_ASSERT(pBrep1 != NULL) ; delete pBrep1 ; pBrep1 = NULL ;
        SM_ASSERT(pBrep2 != NULL) ; delete pBrep2 ; pBrep2 = NULL ;
    }

    if (TRUE) { // Test basic creation of a trimmed surface
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 11")) ;
        SER(my_test_tsurf_insert_edge());
    }

    if (TRUE) { // Test basic creation of a trimmed surface
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 12")) ;
        SmBrep *pBrep = NULL;
        SER(my_test_tsurf_creation(sContext,pBrep));
//        sm_GraphicsLoop();
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        pBrep->Draw();
        }
#endif
//        sm_GraphicsLoop();
        SM_ASSERT(pBrep != NULL) ; delete pBrep ; pBrep = NULL ;
    }

    if (TRUE) { // Test trimming of surfaces with curves
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 14")) ;
        SER(my_test_trim_surfaces());
    }

    if (TRUE) { // Test basic distance finding
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 15")) ;

		    SmTArray<SmCurve*> s3DCurves;
		    SmTArray<SmSurface*> sSurfaces;
		    SmTArray<SmBrep*> sBreps;
		    SmTArray<long> sBooleanTrees;
        
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/ss_2parallel_planes.smp"));
        SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/ss_2parallel_planes.smp"), s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
        }
#endif
        SmBrep *pBrep1 = sBreps[0];
        SmBrep *pBrep2 = sBreps[1];
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            pBrep1->Draw(); pBrep2->Draw();
        }
#endif
        ULONG lCount;
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 69);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 36);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_NORMALIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()){
        }
#endif
        SM_ASSERT(pBrep1 != NULL) ; delete pBrep1 ; pBrep1 = NULL ;
        SM_ASSERT(pBrep2 != NULL) ; delete pBrep2 ; pBrep2 = NULL ;
    }

    if (TRUE) { // Test basic distance finding
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 16")) ;

        SmTArray<SmCurve*> s3DCurves;
		    SmTArray<SmSurface*> sSurfaces;
		    SmTArray<SmBrep*> sBreps;
		    SmTArray<long> sBooleanTrees;
        
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/ss_2adj_planes.smp"));
        SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/ss_2adj_planes.smp"), s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );
        
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
        }
#endif
        SmBrep *pBrep1 = sBreps[0];
        SmBrep *pBrep2 = sBreps[1];
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            pBrep1->Draw(); pBrep2->Draw();
        }
#endif
        ULONG lCount;
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 34);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 18);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_NORMALIZE,NULL,lCount));
        SM_ASSERT(lCount == 0);
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        }
#endif
        SM_ASSERT(pBrep1 != NULL) ; delete pBrep1 ; pBrep1 = NULL ;
        SM_ASSERT(pBrep2 != NULL) ; delete pBrep2 ; pBrep2 = NULL ;
    }
    if (TRUE) { // Test basic distance finding
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 17")) ;

        SmTArray<SmCurve*> s3DCurves;
		    SmTArray<SmSurface*> sSurfaces;
		    SmTArray<SmBrep*> sBreps;
		    SmTArray<long> sBooleanTrees;
        
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/ss_plane_sph1.smp"));
        SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/ss_plane_sph1.smp"), s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );
        SmBrep *pBrep1 = sBreps[0];
        SmBrep *pBrep2 = sBreps[1];
       
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
            pBrep1->Draw(); sm_GraphicsLoop();
            pBrep2->Draw(); sm_GraphicsLoop();
        }
#endif
        ULONG lCount;
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 3);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_NORMALIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);

        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_PROJECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 4);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_PROJECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 33); 
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 4);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 8);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_SIGNED_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 4);

#if ADDITIONAL_PROJ_TESTS
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_SIGNED_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
#endif
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            sm_GraphicsLoop();
        }
#endif
        SM_ASSERT(pBrep1 != NULL) ; delete pBrep1 ; pBrep1 = NULL ;
        SM_ASSERT(pBrep2 != NULL) ; delete pBrep2 ; pBrep2 = NULL ;
    }

    if (TRUE) { // Test basic distance finding
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 18")) ;

        SmTArray<SmCurve*> s3DCurves;
		    SmTArray<SmSurface*> sSurfaces;
		    SmTArray<SmBrep*> sBreps;
		    SmTArray<long> sBooleanTrees;
        
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/ss_plane_sph2.smp"));
        SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/ss_plane_sph2.smp"), s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );
      
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
        }
#endif
        SmBrep *pBrep1 = sBreps[0];
        SmBrep *pBrep2 = sBreps[1];

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        pBrep1->Draw(); pBrep2->Draw();
        }
#endif

        ULONG lCount;
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 6);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 3);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_NORMALIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);

//        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_PROJECTED_MINIMIZE,lCount));
//        SM_ASSERT(lCount == ??);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_PROJECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 9);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 4);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 4);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_SIGNED_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 8);

#if ADDITIONAL_PROJ_TESTS
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_SIGNED_DIRECTED_MINIMIZE,NULL,NULL,lCount));
        SM_ASSERT(lCount == 1);
#endif
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        }
#endif
        SM_ASSERT(pBrep1 != NULL) ; delete pBrep1 ; pBrep1 = NULL ;
        SM_ASSERT(pBrep2 != NULL) ; delete pBrep2 ; pBrep2 = NULL ;
    }

    if (TRUE) { // Test basic distance finding
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 19")) ;

        SmTArray<SmCurve*> s3DCurves;
		    SmTArray<SmSurface*> sSurfaces;
		    SmTArray<SmBrep*> sBreps;
		    SmTArray<long> sBooleanTrees;
        
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/ss_plane_sph3.smp"));
        SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/ss_plane_sph3.smp"), s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
        }
#endif
        SmBrep *pBrep1 = sBreps[0];
        SmBrep *pBrep2 = sBreps[1];
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        pBrep1->Draw(); pBrep2->Draw();
        }
#endif
        ULONG lCount;
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 2);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 3);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_NORMALIZE,NULL,lCount));
        SM_ASSERT(lCount == 0);

        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_PROJECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 18);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_PROJECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 9);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 0);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 0);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_SIGNED_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 0);

#if ADDITIONAL_PROJ_TESTS
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_SIGNED_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
#endif
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        }
#endif
        SM_ASSERT(pBrep1 != NULL) ; delete pBrep1 ; pBrep1 = NULL ;
        SM_ASSERT(pBrep2 != NULL) ; delete pBrep2 ; pBrep2 = NULL ;
    }

    if (TRUE) { // Test basic distance finding
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 20")) ;

        SmTArray<SmCurve*> s3DCurves;
		    SmTArray<SmSurface*> sSurfaces;
		    SmTArray<SmBrep*> sBreps;
		    SmTArray<long> sBooleanTrees;
        
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/ss_plane_sph4.smp"));
        SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/ss_plane_sph4.smp"), s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
        }
#endif
        SmBrep *pBrep1 = sBreps[0];
        SmBrep *pBrep2 = sBreps[1];
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        pBrep1->Draw(); pBrep2->Draw();
        }
#endif
        ULONG lCount;
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 3);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_NORMALIZE,NULL,lCount));
        SM_ASSERT(lCount == 0);

        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_PROJECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 9);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_PROJECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 36);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 2); // USEDTOBE 5);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 2);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_SIGNED_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 2); // USEDTOBE 5);

#if ADDITIONAL_PROJ_TESTS
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_SIGNED_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
#endif
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        }
#endif
        SM_ASSERT(pBrep1 != NULL) ; delete pBrep1 ; pBrep1 = NULL ;
        SM_ASSERT(pBrep2 != NULL) ; delete pBrep2 ; pBrep2 = NULL ;
    }

    if (TRUE) { // Test basic distance finding
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 21")) ;

		    SmTArray<SmCurve*> s3DCurves;
		    SmTArray<SmSurface*> sSurfaces;
		    SmTArray<SmBrep*> sBreps;
		    SmTArray<long> sBooleanTrees;
        
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/ss_plane_tor1.smp"));
        SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/ss_plane_tor1.smp"), s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );
        
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
        }
#endif
        SmBrep *pBrep1 = sBreps[0];
        SmBrep *pBrep2 = sBreps[1];
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        pBrep1->Draw(); pBrep2->Draw();
        }
#endif
        ULONG lCount;
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 3);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_NORMALIZE,NULL,lCount));
        SM_ASSERT(lCount == 4);

        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_PROJECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 4);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_PROJECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 9);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 4);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 4);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_SIGNED_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 2);

#if ADDITIONAL_PROJ_TESTS
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_SIGNED_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
#endif
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        }
#endif
        SM_ASSERT(pBrep1 != NULL) ; delete pBrep1 ; pBrep1 = NULL ;
        SM_ASSERT(pBrep2 != NULL) ; delete pBrep2 ; pBrep2 = NULL ;
    }

    if (TRUE) { // Test basic distance finding
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 22")) ;

        SmTArray<SmCurve*> s3DCurves;
		    SmTArray<SmSurface*> sSurfaces;
		    SmTArray<SmBrep*> sBreps;
		    SmTArray<long> sBooleanTrees;
        
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/ss_plane_tor2.smp"));
        SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/ss_plane_tor2.smp"), s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
        }
#endif
        SmBrep *pBrep1 = sBreps[0];
        SmBrep *pBrep2 = sBreps[1];
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        pBrep1->Draw(); pBrep2->Draw();
        }
#endif
        ULONG lCount;
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MINIMIZE,NULL,lCount));
        // Please note that this is one of those ambiguous cases where
        // different numbers of answers are correct.
        SM_ASSERT(lCount > 10);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 3);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_NORMALIZE,NULL,lCount));
        SM_ASSERT(lCount == 18); // USEDTOBE 17);

        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_PROJECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 5);  // USEDTOBE 4);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_PROJECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 3);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_SIGNED_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);

#if ADDITIONAL_PROJ_TESTS
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_SIGNED_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
#endif
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        }
#endif
        SM_ASSERT(pBrep1 != NULL) ; delete pBrep1 ; pBrep1 = NULL ;
        SM_ASSERT(pBrep2 != NULL) ; delete pBrep2 ; pBrep2 = NULL ;
    }

    if (TRUE) { // Test basic distance finding
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 23")) ;

        SmTArray<SmCurve*> s3DCurves;
		    SmTArray<SmSurface*> sSurfaces;
		    SmTArray<SmBrep*> sBreps;
		    SmTArray<long> sBooleanTrees;
        
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/ss_tor_tor.smp"));
        SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/ss_tor_tor.smp"), s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );
		
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
        }
#endif
        SmBrep *pBrep1 = sBreps[0];
        SmBrep *pBrep2 = sBreps[1];
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        pBrep1->Draw(); pBrep2->Draw();
        }
#endif
        ULONG lCount;
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_NORMALIZE,NULL,lCount));
        SM_ASSERT(lCount == 16);

        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_PROJECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 2);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_PROJECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 2);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_SIGNED_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);

#if ADDITIONAL_PROJ_TESTS
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_DIRECTED_MAXIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
        SER(my_test_tsurface_tsurface_solve(pBrep1,pBrep2,SM_SO_SIGNED_DIRECTED_MINIMIZE,NULL,lCount));
        SM_ASSERT(lCount == 1);
#endif
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        }
#endif
        SM_ASSERT(pBrep1 != NULL) ; delete pBrep1 ; pBrep1 = NULL ;
        SM_ASSERT(pBrep2 != NULL) ; delete pBrep2 ; pBrep2 = NULL ;
    }
    

    if (FALSE) {
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 24")) ;

        SmTArray<SmCurve*> s3DCurves;
		    SmTArray<SmSurface*> sSurfaces;
		    SmTArray<SmBrep*> sBreps;
		    SmTArray<long> sBooleanTrees;
        
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/cyl_hole.smp"));
        SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/cyl_hole.smp"), s3DCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );
		
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
        }
#endif
        SmBrep *pBrep = sBreps[0];
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        pBrep->Draw();
        }
#endif
        SER(my_topo_regression_test(pBrep,4162,400,49,760,4284,97,30,134,110,49,29));

        ULONG lCount;
        SER(my_test_tsurf_silhouette(pBrep,0.0001,SmVector3d(0,1,0),FALSE,lCount));
        SM_ASSERT(lCount == 6);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        }
#endif
        SM_ASSERT(pBrep != NULL) ; delete pBrep ; pBrep = NULL ;
    }

    if (TRUE) {
        MYPRINTF(_T("\nEntered my_test_tsurface_regression 25")) ;

        SmBrep* pBrep = new(sContext) SmBrep();
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/ellips_ts2.smb"));
	      pBrep->ReadFromFile( sContext, _T("../../TestFiles/pt_TestFiles/ellips_ts2.smb"), SM_ASCII );
		
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
            pBrep->Draw();
        }
#endif
        SER(my_topo_regression_test(pBrep,389,362,52,60,389,59,7,36,68,30,8));//?

        ULONG lCount;
        SER(my_test_tsurf_silhouette(pBrep,0.0001,SmVector3d(1,0,-0.1),FALSE,lCount));
        SM_ASSERT(lCount == 2);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
        }
#endif
        SM_ASSERT(pBrep != NULL) ; delete pBrep ; pBrep = NULL ;
    }

    return SM_SUCCESS;

} // end my_test_tsurface_regression


/*********************************************************
PURPOSE ---

USER NOTES ---
*********************************************************/

PT_EXPORT SmStatus my_test_cci_adv(void)
{
  MYPRINTF(_T("\nEntered my_test_cci_adv 1")) ;
    if (TRUE) {
        SmContext sContext;
        SmTArray<SmCurve*> sCurves;
        SmObjsDelete<SmCurve*> sClean1(&sCurves);
		SmTArray<SmSurface*> sSurfaces;
		SmTArray<SmBrep*> sBreps;
		SmTArray<long> sBooleanTrees;
        
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/cci_bug_rhino1.smp"));
        SmBrepData::ReadPartFromFile(sContext, _T("../../TestFiles/pt_TestFiles/cci_bug_rhino1.smp"), sCurves,sSurfaces,sBooleanTrees,sBreps,SM_ASCII );

        SmCurve *pCurve1 = sCurves[0];
        SmCurve *pCurve2 = sCurves[1];
        SmAxis2Placement sA2P;
        sA2P.Translate(SmVector3d(0,0,0));
        SER(pCurve1->Transform(sA2P));
        SER(pCurve2->Transform(sA2P));
#ifdef SM_GFX_CODE
        if (smGet_DoGraphics()) {
            smgfx_Erase();
            smgfx_SetLook(1, 2, 0, 0, 0); pCurve1->Draw(); sm_GraphicsLoop();
            smgfx_SetLook(3, 4, 0, 0, 1); pCurve2->Draw(); sm_GraphicsLoop();
            sm_GraphicsLoop();
        }
#endif
        ULONG lNumFound;
        SER(my_test_GlobalCC_intersect(*pCurve1,*pCurve2,lNumFound));
        SM_ASSERT(lNumFound == 2);
    }
    return SM_SUCCESS;
}

/***********************************************************************
PURPOSE --- build a SmAxis2Placement ref frame at
  origin =  -2,0,0 
  rotated from the global orientation by .1 rad about the axis (1,1,1)  

USAGE NOTES ---
***********************************************************************/
static void my_create_refframe
  (SmAxis2Placement& sA2P)
{
  SmPoint3d sOrigin(-2, 0, 0);
  sA2P.SetCanonical(sOrigin,
                    SmVector3d(1,0,0),
                    SmVector3d(0,1,0));
  SmVector3d sRotAx(1, 1, 1);
  sRotAx.Unitize();
  sA2P.RotateAboutAxisAtPoint(0.1,
                              sOrigin, 
                              sRotAx);

} // end my_create_refframe

/***********************************************************************
PURPOSE --- read half sphere Brep from file ../../TestFiles/pt_TestFiles/surf_sphere150.smb
   and set output to a copy of the contained half sphere surface

USAGE NOTES ---
***********************************************************************/
static SmStatus my_create_test_half_sphere_BSS
  (const SmContext   & crContext, // in :
   SmBSplineSurface *& rpBSSArg)  // out: 
{
	 SmBrep* pBrep = new(crContext) SmBrep();
  MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/surf_sphere150.smb"));
	 pBrep->ReadFromFile( crContext, _T("../../TestFiles/pt_TestFiles/surf_sphere150.smb"), SM_ASCII );

  // verify read by checking face count == 1
  SmTArray<SmFace*> sFaces;
  pBrep->GetFaces(sFaces);
  if (sFaces.GetSize() != 1) { SER(SM_ERR); }

  // locals
  SmFace    * pF = sFaces[0];
  SmSurface * pS = pF->GetSurface();
  NER(pS);
  SmBSplineSurface* pBSS = SM_CAST_PTR(SmBSplineSurface, pS);
  NER(pBSS);

  // copy half sphere
  SmBSplineSurface* pCopiedBSS = new (crContext) SmBSplineSurface(*pBSS);
  NER(pCopiedBSS);

  // all done - clean up, set output, and return
  SM_ASSERT(pBrep != NULL) ; delete pBrep ; pBrep = NULL ;
  rpBSSArg = pCopiedBSS;
  return SM_SUCCESS;

} // end my_create_test_half_sphere_BSS

/***********************************************************************
PURPOSE --- return SmSphere surface copied, trimmed, and transformed 
  from the sole surface in the brep stored in file 
  ../../TestFiles/pt_TestFiles/surf_sphere150.smb

USAGE NOTES ---
***********************************************************************/
static SmStatus my_create_test_half_sphere
  (const SmContext & crContext,
   SmSphere *& rpSphereArg)
{
  // get half sphere surface copy from brep in file ../../TestFiles/pt_TestFiles/surf_sphere150.smb
  SmBSplineSurface* pBSS;
  SER(my_create_test_half_sphere_BSS(crContext, pBSS));
  SmObjDelete sCleanBSS( pBSS );

  // define surface subdomain
  SmExtent2d sUVDomain = pBSS->GetNaturalUVDomain();
  SmPoint2d sMin = sUVDomain.Evaluate(0.1, 0.5);        
  SmPoint2d sMax = sUVDomain.Evaluate(0.5, 1.0);
  SmExtent2d sTrimDomain(sMin, sMax);

  // trim surface to subdomain
  SER(pBSS->TrimWithDomain(sTrimDomain));
  // this creates a new Nurb under the BSplineSurface

  // define a transform, origin -2,0,0 and rotated about the axis 1,1,1
  SmPoint3d sOrigin(-2, 0, 0);
  SmAxis2Placement sA2P;
  my_create_refframe(sA2P);

  // transform the trimmmed surface
  pBSS->Transform(sA2P);

  // verify that copied, trimmed, and transformed surface is still known to be a sphere
  SmSphere* pSphere;
  SmBoolean bIsSphere = SmSphere::IsNurbSurfaceSphere(crContext,
                                                      pBSS,
                                                      pSphere);
  if(!bIsSphere) { SER(SM_ERR); }

  // set output 
  rpSphereArg = pSphere;

  // all done
  return SM_SUCCESS;

} // end my_create_test_half_sphere


/***********************************************************************
PURPOSE ---  Create and return an SmCone object by converting the
  output of SmBSplineSurface::CreateConePatch() to an analytic form
  with a call to CopyAndAddAnalytics()

USAGE NOTES --- 
  Cylinder radius         = 2.0, 
           angular domain = [0 to dAngularStretchDeg]
           height         = 5.0
***********************************************************************/
static SmStatus my_create_test_cyl
  (const SmContext & crContext,           // in : 
   double            dAngularStretchDeg,  // in : 
   SmCone         *& rpCyl)               // out: 
{
  SmBSplineSurface *pCylBSS = NULL ;
  rpCyl = NULL;

  // build transform at -2,0,0 rotated .1 radian about 1,1,1
  SmAxis2Placement sA2P;
  my_create_refframe(sA2P);

  SER(SmBSplineSurface::CreateConePatch(crContext,
                                        sA2P,
                                        2.0,                      // bottom rad
                                        2.0,                      // top rad
                                        0.0, dAngularStretchDeg,  // angular domain
                                        5.0,                      // height
                                        SM_CO_QUADRATIC,pCylBSS));
  SmObjDelete sCleanCylBSS( pCylBSS );

  SmSurface *pBSS;
  ////// SmObjDelete sClean3(pBSS);

  // convert to analytics
  SER(pCylBSS->CopyAndAddAnalytics(crContext,pBSS));
  SM_ASSERT_VALID(pBSS) ;
  SmObjDelete sCleanBSS( pBSS );

  // set output
  pCylBSS = SM_CAST_PTR(SmBSplineSurface, pBSS);
  SmCone* pCyl = SM_CAST_PTR(SmCone, pCylBSS);
  rpCyl = pCyl;
  if ( rpCyl != NULL ) sCleanBSS.Clear();

  // all done
  return SM_SUCCESS;

} // end my_create_test_cyl

/***********************************************************************
PURPOSE ---  Build a SmCone Object using SmBSplineSurface::CreateConePatch()
  and converting to analytic form with CopyAndAddAnalytics().

USAGE NOTES ---
***********************************************************************/
static SmStatus my_create_test_cone
  (const SmContext   & crContext,      // in : 
   const SmPoint3d   & sOrigin,        // in : 
   const SmVector3d  & sZAxis,         // in : 
   const SmVector3d  & sXAxis,         // in : 
   double              dBottomRadius,  // in : 
   double              dTopRadius,     // in : 
   double              dStartAngleDeg, // in : 
   double              dEndAngleDeg,   // in : 
   double              dHeight,        // in : 
   SmCone           *& rpCone)         // out: 
{ 
  // axis placement                                           
  SmAxis2Placement sA2P;                   
  sA2P.SetCanonical(sOrigin,               
                    sXAxis,                
                    sZAxis * sXAxis);

  // build BSplineSurface ConePatch
  SmBSplineSurface *pConeBSS = NULL ;
  SER(SmBSplineSurface::CreateConePatch(crContext,
                                        sA2P,
                                        dBottomRadius, dTopRadius,
                                        dStartAngleDeg, 
                                        dEndAngleDeg,           // angular domain
                                        dHeight,                // height
                                        SM_CO_QUADRATIC,        // SM_CO_QUINTIC: is for now never identified as 
                                                                // a cone
                                        pConeBSS));
  SmObjDelete sCleanCBSS( pConeBSS );

  // convert to analytics
  SmSurface *pBSS;
  SER(pConeBSS->CopyAndAddAnalytics(crContext,pBSS));

  // set output
  pConeBSS = SM_CAST_PTR(SmBSplineSurface, pBSS);
  SmCone* pCone = SM_CAST_PTR(SmCone, pConeBSS);
  rpCone = pCone;

  // all done
  return SM_SUCCESS;

} // end my_create_test_cone

/***********************************************************************
PURPOSE --- Build, test, and return a Cylinder as an SmSurfOfExtrusion object.
  Radius=2.0, Height=5.0, AngularDomainDeg = [0 200]

USAGE NOTES ---
***********************************************************************/
static SmStatus my_create_test_extruded_surf2
  (const SmContext    & crContext,            // in : 
   SmSurfOfExtrusion *& rpSurfOfExtrusionArg) // out: 
{
  // create cylinder: rad=2.0, height=5.0, angular domain=[0,dAngularStrechDeg]
  SmCone* pCyl;
  my_create_test_cyl(crContext, 200.0, pCyl);
  SmObjDelete sCleanCyl( pCyl );
  
  // test the surface for type 
  SmSurfOfExtrusion *pSE;
  SmBoolean bIsSE = SmSurfOfExtrusion::IsNurbSurfaceSurfOfExtrusion(crContext,
                                                                    pCyl,
                                                                    pSE);
  if (!bIsSE) { SER(SM_ERR); }

  // set output
  rpSurfOfExtrusionArg = pSE;

  // all done
  return SM_SUCCESS;

} // end my_create_test_extruded_surf2

/***********************************************************************
PURPOSE --- read and copy an extruded SmBSplineSurface from file ../../TestFiles/pt_TestFiles/surf_extrude.smb

USAGE NOTES ---
***********************************************************************/
static SmStatus my_create_test_extruded_surf_BSS
  (const SmContext   & crContext,  // in : 
   SmBSplineSurface *& rpBSS)      // out: 
{
  SmBrep* pBrep = new(crContext) SmBrep();

  // read from file
  MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/surf_extrude.smb"));
  pBrep->ReadFromFile( crContext, _T("../../TestFiles/pt_TestFiles/surf_extrude.smb"), SM_ASCII );

  // verify face count
  SmTArray<SmFace*> sFaces;
  pBrep->GetFaces(sFaces);
  if (sFaces.GetSize() != 1) { SER(SM_ERR); }

  // get only face->Surface
  SmFace* pF = sFaces[0];
  SmSurface* pS = pF->GetSurface();
  NER(pS);

  // make sure surface is of type SmBSplineSurface
  SmBSplineSurface* pBSS = SM_CAST_PTR(SmBSplineSurface, pS);
  NER(pBSS);

  // copy Surface
  SmBSplineSurface* pCopiedBSS = new (crContext) SmBSplineSurface(*pBSS);
  NER(pCopiedBSS);

  // clean up Brep
  SM_ASSERT(pBrep != NULL) ; delete pBrep ; pBrep = NULL ;

  // set output
  rpBSS = pCopiedBSS;

  // all done
  return SM_SUCCESS;

} // end my_create_test_extruded_surf_BSS

/***********************************************************************
PURPOSE --- read, copy, transform, and test SmSurfOfExtrusion from file
                   ../../TestFiles/pt_TestFiles/surf_extrude.smb
USAGE NOTES ---
***********************************************************************/
static SmStatus my_create_test_extruded_surf
  (const SmContext    & crContext,
   SmSurfOfExtrusion *& rpSurfOfExtrusionArg)
{
  //SmCone* pCyl;
  // create cylinder: rad=2.0, height=5.0, angular domain=[0,dAngularStrechDeg]
  //my_create_test_cyl(crContext, 200.0, pCyl);
  // 
  //SmSurfOfExtrusion *pSE;
  //SmBoolean bIsSE = SmSurfOfExtrusion::IsNurbSurfaceSurfOfExtrusion(
  //                                              crContext,
  //                                              pCyl,
  //                                              pSE);
  // if (!bIsSE) { SER(SM_ERR); }
  // rpSurfOfExtrusionArg = pSE;


  // read and copy an extruded SmBSplineSurface from file ../../TestFiles/pt_TestFiles/surf_extrude.smb
  SmBSplineSurface* pBSS;
  SER(my_create_test_extruded_surf_BSS(crContext, pBSS));
  SmObjDelete sCleanBSS( pBSS );

  SmPoint2d sMin(0.1, 0.5);        
  SmPoint2d sMax(0.5, 1.0);

  // define transform
  SmPoint3d sOrigin(-2, 0, 0);
  SmAxis2Placement sA2P;
  sA2P.SetCanonical(sOrigin,
                    SmVector3d(1,0,0),
                    SmVector3d(0,1,0));
  SmVector3d sRotAx(1, 1, 1);
  sRotAx.Unitize();
  sA2P.RotateAboutAxisAtPoint(0.1,
                              sOrigin, 
                              sRotAx);

  // transform the extruded surface
  pBSS->Transform(sA2P);

  // verify that extrusion can be detected as such
  SmSurfOfExtrusion *pSE;
  SmBoolean bIsSE = SmSurfOfExtrusion::IsNurbSurfaceSurfOfExtrusion(crContext,
                                                                    pBSS,
                                                                    pSE);
  if(!bIsSE) { SER(SM_ERR); }
  NER(pSE);

  // set output
  rpSurfOfExtrusionArg = pSE;
  return SM_SUCCESS;

} // end my_create_test_extruded_surf

/***********************************************************************
PURPOSE ---  Create and return an SmCone object by converting the
  output of SmBSplineSurface::CreateConePatch() to an analytic form
  with a call to CopyAndAddAnalytics()

USAGE NOTES --- 
  Cylinder radius         = 2.0, 
           angular domain = [0 to dAngularStretchDeg]
           height         = 5.0
***********************************************************************/
static SmStatus my_create_test_rotcyl
  (const SmContext     & crContext,              // in : 
   double                dAngularStretchDeg,     // in : 
   SmSurfOfRevolution *& rpSurfOfRevolutionArg)  // out: 
{
   // build cylinder, rad = 2.0, height = 5.0, angular domain=[0,dAngularStretchDeg]
   SmCone* pCyl;
   my_create_test_cyl(crContext, dAngularStretchDeg, pCyl);
   SmObjDelete sCleanCyl( pCyl );

   // test that return surface is of type SmCylinder
   SmSurfOfRevolution *pSR;
   SmBoolean bIsSR = SmSurfOfRevolution::IsNurbSurfaceSurfOfRevolution(crContext,
                                                                       pCyl,
                                                                       pSR);
   if(!bIsSR) { SER(SM_ERR); }
   NER(pSR);

   rpSurfOfRevolutionArg = pSR;
   return SM_SUCCESS;

} // end my_create_test_rotcyl

/***********************************************************************
PURPOSE ---  read in and return a SurfOfRevolution from file
  ../../TestFiles/pt_TestFiles/surf_rev.smb that is modified and tested with 
  copy, trim, and translate.

USAGE NOTES ---
***********************************************************************/
static SmStatus my_create_test_rotsurf
  (const SmContext     & crContext,
   SmSurfOfRevolution *& rpSurfOfRevolutionArg)
{

  //SmBSplineSurface *pConeBSS = NULL ;
  //SmPoint3d sOrigin(-2, 0, 0);
  //SmAxis2Placement sA2P;
  //sA2P.SetCanonical(sOrigin,
  //                  SmVector3d(1,0,0),
  //                  SmVector3d(0,1,0));
  //SmVector3d sRotAx(1, 1, 1);
  //sRotAx.Unitize();
  //sA2P.RotateAboutAxisAtPoint(0.1,
  //                            sOrigin, 
  //                            sRotAx);
  //
  //SER(SmBSplineSurface::CreateConePatch(crContext,
  //       sA2P,
  //       1.0, // bottom rad
  //       2.0, // top rad
  //       0.0,200.0, // angular domain
  //       1.0, // height
  //       SM_CO_QUADRATIC,pConeBSS));


  // the following would create a rot surf from a full sphere
  //SmSphere* pSphere;
  //my_create_test_sphere(crContext, pSphere);
  //SmSurfOfRevolution *pSR;
  //SmBoolean bIsSR = SmSurfOfRevolution::IsNurbSurfaceSurfOfRevolution(
  //                                              crContext,
  //                                              pSphere,
  //                                              pSR);
  //   if (!bIsSR) { SER(SM_ERR); }
  // rpSurfOfRevolutionArg = pSR;


  // read file
  SmBrep* pBrep = new(crContext) SmBrep();
  MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/surf_rev.smb"));
  pBrep->ReadFromFile( crContext, _T("../../TestFiles/pt_TestFiles/surf_rev.smb"), SM_ASCII );

  // verify read worked as expected
  SmTArray<SmFace*> sFaces;
  pBrep->GetFaces(sFaces);
  if (sFaces.GetSize() != 1) { SER(SM_ERR); }

  // locals
  SmFace    * pF = sFaces[0];
  SmSurface * pS = pF->GetSurface();
  NER(pS);
  SmBSplineSurface* pBSS = SM_CAST_PTR(SmBSplineSurface, pS);
  NER(pBSS);

  // copy surface
  SmBSplineSurface* pCopiedBSS = new (crContext) SmBSplineSurface(*pBSS);
  SmObjDelete sCleanBSS( pCopiedBSS );
  NER(pCopiedBSS);

  // define a subdomain
  SmPoint2d sMin = pF->GetUVDomain().Evaluate(0.1, 0.5);        
  SmPoint2d sMax = pF->GetUVDomain().Evaluate(0.5, 1.0);
  SmExtent2d sTrimDomain(sMin, sMax);

  // trim to the subdomain
  SER(pCopiedBSS->TrimWithDomain(sTrimDomain));
  // this creates a new Nurb under the BSplineSurface

  // define a reference frame with orig -2,0,0 and rotated about the axis 1,1,1
  SmAxis2Placement sA2P;
  my_create_refframe(sA2P);

  // apply transform to copied surface
  pCopiedBSS->Transform(sA2P);

  // see that the copied, trimmed, and transformed surface is still known to be a SurfOfRevolution
  SmSurfOfRevolution *pSR;
  SmBoolean bIsSR = SmSurfOfRevolution::IsNurbSurfaceSurfOfRevolution
                       (crContext,
                        pCopiedBSS,
                        pSR);
  if(!bIsSR) { SER(SM_ERR); }
  NER(pSR);

  // all done
  SM_ASSERT(pBrep != NULL) ; delete pBrep ; pBrep = NULL ;

  rpSurfOfRevolutionArg = pSR;
  return SM_SUCCESS;

} // end my_create_test_rotsurf

/***********************************************************************
PURPOSE ---

USAGE NOTES ---
***********************************************************************/
static SmStatus my_create_edge
  (SmBrep* pBrepArg, 
   SmBSplineCurve* pCurveArg)
{
    SmExtent1d sIV( pCurveArg->GetNaturalInterval());
    SmPoint3d sStartPt;
    SER(pCurveArg->EvaluatePoint(sIV.GetMin(), sStartPt));
    SmPoint3d sEndPt;
    SER(pCurveArg->EvaluatePoint(sIV.GetMax(), sEndPt));

    SmShell* pShell; SmVertex* pStartVertex;
    SER(pBrepArg->MakeShellVertex(pBrepArg->GetInfiniteRegion(),
                         sStartPt,
                         pShell, 
                         pStartVertex));

    SmVertex* pEndVertex;
    SER(pBrepArg->MakeShellVertex(pBrepArg->GetInfiniteRegion(),
                         sEndPt,
                         pShell, 
                         pEndVertex));


    SmEdge* pEdge;
    SER(pBrepArg->MakeWireEdge(pBrepArg->GetInfiniteRegion(),
                            pStartVertex, 
                            pEndVertex,
                            pCurveArg,
                            sIV,
                            SM_OT_SAME,
                            pEdge));
    return SM_SUCCESS;

} // end my_create_edge

/***********************************************************************
PURPOSE --- This is a regression test for the new class SmPeriodicExtent1d  
            This function is a friend of the class, that's why it can access
            data members directly.                                          
USAGE NOTES ---
***********************************************************************/
#define DELETE_ALL_PARTS(parts) \
{ for (ULONG z=0; z<(parts).GetSize(); z++) \
    { SM_ASSERT((parts)[z] != NULL) ; delete (parts)[z]; (parts)[z] = NULL ; } \
  (parts).ReSet(); }

/***********************************************************************
PURPOSE ---  When a surface has bounding planes - checks that the
  isoparameter boundary curves do indeed lie on those bounding planes
  as intended.

USAGE NOTES ---
bounding plane = when surface has a planar isoparameter boundary curve
                 and the entire surface is to one side or the other of that
                 plane, then that plane is called a bounding plane.
***********************************************************************/
static SmStatus ucheck_bounding_planes
  (SmBSplineSurface* pBSS)
{

  SmPoint3d sP00, sP01, sP10, sP11;
  SmExtent2d sUVDomain( pBSS->GetNaturalUVDomain());

  // evaluate surface corner points
  pBSS->EvaluatePoint(sUVDomain.GetMin(),sP00);
  pBSS->EvaluatePoint(sUVDomain.GetMax(),sP11);
  pBSS->EvaluatePoint(sUVDomain.Evaluate(1.0,0.0),sP10);
  pBSS->EvaluatePoint(sUVDomain.Evaluate(0.0,1.0),sP01);

  SmSurfaceCache *pSC = smsurf_GetSurfaceCache(pBSS);

  // array of boundary planes defined when 3D region boundary curve is
  //   planar and the surface lies completely to one side of the plane.
  SmPatchBoundaryBoundingPlanes &rPBBPL = pSC->GetPatchBoundaryBoundingPlanes();

  // for every bounding plane - verify that planar boundary curves lie on the bounding planes
  for(ULONG ii=0; ii<4; ii++) 
    {

      SmBoolean ePlaneExists;
      SmBoolean eBoundaryDegenerate;
      SmPoint3d sPlaneBase;
      SmVector3d sPlaneNormal;
      SmVector3d *pSamplePoint=NULL ;
      double dTolerance;

      // retrieve surface bounding plane (if there is one) for given boundary index
      SER(rPBBPL.GetNthBoundingPlane(ii, 
                                     ePlaneExists,
                                     eBoundaryDegenerate,
                                     sPlaneBase, 
                                     sPlaneNormal,
                                     pSamplePoint,
                                     dTolerance));

      if(!ePlaneExists)
          continue;

      double dD00 = (sP00 - sPlaneBase).Dot(sPlaneNormal);
      double dD01 = (sP01 - sPlaneBase).Dot(sPlaneNormal);
      double dD11 = (sP11 - sPlaneBase).Dot(sPlaneNormal);
      double dD10 = (sP10 - sPlaneBase).Dot(sPlaneNormal);

      double smallTol = 1e-4;

      // Checking the allocation of indices:
      if (ii==0) 
        {
          SM_ASSERT(dD00 < smallTol && dD10 < smallTol);
        }
      if (ii==1) 
        {
          SM_ASSERT(dD10 < smallTol && dD11 < smallTol);
        }
      if (ii==2) 
        {
          SM_ASSERT(dD10 < smallTol && dD01 < smallTol);
        }
      if (ii==3) 
        {
          SM_ASSERT(dD00 < smallTol && dD01 < smallTol);
        }

      // build 3d isoParameter Curve for given boundary index
      SmBSplineCurve* pBSC;
      SER(rPBBPL.CreateBoundaryCurveForIndex(ii, pBSC));
      SmObjDelete sCleanup(pBSC);

      SmExtent1d sPD( pBSC->GetNaturalInterval() );

      // evalute isoparameter start, mid, and end points
      SmPoint3d sCP0, sCP5, sCP1;
      SER(pBSC->EvaluatePoint(sPD.Evaluate(0.0), sCP0));
      SER(pBSC->EvaluatePoint(sPD.Evaluate(0.5), sCP5));
      SER(pBSC->EvaluatePoint(sPD.Evaluate(1.0), sCP1));

      // check distance of each point to isoParameter plane
      double dDC0 = (sCP0 - sPlaneBase).Dot(sPlaneNormal);
      double dDC5 = (sCP5 - sPlaneBase).Dot(sPlaneNormal);
      double dDC1 = (sCP1 - sPlaneBase).Dot(sPlaneNormal);

      // The curve must be on the plane
      SM_ASSERT(   dDC0 < smallTol 
                && dDC5 < smallTol 
                && dDC1 < smallTol);

    }
  
  // all done        
  return SM_SUCCESS;

} // end ucheck_bounding_planes

/***********************************************************************
PURPOSE --- Build faces into pBrep on the bounding planes of the input
            SmBSplineSurface

USAGE NOTES ---
***********************************************************************/
static SmStatus my_create_faces_from_boundary_planes
  (SmBSplineSurface * pBSS,        // in : Surface to be boxed in by new bounding plane patches
   double             dSurfSize,   // in : size of surface patches added to pBrep
   SmBrep           * pBrep)       // out: Brep that gets the surfaces
{
  // When a surface has bounding planes - checks that the isoparameter boundary 
  // curves do indeed lie on those bounding planes as intended.
  ucheck_bounding_planes(pBSS);

  SmSurfaceCache *pSC = smsurf_GetSurfaceCache(pBSS);

  SmPatchBoundaryBoundingPlanes &rPBBPL =  pSC->GetPatchBoundaryBoundingPlanes();

  // for every posible bounding plane
  for(ULONG ii=0; ii<4; ii++) 
    {

      SmBoolean ePlaneExists;
      SmBoolean eBoundaryDegenerate;
      SmPoint3d sPlaneBase;
      SmVector3d sPlaneNormal;
      SmVector3d *pSamplePoint;
      double dTolerance;

      // see if the surface has a bounding plane for this index
      SER(rPBBPL.GetNthBoundingPlane(ii, 
                                  ePlaneExists,
                                  eBoundaryDegenerate,
                                  sPlaneBase, 
                                  sPlaneNormal,
                                  pSamplePoint,
                                  dTolerance));

      if(!ePlaneExists)
          continue;

      // build coordinate system about bounding plane normal
      SmVector3d  sXAxis;
      SmVector3d  sYAxis;
      SmVector3d  sZAxis;
      SER(sPlaneNormal.MakeUnitOrthoVectors(NULL, sXAxis, sYAxis, sZAxis));

      // build 4 points on the bounding plane centered on the PlaneBase point
      SmPoint3d sPt00( sPlaneBase - dSurfSize*sYAxis - dSurfSize*sZAxis);
      SmPoint3d sPt01( sPlaneBase - dSurfSize*sYAxis + dSurfSize*sZAxis);
      SmPoint3d sPt11( sPlaneBase + dSurfSize*sYAxis + dSurfSize*sZAxis);
      SmPoint3d sPt10( sPlaneBase + dSurfSize*sYAxis - dSurfSize*sZAxis);

      // Create planar patch between the 4 corners
      SmBSplineSurface* pPlane;
      SER(SmBSplineSurface::CreateBilinearSurface(*pBSS->GetContext(),
                                                  sPt00, sPt10, sPt01, sPt11, 
                                                  pPlane));

      // insert face into pBrep as a Face
      SmFace* pF;
      SER(pBrep->CreateFaceFromSurface(pPlane,
                                       pPlane->GetNaturalUVDomain(),
                                       pF));
    } // end iter every bounding plane

  // all done
  return SM_SUCCESS;

} // end my_create_faces_from_boundary_planes

/***********************************************************************
PURPOSE --- Run a specific test (nTest) or all tests in turn 
            (nTest omitted or -1)

USAGE NOTES --- 
***********************************************************************/
PT_EXPORT SmStatus my_ssi_analytic_demo( )
{

  SmContext sContext;
   TCHAR sBuff[SM_TBLOCK_SIZE];

  for( ULONG nTest = 1; nTest <= 17; nTest++) 
  {

  switch (nTest) 
    {
  case 1: // intersect SmSurfOfRevolution with SmSphere
    {
      SM_SPRINTF( sBuff,_T("%s"), _T( "\nEntered my_ssi_analytic_demo: Test 1 SurfOfRevolution X Sphere \n" ) );
      MYPRINTF( sBuff );

      SmFace* pF;

      // read in and test a SurfOfRevolution from  file ../../TestFiles/pt_TestFiles/surf_rev.smb
      SmSurfOfRevolution* pSR1;
      SER( my_create_test_rotsurf( sContext, pSR1 ) );

      // use the surf to build a one face Brep
      SmBrep *pBrep = new (sContext) SmBrep();
      SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks

      SER( pBrep->CreateFaceFromSurface( pSR1, pSR1->GetNaturalUVDomain(), pF ) );

      // get SurfOfRevolution placement
      const SmAxis2Placement & sSR1RF = pSR1->GetPosition();
      SmVector3d sZAxis( sSR1RF.GetZAxis() );
      SmVector3d sXAxis( sSR1RF.GetXAxis() );
      SmPoint3d sOrigin( sSR1RF.GetOrigin() );

      // read in and test a SmSphere from file ../../TestFiles/pt_TestFiles/surf_sphere150.smb
      SmSphere* pSphere;
      SER( my_create_test_half_sphere( sContext, pSphere ) );

      // sphere placement
      const SmAxis2Placement& sSphereRF = pSphere->GetPosition();
      SmVector3d sSphereXAxis( sSphereRF.GetXAxis() );
      SmPoint3d sSphereOrigin( sSphereRF.GetOrigin() );

      // rotate the sphere
      SmAxis2Placement sXform;
      sXform.RotateAboutAxisAtPoint( SM_PI, sSphereOrigin, sSphereXAxis );
      pSphere->Transform( sXform, NULL );

      // insert sphere into pBrep
      SER( pBrep->CreateFaceFromSurface( pSphere, pSphere->GetNaturalUVDomain(), pF ) );

      // global intersect arguments
      SmBoolean bUseSurfaceEdges[2];
      bUseSurfaceEdges[0] = TRUE;
      bUseSurfaceEdges[1] = TRUE;
      SmTArray<double> sDeviations;
      SmTArray<SmTsectCurveType> sCurveTypes;
      SmTArray<SmCurve*> s3DCurves;
      SmApproxTol3d dApproxTol = 0.0001;

      // intersect the SurfOfRevolution with the Sphere
      SER( pSR1->GlobalSurfaceIntersect( sContext,
           pSR1->GetNaturalUVDomain(),
           *pSphere,
           pSphere->GetNaturalUVDomain(),
           bUseSurfaceEdges,
           &dApproxTol,
           NULL,
           &s3DCurves,
           NULL, // &sUVCurves1,
           NULL, // &sUVCurves2,
           &sCurveTypes,
           &sDeviations ) );

      // check solution count - should be 1
      ULONG nSolutions = s3DCurves.GetSize();
      SM_SPRINTF( sBuff, _T( "\n Number of Solutions: %ld \n" ), nSolutions );
      MYPRINTF( sBuff );
      SM_ASSERT( nSolutions == 0 );

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
      {
        ULONG ii;
        smgfx_Erase();
        smgfx_SetLook( 1, 2, 0, 0, 1 ); pSR1->DrawUV( 8, 8, FALSE, NULL, TRUE ); sm_GraphicsLoop();
        smgfx_SetLook( 1, 2, 0, 1, 0 ); pSphere->DrawUV( 8, 8, FALSE, NULL, TRUE ); sm_GraphicsLoop();
        smgfx_SetLook( 3, 4, 1, 0, 0 ); for(ii = 0; ii < s3DCurves.GetSize(); ii++)
        {
          s3DCurves[ii]->Draw(); sm_GraphicsLoop();
        }
        sm_GraphicsLoop();
      }
#endif
      // insert every solution curve into pBrep as an edge
      for(ULONG ii = 0; ii < nSolutions; ii++)
      {
        SmCurve* pC = s3DCurves[ii];
        SmBSplineCurve* pBSC = SM_CAST_PTR( SmBSplineCurve, pC );
        NER( pBSC );

        SM_ASSERT( pBSC->IsDegenerate() );

        my_create_edge( pBrep, pBSC );
      }

      // inform the public - test the results
      pBrep->Dump();
      pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
      {
        pBrep->Draw(); sm_GraphicsLoop();
      }
#endif

    }
      break;
  case 2: // intersect 2 SmSurfOfRevolution objects representing cylinders
      {
          SM_SPRINTF(sBuff,_T("%s"), _T("\nEntered my_ssi_analytic_demo: Test 2 Rot Surf 2 regions! \n"));
          MYPRINTF(sBuff);

          // build 1st Cylinder as SmSurfOfRevolution: radius=2.0, height=5.0, analDomain=[0,240]
          SmSurfOfRevolution* pRS1;
          my_create_test_rotcyl(sContext, 240.0, pRS1);

          // use 1st Cylinder to make a single face Brep
          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks
          SmFace* pF;
          SER(pBrep->CreateFaceFromSurface(pRS1, pRS1->GetNaturalUVDomain(),  pF));

          // build 2nd cylinder as SmSurfOfRevolution
          SmSurfOfRevolution* pRS2;
          my_create_test_rotcyl(sContext, 240.0, pRS2);

          // might extract from the surface itself, but that's somewhat cumbersome

          // build transform at -2,0,0 rotated .1 radian about 1,1,1
          SmAxis2Placement sA2P;
          my_create_refframe(sA2P);

          SmVector3d sZAxis = sA2P.GetZAxis();

          SmAxis2Placement sTransfRF;
          sTransfRF.Translate( 4.0 * sZAxis );
          sTransfRF.RotateAboutAxisAtPoint(SM_PI, sA2P.GetOrigin(), sZAxis);

          // transform cylinder 2 and insert it into the same pBrep
          pRS2->Transform(sTransfRF,NULL);
          SER(pBrep->CreateFaceFromSurface(pRS2, pRS2->GetNaturalUVDomain(), pF));


          // globalIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE;
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes;
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;

          // intersect paif of SmSurfOfRevolution (cylinder objects)
          SER(pRS1->GlobalSurfaceIntersect(sContext, 
                                           pRS1->GetNaturalUVDomain(),
                                           *pRS2,
                                           pRS2->GetNaturalUVDomain(),
                                           bUseSurfaceEdges,
                                           &dApproxTol,
                                           NULL,
                                           &s3DCurves,
                                           NULL, // &sUVCurves1,
                                           NULL, // &sUVCurves2,
                                           &sCurveTypes,
                                           &sDeviations));

          // verify solution output count
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff);

          SM_ASSERT(nSolutions == 8); 

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pRS1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pRS2->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif
          // insert a copy of each solution edge into the pBrep
          for(ULONG ii=0; ii<s3DCurves.GetSize();ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);
              NER(pBSC);               

              // otherwise nothing can be seen
              SmAxis2Placement sFrame;
              sFrame.Translate( SmVector3d(.05, .05, .05) );
              pBSC->Transform( sFrame,NULL);

              my_create_edge(pBrep, pBSC);
            }

          // inform the public - test results
          pBrep->Dump();
          pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif
      }
      break;
  case 3: // intersect pair of SmSurfOfExtrusion surfaces representing cylinders
      {

          SM_SPRINTF(sBuff,_T("%s"), _T("\nEntered my_ssi_analytic_demo: Test 3 Extruded Surface Region \n"));
          MYPRINTF(sBuff);

          // Build, test, and return a Cylinder as an SmSurfOfExtrusion object. Radius=2.0, Height=5.0, AngularDomainDeg = [0 200]
          SmSurfOfExtrusion* pSE1;
          SER(my_create_test_extruded_surf2(sContext, pSE1));
          SM_ASSERT_VALID(pSE1) ;

          // insert cylinder into pBrep as a face
          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks
          SmFace* pF;
          SER(pBrep->CreateFaceFromSurface(pSE1, pSE1->GetNaturalUVDomain(), pF));


          // Build, test, and return a Cylinder as an SmSurfOfExtrusion object. Radius=2.0, Height=5.0, AngularDomainDeg = [0 200]
          SmSurfOfExtrusion* pSE2;
          SER(my_create_test_extruded_surf2(sContext, pSE2));
          SM_ASSERT_VALID(pSE2) ;

          // define transform
          SmAxis2Placement sA2P;
          my_create_refframe(sA2P);
          SmVector3d sZAxis = sA2P.GetZAxis();

          SmAxis2Placement sTransfRF;
          sTransfRF.Translate( 3.0* sZAxis );
          sTransfRF.RotateAboutAxisAtPoint(200.0*SM_PI/180., 
                                      sA2P.GetOrigin(), 
                                      sZAxis);

          // transform the 2nd surface
          pSE2->Transform(sTransfRF,NULL);
          SM_ASSERT_VALID(pSE2) ;

          // insert 2nd cylinder into pBrep
          SER(pBrep->CreateFaceFromSurface(pSE2, pSE2->GetNaturalUVDomain(), pF));


          // GlobalSurfaceIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE;
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes;
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;

          // intersect the SmSurfOfExtrusion (representing cylinders)
          SER(pSE1->GlobalSurfaceIntersect(sContext, 
                                           pSE1->GetNaturalUVDomain(),
                                           *pSE2,
                                           pSE2->GetNaturalUVDomain(),
                                           bUseSurfaceEdges,
                                           &dApproxTol,
                                           NULL,
                                           &s3DCurves,
                                           NULL, // &sUVCurves1,
                                           NULL, // &sUVCurves2,
                                           &sCurveTypes,
                                           &sDeviations));


          // verify the solution counts
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff);

          SM_ASSERT(nSolutions == 5); 

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pSE1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pSE2->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif
          // insert each intersection curve result into Brep as an edge
          for(ULONG ii=0; ii<s3DCurves.GetSize();ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);
              NER(pBSC);               

              // otherwise nothing can be seen
              SmAxis2Placement sFrame;
              sFrame.Translate( SmVector3d(.05, .05, .05) );
              pBSC->Transform( sFrame,NULL);

              my_create_edge(pBrep, pBSC);
            }

          // inform the public - test results
          pBrep->Dump();
          pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif
      }
      break;
  case 4: // intersect pair of SmSurfOfRevolution surfaces from file after exercising the Directrix function
      {

          SM_SPRINTF(sBuff,_T("%s"), _T("\nEntered my_ssi_analytic_demo: Test 4 SurfOfRevolution \n"));
          MYPRINTF(sBuff);

          // read in and test a SmSurfOfRevolution from  file ../../TestFiles/pt_TestFiles/surf_rev.smb
          SmSurfOfRevolution* pSR1;
          SER(my_create_test_rotsurf(sContext, pSR1));

          // insert SmSurfOfRevolution into pBrep as a Face
          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks
          SmFace* pF;
          SER(pBrep->CreateFaceFromSurface(pSR1, pSR1->GetNaturalUVDomain(), pF));

          // read in and test a 2nd SurfOfRevolution from  file ../../TestFiles/pt_TestFiles/surf_rev.smb
          SmSurfOfRevolution* pSR2;
          SER(my_create_test_rotsurf(sContext, pSR2));

          SmVector3d sZAxis = pSR1->GetPosition().GetZAxis();
          SmVector3d sXAxis = pSR1->GetPosition().GetXAxis();

          // extract Directrix Curve 1 from 2nd Surface
          SmBSplineCurve* pDirectrix1;
          SmPoint3d sAxisStartPoint; double dDirectrix1Radius;
          SER(pSR2->CreateDirectrixAtGeneratorParam(sContext,
                                                    pSR2->GetGeneratorStartParam(),
                                                    pDirectrix1, 
                                                    dDirectrix1Radius, 
                                                    sAxisStartPoint));
          SmObjDelete sCleanup1(pDirectrix1);

          // extract Directrix Curve 1 from 2nd Surface
          SmBSplineCurve* pDirectrix2;
          SmPoint3d sAxisEndPoint; 
          double dDirectrix2Radius;
          SER(pSR2->CreateDirectrixAtGeneratorParam(sContext,
                                                    pSR2->GetGeneratorEndParam(),
                                                    pDirectrix2, 
                                                    dDirectrix2Radius, 
                                                    sAxisEndPoint));
          SmObjDelete sCleanup2(pDirectrix2);

          // check the assumptions about the origin, Z direction 
          {
              const SmAxis2Placement& sSR1Ref = pSR1->GetPosition();
              SmPoint3d sSROrigin = sSR1Ref.GetOrigin();
              SmVector3d sSRZAxis = sSR1Ref.GetZAxis();

              // double dDiscr = (sSROrigin - sAxisStartPoint).Length();
              // The axis start point must be the origin: well, it must 
              // not, experiment shows.
              //if(dDiscr > 1e-4) { SER(SM_ERR); } // is 1e-4 a small number?
                                                   // for SR1 it seems so.

              SmVector3d sAxisDir(sAxisEndPoint - sAxisStartPoint);
              double dDotDiscr = sAxisDir.Dot(sSRZAxis);
              
              if(dDotDiscr < 0.9) { SER(SM_ERR); }
          }

          // build transforms
          SmAxis2Placement sTranslateRF;
          sTranslateRF.Translate( 0.5*(sAxisEndPoint - sAxisStartPoint));

          SmAxis2Placement sMirroringRF;
          sMirroringRF.RotateAboutAxisAtPoint(SM_PI,sAxisStartPoint, sXAxis);

          SmAxis2Placement sRotatingRF;
          sRotatingRF.RotateAboutAxisAtPoint( 40. * SM_PI / 180., 
                                              sAxisStartPoint, sZAxis);


          // tranform Surface2 (a mirror, a rotate, and a translate)
          pSR2->Transform(sMirroringRF,NULL); // mirror (z axis will be opposite)
          pSR2->Transform(sRotatingRF,NULL);  // rotate 
                                              //    (make angle domains intersect)
          pSR2->Transform(sTranslateRF,NULL); // translate 

          // insert Surface2 as a face in pBrep
          SER(pBrep->CreateFaceFromSurface(pSR2,
                                           pSR2->GetNaturalUVDomain(),
                                           pF));

          // GlobalSurfaceIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE;
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes;
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;

          // Intersect pair of SmSurfOfRevolution objects
          SER(pSR1->GlobalSurfaceIntersect(sContext, 
                                           pSR1->GetNaturalUVDomain(),
                                           *pSR2,
                                           pSR2->GetNaturalUVDomain(),
                                           bUseSurfaceEdges,
                                           &dApproxTol,
                                           NULL,
                                           &s3DCurves,
                                           NULL, // &sUVCurves1,
                                           NULL, // &sUVCurves2,
                                           &sCurveTypes,
                                           &sDeviations));

          // verify the number of solutions
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff);
          SM_ASSERT(nSolutions == 1); 

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pSR1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pSR2->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif

          // insert each solution curve as an edge into pBrep
          for(ULONG ii=0; ii<s3DCurves.GetSize();ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);
              NER(pBSC);               
              my_create_edge(pBrep, pBSC);
            }

          // inform the public - test results
          pBrep->Dump();
          pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif

      }
      break;
  case 5: // intersect pair of SmSurfOfExtrusion objects read in from file ../../TestFiles/pt_TestFiles/surf_extrude.smb
      {
          SM_SPRINTF(sBuff,_T("%s"),_T("\nEntered my_ssi_analytic_demo: Test 5 Extruded Surface \n"));
          MYPRINTF(sBuff);

          // read, copy, transform, and test SmSurfOfExtrusion from file ../../TestFiles/pt_TestFiles/surf_extrude.smb
          SmSurfOfExtrusion* pSE1;
          SER(my_create_test_extruded_surf(sContext, pSE1));

          // insert SmSurfOfExtrusion into pBrep as a face
          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks
          SmFace* pF;
          SER(pBrep->CreateFaceFromSurface(pSE1,
                                           pSE1->GetNaturalUVDomain(),
                                           pF));

          //double dStartH = pSE1->GetBotHeight();
          //double dEndH = pSE1->GetTopHeight();
          //if(dEndH < dStartH) { SER(SM_ERR); } 
          //
          //SmBSplineCurve* pBSC0;
          //SER(pSE1->CreateGeneratorFromDistance(
          //                   dEndH - (0.1 * dEndH-dStartH),
          //                   pBSC0));
          //NER(pBSC0);
          //
          //my_create_edge(pBrep, pBSC0);
          //
          //
          //SmBSplineCurve* pBSC1;
          //SER(pSE1->CreateGeneratorFromDistance( 1, //  distance
          //                                      pBSC1));
          //NER(pBSC1);
          //my_create_edge(pBrep, pBSC1);
          //
          //SmBSplineCurve* pBSC2;
          //SER(pSE1->CreateGeneratorFromDistance(4, //  distance
          //                                     pBSC2));
          //NER(pBSC2);
          //my_create_edge(pBrep, pBSC2);


          // read, copy, transform, and test 2nd SmSurfOfExtrusion from file ../../TestFiles/pt_TestFiles/surf_extrude.smb
          SmSurfOfExtrusion* pSE2;
          SER(my_create_test_extruded_surf(sContext, pSE2));

           // shift the second one perp to its extrusion vector
          SmVector3d sExtrusionVect( pSE2->GetExtrusionVector());
          SmVector3d sSHV1, sSHV2, sSHV3;
          SER(sExtrusionVect.MakeUnitOrthoVectors(NULL, sSHV1, sSHV2, sSHV3));

          double dShiftValue1 = 5; double dShiftValue2 = 0;
          double dShiftValue3 = 1;
          SmAxis2Placement sShiftRF;
          sShiftRF.Translate(dShiftValue1 * sSHV1 + 
                             dShiftValue2 * sSHV2 + 
                             dShiftValue3 * sSHV3);
   
          pSE2->Transform(sShiftRF,NULL);

          // add 2nd SmSurfOfExtrsion to pBrep as a face
          SER(pBrep->CreateFaceFromSurface(pSE2,
                                           pSE2->GetNaturalUVDomain(),
                                           pF));


          // GlobalSurfaceIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE;
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes;
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;

          // intersect pair of SmSurfOfExtrusion surfaces
          SER(pSE1->GlobalSurfaceIntersect(sContext, 
                                           pSE1->GetNaturalUVDomain(),
                                           *pSE2,
                                           pSE2->GetNaturalUVDomain(),
                                           bUseSurfaceEdges,
                                           &dApproxTol,
                                           NULL,
                                           &s3DCurves,
                                           NULL, // &sUVCurves1,
                                           NULL, // &sUVCurves2,
                                           &sCurveTypes,
                                           &sDeviations));

          // verify solution count
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff);
          SM_ASSERT(nSolutions == 3); 

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pSE1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pSE2->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif

          // add each solution curve to pBrep as a Edge
          for(ULONG ii=0; ii<s3DCurves.GetSize();ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);
              NER(pBSC);               
              my_create_edge(pBrep, pBSC);
            }

          // inform the public - test results
          pBrep->Dump();
          pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif

      }
      break;
  case 6: // intersect a pair of SmSurfOfExtrusion cylinders 
      {
          SM_SPRINTF(sBuff,_T("%s"),_T("\nEntered my_ssi_analytic_demo: Test 6 Extruded Surface line \n"));
          MYPRINTF(sBuff);

          // Build, test, and return a Cylinder as an SmSurfOfExtrusion object. Radius=2.0, Height=5.0, AngularDomainDeg = [0 200]
          SmSurfOfExtrusion* pSE1;
          SER(my_create_test_extruded_surf2(sContext, pSE1));

          // insert cylinder into pBrep as a face
          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks
          SmFace* pF;
          SER(pBrep->CreateFaceFromSurface(pSE1,
                                           pSE1->GetNaturalUVDomain(),
                                           pF));

          // Build, test, and return a Cylinder as an SmSurfOfExtrusion object. Radius=2.0, Height=5.0, AngularDomainDeg = [0 200]
          SmSurfOfExtrusion* pSE2;
          SER(my_create_test_extruded_surf2(sContext, pSE2));

          // build transfrom
          SmVector3d sExtrusionV = pSE2->GetExtrusionVector();
          
          SmAxis2Placement sA2P;
          my_create_refframe(sA2P);
          SmVector3d sZAxis = sA2P.GetZAxis();

          if(!sZAxis.IsParallelTo(sExtrusionV, 1e-4)) { SER(SM_ERR); }


          SmAxis2Placement sTransfRF;
          sTransfRF.Translate( 3.0* sZAxis );
          sTransfRF.RotateAboutAxisAtPoint(0.5*SM_PI, 
                                      sA2P.GetOrigin(), 
                                      sZAxis);

          // trasnform 2nd cylinder
          pSE2->Transform(sTransfRF,NULL);

          sExtrusionV = pSE2->GetExtrusionVector();
          if(!sZAxis.IsParallelTo(sExtrusionV, 1e-4)) { SER(SM_ERR); }
          
          // insert 2nd cylnder into pBrep as a face
          SER(pBrep->CreateFaceFromSurface(pSE2,
                                           pSE2->GetNaturalUVDomain(),
                                           pF));

          // GlobalSurfaceIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE;
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes;
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;

          // intersect pair of SmSurfOfExtrusion cylinder surfaces
          SER(pSE1->GlobalSurfaceIntersect(sContext, 
                                           pSE1->GetNaturalUVDomain(),
                                           *pSE2,
                                           pSE2->GetNaturalUVDomain(),
                                           bUseSurfaceEdges,
                                           &dApproxTol,
                                           NULL,
                                           &s3DCurves,
                                           NULL, // &sUVCurves1,
                                           NULL, // &sUVCurves2,
                                           &sCurveTypes,
                                           &sDeviations));

          // verify solution counts
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff);
          SM_ASSERT(nSolutions == 4);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pSE1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pSE2->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif
          // insert every solution curve into pBrep as an edge
          for(ULONG ii=0; ii<s3DCurves.GetSize();ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);
              NER(pBSC);               

              // otherwise nothing can be seen
              SmAxis2Placement sFrame;
              sFrame.Translate( SmVector3d(.05, .05, .05) );
              pBSC->Transform( sFrame,NULL);

              my_create_edge(pBrep, pBSC);
            }

          // inform the public - test the results
          pBrep->Dump();
          pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif

      }
      break;
  case 7: // intersect pair of SmSurfOfRevolution cylinders
      {

          SM_SPRINTF(sBuff,_T("%s"),_T("\nEntered my_ssi_analytic_demo: Test 7 Rot Surf \n"));
          MYPRINTF(sBuff);


          // build 1st Cylinder as SmSurfOfRevolution: radius=2.0, height=5.0, analDomain=[0,180]
          SmSurfOfRevolution* pRS1;
          my_create_test_rotcyl(sContext, 180.0, pRS1);

          // insert surface into pBrep as a face
          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks
          SmFace* pF;
          SER(pBrep->CreateFaceFromSurface(pRS1,
                                     pRS1->GetNaturalUVDomain(),
                                     pF));

          // build 2nd Cylinder as SmSurfOfRevolution: radius=2.0, height=5.0, analDomain=[0,180]
          SmSurfOfRevolution* pRS2;
          my_create_test_rotcyl(sContext, 180.0, pRS2);

          // Build transform
          SmAxis2Placement sA2P;
          my_create_refframe(sA2P);
          SmVector3d sZAxis = sA2P.GetZAxis();
          SmAxis2Placement sTransfRF;
          sTransfRF.Translate( 5.0* sZAxis );

          // transform 2nd Surface
          pRS2->Transform(sTransfRF,NULL);

          // add 2nd surface to pBrep as a face
          SER(pBrep->CreateFaceFromSurface(pRS2,
                                           pRS2->GetNaturalUVDomain(),
                                           pF));


          // GlobalSurfaceIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE;
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes;
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;


          // intersect pair of SmSurfOfRevolution objects
          SER(pRS1->GlobalSurfaceIntersect(sContext, 
                                           pRS1->GetNaturalUVDomain(),
                                           *pRS2,
                                           pRS2->GetNaturalUVDomain(),
                                           bUseSurfaceEdges,
                                           &dApproxTol,
                                           NULL,
                                           &s3DCurves,
                                           NULL, // &sUVCurves1,
                                           NULL, // &sUVCurves2,
                                           &sCurveTypes,
                                           &sDeviations));

          // verify solution count
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff);
          SM_ASSERT(nSolutions == 1);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pRS1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pRS2->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif
          // add solution curves to pBrep as Edges
          for(ULONG ii=0; ii<s3DCurves.GetSize();ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);
              NER(pBSC);               

              // otherwise nothing can be seen
              SmAxis2Placement sFrame;
              sFrame.Translate( SmVector3d(.05, .05, .05) );
              pBSC->Transform( sFrame,NULL);

              my_create_edge(pBrep, pBSC);
            }

          // inform the public - test results
          pBrep->Dump();
          pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif

      }
      break;

  case 8: // intersect SmCone and SmSurfOfRevolution
      {

          SM_SPRINTF(sBuff,_T("%s"),_T("\nEntered my_ssi_analytic_demo: Test 8 SurfOfRevolution X Cone \n"));
          MYPRINTF(sBuff);

          SmCone* pCone;
          SmFace* pF;

          // read in and test a SurfOfRevolution from  file ../../TestFiles/pt_TestFiles/surf_rev.smb
          SmSurfOfRevolution* pSR1;
          SER(my_create_test_rotsurf(sContext, pSR1));

          // insert surface into pBrep as a face
          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks
          SER(pBrep->CreateFaceFromSurface(pSR1,
                                           pSR1->GetNaturalUVDomain(),
                                           pF));


          double dStartAngleDeg = pSR1->GetStartAngleDeg();
          double dEndAngleDeg   = pSR1->GetEndAngleDeg();
          const SmAxis2Placement & sSR1RF = pSR1->GetPosition();
          SmVector3d sZAxis(sSR1RF.GetZAxis());
          SmVector3d sXAxis(sSR1RF.GetXAxis());
          SmPoint3d sOrigin(sSR1RF.GetOrigin());
    
          double dShiftValue = -10.0;

          // Build a SmCone Object using SmBSplineSurface::CreateConePatch() and converting to analytic form with CopyAndAddAnalytics().
          SER(my_create_test_cone(sContext,
                                 sOrigin + dShiftValue * sZAxis,
                                 sZAxis,
                                 sXAxis,
                                 6.0, // dBottomRadius, 
                                 3.0, // dTopRadius,
                                 dStartAngleDeg + 0.5*(dEndAngleDeg-dStartAngleDeg),
                                 dEndAngleDeg + 0.5*(dEndAngleDeg-dStartAngleDeg),
                                 15.0, // dHeight,
                                 pCone));
          

          // add 2nd surface to pBrep as face
          SER(pBrep->CreateFaceFromSurface(pCone,
                                           pCone->GetNaturalUVDomain(),
                                           pF));

          // GlobalSurfaceIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE;
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes;
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;

          // intersect SmCone and SmSurfOfRevolution
          SER(pSR1->GlobalSurfaceIntersect(sContext, 
                                           pSR1->GetNaturalUVDomain(),
                                           *pCone,
                                           pCone->GetNaturalUVDomain(),
                                           bUseSurfaceEdges,
                                           &dApproxTol,
                                           NULL,
                                           &s3DCurves,
                                           NULL, // &sUVCurves1,
                                           NULL, // &sUVCurves2,
                                           &sCurveTypes,
                                           &sDeviations));

          // verify solution count
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff);
          SM_ASSERT(nSolutions == 2);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pSR1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pCone->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif

          // add solution curves to pBrep as Edges
          for(ULONG ii=0; ii<s3DCurves.GetSize();ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);
              NER(pBSC);               
              my_create_edge(pBrep, pBSC);
            }

          // inform the public - test results
          pBrep->Dump();
          pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif

      }
      break;
  case 9: // intersect SmSurfOfRevolution and SmSphere
      {
          SM_SPRINTF(sBuff,_T("%s"),_T("\nEntered my_ssi_analytic_demo: Test 9 SurfOfRevolution X Sphere \n"));
          MYPRINTF(sBuff);

          SmFace* pF;

          // read in and test a SurfOfRevolution from  file ../../TestFiles/pt_TestFiles/surf_rev.smb
          SmSurfOfRevolution* pSR1;
          SER(my_create_test_rotsurf(sContext, pSR1));

          // insert surface into pBrep as a face
          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks
          SER(pBrep->CreateFaceFromSurface(pSR1,
                                           pSR1->GetNaturalUVDomain(),
                                           pF));


          // build transform
          const SmAxis2Placement & sSR1RF = pSR1->GetPosition();
          SmVector3d sZAxis(sSR1RF.GetZAxis());
          SmVector3d sXAxis(sSR1RF.GetXAxis());
          SmPoint3d sOrigin(sSR1RF.GetOrigin());
    
          // read in and test a SmSphere from file ../../TestFiles/pt_TestFiles/surf_sphere150.smb
          SmSphere* pSphere;
          SER(my_create_test_half_sphere(sContext, pSphere));
          
        
          // build transform
          const SmAxis2Placement& sSphereRF = pSphere->GetPosition();
          SmVector3d sSphereZAxis(sSphereRF.GetZAxis());
          SmPoint3d sSphereOrigin(sSphereRF.GetOrigin());
     
          SmVector3d sZZ = sSphereZAxis * sZAxis;
    
          SmAxis2Placement sXform;
          //sXform.RotateAboutAxisAtPoint(sZZ.Length(), sSphereOrigin, sZZ);
          //sXform.Translate(sOrigin - sSphereOrigin);
          //double dShift = 3;
          //sXform.Translate( dShift * sZAxis);
          // pSphere->Transform(sXform);


          // insert sphere into pBrep as a face
          SER(pBrep->CreateFaceFromSurface(pSphere,
                                           pSphere->GetNaturalUVDomain(),
                                           pF));

          // GlobalSurfaceIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE;
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes;
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;

          // intersect SmSurfOfRevolution and SmSphere
          SER(pSR1->GlobalSurfaceIntersect(sContext, 
                                           pSR1->GetNaturalUVDomain(),
                                           *pSphere,
                                           pSphere->GetNaturalUVDomain(),
                                           bUseSurfaceEdges,
                                           &dApproxTol,
                                           NULL,
                                           &s3DCurves,
                                           NULL, // &sUVCurves1,
                                           NULL, // &sUVCurves2,
                                           &sCurveTypes,
                                           &sDeviations));

          // verify solution count
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff); 
          SM_ASSERT(nSolutions == 0);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pSR1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pSphere->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif

          // add solution curves to pBrep as Edges
          for(ULONG ii=0; ii<s3DCurves.GetSize();ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);
              NER(pBSC);               

              SM_ASSERT(!pBSC->IsDegenerate() );

              my_create_edge(pBrep, pBSC);
            }

          // inform the public - test results
          pBrep->Dump();
          pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif

      }
      break;
  case 10: // intersect pair of SmSurfOfRevolution cylinders 
      {

          SM_SPRINTF(sBuff,_T("%s"),_T("\nEntered my_ssi_analytic_demo: Test 10 Rot Surf FULL REGION \n"));
          MYPRINTF(sBuff);
     

          // build 1st Cylinder as SmSurfOfRevolution: radius=2.0, height=5.0, analDomain=[0,180]
          SmSurfOfRevolution* pRS1;
          my_create_test_rotcyl(sContext, 180.0, pRS1);

          // insert surface into pBrep as a face
          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks
          SmFace* pF;
          SER(pBrep->CreateFaceFromSurface(pRS1,
                                           pRS1->GetNaturalUVDomain(),
                                           pF));

          // build 2nd Cylinder as SmSurfOfRevolution: radius=2.0, height=5.0, analDomain=[0,180]
          SmSurfOfRevolution* pRS2;
          my_create_test_rotcyl(sContext, 180.0, pRS2);

          // Build transform
          SmAxis2Placement sA2P;
          my_create_refframe(sA2P);
          SmVector3d sZAxis = sA2P.GetZAxis();
          SmAxis2Placement sTransfRF;
          sTransfRF.Translate( 4.0* sZAxis );

          // transform 2nd surface
          pRS2->Transform(sTransfRF,NULL);

          // add 2nd surface to pBrep as face
          SER(pBrep->CreateFaceFromSurface(pRS2,
                                           pRS2->GetNaturalUVDomain(),
                                           pF));


          // GlobalSurfaceIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE;
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes;
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;

          // intersect pair of SmSurfOfRevolution cylinders
          SER(pRS1->GlobalSurfaceIntersect(sContext, 
                                           pRS1->GetNaturalUVDomain(),
                                           *pRS2,
                                           pRS2->GetNaturalUVDomain(),
                                           bUseSurfaceEdges,
                                           &dApproxTol,
                                           NULL,
                                           &s3DCurves,
                                           NULL, // &sUVCurves1,
                                           NULL, // &sUVCurves2,
                                           &sCurveTypes,
                                           &sDeviations));

          // verify solution count
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff);
          SM_ASSERT(nSolutions == 4);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pRS1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pRS2->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif

          // add solution curves to pBrep as Edges
          for(ULONG ii=0; ii<s3DCurves.GetSize();ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);
              NER(pBSC);               

              // otherwise nothing can be seen
              SmAxis2Placement sFrame;
              sFrame.Translate( SmVector3d(.05, .05, .05) );
              pBSC->Transform( sFrame,NULL);

              my_create_edge(pBrep, pBSC);
            }


          // inform the public - test results
          pBrep->Dump();
          pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif
      }
      break;
  case 11: // intersect pair of SmSurfOfRevolution cylinders
      {

          SM_SPRINTF(sBuff,_T("%s"),_T("\nEntered my_ssi_analytic_demo: Test 11 Rot Surf partial REGION \n"));
          MYPRINTF(sBuff);
     

          // build 1st Cylinder as SmSurfOfRevolution: radius=2.0, height=5.0, analDomain=[0,180]
          SmSurfOfRevolution* pRS1;
          my_create_test_rotcyl(sContext, 180.0, pRS1);

          // insert surface into pBrep as a face
          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks
          SmFace* pF;
          SER(pBrep->CreateFaceFromSurface(pRS1,
                                           pRS1->GetNaturalUVDomain(),
                                           pF));

          // build 2nd Cylinder as SmSurfOfRevolution: radius=2.0, height=5.0, analDomain=[0,180]
          SmSurfOfRevolution* pRS2;
          my_create_test_rotcyl(sContext, 180.0, pRS2);

          // Build transform
          SmAxis2Placement sA2P;
          my_create_refframe(sA2P);
          SmVector3d sZAxis = sA2P.GetZAxis();
          SmAxis2Placement sTransfRF;
          sTransfRF.Translate( 4.0* sZAxis );
          sTransfRF.RotateAboutAxisAtPoint(0.5*SM_PI, // radians
                                           sA2P.GetOrigin(),
                                           sZAxis);

          // transform 2nd Surface
          pRS2->Transform(sTransfRF,NULL);

          SER(pBrep->CreateFaceFromSurface(pRS2,
                                     pRS2->GetNaturalUVDomain(),
                                     pF));


          // GlobalSurfaceIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE;
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes;
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;

          // intersect pair of SmSurfOfRevolution cylinders 
          SER(pRS1->GlobalSurfaceIntersect(sContext, 
                                           pRS1->GetNaturalUVDomain(),
                                           *pRS2,
                                           pRS2->GetNaturalUVDomain(),
                                           bUseSurfaceEdges,
                                           &dApproxTol,
                                           NULL,
                                           &s3DCurves,
                                           NULL, // &sUVCurves1,
                                           NULL, // &sUVCurves2,
                                           &sCurveTypes,
                                           &sDeviations));

          // verify solution count
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff);
          SM_ASSERT(nSolutions == 4);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pRS1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pRS2->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif

          // add solution curves to pBrep as Edges
          for(ULONG ii=0; ii<s3DCurves.GetSize();ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);
              NER(pBSC);               

              // otherwise nothing can be seen
              SmAxis2Placement sFrame;
              sFrame.Translate( SmVector3d(.05, .05, .05) );
              pBSC->Transform( sFrame );

              my_create_edge(pBrep, pBSC);
            }


          // inform the public - test results
          pBrep->Dump();
          pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif

      }
      break;
  case 12: // intersect pair SmSurfOfRevolution cylinders
      {
          SM_SPRINTF(sBuff,_T("%s"),_T("\nEntered my_ssi_analytic_demo: Test 12 Rot Surf 2 lines \n"));
          MYPRINTF(sBuff);
     

          // build 1st Cylinder as SmSurfOfRevolution: radius=2.0, height=5.0, analDomain=[0,180]
          SmSurfOfRevolution* pRS1;
          my_create_test_rotcyl(sContext, 180.0, pRS1);

          // insert surface into pBrep as a face
          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks
          SmFace* pF;
          SER(pBrep->CreateFaceFromSurface(pRS1,
                                           pRS1->GetNaturalUVDomain(),
                                           pF));

          // build 1st Cylinder as SmSurfOfRevolution: radius=2.0, height=5.0, analDomain=[0,180]
          SmSurfOfRevolution* pRS2;
          my_create_test_rotcyl(sContext, 180.0, pRS2);

          // Build transform
          SmAxis2Placement sA2P;
          my_create_refframe(sA2P); 
          SmVector3d sZAxis = sA2P.GetZAxis(); 
          SmAxis2Placement sTransfRF;
          sTransfRF.Translate( 4.0 * sZAxis );
          sTransfRF.RotateAboutAxisAtPoint(SM_PI,
                                           sA2P.GetOrigin(),
                                           sZAxis);

          // transform 2nd Surface
          pRS2->Transform(sTransfRF,NULL);

          // insert 2nd surface into pBrep as face
          SER(pBrep->CreateFaceFromSurface(pRS2,
                                     pRS2->GetNaturalUVDomain(),
                                     pF));


          // GlobalSurfaceIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE;
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes; 
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;

          // intersect pair SmSurfOfRevolution cylinders
          SER(pRS1->GlobalSurfaceIntersect(sContext, 
                                           pRS1->GetNaturalUVDomain(),
                                           *pRS2,
                                           pRS2->GetNaturalUVDomain(),
                                           bUseSurfaceEdges,
                                           &dApproxTol,
                                           NULL,
                                           &s3DCurves,
                                           NULL, // &sUVCurves1,
                                           NULL, // &sUVCurves2,
                                           &sCurveTypes,
                                           &sDeviations));

          // verify solution count
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff);
          SM_ASSERT(nSolutions == 2);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pRS1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pRS2->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif
          // insert solution curves into pBrep as edges
          for(ULONG ii=0; ii<nSolutions;ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);
              NER(pBSC);               

              // otherwise nothing can be seen
              SmAxis2Placement sFrame;
              sFrame.Translate( SmVector3d(.05, .05, .05) );
              pBSC->Transform( sFrame );

              my_create_edge(pBrep, pBSC);
            }

          // inform the public - test results
          pBrep->Dump();
          pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif

      }
      break;
  case 13: // intersect pair of SmSurfOfExtrusion surfaces
      {
          SM_SPRINTF(sBuff,_T("%s"),_T("\nEntered my_ssi_analytic_demo: Test 13 Extruded Surface points \n"));
          MYPRINTF(sBuff);

          // read, copy, transform, and test SmSurfOfExtrusion from file ../../TestFiles/pt_TestFiles/surf_extrude.smb
          SmSurfOfExtrusion* pSE1;
          SER(my_create_test_extruded_surf(sContext, pSE1));

          // insert surface into pBrep as a face
          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks
          SmFace* pF;
          SER(pBrep->CreateFaceFromSurface(pSE1,
                                           pSE1->GetNaturalUVDomain(),
                                           pF));

          //double dStartH = pSE1->GetBotHeight();
          //double dEndH = pSE1->GetTopHeight();
          //if(dEndH < dStartH) { SER(SM_ERR); } 
          //
          //SmBSplineCurve* pBSC0;
          //SER(pSE1->CreateGeneratorFromDistance(
          //                   dEndH - (0.1 * dEndH-dStartH),
          //                   pBSC0));
          //NER(pBSC0);
          //
          //my_create_edge(pBrep, pBSC0);
          //
          //
          //SmBSplineCurve* pBSC1;
          //SER(pSE1->CreateGeneratorFromDistance( 1, //  distance
          //                                      pBSC1));
          //NER(pBSC1);
          //
          //my_create_edge(pBrep, pBSC1);
          //
          //SmBSplineCurve* pBSC2;
          //SER(pSE1->CreateGeneratorFromDistance(4, //  distance
          //                                     pBSC2));
          //NER(pBSC2);
          //
          //my_create_edge(pBrep, pBSC2);

          // read, copy, transform, and test SmSurfOfExtrusion from file ../../TestFiles/pt_TestFiles/surf_extrude.smb
          SmSurfOfExtrusion* pSE3;
          SER(my_create_test_extruded_surf(sContext, pSE3));

           // shift the second one perp to its extrusion vector
          SmVector3d sExtrusionVect( pSE3->GetExtrusionVector());
          double dExtrHeight = pSE3->GetTopHeight() - pSE3->GetBotHeight();
          SmVector3d sSHV1, sSHV2, sSHV3;
          SER(sExtrusionVect.MakeUnitOrthoVectors(NULL, sSHV1, sSHV2, sSHV3));

          double dShiftValue1 = 5; double dShiftValue2 = 0;
          double dShiftValue3 = 1;
          SmAxis2Placement sShiftRF;
          sShiftRF.Translate( 
              dShiftValue1 * sSHV1 + 
              dShiftValue2 * sSHV2 + 
              dShiftValue3 * sSHV3);
   

          sShiftRF.Translate( (dExtrHeight - dShiftValue1) * sSHV1);

          // transform 2nd Surface
          pSE3->Transform(sShiftRF,NULL);

          // add 2nd surface to pBrep as face
          SER(pBrep->CreateFaceFromSurface(pSE3,
                                           pSE3->GetNaturalUVDomain(),
                                           pF));

          // GlobalSurfaceIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE;
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes;
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;
           
          // intersect pair of SmSurfOfExtrusion surfaces
          SER(pSE1->GlobalSurfaceIntersect(sContext, 
                                           pSE1->GetNaturalUVDomain(),
                                           *pSE3,
                                           pSE3->GetNaturalUVDomain(),
                                           bUseSurfaceEdges,
                                           &dApproxTol,
                                           NULL,
                                           &s3DCurves,
                                           NULL, // &sUVCurves1,
                                           NULL, // &sUVCurves2,
                                           &sCurveTypes,
                                           &sDeviations));

          // verify solution count
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff);
          SM_ASSERT(nSolutions == 3);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pSE1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pSE3->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif

          // add solution curves to pBrep as Edges
          for(ULONG ii=0; ii<s3DCurves.GetSize();ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);

              SM_ASSERT(pBSC->IsDegenerate());

              NER(pBSC);               
              my_create_edge(pBrep, pBSC);
            }

          // inform the public - test results
          pBrep->Dump();
          pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif

      }
      break;

  case 14: // intersect planar patches
      {

          SM_SPRINTF(sBuff,_T("%s"),_T("\nEntered my_ssi_analytic_demo: Test 14 2 planar patches X \n"));
          MYPRINTF(sBuff);

          SmBSplineSurface* pBSS1, *pBSS2;

          SmPoint3d sPt00( 0,  0, 0);
          SmPoint3d sPt01(10,  0, 0);
          SmPoint3d sPt11(10, 10, 0);
          SmPoint3d sPt10( 0, 10, 0);

          // build bilinear surface 1
          SER(SmBSplineSurface::CreateBilinearSurface(sContext,
                                                      sPt00, sPt10, sPt01, sPt11, 
                                                      pBSS1));

          sPt00 = SmPoint3d(10,  5, 0);
          sPt01 = SmPoint3d(20,  5, 0);
          sPt11 = SmPoint3d(20, 15, 0);
          sPt10 = SmPoint3d(10, 15, 0);

          // build bilinear surface 2
          SER(SmBSplineSurface::CreateBilinearSurface(sContext,
                                                      sPt00, sPt10, sPt01, sPt11, 
                                                      pBSS2));

          // Build transform
          SmAxis2Placement sA2P;
          my_create_refframe(sA2P);

          // transform 2nd Surface
          pBSS2->Transform(sA2P);

          //SmAxis2Placement sA2P2;
          //sA2P2.Translate(SmVector3d(1,0,7));
          //pBSS1->Transform(sA2P2);
          pBSS1->Transform(sA2P);

          // insert surface into pBrep as a face
          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks
          SmFace* pF;
          SER(pBrep->CreateFaceFromSurface(pBSS1,
                                           pBSS1->GetNaturalUVDomain(),
                                           pF));

          // insert 2nd surface into pBrep as a face
          SER(pBrep->CreateFaceFromSurface(pBSS2,
                                           pBSS2->GetNaturalUVDomain(),
                                           pF));
          
          // Build faces into pBrep on the bounding planes of the input SmBSplineSurfaces.
          // This builds 4 sides of a box that incloses surface pBSS1 and pBSS2 whe
          //   the boundary isoparameter curves of pBSS1 and pBSS2 are planar.
          my_create_faces_from_boundary_planes(pBSS1, 5.0, pBrep);
          my_create_faces_from_boundary_planes(pBSS2, 5.0, pBrep);

          // GlobalSurfaceIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE; 
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes; 
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;

          // intersect pair of planar surfaces
          SER(pBSS1->GlobalSurfaceIntersect(sContext, 
                                            pBSS1->GetNaturalUVDomain(),
                                            *pBSS2,
                                            pBSS2->GetNaturalUVDomain(),
                                            bUseSurfaceEdges,
                                            &dApproxTol,
                                            NULL,
                                            &s3DCurves,
                                            NULL, // &sUVCurves1,
                                            NULL, // &sUVCurves2,
                                            &sCurveTypes,
                                            &sDeviations));

          // verify solution count
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff);
          SM_ASSERT(nSolutions == 1);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pBSS1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pBSS2->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif
          for(ULONG ii=0; ii<nSolutions; ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);

              // otherwise nothing can be seen
              SmAxis2Placement sTransfRF;
              sTransfRF.Translate( SmVector3d(.2, .2, .2) );
              pBSC->Transform(sTransfRF);

              SM_ASSERT(!pBSC->IsDegenerate());

              NER(pBSC);               
              my_create_edge(pBrep, pBSC);
            }

          
          // inform the public - test results
          pBrep->Dump();
          pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif
      }
      break;

  case 15: // intersect pair of extruded surfaces represented as SmBSplineSurface objects
      {
          SM_SPRINTF(sBuff,_T("%s"),_T("\nEntered my_ssi_analytic_demo: Empty Test 16 Extrudeds X \n"));
          MYPRINTF(sBuff);

          SmBSplineSurface* pBSS1, *pBSS2;

          // read and copy an extruded SmBSplineSurface from file ../../TestFiles/pt_TestFiles/surf_extrude.smb
          SER(my_create_test_extruded_surf_BSS(sContext, pBSS1));

          // read and copy an extruded SmBSplineSurface from file ../../TestFiles/pt_TestFiles/surf_extrude.smb
          SER(my_create_test_extruded_surf_BSS(sContext, pBSS2));


          // Build transform
          SmAxis2Placement sA2P;
          my_create_refframe(sA2P);

          // transform 2nd Surface
          pBSS2->Transform(sA2P);

          SmAxis2Placement sA2P2;
          sA2P2.Translate(SmVector3d(1,0,7));
          pBSS1->Transform(sA2P2);
          pBSS1->Transform(sA2P);

          // insert surfaces into pBrep as faces
          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks
          SmFace* pF;
          SER(pBrep->CreateFaceFromSurface(pBSS1,
                                           pBSS1->GetNaturalUVDomain(),
                                           pF));

          SER(pBrep->CreateFaceFromSurface(pBSS2,
                                           pBSS2->GetNaturalUVDomain(),
                                           pF));
          

          // Build faces into pBrep on the bounding planes of the input SmBSplineSurfaces.
          // This builds 4 sides of a box that incloses surface pBSS1 and pBSS2 whe
          //   the boundary isoparameter curves of pBSS1 and pBSS2 are planar.
          my_create_faces_from_boundary_planes(pBSS1, 5.0, pBrep);
          my_create_faces_from_boundary_planes(pBSS2, 5.0, pBrep);

          // GlobalSurfaceIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE;
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes;
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;

          // intersect pair of extruded surfaces represented as SmBSplineSurface objects
          SER(pBSS1->GlobalSurfaceIntersect(sContext, 
                                            pBSS1->GetNaturalUVDomain(),
                                            *pBSS2,
                                            pBSS2->GetNaturalUVDomain(),
                                            bUseSurfaceEdges,
                                            &dApproxTol,
                                            NULL,
                                            &s3DCurves,
                                            NULL, // &sUVCurves1,
                                            NULL, // &sUVCurves2,
                                            &sCurveTypes,
                                            &sDeviations));

          // verify solution count
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff);
          SM_ASSERT(nSolutions == 3);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pBSS1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pBSS2->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif
          // add every solution curve to pBrep as an edge
          for(ULONG ii=0; ii<nSolutions; ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);
              NER(pBSC);             

              SM_ASSERT(pBSC->IsDegenerate() );

              my_create_edge(pBrep, pBSC);
            }
          
          // inform the public - test results
          pBrep->Dump();
          pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif

      }
      break;


  case 16: // intersect
      {
          SM_SPRINTF(sBuff,_T("%s"),_T("\nEntered my_ssi_analytic_demo: Empty Test 16 Spheres X \n"));
          MYPRINTF(sBuff);

          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks


          // get half sphere surface copy from brep in file ../../TestFiles/pt_TestFiles/surf_sphere150.smb
          SmBSplineSurface* pBSS1, *pBSS2;
          SER(my_create_test_half_sphere_BSS(sContext, pBSS1));

          // get half sphere surface copy from brep in file ../../TestFiles/pt_TestFiles/surf_sphere150.smb
          SER(my_create_test_half_sphere_BSS(sContext, pBSS2));

          // Build transform
          SmAxis2Placement sA2P;
          my_create_refframe(sA2P);

          // transform 2nd Surface
          pBSS2->Transform(sA2P);


          // build 2nd transform
          SmAxis2Placement sA2P2;
          sA2P2.Translate(SmVector3d(0,1,0));
          sA2P2.RotateAboutAxisAtPoint(SM_PI, SmPoint3d(-2, 0, 0), SmVector3d(0, 1, 0));

          // transform 1st surface
          pBSS1->Transform(sA2P2);
          pBSS1->Transform(sA2P);

          // insert surfaces into pBrep as faces
          SmFace* pF;
          SER(pBrep->CreateFaceFromSurface(pBSS1,
                                           pBSS1->GetNaturalUVDomain(),
                                           pF));

          SER(pBrep->CreateFaceFromSurface(pBSS2,
                                           pBSS2->GetNaturalUVDomain(),
                                           pF));
          

          // Build faces into pBrep on the bounding planes of the input SmBSplineSurfaces.
          // This builds 4 sides of a box that incloses surface pBSS1 and pBSS2 whe
          //   the boundary isoparameter curves of pBSS1 and pBSS2 are planar.
          my_create_faces_from_boundary_planes(pBSS1, 5.0, pBrep);
          my_create_faces_from_boundary_planes(pBSS2, 5.0, pBrep);

          // GlobalSurfaceIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE;
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes;
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;

          // intersect 
          SER(pBSS1->GlobalSurfaceIntersect(sContext, 
                                            pBSS1->GetNaturalUVDomain(),
                                            *pBSS2,
                                            pBSS2->GetNaturalUVDomain(),
                                            bUseSurfaceEdges,
                                            &dApproxTol,
                                            NULL,
                                            &s3DCurves,
                                            NULL, // &sUVCurves1,
                                            NULL, // &sUVCurves2,
                                            &sCurveTypes,
                                            &sDeviations));

          // verify solution count
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff);
          SM_ASSERT(nSolutions == 1);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pBSS1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pBSS2->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif
          // insert solution curves into pBrep as edges
          for(ULONG ii=0; ii<nSolutions; ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);
              NER(pBSC);               

              SM_ASSERT(pBSC->IsDegenerate() );

              my_create_edge(pBrep, pBSC);
            }
          
          // inform the public - test results
          pBrep->Dump();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif

      }
      break;

  case 17: // intersect pair of planar patches
      {

          SM_SPRINTF(sBuff,_T("%s"),_T("\nEntered my_ssi_analytic_demo: Test 14 2 planar patches X \n"));
          MYPRINTF(sBuff);

          SmBSplineSurface* pBSS1, *pBSS2;

          SmPoint3d sPt00( 0,  0, 0);
          SmPoint3d sPt01(10,  0, 0);
          SmPoint3d sPt11(10, 10, 0);
          SmPoint3d sPt10( 0, 10, 0);

          // create planar patch between given corner points
          SER(SmBSplineSurface::CreateBilinearSurface(sContext,
                                                      sPt00, sPt10, sPt01, sPt11, 
                                                      pBSS1));

          sPt00 = SmPoint3d(11,  5, 0);
          sPt01 = SmPoint3d(21,  5, 0);
          sPt11 = SmPoint3d(21, 15, 0);
          sPt10 = SmPoint3d(11, 15, 0);

          // create planar patch between given corner points
          SER(SmBSplineSurface::CreateBilinearSurface(sContext,
                                                      sPt00, sPt10, sPt01, sPt11, 
                                                      pBSS2));

          // Build transform
          SmAxis2Placement sA2P;
          my_create_refframe(sA2P);

          // transform 2nd surface
          pBSS2->Transform(sA2P);

          //SmAxis2Placement sA2P2;
          //sA2P2.Translate(SmVector3d(1,0,7));
          //pBSS1->Transform(sA2P2);

          // transform 1st surface
          pBSS1->Transform(sA2P);

          // insert surfaces into pBrep as faces
          SmBrep *pBrep = new (sContext) SmBrep();
          SmObjDelete sCleanup(pBrep); // JLMCC hunting memory leaks
          SmFace* pF;
          SER(pBrep->CreateFaceFromSurface(pBSS1,
                                           pBSS1->GetNaturalUVDomain(),
                                           pF));

          SER(pBrep->CreateFaceFromSurface(pBSS2,
                                           pBSS2->GetNaturalUVDomain(),
                                           pF));
          

          // Build faces into pBrep on the bounding planes of the input SmBSplineSurfaces.
          // This builds 4 sides of a box that incloses surface pBSS1 and pBSS2 whe
          //   the boundary isoparameter curves of pBSS1 and pBSS2 are planar.
          my_create_faces_from_boundary_planes(pBSS1, 5.0, pBrep);
          my_create_faces_from_boundary_planes(pBSS2, 5.0, pBrep);

          // GlobalSurfaceIntersect arguments
          SmBoolean bUseSurfaceEdges[2];
          bUseSurfaceEdges[0] = TRUE;
          bUseSurfaceEdges[1] = TRUE;
          SmTArray<double> sDeviations;
          SmTArray<SmTsectCurveType> sCurveTypes;
          SmTArray<SmCurve*> s3DCurves;
          SmApproxTol3d dApproxTol = 0.0001;

          // intersect pair of planar patches 
          SER(pBSS1->GlobalSurfaceIntersect(sContext, 
                                            pBSS1->GetNaturalUVDomain(),
                                            *pBSS2,
                                            pBSS2->GetNaturalUVDomain(),
                                            bUseSurfaceEdges,
                                            &dApproxTol,
                                            NULL,
                                            &s3DCurves,
                                            NULL, // &sUVCurves1,
                                            NULL, // &sUVCurves2,
                                            &sCurveTypes,
                                            &sDeviations));

          // verify solution count
          ULONG nSolutions = s3DCurves.GetSize();
          SM_SPRINTF(sBuff,_T("\n Number of Solutions: %ld \n"), nSolutions);
          MYPRINTF(sBuff);
          SM_ASSERT(nSolutions == 0);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
          {
            ULONG ii ;
            smgfx_Erase() ;
            smgfx_SetLook(1,2, 0,0,1); pBSS1->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(1,2, 0,1,0); pBSS2->DrawUV(8,8,FALSE,NULL,TRUE); sm_GraphicsLoop();
            smgfx_SetLook(3,4, 1,0,0); for(ii=0;ii<s3DCurves.GetSize();ii++)
                                         { s3DCurves[ii]->Draw() ; sm_GraphicsLoop() ; }
            sm_GraphicsLoop();
          }
#endif
          for(ULONG ii=0; ii<nSolutions; ii++) 
            {
              SmCurve* pC = s3DCurves[ii];
              SmBSplineCurve* pBSC = SM_CAST_PTR(SmBSplineCurve, pC);

              NER(pBSC);               
              my_create_edge(pBrep, pBSC);
            }
          
          // inform the public - test results
          pBrep->Dump();
          pBrep->ValidatePointers();
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
          {
            pBrep->Draw(); sm_GraphicsLoop();
          }
#endif
      }
      break;

  default:
          break;

    } // end switch on nTest
    } // End for loop

  return SM_SUCCESS;

} // end my_ssi_analytic_demo

/*********************************************************
PURPOSE ---

USER NOTES ---
*********************************************************/
SmStatus my_test_ssi_analytic()
{
    SER(my_ssi_analytic_demo());
    return SM_SUCCESS;

} // end my_test_ssi_analytic

/***********************************************************************
PURPOSE --- Run all unit known unit tests

USAGE NOTES ---
***********************************************************************/
PT_EXPORT SmStatus my_unit_test_suite() 
{

#ifdef SMJAVA
static int cbiJavaOnly=TRUE;
if ( cbiJavaOnly )
  // Java interface tests
  {
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Java Interface\n"));
    SmStatus eStat = my_test_java_interface();

    return eStat;
  }
#endif // SMJAVA

  // SmCurve_test file tests
  {
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Analytic Curves\n"));
    my_test_analytic_curves() ;
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - SmTArray\n"));
    my_test_array();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - SmMap\n"));
    my_test_map();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - SmObject \n"));
    my_test_object();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - SmVector2d \n"));
    my_test_vector2d();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - SmVector3d \n"));
    my_test_vector3d();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - SmMatrix\n"));
    my_test_matrix();

    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - SmAxis2Placement \n"));
    my_test_axis2placement();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - SmExtent1d\n"));
    my_test_extent1d();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - SmExtent2d\n"));
    my_test_extent2d();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - SmExtent3d\n"));
    my_test_extent3d();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - SmPseudoBox\n"));
    my_test_pseudobox();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - SmSolutionArray\n"));
    SER(my_test_solution());

    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Basic creation\n"));
    my_test_creation();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - B-Spline Curve Methods\n"));
    my_test_bsplinecurve();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - B-Spline Curve NLib Methods\n"));
    my_test_bsplinecurve_nlib();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - B-Spline Surface NLib Methods\n"));
    my_test_bsplinesurface_nlib();

    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Intersect Analytics\n"));
    my_test_cci_analy() ;
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Intersect Nurb\n"));
    my_test_cci_nurb();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Intersect Tolerance\n"));
    my_test_cci_tol();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Intersect Tangency\n"));
    my_test_cci_tangent();

    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Intersect Coincidence \n"));
    my_test_cci_coincidence();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve Self Intersection\n"));
    my_test_cci_self();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Projected Intersection\n"));
    // GWC: this is one test of a circle/line projected xSect where the projDir == LineTangent.  I don't think the sol is correct.
    my_test_cci_projection();

    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Solve Nurb\n"));
    my_test_cc_nurb();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Solve Analytic\n"));
    my_test_cc_analy();

    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Methods\n"));
    my_test_methods() ;

    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Point Nurb\n"));
    my_test_cp_nurb();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Point Analytic\n"));
    my_test_cp_analy();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Test Time\n"));
    my_test_time();      
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Test Polynomials\n"));
    my_test_polynomial();

    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve Properties\n"));
    my_test_curve_properties();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Angle Minimize\n"));
    my_test_crv_angle_min();    
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Directed Maximize\n"));
    my_test_crv_directed_max();    
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Directed Minimize\n"));
    my_test_crv_directed_min();    
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Projected Maximize\n"));
    my_test_crv_projected_max(); 
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Projected Minimize\n"));
    my_test_crv_projected_min();    
       
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Projected Tangency\n"));
    my_test_crv_projected_tangency();    
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Signed Angle Minimize\n"));
    my_test_crv_signed_angle_min();        
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Signed Directed Minimize\n"));
    my_test_crv_signed_directed_min();      
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Curve Signed Pivot Minimize\n"));
    my_test_signed_pivot_min();

  } // end SmCurve_test file tests scope

  // smsurf_test file tests
  {
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Analytic Surfaces\n"));
    my_test_analytic_surfaces() ;
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - BSpline Surfaces\n"));
    my_test_bsplinesurface() ;
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Surface Intersect Analytic\n"));
    my_test_csi_analy();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Surface Intersect Nurb\n"));
    my_test_csi_nurb();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Surface Intersect Tolerance\n"));
    my_test_csi_tol();

    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Surface Intersect Tangency\n"));
    my_test_csi_tangency();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Surface Intersect Coincidence\n"));
    my_test_csi_coincidence();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Curve/Surface Solve Nurb\n"));
    my_test_cs_nurb();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Drop Curve onto Surface\n"));
    SER(my_test_drop_curve());
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Surface/Surface Solve Nurb\n"));
    my_test_ss_nurb();

    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Surface Methods\n"));
    my_test_surf_methods() ;
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Surface/Point Solve Nurb\n"));
    my_test_sp_nurb();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Surface/Point Solve Analytic\n"));
    my_test_sp_analy();
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Surface Suite\n"));
    my_test_surface_suite() ;
    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Surface Sectioning\n"));
    my_test_section();

    MYPRINTF(_T("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& - Test Projection\n"));
    my_test_projection() ;

  } // end smsurf_test file tests scope

  // all done
  return(SM_SUCCESS) ;

} // end my_unit_test_suite
