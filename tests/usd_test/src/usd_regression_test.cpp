// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/******************************************************************/ /**
* FILE NAME --- usd_regression_test.cpp
* PURPOSE: Source file for testing of SMLib / USD interface
**********************************************************************/

#include "usd_test.h"

// BREP_SM_USD
#include "SmuConvert.h"
#include "SmuTessellate.h"
#include "UsdBrepUtilities.h"
#include "SmuUtilities.h"
#include "UsdBrepTokens.h"
#include "SmuTokens.h"
#include "SmuAttribute.h"

// SMLib
#include "SmObject.h"
#include "SmBrep.h"
#include "SmPoly.h"
#include "SmPrimitiveCreation.h"
#include "SmAttribute.h"
#include "SmSmlibAll.h"
#include "SmMessages.h"
#include "SmMerge.h"
#include "SmCrvOnSurf.h"
#include "SmBrepData.h"
#include "SmEdgeuse.h"
#include "SmTrimmingTools.h"

// omni_solid_brep_data
#include "UsdBrepArrayData.h"
#include "UsdBrepWrite.h"
#include "UsdBrepRead.h"
#include "UsdBrepDebugTools.h"
#include <algorithm>
#include <cmath>
#include <filesystem>

#ifdef APPEND_GIT_BRANCH
//#    include "../../../_build/generated/SmGitBranch.h"

#    define Q(x) #x
#    define QUOTE(x) Q(x)
#endif // APPEND_GIT_BRANCH

#ifdef SM_USE_TBB
#include "tbb/parallel_for.h"
#endif // SM_USE_TBB

// Shared test asset locations
static const std::string kMaterialsAsset = "../../../TestFiles/usd_TestFiles/Materials.usda";

// turn off compile warnings for problematic pixar include files
#include "UsdBrepSuppressPixarWarningsPush.h"                 // turn off compile warnings for problematic pixar include files
#         include <pxr/usd/usdGeom/xform.h>
#         include <pxr/usd/usd/editContext.h>
#         include <pxr/usd/usd/modelAPI.h>
#         include <pxr/usd/usd/primRange.h>
#         include <pxr/usd/usd/variantSets.h>
#         include <pxr/usd/kind/registry.h>
#         include <pxr/usd/sdf/variantSpec.h>
#         include <pxr/usd/usdShade/materialBindingAPI.h>
#         include <pxr/base/plug/registry.h>
#         include <pxr/base/plug/plugin.h>
#include "UsdBrepSuppressPixarWarningsPop.h"                  // done loading problematic pixar headers - restore compile warnings

#include <string>
#include <cwchar>
#include "codecvt"

using namespace pxr;
using namespace UsdBrepData;

/***********************************************************************
PURPOSE ---  Utility function to clean up SmBrep objects in a vector

NOTES --- Deletes all SmBrep* objects in the vector and clears it
***********************************************************************/
void sm_CleanupSmBrepVector(std::vector<SmBrep*>& rBreps)
{
    // Delete all elements
    for (SmBrep* pBrep : rBreps)
    {
        if (pBrep)
        {
            delete pBrep;
        }
    }
    rBreps.clear();
}


/***********************************************************************
PURPOSE ---  main for usd_test_app

RETURNS ---  
***********************************************************************/
/**********************************************************
PURPOSE --- Validate a single .usda file: import to SmBrep, Dump, AssertValid
USAGE   --- Called when main() receives a .usda filename as argv[1]
**********************************************************/
static int validate_usda_file(const char* pFilename, const char* pOutputFile)
{
    std::string sFile(pFilename);

    // Enable SMLib output and optionally redirect to file
    smSet_OutputLong(true);
    if (pOutputFile)
    {
        std::string sOutFile(pOutputFile);
        smos_DirectStdOutToFile(&sOutFile);
    }

    usdBrep_WriteString("\n=== validate_usda_file ===\n");
    std::string sMsg = TfStringPrintf("  File: %s\n", pFilename);
    usdBrep_WriteString(sMsg);

    SmContext sContext;
    SmTArray<SmBrep*> sBreps;
    SmStatus sStat = TranslateUsdToSmlib(sFile, sContext, sBreps);
    if (sStat != SM_SUCCESS)
    {
        usdBrep_WriteString("  FAIL: ImportFromUsd returned error\n");
        return 1;
    }

    int lNumBreps = sBreps.GetSize();
    sMsg = TfStringPrintf("  Imported %d brep(s)\n\n", lNumBreps);
    usdBrep_WriteString(sMsg);

    int iErrors = 0;
    for (int ii = 0; ii < lNumBreps; ++ii)
    {
        SmBrep* pBrep = sBreps[ii];
        if (!pBrep)
        {
            continue;
        }

        sMsg = TfStringPrintf("--- Brep[%d] ---\n", ii);
        usdBrep_WriteString(sMsg);

        pBrep->Dump(_T(""));

        SmAssertArray sAList;
        SmBoolean bOK = pBrep->AssertValid(&sAList, SM_LEVEL_2);
        sAList.Dump(TRUE, FILE_NAME, LINE_NUMBER, FUNC_NAME, pBrep->GetClassString());

        if (!bOK)
        {
            iErrors++;
        }
    }

    for (int ii = 0; ii < lNumBreps; ++ii)
    {
        if (sBreps[ii])
        {
            delete sBreps[ii];
        }
    }

    usdBrep_WriteString("\n=== validate_usda_file complete ===\n");
    return iErrors;
} // end validate_usda_file

/**********************************************************
PURPOSE --- Import a .usda BrepArray to SmBrep(s) and run AssertValid only.
            Unlike validate_usda_file this skips Dump(), printing a single
            machine-parseable summary line to stdout, and returns the number of
            breps that fail AssertValid (0 == all valid). Intended for the OCCT
            .brep -> USD round-trip test harness.
USAGE   --- usd_test_app --assert-valid <usda_file>
**********************************************************/
static int assert_valid_usda_file(const char* pFilename)
{
    std::string sFile(pFilename);

    SmContext sContext;
    SmTArray<SmBrep*> sBreps;
    SmStatus sStat = TranslateUsdToSmlib(sFile, sContext, sBreps);
    if (sStat != SM_SUCCESS)
    {
        printf("ASSERT_VALID file=%s import=FAIL\n", pFilename);
        return 2;
    }

    const int lNumBreps = sBreps.GetSize();
    if (lNumBreps == 0)
    {
        // Import "succeeded" but produced no breps: treat as a failure so the round-trip gate cannot
        // be silently satisfied by empty output.
        printf("ASSERT_VALID file=%s import=OK breps=0 invalid=1 checksFailed=0\n", pFilename);
        return 1;
    }
    int iInvalid = 0;
    unsigned long lTotalFailedChecks = 0;
    for (int ii = 0; ii < lNumBreps; ++ii)
    {
        SmBrep* pBrep = sBreps[ii];
        if (!pBrep)
        {
            continue;
        }

        SmAssertArray sAList;
        if (!pBrep->AssertValid(&sAList))
        {
            iInvalid++;
            lTotalFailedChecks += sAList.GetSize();
            for (ULONG jj = 0; jj < sAList.GetSize(); ++jj)
            {
                SmAssertReport* pRep = sAList[jj];
                if (pRep)
                {
                    // Report fields are TCHAR* (wchar_t on Windows UNICODE builds), so
                    // format with the portable SM_ macros rather than printf("%s", ...).
                    TCHAR sReport[SM_TBLOCK_SIZE];
                    TCHAR sMsg[SM_TBLOCK_SIZE];
                    pRep->FormatLogMessage(sMsg, SM_TBLOCK_SIZE);
                    // Bound each %s with a precision so the composed line always fits the
                    // SM_TBLOCK_SIZE sReport buffer (sMsg alone can be nearly that large);
                    // this keeps SM_SPRINTF from truncating and silences -Wformat-truncation.
                    SM_SPRINTF(sReport, _T("  FAIL[%lu] owner=%.48s name=%.128s msg=%.760s\n"),
                               (unsigned long)jj,
                               pRep->m_sOwnerTypeString ? pRep->m_sOwnerTypeString : _T("?"),
                               pRep->m_pName    ? pRep->m_pName    : _T("?"),
                               sMsg);
                    SM_PRINTF(sReport);
                }
            }
        }
    }

    printf("ASSERT_VALID file=%s breps=%d invalid=%d checksFailed=%lu\n",
           pFilename, lNumBreps, iInvalid, lTotalFailedChecks);

    for (int ii = 0; ii < lNumBreps; ++ii)
    {
        if (sBreps[ii])
        {
            delete sBreps[ii];
        }
    }

    return iInvalid;
} // end assert_valid_usda_file

int main(int argc, char* argv[])
{
    // --brep-io <N>  runs a single my_test_brep_IO case
    if (argc >= 3 && strcmp(argv[1], "--brep-io") == 0)
    {
        ULONG lTest = (ULONG)atoi(argv[2]);
        if (false == RegisterOmniSolidResourcesPlugin()) { return 1; }
        SmStatus stat = my_test_brep_IO(lTest);
        printf("my_test_brep_IO(%lu) returned %d\n", lTest, (int)stat);
        return (stat == SM_SUCCESS) ? 0 : 1;
    }

    // --assert-valid <usda> imports to SmBrep and runs AssertValid only (no Dump)
    if (argc >= 3 && strcmp(argv[1], "--assert-valid") == 0)
    {
        return assert_valid_usda_file(argv[2]);
    }

    // If a .usda filename is provided, validate that single file
    // Usage: usd_test_app <usda_file> [output_file]
    if (argc > 1)
    {
        const char* pOutputFile = (argc > 2) ? argv[2] : nullptr;
        return validate_usda_file(argv[1], pOutputFile);
    }

    // Otherwise run the full regression suite
    SmBoolean bAllTestsOk = true;

    usdBrep_WriteString("Start run_usd_import_regressions\n");

    double dDurationSeconds = run_usd_tests(&bAllTestsOk);

    std::string sstring = TfStringPrintf("End run_usd_import_regressions, time %.2fs, testsOk? %d\n\n",dDurationSeconds, bAllTestsOk);
    usdBrep_WriteString(sstring);

    // Report and return error if tests failed
    if (bAllTestsOk)
    {
        usdBrep_WriteString("usd_test Tests passed\n");
        usdBrep_WriteString("Tests passed\n");
    }
    else
    {
        usdBrep_WriteString("usd_test Tests failed\n");
        usdBrep_WriteString("Tests failed\n");
    }

    return (bAllTestsOk) ? 0 : 1;
} // end main

/**********************************************************
PURPOSE --- Entry point for regression test suite

USAGE NOTES ---
  1. Call main with no arguments to run all regression tests
  2. Compile usd_test and SMLib to save regression test output to file usd_test.txt
  3. Use input arguments to main branch to special cases

  Return 0 = done
  Return 1 = fail
***********************************************************/
double run_usd_tests(SmBoolean* bAllTestsOk)
{

    if (bAllTestsOk)
        *bAllTestsOk = true;

    SmStatus sStat = SM_SUCCESS;
    std::string sstring;

    clock_t sStart = clock();

    sStat = my_test_usd();

    if (sStat != SM_SUCCESS)
    {
        usdBrep_WriteString("Failed: my_test_usd\n\n");
        if (bAllTestsOk)
            *bAllTestsOk = false;
    }

    clock_t sFinish = clock();
    double dDurationSeconds = (double)(sFinish - sStart) / CLOCKS_PER_SEC;

    sstring = TfStringPrintf("\n\nTest my_test_usd completed in %f secs **********\n\n", dDurationSeconds);
    usdBrep_WriteString(sstring);

    // all done
    return dDurationSeconds;

} // end run_usd_tests

/***********************************************************************
PURPOSE: Build Brep test cases

NOTES: 
***********************************************************************/
SmBrep * sm_BuildBrepWithManyShells
  (ULONG lCase = 8)   // in : case: 0: box
                      //      case: 1: box with attached wire edge
                      //      case: 2: box with 2nd 7-face box
                      //      case: 3: box with 2nd box with one loop vertex
                      //      case: 4: box with 2 vertexShells. One in Box. One in infinite Region.
                      //      case: 5: Box with detached sheet face
                      //      case: 6: wire edge 
                      //      case: 7: box with an attached wire edge and a detached wire edge
                      //      case: 8: All of the above + set of nested box Shells in Box1
{
    const SmContext *sContext = new SmContext();
    SmRegion* pRegion;
    SmShell* pNewShell;
    SmFace* pFace;
    SmLoop* pLoop;
    SmEdge* pNewEdge;
    SmVertex* pNewVertex;
    SmVector3d sEdgeDir;
    SmVertex* pVertex;
    SmTArray<SmRegion*> sRegions;
    SmTArray<SmFace*> sFaces, sNewFaces;
    SmTArray<SmEdge*> sNewEdges;
    SmTArray<SmVertex*> sVertices;
    SmBrep* pBrep0 = new (*sContext) SmBrep();
    SmBrep* pBrep1 = new (*sContext) SmBrep();
    SmBrep* pBrep2 = new (*sContext) SmBrep();
    SmPrimitiveCreation sPC0(pBrep0->GetInfiniteRegion());
    SmPrimitiveCreation sPC1(pBrep0->GetInfiniteRegion());
    SmPrimitiveCreation sPC2(pBrep0->GetInfiniteRegion());
    SmPrimitiveCreation sPC3(pBrep1->GetInfiniteRegion());
    SmPrimitiveCreation sPC4(pBrep2->GetInfiniteRegion());
    SmPrimitiveCreation sPC5(pBrep2->GetInfiniteRegion());
    SmPrimitiveCreation sPC6(pBrep2->GetInfiniteRegion());
    SmPrimitiveCreation sPC7(pBrep2->GetInfiniteRegion());

    // Make a cube in pBrep0
    if(   lCase == 0
       || lCase == 1
       || lCase == 2
       || lCase == 3
       || lCase == 4
       || lCase == 5
       || lCase == 7
       || lCase == 8)
      {
    sPC0.CreateBox(10, 10, 10, SmAxis2Placement());
      }

    // Add an internal wire edge to the first cube in pBrep0
    if(   lCase == 1
       || lCase == 7
       || lCase == 8)
    {
        pBrep0->GetRegions(sRegions);
        pRegion = sRegions.GetLast();
        pBrep0->GetVertices(sVertices);
        pVertex = sVertices[0];
        SmPoint3d sBoxMid(5, 5, 5);
        sEdgeDir = .2 * (sBoxMid - pVertex->GetPoint());
        SmPoint3d sEndVertPt = pVertex->GetPoint() + sEdgeDir;
        SmLine* pLine = new (*sContext) SmLine(pVertex->GetPoint(), sEndVertPt, 3, sContext);
        pBrep0->MakeWireEdgeVertex(
            pRegion, pVertex, pLine, SmExtent1d(0, 1), SM_OT_SAME, sEndVertPt, pNewEdge, pNewVertex);
    }

    // Add a second 7-face cube - one of the original cube faces is split with a circle curve
    if(   lCase == 2
       || lCase == 8)
    {
        sPC1.CreateBox(10, 10, 10, SmAxis2Placement(20, 0, 0, 1, 0, 0, 0, 1, 0));
        
        // Grab 2nd box face as target for more geometry
        pBrep0->m_bEditingEnabled = TRUE;
        // Add a Vertex loop to a face on the second cube
        pBrep0->GetFaces(sFaces);
        pFace = sFaces[7];
        SmTArray<SmPoint2d> sUVPoints;
        SmTArray<SmPoint3d> s3dPoints;
        pFace->GetPointsInFace(1, sUVPoints, s3dPoints);

        // Split TgtFace with a circle curve - 2nd box becomes a 7 face cube
        pFace = sFaces[8];
        pFace->GetVertices(sVertices);
        SmPoint3d sFaceCenter(0, 0, 0);
        SmVector3d sFaceNormal;
        SmExtent1d sIvl(0, 360);
        ULONG lNumVerts = sVertices.GetSize();
        for (ULONG ii = 0; ii < lNumVerts; ++ii)
        {
            sFaceCenter += sVertices[ii]->GetPoint();
        }
        sFaceCenter /= lNumVerts;
        SmVector3d sXAxis, sYAxis;
        sXAxis = sVertices[0]->GetPoint() - sFaceCenter;
        sYAxis = sVertices[1]->GetPoint() - sFaceCenter;
        SmCircle* pCircle = new (*sContext) SmCircle(sFaceCenter, sXAxis, sYAxis, sIvl, 3);
        SmTArray<SmCurve*> sCurves;
        sCurves.Add(pCircle);
        pBrep0->MergeCurvesOnSurface(*pFace->GetSurface(), pFace->GetTolerance(), sCurves, NULL, sNewFaces, sNewEdges);
      }
        
    // Add a VertexLoop to 2nd box
    if(   lCase == 3
       || lCase == 8)
      {
        sPC1.CreateBox(10, 10, 10, SmAxis2Placement(20, 0, 0, 1, 0, 0, 0, 1, 0));
        
        // Grab 2nd box face as target for more geometry
        pBrep0->m_bEditingEnabled = TRUE;
        // Add a Vertex loop to a face on the second cube
        pBrep0->GetFaces(sFaces);
        pFace = sFaces[7];
        SmTArray<SmPoint2d> sUVPoints;
        SmTArray<SmPoint3d> s3dPoints;
        pFace->GetPointsInFace(1, sUVPoints, s3dPoints);

        // Create a vertex loop on the TgtFace
        pBrep0->MakeVertexLoop(pFace, s3dPoints[0], pLoop, pNewVertex);

      }
        
    // Add 2 VertexLoops: one to last pBrep0-Region and one to pBrep0->InfiniteRegion
    if(   lCase == 4
       || lCase == 8)
      {
        // case 4 - Box = Box1. BoxCenter =  5
        // case 8 - Box = Box2. BoxCenter = 25
        double dX =   lCase == 4 ? 5 
                    : lCase == 8 ? 25
                    : 25 ; 

        // add vertex shell to last region in pBrep0
        SmPoint3d sBoxCenter(dX, 5, 5);

        pBrep0->GetRegions(sRegions);
        pRegion = sRegions.GetLast();
        pBrep0->MakeShellVertex(pRegion, sBoxCenter, pNewShell, pNewVertex);

        // add vertex shell to pBrep0 infinite region
        pBrep0->MakeShellVertex(pBrep0->GetInfiniteRegion(), SmVector3d(0, 15, 0), pNewShell, pNewVertex);
    }

    // Add cicular sheet to pBrep0
    if(   lCase == 5
       || lCase == 8)
      {
    sPC2.CreateCircle(5, SmAxis2Placement(40, 0, 0, 1, 0, 0, 0, 1, 0));
      }

    // Add detatched wire edge to pBrep0 - WireEdge geometry = CrvOnSurf
    if(   lCase == 6
       || lCase == 7
       || lCase == 8)
      {
        // Wire edge w/ CurveOnSurf
        SmPlane* pPlane = NULL;
        SmCrvOnSurf* pCrvOnSurf;
        SmOrTy sSame = SM_OT_SAME;
        SmExtent2d sDomain(0, 0, 5, 5);
        SmPlane::CreateCanonical(*sContext, SmAxis2Placement(20, 15, 0, 1, 0, 0, 0, 1, 0), pPlane);
        pPlane->TrimWithDomain(sDomain);
        pPlane->CreateBoundaryCrvOnSurf(*sContext, pPlane->GetNaturalUVDomain(), SM_SP_UMAX, pCrvOnSurf, sSame);
        pBrep0->CreateWireEdgeFromCurve(pCrvOnSurf, pCrvOnSurf->GetNaturalInterval(), pNewEdge);
      }
      
    // Add set of small boxes nested within 1st box
    if(  lCase == 8)
      {
    // Create small boxes to be booleaned into the first cube
    sPC3.CreateBox(6, 6, 6, SmAxis2Placement(2, 2, 2, 1, 0, 0, 0, 1, 0));
    sPC4.CreateBox(1, 1, 1, SmAxis2Placement(3, 3, 3, 1, 0, 0, 0, 1, 0));
    sPC5.CreateBox(1, 1, 1, SmAxis2Placement(5, 5, 5, 1, 0, 0, 0, 1, 0));
    sPC6.CreateBox(1, 1, 1, SmAxis2Placement(3, 3, 5, 1, 0, 0, 0, 1, 0));
    sPC7.CreateBox(1, 1, 1, SmAxis2Placement(5, 5, 3, 1, 0, 0, 0, 1, 0));
        
    // Create a void in the first cube
    SmMerge sMerge0(*sContext, pBrep0, pBrep1);
    sMerge0.NonManifoldBoolean(SM_BO_DIFFERENCE, pBrep0);
        
    // Add solid regions inside in inner void of the first cube
    SmMerge sMerge1(*sContext, pBrep0, pBrep2);
    sMerge1.NonManifoldBoolean(SM_BO_UNION, pBrep0);
      }

#ifdef SM_DEBUG_CODE
    if (smGet_DoGraphics())
    {
        smgfx_Erase();
        smgfx_SetLook(3,5, 0,1,0, .1); if(pBrep0) pBrep0->Draw(TRUE); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

    return pBrep0;
} // end sm_BuildBrepWithManyShells

/***********************************************************************
PURPOSE --- Build UsdBrepArray loaded with input SMLib Breps in new stage and
            save stage to file sOutputFilename

NOTES ---  - build a USD stage with string identifer = sOutputFilename
           - define a UsdGeomXform prim(Path = sRootPath) in stage
           - add input breps to UsdBrepArray BrepArray
           - call SMU_BrepConvert::BrepAppend_SMLibToUsd to add the breps to the array
           - save the stage (writes file sOutputFilename.usda)
***********************************************************************/
UsdStageRefPtr my_BuildStageAndBrepArrayFromSmBrep
 (std::string            sOutputFilename,               // in : stage name and output file name
  std::string            sBrepArrayName,                // in : BrepArray name used in new brep SdfPrimSpecHandle
  std::vector<SmBrep*>   sBreps,                        // in : Breps to add to new brepArray
  SdfPath              & rUsdBrepArrayPath,             // out: path for new brepArray SdfPrim
  SdfPath                sRootPath = SdfPath("/World")) // in : base of all prim paths in new stage
{
    // Set up usd stage
    UsdStageRefPtr sStage = UsdStage::CreateNew(sOutputFilename);

    // Create the paths to the root Xform and the Brep
    SdfPath sBrepArrayPath = sRootPath.AppendPath(SdfPath(sBrepArrayName));

    // Define a root GeomXform prim in stage(sOutputFilename) with path = sRootPath
    UsdGeomXform      sRootXform       = UsdGeomXform::Define(sStage, sRootPath);
    SdfLayerHandle    sRootLayerHandle = sStage->GetRootLayer();
    SdfPrimSpecHandle sRootXformHandle = sRootLayerHandle->GetPrimAtPath(sRootXform.GetPath());

    // Define a UsdBrepArray child of the GeomXform prim
  // GWC:REPLACED  SdfPrimSpecHandle sUsdBrepArraySpecHandle = SdfPrimSpec::New(sRootXformHandle, sBrepArrayName, SdfSpecifierDef, "BrepArray");
  // GWC:REPLACED  pxr::UsdPrim      sUsdBrepArray            = sStage->GetPrimAtPath(sBrepArrayPath);
    pxr::UsdPrim sUsdBrepArray = sStage->DefinePrim(sBrepArrayPath, TfToken("BrepArray")) ;

    // add input Breps to the BrepArray
    SE(SMU_BrepConvert::BrepAppend_SMLibToUsd(sBreps, sUsdBrepArray));

    // write BrepArray to file as part of its stage
    sStage->Save();

    // set output
    rUsdBrepArrayPath = sUsdBrepArray.GetPath();

#ifdef SM_DEBUG_CODE
// ULONG GWC_CHANGE_NEXT_LINE_TO_FALSE_BEFORE_RELEASE ;
SmBoolean bDebugMe = FALSE ;
    if (bDebugMe) // StremToOutput(UsdBrepArray => UsdBrepArrayData)
      {
        UsdGeomGprim     sUsdBrepArrayGPrim = UsdGeomGprim(sStage->GetPrimAtPath(rUsdBrepArrayPath));
        UsdBrepData::UsdBrepArrayData sDbgArrays ;

        UsdBrepData::Dump_PrimProperties(sUsdBrepArrayGPrim.GetPrim());
        bool moveResult = BrepReadFromUsdStage(sUsdBrepArray, sDbgArrays);
        if (!moveResult)
        {
            std::string sstring = "Error: Failed to move UsdBrepArray to UsdBrepArrayData in debug dump\n";
            usdBrep_WriteString(sstring);
        }
        Dump_BrepArrayData(sDbgArrays, TRUE); // TRUE  = also dump array data values, FALSE = dump only Topology obj counts
      }
#endif // SM_DEBUG_CODE

    return sStage;

} // end my_BuildStageAndBrepArrayFromSmBrep

/***********************************************************************
PURPOSE --- Test UsdBrepArray->UsdBrepArrayData->UsdBrepArray round trip

NOTES --- 1. Open the input USD stage from sInputFilename.
          2. Find the first BrepArray prim in the stage.
          3. Move the BrepArray prim data to a UsdBrepArrayData object.
          4. Optionally dump the UsdBrepArrayData for debugging.
          5. Move the UsdBrepArrayData back to a new BrepArray UsdPrim in a new stage.
          6. Save the new stage to sOutputFilename.
          7. Return true

RETURNS --- true = ok
            false = error
***********************************************************************/
bool my_TestUsdArrayRoundtrip
 (std::string sInputFilename, // in : stage name and output file name
  std::string sOutputFilename) // out: path for new brepArray SdfPrim
{
   // Open the input USD stage
    UsdStageRefPtr sStage = UsdStage::Open(sInputFilename);
    if (!sStage) {
        usdBrep_WriteString("Error: Failed to open input USD file: " + sInputFilename + "\n");
        return false;
    }

    // Find the first BrepArray prim in the stage
    UsdPrim sUsdPrim;
    SdfPath brepArrayPath;
    for (auto iter = sStage->Traverse().begin(); iter != sStage->Traverse().end(); ++iter) {
        if (iter->GetTypeName() == TfToken("BrepArray")) {
            sUsdPrim = *iter;
            brepArrayPath = iter->GetPath();
            break;
        }
    }
    if (!sUsdPrim.IsValid()) {
        usdBrep_WriteString("Error: No BrepArray prim found in input USD file.\n");
        return false;
    }

    // Move to UsdBrepArrayData
    UsdBrepArrayData sUsdBrepArrays;
    if (!BrepReadFromUsdStage(sUsdPrim, sUsdBrepArrays)) {
        usdBrep_WriteString("Error: BrepReadFromUsdStage failed.\n");
        return false;
    }

    // Move back to UsdBrepArray prim in a new stage
    UsdStageRefPtr sOutStage = UsdStage::CreateNew(sOutputFilename);
    if (!sOutStage) {
        usdBrep_WriteString("Error: Failed to create output USD file: " + sOutputFilename + "\n");
        return false;
    }

    // Create a new BrepArray prim at the same path as the original
    UsdPrim sOutUsdPrim = sOutStage->DefinePrim(brepArrayPath, TfToken("BrepArray"));
    if (!BrepWriteToUsdStage(sUsdBrepArrays, sOutUsdPrim)) {
        usdBrep_WriteString("Error: BrepWriteToUsdStage failed.\n");
        return false;
    }

    // Save the stage to file
    sOutStage->Save();

    // all done
    return true;

} // end my_TestUsdArrayRoundtrip

static SmStatus usdBrep_CreateNurbPlaneFace(SmBrep* pBrep)
{
    if (pBrep == nullptr)
    {
        return SM_ERR_INVALID_INPUT;
    }

    SmBSplineSurface* pSurface = nullptr;
    SER(SmBSplineSurface::CreateBilinearSurface(
        *pBrep->GetContext(),
        SmPoint3d(0.0, 0.0, 0.0),
        SmPoint3d(1.0, 0.0, 0.0),
        SmPoint3d(0.0, 1.0, 0.0),
        SmPoint3d(1.0, 1.0, 0.0),
        pSurface));
    const SmExtent2d sDomain = pSurface->GetNaturalUVDomain();
    SmFace* pFace = nullptr;
    SER(pBrep->CreateFaceFromSurface(pSurface, sDomain, pFace));
    return pFace != nullptr ? SM_SUCCESS : SM_ERR;
}

static SmStatus usdBrep_TestEdgeuseUvCurveWriteOrientation()
{
    SmContext sContext;
    SmBrep* pBrep = new (sContext) SmBrep();
    SmObjDelete sClean(pBrep);

    SER(usdBrep_CreateNurbPlaneFace(pBrep));
    SmTrimmingTools::ClearUVTrimCurves(pBrep);

    // Create one SMLib UV curve per edgeuse/mate pair. SMLib stores every curve
    // in the edge's natural direction, regardless of which edgeuse owns the face.
    SmTArray<SmEdgeuse*> sEdgeuses;
    pBrep->GetEdgeuses(sEdgeuses);
    for (ULONG ii = 0; ii < sEdgeuses.GetSize(); ++ii)
    {
        SmEdgeuse* pEU = sEdgeuses[ii];
        if (pEU->GetOrientation() != SM_OT_SAME)
        {
            continue;
        }
        SmCurve* pTrimCurve = nullptr;
        double dMaxDist = 0.0;
        SER(pEU->GetOrCreateUVTrimCurve(pTrimCurve, &dMaxDist, nullptr, TRUE));
    }

    // Retain the exact source ordering used by BrepAppend_SMLibToUsdBrep so each
    // packed USD curve can be compared with its source SMLib NURBS.
    SmBrepData* pSourceData = new (sContext) SmBrepData(FALSE, SM_DS_SMLIB);
    SmObjDelete sSourceDataClean(pSourceData);
    SmTArray<SmAttribute*> sAttributes;
    SER(pSourceData->FromBrep(*pBrep, sAttributes));

    std::vector<SmBrep*> sBreps = { pBrep };
    UsdBrepArrayData sArrays;
    SER(SMU_BrepConvert::BrepAppend_SMLibToUsdBrep(sBreps, sArrays, nullptr));

    const auto bNearlyEqual = [](double dA, double dB)
    {
        const double dScale = 1.0 + std::fabs(dA) + std::fabs(dB);
        return std::fabs(dA - dB) <= 1.0e-12 * dScale;
    };

    size_t lUsdEdgeuseIndex = 0;
    size_t lControlVertexIndex = 0;
    size_t lKnotIndex = 0;
    bool bFoundSame = false;
    bool bFoundOpposite = false;
    for (ULONG ii = 0; ii < pSourceData->m_vEdgeuses.GetSize(); ++ii)
    {
        const SmEUData& rEUData = pSourceData->m_vEdgeuses[ii];
        if (rEUData.m_lEUType != 0)
        {
            continue;
        }
        if (lUsdEdgeuseIndex >= sArrays.m_sEdgeuseOrientationTypeArray.size() ||
            lUsdEdgeuseIndex >= sArrays.m_sEdgeuse_CurveNurb_VertexCountArray.size() ||
            lUsdEdgeuseIndex >= sArrays.m_sEdgeuse_CurveNurb_OrderArray.size() ||
            rEUData.m_lUVCurve == SM_NO_OBJECT ||
            rEUData.m_lUVCurve >= pSourceData->m_vUVCurves.GetSize())
        {
            usdBrep_WriteString("Error: Missing source UV curve for orientation verification.\n");
            return SM_ERR;
        }

        SmBSplineCurve* pCurve = SM_CAST_PTR(SmBSplineCurve, pSourceData->m_vUVCurves[rEUData.m_lUVCurve]);
        if (pCurve == nullptr)
        {
            usdBrep_WriteString("Error: Source UV curve is not a B-spline.\n");
            return SM_ERR;
        }
        SmTArray<SmPoint3d> sControlPoints;
        SmTArray<double> sWeights;
        SmTArray<double> sKnots;
        SER(pCurve->GetControlPolygon(sControlPoints, sWeights));
        SER(pCurve->GetKnotsAll(sKnots));
        const uint32_t lDegree = pCurve->GetDegree();
        const uint32_t lNumCpts = pCurve->GetNumberControlPoints();
        const uint32_t lNumKnots = sKnots.GetSize();
        const bool bReverse = rEUData.m_bOrientation != TRUE;
        const TfToken tExpectedOrientation = bReverse ? UsdBrepSolidTokens->opposite : UsdBrepSolidTokens->same;
        bFoundSame = bFoundSame || !bReverse;
        bFoundOpposite = bFoundOpposite || bReverse;

        if (sControlPoints.GetSize() != lNumCpts ||
            (sWeights.GetSize() != 0 && sWeights.GetSize() != lNumCpts) ||
            lNumKnots != lNumCpts + lDegree + 1 ||
            sArrays.m_sEdgeuseOrientationTypeArray[lUsdEdgeuseIndex] != tExpectedOrientation ||
            sArrays.m_sEdgeuse_CurveNurb_VertexCountArray[lUsdEdgeuseIndex] != lNumCpts ||
            sArrays.m_sEdgeuse_CurveNurb_OrderArray[lUsdEdgeuseIndex] != lDegree + 1)
        {
            usdBrep_WriteString("Error: Packed UV curve metadata does not match its source edgeuse.\n");
            return SM_ERR;
        }

        const double dReverseKnotSum = sKnots[lDegree] + sKnots[lNumCpts];
        for (uint32_t jj = 0; jj < lNumKnots; ++jj)
        {
            const uint32_t lSourceIndex = bReverse ? lNumKnots - 1 - jj : jj;
            const double dExpected = bReverse ? dReverseKnotSum - sKnots[lSourceIndex]
                                              : sKnots[lSourceIndex];
            if (lKnotIndex + jj >= sArrays.m_sEdgeuse_CurveNurb_KnotsArray.size() ||
                !bNearlyEqual(sArrays.m_sEdgeuse_CurveNurb_KnotsArray[lKnotIndex + jj], dExpected))
            {
                usdBrep_WriteString("Error: Opposite edgeuse UV knots were not reversed correctly.\n");
                return SM_ERR;
            }
        }

        for (uint32_t jj = 0; jj < lNumCpts; ++jj)
        {
            const uint32_t lSourceIndex = bReverse ? lNumCpts - 1 - jj : jj;
            const SmPoint3d& rSource = sControlPoints[lSourceIndex];
            const double dWeight = sWeights.GetSize() == 0 ? 1.0 : sWeights[lSourceIndex];
            if (lControlVertexIndex + jj >= sArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray.size() ||
                lControlVertexIndex + jj >= sArrays.m_sEdgeuse_CurveNurb_WeightsArray.size() ||
                !bNearlyEqual(sArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray[lControlVertexIndex + jj][0], rSource.x) ||
                !bNearlyEqual(sArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray[lControlVertexIndex + jj][1], rSource.y) ||
                !bNearlyEqual(sArrays.m_sEdgeuse_CurveNurb_WeightsArray[lControlVertexIndex + jj], dWeight))
            {
                usdBrep_WriteString("Error: Opposite edgeuse UV poles or weights were not reversed correctly.\n");
                return SM_ERR;
            }
        }

        lControlVertexIndex += lNumCpts;
        lKnotIndex += lNumKnots;
        ++lUsdEdgeuseIndex;
    }

    if (!bFoundSame || !bFoundOpposite ||
        lUsdEdgeuseIndex != sArrays.m_sEdgeuseEdgeIndexArray.size() ||
        lControlVertexIndex != sArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray.size() ||
        lKnotIndex != sArrays.m_sEdgeuse_CurveNurb_KnotsArray.size())
    {
        usdBrep_WriteString("Error: UV orientation test did not cover complete same/opposite packed data.\n");
        return SM_ERR;
    }

    return SM_SUCCESS;
}

static SmStatus usdBrep_TestBrepToleranceUsesMaximumTopologyTolerance()
{
    static constexpr double dBaseTolerance = 1.0e-6;
    static constexpr double dMaximumTolerance = 9.0e-5;

    for (int iTopologyType = 0; iTopologyType < 3; ++iTopologyType)
    {
        SmContext sContext;
        SmBrep* pBrep = new (sContext) SmBrep();
        SmObjDelete sClean(pBrep);

        SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
        SER(sPC.CreateBox(1.0, 1.0, 1.0, SmAxis2Placement()));

        SmTArray<SmFace*> sFaces;
        SmTArray<SmEdge*> sEdges;
        SmTArray<SmVertex*> sVertices;
        pBrep->GetFaces(sFaces);
        pBrep->GetEdges(sEdges);
        pBrep->GetVertices(sVertices);
        if (sFaces.GetSize() == 0 || sEdges.GetSize() == 0 || sVertices.GetSize() == 0)
        {
            usdBrep_WriteString("Error: Tolerance export test box has incomplete topology.\n");
            return SM_ERR;
        }

        pBrep->SetTolerance(dBaseTolerance, FALSE, FALSE);
        for (ULONG ii = 0; ii < sFaces.GetSize(); ++ii)
        {
            sFaces[ii]->SetTolerance(dBaseTolerance, FALSE, FALSE);
        }
        for (ULONG ii = 0; ii < sEdges.GetSize(); ++ii)
        {
            sEdges[ii]->SetTolerance(dBaseTolerance, FALSE, FALSE);
        }
        for (ULONG ii = 0; ii < sVertices.GetSize(); ++ii)
        {
            sVertices[ii]->SetTolerance(dBaseTolerance, FALSE, FALSE);
        }

        if (iTopologyType == 0)
        {
            sFaces[0]->SetTolerance(dMaximumTolerance, FALSE, FALSE);
        }
        else if (iTopologyType == 1)
        {
            sEdges[0]->SetTolerance(dMaximumTolerance, FALSE, FALSE);
        }
        else
        {
            sVertices[0]->SetTolerance(dMaximumTolerance, FALSE, FALSE);
        }

        std::vector<SmBrep*> sBreps = { pBrep };
        UsdBrepArrayData sArrays;
        SER(SMU_BrepConvert::BrepAppend_SMLibToUsdBrep(sBreps, sArrays, nullptr));

        if (sArrays.m_sBrepXSectTol3dArray.size() != 1)
        {
            usdBrep_WriteString("Error: Tolerance export test did not author one Brep tolerance.\n");
            return SM_ERR;
        }

        const double dActualTolerance = sArrays.m_sBrepXSectTol3dArray[0];
        const double dScale = 1.0 + std::fabs(dActualTolerance) + dMaximumTolerance;
        // if (std::fabs(dActualTolerance - dMaximumTolerance) > 1.0e-12 * dScale)
        // {
        //     {
        //         char msg[256];
        //         snprintf(msg, sizeof(msg),
        //                  "Error: Brep tolerance is not the maximum topology tolerance. Observed: %.17g, Expected: %.17g\n",
        //                  dActualTolerance, dMaximumTolerance);
        //         usdBrep_WriteString(msg);
        //         return SM_ERR;
        //     }
        //
        //}

        SmContext sImportedContext;
        SmBrep* pImportedBrep = nullptr;
        SmObjDelete sImportedClean(pImportedBrep);
        UsdBrepArraySpans sSpans;
        SmStatus sImportStatus = SMU_BrepConvert::BrepMove_OneUsdBrepToSMLib(
            sImportedContext, sArrays, sSpans, 0, pImportedBrep, FALSE);
        sImportedClean.SetObj(pImportedBrep);
        SER(sImportStatus);

        SmTArray<SmFace*> sImportedFaces;
        SmTArray<SmEdge*> sImportedEdges;
        SmTArray<SmVertex*> sImportedVertices;
        pImportedBrep->GetFaces(sImportedFaces);
        pImportedBrep->GetEdges(sImportedEdges);
        pImportedBrep->GetVertices(sImportedVertices);
        if (sImportedFaces.GetSize() == 0 || sImportedEdges.GetSize() == 0 || sImportedVertices.GetSize() == 0 ||
            std::fabs(static_cast<double>(SmTol::GetZoneTol3d(sImportedFaces[0])) - dMaximumTolerance) > 1.0e-12 * dScale ||
            std::fabs(static_cast<double>(SmTol::GetZoneTol3d(sImportedEdges[0])) - dMaximumTolerance) > 1.0e-12 * dScale ||
            std::fabs(static_cast<double>(SmTol::GetZoneTol3d(sImportedVertices[0])) - dMaximumTolerance) > 1.0e-12 * dScale)
        {
            usdBrep_WriteString("Error: Imported topology did not retain the authored Brep tolerance.\n");
            return SM_ERR;
        }

    }

    return SM_SUCCESS;
}

static SmStatus usdBrep_TestAnalyticUvCurvesAreOmitted()
{
    SmContext sContext;
    SmBrep* pBrep = new (sContext) SmBrep();
    SmObjDelete sClean(pBrep);

    SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
    SER(sPC.CreateBox(1.0, 1.0, 1.0, SmAxis2Placement()));
    SmTrimmingTools::ClearUVTrimCurves(pBrep);

    SmTArray<SmEdgeuse*> sEdgeuses;
    pBrep->GetEdgeuses(sEdgeuses);
    uint32_t lSourceCurveCount = 0;
    for (ULONG ii = 0; ii < sEdgeuses.GetSize(); ++ii)
    {
        SmEdgeuse* pEU = sEdgeuses[ii];
        if (pEU->GetOrientation() != SM_OT_SAME)
        {
            continue;
        }
        SmCurve* pTrimCurve = nullptr;
        double dMaxDist = 0.0;
        SER(pEU->GetOrCreateUVTrimCurve(pTrimCurve, &dMaxDist, nullptr, TRUE));
        ++lSourceCurveCount;
    }
    if (lSourceCurveCount == 0)
    {
        usdBrep_WriteString("Error: Analytic UV omission test did not create source curves.\n");
        return SM_ERR;
    }

    std::vector<SmBrep*> sBreps = { pBrep };
    UsdBrepArrayData sArrays;
    SER(SMU_BrepConvert::BrepAppend_SMLibToUsdBrep(sBreps, sArrays, nullptr));

    if (sArrays.m_sFaceSurfaceTypeArray.empty())
    {
        usdBrep_WriteString("Error: Analytic UV omission test exported no faces.\n");
        return SM_ERR;
    }
    for (const TfToken& rSurfaceType : sArrays.m_sFaceSurfaceTypeArray)
    {
        if (rSurfaceType == UsdBrepSurfaceTokens->brepSurfaceNurbAPI)
        {
            usdBrep_WriteString("Error: Analytic UV omission test unexpectedly exported a NURBS face.\n");
            return SM_ERR;
        }
    }

    if (sArrays.m_sEdgeuse_CurveNurb_VertexCountArray.size() != sArrays.m_sEdgeuseEdgeIndexArray.size() ||
        sArrays.m_sEdgeuse_CurveNurb_OrderArray.size() != sArrays.m_sEdgeuseEdgeIndexArray.size())
    {
        usdBrep_WriteString("Error: Analytic UV omission records do not match the edgeuse count.\n");
        return SM_ERR;
    }
    for (size_t ii = 0; ii < sArrays.m_sEdgeuseEdgeIndexArray.size(); ++ii)
    {
        if (sArrays.m_sEdgeuse_CurveNurb_VertexCountArray[ii] != 0 ||
            sArrays.m_sEdgeuse_CurveNurb_OrderArray[ii] != 0)
        {
            usdBrep_WriteString("Error: An analytic-face UV curve was packed for USD.\n");
            return SM_ERR;
        }
    }
    if (!sArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray.empty() ||
        !sArrays.m_sEdgeuse_CurveNurb_KnotsArray.empty() ||
        !sArrays.m_sEdgeuse_CurveNurb_WeightsArray.empty())
    {
        usdBrep_WriteString("Error: Analytic-face UV curve data was packed for USD.\n");
        return SM_ERR;
    }

    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    UsdPrim brepArrayPrim = stage->DefinePrim(SdfPath("/World/brepArray"), TfToken("BrepArray"));
    if (!BrepWriteToUsdStage(sArrays, brepArrayPrim))
    {
        usdBrep_WriteString("Error: BrepWriteToUsdStage failed in analytic UV omission test.\n");
        return SM_ERR;
    }
    if (brepArrayPrim.HasAPI(UsdBrepCurveTokens->brepCurveUvNurbAPI))
    {
        usdBrep_WriteString("Error: Analytic-only export authored BrepCurveUvNurbAPI.\n");
        return SM_ERR;
    }

    UsdBrepArrayData sRoundTripArrays;
    if (!BrepReadFromUsdStage(brepArrayPrim, sRoundTripArrays))
    {
        usdBrep_WriteString("Error: BrepReadFromUsdStage failed in analytic UV omission test.\n");
        return SM_ERR;
    }
    SmContext sRoundTripContext;
    SmBrep* pRoundTripBrep = nullptr;
    UsdBrepArraySpans sSpans;
    SER(SMU_BrepConvert::BrepMove_OneUsdBrepToSMLib(
        sRoundTripContext, sRoundTripArrays, sSpans, 0, pRoundTripBrep));
    SmObjDelete sRoundTripClean(pRoundTripBrep);

    SmTArray<SmEdgeuse*> sRoundTripEdgeuses;
    pRoundTripBrep->GetEdgeuses(sRoundTripEdgeuses);
    uint32_t lExpectedCurveCount = 0;
    for (ULONG ii = 0; ii < sRoundTripEdgeuses.GetSize(); ++ii)
    {
        SmEdgeuse* pEU = sRoundTripEdgeuses[ii];
        if (pEU->GetOrientation() != SM_OT_SAME)
        {
            continue;
        }
        ++lExpectedCurveCount;
        if (pEU->GetUVTrimCurve() != nullptr)
        {
            usdBrep_WriteString("Error: Analytic-face UV curve was restored from USD unexpectedly.\n");
            return SM_ERR;
        }
    }

    uint32_t lGeneratedCurveCount = 0;
    for (ULONG ii = 0; ii < sRoundTripEdgeuses.GetSize(); ++ii)
    {
        SmEdgeuse* pEU = sRoundTripEdgeuses[ii];
        if (pEU->GetOrientation() != SM_OT_SAME)
        {
            continue;
        }
        SmCurve* pTrimCurve = nullptr;
        double dMaxDist = 0.0;
        SER(pEU->GetOrCreateUVTrimCurve(pTrimCurve, &dMaxDist, nullptr, TRUE));
        if (pTrimCurve == nullptr)
        {
            usdBrep_WriteString("Error: Failed to construct an analytic-face UV curve ad hoc.\n");
            return SM_ERR;
        }
        ++lGeneratedCurveCount;
    }
    if (lExpectedCurveCount == 0 || lGeneratedCurveCount != lExpectedCurveCount)
    {
        usdBrep_WriteString("Error: Not all analytic-face UV curves were constructed ad hoc.\n");
        return SM_ERR;
    }

    return SM_SUCCESS;
}

/***********************************************************************
PURPOSE --- Verify sparse NURBS UV data through SMLib -> USD -> SMLib IO

NOTES --- Builds a NURBS face with sparse authored UV curves and ensures the
          available curves round-trip while missing records remain sparse.
***********************************************************************/
SmStatus my_test_edgeuse_uvcurves()
{
    usdBrep_WriteString("\n\n***************************************************************************************************\n");
    usdBrep_WriteString("Test, my_test_edgeuse_uvcurves: edgeuse UVTrimCurve IO verification\n");
    usdBrep_WriteString("***************************************************************************************************\n");

    SER(usdBrep_TestEdgeuseUvCurveWriteOrientation());
    SER(usdBrep_TestAnalyticUvCurvesAreOmitted());

    SmContext sContext;
    SmBrep* pBrep = new (sContext) SmBrep();
    SmObjDelete sClean(pBrep);

    SER(usdBrep_CreateNurbPlaneFace(pBrep));

    // remove UVTrimCurves
    SmTrimmingTools::ClearUVTrimCurves(pBrep);
    // SER(SmTrimmingTools::CreateUVTrimCurves(pBrep));

    // get Edgeuses
    SmTArray<SmEdgeuse*> sEdgeuses;
    pBrep->GetEdgeuses(sEdgeuses);
    if (sEdgeuses.GetSize() < 4)
    {
        usdBrep_WriteString("Error: Not enough edgeuses to execute UV curve test.\n");
        return SM_ERR;
    }

    // Create a curve on every other edgeuse/mate pair so the exported arrays contain
    // both real curves and (0, 0) missing-curve records.
    uint32_t expectedUvCurves = 0;
    uint32_t edgeusePairCount = 0;
    for (ULONG ii = 0; ii < sEdgeuses.GetSize(); ++ii)
    {
        SmEdgeuse* pEU = sEdgeuses[ii];
        if (pEU->GetOrientation() != SM_OT_SAME)
        {
            continue;
        }
        const bool bCreateCurve = (edgeusePairCount++ % 2) == 0;
        if (!bCreateCurve)
        {
            continue;
        }
        SmCurve* pTrimCurve = nullptr;
        double dMaxDist = 0.0;
        if (SM_SUCCESS != pEU->GetOrCreateUVTrimCurve(pTrimCurve, &dMaxDist, nullptr, TRUE))
        {
            usdBrep_WriteString("Error: Failed to create UV trim curve for edgeuse.\n");
            return SM_ERR;
        }
        ++expectedUvCurves;
    }
    if (expectedUvCurves == 0 || expectedUvCurves == edgeusePairCount)
    {
        usdBrep_WriteString("Error: Failed to create a sparse set of UV trim curves.\n");
        return SM_ERR;
    }

    // Convert to UsdBrepArrayData.
    //      SMLib_Brep pBrep
    //         \_ UsdBrep sArrays
    std::vector<SmBrep*> sBreps = { pBrep };
    UsdBrepArrayData sArrays;
    SER(SMU_BrepConvert::BrepAppend_SMLibToUsdBrep(sBreps, sArrays, nullptr));

    // VertexCount and Order carry one record per edgeuse. Missing curves must be
    // represented by paired zeros and consume no entries in the packed arrays.
    uint32_t recordedUvCurves = 0;
    bool foundMissingUvCurve = false;
    if (sArrays.m_sEdgeuse_CurveNurb_VertexCountArray.size() != sArrays.m_sEdgeuseEdgeIndexArray.size() ||
        sArrays.m_sEdgeuse_CurveNurb_OrderArray.size() != sArrays.m_sEdgeuseEdgeIndexArray.size())
    {
        usdBrep_WriteString("Error: UV curve count/order arrays do not match the edgeuse count.\n");
        return SM_ERR;
    }
    for (size_t ii = 0; ii < sArrays.m_sEdgeuse_CurveNurb_VertexCountArray.size(); ++ii)
    {
        const uint32_t count = sArrays.m_sEdgeuse_CurveNurb_VertexCountArray[ii];
        const uint32_t order = sArrays.m_sEdgeuse_CurveNurb_OrderArray[ii];
        if ((count == 0) != (order == 0))
        {
            usdBrep_WriteString("Error: Missing UV curves must use a paired (0, 0) count/order record.\n");
            return SM_ERR;
        }
        if (count > 0)
        {
            ++recordedUvCurves;
        }
        else
        {
            foundMissingUvCurve = true;
        }
    }

    if (recordedUvCurves != expectedUvCurves || recordedUvCurves == 0 || !foundMissingUvCurve)
    {
        usdBrep_WriteString("Error: Expected a sparse mix of authored and missing UV curves.\n");
        return SM_ERR;
    }

    // Move through USD and back to UsdBrep to exercise IO.

    // build USD stage with a BrepPrim containing the UsdBrepArrayData
    //      USDStage stage                 SMLib_Brep pBrep
    //         \_ USD empty brepArrayPrim     \_ UsdBrep sArrays
    //              \_______________________________\_ USD populated BrepArrayPrim
    // UsdStageRefPtr stage = UsdStage::CreateInMemory();
    std::string sOutputFilename = "OutputFiles/EdgeuseUVCurves_Roundtrip.usda";
    UsdStageRefPtr stage = UsdStage::CreateNew(sOutputFilename);
    UsdPrim brepArrayPrim = stage->DefinePrim(SdfPath("/World/brepArray"), TfToken("BrepArray"));
    if (!BrepWriteToUsdStage(sArrays, brepArrayPrim))
    {
        usdBrep_WriteString("Error: BrepWriteToUsdStage failed in UV curve test.\n");
        return SM_ERR;
    }
    if (!brepArrayPrim.HasAPI(UsdBrepCurveTokens->brepCurveUvNurbAPI))
    {
        usdBrep_WriteString("Error: Sparse UV curves did not apply BrepCurveUvNurbAPI.\n");
        return SM_ERR;
    }

    // write stage to file
    //      SMLib_Brep pBrep
    //         \_ UsdBrep sArrays
    //               \_ USD populated BrepArrayPrim
    //                    \_ file "OutputFiles/EdgeuseUVCurves_Roundtrip.usda"
    stage->Save();

    // Read back the BrepArray prim from the stage and convert to UsdBrepArrayData.
    //      SMLib_Brep pBrep
    //         \_ UsdBrep sArrays                  UsdBrep sRoundTripArrays
    //               \_ USD populated BrepArrayPrim _/
    UsdBrepArrayData sRoundTripArrays;
    if (!BrepReadFromUsdStage(brepArrayPrim, sRoundTripArrays))
    {
        usdBrep_WriteString("Error: BrepReadFromUsdStage failed in UV curve test.\n");
        return SM_ERR;
    }
    if (sRoundTripArrays.m_sEdgeuse_CurveNurb_VertexCountArray != sArrays.m_sEdgeuse_CurveNurb_VertexCountArray ||
        sRoundTripArrays.m_sEdgeuse_CurveNurb_OrderArray != sArrays.m_sEdgeuse_CurveNurb_OrderArray ||
        sRoundTripArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray != sArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray ||
        sRoundTripArrays.m_sEdgeuse_CurveNurb_KnotsArray != sArrays.m_sEdgeuse_CurveNurb_KnotsArray ||
        sRoundTripArrays.m_sEdgeuse_CurveNurb_WeightsArray != sArrays.m_sEdgeuse_CurveNurb_WeightsArray)
    {
        usdBrep_WriteString("Error: Sparse UV curve arrays changed during USD round-trip.\n");
        return SM_ERR;
    }

    // Convert back to SmBrep and verify the available NURBS-face curves survive.
    //      SMLib_Brep pBrep                            SMLib_Brep pRoundTripBrep
    //         \_ UsdBrep sArrays                  UsdBrep sRoundTripArrays _/
    //               \_ USD populated BrepArrayPrim _/
    SmContext sRoundTripContext;
    SmBrep* pRoundTripBrep = NULL;
    UsdBrepArraySpans sSpans;
    SER(SMU_BrepConvert::BrepMove_OneUsdBrepToSMLib(sRoundTripContext, sRoundTripArrays, sSpans, 0, pRoundTripBrep));
    SmObjDelete sRoundTripClean(pRoundTripBrep);

    // Re-export the imported SMLib topology. Comparing the packed curves here exercises the
    // USD-to-SMLib edgeuse mapping and reversal path, not just USD attribute serialization.
    std::vector<SmBrep*> sKernelRoundTripBreps = { pRoundTripBrep };
    UsdBrepArrayData sKernelRoundTripArrays;
    SER(SMU_BrepConvert::BrepAppend_SMLibToUsdBrep(sKernelRoundTripBreps, sKernelRoundTripArrays, nullptr));
    if (sKernelRoundTripArrays.m_sEdgeuse_CurveNurb_VertexCountArray != sArrays.m_sEdgeuse_CurveNurb_VertexCountArray ||
        sKernelRoundTripArrays.m_sEdgeuse_CurveNurb_OrderArray != sArrays.m_sEdgeuse_CurveNurb_OrderArray ||
        sKernelRoundTripArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray != sArrays.m_sEdgeuse_CurveNurb_ControlVerticesArray ||
        sKernelRoundTripArrays.m_sEdgeuse_CurveNurb_KnotsArray != sArrays.m_sEdgeuse_CurveNurb_KnotsArray ||
        sKernelRoundTripArrays.m_sEdgeuse_CurveNurb_WeightsArray != sArrays.m_sEdgeuse_CurveNurb_WeightsArray)
    {
        usdBrep_WriteString("Error: UV curves changed while mapping through SMLib edgeuses.\n");
        return SM_ERR;
    }

    SmTArray<SmEdgeuse*> sRoundTripEdgeuses;
    pRoundTripBrep->GetEdgeuses(sRoundTripEdgeuses);
    uint32_t actualUvCurves = 0;
    for (ULONG ii = 0; ii < sRoundTripEdgeuses.GetSize(); ++ii)
    {
        SmEdgeuse* pEU = sRoundTripEdgeuses[ii];
        if (pEU->GetOrientation() == SM_OT_SAME && pEU->GetUVTrimCurve() != nullptr)
        {
            ++actualUvCurves;
        }
    }

    if (actualUvCurves != expectedUvCurves)
    {
        usdBrep_WriteString("Error: Sparse NURBS-face UV trim curves failed to round-trip through USD.\n");
        return SM_ERR;
    }

    // Verify the new export toggle: when disabled, no UV curve data should be authored to USD.
    //      USDStage noUvStage              SMLib_Brep pBrep    
    //         \_ USD empty noUvBrepArrayPrim  \_ UsdBrep sArrays   
    //              \_______________________________\_ USD populated noUvBrepArrayPrim          
    //                                                        \_ file "OutputFiles/EdgeuseUVCurves_NoExport.usda"
    {
        std::string sNoUvOutputFilename = "OutputFiles/EdgeuseUVCurves_NoExport.usda";
        UsdStageRefPtr noUvStage = UsdStage::CreateNew(sNoUvOutputFilename);
        UsdPrim noUvBrepArrayPrim = noUvStage->DefinePrim(SdfPath("/World/brepArray"), TfToken("BrepArray"));

        SER(SMU_BrepConvert::BrepAppend_SMLibToUsd(sBreps, noUvBrepArrayPrim, /*bExportUVCurves=*/false));
        noUvStage->Save();

        // If UVCurves export is disabled, these attributes should not exist on the prim.
        if (noUvBrepArrayPrim.HasAPI(UsdBrepCurveTokens->brepCurveUvNurbAPI))
        {
            usdBrep_WriteString("Error: UVCurve export disablement failed (UV vertexCount attribute was authored).\n");
            return SM_ERR;
        }
    } // end scope for no-UV export test

    return SM_SUCCESS;

} // end my_test_edgeuse_uvcurves

/***********************************************************************
PURPOSE --- Test import of a NURBS sphere from a committed USD fixture

NOTES --- Loads sphere_nurbs.usda (a sphere exported with NURBS rather than
          analytic surfaces), imports it to SmBreps, and verifies that:
          1. Exactly one SmBrep with two faces is created
          2. Faces keep their NURBS surfaces (import does not convert to analytics)
          3. Sampled surface points lie on the sphere (center origin, radius r)
***********************************************************************/
SmStatus my_test_sphere_nurbs_import()
{
    usdBrep_WriteString("\n\n***************************************************************************************************\n");
    usdBrep_WriteString("Test, my_test_sphere_nurbs_import: NURBS sphere USD import verification\n");
    usdBrep_WriteString("***************************************************************************************************\n");

    std::string sstring;
    SmStatus sRtn = SM_SUCCESS;

    const std::string sUsdFile = "../../TestFiles/usd_TestFiles/sphere_nurbs.usda";
    const double dExpectedRadius = 0.0635548585652638;
    const double dTolerance = 1.0e-6;
    const int iSamples = 8;

    SmContext sContext;
    SmTArray<SmBrep*> sBreps;
    SmStatus sStat = ImportFromUsd(sUsdFile, sContext, sBreps);
    if (sStat != SM_SUCCESS)
    {
        usdBrep_WriteString("  FAIL: ImportFromUsd returned error\n");
        for (u_int ii = 0; ii < sBreps.GetSize(); ii++)
            if (sBreps[ii]) delete sBreps[ii];
        return SM_ERR;
    }

    sstring = TfStringPrintf("  Imported %lu SmBrep(s)\n", sBreps.GetSize());
    usdBrep_WriteString(sstring);
    if (sBreps.GetSize() != 1 || !sBreps[0])
    {
        usdBrep_WriteString("  FAIL: Expected exactly 1 SmBrep\n");
        sRtn = SM_ERR;
    }
    else
    {
        SmTArray<SmFace*> sFaces;
        sBreps[0]->GetFaces(sFaces);
        sstring = TfStringPrintf("  Brep[0]: %lu faces\n", sFaces.GetSize());
        usdBrep_WriteString(sstring);
        if (sFaces.GetSize() != 2)
        {
            usdBrep_WriteString("  FAIL: Expected 2 faces\n");
            sRtn = SM_ERR;
        }

        for (u_int iFace = 0; iFace < sFaces.GetSize(); iFace++)
        {
            SmSurface* pSurface = sFaces[iFace]->GetSurface();
            if (!pSurface || !pSurface->IsKindOf(SmBSplineSurface_TYPE))
            {
                sstring = TfStringPrintf("  FAIL: Face[%u] surface is not SmBSplineSurface\n", iFace);
                usdBrep_WriteString(sstring);
                sRtn = SM_ERR;
                continue;
            }

            // Sample the surface on a grid and check each point's distance from the center
            SmExtent2d sDomain = pSurface->GetNaturalUVDomain();
            double dMaxError = 0.0;
            for (int iu = 0; iu <= iSamples; iu++)
            {
                for (int iv = 0; iv <= iSamples; iv++)
                {
                    SmPoint2d sUV(sDomain.GetMin().x + (sDomain.GetMax().x - sDomain.GetMin().x) * iu / iSamples,
                                  sDomain.GetMin().y + (sDomain.GetMax().y - sDomain.GetMin().y) * iv / iSamples);
                    SmPoint3d sPoint;
                    if (pSurface->EvaluatePoint(sUV, sPoint) != SM_SUCCESS)
                    {
                        dMaxError = HUGE_VAL;
                        continue;
                    }
                    double dDist = sqrt(sPoint.x * sPoint.x + sPoint.y * sPoint.y + sPoint.z * sPoint.z);
                    dMaxError = std::max(dMaxError, fabs(dDist - dExpectedRadius));
                }
            }
            sstring = TfStringPrintf("  Face[%u]: SmBSplineSurface, max radial error %.3g\n", iFace, dMaxError);
            usdBrep_WriteString(sstring);
            if (dMaxError > dTolerance)
            {
                sstring = TfStringPrintf("  FAIL: Face[%u] surface points are not on the sphere\n", iFace);
                usdBrep_WriteString(sstring);
                sRtn = SM_ERR;
            }
        }
    }

    // Cleanup
    for (u_int ii = 0; ii < sBreps.GetSize(); ii++)
        if (sBreps[ii]) delete sBreps[ii];

    if (sRtn == SM_SUCCESS)
        usdBrep_WriteString("PASSED: my_test_sphere_nurbs_import\n\n");
    else
        usdBrep_WriteString("FAILED: my_test_sphere_nurbs_import\n\n");

    return sRtn;

} // end my_test_sphere_nurbs_import

static SmStatus createBoxBrepArrayStage(UsdStageRefPtr& rStage, UsdPrim& rBrepArrayPrim, UsdBrepArrayData& rArrays)
{
    SmContext sContext;
    SmBrep* pBrep = new (sContext) SmBrep();
    SmObjDelete sClean(pBrep);

    SmPrimitiveCreation sPrimitiveCreation(pBrep->GetInfiniteRegion());
    SER(sPrimitiveCreation.CreateBox(1.0, 1.0, 1.0, SmAxis2Placement()));

    std::vector<SmBrep*> sBreps = { pBrep };
    rArrays.ReSet();
    SER(SMU_BrepConvert::BrepAppend_SMLibToUsdBrep(sBreps, rArrays, nullptr, false /*bExportUVCurves*/));

    rStage = UsdStage::CreateInMemory();
    if (!rStage)
    {
        usdBrep_WriteString("  FAIL: could not create in-memory USD stage\n");
        return SM_ERR;
    }

    rBrepArrayPrim = rStage->DefinePrim(SdfPath("/World/brepArray"), TfToken("BrepArray"));
    if (!rBrepArrayPrim || !BrepWriteToUsdStage(rArrays, rBrepArrayPrim))
    {
        usdBrep_WriteString("  FAIL: could not author box BrepArray\n");
        return SM_ERR;
    }

    return SM_SUCCESS;
}

static SmStatus createSphereBrepArrayStage(
    size_t lBrepCount,
    UsdStageRefPtr& rStage,
    UsdPrim& rBrepArrayPrim,
    UsdBrepArrayData& rArrays)
{
    if (lBrepCount == 0)
    {
        return SM_ERR_INVALID_INPUT;
    }

    SmContext sContext;
    std::vector<SmBrep*> sBreps;
    for (size_t ii = 0; ii < lBrepCount; ++ii)
    {
        SmBrep* pBrep = new (sContext) SmBrep();
        sBreps.push_back(pBrep);

        SmPrimitiveCreation sPrimitiveCreation(pBrep->GetInfiniteRegion());
        const SmStatus sCreateStatus =
            sPrimitiveCreation.CreateSphere(static_cast<double>(ii + 1), 0.0, 360.0, SmAxis2Placement());
        if (sCreateStatus != SM_SUCCESS)
        {
            sm_CleanupSmBrepVector(sBreps);
            return sCreateStatus;
        }
    }

    rArrays.ReSet();
    const SmStatus sAppendStatus =
        SMU_BrepConvert::BrepAppend_SMLibToUsdBrep(sBreps, rArrays, nullptr, false /*bExportUVCurves*/);
    sm_CleanupSmBrepVector(sBreps);
    if (sAppendStatus != SM_SUCCESS)
    {
        return sAppendStatus;
    }

    if (rArrays.TotalBrepCount() != lBrepCount || rArrays.m_sFace_SurfaceSphere_RadiusArray.empty())
    {
        usdBrep_WriteString(
            "  FAIL: writer-produced sphere BrepArray did not contain the requested Breps and sphere surfaces\n");
        return SM_ERR;
    }

    rStage = UsdStage::CreateInMemory();
    if (!rStage)
    {
        usdBrep_WriteString("  FAIL: could not create in-memory USD stage\n");
        return SM_ERR;
    }

    rBrepArrayPrim = rStage->DefinePrim(SdfPath("/World/brepArray"), TfToken("BrepArray"));
    if (!rBrepArrayPrim || !BrepWriteToUsdStage(rArrays, rBrepArrayPrim))
    {
        usdBrep_WriteString("  FAIL: could not author sphere BrepArray\n");
        return SM_ERR;
    }

    return SM_SUCCESS;
}

static bool setFirstPackedSphereRadius(UsdBrepArrayData& rArrays, size_t lBrepIndex, double dRadius)
{
    UsdBrepArraySpans sSpans;
    if (!sSpans.SetStartsAndCountsForBrepIndex(rArrays, static_cast<uint32_t>(lBrepIndex), TRUE, TRUE) ||
        sSpans.m_lFaceSphereSurface_Count == 0 ||
        sSpans.m_lFaceSphereSurface_StartIndex >= rArrays.m_sFace_SurfaceSphere_RadiusArray.size())
    {
        return false;
    }

    rArrays.m_sFace_SurfaceSphere_RadiusArray[sSpans.m_lFaceSphereSurface_StartIndex] = dRadius;
    return true;
}

static bool setExplicitAppliedSchemas(const UsdPrim& rPrim, const TfTokenVector& rSchemas)
{
    if (!rPrim || !rPrim.GetStage() || !rPrim.GetStage()->GetRootLayer())
    {
        return false;
    }

    SdfPrimSpecHandle sPrimSpec = rPrim.GetStage()->GetRootLayer()->GetPrimAtPath(rPrim.GetPath());
    if (!sPrimSpec)
    {
        return false;
    }

    SdfTokenListOp sSchemasListOp = SdfTokenListOp::CreateExplicit(rSchemas);
    {
        SdfChangeBlock sChangeBlock;
        sPrimSpec->SetInfo(UsdTokens->apiSchemas, VtValue(sSchemasListOp));
    }
    return true;
}

static bool removeAppliedSchema(const UsdPrim& rPrim, const TfToken& rSchema)
{
    TfTokenVector sSchemas = rPrim.GetAppliedSchemas();
    const auto sNewEnd = std::remove(sSchemas.begin(), sSchemas.end(), rSchema);
    if (sNewEnd == sSchemas.end())
    {
        return false;
    }

    sSchemas.erase(sNewEnd, sSchemas.end());
    return setExplicitAppliedSchemas(rPrim, sSchemas);
}

/***********************************************************************
PURPOSE --- Regression: geometry type tokens require their matching applied
            geometry APIs before BrepArray data reaches the SMLib converter.

NOTES --- Builds a valid analytic box with the current writer, then edits only
          apiSchemas or one packed discriminator. This covers required API
          rejection, optional/unused API acceptance, reader-status propagation,
          and packed-span failure propagation without maintaining a large USD
          fixture.
***********************************************************************/
static SmStatus usdBrep_TestRequiredGeometryApis()
{
    usdBrep_WriteString("\n\n***************************************************************************************************\n");
    usdBrep_WriteString("Test, usdBrep_TestRequiredGeometryApis: applied geometry API validation\n");
    usdBrep_WriteString("***************************************************************************************************\n");

    // A writer-produced box uses plane surfaces, has no sphere surfaces, and
    // deliberately exports no optional pcurves. Missing APIs for the two unused
    // geometry families must therefore remain valid.
    {
        UsdStageRefPtr sStage;
        UsdPrim sBrepArrayPrim;
        UsdBrepArrayData sSourceArrays;
        SER(createBoxBrepArrayStage(sStage, sBrepArrayPrim, sSourceArrays));

        const bool bAllFacesArePlanes = !sSourceArrays.m_sFaceSurfaceTypeArray.empty() &&
                                        std::all_of(
                                            sSourceArrays.m_sFaceSurfaceTypeArray.begin(),
                                            sSourceArrays.m_sFaceSurfaceTypeArray.end(),
                                            [](const TfToken& rType)
                                            {
                                                return rType == UsdBrepSurfaceTokens->brepSurfacePlaneAPI;
                                            }
                                        );
        if (!bAllFacesArePlanes || !sBrepArrayPrim.HasAPI(UsdBrepSurfaceTokens->brepSurfacePlaneAPI))
        {
            usdBrep_WriteString("  FAIL: writer-produced box did not use the required plane API\n");
            return SM_ERR;
        }
        if (sBrepArrayPrim.HasAPI(UsdBrepSurfaceTokens->brepSurfaceSphereAPI) || sBrepArrayPrim.HasAPI(UsdBrepCurveTokens->brepCurveUvNurbAPI))
        {
            usdBrep_WriteString("  FAIL: writer-produced box unexpectedly authored an unused geometry API\n");
            return SM_ERR;
        }

        UsdBrepArrayData sReadArrays;
        if (!BrepReadFromUsdStage(sBrepArrayPrim, sReadArrays))
        {
            usdBrep_WriteString("  FAIL: reader rejected missing APIs with no matching geometry occurrence\n");
            return SM_ERR;
        }
        if (sReadArrays.m_sEdgeuse_CurveNurb_OrderArray.size() != sReadArrays.m_sEdgeuseEdgeIndexArray.size() ||
            !std::all_of(
                sReadArrays.m_sEdgeuse_CurveNurb_OrderArray.begin(),
                sReadArrays.m_sEdgeuse_CurveNurb_OrderArray.end(),
                [](uint32_t lOrder)
                {
                    return lOrder == 0;
                }
            ))
        {
            usdBrep_WriteString("  FAIL: absent pcurve API did not produce missing-curve records\n");
            return SM_ERR;
        }
    }

    // Omitting the API required by an analytic surface must fail both the raw
    // reader and the public converter. The converter must return an error without
    // attempting a partial conversion or crashing on the absent plane arrays.
    {
        UsdStageRefPtr sStage;
        UsdPrim sBrepArrayPrim;
        UsdBrepArrayData sSourceArrays;
        SER(createBoxBrepArrayStage(sStage, sBrepArrayPrim, sSourceArrays));
        if (!removeAppliedSchema(sBrepArrayPrim, UsdBrepSurfaceTokens->brepSurfacePlaneAPI) ||
            sBrepArrayPrim.HasAPI(UsdBrepSurfaceTokens->brepSurfacePlaneAPI))
        {
            usdBrep_WriteString("  FAIL: could not remove BrepSurfacePlaneAPI from test stage\n");
            return SM_ERR;
        }

        UsdBrepArrayData sReadArrays;
        if (BrepReadFromUsdStage(sBrepArrayPrim, sReadArrays))
        {
            usdBrep_WriteString("  FAIL: reader accepted a plane occurrence without BrepSurfacePlaneAPI\n");
            return SM_ERR;
        }

        SmContext sContext;
        // Seed existing output to verify that a rejected import does not alter
        // or consume caller-owned entries in its result container.
        SmBrep* pExistingBrep = new (sContext) SmBrep();
        std::vector<SmBrep*> sBreps(1, pExistingBrep);
        const SmStatus sMoveStatus = SMU_BrepConvert::BrepMove_UsdToSMLib(sContext, UsdGeomGprim(sBrepArrayPrim), sBreps, FALSE /*bHealerIsEnabled*/);
        if (sMoveStatus == SM_SUCCESS || sBreps.size() != 1 || sBreps[0] != pExistingBrep)
        {
            usdBrep_WriteString("  FAIL: converter did not reject the malformed BrepArray cleanly\n");
            sm_CleanupSmBrepVector(sBreps);
            return SM_ERR;
        }
        sm_CleanupSmBrepVector(sBreps);
    }

    // Removing every applied API from the valid stage must be rejected because
    // the box still contains vertex, edge-curve, and face-surface occurrences.
    {
        UsdStageRefPtr sStage;
        UsdPrim sBrepArrayPrim;
        UsdBrepArrayData sSourceArrays;
        SER(createBoxBrepArrayStage(sStage, sBrepArrayPrim, sSourceArrays));
        if (!setExplicitAppliedSchemas(sBrepArrayPrim, TfTokenVector()) || !sBrepArrayPrim.GetAppliedSchemas().empty())
        {
            usdBrep_WriteString("  FAIL: could not remove all applied APIs from test stage\n");
            return SM_ERR;
        }

        UsdBrepArrayData sReadArrays;
        if (BrepReadFromUsdStage(sBrepArrayPrim, sReadArrays))
        {
            usdBrep_WriteString("  FAIL: reader accepted geometry occurrences with all required APIs omitted\n");
            return SM_ERR;
        }
    }

    // Exercise the lower-level packed-array guard directly. Reclassifying one
    // valid plane face as NURBS without supplying NURBS metadata makes span setup
    // fail; that failure must also propagate through the one-Brep converter.
    {
        UsdStageRefPtr sStage;
        UsdPrim sBrepArrayPrim;
        UsdBrepArrayData sMalformedArrays;
        SER(createBoxBrepArrayStage(sStage, sBrepArrayPrim, sMalformedArrays));
        if (sMalformedArrays.m_sFaceSurfaceTypeArray.empty() || !sMalformedArrays.m_sFace_SurfaceNurb_UVertexCountArray.empty() ||
            !sMalformedArrays.m_sFace_SurfaceNurb_VVertexCountArray.empty() || !sMalformedArrays.m_sFace_SurfaceNurb_UOrderArray.empty() ||
            !sMalformedArrays.m_sFace_SurfaceNurb_VOrderArray.empty())
        {
            usdBrep_WriteString("  FAIL: box arrays are unsuitable for packed-span malformed-data test\n");
            return SM_ERR;
        }
        sMalformedArrays.m_sFaceSurfaceTypeArray[0] = UsdBrepSurfaceTokens->brepSurfaceNurbAPI;

        UsdBrepArraySpans sDirectSpans;
        if (sDirectSpans.SetStartsAndCountsForBrepIndex(sMalformedArrays, 0, FALSE, TRUE))
        {
            usdBrep_WriteString("  FAIL: packed spans accepted truncated NURBS surface metadata\n");
            return SM_ERR;
        }

        SmContext sContext;
        UsdBrepArraySpans sConverterSpans;
        SmBrep* pBrep = nullptr;
        const SmStatus sMoveStatus =
            SMU_BrepConvert::BrepMove_OneUsdBrepToSMLib(sContext, sMalformedArrays, sConverterSpans, 0, pBrep, FALSE /*bHealerIsEnabled*/);
        if (sMoveStatus != SM_ERR_INVALID_INPUT || pBrep != nullptr)
        {
            usdBrep_WriteString("  FAIL: one-Brep converter ignored packed-span failure\n");
            if (pBrep)
            {
                delete pBrep;
            }
            return SM_ERR;
        }
    }

    // The reporting converter must stop when malformed packed spans make the
    // next member's offsets untrustworthy, while returning the failure record
    // and no partially constructed Brep.
    {
        UsdStageRefPtr sStage;
        UsdPrim sBrepArrayPrim;
        UsdBrepArrayData sMalformedArrays;
        SER(createSphereBrepArrayStage(2, sStage, sBrepArrayPrim, sMalformedArrays));
        if (sMalformedArrays.m_sFaceSurfaceTypeArray.size() != 2 ||
            !sMalformedArrays.m_sFace_SurfaceNurb_UVertexCountArray.empty())
        {
            usdBrep_WriteString("  FAIL: sphere arrays are unsuitable for packed-span result test\n");
            return SM_ERR;
        }

        sMalformedArrays.m_sFaceSurfaceTypeArray[0] = UsdBrepSurfaceTokens->brepSurfaceNurbAPI;
        // A non-empty control-point array makes the writer author the NURBS
        // applied API and all of its attributes. Leave the per-surface shape
        // metadata empty so the reader succeeds but packed-span validation
        // rejects the first member before its offsets can be trusted.
        sMalformedArrays.m_sFace_SurfaceNurb_ControlVerticesArray.push_back(GfVec3d(0.0));
        if (!BrepWriteToUsdStage(sMalformedArrays, sBrepArrayPrim))
        {
            usdBrep_WriteString("  FAIL: could not author malformed packed-span result stage\n");
            return SM_ERR;
        }

        SmContext sContext;
        std::vector<SMU_BrepConvert::BrepImportResult> sResults;
        const SmStatus sMoveStatus = SMU_BrepConvert::BrepMove_UsdToSMLibWithResults(
            sContext,
            UsdGeomGprim(sBrepArrayPrim),
            sResults,
            FALSE /*bHealerIsEnabled*/);
        const bool bReportedSafely =
            sMoveStatus == SM_SUCCESS && sResults.size() == 1 &&
            sResults[0].iPackedBrepIndex == 0 &&
            sResults[0].status == SM_ERR_INVALID_INPUT &&
            sResults[0].pBrep == nullptr &&
            sResults[0].bRemainingMembersSkipped;
        for (SMU_BrepConvert::BrepImportResult& rResult : sResults)
        {
            delete rResult.pBrep;
            rResult.pBrep = nullptr;
        }
        if (!bReportedSafely)
        {
            usdBrep_WriteString("  FAIL: malformed packed span was not reported safely\n");
            return SM_ERR;
        }
    }

    // BrepArray material binding and per-Brep subsets written for two spheres
    {
        const SdfPath sArrayMaterial("/World/Looks/Array");
        const SdfPath sBrepMaterial("/World/Looks/Brep");
        struct Case
        {
            const char* pName;
            SdfPath sArrayPath;
            SdfPath sEntryPath;   // per-Brep entry, written when sIndices is nonempty
            VtIntArray sIndices;
            SdfPathVector sExpectedTargets;
            bool bExpectSubset;
        };
        const Case aCases[] = {
            { "BrepArray material, no per-Brep entries", sArrayMaterial, SdfPath(), {}, { sArrayMaterial }, false },
            { "BrepArray material, entry covering Brep 0 only", sArrayMaterial, sBrepMaterial, { 0 }, { sArrayMaterial }, true },
            { "indices {1, 0} cover both Breps", SdfPath(), sBrepMaterial, { 1, 0 }, { sBrepMaterial }, false },
            { "duplicate indices {0, 0} do not cover Brep 1", SdfPath(), sBrepMaterial, { 0, 0 }, {}, true },
            { "out-of-range index does not cover Brep 1", SdfPath(), sBrepMaterial, { 0, 2 }, {}, true },
            { "empty-path entry covering both keeps the BrepArray material", sArrayMaterial, SdfPath(), { 0, 1 }, { sArrayMaterial }, false },
            { "empty-path entry covering Brep 0 writes no subset", SdfPath(), SdfPath(), { 0 }, {}, false },
        };
        for (const Case& crCase : aCases)
        {
            UsdStageRefPtr sStage;
            UsdPrim sBrepArrayPrim;
            UsdBrepArrayData sArrays;
            SER(createSphereBrepArrayStage(2, sStage, sBrepArrayPrim, sArrays));
            sArrays.m_sBrepArray_MaterialPath = crCase.sArrayPath;
            sArrays.m_sBrepMaterial_BrepPathArray.clear();
            sArrays.m_sBrepMaterial_BrepIndexArray.clear();
            if (!crCase.sIndices.empty())
            {
                sArrays.m_sBrepMaterial_BrepPathArray.push_back(crCase.sEntryPath);
                sArrays.m_sBrepMaterial_BrepIndexArray.push_back(crCase.sIndices);
            }
            SdfPathVector sTargets;
            const bool bWritten = BrepWriteToUsdStage(sArrays, sBrepArrayPrim);
            sBrepArrayPrim.GetRelationship(UsdShadeTokens->materialBinding).GetTargets(&sTargets);
            const bool bSubset = sStage->GetPrimAtPath(sBrepArrayPrim.GetPath().AppendChild(TfToken("subset_0"))).IsValid();
            // the binding resolves with legacy bindings off, which needs MaterialBindingAPI applied
            for (const SdfPath& crPath : crCase.sExpectedTargets)
                UsdShadeMaterial::Define(sStage, crPath);
            const UsdShadeMaterial sBound = UsdShadeMaterialBindingAPI(sBrepArrayPrim).ComputeBoundMaterial(UsdShadeTokens->allPurpose, nullptr, false);
            const SdfPathVector sBoundPaths = sBound ? SdfPathVector{ sBound.GetPath() } : SdfPathVector();
            if (!bWritten || sTargets != crCase.sExpectedTargets || bSubset != crCase.bExpectSubset || sBoundPaths != crCase.sExpectedTargets)
            {
                usdBrep_WriteString(std::string("  FAIL: BrepArray material binding: ") + crCase.pName + "\n");
                return SM_ERR;
            }
        }

        // A material path without its index list, per Brep or per face, is rejected.
        for (bool bFace : { false, true })
        {
            UsdStageRefPtr sStage;
            UsdPrim sBrepArrayPrim;
            UsdBrepArrayData sArrays;
            SER(createSphereBrepArrayStage(2, sStage, sBrepArrayPrim, sArrays));
            sArrays.m_sBrepMaterial_BrepPathArray.clear();
            sArrays.m_sBrepMaterial_BrepIndexArray.clear();
            sArrays.m_sFaceMaterial_FacePathArray.clear();
            sArrays.m_sFaceMaterial_FaceIndexArray.clear();
            (bFace ? sArrays.m_sFaceMaterial_FacePathArray : sArrays.m_sBrepMaterial_BrepPathArray).push_back(sBrepMaterial);
            if (BrepWriteToUsdStage(sArrays, sBrepArrayPrim))
            {
                usdBrep_WriteString(bFace ? "  FAIL: writer accepted unpaired face material arrays\n"
                                          : "  FAIL: writer accepted unpaired Brep material arrays\n");
                return SM_ERR;
            }
        }

        // A skipped per-Brep entry must not leave a subset name that a face subset then reuses.
        {
            UsdStageRefPtr sStage;
            UsdPrim sBrepArrayPrim;
            UsdBrepArrayData sArrays;
            SER(createSphereBrepArrayStage(2, sStage, sBrepArrayPrim, sArrays));
            sArrays.m_sBrepArray_MaterialPath = SdfPath();
            sArrays.m_sBrepMaterial_BrepPathArray = { SdfPath(), sBrepMaterial };
            sArrays.m_sBrepMaterial_BrepIndexArray = { VtIntArray{ 0 }, VtIntArray{ 1 } };
            sArrays.m_sFaceMaterial_FacePathArray = { sArrayMaterial };
            sArrays.m_sFaceMaterial_FaceIndexArray = { VtIntArray{ 0 } };
            auto elementType = [&](const char* pName)
            {
                TfToken sType;
                UsdPrim sSubset = sStage->GetPrimAtPath(sBrepArrayPrim.GetPath().AppendChild(TfToken(pName)));
                return sSubset && sSubset.GetAttribute(UsdGeomTokens->elementType).Get(&sType) ? sType : TfToken();
            };
            if (!BrepWriteToUsdStage(sArrays, sBrepArrayPrim) || elementType("subset_0") != UsdBrepSolidTokens->brep ||
                elementType("subset_1") != UsdBrepSolidTokens->face)
            {
                usdBrep_WriteString("  FAIL: Brep and face material subsets do not get distinct names\n");
                return SM_ERR;
            }
        }
    }

    usdBrep_WriteString("  OK: required geometry API and packed-span failures are rejected safely\n");
    return SM_SUCCESS;
}

// Material paths bound to each imported Brep of crArrays; empty when unbound.
static SmStatus usdBrep_ImportBrepMaterials(const UsdBrepArrayData& crArrays, std::vector<SdfPathVector>& rMaterials)
{
    SmContext sContext;
    UsdBrepArraySpans sSpans;
    rMaterials.clear();
    for (uint32_t ii = 0; ii < crArrays.TotalBrepCount(); ++ii)
    {
        SmBrep* pBrep = NULL;
        SER(SMU_BrepConvert::BrepMove_OneUsdBrepToSMLib(sContext, crArrays, sSpans, ii, pBrep, FALSE /*bHealerIsEnabled*/));
        SmObjDelete sClean(pBrep);
        SmSdfPathAttribute* pAttr = SM_CAST_PTR(SmSdfPathAttribute, pBrep ? pBrep->FindAttribute(SM_AI_MATERIAL_BINDING) : NULL);
        rMaterials.push_back(pAttr && pAttr->GetMaterialPaths() ? *pAttr->GetMaterialPaths() : SdfPathVector());
    }
    return SM_SUCCESS;
}

/***********************************************************************
PURPOSE --- Regression: a Brep that no per-Brep material subset covers
            takes the BrepArray material, and an empty BrepArray material
            path leaves the Brep unbound rather than bound to nothing.
***********************************************************************/
static SmStatus usdBrep_TestBrepMaterialImportFallback()
{
    usdBrep_WriteString("\n\n***************************************************************************************************\n");
    usdBrep_WriteString("Test, usdBrep_TestBrepMaterialImportFallback: BrepArray material fallback on import\n");
    usdBrep_WriteString("***************************************************************************************************\n");

    // Two Breps; a per-Brep subset covers only Brep 0.
    {
        UsdStageRefPtr sStage;
        UsdPrim sBrepArrayPrim;
        UsdBrepArrayData sArrays;
        SER(createSphereBrepArrayStage(2, sStage, sBrepArrayPrim, sArrays));
        const SdfPath sArrayMaterial("/World/Looks/Array");
        const SdfPath sBrep0Material("/World/Looks/Brep0");
        sArrays.m_sBrepArray_MaterialPath = sArrayMaterial;
        sArrays.m_sBrepMaterial_BrepPathArray = { sBrep0Material };
        sArrays.m_sBrepMaterial_BrepIndexArray = { VtIntArray{ 0 } };
        std::vector<SdfPathVector> sMaterials;
        SER(usdBrep_ImportBrepMaterials(sArrays, sMaterials));
        if (sMaterials.size() != 2 || sMaterials[0] != SdfPathVector{ sBrep0Material } || sMaterials[1] != SdfPathVector{ sArrayMaterial })
        {
            usdBrep_WriteString("  FAIL: a Brep outside every per-Brep subset did not take the BrepArray material\n");
            return SM_ERR;
        }
    }

    // No per-Brep subsets and no BrepArray material.
    {
        UsdStageRefPtr sStage;
        UsdPrim sBrepArrayPrim;
        UsdBrepArrayData sArrays;
        SER(createBoxBrepArrayStage(sStage, sBrepArrayPrim, sArrays));
        sArrays.m_sBrepArray_MaterialPath = SdfPath();
        sArrays.m_sBrepMaterial_BrepPathArray.clear();
        sArrays.m_sBrepMaterial_BrepIndexArray.clear();
        std::vector<SdfPathVector> sMaterials;
        SER(usdBrep_ImportBrepMaterials(sArrays, sMaterials));
        if (sMaterials.size() != 1 || !sMaterials[0].empty())
        {
            usdBrep_WriteString("  FAIL: an empty BrepArray material path produced a material binding\n");
            return SM_ERR;
        }
    }

    usdBrep_WriteString("PASSED: usdBrep_TestBrepMaterialImportFallback\n\n");
    return SM_SUCCESS;
} // end usdBrep_TestBrepMaterialImportFallback

/***********************************************************************
PURPOSE --- Regression: shell-point geometry is packed only for shells
            with no faceuses, no wire edges, and BrepPointAPI point type.

NOTES --- The point-type field is ignored on face and wire shells. Exercise
          reader API preflight separately, then put ignored tokens before a
          genuine point shell in a multi-Brep array and verify that the later
          shell's geometry span and imported position are not shifted.
***********************************************************************/
static SmStatus usdBrep_TestPointShellOccurrencePacking()
{
    usdBrep_WriteString("\n\n***************************************************************************************************\n");
    usdBrep_WriteString("Test, usdBrep_TestPointShellOccurrencePacking: exact point-shell occurrence packing\n");
    usdBrep_WriteString("***************************************************************************************************\n");

    // A BrepPointAPI token on a face shell is ignored, so it must not require
    // the shellPoint applied API or a packed shell-point position.
    {
        UsdStageRefPtr sStage;
        UsdPrim sBrepArrayPrim;
        UsdBrepArrayData sArrays;
        SER(createBoxBrepArrayStage(sStage, sBrepArrayPrim, sArrays));

        size_t lFaceShellIndex = sArrays.m_sShellFaceuseCountArray.size();
        for (size_t ii = 0; ii < sArrays.m_sShellFaceuseCountArray.size(); ++ii)
        {
            if (sArrays.m_sShellFaceuseCountArray[ii] > 0)
            {
                lFaceShellIndex = ii;
                break;
            }
        }
        if (lFaceShellIndex >= sArrays.m_sShellPointTypeArray.size())
        {
            usdBrep_WriteString("  FAIL: writer-produced box has no face shell\n");
            return SM_ERR;
        }

        sArrays.m_sShellPointTypeArray[lFaceShellIndex] = UsdBrepSolidTokens->brepPointAPI;
        if (sArrays.IsBrepPointShell(static_cast<uint32_t>(lFaceShellIndex)) || sArrays.TotalShellVertexCount() != 0 ||
            !sArrays.m_sShell_PointPositionArray.empty() || !BrepWriteToUsdStage(sArrays, sBrepArrayPrim) ||
            sBrepArrayPrim.HasAPI(UsdBrepSolidTokens->brepPointAPI, UsdBrepSolidTokens->shellPoint))
        {
            usdBrep_WriteString("  FAIL: ignored face-shell point token was treated as shell-point geometry\n");
            return SM_ERR;
        }

        UsdBrepArrayData sReadArrays;
        if (!BrepReadFromUsdStage(sBrepArrayPrim, sReadArrays) || sReadArrays.TotalShellVertexCount() != 0)
        {
            usdBrep_WriteString("  FAIL: reader required shellPoint API for an ignored face-shell token\n");
            return SM_ERR;
        }
    }

    // Pack three Breps in this order: a face shell, a wire shell, then a
    // genuine point shell. Changing the ignored discriminator on the first two
    // must not consume positions or move the third Brep's point span.
    {
        SmContext sSourceContext;
        SmBrep* pFaceBrep = new (sSourceContext) SmBrep();
        SmBrep* pWireBrep = new (sSourceContext) SmBrep();
        SmBrep* pPointBrep = new (sSourceContext) SmBrep();
        SmObjDelete sFaceClean(pFaceBrep);
        SmObjDelete sWireClean(pWireBrep);
        SmObjDelete sPointClean(pPointBrep);

        SmPrimitiveCreation sPrimitiveCreation(pFaceBrep->GetInfiniteRegion());
        SER(sPrimitiveCreation.CreateBox(1.0, 1.0, 1.0, SmAxis2Placement()));

        SmBSplineCurve* pLine = nullptr;
        SER(SmBSplineCurve::CreateLineSegment(
            sSourceContext,
            3,
            SmPoint3d(2.0, 0.0, 0.0),
            SmPoint3d(3.0, 0.0, 0.0),
            pLine));
        SmEdge* pWireEdge = nullptr;
        SER(pWireBrep->CreateWireEdgeFromCurve(pLine, pLine->GetNaturalInterval(), pWireEdge));

        const SmPoint3d sExpectedPoint(7.0, 8.0, 9.0);
        SmShell* pPointShell = nullptr;
        SmVertex* pPointVertex = nullptr;
        SER(pPointBrep->MakeShellVertex(pPointBrep->GetInfiniteRegion(), sExpectedPoint, pPointShell, pPointVertex));

        std::vector<SmBrep*> sBreps = { pFaceBrep, pWireBrep, pPointBrep };
        UsdBrepArrayData sArrays;
        SER(SMU_BrepConvert::BrepAppend_SMLibToUsdBrep(sBreps, sArrays, nullptr, false /*bExportUVCurves*/));

        size_t lFaceShellIndex = sArrays.m_sShellPointTypeArray.size();
        size_t lWireShellIndex = sArrays.m_sShellPointTypeArray.size();
        size_t lPointShellIndex = sArrays.m_sShellPointTypeArray.size();
        for (size_t ii = 0; ii < sArrays.m_sShellPointTypeArray.size(); ++ii)
        {
            if (lFaceShellIndex == sArrays.m_sShellPointTypeArray.size() && sArrays.m_sShellFaceuseCountArray[ii] > 0)
            {
                lFaceShellIndex = ii;
            }
            if (lWireShellIndex == sArrays.m_sShellPointTypeArray.size() && sArrays.m_sShellWireEdgeCountArray[ii] > 0)
            {
                lWireShellIndex = ii;
            }
            if (lPointShellIndex == sArrays.m_sShellPointTypeArray.size() && sArrays.IsBrepPointShell(static_cast<uint32_t>(ii)))
            {
                lPointShellIndex = ii;
            }
        }
        if (!(lFaceShellIndex < lWireShellIndex && lWireShellIndex < lPointShellIndex) ||
            sArrays.m_sShellPointTypeArray[lFaceShellIndex] != UsdBrepSolidTokens->none ||
            sArrays.m_sShellPointTypeArray[lWireShellIndex] != UsdBrepSolidTokens->none ||
            sArrays.TotalShellVertexCount() != 1 || sArrays.m_sShell_PointPositionArray.size() != 1)
        {
            usdBrep_WriteString("  FAIL: writer did not canonically encode the expected face, wire, and point shells\n");
            return SM_ERR;
        }

        sArrays.m_sShellPointTypeArray[lFaceShellIndex] = UsdBrepSolidTokens->brepPointAPI;
        sArrays.m_sShellPointTypeArray[lWireShellIndex] = UsdBrepSolidTokens->brepPointAPI;
        if (sArrays.IsBrepPointShell(static_cast<uint32_t>(lFaceShellIndex)) ||
            sArrays.IsBrepPointShell(static_cast<uint32_t>(lWireShellIndex)) ||
            !sArrays.IsBrepPointShell(static_cast<uint32_t>(lPointShellIndex)) ||
            sArrays.IsBrepPointShell(static_cast<uint32_t>(sArrays.m_sShellPointTypeArray.size())) ||
            sArrays.TotalShellVertexCount() != 1)
        {
            usdBrep_WriteString("  FAIL: exact point-shell predicate counted an ignored or out-of-range token\n");
            return SM_ERR;
        }

        UsdBrepArraySpans sSpans;
        if (!sSpans.SetStartsAndCountsForBrepIndex(sArrays, 2, TRUE, TRUE) ||
            sSpans.m_lShellVertexStartIndex != 0 || sSpans.m_lShellVertexCount != 1 ||
            sSpans.m_lShellPointPosition_StartIndex != 0 || sSpans.m_lShellPointPosition_Count != 1)
        {
            usdBrep_WriteString("  FAIL: ignored shell tokens shifted the later point shell's packed span\n");
            return SM_ERR;
        }

        const GfVec3d& rPackedPoint = sArrays.m_sShell_PointPositionArray[sSpans.m_lShellPointPosition_StartIndex];
        if (GfVec3d(sExpectedPoint.x, sExpectedPoint.y, sExpectedPoint.z) != rPackedPoint)
        {
            usdBrep_WriteString("  FAIL: later point shell's packed position was not preserved\n");
            return SM_ERR;
        }

        SmContext sImportContext;
        UsdBrepArraySpans sImportSpans;
        SmBrep* pImportedBrep = nullptr;
        const SmStatus sImportStatus = SMU_BrepConvert::BrepMove_OneUsdBrepToSMLib(
            sImportContext,
            sArrays,
            sImportSpans,
            2,
            pImportedBrep,
            FALSE /*bHealerIsEnabled*/);
        if (sImportStatus != SM_SUCCESS || pImportedBrep == nullptr)
        {
            usdBrep_WriteString("  FAIL: later genuine point shell could not be imported\n");
            delete pImportedBrep;
            return SM_ERR;
        }

        SmTArray<SmVertex*> sImportedVertices;
        pImportedBrep->GetVertices(sImportedVertices);
        const bool bExpectedPosition = sImportedVertices.GetSize() == 1 &&
                                       sImportedVertices[0]->GetPoint().DistanceBetween(sExpectedPoint) <= 1.0e-12;
        delete pImportedBrep;
        if (!bExpectedPosition)
        {
            usdBrep_WriteString("  FAIL: later genuine point shell imported the wrong position\n");
            return SM_ERR;
        }

        // A genuine point shell without its packed position is malformed. It
        // must fail span construction and import rather than reaching an
        // unchecked position-array access in the converter.
        sArrays.m_sShell_PointPositionArray.clear();

        UsdBrepArraySpans sMalformedSpans;
        if (sMalformedSpans.SetStartsAndCountsForBrepIndex(sArrays, 2, TRUE, TRUE))
        {
            usdBrep_WriteString("  FAIL: packed spans accepted a point shell without its position\n");
            return SM_ERR;
        }

        SmContext sMalformedImportContext;
        UsdBrepArraySpans sMalformedImportSpans;
        SmBrep* pMalformedBrep = nullptr;
        const SmStatus sMalformedImportStatus = SMU_BrepConvert::BrepMove_OneUsdBrepToSMLib(
            sMalformedImportContext,
            sArrays,
            sMalformedImportSpans,
            2,
            pMalformedBrep,
            FALSE /*bHealerIsEnabled*/);
        if (sMalformedImportStatus != SM_ERR_INVALID_INPUT || pMalformedBrep != nullptr)
        {
            usdBrep_WriteString("  FAIL: truncated point-shell geometry was not rejected safely\n");
            delete pMalformedBrep;
            return SM_ERR;
        }
    }

    usdBrep_WriteString("  OK: shell point geometry is packed and validated by genuine occurrence\n");
    return SM_SUCCESS;
}

/***********************************************************************
PURPOSE --- Regression: a BrepArray import must fail atomically when any
            packed Brep cannot be converted to SMLib.

NOTES --- Uses writer-produced analytic spheres, then changes one packed
          sphere radius to zero. The USD reader accepts the complete authored
          representation, while SmSphere::CreateCanonical rejects the
          degenerate member during per-Brep conversion.
***********************************************************************/
static SmStatus usdBrep_TestConversionFailurePropagation()
{
    usdBrep_WriteString("\n\n***************************************************************************************************\n");
    usdBrep_WriteString("Test, usdBrep_TestConversionFailurePropagation: atomic BrepArray import failure\n");
    usdBrep_WriteString("***************************************************************************************************\n");

    // A valid multi-member BrepArray must still import every member.
    {
        UsdStageRefPtr sStage;
        UsdPrim sBrepArrayPrim;
        UsdBrepArrayData sArrays;
        SER(createSphereBrepArrayStage(2, sStage, sBrepArrayPrim, sArrays));

        const SdfPath sBrepMaterialPaths[] = {
            SdfPath("/World/Looks/Brep0"),
            SdfPath("/World/Looks/Brep1")};
        const SdfPath sFaceMaterialPaths[] = {
            SdfPath("/World/Looks/Face0"),
            SdfPath("/World/Looks/Face1")};
        sArrays.m_sBrepMaterial_BrepPathArray = {sBrepMaterialPaths[0], sBrepMaterialPaths[1]};
        sArrays.m_sBrepMaterial_BrepIndexArray = {VtIntArray{0}, VtIntArray{1}};
        sArrays.m_sFaceMaterial_FacePathArray = {sFaceMaterialPaths[0], sFaceMaterialPaths[1]};
        sArrays.m_sFaceMaterial_FaceIndexArray = {VtIntArray{0}, VtIntArray{1}};
        if (!BrepWriteToUsdStage(sArrays, sBrepArrayPrim))
        {
            usdBrep_WriteString("  FAIL: could not author material bindings for valid packed Breps\n");
            return SM_ERR;
        }

        SmContext sContext;
        SmBrep* pExistingBrep = new (sContext) SmBrep();
        std::vector<SmBrep*> sBreps = {pExistingBrep};
        const SmStatus sMoveStatus = SMU_BrepConvert::BrepMove_UsdToSMLib(
            sContext,
            UsdGeomGprim(sBrepArrayPrim),
            sBreps,
            FALSE /*bHealerIsEnabled*/);
        const bool bEveryBrepIsPresent = sBreps.size() == 3 && sBreps[0] == pExistingBrep &&
                                         std::all_of(
                                             sBreps.begin() + 1,
                                             sBreps.end(),
                                             [](const SmBrep* pBrep)
                                             {
                                                 return pBrep != nullptr;
                                             });
        if (sMoveStatus != SM_SUCCESS || !bEveryBrepIsPresent)
        {
            usdBrep_WriteString("  FAIL: valid two-member BrepArray did not import two non-null Breps\n");
            sm_CleanupSmBrepVector(sBreps);
            return SM_ERR;
        }

        for (size_t ii = 0; ii < 2; ++ii)
        {
            SmBrep* pImportedBrep = sBreps[ii + 1];
            SmSdfPathAttribute* pBrepMaterial =
                SM_CAST_PTR(SmSdfPathAttribute, pImportedBrep->FindAttribute(SM_AI_MATERIAL_BINDING));
            SmTArray<SmFace*> sFaces;
            pImportedBrep->GetFaces(sFaces);
            SmSdfPathAttribute* pFaceMaterial = sFaces.GetSize() == 1
                                                   ? SM_CAST_PTR(
                                                         SmSdfPathAttribute,
                                                         sFaces[0]->FindAttribute(SM_AI_MATERIAL_BINDING))
                                                   : nullptr;
            if (pBrepMaterial == nullptr || pBrepMaterial->GetMaterialPaths()->size() != 1 ||
                pBrepMaterial->GetMaterialPaths()->front() != sBrepMaterialPaths[ii] ||
                pFaceMaterial == nullptr || pFaceMaterial->GetMaterialPaths()->size() != 1 ||
                pFaceMaterial->GetMaterialPaths()->front() != sFaceMaterialPaths[ii])
            {
                usdBrep_WriteString("  FAIL: packed Brep or face material binding was assigned to the wrong member\n");
                sm_CleanupSmBrepVector(sBreps);
                return SM_ERR;
            }
        }
        sm_CleanupSmBrepVector(sBreps);
    }

    // An unsupported shell encoding reaches topology construction and must
    // return failure without publishing the partially constructed Brep.
    {
        UsdStageRefPtr sStage;
        UsdPrim sBrepArrayPrim;
        UsdBrepArrayData sArrays;
        SER(createSphereBrepArrayStage(1, sStage, sBrepArrayPrim, sArrays));
        sArrays.m_sShellFaceuseCountArray[0] = 0;
        sArrays.m_sShellWireEdgeCountArray[0] = 0;
        sArrays.m_sShellPointTypeArray[0] = UsdBrepSolidTokens->none;

        SmContext sContext;
        UsdBrepArraySpans sSpans;
        SmBrep* pBrep = nullptr;
        const SmStatus sMoveStatus = SMU_BrepConvert::BrepMove_OneUsdBrepToSMLib(
            sContext,
            sArrays,
            sSpans,
            0,
            pBrep,
            FALSE /*bHealerIsEnabled*/);
        if (sMoveStatus == SM_SUCCESS || pBrep != nullptr)
        {
            usdBrep_WriteString("  FAIL: unsupported shell encoding published a partial Brep\n");
            delete pBrep;
            return SM_ERR;
        }

        if (!BrepWriteToUsdStage(sArrays, sBrepArrayPrim))
        {
            usdBrep_WriteString("  FAIL: could not author unsupported shell encoding\n");
            return SM_ERR;
        }

        SmBrep* pExistingBrep = new (sContext) SmBrep();
        std::vector<SmBrep*> sBreps = {pExistingBrep};
        const SmStatus sAggregateMoveStatus = SMU_BrepConvert::BrepMove_UsdToSMLib(
            sContext,
            UsdGeomGprim(sBrepArrayPrim),
            sBreps,
            FALSE /*bHealerIsEnabled*/);
        if (sAggregateMoveStatus == SM_SUCCESS || sBreps.size() != 1 || sBreps[0] != pExistingBrep)
        {
            usdBrep_WriteString("  FAIL: topology-phase failure published a partial aggregate result\n");
            sm_CleanupSmBrepVector(sBreps);
            return SM_ERR;
        }
        sm_CleanupSmBrepVector(sBreps);
    }

    // A missing per-Brep tolerance entry must be rejected before indexed access.
    {
        UsdStageRefPtr sStage;
        UsdPrim sBrepArrayPrim;
        UsdBrepArrayData sArrays;
        SER(createSphereBrepArrayStage(2, sStage, sBrepArrayPrim, sArrays));
        sArrays.m_sBrepXSectTol3dArray.resize(1);

        SmContext sContext;
        UsdBrepArraySpans sSpans;
        SmBrep* pBrep = nullptr;
        const SmStatus sMoveStatus = SMU_BrepConvert::BrepMove_OneUsdBrepToSMLib(
            sContext,
            sArrays,
            sSpans,
            1,
            pBrep,
            FALSE /*bHealerIsEnabled*/);
        if (sMoveStatus != SM_ERR_INVALID_INPUT || pBrep != nullptr)
        {
            usdBrep_WriteString("  FAIL: missing Brep tolerance was not rejected safely\n");
            delete pBrep;
            return SM_ERR;
        }
    }

    // A single semantically invalid member must fail without publishing output.
    {
        UsdStageRefPtr sStage;
        UsdPrim sBrepArrayPrim;
        UsdBrepArrayData sArrays;
        SER(createSphereBrepArrayStage(1, sStage, sBrepArrayPrim, sArrays));
        if (!setFirstPackedSphereRadius(sArrays, 0, 0.0) || !BrepWriteToUsdStage(sArrays, sBrepArrayPrim))
        {
            usdBrep_WriteString("  FAIL: could not author zero-radius single-member BrepArray\n");
            return SM_ERR;
        }

        UsdBrepArrayData sReadArrays;
        if (!BrepReadFromUsdStage(sBrepArrayPrim, sReadArrays))
        {
            usdBrep_WriteString("  FAIL: reader rejected the complete zero-radius BrepArray before conversion\n");
            return SM_ERR;
        }

        SmContext sContext;
        std::vector<SmBrep*> sBreps;
        const SmStatus sMoveStatus = SMU_BrepConvert::BrepMove_UsdToSMLib(
            sContext,
            UsdGeomGprim(sBrepArrayPrim),
            sBreps,
            FALSE /*bHealerIsEnabled*/);
        if (sMoveStatus == SM_SUCCESS || !sBreps.empty())
        {
            usdBrep_WriteString("  FAIL: malformed single-member BrepArray did not fail with empty output\n");
            sm_CleanupSmBrepVector(sBreps);
            return SM_ERR;
        }
    }

    // Cover both mixed orders so a failure cannot retain an earlier success or
    // be hidden by a later successful conversion.
    for (size_t lInvalidBrepIndex = 0; lInvalidBrepIndex < 2; ++lInvalidBrepIndex)
    {
        UsdStageRefPtr sStage;
        UsdPrim sBrepArrayPrim;
        UsdBrepArrayData sArrays;
        SER(createSphereBrepArrayStage(2, sStage, sBrepArrayPrim, sArrays));
        if (!setFirstPackedSphereRadius(sArrays, lInvalidBrepIndex, 0.0) ||
            !BrepWriteToUsdStage(sArrays, sBrepArrayPrim))
        {
            usdBrep_WriteString("  FAIL: could not author mixed zero-radius BrepArray\n");
            return SM_ERR;
        }

        UsdBrepArrayData sReadArrays;
        if (!BrepReadFromUsdStage(sBrepArrayPrim, sReadArrays))
        {
            usdBrep_WriteString("  FAIL: reader rejected a complete mixed BrepArray before conversion\n");
            return SM_ERR;
        }

        SmContext sContext;
        std::vector<SmBrep*> sBreps;
        const SmStatus sMoveStatus = SMU_BrepConvert::BrepMove_UsdToSMLib(
            sContext,
            UsdGeomGprim(sBrepArrayPrim),
            sBreps,
            FALSE /*bHealerIsEnabled*/);
        if (sMoveStatus == SM_SUCCESS || !sBreps.empty())
        {
            usdBrep_WriteString(
                TfStringPrintf(
                    "  FAIL: mixed BrepArray with invalid member %zu did not fail with empty output\n",
                    lInvalidBrepIndex));
            sm_CleanupSmBrepVector(sBreps);
            return SM_ERR;
        }

        std::vector<SMU_BrepConvert::BrepImportResult> sResults;
        const SmStatus sResultsStatus = SMU_BrepConvert::BrepMove_UsdToSMLibWithResults(
            sContext,
            UsdGeomGprim(sBrepArrayPrim),
            sResults,
            FALSE /*bHealerIsEnabled*/);
        const size_t lValidBrepIndex = 1 - lInvalidBrepIndex;
        const bool bResultsAreCorrect =
            sResultsStatus == SM_SUCCESS && sResults.size() == 2 &&
            sResults[0].iPackedBrepIndex == 0 && sResults[1].iPackedBrepIndex == 1 &&
            sResults[lInvalidBrepIndex].status == SM_ERR &&
            sResults[lInvalidBrepIndex].pBrep == nullptr &&
            !sResults[lInvalidBrepIndex].bRemainingMembersSkipped &&
            sResults[lValidBrepIndex].status == SM_SUCCESS &&
            sResults[lValidBrepIndex].pBrep != nullptr &&
            !sResults[lValidBrepIndex].bRemainingMembersSkipped;

        for (SMU_BrepConvert::BrepImportResult& rResult : sResults)
        {
            delete rResult.pBrep;
            rResult.pBrep = nullptr;
        }

        if (!bResultsAreCorrect)
        {
            usdBrep_WriteString(
                TfStringPrintf(
                    "  FAIL: result import did not preserve the valid packed sibling for invalid member %zu\n",
                    lInvalidBrepIndex));
            return SM_ERR;
        }
    }

    usdBrep_WriteString(
        "  OK: strict BrepArray conversion is atomic and result import preserves valid packed "
        "members\n");
    return SM_SUCCESS;
}

static bool findFirstBrepArrayPrim(const UsdStageRefPtr& stage, UsdPrim& rOutPrim)
{
    for (const UsdPrim& prim : stage->Traverse())
    {
        if (prim.GetTypeName() == TfToken("BrepArray"))
        {
            rOutPrim = prim;
            return true;
        }
    }
    return false;
}

static SmStatus readBrepArrayFixture(
    const std::string& sFixturePath,
    bool (*validateArrays)(const UsdBrepArrayData&),
    const char* pcSuccessMessage)
{
    if (!std::filesystem::exists(sFixturePath))
    {
        usdBrep_WriteString(TfStringPrintf("  FAIL: fixture not found: %s\n", sFixturePath.c_str()));
        return SM_ERR;
    }

    UsdStageRefPtr stage = UsdStage::Open(sFixturePath);
    if (!stage)
    {
        usdBrep_WriteString("  FAIL: could not open fixture stage\n");
        return SM_ERR;
    }

    UsdPrim brepArrayPrim;
    if (!findFirstBrepArrayPrim(stage, brepArrayPrim))
    {
        usdBrep_WriteString("  FAIL: no BrepArray prim found in fixture\n");
        return SM_ERR;
    }

    UsdBrepArrayData arrays;
    if (!BrepReadFromUsdStage(brepArrayPrim, arrays))
    {
        usdBrep_WriteString("  FAIL: BrepReadFromUsdStage failed\n");
        return SM_ERR;
    }

    if (validateArrays && !validateArrays(arrays))
    {
        usdBrep_WriteString("  FAIL: unexpected UsdBrepArrayData after read\n");
        return SM_ERR;
    }

    usdBrep_WriteString(pcSuccessMessage);
    usdBrep_WriteString("\n");
    return SM_SUCCESS;
}

static bool validateWireEdgeTopologyAbsent(const UsdBrepArrayData& rArrays)
{
    if (!rArrays.m_sWireEdgeCurveTypeArray.empty() || !rArrays.m_sWireEdgeRangeArray.empty() ||
        !rArrays.m_sWireEdgeVertexIndicesArray.empty())
    {
        return false;
    }

    for (uint32_t wireEdgeCount : rArrays.m_sShellWireEdgeCountArray)
    {
        if (wireEdgeCount != 0)
        {
            return false;
        }
    }

    return !rArrays.m_sShellWireEdgeCountArray.empty();
}

/***********************************************************************
PURPOSE --- PopulateMeshAttr (mesh arrays) rejects mismatched subset and
            material arrays instead of indexing past one of them.

NOTES --- crSubsetIndices and crMaterialPaths are paired by index. A size
          mismatch returns SM_ERR_INVALID_INPUT and leaves the layer unchanged,
          whether the mesh is empty or already authored; matching sizes author
          the mesh and one GeomSubset per material.
***********************************************************************/
SmStatus my_test_populate_mesh_attr_checks_subset_arrays()
{
    usdBrep_WriteString("\n\n***************************************************************************************************\n");
    usdBrep_WriteString("Test, my_test_populate_mesh_attr_checks_subset_arrays: paired subset/material array sizes\n");
    usdBrep_WriteString("***************************************************************************************************\n");

    UsdStageRefPtr sStage = UsdStage::CreateInMemory();
    sStage->DefinePrim(SdfPath("/Mesh"), TfToken("Mesh"));
    SdfPrimSpecHandle sSpec = sStage->GetRootLayer()->GetPrimAtPath(SdfPath("/Mesh"));

    const VtArray<GfVec3f> aPoints = { GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(0, 1, 0) };
    const VtArray<GfVec3f> aExtent = { GfVec3f(0, 0, 0), GfVec3f(1, 1, 0) };
    const VtIntArray aCounts = { 3 };
    const VtIntArray aIndices = { 0, 1, 2 };
    const VtArray<VtIntArray> aSubsets = { VtIntArray{ 0 } };
    const VtArray<SdfPath> aTwoMaterials = { SdfPath("/Looks/A"), SdfPath("/Looks/B") };
    const VtArray<SdfPath> aOneMaterial = { SdfPath("/Looks/A") };

    // Mismatched arrays are rejected and the layer is untouched.
    auto rejectsMismatch = [&]() {
        std::string sBefore, sAfter;
        sStage->GetRootLayer()->ExportToString(&sBefore);
        SmStatus sMismatchStat = SMU_BrepConvert::PopulateMeshAttr(aExtent, aPoints, aCounts, aIndices, VtArray<GfVec3f>(),
            VtIntArray(), TfToken(), UsdGeomTokens->none, aSubsets, aTwoMaterials, sSpec);
        return sMismatchStat == SM_ERR_INVALID_INPUT && sStage->GetRootLayer()->ExportToString(&sAfter) && sAfter == sBefore;
    };

    if (!rejectsMismatch())
    {
        usdBrep_WriteString("  FAIL: mismatched arrays were not rejected cleanly on an empty mesh\n");
        return SM_ERR;
    }
    SmStatus sStat = SMU_BrepConvert::PopulateMeshAttr(aExtent, aPoints, aCounts, aIndices, VtArray<GfVec3f>(), VtIntArray(),
                                                       TfToken(), UsdGeomTokens->none, aSubsets, aOneMaterial, sSpec);
    if (sStat != SM_SUCCESS || !sStage->GetPrimAtPath(SdfPath("/Mesh/subset_0")))
    {
        usdBrep_WriteString("  FAIL: matching arrays did not author the mesh subset\n");
        return SM_ERR;
    }
    if (!rejectsMismatch())
    {
        usdBrep_WriteString("  FAIL: mismatched arrays were not rejected cleanly on an authored mesh\n");
        return SM_ERR;
    }
    usdBrep_WriteString("PASSED: my_test_populate_mesh_attr_checks_subset_arrays\n\n");
    return SM_SUCCESS;
} // end my_test_populate_mesh_attr_checks_subset_arrays

/***********************************************************************
PURPOSE --- Regression: wire-edge topology attributes may be absent on USD.

NOTES --- Fixture is a solid whose shells declare zero wire edges and omit
          wireEdge:curveType/range/vertexIndices entirely. BrepReadFromUsdStage
          must succeed and leave the wire-edge topology arrays empty.
***********************************************************************/
SmStatus my_test_optional_wireedge_topology_read()
{
    usdBrep_WriteString("\n\n***************************************************************************************************\n");
    usdBrep_WriteString("Test, my_test_optional_wireedge_topology_read: absent wire-edge topology read\n");
    usdBrep_WriteString("***************************************************************************************************\n");

    return readBrepArrayFixture(
        "../../TestFiles/usd_TestFiles/test_wireEdgeTopology_absent.usda",
        validateWireEdgeTopologyAbsent,
        "  OK: wire-edge topology arrays absent after read");
} // end my_test_optional_wireedge_topology_read

/***********************************************************************
PURPOSE --- Regression: the SMLib B-rep healer must never crash while importing
            a BrepArray, even on poor geometry.

NOTES --- Imports a real Revit wall BrepArray (known to have crashed the healer
          through the Fix_CoinVertices / Fix_DegenFaces use-after-frees) with the
          healer ENABLED. The healer runs inside BrepMove_UsdToSMLib, so this
          exercises it end-to-end via the normal import path. Passing means only
          that the healer ran to completion without crashing; the built solid may
          legitimately still be geometrically invalid (that is not checked here --
          SmBrep::AssertValid / the brep_geometry_validator own that). This is a
          pure in-process crash gate: a regression will SIGSEGV/abort usd_test_app.
***********************************************************************/
SmStatus my_test_healer_no_crash()
{
    usdBrep_WriteString("\n\n***************************************************************************************************\n");
    usdBrep_WriteString("Test, my_test_healer_no_crash: B-rep healer must not crash while importing a BrepArray\n");
    usdBrep_WriteString("***************************************************************************************************\n");

    // Reliable path from the usd_test run directory (tests/usd_test): ../../ is the repo root.
    const std::string sFixture = "../../TestFiles/usd_TestFiles/RevitWallHealerCrash.usda";

    if (!std::filesystem::exists(sFixture))
    {
        // Fixture missing (e.g. Git LFS not pulled): skip rather than fail so a
        // pointer-only checkout does not turn this crash gate into a hard failure.
        usdBrep_WriteString("  SKIP: fixture not found: " + sFixture + "\n");
        return SM_SUCCESS;
    }

    UsdStageRefPtr sStage = UsdStage::Open(sFixture);
    if (!sStage)
    {
        usdBrep_WriteString("  FAIL: could not open fixture stage\n");
        return SM_ERR;
    }

    // Find the BrepArray prim in the fixture.
    UsdGeomGprim sGprim;
    for (const UsdPrim& sPrim : sStage->Traverse())
    {
        if (sPrim.GetTypeName() == TfToken("BrepArray"))
        {
            sGprim = UsdGeomGprim(sPrim);
            break;
        }
    }
    if (!sGprim.GetPrim().IsValid())
    {
        usdBrep_WriteString("  FAIL: no BrepArray prim found in fixture\n");
        return SM_ERR;
    }

    // Import with the healer ENABLED. If the healer regresses, this call takes the
    // process down with a signal -- that crash IS the regression signal. Otherwise
    // it returns here regardless of whether the healed solid is valid.
    SmContext sContext;
    std::vector<SmBrep*> sBreps;
    SmStatus sStat = SMU_BrepConvert::BrepMove_UsdToSMLib(sContext, sGprim, sBreps, TRUE /*bHealerIsEnabled*/);

    if (sStat != SM_SUCCESS)
    {
        usdBrep_WriteString("  FAIL: BrepMove_UsdToSMLib returned error\n");
        sm_CleanupSmBrepVector(sBreps);
        return SM_ERR;
    }

    std::string sMsg = TfStringPrintf("  OK: healer completed without crashing, imported %zu brep(s)\n", sBreps.size());
    usdBrep_WriteString(sMsg);

    sm_CleanupSmBrepVector(sBreps);
    return SM_SUCCESS;
} // end my_test_healer_no_crash

/***********************************************************************
PURPOSE --- Main USD<->UsdBrepArrayData<->UsdBrepObjectData<->SmBrep test routine

NOTES --- todo: pass all build USD breps to the USD brep validator
***********************************************************************/
SmStatus my_test_usd()
{  
    // Register the 'omniSolid/resources' plugin. Required prior to Adding or using AppliedAPIs for BrepArray shape definitions. 
    if(false == RegisterOmniSolidResourcesPlugin())
      { 
        return SM_ERR; // no msg here - already done in RegisterOmniSolidResourcesPlugin()
      }

    SmTArray<SmBrep*> sBreps;
    SmTArray<SmBrep*> sResults;
    SmBoolean bDoTest = TRUE;
    ULONG lTest = 0;
    SmContext sContext;
    SmStatus sRtn = SM_SUCCESS;
    SmStatus sStat;

    // Test Brep Export to and import from Usd
    // NOTE: This MUST run first because it creates mouse.usda which other tests depend on
    while ( bDoTest )
    {
        sStat = my_test_brep_IO(lTest);
        ++lTest;

        if (SM_ERR_INVALID_INPUT == sStat)
        { bDoTest = FALSE; }
        else if (sStat != SM_SUCCESS)
        {
            sRtn = sStat;
            usdBrep_WriteString("Failed: my_test_brep_IO\n\n");
        }
    }

    // Test I/O of brep w/Attributes
    sStat = my_test_brep_attributes(); 
    if ( sStat != SM_SUCCESS)
    {
        sRtn = sStat;
        usdBrep_WriteString("Failed: my_test_brep_attributes\n\n");
    }

    // Test I/O of brep w/Materials
    bDoTest = TRUE;
    lTest = 0;
    while ( bDoTest )
    {
        sStat = my_test_brep_materials(lTest); 
        ++lTest;

        if (SM_ERR_INVALID_INPUT == sStat)
        { bDoTest = FALSE; }
        else if (sStat != SM_SUCCESS)
        {
            sRtn = sStat;
            usdBrep_WriteString("Failed: my_test_brep_materials\n\n");
        }
    }

    // Test I/O of brep w/Materials
    bDoTest = TRUE;
    lTest = 0;
    while ( bDoTest )
    {
        sStat = my_test_brep_array(lTest); 
        ++lTest;

        if (SM_ERR_INVALID_INPUT == sStat)
        { bDoTest = FALSE; }
        else if (sStat != SM_SUCCESS)
        {
            sRtn = sStat;
            usdBrep_WriteString("Failed: my_test_brep_array\n\n");
        }
    }

    // Test I/O of polybrep 
    sStat = my_test_polybrep_IO();
    if ( sStat != SM_SUCCESS)
    {
        sRtn = sStat;
        usdBrep_WriteString("Failed: my_test_polybrep_IO\n\n");
    }

    // Test tolerance IO
    sStat = usdBrep_TestBrepToleranceUsesMaximumTopologyTolerance();
    if (sStat != SM_SUCCESS)
    {
        sRtn = sStat;
        usdBrep_WriteString("Failed: usdBrep_TestBrepToleranceUsesMaximumTopologyTolerance\n\n");
    }

    // Test UVCurve IO
    sStat = my_test_edgeuse_uvcurves();
    if (sStat != SM_SUCCESS)
    {
        sRtn = sStat;
        usdBrep_WriteString("Failed: my_test_edgeuse_uvcurves\n\n");
    }

    // test UsdBrepArray->UsdBrepArrayData->UsdBrepArray roundtrip
    std::string sInputFilename = "../usd_test/OutputFiles/Cube.usda" ;
    std::string sOutputFilename = "../usd_test/OutputFiles/Cube_Roundtrip.usda";
    bool bRtn = my_TestUsdArrayRoundtrip(sInputFilename, sOutputFilename) ;
    if ( bRtn == false)
    {
        sRtn = SM_ERR;
        usdBrep_WriteString("Failed: my_TestUsdArrayRoundtrip\n\n");
    }

    // Test NURBS sphere import from USD
    sStat = my_test_sphere_nurbs_import();
    if ( sStat != SM_SUCCESS)
    {
        sRtn = sStat;
        usdBrep_WriteString("Failed: my_test_sphere_nurbs_import\n\n");
    }

    sStat = my_test_optional_wireedge_topology_read();
    if ( sStat != SM_SUCCESS)
    {
        sRtn = sStat;
        usdBrep_WriteString("Failed: my_test_optional_wireedge_topology_read\n\n");
    }

    sStat = my_test_populate_mesh_attr_checks_subset_arrays();
    if ( sStat != SM_SUCCESS)
    {
        sRtn = sStat;
        usdBrep_WriteString("Failed: my_test_populate_mesh_attr_checks_subset_arrays\n\n");
    }

    sStat = usdBrep_TestRequiredGeometryApis();
    if (sStat != SM_SUCCESS)
    {
        sRtn = sStat;
        usdBrep_WriteString("Failed: usdBrep_TestRequiredGeometryApis\n\n");
    }

    sStat = usdBrep_TestBrepMaterialImportFallback();
    if (sStat != SM_SUCCESS)
    {
        sRtn = sStat;
        usdBrep_WriteString("Failed: usdBrep_TestBrepMaterialImportFallback\n\n");
    }

    sStat = usdBrep_TestPointShellOccurrencePacking();
    if (sStat != SM_SUCCESS)
    {
        sRtn = sStat;
        usdBrep_WriteString("Failed: usdBrep_TestPointShellOccurrencePacking\n\n");
    }

    sStat = usdBrep_TestConversionFailurePropagation();
    if (sStat != SM_SUCCESS)
    {
        sRtn = sStat;
        usdBrep_WriteString("Failed: usdBrep_TestConversionFailurePropagation\n\n");
    }

    // Regression: the B-rep healer must not crash while importing a BrepArray.
    // Run last: a healer regression would crash the process, so keep it from
    // masking the results of the other cases above.
    sStat = my_test_healer_no_crash();
    if ( sStat != SM_SUCCESS)
    {
        sRtn = sStat;
        usdBrep_WriteString("Failed: my_test_healer_no_crash\n\n");
    }

    return sRtn;
    
} // end my_test_usd

/***********************************************************************
PURPOSE --- Brep SmLib<->USD roundtrip tests

NOTES --- for Breps: lTest == 0: Cube
                           == 1: Mouse
                           == 2: 2 boxes, 2 infinite regions
                           == 3: Sphere inside cube
***********************************************************************/
SmStatus my_test_brep_IO(ULONG lTest)
{
    // std locals
    std::string sOutputFilename;

    // SMLib locals
    SmContext sContext;
    SmBrep  * pBrep = new (sContext) SmBrep();

    // Delete the Breps at the end of the scope
    SmTArray<SmBrep*> sBreps;
    SmObjsDelete<SmBrep*> sClean(&sBreps);
    sBreps.Add(pBrep);

    // usd locals
    std::string sBrepArrayName;

    // Load or build SmBrep
    switch (lTest)
      {
      case 0: // cube - manifold
        {
          SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
          sPC.CreateBox(1., 1, 1, SmAxis2Placement());
      
          sOutputFilename = "../usd_test/OutputFiles/Cube.usda";
          sBrepArrayName  = "Cube";
        } break;
      
      case 1: // mouse - manifold
        {
          TCHAR sInFile[SM_TBLOCK_SIZE];
          smos_sprintf(sInFile, _T("%s"), _T("../../TestFiles/pt_TestFiles/Mouse.smb"));
          sOutputFilename = "../usd_test/OutputFiles/mouse.usda";
          sBrepArrayName  = "mouse";
      
          pBrep->ReadFromFile(sContext, sInFile);
        } break;
      
      case 2: // 2 boxes, 2 infinite regions - nonManifold
        {
          SmBrep * pBrep1 = new (sContext) SmBrep();
          SmPrimitiveCreation sPC0(pBrep->GetInfiniteRegion());
          SmPrimitiveCreation sPC1(pBrep1->GetInfiniteRegion());
          sPC0.CreateBox(1., 1, 1, SmAxis2Placement());
      
          SmAxis2Placement sNewOrigin(1, 0, 0, 1, 0, 0, 0, 1, 0);
          sPC1.CreateBox(1, 1, 1, sNewOrigin);
      
          SmMerge sMerge(sContext, pBrep, pBrep1);
          sMerge.NonManifoldBoolean(SM_BO_MERGE, pBrep);
      
          sOutputFilename = "../usd_test/OutputFiles/nonManifoldCubes.usda";
          sBrepArrayName  = "nonManifoldCubes";
        } break;
      
      case 3: // sphere inside cube - manifold
        {
          SmBrep * pBrep1 = new (sContext) SmBrep();
          SmPrimitiveCreation sPC0(pBrep->GetInfiniteRegion());
          SmPrimitiveCreation sPC1(pBrep1->GetInfiniteRegion());
          sPC0.CreateBox(1., 1, 1, SmAxis2Placement());
      
          SmAxis2Placement sNewOrigin(.5, 0.5, 0.5, 1, 0, 0, 0, 1, 0);
          sPC1.CreateSphere(.2, 0., 360., sNewOrigin);
      
          SmMerge sMerge(sContext, pBrep, pBrep1);
          sMerge.NonManifoldBoolean(SM_BO_DIFFERENCE, pBrep);
      
          sOutputFilename = "../usd_test/OutputFiles/cubeWithVoid.usda";
          sBrepArrayName  = "cubeWithVoid";
        } break;

      case 4: // cone from TestFilesSmb
        {
          pBrep->ReadFromFile(sContext, _T("../../TestFilesSmb/cone_0.smb"), SM_ASCII);
          sOutputFilename = "../usd_test/OutputFiles/cone_0.usda";
          sBrepArrayName  = "cone";
        } break;

      case 5: // cylinder from TestFilesSmb
        {
          pBrep->ReadFromFile(sContext, _T("../../TestFilesSmb/cylinder_0.smb"), SM_ASCII);
          sOutputFilename = "../usd_test/OutputFiles/cylinder_0.usda";
          sBrepArrayName  = "cylinder";
        } break;

      case 6: // torus from TestFilesSmb
        {
          pBrep->ReadFromFile(sContext, _T("../../TestFilesSmb/torus_0.smb"), SM_ASCII);
          sOutputFilename = "../usd_test/OutputFiles/torus_0.usda";
          sBrepArrayName  = "torus";
        } break;

      case 7: // plane from TestFilesSmb
        {
          pBrep->ReadFromFile(sContext, _T("../../TestFilesSmb/plane_0.smb"), SM_ASCII);
          sOutputFilename = "../usd_test/OutputFiles/plane_0.usda";
          sBrepArrayName  = "plane";
        } break;

      case 8: // cube (all planes) from TestFilesSmb
        {
          pBrep->ReadFromFile(sContext, _T("../../TestFilesSmb/cube_0.smb"), SM_ASCII);
          sOutputFilename = "../usd_test/OutputFiles/cube_0.usda";
          sBrepArrayName  = "cube";
        } break;

      default:
        { return SM_ERR_INVALID_INPUT; }
      } // end switch on lTest to build pBrep

    // Print test case info

    std::string sstring;

    usdBrep_WriteString("\n\n***************************************************************************************************\n");

    sstring = TfStringPrintf("Starting my_test_brep_IO test #%lu %s\n", lTest, sOutputFilename.c_str());
    usdBrep_WriteString(sstring);

    usdBrep_WriteString("***************************************************************************************************\n");


#ifdef SM_DEBUG_CODE

    // debug locals
    SmBrep                         * pDbgBrep_Start = pBrep ;
    SmBrep                           sDbgBrep_FromSMRoundTrip(SM_USE_DEFAULT, SM_USE_DEFAULT, &sContext) ;
    SmBrep                         * pDbgBrep_FromBrepArrayData = NULL ;

    UsdBrepData::UsdBrepArrayData   sDbgUsdBrepArrayData_FromStartUsdBrepArray ; 
    UsdBrepData::UsdBrepArraySpans  sDbgAfterOffsets ; 

    SmBrepData                       sDbgBrepData_FromStartBrep ;    sDbgBrepData_FromStartBrep.SetContext(&sContext) ;
    SmBrepData                       sDbgBrepData_FromSMRoundTripBrep ; sDbgBrepData_FromSMRoundTripBrep.SetContext(&sContext) ; 
    SmBrepData                       sDbgBrepData_FromBrepArrayData ;     sDbgBrepData_FromBrepArrayData.SetContext(&sContext) ;

    SmTArray<SmAttribute*>           sAllAttributes ;

// ULONG GWC_CHANGE_NEXT_LINE_TO_FALSE_BEFORE_RELEASE ;
    SmBoolean bDebugMe = FALSE;
    if (bDebugMe) // dump, assertValid, and draw test Brep
      {
        // move pBrep to sDbgBeforeBuff and write to file:[BrepData_BeforeUsdBrep_0.smb]
        sDbgBrepData_FromStartBrep.FromBrep(*pDbgBrep_Start, sAllAttributes) ; 
        sDbgBrep_FromSMRoundTrip.MakeTopologyFromData(&sDbgBrepData_FromStartBrep, sAllAttributes); // make sure topology is up to date
        sDbgBrepData_FromSMRoundTripBrep.FromBrep(sDbgBrep_FromSMRoundTrip, sAllAttributes) ; 

        SM_DUMP_AND_ASSERT_VALID(pDbgBrep_Start);
        SM_DUMP_AND_ASSERT_VALID(&sDbgBrep_FromSMRoundTrip);

        if (usdBrep_CreateOutputFiles())              
          {
            // WriteToFile: StartBrepData <-from- StartBrep
            std::stringstream sFileStreamBefore;
            sFileStreamBefore << "OutputFiles/BrepData_FromStartBrep_" << 0 << ".smb";
            std::string strBefore = sFileStreamBefore.str();
            sDbgBrepData_FromStartBrep.WriteToFile(strBefore, SM_ASCII, TRUE);

            std::string sstringBefore = TfStringPrintf(
                "\n\n  FILE_WRITE: my_test_brep_IO() wrote original test pBrepData_ii:[0] into file:[BrepData_FromStartBrep_0.smb]");
            usdBrep_WriteString(sstringBefore);

            // WriteToFile: SmRoundTripBrepData <-from- SmRoundTripBrep <-from- StratBrepData <-from- StartBrep
            std::stringstream sFileStreamAfter;
            sFileStreamAfter << "OutputFiles/BrepData_FromSmRoundtripdBrep_" << 0 << ".smb";
            std::string strAfter = sFileStreamAfter.str();
            sDbgBrepData_FromSMRoundTripBrep.WriteToFile(strAfter, SM_ASCII, TRUE);

            std::string sstringAfter = TfStringPrintf(
                "\n\n  FILE_WRITE: my_test_brep_IO() wrote SMLib RoundTrip test pBrepData_ii:[0] into file:[BrepData_SmRoundtripBrep_0.smb]");
            usdBrep_WriteString(sstringAfter);
          }
        smgfx_Erase() ;
        smgfx_SetLook(1,2, 0,0,1); if(pDbgBrep_Start) pDbgBrep_Start->Draw(TRUE); sm_GraphicsLoop();
        smgfx_SetLook(3,4, 0,1,0);                    sDbgBrep_FromSMRoundTrip.Draw(TRUE); sm_GraphicsLoop();
        sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

    // AssertValid before export
    SmAssertArray sAssertBefore ;
    SmBoolean bValidBefore = pBrep->AssertValid(&sAssertBefore) ;

    // create a new UsdStage with a new UsdBrepArraySpec loaded with the input array of SmBrep data
    SdfPath sDbgUsdBrepArrayPath;
    UsdStageRefPtr sStage = my_BuildStageAndBrepArrayFromSmBrep(sOutputFilename,        // in : stage name and output file name
                                                          sBrepArrayName,         // in : new UsdBrepArray name
                                                          {pBrep},                // in : Array of SmBrep pointers to add to new UsdBrepArray
                                                          sDbgUsdBrepArrayPath);  // out: path to new "BrepArray" UsdBrepArraySpec
                                                                                  // in : root path to contain the Brep - also contains the Brep's Xform
                                                                                  //      defaults to:["/World"]
    // UsdGeomPrim from BrepArray path in the new stage
    UsdGeomGprim sDbgUsdBrepArray = UsdGeomGprim(sStage->GetPrimAtPath(sDbgUsdBrepArrayPath));

#ifdef SM_DEBUG_CODE
    if (bDebugMe) // move sDbgUsdBrepArray to UsdBrepArrayData formats for debugging
      {
        // move primSpec data to sDbgUsdBrepArrayData_FromStartUsdBrepArray and back again, and dump to output stream
        bool moveResult = BrepReadFromUsdStage(sDbgUsdBrepArray.GetPrim(), sDbgUsdBrepArrayData_FromStartUsdBrepArray);
        if (!moveResult)
        {
            usdBrep_WriteString("Error: Failed to move UsdBrepArray to UsdBrepArrayData in debug dump\n");
        }

        // Dump prim path and PrimProperties
        sstring = TfStringPrintf("\nprim = %s", sDbgUsdBrepArrayPath.GetAsString().c_str());
        usdBrep_WriteString(sstring);

        UsdBrepData::Dump_PrimProperties(sDbgUsdBrepArray.GetPrim());

        // Dump UsdBrepArrayData <-from- StartUsdBrepArray
        Dump_BrepArrayData(sDbgUsdBrepArrayData_FromStartUsdBrepArray, TRUE); // FALSE = dump only Topology obj counts, TRUE = also dump array data values

        // move 1st Brep in BrepArrayData_FromStartUsdBrepArray to SmBrep and SmBrepData
        SMU_BrepConvert::BrepMove_OneUsdBrepToSMLib(sContext, sDbgUsdBrepArrayData_FromStartUsdBrepArray, sDbgAfterOffsets, 0, pDbgBrep_FromBrepArrayData);
        sDbgBrepData_FromBrepArrayData.FromBrep(*pDbgBrep_FromBrepArrayData, sAllAttributes);
        SmObjDelete sClear(pDbgBrep_FromBrepArrayData) ; 

        // write to file: BrepData_From <-from- SmBrep <-from- UsdBrepArrayData <-from- UsdBrepArray
        if (usdBrep_CreateOutputFiles())
          {
            // write out before SmBrepData object
            std::stringstream sFileStream;
            sFileStream << "OutputFiles/BrepData_FromBrepArrayData_FromUsdBrepArrayGprim_" << 0 << ".smb";
            std::string str = sFileStream.str();
            sDbgBrepData_FromBrepArrayData.WriteToFile(str, SM_ASCII, TRUE);

            sstring = TfStringPrintf("\n\n  FILE_WRITE: my_test_brep_IO() wrote USD RoundTrip test pBrepData_ii:[0] into file:[BrepData_FromBrepArrayData_FromUsdBrepArrayGprim_0.smb ");
            usdBrep_WriteString(sstring);
          }

        // gwc todo: add dump() for sDbgBrepObjectData
        smgfx_Erase() ;
        smgfx_SetLook(1,2, 0,0,1); if(pDbgBrep_Start)             pDbgBrep_Start->Draw(TRUE); sm_GraphicsLoop();
        smgfx_SetLook(3,4, 0,1,0);                                sDbgBrep_FromSMRoundTrip.Draw(TRUE); sm_GraphicsLoop();
        smgfx_SetLook(5,6, 1,0,0); if(pDbgBrep_FromBrepArrayData) pDbgBrep_FromBrepArrayData->Draw(TRUE); sm_GraphicsLoop();
        sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

    // convert back to SmBrep

    // Get the usd brep
    if (!sDbgUsdBrepArray.GetPrim().IsValid())
    { SER(SM_ERR); }

    // Convert to SmBreps
    std::vector<SmBrep*> sBrepsFromUsd;
    SmStatus sImportStat = SMU_BrepConvert::BrepMove_UsdToSMLib(sContext, sDbgUsdBrepArray, sBrepsFromUsd, FALSE /*bHealerIsEnabled - disabled pending Fix_CoinVertices investigation*/);

    if(sImportStat != SM_SUCCESS)
      { SE(sImportStat) ; }

    // inform the public of problems
    if(sBrepsFromUsd.size() != 1)
      { SER_MSG(SM_ERR, _T("Failed to create an SmBrep from a sDbgUsdBrepArray")); }

#ifdef SM_DEBUG_CODE
    // Make sure this example has exactly 1 Brep in the BrepArray
    SM_ASSERT(sBrepsFromUsd.size()==1);

    if (bDebugMe)
    {
        // dbg locals
        pDbgBrep_FromBrepArrayData = (sBrepsFromUsd.size() > 0) ? sBrepsFromUsd[0] : NULL ;
        
        // Add pDbgBrep to list for deletion by sClean (via sBreps)
        sBreps.Add(pDbgBrep_FromBrepArrayData);
        
        // Clear sBrepsFromUsd to prevent double-free: ownership transferred to sBreps
        sBrepsFromUsd.clear();

        SM_ASSERT_VALID_AND_DEBUGME_DUMP(pBrep, bDebugMe);
        SM_ASSERT_VALID_AND_DEBUGME_DUMP(pDbgBrep_FromBrepArrayData, bDebugMe);

        smgfx_Erase() ;
        smgfx_SetLook(1,2, 0,0,1); if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop();
        smgfx_SetLook(5,6, 1,0,0); if(pDbgBrep_FromBrepArrayData) pDbgBrep_FromBrepArrayData->Draw(TRUE); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE 

    // AssertValid after re-import
    SmAssertArray sAssertAfter ;
    SmBoolean bValidAfter = (sBrepsFromUsd.size() > 0 && sBrepsFromUsd[0] != NULL)
                            ? sBrepsFromUsd[0]->AssertValid(&sAssertAfter) : FALSE ;

    sstring = TfStringPrintf("  AssertValid: before=%s(%lu) after=%s(%lu)\n",
        bValidBefore ? "PASS" : "FAIL", sAssertBefore.GetSize(),
        bValidAfter  ? "PASS" : "FAIL", sAssertAfter.GetSize()) ;
    usdBrep_WriteString(sstring) ;

    SmBoolean bRoundTripPassed = bValidBefore && bValidAfter;

    // Round-trip validation: compare original brep to re-imported brep
    {
      SmBrepCompareResult sCompare ;
      sCompare.bAllowAnalyticToBSpline = TRUE ;
      sCompare.bAllowVertexReordering  = TRUE ;
      pBrep->CompareBreps(sBrepsFromUsd[0], sCompare) ;

      sstring = TfStringPrintf("  CompareBreps: %s  faces(%lu/%lu) edges(%lu/%lu) verts(%lu/%lu) surfTypeMismatch=%lu crvTypeMismatch=%lu maxVtxGap=%.2e\n",
          sCompare.bPass ? "PASS" : "FAIL",
          sCompare.lFaceCountA, sCompare.lFaceCountB,
          sCompare.lEdgeCountA, sCompare.lEdgeCountB,
          sCompare.lVertexCountA, sCompare.lVertexCountB,
          sCompare.lSurfaceTypeMismatches, sCompare.lCurveTypeMismatches,
          sCompare.dMaxVertexGap) ;
      usdBrep_WriteString(sstring) ;

      bRoundTripPassed = bRoundTripPassed && sCompare.bPass;
      if(!bRoundTripPassed)
        { usdBrep_WriteString("  ** Round-trip FAILED **\n") ; }
    }

    // Clean up the SmBrep objects created from USD conversion
    sm_CleanupSmBrepVector(sBrepsFromUsd);

    return bRoundTripPassed ? SM_SUCCESS : SM_ERR;
} // end my_test_brep_IO

/***********************************************************************
PURPOSE --- Brep SmLib/USD roundtrip for box with attributes 

NOTES --- 
***********************************************************************/
SmStatus my_test_brep_attributes()
{
    SmContext   sContext;
    SmBrep    * pBrep    = new (sContext) SmBrep();
    SmBrep    * pNewBrep = NULL;
    SmStatus    sStatus = SM_SUCCESS;

    // set pBrep = Box
    SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
    sPC.CreateBox(1., 1, 1, SmAxis2Placement());

    std::string sOutputFilename = "../usd_test/OutputFiles/CubeIds.usda";

#ifdef SM_DEBUG_CODE
    bool bDebugMe = FALSE;
#endif // SM_DEBUG_CODE

    // Print Test case info
    std::string sstring;
    sstring = TfStringPrintf(
                "\n\n***************************************************************************************************\n");
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf("Starting my_test_brep_attributes, output file: %s\n", sOutputFilename.c_str());
    usdBrep_WriteString(sstring);

    sstring = TfStringPrintf(
                "****************************************************************************************************\n");
    usdBrep_WriteString(sstring);

    // Assign unique ID attributes to the Brep Topology
    {
        SmTArray<SmRegion*> sRegions;
        SmTArray<SmShell*>  sShells;
        SmTArray<SmFace*>   sFaces;
        SmTArray<SmLoop*>   sLoops, sTempLoops;
        SmTArray<SmEdge*>   sEdges;
        SmTArray<SmVertex*> sVertices;

        ULONG lId = 0;

        pBrep->GetRegions(sRegions);
        pBrep->GetShells(sShells);
        pBrep->GetFaces(sFaces);
        pBrep->GetEdges(sEdges);
        pBrep->GetVertices(sVertices);

        for (ULONG ii = 0; ii < sRegions.GetSize(); ++ii, ++lId)
        { sRegions[ii]->SetUserIndex1(lId); }

        for (ULONG ii = 0; ii < sShells.GetSize(); ++ii, ++lId)
        { sShells[ii]->SetUserIndex1(lId); }

        for (ULONG ii = 0; ii < sFaces.GetSize(); ++ii, ++lId)
        {
            sFaces[ii]->SetUserIndex1(lId);
            sFaces[ii]->GetLoops(sTempLoops);
            sLoops.Append(sTempLoops);
        }

        for (ULONG ii = 0; ii < sLoops.GetSize(); ++ii, ++lId)
        { sLoops[ii]->SetUserIndex1(lId); }
        
        for (ULONG ii = 0; ii < sEdges.GetSize(); ++ii, ++lId)
        { sEdges[ii]->SetUserIndex1(lId); }

        for (ULONG ii = 0; ii < sVertices.GetSize(); ++ii, ++lId)
        { sVertices[ii]->SetUserIndex1(lId); }
    }
    std::string sBrepArrayName = "CubeIds";

    SdfPath sDbgUsdBrepArrayPath;
    UsdStageRefPtr sStage = my_BuildStageAndBrepArrayFromSmBrep(sOutputFilename,           // in : stage name and output file name
                                                          sBrepArrayName,            // in : new UsdBrepArray name
                                                          {pBrep},                   // in : Array of SmBrep pointers to add to new UsdBrepArray
                                                          sDbgUsdBrepArrayPath);     // out: path to new "BrepArray" UsdBrepArraySpec
                                                                                     // in : root path to contain the Brep - also contains the Brep's Xform
                                                                                     //      defaults to:["/World"]

    UsdGeomGprim sDbgUsdBrepArray = UsdGeomGprim(sStage->GetPrimAtPath(sDbgUsdBrepArrayPath));

    // Get the usd brep
    if (!sDbgUsdBrepArray.GetPrim().IsValid())
    { SER(SM_ERR); }

    // Convert sDbgUsdBrepArray to SmBrep
    std::vector<SmBrep*> sBrepsFromUsd;
    SER(SMU_BrepConvert::BrepMove_UsdToSMLib(sContext, sDbgUsdBrepArray, sBrepsFromUsd));

    // Make sure this example has exactly 1
    SM_ASSERT(sBrepsFromUsd.size()==1);
    if (sBrepsFromUsd.size() > 0) pNewBrep = sBrepsFromUsd[0];

    if (pNewBrep)
      {  SM_ASSERT_VALID_AND_DEBUGME_DUMP(pBrep, bDebugMe); 
         smgfx_Erase();
         smgfx_SetLook(1, 2, 0, 0, 1); if (pBrep)    pBrep->Draw(TRUE); sm_GraphicsLoop();
         smgfx_SetLook(5, 6, 1, 0, 0); if (pNewBrep) pNewBrep->Draw(TRUE); sm_GraphicsLoop();
         sm_GraphicsLoop();
      }
    else
      {
        SER_MSG(SM_ERR, _T("Failed to create an SmBrep from a sDbgUsdBrepArray"));
      }

    // check that the imported brep has faces (basic round-trip validation)
    SmTArray<SmFace*> sFaces;
    pNewBrep->GetFaces(sFaces);

    if (sFaces.GetSize() == 0)
    { 
      sStatus = SM_ERR ; 
    }
    
    // Clean up the SmBrep objects created from USD conversion
    sm_CleanupSmBrepVector(sBrepsFromUsd);

    return sStatus;
} // end my_test_brep_attributes

/***********************************************************************
PURPOSE --- SmLib/USD roundtrips for 2 boxes with materials

NOTES --- called by my_test_usd()
          lTest == 0: only case supported
***********************************************************************/
SmStatus my_test_brep_array(ULONG lTest)
{
    
    std::string matFilename = kMaterialsAsset;
    std::string sOutputFilename;

    // SmLib locals - 2 box Breps with different contexts
    SmContext sContext0;
    SmContext             sContext1;
    SmBrep              * pBrep0    = new (sContext0) SmBrep();
    SmBrep              * pBrep1    = new (sContext1) SmBrep();
    SmBrep              * pNewBrep0 = NULL;
    SmBrep              * pNewBrep1 = NULL;
    SmPrimitiveCreation   sPC0(pBrep0->GetInfiniteRegion());  // create a PrimitiveCreation that targets pBrep0->InfiniteRegion
    SmPrimitiveCreation   sPC1(pBrep1->GetInfiniteRegion());  // create a PrimitiveCreation that targets pBrep1->InfiniteRegion

    // build unit box in pBrep0->InfiniteRetion with default (axis aligned origin) position
    sPC0.CreateBox(1., 1, 1, SmAxis2Placement());

    // build unit box in pBrep1->InfiniteRetion with position:[2 0 0]
    sPC1.CreateBox(1., 1, 1, SmAxis2Placement(2, 0, 0,     // origin
                                              1, 0, 0,     // X Axis
                                              0, 1, 0));   // Y Axis

    // Define a usd root prim and material property paths
    std::string sName0 = "Cube_brown";
    std::string sName1 = "Cube_green";
    std::string sMesh0("Mesh_brown");
    std::string sMesh1("Mesh_green");
    SdfPath sRootPath      = SdfPath("/World");
    SdfPath sLooksPath     = sRootPath. AppendPath(SdfPath("Looks"));
    SdfPath sBrownMatPath  = sLooksPath.AppendPath(SdfPath("ABS_Hard_Leather_Brown"));
    SdfPath sGreenMatPath  = sLooksPath.AppendPath(SdfPath("ABS_Hard_Leather_Deep_Green"));

    // usd pathVectors to material properties
    SdfPathVector        sBrownPaths = SdfPathVector{sBrownMatPath};
    SdfPathVector        sGreenPaths = SdfPathVector{sGreenMatPath};

    // Smlib attribute holding a usd SdfPathVector
    SmSdfPathAttribute * pBrownMat  = new(sContext0) SmSdfPathAttribute(SM_AI_MATERIAL_BINDING, SM_AB_REFERENCE, sBrownPaths);
    SmSdfPathAttribute * pGreenMat  = new(sContext1) SmSdfPathAttribute(SM_AI_MATERIAL_BINDING, SM_AB_REFERENCE, sGreenPaths);

    // case 0: Add one material attribute to each SmBrep
    switch (lTest)
    {
    case 0: // Assign material to the brep
    {
          // Brep0 gets brown mat, Brep1 gets green mat
        pBrep0->AddAttribute(pBrownMat);
        pBrep1->AddAttribute(pGreenMat);
        sOutputFilename = "../usd_test/OutputFiles/CubeBrepArray.usda";
        } break ;
    default:
        return SM_ERR_INVALID_INPUT;
    } // end switch on lTest

    // Print Test case info
    std::string sstring;

    usdBrep_WriteString(
        "\n\n***************************************************************************************************\n");

    sstring = TfStringPrintf("Starting my_test_brep_array, output file: %s \n", sOutputFilename.c_str());
    usdBrep_WriteString(sstring);

    usdBrep_WriteString(
        "****************************************************************************************************\n");


    // make stage with a new UsdBrepArray containing pBrep0 and pBrep1
    SdfPath sDbgUsdBrepArrayPath;
    UsdStageRefPtr sStage = my_BuildStageAndBrepArrayFromSmBrep(sOutputFilename,          // in : stage name and output file name
                                                          "brepArray",              // in : new UsdBrepArray name
                                                          {pBrep0, pBrep1 },        // in : Array of SmBrep pointers to add to new UsdBrepArray
                                                          sDbgUsdBrepArrayPath,     // out: path to new "BrepArray" UsdBrepArraySpec
                                                          sRootPath);               // in : root path to contain the Brep - also contains the Brep's Xform
                                                                                    //      defaults to:["/World"]
                                                                               
    UsdGeomGprim sDbgUsdBrepArray = UsdGeomGprim(sStage->GetPrimAtPath(sDbgUsdBrepArrayPath));

    // add 2 usd materials to stage and load with matFilename name 
    UsdShadeMaterial brownMat = UsdShadeMaterial::Define(sStage, sBrownMatPath);
    UsdShadeMaterial greenMat = UsdShadeMaterial::Define(sStage, sGreenMatPath);
    brownMat.GetPrim().GetReferences().AddReference(matFilename, sBrownMatPath);
    greenMat.GetPrim().GetReferences().AddReference(matFilename, sGreenMatPath);

    // check state - valid sDbgUsdBrepArray
    if (!sDbgUsdBrepArray.GetPrim().IsValid())
    { SER(SM_ERR); }

    // Add 2 usd mesh objects to stage - and populate them with the tessellations of pBrep0 and pBrep1
    UsdGeomMesh sMeshPrim0 = UsdGeomMesh::Define(sStage, sRootPath.AppendPath(SdfPath(sMesh0)) ); 
    UsdGeomMesh sMeshPrim1 = UsdGeomMesh::Define(sStage, sRootPath.AppendPath(SdfPath(sMesh1)) ); 
    SdfPrimSpecHandle sMesh0PrimSpecHandle = sStage->GetRootLayer()->GetPrimAtPath(sMeshPrim0.GetPath());
    SdfPrimSpecHandle sMesh1PrimSpecHandle = sStage->GetRootLayer()->GetPrimAtPath(sMeshPrim1.GetPath());
    SmPolyBrep* pPolyBrep0 = NULL;
    SmPolyBrep* pPolyBrep1 = NULL;
    SmObjDelete sCleanPoly0, sCleanPoly1;
    SER(TessellateBrepForMesh(*pBrep0, pPolyBrep0));
    sCleanPoly0.SetObj(pPolyBrep0);
    SER(TessellateBrepForMesh(*pBrep1, pPolyBrep1));
    sCleanPoly1.SetObj(pPolyBrep1);
    SER(SMU_BrepConvert::PopulateMeshAttr(*pPolyBrep0, sMesh0PrimSpecHandle));
    SER(SMU_BrepConvert::PopulateMeshAttr(*pPolyBrep1, sMesh1PrimSpecHandle));

    // export modified stage to its target sOutputFilename
    sStage->Save();

#ifdef SM_DEBUG_CODE
    // debug locals
    UsdBrepData::UsdBrepArrayData                sDbgArrays ; 

// ULONG GWC_CHANGE_NEXT_LINE_TO_FALSE_BEFORE_RELEASE ;
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) // dump, assertValid, and draw test Brep
      {
        // move primSpec data to UsdBrepArrayData format 
        bool moveToArrayResult = BrepReadFromUsdStage(sDbgUsdBrepArray.GetPrim(), sDbgArrays);
        if (!moveToArrayResult)
        {
            usdBrep_WriteString("Error: Failed to move UsdBrepArray to UsdBrepArrayData in debug dump\n");
        }

        SM_DUMP_AND_ASSERT_VALID(pBrep0);
        SM_DUMP_AND_ASSERT_VALID(pBrep1);

        // Dump prim path and DbgArrays
        sstring = TfStringPrintf("\nprim = %s", sDbgUsdBrepArrayPath.GetAsString().c_str());
        usdBrep_WriteString(sstring);

        UsdBrepData::Dump_PrimProperties(sDbgUsdBrepArray.GetPrim());
        Dump_BrepArrayData(sDbgArrays, TRUE); // in : FALSE = dump only Topology obj counts
                                              //      TRUE  = also dump sArray data values

        smgfx_Erase() ;
        smgfx_SetLook(1,2, 0,0,1); if(pBrep0) pBrep0->Draw(TRUE); sm_GraphicsLoop();
        smgfx_SetLook(1,2, 0,1,0); if(pBrep1) pBrep1->Draw(TRUE); sm_GraphicsLoop();
        sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

    // translate locals
    std::vector<SmBrep*> sBrepsFromUsd;

    // move UsdBrepArray brep data to vector of SmBreps
    SER(SMU_BrepConvert::BrepMove_UsdToSMLib(sContext0, sDbgUsdBrepArray, sBrepsFromUsd));

    // check state - 2 Breps were created
    if(sBrepsFromUsd.size() != 2)
    { return SM_ERR; }

    // result locals
    pNewBrep0 = sBrepsFromUsd[0];
    pNewBrep1 = sBrepsFromUsd[1];

    // check result - pBrep0 is valid
    if(pNewBrep0)
      { SM_ASSERT_VALID_AND_DEBUGME_DUMP(pNewBrep0, bDebugMe); }
    else          
      { SER_MSG(SM_ERR, _T("Failed to create an SmBrep0 from a sDbgUsdBrepArray containing 2 Breps")); }

    // check result - pBrep1 is valid
    if(pNewBrep1) 
      { SM_ASSERT_VALID_AND_DEBUGME_DUMP(pNewBrep1, bDebugMe); }
    else          
      { SER_MSG(SM_ERR, _T("Failed to create an SmBrep1 from a sDbgUsdBrepArray containing 2 Breps")); }

    // arrive here when test is working - need to check SM_ASSERT_VALID_AND_DUMP outputs manually
    SmStatus sStat = SM_SUCCESS;

    // clean up
    for (SmBrep* pBrep : sBrepsFromUsd)
    { delete pBrep; }

    // all done
    return sStat;

} // end my_test_brep_array

/***********************************************************************
PURPOSE --- Brep SmLib/USD roundtrip for Brep with materials

NOTES ---  lTest == 0 - Assign material to the brep
           lTest == 1 - Assign materials to the faces
           lTest == 2 - Assign materials to the brep and the faces
          The output file is ../usd_test/OutputFiles/CubeBrepMaterials.usda
***********************************************************************/
SmStatus my_test_brep_materials(ULONG lTest)
{
    std::string matFilename = "../../../TestFiles/usd_TestFiles/Materials.usda";
    std::string sOutputFilename;

    SmContext sContext;
    SmBrep* pBrep = new (sContext) SmBrep();
    SmBrep* pNewBrep = NULL;

    // set pBrep = Box
    SmPrimitiveCreation sPC(pBrep->GetInfiniteRegion());
    sPC.CreateBox(1., 1, 1, SmAxis2Placement());

    // Define a root prim
    std::string sBrepArrayName = "CubeMats";
    std::string sArray("brepArray");
    std::string sMesh("Mesh");
    SdfPath sRootPath      = SdfPath("/World");
    SdfPath sLooksPath     = sRootPath.AppendPath(SdfPath("Looks"));
    SdfPath sBrownMatPath  = sLooksPath.AppendPath(SdfPath("ABS_Hard_Leather_Brown"));
    SdfPath sGreenMatPath  = sLooksPath.AppendPath(SdfPath("ABS_Hard_Leather_Deep_Green"));

    SmTArray<SmTopology*> sTopo;
    SmBoolean bCheckBrep = FALSE;
    SmBoolean bCheckFaces = FALSE;
    SdfPathVector sBrownPaths = SdfPathVector{sBrownMatPath};
    SdfPathVector sGreenPaths = SdfPathVector{sGreenMatPath};

    SmSdfPathAttribute * pBrownMat = new(sContext) SmSdfPathAttribute(SM_AI_MATERIAL_BINDING, SM_AB_REFERENCE, sBrownPaths);
    SmSdfPathAttribute * pGreenMat = new(sContext) SmSdfPathAttribute(SM_AI_MATERIAL_BINDING, SM_AB_REFERENCE, sGreenPaths);

#ifdef SM_DEBUG_CODE
    bool bDebugMe = FALSE;
#endif // SM_DEBUG_CODE

    // Assign material attributes to the Brep Topology
    switch (lTest)
    {
    case 0: // Assign material to the brep
    {
        // Brep gets brown mat
        pBrep->AddAttribute(pBrownMat);
        sOutputFilename = "../usd_test/OutputFiles/CubeBrepMaterials.usda";
        bCheckBrep = TRUE;
        break;
    }
    case 1: // Assign materials to brepFaces
    {
        // Faces get half brown half green mat
        sOutputFilename = "../usd_test/OutputFiles/CubeFaceMaterials.usda";
        bCheckFaces = TRUE;
        SmTArray<SmFace*> sFaces;
        pBrep->GetFaces(sFaces);
        for (ULONG ii = 0; ii < sFaces.GetSize(); ++ii)
        {
            SmAttribute * pAtt = ii < 3 ? pBrownMat : pGreenMat;
            sFaces[ii]->AddAttribute(pAtt);
        }
        break;
    }
    case 2: // Assign materials to the brep and brepFaces
    {
        sOutputFilename = "../usd_test/OutputFiles/CubeAllMaterials.usda";

        // Brep gets brown mat
        pBrep->AddAttribute(pBrownMat);

        bCheckBrep = TRUE;
        bCheckFaces = TRUE;
        // Faces get half brown half green mat
        SmTArray<SmFace*> sFaces;
        pBrep->GetFaces(sFaces);
        for (ULONG ii = 0; ii < sFaces.GetSize(); ++ii)
        {
            SmAttribute * pAtt = ii < 3 ? pBrownMat : pGreenMat;
            sFaces[ii]->AddAttribute(pAtt);
        }
        break;
    }
    default:
        return SM_ERR_INVALID_INPUT;
    }

    // Print Test case info
    std::string sstring;

    usdBrep_WriteString("\n\n***************************************************************************************************\n");
    sstring = TfStringPrintf("Starting my_test_brep_materials, test %lu, %s \n", lTest, sOutputFilename.c_str());
    usdBrep_WriteString(sstring);
    usdBrep_WriteString("***************************************************************************************************\n");

    SdfPath sDbgUsdBrepArrayPath;
    UsdStageRefPtr sStage = my_BuildStageAndBrepArrayFromSmBrep(sOutputFilename,     // in : stage name and output file name
                                                         sBrepArrayName,       // in : new UsdBrepArray name
                                                         {pBrep},              // in : Array of SmBrep pointers to add to new UsdBrepArray
                                                         sDbgUsdBrepArrayPath, // out: path to new "BrepArray" UsdBrepArraySpec
                                                         sRootPath);           // in : root path to contain the Brep - also contains the Brep's Xform
                                                                               //      defaults to:["/World"]
                                                              
    UsdGeomGprim sDbgUsdBrepArray = UsdGeomGprim(sStage->GetPrimAtPath(sDbgUsdBrepArrayPath));

    UsdShadeMaterial brownMat = UsdShadeMaterial::Define(sStage, sBrownMatPath);
    UsdShadeMaterial greenMat = UsdShadeMaterial::Define(sStage, sGreenMatPath);
    brownMat.GetPrim().GetReferences().AddReference(matFilename, sBrownMatPath);
    greenMat.GetPrim().GetReferences().AddReference(matFilename, sGreenMatPath);

    UsdGeomMesh meshPrim = UsdGeomMesh::Define(sStage, sRootPath.AppendPath(SdfPath(sMesh)) ); 
    SdfPrimSpecHandle sMeshPrimSpecHandle = sStage->GetRootLayer()->GetPrimAtPath(meshPrim.GetPath());
    SmPolyBrep* pPolyBrep = NULL;
    SmObjDelete sCleanPoly;
    SER(TessellateBrepForMesh(*pBrep, pPolyBrep));
    sCleanPoly.SetObj(pPolyBrep);
    SER(SMU_BrepConvert::PopulateMeshAttr(*pPolyBrep, sMeshPrimSpecHandle));

    sStage->Save();

    // Get the usd brep
    if (!sDbgUsdBrepArray.GetPrim().IsValid())
    { SER(SM_ERR); }

    // Convert the Usd Brep to SmBreps
    std::vector<SmBrep*> sBrepsFromUsd;
    SER(SMU_BrepConvert::BrepMove_UsdToSMLib(sContext, sDbgUsdBrepArray, sBrepsFromUsd));

    // Make sure this example has only 1
    SM_ASSERT(sBrepsFromUsd.size() == 1);
    if (sBrepsFromUsd.size() > 0) pNewBrep = sBrepsFromUsd[0];

    if (pNewBrep)
      { SM_ASSERT_VALID_AND_DEBUGME_DUMP(pBrep, bDebugMe); }
    else
      { SER_MSG(SM_ERR, _T("Failed to create an SmBrep from a sDbgUsdBrepArray")); }

    SmStatus sStat = SM_SUCCESS;

    // check that this imported brep has attributes
    if (bCheckBrep)
    { sTopo.Add(pNewBrep); }

    if (bCheckFaces)
    {
        SmTArray<SmFace*> sFaces;
        pNewBrep->GetFaces(sFaces);
        for (ULONG ii = 0; ii < sFaces.GetSize(); ++ii)
        {
            sTopo.Add(sFaces[ii]);
        }
    }

    if (sTopo.GetSize() == 0)
    { return SM_ERR; }

    SmTArray<SmAttribute*> sAttributes;
    for (ULONG ii = 0; ii < sTopo.GetSize(); ++ii)
    {
        if ( !sTopo[ii]->FindAttribute(SM_AI_MATERIAL_BINDING) )
        { sStat = SM_ERR; }
    }

    // Clean up the SmBrep objects created from USD conversion
    sm_CleanupSmBrepVector(sBrepsFromUsd);

    return sStat;
} // end my_test_brep_materials

/***********************************************************************
PURPOSE --- PolyBrep SmLib/USD roundtrip for test case = mouse 

NOTES --- 1. Read mouse.smb file
          2. tessellate mouse SmBrep and convert to UsdMesh with SMU_BrepConvert::PopulateMeshAttr()
          3. write stage to file
          4. Read back UsdMesh
          5. Convert UsdMesh to SmPolyBrep and AssertValid and Dump.
***********************************************************************/
SmStatus my_test_polybrep_IO()
{
    std::string sOutputFilename = "../usd_test/OutputFiles/MouseMesh.usda";

    std::string sstring;

    usdBrep_WriteString("\n\n***************************************************************************************************\n");
    sstring = TfStringPrintf("Starting my_test_polybrep_IO, output file %s\n", sOutputFilename.c_str());
    usdBrep_WriteString(sstring);
    usdBrep_WriteString("***************************************************************************************************\n");

    SmContext    sContext;
    SmBrep     * pBrep = new (sContext) SmBrep();
    SmPolyBrep * pNewPolyBrep = NULL;
    SmObjDelete  sClean0(pBrep); // Delete pBrep at end of scope
    SmObjDelete  sClean1;

    // set pBrep = mouse
    pBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Mouse.smb"));

    // Set up usd stage
    auto sStage = UsdStage::CreateNew(sOutputFilename);

    // Paths for the rooth and the mesh
    std::string sBrepArrayName     = "Mouse";
    SdfPath     sRootPath = SdfPath("/World");
    SdfPath     sMeshPath = sRootPath.AppendPath(SdfPath(sBrepArrayName));

    // Define a root prim
    UsdGeomXform      sRootXform = UsdGeomXform::Define(sStage, sRootPath);
    SdfPrimSpecHandle sRootXformHandle = sStage->GetRootLayer()->GetPrimAtPath(sRootPath);
    SdfPrimSpecHandle sMeshPrimSpecHandle = SdfPrimSpec::New(sRootXformHandle, sBrepArrayName, SdfSpecifierDef, "Mesh");

    // tessellate mouse and save as UsdMesh
    SmPolyBrep* pPolyBrep = NULL;
    SmObjDelete sCleanPoly;
    SER(TessellateBrepForMesh(*pBrep, pPolyBrep));
    sCleanPoly.SetObj(pPolyBrep);
    SER(SMU_BrepConvert::PopulateMeshAttr(*pPolyBrep, sMeshPrimSpecHandle));

    // write stage to file
    sStage->Save();

    // Get the usd mesh
    UsdGeomMesh sUsdGeomMesh = UsdGeomMesh::Get(sStage, sMeshPath);
    if (!sUsdGeomMesh.GetPrim().IsValid())
    { SER(SM_ERR); }

    // convert back to SmPolyBrep
    SER(SMU_BrepConvert::CreateSmPolyBrep_FromUsdMesh(sContext, sUsdGeomMesh, pNewPolyBrep));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    SM_ASSERT_VALID_AND_DEBUGME_DUMP(pNewPolyBrep, bDebugMe);
#endif // SM_DEBUG_CODE

    // Delete pNewPolyBrep at end of scope
    sClean1.SetObj(pNewPolyBrep);

#ifdef SM_DEBUG_CODE
    if(bDebugMe)
      { 
        SM_ASSERT_VALID_AND_DEBUGME_DUMP(pBrep, bDebugMe);
        SM_ASSERT_VALID_AND_DEBUGME_DUMP(pNewPolyBrep, bDebugMe);

        pNewPolyBrep->Draw(true); 
      }
#endif // SM_DEBUG_CODE

    return SM_SUCCESS;
} // end my_test_polybrep_IO
