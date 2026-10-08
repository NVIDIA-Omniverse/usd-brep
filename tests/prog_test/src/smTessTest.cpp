// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*___*/
/**********************************************************************
* FILE NAME --- smtess_test.cpp
* PURPOSE ---  Test tessellation functions.
*
**********************************************************************/
/*___*/

#include "StdAfx.h"

#include <smTessTest.h>

#include <SmGrid.h>
#include <SmPoly.h>
#include <SmBrepData.h>
#include <SmRayTracer.h>
#include <SmPolySolver.h>
#include <SmPolyMerge.h>
#include <SmPolyDecimate.h>
#include <SmPrimitiveCreation.h>
#include <SmTess.h>
#include <SmGraphicsOutput.h>



static ULONG      s_lNumberTriangles = 0;



/*****************************************************
PURPOSE ---

USAGE NOTES ---
*****************************************************/
static SmStatus my_trimmed_surface_stitch
 (SmBrep * pBrep,               // in : 
  double   dStitchTol3d,  // in : 
  ULONG  & rlNumLaminaEdges)    // out: 
  {
  SmTemporaryChangeValue<SmBoolean> sChange(pBrep->m_bEditingEnabled, TRUE);
  SmStitchCallback sCallBack;
  SmStitch sStitch(sCallBack,dStitchTol3d);
    
  sStitch.m_bDoRegionNesting   = FALSE;
  sStitch.m_bFastEdgeCompare   = TRUE;
  sStitch.m_bIgnoreProblems    = TRUE;
  sStitch.m_bValidateResult    = FALSE;
  sStitch.m_bSqueezeSmallEdges = TRUE;
    
  ULONG lStitchedEdges;
  double dMaxVertexGap, dMaxEdgeGap;
  SER(sStitch.DoStitching(pBrep,            // in : target Brep
                    NULL,             // in : only glue coincident vertices on this list, 
                                      //      NULL = do all vertices
                    NULL,             // in : only glue coincident edge pairs on this list, 
                                      //      NULL = do all edges
                    lStitchedEdges,       // out: number of edges stitched
                    rlNumLaminaEdges, // out: number of lamina edges remaining after stitch
                    dMaxVertexGap,    // out: max gap found between coincident vertices considered for gluing
                    dMaxEdgeGap));    // out: max gap found between coincident edges    considered for gluing
                                      //      NOTE: some coincident vertex and edge pairs do not get glued due to
                                      //            a. gaps exceeding tolerances
                                      //            b. geometries not listed within optional candidate lists
                                      //            c. a failure within the glue edge function
                                      //            d. not being lamina when m_bMakingManifoldSolid == TRUE
                                      // out: min gap found between vertices that did not get glued
                                      //      default:[NULL], NULL to ignore.
                                      // out: min gap found between edges that did not get glued
                                      //      default:[NULL], NULL to ignore.
  return SM_SUCCESS;

  } // end my_trimmed_surface_stitch

/*****************************************************
PURPOSE ---

USAGE NOTES ---
*****************************************************/
PT_EXPORT SmStatus my_test_one_tess
 (SmBrep    * pBrep,            // in : target brep
  double      dCHTol,           // in : chord height tolerance
  double      dAngleTolDeg,     // in : angle tolerance
  double      dMax3DEdge,       // in : max edge length.
                                //      use a negative dMax3D edge to auto compute based on brep->box size
  double      dMaxAspect,       // in : max aspect ratio
  SmBoolean & rbFailedFaces)    // out: TRUE = encountered problem faces
{
   MYPRINTF(_T("\n   Entered: my_test_one_tess")) ;

   // when asked - pick a max 3DEdge size from model size
   if (dMax3DEdge < 0.0) 
     {
        SmExtent3d brepBBox;
        pBrep->CalculateBoundingBox(brepBBox);
        double size = brepBBox.GetSize().Length();
        if (size >= 1.0e15 || size <= 0.0) size = 1.0;
        dMax3DEdge = -0.025 * size * dMax3DEdge;
     }

  // build curve and surface tessellation drives
  SmCurveTessDriver sCrvTess(0, dCHTol, dAngleTolDeg);
  SmSurfaceTessDriver sSrfTess(dCHTol,
                               dAngleTolDeg,
                               dMax3DEdge,
                               0.0,
                               0.001,
                               dMaxAspect);

  // build the SmTess driver (it has no target brep yet)
  SmTess sTess(*pBrep->GetContext(),sCrvTess,sSrfTess);

  // copy the brep since brep edges may be split during tessellation
  SmBrep *pCopy = new (*pBrep->GetContext()) SmBrep(*pBrep);

  // do the tessellation - 
  //  o. the pCopy Brep is stored in m_pTessBrep
  //  o. each m_pTessBrep edge may be split as needed for compatible meshes
  //  o. each m_pTessBrepface is tessellated.  
  //      a SmSrfTessCache is built and stored in m_vCache which contains
  //         SmSrfTessCache::mTS_pPolyBrep which is a UVDomain tessellation of the
  //         Face->Surface.
  //  o. The polygons can be fetched through sTess.OutputPolygons(sOutputPolyBrep) ;
  SER(sTess.DoTessellation(pCopy,rbFailedFaces));
  SmBoolean bFailure = true;
  SmPolygonSLAOutput sSLAOutput(_T("../prog_test/OutputFiles/SLAPolygons.stl"),SM_ASCII,1,bFailure);
  if (!bFailure) 
     SER(sTess.OutputPolygons(sSLAOutput));

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
    {
      if (!bFailure) 
        {
          // use OutputPolygons and the SmPolygonOutputCallback mechanism to
          //  build m_p3DPolyBrep from all the tessellation polygons -
          //   these poygons will not be stitched.
          SmPolygonOutputCallback sOutputPolyBrep ;
          sOutputPolyBrep.SetOutputType(SM_PO_CREATE_POLYBREP) ;
          sTess.OutputPolygons(sOutputPolyBrep) ;

          // fetch the newly constructed 3d polyBrep
          SmPolyBrep *pPolyBrep = sTess.Get3DPolyBrep() ;

          // draw and dump
          SM_ASSERT_VALID(pBrep) ;
          SM_ASSERT_VALID(pCopy) ;
          SM_ASSERT_VALID(pPolyBrep) ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(2,3, 0,1,1) ; if(pCopy) pCopy->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,0) ; if(pPolyBrep) pPolyBrep->Draw(TRUE) ; sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
    }
#endif // SM_GFX_CODE

  // cleanup created file
  SM_REMOVE( _T("../prog_test/OutputFiles/SLAPolygons.stl") );

  // all done
  return SM_SUCCESS;

} // end my_test_one_tess

/*****************************************************
PURPOSE --- This class demonstrates how to override a method in    
            SmPolygonOutputCallback to get polygons to do something
            other than graphics.  It just prints them out.         
USAGE NOTES ---
*****************************************************/
class SmPolygonOutputPrinter : public SmPolygonOutputCallback
{
public:
    SmPolygonOutputPrinter() {}
    virtual ~SmPolygonOutputPrinter() {}

    SmStatus OutputPolygon(ULONG           lPolygonType,        // in : 
                           ULONG           lNumPoints,          // in : 
                           SmPoint3d     * sPoints,             // in : 
                           SmVector3d    * sNormals,            // in : 
                           SmPoint2d     * sUVPoints,           // in : 
                           SmSurface     * pSurface,            // NotUsed: in : 
                           SmFace        * pFace,               // NotUsed: in : 
                           SmGfxArraySet * pOptGfxSet = NULL ); // NotUsed: in : 

} ;  // end class SmPolygonOutputPrinter

/*****************************************************
PURPOSE --- Here is the example method showing how to print polygons.

USAGE NOTES ---
*****************************************************/
SmStatus SmPolygonOutputPrinter::OutputPolygon
 (ULONG           lPolygonType,   // in :
  ULONG           lNumPoints,     // in :
  SmPoint3d     * sPoints,        // in :
  SmVector3d    * sNormals,       // in :
  SmPoint2d     * sUVPoints,      // in :
  SmSurface     * pSurface,       // NotUsed: in :
  SmFace        * pFace,          // NotUsed: in :
  SmGfxArraySet * pOptGfxSet)     // NotUsed: in :
{
  SM_REF3(pSurface, pFace, pOptGfxSet) ; 
    s_lNumberTriangles ++;
    TCHAR sBuff[SM_TBLOCK_SIZE];
    SM_SPRINTF(sBuff,_T("Polygon[%ld] - Type %ld **************************************\n"),s_lNumberTriangles,lPolygonType);
    smos_WriteBuffer(sBuff);
    for (ULONG i=0; i<lNumPoints; i++) 
      {
        smos_WriteBuffer(_T("Point = "));
        sPoints[i].Dump(); 
        smos_WriteBuffer(_T("\nNormal = "));
        sNormals[i].Dump();
        smos_WriteBuffer(_T("\nUV Point = "));
        sUVPoints[i].Dump();
        smos_WriteBuffer(_T("\n"));
      }
    return SM_SUCCESS;

} // end SmPolygonOutputPrinter::OutputPolygon

/*****************************************************
PURPOSE --- read brep files and output a summary
     of the read Brep into the output stream.

USAGE NOTES ---
  Reads brep file with SmBrep::ReadFromFile()

*****************************************************/
SmStatus my_read_brep
  (const SmContext & crContext,            // in : context for new object construction
   const TCHAR     * pFileName,            // in : target file name
   SmBrep         *& rpNewBrep,            // out: Brep constructed by file read
   SmBoolean         bRebuildUVTrimCurves) // in : TRUE = call SmBrep::RebuildUVTrimCurve(), FALSE = Don't
{
  if (   smos_WStrStr(pFileName,_T(".smb"))
      || smos_WStrStr(pFileName,_T(".SMB"))   
      || smos_WStrStr(pFileName,_T(".brp"))   
      || smos_WStrStr(pFileName,_T(".brep"))) 
    { // SM Brep - ASCII
      rpNewBrep = new(crContext) SmBrep();
      SmObjDelete sObj(rpNewBrep);
      SER(rpNewBrep->ReadFromFile(crContext,pFileName,SM_ASCII,bRebuildUVTrimCurves));
      sObj.Clear();
      smos_WriteBuffer(_T("\n\n**********************************************************************\n"));
      TCHAR sBuff[SM_TBLOCK_SIZE];
      SM_SPRINTF(sBuff,_T("BREP File - %s\n"),pFileName);
      smos_WriteBuffer(sBuff);
    } // end read brep file

  // inform the public
  SM_ASSERT_VALID(rpNewBrep) ;

#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
    {
      smgfx_SetLook(1,2); rpNewBrep->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif

  return SM_SUCCESS;

} // end my_read_brep

/*****************************************************
PURPOSE ---  Read in .smb and convert it to polybrep

USAGE NOTES ---
*****************************************************/
SmStatus my_brepfile_to_polybrep
 (const SmContext & crContext,
  const TCHAR     * pBrepFileName,
  double            dCHTol,
  double            dCrvTessAngle,
  double            dSrfTessAngle,
  SmPolyBrep     *& rpPolyBrep)
{
  SmBrep *pBrep = NULL;
  SER(my_read_brep(crContext,pBrepFileName,pBrep,FALSE));
  NER(pBrep);

  // test attribute propagation
  ULONG ii ;
  SmTArray<SmFace*> sFaces ; pBrep->GetFaces(sFaces) ;
  SmTArray<SmEdge*> sEdges ; pBrep->GetEdges(sEdges) ;
  SmTArray<SmVertex*> sVertices ; pBrep->GetVertices(sVertices) ;

  // place attributes on the model
  for(ii=0;ii<sFaces.GetSize();ii++)
    {
      SmFace * pFace = sFaces[ii] ;
      SmLongAttribute * pLongAttribute = new (crContext) SmLongAttribute(SM_AI_FACE_ID, // Attribute ID - unique per object
                                                                         ii,            // Attribute value 
                                                                         SM_AB_COPY) ;  // one user per attribute 
      pFace->AddAttribute(pLongAttribute) ;
    } // end iter faces

  // place attributes on the model
  for(ii=0;ii<sEdges.GetSize();ii++)
    {
      SmEdge * pEdge = sEdges[ii] ;
      SmLongAttribute * pLongAttribute = new (crContext) SmLongAttribute(SM_AI_EDGE_ID, // Attribute ID - unique per object
                                                                         ii,            // Attribute value 
                                                                         SM_AB_COPY) ;  // one user per attribute 
      pEdge->AddAttribute(pLongAttribute) ;
    } // end iter Edges

  // place attributes on the model
  for(ii=0;ii<sVertices.GetSize();ii++)
    {
      SmVertex * pVertex = sVertices[ii] ;
      SmLongAttribute * pLongAttribute = new (crContext) SmLongAttribute(SM_AI_VERTEX_ID, // Attribute ID - unique per object
                                                                         ii,            // Attribute value 
                                                                         SM_AB_COPY) ;  // one user per attribute 
      pVertex->AddAttribute(pLongAttribute) ;
    } // end iter Vertices

  // Convert brep to polybrep
  double    dMax3DEdge = 0.0;
  double    dMaxAspect = 0.0;
  SmBoolean bFailedFaces;
  ULONG     lNumLamina;
  SER( pBrep->ConvertToPolyBrep
  (
       rpPolyBrep,
       bFailedFaces,
       lNumLamina,
       dCHTol,
       dCrvTessAngle,
       dSrfTessAngle,
       dMax3DEdge,
       dMaxAspect,
       FALSE,
       FALSE,
       FALSE
  )) ; 

  if (bFailedFaces || lNumLamina != 0) SER(SM_ERR);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
   if(bDebugMe)
     {
       SM_PTR_ARRAY(sPolyFaces,    SmPolyFace  , 512); // SmTArray<SmPolyFace  *>  
       SM_PTR_ARRAY(sPolyEdges,    SmPolyEdge  , 512); // SmTArray<SmPolyEdge  *>  
       SM_PTR_ARRAY(sPolyVertices, SmPolyVertex, 512); // SmTArray<SmPolyVertex  *>
    
       rpPolyBrep->GetPolyFaces(sPolyFaces);
       rpPolyBrep->GetPolyEdges(sPolyEdges);
       rpPolyBrep->GetPolyVertices(sPolyVertices);

       ULONG lFaceFrequence   = sPolyFaces.GetSize() / 9 ;
       // ULONG lEdgeFrequence   = sPolyEdges.GetSize() / 9 ;
       // ULONG lVertexFrequence = sPolyVertices.GetSize() / 9 ;

       // pretty print some of the objects and their attributes so that we can see attribute propagation working
       rpPolyBrep->Dump() ;
       rpPolyBrep->DumpPolyFaces   (lFaceFrequence, TRUE) ; // TRUE = only dump objs with attributes
       rpPolyBrep->DumpPolyEdges   (1, TRUE) ;              // TRUE = only dump objs with attributes
       rpPolyBrep->DumpPolyVertices(1, TRUE) ;              // TRUE = only dump objs with attributes
     }
#endif // SM_DEBUG_CODE

  // delete Brep 
  SM_ASSERT(pBrep != NULL) ; delete pBrep ; pBrep = NULL ;

#ifdef SM_GFX_CODE
  if (smGet_DoGraphics())
    {
      smgfx_SetLook(1,2, 0,0,0); if(rpPolyBrep) rpPolyBrep->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE

  // all done
  return SM_SUCCESS;

} // end my_brepfile_to_polybrep

/*****************************************************
PURPOSE ---

USAGE NOTES ---
  my_test_poly_solver_1() and my_test_poly_solver_2()
  used to be a single routine, but some compilers
  (64-bit on Vista) have a problem with that, so it was broken up
  into two, _1 and _2.
*****************************************************/
PT_EXPORT SmStatus my_test_poly_solver_1()
{
  MYPRINTF(_T("\nEntered in my_test_poly_solver_1"));

  if (TRUE)
  { // Test polybrep-polybrep SM_SO_3D_SIGNED_DIRECTED_MINIMIZE solver
      MYPRINTF(_T("\n my_test_poly_solver_1: test 1"));
      SmContext sContext;
      SmPolyBrep* pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"));
      pPolyBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"), SM_ASCII);
      SmObjDelete sClean1(pPolyBrep);

      double d3dTolerance = SM_EFF_ZERO;
      SmPoint3d sOrigin(5, 5, 5);
      SmVector3d sXAxis(1, 0, 0);
      SmVector3d sYAxis(0, 1, 0);
      SmAxis2Placement sRecPlacement;
      sRecPlacement.SetCanonical(sOrigin, sXAxis, sYAxis);
      sRecPlacement.RotateAboutAxis(0.25, SmVector3d(0, 0, 1));
      SmExtent2d sRecDomain(SmPoint2d(0, 0), SmPoint2d(8, 8));

#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
      {
          if (pPolyBrep)
          {
                  SM_ASSERT_VALID(pPolyBrep);
          }

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(2, 3, 1, 0, 0);
          sXAxis.Draw(&sOrigin);
          sm_GraphicsLoop();
          smgfx_SetLook(2, 3, 0, 1, 0);
          sYAxis.Draw(&sOrigin);
          sm_GraphicsLoop();
          smgfx_SetLook(2, 3, 1, 0, 1);
          sRecPlacement.Draw(&sRecDomain);
          sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // SM_GFX_CODE

      SmSolution sSData[256];
      SmSolutionArray sSolutions(256, sSData);
      // ask for any one vertex/vertex, vertex/edge, vertex/face, edge/edge, or edge/face
      // point intersection between the rectangle and the PolyBrep
      SER(SmPolySolver::PolyBrepRectangleIntersect(pPolyBrep, sRecPlacement, sRecDomain, d3dTolerance, sSolutions));
//        SM_ASSERT(sSolutions.GetSize() > 0);
#ifdef SM_GFX_CODE
      if (pPolyBrep)
      {
          SM_ASSERT_VALID(pPolyBrep);
      }
      if (smGet_DoGraphics())
      {
          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(2, 3, 1, 0, 0);
          sXAxis.Draw(&sOrigin);
          sm_GraphicsLoop();
          smgfx_SetLook(2, 3, 0, 1, 0);
          sYAxis.Draw(&sOrigin);
          sm_GraphicsLoop();
          smgfx_SetLook(2, 3, 1, 0, 1);
          sRecPlacement.Draw(&sRecDomain);
          sm_GraphicsLoop();
          smgfx_SetLook(5, 6, 0, 1, 1);
          sSolutions.Draw();
          sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // SM_GFX_CODE
  }

  if (TRUE)
  { // Test polybrep-polybrep SM_SO_3D_SIGNED_DIRECTED_MINIMIZE solver
      MYPRINTF(_T("\n my_test_poly_solver_1: test 2"));
      SmContext sContext;
      SmPolyBrep* pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"));
      pPolyBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"), SM_ASCII);
      SmObjDelete sClean1(pPolyBrep);
      SmPolyBrep* pPolyBrep2 = NULL;
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"));
      SER(SmPolyBrep::ReadFromSTLFile(
          sContext, _T("../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"), pPolyBrep2, SM_ASCII));
      SmObjDelete sClean2(pPolyBrep2);

      SmVector3d sProjDirVec[3];
      if (0)
      {
          sProjDirVec[0].Set(1, -0.3, 0);
      }
      else if (0)
      {
          sProjDirVec[0].Set(1, 0.3, 0); // objects will have positive distances between them
      } // result will be smallest absolute distances in
      else // the given direction.
      {
          sProjDirVec[0].Set(-1.0, -0.3, 0); // objects will have negative distances between them
      } // result will be largest absolute distances in
      sProjDirVec[0].Unitize(); // direction oposite to the given direction.

#ifdef SM_GFX_CODE
      if (pPolyBrep)
      {
          SM_ASSERT_VALID(pPolyBrep);
      }
      if (pPolyBrep2)
      {
          SM_ASSERT_VALID(pPolyBrep2);
      }

      if (smGet_DoGraphics())
      {
          SmPoint3d sPoint(15, 15, 0);
          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
                  pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
                  pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sProjDirVec[0].Draw(&sPoint);
          sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // SM_GFX_CODE

      SmSolution sSData[256];
      SmSolutionArray sSolutions(256, sSData);
      double dBestAnswer = SM_BIG_DOUBLE;

      // look for miniminum distance between PolyObjects
      SER(SmPolySolver::PolyBrepPolyBrepSolve(pPolyBrep, pPolyBrep2, SM_SO_3D_SIGNED_DIRECTED_MINIMIZE, SM_SR_ALL,
                                              1.0e-8, dBestAnswer, sProjDirVec, sSolutions));
      SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
      {
          sSolutions.Dump();

          SmPoint3d sPoint(15, 15, 0);

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
                  pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
                  pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sProjDirVec[0].Draw(&sPoint);
          sm_GraphicsLoop();
          for (ULONG k = 0; k < sSolutions.GetSize(); k++)
          {
                  SmPoint3d sPnt1, sPnt2;

                  SmSolution& rSol = sSolutions[k];
                  sPnt1.x = rSol.m_vStart[0];
                  sPnt1.y = rSol.m_vStart[1];
                  sPnt1.z = rSol.m_vStart[2];
                  sPnt2.x = rSol.m_vStart[3];
                  sPnt2.y = rSol.m_vStart[4];
                  sPnt2.z = rSol.m_vStart[5];
                  SmVector3d sDir = sPnt2 - sPnt1;
                  smgfx_SetLook(1, 2, 1, 0, 0);
                  sDir.Draw(&sPnt1);
                  sm_GraphicsLoop();
                  smgfx_SetLook(4, 5, 0, 0, 1);
                  sPnt2.Draw();
                  sm_GraphicsLoop();
          }
          sm_GraphicsLoop();
      }
#endif // SM_GFX_CODE
  }

  if (TRUE)
  { // Test polybrep-polybrep SM_SO_SIGNED_DIRECTED_MINIMIZE solver
      MYPRINTF(_T("\n my_test_poly_solver_1: test 3"));
      SmContext sContext;
      SmPolyBrep* pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"));
      pPolyBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"), SM_ASCII);
      SmObjDelete sClean1(pPolyBrep);
      SmPolyBrep* pPolyBrep2 = NULL;
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"));
      SER(SmPolyBrep::ReadFromSTLFile(
          sContext, _T("../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"), pPolyBrep2, SM_ASCII));
      SmObjDelete sClean2(pPolyBrep2);

      SmVector3d sProjDirVec[3];
      if (0)
      {
          sProjDirVec[0].Set(0, 1, 0); // Top View
          sProjDirVec[1].Set(1, 0, 0); // Top View
      }
      else if (0)
      {
          sProjDirVec[0].Set(0, 0, 1); // Front View
          sProjDirVec[1].Set(1, 0.3, 0); // Front View
          sProjDirVec[1].Unitize();
      }
      else if (0)
      {
          sProjDirVec[0].Set(0, 1, 0); // Top View
          sProjDirVec[1].Set(-1, 0, 0); // Top View
      }
      else
      {
          sProjDirVec[0].Set(0, 0, 1); // Front View
          sProjDirVec[1].Set(-1, -0.3, 0); // Front View
          sProjDirVec[1].Unitize();
      }

#ifdef SM_GFX_CODE
      if (pPolyBrep)
      {
          SM_ASSERT_VALID(pPolyBrep);
      }
      if (pPolyBrep2)
      {
          SM_ASSERT_VALID(pPolyBrep2);
      }
      if (smGet_DoGraphics())
      {
          SmPoint3d sPoint(15, 15, 0);

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
                  pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
                  pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sProjDirVec[0].Draw(&sPoint);
          sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // SM_GFX_CODE

      SmSolution sSData[256];
      SmSolutionArray sSolutions(256, sSData);
      double dBestAnswer = SM_BIG_DOUBLE;
      SER(SmPolySolver::PolyBrepPolyBrepSolve(pPolyBrep, pPolyBrep2, SM_SO_SIGNED_DIRECTED_MINIMIZE, SM_SR_ALL, 1.0e-8,
                                              dBestAnswer, sProjDirVec, sSolutions));
      SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
      {
          sSolutions.Dump();

          SmPoint3d sPoint(15, 15, 0);

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
                  pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
                  pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sProjDirVec[0].Draw(&sPoint);
          sm_GraphicsLoop();
          for (ULONG k = 0; k < sSolutions.GetSize(); k++)
          {
                  SmSolution& rSol = sSolutions[k];
                  SmPoint3d sPnt1, sPnt2;
                  sPnt1.x = rSol.m_vStart[0];
                  sPnt1.y = rSol.m_vStart[1];
                  sPnt1.z = rSol.m_vStart[2];
                  sPnt2.x = rSol.m_vStart[3];
                  sPnt2.y = rSol.m_vStart[4];
                  sPnt2.z = rSol.m_vStart[5];
                  SmVector3d sDir = sPnt2 - sPnt1;
                  smgfx_SetLook(1, 2, 1, 0, 0);
                  sDir.Draw(&sPnt1);
                  sm_GraphicsLoop();
                  smgfx_SetLook(4, 5, 0, 0, 1);
                  sPnt2.Draw();
                  sm_GraphicsLoop();
          }
          sm_GraphicsLoop();
      }
#endif // SM_GFX_CODE
  }

  if (TRUE)
  { // Test polybrep-polybrep SM_SO_DIRECTED_MAXIMIZE solver
      MYPRINTF(_T("\n my_test_poly_solver_1: test 4"));
      SmContext sContext;
      SmPolyBrep* pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"));
      pPolyBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"), SM_ASCII);
      SmObjDelete sClean1(pPolyBrep);
      SmPolyBrep* pPolyBrep2 = NULL;
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"));
      SER(SmPolyBrep::ReadFromSTLFile(
          sContext, _T("../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"), pPolyBrep2, SM_ASCII));
      SmObjDelete sClean2(pPolyBrep2);

      SmVector3d sProjDirVec[3];
      if (0)
      {
          sProjDirVec[0].Set(0, 1, 0); // Top View
          sProjDirVec[1].Set(-1, 0, 0); // Top View
      }
      else
      {
          sProjDirVec[0].Set(0, 0, 1); // Front View
          sProjDirVec[1].Set(-1, -0.3, 0); // Front View
          sProjDirVec[1].Unitize();
      }

#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
      {
          if (pPolyBrep)
          {
                  SM_ASSERT_VALID(pPolyBrep);
          }
          if (pPolyBrep2)
          {
                  SM_ASSERT_VALID(pPolyBrep2);
          }

          SmPoint3d sPoint(15, 15, 0);

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
                  pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
                  pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sProjDirVec[0].Draw(&sPoint);
          sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // SM_GFX_CODE

      SmSolution sSData[256];
      SmSolutionArray sSolutions(256, sSData);
      double dBestAnswer = SM_BIG_DOUBLE;
      SER(SmPolySolver::PolyBrepPolyBrepSolve(
          pPolyBrep, pPolyBrep2, SM_SO_DIRECTED_MAXIMIZE, SM_SR_ALL, 1.0e-8, dBestAnswer, sProjDirVec, sSolutions));
      SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
      {
          sSolutions.Dump();

          SmPoint3d sPoint(15, 15, 0);

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
                  pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
                  pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sProjDirVec[0].Draw(&sPoint);
          sm_GraphicsLoop();
          for (ULONG k = 0; k < sSolutions.GetSize(); k++)
          {
                  SmSolution& rSol = sSolutions[k];
                  SmPoint3d sPnt1, sPnt2;
                  sPnt1.x = rSol.m_vStart[0];
                  sPnt1.y = rSol.m_vStart[1];
                  sPnt1.z = rSol.m_vStart[2];
                  sPnt2.x = rSol.m_vStart[3];
                  sPnt2.y = rSol.m_vStart[4];
                  sPnt2.z = rSol.m_vStart[5];
                  SmVector3d sDir = sPnt2 - sPnt1;
                  smgfx_SetLook(1, 2, 1, 0, 0);
                  sDir.Draw(&sPnt1);
                  sm_GraphicsLoop();
                  smgfx_SetLook(4, 5, 0, 0, 1);
                  sPnt2.Draw();
                  sm_GraphicsLoop();
          }
          sm_GraphicsLoop();
      }
#endif // SM_GFX_CODE
  }

  if (TRUE)
  { // Test polybrep-polybrep SM_SO_DIRECTED_MINIMIZE solver
      MYPRINTF(_T("\n my_test_poly_solver_1: test 5"));
      SmContext sContext;
      SmPolyBrep* pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"));
      pPolyBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"), SM_ASCII);
      SmObjDelete sClean1(pPolyBrep);
      SmPolyBrep* pPolyBrep2 = NULL;
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"));
      SER(SmPolyBrep::ReadFromSTLFile(
          sContext, _T("../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"), pPolyBrep2, SM_ASCII));
      SmObjDelete sClean2(pPolyBrep2);

      SmVector3d sProjDirVec[3];
      if (0)
      {
          sProjDirVec[0].Set(0, 1, 0); // Top View
          sProjDirVec[1].Set(-1, 0, 0); // Top View
      }
      else
      {
          sProjDirVec[0].Set(0, 0, 1); // Front View
          sProjDirVec[1].Set(-1, -0.3, 0); // Front View
          sProjDirVec[1].Unitize();
      }

#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
      {
          if (pPolyBrep)
          {
                  SM_ASSERT_VALID(pPolyBrep);
          }
          if (pPolyBrep2)
          {
                  SM_ASSERT_VALID(pPolyBrep2);
          }

          SmPoint3d sPoint(15, 15, 0);

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
                  pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
                  pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sProjDirVec[0].Draw(&sPoint);
          sm_GraphicsLoop();
          sm_GraphicsLoop();
      }
#endif // SM_GFX_CODE

      SmSolution sSData[256];
      SmSolutionArray sSolutions(256, sSData);
      double dBestAnswer = SM_BIG_DOUBLE;
      SER(SmPolySolver::PolyBrepPolyBrepSolve(
          pPolyBrep, pPolyBrep2, SM_SO_DIRECTED_MINIMIZE, SM_SR_ALL, 1.0e-8, dBestAnswer, sProjDirVec, sSolutions));
      SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
      if (smGet_DoGraphics())
      {
          sSolutions.Dump();

          SmPoint3d sPoint(15, 15, 0);

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
                  pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
                  pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sProjDirVec[0].Draw(&sPoint);
          sm_GraphicsLoop();
          for (ULONG k = 0; k < sSolutions.GetSize(); k++)
          {
                  SmSolution& rSol = sSolutions[k];
                  SmPoint3d sPnt1, sPnt2;
                  sPnt1.x = rSol.m_vStart[0];
                  sPnt1.y = rSol.m_vStart[1];
                  sPnt1.z = rSol.m_vStart[2];
                  sPnt2.x = rSol.m_vStart[3];
                  sPnt2.y = rSol.m_vStart[4];
                  sPnt2.z = rSol.m_vStart[5];
                  SmVector3d sDir = sPnt2 - sPnt1;
                  smgfx_SetLook(1, 2, 1, 0, 0);
                  sDir.Draw(&sPnt1);
                  sm_GraphicsLoop();
                  smgfx_SetLook(4, 5, 0, 0, 1);
                  sPnt2.Draw();
                  sm_GraphicsLoop();
          }
          sm_GraphicsLoop();
      }
#endif // SM_GFX_CODE
  }

  return SM_SUCCESS;

} // end my_test_poly_solver_1


PT_EXPORT SmStatus my_test_poly_solver_2()
{
MYPRINTF(_T("\nEntered in my_test_poly_solver_2"));

if (TRUE)
{ // Test polybrep-polybrep SM_SO_PROJECTED_MINIMIZE solver
        MYPRINTF(_T("\n my_test_poly_solver_2: test 1"));
        SmContext sContext;
        SmPolyBrep* pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"));
        pPolyBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"), SM_ASCII);
        SmObjDelete sClean1(pPolyBrep);
        SmPolyBrep* pPolyBrep2 = NULL;
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"));
        SER(SmPolyBrep::ReadFromSTLFile(
            sContext, _T("../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"), pPolyBrep2, SM_ASCII));
        SmObjDelete sClean2(pPolyBrep2);

        SmVector3d sProjDirVec[3];
        if (0)
        {
          sProjDirVec[0].Set(0, 1, 0); // Top View
        }
        else
        {
          sProjDirVec[0].Set(0, 0, 1); // Front View
        }

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          if (pPolyBrep)
          {
              SM_ASSERT_VALID(pPolyBrep);
          }
          if (pPolyBrep2)
          {
              SM_ASSERT_VALID(pPolyBrep2);
          }

          SmPoint3d sPoint(15, 15, 0);

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
              pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sProjDirVec[0].Draw(&sPoint);
          sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        SmSolution sSData[256];
        SmSolutionArray sSolutions(256, sSData);
        double dBestAnswer = SM_BIG_DOUBLE;
        SER(SmPolySolver::PolyBrepPolyBrepSolve(
            pPolyBrep, pPolyBrep2, SM_SO_PROJECTED_MINIMIZE, SM_SR_ALL, 1.0e-8, dBestAnswer, sProjDirVec, sSolutions));
        SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          sSolutions.Dump();

          SmPoint3d sPoint(15, 15, 0);

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
              pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sProjDirVec[0].Draw(&sPoint);
          sm_GraphicsLoop();
          for (ULONG k = 0; k < sSolutions.GetSize(); k++)
          {
              SmSolution& rSol = sSolutions[k];
              SmPoint3d sPnt1, sPnt2;
              sPnt1.x = rSol.m_vStart[0];
              sPnt1.y = rSol.m_vStart[1];
              sPnt1.z = rSol.m_vStart[2];
              sPnt2.x = rSol.m_vStart[3];
              sPnt2.y = rSol.m_vStart[4];
              sPnt2.z = rSol.m_vStart[5];
              SmVector3d sDir = sPnt2 - sPnt1;
              smgfx_SetLook(1, 2, 1, 0, 0);
              sDir.Draw(&sPnt1);
              sm_GraphicsLoop();
              smgfx_SetLook(4, 5, 0, 0, 1);
              sPnt2.Draw();
              sm_GraphicsLoop();
          }
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
}

if (TRUE)
{ // Test polybrep-polybrep SM_SO_PROJECTED_MAXIMIZE solver
        MYPRINTF(_T("\n my_test_poly_solver_2: test 2"));
        SmContext sContext;
        SmPolyBrep* pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"));
        pPolyBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"), SM_ASCII);
        SmObjDelete sClean1(pPolyBrep);
        SmPolyBrep* pPolyBrep2 = NULL;
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"));
        SER(SmPolyBrep::ReadFromSTLFile(
            sContext, _T("../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"), pPolyBrep2, SM_ASCII));
        SmObjDelete sClean2(pPolyBrep2);

        SmVector3d sProjDirVec[3];
        if (0)
        {
          sProjDirVec[0].Set(1, 0, 0); // Side View
        }
        else if (0)
        {
          sProjDirVec[0].Set(0, 1, 0); // Top View
        }
        else
        {
          sProjDirVec[0].Set(0, 0, 1); // Front View
        }

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          if (pPolyBrep)
          {
              SM_ASSERT_VALID(pPolyBrep);
          }
          if (pPolyBrep2)
          {
              SM_ASSERT_VALID(pPolyBrep2);
          }

          SmPoint3d sPoint(15, 15, 0);

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
              pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sProjDirVec[0].Draw(&sPoint);
          sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        SmSolution sSData[256];
        SmSolutionArray sSolutions(256, sSData);
        double dBestAnswer = SM_BIG_DOUBLE;
        SER(SmPolySolver::PolyBrepPolyBrepSolve(
            pPolyBrep, pPolyBrep2, SM_SO_PROJECTED_MAXIMIZE, SM_SR_ALL, 1.0e-8, dBestAnswer, sProjDirVec, sSolutions));
        SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          sSolutions.Dump();

          SmPoint3d sPoint(15, 15, 0);

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
              pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sProjDirVec[0].Draw(&sPoint);
          sm_GraphicsLoop();
          for (ULONG k = 0; k < sSolutions.GetSize(); k++)
          {
              SmSolution& rSol = sSolutions[k];
              SmPoint3d sPnt1, sPnt2;
              sPnt1.x = rSol.m_vStart[0];
              sPnt1.y = rSol.m_vStart[1];
              sPnt1.z = rSol.m_vStart[2];
              sPnt2.x = rSol.m_vStart[3];
              sPnt2.y = rSol.m_vStart[4];
              sPnt2.z = rSol.m_vStart[5];
              SmVector3d sDir = sPnt2 - sPnt1;
              smgfx_SetLook(2, 3, 1, 0, 0);
              sDir.Draw(&sPnt1);
              sm_GraphicsLoop();
              smgfx_SetLook(1, 2, 1, 0, 0);
              sDir.Draw(&sPnt1);
              sm_GraphicsLoop();
              smgfx_SetLook(4, 5, 0, 0, 1);
              sPnt2.Draw();
              sm_GraphicsLoop();
          }
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
}

if (TRUE)
{ // Test polybrep-polybrep SM_SO_MAXIMIZE solver
        MYPRINTF(_T("\n my_test_poly_solver_2: test 3"));
        SmContext sContext;
        SmPolyBrep* pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"));
        pPolyBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"), SM_ASCII);
        SmObjDelete sClean1(pPolyBrep);
        SmPolyBrep* pPolyBrep2 = NULL;
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"));
        SER(SmPolyBrep::ReadFromSTLFile(
            sContext, _T("../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"), pPolyBrep2, SM_ASCII));
        SmObjDelete sClean2(pPolyBrep2);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          if (pPolyBrep)
          {
              SM_ASSERT_VALID(pPolyBrep);
          }
          if (pPolyBrep2)
          {
              SM_ASSERT_VALID(pPolyBrep2);
          }

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
              pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        SmSolution sSData[256];
        SmSolutionArray sSolutions(256, sSData);
        double dBestAnswer = SM_BIG_DOUBLE;
        SER(SmPolySolver::PolyBrepPolyBrepSolve(
            pPolyBrep, pPolyBrep2, SM_SO_MAXIMIZE, SM_SR_ALL, 1.0e-8, dBestAnswer, NULL, sSolutions));
        SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          sSolutions.Dump();

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
              pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          for (ULONG k = 0; k < sSolutions.GetSize(); k++)
          {
              SmSolution& rSol = sSolutions[k];
              SmPoint3d sPnt1, sPnt2;
              sPnt1.x = rSol.m_vStart[0];
              sPnt1.y = rSol.m_vStart[1];
              sPnt1.z = rSol.m_vStart[2];
              sPnt2.x = rSol.m_vStart[3];
              sPnt2.y = rSol.m_vStart[4];
              sPnt2.z = rSol.m_vStart[5];
              SmVector3d sDir = sPnt2 - sPnt1;
              smgfx_SetLook(2, 3, 1, 0, 0);
              sDir.Draw(&sPnt1);
              sm_GraphicsLoop();
              smgfx_SetLook(1, 2, 1, 0, 0);
              sDir.Draw(&sPnt1);
              sm_GraphicsLoop();
              smgfx_SetLook(4, 5, 0, 0, 1);
              sPnt2.Draw();
              sm_GraphicsLoop();
          }
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
}

if (TRUE)
{ // Test polybrep-polybrep SM_SO_MINIMIZE solver
        MYPRINTF(_T("\n my_test_poly_solver_2: test 4"));
        SmContext sContext;
        SmPolyBrep* pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"));
        pPolyBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"), SM_ASCII);
        SmObjDelete sClean1(pPolyBrep);
        SmPolyBrep* pPolyBrep2 = NULL;
        MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"));
        SER(SmPolyBrep::ReadFromSTLFile(
            sContext, _T("../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"), pPolyBrep2, SM_ASCII));
        SmObjDelete sClean2(pPolyBrep2);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          if (pPolyBrep)
          {
              SM_ASSERT_VALID(pPolyBrep);
          }
          if (pPolyBrep2)
          {
              SM_ASSERT_VALID(pPolyBrep2);
          }

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
              pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        SmSolution sSData[256];
        SmSolutionArray sSolutions(256, sSData);
        double dBestAnswer = SM_BIG_DOUBLE;
        SER(SmPolySolver::PolyBrepPolyBrepSolve(
            pPolyBrep, pPolyBrep2, SM_SO_MINIMIZE, SM_SR_ALL, 1.0e-8, dBestAnswer, NULL, sSolutions));
        SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          sSolutions.Dump();

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 0);
          if (pPolyBrep2)
              pPolyBrep2->Draw(TRUE);
          sm_GraphicsLoop();
          for (ULONG k = 0; k < sSolutions.GetSize(); k++)
          {
              SmSolution& rSol = sSolutions[k];
              SmPoint3d sPnt1, sPnt2;
              sPnt1.x = rSol.m_vStart[0];
              sPnt1.y = rSol.m_vStart[1];
              sPnt1.z = rSol.m_vStart[2];
              sPnt2.x = rSol.m_vStart[3];
              sPnt2.y = rSol.m_vStart[4];
              sPnt2.z = rSol.m_vStart[5];
              SmVector3d sDir = sPnt2 - sPnt1;
              smgfx_SetLook(2, 3, 1, 0, 0);
              sDir.Draw(&sPnt1);
              sm_GraphicsLoop();
              smgfx_SetLook(1, 2, 1, 0, 0);
              sDir.Draw(&sPnt1);
              sm_GraphicsLoop();
              smgfx_SetLook(4, 5, 0, 0, 1);
              sPnt2.Draw();
              sm_GraphicsLoop();
          }
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
}

return SM_SUCCESS;

} // end my_test_poly_solver_2

/*****************************************************
PURPOSE ---

USAGE NOTES ---   my_test_poly_solver_1() and my_test_poly_solver_2()
  used to be a single routine, but some compilers
  (64-bit on Vista) have a problem with that, so it was broken up
  into two, _1 and _2.
*****************************************************/
PT_EXPORT SmStatus my_test_poly_solver_3()
{
    MYPRINTF(_T("\n Entered in my_test_poly_solver_3\n"));

  if (TRUE) 
    { // Test point-polybrep SM_SO_3D_SIGNED_DIRECTED_MINIMIZE solver
      MYPRINTF(_T("\n my_test_poly_solver_3: test 1")) ;
      SmContext sContext;
      SmPolyBrep * pPolyBrep = NULL;
      SER(my_brepfile_to_polybrep(sContext,_T("../../TestFiles/pt_TestFiles/Solids/extrusion_old.smb"),
                                  0.0,5.0,15.0,pPolyBrep));
      SmObjDelete sClean1(pPolyBrep);

      SmVector3d sProjDirVec[3];
      if (0)      { sProjDirVec[0].Set(0,1,0);  }
      else if (0) { sProjDirVec[0].Set(0,-1,0); }
      else        { sProjDirVec[0].Set(-0.3,1,0); }
      sProjDirVec[0].Unitize();
      SmPoint3d sPoint(5.0,16.0,7.0);

#ifdef SM_GFX_CODE
      if(pPolyBrep) { SM_ASSERT_VALID(pPolyBrep); }

      if(smGet_DoGraphics())
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep)  pPolyBrep->Draw(TRUE); sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 1,0,0) ; sPoint.Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; sProjDirVec[0].Draw(&sPoint); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      SmSolution      sSData[64];
      SmSolutionArray sSolutions(64,sSData);
      double          dBestAnswer = SM_BIG_DOUBLE;
      SER(SmPolySolver::PolyBrepPointSolve(pPolyBrep,
                                           sPoint,
                                           SM_SO_3D_SIGNED_DIRECTED_MINIMIZE,
                                           SM_SR_ALL,
                                           1.0e-8,
                                           dBestAnswer,
                                           sProjDirVec,
                                           sSolutions));
      SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          sSolutions.Dump() ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep)  pPolyBrep->Draw(TRUE); sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 1,0,0) ; sPoint.Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; sProjDirVec[0].Draw(&sPoint); sm_GraphicsLoop() ;

          for (ULONG k=0; k<sSolutions.GetSize(); k++) 
            {
              SmSolution & rSol = sSolutions[k];
              SmPoint3d sPnt1, sPnt2;
              sPnt1.x = rSol.m_vStart[0];
              sPnt1.y = rSol.m_vStart[1];
              sPnt1.z = rSol.m_vStart[2];
              sPnt2.x = rSol.m_vStart[3];
              sPnt2.y = rSol.m_vStart[4];
              sPnt2.z = rSol.m_vStart[5];
              SmVector3d sDir = sPnt2 - sPoint;
              smgfx_SetLook(2,3, 1,0,0); sDir.Draw(&sPoint); sm_GraphicsLoop();
              smgfx_SetLook(4,5, 0,0,1); sPnt2.Draw(); sm_GraphicsLoop() ;
            }
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
    }

  if (TRUE) 
    { // Test point-polybrep SM_SO_SIGNED_DIRECTED_MINIMIZE solver
      MYPRINTF(_T("\n my_test_poly_solver_3: test 2")) ;
      SmContext    sContext;
      SmPolyBrep * pPolyBrep = NULL;
      SER(my_brepfile_to_polybrep(sContext,_T("../../TestFiles/pt_TestFiles/Solids/extrusion_old.smb"),
                                  0.0,5.0,15.0,pPolyBrep));
      SmObjDelete sClean1(pPolyBrep);

      SmVector3d sProjDirVec[3];
      if (0)      { sProjDirVec[0].Set(1,0,0); // Side View
                    sProjDirVec[1].Set(0,1,0); // Side View
                  }
      else if (0) { sProjDirVec[0].Set(0,1,0); // Top View
                    sProjDirVec[1].Set(1,0,0); // Top View
                  }
      else        { sProjDirVec[0].Set(0,0,1); // Front View
                    sProjDirVec[1].Set(1,1,0); // Front View
                    sProjDirVec[1].Unitize();
                  }
      SmPoint3d sPoint(13.0,16.0,7.0);

  #ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          if(pPolyBrep) { SM_ASSERT_VALID(pPolyBrep); }

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep)  pPolyBrep->Draw(TRUE); sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 1,0,0) ; sPoint.Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; sProjDirVec[0].Draw(&sPoint); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
  #endif // SM_GFX_CODE

      SmSolution sSData[64];
      SmSolutionArray sSolutions(64,sSData);
      double dBestAnswer=SM_BIG_DOUBLE;
      SER(SmPolySolver::PolyBrepPointSolve(pPolyBrep,
                                           sPoint,
                                           SM_SO_SIGNED_DIRECTED_MINIMIZE,
                                           SM_SR_ALL,
                                           1.0e-8,
                                           dBestAnswer,
                                           sProjDirVec,
                                           sSolutions));
      SM_ASSERT(sSolutions.GetSize() > 0);

  #ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          sSolutions.Dump() ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep)  pPolyBrep->Draw(TRUE); sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 1,0,0) ; sPoint.Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; sProjDirVec[0].Draw(&sPoint); sm_GraphicsLoop() ;

          for (ULONG k=0; k<sSolutions.GetSize(); k++) 
            {
              SmSolution & rSol = sSolutions[k];
              SmPoint3d sPnt1, sPnt2;
              sPnt1.x = rSol.m_vStart[0];
              sPnt1.y = rSol.m_vStart[1];
              sPnt1.z = rSol.m_vStart[2];
              sPnt2.x = rSol.m_vStart[3];
              sPnt2.y = rSol.m_vStart[4];
              sPnt2.z = rSol.m_vStart[5];
              SmVector3d sDir = sPnt2 - sPoint;
              smgfx_SetLook(2,3, 1,0,0); sDir.Draw(&sPoint); sm_GraphicsLoop();
              smgfx_SetLook(4,5, 0,0,1); sPnt2.Draw(); sm_GraphicsLoop() ;
            }
          sm_GraphicsLoop();
        }
  #endif // SM_GFX_CODE
    }

  if (TRUE) 
    { // Test point-polybrep SM_SO_DIRECTED_MAXIMIZE solver
      MYPRINTF(_T("\n my_test_poly_solver_3: test 3")) ;
      SmContext sContext;
      SmPolyBrep * pPolyBrep = NULL;
      SER(my_brepfile_to_polybrep(sContext,_T("../../TestFiles/pt_TestFiles/Solids/extrusion_old.smb"),
                                  0.0,5.0, 15.0,pPolyBrep));
      SmObjDelete sClean1(pPolyBrep);

      SmVector3d sProjDirVec[3];
      if (0) {
          sProjDirVec[0].Set(1,0,0); // Side View
          sProjDirVec[1].Set(0,-1,0); // Side View
      }
      else if (0) {
          sProjDirVec[0].Set(0,1,0); // Top View
          sProjDirVec[1].Set(-1,0,0); // Top View
      }
      else {
          sProjDirVec[0].Set(0,0,1); // Front View
          sProjDirVec[1].Set(-1,-1,0); // Front View
          sProjDirVec[1].Unitize();
      }
      SmPoint3d sPoint(13.0,16.0,7.0);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          if(pPolyBrep) { SM_ASSERT_VALID(pPolyBrep); }

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep)  pPolyBrep->Draw(TRUE); sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 1,0,0) ; sPoint.Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; sProjDirVec[0].Draw(&sPoint); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      SmSolution sSData[64];
      SmSolutionArray sSolutions(64,sSData);
      double dBestAnswer=SM_BIG_DOUBLE;
      SER(SmPolySolver::PolyBrepPointSolve(pPolyBrep,
                                           sPoint,
                                           SM_SO_DIRECTED_MAXIMIZE,
                                           SM_SR_ALL,
                                           1.0e-8,
                                           dBestAnswer,
                                           sProjDirVec,
                                           sSolutions));
      SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          sSolutions.Dump() ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep)  pPolyBrep->Draw(TRUE); sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 1,0,0) ; sPoint.Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; sProjDirVec[0].Draw(&sPoint); sm_GraphicsLoop() ;

          for (ULONG k=0; k<sSolutions.GetSize(); k++) 
            {
              SmSolution & rSol = sSolutions[k];
              SmPoint3d sPnt1, sPnt2;
              sPnt1.x = rSol.m_vStart[0];
              sPnt1.y = rSol.m_vStart[1];
              sPnt1.z = rSol.m_vStart[2];
              sPnt2.x = rSol.m_vStart[3];
              sPnt2.y = rSol.m_vStart[4];
              sPnt2.z = rSol.m_vStart[5];
              SmVector3d sDir = sPnt2 - sPoint;
              smgfx_SetLook(2,3, 1,0,0); sDir.Draw(&sPoint); sm_GraphicsLoop();
              smgfx_SetLook(4,5, 0,0,1); sPnt2.Draw(); sm_GraphicsLoop() ;
            }
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
    }

  if (TRUE) 
    { // Test point-polybrep SM_SO_DIRECTED_MINIMIZE solver
      MYPRINTF(_T("\n my_test_poly_solver_3: test 4")) ;
      SmContext sContext;
      SmPolyBrep * pPolyBrep = NULL;
      SER(my_brepfile_to_polybrep(sContext,_T("../../TestFiles/pt_TestFiles/Solids/extrusion_old.smb"),
                                  0.0,5.0, 15.0,pPolyBrep));
      SmObjDelete sClean1(pPolyBrep);

      SmVector3d sProjDirVec[3];
      if (0) {
          sProjDirVec[0].Set(1,0,0); // Side View
          sProjDirVec[1].Set(0,-1,0); // Side View
      }
      else if (0) {
          sProjDirVec[0].Set(0,1,0); // Top View
          sProjDirVec[1].Set(-1,0,0); // Top View
      }
      else {
          sProjDirVec[0].Set(0,0,1); // Front View
          sProjDirVec[1].Set(-1,-1,0); // Front View
          sProjDirVec[1].Unitize();
      }
      SmPoint3d sPoint(13.0,16.0,7.0);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          if(pPolyBrep) { SM_ASSERT_VALID(pPolyBrep); }

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep)  pPolyBrep->Draw(TRUE); sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 1,0,0) ; sPoint.Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; sProjDirVec[0].Draw(&sPoint); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      SmSolution sSData[64];
      SmSolutionArray sSolutions(64,sSData);
      double dBestAnswer=SM_BIG_DOUBLE;
      SER(SmPolySolver::PolyBrepPointSolve(pPolyBrep,
                                           sPoint,
                                           SM_SO_DIRECTED_MINIMIZE,
                                           SM_SR_ALL,
                                           1.0e-8,
                                           dBestAnswer,
                                           sProjDirVec,
                                           sSolutions));
      SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          sSolutions.Dump() ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep)  pPolyBrep->Draw(TRUE); sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 1,0,0) ; sPoint.Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; sProjDirVec[0].Draw(&sPoint); sm_GraphicsLoop() ;

          for (ULONG k=0; k<sSolutions.GetSize(); k++) 
            {
              SmSolution & rSol = sSolutions[k];
              SmPoint3d sPnt1, sPnt2;
              sPnt1.x = rSol.m_vStart[0];
              sPnt1.y = rSol.m_vStart[1];
              sPnt1.z = rSol.m_vStart[2];
              sPnt2.x = rSol.m_vStart[3];
              sPnt2.y = rSol.m_vStart[4];
              sPnt2.z = rSol.m_vStart[5];
              SmVector3d sDir = sPnt2 - sPoint;
              smgfx_SetLook(2,3, 1,0,0); sDir.Draw(&sPoint); sm_GraphicsLoop();
              smgfx_SetLook(4,5, 0,0,1); sPnt2.Draw(); sm_GraphicsLoop() ;
            }
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
    }

  if (TRUE) 
    { // Test point-polybrep SM_SO_PROJECTED_MINIMIZE solver
      MYPRINTF(_T("\n my_test_poly_solver_3: test 5")) ;
      SmContext sContext;
      SmPolyBrep * pPolyBrep = NULL;
      if (1) {
          SER(my_brepfile_to_polybrep(sContext,_T("../../TestFiles/pt_TestFiles/Solids/extrusion_old.smb"),
              0.0,5.0, 15.0,pPolyBrep));
      }
      else {
          pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
          MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"));
          pPolyBrep->ReadFromFile(sContext,_T("../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"),SM_ASCII);
      }
      SmObjDelete sClean1(pPolyBrep);

      SmVector3d sProjDirVec[3];
      if (0) {
          sProjDirVec[0].Set(1,0,0); // Side View
      }
      else if (0) {
          sProjDirVec[0].Set(0,1,0); // Top View
      }
      else {
          sProjDirVec[0].Set(0,0,1); // Front View
      }
      SmPoint3d sPoint(-23.0,13.0,17.0);
//        SmPoint3d sPoint(13.0,13.0,7.0);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          if(pPolyBrep) { SM_ASSERT_VALID(pPolyBrep); }

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep)  pPolyBrep->Draw(TRUE); sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 1,0,0) ; sPoint.Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; sProjDirVec[0].Draw(&sPoint); sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      SmSolution sSData[64];
      SmSolutionArray sSolutions(64,sSData);
      double dBestAnswer=SM_BIG_DOUBLE;
      SER(SmPolySolver::PolyBrepPointSolve(pPolyBrep,
                                           sPoint,
                                           SM_SO_PROJECTED_MINIMIZE,
                                           SM_SR_ALL,
                                           1.0e-8,
                                           dBestAnswer,
                                           sProjDirVec,
                                           sSolutions));
      SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          sSolutions.Dump() ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep)  pPolyBrep->Draw(TRUE); sm_GraphicsLoop() ;
          smgfx_SetLook(4,5, 1,0,0) ; sPoint.Draw(); sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; sProjDirVec[0].Draw(&sPoint); sm_GraphicsLoop() ;

          for (ULONG k=0; k<sSolutions.GetSize(); k++) 
            {
              SmSolution & rSol = sSolutions[k];
              SmPoint3d sPnt1, sPnt2;
              sPnt1.x = rSol.m_vStart[0];
              sPnt1.y = rSol.m_vStart[1];
              sPnt1.z = rSol.m_vStart[2];
              sPnt2.x = rSol.m_vStart[3];
              sPnt2.y = rSol.m_vStart[4];
              sPnt2.z = rSol.m_vStart[5];
              SmVector3d sDir = sPnt2 - sPoint;
              smgfx_SetLook(2,3, 1,0,0); sDir.Draw(&sPoint); sm_GraphicsLoop();
              smgfx_SetLook(4,5, 0,0,1); sPnt2.Draw(); sm_GraphicsLoop() ;
            }
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
    }

  // all done
  return SM_SUCCESS;

} // end my_test_poly_solver_3

PT_EXPORT SmStatus my_test_poly_solver_4()
{
  MYPRINTF(_T("\n Entered in my_test_poly_solver_4\n"));

  if (TRUE)
  { // Test point-polybrep SM_SO_PROJECTED_MAXIMIZE solver
        MYPRINTF(_T("\n my_test_poly_solver_4: test 1"));
        SmContext sContext;
        SmPolyBrep* pPolyBrep = NULL;
        if (1)
        {
          SER(my_brepfile_to_polybrep(
              sContext, _T("../../TestFiles/pt_TestFiles/Solids/extrusion_old.smb"), 0.0, 5.0, 15.0, pPolyBrep));
        }
        else
        {
          pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
          MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"));
          pPolyBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"), SM_ASCII);
        }
        SmObjDelete sClean1(pPolyBrep);


        SmVector3d sProjDirVec[3];
        if (0)
        {
          sProjDirVec[0].Set(1, 0, 0); // Side View
        }
        else if (0)
        {
          sProjDirVec[0].Set(0, 1, 0); // Top View
        }
        else
        {
          sProjDirVec[0].Set(0, 0, 1); // Front View
        }
        SmPoint3d sPoint(-23.0, 13.0, 17.0);
        //        SmPoint3d sPoint(13.0,13.0,7.0);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          if (pPolyBrep)
          {
              SM_ASSERT_VALID(pPolyBrep);
          }

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sProjDirVec[0].Draw(&sPoint);
          sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        SmSolution sSData[64];
        SmSolutionArray sSolutions(64, sSData);
        double dBestAnswer = SM_BIG_DOUBLE;
        SER(SmPolySolver::PolyBrepPointSolve(
            pPolyBrep, sPoint, SM_SO_PROJECTED_MAXIMIZE, SM_SR_ALL, 1.0e-8, dBestAnswer, sProjDirVec, sSolutions));
        SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          sSolutions.Dump();

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sProjDirVec[0].Draw(&sPoint);
          sm_GraphicsLoop();

          for (ULONG k = 0; k < sSolutions.GetSize(); k++)
          {
              SmSolution& rSol = sSolutions[k];
              SmPoint3d sPnt1, sPnt2;
              sPnt1.x = rSol.m_vStart[0];
              sPnt1.y = rSol.m_vStart[1];
              sPnt1.z = rSol.m_vStart[2];
              sPnt2.x = rSol.m_vStart[3];
              sPnt2.y = rSol.m_vStart[4];
              sPnt2.z = rSol.m_vStart[5];
              SmVector3d sDir = sPnt2 - sPoint;
              smgfx_SetLook(2, 3, 1, 0, 0);
              sDir.Draw(&sPoint);
              sm_GraphicsLoop();
              smgfx_SetLook(4, 5, 0, 0, 1);
              sPnt2.Draw();
              sm_GraphicsLoop();
          }
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
  }

  if (TRUE)
  { // Test point-polybrep SM_SO_MAXIMIZE solver
        MYPRINTF(_T("\n my_test_poly_solver_4: test 2"));
        SmContext sContext;
        SmPolyBrep* pPolyBrep = NULL;
        if (1)
        {
          SER(my_brepfile_to_polybrep(
              sContext, _T("../../TestFiles/pt_TestFiles/Solids/extrusion_old.smb"), 0.0, 5.0, 15.0, pPolyBrep));
        }
        else
        {
          pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
          MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"));
          pPolyBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"), SM_ASCII);
        }
        SmObjDelete sClean1(pPolyBrep);

        SmPoint3d sPoint(-23.0, 13.0, 17.0);
        //        SmPoint3d sPoint(13.0,13.0,7.0);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          if (pPolyBrep)
          {
              SM_ASSERT_VALID(pPolyBrep);
          }

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        SmSolution sSData[64];
        SmSolutionArray sSolutions(64, sSData);
        double dBestAnswer = SM_BIG_DOUBLE;
        SER(SmPolySolver::PolyBrepPointSolve(
            pPolyBrep, sPoint, SM_SO_MAXIMIZE, SM_SR_ALL, 1.0e-8, dBestAnswer, NULL, sSolutions));
        SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          sSolutions.Dump();

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();

          for (ULONG k = 0; k < sSolutions.GetSize(); k++)
          {
              SmSolution& rSol = sSolutions[k];
              SmPoint3d sPnt1, sPnt2;
              sPnt1.x = rSol.m_vStart[0];
              sPnt1.y = rSol.m_vStart[1];
              sPnt1.z = rSol.m_vStart[2];
              sPnt2.x = rSol.m_vStart[3];
              sPnt2.y = rSol.m_vStart[4];
              sPnt2.z = rSol.m_vStart[5];
              SmVector3d sDir = sPnt2 - sPoint;
              smgfx_SetLook(2, 3, 1, 0, 0);
              sDir.Draw(&sPoint);
              sm_GraphicsLoop();
              smgfx_SetLook(4, 5, 0, 0, 1);
              sPnt2.Draw();
              sm_GraphicsLoop();
          }
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
  }

  if (TRUE)
  { // Test point-polybrep SM_SO_MINIMIZE solver
        MYPRINTF(_T("\n my_test_poly_solver_4: test 3"));
        SmContext sContext;
        SmPolyBrep* pPolyBrep = NULL;
        if (1)
        {
          SER(my_brepfile_to_polybrep(
              sContext, _T("../../TestFiles/pt_TestFiles/Solids/extrusion_old.smb"), 0.0, 5.0, 15.0, pPolyBrep));
        }
        else
        {
          pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
          MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"));
          pPolyBrep->ReadFromFile(sContext, _T("../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"), SM_ASCII);
        }
        SmObjDelete sClean1(pPolyBrep);

        //        SmPoint3d sPoint(-23.0,13.0,17.0);
        SmPoint3d sPoint(13.0, 13.0, 7.0);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          if (pPolyBrep)
          {
              SM_ASSERT_VALID(pPolyBrep);
          }

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        SmSolution sSData[64];
        SmSolutionArray sSolutions(64, sSData);
        double dBestAnswer = SM_BIG_DOUBLE;
        SER(SmPolySolver::PolyBrepPointSolve(
            pPolyBrep, sPoint, SM_SO_MINIMIZE, SM_SR_ALL, 1.0e-8, dBestAnswer, NULL, sSolutions));
        SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          sSolutions.Dump();

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();

          for (ULONG k = 0; k < sSolutions.GetSize(); k++)
          {
              SmSolution& rSol = sSolutions[k];
              SmPoint3d sPnt1, sPnt2;
              sPnt1.x = rSol.m_vStart[0];
              sPnt1.y = rSol.m_vStart[1];
              sPnt1.z = rSol.m_vStart[2];
              sPnt2.x = rSol.m_vStart[3];
              sPnt2.y = rSol.m_vStart[4];
              sPnt2.z = rSol.m_vStart[5];
              SmVector3d sDir = sPnt2 - sPoint;
              smgfx_SetLook(2, 3, 1, 0, 0);
              sDir.Draw(&sPoint);
              sm_GraphicsLoop();
              smgfx_SetLook(4, 5, 0, 0, 1);
              sPnt2.Draw();
              sm_GraphicsLoop();
          }
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
  }

  if (TRUE)
  { // Test plane-polybrep solver
        MYPRINTF(_T("\n my_test_poly_solver_4: test 4we"));
        SmContext sContext;
        SmPolyBrep* pPolyBrep = NULL;
        SER(my_brepfile_to_polybrep(
            sContext, _T("../../TestFiles/pt_TestFiles/Solids/extrusion_old.smb"), 0.0, 5.0, 15.0, pPolyBrep));
        SmObjDelete sClean1(pPolyBrep);
        //        SER(SmPolyBrep::ReadFromSTLFile(sContext,"../../TestFiles/pt_TestFiles/Tess/extrusion_old.stl",
        //        SER(SmPolyBrep::ReadFromSTLFile(sContext,"../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl",
        //                                        pPolyBrep,SM_ASCII));
        //        SmPoint3d sPoint(26,0,0);//BoxFillet.stl
        SmPoint3d sPoint(12, 15, 0); // extrusion_old.stl
        //        SmPoint3d sPoint(-2,-5,5);//extrusion_old.stl
        //        SmPoint3d sPoint(8,9,5);//extrusion_old.stl
        SmVector3d sNormal(-1, -1, 0);
        SER(sNormal.Unitize());

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pPolyBrep);

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sNormal.Draw(&sPoint);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 1);
          sNormal.DrawPlane(sPoint);
          sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

        SmSolution sSData[64];
        SmSolutionArray sSolutions(64, sSData);
        double dBestAnswer = SM_BIG_DOUBLE;
        SER(SmPolySolver::PolyBrepPlaneSolve(
            pPolyBrep, sPoint, sNormal, SM_SO_MINIMIZE, SM_SR_ALL, 1.0e-8, dBestAnswer, NULL, sSolutions));
        SM_ASSERT(sSolutions.GetSize() > 0);

#ifdef SM_GFX_CODE
        if (smGet_DoGraphics())
        {
          sSolutions.Dump();

          smgfx_Erase();
          smgfx_SetLook(1, 2, 0, 0, 1);
          if (pPolyBrep)
              pPolyBrep->Draw(TRUE);
          sm_GraphicsLoop();
          smgfx_SetLook(4, 5, 1, 0, 0);
          sPoint.Draw();
          sm_GraphicsLoop();
          smgfx_SetLook(3, 4, 0, 1, 0);
          sNormal.Draw(&sPoint);
          sm_GraphicsLoop();
          smgfx_SetLook(1, 2, 0, 1, 1);
          sNormal.DrawPlane(sPoint);
          sm_GraphicsLoop();

          for (ULONG k = 0; k < sSolutions.GetSize(); k++)
          {
              SmSolution& rSol = sSolutions[k];
              SmPoint3d sPnt1, sPnt2;
              sPnt1.x = rSol.m_vStart[0];
              sPnt1.y = rSol.m_vStart[1];
              sPnt1.z = rSol.m_vStart[2];
              sPnt2.x = rSol.m_vStart[3];
              sPnt2.y = rSol.m_vStart[4];
              sPnt2.z = rSol.m_vStart[5];
              SmVector3d sDir = sPnt2 - sPnt1;
              smgfx_SetLook(4, 5, 1, 0, 0);
              sPnt1.Draw();
              sm_GraphicsLoop();
              smgfx_SetLook(2, 3, 1, 0, 0);
              sDir.Draw(&sPnt1);
              sm_GraphicsLoop();
              smgfx_SetLook(4, 5, 0, 0, 1);
              sPnt2.Draw();
              sm_GraphicsLoop();
          }
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
  }

  // all done
  return SM_SUCCESS;

} // end my_test_poly_solver_4

/*****************************************************
PURPOSE --- Verify PolyVertex face/normal table consistency.

USAGE NOTES ---
*****************************************************/
static SmStatus my_verify_polybrep_face_normal_table
 (SmPolyBrep * pPolyBrep)
{
  if(pPolyBrep == NULL)
    {
      MYPRINTF(_T("\n my_verify_polybrep_face_normal_table: NULL PolyBrep"));
      return SM_ERR_NULL_POINTER;
    }

  SmTArray<SmPolyFace*> sPolyFaces;
  pPolyBrep->GetPolyFaces(sPolyFaces);

  for(ULONG ii=0; ii<sPolyFaces.GetSize(); ++ii)
    {
      SmPolyFace * pPolyFace = sPolyFaces[ii];
      if(pPolyFace == NULL)
        {
          MYPRINTF(_T("\n my_verify_polybrep_face_normal_table: NULL PolyFace"));
          return SM_ERR_NULL_POINTER;
        }

      SmFace * pOriginalFace = pPolyFace->GetOriginalFace();
      if(pOriginalFace == NULL)
        {
          TCHAR sBuff[SM_TBLOCK_SIZE];
          SM_SPRINTF(sBuff, _T("\n PolyFace %ld missing original SmFace label"), ii);
          MYPRINTF(sBuff);
          return SM_ERR_NULL_POINTER;
        }

      SmPolyLoop * pPolyLoop = pPolyFace->GetOuterPolyLoop();
      if(pPolyLoop == NULL)
        { continue; }

      SmTArray<SmPolyEdge*> sPolyEdges;
      pPolyLoop->GetPolyEdges(sPolyEdges);
      for(ULONG jj=0; jj<sPolyEdges.GetSize(); ++jj)
        {
          SmPolyVertex * pPolyVertex = sPolyEdges[jj]->GetStartPolyVertex();
          if(pPolyVertex == NULL)
            {
              MYPRINTF(_T("\n my_verify_polybrep_face_normal_table: NULL PolyVertex"));
              return SM_ERR_NULL_POINTER;
            }

          SmTArray<SmTopology*> & rFaces   = pPolyVertex->GetFacesRef();
          SmTArray<SmVector3d>  & rNormals = pPolyVertex->GetNormalsRef();
          if(rFaces.GetSize() != rNormals.GetSize())
            {
              TCHAR sBuff[SM_TBLOCK_SIZE];
              SM_SPRINTF(sBuff, _T("\n PolyVertex face/normal table size mismatch: face %ld edge %ld faces %ld normals %ld"),
                         ii, jj, rFaces.GetSize(), rNormals.GetSize());
              MYPRINTF(sBuff);
              return SM_ERR;
            }

          ULONG lFoundIndex;
          if(!rFaces.FindElement(pOriginalFace, lFoundIndex))
            {
              TCHAR sBuff[SM_TBLOCK_SIZE];
              SM_SPRINTF(sBuff, _T("\n PolyVertex face/normal table missing PolyFace original SmFace: face %ld edge %ld"),
                         ii, jj);
              MYPRINTF(sBuff);
              return SM_ERR;
            }
        }
    }

  return SM_SUCCESS;

} // end my_verify_polybrep_face_normal_table

/*****************************************************
PURPOSE --- Verify GlueVertices preserves PolyVertex face/normal entries.

USAGE NOTES ---
*****************************************************/
static SmStatus my_test_polybrep_glue_preserves_face_normal_table()
{
  SmContext sContext;
  SmPolyBrep * pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
  SmObjDelete sCleanPolyBrep(pPolyBrep);

  SmPolyVertex * pKeep   = new (pPolyBrep) SmPolyVertex(SmPoint3d(0.0, 0.0, 0.0), SM_EFF_ZERO);
  SmPolyVertex * pDelete = new (pPolyBrep) SmPolyVertex(SmPoint3d(1.0, 0.0, 0.0), SM_EFF_ZERO);
  SmPolyVertex * pOther  = new (pPolyBrep) SmPolyVertex(SmPoint3d(0.0, 1.0, 0.0), SM_EFF_ZERO);
  NER(pKeep);
  NER(pDelete);
  NER(pOther);

  SmTArray<SmPolyVertex*> sVertices;
  sVertices.Add(pKeep);
  sVertices.Add(pDelete);
  sVertices.Add(pOther);

  SmVector3d sFaceNormal(0.0, 0.0, 1.0);
  SmPolyFace * pPolyFace = NULL;
  SER(pPolyBrep->CreatePolyFace(NULL, NULL, sVertices, pPolyFace, &sFaceNormal));
  NER(pPolyFace);

  SmVector3d sDeletedVertexNormal(0.25, 0.5, 1.0);
  pDelete->AddFaceToList(pPolyFace);
  pDelete->AddFaceNormal(sDeletedVertexNormal);

  SmTArray<SmPolyEdge*> sEdgesBetweenVertices;
  SER(pPolyBrep->GlueVertices(pKeep, pDelete, sEdgesBetweenVertices));

  SmTArray<SmTopology*> & rFaces   = pKeep->GetFacesRef();
  SmTArray<SmVector3d>  & rNormals = pKeep->GetNormalsRef();
  SER_MSG(rFaces.GetSize() == rNormals.GetSize() ? SM_SUCCESS : SM_ERR,
          _T("GlueVertices produced mismatched face/normal table"));

  ULONG lFoundIndex;
  if(!rFaces.FindElement(pPolyFace, lFoundIndex))
    {
      SER_MSG(SM_ERR, _T("GlueVertices failed to preserve deleted PolyVertex face entry"));
    }

  SmVector3d sNormalDelta = rNormals[lFoundIndex] - sDeletedVertexNormal;
  SER_MSG(sNormalDelta.LengthSquared() <= SM_EFF_ZERO ? SM_SUCCESS : SM_ERR,
          _T("GlueVertices failed to preserve deleted PolyVertex normal entry"));

  return SM_SUCCESS;

} // end my_test_polybrep_glue_preserves_face_normal_table

/*****************************************************
PURPOSE --- Convert a Brep to PolyBrep and verify every corner can find
            its PolyFace original SmFace in the vertex normal table.

USAGE NOTES ---
*****************************************************/
static SmStatus my_verify_brep_convert_to_polybrep_face_normal_table
 (SmBrep * pBrep)
{
  if(pBrep == NULL)
    {
      MYPRINTF(_T("\n NULL Brep while testing face/normal tables"));
      return SM_ERR_NULL_POINTER;
    }

  SmPolyBrep * pPolyBrep = NULL;
  SmBoolean bFailedFaces = FALSE;
  ULONG lNumLamina = 0;
  SmStatus eConvertStatus = pBrep->ConvertToPolyBrep(pPolyBrep,
                                                     bFailedFaces,
                                                     lNumLamina,
                                                     0.0,
                                                     10.0,
                                                     30.0,
                                                     0.0,
                                                     0.0,
                                                     FALSE,
                                                     FALSE,
                                                     TRUE);
  if(eConvertStatus != SM_SUCCESS)
    {
      MYPRINTF(_T("\n ConvertToPolyBrep returned an error while testing face/normal tables"));
      return eConvertStatus;
    }
  if(pPolyBrep == NULL)
    {
      MYPRINTF(_T("\n ConvertToPolyBrep returned a NULL PolyBrep while testing face/normal tables"));
      return SM_ERR_NULL_POINTER;
    }
  SmObjDelete sCleanPolyBrep(pPolyBrep);
  if(bFailedFaces)
    {
      MYPRINTF(_T("\n ConvertToPolyBrep failed one or more faces while testing face/normal tables"));
      return SM_ERR;
    }

  SER(my_verify_polybrep_face_normal_table(pPolyBrep));

  return SM_SUCCESS;

} // end my_verify_brep_convert_to_polybrep_face_normal_table

/*****************************************************
PURPOSE --- Build a cone Brep and verify its tessellated vertex normal table.

USAGE NOTES ---
*****************************************************/
static SmStatus my_test_cone_face_normal_table
 (double dStartAngle,
  double dEndAngle)
{
  SmContext sContext;
  SmBrep * pBrep = new (sContext) SmBrep();
  if(pBrep == NULL)
    {
      MYPRINTF(_T("\n Failed to allocate cone Brep while testing face/normal tables"));
      return SM_ERR_NULL_POINTER;
    }
  SmObjDelete sCleanBrep(pBrep);
  pBrep->SetTolerance(0.0001);

  SmAxis2Placement sRefFrame;
  sRefFrame.SetCanonical(SmPoint3d(0.0, 0.0, 0.0),
                         SmVector3d(0.0, 0.0, 1.0),
                         SmVector3d(1.0, 0.0, 0.0));

  SmPrimitiveCreation sPrimitiveCreation(pBrep->GetInfiniteRegion(), 123455);
  SER(sPrimitiveCreation.CreateCone(10.0,
                                    1.0,
                                    2.0,
                                    dStartAngle,
                                    dEndAngle,
                                    sRefFrame));

  SmTArray<SmFace*> sFaces;
  pBrep->GetFaces(sFaces);
  SER_MSG(sFaces.GetSize() > 0 ? SM_SUCCESS : SM_ERR,
          _T("Primitive cone did not create Brep faces"));

  SER(my_verify_brep_convert_to_polybrep_face_normal_table(pBrep));

  return SM_SUCCESS;

} // end my_test_cone_face_normal_table

/*****************************************************
PURPOSE ---

USAGE NOTES ---
*****************************************************/
PT_EXPORT SmStatus my_test_tess()
{
  MYPRINTF(_T("\nEntered: my_test_tess")) ;

  if (TRUE)
    {
      MYPRINTF(_T("\n my_test_tess: face/normal table regression")) ;
      SER(my_test_polybrep_glue_preserves_face_normal_table());
      SER(my_test_cone_face_normal_table(0.0, 360.0));
      SER(my_test_cone_face_normal_table(30.0, 180.0));
    }

  if (TRUE) 
    {
      SER( my_test_poly_solver_1() );
      SER( my_test_poly_solver_2() );
      SER( my_test_poly_solver_3() );
      SER( my_test_poly_solver_4() );
    }

  if (TRUE) 
    { // Test CreateTriStripFaces
      MYPRINTF(_T("\n my_test_tess: test 1")) ;
      SmContext sContext;
      SmPolyBrep * pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
      SmObjDelete sClean2(pPolyBrep);
      SmTArray<SmPoint3d> sPnts;
      sPnts.Add(SmPoint3d(0,1,0));
      sPnts.Add(SmPoint3d(0,0,0));
      sPnts.Add(SmPoint3d(1,1,0));
      sPnts.Add(SmPoint3d(1,0,0));
      sPnts.Add(SmPoint3d(2,1,0));
      sPnts.Add(SmPoint3d(2,0,0));
      sPnts.Add(SmPoint3d(3,1,0));
      sPnts.Add(SmPoint3d(3,0,0));
      sPnts.Add(SmPoint3d(4,1,0));
      sPnts.Add(SmPoint3d(4,0,0));
      SER(pPolyBrep->CreateTriStripFaces(sPnts));
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pPolyBrep) ;

          smgfx_Erase();
          if (pPolyBrep) { pPolyBrep->Draw(); sm_GraphicsLoop(); }
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
    }

  if (TRUE) 
    { // Test Simple Merge
      MYPRINTF(_T("\n my_test_tess: test 2")) ;
      SmContext sContext;
      SmPolyBrep * pFromPolyBrep = NULL;
      SER(SmPolyBrep::ReadFromSTLFile(sContext,_T("../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"),
                                      pFromPolyBrep,SM_ASCII));
      SmObjDelete sClean1(pFromPolyBrep);
      SmPolyBrep * pToPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
      SmObjDelete sClean2(pToPolyBrep);
      SmTArray<SmPolyFace*> sFacesToMove;
      pFromPolyBrep->GetPolyFaces(sFacesToMove);
      SER(pFromPolyBrep->SimpleMerge(sFacesToMove,pToPolyBrep));

#ifdef SM_GFX_CODE
      SM_ASSERT_VALID(pToPolyBrep) ;
      SM_ASSERT_VALID(pFromPolyBrep) ;

      if(smGet_DoGraphics())
        {
          smgfx_Erase();
          smgfx_SetLook(1,2) ; if(pFromPolyBrep) pFromPolyBrep->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(3,4) ; if(pToPolyBrep) pToPolyBrep->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
    }

  if (TRUE) 
    { // Test STL input
      MYPRINTF(_T("\n my_test_tess: test 3")) ;
      SmContext sContext;
      SmPolyBrep * pPolyBrep = NULL;
      SER(SmPolyBrep::ReadFromSTLFile(sContext,_T("../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"),
                                      pPolyBrep,SM_ASCII,FALSE,1));
      SmObjDelete sClean1(pPolyBrep);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pPolyBrep) ;

          smgfx_Erase();
          smgfx_SetLook(1,2) ; if(pPolyBrep) pPolyBrep->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
    }

  if (TRUE) 
    {
      // This first example shows how to stitch together faces from a set of
      // trimmed surfaces to create a solid prior to tessellating it.  
      // Note that all of the trimmed surfaces need to live in the same
      // brep prior to stitching.  See SmBrep::MakeFaceWithCurves, 
      // SmBrep::CreateFaceFromSurface, and SmTrimmingTools methods for
      // ways to create faces in a SmBrep programatically.  Examples are
      // in the tutorial.
      
      MYPRINTF(_T("\n my_test_tess: test 4")) ;
      SmContext sContext;

      SmBrep* pBrep = new(sContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/fillet_ts.smb"));
      pBrep->ReadFromFile( sContext, _T("../../TestFiles/pt_TestFiles/Solids/fillet_ts.smb"), SM_ASCII );

      pBrep->SetTolerance(0.0001);
      SmObjDelete sCleanBrep(pBrep);
      pBrep->m_bEditingEnabled = TRUE;
      
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_DUMP_AND_ASSERT_VALID(pBrep) ;

          smgfx_Erase();
          smgfx_SetLook(1,2) ; if(pBrep) pBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

//        SE(pBrep->StitchFaces(0.001,lNumStitched,lNumLamina,dMaxVGap,dMaxEGap));
      ULONG lNumLamina;
      SER(my_trimmed_surface_stitch(pBrep,0.001,lNumLamina));

      pBrep->m_bEditingEnabled = FALSE;
      
      // Note that 0.001 is not large enough to stitch all of the faces
      // and that there will still be lamina edges - so we stitch again
      // with a larger tolerance.
      if (lNumLamina > 0) {
          pBrep->m_bEditingEnabled = TRUE;
          SER(my_trimmed_surface_stitch(pBrep,0.005,lNumLamina));
          pBrep->m_bEditingEnabled = FALSE;
      }
      
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_DUMP_AND_ASSERT_VALID(pBrep) ;

          smgfx_Erase();
          smgfx_SetLook(1,2) ; if(pBrep) pBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      double dChordHeightTol = 0.6;  // Maximum chord height of polygon
      // from original geometry.
      double dAngleTolInDegrees = 40.0; // Maximum angle in U or V that
      // the polygon will span.
      ULONG lMinCurveDivisions = 0; // Don't use this tolerance
      double dMax3DEdgeLength = 4.0; // Maximum length of an edge
      double dMin3DEdgeLength = 0.0; // Minimum length of an edge - not used
      double dMaxAspectRatio = 4.0; // 4:1 aspect ratio
      double dMinUVRatio = 0.001; // Smallest subdivision of a surface is 
      // 1/1000 of the surface parameter space size in U and V.
      
      // Create a curve tessellation driver and a surface tessellation
      // driver to control how subdivision is performed.  Note that
      // you can subclass these method to specialize how tessellation
      // performs the subdivision.  For example you may want to have
      // a view specific tessellation that subdivides based on the
      // view volume and eye position.  
      SmCurveTessDriver sCrvTess(lMinCurveDivisions,
                                 dChordHeightTol,
                                 dAngleTolInDegrees);
      SmSurfaceTessDriver sSrfTess(dChordHeightTol,
                                   dAngleTolInDegrees,
                                   dMax3DEdgeLength,
                                   dMin3DEdgeLength,
                                   dMinUVRatio,
                                   dMaxAspectRatio);
      
      SmTess sTess(sContext,sCrvTess,sSrfTess);
      // Note that the Brep sent into the Tessellator gets modified
      // so you usually will want to make a copy of it is follows:
      SmBrep *pCopy = new (sContext) SmBrep(*pBrep);
      
      // Now do the tessellation process to produce triangles in UV
      // space of the surfaces.
      SmBoolean rbFailedFaces;
      SER(sTess.DoTessellation(pCopy,rbFailedFaces));
      SM_ASSERT(rbFailedFaces == FALSE);
      
      // Output polygons converts to 3D polygons and calls my_output_triangle
      s_lNumberTriangles = 0;
      SmPolygonOutputPrinter sPolyPrinter;
      SER(sTess.OutputPolygons(sPolyPrinter));

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SmPolygonOutputCallback sOutputPolyBrep ;
          sOutputPolyBrep.SetOutputType(SM_PO_CREATE_POLYBREP) ;
          sTess.OutputPolygons(sOutputPolyBrep) ;
          SmPolyBrep *pPolyBrep = sTess.Get3DPolyBrep() ;

          // SM_ASSERT_VALID(pBrep) ;
          SM_ASSERT_VALID(pCopy) ;
          SM_ASSERT_VALID(pPolyBrep) ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(2,3, 0,1,1) ; if(pCopy) pCopy->Draw(TRUE); sm_GraphicsLoop();
          smgfx_SetLook(4,5, 1,0,0) ; if(pPolyBrep) pPolyBrep->Draw(TRUE,TRUE,FALSE,NULL,NULL,SM_ET_LAMINA) ; sm_GraphicsLoop();
          smgfx_SetLook(2,3) ; if(pPolyBrep) pPolyBrep->Draw(TRUE,TRUE,FALSE,NULL,NULL,SM_ET_MANIFOLD) ; sm_GraphicsLoop();
          smgfx_SetLook(1,2) ; if(pPolyBrep) pPolyBrep->Draw(TRUE,TRUE,FALSE,NULL,NULL,SM_ET_ALL) ; sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
      
      // Here we are done.  Note that if you declare SmTess on the stack
      // it will automatically clean up the pCopy SmBrep and all of it's internal
      // structures when it goes out of scope.  pBrep is also cleaned when its
      // SmObjDelete sCleanBrep goes out of scope.
    }

  if (TRUE) 
    { // Test tessellation with angle tolerance
      MYPRINTF(_T("\n my_test_tess: test 5")) ;
      SmContext sContext;

      SmBrep* pBrep1 = new(sContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/nurb_fillet.smb"));
      pBrep1->ReadFromFile( sContext, _T("../../TestFiles/pt_TestFiles/Solids/nurb_fillet.smb"), SM_ASCII );
      SmObjDelete sClean1(pBrep1);
      SmBoolean bFailed;
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pBrep1) ;

          smgfx_Erase();
          smgfx_SetLook(1,2) ; if(pBrep1) pBrep1->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      my_test_one_tess(pBrep1,
                       0.0,
                       30.0,
                       0.0,
                       0.0,
                       bFailed);

      SM_ASSERT(bFailed==FALSE);
    }
  
  if (TRUE) 
    { // Test tessellation with angle tolerance
      MYPRINTF(_T("\n my_test_tess: test 6")) ;
      SmContext sContext;
      SmBrep* pBrep1 = new(sContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/nurb_fillet.smb"));
      pBrep1->ReadFromFile( sContext, _T("../../TestFiles/pt_TestFiles/Solids/nurb_fillet.smb"), SM_ASCII );
      SmObjDelete sClean1(pBrep1);
      SmBoolean bFailed;
      my_test_one_tess(pBrep1,
                       0.2,
                       30.0,
                       0.0,
                       0.0,
                       bFailed);
      SM_ASSERT(bFailed==FALSE);
    }
  
  if (TRUE) 
    { // Test tessellation with angle tolerance
      MYPRINTF(_T("\n my_test_tess: test 7")) ;
      SmContext sContext;
      SmBrep* pBrep1 = new(sContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/nurb_fillet.smb"));
      pBrep1->ReadFromFile( sContext, _T("../../TestFiles/pt_TestFiles/Solids/nurb_fillet.smb"), SM_ASCII );
      SmObjDelete sClean1(pBrep1);
      SmBoolean bFailed;
      my_test_one_tess(pBrep1,
                       0.0,
                       30.0,
                       3.0,
                       0.0,
                       bFailed);
      SM_ASSERT(bFailed==FALSE);
    }
  
  if (TRUE) 
    { // Test tessellation with angle tolerance
      MYPRINTF(_T("\n my_test_tess: test 8")) ;
      SmContext sContext;
      SmBrep* pBrep1 = new(sContext) SmBrep();
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/nurb_fillet.smb"));
      pBrep1->ReadFromFile( sContext, _T("../../TestFiles/pt_TestFiles/Solids/nurb_fillet.smb"), SM_ASCII );
      SmObjDelete sClean1(pBrep1);
      SmBoolean bFailed;
      my_test_one_tess(pBrep1,
                       0.0,
                       30.0,
                       0.0,
                       3.0,
                       bFailed);
      SM_ASSERT(bFailed==FALSE);
    }
  
  if (TRUE) 
    { // Test input/output for poly brep
      MYPRINTF(_T("\n my_test_tess: test 9")) ;
      SmContext sContext;
      SmPolyBrep * pPolyBrep = NULL;
      SER(SmPolyBrep::ReadFromSTLFile(sContext,_T("../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"), pPolyBrep,SM_ASCII));
      SmObjDelete sClean1(pPolyBrep);
#ifdef SM_GFX_CODE
      if(pPolyBrep) { SM_ASSERT_VALID(pPolyBrep); }

      if(smGet_DoGraphics())
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep) pPolyBrep->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
      pPolyBrep->WriteToFile(_T("../prog_test/OutputFiles/write_poly_file_test.pbp"),SM_ASCII);
      SmPolyBrep * pPolyBrep2 = new (sContext) SmPolyBrep(SM_EFF_ZERO);
      pPolyBrep2->ReadFromFile(sContext,_T("../prog_test/OutputFiles/write_poly_file_test.pbp"),SM_ASCII);
      SmObjDelete sClean2(pPolyBrep2);
      pPolyBrep2->WriteToFile(_T("../prog_test/OutputFiles/write_poly_file_test1.pbp"),SM_ASCII);
#ifdef SM_GFX_CODE
     if(smGet_DoGraphics())
       {
          SM_ASSERT_VALID(pPolyBrep2) ;

          smgfx_SetLook(3,4, 0,1,0) ; if(pPolyBrep2) pPolyBrep2->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      // cleanup created files
      SM_REMOVE( _T("../prog_test/OutputFiles/write_poly_file_test.pbp") );
      SM_REMOVE( _T("../prog_test/OutputFiles/write_poly_file_test1.pbp") );
    }

  if (TRUE) 
    { // Test ray-firing for poly brep
      MYPRINTF(_T("\n my_test_tess: test 10")) ;
      SmContext sContext;
      SmPolyBrep * pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"));
      pPolyBrep->ReadFromFile(sContext,_T("../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"),SM_ASCII);
      SmObjDelete sClean1(pPolyBrep);
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pPolyBrep) ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep) pPolyBrep->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      SmExtent3d sBBox;
      pPolyBrep->CalculateBoundingBox(sBBox);
      sBBox.ExpandAbsolute(1.0);
      ULONG lSize[3];
      lSize[0] = 20;
      lSize[1] = 20;
      lSize[2] = 20;
      SmGrid *pGrid = new (sContext) SmGrid(sBBox,lSize);
      SmRayTracer sRayTracer(sContext,pGrid,SM_ZONE_TOL_3D,1.0e-8);
      SER(sRayTracer.AddPolyBrepToGrid(pPolyBrep,0.001));
      double sDData[256];
      double sDData2[256];
      SmTArray<double> sMinDistances(256,sDData);
      SmTArray<double> sMaxDistances(256,sDData2);
      SmGridElement *sEData[256];
      SmSArray<SmGridElement*> sGridElements(256,sEData);
      SmSolution sSData[16];
      SmSolutionArray sSolutions(16,sSData);
      SmSolution sSolution;
      SmBoolean bHitsSomething;
      SmPoint3d sRayPoint = sBBox.Evaluate(0.5,0.5,0.5);
      SmVector3d sRayVector(1.0,1.0,1.0);
      sRayVector.Unitize();
      sRayPoint = sRayPoint-sRayVector*15.0;
      sRayTracer.m_bHitAnyThing = FALSE;
      SER(sRayTracer.FireRay(sRayPoint,sRayVector,40.0,//SM_BIG_DOUBLE,
          bHitsSomething,sSolution,
          sGridElements,sMinDistances,sMaxDistances));
      SM_ASSERT(bHitsSomething == TRUE);
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SmVector3d sDir = sRayVector * sSolution.m_vStart[0];
          SmPoint3d sPntEnd = sRayPoint + sDir;
          smgfx_SetLook(2,3, 1,0,0);  sRayPoint.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 1,0,0);  sDir.Draw(&sRayPoint); sm_GraphicsLoop();
          smgfx_SetLook(2,3, 0,0,1); sPntEnd.Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
    }
  
  if (TRUE) 
    { // Test sectioning for poly brep
      MYPRINTF(_T("\n my_test_tess: test 11")) ;
      SmContext sContext;
      SmPolyBrep * pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"));
      pPolyBrep->ReadFromFile(sContext,_T("../../TestFiles/pt_TestFiles/Tess/Extrusion.pbp"),SM_ASCII);
      SmObjDelete sClean1(pPolyBrep);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pPolyBrep) ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep) pPolyBrep->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      SmPoint3d sPlaneOrig(6.0,0.0,0.0);
      SmVector3d sPlaneNormal(1.0,0.0,0.0);
      SmTArray<SmPoint3d> sLineSegPnts;
      SmTArray<SmPoint3d> sTangentSegPnts;
      SER(pPolyBrep->DoSectioning(sPlaneOrig,
                                  sPlaneNormal,
                                  SM_EFF_ZERO,
                                  &sLineSegPnts,
                                  &sTangentSegPnts));
//        SmPolyBrep * pSectionBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
//        pSectionBrep->MakeLoopsFromPoints(sLineSegPnts,SM_EFF_ZERO);
//        pSectionBrep->MakeLoopsFromPoints(sTangentSegPnts,SM_EFF_ZERO);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          ULONG i0, i1 ;
          SM_ASSERT_VALID(pPolyBrep) ;
          sLineSegPnts.Dump() ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,1,1) ; sPlaneNormal.DrawPlane(sPlaneOrig) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep) pPolyBrep->Draw() ; sm_GraphicsLoop();
          smgfx_SetLook(2,3, 1,0,0) ; for(i0=0,i1=1;i0<sLineSegPnts.GetSize();i0+=2,i1+=2)
                                        { if(i1 == sLineSegPnts.GetSize()) { i1 = 0 ; }
                                          (sLineSegPnts[i1]-sLineSegPnts[i0]).Draw(&sLineSegPnts[i0]) ; sm_GraphicsLoop() ;
                                        }
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      SM_ASSERT(sLineSegPnts.GetSize() > 0);
    }

  if (TRUE) 
    { // test TrimWithPlane
      MYPRINTF(_T("\n my_test_tess: test 12")) ;
      SmContext sContext;
      SmPolyBrep * pPolyBrep = NULL;
      if (1) 
        {
          SER(my_brepfile_to_polybrep(sContext,_T("../../TestFiles/pt_TestFiles/Solids/extrusion_old.smb"),
              0.0,5.0, 15.0,pPolyBrep));
        }
      else 
        {
          SER(SmPolyBrep::ReadFromSTLFile(sContext,_T("../../TestFiles/pt_TestFiles/Tess/BoxFillet.stl"),
                                          pPolyBrep,SM_ASCII));
        }
      SmObjDelete sClean1(pPolyBrep);
//        SmPoint3d sPoint(26,0,0);//BoxFillet.stl
#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pPolyBrep) ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep) pPolyBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

      SmPoint3d sPoint(4.6,0,0);//extrusion_old.stl
      SmVector3d sNormal(-1,0,0);
      SmPolyBrep * pResult = NULL;
      SER(pPolyBrep->TrimWithPlane(sPoint,sNormal,TRUE,pResult));

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pPolyBrep) ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,1,1) ; sNormal.DrawPlane(sPoint) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 0,1,0) ; if(pPolyBrep) pPolyBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

    }

  if (TRUE) 
    { // Test oriented stitching
      MYPRINTF(_T("\n my_test_tess: test 13")) ;
      SmContext sContext;
      SmPolyBrep * pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/SewSolidBefore.pbp"));
      pPolyBrep->ReadFromFile(sContext,_T("../../TestFiles/pt_TestFiles/Tess/SewSolidBefore.pbp"),SM_ASCII);
      SmObjDelete sClean1(pPolyBrep);
      ULONG lNumEdgesStitched;
      ULONG lNumberLaminaRemaining;

#ifdef SM_GFX_CODE
      SM_ASSERT_VALID(pPolyBrep) ;
#endif // SM_GFX_CODE

      SER(pPolyBrep->OrientedStitch(pPolyBrep->GetTolerance(),TRUE,lNumEdgesStitched,lNumberLaminaRemaining)); // note: increments unlocked mark value
      SM_ASSERT(lNumberLaminaRemaining == 0);

#ifdef SM_GFX_CODE
      if(smGet_DoGraphics())
        {
          SM_ASSERT_VALID(pPolyBrep) ;

          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep) pPolyBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE
    }

  if (TRUE) 
    { // Test oriented stitching
      MYPRINTF(_T("\n my_test_tess: test 14")) ;
      SmContext sContext;
      SmPolyBrep * pPolyBrep = new (sContext) SmPolyBrep(SM_EFF_ZERO);
      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/OrientedSewWSplit.pbp"));
      pPolyBrep->ReadFromFile(sContext,_T("../../TestFiles/pt_TestFiles/Tess/OrientedSewWSplit.pbp"),SM_ASCII);
      SmObjDelete sClean1(pPolyBrep);
      ULONG lNumEdgesStitched;
      ULONG lNumberLaminaRemaining;
      SER(pPolyBrep->OrientedStitch(pPolyBrep->GetTolerance(),TRUE,lNumEdgesStitched,lNumberLaminaRemaining)); // note: increments unlocked mark value
      SM_ASSERT(lNumberLaminaRemaining == 0);

#ifdef SM_GFX_CODE
      if(pPolyBrep) { SM_ASSERT_VALID(pPolyBrep); }

      if(smGet_DoGraphics())
        {
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep) pPolyBrep->Draw(TRUE); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_GFX_CODE

    }

  if (TRUE) 
    { // test decimation regression
      MYPRINTF(_T("\n my_test_tess: test 15 - my_test_decimation_regression")) ;
      SER(my_test_decimation_regression());
    }

  if (TRUE) 
    { // test boolean regression
      MYPRINTF(_T("\n my_test_tess: test 16 - my_poly_boolean_regression")) ;
      SER(my_poly_boolean_regression());
    }

 return SM_SUCCESS;

} // end my_test_tess

/*****************************************************
PURPOSE ---  Exercise Boolean operation on PolyBrep inputs

USAGE NOTES ---
*****************************************************/
SmStatus my_test_ppu_poly
 (const SmContext & crContext,
  SmPolyBrep      * pPolyBrep1,
  SmPolyBrep      * pPolyBrep2,
  ULONG             lOperation,    // 0 - union, 1 - intersection, 2 - difference, 3 - merge
  SmPolyBrep     *& rpResult)
{
  MYPRINTF(_T("\n   Entered: my_test_ppu_poly")) ;

  double dAngleTol = 20.0*SM_PI/180.0;
  double dTol      = smos_Max(pPolyBrep1->GetTolerance(),pPolyBrep2->GetTolerance());

  // Note comment this out for debugging where you need the true values
  // Center the world space
  
  // get bounding box for both PolyBreps 
  SmExtent3d sBBox, sBBox2;
  pPolyBrep1->CalculateBoundingBox(sBBox);
  pPolyBrep2->CalculateBoundingBox(sBBox2);
  sBBox.Union(sBBox2, sBBox);

  // move the PolyBreps so that their bounding box center is at the origin
  SmBoolean bTrans  = FALSE;
  SmPoint3d sCenter = sBBox.Evaluate(0.5, 0.5, 0.5);
  if (sCenter.LengthSquared() > 1.0e7) 
    { 
      bTrans = TRUE;
      sCenter = -sCenter;
      SmAxis2Placement sRef;
      sRef.Translate(sCenter);
      pPolyBrep1->Transform(sRef);
      pPolyBrep2->Transform(sRef);
    }

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
    {
      SM_ASSERT_VALID(pPolyBrep1) ;
      SM_ASSERT_VALID(pPolyBrep2) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep1) pPolyBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pPolyBrep2) pPolyBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }        
#endif // SM_GFX_CODE
   
  // build merge object
  SmPolyMerge sPolyMerge(crContext,pPolyBrep1,pPolyBrep2,dTol,dAngleTol);
  sPolyMerge.SetRemoveCoPlanarEdges(FALSE);

  // switch on the operation type
  switch(lOperation)
    {
      case 0 : SER(sPolyMerge.ManifoldBoolean(SM_PBO_UNION,rpResult));
               break ;
      case 1 : SER(sPolyMerge.ManifoldBoolean(SM_PBO_INTERSECTION,rpResult));
               break ;
      case 2 : SER(sPolyMerge.ManifoldBoolean(SM_PBO_DIFFERENCE,rpResult));
               break ;
      case 3 : SER(sPolyMerge.ManifoldBoolean(SM_PBO_MERGE,rpResult));
               break ;
      default: SER(SM_ERR);
    } // end switch on lOperation

  // Transform the result back into original 3d space position
  if (bTrans) 
    {
      sCenter = -sCenter;
      SmAxis2Placement sRef2;
      sRef2.Translate(sCenter);
      rpResult->Transform(sRef2);
    }

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
    {
      SM_ASSERT_VALID(rpResult) ;

      smgfx_Erase() ;
      smgfx_SetLook(2,3, 1,0,0) ; if(rpResult) rpResult->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }        
#endif // SM_GFX_CODE

  return SM_SUCCESS;

} // end my_test_ppu_poly

/*****************************************************
PURPOSE --- Test function for polygon booleans from Breps

USAGE NOTES --- 1. Convert input Breps to PolyBreps
                2. 
*****************************************************/
SmStatus my_test_ppu
  (const SmContext & crContext,
  SmBrep           * pBrep1,
  SmBrep           * pBrep2,
  ULONG              lOperation,        // 0 - union, 1 - intersection, 2 - difference, 3 - merge
  SmPolyBrep      *& rpResult)
{
  // locals
  double       dCHTol        = 0.1;
  double       dCrvTessAngle = 5.0;
  double       dSrfTessAngle = 15.0;
  double       dMax3DEdge    = 0.0;
  double       dMaxAspect    = 0.0;
  SmBoolean    bFailedFaces;
  SmPolyBrep * pPolyBrep1 = NULL;
  SmPolyBrep * pPolyBrep2 = NULL;
  ULONG        lNumLamina;

  // Create pPolyBrep1 from pBrep1  
  SER( pBrep1->ConvertToPolyBrep
  (
       pPolyBrep1,
       bFailedFaces,
       lNumLamina,
       dCHTol,
       dCrvTessAngle,
       dSrfTessAngle,
       dMax3DEdge,
       dMaxAspect,
       FALSE,
       FALSE,
       FALSE
  ) );

  if (bFailedFaces || lNumLamina != 0) SER(SM_ERR);

  // Create pPolyBrep2 from pBrep2  
  SER( pBrep2->ConvertToPolyBrep
  (
       pPolyBrep2,
       bFailedFaces,
       lNumLamina,
       dCHTol,
       dCrvTessAngle,
       dSrfTessAngle,
       dMax3DEdge,
       dMaxAspect,
       FALSE,
       FALSE,
       FALSE
  ) );

  if (bFailedFaces || lNumLamina != 0) SER(SM_ERR);

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
    {
      SM_ASSERT_VALID(pBrep1)     ;
      SM_ASSERT_VALID(pPolyBrep1) ;
      SM_ASSERT_VALID(pBrep2)     ;
      SM_ASSERT_VALID(pPolyBrep2) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;

      smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep1) pPolyBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pPolyBrep2) pPolyBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep1) pPolyBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pPolyBrep2) pPolyBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;

      sm_GraphicsLoop() ;
    }        
#endif // SM_GFX_CODE

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      SM_ASSERT_VALID(pPolyBrep1) ;
      SM_ASSERT_VALID(pPolyBrep2) ;

      smgfx_Erase();
      smgfx_SetLook(1,2, 0,1,0); pPolyBrep1->Draw(); sm_GraphicsLoop(); 
      smgfx_SetLook(1,2, 0,0,1); pPolyBrep2->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

   // build the SmPolyMerge object
  double dTol = smos_Max(pPolyBrep1->GetTolerance(),pPolyBrep2->GetTolerance());
  SmPolyMerge sPolyMerge(crContext,pPolyBrep2,pPolyBrep1,dTol,dSrfTessAngle);

  SER(sPolyMerge.CleanupCoPlanarFaces());
#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
    {
      SM_ASSERT_VALID(pPolyBrep1) ;
      SM_ASSERT_VALID(pPolyBrep2) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep1) pPolyBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pPolyBrep2) pPolyBrep2->Draw(TRUE) ; sm_GraphicsLoop() ;

      sm_GraphicsLoop() ;
    }        
#endif // SM_GFX_CODE

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      SM_ASSERT_VALID(pPolyBrep1) ;
      SM_ASSERT_VALID(pPolyBrep2) ;

      smgfx_Erase();
      smgfx_SetLook(2,3, 0,1,0); pPolyBrep1->Draw(); sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,0,1); pPolyBrep2->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // PolyBoolean
  switch(lOperation)
    {
      case 0: SER(sPolyMerge.ManifoldBoolean(SM_PBO_UNION,rpResult));
              break;
      case 1: SER(sPolyMerge.ManifoldBoolean(SM_PBO_INTERSECTION,rpResult));
              break;
      case 2: SER(sPolyMerge.ManifoldBoolean(SM_PBO_DIFFERENCE,rpResult));
              break;
      case 3: SER(sPolyMerge.ManifoldBoolean(SM_PBO_MERGE,rpResult));
              break;
      default: SER(SM_ERR);
    
    } // end switch on lOperation

#ifdef SM_GFX_CODE
  if(smGet_DoGraphics())
    {
      SM_ASSERT_VALID(rpResult) ;

      smgfx_Erase() ;
      smgfx_SetLook(2,3, 1,0,0) ; if(rpResult) rpResult->Draw(TRUE) ; sm_GraphicsLoop() ;

      sm_GraphicsLoop() ;
    }        
#endif // SM_GFX_CODE

  // all done
  return SM_SUCCESS;

} // end my_test_ppu

                           

/*****************************************************
PURPOSE ---  exercise Poly Boolean on parts nearly coincident

USAGE NOTES ---
*****************************************************/
SmStatus my_box_box_stress
 (const SmContext & crContext,
  SmTArray<SmPolyBrep*> & rPolyBreps)
{

  if (TRUE) 
    { // Moving two boxes to being coincident
      MYPRINTF(_T("\n    my_box_box_stress: test 1")) ;
      double dMove = 0.000999;
      for (ULONG i=0; i<6; i++) 
        {
          SmPolyBrep      *pResult;
          SmPolyBrep      *pCube1, *pCube2;
          SmAxis2Placement sPos;

          // build box 1
          SER(SmPolyBrep::CreateBox(crContext,SM_ZONE_TOL_3D/10.0,1,1,1,sPos,pCube1));
          SmVector3d sMove(dMove,0.0,0.5);
          sPos.Translate(sMove);
          dMove = dMove / 10.0;

          // build box 2  - slightly moved
          SER(SmPolyBrep::CreateBox(crContext,SM_ZONE_TOL_3D/10.0,1,1,1,sPos,pCube2));
          double dAngleTol = 20.0*SM_PI/180.0;

          // PolyBrep Boolean difference
          SmPolyMerge sMerge(crContext,pCube1,pCube2,pCube1->GetTolerance(),dAngleTol);
          SER(sMerge.ManifoldBoolean(SM_PBO_DIFFERENCE,pResult));

          // move the result to a unique location
          SmAxis2Placement sPos2;
          SmVector3d sTrans((i+1.0)*3.0,0.0,0.0);
          sPos2.Translate(sTrans);
          pResult->Transform(sPos2);

          // informat the world
          SM_ASSERT_VALID(pResult) ;
#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
            {
              if(i==0) 
                { smgfx_Erase() ; }
              if (pResult) { pResult->Draw(TRUE); sm_GraphicsLoop(); }
              sm_GraphicsLoop() ;
            }        
#endif // SM_GFX_CODE
          rPolyBreps.Add(pResult);
        }
      return SM_SUCCESS;
    }

  if (TRUE) 
    { // Moving two boxes to being coincident
      MYPRINTF(_T("\n    my_box_box_stress: test 2")) ;
      double dMove = 0.000999;
      for (ULONG i=0; i<6; i++) 
        {
          SmPolyBrep *pResult;
          SmPolyBrep *pCube1, *pCube2;
          SmAxis2Placement sPos;

          // build box 1
          SER(SmPolyBrep::CreateBox(crContext,SM_ZONE_TOL_3D/10.0,1,1,1,sPos,pCube1));
          SmVector3d sMove(dMove,0.0,0.5);
          sPos.Translate(sMove);
          dMove = dMove / 10.0;

          // build box2 nearby
          SER(SmPolyBrep::CreateBox(crContext,SM_ZONE_TOL_3D/10.0,1,1,1,sPos,pCube2));
          double dAngleTol = 20.0*SM_PI/180.0;

          // boolean union
          SmPolyMerge sMerge(crContext,pCube1,pCube2,pCube1->GetTolerance(),dAngleTol);
          SER(sMerge.ManifoldBoolean(SM_PBO_UNION,pResult));

          // move the result to a unique place
          SmAxis2Placement sPos2;
          SmVector3d sTrans((i+1.0)*3.0,0.0,0.0);
          sPos2.Translate(sTrans);
          pResult->Transform(sPos2);

          // inform the public
          SM_ASSERT_VALID(pResult) ;

#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
            {
              if(i==0) 
                { smgfx_Erase() ; }
              if (pResult) { pResult->Draw(TRUE); sm_GraphicsLoop(); }
              sm_GraphicsLoop() ;
            }        
#endif // SM_GFX_CODE

          rPolyBreps.Add(pResult);
        }
      return SM_SUCCESS;
    }

  if (TRUE) 
    { // Moving two boxes to being coincident
      MYPRINTF(_T("\n    my_box_box_stress: test 3")) ;
      double dMove = 0.0001;
      for (ULONG i=0; i<6; i++) 
        {
          SmPolyBrep *pResult;
          SmPolyBrep *pCube1, *pCube2;
          SmAxis2Placement sPos;

          // build box 1
          SER(SmPolyBrep::CreateBox(crContext,SM_ZONE_TOL_3D/10.0,1,1,1,sPos,pCube1));
          SmVector3d sMove(dMove,0.0,0.0);
          sPos.Translate(sMove);
          dMove = dMove / 10.0;

          // build box 2 nearby
          SER(SmPolyBrep::CreateBox(crContext,SM_ZONE_TOL_3D/10.0,1,1,1,sPos,pCube2));
          double dAngleTol = 20.0*SM_PI/180.0;

          // Boolean union
          SmPolyMerge sMerge(crContext,pCube1,pCube2,pCube1->GetTolerance(),dAngleTol);
          SER(sMerge.ManifoldBoolean(SM_PBO_UNION,pResult));

          // place result in unique position
          SmAxis2Placement sPos2;
          SmVector3d sTrans((i+1.0)*3.0,0.0,0.0);
          sPos2.Translate(sTrans);
          pResult->Transform(sPos2);

          // inform the public
          SM_ASSERT_VALID(pResult) ;

#ifdef SM_GFX_CODE
          if(smGet_DoGraphics())
            {
              if(i==0) 
                { smgfx_Erase() ; }
              if (pResult) { pResult->Draw(TRUE); sm_GraphicsLoop(); }
              sm_GraphicsLoop() ;
            }        
#endif // SM_GFX_CODE

          rPolyBreps.Add(pResult);
        }
      return SM_SUCCESS;
    }

  return SM_SUCCESS;

} // end my_box_box_stress

/*****************************************************
PURPOSE ---

USAGE NOTES ---
*****************************************************/
SmStatus my_test_decimation
( const SmContext & crContext,              // in : context for new object construction
 const TCHAR     * pFileName,              // in : target file
 double            dTessellationChTol,     // in : 
 double            dCrvAngleTolDeg,        // in : 
 double            dSrfAngleTolDeg,        // in : 
 double            dPercentReduction,      // in : 
 double            dMaximumReductionError, // in : 
 ULONG             lStage,                 // in : 1 - Just Read in Triangles
                                           //      2 - Decimate Triangles,
                                           //      3 - Both read and decimate
 SmBoolean         bQuadricDecimation,     // in :
 SmTArray<SmPolyBrep*> & rBreps )          // out: if lStage == 1 or 3, from pFileName
                                           // in : if lStage == 2
{
  // when asked - 1. read pBrep, 
  //              2. pBrep->StitchFaces(), 
  //              3. Brep_to_STL_Conversion (gets tessellated and written), 
  //              4. SmPolyBrep::ReadFromSTLFile
  if ( lStage == 1 || lStage == 3 )
  {
    SmBrep* pBrep = NULL;

    // read smb files
    if(smos_WStrStr( pFileName, _T( ".smb" ) ) || smos_WStrStr( pFileName, _T( ".SMB" ) ))
    {
      // read the Brep file
      pBrep = new(crContext) SmBrep();
      SER( pBrep->ReadFromFile( crContext, pFileName, SM_ASCII, FALSE ) );
    } // end .smb file read branch

    // read part files
    else if(smos_WStrStr( pFileName, _T( ".smp" ) ) || smos_WStrStr( pFileName, _T( ".SMP" ) ))
    {
      // read the part file   
      SmTArray<SmBrep*> sBreps;
      SmTArray<SmCurve*> s3DCurves;
      SmTArray<SmSurface*> sSurfaces;
      SmTArray<long> sBooleanTrees;
      SmBrepData::ReadPartFromFile( crContext, pFileName,
                                   s3DCurves, sSurfaces,
                                   sBooleanTrees, sBreps, SM_ASCII );
      // merge all the breps into one 
      pBrep = sBreps[0];
      for(ULONG jj = 1; jj < sBreps.GetSize(); jj++)
      {
        SmBrep* pOtherBrep = sBreps[jj];
        SER( pBrep->MergeBrep( *pOtherBrep ) );
      }
    } // end .smp file read branch
    SmObjDelete sCleanBrep( pBrep );

    // stitch faces
    ULONG  lNumStitched, lNumLamina;
    double dMaxVGap, dMaxEGap;
    pBrep->m_bEditingEnabled = TRUE;
    double dThisApproxTol3d = SM_ZONE_TOL_3D;
    SER( pBrep->StitchFaces( dThisApproxTol3d, lNumStitched, lNumLamina, dMaxVGap, dMaxEGap ) );
    pBrep->m_bEditingEnabled = FALSE;

    // write pBrep to STL (gets tessellated)
    double dCHTol = dTessellationChTol;
    double dMax3DEdge = 0.0;
    double dMaxAspect = 0.0;

    // Output breps to .stl files
    // WE hard code ascii for now
    SmPolyBrep * pPolyBrep = NULL;
    SmBoolean fFaces = 0;

    pBrep->ConvertToPolyBrep(
      pPolyBrep,
      fFaces,
      lNumLamina,
      dCHTol,
      dCrvAngleTolDeg,
      dSrfAngleTolDeg,
      dMax3DEdge,
      dMaxAspect,
      FALSE,
      TRUE,
      FALSE
    );

    // inform the public
    SmTArray<SmPolyFace*> sPolyFaces;
    pPolyBrep->GetPolyFaces( sPolyFaces );
    ULONG lStartFaces = sPolyFaces.GetSize();
    TCHAR sBuff[SM_TBLOCK_SIZE];
    SM_SPRINTF( sBuff, _T( "Original PolyBrep # Triangles = %ld\n" ), lStartFaces );
    smos_WriteBuffer( sBuff );

    rBreps.Add( pPolyBrep );

  } // end asked to read .smb or .smp files and build pPolyBrep check

// when asked - decimate the triangles
  if ( lStage == 2 || lStage == 3 )
  {
    // check input - no PolyBreps to decimate
    if ( rBreps.GetSize() < 1 )
    {
      SER( SM_ERR );
    }

    SmPolyBrep *pPolyBrep = rBreps.GetLast();

#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      SM_ASSERT_VALID( pPolyBrep );
      smgfx_Erase();
      smgfx_SetLook( 1, 2, 0, 0, 1 ); if(pPolyBrep) pPolyBrep->Draw( ); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE

    // get PolyFaces
    SmTArray<SmPolyFace*> sPolyFaces;
    pPolyBrep->GetPolyFaces( sPolyFaces );
    ULONG lStartFaces = sPolyFaces.GetSize();

    // locals
    double dPercentOfReduction = dPercentReduction;
    double dMinFeatureAngle = 45.0;
    double dMinDistToAveragePlane = dMaximumReductionError;
    double dInteriorEdgeWeight = 0.5;
    double dBoundaryEdgeWeight = 0.2;


    // build the PolyDecimate object
    SmPolyDecimate sDecimator( pPolyBrep,
                              dPercentOfReduction,
                              dMinDistToAveragePlane,
                              FALSE,
                              dMinFeatureAngle,
                              dInteriorEdgeWeight,
                              dBoundaryEdgeWeight );

#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      //SM_ASSERT_VALID(pPolyBrep) ;
      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,1 ); if(pPolyBrep) pPolyBrep->Draw( ); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE

    // branch on the kind of requested decimation
    if ( ! bQuadricDecimation )
    {
      SER( sDecimator.DoMeasuredDecimation() );
    }
    else
    {
      SER( sDecimator.DoQuadricDecimation() );
    } // increments unlocked mark value

#ifdef SM_GFX_CODE
    if(smGet_DoGraphics())
    {
      SM_ASSERT_VALID( pPolyBrep );
      smgfx_Erase();
      smgfx_SetLook( 2,3, 1,0,0 ); if(pPolyBrep) pPolyBrep->Draw( ); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_GFX_CODE

    // gather statistics
    pPolyBrep->GetPolyFaces( sPolyFaces );
    ULONG lEndFaces = sPolyFaces.GetSize();

    // inform the public
    TCHAR  sBuff[SM_TBLOCK_SIZE];
    double dReduction = (lStartFaces - lEndFaces) / (1.0*lStartFaces);
    SM_SPRINTF( sBuff, _T( "Initial # Triangles = %ld,  Reduction Percent = %lf,  Final # Triangles = %ld\n" ),
               lStartFaces, dReduction*100.0, lEndFaces );
    smos_WriteBuffer( sBuff );

  } // end need to decimate the triangles check

// all done
  return SM_SUCCESS;

} // end my_test_decimation

/*****************************************************
PURPOSE ---

USAGE NOTES ---
*****************************************************/
SmStatus my_poly_boolean_regression()
{
  MYPRINTF(_T("\n Entered: my_poly_boolean_regression")) ;

  SmContext crContext;
  SmTArray<SmPolyBrep*> rPolyBreps;

  if (TRUE) 
    {
      MYPRINTF(_T("\n    my_poly_boolean_regression: test 1"));
      rPolyBreps.ReSet();
      SmObjsDelete<SmPolyBrep*> sClean(&rPolyBreps);
      SER(my_box_box_stress(crContext,rPolyBreps));
    }

  if (TRUE) 
    {
      MYPRINTF(_T("\n    my_poly_boolean_regression: test 2"));

      rPolyBreps.ReSet();
      SmObjsDelete<SmPolyBrep*> sClean(&rPolyBreps);
      SmPolyBrep *pPolyBrep;
      SmTArray<SmPoint3d> sPnts;
      sPnts.Add(SmPoint3d(0,0,0));
      sPnts.Add(SmPoint3d(3,0,0));
      sPnts.Add(SmPoint3d(3,2,0));
      sPnts.Add(SmPoint3d(2,2,0));
      sPnts.Add(SmPoint3d(2,1,0));
      sPnts.Add(SmPoint3d(1,1,0));
      sPnts.Add(SmPoint3d(1,2,0));
      sPnts.Add(SmPoint3d(0,2,0));
      SmVector3d sSweepVec(0,0,-1);
      SER(SmPolyBrep::CreateLinearSweepSolid( crContext,
                                              1.0e-8,
                                              sPnts,
                                              sSweepVec,
                                              pPolyBrep));

      SmPolyBrep       * pPolyCube;
      SmAxis2Placement   sPos;
      sPos.Translate(SmVector3d(-1.0,1.2,-0.3));
      SER(SmPolyBrep::CreateBox(crContext,
                                1.0e-8,
                                5,
                                0.7,
                                0.5,
                                sPos,
                                pPolyCube));
      double       dAngleTol = 20.0*SM_PI/180.0;
      SmPolyBrep * pPolyResult;

#ifdef SM_GFX_CODE
        if(smGet_DoGraphics())
          {
            SM_ASSERT_VALID(pPolyBrep) ;
            SM_ASSERT_VALID(pPolyCube) ;

            smgfx_Erase() ; 
            smgfx_SetLook(1,2, 0,0,1) ; if(pPolyBrep) pPolyBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
            smgfx_SetLook(1,2, 0,1,0) ; if(pPolyCube) pPolyCube->Draw(TRUE) ; sm_GraphicsLoop() ;
            sm_GraphicsLoop() ;
          }        
#endif // SM_GFX_CODE

      // Boolean difference
      SmPolyMerge sPolyMerge(crContext,pPolyBrep,pPolyCube,pPolyCube->GetTolerance(),dAngleTol);
      SER(sPolyMerge.ManifoldBoolean(SM_PBO_DIFFERENCE,pPolyResult));

#ifdef SM_GFX_CODE
        if(smGet_DoGraphics())
          {
            SM_ASSERT_VALID(pPolyResult) ;

            smgfx_Erase() ; 
            smgfx_SetLook(2,3, 1,0,0) ; if(pPolyResult) pPolyResult->Draw(TRUE) ; sm_GraphicsLoop() ;
            sm_GraphicsLoop() ;
          }        
#endif // SM_GFX_CODE

      rPolyBreps.Add(pPolyResult);

    }

  if (TRUE) 
    {
      MYPRINTF(_T("\n    my_poly_boolean_regression: test 3"));
      rPolyBreps.ReSet();
      SmObjsDelete<SmPolyBrep*> sClean(&rPolyBreps);
      SmPolyBrep * pPolyResult = NULL;

      for (ULONG j=0; j<1; j++) 
        {
          SmPolyBrep     * pPolyCube1, * pPolyCube2;
          SmAxis2Placement sPos;
          SER(SmPolyBrep::CreateBox(crContext,1.0e-8,1,1,1,sPos,pPolyCube1));
          SmVector3d sMove(0.5,0.0,0.0);
          sPos.Translate(sMove);
          SER(SmPolyBrep::CreateBox(crContext,1.0e-8,1,1,1,sPos,pPolyCube2));
          double dAngleTol = 20.0*SM_PI/180.0;

#ifdef SM_GFX_CODE
            if(smGet_DoGraphics())
              {
                SM_ASSERT_VALID(pPolyCube1) ;
                SM_ASSERT_VALID(pPolyCube2) ;

                smgfx_Erase() ; 
                smgfx_SetLook(1,2, 0,0,1) ; if(pPolyCube1) pPolyCube1->Draw(TRUE) ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 0,1,0) ; if(pPolyCube2) pPolyCube2->Draw(TRUE) ; sm_GraphicsLoop() ;
                sm_GraphicsLoop() ;
              }        
#endif // SM_GFX_CODE
          // boolean union
          SmPolyMerge sPolyMerge(crContext,pPolyCube1,pPolyCube2,pPolyCube1->GetTolerance(),dAngleTol);
          SER(sPolyMerge.ManifoldBoolean(SM_PBO_UNION,pPolyResult));

#ifdef SM_GFX_CODE
        if(smGet_DoGraphics())
          {
            SM_ASSERT_VALID(pPolyResult) ;

            smgfx_Erase() ; 
            smgfx_SetLook(2,3, 1,0,0) ; if(pPolyResult) pPolyResult->Draw(TRUE) ; sm_GraphicsLoop() ;
            sm_GraphicsLoop() ;
          }        
#endif // SM_GFX_CODE
        }
      rPolyBreps.Add(pPolyResult);

    }

  if (TRUE) 
    { // test box booleans
      MYPRINTF(_T("\n    my_poly_boolean_regression: test 4"));
      rPolyBreps.ReSet();
      SmObjsDelete<SmPolyBrep*> sClean(&rPolyBreps);
        
      SmTArray<SmBrep*>    sBreps;
      SmTArray<SmCurve*>   s3DCurves;
      SmTArray<SmSurface*> sSurfaces;
      SmTArray<long>       sBooleanTrees;

      SmObjsDelete<SmBrep*   > sCleanB( &sBreps    );
      SmObjsDelete<SmCurve*  > sCleanC( &s3DCurves );
      SmObjsDelete<SmSurface*> sCleanS( &sSurfaces );

      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box_box_grid.smp"));
      SmBrepData::ReadPartFromFile(crContext, _T("../../TestFiles/pt_TestFiles/Solids/box_box_grid.smp"),
                                   s3DCurves,
                                   sSurfaces,
                                   sBooleanTrees,
                                   sBreps,
                                   SM_ASCII );

      for (ULONG i=1; i<sBreps.GetSize(); i=i+2) 
        {
          ULONG lOperation = 0;  // 0 - union, 1 - intersection, 2 difference
          SmPolyBrep *pPolyResult = NULL;
          if (my_test_ppu(crContext,sBreps[i-1],sBreps[i],lOperation,pPolyResult) == SM_SUCCESS) 
            {
              if (pPolyResult) rPolyBreps.Add(pPolyResult);
            }
          else 
            {
              ERR_MSG("**** POLY BOOLEAN Failure *****\n");
            }
        }
    }

  if (TRUE) 
    { // test cylindrical booleans
      MYPRINTF(_T("\n    my_poly_boolean_regression: test 5"));
      rPolyBreps.ReSet();
      SmObjsDelete<SmPolyBrep*> sClean(&rPolyBreps);
      SmTArray<SmBrep*>    sBreps;
      SmTArray<SmCurve*>   s3DCurves;
      SmTArray<SmSurface*> sSurfaces;
      SmTArray<long>       sBooleanTrees;

      SmObjsDelete<SmBrep*   > sCleanB( &sBreps    );
      SmObjsDelete<SmCurve*  > sCleanC( &s3DCurves );
      SmObjsDelete<SmSurface*> sCleanS( &sSurfaces );

      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box_cyl_grid.smp"));
      SmBrepData::ReadPartFromFile(crContext, _T("../../TestFiles/pt_TestFiles/Solids/box_cyl_grid.smp"),
                                   s3DCurves,
                                   sSurfaces,
                                   sBooleanTrees,
                                   sBreps,
                                   SM_ASCII );

      for (ULONG i=1; i<sBreps.GetSize(); i=i+2) 
        {
          ULONG lOperation = 2;  // 0 - union, 1 - intersection, 2 difference
          SmPolyBrep *pPolyResult = NULL;
          if (my_test_ppu(crContext,sBreps[i-1],sBreps[i],lOperation,pPolyResult) == SM_SUCCESS) 
            {
              if (pPolyResult) rPolyBreps.Add(pPolyResult);
            }
          else 
            {
              ERR_MSG("**** POLY BOOLEAN Failure *****\n");
            }
        }
    }

  if (TRUE) 
    { // test cone booleans
      MYPRINTF(_T("\n    my_poly_boolean_regression: test 6"));
      rPolyBreps.ReSet();
      SmObjsDelete<SmPolyBrep*> sClean(&rPolyBreps);
      SmTArray<SmBrep*>    sBreps;
      SmTArray<SmCurve*>   s3DCurves;
      SmTArray<SmSurface*> sSurfaces;
      SmTArray<long>       sBooleanTrees;

      SmObjsDelete<SmBrep*   > sCleanB( &sBreps    );
      SmObjsDelete<SmCurve*  > sCleanC( &s3DCurves );
      SmObjsDelete<SmSurface*> sCleanS( &sSurfaces );

      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Solids/box_cone_grid.smp"));
      SmBrepData::ReadPartFromFile(crContext, _T("../../TestFiles/pt_TestFiles/Solids/box_cone_grid.smp"),
                                   s3DCurves,
                                   sSurfaces,
                                   sBooleanTrees,
                                   sBreps,
                                   SM_ASCII );

      for (ULONG i=1; i<sBreps.GetSize(); i=i+2) 
        {
          ULONG        lOperation = 0;  // 0 - union, 1 - intersection, 2 difference
          SmPolyBrep * pPolyResult = NULL;
          if (my_test_ppu(crContext,sBreps[i-1],sBreps[i],lOperation,pPolyResult) == SM_SUCCESS) 
            {
              if (pPolyResult) rPolyBreps.Add(pPolyResult);
            }
          else 
            {
              ERR_MSG("**** POLY BOOLEAN Failure *****\n");
            }
        }
    }

  if (TRUE) 
    { // test solid/tool boolean
      MYPRINTF(_T("\n    my_poly_boolean_regression: test 7"));
      rPolyBreps.ReSet();
      SmObjsDelete<SmPolyBrep*> sClean(&rPolyBreps);

      SmPolyBrep * pPolyBrep1 = new (crContext) SmPolyBrep(SM_EFF_ZERO);
      SmPolyBrep * pPolyBrep2 = new (crContext) SmPolyBrep(SM_EFF_ZERO);

      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/solid1.pbp"));
      pPolyBrep1->ReadFromFile(crContext,_T("../../TestFiles/pt_TestFiles/Tess/solid1.pbp"),SM_ASCII);

      MYPRINTF(_T("\nReading test file ../../TestFiles/pt_TestFiles/Tess/tool1.pbp"));
      pPolyBrep2->ReadFromFile(crContext,_T("../../TestFiles/pt_TestFiles/Tess/tool1.pbp"),SM_ASCII);

      SM_ASSERT_VALID(pPolyBrep1) ;
      SM_ASSERT_VALID(pPolyBrep2) ;

      //pPolyBrep1->Draw(); sm_GraphicsLoop();
      //sm_GraphicsLoop();
      //pPolyBrep2->Draw(); sm_GraphicsLoop();
      //sm_GraphicsLoop();
      SmPolyBrep *pPolyResult = NULL;
      // 0 - union, 1 - intersection, 2 - difference, 3 - merge
      if (my_test_ppu_poly(crContext,
                           pPolyBrep1, 
                           pPolyBrep2,
                           2,
                           pPolyResult) == SM_SUCCESS) 
        {
          if (pPolyResult) rPolyBreps.Add(pPolyResult);
        }
      else 
        {
          ERR_MSG("**** POLY BOOLEAN Failure *****\n");
        }
    }

  // all done
  return SM_SUCCESS;

} // end my_poly_boolean_regression

/*****************************************************
PURPOSE ---

USAGE NOTES ---
*****************************************************/
SmStatus my_test_decimation_regression()
{
  MYPRINTF(_T("\n   Entered: my_test_decimation_regression 1")) ;

  // locals 
  SmContext sContext;
  SmTArray<SmPolyBrep*> sPolyBreps;

  // Tests

  MYPRINTF(_T("\n my_test_decimation_regression: test 1")) ;
    {
      SmObjsDelete<SmPolyBrep*> sClean(&sPolyBreps);
      SE(my_test_decimation(sContext,_T("../../TestFiles/pt_TestFiles/Solids/nurb_fillet.smb"),
                            0.0, 10.0, 25.0,
                            0.80, 0.0,  3,
                            FALSE,sPolyBreps)); 
    }

  MYPRINTF(_T("\n my_test_decimation_regression: test 2")) ;
  sPolyBreps.ReSet();
    {
      SmObjsDelete<SmPolyBrep*> sClean(&sPolyBreps);
      SE(my_test_decimation(sContext,_T("../../TestFiles/pt_TestFiles/Solids/nurb_fillet.smb"),
                            0.0, 10.0, 40.0,
                            0.80, 0.0,  3,
                            TRUE,sPolyBreps)); 
    }

  MYPRINTF(_T("\n my_test_decimation_regression: test 3")) ;
  sPolyBreps.ReSet();
    {
      SmObjsDelete<SmPolyBrep*> sClean(&sPolyBreps);
      SE(my_test_decimation(sContext,_T("../../TestFiles/pt_TestFiles/Solids/sphere.smb"),
                            0.0, 15.0, 30.0,
                            0.80, 0.0,  3,
                            FALSE,sPolyBreps));  
    }

  MYPRINTF(_T("\n my_test_decimation_regression: test 4")) ;
  sPolyBreps.ReSet();
    {
      SmObjsDelete<SmPolyBrep*> sClean(&sPolyBreps);
      SE(my_test_decimation(sContext,_T("../../TestFiles/pt_TestFiles/Solids/sphere.smb"),
                            0.0, 15.0, 40.0,
                            0.80, 0.0,  3,
                            TRUE,sPolyBreps));  
    }

  MYPRINTF(_T("\n my_test_decimation_regression: test 5")) ;
  sPolyBreps.ReSet();
    {
      SmObjsDelete<SmPolyBrep*> sClean(&sPolyBreps);
      SE(my_test_decimation(sContext,_T("../../TestFiles/pt_TestFiles/Tess/nurby_box_cyl_sphere.smb"),
                            0.0, 15.0, 40.0,
                            0.80, 0.0,  3,
                            FALSE,sPolyBreps));  
    }

  MYPRINTF(_T("\n my_test_decimation_regression: test 6")) ;
  sPolyBreps.ReSet();
    {
      SmObjsDelete<SmPolyBrep*> sClean(&sPolyBreps);
      SE(my_test_decimation(sContext,_T("../../TestFiles/pt_TestFiles/Tess/nurby_box_cyl_sphere.smb"),
                            0.0, 15.0, 40.0,
                            0.80, 0.0,  3,
                            TRUE,sPolyBreps));  
    }

  MYPRINTF(_T("\n my_test_decimation_regression: test 7")) ;
  sPolyBreps.ReSet();
    {
      SmObjsDelete<SmPolyBrep*> sClean(&sPolyBreps);
      SE(my_test_decimation(sContext,_T("../../TestFiles/pt_TestFiles/ellips_ts2.smb"),
                            0.01, 5.0, 25.0,
                            0.80, 0.0,  3,
                            FALSE,sPolyBreps));  
    }

  MYPRINTF(_T("\n my_test_decimation_regression: test 8")) ;
  sPolyBreps.ReSet();
    {
      SmObjsDelete<SmPolyBrep*> sClean(&sPolyBreps);
      SE(my_test_decimation(sContext,_T("../../TestFiles/pt_TestFiles/ellips_ts2.smb"),
                            0.01, 5.0, 30.0,
                            0.80, 0.0,  3,
                            TRUE,sPolyBreps));  
    }

  // all done
  return SM_SUCCESS;

} // end my_test_decimation_regression
