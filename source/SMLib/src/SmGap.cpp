// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmGap.cpp
* PURPOSE: Source file for implementation of SmGapFunction, SmGap, and related class methods.
*
* CONTAINS --- methods for the classes
* Class SmGapFunction          // base class representing the set of all gaps between a pair of topology objects
*                              // currently approximated by an array of SmGapSamples. The exact representation 
*                              // would be a function.
*         SmCrvPtGapFunction   //   GapFunction(s) from Crv to Pt,  defined over Crv param s                       
*         SmCrvCrvGapFunction  //   GapFunction(s) from Crv to Crv, defined over Crv param s
*         SmCrvSrfGapFunction  //   GapFunction(s) from Crv to Srf, defined over Crv param s
*         SmSrfPtGapFunction   //   GapFunction(u,v) from Srf to Pt,  defined over Srf param u,v  
*         SmSrfCrvGapFunction  //   GapFunction(u,v) from Srf to Crv, defined over Srf param u,v 
*         SmSrfSrfGapFunction  //   GapFunction(u,v) from Srf to Srf, defined over Srf param u,v 
*
* class SmGap           // the largest single gap between a pair of topology objects.
*                       // contains: gap topology type, gap 3d distance, topology objects and param values identifying the gap ends
*                       //  Gap Types are oneof: SM_GT_VERTEX_EDGE          (same as  SM_GT_EDGE_VERTEX)
*                       //                       SM_GT_VERTEX_FACETRIMCURVE (same as  SM_GT_UVTRIMCURVE_VERTEX)        
*                       //                       SM_GT_VERTEX_FACE          (same as  SM_GT_FACE_VERTEX)
*                       //                         
*                       //                       SM_GT_EDGE_FACETRIMCURVE   (same as  SM_GT_UVTRIMCURVE_EDGE)  
*                       //                       SM_GT_EDGE_FACE            (same as  SM_GT_FACE_EDGE)       
*                       //                       
* class SmGapArray      // A list of SmGaps for a Topology Object and the largest of all gaps in the list.
* class SmGapSample     // gap geometry type (SM_GD_UNKNOWN  ), gap start and end point data (position and                        )
*                       //                   (SM_GD_NORMAL   )                               ( associated geometry parameter value)
*                       //                   (SM_GD_BOUNDARY )
*                       //                   (SM_GD_NO_DROP  )
* class SmLocalInterval // Interval marking the boundary points where one shape is within tol of another shape.
**********************************************************************/

#include "StdAfx.h"

#include <SmGap.h>
#include <SmGraphicsOutput.h>
#include <SmVertex.h>
#include <SmEdge.h>
#include <SmFace.h>
#include <SmVolume.h>
#include <SmBrep.h>
#include <SmCrvOnSurf.h>

#ifdef SM_DEBUG_CODE
#include <SmMessages.h>
#include <SmAssertArray.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Equality operator

NOTES:
***********************************************************************/
SmBoolean SmGap::operator==    // rtn: TRUE when two gaps are the same
  (const SmGap &crGap)         // in : target gap
 const
{
  if(this == &crGap) 
   { return TRUE ; }

  if(GetType() != crGap.GetType())
   { return FALSE ; }

  // locals
  SmObject   * pThisObj1,  * pThisObj2 ;
  SmObject   * pOtherObj1, * pOtherObj2 ;
  SmScaledZero sScaledZero = SmTol::GetScaledZero(m_dGap3d) ;

  // get gap objects
  GetTopoObjects(pThisObj1,  pThisObj2) ;
  GetTopoObjects(pOtherObj1, pOtherObj2) ;

  // calc and return if two gaps are equivalent
  return (   pThisObj1 == pOtherObj1
          && pThisObj2 == pOtherObj2
          && SM_ARE_SAME_TO_TOL(m_dGap3d, crGap.GetLength(), sScaledZero) ) ; 

  // double dXSectTol2d = 100.0 * SM_EFF_ZERO * (1.0 + (GetEdge() ? m_dEdgeT : 0.0)
  //                                            + (GetFace() ? m_sFaceUV.GetMaxDimension() : 0.0)) ;
  // SmBoolean bRtn = (    GetVertex()  == crGap.GetVertex()
  //                   &&  GetEdge()    == crGap.GetEdge()
  //                   &&  GetFace()    == crGap.GetFace()
  //                   && (GetEdge()    == NULL || SM_ARE_SAME_TO_TOL(m_dEdgeT,        crGap.m_dEdgeT,        dXSectTol2d))
  //                   && (GetEdgeuse() == NULL || SM_ARE_SAME_TO_TOL(m_dUVTrimCurveT, crGap.m_dUVTrimCurveT, dXSectTol2d))
  //                   && (GetFace()    == NULL || m_sFaceUV.CloserThan(dXSectTol2d, crGap.m_sFaceUV))) ; 

} // end SmBoolean SmGap::operator==

/*******************************************************************//**
PURPOSE: Get Topology Objects bounding this gap

NOTES: rather than a bunch of derived methods 
       this single method supports all derived classes.

       This is okay as long as the no new derived classes are made
***********************************************************************/
SmStatus SmGap::GetTopoObjects
  (SmObject *& rpObj1,   // out: one gap bounding object 
   SmObject *& rpObj2)   // out: the other gap bounding object
 const
{ 
  // init output
  rpObj1 = NULL ;
  rpObj2 = NULL ;

  // rather than a bunch of derived methods - just figure out the pair of objects bounding this gap
  SmObject  * pVertex = GetVertex() ;
  SmObject  * pEdge   = GetEdge() ;
  SmObject  * pFace   = GetFace() ;
  SmBoolean   bEdge   = GetEdgeT() != SM_UNDEF_DOUBLE ;

  // set output
  rpObj1   =   pVertex ? pVertex
             : bEdge   ? pEdge
                       : NULL ;
  rpObj2   =   (bEdge && pEdge != rpObj1) ? pEdge
             : pFace                      ? pFace
                                          : NULL ;
  return(SM_SUCCESS) ;

} // end SmGap::GetTopoObjects

/*******************************************************************//**
PURPOSE: Get gap end points

NOTES: rather than a bunch of derived methods 
       this single method supports all derived classes.

       This is okay as long as the no new derived classes are made
***********************************************************************/
SmStatus SmGap::GetGapEndPoints
  (SmPoint3d &rPoint1,   // out: one gap bounding point 
   SmPoint3d &rPoint2)   // out: the other gap bounding point
 const
{ 
  // rather than a bunch of derived methods - build the logic here to get end points 
  SmVertex        * pVertex       = GetVertex() ;
  SmEdge          * pEdge0        = GetEdge(0) ;
  SmEdgeuse       * pEdgeuse0     = GetEdgeuse(0) ;
  SmEdge          * pEdge1        = GetEdge(1) ;
  // SmEdgeuse       * pEdgeuse1     = GetEdgeuse(1) ;
  SmFace          * pFace         = GetFace() ;
                                  
  double            dEdgeT0       = GetEdgeT(0) ;
  double            dEdgeT1       = GetEdgeT(1) ;
  double            dUVTrimCurveT = GetUVTrimCurveT() ;
  const SmPoint2d   sFaceUV       = GetFaceUV() ; 

  // locals
  ULONG       lCnt = 0 ;
  SmPoint3d * pTarget = &rPoint1 ;

  // get gap end points

  // Point = Vertex->Point
  if(pVertex)                          { *pTarget = pVertex->GetPoint() ;
                                          pTarget = &rPoint2 ; 
                                          lCnt++ ;
                                       }
  // Point = Edge0->Curve(T0)
  if(dEdgeT0 != SM_UNDEF_DOUBLE)       { SM_ASSERT_MSG(pEdge0 != NULL, _T("GetGapEndPoints(): Fetched a EdgeT0 value without an Edge0")) ; 
                                         pEdge0->GetCurve()->EvaluatePoint(dEdgeT0, *pTarget) ;
                                         pTarget = &rPoint2 ;
                                         lCnt++ ;
                                       }

  // Point = Edge1->Curve(T1)
  if(dEdgeT1 != SM_UNDEF_DOUBLE)       { SM_ASSERT_MSG(pEdge1 != NULL, _T("GetGapEndPoints(): Fetched a EdgeT1 value without an Edge1")) ; 
                                         pEdge1->GetCurve()->EvaluatePoint(dEdgeT1, *pTarget) ;
                                         pTarget = &rPoint2 ;
                                         lCnt++ ;
                                       }

  // Point = Face->Surface->Evaluate(UVTrimCurve(T))
  if(dUVTrimCurveT != SM_UNDEF_DOUBLE) { SM_ASSERT_MSG(pEdgeuse0 != NULL, _T("GetGapEndPoints(): Fetched a UVTrimCurve without an Edgeuse")) ; 
                                         SmPoint3d        sPtUV ;
                                         SmSurface      * pSurface     = pFace ? pFace->GetSurface() : NULL ;
                                         SmBSplineCurve * pUVTrimCurve = NULL ;
                                         pEdgeuse0->GetOrCreateUVTrimCurve(pUVTrimCurve) ;
                                         if(pUVTrimCurve) { pUVTrimCurve->EvaluatePoint(dUVTrimCurveT, sPtUV) ; }
                                         if(pSurface)     { SmPoint2d sUV(sPtUV.x, sPtUV.y) ;
                                                            pSurface->EvaluatePoint(sUV, *pTarget) ;
                                                            pTarget = &rPoint2 ; 
                                                          }
                                         lCnt++ ;
                                       }
  // Point = Face->Surface(U,V)
  // note: the following else is needed.
  //       SmVertexFaceTrimGap and SmEdgeFaceTrimGaps 
  //       return both dUVTrimCurveT and sFaceUV values 
  //       - only evaluate one
  else if(sFaceUV.IsInitialized())     { SM_ASSERT_MSG(pFace != NULL, _T("GetGapEndPoints(): Fetched a FaceUV value without a Face")) ;
                                         pFace->GetSurface()->EvaluatePoint(sFaceUV, *pTarget) ; 
                                         pTarget = &rPoint2 ;
                                         lCnt++ ;
                                       }
  SM_ASSERT_MSG(lCnt == 2,_T("SmGap::GetGapEndPoints assumptions for Gap End Point access are broken - this is a bug")) ; 

  // all done
  return(lCnt == 2 ? SM_SUCCESS : SM_ERR) ;

} // end SmGap::GetGapEndPoints

/*******************************************************************//**
PURPOSE: Recompute vertex and edge locations to try and fix gap 

NOTES:  
  returns TRUE when any geometry shape changes are made
  else returns FALSE.

  A return value of TRUE does not mean that the gap has
  been fixed.
***********************************************************************/
SmStatus SmGap::RefineGeometry
 (SmGapArray & rAfterGaps,  // out: gaps connected to any vertex or edge 
                            //      connected to this gap after RefineGeometry() runs.
  SmBoolean  & bMadeChange) // out: TRUE = geometry changed (gaps may not be fixed)
                            //      FALSE= geometry not changed
{ 
  // init output
  SmStatus sRtn = SM_SUCCESS ;
  bMadeChange   = FALSE ; 
  rAfterGaps.ReSet() ;

  // locals
  ULONG ii ;
  SmGapArray        sGaps ;
  SmVertex        * pVertex  = GetVertex() ;
  SmEdge          * pEdge    = GetEdge() ;
  SmFace          * pFace    = GetFace() ; 
  SmVertex        * pEdgeV1  = NULL ;
  SmVertex        * pEdgeV2  = NULL ;
  const SmContext * pContext =   pVertex ? pVertex->GetContext() 
                               : pEdge   ? pEdge->GetContext()
                               : pFace   ? pFace->GetContext()
                               : NULL ;
  SmBoolean bMadeEdgeChange   = FALSE ;
  SmBoolean bMadeVertexChange = FALSE ;
  
  // no context - no work
  if(pContext == NULL)
    {
      return(FALSE) ;
    } 

  // get and lock a mark
  SmNewMarkAndLock sMark((SmContext *)pContext, SM_MT_ALLMARKS) ; // increments newly locked mark
  SmMarkType eMarkType = sMark.GetMarkType() ;

  // refine edge vertices and edge shape
  if(pEdge)   
    { 
      sRtn         = pEdge->RefineGeometry(eMarkType, bMadeEdgeChange) ;
      bMadeChange |= bMadeEdgeChange ;
      if(sRtn != SM_SUCCESS)
        {
          return(sRtn) ;
        }
      pEdgeV1 = pEdge->GetVertex() ;
      pEdgeV2 = pEdge->GetOtherVertex(pEdgeV1) ;
    }

  // refine vertex position
  // no need to modify a vertex once its edge has been changed
  if(   pVertex
     && (   pEdge == NULL
         || (   pEdgeV1 != pVertex
             && pEdgeV2 != pVertex))) 
    { 
      sRtn         = pVertex->RefineGeometry(eMarkType, bMadeVertexChange) ;
      bMadeChange |= bMadeEdgeChange ;
      if(sRtn != SM_SUCCESS)
        {
          return(sRtn) ;
        }
    }

  // gather gaps for this vertex and edge
  pVertex->GetMaxGap3d(&rAfterGaps) ;   //  MaxGap3d = Max( Vertex/Edge Vertex/Face Gap )
  pEdge->GetMaxGap3d(&sGaps) ;          //  MaxGap3d = Max( Edge/Face Vertex/Edge Gaps )  - does not include LoopGaps (get those from SmLoop)

  // add every unique Edge->Gap to rAfterGaps array 
  for(ii=0;ii<sGaps.GetSize();ii++)
    {
      SmGap *pGap = sGaps[ii] ;

      // skip vertex gaps conecting to pVertex - already collected
      if(pVertex == pGap->GetVertex())
        { continue ; }

      // otherwise add the gap to the array
      rAfterGaps.Add(sGaps[ii]) ;

    } // end iter every edge gap

  // all done
  return(sRtn) ;

} // end SmGap::RefineGeometry

/*******************************************************************//**
PURPOSE: Pretty print Gap 

NOTES:
***********************************************************************/
void SmGap::Dump
  (ULONG lLabel)    // in: Numeric label prefixed to Gap report
 const              //     SM_UNDEF_DOUBLE to ignore
{
  TCHAR sBuff [SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  // optional numeric label
  if(lLabel != SM_UNDEF_DOUBLE) { smos_sprintf(sBuff, _T("\n    %3lu "), lLabel) ; }
  else                          { smos_sprintf(sBuff, _T("%s"), _T("\n    ")) ; }
  smos_WriteBuffer(sBuff);

  // Get Dump Line
  GetDumpLine(sBuff, sBuffForFile) ; 

  // Dump Line
  smos_WriteBuffer(sBuff, sBuffForFile);

} // end SmGap::Dump      

/*******************************************************************//**
PURPOSE: Pass the virtual Dump() calls along to the SmGap::Dump(lLabel) call

NOTES:
***********************************************************************/
void SmGap                   ::Dump() const { SmGap::Dump( SM_UNDEF_ULONG ) ; }
void SmVertexEdgeGap         ::Dump() const { SmGap::Dump( SM_UNDEF_ULONG ) ; }
void SmVertexFaceTrimCurveGap::Dump() const { SmGap::Dump( SM_UNDEF_ULONG ) ; }
void SmVertexFaceGap         ::Dump() const { SmGap::Dump( SM_UNDEF_ULONG ) ; }
void SmEdgeEdgeGap           ::Dump() const { SmGap::Dump(SM_UNDEF_ULONG) ; }
void SmEdgeFaceTrimCurveGap  ::Dump() const { SmGap::Dump( SM_UNDEF_ULONG ) ; }
void SmEdgeFaceGap           ::Dump() const { SmGap::Dump( SM_UNDEF_ULONG ) ; }

/*******************************************************************//**
PURPOSE: Compose a single line pretty print gap dump report 

NOTES:
***********************************************************************/
void SmGap::GetDumpLine
  (TCHAR sBuff[SM_TBLOCK_SIZE],        // out: Pretty Print Gap Report, sized:[SM_TBLOCK_SIZE]
   TCHAR sBuffForFile[SM_TBLOCK_SIZE]) // out: Pretty Print Gap Report without pointer values, sized:[SM_TBLOCK_SIZE]
 const                
{
  // uses 4 entries but to checkmarx it looks like lCnt can grow to 6 - to avoid a checkmarx error size as 6
  TCHAR sThisBuff[6][SM_TBLOCK_SIZE], sThisBuffForFile[6][SM_TBLOCK_SIZE] ;

  SmVertex  *pVertex   = GetVertex() ;
  SmEdgeuse *pEdgeuse0 = GetEdgeuse(0) ;
  // SmEdgeuse *pEdgeuse1 = GetEdgeuse(1) ;
  SmEdge    *pEdge0    = GetEdge(0) ;
  SmEdge    *pEdge1    = GetEdge(1) ;
  SmFace    *pFace     = GetFace() ;

  double            dEdgeT0       = GetEdgeT(0) ;
  double            dEdgeT1       = GetEdgeT(1) ;
  double            dUVTrimCurveT = GetUVTrimCurveT() ;
  const SmPoint2d   sFaceUV       = GetFaceUV() ;

  // header
  if     (IsKindOf(SmVertexEdgeGap_TYPE         )) { SM_SPRINTF(sThisBuff[0] , _T("%s/%s Gap = %16.16lf"), _T("Vertex"), _T("Edge")          , GetLength()) ; }
  else if(IsKindOf(SmVertexFaceTrimCurveGap_TYPE)) { SM_SPRINTF(sThisBuff[0] , _T("%s/%s Gap = %16.16lf"), _T("Vertex"), _T("FaceTrimCurve") , GetLength()) ; }
  else if(IsKindOf(SmVertexFaceGap_TYPE         )) { SM_SPRINTF(sThisBuff[0] , _T("%s/%s Gap = %16.16lf"), _T("Vertex"), _T("Face")          , GetLength()) ; }
  else if(IsKindOf(SmEdgeEdgeGap_TYPE           )) { SM_SPRINTF(sThisBuff[0] , _T("%s/%s Gap = %16.16lf"), _T("Edge")  , _T("Edge")          , GetLength()) ; }
  else if(IsKindOf(SmEdgeFaceTrimCurveGap_TYPE  )) { SM_SPRINTF(sThisBuff[0] , _T("%s/%s Gap = %16.16lf"), _T("Edge")  , _T("FaceTrimCurve") , GetLength()) ; }
  else if(IsKindOf(SmEdgeFaceGap_TYPE           )) { SM_SPRINTF(sThisBuff[0] , _T("%s/%s Gap = %16.16lf"), _T("Edge")  , _T("Face")          , GetLength()) ; }

  ULONG lCnt = 1 ;

  // connected entity report
  if(pVertex)                          { SM_SPRINTF(sThisBuff[lCnt]        ,_T(", Vertex[0x%p]"), pVertex) ;
                                         SM_SPRINTF(sThisBuffForFile[lCnt] ,_T("%s"), _T(", Vertex[NotNULL]") ) ;
                                         lCnt++ ;
                                       }  
  if(dEdgeT0 != SM_UNDEF_DOUBLE )      { SM_SPRINTF(sThisBuff[lCnt]        ,_T(", Edge[0x%p, T=%16.16lf]"), pEdge0, dEdgeT0) ;
                                         SM_SPRINTF(sThisBuffForFile[lCnt] ,_T(", Edge[NotNULL, T=%16.16lf]"), dEdgeT0) ;
                                         lCnt++ ;                          
                                       }  
  if(dEdgeT1 != SM_UNDEF_DOUBLE )      { SM_SPRINTF(sThisBuff[lCnt]        ,_T(", Edge[0x%p, T=%16.16lf]"), pEdge1, dEdgeT1) ;
                                         SM_SPRINTF(sThisBuffForFile[lCnt] ,_T(", Edge[NotNULL, T=%16.16lf]"), dEdgeT1) ;
                                         lCnt++ ;                          
                                       }  
  if(dUVTrimCurveT != SM_UNDEF_DOUBLE) { SM_SPRINTF(sThisBuff[lCnt]        ,_T(", Edgeuse[0x%p, T=%16.16lf]"), pEdgeuse0, dUVTrimCurveT) ;
                                         SM_SPRINTF(sThisBuffForFile[lCnt] ,_T(", Edgeuse[NotNULL, T=%16.16lf]"), dUVTrimCurveT) ;
                                         lCnt++ ;                          
                                       }  
  if(sFaceUV.IsInitialized()  )        { SM_SPRINTF(sThisBuff[lCnt]        ,_T(", Face[0x%p, UV=%16.16lf, %16.16lf]"), pFace, sFaceUV.x, sFaceUV.y) ;  
                                         SM_SPRINTF(sThisBuffForFile[lCnt] ,_T(", Face[NotNULL, UV=%16.16lf, %16.16lf]"), sFaceUV.x, sFaceUV.y) ;  
                                         lCnt++ ;
                                       }
          
  SM_ASSERT(lCnt == 3 || lCnt == 4) ;

  // compose the output
  if(lCnt == 3)
    {
      smos_sprintf(sBuff,        _T("%.256s%.256s%.256s"), sThisBuff[0], sThisBuff[1], sThisBuff[2]) ; 
      smos_sprintf(sBuffForFile, _T("%.256s%.256s%.256s"), sThisBuff[0], sThisBuffForFile[1], sThisBuffForFile[2]) ; 
    }
  else
    {
      smos_sprintf(sBuff,        _T("%.128s%.128s%.128s%.128s"), sThisBuff[0], sThisBuff[1], sThisBuff[2], sThisBuff[3]) ; 
      smos_sprintf(sBuffForFile, _T("%.128s%.128s%.128s%.128s"), sThisBuff[0], sThisBuffForFile[1], sThisBuffForFile[2], sThisBuffForFile[3]) ; 
    }

} // end SmGap::GetDumpLine

/*******************************************************************//**
PURPOSE: Draw a gap graphic

NOTES:  SmXSectTol3d    sXSectTol3d not a reference because it's given
                        a default const value, SM_XSECT_TOL_3D.  That
                        can't be done with a reference unless we create
                        a global SmXSectTol3d value for that.
***********************************************************************/
SmDisplayList * SmGap::Draw
 (SmXSectTol3d    sXSectTol3d,    // in : SM_USE_NEWTOL: not used
                                  //      OldTol       : override tol value, 0.0 to ignore. default:[SM_XSECT_TOL_3D]
  SmGfxArraySet * pOptGfxSet)     // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
 const                            //      NULL to ignore. default:[NULL]
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // start new displayList (unless one is already open)
  // use gap color and draw sizes rather than default ones
  double dLineWidth = smgfx_GetGapLineWidth() ;
  double dPointSize = smgfx_GetGapPointSize() ;
  smgfx_Open(smgfx_GetGapPointColor(), &dLineWidth, &dPointSize, FALSE, pOptGfxSet);

  // tolerance
#ifdef SM_USE_NEWTOL
  SM_NEWTOL_LINE sXSectTol3d = GetXSectTol3d() ;
#else // SM_USE_OLDTOL
  SM_OLDTOL_LINE if(sXSectTol3d <= 0.0) sXSectTol3d = SM_XSECT_TOL_3D ;
#endif // SM_USE_OLDTOL
  SmXSectTol3d sXSectTol3dSq = sXSectTol3d * sXSectTol3d ;

  // Get Gap end points
  SmPoint3d sPt[3] ; 
  SmStatus sRtn = GetGapEndPoints(sPt[0], sPt[1]) ;

  // when end points are available
  if(sRtn == SM_SUCCESS)
    {
      // find midPoint dXSectTol3d away from sPt[0] when sPt[1] is more than dXSectTol3d away
      double dGapSq = sPt[0].DistanceBetweenSquared(sPt[1]) ;

      // draw two point icons and one or two lines
      smgfx_OutputPoint(sPt[0].x, sPt[0].y, sPt[0].z);
      smgfx_OutputPoint(sPt[1].x, sPt[1].y, sPt[1].z) ;
      SmVector3d sColor = smgfx_OutputColor(  IsKindOf(SmVertexEdgeGap_TYPE         ) ? smgfx_GetVertexEdgeGapColor() 
                                            : IsKindOf(SmVertexFaceTrimCurveGap_TYPE) ? smgfx_GetVertexUVTrimCurveGapColor() 
                                            : IsKindOf(SmVertexFaceGap_TYPE         ) ? smgfx_GetVertexFaceGapColor()
                                            : IsKindOf(SmEdgeEdgeGap_TYPE           ) ? smgfx_GetEdgeEdgeGapColor()
                                            : IsKindOf(SmEdgeFaceTrimCurveGap_TYPE  ) ? smgfx_GetEdgeUVTrimCurveGapColor()
                                            : IsKindOf(SmEdgeFaceGap_TYPE           ) ? smgfx_GetEdgeFaceGapColor()
                                            : SmVector3d(0,0,0) ) ; 
      SmBoolean bDashedLines = smgfx_OutputDashedLines(FALSE) ;
      if(dGapSq <= sXSectTol3dSq)
        {
          smgfx_OutputLine(sPt[0].x, sPt[0].y, sPt[0].z,
                           sPt[1].x, sPt[1].y, sPt[1].z) ;
        }
      else // gap exceeds current tolerance
        {
          double dParam = smos_Sqrt(dGapSq / sXSectTol3dSq) ;
          sPt[2] = (1 - dParam) * sPt[0] + dParam * sPt[1] ;

          smgfx_OutputColor(  smgfx_GetInTolGapColor() , pOptGfxSet) ;
          smgfx_OutputLine(sPt[0].x, sPt[0].y, sPt[0].z,
                           sPt[2].x, sPt[2].y, sPt[2].z, pOptGfxSet) ;

          smgfx_OutputColor(  smgfx_GetOutTolGapColor() , pOptGfxSet) ;
          smgfx_OutputLine(sPt[2].x, sPt[2].y, sPt[2].z,
                           sPt[1].x, sPt[1].y, sPt[1].z, pOptGfxSet) ;
        }
      smgfx_OutputDashedLines(bDashedLines, pOptGfxSet) ;
      smgfx_OutputColor(sColor, pOptGfxSet) ;
    }

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(sXSectTol3d, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmGap::Draw

/*******************************************************************//**
PURPOSE: Draw Neighbor entities for a gap graphic

NOTES:
***********************************************************************/
SmDisplayList * SmGap::DrawNeighbors
 (SmGfxArraySet  * pOptGfxSet)    // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
 const                            //      NULL to ignore. default:[NULL]
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // output graphics locals  
  ULONG ii ;
  SmVector3d sColor ;
  SmVertex  *pVertex  = GetVertex() ;  // only nonNULL for vertex/edge and vertex/face gaps
  SmEdge    *pEdge0    = GetEdge(0) ;    // only nonNULL for vertex/edge and edge/face gaps
  SmEdge    *pEdge1    = GetEdge(1) ;    // only nonNULL for vertex/edge and edge/face gaps
  SmEdgeuse *pEdgeuse0 = GetEdgeuse(0) ; // only nonNULL for vertex/edgeuse and edge/edgeuse gaps
  SmEdgeuse *pEdgeuse1 = GetEdgeuse(1) ; // only nonNULL for vertex/edgeuse and edge/edgeuse gaps
  SmFace    *pFace    = GetFace() ;    // only nonNULL for vertex/face and edge/face gaps

  SmTArray<SmFace *> sFaces, sTmpFaces ;
  SmTArray<SmEdge *> sEdges, sTmpEdges ;

  // gather edges and faces connected to vertex
  if(pVertex) { pVertex->GetFaces(sFaces) ;
                pVertex->GetEdges(sEdges) ;
              }
  
  // gather faces connected to edge0
  if(pEdge0)  { pEdge0->GetFaces(sTmpFaces) ;
                if(sFaces.GetSize() > 0)
                  {
                    ULONG lCnt = sTmpFaces.GetSize();
                    for(ii=0; ii<lCnt; ii++)
                      {
                        SmFace *pF = sTmpFaces[ii] ;
                        sFaces.AddUnique(pF) ;
                      }
                  }
              }

  // gather faces connected to edge1
  if(pEdge1)  { pEdge1->GetFaces(sTmpFaces) ;
                if(sFaces.GetSize() > 0)
                  {
                    ULONG lCnt = sTmpFaces.GetSize();
                    for(ii=0; ii<lCnt; ii++)
                      {
                        SmFace *pF = sTmpFaces[ii] ;
                        sFaces.AddUnique(pF) ;
                      }
                  }
              }

  // gather faces connected to edgeuse0
  if(pEdgeuse0) { pEdgeuse0->GetEdge()->GetFaces(sTmpFaces) ;
                 if(sFaces.GetSize() > 0)
                   {
                     ULONG lCnt = sTmpFaces.GetSize();
                     for(ii=0; ii<lCnt; ii++)
                       {
                         SmFace *pThisFace = sTmpFaces[ii] ;
                         sFaces.AddUnique(pThisFace) ;
                       }
                   }
               }

  // gather faces connected to edgeuse1
  if(pEdgeuse1) { pEdgeuse1->GetEdge()->GetFaces(sTmpFaces) ;
                 if(sFaces.GetSize() > 0)
                   {
                     ULONG lCnt = sTmpFaces.GetSize();
                     for(ii=0; ii<lCnt; ii++)
                       {
                         SmFace *pThisFace = sTmpFaces[ii] ;
                         sFaces.AddUnique(pThisFace) ;
                       }
                   }
               }

  // start new displayList (unless one is already open) use neighbor color and default draw sizes.
  smgfx_Open(smgfx_GetNeighborColor(), NULL, NULL, FALSE, pOptGfxSet);

  // draw objects bounding ends of the gap in appropriate gapType color
  double                  dLineWidth = smgfx_SetLineWidth(2.0 * smgfx_GetLineWidth(), pOptGfxSet) ;
  if(IsKindOf(SmVertexEdgeGap_TYPE         )) sColor     = smgfx_SetColor(smgfx_GetVertexEdgeGapColor(),        pOptGfxSet) ;
  if(IsKindOf(SmVertexFaceTrimCurveGap_TYPE)) sColor     = smgfx_SetColor(smgfx_GetVertexUVTrimCurveGapColor(), pOptGfxSet) ;
  if(IsKindOf(SmVertexFaceGap_TYPE         )) sColor     = smgfx_SetColor(smgfx_GetVertexFaceGapColor(),        pOptGfxSet) ;
  if(IsKindOf(SmEdgeEdgeGap_TYPE           )) sColor     = smgfx_SetColor(smgfx_GetEdgeEdgeGapColor(),          pOptGfxSet) ;
  if(IsKindOf(SmEdgeFaceTrimCurveGap_TYPE  )) sColor     = smgfx_SetColor(smgfx_GetEdgeUVTrimCurveGapColor(),   pOptGfxSet) ;
  if(IsKindOf(SmEdgeFaceGap_TYPE           )) sColor     = smgfx_SetColor(smgfx_GetEdgeFaceGapColor(),          pOptGfxSet) ;

  // draw face
  if(pFace)    { pFace->Draw(SM_DM_CROSSHATCH) ; }

  // draw edge/edgeuse
  smgfx_SetLineWidth(2.0 * smgfx_GetLineWidth()) ;
  if(pEdge0)    { pEdge0->Draw() ; }
  if(pEdge1)    { pEdge1->Draw() ; }
  if(!pEdge0 && pEdgeuse0) 
                { SmSurface      * pSurface = pEdgeuse0->GetFace() ? pEdgeuse0->GetFace()->GetSurface() : NULL ;
                  SmBSplineCurve * pUVTrimCurve ;
                  pEdgeuse0->GetOrCreateUVTrimCurve(pUVTrimCurve) ;
                  if(pUVTrimCurve && pSurface)
                    { SmCrvOnSurf sCrvOnSurf(*pUVTrimCurve, *pSurface) ;
                      sCrvOnSurf.Draw(NULL, FALSE, NULL, pOptGfxSet) ;
                    }
                }
  if(!pEdge1 && pEdgeuse1) 
                { SmSurface      * pSurface = pEdgeuse1->GetFace() ? pEdgeuse1->GetFace()->GetSurface() : NULL ;
                 SmBSplineCurve * pUVTrimCurve ;
                  pEdgeuse1->GetOrCreateUVTrimCurve(pUVTrimCurve) ;
                 if(pUVTrimCurve && pSurface)
                   { SmCrvOnSurf sCrvOnSurf(*pUVTrimCurve, *pSurface) ;
                     sCrvOnSurf.Draw(NULL, FALSE, NULL, pOptGfxSet) ;
                   }
               }

  // draw vertex
  double dPointSize = smgfx_SetPointSize(2.0 * smgfx_GetHotPointSize()) ;
  if(pVertex)  { pVertex->Draw() ; }

  // now the neighbors
  smgfx_SetColor(sColor, pOptGfxSet) ;
  smgfx_SetLineWidth(dLineWidth, pOptGfxSet) ;
  smgfx_SetPointSize(dPointSize, pOptGfxSet) ;

  // draw all the neighbor faces with crosshatch
  for(ii=0;ii<sFaces.GetSize();ii++)
    {
      SmFace *pF = sFaces[ii] ;
      pF->Draw(SM_DM_CROSSHATCH) ;
  
    } // end iter every face
  
  // draw all the neighbor edges 
  for(ii=0;ii<sEdges.GetSize();ii++)
    {
       SmEdge *pE = sEdges[ii] ;
       pE->Draw() ;
    }

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmGap::DrawNeighbors

/*******************************************************************//**
PURPOSE: SmGap Derived class simple access virtual methods
    
NOTES: 
***********************************************************************/
SmVertex    * SmVertexEdgeGap::GetVertex()             const { return m_pVertexuse ? m_pVertexuse->GetVertex()             : NULL ; } 
SmVertexuse * SmVertexEdgeGap::GetVertexuse()          const { return m_pVertexuse ? (SmVertexuse *)m_pVertexuse           : NULL ; }  
SmEdge      * SmVertexEdgeGap::GetEdge(ULONG lIndx)             const { return lIndx == 0 ? (m_pVertexuse ? m_pVertexuse->GetEdgeuse()->GetEdge() : NULL) : NULL ; }  
SmEdgeuse   * SmVertexEdgeGap::GetEdgeuse(ULONG lIndx)          const { return lIndx == 0 ? (m_pVertexuse ? m_pVertexuse->GetEdgeuse()            : NULL) : NULL ; }  
SmBoolean     SmVertexEdgeGap::IsKindOf( SM_TYPE t )   const { return ((SmVertexEdgeGap_TYPE == t) ? TRUE : SmGap::IsKindOf( (t) )); }

SmVertex    * SmVertexFaceTrimCurveGap::GetVertex()    const { return m_pVertexuse ? m_pVertexuse->GetVertex()             : NULL ; }
SmVertexuse * SmVertexFaceTrimCurveGap::GetVertexuse() const { return m_pVertexuse ? (SmVertexuse *)m_pVertexuse           : NULL ; }
SmEdgeuse   * SmVertexFaceTrimCurveGap::GetEdgeuse(ULONG lIndx) const { return lIndx == 0 ? (m_pVertexuse ? m_pVertexuse->GetEdgeuse() : NULL) : NULL ; }
SmFace      * SmVertexFaceTrimCurveGap::GetFace()      const { return (m_pVertexuse && m_pVertexuse->GetFaceuse()) ? m_pVertexuse->GetFaceuse()->GetFace() : NULL ; }
SmBoolean     SmVertexFaceTrimCurveGap::IsKindOf( SM_TYPE t )     const { return ((SmVertexFaceTrimCurveGap_TYPE == t) ? TRUE : SmGap::IsKindOf( (t) )); }

SmVertex    * SmVertexFaceGap::GetVertex()             const { return m_pVertexuse ? m_pVertexuse->GetVertex()             : NULL ; }
SmVertexuse * SmVertexFaceGap::GetVertexuse()          const { return m_pVertexuse ? (SmVertexuse *)m_pVertexuse           : NULL ; }
SmFace      * SmVertexFaceGap::GetFace()               const { return (m_pVertexuse && m_pVertexuse->GetFaceuse()) ? m_pVertexuse->GetFaceuse()->GetFace() : NULL ; }
SmBoolean     SmVertexFaceGap::IsKindOf( SM_TYPE t )   const { return ((SmVertexFaceGap_TYPE == t) ? TRUE : SmGap::IsKindOf( (t) )); }

SmEdge      * SmEdgeEdgeGap::GetEdge(ULONG lIndx)               const { return   lIndx == 0 ? (m_pEdgeuse0 ? m_pEdgeuse0->GetEdge() : NULL) 
                                                                               : lIndx == 1 ? (m_pEdgeuse1 ? m_pEdgeuse1->GetEdge() : NULL) 
                                                                               : NULL ; 
                                                                      } 
SmEdgeuse   * SmEdgeEdgeGap::GetEdgeuse(ULONG lIndx)            const { return   lIndx == 0 ? (m_pEdgeuse0 ? (SmEdgeuse *)m_pEdgeuse0 : NULL) 
                                                                               : lIndx == 1 ? (m_pEdgeuse1 ? (SmEdgeuse *)m_pEdgeuse1 : NULL) 
                                                                               : NULL ; 
                                                                      }
SmBoolean     SmEdgeEdgeGap::IsKindOf( SM_TYPE t )              const { return ((SmEdgeEdgeGap_TYPE == t) ? TRUE : SmGap::IsKindOf( (t) )); }


SmEdge      * SmEdgeFaceTrimCurveGap::GetEdge(ULONG lIndx)      const { return lIndx == 0 ? (m_pEdgeuse ? m_pEdgeuse->GetEdge()   : NULL) : NULL ; } 
SmEdgeuse   * SmEdgeFaceTrimCurveGap::GetEdgeuse(ULONG lIndx)   const { return lIndx == 0 ? (m_pEdgeuse ? (SmEdgeuse *)m_pEdgeuse : NULL) : NULL ; }
SmFace      * SmEdgeFaceTrimCurveGap::GetFace()        const { return m_pEdgeuse ? m_pEdgeuse->GetFace()   : NULL ; }

SmBoolean     SmEdgeFaceTrimCurveGap::IsKindOf( SM_TYPE t )     const { return ((SmEdgeFaceTrimCurveGap_TYPE == t) ? TRUE : SmGap::IsKindOf( (t) )); }

SmEdge      * SmEdgeFaceGap::GetEdge(ULONG lIndx)               const { return lIndx == 0 ? (m_pEdgeuse ? m_pEdgeuse->GetEdge()   : NULL) : NULL ; }
SmEdgeuse   * SmEdgeFaceGap::GetEdgeuse(ULONG lIndx)            const { return lIndx == 0 ? (m_pEdgeuse ? (SmEdgeuse *)m_pEdgeuse : NULL) : NULL ; }
SmFace      * SmEdgeFaceGap::GetFace()                 const { return m_pEdgeuse ? m_pEdgeuse->GetFace()   : NULL ; }

SmBoolean     SmEdgeFaceGap::IsKindOf( SM_TYPE t )     const { return ((SmEdgeFaceGap_TYPE == t) ? TRUE : SmGap::IsKindOf( (t) )); }


SmPoint2d     SmVertexFaceTrimCurveGap::GetFaceUV()    const { if(   m_pVertexuse
                                                                  && m_pVertexuse->GetEdgeuse()
                                                                  && m_pVertexuse->GetEdgeuse()->GetUVTrimCurve())
                                                                 { SmPoint3d sPt3d ;
                                                                   m_pVertexuse->GetEdgeuse()->GetUVTrimCurve()->EvaluatePoint(m_dUVTrimCurveT, sPt3d) ; 
                                                                   return(SmPoint2d(sPt3d.x, sPt3d.y)) ;
                                                                 } 
                                                               else 
                                                                 { return SmPoint2d() ; }
                                                             }
SmPoint2d     SmEdgeFaceTrimCurveGap::GetFaceUV()      const { if(   m_pEdgeuse
                                                                  && m_pEdgeuse->GetUVTrimCurve())
                                                                 { SmPoint3d sPt3d ;
                                                                   m_pEdgeuse->GetUVTrimCurve()->EvaluatePoint(m_dUVTrimCurveT, sPt3d) ; 
                                                                   return(SmPoint2d(sPt3d.x, sPt3d.y)) ;
                                                                 } 
                                                               else 
                                                                 { return SmPoint2d() ; }
                                                             }

/*******************************************************************//**
PURPOSE: Search for matching gap.
    
NOTES: If the element is found set the nFoundIndex and return TRUE, 
    otherwise return FALSE.
***********************************************************************/
SmBoolean SmGapArray::Find
 (const SmGap & crGap,        // in : target gap
  ULONG       & lIndex)       // out: found index
 const
{
  ULONG ii ;

  // for every gap
  for (ii=0; ii<this->GetSize(); ii++) 
    {
      SmGap *pThisGap = m_sGaps[ii] ;

      // look for a match
      if (*pThisGap == crGap) 
        {
          lIndex = ii;
          return TRUE;
        }
    } // end iter every gap

  // arrive here with no matches
  return FALSE;

} // end SmTArray<TYPE>::FindElement
 
/*******************************************************************//**
PURPOSE: Extract lists of all vertices, edges, and edgeuses referenced in
            this list of gaps

NOTES: increments unlocked mark value
***********************************************************************/
SmStatus SmGapArray::GetVerticesAndEdges
  (SmTArray<SmVertex *>  & rVertices,     // out: vertices referenced in this gap list
   SmTArray<SmEdge *>    & rEdges,        // out: edges referenced in this gap list 
   SmTArray<SmEdgeuse *> & rEdgeuses)     // out: edgeuses referenced in this gap list
  const
{
  // init output
  rEdges.ReSet() ;
  rVertices.ReSet() ;

  // no work
  if(GetSize() == 0)
    { return SM_SUCCESS ; }

  // get context from first gap object
  SmGap     * pFirstGap = m_sGaps[0] ;
  SmContext * pContext = (SmContext *)(  pFirstGap->GetVertex() ? pFirstGap->GetVertex()->GetContext()
                                       : pFirstGap->GetEdge()   ? pFirstGap->GetEdge()->GetContext()
                                       : pFirstGap->GetEdgeuse()? pFirstGap->GetEdgeuse()->GetContext()
                                       : pFirstGap->GetFace()   ? pFirstGap->GetFace()->GetContext()
                                       : NULL) ;

  // something odd without a context
  if(pContext == NULL)
    { return(SM_ERR) ; }

  // use mark to keep output lists unique
  SmNewMarkAndLock sMark(pContext, SM_MT_ALLMARKS) ; // increments newly locked mark
  SmMarkType eMarkType = sMark.GetMarkType() ;

  // for every gap
  ULONG ii ;
  for(ii=0;ii<GetSize();ii++)
    {
      SmGap     * pGap     = m_sGaps[ii] ;
      SmVertex  * pVertex  = pGap->GetVertex() ;
      SmEdge    * pEdge    = pGap->GetEdge() ;
      SmEdgeuse * pEdgeuse = pGap->GetEdgeuse() ;

      // set output
      if(pVertex && !pVertex->IsMarked(eMarkType))
        { 
          rVertices.Add(pVertex) ;
          pVertex->Mark(eMarkType) ;
        }

      if(pEdge && !pEdge->IsMarked(eMarkType))
        { 
          rEdges.Add(pEdge) ;
          pEdge->Mark(eMarkType) ;
        }

      if(pEdgeuse && !pEdgeuse->IsMarked(eMarkType))
        { 
          rEdgeuses.Add(pEdgeuse) ;
          pEdgeuse->Mark(eMarkType) ;
        }
    } // end iter every gap

  // all done
  return(SM_SUCCESS) ;

} // end SmGapArray::GetVerticesAndEdges

/*******************************************************************//**
PURPOSE: Draw a SmGapArray Topology Object's gap graphics

NOTES:
***********************************************************************/
SmDisplayList * SmGapArray::Draw
 (SmGfxArraySet   *pOptGfxSet)     // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
 const                             //      NULL to ignore. default:[NULL]
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // start new displayList (unless one is already open)
  // use gap color and draw sizes rather than default ones
  double dLineWidth = smgfx_GetGapLineWidth() ;
  double dPointSize = smgfx_GetGapPointSize() ;
  smgfx_Open(smgfx_GetGapPointColor(), &dLineWidth, &dPointSize, FALSE, pOptGfxSet);
             
  // Gap Graphics
  ULONG ii ;
  for(ii=0;ii<GetSize();ii++)
    {
      SmGap *pGap = m_sGaps[ii] ;

      pGap->Draw(.00001,        // in : SM_USE_NEWTOL: not used
                                //      OldTol       : override tol value, 0.0 to ignore. default:[SM_XSECT_TOL_3D]
                 pOptGfxSet) ;  // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                //      NULL to ignore. default:[NULL]
    } // end iter every gap

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmGapArray::Draw

/*******************************************************************//**
PURPOSE: Pretty print SmGapArray data

NOTES:
***********************************************************************/
void SmGapArray::Dump() const
{
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // label and maxGap value
  smos_sprintf(sBuff, _T("\n  Gap List - Length:[%lu], MaxGap3d:[%16.16lf]"),
             m_sGaps.GetSize(),
             m_sGaps[m_lMaxGap3d]->GetLength()) ;
  smos_WriteBuffer(sBuff);

  // Gap reports
  for(ii=0;ii<GetSize();ii++)
    {
      SmGap *pGap = m_sGaps.GetDataArray()[ii] ;
      pGap->Dump(ii) ;

    } // end iter every gap

} // end SmGapArray::Dump

/*******************************************************************//**
END SmGapArray Methods
***********************************************************************/

/*******************************************************************//**
BEGIN SmGapSample Methods
***********************************************************************/

/*******************************************************************//**
PURPOSE: SmGapSample assignment operator

NOTES:
***********************************************************************/
SmGapSample & SmGapSample::operator=
  (const SmGapSample &crOther)  
{
  if(this == &crOther) return *this ;
  m_sThisPos    = crOther.m_sThisPos ;  
  m_sOtherPos   = crOther.m_sOtherPos ; 
  m_sThisParam  = crOther.m_sThisParam ;
  m_sOtherParam = crOther.m_sOtherParam ;
  m_sXSectTol3d = crOther.m_sXSectTol3d ;
  m_bIsInTol    = crOther.m_bIsInTol ;
  if(!m_bIsInTol) { m_sTolPoint   = crOther.m_sTolPoint ; }
  else            { m_sTolPoint.SetUninitialized() ; }
  return *this ;

} // end SmGapSample::operator= assignment operator

/*******************************************************************//**
PURPOSE: SmGapSample Equality operator 

NOTES: 
***********************************************************************/
SmBoolean SmGapSample::operator==
  (const SmGapSample& crOther) 
 const
{
  return(   m_sThisPos     == crOther.m_sThisPos
         && m_sOtherPos    == crOther.m_sOtherPos  
         && m_sThisParam   == crOther.m_sThisParam 
         && m_sOtherParam  == crOther.m_sOtherParam) ;

} // end SmGapSample::operator==

/*******************************************************************//**
PURPOSE: Compute and save m_bIsInTol and m_sTolPoint for saved 
           SmGapSample and dXSectTol3d values

NOTES: When all values in the gap are initialized, 
  compute and save the m_bIsInTol state and when the gap is out of tol
  compute and save a point which is exactly tol away from the ThisPos
  along the ThisPos to OtherPos vector.  TolPoints are useful
  for visualizing the gap in relationship to the tolerance.
***********************************************************************/
void SmGapSample::SetTolPoint() 
{
  // no work - m_sThisPos or m_sOtherPos still undefined
  if( !(   m_sThisPos.x!=SM_UNDEF_DOUBLE 
        && m_sThisPos.y!=SM_UNDEF_DOUBLE 
        && m_sThisPos.z!=SM_UNDEF_DOUBLE
        && m_sThisPos.z!=NL_NOZ
        && m_sOtherPos.x!=SM_UNDEF_DOUBLE 
        && m_sOtherPos.y!=SM_UNDEF_DOUBLE 
        && m_sOtherPos.z!=SM_UNDEF_DOUBLE
        && m_sOtherPos.z!=NL_NOZ) )
   { return ; } 

  // locals
  if(m_sXSectTol3d <= 0.0) m_sXSectTol3d = SM_XSECT_TOL_3D ;
  SmXSectTol3d dXSectTol3dSq = m_sXSectTol3d * m_sXSectTol3d ;
  double dGapSq        = m_sThisPos.DistanceBetweenSquared(m_sOtherPos) ;

  // check for in/out tolerance gaps
  m_bIsInTol = (dGapSq <= dXSectTol3dSq + m_sXSectTol3d*SM_EFF_ZERO) ;

  // Point at the tolerance bound
  if(m_bIsInTol)
    {
      m_sTolPoint.SetUninitialized() ;
    }
  else
    {
      double dParam = smos_Sqrt(dXSectTol3dSq / dGapSq) ;
      m_sTolPoint = (1 - dParam) * m_sThisPos + dParam * m_sOtherPos ;
    }

} // end SmGapSample::SetTolPoint

/*******************************************************************//**
PURPOSE: Pretty print SmGapArray data

NOTES:
***********************************************************************/
void SmGapSample::Dump
  (SmGapSampleType eType,          // in : default:[SM_GS_SRF_SRF]
   SmBoolean       bAbbrev,        // in : default:[FALSE]
   ULONG          *pOptLabel)      // in : NULL to ignore, default:[NULL]
 const
{
  SM_ASSERT_BREAK(   m_sThisPos.x!=SM_UNDEF_DOUBLE 
                  && m_sThisPos.y!=SM_UNDEF_DOUBLE 
                  && m_sThisPos.z!=SM_UNDEF_DOUBLE
                  && m_sThisPos.z!=NL_NOZ) ;
     
  SM_ASSERT_BREAK(   m_sOtherPos.x!=SM_UNDEF_DOUBLE 
                  && m_sOtherPos.y!=SM_UNDEF_DOUBLE 
                  && m_sOtherPos.z!=SM_UNDEF_DOUBLE
                  && m_sOtherPos.z!=NL_NOZ) ;
                                                     
  TCHAR sBuff[SM_TBLOCK_SIZE];

  double dGapLength = m_sThisPos.DistanceBetween(m_sOtherPos) ;

  // optional numeric label
  if(pOptLabel) { smos_sprintf(sBuff, _T("\n    %3lu :"), *pOptLabel) ; }
  else          { smos_sprintf(sBuff, _T("%s"), _T("\n    :")) ; }
  smos_WriteBuffer(sBuff);


  if ( bAbbrev)
    {
      // gap and from xyz
      smos_sprintf(sBuff,_T(" %s GapLength:[%6.6lf], %s(%6.6lf), FromXYZ:[%6.6lf %6.6lf %6.6lf]"),
                   m_eGapDropType == SM_GD_UNKNOWN  ? _T("UnInit")
                 : m_eGapDropType == SM_GD_NORMAL   ? _T("Normal")
                 : m_eGapDropType == SM_GD_BOUNDARY ? _T("Bndary")
                 : m_eGapDropType == SM_GD_NO_DROP  ? _T("NoDrop") : _T("BadLabel"),
                 dGapLength, 
                 m_bIsInTol ? _T("In Tol") : _T("OutTol"), 
                 (double)m_sXSectTol3d,
                 m_sThisPos.x,  m_sThisPos.y,  m_sThisPos.z) ;
      smos_WriteBuffer(sBuff) ;

      // from param
      switch(eType)
        {
          case SM_GS_CRV_PT   :
          case SM_GS_CRV_CRV  :
          case SM_GS_CRV_SRF  : smos_sprintf(sBuff,_T(" FromU:[%6.6lf]"), m_sThisParam.x) ;
                                break ;
          case SM_GS_SRF_PT   :
          case SM_GS_SRF_CRV  :
          case SM_GS_SRF_SRF  : smos_sprintf(sBuff,_T(" FromUV:[%6.6lf %6.6lf]"), m_sThisParam.x, m_sThisParam.y) ;
                                break ;
        }
      smos_WriteBuffer(sBuff) ;

      // to xyz
      smos_sprintf(sBuff,_T(" , to xyz:[%6.6lf %6.6lf %6.6lf]"),
                 m_sOtherPos.x,  m_sOtherPos.y,  m_sOtherPos.z) ;
      smos_WriteBuffer(sBuff) ;

      // to param
      switch(eType)
        {
          case SM_GS_CRV_PT   :
          case SM_GS_SRF_PT   : 
                                break ;
          case SM_GS_CRV_CRV  :
          case SM_GS_SRF_CRV  : smos_sprintf(sBuff,_T("  ToU:[%6.6lf]"), m_sOtherParam.x) ;
                                break ;
          case SM_GS_CRV_SRF  : 
          case SM_GS_SRF_SRF  : smos_sprintf(sBuff,_T("  ToUV:[%6.6lf %6.6lf]"), m_sOtherParam.x, m_sOtherParam.y) ;
                                break ;
        }
      smos_WriteBuffer(sBuff) ;
      // smos_WriteBuffer(_T("\n")) ;
    }          
  else // not bAbbrev
    {
      // gap and from xyz
      smos_sprintf(sBuff,_T(" %s GapLength:[%16.16lf] %s(%6.6lf), FromXYZ:[%16.16lf %16.16lf %16.16lf]"),
                   m_eGapDropType == SM_GD_UNKNOWN  ? _T("UnInit")
                 : m_eGapDropType == SM_GD_NORMAL   ? _T("Normal")
                 : m_eGapDropType == SM_GD_BOUNDARY ? _T("Bndary")
                 : m_eGapDropType == SM_GD_NO_DROP  ? _T("NoDrop") : _T("BadLabel"),
                 dGapLength, 
                 m_bIsInTol ? _T("In Tol") : _T("OutTol"), 
                 (double)m_sXSectTol3d,
                 m_sThisPos.x,  m_sThisPos.y,  m_sThisPos.z) ;
      smos_WriteBuffer(sBuff) ;

      // from param
      switch(eType)
        {
          case SM_GS_CRV_PT   :
          case SM_GS_CRV_CRV  :
          case SM_GS_CRV_SRF  : smos_sprintf(sBuff,_T(" FromU:[%16.16lf]"), m_sThisParam.x) ;
                                break ;
          case SM_GS_SRF_PT   :
          case SM_GS_SRF_CRV  :
          case SM_GS_SRF_SRF  : smos_sprintf(sBuff,_T(" FromUV:[%16.16lf %16.16lf]"), m_sThisParam.x, m_sThisParam.y) ;
                                break ;
        }
      smos_WriteBuffer(sBuff) ;

      // to xyz
      smos_sprintf(sBuff,_T(" , to xyz:[%16.16lf %16.16lf %16.16lf]"),
                 m_sOtherPos.x,  m_sOtherPos.y,  m_sOtherPos.z) ;
      smos_WriteBuffer(sBuff) ;

      // to param
      switch(eType)
        {
          case SM_GS_CRV_PT   :
          case SM_GS_SRF_PT   : 
                                break ;
          case SM_GS_CRV_CRV  :
          case SM_GS_SRF_CRV  : smos_sprintf(sBuff,_T("  ToU:[%16.16lf]"), m_sOtherParam.x) ;
                                break ;
          case SM_GS_CRV_SRF  : 
          case SM_GS_SRF_SRF  : smos_sprintf(sBuff,_T("  ToUV:[%16.16lf %16.16lf]"), m_sOtherParam.x, m_sOtherParam.y) ;
                                break ;
        }
      smos_WriteBuffer(sBuff) ;
     // smos_WriteBuffer(_T("\n")) ;
    }          

} // end SmGapSample::Dump

/*******************************************************************//**
PURPOSE: pVectorOrigin == NULL, draw point icon
            pVectorOrigin != NULL, draw vector starting at pVectorOrigin

NOTES: 
***********************************************************************/
SmDisplayList * SmGapSample::Draw
 (SmGfxArraySet   *pOptGfxSet)     // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
 const                             //      NULL to ignore. default:[NULL]
{
  SmDisplayList *pRtn = NULL ;

  SM_ASSERT_BREAK(   m_sThisPos.x!=SM_UNDEF_DOUBLE 
                  && m_sThisPos.y!=SM_UNDEF_DOUBLE 
                  && m_sThisPos.z!=SM_UNDEF_DOUBLE
                  && m_sThisPos.z!=NL_NOZ) ;
     
  SM_ASSERT_BREAK(   m_sOtherPos.x!=SM_UNDEF_DOUBLE 
                  && m_sOtherPos.y!=SM_UNDEF_DOUBLE 
                  && m_sOtherPos.z!=SM_UNDEF_DOUBLE
                  && m_sOtherPos.z!=NL_NOZ) ;
     
#ifdef SM_GFX_CODE

  // can't currently add gaps to pick list because gaps are not derived from SmObject
  //      // when asked - add this Point to UI pick list
  //      if(bAddToUIPickList) 
  //        { sm_GraphicsAddToBrepList(this) ; }

  // start new displayList (unless one is already open)
  // use gap color and draw sizes rather than default ones
  double dLineWidth = 1 ;
  double dPointSize = 3 ;
  smgfx_Open(smgfx_GetInTolGapColor(), &dLineWidth, &dPointSize, FALSE, pOptGfxSet);

  dLineWidth        = smgfx_OutputLineWidth(dLineWidth, pOptGfxSet) ;
  dPointSize        = smgfx_OutputPointSize(dLineWidth+2, pOptGfxSet) ;
  
  // draw one or two lines
  if(m_bIsInTol)
    {
      smgfx_OutputLine(m_sThisPos.x,  m_sThisPos.y,  m_sThisPos.z,
                       m_sOtherPos.x, m_sOtherPos.y, m_sOtherPos.z, pOptGfxSet) ;
      smgfx_OutputPoint(m_sThisPos.x,  m_sThisPos.y,  m_sThisPos.z, pOptGfxSet) ;
    }
  else // gap exceeds current tolerance
    {
      SmVector3d sColor = smgfx_OutputColor( smgfx_GetInTolGapColor() , pOptGfxSet) ;
      smgfx_OutputLine(m_sThisPos.x,  m_sThisPos.y,  m_sThisPos.z,
                       m_sTolPoint.x, m_sTolPoint.y, m_sTolPoint.z, pOptGfxSet) ;

      smgfx_OutputColor( smgfx_GetOutTolGapColor() , pOptGfxSet) ;
      smgfx_OutputLine(m_sTolPoint.x, m_sTolPoint.y, m_sTolPoint.z,
                       m_sOtherPos.x, m_sOtherPos.y, m_sOtherPos.z, pOptGfxSet) ;
      smgfx_OutputColor(sColor, pOptGfxSet) ;
    }

  smgfx_OutputLineWidth(dLineWidth, pOptGfxSet) ;
  smgfx_OutputPointSize(dPointSize, pOptGfxSet) ;

  // all done
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF1(pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmGapSample::Draw(const SmVector3d *pVectorOrigin)

/*******************************************************************//**
END SmGapSample Methods
***********************************************************************/

/*******************************************************************//**
BEGIN SmLocalInterval Methods
***********************************************************************/

/*******************************************************************//**
PURPOSE: Pretty print SmGapArray data

NOTES:
***********************************************************************/
void SmLocalInterval::Dump
  (SmBoolean bAbbrev,         // in : TRUE = Shorten reported precision, FALSE use full precision
   ULONG *pOptLabel)          // in : optional numeric label
 const
{
  SmLocalInterval *pLocalIvl = ((SmLocalInterval*)this) ;

  SM_ASSERT_BREAK(   pLocalIvl->m_sMin.GetThisPos().x!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMin.GetThisPos().y!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMin.GetThisPos().z!=SM_UNDEF_DOUBLE
                  && pLocalIvl->m_sMin.GetThisPos().z!=NL_NOZ) ;
     
  SM_ASSERT_BREAK(   pLocalIvl->m_sMin.GetOtherPos().x!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMin.GetOtherPos().y!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMin.GetOtherPos().z!=SM_UNDEF_DOUBLE
                  && pLocalIvl->m_sMin.GetOtherPos().z!=NL_NOZ) ;
                                                     
  SM_ASSERT_BREAK(   pLocalIvl->m_sMax.GetThisPos().x!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMax.GetThisPos().y!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMax.GetThisPos().z!=SM_UNDEF_DOUBLE
                  && pLocalIvl->m_sMax.GetThisPos().z!=NL_NOZ) ;
     
  SM_ASSERT_BREAK(   pLocalIvl->m_sMax.GetOtherPos().x!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMax.GetOtherPos().y!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMax.GetOtherPos().z!=SM_UNDEF_DOUBLE
                  && pLocalIvl->m_sMax.GetOtherPos().z!=NL_NOZ) ;
                                                     
  TCHAR sBuff[SM_TBLOCK_SIZE];

  double dMinGap = pLocalIvl->m_sMin.GetThisPos().DistanceBetween(pLocalIvl->m_sMin.GetOtherPos()) ;
  double dMaxGap = pLocalIvl->m_sMax.GetThisPos().DistanceBetween(pLocalIvl->m_sMax.GetOtherPos()) ;

  // optional numeric label
  if(pOptLabel) { smos_sprintf(sBuff, _T("\n    %3lu :"), *pOptLabel) ; }
  else          { smos_sprintf(sBuff, _T("%s"), _T("\n    :")) ; }
  smos_WriteBuffer(sBuff);


  if ( bAbbrev)
    {
      // short numbers
      smos_sprintf(sBuff,_T(" UIvl[%6.6lf %6.6lf], gaps[%6.6lf %6.6lf], dXSectTol3d[%6.6lf %6.6lf], ThisMin[%6.6lf %6.6lf %6.6lf], OtherMin[%6.6lf %6.6lf %6.6lf], ThisMax[%6.6lf %6.6lf %6.6lf], OtherMax[%6.6lf %6.6lf %6.6lf], VIvl[%6.6lf %6.6lf]"),
                 pLocalIvl->m_sMin.GetThisParam().x, pLocalIvl->m_sMax.GetThisParam().x,
                 dMinGap, dMaxGap, (double)pLocalIvl->m_sMin.GetXSectTol3d(), (double)pLocalIvl->m_sMax.GetXSectTol3d(),
                 pLocalIvl->m_sMin.GetThisPos().x, pLocalIvl->m_sMin.GetThisPos().y, pLocalIvl->m_sMin.GetThisPos().z,
                 pLocalIvl->m_sMin.GetOtherPos().x, pLocalIvl->m_sMin.GetOtherPos().y, pLocalIvl->m_sMin.GetOtherPos().z,
                 pLocalIvl->m_sMax.GetThisPos().x, pLocalIvl->m_sMax.GetThisPos().y, pLocalIvl->m_sMax.GetThisPos().z,
                 pLocalIvl->m_sMax.GetOtherPos().x, pLocalIvl->m_sMax.GetOtherPos().y, pLocalIvl->m_sMax.GetOtherPos().z,
                 pLocalIvl->m_sMin.GetThisParam().y, pLocalIvl->m_sMax.GetThisParam().y) ;
      smos_WriteBuffer(sBuff) ;
    }          
  else // not bAbbrev
    {
      // short numbers
      smos_sprintf(sBuff,_T(" UIvl[%16.16lf %16.16lf], gaps[%16.16lf %16.16lf], dXSectTol3d[%16.16lf %16.16lf], ThisMin[%16.16lf %16.16lf %16.16lf], OtherMin[%16.16lf %16.16lf %16.16lf], ThisMax[%16.16lf %16.16lf %16.16lf], OtherMax[%16.16lf %16.16lf %16.16lf], VIvl[%16.16lf %16.16lf]"),
                 pLocalIvl->m_sMin.GetThisParam().x, pLocalIvl->m_sMax.GetThisParam().x,
                 dMinGap, dMaxGap, (double)pLocalIvl->m_sMin.GetXSectTol3d(), (double)pLocalIvl->m_sMax.GetXSectTol3d(),
                 pLocalIvl->m_sMin.GetThisPos().x, pLocalIvl->m_sMin.GetThisPos().y, pLocalIvl->m_sMin.GetThisPos().z,
                 pLocalIvl->m_sMin.GetOtherPos().x, pLocalIvl->m_sMin.GetOtherPos().y, pLocalIvl->m_sMin.GetOtherPos().z,
                 pLocalIvl->m_sMax.GetThisPos().x, pLocalIvl->m_sMax.GetThisPos().y, pLocalIvl->m_sMax.GetThisPos().z,
                 pLocalIvl->m_sMax.GetOtherPos().x, pLocalIvl->m_sMax.GetOtherPos().y, pLocalIvl->m_sMax.GetOtherPos().z,
                 pLocalIvl->m_sMin.GetThisParam().y, pLocalIvl->m_sMax.GetThisParam().y) ;
      smos_WriteBuffer(sBuff) ;
    }          

} // end SmLocalInterval::Dump

/*******************************************************************//**
PURPOSE: Draw the local neighborhood boundaries (an interval of
  the gap function where the gap vectors are all less than tolerance) 
  boundaries as two pairs of end points connected by a line.

NOTES: If given an optional pThis curve pointer draw the
  curve over the local interval in a heavy weight.
***********************************************************************/
SmDisplayList * SmLocalInterval::Draw
 (const SmCurve  *pThisCurve,    // in : Draw This curve over the local neighborhood interval
                                 //      NULL to ignore, default:[NULL]
  SmGfxArraySet  *pOptGfxSet)    // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
 const                           //      NULL to ignore. default:[NULL]
{
  SmLocalInterval *pLocalIvl = ((SmLocalInterval*)this) ;
  SmDisplayList   *pRtn = NULL ;

  SM_ASSERT_BREAK(   pLocalIvl->m_sMin.GetThisPos().x!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMin.GetThisPos().y!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMin.GetThisPos().z!=SM_UNDEF_DOUBLE
                  && pLocalIvl->m_sMin.GetThisPos().z!=NL_NOZ) ;
     
  SM_ASSERT_BREAK(   pLocalIvl->m_sMin.GetOtherPos().x!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMin.GetOtherPos().y!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMin.GetOtherPos().z!=SM_UNDEF_DOUBLE
                  && pLocalIvl->m_sMin.GetOtherPos().z!=NL_NOZ) ;
                                                     
  SM_ASSERT_BREAK(   pLocalIvl->m_sMax.GetThisPos().x!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMax.GetThisPos().y!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMax.GetThisPos().z!=SM_UNDEF_DOUBLE
                  && pLocalIvl->m_sMax.GetThisPos().z!=NL_NOZ) ;
     
  SM_ASSERT_BREAK(   pLocalIvl->m_sMax.GetOtherPos().x!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMax.GetOtherPos().y!=SM_UNDEF_DOUBLE 
                  && pLocalIvl->m_sMax.GetOtherPos().z!=SM_UNDEF_DOUBLE
                  && pLocalIvl->m_sMax.GetOtherPos().z!=NL_NOZ) ;
                                                     
#ifdef SM_GFX_CODE

  // can't currently add gaps to pick list because gaps are not derived from SmObject
  //      // when asked - add this Point to UI pick list
  //      if(bAddToUIPickList) 
  //        { sm_GraphicsAddToBrepList(this) ; }

  // start new displayList (unless one is already open)
  // use gap color and draw sizes rather than default ones
  smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);

  // draw 4 points and 2 lines marking the local neighborhood boundary
  smgfx_OutputPoint(pLocalIvl->m_sMin.GetThisPos().x,   pLocalIvl->m_sMin.GetThisPos().y,   pLocalIvl->m_sMin.GetThisPos().z, pOptGfxSet) ;
  smgfx_OutputPoint(pLocalIvl->m_sMin.GetOtherPos().x,  pLocalIvl->m_sMin.GetOtherPos().y,  pLocalIvl->m_sMin.GetOtherPos().z, pOptGfxSet) ;
  smgfx_OutputPoint(pLocalIvl->m_sMax.GetThisPos().x,   pLocalIvl->m_sMax.GetThisPos().y,   pLocalIvl->m_sMax.GetThisPos().z, pOptGfxSet) ;
  smgfx_OutputPoint(pLocalIvl->m_sMax.GetOtherPos().x,  pLocalIvl->m_sMax.GetOtherPos().y,  pLocalIvl->m_sMax.GetOtherPos().z, pOptGfxSet) ;
  smgfx_OutputLine(pLocalIvl->m_sMin.GetThisPos().x,   pLocalIvl->m_sMin.GetThisPos().y,   pLocalIvl->m_sMin.GetThisPos().z,
                   pLocalIvl->m_sMin.GetOtherPos().x,  pLocalIvl->m_sMin.GetOtherPos().y,  pLocalIvl->m_sMin.GetOtherPos().z, pOptGfxSet) ;
  smgfx_OutputLine(pLocalIvl->m_sMax.GetThisPos().x,   pLocalIvl->m_sMax.GetThisPos().y,   pLocalIvl->m_sMax.GetThisPos().z,
                   pLocalIvl->m_sMax.GetOtherPos().x,  pLocalIvl->m_sMax.GetOtherPos().y,  pLocalIvl->m_sMax.GetOtherPos().z, pOptGfxSet) ;

  // when given a curve - draw that a little thicker
  if(pThisCurve)
    {
      double dLineWidth = smgfx_GetLineWidth() ;
      SmExtent1d uInterval = pLocalIvl->GetThisUInterval();
      smgfx_SetLineWidth(dLineWidth + 3, pOptGfxSet) ;
      pThisCurve->Draw( &uInterval, FALSE , NULL, pOptGfxSet) ;
      smgfx_SetLineWidth(dLineWidth, pOptGfxSet) ;
    }

  // all done
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF2(pThisCurve, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmLocalInterval::Draw

/*******************************************************************//**
END SmLocalInterval Methods
***********************************************************************/


/*******************************************************************//**
BEGIN SmGapFunction Methods
***********************************************************************/

/*******************************************************************//**
PURPOSE: SmGapFunction Default Constructor

NOTES:
***********************************************************************/
SmGapFunction::SmGapFunction
 (SmGapSampleType eType,             // in : Type so derived classes can mark their type here - yuk!
  const SmXSectTol3d & rXSectTol3d,  // in : XSectTol3d used to judge in/out property for a gap length.
  const SmExtent1d   & rSampleIvlU,  // in : 1st param domain range - used for Curves and Surfaces
  const SmExtent1d   & rSampleIvlV,  // in : 2nd param domain range - only used for Surfaces
  ULONG           lSampleCntU,       // in : number of evenly spaced samples in 1st param dim - used for Curves and Surfaces      
  ULONG           lSampleCntV)       // in : number of evenly spaced samples in 2nd param dim - only used for Surfaces      
: m_eSampleType(eType),
  m_sXSectTol3d(rXSectTol3d),
  m_sSampleIvlU(rSampleIvlU),
  m_sSampleIvlV(rSampleIvlV),
  m_bAreSamplesSet(FALSE),
  m_bIsLocalNeighborhoodSet(FALSE),
  m_lSampleCntU(0),
  m_lSampleCntV(0),
  m_sSampleGaps(),
  m_lMaxSampleIndex(0),
  m_lMinSampleIndex(0),
  m_bAllNormalGaps(UNSURE),
  m_sOtherIvlU(),
  m_sOtherIvlV()
{
  SetSampleCnt(lSampleCntU, lSampleCntV) ; 

} // end SmGapFunction::SmGapFunction default constructor

/**************************************************************
PURPOSE --- Copy Constructor

USAGE NOTES ---
**************************************************************/
SmGapFunction::SmGapFunction     // eff: copy constructor
 (const SmGapFunction &crOther)  // in : object to copy
{
  m_eSampleType               = crOther.m_eSampleType ;
  m_sXSectTol3d               = crOther.m_sXSectTol3d ;   
  m_sSampleIvlU               = crOther.m_sSampleIvlU ;   
  m_sSampleIvlV               = crOther.m_sSampleIvlV ; 
  
  m_bAreSamplesSet            = crOther.m_bAreSamplesSet ;   
  m_bIsLocalNeighborhoodSet   = crOther.m_bIsLocalNeighborhoodSet ;   

  m_lSampleCntU               = crOther.m_lSampleCntU ;   
  m_lSampleCntV               = crOther.m_lSampleCntV ;
  m_sSampleGaps               = crOther.m_sSampleGaps ;

  m_lMaxSampleIndex           = crOther.m_lMaxSampleIndex ;
  m_lMinSampleIndex           = crOther.m_lMinSampleIndex ;
  m_bAllNormalGaps            = crOther.m_bAllNormalGaps  ;
  m_sOtherIvlU                = crOther.m_sOtherIvlU      ;
  m_sOtherIvlV                = crOther.m_sOtherIvlV      ;

} // end SmGapFunction::SmGapFunction copy constructor

/*******************************************************************//**
PURPOSE: assignment operator

NOTES:
***********************************************************************/
SmGapFunction & SmGapFunction::operator=
  (const SmGapFunction &crOther)       // in : object to copy
{ 
  if(this == &crOther) return(*this) ;
  
  // copy local members
  m_eSampleType               = crOther.m_eSampleType ;
  m_sXSectTol3d               = crOther.m_sXSectTol3d ;   
  m_sSampleIvlU               = crOther.m_sSampleIvlU ;   
  m_sSampleIvlV               = crOther.m_sSampleIvlV ; 
  
  m_bAreSamplesSet            = crOther.m_bAreSamplesSet ;   
  m_bIsLocalNeighborhoodSet   = crOther.m_bIsLocalNeighborhoodSet ;   

  m_lSampleCntU               = crOther.m_lSampleCntU ;   
  m_lSampleCntV               = crOther.m_lSampleCntV ;
  m_sSampleGaps               = crOther.m_sSampleGaps ;

  m_lMaxSampleIndex           = crOther.m_lMaxSampleIndex ;
  m_lMinSampleIndex           = crOther.m_lMinSampleIndex ;
  m_bAllNormalGaps            = crOther.m_bAllNormalGaps  ;
  m_sOtherIvlU                = crOther.m_sOtherIvlU      ;
  m_sOtherIvlV                = crOther.m_sOtherIvlV      ;

  // all done
  return(*this) ;

} // end SmGapFunction::operator=

/*******************************************************************//**
PURPOSE: equality operator

NOTES:
***********************************************************************/
SmBoolean SmGapFunction::operator==
  (const SmGapFunction &crOther)       // in : object to copy
 const
{ 
  SmBoolean bRtn = TRUE ;
  ULONG ii ;
  
  // check local members
  bRtn &= m_eSampleType               == crOther.m_eSampleType ;
  bRtn &= m_sXSectTol3d               == crOther.m_sXSectTol3d ;   
  bRtn &= m_sSampleIvlU               == crOther.m_sSampleIvlU ;   
  bRtn &= m_sSampleIvlV               == crOther.m_sSampleIvlV ; 
                                      
  bRtn &= m_bAreSamplesSet            == crOther.m_bAreSamplesSet ;   
  bRtn &= m_bIsLocalNeighborhoodSet   == crOther.m_bIsLocalNeighborhoodSet ;   
                                       
  bRtn &= m_lSampleCntU               == crOther.m_lSampleCntU ;   
  bRtn &= m_lSampleCntV               == crOther.m_lSampleCntV ;

  for(ii=0;bRtn && ii<m_sSampleGaps.GetSize();ii++)
    {
      bRtn &= m_sSampleGaps[ii]       == crOther.m_sSampleGaps[ii] ;
    }

  bRtn &= m_lMaxSampleIndex           == crOther.m_lMaxSampleIndex ;
  bRtn &= m_lMinSampleIndex           == crOther.m_lMinSampleIndex ;
  bRtn &= m_bAllNormalGaps            == crOther.m_bAllNormalGaps  ;
  bRtn &= m_sOtherIvlU                == crOther.m_sOtherIvlU      ;
  bRtn &= m_sOtherIvlV                == crOther.m_sOtherIvlV      ;

  // all done
  return(bRtn) ;

} // end SmGapFunction::operator==

/*******************************************************************//**
PURPOSE: Get targeted SmGapSample object watching out for Curves and Surface
 indexing

NOTES:
***********************************************************************/
SmGapSample & SmGapFunction::GetGap
 (ULONG ii,   // in : 1st index used for Curves and Surfaces                                            
  ULONG jj)   // in : 2nd index used only for Surfaces, default:[0]
{
  if(   m_eSampleType == SM_GS_CRV_PT  
     || m_eSampleType == SM_GS_CRV_CRV 
     || m_eSampleType == SM_GS_CRV_SRF) { return m_sSampleGaps[ii] ; }
  else                                  { return m_sSampleGaps[jj * m_lSampleCntU + ii] ; }

} // end SmGapFunction::GetGap
  
/*******************************************************************//**
PURPOSE: Set Sample Counts

NOTES:
***********************************************************************/
void SmGapFunction::SetSampleCnt
 (ULONG lSampleCntU,        // in : number of samples in the first dimension                                             
  ULONG lSampleCntV)        // in : number of samples in 2nd dimension (will be set to 1 for ThisCurve type GapFunctions)
{ 
  // when CntU needs changing
  if(m_lSampleCntU != lSampleCntU) 
    { 
      m_lSampleCntU    = lSampleCntU ;
      m_bAreSamplesSet = FALSE ;
    }

  // When GapFunction has a Curve ThisGeometry - set CntV = 1
  if(   m_eSampleType == SM_GS_CRV_PT  
     || m_eSampleType == SM_GS_CRV_CRV 
     || m_eSampleType == SM_GS_CRV_SRF)
    { 
      m_lSampleCntV = 1 ; 
    }

  // else when CntV needs changing
  else if(m_lSampleCntV != lSampleCntV) 
    { 
      m_lSampleCntV    = lSampleCntV ;
      m_bAreSamplesSet = FALSE ;
    }

} // end SmGapFunction::SetSampleCnt

/*******************************************************************//**
PURPOSE: Helper function for BuildSampleSet.  Add NewElement to ordered
 ULONG Array when it's not already in the array and return its Array index.

NOTES --  Index = Index of NewElement in Array when value is in array or 
                  ordered insertion index of NewElement when value is not in array.
          works in Param Space with SmTol::GetEffZeroParam()
***********************************************************************/
ULONG sm_AddUniqueGetOrderedIndex
  (SmTArray<double> &rArray,       // in : Array to edit
   double            sNewElement)  // in : Element to add in order to rArray when missing
{
  // low work - no elements
  if(rArray.GetSize() == 0)
    {
      rArray.Add(sNewElement) ;
      return(0) ;
    } // end rArray has 0 entries case

  // low Work - element equal to 1st entry
  if(SM_ARE_SAME_TO_TOL(sNewElement, rArray[0], SmTol::GetEffZeroParam())) 
    { return(0) ; }

  // low Work - element less than 1st entry
  if(sNewElement < rArray[0])
    {
      rArray.InsertAt(0, sNewElement) ;
      return(0) ;
    }
      
  // low Work - element greater than last entry (needed for 1 entry cases)
  if(sNewElement > rArray.GetLast())
    { 
      rArray.Add(sNewElement) ;
      return(rArray.GetSize()-1) ; 
    }

  // low Work - element equal to last entry
  if(SM_ARE_SAME_TO_TOL(sNewElement, rArray.GetLast(), SmTol::GetEffZeroParam())) 
    { return(rArray.GetSize()-1) ; }


  // arrive here after all 0 and 1 entry, below Min, and above Max cases have been handled - 
  // expect 2 or more entries in rArray with NewElement located somewhere inside the Array element range
      
  // case - rArray has 2 or more elements and NewElement is somewhere inside the rArray element range
    {
      ULONG ii ;

      // for every interval rArray[ii-1] to rArray[ii]
      for(ii=1;ii<rArray.GetSize();ii++)
        {
          // when new element already exists - return its index
          if(SM_ARE_SAME_TO_TOL(sNewElement, rArray[ii], SmTol::GetEffZeroParam()))
            { return(ii) ; }

          // else if new element < rArray[ii] - insert value and return index value
          if(sNewElement < rArray[ii])
            {
              rArray.InsertAt(ii,sNewElement) ;
              return(ii) ;
            }
        } // end iter every Array span
    } // end rArray has 2 or more case

  // should never arrive here
  ERR_MSG(_T("sm_AddUniqueGetOrderedIndex bug: Unexpected case found - review and extend this function.")) ;
  return(0) ;

} // end sm_AddUniqueGetOrderedIndex

/*******************************************************************//**
PURPOSE: Evaluate the saved samples used for graphics when needed

NOTES: current sampling = even spaced Param steps through each curve interval. 
       Step Count = m_lSampleCntV x m_lSampleCntU
       ordered: curves   : U [0 1 2 ... n-1]
              : surfaces : UV[00 01 02 ... 0n-1 10 11 12 .. 1n-1 .. m0 m1 m2 .. m-1n-1]

       Special case BSplineCurves to include mid-Knot values since
       many BSplineCurves (e.g. XSectCurves) are built to maximize
       their gap sizes at the mid-knot points.

TODO: Gap Functions should be represented by functions and not by
      sampling.  Max and Min gaps should be found by calling the
      Global solver and using Newton-Raphson to find the actual
      Mins and Maxes.  That code is not written yet and would
      probably be much more expensive to run since we expect the
      Gap Function size to commonly have one maximum within each and every
      BSpline knot interval.
***********************************************************************/
SmStatus SmGapFunction::BuildSampleSet
 (SmBoolean bStopAfterBigGap) // in : FALSE = compute all gaps
                              //      TRUE  = Quit when the 1st gap bigger than tol is found 
                              //      default:[FALSE]
{
  // locals
  SmGapFunction *pThisGF = this ;

  // locals for debugging
#ifdef SM_DEBUG_CODE
static ULONG     lDebugCount = 0 ; lDebugCount++ ;
SmBoolean bDebugMe = FALSE ;
SmBoolean bDebugRecursing = FALSE ;       // FALSE(always) = State bit for calling PrettyPrint methods during debug without infinit recursion
SmBoolean bDebugMeWriteHeader = FALSE ;   // FALSE = no Debugging. TRUE = DumpLines as: "SmGapFunction::BuildSampleSet() CallCount:[xxx] 
                                                 //                                               Type:[CRV_SRF-0x%xx/0x%xx] Smps:[TODO]
                                                 //                                               New_MaxGap:[%16.16lf] Max_ThisParamX:[%16.16lf] Max_OtherParamX:[%16.16lf]"
SmBoolean bDebugMeWriteDetails = FALSE ;  // FALSE = no Debugging. TRUE = DumpLines as: "Added index:[xxx[
TCHAR sBuff[SM_TBLOCK_SIZE];

//SmBoolean bDebugMeCompare = FALSE ;   // TRUE = Compare & rtn Old values, FALSE=Run New only, UNSURE=Run Old only
//SmGapFunction * pGapFunctionCompare = NULL ; 

  // this is a difficult method to debug because of the lazy evaluation design - every pretty print member call causes recursion - so stop infinite recursions here.
  if( bDebugRecursing == TRUE)
    { return SM_SUCCESS ; }
  SmTemporaryChangeValue<SmBoolean> saveRecursing(bDebugRecursing, TRUE) ;

// chance to break on a particular iteration
static constexpr ULONG lTgtCount = 0 ; 
  if(bDebugMe || lTgtCount == lDebugCount)
    {
      this->Dump() ;
    }

  // chance to Print header
 
  if(bDebugMeWriteHeader)
    {
      smos_sprintf(sBuff, _T("\nSmGapFunction::BuildSampleSet() CallCount:[%3ld] Type:[%s-0x%p/0x%p] Smps:[%s]"), 
                 lDebugCount, 
                   (pThisGF->GetSampleType() == SM_GS_CRV_PT ) ? _T("CRV_PT ")
                 : (pThisGF->GetSampleType() == SM_GS_CRV_CRV) ? _T("CRV_CRV")
                 : (pThisGF->GetSampleType() == SM_GS_CRV_SRF) ? _T("CRV_SRF")
                 : (pThisGF->GetSampleType() == SM_GS_SRF_PT ) ? _T("SRF_PT ")
                 : (pThisGF->GetSampleType() == SM_GS_SRF_CRV) ? _T("SRF_CRV")
                 : (pThisGF->GetSampleType() == SM_GS_SRF_SRF) ? _T("SRF_SRF")
                 :                                      _T("UNKNOWN"),
                 pThisGF->GetThisObject(), 
                 pThisGF->GetOtherObject(),
                   pThisGF->m_bAreSamplesSet == TRUE   ? _T("DONE")    
                 : pThisGF->m_bAreSamplesSet == UNSURE ? _T("DONE to 1st out-of-tol gap") 
                 :                                       _T("TODO")) ;  
      smos_WriteBuffer(sBuff) ;
    } // end debug write header check

  if(bDebugMe)
    {
      Dump() ;

      const SmCurve   *pFromCurve   = GetThisCurve() ;
      const SmSurface *pFromSurface = GetThisSurface() ;
      const SmPoint3d *pToPoint     = GetOtherPoint() ;
      const SmCurve   *pToCurve     = GetOtherCurve() ;
      const SmSurface *pToSurface   = GetOtherSurface() ;

      SmEdge *pFromEdge = pFromCurve   ? (SmEdge *)pFromCurve->GetEdge()   : NULL ;
      SmFace *pFromFace = pFromSurface ? (SmFace *)pFromSurface->GetFace() : NULL ;
      SmEdge *pToEdge   = pToCurve     ? (SmEdge *)pToCurve->GetEdge() : NULL ;
      SmFace *pToFace   = pToSurface   ? (SmFace *)pToSurface->GetFace() : NULL ;
      SmBrep *pFromBrep = pFromEdge ? pFromEdge->GetBrep() : pFromFace ? pFromFace->GetBrep() : NULL ;
      SmBrep *pToBrep   = pToEdge ? pToEdge->GetBrep() : pToFace ? pToFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pFromBrep) pFromBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pToBrep)   pToBrep->Draw(TRUE) ;   sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; if(pFromCurve)   pFromCurve->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pFromSurface) pFromSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pToPoint)   pToPoint->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pToCurve)   pToCurve->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1) ; if(pToSurface) pToSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pFromEdge) pFromEdge->Draw() ;                 sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pFromFace) pFromFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pToEdge) pToEdge->Draw() ;                 sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pToFace) pToFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,8, 1,0,0) ; if(pThisGF) pThisGF->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE    

  // no work - samples already set or already have a gap bigger than tol
  if(   pThisGF->m_bAreSamplesSet == TRUE           // m_bAreSamplesSet: TRUE  = m_sSampleGaps already computed
     || pThisGF->m_bAreSamplesSet == UNSURE)        //                   FALSE = m_sSampleGaps not computed
     //  && bStopAfterBigGap == TRUE                //                   UNSURE= some computed until an out-of-tol gap was found
     //  && pThisGF->GetMaxGapSample(TRUE).GetLengthSquared() > pThisGF->m_sXSectTol3d*pThisGF->m_sXSectTol3d)) 
    { 
      return SM_SUCCESS ; 
    }

  // iterators
  ULONG ii, i1, jj, jCnt ;

  // scope to build sampling point normalized param values
  //  Adjust pThisGF->m_lSampleCntU and pThisGF->m_lSampleCntV For BSplineGeometry if needed
  //  Set    sSampleNormalizedVals with U and V sample values ordered:[u0 u1 ... un v0 v1 ... vm]
  // note: sample range = m_sSampleIvlU and m_sSampleIvlU
  SmTArray<double> sSampleNormalizedVals ; 
    {
      // locals
      const SmCurve   * pThisCurve   = pThisGF->GetThisCurve() ; 
      const SmSurface * pThisSurface = pThisGF->GetThisSurface() ;
      SmBoolean         bIsBSpline   =    (pThisCurve && pThisCurve->IsKindOf(SmBSplineCurve_TYPE))
                                       || (pThisSurface && pThisSurface->IsKindOf(SmBSplineSurface_TYPE)) ;
      // When ThisGeometry is BSpline geometry
      //   adjust sample counts and sample UV values to make sure 
      //   the ends and center of everyknot span are sampled
      if(bIsBSpline)
        {
          // locals
          SmTArray<double>  sUniqueKnotsU, sUniqueKnotsV ; 
          ULONG             lStartIndxU = 0, lEndIndxU = 0 ;
          ULONG             lStartIndxV = 0, lEndIndxV = 0 ;

          // Get Spline U (and V) Knots 
          if(pThisCurve) { ((SmBSplineCurve *) pThisCurve)->GetKnots(sUniqueKnotsU, NULL, NULL) ; 
                           sUniqueKnotsV.Add(0.0) ; // have one entry to simplify upcoming loop structurs - fewer conditionals
                         }
          else           { SM_ASSERT_MSG(pThisSurface != NULL, _T("BuildSampleSet - found a case where pThisSurface is not set properly - review and extend method")) ;
                           ((SmBSplineSurface *)pThisSurface)->GetKnots(SM_SP_U, sUniqueKnotsU, NULL, NULL) ;  
                           ((SmBSplineSurface *)pThisSurface)->GetKnots(SM_SP_V, sUniqueKnotsV, NULL, NULL) ;  
                         }

          // Get UniqueKnotsU index values for m_sSampleIvlU EndPoints
          lStartIndxU = sm_AddUniqueGetOrderedIndex(sUniqueKnotsU, pThisGF->m_sSampleIvlU.GetMin()) ;
          lEndIndxU   = sm_AddUniqueGetOrderedIndex(sUniqueKnotsU, pThisGF->m_sSampleIvlU.GetMax()) ;
          if(pThisSurface)
            {
              lStartIndxV = sm_AddUniqueGetOrderedIndex(sUniqueKnotsV, pThisGF->m_sSampleIvlV.GetMin()) ;
              lEndIndxV   = sm_AddUniqueGetOrderedIndex(sUniqueKnotsV, pThisGF->m_sSampleIvlV.GetMax()) ;
            }

          // pick Number of samples per interval so that TotalSampleCount >= m_lSampleCntU (and m_lSampleCntV)
          ULONG lIvlCntU = lEndIndxU - lStartIndxU ;
          ULONG lIvlCntV = lEndIndxV - lStartIndxV ;

          // There should always be 1 or more intervals - only exception should be if m_sSampleIvlU or m_sSampleIvlV length == 0
          //  which is a case that isn't expected to be supported.
          AERN_MSG((lEndIndxU > lStartIndxU) && (pThisSurface == NULL || lEndIndxV > lStartIndxV), SM_ERR_ASSERT_FAILURE,
                        _T("BuildSampleSet for Splines found an unhandled degenerate case - review case and extend method")) ;

          // when working with a ThisCurve or a ThisSurface, set U dir Interval sample counts
          ULONG lSmpCntPerIvlU = 1 ; 
            {
              // integer number of samples per interval (ignoring fist sample at start of first interval
              lSmpCntPerIvlU = (pThisGF->m_lSampleCntU - 1) / lIvlCntU ; 
          
              // when lSmpCntPerIvl was rounded down (Fewer samples than requested)
              if((lSmpCntPerIvlU * lIvlCntU) < (pThisGF->m_lSampleCntU - 1))
                {
                  // increate lSmpCntPerIvl by 1 to guarantee (lSmpCntPerIvlU * lIvlCntU) >= (m_lSampleCntU - 1) 
                  lSmpCntPerIvlU++ ;
                }

              // make sure lSmpCntPerIvl is greater than 1 - guarantees interior samples of each interval
              if(lSmpCntPerIvlU < 2)
                { lSmpCntPerIvlU++ ; }
              
              // make sure lSmpCntPerIvlU is even (places one sample point at interval center and one at the end
              if(lSmpCntPerIvlU % 2 != 0)
                { lSmpCntPerIvlU++ ; }

              // save the actual sample count
              pThisGF->m_lSampleCntU = lSmpCntPerIvlU * lIvlCntU + 1 ;
            } // end working with a ThisCurve or a ThisSurface to set U dir interval sample count scope

          // when working on a Surface, set V dir Interval sample counts
          ULONG lSmpCntPerIvlV = 1 ;
          if(pThisSurface != NULL)
            {
              // integer number of samples per interval (ignoring fist sample at start of first interval
              lSmpCntPerIvlV = (pThisGF->m_lSampleCntV - 1) / lIvlCntV ; 
          
              // when lSmpCntPerIvl was rounded down (Fewer samples than requested)
              if((lSmpCntPerIvlV * lIvlCntV) < (pThisGF->m_lSampleCntV - 1))
                {
                  // increate lSmpCntPerIvl by 1 to guarantee (lSmpCntPerIvlV * lIvlCntV) >= (m_lSampleCntV - 1) 
                  lSmpCntPerIvlV++ ;
                }

              // make sure lSmpCntPerIvl is greater than 1 - guarantees interior samples of each interval
              if(lSmpCntPerIvlV < 2)
                { lSmpCntPerIvlV++ ; }
              
              // make sure lSmpCntPerIvlV is even (places one sample point at interval center and one at the end
              if(lSmpCntPerIvlV % 2 != 0)
                { lSmpCntPerIvlV++ ; }

              // save the actual sample count
              pThisGF->m_lSampleCntV = lSmpCntPerIvlV * lIvlCntV + 1 ;

            } // end if working on a surface to set V dir interval sample count check

          // arrive here after sUniqueKnotsU,  sUniqueKnotsV,
          //                   lStartIndxU,    lStartIndxV,  
          //                   lEndIndxU       lEndIndxV    
          //                   lSmpCntPerIvlU  lSmpCntPerIvlV
          // have been set both U and V dirs
            
          // size Array to hold Sample NormalizedParam values for BSplines: [u0 u1 .. un v0 v1 .. vm]
          sSampleNormalizedVals.SetSize(pThisGF->m_lSampleCntU + pThisGF->m_lSampleCntV) ;

          // ParamLength of U and V domains
          double dParamLengthU = sUniqueKnotsU.GetLast() - sUniqueKnotsU[0] ;  // BSpline Total span - used to normalize values
          double dParamLengthV = 1.0 ;

          // boundary normalized values - set to avoid numerical tolerances
          sSampleNormalizedVals.SetAt(0, (sUniqueKnotsU[lStartIndxU]-sUniqueKnotsU[0])/dParamLengthU) ;  // 1st U - normalized value
          sSampleNormalizedVals.SetAt(pThisGF->m_lSampleCntU-1, (sUniqueKnotsU[lEndIndxU]-sUniqueKnotsU[0])/dParamLengthU) ;  // last U - normalized value
          sSampleNormalizedVals.SetAt(pThisGF->m_lSampleCntU, 0.0) ;  // 1st V  - normalized value
          if(pThisSurface != NULL) // when working with a surface
            { 
              dParamLengthV = sUniqueKnotsV.GetLast() - sUniqueKnotsV[0] ;
              sSampleNormalizedVals.SetAt(pThisGF->m_lSampleCntU, (sUniqueKnotsV[lStartIndxV]-sUniqueKnotsV[0])/dParamLengthV) ;                         // 1st U - normalized value
              sSampleNormalizedVals.SetAt(pThisGF->m_lSampleCntU+pThisGF->m_lSampleCntV-1, (sUniqueKnotsV[lEndIndxV]-sUniqueKnotsV[0])/dParamLengthV) ;  // last U - normalized value
            } 

         // for every U BSpline Span
         for(ii=0,i1=1,jCnt=1;ii<lIvlCntU;ii++,i1++)
           {
             double dStartP = sUniqueKnotsU[ii] ;
             double dEndP   = sUniqueKnotsU[i1] ;
             double jInc    = (dEndP - dStartP) / (double)(lSmpCntPerIvlU) ;
             double jVal    = dStartP + jInc ;

             // for every internal Span U value - BSplineSpan even stride sampling
             for(jj=0;jj<lSmpCntPerIvlU;jj++,jCnt++,jVal+=jInc)
               {
                 // load SampleNormalizedVals with Normalized values
                 sSampleNormalizedVals.SetAt(jCnt, (jVal-sUniqueKnotsU[0])/dParamLengthU) ;
               } // end iter BSplineSpan even stride sampling
           } // end iter every BSplineSpanU

         // when working with a surface
         if(pThisSurface)
           {
             // skip V Start Point value
             jCnt++ ;  

             // for every V BSpline Span
             for(ii=0,i1=1;ii<lIvlCntV;ii++,i1++)
               {
                 double dStartP = sUniqueKnotsV[ii] ;
                 double dEndP   = sUniqueKnotsV[i1] ;
                 double jInc    = (dEndP - dStartP) / (double)(lSmpCntPerIvlV) ;
                 double jVal    = dStartP + jInc ;

                 // for every internal Span U value - BSplineSpan even stride sampling
                 for(jj=0;jj<lSmpCntPerIvlV;jj++,jCnt++,jVal+=jInc)
                   {
                     // load SampleNormalizedVals with Normalized values
                     sSampleNormalizedVals.SetAt(jCnt, (jVal-sUniqueKnotsV[0])/dParamLengthV) ;
                   } // end iter BSplineSpan even stride sampling
               } // end iter every BSplineSpanV
           } // end working with Surface check
        } // end ThisGeometry is BSpline geometry branch

      else // This Geometry is not a BSpline - even stride sampling
        {
          // size and fill Sample values
          sSampleNormalizedVals.SetSize(pThisGF->m_lSampleCntU + pThisGF->m_lSampleCntV) ;

          // boundary values - set to avoid numerical tolerances
          sSampleNormalizedVals.SetAt(0, 0.0) ;                // 1st U
          sSampleNormalizedVals.SetAt(pThisGF->m_lSampleCntU-1, 1.0) ;  // last U
          sSampleNormalizedVals.SetAt(pThisGF->m_lSampleCntU, 0.0) ;    // 1st V
          if(pThisGF->m_lSampleCntV > 1)
            { sSampleNormalizedVals.SetAt(pThisGF->m_lSampleCntU+pThisGF->m_lSampleCntV-1, 1.0) ; } // last V

          // for every internal U value - NaturalInterval even stride sampling
          for(ii=2,jj=1;ii<pThisGF->m_lSampleCntU;ii++,jj++)
            {
              sSampleNormalizedVals.SetAt(jj, (double)jj/(double)(pThisGF->m_lSampleCntU-1) ) ;
            }

          // for every internal V value - NaturalInterval even stride sampling
          for(ii=2,jj=pThisGF->m_lSampleCntU+1;ii<pThisGF->m_lSampleCntV;ii++,jj++)
            {
              sSampleNormalizedVals.SetAt(jj, (double)jj/(double)(pThisGF->m_lSampleCntV-1) ) ;
            }
        } // end ThisGeometry is not BSpline Geometry branch
    } // end Scope to build sampling point normalized param values

  // arrive here after m_lSampleCntU 
  //                   m_lSampleCntV and
  //             array sSampleNormalizedVals (as [u0, u1,.. uN, v0, v1,.. vN]) have been set 

  // Set memory in pThisGF->m_sSampleGaps (When working on curves m_pSampleCntV has been set to 1)
  pThisGF->m_sSampleGaps.SetSize(pThisGF->m_lSampleCntU * pThisGF->m_lSampleCntV) ; 

  // locals
  SmBoolean bUseV = (   pThisGF->m_eSampleType == SM_GS_CRV_SRF
                     || pThisGF->m_eSampleType == SM_GS_SRF_SRF ) ;
  double      *pLastU = NULL ;
  double      *pLastV = NULL ;
  SmGapSample *pLastSample = NULL ;

  // iteration statistics
  ULONG  lSmpCnt   = 0 ;
  double dMinGap   =  SM_BIG_DOUBLE ;
  double dMaxGap   = -SM_BIG_DOUBLE ;
  pThisGF->m_bAllNormalGaps = TRUE ; 
  pThisGF->m_sOtherIvlU.Init() ; 
  pThisGF->m_sOtherIvlV.Init() ;

  // for every sample point row
  for(jj=0;
      jj<pThisGF->m_lSampleCntV;
      jj++,
      pLastSample=&(pThisGF->m_sSampleGaps[(jj-1) * pThisGF->m_lSampleCntU]) )  // do 2nd dim as outer loop because all curves will have SampleCntV = 1
    {
      double dNormalParamV = sSampleNormalizedVals.GetAt(pThisGF->m_lSampleCntU + jj) ;
      double dThisV        = pThisGF->m_sSampleIvlV.Evaluate(dNormalParamV) ;
      ULONG  jCount        = jj * pThisGF->m_lSampleCntU ;

      // for every sample point col entry
      for(ii=0;
          ii<pThisGF->m_lSampleCntU;
          ii++,
          pLastSample=&(pThisGF->m_sSampleGaps[jCount + ii - 1]), lSmpCnt++)
        {
          double dNormalParamU = sSampleNormalizedVals.GetAt(ii) ;
          double dThisU        = pThisGF->m_sSampleIvlU.Evaluate(dNormalParamU) ;

          // evaluate and store this gap in row major order
          if(pLastSample)
            { pLastU = &(pLastSample->GetOtherParam().x) ;
              if(bUseV)
                { pLastV = &(pLastSample->GetOtherParam().y) ; }
            }

#ifdef SM_DEBUG_CODE
          if(bDebugMeWriteDetails)
            { smos_sprintf(sBuff, _T("\n Added index:[%3ld]"), jCount + ii) ; smos_WriteBuffer(sBuff) ; }

          if(jCount +11 > pThisGF->m_lSampleCntU * pThisGF->m_lSampleCntV)
            { SM_ASSERT_MSG( jCount +11 < pThisGF->m_lSampleCntU * pThisGF->m_lSampleCntV,
                            _T("SmGapFunction::BuildSampleSet - illegal indexing - needs debug")) ; 
            }
#endif // SM_DEBUG_CODE

          SER(Evaluate(dThisU,                            // in : This Geom 1st param used for Curves and Surfaces
                       dThisV,                            // in : This Geom 2nd param only used for Surfaces
                       pThisGF->m_sSampleGaps[jCount + ii], // out: Eval holder block
                       pLastU,                            // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore
                       pLastV )) ;                        // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore

          // note when a NonNormal Gap was found
          if(pThisGF->m_sSampleGaps[jCount + ii].GetGapDropType() != SM_GD_NORMAL)
            { pThisGF->m_bAllNormalGaps = FALSE ; }

          // accumulate the drop Domain when the other object is not a point (it's a curve or surface)
          if(GetOtherPoint() == NULL)
            {
              SmPoint2d &rDropPt2d = pThisGF->m_sSampleGaps[jCount + ii].GetOtherParam() ;
              pThisGF->m_sOtherIvlU.AddValue(rDropPt2d.x) ;
              if(GetOtherSurface() != NULL) { pThisGF->m_sOtherIvlV.AddValue(rDropPt2d.y) ; }
            }

          // save min and max indices
          double dThisGap = pThisGF->m_sSampleGaps[jCount + ii].GetLength() ;
          if(dThisGap > dMaxGap) { dMaxGap = dThisGap ;
                                   pThisGF->m_lMaxSampleIndex = jCount + ii ;
                                 }
          if(dThisGap < dMinGap) { dMinGap = dThisGap ;
                                   pThisGF->m_lMinSampleIndex = jCount + ii ;
                                 }

          // when asked - quit at the 1st out of tol gap
          if(   bStopAfterBigGap
             && dThisGap > pThisGF->m_sXSectTol3d)
            { 
              pThisGF->m_bAreSamplesSet = UNSURE ; 
              return(SM_SUCCESS) ;
            }

        } // end iter ii, all 1st dim Samples - evaluating GapSamples
    } // end iter jj, all 2nd dim Samples

  // remember the work
  pThisGF->m_bAreSamplesSet = TRUE ;

#ifdef SM_DEBUG_CODE
  // check indexing state for sample counts
  if(lSmpCnt != pThisGF->m_lSampleCntU * pThisGF->m_lSampleCntV)
    {
      SM_ASSERT_MSG(lSmpCnt == pThisGF->m_lSampleCntU * pThisGF->m_lSampleCntV, _T("SmGapFunction::BuildSampleSet - illegal indexing - needs debug")) ;  
    }

  // when asked - write new value header
  if(bDebugMeWriteHeader)
    {
      SmGapSample *pMaxGap;
      pThisGF->GetMaxGapSample( pMaxGap );
      if ( pMaxGap )
      {
          // bDebugMeCompare: TRUE = Compare & rtn Old values, FALSE=Run New only, UNSURE=Run Old only
          smos_sprintf( sBuff, _T( "\n                                New_MaxGap:[%16.16lf] Max_ThisParamX:[%16.16lf] Max_OtherParamX:[%16.16lf]" ),
                      pMaxGap->GetLength(),
                      pMaxGap->GetThisParam().x,
                      pMaxGap->GetOtherParam().x );
      }
      else
      {
          smos_sprintf( sBuff, _T("%s"), _T( " \n Failed to calculate gap sample set" ) );
      }
      smos_WriteBuffer(sBuff) ;
    }

  // draw the geometry
  if(bDebugMe)
    {
      const SmCurve   *pFromCurve   = GetThisCurve() ;
      const SmSurface *pFromSurface = GetThisSurface() ;
      const SmPoint3d *pToPoint     = GetOtherPoint() ;
      const SmCurve   *pToCurve     = GetOtherCurve() ;
      const SmSurface *pToSurface   = GetOtherSurface() ;

      SmEdge *pFromEdge = pFromCurve   ? (SmEdge *)pFromCurve->GetEdge()   : NULL ;
      SmFace *pFromFace = pFromSurface ? (SmFace *)pFromSurface->GetFace() : NULL ;
      SmEdge *pToEdge   = pToCurve     ? (SmEdge *)pToCurve->GetEdge() : NULL ;
      SmFace *pToFace   = pToSurface   ? (SmFace *)pToSurface->GetFace() : NULL ;
      SmBrep *pFromBrep = pFromEdge ? pFromEdge->GetBrep() : pFromFace ? pFromFace->GetBrep() : NULL ;
      SmBrep *pToBrep   = pToEdge ? pToEdge->GetBrep() : pToFace ? pToFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0, 0,1) ; if(pFromBrep) pFromBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0, 1,0) ; if(pToBrep)   pToBrep->Draw(TRUE) ;   sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0, 1,1) ; if(pFromCurve)   pFromCurve->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0, 1,1) ; if(pFromSurface) pFromSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1, 0,1) ; if(pToPoint)   pToPoint->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1, 0,1) ; if(pToCurve)   pToCurve->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1, 0,1) ; if(pToSurface) pToSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0, 0,0) ; if(pFromEdge) pFromEdge->Draw() ;                 sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0, 0,0) ; if(pFromFace) pFromFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0, 0,0) ; if(pToEdge) pToEdge->Draw() ;                 sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0, 0,0) ; if(pToFace) pToFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,8, 1, 0,0) ; if(pThisGF) pThisGF->Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE  

// all done
  return(SM_SUCCESS) ;

} // end SmGapFunction::BuildSampleSet

// begin SmGapFunction::BuildSampleSet version for new/old method comparative debugging
//  /*******************************************************************//**
//  PURPOSE: Evaluate the saved samples used for graphics when needed
//  
//  NOTES: current sampling = even spaced Param steps. Step Count = m_lSampleCntV x m_lSampleCntU
//         ordered: curves   : U [0 1 2 ... n-1]
//                : surfaces : UV[00 01 02 ... 0n-1 10 11 12 .. 1n-1 .. m0 m1 m2 .. m-1n-1]
//  
//         Special case BSplineCurves to include mid-Knot values since
//         many BSplineCurves (e.g. XSectCurves) are built to maximize
//         their gap sizes at the mid-knot points.
//  
//  TODO: Gap Functions should be represented by functions and not by
//        sampling.  Max and Min gaps should be found by calling the
//        Global solver and using Newton-Raphson to find the actual
//        Mins and Maxes.  That code is not written yet and would
//        probably be much more expensive to run since we expect the
//        Gap Function size to commonly have one maximum within each and every
//        BSpline knot interval.
//  ***********************************************************************/
//  SmStatus SmGapFunction::BuildSampleSet
//   (SmBoolean bStopAfterBigGap) // in : FALSE = compute all gaps
//                                //      TRUE  = Quit when the 1st gap bigger than tol is found 
//                                //      default:[FALSE]
//  {
//  #ifdef SM_DEBUG_CODE
//  SmBoolean bDebugRecursing = FALSE ;
//  TCHAR sBuff[SM_TBLOCK_SIZE];
//  SmBoolean bDebugMe = FALSE ;
//  SmBoolean bDebugMeWriteHeader = FALSE ;
//  SmBoolean bDebugMeWriteDetails = FALSE ;
//  SmBoolean bDebugMeCompare = FALSE ;   // TRUE = Compare & rtn Old values, FALSE=Run New only, UNSURE=Run Old only
//  SmGapFunction * pGapFunctionCompare = NULL ; 
//  static ULONG lDebugCount = 0 ; 
//  
//    // this is a difficult method to debug because of the lazy evaluation design - every pretty print member call causes recursion - so stop infinite recursions here.
//    if( bDebugRecursing == TRUE)
//      { return SM_SUCCESS ; }
//    SmTemporaryChangeValue<SmBoolean> saveRecursing(bDebugRecursing, TRUE) ;
//  
//    // local for debugging
//    SmGapFunction *pThisGF = this ;
//  
//    // chance to Print header and to run old code only
//      {
//        lDebugCount++ ;
//        if(bDebugMeWriteHeader)
//          {
//            smos_sprintf(sBuff, _T("\nSmGapFunction::BuildSampleSet() CallCount:[%3d] Type:[%s-0x%p/0x%p] "), 
//                       lDebugCount, 
//                         (pThisGF->GetSampleType() == SM_GS_CRV_PT ) ? _T("CRV_PT ")
//                       : (pThisGF->GetSampleType() == SM_GS_CRV_CRV) ? _T("CRV_CRV")
//                       : (pThisGF->GetSampleType() == SM_GS_CRV_SRF) ? _T("CRV_SRF")
//                       : (pThisGF->GetSampleType() == SM_GS_SRF_PT ) ? _T("SRF_PT ")
//                       : (pThisGF->GetSampleType() == SM_GS_SRF_CRV) ? _T("SRF_CRV")
//                       : (pThisGF->GetSampleType() == SM_GS_SRF_SRF) ? _T("SRF_SRF")
//                       :                                      _T("UNKNOWN"),
//                       pThisGF->GetThisObject(), 
//                       pThisGF->GetOtherObject()) ; 
//            smos_WriteBuffer(sBuff) ;
//          }
//        
//        // UNSURE = run and return old values only
//        if(bDebugMeCompare == UNSURE)
//          {
//            // run old code and return
//            SmStatus sRtn = SM_SUCCESS ;
//            sRtn = BuildSampleSet_OldDebug(bStopAfterBigGap) ;
//            smos_sprintf(sBuff, _T("\n                                Old_MaxGap:[%16.16lf] Old_ThisParamX:[%16.16lf] Old_OtherParamX:[%16.16lf]"), 
//                       pThisGF->GetMaxGapSample().GetLength(),
//                       pThisGF->GetMaxGapSample().GetThisParam().x,
//                       pThisGF->GetMaxGapSample().GetOtherParam().x ) ; 
//            smos_WriteBuffer(sBuff) ;
//  
//            if(bDebugMe)
//              {
//                pThisGF->Dump() ;
//              }
//            return( sRtn ) ;
//          }
//      } // end scope to print header and run old code only
//  
//    if(bDebugMe)
//      {
//        Dump() ;
//  
//        const SmCurve   *pFromCurve   = GetThisCurve() ;
//        const SmSurface *pFromSurface = GetThisSurface() ;
//        const SmPoint3d *pToPoint     = GetOtherPoint() ;
//        const SmCurve   *pToCurve     = GetOtherCurve() ;
//        const SmSurface *pToSurface   = GetOtherSurface() ;
//  
//        SmEdge *pFromEdge = pFromCurve   ? (SmEdge *)pFromCurve->GetEdge()   : NULL ;
//        SmFace *pFromFace = pFromSurface ? (SmFace *)pFromSurface->GetFace() : NULL ;
//        SmEdge *pToEdge   = pToCurve     ? (SmEdge *)pToCurve->GetEdge() : NULL ;
//        SmFace *pToFace   = pToSurface   ? (SmFace *)pToSurface->GetFace() : NULL ;
//        SmBrep *pFromBrep = pFromEdge ? pFromEdge->GetBrep() : pFromFace ? pFromFace->GetBrep() : NULL ;
//        SmBrep *pToBrep   = pToEdge ? pToEdge->GetBrep() : pToFace ? pToFace->GetBrep() : NULL ;
//  
//        smgfx_Erase() ;
//        smgfx_SetLook(1,2, 0,0,1) ; if(pFromBrep) pFromBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//        smgfx_SetLook(1,2, 0,1,0) ; if(pToBrep)   pToBrep->Draw(TRUE) ;   sm_GraphicsLoop() ;
//        smgfx_SetLook(3,4, 0,1,1) ; if(pFromCurve)   pFromCurve->Draw() ;     sm_GraphicsLoop() ;
//        smgfx_SetLook(1,2, 0,1,1) ; if(pFromSurface) pFromSurface->DrawUV() ; sm_GraphicsLoop() ;
//        smgfx_SetLook(3,4, 1,0,1) ; if(pToPoint)   pToPoint->Draw() ;     sm_GraphicsLoop() ;
//        smgfx_SetLook(3,4, 1,0,1) ; if(pToCurve)   pToCurve->Draw() ;     sm_GraphicsLoop() ;
//        smgfx_SetLook(1,2, 1,0,1) ; if(pToSurface) pToSurface->DrawUV() ; sm_GraphicsLoop() ;
//        smgfx_SetLook(5,6, 0,0,0) ; if(pFromEdge) pFromEdge->Draw() ;                 sm_GraphicsLoop() ;
//        smgfx_SetLook(5,6, 0,0,0) ; if(pFromFace) pFromFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
//        smgfx_SetLook(5,6, 0,0,0) ; if(pToEdge) pToEdge->Draw() ;                 sm_GraphicsLoop() ;
//        smgfx_SetLook(5,6, 0,0,0) ; if(pToFace) pToFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
//        smgfx_SetLook(1,8, 1,0,0) ; if(pThisGF) pThisGF->Draw(TRUE) ; sm_GraphicsLoop() ;
//        sm_GraphicsLoop() ;
//      }
//  #endif // SM_DEBUG_CODE    
//  
//    // no work - samples already set or already have a gap bigger than tol
//    if(   pThisGF->m_bAreSamplesSet == TRUE
//       || (   pThisGF->m_bAreSamplesSet == UNSURE
//           && bStopAfterBigGap == TRUE
//           && pThisGF->GetMaxGapSample(TRUE).GetLengthSquared() > pThisGF->m_sXSectTol3d*pThisGF->m_sXSectTol3d)) 
//      { 
//        if(bDebugMeCompare == TRUE || bDebugMeCompare == UNSURE)
//          {
//            smos_WriteBuffer(_T(" Allready Set")) ;
//          }
//  
//        return SM_SUCCESS ; 
//      }
//  
//    // iterators
//    ULONG ii, i1, jj, jCnt ;
//  
//    // scope to build sampling point normalized param values
//    //  Adjust pThisGF->m_lSampleCntU and pThisGF->m_lSampleCntV For BSplineGeometry if needed
//    //  Set    sSampleNormalizedVals with U and V sample values ordered:[u0 u1 ... un v0 v1 ... vm]
//    // note: sample range = m_sSampleIvlU and m_sSampleIvlU
//    SmTArray<double> sSampleNormalizedVals ; 
//      {
//        // locals
//        const SmCurve   * pThisCurve   = pThisGF->GetThisCurve() ; 
//        const SmSurface * pThisSurface = pThisGF->GetThisSurface() ;
//        SmBoolean         bIsBSpline   =    (pThisCurve && pThisCurve->IsKindOf(SmBSplineCurve_TYPE))
//                                         || (pThisSurface && pThisSurface->IsKindOf(SmBSplineSurface_TYPE)) ;
//        // When ThisGeometry is BSpline geometry
//        //   adjust sample counts and sample UV values to make sure 
//        //   the ends and center of everyknot span are sampled
//        if(bIsBSpline)
//          {
//            // locals
//            SmTArray<double>  sUniqueKnotsU, sUniqueKnotsV ; 
//            ULONG             lStartIndxU = 0, lEndIndxU = 0 ;
//            ULONG             lStartIndxV = 0, lEndIndxV = 0 ;
//  
//            // Get Spline U (and V) Knots 
//            if(pThisCurve) { ((SmBSplineCurve *) pThisCurve)->GetKnots(sUniqueKnotsU, NULL, NULL) ; 
//                             sUniqueKnotsV.Add(0.0) ; // have one entry to simplify upcoming loop structurs - fewer conditionals
//                           }
//            else           { SM_ASSERT_MSG(pThisSurface != NULL, _T("BuildSampleSet - found a case where pThisSurface is not set properly - review and extend method")) ;
//                             ((SmBSplineSurface *)pThisSurface)->GetKnots(SM_SP_U, sUniqueKnotsU, NULL, NULL) ;  
//                             ((SmBSplineSurface *)pThisSurface)->GetKnots(SM_SP_V, sUniqueKnotsV, NULL, NULL) ;  
//                           }
//  
//            // Get UniqueKnotsU index values for m_sSampleIvlU EndPoints
//            lStartIndxU = sm_AddUniqueGetOrderedIndex(sUniqueKnotsU, pThisGF->m_sSampleIvlU.GetMin()) ;
//            lEndIndxU   = sm_AddUniqueGetOrderedIndex(sUniqueKnotsU, pThisGF->m_sSampleIvlU.GetMax()) ;
//            if(pThisSurface)
//              {
//                lStartIndxV = sm_AddUniqueGetOrderedIndex(sUniqueKnotsV, pThisGF->m_sSampleIvlV.GetMin()) ;
//                lEndIndxV   = sm_AddUniqueGetOrderedIndex(sUniqueKnotsV, pThisGF->m_sSampleIvlV.GetMax()) ;
//              }
//  
//            // pick Number of samples per interval so that TotalSampleCount >= m_lSampleCntU (and m_lSampleCntV)
//            ULONG lIvlCntU = lEndIndxU - lStartIndxU ;
//            ULONG lIvlCntV = lEndIndxV - lStartIndxV ;
//  
//            // There should always be 1 or more intervals - only exception should be if m_sSampleIvlU or m_sSampleIvlV length == 0
//            //  which is a case that isn't expected to be supported.
//            SM_ASSERT_MSG((lIvlCntU > 0) && (pThisSurface == NULL || lIvlCntV > 0),
//                          _T("BuildSampleSet for Splines found an unhandled degenerate case - review case and extend method")) ;
//  
//            // when working with a ThisCurve or a ThisSurface, set U dir Interval sample counts
//            ULONG lSmpCntPerIvlU = 1 ; 
//              {
//                // integer number of samples per interval (ignoring fist sample at start of first interval
//                lSmpCntPerIvlU = (pThisGF->m_lSampleCntU - 1) / lIvlCntU ; 
//            
//                // when lSmpCntPerIvl was rounded down (Fewer samples than requested)
//                if((lSmpCntPerIvlU * lIvlCntU) < (pThisGF->m_lSampleCntU - 1))
//                  {
//                    // increate lSmpCntPerIvl by 1 to guarantee (lSmpCntPerIvlU * lIvlCntU) >= (m_lSampleCntU - 1) 
//                    lSmpCntPerIvlU++ ;
//                  }
//  
//                // make sure lSmpCntPerIvl is greater than 1 - guarantees interior samples of each interval
//                if(lSmpCntPerIvlU < 2)
//                  { lSmpCntPerIvlU++ ; }
//                
//                // make sure lSmpCntPerIvlU is even (places one sample point at interval center and one at the end
//                if(lSmpCntPerIvlU % 2 != 0)
//                  { lSmpCntPerIvlU++ ; }
//  
//                // save the actual sample count
//                pThisGF->m_lSampleCntU = lSmpCntPerIvlU * lIvlCntU + 1 ;
//              } // end working with a ThisCurve or a ThisSurface to set U dir interval sample count scope
//  
//            // when working on a Surface, set V dir Interval sample counts
//            ULONG lSmpCntPerIvlV = 1 ;
//            if(pThisSurface != NULL)
//              {
//                // integer number of samples per interval (ignoring fist sample at start of first interval
//                lSmpCntPerIvlV = (pThisGF->m_lSampleCntV - 1) / lIvlCntV ; 
//            
//                // when lSmpCntPerIvl was rounded down (Fewer samples than requested)
//                if((lSmpCntPerIvlV * lIvlCntV) < (pThisGF->m_lSampleCntV - 1))
//                  {
//                    // increate lSmpCntPerIvl by 1 to guarantee (lSmpCntPerIvlV * lIvlCntV) >= (m_lSampleCntV - 1) 
//                    lSmpCntPerIvlV++ ;
//                  }
//  
//                // make sure lSmpCntPerIvl is greater than 1 - guarantees interior samples of each interval
//                if(lSmpCntPerIvlV < 2)
//                  { lSmpCntPerIvlV++ ; }
//                
//                // make sure lSmpCntPerIvlV is even (places one sample point at interval center and one at the end
//                if(lSmpCntPerIvlV % 2 != 0)
//                  { lSmpCntPerIvlV++ ; }
//  
//                // save the actual sample count
//                pThisGF->m_lSampleCntV = lSmpCntPerIvlV * lIvlCntV + 1 ;
//  
//              } // end if working on a surface to set V dir interval sample count check
//  
//            // arrive here after sUniqueKnotsU,  sUniqueKnotsV,
//            //                   lStartIndxU,    lStartIndxV,  
//            //                   lEndIndxU       lEndIndxV    
//            //                   lSmpCntPerIvlU  lSmpCntPerIvlV
//            // have been set both U and V dirs
//              
//            // size Array to hold Sample NormalizedParam values for BSplines: [u0 u1 .. un v0 v1 .. vm]
//            sSampleNormalizedVals.SetSize(pThisGF->m_lSampleCntU + pThisGF->m_lSampleCntV) ;
//  
//            // ParamLength of U and V domains
//            double dParamLengthU = sUniqueKnotsU.GetLast() - sUniqueKnotsU[0] ;  // BSpline Total span - used to normalize values
//            double dParamLengthV = 1.0 ;
//  
//            // boundary normalized values - set to avoid numerical tolerances
//            sSampleNormalizedVals.SetAt(0, (sUniqueKnotsU[lStartIndxU]-sUniqueKnotsU[0])/dParamLengthU) ;  // 1st U - normalized value
//            sSampleNormalizedVals.SetAt(pThisGF->m_lSampleCntU-1, (sUniqueKnotsU[lEndIndxU]-sUniqueKnotsU[0])/dParamLengthU) ;  // last U - normalized value
//            sSampleNormalizedVals.SetAt(pThisGF->m_lSampleCntU, 0.0) ;  // 1st V  - normalized value
//            if(pThisSurface != NULL) // when working with a surface
//              { 
//                dParamLengthV = sUniqueKnotsV.GetLast() - sUniqueKnotsV[0] ;
//                sSampleNormalizedVals.SetAt(pThisGF->m_lSampleCntU, (sUniqueKnotsV[lStartIndxV]-sUniqueKnotsV[0])/dParamLengthV) ;                         // 1st U - normalized value
//                sSampleNormalizedVals.SetAt(pThisGF->m_lSampleCntU+pThisGF->m_lSampleCntV-1, (sUniqueKnotsV[lEndIndxV]-sUniqueKnotsV[0])/dParamLengthV) ;  // last U - normalized value
//              } 
//  
//           // for every U BSpline Span
//           for(ii=0,i1=1,jCnt=1;ii<lIvlCntU;ii++,i1++)
//             {
//               double dStartP = sUniqueKnotsU[ii] ;
//               double dEndP   = sUniqueKnotsU[i1] ;
//               double jInc    = (dEndP - dStartP) / (double)(lSmpCntPerIvlU) ;
//               double jVal    = dStartP + jInc ;
//  
//               // for every internal Span U value - BSplineSpan even stride sampling
//               for(jj=0;jj<lSmpCntPerIvlU;jj++,jCnt++,jVal+=jInc)
//                 {
//                   // load SampleNormalizedVals with Normalized values
//                   sSampleNormalizedVals.SetAt(jCnt, (jVal-sUniqueKnotsU[0])/dParamLengthU) ;
//                 } // end iter BSplineSpan even stride sampling
//             } // end iter every BSplineSpanU
//  
//           // when working with a surface
//           if(pThisSurface)
//             {
//               // skip V Start Point value
//               jCnt++ ;  
//  
//               // for every V BSpline Span
//               for(ii=0,i1=1;ii<lIvlCntV;ii++,i1++)
//                 {
//                   double dStartP = sUniqueKnotsV[ii] ;
//                   double dEndP   = sUniqueKnotsV[i1] ;
//                   double jInc    = (dEndP - dStartP) / (double)(lSmpCntPerIvlV) ;
//                   double jVal    = dStartP + jInc ;
//  
//                   // for every internal Span U value - BSplineSpan even stride sampling
//                   for(jj=0;jj<lSmpCntPerIvlV;jj++,jCnt++,jVal+=jInc)
//                     {
//                       // load SampleNormalizedVals with Normalized values
//                       sSampleNormalizedVals.SetAt(jCnt, (jVal-sUniqueKnotsV[0])/dParamLengthV) ;
//                     } // end iter BSplineSpan even stride sampling
//                 } // end iter every BSplineSpanV
//             } // end working with Surface check
//          } // end ThisGeometry is BSpline geometry branch
//  
//        else // This Geometry is not a BSpline - even stride sampling
//          {
//            // size and fill Sample values
//            sSampleNormalizedVals.SetSize(pThisGF->m_lSampleCntU + pThisGF->m_lSampleCntV) ;
//  
//            // boundary values - set to avoid numerical tolerances
//            sSampleNormalizedVals.SetAt(0, 0.0) ;                // 1st U
//            sSampleNormalizedVals.SetAt(pThisGF->m_lSampleCntU-1, 1.0) ;  // last U
//            sSampleNormalizedVals.SetAt(pThisGF->m_lSampleCntU, 0.0) ;    // 1st V
//            if(pThisGF->m_lSampleCntV > 1)
//              { sSampleNormalizedVals.SetAt(pThisGF->m_lSampleCntU+pThisGF->m_lSampleCntV-1, 1.0) ; } // last V
//  
//            // for every internal U value - NaturalInterval even stride sampling
//            for(ii=2,jj=1;ii<pThisGF->m_lSampleCntU;ii++,jj++)
//              {
//                sSampleNormalizedVals.SetAt(jj, (double)jj/(double)(pThisGF->m_lSampleCntU-1) ) ;
//              }
//  
//            // for every internal V value - NaturalInterval even stride sampling
//            for(ii=2,jj=pThisGF->m_lSampleCntU+1;ii<pThisGF->m_lSampleCntV;ii++,jj++)
//              {
//                sSampleNormalizedVals.SetAt(jj, (double)jj/(double)(pThisGF->m_lSampleCntV-1) ) ;
//              }
//          } // end ThisGeometry is not BSpline Geometry branch
//      } // end Scope to build sampling point normalized param values
//  
//    // arrive here after m_lSampleCntU 
//    //                   m_lSampleCntV and
//    //             array sSampleNormalizedVals (as [u0, u1,.. uN, v0, v1,.. vN]) have been set 
//  
//    // Set memory in pThisGF->m_sSampleGaps (When working on curves m_pSampleCntV has been set to 1)
//    pThisGF->m_sSampleGaps.SetSize(pThisGF->m_lSampleCntU * pThisGF->m_lSampleCntV) ; 
//  
//    // locals
//    SmBoolean bUseV = (   pThisGF->m_eSampleType == SM_GS_CRV_SRF
//                       || pThisGF->m_eSampleType == SM_GS_SRF_SRF ) ;
//    double      *pLastU = NULL ;
//    double      *pLastV = NULL ;
//    SmGapSample *pLastSample = NULL ;
//  
//    // iteration statistics
//    ULONG  lSmpCnt   = 0 ;
//    double dMinGap   =  SM_BIG_DOUBLE ;
//    double dMaxGap   = -SM_BIG_DOUBLE ;
//    pThisGF->m_bAllNormalGaps = TRUE ; 
//    pThisGF->m_sOtherIvlU.Init() ; 
//    pThisGF->m_sOtherIvlV.Init() ;
//  
//    // for every sample point row
//    for(jj=0;
//        jj<pThisGF->m_lSampleCntV;
//        jj++,
//        pLastSample=&(pThisGF->m_sSampleGaps[(jj-1) * pThisGF->m_lSampleCntU]) )  // do 2nd dim as outer loop because all curves will have SampleCntV = 1
//      {
//        double dNormalParamV = sSampleNormalizedVals.GetAt(pThisGF->m_lSampleCntU + jj) ;
//        double dThisV        = pThisGF->m_sSampleIvlV.Evaluate(dNormalParamV) ;
//        ULONG  jCnt          = jj * pThisGF->m_lSampleCntU ;
//  
//        // for every sample point col entry
//        for(ii=0;
//            ii<pThisGF->m_lSampleCntU;
//            ii++,
//            pLastSample=&(pThisGF->m_sSampleGaps[jCnt+ii-1]), lSmpCnt++)
//          {
//            double dNormalParamU = sSampleNormalizedVals.GetAt(ii) ;
//            double dThisU        = pThisGF->m_sSampleIvlU.Evaluate(dNormalParamU) ;
//  
//            // evaluate and store this gap in row major order
//            if(pLastSample)
//              { pLastU = &(pLastSample->GetOtherParam().x) ;
//                if(bUseV)
//                  { pLastV = &(pLastSample->GetOtherParam().y) ; }
//              }
//  
//  #ifdef SM_DEBUG_CODE
//            if(bDebugMeWriteDetails)
//              { smos_sprintf(sBuff, _T("\n Added index:[%3d]"), jCnt + ii) ; smos_WriteBuffer(sBuff) ; }
//  
//            if(jCnt+11 > pThisGF->m_lSampleCntU * pThisGF->m_lSampleCntV)
//              { SM_ASSERT_MSG(jCnt+11 < pThisGF->m_lSampleCntU * pThisGF->m_lSampleCntV, 
//                              _T("SmGapFunction::BuildSampleSet - illegal indexing - needs debug")) ; 
//              }
//  #endif // SM_DEBUG_CODE
//  
//            SER(Evaluate(dThisU,                            // in : This Geom 1st param used for Curves and Surfaces
//                         dThisV,                            // in : This Geom 2nd param only used for Surfaces
//                         pThisGF->m_sSampleGaps[jCnt + ii], // out: Eval holder block
//                         pLastU,                            // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore
//                         pLastV )) ;                        // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore
//  
//            // note when a NonNormal Gap was found
//            if(pThisGF->m_sSampleGaps[jCnt + ii].GetGapDropType() != SM_GD_NORMAL)
//              { pThisGF->m_bAllNormalGaps = FALSE ; }
//  
//            // accumulate the drop Domain when the other object is not a point (it's a curve or surface)
//            if(GetOtherPoint() == NULL)
//              {
//                SmPoint2d &rDropPt2d = pThisGF->m_sSampleGaps[jCnt + ii].GetOtherParam() ;
//                pThisGF->m_sOtherIvlU.AddValue(rDropPt2d.x) ;
//                if(GetOtherSurface() != NULL) { pThisGF->m_sOtherIvlV.AddValue(rDropPt2d.y) ; }
//              }
//  
//            // save min and max indices
//            double dThisGap = pThisGF->m_sSampleGaps[jCnt + ii].GetLength() ;
//            if(dThisGap > dMaxGap) { dMaxGap = dThisGap ;
//                                     pThisGF->m_lMaxSampleIndex = jCnt + ii ;
//                                   }
//            if(dThisGap < dMinGap) { dMinGap = dThisGap ;
//                                     pThisGF->m_lMinSampleIndex = jCnt + ii ;
//                                   }
//  
//            // when asked - quit at the 1st out of tol gap
//            if(   bStopAfterBigGap
//               && dThisGap > pThisGF->m_sXSectTol3d)
//              { 
//                pThisGF->m_bAreSamplesSet = UNSURE ; 
//                return(SM_SUCCESS) ;
//              }
//  
//          } // end iter ii, all 1st dim Samples - evaluating GapSamples
//      } // end iter jj, all 2nd dim Samples
//  
//  #ifdef SM_DEBUG_CODE
//    if(lSmpCnt != pThisGF->m_lSampleCntU * pThisGF->m_lSampleCntV)
//      {
//        SM_ASSERT_MSG(lSmpCnt == pThisGF->m_lSampleCntU * pThisGF->m_lSampleCntV, _T("SmGapFunction::BuildSampleSet - illegal indexing - needs debug")) ;  
//      }
//  #endif // SM_DEBUG_CODE
//  
//    // remember the work
//    pThisGF->m_bAreSamplesSet = TRUE ;
//  
//    // scope to write header values and to compare when asked
//      {
//        // when asked get old values
//        // bDebugMeCompare: TRUE = Compare & rtn Old values, FALSE=Run New only, UNSURE=Run Old only
//        if(bDebugMeCompare == TRUE)
//          { 
//            // put new values into pGapFunctionCompare
//            Copy(pGapFunctionCompare) ; 
//  
//            // clear this object
//            pThisGF->m_bAreSamplesSet = FALSE ;
//  
//            // get old values in this memory
//            BuildSampleSet_OldDebug(bStopAfterBigGap) ;
//            
//            // when asked write Old value header
//            if(bDebugMeWriteHeader)
//              {
//                smos_sprintf(sBuff, _T("\n                                Old_MaxGap:[%16.16lf] Old_ThisParamX:[%16.16lf] Old_OtherParamX:[%16.16lf]"), 
//                           GetMaxGapSample().GetLength(),
//                           GetMaxGapSample().GetThisParam().x,
//                           GetMaxGapSample().GetOtherParam().x ) ; 
//                smos_WriteBuffer(sBuff) ;
//              }
//          } // end scope to copy new gap values into pGapFunctionCompare
//  
//        // when asked - write new value header
//        if(bDebugMeWriteHeader)
//          {
//            // bDebugMeCompare: TRUE = Compare & rtn Old values, FALSE=Run New only, UNSURE=Run Old only
//            SmGapFunction *pNewGF = (bDebugMeCompare == TRUE) ? pGapFunctionCompare : this ; 
//            smos_sprintf(sBuff, _T("\n                                New_MaxGap:[%16.16lf] New_ThisParamX:[%16.16lf] New_OtherParamX:[%16.16lf]"), 
//                       pNewGF->GetMaxGapSample().GetLength(),
//                       pNewGF->GetMaxGapSample().GetThisParam().x,
//                       pNewGF->GetMaxGapSample().GetOtherParam().x ) ; 
//            smos_WriteBuffer(sBuff) ;
//          }
//      } // end scope to write header values and to compare when asked
//  
//  #ifdef SM_DEBUG_CODE
//    // bDebugMeCompare: TRUE = Compare & rtn Old values, FALSE=Run New only, UNSURE=Run Old only
//    SmGapFunction *pNewGF =   (bDebugMeCompare == TRUE)  ? pGapFunctionCompare  
//                            : (bDebugMeCompare == FALSE) ? this : NULL ;
//    SmGapFunction *pOldGF = (bDebugMeCompare == TRUE || bDebugMeCompare == UNSURE) ? this : NULL ;
//    // cull out all but the problem cases
//    if(   (pThisGF->GetMaxGapSample().GetLength() > 0.0001 || pNewGF->GetMaxGapSample().GetLength() > 0.0001)
//       && (pNewGF->GetMaxGapSample().GetLength() > 20 * pThisGF->GetMaxGapSample().GetLength()))
//      {
//        if(bDebugMe)
//          {
//            if(pOldGF) pOldGF->Dump() ;
//            if(pNewGF) pNewGF->Dump() ;
//  
//            const SmCurve   *pFromCurve   = GetThisCurve() ;
//            const SmSurface *pFromSurface = GetThisSurface() ;
//            const SmPoint3d *pToPoint     = GetOtherPoint() ;
//            const SmCurve   *pToCurve     = GetOtherCurve() ;
//            const SmSurface *pToSurface   = GetOtherSurface() ;
//  
//            SmEdge *pFromEdge = pFromCurve   ? (SmEdge *)pFromCurve->GetEdge()   : NULL ;
//            SmFace *pFromFace = pFromSurface ? (SmFace *)pFromSurface->GetFace() : NULL ;
//            SmEdge *pToEdge   = pToCurve     ? (SmEdge *)pToCurve->GetEdge() : NULL ;
//            SmFace *pToFace   = pToSurface   ? (SmFace *)pToSurface->GetFace() : NULL ;
//            SmBrep *pFromBrep = pFromEdge ? pFromEdge->GetBrep() : pFromFace ? pFromFace->GetBrep() : NULL ;
//            SmBrep *pToBrep   = pToEdge ? pToEdge->GetBrep() : pToFace ? pToFace->GetBrep() : NULL ;
//  
//            smgfx_Erase() ;
//            smgfx_SetLook(1,2, 0, 0,1) ; if(pFromBrep) pFromBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
//            smgfx_SetLook(1,2, 0, 1,0) ; if(pToBrep)   pToBrep->Draw(TRUE) ;   sm_GraphicsLoop() ;
//            smgfx_SetLook(3,4, 0, 1,1) ; if(pFromCurve)   pFromCurve->Draw() ;     sm_GraphicsLoop() ;
//            smgfx_SetLook(1,2, 0, 1,1) ; if(pFromSurface) pFromSurface->DrawUV() ; sm_GraphicsLoop() ;
//            smgfx_SetLook(3,4, 1, 0,1) ; if(pToPoint)   pToPoint->Draw() ;     sm_GraphicsLoop() ;
//            smgfx_SetLook(3,4, 1, 0,1) ; if(pToCurve)   pToCurve->Draw() ;     sm_GraphicsLoop() ;
//            smgfx_SetLook(1,2, 1, 0,1) ; if(pToSurface) pToSurface->DrawUV() ; sm_GraphicsLoop() ;
//            smgfx_SetLook(5,6, 0, 0,0) ; if(pFromEdge) pFromEdge->Draw() ;                 sm_GraphicsLoop() ;
//            smgfx_SetLook(5,6, 0, 0,0) ; if(pFromFace) pFromFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
//            smgfx_SetLook(5,6, 0, 0,0) ; if(pToEdge) pToEdge->Draw() ;                 sm_GraphicsLoop() ;
//            smgfx_SetLook(5,6, 0, 0,0) ; if(pToFace) pToFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
//            smgfx_SetLook(1,8, 1, 0,0) ; if(pOldGF) pOldGF->Draw(TRUE) ; sm_GraphicsLoop() ;
//            smgfx_SetLook(1,8, 1,.5,0) ; if(pNewGF) pNewGF->Draw(TRUE) ; sm_GraphicsLoop() ;
//            sm_GraphicsLoop() ;
//          }
//      } // end culling check for quicker debugging
//  #endif // SM_DEBUG_CODE  
//  
//  // all done
//    return(SM_SUCCESS) ;
//  
//  } // end SmGapFunction::BuildSampleSet
// end SmGapFunction::BuildSampleSet version for New/Old method comparative debugging

/*******************************************************************//**
PURPOSE: Old BuildSampleSet - Debug only
***********************************************************************/
#ifdef SM_DEBUG_CODE
SmStatus SmGapFunction::BuildSampleSet_OldDebug
 (SmBoolean bStopAfterBigGap) // in : FALSE = compute all gaps
                              //      TRUE  = Quit when the 1st gap bigger than tol is found 
                              //      default:[FALSE]
{
  // local for debugging
  SmGapFunction *pThisGF = this ;
  SmGapSample *pMaxGap;
  GetMaxGapSample( pMaxGap, TRUE );

  // no work - samples already set or already have a gap bigger than tol
  if(   pThisGF->m_bAreSamplesSet == TRUE
     || (   pThisGF->m_bAreSamplesSet == UNSURE
         && bStopAfterBigGap == TRUE
         && pMaxGap
         && pMaxGap->GetLengthSquared() > pThisGF->m_sXSectTol3d*pThisGF->m_sXSectTol3d)) 
    { return SM_SUCCESS ; }

  // Set memory in pThisGF->m_sSampleGaps
  pThisGF->m_sSampleGaps.SetSize(pThisGF->m_lSampleCntU * pThisGF->m_lSampleCntV) ; 

  // locals
  SmBoolean bUseV = (   pThisGF->m_eSampleType == SM_GS_CRV_SRF
                     || pThisGF->m_eSampleType == SM_GS_SRF_SRF ) ;
  ULONG ii, jj ;
  double      *pLastU = NULL ;
  double      *pLastV = NULL ;
  SmGapSample *pLastSample = NULL ;

  // iteration statistics
  double dMinGap   =  SM_BIG_DOUBLE ;
  double dMaxGap   = -SM_BIG_DOUBLE ;
  pThisGF->m_bAllNormalGaps = TRUE ; 
  pThisGF->m_sOtherIvlU.Init() ; 
  pThisGF->m_sOtherIvlV.Init() ;

  // for every sample point row
  for(jj=0;
      jj<pThisGF->m_lSampleCntV;
      jj++,
      pLastSample=&(pThisGF->m_sSampleGaps[(jj-1)* pThisGF->m_lSampleCntU]) )  // do 2nd dim as outer loop 'cause all curves will have SampleCntV = 1
    {
      double dNormalParamV = pThisGF->m_lSampleCntV == 1 ? 0.0 : ((double)jj/(double)(pThisGF->m_lSampleCntV-1)) ;
      double dThisV        = pThisGF->m_sSampleIvlV.Evaluate(dNormalParamV) ;
      ULONG  jCnt          = jj * pThisGF->m_lSampleCntU ;

      // for every sample point col entry
      for(ii=0;
          ii<pThisGF->m_lSampleCntU;
          ii++,
          pLastSample=&(pThisGF->m_sSampleGaps)[jCnt+ii-1])
        {
          double dNormalParamU = pThisGF->m_lSampleCntU == 1 ? 0.0 : ((double)ii/(double)(pThisGF->m_lSampleCntU-1)) ;
          double dThisU        = pThisGF->m_sSampleIvlU.Evaluate(dNormalParamU) ;

          // evaluate and store this gap in row major order
          if(pLastSample)
            { pLastU = &(pLastSample->GetOtherParam().x) ;
              if(bUseV)
                { pLastV = &(pLastSample->GetOtherParam().y) ; }
            }
          SER(Evaluate(dThisU, dThisV, pThisGF->m_sSampleGaps[jCnt + ii], pLastU, pLastV )) ;

          // note when a NonNormal Gap was found
          if(pThisGF->m_sSampleGaps[jCnt + ii].GetGapDropType() != SM_GD_NORMAL)
            { pThisGF->m_bAllNormalGaps = FALSE ; }

          // accumulate the drop Domain when the other object is not a point (it's a curve or surface)
          if(GetOtherPoint() == NULL)
            {
              SmPoint2d &rDropPt2d = pThisGF->m_sSampleGaps[jCnt + ii].GetOtherParam() ;
              pThisGF->m_sOtherIvlU.AddValue(rDropPt2d.x) ;
              if(GetOtherSurface() != NULL) { pThisGF->m_sOtherIvlV.AddValue(rDropPt2d.y) ; }
            }

          // save min and max indices
          double dThisGap = pThisGF->m_sSampleGaps[jCnt + ii].GetLength() ;
          if(dThisGap > dMaxGap) { dMaxGap = dThisGap ;
                                   pThisGF->m_lMaxSampleIndex = jCnt + ii ;
                                 }
          if(dThisGap < dMinGap) { dMinGap = dThisGap ;
                                   pThisGF->m_lMinSampleIndex = jCnt + ii ;
                                 }

          // when asked - quit at the 1st out of tol gap
          if(   bStopAfterBigGap
             && dThisGap > pThisGF->m_sXSectTol3d)
            { 
              pThisGF->m_bAreSamplesSet = UNSURE ; 
              return(SM_SUCCESS) ;
            }

        } // end iter ii, all 1st dim Samples - evaluating GapSamples
    } // end iter jj, all 2nd dim Samples

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump() ;

      const SmCurve   *pFromCurve   = GetThisCurve() ;
      const SmSurface *pFromSurface = GetThisSurface() ;
      const SmPoint3d *pToPoint     = GetOtherPoint() ;
      const SmCurve   *pToCurve     = GetOtherCurve() ;
      const SmSurface *pToSurface   = GetOtherSurface() ;

      SmEdge *pFromEdge = pFromCurve   ? (SmEdge *)pFromCurve->GetEdge()   : NULL ;
      SmFace *pFromFace = pFromSurface ? (SmFace *)pFromSurface->GetFace() : NULL ;
      SmEdge *pToEdge   = pToCurve     ? (SmEdge *)pToCurve->GetEdge() : NULL ;
      SmFace *pToFace   = pToSurface   ? (SmFace *)pToSurface->GetFace() : NULL ;
      SmBrep *pFromBrep = pFromEdge ? pFromEdge->GetBrep() : pFromFace ? pFromFace->GetBrep() : NULL ;
      SmBrep *pToBrep   = pToEdge ? pToEdge->GetBrep() : pToFace ? pToFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pFromBrep) pFromBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pToBrep)   pToBrep->Draw(TRUE) ;   sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; if(pFromCurve)   pFromCurve->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pFromSurface) pFromSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pToPoint)   pToPoint->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pToCurve)   pToCurve->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1) ; if(pToSurface) pToSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pFromEdge) pFromEdge->Draw() ;                 sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pFromFace) pFromFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pToEdge) pToEdge->Draw() ;                 sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pToFace) pToFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,8, 1,0,0) ; Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE    


  // remember the work
  pThisGF->m_bAreSamplesSet = TRUE ;

  // all done
  return(SM_SUCCESS) ;

} // end SmGapFunction::BuildSampleSet_OldDebug
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Evaluate the saved samples used for graphics when needed

NOTES:
***********************************************************************/
SmStatus SmGapFunction::BuildLocalNeighborhood() 
{
  // no work - samples already set
  if(m_bIsLocalNeighborhoodSet) 
    { return SM_SUCCESS ; }

  // locals
  ULONG ii ;
  // Local neighborhoods currently only supported for FromGeometry==Curve
  //  and not supported for SmCrvSrfGapFunction until SmSurface::GlobalCurveSolve() is extended for SM_SO_AT_DISTANCE
  SmCurveClassification *pCurveClassification = GetLocalNeighborhood(TRUE) ;   // TRUE = fetch as is, without running BuildLocalNeighborhood
  const SmCurve         *pCurve        = GetThisCurve () ;
  const SmPoint3d       *pOtherPoint   = GetOtherPoint() ;
  const SmCurve         *pOtherCurve   = GetOtherCurve() ;
  const SmSurface       *pOtherSurface = GetOtherSurface() ;
  SmSolutionArray sSolutions ;

  // check state - Curves and Curve classifications go together
  SM_ASSERT(   (pCurve == NULL &&  pCurveClassification == NULL)
            || (pCurve != NULL &&  pCurveClassification != NULL)) ;

  // no work - this derived class does not yet have a pLocalNeighborhood definition (true for FromSurfaces and FromSurfToCurve)
  if(pCurveClassification == NULL)
    { return SM_SUCCESS ; }

  // arrive here when we need to classify m_pThisCurve shape against the Other LocalShape

  // clear any current classifications
  pCurveClassification->ReSet() ;

  // solve for interval boundaries
  EvaluateNeighborhoodBoundaries(sSolutions) ;

  // make intervals from solution boundaries
  pCurveClassification->InsertTopologyIntersections(sSolutions, 0) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      pCurveClassification->Dump() ;

      const SmCurve   *pFromCurve   = GetThisCurve() ;
      const SmSurface *pFromSurface = GetThisSurface() ;
      const SmPoint3d *pToPoint     = GetOtherPoint() ;
      const SmCurve   *pToCurve     = GetOtherCurve() ;
      const SmSurface *pToSurface   = GetOtherSurface() ;

      SmEdge *pFromEdge = pFromCurve   ? (SmEdge *)pFromCurve->GetEdge()   : NULL ;
      SmFace *pFromFace = pFromSurface ? (SmFace *)pFromSurface->GetFace() : NULL ;
      SmEdge *pToEdge   = pToCurve     ? (SmEdge *)pToCurve->GetEdge() : NULL ;
      SmFace *pToFace   = pToSurface   ? (SmFace *)pToSurface->GetFace() : NULL ;
      SmBrep *pFromBrep = pFromEdge ? pFromEdge->GetBrep() : pFromFace ? pFromFace->GetBrep() : NULL ;
      SmBrep *pToBrep   = pToEdge ? pToEdge->GetBrep() : pToFace ? pToFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pFromBrep) pFromBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pToBrep)   pToBrep->Draw(TRUE) ;   sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; if(pFromCurve)   pFromCurve->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pFromSurface) pFromSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pToPoint)   pToPoint->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pToCurve)   pToCurve->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1) ; if(pToSurface) pToSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pFromEdge) pFromEdge->Draw() ;                 sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pFromFace) pFromFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pToEdge) pToEdge->Draw() ;                 sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pToFace) pToFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,8, 1,0,0) ; pCurveClassification->Draw(FALSE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // classify all intervals as in/out
  for(ii=0;ii<pCurveClassification->GetSize();ii++)
    {
      // get this interval's mid param value
      SmCurveInterval &rCurveIvl = (*pCurveClassification)[ii] ;
      double           dParam    = rCurveIvl.GetInterval().Evaluate(0.5) ;

      // evaluate the gap at this param
      // GWC NOTE: if we had the gap vectors stored at the interval boundaries the interval
      //           could be classified by the tangents of the ThisCurve and the otherShape gap ends.
      //           Different rules could be generated for the cases where otherShape was a point, a curve,
      //           or a surface.  That would save one more drop point calculation.
      SmGapSample sGapSample(GetXSectTol3d()) ;
      Evaluate(dParam,       // in : This Geom 1st param used for Curves and Surfaces
               0.0,          // in : This Geom 2nd param only used for Surfaces
               sGapSample,   // out: Eval holder block
               NULL,         // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore
               NULL) ;       // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore

      // classify the interval from the gap classification
      if ( sGapSample.IsInTol() )
        {
          if ( pOtherPoint != NULL )
            { rCurveIvl.m_vMid.SetClassObject( SM_PC_POINT,   (SmObject*)pOtherPoint ); }
          else if ( pOtherCurve != NULL )
            { rCurveIvl.m_vMid.SetClassObject( SM_PC_CURVE,   (SmObject*)pOtherCurve ); }
          else if ( pOtherSurface != NULL )
            { rCurveIvl.m_vMid.SetClassObject( SM_PC_SURFACE, (SmObject*)pOtherSurface ); }
          else
            { rCurveIvl.m_vMid.SetClassObject( SM_PC_UNKNOWN, NULL ); }
        }
      else // gap is bigger than tol
        {
          rCurveIvl.m_vMid.SetClassObject( SM_PC_UNKNOWN, NULL );  // used for outside the classification interval
        }

    } // end iter all intervals

#ifdef SM_DEBUG_CODE
  if(bDebugMe)
    {
      pCurveClassification->Dump() ;

      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // remember the work
  m_bIsLocalNeighborhoodSet = TRUE ;

  // all done
  return(SM_SUCCESS) ;

} // end SmGapFunction::BuildLocalNeighborhood

/*******************************************************************//**
PURPOSE: Get a list of local neighborhoods where this geometry
 is within tolerance of some points on the other geometry.

NOTES:
***********************************************************************/
void SmGapFunction::GetLocalIntervals
 (SmTArray<SmLocalInterval> &rLocalIvls)      // out: list of local intervals
  const
{ 
  // init output
  rLocalIvls.ReSet() ;

  // fetch Local Neighborhood - a lazy evaluation  (CRV_SRF GapFunctions don't yet support LocalNeighborhoods)
  SmCurveClassification * pCurveClassification = ((SmGapFunction*)this)->GetLocalNeighborhood() ;
  const SmCurve         * pThisCurve           = pCurveClassification ? pCurveClassification->GetCurve() : NULL ;
  double                  dXSectTol3d          = ((SmGapFunction*)this)->GetXSectTol3d() ;

  // no work for surfaces
  if(pThisCurve == NULL) 
    { return ; }

  // locals
  ULONG ii ;
  const SmPoint3d * pOtherPoint   = GetOtherPoint() ;
  const SmCurve   * pOtherCurve   = GetOtherCurve() ;
  const SmSurface * pOtherSurface = GetOtherSurface() ;
  SmExtent1d sTmp ;
  SmExtent1d sOtherIvlU =   pOtherCurve   ? pOtherCurve->GetNaturalInterval()  
                     : pOtherSurface ? pOtherSurface->GetNaturalUVDomain().GetUInterval() 
                     : sTmp ;
  SmExtent1d sOtherIvlV =   pOtherSurface ? pOtherSurface->GetNaturalUVDomain().GetVInterval() 
                     : sTmp ;

  SM_ASSERT(pThisCurve == GetThisCurve()) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      this->Dump() ;   // should include pCurveClassification->Dump() ;
      pCurveClassification->Dump() ;
    }
#endif // SM_DEBUG_CODE

  // for every interval - build and store a SmLocalInterval object
  for(ii=0;ii<pCurveClassification->GetSize();ii++)
    {
      SmCurveInterval       &rCurveIvl = (*pCurveClassification)[ii] ;
      SmPointClassification &rBegPoint = rCurveIvl.m_vStart ;
      SmPointClassification &rMidPoint = rCurveIvl.m_vMid ;
      SmPointClassification &rEndPoint = rCurveIvl.m_vEnd ;

      // skip outside intervals
      if(rMidPoint.GetPointClass() == SM_PC_UNKNOWN)
        { continue ; }
        
      // build and add SmLocalInterval to output array

      // This param value: for now - only this curves are supported
      SmPoint2d sThisBegParam ( rCurveIvl.m_vInterval.GetMin(), 0.0) ; 
      SmPoint2d sThisEndParam ( rCurveIvl.m_vInterval.GetMax(), 0.0) ;

      // Other param value: other shapes can be a point, curve, or surface
      SmPoint2d sOtherBegParam(   rBegPoint.GetPointClass() == SM_PC_POINT ? 0.0
                                : rBegPoint.GetPointClass() == SM_PC_CURVE ? rBegPoint.GetTParam() 
                                : rBegPoint.GetUVParam().x,
                                  rBegPoint.GetPointClass() == SM_PC_POINT ? 0.0
                                : rBegPoint.GetPointClass() == SM_PC_CURVE ? 0.0 
                                : rBegPoint.GetUVParam().y) ;
      SmPoint2d sOtherEndParam(   rEndPoint.GetPointClass() == SM_PC_POINT ? 0.0
                                : rEndPoint.GetPointClass() == SM_PC_CURVE ? rEndPoint.GetTParam()  
                                : rEndPoint.GetUVParam().x,
                                  rEndPoint.GetPointClass() == SM_PC_POINT ? 0.0
                                : rEndPoint.GetPointClass() == SM_PC_CURVE ? 0.0 
                                : rEndPoint.GetUVParam().y) ;
      // pos locals
      SmPoint3d sThisBegPos, sOtherBegPD[2][2] ;
      SmPoint3d sThisEndPos, sOtherEndPD[2][2] ;

      // This pos: for now - all this geometry is a curve 
      pThisCurve->EvaluatePoint(sThisBegParam.x, sThisBegPos) ;
      pThisCurve->EvaluatePoint(sThisEndParam.x, sThisEndPos) ;

      // Other pos
      if     (pOtherPoint) { sOtherBegPD[0][0] = *pOtherPoint ; 
                             sOtherEndPD[0][0] = *pOtherPoint ;
                           }
      else if(pOtherCurve) { pOtherCurve->Evaluate(sOtherBegParam.x, 1, TRUE, sOtherBegPD[0], TRUE) ; // NonZeroTangents
                             pOtherCurve->Evaluate(sOtherEndParam.x, 1, TRUE, sOtherEndPD[0], TRUE) ; // NonZeroTangents
                             sOtherBegPD[1][0] = sOtherBegPD[0][1] ; // init cross-tangents for upcoming GapDropType check
                             sOtherEndPD[1][0] = sOtherEndPD[0][1] ;
                           }
      else                 { SM_ASSERT(pOtherSurface != NULL) ;
                             pOtherSurface->Evaluate(sOtherBegParam, 1, 1, TRUE, TRUE, TRUE, sOtherBegPD[0], TRUE) ; // NonZeroTangents
                             pOtherSurface->Evaluate(sOtherEndParam, 1, 1, TRUE, TRUE, TRUE, sOtherEndPD[0], TRUE) ; // NonZeroTangents
                           }

      // gapDropTypes
      SmVector3d sBegGap = sOtherBegPD[0][0] - sThisBegPos ;
      SmVector3d sEndGap = sOtherEndPD[0][0] - sThisEndPos ;
      SmGapDropType eGapDropBegType =    pOtherPoint
                                      || sBegGap.LengthSquared() < SM_EFF_ZERO_SQ
                                      || (   (sOtherIvlU.IsInit() || !sOtherIvlU.IsValueOnBoundary(sOtherBegParam.x))
                                          && (sOtherIvlV.IsInit() || !sOtherIvlV.IsValueOnBoundary(sOtherBegParam.y)))
                                      || (   sOtherBegPD[0][1].IsPerpendicularTo(sBegGap, 0.5) 
                                          && sOtherBegPD[1][0].IsPerpendicularTo(sBegGap, 0.5))
                                      ? SM_GD_NORMAL
                                      : SM_GD_BOUNDARY ;
      SmGapDropType eGapDropEndType =    pOtherPoint
                                      || sEndGap.LengthSquared() < SM_EFF_ZERO_SQ
                                      || (   (sOtherIvlU.IsInit() || !sOtherIvlU.IsValueOnBoundary(sOtherEndParam.x))
                                          && (sOtherIvlV.IsInit() || !sOtherIvlV.IsValueOnBoundary(sOtherEndParam.y)))
                                      || (   sOtherEndPD[0][1].IsPerpendicularTo(sEndGap, 0.5) 
                                          && sOtherEndPD[1][0].IsPerpendicularTo(sEndGap, 0.5))
                                      ? SM_GD_NORMAL
                                      : SM_GD_BOUNDARY ;

      // build the GapSamples
      SmGapSample sMinSample(sThisBegPos, sThisBegParam.x, sThisBegParam.y,
                             sOtherBegPD[0][0], sOtherBegParam.x, sOtherBegParam.y,
                             eGapDropBegType, dXSectTol3d) ;
      SmGapSample sMaxSample(sThisEndPos, sThisEndParam.x, sThisEndParam.y,
                             sOtherEndPD[0][0], sOtherEndParam.x, sOtherEndParam.y,
                             eGapDropEndType, dXSectTol3d) ;

      // build the LocalInterval and save it
      SmLocalInterval sLocalIvl(sMinSample, sMaxSample) ;
      rLocalIvls.Add(sLocalIvl) ; 

    } // end ier every interval building SmLocalInterval objects

} // end SmGapFunction::GetLocalIntervals

/*******************************************************************//**
PURPOSE: Draw SmGapFunction graphics

NOTES: Each InTol GapSample is drawn as 2 points and a line from the
         ThisGeometry point to the OtherGeometry point.
       Each OutTol GapSample is draw twice, once at scale 1 and once
         at scale dDisplayScale (so that out-of-tol gaps are easy to see)
         and is drawn as 3 points and 2 lines as

      ThisGeomPt    TolPoint    OtherGeomPt
         +--------------+----------+      where: Dist(ThisGeomPt,TolPt) = XSectTol3d
             InTol         OutTol
          Color Portion  Color Portion

      The OutTol Gap vector is displayed in two different colored 
      portions showing the InTol and OutTol segments of the gap vector
      so that it's easy to see when a gap function exceeds tolerance
      by a small or a large amount.
***********************************************************************/
SmDisplayList * SmGapFunction::Draw
 (SmBoolean       bShowGeometry,             // NotUsed: in : TRUE = Show Geometry bounding the gaps, default:[TRUE]
  SmBoolean       bShowNeighborhoods,        // in : TRUE = Show InTol GapFunction neighborhood intervals, default:[TRUE]
  SmBoolean       bShowSamples,              // in : TRUE  = show all already evaluated sample gaps, 
                                             //      UNSURE= Only show already evaluated bigger-than-tol gaps, 
                                             //      FALSE = Show no sample gaps,
                                             //      default:[FALSE]
  double          dDisplayScale,             // in : Amount micro geometry is scaled for visualization, default:[1000]
  SmGfxArraySet * pOptGfxSet)                // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
                                             //      NULL to ignore. default:[NULL]
 const
{
  SM_REF1(bShowGeometry) ;
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // locals
  ULONG i0, i1 ;
  ULONG lCnt = GetSize() ;

  // gwc - don't let draw change the state of the GapFunction
  //    // make sure the gap samples are available
  //    ((SmGapFunction *)this)->BuildSampleSet() ; 

  // locals for micro graphics - for now, limited to curves
  //const SmCurve *pThisCurve  = GetThisCurve() ; 
  //const SmCurve *pOtherCurve = GetOtherCurve() ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump() ;
    }
#endif // SM_DEBUG_CODE

  // start new displayList (unless one is already open)
  // use gap color and draw sizes rather than default ones
  double dLW = 1 ;
  double dPS = 4 ;
  smgfx_Open(smgfx_GetInTolGapColor(), &dLW, &dPS, FALSE, pOptGfxSet);

  // when asked Draw Local Neighborhood - for now draw the curve interval (nothing yet for surfaces)
  if(bShowNeighborhoods)
    {
      SmTArray<SmLocalInterval> sLocalIvls ;
      GetLocalIntervals(sLocalIvls) ;

      // for every local interval - generate a boundary graphic
      for(i0=0;i0<sLocalIvls.GetSize();i0++)
        {
          smgfx_SetLook(dLW, dPS+5, 1,0,0, pOptGfxSet) ;
          sLocalIvls[i0].Draw(NULL, pOptGfxSet) ;
        }
    } // end when asked to draw neighborhoods check
  
  // things to draw:  1. InTol gap vector               smgfx_GetInTolGapColor(),  dLineWidth
  //                  2. InTol ThisPos points           smgfx_GetInTolGapColor(),  dPointSize + 2
  //                  3. InTol ThisPos segments         smgfx_GetInTolGapColor(),  dLineWidth + 2
  //                  3.a InTol ThisPos vicinity         smgfx_GetInTolGapColor(),  dLineWidth + 2
  //                  4. InTol OtherPos points          smgfx_GetOutTolGapColor(), dPointSize + 4
  //                  5. InTol otherPos segments        smgfx_GetOutTolGapColor(), dLineWidth + 2
  //                  5.a InTol otherPos vicinity        smgfx_GetOutTolGapColor(), dLineWidth + 2
  //   
  //                  6. InTol  portion of gap vector   smgfx_GetInTolGapColor(),  dLineWidth
  //                  7. OutTol portion of gap vector   smgfx_GetOutTolGapColor(), dLineWidth
  //                  8. OutTol ThisPos points          smgfx_GetInTolGapColor(),  dPointSize
  //                  9. OutTol ThisPos segments        smgfx_GetInTolGapColor(),  dLineWidth
  //                  9.a OutTol ThisPos vicinity        smgfx_GetInTolGapColor(),  dLineWidth
  //                 10. OutTol OtherPos points         smgfx_GetOutTolGapColor(), dPointSize + 2
  //                 11. OutTol otherPos segments       smgfx_GetOutTolGapColor(), dLineWidth    
  //                 11.a OutTol otherPos vicinity       smgfx_GetOutTolGapColor(), dLineWidth    
  // Gap Sample Graphics - Faster graphics if all things of common color and size are drawn together
  //                       Faster simpler code in one loop.  It's a trade off.  For now - simplify the code
  if(   (bShowSamples  == TRUE || bShowSamples == UNSURE)
     && (m_bAreSamplesSet == TRUE))
    {
      SmVector3d    sInCol  = smgfx_GetInTolGapColor() ;
      SmVector3d    sOutCol = smgfx_GetOutTolGapColor() ;
      SmPoint3d     sOtherScalePt, sTolScalePt ;
      //const SmXSectTol3d * psXSectTol3d  = &m_sXSectTol3d ;

      // Draw InTol gap vector segs and this Curve points
      SmVector3d sRestoreColor     = smgfx_GetColor() ;
      double     dRestoreLineWidth = smgfx_GetLineWidth() ;
      double     dRestorePointSize = smgfx_GetPointSize() ;
      for(i0=0,i1=1;i0<lCnt;i0++,i1++)
        {
          SmGapSample &rGap0 = m_sSampleGaps.GetDataArray()[i0] ;
          SmGapSample &rGap1 = i1 < lCnt ? m_sSampleGaps.GetDataArray()[i1] : m_sSampleGaps.GetDataArray()[i0] ;

          if(rGap0.IsInTol()) 
            { 
              // when showing all or just bad gaps - always show the good gaps at scale 1

              // 1. inTol gap vector,         ThisPos[0] <-> OtherPos[0]
              smgfx_SetLook(dLW, dPS, sInCol, pOptGfxSet);    smgfx_OutputLine (rGap0.GetThisPos().x,  rGap0.GetThisPos().y,  rGap0.GetThisPos().z,
                                                                    rGap0.GetOtherPos().x, rGap0.GetOtherPos().y, rGap0.GetOtherPos().z, pOptGfxSet) ;
              // 2. InTol ThisPos points,     ThisPos[0]        
              smgfx_SetLook(dLW, dPS+2, sInCol, pOptGfxSet);  smgfx_OutputPoint(rGap0.GetThisPos().x,  rGap0.GetThisPos().y,  rGap0.GetThisPos().z, pOptGfxSet) ;
                                                  
              // 3. InTol ThisPos segments,   ThisPos[0] <-> ThisPos[1]        
              smgfx_SetLook(dLW+2, dPS, sInCol, pOptGfxSet);  smgfx_OutputLine (rGap0.GetThisPos().x,  rGap0.GetThisPos().y,  rGap0.GetThisPos().z,
                                                                    rGap1.GetThisPos().x,  rGap1.GetThisPos().y,  rGap1.GetThisPos().z, pOptGfxSet) ; 

              // 4. InTol OtherPos points,    OtherPos[0]         
              smgfx_SetLook(dLW, dPS, sOutCol, pOptGfxSet); smgfx_OutputPoint(rGap0.GetOtherPos().x,  rGap0.GetOtherPos().y,  rGap0.GetOtherPos().z, pOptGfxSet) ;
           
              // 5. InTol otherPos segments,  OtherPos[0] <-> OtherPos[1]
              smgfx_SetLook(dLW+2, dPS, sOutCol, pOptGfxSet); smgfx_OutputLine (rGap0.GetOtherPos().x,  rGap0.GetOtherPos().y,  rGap0.GetOtherPos().z,
                                                                    rGap1.GetOtherPos().x,  rGap1.GetOtherPos().y,  rGap1.GetOtherPos().z, pOptGfxSet) ; 

              // when showing all gaps - show good gaps at dDisplayScale
              if(bShowSamples == TRUE)
                {
                  sOtherScalePt = (rGap0.GetOtherPos() - rGap0.GetThisPos()) * dDisplayScale + rGap0.GetThisPos() ;

                  // draw some of the geometry a 2nd time at scale = dDisplayScale
                  sOtherScalePt = (rGap0.GetOtherPos() - rGap0.GetThisPos()) * dDisplayScale + rGap0.GetThisPos() ;

                  // 2nd 6. dDisplayScale inTol portion of gap vector
              smgfx_SetLook(dLW, dPS, sInCol, pOptGfxSet);    smgfx_OutputLine (rGap0.GetThisPos().x,  rGap0.GetThisPos().y,  rGap0.GetThisPos().z,
                                                                    sOtherScalePt.x, sOtherScalePt.y, sOtherScalePt.z, pOptGfxSet) ;
                  // 2nd 10. dDisplayScale OutTol OtherPos point         
                  smgfx_SetLook(dLW, dPS, sOutCol, pOptGfxSet); smgfx_OutputPoint(sOtherScalePt.x, sOtherScalePt.y, sOtherScalePt.z, pOptGfxSet) ;
                } // end Show All Gaps check
            } // end Gap is okay branch
          else // Gap is too big branch               
            { 
              // draw geometry at scale 1.0 
                  
              // 6. Scale 1 inTol portion of gap vector
              smgfx_SetLook(dLW, dPS, sInCol, pOptGfxSet);    smgfx_OutputLine (rGap0.GetThisPos().x,  rGap0.GetThisPos().y,  rGap0.GetThisPos().z,
                                                                    rGap0.GetTolPoint().x, rGap0.GetTolPoint().y, rGap0.GetTolPoint().z, pOptGfxSet) ;
              // 7. Scale 1 OutTol portion of gap vector
              smgfx_SetLook(dLW, dPS, sOutCol, pOptGfxSet);   smgfx_OutputLine (rGap0.GetTolPoint().x, rGap0.GetTolPoint().y, rGap0.GetTolPoint().z,
                                                                    rGap0.GetOtherPos().x, rGap0.GetOtherPos().y, rGap0.GetOtherPos().z, pOptGfxSet) ;
              // 8. Scale 1 OutTol ThisPos point          
              smgfx_SetLook(dLW, dPS, sInCol, pOptGfxSet);    smgfx_OutputPoint(rGap0.GetThisPos().x,  rGap0.GetThisPos().y,  rGap0.GetThisPos().z, pOptGfxSet) ;
                                                  
              // 9. Scale 1 OutTol ThisPos segments        
              smgfx_SetLook(dLW, dPS, sInCol, pOptGfxSet);    smgfx_OutputLine (rGap0.GetThisPos().x,  rGap0.GetThisPos().y,  rGap0.GetThisPos().z,
                                                                    rGap1.GetThisPos().x,  rGap1.GetThisPos().y,  rGap1.GetThisPos().z, pOptGfxSet) ; 
              // 10. Scale 1 OutTol OtherPos point         
              smgfx_SetLook(dLW, dPS+2, sOutCol, pOptGfxSet); smgfx_OutputPoint(rGap0.GetOtherPos().x,  rGap0.GetOtherPos().y,  rGap0.GetOtherPos().z, pOptGfxSet) ;
           
              // 11. Scale 1 OutTol otherPos segments
              smgfx_SetLook(dLW, dPS, sOutCol, pOptGfxSet);   smgfx_OutputLine (rGap0.GetOtherPos().x,  rGap0.GetOtherPos().y,  rGap0.GetOtherPos().z,
                                                                    rGap1.GetOtherPos().x,  rGap1.GetOtherPos().y,  rGap1.GetOtherPos().z, pOptGfxSet) ; 
            
              // 12. dDisplayScale OutTol TolScale point         
              smgfx_SetLook(dLW, dPS, sOutCol, pOptGfxSet);   smgfx_OutputPoint(rGap0.GetTolPoint().x, rGap0.GetTolPoint().y, rGap0.GetTolPoint().z, pOptGfxSet) ;
           
              // for graphics to be drawn at scale 1 and again at dDisplayScale (to highlight gap functions which are out of tolerance)
              
              // draw some of the geometry a 2nd time at scale = dDisplayScale

              sOtherScalePt = (rGap0.GetOtherPos() - rGap0.GetThisPos()) * dDisplayScale + rGap0.GetThisPos() ;
              sTolScalePt   = (rGap0.GetTolPoint() - rGap0.GetThisPos()) * dDisplayScale + rGap0.GetThisPos() ;

              // 2nd 6. dDisplayScale inTol portion of gap vector
              smgfx_SetLook(dLW, dPS, sInCol, pOptGfxSet);    smgfx_OutputLine (rGap0.GetThisPos().x,  rGap0.GetThisPos().y,  rGap0.GetThisPos().z,
                                                                    sTolScalePt.x, sTolScalePt.y, sTolScalePt.z, pOptGfxSet) ;
              // 2nd 7. dDisplayScale OutTol portion of gap vector
              smgfx_SetLook(dLW, dPS, sOutCol, pOptGfxSet);   smgfx_OutputLine (sTolScalePt.x, sTolScalePt.y, sTolScalePt.z,
                                                                    sOtherScalePt.x, sOtherScalePt.y, sOtherScalePt.z, pOptGfxSet) ;
              // 2nd 10. dDisplayScale OutTol OtherPos point         
              smgfx_SetLook(dLW, dPS+2, sOutCol, pOptGfxSet); smgfx_OutputPoint(sOtherScalePt.x,  sOtherScalePt.y,  sOtherScalePt.z, pOptGfxSet) ;
           
              // 2nd 12. dDisplayScale OutTol TolScale point         
              smgfx_SetLook(dLW, dPS, sOutCol, pOptGfxSet);   smgfx_OutputPoint(sTolScalePt.x, sTolScalePt.y, sTolScalePt.z, pOptGfxSet) ;

            } // end Gap is too big branch
        } // end iter every gap - drawing in tol gap segment

      // restore modified output parameters
      smgfx_SetColor(sRestoreColor, pOptGfxSet) ;    
      smgfx_SetLineWidth(dRestoreLineWidth, pOptGfxSet) ;
      smgfx_SetPointSize(dRestorePointSize, pOptGfxSet) ;

    } // end draw GapSamples check

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF4(bShowNeighborhoods, bShowSamples, dDisplayScale, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmGapFunction::Draw

/*******************************************************************//**
PURPOSE: Pretty print SmGapFunction data

NOTES: 1. called after SmDerivedGapFunction::Dump() outputs a Derived GapFunction label
       2. No SideEffects.  Dump is often called in a debug block. Don't let 
           Print calls cause side effects through lazy evaluations. 
***********************************************************************/
void SmGapFunction::Dump()                  const { SmGapFunction::Dump(FALSE) ; }
void SmGapFunction::Dump(SmBoolean bAbbrev) const
{
  ULONG ii ;
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  smos_sprintf(sBuff, _T("%s"), _T("\n  Begin Base SmGapFunction::Dump()")) ;
  smos_WriteBuffer(sBuff);

  // gwc RmLine: don't build sample sets - Dump is often run in debug mode so Dump() should have no side effects
  //  ((SmGapFunction *)this)->BuildSampleSet() ;

  // Type, XSectTol3d
  SmBoolean bThisCrv  = (   m_eSampleType == SM_GS_CRV_PT
                         || m_eSampleType == SM_GS_CRV_CRV 
                         || m_eSampleType == SM_GS_CRV_SRF) ;
  SmBoolean bThisSrf  = (   m_eSampleType == SM_GS_SRF_PT
                         || m_eSampleType == SM_GS_SRF_CRV 
                         || m_eSampleType == SM_GS_SRF_SRF) ;
  //SmBoolean bOtherPt  = (   m_eSampleType == SM_GS_CRV_PT
  //                       || m_eSampleType == SM_GS_SRF_PT) ;
  SmBoolean bOtherCrv = (   m_eSampleType == SM_GS_CRV_CRV
                         || m_eSampleType == SM_GS_SRF_CRV) ;
  SmBoolean bOtherSrf = (   m_eSampleType == SM_GS_CRV_SRF
                         || m_eSampleType == SM_GS_SRF_SRF) ;

  smos_sprintf(sBuff, _T("\n   Type & Tol     : GapType             :[%s-0x%p/0x%p]\n                  : XSectTol3d          :[%16.16lf]"),
               m_eSampleType == SM_GS_CRV_PT  ? _T("from_CRV_to_PT ") 
             : m_eSampleType == SM_GS_CRV_CRV ? _T("from_CRV_to_CRV")
             : m_eSampleType == SM_GS_CRV_SRF ? _T("from_CRV_to_SRF")
             : m_eSampleType == SM_GS_SRF_PT  ? _T("from_SRF_to_PT ")
             : m_eSampleType == SM_GS_SRF_CRV ? _T("from_SRF_to_CRV")
             : m_eSampleType == SM_GS_SRF_SRF ? _T("from_SRF_to_SRF") : _T("UNKNOWN"),
             GetThisObject(), GetOtherObject(),
             m_sXSectTol3d.val) ;
  smos_WriteBuffer(sBuff);

  // SmpIvlU - for curves and surfaces
  smos_sprintf(sBuff, _T("\n                  : SmpIvlU             :[%16.16lf %16.16lf]"),
             m_sSampleIvlU.GetMin(), m_sSampleIvlU.GetMax()) ;
  smos_WriteBuffer(sBuff);

  // SmpIvlV - only for surfaces
  if(bThisSrf)
    {
      smos_sprintf(sBuff, _T("\n                  : SmpIvlV             :[%16.16lf %16.16lf]"),
                 m_sSampleIvlV.GetMin(), m_sSampleIvlV.GetMax()) ;
      smos_WriteBuffer(sBuff);
    }

  // Lazy Eval state: Samples set
  smos_sprintf(sBuff, _T("\n   Lazy Eval State: SamplesSet          :[%s]"),
               m_bAreSamplesSet == TRUE   ? _T("TRUE = All Samples Evaluated") 
             : m_bAreSamplesSet == FALSE  ? _T("FALSE = No Samples Evaluated, Vals Not Yet Available") 
             : m_bAreSamplesSet == UNSURE ? _T("UNSURE = Some Samples Computed until first out-of-tol gap was found") 
             : _T("UNKNOWN")) ;
  smos_WriteBuffer(sBuff);

  // Lazy Eval state: LocalNeighborhood - only for FromGeometry == Curve types
  if(bThisCrv)
    {
      smos_sprintf(sBuff, _T("\n                  : LocalNeighborhoodSet:[%s]"),
                 m_bIsLocalNeighborhoodSet ? _T("TRUE = Neighborhood Ivl Evaluated") : _T("FALSE = Neighborhood Ivl Not Evaluated, Ivl Not Yet Available")) ;
      smos_WriteBuffer(sBuff);
    }

  // Sample Size
  smos_sprintf(sBuff, _T("\n   Sample Size    : SmpUCnt             :[%lu]"),
             m_lSampleCntU) ;
  smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff, _T("\n                  : SmpVCnt             :[%lu]"),
             m_lSampleCntV) ;
  smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff, _T("\n                  : DoneSmpGapCnt       :[%lu]"),
             m_sSampleGaps.GetSize()) ;
  smos_WriteBuffer(sBuff);

  // When LazyEval Gaps have been evaluated 
  if(m_bAreSamplesSet == TRUE)
    {                                  
      smos_sprintf(sBuff, _T("\n   Sample Summary : AllNormalGaps       :[%s]"),
                 m_bAllNormalGaps ? _T("TRUE") : _T("FALSE = Some Gaps limited by OtherGeom boundary")) ;
      smos_WriteBuffer(sBuff);

      // SmpIvlU - for curves and surfaces
      smos_sprintf(sBuff, _T("\n                  : %s ThisIvlU        :[%16.16lf %16.16lf]"),
                 bThisCrv ? _T("Crv") : _T("Srf"), m_sSampleIvlU.GetMin(), m_sSampleIvlU.GetMax()) ;
      smos_WriteBuffer(sBuff);

      // SmpIvlV - only for surfaces
      if(bThisSrf)
        {
          smos_sprintf(sBuff, _T("\n                  : Srf ThisIvlV        :[%16.16lf %16.16lf]"),
                     m_sSampleIvlV.GetMin(), m_sSampleIvlV.GetMax()) ;
          smos_WriteBuffer(sBuff);
        }

      // OtherIvlU - only for other curves and surfaces
      if(bOtherCrv || bOtherSrf)
        {
          smos_sprintf(sBuff, _T("\n                  : %s OtherIvlU       :[%16.16lf %16.16lf]"),
                     bOtherCrv ? _T("Crv") : _T("Srf"), m_sOtherIvlU.GetMin(), m_sOtherIvlU.GetMax()) ;
          smos_WriteBuffer(sBuff);
        }

      // OtherIvlV - only for other surfaces
      if(bOtherSrf)
        {
          smos_sprintf(sBuff, _T("\n                  : Srf OtherIvlV       :[%16.16lf %16.16lf]"),
                     m_sOtherIvlV.GetMin(), m_sOtherIvlV.GetMax()) ;
          smos_WriteBuffer(sBuff);
        }

      // LocalNeighborhood - only for FromGeometry == Curve types
      if(   m_bIsLocalNeighborhoodSet == TRUE
         && bThisCrv)
        {
          const SmCurveClassification * pLocalNeighborhood = GetAsIsLocalNeighborhood() ;
          if(pLocalNeighborhood)
            { 
              smos_WriteBuffer(_T("\n    Begin LocalNeighborhood Dump"));
              pLocalNeighborhood->Dump() ; 
              smos_WriteBuffer(_T("\n    End LocalNeighborhood Dump"));
            } // end LocalNeighborhood existence check
        } // end FromGeom == Curve Check

      // Gap reports - header
      smos_sprintf(sBuff, _T("\n   Begin SampleGap Array Dump - Cnt:[%lu], MinSampleGap:[%16.16lf], MaxSampleGap:%16.16lf]"),
             m_sSampleGaps.GetSize(),
             m_sSampleGaps.GetDataArray()[m_lMinSampleIndex].GetLength(),
             m_sSampleGaps.GetDataArray()[m_lMaxSampleIndex].GetLength()) ;
      smos_WriteBuffer(sBuff);

      // for every gap
      for(ii=0;ii<GetSize();ii++)
        {
          const SmGapSample &rGap = m_sSampleGaps.GetAt(ii) ;
          rGap.Dump(m_eSampleType, bAbbrev, &ii) ;

        } // end iter every gap
      smos_WriteBuffer(_T("\n   End SampleGap Array Dump")) ;

    } // end When LazyEval Gaps have been evaluated check

  // all done
  smos_sprintf(sBuff, _T("%s"), _T("\n  End Base SmGapFunction::Dump()")) ;
  smos_WriteBuffer(sBuff);

} // end SmGapFunction::Dump

/*******************************************************************//**
END SmGapFunction Methods
***********************************************************************/

/*******************************************************************//**
BEGIN SmGapFunction Derived Class Methods
***********************************************************************/

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmCrvPtGapFunction::IsKindOf( SM_TYPE t ) const
{
  return ((SmCrvPtGapFunction_TYPE == t) ? TRUE : SmGapFunction::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print SmCrvPtGapFunction data

NOTES: SmDerivedGapFunction::Dump() - outputs a Derived class label
        then calls Base SmGapFunction::Dump()
***********************************************************************/
void SmCrvPtGapFunction::Dump()                  const { SmCrvPtGapFunction::Dump(FALSE) ; }
void SmCrvPtGapFunction::Dump(SmBoolean bAbbrev) const 
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  smos_sprintf(sBuff, _T("%s"), _T("\nBegin SmCrvPtGapFunction::Dump()")) ;
  smos_WriteBuffer(sBuff);

  // Derived class label - GapFunction[ptr] from Curve[ptr] to Point[x y z]
  smos_sprintf(sBuff,        _T("\n GapFunction[0x%p] from Curve[0x%p] To Point[%16.16lf %16.16lf %16.16lf]"),
             this, 
             m_pThisCurve, 
             m_sOtherPoint.x, m_sOtherPoint.y, m_sOtherPoint.z) ;
  smos_sprintf(sBuffForFile, _T("\n GapFunction[%s] from Curve[%s] To Point[%16.16lf %16.16lf %16.16lf]"),
             _T("notNULL"), 
             m_pThisCurve ? _T("notNULL") : _T("NULL"), 
             m_sOtherPoint.x, m_sOtherPoint.y, m_sOtherPoint.z) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // Pass the call along to base class
  SmGapFunction::Dump(bAbbrev) ;

  smos_sprintf(sBuff, _T("%s"), _T("\nEnd   SmCrvPtGapFunction::Dump()")) ;
  smos_WriteBuffer(sBuff);

} // end SmCrvPtGapFunction::Dump

/*******************************************************************//**
PURPOSE: Draw SmCrvPtGapFunction graphics

NOTES:
***********************************************************************/
SmDisplayList * SmCrvPtGapFunction::Draw
 (SmBoolean       bShowGeometry,       // in : TRUE = Show Geometry bounding the gaps, default:[TRUE]
  SmBoolean       bShowNeighborhoods,  // in : TRUE = Show InTol GapFunction neighborhood intervals, default:[TRUE]
  SmBoolean       bShowSamples,        // in : TRUE  = show all sample gaps, 
                                       //      UNSURE= Only show bigger-than-tol gaps, 
                                       //      FALSE = Show no sample gaps,
                                       //      default:[FALSE]
  double          dDisplayScale,       // in : Amount micro geometry is scaled for visualization, default:[1000]
  SmGfxArraySet * pOptGfxSet)          // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
 const                                 //      NULL to ignore. default:[NULL]
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start new displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);
  
  // draw From Geometry
  if(bShowGeometry) 
    { m_pThisCurve->Draw(NULL, TRUE, NULL, pOptGfxSet) ; }

  // draw To Geometry
  if(bShowGeometry) 
    { m_sOtherPoint.Draw(NULL, NULL, pOptGfxSet) ; }

  // draw GapFunction
  SmGapFunction::Draw( bShowGeometry, bShowNeighborhoods, bShowSamples, dDisplayScale, pOptGfxSet) ;

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF5(bShowGeometry, bShowNeighborhoods, bShowSamples, dDisplayScale, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmCrvPtGapFunction::Draw

/*******************************************************************//**
PURPOSE: Evaluate SmCrvPtGapFunction at specified location

NOTES:
***********************************************************************/
SmStatus SmCrvPtGapFunction::Evaluate
 (double       dThisU,            // in : This Geom 1st param used for Curves and Surfaces
  double       dThisV,            // NotUsed: in : This Geom 2nd param only used for Surfaces
  SmGapSample &rGapSample,        // out: Eval holder block
  double      *pOptOtherGuessU,   // NotUsed: in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
  double      *pOptOtherGuessV)   // NotUsed: in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
{
  SM_REF3(dThisV, pOptOtherGuessU, pOptOtherGuessV) ;
  // Curve Eval
  SmPoint3d sThisPos ;
  m_pThisCurve->EvaluatePoint(dThisU, sThisPos) ;
  
  // set output
  rGapSample.SetCanonical(sThisPos, dThisU, 0.0,
                          m_sOtherPoint, 0.0, 0.0,
                          SM_GD_NORMAL, m_sXSectTol3d) ;      
  // all done
  return(SM_SUCCESS) ;

} // end SmCrvPtGapFunction::Evaluate

/*******************************************************************//**
PURPOSE: Evaluate SmCrvPtGapFunction Local Neighborhood boundary points

NOTES:
***********************************************************************/
SmStatus SmCrvPtGapFunction::EvaluateNeighborhoodBoundaries
  (SmSolutionArray &rSolutions) 
{ 
  // init output
  rSolutions.ReSet() ; 

  // try GlobalPropertyAnalysis (SM_SO_AT_DISTANCE)
  m_pThisCurve->GlobalPointSolve     
     ( m_pThisCurve->GetNaturalInterval(), // in : search curve interval                                             
       SM_SO_AT_DISTANCE,                  // in : which solver operation to perform                                
       m_sOtherPoint,                      // in : Euclidean target point                                           
       SM_ZONE_TOL_3D,                     // in : Basically the distance tolerance sets up a range for the          
                                           //      distance measurements where additional answers may exist.         
                                           //      For example if the distance between two local minima/maxima       
                                           //      is less than this tolerance, both answers will be returned.       
      (double*)&m_sXSectTol3d,             // in : If not NULL it will be:                                           
                                           //        SM_SO_AT_DISTANCE       = target distance,                      
                                           //        SM_SO_MINIMIZE/MAXIMIZE = distance limit.                       
                                           //      For example, to find a minimum value only if it less than         
                                           //      the target distance or maximum value only if it is greater than   
                                           //      the target distance                                               
       NULL,                               // in : Vectors used in some of the solvers.                              
                                           //      SM_SO_RAYFIRE,                  SM_SO_3D_SIGNED_DIRECTED_MINIMIZE
                                           //      SM_SO_SIGNED_DIRECTED_MINIMIZE, SM_SO_DIRECTED_MINIMIZE          
                                           //      SM_SO_DIRECTED_MAXIMIZE,        SM_SO_PROJECTED_MINIMIZE         
                                           //      SM_SO_PROJECTED_MAXIMIZE                                          
       SM_SR_ALL,                          // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions                                                                            
       rSolutions) ;                       // out: array of problem solutions reported as curve parameter values                                                               
  
  // all done
  return(SM_SUCCESS) ; 

} // end SmCrvPtGapFunction::EvaluateNeighborhoodBoundaries

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmCrvCrvGapFunction::IsKindOf( SM_TYPE t ) const
{
  return ((SmCrvCrvGapFunction_TYPE == t) ? TRUE : SmGapFunction::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print SmCrvCrvGapFunction data

NOTES: SmDerivedGapFunction::Dump() - outputs a Derived class label
        then calls Base SmGapFunction::Dump()
***********************************************************************/
void SmCrvCrvGapFunction::Dump()                   const { SmCrvCrvGapFunction::Dump(FALSE) ; }
void SmCrvCrvGapFunction::Dump (SmBoolean bAbbrev) const 
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  smos_sprintf(sBuff, _T("%s"), _T("\nBegin SmCrvCrvGapFunction::Dump()")) ;
  smos_WriteBuffer(sBuff);

  // Derived class label - GapFunction[ptr] from Curve[ptr] to Curve[ptr]
  smos_sprintf(sBuff,        _T("\n GapFunction[0x%p] from Curve[0x%p] To Curve[0x%p]"),
             this, m_pThisCurve, m_pOtherCurve) ;
  smos_sprintf(sBuffForFile, _T("\n GapFunction[%s] from Curve[%s] To Curve[%s]"),
             _T("notNULL"), 
             m_pThisCurve ? _T("notNULL") : _T("NULL"), 
             m_pOtherCurve   ? _T("notNULL") : _T("NULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // Pass the call along to base class
  SmGapFunction::Dump(bAbbrev) ;

  smos_sprintf(sBuff, _T("%s"), _T("\nEnd   SmCrvCrvGapFunction::Dump()")) ;
  smos_WriteBuffer(sBuff);

} // end SmCrvCrvGapFunction::Dump

/*******************************************************************//**
PURPOSE: Draw SmCrvCrvGapFunction graphics

NOTES:
***********************************************************************/
SmDisplayList * SmCrvCrvGapFunction::Draw
 (SmBoolean       bShowGeometry,       // in : TRUE = Show Geometry bounding the gaps, default:[TRUE]
  SmBoolean       bShowNeighborhoods,  // in : TRUE = Show InTol GapFunction neighborhood intervals, default:[TRUE]
  SmBoolean       bShowSamples,        // in : TRUE  = show all sample gaps, 
                                       //      UNSURE= Only show bigger-than-tol gaps, 
                                       //      FALSE = Show no sample gaps,
                                       //      default:[FALSE]
  double          dDisplayScale,       // in : Amount micro geometry is scaled for visualization, default:[1000]
  SmGfxArraySet * pOptGfxSet)          // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
 const                                 //      NULL to ignore. default:[NULL]
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start new displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);
  
  // draw From Geometry
  if(bShowGeometry) 
    { m_pThisCurve->Draw(NULL, TRUE, NULL, pOptGfxSet) ; }

  // draw To Geometry
  SmVector3d sColor = smgfx_OutputColor( smgfx_GetOutTolGapColor(), pOptGfxSet) ;
  if(bShowGeometry) 
    { m_pOtherCurve->Draw(NULL, TRUE, NULL, pOptGfxSet) ; }
  smgfx_OutputColor( sColor, pOptGfxSet) ;

  // draw GapFunction
  SmGapFunction::Draw( bShowGeometry, bShowNeighborhoods, bShowSamples, dDisplayScale, pOptGfxSet) ;

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF5(bShowGeometry, bShowNeighborhoods, bShowSamples, dDisplayScale, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmCrvCrvGapFunction::Draw

/*******************************************************************//**
PURPOSE: constructor

NOTES:
***********************************************************************/
SmCrvCrvGapFunction::SmCrvCrvGapFunction
(
  const SmXSectTol3d  & rXSectTol3d, // in : max allowed size for a within TOl gap
  const SmCurve       * pThisCurve,  // in : Gap base curve
  SmExtent1d          & rThisIvl,    // in : Gap base curve interval
  const SmCurve       * pOtherCurve, // in : Gap Target Curve
  ULONG                 lSampleCntU  // in : Number of samples to approximate GapFunctcion, default:[20]
)
 : SmGapFunction(SM_GS_CRV_CRV,      // in : Type so derived classes can mark their type here - yuk!
                 rXSectTol3d,        // in : XSectTol3d used to judge in/out property for a gap length.
                 rThisIvl,           // in : 1st param domain range - used for Curves and Surfaces
                 rThisIvl,           // in : 2nd param domain range - only used for Surfaces
                 lSampleCntU,        // in : number of evenly spaced samples in 1st param dim - used for Curves and Surfaces
                 1),                 // in : number of evenly spaced samples in 2nd param dim - only used for Surfaces      
   m_pThisCurve(pThisCurve),
   m_pOtherCurve(pOtherCurve),
   // gwc: passing a XSectTol as a ZoneTol3d - needs fix
   m_sLocalNeighborhood(pThisCurve, rThisIvl, NULL, (SmZoneTol3d &)rXSectTol3d)
{  
   m_sSampleIvlV.SetMinMax(0.0,1.0) ; 

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SM_DUMP_AND_ASSERT_VALID(pThisCurve) ;
      SM_DUMP_AND_ASSERT_VALID(pOtherCurve) ;

      smgfx_Erase() ;
      smgfx_SetLook(2,4, 0,0,1) ; if(pThisCurve) pThisCurve->DrawParams(&rThisIvl) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,4, 0,0,1) ; if(pOtherCurve) pOtherCurve->DrawParams() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE
 
} // SmCrvCrvGapFunction::SmCrvCrvGapFunction constructor

/*******************************************************************//**
PURPOSE: Evaluate SmCrvCrvGapFunction at specified location

NOTES:
***********************************************************************/
SmStatus SmCrvCrvGapFunction::Evaluate
 (double       dThisU,            // in : This Geom 1st param used for Curves and Surfaces
  double       dThisV,            // NotUsed: in : This Geom 2nd param only used for Surfaces
  SmGapSample &rGapSample,        // out: Eval holder block
  double      *pOptOtherGuessU,   // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
  double      *pOptOtherGuessV)   // NotUsed: in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
{
  SM_REF2(dThisV, pOptOtherGuessV) ;
  // This Curve Eval
  SmPoint3d sThisPos ;
  m_pThisCurve->EvaluatePoint(dThisU, sThisPos) ;

  // Drop point to ToGeometry
  double dOtherU ;
  double dXSectTol3d = this->GetTol(); // Distance between close solutions, not the drop distance [B448]
  double dDistDropped ;
  SmBoolean bSuccess ;
  SmExtent1d sOtherDomain = m_pOtherCurve->GetNaturalInterval() ;
  m_pOtherCurve->DropPoint(sOtherDomain,       // in : target curve allowed domain   
                           sThisPos,           // in : Point to drop to curve   
                           NULL,               // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                               //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                               //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                           dXSectTol3d,        // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                               //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                               //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                               //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                           pOptOtherGuessU,    // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()   
                           bSuccess,           // out: TRUE = found a drop point   
                           dOtherU,            // out: found drop curve param   
                           dDistDropped,       // out: found drop distance   
                           SM_SO_MINIMIZE) ;   // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                               //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                               //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                               //      default:[SM_SO_MINIMIZE] to preserve original behavior

  // If we don't like the result of the drop with a guess,
  // try without a guess (global solve).
  // We do this if:
  // - We did use a guess the first time (if not, then nothing is different), AND
  //   - bSuccess indicates failure OR
  //   - gap is really big (local solve converged to wrong solution) [B279] OR
  //     - Gap is bigger than tol AND
  //     - result is on other curve's boundary.
  SmBoolean bTryGlobal = (    pOptOtherGuessU != NULL
                           && (    bSuccess == FALSE
                                || dDistDropped > m_sXSectTol3d*100
                                || (  dDistDropped > m_sXSectTol3d
                                    && sOtherDomain.IsValueOnBoundary( dOtherU )
                                   )
                              )
                         );

  if ( bTryGlobal )
    {
      // try a global drop point solution
      m_pOtherCurve->DropPoint(sOtherDomain,      // in : target curve allowed domain  
                               sThisPos,          // in : Point to drop to curve 
                               NULL,              // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                  //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                  //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                               dXSectTol3d,       // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                  //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                  //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                  //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                               NULL,              // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve() 
                               bSuccess,          // out: TRUE = found a drop point 
                               dOtherU,           // out: found drop curve param 
                               dDistDropped,      // out: found drop distance 
                               SM_SO_MINIMIZE) ;  // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                  //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                  //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                  //      default:[SM_SO_MINIMIZE] to preserve original behavior
                                                        
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  // draw 
  if(bDebugMe)
    {
      SmPoint3d sToPoint ;
      m_pOtherCurve->EvaluatePoint(dOtherU, sToPoint) ;
      SmEdge *pFromEdge = (SmEdge *)m_pThisCurve->GetEdge() ;
      SmEdge *pToEdge   = (SmEdge *)m_pOtherCurve->GetEdge() ;
      SmBrep *pFromBrep = pFromEdge ? pFromEdge->GetBrep() : NULL ;
      SmBrep *pToBrep   = pToEdge ? pToEdge->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pFromBrep) pFromBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pToBrep)   pToBrep->Draw(TRUE) ;   sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; if(m_pThisCurve) m_pThisCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(m_pOtherCurve)   m_pOtherCurve->Draw() ;   sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pFromEdge) pFromEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pToEdge)   pToEdge->Draw() ;   sm_GraphicsLoop() ;
      smgfx_SetLook(7,8, 1,0,0) ; sThisPos.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(9,10,1,.5,0) ; sToPoint.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // set output
  if(bSuccess)
    {
      // Other Curve dropped point Eval
      SmPoint3d sOtherPD[2] ;
      m_pOtherCurve->Evaluate(dOtherU, 1, TRUE, sOtherPD, TRUE) ; // NonZeroTangents
      SmVector3d sGap = sOtherPD[0]-sThisPos ;
      SmGapDropType eGapDropType =    ( !sOtherDomain.IsValueOnBoundary(dOtherU) ) 
                                   || (  sGap.LengthSquared() < SM_EFF_ZERO_SQ )
                                   || (  sOtherPD[1].IsPerpendicularTo(sGap, 0.5) ) 
                                   ? SM_GD_NORMAL
                                   : SM_GD_BOUNDARY ;

      rGapSample.SetCanonical(sThisPos,    dThisU,  0.0,
                              sOtherPD[0], dOtherU, 0.0,
                              eGapDropType, m_sXSectTol3d) ;      
    }
  else // failed to drop - load a zero length gap vector
    {
      rGapSample.SetCanonical(sThisPos, dThisU, 0.0,
                              sThisPos, m_pOtherCurve->GetNaturalInterval().GetMin(), 0.0,
                              SM_GD_NO_DROP, m_sXSectTol3d) ;      
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmCrvCrvGapFunction::Evaluate

/*******************************************************************//**
PURPOSE: Evaluate SmCrvCrvGapFunction Local Neighborhood boundary points

NOTES:
***********************************************************************/
SmStatus SmCrvCrvGapFunction::EvaluateNeighborhoodBoundaries
  (SmSolutionArray &rSolutions) 
{ 
  // init output
  rSolutions.ReSet() ; 

  // try GlobalPropertyAnalysis (SM_SO_AT_DISTANCE)
  m_pThisCurve->GlobalCurveSolve     
     (*GetThisIvlU(),                       // in : Interval on curve where solution will be searched for            
      *m_pOtherCurve,                       // in : target other curve                                               
       m_pOtherCurve->GetNaturalInterval(), // in : Interval on other curve where solution will be searched for      
       SM_SO_AT_DISTANCE,                   // in : Which solver operation to perform                                
       SM_ZONE_TOL_3D,                      // in : Basically the distance tolerance sets up a range for the         
                                            //      distance measurements where additional answers may exist.        
                                            //      For example if the distance between two local minima/maxima      
                                            //      is less than this tolerance, both answers will be returned.      
      (double*)&m_sXSectTol3d,              // in : If not NULL it will either be the target distance for the        
                                            //      SM_SO_AT_DISTANCE operation, or it will be the corresponding     
                                            //      limit to a minimize/maximize operation.  In otherwords, it will  
                                            //      ask the solver to find a minimum value only if it is less than   
                                            //      the target distance or maximum value only if it is greater than  
                                            //      the target distance                                              
       NULL,                                // in : See SmSolverOperationType documentation for corresponding meaning
                                            //      of these vectors.                                                
       SM_SR_ALL,                           // in : oneof SM_SR_SINGLE, SM_SR_ALL, SM_SR_FIND_AMBIGUITIES, SM_SR_NODES                                                                     
       rSolutions) ;                        // out: 
                                                                           
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      rSolutions.Dump() ;

      SmEdge *pFromEdge = (SmEdge *)m_pThisCurve->GetEdge() ;
      SmEdge *pToEdge   = (SmEdge *)m_pOtherCurve->GetEdge() ;
      SmBrep *pFromBrep = pFromEdge ? pFromEdge->GetBrep() : NULL ;
      SmBrep *pToBrep   = pToEdge ? pToEdge->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pFromBrep) pFromBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pToBrep)   pToBrep->Draw(TRUE) ;   sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; if(m_pThisCurve) m_pThisCurve->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(m_pOtherCurve) m_pOtherCurve->Draw() ;   sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pFromEdge) pFromEdge->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pToEdge)   pToEdge->Draw() ;   sm_GraphicsLoop() ;
      smgfx_SetLook(7,8, 1,0,0) ; rSolutions.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE 
     
  // all done
  return(SM_SUCCESS) ; 

} // end SmCrvCrvGapFunction::EvaluateNeighborhoodBoundaries

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmCrvSrfGapFunction::IsKindOf( SM_TYPE t ) const
{
  return ((SmCrvSrfGapFunction_TYPE == t) ? TRUE : SmGapFunction::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print SmCrvSrfGapFunction data

NOTES: SmDerivedGapFunction::Dump() - outputs a Derived class label
        then calls Base SmGapFunction::Dump()
***********************************************************************/
void SmCrvSrfGapFunction::Dump()                  const { SmCrvSrfGapFunction::Dump(FALSE) ; }
void SmCrvSrfGapFunction::Dump(SmBoolean bAbbrev) const 
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  smos_sprintf(sBuff, _T("%s"), _T("\nBegin SmCrvSrfGapFunction::Dump()")) ;
  smos_WriteBuffer(sBuff);

// Derived class label - GapFunction[ptr] from Curve[ptr] to Surface[ptr]
  smos_sprintf(sBuff,        _T("\n GapFunction:[0x%p] from Curve:[0x%p] To Surface:[0x%p]"),
             this, m_pThisCurve, m_pOtherSurface) ;
  smos_sprintf(sBuffForFile, _T("\n GapFunction:[%s] from Curve:[%s] To Surface:[%s]"),
             _T("notNULL"), 
             m_pThisCurve ? _T("notNULL") : _T("NULL"), 
             m_pOtherSurface ? _T("notNULL") : _T("NULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // Pass the call along to base class
  SmGapFunction::Dump(bAbbrev) ;

  smos_sprintf(sBuff, _T("%s"), _T("\nEnd   SmCrvSrfGapFunction::Dump()")) ;
  smos_WriteBuffer(sBuff);

} // end SmCrvSrfGapFunction::Dump

/*******************************************************************//**
PURPOSE: Draw SmCrvSrfGapFunction graphics

NOTES:
***********************************************************************/
SmDisplayList * SmCrvSrfGapFunction::Draw
 (SmBoolean       bShowGeometry,       // in : TRUE = Show Geometry bounding the gaps, default:[TRUE]
  SmBoolean       bShowNeighborhoods,  // in : TRUE = Show InTol GapFunction neighborhood intervals, default:[TRUE]
  SmBoolean       bShowSamples,        // in : TRUE  = show all sample gaps, 
                                       //      UNSURE= Only show bigger-than-tol gaps, 
                                       //      FALSE = Show no sample gaps,
                                       //      default:[FALSE]
  double          dDisplayScale,       // in : Amount micro geometry is scaled for visualization, default:[1000]
  SmGfxArraySet * pOptGfxSet)          // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
 const                                 //      NULL to ignore. default:[NULL]
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE

  // start new displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);
  SmVector3d sColor = smgfx_GetOutputColor() ; 
  
  // when asked
  if(bShowGeometry) 
    { 
  // draw From Geometry
      m_pThisCurve->Draw(NULL, TRUE, NULL, pOptGfxSet) ; 
    
      sColor = smgfx_OutputColor( smgfx_GetInTolGapColor(), pOptGfxSet) ;

  // draw To Geometry
      m_pOtherSurface->Draw(TRUE, pOptGfxSet) ;
    }

  smgfx_OutputColor( sColor, pOptGfxSet) ;

  // draw GapFunction
  SmGapFunction::Draw( bShowGeometry, bShowNeighborhoods, bShowSamples, dDisplayScale, pOptGfxSet) ;

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF5(bShowGeometry, bShowNeighborhoods, bShowSamples, dDisplayScale, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmCrvSrfGapFunction::Draw

/*******************************************************************//**
PURPOSE: Evaluate SmCrvSrfGapFunction at specified location

NOTES:
***********************************************************************/
SmStatus SmCrvSrfGapFunction::Evaluate
 (double       dThisU,            // in : This Geom 1st param used for Curves and Surfaces
  double       dThisV,            // NotUsed: in : This Geom 2nd param only used for Surfaces
  SmGapSample &rGapSample,        // out: Eval holder block
  double      *pOptOtherGuessU,   // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
  double      *pOptOtherGuessV)   // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
{
  SM_REF1(dThisV) ;
  // This Curve Eval
  // This Curve Eval FromPt
  SmPoint3d sThisPos ;
  m_pThisCurve->EvaluatePoint(dThisU, sThisPos) ;

  // Drop point to ToGeometry
  SmPoint2d *pOtherGuessUV = NULL, sOtherGuessUV ;
  if(pOptOtherGuessU && pOptOtherGuessV)
    {
      sOtherGuessUV.Set(*pOptOtherGuessU, *pOptOtherGuessV) ;
      pOtherGuessUV = &sOtherGuessUV ;
    }

  SmPoint2d sOtherUV ;
  double    dDistDropped = 0.0;
  SmBoolean bIsMulti;
  SmBoolean bSuccess ;
  SmExtent2d sOtherDomain = m_pOtherSurface->GetNaturalUVDomain();
  m_pOtherSurface->DropPoint(sThisPos,      // in : target point to drop
                             sOtherDomain,  // in : target surface domain
                             pOtherGuessUV, // in : When given, uses only local solves
                             bSuccess,      // out: TRUE=Point dropped successfully, FALSE=didn't
                             sOtherUV,      // out: drop result UVPoint
                             dDistDropped,
                             bIsMulti); // out: distance of found point to Pt to drop

  // If we don't like the result of the drop with a guess,
  // try without a guess (global solve).
  // We do this if:
  // - We did use a guess the first time (if not, then nothing is different), AND
  //   - bSuccess indicates failure OR
  //   - gap is really big (local solve converged to wrong solution) [B279] OR
  //     - Gap is bigger than tol AND
  //     - result is on other curve's boundary.
  SmBoolean bTryGlobal = (    ( pOptOtherGuessU != NULL && pOptOtherGuessV != NULL )
                           && (    bSuccess == FALSE
                                || dDistDropped > m_sXSectTol3d*100
                                || (  dDistDropped > m_sXSectTol3d
                                    && sOtherDomain.IsPoint2dOnBoundary( sOtherUV )
                                   )
                              )
                         );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      Dump() ;

      const SmCurve   *pFromCurve   = GetThisCurve() ;
      const SmSurface *pFromSurface = GetThisSurface() ;
      const SmPoint3d *pToPoint     = GetOtherPoint() ;
      const SmCurve   *pToCurve     = GetOtherCurve() ;
      const SmSurface *pToSurface   = GetOtherSurface() ;

      SmEdge *pFromEdge = pFromCurve   ? (SmEdge *)pFromCurve->GetEdge()   : NULL ;
      SmFace *pFromFace = pFromSurface ? (SmFace *)pFromSurface->GetFace() : NULL ;
      SmEdge *pToEdge   = pToCurve     ? (SmEdge *)pToCurve->GetEdge() : NULL ;
      SmFace *pToFace   = pToSurface   ? (SmFace *)pToSurface->GetFace() : NULL ;
      SmBrep *pFromBrep = pFromEdge ? pFromEdge->GetBrep() : pFromFace ? pFromFace->GetBrep() : NULL ;
      SmBrep *pToBrep   = pToEdge ? pToEdge->GetBrep() : pToFace ? pToFace->GetBrep() : NULL ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pFromBrep) pFromBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pToBrep)   pToBrep->Draw(TRUE) ;   sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,1,1) ; if(pFromCurve)   pFromCurve->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pFromSurface) pFromSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pToPoint)   pToPoint->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,0,1) ; if(pToCurve)   pToCurve->Draw() ;     sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1) ; if(pToSurface) pToSurface->DrawUV() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pFromEdge) pFromEdge->Draw() ;                 sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pFromFace) pFromFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pToEdge) pToEdge->Draw() ;                 sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; if(pToFace) pToFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,8, 1,0,0) ; Draw(TRUE) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE    

  if ( bTryGlobal )
    {
      // try a global drop point solution
      m_pOtherSurface->DropPoint(sThisPos,      // in : target point to drop
                                 sOtherDomain,  // in : target surface domain
                                 NULL,          // in : Global solve this time: no guess
                                 bSuccess,      // out: TRUE=Point dropped successfully, FALSE=didn't
                                 sOtherUV,      // out: drop result UVPoint
                                 dDistDropped,
                                 bIsMulti); // out: distance of found point to Pt to drop
    }

  // set output
  if(bSuccess)
    {
      // Other Surface dropped point Eval
      SmPoint3d sOtherPD[2][2] ;
      m_pOtherSurface->Evaluate(sOtherUV, 1, 1, TRUE, TRUE, TRUE, sOtherPD[0], TRUE) ; // NonZeroTangents
      SmVector3d sGap = sOtherPD[0][0]-sThisPos ;
      SmGapDropType eGapDropType =    ( !sOtherDomain.IsPoint2dOnBoundary(sOtherUV) ) 
                                   || (  sGap.LengthSquared() < SM_EFF_ZERO_SQ )
                                   || (   sOtherPD[0][1].IsPerpendicularTo(sGap, 0.5) 
                                       && sOtherPD[1][0].IsPerpendicularTo(sGap, 0.5)) 
                                   ? SM_GD_NORMAL
                                   : SM_GD_BOUNDARY ;
      rGapSample.SetCanonical(sThisPos,       dThisU,     0.0,
                              sOtherPD[0][0], sOtherUV.x, sOtherUV.y,
                              eGapDropType, 
                              m_sXSectTol3d) ;      
    }
  else // failed to drop - load a zero length gap vector
    {
      rGapSample.SetCanonical(sThisPos, dThisU, 0.0,
                              sThisPos, 
                              m_pOtherSurface->GetNaturalUVDomain().GetMin().x, 
                              m_pOtherSurface->GetNaturalUVDomain().GetMin().y,
                              SM_GD_NO_DROP, 
                              m_sXSectTol3d) ;      
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmCrvSrfGapFunction::Evaluate

/*******************************************************************//**
PURPOSE: Evaluate SmCrvSrfGapFunction Local Neighborhood boundary points

NOTES:
***********************************************************************/
SmStatus SmCrvSrfGapFunction::EvaluateNeighborhoodBoundaries
  (SmSolutionArray &rSolutions) 
{ 
  // init output
  rSolutions.ReSet() ; 

  // try GlobalPropertyAnalysis (SM_SO_AT_DISTANCE)
  SE_MSG(SM_ERR, _T("SmCrvSrfGapFunction can't yet EvaluateNeighborhoodBoundaries because SmSurface::GlobalCurveSolve does not support SM_SO_AT_DISTANCE")) ;
  return(SM_SUCCESS) ;

  //      // not yet supported
  //      m_pOtherSurface->GlobalCurveSolve     
  //         ( m_pOtherSurface->GetNaturalUVDomain(), // in : Domain of the surface to be used in solve                        
  //          *m_pThisCurve,                          // in : Curve to solve with                                              
  //          *GetThisIvlU(),                         // in : Curve interval to use in solve                                   
  //           SM_SO_AT_DISTANCE,                     // in :                                                                  
  //           SM_ZONE_TOL_3D,                        // in : 3D distance tolerance to be used for solve                       
  //          &m_sXSectTol3d,                         // in : The cpdOptTargetDistance, if not NULL, will be the corresponding 
  //                                                  //      limit to a minimize/maximize operations.  In otherwords, it will 
  //                                                  //      ask the solver to find a minimum value only if it is less than   
  //                                                  //      the target distance or maximum value only if it is greater than  
  //                                                  //      the target distance.                                             
  //           NULL,                                  // in : See SmSolverOperationType documentation for corresponding meaning
  //                                                  //      of these vectors.                                                
  //           SM_SR_ALL,                             // in : What kind of a solution do you want                                                                                            
  //           rSolutions) ;                          // out: The output of the solve                                                                                             
  //      
  //      // all done
  //      return(SM_SUCCESS) ; 

} // end SmCrvSrfGapFunction::EvaluateNeighborhoodBoundaries

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSrfPtGapFunction::IsKindOf( SM_TYPE t ) const
{
  return ((SmSrfPtGapFunction_TYPE == t) ? TRUE : SmGapFunction::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print SmSrfPtGapFunction data

NOTES: SmDerivedGapFunction::Dump() - outputs a Derived class label
        then calls Base SmGapFunction::Dump()
***********************************************************************/
void SmSrfPtGapFunction::Dump()                  const { SmSrfPtGapFunction::Dump(FALSE) ; }
void SmSrfPtGapFunction::Dump(SmBoolean bAbbrev) const 
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  smos_sprintf(sBuff, _T("%s"),_T("\nBegin SmSrfPtGapFunction::Dump()")) ;
  smos_WriteBuffer(sBuff);

  // Derived class label - GapFunction[ptr] from Surface[ptr] to Point[x y z]
  smos_sprintf(sBuff,        _T("\n GapFunction[0x%p] from Surface[0x%p] To Point[%16.16lf %16.16lf %16.16lf]"),
             this, 
             m_pThisSurface, 
             m_sOtherPoint.x, m_sOtherPoint.y, m_sOtherPoint.z) ;
  smos_sprintf(sBuffForFile, _T("\n GapFunction[%s] from Surface[%s] To Point[%16.16lf %16.16lf %16.16lf]"),
             _T("notNULL"), 
             m_pThisSurface ? _T("notNULL") : _T("NULL"), 
             m_sOtherPoint.x, m_sOtherPoint.y, m_sOtherPoint.z) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // Pass the call along to base class
  SmGapFunction::Dump(bAbbrev) ;

  smos_sprintf(sBuff, _T("%s"),_T("\nEnd   SmSrfPtGapFunction::Dump()")) ;
  smos_WriteBuffer(sBuff);

} // end SmSrfPtGapFunction::Dump

/*******************************************************************//**
PURPOSE: Draw SmSrfPtGapFunction graphics

NOTES:
***********************************************************************/
SmDisplayList * SmSrfPtGapFunction::Draw
 (SmBoolean       bShowGeometry,       // in : TRUE = Show Geometry bounding the gaps, default:[TRUE]
  SmBoolean       bShowNeighborhoods,  // in : TRUE = Show InTol GapFunction neighborhood intervals, default:[TRUE]
  SmBoolean       bShowSamples,        // in : TRUE  = show all sample gaps, 
                                       //      UNSURE= Only show bigger-than-tol gaps, 
                                       //      FALSE = Show no sample gaps,
                                       //      default:[FALSE]
  double          dDisplayScale,       // in : Amount micro geometry is scaled for visualization, default:[1000]
  SmGfxArraySet * pOptGfxSet)          // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
 const                                 //      NULL to ignore. default:[NULL]
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start new displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);
  
  // draw From Geometry
  if(bShowGeometry) 
    { m_pThisSurface->Draw(TRUE, pOptGfxSet) ; }

  // draw To Geometry
  if(bShowGeometry) 
    { m_sOtherPoint.Draw(NULL, NULL, pOptGfxSet) ; }

  // draw GapFunction
  SmGapFunction::Draw( bShowGeometry, bShowNeighborhoods, bShowSamples, dDisplayScale, pOptGfxSet) ;

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF5(bShowGeometry, bShowNeighborhoods, bShowSamples, dDisplayScale, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmSrfPtGapFunction::Draw

/*******************************************************************//**
PURPOSE: Evaluate SmSrfPtGapFunction at specified location

NOTES:
***********************************************************************/
SmStatus SmSrfPtGapFunction::Evaluate
 (double       dThisU,            // in : This Geom 1st param used for Curves and Surfaces
  double       dThisV,            // in : This Geom 2nd param only used for Surfaces
  SmGapSample &rGapSample,        // out: Eval holder block
  double      *pOptOtherGuessU,   // in : NotUsed: When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
  double      *pOptOtherGuessV)   // in : NotUsed: When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
{
  SM_REF2(pOptOtherGuessU, pOptOtherGuessV) ;
  // This Curve Eval
  // Curve Eval
  SmPoint3d sThisPos ;
  SmPoint2d sThisUV(dThisU, dThisV) ;
  m_pThisSurface->EvaluatePoint(sThisUV, sThisPos) ;
  
  // set output
  rGapSample.SetCanonical(sThisPos, dThisU, dThisV,
                          m_sOtherPoint, 0.0, 0.0,
                          SM_GD_NORMAL, m_sXSectTol3d) ;      
  // all done
  return(SM_SUCCESS) ;

} // end SmSrfPtGapFunction::Evaluate

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSrfCrvGapFunction::IsKindOf( SM_TYPE t ) const
{
  return ((SmSrfCrvGapFunction_TYPE == t) ? TRUE : SmGapFunction::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print SmSrfCrvGapFunction data

NOTES: SmDerivedGapFunction::Dump() - outputs a Derived class label
        then calls Base SmGapFunction::Dump()
***********************************************************************/
void SmSrfCrvGapFunction::Dump()                  const { SmSrfCrvGapFunction::Dump(FALSE) ; }
void SmSrfCrvGapFunction::Dump(SmBoolean bAbbrev) const 
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  smos_sprintf(sBuff, _T("%s"),_T("\nBegin SmSrfCrvGapFunction::Dump()")) ;
  smos_WriteBuffer(sBuff);

  // Derived class label - GapFunction[ptr] from Surface[ptr] to Curve[ptr]
  smos_sprintf(sBuff,        _T("\n GapFunction[0x%p] from Surface[0x%p] To Curve[0x%p]"),
             this, m_pThisSurface, m_pOtherCurve) ;
  smos_sprintf(sBuffForFile, _T("\n GapFunction[%s] from Surface[%s] To Curve[%s]"),
             _T("notNULL"), 
             m_pThisSurface ? _T("notNULL") : _T("NULL"), 
             m_pOtherCurve   ? _T("notNULL") : _T("NULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // Pass the call along to base class
  SmGapFunction::Dump(bAbbrev) ;

  smos_sprintf(sBuff, _T("%s"),_T("\nEnd   SmSrfCrvGapFunction::Dump()")) ;
  smos_WriteBuffer(sBuff);

} // end SmSrfCrvGapFunction::Dump

/*******************************************************************//**
PURPOSE: Draw SmSrfCrvGapFunction graphics

NOTES:
***********************************************************************/
SmDisplayList * SmSrfCrvGapFunction::Draw
 (SmBoolean       bShowGeometry,       // in : TRUE = Show Geometry bounding the gaps, default:[TRUE]
  SmBoolean       bShowNeighborhoods,  // in : TRUE = Show InTol GapFunction neighborhood intervals, default:[TRUE]
  SmBoolean       bShowSamples,        // in : TRUE  = show all sample gaps, 
                                       //      UNSURE= Only show bigger-than-tol gaps, 
                                       //      FALSE = Show no sample gaps,
                                       //      default:[FALSE]
  double          dDisplayScale,       // in : Amount micro geometry is scaled for visualization, default:[1000]
  SmGfxArraySet * pOptGfxSet)          // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
 const                                 //      NULL to ignore. default:[NULL]
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start new displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);
  
  // draw From Geometry
  if(bShowGeometry)  
    { m_pThisSurface->Draw(TRUE, pOptGfxSet) ; }

  // draw To Geometry
  SmVector3d sColor = smgfx_OutputColor( smgfx_GetInTolGapColor() , pOptGfxSet) ;
  if(bShowGeometry) 
    { m_pOtherCurve->Draw(NULL, TRUE, NULL, pOptGfxSet) ; }
  smgfx_OutputColor( sColor , pOptGfxSet) ;

  // draw GapFunction
  SmGapFunction::Draw( bShowGeometry, bShowNeighborhoods, bShowSamples, dDisplayScale, pOptGfxSet) ;

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF5(bShowGeometry, bShowNeighborhoods, bShowSamples, dDisplayScale, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmSrfCrvGapFunction::Draw

/*******************************************************************//**
PURPOSE: Evaluate SmSrfCrvGapFunction at specified location

NOTES:
***********************************************************************/
SmStatus SmSrfCrvGapFunction::Evaluate
 (double       dThisU,            // in : This Geom 1st param used for Curves and Surfaces
  double       dThisV,            // in : This Geom 2nd param only used for Surfaces
  SmGapSample &rGapSample,        // out: Eval holder block
  double      *pOptOtherGuessU,   // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
  double      *pOptOtherGuessV)   // NotUsed: in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
{
  SM_REF1(pOptOtherGuessV) ;
  // This Curve Eval
  // This Curve Eval
  SmPoint3d sThisPos ;
  SmPoint2d sThisUV(dThisU, dThisV) ;
  m_pThisSurface->EvaluatePoint(sThisUV, sThisPos) ;

  // Drop point to ToGeometry
  double dOtherU ;
  double dDistanceTolerance = this->GetTol();  //cbi 448:    10.0;  // assume gap functions are near tolerance size - 10 is huge by comparison.
  double dDistDropped ;
  SmBoolean bSuccess ;
  SmExtent1d sOtherDomain = m_pOtherCurve->GetNaturalInterval();
  m_pOtherCurve->DropPoint(sOtherDomain,       // in : target curve allowed domain 
                           sThisPos,           // in : Point to drop to curve 
                           NULL,               // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                               //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                               //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                           dDistanceTolerance, // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance. 
                                               //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                               //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                               //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                           pOptOtherGuessU,    // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve() 
                           bSuccess,           // out: TRUE = found a drop point 
                           dOtherU,            // out: found drop curve param 
                           dDistDropped,       // out: found drop distance 
                           SM_SO_MINIMIZE) ;   // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                               //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                               //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                               //      default:[SM_SO_MINIMIZE] to preserve original behavior                             

  // If we don't like the result of the drop with a guess,
  // try without a guess (global solve).
  // We do this if:
  // - We did use a guess the first time (if not, then nothing is different), AND
  //   - bSuccess indicates failure OR
  //   - gap is really big (local solve converged to wrong solution) [B279] OR
  //     - Gap is bigger than tol AND
  //     - result is on other curve's boundary.
  SmBoolean bTryGlobal = (    pOptOtherGuessU != NULL
                           && (    bSuccess == FALSE
                                || dDistDropped > m_sXSectTol3d*100
                                || (  dDistDropped > m_sXSectTol3d
                                    && sOtherDomain.IsValueOnBoundary( dOtherU )
                                   )
                              )
                         );

  if ( bTryGlobal )
    {
      // try a global drop point solution
      m_pOtherCurve->DropPoint(sOtherDomain,        // in : target curve allowed domain
                               sThisPos,            // in : Point to drop to curve
                               NULL,                // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                    //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                    //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                               dDistanceTolerance,  // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                    //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                    //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                    //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                               NULL,                // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                               bSuccess,            // out: TRUE = found a drop point
                               dOtherU,             // out: found drop curve param
                               dDistDropped,        // out: found drop distance
                               SM_SO_MINIMIZE) ;    // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                    //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                    //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                    //      default:[SM_SO_MINIMIZE] to preserve original behavior
                                  
    }
  // set output
  if(bSuccess)
    {
      // Other Curve dropped point Eval
      SmPoint3d sOtherPD[2] ;
      m_pOtherCurve->Evaluate(dOtherU, 1, TRUE, sOtherPD, TRUE) ; // NonZeroTangents
      SmVector3d sGap = sOtherPD[0]-sThisPos ;
      SmGapDropType eGapDropType =    ( !sOtherDomain.IsValueOnBoundary(dOtherU) ) 
                                   || (  sGap.LengthSquared() < SM_EFF_ZERO_SQ )
                                   || (  sOtherPD[1].IsPerpendicularTo(sGap, 0.5) ) 
                                   ? SM_GD_NORMAL
                                   : SM_GD_BOUNDARY ;

      rGapSample.SetCanonical(sThisPos, dThisU, dThisV,
                              sOtherPD[0], dOtherU, 0.0,
                              eGapDropType, m_sXSectTol3d) ;      
    }
  else // failed to drop - load a zero length gap vector
    {
      rGapSample.SetCanonical(sThisPos, dThisU, dThisV,
                              sThisPos, m_pOtherCurve->GetNaturalInterval().GetMin(), 0.0,
                              SM_GD_NO_DROP, m_sXSectTol3d) ;      
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmSrfCrvGapFunction::Evaluate

/*******************************************************************//**
PURPOSE:
NOTES:
***********************************************************************/
SmBoolean SmSrfSrfGapFunction::IsKindOf( SM_TYPE t ) const
{
  return ((SmSrfSrfGapFunction_TYPE == t) ? TRUE : SmGapFunction::IsKindOf( (t) ));
}

/*******************************************************************//**
PURPOSE: Pretty print SmSrfSrfGapFunction data

NOTES: SmDerivedGapFunction::Dump() - outputs a Derived class label
        then calls Base SmGapFunction::Dump()
***********************************************************************/
void SmSrfSrfGapFunction::Dump()                  const { SmSrfSrfGapFunction::Dump(FALSE) ; }
void SmSrfSrfGapFunction::Dump(SmBoolean bAbbrev) const 
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE] ;

  smos_sprintf(sBuff, _T("%s"),_T("\nBegin SmSrfSrfGapFunction::Dump()")) ;
  smos_WriteBuffer(sBuff);

  // Derived class label - GapFunction[ptr] from Surface[ptr] to Surface[ptr]
  smos_sprintf(sBuff,        _T("\n GapFunction[0x%p] from Surface[0x%p] To Surface[0x%p]"),
             this, m_pThisSurface, m_pOtherSurface) ;
  smos_sprintf(sBuffForFile, _T("\n GapFunction[%s] from Surface[%s] To Surface[%s]"),
             _T("notNULL"), 
             m_pThisSurface ? _T("notNULL") : _T("NULL"), 
             m_pOtherSurface ? _T("notNULL") : _T("NULL")) ;
  smos_WriteBuffer(sBuff, sBuffForFile);

  // Pass the call along to base class
  SmGapFunction::Dump(bAbbrev) ;

  smos_sprintf(sBuff, _T("%s"),_T("\nEnd   SmSrfSrfGapFunction::Dump()")) ;
  smos_WriteBuffer(sBuff);

} // end SmSrfSrfGapFunction::Dump

/*******************************************************************//**
PURPOSE: Draw SmSrfSrfGapFunction graphics

NOTES:
***********************************************************************/
SmDisplayList * SmSrfSrfGapFunction::Draw
 (SmBoolean       bShowGeometry,       // in : TRUE = Show Geometry bounding the gaps, default:[TRUE]
  SmBoolean       bShowNeighborhoods,  // in : TRUE = Show InTol GapFunction neighborhood intervals, default:[TRUE]
  SmBoolean       bShowSamples,        // in : TRUE  = show all sample gaps, 
                                       //      UNSURE= Only show bigger-than-tol gaps, 
                                       //      FALSE = Show no sample gaps,
                                       //      default:[FALSE]
  double          dDisplayScale,       // in : Amount micro geometry is scaled for visualization, default:[1000]
  SmGfxArraySet * pOptGfxSet)          // i/o: used for OpenGL ES 2.0. When given output GfxVertexArrays not GL calls.
 const                                 //      NULL to ignore. default:[NULL]
{
  SmDisplayList *pRtn = NULL ;

#ifdef SM_GFX_CODE
  // start new displayList (unless one is already open)
  smgfx_Open(smgfx_GetRuleColor(), NULL, NULL, FALSE, pOptGfxSet);
  
  // draw From Geometry
  if(bShowGeometry) 
    { m_pThisSurface->Draw(TRUE, pOptGfxSet) ; }

  // draw To Geometry
  SmVector3d sColor = smgfx_OutputColor( smgfx_GetInTolGapColor() , pOptGfxSet) ;
  if(bShowGeometry) 
    { m_pOtherSurface->Draw(TRUE, pOptGfxSet) ; }
  smgfx_OutputColor( sColor , pOptGfxSet) ;

  // draw GapFunction
  SmGapFunction::Draw( bShowGeometry, bShowNeighborhoods, bShowSamples, dDisplayScale, pOptGfxSet) ;

  // end display list
  pRtn = smgfx_Close(pOptGfxSet) ;

#else
  SM_REF5(bShowGeometry, bShowNeighborhoods, bShowSamples, dDisplayScale, pOptGfxSet);
#endif // end SM_GFX_CODE
  return(pRtn) ;

} // end SmSrfSrfGapFunction::Draw

/*******************************************************************//**
PURPOSE: Evaluate SmSrfSrfGapFunction at specified location

NOTES:
***********************************************************************/
SmStatus SmSrfSrfGapFunction::Evaluate
 (double       dThisU,            // in : This Geom 1st param used for Curves and Surfaces
  double       dThisV,            // in : This Geom 2nd param only used for Surfaces
  SmGapSample &rGapSample,        // out: Eval holder block
  double      *pOptOtherGuessU,   // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
  double      *pOptOtherGuessV)   // in : When evaluating a sequence of gaps guess last neighbor gap param, NULL to ignore 
{
  // This Surface Eval
  SmPoint3d sThisPos ;
  SmPoint2d sThisUV(dThisU, dThisV) ;
  m_pThisSurface->EvaluatePoint(sThisUV, sThisPos) ;

  // Drop point to ToGeometry
  SmPoint2d *pOtherGuessUV = NULL, sOtherGuessUV ;
  if(pOptOtherGuessU && pOptOtherGuessV)
    {
      sOtherGuessUV.Set(*pOptOtherGuessU, *pOptOtherGuessV) ;
      pOtherGuessUV = &sOtherGuessUV ;
    }

  SmPoint2d sOtherUV ;
  //double dDistanceTolerance = 10.0 ;  // assume gap functions are near tolerance size - 10 is huge by comparison.
  double dDistDropped = 0.0;
  SmBoolean bSuccess ;
  SmBoolean bIsMulti;
  SmExtent2d sOtherDomain = m_pOtherSurface->GetNaturalUVDomain();
  m_pOtherSurface->DropPoint(sThisPos,      // in : target point to drop
                             sOtherDomain,  // in : target surface domain
                             pOtherGuessUV, // in : When given, uses only local solves
                             bSuccess,      // out: TRUE=Point dropped successfully, FALSE=didn't
                             sOtherUV,      // out: drop result UVPoint
                             dDistDropped,     // out: distance of found point to Pt to drop
                             bIsMulti); //

  // If we don't like the result of the drop with a guess,
  // try without a guess (global solve).
  // We do this if:
  // - We did use a guess the first time (if not, then nothing is different), AND
  //   - bSuccess indicates failure OR
  //   - gap is really big (local solve converged to wrong solution) [B279] OR
  //     - Gap is bigger than tol AND
  //     - result is on other curve's boundary.
  SmBoolean bTryGlobal = (    ( pOptOtherGuessU != NULL && pOptOtherGuessV != NULL )
                           && (    bSuccess == FALSE
                                || dDistDropped > m_sXSectTol3d*100
                                || (  dDistDropped > m_sXSectTol3d
                                    && sOtherDomain.IsPoint2dOnBoundary( sOtherUV )
                                   )
                              )
                         );

  if ( bTryGlobal )
    {
      // try a global drop point solution
      m_pOtherSurface->DropPoint(sThisPos,      // in : target point to drop
                                 sOtherDomain,  // in : target surface domain
                                 NULL,          // in : Global solve this time: no guess
                                 bSuccess,      // out: TRUE=Point dropped successfully, FALSE=didn't
                                 sOtherUV,      // out: drop result UVPoint
                                 dDistDropped,
                                 bIsMulti); // out: distance of found point to Pt to drop
    }
  // set output
  if(bSuccess)
    {
      // Other Surface dropped point Eval
      SmPoint3d sOtherPD[2][2] ;
      m_pOtherSurface->Evaluate(sOtherUV, 1, 1, TRUE, TRUE, TRUE, sOtherPD[0], TRUE) ; // NonZeroTangents
      SmVector3d sGap = sOtherPD[0][0]-sThisPos ;
      SmGapDropType eGapDropType =    ( !sOtherDomain.IsPoint2dOnBoundary(sOtherUV) ) 
                                   || (  sGap.LengthSquared() < SM_EFF_ZERO_SQ )
                                   || (   sOtherPD[0][1].IsPerpendicularTo(sGap, 0.5) 
                                       && sOtherPD[1][0].IsPerpendicularTo(sGap, 0.5)) 
                                   ? SM_GD_NORMAL
                                   : SM_GD_BOUNDARY ;
      rGapSample.SetCanonical(sThisPos, dThisU, dThisV,
                              sOtherPD[0][0], sOtherUV.x, sOtherUV.y,
                              eGapDropType, m_sXSectTol3d) ;      
    }
  else // failed to drop - load a zero length gap vector
    {
      rGapSample.SetCanonical(sThisPos, dThisU, dThisV,
                              sThisPos, 
                              m_pOtherSurface->GetNaturalUVDomain().GetMin().x, 
                              m_pOtherSurface->GetNaturalUVDomain().GetMin().y,
                              SM_GD_NO_DROP, m_sXSectTol3d) ;      
    }

  // all done
  return(SM_SUCCESS) ;

} // end SmSrfSrfGapFunction::Evaluate

/*******************************************************************//**
END SmGapFunction Derived Class Methods
***********************************************************************/


// OBSOLETE
//      
//      
//      /*******************************************************************//**
//      BEGIN SmVertexEdgeGap Methods
//      ***********************************************************************/
//      
//      /*******************************************************************//**
//      PURPOSE: SmVertexEdgeGap constructor
//      
//      NOTES:
//      ***********************************************************************/
//      SmVertexEdgeGap::SmVertexEdgeGap
//        (double       dEdgeToEdgeGap,     
//         SmVertexuse *pVertexuseToEdge,
//         double       dEdgeT)          
//       : SmGap(SM_GT_VERTEX_EDGE),
//         m_dVertexToEdgeGap(dEdgeToEdgeGap),
//         m_pVertexuseToEdge(pVertexuseToEdge),
//         m_dEdgeT          (dEdgeT)          
//      { 
//        // watch out for bad Vertexuse types
//        if(    pVertexuseToEdge
//           && !pVertexuseToEdge->IsEdgeVertexuse())
//          { 
//            // set values to uninitialized
//            m_dVertexToEdgeGap = SM_UNDEF_DOUBLE ;
//            pVertexuseToEdge   = NULL ;
//            m_dEdgeT           = SM_UNDEF_DOUBLE ;
//          }
//      
//      } // end SmVertexEdgeGap::SmVertexEdgeGap constructor
//      
//      /*******************************************************************//**
//      PURPOSE: SmVertexEdgeGap copy constructor
//      
//      NOTES:
//      ***********************************************************************/
//      SmVertexEdgeGap::SmVertexEdgeGap
//         (const SmVertexEdgeGap &crGap) 
//        : SmGap(crGap)                           
//      { 
//        m_pVertexuseToEdge = crGap.m_pVertexuseToEdge ;
//        m_dVertexToEdgeGap = crGap.m_dVertexToEdgeGap ;
//        m_dEdgeT           = crGap.m_dEdgeT ;
//      
//      } // end SmVertexEdgeGap::SmVertexEdgeGap copy constructor
//      
//      /*******************************************************************//**
//      PURPOSE: SmVertexEdgeGap ReSet
//      
//      NOTES:
//      ***********************************************************************/
//      void SmVertexEdgeGap::ReSet()                                              
//      { 
//        m_dVertexToEdgeGap = SM_UNDEF_DOUBLE ;
//        m_pVertexuseToEdge = NULL ;
//        m_dEdgeT           = SM_UNDEF_DOUBLE ;
//      
//      } // end SmVertexEdgeGap::~SmVertexEdgeGap destructor
//      
//      /*******************************************************************//**
//      PURPOSE: SmVertexEdgeGap assignment
//      
//      NOTES:
//      ***********************************************************************/
//      SmVertexEdgeGap & SmVertexEdgeGap::operator=
//        (const SmVertexEdgeGap &crGap)        
//      { 
//        if(this == &crGap) return *this ; 
//      
//        // internal values
//        m_pVertexuseToEdge = crGap.m_pVertexuseToEdge ;
//        m_dVertexToEdgeGap = crGap.m_dVertexToEdgeGap ;
//        m_dEdgeT           = crGap.m_dEdgeT ;
//      
//        // all done
//        return(*this) ;
//      
//      } // end SmVertexEdgeGap::operator=
//      
//      /*******************************************************************//**
//      PURPOSE: SmVertexEdgeGap equality
//      
//      NOTES:
//      ***********************************************************************/
//      SmBoolean SmVertexEdgeGap::operator==
//        (const SmGap &crGap)        
//      { 
//        if(this       == &crGap) return TRUE ; 
//        if(m_eGapType != crGap.m_eGapType) return FALSE ;
//      
//        // compute equality
//        return(   m_pVertexuseToEdge ==           ((SmVertexEdgeGap &)crGap).m_pVertexuseToEdge 
//               && SM_ARE_SAME(m_dVertexToEdgeGap, ((SmVertexEdgeGap &)crGap).m_dVertexToEdgeGap)
//               && SM_ARE_SAME(m_dEdgeT,           ((SmVertexEdgeGap &)crGap).m_dEdgeT)) ;
//      
//      } // end SmVertexEdgeGap::operator=
//      
//      /*******************************************************************//**
//      PURPOSE: return associated Vertex when possible
//      
//      NOTES:
//      ***********************************************************************/
//      SmVertex *SmVertexEdgeGap::GetVertex() const    
//      { 
//        return m_pVertexuseToEdge ? m_pVertexuseToEdge->GetVertex() : NULL ; 
//      
//      } // end SmVertexEdgeGap::GetVertex
//      
//      /*******************************************************************//**
//      PURPOSE: return associated Edge when possible
//      
//      NOTES:
//      ***********************************************************************/
//      SmEdge *SmVertexEdgeGap::GetEdge() const    
//      { 
//        return m_pVertexuseToEdge ? m_pVertexuseToEdge->GetEdgeuse()->GetEdge() : NULL ; 
//      }
//      
//      /*******************************************************************//**
//      PURPOSE: Pretty print Vertex to Edge Gap 
//      
//      NOTES:
//      ***********************************************************************/
//      void SmVertexEdgeGap::Dump() 
//        const
//      {
//        TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
//                                        
//        smos_sprintf(sBuff       ,_T("\n    Vertex/Edge Gap = %16.16lf, Vertex[0x%p], Edge[0x%p, T=%16.16lf])"),
//                   GetGap(),
//                   GetVertex(),
//                   GetEdge(),
//                   GetEdgeT() ) ;
//      
//        smos_sprintf(sBuffForFile,_T("\n    Vertex/Edge Gap = %16.16lf, Vertex[%s], Edge[%s, T=%16.16lf])"),
//                   GetGap(),
//                   GetVertex() ? _T("NotNULL") : _T("NULL") ,
//                   GetEdge()   ? _T("NotNULL") : _T("NULL") ,
//                   GetEdgeT() ) ;
//      
//        smos_WriteBuffer(sBuff, sBuffForFile);
//      
//      } // end SmVertexEdgeGap::Dump      
//      
//      /*******************************************************************//**
//      PURPOSE: Draw a Vertex/Edge graphic
//      
//      NOTES:
//      ***********************************************************************/
//      SmDisplayList * SmVertexEdgeGap::Draw() 
//        const
//      {
//        SmDisplayList *pRtn = NULL ;
//      
//      #ifdef SM_GFX_CODE
//      
//        // start new displayList (unless one is already open)
//        smgfx_Open(smgfx_GetRuleColor()), NULL, NULL, FALSE, pOptGfxSet;
//      
//        // output graphics locals
//        SmVertex *pVertex = GetVertex() ;
//        SmEdge   *pEdge   = GetEdge() ;
//        SmCurve  *pCurve  = pEdge ? pEdge->GetCurve() : NULL ;
//      
//        SmPoint3d sVertexPt, sEdgePt ;
//        
//        if(pEdge && pVertex)
//          {
//            // Vertex Point
//            sVertexPt = pVertex->GetPoint() ;
//        
//            // Edge End Point
//            pEdge->GetCurve()->EvaluatePoint(GetEdgeT(), sEdgePt) ;
//      
//            // output graphics
//            sVertexPt.DrawPointToPoint(sEdgePt) ;
//          }
//      
//        // end display list
//        pRtn = smgfx_Close(pOptGfxSet) ;
//      
//      #endif // end SM_GFX_CODE
//        return(pRtn) ;
//      
//      } // end SmVertexEdgeGap::Draw
//      
//      /*******************************************************************//**
//      END SmVertexEdgeGap Methods
//      ***********************************************************************/
//      
//      /*******************************************************************//**
//      BEGIN SmVertexFaceGap Methods
//      ***********************************************************************/
//      
//      /*******************************************************************//**
//      PURPOSE: SmVertexFaceGap constructor
//      
//      NOTES:
//      ***********************************************************************/
//      SmVertexFaceGap::SmVertexFaceGap
//        (double       dVertexToFaceGap,     
//         SmVertexuse *pVertexuseToFace,
//         SmPoint2d   *pFaceUV)          
//       : SmGap(SM_GT_VERTEX_EDGE),
//         m_dVertexToFaceGap(dVertexToFaceGap),
//         m_pVertexuseToFace(pVertexuseToFace) 
//      { 
//        // watch out for bad Vertexuse types
//        if(    pVertexuseToFace
//           && !pVertexuseToFace->IsEdgeVertexuse()
//           && !pVertexuseToFace->IsLoopVertexuse())
//          { 
//            // set values to uninitialized
//            m_dVertexToFaceGap = SM_UNDEF_DOUBLE ;
//            pVertexuseToFace   = NULL ;
//            m_sFaceUV.SetUninitialized() ;
//          }
//        else
//          {
//            if(pFaceUV) m_sFaceUV.Set(pFaceUV->x, pFaceUV->y) ;
//            else        m_sFaceUV.SetUninitialized() ;
//          }
//      
//      } // end SmVertexFaceGap::SmVertexFaceGap constructor
//      
//      /*******************************************************************//**
//      PURPOSE: SmVertexFaceGap copy constructor
//      
//      NOTES:
//      ***********************************************************************/
//      SmVertexFaceGap::SmVertexFaceGap
//         (const SmVertexFaceGap &crGap)                            
//        : SmGap(crGap)                           
//      { 
//        m_dVertexToFaceGap = crGap.m_dVertexToFaceGap ;
//        m_pVertexuseToFace = crGap.m_pVertexuseToFace ;
//        m_sFaceUV          = crGap.m_sFaceUV ;
//      
//      } // end SmVertexFaceGap::SmVertexFaceGap copy constructor
//      
//      /*******************************************************************//**
//      PURPOSE: SmVertexFaceGap ReSet
//      
//      NOTES:
//      ***********************************************************************/
//      void SmVertexFaceGap::ReSet()                                              
//      { 
//        m_dVertexToFaceGap = SM_UNDEF_DOUBLE ;
//        m_pVertexuseToFace = NULL ;
//        m_sFaceUV.SetUninitialized() ;
//      
//      } // end SmVertexFaceGap::ReSet
//      
//      /*******************************************************************//**
//      PURPOSE: SmVertexFaceGap assignment
//      
//      NOTES:
//      ***********************************************************************/
//      SmVertexFaceGap & SmVertexFaceGap::operator=
//        (const SmVertexFaceGap &crGap)        
//      { 
//        if(this == &crGap) return *this ; 
//      
//        // internal values
//        m_pVertexuseToFace = crGap.m_pVertexuseToFace ;
//        m_dVertexToFaceGap = crGap.m_dVertexToFaceGap ;
//        m_sFaceUV          = crGap.m_sFaceUV ;
//      
//        // all done
//        return(*this) ;
//      
//      } // end SmVertexFaceGap::operator=
//      
//      /*******************************************************************//**
//      PURPOSE: SmVertexFaceGap equality
//      
//      NOTES:
//      ***********************************************************************/
//      SmBoolean SmVertexFaceGap::operator==
//        (const SmGap &crGap)        
//      { 
//        if(this       == &crGap) return TRUE ; 
//        if(m_eGapType != crGap.m_eGapType) return FALSE ;
//      
//        // compute equality
//        return(   m_pVertexuseToFace ==             ((SmVertexFaceGap &)crGap).m_pVertexuseToFace 
//               && SM_ARE_SAME(m_dVertexToFaceGap,   ((SmVertexFaceGap &)crGap).m_dVertexToFaceGap)
//               && m_sFaceUV.CloserThan(SM_EFF_ZERO, ((SmVertexFaceGap &)crGap).m_sFaceUV)) ;
//      
//      } // end SmVertexFaceGap::operator=
//      
//      /*******************************************************************//**
//      PURPOSE: return associated vertex when possible
//      
//      NOTES:
//      ***********************************************************************/
//      SmVertex *SmVertexFaceGap::GetVertex() const    
//      { 
//        return m_pVertexuseToFace ? m_pVertexuseToFace->GetVertex() : NULL ; 
//      
//      } // end SmVertexFaceGap::GetVertex
//      
//      /*******************************************************************//**
//      PURPOSE: return associated face when possible
//      
//      NOTES:
//      ***********************************************************************/
//      SmFace *SmVertexFaceGap::GetFace() const    
//      { 
//        return(  (m_pVertexuseToFace && m_pVertexuseToFace->IsEdgeVertexuse()) 
//               ?  m_pVertexuseToFace->GetEdgeuse()->GetFace() 
//               : (m_pVertexuseToFace && m_pVertexuseToFace->IsLoopVertexuse())
//               ?  m_pVertexuseToFace->GetLoopuse()->GetFaceuse()->GetFace()
//               : NULL) ; 
//      
//      } // end SmVertexFaceGap::GetFace
//      
//      /*******************************************************************//**
//      PURPOSE: Pretty print Vertex to Face Gap 
//      
//      NOTES:
//      ***********************************************************************/
//      void SmVertexFaceGap::Dump() 
//        const
//      {
//        TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
//                                        
//        smos_sprintf(sBuff       ,_T("\n    Vertex/Face Gap = %16.16lf, Vertex[0x%p], Face[0x%p, UV=%16.16lf, %16.16lf])"),
//                   GetGap(),
//                   GetVertex(),
//                   GetFace(),
//                   m_sFaceUV.x,
//                   m_sFaceUV.y ) ;
//      
//        smos_sprintf(sBuffForFile,_T("\n    Vertex/Face Gap = %16.16lf, Vertex[%s], Face[%s, UV=%16.16lf, %16.16lf])"),
//                   GetGap(),
//                   GetVertex() ? _T("NotNULL") : _T("NULL") ,
//                   GetFace()   ? _T("NotNULL") : _T("NULL") ,
//                   m_sFaceUV.x,
//                   m_sFaceUV.y ) ;
//      
//        smos_WriteBuffer(sBuff, sBuffForFile);
//      
//      } // end SmVertexFaceGap::Dump      
//      
//      /*******************************************************************//**
//      PURPOSE: Draw a Vertex/Face graphic
//      
//      NOTES:
//      ***********************************************************************/
//      SmDisplayList * SmVertexFaceGap::Draw() 
//        const
//      {
//        SmDisplayList *pRtn = NULL ;
//      
//      #ifdef SM_GFX_CODE
//      
//        // start new displayList (unless one is already open)
//        smgfx_Open(smgfx_GetRuleColor()), NULL, NULL, FALSE, pOptGfxSet;
//      
//        // output graphics locals
//        SmVertex  *pVertex  = GetVertex() ;
//        SmFace    *pFace    = GetFace() ;
//        SmSurface *pSurface = pFace ? pFace->GetSurface() : NULL ;
//      
//        SmPoint3d sVertexPt, sFacePt ;
//        
//        if(pVertex && pFace)
//          {
//            // Vertex Point
//            sVertexPt = pVertex->GetPoint() ;
//        
//            // Face End Point
//            pFace->GetSurface()->EvaluatePoint( ((SmVertexFaceGap*)this)->GetFaceUV(), sFacePt) ;
//      
//            // output graphics
//            sVertexPt.DrawPointToPoint(sFacePt) ;
//          }
//      
//        // end display list
//        pRtn = smgfx_Close(pOptGfxSet) ;
//      
//      #endif // end SM_GFX_CODE
//        return(pRtn) ;
//      
//      } // end SmVertexFaceGap::Draw
//      
//      /*******************************************************************//**
//      END SmVertexFaceGap Methods
//      ***********************************************************************/
//      
//      
//      /*******************************************************************//**
//      BEGIN SmEdgeVertexGap Methods
//      ***********************************************************************/
//      
//      /*******************************************************************//**
//      PURPOSE: SmEdgeVertexGap constructor
//      
//      NOTES:
//      ***********************************************************************/
//      SmEdgeVertexGap::SmEdgeVertexGap
//        (double     dEdgeToVertexGap,     
//         SmEdgeuse *pEdgeuseToVertex,
//         double     dEdgeT)          
//       : SmGap(SM_GT_EDGE_VERTEX),
//         m_dEdgeToVertexGap(dEdgeToVertexGap),
//         m_pEdgeuseToVertex(pEdgeuseToVertex),
//         m_dEdgeT          (dEdgeT)          
//      { 
//        // watch out for bad Edgeuse types
//        if(    pEdgeuseToVertex
//           && !pEdgeuseToVertex->IsLoopEdgeuse())
//          { 
//            // set values to uninitialized
//            m_dEdgeToVertexGap = SM_UNDEF_DOUBLE ;
//            pEdgeuseToVertex   = NULL ;
//            m_dEdgeT           = SM_UNDEF_DOUBLE ;
//          }
//      
//      } // end SmEdgeVertexGap::SmEdgeVertexGap constructor
//      
//      /*******************************************************************//**
//      PURPOSE: SmEdgeVertexGap copy constructor
//      
//      NOTES:
//      ***********************************************************************/
//      SmEdgeVertexGap::SmEdgeVertexGap
//         (const SmEdgeVertexGap &crGap)                            
//        : SmGap(crGap)                           
//      { 
//        m_dEdgeToVertexGap = crGap.m_dEdgeToVertexGap ;
//        m_pEdgeuseToVertex = crGap.m_pEdgeuseToVertex ;
//        m_dEdgeT           = crGap.m_dEdgeT ;
//      
//      } // end SmEdgeVertexGap::SmEdgeVertexGap copy constructor
//      
//      /*******************************************************************//**
//      PURPOSE: SmEdgeVertexGap ReSet
//      
//      NOTES:
//      ***********************************************************************/
//      void SmEdgeVertexGap::ReSet()                                              
//      { 
//        m_dEdgeToVertexGap = SM_UNDEF_DOUBLE ;
//        m_pEdgeuseToVertex = NULL ;
//        m_dEdgeT           = SM_UNDEF_DOUBLE ;
//      
//      } // end SmEdgeVertexGap::ReSet
//      
//      /*******************************************************************//**
//      PURPOSE: SmEdgeVertexGap assignment
//      
//      NOTES:
//      ***********************************************************************/
//      SmEdgeVertexGap & SmEdgeVertexGap::operator=
//        (const SmEdgeVertexGap &crGap)        
//      { 
//        if(this == &crGap) return *this ; 
//      
//        // internal values
//        m_dEdgeToVertexGap = crGap.m_dEdgeToVertexGap ;
//        m_pEdgeuseToVertex = crGap.m_pEdgeuseToVertex ;
//        m_dEdgeT           = crGap.m_dEdgeT ;
//      
//        // all done
//        return(*this) ;
//      
//      } // end SmEdgeVertexGap::operator=
//      
//      /*******************************************************************//**
//      PURPOSE: SmEdgeVertexGap equality
//      
//      NOTES:
//      ***********************************************************************/
//      SmBoolean SmEdgeVertexGap::operator==
//        (const SmGap &crGap)        
//      { 
//        if(this       == &crGap) return TRUE ; 
//        if(m_eGapType != crGap.m_eGapType) return FALSE ;
//      
//        // compute equality
//        return(   m_pEdgeuseToVertex ==           ((SmEdgeVertexGap &)crGap).m_pEdgeuseToVertex 
//               && SM_ARE_SAME(m_dEdgeToVertexGap, ((SmEdgeVertexGap &)crGap).m_dEdgeToVertexGap)
//               && SM_ARE_SAME(m_dEdgeT,           ((SmEdgeVertexGap &)crGap).m_dEdgeT)) ;
//      
//      } // end SmEdgeVertexGap::operator=
//      
//      /*******************************************************************//**
//      PURPOSE: return associated Edge when possible
//      
//      NOTES:
//      ***********************************************************************/
//      SmEdge *SmEdgeVertexGap::GetEdge() const    
//      {
//        return m_pEdgeuseToVertex ? m_pEdgeuseToVertex->GetEdge() : NULL ; 
//      
//      } // end SmEdgeVertexGap::GetEdge
//      
//      /*******************************************************************//**
//      PURPOSE: return associated Vertex when possible
//      
//      NOTES:
//      ***********************************************************************/
//      SmVertex *SmEdgeVertexGap::GetVertex() const    
//      { 
//        return m_pEdgeuseToVertex ? m_pEdgeuseToVertex->GetVertexuse()->GetVertex() : NULL ; 
//      
//      } // end SmEdgeVertexGap::GetVertex
//      
//      /*******************************************************************//**
//      PURPOSE: Pretty print Edge to Vertex Gap 
//      
//      NOTES:
//      ***********************************************************************/
//      void SmEdgeVertexGap::Dump() 
//        const
//      {
//        TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
//                                        
//        smos_sprintf(sBuff       ,_T("\n    Edge/Vertex Gap = %16.16lf, Edge[0x%p, T=%16.16lf], Vertex[0x%p])"),
//                   GetGap(),
//                   GetEdge(),
//                   GetEdgeT(),
//                   GetVertex() ) ;
//      
//        smos_sprintf(sBuffForFile,_T("\n    Edge/Vertex Gap = %16.16lf, Edge[%s, T=%16.16lf], Vertex[%s])"),
//                   GetGap(),
//                   GetEdge() ? _T("NotNULL") : _T("NULL") ,
//                   GetEdgeT(),
//                   GetVertex()   ? _T("NotNULL") : _T("NULL")  ) ;
//      
//        smos_WriteBuffer(sBuff, sBuffForFile);
//      
//      } // end SmEdgeVertexGap::Dump      
//      
//      /*******************************************************************//**
//      PURPOSE: Draw a Edge/Vertex graphic
//      
//      NOTES:
//      ***********************************************************************/
//      SmDisplayList * SmEdgeVertexGap::Draw() 
//        const
//      {
//        SmDisplayList *pRtn = NULL ;
//      
//      #ifdef SM_GFX_CODE
//      
//        // start new displayList (unless one is already open)
//        smgfx_Open(smgfx_GetRuleColor()), NULL, NULL, FALSE, pOptGfxSet;
//      
//        // output graphics locals
//        SmEdge   *pEdge   = GetEdge() ;
//        SmVertex *pVertex = GetVertex() ;
//        SmCurve  *pCurve  = pEdge ? pEdge->GetCurve() : NULL ;
//      
//        SmPoint3d sEdgePt, sVertexPt ;
//        
//        if(pEdge && pVertex)
//          {
//            // Edge End Point
//            if(pEdge) pEdge->GetCurve()->EvaluatePoint(GetEdgeT(), sEdgePt) ;
//      
//            // Vertex Point
//            if(pVertex) sVertexPt = pVertex->GetPoint() ;
//      
//            // output graphics
//            sEdgePt.DrawPointToPoint(sVertexPt) ;
//          }
//      
//        // end display list
//        pRtn = smgfx_Close(pOptGfxSet) ;
//      
//      #endif // end SM_GFX_CODE
//        return(pRtn) ;
//      
//      } // end SmEdgeVertexGap::Draw
//      
//      /*******************************************************************//**
//      END SmEdgeVertexGap Methods
//      ***********************************************************************/
//      
//      
//      
//      /*******************************************************************//**
//      BEGIN SmEdgeFaceGap Methods
//      ***********************************************************************/
//      
//      /*******************************************************************//**
//      PURPOSE: SmEdgeFaceGap constructor
//      
//      NOTES:
//      ***********************************************************************/
//      SmEdgeFaceGap::SmEdgeFaceGap
//        (double       dEdgeToFaceGap,     
//         SmEdgeuse   *pEdgeuseToFace,
//         SmPoint2d   *pFaceUV,
//         double       dEdgeT)          
//       : SmGap(SM_GT_EDGE_FACE),
//         m_dEdgeToFaceGap(dEdgeToFaceGap),
//         m_pEdgeuseToFace(pEdgeuseToFace),
//         m_dEdgeT(dEdgeT) 
//      { 
//        // watch out for bad Edgeuse types
//        if(    pEdgeuseToFace
//           && !pEdgeuseToFace->IsLoopEdgeuse())
//          { 
//            // set values to uninitialized
//            m_dEdgeToFaceGap = SM_UNDEF_DOUBLE ;
//            pEdgeuseToFace   = NULL ;
//            m_sFaceUV.SetUninitialized() ;
//            m_dEdgeT         = SM_UNDEF_DOUBLE ;
//          }
//        else
//          {
//            if(pFaceUV) m_sFaceUV.Set(pFaceUV->x, pFaceUV->y) ;
//            else        m_sFaceUV.SetUninitialized() ;
//          }
//      
//      } // end SmEdgeFaceGap::SmEdgeFaceGap constructor
//      
//      /*******************************************************************//**
//      PURPOSE: SmEdgeFaceGap copy constructor
//      
//      NOTES:
//      ***********************************************************************/
//      SmEdgeFaceGap::SmEdgeFaceGap
//         (const SmEdgeFaceGap &crGap)                            
//        : SmGap(crGap)                           
//      { 
//        m_dEdgeToFaceGap = crGap.m_dEdgeToFaceGap ;
//        m_pEdgeuseToFace = crGap.m_pEdgeuseToFace ;
//        m_sFaceUV        = crGap.m_sFaceUV ;
//        m_dEdgeT         = crGap.m_dEdgeT ;
//      
//      } // end SmEdgeFaceGap::SmEdgeFaceGap copy constructor
//      
//      /*******************************************************************//**
//      PURPOSE: SmEdgeFaceGap ReSet
//      
//      NOTES:
//      ***********************************************************************/
//      void SmEdgeFaceGap::ReSet()                                              
//      { 
//        m_dEdgeToFaceGap = SM_UNDEF_DOUBLE ;
//        m_pEdgeuseToFace = NULL ;
//        m_sFaceUV.SetUninitialized() ;
//        m_dEdgeT         = SM_UNDEF_DOUBLE ;
//      
//      } // end SmEdgeFaceGap::ReSet
//      
//      /*******************************************************************//**
//      PURPOSE: SmEdgeFaceGap assignment
//      
//      NOTES:
//      ***********************************************************************/
//      SmEdgeFaceGap & SmEdgeFaceGap::operator=
//        (const SmEdgeFaceGap &crGap)        
//      { 
//        if(this == &crGap) return *this ; 
//      
//        // internal values
//        m_pEdgeuseToFace = crGap.m_pEdgeuseToFace ;
//        m_dEdgeToFaceGap = crGap.m_dEdgeToFaceGap ;
//        m_sFaceUV        = crGap.m_sFaceUV ;
//        m_dEdgeT         = crGap.m_dEdgeT ;
//      
//        // all done
//        return(*this) ;
//      
//      } // end SmEdgeFaceGap::operator=
//      
//      /*******************************************************************//**
//      PURPOSE: SmEdgeFaceGap equality
//      
//      NOTES:
//      ***********************************************************************/
//      SmBoolean SmEdgeFaceGap::operator==
//        (const SmGap &crGap)        
//      { 
//        if(this       == &crGap) return TRUE ; 
//        if(m_eGapType != crGap.m_eGapType) return FALSE ;
//      
//        // compute equality
//        return(   m_pEdgeuseToFace ==                ((SmEdgeFaceGap &)crGap).m_pEdgeuseToFace 
//               && SM_ARE_SAME(m_dEdgeToFaceGap,      ((SmEdgeFaceGap &)crGap).m_dEdgeToFaceGap)
//               && m_sFaceUV.CloserThan(SM_EFF_ZERO,  ((SmEdgeFaceGap &)crGap).m_sFaceUV)
//               && SM_ARE_SAME(m_dEdgeT,              ((SmEdgeFaceGap &)crGap).m_dEdgeT)) ;
//      
//      } // end SmEdgeFaceGap::operator=
//      
//      /*******************************************************************//**
//      PURPOSE: return associated Edge when possible
//      
//      NOTES:
//      ***********************************************************************/
//      SmEdge *SmEdgeFaceGap::GetEdge()const    
//      {
//        return m_pEdgeuseToFace ? m_pEdgeuseToFace->GetEdge() : NULL ; 
//      
//      } // end SmEdgeFaceGap::GetEdge
//      
//      /*******************************************************************//**
//      PURPOSE: return associated Face when possible
//      
//      NOTES:
//      ***********************************************************************/
//      SmFace *SmEdgeFaceGap::GetFace()const    
//      {
//        return m_pEdgeuseToFace ? m_pEdgeuseToFace->GetFace() : NULL ; 
//      
//      } // end SmEdgeFaceGap::GetFace
//      
//      /*******************************************************************//**
//      PURPOSE: Pretty print Edge to Face Gap 
//      
//      NOTES:
//      ***********************************************************************/
//      void SmEdgeFaceGap::Dump() 
//        const
//      {
//        TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];
//                                        
//        smos_sprintf(sBuff       ,_T("\n    Edge/Face   Gap = %16.16lf, Edge[0x%p, T=%16.16lf], Face[0x%p, UV=%16.16lf, %16.16lf])"),
//                   GetGap(),
//                   GetEdge(),
//                   GetEdgeT(),
//                   GetFace(),
//                   m_sFaceUV.x,
//                   m_sFaceUV.y ) ;
//      
//        smos_sprintf(sBuffForFile,_T("\n    Edge/Face   Gap = %16.16lf, Edge[%s, T=%16.16lf], Face[%s, UV=%16.16lf, %16.16lf])"),
//                   GetGap(),
//                   GetEdge() ? _T("NotNULL") : _T("NULL") ,
//                   GetEdgeT(),
//                   GetFace()   ? _T("NotNULL") : _T("NULL") ,
//                   m_sFaceUV.x,
//                   m_sFaceUV.y ) ;
//      
//        smos_WriteBuffer(sBuff, sBuffForFile);
//      
//      } // end SmEdgeFaceGap::Dump      
//      
//      /*******************************************************************//**
//      PURPOSE: Draw a Edge/Face graphic
//      
//      NOTES:
//      ***********************************************************************/
//      SmDisplayList * SmEdgeFaceGap::Draw() 
//        const
//      {
//        SmDisplayList *pRtn = NULL ;
//      
//      #ifdef SM_GFX_CODE
//      
//        // start new displayList (unless one is already open)
//        smgfx_Open(smgfx_GetRuleColor()), NULL, NULL, FALSE, pOptGfxSet;
//      
//        // output graphics locals
//        SmEdge    *pEdge    = GetEdge() ;
//        SmFace    *pFace    = GetFace() ;
//        SmSurface *pSurface = pFace ? pFace->GetSurface() : NULL ;
//      
//        SmPoint3d sEdgePt, sFacePt ;
//        
//        if(pEdge && pFace)
//          {
//            // Edge Point
//            pEdge->GetCurve()->EvaluatePoint(GetEdgeT(), sEdgePt) ;
//        
//            // Face End Point
//            pFace->GetSurface()->EvaluatePoint( ((SmEdgeFaceGap*)this)->GetFaceUV(), sFacePt) ;
//      
//            // output graphics
//            sEdgePt.DrawPointToPoint(sFacePt) ;
//          }
//      
//        // end display list
//        pRtn = smgfx_Close(pOptGfxSet) ;
//      
//      #endif // end SM_GFX_CODE
//        return(pRtn) ;
//      
//      } // end SmEdgeFaceGap::Draw
//      
//      /*******************************************************************//**
//      END SmEdgeFaceGap Methods
//      ***********************************************************************/
//      
// end OBSOLETE1

// begin OBSOLETE2
//  /*******************************************************************//**
//  PURPOSE: SM_GT_VERTEX_EDGE/SM_GT_VERTEX_UVTRIMCURVE constructor
//  
//  NOTES:
//  ***********************************************************************/
//  SmGap::SmGap                  // eff: SM_GT_VERTEX_EDGE/SM_GT_VERTEX_UVTRIMCURVE constructor
//    (double       dGap3d,       // in : 3d gap
//     SmVertexuse *pVertexuse,   // in : object connecting vertex to edge
//     double       dEdgeT,       // in : Edge->Curve or UVTrimCurve Param value at Gap     
//     SmGapType    eGapType)     // in : oneof SM_GT_VERTEX_EDGE or SM_GT_VERTEX_UVTRIMCURVE
//                                //      default:[SM_GT_VERTEX_EDGE]
//  {
//    if(   eGapType != SM_GT_VERTEX_EDGE 
//       && eGapType != SM_GT_VERTEX_UVTRIMCURVE) 
//     {
//       WARN(_T("Bad Gap type given as input - being changed to SM_GT_VERTEX_EDGE")) ;
//       eGapType = SM_GT_VERTEX_EDGE ;
//     } 
//            
//    m_eGapType      = eGapType ;
//    m_dGap3d        = dGap3d ;
//    m_pEdgeuse      = NULL ;
//    m_pVertexuse    = pVertexuse ;
//    if(eGapType == SM_GT_VERTEX_EDGE) { m_dEdgeT        = dEdgeT ;
//                                        m_dUVTrimCurveT = SM_UNDEF_DOUBLE ;
//                                      }
//    else                              { m_dEdgeT        = SM_UNDEF_DOUBLE ;
//                                        m_dUVTrimCurveT = dEdgeT ; 
//                                      }
//    m_sFaceUV.SetUninitialized() ;
//  
//  } // end SmGap::SmGap SM_GT_VERTEX_EDGE constructor
//  
//  /*******************************************************************//**
//  PURPOSE: SM_GT_VERTEX_FACE constructor
//  
//  NOTES:
//  ***********************************************************************/
//  SmGap::SmGap                   // eff: SM_GT_VERTEX_FACE constructor
//    (double       dGap3d,        // in : 3d gap                          
//     SmVertexuse *pVertexuse,    // in : object connecting vertex to face
//     SmPoint2d   &rFaceUV)       // in : Face Param value at Gap             
//  {
//    m_eGapType      = SM_GT_VERTEX_FACE ;
//    m_dGap3d        = dGap3d ;
//    m_pEdgeuse      = NULL ;
//    m_pVertexuse    = pVertexuse ;
//    m_dEdgeT        = SM_UNDEF_DOUBLE ;
//    m_dUVTrimCurveT = SM_UNDEF_DOUBLE ;
//    m_sFaceUV       = rFaceUV ;
//  
//  } // end SmGap::SmGap SM_GT_VERTEX_FACE constructor
//  
//  /*******************************************************************//**
//  PURPOSE: SM_GT_EDGE_UVTRIMCURVE constructor
//  
//  NOTES:
//  ***********************************************************************/
//  SmGap::SmGap                 // eff: SM_GT_EDGE_UVTRIMCURVE constructor
//    (double     dGap3d,        // in : 3d Gap
//     SmEdgeuse *pEdgeuse,      // in : object connecting Edge to Face
//     double     dEdgeT,        // in : Edge->Curve Param value at Gap
//     double     dUVTrimCurveT) // in : Edge->UVTrimCurve Param value at Gap
//  { 
//    m_eGapType      = SM_GT_EDGE_UVTRIMCURVE ;
//    m_dGap3d        = dGap3d ;
//    m_pEdgeuse      = pEdgeuse ;
//    m_pVertexuse    = NULL ;
//    m_dEdgeT        = dEdgeT ;
//    m_dUVTrimCurveT = dUVTrimCurveT ;
//    m_sFaceUV.SetUninitialized() ;
//      
//  } // end SmGap::SmGap SM_GT_EDGE_FACE constructor
//  
//  /*******************************************************************//**
//  PURPOSE: SM_GT_EDGE_FACE constructor
//  
//  NOTES:
//  ***********************************************************************/
//  SmGap::SmGap                 // eff: SM_GT_EDGE_FACE constructor
//    (double     dGap3d,        // in : 3d Gap
//     SmEdgeuse *pEdgeuse,      // in : object connecting Edge to Face
//     double     dEdgeT,        // in : Edge Param value at Gap
//     SmPoint2d &rFaceUV)       // in : Face Param value at Gap      
//  { 
//    m_eGapType      = SM_GT_EDGE_FACE ;
//    m_dGap3d        = dGap3d ;
//    m_pEdgeuse      = pEdgeuse ;
//    m_pVertexuse    = NULL ;
//    m_dEdgeT        = dEdgeT ;
//    m_dUVTrimCurveT = SM_UNDEF_DOUBLE ;
//    m_sFaceUV       = rFaceUV ;
//  
//  } // end SmGap::SmGap SM_GT_EDGE_FACE constructor
//  
//  /*******************************************************************//**
//  PURPOSE: copy constructor
//  
//  NOTES:
//  ***********************************************************************/
//  SmGap::SmGap
//    (const SmGap &crGap)        // in : object to copy     
//  { 
//    m_eGapType      = crGap.m_eGapType ; 
//    m_dGap3d        = crGap.m_dGap3d ;   
//    m_pEdgeuse      = crGap.m_pEdgeuse ; 
//    m_pVertexuse    = crGap.m_pVertexuse ;
//    m_dEdgeT        = crGap.m_dEdgeT ;
//    m_dUVTrimCurveT = crGap.m_dUVTrimCurveT ;  
//    m_sFaceUV       = crGap.m_sFaceUV ;  
//  
//  } // end SmGap::SmGap copy constructor
//  
//  /*******************************************************************//**
//  PURPOSE: assignment operator
//  
//  NOTES:
//  ***********************************************************************/
//  SmGap & SmGap::operator=
//    (const SmGap &crGap)  
//  {
//    if(this == &crGap) return *this ;
//    m_eGapType      = crGap.m_eGapType ; 
//    m_dGap3d        = crGap.m_dGap3d ;   
//    m_pEdgeuse      = crGap.m_pEdgeuse ; 
//    m_pVertexuse    = crGap.m_pVertexuse ;
//    m_dEdgeT        = crGap.m_dEdgeT ;   
//    m_dUVTrimCurveT = crGap.m_dUVTrimCurveT ;
//    m_sFaceUV       = crGap.m_sFaceUV ;
//    return *this ;
//  
//  } // end ::SmGap &operator= assignment operator
//  
//  /*******************************************************************//**
//  PURPOSE: ReSet
//  
//  NOTES:
//  ***********************************************************************/
//  void SmGap::ReSet()                     
//  { 
//    m_eGapType      = SM_GT_UNKNOWN ;
//    m_dGap3d        = SM_UNDEF_DOUBLE ;
//    m_pEdgeuse      = NULL ;
//    m_pVertexuse    = NULL ;
//    m_dEdgeT        = SM_UNDEF_DOUBLE ; 
//    m_dUVTrimCurveT = SM_UNDEF_DOUBLE ;
//    m_sFaceUV.SetUninitialized() ;
//  
//  } // end SmGap::ReSet
//  
//  /*******************************************************************//**
//  PURPOSE: Equality operator
//  
//  NOTES:
//  ***********************************************************************/
//  SmBoolean SmGap::operator==    // rtn: TRUE when two gaps are the same
//    (const SmGap &crGap)         // in : target gap
//   const
//  {
//    double dTol3d = 100.0 * SM_EFF_ZERO * (1.0 + m_dGap3d) ;
//    double dTol2d = 100.0 * SM_EFF_ZERO * (1.0 + (GetEdge() ? m_dEdgeT : 0.0)
//                                               + (GetFace() ? m_sFaceUV.GetMaxDimension() : 0.0)) ;
//    SmBoolean bRtn = (    m_eGapType == crGap.m_eGapType
//                      &&  SM_ARE_SAME_TO_TOL(m_dGap3d, crGap.m_dGap3d, dTol3d)
//                      &&  GetVertex()  == crGap.GetVertex()
//                      &&  GetEdge()    == crGap.GetEdge()
//                      &&  GetFace()    == crGap.GetFace()
//                      && (GetEdge()    == NULL || SM_ARE_SAME_TO_TOL(m_dEdgeT,        crGap.m_dEdgeT,        dTol2d))
//                      && (GetEdgeuse() == NULL || SM_ARE_SAME_TO_TOL(m_dUVTrimCurveT, crGap.m_dUVTrimCurveT, dTol2d))
//                      && (GetFace()    == NULL || m_sFaceUV.CloserThan(dTol2d, crGap.m_sFaceUV))) ; 
//    // all done
//    return(bRtn) ;
//  
//  } // end SmBoolean SmGap::operator==
//  
//  /*******************************************************************//**
//  PURPOSE: GetVertex when gap connects to one else return NULL
//  
//  NOTES:
//  ***********************************************************************/
//  SmVertex * SmGap::GetVertex()  const
//  {
//    return(  (   m_eGapType == SM_GT_VERTEX_EDGE
//              || m_eGapType == SM_GT_VERTEX_UVTRIMCURVE 
//              || m_eGapType == SM_GT_VERTEX_FACE)  ? m_pVertexuse->GetVertex()
//           : (   m_eGapType == SM_GT_EDGE_UVTRIMCURVE
//              || m_eGapType == SM_GT_EDGE_FACE)    ? (SmVertex *) NULL
//           : (SmVertex *)NULL) ;
//  
//  } // end SmGap::GetVertex
//  
//  /*******************************************************************//**
//  PURPOSE: Get Edge when gap connects to one else return NULL
//  
//  NOTES:
//  ***********************************************************************/
//  SmEdge * SmGap::GetEdge()    const
//  {
//    return(  (   m_eGapType == SM_GT_VERTEX_EDGE)   ? m_pVertexuse->GetEdgeuse()->GetEdge()
//           : (   m_eGapType == SM_GT_EDGE_UVTRIMCURVE
//              || m_eGapType == SM_GT_EDGE_FACE)     ? m_pEdgeuse->GetEdge()
//           : (   m_eGapType == SM_GT_VERTEX_UVTRIMCURVE
//              || m_eGapType == SM_GT_VERTEX_FACE)   ? (SmEdge *)NULL
//           : (SmEdge *)NULL) ;
//  
//  } // end SmGap::GetEdge
//  
//  /*******************************************************************//**
//  PURPOSE: Get Edgeuse when gap connects to one else return NULL
//  
//  NOTES:
//  ***********************************************************************/
//  SmEdgeuse * SmGap::GetEdgeuse()    const
//  {
//    return(  (   m_eGapType == SM_GT_VERTEX_UVTRIMCURVE) ? m_pVertexuse->GetEdgeuse()
//           : (   m_eGapType == SM_GT_EDGE_UVTRIMCURVE)   ? m_pEdgeuse
//           : (   m_eGapType == SM_GT_VERTEX_EDGE 
//              || m_eGapType == SM_GT_VERTEX_FACE
//              || m_eGapType == SM_GT_EDGE_FACE)      ? (SmEdgeuse *)NULL
//           : (SmEdgeuse *)NULL) ;
//  
//  } // end SmGap::GetEdge
//  
//  /*******************************************************************//**
//  PURPOSE: Get Face when gap connects to one else return NULL
//  
//  NOTES:
//  ***********************************************************************/
//  SmFace * SmGap::GetFace()    const
//  {
//    return(  (   m_eGapType == SM_GT_VERTEX_FACE) ? m_pVertexuse->GetFaceuse()->GetFace()
//           : (   m_eGapType == SM_GT_EDGE_FACE)   ? m_pEdgeuse->GetFace()
//           : (   m_eGapType == SM_GT_VERTEX_EDGE
//              || m_eGapType == SM_GT_EDGE_UVTRIMCURVE
//              || m_eGapType == SM_GT_VERTEX_UVTRIMCURVE) ? (SmFace *)NULL
//           : (SmFace *)NULL) ;
//  
//  } // end SmGap::GetFace
//  
//  /*******************************************************************//**
//  PURPOSE: Get Edge->Curve Param when gap connects to an edge 
//             else return SM_UNDEF_DOUBLE
//  
//  NOTES:
//  ***********************************************************************/
//  double SmGap::GetEdgeT() const
//  {
//    return(m_dEdgeT) ;
//  
//  } // end SmGap::GetEdgeT
//  
//  /*******************************************************************//**
//  PURPOSE: Get Edge->UVTrimCurve Param when gap connects to an 
//    SmCrvOnSurf(UVTrimCurve,Surf) else return SM_UNDEF_DOUBLE
//  
//  NOTES:
//  ***********************************************************************/
//  double SmGap::GetUVTrimCurveT() const
//  {
//    return(m_dUVTrimCurveT) ;
//  
//  } // end SmGap::GetUVTrimCurveT
//  
//  /*******************************************************************//**
//  PURPOSE: Get Face->Surface Param when gap connects to an face else return SM_UNDEF_DOUBLE
//  
//  NOTES:
//  ***********************************************************************/
//  const SmPoint2d SmGap::GetFaceUV() const
//  {
//    return( m_sFaceUV ) ;
//  
//  } // end SmGap::GetFaceUV
//  
//  /*******************************************************************//**
//  PURPOSE: Set a Vertex/Edge Gap
//  
//  NOTES:
//  ***********************************************************************/
//  void SmGap::Set
//    (double       dGap,         // in : 3d Gap
//     SmVertexuse *pVertexuse,   // in : connects vertex to edge
//     double       dEdgeT,       // in : edge param at gap
//     SmGapType    eGapType)     // in : one of SM_GT_VERTEX_EDGE or SM_GT_VERTEX_UVTRIMCURVE
//                                //      default:[SM_GT_VERTEX_EDGE]
//  {
//    if(   eGapType != SM_GT_VERTEX_EDGE 
//       && eGapType != SM_GT_VERTEX_UVTRIMCURVE) 
//     {
//       WARN(_T("Bad Gap type given as input - being changed to SM_GT_VERTEX_EDGE")) ;
//       eGapType = SM_GT_VERTEX_EDGE ;
//     } 
//  
//    m_eGapType      = eGapType ;
//    m_dGap3d        = dGap ;
//    m_pEdgeuse      = NULL ;
//    m_pVertexuse    = pVertexuse ;
//    m_dEdgeT        = dEdgeT ;
//    m_dUVTrimCurveT = SM_UNDEF_DOUBLE ;
//    m_sFaceUV.SetUninitialized() ;
//  
//  } // end SmGap::Set Vertex/Edge gap
//  
//  /*******************************************************************//**
//  PURPOSE: Set a Vertex/Face Gap
//  
//  NOTES:
//  ***********************************************************************/
//  void SmGap::Set
//    (double       dGap,         // in : 3d Gap
//     SmVertexuse *pVertexuse,   // in : connects vertex to face
//     SmPoint2d   &rFaceUV)      // in : face param at gap
//  {
//    m_eGapType      = SM_GT_VERTEX_FACE ;
//    m_dGap3d        = dGap ;
//    m_pEdgeuse      = NULL ;
//    m_pVertexuse    = pVertexuse ;
//    m_dEdgeT        = SM_UNDEF_DOUBLE ;
//    m_dUVTrimCurveT = SM_UNDEF_DOUBLE ;
//    m_sFaceUV       = rFaceUV ;
//  
//  } // end SmGap::Set Vertex/Face gap
//  
//  /*******************************************************************//**
//  PURPOSE: Set a Edge/Edgeuse Gap
//  
//  NOTES:
//  ***********************************************************************/
//  void SmGap::Set
//    (double       dGap,          // in : 3d Gap
//     SmEdgeuse   *pEdgeuse,      // in : connects edge to face
//     double       dEdgeT,        // in : edge->Curve param at gap
//     double       dUVTrimCurveT) // in : edge->UVTrimCurve param at gap
//  {
//    m_eGapType      = SM_GT_EDGE_UVTRIMCURVE ;
//    m_dGap3d        = dGap ;
//    m_pEdgeuse      = pEdgeuse ;
//    m_pVertexuse    = NULL ;
//    m_dEdgeT        = dEdgeT ;   
//    m_dUVTrimCurveT = dUVTrimCurveT ;
//    m_sFaceUV.SetUninitialized() ;
//  
//  } // end SmGap::Set Vertex/Face gap
//  
//  /*******************************************************************//**
//  PURPOSE: Set a Edge/Face Gap
//  
//  NOTES:
//  ***********************************************************************/
//  void SmGap::Set
//    (double       dGap,         // in : 3d Gap
//     SmEdgeuse   *pEdgeuse,     // in : connects edge to face
//     double       dEdgeT,       // in : edge param at gap
//     SmPoint2d   &rFaceUV)      // in : face param at gap
//  {
//    m_eGapType      = SM_GT_EDGE_FACE ;
//    m_dGap3d        = dGap ;
//    m_pEdgeuse      = pEdgeuse ;
//    m_pVertexuse    = NULL ;
//    m_dEdgeT        = dEdgeT ;
//    m_dUVTrimCurveT = SM_UNDEF_DOUBLE ;
//    m_sFaceUV       = rFaceUV ;
//  
//  } // end SmGap::Set Vertex/Face gap
//  
// end OBSOLETE2
