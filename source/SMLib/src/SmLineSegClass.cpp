// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmLineSegClass.cpp 
* PURPOSE: Implementation of curve classification methods.
**********************************************************************/

#include "StdAfx.h"

#include <SmCacheMgr.h>
#include <SmGraphicsExtern.h>
#include <SmLineSegClass.h>
#include <SmPoly.h>
#include <SmGeomUtility.h>

//#define SM_VALIDATE_TOPOLOGY 1

// obsolete:
//  /*******************************************************************//**
//  PURPOSE: Get the tolerance of the object in the point class.
//  
//  NOTES: 
//  ***********************************************************************/
//  SmZoneTol3d SmPolyPointClassification::GetTolerance() const
//  {
//    switch (m_ePolyPointClass) 
//      {
//        case SM_PPC_UNKNOWN: return 0.0; 
//        case SM_PPC_VERTEX : { SmPolyVertex *pV = (SmPolyVertex*)m_pObject;
//                               return pV->GetTolerance();
//                             }
//        case SM_PPC_EDGE   : { SmPolyEdge *pE = (SmPolyEdge*)m_pObject;
//                               return pE->GetTolerance();
//                             }
//        case SM_PPC_FACE   : { SmPolyFace *pF = (SmPolyFace*)m_pObject;
//                               return pF->GetTolerance();
//                             }
//        case SM_PPC_REGION : return 0.0;
//        case SM_PPC_POINT  : return 0.0;
//            break;
//      }
//  
//    return 0.0;
//  } // end SmPolyPointClassification::GetTolerance

/*******************************************************************//**
PURPOSE: returns the best PolyPointClassification to use when
         the two PolyPointClassifications, this and crPointClass, are 
         going to be combined. 

NOTES: Chooses the PolyPointClassification with the best classification as:
       PolyVertex with smallest deviation, when combining two PolyVertices
       PolyVertex when combining a PolyVertex with PolyEdge, PolyFace, UNKNOWN
       PolyEdge   when combining a PolyEdge with a PolyFace or UNKNOWN
       PolyFace   when combining a PolyFace with an UNKNOWN.

       returns *this when the two classifications are the same
       except for two point classifications when the classification
         with the smallest deviation is returned.
***********************************************************************/
SmPolyPointClassification SmPolyPointClassification::Combine
 (const SmPolyPointClassification & crPointClass) 
 const
{
  SmPolyPointClassification sPC;
  switch (m_ePolyPointClass) 
    {
      case SM_PPC_UNKNOWN: sPC = crPointClass;
                           return sPC;

      case SM_PPC_VERTEX : sPC = (   (crPointClass.m_ePolyPointClass == SM_PPC_VERTEX)
                                  && (crPointClass.m_pObject         != m_pObject)
                                  && (crPointClass.m_dDeviation       < m_dDeviation) )
                                 ? crPointClass
                                 : *this ;
                           return sPC;

      case SM_PPC_EDGE   : sPC =  (crPointClass.m_ePolyPointClass == SM_PPC_VERTEX)
                                 ? crPointClass
                                 : (crPointClass.m_dDeviation      < m_dDeviation)
                                   ? crPointClass
                                   : *this ;
                           return sPC;

      case SM_PPC_FACE   : sPC = (   crPointClass.m_ePolyPointClass == SM_PPC_VERTEX
                                  || crPointClass.m_ePolyPointClass == SM_PPC_EDGE)
                                 ? crPointClass
                                 : *this ;
                           return sPC;
      case SM_PPC_REGION :
      case SM_PPC_POINT  : break;
          
    } // end switch on PolyPoint classification type

  // all done
  SE(SM_ERR);
  return *this;

} // end SmPolyPointClassification::Combine

/*******************************************************************//**
PURPOSE: Merge a point class into the topology and create a vertex
   if one is not already existing.  It may update the tolerances of 
   existing vertices.

NOTES: 
***********************************************************************/
SmStatus SmPolyPointClassification::Merge
 (const SmPoint3d       & sVertexPoint, // in : 3d point for new or existing vertex
  double                  dTolerance,   // in : tolerance to assign to vertex
  SmTArray<SmPolyEdge*> * pOptNewEdges) // out:
{
  if (GetPointClass() == SM_PPC_VERTEX) 
    {
      // Almost no work to do there is already a vertex here.
      // Just have to update the tolerance on the vertex.
      SmPolyVertex *pV = (SmPolyVertex*)GetObject();
      double dDistance = sVertexPoint.DistanceBetween(pV->GetPoint());
      if (dDistance > dTolerance/2.0) 
        { dTolerance = dDistance * 2.0; }

      if (pV->GetTolerance() < dTolerance) 
        {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe) 
            {
              smgfx_Erase();
              smgfx_SetLook(1,2, 1,0,0); sVertexPoint.Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,4, 0,0,1); pV->GetPoint().Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
          pV->SetTolerance(dTolerance);
        }
      
      // all done
      return SM_SUCCESS;

    } // end GetPointClass() == SM_PPC_VERTEX check

  // Edge
  if (GetPointClass() == SM_PPC_EDGE) 
    {
      // Split the edge to make a vertex.
      SmPolyEdge   * pEdgeToSplit = (SmPolyEdge*)GetObject();
      SmPolyVertex * pStartV      = pEdgeToSplit->GetStartPolyVertex();
      if(pStartV->GetPoint().DistanceBetween(sVertexPoint) < pStartV->GetTolerance()) 
        {
          // Near Start
          m_ePolyPointClass = SM_PPC_VERTEX;
          m_pObject = pStartV;
          return SM_SUCCESS;
        }

      SmPolyVertex *pEndV = pEdgeToSplit->GetEndPolyVertex();
      if (pEndV->GetPoint().DistanceBetween(sVertexPoint) < pEndV->GetTolerance()) 
        {
          // Near End
          m_ePolyPointClass = SM_PPC_VERTEX;
          m_pObject = pEndV;
          return SM_SUCCESS;
        }

      SmPolyFace   * pF = pEdgeToSplit->GetPolyFace();
      SmPolyEdge   * pNewE = NULL;
      SmPolyVertex * pNewV = NULL;
      SER(pF->MakeVertexSplitPolyEdge(pEdgeToSplit, // in : PolyEdge to split
                                      sVertexPoint, // in : Point split location
                                      pNewE,        // out: new PolyEdge (and new radial partners)
                                      pNewV));      // out: new PolyVertex
      if (pNewE == NULL) 
        { return SM_SUCCESS; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          smgfx_Erase();            
          smgfx_SetLook(2,4, 0,1,0); pEdgeToSplit->Draw(); sm_GraphicsLoop();
          if (pNewE) { smgfx_SetLook(2,4, 0,1,1); pNewE->Draw(); sm_GraphicsLoop(); }
          if (pNewV) { smgfx_SetLook(2,4, 1,0,0); pNewV->Draw(); sm_GraphicsLoop(); }
          smgfx_SetLook(1,2, 0,0,0); pF->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1); pF->GetPolyBrep()->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // when asked - remember the split edges
      if (pOptNewEdges) 
        {
          pOptNewEdges->Add(pEdgeToSplit);
          pOptNewEdges->Add(pNewE);
        }

      this->SetClassObject( SM_PPC_VERTEX, pNewV );
      double dDistance = sVertexPoint.DistanceBetween(pNewV->GetPoint());
      if (dDistance > dTolerance/2.0) 
        { dTolerance = dDistance * 2.0; }

      pNewV->SetTolerance( smos_Max( dTolerance, (double)pEdgeToSplit->GetTolerance() ));

#ifdef SM_VALIDATE_TOPOLOGY
      pEdgeToSplit->GetPolyBrep()->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY
      
      // all done
      return SM_SUCCESS;

    } // end GetPointClass() == SM_PPC_EDGE check

  // Merge point into face
  if (GetPointClass() == SM_PPC_FACE) 
    {
      // Put a single vertex loop into the middle of the  face.
      SmPolyFace   * pF = (SmPolyFace*)GetObject();
      SmPolyLoop   * pNewL = NULL;
      SmPolyVertex * pNewV = NULL;
      SmPolyEdge   * pNewE = NULL;
      SER(pF->AddSinglePolyVertexLoop(sVertexPoint,
                                      dTolerance,
                                      pNewV,
                                      pNewE,
                                      pNewL));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          smgfx_Erase();            
          if (pNewE) { smgfx_SetLook(2,4, 0,1,1); pNewE->Draw(); sm_GraphicsLoop(); }
          if (pNewV) { smgfx_SetLook(2,4, 1,0,0); pNewV->Draw(); sm_GraphicsLoop(); }
          smgfx_SetLook(1,2, 0,0,0); pF->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,1); pF->GetPolyBrep()->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      SetClassObject(SM_PPC_VERTEX, pNewV);
      pNewV->SetTolerance( smos_Max( dTolerance, (double)pF->GetTolerance() ));

#ifdef SM_VALIDATE_TOPOLOGY
      pF->GetPolyBrep()->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY
      
      // all done
      return SM_SUCCESS;

    } // end GetPointClass() == SM_PPC_FACE check

  // arrive here for unhandled PointClassifications - an error
  SE(SM_ERR);
  return SM_ERR;

} // end SmPolyPointClassification::Merge

/*******************************************************************//**
PURPOSE: Constructor for the SmLineSegClassification object.

NOTES: 
***********************************************************************/
SmLineSegClassification::SmLineSegClassification
 (const SmPoint3d   & crLineStart,
  const SmVector3d  & crLineVec,
  SmZoneTol3d         sSrcZoneTol3d,
  ULONG               nDataSize,
  SmLineSegInterval * pOptBorrowedData)
  : SmObject(), m_cLineStart(crLineStart), m_cLineVec(crLineVec),
    m_sSrcZoneTol3d(sSrcZoneTol3d)
{
  m_cpContext = NULL;
  if (pOptBorrowedData) 
    {
      m_bIsBorrowed = TRUE;
      m_pData = pOptBorrowedData;
      m_lMaxSize = nDataSize;
      m_lSize = 0;
    }
  else 
    {
      m_bIsBorrowed = FALSE;
      m_pData = NULL;
      m_lSize = m_lMaxSize = 0;
      SetSize(16); 
      m_lSize = 1; 
    }
  m_pData[0] = SmLineSegInterval(SmExtent1d(0.0,1.0));
  m_lSize = 1;

  m_pData[0].m_vStart.SetSrcZoneTol3d(sSrcZoneTol3d) ;
  m_pData[0].m_vMid.  SetSrcZoneTol3d(sSrcZoneTol3d) ;
  m_pData[0].m_vEnd.  SetSrcZoneTol3d(sSrcZoneTol3d) ;

} // end SmLineSegClassification::SmLineSegClassification constructor

/*******************************************************************//**
PURPOSE: Destructor for the SmLineSegClassification object

NOTES: 
***********************************************************************/
SmLineSegClassification::~SmLineSegClassification()
{
    if (m_pData && !m_bIsBorrowed) { smos_Free(m_pData); m_pData = NULL ; }

} // end SmLineSegClassification::~SmLineSegClassification destructor

/*******************************************************************//**
PURPOSE: Determine if two corresponding intervals are coincident maybe.
    
NOTES: The curve classes need to be homogenized prior to this.

GWC NOTES ---       CoincidentMaybe cases were generated as bad output from the
      intersectors.  The intersectors have been modified to stop generating
      coincident maybe cases.  So from now on, any time AreCoincidentMaybe() 
      returns TRUE should be considered a bug either in the intersector
      or in the logic of AreCoincidentMaybe().

***********************************************************************/
SmBoolean SmLineSegClassification::AreCoincidentMaybe(const SmLineSegClassification & crOther,
                                             ULONG lIntervalIndex,
                                             SmPolyIntervalPosition & rePositionOfCoincidentMaybe) const
{
    rePositionOfCoincidentMaybe = SM_PIP_INSIDE;

    const SmLineSegInterval & rIvl = (*this)[lIntervalIndex];
    const SmLineSegInterval & rOIvl = crOther[lIntervalIndex];
    const SmPolyPointClassification & rMid = rIvl.m_vMid;
    const SmPolyPointClassification & rOMid = rOIvl.m_vMid;
    const SmPolyPointClassification & rStart = rIvl.m_vStart;
    const SmPolyPointClassification & rOStart = rOIvl.m_vStart;
    const SmPolyPointClassification & rEnd = rIvl.m_vEnd;
    const SmPolyPointClassification & rOEnd = rOIvl.m_vEnd;

    // If either LSC is completely unset, then we're not CoincidentMaybe. [B373]
    if (    rMid.  GetPointClass() == SM_PPC_UNKNOWN
         && rStart.GetPointClass() == SM_PPC_UNKNOWN
         && rEnd.  GetPointClass() == SM_PPC_UNKNOWN )
      { return FALSE; }
    if (    rOMid.  GetPointClass() == SM_PPC_UNKNOWN
         && rOStart.GetPointClass() == SM_PPC_UNKNOWN
         && rOEnd.  GetPointClass() == SM_PPC_UNKNOWN )
      { return FALSE; }

    // If the mid point class is an EDGE and the end is an EDGE then
    // we need to look at the other interval to see if three possible
    // conditions exist.  The first condition is that the mid and start
    // of the other one are both EDGE.  The second condition is that the
    // mid and start of the other one are FACE.  The third condition is
    // that the mid and end are both REGION.
    if (rMid.GetPointClass() == SM_PPC_EDGE) {
        if (rStart.GetPointClass() == SM_PPC_EDGE) {
            SmPolyEdge *pEdge = (SmPolyEdge*)rStart.GetObject();
            double dEdgeLeng = pEdge->Length();
            double dCMLeng = (rIvl.m_vInterval.GetMax()-rIvl.m_vInterval.GetMin()) * m_cLineVec.Length();
            if (dCMLeng < dEdgeLeng/10.0 && dCMLeng < (pEdge->GetTolerance()+GetTolerance())*10.0) {
            if (((rOMid.GetPointClass() == rOStart.GetPointClass()) &&
                (rOStart.GetPointClass() == SM_PPC_EDGE ||
                rOStart.GetPointClass() == SM_PPC_FACE ||
                rOStart.GetPointClass() == SM_PPC_REGION ||
                rOStart.GetPointClass() == SM_PPC_UNKNOWN)) ||
                (rOStart.GetPointClass() == SM_PPC_FACE ||
                rOStart.GetPointClass() == SM_PPC_UNKNOWN) ) {
                rePositionOfCoincidentMaybe = SM_PIP_START;
                SM_DBG_WARN(_T("Found a CoincidentMaybe case - a Bug in intersection return or AreCoincidentMaybe() logic")) ;
                return TRUE;
            }
            }
        }
    }

    if (rOMid.GetPointClass() == SM_PPC_EDGE) 
      {
        if (rOStart.GetPointClass() == SM_PPC_EDGE) 
          {
            SmPolyEdge *pEdge = (SmPolyEdge*)rOStart.GetObject();
            double dEdgeLeng = pEdge->Length();
            double dCMLeng = (rIvl.m_vInterval.GetMax()-rIvl.m_vInterval.GetMin()) * m_cLineVec.Length();

            SmZoneTol3d                 sObjZoneTol3d = SmTol::GetZoneTol3d(pEdge) ;
            SmZoneTol3d                 sSrcZoneTol3d = SmTol::GetSrcZoneTol3d(this) ;
            SM_OLDTOL_LINE SmXSectTol3d sXSectTol3d   = 10 * SmTol::GetXSectTol3d(sSrcZoneTol3d, sObjZoneTol3d) ; 
            if (dCMLeng < dEdgeLeng/10.0 && dCMLeng < sXSectTol3d) 
              {
                if (((rMid.GetPointClass() == rStart.GetPointClass()) &&
                    (rStart.GetPointClass() == SM_PPC_EDGE ||
                    rStart.GetPointClass() == SM_PPC_FACE ||
                    rStart.GetPointClass() == SM_PPC_REGION ||
                    rStart.GetPointClass() == SM_PPC_UNKNOWN)) ||
                    (rMid.GetPointClass() == SM_PPC_FACE &&
                    rStart.GetPointClass() == SM_PPC_UNKNOWN) ) 
                  {
                    rePositionOfCoincidentMaybe = SM_PIP_START;
                    SM_DBG_WARN(_T("Found a CoincidentMaybe case - a Bug in intersection return or AreCoincidentMaybe() logic")) ;
                    return TRUE;
                  }
              }
          }
      }

    if (rMid.GetPointClass() == SM_PPC_EDGE) {
        if (rEnd.GetPointClass() == SM_PPC_EDGE) {
            SmPolyEdge *pEdge = (SmPolyEdge*)rEnd.GetObject();
            double dEdgeLeng = pEdge->Length();
            double dCMLeng = (rIvl.m_vInterval.GetMax()-rIvl.m_vInterval.GetMin()) * m_cLineVec.Length();
            if (dCMLeng < dEdgeLeng/10.0 && dCMLeng < (pEdge->GetTolerance()+GetTolerance())*10.0) {
            if (((rOMid.GetPointClass() == rOEnd.GetPointClass()) &&
                (rOEnd.GetPointClass() == SM_PPC_EDGE ||
                rOEnd.GetPointClass() == SM_PPC_FACE ||
                rOEnd.GetPointClass() == SM_PPC_REGION ||
                rOEnd.GetPointClass() == SM_PPC_UNKNOWN)) ||
                (rOMid.GetPointClass() == SM_PPC_FACE &&
                rOEnd.GetPointClass() == SM_PPC_UNKNOWN) ) {
                rePositionOfCoincidentMaybe = SM_PIP_END;
                SM_DBG_WARN(_T("Found a CoincidentMaybe case - a Bug in intersection return or AreCoincidentMaybe() logic")) ;
                return TRUE;
            }
            }
        }
    }

    if (rOMid.GetPointClass() == SM_PPC_EDGE) {
        if (rOEnd.GetPointClass() == SM_PPC_EDGE) {
            SmPolyEdge *pEdge = (SmPolyEdge*)rOEnd.GetObject();
            double dEdgeLeng = pEdge->Length();
            double dCMLeng = (rIvl.m_vInterval.GetMax()-rIvl.m_vInterval.GetMin()) * m_cLineVec.Length();
            if (dCMLeng < dEdgeLeng/10.0 && dCMLeng < (pEdge->GetTolerance()+GetTolerance())*10.0) {
            if (((rMid.GetPointClass() == rEnd.GetPointClass()) &&
                (rEnd.GetPointClass() == SM_PPC_EDGE ||
                rEnd.GetPointClass() == SM_PPC_FACE ||
                rEnd.GetPointClass() == SM_PPC_REGION ||
                rEnd.GetPointClass() == SM_PPC_UNKNOWN)) ||
                (rMid.GetPointClass() == SM_PPC_FACE &&
                rEnd.GetPointClass() == SM_PPC_UNKNOWN) ) {
                rePositionOfCoincidentMaybe = SM_PIP_END;
                SM_DBG_WARN(_T("Found a CoincidentMaybe case - a Bug in intersection return or AreCoincidentMaybe() logic")) ;
                return TRUE;
            }
            }
        }
    }

    // Take care of case where we pass very near to a vertex but
    // do not really intersect it cleanly.
//          if (rMid.GetPointClass() == SM_PPC_FACE || rMid.GetPointClass() == SM_PPC_UNKNOWN) {
//              if (rStart.GetPointClass() == SM_PPC_VERTEX && 
//                  rStart.GetDeviation() > m_sSrcZoneTol3d / 100.0) {
//                  if ((rOMid.GetPointClass() == SM_PPC_FACE || rOMid.GetPointClass() == SM_PPC_UNKNOWN) &&
//                      rOStart.GetDeviation() > m_sSrcZoneTol3d / 100.00) {
//                      // If we have a small segment
//                      double dCMLeng = (rIvl.m_vInterval.GetMax()-rIvl.m_vInterval.GetMin()) * m_cLineVec.Length();
//                      if (dCMLeng < m_sSrcZoneTol3d * 10.0) {
//      //                    SE(SM_ERR);
//      //                    rePositionOfCoincidentMaybe = SM_PIP_START;
//      //                    return TRUE;
//                      }
//                  }
//              }
//          }

//          if (rMid.GetPointClass() == SM_PPC_FACE || rMid.GetPointClass() == SM_PPC_UNKNOWN) {
//              if (rEnd.GetPointClass() == SM_PPC_VERTEX && 
//                  rEnd.GetDeviation() > m_sSrcZoneTol3d / 100.0) {
//                  if ((rOMid.GetPointClass() == SM_PPC_FACE || rOMid.GetPointClass() == SM_PPC_UNKNOWN) &&
//                      rOEnd.GetDeviation() > m_sSrcZoneTol3d / 100.00) {
//                      // If we have a small segment
//                      double dCMLeng = (rIvl.m_vInterval.GetMax()-rIvl.m_vInterval.GetMin()) * m_cLineVec.Length();
//                      if (dCMLeng < m_sSrcZoneTol3d * 10.0) {
//      //                    SE(SM_ERR);
//      //                    rePositionOfCoincidentMaybe = SM_PIP_END;
//      //                    return TRUE;
//                      }
//                  }
//              }
//          }

    return FALSE;

} // end SmLineSegClassification::AreCoincidentMaybe

/*******************************************************************//**
PURPOSE: This method takes two homogenized curve classifications
    and does a cleanup and validation of them.  It will try to take
    care of the dreaded coincident maybe problem.  This problem occurs
    when there are tangencies or near coincidences.  One end of an interval
    is at a vertex or something and it is coincident with an edge at
    the other end but it has a larger deviation.  

NOTES: 
***********************************************************************/
SmStatus SmLineSegClassification::CleanupAndValidate(SmLineSegClassification & rOther)
{
#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE ;
    if (bDebugMe) {
        Dump();
        rOther.Dump();
    }
#endif

    // Next cleanup process is to look for coincident maybe cases
    // and remove them.
    SmBoolean bDone = FALSE;
    while (!bDone) {
        bDone = TRUE;
        for (ULONG ii=0; ii<GetSize(); ii++) {
            SmPolyIntervalPosition eEndOfCM;

            // CoincidentMaybe cases were generated as bad output from the
            // intersectors.  The intersectors have been modified to stop generating
            // coincident maybe cases.  So from now on, any time AreCoincidentMaybe() 
            // returns TRUE should be considered a bug either in the intersector
            // or in the logic of AreCoincidentMaybe().
            if (AreCoincidentMaybe(rOther,ii,eEndOfCM)) {
                ConnectIntervals(ii,eEndOfCM,TRUE);
                rOther.ConnectIntervals(ii,eEndOfCM,TRUE);
                bDone = FALSE;
                break;
            }
        }
    }

    // Now do geometric tests for coincidence.
    // If Start or End classifies to an Edge, and the Mid and/or the other End
    // are within tolerance of that edge, then change their classifications to
    // that Edge.  [B654]
    // This can be done independently for both LineSegClasses.
    SER( this ->CheckIntervalCoincidence() );
    SER( rOther.CheckIntervalCoincidence() );

    // Someday we should add some checks here to validate relationships
    // and error out if we have something invalid.

#ifdef SM_DEBUG_CODE
    if (bDebugMe) {
        this ->Dump();
        rOther.Dump();
    }
#endif
    return SM_SUCCESS;

} // end SmLineSegClassification::CleanupAndValidate

/*******************************************************************//**
PURPOSE: Check the intervals of this CurveClassification for coincidence with an Edge.

NOTES: 1. If Start or End is classified as Edge, then check Mid and the other end
           for coincidence with the same edge.  [B654]
***********************************************************************/
SmStatus SmLineSegClassification::CheckIntervalCoincidence( SmBoolean * pbWasModified ) // out: TRUE = One or more Ivl classifications were changed, FALSE=No Changes
{
  // init output
  if ( pbWasModified != NULL ) { *pbWasModified = FALSE; }

  // Locals
  double     dLineSegParam, dEdgeParam, dDist;
  SmPoint3d  sLineSegPoint, sEdgePoint, sOtherPoint;
  SmVector3d sLineSegVec,   sEdgeVec;
  SmPoint3d  sCurvePt;
  SmStatus   eStat;

  SmZoneTol3d sSrcZoneTol3d = SmTol::GetSrcZoneTol3d(this) ; 

  this->GetLineSeg( sLineSegPoint, sLineSegVec );

  ULONG lNumIvls = this->GetSize();

  // for every interval
  for ( ULONG ii=0; ii<lNumIvls; ii++ )
    {
      // locals
      SmLineSegInterval & rIvl   = (*this)[ii];
      SmPolyPointClass  & rStart = rIvl.m_vStart;
      SmPolyPointClass  & rMid   = rIvl.m_vMid;
      SmPolyPointClass  & rEnd   = rIvl.m_vEnd;

      // Check the Start.
      if ( rStart.GetPointClass() == SM_PPC_EDGE )
        {
          if (   rEnd.GetPointClass() != SM_PPC_EDGE
              && rEnd.GetPointClass() != SM_PPC_VERTEX ) // (Don't yank it off of a Vertex.)
            {
              // See whether End is within tolerance of the Start's Edge.
              dLineSegParam = rIvl.m_vInterval.GetMax();
              sOtherPoint = sLineSegPoint + dLineSegParam * sLineSegVec;

              SmObject *pObj = rStart.GetObject();
              SmPolyEdge *pPolyEdge = SM_CAST_PTR( SmPolyEdge, pObj );
              if ( pPolyEdge == NULL )
                { continue; }
              pPolyEdge->GetLine( sEdgePoint, sEdgeVec );

              eStat = smgu_LinePointDistance( sEdgePoint, sEdgeVec, sOtherPoint, dDist, &dEdgeParam );

              if ( eStat == SM_SUCCESS && dDist < sSrcZoneTol3d )
                {
                  rEnd.SetClassObject( SM_PPC_EDGE, rStart.GetObject() );
                  rEnd.SetTParam     ( dEdgeParam );
                  rEnd.SetDeviation  ( dDist );
                  rMid.SetClassObject( SM_PPC_EDGE, rStart.GetObject() );  // Mid must be within tol as well.

                  // Also set rEnd's mate, if present.
                  // Get the mate.  Note, GetPointClassMateByIndex() returns the End of the lower-index
                  // interval, but here we need the Start of the higher-index interval.
                  // We shoud write a convenience routine, or add an argument to GetPointClassMateByIndex().
                  //cbi SmPolyPointClassification * pMate = this->GetPointClassMateByIndex( ii+1 );
                  if ( ii < GetSize()-1 )
                    {
                      SmPolyPointClassification * pMate = &(*this)[ii+1].m_vStart;
                      pMate->SetClassObject( SM_PPC_EDGE, rStart.GetObject() );
                      pMate->SetTParam     ( dEdgeParam );
                      pMate->SetDeviation  ( dDist );
                    }

                  if ( pbWasModified != NULL ) { *pbWasModified = TRUE; }

                  // At this point, we're done with this interval.
                  continue;
                }
            }
        } // end if Start on an Edge

      // Check the End.
      if ( rEnd.GetPointClass() == SM_PPC_EDGE )
        {
          if (   rStart.GetPointClass() != SM_PPC_EDGE
              && rStart.GetPointClass() != SM_PPC_VERTEX ) // (Don't yank it off of a Vertex.)
            {
              // See whether Start is within tolerance of the End's Edge.
              dLineSegParam = rIvl.m_vInterval.GetMin();
              sOtherPoint = sLineSegPoint + dLineSegParam * sLineSegVec;

              SmObject *pObj = rEnd.GetObject();
              SmPolyEdge *pPolyEdge = SM_CAST_PTR( SmPolyEdge, pObj );
              if ( pPolyEdge == NULL )
                { continue; }
              pPolyEdge->GetLine( sEdgePoint, sEdgeVec );

              eStat = smgu_LinePointDistance( sEdgePoint, sEdgeVec, sOtherPoint, dDist, &dEdgeParam );

              if ( eStat == SM_SUCCESS && dDist < sSrcZoneTol3d )
                {
                  rStart.SetClassObject( SM_PPC_EDGE, rEnd.GetObject() );
                  rStart.SetTParam     ( dEdgeParam );
                  rStart.SetDeviation  ( dDist );
                  rMid.  SetClassObject( SM_PPC_EDGE, rEnd.GetObject() );  // Mid must be within tol as well.

                  // Also set rStart's mate, if present.
                  SmPolyPointClassification * pMate = this->GetPointClassMateByIndex( ii );
                  if(pMate == NULL) continue;
                  pMate->SetClassObject( SM_PPC_EDGE, rEnd.GetObject() );
                  pMate->SetTParam     ( dEdgeParam );
                  pMate->SetDeviation  ( dDist );

                  if ( pbWasModified != NULL ) { *pbWasModified = TRUE; }
                }
            }
        } // end if End on an Edge

    } // end for each interval

  return SM_SUCCESS;

} // end SmLineSegClassification::CheckIntervalCoincidence

/*******************************************************************//**
PURPOSE: Return the SmPolyPointClass at the given index.
   Also return its parameter value.

NOTES: 
***********************************************************************/
SmPolyPointClassification * SmLineSegClassification::GetPointClassByIndex
 (ULONG    lPointClassIndx, // in :
  double * pOptParameter)    // out:
 const
{
  SM_ASSERT(lPointClassIndx <= GetSize());

  if (lPointClassIndx == GetSize()) 
    {
      SmLineSegInterval & rLineSegIvl = (*this)[GetSize()-1];
      if(pOptParameter) { *pOptParameter = rLineSegIvl.m_vInterval.GetMax() ; }
      return &rLineSegIvl.m_vEnd;
    }

  SmLineSegInterval & rLineSegIvl = (*this)[lPointClassIndx];
  if(pOptParameter) { *pOptParameter = rLineSegIvl.m_vInterval.GetMin() ; }
  return &rLineSegIvl.m_vStart;

} // end SmLineSegClassification::GetPointClassByIndex

/*******************************************************************//**
PURPOSE: Find the interval of the given parameter.  

NOTES: 
***********************************************************************/
SmStatus SmLineSegClassification::FindInterval
 (double                   dParameter,         // in : 
  double                   dLastParameter,     // in : 
  SmZoneTol3d              sParamObjZoneTol3d, // in : 
  ULONG                  & rlIndex,            // out: 
  double                 & rdEndDist3d,        // out: 
  SmPolyIntervalPosition & rePosition)         // out: 
 const
{
  rdEndDist3d = 0.0;
  ULONG ii;
  for (ii=0; ii<GetSize(); ii++) 
    {
      const SmLineSegInterval & rTestIvl = (*this)[ii];
      double dTMin = rTestIvl.m_vInterval.GetMin();
      double dTMax = rTestIvl.m_vInterval.GetMax();

      if (dParameter > dTMax + SM_EFF_ZERO) 
        { continue; }

      if (dParameter > dTMax - SM_EFF_ZERO) 
        {
          rlIndex = ii;
          rePosition = SM_PIP_END;
          return SM_SUCCESS;
        }

      if (//dParameter > dTMin &&
          dParameter < dTMin + SM_EFF_ZERO) 
        {
          rlIndex = ii;
          rePosition = SM_PIP_START;
          return SM_SUCCESS;
        }

      rlIndex = ii;
      // Test to see if tolerance makes it fall onto one of ends
      // of the curves.

      SmBoolean bFound = FALSE;

      // If it falls onto a vertex and it is the vertex's edge skip it.
      if (rTestIvl.m_vStart.GetPointClass() == SM_PPC_VERTEX) 
        {
          SmPolyVertex * pV    = (SmPolyVertex*)rTestIvl.m_vStart.GetObject(); NER(pV);
          SmPoint3d      sPnt  = m_cLineStart + dParameter * m_cLineVec;
          double         dDist = pV->GetPoint().DistanceBetween(sPnt);
          double         dTol  = smos_Max( (double)pV->GetTolerance(), (double)sParamObjZoneTol3d );   //cbiTol dXSectTol3d is already a sum... <--
          if (dDist < dTol) 
            {
              rdEndDist3d = smos_Fabs(dParameter-dTMin)*m_cLineVec.Length();
              rePosition  = SM_PIP_START;
              bFound      = TRUE;
            }
        }
        
      if (!bFound && rTestIvl.m_vEnd.GetPointClass() == SM_PPC_VERTEX) 
        {
          SmPolyVertex * pV    = (SmPolyVertex*)rTestIvl.m_vEnd.GetObject(); NER(pV);
          SmPoint3d      sPnt  = m_cLineStart + dParameter * m_cLineVec;
          double         dDist = pV->GetPoint().DistanceBetween(sPnt);
          double         dTol  = smos_Max((double)sParamObjZoneTol3d, (double)pV->GetTolerance() );
          if (dDist < dTol) 
            {
              rdEndDist3d = smos_Fabs(dParameter-dTMax)*m_cLineVec.Length();
              rePosition  = SM_PIP_END;
              bFound      = TRUE;
            }
        }

      if (!bFound) 
        {
          double dDist = smos_Fabs(dParameter-dTMin)*m_cLineVec.Length();
          double dTol  = smos_Max((double)sParamObjZoneTol3d, (double)rTestIvl.m_vStart.GetTolerance() );
          if ( dDist < dTol ) 
            {
              rdEndDist3d = dDist;
              rePosition  = SM_PIP_START;
              bFound      = TRUE;
            }
        }

      if (!bFound) 
        {
          double dDist = smos_Fabs(dParameter-dTMax)*m_cLineVec.Length();
          double dTol  = smos_Max((double)sParamObjZoneTol3d, (double)rTestIvl.m_vEnd.GetTolerance() );
          if ( dDist < dTol ) 
            {
              rdEndDist3d = dDist;
              rePosition  = SM_PIP_END;
              bFound      = TRUE;
            }
        }

      if (bFound && rePosition == SM_PIP_START) 
        {
          double dDiffLast = smos_Fabs(dLastParameter-rTestIvl.m_vInterval.GetMin());
          double dDiffCurr = smos_Fabs(dParameter-rTestIvl.m_vInterval.GetMin());
          // See if it is a case
          if (dDiffLast < dDiffCurr) 
            { 
              rePosition = SM_PIP_INSIDE;
              return SM_SUCCESS;
            }
          return SM_SUCCESS;
        }
        
      if (!bFound) 
        { rePosition = SM_PIP_INSIDE; }
      return SM_SUCCESS;

    } // end for each Interval

  // If make it here interval not found - have an error of some sort
  return SM_ERR;

} // end SmLineSegClassification::FindInterval

/*******************************************************************//**
PURPOSE: Given an existing curve classification, insert a curve interval 
   into it.  If the interval falls onto an existing transition or in a 
   classified interval - use a bunch of ambiguity/presidence rules to fix 
   the problems.

NOTES: 
***********************************************************************/
SmStatus SmLineSegClassification::InsertLineSegInterval
 (const SmLineSegInterval & crLineSegIvl)
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
  if (bDebugMe) 
    {
      Dump() ;
      crLineSegIvl.Dump() ;
    }
#endif // SM_DEBUG_CODE

  // freshen LineSegIvl PolyPointClassification SrcZoneTol3d vals
  SmZoneTol3d sSrcZoneTol3d = SmTol::GetSrcZoneTol3d(this) ;
  ((SmLineSegInterval &)crLineSegIvl).m_vStart.SetSrcZoneTol3d(sSrcZoneTol3d) ;
  ((SmLineSegInterval &)crLineSegIvl).m_vMid.  SetSrcZoneTol3d(sSrcZoneTol3d) ;
  ((SmLineSegInterval &)crLineSegIvl).m_vEnd.  SetSrcZoneTol3d(sSrcZoneTol3d) ;

  // locals
  ULONG lIndex1 ;
  SmPolyIntervalPosition ePosition1 ;
  double                 dEndDist3d1 ;
  SmZoneTol3d            sStartObjZoneTol3d = SmTol::GetObjZoneTol3d(&crLineSegIvl.m_vStart) ;
     
  // obsolete: dXSectTol3d = smos_Max((double)crLineSegIvl.m_vStart.GetTolerance(),(double)m_sSrcZoneTol3d) ;

  SER(FindInterval(crLineSegIvl.m_vInterval.GetMin(),
                   SM_BIG_DOUBLE,
                   sStartObjZoneTol3d,
                   lIndex1,
                   dEndDist3d1,
                   ePosition1));
  SM_ASSERT(ePosition1 != SM_PIP_OUTSIDE);

  ULONG lIndex2;
  SmPolyIntervalPosition ePosition2;
  double                 dEndDist3d2;
  SmZoneTol3d            sEndObjZoneTol3d = SmTol::GetObjZoneTol3d(&crLineSegIvl.m_vEnd) ;

  // obsolete double dXSectTol3d2 = smos_Max((double)crLineSegIvl.m_vEnd.GetTolerance(),(double)m_sSrcZoneTol3d);
  SER(FindInterval(crLineSegIvl.m_vInterval.GetMax(),
                   SM_BIG_DOUBLE,
                   sEndObjZoneTol3d,
                   lIndex2,
                   dEndDist3d2,
                   ePosition2));
  SM_ASSERT(ePosition2 != SM_PIP_OUTSIDE);

  // Adjust second interval back one if it lies on the boundary
  if (ePosition2 == SM_PIP_START && lIndex2 > lIndex1) 
    {
      ePosition2 = SM_PIP_END;
      lIndex2 --;
    }

  // Adjust first interval forward one if it lies on the boundary
  if (ePosition1 == SM_PIP_END && lIndex1 < lIndex2)
    {
      ePosition1 = SM_PIP_START;
      lIndex1 ++;
    }

  // Now both should be on same interval.  If they are not try squeezing
  // out any coincident maybe segments.
  if (lIndex1 != lIndex2) 
    {
      // Let's look and see which way we need to go here.
      ULONG lMin = smos_Min(lIndex1,lIndex2);
      ULONG lMax = smos_Max(lIndex1,lIndex2);
      if (lMax-lMin > 1) 
        {
          MSG(_T("Really Bad Interval")); 
          return SM_SUCCESS;
        }

      SmLineSegInterval & rMinIvl = (*this)[lMin];
      SmLineSegInterval & rMaxIvl = (*this)[lMax];
      if (rMinIvl.m_vEnd.GetPointClass() == SM_PPC_VERTEX) 
        {
          if (rMinIvl.m_vMid.GetPointClass() == SM_PPC_EDGE &&
              rMaxIvl.m_vMid.GetPointClass() != SM_PPC_EDGE) 
            {
              lIndex1 = lIndex2 = lMax;
            }
          else if (rMinIvl.m_vMid.GetPointClass() != SM_PPC_EDGE &&
              rMaxIvl.m_vMid.GetPointClass() == SM_PPC_EDGE) 
            {
              lIndex1 = lIndex2 = lMin;
            }
          else 
            {
              SER(ConnectIntervals(lMin,SM_PIP_END,FALSE));
              lIndex1 = lIndex2 = lMin;
            }
        }
      else 
        {
          SER(ConnectIntervals(lMin,SM_PIP_END,FALSE));
          lIndex1 = lIndex2 = lMin;
        }
    }

    
  // This is the interval we are working on
  SmLineSegInterval & rFoundIvl = (*this)[lIndex1];

  // Take care of case where both fall onto the same point because
  // of they are within tolerance.
  if (    ePosition1 == ePosition2
      && (ePosition1 == SM_PIP_START || ePosition1 == SM_PIP_END) )
    {
      if (ePosition1 == SM_PIP_START) 
        {
          rFoundIvl.m_vStart = rFoundIvl.m_vStart.Combine(crLineSegIvl.m_vStart);
          rFoundIvl.m_vStart = rFoundIvl.m_vStart.Combine(crLineSegIvl.m_vEnd);
        }
      else  // (ePosition1 == SM_PIP_END)
        {
          rFoundIvl.m_vEnd = rFoundIvl.m_vEnd.Combine(crLineSegIvl.m_vStart);
          rFoundIvl.m_vEnd = rFoundIvl.m_vEnd.Combine(crLineSegIvl.m_vEnd);
        }
      return SM_SUCCESS;
    }

  // Now split the interval where needed.

  // Take care of start
  if (ePosition1 == SM_PIP_START) 
    {
      rFoundIvl.m_vStart = rFoundIvl.m_vStart.Combine(crLineSegIvl.m_vStart);
    }

  else if (ePosition1 == SM_PIP_INSIDE) 
    {
      SmBoolean bInsertionMade;
      SER(this->InsertPointClass(crLineSegIvl.m_vStart,
                                 crLineSegIvl.m_vInterval.GetMin(),
                                 FALSE,
                                 bInsertionMade));
      // If we didn't insert one end of interval skip the next
      if (!bInsertionMade) return SM_SUCCESS;
      lIndex1++;  // Get next interval if we inserted one
    }
  else { SER(SM_ERR); }

  SmLineSegInterval & rFoundIvl2 = (*this)[lIndex1];

  // Take care of end
  if (ePosition2 == SM_PIP_END) 
    {
      rFoundIvl2.m_vEnd = rFoundIvl2.m_vEnd.Combine(crLineSegIvl.m_vEnd);
    }
  else if (ePosition2 == SM_PIP_INSIDE) 
    {
      // If the end falls onto the start then just skip inserting it
      // and middle
      ULONG lIndex;
      SmPolyIntervalPosition ePosition;
      double                 dEndDist3d;
      SER(FindInterval(crLineSegIvl.m_vInterval.GetMax(),
                       SM_BIG_DOUBLE,
                       sEndObjZoneTol3d,
                       lIndex,
                       dEndDist3d,
                       ePosition));
      if (ePosition != SM_PIP_INSIDE) 
        {
          return SM_SUCCESS;
        }
      SmBoolean bInsertionMade;
      SER(this->InsertPointClass(crLineSegIvl.m_vEnd,
                                 crLineSegIvl.m_vInterval.GetMax(),
                                 FALSE,
                                 bInsertionMade));
    }

  // Take care of middle
  SmLineSegInterval & rFoundIvl3 = (*this)[lIndex1];
  rFoundIvl3.m_vMid = rFoundIvl3.m_vMid.Combine(crLineSegIvl.m_vMid);

  return SM_SUCCESS;

} // end SmLineSegClassification::InsertLineSegInterval

/*******************************************************************//**
PURPOSE: Given an existing curve classification, insert a point 
   classification into it.  If the point class falls onto an existing
   transition or in a classified interval - use a bunch of ambiguity
   and presidence rules to fix the problem.

NOTES: 
***********************************************************************/
SmStatus SmLineSegClassification::InsertPointClass
 (const SmPolyPointClassification & crPointClass,     // in : 
  double                            dParameter,       // in : 
  SmBoolean                         bForceInsertion,  // in : 
  SmBoolean                       & rbInsertionMade)  // out: 
{
  // init output
  rbInsertionMade = FALSE;

  // locals
  ULONG                  lIndex;
  SmPolyIntervalPosition ePosition;
  double                 dXSectTol3d = smos_Max(crPointClass.GetTolerance(),GetTolerance());
  double                 dEndDist3d;

  SER(FindInterval(dParameter,SM_BIG_DOUBLE,dXSectTol3d,lIndex,dEndDist3d,ePosition));
  SM_ASSERT(ePosition != SM_PIP_OUTSIDE);
  SmLineSegInterval & rFound = (*this)[lIndex];

  // when parameter is at Start of interval
  if (ePosition == SM_PIP_START) 
    {
      //
      if(   rFound.m_vStart.GetPointClass() == SM_PPC_EDGE 
         && crPointClass.GetPointClass() == SM_PPC_EDGE) 
        {
          if (rFound.m_vStart.GetObject() != crPointClass.GetObject()) 
            {
              if (!SM_ARE_SAME(rFound.m_vInterval.GetMin(),dParameter)) 
                { bForceInsertion = TRUE; }
            }
        }

      //
      if(   rFound.m_vStart.GetPointClass() == SM_PPC_VERTEX 
         && crPointClass.GetPointClass() == SM_PPC_VERTEX) 
        {
          SmPolyVertex *pV1 = SM_CAST_PTR(SmPolyVertex,rFound.m_vStart.GetObject());
          SmPolyVertex *pV2 = SM_CAST_PTR(SmPolyVertex,crPointClass.GetObject());
          if (pV1 != pV2) 
            {
              if (!SM_ARE_SAME(rFound.m_vInterval.GetMin(),dParameter)) 
                { bForceInsertion = TRUE; }
            }
        }

      //
      if (   lIndex == 0
          && rFound.m_vStart.GetPointClass() == SM_PPC_UNKNOWN
          && dEndDist3d > dXSectTol3d / 100) 
        { bForceInsertion = TRUE; }


      SmBoolean      bDoAdjacencyTest = FALSE;
      SmPolyEdge   * pEdge            = NULL;
      SmPolyVertex * pVertex          = NULL;
      
      //
      if(   rFound.m_vStart.GetPointClass() == SM_PPC_VERTEX 
         && crPointClass.GetPointClass() == SM_PPC_EDGE) 
        {
          bDoAdjacencyTest = TRUE;
          pVertex          = (SmPolyVertex*)rFound.m_vStart.GetObject();
          pEdge            = (SmPolyEdge*)crPointClass.GetObject();
        }

      //
      if (rFound.m_vStart.GetPointClass() == SM_PPC_EDGE &&
          crPointClass.GetPointClass() == SM_PPC_VERTEX) 
        {
          bDoAdjacencyTest = TRUE;
          pVertex = (SmPolyVertex*)crPointClass.GetObject();
          pEdge = (SmPolyEdge*)rFound.m_vStart.GetObject();
        }

      //
      if (bDoAdjacencyTest) 
        {
          SmPolyVertex *pTestV = pEdge->GetStartPolyVertex();
          SmPolyVertex *pOtherTestV = pEdge->GetEndPolyVertex();
          if (pVertex != pTestV && 
              pVertex != pOtherTestV) 
            {
              if (!SM_ARE_SAME(rFound.m_vInterval.GetMin(),dParameter)) 
                { bForceInsertion = TRUE; }
            }
        }


      double dParamTol = SM_EFF_ZERO * (   smos_Fabs(rFound.m_vInterval.GetMin())
                                         + smos_Fabs(rFound.m_vInterval.GetMax()) );
      if (   rFound.m_vStart.GetPointClass() == SM_PPC_UNKNOWN
          && dParameter > rFound.m_vInterval.GetMin() + dParamTol) 
        { bForceInsertion = TRUE; }
    } // end when parameter is at Start of interval check

  // when parameter is at end of interval
  if (ePosition == SM_PIP_END) 
    {
      //
      if(   rFound.m_vEnd.GetPointClass() == SM_PPC_EDGE 
         && crPointClass.GetPointClass() == SM_PPC_EDGE) 
        {
          if (rFound.m_vEnd.GetObject() != crPointClass.GetObject()) 
            {
              if (!SM_ARE_SAME(rFound.m_vInterval.GetMax(),dParameter)) 
                { bForceInsertion = TRUE; }
            }
        }

      //
      if(   rFound.m_vEnd.GetPointClass() == SM_PPC_VERTEX
         && crPointClass.GetPointClass() == SM_PPC_VERTEX) 
        {
          SmPolyVertex *pV1 = SM_CAST_PTR(SmPolyVertex,rFound.m_vEnd.GetObject());
          SmPolyVertex *pV2 = SM_CAST_PTR(SmPolyVertex,crPointClass.GetObject());
          if (pV1 != pV2) 
            {
              if (!SM_ARE_SAME(rFound.m_vInterval.GetMax(),dParameter)) 
                { bForceInsertion = TRUE; }
            }
        }

      //
      if(   lIndex == GetSize()-1 
         && rFound.m_vEnd.GetPointClass() == SM_PPC_UNKNOWN 
         && dEndDist3d > dXSectTol3d / 100) 
        { bForceInsertion = TRUE; }

      //
      SmBoolean      bDoAdjacencyTest = FALSE;
      SmPolyEdge   * pEdge            = NULL;
      SmPolyVertex  *pVertex =        NULL;
      if(   rFound.m_vEnd.GetPointClass() == SM_PPC_VERTEX 
         &&crPointClass.GetPointClass() == SM_PPC_EDGE) 
        {
          bDoAdjacencyTest = TRUE;
          pVertex = (SmPolyVertex*)rFound.m_vEnd.GetObject();
          pEdge = (SmPolyEdge*)crPointClass.GetObject();
        }
      
      //
      if(   rFound.m_vEnd.GetPointClass() == SM_PPC_EDGE 
         && crPointClass.GetPointClass() == SM_PPC_VERTEX) 
        {
          bDoAdjacencyTest = TRUE;
          pVertex = (SmPolyVertex*)crPointClass.GetObject();
          pEdge = (SmPolyEdge*)rFound.m_vEnd.GetObject();
        }
      
      //
      if (bDoAdjacencyTest) 
        {
          SmPolyVertex *pTestV = pEdge->GetStartPolyVertex();
          SmPolyVertex *pOtherTestV = pEdge->GetEndPolyVertex();
          if (pVertex != pTestV && 
              pVertex != pOtherTestV) 
            {
              if(!SM_ARE_SAME(rFound.m_vInterval.GetMax(),dParameter)) 
                { bForceInsertion = TRUE; }
            }
        }


      if(   rFound.m_vEnd.GetPointClass() == SM_PPC_UNKNOWN  && dParameter < rFound.m_vInterval.GetMin() - (SM_EFF_ZERO * smos_Fabs( dParameter )))
        { bForceInsertion = TRUE; }

    } // end parameter is at end of interval check

  // when parameter is inside interval or bForceInsertion
  if (ePosition == SM_PIP_INSIDE || bForceInsertion) 
    {
      // when interval is already classified - don't insert again
      if(   rFound.m_vMid.GetPointClass() != SM_PPC_UNKNOWN 
         && rFound.m_vMid.GetObject() != crPointClass.GetObject()) 
        {
          // Sorry we can not insert anything here because
          // it already has a classification
          rbInsertionMade = FALSE;
          return SM_SUCCESS;
        }

      rbInsertionMade = TRUE;
      SmLineSegInterval sFound = (*this)[lIndex];
      InsertAt(lIndex+1,sFound);
      SmLineSegInterval & rFound1 = (*this)[lIndex]; // Just in case we reallocated during InsertAt
      SmLineSegInterval & rFound2 = (*this)[lIndex+1];
      rFound1.m_vEnd      = crPointClass;
      rFound1.m_vInterval = SmExtent1d(rFound1.m_vInterval.GetMin(),dParameter);
      rFound2.m_vStart    = crPointClass;
      rFound2.m_vInterval = SmExtent1d(dParameter,rFound2.m_vInterval.GetMax());
    } // end parameter is inside interval or bForceInsertion branch
  else if (ePosition == SM_PIP_START) 
    {
      rFound.m_vStart = rFound.m_vStart.Combine(crPointClass);
      rbInsertionMade = TRUE;
      if (lIndex > 0) 
        {
          SmLineSegInterval & rFound2 = (*this)[lIndex-1];
          rFound2.m_vEnd = rFound2.m_vEnd.Combine(crPointClass);
        }
    } // end parameter is at start of interval branch
  else if (ePosition == SM_PIP_END) 
    {
      rFound.m_vEnd = rFound.m_vEnd.Combine(crPointClass);
      rbInsertionMade = TRUE;
      if (lIndex < GetSize()-1) 
        {
          SmLineSegInterval & rFound2 = (*this)[lIndex+1];
          rFound2.m_vStart = rFound2.m_vStart.Combine(crPointClass);
        }
    } // end parameter is at end of interval branch

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    { this->Dump(); }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmLineSegClassification::InsertPointClass

/*******************************************************************//**
PURPOSE: Find the best end of the interval for squeezing.  Use the
    type of point classes and their tolerances to help determine this.

NOTES: 
***********************************************************************/
SmPolyIntervalPosition SmLineSegClassification::FindBestSqueezeEnd(ULONG lIndex) const
{
    SmLineSegInterval & rCIvl = (*this)[lIndex];
    SmPolyPointClassification & rStart = rCIvl.m_vStart;
    SmPolyPointClassification & rEnd = rCIvl.m_vEnd;
    if (rStart.GetPointClass() < rEnd.GetPointClass()) {
        if (lIndex == 0) {
            rStart = rEnd;
        }
        return SM_PIP_START;
    }
    else if (rStart.GetPointClass() > rEnd.GetPointClass()) {
        if (lIndex == GetSize()-1) {
            rEnd = rStart;
        }
        return SM_PIP_END;
    }
    
    if (rStart.GetDeviation() > rEnd.GetDeviation()) {
        return SM_PIP_START;
    }

    return SM_PIP_END;

} // end SmLineSegClassification::FindBestSqueezeEnd

/*******************************************************************//**
PURPOSE: If the count is not the same then try to fix it by squeezing stuff

NOTES: 
***********************************************************************/
SmStatus SmLineSegClassification::FixProblemsBySqueezing( SmLineSegClassification & rCurveClass, SmLineSegClassification & rCurveClassOther )
{
    ULONG ii;
    for (ii=0; ii<rCurveClass.GetSize(); ii++) {
        SmLineSegInterval & rCIvl = rCurveClass[ii];
        ULONG lIndex1, lIndex2;
        SmPolyIntervalPosition ePosition1, ePosition2;
        double dEndDist3d1 = 0.0, dEndDist3d2 = 0.0;
        if (rCurveClassOther.FindInterval(rCIvl.m_vInterval.GetMin(),SM_BIG_DOUBLE,
            smos_Max(rCIvl.m_vStart.GetTolerance(),rCurveClass.GetTolerance()),
            lIndex1,dEndDist3d1,ePosition1) != SM_SUCCESS) {
            continue;
        }
        if (rCurveClassOther.FindInterval(rCIvl.m_vInterval.GetMax(),SM_BIG_DOUBLE,
            smos_Max(rCIvl.m_vEnd.GetTolerance(),rCurveClass.GetTolerance()),
            lIndex2,dEndDist3d2,ePosition2) != SM_SUCCESS) {
            continue;
        }
        // Adjust second interval back one if it lies on the boundary
        if (ePosition2 == SM_PIP_START && lIndex2 > lIndex1) {
            ePosition2 = SM_PIP_END;
            lIndex2 --;
        }
        // Adjust first interval forward one if it lies on the boundary
        if (ePosition1 == SM_PIP_END && lIndex1 < lIndex2) {
            ePosition1 = SM_PIP_START;
            lIndex1 ++;
        }
        // Now both should be on same interval.  If they are not try squeezing
        // out the middle segment.
        if (lIndex1 != lIndex2) {
//            SE(SM_ERR);
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
            if (bDebugMe) {
                rCurveClass.Dump();
                rCurveClassOther.Dump();
            }
#endif
            SmLineSegInterval & rCIvl1 = rCurveClassOther[lIndex1];
            SmLineSegInterval & rCIvl2 = rCurveClassOther[lIndex2];
            if (rCIvl1.m_vInterval.GetLength() <
                rCIvl2.m_vInterval.GetLength()) {
                SmPolyIntervalPosition eSqueezePos = rCurveClassOther.FindBestSqueezeEnd(lIndex1);
                SER(rCurveClassOther.ConnectIntervals(lIndex1,eSqueezePos,FALSE));
            }
            else {
                SmPolyIntervalPosition eSqueezePos = rCurveClassOther.FindBestSqueezeEnd(lIndex2);
                SER(rCurveClassOther.ConnectIntervals(lIndex2,eSqueezePos,FALSE));
            }
        }
        // lIndex1 == lIndex2
        else if (ePosition1 == ePosition2 && 
            ePosition1 != SM_PIP_INSIDE) {
            // Squeeze away one in rCurveClass with best tolerance
            SmPolyIntervalPosition eSqueezePos = rCurveClass.FindBestSqueezeEnd(ii);
//            SER(rCurveClass.ConnectIntervals(ii,eSqueezePos,FALSE));
            SER(rCurveClass.ConnectIntervals(ii,eSqueezePos,TRUE));
            return FixProblemsBySqueezing(rCurveClass,rCurveClassOther);
        }
    } // end for each Interval

    return SM_SUCCESS;

} // end SmLineSegClass::FixProblemsBySqueezing

/*******************************************************************//**
PURPOSE: Squeeze an interval out of the interval.  This may sometimes
    mean deleting it or combining it with an adjacent interval.

NOTES: 
***********************************************************************/
SmStatus SmLineSegClassification::ConnectIntervals
 (ULONG                  lIntervalIndex,
  SmPolyIntervalPosition eEndToSqueeze,
  SmBoolean              bAllowRemovalOfEnds)
{
#ifdef SM_DEBUG_CODE
    SmBoolean bDebugMe = FALSE ;
    if (bDebugMe) {
        Dump();
    }
#endif

    if (lIntervalIndex >= GetSize()) { SER(SM_ERR); }

    // If we don't allow removal of ends then swap the end to squeeze
    if (!bAllowRemovalOfEnds) {
        if ((eEndToSqueeze == SM_PIP_START && lIntervalIndex == 0)) {
            eEndToSqueeze = SM_PIP_END;
        }
        if (eEndToSqueeze == SM_PIP_END && lIntervalIndex == GetSize()-1) {
            eEndToSqueeze = SM_PIP_START;
        }
    }


    // First take care of easy situation where it is at the end
    if (( eEndToSqueeze == SM_PIP_START && lIntervalIndex == 0 )
     || ( eEndToSqueeze == SM_PIP_END   && lIntervalIndex == GetSize()-1) ) {
        // Remove the intervals
        RemoveAt(lIntervalIndex);
    }

    // Now take care of tricky situation where there is a next interval.
    // We need to make the surviving interval go from its end to the end
    // of the squeezed one.
    else if (eEndToSqueeze == SM_PIP_START) {
        SmLineSegInterval & rIvl = (*this)[lIntervalIndex];
        SmLineSegInterval & rIvlPrev = (*this)[lIntervalIndex-1];
        rIvlPrev.m_vInterval.AddValue(rIvl.m_vInterval.GetMax());
        rIvlPrev.m_vEnd = rIvl.m_vEnd;
        RemoveAt(lIntervalIndex);
    }

    else if (eEndToSqueeze == SM_PIP_END) {
        SmLineSegInterval & rIvl = (*this)[lIntervalIndex];
        SmLineSegInterval & rIvlNext = (*this)[lIntervalIndex+1];
        rIvlNext.m_vInterval.AddValue(rIvl.m_vInterval.GetMin());
        rIvlNext.m_vStart = rIvl.m_vStart;
        RemoveAt(lIntervalIndex);
    }

#ifdef SM_DEBUG_CODE
    if (bDebugMe) {
        Dump();
    }
#endif

    return SM_SUCCESS;

} // end SmLineSegClassification::ConnectIntervals

/*******************************************************************//**
PURPOSE: This method takes two curve classification objects which are
   the result of the classification of the same curve against two different
   topologies and it homogenizes them.  Once they are homogenized they
   will have the same number of intervals at the same points on the curve.

NOTES: 
***********************************************************************/
SmStatus SmLineSegClassification::Homogenize
 (SmLineSegClassification & rOther)
{
  double dThisTol  = GetTolerance();
  double dOtherTol = rOther.GetTolerance();
  ULONG ii;

    { // begin Merge curve classes from this into rOther scope
      double dLastParam = SM_BIG_DOUBLE;

      // for every this classification interval boundary 
      for (ii=0; ii<=GetSize(); ii++) 
        {
          double                      dParam;
          SmPolyPointClassification * pPntClass = GetPointClassByIndex(ii, &dParam);
          double                      dXSectTol3d = smos_Max((double)pPntClass->GetTolerance(),dThisTol);
          SmPolyIntervalPosition      eIvlPos;
          ULONG                       lOtherIndex;
          double                      dEndDist3d = 0.0;

          SER(rOther.FindInterval(dParam,
                                  dLastParam,
                                  dXSectTol3d,
                                  lOtherIndex,
                                  dEndDist3d,
                                  eIvlPos));
          dLastParam = dParam;

          // Check for case where we have a small interval and have to force insertion.
          SmBoolean bForceInsertion = FALSE;

          // Check small previous interval on this which was already inserted.
          if (eIvlPos == SM_PIP_START && ii > 0) 
            {
              SmExtent1d & rIvl = (*this)[ii-1].m_vInterval;
              SmExtent1d & rOIvl = rOther[lOtherIndex].m_vInterval;
              if (SM_ARE_SAME(rIvl.GetMin(),rOIvl.GetMin())) 
                {
                  eIvlPos = SM_PIP_INSIDE;
                  bForceInsertion = TRUE;
                }
            }

          //
          if(   ii > 0 
             && dEndDist3d > dXSectTol3d / 100000.0 
             && eIvlPos == SM_PIP_START) 
            {
              SmLineSegInterval & rIvl      = rOther[lOtherIndex] ;
              double              dParamMin = rIvl.m_vInterval.GetMin() ;
              double              dParamPrev ;

              //SmPolyPointClassification * pPrevPntClass = (*this).PointClassIndex(ii-1, &dParamPrev);
              (*this).GetPointClassByIndex(ii-1, &dParamPrev);
              if (SM_ARE_SAME(dParamMin,dParamPrev)) 
                {
                  // Force insertion 
                  eIvlPos = SM_PIP_INSIDE;
                  bForceInsertion = TRUE;
                }
            }

          //
          if(   ii < GetSize()
             && dEndDist3d > dXSectTol3d / 100000.0 
             && eIvlPos == SM_PIP_END) 
           {
              SmLineSegInterval & rIvl      = rOther[lOtherIndex];
              double              dParamMax = rIvl.m_vInterval.GetMax();
              double              dParamPrev;

              //SmPolyPointClassification * pPrevPntClass = (*this).GetPointClassByIndex(ii+1, &dParamPrev);

              (*this).GetPointClassByIndex(ii+1, &dParamPrev);
              if (SM_ARE_SAME(dParamMax,dParamPrev)) 
                {
                  // Force insertion 
                  eIvlPos = SM_PIP_INSIDE;
                  bForceInsertion = TRUE;
                }
            }

          //
          if (eIvlPos == SM_PIP_OUTSIDE) 
            { SER(SM_ERR); }

          // If we are inside of an interval then split the interval.
          if (eIvlPos == SM_PIP_INSIDE) 
            {
              SmLineSegInterval       & rIvl = rOther[lOtherIndex];
              SmPolyPointClassification sMid = rIvl.m_vMid;

              // See if we need to compute parameter values for face or edge
#if 0
              if (sMid.GetPointClass() == SM_PPC_FACE ||
                  sMid.GetPointClass() == SM_PPC_EDGE) {
                  SmPoint3d sPnt;
                  SER(m_cpCurve->EvaluatePoint(dParam,sPnt));
                  SER(sMid.ComputePointParameters(sPnt));
              }
#endif // 0
              SmBoolean bInsertionMade;
              SER(rOther.InsertPointClass(sMid,dParam,bForceInsertion,bInsertionMade));
              if (!bInsertionMade) SER(SM_ERR);
            }
        } // end iter every this classification interval boundary
    } // end Merge curve classes from this into rOther scope

    {  // begin merge curve classes from rOther into this scope
      double dLastParam = SM_BIG_DOUBLE;

      // for every OtherClassification interval boundary
      for (ii=0; ii<=rOther.GetSize(); ii++) 
        {
          double                      dParam;
          SmPolyPointClassification * pPntClass = rOther.GetPointClassByIndex(ii, &dParam);
          SmPolyIntervalPosition      eIvlPos;
          ULONG                       lOtherIndex;
          double                      dEndDist3d;
          double                      dXSectTol3d = smos_Max((double)pPntClass->GetTolerance(),dOtherTol);

          SER(this->FindInterval(dParam,
                                 dLastParam,
                                 dXSectTol3d,
                                 lOtherIndex,
                                 dEndDist3d,
                                 eIvlPos));
          dLastParam = dParam;

          // Check for case where we have a small interval and have to force insertion.
          SmBoolean bForceInsertion = FALSE;

          // Check for small previous interval on other
          if (   ii > 0
              && eIvlPos == SM_PIP_START ) 
            {
              SmExtent1d & rOIvl = rOther[ii-1].m_vInterval;
              SmExtent1d & rIvl = (*this)[lOtherIndex].m_vInterval;
              if (SM_ARE_SAME(rOIvl.GetMin(),rIvl.GetMin())) 
                {
                  eIvlPos = SM_PIP_INSIDE;
                  bForceInsertion = TRUE;
                }
            }
 
          //
          if(   ii > 0 
             && dEndDist3d > dXSectTol3d / 100000.0 
             && eIvlPos == SM_PIP_START) 
           {
              SmLineSegInterval & rIvl      = (*this)[lOtherIndex];
              double              dParamMin = rIvl.m_vInterval.GetMin();
              double              dParamPrev;

              //SmPolyPointClassification * pPrevPntClass = rOther.GetPointClassByIndex(i-1, &dParamPrev);
              rOther.GetPointClassByIndex(ii-1, &dParamPrev);
              if (SM_ARE_SAME(dParamMin,dParamPrev)) 
                {
                  // Force insertion 
                  eIvlPos = SM_PIP_INSIDE;
                  bForceInsertion = TRUE;
                }
            }

          //
          if(   ii < rOther.GetSize()
             && dEndDist3d > dXSectTol3d / 100000.0 
             && eIvlPos == SM_PIP_END) 
           {
              SmLineSegInterval & rIvl = (*this)[lOtherIndex];
              double              dParamMax = rIvl.m_vInterval.GetMax();
              double              dParamPrev;

              //SmPolyPointClassification * pPrevPntClass = rOther.GetPointClassByIndex(ii+1, &dParamPrev);
              rOther.GetPointClassByIndex(ii+1, &dParamPrev);

              if (SM_ARE_SAME(dParamMax,dParamPrev))
                {
                  // Force insertion 
                  eIvlPos = SM_PIP_INSIDE;
                  bForceInsertion = TRUE;
                }
            }

          if (eIvlPos == SM_PIP_OUTSIDE) 
            { SER(SM_ERR); }

          // If we are inside of an interval then split the interval.
          if (eIvlPos == SM_PIP_INSIDE) 
            {
              SmLineSegInterval       & rIvl = (*this)[lOtherIndex];
              SmPolyPointClassification sMid = rIvl.m_vMid;

              // See if we need to compute parameter values for face or edge
#if 0
              if (sMid.GetPointClass() == SM_PPC_FACE ||
                  sMid.GetPointClass() == SM_PPC_EDGE) {
                  SmPoint3d sPnt;
                  SER(m_cpCurve->EvaluatePoint(dParam,sPnt));
                  if (sMid.ComputePointParameters(sPnt) != SM_SUCCESS) {
                      sMid.SetClassObject(SM_PPC_UNKNOWN, NULL);
                  }
              }
#endif // 0
              SmBoolean bInsertionMade;
              SER(InsertPointClass(sMid,dParam,bForceInsertion,bInsertionMade));
              if (!bInsertionMade) SER(SM_ERR);
            }
        } // end iter every OtherClassification interval boundary
    } // end merge curve classes from rOther into this scope
    
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      Dump();
      rOther.Dump();
    }
#endif // SM_DEBUG_CODE
  
  // when number of intervals is not the same  
  if (GetSize() != rOther.GetSize()) 
    {
      // Let's try to force them to be equal
      FixProblemsBySqueezing(*this,rOther);
      if (GetSize() != rOther.GetSize()) 
        { FixProblemsBySqueezing(rOther,*this); }

      if (GetSize() != rOther.GetSize())
        { SER(SM_ERR); }
    } // end number of intervals not the same check

  // all done
  return SM_SUCCESS;

} // end SmLineSegClassification::Homogenize

/*******************************************************************//**
PURPOSE: Merge results of two classifications into the topology.  
    This method assumes that the two curve classifications have been
    homogenized, cleaned up and validated prior to being sent into this method.

NOTES: 
***********************************************************************/
SmStatus SmLineSegClassification::MergeClassifications
 (SmLineSegClassification & rOther,             // in : classification of curve against Other brep
  SmTArray<SmPolyFace*>   * pOptNewFaces,       // out: New Brep  Faces made by merge
  SmTArray<SmPolyFace*>   * pOptNewFacesOther,  // out: New Other Faces made by merge
  SmTArray<SmPolyEdge*>   * pOptNewEdges,       // out: New Brep  Edges made by merge
  SmTArray<SmPolyEdge*>   * pOptNewEdgesOther)  // out: New Other Edges made by merge
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      this->Dump();
      rOther.Dump();
    }
#endif // SM_DEBUG_CODE

  if (pOptNewFaces     ) pOptNewFaces     ->ReSet();
  if (pOptNewFacesOther) pOptNewFacesOther->ReSet();
  if (pOptNewEdges     ) pOptNewEdges     ->ReSet();
  if (pOptNewEdgesOther) pOptNewEdgesOther->ReSet();

  SmPolyFace * sFData[16];
  SmPolyFace * sFData2[16];
  SmPolyEdge * sEData[16];
  SmPolyEdge * sEData2[16];
  SmTArray<SmPolyFace*> sNewFaces(16,sFData);
  SmTArray<SmPolyFace*> sNewFacesOther(16,sFData2);
  SmTArray<SmPolyEdge*> sNewEdges(16,sEData);
  SmTArray<SmPolyEdge*> sNewEdgesOther(16,sEData2);

  // Merge curve segments
  ULONG ii, lSize = GetSize();

  // for every classification interval
  for (ii=0; ii<lSize; ii++)
    {
      SmLineSegInterval & rThisIvl = (*this)[ii];
      SmLineSegInterval & rOtherIvl = rOther[ii];

      // error: intervals must contain one another (really should be the same)
      if(   (!rOtherIvl.m_vInterval.ContainsValue(rThisIvl.m_vInterval.Evaluate(0.5))) 
         || (!rThisIvl.m_vInterval.ContainsValue(rOtherIvl.m_vInterval.Evaluate(0.5))))
        {
          SE(SM_ERR);
          continue;
        }
     
      // Interval.Min classification types
      SmPolyPointClassificationType eMidPCThis  = rThisIvl.m_vMid.GetPointClass();
      SmPolyPointClassificationType eMidPCOther = rOtherIvl.m_vMid.GetPointClass();

      // when either inteval.mid classifies to a PolyEdge or PolyFace
      if(    (eMidPCThis  == SM_PPC_EDGE || eMidPCThis  == SM_PPC_FACE )
          && (eMidPCOther == SM_PPC_EDGE || eMidPCOther == SM_PPC_FACE ) )
        {
          SER( MergeInterval( ii, &sNewFaces, &sNewEdges ));
          if ( pOptNewFaces ) { pOptNewFaces->Append( sNewFaces ); }
          if ( pOptNewEdges ) { pOptNewEdges->Append( sNewEdges ); }
          //if ( pOptNewVertices ) { pOptNewVertices->Append( sNewVertices ); }

          SER( rOther.MergeInterval( ii, &sNewFacesOther, &sNewEdgesOther ));
          if ( pOptNewFacesOther ) { pOptNewFacesOther->Append( sNewFacesOther ); }
          if ( pOptNewEdgesOther ) { pOptNewEdgesOther->Append( sNewEdgesOther ); }
          //if ( pOptNewVerticesOther ) { pOptNewVerticesOther->Append( sNewVerticesOther ); }
        }
    } // end iter every classification interval

  // all done
  return SM_SUCCESS;

} // end SmLineSegClassification::MergeClassifications

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
SmPolyPointClassification * SmLineSegClassification::GetPointClassMateByIndex
 (ULONG lPointClassIndx) 
 const
{
  SM_ASSERT(lPointClassIndx <= GetSize());

  // No mate for first and last one
  if (lPointClassIndx == GetSize() || lPointClassIndx == 0) 
    { return NULL; }

  return &(*this)[lPointClassIndx-1].m_vEnd;

} // end SmLineSegClassification::GetPointClassMateByIndex

/*******************************************************************//**
PURPOSE: Create topology corresponding to interval and its classification.
    The values of the interval will be updated to reflect the creation
    of new edges, faces and vertices.

NOTES: Adds new topology to the PolyBrep that contains 
       the target classification objects as: 
          SmLineSegInterval.Start.m_pObject => PolyBrep-+
          SmLineSegInterval.Mid.m_pObject   => PolyBrep +- expected to be the same
          SmLineSegInterval.End.m_pObject   => PolyBrep-+
***********************************************************************/
SmStatus SmLineSegClassification::MergeInterval
 (ULONG                   lIntervalIndex,  // in : Interval index to merge
  SmTArray<SmPolyFace*> * pOptNewFaces,    // out: New Faces made by merging the interval
  SmTArray<SmPolyEdge*> * pOptNewEdges)    // out: New Edges made by merging the interval
// SmTArray<SmPolyVertex*> * pOptNewVertices)
{
  // init output
  if (pOptNewFaces) pOptNewFaces->ReSet();
  if (pOptNewEdges) pOptNewEdges->ReSet();

  // locals
  SmLineSegInterval & rIvl = (*this)[lIntervalIndex];

  // skip intervals not classified to an existing brep object
  if (rIvl.m_vMid.GetPointClass() == SM_PPC_UNKNOWN) 
    { return SM_SUCCESS; }

  // next Update unknown ends of intervals using the mid point class

  // Start == UNKNOWN, Mid == FACE: Set Start == FACE
  if (rIvl.m_vStart.GetPointClass() == SM_PPC_UNKNOWN) 
    {
      if (rIvl.m_vMid.GetPointClass() == SM_PPC_FACE) 
        {
          // Must be in the face
          rIvl.m_vStart.SetClassObject(SM_PPC_FACE, rIvl.m_vMid.GetObject());
          // Merge start and end point classes of interval
          SmPolyPointClassification *pMate = GetPointClassMateByIndex(lIntervalIndex);
          if (pMate) (*pMate) = rIvl.m_vStart;
        }
    }

  // End == UNKNOWN, Mid == FACE: Set End == FACE
  if (rIvl.m_vEnd.GetPointClass() == SM_PPC_UNKNOWN) 
    {
      if (rIvl.m_vMid.GetPointClass() == SM_PPC_FACE) 
        {
          rIvl.m_vEnd.SetClassObject(SM_PPC_FACE, rIvl.m_vMid.GetObject());
          SmPolyPointClassification *pMate = GetPointClassMateByIndex(lIntervalIndex+1);
          if (pMate) (*pMate) = rIvl.m_vEnd;
        }
    }

  // Now Merge Interval Start and End Points into PolyBrep
  SER(MergePointClass(lIntervalIndex,pOptNewEdges));
  SER(MergePointClass(lIntervalIndex+1,pOptNewEdges));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
  if ( bDebugMe ) 
    {
      this->Dump();
    }
#endif // SM_DEBUG_CODE

  // Get the vertices from the modified rIvl.  After MergePointClass(),
  // they should both be PolyVertices.
  SmPolyVertex *pStartV = (SmPolyVertex*)rIvl.m_vStart.GetObject();
  SmPolyVertex *pEndV   = (SmPolyVertex*)rIvl.m_vEnd.GetObject();
  if (pStartV == pEndV) 
    {
      // Zero length line, don't make an edge.
      return SM_SUCCESS;
    }

  // Before we actually make the Edge, check the topology to see if we
  // have another edge between these two vertices which is coincident
  // with this segment.
  if (rIvl.m_vMid.GetPointClass() == SM_PPC_FACE)
    {
      SmPolyEdge * sEData[32];
      SmTArray<SmPolyEdge*> sVertEdges(32,sEData);
      pStartV->GetPolyEdges(sVertEdges);

      // for every PolyEdge connected to the PolyVertex
      for (ULONG k=0; k<sVertEdges.GetSize(); k++) 
        {
          SmPolyEdge *pEdge = sVertEdges[k];
          NER(pEdge);

          // when PolyEdge's otherVertex == pEndV - this new edge is coincident
          if(   pEdge->GetStartPolyVertex() == pEndV
             || pEdge->GetEndPolyVertex()   == pEndV) 
            {
              // update the ivl mid classification
              rIvl.m_vMid.SetClassObject(SM_PPC_EDGE, pEdge);
              break;
            }
        } // end iter every PolyEdge connected to Start PolyVertex
    } // end if Ivl.mid classifies to Face check

  // Create the edge in the face or in the region
  if (rIvl.m_vMid.GetPointClass() == SM_PPC_FACE)
    {
      SmPolyFace * pFace    = (SmPolyFace*)rIvl.m_vMid.GetObject();
      SmPolyBrep * pBrep    = pFace->GetPolyBrep();
      SmPolyEdge * pNewE    = NULL;
      SmPolyLoop * pNewL    = NULL;
      SmPolyFace * pNewF    = NULL;
      double       dEdgeTol = pFace->GetTolerance();

      // make the PolyEdge in the PolyFace
      SER(pBrep->MakeEdgeInFace(pFace,
                                pStartV,pEndV,
                                dEdgeTol,
                                pNewE,pNewL,pNewF));
#ifdef SM_VALIDATE_TOPOLOGY
      pBrep->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

      // when a newPolyFace was made because of the new PolyEdge - place NewFace on CPolyFace
      if (pNewF) 
        {
          SmCPolyFace * pCFace = pFace->GetCPolyFace();
          // make sure there is a cPolyFace 
          if (!pCFace) 
            {
              pCFace = new (pFace->GetPolyBrep()) SmCPolyFace(pFace);
            }

          // add the New PolyFace to the CPolyFace list
          pCFace->AddPolyFace(pNewF);

          // when asked - remember the new PolyFace
          if (pOptNewFaces) 
            { pOptNewFaces->Add(pNewF) ; }
        } // end new PolyFace check

      // when asked - remember the new PolyEdge
      if(pNewE && pOptNewEdges) 
        { pOptNewEdges->Add(pNewE) ; }

      // finish the new PolyEdge
      pNewE->SetTolerance(dEdgeTol);
      pNewE->GetRadial()->SetTolerance(dEdgeTol);
      rIvl.m_vMid.SetClassObject(SM_PPC_EDGE, pNewE);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
      if (bDebugMe2) 
        {

          SmExtent1d sSegIvl  = rIvl.m_vInterval;
          SmPoint3d  sStartPt = m_cLineStart + m_cLineVec*sSegIvl.GetMin();
          SmPoint3d  sEndPt   = m_cLineStart + m_cLineVec*sSegIvl.GetMax();
          SmVector3d sVec = sEndPt - sStartPt;

          smgfx_Erase();
          smgfx_SetLook(3,6, 1,1,0); sStartPt.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(3,6, 0,1,1); sEndPt.Draw(); sm_GraphicsLoop();
          smgfx_SetLook(2,4, 1,0,0); sVec.Draw(&sStartPt); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0); pFace->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 1,1,0); if (pNewF) { pNewF->Draw(); } sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
    
      // when a new PolyFace was made by PolyEdge insertion - 
      if (pNewF) 
        {
          // update intervals to see if they fall into the new face.
          for (ULONG k=0; k<GetSize(); k++) 
            {
              SER(ClassifyFaceInterval(k,TRUE,pNewF,pFace));
            }
        } // end new PolyFace check

      // all done
      return SM_SUCCESS;

  } // end if mid is Face

  //all done
  return SM_SUCCESS;

} // end SmLineSegClassification::MergeInterval

/*******************************************************************//**
PURPOSE: Static helper function.
   Determine which of two PolyEdges a given 3d position lies on.

NOTES: 
***********************************************************************/
static SmPolyEdge * sm_PointIsOnWhichEdge( const SmPoint3d  & rPnt,
                                                 SmPolyEdge * pNewE1,
                                                 SmPolyEdge * pNewE2,
                                                 double       dTol )
{
  // Init output
  SmPolyEdge *pRet = NULL;

  // Drop to both edges, instead of checking one first and then
  // the other.  For an interval only a couple of tol's long,
  // the midpoint could be within tol of the endpoint of the
  // adjacent interval.  And DropPoint here is cheap anyway. [B202]
  // (Especially since we're using the sum of tols,
  // and m_sSrcZoneTol3d is already a sum of tols.)

  // Alternatively, we could use something that does not clamp
  // (smgu_LineClosestPoint(), after getting end points),
  // and see which is within bounds and very close.

  // Locals
  double dEdgeT1, dEdgeT2;
  double dDist1,  dDist2;

  // Note, DropPoint() always return SM_SUCCESS.
  pNewE1->DropPoint( rPnt, dEdgeT1, dDist1 );
  pNewE2->DropPoint( rPnt, dEdgeT2, dDist2 );

  // Ideal case: one is substantially closer and the other is on a boundary.
  SmBoolean bBdry1 = dEdgeT1 < SM_EFF_ZERO || dEdgeT1 > 1.0-SM_EFF_ZERO;
  SmBoolean bBdry2 = dEdgeT2 < SM_EFF_ZERO || dEdgeT2 > 1.0-SM_EFF_ZERO;

  if (    dDist1 < dDist2 / 100
       && bBdry2
       && dDist1 < smos_Max( dTol, (double)pNewE1->GetTolerance()) )
  {
      pRet = pNewE1;
  }
  else if (    dDist2 < dDist1 / 100
            && bBdry1
            && dDist2 < smos_Max( dTol, (double)pNewE2->GetTolerance()) )
  {
      pRet = pNewE2;
  }
  else
  {
      // If one is definitely closer, use it.
      if ( dDist1 < dDist2 / 2 )
        { pRet = pNewE1; }
      else if (    dDist2 < dDist1 / 2 )
        { pRet = pNewE2; }
  }

  // One more thing, if not found yet: use whichever is not on a boundary.
  if ( pRet == NULL )
  {
      // Warn of a possible problem.
      WARN( _T("Warning: possible problem in sm_PointIsOnWhichEdge()\n") );

      if ( bBdry1 != bBdry2 )
      {
          pRet = ( bBdry1 ) ? pNewE2 : pNewE1;
      }
      else // Just use the closer one.  Could be meaningless at this point.
      {
          pRet = ( dDist2 < dDist1 ) ? pNewE2 : pNewE1;
      }
  }

//  ... old way: 1st case was close enough to fire when 2nd was correct. [B202]
//  if (pNewE1->DropPoint(rPnt,dEdgeT,dDist) == SM_SUCCESS &&
//      dDist < dTol+pNewE1->GetTolerance()) {
//      pRet = pNewE1;
//  }
//  else if (pNewE2->DropPoint(rPnt,dEdgeT,dDist2) == SM_SUCCESS &&
//      dDist2 < dTol+pNewE2->GetTolerance()) {
//      pRet = pNewE2;
//  }
//  else
//    { SE(SM_ERR); }

  return pRet;

} // end local sm_PointIsOnWhichEdge

/*******************************************************************//**
PURPOSE: Given a point classification on the interval list, merge
    it into the corresponding topology and update the point class and
    its adjacent neighbor.

NOTES: Please note that pOptNewEdges and pOptNewVertices are
    additive arrays in this method.  They are not reset here just add
    to whatever is already in the array.
***********************************************************************/
SmStatus SmLineSegClassification::MergePointClass
 (ULONG                   lPointClassIndex,  // in : Index of Interval Point to merge 
  SmTArray<SmPolyEdge*> * pOptNewEdges)      // out: Any new PolyEdges made by point insertion
// SmTArray<SmPolyVertex*> * pOptNewVertices)
{
  // locals
  SmPolyEdge                * pE              = NULL;
  double                      dParam;
  SmPolyPointClassification * pPointClass     = GetPointClassByIndex(lPointClassIndex, &dParam); NER(pPointClass);
  SmPolyPointClassification * pMatePointClass = GetPointClassMateByIndex(lPointClassIndex);
  SmPoint3d                   sVertPnt        = m_cLineStart + dParam*m_cLineVec;
  SmPolyEdge                * sData[16];
  SmTArray<SmPolyEdge*>       sNewEdges(16,sData);

  // Point classifies to Edge
  if (pPointClass->GetPointClass() == SM_PPC_EDGE) 
    {
      pE = (SmPolyEdge*)pPointClass->GetObject();
      double dThisParam, dDist;
      SER(pE->DropPoint(sVertPnt, dThisParam, dDist));
      SER(pE->EvaluatePoint(dThisParam, sVertPnt));
      if (pOptNewEdges) { pOptNewEdges->Add(pE); }
    } // end Point classifies to Edge check

  // Note, this formerly passed m_sSrcZoneTol3d into pPointClass->Merge().
  // That routine then assigns that tol to the Vertex.  But m_sSrcZoneTol3d
  // is the sum of two tolerances, and there's no reason why the vertex's
  // tol should be that big.  In most cases this results in all involved
  // vertices having their tolerances doubled over the course of a Boolean
  // operation, resulting in different classifications during an operation.
  // Pass in what should be a reasonable tolerance for the vertex.  [B202]
  double dTol = pPointClass->GetTolerance() + pPointClass->GetDeviation();

  //
  SER( pPointClass->Merge( sVertPnt, dTol, &sNewEdges ));

  //
  if (pOptNewEdges)    { pOptNewEdges->Append(sNewEdges);   }
  if (pMatePointClass) { (*pMatePointClass) = *pPointClass; }


  // If we split an edge, we have to go through and update all of the
  // pointers to the old edge.  They should be either pNewE1 or pNewE2
  // (one of which is the old edge).

  if ( sNewEdges.GetSize() == 2 )  // Two new edges means we split an edge.
  {
      SmPolyEdge *pNewE1 = sNewEdges[0];
      SmPolyEdge *pNewE2 = sNewEdges[1];

      ULONG ii;
      for ( ii=0; ii<GetSize(); ii++ )
      {
          SmLineSegInterval & rIvl = (*this)[ii];

          // Check start:
          if ( rIvl.m_vStart.GetObject() == pE )
          {
              double dT = rIvl.m_vInterval.GetMin();
              SmPoint3d sPnt = m_cLineStart + dT*m_cLineVec;

              SmPolyEdge *pWhichEdge = sm_PointIsOnWhichEdge( sPnt, pNewE1, pNewE2, m_sSrcZoneTol3d );
              if ( pWhichEdge != NULL )
                { rIvl.m_vStart.SetClassObject( SM_PPC_EDGE, pWhichEdge ); }
              else
                { SER(SM_ERR); }  // Rather harsh; reproduces old behavior.
                                  // Investigate if ever hit.
          }

          // Check end:
          if ( rIvl.m_vEnd.GetObject() == pE )
          {
              double dT = rIvl.m_vInterval.GetMax();
              SmPoint3d sPnt = m_cLineStart + dT*m_cLineVec;

              SmPolyEdge *pWhichEdge = sm_PointIsOnWhichEdge( sPnt, pNewE1, pNewE2, m_sSrcZoneTol3d );
              if ( pWhichEdge != NULL )
                { rIvl.m_vEnd.SetClassObject( SM_PPC_EDGE, pWhichEdge ); }
              else
                { SER(SM_ERR); } // Investigate if ever hit.
          }

          // Check mid:
          if ( rIvl.m_vMid.GetObject() == pE )
          {
              double dT = rIvl.m_vInterval.GetMid();
              SmPoint3d sPnt = m_cLineStart + dT*m_cLineVec;

              SmPolyEdge *pWhichEdge = sm_PointIsOnWhichEdge( sPnt, pNewE1, pNewE2, m_sSrcZoneTol3d );
              if ( pWhichEdge != NULL )
                { rIvl.m_vMid.SetClassObject( SM_PPC_EDGE, pWhichEdge ); }
              else
                { SER(SM_ERR); } // Investigate if ever hit.
          }

      } // For each interval
  }  // Split an edge (two new edges)

  return SM_SUCCESS;

} // end SmLineSegClassification::MergePointClass

/*******************************************************************//**
PURPOSE: Given an interval classify it relative to the face.

NOTES: 
***********************************************************************/
SmStatus SmLineSegClassification::ClassifyFaceInterval(ULONG lIntervalIndex,
                                              SmBoolean bDoPointClassify,
                                              const SmPolyFace * pFaceToClassify,
                                              const SmPolyFace * pOldFace)
{
    SmLineSegInterval & rLineSegIvl = (*this)[lIntervalIndex];
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_Erase();
        smgfx_SetColor(1,0,0);
        this->Draw(rLineSegIvl);
        sm_GraphicsLoop();
        smgfx_SetColor(0,0,0);
        pFaceToClassify->Draw();
        sm_GraphicsLoop();
    }
#endif
    if ((rLineSegIvl.m_vMid.GetPointClass() == SM_PPC_FACE && pOldFace) || 
        rLineSegIvl.m_vMid.GetPointClass() == SM_PPC_UNKNOWN) {
        if (pOldFace) {
            SmPolyFace *pIvlFace = (SmPolyFace*)rLineSegIvl.m_vMid.GetObject();
            if (pIvlFace != pOldFace) return SM_SUCCESS;
        }
        
        SmPolyFace *pFace = (SmPolyFace*)pFaceToClassify;
        
        SmBoolean bDidMin=FALSE;
        SmBoolean bDidMax=FALSE;
        SmBoolean bInsideMin = FALSE;
        SmBoolean bInsideMax = FALSE;
        
        switch (rLineSegIvl.m_vStart.GetPointClass()) {
        case SM_PPC_VERTEX:
            {
                bDidMin = TRUE;
                SmPolyVertex *pV = SM_CAST_PTR(SmPolyVertex,
                    rLineSegIvl.m_vStart.GetObject());
                NER(pV);
                SmPolyEdge * pEdgeOfSector = NULL;
                SmVector3d sVec = m_cLineStart + m_cLineVec - pV->GetPoint();
                SER(pFace->FindVertexSector(pV,sVec,TRUE,pEdgeOfSector));
                if (pEdgeOfSector)
                    bInsideMin = TRUE;
            }
            break;
        case SM_PPC_EDGE:
            {
                bDidMin = TRUE;
                SmPolyEdge *pE = SM_CAST_PTR(SmPolyEdge,
                    rLineSegIvl.m_vStart.GetObject());
                NER(pE);
                SER(pFace->EdgeLineSegOnClassify(pE,m_cLineVec,bInsideMin));
            }
            break;
        case SM_PPC_UNKNOWN:
        case SM_PPC_REGION:
        case SM_PPC_FACE:
        case SM_PPC_POINT:
            break;
        }
        
        switch (rLineSegIvl.m_vEnd.GetPointClass()) {
        case SM_PPC_VERTEX:
            {
                bDidMax = TRUE;
                SmPolyVertex *pV = SM_CAST_PTR(SmPolyVertex,
                    rLineSegIvl.m_vEnd.GetObject());
                NER(pV);
                SmPolyEdge * pEdgeOfSector = NULL;
                SmVector3d sVec = m_cLineStart - pV->GetPoint();
                SER(pFace->FindVertexSector(pV,sVec,TRUE,pEdgeOfSector));
                if (pEdgeOfSector)
                    bInsideMax = TRUE;
            }
            break;
        case SM_PPC_EDGE:
            {
                bDidMax = TRUE;
                SmPolyEdge *pE = SM_CAST_PTR(SmPolyEdge,
                    rLineSegIvl.m_vEnd.GetObject());
                NER(pE);
                SER(pFace->EdgeLineSegOnClassify(pE,-m_cLineVec,bInsideMax));
            }
            break;
        case SM_PPC_UNKNOWN:
        case SM_PPC_REGION:
        case SM_PPC_FACE:
        case SM_PPC_POINT:
            break;
        }
        
        // If interval is not touching anything have to do a point
        // classification of the middle of the interval
        if (bDoPointClassify && bDidMin == FALSE && bDidMax == FALSE) {
            double dParam = rLineSegIvl.m_vInterval.Evaluate(0.4895);
            SmPoint3d sPnt = m_cLineStart + dParam*m_cLineVec;
            SmBoolean bInside;
            SER(pFace->PointInPolygon(sPnt,bInside,TRUE));
            if (bInside) {
                rLineSegIvl.m_vMid.SetClassObject(SM_PPC_FACE, pFace);
            }
            if (rLineSegIvl.m_vMid.GetPointClass() == SM_PPC_FACE) {
                if (rLineSegIvl.m_vStart.GetPointClass() == SM_PPC_FACE) {
                    rLineSegIvl.m_vStart.SetClassObject(SM_PPC_FACE, rLineSegIvl.m_vMid.GetObject());
                }
                if (rLineSegIvl.m_vEnd.GetPointClass() == SM_PPC_FACE) {
                    rLineSegIvl.m_vEnd.SetClassObject(SM_PPC_FACE, rLineSegIvl.m_vMid.GetObject());
                }
                return SM_SUCCESS;
            }
        }
        
        if (bDidMin && bDidMax && bInsideMin != bInsideMax) {
            double dParam = rLineSegIvl.m_vInterval.Evaluate(0.4895);
            SmPoint3d sPnt = m_cLineStart + dParam*m_cLineVec;
            SmBoolean bInside;
            SER(pFace->PointInPolygon(sPnt,bInside,TRUE));
            if (bInside) {
                rLineSegIvl.m_vMid.SetClassObject(SM_PPC_FACE, pFace);
            }
            else {
                // Not inside do nothing.
                return SM_SUCCESS;
            }
            if (rLineSegIvl.m_vMid.GetPointClass() == SM_PPC_FACE) {
                if (rLineSegIvl.m_vStart.GetPointClass() == SM_PPC_FACE) {
                    rLineSegIvl.m_vStart.SetClassObject(SM_PPC_FACE, rLineSegIvl.m_vMid.GetObject());
                }
                if (rLineSegIvl.m_vEnd.GetPointClass() == SM_PPC_FACE) {
                    rLineSegIvl.m_vEnd.SetClassObject(SM_PPC_FACE, rLineSegIvl.m_vMid.GetObject());
                }
                return SM_SUCCESS;
            }

#ifdef SM_DEBUG_CODE
            WARN(_T("possible face classification problem"));
            
            if (bDebugMe) {
                rLineSegIvl.Dump();
                smgfx_Erase();
                smgfx_SetLineWidth(3.0);
                smgfx_SetColor(1,0,0);
                this->Draw(rLineSegIvl);
                double dParameter = rLineSegIvl.m_vInterval.Evaluate(0.0);
                SmPoint3d sPt = m_cLineStart + dParameter*m_cLineVec;
                sPt.Draw();
                sm_GraphicsLoop();
                smgfx_SetLineWidth(1.0);
                smgfx_SetColor(0,0,1);
                pFace->Draw();
                sm_GraphicsLoop();
                pFace->DrawDebug();
                sm_GraphicsLoop();
            }
#endif
            return SM_SUCCESS;
        }

        if ((bDidMin && bInsideMin) || (bDidMax && bInsideMax)) {
            rLineSegIvl.m_vMid.SetClassObject(SM_PPC_FACE, (SmObject*)pFace);
            if (rLineSegIvl.m_vStart.GetPointClass() == SM_PPC_FACE) {
                rLineSegIvl.m_vStart.SetClassObject(SM_PPC_FACE, pFace);
            }
            if (rLineSegIvl.m_vEnd.GetPointClass() == SM_PPC_FACE) {
                rLineSegIvl.m_vEnd.SetClassObject(SM_PPC_FACE, pFace);
            }
            return SM_SUCCESS;
        }
    }

    return SM_SUCCESS;

} // end SmLineSegClassification::ClassifyFaceInterval

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
#define MIN_ARRAY_SIZE 16
void SmLineSegClassification::SetSize(ULONG nNewSize)
{
  if (nNewSize == 0)
    {
      // shrink to nothing
      if (m_pData && !m_bIsBorrowed) 
        { smos_Free(m_pData); }
        
      m_bIsBorrowed = FALSE;
      m_pData = NULL;
      m_lSize = m_lMaxSize = 0;
    }
  else if (m_pData == NULL)
    {
      ULONG nNewMaxSize = MIN_ARRAY_SIZE;
      while (nNewMaxSize < nNewSize) 
        { nNewMaxSize = nNewMaxSize * 2; }

      // okay to use smos_Calloc on static class (SmLineSegInterval and SmPolyPointClassification) objects.
      m_pData = (SmLineSegInterval*) smos_Calloc(1, nNewMaxSize * sizeof(SmLineSegInterval));
      m_bIsBorrowed = FALSE;

      m_lSize = nNewSize;
      m_lMaxSize = nNewMaxSize;
    }
  else if (nNewSize <= m_lMaxSize)
    {
      // first free up old memory
      if ( nNewSize > m_lSize)
        {
          // initialize the new elements
          // okay to use smos_MemSet on static class (SmLineSegInterval and SmPolyPointClassification) objects.
          smos_MemSet(&m_pData[m_lSize], 0, (nNewSize-m_lSize) * sizeof(SmLineSegInterval));
        }
      m_lSize = nNewSize;
    }
  else
    {
      // otherwise, grow array
      ULONG nNewMaxSize = MIN_ARRAY_SIZE;
      while (nNewMaxSize < nNewSize) 
        { nNewMaxSize = nNewMaxSize * 2; }

      SM_ASSERT(nNewMaxSize >= m_lMaxSize);  // no wrap around

      // okay to use smos_Calloc on static class (SmLineSegInterval and SmPolyPointClassification) objects.
      SmLineSegInterval* pNewData = (SmLineSegInterval*) smos_Calloc(1, nNewMaxSize * sizeof(SmLineSegInterval));

      // copy new data from old
      // okay to use smos_MemCpy on static class (SmLineSegInterval and SmPolyPointClassification) objects.
      SE(smos_MemCpy(pNewData, m_pData, m_lSize * sizeof(SmLineSegInterval), nNewMaxSize * sizeof(SmLineSegInterval)));

      // construct remaining elements
      SM_ASSERT(nNewSize > m_lSize);

      // get rid of old stuff (note: no destructors called)
      if (!m_bIsBorrowed) 
        { smos_Free(m_pData); m_pData = NULL ; }
      m_bIsBorrowed = FALSE;
      m_pData = pNewData;
      m_lSize = nNewSize;
      m_lMaxSize = nNewMaxSize;
    }

} // end SmLineSegClassification::SetSize

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmLineSegClassification::InsertAt(ULONG nIndex, const SmLineSegInterval & crNewLineSegIvl)
{
  ULONG nCount = 1;

  if (nIndex >= m_lSize)
  {
      // adding after the end of the array
      SetSize(nIndex + nCount);  // grow so nIndex is valid
  }
  else
  {
      // inserting in the middle of the array
      ULONG nOldSize = m_lSize;
      SetSize(m_lSize + nCount);  // grow it to new size
      // shift old data up to fill gap
      // okay to use smos_MemMove on dynamic class (SmLineSegInterval and SmPolyPointClassification) objects.
      smos_MemMove(&m_pData[nIndex+nCount], &m_pData[nIndex], (nOldSize-nIndex) * sizeof(SmLineSegInterval));

      // re-init slots we copied from
      // okay to use smos_MemSet on static class (SmLineSegInterval and SmPolyPointClassification) objects.
      smos_MemSet(&m_pData[nIndex], 0, nCount * sizeof(SmLineSegInterval));

  }

  // insert new value in the gap
  SM_ASSERT(nIndex + nCount <= m_lSize);
  while (nCount--)
      m_pData[nIndex++] = crNewLineSegIvl;
} // end SmLineSegClassification::InsertAt

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmLineSegClassification::RemoveAt(ULONG nIndex)
{
  ULONG nCount = 1;
  SM_ASSERT(nIndex + nCount <= m_lSize);

  // just remove a range
  ULONG ii, nMoveCount = m_lSize - (nIndex + nCount);

  if ( nMoveCount>0 ) {
      for (ii=0; ii<nMoveCount; ii++) {
          m_pData[ii+nIndex] = m_pData[ii+nIndex+nCount];
      }
// okay to use smos_MemCpy on static class (SmLineSegInterval and SmPolyPointClassification) objects.
//        smos_MemMove(&m_pData[nIndex], &m_pData[nIndex + nCount], nMoveCount * sizeof(SmLineSegInterval));
  }
  m_lSize -= nCount;

} // end SmLineSegClassification::RemoveAt

/////////////////////////////////////////////////////////////////////////////
// Diagnostics

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmPolyPointClassification::Dump(void) const
{
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  switch (m_ePolyPointClass) 
    {
      case SM_PPC_UNKNOWN:
          smos_WriteBuffer(_T("       Unknown Point Classification\n"));
          break;

      case SM_PPC_POINT:
          smos_sprintf(sBuff,_T("        On Point - 0x%p, Dev = %16.16lf\n"),
              m_pObject,m_dDeviation);
          smos_sprintf(sBuffForFile,_T("        On Point - %s, Dev = %16.16lf\n"),
              m_pObject ? _T("notNULL") : _T("NULL"),m_dDeviation);
          smos_WriteBuffer(sBuff, sBuffForFile);
          break;

      case SM_PPC_VERTEX:
          smos_sprintf(sBuff,       _T("        On Vertex - 0x%p, Dev = %16.16lf\n"),
              m_pObject,m_dDeviation);
          smos_sprintf(sBuffForFile,_T("        On Vertex - %s, Dev = %16.16lf\n"),
              m_pObject ? _T("notNULL") : _T("NULL"),m_dDeviation);
          smos_WriteBuffer(sBuff, sBuffForFile);
          break;

      case SM_PPC_EDGE:
          smos_sprintf(sBuff,       _T("        On Edge - 0x%p, T = %16.16lf, Dev = %16.16lf\n"),
              m_pObject,m_dTParam,m_dDeviation);
          smos_sprintf(sBuffForFile,_T("        On Edge - %s, T = %16.16lf, Dev = %16.16lf\n"),
              m_pObject ? _T("notNULL") : _T("NULL"),m_dTParam,m_dDeviation);
          smos_WriteBuffer(sBuff, sBuffForFile);
          break;

      case SM_PPC_FACE:
          smos_sprintf(sBuff,       _T("        On Face - 0x%p, Dev = %16.16lf\n"),
              m_pObject,m_dDeviation);
          smos_sprintf(sBuffForFile,_T("        On Face - %s, Dev = %16.16lf\n"),
              m_pObject ? _T("notNULL") : _T("NULL"),m_dDeviation);
          smos_WriteBuffer(sBuff, sBuffForFile);
          break;

      default:
          SE(SM_ERR);  
    } // ens switch on m_ePolyPointClass

} // end SmPolyPointClassification::Dump

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmPolyPointClassification::Draw() const
{
#ifdef SM_GFX_CODE
    switch (m_ePolyPointClass) {

    case SM_PPC_VERTEX:
        {
        SmPolyVertex *pV = (SmPolyVertex*)m_pObject;
        pV->GetPoint().Draw();
        sm_GraphicsLoop();
        }
        break;

    case SM_PPC_EDGE:
        {
        SmPolyEdge *pE = (SmPolyEdge*)m_pObject;
        SmPoint3d sPnt;
        pE->EvaluatePoint(m_dTParam,sPnt);
        sPnt.Draw();
        pE->Draw();
        sm_GraphicsLoop();
        }
        break;

    case SM_PPC_FACE:
        {
        SmPolyFace *pF = (SmPolyFace*)m_pObject;
        pF->Draw();
        }
        break;
    case SM_PPC_UNKNOWN:
    case SM_PPC_REGION:
    case SM_PPC_POINT:
        break;
    }
#endif
} // end SmPolyPointClassification::Draw

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmLineSegInterval::Dump(void) const
{
    TCHAR sBuff[SM_TBLOCK_SIZE];
    smos_sprintf(sBuff,_T("    Curve Interval - %16.16lf, %16.16lf\n"),
        m_vInterval.GetMin(), m_vInterval.GetMax());
    smos_WriteBuffer(sBuff);
    m_vStart.Dump();
    m_vMid.Dump();
    m_vEnd.Dump();

} // end SmLineSegInterval::Dump

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmLineSegClassification::Dump(void) const
{
    TCHAR sBuff[SM_TBLOCK_SIZE];
    smos_sprintf(sBuff,_T("SmLineSegClassification size=%ld, alloc size=%ld  "),m_lSize,m_lMaxSize);
    smos_WriteBuffer(sBuff);
    SmObject::Dump();
    for (ULONG i = 0; i < m_lSize; i++) {
        smos_sprintf(sBuff,_T("   [%ld] - "),i);
        smos_WriteBuffer(sBuff);
        m_pData[i].Dump();
    }
} // end SmLineSegClassification::Dump

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmLineSegClassification::Draw(const SmLineSegInterval & crLineSegIvl) const
{
#ifdef SM_GFX_CODE
    SmExtent1d sIvl = crLineSegIvl.m_vInterval;
    SmPoint3d sStartPnt = m_cLineStart + m_cLineVec*sIvl.GetMin();
    SmVector3d sVec = m_cLineVec*(sIvl.GetLength());
    sVec.Draw(&sStartPnt);
#else
  SM_REF1(crLineSegIvl);
#endif // SM_GFX_CODE
} // end SmLineSegClassification::Draw

/*******************************************************************//**
PURPOSE: 

NOTES: 
***********************************************************************/
void SmLineSegClassification::Draw() const
{
#ifdef SM_GFX_CODE
    smgfx_SetLineWidth(1);
    smgfx_SetColor(1,0,0);
    m_cLineVec.Draw(&m_cLineStart);
    smgfx_SetLineWidth(1);
    smgfx_SetColor(0,0,0);
    for (ULONG di=0; di<m_lSize; di++) {
        SmPoint3d sPnt = m_cLineStart + m_pData[di].m_vInterval.GetMin() * m_cLineVec;
        smgfx_ChangeColor(di != 0);
        sPnt.Draw();
        m_pData[di].m_vStart.Draw();
        smgfx_ChangeColor(TRUE);
        m_pData[di].m_vMid.Draw();
        smgfx_ChangeColor(TRUE);
        m_pData[di].m_vEnd.Draw();
    }
#endif // SM_GFX_CODE

} // end SmLineSegClassification::Draw()
