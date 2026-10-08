// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmTopologyIntersector.cpp
* PURPOSE: Source file for SmTopologyIntersector object.
**********************************************************************/

#include "StdAfx.h"

#include <SmTopologyIntersector.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

// Remove Composites
// #ifndef __SMCEDGE_H__
// #include <SmCEdge.h>
// #endif

#ifndef __SMEDGE_H__
#include <SmEdge.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#ifdef SM_USE_TBB
#include "tbb/parallel_for.h"
#include "tbb/blocked_range2d.h"
#endif

#include <SmTopologySolver.h>
#include <SmSurface.h>
#include <SmCurve.h>
#include <SmBSplineCurve.h>
#include <SmBrepCache.h>
#include <SmExtent3d.h>
#include <SmTree.h>
#include <SmTrimSrfCache.h>
#include <SmCacheMgr.h>
#include <SmGraphicsExtern.h>
#include <SmGap.h>
#include <SmProtoTopology.h>
#include <SmAssertArray.h>
#include <vector>

// #define SM_VALIDATE_TOPOLOGY 1


/*******************************************************************//**
PURPOSE: Destructor for the topology intersector.

NOTES:
***********************************************************************/
SmTopologyIntersector::~SmTopologyIntersector()
{
    if (m_pIntersectionTable) { delete m_pIntersectionTable; m_pIntersectionTable = NULL ; }
    if (m_pBToO)              { delete m_pBToO;              m_pBToO              = NULL ; }
    if (m_pOToB)              { delete m_pOToB;              m_pOToB              = NULL ; }
    if (m_pSubset)            { delete m_pSubset;            m_pSubset            = NULL ; }
}

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmTopologyIntersector::SmTopologyIntersector
  (const SmContext & crContext)
 : m_bSkipSSI(FALSE),
   m_bSkipManifoldEdges(TRUE),
   m_bIntersectTangentEdges(TRUE),
   m_lIntersectLaminaEdges(0),
   m_bIntersectWireEdges(TRUE),
   m_bIntersectShellVertices(TRUE),
   m_bIgnoreTangency(FALSE),
   m_bDoGluing(FALSE),
   m_bDoingFillets(FALSE),
   m_bTopologyDeleted(FALSE),
   m_crContext(crContext),
   m_pBrep(NULL),
   m_pOther(NULL),
   m_pBToO(NULL),
   m_pOToB(NULL),
   m_bSwappedOrder(FALSE),
   m_dThisApproxTol3d(0.0),
   m_dThisAngTolRad(0.0),
   m_pIntersectionTable(NULL),
   m_pSubset(NULL)
{
}

/*******************************************************************//**
PURPOSE: Constructor for the topology intersector.

NOTES:
***********************************************************************/
SmTopologyIntersector::SmTopologyIntersector
  (const SmContext & crContext,
   SmBrep          * pBrep,
   SmBrep          * pOther,
   double            dThisApproxTol3d,
   double            dThisAngTolRad)
 : m_bSkipSSI(FALSE),
   m_bSkipManifoldEdges(TRUE),
   m_bIntersectTangentEdges(TRUE),
   m_lIntersectLaminaEdges(0),
   m_bIntersectWireEdges(TRUE),
   m_bIntersectShellVertices(TRUE),
   m_bIgnoreTangency(FALSE),
   m_bDoGluing(FALSE),
   m_bDoingFillets(FALSE),
   m_bTopologyDeleted(FALSE),
   m_crContext(crContext),
   m_pBrep(pBrep),
   m_pOther(pOther),
   m_bSwappedOrder(FALSE),
   m_dThisApproxTol3d(dThisApproxTol3d),
   m_dThisAngTolRad(dThisAngTolRad),
   m_pIntersectionTable(NULL),
   m_pSubset(NULL)
{
    m_pBToO = new(crContext) SmMapPtrToPtr<SmObject, SmObject>;
    m_pOToB = new(crContext) SmMapPtrToPtr<SmObject, SmObject>;
}

/*******************************************************************//**
PURPOSE: Destructor for the intersection table.

NOTES:
***********************************************************************/
SmIntersectionTable::~SmIntersectionTable()
{
  ULONG i, j;
  for (i=0; i<m_vResults.GetSize(); i++) 
    {
      SmIntersectionResults & rResult = m_vResults[i];
      if (rResult.m_p3DCurves) 
        {
          for (j=0; j<rResult.m_p3DCurves->GetSize(); j++) 
            {
              SmCurve *p3DCurve  = (*rResult.m_p3DCurves)[j];
              SmCurve *pUVCurve1 = (*rResult.m_pUVCurves1)[j];
              SmCurve *pUVCurve2 = (*rResult.m_pUVCurves2)[j];

              if (p3DCurve)  { delete p3DCurve;  p3DCurve  = NULL ; }
              if (pUVCurve1) { delete pUVCurve1; pUVCurve1 = NULL ; }
              if (pUVCurve2) { delete pUVCurve2; pUVCurve2 = NULL ; }
            }
          SM_ASSERT(rResult.m_p3DCurves   != NULL) ; delete rResult.m_p3DCurves ;   rResult.m_p3DCurves   = NULL ;
          SM_ASSERT(rResult.m_pUVCurves1  != NULL) ; delete rResult.m_pUVCurves1 ;  rResult.m_pUVCurves1  = NULL ;
          SM_ASSERT(rResult.m_pUVCurves2  != NULL) ; delete rResult.m_pUVCurves2 ;  rResult.m_pUVCurves2  = NULL ;
          SM_ASSERT(rResult.m_pCurveTypes != NULL) ; delete rResult.m_pCurveTypes ; rResult.m_pCurveTypes = NULL ;
          SM_ASSERT(rResult.m_pDeviations != NULL) ; delete rResult.m_pDeviations ; rResult.m_pDeviations = NULL ;
        }
    }
} // end SmIntersectionTable::~SmIntersectionTable destructor

/*******************************************************************//**
PURPOSE: Clear out the existing SSI table information

NOTES:
***********************************************************************/
SmStatus SmTopologyIntersector::ClearExistingSSI()
{
    if (m_pIntersectionTable) { delete m_pIntersectionTable; m_pIntersectionTable = NULL ; }
    m_pIntersectionTable = NULL;
    return SM_SUCCESS;
}

/*******************************************************************//**
PURPOSE: Determine if there has been a registered intersection between
     these two surfaces.

NOTES:
***********************************************************************/
SmStatus SmTopologyIntersector::CheckIfHaveExistingSSI(const SmContext & crContext,
                                                       const SmSurface *pBrepSurface,
                                                       const SmSurface *pOtherSurface,
                                                       SmBoolean & rbHaveExistingSSI,
                                                       SmTArray<SmCurve*> & r3DCurves,
                                                       SmTArray<SmCurve*> & rUVCurves1,
                                                       SmTArray<SmCurve*> & rUVCurves2,
                                                       SmTArray<SmTsectCurveType> & rCurveTypes,
                                                       SmTArray<double> & rDeviations)
{
    r3DCurves.ReSet();
    rUVCurves1.ReSet();
    rUVCurves2.ReSet();
    rCurveTypes.ReSet();
    rDeviations.ReSet();

    rbHaveExistingSSI = FALSE;
    if (!m_pIntersectionTable) {
        return SM_SUCCESS;
    }

    ULONG ii, jj;
    for (ii=0; ii<m_pIntersectionTable->m_vBrepSurfaces.GetSize(); ii++) {
        if (pBrepSurface != m_pIntersectionTable->m_vBrepSurfaces[ii]) continue;
        if (pOtherSurface != m_pIntersectionTable->m_vOtherSurfaces[ii]) continue;
        // Here we have found a match load answers and continue looking
        rbHaveExistingSSI = TRUE;
        const SmIntersectionResults & crResults = m_pIntersectionTable->m_vResults[ii];
        if (crResults.m_p3DCurves) {
            r3DCurves.Append(*crResults.m_p3DCurves);
            rUVCurves1.Append(*crResults.m_pUVCurves1);
            rUVCurves2.Append(*crResults.m_pUVCurves2);
            rCurveTypes.Append(*crResults.m_pCurveTypes);
            rDeviations.Append(*crResults.m_pDeviations);
            for (jj=0; jj<r3DCurves.GetSize(); jj++) {
                SmBSplineCurve *pBSC = SM_CAST_PTR(SmBSplineCurve,r3DCurves[jj]);
                if (pBSC) {
                    r3DCurves[jj] = new(crContext) SmBSplineCurve(*pBSC);
                }
                else { SER(SM_ERR); }
                pBSC = SM_CAST_PTR(SmBSplineCurve,rUVCurves1[jj]);
                if (pBSC) {
                    rUVCurves1[jj] = new(crContext) SmBSplineCurve(*pBSC);
                }
                pBSC = SM_CAST_PTR(SmBSplineCurve,rUVCurves2[jj]);
                if (pBSC) {
                    rUVCurves2[jj] = new(crContext) SmBSplineCurve(*pBSC);
                }
            }
        }
    }

    // Now do a quick check to see if we have an opposing NULL which
    // means that we have an intersection but it is nothing.
    for (ii=0; ii<m_pIntersectionTable->m_vBrepSurfaces.GetSize(); ii++) {
        if (pBrepSurface == m_pIntersectionTable->m_vBrepSurfaces[ii] &&
            m_pIntersectionTable->m_vOtherSurfaces[ii] == NULL) {
            rbHaveExistingSSI = TRUE;
            return SM_SUCCESS;
        }
    }

    return SM_SUCCESS;

}  // end CheckIfHaveExistingSSI



/*******************************************************************//**
PURPOSE: Register an existing intersection between two surfaces
     to be used during the topology intersection process.  All curves
     will be copied in this method.  If one of the surfaces is NULL it
     assumes that you are turning off all non-registered intersections
     of the non NULL surface.

NOTES: Note that the sizes of all  of the arrays must be the
     same.  The size could be zero if there are no intersections between
     these two surfaces.  If there are curves, the 3D curves must always
     be present.  If the UV curve is not computed it may be NULL in the
     array.
***********************************************************************/
SmStatus SmTopologyIntersector::RegisterExistingSSI
  (SmSurface *pBrepSurface,                         // in : surf1
   SmSurface *pOtherSurface,                        // in : surf2
   const SmTArray<SmCurve*> & cr3DCurves,           // in : 3d xsect curves
   const SmTArray<SmCurve*> & crUVCurves1,          // in : associated surf1 UVTrimCurves
   const SmTArray<SmCurve*> & crUVCurves2,          // in : associated surf2 UVTrimCurves
   const SmTArray<SmTsectCurveType> & crCurveTypes, // in : associated intersection curve types
   const SmTArray<double> & crDeviations)           // in : associated intersection curve deviations
{
    if (!m_pIntersectionTable) {
        m_pIntersectionTable = new (m_crContext) SmIntersectionTable();
        NER(m_pIntersectionTable);
    }
    m_pIntersectionTable->m_vBrepSurfaces.Add(pBrepSurface);
    m_pIntersectionTable->m_vOtherSurfaces.Add(pOtherSurface);
    SmIntersectionResults sResult;
    if (cr3DCurves.GetSize() > 0) {
        sResult.m_p3DCurves = new (m_crContext) SmTArray<SmCurve*>(m_crContext);
        sResult.m_p3DCurves->Append(cr3DCurves);
        sResult.m_pUVCurves1 = new (m_crContext) SmTArray<SmCurve*>(m_crContext);
        sResult.m_pUVCurves1->Append(crUVCurves1);
        sResult.m_pUVCurves2 = new (m_crContext) SmTArray<SmCurve*>(m_crContext);
        sResult.m_pUVCurves2->Append(crUVCurves2);
        sResult.m_pCurveTypes = new (m_crContext) SmTArray<SmTsectCurveType>(m_crContext);
        sResult.m_pCurveTypes->Append(crCurveTypes);
        sResult.m_pDeviations = new (m_crContext) SmTArray<double>(m_crContext);
        sResult.m_pDeviations->Append(crDeviations);

        SmTArray<SmCurve*> & r3DCurves = *sResult.m_p3DCurves;
        SmTArray<SmCurve*> & rUVCurves1 = *sResult.m_pUVCurves1;
        SmTArray<SmCurve*> & rUVCurves2 = *sResult.m_pUVCurves2;

        ULONG ii;
        for (ii=0; ii<r3DCurves.GetSize(); ii++) {
            SmBSplineCurve *pBSC = SM_CAST_PTR(SmBSplineCurve,r3DCurves[ii]);
            if (pBSC) {
                r3DCurves[ii] = new(m_crContext) SmBSplineCurve(*pBSC);
            }
            else { SER(SM_ERR); }
            pBSC = SM_CAST_PTR(SmBSplineCurve,rUVCurves1[ii]);
            if (pBSC) {
                rUVCurves1[ii] = new(m_crContext) SmBSplineCurve(*pBSC);
            }
            pBSC = SM_CAST_PTR(SmBSplineCurve,rUVCurves2[ii]);
            if (pBSC) {
                rUVCurves2[ii] = new(m_crContext) SmBSplineCurve(*pBSC);
            }
        }
    }

    m_pIntersectionTable->m_vResults.Add(sResult);
    return SM_SUCCESS;
}




/*******************************************************************//**
PURPOSE: Check to see if the preexisting relationship between two
    vertices indicate that squeezing of them needs to be done.

NOTES:
   If either input vertex is already mated to a different vertex,
   then if the previous mating looks better, this method will take
   no action, and return SM_ERR.
***********************************************************************/
SmStatus SmTopologyIntersector::CheckForSqueezeVertices
  (SmVertex * pBrepVertex,              // in : 1st of mated pair of vertices
   SmVertex * pOtherVertex,             // in : 2nd of mated pair of vertices
   SmTArray<SmVertex*> & rDeletedV,     // out: list of deleted vertices
   SmTArray<SmVertex*> & rSurvivingV,   // out: list of surviving vertices
   SmTArray<SmEdge*> & rDeletedE)       // out: list of deleted edges
{
static constexpr double cbiLim = 1.0;
static constexpr SmStatus cbiAct = SM_SUCCESS;
if ( pBrepVertex->GetTolerance() > cbiLim ||   pOtherVertex->GetTolerance() > cbiLim )
  { SER( cbiAct ); } //cbi breakpoint

#ifdef SM_USE_OLDTOL
    // No, don't sync these, do the pairs that will get combined.  [B458 B459]
    // // Make sure tolerances of newly related vertices are the same.
    // if ( pOtherVertex->GetTolerance() > 1.01 * pBrepVertex->GetTolerance() )
    //   { pBrepVertex->SetTolerance( pOtherVertex->GetTolerance() ) ; }
    // if ( pBrepVertex->GetTolerance() > 1.01*pOtherVertex->GetTolerance() )
    //   { pOtherVertex->SetTolerance( pBrepVertex->GetTolerance() ) ; }
#endif // SM_USE_OLDTOL

    // Get any already-existing vertex relationships.
    SmVertex *pOtherV = (SmVertex*)GetOtherMate( pBrepVertex );
    SmVertex *pBrepV  = (SmVertex*)GetBrepMate( pOtherVertex );

    // Check for a pre-existing, conflicting relationship.
    if ( pOtherV && pOtherV != pOtherVertex )
    {
if ( pOtherV->GetTolerance() > cbiLim )
  { SER( cbiAct ); } //cbi breakpoint

        // If the existing relationship is better than this would be,
        // don't do it.  [B458 B459]
        double dDistOld = pBrepVertex->GetPoint().DistanceBetween( pOtherV     ->GetPoint() );
        double dDistNew = pBrepVertex->GetPoint().DistanceBetween( pOtherVertex->GetPoint() );
        if ( dDistNew > dDistOld + SM_EFF_ZERO_SQRT )
          { SER_MSG( SM_ERR, _T("Conflicting relationship, previous one is better") ); }

        SER(RemoveRelationship(pBrepVertex,pOtherV));
        // pOtherV and rpOtherVertex are probably close check out if
        // it is possible to squeeze them.
        SmEdge *pDeletedOtherEdge = NULL;

        // Combine two vertices which are coincident or nearly so

#ifdef SM_USE_OLDTOL
        SM_OLDTOL_LINE // Make sure tolerances of newly related vertices are the same.
        SM_OLDTOL_LINE if ( pOtherVertex->GetTolerance() > 1.01 * pOtherV->GetTolerance() )
        SM_OLDTOL_LINE   { pOtherV->SetTolerance( pOtherVertex->GetTolerance() ) ; }
        SM_OLDTOL_LINE if ( pOtherV->GetTolerance() > 1.01*pOtherVertex->GetTolerance() )
        SM_OLDTOL_LINE   { pOtherVertex->SetTolerance( pOtherV->GetTolerance() ) ; }
#endif // SM_USE_OLDTOL

        SER( m_pOther->CombineCoincidentVertices(
                pOtherVertex, pOtherV, pDeletedOtherEdge ));
        if ( pDeletedOtherEdge )
        {
            rDeletedE.Add( pDeletedOtherEdge );
            SmEdge *pDeletedBrepEdge = (SmEdge*)GetBrepMate(pDeletedOtherEdge);
            if ( pDeletedBrepEdge )
            { SER( RemoveRelationship( pDeletedBrepEdge, pDeletedOtherEdge ) ); }
        }

        // remember which vertices are deleted and which survive
        rSurvivingV.Add( pOtherVertex );
        rDeletedV.Add( pOtherV );
        //  SER(CheckForSqueezeEdges(pBrepVertex));

    } // end old pBrebVertex relationship exists and is out of date check

    // Check for a pre-existing, conflicting relationship.
    if ( pBrepV && pBrepV != pBrepVertex )
    {
if ( pBrepV->GetTolerance() > cbiLim )
  { SER( cbiAct ); } //cbi breakpoint

        // If the existing relationship is better than this would be,
        // don't do it.  [B458 B459]
        double dDistOld = pOtherVertex->GetPoint().DistanceBetween( pBrepV     ->GetPoint() );
        double dDistNew = pOtherVertex->GetPoint().DistanceBetween( pBrepVertex->GetPoint() );
        if ( dDistNew > dDistOld + SM_EFF_ZERO_SQRT )
          { SER_MSG( SM_ERR, _T("Conflicting relationship, previous one is better") ); }

        SER( RemoveRelationship( pBrepV, pOtherVertex ));
        SmEdge *pDeletedBrepEdge = NULL;

        // Combine two vertices which are coincident or nearly so

#ifdef SM_USE_OLDTOL
        SM_OLDTOL_LINE // Make sure tolerances of newly related vertices are the same.
        SM_OLDTOL_LINE if ( pBrepVertex->GetTolerance() > 1.01*pBrepV->GetTolerance() )
        SM_OLDTOL_LINE   { pBrepV->SetTolerance( pBrepVertex->GetTolerance() ) ; }
        SM_OLDTOL_LINE if ( pBrepV->GetTolerance() > 1.01 * pBrepVertex->GetTolerance() )
        SM_OLDTOL_LINE   { pBrepVertex->SetTolerance( pBrepV->GetTolerance() ) ; }
#endif // SM_USE_OLDTOL

        SER( m_pBrep->CombineCoincidentVertices(
                pBrepVertex, pBrepV, pDeletedBrepEdge ));
        if ( pDeletedBrepEdge )
        {
            rDeletedE.Add( pDeletedBrepEdge );
            SmEdge *pDeletedOtherEdge = (SmEdge*)GetOtherMate(pDeletedBrepEdge);
            if ( pDeletedOtherEdge )
            { SER( RemoveRelationship( pDeletedBrepEdge, pDeletedOtherEdge ) ); }
        }

        // remember which vertex survives and which is deleted
        rSurvivingV.Add( pBrepVertex );
        rDeletedV.Add( pBrepV );
        //  SER(CheckForSqueezeEdges( pBrepVertex ));

    } // end pOtherVertex has an old relationship that is out of date check

    return SM_SUCCESS;

} // end SmTopologyIntersector::CheckForSqueezeVertices

/*******************************************************************//**
PURPOSE: Remove faces listed in crFaces from the merge Brep and
    remove any edges which are turned into wire edges and any vertices
    which are turned into shell vertices by the face removals.

NOTES: when Infinite Region happens to be merged, it is kept and
  the region to which it is being merged is deleted.
***********************************************************************/
SmStatus SmTopologyIntersector::DeleteFaces
 (const SmTArray<SmFace*> & crFaces,         // in : Brep faces to delete
  const SmTArray<SmEdge*> * pOptKeepAsWires) // in : list of edges to turn into wires if all the faces
                                             //      to which they are attached happen to get deleted.
                                             //      Edges not on this list are deleted when all 
                                             //      the faces to which they are attached are deleted.
                                             //      NULL to ignore, default:[NULL].
{
  // locals
  SmTArray<SmVertex*> sFacesVertices(128), sFaceVertices(128);
  SmTArray<SmEdge*>   sFacesEdges(128),    sFaceEdges(128);
  SmBrep *pBrep = NULL;
  ULONG ii, jj ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
#endif // SM_DEBUG_CODE

  // iter every face saving all face vertices and edges and calling SmBrep::DeleteFace()
  for(ii=0; ii<crFaces.GetSize(); ii++)
    {
      SmFace *pF = crFaces[ii];

      // Accumulate DeletedFace unique EdgeList for later
      pF->GetEdges(sFaceEdges);
      for(jj=0; jj<sFaceEdges.GetSize(); jj++)
        {
          sFacesEdges.AddUnique(sFaceEdges[jj]);
        }

      // Accumulate DeletedFace unique VertexList for later
      pF->GetVertices(sFaceVertices);
      for(jj=0; jj<sFaceVertices.GetSize(); jj++)
        {
          sFacesVertices.AddUnique(sFaceVertices[jj]);
        }

      // remove TargetFace<->OtherFace relationship if it exists
      SmFace *pOFace = (SmFace*)GetOtherMate(pF);
      if (pOFace)
        {
          RemoveRelationship(pF,pOFace);
        }

      // TargetFace->Brep - to edit
      pBrep = pF->GetBrep();

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
      { 
          TCHAR sBuff[SM_TBLOCK_SIZE];
          smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pBrep"));
          m_pBrep->DumpRegionsAndAttributes(sBuff) ; 
      }
#endif // SM_DEBUG_CODE

      // delete TargetFace from its Brep 
      //   note: when Infinite region merges it's kept and other region is deleted
      SER( pBrep->DeleteFace( pF, FALSE ));  // FALSE = don't delete dangling edges and vertices,
                                             //         we do that below in accordance with pOptKeepAsWires.
#ifdef SM_DEBUG_CODE
      if (bDebugMe)
      { 
          TCHAR sBuff[SM_TBLOCK_SIZE];
          smos_sprintf(sBuff,_T("%s"), _T("m_vTI.m_pBrep"));
          m_pBrep->DumpRegionsAndAttributes(sBuff) ;
      }
#endif // SM_DEBUG_CODE

    } // end iter every face to delete

  if (!pBrep) return SM_SUCCESS;

#ifdef SM_VALIDATE_TOPOLOGY
      if (m_pBrep) m_pBrep->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      pBrep->Dump();
    }
#endif // SM_DEBUG_CODE

  // iter every edge of every deleted face looking for wire edges to delete
  for(ii=0; ii<sFacesEdges.GetSize(); ii++)
    {
      SmEdge *pE = sFacesEdges[ii];

      // When the edge is a wire
      if (pE->IsWire())
        {
          // save wires when asked
          ULONG lFoundIndex ;
          if(   pOptKeepAsWires != NULL
             && pOptKeepAsWires->FindElement(pE, lFoundIndex))
            { continue ; }

          // else remove the wire

          // remove any TargetEdge<->OtherEdge relationships
          if (GetOtherMate(pE) != NULL)
            {
              SmEdge *pOE = (SmEdge*)GetOtherMate(pE);
              RemoveRelationship(pE,pOE);
#ifdef SM_DEBUG_CODE
              if (bDebugMe)
                {
                  pE->GetCurve()->Dump();
                  sm_GraphicsLoop();
                  smgfx_SetLook(6,8, 1,0,0);
                  pE->GetCurve()->DrawWDeriv(pE->GetInterval(),0);
                  sm_GraphicsLoop();
                  smgfx_SetLineWidth(2.0);
                }
#endif // SM_DEBUG_CODE
            }

          // delete the Edge from the Brep
          SER(pE->GetBrep()->DeleteEdge(pE));

#ifdef SM_VALIDATE_TOPOLOGY
          pBrep->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY
        } // end edge is a wire check
    } // end iter every deleted face edge

#ifdef SM_VALIDATE_TOPOLOGY
      pBrep->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    { pBrep->Dump(); }
#endif // SM_DEBUG_CODE

  // iter every vertex of every deleted face looking for ShellVertices to delete
  for(ii=0; ii<sFacesVertices.GetSize(); ii++)
    {
      SmVertex *pV = sFacesVertices[ii];

      // when vertex is a shell vertex
      if (pV->IsShellVertex())
        {
          // remove any TargetVertex<->OtherVertex relationships
          if (GetOtherMate(pV) != NULL)
            {
              SmVertex *pOV = (SmVertex*)GetOtherMate(pV);
              RemoveRelationship(pV,pOV);
#ifdef SM_DEBUG_CODE
              if (bDebugMe)
                {
                  sm_GraphicsLoop();
                  smgfx_SetLook(6,10, 1,0,0);
                  pV->Draw();
                  sm_GraphicsLoop();
                  smgfx_SetPointSize(5.0);
                }
#endif // SM_DEBUG_CODE
            }

          // delete the vertex from the Brep
          SER(pV->GetBrep()->DeleteVertex(pV));
        } // end vertex is a ShellVertex check
    } // end iter every vertex of every deleted face

  // all done
  return SM_SUCCESS;

} // end SmTopologyIntersector::DeleteFaces

/*******************************************************************//**
PURPOSE: Fix gaps in the intersection loop by squeezing together vertices.

NOTES: The vertices in the array are from m_pBrep.
***********************************************************************/
SmStatus SmTopologyIntersector::FixGaps
  (const SmTArray<SmVertex*> & crSingleVertices,      // in : list of single vertices
   SmBoolean                 & rbModifiedTopology,    // out: was topology modified?
   SmTArray< SmVertex* >     & rDeletedVertices,      // out: deleted verts in m_pBrep
   SmTArray< SmEdge*   >     & rDeletedEdges)         // out: deleted edges in m_pBrep
{
  rbModifiedTopology = FALSE;
  rDeletedVertices.ReSet();
  rDeletedEdges.ReSet();
  ULONG ii, jj, kk, lNumSingleVerts = crSingleVertices.GetSize();

  // Look for gaps bigger than the sum of the two vertices' tolerances.
  {
    // for every single vertex
    for (ii=0; ii<lNumSingleVerts; ii++)
      {
        SmVertex *pV1 = crSingleVertices[ii];

        // for every other single vertex
        for (jj=ii+1; jj<lNumSingleVerts; jj++)
          {
            SmVertex *pV2 = crSingleVertices[jj];

            // Increase tol just a bit.  Was 10.0.  [B498]
            static constexpr double dTolFactor = 1.01;
            double dTol = dTolFactor * (pV1->GetTolerance() + pV2->GetTolerance());

            // look for a close enough pairing
            if (pV1->GetPoint().DistanceBetween(pV2->GetPoint()) < dTol)
              {
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
                if (bDebugMe)
                  {
                    smgfx_Erase();
                    smgfx_SetLook(1,2, 0,0,0); m_pBrep->Draw(); sm_GraphicsLoop();
                    smgfx_SetLook(1,2, 0,0,1); m_pOther->Draw(); sm_GraphicsLoop();
                    smgfx_SetLook(1,2, 0,1,1); this->Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
                    smgfx_SetLook(4,5, 1,0,0); pV1->Draw(); sm_GraphicsLoop();
                    smgfx_SetLook(6,7, 0,1,0); pV2->Draw(); sm_GraphicsLoop();
                    sm_GraphicsLoop();
                  }
#endif // SM_DEBUG_CODE

                // Make sure the vertices do not have a common edge greater than tolerance length
                SmTArray<SmEdge*> sEdges1, sEdges2;
                pV1->GetEdges(sEdges1);
                pV2->GetEdges(sEdges2);
                SmBoolean bFoundCommonEdge = FALSE;

                // for every V1 edge
                for (kk=0; kk<sEdges1.GetSize(); kk++)
                  {
                    SmEdge *pTest = sEdges1[kk];
                    ULONG lFoundIndex;
                    if (sEdges2.FindElement(pTest,lFoundIndex))
                      {
                        double dEdgeLeng = pTest->GetCurve()->ApproximateLength(pTest->GetInterval(),10);
                        if (dEdgeLeng > dTol)
                          {
                            bFoundCommonEdge = TRUE;
                          } // end common edge is bigger than tolerance
                      } // end edge in V2 edgel list check
                  } // end iter every V1 edge

                // when no largerThanTol common edge is found between V1 and V2
                if (!bFoundCommonEdge)
                  {
                    // Have a good candidate for squeezing
                    SmVertex *pOtherV1 = (SmVertex*)GetOtherMate(pV1); NER(pOtherV1);
                    SmVertex *pOtherV2 = (SmVertex*)GetOtherMate(pV2); NER(pOtherV2);
                    SmEdge *pDeletedBrepEdge = NULL, *pDeletedOtherEdge = NULL;

                    // combine coincident vertices
                    SM_DBG_WARN(_T("SmTopologyIntersector::FixGaps: changing topology for a boolean operation")) ;
                    SER(m_pBrep->CombineCoincidentVertices(pV2,pV1,pDeletedBrepEdge));
                    rDeletedVertices.Add( pV1 );
                    rDeletedEdges.Add( pDeletedBrepEdge );

                    SER(m_pOther->CombineCoincidentVertices(pOtherV2,pOtherV1,pDeletedOtherEdge));
                    RemoveRelationship(pV1,pOtherV1);
                    if (pDeletedBrepEdge && pDeletedOtherEdge)
                      {
                        // If these are not alreday a mated pair, then calling
                        // RemoveRelationship() can leave the maps with different
                        // sizes, and containing stale pointers.  [B460]
                        if ( IsMatedPair( pDeletedBrepEdge, pDeletedOtherEdge ) )
                          { RemoveRelationship(pDeletedBrepEdge,pDeletedOtherEdge); }
                        else
                          {
                            SmEdge *pMate = (SmEdge*)GetOtherMate( pDeletedBrepEdge );
                            if ( pMate != NULL )
                              { RemoveRelationship( pDeletedBrepEdge, pMate ); }
                            pMate  = (SmEdge*)GetBrepMate( pDeletedOtherEdge );
                            if ( pMate != NULL )
                              { RemoveRelationship( pMate, pDeletedOtherEdge ); }
                          }
                      }
                    rbModifiedTopology = TRUE;
                    break; // V1 has gone away break out of loop and get another V1 and test it.
                  } // end found a vertex pair with no large common edges connecting them check
              } // end two vertices are close enough check (dist < 10*tol)
          } // end iter every other single vertex looking for pairs
      } // end iter every single vertex
  } // end scope block

  // GWC: this following block of code is not being used
  // (All it did was call TwoPointIntersection(), which was commented out
  // and never implemented.)

//        // when no changes have been made to the topology
//        if (!rbModifiedTopology)
//          {
//            // Try using the two point intersector between faces shared
//            // by the two verices
//
//            ULONG ll ;
//            SmTArray<SmFace*> sV1FacesBrep, sV1FacesOther;
//            SmTArray<SmFace*> sV2FacesBrep, sV2FacesOther;
//            SmTArray<SmFace*> sCommonBrepFaces, sCommonOtherFaces;
//
//            // for every single vertex
//            for (ii=0; ii<crSingleVertices.GetSize(); ii++)
//              {
//                SmVertex *pV1      = crSingleVertices[ii];
//                SmVertex *pV1Other = (SmVertex*)GetOtherMate(pV1);
//                if (!pV1Other) continue;
//
//                pV1->GetFaces(sV1FacesBrep);
//                pV1Other->GetFaces(sV1FacesOther);
//
//                // for every other single vertex
//                for (jj=ii+1; jj<crSingleVertices.GetSize(); jj++)
//                  {
//                    SmVertex *pV2 = crSingleVertices[jj];
//
//                    pV2->GetFaces(sV2FacesBrep);
//                    SmVertex *pV2Other = (SmVertex*)GetOtherMate(pV2);
//                    if (!pV2Other) continue;
//                    pV2->GetFaces(sV2FacesBrep);
//                    pV2Other->GetFaces(sV2FacesOther);
//
//                    // look for common faces between the two vertices
//                    sV1FacesBrep.FindCommonElements(sV2FacesBrep,sCommonBrepFaces);
//                    sV1FacesOther.FindCommonElements(sV2FacesOther,sCommonOtherFaces);
//
//                    // for every common face
//                    for (kk=0; kk<sCommonBrepFaces.GetSize(); kk++)
//                      {
//      //                    SmFace *pBrepFace = sCommonBrepFaces[kk];
//                        for (ll=0; ll<sCommonOtherFaces.GetSize(); ll++)
//                          {
//      //                        SmFace *pOtherFace = sCommonOtherFaces[ll];
//      //                        SER(TwoPointIntersection(pV1,pV2,pV1Other,pV2Other,pBrepFace,pOtherFace));
//                          }
//                      } // end iter every common face
//                  } // end iter every other single vertex looking for common faces between the vertices
//              } // end iter every single vertex
//          } // when no topology has been modified by the distance and squeeze checks

  // all done
  return SM_SUCCESS;

} // end SmTopologyIntersector::FixGaps



/*******************************************************************//**
PURPOSE: Remove the relationship between a brep entity and the other entity.

NOTES: Returns SM_ERR if the two inputs are not related.
***********************************************************************/
SmStatus SmTopologyIntersector::RemoveRelationship(SmTopology * pBrepEntity,
                                                   SmTopology * pOtherEntity)
{
    if ( ! IsMatedPair( pBrepEntity, pOtherEntity ) )
      { SER( SM_ERR ); }

    m_pBToO->Remove(pBrepEntity);
    m_pOToB->Remove(pOtherEntity);
    return SM_SUCCESS;

} // end SmTopologyIntersector::RemoveRelationship


/*******************************************************************//**
PURPOSE: Get the Other Brep entity mated to this Brep topology entity.
   If there is no relationship NULL is returned.

NOTES:
***********************************************************************/
SmObject *SmTopologyIntersector::GetOtherMate(SmObject * pBrepEntity) const
{
  if (m_pBToO)
    {
      return SM_REINTERPRET_CAST(SmObject*,m_pBToO->At(pBrepEntity));
    }
  else
    {
      return NULL;
    }
} // end SmTopologyIntersector::GetOtherMate

/*******************************************************************//**
PURPOSE:  Get the this Brep entity mated to Other topology entity.
   If there is no relationship NULL is returned.

NOTES:
***********************************************************************/
SmObject * SmTopologyIntersector::GetThisMate(SmObject * pOtherEntity) const
{
  if (m_pOToB)
    {
      return SM_REINTERPRET_CAST(SmObject*,m_pOToB->At(pOtherEntity));
    }
  else
    {
      return NULL;
    }
}// end SmTopologyIntersector::GetThisMate

/*******************************************************************//**
PURPOSE:  return TRUE when two objects are mated

NOTES:
***********************************************************************/
SmBoolean SmTopologyIntersector::IsMatedPair
  (SmObject * pBrepEntity,     // in : This Brep entity to check
   SmObject * pOtherEntity)    // in : Other Brep entity to check
 const
{
  if ( pBrepEntity == NULL || pOtherEntity == NULL )
    { return FALSE; }

  if (pOtherEntity == GetOtherMate(pBrepEntity))
    {
      SM_ASSERT(pBrepEntity == GetThisMate(pOtherEntity)) ;
      return(TRUE) ;
    }
  else
    {
      SM_ASSERT(pBrepEntity != GetThisMate(pOtherEntity)) ;
      return(FALSE) ;
    }
} // end SmTopologyIntersector::IsMatedPair

/*******************************************************************//**
PURPOSE: Get the Brep entity from the related Other Brep topology entity.
   If there is no relationship NULL is returned.

NOTES:
***********************************************************************/
SmObject * SmTopologyIntersector::GetBrepMate // rtn: Brep matching entity or NULL
  (SmObject * pOtherEntity)                   // in : target OtherBrep entity to match
 const
{
  // when Brep to OtherBrep Map exists - OtherBrep to Brep map also exists
  if (m_pBToO) { SM_ASSERT(m_pOToB != NULL) ;

                 // fetch the matched entity - returns NULL for no match
                 return SM_REINTERPRET_CAST(SmObject*,m_pOToB->At(pOtherEntity));
               }
  else         { // no map exists - return NULL for no match
                 return NULL;
               }

} // end SmTopologyIntersector::GetBrepMate

/*******************************************************************//**
PURPOSE: Get list of all Other mapped Entities of requested type

NOTES:
***********************************************************************/
void SmTopologyIntersector::GetMappedOtherObjects
( 
  SM_TYPE eDesiredType,                    // in : oneof SmEdge_TYPE, SmVertex_TYPE, SmFace_TYPE, etc
  SmTArray<SmObject*>& rList               // in : Display point size
) const
{
  rList.ReSet();
#ifdef SM_DEFINED_HASH_ORDER
  static_assert(false, "not implemented");
#endif

  SmTArray<SmObject*> sValues;
  m_pBToO->GetAllValues(sValues);

  for (ULONG ii = 0; ii < sValues.GetSize(); ++ii)
  {
    if (sValues[ii]->GetType() == eDesiredType)
    {
      rList.Add(sValues[ii]);
    }
  }
} // end SmTopologyIntersector::GetMappedOtherObjects

/*******************************************************************//**
PURPOSE: Get list of all Brep mapped Entities of requested type

NOTES:
***********************************************************************/
void SmTopologyIntersector::GetMappedBrepObjects
(
  SM_TYPE eDesiredType,          // in : oneof SmEdge_TYPE, SmVertex_TYPE, SmFace_TYPE, etc.
  SmTArray<SmObject *> &rList    // in : Display point size
) const
{
  rList.ReSet() ;
#ifdef SM_DEFINED_HASH_ORDER
  static_assert(false, "not implemented");
#endif

  SmTArray<SmObject*> sValues;
  m_pOToB->GetAllValues(sValues);

  for (ULONG ii = 0; ii < sValues.GetSize(); ++ii)
  {
    if (sValues[ii]->GetType() == eDesiredType)
    {
      rList.Add(sValues[ii]);
    }
  }
} // end SmTopologyIntersector::GetMappedBrepObjects

/*******************************************************************//**
PURPOSE: call Notify(SM_NO_COINCIDENT,...) on every MappedObject 
         pair for attribute propagation

NOTES:
***********************************************************************/
void SmTopologyIntersector::CoincidenceNotify() const
{
#ifdef SM_DEFINED_HASH_ORDER
    static_assert(false, "not implemented");
#endif

  SmTArray<SmObject*> sKeys, sVals;
  m_pBToO->GetAllKeyValuePairs(sKeys, sVals);

  // for every entry in the map list
  for (ULONG ii = 0; ii < sKeys.GetSize(); ++ii)
  {
    SmObject* pBrepKey  = sKeys[ii];
    SmObject* pOtherVal = sVals[ii];

    SM_ASSERT( IsMatedPair( pBrepKey, pOtherVal ) );

    // Get BrepA Key type
    SM_TYPE lType = pBrepKey->GetType();

    // when type is the requested type
    if ( lType == SmFace_TYPE
// Remove Composites
//         || lType == SmCFace_TYPE
         || lType == SmEdge_TYPE
// Remove Composites
//         || lType == SmCEdge_TYPE
         || lType == SmVertex_TYPE )
    {
        // coincidence Notify
        m_pBrep->Notify( SM_NO_COINCIDENT, // in : caller = pBrep
                         pBrepKey,          // in : pData1 = BrepObj
                         pOtherVal,         // in : pData1 = OtherObj
                         m_pOther );        // in : pData3 = Other

    } // end Desired Type check
  } // end iter every mapped entity entry
} // end SmTopologyIntersector::CoincidenceNotify

/*******************************************************************//**
PURPOSE: Get a list of the edges which are common edges between the
    two Breps.

NOTES:
METHOD --- A linear search of all Brep Edges making a linear
           search of all Related Edges to find related pairs
***********************************************************************/
void SmTopologyIntersector::GetCommonEdges
  (SmTArray<SmEdge*> & rEdges,
   SmTArray<SmEdge*> & rOtherEdges)
 const
{
  // init output
  rEdges.ReSet();
  rOtherEdges.ReSet();

  // get all m_pBrep Edges
  SmTArray<SmEdge*> sEdges;
  m_pBrep->GetEdges(sEdges);

  // for every m_pBrep Edge
  ULONG ii;
  for (ii=0; ii<sEdges.GetSize(); ii++)
    {
      SmEdge *pEdge = sEdges[ii];

      // see if it is related to an m_pOther Edge
      SmEdge *pOEdge = (SmEdge*)GetOtherMate(pEdge);

      // add related edge pairs to the output
      if (pOEdge)
        {
          rEdges.Add(pEdge);
          rOtherEdges.Add(pOEdge);
        }
    } // end iter every m_pBrep Edge

} // end SmTopologyIntersector::GetCommonEdges

/*******************************************************************//**
PURPOSE: Get a list of the vertices which are common vertices between the
    two Breps.

NOTES:
***********************************************************************/
void SmTopologyIntersector::GetCommonVertices
  (SmTArray<SmVertex*> & rVertices,
   SmTArray<SmVertex*> & rOtherVertices)
  const
{
  // init output
  rVertices.ReSet();
  rOtherVertices.ReSet();

  // get all m_pBrep Vertices
  SmTArray<SmVertex*> sVertices;
  m_pBrep->GetVertices(sVertices);

  // for every vertex
  ULONG ii;
  for (ii=0; ii<sVertices.GetSize(); ii++)
    {
      // see if Vertex has an m_pOther mate
      SmVertex *pVertex = sVertices[ii];
      SmVertex *pOVertex = (SmVertex*)GetOtherMate(pVertex);

      // add mated pairs to the output arrays
      if (pOVertex)
        {
          rVertices.Add(pVertex);
          rOtherVertices.Add(pOVertex);
        }
    }

} // end SmTopologyIntersector::GetCommonVertices

/*******************************************************************//**
PURPOSE: Insert intersection geometry into the Breps making relationships
    between topology that represents common intersection geometry.

NOTES:

METHOD ---
    1. Intersect the geometry of these two breps,
    2. insert the solutions (points, curves, surfaces) into the corresponding topologies
          making topology entities for each solution in both breps, and
    3. set up the relationships between the mated topology entities that
          represent common intersection geometry in the two Breps.

***********************************************************************/
SmStatus SmTopologyIntersector::IntersectInsertRelate()
{
#ifdef SM_VALIDATE_TOPOLOGY
  smos_WriteBuffer(_T("SmTopologyIntersector - Validate Topology is ON!"));
  if(m_pBrep)  m_pBrep->ValidatePointers();
  if(m_pOther) m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

#ifdef SM_DEBUG_CODE
int iDebugLevel = -1; // m_pProtoTopoMgr->Dump(iDumpLevel) iDumpLevel: -1 = no output
                      //                                                0 = headers, ProtoVert, Edge, and Face Counts
                      //                                                5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                      //                                               10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                      //                                               15 = Brep1, Brep2, and SmTopologyIntersector dumps
SmBoolean bDebugFaceCount = FALSE;
  if ( bDebugFaceCount ) // output face counts
    {
      TCHAR sBuff[SM_TBLOCK_SIZE];
      ULONG lNumFaces1 = m_pBrep->GetNumFaces();
      ULONG lNumFaces2 = m_pOther->GetNumFaces();
      smos_sprintf(sBuff, _T("\nEnter Proto: Num Faces %ld   %ld\n"), lNumFaces1, lNumFaces2 );
      smos_WriteBuffer(sBuff);
      if ( lNumFaces1==8 && lNumFaces2==7 )
        { smos_WriteBuffer(sBuff); } //cbi breakpoint
    }
  
  if ( iDebugLevel > 0 ) // dump and draw Brep(blue) Other(green)
    {
      SM_DUMP_AND_ASSERT_VALID( m_pBrep );
      SM_DUMP_AND_ASSERT_VALID( m_pOther );
      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,1 ); m_pBrep ->Draw(TRUE); sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 0,1,0 ); m_pOther->Draw(TRUE); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
#ifdef SM_USE_TBB

    SmTArray<SmSurface*> surfs;
    SmTArray<SmSurface*> tmp;
    m_pBrep->GetSurfaces(surfs);
    m_pOther->GetSurfaces(tmp);
    surfs.Append(tmp);
    tbb::parallel_for(ULONG(0), surfs.GetSize(),
                      [surfs](ULONG i)
                      {
                          SmSurfaceCache* pSC1 = smsurf_GetSurfaceCache(surfs[i]);
                          if (pSC1)
                          {
                              pSC1->GetTree();
                          }
                      });

#endif
  // Capture Distance: simply using entities' tolerances will miss intersections
  // that could possibly go into the same ProtoVertex.  We should collect candidates
  // into the same PVs, and let Resolve() figure it out, making either one or more
  // SmVerttices out of each PV.
  // This is a concept in progress.  We currently check the tolerance value in
  // the TopologyIntersector, which ultimately comes from the tolerance in the Breps. [B669]
  // We might want to look at Bounding Box sizes, or min Edge length, or such.
  // But currently this works well.
  double dCaptureDistance = 0.01;

  // Make sure it's not smaller than our tolerance value.
  double dTopoIntTol = this->GetThisApproxTol3d();
  if ( dCaptureDistance < dTopoIntTol * 2000 )
    { dCaptureDistance = dTopoIntTol * 2000; }

  m_pProtoTopoMgr = new( m_pBrep ) SmProtoTopologyManager( m_pBrep, m_pOther, this, dCaptureDistance );
  // Did this to avoid a warning message in a test, but can't, because it gets installed
  // in new entities, but then m_crContext is deleted before the new entities are.
  //m_pProtoTopoMgr->SetContext( &m_crContext );

#ifdef SM_DEBUG_CODE      
if ( iDebugLevel > 0 ) // Before IntersectTopology(): DumpAndDraw Brep(blue), Other(green), m_pProtoTopoMgr(cyan)
  {
    SM_DUMP_AND_ASSERT_VALID(m_pBrep) ;
    SM_DUMP_AND_ASSERT_VALID(m_pOther) ;
    m_pProtoTopoMgr->Dump(iDebugLevel) ; // iDumpLevel: -1  = no output
                                         //              0  = headers, ProtoVert, Edge, and Face Counts
                                         //              5  = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                         //              10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                         //              15 = Brep1, Brep2, and SmTopologyIntersector dumps
    smgfx_Erase();
    smgfx_SetLook( 1,2, 0,0,1 ); m_pBrep ->Draw(TRUE); sm_GraphicsLoop();
    smgfx_SetLook( 1,2, 0,1,0 ); m_pOther->Draw(TRUE); sm_GraphicsLoop();
    smgfx_SetLook( 1,2, 0,1,1 ); m_pProtoTopoMgr->DrawDebug( 1 ); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // Intersect Topology
  SmStatus eStat = this->IntersectTopology();

  // If that didn't work, quit before we mess up the Breps.
  SER_MSG( eStat, _T("\nSmProtoExecutive: Failure returned from IntersectTopology().\n") );

#ifdef SM_DEBUG_CODE
if ( iDebugLevel > 0 ) // After IntersectTopology(): DumpAndDraw Brep(blue), Other(green), m_pProtoTopoMgr(cyan)
  {
    SM_DUMP_AND_ASSERT_VALID( m_pBrep );
    SM_DUMP_AND_ASSERT_VALID( m_pOther );
    m_pProtoTopoMgr->Dump( iDebugLevel ); // iDumpLevel: -1  = no output
                                          //              0  = headers, ProtoVert, Edge, and Face Counts
                                          //              5  = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                          //              10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                          //              15 = Brep1, Brep2, and SmTopologyIntersector dumps
    smgfx_Erase();
    smgfx_SetLook( 1,2, 0,0,1 ); m_pBrep ->Draw(TRUE); sm_GraphicsLoop();
    smgfx_SetLook( 1,2, 0,1,0 ); m_pOther->Draw(TRUE); sm_GraphicsLoop();
    smgfx_SetLook( 1,2, 0,1,1 ); m_pProtoTopoMgr->DrawDebug( 1 ); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // Resolve Intersections
  eStat = this->ResolveIntersections();

  // If that didn't work, quit before we mess up the Breps.
  SER_MSG( eStat, _T("\nSmProtoExecutive: Failure returned from ResolveProtoTopology().\n") );

#ifdef SM_DEBUG_CODE
if ( iDebugLevel > 0 ) // After ResolveIntersections(): DumpAndDraw Brep(blue), Other(green), m_pProtoTopoMgr(cyan)
  {
    SM_DUMP_AND_ASSERT_VALID( m_pBrep );
    SM_DUMP_AND_ASSERT_VALID( m_pOther );
    m_pProtoTopoMgr->Dump( iDebugLevel ); // iDumpLevel: -1 = no output
                                          //              0 = headers, ProtoVert, Edge, and Face Counts
                                          //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                          //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                          //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
    smgfx_Erase();
    smgfx_SetLook( 1,2, 0,0,1 ); m_pBrep ->Draw(TRUE); sm_GraphicsLoop();
    smgfx_SetLook( 1,2, 0,1,0 ); m_pOther->Draw(TRUE); sm_GraphicsLoop();
    smgfx_SetLook( 1,2, 0,1,1 ); m_pProtoTopoMgr->DrawDebug( 1 ); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // Imprint Intersections, now we'll modify the Breps.
  eStat = this->ImprintIntersections();

  // If that didn't work, inform the public
  SE_MSG( eStat, _T("\nSmProtoExecutive: Error returned from ImprintIntersections().\n") );

#ifdef SM_DEBUG_CODE
if ( iDebugLevel > 0 ) // After ImprintIntersections(): DumpAndDraw Brep(blue), Other(green), m_pProtoTopoMgr(cyan)
  {
    SM_DUMP_AND_ASSERT_VALID( m_pBrep );
    SM_DUMP_AND_ASSERT_VALID( m_pOther );
    m_pProtoTopoMgr->Dump( iDebugLevel ); // iDumpLevel: -1 = no output
                                          //              0 = headers, ProtoVert, Edge, and Face Counts
                                          //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                          //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                          //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
    smgfx_Erase();
    smgfx_SetLook( 1,2, 0,0,1 ); m_pBrep ->Draw(TRUE); sm_GraphicsLoop();
    smgfx_SetLook( 1,2, 0,1,0 ); m_pOther->Draw(TRUE); sm_GraphicsLoop();
    smgfx_SetLook( 1,2, 0,1,1 ); m_pProtoTopoMgr->DrawDebug( 1 );
    sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

//cbi move to inside ImprintIntersections():
//  // At this point, there might be coincident Vertices and Edges in the Breps.
//  // The rest of ManifoldBoolean() expects coincident Vertices to be glued
//  // (although not coincident Edges).  So do that.  [Offset demo 2]
//
//  eStat = pProto->GlueCoincidentBrepVertices();

  // Set relationship maps.
  eStat = this->SetRelationships();

  // all done - clean up and exit
  if(m_pProtoTopoMgr) { delete m_pProtoTopoMgr ; m_pProtoTopoMgr = NULL; }
  return eStat;

} // end SmTopologyIntersector::IntersectInsertRelate

/*******************************************************************//**
PURPOSE: Static helper: Return whether a PointClassificationType can be imprinted.

USAGE NOTES---- 
***********************************************************************/
static SmBoolean sm_CanImprint( SmPointClassificationType eType )
{
  return(   eType == SM_PC_REGION
         || eType == SM_PC_FACE
         || eType == SM_PC_EDGE
         || eType == SM_PC_VERTEX) ;

} // end sm_CanImprint

/*******************************************************************//**
PURPOSE: Static helper: Find the solution in an array with the lowest-dimensional object.

USAGE NOTES---- 
   Finds the first instance of such a solution.
   If no usable result, return value == rSols.GetSize().
***********************************************************************/
static ULONG sm_FindFirstLowestDimensionSol( SmSolutionArray &crSolutions )
{
  ULONG lWhichSol = 0; // Index of solution to use.

  ULONG ii, lNumSols = crSolutions.GetSize();
  if ( lNumSols < 1 )
    { return lNumSols; }  // Indicates no usable results.

  if (lNumSols > 1)
  {
      lWhichSol = lNumSols; // Would indicate nothing found.

      // Choose the lowest-dimension result: Vertex, then Edge, then Face.
      ULONG lWhichV, lWhichE, lWhichF;
      lWhichV = lWhichE = lWhichF = lNumSols;

      // Use the first one found (of each type).
      for ( ii=0; ii<lNumSols; ii++ )
      {
          const SmSolution &rSol = crSolutions[ii];
          ULONG iType = rSol.m_apObjects[0]->GetType();

          if (iType == SmVertex_TYPE)
          {
              lWhichV = ii;
              break;  // Done: use first.
          }
          else if (iType == SmEdge_TYPE)
          {
              if (lWhichE == lNumSols) { lWhichE = ii; }
          }
          else if (iType == SmFace_TYPE)
          {
              if (lWhichF == lNumSols) { lWhichF = ii; }
          }
      } // end iter every solution

      // lWhichSol defaults to lNumSols if there are none of these,
      // that should not happen.
      SM_ASSERT(   lWhichV < lNumSols
                || lWhichE < lNumSols
                || lWhichF < lNumSols);

      if      (lWhichV < lNumSols) { lWhichSol = lWhichV; }
      else if (lWhichE < lNumSols) { lWhichSol = lWhichE; }
      else if (lWhichF < lNumSols) { lWhichSol = lWhichF; }
      else { SE(SM_ERR); }

  } // end more than 1 solution check

  return lWhichSol;

} // end static sm_FindFirstLowestDimensionSol


/*******************************************************************//**
PURPOSE: Static helper: Intersect Shell Vertices in pBrep1 with pBrep2.

USAGE NOTES---- 
   Creates a ProtoVertex in the ProtoTopology Manager.
***********************************************************************/
static SmStatus sm_IntersectShellVertices( SmBrep *pBrep1, // in: Brep whose vertices to check.
                                           SmBrep *pBrep2, // in: Brep to check Brep1 vertices against.
                                           SmBoolean bSwapOrder, // in: which order in pProtoTopoMgr.
                                           SmProtoTopologyManager *pProtoTopoMgr ) // in/out: 
{
  SmTArray<SmVertex*> sVertexList;

#ifdef CBI_CACHE   //cbi: try caching later; 1st pass just do all topo.
  // Get or create caches and bounding boxes for each of the Breps
  SmExtent3d sBBox1, sBBox2;
  SmBrepCache *pBrepCache1 = (SmBrepCache*)SmCacheMgr::GetOrCreateObjectCache(SM_OC_BREP,pBrep1 );
  SmBrepCache *pBrepCache2 = (SmBrepCache*)SmCacheMgr::GetOrCreateObjectCache(SM_OC_BREP,pBrep2);
  NER(pBrepCache1);
  NER(pBrepCache2);
  //pBrepCache1->GetBBox( sBBox1 );
  //pBrepCache2->GetBBox( sBBox2 );

  // Get the vertices of Brep that are in the common box, and store them in a local array.
  pBrepCache1->GetVertexTree()->GetObjectListInBox( sCommonBox, sObjs1 );

  ULONG ii, lNumObjs = sObjs1.GetSize();
  for ( ii=0; ii<lNumObjs; ii++ )
    { sVertexList.Add( (SmVertex*)( sObjs1[ii]->m_pObject )); }
#else  // CBI_CACHE
  pBrep1->GetVertices( sVertexList );
  ULONG ii, lNumObjs = sVertexList.GetSize();
#endif  // CBI_CACHE

  SmSolutionArray sSolutions;
  SmProtoVertex *pNewPV = NULL;

  for ( ii=0; ii<lNumObjs; ii++ )
  {
      SmVertex *pVertex = sVertexList[ii];

      // Only shell vertices:
      if ( ! pVertex->IsShellVertex() ) { continue; }

      // Tol: BrepPointSolve() adds this to the Topology's tolerance.
      SmZoneTol3d sVtxZoneTol = SmTol::GetZoneTol3d( pVertex );

      // intersect pVertex with Brep
      double dBestAnswer=0.0;
      {
          SmTemporaryChangeValue<SmBoolean> sChange( pBrep2->m_bEditingEnabled, FALSE );

          SER( SmTopologySolver::BrepPointSolve( pBrep2,
                                                 pVertex->GetPoint(),
                                                 SM_SO_INTERSECT,
                                                 SM_SR_ALL,
                                                 sVtxZoneTol,
                                                 dBestAnswer, NULL,
                                                 sSolutions ));

      } // end scope for pBrep2->m_bEditingEnabled.

      // no work - no intersections
      ULONG lNumSols = sSolutions.GetSize();
      if ( lNumSols < 1 ) { continue; }

      // Find the lowest-dimensional result: Vertex first, then Edge, then Face.
      ULONG lWhichSol = sm_FindFirstLowestDimensionSol( sSolutions );

      if ( lWhichSol >= lNumSols )
        { continue; }  // No usable sols.

      // Create a PointClassification for each Brep, and a ProtoVertex from those.
      // The PointClass objects get copied into the new PV.
      SmPointClassification sPC1( sVtxZoneTol, pBrep1->GetContext() );
      sPC1.SetClassObject( SM_PC_VERTEX, pVertex );

      SmSolution & rSol = sSolutions[ lWhichSol ];
      SmPointClassification sPC2( sVtxZoneTol, pBrep2->GetContext() );
      SER( sPC2.LoadFromIntersection(SmTol::GetZoneTol3d(pVertex),rSol.m_apObjects[0], rSol.m_vStart, 0, SM_BIG_DOUBLE ));

      if ( bSwapOrder )
        { pProtoTopoMgr->FindOrCreateProtoVertex( sPC2, sPC1, pNewPV ); }
      else
        { pProtoTopoMgr->FindOrCreateProtoVertex( sPC1, sPC2, pNewPV ); }

  } // end for each m_Brep Vertex

  return SM_SUCCESS;

} // end sm_IntersectShellVertices


/*******************************************************************//**
PURPOSE: Static helper: Intersect Wire and optionally Lamina Edges in pBrep1 with pBrep2.

USAGE NOTES---- 
   Creates a ProtoEdge in the ProtoTopology Manager.
***********************************************************************/
static SmStatus sm_IntersectWireLaminaEdges( SmBrep *pBrep1,       // in: Brep whose vertices to check.
                                             SmBrep *pBrep2,       // in: Brep to check Brep1 vertices against.
                                             SmBoolean bDoLamina,  // in: do lamina as well as wire edges?
                                             SmBoolean bSwapOrder, // in: which order in pProtoTopoMgr.
                                             SmProtoTopologyManager *pProtoTopoMgr )
{

  // Locals
  SmSolutionArray sSolutions;
  SmProtoEdge *pNewPE = NULL;
  ULONG ii, jj;
  SmStatus eStat;

#ifdef CBI_CACHE   //cbi: try caching later; 1st pass just do all topo.
    // Get the curves of Brep that are in the common box, and store them in a local array.
    pBrepCache1 = (SmBrepCache*)SmCacheMgr::GetOrCreateObjectCache(SM_OC_BREP,pBrep1);
    NER(pBrepCache1);
    pBrepCache1->GetCurveTree()->GetObjectListInBox( sCommonBox, sObjs1 );

    ULONG lNumObjs = sObjs1.GetSize();
    SmTArray<SmCurve*> sCurveList;
    for ( ii=0; ii<lNumObjs; ii++ )
      {
        SmCurve* pCrv = (SmCurve*)( sObjs1[ii]->m_pObject );
        sCurveList.Add( pCrv );
      }
#else  // CBI_CACHE
  SmTArray< SmEdge* > sEdgeList;
  pBrep1->GetEdges( sEdgeList );
  ULONG lNumObjs = sEdgeList.GetSize();
#endif  // CBI_CACHE


  for ( ii=0; ii<lNumObjs; ii++ )
  {
      SmEdge *pEdge = sEdgeList[ii];
      if( pEdge == NULL )
        { continue; }

      SmBoolean bProcessThis = pEdge->IsWire();
      bProcessThis |= ( bDoLamina && pEdge->IsLamina() );
      if ( ! bProcessThis )
        { continue; }

      //cbi from IIREdgeBrep():
      // select SolverType - handle wires differently unless bForceFaceIntersections == TRUE
      SmSolverOperationType eSolverOp =   (   TRUE    //cbi  bForceFaceIntersections
                                           || !pEdge->IsWire())
                                        ? SM_SO_INTERSECT
                                        : SM_SO_INTERSECT_WIREFRAME ;
      {
          // temporarily disable EditingEnabled bit
          SmTemporaryChangeValue<SmBoolean> sChange( pBrep2->m_bEditingEnabled, FALSE );

          // Intersect Edge->Curve with Brep
          eStat = SmTopologySolver::BrepCurveSolve( pBrep2,
                                       *pEdge->GetCurve(), pEdge->GetInterval(),
                                        eSolverOp, SM_SR_ALL, pEdge->GetTolerance(),
                                        SM_BIG_DOUBLE, NULL, sSolutions );
      }

      if ( eStat != SM_SUCCESS || sSolutions.GetSize() < 1 )
        { continue; }  // No ints for this Edge.

      // Construct first CurveClassification for all of pEdge.
      SmCurveClassification sCrvClass1( pEdge->GetCurve(), pEdge->GetInterval(),
                                        NULL, pEdge->GetTolerance() );

//cbi: there's a bunch more stuff in sm_SetEdgeClass( sCrvClass1, pEdge ).
//cbi  I don't think we need it.
//cbi  They also reverse it if primary EU is OPP.

      SmVertex *pVertex = pEdge->GetStartVertex();
      sCrvClass1[0].m_vStart.SetClassObject( SM_PC_VERTEX, pVertex );
      sCrvClass1[0].m_vMid  .SetClassObject( SM_PC_EDGE  , pEdge   );
      pVertex = pEdge->GetOtherVertex( pVertex );
      sCrvClass1[0].m_vEnd  .SetClassObject( SM_PC_VERTEX, pVertex );

      // Construct second CurveClassification for pBrep2 intersections.
      SmCurveClassification sCrvClass2( pEdge->GetCurve(), pEdge->GetInterval(),
                                        NULL, pEdge->GetTolerance() );

      // Add curve/Brep intersections to CrvClass2.
      if ( sCrvClass2.InsertTopologyIntersections( sSolutions, 0 ) != SM_SUCCESS )
        { continue; }

//cbi: might have to Fixup(), etc., as in SmTopologyIntersector::MergeCurveClasses() (line 2030)
      // Syncronize the two curve classifications.
      if ( sCrvClass1.Homogenize( sCrvClass2 ) != SM_SUCCESS )
        { return SM_ERR; }

      // Create ProtoEdges from each segment.
      for ( jj=0; jj<sCrvClass2.GetSize(); jj++ )
      {
          if ( bSwapOrder )
            { pProtoTopoMgr->FindOrCreateProtoEdge( sCrvClass2[jj], sCrvClass1[jj], pNewPE ); }
          else
            { pProtoTopoMgr->FindOrCreateProtoEdge( sCrvClass1[jj], sCrvClass2[jj], pNewPE ); }
      }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
      if (bDebugMe || lCount == lDebugCount)
      {
          sSolutions.Dump() ;
          sCrvClass1.Dump();
          sCrvClass2.Dump();

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1); pBrep1->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(3,4, 1,1,0); pEdge->Draw() ; sm_GraphicsLoop() ;
          smgfx_SetLook(5,6, 1,0,0); sSolutions.Draw() ; sm_GraphicsLoop() ;
          //  smgfx_SetLook(5,6, 1,0,0); sCrvClass1->Draw() ; sm_GraphicsLoop() ;
          //  smgfx_SetLook(5,6, 1,0,0); sCrvClass2->Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE


  } // end for each pBrep1 edge

  return SM_SUCCESS;

}  // end sm_IntersectWireLaminaEdges

/*******************************************************************//**
PURPOSE: Intersect all topology in our two Breps.

USAGE NOTES---- 
   Results are stored in our SmProtoTopologyManager object.
***********************************************************************/
SmStatus SmTopologyIntersector::IntersectTopology()
{
  // Init output
  m_pProtoTopoMgr->Reset();

  // Locals, for Face-Face intersections
  SmTArray< SmCurve * > s3dCurves;
  SmTArray< SmCurve * > sUVCurves1;
  SmTArray< SmCurve * > sUVCurves2;
  SmTArray< SmPoint3d > sIntPts;
  SmTArray< SmCurveClassification* > sCCs1;
  SmTArray< SmCurveClassification* > sCCs2;
  SmTArray< SmCurve * > sCC3dCurves;
  SmTArray< SmCurve * > sCCUVCurves1;
  SmTArray< SmCurve * > sCCUVCurves2;

  SmTArray< SmFace* > sFaces1;
  SmTArray< SmFace* > sFaces2;
  m_pBrep ->GetFaces( sFaces1 );
  m_pOther->GetFaces( sFaces2 );

  SmStatus eStat, eStat1, eStat2 ;
  ULONG ii, jj, kk, ll;
  ULONG lNumFaces1 = sFaces1.GetSize();
  ULONG lNumFaces2 = sFaces2.GetSize();
  SmBoolean bDoLamina;

#ifdef SM_DEBUG_CODE
ULONG     di ;
SmBoolean bDebugMe   = FALSE ;         // Dump&Draw every FacePair before XSect, 
                                       // Dump&Draw every FacePair XSect results, 
                                       // Dump&Draw after each CreateProtoFace for every Coincident FacePair, 
                                       // Dump&Draw after all FacePair XSects, 
                                       // Dump&Draw before exiting method 
SmBoolean bDebugMe2  = FALSE ;         // also Dump&Draw each CurveClassification XSect result Pair, 
                                       //      Dump&Draw after each CreateProtoEdge for every XSect IntervalSolution
                                       //      Dump&Draw after each CreateProtoVertex for every XSect PtSolution
SmBoolean bDebugMe3  = FALSE ;         // also Dump&Draw after all FacePair[ii][all] XSects, 
                                       //      Dump&Draw after every ProtoEdge pair SplitIntersectingProtoEdges() call, 
                                       //      Dump&Draw after every ProtoEdge/ProtoVertex RemoveCoincidentNoEdgeProtoVertex() call
int       iDumpLevel = 5 ;
TCHAR     sDebugBuff[SM_TBLOCK_SIZE] ;
#endif // SM_DEBUG_CODE

  // Process ShellVertex Intersections.

  // Do m_pBrep->ShellVertex Intersections with m_Other.
  eStat = sm_IntersectShellVertices( m_pBrep, m_pOther, FALSE, m_pProtoTopoMgr );

  // Do m_pOther->ShellVertex Intersections with m_Brep.
  eStat = sm_IntersectShellVertices( m_pOther, m_pBrep, TRUE,  m_pProtoTopoMgr );

  // Now do Wire Edges.

  // Do m_Brep's Wire/Lamina Edge intersections with m_pOther.
  bDoLamina = (   this->m_lIntersectLaminaEdges == 1    // 1: both Breps
               || this->m_lIntersectLaminaEdges == 2 ); // 2: m_pBrep
      
  eStat = sm_IntersectWireLaminaEdges( m_pBrep, m_pOther, bDoLamina, FALSE, m_pProtoTopoMgr );

  // Do m_Other's Wire/Lamina Edge intersections with m_pBrep.
  bDoLamina = (   this->m_lIntersectLaminaEdges == 1    // 1: both Breps
               || this->m_lIntersectLaminaEdges == 3 ); // 3: m_pOther

  eStat = sm_IntersectWireLaminaEdges( m_pOther, m_pBrep, bDoLamina, TRUE, m_pProtoTopoMgr );

  SM_REF1(eStat);
      
  // ====================================
  // Finally, the Face-Face intersections.
  // For face-face coincidences, we collect all the ProtoEdges for each pair of
  // coincident Surfaces, and then create one or more ProtoFaces from them.
  // (ProtoFaces exist only to handle coincident Faces.)
  SmTArray< SmProtoEdge* > sCoincPEs;
  double     dCoincidentDistance = 0.0;
  SmSurface *pContainingSurface  = NULL;

#ifdef SM_USE_TBB
  std::vector<SmTArray<SmCurve*>> boundaryCurves1(lNumFaces1);
  std::vector<SmTArray<SmCurve*>> boundaryCurves2(lNumFaces2);
  std::vector<SmObjsDelete<SmCurve*>> delBoundaryCurves1(lNumFaces1);
  std::vector<SmObjsDelete<SmCurve*>> delBoundaryCurves2(lNumFaces2);

    static tbb::task_arena sArena;
    sArena.execute([&]() {
        tbb::parallel_for(ULONG(0), lNumFaces1, [&sFaces1, &boundaryCurves1, &delBoundaryCurves1](ULONG i) {
        if (SmBSplineSurface* pSurf = dynamic_cast<SmBSplineSurface*>(sFaces1[i]->GetSurface()))
        {
            SmTArray<SmBSplineCurve*> sBdrysUV;
            SmTArray<SmOrientType> sOrients;
            if (pSurf->CreateNaturalUVTrimCurves(*(pSurf->GetContext()),
                pSurf->GetNaturalUVDomain(),
                SM_SP_V,
                FALSE,
                boundaryCurves1[i],
                sBdrysUV,
                sOrients) == SM_SUCCESS) 
            {
                delBoundaryCurves1[i].SetArray(&boundaryCurves1[i]);
                for (SmCurve * pCurve : boundaryCurves1[i])
                {
                  SmCacheMgr::GetOrCreateObjectCache(SM_OC_CURVE, pCurve);
                }
            }
            else
            {
		  	  for (SmCurve * pCurve : boundaryCurves1[i])
                {
		  	  	delete pCurve;
		  	  }
          
                boundaryCurves1[i].ReSet();
            }
		    for (SmCurve * pCurve : sBdrysUV)
            {
		      	delete pCurve;
		    }
          }
        });
        tbb::parallel_for(ULONG(0), lNumFaces2, [&sFaces2, &boundaryCurves2, &delBoundaryCurves2](ULONG i) {
        if (SmBSplineSurface* pSurf = dynamic_cast<SmBSplineSurface*>(sFaces2[i]->GetSurface()))
        {
            SmTArray<SmBSplineCurve*> sBdrysUV;
            SmTArray<SmOrientType>    sOrients;

            if (pSurf->CreateNaturalUVTrimCurves(*(pSurf->GetContext()),
                pSurf->GetNaturalUVDomain(),
                SM_SP_V,
                FALSE,
                boundaryCurves2[i],
                sBdrysUV,
                sOrients) == SM_SUCCESS)
            {
                delBoundaryCurves2[i].SetArray(&boundaryCurves2[i]);
                for (SmCurve * pCurve : boundaryCurves2[i])
                {
                    SmCacheMgr::GetOrCreateObjectCache(SM_OC_CURVE, pCurve);
                }
            }
            else
            {
                for (SmCurve* pCurve : boundaryCurves2[i])
                {
		  	      delete pCurve;
		  	  }
		        boundaryCurves2[i].ReSet();
	        }
            for (SmCurve* pCurve : sBdrysUV)
            {
                delete pCurve;
            }
        }
        });
      });
#endif // SM_USE_TBB

  // for every m_pBrep/m_pOther face pair - find and store coincidences and XSections
  for(ii=0;ii<lNumFaces1;ii++)
    {
      SmFace *pF1 = sFaces1[ii];

      // for every m_pBrep/m_pOther face pair - find and store coincidences and XSections
      for ( jj=0; jj<lNumFaces2; jj++ )
        {
          SmFace *pF2 = sFaces2[jj];

#ifdef SM_DEBUG_CODE
          if ( bDebugMe ) 
            {
              SmTArray<SmEdge*> sEdges1, sEdges2 ; 
              pF1->GetEdges(sEdges1) ;
              pF2->GetEdges(sEdges2) ;
              smos_WriteBuffer(_T("\n**************************************\\"));
              smos_sprintf(sDebugBuff, _T("\nXSect[ii=%3d][jj=%3d] - Pre XSect"), ii, jj) ; smos_WriteBuffer(sDebugBuff) ;
              smos_WriteBuffer(_T("\n\\**************************************"));
              m_pProtoTopoMgr->Dump( iDumpLevel ); // iDumpLevel: -1 = no output
                                                   //              0 = headers, ProtoVert, Edge, and Face Counts
                                                   //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                                   //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                                   //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
              smgfx_Erase();
              /* edges Face1 */ smgfx_SetLook(4,5, 0,0,1); smgfx_ChangeColor(FALSE) ; for(di=0;di<sEdges1.GetSize();di++) 
                                                             { smgfx_ChangeColor(di != 0) ; if(sEdges1[di]) sEdges1[di]->Draw() ; sm_GraphicsLoop() ; }
              /* Face1       */ smgfx_SetLook(2,3, .2,.2,.6); pF1->Draw(SM_DM_CROSSHATCH,8,8); sm_GraphicsLoop();
              /* edges Face2 */ smgfx_SetLook(6,7, 0,0,1); for(di=0;di<sEdges2.GetSize();di++)                         
                                                             { smgfx_ChangeColor(di != 0) ; if(sEdges2[di]) sEdges2[di]->Draw() ; sm_GraphicsLoop() ; }
              /* Face2       */ smgfx_SetLook(2,3, .6,.2,.2); smgfx_ChangeColor(TRUE) ; pF2->Draw(SM_DM_CROSSHATCH,8,8); sm_GraphicsLoop();
              /* TopoMgr     */ smgfx_SetLook(1,2, 0,0,0); m_pProtoTopoMgr->DrawDebug(TRUE) ; sm_GraphicsLoop() ;
              /* Brep1       */ smgfx_SetLook(1,2, .2,.2,.6); m_pBrep ->Draw(TRUE) ; sm_GraphicsLoop() ;
              /* Brep2       */ smgfx_SetLook(1,2, .6,.2,.2); m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // See if this surface/surface xsect has been saved.
          // Actually, this is currently used only to skip some intersection
          // attempts: if the caller knows that two surfaces do not intersect,
          // it can register them here.  The Offset algorithm does that.
          SmBoolean bHaveSSI;
          SmTArray< SmTsectCurveType > sCurveTypes;
          SmTArray< double > sDeviations;
          this->CheckIfHaveExistingSSI(m_crContext,
                                       pF1->GetSurface(),
                                       pF2->GetSurface(),
                                       bHaveSSI,           // out: yes or no.
                                       s3dCurves,          // out: intersection data...
                                       sUVCurves1,         // out:
                                       sUVCurves2,         // out:
                                       sCurveTypes,        // out:
                                       sDeviations) ;      // out:
          
          if(bHaveSSI)
            { continue ; }
          
          // No existing SSI, check coincidence or intersection.
          SmBoolean bAreCoincident = FALSE ;
          
#ifdef SM_USE_NEWTOL
          SM_NEWTOL_LINE double dTol = SmTolerance::GetXSectTol3d( pF1, pF2, m_dThisApproxTol3d );
#else // SM_USE_OLDTOL
          SM_OLDTOL_LINE double dTol = this->GetThisApproxTol3d();
#endif // SM_USE_OLDTOL

          // For coincident surfaces, all we have to do is Relate() them,
          // the rest is already done.  Create a ProtoFace.
          
          // Check for coincident surfaces -- partial coincidence is ok.
          // Tol: sum of Faces' tols (or default min).
          bAreCoincident = FALSE;
          sCoincPEs.ReSet();
          SER( pF1->GetSurface()->CoincidenceCheck
               ( *(pF2->GetSurface()), // in : target surface to compare
                 dTol,                 // in : max allowed distance sample points on coincident surfaces
                 bAreCoincident,       // out: TRUE = surfaces are the same to within tolerance
                 dCoincidentDistance,  // out: max distance between planes
                 FALSE,                // in : TRUE = Test for complete coincidence.
                                       //      FALSE= Test for any coincident overlap.
                 pContainingSurface)); // out: (Not used here.)
          
          double dAngTolDeg = 20.0;
          
          SER( pF1->FaceIntersect( *( m_pBrep->GetContext() ),  // in : context for new object construction 
                                    pF2,                        // in : other target face 
                                    dTol,                       // in : distance Tolerance used in surf/surf intersection 
                                    dAngTolDeg,                 // in : Angle Tolerance in degrees 
                                    s3dCurves,                  // out: all 3DCurves of intersection between the two faces 
                                  & sUVCurves1,                 // out: optional thisFace UVTrimCurves for each intersection 3DCurve, NULL to ignore 
                                  & sUVCurves2,                 // out: optional otherFace UVTrimCurves for each intersection 3DCurve, NULL to ignore 
                                  & sIntPts,                    // out: optional single-point intersections
                                  & sCCs1,                      // out: optional thisFace  curveClassifications for all face/face intersections 
                                  & sCCs2,                      // out: optional otherFace curveClassifications for all face/face intersections 
                                  & sCC3dCurves,                // out: optional 3D Curves used by OptCCs for memory cleanup      for all face/face intersections
                                  & sCCUVCurves1,               // out: optional UV Curves used by OptThisCCs for memory cleanup  for all face/face intersections
                                  & sCCUVCurves2,               // out: optional UV Curves used by OptOtherCCs for memory cleanup for all face/face intersections
                                  & bAreCoincident ));          // in : opt: If the Surfaces are known to be coincident (or not) 
          
#ifdef SM_DEBUG_CODE
          if ( bDebugMe ) 
            {
              m_pProtoTopoMgr->Dump( iDumpLevel ); // iDumpLevel: -1 = no output
                                                   //              0 = headers, ProtoVert, Edge, and Face Counts
                                                   //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                                   //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                                   //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
              // header
              smos_sprintf(sDebugBuff, _T("\n/********************\nSmTopologyIntersector:FaceIntersect Results: bRaeCoincident:[%s], 3dCurveCnt:[%ld], SinglePntXSectCnts:[%ld], ThisFaceCrvClassificationCnt:[%ld], OtherFaceCrvClassCnt:[%ld]\n**************\n\n"), 
                                       bAreCoincident ? _T("TRUE") : _T("FALSE"),
                                       s3dCurves.GetSize(), 
                                       sIntPts.GetSize(), 
                                       sCCs1.GetSize(), 
                                       sCCs2.GetSize());
              smos_WriteBuffer(sDebugBuff);

              smgfx_Erase();
              /* TopoMgr                    */ smgfx_SetLook(1,2, 0,0,0); m_pProtoTopoMgr->DrawDebug(TRUE) ; sm_GraphicsLoop() ;
              /* Brep1 CurveClassifications */ for(di=0;di<sCCs1.GetSize();di++) 
                                                 { smgfx_SetLook(2,3, 0,0,1); sCCs1[di]->Draw(FALSE) ; sCCs1[di]->Dump() ; sm_GraphicsLoop() ; } /* FALSE: don't draw Objects */ 
              /* Brep2 CurveClassifications */ for(di=0;di<sCCs2.GetSize();di++) 
                                                 { smgfx_SetLook(2,3, 0,1,1); sCCs1[di]->Draw(FALSE) ; sCCs2[di]->Dump() ; sm_GraphicsLoop() ; } /* FALSE: don't draw Objects */ 
              /* intersection Curves3d      */ for(di=0;di<s3dCurves.GetSize();di++) 
                                                 { smgfx_SetLook(4,5, 1,0,0); s3dCurves[di]->DrawParams(); sm_GraphicsLoop(); sm_GraphicsLoop(); }
              /* intersection points3d      */ for(di=0;di<sIntPts.GetSize();di++)   
                                                 { smgfx_SetLook(10,11, 1,0,1); sIntPts[di].Draw(); sm_GraphicsLoop(); sm_GraphicsLoop(); }
              /* Face1                      */ smgfx_SetLook(2,3, .2,.2,.6); pF1->Draw(SM_DM_CROSSHATCH,8,8); sm_GraphicsLoop();
              /* Face2                      */ smgfx_SetLook(2,3, .6,.2,.2); pF2->Draw(SM_DM_CROSSHATCH,8,8); sm_GraphicsLoop();
              /* Brep1                      */ smgfx_SetLook(1,2, 0,0,1); m_pBrep ->Draw(TRUE) ; sm_GraphicsLoop() ;
              /* Brep2                      */ smgfx_SetLook(1,2, 0,1,0); m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
              sm_GraphicsLoop(); 
            }
#endif // SM_DEBUG_CODE

          // All of the curves created in FaceIntersect() will be copied into
          // new EdgeDefinition objects.  These curves are not consumed, so we will
          // have to delete them.
          SmObjsDelete< SmCurve* > sClean1A( & s3dCurves    );
          SmObjsDelete< SmCurve* > sClean2A( & sUVCurves1   );
          SmObjsDelete< SmCurve* > sClean3A( & sUVCurves2   );
          SmObjsDelete< SmCurve* > sClean1B( & sCC3dCurves  );
          SmObjsDelete< SmCurve* > sClean2B( & sCCUVCurves1 );
          SmObjsDelete< SmCurve* > sClean3B( & sCCUVCurves2 );
          SmObjsDelete< SmCurveClassification* > sClean1C( & sCCs1 );
          SmObjsDelete< SmCurveClassification* > sClean2C( & sCCs2 );
          
          // low work - no curve or point XSects to process. [Sweep 25 27 28]
          //  (have to process single-point intersections.  [bbu 2 iter 49, e.g.])
          if(s3dCurves.GetSize() == 0  &&  sIntPts.GetSize() == 0)
            { continue; }
          
          //cbi Also: we have to delete the curves in the CC's.  Same 3d curve in both.
            
          // for each CurveClass piece (CurveInterval) that's on both Faces:
          //  Start: find existing PV; if not, create one.
          //  End  : ditto.
          //  Create a PE, connect to both PV's.

          SmBoolean bWasCreated = FALSE;
          ULONG lNumCCs = sCCs1.GetSize();
          SM_ASSERT( sCCs2.GetSize() == lNumCCs );

          // for each CurveClassification pair returned from intersector
          for(kk=0;kk<lNumCCs;kk++)
            {
              SmCurveClassification *pCC1 = sCCs1[ kk ];
              SmCurveClassification *pCC2 = sCCs2[ kk ];

              ULONG lNumIvls = pCC1->GetSize();
              SM_ASSERT( pCC2->GetSize() == lNumIvls );

              // Make sure that all PointClasses can evaluate their positions.
              pCC1->SetPointClassParameters();
              pCC2->SetPointClassParameters();

#ifdef SM_DEBUG_CODE
              if ( bDebugMe2 ) 
                {
                  smos_WriteBuffer(_T("\n**************************************\\"));
                  smos_sprintf(sDebugBuff, _T("\nClassCurve[kk=%3d] - Post XSect[ii=%3d][jj=%3d]"), kk, ii, jj) ; smos_WriteBuffer(sDebugBuff) ;
                  smos_WriteBuffer(_T("\n\\**************************************"));
                  m_pProtoTopoMgr->Dump( iDumpLevel ); // iDumpLevel: -1 = no output
                                                       //              0 = headers, ProtoVert, Edge, and Face Counts
                                                       //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                                       //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                                       //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
                  smgfx_Erase();
                  /* TopoMgr              */ smgfx_SetLook(1,2, 0,0,0); m_pProtoTopoMgr->DrawDebug(); sm_GraphicsLoop();       /* ProtoTopoMgr              */                  
                  /* CurveClassification1 */ smgfx_SetLook(2,3, 0,0,1); pCC1->Draw(FALSE) ; pCC1->Dump() ; sm_GraphicsLoop() ; /* Brep1 CurveClassification */ /* FALSE: don't draw Objects */ 
                  /* CurveClassification2 */ smgfx_SetLook(2,3, 0,1,1); pCC2->Draw(FALSE) ; pCC2->Dump() ; sm_GraphicsLoop() ; /* Brep2 CurveClassification */ /* FALSE: don't draw Objects */ 
                  /* Face1                */ smgfx_SetLook(2,3, .2,.2,.6); pF1->Draw(SM_DM_CROSSHATCH,8,8); sm_GraphicsLoop();
                  /* Face2                */ smgfx_SetLook(2,3, .6,.2,.2); pF2->Draw(SM_DM_CROSSHATCH,8,8); sm_GraphicsLoop();
                  /* Brep1                */ smgfx_SetLook(1,2, 0,0,1); m_pBrep ->Draw(TRUE) ; sm_GraphicsLoop() ;
                  /* Brep2                */ smgfx_SetLook(1,2, 0,1,0); m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE

              for(ll=0;ll<lNumIvls;ll++)
                {
                  SmProtoEdge *pNewPE = NULL;
                  eStat = m_pProtoTopoMgr->FindOrCreateProtoEdge((*pCC1)[ll],    // in : Face1 CrvInterval classification from a face pair XSection
                                                                 (*pCC2)[ll],    // in : Face2 CrvInterval classification from a face pair XSection
                                                                 pNewPE,         // out: new or found ProtoEdge
                                                                 &bWasCreated) ; // out: optional, TRUE  = rpNewPE was created, NULL to ignore, default:[NULL]
                                                                                 //                FALSE = rpNewPE was found
                  // remember pNewPE is part of the trim boundary of this coincident face pair 
                  if(bAreCoincident && pNewPE != NULL)    /* cbi removed:  && bWasCreated */   
                    { sCoincPEs.AddUnique(pNewPE) ; }
#ifdef SM_DEBUG_CODE
                  if ( bDebugMe2 ) 
                    {
                      smos_WriteBuffer(_T("\n**************************************\\"));
                      smos_sprintf(sDebugBuff, _T("\nClassCurve[kk=%3d]->Interval[ll=%3d] - Post FindOrCreateProtoEdge, Post XSect[ii=%3d][jj=%3d]"), kk, ll, ii, jj) ; smos_WriteBuffer(sDebugBuff) ;
                      smos_WriteBuffer(_T("\n\\**************************************"));
                      m_pProtoTopoMgr->Dump( iDumpLevel ); // iDumpLevel: -1 = no output
                                                           //              0 = headers, ProtoVert, Edge, and Face Counts
                                                           //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                                           //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                                           //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
                      smgfx_Erase();
                      /* TopoMgr              */ smgfx_SetLook(1,2, 0,0,0);    m_pProtoTopoMgr->DrawDebug(); sm_GraphicsLoop();       /* ProtoTopoMgr              */
                      /* CurveClassification1 */ smgfx_SetLook(2,3, 0,0,1);    pCC1->Draw(FALSE) ; pCC1->Dump() ; sm_GraphicsLoop() ; /* Brep1 CurveClassification */ /* FALSE: don't draw Objects */ 
                      /* CurveClassification2 */ smgfx_SetLook(2,3, 0,1,1);    pCC2->Draw(FALSE) ; pCC2->Dump() ; sm_GraphicsLoop() ; /* Brep2 CurveClassification */ /* FALSE: don't draw Objects */ 
                      /* Face1                */ smgfx_SetLook(2,3, .2,.2,.6); pF1->Draw(SM_DM_CROSSHATCH,8,8); sm_GraphicsLoop();
                      /* Face2                */ smgfx_SetLook(2,3, .6,.2,.2); pF2->Draw(SM_DM_CROSSHATCH,8,8); sm_GraphicsLoop();
                      /* Brep1                */ smgfx_SetLook(1,2, 0,0,1);    m_pBrep ->Draw(TRUE) ; sm_GraphicsLoop() ;
                      /* Brep2                */ smgfx_SetLook(1,2, 0,1,0);    m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
                      sm_GraphicsLoop();
                    }
#endif // SM_DEBUG_CODE
                }

            } // end iter every CC returned from intersector

          // Process single-point intersections.  Example: two boxes that meet at one corner.
          // In this case, the outer Shells would be combined (in a Union).
          SmProtoVertex *pPV = NULL;
          SmZoneTol3d sF1ZoneTol = SmTol::GetZoneTol3d( pF1 );
          SmZoneTol3d sF2ZoneTol = SmTol::GetZoneTol3d( pF2 );
          for(kk=0;kk<sIntPts.GetSize();kk++)
            {
              SmPointClassification sPC1(sF1ZoneTol) ;
              SmPointClassification sPC2(sF2ZoneTol) ;

              // classify intersectionPoint[kk] against Brep1 and Brep2 faces
              eStat1 = pF1->Point3DClassify( sIntPts[kk], sF1ZoneTol, TRUE, sPC1 ) ; // TRUE: do boundary intersections
              eStat2 = pF2->Point3DClassify( sIntPts[kk], sF2ZoneTol, TRUE, sPC2 ) ; // TRUE: do boundary intersections

              // check state - skip cases that don't classify
              if(eStat1 != SM_SUCCESS) { continue ; }
              if(eStat2 != SM_SUCCESS) { continue ; }

              // check state - sjip sPCs that don't classify to Regions, Faces, Edges, or Vertices in both Breps
              if(sm_CanImprint(sPC1.GetPointClass()) == FALSE ) { continue ; }
              if(sm_CanImprint(sPC2.GetPointClass()) == FALSE ) { continue ; }

              m_pProtoTopoMgr->FindOrCreateProtoVertex(sPC1,  // in : Brep1 PointClassification for face pair XSectPt3d
                                                       sPC2,  // in : Brep2 PointClassification for face pair XSectPt3d
                                                       pPV) ; // out: SmProtoVertex for face pair XSectPt3d
                                                              // out: TRUE = Created output NewPV
                                                              //      FALSE= Found   output NewPV
#ifdef SM_DEBUG_CODE
              if ( bDebugMe2 ) 
                {
                  smos_WriteBuffer(_T("\n**************************************\\"));
                  smos_sprintf(sDebugBuff, _T("\nXSectPt[kk=%3d] - Post FindOrCreateProtoVertex, Post XSect[ii=%3d][jj=%3d]"), kk, ii, jj) ; smos_WriteBuffer(sDebugBuff) ;
                  smos_WriteBuffer(_T("\n\\**************************************"));
                  m_pProtoTopoMgr->Dump( iDumpLevel ); // iDumpLevel: -1 = no output
                                                       //              0 = headers, ProtoVert, Edge, and Face Counts
                                                       //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                                       //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                                       //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
                  smgfx_Erase();
                  /* TopoMgr              */ smgfx_SetLook(1,2, 0,0,0);    m_pProtoTopoMgr->DrawDebug(); sm_GraphicsLoop();       /* ProtoTopoMgr              */
                  /* CurveClassification1 */ smgfx_SetLook(2,3, 0,0,1);    sPC1.Draw() ; sPC1.Dump() ; sm_GraphicsLoop() ; /* Brep1 Point Classifications */
                  /* CurveClassification2 */ smgfx_SetLook(2,3, 0,1,1);    sPC2.Draw() ; sPC2.Dump() ; sm_GraphicsLoop() ; /* Brep2 Point Classifications */
                  /* Face1                */ smgfx_SetLook(2,3, .2,.2,.6); pF1->Draw(SM_DM_CROSSHATCH,8,8); sm_GraphicsLoop();
                  /* Face2                */ smgfx_SetLook(2,3, .6,.2,.2); pF2->Draw(SM_DM_CROSSHATCH,8,8); sm_GraphicsLoop();
                  /* Brep1                */ smgfx_SetLook(1,2, 0,0,1);    m_pBrep ->Draw(TRUE) ; sm_GraphicsLoop() ;
                  /* Brep2                */ smgfx_SetLook(1,2, 0,1,0);    m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE
            } // end iter every XSectPt3d solution

          // If the Faces are coincident, and there is some overlap - create a ProtoFace.
          if ( bAreCoincident && sCoincPEs.GetSize() > 0 )
            {
              SmProtoFace * pNewPF = NULL;
              m_pProtoTopoMgr->CreateProtoFace(pF1, pF2, sCoincPEs, pNewPF) ;

              // Relate New ProtoFace to every pNewPF->CoincPE
              for(kk=0;kk<sCoincPEs.GetSize();kk++)
                {
                  SmProtoEdge * pProtoEdge = sCoincPEs[kk] ;
                  SM_ASSERT_MSG( (pProtoEdge != NULL) && (pNewPF != NULL), _T("SmTopologyIntersector::IntersectTopology - code building ProtoFaces for coincident face pair generated a NULL PF or PE ptr.  This is a bug")) ; 
                  pProtoEdge->RelateCoincidentPF(pNewPF) ;
                } // end iter kk, every pNewPF->CoincPEs[kk] adding pNewFace to the m_sCoincPFs->m_sCoincPF lists

#ifdef SM_DEBUG_CODE
              if ( bDebugMe ) 
                {
                  smos_WriteBuffer(_T("\n**************************************\\"));
                  smos_sprintf(sDebugBuff, _T("\nPost CreateProtoFace & RelateCoincidentPF, Post XSect[ii=%3d][jj=%3d]"), ii, jj) ; smos_WriteBuffer(sDebugBuff) ;
                  smos_WriteBuffer(_T("\n\\**************************************"));
                  m_pProtoTopoMgr->Dump( iDumpLevel ); // iDumpLevel: -1 = no output
                                                       //              0 = headers, ProtoVert, Edge, and Face Counts
                                                       //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                                       //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                                       //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
                }
#endif // SM_DEBUG_CODE

            } // end make a new ProtoFace check
        } // end iter jj, all m_pOther faces of m_pBrep/m_pOther face pairs

#ifdef SM_DEBUG_CODE
      if(bDebugMe3) 
        {
          smos_WriteBuffer(_T("\n**************************************\\"));
          smos_sprintf(sDebugBuff, _T("\nPost XSect and CreateProtoTopo for Face[ii=%3d] with all jj faces"), ii) ; smos_WriteBuffer(sDebugBuff) ;
          smos_WriteBuffer(_T("\n\\**************************************"));
          this->Dump( iDumpLevel );
          m_pProtoTopoMgr->Dump( iDumpLevel ); // iDumpLevel: -1 = no output
                                               //              0 = headers, ProtoVert, Edge, and Face Counts
                                               //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                               //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                               //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
          smgfx_Erase();
          /* TopoMgr */ smgfx_SetLook( 1,2, 0,0,0 ); m_pProtoTopoMgr->DrawDebug(); sm_GraphicsLoop();
          /* Brep1   */ smgfx_SetLook( 1,2, 0,0,1 ); m_pBrep ->Draw(TRUE) ; sm_GraphicsLoop();
          /* Brep2   */ smgfx_SetLook( 1,2, 0,1,0 ); m_pOther->Draw(TRUE) ; sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

    } // end iter ii, all m_pBrep faces of m_pBrep/m_pOther face pairs

  //cbi maybe subroutine:
  // We have another chore to do.
  // Sometimes faces will have intersections that cross each other
  // (e.g. torus/torus case in my_test_merge()).
  // So we have to intersect our ProtoEdges with each other, splitting
  // them with ProtoVertices.

#ifdef SM_DEBUG_CODE
  if(bDebugMe) // after building ProtoTopoMgr before finding ProtoEdge/ProtoEdge and ProtoEdge/ProtoVertex intersections
               // Check that the ProtoTopoMgr PEs and PVs are properly constructed (no coincident edges - no edge holes)
    {
      SmTArray<SmFace*> sDbgFaces1, sDbgFaces2 ; 
      SmTArray<SmEdge*> sDbgEdges1, sDbgEdges2 ; 
      m_pBrep ->GetFaces(sDbgFaces1) ;
      m_pOther->GetFaces(sDbgFaces2) ;
      m_pBrep ->GetEdges(sDbgEdges1) ;
      m_pOther->GetEdges(sDbgEdges2) ;


      smos_WriteBuffer(_T("\n**************************************\\"));
      smos_sprintf(sDebugBuff, _T("%s"), _T("\nPost XSect and CreateProtoTopo for all Face/Face XSects")) ; smos_WriteBuffer(sDebugBuff) ;
      smos_WriteBuffer(_T("\n\\**************************************"));
      this->Dump( iDumpLevel ) ; // iDumpLevel: -1 = no output
                                 //              0 = headers, ProtoVert, Edge, and Face Counts
                                 //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                 //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                 //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
      m_pProtoTopoMgr->Dump( iDumpLevel ); // iDumpLevel: -1 = no output
                                           //              0 = headers, ProtoVert, Edge, and Face Counts
                                           //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                           //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                           //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
      smgfx_Erase();
      /* TopoMgr     */ smgfx_SetLook( 1,2, 0,0,0 ); m_pProtoTopoMgr->DrawDebug(); sm_GraphicsLoop(); // debug suggestion: step into DrawDebug and watch the PEs draw one at a time
      /* edges Face1 */ smgfx_SetLook( 2,3, FALSE ); for(di=0;di<sDbgEdges1.GetSize();di++) 
                                                       { smgfx_SetLook(2+di,3+di,TRUE) ; if(sDbgEdges1[di]) sDbgEdges1[di]->DrawParams() ; sm_GraphicsLoop() ; }
      /* edges Face2 */ smgfx_SetLook( 2,3, FALSE ); for(di=0;di<sDbgEdges2.GetSize();di++) 
                                                       { smgfx_SetLook(2+di,3+di,TRUE) ; if(sDbgEdges2[di]) sDbgEdges2[di]->DrawParams() ; sm_GraphicsLoop() ; }
      /* Brep1       */ smgfx_SetLook( 1,2, 0,0,1 ); m_pBrep ->Draw(TRUE) ; sm_GraphicsLoop();
      /* Brep2       */ smgfx_SetLook( 1,2, 0,1,0 ); m_pOther->Draw(TRUE) ; sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // locals
  SmTArray< SmProtoEdge *>   & rPEs = m_pProtoTopoMgr->GetProtoEdges();
  SmTArray< SmProtoVertex *> & rPVs = m_pProtoTopoMgr->GetProtoVertices();

  // next: 1. split ProtoEdges at any ProtoEdge/ProtoEdge XSects
  //       2. split ProtoEdges at any ProtoEdge/ProtoVertex XSects

  // 1. for every ProtoEdge pair - split ProtoEdges at ProtoEdge/ProtoEdge XSects
  for(ii=0;ii<rPEs.GetSize();ii++)
    {
      SmProtoEdge *pPE1 = rPEs[ii];
      for(jj=ii+1;jj<rPEs.GetSize();jj++)
        {
          SmProtoEdge *pPE2 = rPEs[jj];

          // Split intersecting ProtoEdges at the ProtoEdge/ProtoEdge XSect points
          m_pProtoTopoMgr->SplitIntersectingProtoEdges( pPE1, pPE2 );
          rPEs = m_pProtoTopoMgr->GetProtoEdges();

#ifdef SM_DEBUG_CODE
          if ( bDebugMe3 ) 
            {
              SmTArray< SmEdgeDefinition* > rEdgeDefs1 = pPE1->GetEdgeDefinitions();
              SmTArray< SmEdgeDefinition* > rEdgeDefs2 = pPE2->GetEdgeDefinitions();
              SmCurve                      *pCrv1      = rEdgeDefs1[0]->GetXSectCurve3d();
              SmCurve                      *pCrv2      = rEdgeDefs2[0]->GetXSectCurve3d();
              SmExtent1d                    sDom1      = pPE1->GetXSectInterval();
              SmExtent1d                    sDom2      = pPE2->GetXSectInterval();
              
              smos_WriteBuffer(_T("\n**************************************\\"));
              smos_sprintf(sDebugBuff, _T("\nPost ProtoEdge[ii=%3d] ProtoEdge[%jj==3d] SplitIntersectingProtoEdges"), ii, jj) ; smos_WriteBuffer(sDebugBuff) ;
              smos_WriteBuffer(_T("\n\\**************************************"));
              m_pProtoTopoMgr->Dump( iDumpLevel ); // iDumpLevel: -1 = no output
                                                   //              0 = headers, ProtoVert, Edge, and Face Counts
                                                   //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                                   //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                                   //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
              if ( jj == ii+1 ) 
                {
                  smgfx_Erase();
                  smgfx_SetLook( 1,2, 0,0,0 ); m_pProtoTopoMgr->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook( 5,6, 1,0,0 ); pCrv1->Draw( &sDom1 ) ; sm_GraphicsLoop();
                }
              smgfx_SetLook( 5,6, 0,1,0 ); pCrv2->Draw( &sDom2 ) ; sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

      } // end iter jj of all ProtoEdge pairs
  } //end iter ii of all ProtoEdge pairs -  spliting ProtoEdges at ProtoEdge/ProtoEdge XSects

  // 2. for every ProtoEdge/ProtoVertex pair - split ProtoEdges at ProtoEdge/ProtoVertex XSects
  for(ii=rPVs.GetSize();ii>0;ii--)  // count backward because pPVs get removed from the rPVs array
    {
      SmProtoVertex *pPV = rPVs[ii-1] ;

      for(jj=0;jj<rPEs.GetSize();jj++)
        {
          SmProtoEdge *pPE = rPEs[jj] ;

          // when pPV is coincident with pPE - remove pPV - its a redundant critical point coincident with another intersection curve.

          // split protoEdges at ProtoVertex/ProtoEdge intersections
          SmBoolean bRemoved = m_pProtoTopoMgr->RemoveCoincidentNoEdgeProtoVertex(pPE, pPV) ;

          // When pPV was coincident and removed - quit looking for more coincidences
          if(bRemoved)
            { break ; }

          // obsolete for 1st idea of splitting PEs and changing the rPEs array: 
          // rPEs = m_pProtoTopoMgr->GetProtoEdges();

#ifdef SM_DEBUG_CODE
          if ( bDebugMe3 ) 
            {
              SmTArray< SmEdgeDefinition* > rEdgeDefs = pPE->GetEdgeDefinitions() ;
              SmCurve                      *pCrv      = rEdgeDefs[0]->GetXSectCurve3d() ;
              SmExtent1d                    sDom      = pPE->GetXSectInterval() ;
              
              smos_WriteBuffer(_T("\n\\**************************************")) ;
              smos_sprintf(sDebugBuff, _T("\nPost ProtoVertex[ii=%3d] ProtoEdge[jj==%3d] RemoveCoincidentNoEdgeProtoVertex"), ii-1, jj) ; smos_WriteBuffer(sDebugBuff) ;
              smos_WriteBuffer(_T("\n**************************************\\")) ;
              
              m_pProtoTopoMgr->Dump( iDumpLevel ); // iDumpLevel: -1 = no output
                                                   //              0 = headers, ProtoVert, Edge, and Face Counts
                                                   //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                                   //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                                   //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
              smgfx_Erase();
              smgfx_SetLook( 1,2, 0,0,0 ); m_pProtoTopoMgr->Draw(); sm_GraphicsLoop();
              smgfx_SetLook( 5,6, 1,0,0 ); if(pCrv) pCrv->Draw( &sDom ) ; sm_GraphicsLoop();
              smgfx_SetLook( 7,8, 0,1,0 ); if(pPV) pPV->Draw() ; sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

        } // end iter jj of all ProtoEdge/ProtoVertex pairs
    } //end iter ii of all ProtoEdge/ProtoVertex pairs - removing coincident edge-less ProtoVerts

#ifdef SM_DEBUG_CODE
  if(bDebugMe) 
    {
      smos_WriteBuffer(_T("\n**************************************\\"));
      smos_sprintf(sDebugBuff, _T("%s"), _T("\n All done - Exiting IntersectTopology")) ; smos_WriteBuffer(sDebugBuff) ;
      smos_WriteBuffer(_T("\n\\**************************************"));
      
      this->Dump( iDumpLevel );
      m_pProtoTopoMgr->Dump( iDumpLevel ); // iDumpLevel: -1 = no output
                                           //              0 = headers, ProtoVert, Edge, and Face Counts
                                           //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                           //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                           //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
      
      smgfx_Erase();
      smgfx_SetLook( 1,2, 0,0,0 ); m_pProtoTopoMgr->DrawDebug(); sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 0,0,1 ); m_pBrep ->Draw(TRUE) ; sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 0,1,0 ); m_pOther->Draw(TRUE) ; sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmTopologyIntersector::IntersectTopology

/*******************************************************************//**
PURPOSE: Resolve all of our ProtoVertices and ProtoEdges.

USAGE NOTES---- Resolve all PVs first, then PEs.
***********************************************************************/
SmStatus SmTopologyIntersector::ResolveIntersections()
{
  // locals
  ULONG ii ;
  SmProtoVertex              * pPV     = NULL ;
  SmTArray< SmProtoVertex* > & rPVs    = m_pProtoTopoMgr->GetProtoVertices() ;
  ULONG                        lNumPVs = rPVs.GetSize();

  // Resolve every ProtoVertex
  for(ii=0;ii<lNumPVs;ii++ )
    {
        pPV = rPVs[ii];

        // resolve ProtoVertex
        if(pPV->Resolve() != SM_SUCCESS)
          { WARN( _T("Warning: problem in ResolveProtoTopology()" )) ; } // breakpoint

      // cbi note: PV->Resolve() can remove all VertexDefs from the PV.
      //           In that case, we could remove the PV.  But why bother?
      //           If it becomes necessary, do that here.
    } // end iter every ProtoVertex calling Resolve

  // locals 
  SmProtoEdge              * pPE     = NULL ;
  SmTArray< SmProtoEdge* > & rPEs    = m_pProtoTopoMgr->GetProtoEdges();
  ULONG                      lNumPEs = rPEs.GetSize();
  
  // Resolve every ProtoEdge
  for(ii=0;ii<lNumPEs;ii++)
    {
      pPE = rPEs[ii];

      // resolve ProtoEdge
      if(pPE->Resolve() != SM_SUCCESS)
        { WARN( _T("Warning: problem in ResolveProtoTopology()" )); } // breakpoint
    }  // end iter every ProtoEdge calling Resolve

  // Resolve OverlappingPEs
  SmStatus sStatus = m_pProtoTopoMgr->ResolveOverlappingProtoEdges() ;
  SE_MSG(sStatus, _T( "Warning: problem in ResolveProtoTopology()" ) );
  
  // all done
  return SM_SUCCESS;

} // end SmTopologyIntersector::ResolveIntersections

/*******************************************************************//**
PURPOSE: Imprint our intersections into our two Breps.

USAGE NOTES---- 
***********************************************************************/
SmStatus SmTopologyIntersector::ImprintIntersections()
{
#ifdef SM_DEBUG_CODE
int iDebugLevel = -1 ; // m_pProtoTopoMgr->Dump(iDumpLevel) iDumpLevel: -1 = no output
                       //                                                0 = headers, ProtoVert, Edge, and Face Counts
                       //                                                5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                       //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                       //                                               15 = Brep1, Brep2, and SmTopologyIntersector dumps
SmBoolean bDebugMe=FALSE;
  if ( bDebugMe ) 
    {
      SM_DUMP_AND_ASSERT_VALID( m_pBrep  );
      SM_DUMP_AND_ASSERT_VALID( m_pOther );
      m_pProtoTopoMgr->Dump(iDebugLevel) ;  // iDumpLevel: -1 = no output
                                            //              0 = headers, ProtoVert, Edge, and Face Counts
                                            //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                            //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                            //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
      smgfx_Erase();
      smgfx_SetLook( 1,4, 0,0,1 ); m_pBrep ->Draw(TRUE);    sm_GraphicsLoop();
      smgfx_SetLook( 1,4, 0,1,0 ); m_pOther->Draw(TRUE);    sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 1,0,1 ); m_pProtoTopoMgr->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
   }
#endif // SM_DEBUG_CODE

  // Don't SER on each imprint: if one fails, keep pushing on.
  SmStatus eStat, eRetStat = SM_SUCCESS;

  SmTemporaryChangeValue <SmBoolean> sTempChange1( m_pBrep ->m_bEditingEnabled, TRUE );
  SmTemporaryChangeValue <SmBoolean> sTempChange2( m_pOther->m_bEditingEnabled, TRUE );
  const SmContext *cpContext = m_pBrep->GetContext();
  SmTemporaryChangeValue <SmBoolean> sTempChangeDB( ((SmContext*)cpContext)->GetDoingBooleanRef(),   TRUE );

  SmTArray< SmProtoVertex* > rPVs = m_pProtoTopoMgr->GetProtoVertices();

  ULONG ii, jj, lNumObjs = rPVs.GetSize();
  for(ii=0;ii<lNumObjs;ii++)
    {
      SmProtoVertex *pPV = rPVs[ii];
      SmTArray< SmVertexDefinition* > & rVDs = pPV->GetVertexDefinitions();

      // Q: if a PV has more than one VD, do we connect them with a new Edge?
      // A: No: if they're meant to be connected, then there will be a ProtoEdge
      //    indicating that.  Sometimes they're meant to be separate: Resolve() takes care of that.
      //    Could be coincident, on coincident Edges, or just close together.
      for(jj=0;jj<rVDs.GetSize();jj++)
        {
          SmVertexDefinition *pVD = rVDs[jj];
          eStat = pVD->Imprint();

          if ( eStat != SM_SUCCESS && eRetStat == SM_SUCCESS ) // Record the first problem.
            { eRetStat = eStat; }
        }
    }

  SmTArray< SmProtoEdge* > rPEs = m_pProtoTopoMgr->GetProtoEdges();
  lNumObjs = rPEs.GetSize();
  for(ii=0;ii<lNumObjs;ii++)
    {
      SmProtoEdge *pPE = rPEs[ii];
      SmTArray< SmEdgeDefinition* > & rEDs = pPE->GetEdgeDefinitions();
      for(jj=0;jj<rEDs.GetSize();jj++)
        {
          SmEdgeDefinition *pED = rEDs[jj];
          eStat = pED->Imprint();

          if(eStat != SM_SUCCESS)
            {
              if(eRetStat == SM_SUCCESS) // Record the first problem.
                { eRetStat = eStat; }

              // Delete its curves, which would have been consumed if Imprint() had worked.
              pPE->RemoveEdgeDefinition( pED );
              pED->Destruct();
              delete pED; pED = NULL;
              jj--;
            }
        }
    }

  // At this point, there might be coincident Vertices and Edges in the Breps.
  // The rest of ManifoldBoolean() expects coincident Vertices to be glued
  // (although not coincident Edges).  So do that.  [Offset demo 2]

  eStat = m_pProtoTopoMgr->GlueCoincidentBrepVertices();

  if(eStat != SM_SUCCESS && eRetStat == SM_SUCCESS) // Record the first problem.
    { eRetStat = eStat; }

#ifdef SM_DEBUG_CODE
  if ( bDebugMe ) 
    {
      SM_DUMP_AND_ASSERT_VALID( m_pBrep  );
      SM_DUMP_AND_ASSERT_VALID( m_pOther );
      m_pProtoTopoMgr->Dump(iDebugLevel) ;  // iDumpLevel: -1 = no output
                                            //              0 = headers, ProtoVert, Edge, and Face Counts
                                            //              5 = List of ProtoVerts, Edges, Faces. Repeate ProtoVert, Edge, and Face counts
                                            //             10 = PE->PVs, List of PE overlappingPE and CoinPF indices, and PE->EdgeDef dumps
                                            //             15 = Brep1, Brep2, and SmTopologyIntersector dumps
      smgfx_Erase();
      smgfx_SetLook( 1,4, 0,0,1 ); m_pBrep ->Draw( TRUE ) ; sm_GraphicsLoop();
      smgfx_SetLook( 1,4, 0,1,0 ); m_pOther->Draw( TRUE ) ; sm_GraphicsLoop();
      smgfx_SetLook( 1,2, 1,0,1 ); m_pProtoTopoMgr->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return eRetStat;

} // end SmTopologyIntersector::ImprintIntersections

/*******************************************************************//**
PURPOSE: Set up Relationships between the two Breps.

USAGE NOTES---- 
***********************************************************************/
SmStatus SmTopologyIntersector::SetRelationships()
{
  SmTArray< SmVertexDefinition* > sVDs;
  m_pProtoTopoMgr->GetAllVertexDefinitions( sVDs );
  ULONG ii, lNumObjs = sVDs.GetSize();
  SmTopology *pTopo1, *pTopo2;

  for ( ii=0; ii<lNumObjs; ii++ )
  {
      SmVertexDefinition *pVD = sVDs[ii];

      pTopo1 = pVD->GetFinalVertex( 1 );
      pTopo2 = pVD->GetFinalVertex( 2 );

      if ( pTopo1 == NULL || pTopo2 == NULL )
      {
          WARN( _T("Warning: SmTopologyIntersector::SetRelationships(): Brep Topology not set.\n") );
          continue;
      }

//cbi chk pre-existing?
//cbi SmTopology *pMate1 = SM_REINTERPRET_CAST( SmTopology*, m_p1To2->GetValueAt( pTopo1 ));
//cbi SmTopology *pMate2 = SM_REINTERPRET_CAST( SmTopology*, m_p2To1->GetValueAt( pTopo2 ));

      this->Relate( pTopo1, pTopo2 );
  }

  SmTArray< SmEdgeDefinition* > sEDs;
  m_pProtoTopoMgr->GetAllEdgeDefinitions( sEDs );
  lNumObjs = sEDs.GetSize();
  for ( ii=0; ii<lNumObjs; ii++ )
  {
      SmEdgeDefinition *pED = sEDs[ii];
      if ( pED == NULL )
        { continue; }

      pTopo1 = pED->GetFinalEdge( 1 );
      pTopo2 = pED->GetFinalEdge( 2 );

      if ( pTopo1 == NULL || pTopo2 == NULL )
      {
          WARN( _T("Warning: SmTopologyIntersector::SetRelationships(): Brep Topology not set.\n") );
          continue;
      }

      this->Relate( pTopo1, pTopo2 );
  }

  // Note: the rest of the Boolean algorithm does not want coincident Faces to be related. [Reg_060124_B...]
static constexpr SmBoolean sbRelateFaces = FALSE;

  if ( sbRelateFaces )
  {
      SmTArray< SmProtoFace* > & rPFs = m_pProtoTopoMgr->GetProtoFaces();
      lNumObjs = rPFs.GetSize();
      for ( ii=0; ii<lNumObjs; ii++ )
      {
          SmProtoFace *pPF = rPFs[ii];

          pTopo1 = pPF->GetSrcTopo( 1 );
          pTopo2 = pPF->GetSrcTopo( 2 );

          if ( pTopo1 == NULL || pTopo2 == NULL )
          {
              WARN( _T("Warning: SmTopologyIntersector::SetRelationships(): Null Brep Topology.\n") );
              continue;
          }

          this->Relate( pTopo1, pTopo2 );

      } // end for each ProtoFace
  }

  return SM_SUCCESS;

} // end SmTopologyIntersector::SetRelationships

/*******************************************************************//**
PURPOSE: Intersect Insert and Relate a Vertex and a Brep

NOTES: pBrep is either the m_pBrep or m_pOther Brep and
                pVertex comes from the remaining Brep Pointer.
***********************************************************************/
SmStatus SmTopologyIntersector::IIRVertexBrep  // eff: intersect vertex with Brep, merge solutions into Brep
  (SmVertex * pVertex,                         // in : Target Vertex
   SmBrep   * pBrep,                           // in : Target Brep to modify
   SmBoolean  bSwapOrder)                      // in : TRUE = pVertex from m_pOther and pBrep is m_pBrep
                                               //      FALSE= pVertex from m_pBrep and pBrep is m_pOther
{
    SmSolution sSData[64];
    SmSolutionArray sSolutions(64,sSData);
    SmZoneTol3d sVertexZoneTol3d = SmTol::GetZoneTol3d(pVertex) ;

    // no work - working with a OtherOBject subset and pVertex is not on the list
    if (m_pSubset && bSwapOrder)
      {
        // quit if Vertex is not in Subset
        if (m_pSubset->At(pVertex) == NULL) return SM_SUCCESS;
      }

    // intersect pVertex with Brep
    double dBestAnswer=0.0;
      {
        SmTemporaryChangeValue<SmBoolean> sChange(pBrep->m_bEditingEnabled,FALSE);
        SER(SmTopologySolver::BrepPointSolve(pBrep,                        // in : Brep to query
                                             pVertex->GetPoint(),          // in : Query point
                                             SM_SO_INTERSECT,              // oneof: SM_SO_INTERSECT, SM_SO_NORMALIZE, SM_SO_RAYFIRE,
                                                                           //        SM_SO_MINIMIZE, SM_SO_PROJECTED_MINIMIZE, SM_SO_DIRECTED_MINIMIZE,
                                                                           //        SM_SO_MAXIMIZE, SM_SO_PROJECTED_MAXIMIZE, SM_SO_DIRECTED_MAXIMIZE,
                                                                           //        SM_SO_SIGNED_DIRECTED_MINIMIZE, SM_SO_3D_SIGNED_DIRECTED_MINIMIZE.
                                             SM_SR_ALL,                    // oneof: SM_SR_SINGLE = Only produce one solution - the best
                                                                           //        SM_SR_ALL    = Find all function satisfying solutions to given tol
                                             sVertexZoneTol3d,             // in : passed along to SmTopologySolver, max valid sol size                 
                                             dBestAnswer,                  // in : passed along to SmTopologySolver,                     
                                             NULL,                         // in : used for some eSolverOperations - see SmSolverOperationType for more
                                             sSolutions));                 // out: One SmSolution object per problem solution
      }

    // no work - no intersections
    if (sSolutions.GetSize() == 0) return SM_SUCCESS;

    // RMB Nov 4, 2004
    // The first solution may not be the correct one to use since
    // pVertex may intersect a vertex, an edge and a face
    // We want to use the intersection with a vertex, an edge or a face
    // in that order.

    ULONG   iSol  = 0 ;               // this is the solution to use
    SM_TYPE iType = SmTopology_TYPE ; // this is its type

    // Find the lowest dimension intersection solution in Subset if Subset is being used
    ULONG ii;
    for (ii=0; ii<sSolutions.GetSize(); ii++)
      {
        SmSolution & rSol = sSolutions[ii];
        SmObject *pTestObject = (SmObject*)rSol.m_apObjects[0];
        switch (pTestObject->GetType())
          {
            case SmVertex_TYPE:
              if (   (   (bSwapOrder == TRUE)                                     //       pTestObject from Brep
                      || (   m_pSubset == NULL                                    //    or no OtherBrep SubSet
                          || m_pSubset->At(rSol.m_apObjects[0]) != NULL)) //       pTetOBject is in the OtherBrep Subset
                  && (   iType == SmEdge_TYPE                                     // and new TYPE is better than Old Type
                      || iType == SmFace_TYPE
                      || iType == SmTopology_TYPE)) { iSol  = ii ;
                                                      iType = SmVertex_TYPE ;
                                                    }
              break ;
            case SmEdge_TYPE  :
              { SmEdge *pE = SM_CAST_PTR(SmEdge,rSol.m_apObjects[0]) ;
                SM_ASSERT(pE != NULL) ;                                           //       pTestObject from Brep
                if (   (   (bSwapOrder == TRUE)                                   //    or no OtherBrep SubSet
                        || (   m_pSubset == NULL                                  //       pTetOBject is in the OtherBrep Subset
                            || m_pSubset->At(pE->GetCurve()) != NULL))    // and new TYPE is better than Old Type
                   && (   iType == SmFace_TYPE
                       || iType == SmTopology_TYPE)) { iSol  = ii ;
                                                       iType = SmEdge_TYPE ;
                                                     }
              }
              break ;
            case SmFace_TYPE  :
              { SmFace *pF = SM_CAST_PTR(SmFace,rSol.m_apObjects[0]) ;
                SM_ASSERT(pF != NULL) ;
                if (   (   (bSwapOrder == TRUE)                                   //       pTestObject from Brep
                        || (   m_pSubset == NULL                                  //    or no OtherBrep SubSet
                            || m_pSubset->At(pF->GetSurface()) != NULL))  //       pTetOBject is in the OtherBrep Subset
                   && (   iType == SmTopology_TYPE)) { iSol  = ii ;               // and new TYPE is better than Old Type
                                                       iType = SmFace_TYPE ;
                                                     }
              }
              break ;
          } // end switch on solution entity type
      } // end iter every solution

    // no work - no suitable solutions
    if(iType == SmTopology_TYPE) return SM_SUCCESS ;

//    ULONG lSize = sSolutions.GetSize();
//    if (lSize > 1)
//      {
//        // we need to choose the 'lowest' dimension result
//        ULONG iV_Type, iE_Type, iF_Type;
//        iV_Type = iE_Type = iF_Type = lSize;
//
//        // for every solution - find lowest dimension result
//        for (ULONG ii=0 ; ii<lSize ; ii++)
//          {
//            SmSolution &rSol = sSolutions[ii];
//            ULONG iType = rSol.m_apObjects[0]->GetType();
//            if (iType == SmVertex_TYPE)
//              {
//                iV_Type = ii;
//                break;
//              }
//            else if (iType == SmEdge_TYPE)
//              {
//                if (iE_Type == lSize) iE_Type = ii;  // use the first one found
//              }
//            else if (iType == SmFace_TYPE)
//              {
//                if (iF_Type == lSize) iF_Type = ii; // use the first one found
//              }
//          } // end iter every solution
//
//        // iSol defaults to 0 if there are none of these,
//        // that should not happen.
//        SM_ASSERT(   iV_Type < lSize
//                  || iE_Type < lSize
//                  || iF_Type < lSize) ;
//
//        if      (iV_Type < lSize) iSol = iV_Type;
//        else if (iE_Type < lSize) iSol = iE_Type;
//        else if (iF_Type < lSize) iSol = iF_Type;
//        else { SE(SM_ERR) ; }
//
//      } // end more than 1 solution check

    // let rSol == selected solution, sPC == Solution's Point Classification
    SmSolution & rSol = sSolutions[iSol];
    SmPointClassification sPC(sVertexZoneTol3d, &GetContext()) ;  // this tol is unlikely to be correct - check it
    SER(sPC.LoadFromIntersection(sVertexZoneTol3d, rSol.m_apObjects[0],rSol.m_vStart,0, SM_BIG_DOUBLE));

//    // no work - when working only on a Subset and intersect objects not in the subset
//    if (m_pSubset)
//      {
//        // quit if Vertex is not in Subset
//        if (m_pSubset->At(pVertex) == NULL) return SM_SUCCESS;
//
//        // quit if intersected Object's geometry is not in the Subset
//        SmFace *pF = SM_CAST_PTR(SmFace,rSol.m_apObjects[0]);
//        if (pF){ if (m_pSubset->At(pF->GetSurface()) == NULL) return SM_SUCCESS;
//               }
//        else   { SmEdge *pE = SM_CAST_PTR(SmEdge,rSol.m_apObjects[0]);
//                 if (pE) { if (m_pSubset->At(pE->GetCurve()) == NULL) { return SM_SUCCESS; }
//                         }
//                 else if (m_pSubset->At(rSol.m_apObjects[0]) == NULL) { return SM_SUCCESS; }
//               }
//      } // end subset check

    // merge the point classification into its m_pObject's topology structure
    //  SmPointClassificationType == SM_PC_VERTEX, update the vertex tolerance
    //                            == SM_PC_EDGE,   call SmBrep::MakeVertexSplitEdge
    //                            == SM_PC_FACE,   call SmBrep::MakeVertexLoop
    //                            == SM_PC_REGION, call SmBrep::MakeVertexShell
    // set PointClassification->Object = MergedVertex
    SER(sPC.MergeIntoObject(pVertex->GetPoint()));

    // map pVertex to the Brep mated Vertex
    if (bSwapOrder)  { SER(Relate((SmTopology*)sPC.GetObject(),pVertex)); }
    else             { SER(Relate(pVertex,(SmTopology*)sPC.GetObject())); }

    // all done
    return SM_SUCCESS;

} // end SmTopologyIntersector::IIRVertexBrep

/*******************************************************************//**
PURPOSE:   Set a CurveClassification to be as if the curve is the
              same as the edge.

NOTES:
  When rCurveClass->Interval == pEdge->Interval
    rCurveClass->m_vStart->m_ePointClass = SM_PC_VERTEX
    rCurveClass->m_vMid->m_ePointClass   = SM_PC_EDGE
    rCurveClass->m_vEnd->m_ePointClass   = SM_PC_VERTEX

  When CurveClassification interval is trimmed then appropriate
    CurveClassification EndPointClassifications are changed from
    VERTEX to EDGE as
          m_vStart->m_ePointClass = SM_PC_EDGE
          m_vEnd->m_ePointClass   = SM_PC_EDGE
***********************************************************************/

static SmStatus sm_SetEdgeClass
  (SmCurveClassification & rCurveClass,  // in : target classification
   SmEdge * pEdge)                       // in : target edge
{
  // edge locals
  SmVertex  *pV1     = pEdge->GetVertex();
  SmVertex  *pV2     = pEdge->GetOtherVertex(pV1);
  SmEdgeuse *pPrimEU = pEdge->GetPrimaryEdgeuse();

  if (pPrimEU->GetOrientation() == SM_OT_OPPOSITE)
    {
      SmVertex *pVTmp = pV1;
      pV1 = pV2;
      pV2 = pVTmp;
    }

  // CurveClassification locals
  SmCurveInterval & rCIvl = rCurveClass[0];

  // set 1st CurveClassification interval to classify to edge and its end vertices
  rCIvl.m_vStart.SetClassObject (SM_PC_VERTEX, pV1);
  rCIvl.m_vStart.SetPreSnapParam(rCurveClass.GetInterval().GetMin());
#ifdef SM_USE_NEWTOL      
  SM_NEWTOL_LINE rCIvl.m_vStart.SetGap3d    (0.0);
#else // SM_USE_OLDTOL 
  SM_OLDTOL_LINE rCIvl.m_vStart.SetDeviation   (0.0);
#endif // SM_USE_OLDTOL

  rCIvl.m_vMid.SetClassObject   (SM_PC_EDGE, pEdge);
  rCIvl.m_vMid.SetPreSnapParam  (SM_BIG_DOUBLE);
#ifdef SM_USE_NEWTOL      
  SM_NEWTOL_LINE rCIvl.m_vMid.SetGap3d     (0.0);
#else // SM_USE_OLDTOL 
  SM_OLDTOL_LINE rCIvl.m_vMid.SetDeviation     (0.0);
#endif // SM_USE_OLDTOL

  rCIvl.m_vEnd.SetClassObject   (SM_PC_VERTEX, pV2);
  rCIvl.m_vEnd.SetPreSnapParam  (rCurveClass.GetInterval().GetMax());
#ifdef SM_USE_NEWTOL      
  SM_NEWTOL_LINE rCIvl.m_vEnd.SetGap3d     (0.0);
#else // SM_USE_OLDTOL 
  SM_OLDTOL_LINE rCIvl.m_vEnd.SetDeviation     (0.0);
#endif // SM_USE_OLDTOL

  // get curve endPoints
  SmPoint3d sStart, sEnd;
  SER(rCurveClass.GetCurve()->EvaluatePoint(rCurveClass.GetInterval().GetMin(),sStart));
  SER(rCurveClass.GetCurve()->EvaluatePoint(rCurveClass.GetInterval().GetMax(),sEnd));

  // Note that the curve on the curve class may be trimmed - take
  // care of that here by replacing the vertex by edge classifications
  // for the ends of the interval.

  // define a tolerance
#ifdef SM_USE_NEWTOL      
  SM_NEWTOL_LINE double dStartTol = pV1->GetTolerance() + rCurveClass.GetMaxGap3d();
#else // SM_USE_OLDTOL 
  SM_OLDTOL_LINE double dStartTol = pV1->GetTolerance() + SmTol::GetSrcZoneTol3d(&rCurveClass);
  SM_OLDTOL_LINE // double dStartTol = pV1->GetTolerance() + rCurveClass.GetZoneTol3d();
#endif // SM_USE_OLDTOL
  SmBoolean bSuccess;
  double dParameter, dDistToCurve;
  SmCurve *pEdgeCurve = pEdge->GetCurve();

  // when curveInterval start is trimmed
  if (sStart.DistanceBetween(pV1->GetPoint()) > dStartTol)
    {
      // change CurveClassification m_vStart from Vertex to Point on Edge classification
      SER(pEdgeCurve->DropPoint(pEdge->GetInterval(), // in : target curve allowed domain
                                sStart,               // in : Point to drop to curve
                                NULL,                 // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                      //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                      //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                dStartTol,            // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                      //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                      //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                      //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                NULL,                 // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                bSuccess,             // out: TRUE = found a drop point
                                dParameter,           // out: found drop curve param
                                dDistToCurve)) ;      // out: found drop distance
                                                      // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                      //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                      //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                      //      default:[SM_SO_MINIMIZE] to preserve original behavior
      rCIvl.m_vStart.SetClassObject (SM_PC_EDGE, pEdge);
      rCIvl.m_vStart.SetPreSnapParam(dParameter);
#ifdef SM_USE_NEWTOL      
      SM_NEWTOL_LINE rCIvl.m_vStart.SetGap3d    (dDistToCurve);
#else // SM_USE_OLDTOL 
      SM_OLDTOL_LINE rCIvl.m_vStart.SetDeviation   (dDistToCurve);
#endif // SM_USE_OLDTOL
      rCIvl.m_vStart.SetTParam      (dParameter);

    } // end is curveInterval start trimmed check

  // when curveInterval end is trimmed
#ifdef SM_USE_NEWTOL      
  SM_NEWTOL_LINE double dEndTol = pV2->GetTolerance() + rCurveClass.GetMaxGap3d();
#else // SM_USE_OLDTOL 
  SM_OLDTOL_LINE double dEndTol = pV2->GetTolerance() + SmTol::GetSrcZoneTol3d(&rCurveClass);
  SM_OLDTOL_LINE // double dEndTol = pV2->GetTolerance() + rCurveClass.GetZoneTol3d();
#endif // SM_USE_OLDTOL
  if (sEnd.DistanceBetween(pV2->GetPoint()) > dEndTol)
    {
      // change CurveClassification m_vEnd from Vertex to Point on Edge classification
      SER(pEdgeCurve->DropPoint(pEdge->GetInterval(),  // in : target curve allowed domain
                                sEnd,                  // in : Point to drop to curve
                                NULL,                  // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                       //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                       //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                dEndTol,               // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                       //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                       //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                       //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                NULL,                  // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                bSuccess,              // out: TRUE = found a drop point
                                dParameter,            // out: found drop curve param
                                dDistToCurve)) ;       // out: found drop distance
                                                       // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                       //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                       //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                       //      default:[SM_SO_MINIMIZE] to preserve original behavior
      rCIvl.m_vEnd.SetClassObject (SM_PC_EDGE, pEdge);
      rCIvl.m_vEnd.SetPreSnapParam(dParameter);
#ifdef SM_USE_NEWTOL      
      SM_NEWTOL_LINE rCIvl.m_vEnd.SetGap3d    (dDistToCurve);
#else // SM_USE_OLDTOL 
      SM_OLDTOL_LINE rCIvl.m_vEnd.SetDeviation   (dDistToCurve);
#endif // SM_USE_OLDTOL
      rCIvl.m_vEnd.SetTParam      (dParameter);

    } // end is curveInterval end trimmed check

  return SM_SUCCESS;

} // end sm_SetEdgeClass

/*******************************************************************//**
PURPOSE: Intersect Insert and Relate an OtherBrep->Edge into a target Brep.

NOTES:
***********************************************************************/
SmStatus SmTopologyIntersector::IIREdgeBrep
  (SmEdge *pEdge,                           // in : target edge from OtherBrep
   SmBrep *pBrep,                           // in : target brep
   SmBoolean bSwapOrder,                    // in : TRUE  = call MergeCurveClasses(pEdgeClassification, pBrepClassification)
                                            //      FALSE = call MergeCurveClasses(pBrepClassification, pEdgeClassification)
   SmBoolean bForceFaceIntersections,       // in : TRUE  = always intersect pEdge with pBrep->Surfaces
                                            //      FALSE = don't when pEdge is a wire
                                            // note: All calls to IIREdgeBrep are currently being made
                                            //       with bForceFaceIntersections == TRUE.
   SmTArray< SmEdge* >   *paDeletedEdges,   // out: ptrs to deleted edges
   SmTArray< SmVertex* > *paDeletedVertices // out: ptrs to deleted vertices
  )
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
  // draw Brep(blue), Curve(Cyan)
  if (bDebugMe)
    {
      pEdge->Dump() ;
      pEdge->GetCurve()->Dump() ;
      pBrep->Dump() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1); pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 1,1,0); pEdge->Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // construct 1st CurveClassification to map curve to pEdge
  SmCurveClassification sCrvClass1(pEdge->GetCurve(),
                                   pEdge->GetInterval(),
                                   NULL,
                                   pEdge->GetTolerance());

  // init CurveClassification to classify to pEdge
  // 1st Interval = [m_vStart = SM_PC_VERTEX, m_vMid = SM_PC_EDGE, m_vEnd = SM_PC_VERTEX]
  SER(sm_SetEdgeClass(sCrvClass1,pEdge));

  // select SolverType - handle wires differently unless bForceFaceIntersections == TRUE
  SmSolverOperationType eSolverOp =   (   bForceFaceIntersections
                                       || !pEdge->IsWire())
                                    ? SM_SO_INTERSECT
                                    : SM_SO_INTERSECT_WIREFRAME ;


  // construct 2nd CurveClassification object to map curve to Brep
  SmCurveClassification sCrvClass2(pEdge->GetCurve(),
                                   pEdge->GetInterval(),
                                   NULL,
                                   pEdge->GetTolerance());

   // used to do two iterations when InsertTopologyIntersections()
   //  modified the shape of the curve - but InsertTopologyIntersections()
   //  no longer does that - so just do one iteration.
   ULONG ii ;
   // gwc:removed SmBoolean bModifiedCurve = FALSE ;
   for(ii=0;ii<1;ii++)  // gwc: used to be  for(ii=0;ii<2;ii++)
     {
       // gwc:removed   // skip 2nd iteration when curve shape was unmodified
       // gwc:removed   if( ii > 0 && bModifiedCurve == FALSE)
       // gwc:removed     { continue ; }
       // gwc:removed   else
         { sCrvClass2.ReSet() ; }

       // BrepCurveSolve locals
       SmSolution sSData[64];
       SmSolutionArray sSolutions(64,sSData);
       {
         // temporarily disable EditingEnabled bit
         SmTemporaryChangeValue<SmBoolean> sChange(pBrep->m_bEditingEnabled,FALSE);

         // Intersect Edge->Curve with Brep
         SER(SmTopologySolver::BrepCurveSolve(pBrep,                  // in : target Brep
                                              *pEdge->GetCurve(),     // in : target Curve
                                              pEdge->GetInterval(),   // in : target Curve interval
                                              eSolverOp,              // in : Oneof the listed values above
                                              SM_SR_ALL,              // in : Oneof: SM_SR_SINGLE, SM_SR_ALL, SM_SR_NODES
                                              pEdge->GetTolerance(),  // in : size distance for solutions
                                              SM_BIG_DOUBLE,          // in : Cull value for min/max operations
                                              NULL,                   // in :
                                              sSolutions));           // out: Solutions list. This list is reset
       }                                                              //      before being loaded in this call.

       // add curve/Brep intersections to CrvClass2
       SER(sCrvClass2.InsertTopologyIntersections(sSolutions, 0));

#ifdef SM_DEBUG_CODE
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
       if (bDebugMe || lCount == lDebugCount)
         {
           sSolutions.Dump() ;
           sCrvClass1.Dump();
           sCrvClass2.Dump();

           smgfx_Erase() ;
           smgfx_SetLook(1,2, 0,0,1); pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
           smgfx_SetLook(3,4, 1,1,0); pEdge->Draw() ; sm_GraphicsLoop() ;
           smgfx_SetLook(5,6, 1,0,0); sSolutions.Draw() ; sm_GraphicsLoop() ;
         //  smgfx_SetLook(5,6, 1,0,0); sCrvClass1->Draw() ; sm_GraphicsLoop() ;
         //  smgfx_SetLook(5,6, 1,0,0); sCrvClass2->Draw() ; sm_GraphicsLoop() ;
           sm_GraphicsLoop();
         }
#endif // SM_DEBUG_CODE
     } // end iter max of two iterations (now only 1 iteration)

  // merge curveClassifications into Breps
  SmBoolean bDeletedTopology = FALSE ;
  if ( !bSwapOrder )
    {
      SER( MergeCurveClasses( sCrvClass1,            // i/o: 1st Target Curve Classification
                              sCrvClass2,            // i/o: 2nd Target Curve Classification
                              TRUE,                  // in : TRUE = Merge all point classifications
                              bDeletedTopology,      // out: TRUE = small edges were deleted by squeeze
                              paDeletedEdges,        // out: ptrs to deleted edges
                              paDeletedVertices ));  // out: ptrs to deleted vertices
    }
  else
    {
      SER( MergeCurveClasses( sCrvClass2,            // i/o: 1st Target Curve Classification
                              sCrvClass1,            // i/o: 2nd Target Curve Classification
                              TRUE,                  // in : TRUE = Merge all point classifications
                              bDeletedTopology,      // out: TRUE = small edges were deleted by squeeze
                              paDeletedEdges,        // out: ptrs to deleted edges
                              paDeletedVertices ));  // out: ptrs to deleted vertices
    }

  // set output values and return
  if (bDeletedTopology)
    { m_bTopologyDeleted = TRUE;
    }

  return SM_SUCCESS;

} // end SmTopologyIntersector::IIREdgeBrep

/*******************************************************************//**
PURPOSE: Merge two curve classifications into the corresponding
   breps.  This modifies the topology graph of both affected Brep objects
   by adding edges representing the classified curve segments within the
   rCrvClass1 and rCrvClass2 arguments.

   Previously unconnected Graphs may become connected as the
   new edges are added.

NOTES:
   Create new SmEdge objects for each curve segment within the rCrvClass1
   and rCrvClass2 input arguments that classified as being on
   an existing Brep Face.  Then insert those SmEdge objects
   into the respective Brep connectivity graphs.  Curve segments that lie
   outside of existing faces are thrown away.

   Curve segment boundaries that have been classified to (intersect with)
   an existing Brep Edge cause that edge to be split in two with
   a new SmVertex object.

   Care is taken for curve segments that are coincident to existing
   Brep edges and for segment boundaries that are coincident to existing
   Brep vertices to reuse existing topology objects rather than creating
   new ones where they are not needed.

  Note this method will do all of the homogenization,
  syncronization and clean up of the classifications prior to merge.
***********************************************************************/
SmStatus SmTopologyIntersector::MergeCurveClasses
 (SmCurveClassification & rCrvClass1,           // i/o: 1st Target Curve Classification
  SmCurveClassification & rCrvClass2,           // i/o: 2nd Target Curve Classification
  SmBoolean               bMergeSingularities,  // in : TRUE = Merge all point classifications
  SmBoolean             & rbTopologyWasDeleted, // out: TRUE = small edges were deleted by squeeze
  SmTArray< SmEdge* >   * paDeletedEdges,       // out: ptrs to deleted edges
  SmTArray< SmVertex* > *paDeletedVertices)     // out: ptrs to deleted vertices
{
  rbTopologyWasDeleted = FALSE;

#ifdef SM_DEBUG_CODE
  // dump input curve classifications
SmBoolean bDebugMe = FALSE;
static ULONG lCount = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if (bDebugMe || lDebugCount == lCount)
    {
      ULONG di ;
      Dump() ;
      SM_DUMP_AND_ASSERT_VALID(&rCrvClass1);   
      SM_DUMP_AND_ASSERT_VALID(&rCrvClass2);

      SmBrep        * pBrep1  = rCrvClass1.GetBrep() ;  SM_ASSERT_VALID(pBrep1) ;
      SmBrep        * pBrep2  = rCrvClass2.GetBrep() ;  SM_ASSERT_VALID(pBrep2) ;
      const SmCurve * pCurve1 = rCrvClass1.GetCurve() ;
      const SmCurve * pCurve2 = rCrvClass2.GetCurve() ;

      smgfx_Erase() ;
      smgfx_SetLook(3,5, 1,0,0) ; rCrvClass1.GetCurve()->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,1) ; Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
      if(FALSE)
        { // the DrawDetails() call is full of expensive gap calcs - only call when you want to see the extra information
          smgfx_SetLook(2,3, 0,1,1) ; DrawDetails(3.0, 4.0) ; sm_GraphicsLoop() ;
        }
      smgfx_SetCurveClassificationObjectLook(5,7,
                                             0, 0,1,
                                             0,.5,1,
                                             .5,0,1) ; rCrvClass1.DrawObjects(); sm_GraphicsLoop();
      smgfx_SetCurveClassificationObjectLook(7,9,
                                             0, 1,0,
                                             0,1,.5,
                                             .5,1,0) ; rCrvClass2.DrawObjects(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; if(pCurve1) pCurve1->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5) ; for(di=0;di<rCrvClass1.GetSize();di++)
        { SmFace   * pStartFace   = SM_CAST_PTR(SmFace,   rCrvClass1[di].m_vStart.GetObject()) ;
          SmEdge   * pStartEdge   = SM_CAST_PTR(SmEdge,   rCrvClass1[di].m_vStart.GetObject()) ;
          SmVertex * pStartVertex = SM_CAST_PTR(SmVertex, rCrvClass1[di].m_vStart.GetObject()) ;
          SmFace   * pMidFace     = SM_CAST_PTR(SmFace,   rCrvClass1[di].m_vMid.GetObject()) ;
          SmEdge   * pMidEdge     = SM_CAST_PTR(SmEdge,   rCrvClass1[di].m_vMid.GetObject()) ;
          SmFace   * pEndFace     = SM_CAST_PTR(SmFace,   rCrvClass1[di].m_vEnd.GetObject()) ;
          SmEdge   * pEndEdge     = SM_CAST_PTR(SmEdge,   rCrvClass1[di].m_vEnd.GetObject()) ;
          SmVertex * pEndVertex   = SM_CAST_PTR(SmVertex, rCrvClass1[di].m_vEnd.GetObject()) ;
          smgfx_ChangeColor( di != 0 ); if(pStartFace) pStartFace->DrawUV(); if(pStartEdge) { SmExtent1d sExt = pStartEdge->GetInterval();  pStartEdge->GetCurve()->DrawParams( &sExt ); } if(pStartVertex) pStartVertex->Draw(); sm_GraphicsLoop();
          smgfx_ChangeColor();      if(pMidFace) pMidFace->DrawUV(); if(pMidEdge) { SmExtent1d sExt = pMidEdge->GetInterval();  pMidEdge->GetCurve()->DrawParams( &sExt); } sm_GraphicsLoop();
          smgfx_ChangeColor();      if(pEndFace) pEndFace->DrawUV(); if(pEndEdge) { SmExtent1d sExt = pEndEdge->GetInterval();  pEndEdge->GetCurve()->DrawParams( &sExt ); } if(pEndVertex) pEndVertex->Draw(); sm_GraphicsLoop();
        }
      smgfx_SetLook(3,4, 0,1,1) ; if(pCurve2) pCurve2->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7) ; for(di=0;di<rCrvClass2.GetSize();di++)
        { SmFace   * pStartFace   = SM_CAST_PTR(SmFace,   rCrvClass2[di].m_vStart.GetObject()) ;
          SmEdge   * pStartEdge   = SM_CAST_PTR(SmEdge,   rCrvClass2[di].m_vStart.GetObject()) ;
          SmVertex * pStartVertex = SM_CAST_PTR(SmVertex, rCrvClass2[di].m_vStart.GetObject()) ;
          SmFace   * pMidFace     = SM_CAST_PTR(SmFace,   rCrvClass2[di].m_vMid.GetObject()) ;
          SmEdge   * pMidEdge     = SM_CAST_PTR(SmEdge,   rCrvClass2[di].m_vMid.GetObject()) ;
          SmFace   * pEndFace     = SM_CAST_PTR(SmFace,   rCrvClass2[di].m_vEnd.GetObject()) ;
          SmEdge   * pEndEdge     = SM_CAST_PTR(SmEdge,   rCrvClass2[di].m_vEnd.GetObject()) ;
          SmVertex * pEndVertex   = SM_CAST_PTR(SmVertex, rCrvClass2[di].m_vEnd.GetObject()) ;
          smgfx_ChangeColor( di != 0 ); if(pStartFace) pStartFace->DrawUV(); if(pStartEdge) { SmExtent1d sExt = pStartEdge->GetInterval(); pStartEdge->GetCurve()->DrawParams( &sExt ); } if(pStartVertex) pStartVertex->Draw(); sm_GraphicsLoop();
          smgfx_ChangeColor();      if(pMidFace) pMidFace->DrawUV(); if(pMidEdge) { SmExtent1d sExt = pMidEdge->GetInterval(); pMidEdge->GetCurve()->DrawParams( &sExt ); } sm_GraphicsLoop();
          smgfx_ChangeColor();      if(pEndFace) pEndFace->DrawUV(); if(pEndEdge) { SmExtent1d sExt = pEndEdge->GetInterval(); pEndEdge->GetCurve()->DrawParams( &sExt ); } if(pEndVertex) pEndVertex->Draw(); sm_GraphicsLoop();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // For endPointClassifications
  //   1. if endPoint close to edge's vertex change classification from EDGE to VERTEX
  //   2. if endPoint separates a short interval from a long - remove the endPoint, merge the intervals
  if (rCrvClass1.FixupClassification() != SM_SUCCESS)
    { return SM_ERR; }
  if (rCrvClass2.FixupClassification() != SM_SUCCESS)
    { return SM_ERR; }

  // Syncronize the two curve classifications
  if (rCrvClass1.Homogenize(rCrvClass2) != SM_SUCCESS)
    { return SM_ERR; }

  // Remove all intervals that pass the AreCoincidentMaybe() test -
  //   Short intervals whose mid and end-pt both classify to an edge.
  if (rCrvClass1.CleanupAndValidate(rCrvClass2) != SM_SUCCESS)
    { return SM_ERR; }

  // For every classification PointPair
  //   When one Point classifies to a mated object,
  //   Verify the other Point classifies to the mate.
  //   1. for edge classified point pairs with errors
  //      set Classifies = SM_PC_UNKNOWN (this was the previous behavior
  //                                      and should be restudied.)
  if (FixupClassMates(rCrvClass1,rCrvClass2) != SM_SUCCESS)
    { return SM_ERR; }

#ifdef SM_DEBUG_CODE
  // dump homogenized curve classifications
  if (bDebugMe)
    {
      rCrvClass1.Dump();
      rCrvClass2.Dump();

      ULONG di ;
      SmBrep  * pBrep1  = rCrvClass1.GetBrep() ;  SM_ASSERT_VALID(pBrep1) ;
      SmBrep  * pBrep2  = rCrvClass2.GetBrep() ;  SM_ASSERT_VALID(pBrep2) ;
      const SmCurve * pCurve1 = rCrvClass1.GetCurve() ;
      const SmCurve * pCurve2 = rCrvClass2.GetCurve() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; if(pCurve1) pCurve1->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5) ; for(di=0;di<rCrvClass1.GetSize();di++)
        { SmFace   * pStartFace   = SM_CAST_PTR(SmFace,   rCrvClass1[di].m_vStart.GetObject()) ;
          SmEdge   * pStartEdge   = SM_CAST_PTR(SmEdge,   rCrvClass1[di].m_vStart.GetObject()) ;
          SmVertex * pStartVertex = SM_CAST_PTR(SmVertex, rCrvClass1[di].m_vStart.GetObject()) ;
          SmFace   * pMidFace     = SM_CAST_PTR(SmFace,   rCrvClass1[di].m_vMid.GetObject()) ;
          SmEdge   * pMidEdge     = SM_CAST_PTR(SmEdge,   rCrvClass1[di].m_vMid.GetObject()) ;
          SmFace   * pEndFace     = SM_CAST_PTR(SmFace,   rCrvClass1[di].m_vEnd.GetObject()) ;
          SmEdge   * pEndEdge     = SM_CAST_PTR(SmEdge,   rCrvClass1[di].m_vEnd.GetObject()) ;
          SmVertex * pEndVertex   = SM_CAST_PTR(SmVertex, rCrvClass1[di].m_vEnd.GetObject()) ;
          smgfx_ChangeColor( di != 0 ); if(pStartFace) pStartFace->DrawUV(); if(pStartEdge) { SmExtent1d sExt = pStartEdge->GetInterval();  pStartEdge->GetCurve()->DrawParams( &sExt ); } if(pStartVertex) pStartVertex->Draw(); sm_GraphicsLoop();
          smgfx_ChangeColor();      if(pMidFace) pMidFace->DrawUV(); if(pMidEdge) { SmExtent1d sExt = pMidEdge->GetInterval();  pMidEdge->GetCurve()->DrawParams( &sExt ); } sm_GraphicsLoop();
          smgfx_ChangeColor();      if(pEndFace) pEndFace->DrawUV(); if(pEndEdge) { SmExtent1d sExt = pEndEdge->GetInterval();  pEndEdge->GetCurve()->DrawParams( &sExt ); } if(pEndVertex) pEndVertex->Draw(); sm_GraphicsLoop();
        }
      smgfx_SetLook(3,4, 0,1,1) ; if(pCurve2) pCurve2->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7) ; for(di=0;di<rCrvClass2.GetSize();di++)
        { SmFace   * pStartFace   = SM_CAST_PTR(SmFace,   rCrvClass2[di].m_vStart.GetObject()) ;
          SmEdge   * pStartEdge   = SM_CAST_PTR(SmEdge,   rCrvClass2[di].m_vStart.GetObject()) ;
          SmVertex * pStartVertex = SM_CAST_PTR(SmVertex, rCrvClass2[di].m_vStart.GetObject()) ;
          SmFace   * pMidFace     = SM_CAST_PTR(SmFace,   rCrvClass2[di].m_vMid.GetObject()) ;
          SmEdge   * pMidEdge     = SM_CAST_PTR(SmEdge,   rCrvClass2[di].m_vMid.GetObject()) ;
          SmFace   * pEndFace     = SM_CAST_PTR(SmFace,   rCrvClass2[di].m_vEnd.GetObject()) ;
          SmEdge   * pEndEdge     = SM_CAST_PTR(SmEdge,   rCrvClass2[di].m_vEnd.GetObject()) ;
          SmVertex * pEndVertex   = SM_CAST_PTR(SmVertex, rCrvClass2[di].m_vEnd.GetObject()) ;
          smgfx_ChangeColor( di != 0 ); if(pStartFace) pStartFace->DrawUV(); if(pStartEdge) { SmExtent1d sExt = pStartEdge->GetInterval(); pStartEdge->GetCurve()->DrawParams( &sExt ); } if(pStartVertex) pStartVertex->Draw(); sm_GraphicsLoop();
          smgfx_ChangeColor();      if(pMidFace) pMidFace->DrawUV(); if(pMidEdge) { SmExtent1d sExt = pMidEdge->GetInterval();  pMidEdge->GetCurve()->DrawParams( &sExt ); } sm_GraphicsLoop();
          smgfx_ChangeColor();      if(pEndFace) pEndFace->DrawUV(); if(pEndEdge) { SmExtent1d sExt = pEndEdge->GetInterval();  pEndEdge->GetCurve()->DrawParams( &sExt ); } if(pEndVertex) pEndVertex->Draw(); sm_GraphicsLoop();
        }
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // special case crvClass1 has 1 interval and m_vMid:[EDGE] and either m_vStart or m_vEnd:[EDGE]. (Curve coin to EDGE)
  // skip curves much smaller than EDGE. (splitting off a DegenEdge from an existing Edge's end for coincidence is not needed)
  if (rCrvClass1.GetSize() == 1)
    {
      SmCurveInterval & rIvl1 = rCrvClass1[0];
      SmCurveInterval & rIvl2 = rCrvClass2[0];
      SmPointClassification & rMid1 = rIvl1.m_vMid;
      SmPointClassification & rMid2 = rIvl2.m_vMid;

      // when crvClass1 interval mid and at least one endPoint map to Edge (Curve coin to EDGE)
      if (   rMid1.GetPointClass() == SM_PC_EDGE
          && (   rIvl1.m_vStart.GetPointClass() == SM_PC_EDGE
              || rIvl1.m_vEnd.GetPointClass()   == SM_PC_EDGE) )
        {
          // when either crvClass2 interval endPoints are not vertices (pBrep->Edge is going to get split)
          if (   rIvl2.m_vStart.GetPointClass() != SM_PC_VERTEX
              || rIvl2.m_vEnd.GetPointClass()   != SM_PC_VERTEX)
            {
              // get curve length
              const SmCurve *pCurve = rCrvClass1.GetCurve();
              double         dLeng  = pCurve->ApproximateLength(rIvl1.m_vInterval,5);

              // get the target edge and its length
              SmEdge *pE        = (SmEdge*)rMid1.GetObject();
              double dLengEdge1 = pE->GetCurve()->ApproximateLength(pE->GetInterval(),5);

              // skip curves which are smaller than the edge - this should be a tolerance check
              // GWC_NEEDS_WORK Change_following_check_from_Percent_to_degen_length_check GWC_LINE ;
              if (dLeng < dLengEdge1 / 20.0)
                { return SM_SUCCESS;
                }

              // when interval2 classifies to an edge
              if (rMid2.GetPointClass() == SM_PC_EDGE)
                {
                  // get edge2 length
                  SmEdge *pE2        = (SmEdge*)rMid2.GetObject();
                  double  dLengEdge2 = pE2->GetCurve()->ApproximateLength(pE2->GetInterval(),5);

                  // skip curves smaller than the edge length
                  if (dLeng < dLengEdge2 / 20.0)
                    { return SM_SUCCESS;
                    }
                } // end interval2 midPoint classifies to an edge check
            } // end interval2 endPoints don't classify to vertices check
        } // end interval1 mid and endPoint classify to an edge check
    } // end single interval classification special case check

  // special case crvClass2 has 1 interval and m_vMid:[EDGE] and either m_vStart or m_vEnd:[EDGE]. (Curve coin to EDGE)
  // skip curves much smaller than EDGE. (splitting off a DegenEdge from an existing Edge's end for coincidence is not needed)
  if (rCrvClass2.GetSize() == 1)
    {
      SmCurveInterval & rIvl1 = rCrvClass1[0];
      SmCurveInterval & rIvl2 = rCrvClass2[0];
      SmPointClassification & rMid1 = rIvl1.m_vMid;
      SmPointClassification & rMid2 = rIvl2.m_vMid;

      // when crvClass2 interval mid and at least one endPoint map to Edge (Curve coin to EDGE)
      if (   rMid2.GetPointClass() == SM_PC_EDGE
          && (   rIvl2.m_vStart.GetPointClass() == SM_PC_EDGE
              || rIvl2.m_vEnd.GetPointClass() == SM_PC_EDGE) )
        {
          // when either crvClass2 interval endPoints are not vertices
          if (!(   rIvl1.m_vStart.GetPointClass() == SM_PC_VERTEX
                && rIvl1.m_vEnd.GetPointClass()   == SM_PC_VERTEX))
            {
              const SmCurve *pCurve = rCrvClass2.GetCurve();
              double dLeng = pCurve->ApproximateLength(rIvl2.m_vInterval,5);
              SmEdge *pE = (SmEdge*)rMid2.GetObject();
              double dLengEdge1 = pE->GetCurve()->ApproximateLength(pE->GetInterval(),5);
              if (dLeng < dLengEdge1 / 20.0)
                { return SM_SUCCESS;
                }

              if (rMid1.GetPointClass() == SM_PC_EDGE)
                {
                  SmEdge *pE2        = (SmEdge*)rMid1.GetObject();
                  double  dLengEdge2 = pE2->GetCurve()->ApproximateLength(pE2->GetInterval(),5);
                  if (dLeng < dLengEdge2 / 20.0)
                    { return SM_SUCCESS;
                    }
                } // end interval1 midPoint classifies to an edge check
            } // end interval1 endPoints don't classify to vertices check
        } // end interval2 mid and endPoint classify to an edge check
    } // end single interval classification special case check

  // Now merge classification data into each Brep
  SmEdge            * sEData[16];
  SmEdge            * sEDataOther[16];
  SmTArray<SmEdge*>   sNewEdges(16,sEData);
  SmTArray<SmEdge*>   sNewEdgesOther(16,sEDataOther);

#ifdef SM_DEBUG_CODE
  if (bDebugMe || lDebugCount == lCount) 
    {
      SM_DUMP_AND_ASSERT_VALID(&rCrvClass1) ;
      SM_DUMP_AND_ASSERT_VALID(&rCrvClass2) ;
    }
#endif // SM_DEBUG_CODE

  // Merge results of two classifications into the SmBrep topology graphs.
  //    Only intervals classified to topology in both SmBreps will be inserted.
  SER(rCrvClass1.MergeClassifications(rCrvClass2,            // in : other target classification
                                      bMergeSingularities,   // in : TRUE = merge each boundary PointClassifications
                                      NULL,                  // out: optional list of thisBrep  new faces, NULL to ignore.
                                                             //      When an existing face is split by an interval into 2
                                                             //      the oldFace is reused and 1 newFace is constructed and
                                                             //      only the newFace is placed on this list.
                                      NULL,                  // out: optional list of otherBrep new faces, NULL to ignore.
                                                             //      When an existing face is split by an interval into 2
                                                             //      the oldFace is reused and 1 newFace is constructed and
                                                             //      only the newFace is placed on this list.
                                      &sNewEdges,            // out: optional list of thisBrep  new edges, NULL to ignore.
                                                             //      when an existing edge is split by a vertex it is reused
                                                             //      and 1 new edge is constructed - BOTH edges are put on this list.
                                      &sNewEdgesOther)) ;    // out: optional list of otherBrep new edges, NULL to ignore.
                                                             //      when an existing edge is split by a vertex it is reused
                                                             //      and 1 new edge is constructed - BOTH edges are put on this list.
                                                             // out: optional list of thisBrep  new vertices, NULL to ignore.
                                                             // out: optional list of otherBrep new vertices, NULL to ignore.

#ifdef SM_DEBUG_CODE
  // dump curveClassifications and draw intersection curve with breps
  if (bDebugMe)
    {
      Dump() ;
      rCrvClass1.Dump();
      rCrvClass2.Dump();
      SM_ASSERT_VALID(m_pBrep) ;
      SM_ASSERT_VALID(m_pOther) ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,1) ; this->Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
      if(FALSE)
        { // the DrawDetails() call is full of expensive gap calcs - only call when you want to see the extra information
          smgfx_SetLook(2,3, 0,1,1) ; this->DrawDetails(3.0, 4.0) ; sm_GraphicsLoop() ;
        }

      smgfx_SetLook(3,4, 1,0,0) ;
      if(rCrvClass1.GetCurve()->IsKindOf(SmBSplineCurve_TYPE))
        { ((SmBSplineCurve *)rCrvClass1.GetCurve())->DrawWithKnots() ; sm_GraphicsLoop() ;
        }
      else
        { rCrvClass1.GetCurve()->Draw() ; sm_GraphicsLoop() ;
        }
      rCrvClass1.GetCurve()->DrawCurvature(-35,75) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop();

      ULONG ii;
      smgfx_SetLook( 5,7, 0,0,1 );
      for ( ii=0; ii<sNewEdges.GetSize(); ii++ )
        { sNewEdges[ii]->Draw(); sm_GraphicsLoop(); }
      sm_GraphicsLoop();

      smgfx_SetLook( 5,7, 0,1,0 );
      for ( ii=0; ii<sNewEdgesOther.GetSize(); ii++ )
        { sNewEdgesOther[ii]->Draw(); sm_GraphicsLoop(); }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Rebuild stale mate relationships for new edges
  //   (because a split edge is reused as one of the new children edges
  //    it may have an old mapping that is no longer valid)

  // First remove any existing relationships from all new edges
  // (which includes original pre-split edges).
  ULONG ii, jj;
  ULONG lNumNewBrep  = sNewEdges.GetSize();
  ULONG lNumNewOther = sNewEdgesOther.GetSize();
  for( ii=0; ii < lNumNewBrep + lNumNewOther; ii++ )
    {
      // look for a ThisEdge<->OtherEdge mapping for current target edge
      SmEdge *pNewE      = NULL;
      SmEdge *pOtherNewE = NULL;
      if ( ii < lNumNewBrep )
        {
          pNewE      = sNewEdges[ii];
          pOtherNewE = (SmEdge*)GetOtherMate( pNewE );
        }
      else
        {
          pOtherNewE = sNewEdgesOther[ ii-lNumNewBrep ];
          pNewE      = (SmEdge*)GetBrepMate( pOtherNewE );
        }

      // when old pNewE<->pOtherNewE mapping exists remove it
      if ( pNewE && pOtherNewE )
        {
          // remove the old mapping
          RemoveRelationship( pNewE, pOtherNewE );
        }
    } // end iter every NewEdge and NewEdgeOther removing stale relationships

  // Now update Relationships on all new edges.
  for( ii=0; ii < lNumNewBrep + lNumNewOther; ii++ )
    {
      // look for a ThisNewEdge<->OtherNewEdge mappings
      SmEdge *pNewE      = NULL;
      SmEdge *pOtherNewE = NULL;
      if ( ii < lNumNewBrep )
        {
          pNewE      = sNewEdges[ii];
          pOtherNewE = NULL ;
        }
      else
        {
          pOtherNewE = sNewEdgesOther[ ii-lNumNewBrep ];
          pNewE      = (SmEdge*)GetBrepMate(pOtherNewE);
        }

      // skip NewE/OtherNewE referenced in the classifications - they're handled later
      SmBoolean bRef1 = rCrvClass1.IsReferenced(pNewE) ;
      SmBoolean bRef2 = rCrvClass2.IsReferenced(pOtherNewE) ;
      if(bRef1 || bRef2)
        { continue ; }

      // skip OtherNewE when it already has a mapping
      if( pNewE && pOtherNewE )
        { continue; }

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          rCrvClass1.Dump() ;
          rCrvClass2.Dump() ;
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1) ; m_pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,0,0) ; m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
          smgfx_SetLook(2,3, 0,1,1) ; Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
          if(FALSE)
            { // the DrawDetails() call is full of expensive gap calcs - only call when you want to see the extra information
              smgfx_SetLook(2,3, 0,1,1) ; DrawDetails(3.0, 4.0) ; sm_GraphicsLoop() ;
            }
          smgfx_SetLook(3,4, 1,0,0) ; if(pNewE) pNewE->Draw(); sm_GraphicsLoop();
          smgfx_SetLook(4,5, 0,1,0) ; if(pOtherNewE) pOtherNewE->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // restore mappings for Edges that were split by adding vertices that once had a mapping
      //   ThisNewEdge  <-> closest OtherNewEdge and
      //   OtherNewEdge <-> closest ThisNewEdge

      // make new mapping between ThisNewEdge and OtherNewEdge nearest in 3D space to ThisNewEdge
      SmEdge * pBestOEdge = NULL;
      if(pNewE)
        {
          // evaluate pNewE->MidPoint
          SmExtent1d  sIvl      = pNewE->GetInterval();
          SmCurve    *pCurve    = pNewE->GetCurve();
          double      dBestDist = SM_BIG_DOUBLE;
          SmPoint3d sMidPnt;
          SER(pCurve->EvaluatePoint(sIvl.Evaluate(0.5),sMidPnt));

          // for all new otherEdges - find the closest to the midPoint
          for (jj=0; jj<lNumNewOther; jj++)
            {
              SmEdge    *pOE     = sNewEdgesOther[jj];

              // skip otheredges referenced in the classification
              if(rCrvClass2.IsReferenced(pOE))
                { continue ; }

              // drop ThisEdge->midPoint to OtherEdge->Curve
              SmCurve   *pOCurve = pOE->GetCurve();
              SmBoolean  bSuccess;
              double     dT, dDist;

              // GWC_NEEDS_WORK USE_OF_APPROX_TOLERANCE_WHERE_ONLY_TOPOLOGY_TOL_MIGHT_BE_USED GWC_LINE ;
              SER(pOCurve->DropPoint
                   (pOE->GetInterval(),                   // in : target curve allowed domain
                    sMidPnt,                              // in : Point to drop to curve
                    NULL,                                 // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                          //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                          //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                    10.0*(  m_dThisApproxTol3d            // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                          + pOE->GetTolerance()),         //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                          //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                          //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                    NULL,                                 // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                    bSuccess,                             // out: TRUE = found a drop point
                    dT,                                   // out: found drop curve param
                    dDist)) ;                             // out: found drop distance
                                                          // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                          //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                          //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                          //      default:[SM_SO_MINIMIZE] to preserve original behavior
              // skip OtherEdges too far away
              if (!bSuccess || dDist > dBestDist) { continue; }

              // drop the otherEdge->MidPoint back onto thisEdge->Curve
              SmPoint3d sMidPnt2;
              SER(pOCurve->EvaluatePoint(pOE->GetInterval().Evaluate(0.5),sMidPnt2));
              SER(pCurve->DropPoint(sIvl,                               // in : target curve allowed domain
                                    sMidPnt2,                           // in : Point to drop to curve
                                    NULL,                               // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                        //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                        //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                    10.0*( m_dThisApproxTol3d           // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                          +pOE->GetTolerance()),        //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                        //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                        //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                    NULL,                               // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                    bSuccess,                           // out: TRUE = found a drop point
                                    dT,                                 // out: found drop curve param
                                    dDist)) ;                           // out: found drop distance
                                                                        // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                        //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                        //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                        //      default:[SM_SO_MINIMIZE] to preserve original behavior

              // when OtherEdge->MidPoint is closer to thisEdge than all others
              if (bSuccess && dDist < dBestDist)
                {
                  // keep it as the BestOEdge
                  pBestOEdge = pOE;
                  dBestDist  = dDist;
                }
            } // end iter all OtherEdges

          // If the closest found edge is not within tolerance, do not relate it.  [B279]
          if ( pBestOEdge!=NULL ) {
            if ( dBestDist > pNewE->GetTolerance() + pBestOEdge->GetTolerance() )
              { pBestOEdge = NULL; }
          }

          // when a mate was found - make thisEdge<->pBestOEdge mapping
          if (pBestOEdge)
            { SER(Relate(pNewE,pBestOEdge));
            }
        } // end make new mapping between ThisEdge and OtherEdge nearest in 3D space to ThisEdge block

      if(pOtherNewE)
        { // make new mapping between OtherEdge and ThisEdge nearest in 3D space to OtherEdge

          // get pOtherEdge->MidPoint
          SmExtent1d   sOIvl     = pOtherNewE->GetInterval();
          SmCurve    * pOCurve   = pOtherNewE->GetCurve();
          SmEdge     * pBestEdge = NULL;
          double       dBestDist = SM_BIG_DOUBLE;
          SmPoint3d sMidPnt;
          SER(pOCurve->EvaluatePoint(sOIvl.Evaluate(0.5),sMidPnt));

          // for every new ThisEdge
          for (jj=0; jj<lNumNewBrep; jj++)
            {
              SmEdge    * pBE    = sNewEdges[jj];

              // skip otheredges referenced in the classification
              if(rCrvClass1.IsReferenced(pBE))
                { continue ; }

              // drop OtherEdge->midPoint to ThisEdge->Curve
              SmCurve   * pCurve = pBE->GetCurve();
              SmBoolean bSuccess = FALSE;
              double    dT, dDist;
              SER(pCurve->DropPoint(pBE->GetInterval(),                  // in : target curve allowed domain
                                    sMidPnt,                             // in : Point to drop to curve
                                    NULL,                                // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                         //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                         //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                    10.0*( m_dThisApproxTol3d            // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                          +pBE->GetTolerance()),         //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                         //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                         //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                    NULL,                                // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                    bSuccess,                            // out: TRUE = found a drop point
                                    dT,                                  // out: found drop curve param
                                    dDist)) ;                            // out: found drop distance
                                                                         // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                         //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                         //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                         //      default:[SM_SO_MINIMIZE] to preserve original behavior

              // skip thisEdges too far away
              if (!bSuccess || dDist > dBestDist) continue;

              // drop ThisEdge->Curve->MidPoint back onto OtherEdge->Curve
              SmPoint3d sMidPnt2;
              SER(pCurve->EvaluatePoint(pBE->GetInterval().Evaluate(0.5),sMidPnt2));
              SER(pOCurve->DropPoint(sOIvl,                              // in : target curve allowed domain
                                     sMidPnt2,                           // in : Point to drop to curve
                                     NULL,                               // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                         //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                         //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                     10.0*( m_dThisApproxTol3d           // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                           +pBE->GetTolerance()),        //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                                         //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                         //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                     NULL,                               // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                     bSuccess,                           // out: TRUE = found a drop point
                                     dT,                                 // out: found drop curve param
                                     dDist)) ;                           // out: found drop distance
                                                                         // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                         //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                         //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                         //      default:[SM_SO_MINIMIZE] to preserve original behavior

              // when ThisEdge->MidPoint is closer to thisEdge than all others
              if (bSuccess && dDist < dBestDist)
                {
                  // keep it as the BestOEdge
                  pBestEdge = pBE;
                  dBestDist = dDist;
                }
          } // end iter all ThisEdges

          // If the closest found edge is not within tolerance, do not relate it.  [B279]
          if ( pBestEdge!=NULL ) {
            if ( dBestDist > pOtherNewE->GetTolerance() + pBestEdge->GetTolerance() )
              { pBestEdge = NULL; }
          }

          // when a OtherEdge <-> ClosestThisEdge mate pair was found
          if (pBestEdge)
            {
              // add the OtherEdge<->ClosestThisEdge mapping
              SER(Relate(pBestEdge,pOtherNewE));
            } // end found BestEdge check
        } // end make new mapping between OtherEdge and ThisEdge nearest in 3D space to OtherEdge block
    } // end iter every newly created edge

  // Now update relationships between CurveClassification topology objects
  SmTArray<SmVertex*> sDeletedV;
  SmTArray<SmVertex*> sSurvivingV;
  SmTArray<SmEdge*> sDeletedEdges;
  SmBoolean bDidLastEnd = FALSE;
  SmStatus eSqueezeStat = SM_SUCCESS;

  // for every classification interval
  for (ii=0; ii<rCrvClass1.GetSize(); ii++)
    {
      SmCurveInterval & rClass1 = rCrvClass1[ii];
      SmCurveInterval & rClass2 = rCrvClass2[ii];

      // See about relating Starts: Vertices.
      if (   rClass1.m_vStart.GetPointClass() == SM_PC_VERTEX
          && rClass2.m_vStart.GetPointClass() == SM_PC_VERTEX)
        {
          // skip interval-start if last interval-end was done
          if (!bDidLastEnd)
            {
              // get vertices from CurveClassification object
              SmVertex *pV1 = (SmVertex*)rClass1.m_vStart.GetObject();
              SmVertex *pV2 = (SmVertex*)rClass2.m_vStart.GetObject();
              ULONG lFoundIndex=0;

              // replace deleted vertices with surviving vertices in CurveClassification
              if (sDeletedV.FindElement(pV1,lFoundIndex))
                {
                  pV1 = sSurvivingV[lFoundIndex];
                  rClass1.m_vStart.SetClassObject(SM_PC_VERTEX, pV1);
                }
              if (sDeletedV.FindElement(pV2,lFoundIndex))
                {
                  pV2 = sSurvivingV[lFoundIndex];
                  rClass2.m_vStart.SetClassObject(SM_PC_VERTEX, pV2);
                }

              // when both this and other Brep are present (the usual case)
              if (m_pBrep && m_pOther)
                {
                  // if vertices have old mates - squeeze old and new mates
                  eSqueezeStat = CheckForSqueezeVertices(pV1,pV2,sDeletedV,sSurvivingV,sDeletedEdges);
                }

              // remember if any edges were deleted by squeeze
              if (sDeletedEdges.GetSize() != 0)
                {
                  rbTopologyWasDeleted = TRUE;
                }

              // create the mate relationship between the two vertices
              if ( eSqueezeStat == SM_SUCCESS )  // [B458 B459]
                { SER( Relate( pV1, pV2 )); }

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              SmVertex *pVOther = (SmVertex*)GetOtherMate(pV1);
              SmVertex *pBV = (SmVertex*)GetBrepMate(pV2);

              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1); m_pBrep->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,0); m_pOther->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,3, 0,1,1); Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
              if(FALSE)
                { // the DrawDetails() call is full of expensive gap calcs - only call when you want to see the extra information
                  smgfx_SetLook(2,3, 0,1,1) ; DrawDetails(3.0, 4.0) ; sm_GraphicsLoop() ;
                }
              smgfx_SetLook(3,4, 1,0,0); pV1->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(4,5, 0,1,0); pV2->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(5,6, 0,1,1); if (pVOther) pVOther->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(6,7, 1,1,0); if (pBV) pBV->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
            } // end don't duplicate work for completed start-points check
        } // end check on relating start-vertices

      // See about relating Mids: Edges.
      if (   rClass1.m_vMid.GetPointClass() == SM_PC_EDGE
          && rClass2.m_vMid.GetPointClass() == SM_PC_EDGE)
        {
          // get Edges from the CurveClassification
          SmEdge *pE1 = (SmEdge*)rClass1.m_vMid.GetObject();
          SmEdge *pE2 = (SmEdge*)rClass2.m_vMid.GetObject();
          ULONG lFoundIndex=0;

          // when neither edge has been deleted
          if (   (!sDeletedEdges.FindElement(pE1,lFoundIndex))
              && (!sDeletedEdges.FindElement(pE2,lFoundIndex)) )
            {
              // create the Edge/Edge mate relationship
              if (Relate(pE1,pE2) != SM_SUCCESS)
                {
#ifdef SM_DEBUG_CODE
                  if (bDebugMe)
                    {
                      smgfx_Erase();
                      smgfx_SetLook(1,2, 0,0,1); m_pBrep->Draw(); sm_GraphicsLoop();
                      smgfx_SetLook(1,2, 0,1,0); m_pOther->Draw(); sm_GraphicsLoop();
                      smgfx_SetLook(2,3, 0,1,1); Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
                      if(FALSE)
                        { // the DrawDetails() call is full of expensive gap calcs - only call when you want to see the extra information
                          smgfx_SetLook(2,3, 0,1,1) ; DrawDetails(3.0, 4.0) ; sm_GraphicsLoop() ;
                        }
                      smgfx_SetLook(3,4, 1,0,0); pE1->Draw(); sm_GraphicsLoop();
                      smgfx_SetLook(4,5, 0,1,0); pE2->Draw(); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                    }
#endif // SM_DEBUG_CODE
                  // when pE2 has a mate but pE1 does not
                  //  - try to fix things by removing unneeded edges
                  if (   GetBrepMate(pE2)  != NULL
                      && GetOtherMate(pE1) == NULL)
                    {
                      SmBrep *pBrep = pE1->GetBrep();
                      SmBoolean bFixedProblem = FALSE;

                      // when pE1 is manifold
                      if (pE1->IsManifold())
                        {
                          SmTArray<SmFace*> sFaces;
                          pE1->GetFaces(sFaces);
                          // and both its faces share the same surface
                          if (   sFaces.GetSize() > 1
                               && sFaces[0]->GetSurface() == sFaces[1]->GetSurface())
                            {
                              // delete the unneeded edge
                              bFixedProblem = TRUE;
                              sDeletedEdges.Add(pE1);
                              SER(pBrep->DeleteEdge(pE1));
                              rbTopologyWasDeleted = TRUE;
                            } // end
                        } // end pE1 IsManifold check

                      // when there is still a problem
                      if (!bFixedProblem)
                        {
                          // when pE2 mate's is manifold
                          SmEdge *pE1Other = (SmEdge*)GetBrepMate(pE2);
                          if (pE1Other && pE1Other->IsManifold())
                            {
                              SmTArray<SmFace*> sFaces;
                              pE1Other->GetFaces(sFaces);
                              // and its faces share the same surface
                              if (   sFaces.GetSize() > 1
                                   && sFaces[0]->GetSurface() == sFaces[1]->GetSurface())
                                {
                                  // remove the unneeded edge
                                  bFixedProblem = TRUE;
                                  RemoveRelationship(pE1Other,pE2);
                                  sDeletedEdges.Add(pE1Other);
                                  SER(pBrep->DeleteEdge(pE1Other));
                                  rbTopologyWasDeleted = TRUE;
                                  SER(Relate(pE1,pE2));
                                }
                            } // end PE1Other Edge IsManifold check
                        } // end bFixedProblem == FALSE check

                      // if there is still a problem - signal an error
                      if (!bFixedProblem)
                        {
                          SER(SM_ERR);
                        }
                    } // end pE2 has a mate but pE1 does not check

                  // when pE1 has a mate but pE2 does not
                  //  - try to fix things by deleting unneeded edges
                  else if (   GetBrepMate(pE2) == NULL
                           && GetOtherMate(pE1) != NULL)
                    {
                      SmBrep *pBrep = pE2->GetBrep();
                      SmBoolean bFixedProblem = FALSE;
                      if (pE2->IsManifold()) {
                          SmTArray<SmFace*> sFaces;
                          pE2->GetFaces(sFaces);
                          if (sFaces.GetSize() > 1 && sFaces[0]->GetSurface() == sFaces[1]->GetSurface()) {
                              bFixedProblem = TRUE;
                              sDeletedEdges.Add(pE2);
                              SER(pBrep->DeleteEdge(pE2));
                              rbTopologyWasDeleted = TRUE;
                          }
                      }
                      if (!bFixedProblem) {
                          SmEdge *pE2Other = (SmEdge*)GetBrepMate(pE1);
                          if (pE2Other && pE2Other->IsManifold()) {
                              SmTArray<SmFace*> sFaces;
                              pE2Other->GetFaces(sFaces);
                              if (sFaces.GetSize() > 1 && sFaces[0]->GetSurface() == sFaces[1]->GetSurface()) {
                                  bFixedProblem = TRUE;
                                  RemoveRelationship(pE1,pE2Other);
                                  sDeletedEdges.Add(pE2Other);
                                  SER(pBrep->DeleteEdge(pE2Other));
                                  rbTopologyWasDeleted = TRUE;
                                  SER(Relate(pE1,pE2));
                              }
                          }
                      }
                      if (!bFixedProblem) {
                          SER(SM_ERR);
                      }
                    }  // end pE1 has a mate but pE2 does not check
                }  // end create EdgeMate relationship failure check
            } // end neither old-edge classification was deleted check

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              SmEdge *pEOther = (SmEdge*)GetOtherMate(pE1);
              SmEdge *pBE     = (SmEdge*)GetBrepMate(pE2);

              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1); m_pBrep->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,0); m_pOther->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,3, 0,1,1); Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
              if(FALSE)
                { // the DrawDetails() call is full of expensive gap calcs - only call when you want to see the extra information
                  smgfx_SetLook(2,3, 0,1,1) ; DrawDetails(3.0, 4.0) ; sm_GraphicsLoop() ;
                }
              smgfx_SetLook(3,4, 1,0,0); pE1->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(4,5, 0,1,0); pE2->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(5,6, 0,1,1); if (pEOther) pEOther->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(6,7, 1,1,0); if (pBE) pBE->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
        } // end check on relating mid-edges

      // See about relating Ends: Vertices.
      if (   rClass1.m_vEnd.GetPointClass() == SM_PC_VERTEX
          && rClass2.m_vEnd.GetPointClass() == SM_PC_VERTEX)
        {
          // get the vertices from the CurveClassification
          SmVertex *pV1 = (SmVertex*)rClass1.m_vEnd.GetObject();
          SmVertex *pV2 = (SmVertex*)rClass2.m_vEnd.GetObject();
          ULONG lFoundIndex;

          // replace deleted vertices with surviving ones in CurveClassification
          if (sDeletedV.FindElement(pV1,lFoundIndex))
            {
              pV1 = sSurvivingV[lFoundIndex];
              rClass1.m_vEnd.SetClassObject(SM_PC_VERTEX, pV1);
            }
          if (sDeletedV.FindElement(pV2,lFoundIndex))
            {
              pV2 = sSurvivingV[lFoundIndex];
              rClass2.m_vEnd.SetClassObject(SM_PC_VERTEX, pV2);
            }

          // when both Brep and OtherBrep are present (the usual case)
          if (m_pBrep && m_pOther)
            {
              // if vertices have old mates - squeeze old and new mates
              eSqueezeStat = CheckForSqueezeVertices( pV1, pV2, sDeletedV, sSurvivingV, sDeletedEdges );
            }

          // remember if edges were deleted by Squeeze
          if (   sDeletedEdges.GetSize() != 0
              || sDeletedV.GetSize()     != 0)
            {
              rbTopologyWasDeleted = TRUE;
            }

          // create interval-end vertex/vertex mate relationship
          if ( eSqueezeStat == SM_SUCCESS )  // [B458 B459]
            { SER( Relate( pV1, pV2 )); }

          // remember the event
          bDidLastEnd = TRUE;

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              Dump();
              SmVertex *pVOther = (SmVertex*)GetOtherMate(pV1);
              SmVertex *pBV = (SmVertex*)GetBrepMate(pV2);

              smgfx_Erase();
              smgfx_SetLook(1,2, 0,0,1); m_pBrep->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,0); m_pOther->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(2,3, 0,1,1); Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 1,0,0); pV1->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(4,5, 0,1,0); pV2->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(5,6, 0,1,1); if (pVOther) pVOther->Draw(); sm_GraphicsLoop();
              smgfx_SetLook(6,7, 1,1,0); if (pBV) pBV->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
        } // end check on relating end-vertices
      else
        {
          // remember not creating interval-end vertex/vertex mate relationship
          bDidLastEnd = FALSE;

        } // end both interval-ends did not classify to vertices branch

    } // end iter every rCrvClass1 classification interval

  // Pass back deleted topology if requested.
  if ( paDeletedEdges != NULL )
    {
      paDeletedEdges->ReSet();
      paDeletedEdges->Append( sDeletedEdges );
    }
  if ( paDeletedVertices != NULL )
    {
      paDeletedVertices->ReSet();
      paDeletedVertices->Append( sDeletedV );
    }

  return SM_SUCCESS;

} // end SmTopologyIntersector::MergeCurveClasses

/*******************************************************************//**
PURPOSE: Merge a curve which is on two surfaces belonging to two faces
         of two different Breps.

    Create edges and insert them into the topology graphs
    of the faces owning pSurface and pOtherSurface representing every
    segment of the curve which is simultaneously contained in both faces.

    Inserting edges into the topology graph may create new vertices,
    and split existing edges and faces.  Shells and Loops are created
    and inserted into the topology graphs as needed when the input
    p3DCurve does not intersect any existing edges.

NOTES: The surface and other surface may be NULL if the
    corresponding brep edge or other brep edge are not NULL.
    This is an optimization that allows us to skip classification
    if we already know the curve represents an edge in one of the
    breps.  This happens when a pair of faces are found to have
    coincident surfaces or when inserting fillets into a
    Brep.  All the edges of one Face are merged into the other
    face's Breps.  Under such circumstances this function assumes
    the the curve is NOT the result of a surface/surface intersection
    and does not apply any surf/surf tolerance tightening heuristics.
***********************************************************************/
SmStatus SmTopologyIntersector::MergeCurveOnSurfaces
  (SmSurface             * pSurface,          // in : 1st surface of surface/surface intersection
   SmSurface             * pOtherSurface,     // in : 2nd surface of surface/surface intersection
   double                  dCurveTolerance,   // in : min length for non-degenerate curve. Degen Crv treated as pt.
   SmCurve               * p3DCurve,          // in : surf/surf 3D xsect curve
   SmCurve               * pUVCurve1,         // in : opt associated UV curve for 1st surface
   SmCurve               * pUVCurve2,         // in : opt associated UV curve for 2nd surface
   SmEdge                * pBrepEdge,         // in : opt already existing edge corresponding to 3DCurve on pSurface,
                                              //      NULL to ignore
   SmEdge                * pOtherEdge,        // in : opt already existing edge which corresponding to 3DCurve on pOtherSurface,
                                              //      NULL to ignore
   SmBoolean             & rbDeletedTopology, // out: TRUE = multiply mated Vertices or Edges squeezed in MergeCurveClasses().
                                              //      FALSE= no Vertices or Edges squeezed
   SmTArray< SmEdge* >   * paDeletedEdges,    // out: ptrs to deleted edges
   SmTArray< SmVertex* > * paDeletedVertices) // out: ptrs to deleted vertices
{
  rbDeletedTopology = FALSE;

  // Get the face or composite face from the surface.
  SmFace *pThisFace  = (pSurface) ? (SmFace*)pSurface->GetFace() : NULL ;
  if(pSurface) { NER(pThisFace) ; }

  SmFace *pOtherFace = (pOtherSurface) ? (SmFace*)pOtherSurface->GetFace() : NULL ;
  if(pOtherSurface) { NER(pOtherFace) ; }

  // when 3D curve is short
  if (p3DCurve->IsDegenerate(dCurveTolerance))
    {
      // Don't allow degenerate cases when using edges
      if (   pSurface      == NULL
          || pOtherSurface == NULL)
        {
          SER(SM_ERR);
        }

      // get curve point
      SmPoint3d sPnt;
      SmExtent1d sIvl = p3DCurve->GetNaturalInterval();
      SER(p3DCurve->EvaluatePoint(sIvl.GetMin(),sPnt));

      // classify point against both faces (in/out trimmed face region)
      SmPointClassification sPointClass1(dCurveTolerance, &GetContext());   // this tol is unlikely to be correct - check it
      SmPointClassification sPointClass2(dCurveTolerance, &GetContext());   // this tol is unlikely to be correct - check it

      SER( pThisFace ->Point3DClassify( sPnt, dCurveTolerance, TRUE, sPointClass1 ));
      SER( pOtherFace->Point3DClassify( sPnt, dCurveTolerance, TRUE, sPointClass2 ));

      // when point was classified on both faces (it was close enough to the surfaces)
      if (   sPointClass1.GetPointClass() != SM_PC_UNKNOWN
          && sPointClass2.GetPointClass() != SM_PC_UNKNOWN)
        {
          // merge the point classification (not the curve) onto both faces
          //  SmPointClassificationType == SM_PC_VERTEX, update the vertex tolerance
          //                            == SM_PC_EDGE,   call SmBrep::MakeVertexSplitEdge
          //                            == SM_PC_FACE,   call SmBrep::MakeVertexLoop
          //                            == SM_PC_REGION, call SmBrep::MakeVertexShell
          // set PointClassification->Object = MergedVertex
          SER( sPointClass1.MergeIntoObject( sPnt ));
          SER( sPointClass2.MergeIntoObject( sPnt ));

          // each merge creates a vertex in its SmBrep - relate those vertices
          SmVertex *pV1 = (SmVertex*)sPointClass1.GetObject();
          SmVertex *pV2 = (SmVertex*)sPointClass2.GetObject();
          SER(Relate(pV1,pV2));

        } // end if both points classified properly check
      return SM_SUCCESS;

    } // end degenerate short edge check

#ifdef SM_DEBUG_CODE // GWC_BOOLEAN_SETBREAK
SmBoolean bDebugMe = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ; 
static ULONG lDebugCount2 = 0 ;
  // draw relatedTopology(magenta), Breps(blue,green), Surfs(cyan,yellow)
  //      Faces(black), xSectBox(grey), xSectCurve(red)
  if(bDebugMe || lDebugCount == lCount || lDebugCount2 == lCount)
    {
      // dump SmTopolgyIntersector and Breps
      Dump() ;
      SM_DUMP_AND_ASSERT_VALID(m_pBrep) ;
      SM_DUMP_AND_ASSERT_VALID(m_pOther) ;
      SM_DUMP_AND_ASSERT_VALID(p3DCurve) ;

      // draw Breps, surfs, faces, xSectCurve, and optional edges
      smgfx_Erase() ;
      smgfx_SetLook(3,5, 1,0,0) ; p3DCurve->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,7, 1,0,1) ; if(pBrepEdge)  { pBrepEdge->Draw() ; } sm_GraphicsLoop();
      smgfx_SetLook(6,8, 1,0,1) ; if(pOtherEdge) { pOtherEdge->Draw() ; } sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,1) ; if(m_pBrep) { m_pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(m_pOther){ m_pOther->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pSurface){ pSurface->DrawUV(4, 4) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; if(pOtherSurface){ pOtherSurface->DrawUV(4, 4) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pThisFace){ pThisFace->Draw(SM_DM_CROSSHATCH, 6, 6) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ; if(pOtherFace){ pOtherFace->Draw(SM_DM_CROSSHATCH, 6, 6) ; } sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // old idea : for 8 passes - if MergeCurveClasses() failed retry with bigger tolerance
  // currently: don't iterate - fix MergeCurveClasses problems with extra fixup code
  //for (ULONG kkk=0; kkk<8; kkk++) {
 
  // next: Classify curve against thisFace (get curve segments inside trimmed face boundaries)
  
  // CurveClassification locals
  SmCurveClassification sCrvClass1(p3DCurve,
                                   p3DCurve->GetNaturalInterval(),
                                   pUVCurve1,
                                   dCurveTolerance);
  SmCurveClassification sCrvClass2(p3DCurve, 
                                   p3DCurve->GetNaturalInterval(), 
                                   pUVCurve2, 
                                   dCurveTolerance);

  // when asked - set up fillet data
  if(m_bDoingFillets)
    {
      sCrvClass1.SetFilletData(&sCrvClass2, TRUE) ;
      sCrvClass2.SetFilletData(&sCrvClass1, TRUE) ;
    }

  // Assume this is an intersection curve only when both pBrepEdge and pOtherEdge are NULL (GWC:CHANGE_SSS)
  if(   pBrepEdge  == NULL
     && pOtherEdge == NULL)
       { sCrvClass1.SetIntersectData(pSurface, pOtherSurface, pUVCurve2, TRUE, this) ; }
  else { sCrvClass1.SetSurface1(pSurface) ; }

  if ( m_bIgnoreTangency )
    { sCrvClass1.SetIgnoreTangency( TRUE ); }

  // when curve is coincident with an existing brep edge
  if (pBrepEdge)
    {
      // Set sCrvClass1 as such {m_vStart=SM_PC_VERTEX, m_vMid=SM_PC_EDGE, m_vEnd=SM_PC_VERTEX}
      SER(sm_SetEdgeClass(sCrvClass1,pBrepEdge));
    }
  else // Intersect this curve with the topology objects of this face.
    {
      // XSect p3DCurve with ThisFace - use XSects to load sCrvClass2 with bounded-segment sequence
      //   Inside the sCrvClass1 object:
      //     classify each segment as oneof:
      //       a. SM_PC_EDGE    = the curve segment is coincident with a Face's edge
      //       b. SM_PC_FACE    = the curve segment lies on the face
      //       c. SM_PC_UNKNOWN = the curve segment is outside the face or may need an extra
      //                          ray_casting step to find out that its inside a face.
      //     classify each segment boundary as oneof:
      //       a. SM_PC_VERTEX = The curve intersected one of the Face's existing vertices.
      //       b. SM_PC_EDGE   = The curve intersected one of the Face's existing edges.
      //   
      //     Each classification remembers the curve location (parameter range) and
      //     the face object (vertex, edge, or face) of the curve/FaceObject intersection.
      SER(pThisFace->CurveOnClassify(TRUE,sCrvClass1));
    }

#ifdef SM_DEBUG_CODE  // GWC_BOOLEAN_SETBREAK
SmBoolean bDebug1 = FALSE;
  // draw faces with sCrvClass1 classification objects
  if(bDebug1 || lDebugCount == lCount)
    {
      SM_DUMP_AND_ASSERT_VALID(&sCrvClass1);

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(m_pBrep) { m_pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(m_pOther){ m_pOther->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pThisFace) pThisFace->Draw(SM_DM_CROSSHATCH, 6, 6) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; if(pOtherFace) pOtherFace->Draw(SM_DM_CROSSHATCH, 6, 6) ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,5, 0,0,0) ; sCrvClass1.Draw() ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // If classification found no intersections, quit.
  if (    sCrvClass1.GetSize() == 1
       && sCrvClass1[0].m_vStart.GetPointClass() == SM_PC_UNKNOWN
       && sCrvClass1[0].m_vMid.GetPointClass()   == SM_PC_UNKNOWN
       && sCrvClass1[0].m_vEnd.GetPointClass()   == SM_PC_UNKNOWN )
    { return SM_SUCCESS; }

  // next: Classify the curve against otherFace (get curve segments inside trimmed face boundaries)

  // Assume this is an intersection curve only when both pBrepEdge and pOtherEdge are NULL (GWC:CHANGE_SSS)
  if(   pBrepEdge  == NULL
     && pOtherEdge == NULL)
       { sCrvClass2.SetIntersectData(pOtherSurface, pSurface, pUVCurve1, sCrvClass1.GetIsCurveCurrent(), this) ; }
  else { sCrvClass2.SetSurface1(pOtherSurface) ; }

  if ( m_bIgnoreTangency )
    { sCrvClass2.SetIgnoreTangency( TRUE ); }

  // when curve is coincident with an existing brep edge
  if (pOtherEdge)
    {
      // reuse the edge
      SER(sm_SetEdgeClass(sCrvClass2,pOtherEdge));
    }
  else // Intersect this curve with the topology objects of this OtherFace.
    {
      // XSect p3DCurve with OtherFace - use XSects to load sCrvClass2 with bounded-segment sequence
      SER(pOtherFace->CurveOnClassify(TRUE,sCrvClass2));

      // make sure the obsolete bit is the same in both classifications
      sCrvClass1.SetIsCurveCurrent(sCrvClass2.GetIsCurveCurrent()) ;
    }

#ifdef SM_DEBUG_CODE  // GWC_BOOLEAN_SETBREAK
  if(bDebug1 || lDebugCount == lCount)
  // draw faces with sCrvClass2 classification objects
    {
      ULONG di ;
      const SmCurve * pCurve1 = sCrvClass1.GetCurve() ;
      const SmCurve * pCurve2 = sCrvClass2.GetCurve() ;
      sCrvClass2.Dump();

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(m_pBrep) { m_pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(m_pOther){ m_pOther->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ; if(pThisFace) pThisFace->Draw(SM_DM_CROSSHATCH, 6, 6) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0) ; if(pOtherFace) pOtherFace->Draw(SM_DM_CROSSHATCH, 6, 6) ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 0,0,0) ; sCrvClass2.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,1) ; if(pCurve1) pCurve1->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5) ; for(di=0;di<sCrvClass1.GetSize();di++)
        { SmFace   * pStartFace   = SM_CAST_PTR(SmFace,   sCrvClass1[di].m_vStart.GetObject()) ;
          SmEdge   * pStartEdge   = SM_CAST_PTR(SmEdge,   sCrvClass1[di].m_vStart.GetObject()) ;
          SmVertex * pStartVertex = SM_CAST_PTR(SmVertex, sCrvClass1[di].m_vStart.GetObject()) ;
          SmFace   * pMidFace     = SM_CAST_PTR(SmFace,   sCrvClass1[di].m_vMid.GetObject()) ;
          SmEdge   * pMidEdge     = SM_CAST_PTR(SmEdge,   sCrvClass1[di].m_vMid.GetObject()) ;
          SmFace   * pEndFace     = SM_CAST_PTR(SmFace,   sCrvClass1[di].m_vEnd.GetObject()) ;
          SmEdge   * pEndEdge     = SM_CAST_PTR(SmEdge,   sCrvClass1[di].m_vEnd.GetObject()) ;
          SmVertex * pEndVertex   = SM_CAST_PTR(SmVertex, sCrvClass1[di].m_vEnd.GetObject()) ;
          smgfx_ChangeColor( di != 0 ); if(pStartFace) pStartFace->DrawUV(); if(pStartEdge) { SmExtent1d sExt = pStartEdge->GetInterval();  pStartEdge->GetCurve()->DrawParams( &sExt ); } if(pStartVertex) pStartVertex->Draw(); sm_GraphicsLoop();
          smgfx_ChangeColor();      if(pMidFace) pMidFace->DrawUV(); if(pMidEdge) { SmExtent1d sExt = pMidEdge->GetInterval();  pMidEdge->GetCurve()->DrawParams( &sExt ); } sm_GraphicsLoop();
          smgfx_ChangeColor();      if(pEndFace) pEndFace->DrawUV(); if(pEndEdge) { SmExtent1d sExt = pEndEdge->GetInterval();  pEndEdge->GetCurve()->DrawParams( &sExt ); } if(pEndVertex) pEndVertex->Draw(); sm_GraphicsLoop();
        }
      smgfx_SetLook(3,4, 0,1,1) ; if(pCurve2) pCurve2->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7) ; for(di=0;di<sCrvClass2.GetSize();di++)
        { SmFace   * pStartFace   = SM_CAST_PTR(SmFace,   sCrvClass2[di].m_vStart.GetObject()) ;
          SmEdge   * pStartEdge   = SM_CAST_PTR(SmEdge,   sCrvClass2[di].m_vStart.GetObject()) ;
          SmVertex * pStartVertex = SM_CAST_PTR(SmVertex, sCrvClass2[di].m_vStart.GetObject()) ;
          SmFace   * pMidFace     = SM_CAST_PTR(SmFace,   sCrvClass2[di].m_vMid.GetObject()) ;
          SmEdge   * pMidEdge     = SM_CAST_PTR(SmEdge,   sCrvClass2[di].m_vMid.GetObject()) ;
          SmFace   * pEndFace     = SM_CAST_PTR(SmFace,   sCrvClass2[di].m_vEnd.GetObject()) ;
          SmEdge   * pEndEdge     = SM_CAST_PTR(SmEdge,   sCrvClass2[di].m_vEnd.GetObject()) ;
          SmVertex * pEndVertex   = SM_CAST_PTR(SmVertex, sCrvClass2[di].m_vEnd.GetObject()) ;
          smgfx_ChangeColor( di != 0 ); if(pStartFace) pStartFace->DrawUV(); if(pStartEdge) { SmExtent1d sExt = pStartEdge->GetInterval();  pStartEdge->GetCurve()->DrawParams( &sExt ); } if(pStartVertex) pStartVertex->Draw(); sm_GraphicsLoop();
          smgfx_ChangeColor();      if(pMidFace) pMidFace->DrawUV(); if(pMidEdge) { SmExtent1d sExt = pStartEdge->GetInterval();  pMidEdge->GetCurve()->DrawParams( &sExt ); } sm_GraphicsLoop();
          smgfx_ChangeColor();     if(pEndFace) pEndFace->DrawUV();  if(pEndEdge) { SmExtent1d sExt = pStartEdge->GetInterval();  pEndEdge->GetCurve()->DrawParams( &sExt ); } if(pEndVertex) pEndVertex->Draw(); sm_GraphicsLoop();
        }

      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // If classification found no intersection, quit.
  if (    sCrvClass2.GetSize() == 1
       && sCrvClass2[0].m_vStart.GetPointClass() == SM_PC_UNKNOWN
       && sCrvClass2[0].m_vMid.GetPointClass()   == SM_PC_UNKNOWN
       && sCrvClass2[0].m_vEnd.GetPointClass()   == SM_PC_UNKNOWN )
    { return SM_SUCCESS; }

  double dNewTol;
  SmBSplineCurve *pNewBSC = NULL ;

  // when XSect curve lies along two edges from two faces that are partially coincident
  //  - make a new curve that is the coincident sub-interval
  SER(sCrvClass1.TestForPartialCoincidentEdges(sCrvClass2,dNewTol,pNewBSC));
  if (dNewTol < 0.0) { SER(SM_ERR); }
  SmObjDelete sCleanBSC(pNewBSC);

  // when a coincident sub-interval curve was made
  if (   pNewBSC
      && pSurface      != NULL
      && pOtherSurface != NULL)
    {
      // classify the coincident curve against face1
      SmCurveClassification sNewCrvClass1(pNewBSC,
                                          pNewBSC->GetNaturalInterval(),
                                          NULL,
                                          dNewTol);
// GWC:CHANGE_SSS - add next block
      if(   pBrepEdge  == NULL
         && pOtherEdge == NULL)
           { sNewCrvClass1.SetIntersectData(pSurface, pOtherSurface, pUVCurve2, TRUE, this) ; }
      else { sNewCrvClass1.SetSurface1(pSurface) ; }

      SER(pThisFace->CurveOnClassify(TRUE,sNewCrvClass1));

      // classify the coincident curve against face2
      SmCurveClassification sNewCrvClass2(pNewBSC,
                                          pNewBSC->GetNaturalInterval(),
                                          NULL,
                                          dNewTol);
// GWC:CHANGE_SSS - add next block
      if(   pBrepEdge  == NULL
         && pOtherEdge == NULL)
           { sNewCrvClass2.SetIntersectData(pOtherSurface, pSurface, pUVCurve1, sCrvClass1.GetIsCurveCurrent(), this) ; }
      else { sNewCrvClass2.SetSurface1(pOtherSurface) ; }
      SER(pOtherFace->CurveOnClassify(TRUE,sNewCrvClass2));

      // merge those curves into their respective SmBreps
      SER( MergeCurveClasses( sNewCrvClass1,        // i/o: 1st Target Curve Classification
                              sNewCrvClass2,        // i/o: 2nd Target Curve Classification
                              FALSE,                // in : TRUE = Merge all point classifications
                              rbDeletedTopology,    // out: TRUE = small edges were deleted by squeeze
                              paDeletedEdges,       // out: ptrs to deleted edges
                              paDeletedVertices )); // out: ptrs to deleted vertices

      // We still have some 3D curve left over Classify again

      // classify the curve against face 1 for the 2nd time
      SmCurveClassification sCrvClass12(p3DCurve, p3DCurve->GetNaturalInterval(),
                                        pUVCurve1, dCurveTolerance);
// GWC:CHANGE_SSS - add next line
      if(   pBrepEdge  == NULL
         && pOtherEdge == NULL)
           { sCrvClass12.SetIntersectData(pSurface, pOtherSurface, pUVCurve2, TRUE, this) ; }
      else { sCrvClass12.SetSurface1(pSurface) ; }

      if (pBrepEdge) { SER(sm_SetEdgeClass(sCrvClass12,pBrepEdge)); }
      else           { SER(pThisFace->CurveOnClassify(TRUE,sCrvClass12)); }

      // classify the curve against face 2 for the 2nd time
      SmCurveClassification sCrvClass22(p3DCurve, p3DCurve->GetNaturalInterval(),
                                        pUVCurve2, dCurveTolerance);
// GWC:CHANGE_SSS - add next line
      if(   pBrepEdge  == NULL
         && pOtherEdge == NULL)
           { sCrvClass22.SetIntersectData(pOtherSurface, pSurface, pUVCurve1, sCrvClass1.GetIsCurveCurrent(), this) ; }
      else { sCrvClass22.SetSurface1(pOtherSurface) ; }

      if (pOtherEdge) { SER(sm_SetEdgeClass(sCrvClass22,pOtherEdge)); }
      else            { SER(pOtherFace->CurveOnClassify(TRUE,sCrvClass22)); }

      SER( MergeCurveClasses( sCrvClass12,         // i/o: 1st Target Curve Classification
                              sCrvClass22,         // i/o: 2nd Target Curve Classification
                              FALSE,               // in : TRUE = Merge all point classifications
                              rbDeletedTopology,   // out: TRUE = small edges were deleted by squeeze
                              paDeletedEdges,      // out: ptrs to deleted edges
                              paDeletedVertices )) // out: ptrs to deleted vertices;

      return SM_SUCCESS;
    } // end when a coincident sub-interval curve was made

  // arrive here after curve is classified against both Faces

#ifdef SM_DEBUG_CODE
  // dump classifications - draw interval curves and points(cyan) and their objects(yellow)
  if (bDebugMe || lCount == lDebugCount)
    {
      ULONG di ;
      double dCH = .1 ;
      sCrvClass1.Dump();
      sCrvClass2.Dump();

      // draw interval curves, points(cyan), and their objects(yellow)
      smgfx_Erase() ;
      dCH = smgfx_SetLook(3,5, 1,0,0, .0001) ; p3DCurve->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(6,7, 0,1,1, .0001) ; for(di=0;di<sCrvClass1.GetSize();di++)
                                            { p3DCurve->DrawParams(&sCrvClass1[di].GetInterval()) ; smgfx_ChangeColor() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(8,9, 1,0,1, .0001) ; for(di=0;di<sCrvClass2.GetSize();di++)
                                            { p3DCurve->DrawParams(&sCrvClass2[di].GetInterval()) ; smgfx_ChangeColor() ; sm_GraphicsLoop() ; }
      smgfx_SetLook(1,2, 0,0,1, .0001) ; if(m_pBrep) { m_pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0, .0001) ; if(m_pOther){ m_pOther->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1, .0001) ; if(pThisFace) pThisFace->Draw(SM_DM_CROSSHATCH, 6, 6) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,1,0, .0001) ; if(pOtherFace) pOtherFace->Draw(SM_DM_CROSSHATCH, 6, 6) ; sm_GraphicsLoop() ;
      smgfx_SetCurveClassificationObjectLook(5,7,
                                             0, 0,1,
                                             0,.5,1,
                                             .5,0,1) ; sCrvClass1.DrawObjects(); sm_GraphicsLoop();
      smgfx_SetCurveClassificationObjectLook(7,9,
                                             0, 1,0,
                                             0,1,.5,
                                             .5,1,0) ; sCrvClass2.DrawObjects(); sm_GraphicsLoop();
      smgfx_SetChordHeight(dCH) ;
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
  // 1. Create and insert an SmEdge into the Brep and the OBrep topology graphs
  //    for each interval classified to geometry in both Breps.
  //    side effects: SplitFaces, SplitEdges, InsertEdges, InsertVertices.
  // 2. Create a mate relationship between the objects of every
  //    matching end-point and mid-point CurveClassification.
  //    side effects: equalize tolerances of mated objects,
  //                  squeeze multiply mated vertices -> DeleteVertices, DeleteEdges
  //
  SER( MergeCurveClasses(sCrvClass1,           // i/o: 1st Target Curve Classification
                         sCrvClass2,           // i/o: 2nd Target Curve Classification
                         FALSE,                // in : TRUE = Merge all point classifications
                         rbDeletedTopology,    // out: TRUE = small edges were deleted by squeeze
                         paDeletedEdges,       // out: ptrs to deleted edges
                         paDeletedVertices )); // out: ptrs to deleted vertices

#ifdef SM_DEBUG_CODE
  if(bDebugMe || lCount == lDebugCount)
    {
      SM_DUMP_AND_ASSERT_VALID(&sCrvClass1);
      SM_DUMP_AND_ASSERT_VALID(&sCrvClass2);
      this->Dump() ;
      if ( FALSE ) 
        { m_pBrep->Dump(SM_BD_POINTS) ;
          m_pOther->Dump(SM_BD_POINTS) ;
        }
    }
#endif // SM_DEBUG_CODE
//        if (MergeCurveClasses(sCrvClass1,sCrvClass2,FALSE) != SM_SUCCESS) {
//            // If we have trouble merging increase tolerance and classify again
//            dCurveTolerance = dCurveTolerance * 2.0;
//            continue; }

  // } // end obsolete iter kkk from 0 to 7

  return SM_SUCCESS;

} // end SmTopologyIntersector::MergeCurveOnSurfaces

/*******************************************************************//**
PURPOSE: Make the Breps containing two coincident surfaces compatible
            by making sure every BSurface->Face->Edge interval that classifies
            to geometry in the OSurface->Face is coincident with a
            a OSurface->Face->Edge and vice versa.

            When a coincident edge is missing from a Brep it is added
            with a merge step.

RETURNS --- List of all coincident face pairs whose edges are all guaranteed
            to have coincident edge partners and be listed in the
            m_pBToO and m_pOToB mated pairs lists.

NOTES: Someday we may want to add vertices to this process.

***********************************************************************/
SmStatus SmTopologyIntersector::MergeCoincidentSurfaces
  (SmSurface * pBSurface,          // in : 1st surface coincident with 2nd surface
   SmSurface * pOSurface,          // in : 2nd surface coincident with 1st surface
   SmTArray<SmFace*> & rFaces,     // out: list of BSurface faces coincident with OSurface faces
   SmTArray<SmFace*> & rOFaces)    // out: list of OSurface faces coincident with BSurface faces
{
  // init output
  rFaces.ReSet();
  rOFaces.ReSet();

  // locals
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
#endif // SM_DEBUG_CODE

  SmFace *pBFace = (SmFace*)pBSurface->GetFace();
  SmFace *pOFace = (SmFace*)pOSurface->GetFace();

  SmEdge         * sEData1[64];
  SmEdge         * sEData2[64];
  SmBSplineCurve * sBData[16];
  SmTArray<SmEdge*>         sEdgesB(64,sEData1);
  SmTArray<SmEdge*>         sEdgesO(64,sEData2);
  SmTArray<SmBSplineCurve*> sUVCurves(16,sBData);

  // detect special case - 2 planar surfaces
  SmBoolean bAreBothPlanes =  (   pBSurface->IsKindOf(SmPlane_TYPE)
                               && pOSurface->IsKindOf(SmPlane_TYPE))
                             ? TRUE
                             : FALSE ;

  // get surface bounding boxes
  SmExtent3d sThisFaceBBox, sOtherFaceBBox, sIntersectionBBox;
  SER(pBFace->CalculateBoundingBox(sThisFaceBBox));
  SER(pOFace->CalculateBoundingBox(sOtherFaceBBox));
  sThisFaceBBox.ExpandAbsolute(m_dThisApproxTol3d);

  // no work - bounding boxes don't intersect
  if (sThisFaceBBox.AreDisjoint(sOtherFaceBBox))
    { return SM_SUCCESS; }

  // get FaceBBox/FaceBBox intersection
  SER(sThisFaceBBox.Intersect(sOtherFaceBBox,sIntersectionBBox));

  // loop
  //   Project the curves of one face onto the other
  //   Merge the projected curves into the surface faces
  //   Repeat - until merge no longer causes topology changes
  ULONG ii, jj, kk;

  SmBoolean bTopologyChanged = TRUE;
  while ( bTopologyChanged )
    {
      bTopologyChanged = FALSE;

      // get surface->Face ptrs
      pBFace = (SmFace*)pBSurface->GetFace();
      pOFace = (SmFace*)pOSurface->GetFace();

#ifdef SM_DEBUG_CODE
      if (bDebugMe)
        {
          SmBrep *pBrep1 = pBFace->GetBrep() ;
          SmBrep *pBrep2 = pOFace->GetBrep() ;

          // draw pBSurf(blue), pOSurface(green), and their faces(black)
          smgfx_Erase();
          smgfx_SetLook(1,2, 0,0,1); if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,0); if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,1,1); pBFace->GetSurface()->DrawUV(); sm_GraphicsLoop();
          smgfx_SetLook(1,2, 0,0,0); pBFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 1,1,0); pOFace->GetSurface()->DrawUV(); sm_GraphicsLoop();
          smgfx_SetLook(3,4, 0,0,0); pOFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
          smgfx_SetLook(2,3, .3,.3,.3); sIntersectionBBox.Draw(pBFace->GetContext()) ;   sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE
      // step 1 project and merge BFace->edges into OFace
      pBFace->GetEdges(sEdgesB);
      ULONG lNumEdgesB = sEdgesB.GetSize();
      for (ii=0; ii<lNumEdgesB; ii++)
        {
          // skip edges already mated to other Brep edges
          SmEdge *pBEdge = sEdgesB[ii];
          SmEdge *pOEdge = (SmEdge*)GetOtherMate(pBEdge);

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              SmBrep *pBrep1 = pBFace->GetBrep() ;
              SmBrep *pBrep2 = pOFace->GetBrep() ;
              this->Dump();

              // draw pBSurf(blue), pOSurface(green), and their faces(black)
              //      current pBFace->edge(red)
              smgfx_Erase();
              smgfx_SetLook(2,3, 1,0,1); this->Draw(3,5, 7,9)             ; sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1); if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,0); if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1); pBFace->GetSurface()->DrawUV(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,0); pBFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
              smgfx_SetLook(3,4, 1,1,0); pOFace->GetSurface()->DrawUV(); sm_GraphicsLoop();
              smgfx_SetLook(3,4, 0,0,0); pOFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
              smgfx_SetLook(6,7, 1,0,0); pBEdge->Draw();                 sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
          // skip edges already mated to OtherBrep
          if (pOEdge) continue;

          // Get BEdge's curve and bounding box
          SmExtent3d      sEdgeBBox;
          SmCurve *pEdgeCurve = pBEdge->GetCurve();
          SER(pEdgeCurve->CalculateBoundingBox(pBEdge->GetInterval(),&sEdgeBBox));
          sEdgeBBox.ExpandAbsolute(pBEdge->GetTolerance());

          // no work - EdgeBBox does not intersect FaceBBox/FaceBBox intersectionBBox
          if (sEdgeBBox.AreDisjoint(sIntersectionBBox))
            { continue; }

          double dDistToSurface = 0.0, dDeviation = 0.0;

          // pick a tolerance

          // GWC_NEEDS_WORK ANOTHER_PLACE_MIXING_APPROX_AND_TOPOLOGY_TOLS GWC_LINE ;
//cbiTol: New topology will generally be created with this tolerance, such as
//cbiTol  in SmPointClassification::Merge().  This will generally result in
//cbiTol  doubling of tolerances at every step of a Boolean operation.

          double dTol = pOFace->GetTolerance() + pBEdge->GetTolerance();
          dTol = smos_Max(dTol,m_dThisApproxTol3d);

          // make temporary BEdge->Curve Copy so the original won't be modified
          SmExtent1d   sTmpIvl = pBEdge->GetInterval();
          SmCurve    * pECopy;
          SER(pEdgeCurve->Copy(*pEdgeCurve->GetContext(),pECopy)); NER(pECopy);
          SmObjDelete sCleanCopyE(pECopy);
          pECopy->SetOwner( pEdgeCurve->GetOwner() ); // Set owner for access to Tol value

          // trim BEdge->Curve to BEdge interval
          SER(pECopy->Trim(sTmpIvl));   // may snap sIvl by tol to existing knots

          // when surfaces are planar - merge curve into Breps without making UVTrimCurves first
          if (bAreBothPlanes)
            {
              ULONG lNumBrepEdges = sEdgesB.GetSize();

              // merge the EdgeCurve into both surfaces
              SER(MergeCurveOnSurfaces(pBFace->GetSurface(),
                                       pOFace->GetSurface(),
                                       dTol,   pECopy,
                                       NULL,   NULL,         // in : UVTrimCurves for pECopy
                                       pBEdge, NULL,         // in : edges already mapped to Curve
                                       bTopologyChanged));   // out: edges and vertices deleted by squeezing

              // if topology was deleted - refresh the pBFace ptr
              // This can happen with Composite Faces.  [090830]
              if ( bTopologyChanged )
                {
                  pBFace = (SmFace*)pBSurface->GetFace();
                }

              // if there is a face - note if its edge count has changed
              if (pBFace != NULL)
                {
                  pBFace->GetEdges(sEdgesB);
                  if (sEdgesB.GetSize() != lNumBrepEdges)
                    {
                      bTopologyChanged = TRUE;
                    }
                }
            } // end planar surfaces branch
          else // else for general surfaces - get OSurface->UVTrimCurves 1st then merge those into Breps
            {
              // Get temporary OSurface UVTrimCurve for BEdge->Curve
              if (pOSurface->DropAndTrimCurve(*pEdgeCurve->GetContext(),
                                              pOFace->GetUVDomain(),
                                              *pECopy,
                                              pECopy->GetNaturalInterval(),
                                              dTol/4.0,
                                              dDistToSurface,
                                              dDeviation, sUVCurves, TRUE, TRUE ) != SM_SUCCESS)
                { continue; }
              SmObjsDelete<SmBSplineCurve*> sCleanUV(&sUVCurves);

              // skip curves that did not project or that were too far from the surface
              if (sUVCurves.GetSize() == 0)   { continue; }
              if (dDistToSurface > dTol*10.0) { continue; }

static int cbiTol=0; // 1: dTol = smos_Max( m_dThisAPproxTol3d, dDist ); ... otherwise it gets set smaller

              // expand Tol as needed
if ( cbiTol==0 ) {
              dTol = smos_Max( dTol, dDistToSurface ); // (was m_dThisApproxTol3d, not dTol)
} else {
              dTol = smos_Max( m_dThisApproxTol3d, dDistToSurface );
} 
              // for every pOSurface UVTrimCurve
              //  (more than 1 if - 3dCurve wanders in and out of Surface boundaries,
              //                  - crosses a closed surface seam or
              //                  - projects to a closed surface seam)
              ULONG lNumUVCurves = sUVCurves.GetSize();
              for (jj=0; jj<lNumUVCurves; jj++)
                {
                  SmBSplineCurve * pUVCurve = sUVCurves[jj];
                  SmExtent1d       sIvl     = pUVCurve->GetNaturalInterval();
                  SmCurve        * pECopy2;

                  // get temporary curve copy trimmmed to UVTrimCurve extents
                  SER(pEdgeCurve->Copy(*pEdgeCurve->GetContext(),pECopy2)); NER(pECopy2);
                  SmObjDelete sCleanCopyE2(pECopy2);
                  SER(pECopy2->Trim(sIvl));  // may snap sIvl by tol to existing knots

                  // merge the Projected Curve into both surfaces with tol set large enough
                  // to account for the project distance
                  SER(MergeCurveOnSurfaces(pBFace->GetSurface(),
                                           pOFace->GetSurface(),
                                           dTol,   pECopy2,
                                           NULL,   pUVCurve,    // in : UVTrimCurves for pECopy2
                                           pBEdge, NULL,        // in : edges already mapped to Curve
                                           bTopologyChanged));  // out: edges and vertices deleted by squeezing

                  // if topology was deleted - refresh the pBFace ptr
                  if ( bTopologyChanged )
                    {
                      pBFace = (SmFace*)pBSurface->GetFace();
                    }

                  // Check to make sure there are the same number of edges otherwise we have
                  // to jump out here becuase pBEdge may no longer correspond to the geometry.
                  if ( pBFace != NULL )
                    {
                      ULONG lNumBrepEdges = sEdgesB.GetSize();
                      pBFace->GetEdges(sEdgesB);
                      if (sEdgesB.GetSize() != lNumBrepEdges)
                        {
                          bTopologyChanged = TRUE;
                        }
                    }

#ifdef SM_VALIDATE_TOPOLOGY
                  if(m_pBrep)  m_pBrep->ValidatePointers();
                  if(m_pOther) m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY
                  // if topology has changed - break and restart outer loop
                  if ( bTopologyChanged ) break;

                } // end iter every UVTrimCurve segment
            } // end general surface (not planar) branch

          // quit edge-loop with changed topology - restart big loop sequence
          if ( bTopologyChanged ) break;

        } // end iter every BFace->edge

      // when topology changed -- restart the outer loop
      if ( bTopologyChanged ) continue;

      // refresh Face pointers in case they changed with curve insertions
      pBFace = (SmFace*)pBSurface->GetFace();
      pOFace = (SmFace*)pOSurface->GetFace();

      // step 2 project and merge OFace->edges into BFace
      pOFace->GetEdges(sEdgesO);
      ULONG lNumEdgesO = sEdgesO.GetSize();
      for (jj=0; jj<lNumEdgesO; jj++)
        {
          // skip OBrep edges already mated to Brep edges
          SmEdge *pOEdge = sEdgesO[jj];
          SmEdge *pBEdge = (SmEdge*)GetBrepMate(pOEdge);

#ifdef SM_DEBUG_CODE
          if (bDebugMe)
            {
              SmBrep *pBrep1 = pBFace->GetBrep() ;
              SmBrep *pBrep2 = pOFace->GetBrep() ;

              // draw pBSurf(blue), pOSurface(green), and their faces(black)
              //      current pBFace->edge(red)
              smgfx_Erase();
              smgfx_SetLook(1,2, 0,1,0); if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,1); if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,1,1); pOFace->GetSurface()->DrawUV(); sm_GraphicsLoop();
              smgfx_SetLook(1,2, 0,0,0); pOFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
              smgfx_SetLook(3,4, 1,1,0); pBFace->GetSurface()->DrawUV(); sm_GraphicsLoop();
              smgfx_SetLook(3,4, 0,0,0); pBFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
              smgfx_SetLook(5,6, 1,0,0); pOEdge->Draw();                 sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
          if (pBEdge) { continue; }

          // Get OEdge's curve and bounding box
          SmExtent3d      sEdgeBBox;
          SmCurve *pEdgeCurve = pOEdge->GetCurve();
          SER(pEdgeCurve->CalculateBoundingBox(pOEdge->GetInterval(),&sEdgeBBox));
          sEdgeBBox.ExpandAbsolute(pOEdge->GetTolerance());

          // skip edges outside of surf/surf bounding box intersection
          if (sEdgeBBox.AreDisjoint(sIntersectionBBox))
            { continue; }

          double dDistToSurface = 0.0, dDeviation = 0.0;

          // pick a tolerance

          // GWC_NEEDS_WORK THIS_TOL_SHOULD_BE_OK_WHEN_APPROX_TOL_IS_LESS_THAN_BREP_TOL GWC_LINE ;
          double dTol = pBFace->GetTolerance() + pOEdge->GetTolerance();
          dTol = smos_Max(dTol,m_dThisApproxTol3d);

          // make temporary OEdge->Curve Copy so the original won't be modified
          SmExtent1d   sTmpIvl = pOEdge->GetInterval();
          SmCurve    * pECopy;
          SER(pEdgeCurve->Copy(*pEdgeCurve->GetContext(),pECopy)); NER(pECopy);
          SmObjDelete sCleanCopyE3(pECopy);
          pECopy->SetOwner( pEdgeCurve->GetOwner() ); // Set owner for access to Tol value

          // trim OEdge->Curve to BEdge interval
          SER(pECopy->Trim(sTmpIvl));  // may snap sIvl by tol to existing knots
          pECopy->SetOwner( pEdgeCurve->GetOwner() ); // Set owner for access to Tol value

          ULONG lOEdgesCount = sEdgesO.GetSize();

          // when surfaces are planar - merge curve into Breps without making UVTrimCurves first
          if (bAreBothPlanes)
            {
              SER(MergeCurveOnSurfaces(pBFace->GetSurface(),
                                       pOFace->GetSurface(),
                                       dTol, pECopy,
                                       NULL, NULL,          // in : UVTrimCurves for pECopy
                                       NULL, pOEdge,        // in : edges already mapped to Curve
                                       bTopologyChanged));  // out: edges and vertices deleted by squeezing

              // if topology has changed - refresh the pOFace ptr
              if ( bTopologyChanged )
                {
                  pOFace = (SmFace*)pOSurface->GetFace();
                }

              // if there is a face - note if its edge count has changed
              if (pOFace != NULL)
                {
                  pOFace->GetEdges(sEdgesO);
                  if (sEdgesO.GetSize() != lOEdgesCount)
                    {
                      bTopologyChanged = TRUE;
                    }
                }
            } // end planar surfaces branch
          else // else for general surfaces - get OSurface->UVTrimCurves 1st then merge those into Breps
            {
              // Get temporary BSurface UVTrimCurve for BEdge->Curve
              if (pBSurface->DropAndTrimCurve(*pEdgeCurve->GetContext(),
                                               pBFace->GetUVDomain(),
                                              *pECopy,
                                               pECopy->GetNaturalInterval(),
                                               dTol/4.0,
                                               dDistToSurface,
                                               dDeviation, sUVCurves, TRUE, TRUE ) != SM_SUCCESS )
                { continue; }
              SmObjsDelete<SmBSplineCurve*> sCleanUV(&sUVCurves);

              // skip curves that did not project or that were too far from the surface
              if (sUVCurves.GetSize() == 0)   { continue; }
              if (dDistToSurface > dTol*10.0) { continue; }

              // expand Tol as needed
              dTol = smos_Max( dTol, dDistToSurface ); // (was m_dThisApproxTol3d, not dTol)

              // for every pBSurface UVTrimCurve
              //  (more than 1 if - 3dCurve wanders in and out of Surface boundaries,
              //                  - crosses a closed surface seam or
              //                  - projects to a closed surface seam)
              for (kk=0; kk<sUVCurves.GetSize(); kk++)
                {
                  SmBSplineCurve * pUVCurve = sUVCurves[kk];
                  SmExtent1d       sIvl     = pUVCurve->GetNaturalInterval();
                  SmCurve        * pECopy2;

                  // get temporary curve copy trimmmed to UVTrimCurve extents
                  SER(pEdgeCurve->Copy(*pEdgeCurve->GetContext(),pECopy2)); NER(pECopy2);
                  SmObjDelete sCleanCopyE4(pECopy2);
                  SER(pECopy2->Trim(sIvl));   // may snap sIvl by tol to existing knots

                  // merge the Projected Curve into both surfaces with tol set large enough
                  // to account for the project distance
                  SER(MergeCurveOnSurfaces(pBFace->GetSurface(),
                                           pOFace->GetSurface(),
                                           dTol,     pECopy2,
                                           pUVCurve, NULL,     // in : UVTrimCurves for pECopy2
                                           NULL,     pOEdge,   // in : edges already mapped to Curve
                                           bTopologyChanged)); // out: edges and vertices deleted by squeezing

                  // if topology has changed - refresh the pOFace ptr
                  if ( bTopologyChanged )
                    {
                      pOFace = (SmFace*)pOSurface->GetFace();
                    }

                  // if there is a face - note if its edge count has changed
                  if (pOFace != NULL)
                    {
                      pOFace->GetEdges(sEdgesO);
                      if (sEdgesO.GetSize() != lOEdgesCount)
                        {
                          bTopologyChanged = TRUE;
                        }
                    }

#ifdef SM_VALIDATE_TOPOLOGY
                  if(m_pBrep)  m_pBrep->ValidatePointers();
                  if(m_pOther) m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY
                  // if topology has changed - break
                  if ( bTopologyChanged ) break;

                } // end iter every UVTrimCurve segment
            } // end general surface (not planar) branch

          // quit edge-loop with topology change -- restart big loop sequence
          if ( bTopologyChanged ) break;

        } // end iter every OFace->Edge
    } // end While ( bTopologyChanged )

  // arrive here after all edges on faces within the surf/surf bounding box intersection
  // have been made compatible i.e. every BFace->Edge is coincident to a OFace->Edge and vice versa.

  // Now Look for and save coincident faces in the output arrays - those get related by the calling method

  // refresh face pointers
  pBFace = (SmFace*)pBSurface->GetFace();
  pOFace = (SmFace*)pOSurface->GetFace();

  // locals
  SmEdgeuse * pEUData[64];
  SmLoopuse * pLUData[16];
  SmTArray<SmEdgeuse*> sEdgeuses(64,pEUData);
  SmTArray<SmLoopuse*> sLoopuses(16,pLUData);

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      Dump() ;
      SmBrep *pBrep1 = pBFace->GetBrep() ;
      SmBrep *pBrep2 = pOFace->GetBrep() ;

      // draw pBSurf(blue), pOSurface(green), and their faces(black)
      smgfx_Erase();
      smgfx_SetLook(2,3, 1,0,1); Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1); if(pBrep1) pBrep1->Draw(TRUE) ; sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0); if(pBrep2) pBrep2->Draw(TRUE) ; sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,1); pBFace->GetSurface()->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,0,0); pBFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 1,1,0); pOFace->GetSurface()->DrawUV(); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,0,0); pOFace->Draw(SM_DM_CROSSHATCH); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // for every BFace->Loop
  pBFace->GetUpwardLoopuses(sLoopuses);
  ULONG lNumLUs = sLoopuses.GetSize();
  for (ii=0; ii<lNumLUs; ii++)
    {
      SmLoopuse *pLU = sLoopuses[ii];
      if (pLU->IsVertexLoopuse())
        { continue; }

      // for every Loop->Edgeuse until
      // find 1st Edgeuse with matching other Brep Edge that can classify
      pLU->GetEdgeuses(sEdgeuses);
      ULONG lNumEUs = sEdgeuses.GetSize();
      for (jj=0; jj<lNumEUs; jj++)
        {
          SmEdgeuse *pEU = sEdgeuses[jj];

          // get Edgeuse's Edge and its matched Edge partner
          SmEdge *pBEdge = pEU->GetEdge();
          SmEdge *pOEdge = (SmEdge*)GetOtherMate(pBEdge);

          // skip cases where edges don't match (outside of surf/surf boundingBox intersection)
          if (!pOEdge) continue;

          // Classify this edgeuse (face) against the matching other Edge
          SmEdgeuse *pOFoundEU = NULL;
          SmFaceuse *pOFoundFU = NULL;
          if (pOEdge->FindRadialSector(pEU,NULL,SM_OT_SAME,pOFoundEU,pOFoundFU) != SM_SUCCESS)
            {
              continue; // This was too difficult an edge to sector - let it choose
                        // a different one later on - just skip it
            }

          // when the classification found a coincident face to the edgeuse->Face
          // check to see if all their edges match
          if (pOFoundFU)
            {
              // get the coincident matching faces
              SmFace *pOFaceMatch = pOFoundFU->GetFace(); NER(pOFaceMatch);
              SmFace *pBFaceMatch = pEU->GetFace(); NER(pBFaceMatch);

              // get face->edges
              pOFaceMatch->GetEdges(sEdgesO);
              pBFaceMatch->GetEdges(sEdgesB);

              // when edge counts are the same
              if (sEdgesO.GetSize() == sEdgesB.GetSize())
                {
                  // note if any edges don't have a match
                  SmBoolean bAllEdgesMatch = TRUE;
                  ULONG lNumEdgesB = sEdgesB.GetSize();
                  for (kk=0; kk<lNumEdgesB; kk++)
                    {
                      SmEdge *pEdgeO = (SmEdge*)GetOtherMate( sEdgesB[kk] );
                      if (!pEdgeO)
                        {
                          bAllEdgesMatch = FALSE;
                          break;
                        }
                      ULONG lFoundIndex;
                      if (!sEdgesO.FindElement(pEdgeO,lFoundIndex))
                        {
                          bAllEdgesMatch = FALSE;
                          break;
                        }
                    } // end iter every pBFace->Edge

                  // when all edges match - add the face pair to the coincident face list
                  if (bAllEdgesMatch)
                    {
                      rFaces.AddUnique(pBFaceMatch);
                      rOFaces.AddUnique(pOFaceMatch);
                    } // end all face->Edges match check
                } // end matching edge count check
            } // end found a coincident face check

          // done with Edgeuse search - its face is either coincident or not
          break;
        } // end iter ever Loop->Edgeuse
    } // end iter every pFace->Loop

  // all done
  return SM_SUCCESS;

} // end SmTopologyIntersector::MergeCoincidentSurfaces

/*******************************************************************//**
PURPOSE: Intersect, Insert and Relate two surfaces from the corresponding breps.

NOTES:
***********************************************************************/
SmStatus SmTopologyIntersector::IIRSurfs
  (SmSurface *pSurface,                    // in : target surface
   SmSurface *pOtherSurface,               // in : other target surface to be intersected
   const SmExtent3d * cpIntersectionBBox)  // in : a limiting region of interest
{
  // no work - otherSurface is not in specified subset of surfaces
  if(   m_pSubset
     && m_pSubset->At(pOtherSurface) == 0)
    {
      return SM_SUCCESS;
    }

  // locals - surface/surface intersection input/output arguments
  SmCurve         * sData1[64];
  SmCurve         * sData2[64];
  SmCurve         * sData3[64];
  double            sData4[64];
  SmTsectCurveType  sData5[64];
  SmTArray<SmCurve*>         s3DCurves  (64,sData1);
  SmTArray<SmCurve*>         sUVCurves1 (64,sData2);
  SmTArray<SmCurve*>         sUVCurves2 (64,sData3);
  SmTArray<double>           sDeviations(64,sData4);
  SmTArray<SmTsectCurveType> sCurveTypes(64,sData5);
  SmObjsDelete<SmCurve*> sClean1(&s3DCurves);
  SmObjsDelete<SmCurve*> sClean2(&sUVCurves1);
  SmObjsDelete<SmCurve*> sClean3(&sUVCurves2);

  // start surf/surf xsects at natural_boundary/otherSurface xsect points
  SmBoolean bUseSurfaceEdges[2];
  bUseSurfaceEdges[0] = TRUE;
  bUseSurfaceEdges[1] = TRUE;

  // Get face or composite face from surface
  SmFace *pThisFace  = (SmFace*)pSurface->GetFace();      NER(pThisFace);
  SmFace *pOtherFace = (SmFace*)pOtherSurface->GetFace(); NER(pOtherFace);

#ifdef SM_DEBUG_CODE // GWC_BOOLEAN_SETBREAK
SmBoolean bDebugMe = FALSE;
static ULONG lCount = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
static ULONG lDebugCount2 = 0 ;
  // draw relatedTopology(magenta), Breps(blue,green), Surfs(cyan,yellow)
  //      Faces(black), xSectBox(grey), xSectCurves(red)
  if(bDebugMe || lDebugCount == lCount)
    {
      // dump SmTopolgyIntersector and Breps
      Dump() ;
      m_pBrep->Dump() ;
      m_pOther->Dump() ;

      // draw RelatedTopo, BBox, Breps, surfs, faces and xSectCurve
      smgfx_Erase() ;
      smgfx_SetLook(2,3, 1,0,1) ;    Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
      if ( cpIntersectionBBox ) {
          smgfx_SetLook(2,3, .5,.5,.5) ; cpIntersectionBBox->Draw(pThisFace->GetContext()) ; sm_GraphicsLoop();
      }
      smgfx_SetLook(1,2, 0,0,1) ;    m_pBrep->Draw(TRUE) ;  sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ;    m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,1) ;    pSurface->DrawUV(4, 4) ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,1,0) ;    pOtherSurface->DrawUV(4, 4) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ;    pThisFace->Draw(SM_DM_CROSSHATCH, 6, 6) ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,0) ;    pOtherFace->Draw(SM_DM_CROSSHATCH, 6, 6) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // Compute tolerance as max(sum of two face tolerances, approxTol)

  // GWC_NEEDS_WORK StopMixingApproxAndTopoTolerancesOnNextLine GWC_LINE ;
  double dTol = smos_Max(pThisFace->GetTolerance() + pOtherFace->GetTolerance(),
                         m_dThisApproxTol3d);
//cbiTol: dTol = TOL( pThisFace, pOtherFace, this );
  // get intersection bounding box
  SmExtent3d sIntersectionBBox;
  if (!cpIntersectionBBox)
    {
      // when not given - start with Face bounding boxes
      SmExtent3d sThisFaceBBox, sOtherFaceBBox;
      SER(pThisFace->CalculateBoundingBox(sThisFaceBBox));
      SER(pOtherFace->CalculateBoundingBox(sOtherFaceBBox));
      
      // Expand one BBox by dSXectTol3d to make sure all possible intersections are found
      sThisFaceBBox.ExpandAbsolute(dTol);

      // no work - bounding boxes don't intersect
      if (sThisFaceBBox.AreDisjoint(sOtherFaceBBox))
        { return SM_SUCCESS; }

      // let xsect box = xsect of surface bounding boxes
      SER(sThisFaceBBox.Intersect(sOtherFaceBBox,sIntersectionBBox));
    } // end no given IntersectionBox branch

  else // intersection box is given - use it
    {
      sIntersectionBBox = *cpIntersectionBBox;
    } // end given intersectionBox branch

  // Intersect surfaces

  // see if this surface/surface xsect has been saved
  SmBoolean bHaveSSI;
  SER(CheckIfHaveExistingSSI(*pSurface->GetContext(),pSurface,pOtherSurface,
                              bHaveSSI,s3DCurves,sUVCurves1,sUVCurves2,
                              sCurveTypes,sDeviations));

  // when surface/surface xsect is unavailable - get coincidence or intersection
  SmBoolean bAreCoincident = FALSE;
  if (!bHaveSSI)
    {
      // check for coincident surfaces - partial coincidence is ok
      //    because upcoming MergeCoincidentSurfaces() accounts for that
      double dCoincidentDistance = 0.0;
      SmSurface *pContainingSurface = NULL ;
      SER(pSurface->CoincidenceCheck
           (*pOtherSurface,               // in : target surface to compare
             m_dThisApproxTol3d,                        // in : max allowed distance sample points on coincident surfaces
             bAreCoincident,              // out: TRUE = surfaces are the same to within tolerance
             dCoincidentDistance,         // out: max distance between planes
             FALSE,                       // in : TRUE = Test for complete coincidence.
                                          //      FALSE= Test for any coincident overlap.
             pContainingSurface)) ;       // out: (Not used.)

      // for surfaces that are not coincident
      if (!bAreCoincident)
        {
          // Intersect them.
          // ... This will enable stale caches to be recreated: // [B352]
          SmTemporaryChangeValue<SmBoolean> sTemp( m_pBrep->m_bEditingEnabled, FALSE );

#ifndef SM_VALIDATE_INTERSECTORS
          if (pSurface->GlobalSurfaceIntersect
#else // no SM_VALIDATE_INTERSECTORS
          if (pSurface->GlobalSurfaceIntersectAndValidate
#endif // no SM_VALIDATE_INTERSECTORS
                (*pSurface->GetContext(),       // in : Context for new object construction
                 pThisFace->GetUVDomain(),      // in : this Surface intersection limits
                 *pOtherSurface,                // in : target 2nd intersecting surface
                 pOtherFace->GetUVDomain(),     // in : 2nd Surface intersection limits
                 bUseSurfaceEdges,              // in : array[2], TRUE = Find xSect curve start points by XSecting
                                                //                       the edges of one surface with the other.
                 SM_CAST_APPROXTOL3D_PTR(&m_dThisApproxTol3d),  // in : If not given it uses 1/1000 of surface size as approximation tolerance
                 &m_dThisAngTolRad,             // in : If not given it uses 30 degrees
                 &s3DCurves,                    // out: 3D curves produced by intersection
                 &sUVCurves1,                   // out: UV curves on this surface produced by intersection
                 &sUVCurves2,                   // out: UV curves on crOtherSurface produced by intersection
                 &sCurveTypes,                  // out: oneof: SM_TC_TOUCHING   - single point intersection (surf norms parallel)
                                                //             SM_TC_CROSSING   - curve intersection (surf norms not parallel)
                                                //             SM_TC_TANGENT    - curve intersection (surf norms parallel)
                                                //             SM_TC_COINCIDENT - curve intersection (surf norms parallel & cross-tangents equal)
                                                //             SM_TC_NEAR_TANGENT    - curve has small angle of intersection
                                                //             SM_TC_REGION_BOUNDARY - curve bounds region within which the surfs are coincident
                 &sDeviations) != SM_SUCCESS)   // out: produced Curve Deviations from each UVCurve to true xSect, NULL to ignore
            {
              SM_ASSERT_ERR ;
              SM_DBG_WARN(_T("Surface/Surface Intersection Error - Proceeding\n"));

            } // end failed Surf/Surf xsect check


#ifdef SM_DEBUG_CODE // GWC_BOOLEAN_SETBREAK
            // draw relatedTopology(magenta), Breps(blue,green), Surfs(cyan,yellow)
            //      Faces(black), xSectBox(grey), xSectCurves(red)
            if(bDebugMe || lDebugCount == lCount || lDebugCount2 == lCount)
              {
                // dump SmTopolgyIntersector and Breps
                Dump() ;
                m_pBrep->Dump() ;
                m_pOther->Dump() ;
                SM_ASSERT_VALID(m_pBrep) ;
                SM_ASSERT_VALID(m_pOther) ;
                SM_ASSERT_VALID(pSurface) ;
                SM_ASSERT_VALID(pOtherSurface) ;

                // draw RelatedTopo, BBox, Breps, surfs, faces and xSectCurve
                smgfx_Erase() ;
                ULONG di;
                smgfx_SetLook(4,6, 1,0,0) ; for(di=0;di<s3DCurves.GetSize();di++)
                                              { SmCurve *pCrv = s3DCurves[di];
                                                pCrv->Dump();
                                                SM_ASSERT_VALID(pCrv) ;
                                                pCrv->Draw(); sm_GraphicsLoop() ;
                                              }
                smgfx_SetLook(2,3, 1,0,1) ;    Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
                smgfx_SetLook(2,3, .5,.5,.5) ; cpIntersectionBBox->Draw(m_pBrep->GetContext()) ; sm_GraphicsLoop();
                smgfx_SetLook(1,2, 0,0,1) ;    m_pBrep->Draw(TRUE) ;  sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 0,1,0) ;    m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 0,1,1) ;    pSurface->DrawUV(4, 4) ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 1,1,0) ;    pOtherSurface->DrawUV(4, 4) ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 0,0,0) ;    pThisFace->Draw(SM_DM_CROSSHATCH, 6, 6) ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 0,0,0) ;    pOtherFace->Draw(SM_DM_CROSSHATCH, 6, 6) ; sm_GraphicsLoop() ;
                sm_GraphicsLoop() ;
              }
#endif // SM_DEBUG_CODE
        } // end no coincidence check
    } // end no PreExisting surf/surf xsect solution check

  // count the number of xsect generated coincident curves
  ULONG ii, lCoinEdgeCount = 0;
  for ( ii=0; ii<s3DCurves.GetSize(); ii++ )
    {
      if (   sCurveTypes[ii] == SM_TC_COINCIDENT
          || sCurveTypes[ii] == SM_TC_REGION_BOUNDARY )
        {
          lCoinEdgeCount ++;
        }
    }

  // merge coincident surfaces if surfaces or any xsect edges were coincident
  if (bAreCoincident || lCoinEdgeCount > 1)
    {
      SmFace * pFData[64];
      SmFace * pFData2[64];
      SmTArray<SmFace*> sFaces(64,pFData);
      SmTArray<SmFace*> sOFaces(64,pFData2);
      SmFace *pFace  = (SmFace*)pSurface->GetFace(); NER(pFace);
      SmFace *pFaceOther = (SmFace*)pOtherSurface->GetFace(); NER( pFaceOther );

      // merge the coincident surface->faces
      //   make sure all edges of the coincident surface pair faces are also coincident
      //   and mated by projecting and merging the edges of each surface->Face onto the other.
      SER(MergeCoincidentSurfaces(pSurface,pOtherSurface,sFaces,sOFaces));

      // record coincident faces as mated topology objects
      SM_ASSERT(sFaces.GetSize() == sOFaces.GetSize()) ;
      for( ii=0; ii<sFaces.GetSize(); ii++)
        {
          Relate( sFaces[ii], sOFaces[ii] );
        }

      if(0 == s3DCurves.GetSize())
        { return(SM_SUCCESS) ; }

    } // end coincident check

  // for 2 passes - merge, first the non-coincident edges followed by coincident edges into breps
  ULONG mmm;
  for (mmm=0; mmm<2; mmm++)
    {
      // for every 3D xsect curve
      for (ii=0; ii<s3DCurves.GetSize(); ii++)
        {
          // pass 1: skip    coincident xsect edges
          if ( mmm == 0
               && (   sCurveTypes[ii] == SM_TC_COINCIDENT
                   || sCurveTypes[ii] == SM_TC_REGION_BOUNDARY
             )    )
               { continue; }

          // pass 2: process coincident xsect edges
          if ( mmm == 1
               && (   sCurveTypes[ii] != SM_TC_COINCIDENT
                   && sCurveTypes[ii] != SM_TC_REGION_BOUNDARY
             )    )
              { continue; }


          // get this curve and its bounding box
          SmCurve *pCrv = s3DCurves[ii];
          SmExtent3d sCrvBBox;
          SER(pCrv->CalculateBoundingBox(pCrv->GetNaturalInterval(),
                                         &sCrvBBox));

          // GWC_NEEDS_WORK StopMixingApproxAndTopoTolerancesOnNextLine2 GWC_LINE ;
          sCrvBBox.ExpandAbsolute(m_dThisApproxTol3d);

          // no work - XSectCurve BBox does not intersect the FaceBBox/FaceBBox Intersection BBox
          if (sCrvBBox.AreDisjoint(sIntersectionBBox))
            { continue; }

          // note: used to skip near tangent curves - currently processes them
          // if (sCurveTypes[ii] == SM_TC_NEAR_TANGENT)
          //   {
          //     continue;
          //   }

          // Do not skip degenerate curves.  They can arise from a corner of one
          // face just touching an edge of the other, with topology there (a vertex
          // and an edge).  That can cause a vertex in the resulting Brep to be
          // sitting in the middle of an edge, without the edge being split, which
          // is incorrect coincident topology.  Moreover, in the nonmanifold case of
          // two original Breps just touching at a vertex, the shells won't get merged.
          // [mouse 5; box_cyl_grid iter 7; box_cone_grid iter 33]
          //// skip short xsect curves 1. low knot count   (less than or equal to 8)
          ////                         2. length less than (   ApproximationTolerance
          ////                                              or SM_EFF_ZERO
          ////                                              or 100 * curve deviation)
          //if (pCrv->GetNumberNaturalKnots() <= 8)
          //  {
          //
          //    // estimate curve's length
          //    double dLeng = pCrv->ApproximateLength(pCrv->GetNaturalInterval(),5);
          //
          //    // Test for short edge cases
          //    if (dLeng < 100 * m_dThisApproxTol3d)
          //      {
          //        double dDev = sDeviations[ii];
          //
          //        // when length is short or about the same size as the deviation
          //        if(   dLeng < SM_EFF_ZERO
          //           || dLeng < dDev * 100.0
          //           || dLeng < m_dThisApproxTol3d)
          //          {
          //            // skip the short edge
          //            //  WARN(_T("Removing a small intersection curve"));
          //            continue;
          //          } // end skip edge check
          //      } // end short length check
          //  } // end suspiciously low knot-count edge check

#ifdef SM_DEBUG_CODE // GWC_BOOLEAN_SETBREAK
            // draw relatedTopology(magenta), Breps(blue,green), Surfs(cyan,yellow)
            //      Faces(black), xSectBox(grey), xSectCurve(red)
             lCount      = 1 ; lCount++ ;
             lDebugCount = 0 ;
             lDebugCount2= 0 ;
            if(bDebugMe || lDebugCount == lCount || lDebugCount2 == lCount)
              {
                // dump SmTopolgyIntersector and Breps
                Dump() ;
                m_pBrep->Dump() ;
                m_pOther->Dump() ;
                pCrv->Dump() ;

                // draw RelatedTopo, BBox, Breps, surfs, faces and xSectCurve
                smgfx_Erase() ;
                smgfx_SetLook(2,3, 1,0,1) ;    Draw(3,5, 7,9) ; sm_GraphicsLoop() ;
                smgfx_SetLook(2,3, .5,.5,.5) ; if(cpIntersectionBBox) { cpIntersectionBBox->Draw(m_pBrep->GetContext()) ; } sm_GraphicsLoop();
                smgfx_SetLook(1,2, 0,0,1) ;    m_pBrep->Draw(TRUE) ;  sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 0,1,0) ;    m_pOther->Draw(TRUE) ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 0,1,1) ;    pSurface->DrawUV(4, 4) ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 1,1,0) ;    pOtherSurface->DrawUV(4, 4) ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 0,0,0) ;    pThisFace->Draw(SM_DM_CROSSHATCH, 6, 6) ; sm_GraphicsLoop() ;
                smgfx_SetLook(1,2, 0,0,0) ;    pOtherFace->Draw(SM_DM_CROSSHATCH, 6, 6) ; sm_GraphicsLoop() ;
                smgfx_SetLook(5,7, 1,0,0) ;    pCrv->Draw() ; sm_GraphicsLoop() ;   // <== Curve being merged
                sm_GraphicsLoop() ;
              }
#endif // SM_DEBUG_CODE

          // merge the 3d curve into the surface topology structures
          SmBoolean bDeletedTopology;
          SER(MergeCurveOnSurfaces(pSurface,                    // in : 1st surface of surface/surface intersection
                                   pOtherSurface,               // in : 2nd surface of surface/surface intersection
                                   m_dThisApproxTol3d,          // in : min length for non-degenerate curve, degen Curve treated as pt
                                   s3DCurves[ii],               // in : surf/surf 3D xsect curve
                                   sUVCurves1[ii],              // in : opt associated UV curve for 1st surface
                                   sUVCurves2[ii],              // in : opt associated UV curve for 2nd surface
                                   NULL,                        // in : opt already existing edge corresponding to 3DCurve on pSurface, NULL to ignore
                                   NULL,                        // in : opt already existing edge which corresponding to 3DCurve on pOtherSurface,     
                                   bDeletedTopology));          // out: TRUE = multiply mated Vertices or Edges squeezed in MergeCurveClasses().  
                                                                //      FALSE= no Vertices or Edges squeezed
          // remember when MergeCurveOnSurfaces deleted topology
          if ( bDeletedTopology )
              m_bTopologyDeleted = TRUE;

        } // end iter every 3D xsect curve
    } // end iter 2 passes - pass 1 = skip coincident curve, pass2 - hit only coincident curves

#ifdef SM_VALIDATE_TOPOLOGY
  if(m_pBrep)  m_pBrep->ValidatePointers();
  if(m_pOther) m_pOther->ValidatePointers();
#endif // SM_VALIDATE_TOPOLOGY

  return(SM_SUCCESS) ;

} // end SmTopologyIntersector::IIRSurfs

/*******************************************************************//**
PURPOSE: Relate two topology objects in the m_pBToO and m_pOToB lists
  after setting vertex or edge tolerances equal.

NOTES:
  1. Check for errors:
       Either object already has a mate not equal to input partner.
  2. For Vertices and Edges,
       Set tolerance = Max(pBrepTopo->GetTolerance(),pOtherTopo->GetTolerance())
  3. Add mate relationship to m_pBToO and m_pOToB lists.
***********************************************************************/
SmStatus SmTopologyIntersector::Relate
  (SmObject *pBrepTopo,          // in : Object in ThisBrep to relate
   SmObject *pOtherTopo)         // in : Object in OtherBrep to relate
{
  // check for conflicting existing mate relationships
  SmObject *pOth = GetOtherMate(pBrepTopo);
  SmObject *pBrp = GetBrepMate(pOtherTopo);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
  if ( bDebugMe )
  {
    if (pBrepTopo->IsKindOf(SmFace_TYPE))
      {
        SmFace *pFace1 = (SmFace*)pBrepTopo;
        SmFace *pFace2 = (SmFace*)pOtherTopo;
        SmFace *pOthFace = (SmFace*)(pOth ? pOth : NULL) ;
        SmFace *pBrpFace = (SmFace*)(pBrp ? pBrp : NULL) ;
        SmBrep *pBrep1 = ( pFace1 == NULL ) ? NULL : pFace1->GetBrep();
        SmBrep *pBrep2 = ( pFace2 == NULL ) ? NULL : pFace2->GetBrep();

        smgfx_Erase();
        smgfx_SetLook(1,2, 0,0,1); if(pBrep1) pBrep1->Draw(TRUE); sm_GraphicsLoop();
        smgfx_SetLook(1,2, 0,1,0); if(pBrep2) pBrep2->Draw(TRUE); sm_GraphicsLoop();
        smgfx_SetLook(2,2, 0,0,0); if(pFace1) pFace1->Draw(SM_DM_CROSSHATCH, 3, 3); sm_GraphicsLoop();
        smgfx_SetLook(4,2, 1,0,0); if(pFace2) pFace2->Draw(SM_DM_CROSSHATCH, 4, 4); sm_GraphicsLoop();
        smgfx_SetLook(5,2, 0,1,1); if(pOthFace) pOthFace->Draw(SM_DM_CROSSHATCH, 5, 5); sm_GraphicsLoop();
        smgfx_SetLook(6,2, 1,0,1); if(pBrpFace) pBrpFace->Draw(SM_DM_CROSSHATCH, 7, 7); sm_GraphicsLoop();
        sm_GraphicsLoop();
      }
    else if(pBrepTopo->IsKindOf(SmEdge_TYPE))
      {
        SmEdge *pEdge1 = (SmEdge*)pBrepTopo;
        SmEdge *pEdge2 = (SmEdge*)pOtherTopo;
        SmEdge *pOthEdge = (SmEdge*)(pOth ? pOth : NULL) ;
        SmEdge *pBrpEdge = (SmEdge*)(pBrp ? pBrp : NULL) ;
        SmBrep *pBrep1 = ( pEdge1 == NULL ) ? NULL : pEdge1->GetBrep();
        SmBrep *pBrep2 = ( pEdge2 == NULL ) ? NULL : pEdge2->GetBrep();

        smgfx_Erase();
        smgfx_SetLook(1,2, 0,0,1); if(pBrep1) pBrep1->Draw(TRUE); sm_GraphicsLoop();
        smgfx_SetLook(1,2, 0,1,0); if(pBrep2) pBrep2->Draw(TRUE); sm_GraphicsLoop();
        smgfx_SetLook(2,2, 0,0,0); if(pEdge1) pEdge1->Draw(); sm_GraphicsLoop();
        smgfx_SetLook(4,2, 1,0,0); if(pEdge2) pEdge2->Draw(); sm_GraphicsLoop();
        smgfx_SetLook(5,2, 0,1,1); if(pOthEdge) pOthEdge->Draw(); sm_GraphicsLoop();
        smgfx_SetLook(6,2, 1,0,1); if(pBrpEdge) pBrpEdge->Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
      }
  }
#endif // SM_DEBUG_CODE

  if (pOth && (pOth != pOtherTopo))
    { SER(SM_ERR); }
  if (pBrp && (pBrp != pBrepTopo ))
    { SER(SM_ERR); }

  // when objects are vertices - set object tolerances to max(object1 tol, object2 tol)
  if (pBrepTopo->IsKindOf(SmVertex_TYPE))
    {
      SmVertex *pV1 = SM_CAST_PTR(SmVertex,pBrepTopo); NER(pV1);
      SmVertex *pV2 = SM_CAST_PTR(SmVertex,pOtherTopo); NER(pV2);
#ifdef SM_USE_OLDTOL      
      SM_OLDTOL_LINE if (pV1->GetTolerance() < pV2->GetTolerance()) { pV1->SetTolerance(pV2->GetTolerance());}
      SM_OLDTOL_LINE if (pV2->GetTolerance() < pV1->GetTolerance()) { pV2->SetTolerance(pV1->GetTolerance());}
#endif // SM_USE_OLDTOL
    }

  // when objects are edges - set object tolerances to max(object1 tol, object2 tol)
  if (pBrepTopo->IsKindOf(SmEdge_TYPE))
    {
      SmEdge *pE1 = SM_CAST_PTR(SmEdge,pBrepTopo); NER(pE1);
      SmEdge *pE2 = SM_CAST_PTR(SmEdge,pOtherTopo); NER(pE2);
#ifdef SM_USE_OLDTOL      
      SM_OLDTOL_LINE if (pE1->GetTolerance() < pE2->GetTolerance()) { pE1->SetTolerance(pE2->GetTolerance()); }
      SM_OLDTOL_LINE if (pE2->GetTolerance() < pE1->GetTolerance()) { pE2->SetTolerance(pE1->GetTolerance()); }
#endif // SM_USE_OLDTOL

    }

  // add the pair to the Ptr to Ptr map
  m_pBToO->Insert(pBrepTopo,pOtherTopo);
  m_pOToB->Insert(pOtherTopo,pBrepTopo);
  return SM_SUCCESS;

} // end SmTopologyIntersector::Relate

/*******************************************************************//**
PURPOSE: Check midPoint and EndPoint consistency
   in homogenized CurveClassifications that classify to
   mated edge and vertex pairs.

NOTES:
   1. Corresponding point classifications that point to
      mated VERTICES should point to vertices mated to one another.

   2. Corresponding point classifications that point to
      mated EDGES should point to edges mated to one another.

When violations of the above rules are encountered, the case
  is to be studied, and if a pattern is discerned that is
  caused by desiciions made throughout the code, then edit
  the classifications here.

  CASE 1: When classifying two coincident faces eventually an edge
    of one face that was coincident with the other face was
    classified in preparation for insertion.  Due to random ordering
    the end vertices of the edge happened to already be inserted
    into the face and matched.  The classification of the edge's curve was
    corrupted due to small gap errors larger than tol. The
    classification violated the matching vertex rule because
    the matching vertex pair was split by a small interval as:

    SMTOPOLOGYINTERSECTOR ENTITY MAP, # ENTITY MAPS = 6
         Vertex : [0x05786A68] = 0x0567FDB8, Vert/Vert Dist = 0.0001222302924662

       THIS CLASIFICATION                              OTHER CLASSIFIATION
   [0] - 3dCurve Interval [0.0000000000000000,         [0] - 3dCurve Interval [0.0000000000000000,
                           0.0000852918015221]                                 0.0000852918015221]
     On Vertex - 0x05786A68,  <-+                        Unknown Point Classification
     On Face   - 0x0391EEE8,    |                        Unknown Point Classification
     On Face   - 0x0391EEE8,    +-matched-vertex-pair->  On Vertex - 0x0567FDB8,

   [1] - 3dCurve Interval [0.0000852918015221,         [1] - 3dCurve Interval [0.0000852918015221,
                           72.0620695551990170]                               72.0620695551990170]
     On Face   - 0x0391EEE8,                             On Vertex - 0x0567FDB8,
     On Face   - 0x0391EEE8,                             On Edge   - 0x056800F8,
     On Face   - 0x0391EEE8,                             On Edge   - 0x056800F8,

  CASE 2: Check consistency of CurveIntervals classified to edge mate pairs.
      Signal warning when edge mates are not each other.

      When this condition is violated - the classification of each
       midPoint is reset to SM_PC_UNKNOWN

***********************************************************************/
SmStatus SmTopologyIntersector::FixupClassMates
  (SmCurveClassification & rCurveClassBrep,  // in : first member of a pair of homogonized curveClassifications
   SmCurveClassification & rCurveClassOther) // in : the other member of the pair
{
  // locals
  const SmCurve *cpClassificationCurve = rCurveClassBrep.GetCurve() ;
  ULONG          lCurveBrepDim         = cpClassificationCurve->GetDim() ;

  // check state - both classification should be for the same curve
  SM_ASSERT(cpClassificationCurve == rCurveClassOther.GetCurve()) ;

  // no work - classifications are not for the same curve
  if(cpClassificationCurve != rCurveClassOther.GetCurve())
    {
      // no work to do
      return(SM_SUCCESS) ;
    }

  // error condition - curveClassifications not homogenized
  if (rCurveClassBrep.GetSize() != rCurveClassOther.GetSize())
    {
      SER(SM_ERR);  // Probably not homogenized
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount)
    {
      this->Dump() ;           // show matched entity mapping
      rCurveClassBrep.Dump();
      rCurveClassOther.Dump();

      SmBrep *pBrep1 = rCurveClassBrep.GetBrep() ;
      SmBrep *pBrep2 = rCurveClassOther.GetBrep() ;

      smgfx_Erase() ;
      smgfx_SetLook(3,5, 1,0,0) ; cpClassificationCurve->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetCurveClassificationObjectLook(5,7,
                                             0, 0,1,
                                             0,.5,1,
                                             .5,0,1) ; rCurveClassBrep.Draw(); sm_GraphicsLoop();
      smgfx_SetCurveClassificationObjectLook(7,9,
                                             0, 1,0,
                                             0,1,.5,
                                             .5,1,0) ; rCurveClassOther.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // for every interval
  long ii;
  for (ii=0; (ULONG)ii<rCurveClassBrep.GetSize(); ii++)
    {
      // locals
      SmCurveInterval & rIvlB = rCurveClassBrep [(ULONG)ii];
      SmCurveInterval & rIvlO = rCurveClassOther[(ULONG)ii];
      SmExtent1d        sIvl  = rIvlB.GetInterval() ;

      // check for short intervals that split matched vertices
      // skip closed intervals - both ends will often map to mated vertex pairs
      if(!cpClassificationCurve->IsClosed(sIvl))
        {
          if(   (   rIvlB.m_vStart.GetPointClass() == SM_PC_VERTEX
                 && rIvlO.m_vEnd.GetPointClass()   == SM_PC_VERTEX
                 && IsMatedPair(rIvlB.m_vStart.GetObject(), rIvlO.m_vEnd.GetObject()))
             || (   rIvlB.m_vEnd.GetPointClass() == SM_PC_VERTEX
                 && rIvlO.m_vStart.GetPointClass()   == SM_PC_VERTEX
                 && IsMatedPair(rIvlB.m_vEnd.GetObject(), rIvlO.m_vStart.GetObject())))
            {
              SmBoolean bChangedClassifications = FALSE ;
              SER(FixupSplitVertexPair(ii, rCurveClassBrep, rCurveClassOther, bChangedClassifications)) ;
              if( bChangedClassifications)
                {
                  // Start the iteration over
                  ii = -1 ;
                  continue ;
                }
            }
        }

      // check all start points that classify to something
      if(   rIvlB.m_vStart.GetPointClass() != SM_PC_UNKNOWN
         && rIvlO.m_vStart.GetPointClass() != SM_PC_UNKNOWN)
        {
          SER(FixupEndPointClassMates(lCurveBrepDim, rIvlB.m_vStart, rIvlO.m_vStart, cpClassificationCurve));
        }

      // check all mid points that classify to something
      if(   rIvlB.m_vMid.GetPointClass() != SM_PC_UNKNOWN
         && rIvlO.m_vMid.GetPointClass() != SM_PC_UNKNOWN)
        {
          SER(FixupMidPointClassMates(lCurveBrepDim, rIvlB.m_vMid, rIvlO.m_vMid, cpClassificationCurve));
        }

      // check last end points that classify to something
      if(   ((ULONG)ii == rCurveClassBrep.GetSize() - 1)
         && rIvlB.m_vEnd.GetPointClass() != SM_PC_UNKNOWN
         && rIvlO.m_vEnd.GetPointClass() != SM_PC_UNKNOWN)
        {
          SER(FixupEndPointClassMates(lCurveBrepDim, rIvlB.m_vEnd, rIvlO.m_vEnd, cpClassificationCurve));
        }

    } // end iter every interval

#ifdef SM_DEBUG_CODE
  if (bDebugMe)
    {
      rCurveClassBrep.Dump();
      rCurveClassOther.Dump();

      SmBrep        *pBrep1 = rCurveClassBrep.GetBrep() ;
      SmBrep        *pBrep2 = rCurveClassOther.GetBrep() ;
      const SmCurve *pCurve = rCurveClassBrep.GetCurve() ;

      smgfx_Erase();
      smgfx_SetLook(3,2, 1,0,0) ; pCurve->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(1,3, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,3, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(2,6, 0,1,1) ; rCurveClassBrep.Draw(); sm_GraphicsLoop();
      smgfx_SetLook(3,8, 1,0,1) ; rCurveClassOther.Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE
  return SM_SUCCESS;

} // end SmTopologyIntersector::FixupClassMates

/*******************************************************************//**
PURPOSE: Check VERTEX classified EndPoint Classifications for consistency.
     When PointClassifications map to mated pairs check that the mates
     are the same SmObjects as the classification objects.

NOTES:
   1. Interval Ends can classify to different Edges: they hit the Edge at
      only one point, not a coincident segment.  [B279]

   1a. It is legal for one to be classified to a Vertex and the other
       to an Edge, for example.  Therefore, we check only if they are
       both classified to Vertices.

   2. Check consistency of PointClassifications classified to vertex mate pairs.
      Signal warning when vertex mates are not each other.

      When this condition is violated - the classification of each
       midPoint is reset to SM_PC_UNKNOWN

***********************************************************************/
SmStatus SmTopologyIntersector::FixupEndPointClassMates
  (ULONG                   lPointDim,             // in : Classification point dim, 2 or 3
   SmPointClassification & rPCBrep,               // in : 1st of pair of matching end point classifications
   SmPointClassification & rPCOther,              // in : the other member of the pair
   const SmCurve         * cpClassificationCurve) // NotUsed: in : pointer to curve being classified -
                                                  //      currently used for error recovery and debugging
{
  SM_REF1(cpClassificationCurve) ; 
  // Point classification types
  SmPointClassificationType eBrepClass  = rPCBrep.GetPointClass() ;
  SmPointClassificationType eOtherClass = rPCOther.GetPointClass() ;

  if ( eBrepClass != SM_PC_VERTEX || eOtherClass != SM_PC_VERTEX )
    { return SM_SUCCESS; }

  // At this point, both classify to vertices.

  // get the ThisBrep Object and its OtherBrep mate
  SmObject     *pBrepObject  = rPCBrep.GetObject() ;
  SmObject     *pBrepMate    = pBrepObject ? GetOtherMate(pBrepObject) : NULL ;

  // get the OtherBrep Object and its ThisBrep mate
  SmObject     *pOtherObject = rPCOther.GetObject() ;
  SmObject     *pOtherMate   = pOtherObject ? GetBrepMate(pOtherObject) : NULL ;

  // when either point classifies to a mated vertex
  if ( pBrepMate || pOtherMate )
    {
      // When both Points don't map to one another's mates.
      // Note, not a problem when the topologies are different,
      // e.g., one classifies to an Edge and the other to a Face. [B279]
      if( (   pOtherMate != pBrepObject
           || pBrepMate  != pOtherObject
          )
        )
        {
          // This is an error condition.  Inform the public.
          SM_DBG_WARN(_T("SmTopologyIntersector::FixupPointClassMates(): End PointClassifications classified to Vertices whose mates are not each other")) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe)
            {
              rPCBrep.Dump(lPointDim);
              rPCOther.Dump(lPointDim);

              SmBrep   *pBrep1       = rPCBrep.GetBrep() ;
              SmBrep   *pBrep2       = rPCOther.GetBrep() ;
              const SmFace *pBrepFace    = rPCBrep.GetFace();
              const SmFace *pOtherFace   = rPCOther.GetFace();
              SmVertex *pVertex1     = (pBrepObject  && pBrepObject->IsKindOf(SmVertex_TYPE))  ? (SmVertex*)pBrepObject  : NULL ;
              SmVertex *pVertex2     = (pOtherObject && pOtherObject->IsKindOf(SmVertex_TYPE)) ? (SmVertex*)pOtherObject : NULL ;
              SmVertex *pVertex1Mate = (pBrepMate    && pBrepMate->IsKindOf(SmVertex_TYPE))    ? (SmVertex*)pBrepMate    : NULL ;
              SmVertex *pVertex2Mate = (pOtherMate   && pOtherMate->IsKindOf(SmVertex_TYPE))   ? (SmVertex*)pOtherMate   : NULL ;
              SmPoint3d sBrepPoint, sOtherPoint ;
              rPCBrep.FindObjPoint3d(sBrepPoint) ;
              rPCOther.FindObjPoint3d(sOtherPoint) ;

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(8,9, 1,0,0) ; sBrepPoint.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(8,9, 1,0,0) ; sOtherPoint.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,0) ; if(pBrepFace) { pBrepFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,0) ; if(pOtherFace) { pOtherFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,0,1) ; if(pVertex1) { pVertex1->Draw() ; } sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,1,0) ; if(pVertex2) { pVertex2->Draw() ; } sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, 1,0,1) ; if(pVertex1Mate) { pVertex1Mate->Draw() ; } sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, 0,1,1) ; if(pVertex2Mate) { pVertex2Mate->Draw() ; } sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, 0,1,1) ; rPCBrep.Draw(); sm_GraphicsLoop();
              smgfx_SetLook(6,7, 1,0,1) ; rPCOther.Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#else
          SM_REF1(lPointDim);
#endif // SM_DEBUG_CODE

        } // end bad mated pair classification check
    } // end either Point classifies to a vertex check

  // all done
  return SM_SUCCESS;

} // end SmTopologyIntersector::FixupEndPointClassMates

/*******************************************************************//**
PURPOSE: Check EDGE and VERTEX classified MidPoint Classifications for consistency.
     When PointClassification map to mated pairs check that the mates
     are the same SmObjects as the classification objects.  If they
     aren't set classification to UNKNOWN.

NOTES:
   1. Check consistency of PointClassifications classified to edge mate pairs.
      Signal warning when edge mates are not each other.

   2. Check consistency of PointClassifications classified to vertex mate pairs.
      Signal warning when vertex mates are not each other.

      When this condition is violated - the classification of each
       midPoint is reset to SM_PC_UNKNOWN

***********************************************************************/
SmStatus SmTopologyIntersector::FixupMidPointClassMates
  (ULONG lPointDim,                      // in : Classification point dim, 2 or 3
   SmPointClassification & rPCBrep,      // in : 1st of pair of matching mid point classifications
   SmPointClassification & rPCOther,     // in : the other member of the pair
   const SmCurve *cpClassificationCurve) // in : pointer to curve being classified -
                                         //      currently used for error recovery and debugging
{
  // Point classification types
  SmPointClassificationType eBrepClass  = rPCBrep.GetPointClass() ;
  SmPointClassificationType eOtherClass = rPCOther.GetPointClass() ;

  // get the ThisBrep Object and its OtherBrep mate
  SmObject     *pBrepObject  = rPCBrep.GetObject() ;
  SmObject     *pBrepMate    = pBrepObject ? GetOtherMate(pBrepObject) : NULL ;

  // get the OtherBrep Object and its ThisBrep mate
  SmObject     *pOtherObject = rPCOther.GetObject() ;
  SmObject     *pOtherMate   = pOtherObject ? GetBrepMate(pOtherObject) : NULL ;

#ifdef SM_DEBUG_CODE
  const SmFace* pBrepFace = rPCBrep.GetFace();
  const SmFace* pOtherFace = rPCOther.GetFace();
#endif

  // when either point classifies to a mated vertex or edge
  if(   (   (eBrepClass  == SM_PC_VERTEX || eBrepClass  == SM_PC_EDGE) && pBrepMate)
     || (   (eOtherClass == SM_PC_VERTEX || eOtherClass == SM_PC_EDGE) && pOtherMate))
    {
      // see if the mate lies on the face of the partner classification
      // remember if the mated objects are connected to the face being classified
      //SmBoolean bSameThisFace  =   pBrepFace && pOtherMate
      //                          && (   ((eOtherClass == SM_PC_EDGE)  && ((SmEdge*)pOtherMate)->IsConnectedToFace(pBrepFace))
      //                              || ((eOtherClass == SM_PC_VERTEX)&& ((SmVertex*)pOtherMate)->IsConnectedToFace(pBrepFace))) ;
      //SmBoolean bSameOtherFace =   pOtherFace && pBrepMate
      //                          && (   ((eBrepClass == SM_PC_EDGE)   && ((SmEdge*)pBrepMate)->IsConnectedToFace(pOtherFace))
      //                              || ((eBrepClass == SM_PC_VERTEX) && ((SmVertex*)pBrepMate)->IsConnectedToFace(pOtherFace))) ;

      // When both Points don't map to one another's mates.
      // Note, not a problem when the topologies are different,
      // e.g., one classifies to an Edge and the other to a Face. [B279]
      if(   (   pOtherMate != pBrepObject
             || pBrepMate  != pOtherObject
            )
         && ( eBrepClass == eOtherClass )
        )
        {
          // This is an error condition.
          // Inform the public and possibly remove the midPoint classifications.
          SM_DBG_WARN(_T("SmTopologyIntersector::FixupMidPointClassMates(): Mid PointClassifications classified to objects whose mates are not each other")) ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe)
            {
              // When class objects are edges, let's examine the mismatched curve intersections a bit
              if(eBrepClass == SM_PC_EDGE)
                {
                  SmEdge  *pBrepEdge   = ((SmEdge *)pBrepObject) ;
                  SmEdge  *pOtherEdge  = ((SmEdge *)pOtherObject) ;
                  SmCurve *pBrepCurve  = pBrepEdge->GetCurve() ;
                  SmCurve *pOtherCurve = pOtherEdge->GetCurve() ;
#ifdef SM_USE_NEWTOL      
                  SM_NEWTOL_LINE double   dBrepTol    = SmTol::GetZoneTol3d(pBrepObject) ;
                  SM_NEWTOL_LINE double   dOtherTol   = SmTol::GetZoneTol3d(pOtherObject) ;
#else // SM_USE_OLDTOL 
                  SM_OLDTOL_LINE double   dBrepTol    = rPCBrep.GetTolerance() ;
                  SM_OLDTOL_LINE double   dOtherTol   = rPCOther.GetTolerance() ;
#endif // SM_USE_OLDTOL
                  double   dTol        = smos_Max(dBrepTol, dOtherTol) ;
                  SmSolutionArray sTgtTgtSol ;
                  SmSolutionArray sClassification2BrepSol, sClassification2OtherSol ;
                  SmSolutionArray sTwoBrepCurvesSol,       sTwoOtherCurvesSol ;
                  SmPoint3d sClassBrepPt, sClassOtherPt, sBrepPt, sOtherPt ;
                  //double dClassBrepDev = 0.0, dClassOtherDev = 0.0, dBrepOtherDev = 0.0 ;

                  // see where the two classified curves intersect one another.  Is this a case of transitive tolerances
                  // where the classified curve intersects the target curves but the target curves don't intersect
                  // one another?
                  pBrepCurve->GlobalCurveIntersect(pBrepEdge->GetInterval(),
                                                   *pOtherCurve, pOtherEdge->GetInterval(),
                                                   dTol,
                                                   sTgtTgtSol) ;
                  sTgtTgtSol.Dump() ;

                  // see where the classification curve intersects the Brep target curve
                  cpClassificationCurve->GlobalCurveIntersect(cpClassificationCurve->GetNaturalInterval(),
                                                              *pBrepCurve, pBrepEdge->GetInterval(),
                                                              dTol,
                                                              sClassification2BrepSol) ;
                  sClassification2BrepSol.Dump() ;
                  // get 1st solution 3d points
                  if(sClassification2BrepSol.GetSize() > 0)
                    {
                      cpClassificationCurve->EvaluatePoint(sClassification2BrepSol[0].m_vStart[0], sClassBrepPt) ;
                      pBrepCurve->EvaluatePoint(sClassification2BrepSol[0].m_vStart[1], sBrepPt) ;
                      //dClassBrepDev = sBrepPt.DistanceBetween(sClassBrepPt) ;
                    }

                  // see where the classification curve intersects the Other target curve
                  cpClassificationCurve->GlobalCurveIntersect(cpClassificationCurve->GetNaturalInterval(),
                                                              *pOtherCurve, pOtherEdge->GetInterval(),
                                                              dTol,
                                                              sClassification2OtherSol) ;
                  sClassification2OtherSol.Dump() ;
                  // get 1st solution 3d points
                  if(sClassification2OtherSol.GetSize() > 0)
                    {
                      cpClassificationCurve->EvaluatePoint(sClassification2OtherSol[0].m_vStart[0], sClassOtherPt) ;
                      pOtherCurve->EvaluatePoint(sClassification2OtherSol[0].m_vStart[1], sOtherPt) ;
                      //dClassOtherDev = sOtherPt.DistanceBetween(sClassOtherPt) ;
                    }
                  if(   sClassification2BrepSol.GetSize() > 0
                     && sClassification2OtherSol.GetSize() > 0)
                    {
                      //dBrepOtherDev = sBrepPt.DistanceBetween(sOtherPt) ;
                    }

                  // when there are two different curves in the Brep model - see if they intersect
                  if(pBrepObject && pOtherMate)
                    {
                      SmEdge  *pMateEdge = ((SmEdge *)pOtherMate) ;
                      SmCurve *pMateCurve = pMateEdge->GetCurve() ;
                      pBrepCurve->GlobalCurveIntersect(pBrepEdge->GetInterval(),
                                                       *pMateCurve, pMateEdge->GetInterval(),
                                                       dTol,
                                                       sTwoBrepCurvesSol) ;
                      sTwoBrepCurvesSol.Dump() ;
                   }

                  // when there are two different curves in the Other model - see if they intersect
                  if(pOtherObject && pBrepMate)
                    {
                      SmEdge  *pMateEdge   = ((SmEdge *)pBrepMate) ;
                      SmCurve *pMateCurve  = pMateEdge->GetCurve() ;
                      pOtherCurve->GlobalCurveIntersect(pOtherEdge->GetInterval(),
                                                        *pMateCurve, pMateEdge->GetInterval(),
                                                        dTol,
                                                        sTwoOtherCurvesSol) ;
                      sTwoOtherCurvesSol.Dump() ;
                   }
                } // end classification object check is an edge check

              rPCBrep.Dump(lPointDim);
              rPCOther.Dump(lPointDim);

              SmBrep   *pBrep1       = rPCBrep.GetBrep() ;
              SmBrep   *pBrep2       = rPCOther.GetBrep() ;
              SmEdge   *pEdge1       = (pBrepObject  && pBrepObject->IsKindOf(SmEdge_TYPE))    ? (SmEdge*)pBrepObject    : NULL ;
              SmEdge   *pEdge2       = (pOtherObject && pOtherObject->IsKindOf(SmEdge_TYPE))   ? (SmEdge*)pOtherObject   : NULL ;
              SmEdge   *pEdge1Mate   = (pBrepMate    && pBrepMate->IsKindOf(SmEdge_TYPE))      ? (SmEdge*)pBrepMate      : NULL ;
              SmEdge   *pEdge2Mate   = (pOtherMate   && pOtherMate->IsKindOf(SmEdge_TYPE))     ? (SmEdge*)pOtherMate     : NULL ;
              SmVertex *pVertex1     = (pBrepObject  && pBrepObject->IsKindOf(SmVertex_TYPE))  ? (SmVertex*)pBrepObject  : NULL ;
              SmVertex *pVertex2     = (pOtherObject && pOtherObject->IsKindOf(SmVertex_TYPE)) ? (SmVertex*)pOtherObject : NULL ;
              SmVertex *pVertex1Mate = (pBrepMate    && pBrepMate->IsKindOf(SmVertex_TYPE))    ? (SmVertex*)pBrepMate    : NULL ;
              SmVertex *pVertex2Mate = (pOtherMate   && pOtherMate->IsKindOf(SmVertex_TYPE))   ? (SmVertex*)pOtherMate   : NULL ;
              SmPoint3d sBrepPoint, sOtherPoint ;
              rPCBrep.FindObjPoint3d(sBrepPoint) ;
              rPCOther.FindObjPoint3d(sOtherPoint) ;

              smgfx_Erase() ;
              smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1) { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(2,3, 0,1,0) ; if(pBrep2) { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(8,9, 1,0,0) ; sBrepPoint.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(8,9, 1,0,0) ; sOtherPoint.Draw() ; sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,0) ; if(pBrepFace) { pBrepFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(1,2, 0,0,0) ; if(pOtherFace) { pOtherFace->Draw(SM_DM_CROSSHATCH) ; } sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,0,1) ; if(pEdge1) { pEdge1->Draw() ; } sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,1,0) ; if(pEdge2) { pEdge2->Draw() ; } sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 1,1,0) ; if(pEdge1Mate) { pEdge1Mate->Draw() ; } sm_GraphicsLoop() ;
              smgfx_SetLook(5,6, 1,0,1) ; if(pEdge2Mate) { pEdge2Mate->Draw() ; } sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,0,1) ; if(pVertex1) { pVertex1->Draw() ; } sm_GraphicsLoop() ;
              smgfx_SetLook(3,4, 0,1,0) ; if(pVertex2) { pVertex2->Draw() ; } sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, 1,0,1) ; if(pVertex1Mate) { pVertex1Mate->Draw() ; } sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, 0,1,1) ; if(pVertex2Mate) { pVertex2Mate->Draw() ; } sm_GraphicsLoop() ;
              smgfx_SetLook(4,5, 0,1,1) ; rPCBrep.Draw(); sm_GraphicsLoop();
              smgfx_SetLook(6,7, 1,0,1) ; rPCOther.Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#else
          SM_REF1(cpClassificationCurve);
          SM_REF1(lPointDim);
#endif // SM_DEBUG_CODE

          // When both points classify to edges, reset point classifications back to unknown.
          // Note: check only when they both classify to edges.
          // [ 060808; also Fillet regression 2:219 ]
          // (Although now, classifications must be the same anyway. [B279])
          if(   eBrepClass  == SM_PC_EDGE
             && eOtherClass == SM_PC_EDGE)
            {
              rPCBrep .SetClassObject(SM_PC_UNKNOWN, NULL);
              rPCOther.SetClassObject(SM_PC_UNKNOWN, NULL);
            }
        } // end bad mated pair classification check
    } // end either Point classifies to an edge or a vertex check

  // all done
  return SM_SUCCESS;

} // end SmTopologyIntersector::FixupMidPointClassMates

/*******************************************************************//**
PURPOSE: Check specified interval to see if it split up
    a matched vertex point pair.  If it did and it's relatively
    short, replace the interval in both classifications with
    a single point classification tying the two matched vertices
    back together.

NOTES:

  CASE 1: When classifying two coincident faces eventually an edge
    of one face that was coincident with the other face was
    classified in preparation for insertion.  Due to random ordering
    the end vertices of the edge happened to already be inserted
    into the face and matched.  The classification of the edge's curve was
    corrupted due to small gap errors larger than tol. The
    classification violated the matching vertex rule because
    the matching vertex pair was split by a small interval as:

    SMTOPOLOGYINTERSECTOR ENTITY MAP, # ENTITY MAPS = 6
         Vertex : [0x05786A68] = 0x0567FDB8, Vert/Vert Dist = 0.0001222302924662

       THIS CLASIFICATION                              OTHER CLASSIFIATION
   [0] - 3dCurve Interval [0.0000000000000000,         [0] - 3dCurve Interval [0.0000000000000000,
                           0.0000852918015221]                                 0.0000852918015221]
     On Vertex - 0x05786A68,  <-+                        Unknown Point Classification
     On Face   - 0x0391EEE8,    |                        Unknown Point Classification
     On Face   - 0x0391EEE8,    +-matched-vertex-pair->  On Vertex - 0x0567FDB8,

   [1] - 3dCurve Interval [0.0000852918015221,         [1] - 3dCurve Interval [0.0000852918015221,
                           72.0620695551990170]                               72.0620695551990170]
     On Face   - 0x0391EEE8,                             On Vertex - 0x0567FDB8,
     On Face   - 0x0391EEE8,                             On Edge   - 0x056800F8,
     On Face   - 0x0391EEE8,                             On Edge   - 0x056800F8,

***********************************************************************/
SmStatus SmTopologyIntersector::FixupSplitVertexPair
  (ULONG lIntervalIndex,                     // in : index of target interval
   SmCurveClassification & rCurveClassBrep,  // in : first member of a pair of homogonized curveClassifications
   SmCurveClassification & rCurveClassOther, // in : the other member of the pair
   SmBoolean &bChangedClassifications)       // out: TRUE = changed input curveClassifications
                                             //      FALSE= didn't
{
  //// init output
  bChangedClassifications = FALSE ;

  // check state - homogenized classifications
  SM_ASSERT(   rCurveClassBrep.GetSize()  == rCurveClassOther.GetSize()
            && rCurveClassBrep.GetCurve() == rCurveClassOther.GetCurve()) ;

  // locals
  ULONG             lSize                  = rCurveClassBrep.GetSize() ;
  SmCurveInterval & rIvlB                  = rCurveClassBrep [lIntervalIndex];
  SmCurveInterval & rIvlO                  = rCurveClassOther[lIntervalIndex];
  SmZoneTol3d       sBrepSrcZoneTol3d      = SmTol::GetSrcZoneTol3d(&rCurveClassBrep) ; 
  SmZoneTol3d       sOtherSrcZoneTol3d     = SmTol::GetSrcZoneTol3d(&rCurveClassOther) ; 
  const SmCurve   * cpClassificationCurve  = rCurveClassBrep.GetCurve() ;
  SmExtent1d        sIvl ;
  rCurveClassBrep.GetInterval().Union(rCurveClassOther.GetInterval(), sIvl) ;

  // no work - don't fix closed intervals - expect interval ends to map to mated vertex pairs
  if(cpClassificationCurve->IsClosed(rIvlB.GetInterval()))
    {
      bChangedClassifications = FALSE ;
      return(SM_SUCCESS) ;
    }

  // find the pointClassifications with matched vertices
  SmPointClassification sPointClassB(sBrepSrcZoneTol3d,  &GetContext()),   // this tol is unlikely to be correct - check it
                        sPointClassO(sOtherSrcZoneTol3d, &GetContext()) ;  // this tol is unlikely to be correct - check it

  if     (   rIvlB.m_vStart.GetPointClass() == SM_PC_VERTEX
          && rIvlO.m_vEnd.GetPointClass()   == SM_PC_VERTEX
          && IsMatedPair(rIvlB.m_vStart.GetObject(), rIvlO.m_vEnd.GetObject()))
    {
      // save the startPoint of This and the EndPoint of other
      sPointClassB = rIvlB.m_vStart ;
      sPointClassO = rIvlO.m_vEnd ;
    }
  else if(   rIvlB.m_vEnd.GetPointClass() == SM_PC_VERTEX
          && rIvlO.m_vStart.GetPointClass()   == SM_PC_VERTEX
          && IsMatedPair(rIvlB.m_vEnd.GetObject(), rIvlO.m_vStart.GetObject()))
    {
      // save the EndPoint of This and the StartPoint of other
      sPointClassB = rIvlB.m_vEnd ;
      sPointClassO = rIvlO.m_vStart ;
    }
  else
    {
      // nothing to clean up
      bChangedClassifications = FALSE ;
      return(SM_SUCCESS) ;
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
static ULONG lCount      = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
  if (bDebugMe || lCount == lDebugCount)
    {
      this->Dump() ;           // show matched entity mapping
      rCurveClassBrep.Dump();
      rCurveClassOther.Dump();

      SmBrep *pBrep1     = rCurveClassBrep.GetBrep() ;
      SmBrep *pBrep2     = rCurveClassOther.GetBrep() ;
      SmVertex *pVertex1 = sPointClassB.GetVertexObject() ;
      SmVertex *pVertex2 = sPointClassO.GetVertexObject() ;

      smgfx_Erase() ;
      smgfx_SetLook(1,2, 0,0,1) ; if(pBrep1)  { pBrep1->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 0,1,0) ; if(pBrep2)  { pBrep2->Draw(TRUE) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,.5,1); if(pVertex1){ pVertex1->Draw() ; } sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, .2,1,0); if(pVertex2){ pVertex2->Draw() ; } sm_GraphicsLoop() ;
      smgfx_SetLook(1,2, 1,0,1) ; if(pVertex1 && pVertex2) { pVertex2->GetPoint().DrawPointToPoint(pVertex1->GetPoint()) ; } sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 1,0,0) ; rCurveClassBrep.GetCurve()->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 0,1,1) ; rCurveClassBrep.GetCurve()->Draw(&rIvlB.m_vInterval) ; sm_GraphicsLoop() ;
      sm_GraphicsLoop() ;
      //      smgfx_SetCurveClassificationObjectLook(5,7,
      //                                             0, 0,1,
      //                                             0,.5,1,
      //                                             .5,0,1) ; rCurveClassBrep.DrawObjects(); sm_GraphicsLoop();
      //      smgfx_SetCurveClassificationObjectLook(7,9,
      //                                             0, 1,0,
      //                                             0,1,.5,
      //                                             .5,1,0) ; rCurveClassOther.DrawObjects(); sm_GraphicsLoop();
      //      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // arrive here after finding matched point classifications on different
  // ends of an interval classification.

  // figure out what's going on. In the case that generated the need for this code
  //  An edge was mapped to a face which had a vertex in it.  The edge's end
  //  vertices were mapped to vertices in the face. For some
  //  reason the classification parameters for the matched vertices were
  //  different and a short interval was created.  Perhaps we should check
  //  for some kind of consistent classifications before applying the following
  //  fix.  For now, just remove the interval, fix up the newly mated end points and return.

  // drop the mated vertices to the curve to find their best projection points
  SmBoolean bSuccessB, bSuccessO ;
  double dDropParamB = 0.0, dDistToCurveB = 0.0, dParam = 0.0;
  double dDropParamO = 0.0, dDistToCurveO = 0.0;
  double dGuessTB = sPointClassB.GetTParam() ;
  double dGuessTO = sPointClassO.GetTParam() ;
  cpClassificationCurve->DropPoint(sIvl,                                               // in : target curve allowed domain
                                   sPointClassB.GetVertexObject()->GetPoint(),         // in : Point to drop to curve
                                   NULL,                                               // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                                       //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                                       //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
#ifdef SM_USE_NEWTOL
                                   SM_NEWTOL_LINE 10.0 * sPointClassB.GetGap3d(),      // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
#else // SM_USE_OLDTOL
                                   SM_OLDTOL_LINE 10.0 * sPointClassB.GetDeviation(),  //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
#endif // SM_USE_OLDTOL
                                                                                       //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                                       //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                   &dGuessTB,                                          // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                   bSuccessB,                                          // out: TRUE = found a drop point
                                   dDropParamB,                                        // out: found drop curve param
                                   dDistToCurveB) ;                                    // out: found drop distance
                                                                                       // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                                       //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                                       //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                                       //      default:[SM_SO_MINIMIZE] to preserve original behavior
  
  cpClassificationCurve->DropPoint(sIvl,                                               // in : target curve allowed domain
                                   sPointClassO.GetVertexObject()->GetPoint(),         // in : Point to drop to curve
                                   NULL,                                               // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                                                       //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                                                       //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
#ifdef SM_USE_OLDTOL
                                   SM_OLDTOL_LINE 10.0 * sPointClassO.GetGap3d(),      // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
#else // SM_USE_NEWTOL
                                   SM_NEWTOL_LINE 10.0 * sPointClassO.GetDeviation(),  //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
#endif // SM_USE_NEWTOL
                                                                                       //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                                                       //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                   &dGuessTO,                                          // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                   bSuccessO,                                          // out: TRUE = found a drop point
                                   dDropParamO,                                        // out: found drop curve param
                                   dDistToCurveO) ;                                    // out: found drop distance
                                                                                       // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                                                       //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                                                       //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                                                       //      default:[SM_SO_MINIMIZE] to preserve original behavior
  if(!bSuccessB && !bSuccessO)
    {
      bChangedClassifications = FALSE ;
      return(SM_ERR) ;
    }

  // pick a common curve parameter for the saved PointClasses
  //   favor the ThisBrep parameter value when its deviation is small
  //   since we will be merging geometry into the this brep
  dParam =   (   bSuccessB && bSuccessO
              && dDistToCurveB < dDistToCurveO/5.0) ? (dDropParamB)
           : (bSuccessB && bSuccessO) ? (dDropParamB + dDropParamO)/2.0
           : (bSuccessB) ? dDropParamB
           : dDropParamO ;

  // update saved point data
  sPointClassB.SetTParam(dParam) ;
#ifdef SM_USE_NEWTOL      
  SM_NEWTOL_LINE sPointClassB.SetGap3d(bSuccessB ? dDistToCurveB : sPointClassB.GetGap3d()) ;
#else // SM_USE_OLDTOL 
  SM_OLDTOL_LINE sPointClassB.SetDeviation(bSuccessB ? dDistToCurveB : sPointClassB.GetDeviation()) ;
#endif // SM_USE_OLDTOL

  sPointClassO.SetTParam(dParam) ;
#ifdef SM_USE_NEWTOL      
  SM_NEWTOL_LINE sPointClassO.SetGap3d(bSuccessO ? dDistToCurveO : sPointClassO.GetGap3d()) ;
#else // SM_USE_OLDTOL 
  SM_OLDTOL_LINE sPointClassO.SetDeviation(bSuccessO ? dDistToCurveO : sPointClassO.GetDeviation()) ;
#endif // SM_USE_OLDTOL

  // propogate the persistent PtClassification data to the
  // neighbor boundaries as appropriate
  if(lIntervalIndex >= 1)          { rCurveClassBrep [lIntervalIndex-1].m_vEnd = sPointClassB ;
                                     rCurveClassOther[lIntervalIndex-1].m_vEnd = sPointClassO ;
                                   }
  if(   lSize > 1
     && lIntervalIndex <= lSize-2) { rCurveClassBrep [lIntervalIndex+1].m_vStart = sPointClassB ;
                                     rCurveClassOther[lIntervalIndex+1].m_vStart = sPointClassO ;
                                   }
  // remove the specified interval
  rCurveClassBrep .RemoveAt(lIntervalIndex) ;
  rCurveClassOther.RemoveAt(lIntervalIndex) ;

  // all done
  SM_DBG_WARN(_T("SmTopologyIntersector::FixupSplitVertexPair - modified classification pair to match vertices split by tolerance")) ;
  bChangedClassifications = TRUE ;
  return(SM_SUCCESS) ;

} // end SmTopologyIntersector::FixupSplitVertexPair

/*******************************************************************//**
PURPOSE: Try to fix a problem with a gap in the intersection loop
   by looking for an adjacent vertex that is related.  Then test the
   edge between them to see if it is on both.

NOTES:
***********************************************************************/
SmStatus SmTopologyIntersector::CloseIntersectionLoop
  (SmVertex  * pBrepVertex,          // in : target vertex from m_pBrep
   SmBoolean & rbMadeNewTopology)    // out: TRUE =
                                     //      FALSE=
{
  // check input
  NER(pBrepVertex);

  // find mating vertex in m_pOther brep
  SmVertex *pOtherVertex = SM_CAST_PTR(SmVertex,GetOtherMate(pBrepVertex));
  NER(pOtherVertex);

  // init output
  rbMadeNewTopology = FALSE;

  // locals
  SM_PTR_ARRAY(sOCommonEdges,SmEdge,32);
  ULONG ii, jj;

    { // scope to hold all edges connected to target vertex

      // get all edges connected to target vertex
      SmTArray<SmEdge*> sBrepEdges;
      pBrepVertex->GetEdges(sBrepEdges);

      // for every edge connected to target vertex
      for (ii=0; ii<sBrepEdges.GetSize(); ii++)
        {
          // target edge and any mated edge from m_pOther brep
          SmEdge *pBE = sBrepEdges[ii];
          SmEdge *pOE = SM_CAST_PTR(SmEdge,GetOtherMate(pBE));

          // when there is no mated edge
          if (!pOE)
            {
              // get target edge's other vertex and see if that vertex is mated
              SmVertex *pBVertex2 = pBE->GetOtherVertex(pBrepVertex);
              SmVertex *pOVertex2 = SM_CAST_PTR(SmVertex,GetOtherMate(pBVertex2));

              // skip closed curves
              if (pBVertex2 == pBrepVertex)
                { continue; }

              // quit if other edge vertex is not mated
              if (!pOVertex2)
                { continue; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
              if (bDebugMe)
                {
                  smgfx_Erase();
                  smgfx_SetLook(3,4, 1,0,0); pBVertex2->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(3,4, 0,0,1); pBrepVertex->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(2,3, 1,0,1); pBE->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(1,2, 0,0,0); m_pBrep->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(1,2, 0,1,1); m_pOther->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE
              // get all edges connected to edge's othervertex mated partner
              pOtherVertex->GetCommonEdges(pOVertex2,sOCommonEdges);

              // When only one edge connects to the vertex
              if (sOCommonEdges.GetSize() == 1)
                {
                  // We might be able to solve the problem right here.

                  // distances between mated pairs of vertices
                  double dDist1 = pBrepVertex->GetPoint().DistanceBetween(pOtherVertex->GetPoint());
                  double dDist2 = pBVertex2->GetPoint().DistanceBetween(pOVertex2->GetPoint());
                  double dMaxDist = smos_Max(dDist1,dDist2);

                  // midPoint of this m_pBrep target edge
                  SmPoint3d sMidPnt;
                  SER(pBE->GetPrimaryEdgeuse()->NormalizedEvaluate(0.5,FALSE,sMidPnt));  // TRUE = UV Eval, FALSE = 3d Eval

                  // drop midPoint to sole other m_pOther Brep edge connected to mated vertex
                  SmEdge *pOEdge = sOCommonEdges[0];
                  SmBoolean bSuccess;
                  double dParam = 0.0, dDistToCurve = 0.0;
                  SER(pOEdge->GetCurve()->DropPoint
                               (pOEdge->GetInterval(), // in : target curve allowed domain
                                sMidPnt,               // in : Point to drop to curve
                                NULL,                  // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                                       //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                                       //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                                dMaxDist*2.0,          // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                                       //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                                       //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                                       //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                NULL,                  // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                                bSuccess,              // out: TRUE = found a drop point
                                dParam,                // out: found drop curve param
                                dDistToCurve)) ;       // out: found drop distance
                                                       // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                                       //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                                       //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                                       //      default:[SM_SO_MINIMIZE] to preserve original behavior

                  // when point dropped to curve within tolerance
                  if (bSuccess)
                    {
                      // add the edge pair to the list of mated object
                      // thus filling a potential intersection loop gap
                      Relate(pBE,pOEdge);
                      rbMadeNewTopology = TRUE;
                      return SM_SUCCESS;
                    }
                } // end Edge->OtherVertex->Mate connected to only one edge check

              // intersect the m_pOTher Brep with the target edge
              SmSolutionArray sSolutions;
                {
                  SmTemporaryChangeValue<SmBoolean> sChange(m_pOther->m_bEditingEnabled,FALSE);
                  SER(SmTopologySolver::BrepCurveSolve(m_pOther,
                                                       *pBE->GetCurve(),
                                                       pBE->GetInterval(),
                                                       SM_SO_INTERSECT,
                                                       SM_SR_ALL,
                                                       pBE->GetTolerance(),
                                                       SM_BIG_DOUBLE,
                                                       NULL,
                                                       sSolutions));
                }

              // check every solution - for a coincident solution
              SmBoolean bDoMerge = FALSE;
              for (jj=0; jj<sSolutions.GetSize(); jj++)
                {
                  SmSolution & rSol = sSolutions[jj];

                  // remember that a coincident solution was seen
                  if (rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
                    {
                      bDoMerge = TRUE;
                      break ;
                    }
                }

              // when a coincident solution was seen
              if (bDoMerge)
                {
                  SmTArray<SmEdge*> sOldEdges, sNewEdges;
                  m_pOther->GetEdges(sOldEdges);

                  // intersect, insert, and relate target Edge with m_pOther brep
                  SER(IIREdgeBrep(pBE,m_pOther,FALSE,TRUE));

                  // remember if  new edges were added to m_pOther
                  m_pOther->GetEdges(sNewEdges);
                  if (sNewEdges.GetSize() > sOldEdges.GetSize())
                    {
                      rbMadeNewTopology = TRUE;
                    }
                  break;
                }
            } // end no mated edge check
        } // end iter every edge connected to target vertex
    } // end scope to hold all edges connected to target vertex

  // when no topology was added to m_pOther - see if other edges can be added to this m_pBrep
  if (!rbMadeNewTopology)
    {
      // get all edges connected to other vertex
      SmTArray<SmEdge*> sOtherEdges;
      pOtherVertex->GetEdges(sOtherEdges);

      // for every edge connected to otherVertex
      for (ii=0; ii<sOtherEdges.GetSize(); ii++)
        {
          // get target edge and any mated edge within m_pBrep
          SmEdge *pOE = sOtherEdges[ii];
          SmEdge *pBE = SM_CAST_PTR(SmEdge,GetBrepMate(pOE));

          // when target edge is not mated to a m_pBrep edge
          if (!pBE)
            {
              // get targetEdge->OtherVertex
              SmVertex *pOVertex2 = pOE->GetOtherVertex(pOtherVertex);

              // skip closed curves
              if (pOtherVertex == pOVertex2)
                { continue; }

              // skip cases where OtherVertex is not mated to a m_pBrep Vertex
              SmVertex *pBVertex2 = SM_CAST_PTR(SmVertex,GetBrepMate(pOVertex2));
              if (!pBVertex2)
                { continue; }
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
              if (bDebugMe)
                {
                  smgfx_Erase();
                  smgfx_SetLook(3,4, 1,0,0); pBVertex2->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(3,4, 0,0,1); pBrepVertex->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(2,3, 1,0,1); pOE->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(1,2, 0,0,0); m_pBrep->Draw(); sm_GraphicsLoop();
                  smgfx_SetLook(1,2, 0,1,1); m_pOther->Draw(); sm_GraphicsLoop();
                  sm_GraphicsLoop();
                }
#endif // SM_DEBUG_CODE
              // intersect target edge with m_pBrep
              SmSolutionArray sSolutions;
                {
                  SmTemporaryChangeValue<SmBoolean> sChange(m_pBrep->m_bEditingEnabled,FALSE);
                  SER(SmTopologySolver::BrepCurveSolve(m_pBrep,*pOE->GetCurve(),pOE->GetInterval(),
                      SM_SO_INTERSECT,SM_SR_ALL,pOE->GetTolerance(),SM_BIG_DOUBLE,NULL,sSolutions));
                }

              // Check solutions for a coincident solution
              SmBoolean bDoMerge = FALSE;
              for (jj=0; jj<sSolutions.GetSize(); jj++)
                {
                  SmSolution & rSol = sSolutions[jj];
                  if (rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES)
                    {
                      bDoMerge = TRUE;
                      break ;
                    }
                }

              // when a coincident solution was found
              if (bDoMerge)
                {
                  SmTArray<SmEdge*> sOldEdges, sNewEdges;
                  m_pBrep->GetEdges(sOldEdges);

                  // intersect, insert, and relate target edge into this  m_pBrep
                  SER(IIREdgeBrep(pOE,m_pBrep,TRUE,TRUE));
                  m_pBrep->GetEdges(sNewEdges);

                  // remember if new topology was added to m_pBrep
                  if (sNewEdges.GetSize() > sOldEdges.GetSize())
                    {
                      rbMadeNewTopology = TRUE;
                    }
                  break;
                }
            } // end target edge not mated check
        } // end iter every edge connected to OtherVertex
    } // end no topology added to m_pOther Brep check

  // all done
  return SM_SUCCESS;

} // end SmTopologyIntersector::CloseIntersectionLoop

/*******************************************************************//**
PURPOSE: Re-intersect all surfaces surrounding this vertex in an
    attempt to pick up one or more missing edges going out of the
    vertex.

NOTES:
***********************************************************************/
SmStatus SmTopologyIntersector::ReIntersectAtVertex
  (SmVertex  * pBrepVertex,           // in :
   SmVertex  * pOtherVertex,          // in :
   SmBoolean & rbMadeNewTopology)     // out:
{
  // check input
  NER(pBrepVertex);
  NER(pOtherVertex);

  // init output
  rbMadeNewTopology = FALSE;

  // locals
  SmTArray<SmFace*> sBFaces;
  SmTArray<SmFace*> sOFaces;
  SmTArray<SmSurface*> sBSurfaces;
  SmTArray<SmSurface*> sOSurfaces;

  // get all faces connected to inupt pair of vertices
  pBrepVertex->GetFaces(sBFaces);
  pOtherVertex->GetFaces(sOFaces);

  // get all unique surfaces used by the pBrepVertex VertexFaces
  ULONG ii, jj;
  for (ii=0; ii<sBFaces.GetSize(); ii++)
    {
      SmFace    *pBFace    = sBFaces[ii];
      SmSurface *pBSurface = pBFace->GetSurface();
      sBSurfaces.AddUnique(pBSurface);
    }

  // get all unique surfaces used by the pOtherVertex VertexFaces
  for (ii=0; ii<sOFaces.GetSize(); ii++)
    {
      SmFace    *pOFace    = sOFaces[ii];
      SmSurface *pOSurface = pOFace->GetSurface();
      sOSurfaces.AddUnique(pOSurface);
    }

  // Get all currently mapped Edges and Vertices prior to upcoming surf/surf IIR calls
  SmTArray<SmEdge*> sBEdges, sOEdges;
  SmTArray<SmVertex*> sBVerts, sOVerts;
  GetCommonEdges(sBEdges,sOEdges);
  GetCommonVertices(sBVerts,sOVerts);
  ULONG lECount = sBEdges.GetSize();
  ULONG lVCount = sBVerts.GetSize();

  // IIR every Brep/Other pair of surfaces
  for (ii=0; ii<sBSurfaces.GetSize(); ii++)
    {
      SmSurface *pBSurface = sBSurfaces[ii];

      for (jj=0; jj<sOSurfaces.GetSize(); jj++)
        {
          SmSurface *pOSurface = sOSurfaces[jj];

          // interesect insert and relate surfaces
          SER(IIRSurfs(pBSurface,pOSurface));
        }
    } // end iter every pair of Brep/Other surfaces

  // get all mapped Edges and Vertices after surf/surf IIR calls
  GetCommonEdges(    sBEdges, sOEdges );
  GetCommonVertices( sBVerts, sOVerts );

  // when the mapped vertex or edge counts changed
  if(   lECount != sBEdges.GetSize()
     || lVCount != sBVerts.GetSize() )
    {
      // remember that new topology was added to the Brep
      rbMadeNewTopology = TRUE;
    }

  // all done
  return SM_SUCCESS;

} // end SmTopologyIntersector::ReIntersectAtVertex


/*******************************************************************//**
PURPOSE: Sets the subset of geometry objects in the first brep
  to intersect with the other brep.

NOTES: This operation clears all previous subsets.
***********************************************************************/
SmStatus SmTopologyIntersector::SetSubset
  (const SmTArray<SmObject*> & crSubsetGeometry)  // in : subset of Geometry
{
  // clear out any old Subset SmMapPtrToPtr object
  if (m_pSubset) { delete m_pSubset;
                   m_pSubset = NULL;
                 }

  // size the new array
  if (crSubsetGeometry.GetSize() > 0)
    {
      m_pSubset = new (m_crContext) SmMapPtrToPtr<SmObject, SmObject>();
    }

  // for every Object in the subset make a subset entry
  ULONG ii, lNumSubs = crSubsetGeometry.GetSize();
  for (ii=0; ii<lNumSubs; ii++)
    {
      SmObject *pObj = crSubsetGeometry[ii];
      m_pSubset->Insert(pObj,pObj);
    }

  // all done
  return SM_SUCCESS;

} // end SmTopologyIntersector::SetSubset

/*******************************************************************//**
PURPOSE:  Return TRUE when every mapped m_pBrep edge is connected
  to vertices that are connected to at least two mapped edges.

NOTES:  When this condition is violated it means that some
  intersection loop is not closed.  This is an error condition when
  doing booleans on manifold objects after all Intersect, Insert, and
  Relate relations are processed - but not an error condition for
  nonManifold cases.
***********************************************************************/
SmBoolean SmTopologyIntersector::AreIntersectionLoopsClosed()
 const
{
  // locals
  ULONG ii, jj, kk, lEdgeCount, lFoundIndex ;
  SmEdge      *pEdgeData[20] ;
  SmVertex    *pVertexData[2] ;
  SmTArray<SmEdge*>      sVertexEdges(20, pEdgeData) ;
  SmTArray<SmVertex*>    sVertices   (2, pVertexData) ;

  // get all mapped m_pBrep edges
  SmTArray<SmObject *> sEdgeList ;
  GetMappedBrepObjects(SmEdge_TYPE, sEdgeList) ;

  // for every mapped edge
  for(ii=0;ii<sEdgeList.GetSize();ii++)
    {
      SmEdge *pEdge = (SmEdge *)sEdgeList[ii] ;

      // Get Edge end Vertices
      pEdge->GetVertices(sVertices) ; // only gets one vertex for closed curves
      SM_ASSERT(   (sVertices.GetSize() == 2 && pEdge->IsClosed() == FALSE)
                || (sVertices.GetSize() == 1 && pEdge->IsClosed() == TRUE))

      // for both edge vertices
      for(jj=0;jj<sVertices.GetSize();jj++)
        {
          SmVertex *pVertex = sVertices[jj] ;

          // get all edges connected to Vertex
          pVertex->GetEdges(sVertexEdges) ;

          // for every vertexEdge - count the mapped edges
          lEdgeCount = 0 ;
          for (kk=0; kk<sVertexEdges.GetSize(); kk++)
            {
              SmEdge *pTrialEdge = sVertexEdges[kk];

              // when pTrialEdge is a mapped edge
              if(   pTrialEdge == pEdge
                 || sEdgeList.FindElement(pTrialEdge, lFoundIndex))
                {
                  // count open edges once and closed edges twice
                  lEdgeCount += (pTrialEdge->IsClosed()) ? 2 : 1 ;

                  // done with this vertex when 2nd mapped edge is found
                  if(lEdgeCount >= 2)
                    break ;
                } // end found a mapped edge check
            } // end iter every VertexEdge attached to this vertex

          // return FALSE when any vertex is found attache to just one mapped edge
          SM_ASSERT(lEdgeCount >= 1) ;
          if(lEdgeCount < 2)
            {
              // not all intersection loops are closed
              return(FALSE) ;
            }

        } // end iter both Edge->Vertices
    } // end iter every mapped m_pBrep Edge

  // arrive here when all mapped edges are connected to vertices
  // which are connected to at least two mapped edges.  This
  // means that all intersection loops are closed at this time.
  return(TRUE) ;

} // end SmTopologyIntersector::AreIntersectionLoopsClosed



//----------------------------------------------------------------------
//
// The following four methods are generally for internal use.
// They make use of already-processed topology.
// First there are five static helper routines.
//
//----------------------------------------------------------------------

/*******************************************************************//**
PURPOSE: static helper:
   Test to see whether a parameter value is in the interior of an Edge, to tolerance.
   I.e., should we split the edge at this parameter value.

NOTES: If trouble (return SM_ERR), returns bInterior == FALSE.
***********************************************************************/
static SmStatus sm_CheckInteriorEdge(
        const SmEdge     * pEdge,
              double       dT,
              double       dTol,
              SmBoolean  & bInterior
    )
{
  bInterior = FALSE;

  // Check common simple case first.
  // Note, negative tol excludes the ends.
  SmExtent1d sEdgeDomain = pEdge->GetInterval();
  if ( ! sEdgeDomain.ContainsValue( dT, -SM_EFF_ZERO ) )
    { return SM_SUCCESS; }

  SmPoint3d sCrvPt[2];
  SmCurve *pCurve = pEdge->GetCurve();
  SER( pCurve->Evaluate( dT, 1, FALSE, sCrvPt ) );
  double dParamLen = sCrvPt[1].Length();
  if ( dParamLen > 0.1 )
  {
      double dParamTol = dTol / dParamLen;

      // Because the param length can vary, use this test
      // only as a rough guide: if it's anywhere close,
      // go on to the 3d test.  So use a bigger tolerance.
      dParamTol *= 20.0;
      if ( sEdgeDomain.ContainsValue( dT, -dParamTol ) )
      {
          bInterior = TRUE;
          return SM_SUCCESS;
      }
  }
  // Work in 3d.
  // This is also more reliable in case of vertex/edge tolerance.

  SmVertex *pV = pEdge->GetVertex();
  double dDist = sCrvPt[0].DistanceBetween( pV->GetPoint() );
  if ( dDist <= dTol )
    { return SM_SUCCESS; }

  pV = pEdge->GetOtherVertex( pV );
  dDist = sCrvPt[0].DistanceBetween( pV->GetPoint() );
  if ( dDist <= dTol )
    { return SM_SUCCESS; }

  bInterior = TRUE;
  return SM_SUCCESS;

} // end static sm_CheckInteriorEdge


// A local enum, for sm_TestAndSplitTwoEdges() and sm_TestAndImprintFace().
enum sReasonType
{
  NO_REASON,
  FAR_FROM_GEOM,
  CLOSE_TO_BOUNDARY
};


/*******************************************************************//**
PURPOSE: static helper:
   Test to see if two edges intersect in their interiors,
   and if so, create a new vertex and split the edges with it.

NOTES: 
   Doesn't check bounding boxes; the caller should do that first.
***********************************************************************/

static SmStatus
sm_TestAndSplitTwoEdges(SmEdge      * pEdge1,       // in
                            SmEdge      * pEdge2,       // in
                            double        dTol,         // in
                            double      & rdDistToEdge, // out: always set, whether split or not
                            double      & rdMinUnstitched,  // out: always set, whether split or not
                            SmTArray< SmVertex *> & rNewVerts,  // out
                            SmTArray< SmEdge   *> & rNewEdges1, // out
                            SmTArray< SmEdge   *> & rNewEdges2) // out
{
    // Init outputs
    rdDistToEdge = 0.0;
    rdMinUnstitched  = SM_BIG_DOUBLE;

    rNewVerts .ReSet();
    rNewEdges1.ReSet();
    rNewEdges2.ReSet();

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
    if (bDebugMe) {
        sm_GraphicsLoop();
        smgfx_Erase();
        smgfx_SetLook( 2,4, 1,0,0); pEdge1->Draw(); sm_GraphicsLoop();
        smgfx_SetLook( 2,3, 0,1,1); pEdge2->Draw(); sm_GraphicsLoop();
        sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE


    SmCurve *pCurve1 = pEdge1->GetCurve(); NER(pCurve1);
    SmCurve *pCurve2 = pEdge2->GetCurve(); NER(pCurve2);

    SmSolutionArray sSolutions;
    SmBoolean bSkipGlobalCoincidenceCheck = TRUE; //cbi ?

    SmExtent1d sE1Domain = pEdge1->GetInterval();
    SmExtent1d sE2Domain = pEdge2->GetInterval();

    // Use a larger tol, to catch close ones for rdMinUnstitched.
    SER(pCurve1->GlobalCurveIntersect( sE1Domain, *pCurve2, sE2Domain,
        10*dTol, sSolutions, bSkipGlobalCoincidenceCheck ));

    ULONG lNumSols = sSolutions.GetSize();
    if ( lNumSols < 1 )
    {
        return SM_SUCCESS;
    }

    ULONG ii;
    for ( ii = 0; ii < lNumSols; ii++ )
    {
        SmSolution & rSol = sSolutions[ii];

        // We should be able to skip range solutions.
        if ( rSol.m_eSolutionType != SM_ST_SINGLE_VALUE )
          { continue; }

        if ( rSol.m_vStart.m_dSolutionValue > dTol )
        {
            rdMinUnstitched = smos_Min( rdMinUnstitched, rSol.m_vStart.m_dSolutionValue );
            continue;
        }

        // Check for internal to both edges.
        double dT1 = rSol.m_vStart[0];
        SmBoolean bInterior = FALSE;
        SmStatus eStat = sm_CheckInteriorEdge( pEdge1, dT1, dTol, bInterior );
        if ( bInterior == FALSE ) { continue; }

        double dT2 = rSol.m_vStart[1];
        eStat = sm_CheckInteriorEdge( pEdge2, dT2, dTol, bInterior );
        if ( bInterior == FALSE ) { continue; }

        // The intersection is interior to both Edges.
        // Split them both, and then glue the vertices.
        SmBrep *pBrep = pEdge1->GetBrep();  NER( pBrep );

        SmEdge *pE1 = NULL, *pE2 = NULL;
        SmVertex *pNewV1 = NULL, *pNewV2 = NULL;
        eStat = pBrep->MakeVertexSplitEdge( pEdge1, dT1, pE1, pE2, pNewV1 );
        if ( eStat != SM_SUCCESS)
        {
            SM_ASSERT( FALSE );
            // Maybe because of tolerance.  Check that.
            SmPoint3d sPt1, sPt2;
            pCurve1->EvaluatePoint( dT1, sPt1 );
            pCurve2->EvaluatePoint( dT2, sPt2 );
            if ( sPt1.CloserThan( rdMinUnstitched, sPt2 ) )
              { rdMinUnstitched = sPt1.DistanceBetween( sPt2 ); }

            continue;
        }
        NER(pNewV1);
        SmEdge *pNewEdge = pE2;
        if (pE2 == pEdge1) {
            pNewEdge = pE1;
        }
        rNewEdges1.Add( pNewEdge );

        pBrep = pEdge2->GetBrep();  // Presumably the same...
        eStat = pBrep->MakeVertexSplitEdge( pEdge2, dT2, pE1, pE2, pNewV2 );
        if ( eStat != SM_SUCCESS)
        {
            SM_ASSERT( FALSE );
            continue;
        }
        NER(pNewV2); // pNewV2 will be deleted, so don't add to output list.
        pNewEdge = pE2;
        if (pE2 == pEdge2) {
            pNewEdge = pE1;
        }
        rNewEdges2.Add( pNewEdge );

        rdDistToEdge = pNewV1->GetPoint().DistanceBetween( pNewV2->GetPoint() );

        // And glue the vertices.

        // First set the position of the surviving vertex to be
        // the midpoint of the two ... this should help to keep tolerances
        // from growing excessively.
        SmPoint3d sMidPt = 0.5 * ( pNewV1->GetPoint() + pNewV2->GetPoint() );
        pNewV1->SetPoint( sMidPt );

        pBrep->GlueVertices( pNewV1, pNewV2 );

        pNewV2 = NULL;
        rNewVerts.Add( pNewV1 ); // This one will survive.
    }

    return SM_SUCCESS;

} // end static sm_TestAndSplitTwoEdges


/*******************************************************************//**
PURPOSE: static helper:
   Test to see if a point is on a face in its interior,
   and if so, create a new shell vertex and imprint it into the face.

NOTES: 
***********************************************************************/
static SmStatus sm_TestAndImprintFace
 (SmFace          * pFace,
  const SmPoint3d & crPoint,       // in
  SmZoneTol3d       sSrcZoneTol3d, // in : Obj ZoneTol3d assoc with crPoint
  sReasonType     & reReason,      // out
  double          & rdDistToFace,  // out: always set, whether split or not
  SmVertex       *& rpNewVertex)   // out
{
  // Init outputs
  rdDistToFace = SM_BIG_DOUBLE;
  rpNewVertex  = NULL;
  reReason     = NO_REASON;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      sm_GraphicsLoop();
      smgfx_Erase();
      smgfx_SetLook( 2,4, 1,0,0); crPoint.Draw(); sm_GraphicsLoop();
      smgfx_SetLook( 1,3, 0,1,1); pFace ->Draw(); sm_GraphicsLoop();
      sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // SmZoneTol3d sFaceZoneTol3d = SmTol::GetZoneTol3d(pFace) ;
  SmSurface * pSurface       = pFace->GetSurface(); NER(pSurface);
  SmBoolean   bSuccess;
  SmBoolean   bIsMulti;
  SmPoint2d   sUV;
  SER(pSurface->DropPoint(crPoint, pFace->GetUVDomain(), NULL,
                          bSuccess, sUV, rdDistToFace, bIsMulti ));

  // Check far from surface.
  SmXSectTol3d sXSectTol3d = SmTol::GetXSectTol3d(sSrcZoneTol3d, SmTol::GetZoneTol3d(pFace)) ;
  if ( !bSuccess || rdDistToFace > sXSectTol3d )
  {
      reReason = FAR_FROM_GEOM;
      return SM_SUCCESS;
  }

  // Check interior.
  // Note, it will presumably not be close to a vertex or an edge of the face.
  SmBoolean bIntWithBoundary = FALSE;
  SmPointClassification sPointClass(sSrcZoneTol3d, pFace->GetContext()) ;  

  pFace->PointClassify( sUV, sSrcZoneTol3d, bIntWithBoundary, TRUE, sPointClass );

  if ( sPointClass.GetPointClass() == SM_PC_UNKNOWN )
  {
      reReason = FAR_FROM_GEOM;
      return SM_SUCCESS;
  }
  if (   sPointClass.GetPointClass() == SM_PC_EDGE
      || sPointClass.GetPointClass() == SM_PC_VERTEX )
  {
      reReason = CLOSE_TO_BOUNDARY;
      return SM_SUCCESS;
  }
  if ( sPointClass.GetPointClass() != SM_PC_FACE )
  {
      SM_ASSERT( FALSE );
      reReason = CLOSE_TO_BOUNDARY; // most probable reason; don't really care too much.
      return SM_SUCCESS;
  }


  // Point is on the surface, within the face.
  SmVertex *pNewV = NULL;
  SmLoop   *pNewLoop = NULL;
  if (pFace->GetBrep()->MakeVertexLoop(pFace,crPoint,pNewLoop,pNewV) != SM_SUCCESS) {
      reReason = CLOSE_TO_BOUNDARY; // most probable reason; don't really care too much.
      return SM_SUCCESS;
  }
  NER(pNewV);
  rpNewVertex = pNewV;

  return SM_SUCCESS;

} // end static sm_TestAndImprintFace

#if 0  // Unused
/*******************************************************************//**
PURPOSE: static helper:
   See whether a curve is coincident with an Edge.

NOTES: 
***********************************************************************/
static SmBoolean sm_CurveEdgeCoincidence( SmCurve *pCurve, SmEdge *pEdge, double dTol )
{
  // Easy outs:
  if ( pCurve == NULL ) { return FALSE; }
  if ( pEdge  == NULL ) { return FALSE; }
  if ( dTol   <  SM_EFF_ZERO_SQ ) { return FALSE; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if(bDebugMe)
  {
    smgfx_SetLook( 4,6, 0,1,1 ); pEdge ->DrawParams(); sm_GraphicsLoop();
    smgfx_SetLook( 2,4, 1,0,0 ); pCurve->DrawParams(); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  // Method:
  // Get verts of the edge.
  // Drop them to the 3d curve: if too far, not coinc.
  // Make an Interval of the two drop params on the curve.
  // Call crv coincidence checker.

  // First see whether their directions are the same or opposite.
  // Evaluate tangents at midpoints.
  SmExtent1d sEdgeIvl = pEdge->GetInterval();
  SmExtent1d sCurvIvl = pCurve->GetNaturalInterval();
  SmVector3d sEdgePtDer[2], sCurvPtDer[2];
  pCurve->Evaluate( sCurvIvl.Evaluate( 0.5 ), 1, TRUE, sCurvPtDer, TRUE );
  pEdge->GetCurve()->Evaluate( sEdgeIvl.Evaluate( 0.5 ), 1, TRUE, sEdgePtDer, TRUE );
  SmBoolean bRev = ( sCurvPtDer[1].Dot( sEdgePtDer[1] ) < 0.0 );

  SmVertex *pVtx = pEdge->GetStartVertex();
  double dParam, dDist, dGuessT = ( bRev ) ? sCurvIvl.GetMax() : sCurvIvl.GetMin();
  SmBoolean bOk;
  pCurve->DropPoint(sCurvIvl,         // in : target curve allowed domain
                    pVtx->GetPoint(), // in : Point to drop to curve
                    NULL,             // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                      //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                      //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                    dTol,             // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                      //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                      //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                      //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                    &dGuessT,         // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                    bOk,              // out: TRUE = found a drop point
                    dParam,           // out: found drop curve param
                    dDist) ;          // out: found drop distance
                                      // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                      //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                      //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                      //      default:[SM_SO_MINIMIZE] to preserve original behavior

  if ( ! bOk        ) { return FALSE; }
  if ( dDist > dTol ) { return FALSE; }

  // Got one point.
  SmExtent1d sCrvRange( dParam );

  pVtx = pEdge->GetOtherVertex( pVtx );
  dGuessT = ( bRev ) ? sCurvIvl.GetMin() : sCurvIvl.GetMax();
  pCurve->DropPoint(sCurvIvl,         // in : target curve allowed domain
                    pVtx->GetPoint(), // in : Point to drop to curve
                    NULL,             // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                      //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                      //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                    dTol,             // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                      //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                      //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                      //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                    &dGuessT,         // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                    bOk,              // out: TRUE = found a drop point
                    dParam,           // out: found drop curve param
                    dDist) ;          // out: found drop distance
                                      // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                      //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                      //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                      //      default:[SM_SO_MINIMIZE] to preserve original behavior

  if ( ! bOk )         { return FALSE; }
  if ( dDist > dTol )  { return FALSE; }

  // Got two points.
  sCrvRange.AddValue( dParam );

  // Finally, call the coincidence checker.
  // Actually, don't call it.  They don't do exactly what we want
  // and work very hard to do it.  We don't care about coincident
  // regions or anything; they're either the same or they aren't.
  // So just drop some interior points.
  ULONG ii;
  double dFrac;
  SmPoint3d sEdgePt;
  for ( ii = 0; ii < 4; ii++ )
  {
      dFrac = (ii + 1.0) / 5.0;
      pEdge->GetCurve()->EvaluatePoint( sEdgeIvl.Evaluate( dFrac ), sEdgePt );
      if ( bRev ) { dFrac = 1.0 - dFrac; }
      dGuessT = sCrvRange.Evaluate( dFrac );
      pCurve->DropPoint(sCurvIvl,  // in : target curve allowed domain
                        sEdgePt,   // in : Point to drop to curve
                        NULL,      // in : For Snapping, NULL=NoSnap, Vec pointing to CrvInside from desired EndPt for snapping.
                                   //      when DropPt is within dDistTol in cpVecToCrvInside dir of EndPt, Snap to Ivl EndPt.
                                   //      Vec handles ambiguities and makes sure that curves are not just touching at the ends.
                        dTol,      // in : Used for SnapDist, and SM_SO_INTERSECT sXSectTol3d distance.
                                   //      sXSectTol3d = dDistTol ;      SM_SO_INTERSECT 
                                   //      sXSectTol3d = SM_BIG_DOUBLE ; SM_SO_MINIMIZE, SM_SO_NORMALIZE
                                   //      sSnapTol3d  = dDistTol ;      SM_SO_MINIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                        &dGuessT,  // in : If given, call cheap LocalPointSolve() else call expensive GlobalPointSolve()
                        bOk,       // out: TRUE = found a drop point
                        dParam,    // out: found drop curve param
                        dDist) ;   // out: found drop distance
                                   // in : SM_SO_MINIMIZE = allow nonNormal drops near endPoints
                                   //      SM_SO_INTERSECT= like Minimize but point must be within dDistTol
                                   //      SM_SO_NORMALIZE= exclude nonNormal drops near endPoints
                                   //      default:[SM_SO_MINIMIZE] to preserve original behavior

      if ( ! bOk        )  { return FALSE; }
      if ( dDist > dTol )  { return FALSE; }
  }

  // Must be ok.
  return TRUE;

} // end static sm_CurveEdgeCoincidence
#endif


//----------------------------------------------------------------------
//
// End of the four static helper routines.
// Following are the six internal-use methods.
//
//----------------------------------------------------------------------

/*******************************************************************//**
PURPOSE: Imprint Faces in a Brep that are close to given vertices
   in their interiors.

NOTES: 
   Find any faces that are within tolerance of each vertex.
   If the vertex is in the interior of the face
   (not within tolerance of any boundary),
   imprint the vertex in the face (creating a new shell vertex),
   and glue the two vertices together.
   Return a list of the processed vertices -- the surviving one
   of each glued pair, which was the original.
   and the original list with the processed vertices removed.

INPUTS;
  rVerts : vertex list
  rFaces : face list

OUTPUTS;
  Vertex list has all glued verts set to NULL.
  rProcessedVerts has all the glued verts that were set to Null in vertex list.

  Side effect: nearby faces have vertices imprinted in them.
***********************************************************************/
SmStatus SmTopologyIntersector::VertexFaceIntersect
 (SmTArray< SmVertex* > & rVerts,               // i/o: entries set to Null when used.
  SmTArray< SmFace  * > & rFaces,               // in :
  SmTArray< SmVertex* > & rProcessedVerts,      // out:
  SmTArray< SmFace  * > & rProcessedFaces,      // out:
  double                & rdMaxVertexGap,       // out:
  double                & rdMinUnstitchedVertGap, // out:
                                                //  
  SmBoolean               bMakingManifoldSolid, // NotUsed: in :
  SmBoolean               bIgnoreProblems)      // in :
{
  SM_REF1(bMakingManifoldSolid) ;
  // Init outputs.
  rdMaxVertexGap = 0;
  rdMinUnstitchedVertGap = SM_BIG_DOUBLE;
  rProcessedVerts.ReSet();
  rProcessedFaces.ReSet();


  // Locals.
  ULONG lNumVerts = rVerts.GetSize();
  if ( lNumVerts < 1 ) { return SM_SUCCESS; }
  ULONG lNumFaces = rFaces.GetSize();
  if ( lNumFaces < 1 ) { return SM_SUCCESS; }

  ULONG ii, jj;
  SmVertex   *pV;
  SmVertex   *pNewVertex;
  double      dDistToFace;
  sReasonType eReason;
  SmStatus    eStat;

  // For each vertex in the list, look for close faces.
  for ( ii = 0; ii < lNumVerts; ii++ )
  {
      pV = rVerts[ii];
      if ( pV == NULL ) { continue; }

      //cbi // Do not do Manifold vertices if indicated.
      //cbi // We do allow Wire vertices though.
      //cbi if ( bMakingManifoldSolid  &&  ! pV->IsLaminaVertex() &&  ! pV->IsWireVertex() )
      //cbi   { continue; }

      SmPoint3d sPvPoint = pV->GetPoint();

      // Find all faces close to this vertex.
      SmExtent3d sVBBox( pV->GetPoint() );
      sVBBox.ExpandAbsolute( pV->GetTolerance() ); //cbi: was tol * 20.0
      for ( jj = 0; jj < lNumFaces; jj++ )
      {
          SmFace *pFace = rFaces[jj];
          if ( pFace == NULL ) { continue; }

          double dThisTol = smos_Max( (double)pV->GetTolerance(), m_dThisApproxTol3d );

          eStat = sm_TestAndImprintFace( pFace, sPvPoint, dThisTol, eReason,
                                            dDistToFace, pNewVertex );

          if ( eStat != SM_SUCCESS )
          {
              if ( bIgnoreProblems ) { continue;     }
              else                   { SER( eStat ); }
          }

          // when imprint created a new vertex
          if ( pNewVertex != NULL )
          {
              // Glue the new vertex to pV, and switch lists.
              SmBrep *pBrep = pV->GetBrep();
              if ( pBrep == NULL )
              {
                  if ( bIgnoreProblems ) { continue;     }
                  else                   { SER( eStat ); }
              }

              // First set the position of the surviving vertex to be
              // the midpoint of the two ... this should help to keep tolerances
              // from growing excessively.
              SmPoint3d sMidPt = 0.5 * ( pV->GetPoint() + pNewVertex->GetPoint() );
              pV->SetPoint( sMidPt );

              pBrep->GlueVertices( pV, pNewVertex );
              rProcessedVerts.AddUnique( pV );
              rVerts[ii] = pNewVertex = NULL;

              rProcessedFaces.AddUnique( pFace );

              // Set max distance.
              rdMaxVertexGap = smos_Max( rdMaxVertexGap, dDistToFace );

              SmTol::UpdateObjectTolerance( pV,    dDistToFace );
              SmTol::UpdateObjectTolerance( pFace, dDistToFace );
          }
          else
          {
              // Set min distance, if it didn't split due to being
              // too far away (as opposed to being at a vertex).
              if ( eReason == FAR_FROM_GEOM )
              {
                  rdMinUnstitchedVertGap = smos_Min ( rdMinUnstitchedVertGap, dDistToFace );
              }
          }

      } // end for all close edges to pV (jj)
  } // end loop on unprocessed vertices looking for close edges in Brep to split

  return SM_SUCCESS;

} // end ImprintFacesWithVertices

/*******************************************************************//**
PURPOSE: Intersect edges in both of their interiors,
   creating a vertex when found.

NOTES: 
   Return a list of the new vertices.

INPUTS:
  rEdges1, rEdges2      edge lists
***********************************************************************/
SmStatus SmTopologyIntersector::EdgeEdgeIntersect
 (SmTArray< SmEdge  * > & rInputEdges1,          // in : unchanged.
  SmTArray< SmEdge  * > & rInputEdges2,          // in : unchanged.
                                                 
  SmTArray< SmEdge  * > & rSplitEdges1,          // out:
  SmTArray< SmEdge  * > & rNewEdges1,            // out:
  SmTArray< SmEdge  * > & rSplitEdges2,          // out:
  SmTArray< SmEdge  * > & rNewEdges2,            // out:
  SmTArray< SmVertex* > & rNewVerts,             // out:
  double                & rdMaxEdgeGap,          // out:
  double                & rdMinUnstitchedEdgeGap,    // out:

  SmBoolean               bMakingManifoldSolid,  // NotUsed: in :
  SmBoolean               bIgnoreProblems)       // in :
{
  SM_REF1(bMakingManifoldSolid) ;
  // Init outputs.
  rdMaxEdgeGap = 0;
  rdMinUnstitchedEdgeGap = SM_BIG_DOUBLE;
  rSplitEdges1.ReSet();
  rNewEdges1.  ReSet();
  rSplitEdges2.ReSet();
  rNewEdges2.  ReSet();
  rNewVerts.   ReSet();

  // Locals.
  ULONG ii, jj, kk;
  SmEdge *pE1, *pE2;
  SmTArray< SmVertex *> sNewV;
  SmTArray< SmEdge   *> sNewE1;
  SmTArray< SmEdge   *> sNewE2;
  double      dDistToEdge;
  double      dMinUnStitch;
  SmStatus    eStat;

  ULONG lNumEdges1 = rInputEdges1.GetSize();
  if ( lNumEdges1 < 1 ) { return SM_SUCCESS; }
  ULONG lNumEdges2 = rInputEdges2.GetSize();
  if ( lNumEdges2 < 1 ) { return SM_SUCCESS; }

  // When we split an edge, we still have to continue processing both pieces.
  // Accomplish that by adding both pieces to the ends of the lists.
  // So work with copies of the input lists, so as not to modify them.
  SmTArray< SmEdge * > sEdges1 = rInputEdges1;
  SmTArray< SmEdge * > sEdges2 = rInputEdges2;


  // Double loop on the two edge lists.
  for ( ii = 0; ii < lNumEdges1; ii++ )
  {
      pE1 = sEdges1[ii];
      if ( pE1 == NULL ) { continue; }

//    // Do not do Manifold vertices if indicated.
//    // We do allow Wire vertices though.
//    if ( bMakingManifoldSolid  &&  ! pE1->IsLaminaVertex() &&  ! pE1->IsWireVertex() )
//      { continue; }

      SmExtent3d sE1Box;
      pE1->CalculateBoundingBox( &sE1Box );
      //cbi sVBBox.ExpandAbsolute( pE1->GetTolerance() );  //cbi was tol * 20
      for ( jj = 0; jj < lNumEdges2; jj++ )
      {
          pE2 = sEdges2[jj];
          if ( pE2 == NULL ) { continue; }

//        // Again, do only Lamina edges if indicated.
//        if ( bMakingManifoldSolid  &&  ! pE2->IsLamina() &&  ! pE2->IsWire() )
//          { continue; }

          SmExtent3d sE2Box;
          pE2->CalculateBoundingBox( &sE2Box );
          //cbi sVBBox.ExpandAbsolute( pE2->GetTolerance() );  //cbi was tol * 20

          double dThisTol = smos_3Max( m_dThisApproxTol3d,
                                       (double)pE1->GetTolerance(),
                                       (double)pE2->GetTolerance() );

          if ( sE1Box.AreDisjoint( sE2Box, dThisTol ) )
            { continue; }

          // We can also skip edges that share a vertex.
          SmVertex *pV11 = pE1->GetVertex();
          SmVertex *pV12 = pE1->GetOtherVertex( pV11 );
          SmVertex *pV21 = pE2->GetVertex();
          if ( pV21 == pV11 || pV21 == pV12 )
            { continue; }
          SmVertex *pV22 = pE2->GetOtherVertex( pV21 );
          if ( pV22 == pV11 || pV22 == pV12 )
            { continue; }

          // Do the intersection.
          eStat = sm_TestAndSplitTwoEdges( pE1, pE2, dThisTol, dDistToEdge, dMinUnStitch,
                                               sNewV, sNewE1, sNewE2 );

          if ( eStat != SM_SUCCESS )
          {
              if ( bIgnoreProblems ) { continue;     }
              else                   { SER( eStat ); }
          }

          ULONG lNumNewV = sNewV.GetSize();
          if ( lNumNewV < 1 )
            { continue; }

          // We did a split.

          rdMinUnstitchedEdgeGap = smos_Min ( rdMinUnstitchedEdgeGap, dMinUnStitch  );
          rdMaxEdgeGap       = smos_Max ( rdMaxEdgeGap,       dDistToEdge );

          // Add the vertices, split edge and the new edge to the output lists.
          
          rNewVerts.Append( sNewV );
          rNewEdges1.Append( sNewE1 );
          rNewEdges2.Append( sNewE2 );
          rSplitEdges1.AddUnique( pE1 );
          rSplitEdges2.AddUnique( pE2 );

          // We also have to add new edges to the lists we're working on.
          sEdges1.Add( pE1 );
          sEdges1.Append( sNewE1 );
          sEdges2.Add( pE2 );
          sEdges2.Append( sNewE2 );
          lNumEdges1 = sEdges1.GetSize();
          lNumEdges2 = sEdges2.GetSize();

          // Update tolerances.
          SmTol::UpdateObjectTolerance( pE1, dDistToEdge );
          SmTol::UpdateObjectTolerance( pE2, dDistToEdge );

          SmBoolean bCheckGaps = FALSE;
          if ( dDistToEdge > m_dThisApproxTol3d / 4 )
            { bCheckGaps = TRUE; }

          for ( kk = 0; kk < lNumNewV; kk++ )
            { SmTol::UpdateObjectTolerance( sNewV[kk], dDistToEdge, 0.0, bCheckGaps ); }
          ULONG lCount = sNewE1.GetSize();
          for ( kk = 0; kk < lCount; kk++ )
            { SmTol::UpdateObjectTolerance( sNewE1[kk], dDistToEdge, pE1->GetTolerance() ); }
          lCount = sNewE2.GetSize();
          for ( kk = 0; kk < lCount; kk++ )
            { SmTol::UpdateObjectTolerance( sNewE2[kk], dDistToEdge, pE2->GetTolerance() ); }

      } // end inner loop on rE2
  } // end outer loop on rE1

  return SM_SUCCESS;

} // end EdgeEdgeIntersect

/*******************************************************************//**
PURPOSE: Intersect Edges with Faces in a Brep in their interiors.

NOTES:
   For each input edge, intersect it with nearby faces.
   If the intersection is in the interior of both the edge and the face
   (not within tolerance of any boundary),
   split the edge by creating a new vertex,
   imprint the vertex in the face (creating a new shell vertex),
   and glue the two vertices together.
   Append the new vertex -- the surviving one of each glued pair --
   to the list of ProcessedVerts.

INPUTS:
  rEdges : edge list
  rFaces : face list

OUTPUTS:
  sNewVerts            : new vertices, in face, splitting edges.
  sSplitEdges          : originals that were split.
  sNewEdges            : new, created by split.
  sProcessedFaces      : from input list; they contain a new vertex.
  rdMaxVertexGap       : largest gap of a found intersection
  rdMinUnstitchedVertGap : smallest candidate intersection gap that was not stitched.
***********************************************************************/
SmStatus SmTopologyIntersector::EdgeFaceIntersect
 (SmTArray< SmEdge*   > & rEdges,                // in : unchanged
  SmTArray< SmFace*   > & rFaces,                // in : unchanged
  SmTArray< SmVertex* > & rNewVerts,             // out:
  SmTArray< SmEdge  * > & rSplitEdges,           // out:
  SmTArray< SmEdge  * > & rNewEdges,             // out:
  SmTArray< SmFace  * > & rProcessedFaces,       // out:
  double                & rdMaxVertexGap,        // out:
  double                & rdMinUnstitchedVertGap,  // out:
  SmBoolean               bMakingManifoldSolid,  // NotUsed: in :
  SmBoolean               bIgnoreProblems)       // in :
{
  SM_REF1(bMakingManifoldSolid) ;
  // Init outputs.
  rdMaxVertexGap = 0;
  rdMinUnstitchedVertGap = SM_BIG_DOUBLE;

  rNewVerts      .ReSet();
  rSplitEdges    .ReSet();
  rNewEdges      .ReSet();
  rProcessedFaces.ReSet();


  // Locals.
  ULONG lNumEdges = rEdges.GetSize();
  if ( lNumEdges < 1 ) { return SM_SUCCESS; }
  ULONG lNumFaces = rFaces.GetSize();
  if ( lNumFaces < 1 ) { return SM_SUCCESS; }

  ULONG ii, jj, kk, mm;
  SmEdge     *pEdge;
  SmFace     *pFace;
  SmEdge     *pNewEdge;
  sReasonType eReason = NO_REASON;
  SmStatus    eStat;
  SmSolutionArray sSolutions;

  // A complication: if more than one intersection, then after the
  // first split, the next intersection could be in either of the
  // split edges.  So we have to look at all of them.
  SmTArray< SmEdge* > sEdgesSplitFromThis;

  // But there's more: coincident faces.
  // If an edge is split by one, then the original algorithm would
  // put a vertex there, and the second face would not be seen.
  // So: collect all intersections first, then process them afterwards.
  // We'll collect the intersections in these arrays, which will
  // all be kept in sync:
  SmTArray< SmEdge *  > sEdges;
  SmTArray< SmFace *  > sFaces;
  SmTArray< double    > sEdgeParams;
  SmTArray< SmPoint2d > sFaceParams;

  // For each edge in the list, look for nearby faces.
  // Collect the info in the lists.
  for ( ii = 0; ii < lNumEdges; ii++ )
  {
      pEdge = rEdges[ii];
      SmZoneTol3d sEdgeZoneTol3d = SmTol::GetZoneTol3d(pEdge) ;

      if ( pEdge == NULL ) { continue; }
      SmCurve *pCurve = pEdge->GetCurve();
      SmExtent1d sEdgeDomain = pEdge->GetInterval();

      //cbi // Do not do Manifold vertices if indicated.
      //cbi // We do allow Wire vertices though.
      //cbi if ( bMakingManifoldSolid  &&  ! pEdge->IsLaminaVertex() &&  ! pEdge->IsWireVertex() )
      //cbi   { continue; }

      // We will skip faces and edges that are already connected.
      SmTArray< SmFace* > sThisEdgesFaces;
      pEdge->GetFaces( sThisEdgesFaces );

#if SM_DEBUG_CODE
      SmBoolean bDebugMe = FALSE;
      if ( bDebugMe )
      {
          if ( FALSE )
           { smgfx_Erase(); }
          smgfx_SetLook( 3,5, 1,0,0 ); pEdge->Draw(); sm_GraphicsLoop();
          sm_GraphicsLoop();
          for ( kk = 0; kk < sThisEdgesFaces.GetSize(); kk++ )
          {
              SmFace *pThisFace = sThisEdgesFaces[kk];
              smgfx_SetLook( 2,4, 0,0,0 ); pThisFace->DrawUV(4,4); sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
      }
#endif // SM_DEBUG_CODE

      // Find all input faces close to this edge.
      SmExtent3d sEBBox, sFBBox;
      pEdge->CalculateBoundingBox( & sEBBox );
      sEBBox.ExpandAbsolute( pEdge->GetTolerance() );  //cbi: was tol * 20.0
      for ( jj = 0; jj < lNumFaces; jj++ )
      {
          pFace = rFaces[ jj ];
          if ( pFace == NULL ) { continue; }
          SmZoneTol3d sFaceZoneTol3d = SmTol::GetZoneTol3d(pFace) ;

          // If face and edge are already connected, skip it.
          ULONG lIdx;
          if ( sThisEdgesFaces.FindElement( pFace, lIdx ) )
            { continue; }

          // BBox check.
          pFace->CalculateBoundingBox( sFBBox );
          if ( sEBBox.AreDisjoint( sFBBox, pFace->GetTolerance() ) )
            { continue; }

          double dThisTol = smos_Max( (double)pEdge->GetTolerance(), m_dThisApproxTol3d );

          SmSurface *pSurf = pFace->GetSurface();
          SmExtent2d sFaceDomain = pFace->GetUVDomain();

          // If possible, reduce the surface domain.
          // This can save a lot of work; e.g., two halves of a cylinder.
          SmExtent2d sTempDomain;
          eStat = pFace->CalculateUVDomainFromUVTrimCurves( sTempDomain );
          if ( eStat == SM_SUCCESS )
            { sFaceDomain = sTempDomain; }

//cbi maybe subroutine, as in sm_TestAndImprintFace():

          pSurf->GlobalCurveIntersect( sFaceDomain, *pCurve, sEdgeDomain, dThisTol,
                                       sSolutions );

          ULONG lNumSols = sSolutions.GetSize();

          // If the edge is coincident with the face, it will return several solutions.
          // That is not what we're looking for.
          if ( lNumSols > 3 )
            { continue; }

          for ( kk = 0; kk < lNumSols; kk++ )
          {
              SmSolution & rSol = sSolutions[kk];

              // Don't deal with range solutions.
              if ( rSol.m_eSolutionType != SM_ST_SINGLE_VALUE )
                { continue; }

              // Check interior to both Edge and Face.
              SmBoolean bInterior = TRUE;

              double dT = rSol.m_vStart[0];

              // Check interior of Edge
              eStat = sm_CheckInteriorEdge( pEdge, dT, dThisTol, bInterior );

              if ( bInterior == FALSE )
                { eReason = CLOSE_TO_BOUNDARY; }

              // Check the face (if this intersection is interior to the edge).
              // Note, it will presumably not be close to a vertex or an edge of the face.

              SmPoint2d sUV( rSol.m_vStart[1], rSol.m_vStart[2] );

              if ( bInterior )
              {
                  SmBoolean bIntWithBoundary = FALSE;
                  SmPointClassification sPointClass(sFaceZoneTol3d, &GetContext()) ;  // this tol is unlikely to be correct - check it

                  pFace->PointClassify( sUV, sEdgeZoneTol3d, bIntWithBoundary, TRUE, sPointClass );

                  if ( sPointClass.GetPointClass() == SM_PC_UNKNOWN )
                  {
                      bInterior = FALSE;
                      eReason = FAR_FROM_GEOM;
                  }
                  else if (   sPointClass.GetPointClass() == SM_PC_EDGE
                           || sPointClass.GetPointClass() == SM_PC_VERTEX )
                  {
                      bInterior = FALSE;
                      eReason = CLOSE_TO_BOUNDARY;
                  }
                  else if ( sPointClass.GetPointClass() != SM_PC_FACE )
                  {
                      SM_ASSERT( FALSE );
                      bInterior = FALSE;
                      eReason = CLOSE_TO_BOUNDARY; // most probable reason; don't really care too much.
                  }
              }

              if ( bInterior )
              {
                  // Intersection is interior to both edge and face.
                  // Save the intersection info.
                  sEdges.Add( pEdge );
                  sFaces.Add( pFace );
                  sEdgeParams.Add( dT );
                  sFaceParams.Add( sUV );
              }

          } // end for all intersection solutions (kk)

      } // end for all nearby faces to pEdge (jj)
  } // end loop on input edges looking for nearby faces in Brep to intersect (ii)


  // Now process each intersection found.

  ULONG lNumInts = sEdges.GetSize();

  for ( ii = 0; ii < lNumInts; ii++ )
  {
      pEdge = sEdges[ii];
      pFace = sFaces[ii];
      double dParam = sEdgeParams[ii];
      SmPoint2d sUV = sFaceParams[ii];

      SmCurve  *pCurve = pEdge->GetCurve();
      SmSurface *pSurf = pFace->GetSurface();
      SmBrep    *pBrep = pFace->GetBrep();

#if SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
if ( bDebugMe2 )
{
  if ( FALSE )
   { smgfx_Erase(); }
  smgfx_SetLook( 1,2, 0,0,1 ); pFace->DrawUV(4,4);                sm_GraphicsLoop();
  smgfx_SetLook( 2,4, 1,0,1 ); pFace->GetSurface()->DrawAt(sUV);  sm_GraphicsLoop();
  smgfx_SetLook( 1,2, 0,1,0 ); pEdge->Draw();                     sm_GraphicsLoop();
  smgfx_SetLook( 2,6, 1,0,0 ); pEdge->GetCurve()->DrawAt(dParam); sm_GraphicsLoop();
  sm_GraphicsLoop();
}
#endif // SM_DEBUG_CODE


      // Find the point -- make sure they're the same. Can err if pEdge was edited by prior loop iteration
      SmPoint3d sEdgePt, sFacePt;
      pCurve->EvaluatePoint( dParam, sEdgePt );
      pSurf ->EvaluatePoint( sUV,    sFacePt );

      double dThisTol = smos_Max( (double)pFace->GetTolerance(), m_dThisApproxTol3d );
      dThisTol = smos_Max( (double)pEdge->GetTolerance(), dThisTol );
      double dThisDeviation = sEdgePt.DistanceBetween( sFacePt );

      if ( dThisDeviation > dThisTol )
      {
          // Try to find the edge that had the intersection in rNewEdges
          // SmBoolean bFoundEdge = FALSE;
          for ( jj = 0; jj < rNewEdges.GetSize(); jj++ )
          {
              SmEdge *pThisEdge = rNewEdges[jj];
              pCurve = pThisEdge->GetCurve();
              pCurve->EvaluatePoint( dParam, sEdgePt );

              double dNewDeviation = sEdgePt.DistanceBetween( sFacePt );

              if ( dNewDeviation < dThisDeviation )
              {
                  dThisDeviation = dNewDeviation;
                  pEdge = pThisEdge;
              }
          }
      }

      // Now do all the topology:
      // 1: create a new vtx in the Face
      // 2: chk each edge for param being interior
      // 3: if interior,
      //      split edge
      //      add new edge to list -- get new vertex
      //    else
      //      get vertex
      // 4: glue verts.

      sEdgesSplitFromThis.ReSet();
      sEdgesSplitFromThis.Add( pEdge );

      SmVertex *pNewEdgeVtx = NULL;
      SmVertex *pNewFaceVtx = NULL;

      // Create a new Vertex in the Face.
      SmLoop *pNewLoop = NULL;
      eStat = pBrep->MakeVertexLoop( pFace, sFacePt, pNewLoop, pNewFaceVtx );
      if ( eStat != SM_SUCCESS) {
          SM_ASSERT( FALSE );
          if ( bIgnoreProblems ) { continue;     }
          else                   { SER( eStat ); }
      }

      rProcessedFaces.AddUnique( pFace );

      SmBoolean bCheckGaps = FALSE;
      if ( dThisDeviation > dThisTol/4.0 )
        { bCheckGaps = TRUE; }
      SmTol::UpdateObjectTolerance( pNewFaceVtx, dThisDeviation, 0.0, bCheckGaps );
      SmTol::UpdateObjectTolerance( pFace,       dThisDeviation );


      // Check interior of Edge -- all edges in sEdgesSplitFromThis.
      SmBoolean bInterior = FALSE;
      SmEdge *pThisSplitEdge = pEdge;
      ULONG   lNumSplitEdges = sEdgesSplitFromThis.GetSize();
      for ( mm = 0; mm < lNumSplitEdges; mm++ )
      {
          pThisSplitEdge = sEdgesSplitFromThis[mm];
          eStat = sm_CheckInteriorEdge( pThisSplitEdge, dParam, dThisTol, bInterior );
          if ( bInterior )
            { break; }
      }

      if ( bInterior )
      {
          // Intersection is interior to pThisSplitEdge.
          // Split the edge with a new vertex.

          SmEdge *pE1 = NULL, *pE2 = NULL;
          eStat = pThisSplitEdge->GetBrep()->MakeVertexSplitEdge( pThisSplitEdge, dParam, pE1, pE2, pNewEdgeVtx );
          if ( eStat != SM_SUCCESS) {
              SM_ASSERT( FALSE );
              if ( bIgnoreProblems ) { continue;     }
              else                   { SER( eStat ); }
          }

          rNewVerts.Add( pNewEdgeVtx ); // We'll keep this vertex.

          pNewEdge = ( pThisSplitEdge == pE1 ) ? pE2 : pE1;

          // Add to the output lists.
          rSplitEdges.AddUnique( pEdge ); // pEdge: only input edges on this list.
          rNewEdges.AddUnique( pNewEdge );

          // Also add it to our working edge list.
          sEdgesSplitFromThis.Add( pNewEdge );

          SmTol::UpdateObjectTolerance( pE1,         dThisDeviation, pThisSplitEdge->GetTolerance() );
          SmTol::UpdateObjectTolerance( pE2,         dThisDeviation, pThisSplitEdge->GetTolerance() );
          SmTol::UpdateObjectTolerance( pNewEdgeVtx, dThisDeviation, 0.0, bCheckGaps );
      }
      else
      {
          // Hit an existing vertex.  (Probably coincident Faces.)
          // Glue that vertex to the Face Vertex we just created.
          // Find that vertex: search params of boundaries of Edges in sEdgesSplitFromThis list.
          pNewEdgeVtx = NULL;

          ULONG lNumSplit = sEdgesSplitFromThis.GetSize();
          for ( jj = 0; jj < lNumSplit; jj++ )
          {
              SmEdge *pThisE = sEdgesSplitFromThis[ jj ];
              SmVertex *pVtx = pThisE->GetStartVertex();
              if ( pVtx->GetPoint().CloserThan( dThisTol, pNewFaceVtx->GetPoint() ) )

              //cbi SmExtent1d sDom = pThisE->GetInterval();
              //cbi if ( smos_Fabs( dParam - sDom.GetMin() ) <= dThisTol )
              {
                  pNewEdgeVtx = pVtx;  //cbi pThisE->GetStartVertex();
                  break;
              }

              // Check top of domain for last piece.
              if ( jj == lNumSplit-1 )
              {
                  pVtx = pThisE->GetOtherVertex( pVtx );
                  if ( pVtx->GetPoint().CloserThan( dThisTol, pNewFaceVtx->GetPoint() ) )

                  //cbi if ( smos_Fabs( dParam - sDom.GetMax() ) <= dThisTol )
                  {
                      //cbi SmVertex *pTmp = pThisE->GetStartVertex();
                      pNewEdgeVtx = pVtx;  //cbi pThisE->GetOtherVertex( pTmp );
                      break;
                  }
              }
          } // end for each edge split from pEdge
      } // end if-else edge param was interior

      // State: we have pNewFaceVtx and pNewEdgeVtx.
      // Glue them together (if indeed they both exist).
      if ( pNewEdgeVtx != NULL  &&  pNewFaceVtx != NULL )
      {
          // Glue the new vertices together, and add to processed list.

          // First set the position of the surviving vertex to be
          // the midpoint of the two ... this should help to keep tolerances
          // from growing excessively.
          SmPoint3d sMidPt = 0.5 * ( pNewEdgeVtx->GetPoint() + pNewFaceVtx->GetPoint() );
          pNewEdgeVtx->SetPoint( sMidPt );

          pBrep->GlueVertices( pNewEdgeVtx, pNewFaceVtx );

          // Set max distance.
          rdMaxVertexGap = smos_Max( rdMaxVertexGap, dThisDeviation );

          SmTol::UpdateObjectTolerance( pNewEdgeVtx, dThisDeviation, 0.0, bCheckGaps );
      }
      else
      {
          // Set min distance, if it didn't split due to being
          // too far away (as opposed to being at a vertex).
          if ( eReason == FAR_FROM_GEOM )
          {
              rdMinUnstitchedVertGap = smos_Min( rdMinUnstitchedVertGap, dThisDeviation );
          }
      }

  } // end for all intersection found (ii)

  return SM_SUCCESS;

} // end EdgeFaceIntersect


/*******************************************************************//**
PURPOSE: Look for edges, connecting given vertices, and not in the exclude list,
  that are coincident with a face, and embed such edges in those faces.

  Side effect: coincident edges incident on the given vertices are embedded in faces.

NOTES:
   Find any edges, both of whose vertices are in the input list.
   If the edge is coincident with any face, and is not in the exclude list,
   embed it in the face.

   In SmStitch::DoStitching, they keep track of Faces that have any of
   their edges modified, and CreateUVTrimCurves() on them.
   This routine does not do that.
   If desired, that could be done by creating trim curves on
   all Faces connected to the edges returned in rGluedEdges.

OUTPUTS:
  sNewFaces and sSplitFaces are in sync: sNewFaces[i] was split
  off from sSplitFaces[i].
***********************************************************************/

SmStatus SmTopologyIntersector::EdgeFaceCoincidence(
      const SmTArray< SmVertex* > & rGivenVerts,     // in
      const SmTArray< SmEdge  * > & rExcludedEdges,  // in:  don't try to imprint these.
            SmTArray< SmEdge  * > & rImprintedEdges, // out: edges imprinted into a face.
            SmTArray< SmFace  * > & rProcessedFaces, // out: faces with edges imprinted in them, including new faces.
            SmTArray< SmFace  * > & rNewFaces,       // out: if an imprinted edge split a face.
            SmTArray< SmFace  * > & rSplitFaces,     // out: what sNewFaces were split from.
            double & dMaxEdgeGap,
            double & dThisMinUnstitched,
            SmBoolean bIgnoreProblems,
            SmBoolean bDoRegionNesting
         )
{
  // The edges must have (both) vertices lying in a face of the other.
  // Also, don't check against a face that already contains the edge.
  // So: for each input vertex,
  //       for each of its edges,
  //         if edge is not excluded, and its other vertex is processed,
  //            for each face that contains both vertices,
  //               if the face doesn't contain the edge,
  //                  check for edge/face coincidence

  // init outputs   //cbi TODO: set these.
  dMaxEdgeGap = 0;
  dThisMinUnstitched = SM_BIG_DOUBLE;
  rImprintedEdges.ReSet();
  rProcessedFaces.ReSet();
  rNewFaces      .ReSet();
  rSplitFaces    .ReSet();

  // Locals
  ULONG ii, jj, kk;

  SmTArray< SmEdge* > sEdgesOfVtx;

  ULONG lNumGivenVerts = rGivenVerts.GetSize();
  for ( ii = 0; ii < lNumGivenVerts; ii++ )
  {
      SmVertex *pV = rGivenVerts[ii];
      if ( pV == NULL ) { continue; }

      pV->GetEdges( sEdgesOfVtx );
      ULONG lNumEdges = sEdgesOfVtx.GetSize();
      for ( jj = 0; jj < lNumEdges; jj++ )
      {
          SmEdge *pE = sEdgesOfVtx[jj];
          if ( pE == NULL ) { continue; }

          // Can skip edges that have been glued.
          ULONG lIdx;
          if ( rExcludedEdges.FindElement( pE, lIdx ) )
            { continue; }

          SmVertex *pOtherV = pE->GetOtherVertex( pV );

          if ( ! rGivenVerts.FindElement( pOtherV, lIdx ) )
            { continue; }

          // Ok, we have an unprocessed edge, with both vertices processed.
          // Find all faces that contain both vertices.
          SmTArray< SmFace* > sFaces1, sFaces2, sCommonFaces, sEdgeFaces;
          pV->GetFaces( sFaces1 );
          pOtherV->GetFaces( sFaces2 );
          sFaces1.FindCommonElements( sFaces2, sCommonFaces );

          pE->GetFaces( sEdgeFaces );

          ULONG lNumCommonFaces = sCommonFaces.GetSize();
          for ( kk = 0; kk < lNumCommonFaces; kk++ )
          {
              SmFace *pF = sCommonFaces[ kk ];

              if ( sEdgeFaces.FindElement( pF, lIdx ) )
                { continue; }

              // Ok, all the criteria are satisfied.
              // Check for coincidence of pE and pF.

              SmCurve   * pCurve         = pE->GetCurve();
              SmSurface * pSurf          = pF->GetSurface();
              SmExtent1d  sEdgeDom       = pE->GetInterval();
              SmExtent2d  sFaceDom       = pF->GetUVDomain();
              SmZoneTol3d sEdgeZoneTol3d = SmTol::GetZoneTol3d(pE) ; 

//cbi: maybe do this ahead of time, for all added faces:
              pF->CalculateUVDomainFromUVTrimCurves( sFaceDom );
              SmBoolean bIsCoin, bPartialCoinOk = FALSE;
              SmSolution sSol;
              pSurf->SimpleCoincidenceChecker(
                      sFaceDom, *pCurve, sEdgeDom, m_dThisApproxTol3d,
                      bIsCoin, sSol, bPartialCoinOk
              );

              if ( ! bIsCoin || sSol.m_vStart.m_dSolutionValue > m_dThisApproxTol3d )
                { continue; }

              // It's possible that the curve is in the surface but not in
              // the Face -- there's often lots of area inside the Face domain
              // rectangle that is not within the face boundaries.
              // Test two points.
              SmPointClassification sPtClass(sEdgeZoneTol3d, &GetContext()) ; // this tol is unlikely to be correct - check it
              SmPoint3d sCrvPt;
              double dTestT = 0.38;
              pCurve->EvaluatePoint( sEdgeDom.Evaluate( dTestT ), sCrvPt );

              // Allow any part of the Face: including Edges and Vertices.
              pF->Point3DClassify( sCrvPt, sEdgeZoneTol3d, FALSE, sPtClass );
              if (    sPtClass.GetPointClass() != SM_PC_FACE
                   && sPtClass.GetPointClass() != SM_PC_EDGE
                   && sPtClass.GetPointClass() != SM_PC_VERTEX )
                { continue; }
#ifdef SM_USE_NEWTOL
              SM_NEWTOL_LINE if ( sPtClass.GetGap3d() > m_dThisApproxTol3d )
              SM_NEWTOL_LINE   { continue; }
#else // SM_USE_OLDTOL
              SM_OLDTOL_LINE if ( sPtClass.GetDeviation() > m_dThisApproxTol3d )
              SM_OLDTOL_LINE   { continue; }
#endif // SM_USE_OLDTOL

              dTestT = 0.71;
              pCurve->EvaluatePoint( sEdgeDom.Evaluate( dTestT ), sCrvPt );
              pF->Point3DClassify( sCrvPt, m_dThisApproxTol3d, FALSE, sPtClass );
              if (    sPtClass.GetPointClass() != SM_PC_FACE
                   && sPtClass.GetPointClass() != SM_PC_EDGE
                   && sPtClass.GetPointClass() != SM_PC_VERTEX )
                { continue; }
#ifdef SM_USE_NEWTOL
              SM_NEWTOL_LINE if ( sPtClass.GetGap3d() > m_dThisApproxTol3d )
              SM_NEWTOL_LINE   { continue; }
#else // SM_USE_OLDTOL
              SM_OLDTOL_LINE if ( sPtClass.GetDeviation() > m_dThisApproxTol3d )
              SM_OLDTOL_LINE   { continue; }
#endif // SM_USE_OLDTOL

//cbi subroutine for this too?:

              // Merge this edge into this face.
              // Determine the orientation: make it go from pV to pOtherV.
              // The orienatation is whether the curve goes from pV to pOtherV.
              SmPoint3d sPt, sVtxPt = pV->GetPoint();
              pCurve->EvaluatePoint( sEdgeDom.GetMin(), sPt );
              double       dDist1  = sVtxPt.DistanceBetween( sPt );
              pCurve->EvaluatePoint( sEdgeDom.GetMax(), sPt );
              double       dDist2  = sVtxPt.DistanceBetween( sPt );
              SmOrientType eOrient = ( dDist1 < dDist2 ) ? SM_OT_SAME : SM_OT_OPPOSITE;

              double dEdgeTol = pE->GetTolerance();

              SmEdge *pNewEdge = NULL;
              SmLoop *pNewLoop = NULL;
              SmFace *pNewFace = NULL;


              SmBrep *pBrep = pF->GetBrep();

              // We have to make a copy of the curve: what's passed in
              // gets installed in the new edge.
              SmCurve *pCurveCopy = NULL;
              pCurve->Copy( *(pBrep->GetContext()), pCurveCopy );

              pBrep->MakeEdgeInFace(
                  pF, pV, pOtherV, pCurveCopy, NULL, sEdgeDom, eOrient, dEdgeTol,
                  pNewEdge, pNewLoop, pNewFace
              );

//#ifdef SM_DEBUG_CODE
//              sbChkEdgeCrvOwner( sEdgesOfVtx );
//              sbChkEdgeCrvOwner( pBrep );
//#endif // SM_DEBUG_CODE

              rProcessedFaces.AddUnique( pF );

              double dDev = sSol.m_vStart.m_dSolutionValue;

              if ( pNewFace != NULL )
              {
                  rNewFaces.      Add( pNewFace );
                  rProcessedFaces.Add( pNewFace );
                  rSplitFaces.    Add( pF );

                  SmTol::UpdateObjectTolerance( pNewFace, dDev, pF->GetTolerance() );
              }

              // A new edge would be glued to the edge it was made from.
              if ( pNewEdge != NULL )
              {
                  SmTol::UpdateObjectTolerance( pNewEdge, dDev, dEdgeTol );
                  SmTol::UpdateObjectTolerance( pF,       dDev, dEdgeTol );  // We modified this.

                  // Note: GlueEdgesGeneral() will delete pNewEdge,
                  // along with its curve.  But MakeEdgeInFace() just copied
                  // pCurve into pNewEdge, so right now, pE and pNewEdge
                  // both point to pCurve, and not as SmCEdge's.
                  // So right now, that's an invalid situation.
                  // But, the Edge destructor won't delete the curve
                  // if the curve's owner is not the edge.
                  // So we set pCurve's owner back where it was.
                  if ( pCurve->GetOwner() == pNewEdge )
                  {
                      pCurve->SetOwner( pE );
                  }

                  // glue pE/pNewE edge pair - keep the original edge, pE
                  SmOrientType eRelOrient   = SM_OT_SAME; // same curve.
                  double       dDistToCurve = 0.0; // same curve.
                  if ( pBrep->GlueEdgesGeneral( pE, eRelOrient, dDistToCurve,
                         bDoRegionNesting, pNewEdge ) == SM_SUCCESS )
                  {
                      rImprintedEdges.Add( pE );
                      //cbi no:  rnNumEdgesStitched++;
                      pNewEdge = NULL;

//#ifdef SM_DEBUG_CODE
//                      sbChkEdgeCrvOwner( rImprintedEdges );
//                      sbChkEdgeCrvOwner( pBrep );
//#endif // SM_DEBUG_CODE

                  }
                  else if ( ! bIgnoreProblems )
                  {
                      SER(SM_ERR);
                  }
              }
          }
      }
  }
  return SM_SUCCESS;

} // end EdgeFaceCoincidence


/*******************************************************************//**
PURPOSE: For each vertex in the given list, see whether any pairs
  of incident faces should be intersected.

  Side effect: intersecting faces are intersected, and an edge created.

USAGE NOTES:
  We are also given two lists of faces: we do not attempt to intersect
  two faces that are in the same list.  (The idea being that one list is
  from one Brep, and the other from another Brep, and those Breps do not
  need their own processing; we do only intersections between different
  Breps.)  That is checked for in a subroutine that we call, so we just
  call that subroutine on every pair, and pass it the lists.

  We have to check pairs of vertices, and also single vertices for closed
  intersection curves, such as a cylinder and a plane.
  For each Face in List1
    For each Face in List2
      if the faces share 2 vertices or either face has only 1 vertex
        Intersect the faces and imprint the curves as manifold edges

  This method is not designed to handle coincident faces. For faces that
  can cover another no edges are created.

  Rewritten in Revision 8887. Helper function sm_IntersectFaces removed.
***********************************************************************/
SmStatus SmTopologyIntersector::FaceFaceIntersect
(SmTArray< SmVertex * > & rInputVerts,      // in :
 SmTArray< SmFace   * > & rFaceList1,       // i/o:
 SmTArray< SmFace   * > & rFaceList2,       // i/o:
 SmTArray< SmFace   * > & rProcessedFaces,  // out:
 SmTArray< SmFace   * > & rNewFaces,        // out: if faces were split
 SmTArray< SmEdge   * > & rNewEdges,        // out:
 double                 & rdMaxGap,         // NotUsed: out:
 double                 & rdMinUnstitched,      // NotUsed: out:
 SmBoolean                bIgnoreProblems,  // NotUsed: in :  not used in this routine.
 SmBoolean                bDoRegionNesting) // in :
{
  SM_REF1(bIgnoreProblems) ;
  // Init outputs
  rProcessedFaces.ReSet();
  rNewFaces.ReSet();
  rNewEdges.ReSet();
  rdMaxGap = 0;
  rdMinUnstitched = SM_BIG_DOUBLE;

  // Locals
  ULONG ii, jj, kk, ll, mm, nn;
  SmTArray< SmFace* > sFaces1 = rFaceList1, sFaces2 = rFaceList2, sCommonFaces;

#ifdef SM_DEBUG_CODE
  SmBoolean bDebugMe = FALSE;
  if (bDebugMe) {
      SmBrep *pBrep = (rFaceList1.GetSize() > 0) ? rFaceList1[0]->GetBrep() : NULL;
      if (pBrep == NULL && rFaceList2.GetSize() > 0)
      {
          pBrep = rFaceList2[0]->GetBrep();
      }
      if (bDebugMe) {
          if (pBrep) {
              smgfx_Erase();
              smgfx_SetLook(1, 2, 0, 0, 0); pBrep->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
      }
  }
#endif // SM_DEBUG_CODE

  // Compare each face in rFaceList1 with each face in rFaceList2
  // Filtering out those that don't share multiple vertices
  for ( ii = 0; ii < sFaces1.GetSize(); ii++ )
  {
      // Loop locals
      SmFace    *pFace1 = sFaces1[ii]; 
      SmSurface *pSurf1 = pFace1->GetSurface();
      SmExtent2d sUV1   = pFace1->GetUVDomain();
      SmTArray<SmVertex*> sV1s;
      pFace1->GetVertices( sV1s );
      SmBoolean bIntersectF1 = pSurf1->IsClosed(sUV1, SM_SP_U) || pSurf1->IsClosed(sUV1, SM_SP_V);
      
      for ( jj = 0; jj < sFaces2.GetSize(); jj++ )
      {
          // Loop locals
          SmFace    *pFace2 = sFaces2[jj];
          SmSurface *pSurf2 = pFace2->GetSurface();
          SmExtent2d sUV2   = pFace2->GetUVDomain();
          SmTArray<SmVertex*> sV2s, sCommonVs;
          pFace2->GetVertices( sV2s );
          SmBoolean bIntersectF2 = pSurf2->IsClosed(sUV2, SM_SP_U) || pSurf2->IsClosed(sUV2, SM_SP_V);

          // Do these 2 faces share 2 or more common vertices?
          sV1s.FindCommonElements( sV2s, sCommonVs );
          sCommonVs.FindCommonElements( rInputVerts, sCommonVs );
          SmBoolean bShareInputVs = sCommonVs.GetSize() > 1;

          // Only intersect faces that share 2 rInputVerts or have only 1 vertex
          if ( !bIntersectF1 && !bIntersectF2 && !bShareInputVs )
          { continue; }

          // We do not handle coincident edges here
          SmBoolean bAreCoincident = FALSE;
          SmTArray< SmEdge* > sFace1Edges, sFace2Edges, sCommonEdges;
          pFace1->GetEdges( sFace1Edges );
          pFace2->GetEdges( sFace2Edges );
          sFace1Edges.FindCommonElements( sFace2Edges, sCommonEdges );

          // In the SmMerge::MergeAddedTopology routine, by now all coincident edges should have common edges
          if ( sCommonEdges.GetSize() > 0 )
          {
              double dMaxDist = 0.0;
              SmSurface *pSCover = NULL;

              pFace1->GetSurface()->CoincidenceCheck( *pFace2->GetSurface(), m_dThisAngTolRad, bAreCoincident, dMaxDist, FALSE, pSCover );
              if ( bAreCoincident )
              { continue; }
          }

          // If we arrive here, we are ready to intersect the faces
          SmTArray<SmCurve*> s3DCurves, sCC3DCrvs, sCCUVCrvs1, sCCUVCrvs2;
          SmTArray<SmCurveClassification*> siiCC, sjjCC;
          SmObjsDelete<SmCurve*> sClean3Ds( &s3DCurves ), sCleanCC3D(&sCC3DCrvs), sCleanUV1( &sCCUVCrvs1), sCleanUV2( &sCCUVCrvs2);
          SmObjsDelete<SmCurveClassification*> sClean0( &siiCC ), sClean1( &sjjCC);
          double dAngTolDeg = 180. / SM_PI * m_dThisAngTolRad;
          pFace1->FaceIntersect( m_crContext, pFace2, m_dThisApproxTol3d, dAngTolDeg, s3DCurves, NULL, NULL,
                                 NULL, &siiCC, &sjjCC, &sCC3DCrvs, &sCCUVCrvs1, &sCCUVCrvs2, &bAreCoincident );

          // In this loop we imprint each intersection as an edge
          // Within the SmMerge::MergeAddedTopology sequence all faces arrive here with vertices in place,
          // so we expect any intersection will classify at mid to both faces and at ends to the same vertex pair
          // This also implies that the curve classifications and intervals will be the same size for each face
          for ( kk = 0; kk < siiCC.GetSize(); kk++ )
          {
              SmCurveClassification *pCC1 = siiCC[kk], *pCC2 = sjjCC[kk];
              pCC1->SetPointClassParameters( TRUE );
              pCC2->SetPointClassParameters( TRUE );

              // For each interval in the classification
              for ( ll = 0; ll < pCC1->GetSize(); ll++ )
              {
                  SmCurveInterval * pCI1 = &( *pCC1 )[ll];
                  SmCurveInterval * pCI2 = &( *pCC2 )[ll];
                  SmEdge *pNewEdge1 = NULL, *pNewEdge2 = NULL;

                  // Have to imprint edges on faces
                  if ( pCI1->m_vMid.GetPointClass() != SM_PC_FACE || pCI2->m_vMid.GetPointClass() != SM_PC_FACE )
                  { continue; }

                  if ( pCI1->m_vStart.GetPointClass() == SM_PC_FACE && pCI2->m_vStart.GetPointClass() == SM_PC_FACE )
                  {
                      // Locals
                      SmPoint3d sPt;
                      SmFace   *pF1 = pCI1->m_vStart.GetFaceObject();
                      SmFace   *pF2 = pCI2->m_vStart.GetFaceObject();
                      pCI1->m_vStart.FindObjPoint3d( sPt );

                      // This classification may be outdated. Check and update. Will either be face or vertex
                      SmVector2d pCI1_start_uv = pCI1->m_vStart.GetUVParam();
                      SmVector2d pCI2_start_uv = pCI2->m_vStart.GetUVParam(); 
                      pF1->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI1->m_vStart, &pCI1_start_uv );
                      pF2->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI2->m_vStart, &pCI2_start_uv );

                      // The CC may be unknow when a new face was created prior. m_vMid was updated, so check that face
                      if ( pCI1->m_vStart.GetPointClass() == SM_PC_UNKNOWN )
                      {
                          pF1 = pCI1->m_vMid.GetFaceObject();
                          pF1->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI1->m_vStart, &pCI1_start_uv );
                      }
                      if ( pCI2->m_vStart.GetPointClass() == SM_PC_UNKNOWN )
                      {
                          pF2 = pCI2->m_vMid.GetFaceObject();
                          pF2->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI2->m_vStart, &pCI2_start_uv );
                      }

                      // We have intersecting 3dcurves. Imprint the point as a vertex on each face.
                      // Glue the vertices, then keep going
                      if ( pCI1->m_vStart.GetPointClass() == SM_PC_FACE )
                      {
                          SmLoop   *pLoop = NULL;
                          SmVertex *pV1 = NULL, *pV2 = NULL;
                          m_pBrep->MakeVertexLoop( pF1, sPt, pLoop, pV1 );

                          pCI2->m_vStart.FindObjPoint3d( sPt );
                          m_pBrep->MakeVertexLoop( pF2, sPt, pLoop, pV2 );

                          m_pBrep->GlueVertices( pV1, pV2 );

                          // Update classifications
                          pCI1_start_uv = pCI1->m_vStart.GetUVParam();
                          pCI2_start_uv = pCI2->m_vStart.GetUVParam();
                          pF1->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI1->m_vStart, &pCI1_start_uv );
                          pF2->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI2->m_vStart, &pCI2_start_uv );
                      }
                  }
                  else if ( pCI1->m_vStart.GetPointClass() != SM_PC_VERTEX || pCI2->m_vStart.GetPointClass() != SM_PC_VERTEX )
                  {
                      SE_MSG( SM_ERR, _T( "Missing vertices should have been imprinted already" ) );
                      continue;
                  }

                  if ( pCI1->m_vEnd.GetPointClass() == SM_PC_FACE && pCI2->m_vEnd.GetPointClass() == SM_PC_FACE )
                  {
                      // Locals
                      SmPoint3d sPt;
                      SmFace   *pF1 = pCI1->m_vEnd.GetFaceObject();
                      SmFace   *pF2 = pCI2->m_vEnd.GetFaceObject();
                      pCI1->m_vEnd.FindObjPoint3d( sPt );

                      // This classification may be outdated. Check and update. Will either be face or vertex
                      SmVector2d pCI1_end_uv = pCI1->m_vEnd.GetUVParam();
                      SmVector2d pCI2_end_uv = pCI2->m_vEnd.GetUVParam();
                      pF1->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI1->m_vEnd, &pCI1_end_uv );
                      pF2->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI2->m_vEnd, &pCI2_end_uv );

                      // The CC may be unknow when a new face was created prior. m_vMid was updated, so check that face
                      if ( pCI1->m_vEnd.GetPointClass() == SM_PC_UNKNOWN )
                      {
                          pF1 = pCI1->m_vMid.GetFaceObject();
                          pF1->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI1->m_vEnd, &pCI1_end_uv );
                      }
                      if ( pCI2->m_vEnd.GetPointClass() == SM_PC_UNKNOWN )
                      {
                          pF2 = pCI2->m_vMid.GetFaceObject();
                          pF2->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI2->m_vEnd, &pCI2_end_uv );
                      }

                      // We have intersecting 3dcurves. Imprint the point as a vertex on each face.
                      // Glue the vertices, then keep going
                      if ( pCI1->m_vEnd.GetPointClass() == SM_PC_FACE )
                      {
                          SmLoop   *pLoop = NULL;
                          SmVertex *pV1 = NULL, *pV2 = NULL;
                          m_pBrep->MakeVertexLoop( pF1, sPt, pLoop, pV1 );

                          pCI2->m_vEnd.FindObjPoint3d( sPt );
                          m_pBrep->MakeVertexLoop( pF2, sPt, pLoop, pV2 );

                          m_pBrep->GlueVertices( pV1, pV2 );

                          // Update classifications
                          pCI1_end_uv = pCI1->m_vEnd.GetUVParam();
                          pCI2_end_uv = pCI2->m_vEnd.GetUVParam();
                          pF1->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI1->m_vEnd, &pCI1_end_uv );
                          pF2->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI2->m_vEnd, &pCI2_end_uv );
                      }
                  }
                  else if ( pCI1->m_vEnd.GetPointClass() != SM_PC_VERTEX || pCI2->m_vEnd.GetPointClass() != SM_PC_VERTEX )
                  {
                      SE_MSG( SM_ERR, _T( "Missing vertices should have been imprinted already" ) );
                      continue;
                  }

                  // Make an edge in the first face
                  {
                      SmVertex *pV1 = pCI1->m_vStart.GetVertexObject();
                      SmVertex *pV2 = pCI1->m_vEnd.GetVertexObject();
                      const SmCurve *p3dCurve = pCC1->GetCurve();
                      SmCurve *pCopyC = NULL;
                      p3dCurve->Copy( m_crContext, pCopyC );
                      SmExtent1d sIvl = pCI1->m_vInterval;

                      SmLoop *pNewLoop = NULL;
                      SmFace *pNewFace = NULL, *pOldFace = pCI1->m_vMid.GetFaceObject();

                      m_pBrep->MakeEdgeInFace( pOldFace, pV1, pV2, pCopyC, NULL, sIvl, SM_OT_SAME, m_dThisApproxTol3d, pNewEdge1, pNewLoop, pNewFace );

                      // If the face was split, update classifications and add new face to intersection list
                      if ( pNewFace )
                      {
                          // Classification update: if the interval m_vMid classifies to a face, does it classify to the new face?
                          for ( mm = kk; mm < siiCC.GetSize(); mm++ )
                          {
                              SmCurveClassification *pCC = siiCC[mm];
                              const SmCurve         *pThisCurve = pCC->GetCurve();
                              for ( nn = 0; nn < pCC->GetSize(); nn++ )
                              {
                                  SmCurveInterval *pCI = &( *pCC )[nn];
                                  SmPoint3d sPt;
                                  SmFace *pThisFace = pCI->m_vMid.GetFaceObject();
                                  pThisCurve->EvaluatePoint( pCI->m_vInterval.GetMid(), sPt );

                                  if ( pThisFace )
                                  {
                                      SmVector2d pCI_mid_uv = pCI->m_vMid.GetUVParam();
                                      pThisFace->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI->m_vMid, &pCI_mid_uv );
                                  }
                                  if ( pCI->m_vMid.GetPointClass() != SM_PC_FACE && pThisFace )
                                  {
                                      SmVector2d pCI_mid_uv = pCI->m_vMid.GetUVParam();
                                      pNewFace->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI->m_vMid, &pCI_mid_uv );
                                  }
                              }
                          } // end updating classifications

                          // Record new face for output and to include in future intersections
                          sFaces1.Add( pNewFace );   // some redundancy, will intersect all sFaces2, but only needs from jj+1
                          rNewFaces.Add( pNewFace );
                      } // end if pNewFace
                  } // end making an edge in the first face

                  // Make an edge in the second face
                  {
                      SmVertex *pV1 = pCI2->m_vStart.GetVertexObject();
                      SmVertex *pV2 = pCI2->m_vEnd.GetVertexObject();
                      const SmCurve *p3dCurve = pCC2->GetCurve();
                      SmCurve *pCopyC = NULL;
                      p3dCurve->Copy( m_crContext, pCopyC );
                      SmExtent1d sIvl = pCI2->m_vInterval;

                      SmLoop *pNewLoop = NULL;
                      SmFace *pNewFace = NULL, *pOldFace = pCI2->m_vMid.GetFaceObject();

                      m_pBrep->MakeEdgeInFace( pOldFace, pV1, pV2, pCopyC, NULL, sIvl, SM_OT_SAME, m_dThisApproxTol3d, pNewEdge2, pNewLoop, pNewFace );

                      // If the face was split, update classifications and add new face to intersection list
                      if ( pNewFace )
                      {
                          // Classification update: if the interval m_vMid classifies to a face, does it classify to the new face?
                          for ( mm = kk; mm < sjjCC.GetSize(); mm++ )
                          {
                              SmCurveClassification *pCC = sjjCC[mm];
                              const SmCurve         *pThisCurve = pCC->GetCurve();
                              for ( nn = 0; nn < pCC->GetSize(); nn++ )
                              {
                                  SmCurveInterval *pCI = &( *pCC )[nn];
                                  SmPoint3d sPt;
                                  SmFace *pThisFace = pCI->m_vMid.GetFaceObject();
                                  pThisCurve->EvaluatePoint( pCI->m_vInterval.GetMid(), sPt );

                                  if ( pThisFace )
                                  {
                                      SmVector2d pCI_mid_uv = pCI->m_vMid.GetUVParam();
                                      pThisFace->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI->m_vMid, &pCI_mid_uv );
                                  }
                                  if ( pCI->m_vMid.GetPointClass() != SM_PC_FACE && pThisFace )
                                  {
                                      SmVector2d pCI_mid_uv = pCI->m_vMid.GetUVParam();
                                      pNewFace->Point3DClassify( sPt, m_dThisApproxTol3d, FALSE, pCI->m_vMid, &pCI_mid_uv );
                                  }
                              }
                          } // end updating classifications

                          // Record new face for output and to intersect both the old and the new face 
                          rNewFaces.Add( pNewFace );

                          // Intersect both the old and the new face
                          sFaces2.InsertAt( jj + 1, pNewFace );
                          jj++;
                      } // end if pNewFace
                  } // end making a new edge in the second face

                  // Keep track of which faces have new edges
                  rProcessedFaces.AddUnique( pFace1 );
                  rProcessedFaces.AddUnique( pFace2 );

                  // We want only one edge shared between the faces
                  m_pBrep->GlueEdgesGeneral( pNewEdge1, SM_OT_SAME, 0.0, bDoRegionNesting, pNewEdge2 );
                  rNewEdges.Add( pNewEdge1 );

              } // end for each SmCurveInterval
          } // end for each SmCurveClassification
      } // end for each face in sFaces2
  }  // end for each face in sFaces1

  return SM_SUCCESS;

} // end SmTopologyIntersector::FaceFaceIntersect



/***********************************************************************
PURPOSE: For each edge in the given list, glue any pairs of coincident faces
  that share all the same edges.

  Side effect: coincident faces incident are glued (one is deleted).

NOTES:
  For each glued edge,
  if any pair of its faces have the same set of edges in both faces,
  then check those faces for coincidence.
  If coincident, delete one.

  Input:
    rFacesToDelete: If we find a pair of coincident faces, and one of them
      is in this list, then delete that one and keep the other.

  Output:
    rProcessedFaces and rDeletedFaces will have the same size,
      with corresponding entries.
      In the case of three or more faces all coincident (or certain
      error conditions), rProcessedFaces could contain duplicates.
***********************************************************************/
SmStatus SmTopologyIntersector::FaceFaceCoincidence
(
  SmTArray< SmEdge * > & rInputEdges,      // in: coincident faces will contain these edges
  SmTArray< SmFace * > & rFacesToDelete,   // in: see Usage Notes.
  SmTArray< SmFace * > & rProcessedFaces,  // out: surviving coincident faces
  SmTArray< SmFace * > & rDeletedFaces,    // out: deleted   coincident faces
  double               & rdMaxGap,         // out
  double               & rdMinUnstitched,      // out
  SmBoolean              bIgnoreProblems,  // in
  SmBoolean              bDoRegionNesting  // NotUsed: in:  not used in this routine.
)
{
  SM_REF1(bDoRegionNesting) ;
  // Init outputs
  rProcessedFaces.ReSet();
  rDeletedFaces  .ReSet();
  rdMaxGap = 0;
  rdMinUnstitched = SM_BIG_DOUBLE;

  // Locals
  ULONG ii, jj, kk, lIdx;
  double dGap = 0.0;
  SmTArray< SmEdge* > sEdges1, sEdges2, sCommonEdges;
  SmTArray< SmFace* > sFacesOfEdge;
  ULONG lNumEdges = rInputEdges.GetSize();

  for ( ii = 0; ii < lNumEdges; ii++ )
  {
      SmEdge * pEdge = rInputEdges[ ii ];
      if ( pEdge == NULL ) { continue; }
      pEdge->GetFaces( sFacesOfEdge );
      ULONG lNumFaces = sFacesOfEdge.GetSize();
      for ( jj = 0; jj+1 < lNumFaces; jj++ )  // (Can't say lNumFaces-1 with Unsigned.)
      {
          if ( lNumFaces == 0 ) { break; } // (Because of Unsigned.)

          SmFace* pFace1 = sFacesOfEdge[ jj ];
          if ( pFace1 == NULL ) { continue; }

          pFace1->GetEdges( sEdges1 );
          ULONG lNumEdges1 = sEdges1.GetSize();
          for ( kk = jj+1; kk < lNumFaces; kk++ )
          {
              SmFace* pFace2 = sFacesOfEdge[ kk ];
              if ( pFace2 == NULL ) { continue; }

              pFace2->GetEdges( sEdges2 );
              ULONG lNumEdges2 = sEdges2.GetSize();

              // See whether they're the same sets of edges.
              if ( lNumEdges1 != lNumEdges2 )
                { continue; }

              sEdges1.FindCommonElements( sEdges2, sCommonEdges );
              if ( sCommonEdges.GetSize() != lNumEdges1 )
                { continue; }

              // Have two faces sharing the same set of edges.
              // Check for coincidence.
              // Note, they're coincident all around their edges,
              // so they're almost certainly either coincident,
              // or something like two halves of a sphere.
              SmBoolean bCoinFaces = FALSE;
              SmSurface *pSurf1 = pFace1->GetSurface();
              SmSurface *pSurf2 = pFace2->GetSurface();
              if ( pSurf1 == pSurf2 )
              {
                  bCoinFaces = TRUE;
                  rdMaxGap = 0.0;
              }
              else
              {
                  // Check an interior point.
                  SmExtent2d sDomain1 = pFace1->GetUVDomain();
                  SmExtent2d sDomain2 = pFace2->GetUVDomain();

                  // Note, that's not good enough, Face domains may not be at all tight, so:
                  SmExtent2d sTempDomain;
                  SmStatus eStat;
                  eStat = pFace1->CalculateUVDomainFromUVTrimCurves( sTempDomain );
                  if ( eStat == SM_SUCCESS )
                    { sDomain1 = sTempDomain; }
                  eStat = pFace2->CalculateUVDomainFromUVTrimCurves( sTempDomain );
                  if ( eStat == SM_SUCCESS )
                    { sDomain2 = sTempDomain; }

                  SmPoint2d sGuessUV1 = sDomain1.Evaluate( 0.5, 0.5 );
                  SmPoint2d sGuessUV2 = sDomain2.Evaluate( 0.5, 0.5 );
                  SmPoint2d sUV;
                  SmPoint3d sPt3d1, sPt3d2;

                  SmBoolean bOk;
                  SmBoolean bIsMulti;
                  pSurf1->EvaluatePoint( sGuessUV1, sPt3d1 );
                  pSurf2->DropPoint( sPt3d1, sDomain2, &sGuessUV2, bOk, sUV, dGap, bIsMulti );
                  if ( bOk && dGap <= m_dThisApproxTol3d )
                  {
                      bCoinFaces = TRUE;

                      rdMaxGap = smos_Max( rdMaxGap, dGap );
                  }
                  else
                  {
                      rdMinUnstitched = smos_Min( rdMinUnstitched, dGap );
                  }
              }

              if ( bCoinFaces )
              {
                  // Delete one face, and add pointers to lists.
                  SmFace * pDelFace = pFace1;
                  SmFace * pSavFace = pFace2;
                  if ( rFacesToDelete.FindElement( pFace2, lIdx ) )
                  {
                      pDelFace = pFace2;
                      pSavFace = pFace1;
                  }

                  SmBrep *pBrep = pDelFace->GetBrep();
                  if ( pBrep == NULL )
                  {
                      if ( bIgnoreProblems ) { continue;      }
                      else                   { SER( SM_ERR ); }
                  }
                  double dOldTol = pDelFace->GetTolerance();
                  pBrep->DeleteFace( pDelFace );

                  // Add to the output lists.
                  // In the case of three or more faces all coincident (or
                  // certain error conditions), pDelFace could be contained in
                  // rProcessedFaces.  In that case, set that entry to pSavFace.
                  if ( rProcessedFaces.FindElement( pDelFace, lIdx ) )
                    { rProcessedFaces.SetAt( lIdx, pSavFace ); }

                  rProcessedFaces.Add( pSavFace );
                  rDeletedFaces  .Add( pDelFace );

                  SmTol::UpdateObjectTolerance( pSavFace, dGap, dOldTol );

                  if ( pDelFace == pFace1 )
                    {
                      sFacesOfEdge[ jj ] = pDelFace = NULL;
                      // In this case, we're done with FacesOfEdge[jj].
                      break;
                    }
                  else
                    { sFacesOfEdge[ kk ] = pDelFace = NULL; }
              }
          } // end of inner loop on faces of edge (kk)
      } // end of outer loop on faces of edge (jj)
  } // end loop on all edges (ii)

  return SM_SUCCESS;

} // end FaceFaceCoincidence

//----------------------------------------------------------------------
// End of the six internal-use methods.
//----------------------------------------------------------------------



//
// Utilities
//

/*******************************************************************//**
PURPOSE:  Pretty Print SmTopologyIntersector Summary

NOTES:
***********************************************************************/
void SmTopologyIntersector::Dump
  (SmBoolean bValidPtrs,       // in : TRUE = mapped entities are still valid
                               //      FALSE= they have been deleted; default: TRUE
   SmBoolean bDumpMapObjects)  // in : TRUE = Dump every map object
                               //      FALSE= don't, default:[FALSE]
 const
{
  // locals
  TCHAR sBuff[SM_TBLOCK_SIZE], sBuffForFile[SM_TBLOCK_SIZE];

  // brep and Other locals
  SmTArray<SmFace*>   sFaces,    sOFaces;
  SmTArray<SmEdge*>   sEdges,    sOEdges;
  SmTArray<SmVertex*> sVertices, sOVertices;
  m_pBrep->GetFaces   (sFaces);
  m_pBrep->GetEdges   (sEdges);
  m_pBrep->GetVertices(sVertices);
  if(m_pOther) m_pOther->GetFaces   (sOFaces);
  if(m_pOther) m_pOther->GetEdges   (sOEdges);
  if(m_pOther) m_pOther->GetVertices(sOVertices);

  smos_sprintf(sBuff,       _T("\n\nSmTopologyIntersector  = 0x%p"), this);
  smos_sprintf(sBuffForFile,_T("\n\nSmTopologyIntersector  = %s"), _T("notNULL"));
             smos_WriteBuffer(sBuff, sBuffForFile);

  // output control parameters
  smos_sprintf(sBuff,_T("\nbSkipSSI               = %d,  bSkipManifoldEdges      = %d"),
             m_bSkipSSI, m_bSkipManifoldEdges);
             smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff,_T("\nbIntersectTangentEdges = %d,  lIntersectLaminaEdges   = %ld"),
             m_bIntersectTangentEdges, m_lIntersectLaminaEdges);
             smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff,_T("\nbIntersectWireEdges    = %d,  bIntersectShellVertices = %d"),
             m_bIntersectWireEdges, m_bIntersectShellVertices);
             smos_WriteBuffer(sBuff);
  smos_sprintf(sBuff,_T("\nbIgnoreTangency        = %d"),
             m_bIgnoreTangency);
             smos_WriteBuffer(sBuff);

  // output control parameters
  smos_sprintf(sBuff,_T("\nm_bTopologyDeleted = %d"),  m_bTopologyDeleted);
             smos_WriteBuffer(sBuff);

  // Don't need these anymore:
  //smos_sprintf(sBuff,_T("\n Types: Region = 16012"  )); smos_WriteBuffer(sBuff);
  //smos_sprintf(sBuff,_T("\n        Face   = 16002"  )); smos_WriteBuffer(sBuff);
  //smos_sprintf(sBuff,_T("\n        CFace  = 16023"  )); smos_WriteBuffer(sBuff);
  //smos_sprintf(sBuff,_T("\n        Edge   = 16016"  )); smos_WriteBuffer(sBuff);
  //smos_sprintf(sBuff,_T("\n        CEdge  = 16017"  )); smos_WriteBuffer(sBuff);
  //smos_sprintf(sBuff,_T("\n        Vertex = 16021")); smos_WriteBuffer(sBuff);

  // output mapped entity list
  smos_sprintf(sBuff,_T("\n\n # Entity Maps = %ld (start of list dump)" ), m_pBToO->Count());
  smos_WriteBuffer(sBuff);

  SmBoolean bCurve, bSurface ;
  ULONG lNumVerts=0, lNumEdges=0, lNumFaces=0;

#ifdef SM_DEFINED_HASH_ORDER
  static_assert(false, "not implemented");
#endif

  SmTArray<SmObject*> sKeys, sVals;
  m_pBToO->GetAllKeyValuePairs(sKeys, sVals);

  // Dump in format "type: [key] -> value"
  for (ULONG ii = 0; ii < sKeys.GetSize(); ++ii)
  {
      SmObject* pThisObject = sKeys[ii];
      SmObject* pOtherObject = sVals[ii];
      SM_ASSERT(pThisObject != NULL && pOtherObject != NULL);

      long lType;

      if ( bValidPtrs )
      {
          lType = pThisObject->GetType();
          bCurve = pThisObject->IsKindOf( SmCurve_TYPE );
          bSurface = pThisObject->IsKindOf( SmSurface_TYPE );
      }
      else
      {
          lType = SmUnknown_TYPE;
          bCurve = FALSE;
          bSurface = FALSE;
      }

      // Count Vertices, Edges, and Faces for output.
      if ( lType == SmVertex_TYPE )
      { lNumVerts++; }
      else if ( lType == SmEdge_TYPE )
      { lNumEdges++; }
      else if ( lType == SmFace_TYPE )
      { lNumFaces++; }

      double dDist = 0.0;
      // check for widely separated vertices
      if ( lType == SmVertex_TYPE
           && m_pOther )
      {
          dDist = ( (SmVertex *) pThisObject )->GetPoint().DistanceBetween( ( (SmVertex *) pThisObject )->GetPoint() );
      }
      else
      {
          dDist = 0.0;
      }

      // output the relationship
      smos_sprintf( sBuff, _T( "\n\t%s : [0x%p] = 0x%p" ),
                  lType == SmRegion_TYPE ? _T( " Region " )
                  : lType == SmFace_TYPE ? _T( " Face   " )
// Remove Composites
//                  : lType == SmCFace_TYPE ? _T( " CFace  " )
                  : lType == SmEdge_TYPE ? _T( " Edge   " )
// Remove Composites
//                  : lType == SmCEdge_TYPE ? _T( " CEdge  " )
                  : lType == SmVertex_TYPE ? _T( " Vertex " )
                  : lType == SmUnknown_TYPE ? _T( " <Deleted> " )
                  : bCurve ? _T( " Curve  " )
                  : bSurface ? _T( " Surface" )
                  : _T( " Other" ),
                  pThisObject, pOtherObject );
      smos_sprintf( sBuffForFile, _T( "\n\t%s : [%s] = %s" ),
                  lType == SmRegion_TYPE ? _T( " Region " )
                  : lType == SmFace_TYPE ? _T( " Face   " )
// Remove Composites
//                  : lType == SmCFace_TYPE ? _T( " CFace  " )
                  : lType == SmEdge_TYPE ? _T( " Edge   " )
// Remove Composites
//                  : lType == SmCEdge_TYPE ? _T( " CEdge  " )
                  : lType == SmVertex_TYPE ? _T( " Vertex " )
                  : lType == SmUnknown_TYPE ? _T( " <Deleted> " )
                  : bCurve ? _T( " Curve  " )
                  : bSurface ? _T( " Surface" )
                  : _T( " Other" ),
                  pThisObject ? _T( "notNULL" ) : _T( "NULL" ), pOtherObject ? _T( "notNULL" ) : _T( "NULL" ) );

      smos_WriteBuffer( sBuff, sBuffForFile );

      // add in data for widely spaced vertices
      if ( dDist > SM_EFF_ZERO * 2.0 )
      {
          smos_sprintf( sBuff, _T( " Vert/Vert Dist = %16.16lf" ), dDist );
          smos_WriteBuffer( sBuff );
      }
  } // end while outputting one line per m_pBToO entry

  // add in object dumps
  if ( bDumpMapObjects && bValidPtrs )
  {
      smos_sprintf( sBuff,_T("%s"), _T( "\n\n  EntityMap Object-Dumps:" ) );
      smos_WriteBuffer( sBuff );

#ifdef SM_DEFINED_HASH_ORDER
      static_assert(false, "not implemented");
#endif

      for(ULONG ii = 0; ii < sKeys.GetSize(); ++ii)
      {
          SmObject *pThisObject = sKeys[ii];
          SmObject *pOtherObject = sVals[ii];

          long lType = pThisObject->GetType();

          // output the relationship
          smos_sprintf( sBuff, _T( "\n\t%ld : [0x%p] = 0x%p" ), lType, pThisObject, pOtherObject );
          smos_sprintf( sBuffForFile, _T( "\n\t%ld : [%s] = %s" ), lType, pThisObject ? _T( "notNULL" ) : _T( "NULL" ), pOtherObject ? _T( "notNULL" ) : _T( "NULL" ) );
          smos_WriteBuffer( sBuff, sBuffForFile );
          smos_WriteBuffer( _T( "\n" ) );
          switch ( lType )
          {
          case SmVertex_TYPE: ( (SmVertex *) pThisObject )->Dump();
              if ( m_pOther ) ( (SmVertex *) pOtherObject )->Dump();
              break;
          case SmEdge_TYPE: ( (SmEdge *) pThisObject )->Dump();
              ( (SmEdge *) pThisObject )->GetCurve()->Dump();
              if ( m_pOther ) ( (SmEdge *) pOtherObject )->Dump();
              if ( m_pOther ) ( (SmEdge *) pOtherObject )->GetCurve()->Dump();
              break;
          case SmFace_TYPE: ( (SmFace *) pThisObject )->Dump();
              ( (SmFace *) pThisObject )->GetSurface()->Dump();
              if ( m_pOther ) ( (SmFace *) pOtherObject )->Dump();
              if ( m_pOther ) ( (SmFace *) pOtherObject )->GetSurface()->Dump();
              break;
          } // end switch on lType
      }
  } // end bDumpMapObjects check

  // output mapped entity list count and types
  if ( bValidPtrs ) {
      smos_sprintf(sBuff,_T("\n # Entity Maps = %ld: Verts %ld, Edges %ld, Faces %ld" ),
             m_pBToO->Count(), lNumVerts, lNumEdges, lNumFaces );
  } else {
      smos_sprintf(sBuff,_T("\n # Entity Maps = %ld" ), m_pBToO->Count() );
  }
  smos_WriteBuffer(sBuff);

  // output brep arguments - with some entity counts
  smos_sprintf(sBuff,       _T("\n\nBrep  = 0x%p,  # Faces = %ld, # Edges = %ld, # Vertices = %ld"),
             m_pBrep,sFaces.GetSize(),sEdges.GetSize(),sVertices.GetSize());
  smos_sprintf(sBuffForFile,_T("\n\nBrep  = %s,  # Faces = %ld, # Edges = %ld, # Vertices = %ld"),
             m_pBrep ? _T("notNULL") : _T("NULL"),sFaces.GetSize(),sEdges.GetSize(),sVertices.GetSize());
             smos_WriteBuffer(sBuff, sBuffForFile);

  // output other arguments - with some entity counts
  smos_sprintf(sBuff,       _T("\nOther = 0x%p,  # Faces = %ld, # Edges = %ld, # Vertices = %ld"),
             m_pOther,sOFaces.GetSize(),sOEdges.GetSize(),sOVertices.GetSize());
  smos_sprintf(sBuffForFile,_T("\nOther = %s,  # Faces = %ld, # Edges = %ld, # Vertices = %ld"),
             m_pOther ? _T("notNULL") : _T("NULL"),sOFaces.GetSize(),sOEdges.GetSize(),sOVertices.GetSize());
             smos_WriteBuffer(sBuff, sBuffForFile);

} // end SmTopologyIntersector::Dump

/*******************************************************************//**
PURPOSE: Draw routine for SmTopologyIntersectors

NOTES: Draw the list of mapped brep objects
***********************************************************************/
void SmTopologyIntersector::Draw
 (double dThisLineWidth,   // in : This Geometry Display line width
  double dOtherLineWidth,  // in : Other Geometry Display line width
  double dThisPointSize,   // in : This Geometry Display point size
  double dOtherPointSize)  // in : This Geometry Display point size
 const
{
#ifdef SM_GFX_OUTPUT_CODE

  // locals
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // locals
  long lType ;
  SmVertex *pVertex, *pOVertex ;
  SmEdge   *pEdge,   *pOEdge ;
  SmFace   *pFace,   *pOFace ;
  double dRv, dGv, dBv ;
  double dDist ;
  double     dInitPointSize = smgfx_GetPointSize() ;
  double     dInitLineWidth = smgfx_GetLineWidth() ;
  SmVector3d sThisColor     = smgfx_GetColor() ;
  SmVector3d sOtherColor(0,0,0) ;

  // pick a bad-vertex (gap greater than SM_EFF_ZERO * 2.0) color
  if(smos_3Max(sThisColor.x, sThisColor.y, sThisColor.z) == sThisColor.x)
    { dRv = 0.0 ; dGv = 1.0 ; dBv = 1.0 ;
    }
  else
    { dRv = 1.0 ; dGv = 0.0 ; dBv = 0.0 ;
    }

  smgfx_SetLineWidth(dThisLineWidth) ;
  smgfx_SetPointSize(dThisPointSize) ;

  // set up a color sequence for connected objects

  // for every entry in the map list
#ifdef SM_DEFINED_HASH_ORDER
  static_assert(false, "not implemented");
#endif
  SmTArray<SmObject*> sKeys, sVals;
  m_pBToO->GetAllKeyValuePairs(sKeys, sVals);

  for (ULONG ii = 0; ii < sKeys.GetSize(); ++ii)
    {
      SmObject *pThisObject  = sKeys[ii];
      SmObject *pOtherObject = sVals[ii];

      lType = pThisObject->GetType() ;

      // switch on type
      switch(lType)
        {
          // render mapped edges
          case SmEdge_TYPE   : pEdge  = (SmEdge *)pThisObject ;
                               pOEdge = (SmEdge *)pOtherObject ;
                               smgfx_SetColor(sThisColor) ;
                               smgfx_SetLineWidth(dThisLineWidth) ;  if(pEdge)  pEdge->Draw();
                               if ( FALSE )
                                 { if(pEdge)  pEdge->DrawParams(); }
                               smgfx_SetColor(sOtherColor) ;
                               smgfx_SetLineWidth(dOtherLineWidth) ; if(pOEdge) pOEdge->Draw() ;
                               break ;

          // render mapped vertices - draw bad maps as extra large
          case SmVertex_TYPE : pVertex  = (SmVertex *)pThisObject ;
                               smgfx_SetColor(sThisColor) ;
                               smgfx_SetPointSize(dThisPointSize) ;
                               if(pVertex) pVertex->Draw() ;

                               if(m_pOther)
                                 {
                                   pOVertex = (SmVertex *)pOtherObject ;
                                   dDist    = pVertex->GetPoint().DistanceBetween(pOVertex->GetPoint()) ;
                                   if(dDist > SM_EFF_ZERO * 2.0) { smgfx_SetColor(dRv, dGv, dBv) ; }
                                   else                          { smgfx_SetColor(sOtherColor) ; }
                                   smgfx_SetPointSize(dOtherPointSize) ;
                                   if(pOVertex) pOVertex->Draw() ;
                                 }
                               break ;

          // render mapped faces
          case SmFace_TYPE   : pFace  = (SmFace *)pThisObject ;
                               pOFace = (SmFace *)pOtherObject ;
                               
                               smgfx_SetColor(sThisColor) ;
                               smgfx_SetLineWidth(dThisLineWidth) ;  if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ;
                               smgfx_SetColor(sOtherColor) ;
                               smgfx_SetLineWidth(dOtherLineWidth) ; if(pOFace) pOFace->Draw(SM_DM_CROSSHATCH) ;
                               break ;

          // deleted object
          case SmUnknown_TYPE : break ;

          default:
            if(((SmObject *)pThisObject)->IsKindOf(SmCurve_TYPE))
              {
                SmCurve *pCurve  = (SmCurve *)pThisObject ;
                SmCurve *pOCurve = (SmCurve *)pOtherObject ;
                smgfx_SetColor(sThisColor) ;
                smgfx_SetLineWidth(dThisLineWidth) ;  if(pCurve) pCurve->DrawParams() ;
                smgfx_SetColor(sOtherColor) ;
                smgfx_SetLineWidth(dOtherLineWidth) ; if(pOCurve) pOCurve->DrawParams() ;
              } // end curve type check

            else if(((SmObject *)pThisObject)->IsKindOf(SmSurface_TYPE))
              {
                SmSurface *pSurface  = (SmSurface *)pThisObject ;
                SmSurface *pOSurface = (SmSurface *)pOtherObject ;
                smgfx_SetColor(sThisColor) ;
                smgfx_SetLineWidth(dThisLineWidth) ;  if(pSurface) pSurface->DrawUV() ;
                smgfx_SetColor(sOtherColor) ;
                smgfx_SetLineWidth(dOtherLineWidth) ; if(pOSurface) pOSurface->DrawUV() ;
              } // end surface type check

            else
              {
                smos_sprintf(sBuff,_T("Unsupported Object type : %ld "), lType) ;
                WARN(sBuff);
              }

        } // end switch on lType
    } // end iter every mapped m_pBToO entity entry

  // restore drawing state
  smgfx_SetLineWidth(dInitLineWidth) ;
  smgfx_SetPointSize(dInitPointSize) ;
  smgfx_SetColor(sThisColor) ;

#else
  SM_REF4(dThisLineWidth, dOtherLineWidth, dThisPointSize, dOtherPointSize);
#endif // SM_GFX_OUTPUT_CODE

} // end SmTopologyIntersector::Draw

/*******************************************************************//**
PURPOSE: Draw routine for SmTopologyIntersectors

NOTES: Draw the iith member of the list of mapped brep objects
***********************************************************************/
void SmTopologyIntersector::DrawOneAssoc
 (ULONG  iIndx,            // in : draw just the iith Assoc
  double dThisLineWidth,   // in : This Geometry Display line width
  double dOtherLineWidth,  // in : Other Geometry Display line width
  double dThisPointSize,   // in : This Geometry Display point size
  double dOtherPointSize)  // in : This Geometry Display point size
 const
{
#ifdef SM_GFX_OUTPUT_CODE

  // locals
  TCHAR sBuff[SM_TBLOCK_SIZE] ;

  // locals
  ULONG iCnt = 0 ;
  long lType ;
  SmVertex *pVertex, *pOVertex ;
  SmEdge   *pEdge,   *pOEdge ;
  SmFace   *pFace,   *pOFace ;
  double dRv, dGv, dBv ;
  double dDist ;
  double     dInitPointSize = smgfx_GetPointSize() ;
  double     dInitLineWidth = smgfx_GetLineWidth() ;
  SmVector3d sThisColor     = smgfx_GetColor() ;
  SmVector3d sOtherColor(0,0,0) ;

  // pick a bad-vertex (gap greater than SM_EFF_ZERO * 2.0) color
  if(smos_3Max(sThisColor.x, sThisColor.y, sThisColor.z) == sThisColor.x)
    { dRv = 0.0 ; dGv = 1.0 ; dBv = 1.0 ;
    }
  else
    { dRv = 1.0 ; dGv = 0.0 ; dBv = 0.0 ;
    }

  smgfx_SetLineWidth(dThisLineWidth) ;
  smgfx_SetPointSize(dThisPointSize) ;

  // set up a color sequence for connected objects

  // for every entry in the map list
#ifdef SM_DEFINED_HASH_ORDER
  static_assert(false, "not implemented");
#endif
  SmTArray<SmObject*> sKeys, sVals;
  m_pBToO->GetAllKeyValuePairs( sKeys, sVals );

  for (ULONG ii =0; ii < sKeys.GetSize(); ii++ )
  {
      SmObject *pThisObject  = (SmObject *)sKeys[ii];
      SmObject *pOtherObject = (SmObject *)sVals[ii];

      lType = pThisObject->GetType() ;

      // only draw the iith member
      if(iCnt == iIndx)
        {
          // switch on type
          switch(lType)
            {
              // render mapped edges
              case SmEdge_TYPE   : pEdge  = (SmEdge *)pThisObject ;
                                   pOEdge = (SmEdge *)pOtherObject ;
                                   smgfx_SetColor(sThisColor) ;
                                   smgfx_SetLineWidth(dThisLineWidth) ;  if(pEdge)  pEdge->Draw();
                                                                         if(pEdge)  pEdge->DrawParams();
                                   smgfx_SetColor(sOtherColor) ;
                                   smgfx_SetLineWidth(dOtherLineWidth) ; if(pOEdge) pOEdge->Draw() ;
                                   break ;

              // render mapped vertices - draw bad maps as extra large
              case SmVertex_TYPE : pVertex  = (SmVertex *)pThisObject ;
                                   smgfx_SetColor(sThisColor) ;
                                   smgfx_SetPointSize(dThisPointSize) ;
                                   if(pVertex) pVertex->Draw() ;

                                   if(m_pOther)
                                     {
                                       pOVertex = (SmVertex *)pOtherObject ;
                                       dDist    = pVertex->GetPoint().DistanceBetween(pOVertex->GetPoint()) ;
                                       if(dDist > SM_EFF_ZERO * 2.0) { smgfx_SetColor(dRv, dGv, dBv) ; }
                                       else                          { smgfx_SetColor(sOtherColor) ; }
                                       smgfx_SetPointSize(dOtherPointSize) ;
                                       if(pOVertex) pOVertex->Draw() ;
                                     }
                                   break ;

              // render mapped faces
              case SmFace_TYPE   : pFace  = (SmFace *)pThisObject ;
                                   pOFace = (SmFace *)pOtherObject ;
                               
                                   smgfx_SetColor(sThisColor) ;
                                   smgfx_SetLineWidth(dThisLineWidth) ;  if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ;
                                   smgfx_SetColor(sOtherColor) ;
                                   smgfx_SetLineWidth(dOtherLineWidth) ; if(pOFace) pOFace->Draw(SM_DM_CROSSHATCH) ;
                                   break ;

              // deleted object
              case SmUnknown_TYPE : break ;

              default:
                if(((SmObject *)pThisObject)->IsKindOf(SmCurve_TYPE))
                  {
                    SmCurve *pCurve  = (SmCurve *)pThisObject ;
                    SmCurve *pOCurve = (SmCurve *)pOtherObject ;
                    smgfx_SetColor(sThisColor) ;
                    smgfx_SetLineWidth(dThisLineWidth) ;  if(pCurve) pCurve->DrawParams() ;
                    smgfx_SetColor(sOtherColor) ;
                    smgfx_SetLineWidth(dOtherLineWidth) ; if(pOCurve) pOCurve->DrawParams() ;
                  } // end curve type check

                else if(((SmObject *)pThisObject)->IsKindOf(SmSurface_TYPE))
                  {
                    SmSurface *pSurface  = (SmSurface *)pThisObject ;
                    SmSurface *pOSurface = (SmSurface *)pOtherObject ;
                    smgfx_SetColor(sThisColor) ;
                    smgfx_SetLineWidth(dThisLineWidth) ;  if(pSurface) pSurface->DrawUV() ;
                    smgfx_SetColor(sOtherColor) ;
                    smgfx_SetLineWidth(dOtherLineWidth) ; if(pOSurface) pOSurface->DrawUV() ;
                  } // end surface type check

                else
                  {
                    smos_sprintf(sBuff,_T("Unsupported Object type : %ld "), lType) ;
                    WARN(sBuff);
                  }

            } // end switch on lType
        } // end check for iith index
    } // end iter every mapped m_pBToO entity entry

  // restore drawing state
  smgfx_SetLineWidth(dInitLineWidth) ;
  smgfx_SetPointSize(dInitPointSize) ;
  smgfx_SetColor(sThisColor) ;

#else
  SM_REF5(iIndx, dThisLineWidth, dOtherLineWidth, dThisPointSize, dOtherPointSize);
#endif // SM_GFX_OUTPUT_CODE

} // end SmTopologyIntersector::DrawOneAssoc

/*******************************************************************//**
PURPOSE: Draw routine for SmTopologyIntersectors

NOTES: Draw the list of mapped brep objects
***********************************************************************/

void SmTopologyIntersector::DrawDetails
 (double dLineWidth,   // in : Display line width
  double dPointSize)   // in : Display point size
   const
{
#ifdef SM_GFX_OUTPUT_CODE

  // locals
  TCHAR sBuff[SM_TBLOCK_SIZE] ;
  ULONG lColorCnt = 16, lColorIndex = 0 ;
  SmVector3d sColors[16]  = { SmVector3d( 0, 0, 0), SmVector3d( 0, 0, 1), SmVector3d( 0, 1, 0), SmVector3d( 1, 0, 0),
                              SmVector3d( 1, 0, 1), SmVector3d( 0, 1, 1), SmVector3d(.5, 1, 0), SmVector3d( 0,.5, 1),
                              SmVector3d( 1, 0,.5), SmVector3d( 1,.5, 0), SmVector3d( 0, 1,.5), SmVector3d(.5, 0, 1),
                              SmVector3d(.2,.4,.8), SmVector3d(.8,.2,.4), SmVector3d(.4,.8,.2), SmVector3d(.5,.3,.2) } ;
  TCHAR sColorTag[16][32] = {      {_T("[ 0  0  0]")},   {_T("[ 0  0  1]")},   {_T("[ 0  1  0]")},   {_T("[ 1  0  0]")},
                                   {_T("[ 1  0  1]")},   {_T("[ 0  1  1]")},   {_T("[.5  1  0]")},   {_T("[ 0 .5  1]")},
                                   {_T("[ 1  0 .5]")},   {_T("[ 1 .5  0]")},   {_T("[ 0  1 .5]")},   {_T("[.5  0  1]")},
                                   {_T("[.2 .4 .8]")},   {_T("[.8 .2 .4]")},   {_T("[.4 .8 .2]")},   {_T("[.5 .3 .2]")} } ;
  // locals
  long lType ;
  SmVertex *pVertex, *pOVertex ;
  SmEdge   *pEdge,   *pOEdge ;
  SmFace   *pFace,   *pOFace ;
  double      dGap ;
  SmZoneTol3d sZoneTol3d ;
  double      dInitPointSize = smgfx_GetPointSize() ;
  double      dInitLineWidth = smgfx_GetLineWidth() ;
  SmVector3d  sColor         = smgfx_GetColor() ;
  SmBoolean   bBadGap        = FALSE ;

  // init line and point sizes
  smgfx_SetLineWidth(dLineWidth) ;
  smgfx_SetPointSize(dPointSize) ;

  // for every entry in the map list
#ifdef SM_DEFINED_HASH_ORDER
  static_assert("not implemented");
#endif
  ULONG ii;
  SmTArray<SmObject*> sKeys, sVals;
  m_pBToO->GetAllKeyValuePairs( sKeys, sVals );
  for (ii =0; ii < sKeys.GetSize(); ii++ )
  {
      SmObject *pThisObject  = (SmObject *)sKeys[ii];
      SmObject *pOtherObject = (SmObject *)sVals[ii];


      lType = pThisObject->GetType() ;
      SmBoolean bCurve   = pThisObject->IsKindOf(SmCurve_TYPE) ;
      SmBoolean bSurface = pThisObject->IsKindOf(SmSurface_TYPE) ;

      // increment color index and switch colors
      lColorIndex = (lColorIndex + 1) % lColorCnt ;
      smgfx_SetColor(sColors[lColorIndex]) ;

      // output text to help interpret the drawing - we're looking for confused mappings
      smos_sprintf(sBuff, _T("\n\t%s%s : [0x%p] = 0x%p"),  lType == SmRegion_TYPE ? _T(" Region ")
                                                       : lType == SmFace_TYPE   ? _T(" Face   ")
// Remove Composites
//                                                       : lType == SmCFace_TYPE  ? _T(" CFace  ")
                                                       : lType == SmEdge_TYPE   ? _T(" Edge   ")
// Remove Composites
//                                                       : lType == SmCEdge_TYPE  ? _T(" CEdge  ")
                                                       : lType == SmVertex_TYPE ? _T(" Vertex ")
                                                       : bCurve                 ? _T(" Curve  ")
                                                       : bSurface               ? _T(" Surface")
                                                       : _T(" Other"),
                                                       sColorTag[lColorIndex],
                                                       pThisObject, pOtherObject);
      smos_WriteBuffer(sBuff) ;

      // switch on type
      switch(lType)
        {
          // render mapped edges
          case SmEdge_TYPE   : { pEdge  = (SmEdge *)pThisObject ;
                                 pOEdge = (SmEdge *)pOtherObject ;

                                 bBadGap = FALSE ;
                                 if(pEdge && pOEdge)
                                   {
                                     sZoneTol3d = pEdge->GetTolerance() ;
                                     SmExtent1d sInterval = pEdge->GetInterval();
                                     SmCrvCrvGapFunction sCrvCrvGap( (SmXSectTol3d)sZoneTol3d, pEdge->GetCurve(), sInterval, pOEdge->GetCurve() ) ;
                                     SmGapSample *pMaxGapSample;  
                                     sCrvCrvGap.GetMaxGapSample( pMaxGapSample );
                                     if ( !pMaxGapSample )
                                     { continue; }
                                     
                                     dGap = pMaxGapSample->GetLength() ;
                                     if(dGap > sZoneTol3d) { smgfx_SetLineWidth(2 * dLineWidth) ;
                                                       smgfx_SetPointSize(2 * dPointSize) ;
                                                       bBadGap = TRUE ;

                                                       sCrvCrvGap.Draw(FALSE, TRUE, TRUE) ;

                                                       // output text to help interpret the drawing - we're looking for confused mappings
                                                       smos_sprintf(sBuff, _T(" %s [Gap=%16.16lf, Tol=%16.16lf]"),
                                                                         _T("Bad Gap"), dGap, sZoneTol3d.val);
                                                       smos_WriteBuffer(sBuff) ;
                                                     }
                                   }
                                 else
                                   {
                                     // output text to help interpret the drawing - we're looking for confused mappings
                                     smos_sprintf(sBuff, _T(" %s [Edge=0x%p, OEdge=0x%p]"),
                                                       _T("Bad Record - NULL objects"), pEdge, pOEdge);
                                     smos_WriteBuffer(sBuff) ;
                                   }

                                 pEdge->Draw() ;
                                 if(m_pOther) pOEdge->Draw() ;

                                 if(bBadGap) { smgfx_SetLineWidth(dLineWidth) ;
                                               smgfx_SetPointSize(dPointSize) ;
                                               bBadGap = FALSE ;
                                             }
                               }
                               break ;

          // render mapped vertices - draw bad maps as extra large
          case SmVertex_TYPE : { pVertex  = (SmVertex *)pThisObject ;
                                 bBadGap  = FALSE ;

                                 if(m_pOther)
                                   {
                                     pOVertex = (SmVertex *)pOtherObject ;
                                     sZoneTol3d     = pVertex->GetTolerance() ;
                                     dGap     = pVertex->GetPoint().DistanceBetween(pOVertex->GetPoint()) ;

                                     if(dGap > sZoneTol3d) { smgfx_SetLineWidth(2 * dLineWidth) ;
                                                       smgfx_SetPointSize(2 * dPointSize) ;
                                                       bBadGap = TRUE ;

                                                       SmPoint3d sPt1 = pVertex->GetPoint() ;
                                                       SmPoint3d sPt2 = pOVertex->GetPoint() ;
                                                       smgfx_DrawLine(&sPt1, &sPt2) ;

                                                       // output text to help interpret the drawing - we're looking for confused mappings
                                                       smos_sprintf(sBuff, _T(" %s [Gap=%16.16lf, Tol=%16.16lf]"),
                                                                         _T("Bad Gap"), dGap, sZoneTol3d.val);
                                                       smos_WriteBuffer(sBuff) ;
                                                     }
                                     pOVertex->Draw() ;
                                   }
                                 else
                                   {
                                     // output text to help interpret the drawing - we're looking for confused mappings
                                     smos_sprintf(sBuff, _T(" %s [Vertex=0x%p, OVertex=0x%p]"),
                                                       _T("Bad Record - NULL objects"), pVertex, m_pOther);
                                     smos_WriteBuffer(sBuff) ;
                                   }

                                 pVertex->Draw() ;

                                 if(bBadGap) { smgfx_SetLineWidth(dLineWidth) ;
                                               smgfx_SetPointSize(dPointSize) ;
                                               bBadGap = FALSE ;
                                             }
                               }
                               break ;

          // render mapped faces
          case SmFace_TYPE   : { pFace  = (SmFace *)pThisObject ;
                                 pOFace = (SmFace *)pOtherObject ;

                                 bBadGap = FALSE ;
                                 if(pFace && pOFace)
                                   {
                                     sZoneTol3d = pFace->GetTolerance() ;
                                     SmExtent2d sDomain = pFace->GetUVDomain();
                                     SmSrfSrfGapFunction sSrfSrfGap( (SmXSectTol3d)sZoneTol3d, pFace->GetSurface(), sDomain, pOFace->GetSurface() ) ;
                                     SmGapSample *pMaxGapSample;  
                                     sSrfSrfGap.GetMaxGapSample(pMaxGapSample);
                                     if (!pMaxGapSample)
                                     { continue; }
                                     
                                     dGap = pMaxGapSample->GetLength() ;
                                     if(dGap > sZoneTol3d) { smgfx_SetLineWidth(2 * dLineWidth) ;
                                                       smgfx_SetPointSize(2 * dPointSize) ;
                                                       bBadGap = TRUE ;

                                                       sSrfSrfGap.Draw(FALSE, TRUE, TRUE) ;

                                                       // output text to help interpret the drawing - we're looking for confused mappings
                                                       smos_sprintf(sBuff, _T(" %s [Gap=%16.16lf, Tol=%16.16lf]"),
                                                                         _T("Bad Gap"), dGap, sZoneTol3d.val);
                                                       smos_WriteBuffer(sBuff) ;
                                                     }
                                   }
                                 else
                                   {
                                     // output text to help interpret the drawing - we're looking for confused mappings
                                     smos_sprintf(sBuff, _T(" %s [Face=0x%p, OFace=0x%p]"),
                                                       _T("Bad Record - NULL objects"), pFace, pOFace);
                                     smos_WriteBuffer(sBuff) ;
                                   }

                                 pFace->Draw(SM_DM_CROSSHATCH) ;
                                 if(m_pOther) pOFace->Draw(SM_DM_CROSSHATCH) ;

                                 if(bBadGap) { smgfx_SetLineWidth(dLineWidth) ;
                                               smgfx_SetPointSize(dPointSize) ;
                                               bBadGap = FALSE ;
                                             }
                               }
                               break ;

          default:
            if(((SmObject *)pThisObject)->IsKindOf(SmCurve_TYPE))
              {
                SmCurve *pCurve  = (SmCurve *)pThisObject ;
                SmCurve *pOCurve = (SmCurve *)pOtherObject ;

                bBadGap = FALSE ;
                if(pCurve && pOCurve)
                  {
                    sZoneTol3d = m_pBrep->GetTolerance() ;
                    SmExtent1d sInterval = pCurve->GetNaturalInterval();
                    SmCrvCrvGapFunction sCrvCrvGap( (SmXSectTol3d)sZoneTol3d, pCurve, sInterval, pOCurve ) ;
                    SmGapSample *pMaxGapSample;  
                    sCrvCrvGap.GetMaxGapSample( pMaxGapSample );
                    if (!pMaxGapSample )
                    { continue; }
                    
                    dGap = pMaxGapSample->GetLength() ;
                    if(dGap > sZoneTol3d) { smgfx_SetLineWidth(2 * dLineWidth) ;
                                      smgfx_SetPointSize(2 * dPointSize) ;
                                      bBadGap = TRUE ;

                                      // output text to help interpret the drawing - we're looking for confused mappings
                                      smos_sprintf(sBuff, _T(" %s [Gap=%16.16lf, Tol=%16.16lf]"),
                                                        _T("Bad Gap"), dGap, sZoneTol3d.val);
                                      smos_WriteBuffer(sBuff) ;
                                    }
                  }
                else
                  {
                    // output text to help interpret the drawing - we're looking for confused mappings
                    smos_sprintf(sBuff, _T(" %s [Curve=0x%p, OCurve=0x%p]"),
                                      _T("Bad Record - NULL objects"), pCurve, pOCurve);
                    smos_WriteBuffer(sBuff) ;
                  }

                pCurve->DrawParams() ;
                if(m_pOther && pOCurve) pOCurve->DrawParams() ;

                if(bBadGap) { smgfx_SetLineWidth(dLineWidth) ;
                              smgfx_SetPointSize(dPointSize) ;
                              bBadGap = FALSE ;
                            }
              } // end curve type check

            else if(((SmObject *)pThisObject)->IsKindOf(SmSurface_TYPE))
              {
                SmSurface *pSurface  = (SmSurface *)pThisObject ;
                SmSurface *pOSurface = (SmSurface *)pOtherObject ;

                bBadGap = FALSE ;
                if(pSurface && pOSurface)
                  {
                    sZoneTol3d = m_pBrep->GetTolerance() ;
                    SmExtent2d sDomain = pSurface->GetNaturalUVDomain();
                    SmSrfSrfGapFunction sSrfSrfGap( (SmXSectTol3d)sZoneTol3d, pSurface, sDomain, pOSurface ) ;
                    SmGapSample *pMaxGapSample;  
                    sSrfSrfGap.GetMaxGapSample(pMaxGapSample);
                    if (!pMaxGapSample)
                    { continue; }
                    
                    dGap = pMaxGapSample->GetLength() ;
                    if(dGap > sZoneTol3d) { smgfx_SetLineWidth(2 * dLineWidth) ;
                                      smgfx_SetPointSize(2 * dPointSize) ;
                                      bBadGap = TRUE ;

                                      // output text to help interpret the drawing - we're looking for confused mappings
                                      smos_sprintf(sBuff, _T(" %s [Gap=%16.16lf, Tol=%16.16lf]"),
                                                        _T("Bad Gap"), dGap, sZoneTol3d.val);
                                      smos_WriteBuffer(sBuff) ;
                                    }
                  }
                else
                  {
                    // output text to help interpret the drawing - we're looking for confused mappings
                    smos_sprintf(sBuff, _T(" %s [Surface=0x%p, OSurface=0x%p]"),
                                      _T("Bad Record - NULL objects"), pSurface, pOSurface);
                    smos_WriteBuffer(sBuff) ;
                  }

                pSurface->DrawUV() ;
                if(m_pOther && pOSurface) pOSurface->DrawUV() ;

                if(bBadGap) { smgfx_SetLineWidth(dLineWidth) ;
                              smgfx_SetPointSize(dPointSize) ;
                              bBadGap = FALSE ;
                            }
              } // end surface type check
            else
              {
                smos_sprintf(sBuff,_T("Unsupported Object type : %ld "), lType);
                WARN(sBuff);
              }

        } // end switch on lType
    } // end iter every mapped m_pBToO entity entry

  // restore drawing state
  smgfx_SetLineWidth(dInitLineWidth) ;
  smgfx_SetPointSize(dInitPointSize) ;
  smgfx_SetColor(sColor) ;

#else
  SM_REF2(dLineWidth, dPointSize);
#endif // SM_GFX_OUTPUT_CODE

} // end SmTopologyIntersector::DrawDetails


