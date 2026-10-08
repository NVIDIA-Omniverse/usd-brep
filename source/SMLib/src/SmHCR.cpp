// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmHCR.cpp
* PURPOSE: Implementation of hidden curve removal methods.
**********************************************************************/

//#define FIX_ME 1

/*
 * About this functionality:
 *
 * 1) It is not tessellation-based, but works off the TRUE NURBS geometry.
 *
 * 2) It captures the rotation matrix of the current view from the graphics...
 *  this gives the real-space direction vector for the eye------>
 *
 * 3) It computes which edges to display, based on:
 *   a) Face boundaries (inner and outer curves).
 *   b) Surface silhouette curves computed from points where the surface normal is orthogonal
 *    to a direction vector from the 'eye' to the surface point
 *
 * 4) These curves are then trimmed by :
 *   a) face boundaries.
 *   b) visibility wrt a ray from the eye to the curve.....if there is a obstructing face
 *    then that segment is not visible.
 *
 * 5) The remaining visible curve (segments)s are then drawn in 3d in their
 *  true 3d location.  This gives the illusion that they are being drawn in 2d
 *  screen space.  If you rotate the result you will see that they are 3d curves.
 *
 * 6) All computations are done within a single brep.  A curve is only hidden
 *  by surface within its own brep. If you have multiple breps, they should be
 *  merged into one, if you need this feature.
 *  ( Use pBrep->MergeBrep(*pBrep1); )
 *
 * 7) When two faces have a common c1 cross-continuous boundary (for example,
 *    two coplanar triangles with a shared diagonal) we display the boundary curve...
 *    some customers have asked us to not do that,
 *    and we now provide the option not to display such seam edges.
 *     (SmHCR::SetDisplaySmoothEdges() and SmHCR::GetDisplaySmoothEdges()).
 */

#include "StdAfx.h"

#include <SmGraphicsExtern.h>
#include <SmHCR.h>
#include <SmTopologySolver.h>
#include <SmBSplineCurve.h>
#include <SmBSplineSurface.h>
#include <SmPlane.h>
#include <SmTree.h>
#include <SmGraphicsOutput.h>
#include <SmBrep.h>
#include <SmFace.h>
#include <SmEdge.h>
#include <SmVertex.h>
#include <SmCurveClass.h>
#include <SmAttribute.h>

#define SM_VERTEX_POINTSIZE 4.0

/*******************************************************************//**
PURPOSE: Destructor for a HCR object.

NOTES:
***********************************************************************/
SmHCR::~SmHCR()
{
  // free contained objects
  SM_ASSERT(m_pSceneBrep       != NULL) ; delete m_pSceneBrep ;       m_pSceneBrep       = NULL ;
  SM_ASSERT(m_pFrontFaceTree   != NULL) ; delete m_pFrontFaceTree ;   m_pFrontFaceTree   = NULL ;
  SM_ASSERT(m_pSilVertexTree   != NULL) ; delete m_pSilVertexTree ;   m_pSilVertexTree   = NULL ;
  SM_ASSERT(m_pSilEdgeTree     != NULL) ; delete m_pSilEdgeTree ;     m_pSilEdgeTree     = NULL ;
  SM_ASSERT(m_pSilVertices     != NULL) ; delete m_pSilVertices ;     m_pSilVertices     = NULL ;
  SM_ASSERT(m_pSilEdges        != NULL) ; delete m_pSilEdges ;        m_pSilEdges        = NULL ;
  SM_ASSERT(m_pBackFacingEdges != NULL) ; delete m_pBackFacingEdges ; m_pBackFacingEdges = NULL ;

  // free m_pVisibleCurves and its items
  ULONG ii, lNumCurves = m_pVisibleCurves->GetSize();
  for (ii=0; ii<lNumCurves; ii++) 
    {
      SmCurve *pCurve = (*m_pVisibleCurves)[ii];
      SM_ASSERT(pCurve != NULL) ; delete pCurve ; pCurve = NULL ;
    }
  SM_ASSERT(m_pVisibleCurves != NULL) ; delete m_pVisibleCurves ; m_pVisibleCurves = NULL ;

  // free m_pInvisibleCurves and its items
  lNumCurves = m_pInvisibleCurves->GetSize();
  for (ii=0; ii<lNumCurves; ii++) 
    {
      SmCurve *pCurve = (*m_pInvisibleCurves)[ii];
      SM_ASSERT(pCurve != NULL) ; delete pCurve ; pCurve = NULL ;
    }
  SM_ASSERT(m_pInvisibleCurves != NULL) ; delete m_pInvisibleCurves ; m_pInvisibleCurves = NULL ;

  // free the rest of the contained objects
  SM_ASSERT(m_pVisiblePoints   != NULL) ; delete m_pVisiblePoints ;   m_pVisiblePoints   = NULL ;
  SM_ASSERT(m_pInvisiblePoints != NULL) ; delete m_pInvisiblePoints ; m_pInvisiblePoints = NULL ;
  if (m_pRayTracer)    { delete m_pRayTracer;    m_pRayTracer    = NULL ; }
  if (m_pGridElements) { delete m_pGridElements; m_pGridElements = NULL ; }
  if (m_pMinDistances) { delete m_pMinDistances; m_pMinDistances = NULL ; }
  if (m_pMaxDistances) { delete m_pMaxDistances; m_pMaxDistances = NULL ; }

} // end SmHCR::~SmHCR  destructor

/*******************************************************************//**
PURPOSE: Constructor for a HCR object from given ViewVector.

NOTES: 

***********************************************************************/
SmHCR::SmHCR
  (const SmContext        & crContext,                // in : context for new object construction, stored with this object
   const SmBrep           * pBrep,                    // in : Target Brep, an internal copy is stored
   const SmVector3d       & crViewVector,             // in : eye direction runs parallel to this vector
   double                   dThisApproxTol3d,         // in : dist tol  that limits segment sizes when walking silhouette curves
   double                   dThisAngTolRad,           // in : angle tol that limits segment sizes when walking silhouette curves
   SmBoolean                bDisplaySeams,            // in : TRUE = marks seams as visible, FALSE= lets seams be invisible
   SmBoolean                bDisplaySmoothEdges)      // in : TRUE = marks manifold edges that are G1 between their faces as visible, 
                                                      //      FALSE=lets them be invisible
 : m_crContext(crContext),
   m_pOriginalBrep(pBrep),
   m_dThisApproxTol3d(dThisApproxTol3d),
   m_dThisAngTolRad(dThisAngTolRad),
   m_bHCRForSilhouettes(FALSE),
   m_bDisplaySeams(bDisplaySeams),
   m_bDisplaySmoothEdges(bDisplaySmoothEdges),
   m_pRayTracer(NULL),
   m_pGridElements(NULL),
   m_pMinDistances(NULL),
   m_pMaxDistances(NULL)
{
  SM_ASSERT(pBrep != NULL);

  // Set up the Brep in the view coordinate system
  // where the eye is looking parallel to the Z axis.
  SmPoint3d sOrigin(0, 0, 0) ;
  SmVector3d sViewVector(crViewVector), sPerp1, sPerp2 ;
  SmAxis2Placement sViewTransform ;

  // get vectors perp to viewing vector
  sViewVector.MakeUnitOrthoVectors(NULL, sViewVector, sPerp1, sPerp2) ;

  // now make the ViewTransform
  sViewTransform.SetSTEPCanonical(sOrigin, sViewVector, sPerp1) ;
  sViewTransform.Invert(sViewTransform) ;


  // Init code common to all constructors
  ConstructorInit(sViewTransform) ;

} // end SmHCR::SmHCR constructor

/*******************************************************************//**
PURPOSE: Constructor for a HCR object from SmAxis2Plaement eye postion.

NOTES: 
    Note on View Vector:
       [0 0 1] = OrigViewVector * crViewTransform, thus
       OrigViewVector = [0 0 1] * InvViewTransform.

   To build an SmAxis2Placement for a given ViewVector 
     either call the previous constructor or use the following

     SmPoint3d sOrigin(0, 0, 0) ;
     SmVector3d sViewVector, sPerp1, sPerp2 ;
     SmAxis2Placement sViewTransform ;
     sViewVector.MakeUnitOrthoVectors(NULL, sViewVector, sPerp1, sPerp2) ;
     sViewTransform.SetSTEPCanonical(sOrigin, sViewVector, sPerp1) ;
     sViewTransform.Invert(sViewTransform) ;

***********************************************************************/
SmHCR::SmHCR
  (const SmContext        & crContext,                // in : context for new object construction, stored with this object
   const SmBrep           * pBrep,                    // in : Target Brep, an internal copy is stored
   const SmAxis2Placement & crViewTransform,          // in : Set eye orientation: SceneBrep = OrigBrep_Shapes * TransMatrix
                                                      //        SceneViewVector = [0 0 1] * InvViewTransform
   double                   dThisApproxTol3d,         // in : dist tol  that limits segment sizes when walking silhouette curves
   double                   dThisAngTolRad,           // in : angle tol that limits segment sizes when walking silhouette curves
   SmBoolean                bDisplaySeams,            // in : TRUE = marks seams as visible, FALSE= lets seams be invisible
   SmBoolean                bDisplaySmoothEdges)      // in : TRUE = marks manifold edges that are G1 between their faces as visible, 
                                                      //      FALSE=lets them be invisible
 : m_crContext(crContext),
   m_pOriginalBrep(pBrep),
   m_dThisApproxTol3d(dThisApproxTol3d),
   m_dThisAngTolRad(dThisAngTolRad),
   m_bHCRForSilhouettes(FALSE),
   m_bDisplaySeams(bDisplaySeams),
   m_bDisplaySmoothEdges(bDisplaySmoothEdges),
   m_pRayTracer(NULL),
   m_pGridElements(NULL),
   m_pMinDistances(NULL),
   m_pMaxDistances(NULL)
{
  SM_ASSERT(pBrep != NULL);

  // Set up the Brep in the view coordinate system
  // where the eye is looking parallel to the Z axis.

  // Init code common to all constructors
  ConstructorInit(crViewTransform) ;

} // end SmHCR::SmHCR constructor

/*******************************************************************//**
PURPOSE: Init code common to all constructors

NOTES:  Allocate internal memory and set up the 
    SceneBrep from the OrigBrep using the given ViewTransformation

ASSUMES --- the following fields have been set as:
    m_crContext              (crContext),                            
    m_pOriginalBrep          (pBrep),                            
    m_dThisApproxTol3d(dThisApproxTol3d),
    m_dThisAngTolRad (dThisAngTolRad),  
    m_bHCRForSilhouettes     (FALSE),                       
    m_bDisplaySeams          (bDisplaySeams),                    
    m_bDisplaySmoothEdges    (bDisplaySmoothEdges),        
    m_pRayTracer             (NULL),                                
    m_pGridElements          (NULL),                             
    m_pMinDistances          (NULL),                             
    m_pMaxDistances          (NULL),                                                          
  
***********************************************************************/
void SmHCR::ConstructorInit
  (const SmAxis2Placement & crViewTransform)   // in : Set eye orientation: SceneBrep = OrigBrep_Shapes * TransMatrix
                                               //        SceneViewVector = [0 0 1] * InvViewTransform
{
  // save the View Transformation
  m_vViewTransform = crViewTransform ;                 

  // zero out translation component of given ViewTransformation
  const SmVector3d &rTranslate = -m_vViewTransform.GetOriginRef();
  m_vViewTransform.Translate(rTranslate);

  // ViewTransform invert
  m_vViewTransform.Invert(m_vInvViewTransform);
                                                                                      
  // copy the original Brep
  m_pSceneBrep = new (m_crContext) SmBrep(*m_pOriginalBrep);
  SM_ASSERT(m_pSceneBrep != NULL);
  m_pSceneBrep->m_bEditingEnabled = TRUE;

// Remove Composites
// #ifndef SM_NO_COMPOSITES   //cbi_CEdge: 20
//   m_pSceneBrep->m_bMakeComposites = TRUE;
// #endif

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smos_WriteBuffer(_T("\n\nHidden Curve - Copy of Brep Done\n"));
    }
#endif // SM_DEBUG_CODE
  
  // transform the copied Brep to place the eye looking up the Z axis
  SE(m_pSceneBrep->Transform(m_vViewTransform));
  SE(m_pSceneBrep->FindAndSetInfiniteRegion()); // [B23]
  
  // store the view direction for the rotated SceneBrep
  m_vViewVector.Set(0,0,1);

  // Scene Brep Bounding box
  m_pSceneBrep->CalculateBoundingBox(m_vProjBBox);

  // Ray Start tolerance based on BoundingBox size
  m_dRayOffsetTolerance = (m_vProjBBox.GetMax().z-m_vProjBBox.GetMin().z + 1.0) / 1000.0;

  // Make the bounding box big enough to contain all possible silhouettes.
  SmVector3d sSize    = m_vProjBBox.GetSize();
  double     dMaxSize = sSize.GetMaxDimension();
  m_vProjBBox.ExpandAbsolute(dMaxSize);

  // allocate Decomposition Trees that will hold SceneBrep vertices, edges, and faces
  m_pSilVertexTree = new (m_crContext) SmTree(m_vProjBBox);
  m_pSilEdgeTree   = new (m_crContext) SmTree(m_vProjBBox);
  m_pFrontFaceTree = new (m_crContext) SmTree(m_vProjBBox);
  SM_ASSERT(m_pSilVertexTree != NULL);
  SM_ASSERT(m_pSilEdgeTree   != NULL);
  SM_ASSERT(m_pFrontFaceTree != NULL);

  // allocate containers to hold pointers to classified geometry
  m_pVisibleCurves   = new(m_crContext) SmTArray<SmCurve*>(m_crContext);
  m_pInvisibleCurves = new(m_crContext) SmTArray<SmCurve*>(m_crContext);
  m_pVisiblePoints   = new(m_crContext) SmTArray<SmPoint3d>(m_crContext);
  m_pInvisiblePoints = new(m_crContext) SmTArray<SmPoint3d>(m_crContext);
 
  // allocate maps (That seem to be used as lists of new silhouette geometry and backFacingEdges)
  m_pSilVertices     = new(m_crContext) SmMapPtrToPtr<SmVertex, SmVertex>(&m_crContext);
  m_pSilEdges        = new(m_crContext) SmMapPtrToPtr<SmEdge, SmEdge>    (&m_crContext);
  m_pBackFacingEdges = new(m_crContext) SmMapPtrToPtr<SmEdge, SmEdge>    (&m_crContext);

} // end SmHCR::ConstructorInit


/*******************************************************************//**
PURPOSE: Return our Silhouette Edges.

NOTES: 

***********************************************************************/
SmStatus SmHCR::GetSilhouetteEdges( SmTArray< SmEdge* > &rSilEdges )
{
  rSilEdges.ReSet();

#ifdef SM_DEFINED_HASH_ORDER
  static_assert(false, "not implemented")
#else
  m_pSilEdges->GetAllKeys(rSilEdges);
#endif
  return SM_SUCCESS;

} // end GetSilhouetteEdges

/*******************************************************************//**
PURPOSE:  Compute the visibility of a curve.  You may be given hints
  about the start and end point visibility.

NOTES: Visibility is stored in the SmCurveClassification.

METHOD ---
  rCurveClass->m_pCurve is projected intersected with all SceneBrep edges and vertices
    which breaks the rCurveClass object into a set of intervals
  Each interval is classified.  Intervals which are 
      visible: m_vMid.m_ePointClass = SM_PC_UNKNOWN
    invisible: m_vMid.m_ePointClass = SM_PC_REGION

***********************************************************************/
SmStatus SmHCR::ComputeCurveVisibility
  (SmCurveClassification & rCurveClass,  // i/o: curve to classify, gets classification
   ULONG                   lStartHint,   // in : 0 - no hint, 1 - visible, 2 - invisible
   ULONG                   lEndHint)     // in : 0 - no hint, 1 - visible, 2 - invisible
{
  // classify TgtCurve into intervals divided by projected curve and vertex xsects.
  // modifies the rCurveClass intervals.
  SER(IntersectCurveWithSilhouettes(rCurveClass));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if ( bDebugMe )
    { rCurveClass.Dump(); }
#endif

  // for every interval
  ULONG i, lNumIntervals = rCurveClass.GetSize();
  for(i=0; i<lNumIntervals; i++ )
    {
      SmCurveInterval & rCIvl = rCurveClass[i];

      // skip zero length intervals
      if ( SM_ARE_SAME( rCIvl.m_vInterval.GetMin(), rCIvl.m_vInterval.GetMax()) ) 
        { continue; }

      SmPoint3d sMidPnt;
#ifdef SM_DEBUG_CODE
static ULONG lCount    = 1 ; lCount++ ;
static ULONG lDebugCount = 0 ;
      // draw curve trim interval with all edges of the SceneBrep
      if ( bDebugMe || lCount == lDebugCount )
        {
          SmBSplineCurve *pTmp = new(m_crContext) SmBSplineCurve((SmBSplineCurve&)*(rCurveClass.GetCurve()));
          SmObjDelete sCleanTmp(pTmp);
          pTmp->Trim(rCIvl.m_vInterval);   // may snap sIvl by tol to existing knots
          pTmp->Transform(m_vInvViewTransform);   // what's going on with this transformation?
          SmTArray<SmEdge*> sEdges;
          m_pSceneBrep->GetEdges(sEdges);
          
          smgfx_Erase();
          smgfx_SetLook(2,2, 1,0,0); pTmp->DrawWDeriv(pTmp->GetNaturalInterval(),0); sm_GraphicsLoop();
          smgfx_SetLook( 1,2, 0,0,0);
          for (ULONG j=0; j<sEdges.GetSize(); j++) 
            { sEdges[j]->Draw(); sm_GraphicsLoop(); }
          sm_GraphicsLoop();
      }
#endif

      // Classify this curve interval
      SmBoolean bVisibility = FALSE;
     
      // when given end hints - use them to classify the interval
       if ( i == 0 && lStartHint != 0 )
        {
          if ( lStartHint == 1 ) { bVisibility = TRUE; }
        }
      else if ( i == lNumIntervals-1 && lEndHint != 0 )
        {
          if ( lEndHint == 1 )   { bVisibility = TRUE; }
        }
      else // classify the interval by a raycast from the interval midPoint
        {
          //      double sKData[256];
          //      SmTArray<double> sKnots(256,sKData);
          //      rCurveClass.GetCurve()->GetKnots(sKnots);
          SmExtent1d sIvl  = rCIvl.m_vInterval;
          double     dTMid = sIvl.Evaluate(0.534567);
          SER(rCurveClass.GetCurve()->EvaluatePoint(dTMid,sMidPnt));
          bVisibility = IsPointVisible(sMidPnt);
        }

      // when the interval is visible
      if ( bVisibility )
        {
          rCIvl.m_vMid.SetClassObject( SM_PC_UNKNOWN, NULL );
        }
      else // interval is not visible
        {
          // (Note, the object is not used, just put in something for completeness and safety.)
          rCIvl.m_vMid.SetClassObject( SM_PC_REGION, m_pSceneBrep->GetInfiniteRegion() );
        }
    } // end iter all intervals in rCurveClass

  return SM_SUCCESS;

} // end SmHCR::ComputeCurveVisibility

/*******************************************************************//**
PURPOSE: Compute and display the visibility of a single Brep.

NOTES: 
  Builds   m_pVisibleCurves   = list of all edge and silhouette curve intervals which are visible  
           m_pInvisibleCurves = list of all edge and silhouette curve intervals which are invisible

  all curves in the lists are copies of curves from the SceneBrep and are stored
  in the OrigBrep orientation.

  Propagates attributes from Edges or Faces attached to edges to the
          curve copies int m_pVisibleCurves and m_pInvisibleCurves

  increments unlocked mark value
***********************************************************************/
SmStatus SmHCR::ComputeGlobalVisibility()
{
  ULONG ii, jj;

  // Find silhouette curves on faces
  //   - compute all silhouette curves on surfaces and add those to SceneBrep as new edges
  //   - For every edge in SceneBrep
  //       if(SilhouetteEdge) place edge           in the m_pSilEdgeTree   and on the m_pSilEdges list
  //                          place edge->vertices in the m_pSilVertexTree and on the m_pSilVertices list
  //       if(BackFacingEdge) Place edge in m_pBackFacingEdges list 
  ComputeSilhouettes();
  
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smos_WriteBuffer(_T("Silhouette of Brep Done\n"));

      smgfx_Erase();

      // Draw sil edges, red.
      SmTArray< SmEdge* > sSilEdgesKeys;
      m_pSilEdges->GetAllKeys(sSilEdgesKeys);
      smgfx_SetLook( 2,4, 1,0,0 );
      for(jj = 0; jj < sSilEdgesKeys.GetSize(); jj++)
      {
        SmEdge *pEdge = sSilEdgesKeys[jj];
        if(pEdge) { pEdge->DrawWTransform( m_vInvViewTransform ); sm_GraphicsLoop(); }
      }

      // Draw sil vertices, green.
      smgfx_SetLook( 2,4, 0,1,0 );
      SmTArray< SmVertex* > sSilVerticesKeys;
      m_pSilVertices->GetAllKeys(sSilVerticesKeys);
      for(jj = 0; jj < sSilVerticesKeys.GetSize(); jj++)
      {
        SmVertex *pVertex = sSilVerticesKeys[jj];
        if(pVertex) { pVertex->Draw(); sm_GraphicsLoop(); }
      }

      // Draw back-facing edges, blue.
      smgfx_SetLook( 1,4, 0,0,1 );
      SmTArray< SmEdge* > sBackFacingEdgesKeys;
      m_pBackFacingEdges->GetAllKeys(sBackFacingEdgesKeys);
      for(jj = 0; jj < sBackFacingEdgesKeys.GetSize(); jj++)
      {
        SmEdge *pEdge = sBackFacingEdgesKeys[jj];
        if(pEdge) { pEdge->DrawWTransform( m_vInvViewTransform ); sm_GraphicsLoop(); }
      }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Attributes for curves: 
  //   for each edge, if it has an attribute,
  //                  or if one of its faces has an attribute,
  //   attach that attribute to the edge's curve.
  SmTArray<SmEdge*> sEdges;
  m_pSceneBrep->GetEdges(sEdges);

  SmTArray<SmAttribute*> sAttributes;
  ULONG nAttributes = 0;

  // for every SceneBrep Edge
  ULONG lNumEdges = sEdges.GetSize();
  for(ii=0; ii<lNumEdges; ii++ )
    {
      SmEdge *pEdge = sEdges[ii];

      pEdge->GetAttributes( sAttributes );
      nAttributes = sAttributes.GetSize();

      // when edge has no attributes look in attached faces
      if( nAttributes == 0 )
      {
          SmTArray<SmFace*> rFaces;
          pEdge->GetFaces(rFaces);

          // an edge may belong to more than 1 face....ambiguity
          // get the attribute of any face (or first) off this edge
           jj = 0;
           ULONG lNumFaces = rFaces.GetSize();
           while (nAttributes == 0 && jj < lNumFaces)
             {
               rFaces[jj++]->GetAttributes(sAttributes);
               nAttributes = sAttributes.GetSize();
             }
      }

      // when attributes were found, add all attributes to the curve
      if ( nAttributes > 0 )
      {
          SmCurve *pCurve = pEdge->GetCurve();
          for( jj = 0; jj < nAttributes; jj++ )
          {
              pCurve->AddAttribute( sAttributes[jj] );
          }
      }
    } // end iter every SceneBrep edge attaching color attributes to edge->curve


  // Disable editing now
  m_pSceneBrep->m_bEditingEnabled = FALSE;

#ifdef SM_DEBUG_CODE
  if (bDebugMe) 
    {
      m_pSceneBrep->WriteToFile(_T("./scene_brep.smb"), SM_ASCII, TRUE, FALSE, 0.0);

      // draw the SceneBrep
      smgfx_Erase();
      SmBoolean bHiddenCurveSave;
      ULONG lHCDashedSave;
      smgfx_GetHiddenCurve(bHiddenCurveSave,lHCDashedSave);
      smgfx_SetHiddenCurve(FALSE,1);

      smgfx_SetLook(1,2, 0,0,1) ; m_pSceneBrep->Draw(); sm_GraphicsLoop() ;
      sm_GraphicsLoop();
      smgfx_SetHiddenCurve(bHiddenCurveSave,lHCDashedSave);
    }
#endif // SM_DEBUG_CODE

  // We will put a mark:[eMarkType] on each edge that is not visible.
  // Note that SmHCR::ComputeEdgeVisibility puts a mark:[eMarkType] on each edge it processes.

  // increment and lock an unlocked mark
  SmNewMarkAndLock sMarkLock( m_pSceneBrep->GetContext(), SM_MT_ALLMARKS) ; // increment and lock any unlocked mark
  SmMarkType eMarkType = sMarkLock.GetMarkType() ;

  // Mark back-facing lines of faces parallel to vector
    {
      SmVector3d sLineVec, sBinVec;
      SmPoint3d  sLinePnt, sBinPnt;
      SmVector3d sZVector(0,0,-1);
      SmTArray<SmFace*> sFaces;
      SmTArray<SmLoopuse*> sLoopuses;
      SmTArray<SmEdgeuse*> sEdgeuses;

      // get all SceneBrep faces
      m_pSceneBrep->GetFaces(sFaces);
      ULONG lNumFaces = sFaces.GetSize();

      // Loop over faces looking for those parallel to view vector.
      for ( ii=0; ii<lNumFaces; ii++ )
        {
          SmFace    *pF       = sFaces[ii];
          SmSurface *pSurface = pF->GetSurface();

          // skip faces not parallel to viewing direction
          if ( ! pSurface->IsParallelToVector( sZVector ) )
            { continue; }

          // This face is parallel to the view vector.
          SmFaceuse *pFU = pF->GetUpwardFaceuse();
          pFU->GetLoopuses(sLoopuses);
          ULONG lNumLUs = sLoopuses.GetSize();

          // for every face->Loopuse
          for ( jj=0; jj<lNumLUs; jj++ )
            {
              SmLoopuse *pLU = sLoopuses[jj];

              pLU->GetEdgeuses(sEdgeuses);
              ULONG kk, lNumEUs = sEdgeuses.GetSize();

              // for every face->Loopuse->edgeuse
              for ( kk=0; kk<lNumEUs; kk++ )
                {
                  SmEdgeuse *pEU = sEdgeuses[kk];
                  SmEdge    *pE  = pEU->GetEdge();

                  if ( jj > 0 )  // Inner loops are not visible:
                    {
                      pE->Mark(eMarkType);
                    }
                  else  // Outer loop only: mark:[eMarkType] lines on the back side, or perpendicular.
                    {
                      SmCurve *pCurve = pE->GetCurve();

                      // We should do some processing for silhouettes of lamina edges here.

                      if(   pCurve 
                         && pCurve->IsLine(10,m_pSceneBrep->GetTolerance()/100.0,sLinePnt,sLineVec))
                        {
                          SER(pEU->EvaluateBinormal(pE->GetInterval().Evaluate(0.5),FALSE,sBinPnt, sBinVec));
                          if ( sBinVec.z > -SM_EFF_ZERO )
                            {
                              // Binormal is toward eye or perpendicular
                              // to the eye so the edge is invisible
                              pE->Mark( eMarkType );
                            }

                        } // end if curve is linear
                    } // end else (outer loop only)
                } // end iter all Edgeuses in this Loopuse (kk)
            } // end iter all Loopuses (jj)
        } // end iter all Faces (ii)
    } // end local scope, marking:[eMarkType] back-facing linear edges of parallel faces.


  // Now process the edges that we have marked as invisible.
  // For each invisible edge: copy it, trim the copy to Edge interval,
  // and add it to m_pInvisibleCurves array.
  m_pSceneBrep->GetEdges(sEdges);
  lNumEdges = sEdges.GetSize();
  ULONG lModVal = lNumEdges / 100;
  if ( lModVal < 10 ) { lModVal = 10; }

  // for every SceneBrep edge
  for ( ii=0; ii<lNumEdges; ii++ )
    {
      SmEdge *pE = sEdges[ii];

      // for marked edges
      if ( pE->IsMarked( eMarkType ))
        {
          // copy the EdgeCurve
          SmCurve        *pCurve = pE->GetCurve();
          SmCurve * pNewCurve;
          pCurve->Copy( m_crContext, pNewCurve );

          // trim the EdgeCurve
          SmExtent1d sIvl = pE->GetInterval();
          SER(pNewCurve->Trim(sIvl));  // may snap sIvl by tol to existing knots

          // transform it back to the original Brep orientation
          SER( pNewCurve->Transform( m_vInvViewTransform ));

          // and save it
          m_pInvisibleCurves->Add(pNewCurve);

#ifdef SM_DEBUG_CODE
          if ( bDebugMe ) {
              smgfx_SetLook( 4,6, 1,0,1 ); pNewCurve->Draw(); sm_GraphicsLoop();
              sm_GraphicsLoop();
          }
#endif

        } // end edge is marked:[eMarkType] check
    } // end iter every SceneBrep edge

  // Now compute visibility for every edge that has not been marked as invisible.
  // ComputeEdgeVisibility() fills in m_pVisibleCurves and m_pInvisibleCurves.
  for ( ii=0; ii<lNumEdges; ii++ )
    {
      SmEdge *pE = sEdges[ii];

      // If For Silhouettes, skip edges on the m_pSilEdges list.
      if ( m_bHCRForSilhouettes )
        {
          void* pVoid = (void*) m_pSilEdges->At(pE);
          if ( pVoid == NULL ) { continue; }
        }

      // Compute Edge Visibility for unmarked Edges.
      if ( ! pE->IsMarked( eMarkType ))
        {
          SER( ComputeEdgeVisibility( pE, NULL, FALSE, eMarkType ));
        }
      
#ifdef SM_DEBUG_CODE
      if( bDebugMe ) 
        {
          // inform the public
          if(  (ii+1) % lModVal == 0  || ii == lNumEdges-1 )
            {
              TCHAR sBuff[SM_TBLOCK_SIZE];
              smos_sprintf(sBuff,_T(" Edge # %ld of %ld Completed\n"),(ii+1),lNumEdges);
              smos_WriteBuffer(sBuff);
            }
        }
#endif // SM_DEBUG_CODE
    } // end iter every SceneBrep edge

  return SM_SUCCESS;

} // end SmHCR::ComputeGlobalVisibility

/*******************************************************************//**
PURPOSE: Compute the visibility of an edge.

NOTES:
   Uses mark:[eMarkType]: sets it on the given edge, and does not process
                          the edge if it is already set.

METHOD --- projected intersects the pEdgeArg->Curve with all SceneBrep
           vertices and edges to divide a curve classification into a sequence of intervals.
           Each interval is classified as visible or invisible.
           
           A trimmed copy of each curve interval is placed
             on m_pVisibleCurves and m_pInvisibleCurves lists.
           IN THE ORIENTATION OF THE ORIGINAL BREP.

           The curve and the intersections are all done in the SceneBrep orientation.
***********************************************************************/
SmStatus SmHCR::ComputeEdgeVisibility
  (SmEdge    * pEdgeArg,             // in : Target Edge, wkipped when marked, gets marked
   SmVertex  * pKnownVertexArg,      // in : can be NULL to start
   SmBoolean   bVertexIsVisibleArg,  // in : when pKnownVertexArg==NULL set to FALSE
   SmMarkType  eMarkType)            // in : MarkType to use when marking
{
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
SmBoolean bErase   = FALSE;
  if (bDebugMe) 
    {
      // draw the target edge, black
      if ( bErase )
        { smgfx_Erase(); }

      SmBoolean bDashedSave = smgfx_GetDashedLines();
      smgfx_SetDashedLines(TRUE);
      smgfx_SetLook( 1,2, 0,0,0 ); pEdgeArg->GetCurve()->DrawWDeriv(pEdgeArg->GetInterval(),0); sm_GraphicsLoop();
      sm_GraphicsLoop();

      smgfx_SetDashedLines(bDashedSave);
    }
#endif // SM_DEBUG_CODE

  // locals
  SmEdge         * pEData[64];
  SmCurveInterval  pData[64];
  SmEdge         * sEData[256];
  SmVertex       * sVData[256];
  SmBoolean        sBData[256];
  SmTArray<SmEdge*>   sVertEdges         (64, pEData);
  SmTArray<SmEdge*>   sEdges             (256,sEData);
  SmTArray<SmVertex*> sKnownVertices     (256,sVData);
  SmTArray<SmBoolean> sVerticesAreVisible(256,sBData);

  // (if FIX_ME) Work this like a stack.
  sEdges.Add(pEdgeArg);
  sKnownVertices.Add(pKnownVertexArg);
  sVerticesAreVisible.Add(bVertexIsVisibleArg);

  // (if FIX_ME) while there are edges in the stack
  // (else)      just once through this while loop
  while (sEdges.GetSize() > 0)
    {
      SmEdge   * pEdge            = sEdges             .GetLast();  sEdges             .RemoveLast();
      SmVertex * pKnownVertex     = sKnownVertices     .GetLast();  sKnownVertices     .RemoveLast();
      SmBoolean  bVertexIsVisible = sVerticesAreVisible.GetLast();  sVerticesAreVisible.RemoveLast();

      // Stop if we have already touched this edge
      if ( pEdge->IsMarked( eMarkType )) 
        { return SM_SUCCESS; }

      // Touch this edge.
      pEdge->Mark( eMarkType );

      // If this edge is back facing, label it invisible.
      SmBoolean bVisibleEdge = TRUE;
      SmCurve  *pCurve       = pEdge->GetCurve();
      void     *pVoid        = (void*) m_pBackFacingEdges->At(pEdge);
      if ( pVoid != NULL )
        { bVisibleEdge = FALSE; }  // Is a backfacing face.

      // when not displaying seams, make seams invisible except for silhouette edges
      if ( bVisibleEdge && !m_bDisplaySeams && pEdge->IsSeam() )
        { bVisibleEdge = IsSilhouetteEdge( pEdge ); }

      // when not displaying G1 smooth edges, make G1 smooth edges invisible except for silhouette edges
      if ( bVisibleEdge && !m_bDisplaySmoothEdges && pEdge->IsTangentEdge(2.0) )
        { bVisibleEdge = IsSilhouetteEdge( pEdge ); }

      // when the edge is not Visible
      if ( ! bVisibleEdge )
        {
          // copy and trim the edge
          SmCurve *pNewCurve; 
          pCurve->Copy( m_crContext, pNewCurve );
          SmExtent1d      sIvl = pEdge->GetInterval();
          SER(pNewCurve->Trim(sIvl));   // may snap sIvl by tol to existing knots

          // transform it back into OrigBrep orientation
          SER(pNewCurve->Transform(m_vInvViewTransform));

          // add to the working list
          m_pInvisibleCurves->Add(pNewCurve);

#ifdef SM_DEBUG_CODE
          if (bDebugMe) 
            {
              // draw the already-known-to-be-invisible curve, magenta
              smgfx_SetLook(2,4, 1,0,1 );
              SmBoolean bDashedSave = smgfx_GetDashedLines();
              smgfx_SetDashedLines(TRUE);
              pNewCurve->DrawWDeriv(pNewCurve->GetNaturalInterval(),0); sm_GraphicsLoop();
              smgfx_SetDashedLines(bDashedSave);
              sm_GraphicsLoop();
            }
#endif
          // all done
          return SM_SUCCESS;
        
        } // end edge is not visible check

      ULONG lStartHint = 0; // No Hint initially for start or end of curve
      ULONG lEndHint   = 0;

      // If we have a known vertex and its visibilty
      //  store a StartHint or EndHint for the curve visibility
      if ( pKnownVertex )
        {
          SmEdgeuse *pPrimEU = pEdge->GetPrimaryEdgeuse();

          // vertex is at edge start
          if ( pPrimEU->GetVertexuse()->GetVertex() == pKnownVertex )
            {
              if ( pPrimEU->GetOrientation() == SM_OT_SAME )
                {
                  if ( bVertexIsVisible ) { lStartHint = 1; } // Visible
                  else                    { lStartHint = 2; } // Invisible
                }
              else
                {
                  if ( bVertexIsVisible ) { lEndHint = 1; } // Visible
                  else                    { lEndHint = 2; } // Invisible
                }
            }

          // vertex is at edge end
          if ( pPrimEU->GetMate()->GetVertexuse()->GetVertex() == pKnownVertex )
            {
              if ( pPrimEU->GetMate()->GetOrientation() == SM_OT_SAME )
                {
                  if ( bVertexIsVisible ) { lStartHint = 1; } // Visible
                  else                    { lStartHint = 2; } // Invisible
                }
              else
                {
                  if ( bVertexIsVisible ) { lEndHint = 1; } // Visible
                  else                    { lEndHint = 2; } // Invisible
                }
            }
        } // end have a pKnownVertex visibility check

#ifdef FIX_ME
      int iStartVisibility = 0; // Unknown visibility
      int iEndVisibility   = 0;
#endif

      // CurveClassification whose intervals will eventually be classified as visible or not
      SmCurveClassification sCurveClass( pCurve, 
                                         pEdge->GetInterval(), 
                                         NULL,
                                         pEdge->GetTolerance(), 
                                         64, 
                                         pData );

      // divide Classification at curve SceneBrep projected xsects and classify every interval
      SER( ComputeCurveVisibility( sCurveClass, lStartHint, lEndHint ));

      // for every interval
      ULONG ii, lNumIntervals = sCurveClass.GetSize();
      for(ii=0; ii<lNumIntervals; ii++ )
        {
          SmCurveInterval & rCIvl = sCurveClass[ii];

          // skip zero length intervals
          if ( SM_ARE_SAME( rCIvl.m_vInterval.GetMax(), rCIvl.m_vInterval.GetMin() ))
            { continue; }

          // When interval is visible
          // (ComputeCurveVisibility() sets the intervals' Midpoints to:
          // - SM_PC_UNKNOWN for Visible
          // - SM_PC_REGION  for Invisible
          if ( rCIvl.m_vMid.GetPointClass() == SM_PC_UNKNOWN )
            {
              // when interval is nonZero length
              if ( rCIvl.m_vInterval.GetLength() > SM_EFF_ZERO_SQRT )
                {
#ifdef FIX_ME
                  // remember when first interval is visible
                  if ( ii == 0 )
                    { iStartVisibility = 1; }

                  // when interval start point is not an intersection
                  if ( rCIvl.m_vStart.GetPointClass() == SM_PC_UNKNOWN )
                    {
                      // remember when last interval is visible
                      if ( ii == lNumIntervals-1 )
                        { iEndVisibility = 1; }
                    }

                  // when interval start point is a vertex intersection
                  if ( rCIvl.m_vStart.GetPointClass() == SM_PC_VERTEX )
                    {
                      SmVertex *pV  = (SmVertex*)rCIvl.m_vStart.GetObject();
                      SmVertex *pV1 = pEdge->GetVertex();
                      SmVertex *pV2 = pEdge->GetOtherVertex(pV1);

                      // remember last and first interval visibility
                      if ( pV == pV1 || pV == pV2 )
                        {
                          if ( ii == 0 )               { iStartVisibility = 1; }
                          if ( ii == lNumIntervals-1 ) { iEndVisibility   = 1; }
                        }
                    }
#endif

                  // arrive here for a Visible segment of curve.

                  // copy curve and trim to segment
                  SmCurve *pNewCurve; 
                  pCurve->Copy( m_crContext, pNewCurve );
                  SER(pNewCurve->Trim(rCIvl.m_vInterval));  // may snap sIvl by tol to existing knots

                  // transform curve from SceneBrep to OrigBrep orientation
                  SER(pNewCurve->Transform(m_vInvViewTransform));

                  // add it to the working visible curves list
                  m_pVisibleCurves->Add(pNewCurve);

#ifdef SM_DEBUG_CODE
                  if (bDebugMe) 
                    {
                      // draw the visible curve, red
                      sm_GraphicsLoop();
                      smgfx_SetLook(2,4, 1,0,0); pNewCurve->DrawWDeriv(pNewCurve->GetNaturalInterval(),0); sm_GraphicsLoop();
                      sm_GraphicsLoop();
                    }
#endif
                } // end if this interval has length
            } // end visible interval branch
          else // interval is not visible
            {
              // This interval's mid is classified to some topology (not UNKNOWN).

              // when interval is nonZero length 
              if ( rCIvl.m_vInterval.GetLength() > SM_EFF_ZERO_SQRT )
                {
                  // copy and trim the interval curve
                  SmCurve * pNewCurve;
                  pCurve->Copy( m_crContext, pNewCurve );
                  SER(pNewCurve->Trim(rCIvl.m_vInterval));  // may snap sIvl by tol to existing knots

#ifdef FIX_ME
                  // Set visibilities to invisible
                  if ( rCIvl.m_vEnd.GetPointClass() == SM_PC_UNKNOWN )
                    {
                      if (ii == 0)               { iStartVisibility = 2; }
                      if (ii == lNumIntervals-1) { iEndVisibility   = 2; }
                    }

                  // set interval boundary vertex to invisible
                  if ( rCIvl.m_vEnd.GetPointClass() == SM_PC_VERTEX )
                    {
                      SmVertex *pV  = (SmVertex*)rCIvl.m_vEnd.GetObject();
                      SmVertex *pV1 = pEdge->GetVertex();
                      SmVertex *pV2 = pEdge->GetOtherVertex(pV1);
                      if ( pV == pV1 || pV == pV2 )
                        {
                          if ( ii == 0 )               { iStartVisibility = 2; }
                          if ( ii == lNumIntervals-1 ) { iEndVisibility   = 2; }
                        }
                    }
#endif

                  // transform the curve from SceneBrep to OrigBrep orientation
                  SER(pNewCurve->Transform(m_vInvViewTransform));

                  // add it to the working invisible curves list
                  m_pInvisibleCurves->Add(pNewCurve);

#ifdef SM_DEBUG_CODE
                  if (bDebugMe) 
                    {
                      // draw the invisible curve, blue
                      SmBoolean bDashedSave = smgfx_GetDashedLines();
                      smgfx_SetDashedLines(TRUE);
                      smgfx_SetLook(2,4, 0,0,1); pNewCurve->DrawWDeriv(pNewCurve->GetNaturalInterval(),0); sm_GraphicsLoop();
                      sm_GraphicsLoop();

                      smgfx_SetDashedLines(bDashedSave);
                    }
#endif
                } // end interval is nonZero length check

            } // end else, interval's mid is classified to some topology
        } // end iter every sCurveClass intervals


#ifdef FIX_ME
      // See about propagating visibility through the vertices.

      SmVertex *pStartV, *pEndV;
      SmEdgeuse *pPrimEU = pEdge->GetPrimaryEdgeuse();
      if ( pPrimEU->GetOrientation() == SM_OT_SAME )
        {
          pStartV = pPrimEU->GetVertexuse()->GetVertex();
          pEndV   = pPrimEU->GetMate()->GetVertexuse()->GetVertex();
        }
      else
        {
          pEndV   = pPrimEU->GetVertexuse()->GetVertex();
          pStartV = pPrimEU->GetMate()->GetVertexuse()->GetVertex();
        }

      if ( iStartVisibility != 0 )
        {
          // Propagate start visibility through the start vertex.
          // If everything is nice and manifold
          SmBoolean bManifold = TRUE;
          pStartV->GetEdges(sVertEdges);
          ULONG lNumEdges = sVertEdges.GetSize();
          for(ii=0; ii<lNumEdges; ii++ )
            {
              SmEdge* pVertEdge = sVertEdges[ii];
              if ( ! pVertEdge->IsManifold() )
                {
                  bManifold = FALSE;
                  break;
                }
            }
          if ( bManifold && lNumEdges < 5 )
            {
              for(ii=0; ii<lNumEdges; ii++ )
                {
                  sKnownVertices.Add( pStartV );

                  SmEdge* pVertEdge = sVertEdges[ii];
                  sEdges.Add( pVertEdge );

                  SmBoolean bVertVisible = ( iStartVisibility == 1 );
                  sVerticesAreVisible.Add( bVertVisible );
                }
            }
        }
      if ( iEndVisibility && pStartV != pEndV )
        {
          // Propagate end the visibility through the End vertex
          // If everything is nice and manifold
          pEndV->GetEdges(sVertEdges);
          SmBoolean bManifold = TRUE;
          ULONG lNumEdges = sVertEdges.GetSize();
          for(ii=0; ii<lNumEdges; ii++ )
            {
              SmEdge* pVertEdge = sVertEdges[ii];
              if ( ! pVertEdge->IsManifold() )
                {
                  bManifold = FALSE;
                  break;
                }
            }
          if ( bManifold && lNumEdges < 5 )
            {
              for(ii=0; ii<lNumEdges; ii++ )
                {
                  sKnownVertices.Add(pEndV);

                  SmEdge* pVertEdge = sVertEdges[ii];
                  sEdges.Add(pVertEdge);

                  SmBoolean bVertVisible = ( iEndVisibility == 1 );
                  sVerticesAreVisible.Add(bVertVisible);
                }
            }
        }
#endif // FIX_ME
   } // end while there are edges in the stack

  return SM_SUCCESS;

} // end SmHCR::ComputeEdgeVisibility

/*******************************************************************//**
PURPOSE: Compute silhouette curves.  Create corresponding edges and
  vertices and add to tables and edge/vertex trees.

NOTES:
  Sets: m_pSilVertices, m_pSilEdges, m_pSilVertexTree, m_pSilEdgeTree.
  Sets topology in m_pSceneBrep.

METHOD ---
  Find all silhouette curves on every surface.
  Add those to SceneBrep as new edges on faces.
     Place new edges    in the m_pSilEdgeTree   and on the m_pSilEdges list
     Place new vertices in the m_pSilVertexTree and on the m_pSilVertices list
  For Every Edge in SceneBrep
     If(Edge is SilhouetteEdge) 
         Place edge     in the m_pSilEdgeTree   and on the m_pSilEdges list
         Place vertices in the m_pSilVertexTree and on the m_pSilVertices list
     else if(Edge is backFaceing edge) 
         Place edge in m_pBackFacingEdges list
***********************************************************************/
SmStatus SmHCR::ComputeSilhouettes()
{
  // locals
  SmTArray<SmFace*>  sNewFaces;
  SmTArray<SmCurve*> s3DCurves;
  SmTArray<SmCurve*> sUVCurves;
  SmTArray<SmCurve*> sUVBSCurves;
  SmTArray<SmSurface*>      sSurfs(128);
  SmTArray<SmEdge*>         sEdges;
  SmStatus                  eStat;
   ULONG ii, jj, lNumSurfs ;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  ULONG kk;
  SmTArray< SmFace* > sDbgFaces;
  m_pOriginalBrep->GetFaces(sDbgFaces);

  // draw Original Brep faces
  if (bDebugMe) 
    {
      m_pOriginalBrep->GetEdges(sEdges);
      for(kk=0; kk<sEdges.GetSize(); kk++ ) 
        {
          sEdges[kk]->Draw();
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Get SceneBrep Surfaces
  m_pSceneBrep->GetSurfaces(sSurfs);
  lNumSurfs = sSurfs.GetSize();

  // for every surface
  for ( ii=0; ii<lNumSurfs; ii++ )
    {
      SmBSplineSurface *pSurface = SM_CAST_PTR(SmBSplineSurface,sSurfs[ii]);
      NER(pSurface);

      // skip Planes - they don't have silhouettes.
      if ( pSurface->IsKindOf( SmPlane_TYPE )) 
       { continue; }

      // Get Surface->Face
      SmFace *pFace = (SmFace*)pSurface->GetFace();

      // make s3DCurves and sUVCurves items all temporary
      SmObjsDelete<SmCurve*> sClean1(&s3DCurves);
      SmObjsDelete<SmCurve*> sClean2(&sUVCurves);

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe2 = FALSE;
SmBoolean bErase   = FALSE;
SmBoolean bWrite   = FALSE;
          // draw target surface
          if (bDebugMe2) 
            {
              if ( bWrite ) {
                  pSurface->Dump();
                  pSurface->WriteToFile(_T("./sil_surface.sms"), true, true);
              }

              if ( bErase )
              {
                smgfx_Erase();
                //smgfx_SetLook( 1,2, 0,0,0 ); m_pSceneBrep->Draw(TRUE); sm_GraphicsLoop();
                m_pSceneBrep->GetEdges(sEdges);
                for(kk=0; kk<sEdges.GetSize(); kk++ ) 
                  { sEdges[kk]->Draw(); }
              }
              //smgfx_SetLook( 2,2, 0,0,0 ); pSurface->DrawUV(2,2,FALSE,&(pFace->GetUVDomain())); sm_GraphicsLoop();
              smgfx_SetLook( 2,4, 1,1,0 ); pSurface->DrawUV(3,3); sm_GraphicsLoop();
              //smgfx_SetLook( 1,2, 0,0,1 ); sDbgFaces[ii]->GetSurface()->DrawUV(3,3); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

      // pick tol to limit segment sizes in DoTRace 
      double dSilTol = smos_Min( m_dThisApproxTol3d, m_dRayOffsetTolerance/100.0 );

      // walk the silhouttte curves for this curves and this viewVector
      eStat = pSurface->CreateSilhouetteCurves( m_crContext, 
                                                pFace->GetUVDomain(),
                                                m_vViewVector, 
                                                FALSE,
                                                SM_CAST_APPROXTOL3D_PTR(&dSilTol), 
                                                &m_dThisAngTolRad, 
                                                &s3DCurves,    // out: 3d silhouette curve
                                                &sUVCurves );  // out: 3d silhouette UVcurve
      if ( eStat != SM_SUCCESS )
        { continue; }

#ifdef SM_DEBUG_CODE
      if (bDebugMe2)  {
          smgfx_SetLook( 3,5, 1,0,0 );
          for ( jj=0; jj<s3DCurves.GetSize(); jj++ )
            { s3DCurves[jj]->Draw(); sm_GraphicsLoop(); }
          sm_GraphicsLoop();
      }
#endif // SM_DEBUG_CODE

      // store silhouette UVcurves as SmBSpline UVCurves
      sUVBSCurves.ReSet();
      ULONG lNumCurves = sUVCurves.GetSize();
      for ( jj=0; jj<lNumCurves; jj++ )
        { sUVBSCurves.Add(sUVCurves[jj] ); }

      // Turn the curves into Edges in this surface's face.
      eStat = pFace->GetBrep()->MergeCurvesOnSurface
                ( *pSurface,                  // in : Surface must be owned by a face in this Brep.
                   m_dThisApproxTol3d,        // in : 3D Tol for curves greater than MaxCrvSrfGap and MaxCrvTrimCrvGap.
                   s3DCurves,                 // in : 3D Curves on surface not necessarily trimmed to face boundary.
                                              //      Curves should not cross each other.
                   &sUVBSCurves,              // in : associated UV Curves having a dim of 2.
                   sNewFaces,                 // out: Newly created faces, if any.
                   sEdges,                    // out: Brep Edges for cr3DCurves (new and/or existing) 
                                              //      ordered:[CurveParameritization]
                   TRUE );                    // in : TRUE = use projection for intersections to allow for tolerances.
                                              //      FALSE= no projections for faces already toleranced. default:[FALSE]
                                              // in : OtherSurface when cr3DCurves are from surf/surf XSects. 
                                              //      As curves are merged into Brep - surf/surf/surf XSects are
                                              //      used for new verts and to improve Edge->Curve interpolation.
                                              //      NULL to ignore, default:[NULL], NULL creates larger gaps.
                                              // in : TRUE = try UVSpace Classification if 3Space try is dodgey (tolerant cases)
                                              //      FALSE= don't try UVSpace because UVTrimCurves aren't yet valid, default:[TRUE]
      if ( eStat != SM_SUCCESS )
        { continue; }

      // For each new silhouette edge,
      // if it split the face (or is part of a series of edges that
      // split the face), add it and its vertices to our lists.
      // Or, add it to m_pBackFacingEdges if appropriate.

      // for every new silhouette edge
      ULONG lNumEdges = sEdges.GetSize();
      for(jj=0; jj<lNumEdges; jj++ )
        {
          SmEdge *pEdge = sEdges[jj];
          pEdge->GetFaces( sNewFaces );

          // If new silhouette edge split the face, sNewFaces will have two faces, each on the same surface.

          // skip edges that don't split faces
          if(   sNewFaces.GetSize()        != 2 
             || sNewFaces[0]->GetSurface() != sNewFaces[1]->GetSurface())                                 
            { continue; }

#ifdef SM_DEBUG_CODE
          if (bDebugMe2) 
            {
              // draw new silhouette Edge (with InvViewTransform), Draw the TgtSurface
              sm_GraphicsLoop();
              smgfx_SetLook( 2,3, 1,0,0 ); pEdge->DrawWTransform(m_vInvViewTransform); sm_GraphicsLoop();
              if ( bErase ) 
                { SmExtent2d domain = pFace->GetUVDomain(); 

                  smgfx_Erase();
                  smgfx_SetLook( 1,2, 0,1,1 ); pSurface->DrawUV(2,2,FALSE,&domain); sm_GraphicsLoop();
                  smgfx_SetLook( 2,3, 1,0,0 ); pEdge->DrawWTransform(m_vInvViewTransform); sm_GraphicsLoop();
                }
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // Note: IsConcaveSilhouette() can give false positives
          // for certain surfaces with negative Gaussian curvature.
          // Moreover, such a curve will be hidden by other faces anyway
          // and so will ultimately be removed from the display. [091023]
          // if(IsConcaveSilhouette( pEdge ) )
          //   {
          //     m_pBackFacingEdges->SetAt(pEdge,pEdge);
          //  }
          
          // new silhouette edge locals
          SmVertex  *pV1 = pEdge->GetVertex();
          SmVertex  *pV2 = pEdge->GetOtherVertex(pV1);
          SmExtent3d sVBBox1(pV1->GetPoint());
          SmExtent3d sVBBox2(pV2->GetPoint());
          SmExtent3d sEBBox;
          SE(pEdge->GetCurve()->CalculateBoundingBox(pEdge->GetInterval(),&sEBBox));
          
          // Add edge to SmHCR's newEdgeList and edge SpatialTree.
          m_pSilEdges->Insert(pEdge,pEdge);
          SE(m_pSilEdgeTree->AddToSpatialTree(sEBBox,pEdge));

          // Add vertices to SmHCR's newVertexList and Vertex SpatialTree.
          m_pSilVertices->Insert(pV1,pV1);
          m_pSilVertices->Insert(pV2,pV2);
          SE(m_pSilVertexTree->AddToSpatialTree(sVBBox1,pV1));
          SE(m_pSilVertexTree->AddToSpatialTree(sVBBox2,pV2));
           
        } // end iter every new silhouette edge created for this surface with this viewing dir
    } // end iter every surface

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe3  = FALSE;
if(bDebugMe3)
{
  SmTArray< SmEdge* > sKeys;
  m_pOriginalBrep->GetEdges( sEdges );
  m_pSilEdges->GetAllKeys( sKeys );

  smgfx_Erase();
  smgfx_SetLook( 1, 2, 0, 0, 0 ); 
  for(kk = 0; kk < sEdges.GetSize(); kk++)
  {
    sEdges[kk]->Draw(); sm_GraphicsLoop();
  }
  smgfx_SetLook( 2, 4, 1, 0, 0 ); 
  for(kk = 0; kk < sKeys.GetSize(); kk++)
  {
    SmEdge  *pEdge = sKeys[kk];
    if(pEdge) { pEdge->DrawWTransform( m_vInvViewTransform ); sm_GraphicsLoop(); }
  }
  sm_GraphicsLoop();
}
#endif // SM_DEBUG_CODE

  // done with building (and adding) silhouette curves for every surface

  // classify every edge in m_pSceneBrep
  // and place it and its vertices in m_pSilEdges, m_pSilEdgeTree, m_pSilVertices, and m_pSilVertexTree
  // or in m_pBackFacingEdges, as appropriate.

  // get all edges
  //  GWC: must be done here.  Adding surface silhouette edges may have split SceneBrep edges
  m_pSceneBrep->GetEdges(sEdges);
  ULONG lNumEdges = sEdges.GetSize();

  // for every SceneBrep Edge
  for(ii=0; ii<lNumEdges; ii++ )
    {
      SmEdge *pEdge = sEdges[ii];

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe4  = FALSE;
      if (bDebugMe4) 
        {
          // draw the tgt edge
          smgfx_SetLook( 3,4, 0,0,1 ); pEdge->DrawWTransform(m_vInvViewTransform); sm_GraphicsLoop();
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // when the edge classifies as a silhouette
      if ( IsSilhouetteEdge( pEdge ) )
        {

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe5 = FALSE;
          if (bDebugMe5) 
            {
              // draw the tgt edge
              smgfx_SetLook( 4,6, 1,0,1 ); pEdge->DrawWTransform(m_vInvViewTransform); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE
          SmVertex *pV1 = pEdge->GetVertex();
          SmVertex *pV2 = pEdge->GetOtherVertex(pV1);
          SmExtent3d sEBBox;
          SE(pEdge->GetCurve()->CalculateBoundingBox(pEdge->GetInterval(),&sEBBox));
          
          // add edge to m_pSilEdges and m_pSilEdgeTree
          m_pSilEdges->Insert(pEdge,pEdge);
          SE(m_pSilEdgeTree->AddToSpatialTree(sEBBox,pEdge));

          // If these are Null, it's some kind of error, but don't crash. [B90]
          if ( pV1 != NULL )
            {
              SmExtent3d sVBBox1(pV1->GetPoint());

              m_pSilVertices->Insert(pV1,pV1);
              SE(m_pSilVertexTree->AddToSpatialTree(sVBBox1,pV1));
            }

          if ( pV2 != NULL )
            {
              SmExtent3d sVBBox2(pV2->GetPoint());

              m_pSilVertices->Insert(pV2,pV2);
              SE(m_pSilVertexTree->AddToSpatialTree(sVBBox2,pV2));
            }
        } // end silhouette edge branch

      else if ( IsBackFacingEdge( pEdge ) )
        {
          // add backfacing edges to m_pBackFacingEdges
          m_pBackFacingEdges->Insert(pEdge,pEdge);

#ifdef SM_DEBUG_CODE
          if (bDebugMe4) 
            {
              // draw the back facing edge
              smgfx_SetLook( 2,2, 0,0,1 ); pEdge->DrawWTransform(m_vInvViewTransform); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif
        } // end BackFacingEdge branch
    } // end iter every SceneBrep edge

  // all done
  return SM_SUCCESS;

} // end SmHCR::ComputeSilhouettes


/*******************************************************************//**
PURPOSE: Return TRUE when an edge is a silhouette edge.

NOTES: Determines if edge is Silhouette or not based on the
  geometry of the faces to which it attaches.
***********************************************************************/
SmBoolean SmHCR::IsSilhouetteEdge
  (SmEdge *pEdge)
{
  // low work - edge already classified
  void *pVoid = m_pSilEdges->At( pEdge );
  if ( pVoid != NULL )
    { return TRUE; }

  // Wire edges are not silhouettes; Lamina edges are.
  if ( pEdge->IsWire()   ) { return FALSE; }
  if ( pEdge->IsLamina() ) { return TRUE; }

  // arrive here for manifold and spine edge.

  // locals
  SmEdgeuse *pData[32];
  SmTArray<SmEdgeuse*> sEdgeuses(32,pData);

  // get Edge->Edgeuses
  pEdge->GetEdgeuses(sEdgeuses);

  // Look at normals and binormals into both faces.
  SmEdgeuse *pPrimeEU  = sEdgeuses[0];
  SmEdgeuse *pEURadial = pPrimeEU->GetRadial();
  double dT = pEdge->GetInterval().Evaluate(0.5123);
  SmPoint3d sPnt;
  SmVector3d sBin1, sBin2, sNorm1, sNorm2, sTan1;
  SE( pPrimeEU ->EvaluateBinormal( dT, FALSE, sPnt, sBin1, &sTan1, &sNorm1 ) );
  SE( pEURadial->EvaluateBinormal( dT, FALSE, sPnt, sBin2, NULL,   &sNorm2 ) );

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      //draw Edge and its evaluations
      SmVector3d sTPnt, sTBin1, sTBin2, sTNorm1, sTNorm2;
      m_vInvViewTransform.TransformPoint (sPnt,  sTPnt);
      m_vInvViewTransform.TransformVector(sBin1, sTBin1);
      m_vInvViewTransform.TransformVector(sBin2, sTBin2);
      m_vInvViewTransform.TransformVector(sNorm1,sTNorm1);
      m_vInvViewTransform.TransformVector(sNorm2,sTNorm2);
      
      smgfx_SetLook(1,2, 0,0,0); pEdge->DrawWTransform(m_vInvViewTransform); sm_GraphicsLoop();
      smgfx_SetLook(3,4, 0,0,1); sTPnt  .Draw();       sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,0); (3*sTBin1 ).Draw(&sTPnt); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 0,1,1); (3*sTNorm1).Draw(&sTPnt); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,0,0); (3*sTBin2 ).Draw(&sTPnt); sm_GraphicsLoop();
      smgfx_SetLook(1,2, 1,1,0); (3*sTNorm2).Draw(&sTPnt); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // If both dot products are substantial, and point in generally the
  // same direction w.r.t. the view vector, then it's not a silhouette.
  if ( sNorm1.Dot(m_vViewVector) * sNorm2.Dot(m_vViewVector) >= SM_EFF_ZERO_SQRT )
    { return FALSE; }

  // Either one of the dot products is very small,
  // or they are in opposite directions w.r.t. the view vector,
  // so it might be a silhouette curve.

  // Now return True only if it is not concave silhouette case.
  // Note: should check infinite region, not void/solid,
  // because any internal voids will not be visible.
  // For visibility, all we care about is infinite region vs. any other region.
  // SmBoolean bVoidThisSide  = pPrimeEU->GetShell()->GetRegion()->IsVoid();
  // SmBoolean bVoidOtherSide = pPrimeEU->GetMate()->GetShell()->GetRegion()->IsVoid();

  SmRegion *pInfRegion    = m_pSceneBrep->GetInfiniteRegion();
  SmBoolean bInfThisSide  = ( pPrimeEU->GetShell()->GetRegion() == pInfRegion );
  SmBoolean bInfOtherSide = ( pPrimeEU->GetMate()->GetShell()->GetRegion() == pInfRegion );

  // If neither side is the infinite region then it can't be a silhouette.
  // NOTE - other problems caused errors and needed to return TRUE temporarily.
  if ( ! bInfThisSide && ! bInfOtherSide )
    { return FALSE; }

  // If both sides are the infinite region then it must be a silhouette.
  if ( bInfThisSide && bInfOtherSide )
    { return TRUE; }

  // If edge bounds a solid then do sectoring to see what the story is.

  // Note: 
  //  sProjViewVec will be the vector that's in the same plane as these two vectors,
  //  perp to sTan1, in the general direction of m_vViewVector.
  // It will be zero iff the vectors line up (parallel or anti-parallel).
  SmVector3d sProjViewVec = sTan1 * m_vViewVector * sTan1;
  if ( sProjViewVec.LengthSquared() < SM_EFF_ZERO_SQ )
    { return FALSE; } // curve points in direction of view

  // Do an angle test to see if vector is same as one of binormals.
  double dAngle;
  SER( sProjViewVec.AngleBetween( sBin1, dAngle ) );
  if ( dAngle < 0.01*SM_PI/180.0 )
    { return TRUE; }

  SER( sProjViewVec.AngleBetween( sBin2, dAngle ) );
  if ( dAngle < 0.01*SM_PI/180.0 )
    { return TRUE; }

  // Return FALSE if it is an inside edge.
  // Make sNorm1 point into the solid region.
  if ( bInfThisSide )
    { sNorm1 = - sNorm1; }

  // 
  if ( sProjViewVec.IsInsideSector( sBin1, sNorm1, sBin2 ) )
    { return FALSE; }

  // arrive here when the edge is a silhouette
  return TRUE;

} // end SmHCR::IsSilhouetteEdge

/*******************************************************************//**
PURPOSE: Computes if a surface silhouette edge is concave.

NOTES:
   If the curvature is concave on the side of the infinite region,
   then we can't see the silhouette edge.
   Picture a cylinder with a smooth groove along its length.
***********************************************************************/
SmBoolean SmHCR::IsConcaveSilhouette
  (SmEdge *pEdge)
{
  // locals
  SmPoint3d  sPnt;
  SmVector3d sTangentPlaneDir, sNorm;
  double     dCurvature = 0.0;
  SmEdgeuse *pEU        = pEdge->GetPrimaryEdgeuse();
  SmFace    *pFace      = pEU->GetFace();
  SmSurface *pSurface   = pFace->GetSurface();
  SmFaceuse *pUpwardFU  = pFace->GetUpwardFaceuse();
  SmFaceuse *pOtherFU   = pUpwardFU->GetMate();
  SmRegion  *pInfRegion = m_pSceneBrep->GetInfiniteRegion();
  SmRegion  *pUpwardReg = pUpwardFU->GetShell()->GetRegion();
  SmRegion  *pOtherReg  = pOtherFU ->GetShell()->GetRegion();
  
  // edgeuse midPoint => Surface UVPoint
  SER(pEU->NormalizedEvaluate(0.5,TRUE,sPnt));  // TRUE = UV Eval, FALSE = 3d Eval
  SmPoint2d sUV(sPnt.x,sPnt.y);

  // Get surface curvature from the ViewVector direction
  SER(pSurface->EvaluateNormalSection(sUV,
                                      TRUE,
                                      TRUE,
                                      m_vViewVector,
                                      sTangentPlaneDir, 
                                      dCurvature, 
                                      sNorm));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe=FALSE;
if ( bDebugMe ) 
  {
    SmSurface *pSrf = NULL;
    pSurface->Copy( *(pSurface->GetContext()), pSrf );
    SmObjDelete sClean(pSrf) ;

    if(pSrf) pSrf->Transform( m_vInvViewTransform );

    smgfx_SetLook( 2,2, 0,0,1 ); if(pSrf)pSrf->DrawAt( sUV, 1 ); sm_GraphicsLoop();
    sm_GraphicsLoop();
  }
#endif // SM_DEBUG_CODE

  if ( dCurvature < -SM_EFF_ZERO )
    {
      // Upward Faceuse is on the outside of the cylinder.
      // Test to see if it's the infinite region.
      if(   pUpwardReg != pInfRegion
         && pOtherReg  == pInfRegion )
        {
          return TRUE;
        }
    }
  else if ( dCurvature > SM_EFF_ZERO )
    {
      // Other Faceuse is on outside of cylinder.
      // Test to see if it's the infinite region.
      if (    pOtherReg  != pInfRegion
           && pUpwardReg == pInfRegion )
        {
          return TRUE;
        }
    }

  // arrive here when curvature is within SM_EFF_ZERO of zero
  return FALSE;

} // end SmHCR::IsConcaveSilhouette


/*******************************************************************//**
PURPOSE: Determine if a face is a front-facing face.

NOTES: Face is front facing when it has no backFacing edges
***********************************************************************/
SmBoolean SmHCR::IsFrontFacingFace(SmFace *pFace)
{
  SmEdge * sEData[128];
  SmTArray<SmEdge*> sEdges(128,sEData);

  // get face edges
  pFace->GetEdges(sEdges);
  ULONG ii, lNumEdges = sEdges.GetSize();

  // for every face edge
  for(ii=0; ii<lNumEdges; ii++ )
    {
      if ( IsBackFacingEdge( sEdges[ii]) ) { return FALSE; }
    }

  // have a front facing face
  return TRUE;

} // end SmHCR::IsFrontFacingFace

/*******************************************************************//**
PURPOSE: Determine if an edge is a back-facing edge.

NOTES:
***********************************************************************/
SmBoolean SmHCR::IsBackFacingEdge(SmEdge *pEdge)
{
  // low work - edge already classified
  void *pVoid = (void*) m_pSilEdges->At( pEdge );
  if ( pVoid != NULL )
    { return TRUE; }

  // Wire and Lamina edges do not count as back-facing.
  if ( pEdge->IsWire()   ) { return FALSE; }
  if ( pEdge->IsLamina() ) { return FALSE; }  // (Why not Lamina?)

  // arrive here for manifold and spine edges.

  // locals
  SmEdgeuse *pData[32];
  SmTArray<SmEdgeuse*> sEdgeuses(32,pData);

  // get edge->edgeuses
  pEdge->GetEdgeuses(sEdgeuses);

  // edgeuse locals
  SmEdgeuse *pPrimeEU     = sEdgeuses[0];
  SmEdgeuse *pPrimeEUMate = pPrimeEU->GetMate();
  SmBrep    *pBrep        = pEdge->GetBrep();

  // set pPRimeEU to any edgeuse in the infinite region when Mate is not in the infinite region
  if (    pPrimeEU    ->GetShell()->GetRegion() == pBrep->GetInfiniteRegion()
       && pPrimeEUMate->GetShell()->GetRegion() != pBrep->GetInfiniteRegion())
    {
      pPrimeEU = sEdgeuses[0];
    }
  else if (    pPrimeEUMate->GetShell()->GetRegion() == pBrep->GetInfiniteRegion()
            && pPrimeEU    ->GetShell()->GetRegion() != pBrep->GetInfiniteRegion())
    {
      pPrimeEU = pPrimeEUMate;
    }
  else // neither or both edgeuses are in the infinite region (GWC: is this right?)
    {
      return FALSE;
    }

  // evaluate the edgeuse binormals for the infinite region sector
  SmEdgeuse *pEURadial = pPrimeEU->GetRadial();
  double     dT        = pEdge->GetInterval().Evaluate(0.5);
  SmPoint3d sPnt;
  SmVector3d sBin1, sBin2, sNorm1, sNorm2, sTan1;
  SE( pPrimeEU ->EvaluateBinormal( dT, FALSE, sPnt, sBin1, &sTan1, &sNorm1 ));
  SE( pEURadial->EvaluateBinormal( dT, FALSE, sPnt, sBin2, NULL,   &sNorm2 ));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      SmVector3d sTPnt, sTBin1, sTBin2, sTNorm1, sTNorm2, sTTan1;
      m_vInvViewTransform.TransformPoint (sPnt,  sTPnt);
      m_vInvViewTransform.TransformVector(sBin1, sTBin1);
      m_vInvViewTransform.TransformVector(sBin2, sTBin2);
      m_vInvViewTransform.TransformVector(sNorm1,sTNorm1);
      m_vInvViewTransform.TransformVector(sNorm2,sTNorm2);
      m_vInvViewTransform.TransformVector(sTan1, sTTan1);

      smgfx_SetLook(1,3, 0,0,0); pEdge->DrawWTransform(m_vInvViewTransform); sm_GraphicsLoop();
      smgfx_SetLook(1,3, 1,0,0); sTPnt.Draw();         sm_GraphicsLoop();
      smgfx_SetLook(1,3, 1,0,0); sTBin1.Draw(&sPnt);   sm_GraphicsLoop();
      smgfx_SetLook(1,3, 0,1,1); sTNorm1.Draw(&sPnt); sm_GraphicsLoop();
      smgfx_SetLook(1,3, 0,0,1); sTBin2  .Draw(&sPnt); sm_GraphicsLoop();
      smgfx_SetLook(1,3, 1,1,0); sTNorm2.Draw(&sPnt); sm_GraphicsLoop();
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // If both Normals point away, it's back-facing.
  if (    sNorm1.Dot(m_vViewVector) < 0.0
       && sNorm2.Dot(m_vViewVector) < 0.0 )
    {
      return TRUE;
    }
  // If faces are on the same side of the edge relative to
  // the silhouette check to see if it is a case where the
  // edge is concave (i.e. edges on bottom of a hole).
  if ( sNorm1.Dot( m_vViewVector ) * sNorm2.Dot( m_vViewVector ) < SM_EFF_ZERO_SQRT )
    {
      // If edge bounds a solid then do sectoring to see what the story is
      SmVector3d sProjViewVec = sTan1 * m_vViewVector * sTan1;

      // Make sNorm1 point into the solid region
      sNorm1 = - sNorm1;

      // Return FALSE if it is an inside edge
      if ( sProjViewVec.LengthSquared() < SM_EFF_ZERO_SQ   ) { return FALSE; }
      if ( sProjViewVec.IsInsideSector(sBin1,sNorm1,sBin2) ) { return TRUE;  }
    }

  //
  return FALSE;

} // end SmHCR::IsBackFacingEdge


/*******************************************************************//**
PURPOSE: Determine the visibility of a point in the transformed space
  by doing a ray fire.

NOTES: 
  1. Builds the RayTracer object as a side effect
  2. SmBrep::ShrinkGeometry() should have been called on m_pSceneBrep
     prior to this call to avoid the possibility of small and unlikely
     tolerance errors between the RayTracer Object and the m_pSceneBrep 
     created when trimming Analytic surfaces.
***********************************************************************/
SmBoolean SmHCR::IsPointVisible(SmPoint3d & crPointToTest)
{
  SmStatus eStat = SM_SUCCESS;

  // Set up ray tracer if necessary.
  if ( m_pRayTracer == NULL )
    {
      SmTArray<SmSurface*> sSurfaces;

      // Get the raytrace grid size from the number of SceneBrep surfaces
      m_pSceneBrep->GetSurfaces(sSurfaces);
      ULONG lNumSurfs = sSurfaces.GetSize();
      ULONG lSize[3];
      if      (lNumSurfs <  100) { lSize[0] = lSize[1] = lSize[2] = 10; }
      else if (lNumSurfs < 1000) { lSize[0] = lSize[1] = lSize[2] = 20; }
      else                       { lSize[0] = lSize[1] = 40;
                                   lSize[2] = 20;
                                 }
      // raytracer locals
      ULONG kk;
      SmExtent3d sBBox       = m_vProjBBox;
      SmVector3d sDiagVector = sBBox.GetSize();
      sBBox.ExpandAbsolute(sDiagVector.Length() / 100.0);
      // double dChordHeight = sDiagVector.Length() / 200.0;

      // allocate the ray tracer grid
      SmGrid *pGrid = new (m_crContext) SmGrid(sBBox,lSize);

      // make sure that if m_pRayTracer is defined, other stuff is also.
      double dApproxTol      = m_dThisApproxTol3d;  // was 1.0e-8 [B387]
      double dConvergenceTol = 1.0e-10;
      m_pRayTracer    = new (m_crContext) SmRayTracer(m_crContext, pGrid, dApproxTol, dConvergenceTol );
      m_pRayTracer->SetSolverTolerance( dApproxTol ); // [B387]
      m_pGridElements = new (m_crContext) SmTArray<SmGridElement*>(m_crContext);
      m_pMinDistances = new (m_crContext) SmTArray<double>(m_crContext);
      m_pMaxDistances = new (m_crContext) SmTArray<double>(m_crContext);
      SmVector3d sZVector(0,0,-1); // ray cast direction

      // for every SceneBrep surface
      for(kk=0;kk<lNumSurfs;kk++)
        {
          SmSurface *pSurface = sSurfaces[kk];

          // If the surface is parallel to the view vector then we can skip it
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
          if (bDebugMe) 
            {
              // draw the target surface
              smgfx_Erase();
              smgfx_SetLook( 1,2, 0,1,1 ); pSurface->DrawUV(3,3); sm_GraphicsLoop();
              sm_GraphicsLoop();
            }
#endif // SM_DEBUG_CODE

          // skip surfaces parallel to viewing vector
          if ( pSurface->IsParallelToVector( sZVector ))
            { continue; }

          // For non-composite faces - If the Face is UVSquare trimmed (i.e.,
          // has a rectangular domain, possibly a subset of the surface's domain),
          // copy (and trim, if subset) and add copy to SceneBrep->m_pDeletedSurface array.
          SmFace *pFace = (SmFace*)pSurface->GetFace();
// Remove Composites
//        if  (! pFace->IsKindOf( SmCFace_TYPE ))
            {
              SmExtent2d sUVDomain;
              SmBoolean  bRectangularTrim ;
              SmBoolean  bNaturalTrim ;
              eStat = pFace->FindIfRectangularTrim( sUVDomain, bRectangularTrim, bNaturalTrim) ;
              if ( eStat != SM_SUCCESS ) 
                {
                  SE(SM_ERR);
                  bRectangularTrim = FALSE;
                }

              // when Face is trimmed by a UVRectangles
              if ( bRectangularTrim == TRUE)
                {
                  SmExtent2d sNaturalUVDomain = pSurface->GetNaturalUVDomain();

                  // when face's UVRectangle boundary == face->surface->naturalboundaries
                  if (bNaturalTrim)
                    {
                      // Domain match, no trimming needed: just copy.
                      SmSurface * pNewSurface = NULL;
                      eStat = pSurface->Copy(m_crContext, pNewSurface);
                      if ( eStat != SM_SUCCESS || pNewSurface == NULL )
                        {
                          if ( pNewSurface ) { delete pNewSurface; }
                          SE(SM_ERR);
                          return FALSE;
                        }

                      pSurface = pNewSurface;
                      m_pSceneBrep->AddDeletedSurface(pSurface);
                    }
                  else // RectangularTrim but not NaturalTrim: copy and trim.
                    {
                      SmBSplineSurface *pBSS = SM_CAST_PTR(SmBSplineSurface,pSurface);
                      if (pBSS)
                        {
                          SmBSplineSurface * pNewBSS = new (m_crContext) SmBSplineSurface(*pBSS);
                          SmObjDelete sCleanNewBSS(pNewBSS);
                          eStat = pNewBSS->TrimWithDomain(sUVDomain);
                          SmSurface * pNewSurface;

                          // Copy pNewBSS, when possible as an analytic surface
                          eStat = pNewBSS->CopyAndAddAnalytics(m_crContext,pNewSurface);
                          if ( eStat != SM_SUCCESS || pNewSurface == NULL )
                            {
                              if ( pNewSurface ) { delete pNewSurface; }
                              SE(SM_ERR);
                              return FALSE;
                            }
                          pSurface = pNewSurface;
                          m_pSceneBrep->AddDeletedSurface(pSurface);
                        } // end BSplineSurface check
                    } // else sUVDomain is less than sNaturalUVDomain check
                } // end if is UVSquared trimmed check.
            } // end process SmFace block

          // add the surface to the grid
          eStat = m_pRayTracer->AddSurfaceToGrid(pSurface);
          if ( eStat != SM_SUCCESS )
            { SE(SM_ERR); } // ... and proceed to next surface.

        } // end loop on all surfaces in scene Brep populating the RayTracer grid.

    } // end if ray tracer was null.

  // Note that someday we should fire rays in the opposite direction
  // so that we hit only front facing things. That way we would reduce
  // the number of elements that need work done.

  // create a ray segment [rayEnd RayStart], and rayVec = [0 0 -1]
  //  RayEnd = StartPoint + tol
  SmPoint3d sRayEnd = crPointToTest;
  sRayEnd.z         = sRayEnd.z + m_dRayOffsetTolerance;
  SmVector3d sRayVec(0,0,-1);

  //  RayStart = StartPoint + MaxDist
  double    dMaxDistance = 2.0 * (m_vProjBBox.GetMax().z - m_vProjBBox.GetMin().z);
  SmPoint3d sRayStart    = sRayEnd;
  sRayStart.z = sRayStart.z + dMaxDistance;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      smgfx_Erase();
      smgfx_SetLook(2,3, 0,0,1); sRayStart.Draw();         sm_GraphicsLoop();
      smgfx_SetLook(2,3, 0,1,0); sRayEnd.Draw();           sm_GraphicsLoop();
      smgfx_SetLook(2,3, 1,0,0); sRayVec.Draw(&sRayStart); sm_GraphicsLoop();

      SmBoolean bHiddenCurveSave;
      ULONG lHCDashedSave;
      smgfx_GetHiddenCurve(bHiddenCurveSave,lHCDashedSave);
      smgfx_SetHiddenCurve(FALSE,1);

      smgfx_SetLook(1,2, 0,0,0); m_pSceneBrep->Draw();     sm_GraphicsLoop();
      smgfx_SetHiddenCurve(bHiddenCurveSave,lHCDashedSave);
      sm_GraphicsLoop();
    }
#endif

  // fire a ray in the -z direction that ends just shy of the target point
  SmSolution sSolution;
  SmBoolean bHitsSomething;
  eStat = m_pRayTracer->FireRay(sRayStart, sRayVec,
                                dMaxDistance,
                                bHitsSomething,
                                sSolution,
                                *m_pGridElements,
                                *m_pMinDistances,
                                *m_pMaxDistances );
  // when something went wrong
  if ( eStat != SM_SUCCESS )
    {
      // signal an error and guess that the point is visible for what it's worth
      SE(SM_ERR);
      return TRUE;
    }

  // when the ray hit anything - the point is invisible
  return( bHitsSomething ? FALSE : TRUE ) ;

} // end SmHCR::IsPointVisible

/*******************************************************************//**
PURPOSE: Intersect a curve with silhouette vertices and edges which are in front of it.

NOTES:
  Intersects against all vertices and edges found in our Vertex and Edge trees.
  Intersects using projection along view vector.
***********************************************************************/
SmStatus SmHCR::IntersectCurveWithSilhouettes
  (SmCurveClassification & rCurveClass)   // i/o: divide classification at 
                                          //      projected curve/edge and curve/vertex intersections
{
  // locals
  const SmCurve & crCurve         = *rCurveClass.GetCurve();
  SmZoneTol3d     sCurveZoneTol3d = SmTol::GetSrcZoneTol3d(&rCurveClass) ;
  SmExtent1d      sIvl            = rCurveClass.GetInterval();

  // build bounding box for curve big enough to include all projected geometry
  SmExtent3d sEBBox;
  SER(crCurve.CalculateBoundingBox( sIvl, &sEBBox ));
  SmPoint3d sCorner = sEBBox.GetMax();
  sCorner.z = m_vProjBBox.GetMax().z;
  sEBBox.AddPoint3d(sCorner);
  sEBBox.ExpandAbsolute( SmTol::GetXSectTol3d(&rCurveClass, &rCurveClass) );

  // First do vertices.
  SmVertex * sData[256];
  SmTArray<SmVertex*> sSilVertices(256,sData);

  // only intersect with vertices in bounding box
  // Temp workaround to appease Linux gcc compiler
  // Original code with typecast has warning: dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]
  // SER(m_pSilVertexTree->GetObjectsInBox(sEBBox,SM_REINTERPRET_CAST(SmTArray<SmObject*> &,sSilVertices)));

  ULONG ii, jj;

  SmTArray<SmObject*> sObjects;
  SER(m_pSilVertexTree->GetObjectsInBox(sEBBox, sObjects));
  for (ii = 0;ii < sObjects.GetSize();ii++) { sSilVertices.Add((SmVertex*)sObjects[ii]); }
  
  
  ULONG lNumVerts = sSilVertices.GetSize();

  // for every vertex in the bounding box
  for(ii=0; ii<lNumVerts; ii++ )
    {
      SmVertex *pV = sSilVertices[ii];

      // GlobalPointSolve locals
      SmPoint3d    sVertPnt = pV->GetPoint();
      SmZoneTol3d  sVertZoneTol3d = SmTol::GetZoneTol3d(pV) ;
      SmXSectTol3d sXSectTol3d = SmTol::GetXSectTol3d(sVertZoneTol3d, sCurveZoneTol3d) ;
      double       dZero    = 0.0;
      SmSolution   sSData[16];
      SmSolutionArray sSolutions(16,sSData);

      // Find projection of vertex on the curve along view vector.
      SER( crCurve.GlobalPointSolve( sIvl,
                                    SM_SO_PROJECTED_MINIMIZE, 
                                    sVertPnt, 
                                    sXSectTol3d, 
                                    &dZero, 
                                    &m_vViewVector,
                                    SM_SR_ALL, 
                                    sSolutions));

      // for every solution
      ULONG lNumSols = sSolutions.GetSize();
      for(jj=0; jj<lNumSols; jj++ )
        {
          SmSolution & rSol        = sSolutions[jj];
          double       dCurveParam = rSol.m_vStart[0];

          // skip solutions outside current interval
          if ( ! rCurveClass.GetInterval().ContainsValue( dCurveParam ) )
            { continue; }

          // skip solutions behind this curve
          SmPoint3d sCrvPnt;
          SER(crCurve.EvaluatePoint( dCurveParam, sCrvPnt ));
          if ( sCrvPnt.z > sVertPnt.z + m_dRayOffsetTolerance )
            { continue; }

          // If another vertex projected to the same point,
          // chose the frontmost one.
          // (Frontmost is highest z.)
          SmBoolean bReplaceInstead = FALSE;  // To replace instead of inserting.

          ULONG              lIndex;
          double             dEndDist3d;
          SmIntervalPosition ePosition;

          // find interval for intersections
          rCurveClass.FindInterval(dCurveParam,    // in : target parameter value
                                   SM_BIG_DOUBLE,  // in : previous call Param to prevent double classifications, SM_BIG_DOUBLE to ignore
                                   sVertZoneTol3d, // in : SmTol::GetZoneTol3d(pAssocObj), always 3d Tol even when SrcCurve->Dim == 2
                                   lIndex,         // out: containing interval index
                                   dEndDist3d,     // out: for SM_IP_START and SM_IP_END pts, dist3d (or dist2d) along m_cpCurve to ivl end point
                                   ePosition ) ;   // out: oneof SM_IP_START, SM_IP_INSIDE, SM_IP_END, SM_IP_OUTSIDE dParameter classification

          // when vertex projected to existing curve interval boundary
          if(   ePosition == SM_IP_START 
             || ePosition == SM_IP_END )
            {
              if ( ePosition == SM_IP_END )
                { lIndex++; } // for getting point class.

              SmPointClassification *pPtClass = rCurveClass.GetPointClassByIndex( lIndex );

              // If it's already classified to a vertex:
              if ( pPtClass->GetPointClass() == SM_PC_VERTEX )
                {
                  SmPoint3d sExistingPos;
                  pPtClass->FindObjPoint3d( sExistingPos );
                  if ( sExistingPos.z >= sVertPnt.z )
                    {
                      continue;  // Just ignore this one.
                    }

                  // Signal to replace, not insert.
                  bReplaceInstead = TRUE;

                } // end already classified to vertex check
            } // end if FindInterval found an interval Start or End.

          // place projected xsect into a Point Classification
          SmPointClassification sPC(SmTol::GetSrcZoneTol3d(&rCurveClass), rCurveClass.GetContext()) ;
          sPC.SetClassObject ( SM_PC_VERTEX, pV );
          sPC.SetGap3d       ( rSol.m_vStart.m_dSolutionValue );
          sPC.SetSrcZoneTol3d(SmTol::GetSrcZoneTol3d(&rCurveClass)) ;
          sPC.SetPreSnapParam( rSol.m_vStart[0] );
          SmBoolean bInsertionMade;
          SmStatus eStat;

          // when replacing an existing vertex
          if ( bReplaceInstead )
            {
              eStat = rCurveClass.UpdatePointClass( lIndex, sPC );
            }
          else // insert the new PointClass into the Curve Classification
            {
              eStat = rCurveClass.InsertPointClass(sPC, 
                                                   dCurveParam, 
                                                   SM_BIG_DOUBLE, 
                                                   bInsertionMade );
            }

          // when the classification modification fails
          if ( eStat != SM_SUCCESS )
            {
              // just inform the public
              SM_DBG_WARN(_T("SmHCR::IntersectCurveWithSilhouettes(): InsertPointClass() failed.") );
            }
        } // end iter all vertex/curve projected xsect solutions from GlobalPointSolve
    } // end iter all vertices in curve bounding box

  // Now do Edges.
  SmEdge * sData2[256];
  SmTArray<SmEdge*> sSilEdges(256,sData2);
  
  // Get all edges within the curve projection bounding box
  // Temp workaround to appease Linux gcc compiler
  // Original code with typecast has warning: dereferencing type-punned pointer will break strict-aliasing rules [-Wstrict-aliasing]
  // SER(m_pSilEdgeTree->GetObjectsInBox(sEBBox,SM_REINTERPRET_CAST(SmTArray<SmObject*> &,sSilEdges)));

  SmTArray<SmObject*> sObjectsInBox;
  SER(m_pSilEdgeTree->GetObjectsInBox(sEBBox, sObjectsInBox));
  for (ii = 0;ii < sObjects.GetSize();ii++) { sSilEdges.Add((SmEdge*)sObjectsInBox[ii]); }

  ULONG lNumEdges = sSilEdges.GetSize();

  // for every edge in the curve bounding box
  for(ii=0; ii<lNumEdges; ii++ )
    {
      SmEdge     * pE             = sSilEdges[ii];
      SmZoneTol3d  sEdgeZoneTol3d = SmTol::GetZoneTol3d(pE) ;
      SmXSectTol3d sXSectTol3d    = SmTol::GetXSectTol3d(sCurveZoneTol3d, sEdgeZoneTol3d) ;

      // Don't intersect with self
      if ( pE == NULL || pE->GetCurve() == &crCurve ) 
        { continue; }

      // locals
      SmSolution sSData[16];
      SmSolutionArray sSolutions(16,sSData);

      // find curve/curve projected intersections
      SER( crCurve.GlobalCurveSolve( sIvl, 
                                     *pE->GetCurve(), 
                                     pE->GetInterval(),
                                     SM_SO_PROJECTED_INTERSECT, 
                                     sXSectTol3d, 
                                     NULL, 
                                     &m_vViewVector, 
                                     SM_SR_ALL, 
                                     sSolutions ));

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
      if (bDebugMe) 
        {
          // draw curves and solutions being intersected
          SmBSplineCurve *pTmp = new(m_crContext) SmBSplineCurve( (SmBSplineCurve&)crCurve);
          SmObjDelete sCleanTmp(pTmp);
          pTmp->Transform(m_vInvViewTransform);
          
          sSolutions.Dump();
          smgfx_Erase();
          smgfx_SetLook(3,4, 0,0,1); pTmp->DrawWDeriv(pTmp->GetNaturalInterval(),0); sm_GraphicsLoop();
          smgfx_SetLook(5,6, 0,1,0); pE->DrawWTransform(m_vInvViewTransform); sm_GraphicsLoop();
          smgfx_SetLook(6,7, 1,0,0); sSolutions.Draw() ; sm_GraphicsLoop() ;
          sm_GraphicsLoop();
        }
#endif // SM_DEBUG_CODE

      // for every solution
      ULONG lNumSols = sSolutions.GetSize();
      for(jj=0; jj<lNumSols; jj++ )
        {
          SmSolution & rSol = sSolutions[jj];

          // Evaluate Edge and Curve solution points
          SmPoint3d sPnt1, sPnt2;
          SER( crCurve        .EvaluatePoint( rSol.m_vStart[0], sPnt1  ));
          SER( pE->GetCurve()->EvaluatePoint( rSol.m_vStart[1], sPnt2 ));

          // Don't use this intersection if it is behind the curve
          if ( sPnt1.z < sPnt2.z + m_dRayOffsetTolerance )
            {
              // build a point classification for the solution
              SmPointClassification sPC(SmTol::GetSrcZoneTol3d(&rCurveClass), rCurveClass.GetContext()) ;
              sPC.SetClassObject ( SM_PC_EDGE, pE );
              sPC.SetGap3d       ( rSol.m_vStart.m_dSolutionValue );
              sPC.SetSrcZoneTol3d(SmTol::GetSrcZoneTol3d(&rCurveClass)) ;
              sPC.SetPreSnapParam( rSol.m_vStart[0] );
              sPC.SetTParam      ( rSol.m_vStart[1] );
              SmBoolean bInsertionMade;

              // Note someday we will have to do a Z-sort here to put the frontmost
              // intersection into the rCurveClass.

              // when the solution is in the curve interval
              if ( rCurveClass.GetInterval().ContainsValue( rSol.m_vStart[0] ))
                {
                  // divide the curve classification at the point solution
                  SER( rCurveClass.InsertPointClass( sPC, 
                                                     rSol.m_vStart[0], 
                                                     SM_BIG_DOUBLE,
                                                     bInsertionMade ));
                }
            } // end behind the curve solution check

          // If a range solution was returned, process the end as well.
          if ( rSol.m_eSolutionType == SM_ST_RANGE_OF_VALUES )
            {
              // evaluate edge and curve solution points
              SER( crCurve.EvaluatePoint( rSol.m_vEnd[0], sPnt1 ));
              SER( pE->GetCurve()->EvaluatePoint( rSol.m_vEnd[1], sPnt2 ));

              // Don't use this intersection if it is behind the curve
              if ( sPnt1.z < sPnt2.z + m_dRayOffsetTolerance )
                {
                  // build a point classification for the point solution
                  SmPointClassification sPC(SmTol::GetSrcZoneTol3d(&rCurveClass), rCurveClass.GetContext()) ;
                  sPC.SetClassObject ( SM_PC_EDGE, pE );
                  sPC.SetGap3d       ( rSol.m_vEnd.m_dSolutionValue );
                  sPC.SetSrcZoneTol3d(SmTol::GetSrcZoneTol3d(&rCurveClass)) ;
                  sPC.SetPreSnapParam( rSol.m_vEnd[0] );
                  sPC.SetTParam      ( rSol.m_vEnd[1] );
                  SmBoolean bInsertionMade;

                  // Note someday we will have to do a Z-sort here to put the frontmost
                  // intersection into the rCurveClass.

                  // when the point is in the curve interval
                  if ( rCurveClass.GetInterval().ContainsValue( rSol.m_vEnd[0] ))
                    {
                      // divide the curve classification at the point
                      SER( rCurveClass.InsertPointClass(sPC, 
                                                        rSol.m_vEnd[0], 
                                                        SM_BIG_DOUBLE,
                                                        bInsertionMade ));
                    
                    } // end is solution in the curve interval check
                } // end is solution behind the curve check
            } // end Range solution check
        } // end iter all curve/curve projected xsect solutions from GlobalCurveSolve
    } // end iter every edge in the curve bounding box

  // all done
  return SM_SUCCESS;

} // end SmHCR::IntersectCurveWithSilhouettes

/*******************************************************************//**
PURPOSE: Create a graphics display list of the visible curves.

NOTES:
***********************************************************************/
SmStatus SmHCR::DrawVisibleCurves
  (SmBoolean                   bDrawInvisibleAsDashed,  // in : 
   const SmDisplayParameters & crDisp,                  // in : 
   SmDisplayList             * pDisplayList )           // out: ptr to s_View.m_pActiveLists[last] when s_View.m_bGraphicsInitialized
                                                        //      else returns NULL
{
#ifdef SM_GFX_OUTPUT_CODE
  ULONG ii, lNumCurves;
  SmDisplayList rVisibleCurves = smgfx_Open( smgfx_GetRuleColor() );
    {
      smgfx_OutputLineWidth( 2.0 );
      smgfx_OutputPointSize( SM_VERTEX_POINTSIZE );
      smgfx_OutputDashedLines( FALSE );
      lNumCurves = m_pVisibleCurves->GetSize();
      for(ii=0; ii<lNumCurves; ii++ )
        {
          SmCurve *pCurve = (*m_pVisibleCurves)[ii];
          SE( pCurve->OutputGraphics( pCurve->GetNaturalInterval(), crDisp, NULL ));
        }
    }
  smgfx_Close();


  SmDisplayList rInvisibleCurves = smgfx_Open( smgfx_GetRuleColor() );
    {
      smgfx_OutputLineWidth( 1.0 );
      smgfx_OutputPointSize( SM_VERTEX_POINTSIZE );
      smgfx_OutputDashedLines( TRUE );
      lNumCurves = m_pInvisibleCurves->GetSize();
      for(ii=0; ii<lNumCurves; ii++ )
        {
          SmCurve *pCurve = (*m_pInvisibleCurves)[ii];
          if ( bDrawInvisibleAsDashed )
            {
              SE( pCurve->OutputGraphics( pCurve->GetNaturalInterval(), crDisp, NULL ));
            }
        }
    }
  smgfx_OutputDashedLines( FALSE );

  // Now make a display list out of visible and invisible list
  smgfx_Open( smgfx_GetRuleColor() );
  pDisplayList = smgfx_Close();  // close last Open() call and get ptr to s_View.m_pActiveLists[last]

#else
  SM_REF3(bDrawInvisibleAsDashed, crDisp, pDisplayList);
#endif // SM_GFX_OUTPUT_CODE

  return SM_SUCCESS;

} // end SmHCR::DrawVisibleCurves


