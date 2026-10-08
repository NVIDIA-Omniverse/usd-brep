// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmHCR.h
* PURPOSE: Header file for SmHCR object.
**********************************************************************/

#ifndef __SMHCR_H__
#define __SMHCR_H__

#ifndef __SMSURF_TYPES_H__
#include <SmSurfTypes.h>
#endif

#ifndef __SMCORE_TYPES_H__
#include <SmCoreTypes.h>
#endif

#ifndef __SMCURV_TYPES_H__
#include <SmCurveTypes.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMTOPO_TYPES_H__
#include <SmTopoTypes.h>
#endif

#ifndef __SMCURV_TYPES_H__
#include <SmCurveTypes.h>
#endif

#ifndef __SMAXIS2PLACEMENT_H__
#include <SmAxis2Placement.h>
#endif

#ifndef __SMPOLY_H__
#include <SmPoly.h>
#endif

#ifndef __SMRAYTRACER_H__
#include <SmRayTracer.h>
#endif

class SmDisplayParameters ;

/*******************************************************************//**
PURPOSE: The HCR (Hidden Curve Removal) object provides the
    ability to determine visibility of a topological object or
    scene of disjoint topological objects.

NOTES:
***********************************************************************/
class SM_EXPORT SmHCR
{
private:
    const SmContext          & m_crContext;           // needed for new object construction
    SmVector3d                 m_vViewVector;         // viewing direction in the SceneBrep orientation:
                                                      //   set to [0 0 1] in constructor
                                                      //   ray cast directions are in -z from geometry locations
                                                      //   (if the ray hits nothing that geometry location can be seen in the viewing direction)

    double                     m_dRayOffsetTolerance; // Tolerance used at ray start positions
                                                      // currently set to 1/1000 of m_vProjBBox diagonal

    const SmBrep             * m_pOriginalBrep;       // original Brep
    SmBrep                   * m_pSceneBrep;          //copied and rotated version of original Brep

    SmAxis2Placement           m_vViewTransform;      // modelView transformation:
    SmAxis2Placement           m_vInvViewTransform;   // modelView Inverse transform:
                                                      //   sets SceneBrep orientation as
                                                      //    SceneBrep = OrigBrep_Shapes * TransMatrix
                                                      //    with SceneViewingVector set to [0 0 1],
                                                      //    Let OVV = OrigViewingVector, OVV can be found as
                                                      //    [0 0 1]          = [OVVx OVVx OVVz] * TransMatrix, and
                                                      //    [OVVx OVVx OVVz] = [ 0    0    1  ] * InvTransMatrix
                                                      // To Build ViewTransform from a ViewingVector:
                                                      //      SmAxis2Placement sViewTransform ;
                                                      //      sViewTransform.SetSTEPCanonical( origin, ViewingVector, PerpToViewingVector ) ;
                                                      //      sViewTransform.Invert(sViewTransform) ;

    SmExtent3d                 m_vProjBBox;           // a bounding box containing m_pSceneBrep, set to 3 times the size of the m_pSceneBrep bounding box

    SmTree                   * m_pSilVertexTree;      // hierarchical tree decomposition of all silhouette vertices in SceneBrep orientation
    SmTree                   * m_pSilEdgeTree;        // hierarchical tree decomposition of all silhouette edges    in SceneBrep orientation
    SmTree                   * m_pFrontFaceTree;      // not being used - hierarchical tree decomposition of all silhouette faces

    SmMapPtrToPtr<SmVertex, SmVertex> * m_pSilVertices;     // list of all SceneBrep silhouette vertices  in SceneBrep orientation
    SmMapPtrToPtr<SmEdge, SmEdge>     * m_pSilEdges;        // list of all SceneBrep silhouette edges     in SceneBrep orientation
    SmMapPtrToPtr<SmEdge, SmEdge>     * m_pBackFacingEdges; // list of all SceneBrep BackFacing edges     in SceneBrep orientation
                                                      //    vertices attached to BackFacing edges are not put on lists

    SmTArray<SmCurve*>       * m_pVisibleCurves;      //   visible curves (copies of SceneBrep curve intervals in OrigBrep orientation)
    SmTArray<SmCurve*>       * m_pInvisibleCurves;    // invisible curves (copies of SceneBrep curve intervals in OrigBrep orientation)

    SmTArray<SmPoint3d>      * m_pVisiblePoints;           // not being used - list of   visible points in SceneBrep
    SmTArray<SmPoint3d>      * m_pInvisiblePoints;         // not being used - list of invisible points in SceneBrep

    double                     m_dThisApproxTol3d;         // dist tol  that limits segment sizes when walking silhouette curves
    double                     m_dThisAngTolRad;           // angle tol that limits segment sizes when walking silhouette curves
    SmBoolean                  m_bHCRForSilhouettes;       // TRUE=skip ComputeEdgeVisibility for Concave Silhouettes,
                                                           // apparently disabled
                                                           // init to:[FALSE]
    SmBoolean                  m_bDisplaySeams;            // TRUE=marks seams as visible edges, FALSE=lets seams be invisible
    SmBoolean                  m_bDisplaySmoothEdges;      // TRUE=marks manifold edges that are G1 between their faces as visible, FALSE=lets them be invisible

    // The following are used by ray tracer.
    SmRayTracer              * m_pRayTracer;         // RayTracer Object for rapid Ray Firing, init to NULL
    SmTArray<SmGridElement*> * m_pGridElements;      // Grid in the RayTradcer, init to NULL
    SmTArray<double>         * m_pMinDistances;      // init to NULL
    SmTArray<double>         * m_pMaxDistances;      // init to NULL

public:
    // constructor with a viewing direction
    SmHCR(const SmContext        & crContext,                     // in : new object construction
          const SmBrep           * pBrep,                         // in : Original Brep - never modified
          const SmVector3d       & crViewVector,                  // in : eye direction runs parallel to this vector
          double                   dThisApproxTol3d,              // in : dist tol  that limits segment sizes when walking silhouette curves
          double                   dThisAngTolRad,                // in : angle tol that limits segment sizes when walking silhouette curves
          SmBoolean                bDisplaySeams = TRUE,          // in : TRUE=marks seams as visible edges, FALSE=lets seams be invisible
          SmBoolean                bDisplaySmoothEdges = TRUE ) ; // in : TRUE=marks manifold edges that are G1 between their faces as visible, FALSE=lets them be invisible

    // constructor with a SmAxis2Placment transformation
    SmHCR(const SmContext        & crContext,                     // in : new object construction
          const SmBrep           * pBrep,                         // in : Original Brep - never modified
          const SmAxis2Placement & crViewTransformation,          // in : Set eye orientation: SceneBrep = OrigBrep_Shapes * TransMatrix
                                                                  //        SceneViewVector = [0 0 1] * InvViewTransform
          double                   dThisApproxTol3d,              // in : dist tol  that limits segment sizes when walking silhouette curves
          double                   dThisAngTolRad,                // in : angle tol that limits segment sizes when walking silhouette curves
          SmBoolean                bDisplaySeams = TRUE,          // in : TRUE=marks seams as visible edges, FALSE=lets seams be invisible
          SmBoolean                bDisplaySmoothEdges = TRUE );  // in : TRUE=marks manifold edges that are G1 between their faces as visible, FALSE=lets them be invisible

    // destructor
    ~SmHCR();

    // Find and store copies of visible and invisible curve intervals
    //   for this viewing angle in the m_pVisibleCurves and m_pInvisibleCurves lists
    SmStatus ComputeGlobalVisibility();   // increments unlocked mark value

    // export the visible curve interval list
    void GetVisibleCurves(SmTArray<SmCurve*> & rCurves)            { rCurves.ReSet(); rCurves.Append(*m_pVisibleCurves); }
    void GetInvisibleCurves(SmTArray<SmCurve*>& rCurves)           { rCurves.ReSet(); rCurves.Append(*m_pInvisibleCurves); }

    SmStatus GetSilhouetteEdges( SmTArray< SmEdge* > &rSilEdges );

    // control state
    void      SetHCRForSilhouettes (SmBoolean bHCRForSilhouettes)  { m_bHCRForSilhouettes = bHCRForSilhouettes; }
    void      SetDisplaySeams      (SmBoolean bDisplaySeams)       { m_bDisplaySeams = bDisplaySeams; }
    void      SetDisplaySmoothEdges(SmBoolean bDisplaySmoothEdges) { m_bDisplaySmoothEdges = bDisplaySmoothEdges; }

    SmBoolean GetDisplaySeams      () const                    { return m_bDisplaySeams; }
    SmBoolean GetDisplaySmoothEdges() const                    { return m_bDisplaySmoothEdges; }
    double    GetThisApproxTol3d   () const                    { SM_ASSERT_TOL(m_dThisApproxTol3d) ; return m_dThisApproxTol3d; }
    double    GetThisAngTolRad     () const                    { SM_ASSERT_TOL(m_dThisAngTolRad) ; return m_dThisAngTolRad;   }
    SmBrep  * GetSceneBrep         () const                    { return m_pSceneBrep; }

    // support functions

    // Init code common to all constructors
    void ConstructorInit
    (const SmAxis2Placement & crViewTransform) ; // in : Set eye orientation: SceneBrep = OrigBrep_Shapes * TransMatrix
                                                 //        SceneViewVector = [0 0 1] * InvViewTransform

    // classify every curve interval (made by proj xsects with SceneBrep) as visible/invisible
    SmStatus ComputeCurveVisibility(SmCurveClassification & crCurveClass,
                                    ULONG lStartHint,
                                    ULONG lEndHint);

    // Classify edge->Curve into sequence of visible/invisible intervals and
    //   place a trimmed copy each Interval in the OrigBrep orientaion in the
    //   m_pVisibleCurves and m_pInvisibleCurves lists
    SmStatus ComputeEdgeVisibility(SmEdge   * pEdge,
                                   SmVertex * pKnownVertex,
                                   SmBoolean  bVertexIsVisible,
                                   SmMarkType eMarkType);

    // computes and inserts surface->Silhouetted curves into the SceneBrep as new edges.
    //  side effect: classifies every edge as Silhouette or Backfacing
    SmStatus ComputeSilhouettes();

    // test an edge for BackFacing
    SmBoolean IsBackFacingEdge(SmEdge *pEdge);

    // classify concavity of edge with respect to a viewing direction
    SmBoolean IsConcaveSilhouette(SmEdge *pEdge);

    // Face is front facing when it has no backFacing edges
    SmBoolean IsFrontFacingFace(SmFace *pFace);

    // test an edge for silhouette (not backfacing)
    SmBoolean IsSilhouetteEdge(SmEdge *pEdge);

    // use ray casting to see if TestPoint is visible for viewing direction
    SmBoolean IsPointVisible(SmPoint3d & crPointToTest);

    // Find curve intervals defined by all SceneBrep vertex and edge projected intersections
    SmStatus IntersectCurveWithSilhouettes(SmCurveClassification & crCurveClass);

    // clear the visible curve interval list
    void ClearVisibleCurves() { m_pVisibleCurves->ReSet(); }

    // maintenance
    SmStatus DrawVisibleCurves(SmBoolean                   bDrawInvisibleAsDashed,
                               const SmDisplayParameters & crDisp,
                               SmDisplayList             * pDisplayList);

} ; // end class SmHCR


#endif // !__SMHCR_H__
