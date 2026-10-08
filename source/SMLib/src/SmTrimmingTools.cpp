// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**********************************************************************//**
* FILE NAME --- SmTrimmingTools.cpp
* PURPOSE: Source file for SmTrimmingTools object.
**********************************************************************/

#include "StdAfx.h"

#include <SmTrimmingTools.h>

#ifndef __SMBREP_H__
#include <SmBrep.h>
#endif

#ifndef __SMFACE_H__
#include <SmFace.h>
#endif

#ifndef __SMLOOP_H__
#include <SmLoop.h>
#endif

#ifndef __SMREGION_H__
#include <SmRegion.h>
#endif

#ifndef __SMEDGE_H__
#include <SmEdge.h>
#endif

#ifndef __SMEDGEUSE_H__
#include <SmEdgeuse.h>
#endif

#ifndef __SMVERTEX_H__
#include <SmVertex.h>
#endif

#include <SmTools.h>
#include <SmBSplineSurface.h>
#include <SmSurface.h>
#include <SmCompositeCurve.h>
#include <SmPointSet.h>
#include <SmAssertArray.h>

#include <nurbs.h>

#ifdef SM_DEBUG_CODE
  #include <SmCrvOnSurf.h>
#endif // SM_DEBUG_CODE

/*******************************************************************//**
PURPOSE: Trim a tgt surface to an ordered set of 3dCurves and use the
  result to create one or more Faces within a specified Region (which exists
  within an existing Brep).

Example:
     It specifically handles the following cases:

    1) The loops are not ordered correctly, that is, the outer loop
       is not first in the list.

    2) The loops are not oriented correctly, that is, the outer loop
       goes clockwise around the surface normal or an inner loop goes
       counter-clockwise.  (Works on G1-continuous surfaces only.)

    (Note that one of the two conditions must be TRUE -- the loops must
    be either oriented correctly or ordered correctly.)

    3) The edges cross the seam of the surface on a closed surface.
    (Works on correctly-oriented G1-continuous surfaces only.)

NOTES:
   It also runs a number of checks to verify the surface has been
   correctly created.

    This is especially good for the common case of analytic brep faces
    from other systems.  Because the analytic surface has no seam,
    the edges will frequently cross the seam on the NURBS equivalent.
    In that case, the loops are normally correctly oriented.

    If the surface is not G1-continuous or loops cross seams, more than
    one face may be generated.
***********************************************************************/
SmStatus SmTrimmingTools::TrimSurfaceWithModelSpaceCurves
  (SmContext const                   & crContext,                  // in : new object construction
   SmRegion                          * pRegion,                    // in : Target region in existing Brep to receive new trimmed surface.
   double                              d3DTolerance,               // in : Tolerance assigned to Brep and passed to SplitAtSeams().
   const SmTArray<ULONG>             & crCurveLoops,               // in : crCurveLoops.GetSize() = number of loops to make
                                                                   //      crCurveLoops[i]        = number of curves in ith loop 
   const SmTArray<SmCurve*>          & cp3DCurves,                 // in : Curve array ordered by loop and neighbor.  
                                                                   //      1st loop is outer loop, subsequent optLoops are inner loops. 
                                                                   //      outer loop orientation = counter clockwise
                                                                   //      inner loop orientation = clockwise
                                                                   //      These curves are consumed by this method.
   const SmTArray<SmOrientType>      & crCurveOrientations,        // in : SM_OT_SAME     = curve's orientation in its loop is the same as it's parametric orientation
                                                                   //      SM_OT_OPPOSITE = curve's orientation in its loop is the oppositie of it's parametric orientation
   const SmTArray<SmPoint3d>         & crLoopPoints,               // in : Points within the outer loop to become vertex loops.
   SmSurface                         * pSurface,                   // in : The surface to be trimmed. It's consumed by this operation.
   const SmExtent2d                  & crUVDomain,                 // in : Domain of the surface used by the face.
   SmOrientType                        eSurfaceOrientation,        // in : SM_OT_SAME     = face oriented with the surface normal
                                                                   //      SM_OT_OPPOSITE = face oriented against the surface normal (negates the CurveOrientation requirements)
   SmBoolean                           bFirstLoopIsOuterLoop,      // in : Not used in this function.
   SmBoolean                           bLoopsAreOrientedCorrectly, // in : TRUE = 
                                                                   //      FALSE= 
   SmBoolean                           bLoopMayCrossASeam,         // in : TRUE =
                                                                   //      FALSE=
    SmTArray<SmFace*>                * rpNewFaces                  // out: Optional pointer to array of new faces
  )
{
  // make pSurface and cp3DCurves inputs temporary 
  SmObjDelete            sSurfaceClean (pSurface);
  SmObjsDelete<SmCurve*> sCurvesClean  (&cp3DCurves);

  NER(pRegion);
  SmBrep *pBrep = pRegion->GetBrep();
  NER(pBrep);

  SM_ASSERT (bLoopsAreOrientedCorrectly || bFirstLoopIsOuterLoop);

  // TODO: Handle cases which require proper orientation but lack it,
  //       like the next one.
  if (bLoopMayCrossASeam) 
    {
      SM_ASSERT (bLoopsAreOrientedCorrectly);
    }

  // set Brep tolerance
  pBrep->SetTolerance (d3DTolerance);

  // remember if Surface is G1 continuous
  SmBoolean bG1Surface;
  SER (SurfaceContinuityCheck (pSurface, bG1Surface));

  // for nonG1 surfaces, Loops must be oriented correctly
  if(   !bG1Surface 
     && !bLoopsAreOrientedCorrectly)
    { SER (SM_ERR); }

  // locals
  ULONG ii;
  SmRegion * sNewRegion = NULL;
  SmShell  * sNewShell = NULL;
  SmTArray<SmFace *> sNewFaces;

  // no more unexpected SERs - remove temporary status of the input data
  sSurfaceClean.Clear ();
  sCurvesClean.Clear ();

  // when Surface is G1Continuous - just call MakeFaceWithCurves()
  if (bG1Surface)
    {
      SmFace *sNewFace = NULL;
      SER (pBrep->MakeFaceWithCurves
            (pRegion,                  // in : region to contain new topology objects
             crCurveLoops,             // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop
             &cp3DCurves,              // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts
             0,                        // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts
             crCurveOrientations,      // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE
             crLoopPoints,             // in : Point positions to build SmVertex VertexLoops
             pSurface,                 // in : new face->Surface
             crUVDomain,               // in : domain of Surface used by face
             eSurfaceOrientation,      // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE
             sNewRegion,               // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids.
             sNewShell,                // out: New shell if any.  Trimmed surfaces always create a new shell.
             sNewFace));               // out: the new face
                                                                         
      sNewFaces.Add (sNewFace);
    }
  else // when Surface is not G1Continuous
  
    {
      // make a MakeFaceWithCurves call followed by a SubdivideFaceAtDiscontinuities() call.
      SER (pBrep->MakeFacesWithCurves
            (pRegion,                  // in : region to contain new topology objects
             crCurveLoops,             // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop
             &cp3DCurves,              // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts
             0,                        // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts
             crCurveOrientations,      // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE
             crLoopPoints,             // in : Point positions to build SmVertex VertexLoops
             pSurface,                 // in : new face->Surface
             crUVDomain,               // in : domain of Surface used by face
             eSurfaceOrientation,      // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE
             sNewRegion,               // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids.
             sNewShell,                // out: New shell if any.  Trimmed surfaces always create a new shell.
             sNewFaces));              // out: the new face

      // I don't know why this clear is needed, but it seems to make
      // a big difference.  (Possibly this is a bug still lurking in
      // GetEdgeUVExtremes.)
      for (ii = 0; ii < sNewFaces.GetSize (); ii++)
          ClearUVTrimCurves (sNewFaces [ii]);
    } // end Surface is not G1Continuous branch

  // when warned - split all new faces at seam boundaries 
  if (bLoopMayCrossASeam)
    {
      SmTArray<SmFace *> sNewSplitFaces;

      // for every new face - Split at surface seams
      for (ii = 0; ii < sNewFaces.GetSize (); ii++)
        {
          SmTArray<SmFace *> sNewThisSplitFaces;
          SER (SplitAtSeams (crContext,         // note: increments unlocked mark value
                            sNewFaces [ii], 
                            sNewThisSplitFaces));
          
          // accumulate all new Faces
          if (sNewThisSplitFaces.GetSize () > 0)
              sNewSplitFaces.Append (sNewThisSplitFaces);
        } // end iter all new faces

      // add new split faces to sNewFaces list
      if (sNewSplitFaces.GetSize () > 0)
          sNewFaces.Append (sNewSplitFaces);

    } // end bLoopMayCrossASeam check

  // for every new face - fix up known common problems 
  for (ii = 0; ii < sNewFaces.GetSize (); ii++)
    {
      if (!AreUVCurvesPresent (sNewFaces [ii]))
          SER (CreateUVTrimCurves (sNewFaces [ii]));

      // when told loops are oriented correctly
      if (bLoopsAreOrientedCorrectly)
        {
          SmBoolean bTinyGapRepair;
          SmBoolean bReorderedLoops;

          //
          SER (FixTrimLoopTinyParameterSpaceGaps (sNewFaces [ii], SM_ZONE_TOL_3D/10.0, bTinyGapRepair));

          //
          SER (FixTrimLoopOrder (sNewFaces [ii], bReorderedLoops));
        }
      else // loop may not be oriented correctly
        {
          SmBoolean bOuterLoopRepair;
          SmBoolean bTinyGapRepair;
          ULONG bLoopOrientRepair;
          
          //
          SER(FixBackwardsOuterLoopOnSeam (sNewFaces [ii], bOuterLoopRepair));
          
          // 
          SER(FixTrimLoopTinyParameterSpaceGaps (sNewFaces [ii], SM_ZONE_TOL_3D/10.0, bTinyGapRepair));
          
          // 
          SER(FixTrimLoopOrientation (sNewFaces [ii], bLoopOrientRepair));
        }
    } // end every trim face - repairing known possible problems

  // all done
  SER (CheckFaces (sNewFaces, d3DTolerance));

  // Move pointers to optional outgoing array
  if(rpNewFaces != NULL)
  {
    rpNewFaces->SetSize( sNewFaces.GetSize() );
    for(ULONG jj = 0; jj < sNewFaces.GetSize(); jj++)
    {
      (*rpNewFaces)[jj] = sNewFaces[jj];
    }
  }

  return SM_SUCCESS;

} // end SmTrimmingTools::TrimSurfaceWithModelSpaceCurves

/*******************************************************************//**
PURPOSE: Powerful method to create an SmBrep containing a single trimmed surface.

NOTES:
    It specifically handles the following cases:
    1) The loops are not ordered correctly, that is, the outer loop
       is not first in the list.

    2) The loops are not oriented correctly, that is, the outer loop
       goes clockwise around the surface normal or an inner loop goes
       counter-clockwise.  (Works on G1-continuous surfaces only.)

    (Note that one of the two conditions must be TRUE -- the loops must
    be either oriented correctly or ordered correctly.)

    3) The edges cross the seam of the surface on a closed surface.
    (Works on G1-continuous surfaces only.)

   It also runs a number of checks to verify the surface has been
   correctly created.

    This is especially good for the common case of analytic brep faces
    from other systems.  Because the analytic surface has no seam,
    the edges will frequently cross the seam on the NURBS equivalent.
    In that case, the loops are normally correctly oriented.

    If the surface is not G1-continuous or loops cross seams, more than
    one face may be generated.
***********************************************************************/
SmStatus SmTrimmingTools::TrimSurfaceWithModelSpaceCurves
  (SmContext const &crContext,
   double d3DTolerance,
   const SmTArray<ULONG> & crCurveLoops,
   const SmTArray<SmCurve*> & cp3DCurves,                   // Input curves consumed by this method.
   const SmTArray<SmOrientType> & crCurveOrientations,
   const SmTArray<SmPoint3d> & crLoopPoints,
   SmSurface * pSurface,                                    // Surface of the trimmed surface. Note that it is consumed by this operation.
   const SmExtent2d & crUVDomain,
   SmOrientType eSurfaceOrientation,
   SmBoolean bFirstLoopIsOuterLoop,
   SmBoolean bLoopsAreOrientedCorrectly,
   SmBoolean bLoopMayCrossASeam,
   SmBrep *& rpBrep)
{
    rpBrep = new (crContext) SmBrep();
    if (rpBrep == 0)
        SER (SM_ERR_OUT_OF_MEMORY);
    // gwc: removed next line - sets rpBrep->Tol = crContext::ZoneTol3d
    //    rpBrep->SetTolerance (d3DTolerance);

    SER (TrimSurfaceWithModelSpaceCurves (crContext,
                                          rpBrep->GetInfiniteRegion (),
                                          d3DTolerance, crCurveLoops,
                                          cp3DCurves, crCurveOrientations,
                                          crLoopPoints, pSurface, crUVDomain,
                                          eSurfaceOrientation,
                                          bFirstLoopIsOuterLoop,
                                          bLoopsAreOrientedCorrectly,
                                          bLoopMayCrossASeam));
    return SM_SUCCESS;

} // end SmTrimmingTools::TrimSurfaceWithModelSpaceCurves

/*******************************************************************//**
PURPOSE: Powerful method to create a trimmed surface(s) in an existing Brep.

  NOTES:
     It specifically handles the following cases:

    1) The loops are not ordered correctly, that is, the outer loop
       is not first in the list.

    2) The loops are not oriented correctly, that is, the outer loop
       goes clockwise around the surface normal or an inner loop goes
       counter-clockwise.

    (Note that one of the two conditions must be TRUE -- the loops must
    be either oriented correctly or ordered correctly.)

    3) One or more of the parameter space curves lie entirely on a singularity.

   It also runs a number of checks to verify the surface has been correctly created.

    If the surface is not G1-continuous more than one face may be generated.
***********************************************************************/
SmStatus SmTrimmingTools::TrimSurfaceWithParameterSpaceCurves
 (SmContext const                   & crContext,                   // NotUsed: in : 
  SmRegion                          * pRegion,                     // in :  Region in an existing brep in which new trimmed surface is to be created.
  double                              d3DTolerance,                // in : 
  const SmTArray<ULONG>             & crCurveLoops,                // in : 
  const SmTArray<SmBSplineCurve*>   & cpUVCurves,                  // in :  Parameter space curves which are consumed in this operation.
  const SmTArray<SmOrientType>      & crCurveOrientations,         // in : 
  const SmTArray<SmPoint3d>         & crLoopPoints,                // in : 
  SmSurface                         * pSurface,                    // in :  Surface which is consumed in this operation.
  const SmExtent2d                  & crUVDomain,                  // in : 
  SmOrientType                        eSurfaceOrientation,         // in : 
  SmBoolean                           bFirstLoopIsOuterLoop,       // in : 
  SmBoolean                           bLoopsAreOrientedCorrectly)  // in : 
{
  SM_REF1(crContext) ;
    SmObjDelete sSurfaceClean (pSurface);

    NER(pRegion);
    SmBrep *pBrep = pRegion->GetBrep();
    NER(pBrep);
    SM_ASSERT (bLoopsAreOrientedCorrectly || bFirstLoopIsOuterLoop);

    SmTArray<ULONG> loop_curve_counts;
    SmTArray<SmBSplineCurve*> uv_curves;
    SmTArray<SmOrientType> curve_orientations;
        // CopyNonDegenerate transfers cpUVCurves to uv_curves.
        // If CopyNonDegenerate has an internal error, some curves may
        // not be deleted.

    SmTArray<SmBSplineCurve*> tmp_uv_curves;
    tmp_uv_curves.Append(cpUVCurves);
    SmObjsDelete<SmBSplineCurve*> sCurvesClean (&tmp_uv_curves);

    SER (CopyNonDegenerate (crCurveLoops, tmp_uv_curves, crCurveOrientations,
                            pSurface, d3DTolerance,
        loop_curve_counts, uv_curves, curve_orientations));
    sCurvesClean.SetArray (&uv_curves);

    if (!bLoopsAreOrientedCorrectly)
        SER (OrientUVLoops (loop_curve_counts, uv_curves, curve_orientations));

    SmBoolean surface_is_g1_continuous;
    SER (SurfaceContinuityCheck (pSurface,
        surface_is_g1_continuous));

    SmRegion *sNewRegion = NULL;
    SmShell *sNewShell = NULL;
    SmTArray<SmFace *> sNewFaces;

    sSurfaceClean.Clear ();
    sCurvesClean.Clear ();
    ULONG ii;
    if (surface_is_g1_continuous)
    {
        SmFace *sNewFace = NULL;
        SER (pBrep->MakeFaceWithCurves (pRegion,              // in : region to contain new topology objects
                                       loop_curve_counts,     // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop
                                       0,                     // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts
                                       &uv_curves,            // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts
                                       curve_orientations,    // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE
                                       crLoopPoints,          // in : Point positions to build SmVertex VertexLoops
                                       pSurface,              // in : new face->Surface
                                       crUVDomain,            // in : domain of Surface used by face
                                       eSurfaceOrientation,   // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE
                                       sNewRegion,            // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids.
                                       sNewShell,             // out: New shell if any.  Trimmed surfaces always create a new shell.
                                       sNewFace));            // out: the new face
        sNewFaces.Add (sNewFace);
    }
    else
    {
        SER (pBrep->MakeFacesWithCurves (pRegion,             // in : region to contain new topology objects
                                        loop_curve_counts,    // in : 1 entry per loop, value = loop edge count, 1st entry=outer loop
                                        0,                    // in : opt ordered 3d trimming curves assigned to loops per sLoopEUCounts
                                        &uv_curves,           // in : opt ordered 2d trimming curves assigned to loops per sLoopEUCounts
                                        curve_orientations,   // in : associated orients for each trimming curve, SM_OT_SAME or SM_OT_OPPOSITE
                                        crLoopPoints,         // in : Point positions to build SmVertex VertexLoops
                                        pSurface,             // in : new face->Surface
                                        crUVDomain,           // in : domain of Surface used by face
                                        eSurfaceOrientation,  // in : Surface orient, oneof SM_OT_SAME or SM_OT_OPPOSITE
                                        sNewRegion,           // out: New region if any. NULL when building trimmed surfaces, may be NotNULL for solids.
                                        sNewShell,            // out: New shell if any.  Trimmed surfaces always create a new shell.
                                        sNewFaces));          // out: the new face

        for (ii = 0; ii < sNewFaces.GetSize (); ii++)
        {
            SER (RedropUVTrimCurves (sNewFaces [ii]));
            SmBoolean tiny_gap_repair_made;
            SER (FixTrimLoopTinyParameterSpaceGaps (sNewFaces [ii], SM_ZONE_TOL_3D/10.0,
                tiny_gap_repair_made));
        }
    }

    for (ii = 0; ii < sNewFaces.GetSize (); ii++)
    {
        if (bLoopsAreOrientedCorrectly)
        {
            SmBoolean loops_reordered;
            SER (FixTrimLoopOrder(sNewFaces[ii], loops_reordered ));
        }
        else
        {
            // This should not be necessary, because we've oriented them on the
            // way in.  However, for reasons I don't understand, sometimes
            // that fails and this doesn't.

            ULONG loops_reoriented;
            SER (FixTrimLoopOrientation(sNewFaces[ii], loops_reoriented ));
        }
    }

    SER (CheckFaces (sNewFaces, d3DTolerance));

    return SM_SUCCESS;

} // end SmTrimmingTools::TrimSurfaceWithParameterSpaceCurves

/*******************************************************************//**
PURPOSE: Powerful method to create an SmBrep containing a trimmed surface.

NOTES:
     It specifically handles the following cases:

    1) The loops are not ordered correctly, that is, the outer loop
       is not first in the list.

    2) The loops are not oriented correctly, that is, the outer loop
       goes clockwise around the surface normal or an inner loop goes
       counter-clockwise.

    (Note that one of the two conditions must be TRUE -- the loops must
    be either oriented correctly or ordered correctly.)

    3) One or more of the parameter space curves lie entirely on a singularity.

   It also runs a number of checks to verify the surface has been correctly created.

    If the surface is not G1-continuous more than one face may be generated.
***********************************************************************/
SmStatus SmTrimmingTools::TrimSurfaceWithParameterSpaceCurves
  (SmContext const                   & crContext,
   double                              d3DTolerance,
   const SmTArray<ULONG>             & crCurveLoops,
   const SmTArray<SmBSplineCurve*>   & cpUVCurves,            // Parameter space curves which are consumed in this operation.
   const SmTArray<SmOrientType>      & crCurveOrientations,
   const SmTArray<SmPoint3d>         & crLoopPoints,
   SmSurface                         * pSurface,              // Surface which is consumed in this operation.
   const SmExtent2d                  & crUVDomain,
   SmOrientType                        eSurfaceOrientation,
   SmBoolean                           bFirstLoopIsOuterLoop,
   SmBoolean                           bLoopsAreOrientedCorrectly,
   SmBrep                           *& rpBrep)
{
    rpBrep = new (crContext) SmBrep();
    if (rpBrep == 0)
        SER (SM_ERR_OUT_OF_MEMORY);
    // gwc: removed next line - sets rpBrep->Tol = crContext::ZoneTol3d
    //    rpBrep->SetTolerance (d3DTolerance);

    SER (TrimSurfaceWithParameterSpaceCurves (crContext,
                                              rpBrep->GetInfiniteRegion (),
                                              d3DTolerance, crCurveLoops,
                                              cpUVCurves, crCurveOrientations,
                                              crLoopPoints,
                                              pSurface, crUVDomain,
                                              eSurfaceOrientation,
                                              bFirstLoopIsOuterLoop,
                                              bLoopsAreOrientedCorrectly));
    return SM_SUCCESS;

} // end SmTrimmingTools::TrimSurfaceWithParameterSpaceCurves

/*******************************************************************//**
PURPOSE: Check whether all Edgeuses in a Loop have uv trim curves.

NOTES: Returns False if any Edgeuse/Mate pair is missing a uv curve.
***********************************************************************/
SmBoolean SmTrimmingTools::AreUVCurvesPresent( SmLoop const * pLoop )
{
  // loop locals
  SmLoopuse *loopuse;
  SmLoopuse *other_loopuse;
  pLoop->GetLoopuses (loopuse, other_loopuse);

  // loop->Edgeuses
  SmTArray<SmEdgeuse*> sEdgeuses;
  loopuse->GetEdgeuses (sEdgeuses);

  // for ever edgeuse
  ULONG ii, lNumEUs = sEdgeuses.GetSize();
  for (ii = 0; ii < lNumEUs; ii++)
    {
      // return FALSE when edgeuse has no UVTrimCurve
      if(sEdgeuses [ii]->GetUVTrimCurve () == NULL )
        { return FALSE; }
    }
  return TRUE;

} // end SmTrimmingTools::AreUVCurvesPresent

/*******************************************************************//**
PURPOSE: Check whether all Edgeuses in a Face have uv trim curves.

NOTES: Returns False if any Edgeuse is missing a uv curve.
***********************************************************************/
SmBoolean SmTrimmingTools::AreUVCurvesPresent
  (SmFace const *pFace)
{
  SmTArray<SmLoop *> sLoops;
  pFace->GetLoops (sLoops);
  ULONG ii, lNumLoops = sLoops.GetSize();
  for (ii = 0; ii < lNumLoops; ii++)
    {
      if (!AreUVCurvesPresent (sLoops [ii]))
        { return FALSE; }
    }
  return TRUE;

} // end SmTrimmingTools::AreUVCurvesPresent

/*******************************************************************//**
PURPOSE: Check whether all Edgeuses in a Brep have uv trim curves.

NOTES: Returns False if any Edgeuse is missing a uv curve.
***********************************************************************/
SmBoolean SmTrimmingTools::AreUVCurvesPresent
  (SmBrep const *pBrep)
{
  // brep faces
  SmTArray<SmFace *> sFaces;
  pBrep->GetFaces (sFaces);

  // for every face
  ULONG ii, lNumFaces = sFaces.GetSize();
  for (ii = 0; ii < lNumFaces; ii++)
    {
      // check face->edges have UVTrimCurves
      if (!AreUVCurvesPresent (sFaces [ii]))
        { return FALSE; }
    }

  // all done
  return TRUE;

} // end SmTrimmingTools::AreUVCurvesPresent

/*******************************************************************//**
PURPOSE: pass the call to SmFace::CreateUVTrimCurves which
            creates a trim curve for every face->edge.

NOTES: CreateUVTrimCurves can adjust vertex and edge tolerances
            to contain any 3d Gaps, but this function does not use
            that option.
***********************************************************************/
SmStatus SmTrimmingTools::CreateUVTrimCurves
  (SmFace const *pFace)             // in : target face
{
  double rdMeanCurveSurfaceGap;
  double rdMaxCurveSurfaceGap;
  double rdMean3DVertexGap;
  double rdMax3DVertexGap;
  double rdMeanPSCurveCurveGap;
  double rdMaxPSCurveCurveGap;

  // Latest version of CreateUVTrimCurves
  SER (pFace->CreateUVTrimCurves
           (FALSE,
            0, 0,    // cpOptUVCurves, cpOptOrientations
            rdMeanCurveSurfaceGap, rdMaxCurveSurfaceGap,
            rdMean3DVertexGap, rdMax3DVertexGap,
            rdMeanPSCurveCurveGap, rdMaxPSCurveCurveGap));

  return SM_SUCCESS;

} // end SmTrimmingTools::CreateUVTrimCurves

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmTrimmingTools::CreateUVTrimCurves
  (SmBrep const *pBrep)
{
    SmTArray<SmFace*> faces;
    pBrep->GetFaces (faces);
    ULONG ii, lNumFaces = faces.GetSize();
    for (ii = 0; ii < lNumFaces; ii++)
        SER (CreateUVTrimCurves (faces [ii]));
    return SM_SUCCESS;

} // end SmTrimmingTools::CreateUVTrimCurves

/*******************************************************************//**
PURPOSE: Delete uv trim curves on all Edgeuses of a Face.

NOTES:
***********************************************************************/
void SmTrimmingTools::ClearUVTrimCurves
  (SmFace const *pFace)
{
    SmTArray<SmEdge *> sEdges;
    pFace->GetEdges (sEdges);
    ULONG ii, jj, lNumEdges = sEdges.GetSize();
    for (ii = 0; ii < lNumEdges; ii++)
    {
        SmTArray<SmEdgeuse*> sEUs;
        sEdges[ii]->GetEdgeuses( sEUs );
        ULONG lNumEUs = sEUs.GetSize();
        for (jj = 0; jj < lNumEUs; jj++)
        {
            sEUs[jj]->SetUVTrimCurve (NULL, 0.0, TRUE) ; // TRUE = delete existing UVTrimCurve
        }
    }
}  // end SmTrimmingTools::ClearUVTrimCurves

/*******************************************************************//**
PURPOSE: Delete uv trim curves on all Edgeuses in a Brep.

NOTES:
***********************************************************************/
void SmTrimmingTools::ClearUVTrimCurves
  (SmBrep const *pBrep)
{
    SmTArray<SmFace*> sFaces;
    pBrep->GetFaces (sFaces);
    ULONG ii, lNumFaces = sFaces.GetSize();
    for (ii = 0; ii < lNumFaces; ii++)
      { ClearUVTrimCurves( sFaces[ii] ); }

} // end SmTrimmingTools::ClearUVTrimCurves

/*******************************************************************//**
PURPOSE: Delete and recreate uv trim curves on all Edgeuses of a Face.

NOTES:
***********************************************************************/
SmStatus SmTrimmingTools::RedropUVTrimCurves
  (SmFace *pFace)
{
    ClearUVTrimCurves (pFace);
    SER (CreateUVTrimCurves (pFace));
    return SM_SUCCESS;

} // end SmTrimmingTools::RedropUVTrimCurves

/*******************************************************************//**
PURPOSE: Delete and recreate uv trim curves on all Edgeuses in a Brep.

NOTES:
***********************************************************************/
SmStatus SmTrimmingTools::RedropUVTrimCurves
  (SmBrep *pBrep)
{
    ClearUVTrimCurves (pBrep);
    SER (CreateUVTrimCurves (pBrep));
    return SM_SUCCESS;

} // end SmTrimmingTools::RedropUVTrimCurves

#if 0 // UNUSED 
/*******************************************************************//**
PURPOSE: Get the vertexuse use associated with a vertexuse.

NOTES:
***********************************************************************/
static SmVertexuse *GetVertexuseOfFace
  (SmVertex const *vertex,
   const SmFace *pFace)
{
    SmVertexuse *pRet = NULL;

    if(pFace == NULL )
        return pRet;

    SmVertexuse *pData[32];
    SmTArray<SmVertexuse*> sVertexuses(32,pData);

    vertex->GetVertexuses (sVertexuses);
    ULONG ii;
    for(ii=0; ii < sVertexuses.GetSize(); ii++ )
    {
        SmVertexuse *pEU = sVertexuses[ii];
        if(pFace == pEU->GetFaceuse()->GetFace () )
        {
            pRet = pEU;
            break;
        }
    }
    return pRet;

} // end GetVertexuseOfFace
#endif

/*******************************************************************//**
PURPOSE: Split the targetFace with all Face->Surface->SeamCurves.

NOTES:
  1. This is an older approach to healing superseded by the newer
     SmBrep::HealBrep() method.  For now, try calling
     SmBrep::HealBrep() with thisFace loaded into the
     input pOptFaces input argument.

  2. Use this function to change a cylinder with two end curves
  (not a legal SMLib representation) into a cylinder with one
  outer loop containing 3 edges (a legal SMLib representation).
  This also works for a full surface of revolution with a pole
  at the center, with edges only around the outer boundary
  and not along the seams.

  A face can be split into two or more when split by seam curves.
  All new faces are placed in the rpNewFaces output array.

  A SeamCurve is an isoParameter curve on a closed surface boundary and
  when the surface is closed and bAlsoSplitAtMiddleOfClosedSurfaces == TRUE
  another isoParameter curve at the center of the surface domain.

  Side effects:
  - Finds one outer loop setting its orientation bits to
    SM_OT_SAME and for every other face->loop
    sets the orientation bits to label the loops
    as inner loops.
  - Might add a vertex at a singularity (and edges to it).

  increments unlocked mark value
***********************************************************************/
SmStatus SmTrimmingTools::SplitAtSeams
  (const SmContext   & crContext,                          // NotUsed: in : context for new object construction
   SmFace            * pFace,                              // in : target face
   SmTArray<SmFace*> & rNewFaces,                          // out: array of new faces appended to by this result
   SmBoolean           bAlsoSplitAtMiddleOfClosedSurfaces) // NotUsed: in : TRUE = when surface is closed, also split face
                                                           //             at midSurface isoParameterCurve.
                                                           //      FALSE= just split face at closed boundaries
{
  SM_REF2(crContext, bAlsoSplitAtMiddleOfClosedSurfaces) ;
  SmTArray<SmEdge *>   sNewSeamEdges ; 
  SmTArray<SmVertex *> sNewSeamVertices ;
  SmCurveClassification sCrvClassU ;
  SmCurveClassification sCrvClassV ;
  SmSurfParamType eCrossedSeam ;
  SmSurfParamType eMissingSeam ;
  SmSurfParamType eNearMissSeam ;

  // when Face has Seam problems
  if(pFace->HasSeamProblem(sCrvClassU, 
                           sCrvClassV, 
                           eCrossedSeam, 
                           eMissingSeam, 
                           eNearMissSeam))
    {
      // this is an obsolete approach to healing - 
      // in the future: Try the new SmBrep::HealBrep with thisFace in the pOptFaces list argument
  // Removed obsolete code in favor of new SmFace::SplitAtSeam 02/24/2020. See v8831 to retrieve code
      return (pFace->SplitAtSeam( sCrvClassU, 
                                  sCrvClassV, 
                                  eCrossedSeam, 
                                  eMissingSeam, 
                                  eNearMissSeam,
                                 &rNewFaces, 
                                  NULL,
                                  NULL,
                                  NULL));
    }

  // all done - no problems branch
  return(SM_SUCCESS) ;

} // end SmTrimmingTools::SplitAtSeams

/*******************************************************************//**
PURPOSE: Pass the SplitAtSeams call along to every face in target Brep
       so that each face can be split along its surface's seam boundaries.

NOTES:
  Before splitting any face, every face is checked to make sure
  that (1) it has a surface, and (2) its surface has an owner.
  An error is signalled and the function returns having
  made no changes when missing pointers are detected.
***********************************************************************/
SmStatus SmTrimmingTools::SplitAtSeams
  (SmContext const   & crContext,                // in : context for new object construction
   SmBrep            * pBrep,                    // in : target Brep
   SmTArray<SmFace*> & rpNewFaces,               // out: resulting faces created by operation
   SmBoolean bAlsoSplitAtMiddleOfClosedSurfaces) // in : TRUE = when surface is closed, also split face
                                                 //             at midSurface isoParameterCurve.
                                                 //      FALSE= just split face at closed boundaries
{
  // get all brep faces
  SmTArray<SmFace*> sFaces;
  pBrep->GetFaces (sFaces);
  ULONG ii, lNumFaces = sFaces.GetSize();

  // check brep: for every face
  // check face->surface and surface->face pointers
  for (ii = 0; ii < lNumFaces; ii++)
    {
      SmFace *pFace = sFaces[ii];
      SmSurface *pSurf = pFace->GetSurface();
      if(pSurf == NULL || pSurf->GetFace() == NULL )
        {
          // missing pointers - return and signal error
          SER (SM_ERR);
        }
    }  // end iter every face

  SmTemporaryChangeValue< SmBoolean > sDoingBool( SM_CONST_CAST(SmContext*, &crContext)->GetDoingBooleanRef(), TRUE );
  // for every face - pass the call along
  SmTArray<SmFace*> sNewFaces;
  for (ii = 0; ii < lNumFaces; ii++)
    {
      SmFace *pFace = sFaces[ii];
      SER (SplitAtSeams (crContext,
                         pFace,
                         sNewFaces,
                         bAlsoSplitAtMiddleOfClosedSurfaces)); // note: increments unlocked mark value

      if (sNewFaces.GetSize () > 0)
        { rpNewFaces.Append (sNewFaces); }
    }

  // TODO: Should we make uv curves here?
  //  No: SplitAtSeams( pFace ) does that already.
  //  And it's probably not necessary anyway, they'll be created when needed.

  return SM_SUCCESS;

} // end SmTrimmingTools::SplitAtSeams

// decided not to build this method - not needed for the PRC<->SMB project
//  /*******************************************************************//**
//  PURPOSE: extract all loops made from a connected network of edges.
//  
//  NOTES: 1. Tolerances: EdgeEnds within tolerance of one another are assumed
//                        to connect
//  
//         1. a. on input : Assumes edges are connected head to tail
//                          Presumes no orientation data
//            b. on output: both sides of every input edge are sorted
//                          into a set of CW and CCW loops.
//            c. rules:
//              - 2 connects          : when every vertex (EdgeEnd/EdgeEnd connection)
//                                      connects exactly 2 edges, then the output
//                                      will have 1 CCW loop and 1 CW loop where
//                                      both loops use all edges in opposite order.
//              - More than 2 connects: when any vertex (EdgeEnd/EdgeEnd connection)
//                                      connects to more than 2 edges, then the output
//                                      will have 1 CCW loop and 1 or more CW Loops where
//                                      the CW loops won't be using the same edges as
//                                      the CCW loop.
//              - 1 connect           : when any vertex (EdgeEnd/EdgeEnd connection)
//                                      connects to 1 edge - that edge will be a crack
//                                      or a spoke and will be used twice in just one
//                                      loop.
//         
//         2. Loops have sides.  Consider the curves on a single face
//         
//         
//                            Curves on a face                 
//                 *============*          *=============* 
//                 "            "          "             " 
//                 "            "          "             " 
//                 "            *==========*             " 
//                 "            "          "             " 
//                 "            "          "             " 
//                 *============*          *=============*
//                 
//                       Associated loops on the face
//               o----------------o      o-----------------o
//               |                |      |                 |
//               |   o--------o   |      |   o---------o   |
//               |   |        |   o------o   |         |   |
//               |   ++Loop 0 o              o  Loop 1++   |
//               |   |        |   o------o   |         |   |
//               |   o--------o   |      |   o---------o   |
//               |                |      |                 |
//               o----------------o      o-----------+-----o
//                                             Loop 2+
//  
//                                   *
/*                                  / \                                     */
/*                                 *---*                                    */
/*                                / \ / \                                   */
//                               *---*---*
//         
//          a. Loops are composed of the sides of the curves that define them.
//             This example shows one set of loop curves that make 3 loops.
//         
//             Key: *   vertex             o  loop EdgeEnd/EdgeEnd connections (can have a gap)
//                 ===  horizontal curve  --- horizontal edge side (can map to an edgeuse) 
//                  "    vertical curve    |  vertical   edge side (can map to an edgeuse)
//         
//          b. An edge can be used two times within a single loop since each loop
//             uses the sides of an edge and not the edge itself.  This example
//             shows the outer loop using the middle edge two times.
//            
//  ***********************************************************************/
//    SmStatus SmTrimmingTools::FindTrimmingLoopsInConnectedCurves
//   (SmSurface                & rSurface,                             ///< [in] : Surface to trim                            
//      SmTArray<SmCurve*>     & rCurvesInOneLoop,                     ///< [in] : Loop to trim surface:                      
//      SmTArray<SmOrientType> & rCurveOrientationsInOneLoop,          ///< [i/o]: associated Trim curve orientations         
//                                                                     ///<        oneof: SM_OT_SAME, SM_OT_OPPOSITE          
//      SmOrientType           & rOrientationWithSurface)              ///< [in] : SM_OT_SAME     = loop on surface up side   
//                                                                     ///< [in] : SM_OT_OPPOSITE = loop on surface down side 
//  {
//  } // end SmTrimmingTools::FindTrimmingLoopsInConnectedCurves
// end decided not to build this method now

/*******************************************************************//**
PURPOSE: Find Loop Curve orientations

NOTES:
METHOD: 1. Given: a loop of curves connected End-to-End
           Find : Orientations for each curve. Walk the loop picking an orientation
           consistent with the specified StartEdge start directions and the left-hand rule.
            a. Left-hand rule:  The loop will be walked from the first curve in the
               specified direction keeping the material on the left hand side of an observer,
               oriented with the surface normal, walking the sequence of curves in the loop.
               If you want the other side of the loop, call this function with the
               input eFirstCurveOrientation set in the other direction.
        
        2. Although it's expected that this method will only be given arrays of curves
           properly sequenced to fill in a loop, the input curve array will be
           reordered into the sequence that the curves were encountered on the
           walk and the rCurveOrientations will be set with the associated orientation
           for each curve in the input.

        3. A curve used twice in one loop will be listed twice in the output even
           if the curve is only listed once in the input.

        4. Seams, cracks and structs can be represented by listing the same curve twice
           in the input curve array.

        4. When a loop walk does not encounter every input curve the
           curve array will be resized to the number of curves that the walk
           did encounter.  Curves used as seams, cracks or spokes will be listed twice.
            a. Don't use the output array as a memory manager for the curve list.
               The length of the list might be shortened by this function and if
               the only handle the caller has to the curve is this list that curve
               will end up a memory leak.
***********************************************************************/
SmStatus SmTrimmingTools::FindLoopCurveOrientations    
(SmSurface                & rSurface,          ///< [in] : Surface to trim                                            
   SmTArray<SmCurve*>     & rCurvesInOneLoop,  ///< [in] : curves to make into a surface loop (dim == 2 or 3)         
   SmOrientType             eFirstCurveOrient, ///< [in] : SM_OT_SAME     = walk with 1st crv parameterization dir    
                                               ///<      : SM_OT_OPPOSITE = walk against 1st crv parameterization dir 
   SmTArray<SmCurve*>     & rOrderedCurves,    ///< [out]: Curves ordered head to tail                                
   SmTArray<SmOrientType> & rCurveOrients)     ///< [out]: associated Trim curve orientations                         
                                               ///<        oneof: SM_OT_SAME, SM_OT_OPPOSITE                          
{
  // locals
  ULONG ii, jj ;
  ULONG                  lCurveCnt = rCurvesInOneLoop.GetSize() ;
  SmPoint2d              sThisCurveEndUV[2] ;            
  SmPoint3d              sThisCurveEnd3d[2] ;
  SmPoint2d              sNextCurveStartUV[2], sNextCurveEndUV[2] ;
  SmPoint3d              sNextCurveStart3d[2], sNextCurveEnd3d[2] ;
  SmTArray<SmCurve*>     sNextCurves ; 
  SmTArray<SmPoint3d>    sNextCurvePos3ds ;
  SmTArray<SmPoint3d>    sNextCurve1stDir3ds ;
  SmTArray<SmOrientType> sNextCurveOrients ;
  SmVector3d             sSurfaceNormal ;
  SmBoolean              bSuccess = UNSURE ;
  SmBoolean              bIsMulti;
  double                 dGap = 0.0;

  // check input - bad first curve orientation
  AERS_MSG(   eFirstCurveOrient == SM_OT_SAME
           || eFirstCurveOrient == SM_OT_OPPOSITE,
           _T("SmTrimmingTools::SetLoopCurveOrientations: bad input - eFirstCurveOrient not either SM_OT_SAME or SM_OT_OPPOSITE")) ;

  // check input - No TrimCurves
  AERS_MSG(lCurveCnt > 0, _T("SmTrimmingTools::SetLoopCurveOrientations: bad input - TrimCurve array is empty")) ;

  // init output
  rCurveOrients.ReSet() ;
  rOrderedCurves.Add(rCurvesInOneLoop[0]) ;
  rCurveOrients .Add(eFirstCurveOrient) ;

  // for every curve
  for(ii=0;ii<lCurveCnt;ii++)
    {
      // ThisCurve locals
      SmCurve     * pThisCurve     = rCurvesInOneLoop.GetLast() ;
      SmOrientType  sThisOrient    = rCurveOrients.GetLast() ;
      SmZoneTol3d   sThisZoneTol3d = SmTol::GetZoneTol3d(pThisCurve) ;
      SmExtent1d    sThisIvl       = pThisCurve->GetNaturalInterval() ;

      // init nextCurve arrays
      sNextCurves.ReSet() ;          // list of curves coincident to a target curve end point
      sNextCurvePos3ds.ReSet() ;     // associated Curve pos
      sNextCurve1stDir3ds.ReSet() ;  // associated Curve 1st deriv
      sNextCurveOrients.ReSet() ;    // associated Curve orientation: SM_OT_SAME=CurvePos is StartPos, SM_OT_OPPOSITE=CurvePos is EndPos.

      // ThisCurve 2d and 3d evaluate: {ThisCurveEndUV[2], ThisCurveEnd3d[2]}
        {
          // ThisCurve evaluate(dim=2 or 3)
          pThisCurve->Evaluate(sThisOrient == SM_OT_SAME ? sThisIvl.GetMin() : sThisIvl.GetMax(), 1, TRUE, sThisCurveEnd3d) ;

          // 2d branch
          if(pThisCurve->GetDim() == 2)
            {
              sThisCurveEndUV[0] = sThisCurveEnd3d[0] ;
              sThisCurveEndUV[1] = sThisCurveEnd3d[1] ;
          
              rSurface.EvaluateDirectionalDerivs(sThisCurveEndUV[0], sThisCurveEndUV[1], 1, sThisCurveEnd3d, TRUE, TRUE) ;
            } // end curve in 2d branch

          // 3d branch
          else // Curve in 3d branch
            {
              rSurface.DropPoint(sThisCurveEnd3d[0], rSurface.GetNaturalUVDomain(), NULL, bSuccess, sThisCurveEndUV[0], dGap, bIsMulti) ;
              rSurface.DropVectors(sThisCurveEndUV[0], TRUE, TRUE, 1, &sThisCurveEnd3d[1], &sThisCurveEndUV[1]) ;
            }
        } // end scope - {ThisCurveEndUV[2], ThisCurveEnd3d[2]} from 2d or 3d Curves

      // surface normal
      rSurface.EvaluateNormal(sThisCurveEndUV[0], TRUE, TRUE, sSurfaceNormal) ; 

      // for every curve - check for multiple curves connecting to the same point3d - remember curves can be closed
      // GWC note:  Assuming lCurveCnt is small this code is okay being N**2.  We could use the SmObjsInVoxels class to reduce that for large CurveCnt cases.
      for(jj=0;jj<lCurveCnt;jj++)
        {
          // locals
          SmCurve    * pNextCurve     = rCurvesInOneLoop[ii] ;
          SmZoneTol3d  sNextZoneTol3d = SmTol::GetZoneTol3d(pNextCurve) ;
          SmExtent1d   sNextIvl       = pNextCurve->GetNaturalInterval() ;
          SmXSectTol3d sXSectTol3dSq  = (sThisZoneTol3d + sNextZoneTol3d) * (sThisZoneTol3d + sNextZoneTol3d) ;

          // NextCurve start evaluate(dim=2 and 3): {sNextCurveStartUV[2], sNextCurveStart3d[2]}
          // NextCurve end   evaluate(dim=2 and 3): {sNextCurveEndUV[2],   sNextCurveEnd3d[2]}
            {
              // NextCurve evaluate(dim=2 or 3)
              pNextCurve->Evaluate(sNextIvl.GetMin(), 1, TRUE, sNextCurveStart3d) ;
              pNextCurve->Evaluate(sNextIvl.GetMax(), 1, TRUE, sNextCurveEnd3d) ;

              // 2d branch
              if(pNextCurve->GetDim() == 2)
                {
                  sNextCurveStartUV[0] = sNextCurveStart3d[0] ;
                  sNextCurveStartUV[1] = sNextCurveStart3d[1] ;
                  sNextCurveEndUV[0]   = sNextCurveEnd3d[0] ;
                  sNextCurveEndUV[1]   = sNextCurveEnd3d[1] ;
              
                  rSurface.EvaluateDirectionalDerivs(sNextCurveStartUV[0], sNextCurveStartUV[1], 1, sNextCurveStart3d, TRUE, TRUE) ;
                  rSurface.EvaluateDirectionalDerivs(sNextCurveEndUV[0],   sNextCurveEndUV[1],   1, sNextCurveEnd3d,   TRUE, TRUE) ;
                } // end curve in 2d branch

              // 3d branch
              else // Curve in 3d branch
                {
                  rSurface.DropPoint  (sNextCurveStart3d[0], rSurface.GetNaturalUVDomain(), NULL, bSuccess, sNextCurveStartUV[0], dGap, bIsMulti) ;
                  rSurface.DropVectors(sNextCurveStartUV[0], TRUE, TRUE, 1, &sNextCurveStart3d[1], &sNextCurveStartUV[1]) ;

                  rSurface.DropPoint  (sNextCurveEnd3d[0], rSurface.GetNaturalUVDomain(), NULL, bSuccess, sNextCurveEndUV[0], dGap, bIsMulti) ;
                  rSurface.DropVectors(sNextCurveEndUV[0], TRUE, TRUE, 1, &sNextCurveEnd3d[1], &sNextCurveEndUV[1]) ;
                }
            } // end scope - NextCurveEndUV and NextCurveEnd3d from 2d or 3d Curves
          
          // check for CurveStart coincidence 
          if(  (  pNextCurve != pThisCurve                  //    check all other CurveStarts
                 || sNextCurveOrients[ii] == SM_OT_SAME)      // or ThisCurveStart when working with ThisCurveEnd 
             &&(  sThisCurveEnd3d[0].DistanceBetweenSquared(sNextCurveStart3d[0]) < sXSectTol3dSq))
            {
              sNextCurves        .Add(pNextCurve) ;
              sNextCurvePos3ds   .Add(sNextCurveStart3d[0]) ; 
              sNextCurve1stDir3ds.Add(sNextCurveStart3d[1]) ; 
              sNextCurveOrients  .Add(SM_OT_SAME) ;
            }  // end CurveStart coincidence check

          // check for CurveEnd coincidence
          if(  (  pNextCurve != pThisCurve                  //    check all other CurveStarts
                 || sNextCurveOrients[ii] == SM_OT_OPPOSITE)  // or ThisCurveEnd when working with ThisCurveStart 
             &&(  sThisCurveEnd3d[0].DistanceBetweenSquared(sNextCurveEnd3d[0]) < sXSectTol3dSq))
            {
              sNextCurves        .Add(pNextCurve) ;
              sNextCurvePos3ds   .Add(sNextCurveEnd3d[0]) ;
              sNextCurve1stDir3ds.Add(sNextCurveEnd3d[1]) ;   
              sNextCurveOrients  .Add(SM_OT_OPPOSITE) ;
            } // end CurveEnd coincidence check

        }  // end iter jj, every curve looking for NextCurveEnd Pt coincidences to ThisCurveEnd pt

     // check state - Every curve should connect head-to-tail to at least one other curve (even if that is itself)
     AERN_MSG(sNextCurves.GetSize() >= 1, SM_ERR_ASSERT_FAILURE, _T("SmTrimmingTools::SetLoopCurveOrientations: Loop has CurveEnd/CurveEnd gap larger than XSect distance - aborting")) ;

     // arrive here: pThisCurve     = current LoopCurve Tgt seeking a next LoopCurve connection
     //              sThisZoneTol3d = associated CurrentLoopCurve prop
     //              sThisIvl       = associated CurrentLoopCurve prop
     //              sThisOrient    = associated CurrentLoopCurve prop
     //              {sThisCurveEndUV[2], sThisCurveEnd3d[2]} = ThisCurve 2d and 3d [pos, 1stDir] pairs
     // 
     //              sNextCurves         = list of curve ends coincident with pThisCurve, sThisCurveEndUV, sThisCurveEnd3d
     //              sNextCurvePos3ds    = associated sNextCurveEnd coincident Pos3ds
     //              sNextCurve1stDir3ds = associated sNextCurveEnd coincident 1stDir3ds
     //              sNextCurveOrients   = associated sNextCurve orients SM_OT_SAME     = coincident Start CurveEnd
     //                                                                  SM_OT_OPPOSITE = coincident End CurveEnd
     // next:        Set lTgtIndx = sNextCurves[lTgtIndx] of the next curve in the loop
     //              accumulate output: rOrderedCurves, rCurveOrients.

     // when ThisCurveEnd is coincident to multiple potential NextCurveEnd candidates - place Closest Clockwise neighbor at end of NextCurves list
     ULONG lLastIndx = sNextCurves.GetSize() - 1 ; 
     if(sNextCurves.GetSize() > 0)
       {
         double dAngRad, dMinAngRad ;

         // This and Next Curve End Vectors pointing away from the CoincidentPosition
         SmVector3d sThisCrvAway = (sThisOrient                  == SM_OT_SAME) ? sThisCurveEnd3d[1] :             -sThisCurveEnd3d[1] ;
         SmVector3d sNextCrvAway = (sNextCurveOrients[lLastIndx] == SM_OT_SAME) ? sNextCurve1stDir3ds[lLastIndx] : -sNextCurve1stDir3ds[lLastIndx] ;
         sSurfaceNormal.CCWAngleBetween(sThisCrvAway, sNextCrvAway, dMinAngRad) ; 

         // For every candidate - find and place the most left-handed NextCurve at the end of the sNextCurves Array
         for(jj=lLastIndx;jj>0;jj--)
           {
             // Next Curve End Vector pointing away from the CoincidentPosition
             sNextCrvAway = (sNextCurveOrients[jj-1] == SM_OT_SAME) ? sNextCurve1stDir3ds[jj-1] : -sNextCurve1stDir3ds[jj-1] ;
             sSurfaceNormal.CCWAngleBetween(sThisCrvAway, sNextCrvAway, dAngRad) ;

             // place candidates closer to ThisCrvAway in the ClockWise direction at end of the sNextCurves array
             SM_ASSERT_MSG(SM_ARE_SAME(dAngRad, dMinAngRad) == FALSE, _T("SmTrimmingTools::FindLoopCurveOrientations: Found a loop where multiple edges are coincident and tangent at a single vertex - method needs extensions here to handle this case. order those curves by sampling pts moving away from the vertex.")) ; 
             if(dAngRad < dMinAngRad)
               {
                 // swap NextCurve data
                 SM_SWAP_PTR(SmCurve,      sNextCurves        [jj-1], sNextCurves        [lLastIndx]) ;
                 SM_SWAP    (SmPoint3d,    sNextCurvePos3ds   [jj-1], sNextCurvePos3ds   [lLastIndx]) ; 
                 SM_SWAP    (SmPoint3d,    sNextCurve1stDir3ds[jj-1], sNextCurve1stDir3ds[lLastIndx]) ; 
                 SM_SWAP    (SmOrientType, sNextCurveOrients  [jj-1], sNextCurveOrients  [lLastIndx]) ; 

                 // store best angle found
                 dMinAngRad = dAngRad ;
              } // end found a better candidate check
           } // end iter jj, every candidate NextCurfve looking for closest CW neighbor
       } // end ThisCurveEnd coincident to multiple potential NextCurveEnd candidates branch

      // When nextCurve is the same as the firstcurve - loop is closed and we are done
      if(   sNextCurves[lLastIndx] == rOrderedCurves[0]
         && sNextCurveOrients[lLastIndx] == rCurveOrients[0])
        {
          // all done
          return(SM_SUCCESS) ;
        }

      // else add NextCurve to the output arrays
      rOrderedCurves.Add(sNextCurves[lLastIndx]) ; 
      rCurveOrients .Add(sNextCurveOrients[lLastIndx]) ;

    } // end iter ii, every other Curve

  // arrive here when walking the curves did not find a closed loop
  return(SM_ERR) ;

} // end SmTrimmingTools::FindLoopCurveOrientations

/*******************************************************************//**
PURPOSE: For a loop, close up tiny uv-space gaps between Edgeuses.

NOTES:
***********************************************************************/
SmStatus SmTrimmingTools::FixTrimLoopTinyParameterSpaceGaps
  (SmLoop    *pLoop,            // in : target loop
   double     d2DTolerance,     // in : max allowed UVTrimCurve-UVTrimCurve gap
   SmBoolean &rbRepairsMade)    // out: TRUE = UVTrimCurves modified to close gaps
                                //      FALSE= NO UVTrimCurves were modified
{
  // init output
  rbRepairsMade = FALSE;

  // locals - get upward sEdgeuses
  SmBoolean bEditStartPoint ;
  SmTArray<SmEdgeuse*> sEdgeuses;
  SmLoopuse *pLoopuse, *pOtherLoopuse;
  pLoop->GetLoopuses(pLoopuse, pOtherLoopuse);
  pLoopuse->GetEdgeuses(sEdgeuses);

  // I don't think we need to worry about the pLoop reverse flags here,
  // because the pLoop is required to be closed (in 3d space) either way.

  // for every upward edgeuse
  ULONG ii, i0, lNumEUs = sEdgeuses.GetSize();
  for (ii=0; ii < lNumEUs; ii++)
    {
      i0 = (ii == 0) ? sEdgeuses.GetSize() - 1
                     : ii - 1 ;
      SmEdgeuse *pCurrEdgeuse = sEdgeuses[ii] ;
      SmEdgeuse *pPrevEdgeuse = sEdgeuses[i0] ;

      SmPoint2d sPrevUVStart, sPrevUVEnd;
      SmPoint2d sCurrUVStart, sCurrUVEnd;
      double dPrevStartWeight, dPrevEndWeight;
      double dCurrStartWeight, dCurrEndWeight;

      SER (GetEdgeUVExtremes (pPrevEdgeuse,
                              sPrevUVStart, dPrevStartWeight,
                              sPrevUVEnd,   dPrevEndWeight));
      SER (GetEdgeUVExtremes (pCurrEdgeuse,
                              sCurrUVStart, dCurrStartWeight,
                              sCurrUVEnd,   dCurrEndWeight));

      // Only fix them if they are fairly close to being right anyway.
      //   really large gaps happen at singularities where the loop
      //   needs to be closed by inserting a UVTrimCurve that runs the length
      //   of the singularity.
      double dUVGapSize = sPrevUVEnd.DistanceBetween (sCurrUVStart);
      if (dUVGapSize < 2.0 * d2DTolerance)
        {
          SmPoint3d     sTargetPoint = (sPrevUVEnd + sCurrUVStart) / 2.0;
          SmEditEndType eEditEndType = SM_EE_UVTRIMCURVE_END_POINT_AVERAGE ;

          // First edit the previous curve.
          // We want the end, so we look at the start only if the
          // edge's orientation is reversed.
          bEditStartPoint =   (pPrevEdgeuse->GetOrientation () == SM_OT_OPPOSITE)
                            ? TRUE
                            : FALSE ;
          SmBSplineCurve *pPrevUVCurve = pPrevEdgeuse->GetUVTrimCurve();
          if (pPrevUVCurve->IsRational())
               { SER(pPrevUVCurve->EditEndPoint(sTargetPoint * dPrevEndWeight,
                                                bEditStartPoint,
                                                &dPrevEndWeight,
                                                eEditEndType));
               }
          else { SER(pPrevUVCurve->EditEndPoint(sTargetPoint, bEditStartPoint, NULL, eEditEndType));
               }

          // Then edit the current curve.
          bEditStartPoint =   (pCurrEdgeuse->GetOrientation () == SM_OT_SAME)
                            ? TRUE
                            : FALSE ;
          SmBSplineCurve *pCurrUVCurve = pCurrEdgeuse->GetUVTrimCurve();
          if (pCurrUVCurve->IsRational())
               { SER(pCurrUVCurve->EditEndPoint(sTargetPoint * dCurrStartWeight,
                                                bEditStartPoint,
                                                &dCurrStartWeight,
                                                eEditEndType));
               }
          else { SER(pCurrUVCurve->EditEndPoint(sTargetPoint, bEditStartPoint, NULL, eEditEndType));
               }
          rbRepairsMade = TRUE;
        }
    } // end iter every upward edgeuse checking gaps

  // all done
  return SM_SUCCESS;

} // end SmTrimmingTools::FixTrimLoopTinyParameterSpaceGaps

/*******************************************************************//**
PURPOSE --For a face, close up tiny uv-space gaps between Edgeuses.

NOTES:
***********************************************************************/
SmStatus SmTrimmingTools::FixTrimLoopTinyParameterSpaceGaps
  (SmFace *pFace,
   double d2DTolerance,
   SmBoolean &rbRepairsMade)
{
    rbRepairsMade = FALSE;
    SmTArray<SmLoop *> sLoops;
    pFace->GetLoops (sLoops);
    ULONG ii, lNumLoops = sLoops.GetSize();
    for (ii = 0; ii < lNumLoops; ii++)
    {
        SmBoolean local_repairs_made;
        SER (FixTrimLoopTinyParameterSpaceGaps (sLoops [ii], d2DTolerance,
                local_repairs_made));
        if (local_repairs_made)
            rbRepairsMade = TRUE;
    }
    return SM_SUCCESS;

} // end SmTrimmingTools::FixTrimLoopTinyParameterSpaceGaps

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmTrimmingTools::EnsureLastEdgeuseUVCorrect
  (SmSurface const *pSurface,
   SmTArray<SmEdgeuse*> &rpEdgeuses,
   SmBoolean &rbAllEdgeusesCorrect)
{
    rbAllEdgeusesCorrect = FALSE;

    // Try to find an edge which is not an isocurve on a seam.
    ULONG dropped_correctly = rpEdgeuses.GetSize ();
    SmBoolean iso_curve_on_seam_present = FALSE;
    ULONG ii, lNumEUs = rpEdgeuses.GetSize();
    for (ii = 0; ii < lNumEUs; ii++)
    {
        SmBSplineCurve *edge = rpEdgeuses[ii]->GetUVTrimCurve ();
        SmSurfParamType side=SM_SP_U;  // init to dummy
        double parameter=0.0;
        if(IsIsoCurveOnSeam( pSurface, edge, side, parameter ))
          { iso_curve_on_seam_present = TRUE; }
        else
          { dropped_correctly = ii; }

        if(  iso_curve_on_seam_present
            && dropped_correctly < lNumEUs
           )
          { break; }
    }

    // No iso curve on seam, nothing to worry about!
    if (!iso_curve_on_seam_present)
    {
        rbAllEdgeusesCorrect = TRUE;
        return SM_SUCCESS;
    }

    // All curves are iso on seam, so the orientation is going to
    // be consistent.  All we can do is hope these cases are right.
    // (All the ones I've been able to think of should be.)
    if (dropped_correctly == lNumEUs)
    {
        rbAllEdgeusesCorrect = TRUE;
        return SM_SUCCESS;
    }

    rpEdgeuses.RotateArray((dropped_correctly + 1) < lNumEUs ?
                             (dropped_correctly + 1) : 0);

    return SM_SUCCESS;

} // end SmTrimmingTools::EnsureLastEdgeuseUVCorrect

/*******************************************************************//**
PURPOSE:

NOTES:
   This function assumes the loops are ordered, with the outer loop first.
***********************************************************************/
SmStatus SmTrimmingTools::FixBackwardsOuterLoopOnSeam
  (SmFace *pFace,
   SmBoolean &rbRepairMade)
{
    rbRepairMade = FALSE;
    SmSurface *surface = pFace->GetSurface ();
    SmExtent2d const bounds = surface->GetNaturalUVDomain ();
    SmBoolean closed [2];
    closed [0] = surface->IsClosed (bounds, SM_SP_U) ? TRUE : FALSE;
    closed [1] = surface->IsClosed (bounds, SM_SP_V) ? TRUE : FALSE;
    if (!closed [0] && !closed [1])
        return SM_SUCCESS;
    SmVector2d low (bounds.GetMin ());
    SmVector2d high (bounds.GetMax ());

    // This is the tolerance that tells us something is wrong.
    // We're looking for places where the one curve is on the other
    // side of the parameterization from next, so there should always
    // be a gap equal to high_u - low_u or high_v - low_v (within
    // tolerance).  We take the least of those two, and cut it in
    // half for good measure.
    double big_tolerance = Min (high [0] - low [0], high [1] - low [1]) / 2.0;

    // These are the tolerances we use to check to see if a correction
    // makes sense.
    SmVector2d const tolerance ((high [0] - low [0]) / 1000.0,
                                (high [1] - low [1]) / 1000.0);

    // Get the edgeuses on the first (ie outer) loop.
    SmTArray<SmLoop *> loops;
    pFace->GetLoops (loops);
    SmLoopuse *loopuse = NULL;
    SmLoopuse *other_loopuse = NULL;
    loops [0]->GetLoopuses (loopuse, other_loopuse);
    SmTArray<SmEdgeuse*> sEdgeuses;
    loopuse->GetEdgeuses (sEdgeuses);

    // First we try to find an edge which is not an isocurve on a seam.
    // The reason is that isocurves on seams dropped incorrectly is what
    // we are trying to fix.  If you start with one such, then you will
    // flip what follows to the wrong direction.  So we try to find a
    // segment which will drop correctly to be the last segment.
    SmBoolean all_edgeuses_correct;
    SER (EnsureLastEdgeuseUVCorrect (surface, sEdgeuses,
        all_edgeuses_correct));
    if (all_edgeuses_correct)
      { return SM_SUCCESS; }

    // Now look for gaps caused by incorrect seam curves.

    // (We don't need to worry about the loop reverse flags here, because
    // the loop is required to be closed (in 3d space) either way.)
    ULONG ii, jj, lNumEUs = sEdgeuses.GetSize();
    for (ii = 0; ii < lNumEUs; ii++)
    {
        ULONG previous;
        previous = (ii == 0 ? lNumEUs - 1 : ii - 1);

        SmPoint2d previous_start, previous_end;
        SER (GetEdgeUVExtremes (sEdgeuses [previous], previous_start, previous_end));
        SmPoint2d current_start, current_end;
        SER (GetEdgeUVExtremes (sEdgeuses [ii], current_start, current_end));

        if (previous_end.DistanceBetween (current_start) > big_tolerance)
        {
            for (jj = 0; jj < 2; jj++)
            {
                if (closed [jj]
                    && fabs (current_start [jj] - current_end [jj]) < tolerance [jj])
                {
                    if (fabs (current_start [jj] - high [jj]) < tolerance [jj])
                        SM_SWAP (double, low [jj], high [jj]);

                    if (fabs (current_start [jj] - low [jj]) < tolerance [jj])
                    {
                        SmPoint2d alt_start = current_start;
                        alt_start [jj] = high [jj];
                        if (previous_end.DistanceBetween (alt_start) < tolerance [jj])
                        {
                            if (IsIsoCurve (sEdgeuses [ii]->GetUVTrimCurve (),
                                            jj == 0 ? SM_SP_U : SM_SP_V,
                                            low [jj], tolerance [jj]))
                            {
                                SER (MirrorCurve
                                            (sEdgeuses [ii]->GetUVTrimCurve (),
                                             jj == 0 ? SM_SP_U : SM_SP_V,
                                             low [jj], high [jj],
                                             tolerance [jj]));
                                rbRepairMade = TRUE;
                            }
                        }
                    }

                    if (high [jj] < low [jj])
                        SM_SWAP (double, low [jj], high [jj]);
                }
            }
        } // end if prev end and current start too far apart
    } // end for each Edgeuse

    return SM_SUCCESS;

} // end SmTrimmingTools::FixBackwardsOuterLoopOnSeam

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmTrimmingTools::FixTrimLoopOrientation
  (SmFace *pFace, SmLoop *pLoop,
   SmBoolean bOuterLoop,
   SmBoolean &rbLoopReoriented)
{
    rbLoopReoriented = FALSE;
    // Determine if the orientation is correct
    SmOrientType orientation_desired = (bOuterLoop ? SM_OT_SAME
                                                   : SM_OT_OPPOSITE);
    SmOrientType loop_orientation;
    SER (pFace->ComputeLoopOrientation (pLoop,
        loop_orientation));
    if (loop_orientation != orientation_desired && loop_orientation != SM_OT_UNKNOWN)
    {
        if (bOuterLoop)
        {
            SER (pFace->FlipFaceOrientation ());
        }
        else
        {
            SER (pLoop->FlipLoopOrientation ());
        }

        rbLoopReoriented = TRUE;

    }
    return SM_SUCCESS;

} // end SmTrimmingTools::FixTrimLoopOrientation

/*******************************************************************//**
PURPOSE:

NOTES: This function assumes that the loops are correctly ordered, outer
   loop first.
***********************************************************************/
SmStatus SmTrimmingTools::FixTrimLoopOrientation
 (SmFace *pFace,
  ULONG &iNumberLoopsReoriented)
{
    iNumberLoopsReoriented = 0;
    SmTArray<SmLoop *> sLoops;
    pFace->GetLoops (sLoops);
    ULONG ii;
    for (ii = 0; ii < sLoops.GetSize (); ii++)
    {
        SmBoolean rbLoopReoriented;
        SER (FixTrimLoopOrientation (pFace, sLoops[ii], ii == 0, rbLoopReoriented));
        if (rbLoopReoriented)
          { iNumberLoopsReoriented++; }
    }
    return SM_SUCCESS;

} // end SmTrimmingTools::FixTrimLoopOrientation

/*******************************************************************//**
PURPOSE:

NOTES:
   Find the outer loop as the outer envelope in UV space of the
   min-max boxes of the edge curves of each loop..
   Make sure the outer loop is first. Set all other
   loops to inner.
***********************************************************************/

SmStatus SmTrimmingTools::FixTrimLoopOrder
  (SmFace *pFace,
   SmBoolean &rbLoopsReordered)

{
    rbLoopsReordered = FALSE;

    // Get the loops
    SmTArray <SmLoop *> sLoops;
    pFace->GetLoops(sLoops);
    ULONG lNumLoops = sLoops.GetSize();
    if (lNumLoops == 0)
        SER(SM_ERR); // no loops at all

    // Find the outer loop
    ULONG ii, jj, outer_index = 99999;
    SmLoop        * loop = 0;
    SmLoop        * inner_loop = 0;
    SmLoop        * outer_loop = 0;
    SmOrientType    loop_orientation;
    double          sSizeMax = -1.0;
    SmTArray <int>  sInners; //loop numbers for inner loops
    for (ii = 0; ii < lNumLoops; ii++)
    {
        loop = sLoops[ii];

        // find the 2d size of the loop and save the largest size
        SmTArray < SmEdge *>sEdges;
        loop->GetEdges(sEdges);
        SmExtent3d sUVBBox, sCrvBBox;

        ULONG lNumEdges = sEdges.GetSize();
        for (jj = 0; jj < lNumEdges; jj++)
        {
            SmEdge *pE = sEdges[jj];
            SmBSplineCurve *pCrv = pE->GetUVTrimCurveOfSurface(pFace->GetSurface());

            // get curve's UV bounding box
            SER(pCrv->CalculateBoundingBox(pCrv->GetNaturalInterval(), &sCrvBBox));

            // get union of all curve bounding boxes
            if (jj == 0)
            {
                sUVBBox = sCrvBBox;
            }
            else
            {
                sUVBBox.Union(sCrvBBox, sUVBBox);
            }
        } // end for each Edge

        double sSize = sUVBBox.GetSize().LengthSquared();
        if (outer_index == 99999)
        {
            // first "outer' candidate, save size and index
            sSizeMax = sSize;
            outer_index = ii;
        }
        else if (sSize > sSizeMax)
        {  // save the new best, put previous on inners list
            sSizeMax = sSize;
            sInners.Add(outer_index);
            outer_index = ii;
        }
        else    // not big enough ..add this to inners list
            sInners.Add(ii);

    } // end loop on all Loops


    // set direction of the outer loop
    outer_loop = sLoops[outer_index];

    // if outer is not the first
    if (outer_index !=0) { rbLoopsReordered = TRUE; }

    SER( pFace->ComputeLoopOrientation(outer_loop, loop_orientation ));
    if (loop_orientation == SM_OT_OPPOSITE)   // showing inner
    {
        FixTrimLoopOrientation (pFace, outer_loop, TRUE, rbLoopsReordered);
    }

    // set direction of the inner loops
    for (ii = 0; ii < sInners.GetSize(); ii++)
    {
        ULONG k = sInners[ii];
        inner_loop = sLoops[k];
        SER(pFace->ComputeLoopOrientation(inner_loop, loop_orientation));
        if (loop_orientation == SM_OT_SAME)   // showing outer
        {
            FixTrimLoopOrientation (pFace, inner_loop, FALSE, rbLoopsReordered);
        }
    }


    // Move outer to the first position
    SmLoopuse *loop_use1;
    SmLoopuse *loop_use2;
    outer_loop->GetLoopuses(loop_use1, loop_use2);
    SmFaceuse *face_use1 = loop_use1->GetFaceuse();
    NER(face_use1);
    SmFaceuse *face_use2 = loop_use2->GetFaceuse();
    NER(face_use2);
    SER(face_use1->Remove(loop_use1));
    SER(face_use2->Remove(loop_use2));
    SER(face_use1->PreInsert(loop_use1));
    SER(face_use2->PreInsert(loop_use2));

    // First set them all to inner loops
    for (ii = 0; ii < lNumLoops; ii++)
    {
        sLoops[ii]->GetLoopuses(loop_use1, loop_use2);
        loop_use1->SetOrientation(SM_OT_OPPOSITE);
        loop_use2->SetOrientation(SM_OT_OPPOSITE);
    }

    // Then set the outer loop
    outer_loop->GetLoopuses(loop_use1, loop_use2);
    loop_use1->SetOrientation(SM_OT_SAME);
    loop_use2->SetOrientation(SM_OT_SAME);


    return SM_SUCCESS;

} // end SmTrimmingTools::FixTrimLoopOrder

/*******************************************************************//**
   Utility functions for CreatePlanarFaceWith3DCurves
***********************************************************************/

/*******************************************************************//**
PURPOSE: Given a set of curves, order them so they represent
            the valid SMLib loop structure.

NOTES:
   The valid SMLib loop structure is:
   - There is one outer loop, and it is first in the list.
   - The outer loop goes counterclockwise (relative to the normal
     that is calculated in this routine).
   - All other loops go clockwise.
   - All loops are closed.
   - There are no intersections or self-intersections.

LIMITATIONS ---
   This does not check for more than one nesting level.
   If an inner loop is nested inside another inner loop,
   its orientation should be reversed: alternating inward.
   That check could be added if desired.

METHOD ---
  1. For Every Edge-
    1a. Check for degenerate edges - return error if found
    1b. Accumulate Edge's bounding box into sFaceExtent3
    1c. Place Edges Start and End points into a vertex list, sVertexPoints
  While any edges are not yet in a loop-
  2. Place Curves into loops by matching start/end points
    2a. Find an edge not yet used in a loop
    2b. while working on the current loop
      2b1. Add curve to loop arrays: sOrients, s3DCurves
      2b2. Add 3 distinct curve points to the aLoopPoints array
           moving around the loop, matching curve begins with curve ends.
         note: aLoopPoints array will contain a closed sequence of
            pionts as [pt0, pt1, pt1, pt2, pt2 ... ptn, ptn, pt0]
      2b3. Find next curve in loop by matching curve start/end points
    2c. When a loop closes -
        record number of curves in this loop
    2d. Let PlaneBase      += Sum of Loop Points
    2e. Let loopPlaneNormal = Sum_around_loop(CrossProduct(PTi,PTi+1)
    2f. Let PlaneNormal   = avg(PlaneNormal, loopPlaneNormal)
  3 get plane parmaeters
    3a. Let PlanePoint  = avg(All Loop Points)
    3b. Let PlaneNormal = unitize(avg of all Loop Normals)
  4. Test that every curve is on the plane to within 1000*given Tolerance

***********************************************************************/
SmStatus SmTrimmingTools::OrderCurvesIntoLoops
  (const SmTArray<SmCurve *>     & crCurves,         // in : target curves to order
   double                          d3DTol,           // in : maximum length for a degenerate curve and
                                                     //      lower bound on maximum allowed curve/curve gap size
                                                     //      max dist from plane
   SmTArray<SmCurve *>           & rsOrderedCurves,  // out: Curves reordered into loop sequences (not copied) 
   SmTArray<ULONG>               & rsLoopCounts,     // out: number of output loops       = rsLoopCounts.GetSize()   
                                                     //      number of curves in ith loop = rsLoopCounts[i], 
   SmTArray<SmOrientType>        & rsOrients,        // out: a SM_OT_SAME/SM_OT_OPPOSITE value for each OrderedCurve
   SmPoint3d                     & rsPlanePoint,     // out: Pt on output plane containing all input curves                   
   SmVector3d                    & rsPlaneNormal,    // out: Normal to output plane containing all input curves                  
   SmExtent3d                    & rsCurveBBox)      // out: Bounding box containing all input curves                  
{
  // init outputs
  rsOrderedCurves.ReSet();
  rsOrients.ReSet();                                                       
  rsLoopCounts.ReSet();                                                    
  rsPlanePoint.Set( 0,0,0 );                                               
  rsPlaneNormal.Set( 0,0,0 );                                              
                                                                           
  // Locals                                                                
  ULONG ii, jj;                                                            
  ULONG nCurves = crCurves.GetSize();                                      
  SmTArray<SmPoint3d> sVertexPoints;  // curve EndPoint list to be ordered and matched in pairs
  SmTArray<SmBoolean> aSeenFlags;     // Array of flags for curve seen

  // For each curve:
  // - init aSeenFlags.
  // - check for degenerate.
  //   - Note: SER if so.  Maybe just skip?
  // - collect start and end points into sVertexPoints.
  // - accumulate bounding boxes in rsCurveBBox.

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE;
  if (bDebugMe) 
    {
      // draw input curves
      smgfx_Erase() ;
      smgfx_ChangeColor(FALSE) ; 
      for(ii=0;ii<crCurves.GetSize();ii++)
        { SmCurve *pCurve = crCurves[ii] ;
          if(pCurve) { SmExtent1d sIvl = pCurve->GetNaturalInterval() ;
                       SmVector3d sStartPt, sEndPt ;
                       pCurve->EvaluatePoint( sIvl.GetMin(), sStartPt) ;
                       pCurve->EvaluatePoint( sIvl.GetMax(), sEndPt) ;
                       smgfx_SetLook(2,3) ; smgfx_ChangeColor() ; pCurve->Draw() ; sm_GraphicsLoop() ;
                       smgfx_SetLook(3,4, 1,0,0) ; sStartPt.Draw() ; sm_GraphicsLoop() ;
                       smgfx_SetLook(5,6, 0,0,1) ; sEndPt.Draw() ; sm_GraphicsLoop() ;
                     }
        } 
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // for every input curve - build sVertexPoints list and rsCurveBBox
  for(ii=0;ii<nCurves;ii++)
    {
      const SmCurve* pC = crCurves[ii];
      aSeenFlags.Add( FALSE );

      if (pC == NULL)  // RCLxx then modified by GWC from 'break' to 'continue'
        { continue; }

      // locals
      SmExtent1d sIV( pC->GetNaturalInterval() );
      SmPoint3d sStartPt, sEndPt;
      
      // Err: Degenerate Curve - tight tol, 3-pt check
      //      tol = 1000 * SM_EFF_ZERO * (1 + MaxDimension(StartPoint))
      // Was 1000 times - changed to one to fix issue when object is miles away from origin - st
      // Also changed SER to SE - should not fail when one curve is very small
      if (pC->IsDegenerate()) 
        { SE(SM_ERR); }
     
      // curve start/end points
      SER(pC->EvaluatePoint( sIV.GetMin(), sStartPt ));
      SER(pC->EvaluatePoint( sIV.GetMax(), sEndPt ));

      // when endPoints are within tol
      if (sStartPt.DistanceBetween(sEndPt) < 2.0 * d3DTol) 
        {
          // Err: Degenerate Curve - for given tol, 10-pt check
          double dDist = pC->ApproximateLength(sIV,5);
          if (dDist < 2.0 * d3DTol) 
            { SE(SM_ERR); } // RCLxx SER
        }

      // build sVertexPoints list
      sVertexPoints.Add(sStartPt);
      sVertexPoints.Add(sEndPt);

      // accumulate bounding box for all curves
      SmExtent3d sCurveExtent3;
      SER( pC->CalculateBoundingBox( sIV, &sCurveExtent3 ));
      if (ii==0) { rsCurveBBox = sCurveExtent3; }
      else       { rsCurveBBox.Union(sCurveExtent3, rsCurveBBox); }

    } // end iter every input curve - building sVertexPoints list and rsCurveBBox

  // curve bounding box size scale
  SmVector3d sFaceCenter; 
  double dFaceRadius = 0.0;
  rsCurveBBox.ComputeSphereBound(sFaceCenter, dFaceRadius);
  double dFeatureSize = dFaceRadius/1000.0;

  // locals
  ULONG nPlaneBaseComponentCount = 0;
  SmStatus sRet = SM_SUCCESS;

  // start-next-loop while block
  while( TRUE ) 
    {
      // locals
      const SmCurve* pC = NULL;
      ULONG nCurveIndex = 0;
      SmTArray<SmPoint3d> aLoopPoints; // contains [BegPt1 MidPt1 MidPt1 EndPt1, BegPt2 MidPt2 ...] for all curves in loop
                                       //  representing a tail-head segment sequence that makes a rough polygon of the actual loop

      // find a curve and its curve index not yet used in a loop
      for(ii=0; ii<nCurves; ii++) 
        {
          SmCurve* pCurrC = crCurves[ii];
          if(aSeenFlags[ii] == TRUE)
            { continue ; }

          pC = pCurrC;
          nCurveIndex = ii;
          break;
        } // end iter every curve - looking for curve not yet used in a loop

      // exit condition: all curves seen
      if(pC == NULL)
        { break ; }
      
      // arrive here when pC = first curve of a new loop:
      SmOrientType eCurveOrient = SM_OT_SAME;
      ULONG        nLoopEdges   = 0;
      
      // build-this-loop while block
      while( TRUE ) 
        {
          // Registering the current curve:
          rsOrients.Add(eCurveOrient);
          rsOrderedCurves.Add( (SmCurve*)pC );
          nLoopEdges++;

          // mark this curve as seen
          aSeenFlags.SetAt(nCurveIndex, TRUE);
          
          // Retrieving pC's start/end points:
          const SmPoint3d& sStartPt = sVertexPoints[2*nCurveIndex];
          const SmPoint3d& sEndPt   = sVertexPoints[2*nCurveIndex+1];
          
          // curve endPt gap
          SmPoint3d  sPt1, sPt2, sPt3;
          SmExtent1d sIvl( pC->GetNaturalInterval()); 
          double     dEndPtGap        = sStartPt.DistanceBetween(sEndPt); 
          SmBoolean  bSingleCurveLoop = (dEndPtGap < d3DTol);

          // first loop curve is not closed
          if(    nCurves == 1 
             && !bSingleCurveLoop ) 
            {
              // Tentatively set return error
              sRet = SM_ERR;
            }
          
          // Get 3 distinct points for the plane computation
          if (bSingleCurveLoop) 
            {
              sPt1 = sStartPt;
              SER(pC->EvaluatePoint(sIvl.Evaluate(0.3), sPt2));
              SER(pC->EvaluatePoint(sIvl.Evaluate(0.6), sPt3));
            } // end single curve loop branch
          else // multi-curve loop branch
            { 
              // Normally the vertices suffice, but in case the loop
              // consists of 2 vertices, 1 mid point is needed.
              sPt1 = sStartPt;
              SER(pC->EvaluatePoint(sIvl.Evaluate(0.5), sPt2));
              sPt3 = sEndPt;
            } // end multi-curve loop branch

          // Registering the 3 loop points in the correct order
          // add internal points twice
          if (eCurveOrient == SM_OT_SAME) 
            {
              aLoopPoints.Add(sPt1);
              aLoopPoints.Add(sPt2);
              aLoopPoints.Add(sPt2);
              aLoopPoints.Add(sPt3);

              // add closing points for closed edges
              if (bSingleCurveLoop) {
                 aLoopPoints.Add(sPt3);// add internal points twice
                 aLoopPoints.Add(sPt1);
              }

            } // end SAME curve orientation branch
          else // OPPOSITE curve orienation branch
            { 
              aLoopPoints.Add(sPt3);
              aLoopPoints.Add(sPt2);
              aLoopPoints.Add(sPt2);
              aLoopPoints.Add(sPt1);

              // add closing points for closed edges
              if (bSingleCurveLoop)  {
                 aLoopPoints.Add(sPt1); // add internal points twice
                 aLoopPoints.Add(sPt3);
              }
          } // end OPPOSITE curve orientation branch

          // locals
          SmBoolean bFound = FALSE; 
          double    dDist, d2ndDist; 
          ULONG     nFoundIndex;

          // find unique matching curve-end point within tol.
          //    More than 1 matching point is returned as an error
          SER(SmTools::FindFirstPointWithinTolToPnt
                 (sVertexPoints,
                  (eCurveOrient == SM_OT_SAME) ? 2*nCurveIndex+1 : 2*nCurveIndex,
                  smos_Max(d3DTol,dFeatureSize),
                  bFound,           // out: set to TRUE when exactly one point is within tol of target point
                  nFoundIndex,      // out: sVertexPoints found index 
                  dDist,            // out: dist between target and found neighbor
                  d2ndDist));       // out: dist between target and next nearest neighbor

          // abnormal exit: from build-this-loop block: open loop (no matches) or compound loop (multiple matches)
          if( !bFound ) 
            { sRet = SM_ERR ; 
              // back into the start-next-loop block
              break ;                 
            }                       
          
          // Matching Curve index and seen status
          ULONG     nNextCurveIndex =   (nFoundIndex % 2 == 0) 
                                      ?  nFoundIndex / 2 
                                      : (nFoundIndex-1) / 2;
          SmBoolean bNextCurveSeen  = aSeenFlags[nNextCurveIndex];

          // expected exit: from build-this-loop block: closed loop
          if (bNextCurveSeen) 
            {
              // back into the start-next-loop block
              break;
          }

          // Preparing for next iter
          pC           = crCurves[nNextCurveIndex];
          eCurveOrient =   (nFoundIndex % 2 == 0) 
                         ? SM_OT_SAME 
                         : SM_OT_OPPOSITE;
          nCurveIndex  = nNextCurveIndex;
        
        } // end build-this-loop while block

      // arrive here when 
      // rsOrderedCurves = extended to contain all curves for this loop
      // rsOrients       = set for all curves in this loop
      // nLoopEdges      = number of curves in this loop
      // aSeenFlags      = entries for all curves in this loop set to TRUE
      // aLoopPoints     = ordered list of [startPt midPt midPt endPt] for each curve in this loop
      
      // set output
      rsLoopCounts.Add(nLoopEdges);

      // Ccompute thisLoop's plane normal and base pt
      //     base pt      = avg(loop points)
      //     plane normal = unitized(Sum_around_loop(cross(pti,pti+1))
      // note: sRet = SM_ERR for open and spine loops

      // for every unique current-loop point - gather loop point avg and Sum(cross(veci,veci+1)
      SmVector3d sLoopPlaneNormal(0, 0, 0);
      for (jj=0; jj<aLoopPoints.GetSize(); jj+=2) 
        {
          SmPoint3d sPt1(aLoopPoints[jj]);
          SmPoint3d sPt2(aLoopPoints[jj+1]);

          // add up paired-point cross products - 
          //   When aLoopPoints is a regularly sampled set of points from
          //   a closed loop on a plane, the following sum will be the normal
          //   to that plane.  If the loop is open but planar, then the vector
          //   will not be the normal to that plane. In that sense this is another
          //   test for a closed loop.
          sLoopPlaneNormal.Set(sLoopPlaneNormal.x +(sPt1.y - sPt2.y ) *(sPt1.z + sPt2.z),
                               sLoopPlaneNormal.y +(sPt1.z - sPt2.z ) *(sPt1.x + sPt2.x),
                               sLoopPlaneNormal.z +(sPt1.x - sPt2.x ) *(sPt1.y + sPt2.y));

          // GWC: we could avoid adding and subtracting potentially large terms
          //      into the sum with the following equivalent computation
          //    - but it changes 3 multiplies and 9 adds 
          //      into the marginally more costly 6 multiplies and 6 adds.
      //    sLoopPlaneNormal.Set(
      //        sLoopPlaneNormal.x +(sPt1.y * sPt2.z - sPt2.y * sPt1.z),
      //        sLoopPlaneNormal.y +(sPt1.z * sPt2.x - sPt2.z * sPt1.x),
      //        sLoopPlaneNormal.z +(sPt1.x * sPt2.y - sPt2.x * sPt1.y) );

          // add up all loop point positions
          rsPlanePoint += sPt1;
          nPlaneBaseComponentCount++;
        } // end iter every point in current-loop building avg Pt and sum(Cross(vec1,vec2))

      SER(sLoopPlaneNormal.Unitize());

      // Update the plane data for all loops:
      if (rsLoopCounts.GetSize() == 1) 
        {
          rsPlaneNormal = sLoopPlaneNormal;
        }
      else 
        {
          double dDiscr = rsPlaneNormal.Dot( sLoopPlaneNormal );
          if (dDiscr < 0.0) { rsPlaneNormal = 0.5 * (rsPlaneNormal - sLoopPlaneNormal); }
          else              { rsPlaneNormal = 0.5 * (rsPlaneNormal + sLoopPlaneNormal); }
        }

    } // end start-next-loop while block

  // let the planeBase = Avg(All Plane Points)
  rsPlanePoint = rsPlanePoint / (double)nPlaneBaseComponentCount;

  // Now we can create the plane from normal, basepoint, extents
  SER( rsPlaneNormal.Unitize() );

  double dMaxMax = 0.0;

  // test every curve to see if it's on the plane
  for (ii=0; ii<nCurves; ii++)
    {
      SmCurve* pC = crCurves[ii];
      SmBoolean bOnPlane;
      double dMaxDistToSurf;

      // Is curve on plane test - 
      //   planar curves will not be on the computed plane when
      //   the planar curves do not form a closed loop.
      SER(pC->IsOnPlane( rsPlanePoint, 
                         rsPlaneNormal, 
                         d3DTol*1000.0, 
                         bOnPlane, 
                         dMaxDistToSurf));
      if (dMaxDistToSurf > dMaxMax) dMaxMax = dMaxDistToSurf;
      if (!bOnPlane) return SM_ERR;

    } // end iter every curve

  // all done
  return sRet;

} // end SmTrimmingTools::OrderCurvesIntoLoops

/*******************************************************************//**
PURPOSE: Given a set of curves loops -- formatted as the output from
            OrderCurvesIntoLoop() -- find inner and outer loops.

NOTES:
   The input is in the form of the output from OrderCurvesIntoLoops():
     SmTArray<SmCurve*>     & rsInCurves : list of all curve pointers,
       partitioned into separate loops, and in consecutive order within each loop.
     SmTArray<ULONG>        & rsInLoopCounts : in=number of Curves in each loop
     SmTArray<SmOrientType> & rsOrients : One entry for each curve in
       rsOrderedCurves, indicating its relative orientation within its loop.
       Each loop may be CW or CCW as a whole; this routine will orient
       them correctly: outer loops will be CCW (as a whole), and inner CW.

   Output:
     rsOrients : might have the individual loops flipped, such that
       outer loops are counterclockwise and inner loops are clockwise.
     rsOuterLoopCounts : just as rsLoopStartIndices contains an index
       into rsOrderedCurves of the start curve of each loop, this array
       contains an index into rsOrderedCurves of the start curve of each
       outer loop.  When there are no inner loops, this array is identical
       to rsLoopStartIndices.

   This routine does not deal with nested loops.  If a loop is contained
   within another inner loop, it will simply be listed as an inner loop
   within the outermost loop.  Such a configuration cannot be used to
   create an SMLib face; the 'island' would have to be a separate face.
   (It would also complicate this routine's output considerably.)

   If you might have nested loops, you can deal with them using this routine.
   After the call, take each list of inner loops, and call this routine
   again on those -- each list of loops contained within a single outer loop.
   If that call indicates that there are any inner loops, then the inner
   loops are multiply nested within the outer loop.  If there are no inner
   loops, that case is indicated by rsOuterLoopCounts having the same
   number of entries as rsLoopStartIndices.

METHOD ---
  while not done
    find the largest loop of all unclassified loops
      add it to the output arrays: curves and loop indices
      make sure its curves go CCW
      add its index to the output array of outer loop indices
      mark it as classified
    find all unclassified loops that are contained in that largest loop
      add them to the output arrays: curves and loop indices
      make sure its curves go CW
      mark them as classified
***********************************************************************/
SmStatus SmTrimmingTools::FindOuterLoops
  (SmTArray< SmCurve* >     & rsInCurves,         // i/o: consecutive Loop CrvSets. (CrvSet = Curves of one loop ordered 1st to last)
                                                  //      in = Loops not yet ordered
                                                  //      out= [Sequence of[OuterLoop CCW CrvSet followed by its InnerLoop CW CrvSets]]
                                                  //      sized:[NumCrvs]
   SmTArray< ULONG >        & rsInLoopCounts,     // i/o: Num of Crvs in each Loop, sized:[NumLoops]
   SmTArray< SmOrientType > & rsInOrients,        // i/o: assoc orientation of each Crv in its Loop, sized:[NumCrvs]
   SmVector3d               & rsPlaneNormal,      // in : Normal to plane containing curves
   double                     d3DTol,             // in : min dist between distinct points  
   SmTArray< ULONG >        & rsOuterLoopCounts)  // out: Num of Loops in each outerLoop, sized:[NumOuterLoops]
                                                  //      ex: rsOuterLoopCounts[i]=3 => 2 innerLoops inside OuterLoop[i]
{
  // init outputs
  rsOuterLoopCounts.ReSet();

  if(rsInLoopCounts.GetSize() < 1 ) { return SM_SUCCESS; }

#ifdef SM_DEBUG_CODE
SmBoolean bDebug = FALSE;
  if(bDebug)
    {
      ULONG di, dj, lCnt ;

      rsInLoopCounts.Dump() ;

      for(di=0,lCnt=0;di<rsInLoopCounts.GetSize();di++)
        { for(dj=0;dj<rsInLoopCounts[di];dj++,lCnt++)
           { smgfx_ChangeColor(dj!=0); if(rsInCurves[lCnt]) rsInCurves[lCnt]->DrawParams(); sm_GraphicsLoop(); }
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // Locals
  ULONG ii, jj, lLoopIndx ;
  SmContext sContext ;  // local construction
  double    dArea;
  ULONG     lNumCurves = rsInCurves.GetSize();
  ULONG     lNumLoops  = rsInLoopCounts.GetSize();

  // NOTE: Indexing gets complicated because there are two sets of indices,
  //       one for the unordered input loops and curves, and 
  //       a second for the ordered output loops and curves.
  //       When reading this code - understand when working with in or out indices.

  // Build up output arrays in locals, then move to the outputs when done.
  SmTArray<SmCurve*>             sOutCurves(lNumCurves ); // ordered LoopCrvSets [Outer 0 [Inner 0 1 2 ...] Outer 1 [Inner 0 1 ...] ... ]
  SmTArray<ULONG>                sOutLoops (lNumLoops  ); // sOutLoop[i] = Output Loop[i+1] FirstCrv index in sOutCurves
  SmTArray<SmOrientType>         sOutOrients( lNumCurves ); // assoc CrvOrients for each curve in sOutCurves
  SmTArray<ULONG>                sOutOuterLoops;            // maybe index to 1stCurve in Loop following this outerloop, sized:[NumOuterLoops]
  SmTArray<ULONG>                sInLoops  (lNumLoops );  // sInLoops[i] = Input loop[i] FirstCrv index in array rsInCurves
                                 
  SmTArray<double>               sLoopAreas   (lNumLoops, NULL, lNumLoops ); // sLoopAreas[i]   = input loop[i] area
  SmTArray<SmOrientType>         sLoopOrients (lNumLoops, NULL, lNumLoops ); // sLoopOrients[i] = input loop[i] orient (SAME = CCW; OPP = CW)
  SmTArray<SmPointSequence*>     sPtSeqs      (lNumLoops, NULL, lNumLoops ); // sPtSeqs[i]      = input loop[i] Ordered tessellated PtSequence 
  SmObjsDelete<SmPointSequence*> cleanUpPtSets(&sPtSeqs);
  SmTArray<SmPoint3d>            sCrvTessPts, sLoopTessPts;  // temp work space to accumulate Loop tessellation points

  // set sInLoops[i], so Input Loop[i] crvs run from rsInCurves[sInLoops[i], rsInLoops[i+1]]
  for(lLoopIndx=0,ii=0;ii<lNumLoops;ii++)
    {
      sInLoops.Add( lLoopIndx );
      lLoopIndx += rsInLoopCounts[ii];
    }
  SM_ASSERT( lLoopIndx == lNumCurves );
  SM_ASSERT( sInLoops.GetSize() == lNumLoops );

  // next: Tessellate each loop, create a SmPointSequence for it, and get its area and orientation.

  // Tessellate parameters
  double dCH      = 0.0;
  double dAng     = 15.0;
  ULONG  lMinSegs = 2;    // was 0: can result in 2 pts total, if only 2 curves.  [B343]

  ULONG lThisLoopStart = 0;
  ULONG lNextLoopStart = 0;

  // for every loop - build LoopData Arrays: sLoopAreas, sLoopOrients, sPtSeqs 
  for(ii=0;ii<lNumLoops;ii++)
    {
      sLoopTessPts.ReSet();
      lThisLoopStart  = lNextLoopStart;      SM_ASSERT(lThisLoopStart == sInLoops[ii]) ;
      lNextLoopStart += rsInLoopCounts[ii];  SM_ASSERT(ii == lNumLoops-1 || lNextLoopStart == sInLoops[ii+1]) ;

      // for every Loop->Curve - tessellate
      for(jj=lThisLoopStart;jj<lNextLoopStart;jj++)
        {
          SmCurve * pCrv = rsInCurves[jj];

          // tessellate the curve
          pCrv->Tessellate( pCrv->GetNaturalInterval(), dCH, dAng, lMinSegs, NULL, &sCrvTessPts );

          // ensure tessellate pts run in proper direction for loop
          if(rsInOrients[jj] == SM_OT_OPPOSITE )
            { sCrvTessPts.ReverseArray( 0, sCrvTessPts.GetSize() ); }

          // accumulate all loop tessellation points
          sLoopTessPts.Append( sCrvTessPts );

        } // end iter jj, every Loop building Loop data arrays: sLoopAreas, sLoopOrients, sPtSeqs

      // copy Loop tessellation points into SmPointSequence
      sPtSeqs[ii] = new (sContext) SmPointSequence
       (sLoopTessPts,   // in : array copied or converted into PointSequence                                          
          d3DTol,         // in : min dist between distinct points                                    
          TRUE,           // in : TRUE = implied linear connection between 1st and last point         
          TRUE,           // in : TRUE = PointSequence is known to lie on a plane                          
          &rsPlaneNormal, // in : Normal to plane when point sequence is known to be planar, NULL to ignore 
          TRUE );         // in : TRUE=Copy sPoints into a new array - sPoints and this m_pData arrays are different
                          //      FALSE=Share sPoints with new array - sPoints and this m_pData arrays are same
                          //      default:[TRUE] 
      
      // compute Area enclosed by the closed set of ordered points 
      dArea = sPtSeqs[ii]->GetArea();

      // save InitloopOrientation (SM_OT_SAME == CCW, SM_OT_OPPOSITE=CW), and LoopArea
      sLoopOrients[ii] = (dArea > 0 ) ? SM_OT_SAME : SM_OT_OPPOSITE;
      sLoopAreas[ii]   = smos_Fabs( dArea );

    } // end iter ii, every loop - - saving Crv tessellations, Loop init orientations, and Loop areas

  SmBoolean bDone = FALSE;

  // while loops remain to be classified
  while(!bDone)
    {
      double dMaxArea    = -1.0 ;
      ULONG  lLargestIndx = 0;

      // Find largest unprocessed loop - it's the next OuterLoop
      for(ii=0;ii<lNumLoops;ii++)
        {
          // Skip processed loops
          if(sLoopAreas[ii] <= 0.0) 
            { continue; }

          // save largest Area Loop input index
          if(sLoopAreas[ii] > dMaxArea )
            {
              dMaxArea = sLoopAreas[ii];
              lLargestIndx = ii;
            }
        } // end iter ii, looking for largest area loop

      // exit case - no unprocessed loops
      if(dMaxArea < 0.0 ) { bDone = TRUE;
                              break;
                            }
      
      // arrive here - remaining LargestLoop[lLargestIndx] is an OuterLoop - set its output values

      // get largest loop's start and end Curve indices
      lThisLoopStart = sInLoops[lLargestIndx] ;
      lNextLoopStart = (lLargestIndx+1 >= lNumLoops ) ? lNumCurves : sInLoops[ lLargestIndx+1 ];
      
      // remember to flip CW outer loops (outerloops are stored CCW)
      SmBoolean bFlip = (SM_OT_OPPOSITE == sLoopOrients[lLargestIndx]) ;

      // for every LargestLoop Curve
      for(ii=lThisLoopStart;ii<lNextLoopStart;ii++)
        {
          // get start Index to LargestLoop next curve (when flipping - reverse curve order)
          ULONG lIndex = (bFlip ) ? (lNextLoopStart-1) - (ii-lThisLoopStart)   // lIndex = from [lNextLoopStart-1] to [lThisLoopStart]
                                   : ii ;                                       // lIndex = from [lThisLoopStart] to [lNextLoopStart-1]
                                    
          // get this Curve's proper orientation for this OuterLoop
          SmOrientType eThisOrient = bFlip ? (rsInOrients[lIndex] == SM_OT_SAME ? SM_OT_OPPOSITE : SM_OT_SAME)
                                           : (rsInOrients[lIndex] ) ;

          // add ordered LargestLoop curves to sOutArray and sOutOrients
          sOutCurves.Add( rsInCurves [lIndex] );
          sOutOrients.Add( eThisOrient );

        } // end iter ii, every LargestLoop Crv

      // set sOutLoop[i] = sOutCurves index to start curve for Loop[i+1]  (GWC:how odd)
      sOutLoops.Add( sOutCurves.GetSize() );

      // Mark Loop as processed (set its area to -1)
      sLoopAreas[lLargestIndx] *= -1.0;

      // arrive here when an OuterLoop was found and added to output arrays 
      //    sOutCurves  =  ordered list of OuterLoop curves
      //    sOutOrients =  associated list of Curve orientations
      //    sOutLoops   =  sOutCurves index for NextLoopStart curve 
      //                   So, ith outLoop crvs run from sOutCurves[sOutLoops[i-1], sOutLoops[i]]

      // next - find all loops contained in this OuterLoop.
      //         Assume Curves of each loop DO NOT intersect the curves of any other.
      //         So, every tessellation point of one loop will classify in or out of any other loop.
      //         So, classify one tessellation point against OuterLoop to classify the whole loop.
      //         In practice: find two point classificzations that are the same.
      
      int iInCount = 0;
      SmPointObjectContainmentType eContain;

      // for every inloop - classify Loop against current OuterLoop
      for(ii=0;ii<lNumLoops;ii++)
        {
          // skip current LargestLoop itself and skip processed loops
          if(ii == lLargestIndx  ) { continue; } 
          if(sLoopAreas[ii] < 0 ) { continue; } 

          // locals
          iInCount = 0;
          SmPointSequence & rThisPtSeq = *(sPtSeqs[ii]); 
          ULONG             lNumPts    = rThisPtSeq.GetSize();

          // for every current inloop tessellation point - classify Point against LargestLoop
          for(jj=0;jj<lNumPts;jj++)
            {
              // classify CurrentLoop point against LargestLoop
              eContain = sPtSeqs[lLargestIndx]->PointContainment( rThisPtSeq[jj] ); 

              // Count consistent point classifications: increment (SM_POC_INSIDE) or decrement (SM_POC_OUTSIDE) iInCount as needed
              iInCount +=   (eContain == SM_POC_INSIDE)  ?  1 
                          : (eContain == SM_POC_OUTSIDE) ? -1 
                          : 0 ;

              // when 2 points classify the same - quit iterations
              if(iInCount >= 2 || iInCount <= -2 )
                { break; }

            } // end iter jj - until two point current inloop points classify the same against LargestLoop

          // classify the loop from the point classifications - allow for small point sets
          SmBoolean bContained =    (iInCount >= 2)
                                 || (iInCount > 0 && lNumPts < 3) ;

          // NotContained loops are new OuterLoops - skip processing them as an InnerLoop
          if(! bContained )
            { continue; }

          // arrive here when Current Input Loop is an InnerLoop of the current LargestLoop

          // next add CurrentLoop data to output as an InnerLoop

          // get currentLoop start and end InputArray indices
          lThisLoopStart = sInLoops[ii];
          lNextLoopStart = (ii+1 >= lNumLoops ) ? lNumCurves : sInLoops[ii+1];

          // Inner loops are output with 'Opposite' CW orientation
          bFlip = (sLoopOrients[ ii ] == SM_OT_SAME );

          // for every CurrentLoop Curve
          for(jj=lThisLoopStart;jj<lNextLoopStart;jj++)
            {
              // If we're flipping the orientation of this loop,
              // we also have to reverse the order of curves/orients in the loop.
              ULONG lIndex = (bFlip ) ? (lNextLoopStart-1) - (jj-lThisLoopStart)  // iter curves from [lNextLoopStart-1] to [lThisLoopStart]
                                       : jj ;                                      //             from [lThisLoopStart] to [lNextLoopStart-1]

              // get currentCurve orientation for this loop
              SmOrientType eThisOrient = bFlip ? (rsInOrients[lIndex] == SM_OT_SAME ? SM_OT_OPPOSITE : SM_OT_SAME)
                                               : (rsInOrients[lIndex] ) ;

              // Add current Curve data to output arrays: sOutCurves and sOutOrients
              sOutCurves.Add( rsInCurves [lIndex] );
              sOutOrients.Add( eThisOrient );

            } // end iter jj, every CurrentLoop Curve

          // Set sOutLoop[i] = NextLoop StartIndex in array sOutCurves
          sOutLoops.Add( sOutCurves.GetSize() );

          // mark current loop as processed (set Area to -1)
          sLoopAreas[ii] *= -1.0;
        
        } // end while loops remain to be classified as innerLoops or current LargestLoop 

      // set sOutOuterLoops[i] = NextOuterLoop StartIndex in sOutLoops array
      sOutOuterLoops.Add( sOutLoops.GetSize() );

    } // end while loops remain to be classified as OuterLoops

  // set output

  // all curves list
  rsInCurves  = sOutCurves;  // rearranged so that 1. OuterLoop CrvSet is followed by its InnerLoop CrvSets
                             //                    2. OuterLoop CrvSets run CCW and InnerLoop CrvSets run CW
  // all curve orients list
  rsInOrients = sOutOrients; // assoc CrvOrient for every curve in its loop

  // all loops list - translated back from sOutLoops[i] = StartIndex of Loop[i+1] to curve counts per loop
  rsInLoopCounts[0] = sOutLoops[0];
  for(ii=1;ii<lNumLoops;ii++) { rsInLoopCounts[ii] = sOutLoops[ii] - sOutLoops[ii-1]; }

  // OuterLoops list - translated back from sOutOuterLoops[i] = StartIndex of OuterLoop[i+1] to Loop counts per OuterLoop
  ULONG lNumOuterLoops = sOutOuterLoops.GetSize();
  if( lNumOuterLoops > 0 )         { rsOuterLoopCounts.Add( sOutOuterLoops[0] ); }
  for(ii=1;ii<lNumOuterLoops;ii++) { rsOuterLoopCounts.Add( sOutOuterLoops[ii] - sOutOuterLoops[ii-1] ) ; }

#ifdef SM_DEBUG_CODE
  if(bDebug)
    {
      ULONG di, dj, lCnt ;

      rsInLoopCounts.Dump() ;
      rsOuterLoopCounts.Dump() ;

      for(di=0,lCnt=0;di<rsInLoopCounts.GetSize();di++)
        { for(dj=0;dj<rsInLoopCounts[di];dj++,lCnt++)
           { smgfx_ChangeColor(dj!=0); if(rsInCurves[lCnt]) rsInCurves[lCnt]->DrawParams(); sm_GraphicsLoop(); }
        }
      sm_GraphicsLoop();
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmTrimmingTools::FindOuterLoops

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sTrimmingCheckLoopForMiniHourglass_list[] =
/* lOwnerLabelIndex == 0 */
{
  /*  0 */ {SM_AT_GEOMETRIC, _T("Small Closed Edge"), _T("Signal as a problem any closed edge whose length is less than tol") }
} ;

/*******************************************************************//**
PURPOSE: Checks to see if the loop is closed in 3D space

NOTES: GWC: This is an odd check
    An error is returned if the loop has a closed edge whose length
    is less than .001.  I don't see the problem for which this test is checking.

    However, the function is never called - so it doesn't really matter.
***********************************************************************/
SmStatus SmTrimmingTools::CheckLoopForMiniHourglass
 (SmLoop const     * pLoop,         // in : 
  SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
{
  SM_REF1(pTestRequests);

  // loop edges
  SmTArray<SmEdge*> sEdges;
  pLoop->GetEdges (sEdges);
  ULONG ii, lNumEdges = sEdges.GetSize();

  // when loop has edges
  if (lNumEdges > 1)
    {
      double shortest_length = 10000000.0;

      // for every edge
      for (ii = 0; ii < lNumEdges; ii++)
        {
          SmVertex *start_vertex = sEdges [ii]->GetVertex ();
          SmVertex *end_vertex   = sEdges [ii]->GetOtherVertex (start_vertex);

          // when edge is closed
          if (start_vertex == end_vertex)
            {
              SmCurve *curve  = sEdges [ii]->GetCurve ();
              double   length = curve->ApproximateLength(sEdges[ii]->GetInterval(), 11);
              // save shorted closed edge seen
              if (shortest_length > length)
                  shortest_length = length;

              // if closed edge is too short
              if (length < .001)
                {
                  SM_ASSERT_OBJ_VALUE_REPORT(pLoop, SM_LIST_0, 0, SM_LEVEL_0, (length < .001), pLoop, .001, length, _T("") ) ;
                  SER (SM_ERR);
                }
            } // end edge is closed check
        } // end iter every edge
    } // end loop has edges check
  return SM_SUCCESS;

} // end SmTrimmingTools::CheckLoopForMiniHourglass

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sTrimmingCheckTessellation_list[] =
/* lOwnerLabelIndex == 1 */
{
  /*  0 */ {SM_AT_GEOMETRIC, _T("tessellation"), _T("This face could not be tessellated by NLib ") }
} ;


/*******************************************************************//**
PURPOSE: Test if a face is made properly for tessellation by tessellating it.

NOTES: The face is tested by calling the NLib face tessellation
 routine.  returns TRUE when that tessellation succeeds, else returns false.
***********************************************************************/

// This is an unnessary use of N_TessTrimmedSrf. It can be eliminated if we choose to eliminate N_TessTrimmedSrf

SmStatus SmTrimmingTools::CheckTessellation
 (SmFace const     * pFace,         // in : target face
  SmAssertArray    * pAList,        // NotUsed: i/o: Accumulating list of failed Asserts, NULL to ignore
  SmAssertTestLevel  eTestLevel,    // NotUsed: in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmTArray<ULONG>  * pTestRequests) // NotUsed: in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
{                                 
  SM_REF3(pAList, eTestLevel, pTestRequests) ;
  // locals
  NL_CURVE ***cuo;
  NL_CURVE ****cui;
  NL_SURFACE *sur;
  NL_INDEX m = 0;
  NL_INDEX ho[1];
  NL_INDEX hi[1];
  NL_INDEX **hs;
//    NL_REAL epc = 3.0e-4;
//    NL_REAL eps = 1.0e-3;
//    NL_REAL tol = SM_ZONE_TOL_3D/10.0;
  NL_REAL epc = 3.0e-3;              // epsilon curve
  NL_REAL eps = 1.0e-2;              // epsilon surface
  NL_REAL tol = SM_ZONE_TOL_3D/10.0; // 3d tolerance

  // face locals
  SmExtent3d           sBBox;
  SmTArray<SmLoopuse*> sLoopuses;
  SmSurface           *pSurface = pFace->GetSurface();
  SmFaceuse            *pFU      = pFace->GetUpwardFaceuse();
  SER(pSurface->CalculateBoundingBox(pFace->GetUVDomain(),&sBBox));
  pFU->GetLoopuses(sLoopuses);

  // let SizeScale = face->Surface 3d size
  SmVector3d sSize = sBBox.GetSize();

  // Scale tolerances to SizeScale
  eps *= sSize.Length();
  epc *= sSize.Length();

  /* start NURBS (initializes memory STACKS) */
  NL_STACKS S;

  // init NLib memory structures when possible
  hs     = N_AllocInt2dArray(0, sLoopuses.GetSize()-2, &S );
  hi[0]  = sLoopuses.GetSize() - 2;
  cui    = N_AllocArrayTripleCrvPtrs(m, &S );   if (cui == NULL) SER(SM_ERR);
  cui[0] = N_AllocArrayRealCrvPtrs(hi[0], &S ); if (cui[0] == NULL) SER(SM_ERR);
  cuo    = N_AllocArrayRealCrvPtrs(m, &S );

  SmTArray<NL_CURVE*>          sUVCurves;
  SmTArray<ULONG>           sLoops;
  SmTArray<SmBSplineCurve*> sUVCurves2;
  SmObjsDelete<SmBSplineCurve*> sCleanup(&sUVCurves2);

  // get copies of all face->Edgeuse boundary curves 
  // (UVtrimCurves or 3dCurves) in loop traversal order,
  // CCW for 1st outerLoop and CW for all subsequent innerLoops.
  SER(pFace->CreateOrientedTrimBoundary(*pFace->GetContext(),
                                        TRUE,                 // in : TRUE = get UVTrimCurves, FALSE = get 3dCurves
                                        TRUE,                 // in : TRUE = make extra UVTrimCurves to jump SurfaceSingularities, FALSE = don't
                                        sLoops,               // out: number of loops and number of edges in each loop
                                        sUVCurves2));         // out: ordered:[loop1_edges (counter-clockwise), ... loopnedges (clockwise)]
                                                              //      malloc:[curves to be freed by caller] 
  //for ever loop
  ULONG lEdgeIndex = 0;
  ULONG ii, jj, kk, lNumLoops = sLoops.GetSize();
  for (ii=0; ii<lNumLoops; ii++)
    {
      // array of all curves in the iith loop
      sUVCurves.ReSet();

      // for every edge in the iith loop - add curve to sUVCurves
      for (kk=0; kk<sLoops[ii]; kk++)
        {
          SmBSplineCurve *pBSC = sUVCurves2[lEdgeIndex++];
          SER(pBSC->RemoveExtraKnots(SM_ZONE_TOL_3D/10.0));
          sUVCurves.Add(pBSC->GetGwNurbPointer());
        }
      ULONG lNumCurves = sUVCurves.GetSize();

      // for the outer loop
      if (ii==0)
        {
          // build ho and cuo "outer loop" input arguments
          ho[0]  = lNumCurves - 1;
          cuo[0] = N_AllocArrayCrvPtrs(ho[0], &S);
          if (cuo[0] == NULL) SER(SM_ERR);
          for (jj=0; jj<lNumCurves; jj++)
            {
              cuo[0][jj] = sUVCurves[jj];
            }
        } // end outer loop branch
      else // inner loop
        {
          // build hs and cui "inner loop" input arguments
          hs [0][ii-1] = lNumCurves - 1;
          cui[0][ii-1] = N_AllocArrayCrvPtrs(hs[0][ii-1], &S);
          if (cui[0][ii-1] == NULL) { SER(SM_ERR); }
          for (jj=0; jj<lNumCurves; jj++)
            {
              NL_CURVE * cur = sUVCurves[jj];
              cui[0][ii-1][jj] = cur;
            }
        } // end inner loop branch
    } // end iter every loop

  // copy the face->Surface
  SmBSplineSurface *pBSS = new (pFace->GetContext()) SmBSplineSurface(*(SmBSplineSurface*)pFace->GetSurface());
  SmObjDelete sSurfCleanup(pBSS);

  // locals for NLIb call
  sur = pBSS->GetGwNurbPointer();
  NL_PARAMETER *u, *v;
  NL_INDEX n;
  NL_INDEX **DT, *hd;

  // Ask NLib to tessellate the surface
  if (N_TessTrimmedSrf(sur,cuo,cui,m,ho,hi,hs,epc,eps,tol,
      &u,      // out: u Parameters corresponding to the vertices of triangles
      &v,      // out: v Parameters corresponding to the vertices of triangles
      &n,      // out: Highest index in(u, v) 
      &DT,     // out: Point lists of triangulation, i.e. DT[i] points to
               //      a list of indexes representing  points surrounding
               //      the point(u[i], v[i]). The indexes  are listed in
               //      COUNTERCLOCKWISE  order. If  DT[i][0]  is  -1, the
               //      point(u[i], v[i]) is out of the domain.
      &hd,     // out: hd[i] is the highest index of array DT[i]
      &S ) == NL_YES)
    {
      // SmBoolean bRtn = SM_ASSERT_OBJ_VALUE_REPORT(this, SM_LIST_1, 0, SM_LEVEL_0, (FALSE), tol, _T("") ) ;
      SER (SM_ERR);
    }

  // all done - success
  return SM_SUCCESS;

} // end SmTrimmingTools::CheckTessellation

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sTrimmingCheckFace_list[] =
/* lOwnerLabelIndex == 2 */
{
  /*  0 */ {SM_AT_POINTER,   _T("Loop"),             _T("Face has no Loops") },
  /*  1 */ {SM_AT_GEOMETRIC, _T("UVTrimCurve"),      _T("Info only: Face has edges without UVTrimCurves") },
  /*  2 */ {SM_AT_GEOMETRIC, _T("Loop Orientation"), _T("unable to compute loop orientation for Face Loop") },
  /*  3 */ {SM_AT_GEOMETRIC, _T("Loop Orientation"), _T("Face Loop has bad orientation") },
  /*  4 */ {SM_AT_GEOMETRIC, _T("Loop Orientation"), _T("Face Loop has zero area") }
} ;

// #define CHECK_TESSELLATION

/*******************************************************************//**
PURPOSE: Signal Errors when
  1. Face is missing loops
  2. Face->loop has a SurfaceGap exceeding d3DTolerance
        SurfaceGap = Surface(Loop->edge->EndUPoint) - Surface(Loop->PreviousEdge->EndUPoint)

NOTES: Side Effects:
   1. Build UVTrimCurves when they are missing
***********************************************************************/
SmStatus SmTrimmingTools::CheckFace                                      
 (SmFace const     * pFace,         // in : target face
  double             d3DTolerance,  // in : Given Max SurfaceGap Size
  SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmTArray<ULONG>  * pTestRequests, // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
  SmBoolean          bSkipCheckLoop)// in : TRUE = skip CheckLoop (Loop::AssertValid will run it)
{
  SM_REF1(pTestRequests);

#ifdef CHECK_TESSELLATION
    // This is an unnessary use of N_TessTrimmedSrf. It can be eliminated if we choose to eliminate N_TessTrimmedSrf
    // Have NLib tessellate the face - If it can, the loop structure and shape of the UVTrimCurves are in good shape.
  SER (CheckTessellation(pFace, pAList, eTestLevel));
#endif

#ifdef SM_DEBUG_CODE
static int iDebugTiming=0;
static double stTotalTime = 0.0;
static double stTotalTime_1 = 0.0; // for checking CheckLoop()
static double stTotalTime_2 = 0.0; // for checking ComputeLoopOrientation()
static int siEntries = 0;
static int siExits   = 0;  // to check for returns in the middle.
clock_t tStart = 0, tStart_1 = 0, tStart_2 = 0;
  if(iDebugTiming > 0 )
  {
    siEntries++;
    tStart = clock();
  }
#endif // SM_DEBUG_CODE

  // accumulative error
  SmBoolean bOK = TRUE ;

// Remove Composite
//  // skip checks for SmCFace types
//  if(pFace->IsKindOf(SmCFace_TYPE))
//    { return( SM_SUCCESS ) ; }

  // get face->Loops
  SmTArray<SmLoop *> sLoops;
  pFace->GetLoops (sLoops);

  // error - missing loops
  if (sLoops.GetSize () == 0)
    {
      bOK &= SM_ASSERT_OBJ_BOOLEAN_REPORT(pFace, SM_LIST_2, 0, SM_LEVEL_0, (sLoops.GetSize () > 0), pFace, _T("") ) ;
      SER (SM_ERR); // Loops are missing
    }

  ULONG ii;

  // When UVCurves are missing - take a moment so that in debug mode the objects can be drawn and dumped
  SmBoolean bRtn = AreUVCurvesPresent( pFace ) ;
  if(!bRtn)
    {
      // note when a face->Edge is missing a UVTrimCurve
      // no need for the warning - this is not an illegal case or uncommon case
      // SM_DBG_WARN( _T("SmTrimmingTools::CheckFace(): UVTrimCurves are missing"));
      // bOK &= SM_ASSERT_OBJ_BOOLEAN_REPORT(SM_LIST_2, 1, SM_LEVEL_0, bRtn, pFace, _T("") ) ;
      
      // This is just a Check routine, often called only in debug mode.
      // Do not change the database here by adding missing UVTrimCurves.

#ifdef SM_DEBUG_CODE  
SmBoolean bDebug = FALSE;
      // draw faces with sCrvClass1 classification objects
      if(bDebug)
        {
          SmBrep    *pBrep    = pFace->GetBrep() ;
          SmSurface *pSurface = pFace->GetSurface() ;
          SmLoopuse *pLoopuse, *pOther_loopuse;
          SmTArray<SmEdgeuse*> sEdgeuses;


          pFace->Dump();

          smgfx_Erase() ;
          smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) { pBrep->Draw(TRUE) ; } sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 0,1,1) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH, 6, 6) ; sm_GraphicsLoop() ;
          smgfx_SetLook(1,2, 1,1,0) ; if(pSurface) pSurface->DrawUV() ; sm_GraphicsLoop() ;
          ULONG lNumLoops = sLoops.GetSize();
          for (ii = 0; ii < lNumLoops; ii++)
            {
              SmLoop *pLoop = sLoops[ii] ;
              pLoop->GetLoopuses (pLoopuse, pOther_loopuse);
              pLoopuse->GetEdgeuses (sEdgeuses);

              ULONG lNumEUs = sEdgeuses.GetSize();
              for (ULONG jj = 0; jj < lNumEUs; jj++)
                {
                  SmEdgeuse *pEdgeuse = sEdgeuses[jj] ;
                  SmEdge    *pEdge    = pEdgeuse->GetEdge() ;
                  if (pEdgeuse->GetUVTrimCurve () == 0)
                    {
                      smgfx_SetLook(3,5, 1,0,0) ; pEdgeuse->Draw() ; sm_GraphicsLoop() ;
                      pEdge->Dump() ;
                    }
                  else // This edge has a loopuse
                    {
                      smgfx_SetLook(3,5, 0,1,0) ; pEdgeuse->Draw() ; sm_GraphicsLoop() ;
                    }
                } // end every Loop->Edge
            } // end every face->Loop
          sm_GraphicsLoop() ;
        }
#endif // SM_DEBUG_CODE

      // it's not a problem when an edge does not have a stored UVTrimCurve - that's allowed.
      return SM_SUCCESS;

      // SER (CreateUVTrimCurves(pFace));
    }

  // for every loop
  ULONG lClosure;
  ULONG lNumLoops = sLoops.GetSize();
  for(ii=0; ii < lNumLoops; ii++)
    {
      // Check for face->Loop SurfaceGaps exceeding d3DTolerance

#ifdef SM_DEBUG_CODE
  if(iDebugTiming > 0 )
    { tStart_1 = clock(); }
#endif

      SmStatus sRtn = SM_SUCCESS;
      if(!bSkipCheckLoop)
        {
          sRtn = CheckLoop(pFace->GetSurface(), sLoops[ii], d3DTolerance, lClosure, pAList, eTestLevel);
          SER_MSG(sRtn, _T("SmTrimmingTools::CheckFace - Failed CheckLoop cascading message")) ;
        }

#ifdef SM_DEBUG_CODE
  if(iDebugTiming > 1 ) {
    clock_t tFinish = clock();
    double tDuration = (tFinish-tStart_1) / 1000.0;
    stTotalTime_1 += tDuration;

    tStart_2 = clock(); // Start the next one.
  }
#endif

      // Check the loop orientation results.
      SmOrientType loop_orientation;
      SmStatus     eStat = pFace->ComputeLoopOrientation( sLoops[ii], loop_orientation );

#ifdef SM_DEBUG_CODE
  if(iDebugTiming > 0 ) {
    clock_t tFinish = clock();
    double tDuration = (tFinish-tStart_2) / 1000.0;
    stTotalTime_2 += tDuration;
  }
#endif

      bOK &= SM_ASSERT_OBJ_VALUE_REPORT(pFace, SM_LIST_2, 2, SM_LEVEL_0, (eStat == SM_SUCCESS), pFace, d3DTolerance, SM_UNDEF_DOUBLE, _T("") ) ;
      SER_MSG( eStat, _T("SmTrimmingTools::CheckFace(): unable to compute loop orientation"));

      bOK &= SM_ASSERT_OBJ_BOOLEAN_REPORT(pFace, SM_LIST_2, 3, SM_LEVEL_0, 
                                         (  ((ii == 0) && loop_orientation == SM_OT_SAME)        
                                           || ((ii != 0) && loop_orientation == SM_OT_OPPOSITE)),
                                           pFace, 
                                           _T("") ) ;
      
      // signal bad loop errors: we want 1st Loop Outer, rest Inner
      if(   ((ii == 0) && loop_orientation == SM_OT_OPPOSITE)
         || ((ii != 0) && loop_orientation == SM_OT_SAME))
        {
          // This will probably fix it up ok
          //SmBoolean  bLoopsReordered = FALSE;
          //FixTrimLoopOrder((SmFace *)pFace, bLoopsReordered);
          // but in a check we only report
          // The caller can fix if desired.
          SER_MSG( SM_ERR, _T("SmTrimmingTools::CheckFace() found loop with wrong orientation"));
        }

      bOK &= SM_ASSERT_OBJ_BOOLEAN_REPORT(pFace, SM_LIST_2, 4, SM_LEVEL_0,(loop_orientation != SM_OT_UNKNOWN ), pFace, _T("") ) ;

      // Also check for zero-area loop.
      if( loop_orientation == SM_OT_UNKNOWN )
        {
          SER_MSG( SM_ERR, _T("SmTrimmingTools::CheckFace() found loop with zero area"));
        }

    } // end iter every loop

#ifdef SM_DEBUG_CODE
  if(iDebugTiming > 0 ) {
    clock_t tFinish = clock();
    double tDuration = (tFinish-tStart) / 1000.0;
    stTotalTime += tDuration;
    siExits++;
    if(iDebugTiming > 1 ) {
      TCHAR sBuff[SM_TBLOCK_SIZE];
      smos_sprintf(sBuff, _T("  SmTrimmingTools::CheckFace() Entries, Exits, Time, Total:  %5d  %5d    %lf   %lf\n"),
              siEntries, siExits, tDuration, stTotalTime );
      smos_WriteBuffer( sBuff );
      smos_sprintf(sBuff, _T("  SmTrimmingTools::CheckFace()  total time in CheckLoop() and ComputeLoopOrientation(): %lf  %lf\n"),
              stTotalTime_1, stTotalTime_2 );
      smos_WriteBuffer( sBuff );
    }
  }
#endif

  return SM_SUCCESS;

} // end SmTrimmingTools::CheckFace

/*******************************************************************//**
PURPOSE: Call CheckFace on all given faces

NOTES: No checks here - just in the CheckFace method
***********************************************************************/
SmStatus SmTrimmingTools::CheckFaces
 (SmTArray<SmFace *> const  & crpFaces,      // in : list of target faces
  double                      d3DTolerance,  // in : Tolerance used to check loop gaps
  SmAssertArray             * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore
  SmAssertTestLevel           eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                             //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                             //      default:[SM_LEVEL_0] 
  SmTArray<ULONG>           * pTestRequests) // NotUsed: in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
{
  SM_REF1(pTestRequests) ;
  // pass the call along for every given face
  ULONG ii, lNumFaces = crpFaces.GetSize();
  for (ii = 0; ii < lNumFaces; ii++)
    {
      SER (CheckFace (crpFaces [ii], d3DTolerance, pAList, eTestLevel));
    }

  // all done
  return SM_SUCCESS;

} // end SmTrimmingTools::CheckFaces

/*******************************************************************//**
PURPOSE: Bound a UV endpoint gap by its B-spline subpatch or isoparametric curve.

NOTES: Positive-weight B-splines lie in their control-point convex hull.
       Requiring that hull in both endpoint tolerance balls is conservative;
       failure to establish the bound leaves the original UV check in force.
***********************************************************************/
static SmBoolean sm_IsLoopUVGapWithinTolerance
 (const SmSurface & crSurface,       // in : target loop surface
  const SmPoint2d & crStartUV,       // in : preceding edgeuse's ending UV
  const SmPoint2d & crEndUV,         // in : current edgeuse's starting UV
  const SmPoint3d & crStartPoint,    // in : surface point at crStartUV
  const SmPoint3d & crEndPoint,      // in : surface point at crEndUV
  SmXSectTol3d      sXSectTol3d)     // in : intersection tolerance of the adjoining edges
{
  const SmBSplineSurface * pSurface = dynamic_cast<const SmBSplineSurface *>(&crSurface);
  if(pSurface == NULL)
    { return FALSE; }

  SmExtent2d sGapDomain;
  sGapDomain.AddPoint2d(crStartUV);
  sGapDomain.AddPoint2d(crEndUV);
  // Only certify gaps inside the surface's natural domain.
  const SmExtent2d sNaturalDomain = pSurface->GetNaturalUVDomain();
  if(   sGapDomain.GetMin().x < sNaturalDomain.GetMin().x
     || sGapDomain.GetMin().y < sNaturalDomain.GetMin().y
     || sGapDomain.GetMax().x > sNaturalDomain.GetMax().x
     || sGapDomain.GetMax().y > sNaturalDomain.GetMax().y)
    { return FALSE; }

  SmTArray<SmPoint3d> sPoints;
  SmTArray<double> sWeights;
  if(sGapDomain.XLength() == 0.0 || sGapDomain.YLength() == 0.0)
    {
      // A zero-width rectangle is an isoparametric curve, not a surface patch.
      const SmSurfParamType eConstantDir = sGapDomain.XLength() == 0.0 ? SM_SP_U : SM_SP_V;
      const double dConstantParam = eConstantDir == SM_SP_U ? crStartUV.x : crStartUV.y;
      const SmExtent1d sGapInterval = eConstantDir == SM_SP_U ? sGapDomain.GetVInterval() : sGapDomain.GetUInterval();
      if(sGapInterval.GetLength() <= 0.0)
        { return FALSE; }
      SmBSplineCurve * pCurve = NULL;
      // Use the exact NURBS extraction, including for analytic subclasses.
      const SmStatus eStatus = pSurface->SmBSplineSurface::CreateIsoParametricCurve(
          *pSurface->GetContext(), // in : context for the temporary curve
          eConstantDir,           // in : constant surface parameter direction
          dConstantParam,         // in : constant surface parameter value
          SmApproxTol3d(0.0),     // in : unused by exact NURBS extraction
          pCurve                  // out: exact B-spline isoparametric curve
      );
      SmObjDelete sDeleteCurve(pCurve);
      if(eStatus != SM_SUCCESS || pCurve == NULL)
        { return FALSE; }
      SmExtent1d sTrimInterval = sGapInterval;
      if(pCurve->Trim(sTrimInterval) != SM_SUCCESS)
        { return FALSE; }
      // Trimming can snap to knots; the curve must still cover the whole gap.
      const SmExtent1d sCurveInterval = pCurve->GetNaturalInterval();
      if(   sCurveInterval.GetMin() > sGapInterval.GetMin()
         || sCurveInterval.GetMax() < sGapInterval.GetMax()
         || pCurve->GetControlPolygon(sPoints, sWeights) != SM_SUCCESS)
        { return FALSE; }
    }
  else
    {
      SmBSplineSurface * pPatch = NULL;
      const SmStatus eStatus = pSurface->CopySubPatch(
          *pSurface->GetContext(), // in : context for the temporary subpatch
          sGapDomain,              // in : UV rectangle between the endpoints
          pPatch                   // out: exact B-spline subpatch
      );
      SmObjDelete sDeletePatch(pPatch);
      if(eStatus != SM_SUCCESS || pPatch == NULL)
        { return FALSE; }

      // CopySubPatch can snap to knots. A shrunken domain cannot certify this gap.
      const SmExtent2d sPatchDomain = pPatch->GetNaturalUVDomain();
      if(   sPatchDomain.GetMin().x > sGapDomain.GetMin().x
         || sPatchDomain.GetMin().y > sGapDomain.GetMin().y
         || sPatchDomain.GetMax().x < sGapDomain.GetMax().x
         || sPatchDomain.GetMax().y < sGapDomain.GetMax().y)
        { return FALSE; }

      ULONG lNumU, lNumV;
      if(pPatch->GetControlPointNet(lNumU, lNumV, sPoints, sWeights) != SM_SUCCESS)
        { return FALSE; }
    }
  if(sPoints.GetSize() == 0)
    { return FALSE; }
  // Empty weights mean a nonrational curve or patch, with implicit unit weights.
  const SmBoolean bRational = sWeights.GetSize() != 0;
  for(ULONG lPoint = 0; lPoint < sPoints.GetSize(); ++lPoint)
    {
      if(   (bRational && !(sWeights[lPoint] > 0.0))
         || !(sPoints[lPoint].DistanceBetween(crStartPoint) <= sXSectTol3d)
         || !(sPoints[lPoint].DistanceBetween(crEndPoint) <= sXSectTol3d))
        { return FALSE; }
    }
  return TRUE;
}

/*******************************************************************//**
PURPOSE: AssertValid() Test reporting static labels
***********************************************************************/
SmAssertReportLabel sTrimmingCheckLoop_list[] =
/* lOwnerLabelIndex == 3 */
{
  /*  0 */ {SM_AT_GEOMETRIC, _T("Edge/Edge Gap"),     _T("Gap between edge endPoints exceeds tolerance") },
  /*  1 */ {SM_AT_DOMAIN,    _T("UVEdge/UVEdge Gap"), _T("too large U Dir gap between loop Edge endPoints that are not on a singularity") },
  /*  2 */ {SM_AT_DOMAIN,    _T("UVEdge/UVEdge Gap"), _T("too large V Dir gap between loop Edge endPoints that are not on a singularity") }
} ;

/*******************************************************************//**
PURPOSE:  signal errors when gap between any connecting
             pair of loop->edges exceeds d3DTolerance.

             gap = Surface(loop->edge->StartUVPoint) - Surface(loop->prevEdge->EndUVPoint)

          Large nonperiodic UV gaps may close within the edge intersection
          tolerance when their entire intervening B-spline subpatch (or
          isoparametric curve) is bounded in both lifted endpoint neighborhoods.
          Periodic gaps remain errors except for the existing singularity exemption.

NOTES: values of the closure argument, rlClosure:
  0: The uv-curves form a closed loop.
  1: The uv-curves are closed except at seams or singularities.
  2: The uv-curves are not closed, even with singularities.
  3: The loop is not closed in 3d.
Cases 0 and 1 are valid SMLib topology, 2 and 3 are not.

***********************************************************************/
SmStatus SmTrimmingTools::CheckLoop
 (SmSurface const  * pSurface,      // in : target loop->Face->Surface
  SmLoop const     * pLoop,         // in : target loop
  double             d3DTolerance,  // in : max distance allowed between loop edges
  ULONG            & rlClosure,     // out: 0: The uv-curves form a closed loop.                     
                                    //      1: The uv-curves are closed except at singularities.     
                                    //      2: The uv-curves are not closed, even with singularities.
                                    //      3: The loop is not closed in 3d.                         
  SmAssertArray    * pAList,        // i/o: Accumulating list of failed Asserts, NULL to ignore
  SmAssertTestLevel  eTestLevel,    // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                    //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                    //      default:[SM_LEVEL_0] 
  SmTArray<ULONG>  * pTestRequests) // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
{
  SM_REF1(pTestRequests);

  // init closure arg to Good until we find out differently.
  rlClosure = 0;
  SmBoolean bOK = TRUE ;

  // get Loop->Loopuses
  SmLoopuse *loopuse;
  SmLoopuse *other_loopuse;
  pLoop->GetLoopuses(loopuse, other_loopuse);

  // get Loopuse->edgeuses
  SmTArray<SmEdgeuse*> sEdgeuses;
  loopuse->GetEdgeuses (sEdgeuses);

  // I don't think we need to worry about the loop reverse flags here,
  // because the loop is required to be closed (in 3d space) either way.

  // for every edgeuse
  ULONG ii, lNumEUs = sEdgeuses.GetSize();
  for (ii=0; ii< lNumEUs; ii++)
    {
      // get thisEdgeuse UVPoints from the edge's UVTrimCurve
      SmPoint2d current_start, current_end;
      SER(GetEdgeUVExtremes(sEdgeuses[ii],
                            current_start,
                            current_end));

      // get previous loop->edgeuse UVPoints  from the edge's UVTrimCurve
      ULONG previous =   (ii == 0)
                       ? lNumEUs - 1
                       : ii - 1 ;
      SmPoint2d previous_start, previous_end;
      SER(GetEdgeUVExtremes(sEdgeuses[previous],
                            previous_start,
                            previous_end));

      // When UVPoints vary by more than 1e-10
      if (previous_end.DistanceBetween(current_start) > 1e-10)
        {
          // Get SurfacePoints for each UVPoint
          SmPoint3d previous_end_3d, current_start_3d;
          SER(pSurface->EvaluatePoint(previous_end,
                                      previous_end_3d));
          SER(pSurface->EvaluatePoint(current_start,
                                      current_start_3d));

          // error - when SurfacePoints vary by more than d3DTolerance
          double dEndStartDist = previous_end_3d.DistanceBetween(current_start_3d) ;

          // check the Edge/Edge xsect distance
          //   gwc: this is a tight test - edges must be within XSectTol3d of Vertices, but two edges connected to one
          //        vertex could be positioned so that their EndPoint distance is greater than XSectTol3d(pEdge1,pEdge2)
          SmXSectTol3d sXSectTol3d = SmTol::GetXSectTol3d(sEdgeuses[ii]->GetEdge(), sEdgeuses[previous]->GetEdge()) ; 

          bOK &= SM_ASSERT_OBJ_VALUE_REPORT(pLoop, SM_LIST_3, 0, SM_LEVEL_0, (dEndStartDist <= sXSectTol3d), pLoop, sXSectTol3d, dEndStartDist, _T("") ) ;
          if(dEndStartDist > sXSectTol3d )
            {
              rlClosure = 3;

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
              if(bDebugMe)
                {
                   sEdgeuses.Dump() ;
                   SM_DUMP_AND_ASSERT_VALID(pLoop) ;

                   ULONG  di; 
                   double dCH = .1 ;
                   SmFace *pFace = (SmFace *)pSurface->GetFace() ;
                   SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;  
                   SmPoint3d previous_start_3d, current_end_3d;
                   SER(pSurface->EvaluatePoint(previous_start, previous_start_3d));
                   SER(pSurface->EvaluatePoint(current_end,    current_end_3d));

                   pSurface->Dump() ;

                   smgfx_Erase() ;
                   dCH = smgfx_SetLook(1,2, 0,0,1, .0001) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                   smgfx_SetLook(1,2, 0,1,0, .0001) ; pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
                   smgfx_SetLook(1,2, 0,0,0, .0001) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
                   smgfx_SetChordHeight(dCH) ;
                   smgfx_SetLook(6,7, 1,0,0) ; previous_end_3d.Draw() ; sm_GraphicsLoop() ;
                   smgfx_SetLook(8,9, 0,1,1) ; current_start_3d.Draw() ; sm_GraphicsLoop() ;

                   smgfx_SetLook(10,11, 0,0,1) ; previous_start_3d.Draw() ; sm_GraphicsLoop() ;
                   smgfx_SetLook(10,11, 0,0,1) ; current_end_3d.Draw() ; sm_GraphicsLoop() ;

                   smgfx_SetLineWidth(3) ; 
                   smgfx_ChangeColor() ; sEdgeuses[ii]->Draw(); sm_GraphicsLoop() ; 
                   smgfx_SetPointSize(14) ; sEdgeuses[ii]->GetVertexuse()->GetVertex()->GetPoint().Draw() ; sm_GraphicsLoop() ;
                   smgfx_SetLineWidth(5) ; if(sEdgeuses[ii]->GetEdge()) sEdgeuses[ii]->GetEdge()->DrawParams() ; sm_GraphicsLoop() ;
                   smgfx_ChangeColor() ; sEdgeuses[previous]->Draw(); sm_GraphicsLoop() ;
                   smgfx_SetPointSize(14) ; sEdgeuses[previous]->GetVertexuse()->GetVertex()->GetPoint().Draw() ; sm_GraphicsLoop() ;
                   smgfx_SetPointSize(14) ; sEdgeuses[previous]->GetMate()->GetVertexuse()->GetVertex()->GetPoint().Draw() ; sm_GraphicsLoop() ;
                   smgfx_SetLineWidth(5) ; if(sEdgeuses[previous]->GetEdge()) sEdgeuses[previous]->GetEdge()->DrawParams() ; sm_GraphicsLoop() ;
                   sm_GraphicsLoop() ;

                   // draw EdgeUVTrimCurve data. The gaps being tested are defined by the UVTrimCurves.
                   for(di=0; di<lNumEUs; di++)
                     {
                       smgfx_ChangeColor() ;

                       SmEdgeuse * pEdgeuse = sEdgeuses[di] ;
                       SmEdge    * pEdge    = pEdgeuse->GetEdge() ;

                       smgfx_SetLineWidth(3) ; pEdgeuse->Draw(); sm_GraphicsLoop() ;
                       smgfx_SetLineWidth(5) ; if(pEdge) pEdge->DrawParams() ; sm_GraphicsLoop() ;
                     }
                   sm_GraphicsLoop() ;
                }
#endif // SM_DEBUG_CODE
              TCHAR sBuff[SM_TBLOCK_SIZE] ;
              smos_sprintf(sBuff, _T("SmTrimmingTools::CheckLoop() - Gap/Tol [%16.16lf / %16.16lf] in Loop for edgeuse.\n"),
                                dEndStartDist, (double)sXSectTol3d /* , sEdgeuses[i] */ ) ;
              SE_MSG(SM_ERR,sBuff);  // was SER_MSG(SM_ERR,sBuff); prevented further tests.
            }

          // Large UV gaps can be physically small near a pole. Allow a bounded
          // nonperiodic subpatch within the edge intersection tolerance.
          // Keep the seam guard: even a full-period jump can fit inside that
          // neighborhood near a pole. A loop cannot jump a seam. [B431; B279]
          SmExtent2d sDomain = pSurface->GetNaturalUVDomain();
          SmSurfParamType eSingDir1, eSingDir2;
          pSurface->IsSingularity( previous_end,  eSingDir1, d3DTolerance ) ;
          pSurface->IsSingularity( current_start, eSingDir2, d3DTolerance ) ;
          const SmBoolean bLargeGapU = smos_Fabs(current_start.x - previous_end.x) > sDomain.XLength() * 0.10;
          const SmBoolean bLargeGapV = smos_Fabs(current_start.y - previous_end.y) > sDomain.YLength() * 0.10;
          const SmBoolean bSingularU = (eSingDir1 == SM_SP_U || eSingDir1 == SM_SP_BOTH) &&
                                      (eSingDir2 == SM_SP_U || eSingDir2 == SM_SP_BOTH);
          const SmBoolean bSingularV = (eSingDir1 == SM_SP_V || eSingDir1 == SM_SP_BOTH) &&
                                      (eSingDir2 == SM_SP_V || eSingDir2 == SM_SP_BOTH);
          SmBoolean bBoundedGap = FALSE;
          if(   dEndStartDist <= sXSectTol3d
             && ((bLargeGapU && !bSingularU) || (bLargeGapV && !bSingularV))
             && !(bLargeGapU && !bSingularU && pSurface->IsClosed(sDomain, SM_SP_U))
             && !(bLargeGapV && !bSingularV && pSurface->IsClosed(sDomain, SM_SP_V)))
            {
              bBoundedGap = sm_IsLoopUVGapWithinTolerance(
                  *pSurface,         // in : target loop surface
                  previous_end,      // in : preceding edgeuse's ending UV
                  current_start,     // in : current edgeuse's starting UV
                  previous_end_3d,   // in : surface point at crStartUV
                  current_start_3d,  // in : surface point at crEndUV
                  sXSectTol3d        // in : intersection tolerance of the adjoining edges
              );
            }
          bOK &= SM_ASSERT_OBJ_VALUE_REPORT(pLoop, SM_LIST_3, 1, SM_LEVEL_0,
              (!bLargeGapU || bSingularU || bBoundedGap), pLoop, sDomain.XLength() * 0.10,
              smos_Fabs(current_start.x - previous_end.x), _T(""));
          bOK &= SM_ASSERT_OBJ_VALUE_REPORT(pLoop, SM_LIST_3, 2, SM_LEVEL_0,
              (!bLargeGapV || bSingularV || bBoundedGap), pLoop, sDomain.YLength() * 0.10,
              smos_Fabs(current_start.y - previous_end.y), _T(""));
#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
          if(bOK == 0)
            {
              if(bDebugMe)
                {
                   sEdgeuses.Dump() ;

                   ULONG j ;
                   SmFace *pFace = (SmFace *)pSurface->GetFace() ;
                   SmBrep *pBrep = pFace ? pFace->GetBrep() : NULL ;
                   SmExtent2d sUVDomain = pFace->GetUVDomain();
                   pSurface->Dump() ;

                   smgfx_Erase() ;
                   smgfx_SetLook(1,2, 0,0,1) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;
                   smgfx_SetLook(1,2, 0,1,0) ; pSurface->DrawUV(8,8,FALSE,NULL,TRUE) ; sm_GraphicsLoop() ;
                   smgfx_SetLook(1,2, 0,1,0) ; pSurface->DrawParams() ; sm_GraphicsLoop() ;
                   smgfx_SetLook(1,2, 1,1,0) ; pSurface->DrawSeams() ; sm_GraphicsLoop() ;
                   smgfx_SetLook(1,2, 0,0,0) ; if(pFace) pFace->Draw(SM_DM_CROSSHATCH) ; sm_GraphicsLoop() ;
                   smgfx_SetLook(6,7, 1,0,0) ; previous_end_3d.Draw() ; sm_GraphicsLoop() ;
                   smgfx_SetLook(8,9, 0,1,1) ; current_start_3d.Draw() ; sm_GraphicsLoop() ;

                   smgfx_SetLook(1,2, .5,0,1) ; sUVDomain.Draw();
                   sm_GraphicsLoop() ;

                   // draw EdgeUVTrimCurve data. The gaps being tested are defined by the UVTrimCurves.
                   for(j=0; j<lNumEUs; j++)
                     {
                       smgfx_ChangeColor() ;

                       SmEdgeuse *      pEdgeuse = sEdgeuses[j] ;
                       SmEdge    *      pEdge    = pEdgeuse->GetEdge() ;
                       SmBSplineCurve* pUVTrim = pEdgeuse->GetUVTrimCurve();  
                        
                       smgfx_SetLineWidth(3+j) ; pEdgeuse->Draw(); sm_GraphicsLoop() ;
                       smgfx_SetLineWidth(5+j) ; if(pEdge) pEdge->Draw() ; sm_GraphicsLoop() ;
                       if (pUVTrim)
                       {
                           pUVTrim->Draw(NULL, TRUE);
                           sm_GraphicsLoop();
                       }
                       if (pUVTrim) pUVTrim -> Dump();
                       sm_GraphicsLoop();
                     }
                   sm_GraphicsLoop() ;
                }
            }
#endif // SM_DEBUG_CODE

          if((bLargeGapU || bLargeGapV) && !bBoundedGap)
            {
              rlClosure = 2;

              if(eSingDir1 != SM_SP_NEITHER || eSingDir2 != SM_SP_NEITHER )
                { rlClosure = 1; }

              if(rlClosure >= 2 )
                { SER_MSG (SM_ERR,_T("SmTrimmingTools::CheckLoop: UVSpace EdgeEnd/EdgeEnd gap > 10%% of sDomain Size")); }
            } // end uv points are not even close

        } // end UVpoints Vary by more than 1e-10
    } // end iter every loopuse->edgeuse

  return SM_SUCCESS;

} // end SmTrimmingTools::CheckLoop

/*******************************************************************//**
PURPOSE: Get Edgeuse->UVTrimCurve Natural EndPoint UVPositions and weights

NOTES: This returns the UV Start and End point positions of the Edgeuse.
  When the edgeuse::m_eOrientation == SM_OT_OPPOSITE,
    the Edgeuse runs in the opposite direction of its Edge.
  The UVTrimCurve always runs in the same direction of the Edge.
  When computing  UVEndPoints from a UVTrimCurve, the
    EndPoints have to be swapped when the edgeuse is SM_OT_OPPOSITE.
    When computing the UVEndPoints directly from Edgeuse->Vertexuse
    locations, the EndPoints never need to be swapped.
***********************************************************************/
SmStatus SmTrimmingTools::GetEdgeUVExtremes
  (SmEdgeuse const * pEdgeuse,         // in : target edgeuse
   SmPoint2d       & rStart,           // out: edgeuse->UVTrimCurve Natural Start
   double          & rdStartWeight,    // out: associated weight
   SmPoint2d       & rEnd,             // out: edgeuse->UVTrimCurve Natural Start
   double          & rdEndWeight)      // out: associated weight
{
  // get underlying edgeuse->UVTrimCurve
  SmBSplineCurve *pUVCurve = pEdgeuse->GetUVTrimCurve();
  if(pUVCurve == NULL )
    {
      // Don't try to create a uv curve, the topology might not be
      // in good shape (as during read-in).  [B513]
      SmPoint2d sUV1, sUV2;
      SmVertexuse *pVU1 = pEdgeuse->GetVertexuse();
      SER( pVU1->ComputeUVPoint( sUV1, FALSE ));  // False: don't use uv curve.
      SmVertexuse *pVU2 = pEdgeuse->GetMate()->GetVertexuse();
      SER( pVU2->ComputeUVPoint( sUV2, FALSE ));

      // return edgeuse start and end positions
      rStart = sUV1;
      rEnd   = sUV2;

      // old: mistaken swap for SM_OT_OPPOSITE Edgeuses.
      // if(pEdgeuse->GetOrientation() == SM_OT_SAME )
      //   {
      //     rStart = sUV1;
      //     rEnd   = sUV2;
      //   }
      // else
      //   {
      //     rEnd   = sUV1;
      //     rStart = sUV2;
      //   }

      return SM_SUCCESS;

    } // end no UVTrimCurve check

  // arrive here when edgeuse has a UVTrimCurve

  // get UVTrimCurve natural start/end positions and weights
  SmPoint3d start_3d, end_3d;
  pUVCurve->GetEnds(start_3d, end_3d,
                    &rdStartWeight, &rdEndWeight);

  // store 2d point data in 2d objects
  rStart = SmPoint2d(start_3d [0], start_3d [1]);
  rEnd   = SmPoint2d(end_3d [0], end_3d [1]);

  // swap ends when Orientation is reversed
  if(pEdgeuse->GetOrientation() == SM_OT_OPPOSITE)
    {
      SM_SWAP(SmPoint2d, rStart, rEnd);
      SM_SWAP(double, rdStartWeight, rdEndWeight);
    }

#ifdef SM_DEBUG_CODE
SmBoolean bDebugMe = FALSE ;
  if(bDebugMe)
    {
      SmEdgeuse   * pEdgeMate    = pEdgeuse->GetMate() ;
      SmLoopuse   * pLoopuse     = pEdgeuse->GetLoopuse() ;
      SmLoop      * pLoop        = pLoopuse ? pLoopuse->GetLoop() : NULL ; 
      SmFace      * pFace        = pEdgeuse->GetFace() ;
      SmSurface   * pSurface     = pFace ? pFace->GetSurface() : NULL ; 
      SmEdge      * pEdge        = pEdgeuse->GetEdge() ;
      SmCurve     * pUVTrimCurve = pEdgeuse->GetUVTrimCurve() ;
      SmBrep      * pBrep        = pEdge ? pEdge->GetBrep() : NULL ;

      SmVertexuse * pStartVU = pEdgeuse->GetVertexuse() ;
      SmVertexuse * pEndVU   = pEdgeuse->GetMate()->GetVertexuse() ;
      SmVertex    * pStartV  = pStartVU ? pStartVU->GetVertex() : NULL ;
      SmVertex    * pEndV    = pEndVU ? pEndVU->GetVertex() : NULL ;

      SmPoint2d sStartUV, sEndUV ;
      SmPoint3d sStartXYZ, sEndXYZ, sStartVertXYZ, sEndVertXYZ ;
      pStartVU->ComputeUVPoint(sStartUV, FALSE) ;
      pEndVU->ComputeUVPoint(sEndUV, FALSE) ;
      if(pSurface) { pSurface->EvaluatePoint(sStartUV, sStartXYZ) ;
                     pSurface->EvaluatePoint(sEndUV, sEndXYZ) ;
                   }
      if(pStartV)  { sStartVertXYZ = pStartV->GetPoint() ; }
      if(pEndV)    { sEndVertXYZ = pEndV->GetPoint() ; }
      SmCrvOnSurf sProjCurve(*pUVTrimCurve, *pSurface) ;

      SM_DUMP_AND_ASSERT_VALID(pLoop) ;
      SM_DUMP_AND_ASSERT_VALID(pEdge) ;
      SM_DUMP_AND_ASSERT_VALID(pEdgeuse) ; 

      double dCH = .1 ;

      smgfx_Erase() ;
      dCH = smgfx_SetLook(1,2, 0,0,1, .0001) ; if(pBrep) pBrep->Draw(TRUE) ; sm_GraphicsLoop() ;

      smgfx_SetLook(2,3, 0,1,0, .0001) ; pEdgeuse->Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(2,3, 0,1,1, .0001) ; if(pEdgeMate) pEdgeMate->Draw() ; sm_GraphicsLoop() ;

      smgfx_SetLook(3,4, 1,0,0) ;   sStartXYZ.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, 1,.1,.5) ; sStartVertXYZ.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(3,4, 0,0,1) ;   sEndXYZ.Draw() ; sm_GraphicsLoop() ;
      smgfx_SetLook(5,6, .5,.2,1) ; sEndVertXYZ.Draw() ; sm_GraphicsLoop() ;

      smgfx_SetLook(2,3, 0,0,0, .0001) ; if(pEdge) pEdge->DrawParams() ; sm_GraphicsLoop() ;
      smgfx_SetLook(4,5, 0,1,1, .0001) ; sProjCurve.DrawParams() ; sm_GraphicsLoop() ;

      smgfx_SetLook(1,2, 0,1,0, .0001) ; if(pSurface) pSurface->DrawUV(20, 20, FALSE, NULL, TRUE) ; 
      smgfx_SetChordHeight(dCH) ;
      sm_GraphicsLoop() ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmTrimmingTools::GetEdgeUVExtremes

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmTrimmingTools::GetEdgeUVExtremes
  (SmEdgeuse const *pEdgeuse,         // in : target edgeuse
   SmPoint2d &rStart,                 // out: edgeuse->StartUVPoint
   SmPoint2d &rEnd)                   // out: edgeuse->EndUVPoint
{
    double dDummy1, dDummy2;
    return GetEdgeUVExtremes( pEdgeuse, rStart, dDummy1, rEnd, dDummy2 );

} // end SmTrimmingTools::GetEdgeUVExtremes

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmTrimmingTools::SurfaceContinuityCheck
  (SmSurface const *pSurface,
   SmBoolean &rbSurfaceIsAtLeastG1)
{
    SmContinuityType continuity_u;
    SmContinuityType continuity_v;
    SmTArray<SmContinuityType> dummy;

#ifdef SM_VERSION_NUMBER
    // Newer versions have this function on SmSurface
    SER (pSurface->CalculateContinuities (SM_SP_U, continuity_u, dummy));
    SER (pSurface->CalculateContinuities (SM_SP_V, continuity_v, dummy));
#else
    // Older versions only have this function on SmBSplineSurface.
    // Luckily, in that case we only have bsplines to worry about.
    SM_ASSERT (pSurface->IsKindOf (SmBSplineSurface_TYPE));
    SmBSplineSurface *bspline = (SmBSplineSurface *) pSurface;
    SER (bspline->CalculateContinuities (SM_SP_U, continuity_u, dummy));
    SER (bspline->CalculateContinuities (SM_SP_V, continuity_v, dummy));
#endif

    if (continuity_u == SM_CT_DISCONTINUOUS
        || continuity_v == SM_CT_DISCONTINUOUS)
        SER (SM_ERR);

    rbSurfaceIsAtLeastG1 = !(continuity_u == SM_CT_C0 || continuity_v == SM_CT_C0);

    return SM_SUCCESS;

} // end SmTrimmingTools::SurfaceContinuityCheck

/*******************************************************************//**
PURPOSE:

NOTES: Private method; make it public if you need it.
***********************************************************************/
SmStatus SmTrimmingTools::MirrorCurve
 (SmBSplineCurve * curve,      // in :
  SmSurfParamType  side,       // in :
  double           low,        // in :
  double           high,       // in :
  double           tolerance)  // NotUsed: in :
{
  SM_REF1(tolerance) ;
    ULONG const coord = side == SM_SP_U ? 0 : 1;

    // Get the curve's control point info
    ULONG dimension;
    ULONG degree;
    SmTArray<SmPoint3d> control_points;
    SmBSplineCurveForm bspline_curve_form;
    SmTArray<ULONG> knot_multiplicities;
    SmTArray<double> knots;
    SmKnotType knot_type;
    SmTArray<double> weights;
    SER (curve->GetCanonical (dimension, degree,
                              control_points, bspline_curve_form,
                              knot_multiplicities, knots, knot_type, weights));

    ULONG ii, lNumPts = control_points.GetSize();
    for (ii = 0; ii < lNumPts; ii++)
    {
        control_points [ii][coord] = high - (control_points [ii][coord] - low);
        if (ii < weights.GetSize ())
            control_points [ii] = control_points [ii] / weights [ii];
    }

    SER (curve->SetCanonical (dimension, degree,
                              control_points, bspline_curve_form,
                              knot_multiplicities, knots, knot_type,
                              weights.GetSize () > 0 ? &weights : 0,
                              0));

    SER(curve->ConvertTo2D());

    return SM_SUCCESS;

} // end SmTrimmingTools::MirrorCurve

/*******************************************************************//**
PURPOSE: Test whether a given parameter-space curve is an iso-parameteric curve.

NOTES:
   reWhichDir is the constant direction:
    SM_SP_U means a constant-u curve, running in the v-direction,
    and rdParamValue is a u-parameter value.
***********************************************************************/
SmBoolean SmTrimmingTools::IsIsoCurve
 (const SmCurve   *cpUVCurve,   // in: uv curve to test
    const SmSurface *cpSurf,      // in: surface in which cpUVCurve is defined
    double           dTol3d,      // in: 3d tolerance
    SmSurfParamType &reWhichDir,  // out: SM_SP_U or _V
    double          &rdParamValue // out: constant u- or v- parameter
  )
{
  reWhichDir = SM_SP_UNKNOWN;
  rdParamValue = 0;

  SmExtent3d sTempBBox;
  cpUVCurve->CalculateBoundingBox( cpUVCurve->GetNaturalInterval(), &sTempBBox );
  SM_ASSERT( sTempBBox.GetWInterval().GetLength() < SM_EFF_ZERO );
  SmExtent2d sCurveBBox( sTempBBox.GetUInterval(), sTempBBox.GetVInterval() );
  SmExtent2d sSrfDomain = cpSurf->GetNaturalUVDomain();
  sCurveBBox.Intersect( sSrfDomain, sCurveBBox );

  SmExtent1d sCrvUIvl = sCurveBBox.GetUInterval();
  SmExtent1d sCrvVIvl = sCurveBBox.GetVInterval();
  SmExtent1d sSrfUIvl = sSrfDomain.GetUInterval();
  SmExtent1d sSrfVIvl = sSrfDomain.GetVInterval();

  if(sCrvUIvl.GetLength() < sSrfUIvl.GetLength() / 100 )
  {
      // Convert dTol3d into a uv tolerance.
      SmPoint2d sUV = sCurveBBox.GetMid();
      SmVector2d sUVDir( 1, 0 );
      SmTol2d sUTol = SmTol::MapTo2d( dTol3d, sUV, sUVDir, *cpSurf );
      if(sCrvUIvl.GetLength() < sUTol )
      {
          reWhichDir = SM_SP_U;

          double dScaledZero = SM_EFF_ZERO *(1.0 + sSrfUIvl.GetLength() );
          if(sCurveBBox.GetMin().x < sSrfDomain.GetMin().x + dScaledZero )
            { rdParamValue = sSrfDomain.GetMin().x; }
          else if(sCurveBBox.GetMax().x > sSrfDomain.GetMax().x - dScaledZero )
            { rdParamValue = sSrfDomain.GetMax().x; }
          else
            { rdParamValue = sCrvUIvl.GetMid(); }

          return TRUE;
      }
  }  // end if possible u-param isocurve

  if(sCrvVIvl.GetLength() < sSrfVIvl.GetLength() / 100 )
  {
      // Convert dTol3d into a uv tolerance.
      SmPoint2d sUV = sCurveBBox.GetMid();
      SmVector2d sUVDir( 0, 1 );
      SmTol2d sVTol = SmTol::MapTo2d( dTol3d, sUV, sUVDir, *cpSurf );
      if(sCrvVIvl.GetLength() < sVTol )
      {
          reWhichDir = SM_SP_V;

          double dScaledZero = SM_EFF_ZERO *(1.0 + sSrfVIvl.GetLength() );
          if(sCurveBBox.GetMin().y < sSrfDomain.GetMin().y + dScaledZero )
            { rdParamValue = sSrfDomain.GetMin().y; }
          else if(sCurveBBox.GetMax().y > sSrfDomain.GetMax().y - dScaledZero )
            { rdParamValue = sSrfDomain.GetMax().y; }
          else
            { rdParamValue = sCrvVIvl.GetMid(); }

          return TRUE;
      }
  }  // end if possible v-param isocurve

  reWhichDir = SM_SP_NEITHER;

  return FALSE;

} // end SmTrimmingTools::IsIsoCurve

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmBoolean SmTrimmingTools::IsIsoCurve
  (SmBSplineCurve const *cpUVCurve,
   SmSurfParamType eSide,
   double dLow,
   double d2DTolerance)
{
  ULONG dimension;
  ULONG degree;
  SmTArray<SmPoint3d> control_points;
  SmBSplineCurveForm bspline_curve_form;
  SmTArray<ULONG> knot_multiplicities;
  SmTArray<double> knots;
  SmKnotType knot_type;
  SmTArray<double> weights;
  SER (cpUVCurve->GetCanonical (dimension, degree,
                                control_points, bspline_curve_form,
                                knot_multiplicities, knots, knot_type, weights));

  ULONG const coord = eSide == SM_SP_U ? 0 : 1;
  ULONG ii, lNumCPts = control_points.GetSize();
  for (ii = 0; ii < lNumCPts; ii++)
    {
      if (fabs (control_points [ii][coord] - dLow) > d2DTolerance)
          return FALSE;
    }

  return TRUE;

} // end SmTrimmingTools::IsIsoCurve

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmBoolean SmTrimmingTools::IsIsoCurve
  (SmBSplineCurve const *cpUVCurve,
   SmSurfParamType &rSide,
   double &rdParameter)
{
  double const tolerance = SM_ZONE_TOL_3D/10.0;

  SmPoint3d start, end;
  cpUVCurve->GetEnds (start, end, 0, 0);

  ULONG ii;
  for (ii = 0; ii < 2; ii++)
    {
      if (fabs (start [ii] - end [ii]) < tolerance)
        {
          rSide = ii == 0 ? SM_SP_U : SM_SP_V;
          rdParameter = (start [ii] + end [ii]) / 2.0;
          if (IsIsoCurve (cpUVCurve, rSide, rdParameter, tolerance))
              return TRUE;
        }
    }

  return FALSE;

} // end SmTrimmingTools::IsIsoCurve

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmBoolean SmTrimmingTools::IsIsoCurveOnSeam
  (SmSurface const *cpSurface,
   SmBSplineCurve const *cpUVCurve,
   SmSurfParamType &rSide, double &rdParameter)
{
    if (!IsIsoCurve (cpUVCurve,
        rSide, rdParameter))
        return FALSE;

    ULONG index = rSide == SM_SP_U ? 0 : 1;
    SmExtent2d const bounds = cpSurface->GetNaturalUVDomain ();
    SmBoolean const closed = cpSurface->IsClosed (bounds, rSide) ? TRUE : FALSE;
    SmExtent1d const range (bounds.GetMin () [index], bounds.GetMax () [index]);
    double const tolerance = (range.GetLength ()) / 1000.0;

    return closed && (fabs (rdParameter - range.GetMin ()) < tolerance
                   || fabs (rdParameter - range.GetMax ()) < tolerance);

} // end SmTrimmingTools::IsIsoCurveOnSeam

/*******************************************************************//**
PURPOSE: Compute the orientation of a loop of trimming curves.

NOTES:
   The curves are assumed to be 2d (z == 0).
   Start and end indices are given, to allow selecting one loop
   out of a single array containing several loops.
***********************************************************************/
SmStatus SmTrimmingTools::ComputeLoopOrientation
  (SmTArray<SmBSplineCurve * > const & crpAllUVCurves, // in
   SmTArray<SmOrientType> const & creAllCurveSenses,   // in
   ULONG iIndexBegin, ULONG iIndexEnd,                 // in: indices of this loop in the input arrays
   SmOrientType &rOrientation)                         // out
{
    SmTArray<SmCurve*> curves;
    curves.SetSize (iIndexEnd - iIndexBegin);
    SmTArray<SmBoolean> senses;
    senses.SetSize (iIndexEnd - iIndexBegin);
    ULONG ii;
    for (ii = iIndexBegin; ii < iIndexEnd; ii++)
    {
        curves [ii - iIndexBegin] = (SmCurve *) crpAllUVCurves [ii];
        if (ii < creAllCurveSenses.GetSize ())
            senses [ii - iIndexBegin] = creAllCurveSenses [ii];
        else
            senses [ii - iIndexBegin] = SM_OT_SAME;
    }


    SmVector3d normal (0.0, 0.0, 1.0);  // assume 2d curves.
    SER (SmCompositeCurve::ComputeProjectedLoopOrientation (curves, senses,
                                                            normal,
                                                            rOrientation, 0));
    return SM_SUCCESS;

} // end SmTrimmingTools::ComputeLoopOrientation

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmTrimmingTools::ApproximateLengthThroughSurface
  (SmBSplineCurve const *cpUVCurve,
   SmSurface const *pSurface,
   ULONG iNumberSamples,
   double &rdApproximateLength)
{
    SM_ASSERT (iNumberSamples > 1);
    rdApproximateLength = 0.0;
    SmExtent1d interval = cpUVCurve->GetNaturalInterval ();
    double delta = 1.0 / (double) (iNumberSamples - 1);
    double t = 0.0;

    SmPoint3d point_2d;
    SER (cpUVCurve->EvaluatePoint (interval.Evaluate (t),
        point_2d));
    SmPoint3d last_point_3d;
    SER (pSurface->EvaluatePoint (SmPoint2d (point_2d [0], point_2d [1]),
        last_point_3d));

    t += delta;
    ULONG ii;
    for (ii = 1; ii < iNumberSamples; ii++)
    {
        SER (cpUVCurve->EvaluatePoint (interval.Evaluate (t),
            point_2d));
        SmPoint3d point_3d;
        SER (pSurface->EvaluatePoint (SmPoint2d (point_2d [0], point_2d [1]),
            point_3d));
        rdApproximateLength += last_point_3d.DistanceBetween (point_3d);

        t += delta;
        last_point_3d = point_3d;
    }

    return SM_SUCCESS;

} // end SmTrimmingTools::ApproximateLengthThroughSurface

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmTrimmingTools::IsCurveOnSurfaceDegenerate
  (SmBSplineCurve *cpUVCurve,
   SmSurface * pSurface,
   double d3DTolerance,
   SmBoolean &rbCurveisDegenerate)
{
    double approximate_length;
    SER (ApproximateLengthThroughSurface (cpUVCurve, pSurface, 3,
        approximate_length));
    if (approximate_length > 10.0 * d3DTolerance)
    {
        rbCurveisDegenerate = FALSE;
        return SM_SUCCESS;
    }
    SER (ApproximateLengthThroughSurface (cpUVCurve, pSurface, 7,
        approximate_length));
    if (approximate_length > d3DTolerance)
    {
        rbCurveisDegenerate = FALSE;
        return SM_SUCCESS;
    }

    rbCurveisDegenerate = TRUE;
    return SM_SUCCESS;


} // end SmTrimmingTools::IsCurveOnSurfaceDegenerate

// Note that this deletes the curves which are not copied!

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmTrimmingTools::CopyNonDegenerate
  (const SmTArray<ULONG> & crCurveLoops,
   SmTArray<SmBSplineCurve*> & cpUVCurves,
   const SmTArray<SmOrientType> & crCurveOrientations,
   SmSurface * pSurface,
   double d3DTolerance,
   SmTArray<ULONG> &loop_curve_counts,
   SmTArray<SmBSplineCurve*> &uv_curves,
   SmTArray<SmOrientType> &curve_orientations)
{
    ULONG index = 0;
    ULONG ii, jj, lNumCrvLoops = crCurveLoops.GetSize();
    for (ii = 0; ii < lNumCrvLoops; ii++)
    {
        ULONG start_of_loop = uv_curves.GetSize ();
        ULONG index_end = index + crCurveLoops [ii];

        for (jj = index; jj < index_end; jj++)
        {
            SmBoolean curve_is_degenerate;
            SER (IsCurveOnSurfaceDegenerate (cpUVCurves [jj], pSurface,
                                             d3DTolerance,
                curve_is_degenerate));

            if (!curve_is_degenerate)
            {
                uv_curves.Add (cpUVCurves [jj]);
                curve_orientations.Add (crCurveOrientations [jj]);
            }
            else {
                SM_ASSERT(cpUVCurves[jj] != NULL) ; delete cpUVCurves[jj] ;
                cpUVCurves[jj] = NULL;
            }
        }

        if (start_of_loop == uv_curves.GetSize ())
            SER (SM_ERR); // All of loop's segments were deleted

        loop_curve_counts.Add (uv_curves.GetSize () - start_of_loop);
        index = index_end;
    }
    return SM_SUCCESS;

} // end SmTrimmingTools::CopyNonDegenerate

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
SmStatus SmTrimmingTools::OrientUVLoops
  (SmTArray<ULONG> const &loop_curve_counts,
   SmTArray<SmBSplineCurve*> &uv_curves,
   SmTArray<SmOrientType> &curve_orientations)
{
    ULONG index = 0;
    ULONG ii, jj, lNumCounts = loop_curve_counts.GetSize();
    for (ii = 0; ii < lNumCounts; ii++)
    {
        ULONG index_end = index + loop_curve_counts [ii];
        SmOrientType orientation;
        SER (ComputeLoopOrientation (uv_curves, curve_orientations,
                                     index, index_end,
            orientation));
        if ((ii == 0 && orientation == SM_OT_OPPOSITE)
            || (ii > 0 && orientation == SM_OT_SAME))
        {
            uv_curves.ReverseArray( index, index_end );
            curve_orientations.ReverseArray (index, index_end);
            for (jj = index; jj < index_end; jj++)
                if (curve_orientations [jj] == SM_OT_SAME)
                    curve_orientations [jj] = SM_OT_OPPOSITE;
                else
                    curve_orientations [jj] = SM_OT_SAME;
        }

        index = index_end;
    }

    return SM_SUCCESS;

} // end SmTrimmingTools::OrientUVLoops

/*******************************************************************//**
PURPOSE:

NOTES:
***********************************************************************/
double SmTrimmingTools::Min
  (double a,
   double b)
{
    if (a < b) return a;
    else return b;

} // end SmTrimmingTools::Min

/*******************************************************************//**
PURPOSE: Return all Edgeuse in a Face that jump across surface seams.

NOTES: Returns the Edgeuse that contains the bad uv curve directly, not its mate.
***********************************************************************/
SmStatus SmTrimmingTools::CheckFaceUVSeamJumps
 (const SmFace            * pFace,         // in :
  SmTArray< SmEdgeuse* >  & raBadEUs,      // out:
  SmAssertArray           * pAList,        // NotUsed: i/o: Accumulating list of failed Asserts, NULL to ignore
  SmAssertTestLevel         eTestLevel,    // NotUsed: in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests, 
                                           //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests 
                                           //      default:[SM_LEVEL_0] 
  SmTArray<ULONG>         * pTestRequests) // NotUsed: in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL]
{
  SM_REF3(pAList, eTestLevel, pTestRequests) ;
  raBadEUs.ReSet();

  SmSurface *pSurface = pFace->GetSurface();
  if(pSurface == NULL ) { return SM_SUCCESS; }

  SmExtent2d sSurfDomain = pSurface->GetNaturalUVDomain();
  double dTol = pFace->GetTolerance();
  SmBoolean bClosedU = pSurface->IsClosed( sSurfDomain, SM_SP_U, &dTol );
  SmBoolean bClosedV = pSurface->IsClosed( sSurfDomain, SM_SP_V, &dTol );
  if(!bClosedU && !bClosedV )
    { return SM_SUCCESS; }

  // Explicitly check EUs and their Mates,
  // so that we can return which one has the bad UV curve.
  // So just work from one Faceuse.
  SmFaceuse *pFU = pFace->GetUpwardFaceuse();
  SmTArray< SmEdgeuse* > sEUs;
  pFU->GetEdgeuses( sEUs );
  SmEdgeuse * pEU = NULL;
  ULONG ii, lNumEUs = sEUs.GetSize();
  for(ii = 0; ii < lNumEUs; ii++ )
    {
      pEU = sEUs[ ii ];
      SmBSplineCurve * pUVCrv = pEU->GetUVTrimCurvePointer();
      if(pUVCrv == NULL )
        {
          pEU = pEU->GetMate();
          pUVCrv = pEU->GetUVTrimCurvePointer();
        }

      // There might not be one at all: that's fine.
      if(pUVCrv != NULL )
        {
          SmBoolean bOk = pUVCrv->AssertNoPeriodicJump(
               bClosedU, bClosedV, sSurfDomain );
          if(! bOk )
            { raBadEUs.Add( pEU ); }
        }
    } // end for each Edgeuse

  return SM_SUCCESS;

} // end SmTrimmingTools::CheckFaceUVSeamJumps

/*******************************************************************//**
PURPOSE: Regenerate the uv trim curves of the given Edgeuses.

NOTES: Returns error if any one fails, but attempts every one regardless.
***********************************************************************/
SmStatus SmTrimmingTools::RecreateUVTrimCurves
 (SmTArray<SmEdgeuse*> & rEdgeuses )
{
  // init return
  SmStatus eStat, eRetStat = SM_SUCCESS;

  // locals
  SmEdgeuse * pEU = NULL;
  ULONG ii, lNumEUs = rEdgeuses.GetSize();

  // for every target Edgeuse
  for(ii=0;ii<lNumEUs;ii++)
    {
      pEU = rEdgeuses[ii];

      // rebuild UVTrimCurve
      eStat = pEU->RebuildUVTrimCurve();
      if(eStat != SM_SUCCESS )
        { eRetStat = eStat; }
    }

  // all done
  return eRetStat;

} // end SmTrimmingTools::RecreateUVTrimCurves


/*******************************************************************//**
PURPOSE: Methods of helper class SmCurveLoops
***********************************************************************/

/*******************************************************************//**
PURPOSE: SmCurveLoops Constructor

NOTES: Sets member m_bIsValid 
         TRUE = constructor input ok
         FALSE= constructor input included neither UVTrimCurves nor 3D Curves - must have some curves
***********************************************************************/
SmCurveLoops::SmCurveLoops
 (const SmTArray<ULONG>             & crCurveLoops,        // in : List of the counts of curves in each Loop, m_sLoopCounts.GetSize() = number of loops  
  const SmTArray<SmBSplineCurve*>   * cpOptUVCurves,       // in : opt list of UVTrimCurves, one of cpOptUVCurves or cpOpt3DCurves must be NonNULL  
  const SmTArray<SmCurve*>          * cpOpt3DCurves,       // in : opt List of 3d curves,    only used when cpOptUVCurves is NULL, one of cpOptUVCurves or cpOpt3DCurves must be NonNULL  
  const SmTArray<SmOrientType>      & crCurveOrientations) // in : List of orientations for each of the curves     
 : m_sOrients(crCurveOrientations),
   m_sLoopCounts(crCurveLoops)
{
  // init state
  m_bIsValid = TRUE;

  // copy input arrays.
  if(cpOptUVCurves != NULL)
    {
      // for every 
      ULONG lNumCurves = cpOptUVCurves->GetSize();
      for(ULONG ii=0;ii<lNumCurves;ii++)
        {
          // add Curve to m_sCurves array - copies the input curve here
          m_sCurves.Add( (SmCurve*)(*cpOptUVCurves)[ii] );
        }
    } // end UVTrimCurve branch

  else if(cpOpt3DCurves != NULL)
    {
      m_sCurves = *cpOpt3DCurves; // Assignment operator - copies the input curve
    }

  else // no in[ut curve given - remember error
    { m_bIsValid = FALSE; }

} // End SmCurveLoops constructor

/*******************************************************************//**
PURPOSE: Empty member data arrays

NOTES: 
***********************************************************************/
SmStatus SmCurveLoops::Reset()
{
  m_sCurves.ReSet();
  m_sOrients.ReSet();
  m_sLoopCounts.ReSet();
  m_bIsValid = TRUE; // Nothing wrong with an empty object.

  // all done
  return SM_SUCCESS;

} // end SmCurveLoops::Reset

/*******************************************************************//**
PURPOSE: Find the index into m_sCurves of the start of the given Loop in m_sLoopCounts.

NOTES: Returns error if lLoopIndx is out of range.
***********************************************************************/
SmStatus SmCurveLoops::GetLoopStartIndex
 (ULONG   lLoopIndx,  // in : Loop number: Index into m_sLoopCounts    
  ULONG & rlStart )   // out: Index of the first curve in lLoopIndx's loop  
 const
{
  // init output
  rlStart = 0;

  // check state - out of bounds lLoopIndx
  if(lLoopIndx >= m_sLoopCounts.GetSize())
    { return SM_ERR_INVALID_INPUT; }

  // count number of curves to the first curve in the lLoopIndx-th loop
  for(ULONG ii=0; ii<lLoopIndx; ii++ )
    { rlStart += m_sLoopCounts[ii]; }

  // all done
  return SM_SUCCESS;

} // end SmCurveLoops::GetLoopStartIndex

/*******************************************************************//**
PURPOSE: Find the loop indices of the start and end of the loop containing the given lCurveIndx.

NOTES: Returns error if lCurveIndx is out of range.
***********************************************************************/
SmStatus SmCurveLoops::GetLoopStartEnd
 (ULONG   lCurveIndx, // in  : Index of the curve in question
  ULONG & rlStart,    // out : Index of the first curve in lCurveIndx's loop
  ULONG & rlEnd)      // out : Index of the last  curve in lCurveIndx's loop
    const
{
  // init output
  ULONG ii ;
  rlStart = 0 ;
  rlEnd   = 0 ;

  // for every loop
  for(ii=0;ii<m_sLoopCounts.GetSize();ii++)
    {
      // count the curves in the array prior to the loop containing the tgt curve
      rlEnd += m_sLoopCounts[ii];
      if(ii == 0) { rlEnd--; }

      // done when curve count passes tgt curve 
      if(lCurveIndx <= rlEnd )
        { return SM_SUCCESS; }

      // incrment rlStart to beginning of next loop
      rlStart = rlEnd;
    } // end iter every loop looking for tgt curve containment

  // all done - only arrive here when TgtCurve index is not in a loop
  return SM_ERR;

} // end SmCurveLoops::GetLoopStartEnd

/*******************************************************************//**
PURPOSE: Find the adjacent loop curve, given the start or end of a tgt loop curve.

NOTES: Returns error if any one fails, but attempts every one regardless.
***********************************************************************/
SmStatus SmCurveLoops::FindAdjacentLoopCurve
 (SmCurve * pCrv,       // in : Pointer to the curve in question
  double    dParam,     // in : Parameter on pCrv: indicates start/end
  ULONG   & rlAdjIndx ) // out: Index of the adjacent curve in the loop
{
  // init output
  ULONG lCurveIndx = 0 ;

  // locals
  ULONG lStart, lEnd;

  // get CurveIndx for TgtCurve
  if(!m_sCurves.FindElement(pCrv, lCurveIndx))
    { return SM_ERR; }

  // get range of curves in the loop that contains pCrv
  this->GetLoopStartEnd( lCurveIndx, lStart, lEnd );

  // dParam in 1st half of curve branch
  if(dParam < pCrv->GetNaturalInterval().GetMid()) { // get previous curve on circular list
                                                     rlAdjIndx = (lCurveIndx == lStart) ? lEnd : lCurveIndx-1 ;
                                                   } 

  else /* dParam in 2nd half of curve branch */    { // get following curve.on circular list
                                                     rlAdjIndx = (lCurveIndx == lEnd) ? lStart : lCurveIndx+1 ;
                                                   }
  // all done
  return SM_SUCCESS;

} // End SmCurveLoops::FindAdjacentLoopCurve

/*******************************************************************//**
PURPOSE: Given a consecutive pair of curves in a loop - 
         find which is entering and which is leaving

NOTES: They will usually be consecutive numbers, and the start will be the first one,
   but they might straddle the loop boundary in the loop structure.
   That's why this convenience routine exists: it's a lot of busy work to include in the code.
***********************************************************************/
SmStatus SmCurveLoops::FindEnteringAndLeaving
 (ULONG   lCurveIndx1,     // in : index in the LoopCurves array of one of the two connecting Loop curves
  ULONG   lCurveIndx2,     // in : index of the other intersecting Loop curve
  ULONG & rlEnteringCurve, // out: either lCurveIndx1 or lCurveIndx2, whichever is 'entering' this intersection
  ULONG & rlLeavingCurve)  // out: the other Indx, which is 'leaving'
{                       
  // check state - must have loops
  if(m_sLoopCounts.GetSize() < 1)
    { return SM_ERR; }

  // locals
  ULONG ii ;
  ULONG lLoopNumber    = 0;
  ULONG lNextLoopStart = 0;

  // for every loop - find the loop containing lCurveIndx1 
  for(ii=0;ii<m_sLoopCounts.GetSize();ii++)
    {
      lNextLoopStart += m_sLoopCounts[ii];

      if(lCurveIndx1 < lNextLoopStart)
        {
          lLoopNumber = ii;
          break;
        } // end lCurveIndx1 is in this loop check
    } // end iter every loop looking for lCurveIndx1 containment

  // containing loop start and end CrvIndices
  ULONG lStartCurveIndx = lNextLoopStart - m_sLoopCounts[lLoopNumber];
  ULONG lEndCurveIndx   = lNextLoopStart - 1;

  // Check state - lCurveIndx1 and lCurveIndx2 are in the same loop
  if(   lCurveIndx2 < lStartCurveIndx  
     || lCurveIndx2 > lEndCurveIndx )
    { return SM_ERR; }

  // If there are more than two curves in the loop, 
  //   then if the two indices differ by 1,
  //   then the lower one is the entering curve, higher is leaving.
  if(m_sLoopCounts[lLoopNumber] > 2)
    {
      if(lCurveIndx1 == lCurveIndx2+1)      { rlEnteringCurve = lCurveIndx2;
                                              rlLeavingCurve  = lCurveIndx1;
                                              return SM_SUCCESS;
                                            }
      else if(lCurveIndx2 == lCurveIndx1+1) { rlEnteringCurve = lCurveIndx1;
                                              rlLeavingCurve  = lCurveIndx2;
                                              return SM_SUCCESS;
                                            }
    }

  // Either two or fewer curves, or they're not consecutive.
  // Have to check for straddling the 'seam': start & end of loop.
  // If that's the case, then Enter is the end of the loop and Leave is the start.
  if( (lCurveIndx1 == lStartCurveIndx && lCurveIndx2 == lEndCurveIndx )
      ||(lCurveIndx2 == lStartCurveIndx && lCurveIndx1 == lEndCurveIndx ) )
    {
      rlEnteringCurve = lEndCurveIndx;
      rlLeavingCurve  = lStartCurveIndx;
    }
  else
    {
      // They don't straddle the loop's end, so they should be consecutive.
      SM_ASSERT( lCurveIndx1 == lCurveIndx2+1 || lCurveIndx1 == lCurveIndx2-1 );
      if(lCurveIndx2 == lCurveIndx1+1)      { rlEnteringCurve = lCurveIndx1;
                                              rlLeavingCurve  = lCurveIndx2;
                                            }
      else if(lCurveIndx1 == lCurveIndx2+1) { rlEnteringCurve = lCurveIndx2;
                                              rlLeavingCurve  = lCurveIndx1;
                                            }
      else                                  { return SM_ERR; }
    }

  // all done
  return SM_SUCCESS;

} // end SmCurveLoops::FindEnteringAndLeaving

/*******************************************************************//**
PURPOSE: Find points just before and after the given end of the given loop curve.

NOTES: When dParam is at a curve end - sample point on adjacent curve in the loop
***********************************************************************/
SmStatus SmCurveLoops::SampleNearbyPoints
 (SmCurve   * pCrv,     // in : Pointer to the curve in question
  double      dParam,   // in : Parameter on pCrv: indicates start/end
  SmPoint3d & rPt1,     // out: A point on the loop just before pCrv/dParam
  SmPoint3d & rPt2)     // out: A point on the loop just after pCrv/dParam
{
  // locals
  SmExtent1d sDomain = pCrv->GetNaturalInterval();
  double     dT;
  double     dTol = sDomain.GetLength() / 1000; //cbi anything better?

  // when dParam is in the interior by more than tol
  if(sDomain.ContainsValue(dParam, -dTol)) // Neg tol: strictly interior
    {
      // Use 1/5 of the way to the ends of the domain.
      dT = 0.8 * dParam + 0.2 * sDomain.GetMin();
      pCrv->EvaluatePoint( dT, rPt1 );
      dT = 0.8 * dParam + 0.2 * sDomain.GetMax();
      pCrv->EvaluatePoint( dT, rPt2 );
    }
  else // dParam is within tol of the boundaries - eval point on adjacent curve
    {
      // Param is at one end of the domain.
      SM_ASSERT( sDomain.ContainsValue( dParam, dTol ) ); // S/B at an end.

      // locals
      ULONG lAdjIndx = 0 ;

      // find adjacent loop curve
      this->FindAdjacentLoopCurve( pCrv, dParam, lAdjIndx );
      SmCurve     * pAdjCrv    = m_sCurves[ lAdjIndx ];
      SmExtent1d    sAdjDomain = pAdjCrv->GetNaturalInterval();
      SmOrientType  eAdjOrient = m_sOrients[ lAdjIndx ];

      // when dParam is at curve begin - evaluate adjacent curve near connection
      if(dParam < sDomain.GetMid() )
        {
          dT = sDomain.Evaluate( 0.2 );
          pCrv->EvaluatePoint( dT, rPt1 );

          dT = (eAdjOrient == SM_OT_SAME) ? sAdjDomain.Evaluate( 0.8 )
                                          : sAdjDomain.Evaluate( 0.2 );
          pAdjCrv->EvaluatePoint( dT, rPt2 );
        }
      else // when dParam is at curve end - evaluate adjacent curve near connection
        {
          dT = sDomain.Evaluate( 0.8 );
          pCrv->EvaluatePoint( dT, rPt1 );

          dT = (eAdjOrient == SM_OT_SAME ) ? sAdjDomain.Evaluate( 0.2 )
                                           : sAdjDomain.Evaluate( 0.8 );
          pAdjCrv->EvaluatePoint( dT, rPt2 );
        }
    } // end dParam is within tol of curve end branch - evaluate point on adjacent curve

  // all done
  return SM_SUCCESS;

} // End SmCurveLoops::SampleNearbyPoints

/*******************************************************************//**
PURPOSE: Given a loop curve and its start/end, decide whether the intersection
         is crossing or same-side (possibly tangent).

NOTES: The curve-end is the intersection with the isocurve in pCurveInts.
***********************************************************************/
SmBoolean SmCurveLoops::IsSameSideIntersection
 (SmCurve              * pCrv,         // in : Pointer to the curve in question
  double                 dParam,       // in : Parameter on pCrv: indicates start/end
  SmCurveIntersections * pCurveInts,   // in : An isocurve/loop curve intersection
  const SmSurface      * pSrf,         // in : surface containing the curves
  const SmExtent2d     & crSrfDomain,  // in : surface domain of interest
  SmZoneTol3d          & rZoneTol3d )  // in : An isocurve/loop curve intersection
{
  // locals
  SmPoint3d sPt1, sPt2;

  // sample points near TgtCurve
  this->SampleNearbyPoints( pCrv, dParam, sPt1, sPt2 );
  SmPoint3d sUv1( sPt1 ), sUv2( sPt2 );

  // when Curves are in 3d - drop nearby Pts to Surface UV domain
  if(pCrv->GetDim() == 3 )
    {
      // We can't work in 3d: if a crease angle is acute, dot product will be positive.
      SmSolutionArray sSols;

      // drop nearby TgtCurve Pt1 to Surface
      SmStatus eStat = pSrf->GlobalPointSolve(crSrfDomain,     // in : Domain of surface to search for solutions  
                                              SM_SO_INTERSECT, // in : oneof: SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT  
                                              sPt1,            // in : Target point for the solve operation  
                                              rZoneTol3d,      // in : Obj ZoneTol3d assoc with Target Point, if none, use: SmTol::GetZoneTol3d(BREP_CONTEXT_OR_NULL)  
                                              NULL,            // in : Max/Min Drop distance for min/max and normalize operations. NULL to ignore.  
                                              SM_SR_SINGLE,    // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions  
                                              sSols );         // out: array of problem solutions reported as surface UV parameter values  
      // When point fails to drop to surface - return FALSE - this is not a SameSideIntersection
      if(eStat != SM_SUCCESS  ||  sSols.GetSize() != 1 )
        { return FALSE; }

      // map solution to a UV Point
      sUv1.Set(sSols[0].m_vStart.m_adParameters[0], 
               sSols[0].m_vStart.m_adParameters[1], 
               0.0 ) ;

      // drop nearby TgtCurve Pt2 to Surface
      eStat = pSrf->GlobalPointSolve(crSrfDomain,      // in : Domain of surface to search for solutions    
                                     SM_SO_INTERSECT,  // in : oneof: SM_SO_MINIMIZE, SM_SO_MAXIMIZE, SM_SO_NORMALIZE, SM_SO_INTERSECT    
                                     sPt2,             // in : Target point for the solve operation    
                                     rZoneTol3d,       // in : Obj ZoneTol3d assoc with Target Point, if none, use: SmTol::GetZoneTol3d(BREP_CONTEXT_OR_NULL)  
                                     NULL,             // in : Max/Min Drop distance for min/max and normalize operations. NULL to ignore.    
                                     SM_SR_SINGLE,     // in : SM_SR_SINGLE=get best solution, SM_SR_ALL=get all solutions    
                                     sSols) ;          // out: array of problem solutions reported as surface UV parameter values    
      // When point fails to drop to surface - return FALSE - this is not a SameSideIntersection
      if(eStat != SM_SUCCESS  ||  sSols.GetSize() != 1 )
        { return FALSE; }

      // map solution to a UV Point
      sUv2.Set(sSols[0].m_vStart.m_adParameters[0], 
               sSols[0].m_vStart.m_adParameters[1], 
               0.0) ;
    } // end curves are in 3d check

  // 
  SmSurfParamType eUVDir = pCurveInts->m_eSrfDir ;
  double          dVal1 = (eUVDir == SM_SP_U ) ? sUv1.x : sUv1.y;
  double          dVal2 = (eUVDir == SM_SP_U ) ? sUv2.x : sUv2.y;
  double          dIsoParam = pCurveInts->m_dIsoParam;

  // all done 
  return(dVal1-dIsoParam ) *(dVal2-dIsoParam ) > 0.0; // Same or opposite signs

} // end SmBoolean SmCurveLoops::IsSameSideIntersection

/*******************************************************************//**
PURPOSE: Append a Loop from another SmCurveLoops object onto this one.

NOTES: Returns error if lOtherLoopIndex is out of range.
***********************************************************************/
SmStatus SmCurveLoops::AppendLoop
 (const SmCurveLoops & crOther,         // in : Another SmCurveLoops structure
  ULONG                lOtherLoopIndex) // in : Index into m_sLoopCounts in crOther
{
  // check input - lOtherLoopIndex must be in NumLoops
  if(lOtherLoopIndex >= crOther.GetNumLoops() )
    { return SM_ERR_INVALID_INPUT; }

  // locals
  ULONG ii ;
  ULONG lStart = 0;

  // find 1st Curve indx in OtherCurveLoops for this OtherLoopIndex
  SER( crOther.GetLoopStartIndex( lOtherLoopIndex, lStart ));

  // find first curve indx of the loop following the one that contains lOytherLoopIndex
  ULONG lNextStart = lStart + crOther.m_sLoopCounts[ lOtherLoopIndex ];

  // for every Curve in the OtherTgtLoop
  for(ii=lStart;ii<lNextStart;ii++)
    {
      // copy curve data into this SmCurveLoops
      m_sCurves.Add(crOther.m_sCurves [ii]) ;
      m_sOrients.Add(crOther.m_sOrients[ii]) ;
    }

  // add the curve count for the new loop to this m_sLoopCounts arrray
  m_sLoopCounts.Add( crOther.m_sLoopCounts[ lOtherLoopIndex ] );

  // all done
  return SM_SUCCESS;

} // end SmCurveLoops::AppendLoop

/*******************************************************************//**
PURPOSE: Insesrt a curve into the curve loop structure.
         Used when a loop curve is split.

NOTES:
***********************************************************************/
SmStatus SmCurveLoops::InsertIntoLoops
 (SmCurve * pNewCurve,  // in: new curve to insert
  SmCurve * pLoopCrv )  // in: existing curve to insert after
{
  // lcoals
  ULONG ii ;
  ULONG lCurveIndx ;

  // check state TgtLoopCrv is in m_sCurves
  if(FALSE == m_sCurves.FindElement( pLoopCrv, lCurveIndx ) )
    { return SM_ERR_INVALID_INPUT; }

  // insert curve and orient data into array
  m_sCurves .InsertAt( lCurveIndx+1, pNewCurve );
  m_sOrients.InsertAt( lCurveIndx+1, m_sOrients[lCurveIndx] );  // Same orientation as original

  // Update sLoops: just find the appropriate entry and increment it by one.
  ULONG lTotal = 0;
  for(ii=0;ii<m_sLoopCounts.GetSize();ii++)
    {
      // when this Curve index is in current Loop
      if(lCurveIndx < lTotal + m_sLoopCounts[ii] )
        {
          // increment the CurveCount for this loop - and exit
          m_sLoopCounts[ii]++;
          break;
        } // end found the containing loop check

      // set up for next iteration
      lTotal += m_sLoopCounts[ii];

    } // end iter every Loop - accumulating crv counts

  // all done
  return SM_SUCCESS;

} // end SmCurveLoops::InsertIntoLoops

/*******************************************************************//**
PURPOSE: Insert this segment of a curve into the curve loop structure.

NOTES: This will split a curve loop.  The argument contains the curve to insert,
       and information about where to insert it in the loop.
***********************************************************************/
SmStatus SmCurveLoops::InsertIsoXSectSegment
 (SmIsoXSectSegment & rIntSeg) // in : segment to be inserted into loop
                               //           rIntSeg::m_pIsoCurve - curve being inserted to the loop
                               //                    m_sInterval - curve's insert interval between Enter and Leave
                               //                    m_eOrient   - oneof: SM_OT_SAME, SM_OT_OPPOSITE. Orientation of curve insert interval
                               //                    m_lStartEnter - index of LoopCurve coming into the insertion loop start point
                               //                    m_lStartLeave - index of LoopCurve leaving the inserting loop start point
                               //                    m_lEndEnter   - index of LoopCurve coming into the insertion loop end point
                               //                    m_lEndLeave   - index of LoopCurve leaving the inserting loop end point
{
#ifdef SM_DEBUG_CODE
static constexpr SmBoolean bDebugMe=FALSE;
  if(bDebugMe ) 
    {
      SM_DUMP_TARRAY( m_sLoopCounts );
      SM_DUMP_TARRAY( m_sCurves );
      SM_DUMP_TARRAY( m_sOrients );
    }
#endif // SM_DEBUG_CODE

  // We're going to copy the curve loop structure into a new one,
  // adding in new curves for the iso curves.
  // Otherwise it would be too awkward to modify the lists while we're working on them.

  // locals
  ULONG ii ;
  SmTArray<ULONG>        sNewLoops ;
  SmTArray<SmCurve *>    sNewLoopCurves ;
  SmTArray<SmOrientType> sNewOrients ;

  // Find which loop the iso edge will go into.
  ULONG lMinIndx = rIntSeg.GetMinIndex();
  if(lMinIndx >= m_sCurves.GetSize())
    { return SM_ERR; }

  ULONG lLoopNumber    = m_sLoopCounts.GetSize();  // Error if it stays there.
  ULONG lTotalCrvCount = 0;

  for(ii=0;ii<m_sLoopCounts.GetSize();ii++)
    {
      if(lMinIndx < lTotalCrvCount + m_sLoopCounts[ii] )
        {
          lLoopNumber = ii;
          break;
        }
      lTotalCrvCount += m_sLoopCounts[ii];
    }

  if(lLoopNumber >= m_sLoopCounts.GetSize() )
    { return SM_ERR; }

  // check state - highest index is in the same loop.
  // End of current loop would be lTotalCrvCount + m_sLoopCounts[ii].
  ULONG lMaxIndx = rIntSeg.GetMaxIndex();
  if(lMaxIndx >= lTotalCrvCount + m_sLoopCounts[lLoopNumber] )
    { return SM_ERR; }

  // Okay, we have identified the loop containing the insertion.
  // Copy any previous loops into the new loop structures.
  ULONG lStartCurveIndx = 0;
  for(ii=0;ii<lLoopNumber;ii++)
    {
      sNewLoops.Add( m_sLoopCounts[ii] );

      for(ULONG jj=0; jj< m_sLoopCounts[ii]; jj++ )
        {
          sNewLoopCurves.Add( m_sCurves[ lStartCurveIndx + jj ] );
          sNewOrients   .Add( m_sOrients   [ lStartCurveIndx + jj ] );
        }
      lStartCurveIndx += m_sLoopCounts[ii];
    }

  // Now create two new loops with new iso curves inserted.
  // Start the new loops at the new iso curves.  Doesn't hurt anything to have
  // different starting points, and it makes the code easier.

  // Create a new Forward curve.
  SmCurve * pNewIso1  = NULL;
  SmCurve * pIsoCurve = rIntSeg.m_pIsoCurve;

  SER( pIsoCurve->Copy( *(pIsoCurve->GetContext()), pNewIso1 ) );
  SER( pNewIso1->Trim( rIntSeg.m_sInterval ) );

  // Start a new Forward loop from there: from End-Leave to Start-Enter.
  sNewLoopCurves.Add( pNewIso1 );
  sNewOrients.   Add( SM_OT_SAME );
  sNewLoops.     Add( 1 ); // That's how big it is now.

  ULONG lNumLoopCrvs    = m_sCurves.GetSize();
  ULONG lThisIndx       = rIntSeg.m_lEndLeave;
        lStartCurveIndx = lTotalCrvCount;
  ULONG lEndCurveIndx   = lTotalCrvCount + m_sLoopCounts[ lLoopNumber ];

  // Note: this is a 'fake' for-loop.  The loop should run from EndLeave to StartEnter,
  // including both of them.  To do that, we do the action, then the check, then
  // increment the index.  That gets the action done one more time than the incrementing,
  // which is necessary in this case.  (The Fencepost problem.)
  // But doing the for-loop has the advantage of preventing an infinite loop,
  // which can happen when the termination condition is an exact comparison
  // (e.g., not '>' or '<').
  for(ii=0;ii<=lNumLoopCrvs;ii++)
    {
      sNewLoopCurves.Add( m_sCurves[lThisIndx]) ;
      sNewOrients.Add( m_sOrients[lThisIndx]) ;
      sNewLoops[lLoopNumber]++ ;

      if(lThisIndx == rIntSeg.m_lStartEnter )
        { break; }

      lThisIndx++;
      if(lThisIndx >= lEndCurveIndx )
        { lThisIndx = lStartCurveIndx; }  // Circular within this curve loop.
    }

  // Create a new Backwards curve.
  SmCurve *pNewIso2 = NULL;
  SER( pNewIso1->Copy( *(pIsoCurve->GetContext()), pNewIso2 ) );  // (already trimmed)

  // Start a new Forward loop from there: from End-Leave to Start-Enter.
  sNewLoopCurves.Add( pNewIso2 );
  sNewOrients.   Add( SM_OT_OPPOSITE );
  sNewLoops.     Add( 1 ); // That's how big it is now.

  lLoopNumber++;
  lThisIndx = rIntSeg.m_lStartLeave;

  // Fake for-loop: see note above.
  for(ii=0;ii<=lNumLoopCrvs;ii++ )
    {
      sNewLoopCurves.Add( m_sCurves[lThisIndx]) ;
      sNewOrients.Add( m_sOrients[lThisIndx]) ;
      sNewLoops[lLoopNumber]++;

      if(lThisIndx == rIntSeg.m_lEndEnter) 
        { break; }

      lThisIndx++;
      if(lThisIndx >= lEndCurveIndx )
        { lThisIndx = lStartCurveIndx; }  // Circular within this loop.
    }

  // Now copy any following loops into the new loop structures.
  // Figure where to start in m_sCurves.
  lTotalCrvCount = 0;
  for(ii=0;ii<lLoopNumber;ii++)
    { lTotalCrvCount += m_sLoopCounts[ii]; }

  for(; lLoopNumber< m_sLoopCounts.GetSize(); lLoopNumber++ )
    {
      sNewLoops.Add( m_sLoopCounts[lLoopNumber] );

      for(ULONG jj=lTotalCrvCount; jj < lTotalCrvCount + m_sLoopCounts[lLoopNumber]; jj++ )
        {
          sNewLoopCurves.Add( m_sCurves[jj] );
          sNewOrients   .Add( m_sOrients[jj] );
        }
      lTotalCrvCount += m_sLoopCounts[lLoopNumber];
      //cbi lLoopNumber++;
    }

  // Update the loop structure for return.
  m_sLoopCounts = sNewLoops;
  m_sCurves     = sNewLoopCurves;
  m_sOrients    = sNewOrients;

#ifdef SM_DEBUG_CODE
  if(bDebugMe ) 
    {
      SM_DUMP_TARRAY(m_sLoopCounts) ;
      SM_DUMP_TARRAY(m_sCurves) ;
      SM_DUMP_TARRAY(m_sOrients) ;
    }
#endif // SM_DEBUG_CODE

  // all done
  return SM_SUCCESS;

} // end SmCurveLoops::InsertIsoXSectSegment

/*******************************************************************//**
PURPOSE: After splitting a curve, any solutions that are on that curve
   might now lie on the newly-split-off curve.  If so, update the solution.

NOTES:
***********************************************************************/
SmStatus SmCurveLoops::UpdateSolutionObjects
(SmCurveIntersections * pCrvInts,   // in : linked list of all the CurveInts 
 SmCurve              * pOrigCurve, // in : the curve that got split (reused as one child) 
 SmCurve              * pNewCurve,  // in : the new curve, (the other child) split off from pOrigCurve
 SmZoneTol3d          & rTol3d )    // in : siZe of tolerant neighborhood about each curve
{
  // locals
  ULONG ii ;
  SmExtent1d             sDomainOrig = pOrigCurve->GetNaturalInterval() ;
  SmExtent1d             sDomainNew  = pNewCurve ->GetNaturalInterval() ;
  SmCurveIntersections * pCrvInt     = pCrvInts ;

  // while pCrvInts remain to be processed
  while(pCrvInt != NULL )
    {
      // for every XSect solution for this pCrvInt
      for(ii=0;ii<pCrvInt->m_sSolArray.GetSize();ii++)
        {
          SmSolution & rSol     = pCrvInt->m_sSolArray[ii];
          SmObject   * pObj2    = rSol.m_apObjects[1];  // This is the loop curve.
          SmCurve    * pLoopCrv = SM_CAST_PTR( SmCurve, pObj2 );

          // skip curves that are not the Origin Curve
          if(pLoopCrv != pOrigCurve )
            { continue; }

          // This solution points to pOrigCurve.
          double  dParamLC  = rSol.m_vStart.m_adParameters[1];
          SmTol1d sParamTol = SmTol::MapTo1d( rTol3d, dParamLC, *pOrigCurve );

          // when Dropped Param is NOT contained in the Curve Natural Ivl
          if(FALSE == sDomainOrig.ContainsValue( dParamLC, sParamTol ))
            {
              // when Dropped Param is contained by New Curve - make New Curve the sol's Object
              if(sDomainNew.ContainsValue( dParamLC, sParamTol ) ) { rSol.m_apObjects[1] = pNewCurve; }
              else /* error - signal error and move to next Sol */ { SE( SM_ERR );
                                                                     continue;  // Just keep going for now.
                                                                   }
            } // end Dropped para is NOT contained in the Curve Natural Ivl check
        } // end iter every XSect solution for this pCrvInt

      // increment pCrvInt ptr to next Crv in the linked list
      pCrvInt = pCrvInt->m_pNext;

    } // end while pCrvIts remain to be processed

  // all done
  return SM_SUCCESS;

}  // end SmCurveLoops::UpdateSolutionObjects
