// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/*******************************************************************//**
* FILE NAME --- SmFace.h 
* PURPOSE: Header file for SmFace class.
**********************************************************************/

#ifndef __SMFACE_H__
#define __SMFACE_H__

#ifndef __SMTOPOLOGY_H__
#include <SmTopology.h>
#endif

#ifndef __SMSURF_TYPES_H__
#include <SmSurfTypes.h>
#endif

#ifndef __SMEXTENT2D_H__
#include <SmExtent2d.h>
#endif

#ifndef __SMVECTOR3D_H__
#include <SmVector3d.h>
#endif

#ifndef __SMPOINTCLASSIFICATION_H__
#include <SmPointClass.h>
#endif

#ifndef __SMCURVECLASSIFICATION_H__
#include <SmCurveClass.h>
#endif

#include <SmGraphicsExtern.h>

#ifndef __SMSURFACE_H__
#include <SmSurface.h>
#endif

#ifndef __SMSHELL_H__
#include <SmShell.h> 
#endif

#ifndef __SMFACEUSE_H__
#include <SmFaceuse.h>
#endif

#ifndef __SMFACEPROPS_H__
#include <SmFaceProps.h>
#endif

#ifndef __SMTOLERANCE_H__
#include <SmTol.h>
#endif

#ifndef __UNORDERED_SET__
#define __UNORDERED_SET__
#include <unordered_set>
#endif

class SmGfxArraySet ;
class SmGapArray ;
class SmFaceProps ;
class SmLooptreeItem ;

// Independent groups for SmFace::ComputePreciseProperties; combine with |.
// Second moments include squared-coordinate integrals, products, and inertia
// about axes through the supplied origin. They do not request first moments.
enum SmPrecisePropertyFlags
{
  SM_PPF_AREA                 = 0x01,
  SM_PPF_AREA_FIRST_MOMENTS    = 0x02,
  SM_PPF_AREA_SECOND_MOMENTS   = 0x04,
  SM_PPF_VOLUME               = 0x08,
  SM_PPF_VOLUME_FIRST_MOMENTS  = 0x10,
  SM_PPF_VOLUME_SECOND_MOMENTS = 0x20,
  SM_PPF_ALL                  = 0x3f
};

/*******************************************************************//**
PURPOSE: This class represents topological faces within a Brep.

NOTES: 
***********************************************************************/
class SM_EXPORT SmFace : public SmTopology
{
  friend class SmBrep;
  friend class SmLoop;
  friend class SmShell;
// Remove Composites
// friend class SmCFace;
  friend class SmBrepConstructor;
  friend class SmObjsDelete<SmFace*>;
  friend class SmHealData ;

protected:
  // inherited:
  //                 SmTopology::m_pListOwner - when in a Brep - SmBrep::m_pFaceListHead
  //                 SmTopology::m_pNext      - Next member of face list owned by SmBrep::m_pFaceListHead
  //                 SmTopology::m_pLast      - Last member of face list owned by SmBrep::m_pFaceListHead 
  SM_NEWTOL_LINE // SmTopology::m_bIsSmallTopology - used by Face, Edge, Vertex (rarely needed - leave defaulted 99.99% of the time)
                  //                                - TRUE = Intended small geometry size (Pinhole in Battleship) - gets tighter tolerances
                  //                                - FALSE= Typical size - gets typical tolerances
                  //                                -   default:[FALSE] 
  
  SmFaceuse          * m_pFU;               // First faceuse is always the upward faceuse by convention.
                                            //   So, Faceuse Normal = Face->m_pSurface->Normal and
                                            //       Faceuse Orientation = SM_OT_SAME.
                                            //   Primary Faceuse is always connected to a SmLoop's primary Loopuse
                                            //   and Secondary Faceuse is always connected to a SmLoop's secondary Loopuse.
  SmBoolean            m_bRectangularTrim;  // TRUE  = Face has 1 UVRectangle OuterLoop of 4 isoUVTrimCurves with extents m_vUVDomain, FALSE=isn't, UNSURE = Not yet set
                                            // FALSE = isn't
                                            // UNSURE = Not yet set 
  SmSurface          * m_pSurface = NULL;   // Pointer to surface associated with the face
  SmExtent2d           m_vUVDomain;         // Domain on surface used by face
  //SmBoolean            m_bSwapUVInOrient;   // This face requires swapping UV after import. Included to remove redundant swapping of UV when importing faces.
  //                                          //   [TRUE]  = Negate this face's normal when looking for a majority to determine normal orientation in SmBrep::OrientTrimmedSurfaces
  //                                          //   [FALSE] = No need to negate normal. Default is [FALSE].


  SM_OLDTOL_LINE mutable SmZoneTol3d m_sZoneTol3d ;        // Stored ZoneTol3d for this object

protected:
  SmFace() ;
  virtual ~SmFace();

public:
  // Build an approx to current Surface with new parameterization
  SmStatus ApproximateSurface
  (
    SmBSplineSurface *& rpNewBSplineSurface,  // out: New Surf if possible 
    SmBoolean           bReplaceSurf=TRUE     // in : TRUE=replace and delete Curr Surf, FALSE = don't 
  );

  // Replace a nonSmBSpline face->surface with an SmBSpline Surface approximation
  SmStatus ApproximateWithBSpline
  (
    double   dApproxTol3d,   // in : maximum allowed distance between Approx and original surfaces 
    double & dAchievedTol3d  // out: actual max distance between approx and original surfaces 
  ) ;

  // Calculate the normal axis aligned bounding box of the Face
  SmStatus CalculateBoundingBox
  (
    SmExtent3d & rFaceBBox,       // out: bounding box as large or larger than this SmFace       
    SmBoolean    bTight = FALSE   // in : TRUE = compute minimal box  (expensive)       
                                  //        FALSE= compute any box larger than this Face (cheaper) 
  ) const;

  // Calculate a more precise normal axis aligned bounding box of the Face
  SmStatus CalculateTightBoundingBox( SmExtent3d & rFaceBBox ) const;

  // Generates a point known to be on the face and inside the face boundaries
  SmStatus CalculateAnInternalPoint( SmPoint3d &rPointInFace ) const;

  // Calculate UV Domain of the face from the trim curves bounding box
  SmStatus CalculateUVDomainFromUVTrimCurves
  (
    SmExtent2d & rNewFaceDomain, // out: UV BoundingBox for all UVTrimCurve Control Point Positions 
    SmBoolean    bLimitDomain=TRUE,        // in : TRUE= do not return a domain larger than the surface's domain Default:[TRUE] 
    double     * pOptMaxUVLoopGap=NULL,    // out: Max 2D Loop->EdgeuseEnd_to_EdgeuseEnd Gap, NULL to ignore. Default:[NULL] 
    double     * pOpt2ndMaxUVLoopGap=NULL  // out: 2nd largest UVLoopGap (for MissingSeamFaces where MaxUVLoopGap can be as  
                                           //      large as the surface domain but the 2nd largest is expected to be a good  
                                           //      size scale for tolerances.) NULL to ignore, Default:[NULL] 
  ) const;                              
                                       
  // Calculate UV Domain of the face from the trim curves bounding box
  SmStatus CalculateMaxLoopUVGap
  (
    double     & rMaxUVLoopGap,            // out: Max 2D Loop->EdgeuseEnd_to_EdgeuseEnd Gap, NULL to ignore. 
    double     * pOpt2ndMaxUVLoopGap=NULL  // out: 2nd largest UVLoopGap (for MissingSeamFaces where MaxUVLoopGap can be as 
                                           //      large as the surface domain but the 2nd largest is expected to be a good 
                                           //      size scale for tolerances.) NULL to ignore, Default:[NULL] 
  ) const;  
                                       
  // Compute the orientation of a loop within a face
  SmStatus ComputeLoopOrientation
  (
    const SmLoop      * pLoop,                        // in : target loop 
    SmOrientType      & reLoopOrient,                 // out: SM_OT_SAME     = outerLoop 
                                                      //      SM_OT_OPPOSITE = InnerLoop 
    SmExtent2d        * pUVBounds = NULL,             // out: UVBoundingBox of all loop->Edges 
                                                      //      NULL to ignore. default:[NULL] 
    SmBoolean           bDo3DLoopComputation = FALSE  // in : TRUE  = Walk candidate outerLoops in 3D to classify 
                                                      //      FALSE = Walk candidate outerLoops in 2D to classify 
                                                      //      default:[FALSE] 
  ) const;

  // Classify the Loop Properties of all FaceProperties using CurveClassify (an alternative algorithm to ComputeLoopOrientation)
  SmStatus ClassifyLoops
  (
    SmTArray<SmLoop*>          & rLoops,                  // in : TgtLoops List - empty=Classify all Loops
    SmHealNotYetFaceType       & reClassifiable,          // out: SM_HNF_NO_PROBS = SUCCESS - use the results  
                                                          //        SM_HNF_NO_CLASSIFY_ZERO_CROSSCNT = Problem Face - don't use the results 
                                                          //        SM_HNF_NO_CLASSIFY_ODD_CROSSCNT  = Problem Face - don't use the results 
    SmTArray<SmClassifyLoopIO> & rClassifyLoopIOs,        // out: associated LoopClassification containing: 
                                                          //    m_pLoop ;            // Classified Loop       
                                                          //    m_eOrient ;          // SM_OT_SAME     = CCW (expected Area_OuterLoop dir (for NoArea_Loops SM_OT_UPPERDOMAIN)) 
                                                          //                         // SM_OT_OPPOSITE = CW  (expected Area_InnerLoop dir (for NoArea_Loops SM_OT_LOWERDOMAIN)) 
                                                          //    m_bArea_Loop ;       // TRUE = Loop is Area_Loop (normal case)                                 
                                                          //                         // FALSE= Loop is NoArea_Loop - a problem only seen on MissingSeamFaces   
                                                          //    m_eContainmentType ; // oneof: SM_CMT_OUTERLOOP,       // Area_Loop not in another, contains all UVPoints 'inside' loop  
                                                          //                         //        SM_CMT_INNERLOOP,       // Area_Loop in another, excludes all UVPoints 'inside' loop
                                                          //                         //        SM_CMT_NESTEDLOOP_EVEN, // Area_Loop (invalid) nested in InnerLoop with an even nesting depth (0,2,4) - healed to outer loop
                                                          //                         //        SM_CMT_NESTEDLOOP_ODD,  // Area_Loop (invalid) nested in InnerLoop with an odd nesting depth (1,3,5) - healed to inner loop
                                                          //                         //        SM_CMT_BOTLOOP,         // NoArea_Loop (invalid) contains upperdomain UVPoints - AddMisingSeam heals to OuterLoop
                                                          //                         //        SM_CMT_TOPLOOP,         // NoArea_Loop (invalid) contains lowerdomain UVPoints - AddMisingSeam heals to OuterLoop
                                                          //                         //        SM_CMT_WIRELOOP,        // A set of connected Edgeuses without a closed portion.
    SmCrvDirOnFaceType         & reClassifyCrv_StartType, // out: ClassifyCrv StartPt classification pointing out from start of ClassifyCrv (not always a touch)
                                                          //        oneof: SM_CD_LAMINA SM_CD_SEAM SM_CD_POLE SM_CD_POLE_NO_VERTEX SM_CD_NONE
    SmCrvDirOnFaceType         & reClassifyCrv_StopType,  // out: ClassifyCrv StopPt classification pointing out from End of ClassifyCrv (not always a touch)
                                                          //        oneof: SM_CD_LAMINA SM_CD_SEAM SM_CD_POLE SM_CD_POLE_NO_VERTEX SM_CD_NONE
    SmTArray<ULONG>            & rOrderedNoAreaIndices,   // out: NoArea_loops Ordered from lowest domain to highest domain.             
    SmLooptreeItem            *& rpLooptreeRoot,          // out: LoopContainmentTree based solely on geometry relations, NULL on Input  
    SmCurveClassification      & rCrvClassification,      // in : scratch CurveClassification memory to save time 
    SmFaceProps                & rFaceProps               // i/o: in: FaceProps after running SetProps_Stage2() and after
                                                          //            and after SetProps_Stage3 has run SmLoopProps::SetProps on all FaceProps
                                                          //        out: Set the rFaceProps.m_sLoopProps[ii].Containment/Orientation properties
  ) const;                                                            

  // Compute face properties (area, delta volume properties) using numerical integration over trimmed boundaries
  SmStatus ComputePreciseProperties
  (
    SmOrientType           eOrientation,           // in : 
    double                 dDesiredAccuracy,       // in : 
    const SmPoint3d      & crOriginOfComputation,  // in : 
    double                 dEstimatedArea,         // in : 
    double                 dFaceThickness,         // in : If > 0.0 treat face as a thin solid with given face thickness. 
    double               & rdDeltaArea,            // out: 
    double               & rdDeltaVolume,          // out: 
    SmTArray<SmVector3d> & rDeltaMoments           // out: 
  ) const;

  // Select property groups in one integration; the overload above requests ALL.
  // Unrequested outputs are zero on success; rDeltaMoments always has 8 vectors:
  // AREA_FIRST -> [0], AREA_SECOND -> [1..3], VOLUME_FIRST -> [4], VOLUME_SECOND -> [5..7].
  // Groups are independent: for an area centroid request AREA | AREA_FIRST_MOMENTS.
  // The area-derived error-control integral is retained for every selection.
  // A zero mask or unknown flag bits returns SM_ERR_INVALID_INPUT without changing outputs.
  SmStatus ComputePreciseProperties
  (
    SmOrientType           eOrientation,           // in :
    double                 dDesiredAccuracy,       // in :
    const SmPoint3d      & crOriginOfComputation,  // in :
    double                 dEstimatedArea,         // in :
    double                 dFaceThickness,         // in : Same offset-face area semantics as full integration
    double               & rdDeltaArea,            // out: Zero unless AREA is requested
    double               & rdDeltaVolume,          // out: Zero unless VOLUME is requested
    SmTArray<SmVector3d> & rDeltaMoments,          // out: Unrequested moment groups are zero
    ULONG                  lPropertyFlags         // in : Nonempty combination of SmPrecisePropertyFlags
  ) const;

  // Compute face properties (area, delta volume properties) using triangular tessellation and summation
  SmStatus ComputeProperties
  (
    SmOrientType         eOrientation,            // in : faceuse orientation 
    double               dEdgeTessTol,            // in : tolerance to pass to the NLib tessellator 
    double               dFaceTessTol,            // in : tolerance to pass to the NLib tessellator 
    const SmPoint3d    & crOriginOfComputation,   // in : see usage notes 
    double             & rdDeltaArea,             // out: area of face 
    double             & rdDeltaMass,             // out: volume 'between' face and origin 
    SmPoint3d          & rDeltaBarycenter,        // out: centroid times delta volume 
    SmVector3d           aDeltaMoments[2],        // out: [0] Moments of inertia,  Ixx, Iyy, Izz 
                                                  //      [1] Products of inertia, Ixy, Iyz, Izx 
    SmMassPropertiesType eWhichProps = SM_MPT_ALL // in : see Usage Notes, above. 
  ) const;

  // create one composite curve for every loop
  SmStatus CreateComposites
  (
    const SmContext             & crContext,  // in : context for new object construction 
    SmTArray<SmCompositeCurve*> & rComposites // out: One composite curve for every loop 
  );

  // Create copies of given face curves trimmed to the portions in or on this face
  SmStatus CreateCurvesByTrimming
  (
    const SmContext          & crContext,               // in : context for new object construction 
    const SmApproxTol3d      * cpdOpt3DCurveTolerance,  // in : when given, should be the tol input curves were built to. 
                                                        //      notNULL: tol = max(Face->GetTol(),givenTol)
                                                        //      NULL   : tol = Face->GetTol()
    const SmTArray<SmCurve*> * cpOpt3DCurvesToTrim,     // in : 3D Curves to trim to face 
    const SmTArray<SmCurve*> * cpOptUVCurvesToTrim,     // in : associated UVCurves to trim to face 
    SmTArray<SmCurve*>       * cpOptNew3DCurves,        // out: Curves trimmed to face 
    SmTArray<SmCurve*>       * cpOptNewUVCurves,        // out: associated UVCurves trimmed to face 
    SmBoolean                  bForceUVClassify = FALSE // NotUsed: in : not used in this function 
  ) const;

  // create face boundary curve copies in format ready for input to MakeFaceFromCurves
  SmStatus CreateCurvesFromFace
  (
    const SmContext             & crContext,     // in : context for new object construction 
    SmOrientType                  eOrientation,  // in : use Faceuse whose orientation is the same as eOrientation 
    SmTArray<ULONG>             & rCurveLoops,   // out: number of edges in each edge loop 
    SmTArray<SmCurve*>          * pOpt3DCurves,  // out: optional copies of all 3D loop curves  
    SmTArray<SmBSplineCurve*>   * pOptUVCurves,  // out: optional copies of all 2D loop curves  
    SmTArray<SmOrientType>      & rCurveOrients, // out: Curve orientation in relation to its loop for each curve 
    SmTArray<SmPoint3d>         & rLoopPoints    // out: point position of each of face's vertex loops 
  ) const;

  // create face boundary curve copies in oriented format (ordered by loops, CCW for outer loop, CW for inner loops)
  SmStatus CreateOrientedTrimBoundary
  (
    const SmContext           & crContext,                                 // in : context for new object construction 
    SmBoolean                   bCreateParameterSpaceCurves,               // in : TRUE = Place copies of all Edge->UVTrimCurves in rTrimBoundary 
                                                                           //      FALSE= Place copies of all Edge->Curves       in rTrimBoundary
    SmBoolean                   bMakeExtraParameterCurvesForSingularities, // in : TRUE = Make extra UVTrimCurves to jump SurfaceSingularity parameter space gaps 
                                                                           //      FALSE= Don't; only used when bCreateParameterSpaceCurves == TRUE
    SmTArray<ULONG>           & rLoops,                                    // out: number of loops and number of curves in each loop 
    SmTArray<SmBSplineCurve*> & rTrimBoundary,                             // out: ordered: loop1_edges (counter-clockwise), ... loopnedges (clockwise) 
                                                                           //      malloc:[curves to be freed by caller]
    const SmLoop              * pLoopToUse = NULL                          // in : opt: if given, create boundary only for this loop. 
  ) const;

  // classify (in/out/unknown) a 3DPoint against the boundaries of a Trimmed Face
  SmStatus Point3DClassify
  (
    const SmPoint3d       & cr3DPointToClassify,      // in : 3dPoint to classify  
    SmZoneTol3d             sSrcZoneTol3d,            // in : Obj ZoneTol3d assoc with 3dPoint, if none, use:SmTol::GetZoneTol3d(Brep_Context_Or_NULL)  
    SmBoolean               bDoBoundaryIntersections, // in : TRUE = intersect point with Face boundaries  
                                                      //      before doing loop-containment tests.  
                                                      //      FALSE= skip intersections go straight to  
                                                      //      loop-containment tests.  
    SmPointClassification & rPointClassification,     // out: Point classification for this point. 
                                                      //      SM_PC_UNKNOWN = point outside of Face 
                                                      //      SM_PC_FACE    = point in Face 
                                                      //      SM_PC_EDGE    = point on Edge boundary of Face 
                                                      //      SM_PC_VERTEX  = point on Vertex boundary of Face 
    SmPoint2d             * pOptGuessUV = NULL,       // in : Optional Guess UV for Point Drop, NULL to ignore, default:[NULL]  
    SmBoolean               bTgtLoopsOnly=FALSE,      // in : TRUE           = only classify against pOptTgtLoops loops   
                                                      //        default:[FALSE]= classify against all Face->Loops           
    SmTArray<SmLoop*>     * pOptTgtLoops=NULL,        // in : When bTgtLoopsOnly == TRUE                                
                                                      //        default:[NULL or Size=0] = Only classify against the Face's OuterLoop
                                                      //        Size>0                   = Only classify against the these Loops
    SmBoolean               bUVSpaceOkay=TRUE         // in : TRUE = okay to try UVSpace Classification if 3Space classification is dodgey (tolerant cases) 
                                                      //      FALSE= don't use UVSpace because UVTrimCurves are not known to be valid, default:[TRUE] 
  ) const ;

  // classify (in or out) a UVPoint against the boundaries of a Trimmed Face - uses loop containment calc - fast but breaks with unhealed missing seam faces
  virtual SmStatus PointClassify
  (
    const SmPoint2d       & crUVPointToClassify,      // in : UVPoint to Classify 
    SmZoneTol3d             sSrcZoneTol3d,            // NotUsed: in : Obj ZoneTol3d assoc with UVPoint, not this face  
                                                      //     (if none, use: SmTol::GetZoneTol3d(Brep_Context_Or_NULL) 
    SmBoolean               bDoBoundaryIntersections, // NotUsed: in : TRUE = return point face->Vertex/Edge intersections when found 
                                                      //      FALSE= return loop-containment classifications. 
    SmBoolean               bUseUVClassification,     // in : No longer used 
    SmPointClassification & rPointClassification,     // out: Classification of the point. 
                                                      //      when in : m_ePointClass = SM_PC_FACE 
                                                      //           out: m_ePointClass = SM_PC_UNKNOWN 
                                                      //           on : m_ePointClass = SM_PC_EDGE or SM_PC_VERTEX 
    SmBoolean               bTgtLoopsOnly=FALSE,      // in : TRUE           = only classify against pOptTgtLoops loops   
                                                      //        default:[FALSE]= classify against all Face->Loops           
    SmTArray<SmLoop*>     * pOptTgtLoops=NULL,        // in : When bTgtLoopsOnly == TRUE                                
                                                      //        default:[NULL or Size=0] = Only classify against the Face's OuterLoop
                                                      //        Size>0                   = Only classify against the these Loops
    SmBoolean               bUVSpaceOkay=TRUE         // NotUsed: in : TRUE = okay to try UVSpace Classification if 3Space classification is dodgey (tolerant cases) 
                                                      //      FALSE= don't use UVSpace because UVTrimCurves are not known to be valid, default:[TRUE] 
   ) const ;

  // classify a curve known to be on this face into homogenous segments and point boundaries
  SmStatus CurveOnClassify
  (
    SmBoolean               bDoPointClassify,        // in : TRUE =Do expensive PtClassification for Curves not XSecting any Face->Bndrys 
                                                     //      FALSE=classify such points as unknown. (good when Curve is known to be InFace) 
    SmCurveClassification & rCurveClassification,    // i/o: contains m_cpCurve to classify, accumulates ivls as they are found. 
    SmBoolean               bHasBigGaps=FALSE,       // in : TRUE = m_cpCurve and Face->Edges connect to Face where Edge/Face gaps are suspected to exceed XSectTol3d. 
                                                     //           increase XSectTol3d to accomodate large Edge/Face gaps 
                                                     //           (more expensive option - computes Face/Surface gaps) 
                                                     //      FALSE= Expect all Face/Edge gaps to be within XSectTol3d, use standard SmTol::GetXSectTol3d tols. 
                                                     //      default:[FALSE] 
    SmBoolean               bDoCheck3d=FALSE,        // in : TRUE = When finding XSectPoints, also check 3d dists 
                                                     //      FALSE= When finding XSectPoints, only check 2d dists for XSects 
                                                     //      default:[FALSE] 
    SmBoolean             * pbOptModifiedCurve=NULL, // out: TRUE  = m_cpCurve modified to tighten tols 
                                                     //      FALSE = m_cpCurve not modified 
                                                     //      When m_cpCurve is modified, the classification is cleared and 
                                                     //      rebuilt from scratch - if accumulating classifications from   
                                                     //      a sequence of calls, start over. Call CrvClassif->ReSet() and restart sequence. 
                                                     //      If this is the only classification call, then CrvClassif is complete 
                                                     //      and may be used as returned. 
                                                     //      NULL to ignore, default:[NULL] 
    SmBoolean               bUVSpaceOkay=TRUE        // in : TRUE = okay to try UVSpace Classification if 3Space classification is dodgey (tolerant cases) 
                                                     //      FALSE= don't use UVSpace because UVTrimCurves are not known to be valid, default:[TRUE] 
   ) const;                                          


  // classify an edge known to be on the face's surface by intersecting it 
  // with this face's vertices and edges.
  SmStatus EdgeCurveOnClassify
  (
    const SmEdge      * cpEdge,             // in : edge intersected by curve 
    double              dEdgeParam,         // in : parameter of edge/curve intersection 
    const SmCurve     & crCurve,            // in : curve to classify 
    const SmExtent1d  & crCurveInterval,    // in : range of interest of curve 
    SmOrientType        eCurveOrientation,  // in : SM_OT_SAME     = interested in CurveSegment after dEdgeParam 
                                            //      SM_OT_OPPOSITE = interested in CurveSegment before dEdgeParam 
    double              d3dCurveTolerance,  // in : 
    SmBoolean         & rbIsInsideFace,     // out: TRUE=Inside Face, FALSE=outside face
    SmBoolean         & rbPossibleProblem,  // out: TRUE=Classification Problem, FALSE=Classification is solid 
    SmClassifyProblem & reClassifyProblem,  // out: set when rbPossibleProblem is TRUE to oneof 
                                            //      SM_CP_NONE, 
                                            //      SM_CP_VERTEX_ON_SINGULARITY,   SM_CP_SEESAW_CLASSIFICATION,     
                                            //      SM_CP_TOGGLED_CLASSIFICATION,  SM_CP_UNRESOLVED_CLASSIFICATION, 
                                            //      SM_CP_SHORT_SECTOR_EDGEUSE1,   SM_CP_VERTEX_ON_SEAM            
                                            //      SM_CP_SHORT_SECTOR_EDGEUSE2, 
                                            //      SM_CP_SHORT_INTERVAL, 
    SmBoolean           bUVSpaceOkay = TRUE,      // in : TRUE = okay to try UVSpace Classification if 3Space classification is dodgey (tolerant cases) 
                                            //      FALSE= don't use UVSpace because UVTrimCurves are not known to be valid, default:[TRUE] 
    ULONG               lFaceIndx=SM_UNDEF_ULONG  // in : When rFaceProps is available to caller, pass rFaceProps->m_lFaceIndx to debug a target face
                                                  //      SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_ULONG]
  ) const ;
   
  // classify curve-segment known to be on this face using Edgeuse crossing data
  SmStatus SectorClassifyCurveOn
  (
    const SmEdgeuse   * cpEU1,                    // in : 1st Face->Edgeuse crossed by curve 
    double              dEU1Param,                // in : edge param of curve/edge intersection 
    SmZoneTol3d         sEdge1ZoneTol3d,          // in : ZoneTol3d(pEU1->Edge) 
    SmOrientType        eEU1Orientation,          // in : SM_OT_SAME     - Sector bounded by pos moving edge 
                                                  //      SM_OT_OPPOSITE - Sector bounded by neg moving edge 
    const SmEdgeuse   * cpEU2,                    // in : 2nd Face->Edgeuse crossed by curve 
    double              dEU2Param,                // in : edge param of curve/edge intersection 
    SmZoneTol3d         sEdge2ZoneTol3d,          // in : ZoneTol3d(pEU2->Edge) 
    SmOrientType        eEU2Orientation,          // in : SM_OT_SAME     - Sector bounded by pos moving edge 
                                                  //      SM_OT_OPPOSITE - Sector bounded by neg moving edge 
    const SmCurve     & crCurve,                  // in : a 2d/3d curve known to be on this face to classify 
    const SmExtent1d  & crCurveInterval,          // in : interval with an end bounded by the Edgeuses 
    SmOrientType        eCurveOrientation,        // in : SM_OT_SAME     - classify curve at Min End moving in pos dir 
                                                  //      SM_OT_OPPOSITE - classify curve at Max End moving in neg dir 
    SmZoneTol3d         sCurveZoneTol3d,          // in : ZoneTol3d(Obj Assoc to crCurve)   
                                                  //      (e.g from Edge:[ZoneTol3d(Edge)], from Surf/Surf XSect:[Max(ZoneTol3d(Surf1),ZoneTol3d(Surf2))] 
    SmBoolean           bIsOnSingularityPoint,    // in : TRUE=point is singular - typically top of sphere or cone. 
    SmPoint2d           sUVofSingularity,         // in : SurfacePoint of singularity 
    SmBoolean         & rbIsInsideThisFaceSector, // out: TRUE = classified segment is in FACE, FALSE=NOT 
    SmBoolean         & rbPossibleProblem,        // out: TRUE = probs and reClassifyProblem is set, FALSE=a good classification 
    SmClassifyProblem & reClassifyProblem,        // out: set when rbPossibleProblem is TRUE to oneof 
                                                  //      SM_CP_NONE, 
                                                  //      SM_CP_VERTEX_ON_SINGULARITY,  SM_CP_SEESAW_CLASSIFICATION, 
                                                  //      SM_CP_TOGGLED_CLASSIFICATION, SM_CP_UNRESOLVED_CLASSIFICATION, 
                                                  //      SM_CP_SHORT_SECTOR_EDGEUSE1,  SM_CP_VERTEX_ON_SEAM             
                                                  //      SM_CP_SHORT_SECTOR_EDGEUSE2, 
                                                  //      SM_CP_SHORT_INTERVAL, 
    SmBoolean           bUVSpaceOkay = TRUE,      // in : TRUE = okay to try UVSpace Classification if 3Space classification is dodgey (tolerant cases) 
                                                  //      FALSE= don't use UVSpace because UVTrimCurves are not known to be valid, default:[TRUE] 
    ULONG               lFaceIndx=SM_UNDEF_ULONG  // in : When rFaceProps is available to caller, pass rFaceProps->m_lFaceIndx to debug a target face
                                                  //      SM_UNDEF_ULONG to ignore, default:[SM_UNDEF_ULONG]
  ) const ;

  // classify (in or out) curve-segment known to be on this face that crosses a vertex
  SmStatus FindVertexSector
  (
    const SmVertex    * cpVertex,                // in : Face->Vertex crossed by curve 
    const SmCurve     & cr3DCurve,               // in : 3d curve being classified 
    const SmCurve     * cpUVCurve,               // in : optional associated UV curve 
    double              dCurveParameterAtVertex, // in : curve parameter at vertex crossing 
    double              dCurveParameterIvlEnd,   // in : limit on curve parameter to be examined for classification 
    SmOrientType        eCurveOrientation,       // in : Direction of curve to test. 
                                                 //      SM_OT_SAME     = classify curve segment starting at param 
                                                 //                        going forward along the curve. 
                                                 //      SM_OT_OPPOSITE = classify curve segment starting at param 
                                                 //                       going backward along the curve. 
    SmZoneTol3d         sZoneTol3d,              // in : ZoneTol3d(Obj Assoc to cr3DCurve)  
                                                 //      (e.g from Edge : [ZoneTol3d( Edge )], from Surf / Surf XSect : [Max( ZoneTol3d( Surf1 ), ZoneTol3d( Surf2 ) )] 
    SmBoolean           bValidateSector,         // in : TRUE  = test all sectors. 
                                                 //      FALSE = optimize if only one possible sector is found. optimize if the curve is known to be in the face. 
    SmVertexuse      *& rpVUOfSector,            // out: NotNULL = Curve is inside face sector defined by vertexuse on upward faceuse 
                                                 //      NULL    = cr3DCurve is not inside the face. 
    SmBoolean         & rbPossibleProblem,       // out: TRUE = Answer can't be trusted for important decisions. 
                                                 //      Either tight curvatures combined with endPoint tolerances make classification difficult or  
                                                 //      A curve was too short to supply a sample point far enough away from tolerances to make a clean classification  
    SmClassifyProblem & reClassifyProblem,       // out: set when rbPossibleProblem is TRUE to oneof 
                                                 //      SM_CP_NONE, 
                                                 //      SM_CP_VERTEX_ON_SINGULARITY,   SM_CP_SEESAW_CLASSIFICATION,     
                                                 //      SM_CP_TOGGLED_CLASSIFICATION,  SM_CP_UNRESOLVED_CLASSIFICATION, 
                                                 //      SM_CP_SHORT_SECTOR_EDGEUSE1,   SM_CP_VERTEX_ON_SEAM             
                                                 //      SM_CP_SHORT_SECTOR_EDGEUSE2, 
                                                 //      SM_CP_SHORT_INTERVAL  
    SmBoolean           bUVSpaceOkay = TRUE      // in : TRUE = okay to try UVSpace Classification if 3Space classification is dodgey (tolerant cases) 
                                                 //      FALSE = don't use UVSpace because UVTrimCurves are not known to be valid, default:[TRUE] 
  ) const;

  // set rbIsFillet: TRUE = Face could be a fillet based on geometric checks, FALSE = can't
  SmStatus IdentifyFillet
  ( 
    double              d3DTolerance,          // in : Used for many computations 
    double              dTangencyTolDeg,       // in : Must be tangent to this angle 
    SmBoolean           bRequireTangent,       // in : TRUE = allow only tangent rail edges. 
    SmBoolean           bRequireManifold,      // in : TRUE = allow only manifold rail edges. 
                                               //      if we are going to consider it for a fillet. 
    SmBoolean         & rbIsFillet,            // out: TRUE = Surface has a filletCrossSection 
                                               //      FALSE=  
    ULONG             & rlFilletCrossSection,  // out:  Fillet's crossSection Shape 0 - circular,  
                                               //                                   1 - Approx Circular,  
                                               //                                   2 - Linear  
                                               //                                   3 - G1 Blend,  
                                               //                                   4 - G2 Blend, 
                                               //                                   5 - G3 Blend, 
                                               //                                   6 - Other. 
    ULONG             & rlFilletSolverType,    // out:  Fillet's length behavior    0 - constant radius 
                                               //                                   1 - constant distance, 
                                               //                                   2 - variable radius 
    SmSurfParamType   & reFilletRailDirection, // out: see Usage Notes. 
    double            & rdRadius,              // out: Radius, if CrossSection is 0 or 1. 
    double            & rdDist,                // out: Cross section distance, if SolverType is 1. 
    SmTArray<SmEdge*> & rRailEdgesMin,         // out: Edges that are rails on the low isoparameter 
    SmTArray<SmEdge*> & rRailEdgesMax,         // out: Edges that are rails on the high isoparameter 
    SmExtent1d        * pMinMaxRadii = NULL    // out: [optional; default NULL] min and max radii, if SolverType is 2 
  ) const;

  // set rbIsFillet: TRUE = Face could be a fillet in the indicated direction
  SmStatus IdentifyFilletUOrV
  ( 
    SmSurfParamType     eFilletRailDirection,  // in : see Usage Notes. 
    double              d3DTolerance,          // in : Used for many computations 
    double              dTangencyTolDeg,       // in : Must be tangent to this angle 
                                               //      if we are going to consider it for a fillet. 
    SmBoolean           bRequireTangent,       // in : TRUE = allow only tangent rail edges. 
    SmBoolean           bRequireManifold,      // in : TRUE = allow only manifold rail edges. 
    SmBoolean         & rbIsFillet,            // out: TRUE = Surface has a filletCrossSection 
    ULONG             & rlFilletCrossSection,  // out: Fillet's crossSection Shape 0 - circular, 
                                               //                                  1 - Approx Circular, 
                                               //                                  2 - Linear 
                                               //                                  3 - G1 Blend, 
                                               //                                  4 - G2 Blend, 
                                               //                                  5 - G3 Blend 
                                               //                                  6 - Other. 
    ULONG             & rlFilletSolverType,    // out: Fillet's length behavior    0 - constant radius 
                                               //                                  1 - constant distance, 
                                               //                                  2 - variable radius 
    double            & rdRadius,              // out: Radius, if CrossSection is 0 or 1. 
    double            & rdDist,                // out: Cross section distance, if SolverType is 1. 
    SmTArray<SmEdge*> & rRailEdgesMin,         // out: Edges that are rails on the low isoparameter 
    SmTArray<SmEdge*> & rRailEdgesMax,         // out: Edges that are rails on the high isoparameter 
    SmExtent1d        * pMinMaxRadii = NULL    // out: [optional; default NULL] min and max radii, if SolverType is 2 
  ) const;

  // Extract Fillet information for this face
  SmStatus ExtractFillet
  (
    const SmContext           & crContext,             // in : context for new object construction 
    double                      d3DTolerance,          // in : Used for many computations 
    double                      dTangencyTolDeg,       // in : Must be tangent to this angle 
                                                       //      if we are going to consider it for a fillet. 
    SmBoolean                 & rbIsFillet,            // out: TRUE = Surface has a filletCrossSection 
                                                       //      FALSE= 
    ULONG                     & rlFilletCrossSection,  // out: Fillet's crossSection Shape 0 - circular, 
                                                       //                                  1 - Approx Circular, 
                                                       //                                  2 - Linear 
                                                       //                                  3 - G1 Blend, 
                                                       //                                  4 - G2 Blend, 
                                                       //                                  5 - G3 Blend 
    ULONG                     & rlFilletSolverType,    // out:  Fillet's length behavior   0 - constant radius 
                                                       //                                  1 - constant distance, 
                                                       //                                  2 - variable radius 
    SmSurfParamType           & reFilletRailDirection, // out: see Usage Notes. 
    SmBoolean                 & rbNormalIsOutward,     // out: TRUE = surface normal points in the convex direction. 
                                                       //      FALSE= 
    SmTArray<SmBSplineCurve*> & rRailCurves,           // out: 
    SmTArray<SmFace*>         & rRailFaces,            // out: 
    double                    * pdRadius = NULL,       // out: [optional, default NULL]: radius, if constant radius 
    double                    * pdDist = NULL          // out: [optional, default NULL]: cross-section distance, if const distance 

  ) const;

  // intersect two faces and return resulting 3d and UV intersection curves, singular points, and classifications
  SmStatus FaceIntersect
  ( 
    const SmContext                  & crContext,                  // in : context for new object construction 
    SmFace                           * pOtherFace,                 // in : other target face 
    SmApproxTol3d                      dApproxTol3d,               // in : distance Tolerance used in surf/surf intersection 
    double                             dCrvAngTolDeg,              // in : Angle Tolerance in degrees 
    SmTArray<SmCurve*>               & r3DCurves,                  // out: all 3DCurves of intersection between the two faces 
    SmTArray<SmCurve*>               * pOptThisUVCurves = NULL,    // out: optional thisFace UVTrimCurves for each intersection 3DCurve, NULL to ignore 
    SmTArray<SmCurve*>               * pOptOtherUVCurves = NULL,   // out: optional otherFace UVTrimCurves for each intersection 3DCurve, NULL to ignore 
    SmTArray<SmPoint3d>              * pOpt3DPoints = NULL,        // out: optional single-point intersections
    SmTArray<SmCurveClassification*> * pOptThisCrvClasses = NULL,  // out: optional thisFace  curveClassifications for all face/face intersections 
    SmTArray<SmCurveClassification*> * pOptOtherCrvClasses = NULL, // out: optional otherFace curveClassifications for all face/face intersections 
    SmTArray<SmCurve*>               * pOptCC3DCrvs = NULL,        // out: optional 3D Curves used by OptCCs       for all face/face intersections 
    SmTArray<SmCurve*>               * pOptThisCCUVCrvs = NULL,    // out: optional UV Curves used by OptThisCCs   for all face/face intersections 
    SmTArray<SmCurve*>               * pOptOtherCCUVCrvs = NULL,   // out: optional UV Curves used by OptOtherCCs  for all face/face intersections 
    SmBoolean                        * pbOptCoincidenceFlagv = NULL// in : opt: If the Surfaces are known to be coincident (or not) 
  );

  // intersect a line with a face - same I/O as SmSurface::GlobalLineIntersect()
  SmStatus GlobalLineIntersect
  ( 
    const SmExtent2d & crUVDomain,        // in : Surface domain to intersect 
    const SmPoint3d  & crLinePoint,       // in : Point on the infinite line 
    const SmVector3d & crLineVector,      // in : Direction vector of the infinite line 
    const SmExtent1d * cpOptLineInterval, // in : If specified, bounds the line to a specific segment 
    SmBoolean          bFireRay,          // in : If TRUE specifies that the line is bounded only at the 
                                          //      start point and proceeds along the vector to infinity.
    SmZoneTol3d        sSrcZoneTol3d,     // in : Obj ZoneTol3d assoc with Line, If none, use:SmTol::GetZoneTol3d(Context_Brep_Or_NULL) 
    SmSolutionArray  & rSolutions         // out: Solution array 
  ) const;
 
  // remove UVTrimCurves from face->Edgeuses
  SmStatus RemoveUVTrimCurves() ;

  // create or attach UVTrimCurves to face->Edgeuses, optionally
  // increase face->edge and face->vertex tolerances to include measured gaps 
  #define CreateTrimCurves CreateUVTrimCurves
  SmStatus CreateUVTrimCurves
  (
    SmBoolean bAdjustTolerances,                             // in : TRUE = adjust Edge and Vertex tolerances to include measured gaps 
    const SmTArray<SmBSplineCurve*> * cpOptUVCurves,         // in : optional array of UVTrimCurves to assign to Face->Edgeuses cpOptOrientations must be notNULL. 
    const SmTArray<SmOrientType>    * cpOptOrientations,     // in : associated orientations. If cpOptUVCurves notNULL, then 
    double                          & rdMeanCurveSurfaceGap, // out: average new UVTrimCurve/Surface distance 
    double                          & rdMaxCurveSurfaceGap,  // out: max     UVTrimCurve/Surface distance 
    double                          & rdMean3DVertexGap,     // out: average Vertex/EdgeuseEndPoint distance 
    double                          & rdMax3DVertexGap,      // out: max     Vertex/EdgeuseEndPoint distance 
    double                          & rdMeanPSCurveCurveGap, // out: average UVTrimCurve/CCW_UVTrimCurve 2D gap 
    double                          & rdMaxPSCurveCurveGap   // out: max     UVTrimCurve/CCW_UVTrimCurve 2D gap 
  ) const;
    
  // insert and extract loops from face. 
  SmStatus InsertLoop     (SmLoop * pLoopToInsert); // 1st insert is outer loop, subsequent inserts are inner loops.
  SmStatus RemoveLoop     (SmLoop * pLoopToRemove); // don't delete pLoopToRemove

  // Delete a lamina inner loop and everything associated with it from this Face.
  SmStatus DeleteInnerLoop( SmLoop * pLoopToDelete );

  // fill gaps around a vertex using small edges or curve extensions
  SmStatus FillVertexGaps();

  // flip face orientation, i.e. reverse the face normals
  // prefer SwapUV: flip orientation bits and reverse loop orders.
  SmStatus FlipFaceOrientation(); 

  // only for SmBSplineSurfaces: swap surface UV and corresponding UV Curves.
  // thus flipping face normal. see also SmBrep::OrientTrimmedSurfaces(). 
  SmStatus SwapUV();
                                  
// Remove Composites
//  // shrink surface domain to boundary geometry, i.e. reduce control point counts
//  // and shrink curves to edge extents, passes to SmCFace::ShrinkGeometry() when needed.
//  SmStatus ShrinkGeometry();
  
  // shrink surface domain to bndry geometry, i.e. reduce control point counts & shrink curves to edge extents
  SmStatus ShrinkGeometry();

  // only for nonComposite faces.  Prefer ShrinkGeometry().  
  SmStatus ShrinkSurface();                         

  // simple data access
  virtual SmBrep    * GetBrep              () const ;
  SmAObject         * GetAOwner            () const { return ((SmAObject*)GetBrep()); } // get attribute inheritance owner
  ULONG               GetFaceNumberInBrep  () const ;
  SmSurface         * GetSurface           () const { SM_ASSERT_BREAK(m_pSurface != NULL) ; return m_pSurface ; }
  SmExtent2d          GetUVDomain          () const { return m_vUVDomain; }
  SmBoolean           GetRectangularTrim   () const { return m_bRectangularTrim; } // TRUE = Face has 1 UVRectangle OuterLoop of 4 isoUVTrimCurves with extents m_vUVDomain
// Remove Composites
//   SmCFace           * GetCompositeFaceOwner() const ; 
  
  // Loop selection shared by GetLoops(), GetEdges(), and GetVertices():
  //   bTgtLoopsOnly == FALSE:
  //     pOptTgtLoops is ignored.  All loops are visited in the primary/upward
  //     Faceuse's list order, with the outer loop first.
  //   bTgtLoopsOnly == TRUE and pOptTgtLoops is NULL or empty:
  //     Only the outer loop is selected.
  //   bTgtLoopsOnly == TRUE and pOptTgtLoops is nonempty:
  //     Matching loops are visited in pOptTgtLoops order, not Face list order.
  //     Consequently, the outer loop need not be present or first.  Loops from
  //     other Faces are ignored.
  // Nonempty target arrays must contain nonNULL, live SmLoop pointers.
  virtual void        GetLoops             (SmTArray<SmLoop*>      & rLoops,                               SmBoolean bTgtLoopsOnly=FALSE, const SmTArray<SmLoop*> *pOptTgtLoops=NULL) const ;
  virtual void        GetEdges             (SmTArray<SmEdge*>      & rEdges,      ULONG * pOptAttribId=NULL, SmBoolean bTgtLoopsOnly=FALSE, const SmTArray<SmLoop*> *pOptTgtLoops=NULL) const ;
  virtual void        GetVertices          (SmTArray<SmVertex*>    & rVertices,   ULONG * pOptAttribId=NULL, SmBoolean bTgtLoopsOnly=FALSE, const SmTArray<SmLoop*> *pOptTgtLoops=NULL) const ;
  virtual void        GetEdges             (std::unordered_set<SmEdge*>& rEdges, ULONG* pOptAttribId = NULL, SmBoolean bTgtLoopsOnly = FALSE, const SmTArray<SmLoop*>* pOptTgtLoops = NULL) const;
  virtual void        GetVertices          (std::unordered_set<SmVertex*>& rVertices, ULONG* pOptAttribId = NULL, SmBoolean bTgtLoopsOnly = FALSE, const SmTArray<SmLoop*>* pOptTgtLoops = NULL) const;
  /* note:  to        GetTypeuse()         - See the GetUpwardTypeuse() methods below */
//  SmFaceProps       * GetFaceProps()       { return( m_pFaceProps) ; }

  // Convenience queries equivalent to the corresponding getter with
  // bTgtLoopsOnly == TRUE and pOptTgtLoops == NULL.  Edge and Vertex results
  // contain unique objects from the primary/upward Faceuse's outer loop; they
  // do not preserve repeated Edgeuse occurrences or expose their orientation.
  void GetOuterLoops        (SmTArray<SmLoop*>      & rLoops)                               const { GetLoops   (rLoops,    TRUE, NULL) ; } // out: one Loop or one Loop per face for CFaces
  void GetOuterLoopEdges    (SmTArray<SmEdge*>      & rEdges,    ULONG * pOptAttribId=NULL) const { GetEdges   (rEdges,    pOptAttribId, TRUE, NULL) ; }
  void GetOuterLoopVertices (SmTArray<SmVertex*>    & rVertices, ULONG * pOptAttribId=NULL) const { GetVertices(rVertices, pOptAttribId, TRUE, NULL) ; }

  virtual void        GetFaceuses          (SmFaceuse             *& rpFaceuse1, SmFaceuse *& rpFaceuse2) const ;
  virtual SmFaceuse * GetUpwardFaceuse     () const ;              
  virtual void        GetUpwardFaceuses    (SmTArray<SmFaceuse*>   & rFaceuses) const ; 
  virtual void        GetUpwardLoopuses    (SmTArray<SmLoopuse*>   & rLoopuses,   SmBoolean bTgtLoopsOnly=FALSE, const SmTArray<SmLoop*> *pOptTgtLoops=NULL) const ;
  virtual void        GetUpwardVertexuses  (SmTArray<SmVertexuse*> & rVertexuses, SmBoolean bTgtLoopsOnly=FALSE, const SmTArray<SmLoop*> *pOptTgtLoops=NULL) const ;
  virtual void        GetUpwardEdgeuses    (SmTArray<SmEdgeuse*>   & rEdgeuses,   SmBoolean bTgtLoopsOnly=FALSE, const SmTArray<SmLoop*> *pOptTgtLoops=NULL) const ;

  // adds MicroColor attr if needed before returning
  SmVector3d          GetOrCreateMicroColor() ;

  // returns NULL for No MicroColor attr
  const SmVector3d  * GetMicroColor        () const ;

  SmStatus            GetPointsInFace(ULONG                lPointCount,                      
                                      SmTArray<SmPoint2d> &rUV,               
                                      SmTArray<SmPoint3d> &rPV) const ;

  void         SetUVDomain          (const SmExtent2d & crNewUVDomain)   { m_vUVDomain = crNewUVDomain; }
  void         SetIsRectangularTrim (SmBoolean bRectangularTrim)         { m_bRectangularTrim = bRectangularTrim; }
  
  // Change Face->Surface, see also SmBrep::ReplaceSurface(), uses Notify() to clear Edgeuse and Vertexuse cached data
  virtual void SetSurface(SmSurface * pSurface,                // in : Surface to save            
                          SmBoolean   bDeleteCurrFace=FALSE) ; // in : TRUE = delete NonNULL current pSurface, FALSE=don't 
                                                               //      default:[FALSE] previous behavior

  // Use with caution.  Can be used to set m_pFU to Null during deletion.
  void         SetFaceuse (SmFaceuse * pNewFU)  { m_pFU = pNewFU; } 

  // predicates
// Remove Composites
//  SmBoolean         IsCompositeFace   () const ; // this Face is part of a composite face (returns FALSE for SmCFace objects)
  // With bSkipQuickCheck == FALSE, IsDegenerate is logically const but lazily
  // creates any missing edgeuse UV trim curves (via NormalizedEvaluate /
  // PointClassify); existing curves are not changed. That path is not safe to call
  // concurrently on a shared Brep.
  SmBoolean         IsDegenerate      (SmBoolean bSkipQuickCheck = FALSE, SmZoneTol3d * pOptZoneTol3d = NULL) const ; // all boundaries are within tolerance of other boundaries
  SmBoolean         IsLamina          () const ; // rtn TRUE when all bounding edges are laminar
  SmBoolean         IsSheet           () const ; // rtn TRUE when Face->Upper and Lower Faceuse->Regions are the same
  SmBoolean         HasLaminaEdge     () const ; // rtn TRUE when any bounding edge  is  laminar
  SmBoolean         HasStrutEdge      () const ; // rtn TRUE when any bounding edge  is  a strut
  SmBoolean         HasConsecutiveLaminaEdges () const ; // rtn TRUE when two consecutive bounding edges are laminar
  virtual SmBoolean IsConnectedTo     (const SmTopology *cpConnectTgt) const ;

  // Check for 1 rectangular four-IsoParam-UVTrimCurve outer loop defined by output rUVDomain
  SmStatus FindIfRectangularTrim(SmExtent2d & rUVDomain,          // out: uninit except when rbRectangularTrim == TRUE, UVDomain = UVRectangle of loop's 4 isoUVTrimCurves
                                                                  //                    when rbNaturalTrimm    == TRUE, UVDomain = FaceSurface->NaturalBoundaries
                                  SmBoolean  & rbRectangularTrim, // out: TRUE = Face has 1 UVRectangle OuterLoop of 4 isoUVTrimCurves with extents rUVDomain
                                  SmBoolean  & rbNaturalTrim)     // out: TRUE = Face UVRectangle bndry == Face->Surface->NaturalBoundaries.
                                 const ;
           
  SmBoolean         IsZoneTol3dConsistent(SmZoneTol3d * pOptZoneTol3d=NULL) const ; // TRUE = SM_ARE_SAME(m_sZoneTol3d == SmTol::GetZoneTol3d(this,0,TRUE))
 
  SmBoolean         HasCoincidentVertices( SmTArray<SmVertex *>  * pOptToSurviveVerts=NULL,
                                           SmTArray<SmVertex *>  * pOptToDeleteVerts=NULL
                                         ) const ;
 
  SmBoolean         HasCoincidentEdges   ( SmTArray<SmEdge *>    * pOptToSurviveEdges=NULL, // out: List of Edges with coincident partners, NULL to ignore, default:[NULL] 
                                           SmTArray<SmEdge *>    * pOptToDeleteEdges=NULL,  // out: associated coincident partner, NULL to ignore, default:[NULL]
                                           SmTArray<SmVertex *>  * pOptToSurviveVerts=NULL, // NotUsed: out: Opt HasCoincidentVertices() output, List of Verts with coincident partners, NULL to ignore, default:[NULL]
                                           SmTArray<SmVertex *>  * pOptToDeleteVerts=NULL   // NotUsed: out: Opt HasCoincidentVertices() output, associated coincident partner, NULL to ignore, default:[NULL]
                                         ) const ;

  // rtn TRUE When A Seam is missing or an Edge crosses a Seam without a vertex
  // note: HasSeamProblem() output is made to be used as SplitAtSeam() input.
  SmBoolean HasSeamProblem 
  (
    SmCurveClassification & rCrvClassU,     // i/o: SeamConstU/Face Classification, const U isoParameterCurve 
    SmCurveClassification & rCrvClassV,     // i/o: SeamConstV/Face Classification, const V isoParameterCurve 
    SmSurfParamType       & reCrossedSeam,  // out: SM_SP_U       = periodic U bndry seam crossed by Face->Edges  
                                            //      SM_SP_V       = periodic V bndry seam crossed by Face->Edges 
                                            //      SM_SP_BOTH    = both periodic U and V bndry seams crossed by Face->Edges 
                                            //      SM_SP_NEITHER = no seams crossed by Face->Edges 
    SmSurfParamType       & reMissingSeam,  // out: SM_SP_U       = periodic U bndry seam not represented by Face->Edges 
                                            //      SM_SP_V       = periodic V bndry seam not represented by Face->Edges 
                                            //      SM_SP_BOTH    = both periodic U and V bndry seams not represented by Face->Edges 
                                            //      SM_SP_NEITHER = both periodic U and V bndry seams are represented by Face->Edges 
    SmSurfParamType       & reNearMissSeam, // out: SM_SP_U       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic U BndrySeam 
                                            //      SM_SP_V       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic V BndrySeam 
                                            //      SM_SP_BOTH    = NearMiss for both U and V periodic BndrySeams 
                                            //      SM_SP_NEITHER = No NearMiss CoincidentEdges or Vertices for BndrySeams 
                                            //      NearMiss Topo connections are valid
    SmBoolean         bOuterLoopOnly=FALSE, // in : TRUE = Only classify against the Face outer loop, used internally on partially built faces 
                                            //      default:[FALSE] = classify against all Face->Loops 
    SmBoolean       * pOptPeriodicU=NULL,   // in : TRUE=known to be U periodic, FALSE=not, NULL to ignore 
    SmBoolean       * pOptPeriodicV=NULL,   // in : TRUE=known to be V periodic, FALSE=not, NULL to ignore 
    SmFaceProps     * pOptFaceProps=NULL    // in : optional pre-cached set of this-Face properites, NULL to ignore
  ) const ;

  // :--------------------------------------------------------------------------------------------:
  // :                          UpDim, DownDim, and SameDim Gaps                                  :
  // :----------+--------------------------+---------------------------+--------------------------+            
  // : Topology : UpDim Gap types          : DownDim Gap types         : SameDim Gap types        :      
  // :----------+--------------------------+---------------------------+--------------------------+       
  // :  Vertex  : Vertex/Edge, Vertex/Face : none                      : none                     :       
  // :  Edge    : Edge/Face                : Vertex/Edge               : none                     :       
  // :  Face    : none                     : Vertex/Face, Edge/Face    : EveryLoop: MaxLoopGap    :       
  // :  Loop    : none                     : none                      : EdgeEnd/EdgeEnd LoopGaps :       
  // :----------+--------------------------+---------------------------+--------------------------+   
  //        MaxGap3d     = Max( Max(UpDimGap), Max(DownDimGap) )  - does not include LoopGaps
  //        MaxLoopGap3d = Max( LoopGaps )
  virtual const SmGap * GetMaxGap3d       (SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const  { return( GetMaxDownDimGap3d(pOptGapArray, pOptTol3d) ) ; }
          const SmGap * GetMaxLoopGap3d   (SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const  { return( GetMaxSameDimGap3d(pOptGapArray, pOptTol3d) ) ; }
  virtual const SmGap * GetMaxDownDimGap3d(SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const  ; // max Vertex/Face, Edge/Face, or Edge/FaceTrimCurve gap
  virtual const SmGap * GetMaxSameDimGap3d(SmGapArray * pOptGapArray=NULL, SmTol3d * pOptTol3d=NULL) const  ; // max Vertex/Face, Edge/Face, or Edge/FaceTrimCurve gap

  SM_OLDTOL_LINE // local tolerance management
  SM_OLDTOL_LINE virtual SmZoneTol3d GetTolerance() const     { return(m_sZoneTol3d) ; }   // old Tol ZoneTol3d stored on the object
  SM_OLDTOL_LINE virtual void        SetTolerance    (SmZoneTol3d sNewZoneTol3d, 
                                                      SmBoolean bUpdateOnlyIfLarger=TRUE,
                                                      SmBoolean bCascadeToBndries=TRUE) ;
  SM_OLDTOL_LINE SmStatus            RefreshTolerance(SmBoolean bNestedRefreshes=TRUE) ;  // Assign Edge and Faces connected to Edge consistent tolerance values, // NotUsed: in :

  SM_NEWTOL_LINE // local tolerance management 
  SM_NEWTOL_LINE //   inherited from SmTopology 
  SM_NEWTOL_LINE //     tolerance model: for pinholes in battleships (rarely used) - default:[FALSE] (99.9% of the time - leave it that way)
  SM_NEWTOL_LINE //        SmBoolean   IsSmallTopology() const ;
  SM_NEWTOL_LINE //        void        SetIsSmallTopology(SmBoolean bIsSmall) ;
  SM_NEWTOL_LINE //        
  SM_NEWTOL_LINE //     tolerance model: obsolete old-style compatible - instead use SmTol::GetZoneTol3d(this) ; 
  SM_NEWTOL_LINE //        SmZoneTol3d GetTolerance() const                   { return SmTol::GetZoneTol3d(this) ; } // GWC: obsolete
  SM_NEWTOL_LINE //        void        SetTolerance(SmZoneTol3d, SmBoolean)   { /* no action - only for backward compatibility */ ; }

  // When Face->Surface is periodic and Face->OuterEdges XSect Seam, move Seam to better location - calls SetSurface() which removes Edgeuse and Vertexuse Cached data
  SmStatus MoveSeam                                   // [rtn]: SM_SUCCESS = okay, SM_ERR_INVALID_INPUT = input face is member of a CompositeFace
  (                                                   
    SmFixSeamPlanType     & eMoveSeamResult_U,        // out: orof SM_MS_NO_MOVE, SM_MS_OUT_OF_FACE_U, SM_MS_OUT_OF_FACE_V   
                                                      //                          SM_MS_IN_FACE_U,     SM_MS_IN_FACE_V        
    SmFixSeamPlanType     & eMoveSeamResult_V,        // out: orof SM_MS_NO_MOVE, SM_MS_OUT_OF_FACE_U, SM_MS_OUT_OF_FACE_V   
                                                      //                          SM_MS_IN_FACE_U,     SM_MS_IN_FACE_V        
    SmCurveClassification & rCrvClassU,               // in : SeamU/Face->Edges XSects, const U isoParameterCurve from HasSeamProblem() call 
    SmCurveClassification & rCrvClassV,               // in : SeamV/Face->Edges XSects, const V isoParameterCurve from HasSeamProblem() call  
    SmSurfParamType         eCrossedSeam,             // in : from HasSeamProblem() output(bOuterLoopOnly==FALSE) 
                                                      //      SM_SP_U       = periodic U bndry seam crossed by Face->Edges                 
                                                      //      SM_SP_V       = periodic V bndry seam crossed by Face->Edges                
                                                      //      SM_SP_BOTH    = both periodic U and V bndry seams crossed by Face->Edges     
                                                      //      SM_SP_NEITHER = no seams crossed by Face->Edges                              
    SmSurfParamType         eMissingSeam,             // in : from HasSeamProblem() call output(bOuterLoopOnly==FALSE)   
                                                      //      SM_SP_U       = periodic U bndry seam not represented by Face->Edges          
                                                      //      SM_SP_V       = periodic V bndry seam not represented by Face->Edges            
                                                      //      SM_SP_BOTH    = both periodic U and V bndry seams not represented by Face->Edges 
                                                      //      SM_SP_NEITHER = both periodic U and V bndry seams are represented by Face->Edges 
    SmSurfParamType         eNearMissSeam,            // in : from HasSeamProblem() call output(bOuterLoopOnly==FALSE)   
                                                      //      SM_SP_U       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic U BndrySeam 
                                                      //      SM_SP_V       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic V BndrySeam 
                                                      //      SM_SP_BOTH    = NearMiss for both U and V periodic BndrySeams 
                                                      //      SM_SP_NEITHER = No NearMiss CoincidentEdges or Vertices for BndrySeams 
    SmBoolean               bOkToMoveToCenter=TRUE,   // in : TRUE =okay to move Seam in Face when seam can't move out of Face to avoid sliverFaces 
                                                      //        FALSE=only move Seam out of Face 
    SmBoolean               bOkToSplit=TRUE,          // in : TRUE = split Face at Seam(s) when seam can't move out of Face 
                                                      //      FALSE= don't split Face at Seam(s) 
    SmTArray<SmFace*>     * pOptNewFaces=NULL,        // out: NewFaces made if any, only with bOKToSplit==TRUE.  (does not include this face) 
                                                      //        NULL to ignore, default:[NULL] 
    SmTArray<SmEdge*>     * pOptNewSeamEdges=NULL,    // out: New SeamEdges added if any, only with bOkToSplit==TRUE    
    SmTArray<SmVertex*>   * pOptSeamVertices=NULL,    // out: New Vertices added at the Seam if any, only with bOkToSplit==TRUE     
    SmFaceProps           * pOptFaceProps=NULL        // in : optional FaceProps to save time, NULL to ignore, default:[NULL]  
  );

// When Face->Surface is periodic, add missing Seam segments and split Face->Edges that cross the missing seam.
  SmStatus SplitAtSeam
  (
    SmCurveClassification & rCrvClassU,               // in : const U isoParameterCurve from HasSeamProblem() output(bOuterLoopOnly==FALSE) 
    SmCurveClassification & rCrvClassV,               // in : const V isoParameterCurve from HasSeamProblem() output(bOuterLoopOnly==FALSE) 
    SmSurfParamType         eCrossedSeam,             // in : from HasSeamProblem() output(bOuterLoopOnly==FALSE), not used when pOptCrvClass == NULL 
                                                      //      SM_SP_U       = periodic U bndry seam crossed by Face->Edges                 
                                                      //      SM_SP_V       = periodic V bndry seam crossed by Face->Edges                
                                                      //      SM_SP_BOTH    = both periodic U and V bndry seams crossed by Face->Edges     
                                                      //      SM_SP_NEITHER = no seams crossed by Face->Edges                              
    SmSurfParamType         eMissingSeam,             // in : from HasSeamProblem() call output(bOuterLoopOnly==FALSE), not used when pOptCrvClass == NULL   
                                                      //      SM_SP_U       = periodic U bndry seam not represented by Face->Edges          
                                                      //      SM_SP_V       = periodic V bndry seam not represented by Face->Edges            
                                                      //      SM_SP_BOTH    = both periodic U and V bndry seams not represented by Face->Edges 
                                                      //      SM_SP_NEITHER = both periodic U and V bndry seams are represented by Face->Edges 
    SmSurfParamType         eNearMissSeam,            // in : from HasSeamProblem() call output(bOuterLoopOnly==FALSE), not used when pOptCrvClass == NULL   
                                                      //      SM_SP_U       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic U BndrySeam 
                                                      //      SM_SP_V       = 1 or more CoincidentEdges or Vertices NearMiss (not Exact) XSect the periodic V BndrySeam 
                                                      //      SM_SP_BOTH    = NearMiss for both U and V periodic BndrySeams 
                                                      //      SM_SP_NEITHER = No NearMiss CoincidentEdges or Vertices for BndrySeams 
    SmTArray<SmFace*>     * pOptNewFaces=NULL,        // out: NewFaces made if any when bOKToSplit==TRUE.  (does not include this face) 
                                                      //        NULL to ignore, default:[NULL] 
    SmTArray<SmEdge*>     * pOptNewSeamEdges=NULL,    // out: New SeamEdges added if any when bOkToSplit==TRUE    
    SmTArray<SmVertex*>   * pOptSeamVertices=NULL,    // out: New Vertices added at the Seam if any when bOkToSplit==TRUE     
    SmFaceProps           * pOptFaceProps=NULL        // in : optional pre-cached set of this-Face properites, NULL to ignore
  ) ;


  // utilities: Graphics, pretty print, AssertValid, Memory use, and Dump

  // switch on eMode to add Face graphics to open displayList
  SmStatus OutputGraphics
  (
    const SmDisplayParameters & crDisp,                // in : display control parameters 
    SmBoolean                   bReverseNormals=FALSE, // in : Only used when crDisp.eMode == SM_DM_NLIB_FACETS 
                                                       //      TRUE = reverse surface normals before output. default:[FALSE] 
    const SmExtent2d          * pOptUVDomain=NULL,     // in : UVDomain to crossHatch, NULL=NaturalUVDomain 
    SmGfxArraySet             * pOptGfxSet=NULL        // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                                       //       NULL to ignore. default:[NULL] 
  ) const ;

  // use NLib trimmed Surface tessellator to triangulate face
  SmStatus OutputPolygons
  (
    double                    dSurfaceDistTolerance,    // in : max distance between polygon and surface                     
    double                    dCurveDistTolerance,      // in : max distance between polygon edge and surface                
    SmBoolean                 bReverseNormals,          // in : TRUE = Reverse polygon normals prior to display              
                                                        //      FALSE= don't                                                 
    SmPolygonOutputCallback & rPolygonOutput,           // in : Object used to actually output the triangles                 
    SmGfxArraySet           * pOptGfxSet = NULL,        // in : optional alternate output, NULL to ignore, default:[NULL]    
    SmBSplineSurface        * pOptApproxSurface = NULL  // in : approx surface to tessellate, NULL to ignore, default:[NULL] 
  ) const ;

  // add face graphics to new or open displayList branching on global displayParameters
  virtual SmDisplayList * Draw                  (SmGfxArraySet * pOptGfxSet=NULL) const ;
  SmDisplayList         * DrawLoopusesForFaceuse(SmFaceuse     * pFaceuse=NULL,               // in: TgtFaceuse, NULL=UpwardFaceuse, default:[NULL]
                                                 SmGfxArraySet * pOptGfxSet=NULL) const ;
  SmDisplayList         * DrawNeighbors         (SmGfxArraySet * pOptGfxSet=NULL) const ;
  SmDisplayList         * DrawMicro             (SmGfxArraySet * pOptGfxSet=NULL) const ;

  // add face->UVTrimCurves graphics in 3d and in 2d (on nearby or z=0 plane) to new or open displayList
  virtual SmDisplayList * DrawUVCurves 
  (
    SmBoolean       bDrawNearBy=TRUE,        // in : TRUE=Draw 2d UVCurves on nearby Plane, FALSE=Draw on Z=0 plane, default:[TRUE]
    SmGfxArraySet * pOptGfxSet=NULL          // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. 
  ) const ;

  // draw face with mode and optional xHatch counts
  virtual SmDisplayList * Draw         
  (
    SmDrawModeType lDrawingMode,    // in : any SmDrawModeType; SM_DM_CURRENT = keep the current global display parameters
    ULONG lUHatch = 8,              // in : evenly spaced U IsoParameter line count drawn on Face->Surface, default:[8]  
    ULONG lVHatch = 8,              // in : evenly spaced V IsoParameter line count drawn on Face->Surface, default:[8]  
    SmGfxArraySet * pOptGfxSet=NULL // in : used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls.
                                    //      NULL to ignore. default:[NULL]  
  ) const ;                               
                                          
  // draw surface and crossHatch lines
  virtual SmDisplayList * DrawUV       
  (
    ULONG              lUHatch=8,                    // in : number of U dir crosshatch lines. default:[8]     
    ULONG              lVHatch=8,                    // in : number of V dir crosshatch lines. default:[8]   
    SmBoolean          bVaryCrossHatchColor = FALSE, // in : TRUE = Draw U Lines in ObjectColor, Draw V lines in m_VaryCrossHatchColor 
                                                     //      FALSE = Draw both U and V Lines in ObjectColor default:[FALSE] 
    const SmExtent2d * pOptUVDomain = NULL,          // in : UVDomain to crossHatch, NULL=NaturalUVDomain. default:[NULL] 
    SmBoolean          bAddToUIPickList = FALSE,     // in : TRUE = add to display list so this can be picked, FALSE=don't. default:[FALSE] 
    SmGfxArraySet    * pOptGfxSet = NULL             // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. NULL to ignore 
  ) const ;  
    
  SmDisplayList * DrawSubdivisions
  (
    SmBoolean       bAddToUIPickList        = FALSE,   // NotUsed: in : TRUE = add to display list so this can be picked, FALSE=don't.  
    SmBoolean       bAddTessellation        = TRUE,    // in : TRUE = show Tessellation triangles 
    SmGfxArraySet * pOptGfxSet              = NULL,    // i/o: used for OpenGL ES 2.0. When given, accumulate GfxVertexArrays not GL calls. 
                                                       //      NULL to ignore. default:[NULL] 
    ULONG           lCrvMinSegNumber        = 0,       // in : def:[ 0]     Min number of segments in tess polygon,   0=ignore 
    double          dCrvChordHeight         = 0.0,     // in : def:[ 0]     Max dist3D between geom and tess segment, 0=ignore 
    double          dCrvAngTolDeg           = 8.0,     // in : def:[ 8]     Max angDeg between tess tangents,         0=ignore 
    double          dCrvMaxDist3DBetweenPts = 0.0,     // in : def:[ 0]     Max dist3D between tess pts,              0=ignore 
    double          dCrvMinParamRatio       = 0.00001, // in : def:[.00001] Min SegParamLength/DomainLength ratio,    0=ignore 
    double          dSrfChordHeight         = 0.0,     // in : def:[ 0]   Max dist3D between geom and tess segment,    0=ignore  
    double          dSrfAngTolDeg           = 25.0,    // in : def:[25]   Max angDeg between tess tangents,            0=ignore  
    double          dSrfMaxEdgeLength3D     = 0.0,     // in : def:[ 0]   Max 3D polygon edge length,                  0=ignore  
    double          dSrfMinEdgeLength3D     = 0.0,     // in : def:[ 0]   Min 3D polygon edge length,                  0=ignore  
    double          dSrfMinEdgeLengthRatioUV= 0.001,   // in : def:[.001] Min polygon edge u or v domain size fraction,0=ignore 
    double          dSrfMaxAspectRatio      = 0        // in : def:[ 0]   Max subdivision rectangular node size ratio, 0=ignore 
                                                       //                   AspectRatio = SizeOfLargest/sizeOfSmallest.                                                                                     
  ) const ;  
  
  void WriteAsTrimmedSurface(const TCHAR * cOutputFileName) const;

  // get memory used for face, its surface, all its faceuses, loops, and loopuses and their attributes
  virtual ULONG GetMemoryUsed
  (
    ULONG    & rlMemoryAllocated,      // out: bigger size of all allocated memory in bytes
    SmMarkType eMarkType=SM_MT_NOMARK // in : uses without increment eMarkType value
  ) const ;

  // private: The SmFace's part of the global Notify mechanism - used to clean up caches after shape changes
  virtual void      Notify      
  (
    SmNotifyOperation  eNotifyOperation, // in :       event                | caller      |  pData1  | pData2                | pData3                    
    SmObject         * pData1,           // in : ---------------------------+-------------+----------+-----------------------+--------------------------
    SmObject         * pData2,           // in : SM_NO_ADD_TO_BREP          | Brep        | AddObj   | Brep                  | AddObj's GeomPtr or NULL  
    SmObject         * pData3            // in : SM_NO_SPLIT_IN_BREP        | Brep/TopoObj| OrigObj  | Child1                | Child2                    
                                         // in : SM_NO_MERGE_IN_BREP        | Brep/TopoObj| SurvObj  | DelObj                | Brep                      
                                         // in : SM_NO_TRIM_NO_SPLIT_IN_BREP| Brep        | TgtObj   | AddedBndryObj         | NULL                     
                                         // in : SM_NO_COINCIDENT           | BrepA       | BrepAObj | BrepBObj              | BrepB                     
                                         // in : SM_NO_RM_FROM_BREP         | Brep        | RmObj    | Brep                  | RmObj's GeomPtr or NULL   
                                         // in : SM_NO_CHANGE_GEOMETRY      | TopoObj     | NewGeom  | Brep or NULL          | OldGeom or NULL           
                                         // in : SM_NO_CHANGE_OWNER         | GeomObj     | NewOwner | NewOwner Brep or NULL | OldOwner or NULL          
                                         // in : SM_NO_CONSTRUCTION         | NewObj      |  NewObj  | CopyFromObj or NULL   | NULL                      
                                         // in : SM_NO_COPY                 | FromObj     | ToObj    | ToObj's Owner or NULL | FromObj's Owner or NULL   
                                         // in : SM_NO_PRE_EDIT             | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
                                         // in : SM_NO_POST_EDIT            | EditObj     | EditObj  | EditObj Owner or NULL | NULL                      
                                         // in : SM_NO_SPLIT                | SplitGeomObj| Child1   | Child2                | SplitObj's Owner or NULL  
                                         // in : SM_NO_MERGE                | MergeGeomObj| OrigObj1 | OrigObj2              | MergeObj's Owner or NULL  
                                         // in : SM_NO_REG_PROPAGATION      | MergeReg    | ThisRegs | OtherBrep->SrcRegs    | ThisBrep->MergeReg        
                                         // in : SM_NO_DESTRUCTION          | DelObj      | DelObj   |  NULL                 |  NULL                             
  ) ;
                                     
  virtual SmBoolean AssertValid
  (
    SmAssertArray    * pAList=NULL,           // i/o: Accumulating list of failed Asserts, NULL to ignore 
    SmAssertTestLevel  eTestLevel=SM_LEVEL_0, // in : SM_LEVEL_0=fewest tests, SM_LEVEL_1, SM_LEVEL_2=all tests,  
                                              //      SM_LEVEL_GIVEN = run tests in order requested in pTestRequests  
                                              //      default:[SM_LEVEL_0]  
    SmAssertWalking    eWalkTree=SM_WALK,     // in : SM_WALK = run AssertValid on any topology graph descendants, SM_NO_WALK=don't, default:[SM_WALK]  
    SmTArray<ULONG>  * pTestRequests=NULL     // in : when eTestLevel == SM_LEVEL_GIVEN, run these tests in this order, default:[NULL] 
  ) const ;

 // obsolete
 //  virtual SmBoolean AssertHeal (SmAssertReport & rAReport, SmAssertArray * pAList) ;

  // Define  GetType(), IsKindOf(), and virtual Dump() (Dump needs local implementation)
  SM_COMMON(SmFace,SmTopology,SmFace_TYPE);

  void Dump        (ULONG)                  const ;
  void Dump        (TCHAR   * message)      const ;
  void Dump        (SmBoolean bAbbrev)      const ;  // in : FALSE=dump loops and UVTrimCurves and Dump Edges and Surface(bAbbrev=FALSE)
                                                     //      TRUE =don't dump loops and UVTrimCurves and Dump Edges and Surface(bAbbrev=TRUE), 
                                                     //      default:[FALSE]

  void DumpTopology(ULONG     lWalkDepth=0) const ;  // lWalkDepth[0] = no walk, [99] = walk to bottom

} ; // end class SmFace

// GWC:BIND_TEMPLATE_MOVE     SM_TARRAY_TEMPLATE_PREDECLARATION(SmFace*) ;

#endif // !__SMFACE_H__
